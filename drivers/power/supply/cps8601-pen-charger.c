// SPDX-License-Identifier: GPL-2.0-only
/*
 * ConvenientPower CPS8601 wireless power transmitter, as the stylus charger
 * of the OnePlus Pad 3.
 *
 * The CPS8601 runs a OnePlus firmware: once released it pings on its own,
 * powers a stylus that answers and forwards the stylus' packets (identity,
 * state of charge). The host keeps the coil supply at an idle voltage, raises
 * it while a stylus is attached, gates the transmitter with a GPIO and stops
 * it when the stylus reports full or the transmitter reports a fault.
 *
 * The stylus also sends its Bluetooth address over the coil; it is exposed for
 * userspace to pair the stylus. The stylus reports its state of charge over
 * the coil only when it stops charging, so a full stylus is charged again
 * when userspace, which reads the stylus battery over Bluetooth, reports that
 * it went below the start threshold.
 *
 * The register map and the sequences follow the OnePlus driver.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define CPS8601_CHIP_ID			0x8601

/* 16-bit address, little-endian multi-byte fields */
#define CPS8601_REG_CHIP_ID		0x0000
#define CPS8601_REG_FW_VER		0x0002
#define CPS8601_REG_INT_EN		0x0005
#define CPS8601_REG_INT_FLAG		0x0007
#define CPS8601_REG_INT_CLR		0x0009
#define CPS8601_REG_CMD			0x000b
#define CPS8601_REG_OCP_TH		0x0028
#define CPS8601_REG_OVP_TH		0x002a
#define CPS8601_REG_UVP_TH		0x002c
#define CPS8601_REG_FOD_TH		0x002f
#define CPS8601_REG_EPT_RSN		0x003e
#define CPS8601_REG_RX_PKT		0x0040

#define CPS8601_RX_PKT_LEN		11

#define CPS8601_CMD_RESET_SYS		BIT(5)

#define CPS8601_INT_WAKEUP		BIT(0)
#define CPS8601_INT_SSP			BIT(2)
#define CPS8601_INT_ASK_PKT		BIT(5)
#define CPS8601_INT_EPT			BIT(6)
#define CPS8601_INT_RX_ATTACH		BIT(9)
#define CPS8601_INT_RX_REMOVED		BIT(10)
#define CPS8601_INT_MIS_LOC		BIT(13)

#define CPS8601_EPT_OCP			BIT(6)
#define CPS8601_EPT_OVP			BIT(7)
#define CPS8601_EPT_UVP			BIT(8)
#define CPS8601_EPT_FOD			BIT(9)
#define CPS8601_EPT_OTP			BIT(10)
#define CPS8601_EPT_POCP		BIT(11)
#define CPS8601_EPT_FAULT		(CPS8601_EPT_OCP | CPS8601_EPT_OVP | \
					 CPS8601_EPT_FOD | CPS8601_EPT_OTP | \
					 CPS8601_EPT_POCP)

/* stylus packets forwarded by the firmware */
#define PKT_MAC_HEAD			0x48
#define PKT_MAC_CHECK			0xc1
#define PKT_MAC_ADDR1			0xb6
#define PKT_MAC_ADDR2			0xb7
#define PKT_POWER_HEAD			0x28
#define PKT_POWER_SOC			0x17

/* stock thresholds (mA, mV, firmware units) */
#define CPS8601_OCP_MA			500
#define CPS8601_OVP_MV			12000
#define CPS8601_UVP_MV			4000
#define CPS8601_FOD			400
#define CPS8601_START_THRESHOLD		90	/* % to charge a full stylus again */

#define CPS8601_WAKE_MS			2500
/*
 * A stylus that was already docked when the chip came up has been powered by
 * the firmware run of the bring-up. The stock driver's 50 ms power cycle is
 * too short for it to reset: it then does not answer the next ping and the
 * transmitter ends with a foreign object fault. 500 ms was measured to be
 * enough on the Pad 3; take twice that.
 */
#define CPS8601_RESTART_OFF_MS		1000
#define CPS8601_IDENT_TIMEOUT_MS	5000
#define CPS8601_CHARGE_TIMEOUT_MIN	120
#define CPS8601_BOOST_RETRIES		60
#define CPS8601_BOOST_RETRY_MS		500

enum cps8601_stop {
	CPS8601_STOP_NONE,
	CPS8601_STOP_FULL,
	CPS8601_STOP_PEN,
	CPS8601_STOP_TIMEOUT,
	CPS8601_STOP_EPT,
	CPS8601_STOP_FAULT,
	CPS8601_STOP_IDENT,
};

struct cps8601 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator *boost;
	struct power_supply *psy;

	struct gpio_desc *wake_gpio;	/* GP0, high = awake */
	struct gpio_desc *scan_gpio;	/* GP1 */
	struct gpio_desc *enable_gpio;	/* GP2, chip power switch */
	struct gpio_desc *txoff_gpio;	/* GP3, high = transmitter off */

	u32 idle_uv;
	u32 charge_uv;

	struct work_struct boost_work;
	struct delayed_work ident_work;
	struct delayed_work timeout_work;

	/* protects the state below */
	struct mutex lock;
	bool ready;
	bool attached;
	bool present;
	bool tx_allowed;
	enum cps8601_stop stop;
	int capacity;
	int start_threshold;
	bool have_check;
	u64 mac_check;
	u64 mac;
	u64 reported_mac;	/* address announced to userspace, 0 if none */
};


static const struct regmap_config cps8601_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
};

static int cps8601_read16(struct cps8601 *cps, unsigned int reg, u16 *val)
{
	u8 buf[2];
	int ret;

	ret = regmap_bulk_read(cps->regmap, reg, buf, sizeof(buf));
	if (ret)
		return ret;

	*val = buf[0] | buf[1] << 8;

	return 0;
}

/* One byte per transfer, as the stock driver writes. */
static int cps8601_write16(struct cps8601 *cps, unsigned int reg, u16 val)
{
	int ret;

	ret = regmap_write(cps->regmap, reg, val & 0xff);
	if (ret)
		return ret;

	return regmap_write(cps->regmap, reg + 1, val >> 8);
}

static int cps8601_set_boost(struct cps8601 *cps, u32 uv)
{
	return regulator_set_voltage(cps->boost, uv, uv);
}

/* Tell userspace that an identified stylus came or went, with its address. */
static void cps8601_send_event(struct cps8601 *cps, bool present, u64 mac)
{
	char addr[sizeof("ACCESSORY_BT_ADDRESS=00:00:00:00:00:00")];
	char *envp[] = {
		"ACCESSORY_TYPE=stylus",
		present ? "ACCESSORY_PRESENT=1" : "ACCESSORY_PRESENT=0",
		addr,
		NULL
	};
	u8 a[6];

	put_unaligned_be48(mac, a);
	snprintf(addr, sizeof(addr), "ACCESSORY_BT_ADDRESS=%pM", a);
	kobject_uevent_env(&cps->psy->dev.kobj, KOBJ_CHANGE, envp);
}

/* Called with the lock held. */
static void cps8601_set_tx(struct cps8601 *cps, bool allow)
{
	gpiod_set_value_cansleep(cps->txoff_gpio, !allow);
	cps->tx_allowed = allow;
}

static void cps8601_stop_locked(struct cps8601 *cps, enum cps8601_stop why)
{
	cps8601_set_tx(cps, false);
	cps->stop = why;
}

static void cps8601_boost_work(struct work_struct *work)
{
	struct cps8601 *cps = container_of(work, struct cps8601, boost_work);
	int retries = CPS8601_BOOST_RETRIES;
	u32 uv;
	int ret;

	do {
		scoped_guard(mutex, &cps->lock)
			uv = cps->attached ? cps->charge_uv : cps->idle_uv;

		ret = cps8601_set_boost(cps, uv);
		if (!ret)
			return;

		msleep(CPS8601_BOOST_RETRY_MS);
	} while (--retries);

	dev_err(cps->dev, "failed to set the coil supply to %u uV: %d\n", uv, ret);
}

/* A stylus that does not identify itself within 5 s is not charged. */
static void cps8601_ident_work(struct work_struct *work)
{
	struct cps8601 *cps = container_of(to_delayed_work(work), struct cps8601,
					   ident_work);

	scoped_guard(mutex, &cps->lock) {
		if (!cps->attached || cps->present || !cps->tx_allowed)
			return;

		dev_info(cps->dev, "receiver did not identify itself, stopping\n");
		cps8601_stop_locked(cps, CPS8601_STOP_IDENT);
	}

	power_supply_changed(cps->psy);
}

static void cps8601_timeout_work(struct work_struct *work)
{
	struct cps8601 *cps = container_of(to_delayed_work(work), struct cps8601,
					   timeout_work);

	scoped_guard(mutex, &cps->lock) {
		if (!cps->attached || !cps->tx_allowed)
			return;

		dev_info(cps->dev, "charge time limit reached, stopping\n");
		cps8601_stop_locked(cps, CPS8601_STOP_TIMEOUT);
	}

	power_supply_changed(cps->psy);
}

/* The stylus sends its address with each nibble swapped across two halves. */
static u64 cps8601_decode_mac(const u8 *d)
{
	u8 a[6];

	a[5] = (d[9] & 0x0f) << 4 | (d[4] & 0xf0) >> 4;
	a[4] = (d[8] & 0x0f) << 4 | (d[3] & 0xf0) >> 4;
	a[3] = (d[7] & 0x0f) << 4 | (d[2] & 0xf0) >> 4;
	a[2] = (d[4] & 0x0f) << 4 | (d[9] & 0xf0) >> 4;
	a[1] = (d[3] & 0x0f) << 4 | (d[8] & 0xf0) >> 4;
	a[0] = (d[2] & 0x0f) << 4 | (d[7] & 0xf0) >> 4;

	return (u64)a[5] << 40 | (u64)a[4] << 32 | (u64)a[3] << 24 |
	       (u64)a[2] << 16 | (u64)a[1] << 8 | a[0];
}

static bool cps8601_mac_valid(u64 mac, u64 check)
{
	return (((mac >> 40) ^ (mac >> 16)) & 0xff) == ((check >> 16) & 0xff) &&
	       (((mac >> 32) ^ (mac >> 8)) & 0xff) == ((check >> 8) & 0xff) &&
	       (((mac >> 24) ^ mac) & 0xff) == (check & 0xff);
}

/* Called with the lock held. */
static int cps8601_handle_packet(struct cps8601 *cps, u64 *gone, u64 *came)
{
	u8 d[CPS8601_RX_PKT_LEN];
	int ret;

	ret = regmap_bulk_read(cps->regmap, CPS8601_REG_RX_PKT, d, sizeof(d));
	if (ret)
		return ret;

	dev_dbg(cps->dev, "packet %*ph\n", (int)sizeof(d), d);

	if (d[0] == PKT_MAC_HEAD && d[1] == PKT_MAC_CHECK) {
		cps->mac_check = d[4] << 16 | d[3] << 8 | d[2];
		cps->have_check = true;
	}

	if (d[0] == PKT_MAC_HEAD && d[1] == PKT_MAC_ADDR1 &&
	    d[5] == PKT_MAC_HEAD && d[6] == PKT_MAC_ADDR2) {
		/*
		 * The chip keeps only the last packet. A stylus that was docked
		 * before the interrupt was set up can have its check packet
		 * overwritten by the address; wait for the next exchange (the
		 * firmware restart at the end of probe starts one) instead of
		 * refusing the stylus.
		 */
		if (!cps->have_check) {
			dev_dbg(cps->dev, "address before the check packet, ignored\n");
			return 0;
		}

		cps->mac = cps8601_decode_mac(d);
		if (cps8601_mac_valid(cps->mac, cps->mac_check)) {
			cps->present = true;
			dev_dbg(cps->dev, "stylus %012llx\n", cps->mac);
			if (cps->reported_mac != cps->mac) {
				if (cps->reported_mac) {
					*gone = cps->reported_mac;
					cps->capacity = -ENODATA;
				}
				*came = cps->mac;
				cps->reported_mac = cps->mac;
			}
		} else {
			dev_info(cps->dev, "stylus identity check failed, stopping\n");
			cps8601_stop_locked(cps, CPS8601_STOP_IDENT);
		}
	}

	if (d[0] == PKT_POWER_HEAD && d[1] == PKT_POWER_SOC) {
		cps->capacity = min_t(int, d[2], 100);
		cps8601_stop_locked(cps, cps->capacity == 100 ? CPS8601_STOP_FULL :
							      CPS8601_STOP_PEN);
		dev_dbg(cps->dev, "stylus at %d%%, charging stopped\n", cps->capacity);
	}

	return 0;
}

/* Called with the lock held. */
static void cps8601_handle_ept(struct cps8601 *cps)
{
	u16 ept;

	if (cps8601_read16(cps, CPS8601_REG_EPT_RSN, &ept))
		return;

	dev_info(cps->dev, "end of power transfer, reason %#06x\n", ept);

	if (ept & CPS8601_EPT_FAULT) {
		gpiod_set_value_cansleep(cps->scan_gpio, 1);
		cps8601_stop_locked(cps, CPS8601_STOP_FAULT);
	} else if (ept & CPS8601_EPT_UVP) {
		schedule_work(&cps->boost_work);
		gpiod_set_value_cansleep(cps->scan_gpio, 1);
		cps8601_stop_locked(cps, CPS8601_STOP_FAULT);
	} else {
		cps->stop = CPS8601_STOP_EPT;
	}
}

/* Called with the lock held. */
static void cps8601_handle_removed(struct cps8601 *cps, u64 *gone)
{
	bool fault = cps->stop == CPS8601_STOP_FAULT || cps->stop == CPS8601_STOP_EPT;

	*gone = cps->reported_mac;
	cps->reported_mac = 0;

	cps->attached = false;
	cps->capacity = -ENODATA;
	cps->present = false;
	cps->mac = 0;
	cps->mac_check = 0;
	cps->have_check = false;
	cps->stop = CPS8601_STOP_NONE;

	/* the stock driver restarts the firmware after a fault */
	if (fault)
		regmap_update_bits(cps->regmap, CPS8601_REG_CMD,
				   CPS8601_CMD_RESET_SYS, CPS8601_CMD_RESET_SYS);

	gpiod_set_value_cansleep(cps->scan_gpio, 1);
	cps8601_set_tx(cps, true);
}

static irqreturn_t cps8601_irq(int irq, void *data)
{
	struct cps8601 *cps = data;
	bool changed = false;
	u64 gone = 0, came = 0;
	u16 flags;
	int ret;

	scoped_guard(mutex, &cps->lock) {
		/* the firmware forgets its mask, as the stock driver sets it every time */
		cps8601_write16(cps, CPS8601_REG_INT_EN, 0xffff);

		ret = cps8601_read16(cps, CPS8601_REG_INT_FLAG, &flags);
		if (ret) {
			dev_err_ratelimited(cps->dev, "failed to read interrupt flags: %d\n", ret);
			return IRQ_HANDLED;
		}

		cps8601_write16(cps, CPS8601_REG_INT_CLR, flags);
		dev_dbg(cps->dev, "interrupt %#06x\n", flags);

		/* events of the firmware run before the restart in probe are stale */
		if (!cps->ready)
			return IRQ_HANDLED;

		if (flags & CPS8601_INT_SSP)
			cps->present = true;

		if (flags & CPS8601_INT_RX_ATTACH) {
			/*
			 * Not the capacity: turning the transmitter back on to
			 * recharge a docked stylus attaches it again, and its
			 * last known charge stays valid until it is removed.
			 */
			cps->attached = true;
			cps->have_check = false;
			schedule_work(&cps->boost_work);
			mod_delayed_work(system_dfl_wq, &cps->ident_work,
					 msecs_to_jiffies(CPS8601_IDENT_TIMEOUT_MS));
			mod_delayed_work(system_dfl_wq, &cps->timeout_work,
					 secs_to_jiffies(CPS8601_CHARGE_TIMEOUT_MIN * 60));
			changed = true;
		}

		if (flags & CPS8601_INT_RX_REMOVED) {
			cps8601_handle_removed(cps, &gone);
			schedule_work(&cps->boost_work);
			cancel_delayed_work(&cps->ident_work);
			cancel_delayed_work(&cps->timeout_work);
			changed = true;
		}

		if (flags & CPS8601_INT_ASK_PKT) {
			cps8601_handle_packet(cps, &gone, &came);
			changed = true;
		}

		if (flags & CPS8601_INT_EPT) {
			cps8601_handle_ept(cps);
			changed = true;
		}

		if (flags & CPS8601_INT_MIS_LOC)
			dev_dbg(cps->dev, "stylus misplaced\n");
	}

	if (changed)
		power_supply_changed(cps->psy);
	if (gone)
		cps8601_send_event(cps, false, gone);
	if (came)
		cps8601_send_event(cps, true, came);

	return IRQ_HANDLED;
}

static int cps8601_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct cps8601 *cps = power_supply_get_drvdata(psy);

	guard(mutex)(&cps->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = cps->attached && cps->present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = cps->attached && cps->tx_allowed;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!cps->attached)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (cps->stop == CPS8601_STOP_FULL)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (cps->tx_allowed && cps->present)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!cps->attached || cps->capacity < 0)
			return -ENODATA;
		val->intval = cps->capacity;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = cps->start_threshold;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * The stock policy: a stylus that reported full is charged again once its
 * state of charge, as userspace read it, is at or below the threshold.
 * Called with the lock held; returns whether charging restarted.
 */
static bool cps8601_recharge_locked(struct cps8601 *cps)
{
	if (!cps->attached || !cps->present || cps->tx_allowed ||
	    cps->stop != CPS8601_STOP_FULL || cps->capacity < 0 ||
	    cps->capacity > cps->start_threshold)
		return false;

	dev_dbg(cps->dev, "stylus at %d%%, charging again\n", cps->capacity);
	cps->stop = CPS8601_STOP_NONE;
	cps8601_set_tx(cps, true);
	mod_delayed_work(system_dfl_wq, &cps->timeout_work,
			 secs_to_jiffies(CPS8601_CHARGE_TIMEOUT_MIN * 60));

	return true;
}

static int cps8601_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct cps8601 *cps = power_supply_get_drvdata(psy);
	bool changed;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		if (val->intval < 0 || val->intval > 100)
			return -EINVAL;
		scoped_guard(mutex, &cps->lock) {
			cps->start_threshold = val->intval;
			changed = cps8601_recharge_locked(cps);
		}
		break;
	default:
		return -EINVAL;
	}

	if (changed)
		power_supply_changed(psy);

	return 0;
}

static int cps8601_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD;
}

static ssize_t bt_address_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct cps8601 *cps = power_supply_get_drvdata(to_power_supply(dev));
	u8 a[6];

	scoped_guard(mutex, &cps->lock) {
		if (!cps->reported_mac)
			return -ENODATA;
		put_unaligned_be48(cps->reported_mac, a);
	}

	return sysfs_emit(buf, "%pM\n", a);
}
static DEVICE_ATTR_RO(bt_address);

static ssize_t bt_capacity_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct cps8601 *cps = power_supply_get_drvdata(to_power_supply(dev));

	guard(mutex)(&cps->lock);

	if (!cps->reported_mac || cps->capacity < 0)
		return -ENODATA;

	return sysfs_emit(buf, "%d\n", cps->capacity);
}

static ssize_t bt_capacity_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct power_supply *psy = to_power_supply(dev);
	struct cps8601 *cps = power_supply_get_drvdata(psy);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val > 100)
		return -EINVAL;

	scoped_guard(mutex, &cps->lock) {
		/* only for the stylus that is docked now */
		if (!cps->attached || !cps->reported_mac)
			return -ENODEV;
		cps->capacity = val;
		cps8601_recharge_locked(cps);
	}

	power_supply_changed(psy);

	return count;
}
static DEVICE_ATTR_RW(bt_capacity);

static struct attribute *cps8601_attrs[] = {
	&dev_attr_bt_address.attr,
	&dev_attr_bt_capacity.attr,
	NULL
};
ATTRIBUTE_GROUPS(cps8601);

static const enum power_supply_property cps8601_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
};

static const struct power_supply_desc cps8601_psy_desc = {
	.name = "cps8601-stylus",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = cps8601_properties,
	.num_properties = ARRAY_SIZE(cps8601_properties),
	.get_property = cps8601_get_property,
	.set_property = cps8601_set_property,
	.property_is_writeable = cps8601_property_is_writeable,
};

static void cps8601_power_off(void *data)
{
	struct cps8601 *cps = data;

	gpiod_set_value_cansleep(cps->txoff_gpio, 1);
	gpiod_set_value_cansleep(cps->wake_gpio, 0);
	gpiod_set_value_cansleep(cps->enable_gpio, 0);
	cancel_work_sync(&cps->boost_work);
	cancel_delayed_work_sync(&cps->ident_work);
	cancel_delayed_work_sync(&cps->timeout_work);
}

/* The stock bring-up: supply, power switch, wake, identify, thresholds. */
static int cps8601_hw_init(struct cps8601 *cps)
{
	int retries = CPS8601_BOOST_RETRIES;
	u16 id = 0, ver, th[4];
	int ret, i;

	while ((ret = cps8601_set_boost(cps, cps->idle_uv))) {
		if (!--retries)
			return dev_err_probe(cps->dev, ret, "failed to set the coil supply\n");
		msleep(CPS8601_BOOST_RETRY_MS);
	}

	gpiod_set_value_cansleep(cps->enable_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(cps->wake_gpio, 1);
	msleep(CPS8601_WAKE_MS);

	for (i = 0; i < 5; i++) {
		ret = cps8601_read16(cps, CPS8601_REG_CHIP_ID, &id);
		if (!ret && id == CPS8601_CHIP_ID)
			break;
		msleep(100);
	}
	if (id != CPS8601_CHIP_ID)
		return dev_err_probe(cps->dev, ret ?: -ENODEV,
				     "unexpected chip id %#06x\n", id);

	ret = cps8601_read16(cps, CPS8601_REG_FW_VER, &ver);
	if (ret)
		return ret;
	dev_info(cps->dev, "CPS8601 firmware %#06x\n", ver);

	ret = cps8601_write16(cps, CPS8601_REG_OCP_TH, CPS8601_OCP_MA);
	if (!ret)
		ret = cps8601_write16(cps, CPS8601_REG_UVP_TH, CPS8601_UVP_MV);
	if (!ret)
		ret = cps8601_write16(cps, CPS8601_REG_OVP_TH, CPS8601_OVP_MV);
	if (!ret)
		ret = cps8601_write16(cps, CPS8601_REG_FOD_TH, CPS8601_FOD);
	if (ret)
		return dev_err_probe(cps->dev, ret, "failed to set the thresholds\n");

	/* read back what the firmware took */
	if (!cps8601_read16(cps, CPS8601_REG_OCP_TH, &th[0]) &&
	    !cps8601_read16(cps, CPS8601_REG_OVP_TH, &th[1]) &&
	    !cps8601_read16(cps, CPS8601_REG_UVP_TH, &th[2]) &&
	    !cps8601_read16(cps, CPS8601_REG_FOD_TH, &th[3]))
		dev_dbg(cps->dev, "OCP %u mA, OVP %u mV, UVP %u mV, FOD %u\n",
			th[0], th[1], th[2], th[3]);

	gpiod_set_value_cansleep(cps->scan_gpio, 1);

	return 0;
}

static int cps8601_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	struct cps8601 *cps;
	int ret;

	cps = devm_kzalloc(dev, sizeof(*cps), GFP_KERNEL);
	if (!cps)
		return -ENOMEM;

	cps->dev = dev;
	cps->capacity = -ENODATA;
	cps->start_threshold = CPS8601_START_THRESHOLD;
	ret = devm_mutex_init(dev, &cps->lock);
	if (ret)
		return ret;
	INIT_WORK(&cps->boost_work, cps8601_boost_work);
	INIT_DELAYED_WORK(&cps->ident_work, cps8601_ident_work);
	INIT_DELAYED_WORK(&cps->timeout_work, cps8601_timeout_work);

	cps->regmap = devm_regmap_init_i2c(client, &cps8601_regmap_config);
	if (IS_ERR(cps->regmap))
		return PTR_ERR(cps->regmap);

	ret = device_property_read_u32(dev, "oneplus,idle-microvolt", &cps->idle_uv);
	if (ret)
		return dev_err_probe(dev, ret, "missing oneplus,idle-microvolt\n");
	ret = device_property_read_u32(dev, "oneplus,charge-microvolt", &cps->charge_uv);
	if (ret)
		return dev_err_probe(dev, ret, "missing oneplus,charge-microvolt\n");

	cps->boost = devm_regulator_get(dev, "vin");
	if (IS_ERR(cps->boost))
		return dev_err_probe(dev, PTR_ERR(cps->boost), "failed to get vin\n");

	/* transmitter off until the interrupt is in place */
	cps->txoff_gpio = devm_gpiod_get(dev, "tx-disable", GPIOD_OUT_HIGH);
	if (IS_ERR(cps->txoff_gpio))
		return PTR_ERR(cps->txoff_gpio);
	cps->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(cps->enable_gpio))
		return PTR_ERR(cps->enable_gpio);
	cps->wake_gpio = devm_gpiod_get(dev, "wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(cps->wake_gpio))
		return PTR_ERR(cps->wake_gpio);
	cps->scan_gpio = devm_gpiod_get(dev, "scan", GPIOD_OUT_LOW);
	if (IS_ERR(cps->scan_gpio))
		return PTR_ERR(cps->scan_gpio);

	ret = devm_add_action_or_reset(dev, cps8601_power_off, cps);
	if (ret)
		return ret;

	ret = cps8601_hw_init(cps);
	if (ret)
		return ret;

	psy_cfg.drv_data = cps;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.attr_grp = cps8601_groups;
	cps->psy = devm_power_supply_register(dev, &cps8601_psy_desc, &psy_cfg);
	if (IS_ERR(cps->psy))
		return dev_err_probe(dev, PTR_ERR(cps->psy), "failed to register power supply\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, cps8601_irq,
					IRQF_ONESHOT, "cps8601", cps);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	device_init_wakeup(dev, true);

	scoped_guard(mutex, &cps->lock)
		cps8601_set_tx(cps, true);

	/* let the firmware sleep, then restart it with the transmitter enabled */
	gpiod_set_value_cansleep(cps->wake_gpio, 0);
	gpiod_set_value_cansleep(cps->enable_gpio, 0);
	msleep(CPS8601_RESTART_OFF_MS);
	scoped_guard(mutex, &cps->lock)
		cps->ready = true;
	gpiod_set_value_cansleep(cps->enable_gpio, 1);
	msleep(50);

	return 0;
}

/* The stock driver does not touch the bus while the system sleeps. */
static int cps8601_suspend(struct device *dev)
{
	if (device_may_wakeup(dev))
		enable_irq_wake(to_i2c_client(dev)->irq);
	disable_irq(to_i2c_client(dev)->irq);

	return 0;
}

static int cps8601_resume(struct device *dev)
{
	if (device_may_wakeup(dev))
		disable_irq_wake(to_i2c_client(dev)->irq);
	enable_irq(to_i2c_client(dev)->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cps8601_pm_ops, cps8601_suspend, cps8601_resume);

static const struct of_device_id cps8601_of_match[] = {
	{ .compatible = "convenientpower,cps8601" },
	{}
};
MODULE_DEVICE_TABLE(of, cps8601_of_match);

static struct i2c_driver cps8601_driver = {
	.driver = {
		.name = "cps8601-pen-charger",
		.of_match_table = cps8601_of_match,
		.pm = pm_sleep_ptr(&cps8601_pm_ops),
	},
	.probe = cps8601_probe,
};
module_i2c_driver(cps8601_driver);

MODULE_DESCRIPTION("CPS8601 stylus wireless charger");
MODULE_LICENSE("GPL");
