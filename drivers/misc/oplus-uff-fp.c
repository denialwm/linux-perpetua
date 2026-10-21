// SPDX-License-Identifier: GPL-2.0-only
/* Oplus UFF sensor platform control. SPI transfers are owned by QTEE.
 * GPIOs, supply and timings come from Aston's factory tree and UFF driver.
 */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uff_fp.h>

struct uff_fp {
	struct miscdevice misc;
	struct mutex lock;
	struct kref ref;
	struct gpio_desc *reset, *irq_gpio;
	struct regulator *vdd;
	wait_queue_head_t wait;
	atomic_t sequence;
	u32 seen;
	int irq;
	bool opened, powered, standby, removed;
	u32 reset_sequence;
};

static void uff_fp_free(struct kref *ref)
{
	kfree(container_of(ref, struct uff_fp, ref));
}

static int uff_fp_power_off(struct uff_fp *fp)
{
	int ret;

	if (!fp->powered)
		return 0;
	gpiod_set_value_cansleep(fp->reset, 1);
	ret = regulator_disable(fp->vdd);
	if (ret)
		return ret;
	if (!fp->standby)
		disable_irq(fp->irq);
	fp->standby = false;
	fp->powered = false;
	return 0;
}

static int uff_fp_power_on(struct uff_fp *fp)
{
	int ret;

	if (fp->powered)
		return 0;
	/* A worker cancelled from retained sleep leaves a low load vote.
	 * Every fresh initialization must restore the active vote first. */
	ret = regulator_set_load(fp->vdd, 150000);
	if (ret < 0)
		return ret;
	ret = regulator_enable(fp->vdd);
	if (ret)
		return ret;
	/* Factory fp_power_on waits 30 ms, then fp_hw_reset holds low 5 ms. */
	msleep(30);
	gpiod_set_value_cansleep(fp->reset, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(fp->reset, 0);
	msleep(10);
	fp->reset_sequence++;
	fp->powered = true;
	enable_irq(fp->irq);
	return 0;
}

static irqreturn_t uff_fp_irq(int irq, void *data)
{
	struct uff_fp *fp = data;

	atomic_inc(&fp->sequence);
	wake_up_interruptible(&fp->wait);
	return IRQ_HANDLED;
}

static int uff_fp_open(struct inode *inode, struct file *file)
{
	struct uff_fp *fp = container_of(file->private_data, struct uff_fp, misc);
	int ret = 0;

	mutex_lock(&fp->lock);
	if (fp->removed)
		ret = -ENODEV;
	else if (fp->opened)
		ret = -EBUSY;
	else {
		fp->opened = true;
		fp->seen = atomic_read(&fp->sequence);
		kref_get(&fp->ref);
		file->private_data = fp;
	}
	mutex_unlock(&fp->lock);
	return ret ? ret : nonseekable_open(inode, file);
}

static int uff_fp_release(struct inode *inode, struct file *file)
{
	struct uff_fp *fp = file->private_data;
	int ret = 0;

	mutex_lock(&fp->lock);
	if (!fp->removed)
		ret = uff_fp_power_off(fp);
	fp->opened = false;
	mutex_unlock(&fp->lock);
	kref_put(&fp->ref, uff_fp_free);
	return ret;
}

static long uff_fp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct uff_fp *fp = file->private_data;
	__s32 value;
	__u32 standby;
	struct uff_fp_state state;
	int ret = 0;

	mutex_lock(&fp->lock);
	if (fp->removed) {
		ret = -ENODEV;
		goto out;
	}
	switch (cmd) {
	case UFF_FP_IOC_GET_STATE:
		state = (struct uff_fp_state) {
			.version = 1, .powered = fp->powered,
			.standby = fp->standby, .reset_sequence = fp->reset_sequence,
		};
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			ret = -EFAULT;
		break;
	case UFF_FP_IOC_SET_STANDBY:
		if (copy_from_user(&standby, (void __user *)arg, sizeof(standby))) {
			ret = -EFAULT;
			break;
		}
		if (standby > 1) {
			ret = -EINVAL;
			break;
		}
		if (!fp->powered) {
			ret = -EHOSTDOWN;
			break;
		}
		if (fp->standby == standby)
			break;
		ret = regulator_set_load(fp->vdd, standby ? 1000 : 150000);
		if (ret < 0)
			break;
		ret = 0;
		if (standby)
			disable_irq(fp->irq);
		else
			enable_irq(fp->irq);
		fp->standby = standby;
		break;
	case UFF_FP_IOC_ENABLE_POWER:
		ret = uff_fp_power_on(fp);
		break;
	case UFF_FP_IOC_DISABLE_POWER:
		ret = uff_fp_power_off(fp);
		break;
	case UFF_FP_IOC_RESET:
		if (fp->standby) {
			ret = -EBUSY;
			break;
		}
		if (!fp->powered) {
			ret = -EHOSTDOWN;
			break;
		}
		gpiod_set_value_cansleep(fp->reset, 1);
		usleep_range(5000, 6000);
		gpiod_set_value_cansleep(fp->reset, 0);
		msleep(10);
		fp->reset_sequence++;
		break;
	case UFF_FP_IOC_RD_IRQ_VALUE:
		value = gpiod_get_value_cansleep(fp->irq_gpio);
		if (value < 0)
			ret = value;
		else if (copy_to_user((void __user *)arg, &value, sizeof(value)))
			ret = -EFAULT;
		break;
	default:
		ret = -ENOTTY;
	}
 out:
	mutex_unlock(&fp->lock);
	return ret;
}

static ssize_t uff_fp_read(struct file *file, char __user *buf, size_t count,
			   loff_t *pos)
{
	struct uff_fp *fp = file->private_data;
	u32 sequence;
	int ret;

	if (count < sizeof(sequence))
		return -EINVAL;
	if (file->f_flags & O_NONBLOCK) {
		if (!READ_ONCE(fp->removed) &&
		    atomic_read(&fp->sequence) == READ_ONCE(fp->seen))
			return -EAGAIN;
	} else {
		ret = wait_event_interruptible(fp->wait,
			READ_ONCE(fp->removed) ||
			atomic_read(&fp->sequence) != READ_ONCE(fp->seen));
		if (ret)
			return ret;
	}
	if (READ_ONCE(fp->removed))
		return -ENODEV;
	sequence = atomic_read(&fp->sequence);
	if (copy_to_user(buf, &sequence, sizeof(sequence)))
		return -EFAULT;
	WRITE_ONCE(fp->seen, sequence);
	return sizeof(sequence);
}

static __poll_t uff_fp_poll(struct file *file, poll_table *wait)
{
	struct uff_fp *fp = file->private_data;

	poll_wait(file, &fp->wait, wait);
	if (READ_ONCE(fp->removed))
		return EPOLLHUP | EPOLLERR;
	if (atomic_read(&fp->sequence) != READ_ONCE(fp->seen))
		return EPOLLIN | EPOLLRDNORM;
	return 0;
}

static const struct file_operations uff_fp_fops = {
	.owner = THIS_MODULE,
	.open = uff_fp_open,
	.release = uff_fp_release,
	.unlocked_ioctl = uff_fp_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.read = uff_fp_read,
	.poll = uff_fp_poll,
};

static int uff_fp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uff_fp *fp;
	int ret;

	fp = kzalloc(sizeof(*fp), GFP_KERNEL);
	if (!fp)
		return -ENOMEM;
	mutex_init(&fp->lock);
	kref_init(&fp->ref);
	init_waitqueue_head(&fp->wait);
	atomic_set(&fp->sequence, 0);
	fp->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(fp->reset)) {
		ret = PTR_ERR(fp->reset);
		goto fail;
	}
	fp->irq_gpio = devm_gpiod_get(dev, "interrupt", GPIOD_IN);
	if (IS_ERR(fp->irq_gpio)) {
		ret = PTR_ERR(fp->irq_gpio);
		goto fail;
	}
	fp->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(fp->vdd)) {
		ret = PTR_ERR(fp->vdd);
		goto fail;
	}
	/* Factory asks for 3.010 V; PM8550 PLDO steps round this to 3.016 V. */
	ret = regulator_set_voltage(fp->vdd, 3010000, 3016000);
	if (ret)
		goto fail;
	ret = regulator_set_load(fp->vdd, 150000);
	if (ret)
		goto fail;
	fp->irq = gpiod_to_irq(fp->irq_gpio);
	if (fp->irq < 0) {
		ret = fp->irq;
		goto fail;
	}
	ret = devm_request_irq(dev, fp->irq, uff_fp_irq,
			       IRQF_TRIGGER_RISING | IRQF_NO_AUTOEN,
			       dev_name(dev), fp);
	if (ret)
		goto fail;
	fp->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "fingerprint_dev",
		.fops = &uff_fp_fops,
		.parent = dev,
		.mode = 0600,
	};
	ret = misc_register(&fp->misc);
	if (ret) {
		devm_free_irq(dev, fp->irq, fp);
		goto fail;
	}
	platform_set_drvdata(pdev, fp);
	return 0;
 fail:
	kref_put(&fp->ref, uff_fp_free);
	return dev_err_probe(dev, ret, "UFF platform setup failed\n");
}

static void uff_fp_remove(struct platform_device *pdev)
{
	struct uff_fp *fp = platform_get_drvdata(pdev);

	misc_deregister(&fp->misc);
	mutex_lock(&fp->lock);
	fp->removed = true;
	if (uff_fp_power_off(fp))
		dev_err(&pdev->dev, "Failed to disable fingerprint supply\n");
	mutex_unlock(&fp->lock);
	wake_up_interruptible(&fp->wait);
	devm_free_irq(&pdev->dev, fp->irq, fp);
	kref_put(&fp->ref, uff_fp_free);
}

static int uff_fp_suspend(struct device *dev)
{
	struct uff_fp *fp = dev_get_drvdata(dev);
	int ret;

	/* Signed-firmware sleep with the IRQ gated may retain sensor state.
	 * Unprepared/active capture must still block system suspend. */
	mutex_lock(&fp->lock);
	ret = fp->powered && !fp->standby ? -EBUSY : 0;
	mutex_unlock(&fp->lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(uff_fp_pm_ops, uff_fp_suspend, NULL);

static const struct of_device_id uff_fp_of_match[] = {
	{ .compatible = "oneplus,aston-fingerprint" },
	{ }
};
MODULE_DEVICE_TABLE(of, uff_fp_of_match);

static struct platform_driver uff_fp_driver = {
	.probe = uff_fp_probe,
	.remove = uff_fp_remove,
	.driver = {
		.name = "oplus-uff-fp",
		.of_match_table = uff_fp_of_match,
		.pm = pm_sleep_ptr(&uff_fp_pm_ops),
	},
};
module_platform_driver(uff_fp_driver);

MODULE_DESCRIPTION("OnePlus Aston UFF fingerprint platform control");
MODULE_LICENSE("GPL");
