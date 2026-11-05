// SPDX-License-Identifier: GPL-2.0-only
/* Power, reset and interrupt control for fingerprint sensors whose SPI bus
 * QTEE owns: /dev/tee_fp. The sensor's trusted application does the rest.
 * Reset timing and IRQ polarity follow each sensor's stock module: the
 * NX809J's zte_fingerprint for the Goodix GUM5228, the Edge 70's
 * anc_fps_mmi for the JIIOV JV0307. They differ only in how long the sensor
 * settles after reset.
 */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/tee_fp.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

struct tee_fp {
	struct miscdevice misc;
	unsigned int settle_us; /* After reset is released. */
	struct mutex lock; /* Serializes control, close and removal. */
	struct kref ref;
	struct gpio_desc *reset, *interrupt;
	struct regulator *vdd;
	wait_queue_head_t wait;
	atomic_t sequence;
	u32 seen, reset_sequence;
	int irq;
	bool opened, powered, removed;
};

static void tee_fp_free(struct kref *ref)
{
	kfree(container_of(ref, struct tee_fp, ref));
}

static void tee_fp_reset(struct tee_fp *fp)
{
	/* Physical low for 10 ms, then high while the sensor settles. */
	gpiod_set_value_cansleep(fp->reset, 1);
	usleep_range(10000, 10100);
	gpiod_set_value_cansleep(fp->reset, 0);
	usleep_range(fp->settle_us, fp->settle_us + 1000);
	fp->reset_sequence++;
}

static int tee_fp_power_on(struct tee_fp *fp)
{
	int ret;

	if (fp->powered)
		return 0;
	ret = regulator_enable(fp->vdd);
	if (ret)
		return ret;
	tee_fp_reset(fp);
	fp->powered = true;
	enable_irq(fp->irq);
	return 0;
}

static int tee_fp_power_off(struct tee_fp *fp)
{
	int ret;

	if (!fp->powered)
		return 0;
	disable_irq(fp->irq);
	gpiod_set_value_cansleep(fp->reset, 1);
	ret = regulator_disable(fp->vdd);
	if (ret) {
		enable_irq(fp->irq);
		return ret;
	}
	fp->powered = false;
	return 0;
}

static irqreturn_t tee_fp_irq(int irq, void *data)
{
	struct tee_fp *fp = data;

	atomic_inc(&fp->sequence);
	wake_up_interruptible(&fp->wait);
	return IRQ_HANDLED;
}

static int tee_fp_open(struct inode *inode, struct file *file)
{
	struct tee_fp *fp = container_of(file->private_data,
					   struct tee_fp, misc);
	int ret = 0;

	mutex_lock(&fp->lock);
	if (fp->removed) {
		ret = -ENODEV;
	} else if (fp->opened) {
		ret = -EBUSY;
	} else {
		fp->opened = true;
		fp->seen = atomic_read(&fp->sequence);
		kref_get(&fp->ref);
		file->private_data = fp;
	}
	mutex_unlock(&fp->lock);
	return ret ? ret : nonseekable_open(inode, file);
}

static int tee_fp_release(struct inode *inode, struct file *file)
{
	struct tee_fp *fp = file->private_data;
	int ret = 0;

	mutex_lock(&fp->lock);
	if (!fp->removed)
		ret = tee_fp_power_off(fp);
	fp->opened = false;
	mutex_unlock(&fp->lock);
	kref_put(&fp->ref, tee_fp_free);
	return ret;
}

static long tee_fp_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct tee_fp *fp = file->private_data;
	struct tee_fp_state state;
	int ret = 0;

	mutex_lock(&fp->lock);
	if (fp->removed) {
		ret = -ENODEV;
		goto out;
	}
	switch (cmd) {
	case TEE_FP_IOC_ENABLE_POWER:
		ret = tee_fp_power_on(fp);
		break;
	case TEE_FP_IOC_DISABLE_POWER:
		ret = tee_fp_power_off(fp);
		break;
	case TEE_FP_IOC_RESET:
		if (!fp->powered)
			ret = -EHOSTDOWN;
		else
			tee_fp_reset(fp);
		break;
	case TEE_FP_IOC_GET_STATE:
		state = (struct tee_fp_state) {
			.version = 1,
			.powered = fp->powered,
			.irq_sequence = atomic_read(&fp->sequence),
			.reset_sequence = fp->reset_sequence,
			.irq_level = gpiod_get_value_cansleep(fp->interrupt),
		};
		if (state.irq_level < 0)
			ret = state.irq_level;
		else if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			ret = -EFAULT;
		break;
	default:
		ret = -ENOTTY;
	}
 out:
	mutex_unlock(&fp->lock);
	return ret;
}

static ssize_t tee_fp_read(struct file *file, char __user *buf,
			      size_t count, loff_t *offset)
{
	struct tee_fp *fp = file->private_data;
	u32 sequence;
	int ret;

	if (count < sizeof(sequence))
		return -EINVAL;
	if (READ_ONCE(fp->removed))
		return -ENODEV;
	if (atomic_read(&fp->sequence) == READ_ONCE(fp->seen)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
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

static __poll_t tee_fp_poll(struct file *file, poll_table *wait)
{
	struct tee_fp *fp = file->private_data;

	poll_wait(file, &fp->wait, wait);
	if (READ_ONCE(fp->removed))
		return EPOLLHUP | EPOLLERR;
	if (atomic_read(&fp->sequence) != READ_ONCE(fp->seen))
		return EPOLLIN | EPOLLRDNORM;
	return 0;
}

static const struct file_operations tee_fp_fops = {
	.owner = THIS_MODULE,
	.open = tee_fp_open,
	.release = tee_fp_release,
	.unlocked_ioctl = tee_fp_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.read = tee_fp_read,
	.poll = tee_fp_poll,
};

static int tee_fp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tee_fp *fp;
	int ret;

	fp = kzalloc_obj(*fp);
	if (!fp)
		return -ENOMEM;
	mutex_init(&fp->lock);
	kref_init(&fp->ref);
	init_waitqueue_head(&fp->wait);
	atomic_set(&fp->sequence, 0);
	fp->settle_us = (uintptr_t)device_get_match_data(dev);
	fp->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(fp->reset)) {
		ret = PTR_ERR(fp->reset);
		goto fail;
	}
	fp->interrupt = devm_gpiod_get(dev, "interrupt", GPIOD_IN);
	if (IS_ERR(fp->interrupt)) {
		ret = PTR_ERR(fp->interrupt);
		goto fail;
	}
	fp->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(fp->vdd)) {
		ret = PTR_ERR(fp->vdd);
		goto fail;
	}
	fp->irq = gpiod_to_irq(fp->interrupt);
	if (fp->irq < 0) {
		ret = fp->irq;
		goto fail;
	}
	/* Both stock modules request IRQF_TRIGGER_RISING | IRQF_ONESHOT. */
	ret = devm_request_irq(dev, fp->irq, tee_fp_irq,
			       IRQF_TRIGGER_RISING | IRQF_NO_AUTOEN,
			       dev_name(dev), fp);
	if (ret)
		goto fail;
	fp->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "tee_fp",
		.fops = &tee_fp_fops,
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
	kref_put(&fp->ref, tee_fp_free);
	return dev_err_probe(dev, ret, "fingerprint platform setup failed\n");
}

static void tee_fp_remove(struct platform_device *pdev)
{
	struct tee_fp *fp = platform_get_drvdata(pdev);

	misc_deregister(&fp->misc);
	mutex_lock(&fp->lock);
	fp->removed = true;
	if (tee_fp_power_off(fp))
		dev_err(&pdev->dev, "Failed to disable fingerprint supply\n");
	mutex_unlock(&fp->lock);
	wake_up_interruptible(&fp->wait);
	devm_free_irq(&pdev->dev, fp->irq, fp);
	kref_put(&fp->ref, tee_fp_free);
}

static int tee_fp_suspend(struct device *dev)
{
	struct tee_fp *fp = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&fp->lock);
	/* Retained sensor sleep and wake-on-finger need separate validation. */
	ret = fp->powered ? -EBUSY : 0;
	mutex_unlock(&fp->lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(tee_fp_pm_ops, tee_fp_suspend, NULL);

static const struct of_device_id tee_fp_of_match[] = {
	/* Stock gf_hw_reset settles 5 ms, anc_reset 10 ms. */
	{ .compatible = "goodix,gum5228", .data = (void *)5000 },
	{ .compatible = "jiiov,jv0307", .data = (void *)10000 },
	{ }
};
MODULE_DEVICE_TABLE(of, tee_fp_of_match);

static struct platform_driver tee_fp_driver = {
	.probe = tee_fp_probe,
	.remove = tee_fp_remove,
	.driver = {
		.name = "tee-fp",
		.of_match_table = tee_fp_of_match,
		.pm = pm_sleep_ptr(&tee_fp_pm_ops),
	},
};
module_platform_driver(tee_fp_driver);

MODULE_DESCRIPTION("Fingerprint platform control for QTEE-owned sensors");
MODULE_LICENSE("GPL");
