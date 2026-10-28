// SPDX-License-Identifier: GPL-2.0-only
/* Goodix ultrasonic fingerprint platform control. QTEE owns the SPI bus.
 * Reset timing and IRQ polarity follow NX809J's zte_fingerprint module.
 */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/goodix_tee_fp.h>
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

struct goodix_fp {
	struct miscdevice misc;
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

static void goodix_fp_free(struct kref *ref)
{
	kfree(container_of(ref, struct goodix_fp, ref));
}

static void goodix_fp_reset(struct goodix_fp *fp)
{
	/* Stock gf_hw_reset: physical low for 10 ms, then high for 5 ms. */
	dev_info(fp->misc.parent, "reset: asserting GPIO\n");
	gpiod_set_value_cansleep(fp->reset, 1);
	dev_info(fp->misc.parent, "reset: GPIO asserted\n");
	usleep_range(10000, 10100);
	dev_info(fp->misc.parent, "reset: deasserting GPIO\n");
	gpiod_set_value_cansleep(fp->reset, 0);
	dev_info(fp->misc.parent, "reset: GPIO deasserted\n");
	usleep_range(5000, 6000);
	fp->reset_sequence++;
	dev_info(fp->misc.parent, "reset: complete\n");
}

static int goodix_fp_power_on(struct goodix_fp *fp)
{
	int ret;

	if (fp->powered)
		return 0;
	dev_info(fp->misc.parent, "power-on: enabling vdd\n");
	ret = regulator_enable(fp->vdd);
	dev_info(fp->misc.parent, "power-on: vdd result %d\n", ret);
	if (ret)
		return ret;
	goodix_fp_reset(fp);
	fp->powered = true;
	dev_info(fp->misc.parent, "power-on: enabling IRQ\n");
	enable_irq(fp->irq);
	dev_info(fp->misc.parent, "power-on: complete\n");
	return 0;
}

static int goodix_fp_power_off(struct goodix_fp *fp)
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

static irqreturn_t goodix_fp_irq(int irq, void *data)
{
	struct goodix_fp *fp = data;

	atomic_inc(&fp->sequence);
	wake_up_interruptible(&fp->wait);
	return IRQ_HANDLED;
}

static int goodix_fp_open(struct inode *inode, struct file *file)
{
	struct goodix_fp *fp = container_of(file->private_data,
					   struct goodix_fp, misc);
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

static int goodix_fp_release(struct inode *inode, struct file *file)
{
	struct goodix_fp *fp = file->private_data;
	int ret = 0;

	mutex_lock(&fp->lock);
	if (!fp->removed)
		ret = goodix_fp_power_off(fp);
	fp->opened = false;
	mutex_unlock(&fp->lock);
	kref_put(&fp->ref, goodix_fp_free);
	return ret;
}

static long goodix_fp_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct goodix_fp *fp = file->private_data;
	struct goodix_fp_state state;
	int ret = 0;

	mutex_lock(&fp->lock);
	if (fp->removed) {
		ret = -ENODEV;
		goto out;
	}
	switch (cmd) {
	case GOODIX_FP_IOC_ENABLE_POWER:
		ret = goodix_fp_power_on(fp);
		break;
	case GOODIX_FP_IOC_DISABLE_POWER:
		ret = goodix_fp_power_off(fp);
		break;
	case GOODIX_FP_IOC_RESET:
		if (!fp->powered)
			ret = -EHOSTDOWN;
		else
			goodix_fp_reset(fp);
		break;
	case GOODIX_FP_IOC_GET_STATE:
		state = (struct goodix_fp_state) {
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

static ssize_t goodix_fp_read(struct file *file, char __user *buf,
			      size_t count, loff_t *offset)
{
	struct goodix_fp *fp = file->private_data;
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

static __poll_t goodix_fp_poll(struct file *file, poll_table *wait)
{
	struct goodix_fp *fp = file->private_data;

	poll_wait(file, &fp->wait, wait);
	if (READ_ONCE(fp->removed))
		return EPOLLHUP | EPOLLERR;
	if (atomic_read(&fp->sequence) != READ_ONCE(fp->seen))
		return EPOLLIN | EPOLLRDNORM;
	return 0;
}

static const struct file_operations goodix_fp_fops = {
	.owner = THIS_MODULE,
	.open = goodix_fp_open,
	.release = goodix_fp_release,
	.unlocked_ioctl = goodix_fp_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.read = goodix_fp_read,
	.poll = goodix_fp_poll,
};

static int goodix_fp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct goodix_fp *fp;
	int ret;

	fp = kzalloc_obj(*fp);
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
	/* Stock gf_open requests IRQF_TRIGGER_RISING | IRQF_ONESHOT. */
	ret = devm_request_irq(dev, fp->irq, goodix_fp_irq,
			       IRQF_TRIGGER_RISING | IRQF_NO_AUTOEN,
			       dev_name(dev), fp);
	if (ret)
		goto fail;
	fp->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "goodix_fp",
		.fops = &goodix_fp_fops,
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
	kref_put(&fp->ref, goodix_fp_free);
	return dev_err_probe(dev, ret, "Goodix platform setup failed\n");
}

static void goodix_fp_remove(struct platform_device *pdev)
{
	struct goodix_fp *fp = platform_get_drvdata(pdev);

	misc_deregister(&fp->misc);
	mutex_lock(&fp->lock);
	fp->removed = true;
	if (goodix_fp_power_off(fp))
		dev_err(&pdev->dev, "Failed to disable fingerprint supply\n");
	mutex_unlock(&fp->lock);
	wake_up_interruptible(&fp->wait);
	devm_free_irq(&pdev->dev, fp->irq, fp);
	kref_put(&fp->ref, goodix_fp_free);
}

static int goodix_fp_suspend(struct device *dev)
{
	struct goodix_fp *fp = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&fp->lock);
	/* Retained sensor sleep and wake-on-finger need separate validation. */
	ret = fp->powered ? -EBUSY : 0;
	mutex_unlock(&fp->lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(goodix_fp_pm_ops, goodix_fp_suspend, NULL);

static const struct of_device_id goodix_fp_of_match[] = {
	{ .compatible = "goodix,gum5228" },
	{ }
};
MODULE_DEVICE_TABLE(of, goodix_fp_of_match);

static struct platform_driver goodix_fp_driver = {
	.probe = goodix_fp_probe,
	.remove = goodix_fp_remove,
	.driver = {
		.name = "goodix-tee-fp",
		.of_match_table = goodix_fp_of_match,
		.pm = pm_sleep_ptr(&goodix_fp_pm_ops),
	},
};
module_platform_driver(goodix_fp_driver);

MODULE_DESCRIPTION("Goodix ultrasonic fingerprint platform control");
MODULE_LICENSE("GPL");
