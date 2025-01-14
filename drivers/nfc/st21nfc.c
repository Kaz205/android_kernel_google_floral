// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 */

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>
#include <net/nfc/nci.h>
#include <linux/clk.h>

#define MAX_BUFFER_SIZE 260
#define HEADER_LENGTH 3
#define IDLE_CHARACTER 0x7e
#define WAKEUP_SRC_TIMEOUT 2000

#define DRIVER_VERSION "2.0.14"
#define ST21NFC_NAME "st21nfc"

#define ST21NFC_MAGIC	0xEA
/*
 * ST21NFC power control via ioctl
 * ST21NFC_GET_WAKEUP :  poll gpio-level for Wakeup pin
 */
#define ST21NFC_GET_WAKEUP	      _IOR(ST21NFC_MAGIC, 0x01, unsigned int)
#define ST21NFC_PULSE_RESET		_IOR(ST21NFC_MAGIC, 0x02, unsigned int)
#define ST21NFC_SET_POLARITY_HIGH     _IOR(ST21NFC_MAGIC, 0x05, unsigned int)

enum st21nfc_read_state {
	ST21NFC_HEADER,
	ST21NFC_PAYLOAD
};

struct st21nfc_device {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice st21nfc_device;
	atomic_t irq_enabled;
	bool irq_wake_up;
	bool irq_is_attached;
	bool device_open; /* Is device open? */
	enum st21nfc_read_state r_state_current;

	/* CLK control */
	struct clk *s_clk;

	/* GPIO for NFCC IRQ pin (input) */
	struct gpio_desc *gpiod_irq;
	/* GPIO for NFCC Reset pin (output) */
	struct gpio_desc *gpiod_reset;
};

/*
 * Routine to enable clock.
 * this routine can be extended to select from multiple
 * sources based on clk_src_name.
 */
static int st21nfc_clock_select(struct st21nfc_device *st21nfc_dev,
				struct device *dev)
{
	/* Don't initialize clock if pinctrl is enabled */
	if (device_property_read_bool(dev, "st,clk_pinctrl"))
		return 0;

	st21nfc_dev->s_clk = clk_get(&st21nfc_dev->client->dev, "nfc_ref_clk");

	/* if NULL we assume external crystal and dont fail */
	if ((st21nfc_dev->s_clk == NULL) || IS_ERR(st21nfc_dev->s_clk))
		return 0;

	if (clk_prepare_enable(st21nfc_dev->s_clk))
		return -EINVAL;

	return 0;
}

static irqreturn_t st21nfc_dev_irq_handler(int irq, void *dev_id)
{
	struct st21nfc_device *st21nfc_dev = dev_id;

	if (!atomic_read(&st21nfc_dev->irq_enabled))
		return IRQ_NONE;

	if (device_may_wakeup(&st21nfc_dev->client->dev))
		pm_wakeup_event(&st21nfc_dev->client->dev,
			WAKEUP_SRC_TIMEOUT);
	atomic_set(&st21nfc_dev->irq_enabled, 0);

	/* Wake up waiting readers */
	wake_up(&st21nfc_dev->read_wq);

	return IRQ_HANDLED;
}

static int st21nfc_loc_set_polaritymode_high(struct st21nfc_device *st21nfc_dev)
{
	struct i2c_client *client = st21nfc_dev->client;
	struct device *dev = &client->dev;

	if (st21nfc_dev->irq_is_attached) {
		devm_free_irq(dev, client->irq, st21nfc_dev);
		st21nfc_dev->irq_is_attached = false;
	}
	if (irq_set_irq_type(client->irq, IRQ_TYPE_LEVEL_HIGH))
		return -ENODEV;

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	atomic_set(&st21nfc_dev->irq_enabled, 1);

	if (devm_request_irq(dev, client->irq, st21nfc_dev_irq_handler,
				IRQ_TYPE_LEVEL_HIGH, client->name, st21nfc_dev))
		return -ENODEV;

	st21nfc_dev->irq_is_attached = true;
	atomic_set(&st21nfc_dev->irq_enabled, 0);

	return 0;
}

static ssize_t st21nfc_dev_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	struct st21nfc_device *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_device,
						       st21nfc_device);
	char buffer[MAX_BUFFER_SIZE];
	int ret, idle;
	DEFINE_MUTEX(r_state_current_lock);

	if (!count)
		return 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(buffer, 0, sizeof(buffer));
	/* Read data */
	ret = i2c_master_recv(st21nfc_dev->client, buffer, count);
	if (ret < 0) {
		return ret;
	}
	mutex_lock(&r_state_current_lock);
	if (st21nfc_dev->r_state_current == ST21NFC_HEADER) {
		/* Counting idle index */
		for (idle = 0;
		     idle < ret && buffer[idle] == IDLE_CHARACTER;
		     idle++)
			;

		if (idle > 0 && idle < HEADER_LENGTH) {
			memmove(buffer, buffer + idle, ret - idle);
			ret = i2c_master_recv(st21nfc_dev->client,
					      buffer + ret - idle, idle);
			if (ret < 0) {
				mutex_unlock(&st21nfc_dev->read_mutex);
				return ret;
			}
			ret = count;
		}
	}
	mutex_unlock(&r_state_current_lock);

	if (idle < HEADER_LENGTH) {
		/* change state only if a payload is detected, i.e. size > 0*/
		if ((st21nfc_dev->r_state_current == ST21NFC_HEADER) &&
			(buffer[2] > 0))
			st21nfc_dev->r_state_current = ST21NFC_PAYLOAD;
		else
			st21nfc_dev->r_state_current = ST21NFC_HEADER;
	}

	if (copy_to_user(buf, buffer, ret))
		return -EFAULT;

	return ret;
}

static ssize_t st21nfc_dev_write(struct file *filp, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct st21nfc_device *st21nfc_dev = container_of(filp->private_data,
				   struct st21nfc_device, st21nfc_device);
	char buffer[MAX_BUFFER_SIZE];

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	/* Write data */
	if (i2c_master_send(st21nfc_dev->client, buffer, count) != count)
		return -EIO;

	return 0;
}

static int st21nfc_dev_open(struct inode *inode, struct file *filp)
{
	struct st21nfc_device *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_device,
						       st21nfc_device);
	int ret = 0;

	if (st21nfc_dev->device_open)
		ret = -EBUSY;
	else
		st21nfc_dev->device_open = true;

	return ret;
}

static int st21nfc_release(struct inode *inode, struct file *file)
{
	struct st21nfc_device *st21nfc_dev = container_of(file->private_data,
						       struct st21nfc_device,
						       st21nfc_device);

	st21nfc_dev->device_open = false;
	return 0;
}

static long st21nfc_dev_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	struct st21nfc_device *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_device,
						       st21nfc_device);

	int ret = 0;

	switch (cmd) {
	case ST21NFC_SET_POLARITY_HIGH:
		st21nfc_loc_set_polaritymode_high(st21nfc_dev);
		break;
	case ST21NFC_PULSE_RESET:
		/* Double pulse is done to exit Quick boot mode.*/
		if (!IS_ERR(st21nfc_dev->gpiod_reset)) {
			/* pulse low for 20 millisecs */
			gpiod_set_value(st21nfc_dev->gpiod_reset, 0);
			msleep(20);
			gpiod_set_value(st21nfc_dev->gpiod_reset, 1);
			usleep_range(10000, 11000);
			/* pulse low for 20 millisecs */
			gpiod_set_value(st21nfc_dev->gpiod_reset, 0);
			msleep(20);
			gpiod_set_value(st21nfc_dev->gpiod_reset, 1);
		}
		st21nfc_dev->r_state_current = ST21NFC_HEADER;
		break;
	case ST21NFC_GET_WAKEUP:
		/*
		 * deliver state of Wake_up_pin as return value of ioctl
		 *
		 * Warning: depending on gpiod_get_value implementation,
		 * it can returns a value different than 1 in case of high level
		 */
		ret = !!gpiod_get_value(st21nfc_dev->gpiod_irq);
		break;
	default:
		pr_err("%s: Unknown ioctl %u", __func__, cmd);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static unsigned int st21nfc_poll(struct file *file, poll_table *wait)
{
	struct st21nfc_device *st21nfc_dev = container_of(file->private_data,
						       struct st21nfc_device,
						       st21nfc_device);
	unsigned int mask = 0;
	int pinlev;

	/* wait for Wake_up_pin == high  */
	poll_wait(file, &st21nfc_dev->read_wq, wait);

	pinlev = gpiod_get_value(st21nfc_dev->gpiod_irq);

	if (pinlev) {
		mask = POLLIN | POLLRDNORM;	/* signal data avail */
		atomic_set(&st21nfc_dev->irq_enabled, 0);
	} else {
		atomic_set(&st21nfc_dev->irq_enabled, 1);
	}
	return mask;
}

static const struct file_operations st21nfc_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = st21nfc_dev_read,
	.write = st21nfc_dev_write,
	.open = st21nfc_dev_open,
	.poll = st21nfc_poll,
	.release = st21nfc_release,
	.unlocked_ioctl = st21nfc_dev_ioctl
};

static int st21nfc_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct st21nfc_device *st21nfc_dev;
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	st21nfc_dev = devm_kzalloc(dev, sizeof(*st21nfc_dev), GFP_KERNEL);
	if (st21nfc_dev == NULL)
		return -ENOMEM;

	/* store for later use */
	st21nfc_dev->client = client;
	st21nfc_dev->r_state_current = ST21NFC_HEADER;
	client->adapter->retries = 0;

	st21nfc_dev->gpiod_irq = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(st21nfc_dev->gpiod_irq))
		return -ENODEV;

	st21nfc_dev->gpiod_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(st21nfc_dev->gpiod_reset))
		return -ENODEV;

	ret = st21nfc_clock_select(st21nfc_dev, dev);
	if (ret)
		return ret;

	client->irq = gpiod_to_irq(st21nfc_dev->gpiod_irq);

	/* init mutex and queues */
	init_waitqueue_head(&st21nfc_dev->read_wq);
	st21nfc_dev->st21nfc_device.minor = MISC_DYNAMIC_MINOR;
	st21nfc_dev->st21nfc_device.name = "st21nfc";
	st21nfc_dev->st21nfc_device.fops = &st21nfc_dev_fops;
	st21nfc_dev->st21nfc_device.parent = dev;

	i2c_set_clientdata(client, st21nfc_dev);
	ret = misc_register(&st21nfc_dev->st21nfc_device);
	if (ret)
		return ret;

	device_init_wakeup(&client->dev, true);
	device_set_wakeup_capable(&client->dev, true);
	st21nfc_dev->irq_wake_up = false;

	return 0;
}

static int st21nfc_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct st21nfc_device *st21nfc_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) &&
	    atomic_read(&st21nfc_dev->irq_enabled)) {
		if (!enable_irq_wake(client->irq))
			st21nfc_dev->irq_wake_up = true;
	}

	return 0;
}

static int st21nfc_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct st21nfc_device *st21nfc_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && st21nfc_dev->irq_wake_up
	    && !disable_irq_wake(client->irq))
		st21nfc_dev->irq_wake_up = false;

	return 0;
}

static const struct i2c_device_id st21nfc_id[] = {
	{ "st21nfc", 0 },
	{ }
};

static const struct of_device_id st21nfc_of_match[] = {
	{ .compatible = "st,st21nfc" },
	{ }
};

static const struct dev_pm_ops st21nfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st21nfc_suspend, st21nfc_resume)
};

static struct i2c_driver st21nfc_driver = {
	.driver = {
		.name	= "st21nfc",
		.owner	= THIS_MODULE,
		.of_match_table	= st21nfc_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm = &st21nfc_pm_ops,
	},
	.probe		= st21nfc_probe,
	.id_table = st21nfc_id,
};

static int __init st21nfc_dev_init(void)
{
	return i2c_add_driver(&st21nfc_driver);
}
device_initcall(st21nfc_dev_init);
