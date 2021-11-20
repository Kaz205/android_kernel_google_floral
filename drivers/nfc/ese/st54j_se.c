// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 ST Microelectronics S.A.
 * Copyright 2019 Google Inc.
 * Copyright (C) 2021 Kazuki Hashimoto <kazukih@tuta.io>.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi-geni-qcom.h>

#define DRIVER_VERSION "1.1.4"
#define ST54_MAX_BUF 258U

#define ST54J_SE_MAGIC	0xE5
#define ST54J_SE_RESET            _IOR(ST54J_SE_MAGIC, 0x01, unsigned int)

struct st54j_se_dev {
	struct spi_device	*spi;
	struct mutex		mutex;
	struct	miscdevice	device;
	bool device_open;
	/* GPIO for SE Reset pin (output) */
	struct gpio_desc *gpiod_se_reset;
};

static long st54j_se_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int ret = 0;
	struct st54j_se_dev *ese_dev = filp->private_data;
	dev_dbg(&ese_dev->spi->dev, "%s: enter, cmd=%u\n", __func__, cmd);

	if (cmd == ST54J_SE_RESET) {
		dev_info(&ese_dev->spi->dev, "%s  Reset Request received!!\n",
			 __func__);
		mutex_lock(&ese_dev->mutex);
		if (!IS_ERR(ese_dev->gpiod_se_reset)) {
			/* pulse low for 5 millisecs */
			gpiod_set_value(ese_dev->gpiod_se_reset, 0);
			usleep_range(5000, 5500);
			gpiod_set_value(ese_dev->gpiod_se_reset, 1);
			dev_info(&ese_dev->spi->dev,
				 "%s sent Reset request on eSE\n", __func__);
		} else {
			ret = -ENODEV;
			dev_err(&ese_dev->spi->dev,
			"%s : Unable to request esereset %d \n",
			__func__, IS_ERR(ese_dev->gpiod_se_reset));
		}
		mutex_unlock(&ese_dev->mutex);
	}
	return ret;
}

static int st54j_se_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct st54j_se_dev *ese_dev = container_of(filp->private_data,
				struct st54j_se_dev, device);
	if (ese_dev->device_open) {
		ret = -EBUSY;
		dev_info(&ese_dev->spi->dev, "%s: device already opened\n",
			 __func__);
	} else {
		ese_dev->device_open = true;
		filp->private_data = ese_dev;
		dev_info(&ese_dev->spi->dev, "%s: device_open = %d", __func__,
			 ese_dev->device_open);
	}
	return ret;
}

static int st54j_se_release(struct inode *ino, struct file *filp)
{
	struct st54j_se_dev *ese_dev = filp->private_data;

	ese_dev->device_open = false;
	dev_dbg(&ese_dev->spi->dev, "%s : device_open  = %d\n",
		 __func__, ese_dev->device_open);
	return 0;
}

static ssize_t st54j_se_write(struct file *filp, const char __user *ubuf,
			      size_t len, loff_t *offset)
{
	struct st54j_se_dev *ese_dev = filp->private_data;
	int ret = -EFAULT;
	size_t bytes = len;
	char tx_buf[ST54_MAX_BUF];

	if (len > INT_MAX)
		return -EINVAL;
	dev_dbg(&ese_dev->spi->dev, "%s : writing %zu bytes.\n", __func__,
		bytes);
	mutex_lock(&ese_dev->mutex);
	while (bytes > 0) {
		size_t block = bytes < ST54_MAX_BUF ? bytes : ST54_MAX_BUF;

		ret = copy_from_user(tx_buf, ubuf, block);
		if (ret) {
			dev_dbg(&ese_dev->spi->dev,
				"failed to copy from user\n");
			goto err;
		}

		ret = spi_write(ese_dev->spi, tx_buf, block);
		if (ret < 0) {
			dev_dbg(&ese_dev->spi->dev, "failed to write to SPI\n");
			goto err;
		}
		ubuf += block;
		bytes -= block;
	}
	ret = len;
err:
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static ssize_t st54j_se_read(struct file *filp, char __user *ubuf, size_t len,
			     loff_t *offset)
{
	struct st54j_se_dev *ese_dev = filp->private_data;
	ssize_t ret = -EFAULT;
	size_t bytes = len;
	char rx_buf[ST54_MAX_BUF] = {0};

	if (len > INT_MAX)
		return -EINVAL;
	dev_dbg(&ese_dev->spi->dev, "%s : reading %zu bytes.\n", __func__,
		bytes);
	mutex_lock(&ese_dev->mutex);
	while (bytes > 0) {
		size_t block = bytes < ST54_MAX_BUF ? bytes : ST54_MAX_BUF;

		ret = spi_read(ese_dev->spi, rx_buf, block);
		if (ret < 0) {
			dev_err(&ese_dev->spi->dev,
				"failed to read from SPI\n");
			goto err;
		}

		ret = copy_to_user(ubuf, rx_buf, block);
		if (ret) {
			dev_err(&ese_dev->spi->dev,
				"failed to copy from user\n");
			goto err;
		}
		ubuf += block;
		bytes -= block;
	}
	ret = len;
err:
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static const struct file_operations st54j_se_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = st54j_se_read,
	.write = st54j_se_write,
	.open = st54j_se_open,
	.release = st54j_se_release,
	.unlocked_ioctl = st54j_se_ioctl
};

static int st54j_se_probe(struct spi_device *spi)
{
	struct st54j_se_dev *ese_dev;
	struct spi_geni_qcom_ctrl_data *spi_param;
	struct device *dev = &spi->dev;
	struct device_node *np = dev_of_node(&spi->dev);
	int ret;

	dev_dbg(dev, "%s entry\n", __func__);

	if (!np) {
		dev_err(dev, "%s: device tree data missing\n", __func__);
		return -EINVAL;
	}

	ese_dev = kmalloc(sizeof(*ese_dev), GFP_KERNEL);
	if (ese_dev == NULL)
		return -ENOMEM;

	spi_param = kmalloc(sizeof(*spi_param), GFP_KERNEL);
	if (spi_param == NULL)
		return -ENOMEM;

	ese_dev->spi = spi;
	ese_dev->device.minor = MISC_DYNAMIC_MINOR;
	ese_dev->device.name = "st54j_se";
	ese_dev->device.fops = &st54j_se_dev_fops;

	spi->bits_per_word = 8;
	spi_param->spi_cs_clk_delay = 90;
	spi->controller_data = spi_param;

	ese_dev->gpiod_se_reset = gpiod_get(dev, "esereset", GPIOD_OUT_HIGH);
	if (IS_ERR(ese_dev->gpiod_se_reset)) {
		dev_err(dev,
			"%s : Unable to request esereset %d \n",
			__func__, IS_ERR(ese_dev->gpiod_se_reset));
		return -ENODEV;
	}

	mutex_init(&ese_dev->mutex);
	ret = misc_register(&ese_dev->device);
	if (ret) {
		dev_err(dev, "%s: misc_register failed\n", __func__);
		goto err;
	}
	dev_dbg(dev, "%s: eSE is configured\n", __func__);
	spi_set_drvdata(spi, ese_dev);

	return 0;
err:
	mutex_destroy(&ese_dev->mutex);
	return ret;
}

static const struct of_device_id st54j_se_match_table[] = {
	{ .compatible = "st,st54j_se" },
	{ }
};

static struct spi_driver st54j_se_driver = {
	.probe = st54j_se_probe,
	.driver = {
		.name = "st54j_se",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = st54j_se_match_table,
	},
};

static int __init st54j_se_init(void)
{
	return spi_register_driver(&st54j_se_driver);
}
device_initcall(st54j_se_init);
