/*
 * Passive Serial Programming Driver
 *
 * Copyright (C) 2014 Michael Grzeschik <mgr at pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <linux/spi/spi.h>

#include <linux/uaccess.h>

#define N_PS_MINORS		32

unsigned int major;
static DECLARE_BITMAP(minors, N_PS_MINORS);

struct psdev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
	bool			open;

	struct mutex		buf_lock;
	u8			*buffer;

	/* gpios for ps protocol */
	unsigned		nconfig_gpio;
	unsigned		confd_gpio;
	unsigned		nstat_gpio;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(minor_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void psdev_complete(void *arg)
{
	if (arg)
		complete(arg);
}

static ssize_t psdev_sync(struct psdev_data *psdev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = psdev_complete;
	message->context = &done;

	spin_lock_irq(&psdev->spi_lock);
	if (psdev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(psdev->spi, message);
	spin_unlock_irq(&psdev->spi_lock);

	if (status == 0) {
		unsigned long time_left = msecs_to_jiffies(100);

		time_left = wait_for_completion_timeout(&done, time_left);
		if (!time_left) {
			message->context = NULL;
			return -ETIMEDOUT;
		}
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t psdev_sync_write(struct psdev_data *psdev, size_t len)
{
	struct spi_transfer t = {
		.tx_buf		= psdev->buffer,
		.len		= len,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return psdev_sync(psdev, &m);
}

/* Write-only message with current device setup */
static ssize_t psdev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct psdev_data *psdev;
	ssize_t	status = 0;
	unsigned long missing;

	if (count > bufsiz)
		return -EMSGSIZE;

	psdev = filp->private_data;

	mutex_lock(&psdev->buf_lock);
	missing = copy_from_user(psdev->buffer, buf, count);
	if (missing == 0)
		status = psdev_sync_write(psdev, count);
	else
		status = -EFAULT;
	mutex_unlock(&psdev->buf_lock);

	return status;
}

static int psdev_start(struct psdev_data *psdev)
{
	struct device *dev = &psdev->spi->dev;
	unsigned long timeout;
	int ret = -ENXIO;

	dev_dbg(dev, "Initiating programming\n");

	/* initiate an FPGA programming */
	gpio_set_value(psdev->nconfig_gpio, 0);

	/*
	 * after about 2 µs the FPGA must acknowledge with
	 * STATUS and CONFIG DONE lines at low level
	 */
	timeout = jiffies + usecs_to_jiffies(2);
	do {
		if ((gpio_get_value(psdev->nstat_gpio) == 0) &&
		    (gpio_get_value(psdev->confd_gpio) == 0)) {
			ret = 0;
			break;
		}
		ndelay(100);

	} while (time_before(jiffies, timeout));

	if (ret != 0) {
		dev_err(dev, "FPGA does not acknowledge the programming initiation\n");
		if (gpio_get_value(psdev->nstat_gpio))
			dev_err(dev, "STATUS is still high!\n");
		if (gpio_get_value(psdev->confd_gpio))
			dev_err(dev, "CONFIG DONE is still high!\n");

		return ret;
	}

	/* arm the FPGA to await its new firmware */
	gpio_set_value(psdev->nconfig_gpio, 1);

	ret = -ENXIO;
	timeout = jiffies + usecs_to_jiffies(1600);
	do {
		if (gpio_get_value(psdev->nstat_gpio) == 0) {
			ret = 0;
			break;
		}
		udelay(10);

	} while (time_before(jiffies, timeout));

	if (ret != 0) {
		dev_err(dev, "FPGA does not acknowledge the programming start\n");
		return ret;
	}

	dev_dbg(dev, "Initiating passed\n");

	/* at the end, wait at least 2 µs prior beginning writing data */
	udelay(2);

	return 0;
}

static int psdev_open(struct inode *inode, struct file *filp)
{
	struct psdev_data *psdev, *tmp;
	struct device *dev;
	int status = -ENXIO;

	list_for_each_entry_safe(psdev, tmp, &device_list, device_entry) {
		if (psdev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (psdev->open) {
		status = -EBUSY;
		goto out;
	}

	dev = &psdev->spi->dev;

	if (status == 0) {
		if (!psdev->buffer) {
			psdev->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!psdev->buffer) {
				dev_dbg(dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			filp->private_data = psdev;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("psdev: nothing for minor %d\n", iminor(inode));

	psdev_start(psdev);

	psdev->open = 1;

 out:

	return status;
}

static int psdev_release(struct inode *inode, struct file *filp)
{
	struct psdev_data *psdev;
	unsigned long timeout;
	struct device *dev;
	int status = -ENXIO;

	psdev = filp->private_data;
	filp->private_data = NULL;

	dev = &psdev->spi->dev;

	/*
	 * when programming was successfully,
	 * both status lines should be at high level
	 */
	timeout = jiffies + usecs_to_jiffies(1000);
	do {
		if ((gpio_get_value(psdev->nstat_gpio)) &&
		    (gpio_get_value(psdev->confd_gpio))) {
			status = 0;
			break;
		}
		udelay(10);

	} while (time_before(jiffies, timeout));

	if (status != 0) {
		dev_err(dev, "FPGA does not acknowledge the programming initiation\n");
		if (gpio_get_value(psdev->nstat_gpio) == 0)
			dev_err(dev, "STATUS is still low!\n");
		if (gpio_get_value(psdev->confd_gpio) == 0)
			dev_err(dev, "CONFIG DONE is still low!\n");
	}

	kfree(psdev->buffer);
	psdev->buffer = NULL;

	psdev->open = 0;

	return status;
}

static const struct file_operations psdev_fops = {
	.owner =	THIS_MODULE,
	.write =	psdev_write,
	.open =		psdev_open,
	.release =	psdev_release,
	.llseek =	no_llseek,
};

static struct class *psdev_class;

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_OF

static const struct of_device_id psdev_dt_ids[] = {
	{ .compatible = "altr,passive-serial" },
	{},
};

MODULE_DEVICE_TABLE(of, psdev_dt_ids);

static int psdev_probe_dt(struct psdev_data *psdev)
{
	int ret = 0;
	struct device *dev = &psdev->spi->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id =
			of_match_device(psdev_dt_ids, &psdev->spi->dev);

	if (!of_id)
		return -EINVAL;

	ret = of_get_named_gpio(np, "gpio-nstatus", 0);
	if (ret < 0) {
		dev_err(dev, "gpio-nstatus property not found\n");
		goto error;
	}
	psdev->nstat_gpio = ret;

	ret = of_get_named_gpio(np, "gpio-confdone", 0);
	if (ret < 0) {
		dev_err(dev, "gpio-confdone property not found\n");
		goto error;
	}
	psdev->confd_gpio = ret;

	ret = of_get_named_gpio(np, "gpio-nconfig", 0);
	if (ret < 0) {
		dev_err(dev, "gpio-nconfig property not found\n");
		goto error;
	}
	psdev->nconfig_gpio = ret;

	ret = devm_gpio_request_one(dev, psdev->nconfig_gpio,
			GPIOF_OUT_INIT_HIGH, "nconfig");
	if (ret) {
		dev_err(dev, "unable to get nconfig gpio\n");
		goto error;
	}

	ret = devm_gpio_request_one(dev, psdev->nstat_gpio,
			GPIOF_DIR_IN,  "nstat");
	if (ret) {
		dev_err(dev, "unable to get nstat gpio\n");
		goto error;
	}

	ret = devm_gpio_request_one(dev, psdev->confd_gpio,
			GPIOF_DIR_IN, "confd");
	if (ret) {
		dev_err(dev, "unable to get confd gpio\n");
		goto error;
	}

 error:
	return ret;
}
#else
static inline int spi_gpio_probe_dt(struct platform_device *pdev)
{
	return -EINVAL;
}
#endif

static int psdev_probe(struct spi_device *spi)
{
	struct psdev_data *psdev;
	unsigned long minor;
	int status = 0;

	/* Allocate driver data */
	psdev = devm_kzalloc(&spi->dev, sizeof(*psdev), GFP_KERNEL);
	if (!psdev)
		return -ENOMEM;

	/* Initialize the driver data */
	psdev->spi = spi;
	spin_lock_init(&psdev->spi_lock);
	mutex_init(&psdev->buf_lock);

	INIT_LIST_HEAD(&psdev->device_entry);
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&minor_lock);
	minor = find_first_zero_bit(minors, N_PS_MINORS);
	if (minor < N_PS_MINORS) {
		struct device *dev;

		psdev->devt = MKDEV(major, minor);
		dev = device_create(psdev_class, &spi->dev, psdev->devt,
				    psdev, "psdev%lu", minor);
		status = ((dev==NULL) || ((int)dev==-1));
		if (status) {
			mutex_unlock(&minor_lock);
			return -ENODEV;
		}
	} else {
		mutex_unlock(&minor_lock);
		dev_dbg(&spi->dev, "no minor number available!\n");
		return -ENODEV;
	}
	set_bit(minor, minors);
	mutex_unlock(&minor_lock);

	list_add(&psdev->device_entry, &device_list);

	spi->mode |= SPI_LSB_FIRST | SPI_NO_CS;
	spi_setup(spi);

	status = psdev_probe_dt(psdev);
	if (status)
		goto error;

	spi_set_drvdata(spi, psdev);

	return status;

 error:
	mutex_lock(&minor_lock);
	clear_bit(MINOR(psdev->devt), minors);
	mutex_unlock(&minor_lock);

	list_del(&psdev->device_entry);
	device_destroy(psdev_class, psdev->devt);

	return status;
}

static int psdev_remove(struct spi_device *spi)
{
	struct psdev_data *psdev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&psdev->spi_lock);
	psdev->spi = NULL;
	spin_unlock_irq(&psdev->spi_lock);

	/* prevent new opens */
	list_del(&psdev->device_entry);
	device_destroy(psdev_class, psdev->devt);
	mutex_lock(&minor_lock);
	clear_bit(MINOR(psdev->devt), minors);
	mutex_unlock(&minor_lock);

	return 0;
}

static struct spi_driver psdev_spi_driver = {
	.driver = {
		.name =		"psdev",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(psdev_dt_ids),
	},
	.probe =	psdev_probe,
	.remove =	psdev_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init psdev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_PS_MINORS > 256);
	status = register_chrdev(0, "psdev", &psdev_fops);
	if (status < 0)
		return status;

	major = status;

	psdev_class = class_create(THIS_MODULE, "psdev");
	if (IS_ERR(psdev_class)) {
		unregister_chrdev(major, psdev_spi_driver.driver.name);
		return PTR_ERR(psdev_class);
	}

	status = spi_register_driver(&psdev_spi_driver);
	if (status < 0) {
		class_destroy(psdev_class);
		unregister_chrdev(major, psdev_spi_driver.driver.name);
	}
	return status;
}
module_init(psdev_init);

static void __exit psdev_exit(void)
{
	spi_unregister_driver(&psdev_spi_driver);
	class_destroy(psdev_class);
	unregister_chrdev(major, psdev_spi_driver.driver.name);
}
module_exit(psdev_exit);

MODULE_AUTHOR("Michael Grzeschik, <mgrzeschik at gmx.net>");
MODULE_DESCRIPTION("User mode SPI Passive Serial interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:psdev");
