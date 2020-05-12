/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/sx127xB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

uint32_t RESET_SX127X = 0;
uint32_t CS_SX127X = 0;
static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
				| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct sx127x_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void sx127x_complete(void *arg)
{
	complete(arg);
}

static ssize_t
sx127x_sync(struct sx127x_data *sx127x, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = sx127x_complete;
	message->context = &done;

	spin_lock_irq(&sx127x->spi_lock);
	if (sx127x->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(sx127x->spi, message);
	spin_unlock_irq(&sx127x->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
sx127x_sync_write(struct sx127x_data *sx127x, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= sx127x->tx_buffer,
			.len		= len,
			.speed_hz	= sx127x->speed_hz,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return sx127x_sync(sx127x, &m);
}

static inline ssize_t
sx127x_sync_read(struct sx127x_data *sx127x, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= sx127x->rx_buffer,
			.len		= len,
			.speed_hz	= sx127x->speed_hz,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return sx127x_sync(sx127x, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
sx127x_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct sx127x_data	*sx127x;
	ssize_t			status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	sx127x = filp->private_data;

	mutex_lock(&sx127x->buf_lock);
	status = sx127x_sync_read(sx127x, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, sx127x->rx_buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&sx127x->buf_lock);
	return status;
}

/* Write-only message with current device setup */
static ssize_t
sx127x_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct sx127x_data	*sx127x;
	ssize_t			status = 0;
	unsigned long		missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	sx127x = filp->private_data;

	mutex_lock(&sx127x->buf_lock);
	missing = copy_from_user(sx127x->tx_buffer, buf, count);
	if (missing == 0)
		status = sx127x_sync_write(sx127x, count);
	else
		status = -EFAULT;
	mutex_unlock(&sx127x->buf_lock);
	return status;
}

static int sx127x_message(struct sx127x_data *sx127x,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total, tx_total, rx_total;
	u8			*tx_buf, *rx_buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = sx127x->tx_buffer;
	rx_buf = sx127x->rx_buffer;
	total = 0;
	tx_total = 0;
	rx_total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Since the function returns the total length of transfers
		 * on success, restrict the total to positive int values to
		 * avoid the return value looking like an error.  Also check
		 * each transfer length to avoid arithmetic overflow.
		 */
		if (total > INT_MAX || k_tmp->len > INT_MAX) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			/* this transfer needs space in RX bounce buffer */
			rx_total += k_tmp->len;
			if (rx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->rx_buf = rx_buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
			rx_buf += k_tmp->len;
		}
		if (u_tmp->tx_buf) {
			/* this transfer needs space in TX bounce buffer */
			tx_total += k_tmp->len;
			if (tx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->tx_buf = tx_buf;
			if (copy_from_user(tx_buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
			tx_buf += k_tmp->len;
		}

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
		if (!k_tmp->speed_hz)
			k_tmp->speed_hz = sx127x->speed_hz;
#ifdef VERBOSE
		dev_dbg(&sx127x->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : sx127x->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : sx127x->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = sx127x_sync(sx127x, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	rx_buf = sx127x->rx_buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, rx_buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
			rx_buf += u_tmp->len;
		}
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static struct spi_ioc_transfer *
sx127x_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
		unsigned *n_ioc)
{
	struct spi_ioc_transfer	*ioc;
	u32	tmp;

	/* Check type, command number and direction */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC
			|| _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
			|| _IOC_DIR(cmd) != _IOC_WRITE)
		return ERR_PTR(-ENOTTY);

	tmp = _IOC_SIZE(cmd);
	if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
		return ERR_PTR(-EINVAL);
	*n_ioc = tmp / sizeof(struct spi_ioc_transfer);
	if (*n_ioc == 0)
		return NULL;

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (!ioc)
		return ERR_PTR(-ENOMEM);
	if (__copy_from_user(ioc, u_ioc, tmp)) {
		kfree(ioc);
		return ERR_PTR(-EFAULT);
	}
	return ioc;
}

static long
sx127x_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct sx127x_data	*sx127x;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	sx127x = filp->private_data;
	spin_lock_irq(&sx127x->spi_lock);
	spi = spi_dev_get(sx127x->spi);
	spin_unlock_irq(&sx127x->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&sx127x->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MODE32:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u32 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(sx127x->speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE)
			retval = __get_user(tmp, (u8 __user *)arg);
		else
			retval = __get_user(tmp, (u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u16)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval >= 0)
				sx127x->speed_hz = tmp;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
			spi->max_speed_hz = save;
		}
		break;

	default:
		/* segmented and/or full-duplex I/O request */
		/* Check message and copy into scratch area */
		ioc = sx127x_get_ioc_message(cmd,
				(struct spi_ioc_transfer __user *)arg, &n_ioc);
		if (IS_ERR(ioc)) {
			retval = PTR_ERR(ioc);
			break;
		}
		if (!ioc)
			break;	/* n_ioc is also 0 */

		/* translate to spi_message, execute */
		retval = sx127x_message(sx127x, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&sx127x->buf_lock);
	spi_dev_put(spi);
	return retval;
}

static long cs_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
        int ret = 3;

        switch(cmd)
        {
                case 0:
                        gpio_direction_output(CS_SX127X, arg);
                        gpio_free(CS_SX127X);
                        break;
                default:
                        return -EINVAL;
        }

        return ret;
}


#ifdef CONFIG_COMPAT
static long
sx127x_compat_ioc_message(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct spi_ioc_transfer __user	*u_ioc;
	int				retval = 0;
	struct sx127x_data		*sx127x;
	struct spi_device		*spi;
	unsigned			n_ioc, n;
	struct spi_ioc_transfer		*ioc;

	u_ioc = (struct spi_ioc_transfer __user *) compat_ptr(arg);
	if (!access_ok(VERIFY_READ, u_ioc, _IOC_SIZE(cmd)))
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	sx127x = filp->private_data;
	spin_lock_irq(&sx127x->spi_lock);
	spi = spi_dev_get(sx127x->spi);
	spin_unlock_irq(&sx127x->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* SPI_IOC_MESSAGE needs the buffer locked "normally" */
	mutex_lock(&sx127x->buf_lock);

	/* Check message and copy into scratch area */
	ioc = sx127x_get_ioc_message(cmd, u_ioc, &n_ioc);
	if (IS_ERR(ioc)) {
		retval = PTR_ERR(ioc);
		goto done;
	}
	if (!ioc)
		goto done;	/* n_ioc is also 0 */

	/* Convert buffer pointers */
	for (n = 0; n < n_ioc; n++) {
		ioc[n].rx_buf = (uintptr_t) compat_ptr(ioc[n].rx_buf);
		ioc[n].tx_buf = (uintptr_t) compat_ptr(ioc[n].tx_buf);
	}

	/* translate to spi_message, execute */
	retval = sx127x_message(sx127x, ioc, n_ioc);
	kfree(ioc);

done:
	mutex_unlock(&sx127x->buf_lock);
	spi_dev_put(spi);
	return retval;
}

static long
sx127x_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	if (_IOC_TYPE(cmd) == SPI_IOC_MAGIC
			&& _IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(0))
			&& _IOC_DIR(cmd) == _IOC_WRITE)
		return sx127x_compat_ioc_message(filp, cmd, arg);

	return sx127x_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define sx127x_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int sx127x_open(struct inode *inode, struct file *filp)
{
	struct sx127x_data	*sx127x;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(sx127x, &device_list, device_entry) {
		if (sx127x->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	if (!sx127x->tx_buffer) {
		sx127x->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!sx127x->tx_buffer) {
				dev_dbg(&sx127x->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			goto err_find_dev;
			}
		}

	if (!sx127x->rx_buffer) {
		sx127x->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!sx127x->rx_buffer) {
			dev_dbg(&sx127x->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	sx127x->users++;
	filp->private_data = sx127x;
	nonseekable_open(inode, filp);

	mutex_unlock(&device_list_lock);
	return 0;

err_alloc_rx_buf:
	kfree(sx127x->tx_buffer);
	sx127x->tx_buffer = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int sx127x_release(struct inode *inode, struct file *filp)
{
	struct sx127x_data	*sx127x;
	int			status = 0;

	mutex_lock(&device_list_lock);
	sx127x = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	sx127x->users--;
	if (!sx127x->users) {
		int		dofree;

		kfree(sx127x->tx_buffer);
		sx127x->tx_buffer = NULL;

		kfree(sx127x->rx_buffer);
		sx127x->rx_buffer = NULL;

		if (sx127x->spi)
			sx127x->speed_hz = sx127x->spi->max_speed_hz;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&sx127x->spi_lock);
		dofree = (sx127x->spi == NULL);
		spin_unlock_irq(&sx127x->spi_lock);

		if (dofree)
			kfree(sx127x);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations sx127x_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	sx127x_write,
	.read =		sx127x_read,
	.unlocked_ioctl = cs_ioctl,//sx127x_ioctl,
	.compat_ioctl = sx127x_compat_ioctl,
	.open =		sx127x_open,
	.release =	sx127x_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *sx127x_class;

#ifdef CONFIG_OF
static const struct of_device_id sx127x_dt_ids[] = {
	{ .compatible = "rohm,dh2228fv" },
	{ .compatible = "sx127x" },
	{},
};
MODULE_DEVICE_TABLE(of, sx127x_dt_ids);
#endif

/*-------------------------------------------------------------------------*/

static int sx127x_probe(struct spi_device *spi)
{
	struct sx127x_data	*sx127x;
	int			status;
	unsigned long		minor;
	
	int 			ret;
	uint32_t 		value;
	uint8_t			txdata,rxdata;

	//printk("***************** fun:%s, line = %d\n", __FUNCTION__, __LINE__);

//add by lsb 20190612
	RESET_SX127X = of_get_named_gpio(spi->dev.of_node, "reset-sx127x", 0);
        if (RESET_SX127X < 0) {
                dev_err(&spi->dev, "error sx127x reset gpio: %d\n", RESET_SX127X);
                return RESET_SX127X;
        }
	ret = gpio_request(RESET_SX127X, "res-sx127x");
        if(ret) {
                printk("error sx127x reset gpio: %d\n", ret);
                return ret;
        }
	gpio_free(RESET_SX127X);
	gpio_direction_output(RESET_SX127X, 0);
        gpio_set_value(RESET_SX127X, 0);

        mdelay(5);
        gpio_set_value(RESET_SX127X, 1);
//end add

//add by lsb 20190725
	CS_SX127X = of_get_named_gpio(spi->dev.of_node, "cs-sx127x", 0);
        if (CS_SX127X < 0) {
                dev_err(&spi->dev, "error sx127x cs gpio: %d\n", CS_SX127X);
                return CS_SX127X;
        }
	ret = gpio_request(CS_SX127X, "cs-sx127x");
        if(ret) {
                printk("error sx127x cs gpio: %d\n", ret);
                return ret;
        }
	gpio_free(CS_SX127X);
    gpio_set_value(CS_SX127X, 1);
//end add
	printk("sx127x cs reset gpio ok\n");

	/*
	 * sx127x should never be referenced in DT without a specific
	 * compatbile string, it is a Linux implementation thing
	 * rather than a description of the hardware.
	 */
	if (spi->dev.of_node && !of_match_device(sx127x_dt_ids, &spi->dev)) {
		dev_err(&spi->dev, "buggy DT: sx127x listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
			!of_match_device(sx127x_dt_ids, &spi->dev));
	}
	printk("spi mode = %4x.\n ",spi->mode);
	/* Allocate driver data */
	sx127x = kzalloc(sizeof(*sx127x), GFP_KERNEL);
	if (!sx127x)
		return -ENOMEM;
	printk("spi mode = %4x.\n ",spi->mode);
	/* Initialize the driver data */
	sx127x->spi = spi;
	spin_lock_init(&sx127x->spi_lock);
	mutex_init(&sx127x->buf_lock);

	INIT_LIST_HEAD(&sx127x->device_entry);
	printk("spi mode = %4x.\n ",spi->mode);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		sx127x->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(sx127x_class, &spi->dev, sx127x->devt,
				    sx127x, "sx127x",
				    spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&sx127x->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	sx127x->speed_hz = spi->max_speed_hz;

	if (status == 0)
		spi_set_drvdata(spi, sx127x);
	else
		kfree(sx127x);
    printk("sx127x ok!\n");
	return status;
}

static int sx127x_remove(struct spi_device *spi)
{
	struct sx127x_data	*sx127x = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&sx127x->spi_lock);
	sx127x->spi = NULL;
	spin_unlock_irq(&sx127x->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&sx127x->device_entry);
	device_destroy(sx127x_class, sx127x->devt);
	clear_bit(MINOR(sx127x->devt), minors);
	if (sx127x->users == 0)
		kfree(sx127x);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver sx127x_spi_driver = {
	.driver = {
		.name =		"sx127x",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(sx127x_dt_ids),
	},
	.probe =	sx127x_probe,
	.remove =	sx127x_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init sx127x_init(void)
{
	int status;

	//printk("***************** fun:%s, line = %d\n", __FUNCTION__, __LINE__);
	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &sx127x_fops);
	if (status < 0)
		return status;

	sx127x_class = class_create(THIS_MODULE, "sx127x");
	if (IS_ERR(sx127x_class)) {
		unregister_chrdev(SPIDEV_MAJOR, sx127x_spi_driver.driver.name);
		return PTR_ERR(sx127x_class);
	}

	status = spi_register_driver(&sx127x_spi_driver);
	if (status < 0) {
		class_destroy(sx127x_class);
		unregister_chrdev(SPIDEV_MAJOR, sx127x_spi_driver.driver.name);
	}
	//printk("***************** fun:%s, line = %d\n", __FUNCTION__, __LINE__);
	return status;
}
module_init(sx127x_init);

static void __exit sx127x_exit(void)
{
	spi_unregister_driver(&sx127x_spi_driver);
	class_destroy(sx127x_class);
	unregister_chrdev(SPIDEV_MAJOR, sx127x_spi_driver.driver.name);
}
module_exit(sx127x_exit);

MODULE_AUTHOR("topeet: lsb");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("spi:spidev");
