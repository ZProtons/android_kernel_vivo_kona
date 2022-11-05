/* Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include "sn100.h"
#include <linux/clk.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/time.h>

/* vivo zhangjiebin add begin, clock will be control by qcommon powermanager */
//NOTE: default status is close
//#define USE_CLK_CONTROL
/* vivo zhangjiebin add end */

/* vivo zhangjiebin add begin, for VDDIO control*/
//NOTE: default status is close
//#define VDDIO_USE_LDO_CONTROL
/* vivo zhangjiebin add end */

/* vivo zhangjiebin add begin, for nfc hardware version check*/
//NOTE: default status is close
//#define NFC_HW_CHECK
/* vivo zhangjiebin add end */

/* vivo xinhaibo add begin, for nfc ic check*/
//#define NFC_HW_DETECT
/* vivo xinhaibo add end */

/* vivo caibinchen add begin, do not enable nfc ven in smt */
//#define NFC_SMT_UNLOAD
/* vivo caibinchen add end */
/* vivo caibinchen add begin, for vddal7 power set*/
#define NFC_L7A_VOL_MIN 1800000
#define NFC_L7A_VOL_MAX 1800000
#define NFC_L7A_VOL_LOAD 1000000
/* vivo caibinchen add begin, for vddal7 power set*/

struct nqx_platform_data {
	unsigned int irq_gpio;
	unsigned int en_gpio;
	/* vivo zhangjiebin add begin, for VDDIO control*/
	//#ifdef VDDIO_USE_LDO_CONTROL
	unsigned int ldo_en_gpio;
	//#endif
	/* vivo zhangjiebin add end*/
	unsigned int clkreq_gpio;
	unsigned int firm_gpio;
	unsigned int ese_gpio;
	const char *clk_src_name;
	/* NFC_CLK pin voting state */
	bool clk_pin_voting;
};

static const struct of_device_id msm_match_table[] = {
	{.compatible = "qcom,sn100"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

#define MAX_BUFFER_SIZE			(320)
#define WAKEUP_SRC_TIMEOUT		(2000)
#define MAX_RETRY_COUNT			3

struct nqx_dev {
	wait_queue_head_t	read_wq;
	struct	mutex		read_mutex;
	struct	mutex		write_mutex;
	struct  mutex		ioctl_mutex;
	struct	i2c_client	*client;
	struct	miscdevice	nqx_device;
	struct	miscdevice	nqx_device_test;
	union  nqx_uinfo	nqx_info;
	/* NFC GPIO variables */
	unsigned int		irq_gpio;
	unsigned int		en_gpio;
	/* vivo zhangjiebin add begin, for VDDIO control*/
	//#ifdef VDDIO_USE_LDO_CONTROL
	unsigned int		ldo_en_gpio;
	//#endif
	/* vivo zhangjiebin add end*/
	unsigned int		firm_gpio;
	unsigned int		clkreq_gpio;
	unsigned int		ese_gpio;
	/* NFC VEN pin state powered by Nfc */
	bool			nfc_ven_enabled;
	/* NFC_IRQ state */
	bool			irq_enabled;
	/* NFC_IRQ wake-up state */
	bool			irq_wake_up;
	spinlock_t		irq_enabled_lock;
	unsigned int		count_irq;
	/* Initial CORE RESET notification */
	unsigned int		core_reset_ntf;
	struct timeval tv_read;
	struct timeval tv_write;
#ifdef USE_CLK_CONTROL
	/* CLK control */
	bool			clk_run;
	struct	clk		*s_clk;
#endif
	/* read buffer*/
	size_t kbuflen;
	u8 *kbuf;
	struct nqx_platform_data *pdata;
	/*determine nfc */
	struct kobject *kobjectDebug;
	/* vivo caibinchen add begin, for vddal7 power set*/
	struct regulator *vddal7;
	/* vivo caibinchen add end, for vddal7 power set*/
};

extern char *get_bbk_board_version(void);
extern unsigned int recoverymode;
extern char *saved_command_line;
#ifdef USE_CLK_CONTROL
/*clock enable function*/
static int nqx_clock_select(struct nqx_dev *nqx_dev);
/*clock disable function*/
static int nqx_clock_deselect(struct nqx_dev *nqx_dev);
#endif

static int nfcc_reboot(struct notifier_block *notifier, unsigned long val,
			void *v);

static struct notifier_block nfcc_notifier = {
	.notifier_call	= nfcc_reboot,
	.next			= NULL,
	.priority		= 0
};

static unsigned int	disable_ctrl;
static struct regulator *disable_vddal7;
/*add a node for load nfc service start*/
static unsigned int   nfc_result;
static struct kobject *nfc_kobject;
/*add a node for load nfc service end*/

/* vivo zhangjiebin add begin, to check board version */
static unsigned int nfc_support;
static unsigned int boardversion_mask;
static unsigned int boardversion_shift;
static unsigned int boardversion_num;
static const char *boardversions[20];
/* vivo zhangjiebin add end */

/* vivo zhoushiren add for PD2011 vddio use ldo control begin*/
const char *vivo_project_name;
static bool vddio_use_ldo_control;
/* vivo zhoushiren add for PD2011 vddio use ldo control end*/
static bool vddio_use_l7_control;

extern unsigned int power_off_charging_mode;

static void nqx_init_stat(struct nqx_dev *nqx_dev)
{
	nqx_dev->count_irq = 0;
}

static void nqx_disable_irq(struct nqx_dev *nqx_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&nqx_dev->irq_enabled_lock, flags);
	if (nqx_dev->irq_enabled) {
		disable_irq_nosync(nqx_dev->client->irq);
		nqx_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&nqx_dev->irq_enabled_lock, flags);
}

/**
 * nqx_enable_irq()
 *
 * Check if interrupt is enabled or not
 * and enable interrupt
 *
 * Return: void
 */
static void nqx_enable_irq(struct nqx_dev *nqx_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&nqx_dev->irq_enabled_lock, flags);
	if (!nqx_dev->irq_enabled) {
		nqx_dev->irq_enabled = true;
		enable_irq(nqx_dev->client->irq);
	}
	spin_unlock_irqrestore(&nqx_dev->irq_enabled_lock, flags);
}

static irqreturn_t nqx_dev_irq_handler(int irq, void *dev_id)
{
	struct nqx_dev *nqx_dev = dev_id;
	unsigned long flags;

	if (device_may_wakeup(&nqx_dev->client->dev))
		pm_wakeup_event(&nqx_dev->client->dev, WAKEUP_SRC_TIMEOUT);

	nqx_disable_irq(nqx_dev);
	spin_lock_irqsave(&nqx_dev->irq_enabled_lock, flags);
	nqx_dev->count_irq++;
	spin_unlock_irqrestore(&nqx_dev->irq_enabled_lock, flags);
	wake_up(&nqx_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t nfc_read(struct file *filp, char __user *buf,
					size_t count, loff_t *offset)
{
	struct nqx_dev *nqx_dev = filp->private_data;
	unsigned char *tmp = NULL;
	int ret;
	int irq_gpio_val = 0;

	if (!nqx_dev) {
		ret = -ENODEV;
		goto out;
	}

	if (count > nqx_dev->kbuflen)
		count = nqx_dev->kbuflen;

	dev_dbg(&nqx_dev->client->dev, "%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&nqx_dev->read_mutex);

	irq_gpio_val = gpio_get_value(nqx_dev->irq_gpio);
	if (irq_gpio_val == 0) {
		if (filp->f_flags & O_NONBLOCK) {
			dev_err(&nqx_dev->client->dev, ":f_falg has O_NONBLOCK. EAGAIN\n");
			ret = -EAGAIN;
			goto err;
		}
		while (1) {
			ret = 0;
			if (!nqx_dev->irq_enabled) {
				nqx_dev->irq_enabled = true;
				enable_irq(nqx_dev->client->irq);
			}
			if (!gpio_get_value(nqx_dev->irq_gpio)) {
				ret = wait_event_interruptible(nqx_dev->read_wq,
					!nqx_dev->irq_enabled);
			}
			if (ret)
				goto err;
			nqx_disable_irq(nqx_dev);

			if (gpio_get_value(nqx_dev->irq_gpio))
				break;
			dev_err_ratelimited(&nqx_dev->client->dev, "gpio is low, no need to read data\n");
		}
	}

	tmp = nqx_dev->kbuf;
	if (!tmp) {
		dev_err(&nqx_dev->client->dev, "%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		goto err;
	}
	memset(tmp, 0x00, count);

	do_gettimeofday(&nqx_dev->tv_read);
	/* Read data */
	ret = i2c_master_recv(nqx_dev->client, tmp, count);
	if (ret < 0) {
		dev_err(&nqx_dev->client->dev, "%s: i2c_master_recv returned %d\n", __func__, ret);
		goto err;
	}
	if (ret > count) {
		dev_err(&nqx_dev->client->dev,
			"%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		ret = -EIO;
		goto err;
	}

#ifdef NFC_KERNEL_BU
	dev_dbg(&nqx_dev->client->dev, "%s : NfcNciRx %x %x %x\n",
		__func__, tmp[0], tmp[1], tmp[2]);
#endif

	if (copy_to_user(buf, tmp, ret)) {
		dev_warn(&nqx_dev->client->dev, "%s : failed to copy to user space\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	mutex_unlock(&nqx_dev->read_mutex);

	return ret;
err:
	mutex_unlock(&nqx_dev->read_mutex);
out:
	return ret;
}

static ssize_t nfc_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct nqx_dev *nqx_dev = filp->private_data;
	char *tmp = NULL;
	int ret = 0;
	char val[1] = {0};

	if (!nqx_dev) {
		ret = -ENODEV;
		goto out;
	}
	if (count > nqx_dev->kbuflen) {
		dev_err(&nqx_dev->client->dev, "%s: out of memory\n", __func__);
		ret = -ENOMEM;
		goto out;
	}

	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp)) {
		dev_err(&nqx_dev->client->dev, "%s: memdup_user failed\n", __func__);
		ret = PTR_ERR(tmp);
		goto out;
	}
	do_gettimeofday(&nqx_dev->tv_write);
	mutex_lock(&nqx_dev->write_mutex);

	if ((nqx_dev->tv_write.tv_sec * 1000 + nqx_dev->tv_write.tv_usec / 1000) - (nqx_dev->tv_read.tv_sec * 1000 + nqx_dev->tv_read.tv_usec / 1000) > 700) {
		ret = i2c_master_send(nqx_dev->client, val, 0);
		if (ret < 0)
			pr_err("%s: write test ret = %d\n", __func__, ret);
		else
			pr_err("%s: write test ret = %d\n", __func__, ret);
		usleep_range(5000, 5100);
	}

	ret = i2c_master_send(nqx_dev->client, tmp, count);
	if (ret != count) {
		dev_err(&nqx_dev->client->dev, "%s: failed to write %d\n", __func__, ret);
		ret = -EIO;
		goto out_free;
	}
#ifdef NFC_KERNEL_BU
	dev_dbg(&nqx_dev->client->dev,
			"%s : i2c-%d: NfcNciTx %x %x %x\n",
			__func__, iminor(file_inode(filp)),
			tmp[0], tmp[1], tmp[2]);
#endif
	usleep_range(1000, 1100);
out_free:
	kfree(tmp);
	mutex_unlock(&nqx_dev->write_mutex);
out:
	return ret;
}

/**
 * nqx_standby_write()
 * @buf:       pointer to data buffer
 * @len:       # of bytes need to transfer
 *
 * write data buffer over I2C and retry
 * if NFCC is in standby mode
 *
 * Return: # of bytes written or -ve value in case of error
 */
static int nqx_standby_write(struct nqx_dev *nqx_dev,
				const unsigned char *buf, size_t len)
{
	int ret = -EINVAL;
	int retry_cnt;

	for (retry_cnt = 1; retry_cnt <= MAX_RETRY_COUNT; retry_cnt++) {
		ret = i2c_master_send(nqx_dev->client, buf, len);
		if (ret < 0) {
			dev_err(&nqx_dev->client->dev,
				"%s: write failed, Maybe in Standby Mode - Retry(%d)\n",
				 __func__, retry_cnt);
			usleep_range(1000, 1100);
		} else if (ret == len)
			break;
	}

	return ret;
}

#ifdef NFC_SMT_UNLOAD
int is_normal_boot(void)
{
	if (saved_command_line == NULL)
		return 2;
	if (strnstr(saved_command_line, "sendAT", strlen(saved_command_line)) != NULL)
		return 0;
	return 1;
}
#endif

static int vddal7_power_on(struct nqx_dev *nqx_dev)
{
	int ret = 0;
	ret = regulator_set_load(nqx_dev->vddal7, NFC_L7A_VOL_LOAD);
	if (ret < 0) {
		dev_err(&nqx_dev->client->dev, "Unable to load vddal7 LPM:%d\n", ret);
		goto disable_vddal7;
	}
	ret = regulator_set_voltage(nqx_dev->vddal7, NFC_L7A_VOL_MIN, NFC_L7A_VOL_MAX);
	if (ret) {
		dev_err(&nqx_dev->client->dev, "Unable to set vddal7_power_on 2 voltage:%d\n", ret);
		goto put_vddal7;
	}
	ret = regulator_enable(nqx_dev->vddal7);
	if (ret) {
		dev_err(&nqx_dev->client->dev, "Unable to enable vddal7_power_on 3 voltage:%d\n", ret);
		goto unset_vddal7;
	}
	ret = regulator_get_voltage(nqx_dev->vddal7);
	if (ret < 0) {
		dev_err(&nqx_dev->client->dev, "Unable to get vddal7_power_on 4 voltage:%d\n", ret);
		goto unset_vddal7;
	}
	printk("nfc vddal7_power_on voltage is %d\n", ret);
	return ret;
unset_vddal7:
	ret = regulator_set_voltage(nqx_dev->vddal7, 0, NFC_L7A_VOL_MAX);
	if (ret)
		dev_err(&nqx_dev->client->dev, "Unable to set (0) voltage for vddal7:%d\n", ret);
put_vddal7:
	ret = regulator_set_load(nqx_dev->vddal7, 0);
	if (ret < 0)
		dev_err(&nqx_dev->client->dev, "Unable to set load vddal7\n", ret);
disable_vddal7:
	ret = regulator_disable(nqx_dev->vddal7);
	if (ret)
		dev_err(&nqx_dev->client->dev, "Unable to disable vddal7:%d\n", ret);
	return ret;
}

static int disable_vddal7_power_off(struct regulator *disable_vddal7)
{
	int ret = 0;
	if (disable_vddal7 == NULL)
		return ret;
	ret = regulator_set_load(disable_vddal7, 0);
	if (ret < 0)
		pr_err("%s: Unable to set load vddal7:%d\n", __func__, ret);
	ret = regulator_set_voltage(disable_vddal7, 0, NFC_L7A_VOL_MAX);
	if (ret)
		pr_err("%s: Unable to set (0) voltage for vddal7:%d\n", __func__, ret);
	ret = regulator_disable(disable_vddal7);
	if (ret)
		pr_err("%s: Unable to disable vddal7:%d\n", __func__, ret);
	return ret;
}

//vivo caizhaohui add begin
static bool isVddal7_Supply(void)
{
	if (!strncmp(vivo_project_name, "PD2025", 6)) {
		return true;
	} else if (!strncmp(vivo_project_name, "PD2024", 6) && vddio_use_l7_control) {
		return true;
	} else if (!strncmp(vivo_project_name, "PD2059", 6)) {
		return true;
    } else if (!strncmp(vivo_project_name, "PD2046F_EX", 6)) {
		return true;
	} else {
		return false;
	}
}
//vivo caizhaohui add end

/*
 * Power management of the eSE
 * NFC & eSE ON : NFC_EN high and eSE_pwr_req high.
 * NFC OFF & eSE ON : NFC_EN high and eSE_pwr_req high.
 * NFC OFF & eSE OFF : NFC_EN low and eSE_pwr_req low.
 */
static int nqx_ese_pwr(struct nqx_dev *nqx_dev, unsigned long int arg)
{
	int r = -1;
	const unsigned char svdd_off_cmd_warn[] =  {0x2F, 0x31, 0x01, 0x01};
	const unsigned char svdd_off_cmd_done[] =  {0x2F, 0x31, 0x01, 0x00};

#ifdef NFC_SMT_UNLOAD
	if (is_normal_boot() == 0) {
		dev_err(&nqx_dev->client->dev, "in smt mode\n");
		return r;
	}
#endif

	if (!gpio_is_valid(nqx_dev->en_gpio)) {
		dev_err(&nqx_dev->client->dev, "%s: en_gpio is not valid\n", __func__);
		return -EINVAL;
	}
	if (arg == 0) {
		/*
		 * We want to power on the eSE and to do so we need the
		 * eSE_pwr_req pin and the NFC_EN pin to be high
		 */
			/**
			 * Let's store the NFC_EN pin state
			 * only if the eSE is not yet on
			 */
			nqx_dev->nfc_ven_enabled = gpio_get_value(nqx_dev->en_gpio);
			if (!nqx_dev->nfc_ven_enabled) {
				usleep_range(5000, 5100);
				gpio_set_value(nqx_dev->en_gpio, 1);
				/* hardware dependent delay */
				usleep_range(1000, 1100);
			}

		r = 0;


	} else if (arg == 1) {
		if (nqx_dev->nfc_ven_enabled &&
			((nqx_dev->nqx_info.info.chip_type == NFCC_NQ_220) ||
			(nqx_dev->nqx_info.info.chip_type == NFCC_PN66T))) {
			/**
			 * Let's inform the CLF we're
			 * powering off the eSE
			 */
			r = nqx_standby_write(nqx_dev, svdd_off_cmd_warn, sizeof(svdd_off_cmd_warn));
			if (r < 0) {
				dev_err(&nqx_dev->client->dev, "%s: write failed after max retry\n", __func__);
				return -ENXIO;
			}
			dev_dbg(&nqx_dev->client->dev, "%s: svdd_off_cmd_warn sent\n", __func__);

			/* let's power down the eSE */
			gpio_set_value(nqx_dev->ese_gpio, 0);
			dev_dbg(&nqx_dev->client->dev, "%s: nqx_dev->ese_gpio set to 0\n", __func__);

			/**
			 * Time needed for the SVDD capacitor
			 * to get discharged
			 */
			usleep_range(8000, 8100);

			/* Let's inform the CLF the eSE is now off */
			r = nqx_standby_write(nqx_dev, svdd_off_cmd_done, sizeof(svdd_off_cmd_done));
			if (r < 0) {
				dev_err(&nqx_dev->client->dev, "%s: write failed after max retry\n", __func__);
				return -ENXIO;
			}
			dev_dbg(&nqx_dev->client->dev, "%s: svdd_off_cmd_done sent\n", __func__);
		}
		if (!nqx_dev->nfc_ven_enabled) {
			/* hardware dependent delay */
			usleep_range(1000, 1100);
			dev_dbg(&nqx_dev->client->dev, "disabling en_gpio\n");
			gpio_set_value(nqx_dev->en_gpio, 0);
		}
		r = 0;
	} else if (arg == 3) {
		r = gpio_get_value(nqx_dev->en_gpio);
	}

	return r;
}

static int nfc_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct nqx_dev *nqx_dev = container_of(filp->private_data,
				struct nqx_dev, nqx_device);

	filp->private_data = nqx_dev;
	nqx_init_stat(nqx_dev);

	dev_dbg(&nqx_dev->client->dev,
			"%s: %d,%d\n", __func__, imajor(inode), iminor(inode));
	return ret;
}

/*add a node for load nfc service start*/
static ssize_t nfc_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (nfc_result == 1)
		return snprintf(buf, 2, "%d", 1);
	else
		return snprintf(buf, 2, "%d", 0);
}

static ssize_t nfc_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return nfc_result;
}

static DEVICE_ATTR(nfc_enable, 0664, nfc_enable_show, nfc_enable_store);

static struct attribute *nfc_attributes[] = {
	&dev_attr_nfc_enable.attr,
	NULL
};

static const struct attribute_group nfc_attr_group = {
	.attrs = nfc_attributes,
};
/*add a node for load nfc service end*/

/*
 * nfc_ioctl_power_states() - power control
 * @filp:	pointer to the file descriptor
 * @arg:	mode that we want to move to
 *
 * Device power control. Depending on the arg value, device moves to
 * different states
 * (arg = 0): NFC_ENABLE	GPIO = 0, FW_DL GPIO = 0
 * (arg = 1): NFC_ENABLE	GPIO = 1, FW_DL GPIO = 0
 * (arg = 2): FW_DL GPIO = 1
 *
 * Return: -ENOIOCTLCMD if arg is not supported, 0 in any other case
 */
static int nfc_ioctl_power_states(struct file *filp, unsigned long arg)
{
	int r = 0;
	struct nqx_dev *nqx_dev = filp->private_data;

	mutex_lock(&nqx_dev->ioctl_mutex);

#ifdef NFC_SMT_UNLOAD
	if (is_normal_boot() == 0) {
		dev_err(&nqx_dev->client->dev, "in smt mode\n");
		return r;
	}
#endif

	printk("nfc printk ioctrl en_gpio is %d", gpio_get_value(nqx_dev->en_gpio));
	if (arg == 0) {
		/*
		 * We are attempting a hardware reset so let us disable
		 * interrupts to avoid spurious notifications to upper
		 * layers.
		 */
		nqx_disable_irq(nqx_dev);
		dev_dbg(&nqx_dev->client->dev,
			"gpio_set_value disable: %s: info: %p\n",
				__func__, nqx_dev);
		if (gpio_is_valid(nqx_dev->firm_gpio)) {
			dev_err(&nqx_dev->client->dev, "nfc set firm_gpio to low :%d\n", __LINE__);
			gpio_set_value(nqx_dev->firm_gpio, 0);
			usleep_range(10000, 10100);
		}

		if (gpio_is_valid(nqx_dev->ese_gpio)) {
			if (!gpio_get_value(nqx_dev->ese_gpio)) {
				dev_err(&nqx_dev->client->dev, "nfc set en_gpio to low :%d\n", __LINE__);
				gpio_set_value(nqx_dev->en_gpio, 0);
				usleep_range(10000, 10100);
			} else {
				dev_dbg(&nqx_dev->client->dev, "keeping en_gpio high\n");
			}
		} else {
			dev_err(&nqx_dev->client->dev, "nfc set en_gpio to low :%d\n", __LINE__);
			gpio_set_value(nqx_dev->en_gpio, 0);
			usleep_range(10000, 10100);
		}

		#ifdef USE_CLK_CONTROL
		if (nqx_dev->pdata->clk_pin_voting) {
			r = nqx_clock_deselect(nqx_dev);
			if (r < 0)
				dev_err(&nqx_dev->client->dev, "unable to disable clock\n");
		}
		#endif

		nqx_dev->nfc_ven_enabled = false;
	} else if (arg == 1) {
		nqx_enable_irq(nqx_dev);
		dev_dbg(&nqx_dev->client->dev,
			"gpio_set_value enable: %s: info: %p\n",
				__func__, nqx_dev);
		if (gpio_is_valid(nqx_dev->firm_gpio)) {
			dev_err(&nqx_dev->client->dev, "nfc set firm_gpio to low :%d\n", __LINE__);
			gpio_set_value(nqx_dev->firm_gpio, 0);
			usleep_range(10000, 10100);
		}

		if (gpio_get_value(nqx_dev->en_gpio)) {
			dev_dbg(&nqx_dev->client->dev, "VEN gpio already high\n");
		} else {
			dev_err(&nqx_dev->client->dev, "nfc set en_gpio to high :%d\n", __LINE__);
			gpio_set_value(nqx_dev->en_gpio, 1);
			usleep_range(10000, 10100);
		}

		#ifdef USE_CLK_CONTROL
		if (nqx_dev->pdata->clk_pin_voting) {
			r = nqx_clock_select(nqx_dev);
			if (r < 0)
				dev_err(&nqx_dev->client->dev, "unable to enable clock\n");
		}
		#endif

		nqx_dev->nfc_ven_enabled = true;
	} else if (arg == 2) {
		/*
		 * We are switching to Dowload Mode, toggle the enable pin
		 * in order to set the NFCC in the new mode
		 */
		if (gpio_is_valid(nqx_dev->ese_gpio)) {
			if (gpio_get_value(nqx_dev->ese_gpio)) {
				dev_err(&nqx_dev->client->dev, "FW download forbidden while ese is on\n");

				/* vivo zhangjiebin add begin, to release mutex lock */
				mutex_unlock(&nqx_dev->ioctl_mutex);
				/* vivo zhangjiebin add end */

				return -EBUSY; /* Device or resource busy */
			}
		}
		gpio_set_value(nqx_dev->en_gpio, 1);
		usleep_range(10000, 10100);
		if (gpio_is_valid(nqx_dev->firm_gpio)) {
			gpio_set_value(nqx_dev->firm_gpio, 1);
			usleep_range(10000, 10100);
		}
		gpio_set_value(nqx_dev->en_gpio, 0);
		usleep_range(10000, 10100);
		gpio_set_value(nqx_dev->en_gpio, 1);
		usleep_range(10000, 10100);
	} else if (arg == 4) {
		/*
		 * Setting firmware download gpio to HIGH for SN100U
		 * before FW download start
		 */
		dev_dbg(&nqx_dev->client->dev, "SN100 fw gpio HIGH\n");
		if (gpio_is_valid(nqx_dev->firm_gpio)) {
			gpio_set_value(nqx_dev->firm_gpio, 1);
			usleep_range(10000, 10100);
		} else
			dev_err(&nqx_dev->client->dev, "firm_gpio is invalid\n");
	} else if (arg == 6) {
		/*
		 * Setting firmware download gpio to LOW for SN100U
		 * FW download finished
		 */
		dev_dbg(&nqx_dev->client->dev, "SN100 fw gpio LOW\n");
		if (gpio_is_valid(nqx_dev->firm_gpio)) {
			gpio_set_value(nqx_dev->firm_gpio, 0);
			usleep_range(10000, 10100);
		} else {
			dev_err(&nqx_dev->client->dev, "firm_gpio is invalid\n");
		}
	} else {
		r = -ENOIOCTLCMD;
	}

	mutex_unlock(&nqx_dev->ioctl_mutex);
	return r;
}

#ifdef CONFIG_COMPAT
static long nfc_compat_ioctl(struct file *pfile, unsigned int cmd,
				unsigned long arg)
{
	long r = 0;

	arg = (compat_u64)arg;
	switch (cmd) {
	case NFC_SET_PWR:
		nfc_ioctl_power_states(pfile, arg);
		break;
	case ESE_SET_PWR:
		nqx_ese_pwr(pfile->private_data, arg);
		break;
	case ESE_GET_PWR:
		nqx_ese_pwr(pfile->private_data, 3);
		break;
	case SET_RX_BLOCK:
		break;
	case SET_EMULATOR_TEST_POINT:
		break;
	default:
		r = -ENOTTY;
	}
	return r;
}
#endif

/*
 * nfc_ioctl_core_reset_ntf()
 * @filp:       pointer to the file descriptor
 *
 * Allows callers to determine if a CORE_RESET_NTF has arrived
 *
 * Return: the value of variable core_reset_ntf
 */
static int nfc_ioctl_core_reset_ntf(struct file *filp)
{
	struct nqx_dev *nqx_dev = filp->private_data;

	dev_dbg(&nqx_dev->client->dev, "%s: returning = %d\n", __func__,
		nqx_dev->core_reset_ntf);
	return nqx_dev->core_reset_ntf;
}

/*
 * Inside nfc_ioctl_nfcc_info
 *
 * @brief   nfc_ioctl_nfcc_info
 *
 * Check the NQ Chipset and firmware version details
 */
static unsigned int nfc_ioctl_nfcc_info(struct file *filp, unsigned long arg)
{
	unsigned int r = 0;
	struct nqx_dev *nqx_dev = filp->private_data;

	r = nqx_dev->nqx_info.i;
	dev_dbg(&nqx_dev->client->dev, "nqx nfc : %s r = %d\n", __func__, r);

	return r;
}

static long nfc_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	int r = 0;

	switch (cmd) {
	case NFC_SET_PWR:
		r = nfc_ioctl_power_states(pfile, arg);
		break;
	case ESE_SET_PWR:
		r = nqx_ese_pwr(pfile->private_data, arg);
		break;
	case ESE_GET_PWR:
		r = nqx_ese_pwr(pfile->private_data, 3);
		break;
	case SET_RX_BLOCK:
		break;
	case SET_EMULATOR_TEST_POINT:
		break;
	case NFCC_INITIAL_CORE_RESET_NTF:
		r = nfc_ioctl_core_reset_ntf(pfile);
		break;
	case NFCC_GET_INFO:
		r = nfc_ioctl_nfcc_info(pfile, arg);
		break;
	case NFC_GET_PLATFORM_TYPE:
		r = PLATFORM_IF_I2C;
	default:
		r = -ENOIOCTLCMD;
	}
	return r;
}

static const struct file_operations nfc_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read  = nfc_read,
	.write = nfc_write,
	.open = nfc_open,
	.unlocked_ioctl = nfc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nfc_compat_ioctl
#endif
};

#ifdef NFC_HW_CHECK
/* Check for availability of NQ_ NFC controller hardware */
static int nfcc_hw_check(struct i2c_client *client, struct nqx_dev *nqx_dev)
{
	int ret = 0;
	unsigned int enable_gpio = nqx_dev->en_gpio;
	unsigned int firm_gpio = nqx_dev->firm_gpio;
	//unsigned char raw_nci_reset_cmd[] =  {0x20, 0x00, 0x01, 0x00};
	//unsigned char nci_reset_rsp[6];

	unsigned char raw_fw_get_version_cmd[] =  {0x00, 0x04, 0xF1, 0x00, 0x00, 0x00, 0x6E, 0xEF};
	unsigned char fw_get_version_rsp[14];

	gpio_set_value(firm_gpio, 1);
	msleep(10);
	/* making sure that the NFCC starts in a clean state. */
	gpio_set_value(enable_gpio, 0);/* ULPM: Disable */
	/* hardware dependent delay */
	msleep(10);
	gpio_set_value(enable_gpio, 1);/* HPD : Enable*/
	/* hardware dependent delay */
	msleep(10);
	/* send get FW Version CMD */
	ret = i2c_master_send(client, raw_fw_get_version_cmd,
						sizeof(raw_fw_get_version_cmd));
	if (ret < 0) {
		dev_err(&client->dev, "%s: - i2c_master_send Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	pr_err("%s: raw_fw_get_version success\n", __func__);
	/* hardware dependent delay */
	msleep(30);
	/* Read Response of get fw version */
	ret = i2c_master_recv(client, fw_get_version_rsp,
						sizeof(fw_get_version_rsp));
	dev_err(&client->dev,
		"%s: - nq - firm cmd answer : NfcNciRx %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n",
			__func__, fw_get_version_rsp[0], fw_get_version_rsp[1],
			fw_get_version_rsp[2], fw_get_version_rsp[3], fw_get_version_rsp[4], fw_get_version_rsp[5], fw_get_version_rsp[6],  
			fw_get_version_rsp[7], fw_get_version_rsp[8], fw_get_version_rsp[9], fw_get_version_rsp[10], fw_get_version_rsp[11],
			fw_get_version_rsp[12], fw_get_version_rsp[13]);
	if (ret < 0) {
		dev_err(&client->dev, "%s: - i2c_master_recv Error\n", __func__);
		goto err_nfcc_hw_check;
	}

	ret = 0;
	goto done;

err_nfcc_hw_check:
	ret = -ENXIO;
	dev_err(&client->dev, "%s: - NFCC HW not available\n", __func__);
done:
	gpio_set_value(firm_gpio, 0);
	gpio_set_value(enable_gpio, 0);/* ULPM: Disable */
	return ret;
}
#endif


#ifdef NFC_HW_DETECT
static int nfcc_hw_detect(struct nqx_dev *nqx_dev)
{
	int ret = 0;
	char val[1] = {0};
	int count = 0;
	unsigned int enable_gpio = nqx_dev->en_gpio;
	unsigned int firm_gpio = nqx_dev->firm_gpio;

	gpio_set_value(firm_gpio, 1);
	msleep(10);
	do {
		gpio_set_value(enable_gpio, 0);
		/* hardware dependent delay */
		msleep(10);
		gpio_set_value(enable_gpio, 1);
		msleep(10);
		ret = i2c_master_send(nqx_dev->client, val, 0);
		if (ret < 0)
			dev_err(&nqx_dev->client->dev, "%s:returned %d;count %d\n", __func__, ret, count);
		else
			dev_err(&nqx_dev->client->dev, "%s:returned %d;count %d\n", __func__, ret, count);
	} while ((count++ < 3) && (ret < 0));

	gpio_set_value(firm_gpio, 0);
	gpio_set_value(enable_gpio, 0);
	msleep(20);
	return ret;
}
#endif

#ifdef USE_CLK_CONTROL
/*
 * Routine to enable clock.
 * this routine can be extended to select from multiple
 * sources based on clk_src_name.
 */
static int nqx_clock_select(struct nqx_dev *nqx_dev)
{
	int r = 0;

	nqx_dev->s_clk = clk_get(&nqx_dev->client->dev, "ref_clk");
	if (nqx_dev->s_clk == NULL)
		goto err_clk;

	if (nqx_dev->clk_run == false)
		r = clk_prepare_enable(nqx_dev->s_clk);

	if (r)
		goto err_clk;

	nqx_dev->clk_run = true;

	return r;

err_clk:
	r = -1;
	return r;
}

/*
 * Routine to disable clocks
 */
static int nqx_clock_deselect(struct nqx_dev *nqx_dev)
{
	int r = -1;

	if (nqx_dev->s_clk != NULL) {
		if (nqx_dev->clk_run == true) {
			clk_disable_unprepare(nqx_dev->s_clk);
			nqx_dev->clk_run = false;
		}
		return 0;
	}
	return r;
}
#endif

static int nfc_parse_dt(struct device *dev, struct nqx_platform_data *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;
	/* vivo zhangjiebin add begin, to check board version */
	int i = 0;
	int ret = 0;
	char *board_version = NULL;

	ret = of_property_read_u32(np, "vivo,nfc_support", &nfc_support);
	if (ret < 0) {
		dev_err(dev, "Failure reading vivo,nfc_support, ret = %d\n", ret);
		nfc_support = 0;
	}
	printk(KERN_DEBUG "nfc nfc_support:%d\n", nfc_support);

	ret = of_property_read_u32(np, "vivo,boardversion_shift", &boardversion_shift);
	if (ret < 0) {
		dev_err(dev, "Failure reading vivo,boardversion_shift, ret = %d\n", ret);
		boardversion_shift = 0;
	}
	printk(KERN_DEBUG "nfc boardversion_shift:%d\n", boardversion_shift);

	ret = of_property_read_u32(np, "vivo,boardversion_mask", &boardversion_mask);
	if (ret < 0) {
		dev_err(dev, "Failure reading vivo,boardversion_mask, ret = %d\n", ret);
		boardversion_mask = 0;
	}
	printk(KERN_DEBUG "nfc boardversion_mask:%d\n", boardversion_mask);

	ret = of_property_read_u32(np, "vivo,boardversion_num", &boardversion_num);
	if (ret < 0) {
		dev_err(dev, "Failure reading vivo,boardversion_num, ret = %d\n", ret);
		boardversion_num = 0;
	}
	printk(KERN_DEBUG "nfc boardversion_num:%d\n", boardversion_num);

	if (boardversion_num > 0) {
		ret = of_property_read_string_array(np, "vivo,boardversions", boardversions, boardversion_num);
		if (ret < 0)
			dev_err(dev, "Failure reading vivo,boardversions, ret = %d\n", ret);

		for (i = 0; i < boardversion_num; i++)
			printk(KERN_DEBUG "nfc :%s\n", boardversions[i]);
	}
	/* vivo zhangjiebin add end */

    /* vivo zhoushiren add for PD2011 vddio use ldo control begin*/
	ret = of_property_read_string(np, "vivo,project-name", &vivo_project_name);
	if (ret < 0) {
		printk("%s:vivo,project-name property do not find\n", __func__);
		vivo_project_name = "default";
	}
	if (!strncmp(vivo_project_name, "PD2011", 6)) {

		board_version = get_bbk_board_version();
		printk("%s: board_version is %s\n", __func__, board_version);
		if (!strncmp(&(board_version[2]), "1", 1)) {
			printk("%s: this is old board, vddio not use to control\n", __func__);
			vddio_use_ldo_control = false;
		} else {
			printk("%s: this is new board, vddio use to control\n", __func__);
			vddio_use_ldo_control = true;
		}
	} else {
		vddio_use_ldo_control = false;
	}
	if (!strncmp(vivo_project_name, "PD2024", 6)) {
		board_version = get_bbk_board_version();
		printk("%s: nfc board_version is %s\n", __func__, board_version);
		if (!strncmp(&(board_version[4]), "0", 1)) {
			vddio_use_l7_control = true;
		} else {
			vddio_use_l7_control = false;
		}
	} else {
		vddio_use_l7_control = false;
	}

	if (strncmp(vivo_project_name, "PD2011", 6) == 0 && vddio_use_ldo_control == true)
		pdata->en_gpio = of_get_named_gpio(np, "qcom,nq-ven_new", 0);
	else if (strncmp(vivo_project_name, "PD2024", 6) == 0 && vddio_use_l7_control == true)
		pdata->en_gpio = of_get_named_gpio(np, "qcom,nq-ven_new", 0);
	else
		pdata->en_gpio = of_get_named_gpio(np, "qcom,nq-ven", 0);
	/* vivo zhoushiren add for PD2011 vddio use ldo control end*/
	if ((!gpio_is_valid(pdata->en_gpio)))
		return -EINVAL;
	disable_ctrl = pdata->en_gpio;

	/* vivo zhangjiebin add begin, for VDDIO control*/
	if (vddio_use_ldo_control) {
		pdata->ldo_en_gpio = of_get_named_gpio(np, "qcom,nq-ldo_en", 0);
		if ((!gpio_is_valid(pdata->ldo_en_gpio)))
			return -EINVAL;
		printk("%s: pdata->ldo_en_gpio is %d\n", __func__, pdata->ldo_en_gpio);
	}
	/* vivo zhangjiebin add end*/

	pdata->irq_gpio = of_get_named_gpio(np, "qcom,nq-irq", 0);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;

	pdata->firm_gpio = of_get_named_gpio(np, "qcom,nq-firm", 0);
	if (!gpio_is_valid(pdata->firm_gpio)) {
		dev_warn(dev,
			"FIRM GPIO <OPTIONAL> error getting from OF node\n");
		pdata->firm_gpio = -EINVAL;
	}

	pdata->ese_gpio = of_get_named_gpio(np, "qcom,nq-esepwr", 0);
	if (!gpio_is_valid(pdata->ese_gpio)) {
		dev_warn(dev,
			"ese GPIO <OPTIONAL> error getting from OF node\n");
		pdata->ese_gpio = -EINVAL;
	}

	if (of_property_read_string(np, "qcom,clk-src", &pdata->clk_src_name))
		pdata->clk_pin_voting = false;
	else
		pdata->clk_pin_voting = true;

	pdata->clkreq_gpio = of_get_named_gpio(np, "qcom,nq-clkreq", 0);
	if (!gpio_is_valid(pdata->clkreq_gpio)) {
		dev_warn(dev, "clkreq GPIO <OPTIONAL> error getting from OF node\n");
		pdata->clkreq_gpio = -EINVAL;
	}
	return r;
}

static inline int gpio_input_init(const struct device * const dev,
			const int gpio, const char * const gpio_name)
{
	int r = gpio_request(gpio, gpio_name);

	if (r) {
		dev_err(dev, "unable to request gpio [%d]\n", gpio);
		return r;
	}

	r = gpio_direction_input(gpio);
	if (r)
		dev_err(dev, "unable to set direction for gpio [%d]\n", gpio);

	return r;
}

static int nqx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	char *board_version = NULL;
	int r = 0;
	/* vivo zhangjiebin add begin, to check board version */
	int i = 0;
	/* vivo zhangjiebin add end */
	int irqn = 0;
	struct nqx_platform_data *platform_data;
	struct nqx_dev *nqx_dev;

	dev_err(&client->dev, "%s: enter\n", __func__);
	dev_err(&client->dev, "nfc: power_off_charging_mode=%d recoverymode=%d\n", power_off_charging_mode, recoverymode);
	disable_vddal7  = NULL;
	nfc_kobject = kobject_create_and_add("nfc", NULL);
	if (nfc_kobject == NULL) {
		pr_err("%s: nfc node create error\n", __func__);
		return -ENOMEM;
	}

	if (sysfs_create_group(nfc_kobject, &nfc_attr_group)) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		kobject_put(nfc_kobject);
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(struct nqx_platform_data), GFP_KERNEL);
		if (!platform_data) {
			r = -ENOMEM;
			goto err_platform_data;
		}
		r = nfc_parse_dt(&client->dev, platform_data);
		if (r)
			goto err_free_data;
	} else
		platform_data = client->dev.platform_data;

	dev_dbg(&client->dev,
		"%s, inside nfc-nci flags = %x\n",
		__func__, client->flags);

	/* vivo zhangjiebin add begin, to check board version */
	if (nfc_support == 1) {
		if (boardversion_num == 0) {
				dev_err(&client->dev, "The machine has NFC HW\n");
				nfc_result = 1;
		} else {
			board_version = get_bbk_board_version();
			dev_err(&client->dev, " board_version is: %s\n", board_version);
			for (i = 0; i < boardversion_num; i++) {
				//printk(KERN_INFO "nfc:%s,%s\n", boardversions[i],&(board_version[boardversion_shift]));
				if (strncmp(boardversions[i], &(board_version[boardversion_shift]), boardversion_mask) == 0) {
					dev_err(&client->dev, "The machine has NFC HW\n");
					nfc_result = 1;
					break;
				}
			}
			if (i == boardversion_num) {
				dev_err(&client->dev, "The machine hasn't NFC HW\n");
				nfc_result = 0;
			}
		}
	} else {
		dev_err(&client->dev, "The machine hasn't NFC HW\n");
		nfc_result = 0;
	}
	/* vivo zhangjiebin add end */

	if (platform_data == NULL) {
		dev_err(&client->dev, "%s: failed\n", __func__);
		r = -ENODEV;
		goto err_platform_data;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: need I2C_FUNC_I2C\n", __func__);
		r = -ENODEV;
		goto err_free_data;
	}

	nqx_dev = kzalloc(sizeof(*nqx_dev), GFP_KERNEL);
	if (nqx_dev == NULL) {
		r = -ENOMEM;
		goto err_free_data;
	}
	nqx_dev->client = client;
	nqx_dev->kbuflen = MAX_BUFFER_SIZE;
	nqx_dev->kbuf = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!nqx_dev->kbuf) {
		dev_err(&client->dev,
			"failed to allocate memory for nqx_dev->kbuf\n");
		r = -ENOMEM;
		goto err_free_dev;
	}

	if (gpio_is_valid(platform_data->en_gpio)) {
		r = gpio_request(platform_data->en_gpio, "nfc_reset_gpio");
		if (r) {
			dev_err(&client->dev,
			"%s: unable to request nfc reset gpio [%d]\n",
				__func__, platform_data->en_gpio);
			goto err_mem;
		}
		if (!vddio_use_ldo_control && !isVddal7_Supply()) {
			if ((power_off_charging_mode == 1) || (recoverymode == 1))
				r = gpio_direction_output(platform_data->en_gpio, 1);
			else
				r = gpio_direction_output(platform_data->en_gpio, 0);
		} else {
			r = gpio_direction_output(platform_data->en_gpio, 0);
		}
		if (r) {
			dev_err(&client->dev,
				"%s: unable to set direction for nfc reset gpio [%d]\n",
					__func__, platform_data->en_gpio);
			goto err_en_gpio;
		}
	} else {
		dev_err(&client->dev, "%s: nfc reset gpio not provided\n", __func__);
		goto err_mem;
	}
	/* vivo caibinchen add begin, for vddal7 power set*/
	if (isVddal7_Supply()) {
		nqx_dev->vddal7 = devm_regulator_get(&client->dev, "vddal7");
		if (IS_ERR(nqx_dev->vddal7)) {
			dev_err(&client->dev, "%s: unable to get vddal7 supply\n", __func__);
			r = PTR_ERR(nqx_dev->vddal7);
			return r;
		}
		disable_vddal7 = nqx_dev->vddal7;
		if (power_off_charging_mode == 0 && recoverymode == 0) {
			r = vddal7_power_on(nqx_dev);
			if (r < 0) {
				dev_err(&client->dev, "%s: vddal7 is not power on\n", __func__);
				return r;
			}
		}
	}
	/* vivo caibinchen add end, for vddal7 power set*/

	/* vivo zhangjiebin add begin, for VDDIO control*/
	if (vddio_use_ldo_control) {
		if (gpio_is_valid(platform_data->ldo_en_gpio)) {
			r = gpio_request(platform_data->ldo_en_gpio, "nfc_ldo_en_gpio");
			if (r) {
				dev_err(&client->dev,
					"%s: unable to request nfc ldo_en gpio [%d]\n",
						__func__, platform_data->ldo_en_gpio);
				goto err_mem;
			}
			if (power_off_charging_mode == 1 || recoverymode == 1)
				r = gpio_direction_output(platform_data->ldo_en_gpio, 0);
			else
				r = gpio_direction_output(platform_data->ldo_en_gpio, 1);

			if (r) {
				dev_err(&client->dev,
					"%s: unable to set direction for nfc reset gpio [%d]\n",
						__func__, platform_data->ldo_en_gpio);
				goto err_en_gpio;
			}
		} else {
			dev_err(&client->dev, "%s: nfc ldo_en gpio not provided\n", __func__);
			goto err_mem;
		}
	}
	/* vivo zhangjiebin add end*/

	do_gettimeofday(&nqx_dev->tv_read);
	do_gettimeofday(&nqx_dev->tv_write);

	if (gpio_is_valid(platform_data->irq_gpio)) {
		r = gpio_request(platform_data->irq_gpio, "nfc_irq_gpio");
		if (r) {
			dev_err(&client->dev, "%s: unable to request nfc irq gpio [%d]\n",
				__func__, platform_data->irq_gpio);
			goto err_en_gpio;
		}
		r = gpio_direction_input(platform_data->irq_gpio);
		if (r) {
			dev_err(&client->dev,
			"%s: unable to set direction for nfc irq gpio [%d]\n",
				__func__, platform_data->irq_gpio);
			goto err_irq_gpio;
		}
		irqn = gpio_to_irq(platform_data->irq_gpio);
		if (irqn < 0) {
			r = irqn;
			goto err_irq_gpio;
		}
		client->irq = irqn;
	} else {
		dev_err(&client->dev, "%s: irq gpio not provided\n", __func__);
		goto err_en_gpio;
	}

	if (gpio_is_valid(platform_data->firm_gpio)) {
		r = gpio_request(platform_data->firm_gpio,
			"nfc_firm_gpio");
		if (r) {
			dev_err(&client->dev,
				"%s: unable to request nfc firmware gpio [%d]\n",
				__func__, platform_data->firm_gpio);
			goto err_irq_gpio;
		}
		r = gpio_direction_output(platform_data->firm_gpio, 0);
		if (r) {
			dev_err(&client->dev,
				"%s: cannot set direction for nfc firmware gpio [%d]\n",
					__func__, platform_data->firm_gpio);
			goto err_firm_gpio;
		}
	} else {
		dev_err(&client->dev, "%s: firm gpio not provided\n", __func__);
		goto err_irq_gpio;
	}

	if (gpio_is_valid(platform_data->ese_gpio)) {
		r = gpio_request(platform_data->ese_gpio, "nfc-ese_pwr");
		if (r) {
			nqx_dev->ese_gpio = -EINVAL;
			dev_err(&client->dev,
				"%s: unable to request nfc ese gpio [%d]\n",
					__func__, platform_data->ese_gpio);
			/* ese gpio optional so we should continue */
		} else {
			nqx_dev->ese_gpio = platform_data->ese_gpio;
			r = gpio_direction_output(platform_data->ese_gpio, 0);
			if (r) {
				/*
				 * free ese gpio and set invalid
				 * to avoid further use
				 */
				gpio_free(platform_data->ese_gpio);
				nqx_dev->ese_gpio = -EINVAL;
				dev_err(&client->dev,
					"%s: cannot set direction for nfc ese gpio [%d]\n",
						__func__, platform_data->ese_gpio);
				/* ese gpio optional so we should continue */
			}
		}
	} else {
		nqx_dev->ese_gpio = -EINVAL;
		dev_err(&client->dev, "%s: ese gpio not provided\n", __func__);
		/* ese gpio optional so we should continue */
	}

	if (gpio_is_valid(platform_data->clkreq_gpio)) {
		r = gpio_request(platform_data->clkreq_gpio, "nfc_clkreq_gpio");
		if (r) {
			dev_err(&client->dev,
				"%s: unable to request nfc clkreq gpio [%d]\n",
					__func__, platform_data->clkreq_gpio);
			goto err_ese_gpio;
		}
		r = gpio_direction_input(platform_data->clkreq_gpio);
		if (r) {
			dev_err(&client->dev,
			"%s: cannot set direction for nfc clkreq gpio [%d]\n",
				__func__, platform_data->clkreq_gpio);
			goto err_clkreq_gpio;
		}
	} else {
		dev_err(&client->dev, "%s: clkreq gpio not provided\n", __func__);
		goto err_ese_gpio;
	}

	nqx_dev->en_gpio = platform_data->en_gpio;
	/* vivo zhangjiebin add begin, for VDDIO control*/
	if (vddio_use_ldo_control)
		nqx_dev->ldo_en_gpio = platform_data->ldo_en_gpio;

	/* vivo zhangjiebin add end*/
	nqx_dev->irq_gpio = platform_data->irq_gpio;
	nqx_dev->firm_gpio  = platform_data->firm_gpio;
	nqx_dev->clkreq_gpio = platform_data->clkreq_gpio;
	nqx_dev->pdata = platform_data;

	/* init mutex and queues */
	init_waitqueue_head(&nqx_dev->read_wq);
	mutex_init(&nqx_dev->read_mutex);
	mutex_init(&nqx_dev->write_mutex);
	mutex_init(&nqx_dev->ioctl_mutex);
	spin_lock_init(&nqx_dev->irq_enabled_lock);

	nqx_dev->nqx_device.minor = MISC_DYNAMIC_MINOR;
	nqx_dev->nqx_device.name = "nq-nci";
	nqx_dev->nqx_device.fops = &nfc_dev_fops;

	r = misc_register(&nqx_dev->nqx_device);  // misc device
	if (r) {
		dev_err(&client->dev, "%s: misc_register failed\n", __func__);
		goto err_misc_register;
	}

	/* NFC_INT IRQ */
	nqx_dev->irq_enabled = true;
	r = request_irq(client->irq, nqx_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, nqx_dev);
	if (r) {
		dev_err(&client->dev, "%s: request_irq failed\n", __func__);
		goto err_request_irq_failed;
	}
	nqx_disable_irq(nqx_dev);

	/*
	 * To be efficient we need to test whether nfcc hardware is physically
	 * present before attempting further hardware initialisation.
	 *
	 */
	#ifdef NFC_HW_CHECK
	r = nfcc_hw_check(client, nqx_dev);
	if (r) {
		/* make sure NFCC is not enabled */
		gpio_set_value(platform_data->en_gpio, 0);
		/* We don't think there is hardware switch NFC OFF */
		goto err_request_hw_check_failed;
	}
	#endif
#ifdef NFC_HW_DETECT
	if (strncmp(vivo_project_name, "PD2011", 6)) {

		r = nfcc_hw_detect(nqx_dev);
		if (r < 0)
			nfc_result = 0;
		else
			nfc_result = 1;
		if ((power_off_charging_mode == 1) || (recoverymode == 1))
			r = gpio_direction_output(platform_data->en_gpio, 1);
		else
			r = gpio_direction_output(platform_data->en_gpio, 0);
	}
#endif

	/* Register reboot notifier here */
	r = register_reboot_notifier(&nfcc_notifier);
	if (r) {
		dev_err(&client->dev,
			"%s: cannot register reboot notifier(err = %d)\n",
			__func__, r);
		/*
		 * nfcc_hw_check function not doing memory
		 * allocation so using same goto target here
		 */
		goto err_request_hw_check_failed;
	}

#ifdef NFC_KERNEL_BU
	r = nqx_clock_select(nqx_dev);
	if (r < 0) {
		dev_err(&client->dev, "%s: nqx_clock_select failed\n", __func__);
		goto err_clock_en_failed;
	}
	gpio_set_value(platform_data->en_gpio, 1);
#endif


	device_init_wakeup(&client->dev, true);
	device_set_wakeup_capable(&client->dev, true);
	i2c_set_clientdata(client, nqx_dev);
	nqx_dev->irq_wake_up = false;

	dev_err(&client->dev, "%s: probing NFCC SN100 successfully\n", __func__);
	return 0;

#ifdef NFC_KERNEL_BU
err_clock_en_failed:
	unregister_reboot_notifier(&nfcc_notifier);
#endif
err_request_hw_check_failed:
	free_irq(client->irq, nqx_dev);
err_request_irq_failed:
	misc_deregister(&nqx_dev->nqx_device);

err_misc_register:
	mutex_destroy(&nqx_dev->read_mutex);
	mutex_destroy(&nqx_dev->write_mutex);
	mutex_destroy(&nqx_dev->ioctl_mutex);
err_clkreq_gpio:
	gpio_free(platform_data->clkreq_gpio);
err_ese_gpio:
	/* optional gpio, not sure was configured in probe */
	if (nqx_dev->ese_gpio > 0)
		gpio_free(platform_data->ese_gpio);
err_firm_gpio:
	gpio_free(platform_data->firm_gpio);
err_irq_gpio:
	gpio_free(platform_data->irq_gpio);
err_en_gpio:
	gpio_free(platform_data->en_gpio);
err_mem:
	kfree(nqx_dev->kbuf);
err_free_dev:
	kfree(nqx_dev);
err_free_data:
	if (client->dev.of_node)
		devm_kfree(&client->dev, platform_data);
err_platform_data:
	dev_err(&client->dev, "%s: probing nqxx failed, check hardware\n", __func__);
	return r;
}

static int nqx_remove(struct i2c_client *client)
{
	int ret = 0;
	struct nqx_dev *nqx_dev;

	nqx_dev = i2c_get_clientdata(client);
	if (!nqx_dev) {
		dev_err(&client->dev, "%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	unregister_reboot_notifier(&nfcc_notifier);
	free_irq(client->irq, nqx_dev);
	misc_deregister(&nqx_dev->nqx_device);
	mutex_destroy(&nqx_dev->read_mutex);
	mutex_destroy(&nqx_dev->write_mutex);
	mutex_destroy(&nqx_dev->ioctl_mutex);
	gpio_free(nqx_dev->clkreq_gpio);
	/* optional gpio, not sure was configured in probe */
	if (nqx_dev->ese_gpio > 0)
		gpio_free(nqx_dev->ese_gpio);
	gpio_free(nqx_dev->firm_gpio);
	gpio_free(nqx_dev->irq_gpio);
	gpio_free(nqx_dev->en_gpio);
	//add a node for load nfc service start
	sysfs_remove_group(nfc_kobject, &nfc_attr_group);
	kobject_put(nfc_kobject);
	//add a node for load nfc service end
	kfree(nqx_dev->kbuf);
	if (client->dev.of_node)
		devm_kfree(&client->dev, nqx_dev->pdata);

	kfree(nqx_dev);
err:
	return ret;
}

static int nqx_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nqx_dev *nqx_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && nqx_dev->irq_enabled) {
		if (!enable_irq_wake(client->irq))
			nqx_dev->irq_wake_up = true;
	}
	return 0;
}

static int nqx_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nqx_dev *nqx_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && nqx_dev->irq_wake_up) {
		if (!disable_irq_wake(client->irq))
			nqx_dev->irq_wake_up = false;
	}
	return 0;
}

static const struct i2c_device_id nqx_id[] = {
	{"nqx-i2c", 0},
	{}
};

static const struct dev_pm_ops nfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nqx_suspend, nqx_resume)
};

static struct i2c_driver nqx = {
	.id_table = nqx_id,
	.probe = nqx_probe,
	.remove = nqx_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sn100",
		.of_match_table = msm_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm = &nfc_pm_ops,
	},
};

static int nfcc_reboot(struct notifier_block *notifier, unsigned long val,
			  void *v)
{
	int ret = 0;
	printk("%s: nfcc_reboot enter here\n", __func__);
	if (strncmp(vivo_project_name, "PD2011", 6) == 0 && vddio_use_ldo_control == true && power_off_charging_mode == 1)
		gpio_set_value(disable_ctrl, 0);
	else if (isVddal7_Supply()) {
		if (power_off_charging_mode == 1 || recoverymode == 1) {
			printk("%s: in power_off_charging_mode or recoverymode\n", __func__);
			gpio_set_value(disable_ctrl, 0);
			return NOTIFY_OK;
		}
		gpio_set_value(disable_ctrl, 1);
		usleep_range(3000, 3100);
		gpio_set_value(disable_ctrl, 0);
		ret = disable_vddal7_power_off(disable_vddal7);
		printk("%s: poweroff vddal7 ret: %d\n", __func__, ret);
		if (ret < 0)
			return ret;
	} else
		gpio_set_value(disable_ctrl, 1);
	return NOTIFY_OK;
}

/*
 * module load/unload record keeping
 */
static int __init nqx_dev_init(void)
{
	return i2c_add_driver(&nqx);
}
module_init(nqx_dev_init);

static void __exit nqx_dev_exit(void)
{
	unregister_reboot_notifier(&nfcc_notifier);
	i2c_del_driver(&nqx);
}
module_exit(nqx_dev_exit);

MODULE_DESCRIPTION("NFC nqx");
MODULE_LICENSE("GPL v2");
