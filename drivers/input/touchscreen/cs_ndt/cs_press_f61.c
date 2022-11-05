/* drivers/input/cs_press/cs_press_f61.c
 *
 * 2017 - 2020 Chipsea Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the CHIPSEA's PRESS IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Revision Record:
 *      V1.0:
 *          first release. By Yuanmx, 2017/07/26.
 *      V1.2:
 *          add hex_fw_update. By Yuanmx, 2017/08/22.
 *      V1.3:
 *          conform to the standard. By Yuanmx, 2019/01/16.
 *      V1.4:
 *          add customer specific needs. By Yuanmx, 2019/03/25.
 *      V1.5:
 *          add reset IC in probe and other function. By Yuanmx, 2019/04/11.
 *
 */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/mount.h>
#include <linux/proc_fs.h>

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/pinctrl/consumer.h>
#include "cs_press_f61.h"
#include "cs_press_1933.h"
#include "../vivo_force/vivo_force.h"
#include <uapi/linux/mobileevent_sensor_key.h>
#include <linux/mobileevent.h>

#ifdef CONFIG_MTK_I2C_EXTENSION
#include <linux/dma-mapping.h>
#endif

/******* Macro definition **********/
#define LOG(fmt, args...) printk("[cs_press] [%s: %d] "fmt,  __func__, __LINE__, ##args)

#define FW_PATH		"/sdcard/cs_press/pressure_f61.nfw"
#define FW_PATH_1	"/data/cs_press/pressure_f61.nfw"

#define READ_FW_ROM_TO_FILE "/sdcard/cs_press_fw.txt"

#define CS_CHRDEV_NAME "cs_ndt_press"
#define CS_I2C_ADDR			0x50 /* (0xA0>>1) */


/*IIC REG*/
#define IIC_EEPROM			0x00
#define IIC_TEST			0x00
#define IIC_HANDSHAKE     	0x01
#define IIC_CMD     		0x02
#define IIC_HEARTBEAT		0x03
#define IIC_COUNT_INFO		0x06

#define IIC_MCU_ID      	0x0B
#define IIC_RSTSRC			0x0C
#define IIC_PROJECT_ID    	0x0F
#define IIC_FW_VER       	0x10

#define IIC_WAKE_UP      	0x06
#define IIC_SLEEP        	0x07
#define IIC_GREEN        	0x08
#define IIC_GREEN2NORMAL 	0x10

#define IIC_AFE_GAIN     	0x16
#define IIC_DAC_VALUE     	0x17
#define IIC_ADC_UVOLT_COEF	0x1B
#define IIC_DAC_UVOLT_COEF	0x1C

#define IIC_SENSOR_CONN_INFO	0x29

#define IIC_GPIO_STATUS		0x2E

#define IIC_TEMPERATURE			0x54
#define IIC_AP_STATUS			0x55
#define IIC_SCAN_MODE_SWITCH	0x56
#define IIC_FUN_SWITCH			0x57
#define IIC_BUTTON_FORCE_THR	0x5B
#define IIC_BUTTON_POS			0x5C
#define IIC_SLIDE_FORCE_THR		0x60

#define IIC_EVENT_STATUS		0xAB

#define IIC_AP_COEF_VALUE		0xB5

#define IIC_DEBUG_MODE   0xEC
#define IIC_DATA_READY   0xED
#define IIC_DEBUG_DATA1  0xEE

#define IIC_DEBUG_MODE2   0xFB
#define IIC_DEBUG_READY2   0xFC
#define IIC_DEBUG2_DATA  0xFD

#define IIC_KEY_SEN      0xD0
#define IIC_KEY_EVENT      0xD3

#define CS_MCU_ID_LEN   0x8
#define CS_PROJECT_ID_LEN 0x2
#define CS_FW_VER_LEN     0x4 

#define DEBUG_RAW_MODE	0x10
#define DEBUG_DIFF_MODE	0x20

#define CURRENT_LOAD_UA 	(200000)//200mA

#define IIC_RESETCMD 0xf17c
#define FW_UPDATA_MAX_LEN (64*1024)

#define CHECK_DELAY_TIME	(600000)
#define CS_RESET_TIME		(60000)
#define I2C_CHECK_SCHEDULE
#define INT_SET_EN

#define AP_STATUS_POWER_SWITCH		(0x01 << 0)
#define AP_STATUS_GAME_MODE			(0x01 << 1)
#define AP_STATUS_FACTORY_MODE		(0x01 << 2)
#define AP_STATUS_OPT				(0x01 << 3)

#define AP_STATUS_OPT_LANDSCAPE		(0x01 << 0)
#define AP_STATUS_OPT_CALLING		(0x01 << 1)

#define KEY_NUM			3
#define POWER_KEY		(1UL << 1) 
#define VOLUP_KEY		(1UL << 0)     
#define VOLDOWN_KEY		(1UL << 2)

static uint8_t CS_KEY_MAP[] = {
	VOLUP_KEY,
	VOLDOWN_KEY,
	POWER_KEY
};

#define KEY_THRESHOLD_GEAR_MAX		5
static const int KEY_THRESHOLD_DEFAULT[] = {120, 160, 200, 240, 280};

#define TOP_KEYCODE		KEY_TOP_BUTTON
/******* value definition **********/
static unsigned short g_cs_reg;
static struct i2c_client *g_cs_client;

static struct mutex	i2c_rw_lock;
static DEFINE_MUTEX(i2c_rw_lock);

static int defail_reset_mode = 1;

/*values for read/write any register.*/
static unsigned char read_reg_data[64];
static int read_reg_len;

#ifdef I2C_CHECK_SCHEDULE
static struct delayed_work i2c_check_worker;
#endif
static struct delayed_work update_worker;
static struct work_struct i2c_check_resume_worker;
static struct work_struct top_key_reset_worker;

static struct timer_list cs_reset_timer;
static atomic_t cs_reset_timer_ok = ATOMIC_INIT(1);
static atomic_t cs_update_flag;

static DEFINE_MUTEX(cs_reset_lock);

static int press_threshold[2] = { 60, 50, };
static int debug_mode;
static unsigned int cali_param[1] = {100};
static int cali_channel;
static struct kobject *cs_kobj;
static struct wakeup_source cs_wakelock;

#ifdef INT_SET_EN
static DECLARE_WAIT_QUEUE_HEAD(cs_press_waiter);
static int cs_press_int_flag;
static int cs_press_irq;
static const char *product_name = NULL;
static char *cs_press_f = NULL;
/*1 enable,0 disable,  need to confirm after register eint*/
static int cs_irq_flag = 1;
static struct input_dev *cs_input_dev;
#endif
static int cs_press_rst;

static int i2c_suspend;
static int cmd_success;

static atomic_t ap_status_landscape = ATOMIC_INIT(0);

enum CS_BIG_DATA_TYPE {
	IC_ERROR = 0,
	IC_RESET,
};

#define DATA_MAX_LEN	100

/******* fuction definition start **********/
#ifdef INT_SET_EN
static void cs_irq_enable(void);
static void cs_irq_disable(void);
#endif

static int cs_open(struct inode *inode, struct file *file);
static int cs_close(struct inode *inode, struct file *file);
static loff_t cs_lseek(struct file *file, loff_t offset, int whence);
static ssize_t cs_read(struct file *file, char __user *buf,
		size_t count, loff_t *offset);
static ssize_t cs_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset);
static void cs_rst_set(void);
static void cs_set_rst_pin(int val);
/******* fuction definition end **********/
#ifdef BIG_DATA_OLD
extern int writeData(char *modelId, char *filename, char *data);

static int cs_write_bigdata(unsigned char id, char *fmt, ...)
{
	va_list args;
	char data[DATA_MAX_LEN];
	int len;

	VFI("write bigdata id = %d", id);

	memset(data, 0, sizeof(data));
	va_start(args, fmt);
	len = snprintf(data, 6, "%4d:", id);
	vscnprintf(data + len, DATA_MAX_LEN, fmt, args);
	va_end(args);
	return writeData("1405", "1405_0", data);
}
#endif

/*******************************************************	
Function:
	Read data from the i2c slave device.

Input:
	reg:	i2c register.
	datbuf:read data buffer.
	byte_len:operate length.
	
Output:
	numbers of i2c_msgs to transfer
*********************************************************/
static int cs_i2c_read(unsigned char reg, unsigned char *datbuf, int byte_len)
{
	struct i2c_msg msg[2];
	int ret = 0;
	int i = 5;
	int retry_count = 0;

	msg[0].addr  = g_cs_client->addr;
	msg[0].flags = 0;
	msg[0].len   = 1;
	msg[0].buf   = &reg;

	msg[1].addr  = g_cs_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = byte_len;
	msg[1].buf   = datbuf;

	while (i2c_suspend > 0) {
		VFI("warning:iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if (retry_count > 50) {
			VFI("error:after 500ms delay , iic is still  on suspend.");
			return -EPERM;
		}
	}

	mutex_lock(&i2c_rw_lock);
	do {
		ret = i2c_transfer(g_cs_client->adapter, msg,
			sizeof(msg) / sizeof(struct i2c_msg));
		if (ret < 0 && i < 4)
			VFI("i2c_transfer Error ! err_code:%d,i=%d", ret,i);
		else if (ret >= 0) {
			break;
		}

		i--;
	} while (i > 0);
	mutex_unlock(&i2c_rw_lock);

	return ret;
}

/*******************************************************	
Function:
	write data to the i2c slave device.

Input:
	reg:	i2c register.
	datbuf:write data buffer.
	byte_len:operate length.
	
Output:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int cs_i2c_write(unsigned char reg,
		unsigned char *datbuf, int byte_len)
{
	unsigned char *buf;
	struct i2c_msg msg;
	int ret = 0;
	int i = 5;
	int retry_count = 0;

	if (!datbuf || byte_len <= 0)
		return -1;

	buf = (unsigned char *)kmalloc(byte_len + 1, GFP_KERNEL);
	if (!buf)
		return -1;

	memset(buf, 0, byte_len + 1);
	buf[0] = reg;
	memcpy(buf + 1, datbuf, byte_len);

	msg.addr  = g_cs_client->addr;
	msg.flags = 0;
	msg.len   = byte_len + 1;
	msg.buf   = buf;

	while (i2c_suspend > 0) {
		VFI("warning:iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if (retry_count > 50) {
			VFI("error:after 500ms delay , iic is still  on suspend.");
			return -EPERM;
		}
	}

	mutex_lock(&i2c_rw_lock);
	do {
		ret = i2c_transfer(g_cs_client->adapter, &msg, 1);
		if (ret < 0  && i < 4)
			VFI("i2c_transfer Error! err_code:%d,i=%d", ret, i);
		else if (ret >= 0) {
			break;
		}

		i--;
	} while ( i > 0);
	mutex_unlock(&i2c_rw_lock);

	kfree(buf);

	return ret;
}

/*******************************************************	
Function:
	Read data from the i2c slave device.

Input:
	reg:	i2c register, two byte len addr.
	datbuf:read data buffer.
	byte_len:operate length.
	
Output:
	numbers of i2c_msgs to transfer
*********************************************************/
static int cs_i2c_double_read(unsigned short reg,
	unsigned char *datbuf, int byte_len)
{
	struct i2c_msg msg[2];
	int ret = 0;
	int i = 5;
	unsigned char reg16[2];
	int retry_count = 0;


	if (!datbuf)
		return -1;

	reg16[0] = (reg >> 8) & 0xff;
	reg16[1] = reg & 0xff;

	msg[0].addr  = g_cs_client->addr;
	msg[0].flags = 0;
	msg[0].len   = sizeof(reg16);
	msg[0].buf   = reg16;

	msg[1].addr  = g_cs_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = byte_len;
	msg[1].buf   = datbuf;
	while (i2c_suspend > 0) {
		VFI("warning:iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if (retry_count > 50) {
			VFI("error:after 500ms delay , iic is still  on suspend.");
			return -EPERM;
		}
	}

	do {
		ret = i2c_transfer(g_cs_client->adapter, msg,
		sizeof(msg) / sizeof(struct i2c_msg));
		if (ret < 0 && i < 4)
			VFI("i2c_transfer Error ! err_code:%d, i=%d", ret, i);
		else if (ret >= 0) {
			break;
		}

		i--;
	} while (i > 0);

	return ret;
}

/*******************************************************	
Function:
	write data to the i2c slave device.

Input:
	reg:	i2c register, two byte len addr.
	datbuf:write data buffer.
	byte_len:operate length.
	
Output:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int cs_i2c_double_write(unsigned short reg,
	unsigned char *datbuf, int byte_len)
{
	unsigned char *buf = NULL;
	struct i2c_msg msg;
	int ret = 0;
	int i = 5;
	int retry_count = 0;

	if (!datbuf || byte_len <= 0)
		return -1;
	buf = (unsigned char *)kmalloc(byte_len + sizeof(reg), GFP_KERNEL);
	if (!buf)
		return -1;

	memset(buf, 0, byte_len + sizeof(reg));
	buf[0] = (reg >> 8) & 0xff;
	buf[1] = reg & 0xff;
	memcpy(buf + sizeof(reg), datbuf, byte_len);

	msg.addr  = g_cs_client->addr;
	msg.flags = 0;
	msg.len   = byte_len + sizeof(reg);
	msg.buf   = buf;
	while (i2c_suspend > 0) {
		VFI("warning:iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if (retry_count > 50) {
			VFI("error:after 500ms delay , iic is still  on suspend.");
			return -EPERM;
		}
	}

	do {
		ret = i2c_transfer(g_cs_client->adapter, &msg, 1);
		if (ret < 0  && i < 4)
			VFI("i2c_transfer Error! err_code:%d, i=%d", ret, i);
		else if (ret >= 0) {
			break;
		}

		i--;
	} while (i > 0);

	kfree(buf);
	return ret;
}

/*******************************************************	
Function:
	fw updata, erase flash cmd.

Input:
	client:	i2c device..
	
Output:
	0:success;  -1:fail.
*********************************************************/
static int cs_upgrade_erase(struct i2c_client *client)
{
	unsigned char erase_cmd[] = {0xAA, 0x55, 0xA5, 0x5A};

	/*erase flash*/
	if (cs_i2c_double_write(IIC_EEPROM, erase_cmd, sizeof(erase_cmd)) <= 0) {
		VFE("cs_i2c_double_write fails in cs_upgrade_erase\n");
		return -1;
	}

	msleep(2000);
	return 0;
}

/*******************************************************	
Function:
	fw updata, skip boot cmd.

Input:
	client:	i2c device..
	
Output:
	0:success;  -1:fail.
*********************************************************/
static int cs_upgrade_skip(struct i2c_client *client)
{
	unsigned char erase_cmd[] = {0x7E, 0xE7, 0xEE, 0x77};

	if (cs_i2c_double_write(IIC_EEPROM, erase_cmd, sizeof(erase_cmd)) <= 0) {
		VFE("cs_i2c_double_write fails in cs_upgrade_skip\n");
		return -1;
	}
	return 0;
}

/*******************************************************	
Function:
	fw updata, read cmd.

Input:
	
Output:
	0:success;  -1:fail.
*********************************************************/
static int cs_upgrade_read(void)
{
	unsigned char cmd[] = {0xA7, 0x00, 0x00, 0x59};

	if (cs_i2c_double_write(IIC_RESETCMD, cmd, sizeof(cmd)) <= 0) {
		VFE("cs_i2c_double_write fail !\n");
		return -1;
	}
	return 0;
}

/*******************************************************	
Function:
	Set IC Reset.

Input:
	shval:0,sorf reset; 1,hard reset; 2, defail reset.
	
Output:
	0:success;  -1:fail.
*********************************************************/
static int cs_reset(int shval)
{
	unsigned char erase_cmd[] = {0xA0, 0x00, 0x00, 0x60};

	if (shval == 0)
		cs_i2c_double_write(IIC_RESETCMD, erase_cmd, sizeof(erase_cmd));
	else if (shval == 1)
		cs_rst_set();
	else
		if (defail_reset_mode == 0)
			cs_i2c_double_write(IIC_RESETCMD, erase_cmd, sizeof(erase_cmd));
		else if (defail_reset_mode == 1)
			cs_rst_set();


	return 0;
}

/*******************************************************	
Function:
	wake up IC FW from sleep.

Input:
	
Output:
	>0:success;  <=0:fail.
*********************************************************/
static int wake_up_fw(void)
{
	int retry_count = 3;
	int ret = 0;
	char reg_data[2];
	do {
		ret = cs_i2c_read(0x03, reg_data, 1);
		retry_count--;
		if (retry_count == 0)
			VFI("wake up retry %d.\n", retry_count);
	} while (ret <= 0 && retry_count > 0);
	
	return ret;
}

/*******************************************************	
Function:
	updata fw.

Input:
	buf:the fw buf that need to updata.
	len:the fw length.
	
Output:
	>=0:success;  <0:fail.
*********************************************************/
static int burn_fw(unsigned char *buf, int len)
{
	unsigned short reg = 0;
	int byte_len = 0;
	int pos = 0;
	unsigned char *read_buf, *err_buf;
	int ret = 0;
	int number = 0;
	bool i2c_ok_flag = true;
	int page_end = 0;
	//char err_buf[256*3+3];
	int err_line = 16;
	int err_len = 0;
	int i;

	if (len % 128) {
		VFE("burn len is not 128*");
		return -1;
	}
	page_end = len%256;
	VFI("read page_end:%d\n", page_end);
	read_buf = (unsigned char *)kmalloc(len, GFP_KERNEL);
	if (NULL == read_buf) {
		VFE("kmalloc fails. line:%d\n", __LINE__);
		return -1;
	}
	err_buf = (unsigned char *)kmalloc(256*4, GFP_KERNEL);
	if (NULL == err_buf) {
		VFE("kmalloc fails. line:%d\n", __LINE__);
		kfree(read_buf);
		return -EPERM;
	}
#ifdef INT_SET_EN
	cs_irq_disable(); /*close enit irq.*/
#endif
	mutex_lock(&i2c_rw_lock);

	do {
		i2c_ok_flag = true;
		cs_reset(2);
		msleep(60);

		ret = cs_upgrade_erase(g_cs_client);
		if (ret < 0) {
			i2c_ok_flag = false;
			goto I2C_BAD_OUT;
		}
		ret = 0;

		/*write upgrade*/
		reg = 0x00;
		pos = 0;
		byte_len = 128;
		while (pos < len) {
			ret = cs_i2c_double_write(pos, buf + pos, byte_len);
			if (ret < 0) {
				VFE("CS:::[%d] cs_i2c_double_write fails\n",
					__LINE__);
				i2c_ok_flag = false;
				goto I2C_BAD_OUT;
			}
			pos += byte_len;
			reg++;
			msleep(15);
		}

		/*read upgrade*/
		reg = 0x00;
		pos = 0;
		byte_len = 256;
		while (pos < len) {
			ret = cs_i2c_double_read(pos, read_buf + pos, byte_len);
			if (ret < 0) {
				VFE("CS:::[%d]read page fail !page:%d\n",
					__LINE__, reg);
				i2c_ok_flag = false;
				ret = -1;

				err_len = 0;
				for (i = 0; i < byte_len; i++) {
					
					err_len += sprintf(err_buf + err_len,
						"%02x ", read_buf[pos + i]);
					if (i%err_line == 0)
						err_len += sprintf(err_buf + err_len, "\n");
				}
				err_len += sprintf(err_buf + err_len, "\n");
				VFI("err_len = %d", err_len);
				VFI("buf=%s", err_buf);

				goto I2C_BAD_OUT;
			}
			/*check*/
			if (memcmp(buf + pos, read_buf + pos, byte_len)) {
				VFE("CS:::[%d]read page cmp fail !page:%d\n",
					__LINE__, reg);
				i2c_ok_flag = false;
				ret = -1;

				err_len = 0;
				for (i = 0; i < byte_len; i++) {
					err_len += sprintf(err_buf + err_len,
						"%02x ", read_buf[pos + i]);
					if (i%err_line == 0)
						err_len += sprintf(err_buf + err_len, "\n");
				}
				err_len += sprintf(err_buf + err_len, "\n");
				VFI("err_len = %d", err_len);
				VFI("buf=%s", err_buf);

				goto I2C_BAD_OUT;
			}
			pos += byte_len;
			reg++;
			if ((page_end > 0) && (reg >= len / 256))
				byte_len = page_end;
			msleep(15);
		}
I2C_BAD_OUT:
		number++;
	} while (number < 3 && ret < 0 && i2c_ok_flag == false);


	if (ret < 0 || i2c_ok_flag == false)
		VFE("[%d] burn firmware err", __LINE__);
	else
		VFI("[%d] burn firmware success", __LINE__);

	msleep(100);
	cs_reset(2);
	msleep(100);
	cs_upgrade_skip(g_cs_client);
	if (read_buf != NULL)
		kfree(read_buf);
	if (err_buf != NULL)
		kfree(err_buf);

	mutex_unlock(&i2c_rw_lock);
#ifdef INT_SET_EN
	cs_irq_enable(); /*open enit irq.*/
#endif
	return ret;
}
#ifdef CS_FW_AUTOPUDATA_HEX

/*******************************************************	
Function:
	boot up automatic upgrade. compare fw ver.
	fw update from .h file.

Input:
	
Output:
	>=0:success;  <0:fail.
*********************************************************/
static int cs_fw_update(void)
{
	loff_t pos;
	int len = 0;
	unsigned char ic_fw_ver[CS_FW_VER_LEN] = {0};
	unsigned char file_fw_ver[CS_FW_VER_LEN] = {0};
	int ret = -1;

	atomic_set(&cs_update_flag, 1);
	wake_up_fw();
	ret = cs_i2c_read(IIC_FW_VER, ic_fw_ver, sizeof(ic_fw_ver));
	if (ret < 0) {
		VFE("fail to read firmware version\n");
		ret = -EPERM;
		goto close_file_out;
	}

	pos = 8;
	memcpy(file_fw_ver, &cs_press_f[pos], sizeof(file_fw_ver));

	VFI("[current fw:%x][file fw:%x]\n",
		*(unsigned int *)ic_fw_ver, *(unsigned int *)file_fw_ver);

	if (ic_fw_ver[2] == file_fw_ver[2] && ic_fw_ver[3] == file_fw_ver[3]) {
		ret = -EPERM;
		goto close_file_out;
	}

	pos = 0x0c;
	memcpy((char *)&len, &cs_press_f[pos], sizeof(len));
	len = swab32(len);
	VFI("[file fw len:%x]\n", len);
	if (len <= 0 || len > FW_UPDATA_MAX_LEN) {
		VFE("[err!len overflow!len=%x]\n", len);
		ret = -EPERM;
		goto close_file_out;
	}

	pos = 0x100;
	ret = burn_fw(&cs_press_f[pos], len);
	if (ret < 0)
		VFE("Burning firmware fails\n");

close_file_out:
	atomic_set(&cs_update_flag, 0);
	return ret;
}

/*******************************************************	
Function:
	boot up automatic upgrade force.
	fw update from .h file.

Input:
	
Output:
	>=0:success;  <0:fail.
*********************************************************/
static int cs_fw_update_force(void)
{
	loff_t pos;
	int len = 0;
	int ret = -1;

	atomic_set(&cs_update_flag, 1);
	pos = 0x0c;
	memcpy((char *)&len, &cs_press_f[pos], sizeof(len));
	len = swab32(len);
	VFI("[file fw len:%x]\n", len);
	if (len <= 0 || len > FW_UPDATA_MAX_LEN) {
		VFE("[err!len overflow!len=%x]\n", len);
		ret = -EPERM;
		goto close_file_out;
	}

	pos = 0x100;
	ret = burn_fw(&cs_press_f[pos], len);
	if (ret < 0)
		VFE("Burning firmware fails\n");

close_file_out:
	atomic_set(&cs_update_flag, 0);
	return ret;
}
#else

/*******************************************************	
Function:
	boot up automatic upgrade. compare fw ver.
	fw update from .nfw file.

Input:
	
Output:
	>=0:success;  <0:fail.
*********************************************************/
static int cs_fw_update(void)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	int len = 0;
	char *fw_data;
	unsigned char ic_fw_ver[CS_FW_VER_LEN] = {0};
	unsigned char file_fw_ver[CS_FW_VER_LEN] = {0};
	int ret = 1;

	ret = cs_i2c_read(IIC_FW_VER, ic_fw_ver, sizeof(ic_fw_ver));
	if (ret < 0) {
		VFE("fail to read firmware version\n");
		return -1;
	}

	fp = filp_open(FW_PATH, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		fp = filp_open(FW_PATH_1, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			VFE("open file error\n");
			return -1;
		}
	}
	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 8;
	vfs_read(fp, file_fw_ver, sizeof(file_fw_ver), &pos);

	VFI("[current fw:%x][file fw:%x]\n",
		*(unsigned short *)ic_fw_ver, *(unsigned short *)file_fw_ver);

	if (ic_fw_ver[2] > file_fw_ver[2]) {
		goto close_file_out;
			
	} else {
		if (ic_fw_ver[3] >= file_fw_ver[3]) {
			goto close_file_out;
		}
	}


	pos = 0x0c;
	vfs_read(fp, (char *)&len, sizeof(len), &pos);
	len = swab32(len);
	VFI("[file fw len:%x]\n", len);
	if (len <= 0 || len > FW_UPDATA_MAX_LEN) {
		VFE("[err!len overflow!len=%x]\n", len);
		goto close_file_out;
	}
		
	fw_data = (char *)kmalloc(len, GFP_KERNEL);
	if (!fw_data) {
		VFE("fail to malloc buffer\n");
		goto close_file_out;
	}

	pos = 0x100;
	vfs_read(fp, fw_data, len, &pos);

	ret = burn_fw(fw_data, len);
	if (ret == 0)
		VFE("Burning firmware fails\n");

	kfree(fw_data);
close_file_out:
	filp_close(fp, NULL);
	set_fs(fs);
	return ret;
}

#endif /* CS_FW_AUTOPUDATA_HEX */

/*******************************************************	
Function:
	write register need to handshake

Input:
	cmd: register
	
Output:
	=0:success;  <0:fail.
*********************************************************/
static int cs_handshake_done(unsigned char cmd)
{
	unsigned char handshake[2];
	int i = 0, ret;

	handshake[0] = cmd;
	handshake[1] = ~cmd + 1;
	VFI("handshake = 0x%2x 0x%2x", handshake[0], handshake[1]);
	cs_i2c_write(IIC_HANDSHAKE, handshake, 2);

	do {
		cs_i2c_read(IIC_HANDSHAKE, handshake, 2);
		if (handshake[0] == 0x00 && handshake[1] == 0x00)
			break;
		mdelay(10);
		i++;
		
	} while (i < 20);

	if (i < 20)
		ret = 0;
	else
		ret = -EIO;
	
	return ret;
}


/*******************************************************	
Function:
	for set the key trigger threshold .

Input:
	touchTh: key down threshold.
	leaveTh: key up threshold.
	channel: the key channel number.
	
Output:
	1:success;  <=0:fail.
*********************************************************/
static int set_press_threshold(int touchTh, int leaveTh, char channel)
{
	unsigned char datbuf[8];

	VFI("w TouchTh:%d,LeaveTh:%d,channel:%d\n",
		touchTh, leaveTh, channel);

	wake_up_fw();
	datbuf[0] = 0x00;
	cs_i2c_write(0x80, datbuf, 1);

	datbuf[0] = 0x32;
	cs_i2c_write(0x80, datbuf, 1);

	datbuf[0] = 0x00;
	datbuf[1] = channel;

	datbuf[2] = touchTh & 0xff;
	datbuf[3] = (touchTh >> 8) & 0xff;
	datbuf[4] = leaveTh & 0xff;
	datbuf[5] = (leaveTh >> 8) & 0xff;

	cs_i2c_write(0x82, datbuf, 6);

	datbuf[0] = 0x06;
	cs_i2c_write(0x81, datbuf, 1);

	return 1;
}

/*******************************************************	
Function:
	get the key trigger threshold .

Input:
	touchTh: key down threshold pointer.
	leaveTh: key up threshold pointer.
	channel: the key channel number.
	
Output:
	1:success;  <=0:fail.
*********************************************************/
static int get_press_threshold(unsigned int *touchTh,
	unsigned int *leaveTh, char channel)
{
	unsigned char datbuf[8];
	unsigned int mTouchTh = 0;
	unsigned int mLeaveTh = 0;

	wake_up_fw();
	datbuf[0] = 0x00;
	cs_i2c_write(0x80, datbuf, 1);

	datbuf[0] = 0x33;
	cs_i2c_write(0x80, datbuf, 1);

	datbuf[0] = 0x00;
	datbuf[1] = channel;

	cs_i2c_write(0x82, datbuf, 2);

	datbuf[0] = 0x02;
	cs_i2c_write(0x81, datbuf, 1);
	msleep(30);

	datbuf[0] = 0;
	cs_i2c_read(0x81, datbuf, 1);
	if (datbuf[0] == 4) {
		memset(datbuf, 0, 8*sizeof(unsigned char));
		cs_i2c_read(0x82, datbuf, 4);
		mTouchTh = (unsigned int)(short)(datbuf[0]
			| ((datbuf[1]<<8)&0xff00));
		mLeaveTh = (unsigned int)(short)(datbuf[2]
			| ((datbuf[3]<<8)&0xff00));
		*touchTh = mTouchTh;
		*leaveTh = mLeaveTh;
		VFI("TouchTh:%d,LeaveTh:%d\n", mTouchTh, mLeaveTh);
		return 1;
	}
	return 0;
}

/*******************************************************	
Function:
	Write the analog channel calibrated parameter.

Input:
	data1: the cali  parameter. 4 byte.
	channel: the channel number.
	
Output:
	1:success;  <=0:fail.
*********************************************************/
static int write_calibrate_param(unsigned int data1, char channel)
{
	unsigned char datbuf[8];

	VFI("w calibrate_param:%d,channel:%d\n", data1, channel);

	wake_up_fw();

	datbuf[0] = 0x00;
	cs_i2c_write(IIC_DEBUG_MODE2, datbuf, 1);

	datbuf[0] = 0x30;
	cs_i2c_write(IIC_DEBUG_MODE2, datbuf, 1);

	datbuf[0] = channel;

	datbuf[1] = 0x00;

	datbuf[2] = data1 & 0xff;
	datbuf[3] = (data1 >> 8) & 0xff;
	datbuf[4] = (data1 >> 16) & 0xff;
	datbuf[5] = (data1 >> 24) & 0xff;
	datbuf[6] = datbuf[0] + datbuf[1] + datbuf[2]
		+ datbuf[3] + datbuf[4] + datbuf[5]; /*checksum*/
	cs_i2c_write(IIC_DEBUG2_DATA, datbuf, 6);

	datbuf[0] = 0x06;
	cs_i2c_write(IIC_DEBUG_READY2, datbuf, 1);

	return 1;

}

/*******************************************************	
Function:
	Read the analog channel calibrated parameter.

Input:
	data1: the cali  parameter pointer. 4 byte.
	channel: the channel number.
	
Output:
	1:success;  <=0:fail.
*********************************************************/
static int read_calibrate_param(unsigned int *data1, char channel)
{
	unsigned char datbuf[8];
	unsigned int m_data = 0;

	wake_up_fw();

	datbuf[0] = 0x00;
	cs_i2c_write(IIC_DEBUG_MODE2, datbuf, 1);

	datbuf[0] = 0x31;
	cs_i2c_write(IIC_DEBUG_MODE2, datbuf, 1);

	datbuf[0] = channel;

	datbuf[1] = 0x00;
	cs_i2c_write(IIC_DEBUG2_DATA, datbuf, 2);

	datbuf[0] = 0x02;
	cs_i2c_write(IIC_DEBUG_READY2, datbuf, 1);
	msleep(30);

	memset(datbuf, 0, 8 * sizeof(unsigned char));
	cs_i2c_read(IIC_DEBUG_READY2, datbuf, 1);
	VFI("len:%02x\n", datbuf[0]);
	if (datbuf[0] == 4) {
		cs_i2c_read(IIC_DEBUG2_DATA, datbuf, 4);
		VFI("R:%02x,%02x,%02x,%02x\n", datbuf[0],
			datbuf[1], datbuf[2], datbuf[3]);
		m_data = (unsigned int)(datbuf[0] + (datbuf[1] << 8)
			+ (datbuf[2]<<16) + (datbuf[3] << 24));
		*data1 = m_data;
		VFI("R calibrate_param:%d", m_data);
		return 1;
	}
	return 0;
}

#if 0
static int compare_cali_param(void)
{
	char temp[256];
	int slp1, itr1, slp2, itr2, ret;
	int is_cal = 0;
	unsigned int cali_ic0[2];
	unsigned int cali_ic1[2];
	unsigned int cali_data1 = 0;
	char count;

/*	private_slot_rw(0, PRIVATE_SLOT_IDX_FORCE_SENSOR,
		temp, sizeof(temp));
*/
	sprintf(temp, "1 4567 -100 -4668 -200");

	ret = sscanf(temp, "%d %d %d %d %d",
		&is_cal, &slp1, &itr1, &slp2, &itr2);

	VFI("read ROM Cali:%d %d %d %d %d",
		is_cal, slp1, itr1, slp2, itr2);

	if (is_cal == 1) {

		/*ch0*/
		count = 5;

		cali_data1 = (unsigned int)(slp1 + (itr1 << 16));
		do {
			read_calibrate_param(cali_ic0, 0);
			VFI("read Cali CH0 : 0x%08x", cali_ic0[0]);

			if (cali_data1 == cali_ic0[0]) {
				VFI("CH0 is Same!");
				break;
			}

			write_calibrate_param(cali_data1, 0);
			VFI("write Cali CH0 : 0x%08x", cali_data1);
			msleep(20);

			count--;
		} while (count);		

		/*ch1*/
		count = 5;

		cali_data1 = (unsigned int)(slp2 + (itr2 << 16));
		do {
			read_calibrate_param(cali_ic1, 1);
			VFI("read Cali CH1 : 0x%08x", cali_ic1[0]);

			if (cali_data1 == cali_ic1[0]) {
				VFI("CH1 is Same!");
				break;
			}

			write_calibrate_param(cali_data1, 1);
			VFI("write Cali CH1 : 0x%08x", cali_data1);
			msleep(20);

			count--;
		} while (count);			

	}

	return 0;
}
#endif
/*******************************************************	
Function:
	for check ic.

Input:
	
Output:
	0: ic normal;  -1:ic fail.
*********************************************************/

static int cs_ic_check(void)
{
	unsigned char rbuf[4] = {0, };
	int len = 4;
	u16 heartBeat[2] = {0, };
	static u16 last_ht;

	if (cs_i2c_read(IIC_HEARTBEAT, rbuf, len) < 0) {
		VFE("read heart beat data error");
	}

	heartBeat[0] = (rbuf[1] << 8) + rbuf[0];
	heartBeat[1] = (rbuf[3] << 8) + rbuf[2];

	if (last_ht != heartBeat[0] && (heartBeat[0] + heartBeat[1] == 65536)) {
		last_ht = heartBeat[0];
		VFD("ic check success");
		return 0;
	} else {
		VFE("ic check error, last ht = %d, current ht = %d, %d", last_ht, heartBeat[0], heartBeat[1]);
		return  -EPERM;
	}
}
/*******************************************************	
Function:
	for check i2c status.

Input:
	
Output:
	1: ic is normal;  -1:ic is fail.
*********************************************************/
static int check_ic(void)
{
	int retry = 3;
	#ifdef BIG_DATA_OLD
	static int error_times;
	#endif

	do {
		wake_up_fw();
		if (cs_ic_check() >= 0)
			return 1;
		retry--;
		VFE("read fw ver fail!retry:%d\n", retry);
	} while (retry > 0);

	retry = 3;
	do {
		cs_reset(2);
		atomic_set(&ap_status_landscape, 0);
		msleep(300);
		if (cs_ic_check() >= 0)
			return 1;
		retry--;
		VFE("reset fw fail!retry:%d\n", retry);
	} while (retry > 0);
	#ifdef BIG_DATA_OLD
	error_times++;
	cs_write_bigdata(IC_ERROR, "%d", error_times);
	#else
	mobile_event1(MOBILE_MODULE_ID_SENSOR_KEY, SENSOR_KEY_IIC_ERROR_ID, 0);
	#endif

	return -1;
}

/*******************************************************	
Function:
	for check i2c status.

Input:
	
Output:
	1: i2c transfer normal;  -1:i2c transfer fail.
*********************************************************/
static int check_i2c(void)
{
	int retry = 3;
	#ifdef BIG_DATA_OLD
	static int error_times;
	#endif
	unsigned char rbuf = 0;

	do {
		wake_up_fw();
		if (cs_i2c_read(IIC_HEARTBEAT, &rbuf, 1) >= 0)
			return 1;
		retry--;
		VFE("read fw ver fail!retry:%d\n", retry);
	} while (retry > 0);

	retry = 3;
	do {
		cs_reset(2);
		msleep(300);
		if (cs_i2c_read(IIC_HEARTBEAT, &rbuf, 1) >= 0)
			return 1;
		retry--;
		VFE("reset fw fail!retry:%d\n", retry);
	} while (retry > 0);
	#ifdef BIG_DATA_OLD
	error_times++;
	cs_write_bigdata(IC_ERROR, "%d", error_times);
	#else
	mobile_event1(MOBILE_MODULE_ID_SENSOR_KEY, SENSOR_KEY_IIC_ERROR_ID, 0);
	#endif

	return -EPERM;
}

/*******************************************************	
Function:
	check IC FW status.

Input:
	
Output:
	1: FW status normal;  -1:FW status error.
*********************************************************/
static int check_fw_err(void)
{
	int retry = 3;
	int result = 0;

	do {
		if (check_i2c() < 0) {
			result = cs_fw_update_force();
			if (result < 1)
				VFE("update fw fail!retry:%d\n", retry);
		} else {
			return 1;
		}
	retry--;
	} while (retry > 0);
	return -EPERM;
}

#ifdef I2C_CHECK_SCHEDULE
/*******************************************************	
Function:
	schedule delayed for check i2c status.

Input:
	worker: work struct pointer.
	
Output:
	none.
*********************************************************/
static void  i2c_check_func(struct work_struct *worker)
{
	int ret = 0;
	ret = check_ic();
	if (ret >= 0) {
		schedule_delayed_work(&i2c_check_worker, msecs_to_jiffies(CHECK_DELAY_TIME));
		VFI("i2c_check_func schedule work, delay 10 min.");
	} else {
		ret = cs_fw_update_force();
		if (ret < 0)
			VFE("update fw fail!");
	}
}
#endif

static void resume_i2c_check_func(struct work_struct *worker)
{
	int ret;

	if (!mutex_trylock(&cs_reset_lock)) {
		VFE("cs_reset is lock, and resume ic get lock fail");
		return;
	}

	__pm_stay_awake(&cs_wakelock);
	
	ret = check_ic();
	if (ret >= 0) {
		VFI("check i2c is success");
	} else {
		ret = cs_fw_update_force();
		if (ret < 0)
			VFE("update fw fail!");
	}

	__pm_relax(&cs_wakelock);

	mutex_unlock(&cs_reset_lock);
}

/*******************************************************	
Function:
	get eint irq status.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return buff data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_switch_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
#ifdef INT_SET_EN
	ret =  sprintf(buf, "%d\n", cs_irq_flag);
	VFI("irq flag=%d\n", cs_irq_flag);
#endif
	return ret;
}

/*******************************************************	
Function:
	set eint irq enable or disable.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the buff data.
	count: the buff len.
	
Output:
	1:success.
*********************************************************/
static ssize_t cs_switch_irq_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef INT_SET_EN
	if (buf[0] == '0') {
		cs_irq_disable();
		VFI("disable irq\n");
	} else if (buf[0] == '1') {
		cs_irq_enable();
		VFI("enable irq\n");
	}
#endif
	return 1;
}

/*******************************************************	
Function:
	force update the fw from FW_PATH.

Input:
	none.
	
Output:
	1:success.  <=0: fail.
*********************************************************/
static int force_update_fw(void)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	unsigned int len = 0;
	char *fw_data = NULL;
	ssize_t ret = 0;

	fp = filp_open(FW_PATH, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		fp = filp_open(FW_PATH_1, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			VFE("open file error");
		return -1;
		}
	}
	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0x0c;
	vfs_read(fp, (char *)&len, 4, &pos);
	len = swab32(len);
	VFI("fw len = %d\n", len);
	if (len <= 0 || len > FW_UPDATA_MAX_LEN) {
		VFE("[err! len overflow!len=%x]\n", len);
		goto exit_fw_buf;
	}
	
	fw_data = (char *)kmalloc(len, GFP_KERNEL);
	if (!fw_data) {
		VFE("fail to kmalloc buffer\n");
		ret = -ENOMEM;
		goto exit_fw_buf;
	}
	pos = 0x100;
	vfs_read(fp, fw_data, len, &pos);

	ret = burn_fw(fw_data, len);
	if (ret <= 0)
		VFE("Burning firmware fails\n");

	kfree(fw_data);
exit_fw_buf:
	set_fs(fs);
	filp_close(fp, NULL);
	return ret;
}

/*******************************************************	
Function:
	doing the force update fw.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the buff data.
	count: the buff len.
	
Output:
	1:success.  <=0: fail.
*********************************************************/
static ssize_t cs_force_update_fw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return force_update_fw();

}

/*******************************************************	
Function:
	doing the force update fw.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return buff data.
	
Output:
	1:success.  <=0: fail.
*********************************************************/
static ssize_t cs_force_update_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	int result = 0;

	result = force_update_fw();
	if (result >= 1)
		ret = sprintf(buf, "%d,pass!\n", result);
	else
		ret = sprintf(buf, "%d,failed!\n", result);

	VFI("%s", buf);
	return ret+1;
}

/*******************************************************	
Function:
	get fw info.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all the fw info data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_fw_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i   = 0;
	ssize_t ret = 0;
	unsigned char mcu_id[CS_MCU_ID_LEN]     = {0};
	unsigned char s_mcu_id[CS_MCU_ID_LEN*3+2] =  {0};
	unsigned char project_id[CS_PROJECT_ID_LEN] = {0};
	unsigned char fw_ver[CS_FW_VER_LEN]        = {0};

	wake_up_fw();

	ret = cs_i2c_read(IIC_MCU_ID, mcu_id, sizeof(mcu_id));
	if (ret < 0)
		VFE("fail to read MCU_ID\n");
	ret = cs_i2c_read(IIC_PROJECT_ID, project_id, sizeof(project_id));
	if (ret < 0)
		VFE("fail to read PROJECT_ID\n");
	ret = cs_i2c_read(IIC_FW_VER, fw_ver, sizeof(fw_ver));
	if (ret < 0)
		VFE("fail to read firmware version\n");



	for (i = 0; i < CS_MCU_ID_LEN; i++)
		ret += sprintf(s_mcu_id + 3 * i, "%02x ", mcu_id[i]);


	ret = sprintf(buf, "mcu_id:%s\nproject_id:%02x %02x \nfw_ver:%02x %02x %02x %02x\n",
		s_mcu_id, project_id[0], project_id[1], fw_ver[0], fw_ver[1],
		fw_ver[2], fw_ver[3]);
	
	VFI("fw_info:%s\n", buf);
	return ret;
}

/*******************************************************	
Function:
	set debug mode.

Input:
	addr: debug mode address.
	data: write data.
	
Output:
	return write data len.
*********************************************************/
static int set_debug_mode(unsigned char addr, unsigned char data)
{
	int ret;
	unsigned char reg_addr;
	unsigned char reg_data[2];
	int len = 1;
	
	reg_addr = addr;
	len = 1;
	reg_data[0] = data;
	ret = cs_i2c_write(reg_addr, reg_data, len);
	if (ret <= 0) {
		VFE("reg=%d, data=%d, len=%d, err\n",
			reg_addr, reg_data[0], len);
	}
	reg_addr = IIC_DATA_READY;
	len = 1;
	reg_data[0] = 0x0;
	ret = cs_i2c_write(reg_addr, reg_data, len);
	if (ret <= 0) {
		VFE("reg=%d, data=%d, len=%d, err\n",
			reg_addr, reg_data[0], len);
	}
	return ret;
}

/*******************************************************	
Function:
	get debug mode ready status.

Input:
	addr: debug mode address.
	
Output:
	return ready status.
*********************************************************/
static int get_debug_data_ready(unsigned char addr)
{
	int ret;
	unsigned char reg_addr;
	unsigned char reg_data[1];
	int len = 1;
	
	reg_addr = addr;
	len = 1;
	reg_data[0] = 0;
	ret = cs_i2c_read(reg_addr, reg_data, len);
	if (ret <= 0) {
		VFE("reg=%d, data=%d, len=%d, err\n",
			reg_addr, reg_data[0], len);
	}
	VFI("reg=%d, data=%d, len=%d, err\n",
			reg_addr, reg_data[0], len);
	return (int)reg_data[0];
}

/*******************************************************	
Function:
	get debug data.

Input:
	addr: debug mode address.
	data: return debug data.
	len: get data len.
	
Output:
	return data len.
*********************************************************/
static int get_debug_data(unsigned char addr, unsigned char *data, int len)
{
	int ret;
	unsigned char reg_addr;
	unsigned char *reg_data;
	int i;

	reg_addr = addr;
	reg_data = (char *)kmalloc(len, GFP_KERNEL);
	if (reg_data == NULL) {
		VFI("reg_data malloc failed!!!");
		return -EPERM;
	}

	reg_data[0] = 0;
	ret = cs_i2c_read(reg_addr, reg_data, len);
	if (ret <= 0) {
		VFE("reg=%d, data=%d, len=%d, err\n",
			reg_addr, reg_data[0], len);
	}
	for (i = 0; i < len; i++)
		data[i] = reg_data[i];

	return ret;
}

/*******************************************************	
Function:
	get RawData.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all channel RawData.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_get_rawdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned char reg_addr;
	unsigned char reg_data[64];

	int len = 1;
	int raw_data[16];
	int i;

	VFI("buf = %s \n", buf);

	wake_up_fw();

	set_debug_mode(IIC_DEBUG_MODE, DEBUG_RAW_MODE);
	i = 50;
	do {
		msleep(10);
		len = get_debug_data_ready((unsigned char)IIC_DATA_READY);
		VFI("len:%d\n", len);
		if (len > 0) {
			ret = get_debug_data(IIC_DEBUG_DATA1, reg_data, len);
			if (ret < 0)
				return ret;
			VFI("D0:%d,D1:%d,D2:%d,len:%d\n", reg_data[0], reg_data[1], reg_data[2], len);
		}
		i--;
	} while (len == 0 && i > 0);

	ret = 0;
	if (len > 40)
		len = 40;
	for (i = 0; i < len/2; i++) {
		raw_data[i] = ((int)(reg_data[i*2] & 0xff)
			| ((int)(reg_data[i*2 + 1] & 0xff) << 8));

		ret += sprintf(buf + 6 * i, "%05d ", raw_data[i]);
		VFI("raw_data %d=%d\n", i, raw_data[i]);
	}

	ret += sprintf(buf + 6 * i, "\n");
	VFI("buf=%s\n", buf);

	reg_addr = IIC_DEBUG_MODE;
	len = 1;
	reg_data[0] = 0x0;
	cs_i2c_write(reg_addr, reg_data, len);

	return ret;
}

/*******************************************************	
Function:
	get DiffData.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all channel DiffData.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_get_forcedata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned char reg_addr;
	unsigned char reg_data[64];

	int len = 1;
	int raw_data[16];
	int i;

	VFI("buf = %s \n", buf);

	wake_up_fw();
	
	set_debug_mode(IIC_DEBUG_MODE, DEBUG_DIFF_MODE);
	i = 50;
	do {
		msleep(10);
		len = get_debug_data_ready((unsigned char)IIC_DATA_READY);
		VFI("len:%d\n", len);
		if (len > 0) {
			ret = get_debug_data(IIC_DEBUG_DATA1, reg_data, len);
			if (ret < 0)
				return ret;
			VFI("D0:%d,D1:%d,D2:%d,len:%d\n", reg_data[0], reg_data[1], reg_data[2], len);
		}
		i--;
	} while (len == 0 && i > 0);
	
	ret = 0;
	for (i = 0; i < len/2; i++) {
		raw_data[i] = ((int)(reg_data[i*2] & 0xff)
			| ((int)(reg_data[i*2 + 1] & 0xff) << 8));

		ret += sprintf(buf + 6 * i, "%05d ", raw_data[i]);
		VFI("diff_data %d=%d\n", i, raw_data[i]);
	}

	ret += sprintf(buf + 6 * i, "\n");
	VFI("buf=%s\n", buf);

	reg_addr = 0x80;
	len = 1;
	reg_data[0] = 0x0;
	cs_i2c_write(reg_addr, reg_data, len);

	return ret;
}

/*******************************************************	
Function:
	set reset pin high or low.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the reset status.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;

	if (buf[0] == '0')
		val = 0;
	else if (buf[0] == '1')
		val = 1;
	VFI("val:%d,buf:%s\n", val,  buf);
	cs_set_rst_pin(val);
	return count;
}

/*******************************************************	
Function:
	switch sorf reset or hard reset.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the buff data.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_sorf_hard_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 1;

	if (buf[0] == '0')
		val = 0;
	else if (buf[0] == '1')
		val = 1;
	VFI("buf:%s,val:%d\n",  buf, val);
	defail_reset_mode = val;

	return count;
}

/*******************************************************	
Function:
	return read reg data.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return buff data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_rw_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i = 0;

	if (read_reg_len != 0) {
		for (i = 0; i < read_reg_len; i++)
			ret += sprintf(buf + 3 * i, "%02x ", read_reg_data[i]);

	}
	ret += sprintf(buf + 3 * i, "\n");

	VFI("buf=%s\n", buf);

	return ret;
}

/*******************************************************	
Function:
	read or write the register.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the buff data.
	count: the buff len.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_rw_reg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	const char *startpos = buf;
	const char *lastc = buf + count;
	unsigned int tempdata = 0;
	char *firstc = NULL;
	int idx = 0;
	int ret = 0;
	int set_mode = 0;
	unsigned char change_val[32] = { 0 };

	if (!buf || count <= 0) {
		VFE("CS::argument err\n");
		return -EINVAL;
	}

	while (startpos < lastc) {
		VFI("idx:%d\n", idx);
		firstc = strstr(startpos, "0x");
		if (!firstc) {
			VFE("CS::cant find firstc\n");
			return -EINVAL;
		}

		firstc[4] = 0;

		ret = kstrtouint(startpos, 0, &tempdata);
		if (ret) {
			VFE("CS::fail to covert digit\n");
			return -EINVAL;
		}
		if (idx == 0) {
			set_mode = tempdata;
			VFI("set_mode:%d\n", set_mode);
		} else {
			change_val[idx - 1] = tempdata;
			VFI("tempdata:%d\n", tempdata);
		}

		startpos = firstc + 5;

		idx++;

		if (set_mode == 0 && idx > 3 && idx >= change_val[1] + 3)
			break;
		else if (set_mode == 1 && idx > 3)
			break;

	}

	if (set_mode == 0) {
		cs_i2c_write(change_val[0],
			&change_val[2], (int)change_val[1]);
		read_reg_len = 0;
	} else if (set_mode == 1) {
		cs_i2c_read(change_val[0],
			&read_reg_data[0], (int)change_val[1]);
		read_reg_len = change_val[1];
	} else if (set_mode == 2) {
		cs_i2c_double_write(((change_val[0]<<8) | change_val[1]),
			&change_val[3], (int)change_val[2]);
		read_reg_len = 0;
	} else if (set_mode == 3) {
		cs_i2c_double_read(((change_val[0]<<8) | change_val[1]),
			&read_reg_data[0], (int)change_val[2]);
		read_reg_len = change_val[2];
	} else {
		read_reg_len = 0;
	}

	return count;
}

/*******************************************************	
Function:
	read fw rom.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return buff data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_read_fw_rom_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	unsigned int len = 48 * 1024;
	ssize_t ret = 0;

	unsigned short reg = 0;
	int byte_len = 0;
	char *read_buf;
	char err_buf[256*3+3];
	unsigned int err_len = 0;
	int i;

	read_buf = (char *)kmalloc(len, GFP_KERNEL);
	if (NULL == read_buf) {
		VFE("kmalloc fails. line:%d\n", __LINE__);
		return -1;
	}
	memset(read_buf, 0, len);
#ifdef INT_SET_EN
	cs_irq_disable();
#endif
	mutex_lock(&i2c_rw_lock);
/*read upgrade*/
		cs_reset(2);
		msleep(60);

		cs_upgrade_read();

		reg = 0x00;
		pos = 0;
		byte_len = 256;
		while (pos < len) {
			ret = cs_i2c_double_read(pos, read_buf + pos, byte_len);
			if (ret < 0) {
				VFE("read page fail !page:%d\n", reg);
				ret = -1;

				err_len = 0;
				for (i = 0; i < byte_len; i++)
					err_len += sprintf(err_buf + err_len,
						"%02x ", read_buf[pos + i]);

				err_len += sprintf(err_buf + err_len, "\n");
				VFE("buf=%s\n", err_buf);
				goto READ_FW_ROM_ERR;
			}
			pos += byte_len;
			reg++;
			msleep(15);
		}

	cs_reset(2);
	msleep(100);
	cs_upgrade_skip(g_cs_client);
READ_FW_ROM_ERR:
	mutex_unlock(&i2c_rw_lock);
#ifdef INT_SET_EN
	cs_irq_enable();
#endif

	fp = filp_open(READ_FW_ROM_TO_FILE, O_RDWR, 0);
	if (IS_ERR(fp)) {
		VFE("open file error\n");
		return -1;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0x00;
	vfs_write(fp, read_buf, len, &pos);

	set_fs(fs);
	filp_close(fp, NULL);

	if (read_buf != NULL)
		kfree(read_buf);

	return ret;
}

/*******************************************************	
Function:
	get debug mode data.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return buff data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_debug_th_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = 1;

	int change_val[2] = {100, 0};
	
	VFI("debug_mode:%d \n", debug_mode);
	if (debug_mode == 2) {
		change_val[0] = press_threshold[0];
		change_val[1] = press_threshold[1];
	}

	VFI("%d,%d \n", change_val[0], change_val[1]);

	ret = sprintf(buf, "%d,%d\n", change_val[0], change_val[1]);
	return ret;

}

/*******************************************************	
Function:
	doing the debug mode operation.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the buff data.
	count: the buff len.
	
Output:
	the buff len.
*********************************************************/
static ssize_t cs_debug_th_store(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t count)
{
	const char *startpos = buf;
	const char *lastc = buf + count;
	unsigned int tempdata = 0;
	char *firstc = NULL;
	int idx = 0;
	int ret = 0;
	int change_val[3] = {60, 50, 0};
	int set_mode = 0;

	if (!buf || count <= 0) {
		VFE("CS::argument err\n");
		return -EINVAL;
	}

	while (startpos < lastc) {
		VFI("idx:%d\n", idx);
		firstc = strstr(startpos, "0x");
		if (!firstc) {
			VFE("CS::cant find firstc\n");
			return -EINVAL;
		}
		if ((idx == 0) | (idx >= 3))
			firstc[4] = 0;
		else if ((idx == 1) | (idx == 2))
			firstc[6] = 0;

		ret = kstrtouint(startpos, 0, &tempdata);
		if (ret) {
			VFE("CS::fail to covert digit\n");
			return -EINVAL;
		}
		if (idx == 0) {
			set_mode = tempdata;
			VFI("set_mode:%d", set_mode);
		} else if (idx > 0 && idx < 4) {
			change_val[idx - 1] = tempdata;
			VFI("tempdata:%d", tempdata);
		}
		if ((idx == 0) | (idx >= 3))
			startpos = firstc + 5;
		else if ((idx == 1) | (idx == 2))
			startpos = firstc + 7;

		idx++;

		if (idx >= 4)
			break;
	}

	if (set_mode == 1) {
		set_press_threshold(change_val[0],
			change_val[1], change_val[2]);
	} else if (set_mode == 2) {
		get_press_threshold(&press_threshold[0],
			&press_threshold[1], change_val[2]);
		debug_mode = 2;
	} else if (set_mode == 5) { 
		debug_mode = 0;
	}

	return count;
}

/*******************************************************	
Function:
	get cali param data.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return buff data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_cali_param_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i = 0;
	int off = 0;
	int bak = 0;
	unsigned int change_val[3];

	VFI("debug_mode:%d ", debug_mode);
	change_val[0] = cali_param[0] & 0x0000ffff;
	change_val[1] = (cali_param[0] >> 16) & 0x0000ffff;
	change_val[2] = cali_channel;
	VFI("%d,%d ", change_val[0], change_val[1]);
	while (i < 2) {
		bak = change_val[i];
		if (bak <= 9) {
			buf[off] = bak + '0';
			off++;
			buf[off] = ' ';
			off++;
		} else if (bak <= 99) {
			buf[off] = bak/10 + '0';
			off++;
			buf[off] = bak%10 + '0';
			off++;
			buf[off] = ' ';
			off++;
		} else if (bak <= 999) {
			buf[off] = bak/100 + '0';
			bak = bak%100;
			off++;
			buf[off] = bak/10 + '0';
			off++;
			buf[off] = bak%10 + '0';
			off++;
			buf[off] = ' ';
			off++;
		} else if (bak <= 9999) {
			buf[off] = bak/1000 + '0';
			bak = bak%1000;
			off++;
			buf[off] = bak/100 + '0';
			bak = bak%100;
			off++;
			buf[off] = bak/10 + '0';
			off++;
			buf[off] = bak%10 + '0';
			off++;
			buf[off] = ' ';
			off++;
		} else if (bak <= 99999) {
			buf[off] = bak/10000 + '0';
			bak = bak%10000;
			off++;
			buf[off] = bak/1000 + '0';
			bak = bak%1000;
			off++;
			buf[off] = bak/100 + '0';
			bak = bak%100;
			off++;
			buf[off] = bak/10 + '0';
			off++;
			buf[off] = bak%10 + '0';
			off++;
			buf[off] = ' ';
			off++;
		}
		i++;
	}
	buf[off] = '\0';
	ret = strlen(buf);
	return ret;
}

/*******************************************************	
Function:
	write cali param data.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the buff data.
	count: the buff len.
	
Output:
	the buff len.
*********************************************************/
static ssize_t cs_cali_param_store(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t count)
{
	const char *startpos = buf;
	const char *lastc = buf + count;
	unsigned int tempdata = 0;
	char *firstc = NULL;
	int idx = 0;
	int ret = 0;
	unsigned int change_val[2] = {100,  0};
	int set_mode = 0;

	if (!buf || count <= 0) {
		VFE("argument err\n");
		return -EINVAL;
	}

	while (startpos < lastc) {
		VFI("idx:%d", idx);
		firstc = strstr(startpos, "0x");
		if (!firstc) {
			VFE("cant find firstc\n");
			return -EINVAL;
		}
		if ((idx == 0) || (idx > 1))
			firstc[4] = 0;
		else if (idx == 1)
			firstc[10] = 0;

		ret = kstrtouint(startpos, 0, &tempdata);
		if (ret) {
			VFE("fail to covert digit\n");
			return -EINVAL;
		}
		if (idx == 0) {
			set_mode = tempdata;
			VFI("set_mode:%d", set_mode);
		} else if (idx >= 1 && idx < 4) {
			change_val[idx-1] = tempdata;
			VFI("val:%d", change_val[idx-1]);
		}

		if (idx == 0)
			startpos = firstc + 5;
		else if (idx == 1)
			startpos = firstc + 11;
		else if (idx > 1)
			startpos = firstc + 5;

		idx++;

		if (idx >= 3)
			break;
	}

	if (set_mode == 1) {
		write_calibrate_param(change_val[0], (char)change_val[1]);
	} else if (set_mode == 2) {
		read_calibrate_param(cali_param, (char)change_val[1]);
		cali_channel = change_val[1];
	}

	return count;
}

/*******************************************************	
Function:
	get the handshake status.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all the fw info data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_handshake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char handshake[2]     = {0};

	wake_up_fw();;

	ret = cs_i2c_read(IIC_HANDSHAKE, handshake, sizeof(handshake));
	if (ret < 0)
		VFE("fail to read handshake!\n");

	ret = sprintf(buf, "handshake: %02x %02x\n\r",
		handshake[0], handshake[1]);
	
	return ret;
}


/*******************************************************	
Function:
	write the handshake status.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the reset status.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_handshake_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret = 0;
	unsigned char datbuf[2];
	unsigned int handshake[2]; 
	
	ret = sscanf(buf, "%x %x", &handshake[0], &handshake[1]);
	VFI("handshake:%02X %02X\n", handshake[0], handshake[1]);

	wake_up_fw();
	datbuf[0] = (unsigned char)(handshake[0]&0xff);
	datbuf[1] = (unsigned char)(handshake[1]&0xff);
	ret = cs_i2c_write(IIC_HANDSHAKE, datbuf, 2);

	return count;
}

/*******************************************************	
Function:
	get command status.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all the fw info data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_cmd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	
	ret = snprintf(buf, 128, "%d", cmd_success);
	cmd_success = 0;
	
	return ret;
}

/*******************************************************	
Function:
	write the command data.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the reset status.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_cmd_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret = 0, i = 0;
	unsigned char datbuf[2];
	unsigned int cmd = 0; 
	
	ret = sscanf(buf, "%x", &cmd);
	VFI("cmd:%02x\n", cmd);

	wake_up_fw();
	datbuf[0] = (unsigned char)(cmd&0xff);
	datbuf[1] = (unsigned char)((0-cmd)&0xff);
	ret = cs_i2c_write(IIC_CMD, datbuf, 2);
	if (ret < 0) {
		goto result_handler;
	}
	do {
		cs_i2c_read(IIC_CMD, datbuf, 2);
		if (datbuf[0] == 0x00 && datbuf[1] == 0x00)
			break;
		mdelay(10);
		i++;

	} while (i < 20);

result_handler:
	if (i < 20 && ret >= 0) {
		cmd_success = 1;
	} else {
		ret = -EIO;
		VFE("write cmd error");
		cmd_success = 0;
	}

	return count;
}

/*******************************************************	
Function:
	get sensor connecting infomation.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all the fw info data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_sensor_conn_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char sensor_conn_info[1]     = {0};

	wake_up_fw();

	ret = cs_i2c_read(IIC_SENSOR_CONN_INFO, sensor_conn_info, sizeof(sensor_conn_info));
	if (ret < 0)
		VFE("fail to read sensor_conn_info!\n");

	ret = sprintf(buf, "sensor_conn_info: %02x\n\r",
		sensor_conn_info[0]);
	
	return ret;
}

/*******************************************************	
Function:
	get scan mode status.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all the fw info data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_scan_mode_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char scan_mode[1]     = {0};

	wake_up_fw();;

	ret = cs_i2c_read(IIC_SCAN_MODE_SWITCH, scan_mode, sizeof(scan_mode));
	if (ret < 0)
		VFE("fail to read scan_mode status!\n");

	ret = sprintf(buf, "scan_mode: %02x\n", scan_mode[0]);

	return ret;
}

/*******************************************************	
Function:
	change the scan mode.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the reset status.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_scan_mode_switch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret = 0;
	unsigned char datbuf[1];
	unsigned int scan_mode = 0; 
	
	ret = sscanf(buf, "%x", &scan_mode);
	VFI("scan_mode:%02x\n", scan_mode);

	wake_up_fw();
	datbuf[0] = (unsigned char)(scan_mode&0xff);
	ret = cs_i2c_write(IIC_SCAN_MODE_SWITCH, datbuf, 2);

	return count;
}

/*******************************************************	
Function:
	get the function status.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all the fw info data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_function_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char function[1]     = {0};

	wake_up_fw();;

	ret = cs_i2c_read(IIC_FUN_SWITCH, function, sizeof(function));
	if (ret < 0)
		VFE("fail to read function status!\n");

	ret = sprintf(buf, "function status: %02x\n", function[0]);
	
	return ret;
}

/*******************************************************	
Function:
	change the function of edge sensor.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the reset status.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_function_switch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret = 0;
	unsigned char datbuf[1];
	unsigned int function = 0; 
	
	ret = sscanf(buf, "%x", &function);
	VFI("function:%02x\n", function);

	wake_up_fw();
	datbuf[0] = (unsigned char)(function&0xff);
	ret = cs_i2c_write(IIC_FUN_SWITCH, datbuf, 1);

	return count;
}

/*******************************************************	
Function:
	get the button force threshold. 
	each key have two threshold:down and up.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all the fw info data.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_button_force_th_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	int ret = 0;
	unsigned char button_force_th[6*2] = {0};

	wake_up_fw();

	ret = cs_i2c_read(IIC_BUTTON_FORCE_THR, button_force_th, sizeof(button_force_th));
	if (ret < 0)
		VFE("fail to read button_force_th!\n");
	ret = 0;
	for (i = 0; i < 6; i++) {
		ret += sprintf(buf + 5 * i, "%04x ",
		((button_force_th[i*2]&0xff)+((button_force_th[i*2+1]<<8)&0xff00)));
	}
	//VFI("th buf:%s\n",buf);

	ret += sprintf(buf+ret, "\n");
	VFI("th buf:%s\n", buf);
	return ret;
}

/*******************************************************	
Function:
	change the button force threshold. 
	each key have two threshold:down and up.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the reset status.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_button_force_th_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret = 0;
	int i;
	unsigned char datbuf[6*2];
	unsigned int button_force_th[6]; 

	ret = sscanf(buf, "%x %x %x %x %x %x", &button_force_th[0],
		&button_force_th[1], &button_force_th[2], &button_force_th[3],
		&button_force_th[4], &button_force_th[5]);
	VFI("Th:%x %x %x %x %x %x\n", button_force_th[0],
		button_force_th[1], button_force_th[2], button_force_th[3], button_force_th[4], button_force_th[5]);

	for (i = 0; i < 6; i++) {
		datbuf[i*2] = (button_force_th[i]&0xff);
		datbuf[i*2+1] = ((button_force_th[i]>>8)&0xff);
	}
	wake_up_fw();

	ret = cs_i2c_write(IIC_BUTTON_FORCE_THR, datbuf, sizeof(datbuf));

	return count;
}
/*******************************************************	
Function:
	get the GPIO status. 
	each IO have two byte.

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: return all the IO status.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_get_gpio_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	int ret = 0;
	int gpio_num = 5;
	unsigned char gpio_status[2*5] = {0};

	wake_up_fw();

	ret = cs_i2c_read(IIC_GPIO_STATUS, gpio_status, sizeof(gpio_status));
	if (ret < 0)
		VFE("fail to read get_gpio_status!\n");
	ret = 0;
	for (i = 0; i < gpio_num; i++) {
		ret += sprintf(buf + 5 * i, "%04x ",
		((gpio_status[i*2+1]&0xff)+((gpio_status[i*2]<<8)&0xff00))); //just for show H+L;
	}
	//VFI("th buf:%s\n",buf);

	ret += sprintf(buf+ret, "\n");
	VFI("gpio status:%s\n", buf);
	return ret;
}

static int cs_ap_status_set(unsigned char status, bool state)
{
	ssize_t ret = 0;
	unsigned char ap_status = 0;
	
	wake_up_fw();
	ret = cs_i2c_read(IIC_AP_STATUS, &ap_status, sizeof(ap_status));
	if (ret < 0) {
		VFE("fail to read ap_status!");
		goto error_ret;
	}
	if (state) {
		ap_status |= status;
	} else {
		ap_status &= ~status;
	}
	ret = cs_i2c_write(IIC_AP_STATUS, &ap_status, sizeof(ap_status));
	if (ret < 0) {
		VFE("fail to write ap_status!");
		goto error_ret;
	}
	ret = cs_handshake_done(IIC_AP_STATUS);
	if (ret < 0) {
		VFE("write power status is error");
	}

error_ret:
	return ret;
}

/*******************************************************	
Function:
	set IC to factory mode or quit to normal mode

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: set factory mode.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_ap_status_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret = 0;
	unsigned char ap_status = 0;
	
	ret = sscanf(buf, "%d", &ap_status);
	VFI("ap_status: %d", ap_status);

	wake_up_fw();
	ret = cs_i2c_write(IIC_AP_STATUS, &ap_status, sizeof(ap_status));
	if (ret < 0) {
		VFE("fail to write ap_status!");
		goto error_ret;
	}
	ret = cs_handshake_done(IIC_AP_STATUS);
	if (ret < 0) {
		VFE("write factory mode status is error");
	}

error_ret:
	if (ret < 0)
		return ret;
	else
		return count;
}

/*******************************************************	
Function:
	get factory mode

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: factory mode.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_ap_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char ap_status = 0;

	wake_up_fw();

	ret = cs_i2c_read(IIC_AP_STATUS, &ap_status, sizeof(ap_status));
	if (ret < 0) {
		VFE("fail to read get_gpio_status!");
		return ret;
	}
	ret = 0;

	ret += snprintf(buf, 128, "%d", ap_status);
	VFI("ap_status:%s", buf);
	return ret;
}


/*******************************************************	
Function:
	set IC to poweroff status or poweron status

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the reset status.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_power_switch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret = 0;
	unsigned int power_switch = 0;
	
	ret = sscanf(buf, "%d", &power_switch);
	VFI("power_switch: %d", power_switch);

	ret = cs_ap_status_set(AP_STATUS_POWER_SWITCH, !power_switch);
	if (ret < 0) {
		VFE("write power status is error");
		return ret;
	} else {
		return count;
	}
}

/*******************************************************	
Function:
	set IC to factory mode or quit to normal mode

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: set factory mode.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_factory_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t ret = 0;
	unsigned int factory_mode = 0;
	
	ret = sscanf(buf, "%d", &factory_mode);
	VFI("factory_mode: %d", factory_mode);

	ret = cs_ap_status_set(AP_STATUS_FACTORY_MODE, !!factory_mode);
	if (ret < 0) {
		VFE("write factory mode status is error");
		return ret;
	} else {
		return count;
	}
}

/*******************************************************	
Function:
	get factory mode

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: factory mode.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_factory_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char ap_status = 0;

	wake_up_fw();

	ret = cs_i2c_read(IIC_AP_STATUS, &ap_status, sizeof(ap_status));
	if (ret < 0) {
		VFE("fail to read get_gpio_status!");
		return ret;
	}
	ret = 0;

	ap_status = ((ap_status & 0x04) == 0x04) ? 1 : 0;
	ret += snprintf(buf, 128, "%d", ap_status);
	VFI("factory mode:%s", buf);
	return ret;
}

/*******************************************************	
Function:
	set temperature value to ic

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: getting the temperature.
	count: the buf len.
	
Output:
	the buf len.
*********************************************************/
static ssize_t cs_temperature_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	
	ssize_t ret = -1;
	s8 temperature[2] = {0, };
	int temp = 0;
	
	ret = sscanf(buf, "%d", &temp);
	if (temp > 1279 || temp < -1279) {
		VFE("set temperature is valid");
		goto error_ret;
	}
	temperature[0] = (s8)(temp / 10);
	temperature[1] = ~temperature[0] + 1;
	VFI("temperature is: %d, inverse value is: %d", temperature[0], temperature[1]);

	wake_up_fw();
	ret = cs_i2c_write(IIC_TEMPERATURE, temperature, sizeof(temperature));
	if (ret < 0) {
		VFE("fail to write temperature!");
		goto error_ret;
	}
	temperature[0] = temperature[1] = 0;
	ret = cs_i2c_read(IIC_TEMPERATURE, temperature, sizeof(temperature));
	if (ret < 0) {
		VFE("fail to read temperature!");
		goto error_ret;
	}
	VFI("read temperature is: %d, inverse value is: %d", temperature[0], temperature[1]);
error_ret:
	if (ret < 0)
		return ret;
	else
		return count;

}

/*******************************************************	
Function:
	get temperature value from IC

Input:
	dev: device struct.
	attr: device_attribute struct.
	buf: factory mode.
	
Output:
	the return buf len.
*********************************************************/
static ssize_t cs_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = -1;
	s8 temperature = 0;

	wake_up_fw();

	ret = cs_i2c_read(IIC_TEMPERATURE, &temperature, sizeof(temperature));
	if (ret < 0) {
		VFE("fail to read temperature!");
		return ret;
	}
	ret = 0;

	ret += snprintf(buf, 128, "%d", temperature);
	VFI("temperature:%s", buf);
	return ret;
}

static DEVICE_ATTR(force_update_fw, S_IRUGO | S_IWUSR,
		cs_force_update_fw_show, cs_force_update_fw_store);
static DEVICE_ATTR(fw_info, S_IRUGO , cs_fw_info_show, NULL);
static DEVICE_ATTR(get_rawdata, S_IRUGO, cs_get_rawdata_show, NULL);
static DEVICE_ATTR(get_forcedata, S_IRUGO, cs_get_forcedata_show, NULL);
static DEVICE_ATTR(switch_irq, S_IRUGO | S_IWUSR, cs_switch_irq_show,
		cs_switch_irq_store);
static DEVICE_ATTR(reset,  S_IWUSR,
		NULL, cs_reset_store);
static DEVICE_ATTR(sorf_hard_reset,  S_IWUSR,
		NULL, cs_sorf_hard_reset_store);
static DEVICE_ATTR(rw_reg, S_IRUGO | S_IWUSR,
		cs_rw_reg_show, cs_rw_reg_store);
static DEVICE_ATTR(read_fw_rom, S_IRUGO , cs_read_fw_rom_show, NULL);
static DEVICE_ATTR(debug_th, S_IRUGO | S_IWUSR,
	cs_debug_th_show, cs_debug_th_store);
static DEVICE_ATTR(cali_param, S_IRUGO | S_IWUSR,
	cs_cali_param_show, cs_cali_param_store);
static DEVICE_ATTR(handshake, S_IRUGO | S_IWUSR,
	cs_handshake_show, cs_handshake_store);
static DEVICE_ATTR(cmd, S_IRUGO | S_IWUSR,
	cs_cmd_show, cs_cmd_store);
static DEVICE_ATTR(sensor_conn_info, S_IRUGO,
	cs_sensor_conn_info_show, NULL);
static DEVICE_ATTR(scan_mode_switch, S_IRUGO | S_IWUSR,
	cs_scan_mode_switch_show, cs_scan_mode_switch_store);
static DEVICE_ATTR(function_switch, S_IRUGO | S_IWUSR,
	cs_function_switch_show, cs_function_switch_store);
static DEVICE_ATTR(button_force_th, S_IRUGO | S_IWUSR,
	cs_button_force_th_show, cs_button_force_th_store);
static DEVICE_ATTR(get_gpio_status, S_IRUGO,
	cs_get_gpio_status_show, NULL);
static DEVICE_ATTR(power_switch, S_IWUSR,
	NULL, cs_power_switch_store);
static DEVICE_ATTR(factory_mode, S_IRUGO | S_IWUSR,
	cs_factory_mode_show, cs_factory_mode_store);
static DEVICE_ATTR(temperature, S_IRUGO | S_IWUSR,
	cs_temperature_show, cs_temperature_store);
static DEVICE_ATTR(ap_status, S_IRUGO | S_IWUSR,
	cs_ap_status_show, cs_ap_status_store);

static struct attribute *cs_sys_attrs[] = {
	&dev_attr_force_update_fw.attr,
	&dev_attr_fw_info.attr,
	&dev_attr_get_rawdata.attr,
	&dev_attr_get_forcedata.attr,
	&dev_attr_switch_irq.attr,
	&dev_attr_reset.attr,
	&dev_attr_sorf_hard_reset.attr,
	&dev_attr_rw_reg.attr,
	&dev_attr_read_fw_rom.attr,
	&dev_attr_debug_th.attr,
	&dev_attr_cali_param.attr,
	&dev_attr_handshake.attr,
	&dev_attr_cmd.attr,
	&dev_attr_sensor_conn_info.attr,
	&dev_attr_scan_mode_switch.attr,
	&dev_attr_function_switch.attr,
	&dev_attr_button_force_th.attr,
	&dev_attr_get_gpio_status.attr,
	&dev_attr_power_switch.attr,
	&dev_attr_factory_mode.attr,
	&dev_attr_temperature.attr,
	&dev_attr_ap_status.attr,
	NULL,
};

static const struct attribute_group cs_sys_attr_group = {
	.attrs = cs_sys_attrs,
	.name = "cs_press",
};

/*******************************************************	
Function:
	create the cs_press files.

Input:
	none.
	
Output:
	0:success. error code:fail
*********************************************************/
static int cs_sys_create(void)
{
	int ret = 0;

	cs_kobj = kobject_create_and_add("cs_press", NULL);
	if (cs_kobj == NULL) {
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_group(cs_kobj, &cs_sys_attr_group);
	return ret;
}


static const struct file_operations cs_fops = {
	.owner		= THIS_MODULE,
	.read		= cs_read,
	.write		= cs_write,
	.llseek		= cs_lseek,
	.open		= cs_open,
	.release	= cs_close,
};
static struct miscdevice cs_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "ndt",
	.fops  = &cs_fops,
};

/*******************************************************	
Function:
	open the file.

Input:
	inode: inode struct.
	file: file path and name.
	
Output:
	0:success.  	-1:fail.
*********************************************************/
static int cs_open(struct inode *inode, struct file *file)
{
	g_cs_reg = 0;

	if (NULL == g_cs_client)
		return -1;
	else
		return 0;
}

/*******************************************************	
Function:
	close the file.

Input:
	none.
	
Output:
	0:success.  
*********************************************************/
static int cs_close(struct inode *inode, struct file *file)
{
	return 0;
}

/*******************************************************	
Function:
	set the file deviation.

Input:
	file: file name.
	offset: file deviation.
	whence: end point.
	
Output:
	file deviation.  
*********************************************************/
static loff_t cs_lseek(struct file *file, loff_t offset, int whence)
{
	VFI("cs_lseek: %d", (int)offset);
	g_cs_reg = offset;
	return offset;
}

/*******************************************************	
Function:
	read file data.

Input:
	file: file name.
	buf: return buff data.
	count: the buff len.
	offset: the file deviation.
	
Output:
	>0:success.	<0:err.
*********************************************************/
static ssize_t cs_read(struct file *file, char __user *buf,
		size_t count, loff_t *offset)
{
	int err;
	char *kbuf = NULL;
	char reg;

	kbuf = kzalloc(count, GFP_KERNEL);
	if (!kbuf) {
		err = -ENOMEM;
		goto exit;
	}

	/*get reg addr buf[0]*/
	if (copy_from_user(&reg, buf, 1)) {
		err = -EFAULT;
		goto exit_kfree;
	}

	err = cs_i2c_read(reg, kbuf, count);
	if (err < 0)
		goto exit_kfree;

	if (copy_to_user(buf+1, kbuf, count))
		err = -EFAULT;

exit_kfree:
	kfree(kbuf);

exit:
	return err;
}

/*******************************************************	
Function:
	write file data.

Input:
	file: file name.
	buf: return buff data.
	count: the write buff len.
	offset: the file deviation.
	
Output:
	>0:success.	<0:err.
*********************************************************/
static ssize_t cs_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	int err;
	char *kbuf = NULL;
	char reg;

	kbuf = kzalloc(count, GFP_KERNEL);
	if (!kbuf) {
		err = -ENOMEM;
		goto exit;
	}

	if (copy_from_user(&reg, buf, 1)
		|| copy_from_user(kbuf, buf+1, count)) {
		err = -EFAULT;
		goto exit_kfree;
	}

	err = cs_i2c_write(reg, kbuf, count);

exit_kfree:
	kfree(kbuf);

exit:
	return err;
}

/*---add eint detect start----*/
#ifdef INT_SET_EN

/*******************************************************	
Function:
	enable irq.

Input:
	none.
	
Output:
	none.
*********************************************************/
static void cs_irq_enable(void)
{
	if (cs_irq_flag == 0) {
		cs_irq_flag++; 
		enable_irq(cs_press_irq);
	} else {
		VFI("cs_press Eint already enabled!");
	}
	VFI("Enable irq_flag=%d", cs_irq_flag);

}

/*******************************************************	
Function:
	disable irq.

Input:
	none.
	
Output:
	none.
*********************************************************/
static void cs_irq_disable(void)
{
	if (cs_irq_flag == 1) {
		cs_irq_flag--;
		disable_irq_nosync(cs_press_irq);
	} else {
		VFI("cs_press Eint already disabled!");
	}
	VFI("Disable irq_flag=%d", cs_irq_flag);
}

/*******************************************************	
Function:
	the irq interrupt function.

Input:
	irq: interrupt number.
	dev_id: device id.
	
Output:
	return irq handled.
*********************************************************/
static irqreturn_t cs_press_interrupt_handler(int irq, void *dev_id)
{
	printk("cs_press entry irq ok.\n");
	cs_press_int_flag = 1;

	//cs_irq_disable();
	wake_up_interruptible(&cs_press_waiter);

	return IRQ_HANDLED;
}

static void report_event(void)
{	
	uint8_t val[2] = {0};
	uint8_t rst_val[4] = {0, };
	uint8_t i;
	static uint8_t key_state;
	static uint8_t val_prev[2] = {0};
	int rst_value = 0;

	cs_i2c_read(IIC_EVENT_STATUS, val, 2);
	VFI("reg IIC_EVENT_STATUS val[0] = 0x%02X  val[1] = 0x%02X", val[0], val[1]);

	/*just for key event, no need to input*/
	if (val[0] & 0x02) {
		cs_i2c_read(IIC_RSTSRC, rst_val, 4);
		for (i = 0; i < 4; i++)
			rst_value += (rst_val[i] << i * 8);
		VFI("rst_val is 0x%08X", rst_value);
		#ifdef BIG_DATA_OLD
		cs_write_bigdata(IC_RESET, "%ld", rst_value);
		#endif
	}

	if (val[0] & 0x01) {
		for (i = 0; i < sizeof(CS_KEY_MAP); i++) {
			if ((val_prev[1] ^ val[1]) & CS_KEY_MAP[i]) {
				if (val[1] & CS_KEY_MAP[i]) {
					switch (CS_KEY_MAP[i]) {
					case VOLUP_KEY:
						VFI("input key event VOLUP_KEY DOWN");
						break;
					case VOLDOWN_KEY:
						VFI("input key event VOLDOWN_KEY DOWN");
						break;
					case POWER_KEY:
						VFI("input key event POWER_KEY DOWN");
						break;
					}
					key_state |= CS_KEY_MAP[i];
				} else if (key_state & CS_KEY_MAP[i]) {
					switch (CS_KEY_MAP[i]) {
					case VOLUP_KEY:
						VFI("input key event VOLUP_KEY UP");
						break;
					case VOLDOWN_KEY:
						VFI("input key event VOLDOWN_KEY UP");
						break;
					case POWER_KEY:
						VFI("input key event POWER_KEY UP");
						break;
					}
					key_state &= ~CS_KEY_MAP[i];
				}
			}
		}
	}
	
	val_prev[0] = val[0];
	val_prev[1] = val[1];
	
	val[0] = 0;
	cs_i2c_write(IIC_EVENT_STATUS, val, 1);
}


/*******************************************************	
Function:
	the interrupt thread.

Input:
	unused: thread attribute.
	
Output:
	0:success.
*********************************************************/
static int cs_press_event_handler(void *unused)
{
	do {
		//VFI("cs_press_event_handler do wait");
		wait_event_interruptible(cs_press_waiter,
			cs_press_int_flag != 0);
		//VFI("cs_press_event_handler enter wait");
		cs_press_int_flag = 0;

		report_event();

	} while (!kthread_should_stop());
	return 0;
}

/*******************************************************	
Function:
	eint interrupt init.

Input:
	none.
	
Output:
	none.
*********************************************************/
static void eint_init(void)
{
	int ret;
	
	init_waitqueue_head(&cs_press_waiter);

	kthread_run(cs_press_event_handler, 0, CS_CHRDEV_NAME);/*eint thread*/
	ret = request_irq(cs_press_irq,
			  (irq_handler_t)cs_press_interrupt_handler,
			  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			  "CS_PRESS-eint", NULL);
	if (ret > 0) {
		ret = -1;
		VFI("cspress request_irq IRQ fail !,ret=%d.", ret);
	}
	//cs_irq_disable();
	VFI("init_irq ok");

	cs_input_dev = input_allocate_device();
	if (cs_input_dev != NULL) {
		cs_input_dev->name = CS_CHRDEV_NAME;
		__set_bit(EV_KEY, cs_input_dev->evbit);
		__set_bit(KEY_HOME, cs_input_dev->keybit);
		__set_bit(KEY_VOLUMEUP, cs_input_dev->keybit);
		__set_bit(KEY_VOLUMEDOWN, cs_input_dev->keybit);
		__set_bit(KEY_POWER, cs_input_dev->keybit);
		ret = input_register_device(cs_input_dev);
		if (ret != 0)
			VFE("input register device error = %d", ret);
	}
}
#endif
/*---add eint detect end----*/

/*******************************************************	
Function:
	setting hardware dtsi configuration.

Input:
	pdev: i2c_client struct.
	
Output:
	>=0:success.	<0:fail.
*********************************************************/
static int parse_dt(struct i2c_client *pdev)
{
	int ret = -1;
	ret = of_property_read_string((&pdev->dev)->of_node, "product-name", &product_name);
	if (ret) {
		product_name = NULL;
		VFI("get product_name fail!");
		return ret;
	}
#ifdef INT_SET_EN
	cs_press_irq = of_get_named_gpio((&pdev->dev)->of_node, "press,irq-gpio", 0);
	if (!gpio_is_valid(cs_press_irq)) {
		VFI("irq gpio is invaid!");
	} else {
		ret = gpio_request(cs_press_irq, "press,irq-gpio");
		if (ret < 0) {
			VFI("request cs_press_irq gpio fail!");
		}
		ret = gpio_direction_input(cs_press_irq);
		msleep(50);
		cs_press_irq = gpio_to_irq(cs_press_irq);
	}
#endif
	cs_press_rst = of_get_named_gpio((&pdev->dev)->of_node, "press,rst-gpio", 0);
	if (!gpio_is_valid(cs_press_rst)) {
		dev_err(&pdev->dev, "cs_press request_rst fail");
	} else {
		ret = gpio_request(cs_press_rst, "press,rst-gpio");
		if (ret) {
			dev_err(&pdev->dev, "cs_press request rst fail !,ret=%d.\n", ret);
		}
		ret = gpio_direction_output(cs_press_rst, 0);
		msleep(50);
		gpio_set_value(cs_press_rst, 0);
	}
	VFI("end---");
	return 0;
}

/*******************************************************	
Function:
	setting reset pin level.

Input:
	val: the value of reset pin.
	
Output:
	none.
*********************************************************/
static void cs_set_rst_pin(int val)
{
	if (gpio_is_valid(cs_press_rst)) {
		if (val == 0)
			gpio_set_value(cs_press_rst, 0);
		else if (val == 1)
			gpio_set_value(cs_press_rst, 1);
	}
}

/*******************************************************	
Function:
	reset IC.

Input:
	none.
	
Output:
	none.
*********************************************************/
static void cs_rst_set(void)
{
	if (gpio_is_valid(cs_press_rst)) {
		rt_mutex_lock(&g_cs_client->adapter->bus_lock);

		gpio_set_value(cs_press_rst, 1);
		VFI("rst set 1");
		msleep(10);
		gpio_set_value(cs_press_rst, 0);
		VFI("rst set 0");

		rt_mutex_unlock(&g_cs_client->adapter->bus_lock);
	}
}

/* topkey reset ic handler */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
static void cs_reset_timer_up(unsigned long arg)
#else
static void cs_reset_timer_up(struct timer_list *t)
#endif
{
	VFI("cs timer is up");
	atomic_set(&cs_reset_timer_ok, 1);
}

static void top_key_reset_func(struct work_struct *worker)
{
	VFI("top key to cs_reset");
	mutex_lock(&cs_reset_lock);
	cs_reset(2);
	atomic_set(&ap_status_landscape, 0);
	msleep(300);
	mutex_unlock(&cs_reset_lock);
	mod_timer(&cs_reset_timer, jiffies + msecs_to_jiffies(CS_RESET_TIME));
}

static void cs_input_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	if (code == TOP_KEYCODE && value == 0 && atomic_read(&cs_update_flag) == 0 && atomic_read(&cs_reset_timer_ok)) {
		VFI("top key press up");
		atomic_set(&cs_reset_timer_ok, 0);
		schedule_work(&top_key_reset_worker);
	}
}

static int cs_input_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret = -1;

	if ((strcmp(dev->name, "gpio-keys")) && (strcmp(dev->name, "qpnp_pon")))  {
		VFI("%s: %s: Not top key input device\n", 
					__func__, dev->name);
		return ret;
	}

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cs_input_handle";

	ret = input_register_handle(handle);
	if (ret) {
		VFE("cs input handle register error");
		goto err2;
	}

	ret = input_open_device(handle);
	if (ret) {
		VFE("input device open error");
		goto err1;
	}

	INIT_WORK(&top_key_reset_worker, top_key_reset_func);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 14, 0)
	init_timer(&cs_reset_timer);
	cs_reset_timer.function = cs_reset_timer_up;
#else
	timer_setup(&cs_reset_timer, cs_reset_timer_up, 0);
#endif

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return ret;
}

static void cs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cs_input_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler cs_input_handler = {
	.event		= cs_input_event,
	.connect	= cs_input_connect,
	.disconnect	= cs_input_disconnect,
	.name		= "cs_input",
	.id_table	= cs_input_ids,
};

static int cs_input_register(void)
{
	int ret;
	
	ret = input_register_handler(&cs_input_handler);
	if (ret) {
		VFE("cs input register handler error");
	}

	return ret;
}


/*---add gpio setting end---*/
static void update_work_func(struct work_struct *worker)
{
	cs_fw_update();
	if (cs_input_register()) {
		VFE("register input handler failed");
	}
}
static int cs_get_rawdiff(int which, char *resbuf)
{
	int ret;
	char reg_data[32];

	int len = 1;
	int raw_data[8];
	int i;

	wake_up_fw();

	if (which == VIVO_FORCE_RAW_DATA) {
		set_debug_mode(IIC_DEBUG_MODE, DEBUG_RAW_MODE);
	} else if (which == VIVO_FORCE_DIF_DATA) {
		set_debug_mode(IIC_DEBUG_MODE, DEBUG_DIFF_MODE);
	} else {
		return 0;
	}
	
	i = 50;
	do {
		msleep(10);
		len = get_debug_data_ready((unsigned char)IIC_DATA_READY);
		VFI("len:%d\n", len);
		if (len > 0) {
			ret = get_debug_data(IIC_DEBUG_DATA1, reg_data, len);
			if (ret < 0)
				return ret;
			VFI("D0:%d,D1:%d,D2:%d,len:%d\n", reg_data[0], reg_data[1], reg_data[2], len);
		}
		i--;
	} while (len == 0 && i > 0);

	ret = 0;

	for (i = 0; i < len/2; i++) {
		raw_data[i] = ((int)(reg_data[i*2] & 0xff)
			| ((int)(reg_data[i*2 + 1] & 0xff) << 8));

		ret += snprintf(resbuf + 6 * i, 255, "%05d ", raw_data[i]);
		VFI("raw_data %d=%d\n", i, raw_data[i]);
	}

	ret += snprintf(resbuf + 6 * i, 255, "\n");
	VFI("buf=%s\n", resbuf);
	
	return ret;
}

static unsigned char* cs_fw_get_version(void)
{
	int ret;
	static unsigned char fw_ver[CS_FW_VER_LEN] = {0, };

	wake_up_fw();
	ret = cs_i2c_read(IIC_FW_VER, fw_ver, sizeof(fw_ver));
	if (ret < 0)
		VFE("fail to read firmware version");
	VFI("fw version:%02x %02x %02x %02x", fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3]);

	return fw_ver;
}
#ifdef CS_PRESS_COEF

/*******************************************************	
Function:
	the interface of get calibration coefficients for game.

Input:
	the calibration.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_get_calibration_coefficients(unsigned char *coef)
{
	int ret = 0;
	int i;
	unsigned char res[KEY_NUM];
	
	wake_up_fw();
	ret = cs_i2c_read(IIC_AP_COEF_VALUE, res, KEY_NUM);
	if (ret < 0) {
		VFE("read coef_addr fail ret = %d", ret);
		goto END;
	}

	for (i = 0; i < KEY_NUM; i++) {
		VFI("key %d coef: %05d", i + 1, res[i]);
		ret += snprintf(coef + 14 * i, 255, "key %02d: %05d ", i + 1, res[i]);
	}
	ret += snprintf(coef + 14 * i, 255, "\n");
END:
	return ret;
}

/*******************************************************	
Function:
	the interface of change calibration coefficients for game.

Input:
	which button, the calibration to change.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_change_calibration_coefficients (int which, int coef)
{
	int ret = 0;
	unsigned char data[KEY_NUM];

	VFI("The input arg is which = %d  coef = %d", which, coef);
	if (coef < 7 || coef > 13) {
		VFE("The input coefficients is out of range");
		ret = -EPERM;
		goto END;
	}
	if (which >= KEY_NUM) {
		VFE("Key value is out of range");
		ret = -EPERM;
		goto END;
	}
	wake_up_fw();
	ret = cs_i2c_read(IIC_AP_COEF_VALUE, data, KEY_NUM);
	if (ret < 0) {
		VFE("read coef_addr fail ret = %d", ret);
		goto END;
	}

	data[which] = coef & 0xf;

	ret = cs_i2c_write(IIC_AP_COEF_VALUE, data, KEY_NUM);
	if (ret < 0) {
		VFE("write coef_addr fail ret = %d", ret);
		goto END;
	}
	ret = cs_handshake_done(IIC_AP_COEF_VALUE);
	if (ret < 0) {
		VFE("handshake is error ret = %d", ret);
		goto END;
	}
	VFI("change calibration coefficients success");
	
END:
	return ret;
}
#else

static unsigned char cs_get_gear(int threshold)
{
	int i;

	for (i = 0; i < KEY_THRESHOLD_GEAR_MAX; i++) {
		if (threshold <= KEY_THRESHOLD_DEFAULT[i]) {
			return (i + 1);
		}
	}
	
	return KEY_THRESHOLD_GEAR_MAX;
}

/*******************************************************	
Function:
	the interface of get threshold gear.

Input:
	the threshold gear.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_get_threshold_gear(unsigned char *gear)
{
	int ret = 0;
	int i;
	int temp[KEY_NUM * 2];
	unsigned char threshold_temp[KEY_NUM * 4];
	unsigned char threshold_gear[KEY_NUM];
	wake_up_fw();
	ret = cs_i2c_read(IIC_BUTTON_FORCE_THR, threshold_temp, KEY_NUM * 4);
	if (ret < 0) {
		VFE("read threshold_addr fail ret = %d", ret);
		goto END;
	}

	for (i = 0 ; i < KEY_NUM; i++) {
		temp[i * 2] = (int)(threshold_temp[i * 4] & 0xFF) | (int)((threshold_temp[i * 4 + 1] & 0xFF) << 8);
		temp[i * 2 + 1] = (int)(threshold_temp[i * 4 + 2] & 0xFF) | (int)((threshold_temp[i * 4 + 3] & 0xFF) << 8);
		threshold_gear[i] = cs_get_gear(temp[i * 2]);
		VFI("key %d down threshold:%05d, up threshold:%05d, threshold gear:%2d", i + 1, temp[i], temp[i + 1], threshold_gear[i]);
		ret += snprintf(gear + i * 18, 255, "key %02d gear:%05d ", i + 1, threshold_gear[i]);
	}
	ret += snprintf(gear + i * 18, 255, "\n");
END:
	return ret;
}

/*******************************************************	
Function:
	the interface of change threshold gear

Input:
	which button, the threshold gear to change.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_set_threshold_gear (int which, int gear)
{
	int ret = 0;
	unsigned char data[KEY_NUM * 4];
	int threshold;

	VFI("The input arg is which = %d gear = %d", which, gear);

	if (gear < 1 || gear > KEY_THRESHOLD_GEAR_MAX) {
		VFE("The input threshold gear is out of range");
		ret = -EPERM;
		goto END;
	}
	if (which >= KEY_NUM) {
		VFE("Key value is out of range");
		ret = -EPERM;
		goto END;
	}

	wake_up_fw();
	ret = cs_i2c_read(IIC_BUTTON_FORCE_THR, data, KEY_NUM * 4);
	if (ret < 0) {
		VFE("read threshold_addr fail ret = %d", ret);
		goto END;
	}

	threshold = KEY_THRESHOLD_DEFAULT[gear - 1];
	data[which * 4] = threshold & 0xFF;
	data[which * 4 + 1] = (threshold >> 8) & 0xFF;
	threshold = KEY_THRESHOLD_DEFAULT[gear - 1] * 3 / 5;
	data[which * 4 + 2] = threshold & 0xFF;
	data[which * 4 + 3] = (threshold >> 8) & 0xFF;
	
	wake_up_fw();
	ret = cs_i2c_write(IIC_BUTTON_FORCE_THR, data, sizeof(data));
	if (ret < 0) {
		VFE("write threshold_addr fail ret = %d", ret);
		goto END;
	}
	ret = cs_handshake_done(IIC_BUTTON_FORCE_THR);
	if (ret < 0) {
		VFE("handshake is error ret = %d", ret);
		goto END;
	}
	VFI("change threadhold success");
	
END:
	return ret;
}

#endif

/*******************************************************	
Function:
	the interface of change update baseline time.

Input:
	the time to change.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_get_key_threshold(unsigned char *threshold)
{
	int ret = 0;
	int i;
	int temp[KEY_NUM * 2];
	unsigned char threshold_temp[KEY_NUM * 4];
	wake_up_fw();
	ret = cs_i2c_read(IIC_BUTTON_FORCE_THR, threshold_temp, KEY_NUM * 4);
	if (ret < 0) {
		VFE("read threshold_addr fail ret = %d", ret);
		goto END;
	}

	for (i = 0 ; i < KEY_NUM; i++) {
		temp[i * 2] = (int)(threshold_temp[i * 4] & 0xFF) | (int)((threshold_temp[i * 4 + 1] & 0xFF) << 8);
		temp[i * 2 + 1] = (int)(threshold_temp[i * 4 + 2] & 0xFF) | (int)((threshold_temp[i * 4 + 3] & 0xFF) << 8);
		VFI("key %d down threshold:%05d, up threshold:%05d", i + 1, temp[i], temp[i + 1]);
		ret += snprintf(threshold + i * 34, 255, "key %02d down:%05d\n", i + 1, temp[i * 2]);
		ret += snprintf(threshold + i * 34 + 18, 255, "key %02d up:%05d\n", i + 1, temp[i * 2 + 1]);
	}
	ret += snprintf(threshold + i * 34, 255, "\n");
END:
	return ret;

}

/*******************************************************	
Function:
	the interface of change threadhold of down or up.

Input:
		which = 0 : volume up  1 : power key  2 : volume down
		downup = 0 : down 1 : up
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_change_key_threshold (int which, int downup, int threshold)
{
	int ret = 0;
	unsigned char data[KEY_NUM * 4];

	VFI("The input arg is which = %d  downup = %d  threshold = %d", which, downup, threshold);

	if (threshold < 50 || threshold > 300) {
		VFE("The input threshold is out of range");
		ret = -EPERM;
		goto END;
	}
	if (which >= KEY_NUM) {
		VFE("Key value is out of range");
		ret = -EPERM;
		goto END;
	}
	if (downup > 1) {
		VFE("Key operation value is out of range");
		ret = -EPERM;
		goto END;
	}
	wake_up_fw();
	ret = cs_i2c_read(IIC_BUTTON_FORCE_THR, data, KEY_NUM * 4);
	if (ret < 0) {
		VFE("read threshold_addr fail ret = %d", ret);
		goto END;
	}

	data[which * 4 + downup * 2] = threshold & 0xFF;
	data[which * 4 + downup * 2 + 1] = (threshold >> 8) & 0xFF;

	wake_up_fw();
	ret = cs_i2c_write(IIC_BUTTON_FORCE_THR, data, sizeof(data));
	if (ret < 0) {
		VFE("write threshold_addr fail ret = %d", ret);
		goto END;
	}
	ret = cs_handshake_done(IIC_BUTTON_FORCE_THR);
	if (ret < 0) {
		VFE("handshake is error ret = %d", ret);
		goto END;
	}
	VFI("change threadhold success");
	
END:
	return ret;
}


static int cs_set_state(unsigned char *app_name)
{
	int status = atomic_read(&ap_status_landscape);
	
	if (strcmp(app_name, "ROTATION:0") == 0 || strcmp(app_name, "ROTATION:2") == 0) {
		atomic_set(&ap_status_landscape, status | AP_STATUS_OPT_LANDSCAPE);
	} else if (strcmp(app_name, "ROTATION:1") == 0) {
		atomic_set(&ap_status_landscape, status & ~AP_STATUS_OPT_LANDSCAPE);
	} else if (strcmp(app_name, "CALLING") == 0) {
		atomic_set(&ap_status_landscape, status | AP_STATUS_OPT_CALLING);
	} else if (strcmp(app_name, "CALL_NONE") == 0) {
		atomic_set(&ap_status_landscape, status & ~AP_STATUS_OPT_CALLING);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int cs_get_state(void)
{
	return atomic_read(&ap_status_landscape);
}

/*******************************************************	
Function:
	the interface of process appname from service.

Input:
	app_name
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_process_package(unsigned char *app_name)
{
	int ret = 0;
	
	//VFI("app name from service is %s", app_name);

	ret = cs_set_state(app_name);
	if (ret < 0) {
		VFE("cs set state failed");
		goto cs_process_ret;
	}

	if (cs_get_state()) {
		VFI("enable optimize status, ap_status_landscape = %d", atomic_read(&ap_status_landscape));
		ret = cs_ap_status_set(AP_STATUS_OPT, 1);
		if (ret < 0) {
			VFE("cs set ap_status failed");
			goto cs_process_ret;
		}
	} else {
		VFI("disable optimize status, ap_status_landscape = %d", atomic_read(&ap_status_landscape));
		ret = cs_ap_status_set(AP_STATUS_OPT, 0);
		if (ret < 0) {
			VFE("cs set ap_status failed");
			goto cs_process_ret;
		}
	}

cs_process_ret:
	return ret;
}

/*******************************************************	
Function:
	the driver of i2c probe.

Input:
	client: i2c_client struct.
	id: i2c_device_id struct.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int __cs_probe(struct i2c_client *client)
{
	int ret;
	struct VForce *vfd = NULL;

	VFI("start");

	vfd = vForceAlloc();
	if (vfd == NULL) {
		VFI("vForceAlloc fail");
		return 0;
	}
	vfd->client = client;
	vfd->getChipFwOrCfgVer = cs_fw_get_version;
	vfd->getRawOrDiffData = cs_get_rawdiff;
	vfd->updateFirmware = force_update_fw;
	vfd->getKeyThreshold = cs_get_key_threshold;
	vfd->changeKeyThreshold = cs_change_key_threshold;
	#ifdef CS_PRESS_COEF
	vfd->getCaliCoef = cs_get_calibration_coefficients;
	vfd->changeCaliCoef = cs_change_calibration_coefficients;
	#else
	vfd->getCaliCoef = cs_get_threshold_gear;
	vfd->changeCaliCoef = cs_set_threshold_gear;
	#endif
	vfd->processByPackage = cs_process_package;
	ret = vForceInit(vfd);
	if (ret < 0) {
		VFE("vForce init fail");
		goto cs_misc_register_error;
	}

	ret = misc_register(&cs_misc);
	if (ret < 0) {
		VFE("misc register fail");
		goto cs_misc_register_error;
	}

	g_cs_client = client;

	ret = cs_sys_create();
	if (ret < 0) {
		VFE("cs create sys/cs_press/cs_press fail");
		goto cs_sys_create_error;
	}

	ret = parse_dt(client);
	if (ret < 0) {
		VFE("product not exist!");
		goto cs_parse_dt_error;
	}

	if (product_name && !(strcmp(product_name, "EXP1933"))) {
		VFI("load EXP1933 firmware.\n");
		cs_press_f = cs_pt101_fw_1933_hex;
	} else {
		cs_press_f = cs_pt101_fw_hex;
	}

	ret = check_fw_err();
	if (ret < 0) {
		VFE("check firmware error");
		goto cs_check_fw_error;
	}
#ifdef INT_SET_EN
	eint_init();
#endif
	wakeup_source_init(&cs_wakelock, "cs_i2c_check_resume_wakelock");
	INIT_WORK(&i2c_check_resume_worker, resume_i2c_check_func);
	
	INIT_DELAYED_WORK(&update_worker, update_work_func);
	schedule_delayed_work(&update_worker, msecs_to_jiffies(2000));
	VFI("update_work_func start,delay 2s.\n");

#ifdef I2C_CHECK_SCHEDULE
	INIT_DELAYED_WORK(&i2c_check_worker, i2c_check_func);
	schedule_delayed_work(&i2c_check_worker, msecs_to_jiffies(CHECK_DELAY_TIME));
	VFI("i2c_check_func schedule work, delay 10 min.");
#endif

	cs_rst_set();
	VFI("end!");
	return 0;

cs_check_fw_error:
#ifdef INT_SET_EN
	if (gpio_is_valid(cs_press_irq))
		gpio_free(cs_press_irq);
#endif
	if (gpio_is_valid(cs_press_rst))
		gpio_free(cs_press_rst);

cs_parse_dt_error:
	if (cs_kobj != NULL) {
		sysfs_remove_group(cs_kobj, &cs_sys_attr_group);
	}
cs_sys_create_error:
	if (cs_kobj != NULL) {
		kobject_del(cs_kobj);
	}
	misc_deregister(&cs_misc);

cs_misc_register_error:
	vForceDeinit();

	return ret;
}

static int cs_key_init_thread(void *dev)
{
	return __cs_probe((struct i2c_client *)dev);
}

static int cs_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	kthread_run(cs_key_init_thread, client, dev_name(&client->dev));
	return 0;
}

/*******************************************************	
Function:
	the driver of remove device.

Input:
	client: i2c_client struct.
	
Output:
	0:success.
*********************************************************/
static int cs_remove(struct i2c_client *client)
{
	if (NULL == g_cs_client)
		return 0;

	i2c_unregister_device(g_cs_client);
	misc_deregister(&cs_misc);
	g_cs_client = NULL;

#ifdef INT_SET_EN
	if (gpio_is_valid(cs_press_irq))
		gpio_free(cs_press_irq);
#endif
	if (gpio_is_valid(cs_press_rst))
		gpio_free(cs_press_rst);

	if (cs_kobj != NULL) {
		sysfs_remove_group(cs_kobj, &cs_sys_attr_group);
		kobject_del(cs_kobj);
	}
	cancel_work_sync(&i2c_check_resume_worker);
	cancel_delayed_work_sync(&update_worker);
#ifdef I2C_CHECK_SCHEDULE
	cancel_delayed_work_sync(&i2c_check_worker);
#endif
	vForceDeinit();

	return 0;
}

/*******************************************************	
Function:
	the driver of sleep.

Input:
	device: device struct.
	
Output:
	0:success.
*********************************************************/
static int cs_suspend(struct device *device)
{
	i2c_suspend = 1;
	
#ifdef I2C_CHECK_SCHEDULE
	VFI("i2c_check_func cancel");
#endif

	return 0;
}

/*******************************************************	
Function:
	the driver of wake up.

Input:
	device: device struct.
	
Output:
	0:success.
*********************************************************/
static int cs_resume(struct device *device)
{
	i2c_suspend = 0;
	
#ifdef I2C_CHECK_SCHEDULE
	VFI("i2c_check_func schedule work, delay 10 min.");
#endif

	schedule_work(&i2c_check_resume_worker);

	return 0;
}

static const struct dev_pm_ops cs_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cs_suspend, cs_resume)
};


static const struct i2c_device_id cs_id_table[] = {
	{CS_CHRDEV_NAME, 0},
	{},
};

static struct of_device_id cs_match_table[] = {
	{ .compatible = "cs_ndt_press,pt101", },
	{},
};
MODULE_DEVICE_TABLE(i2c, cs_id_table);

static struct i2c_driver cs_driver = {
	.id_table = cs_id_table,
	.probe = cs_probe,
	.remove = cs_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = CS_CHRDEV_NAME,
		.of_match_table = cs_match_table,
		.pm = &cs_pm_ops,
	},
};
#if 0
extern int get_survivalmode_value(void);
extern int get_recoverymode_value(void);
#endif
/*******************************************************	
Function:
	the driver of module_init.

Input:
	none.
	
Output:
	0:success.
*********************************************************/
static int __init cs_init(void)
{
	int ret;

	VFI("start.");
#if 0
	VFI("survivalmode_value is %d, recoverymode_value is %d", get_survivalmode_value(), get_recoverymode_value());
	if ((get_survivalmode_value() == 1) || (get_recoverymode_value() == 1)) {
		VFI("It is recoverymode");
		return 0;
	}
#endif
	ret = i2c_add_driver(&cs_driver);
	if (ret < 0)
		VFE(" i2c_add_driver fail,status=%d\n", ret);

	return 0;
}

/*******************************************************	
Function:
	the driver of module_exit.

Input:
	none.
	
Output:
	none.
*********************************************************/
static void __exit cs_exit(void)
{
	i2c_del_driver(&cs_driver);
}

module_init(cs_init);
module_exit(cs_exit);
