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
#include "../vivo_force/vivo_force.h"
#include <linux/bbk_drivers_info.h>
#include <linux/vivo_touchscreen_config.h>



#ifdef CONFIG_MTK_I2C_EXTENSION
#include <linux/dma-mapping.h>
#endif

/******* Macro definition **********/
#define LOG(fmt, args...) printk("[cs_press] [%s: %d] "fmt,  __func__, __LINE__, ##args)

#define FW_PATH		"/sdcard/cs_press/pressure_f61.nfw"
#define FW_PATH_1	"/data/cs_press/pressure_f61.nfw"

#define READ_FW_ROM_TO_FILE "/sdcard/cs_press_fw.txt"

#define CS_CHRDEV_NAME "cs_press_pt100"
#define CS_I2C_ADDR		0x50 /* (0xA0>>1) */


/*IIC REG*/
#define IIC_EEPROM			0x00
#define IIC_TEST			0x00
#define IIC_HANDSHAKE     	0x01
#define IIC_CMD     		0x02
#define IIC_COUNT_INFO		0x06

#define IIC_MCU_ID      	0x0B
#define IIC_RSTSRC			0x0C
#define IIC_PROJECT_ID    	0x0F
#define IIC_FW_VER       	0x10

#define IIC_WAKE_UP      0x06
#define IIC_SLEEP        0x07
#define IIC_GREEN        0x08
#define IIC_GREEN2NORMAL 0x10

#define IIC_AFE_GAIN     0x16
#define IIC_DAC_VALUE     0x17
#define IIC_ADC_UVOLT_COEF     0x1B
#define IIC_DAC_UVOLT_COEF     0x1C

#define IIC_SENSOR_CONN_INFO	0x29

#define IIC_GPIO_STATUS		0x2E

#define IIC_TEMPERATURE 0x54
#define IIC_SCAN_MODE_SWITCH	0x56
#define IIC_FUN_SWITCH	0x57
#define IIC_BUTTON_FORCE_THR	0x5B
#define IIC_BUTTON_POS	0x5C
#define IIC_SLIDE_FORCE_THR	0x60

#define IIC_EVENT_STATUS		0xAB

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

#define CHECK_DELAY_TIME	(20000)
#define LONG_DELAY		(200000) 
#define SHORT_DELAY		(20000)
#define I2C_CHECK_SCHEDULE
#define INT_SET_EN

#define POWER_KEY		(1UL << 2) 
#define VOLUP_KEY		(1UL << 0)     
#define VOLDOWN_KEY		(1UL << 1)

/******* value definition **********/
unsigned short g_cs_reg;
static struct i2c_client *g_cs_client;
//struct pinctrl *cs_press_pinctrl;
//struct pinctrl_state *cs_press_rst0;
//struct pinctrl_state *cs_press_rst1;

static struct mutex	i2c_rw_lock;
static DEFINE_MUTEX(i2c_rw_lock);

static int defail_reset_mode = 1;

/*values for read/write any register.*/
static unsigned char read_reg_data[64];
static int read_reg_len;

#ifdef I2C_CHECK_SCHEDULE
struct delayed_work i2c_check_worker;
#endif
struct work_struct update_worker;

static int press_threshold[2] = { 60, 50, };
static int debug_mode;
static unsigned int cali_param[1] = {100};
static int cali_channel;


#ifdef INT_SET_EN
static DECLARE_WAIT_QUEUE_HEAD(cs_press_waiter);
static int cs_press_int_flag;
static int cs_press_irq;
static int cs_press_rst;
/*1 enable,0 disable,  need to confirm after register eint*/
static int cs_irq_flag = 1;
struct input_dev *ndt_input_dev;
#endif

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
static int i2c_suspend;
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
	
	while(i2c_suspend > 0) {
		VFI("warning:iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if (retry_count > 10) {
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
	
	while(i2c_suspend > 0) {
		VFI("warning:iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 50) {
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
	} while ( i> 0);
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
	
	while(i2c_suspend > 0) {
		VFI("warning:iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 50) {
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
	
	while(i2c_suspend > 0) {
		VFI("warning:iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 50) {
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
				for (i = 0; i < byte_len; i++){
					
					err_len += sprintf(err_buf + err_len,
						"%02x ", read_buf[pos + i]);
					if( i%err_line == 0)
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
				for (i = 0; i < byte_len; i++){
					err_len += sprintf(err_buf + err_len,
						"%02x ", read_buf[pos + i]);
					if( i%err_line == 0)
						err_len += sprintf(err_buf + err_len, "\n");
				}
				err_len += sprintf(err_buf + err_len, "\n");
				VFI("err_len = %d", err_len);
				VFI("buf=%s", err_buf);

				goto I2C_BAD_OUT;
			}
			pos += byte_len;
			reg++;
			if ((page_end > 0) && (reg >= len/256))
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
		if(retry_count == 0)
			VFI("wake up retry %d.\n", retry_count);
	} while(ret <= 0 && retry_count > 0);
	
	return ret;
}

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

	wake_up_fw();
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
		if(ic_fw_ver[3] >= file_fw_ver[3]){
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
/*******************************************************	
Function:
	fw update from .h array hex.

Input:
	
Output:
	>=0:success;  <0:fail.
*********************************************************/
static int cs_fw_update_array_h(void)
{
	loff_t pos;
	int len = 0;
	unsigned char ic_fw_ver[CS_FW_VER_LEN] = {0};
	unsigned char file_fw_ver[CS_FW_VER_LEN] = {0};
	int ret = 0;
	vForceGetData()->fwUpdatingFlag = 1;
	wake_up_fw();
	ret = cs_i2c_read(IIC_FW_VER, ic_fw_ver, sizeof(ic_fw_ver));
	if (ret < 0) {
		VFE("CS:::fail to read firmware version");
		goto close_file_out;
	}

	pos = 8;
	memcpy(file_fw_ver, &cs_default_fw_hex[pos], sizeof(file_fw_ver));

	*(unsigned short *)file_fw_ver = swab16(*(unsigned short *)file_fw_ver);
	if (*(unsigned short *)ic_fw_ver == *(unsigned short *)file_fw_ver) {
		VFE("CS:::[current fw:%x][previous:%x]",
			*(unsigned short *)ic_fw_ver,
			*(unsigned short *)file_fw_ver);
		goto close_file_out;
	}
	VFI("CS:[current fw:%x][file fw:%x]",
		*(unsigned short *)ic_fw_ver, *(unsigned short *)file_fw_ver);

	pos = 0x0c;
	memcpy((char *)&len, &cs_default_fw_hex[pos], sizeof(len));

	len = swab32(len);
	VFI("CS:[file fw len:%x]", len);

	pos = 0x100;

	ret = burn_fw(&cs_default_fw_hex[pos], len);
	if (ret < 0)
		VFE("CS:::Burning firmware fails");

close_file_out:
	vForceGetData()->fwUpdatingFlag = 0;

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
		VFI("R calibrate_param:%d\n", m_data);
		return 1;
	}
	return 0;
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
	unsigned char rbuf[2];
	unsigned char addr;
	int len = 0;

	addr = 0x03;
	len = 1;
	rbuf[0] = 0;
	rbuf[1] = 0;

	do {
		wake_up_fw();
		if (cs_i2c_read(addr, rbuf, len) >= 0)
			return 1;
		retry--;
		VFE("read fw ver fail!retry:%d\n", retry);
	} while (retry > 0);

	retry = 3;
	do {
		cs_reset(2);
		msleep(300);
		if (cs_i2c_read(addr, rbuf, len) >= 0)
			return 1;
		retry--;
		VFE("reset fw fail!retry:%d\n", retry);
	} while (retry > 0);

	return -1;
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
			result = cs_fw_update_array_h();
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
	ret = check_i2c();
	if(ret != -1){
		schedule_delayed_work(&i2c_check_worker, msecs_to_jiffies(vForceGetData()->i2cCheckDelay));
		VFI("i2c_check_func start,delay %ds.\n", vForceGetData()->i2cCheckDelay/1000);
	}
}
#endif

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
	ret =  sprintf(buf, "%d\n",cs_irq_flag);
	VFI("irq flag=%d\n",cs_irq_flag );
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
	if (buf[0] == '0'){
		cs_irq_disable();
		VFI("disable irq\n");
	} else if (buf[0] == '1'){
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
			VFI("open file error\n");
		return -1;
		}
	}
	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0x0c;
	vfs_read(fp, (char *)&len, 4, &pos);
	len = swab32(len);
	VFI("fw len = %d\n", len);
	if(len <= 0 || len > FW_UPDATA_MAX_LEN){
		VFI("[err! len overflow!len=%x]\n", len);
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
	
	
	VFI("fw_info:%s\n",buf);
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
static int set_debug_mode(unsigned char addr,unsigned char data)
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
	reg_addr = IIC_DEBUG_READY2;
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
static int get_debug_data(unsigned char addr,unsigned char* data, int len)
{
	int ret;
	unsigned char reg_addr;
	unsigned char *reg_data;
	int i;
	reg_data = kzalloc(len, GFP_KERNEL);
	if (!reg_data) {
		return -ENOMEM;
	}
	reg_addr = addr;
	reg_data[0] = 0;
	ret = cs_i2c_read(reg_addr, reg_data, len);
	if (ret <= 0) {
		VFE("reg=%d, data=%d, len=%d, err\n",
			reg_addr, reg_data[0], len);
	}
	for(i=0;i<len;i++)
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

	set_debug_mode(IIC_DEBUG_MODE2, DEBUG_RAW_MODE);
	i=50;
	do {
		len = get_debug_data_ready((unsigned char)IIC_DEBUG_READY2);
		VFI("len:%d\n", len);
		if (len > 0) {
			ret = get_debug_data(IIC_DEBUG2_DATA, reg_data, len);
			VFI("D0:%d,D1:%d,D2:%d,len:%d\n", reg_data[0], reg_data[1], reg_data[2], len);
		}
		i--;
	} while(len == 0 && i > 0);

	ret = 0;
	if(len > 40)
		len = 40;
	for (i = 0; i < len/2; i++) {
		raw_data[i] = ((int)(reg_data[i*2] & 0xff)
			| ((int)(reg_data[i*2 + 1] & 0xff) << 8));

		ret += sprintf(buf + 6 * i, "%05d ", raw_data[i]);
		VFI("raw_data %d=%d\n", i, raw_data[i]);
	}

	ret += sprintf(buf + 6 * i, "\n");
	VFI("buf=%s\n", buf);

	reg_addr = IIC_DEBUG_MODE2;
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
	
	set_debug_mode(IIC_DEBUG_MODE2, DEBUG_DIFF_MODE);
	i=50;
	do {
		len = get_debug_data_ready((unsigned char)IIC_DEBUG_READY2);
		VFI("len:%d\n", len);
		if (len > 0) {
			ret = get_debug_data(IIC_DEBUG2_DATA, reg_data, len);
			VFI("D0:%d,D1:%d,D2:%d,len:%d\n", reg_data[0], reg_data[1], reg_data[2], len);
		}
		i--;
	} while(len == 0 && i > 0);
	
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

	if(buf[0] == '0')
		val = 0;
	else if(buf[0] == '1')
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

	if(buf[0] == '0')
		val = 0;
	else if(buf[0] == '1')
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

	int change_val[2]={100,0};
	
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

	VFI("debug_mode:%d,cali:%d \n", debug_mode, cali_param[0]);

	ret = sprintf(buf, "%d\n", cali_param[0]);

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
		VFI("fail to read handshake!\n");

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
	VFI("handshake:%02X %02X\n", handshake[0],handshake[1]);

	wake_up_fw();
	datbuf[0] = (unsigned char)(handshake[0]&0xff);
	datbuf[1] = (unsigned char)(handshake[1]&0xff);
	ret = cs_i2c_write(IIC_HANDSHAKE, datbuf, 2);

	return count;
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
	ssize_t ret = 0;
	unsigned char datbuf[2];
	unsigned int cmd = 0; 
	
	ret = sscanf(buf, "%x", &cmd);
	VFI("cmd:%02x\n", cmd);

	wake_up_fw();
	datbuf[0] = (unsigned char)(cmd&0xff);
	datbuf[1] = (unsigned char)((0-cmd)&0xff);
	ret = cs_i2c_write(IIC_CMD, datbuf, 2);

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
	for (i = 0; i < 6;i++ ){
		ret += sprintf(buf + 5 * i, "%04x ",
		((button_force_th[i*2]&0xff)+((button_force_th[i*2+1]<<8)&0xff00)));
	}
	//VFI("th buf:%s\n",buf);

	ret += sprintf(buf+ret, "\n");
	VFI("th buf:%s\n",buf);
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

	for(i=0;i<6;i++ ){
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
	for (i = 0; i < gpio_num;i++ ){
		ret += sprintf(buf + 5 * i, "%04x ",
		((gpio_status[i*2+1]&0xff)+((gpio_status[i*2]<<8)&0xff00))); //just for show H+L;
	}
	//VFI("th buf:%s\n",buf);

	ret += sprintf(buf+ret, "\n");
	VFI("gpio status:%s\n",buf);
	return ret;
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




static DEVICE_ATTR(force_update_fw, S_IRUGO | S_IWUSR,
		cs_force_update_fw_show, cs_force_update_fw_store);
static DEVICE_ATTR(fw_info, S_IRUGO , cs_fw_info_show, NULL);
static DEVICE_ATTR(get_rawdata, S_IRUGO , cs_get_rawdata_show, NULL);
static DEVICE_ATTR(get_forcedata, S_IRUGO , cs_get_forcedata_show, NULL);
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
static DEVICE_ATTR(cmd, S_IWUSR,
	NULL, cs_cmd_store);
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
static DEVICE_ATTR(temperature, S_IRUGO | S_IWUSR,
	cs_temperature_show, cs_temperature_store);

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
	&dev_attr_temperature.attr,
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
	1:success.
*********************************************************/
static int cs_sys_create(void)
{
	int ret = 0;
	struct kobject *kobj;

	kobj = kobject_create_and_add("cs_press", NULL);
	if (kobj == NULL) {
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_group(kobj, &cs_sys_attr_group);
	return 1;
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

	cs_irq_disable();
	wake_up_interruptible(&cs_press_waiter);

	return IRQ_HANDLED;
}

/*******************************************************	
Function:
	read the key event status.

Input:
	none.
	
Output:
	return event status.
*********************************************************/
unsigned char get_key_event(void){
	unsigned char rbuf[1];
	unsigned char addr;
	int len = 0;
	
	addr = IIC_KEY_EVENT;
	len = 1;
	rbuf[0] = 0x0;
	if (cs_i2c_read(addr, rbuf, len) <= 0) {
		VFI("reg=%d,buf[0]=%d,len=%d,err\n",
		addr, rbuf[0], len);
	}
	return rbuf[0];

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
		wait_event_interruptible(cs_press_waiter,
			cs_press_int_flag != 0);
		VFI("cs_press_event_handler enter wait\n");
		cs_press_int_flag = 0;
/*doing something in interrupt you want*/

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

	ndt_input_dev = input_allocate_device();
	if (ndt_input_dev != NULL) {
		ndt_input_dev->name = CS_CHRDEV_NAME;
		__set_bit(EV_KEY, ndt_input_dev->evbit);
		__set_bit(KEY_HOME, ndt_input_dev->keybit);
		__set_bit(KEY_VOLUMEUP, ndt_input_dev->keybit);
		__set_bit(KEY_VOLUMEDOWN, ndt_input_dev->keybit);
		__set_bit(KEY_POWER, ndt_input_dev->keybit);
		ret = input_register_device(ndt_input_dev);
		if (ret != 0)
			VFE("input register device error = %d", ret);
	}
}
#endif

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

#ifdef INT_SET_EN
	cs_press_irq = of_get_named_gpio((&pdev->dev)->of_node,"press,irq-gpio",0);
    if(!gpio_is_valid(cs_press_irq)) {
        dev_err(&pdev->dev, "cs_press request_irq IRQ fail");
    }
    else
    {
        ret = gpio_request(cs_press_irq, "press,irq-gpio");
        if(ret) {
            dev_err(&pdev->dev, "cs_press request_irq IRQ fail !,ret=%d.\n", ret);
        }
        ret = gpio_direction_input(cs_press_irq);
        msleep(50);
        cs_press_irq = gpio_to_irq(cs_press_irq);
    }
#endif
	cs_press_rst = of_get_named_gpio((&pdev->dev)->of_node,"press,rst-gpio",0);
  	if(!gpio_is_valid(cs_press_rst)) {
		dev_err(&pdev->dev, "cs_press request_rst fail");
	} else {
		ret = gpio_request(cs_press_rst, "press,rst-gpio");
	        if(ret) {
	            dev_err(&pdev->dev, "cs_press request rst fail !,ret=%d.\n", ret);
	        }
	        ret = gpio_direction_output(cs_press_rst,0);
	        msleep(50);
		gpio_set_value(cs_press_rst, 0);
	}
	VFI("end---\n");
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
	if(gpio_is_valid(cs_press_rst)) {
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
	if(gpio_is_valid(cs_press_rst)) {
		rt_mutex_lock(&g_cs_client->adapter->bus_lock);

		gpio_set_value(cs_press_rst, 1);
		VFI("rst set 1\n");
		msleep(10);
		gpio_set_value(cs_press_rst, 0);
		VFI("rst set 0\n");

		rt_mutex_unlock(&g_cs_client->adapter->bus_lock);
	}
}
/*---add gpio setting end---*/
/*******************************************************
Function:
	AI key callback.

Input:
	 .
	
Output:
	none.
*********************************************************/
static int cs_key_int_switch (unsigned char state);
static void cs_input_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	if (code == 600) {
		if (value == 1)
			cs_key_int_switch(3);
		else if (value == 0)
			cs_key_int_switch(4);
		
	}
}
static int cs_input_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret = -1;

	if ((strcmp(dev->name, "gpio-keys")))  {
		VFI("%s: %s: Not AI key input device\n", 
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

/*******************************************************	
Function:
	the thread of auto update fw.

Input:
	worker: work_struct .
	
Output:
	none.
*********************************************************/
static void update_work_func(struct work_struct *worker)
{
	cs_fw_update_array_h();
	if (cs_input_register()) {
		VFE("register input handler failed");
	}
}
/*******************************************************	
Function:
	the interface wait for the change of register.

Input:
	void.
	
Output:
	retry.
*********************************************************/
static int wait_for_ready(void)
{
	unsigned char data[2];
	int retry = 10;
	unsigned char switch_addr = 0x01;
	
	do {
		retry--;
		msleep(10);
		cs_i2c_read(switch_addr, data, 2);
		VFI("data read from switch_addr is %d, %d", data[0], data[1]);
		if (!data[0] && !data[1])
			break;
		
	} while (retry);

	return retry;
}
/*******************************************************	
Function:
	the interface of get fw version.

Input:
	void.
	
Output:
	fw_ver.
*********************************************************/
static unsigned char* cs_fw_get_version(void)
{
	ssize_t ret = 0;
	static unsigned char fw_ver[CS_FW_VER_LEN] = {0};

	wake_up_fw();
	ret = cs_i2c_read(IIC_FW_VER, fw_ver, sizeof(fw_ver));
	if (ret < 0) {
		VFE("fail to read firmware version\n");
	} else {
		VFE("fw version:%02x %02x %02x %02x", fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3]);
	}
	return fw_ver;
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
	unsigned char data[2];
	unsigned char coef_addr = 0xB5;
	unsigned char switch_addr = 0x01;
	unsigned char switch_data[2] = {0xB5,0x4B};

	VFI("The input arg is which = %d  coef = %d", which, coef);
	if (coef < 7 || coef > 13) {
		VFE("The input coefficients is out of range");
		ret = -1;
		goto END;
	}
	mutex_lock(&(vForceGetData()->rw_mutex));
	VFI("change calibration func mutex lock");
	wake_up_fw();
	ret = cs_i2c_read(coef_addr, data, 2);
	if (ret < 0) {
		VFE("read coef_addr fail ret = %d", ret);
		goto END;
	}
	if (which == 0) {
		data[0] = coef & 0xf;
	} else if (which == 1) {
		data[1] = coef & 0xf;
	}	
	ret = cs_i2c_write(coef_addr, data, 2);
	if (ret < 0) {
		VFE("write coef_addr fail ret = %d", ret);
		goto END;
	}
	ret = cs_i2c_write(switch_addr, switch_data, 2);
	if (ret < 0) {
		VFI("write switch_addr fail ret = %d", ret);
		goto END;
	}
	wait_for_ready();
	VFI("change calibration coefficients success");
	
END:
	VFI("change calibration func mutex unlock");
	mutex_unlock(&(vForceGetData()->rw_mutex));
	return ret;
}
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
	unsigned char coef_addr = 0xB5;
	unsigned char res[2];
	wake_up_fw();
	ret = cs_i2c_read(coef_addr, res, 2);
	if (ret < 0) {
		VFE("read coef_addr fail ret = %d", ret);
		goto END;
	}
	VFI("left key coef: %05d", res[0]);
	VFI("right key coef: %05d ", res[1]);
	ret += snprintf(coef, 255, "left: %05d ", res[0]);
	ret += snprintf(coef + 12, 255, "right: %05d", res[1]);
END:
	return ret;
}
/*******************************************************	
Function:
	the interface of change the state of key interrupt.

Input:
	enable or disable key interrupt.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_key_int_switch (unsigned char state)
{
	int ret = 0;
	VFI("The input state is %d", state);
	/*3 is open by AI key, 4 is close by AI key*/
	if (state != 1 && state != 0 && state != 3 && state != 4) {
		VFE("The input arg is invalid");
		ret = -1;
		goto END;
	}
	if (state == 3 || state == 4) {
		if (vForceGetData()->lastIntFlag == 1) {
			VFE("still in game mode");
			ret = -1;
			goto END;
		}
	}
	if (state == 4) {
		if (vForceGetData()->lastIntFlag == 0 || vForceGetData()->lastIntFlag == 4) {
			VFE("has been closed");
			ret = -1;
			goto END;
		}
	}
	if (vForceGetData()->fwUpdatingFlag == 1) {
		VFE("fw is updating");
		ret = -1;
		goto END;
	}
	vForceGetData()->intSwitch = state;
	VFI("acquire wake lock");
	__pm_stay_awake(&vForceGetData()->wakeLock);
	schedule_work(&vForceGetData()->switch_worker);

END:
	return ret;
}




static void change_switch_func(struct work_struct *worker)
{
	int ret = 0;
	unsigned char data;
	unsigned char flag;
	int state;
	unsigned char base_addr = 0x55;
	unsigned char switch_addr = 0x01;
	unsigned char switch_data[2] = {0x55, 0xAB};
	struct VForce *vfd = container_of(worker, struct VForce, switch_worker);
	state = vfd->intSwitch;

	if (state == 1 || state == 3) {
		flag = 1;
	} else {
		flag = 0;
	}

	mutex_lock(&(vfd->rw_mutex));
	VFI("switch func mutex lock");
	wake_up_fw();
	ret = cs_i2c_read(base_addr, &data, 1);
	if (ret < 0) {
		VFE("read base_addr fail ret = %d", ret);
		goto END1;
	}
	if (flag) {
		data |= 1UL << 1;
	} else {
		data &= ~(1UL << 1);
	}
	ret = cs_i2c_write(base_addr, &data, 1);
	if (ret < 0) {
		VFE("write base_addr fail ret = %d", ret);
		goto END1;
	}
	ret = cs_i2c_write(switch_addr, switch_data, 2);
	if (ret < 0) {
		VFE("write switch_addr fail ret = %d", ret);
		goto END1;
	}
	wait_for_ready();
	ret = cs_i2c_read(base_addr, &data, 1);
	if (ret < 0) {
		VFE("read base_addr fail ret = %d", ret);
		goto END1;
	}
	if (data & (1UL << 1)) {
		data = 1;
	} else {
		data = 0;
	}
	if (flag != data)
		goto END1;		
	VFI("change key int switch %d success", data);
	vForceGetData()->lastIntFlag = state;
	goto END;

END1:
	if (state == 1) {
		cs_reset(2);
		msleep(100);
	}
END:
	VFI("switch func mutex unlock");
	mutex_unlock(&(vfd->rw_mutex));
	VFI("relax wake lock");
	__pm_relax(&vfd->wakeLock);
	return;
}
/*******************************************************
Function:
	the interface of get calibration coefficients for game.

Input:
	**buf.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_get_key_switch_state(void)
{
	int ret = 0;
	unsigned char state = 0;
	unsigned char state_addr = 0x55;
	wake_up_fw();
	ret = cs_i2c_read(state_addr, &state, 1);
	if (ret < 0) {
		VFE("read state_addr fail ret = %d", ret);
		goto END;
	}
	if (state & 0x02)
		state = 1;
	else
		state = 0;
	VFI("get key int state %d", state);
	ret = state;
END:
	return ret;
}
/*******************************************************	
Function:
	the interface of get calibration coefficients for game.

Input:
	void.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_get_update_baseline_time(void)
{
	int ret = 0;
	unsigned char update_time_addr = 0x51;
	unsigned char temp[2];
	wake_up_fw();
	ret = cs_i2c_read(update_time_addr, temp, 2);
	if (ret < 0) {
		VFE("read update_time_addr fail ret = %d", ret);
		goto END;
	}
	ret = temp[0] | (temp[1] << 8);
	VFI("update baseline time: %05d ", ret);
END:
	return ret;
}
/*******************************************************	
Function:
	the interface of change update baseline time.

Input:
	the time to change.
	
Output:
	0:success.	<0:fail.
*********************************************************/
static int cs_change_update_baseline_time(int time)
{
	int ret = 0;
	unsigned char update_time_addr = 0x51;
	unsigned char switch_addr = 0x01;
	unsigned char switch_data[2] = {0x51, 0xAF};
	unsigned char data[2];
	data[0] = time & 0xff;
	data[1] = (time >> 8) & 0xff;
	mutex_lock(&(vForceGetData()->rw_mutex));
	VFI("change baseline func mutex lock");
	wake_up_fw();
	ret = cs_i2c_write(update_time_addr, data, 2);
	if (ret < 0) {
		VFE("write update_time_addr fail ret = %d", ret);
		goto END;
	}
	ret = cs_i2c_write(switch_addr, switch_data, 2);
	if (ret < 0) {
		VFI("write switch_addr fail ret = %d", ret);
		goto END;
	}
	wait_for_ready();
	VFE("change update_baseline_time %ds success", time);

END:
	VFI("change baseline func mutex unlock");
	mutex_unlock(&(vForceGetData()->rw_mutex));
	return ret;
}
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
	int temp[4];
	unsigned char threshold_addr = 0x5B;
	wake_up_fw();
	ret = cs_i2c_read(threshold_addr, threshold, 8);
	if (ret < 0) {
		VFE("read threshold_addr fail ret = %d", ret);
		goto END;
	}
	temp[0] = (int)(threshold[0] & 0xFF) | (int)((threshold[1] & 0xFF) << 8);
	temp[1] = (int)(threshold[2] & 0xFF) | (int)((threshold[3] & 0xFF) << 8);
	temp[2] = (int)(threshold[4] & 0xFF) | (int)((threshold[5] & 0xFF) << 8);
	temp[3] = (int)(threshold[6] & 0xFF) | (int)((threshold[7] & 0xFF) << 8);
	VFI("left key down threshold:%05d, up threshold:%05d", temp[0], temp[1]);
	VFI("right key down threshold:%05d, up threshold:%05d", temp[2], temp[3]);
	ret += snprintf(threshold, 255, "left down:%05d ", temp[0]);
	ret += snprintf(threshold + 16, 255, "left up:%05d", temp[1]);
	ret += snprintf(threshold + 32, 255, "left down:%05d", temp[2]);
	ret += snprintf(threshold + 48, 255, "left up:%05d", temp[3]);
	ret += snprintf(threshold + 64, 255, "\n");
END:
	return ret;

}

/*******************************************************	
Function:
	the interface of change threadhold of down or up.

Input:
		which = 0 : the left key  1 : the rght key
		downup = 0 : down 1 : up
	
Output:
	0:success.	<0:fail.
*********************************************************/

static int cs_change_key_threshold (int which, int downup, int threshold)
{
	int ret = 0;
	unsigned char data[8];
	unsigned char threshold_addr = 0x5B;
	unsigned char switch_addr = 0x01;
	unsigned char switch_data[2] = {0x5B, 0xA5};

	VFI("The input arg is which = %d  downup = %d  threshold = %d", which, downup, threshold);
	if (threshold < 30 || threshold > 200) {
		VFE("The input threshold is out of range");
		ret = -1;
		goto END;
	}
	mutex_lock(&(vForceGetData()->rw_mutex));
	VFI("change threshold func mutex lock");
	wake_up_fw();
	ret = cs_i2c_read(threshold_addr, data, 8);
	if (ret < 0) {
		VFE("read threshold_addr fail ret = %d", ret);
		goto END;
	}
	if (which == 0) {
		if (downup == 0) {
			data[0] = threshold & 0xFF;
			data[1] = threshold & 0xFF00;
		} else if (downup == 1) {
			data[2] = threshold & 0xFF;
			data[3] = threshold & 0xFF00;
		}
	} else if (which == 1) {
		if (downup == 0) {
			data[4] = threshold & 0xFF;
			data[5] = threshold & 0xFF00;
		} else if (downup == 1) {
			data[6] = threshold & 0xFF;
			data[7] = threshold & 0xFF00;	
		}
	}
	wake_up_fw();
	ret = cs_i2c_write(threshold_addr, data, sizeof(data));
	if (ret < 0) {
		VFE("write threshold_addr fail ret = %d", ret);
		goto END;
	}
	ret = cs_i2c_write(switch_addr, switch_data, 2);
	if (ret < 0) {
		VFE("write switch_addr fail ret = %d", ret);
		goto END;
	}
	wait_for_ready();
	VFI("change threadhold success");
	
END:
	VFI("change threshold func mutex unlock");
	mutex_unlock(&(vForceGetData()->rw_mutex));
	return ret;
}

/*******************************************************	
Function:
	the interface of game_mode_func.

Input:
	worker.
	
Output:
	void.
*********************************************************/

static void game_mode_func(struct work_struct *worker)
{
	struct VForce *vfd = container_of(worker, struct VForce, game_worker);
	if (vfd->gameMode == 1) {
		VFI("game in");
		cs_key_int_switch (1);
		cs_change_update_baseline_time(25000);
		VFI("change i2cCheckDelay SHORT_DELAY");
		vForceGetData()->i2cCheckDelay = SHORT_DELAY;
		if (1) {
			cancel_delayed_work_sync(&i2c_check_worker);
			schedule_delayed_work(&i2c_check_worker, msecs_to_jiffies(vForceGetData()->i2cCheckDelay));
		}

	} else if (vfd->gameMode == 0) {
		VFI("game out");
		cs_key_int_switch (0);
		cs_change_update_baseline_time(6000);
		VFI("change i2cCheckDelay LONG_DELAY");
		vForceGetData()->i2cCheckDelay = LONG_DELAY;
		if (1) {
			cancel_delayed_work_sync(&i2c_check_worker);
			schedule_delayed_work(&i2c_check_worker, msecs_to_jiffies(vForceGetData()->i2cCheckDelay));
		}

	}
}
static int cs_enter_game_mode_new(int state)
{
	vForceGetData()->gameMode = state;	
	schedule_work(&vForceGetData()->game_worker);
	return 0;
}

static int vts_game_mode;
static int vts_game_space_mode;

static int cs_process_package(unsigned char *app_name)
{
	VFI("app name from service is %s", app_name);
	if (!strcmp(app_name, "GAME_IN")) {
		VFI("enter game mode");
		vts_game_mode = 1;
	} else if (!strcmp(app_name, "GAME_OUT")) {
		VFI("out game mode");
		vts_game_mode = 0;
	} else if (!strcmp(app_name, "GAME_SPACE_IN")) {
		VFI("enter game space");
		vts_game_space_mode = 1;
	} else if (!strcmp(app_name, "GAME_SPACE_OUT")) {
		VFI("out game space");
		vts_game_space_mode = 0;
	} else {
		VFI("invalid app name!");
		return 0;
	}

	if (vts_game_mode == 1 || vts_game_space_mode == 1)
		cs_enter_game_mode_new(1);
	else if (vts_game_mode == 0 && vts_game_space_mode == 0)
		cs_enter_game_mode_new(0);
	
	return 0;
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
	vfd->updateFirmware = cs_fw_update;
	vfd->changeCaliCoef = cs_change_calibration_coefficients;
	vfd->getCaliCoef = cs_get_calibration_coefficients;
	vfd->keyIntSwitch = cs_key_int_switch;
	vfd->getKeySwitchState = cs_get_key_switch_state;
	vfd->getKeyThreshold = cs_get_key_threshold;
	vfd->changeKeyThreshold = cs_change_key_threshold;
	vfd->i2cCheckDelay = LONG_DELAY;
	vfd->processByPackage = cs_process_package;

	vForceInit(vfd);

	ret = misc_register(&cs_misc);

	g_cs_client = client;

	cs_sys_create();

	parse_dt(client);

	ret = check_fw_err();
	if (ret == 0 )
		return -1 ;


#ifdef INT_SET_EN
	eint_init();
#endif
	cs_rst_set();
	msleep(100);

	INIT_WORK(&update_worker, update_work_func);
	schedule_work(&update_worker);
	
	msleep(50);
	while (vfd->fwUpdatingFlag == 1) {
		msleep(10);
	}
	msleep(400);
	INIT_WORK(&vfd->game_worker, game_mode_func);
	INIT_WORK(&vfd->switch_worker, change_switch_func);
	cs_key_int_switch (0);
	cs_get_update_baseline_time();


#ifdef I2C_CHECK_SCHEDULE
	INIT_DELAYED_WORK(&i2c_check_worker, i2c_check_func);
	schedule_delayed_work(&i2c_check_worker, msecs_to_jiffies(vfd->i2cCheckDelay));
	VFI("i2c_check_func start,delay %ds.\n", vfd->i2cCheckDelay/1000);
#endif

	VFI("end!\n");
	return 0;

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

	wakeup_source_trash(&(vForceGetData()->wakeLock));
	i2c_unregister_device(g_cs_client);
	misc_deregister(&cs_misc);
	g_cs_client = NULL;
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
	cancel_delayed_work_sync(&i2c_check_worker);
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
	schedule_delayed_work(&i2c_check_worker, 0);
	VFI("i2c_check_func start.\n");
#endif

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
	{ .compatible = "cs_press_pt100,f61", },
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
	char *board_id = NULL;
	const char *product_name = NULL; 

	VFI("start.\n");
	vivo_touchscreen_get_product_name(&product_name);
	if (product_name && (!strcmp(product_name, "PD1955"))) {
		board_id = get_bbk_board_version();
		if (board_id && (board_id[4] == '1')) {
			VFI("no press key");
			return 0;
		}
	}

	ret = i2c_add_driver(&cs_driver);
	if (ret < 0)
		VFI(" i2c_add_driver fail,status=%d\n", ret);

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
