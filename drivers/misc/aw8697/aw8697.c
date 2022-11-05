/*
 * aw8697.c   aw8697 haptic module
 *
 * Version: v1.3.5
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li < liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#define DEBUG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/kthread.h>
#include <linux/kobject.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <uapi/linux/sched/types.h>
#endif

#include "aw8697.h"
#include "aw8697_reg.h"
#include "aw8697_config.h"
#include <linux/pm_qos.h>
#include <linux/wakelock.h>



/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW8697_I2C_NAME "aw8697_haptic"
#define AW8697_HAPTIC_NAME "awinic_haptic"

#define AW8697_VERSION "v1.3.5"


#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 3
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define AW8697_MAX_DSP_START_TRY_COUNT    10

#define PLAYBACK_INFINITELY_RAM_ID 6



#define AW8697_MAX_FIRMWARE_LOAD_CNT 20

#define OP_OCS_CALIBRATION_T_LENGTH 5000000

#define RTP_BIN_MAX_SIZE 2000000
#define RTP_SLAB_SIZE 512

struct pm_qos_request pm_qos_req_vb;

static struct kmem_cache *rtp_cachep;

static struct workqueue_struct *rtp_wq;

static int playRoll_flag; //用于寿命测试标志位
static bool rtp_check_flag; //防止rtp调用返回上层，刚执行work时候，中间来了一个cancel导致cancel没有效果，rtp仍然会继续播完
static volatile int i2c_suspend;


enum haptics_custom_effect_param {
	CUSTOM_DATA_EFFECT_IDX,
	CUSTOM_DATA_TIMEOUT_SEC_IDX,
	CUSTOM_DATA_TIMEOUT_MSEC_IDX,
	CUSTOM_DATA_LEN,
};


/******************************************************
 *
 * variable
 *
 ******************************************************/
//#define AW8697_RTP_NAME_MAX        64
static char *aw8697_ram_name = "aw8697_haptic.bin";
#if 0
static char aw8697_rtp_name[][AW8697_RTP_NAME_MAX] = {
	{"aw8697_rtp.bin"},
	{"aw8697_rtp_lighthouse.bin"},
	{"aw8697_rtp_silk.bin"},
};
#endif
struct aw8697_container *aw8697_rtp;
struct aw8697 *g_aw8697;
static char osc_cali_result[128];
static int osc_cali_result_mode;


static int at_test; //用于trigger硬件通路检测时候，屏蔽其它振动；1表示屏蔽，0表示不屏蔽

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw8697_interrupt_clear(struct aw8697 *aw8697);
static int aw8697_haptic_trig_enable_config(struct aw8697 *aw8697);
static int aw8697_haptic_set_gain(struct aw8697 *aw8697, unsigned char gain);
static int aw8697_ff_msg_send(struct aw8697 *aw8697, enum aw8697_ff_msg_type type, int arg1, struct aw8697_play_info *arg2);
static int aw8697_haptic_set_bst_vol(struct aw8697 *aw8697, unsigned char bst_vol);
static void aw8697_interrupt_setup(struct aw8697 *aw8697);
static void aw8697_vol_trig_switch(struct aw8697 *aw8697, bool sw);
static int aw8697_set_clock(struct aw8697 *aw8697, int clock_type);
static void aw8697_lra_get(struct aw8697 *aw8697, unsigned char *reg_val);
static int aw8697_haptic_set_wav_seq(struct aw8697 *aw8697, unsigned char wav, unsigned char seq);

 /******************************************************
 *
 * aw8697 i2c write/read
 *
 ******************************************************/
static int aw8697_i2c_write(struct aw8697 *aw8697,
		 unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw8697->bus_lock);
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw8697->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d, reg_addr=%#x, reg_data=%#x\n", __func__, cnt, ret, reg_addr, reg_data);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}
	//pr_debug("%s reg_addr=%#x, reg_data=%#x\n", __func__, reg_addr, reg_data);
	mutex_unlock(&aw8697->bus_lock);
	return ret;
}

static int aw8697_i2c_read(struct aw8697 *aw8697,
		unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw8697->bus_lock);
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw8697->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}
	mutex_unlock(&aw8697->bus_lock);

	return ret;
}

static int aw8697_i2c_write_bits(struct aw8697 *aw8697,
		 unsigned char reg_addr, unsigned int mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;
	int ret = 0;

	ret = aw8697_i2c_read(aw8697, reg_addr, &reg_val);
	if (ret < 0) {
		pr_err("%s i2c read failed, ret=%d\n", __func__, ret);
		return -EINVAL;
	}
	reg_val &= mask;
	reg_val |= reg_data;
	aw8697_i2c_write(aw8697, reg_addr, reg_val);

	return 0;
}

static int aw8697_i2c_writes(struct aw8697 *aw8697,
		unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	if ((rtp_cachep == NULL) || (len > RTP_SLAB_SIZE)) {

		data = kmalloc(len+1, GFP_KERNEL);
		if (data == NULL) {
			pr_err("%s: can not allocate memory\n", __func__);
			return  -ENOMEM;
		}
	} else {
		data = kmem_cache_alloc(rtp_cachep, GFP_KERNEL);
		if (!data) {
			pr_err("%s can not alloc cache memory\n", __func__);
			return -ENOMEM;
		}
	}
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	mutex_lock(&aw8697->bus_lock);

	ret = i2c_master_send(aw8697->i2c, data, len+1);
	if (ret < 0) {
		pr_err("%s: i2c master send error\n", __func__);
	}

	mutex_unlock(&aw8697->bus_lock);
	if ((rtp_cachep == NULL) || (len > RTP_SLAB_SIZE)) {
		if (data != NULL)
			kfree(data);
	} else {
		kmem_cache_free(rtp_cachep, data);
	}
	return ret;
}


/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw8697_rtp_loaded(const struct firmware *cont, void *context)
{
	struct aw8697 *aw8697 = context;
	struct aw8697_wavefrom_rtp_info *waveform_rtp_table = aw8697->waveform_rtp_table;
	if (!cont) {
		//pr_err("%s: failed to read %s\n", __func__, aw8697_rtp_name[aw8697->rtp_file_num]);
		pr_err("%s: failed to read %s\n", __func__, waveform_rtp_table[aw8697->rtp_file_num].rtp_name);
		release_firmware(cont);
		return;
	}

	//pr_info("%s: loaded %s - size: %zu\n", __func__, aw8697_rtp_name[aw8697->rtp_file_num],
	//				cont ? cont->size : 0);
	pr_info("%s enter: loaded %s - size: %zu\n", __func__, waveform_rtp_table[aw8697->rtp_file_num].rtp_name,
					cont ? cont->size : 0);
#if 0
	/* aw8697 rtp update */
	aw8697_rtp = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw8697_rtp) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
#else

	if (aw8697_rtp == NULL) {
		aw8697_rtp = devm_kzalloc(aw8697->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);//TODO
		if (aw8697_rtp == NULL) {
			pr_err("%s devm kzalloc failed\n", __func__);
			release_firmware(cont);
			return;
		}
	}
	memset(aw8697_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));

#endif
	aw8697_rtp->len = cont->size;
	//pr_info("%s: rtp size = %d\n", __func__, aw8697_rtp->len);
	memcpy(aw8697_rtp->data, cont->data, cont->size);
	release_firmware(cont);

	aw8697->rtp_init = 1;
	pr_info("%s: rtp update complete\n", __func__);
}

static int aw8697_rtp_update(struct aw8697 *aw8697)
{
	struct aw8697_wavefrom_rtp_info *waveform_rtp_table = aw8697->waveform_rtp_table;
	char file_name[128] = {0};
	
	if (aw8697->add_suffix) {
		snprintf(file_name, 128, "%s%s", "_", waveform_rtp_table[aw8697->rtp_file_num].rtp_name);
	} else {
		strlcpy(file_name, waveform_rtp_table[aw8697->rtp_file_num].rtp_name, 128);
	}	
	pr_info("%s enter, file_name: %s\n", __func__, file_name);
#if 0
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw8697_rtp_name[aw8697->rtp_file_num], aw8697->dev, GFP_KERNEL,
				aw8697, aw8697_rtp_loaded);
#else
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				file_name, aw8697->dev, GFP_KERNEL,
				aw8697, aw8697_rtp_loaded);

#endif
}


 static void aw8697_container_update(struct aw8697 *aw8697,
		struct aw8697_container *aw8697_cont)
{
	int i = 0;
	unsigned int shift = 0;

	pr_info("%s enter\n", __func__);

	mutex_lock(&aw8697->lock);

	aw8697->ram.baseaddr_shift = 2;
	aw8697->ram.ram_shift = 4;

	/* RAMINIT Enable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);

	/* base addr */
	shift = aw8697->ram.baseaddr_shift;
	aw8697->ram.base_addr = (unsigned int)((aw8697_cont->data[0+shift]<<8) |
			(aw8697_cont->data[1+shift]));
	pr_info("%s: base_addr=0x%4x\n", __func__, aw8697->ram.base_addr);

	aw8697_i2c_write(aw8697, AW8697_REG_BASE_ADDRH, aw8697_cont->data[0+shift]);
	aw8697_i2c_write(aw8697, AW8697_REG_BASE_ADDRL, aw8697_cont->data[1+shift]);

	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AEH,
					(unsigned char)((aw8697->ram.base_addr>>1) >> 8));//1/2 FIFO
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AEL,
					(unsigned char)((aw8697->ram.base_addr>>1)&0x00FF));//1/2 FIFO
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AFH,
					(unsigned char)((aw8697->ram.base_addr-(aw8697->ram.base_addr>>2)) >> 8));
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AFL,
					(unsigned char)((aw8697->ram.base_addr-(aw8697->ram.base_addr>>2))&0x00FF));

	/* ram */
	shift = aw8697->ram.baseaddr_shift;
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, aw8697_cont->data[0+shift]);
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, aw8697_cont->data[1+shift]);
	shift = aw8697->ram.ram_shift;
	for (i = shift; i < aw8697_cont->len; i++) {
		aw8697_i2c_write(aw8697, AW8697_REG_RAMDATA, aw8697_cont->data[i]);
	}

#if 0
	/* ram check */
	shift = aw8697->ram.baseaddr_shift;
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, aw8697_cont->data[0+shift]);
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, aw8697_cont->data[1+shift]);
	shift = aw8697->ram.ram_shift;
	for (i = shift; i < aw8697_cont->len; i++) {
		aw8697_i2c_read(aw8697, AW8697_REG_RAMDATA, &reg_val);
		if (reg_val != aw8697_cont->data[i]) {
			pr_err("%s: ram check error addr=0x%04x, file_data=0x%02x, ram_data=0x%02x\n",
						__func__, i, aw8697_cont->data[i], reg_val);
			return;
		}
	}
#endif

	/* RAMINIT Disable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	mutex_unlock(&aw8697->lock);

	pr_info("%s exit\n", __func__);
}


static void aw8697_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw8697 *aw8697 = context;
	struct aw8697_container *aw8697_fw;
	int i = 0;
	unsigned short check_sum = 0;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw8697_ram_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s enter: loaded %s - size: %zu\n", __func__, aw8697_ram_name,
					cont ? cont->size : 0);
/*
	for (i = 0; i < cont->size; i++) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
	}
*/

	/* check sum */
	for (i = 2; i < cont->size; i++) {
		check_sum += cont->data[i];
	}
	if (check_sum != (unsigned short)((cont->data[0]<<8)|(cont->data[1]))) {
		pr_err("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
		return;
	} else {
		pr_info("%s: check sum pass : 0x%04x\n", __func__, check_sum);
		aw8697->ram.check_sum = check_sum;
	}

	/* aw8697 ram update */
	aw8697_fw = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw8697_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8697_fw->len = cont->size;
	memcpy(aw8697_fw->data, cont->data, cont->size);
	release_firmware(cont);

	aw8697_container_update(aw8697, aw8697_fw);

	aw8697->ram.len = aw8697_fw->len;

	kfree(aw8697_fw);

	aw8697->ram_init = 1;
	pr_info("%s: ram fw update complete\n", __func__);

	aw8697_haptic_trig_enable_config(aw8697);

	aw8697_rtp_update(aw8697);
}


static int aw8697_ram_update(struct aw8697 *aw8697)
{
	char file_name[128] = {0};

	aw8697->ram_init = 0;
	aw8697->rtp_init = 0;

	if (aw8697->add_suffix) {
		snprintf(file_name, 128, "%s%s", "_", aw8697_ram_name);
	} else {
		strlcpy(file_name, aw8697_ram_name, 128);
	}

	pr_debug("%s enter, file_name: %s\n", __func__, file_name);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				file_name, aw8697->dev, GFP_KERNEL,
				aw8697, aw8697_ram_loaded);
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static void aw8697_ram_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 = container_of(work, struct aw8697, ram_work.work);

	pr_info("%s enter\n", __func__);

	aw8697_ram_update(aw8697);

}
#endif

static int aw8697_ram_init(struct aw8697 *aw8697)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = 5000;
	INIT_DELAYED_WORK(&aw8697->ram_work, aw8697_ram_work_routine);
	schedule_delayed_work(&aw8697->ram_work, msecs_to_jiffies(ram_timer_val));
#else
	aw8697_ram_update(aw8697);
#endif
	return 0;
}



/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static void aw8697_vol_trig_switch(struct aw8697 *aw8697, bool sw)
{
	if (!aw8697->no_trigger) {
		if (sw) {
			aw8697->trig[0].enable = 1;
			aw8697->trig[1].enable = 1;
			aw8697->trig[2].enable = 1;
			pr_debug("%s sw=%d\n", __func__, sw);
			aw8697_haptic_trig_enable_config(aw8697);
		} else {
			aw8697->trig[0].enable = 0;
			aw8697->trig[1].enable = 0;
			aw8697->trig[2].enable = 0;
			pr_debug("%s sw=%d\n", __func__, sw);
			aw8697_haptic_trig_enable_config(aw8697);
		}
	}
}

static int aw8697_haptic_softreset(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	aw8697_i2c_write(aw8697, AW8697_REG_ID, 0xAA);
	msleep(1);
	return 0;
}

static int aw8697_haptic_active(struct aw8697 *aw8697)
{
	pr_debug("%s enter\n", __func__);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_WORK_MODE_MASK, AW8697_BIT_SYSCTRL_ACTIVE);
	aw8697_interrupt_clear(aw8697);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_EN);
	return 0;
}

static int aw8697_haptic_play_mode(struct aw8697 *aw8697, unsigned char play_mode)
{
	pr_debug("%s enter, mode=%d\n", __func__, play_mode);

	switch (play_mode) {
	case AW8697_HAPTIC_STANDBY_MODE:
		aw8697->play_mode = AW8697_HAPTIC_STANDBY_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_OFF);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_WORK_MODE_MASK&AW8697_BIT_SYSCTRL_WAVDAT_MODE_MASK, AW8697_BIT_SYSCTRL_STANDBY|AW8697_BIT_SYSCTRL_WAVDAT_MODE_1X);
		break;
	case AW8697_HAPTIC_RAM_MODE:
		aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_PLAY_MODE_MASK&AW8697_BIT_SYSCTRL_WAVDAT_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM|AW8697_BIT_SYSCTRL_WAVDAT_MODE_1X);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_ENABLE);
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK&AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					AW8697_BIT_SYSCTRL_BST_MODE_BOOST|AW8697_BIT_SYSCTRL_STANDBY);
			aw8697_haptic_active(aw8697);
		} else {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		//msleep(3);
		mdelay(2);
		break;
	case AW8697_HAPTIC_RAM_LOOP_MODE:
		aw8697->play_mode = AW8697_HAPTIC_RAM_LOOP_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_PLAY_MODE_MASK&AW8697_BIT_SYSCTRL_WAVDAT_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM|AW8697_BIT_SYSCTRL_WAVDAT_MODE_1X);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		break;
	case AW8697_HAPTIC_RTP_MODE:
		aw8697->play_mode = AW8697_HAPTIC_RTP_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_PLAY_MODE_MASK&AW8697_BIT_SYSCTRL_WAVDAT_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RTP|AW8697_BIT_SYSCTRL_WAVDAT_MODE_1X);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RTP_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RTP_ENABLE);
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK&AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					AW8697_BIT_SYSCTRL_BST_MODE_BOOST|AW8697_BIT_SYSCTRL_STANDBY);
			aw8697_haptic_active(aw8697);
		} else {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		//msleep(3);
		mdelay(2);

		break;
	case AW8697_HAPTIC_TRIG_MODE:
		aw8697->play_mode = AW8697_HAPTIC_TRIG_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_PLAY_MODE_MASK&AW8697_BIT_SYSCTRL_WAVDAT_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM|AW8697_BIT_SYSCTRL_WAVDAT_MODE_1X);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_ENABLE);
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK&AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					AW8697_BIT_SYSCTRL_BST_MODE_BOOST|AW8697_BIT_SYSCTRL_STANDBY);
			aw8697_haptic_active(aw8697);
		} else {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		//msleep(3);
		mdelay(2);
		break;
	case AW8697_HAPTIC_CONT_MODE:
		aw8697->play_mode = AW8697_HAPTIC_CONT_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_PLAY_MODE_MASK&AW8697_BIT_SYSCTRL_WAVDAT_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_CONT|AW8697_BIT_SYSCTRL_WAVDAT_MODE_1X);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK, AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		break;
	default:
		dev_err(aw8697->dev, "%s: play mode %d err",
				__func__, play_mode);
		break;
	}
	return 0;
}

static int aw8697_haptic_play_go(struct aw8697 *aw8697, bool flag)
{
	long long delta_ms = 0;

	pr_debug("%s enter, flag=%d\n", __func__, flag);

	if (flag == true) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_GO,
			AW8697_BIT_GO_MASK, AW8697_BIT_GO_ENABLE);
		do_gettimeofday(&aw8697->begin); // get begin time
	} else {
		do_gettimeofday(&aw8697->cancel); // get cancel time
		delta_ms = (aw8697->cancel.tv_sec - aw8697->begin.tv_sec) * 1000 +
			(aw8697->cancel.tv_usec - aw8697->begin.tv_usec) / 1000;
		if (delta_ms < 5) { // ic need at least 5ms between begin and stop
			mdelay(5);
			pr_info("%s --->cancel delay, delta_ms=%u\n", __func__, delta_ms);
		}
		aw8697_i2c_write_bits(aw8697, AW8697_REG_GO,
			AW8697_BIT_GO_MASK, AW8697_BIT_GO_DISABLE);
	}
	return 0;
}

static int aw8697_haptic_stop_delay(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	unsigned int cnt = 45;

	while (cnt--) {
		aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &reg_val);
		if ((reg_val&0x0f) == 0x00) {
			return 0;
		}
		//msleep(2);
		mdelay(2);
		pr_debug("%s wait for standby, reg glb_state=0x%02x\n",
			__func__, reg_val);
	}
	pr_err("%s do not enter standby automatically\n", __func__);

	return 0;
}

static void aw8697_double_click_switch(struct aw8697 *aw8697, bool sw)
{
	if (sw) {
		aw8697_haptic_set_wav_seq(aw8697, 0x00, 0x01);
		aw8697_haptic_set_wav_seq(aw8697, 0x01, ((AW8697_DUOBLE_CLICK_DELTA / SEQ_WAIT_UNIT) & 0x7f) | 0x80);
		aw8697_haptic_set_wav_seq(aw8697, 0x02, 0x01);
	} else {
		aw8697_haptic_set_wav_seq(aw8697, 0x00, 0x00);
		aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);
		aw8697_haptic_set_wav_seq(aw8697, 0x02, 0x00);
	}
}

static void aw8697_quadra_click_switch(struct aw8697 *aw8697, bool sw)
{
	if (sw) {
		aw8697_haptic_set_wav_seq(aw8697, 0x00, 0x09);
		aw8697_haptic_set_wav_seq(aw8697, 0x01, ((AW8697_QUADRA_CLICK_DELTA / SEQ_WAIT_UNIT) & 0x7f) | 0x80);
		aw8697_haptic_set_wav_seq(aw8697, 0x02, 0x09);
		aw8697_haptic_set_wav_seq(aw8697, 0x03, ((AW8697_QUADRA_CLICK_DELTA / SEQ_WAIT_UNIT) & 0x7f) | 0x80);
		aw8697_haptic_set_wav_seq(aw8697, 0x04, 0x01);
		aw8697_haptic_set_wav_seq(aw8697, 0x05, ((AW8697_QUADRA_CLICK_DELTA / SEQ_WAIT_UNIT) & 0x7f) | 0x80);
		aw8697_haptic_set_wav_seq(aw8697, 0x06, 0x03);
	} else {
		aw8697_haptic_set_wav_seq(aw8697, 0x00, 0x00);
		aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);
		aw8697_haptic_set_wav_seq(aw8697, 0x02, 0x00);
	}
}

static int aw8697_haptic_stop(struct aw8697 *aw8697)
{
	pr_debug("%s enter\n", __func__);

	aw8697_haptic_play_go(aw8697, false);
	aw8697_haptic_stop_delay(aw8697);
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
	//aw8697_vol_trig_switch(aw8697, true);
	//aw8697_set_clock(aw8697, AW8697_HAPTIC_CLOCK_CALI_F0);
	/* 设置trigger的电压状态 */
	if (aw8697->play.ram_id == 0) {
		aw8697_double_click_switch(aw8697, false);
	} else if (aw8697->play.ram_id == 88) {
		aw8697_quadra_click_switch(aw8697, false);
	}
	aw8697_haptic_set_gain(aw8697, 0x80);
	aw8697_haptic_set_bst_vol(aw8697, 0x13);

	return 0;
}

static int aw8697_haptic_start(struct aw8697 *aw8697)
{

	aw8697_haptic_play_go(aw8697, true);
	/* 获取一次开始时间 */
	//do_gettimeofday(&aw8697->begin);
	return 0;
}

static int aw8697_haptic_set_wav_seq(struct aw8697 *aw8697,
		unsigned char wav, unsigned char seq)
{
	aw8697_i2c_write(aw8697, AW8697_REG_WAVSEQ1+wav,
			seq);
	return 0;
}

static int aw8697_haptic_set_wav_loop(struct aw8697 *aw8697,
		unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav%2) {
		tmp = loop<<0;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_WAVLOOP1+(wav/2),
				AW8697_BIT_WAVLOOP_SEQNP1_MASK, tmp);
	} else {
		tmp = loop<<4;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_WAVLOOP1+(wav/2),
				AW8697_BIT_WAVLOOP_SEQN_MASK, tmp);
	}

	return 0;
}
/*
static int aw8697_haptic_set_main_loop(struct aw8697 *aw8697,
		unsigned char loop)
{
	aw8697_i2c_write(aw8697, AW8697_REG_MAIN_LOOP, loop);
	return 0;
}
*/

static int aw8697_haptic_set_repeat_wav_seq(struct aw8697 *aw8697, unsigned char seq)
{
	aw8697_haptic_set_wav_seq(aw8697, 0x00, seq);
	aw8697_haptic_set_wav_loop(aw8697, 0x00, AW8697_BIT_WAVLOOP_INIFINITELY);

	return 0;
}


static int aw8697_haptic_set_bst_vol(struct aw8697 *aw8697, unsigned char bst_vol)
{
	if (bst_vol & 0xe0) {
		bst_vol = 0x1f;
	}
	aw8697_i2c_write_bits(aw8697, AW8697_REG_BSTDBG4,
			AW8697_BIT_BSTDBG4_BSTVOL_MASK, (bst_vol<<1));
	return 0;
}

static int aw8697_haptic_set_bst_peak_cur(struct aw8697 *aw8697, unsigned char peak_cur)
{
	peak_cur &= AW8697_BSTCFG_PEAKCUR_LIMIT;
	aw8697_i2c_write_bits(aw8697, AW8697_REG_BSTCFG,
			AW8697_BIT_BSTCFG_PEAKCUR_MASK, peak_cur);
	return 0;
}

static int aw8697_haptic_set_gain(struct aw8697 *aw8697, unsigned char gain)
{
	aw8697_i2c_write(aw8697, AW8697_REG_DATDBG, gain);
	return 0;
}

static int aw8697_haptic_set_pwm(struct aw8697 *aw8697, unsigned char mode)
{
	switch (mode) {
	case AW8697_PWM_48K:
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
			AW8697_BIT_PWMDBG_PWM_MODE_MASK, AW8697_BIT_PWMDBG_PWM_48K);
		break;
	case AW8697_PWM_24K:
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
			AW8697_BIT_PWMDBG_PWM_MODE_MASK, AW8697_BIT_PWMDBG_PWM_24K);
		break;
	case AW8697_PWM_12K:
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
			AW8697_BIT_PWMDBG_PWM_MODE_MASK, AW8697_BIT_PWMDBG_PWM_12K);
		break;
	default:
		break;
	}
	return 0;
}

static int aw8697_haptic_play_wav_seq(struct aw8697 *aw8697, unsigned char flag)
{
	pr_debug("%s ram enter\n", __func__);

	if (flag) {
		aw8697_vol_trig_switch(aw8697, false);
		aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_MODE);
		aw8697_haptic_start(aw8697);
	}
	return 0;
}

static int aw8697_haptic_play_repeat_seq(struct aw8697 *aw8697, unsigned char flag)
{
	pr_debug("%s ram loop enter\n", __func__);

	if (flag) {
		aw8697_vol_trig_switch(aw8697, false);
		aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_LOOP_MODE);
		aw8697_haptic_start(aw8697);
	}

	return 0;
}

/*****************************************************
 *
 * motor protect
 *
 *****************************************************/
static int aw8697_haptic_swicth_motorprotect_config(struct aw8697 *aw8697, unsigned char addr, unsigned char val)
{
	pr_debug("%s enter\n", __func__);
	if (addr == 1) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
				AW8697_BIT_DETCTRL_PROTECT_MASK, AW8697_BIT_DETCTRL_PROTECT_SHUTDOWN);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
				AW8697_BIT_PWMPRC_PRC_MASK, AW8697_BIT_PWMPRC_PRC_ENABLE);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
				AW8697_BIT_PRLVL_PR_MASK, AW8697_BIT_PRLVL_PR_ENABLE);
	} else if (addr == 0) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
				AW8697_BIT_DETCTRL_PROTECT_MASK,  AW8697_BIT_DETCTRL_PROTECT_NO_ACTION);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
				AW8697_BIT_PWMPRC_PRC_MASK, AW8697_BIT_PWMPRC_PRC_DISABLE);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
				AW8697_BIT_PRLVL_PR_MASK, AW8697_BIT_PRLVL_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
				AW8697_BIT_PWMPRC_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
				AW8697_BIT_PRLVL_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PRTIME,
				AW8697_BIT_PRTIME_PRTIME_MASK, val);
	} else {
		/*nothing to do;*/
	}
	return 0;
}

/*****************************************************
 *
 * os calibration
 *
 *****************************************************/
static int aw8697_haptic_os_calibration(struct aw8697 *aw8697)
{
	unsigned int cont = 2000;
	unsigned char reg_val = 0;
	pr_debug("%s enter\n", __func__);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			AW8697_BIT_DETCTRL_DIAG_GO_MASK, AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
	while (1) {
		aw8697_i2c_read(aw8697, AW8697_REG_DETCTRL, &reg_val);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		 cont--;
	}
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);
	return 0;
}
/*****************************************************
 *
 * trig config
 *
 *****************************************************/
static int aw8697_haptic_trig_param_init(struct aw8697 *aw8697)
{
	pr_debug("%s enter\n", __func__);

	aw8697->trig[0].enable = AW8697_TRG1_ENABLE;
	aw8697->trig[0].default_level = AW8697_TRG1_DEFAULT_LEVEL;
	aw8697->trig[0].dual_edge = AW8697_TRG1_DUAL_EDGE;
	aw8697->trig[0].frist_seq = AW8697_TRG1_FIRST_EDGE_SEQ;
	aw8697->trig[0].second_seq = AW8697_TRG1_SECOND_EDGE_SEQ;

	aw8697->trig[1].enable = AW8697_TRG2_ENABLE;
	aw8697->trig[1].default_level = AW8697_TRG2_DEFAULT_LEVEL;
	aw8697->trig[1].dual_edge = AW8697_TRG2_DUAL_EDGE;
	aw8697->trig[1].frist_seq = AW8697_TRG2_FIRST_EDGE_SEQ;
	aw8697->trig[1].second_seq = AW8697_TRG2_SECOND_EDGE_SEQ;

	aw8697->trig[2].enable = AW8697_TRG3_ENABLE;
	aw8697->trig[2].default_level = AW8697_TRG3_DEFAULT_LEVEL;
	aw8697->trig[2].dual_edge = AW8697_TRG3_DUAL_EDGE;
	aw8697->trig[2].frist_seq = AW8697_TRG3_FIRST_EDGE_SEQ;
	aw8697->trig[2].second_seq = AW8697_TRG3_SECOND_EDGE_SEQ;

	return 0;
}

static int aw8697_haptic_trig_param_config(struct aw8697 *aw8697)
{
	pr_debug("%s enter\n", __func__);

	if (aw8697->trig[0].default_level) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG1_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG1_POLAR_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG1_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG1_POLAR_POS);
	}
	if (aw8697->trig[1].default_level) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG2_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG2_POLAR_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG2_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG2_POLAR_POS);
	}
	if (aw8697->trig[2].default_level) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG3_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG3_POLAR_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG3_POLAR_MASK, AW8697_BIT_TRGCFG1_TRG3_POLAR_POS);
	}

	if (aw8697->trig[0].dual_edge) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG1_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG1_EDGE_POS_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG1_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG1_EDGE_POS);
	}
	if (aw8697->trig[1].dual_edge) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG2_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG2_EDGE_POS_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG2_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG2_EDGE_POS);
	}
	if (aw8697->trig[2].dual_edge) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG3_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG3_EDGE_POS_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				AW8697_BIT_TRGCFG1_TRG3_EDGE_MASK, AW8697_BIT_TRGCFG1_TRG3_EDGE_POS);
	}

	if (aw8697->trig[0].frist_seq) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG1_WAV_P, aw8697->trig[0].frist_seq);
	}
	if (aw8697->trig[0].second_seq && aw8697->trig[0].dual_edge) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG1_WAV_N, aw8697->trig[0].second_seq);
	}
	if (aw8697->trig[1].frist_seq) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG2_WAV_P, aw8697->trig[1].frist_seq);
	}
	if (aw8697->trig[1].second_seq && aw8697->trig[1].dual_edge) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG2_WAV_N, aw8697->trig[1].second_seq);
	}
	if (aw8697->trig[2].frist_seq) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG3_WAV_P, aw8697->trig[1].frist_seq);
	}
	if (aw8697->trig[2].second_seq && aw8697->trig[2].dual_edge) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG3_WAV_N, aw8697->trig[1].second_seq);
	}

	return 0;
}

static int aw8697_haptic_trig_enable_config(struct aw8697 *aw8697)
{

	if (!aw8697->no_trigger) {
		pr_debug("%s enter\n", __func__);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
			AW8697_BIT_TRGCFG2_TRG1_ENABLE_MASK, aw8697->trig[0].enable ? AW8697_BIT_TRGCFG2_TRG1_ENABLE:AW8697_BIT_TRGCFG2_TRG1_DISABLE);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
			AW8697_BIT_TRGCFG2_TRG2_ENABLE_MASK, aw8697->trig[1].enable ? AW8697_BIT_TRGCFG2_TRG2_ENABLE:AW8697_BIT_TRGCFG2_TRG2_DISABLE);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
			AW8697_BIT_TRGCFG2_TRG3_ENABLE_MASK, aw8697->trig[2].enable ? AW8697_BIT_TRGCFG2_TRG3_ENABLE:AW8697_BIT_TRGCFG2_TRG3_DISABLE);
	}
	return 0;
}


static int aw8697_haptic_auto_boost_config(struct aw8697 *aw8697, unsigned char flag)
{
	aw8697->auto_boost = flag;
	if (flag) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
				AW8697_BIT_BST_AUTO_BST_AUTOSW_MASK, AW8697_BIT_BST_AUTO_BST_AUTOMATIC_BOOST);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
				AW8697_BIT_BST_AUTO_BST_AUTOSW_MASK, AW8697_BIT_BST_AUTO_BST_MANUAL_BOOST);
	}
	return 0;
}

/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static int aw8697_haptic_cont_vbat_mode(struct aw8697 *aw8697, unsigned char flag)
{
	if (flag == AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_ADCTEST,
				AW8697_BIT_ADCTEST_VBAT_MODE_MASK, AW8697_BIT_ADCTEST_VBAT_HW_COMP);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_ADCTEST,
				AW8697_BIT_ADCTEST_VBAT_MODE_MASK, AW8697_BIT_ADCTEST_VBAT_SW_COMP);
	}
	return 0;
}

static int aw8697_haptic_get_vbat(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	unsigned int cont = 30;

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			AW8697_BIT_DETCTRL_VBAT_GO_MASK, AW8697_BIT_DETCTRL_VABT_GO_ENABLE);

	while (1) {
		aw8697_i2c_read(aw8697, AW8697_REG_DETCTRL, &reg_val);
		if ((reg_val & 0x02) == 0 || cont == 0)
			 break;
		pr_debug("%s ---->\n", __func__);
		cont--;
	}

	aw8697_i2c_read(aw8697, AW8697_REG_VBATDET, &reg_val);
	aw8697->vbat = 6100 * reg_val / 256;
	if (aw8697->vbat > AW8697_VBAT_MAX) {
		aw8697->vbat = AW8697_VBAT_MAX;
		pr_debug("%s vbat max limit = %dmV\n", __func__, aw8697->vbat);
	}
	if (aw8697->vbat < AW8697_VBAT_MIN) {
		aw8697->vbat = AW8697_VBAT_MIN;
		pr_debug("%s vbat min limit = %dmV\n", __func__, aw8697->vbat);
	}

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	return 0;
}

static int aw8697_haptic_ram_vbat_comp(struct aw8697 *aw8697, bool flag) //用于长振
{
	int temp_gain = 0;

	if (flag) {
		if (aw8697->ram_vbat_comp == AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			aw8697_haptic_get_vbat(aw8697);
			temp_gain = aw8697->gain * AW8697_VBAT_REFER / aw8697->vbat;
			if (temp_gain > (128*AW8697_VBAT_REFER/AW8697_VBAT_MIN)) {
				temp_gain = 128*AW8697_VBAT_REFER/AW8697_VBAT_MIN;
				pr_debug("%s gain limit=%d\n", __func__, temp_gain);
			}
			aw8697_haptic_set_gain(aw8697, temp_gain);
		} else {
			aw8697_haptic_set_gain(aw8697, aw8697->gain);
		}
	} else {
		aw8697_haptic_set_gain(aw8697, aw8697->gain);
	}

	return 0;
}

/*****************************************************
 *
 * f0
 *
 *****************************************************/
static int aw8697_haptic_set_f0_preset(struct aw8697 *aw8697)
{
	unsigned int f0_reg = 0;

	pr_debug("%s enter\n", __func__);

	f0_reg = 1000000000/(aw8697->f0_pre*AW8697_HAPTIC_F0_COEFF);
	aw8697_i2c_write(aw8697, AW8697_REG_F_PRE_H, (unsigned char)((f0_reg>>8)&0xff));
	aw8697_i2c_write(aw8697, AW8697_REG_F_PRE_L, (unsigned char)((f0_reg>>0)&0xff));

	return 0;
}

static int aw8697_haptic_read_f0(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_debug("%s enter\n", __func__);

	ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_F0_H, &reg_val);
	f0_reg = (reg_val<<8);
	ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_F0_L, &reg_val);
	f0_reg |= (reg_val<<0);
	f0_tmp = 1000000000/(f0_reg*AW8697_HAPTIC_F0_COEFF);
	aw8697->f0 = (unsigned int)f0_tmp;
	pr_info("%s f0=%d\n", __func__, aw8697->f0);

	return 0;
}

static int aw8697_haptic_read_cont_f0(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_debug("%s enter\n", __func__);

	ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_CONT_H, &reg_val);
	f0_reg = (reg_val<<8);
	ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_CONT_L, &reg_val);
	f0_reg |= (reg_val<<0);
	f0_tmp = 1000000000/(f0_reg*AW8697_HAPTIC_F0_COEFF);
	aw8697->cont_f0 = (unsigned int)f0_tmp;
	pr_info("%s f0=%d\n", __func__, aw8697->cont_f0);

	return 0;
}

static int aw8697_haptic_read_beme(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char reg_val = 0;

	ret = aw8697_i2c_read(aw8697, AW8697_REG_WAIT_VOL_MP, &reg_val);
	aw8697->max_pos_beme = (reg_val<<0);
	ret = aw8697_i2c_read(aw8697, AW8697_REG_WAIT_VOL_MN, &reg_val);
	aw8697->max_neg_beme = (reg_val<<0);

	pr_info("%s max_pos_beme=%d\n", __func__, aw8697->max_pos_beme);
	pr_info("%s max_neg_beme=%d\n", __func__, aw8697->max_neg_beme);

	return 0;
}



/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw8697_haptic_set_rtp_aei(struct aw8697 *aw8697, bool flag)
{
	if (flag) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				AW8697_BIT_SYSINTM_FF_AE_MASK, AW8697_BIT_SYSINTM_FF_AE_EN);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				AW8697_BIT_SYSINTM_FF_AE_MASK, AW8697_BIT_SYSINTM_FF_AE_OFF);
	}
}
/*
static void aw8697_haptic_set_rtp_afi(struct aw8697 *aw8697, bool flag)
{
	if (flag) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				AW8697_BIT_SYSINTM_FF_AF_MASK, AW8697_BIT_SYSINTM_FF_AF_EN);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				AW8697_BIT_SYSINTM_FF_AF_MASK, AW8697_BIT_SYSINTM_FF_AF_OFF);
	}
}
*/
#if 0
static unsigned char aw8697_haptic_rtp_get_fifo_aei(struct aw8697 *aw8697)
{
	unsigned char ret;
	unsigned char reg_val;


	if (aw8697->osc_cali_flag == 1) {
		aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
		reg_val &= AW8697_BIT_SYSST_FF_AES;
		ret = reg_val>>4;
	} else {
	  aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
	  reg_val &= AW8697_BIT_SYSINT_FF_AEI;
	  ret = reg_val>>4;
	}

	return ret;
}
#endif
static int aw8697_haptic_rtp_get_fifo_afi(struct aw8697 *aw8697)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;
	int rc = 0;


	if (aw8697->osc_cali_flag == 1) {
		rc = aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
		if (rc < 0) {
			pr_info("%s failed, ret=%d, reg_val=%d", __func__, ret, reg_val);
			return -EBUSY;
		}
		reg_val &= AW8697_BIT_SYSST_FF_AFS;
		ret = reg_val>>3;
	} else {
		rc = aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
		if (rc < 0) {
			pr_info("%s failed, ret=%d, reg_val=%d", __func__, ret, reg_val);
			return -EBUSY;
		}

		reg_val &= AW8697_BIT_SYSINT_FF_AFI;
		ret = reg_val>>3;
	}

	return ret;
}

//vivo zhangxiaodong add for rtp int begin

/* 通过SYSST状态寄存器的fifo满bit位来判断fifo是否满，代替通过SYSINT的fifo满bit位
 * 防止读SYSINT寄存器判断fifo是否满的时候，同时将fifo空的bit位清除掉了
 */

static int aw8697_haptic_rtp_get_fifo_afs(struct aw8697 *aw8697)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;
	int rc = 0;

	rc = aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
	if (rc < 0) {
		pr_info("%s failed, ret=%d, reg_val=%#x", __func__, ret, reg_val);
		return -EBUSY;
	}
	reg_val &= AW8697_BIT_SYSST_FF_AFS;
	ret = reg_val >> 3;

	return ret; //0x01表示fifo满，0x00表示fifo空，负数表示i2c读取错误
}


//vivo zhangxiaodong add end

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static int aw8697_haptic_rtp_init(struct aw8697 *aw8697)
{
	unsigned int buf_len = 0;
	int count = 0;
	int ret = 0, retval = 0;

	pr_info("%s enter\n", __func__);
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, 400);
	//aw8697->rtp_cnt = 0;
	mutex_lock(&aw8697->rtp_lock);
	retval = aw8697_haptic_rtp_get_fifo_afs(aw8697);
	//vivo zhangxiaodong add
	if ((!retval) &&
		(aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
		pr_info("%s rtp cnt = %d\n", __func__, aw8697->rtp_cnt);

		if ((aw8697_rtp->len - aw8697->rtp_cnt) < (aw8697->ram.base_addr)) {
			buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
		} else {
			buf_len = (aw8697->ram.base_addr);//2k 数据写入
		}

		ret = aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
			&aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
		if (ret < 0) {
			aw8697->rtp_cnt = 0;
			mutex_unlock(&aw8697->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);
			return -EBUSY;
		}
		aw8697->rtp_cnt += buf_len;

		if (aw8697->rtp_cnt >= aw8697_rtp->len) {
			pr_info("%s: rtp update complete\n", __func__);
			aw8697->rtp_cnt = 0;
			mutex_unlock(&aw8697->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);

			return 0;
		}
	}

	if (retval < 0) {
		pr_err("%s retval return negtive, retval=%d\n", __func__, retval);
		aw8697->rtp_cnt = 0;
		mutex_unlock(&aw8697->rtp_lock);
		pm_qos_remove_request(&pm_qos_req_vb);
		return 0;
	}

	count = 0;
	//vivo zhangxiaodong add end
	while ((!(retval = aw8697_haptic_rtp_get_fifo_afs(aw8697))) &&
			(aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
		pr_info("%s rtp cnt = %d\n", __func__, aw8697->rtp_cnt);

		if ((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr>>2)) {
			buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
		} else {
			buf_len = (aw8697->ram.base_addr>>2);
		}
		ret = aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
				&aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
		if (ret < 0) {
			aw8697->rtp_cnt = 0;
			mutex_unlock(&aw8697->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}

		aw8697->rtp_cnt += buf_len;
		count++;
		if (aw8697->rtp_cnt >= aw8697_rtp->len) {
			pr_info("%s: rtp update complete\n", __func__);
			aw8697->rtp_cnt = 0;
			mutex_unlock(&aw8697->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}
	mutex_unlock(&aw8697->rtp_lock);
	pr_info("%s count=%d\n", __func__, count);
	if ((aw8697->play_mode == AW8697_HAPTIC_RTP_MODE) && (retval >= 0)) {
		pr_err("%s not rtp mode, retval=%d\n", __func__, retval);
		aw8697_haptic_set_rtp_aei(aw8697, true);
	}



	pr_info("%s exit\n", __func__);
	pm_qos_remove_request(&pm_qos_req_vb);
	return 0;
}
// vivo zhangxiaodong add
static int aw8697_set_clock(struct aw8697 *aw8697, int clock_type)
{
	unsigned char code;
	pr_debug("%s set clock, clock_type=%d\n", __func__, clock_type);
	if (clock_type == AW8697_HAPTIC_CLOCK_CALI_F0) {
		code = (unsigned char)atomic_read(&aw8697->f0_freq_cali);
		pr_debug("%s cali f0 below 0, value=%d, code=%#x\n", __func__, atomic_read(&aw8697->f0_freq_cali), code);
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, code);

	} else if (clock_type == AW8697_HAPTIC_CLOCK_CALI_OSC_STANDARD) {
		code = (unsigned char)atomic_read(&aw8697->standard_osc_freq_cali);
		pr_debug("%s cali f0 below 0, value=%d, code=%#x\n", __func__, atomic_read(&aw8697->standard_osc_freq_cali), code);
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, code);
	}
	return 0;
}

//vivo zhangxiaodong add end
static int aw8697_rtp_trim_lra_calibration(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	unsigned int fre_val = 0;
	unsigned int theory_time = 0;
	unsigned int real_code = 0;
	unsigned int lra_rtim_code = 0;
	unsigned int cali_err_thr = 20; 	// 2%
	 unsigned int cali_min_thr = 1; 	// 0.1%


	aw8697_i2c_read(aw8697, AW8697_REG_PWMDBG, &reg_val);
	fre_val = (reg_val & 0x006f) >> 5;

	if (fre_val == 3)
	  theory_time = (aw8697->rtp_len / 12000) * 1000000; /*12K */
	if (fre_val == 2)
	  theory_time = (aw8697->rtp_len / 24000) * 1000000; /*24K */
	if (fre_val == 1 || fre_val == 0)
	  theory_time = (aw8697->rtp_len / 48000) * 1000000; /*48K */
#if 0
	//printk("microsecond:%ld  theory_time = %d\n",aw8697->microsecond,theory_time);

	if (theory_time <= aw8697->microsecond) {
		if ((aw8697->microsecond - theory_time) > (theory_time/50))
			return 0;
		 real_code = 32 + ((aw8697->microsecond - theory_time) * 400) / theory_time;
	}
	if (theory_time > aw8697->microsecond) {
		if ((theory_time - aw8697->microsecond) > (theory_time/50))
			return 0;
		 real_code = 32 - ((theory_time - aw8697->microsecond) * 400) / theory_time ;
	}
	//printk("microsecond:%ld  theory_time = %d real_code =%d\n",aw8697->microsecond,theory_time,real_code);

	lra_rtim_code = real_code > 31 ? (real_code - 32) : (real_code + 32);
	printk("lra_rtim_code = %d\n", lra_rtim_code);

#endif
	if (theory_time <= aw8697->microsecond) {
		 /* diff over 2%, test data error */
		 if ((aw8697->microsecond - theory_time) > (theory_time*cali_err_thr/1000)) {
			pr_err("%s test data error, +delta=%d\n", __func__, aw8697->microsecond - theory_time);
			 return CALI_OVER_RANGE;
		 }
		 /* diff less 0.1%, no need to cali */
		 if ((aw8697->microsecond - theory_time) < (theory_time*cali_min_thr/1000)) {
			pr_err("%s no need to cali, +delta=%d\n", __func__, aw8697->microsecond - theory_time);
			return CALI_NO_NEED;
		 }
		 real_code = ((aw8697->microsecond - theory_time) * 4000) / theory_time;
		 real_code = ((real_code%10 < 5) ? 0 : 1) + real_code/10;
		 real_code = 32 + real_code;
	 }
	 if (theory_time > aw8697->microsecond) {
		 /* diff over 2%, test data error */
		 if ((theory_time - aw8697->microsecond) > (theory_time*cali_err_thr/1000)) {
			pr_err("%s test data error, -delta=%d\n", __func__, theory_time - aw8697->microsecond);
			return CALI_OVER_RANGE;
		 }
		 /* diff less 0.1%, no need to cali */
		 if ((theory_time - aw8697->microsecond) < (theory_time*cali_min_thr/1000)) {
			pr_err("%s no need to cali, -delta=%d\n", __func__, theory_time - aw8697->microsecond);
			return CALI_NO_NEED;
		 }
		 real_code = ((theory_time - aw8697->microsecond) * 4000) / theory_time ;
		 real_code = ((real_code%10 < 5) ? 0 : 1) + real_code/10;
		 real_code = 32 - real_code;
	 }
	 pr_info("%s microsecond=%ld, theory_time=%d, real_code=%d\n",
			 __func__, aw8697->microsecond, theory_time, real_code);

	 lra_rtim_code = real_code > 31 ? (real_code - 32) : (real_code + 32);
	 pr_info("%s lra_rtim_code = %d\n", __func__, lra_rtim_code);
	atomic_set(&aw8697->standard_osc_freq_cali, lra_rtim_code);
	if (lra_rtim_code > 0)
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, (char)lra_rtim_code);
	return 0;
}


static unsigned char aw8697_haptic_osc_read_int(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	aw8697_i2c_read(aw8697, AW8697_REG_DBGSTAT, &reg_val);
	return reg_val;
}


static int aw8697_rtp_osc_calibration(struct aw8697 *aw8697)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int buf_len = 0;
	unsigned char osc_int_state = 0;
	struct aw8697_wavefrom_rtp_info *waveform_rtp_table = aw8697->waveform_rtp_table;

	//unsigned char reg_val = 0;
	//unsigned int  pass_cont = 1;
	//unsigned int  cyc_cont = 150;
	aw8697->rtp_cnt = 0;
	aw8697->timeval_flags = 1;
	aw8697->osc_cali_flag = 1;

	pr_info("%s enter\n", __func__);
#if 0
	/* fw loaded */
	ret = request_firmware(&rtp_file,
			aw8697_rtp_name[/*aw8697->rtp_file_num*/ 0],
			aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
				aw8697_rtp_name[/*aw8697->rtp_file_num*/ 0]);
		return ret;
	}
#else
	/* fw loaded */
	ret = request_firmware(&rtp_file,
			waveform_rtp_table[0].rtp_name,
			aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
				waveform_rtp_table[0].rtp_name);
		return ret;
	}

#endif
	aw8697_haptic_stop(aw8697);

	aw8697->rtp_init = 0;
#if 0
	kfree(aw8697_rtp);
	aw8697_rtp = kzalloc(rtp_file->size+sizeof(int), GFP_KERNEL);
	if (!aw8697_rtp) {
		release_firmware(rtp_file);
		pr_err("%s: error allocating memory\n", __func__);
		return -EPERM;
	}
#else
//TODO
	if (aw8697_rtp == NULL) {
		aw8697_rtp = devm_kzalloc(aw8697->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);//TODO
		if (aw8697_rtp == NULL) {
			pr_err("%s devm kzalloc failed\n", __func__);
			release_firmware(rtp_file);
			return -ENOMEM;
		}
	}
	memset(aw8697_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));	

#endif
	aw8697_rtp->len = rtp_file->size;
	aw8697->rtp_len = rtp_file->size;
#if 0
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
			aw8697_rtp_name[/*aw8697->rtp_file_num*/ 0], aw8697_rtp->len);
#else
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
			waveform_rtp_table[0].rtp_name, aw8697_rtp->len);

#endif
	memcpy(aw8697_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);

	//aw8697->rtp_init = 1; //Don't enter aw8697_irq,because osc calibration use while (1) function

	/* gain */
	aw8697_haptic_ram_vbat_comp(aw8697, false);

	/* rtp mode config */
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RTP_MODE);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL,
			AW8697_BIT_DBGCTRL_INT_MODE_MASK, AW8697_BIT_DBGCTRL_INT_MODE_EDGE);
	disable_irq(gpio_to_irq(aw8697->irq_gpio));

	/* haptic start */
	aw8697_haptic_start(aw8697);

#if 0
	//while (aw8697->rtp_file_num > 0 && (aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
	while (1) {
		if (!aw8697_haptic_rtp_get_fifo_afi(aw8697)) {
			if ((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr>>2)) {
				buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
			} else {
				buf_len = (aw8697->ram.base_addr>>2);
			}
			if (aw8697->timeval_flags == 1) {
				do_gettimeofday(&aw8697->start);
				aw8697->timeval_flags = 0;
			}
			aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
				&aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
			aw8697->rtp_cnt += buf_len;

			if (aw8697->rtp_cnt >= aw8697_rtp->len) {
				while (pass_cont == 1) {
					aw8697_i2c_read(aw8697, AW8697_REG_DBGSTAT, &reg_val);
					if (reg_val & AW8697_BIT_SYSST_DONES) {
						do_gettimeofday(&aw8697->end);
						pass_cont = 0;
						//printk(" %s microsecond:%ld  \n", __func__,
						//    (aw8697->end.tv_sec - aw8697->start.tv_sec) *1000000 +
						//    (aw8697->end.tv_usec - aw8697->start.tv_usec));
						printk("%s chip playback done\n", __func__);

					}
				}
				printk("%s: rtp update complete\n", __func__);
				aw8697->rtp_cnt = 0;
				break;
			}
		} else {
				aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
				if (reg_val & AW8697_BIT_SYSST_DONES) {
					  do_gettimeofday(&aw8697->end);
					  //pr_info("microsecond:%ld  \n",
					  //    (aw8697->end.tv_sec - aw8697->start.tv_sec) *1000000 +
					  //    (aw8697->end.tv_usec - aw8697->start.tv_usec));
					  pr_info("%s chip playback done\n", __func__);
					 break;
				}
				while (1) {
					if (aw8697_haptic_rtp_get_fifo_aei(aw8697)) {
						printk("-----%s---%d----while (1)--\n", __func__, __LINE__);
						break;
					}
					cyc_cont--;
					if (cyc_cont == 0) {
						cyc_cont = 150;
					break;
					}
				}
			} /*else*/
		} /*while*/
#endif
	/* 优化最新版的校准方式 */
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, 400);

	while (1) {
		   if (!aw8697_haptic_rtp_get_fifo_afi(aw8697)) {
			   pr_info("%s !aw8697_haptic_rtp_get_fifo_afi done aw8697->rtp_cnt= %d \n", __func__, aw8697->rtp_cnt);
			  // mutex_lock(&aw8697->rtp_lock);
			   if ((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr>>2))
				   buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
			   else
				   buf_len = (aw8697->ram.base_addr>>2);

			   if (aw8697->rtp_cnt >= aw8697_rtp->len) {
				   if (aw8697->timeval_flags == 1) {
					   do_gettimeofday(&aw8697->start);
					   aw8697->timeval_flags = 0;
				   }
				  aw8697_i2c_writes(aw8697,
							   AW8697_REG_RTP_DATA,
							   &aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
				   aw8697->rtp_cnt += buf_len;
			   }
				//mutex_unlock(&aw8697->rtp_lock);
		   }

		   osc_int_state = aw8697_haptic_osc_read_int(aw8697);
		   if (osc_int_state&AW8697_BIT_SYSINT_DONEI) {
			   do_gettimeofday(&aw8697->end);
			   pr_info("%s vincent playback done aw8697->rtp_cnt= %d \n", __func__, aw8697->rtp_cnt);
			   break;
		   }
		   do_gettimeofday(&aw8697->end);
		   aw8697->microsecond = (aw8697->end.tv_sec - aw8697->start.tv_sec)*1000000 +
		   (aw8697->end.tv_usec - aw8697->start.tv_usec);
		   if (aw8697->microsecond > OP_OCS_CALIBRATION_T_LENGTH) {
			   pr_info("%s vincent time out aw8697->rtp_cnt %d osc_int_state %02x\n", __func__, aw8697->rtp_cnt, osc_int_state);
			   break;
			 }
	   }
	   pm_qos_remove_request(&pm_qos_req_vb);
	   enable_irq(gpio_to_irq(aw8697->irq_gpio));

	aw8697->osc_cali_flag = 0;
	aw8697->microsecond = (aw8697->end.tv_sec - aw8697->start.tv_sec) * 1000000 +
		(aw8697->end.tv_usec - aw8697->start.tv_usec);
	/*calibration osc*/
	pr_info("aw8697 2018_microsecond:%ld \n", aw8697->microsecond);

	pr_info("%s exit\n", __func__);
	return 0;
}

static void aw8697_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	struct aw8697 *aw8697 = container_of(work, struct aw8697, rtp_work);
	struct aw8697_wavefrom_rtp_info *waveform_rtp_table = aw8697->waveform_rtp_table;
	
	char file_name[128] = {0};
	
	if (aw8697->add_suffix) {
		snprintf(file_name, 128, "%s%s", "_", waveform_rtp_table[aw8697->rtp_file_num].rtp_name);
	} else {
		strlcpy(file_name, waveform_rtp_table[aw8697->rtp_file_num].rtp_name, 128);
	}	
	pr_info("%s enter, file_name: %s\n", __func__, file_name);
	
#if 0
	/* fw loaded */
	ret = request_firmware(&rtp_file,
			aw8697_rtp_name[aw8697->rtp_file_num],
			aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
				aw8697_rtp_name[aw8697->rtp_file_num]);
		return ;
	}
#else
	/* fw loaded */
	ret = request_firmware(&rtp_file,
			file_name,
			aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
				file_name);
		return ;
	}

#endif
	mutex_lock(&aw8697->rtp_lock);
	aw8697->rtp_init = 0;
	aw8697_haptic_set_rtp_aei(aw8697, false);
#if 0
	kfree(aw8697_rtp);
	aw8697_rtp = kzalloc(rtp_file->size+sizeof(int), GFP_KERNEL);
	if (!aw8697_rtp) {
		release_firmware(rtp_file);
		pr_err("%s: error allocating memory\n", __func__);
		mutex_unlock(&aw8697->rtp_lock);
		return;
	}
#else
	if (aw8697_rtp == NULL) {
		aw8697_rtp = devm_kzalloc(aw8697->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);//TODO
		if (aw8697_rtp == NULL) {
			pr_err("%s devm kzalloc failed\n", __func__);
			release_firmware(rtp_file);
			mutex_unlock(&aw8697->rtp_lock);
			return;
		}
	}
	memset(aw8697_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));	
#endif
	aw8697_rtp->len = rtp_file->size;
#if 0
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
			aw8697_rtp_name[aw8697->rtp_file_num], aw8697_rtp->len);
#else
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
			waveform_rtp_table[aw8697->rtp_file_num].rtp_name, aw8697_rtp->len);

#endif
	memcpy(aw8697_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);

	aw8697->rtp_init = 1;
	aw8697->rtp_cnt = 0;
	mutex_unlock(&aw8697->rtp_lock);
	/* gain */
	//aw8697_haptic_ram_vbat_comp(aw8697, false);

	/* rtp mode config */
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RTP_MODE);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL,
			AW8697_BIT_DBGCTRL_INT_MODE_MASK, AW8697_BIT_DBGCTRL_INT_MODE_EDGE);

	/* haptic start */
	mutex_lock(&aw8697->rtp_check_lock);
	if (rtp_check_flag) {
		aw8697_set_clock(aw8697, AW8697_HAPTIC_CLOCK_CALI_OSC_STANDARD);
		aw8697_vol_trig_switch(aw8697, false);
		aw8697_haptic_start(aw8697);
		pr_info("%s ------------------>\n", __func__);
	} else {
		pr_info("%s rtp work has cancel\n", __func__);
		mutex_unlock(&aw8697->rtp_check_lock);
		return;
	}
	mutex_unlock(&aw8697->rtp_check_lock);
	aw8697_haptic_rtp_init(aw8697);
}

static void aw8697_init_setting_work_routine(struct work_struct *work)
{

	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	struct aw8697 *aw8697 = container_of(work, struct aw8697, init_setting_work);
	unsigned char lra_val = 0;
	int retry = 3;


	pr_info("%s enter\n", __func__);
	aw8697_rtp = devm_kzalloc(aw8697->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);
	if (aw8697_rtp == NULL) {
		pr_err("%s devm kzalloc failed\n", __func__);
	}
/* haptic init */
	mutex_lock(&aw8697->lock);

	aw8697_interrupt_setup(aw8697);

	aw8697->activate_mode = AW8697_HAPTIC_ACTIVATE_CONT_MODE;

	ret = aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1, &reg_val);
	aw8697->index = reg_val & 0x7F;
	ret = aw8697_i2c_read(aw8697, AW8697_REG_DATDBG, &reg_val);
	aw8697->gain = reg_val & 0xFF;
	ret = aw8697_i2c_read(aw8697, AW8697_REG_BSTDBG4, &reg_val);
	aw8697->vmax = (reg_val>>1)&0x1F;
	for (i = 0; i < AW8697_SEQUENCER_SIZE; i++) {
		ret = aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1+i, &reg_val);
		aw8697->seq[i] = reg_val;
	}

	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);

	aw8697_haptic_set_pwm(aw8697, AW8697_PWM_24K);

	aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG1, 0x30);
	aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG2, 0xeb);
	aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG3, 0xd4);
	aw8697_i2c_write(aw8697, AW8697_REG_TSET, 0x12);
	aw8697_i2c_write(aw8697, AW8697_REG_R_SPARE, 0x68);
	aw8697_i2c_write(aw8697, 0x0E, 0x01);//test

	aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL, AW8697_BIT_DBGCTRL_WAITSLOT_MASK,
			AW8697_BIT_DBGCTRL_WAITSLOT_1280US);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANADBG,
			AW8697_BIT_ANADBG_IOC_MASK, AW8697_BIT_ANADBG_IOC_4P65A);

	aw8697_haptic_set_bst_peak_cur(aw8697, AW8697_DEFAULT_PEAKCUR);

	aw8697_haptic_swicth_motorprotect_config(aw8697, 0x00, 0x00);

	aw8697_haptic_auto_boost_config(aw8697, true);

	aw8697_haptic_trig_param_init(aw8697);
	aw8697_haptic_trig_param_config(aw8697);

   aw8697_haptic_os_calibration(aw8697);

	aw8697_haptic_cont_vbat_mode(aw8697,
			AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE);
	aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;

	/* f0 calibration */
	aw8697->f0_pre = aw8697->lra_info.AW8697_HAPTIC_F0_PRE;
	aw8697->cont_drv_lvl = aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL;
	aw8697->cont_drv_lvl_ov = aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL_OV;
	aw8697->cont_td = aw8697->lra_info.AW8697_HAPTIC_CONT_TD;
	aw8697->cont_zc_thr = aw8697->lra_info.AW8697_HAPTIC_CONT_ZC_THR;
	aw8697->cont_num_brk = aw8697->lra_info.AW8697_HAPTIC_CONT_NUM_BRK;
	/* 取消每次开机校准，改为产线校准，然后存储校准值 */
	//aw8697_haptic_f0_calibration(aw8697);

	// 读取阻抗值，打印log，方便排除硬件问题
	while (retry--) {
		aw8697_lra_get(aw8697, &lra_val);
		aw8697->lra = 298 * lra_val;
		if ((aw8697->lra >= aw8697->resistance_min && aw8697->lra <= aw8697->resistance_max)) {
			pr_err("%s lra resistent ok, lra=%d\n", __func__, aw8697->lra);
			break;
		} else {
			pr_err("%s lra resistent over range, lra=%d\n", __func__, aw8697->lra);
		}
		msleep(5);
	}

	mutex_unlock(&aw8697->lock);
	/* 由于每次写512字节，并且频繁申请和释放，所以为rtp模式开辟单独的高速缓存，提高内存分配速度 */
	rtp_cachep = kmem_cache_create("aw8697-hap", RTP_SLAB_SIZE + 1, 0, SLAB_HWCACHE_ALIGN, NULL);
	if (rtp_cachep == NULL) {
		pr_err("%s alloc high cache failed\n", __func__);
	} else {
		pr_err("%s alloc high cache success\n", __func__);
	}

}

/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static enum hrtimer_restart aw8697_haptic_audio_timer_func(struct hrtimer *timer)
{
	struct aw8697 *aw8697 = container_of(timer, struct aw8697, haptic_audio.timer);

	pr_debug("%s enter\n", __func__);
	schedule_work(&aw8697->haptic_audio.work);

	hrtimer_start(&aw8697->haptic_audio.timer,
			ktime_set(aw8697->haptic_audio.timer_val/1000000,
					(aw8697->haptic_audio.timer_val%1000000) * 1000),
			HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void aw8697_haptic_audio_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 = container_of(work, struct aw8697, haptic_audio.work);
	struct haptic_audio *haptic_audio = &(aw8697->haptic_audio);
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;
	unsigned int ctr_list_input_cnt = 0;
	unsigned int ctr_list_output_cnt = 0;
	unsigned int ctr_list_diff_cnt = 0;
	unsigned int ctr_list_del_cnt = 0;

	struct haptic_ctr temp_haptic_audio;

	memset(&temp_haptic_audio, 0, sizeof(struct haptic_ctr));
	//int rtp_is_going_on = 0;

	pr_debug("%s enter\n", __func__);

	spin_lock(&aw8697->haptic_audio.list_lock);

	if (!list_empty(&(haptic_audio->ctr_list))) {

		list_for_each_entry_safe(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
			ctr_list_input_cnt = p_ctr->cnt;
			break;
		}

		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
						 &(haptic_audio->ctr_list),
						 list) {
			ctr_list_output_cnt = p_ctr->cnt;
			break;
		}

		if (ctr_list_input_cnt > ctr_list_output_cnt) {
			ctr_list_diff_cnt = ctr_list_input_cnt - ctr_list_output_cnt;
		}

		if (ctr_list_input_cnt < ctr_list_output_cnt) {
			ctr_list_diff_cnt = 32 + ctr_list_input_cnt - ctr_list_output_cnt;
		}

		if (ctr_list_diff_cnt > 2) {

			list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
							 &(haptic_audio->ctr_list), list) {
				if ((p_ctr->play == 0) && 
					(AW8697_HAPTIC_CMD_ENABLE == (AW8697_HAPTIC_CMD_HAPTIC & p_ctr->cmd))) {

					list_del(&p_ctr->list);
					kfree(p_ctr);
					ctr_list_del_cnt++;
				}
				if (ctr_list_del_cnt >= ctr_list_diff_cnt)
					break;
				pr_debug("%s: ctr_list_del_cnt=%d\n", ctr_list_del_cnt);
			}
		}

	} else {
		pr_info("%s: list is empty\n");
		spin_unlock(&aw8697->haptic_audio.list_lock);
		return;
	}



	/* get the last data from list */
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					&(haptic_audio->ctr_list), list) {

		temp_haptic_audio.cnt = p_ctr->cnt;
		temp_haptic_audio.cmd = p_ctr->cmd;
		temp_haptic_audio.play = p_ctr->play;
		temp_haptic_audio.wavseq = p_ctr->wavseq;
		temp_haptic_audio.loop = p_ctr->loop;
		temp_haptic_audio.gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		break;
	}

	spin_unlock(&aw8697->haptic_audio.list_lock);

	mutex_lock(&aw8697->haptic_audio.lock);

	memset(&aw8697->haptic_audio.ctr, 0, sizeof(struct haptic_ctr));

	haptic_audio->ctr = temp_haptic_audio;

	if (aw8697->haptic_audio.ctr.play) {
		pr_info("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
		 __func__, aw8697->haptic_audio.ctr.cnt,
		 aw8697->haptic_audio.ctr.cmd,
		 aw8697->haptic_audio.ctr.play,
		 aw8697->haptic_audio.ctr.wavseq,
		 aw8697->haptic_audio.ctr.loop,
		 aw8697->haptic_audio.ctr.gain);
	}

	if (!haptic_audio->haptic_audio_cancel_flag) {
		if (aw8697->haptic_audio.ctr.cmd == AW8697_HAPTIC_CMD_ENABLE) {

			if (aw8697->haptic_audio.ctr.play == AW8697_HAPTIC_PLAY_ENABLE) {
				pr_info("%s: haptic_audio_play_start\n", __func__);
				mutex_lock(&aw8697->lock);
				aw8697_haptic_stop(aw8697);
				aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_MODE);

				aw8697_haptic_set_wav_seq(aw8697, 0x00,
							aw8697->haptic_audio.ctr.wavseq);

				aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);

				aw8697_haptic_set_wav_loop(aw8697, 0x00,
							aw8697->haptic_audio.ctr.loop);

				aw8697_haptic_set_gain(aw8697,
							aw8697->haptic_audio.ctr.gain);

				aw8697_haptic_start(aw8697);
				mutex_unlock(&aw8697->lock);

			} else if (AW8697_HAPTIC_PLAY_STOP == aw8697->haptic_audio.ctr.play) {

				mutex_lock(&aw8697->lock);
				aw8697_haptic_stop(aw8697);
				mutex_unlock(&aw8697->lock);

			} else if (AW8697_HAPTIC_PLAY_GAIN == aw8697->haptic_audio.ctr.play) {

				mutex_lock(&aw8697->lock);
				aw8697_haptic_set_gain(aw8697,
								aw8697->haptic_audio.ctr.gain);
				mutex_unlock(&aw8697->lock);
			}
		}
	}
	mutex_unlock(&aw8697->haptic_audio.lock);
}


/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static int aw8697_haptic_cont(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	/* work mode */
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_CONT_MODE);

	/* preset f0 */
	aw8697->f0_pre = aw8697->f0;
	aw8697_haptic_set_f0_preset(aw8697);

	/* lpf */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
			AW8697_BIT_DATCTRL_FC_MASK, AW8697_BIT_DATCTRL_FC_1000HZ);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
			AW8697_BIT_DATCTRL_LPF_ENABLE_MASK, AW8697_BIT_DATCTRL_LPF_ENABLE);

	/* cont config */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_ZC_DETEC_MASK, AW8697_BIT_CONT_CTRL_ZC_DETEC_ENABLE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_WAIT_PERIOD_MASK, AW8697_BIT_CONT_CTRL_WAIT_1PERIOD);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_MODE_MASK, AW8697_BIT_CONT_CTRL_BY_GO_SIGNAL);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_CONT_PLAYBACK_MODE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_F0_DETECT_MASK, AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_O2C_MASK, AW8697_BIT_CONT_CTRL_O2C_DISABLE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_AUTO_BRK_MASK, AW8697_BIT_CONT_CTRL_AUTO_BRK_ENABLE);

	/* TD time */
	aw8697_i2c_write(aw8697, AW8697_REG_TD_H, (unsigned char)(aw8697->cont_td>>8));
	aw8697_i2c_write(aw8697, AW8697_REG_TD_L, (unsigned char)(aw8697->cont_td>>0));
	aw8697_i2c_write(aw8697, AW8697_REG_TSET, 0x12);

	/* zero cross */
	aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_H, (unsigned char)(aw8697->cont_zc_thr>>8));
	aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_L, (unsigned char)(aw8697->cont_zc_thr>>0));

	/* bemf */
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_H, 0x10);
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_L, 0x08);
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_H, 0x03);
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_L, 0xf8);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_BEMF_NUM,
			AW8697_BIT_BEMF_NUM_BRK_MASK, aw8697->cont_num_brk);
	aw8697_i2c_write(aw8697, AW8697_REG_TIME_NZC, 0x23);    // 35*171us=5.985ms

	/* f0 driver level */
	aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, aw8697->cont_drv_lvl);
	aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL_OV, aw8697->cont_drv_lvl_ov);

	/* cont play go */
	aw8697_haptic_play_go(aw8697, true);

	return 0;
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
static int aw8697_haptic_get_f0(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char f0_pre_num = 0;
	unsigned char f0_wait_num = 0;
	unsigned char f0_repeat_num = 0;
	unsigned char f0_trace_num = 0;
	unsigned int t_f0_ms = 0;
	unsigned int t_f0_trace_ms = 0;
	unsigned int f0_cali_cnt = 50;

	pr_info("%s enter\n", __func__);

	aw8697->f0 = aw8697->f0_pre;

	/* f0 calibrate work mode */
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_CONT_MODE);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_BIT_CONT_CTRL_OPEN_PLAYBACK);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_F0_DETECT_MASK, AW8697_BIT_CONT_CTRL_F0_DETECT_ENABLE);

	/* LPF */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
			AW8697_BIT_DATCTRL_FC_MASK, AW8697_BIT_DATCTRL_FC_1000HZ);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
			AW8697_BIT_DATCTRL_LPF_ENABLE_MASK, AW8697_BIT_DATCTRL_LPF_ENABLE);

	/* LRA OSC Source */
	if (aw8697->f0_cali_flag == AW8697_HAPTIC_CALI_F0) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
				AW8697_BIT_ANACTRL_LRA_SRC_MASK, AW8697_BIT_ANACTRL_LRA_SRC_REG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
				AW8697_BIT_ANACTRL_LRA_SRC_MASK, AW8697_BIT_ANACTRL_LRA_SRC_EFUSE);
	}

	/* preset f0 */
	aw8697_haptic_set_f0_preset(aw8697);

	/* beme config */
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_H, 0x10);
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_L, 0x08);
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_H, 0x03);
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_L, 0xf8);

	/* f0 driver level */
	aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, aw8697->cont_drv_lvl);

	/* f0 trace parameter */
	f0_pre_num = 0x05;
	f0_wait_num = 0x03;
	f0_repeat_num = 0x02;
	f0_trace_num = 0x0f;
	aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_1, (f0_pre_num<<4)|(f0_wait_num<<0));
	aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_2, (f0_repeat_num<<0));
	aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_3, (f0_trace_num<<0));

	/* clear aw8697 interrupt */
	ret = aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);

	/* play go and start f0 calibration */
	aw8697_haptic_play_go(aw8697, true);

	/* f0 trace time */
	t_f0_ms = 1000*10/aw8697->f0_pre;
	t_f0_trace_ms = t_f0_ms * (f0_pre_num + f0_wait_num + (f0_trace_num + f0_wait_num) * (f0_repeat_num - 1));
	msleep(t_f0_trace_ms);

	for (i = 0; i < f0_cali_cnt; i++) {
		ret = aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
		/* f0 calibrate done */
		if (reg_val & 0x01) {
			aw8697_haptic_read_f0(aw8697);
			aw8697_haptic_read_beme(aw8697);
			break;
		}
		msleep(10);
		pr_info("%s f0 cali sleep 10ms\n", __func__);
	}

	if (i == f0_cali_cnt) {
		ret = -1;
	} else {
		ret = 0;
	}

	/* restore default config */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK, AW8697_CONT_PLAYBACK_MODE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			AW8697_BIT_CONT_CTRL_F0_DETECT_MASK, AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);

	return ret;
}

static int aw8697_haptic_f0_calibration(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
//	int f0_dft_step = 0;

	pr_info("%s enter\n", __func__);

	aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
	aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, 0x00);

	if (aw8697_haptic_get_f0(aw8697)) {
		pr_err("%s get f0 error, user defafult f0\n", __func__);
	} else {
		/* max and min limit */
		f0_limit = aw8697->f0;
		if (aw8697->f0*100 < aw8697->lra_info.AW8697_HAPTIC_F0_PRE*(100-aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN)) {
			f0_limit = aw8697->lra_info.AW8697_HAPTIC_F0_PRE*(100-aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN) / 100;
		}
		if (aw8697->f0*100 > aw8697->lra_info.AW8697_HAPTIC_F0_PRE*(100+aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN)) {
			f0_limit = aw8697->lra_info.AW8697_HAPTIC_F0_PRE*(100+aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN) / 100;
		}

		/* calculate cali step */
		f0_cali_step = 100000*((int)f0_limit-(int)aw8697->f0_pre) / ((int)f0_limit*25);
		pr_debug("%s f0_cali_step=%d\n", __func__, f0_cali_step);
#if 0

		/* get default cali step */
		aw8697_i2c_read(aw8697, AW8697_REG_TRIM_LRA, &reg_val);
		if (reg_val & 0x20) {
			f0_dft_step = reg_val - 0x40;
		} else {
			f0_dft_step = reg_val;
		}
		pr_debug("%s f0_dft_step=%d\n", __func__, f0_dft_step);

		/* get new cali step */
		f0_cali_step += f0_dft_step;
		pr_debug("%s f0_cali_step=%d\n", __func__, f0_cali_step);

		if (f0_cali_step > 31) {
			f0_cali_step = 31;
		} else if (f0_cali_step < -32) {
			f0_cali_step = -32;
		}
		f0_cali_lra = (char)f0_cali_step;
		pr_debug("%s f0_cali_lra=%d\n", __func__, f0_cali_lra);

		/* get cali step complement code*/
		if (f0_cali_lra < 0) {
			f0_cali_lra += 0x40;
		}
		pr_debug("%s reg f0_cali_lra=%d\n", __func__, f0_cali_lra);

		/* update cali step */
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, (char)f0_cali_lra);
		aw8697_i2c_read(aw8697, AW8697_REG_TRIM_LRA, &reg_val);
		pr_info("%s final trim_lra=0x%02x\n", __func__, reg_val);
#endif
		if (f0_cali_step >= 0) {   /*f0_cali_step >= 0*/
			if (f0_cali_step % 10 >= 5) {
				f0_cali_step = f0_cali_step/10 + 1 + 32;
			} else {
				f0_cali_step = f0_cali_step/10  + 32;
			}
		} else {  /*f0_cali_step < 0*/
			if (f0_cali_step % 10 <= -5) {
				f0_cali_step = 32 + (f0_cali_step/10 - 1);
			} else {
				f0_cali_step = 32 + f0_cali_step/10;
			}
		}

		if (f0_cali_step > 31) {
			f0_cali_lra = (char)f0_cali_step - 32;
		} else {
			f0_cali_lra = (char)f0_cali_step + 32;
		}
		pr_info("%s f0_cali_lra=%d\n", __func__, (int)f0_cali_lra);
		atomic_set(&aw8697->f0_freq_cali, f0_cali_lra);
		/* update cali step */
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, (char)f0_cali_lra);
		aw8697_i2c_read(aw8697, AW8697_REG_TRIM_LRA, &reg_val);
		//aw8697->clock_system_f0_cali_lra = f0_cali_lra;//vincent for f0 cali and OSC cali
		pr_info("%s final trim_lra=0x%02x\n", __func__, reg_val);
	}
#if 0
	if (aw8697_haptic_get_f0(aw8697)) {
		pr_err("%s get f0 error, user defafult f0\n", __func__);
	}
#endif
	/* restore default work mode */
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
	aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
	aw8697_haptic_stop(aw8697);

	return ret;
}

/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw8697_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)g_aw8697;

	return 0;
}

static int aw8697_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static long aw8697_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct aw8697 *aw8697 = (struct aw8697 *)file->private_data;

	int ret = 0;

	dev_info(aw8697->dev, "%s: cmd=0x%x, arg=0x%lx\n",
			  __func__, cmd, arg);

	mutex_lock(&aw8697->lock);

	if (_IOC_TYPE(cmd) != AW8697_HAPTIC_IOCTL_MAGIC) {
		dev_err(aw8697->dev, "%s: cmd magic err\n",
				__func__);
		mutex_unlock(&aw8697->lock);
		return -EINVAL;
	}

	switch (cmd) {
	default:
		dev_err(aw8697->dev, "%s, unknown cmd\n", __func__);
		break;
	}

	mutex_unlock(&aw8697->lock);

	return ret;
}

static ssize_t aw8697_file_read(struct file *filp, char *buff, size_t len, loff_t *offset)
{
	struct aw8697 *aw8697 = (struct aw8697 *)filp->private_data;
	int ret = 0;
	int i = 0;
	unsigned char reg_val = 0;
	unsigned char *pbuff = NULL;

	mutex_lock(&aw8697->lock);

	dev_info(aw8697->dev, "%s: len=%zu\n", __func__, len);

	switch (aw8697->fileops.cmd) {
	case AW8697_HAPTIC_CMD_READ_REG:
		pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
		if (pbuff != NULL) {
			for (i = 0; i < len; i++) {
				aw8697_i2c_read(aw8697, aw8697->fileops.reg+i, &reg_val);
				pbuff[i] = reg_val;
			}
			for (i = 0; i < len; i++) {
				dev_info(aw8697->dev, "%s: pbuff[%d]=0x%02x\n",
						__func__, i, pbuff[i]);
			}
			ret = copy_to_user(buff, pbuff, len);
			if (ret) {
				dev_err(aw8697->dev, "%s: copy to user fail\n", __func__);
			}
			kfree(pbuff);
		} else {
			dev_err(aw8697->dev, "%s: alloc memory fail\n", __func__);
		}
		break;
	default:
		dev_err(aw8697->dev, "%s, unknown cmd %d \n", __func__, aw8697->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8697->lock);


	return len;
}

static ssize_t aw8697_file_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	struct aw8697 *aw8697 = (struct aw8697 *)filp->private_data;
	int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;

	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		dev_err(aw8697->dev, "%s: alloc memory fail\n", __func__);
		return len;
	}
	ret = copy_from_user(pbuff, buff, len);
	if (ret) {
		dev_err(aw8697->dev, "%s: copy from user fail\n", __func__);
		kfree(pbuff);
		return len;
	}

	for (i = 0; i < len; i++) {
		dev_info(aw8697->dev, "%s: pbuff[%d]=0x%02x\n",
				__func__, i, pbuff[i]);
	}

	mutex_lock(&aw8697->lock);

	aw8697->fileops.cmd = pbuff[0];

	switch (aw8697->fileops.cmd) {
	case AW8697_HAPTIC_CMD_READ_REG:
		if (len == 2) {
			aw8697->fileops.reg = pbuff[1];
		} else {
			dev_err(aw8697->dev, "%s: read cmd len %zu err\n", __func__, len);
		}
		break;
	case AW8697_HAPTIC_CMD_WRITE_REG:
		if (len > 2) {
			for (i = 0; i < len-2; i++) {
				dev_info(aw8697->dev, "%s: write reg0x%02x=0x%02x\n",
						__func__, pbuff[1]+i, pbuff[i+2]);
				aw8697_i2c_write(aw8697, pbuff[1]+i, pbuff[2+i]);
			}
		} else {
			dev_err(aw8697->dev, "%s: write cmd len %zu err\n", __func__, len);
		}
		break;
	default:
		dev_err(aw8697->dev, "%s, unknown cmd %d \n", __func__, aw8697->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8697->lock);

	if (pbuff != NULL) {
		kfree(pbuff);
	}
	return len;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw8697_file_read,
	.write = aw8697_file_write,
	.unlocked_ioctl = aw8697_file_unlocked_ioctl,
	.open = aw8697_file_open,
	.release = aw8697_file_release,
};

static struct miscdevice aw8697_haptic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW8697_HAPTIC_NAME,
	.fops = &fops,
};

static int aw8697_haptic_init(struct aw8697 *aw8697)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);

	schedule_work(&aw8697->init_setting_work);

	return ret;
}



/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
#ifdef TIMED_OUTPUT
static int aw8697_vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw8697 *aw8697 = container_of(dev, struct aw8697, to_dev);

	if (hrtimer_active(&aw8697->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw8697->timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void aw8697_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw8697 *aw8697 = container_of(dev, struct aw8697, to_dev);

	mutex_lock(&aw8697->lock);

	pr_debug("%s enter\n", __func__);

	aw8697_haptic_stop(aw8697);

	if (value > 0) {
		aw8697_haptic_ram_vbat_comp(aw8697, false);
		aw8697_haptic_play_wav_seq(aw8697, value);
	}

	mutex_unlock(&aw8697->lock);

	pr_debug("%s exit\n", __func__);
}

#else
static enum led_brightness aw8697_haptic_brightness_get(struct led_classdev *cdev)
{
	struct aw8697 *aw8697 =
		container_of(cdev, struct aw8697, cdev);

	return aw8697->amplitude;
}

static void aw8697_haptic_brightness_set(struct led_classdev *cdev,
				enum led_brightness level)
{
	struct aw8697 *aw8697 =
		container_of(cdev, struct aw8697, cdev);

	aw8697->amplitude = level;

	mutex_lock(&aw8697->lock);

	aw8697_haptic_stop(aw8697);
	if (aw8697->amplitude > 0) {
		aw8697_haptic_ram_vbat_comp(aw8697, false);
		aw8697_haptic_play_wav_seq(aw8697, aw8697->amplitude);
	}

	mutex_unlock(&aw8697->lock);

}
#endif

/*
static ssize_t aw8697_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->state);
}

static ssize_t aw8697_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
*/
static ssize_t aw8697_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw8697->timer)) {
		time_rem = hrtimer_get_remaining(&aw8697->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t aw8697_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
	struct aw8697_play_info *play = &aw8697->play;
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	struct aw8697_play_info *play = &aw8697->play;
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw8697->duration = val;
	play->playLength = val * USEC_PER_MSEC;
	play->ram_id = PLAYBACK_INFINITELY_RAM_ID;
	play->type = TIME_TYPE;

	return count;
}

static ssize_t aw8697_activate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->state);
}

static ssize_t aw8697_activate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 0 && val != 1)
		return count;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	#if 0
	mutex_lock(&aw8697->lock);


	hrtimer_cancel(&aw8697->timer);

	aw8697->state = val;

	if (aw8697->state) {
		/* clip value to max */
		val = aw8697->duration;
		/* run ms timer */
		hrtimer_start(&aw8697->timer,
				  ktime_set(val / 1000, (val % 1000) * 1000000),
				  HRTIMER_MODE_REL);
	}
	mutex_unlock(&aw8697->lock);
	schedule_work(&aw8697->vibrator_work);
	#else
	aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_PLAYBACK, val, NULL);
	#endif

	playRoll_flag = 0;

	return count;
}

static ssize_t aw8697_activate_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "activate_mode=%d\n", aw8697->activate_mode);
}

static ssize_t aw8697_activate_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697->lock);
	aw8697->activate_mode = val;
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_index_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned char reg_val = 0;
	aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1, &reg_val);
	aw8697->index = reg_val;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->index);
}

static ssize_t aw8697_index_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw8697->lock);
	aw8697->index = val;
	aw8697_haptic_set_repeat_wav_seq(aw8697, aw8697->index);
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_vmax_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697->vmax);
}

static ssize_t aw8697_vmax_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw8697->lock);
	aw8697->vmax = val;
	aw8697_haptic_set_bst_vol(aw8697, aw8697->vmax);
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697->gain);
}

static ssize_t aw8697_gain_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw8697->lock);
	aw8697->gain = val;
	if (aw8697->gain == 0)
		aw8697_haptic_set_gain(aw8697, 0x70);
	else if (aw8697->gain == 1)
		aw8697_haptic_set_gain(aw8697, 0x80);
	else if (aw8697->gain == 2)
		aw8697_haptic_set_gain(aw8697, 0x90);
	else
		aw8697_haptic_set_gain(aw8697, 0x80);
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_seq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_SEQUENCER_SIZE; i++) {
		aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1+i, &reg_val);
		count += snprintf(buf+count, PAGE_SIZE-count,
				"seq%d: 0x%02x\n", i+1, reg_val);
		aw8697->seq[i] |= reg_val;
	}
	return count;
}

static ssize_t aw8697_seq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		pr_debug("%s: seq%d=0x%x\n", __FUNCTION__, databuf[0], databuf[1]);
		mutex_lock(&aw8697->lock);
		aw8697->seq[databuf[0]] = (unsigned char)databuf[1];
		aw8697_haptic_set_wav_seq(aw8697, (unsigned char)databuf[0],
				aw8697->seq[databuf[0]]);
		mutex_unlock(&aw8697->lock);
	}
	return count;
}

static ssize_t aw8697_loop_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_SEQUENCER_LOOP_SIZE; i++) {
		aw8697_i2c_read(aw8697, AW8697_REG_WAVLOOP1+i, &reg_val);
		aw8697->loop[i*2+0] = (reg_val>>4)&0x0F;
		aw8697->loop[i*2+1] = (reg_val>>0)&0x0F;

		count += snprintf(buf+count, PAGE_SIZE-count,
				"seq%d loop: 0x%02x\n", i*2+1, aw8697->loop[i*2+0]);
		count += snprintf(buf+count, PAGE_SIZE-count,
				"seq%d loop: 0x%02x\n", i*2+2, aw8697->loop[i*2+1]);
	}
	return count;
}

static ssize_t aw8697_loop_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		pr_debug("%s: seq%d loop=0x%x\n", __FUNCTION__, databuf[0], databuf[1]);
		mutex_lock(&aw8697->lock);
		aw8697->loop[databuf[0]] = (unsigned char)databuf[1];
		aw8697_haptic_set_wav_loop(aw8697, (unsigned char)databuf[0],
				aw8697->loop[databuf[0]]);
		mutex_unlock(&aw8697->lock);
	}

	return count;
}

static ssize_t aw8697_reg_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for (i = 0; i < AW8697_REG_MAX; i++) {
		if (!(aw8697_reg_access[i]&REG_RD_ACCESS))
		   continue;
		aw8697_i2c_read(aw8697, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}

static ssize_t aw8697_reg_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8697_i2c_write(aw8697, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8697_rtp_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "rtp play: %d\n", aw8697->rtp_cnt);

	return len;
}

static ssize_t aw8697_rtp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw8697_haptic_stop(aw8697);
	aw8697_haptic_set_rtp_aei(aw8697, false);
	aw8697_interrupt_clear(aw8697);
#if 0
	if (val < (sizeof(aw8697_rtp_name) / AW8697_RTP_NAME_MAX)) {
#else
	if (val < aw8697->rtp_effect_count) {

#endif
		aw8697->rtp_file_num = val;
		if (val) {
			//schedule_work(&aw8697->rtp_work);
			queue_work(rtp_wq, &aw8697->rtp_work);

		}
	} else {
		pr_err("%s: rtp_file_num 0x%02x over max value \n", __func__, aw8697->rtp_file_num);
	}

	return count;
}

static ssize_t aw8697_ram_update_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	//struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "sram update mode\n");
	return len;
}

static ssize_t aw8697_ram_update_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		aw8697_ram_update(aw8697);
	}
	return count;
}

static ssize_t aw8697_f0_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8697->lock);
	aw8697->f0_cali_flag = AW8697_HAPTIC_LRA_F0;
	aw8697_haptic_get_f0(aw8697);
	mutex_unlock(&aw8697->lock);
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 lra f0 = %d\n", aw8697->f0);
	return len;
}

static ssize_t aw8697_f0_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	//struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t aw8697_cont_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	aw8697_haptic_read_cont_f0(aw8697);
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont f0 = %d\n", aw8697->cont_f0);
	return len;
}

static ssize_t aw8697_cont_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw8697_haptic_stop(aw8697);
	if (val) {
		aw8697_haptic_cont(aw8697);
	}
	return count;
}


static ssize_t aw8697_cont_td_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont delay time = 0x%04x\n", aw8697->cont_td);
	return len;
}

static ssize_t aw8697_cont_td_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = {0};
	if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw8697->cont_td = databuf[0];
		aw8697_i2c_write(aw8697, AW8697_REG_TD_H, (unsigned char)(databuf[0]>>8));
		aw8697_i2c_write(aw8697, AW8697_REG_TD_L, (unsigned char)(databuf[0]>>0));
	}
	return count;
}

static ssize_t aw8697_cont_drv_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont drv level = %d\n", aw8697->cont_drv_lvl);
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont drv level overdrive= %d\n", aw8697->cont_drv_lvl_ov);
	return len;
}

static ssize_t aw8697_cont_drv_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = {0, 0};
	if (2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
		aw8697->cont_drv_lvl = databuf[0];
		aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, aw8697->cont_drv_lvl);
		aw8697->cont_drv_lvl_ov = databuf[1];
		aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL_OV, aw8697->cont_drv_lvl_ov);
	}
	return count;
}

static ssize_t aw8697_cont_num_brk_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont break num = %d\n", aw8697->cont_num_brk);
	return len;
}

static ssize_t aw8697_cont_num_brk_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = {0};
	if (1 == sscanf(buf, "%d", &databuf[0])) {
		aw8697->cont_num_brk = databuf[0];
		if (aw8697->cont_num_brk > 7) {
			aw8697->cont_num_brk = 7;
		}
		aw8697_i2c_write_bits(aw8697, AW8697_REG_BEMF_NUM,
				AW8697_BIT_BEMF_NUM_BRK_MASK, aw8697->cont_num_brk);
	}
	return count;
}

static ssize_t aw8697_cont_zc_thr_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8697 cont zero cross thr = 0x%04x\n", aw8697->cont_zc_thr);
	return len;
}

static ssize_t aw8697_cont_zc_thr_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = {0};
	if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw8697->cont_zc_thr = databuf[0];
		aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_H, (unsigned char)(databuf[0]>>8));
		aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_L, (unsigned char)(databuf[0]>>0));
	}
	return count;
}

static ssize_t aw8697_vbat_monitor_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8697->lock);
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_get_vbat(aw8697);
	len += snprintf(buf+len, PAGE_SIZE-len, "vbat=%dmV\n", aw8697->vbat);
	mutex_unlock(&aw8697->lock);

	return len;
}

static ssize_t aw8697_vbat_monitor_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}

static void aw8697_lra_get(struct aw8697 *aw8697, unsigned char *reg_val)
{

	pr_info("%s enter\n", __func__);

	aw8697_haptic_stop(aw8697);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
		AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
		AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
		AW8697_BIT_ANACTRL_HD_PD_MASK, AW8697_BIT_ANACTRL_HD_HZ_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
		AW8697_BIT_D2SCFG_CLK_ADC_MASK, AW8697_BIT_D2SCFG_CLK_ASC_1P5MHZ);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
		AW8697_BIT_DETCTRL_RL_OS_MASK, AW8697_BIT_DETCTRL_RL_DETECT);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
		AW8697_BIT_DETCTRL_DIAG_GO_MASK, AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
	msleep(3);
	aw8697_i2c_read(aw8697, AW8697_REG_RLDET, reg_val);
	aw8697->lra = 298 * (*reg_val);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
		AW8697_BIT_ANACTRL_HD_PD_MASK, AW8697_BIT_ANACTRL_HD_PD_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
		AW8697_BIT_D2SCFG_CLK_ADC_MASK, AW8697_BIT_D2SCFG_CLK_ASC_6MHZ);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
		AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
		AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);

}

static ssize_t aw8697_lra_resistance_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);
	mutex_lock(&aw8697->lock);

#if 0

	aw8697_haptic_stop(aw8697);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);


	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
			AW8697_BIT_ANACTRL_HD_PD_MASK, AW8697_BIT_ANACTRL_HD_HZ_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
			AW8697_BIT_D2SCFG_CLK_ADC_MASK, AW8697_BIT_D2SCFG_CLK_ASC_1P5MHZ);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			AW8697_BIT_DETCTRL_RL_OS_MASK, AW8697_BIT_DETCTRL_RL_DETECT);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			AW8697_BIT_DETCTRL_DIAG_GO_MASK, AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
	msleep(3);
	aw8697_i2c_read(aw8697, AW8697_REG_RLDET, &reg_val);
	aw8697->lra = 298 * reg_val;
	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
			AW8697_BIT_ANACTRL_HD_PD_MASK, AW8697_BIT_ANACTRL_HD_PD_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
			AW8697_BIT_D2SCFG_CLK_ADC_MASK, AW8697_BIT_D2SCFG_CLK_ASC_6MHZ);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);

#else
	aw8697_lra_get(aw8697, &reg_val);
	aw8697->lra = 298 * reg_val;
#endif
	/* 如果阻抗超范围，说明马达器件异常，则禁止输出振动信号 */
	if ((aw8697->lra >= aw8697->resistance_min && aw8697->lra <= aw8697->resistance_max)) {
		pr_err("%s lra resistent test ok, lra=%d\n", __func__, aw8697->lra);
		aw8697->perm_disable = false;
	} else {
		pr_err("%s lra resistent over range, lra=%d\n", __func__, aw8697->lra);
		aw8697->perm_disable = true;
		aw8697->trig[0].enable = 0;
		aw8697->trig[1].enable = 0;
		aw8697->trig[2].enable = 0;
		aw8697_haptic_trig_enable_config(aw8697);
	}
	mutex_unlock(&aw8697->lock);

    if (1 == aw8697->is_new_gongmo) {
        return snprintf(buf, PAGE_SIZE, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
            (aw8697->lra >= aw8697->resistance_min && aw8697->lra <= aw8697->resistance_max) ? "ok" : "fail",
            aw8697->lra/1000, aw8697->lra%1000,
            aw8697->resistance_min/1000, aw8697->resistance_min%1000/100,
            aw8697->resistance_max/1000, aw8697->resistance_max%1000/100);
    }
    else {
        return snprintf(buf, PAGE_SIZE, "%s %d.%d (range:%d.%d-%d.%d)ou\n",
            (aw8697->lra >= aw8697->resistance_min && aw8697->lra <= aw8697->resistance_max) ? "ok" : "fail",
            aw8697->lra/1000, aw8697->lra%1000,
            aw8697->resistance_min/1000, aw8697->resistance_min%1000/100,
            aw8697->resistance_max/1000, aw8697->resistance_max%1000/100);
    }
}

static ssize_t aw8697_lra_resistance_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697_auto_boost_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "auto_boost=%d\n", aw8697->auto_boost);

	return len;
}


static ssize_t aw8697_auto_boost_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697->lock);
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_auto_boost_config(aw8697, val);
	mutex_unlock(&aw8697->lock);

	return count;
}

static ssize_t aw8697_prctmode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char reg_val = 0;

	aw8697_i2c_read(aw8697, AW8697_REG_RLDET, &reg_val);

	len += snprintf(buf+len, PAGE_SIZE-len, "prctmode=%d\n", reg_val&0x20);
	return len;
}


static ssize_t aw8697_prctmode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
	#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	#endif
	unsigned int databuf[2] = {0, 0};
	unsigned int addr = 0;
	unsigned int val = 0;
	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		addr = databuf[0];
		val = databuf[1];
		mutex_lock(&aw8697->lock);
		aw8697_haptic_swicth_motorprotect_config(aw8697, addr, val);
		mutex_unlock(&aw8697->lock);
   }
	return count;
}

static ssize_t aw8697_trig_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;
	for (i = 0; i < AW8697_TRIG_NUM; i++) {
		len += snprintf(buf+len, PAGE_SIZE-len,
			"trig%d: enable=%d, default_level=%d, dual_edge=%d, frist_seq=%d, second_seq=%d\n",
			i+1, aw8697->trig[i].enable, aw8697->trig[i].default_level, aw8697->trig[i].dual_edge,
			aw8697->trig[i].frist_seq, aw8697->trig[i].second_seq);
	}

	return len;
}

static ssize_t aw8697_trig_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
	#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	#endif
	unsigned int databuf[6] = {0};
	if (sscanf(buf, "%d %d %d %d %d %d",
			&databuf[0], &databuf[1], &databuf[2], &databuf[3], &databuf[4], &databuf[5])) {
		pr_debug("%s: %d, %d, %d, %d, %d, %d\n", __func__,
			databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
		if (databuf[0] > 3) {
			databuf[0] = 3;
		}
		if (databuf[0] > 0) {
			databuf[0] -= 1;
		}
		aw8697->trig[databuf[0]].enable = databuf[1];
		aw8697->trig[databuf[0]].default_level = databuf[2];
		aw8697->trig[databuf[0]].dual_edge = databuf[3];
		aw8697->trig[databuf[0]].frist_seq = databuf[4];
		aw8697->trig[databuf[0]].second_seq = databuf[5];
		mutex_lock(&aw8697->lock);
		aw8697_haptic_trig_param_config(aw8697);
		aw8697_haptic_trig_enable_config(aw8697);
		mutex_unlock(&aw8697->lock);
   }
	return count;
}

static ssize_t aw8697_ram_vbat_comp_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "ram_vbat_comp=%d\n", aw8697->ram_vbat_comp);

	return len;
}


static ssize_t aw8697_ram_vbat_comp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697->lock);
	if (val) {
		aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;
	} else {
		aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_DISABLE;
	}
	mutex_unlock(&aw8697->lock);

	return count;
}

static ssize_t aw8697_osc_cali_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#if 0
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
#endif
	//strncpy(buf, osc_cali_result, strlen(osc_cali_result) + 1);
	//pr_err("%s osc_cali_result : %s", __func__, osc_cali_result);
	//pr_err("%s osc_cali_result : %s", __func__, buf);
	at_test = 0;
	return snprintf(buf, PAGE_SIZE, "%s", osc_cali_result);
}

static ssize_t aw8697_osc_cali_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	unsigned long int theory_times = DEFAULT_OSC_CALI_TIMES_US;

	int rc = 0;
	int ret = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	at_test = 1;
	pr_info("%s enter, val=%d\n", __func__, val);
	/* 先校准，然后读取时间确认 */
	mutex_lock(&aw8697->lock);
	if (val == 3) { //校准
		memset(osc_cali_result, 0, sizeof(osc_cali_result));
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, 0x00);
		aw8697_rtp_osc_calibration(aw8697);
		ret = aw8697_rtp_trim_lra_calibration(aw8697);
		if (ret == CALI_OVER_RANGE) {
			/* 表示晶振偏移过大，硬件异常*/
			osc_cali_result_mode = CALI_OVER_RANGE;
			snprintf(osc_cali_result, sizeof(osc_cali_result), "failed osc over range");
		} else if (ret == CALI_NO_NEED) {
			/* 表示晶振偏移在千分之一以内，无需校准*/
			osc_cali_result_mode = CALI_NO_NEED;
			snprintf(osc_cali_result, sizeof(osc_cali_result), "ok_1 osc_data=0");
		} else if (ret == CALI_NARMAL) {
			/* 正常校准，需要再次复核时间，2.5 / 1000 表示校准有效 */
			osc_cali_result_mode = CALI_NARMAL;
			pr_err("%s micro times -- before cali = %d\n", __func__, aw8697->microsecond);
		}
	}

	if (val == 1) { //读取时间
		aw8697_rtp_osc_calibration(aw8697);
		if (osc_cali_result_mode == CALI_NARMAL) {

			if (aw8697->microsecond > theory_times) {

				if ((aw8697->microsecond - theory_times) < (theory_times * 25 / 10000)) {
					pr_err("%s normal cali success\n", __func__);
					snprintf(osc_cali_result, sizeof(osc_cali_result), "ok_2 osc_data=%d", atomic_read(&aw8697->standard_osc_freq_cali));
				} else {
					pr_err("%s normal cali failed\n", __func__);
					snprintf(osc_cali_result, sizeof(osc_cali_result), "failed osc failed");
				}

			} else {

				if ((theory_times - aw8697->microsecond) < (theory_times * 25 / 10000)) {
					pr_err("%s normal cali success\n", __func__);
					snprintf(osc_cali_result, sizeof(osc_cali_result), "ok_2 osc_data=%d", atomic_read(&aw8697->standard_osc_freq_cali));
				} else {
					pr_err("%s normal cali failed\n", __func__);
					snprintf(osc_cali_result, sizeof(osc_cali_result), "failed osc failed");
				}
			}

		}
	}

	pr_err("%s osc_cali_result:%s\n", __func__, osc_cali_result);

	mutex_unlock(&aw8697->lock);

	return count;
}


#if 0
static ssize_t aw8697_haptic_audio_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
#else
static ssize_t aw8697_haptic_audio_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct aw8697 *aw8697 = g_aw8697;
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw8697->haptic_audio.cnt);
	return len;
}


		
		
/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static void aw8697_haptic_audio_ctr_list_insert(struct haptic_audio
		*haptic_audio, struct haptic_ctr *haptic_ctr)
{
	spin_lock(&haptic_audio->list_lock);
	INIT_LIST_HEAD(&(haptic_ctr->list));
	list_add(&(haptic_ctr->list), &(haptic_audio->ctr_list));
	spin_unlock(&haptic_audio->list_lock);

}


		
static void aw8697_haptic_audio_ctr_list_clear(struct haptic_audio *haptic_audio)
{
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;

	spin_lock(&haptic_audio->list_lock);
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
		&(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
	}
	spin_unlock(&haptic_audio->list_lock);

}


static void aw8697_haptic_audio_init(struct aw8697 *aw8697)
{

	aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);

}


static void aw8697_haptic_audio_off(struct aw8697 *aw8697)
{
	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw8697->lock);
	//aw8697_haptic_set_gain(aw8697, 0x80);
	aw8697_haptic_stop(aw8697);
	//aw8697_haptic_audio_ctr_list_clear(&aw8697->haptic_audio);
	mutex_unlock(&aw8697->lock);

}

static void aw8697_haptic_audio_cancel(struct aw8697 *aw8697)
{

	pr_info("%s: haptic_audio stop\n", __func__);

	if (hrtimer_active(&aw8697->haptic_audio.timer)) {
		pr_info("%s: cancel haptic_audio_timer\n", __func__);
		hrtimer_cancel(&aw8697->haptic_audio.timer);
		//aw8697->haptic_audio.ctr.cnt = 0;
		//aw8697_haptic_audio_off(aw8697);
	}
	aw8697->haptic_audio.ctr.cnt = 0;
	aw8697->haptic_audio.haptic_audio_cancel_flag = true;
	aw8697_haptic_audio_ctr_list_clear(&aw8697->haptic_audio);
}

static ssize_t aw8697_haptic_audio_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct aw8697 *aw8697 = g_aw8697;

	unsigned int databuf[6] = { 0 };
	struct haptic_ctr *hap_ctr = NULL;

	if (!aw8697->ram_init) {
		pr_err("%s: ram not init\n", __func__);
		return count;
	}
	pr_info("%s enter, buf=%s\n", __func__, buf);

	if (6 == sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1],
					&databuf[2], &databuf[3], &databuf[4], &databuf[5])) {
/*
		pr_info("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
				__func__, databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
*/
		hap_ctr = (struct haptic_ctr *)kzalloc(sizeof(struct haptic_ctr), GFP_KERNEL);
		if (hap_ctr == NULL) {
			pr_err("%s: kzalloc memory fail\n", __func__);
			return count;
		}


		hap_ctr->cnt = (unsigned char)databuf[0];
		hap_ctr->cmd = (unsigned char)databuf[1];
		hap_ctr->play = (unsigned char)databuf[2];
		hap_ctr->wavseq = (unsigned char)databuf[3];
		hap_ctr->loop = (unsigned char)databuf[4];
		hap_ctr->gain = (unsigned char)databuf[5];

		aw8697_haptic_audio_ctr_list_insert(&aw8697->haptic_audio, hap_ctr);			
		mutex_lock(&aw8697->haptic_audio.lock);

		if (hap_ctr->cmd == 0xff) {

			aw8697_haptic_audio_cancel(aw8697);
			aw8697_haptic_audio_off(aw8697);
			mutex_unlock(&aw8697->haptic_audio.lock);
			return count;

		} else {
			if (list_empty(&aw8697->ff_messages) && (aw8697->ff_play_finish)) {
				if (!hrtimer_active(&aw8697->haptic_audio.timer)) {

					pr_info("%s: start haptic_audio_timer\n", __func__);
					aw8697_haptic_audio_init(aw8697);
					hrtimer_start(&aw8697->haptic_audio.timer,
								ktime_set(aw8697->haptic_audio.delay_val / 1000000,
										(aw8697->haptic_audio.delay_val % 1000000) * 1000),
										HRTIMER_MODE_REL);
				} else
					pr_info("%s: audio timer is running now\n", __func__);

				aw8697->haptic_audio.haptic_audio_cancel_flag = false;
			}

			mutex_unlock(&aw8697->haptic_audio.lock);
			return count;
		}


	}

	pr_err("%s: sscanf failed, buf = %s\n", __func__, buf);

	return count;

}


static ssize_t aw8697_haptic_audio_time_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "haptic_audio.delay_val=%dus\n", aw8697->haptic_audio.delay_val);
	len += snprintf(buf+len, PAGE_SIZE-len, "haptic_audio.timer_val=%dus\n", aw8697->haptic_audio.timer_val);
	return len;
}

static ssize_t aw8697_haptic_audio_time_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = {0};

	if (2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
		aw8697->haptic_audio.delay_val = databuf[0];
		aw8697->haptic_audio.timer_val = databuf[1];
	}
	return count;
}

static ssize_t aw8697_effect_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "effect=%d\n", aw8697->effect);
	len += snprintf(buf+len, PAGE_SIZE-len, "effect_strength=%d\n", aw8697->effect_strength);
	return len;
}

static ssize_t aw8697_effect_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = {0};

	pr_info("%s: %s\n", __func__, buf);

	if (2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
		mutex_lock(&aw8697->lock);
		aw8697->effect = databuf[0];
		aw8697->gain = aw8697->effect_strength = (databuf[1] == 1 ? 0x80 : 0x90);
		aw8697_haptic_set_gain(aw8697, aw8697->gain);
		aw8697_haptic_stop(aw8697);
		if (aw8697->effect_strength > 0) {
			aw8697_haptic_ram_vbat_comp(aw8697, false);
			aw8697_haptic_play_wav_seq(aw8697, 1);
		}
		mutex_unlock(&aw8697->lock);
	}
	return count;
}

static ssize_t aw8697_iic_int_rst_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	unsigned char reg = 0;
	
	pr_info("%s enter\n", __func__);

	mutex_lock(&aw8697->lock);
	aw8697_i2c_read(aw8697, AW8697_REG_ID, &reg);
	mutex_unlock(&aw8697->lock);
	if (reg != AW8697_CHIPID) {
		pr_err("%s:chip id incorrect! reg = %d\n", __func__, reg);
		return snprintf(buf, PAGE_SIZE, "+IIC:\"0\"\n+INT:\"1\"\n+RST:\"1\"\n");
	}
	return snprintf(buf, PAGE_SIZE, "+IIC:\"1\"\n+INT:\"1\"\n+RST:\"1\"\n");
}

static ssize_t aw8697_selftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char reg = 0;
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	pr_info("%s enter\n", __func__);

	mutex_lock(&aw8697->lock);
	aw8697_i2c_read(aw8697, AW8697_REG_ID, &reg);
	mutex_unlock(&aw8697->lock);
	if (reg != AW8697_CHIPID) {
		pr_err("%s:chip id incorrect! reg = %d\n", __func__, reg);
		return snprintf(buf, PAGE_SIZE, "failed selftest\n");
	}

	mutex_lock(&aw8697->lock);
	aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
	aw8697_haptic_get_f0(aw8697);
	mutex_unlock(&aw8697->lock);
	pr_err("%s:f0 = %d\n", __func__, aw8697->f0);
	if (aw8697->f0 < aw8697->freq_min || aw8697->f0 > aw8697->freq_max) {
		pr_err("%s:f0 incorrect! f0 = %d\n", __func__, aw8697->f0);
		return snprintf(buf, PAGE_SIZE, "failed selftest\n");
	}

	msleep(100);
	mutex_lock(&aw8697->lock);
	aw8697_haptic_stop(aw8697);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);


	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
			AW8697_BIT_ANACTRL_HD_PD_MASK, AW8697_BIT_ANACTRL_HD_HZ_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
			AW8697_BIT_D2SCFG_CLK_ADC_MASK, AW8697_BIT_D2SCFG_CLK_ASC_1P5MHZ);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			AW8697_BIT_DETCTRL_RL_OS_MASK, AW8697_BIT_DETCTRL_RL_DETECT);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			AW8697_BIT_DETCTRL_DIAG_GO_MASK, AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
	msleep(3);
	aw8697_i2c_read(aw8697, AW8697_REG_RLDET, &reg);
	aw8697->lra = 298 * reg;

	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
			AW8697_BIT_ANACTRL_HD_PD_MASK, AW8697_BIT_ANACTRL_HD_PD_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
			AW8697_BIT_D2SCFG_CLK_ADC_MASK, AW8697_BIT_D2SCFG_CLK_ASC_6MHZ);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
	mutex_unlock(&aw8697->lock);
	pr_info("%s: lra = %dmohm\n", __func__, aw8697->lra);
	if (aw8697->lra < aw8697->resistance_min || aw8697->lra > aw8697->resistance_max) {
		pr_err("%s:lra incorrect! lra = %d\n", __func__, aw8697->lra);
		return snprintf(buf, PAGE_SIZE, "failed selftest\n");
	}
	return snprintf(buf, PAGE_SIZE, "ok selftest\n");
}

static ssize_t aw8697_selftest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697_at_trigger_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t len = 0;
	unsigned char reg = 0;
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	pr_info("%s enter\n", __func__);
	mutex_lock(&aw8697->lock);
	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg);
	pr_info("%s reg2=%#x\n", __func__, reg);
	if (reg & 0x01) {
		len += snprintf(buf+len, PAGE_SIZE-len, "1");
	} else {
		len += snprintf(buf+len, PAGE_SIZE-len, "0");
	}
	at_test = 0; //清空屏蔽
	mutex_unlock(&aw8697->lock);

	return len;
}

static ssize_t aw8697_at_trigger_state_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0, ret = 0;
	unsigned char reg = 0;
	struct aw8697_ff_msg *msg = NULL;
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	pr_info("aw8697, at_trigger_state_store enter, %s\n", buf);

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		pr_err("aw8697, at_trigger_state_store kstrtoint failed, ret=%d\n", ret);
		return -EINVAL;
	}
	mutex_lock(&aw8697->lock);
	at_test = val;
	/* 清空链表中的操作 */
	spin_lock(&aw8697->ff_lock);
	while (1) {
		if (!list_empty(&aw8697->ff_messages)) {
			msg = list_first_entry(&aw8697->ff_messages, struct aw8697_ff_msg, list);
			list_del(&msg->list);
			kmem_cache_free(aw8697->ff_km, msg);
		} else {
			pr_err("%s list is empty\n", __func__);
			break;
		}
	}
	spin_unlock(&aw8697->ff_lock);

	/* 清除timer*/
	if (hrtimer_active(&aw8697->timer)) {
		hrtimer_cancel(&aw8697->timer);
		pr_err("%s cancel timer\n", __func__);
	}
	aw8697_haptic_stop(aw8697);
	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg);
	pr_info("%s reg=%#x\n", __func__, reg);
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_last_cali_f0_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif


	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&aw8697->f0_freq_cali));

}
static ssize_t aw8697_last_cali_f0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	int ret = 0;
	int val = 0;

	pr_info("%s enter, f0=%s\n", __func__, buf);

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		pr_err("%s kstrtoint failed, ret=%d\n", __func__, ret);
		return ret;
	}

	if (val < 0) {
		pr_err("%s val error, val=%d\n", __func__, val);
	} else
		atomic_set(&aw8697->f0_freq_cali, val);

	return count;
}

static ssize_t aw8697_last_osc_freq_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif


	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&aw8697->standard_osc_freq_cali));

}
static ssize_t aw8697_last_osc_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	int ret = 0;
	int val = 0;

	pr_info("%s enter, f0=%s\n", __func__, buf);

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		pr_err("%s kstrtoint failed, ret=%d\n", __func__, ret);
		return ret;
	}

	if (val < 0) {
		pr_err("%s val error, val=%d\n", __func__, val);
	} else
		atomic_set(&aw8697->standard_osc_freq_cali, val);

	return count;
}

static ssize_t aw8697_playRoll_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	pr_debug("%s enter\n", __func__);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->have_test_timse);

}

static void aw8697_playRoll_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 = container_of(work, struct aw8697, playRoll_work);
	
	struct aw8697_play_info *play = NULL;
	int i;
	struct aw8697_wavefrom_ram_info *waveform_ram_table = aw8697->waveform_ram_table;

	if (aw8697->have_test_timse < aw8697->total_test_times && !!at_test) {
		play = kzalloc(sizeof(struct aw8697_play_info), GFP_KERNEL);
		if (play == NULL) {
			pr_err("%s kzalloc failed\n", __func__);
			at_test = 0;
			return;
		}

		play->type = TIME_TYPE;
		play->playLength = 500 * USEC_PER_MSEC;//ms
		aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_UPLOAD_EFFECT, 0, play);
		aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_PLAYBACK, 1, NULL);
		msleep(500 + 10);
		aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_PLAYBACK, 0, NULL);

		for (i = 0; i < aw8697->ram_effect_count; i++) {
			play = kzalloc(sizeof(struct aw8697_play_info), GFP_KERNEL);
				if (play == NULL) {
					pr_err("%s kzalloc failed\n", __func__);
					at_test = 0;
					return;
			}

			play->vmax = waveform_ram_table[i].vmax;;
			play->times_us = waveform_ram_table[i].times_us;
			play->type = RAM_TYPE;
			play->ram_id = waveform_ram_table[i].ram_id;

			play->type = RAM_TYPE;
			play->playLength = 500 * USEC_PER_MSEC;//ms
			aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_UPLOAD_EFFECT, 0, play);
			aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_PLAYBACK, 1, NULL);
			msleep(play->times_us / 1000 + 10);
			aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_PLAYBACK, 0, NULL);	
		}

		aw8697->have_test_timse++;

		msleep(20);
		if (!!playRoll_flag) {
			schedule_work(&aw8697->playRoll_work);
		} else {
			at_test = 0;
		}
	} else {
		at_test = 0;
		playRoll_flag = 0;
	}

}

static ssize_t aw8697_playRoll_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	int ret = 0;
	u32 val = 0;

	pr_info("%s enter, buf=%s\n", __func__, buf);

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		pr_err("%s kstrtoint failed, ret=%d\n", __func__, ret);
		return -EINVAL;
	}
	at_test = 1;
	aw8697->total_test_times = val;
	aw8697->have_test_timse = 0;
	playRoll_flag = 1;
	schedule_work(&aw8697->playRoll_work);

	return count;
}
//lizeyuan add for gongmo ETD1915
//先校准再读f0值，再读lra_resistance值
static ssize_t aw8697_cali_f0_resis_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    unsigned char reg_val = 0;
	ssize_t len = 0;
    mutex_lock(&aw8697->lock);
	//在测试之前，将状态改为测试状态，并停止当前所有振动
	at_test = 1;

    //先读取阻抗值
    aw8697_lra_get(aw8697, &reg_val);
	aw8697->lra = 298 * reg_val;

    if ((aw8697->lra >= aw8697->resistance_min) && (aw8697->lra <= aw8697->resistance_max)) {
		pr_err("%s lra resistent test ok, lra=%d\n", __func__, aw8697->lra);
		aw8697->perm_disable = false;
	} else {
		pr_err("%s lra resistent over range, lra=%d\n", __func__, aw8697->lra);
        //aw8697->perm_disable标志位为true
		aw8697->perm_disable = true;
	}

    len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw8697->lra >= aw8697->resistance_min && aw8697->lra <= aw8697->resistance_max) ? "ok" : "fail",
		aw8697->lra/1000, aw8697->lra%1000,
		aw8697->resistance_min/1000, aw8697->resistance_min%1000/100,
		aw8697->resistance_max/1000, aw8697->resistance_max%1000/100);

    //中间延时
    mdelay(20);

    //先校准再读取f0值
	aw8697_haptic_f0_calibration(aw8697);

    //中间延时
    mdelay(200);

    //获取校准值
	aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
	aw8697_haptic_get_f0(aw8697);
    
	len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
		(aw8697->f0 >= aw8697->freq_min && aw8697->f0 <= aw8697->freq_max) ? "ok" : "fail",
		aw8697->f0/10, aw8697->f0%10,
		aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10);

	len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#%s#%d\n",
		(aw8697->f0 >= aw8697->freq_min && aw8697->f0 <= aw8697->freq_max) ? "ok" : "fail",
		atomic_read(&aw8697->f0_freq_cali));

    //在测试之后，解除测试状态
	at_test = 0;
    mutex_unlock(&aw8697->lock);

    return len;
}

static ssize_t aw8697_cali_f0_resis_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697_resis_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

    unsigned char reg_val = 0;
	ssize_t len = 0;
    mutex_lock(&aw8697->lock);
	//在测试之前，将状态改为测试状态，并停止当前所有振动
	at_test = 1;

    //先读取阻抗值
    aw8697_lra_get(aw8697, &reg_val);
	aw8697->lra = 298 * reg_val;

    if ((aw8697->lra >= aw8697->resistance_min) && (aw8697->lra <= aw8697->resistance_max)) {
		pr_err("%s lra resistent test ok, lra=%d\n", __func__, aw8697->lra);
		aw8697->perm_disable = false;
	} else {
		pr_err("%s lra resistent over range, lra=%d\n", __func__, aw8697->lra);
        //aw8697->perm_disable标志位为true
		aw8697->perm_disable = true;
	}

    len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw8697->lra >= aw8697->resistance_min && aw8697->lra <= aw8697->resistance_max) ? "ok" : "fail",
		aw8697->lra/1000, aw8697->lra%1000,
		aw8697->resistance_min/1000, aw8697->resistance_min%1000/100,
		aw8697->resistance_max/1000, aw8697->resistance_max%1000/100);

    //中间延时
    mdelay(20);

    //获取校准值
    aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
	aw8697_haptic_get_f0(aw8697);
    
	len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
		(aw8697->f0 >= aw8697->freq_min && aw8697->f0 <= aw8697->freq_max) ? "ok" : "fail",
		aw8697->f0/10, aw8697->f0%10,
		aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10);

	//在测试之后，解除测试状态
	at_test = 0;

    mutex_unlock(&aw8697->lock);

    return len;
}

static ssize_t aw8697_resis_f0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

//显示播放次数
static ssize_t aw8697_play_roll_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->have_test_timse);
}

//设置轮播次数，并开始轮播 如果输入0，则停止播放
static ssize_t aw8697_play_roll_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
    int ret = 0;
    u32 val = 0;
    
    pr_info("%s<%d> enter, buf=%s\n", __func__, __LINE__, buf);
    
    ret = kstrtoint(buf, 0, &val);
    if (ret) {
        pr_err("%s kstrtoint failed, ret=%d\n", __func__, ret);
        return -EINVAL;
    }
    if (val > 0) { //大于0 表示循环播放的次数
        //查看当前是否已经在测试了，如果在，则先停止当前测试
        at_test = 1;
        aw8697->total_test_times = val;
        aw8697->have_test_timse = 0;
        playRoll_flag = 1;
        schedule_work(&aw8697->playRoll_work);
    }
    else { //如果val小于等于0，则说明是停止振动
        at_test = 0;
        playRoll_flag = 0;
        aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_PLAYBACK, 0, NULL);
    }
	return count;
}

/*售后工具线性马达校准项
 *AT+BK_VBR_CAL=1  
 *【指令】：cat /sys/class/leds/vibrator/cali_f0
 *Z轴线性马达不需要校准，校准反而可能会影响震动效果
 *故offset值返回0，且不进行校准，只读f0
 */
static ssize_t aw8697_cali_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	ssize_t len = 0;
    mutex_lock(&aw8697->lock);
	//在测试之前，将状态改为测试状态，并停止当前所有振动
	at_test = 1;

    if(aw8697->lra_information == AW8697_LRA_1040){
      
        pr_info("%s:enter;aw8697->lra_information is 1040\n", __func__);
        
        len += snprintf(buf+len, PAGE_SIZE-len, "ok f0 170.0 (range:%d.%d-%d.%d)hz f0_offset=0\n",
    		aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10);

    }else{

        pr_info("%s:enter;aw8697->lra_information is not 1040\n", __func__);

        //先校准再读取f0值
    	aw8697_haptic_f0_calibration(aw8697);

        //中间延时
        mdelay(200);

        //获取校准值
    	aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
    	aw8697_haptic_get_f0(aw8697);


    	len += snprintf(buf+len, PAGE_SIZE-len, "%s f0 %d.%d (range:%d.%d-%d.%d)hz f0_offset=%d\n",
            (aw8697->f0 >= aw8697->freq_min && aw8697->f0 <= aw8697->freq_max) ? "ok" : "fail",
    		aw8697->f0/10, aw8697->f0%10,
    		aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10,
    		atomic_read(&aw8697->f0_freq_cali));
       
    }


	//在测试之后，解除测试状态
	at_test = 0;
    
    mutex_unlock(&aw8697->lock);
	
	return len;
}


static ssize_t aw8697_cali_f0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

//检测是否需要线性马达校准
static ssize_t aw8697_is_need_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	ssize_t len = 0;
    int need_cali = 1;
    char lra[] = "X-LRA 0619";

    mutex_lock(&aw8697->lock);
	//在测试之前，将状态改为测试状态，并停止当前所有振动
	at_test = 1;


    switch (aw8697->lra_information) {
        
        case AW8697_LRA_0815:
            strcpy(lra, "X-LRA 0815");
            need_cali = 1;
            break;
        case AW8697_LRA_1040:
            strcpy(lra, "Z-LRA 1040");
            need_cali = 0;
            break;
        case AW8697_LRA_0832:
            strcpy(lra, "Z-LRA 0832");
            need_cali = 0;
            break;
        default:
            strcpy(lra, "X-LRA 0619");
            need_cali = 1;
            break;

    }

	len += snprintf(buf+len, PAGE_SIZE-len, "+Type:\"%s\"\n", lra);
    len += snprintf(buf+len, PAGE_SIZE-len, "+Require:\"%d\"\n", need_cali);

	//在测试之后，解除测试状态
	at_test = 0;

    mutex_unlock(&aw8697->lock);
	return len;
}

static ssize_t aw8697_is_need_cali_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697_f0_offset_10_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	ssize_t len = 0;
	return len;
}

static ssize_t aw8697_f0_offset_10_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = {0};
    unsigned char reg_val = 0;
    unsigned int f0_limit = 0;
    char f0_cali_lra = 0;
    int f0_cali_step = 0;


	pr_info("%s:enter;buf is %s\n", __func__, buf);

	if (1 == sscanf(buf, "%d", &databuf[0])) {
		mutex_lock(&aw8697->lock);
        //在测试之前，将状态改为测试状态，并停止当前所有振动
	    at_test = 1;

        aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
        aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, 0x00);
    
        if (aw8697_haptic_get_f0(aw8697)) {
            pr_err("%s get f0 error, user defafult f0\n", __func__);
        } else {

            switch (databuf[0]) {

                case 0:
                    f0_limit = aw8697->f0;
                    break;
                case 10:
                    f0_limit = aw8697->f0 + 100;
                    break;
                case -10:
                    f0_limit = aw8697->f0 - 100;
                    break;
                default:
                    f0_limit = aw8697->f0;
                    break;
            }
           pr_debug("%s f0_pre = %d\n", __func__, aw8697->f0_pre);

        /* calculate cali step */
           f0_cali_step = 100000*((int)f0_limit-(int)aw8697->f0_pre) / ((int)f0_limit*25);
           pr_debug("%s f0_cali_step=%d\n", __func__, f0_cali_step);
            if (f0_cali_step >= 0) {   /*f0_cali_step >= 0*/
                if (f0_cali_step % 10 >= 5) {
                    f0_cali_step = f0_cali_step/10 + 1 + 32;
                } else {
                    f0_cali_step = f0_cali_step/10  + 32;
                }
            } else {  /*f0_cali_step < 0*/
                if (f0_cali_step % 10 <= -5) {
                  	f0_cali_step = 32 + (f0_cali_step/10 - 1);
                } else {
                    f0_cali_step = 32 + f0_cali_step/10;
                }
            }

    		if (f0_cali_step > 31) {
    			f0_cali_lra = (char)f0_cali_step - 32;
    		} else {
    			f0_cali_lra = (char)f0_cali_step + 32;
    		}
    		pr_info("%s f0_cali_lra=%d\n", __func__, (int)f0_cali_lra);
    		atomic_set(&aw8697->f0_freq_cali, f0_cali_lra);
    		/* update cali step */
    		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, (char)f0_cali_lra);
    		aw8697_i2c_read(aw8697, AW8697_REG_TRIM_LRA, &reg_val);
    		//aw8697->clock_system_f0_cali_lra = f0_cali_lra;//vincent for f0 cali and OSC cali
    		pr_info("%s final trim_lra=0x%02x\n", __func__, reg_val); 
        }
        /* restore default work mode */
        aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
        aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
        aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
        aw8697_haptic_stop(aw8697);

        //中间延时
        mdelay(100);
        pr_info("%s set freq to %dHZ\n", __func__,f0_limit); 

        //获取校准值
        //aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
        //aw8697_haptic_get_f0(aw8697);

        //在测试之后，解除测试状态
        at_test = 0;

        mutex_unlock(&aw8697->lock);

	}
	return count;
}



//static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw8697_state_show, aw8697_state_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw8697_duration_show, aw8697_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw8697_activate_show, aw8697_activate_store);
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, aw8697_activate_mode_show, aw8697_activate_mode_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw8697_index_show, aw8697_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw8697_vmax_show, aw8697_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw8697_gain_show, aw8697_gain_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw8697_seq_show, aw8697_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw8697_loop_show, aw8697_loop_store);
static DEVICE_ATTR(register, S_IWUSR | S_IRUGO, aw8697_reg_show, aw8697_reg_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw8697_rtp_show, aw8697_rtp_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw8697_ram_update_show, aw8697_ram_update_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO, aw8697_f0_show, aw8697_f0_store);
static DEVICE_ATTR(cont, S_IWUSR | S_IRUGO, aw8697_cont_show, aw8697_cont_store);
static DEVICE_ATTR(cont_td, S_IWUSR | S_IRUGO, aw8697_cont_td_show, aw8697_cont_td_store);
static DEVICE_ATTR(cont_drv, S_IWUSR | S_IRUGO, aw8697_cont_drv_show, aw8697_cont_drv_store);
static DEVICE_ATTR(cont_num_brk, S_IWUSR | S_IRUGO, aw8697_cont_num_brk_show, aw8697_cont_num_brk_store);
static DEVICE_ATTR(cont_zc_thr, S_IWUSR | S_IRUGO, aw8697_cont_zc_thr_show, aw8697_cont_zc_thr_store);
static DEVICE_ATTR(vbat_monitor, S_IWUSR | S_IRUGO, aw8697_vbat_monitor_show, aw8697_vbat_monitor_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO, aw8697_lra_resistance_show, aw8697_lra_resistance_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, aw8697_auto_boost_show, aw8697_auto_boost_store);
static DEVICE_ATTR(prctmode, S_IWUSR | S_IRUGO, aw8697_prctmode_show, aw8697_prctmode_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw8697_trig_show, aw8697_trig_store);
static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO, aw8697_ram_vbat_comp_show, aw8697_ram_vbat_comp_store);
static DEVICE_ATTR(osc_cali, S_IWUSR | S_IRUGO, aw8697_osc_cali_show, aw8697_osc_cali_store);
//static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, aw8697_haptic_audio_show, aw8697_haptic_audio_store);
static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO, aw8697_haptic_audio_time_show, aw8697_haptic_audio_time_store);
static DEVICE_ATTR(effect, S_IWUSR | S_IRUGO, aw8697_effect_show, aw8697_effect_store);
static DEVICE_ATTR(selftest, S_IWUSR | S_IRUGO, aw8697_selftest_show, aw8697_selftest_store);
static DEVICE_ATTR(iic_int_rst, S_IWUSR | S_IRUGO, aw8697_iic_int_rst_show, NULL);
static DEVICE_ATTR(at_trigger_state, S_IWUSR | S_IRUGO, aw8697_at_trigger_state_show, aw8697_at_trigger_state_store);
static DEVICE_ATTR(last_cali_f0, S_IWUSR | S_IRUGO, aw8697_last_cali_f0_show, aw8697_last_cali_f0_store);
static DEVICE_ATTR(last_osc_freq, S_IWUSR | S_IRUGO, aw8697_last_osc_freq_show, aw8697_last_osc_freq_store);
static DEVICE_ATTR(playRoll, S_IWUSR | S_IRUGO, aw8697_playRoll_show, aw8697_playRoll_store);
//lizeyuan add for gongmo ETD1915
static DEVICE_ATTR(cali_f0_resis, S_IWUSR | S_IRUGO, aw8697_cali_f0_resis_show, aw8697_cali_f0_resis_store);
static DEVICE_ATTR(resis_f0, S_IWUSR | S_IRUGO, aw8697_resis_f0_show, aw8697_resis_f0_store);
static DEVICE_ATTR(play_roll, S_IWUSR | S_IRUGO, aw8697_play_roll_show, aw8697_play_roll_store);
static DEVICE_ATTR(cali_f0, S_IWUSR | S_IRUGO, aw8697_cali_f0_show, aw8697_cali_f0_store);
static DEVICE_ATTR(is_need_cali, S_IWUSR | S_IRUGO, aw8697_is_need_cali_show, aw8697_is_need_cali_store);
//lizeyuan add for fo+-10HZ
static DEVICE_ATTR(f0_offset_10, S_IWUSR | S_IRUGO, aw8697_f0_offset_10_show, aw8697_f0_offset_10_store);






static struct attribute *aw8697_vibrator_attributes[] = {
//	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_register.attr,
	&dev_attr_rtp.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_f0.attr,
	&dev_attr_cont.attr,
	&dev_attr_cont_td.attr,
	&dev_attr_cont_drv.attr,
	&dev_attr_cont_num_brk.attr,
	&dev_attr_cont_zc_thr.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_auto_boost.attr,
	&dev_attr_prctmode.attr,
	&dev_attr_trig.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_osc_cali.attr,
	//&dev_attr_haptic_audio.attr,
	&dev_attr_haptic_audio_time.attr,
	&dev_attr_effect.attr,
	&dev_attr_selftest.attr,
	&dev_attr_iic_int_rst.attr,
	&dev_attr_at_trigger_state.attr,//获取trigger通路状态
	&dev_attr_last_cali_f0.attr, //上一次校准的f0，存储在属性中
	&dev_attr_last_osc_freq.attr, //上一次校准的f0，存储在属性中
	&dev_attr_playRoll.attr,
	//lizeyuan add for gongmo
	&dev_attr_cali_f0_resis.attr,
	&dev_attr_resis_f0.attr,
	&dev_attr_play_roll.attr,
	&dev_attr_cali_f0.attr,
	&dev_attr_is_need_cali.attr,
	&dev_attr_f0_offset_10.attr,
	NULL
};

static struct attribute_group aw8697_vibrator_attribute_group = {
	.attrs = aw8697_vibrator_attributes
};

static enum hrtimer_restart aw8697_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw8697 *aw8697 = container_of(timer, struct aw8697, timer);

	pr_debug("%s ---> enter\n", __func__);
	aw8697->state = 0;
	schedule_work(&aw8697->vibrator_work);

	return HRTIMER_NORESTART;
}

static void aw8697_vibrator_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 = container_of(work, struct aw8697, vibrator_work);

	//mutex_lock(&aw8697->lock);

	#if 0
	aw8697_haptic_stop(aw8697);
	if (aw8697->state) {
		if (aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_RAM_MODE) {
			aw8697_haptic_ram_vbat_comp(aw8697, true);
			aw8697_haptic_play_repeat_seq(aw8697, true);
			mutex_unlock(&aw8697->lock);
			return;
		} else if (aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_CONT_MODE) {
			aw8697_haptic_cont(aw8697);
			mutex_unlock(&aw8697->lock);
			return;
		} else {
		}
	}
	#endif
	aw8697_haptic_stop(aw8697);
	aw8697_vol_trig_switch(aw8697, true);
	aw8697_set_clock(aw8697, AW8697_HAPTIC_CLOCK_CALI_F0);
	//mutex_unlock(&aw8697->lock);
}

/* vivo zhangxiaodogn add for debug*/


static struct kobj_attribute vib_haptic_audio =
	__ATTR(haptic_audio, 0644, aw8697_haptic_audio_show, aw8697_haptic_audio_store);


static struct attribute *vibrator_node_sys_attrs[] = {
	&vib_haptic_audio.attr,
	NULL
};


static ssize_t vibrator_debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t vibrator_debug_object_store(struct kobject *k, struct attribute *attr,
	const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void vibrator_debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops vibrator_debug_object_sysfs_ops = {
	.show = vibrator_debug_object_show,
	.store = vibrator_debug_object_store,
};

static struct kobj_type vibrator_debug_object_type = {
		.sysfs_ops	= &vibrator_debug_object_sysfs_ops,
		.release	= vibrator_debug_object_release,
		.default_attrs = vibrator_node_sys_attrs,
};


static int aw8697_vibrator_init(struct aw8697 *aw8697)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);

#ifdef TIMED_OUTPUT
	aw8697->to_dev.name = "vibrator";
	aw8697->to_dev.get_time = aw8697_vibrator_get_time;
	aw8697->to_dev.enable = aw8697_vibrator_enable;

	ret = timed_output_dev_register(&(aw8697->to_dev));
	if (ret < 0) {
		dev_err(aw8697->dev, "%s: fail to create timed output dev\n",
				__func__);
		return ret;
	}
	ret = sysfs_create_group(&aw8697->to_dev.dev->kobj, &aw8697_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s error creating sysfs attr files\n", __func__);
		return ret;
	}
#else
	aw8697->cdev.name = "vibrator";
	aw8697->cdev.brightness_get = aw8697_haptic_brightness_get;
	aw8697->cdev.brightness_set = aw8697_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw8697->i2c->dev, &aw8697->cdev);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s: fail to create led dev\n",
				__func__);
		return ret;
	}
	ret = sysfs_create_group(&aw8697->cdev.dev->kobj, &aw8697_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s error creating sysfs attr files\n", __func__);
		return ret;
	 }
#endif

	ret = kobject_init_and_add(&aw8697->kobjectDebug, &vibrator_debug_object_type, NULL, "vibrator");
	if (ret) {
		pr_err("%s create vibrator node error!", __func__);
		ret = -1;
		return ret;
	}

	hrtimer_init(&aw8697->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8697->timer.function = aw8697_vibrator_timer_func;
	INIT_WORK(&aw8697->vibrator_work, aw8697_vibrator_work_routine);

	INIT_WORK(&aw8697->init_setting_work, aw8697_init_setting_work_routine);
	INIT_WORK(&aw8697->playRoll_work, aw8697_playRoll_work_routine);

	/* rtp模式要求数据写入及时，不能断流，故建立单独的工作队列，专门处理rtp数据传输，而不使用系统默认的工作队列 */
	rtp_wq = create_singlethread_workqueue("rtp_wq");
	INIT_WORK(&aw8697->rtp_work, aw8697_rtp_work_routine);

	mutex_init(&aw8697->lock);
	mutex_init(&aw8697->bus_lock);
	mutex_init(&aw8697->rtp_check_lock);
	mutex_init(&aw8697->rtp_lock);
	wake_lock_init(&aw8697->wake_lock, WAKE_LOCK_SUSPEND, "vivo-aw8697-wakelock");


	ret = misc_register(&aw8697_haptic_misc);
	if (ret) {
		dev_err(aw8697->dev,  "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}

	/* haptic audio */
	aw8697->haptic_audio.delay_val = 20833; //us
	aw8697->haptic_audio.timer_val = 20833; //us
	aw8697->haptic_audio.haptic_audio_cancel_flag = true;
    aw8697->ff_play_finish = true;

	hrtimer_init(&aw8697->haptic_audio.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8697->haptic_audio.timer.function = aw8697_haptic_audio_timer_func;
	INIT_WORK(&aw8697->haptic_audio.work, aw8697_haptic_audio_work_routine);
	INIT_LIST_HEAD(&aw8697->haptic_audio.ctr_list);
	INIT_LIST_HEAD(&(aw8697->haptic_audio.list));

	spin_lock_init(&aw8697->haptic_audio.list_lock);
	mutex_init(&aw8697->haptic_audio.lock);

	atomic_set(&aw8697->f0_freq_cali, DEFAULT_CALI_F0);
	atomic_set(&aw8697->standard_osc_freq_cali, DEFAULT_OSC_CALI_DATA);

	aw8697->perm_disable = false;

	return 0;
}


/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw8697_interrupt_clear(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
	pr_debug("%s enter: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw8697_interrupt_setup(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

	/* edge int mode */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL,
			AW8697_BIT_DBGCTRL_INT_MODE_MASK, AW8697_BIT_DBGCTRL_INT_MODE_EDGE);

	/* int enable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_BSTERR_MASK, AW8697_BIT_SYSINTM_BSTERR_OFF);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_OV_MASK, AW8697_BIT_SYSINTM_OV_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_OCD_MASK, AW8697_BIT_SYSINTM_OCD_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_OT_MASK, AW8697_BIT_SYSINTM_OT_EN);
}

static irqreturn_t aw8697_irq(int irq, void *data)
{
	struct aw8697 *aw8697 = data;
	unsigned char reg_val = 0;
	unsigned char dbg_val = 0;
	unsigned int buf_len = 0;
	int ret = 0, retval = 0;

	pr_debug("%s enter\n", __func__);

	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	aw8697_i2c_read(aw8697, AW8697_REG_DBGSTAT, &dbg_val);
	pr_info("%s: reg DBGSTAT=0x%x\n", __func__, dbg_val);

	if (reg_val & AW8697_BIT_SYSINT_OVI) {
		pr_err("%s chip ov int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_UVLI) {
		pr_err("%s chip uvlo int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_OCDI) {
		pr_err("%s chip over current int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_OTI) {
		pr_err("%s chip over temperature int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_DONEI) {
		pr_info("%s chip playback done\n", __func__);
	}

	if (reg_val & AW8697_BIT_SYSINT_FF_AEI) {
		pr_debug("%s: aw8697 rtp fifo almost empty int\n", __func__);
		mutex_lock(&aw8697->rtp_lock);
		if (aw8697->rtp_init) {
			while ((!(retval = aw8697_haptic_rtp_get_fifo_afs(aw8697))) &&
					(aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
				pr_info("%s: aw8697 rtp mode fifo update, cnt=%d\n",
						__func__, aw8697->rtp_cnt);
				if ((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr>>2)) {
					buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
				} else {
					buf_len = (aw8697->ram.base_addr>>2);
				}
				ret = aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
						&aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
				if (ret < 0) {
					pr_err("%s error, ret=%d\n", __func__, ret);
					aw8697_haptic_set_rtp_aei(aw8697, false);
					aw8697->rtp_cnt = 0;
					aw8697->rtp_init = 0;
					break;
				}
				aw8697->rtp_cnt += buf_len;
				if (aw8697->rtp_cnt >= aw8697_rtp->len) {
					pr_info("%s: rtp update complete\n", __func__);
					aw8697_haptic_set_rtp_aei(aw8697, false);
					aw8697->rtp_cnt = 0;
					aw8697->rtp_init = 0;
					break;
				}
			}
		} else {
			pr_err("%s: aw8697 rtp init = %d, init error\n", __func__, aw8697->rtp_init);
			aw8697_haptic_set_rtp_aei(aw8697, false);

		}
		mutex_unlock(&aw8697->rtp_lock);
	}

	if (reg_val & AW8697_BIT_SYSINT_FF_AFI) {
		pr_debug("%s: aw8697 rtp mode fifo full empty\n", __func__);
	}

	if ((aw8697->play_mode != AW8697_HAPTIC_RTP_MODE) || (retval < 0)) {
		aw8697_haptic_set_rtp_aei(aw8697, false);
	}

	//aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
	//pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	//aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
	//pr_debug("%s: reg SYSST=0x%x\n", __func__, reg_val);

	pr_debug("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * 根据马达型号配置不同参数，校准使用
 *
 *****************************************************/
static int  aw8697_lra_information_ctr(struct aw8697 *aw8697)
{
    switch(aw8697->lra_information)
    {
        case AW8697_LRA_0832:
            pr_debug("%s enter AW8697_LRA_0832 \n", __func__);
            aw8697->lra_info.AW8697_HAPTIC_F0_PRE = 2350;
            aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN = 7;
            aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL = 125;
            aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL_OV = 155;
            aw8697->lra_info.AW8697_HAPTIC_CONT_TD = 0x006c;
            aw8697->lra_info.AW8697_HAPTIC_CONT_ZC_THR = 0x0ff1;
            aw8697->lra_info.AW8697_HAPTIC_CONT_NUM_BRK = 3;
            break;
        case AW8697_LRA_0815:
            pr_debug("%s enter AW8697_LRA_0815\n", __func__);
            aw8697->lra_info.AW8697_HAPTIC_F0_PRE = 1700;   // 170Hz
            aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN = 7;       // -7%~7%
            aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL = 53;   // 71*6.1/256=1.69v
            aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL_OV = 125;    // 125*6.1/256=2.98v
            aw8697->lra_info.AW8697_HAPTIC_CONT_TD = 0x009a;
            aw8697->lra_info.AW8697_HAPTIC_CONT_ZC_THR = 0x0ff1;
            aw8697->lra_info.AW8697_HAPTIC_CONT_NUM_BRK = 3;
            aw8697->lra_info.AW8697_HAPTIC_RATED_VOLTAGE = 1270; //mv-Vp
            break;
        default:
            pr_debug("%s enter AW8697_LRA_DEFAULT\n", __func__);
            aw8697->lra_info.AW8697_HAPTIC_F0_PRE = 1700;   // 170Hz
            aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN = 7;       // -7%~7%
            aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL = 71;   // 71*6.1/256=1.69v
            aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL_OV = 125;    // 125*6.1/256=2.98v
            aw8697->lra_info.AW8697_HAPTIC_CONT_TD = 0x009a;
            aw8697->lra_info.AW8697_HAPTIC_CONT_ZC_THR = 0x0ff1;
            aw8697->lra_info.AW8697_HAPTIC_CONT_NUM_BRK = 3;
            aw8697->lra_info.AW8697_HAPTIC_RATED_VOLTAGE = 1700; //mv-Vp
            break;
    }
    pr_debug("%d AW8697_HAPTIC_F0_PRE = %d \n", __func__, aw8697->lra_info.AW8697_HAPTIC_F0_PRE);
    pr_debug("%s AW8697_HAPTIC_F0_CALI_PERCEN = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN);
    pr_debug("%s AW8697_HAPTIC_CONT_DRV_LVL = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL);
    pr_debug("%s AW8697_HAPTIC_CONT_DRV_LVL_OV = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL_OV);
    pr_debug("%s AW8697_HAPTIC_CONT_TD = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_TD);
    pr_debug("%s AW8697_HAPTIC_CONT_ZC_THR = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_ZC_THR);
    pr_debug("%s AW8697_HAPTIC_CONT_NUM_BRK = %d;\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_NUM_BRK);
    pr_debug("%s AW8697_HAPTIC_RATED_VOLTAGE = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_RATED_VOLTAGE);
    return 0;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw8697_parse_dt(struct device *dev, struct aw8697 *aw8697,
		struct device_node *np)
{
	struct device_node *effect_np;
	struct device_node *child_node;

	int elment_count;
	struct aw8697_wavefrom_ram_info *ram_info = NULL;
	struct aw8697_wavefrom_rtp_info *rtp_info = NULL;

	//const char *name_str;

	int ret, i, j = 0;

	aw8697->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw8697->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
		//return -1;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw8697->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
	if (aw8697->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
	} else {
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
	}

	if (of_property_read_u32(np, "resistance_min", &aw8697->resistance_min))
		dev_err(dev, "%s: no resistance_min\n", __func__);
	dev_info(dev, "resistance_min:%d\n", aw8697->resistance_min);

	if (of_property_read_u32(np, "resistance_max", &aw8697->resistance_max))
		dev_err(dev, "%s: no resistance_max\n", __func__);
	dev_info(dev, "resistance_max:%d\n", aw8697->resistance_max);

	if (of_property_read_u32(np, "freq_min", &aw8697->freq_min))
		dev_err(dev, "%s: no freq_min\n", __func__);
	dev_info(dev, "freq_min:%d\n", aw8697->freq_min);

	if (of_property_read_u32(np, "freq_max", &aw8697->freq_max))
		dev_err(dev, "%s: no freq_max\n", __func__);
	dev_info(dev, "freq_max:%d\n", aw8697->freq_max);

	if (of_property_read_bool(np, "disable-trigger")) {
		dev_info(dev, "not support trigger\n");
		aw8697->no_trigger = true;
	}
    /* prase new gongmo mode */
    if (of_property_read_u32(np, "is_new_gongmo", &aw8697->is_new_gongmo)) {
		dev_err(dev, "%s: now is_new_gongmo\n", __func__);
        aw8697->is_new_gongmo = 0;
    }
	dev_info(dev, "is_new_gongmo:%d\n", aw8697->is_new_gongmo);

     /* prase lra_info */
    if (of_property_read_u32(np, "lra_info", &aw8697->lra_information)) {
		dev_err(dev, "%s: lra_info\n", __func__);
        aw8697->lra_information = 619;
    }

	aw8697->add_suffix = of_property_read_bool(np, "effect,add-suffix");

    ret = aw8697_lra_information_ctr(aw8697);
    if (ret < 0) {
		pr_err("%s: lra_information_ctr get failed\n", __func__);

	} 
	dev_info(dev, "lra_info:%d\n", aw8697->lra_information);

	/* parse effect */
	effect_np = of_parse_phandle(np, "effect_array", 0);
	if (IS_ERR(effect_np)) {
		pr_err("%s : parse phandle failed\n", __func__);
		aw8697->waveform_ram_table = waveform_ram_table_default;
		aw8697->waveform_rtp_table = waveform_rtp_table_default;
		aw8697->ram_effect_count = 1;
		aw8697->rtp_effect_count = 1;
		return 0;
	}

	elment_count = of_property_count_elems_of_size(effect_np, "ram_effect", sizeof(u32));
	if (elment_count < 4) {
		pr_err("%s: ram effect get failed, elment_count=%d\n", __func__, elment_count);
		aw8697->waveform_ram_table = waveform_ram_table_default;
		aw8697->ram_effect_count = 1;
	} else {
	
		pr_debug("%s : element size is %u\n", __func__, elment_count);

		ram_info = devm_kcalloc(dev, elment_count, sizeof(u32), GFP_KERNEL);
		if (!ram_info) {
			pr_err("%s : kzalloc failed\n", __func__);
			aw8697->waveform_ram_table = waveform_ram_table_default;
			aw8697->ram_effect_count = 1;
		} else {

			ret = of_property_read_u32_array(effect_np, "ram_effect", (u32 *)ram_info, elment_count);
			if (ret < 0) {
				pr_err("%s : read u32 array failed, ret=%d\n", __func__, ret);
				aw8697->waveform_ram_table = waveform_ram_table_default;
				aw8697->ram_effect_count = 1;
			} else {

				aw8697->ram_effect_count = elment_count / 4; //每个效果对应4个参数
				aw8697->waveform_ram_table = ram_info;
			}
		}

	}

	for (i = 0; i < aw8697->ram_effect_count; i++) {
	
		pr_debug("%s: idx=%u, ram_id=%u, vmax=%u, times_us=%u\n", __func__, aw8697->waveform_ram_table[i].idx, aw8697->waveform_ram_table[i].ram_id, aw8697->waveform_ram_table[i].vmax, aw8697->waveform_ram_table[i].times_us);
	
	}


	elment_count = of_get_available_child_count(effect_np);
	if (elment_count == 0) {
		pr_err("%s: no rtp effect, elment_count=%d\n", __func__, elment_count);
		aw8697->waveform_rtp_table = waveform_rtp_table_default;
		aw8697->rtp_effect_count = 1;
	} else {
		pr_info("%s: rtp effect, elment_count=%d\n", __func__, elment_count);
		aw8697->waveform_rtp_table = devm_kcalloc(dev, elment_count, sizeof(struct aw8697_wavefrom_rtp_info), GFP_KERNEL);
		if (!aw8697->waveform_rtp_table) {
			pr_err("%s : rtp kcalloc failed, nomem:-12\n", __func__);
			aw8697->waveform_rtp_table = waveform_rtp_table_default;
			aw8697->rtp_effect_count = 1;
		} else {
			for_each_available_child_of_node(effect_np, child_node) {
				rtp_info = &aw8697->waveform_rtp_table[j++];
				ret = of_property_read_u32(child_node, "rtp_index", &rtp_info->idx);
				if (ret < 0) {
					pr_err("%s : rtp read u32 failed, ret=%d\n", __func__, ret);
				}
			#if 0
				ret = of_property_read_string(child_node, "rtp_name", &name_str);
				if (ret < 0) {
					pr_err("%s : rtp read string failed, ret=%d\n", __func__, ret);
				}

				strlcpy(rtp_info->rtp_name, name_str, sizeof(rtp_info->rtp_name));
			#else
				ret = of_property_read_string(child_node, "rtp_name", &rtp_info->rtp_name);
				if (ret < 0) {
					pr_err("%s : rtp read string failed, ret=%d\n", __func__, ret);
					rtp_info->rtp_name = "invalid file";
				}

			#endif
				ret = of_property_read_u32(child_node, "rtp_vmax", &rtp_info->vmax);
				if (ret < 0) {
					pr_err("%s : rtp read u32 failed, ret=%d\n", __func__, ret);
				}

				ret = of_property_read_u32(child_node, "rtp_len", &rtp_info->times_ms);
				if (ret < 0) {
					pr_err("%s : rtp read u32 failed, ret=%d\n", __func__, ret);
				}

			}

			aw8697->rtp_effect_count = elment_count;
		}
	}

	for (j = 0; j < aw8697->rtp_effect_count; j++) {
		pr_debug("%s: idx=%u, rtp_name=%s, vmax=%u, times_ms=%u\n", __func__, aw8697->waveform_rtp_table[j].idx, aw8697->waveform_rtp_table[j].rtp_name, aw8697->waveform_rtp_table[j].vmax, aw8697->waveform_rtp_table[j].times_ms);
	}

	return 0;
}

static int aw8697_hw_reset(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	if (aw8697 && gpio_is_valid(aw8697->reset_gpio)) {
		gpio_set_value_cansleep(aw8697->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw8697->reset_gpio, 1);
		msleep(2);
	} else {
		dev_err(aw8697->dev, "%s:  failed\n", __func__);
	}
	return 0;
}


/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw8697_read_chipid(struct aw8697 *aw8697)
{
	int ret = -1;
	unsigned char reg = 0;

	/* hardware reset */
	aw8697_hw_reset(aw8697);

	ret = aw8697_i2c_read(aw8697, AW8697_REG_ID, &reg);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s: failed to read register AW8697_REG_ID: %d\n", __func__, ret);
	}
	switch (reg) {
	case AW8697_CHIPID:
		pr_info("%s aw8697 detected\n", __func__);
		aw8697->chipid = AW8697_CHIPID;
		//aw8697->flags |= AW8697_FLAG_SKIP_INTERRUPTS;
		aw8697_haptic_softreset(aw8697);
		return 0;
	default:
		pr_info("%s unsupported device revision (0x%x)\n", __func__, reg);
		break;
	}

	return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw8697_i2c_reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8697_i2c_write(aw8697, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8697_i2c_reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for (i = 0; i < AW8697_REG_MAX; i++) {
		if (!(aw8697_reg_access[i]&REG_RD_ACCESS))
		   continue;
		aw8697_i2c_read(aw8697, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}
static ssize_t aw8697_i2c_ram_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);

	unsigned int databuf[1] = {0};

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (1 == databuf[0]) {
			aw8697_ram_update(aw8697);
		}
	}

	return count;
}

static ssize_t aw8697_i2c_ram_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	aw8697_haptic_stop(aw8697);
	/* RAMINIT Enable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);

	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, (unsigned char)(aw8697->ram.base_addr>>8));
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, (unsigned char)(aw8697->ram.base_addr&0x00ff));
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8697_haptic_ram:\n");
	for (i = 0; i < aw8697->ram.len; i++) {
		aw8697_i2c_read(aw8697, AW8697_REG_RAMDATA, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02x,", reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	/* RAMINIT Disable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8697_i2c_reg_show, aw8697_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8697_i2c_ram_show, aw8697_i2c_ram_store);

static struct attribute *aw8697_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};

static struct attribute_group aw8697_attribute_group = {
	.attrs = aw8697_attributes
};

#if 1
static int aw8697_ff_msg_send(struct aw8697 *aw8697, enum aw8697_ff_msg_type type, int arg1, struct aw8697_play_info *arg2)
{
	struct aw8697_ff_msg *msg;
	pr_debug("%s enter, action_type=%d, arg1=%d\n", __func__, type, arg1);
	if (type == AW8697_FF_MSG_UPLOAD_EFFECT)
		pr_debug("%s type=%d ram_id=%d vamx=%d\n", __func__, arg2->type, arg2->ram_id, arg2->vmax);

	msg = kmem_cache_zalloc(aw8697->ff_km, GFP_ATOMIC);
	if (!msg) {
		pr_err("[aw8697]alloc message buffer failed");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&msg->list);
	msg->type = type;
	msg->arg1 = arg1;
	msg->arg2 = arg2;

	spin_lock(&aw8697->ff_lock);
	list_add_tail(&msg->list, &aw8697->ff_messages);
	spin_unlock(&aw8697->ff_lock);

	up(&aw8697->ff_sem);

	return 0;
}
#endif

static int aw8697_ff_msg_recv(struct aw8697 *aw8697, enum aw8697_ff_msg_type *type, int *arg1, struct aw8697_play_info **arg2)
{
	struct aw8697_ff_msg *msg = NULL;
	int ret;

	ret = down_interruptible(&aw8697->ff_sem);

	spin_lock(&aw8697->ff_lock);
	if (list_empty(&aw8697->ff_messages)) {
		spin_unlock(&aw8697->ff_lock);
		return -EAGAIN;
	}

	msg = list_first_entry(&aw8697->ff_messages, struct aw8697_ff_msg, list);
	if (type == AW8697_FF_MSG_UPLOAD_EFFECT)
		pr_err("%s enter, type=%d ram_id=%d vamx=%d\n", __func__, msg->arg2->type, msg->arg2->ram_id, msg->arg2->vmax);
	*type = msg->type;
	*arg1 = msg->arg1;
	*arg2 = msg->arg2;
	list_del(&msg->list);
	spin_unlock(&aw8697->ff_lock);
	kmem_cache_free(aw8697->ff_km, msg);
	//pr_debug("%s enter, action_type=%d, arg1=%d\n", msg->type, msg->arg1);
	return 0;
}

static int find_ram_idx_from_table(struct aw8697 *aw8697, s16 idx)
{
	int i;
	struct aw8697_wavefrom_ram_info *waveform_ram_table = aw8697->waveform_ram_table;
	struct aw8697_wavefrom_rtp_info *waveform_rtp_table = aw8697->waveform_rtp_table;

	if (idx < AW8697_RAM_MODE_ID_MAX) {
		for (i = 0; i < aw8697->ram_effect_count; i++) {

			if (waveform_ram_table[i].idx == idx) {
				return i;
			} else
				continue;
		}
	} else {
		for (i = 0; i < aw8697->rtp_effect_count; i++) {

			if (waveform_rtp_table[i].idx == idx)
				return i;
			else
				continue;
		}
	}

	return -EINVAL;
}


static int aw8697_upload_effect(struct input_dev *dev,
		struct ff_effect *effect, struct ff_effect *old)
{


	struct aw8697 *aw8697 = (struct aw8697 *)input_get_drvdata(dev);
	struct aw8697_wavefrom_ram_info *waveform_ram_table = aw8697->waveform_ram_table;
	struct aw8697_wavefrom_rtp_info *waveform_rtp_table = aw8697->waveform_rtp_table;	
	s16 level, data[CUSTOM_DATA_LEN];
	int id, table_id;
	struct aw8697_play_info *play;
	int tmp, ret;

	if ((!!at_test) || (aw8697->perm_disable)) {
		pr_err("%s now is at test, at_test = %d , aw8697->perm_disable = %d\n", __func__, at_test, aw8697->perm_disable);
		return 0;
	}

	play = kzalloc(sizeof(struct aw8697_play_info), GFP_KERNEL);
	if (play == NULL) {
		pr_err("%s kzalloc failed\n", __func__);
		return -ENOMEM;
	}


	switch (effect->type) {

	case FF_CONSTANT:
		/* 使用ram_loop模式，不开升压；采用正弦波循环*/
		play->playLength = effect->replay.length * USEC_PER_MSEC;
		play->type = TIME_TYPE;
		level = effect->u.constant.level;
		tmp = level * aw8697->lra_info.AW8697_HAPTIC_RATED_VOLTAGE;
		play->vmax = tmp / 0x7fff;
		pr_err("%s type=%d ram_id=%d, vmax=%d, times_us=%d, play_length=%d, level=%d\n",
			__func__, play->type, play->ram_id, play->vmax, play->times_us, play->playLength, level);

		ret = aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_UPLOAD_EFFECT, 0, play);
		if (ret < 0) {
			pr_err("%s ff_msg_send failed, ret=%d\n", __func__, ret);
			kfree(play);
			return -ENOMEM;
		}
		break;

	case FF_PERIODIC:
		/* 使用ram模式，开升压*/

		if (effect->u.periodic.waveform != FF_CUSTOM) {
			pr_err("%s ff-device Only accept custom waveforms\n", __func__);
			kfree(play);
			return -EINVAL;
		}

		if (copy_from_user(data, effect->u.periodic.custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			pr_err("%s copy from user failed\n", __func__);
			kfree(play);
			return -EFAULT;
		}
		id = data[CUSTOM_DATA_EFFECT_IDX];
		/* 在列表中查询实际波形编号 */
		table_id = find_ram_idx_from_table(aw8697, id);
		if (table_id < 0) {
			pr_err("%s invalid id\n", __func__);
			kfree(play);
			return -EINVAL;
		}

		if (id < AW8697_RAM_MODE_ID_MAX) {
			level = effect->u.periodic.magnitude;
			tmp = waveform_ram_table[table_id].vmax;
			play->vmax = level * tmp / 0x7fff;
			play->times_us = waveform_ram_table[table_id].times_us;
			play->type = RAM_TYPE;
			play->ram_id = waveform_ram_table[table_id].ram_id;

		} else {
			play->type = RTP_TYPE;
			level = effect->u.periodic.magnitude;
			tmp = waveform_rtp_table[table_id].vmax;
			play->vmax = level * tmp / 0x7fff;
			play->times_us = waveform_rtp_table[table_id].times_ms * 1000;
			if (table_id < aw8697->rtp_effect_count)
				aw8697->rtp_file_num = table_id;
			else
				pr_err("%s rtp_file_num 0x%02x over max value \n", __func__, aw8697->rtp_file_num);
		}

		data[CUSTOM_DATA_TIMEOUT_SEC_IDX] = play->times_us / USEC_PER_SEC;
		data[CUSTOM_DATA_TIMEOUT_MSEC_IDX] = (play->times_us % USEC_PER_SEC) / USEC_PER_MSEC + 15;

		pr_err("%s type=%d id=%d, ram_id=%d, vmax=%d, play_vmax=%d, times_us=%d, play_length=%d, level=%d\n",
			__func__, play->type, id, play->ram_id, tmp, play->vmax, play->times_us, play->playLength, level);

		if (copy_to_user(effect->u.periodic.custom_data, data, sizeof(s16) * CUSTOM_DATA_LEN))	{
			pr_err("%s copy to user failed\n", __func__);
			kfree(play);
			return -EFAULT;
		}

		pr_err("%s play point %p\n", __func__, play);
		ret = aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_UPLOAD_EFFECT, 0, play);
		if (ret < 0) {
			pr_err("%s ff_msg_send failed, ret=%d\n", __func__, ret);
			kfree(play);
			return -ENOMEM;
		}
		break;
	default:
		pr_err("%s Unsupported effect type: %d\n", __func__, effect->type);
		kfree(play);
		return -EINVAL;
	}

	return 0;
}


static int aw8697_playback(struct input_dev *dev, int effect_id, int val)
{

	struct aw8697 *aw8697 = (struct aw8697 *)input_get_drvdata(dev);
	int ret = 0;

	pr_info("%s effect_id = %d, val=%d\n", __func__, effect_id, val);

	if ((!!at_test) || (aw8697->perm_disable)) {
        pr_err("%s now is at test, at_test = %d , aw8697->perm_disable = %d\n", __func__, at_test, aw8697->perm_disable);
		return 0;
	}

	/* 将效果调用放入FIFO*/
	ret = aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_PLAYBACK, val, NULL);
	if (ret) {
		pr_err("%s: ff msg send failed ,ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int aw8697_erase(struct input_dev *dev, int effect_id)
{
	struct aw8697 *aw8697 = (struct aw8697 *)input_get_drvdata(dev);
	pr_info("%s erase enter\n", __func__);
	if ((!!at_test) || (aw8697->perm_disable)) {
        pr_err("%s now is at test, at_test = %d , aw8697->perm_disable = %d\n", __func__, at_test, aw8697->perm_disable);
		return 0;
	}
	return 0;
}

static void aw8697_gain(struct input_dev *dev, u16 gain)
{
	struct aw8697 *aw8697 = (struct aw8697 *)input_get_drvdata(dev);

	pr_debug("%s gain enter, gain=%d\n", __func__, gain);
	if ((!!at_test) || (aw8697->perm_disable)) {
        pr_err("%s now is at test, at_test = %d , aw8697->perm_disable = %d\n", __func__, at_test, aw8697->perm_disable);
		return;
	}

/* 将效果调用放入FIFO*/
	aw8697_ff_msg_send(aw8697, AW8697_FF_MSG_GAIN, gain, NULL);
}

static aw8697_set_play_vol(struct aw8697 *aw8697)
{
	int ret = 0;
	u8 vmax_val;
	u8 gain_val;

	if (aw8697->play.vmax < 6000) {
		vmax_val = 0x00;
		gain_val = aw8697->play.vmax * 128 / 6000;
	} else {
		vmax_val = (aw8697->play.vmax - 6000) / 142;
		vmax_val = vmax_val + 5; //添加0.7v的线损补偿
		gain_val = 0x80;
	}
	pr_debug("aw8697 vmax_val=%d, gain_val=%d\n", vmax_val, gain_val);

	aw8697_haptic_set_bst_vol(aw8697, vmax_val);
	aw8697_haptic_set_gain(aw8697, gain_val);

	return ret;

}

static int aw8697_ff_thread(void *dev)
{
	struct aw8697 *aw8697 = (struct aw8697 *)dev;
	enum aw8697_ff_msg_type type;
	struct aw8697_play_info *play = &aw8697->play;
	int arg1;
	struct aw8697_play_info *up_play = NULL;//arg2
	int ret;
	int val;
	//unsigned long delta_ms = 0;

	struct aw8697_wavefrom_rtp_info *waveform_rtp_table = aw8697->waveform_rtp_table;


	static struct sched_param para = {
		.sched_priority = 40,
	};

	sched_setscheduler(current, SCHED_FIFO, &para);

	while (!kthread_should_stop()) {
		ret = aw8697_ff_msg_recv(aw8697, &type, &arg1, &up_play);
		if (ret)
			continue;
		pr_debug("%s enter, action_type=%d, arg1=%d, arg2=%p\n", __func__, type, arg1, up_play);

		mutex_lock(&aw8697->haptic_audio.lock);

		if ((!aw8697->haptic_audio.haptic_audio_cancel_flag) && (type == AW8697_FF_MSG_UPLOAD_EFFECT)) {
			aw8697_haptic_audio_cancel(aw8697);
			//aw8697_haptic_stop(aw8697);

		}
		aw8697->ff_play_finish = false;
		mutex_unlock(&aw8697->haptic_audio.lock);


		mutex_lock(&aw8697->lock);

		if (type == AW8697_FF_MSG_PLAYBACK) {

			if (!!arg1) {

				/* 开始播放 */
				pr_info("%s type=%d, vmax=%d, ram_id=%d", __func__, aw8697->play.type, aw8697->play.vmax, aw8697->play.ram_id);

				if (play->type == RAM_TYPE) {

					/* RAM 模式振动 */
					aw8697_haptic_stop(aw8697);
					aw8697_haptic_set_wav_loop(aw8697, 0x00, 0x00);
					aw8697_haptic_set_wav_seq(aw8697, 0x00, play->ram_id);
					aw8697_set_play_vol(aw8697);
					if (aw8697->play.ram_id == 0) {
						aw8697_double_click_switch(aw8697, true);
					} else if (aw8697->play.ram_id == 88) {
						aw8697_quadra_click_switch(aw8697, true);
					}
					aw8697_haptic_play_wav_seq(aw8697, true);

				} else if (play->type == RTP_TYPE) {

					/* RTP模式振动*/
					if (aw8697->rtp_file_num > 0) {
						if ((waveform_rtp_table[aw8697->rtp_file_num].idx < 500) &&
									(waveform_rtp_table[aw8697->rtp_file_num].idx > 400)) {
							pr_err("%s now is notify\n", __func__);
							if (!wake_lock_active(&aw8697->wake_lock)) {
								pr_err("%s wake lock\n", __func__);
								wake_lock(&aw8697->wake_lock);
							}
						}

						aw8697_haptic_stop(aw8697);
						aw8697_haptic_set_rtp_aei(aw8697, false);
						aw8697_haptic_set_wav_loop(aw8697, 0x00, 0x00);
						aw8697_set_play_vol(aw8697);
						aw8697_interrupt_clear(aw8697);

						queue_work(rtp_wq, &aw8697->rtp_work);
						rtp_check_flag = true;

					}

				} else if (play->type == TIME_TYPE) {

					/* 按时间长振 */
					aw8697_haptic_stop(aw8697);
					aw8697_haptic_set_wav_loop(aw8697, 0x00, 0x0f);
					aw8697->seq[0] = PLAYBACK_INFINITELY_RAM_ID;
					aw8697_haptic_set_wav_seq(aw8697, 0x00, PLAYBACK_INFINITELY_RAM_ID);
					//aw8697_haptic_set_repeat_wav_seq(aw8697, PLAYBACK_INFINITELY_RAM_ID);
					//aw8697_haptic_set_gain(aw8697, play->vmax * 128 / AW8697_VBAT_REFER);

					aw8697->activate_mode = AW8697_HAPTIC_ACTIVATE_RAM_MODE;
					aw8697->state = 1;//表示启动播放状态
					aw8697->gain = 0x80; //波形已经适配成1.7Vp

					if (aw8697->state) {
						/* clip value to max */
						val = play->playLength;
						pr_info("%s play times is %d\n", __func__, val);


						if (aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_RAM_MODE) {
							aw8697_haptic_ram_vbat_comp(aw8697, false);
							aw8697_haptic_play_repeat_seq(aw8697, true);
						} else if (aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_CONT_MODE) {
							aw8697_haptic_cont(aw8697);
						} else {
						}
						/* run us timer */
						hrtimer_start(&aw8697->timer,
								  ktime_set(val / USEC_PER_SEC, (val % USEC_PER_SEC) * NSEC_PER_USEC),
								  HRTIMER_MODE_REL);


					}

				}

			} else {
			/*
				do_gettimeofday(&aw8697->cancel);
				delta_ms = (aw8697->cancel.tv_sec - aw8697->begin.tv_sec) * 1000 + (aw8697->cancel.tv_usec - aw8697->begin.tv_usec) / 1000;
				if (delta_ms < 10) {
					mdelay(10);
				}
				pr_info("%s --->cancel delay, delta_ms=%u\n", __func__, delta_ms);
			*/
				//清除timer
				if (play->type != TIME_TYPE) {
					if (hrtimer_active(&aw8697->timer)) {
						hrtimer_cancel(&aw8697->timer);
						pr_err("%s playback cancel timer\n", __func__);
					}
				}
				aw8697_rtp->len = 0;
				/* 取消rtp work */
				if (cancel_work_sync(&aw8697->rtp_work)) {
					pr_debug("%s palyback pending work cancle success\n", __func__);
				}
				mutex_lock(&aw8697->rtp_check_lock);
				aw8697_haptic_stop(aw8697);
				rtp_check_flag = false;
				mutex_unlock(&aw8697->rtp_check_lock);

				if ((play->type == RTP_TYPE) || (play->type == RAM_TYPE)) {
					aw8697_vol_trig_switch(aw8697, true);
				}
				aw8697_set_clock(aw8697, AW8697_HAPTIC_CLOCK_CALI_F0);

				if (wake_lock_active(&aw8697->wake_lock)) {
					pr_err("%s unlock wake lock\n", __func__);
					wake_unlock(&aw8697->wake_lock);
				}

			}

		} else if (type == AW8697_FF_MSG_GAIN) {

			if (play->type == TIME_TYPE) {
				pr_info("%s actul gain= ox%x, vol is %d\n", __func__, (arg1 * 0x80) / 0x7fff, (arg1 * aw8697->lra_info.AW8697_HAPTIC_RATED_VOLTAGE) / 0x7fff);
				if (arg1 > 0x7fff)
					arg1 = 0x7fff;
				aw8697->gain = ((u32)(arg1 * 0x80)) / 0x7fff;
				aw8697_haptic_ram_vbat_comp(aw8697, true);
			}
			if (play->type == RTP_TYPE && waveform_rtp_table[aw8697->rtp_file_num].idx < 400 &&
				(waveform_rtp_table[aw8697->rtp_file_num].idx > 300)) {
				pr_info("%s actul gain= ox%x, vol is %d\n", __func__, (arg1 * 0x80) / 0x7fff, (arg1 * waveform_rtp_table[aw8697->rtp_file_num].vmax) / 0x7fff);
				if (arg1 > 0x7fff)
					arg1 = 0x7fff;
				aw8697->gain = ((u32)(arg1 * 0x80)) / 0x7fff;
				aw8697_haptic_set_gain(aw8697, aw8697->gain);
			}

		} else if (type == AW8697_FF_MSG_UPLOAD_EFFECT) {

			if (up_play) {
				pr_debug("%s upload deal, point=%p, type=%d, ramid=%d, vmax=%d\n", __func__, up_play, up_play->type, up_play->ram_id, up_play->vmax);
				memcpy(play, up_play, sizeof(struct aw8697_play_info));
				kfree(up_play);

				/* 清除timer*/
				if (hrtimer_active(&aw8697->timer)) {
					hrtimer_cancel(&aw8697->timer);
					pr_err("%s upload cancel timer\n", __func__);
				}

				if (cancel_work_sync(&aw8697->rtp_work)) {
					pr_err("%s upload pending work cancle success\n", __func__);
				}

			}

		}
		mutex_unlock(&aw8697->lock);

		mutex_lock(&aw8697->haptic_audio.lock);
		if ((type == AW8697_FF_MSG_PLAYBACK) && (!arg1)) {
			pr_debug("%s: haptic audio ,ff play finish\n", __func__);
			aw8697->ff_play_finish = true;

		}
		mutex_unlock(&aw8697->haptic_audio.lock);

	}

	return 0;
}

static int aw8697_ff_device_init(struct aw8697 *aw8697)
{
	struct ff_device *ff;
	int ret;

	aw8697->ff_km = kmem_cache_create("aw8697_ff", sizeof(struct aw8697_ff_msg), 0, 0, NULL);
	if (!aw8697->ff_km) {
		pr_err("aw8697 create mem cache failed!\n");
		return -ENOMEM;
	}

	aw8697->input = input_allocate_device();
	if (!aw8697->input) {
		pr_err("[aw8697] alloc input dev error!\n");
		kmem_cache_destroy(aw8697->ff_km);
		return -ENOMEM;
	}

	aw8697->input->name = "aw8697";
	input_set_capability(aw8697->input, EV_FF, FF_CONSTANT);
	input_set_capability(aw8697->input, EV_FF, FF_GAIN);
	input_set_capability(aw8697->input, EV_FF, FF_PERIODIC);
	input_set_capability(aw8697->input, EV_FF, FF_CUSTOM);
	input_set_drvdata(aw8697->input, aw8697);

	ret = input_ff_create(aw8697->input, FF_MAX_EFFECTS);
	if (ret) {
		pr_err("[aw8697] create ff device failed!, ret = %d\n", ret);
		input_free_device(aw8697->input);
		kmem_cache_destroy(aw8697->ff_km);
		return ret;
	}

	ff = aw8697->input->ff;
	ff->upload = aw8697_upload_effect;
	ff->playback = aw8697_playback;
	ff->erase = aw8697_erase;
	ff->set_gain = aw8697_gain;
	spin_lock_init(&aw8697->ff_lock);
	sema_init(&aw8697->ff_sem, 0);
	INIT_LIST_HEAD(&aw8697->ff_messages);

	ret = input_register_device(aw8697->input);
	if (ret) {
		pr_err("aw8697 register input device failed, ret=%d\n", ret);
		input_ff_destroy(aw8697->input);
		input_free_device(aw8697->input);
		kmem_cache_destroy(aw8697->ff_km);
		return ret;
	}

	aw8697->ff_task = kthread_run(aw8697_ff_thread, aw8697, "aw8697_ff");
	if (IS_ERR_OR_NULL(aw8697->ff_task)) {
		input_ff_destroy(aw8697->input);
		input_unregister_device(aw8697->input);
		kmem_cache_destroy(aw8697->ff_km);
		return PTR_ERR(aw8697->ff_task);
	}

	return 0 ;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw8697_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw8697 *aw8697;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw8697 = devm_kzalloc(&i2c->dev, sizeof(struct aw8697), GFP_KERNEL);
	if (aw8697 == NULL)
		return -ENOMEM;

	aw8697->dev = &i2c->dev;
	aw8697->i2c = i2c;

	i2c_set_clientdata(i2c, aw8697);

	/* aw8697 rst & int */
	if (np) {
		ret = aw8697_parse_dt(&i2c->dev, aw8697, np);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
			return -ENODEV;
		}
	} else {
		aw8697->reset_gpio = -1;
		aw8697->irq_gpio = -1;
	}

	if (gpio_is_valid(aw8697->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8697->reset_gpio,
			GPIOF_OUT_INIT_HIGH, "aw8697_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
			return -ENODEV;
		}
	}

	if (gpio_is_valid(aw8697->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8697->irq_gpio,
			GPIOF_DIR_IN, "aw8697_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n", __func__);
			return -ENODEV;
		}
	}

	/* aw8697 chip id */
	ret = aw8697_read_chipid(aw8697);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw8697_read_chipid failed ret=%d\n", __func__, ret);
		return -ENODEV;
	}


	/* aw8697 irq */
	if (gpio_is_valid(aw8697->irq_gpio) &&
		!(aw8697->flags & AW8697_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
					gpio_to_irq(aw8697->irq_gpio),
					NULL, aw8697_irq, irq_flags,
					"aw8697", aw8697);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
					__func__, gpio_to_irq(aw8697->irq_gpio), ret);
			return -ENODEV;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw8697->flags |= AW8697_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw8697);

	ret = sysfs_create_group(&i2c->dev.kobj, &aw8697_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n", __func__);
		return -EIO;
	}

	g_aw8697 = aw8697;

	ret = aw8697_vibrator_init(aw8697);
	if (ret) {
		pr_err("%s: vibrator init failed, ret=%d\n", __func__, ret);
		goto err_vibrator_init;
	}

	aw8697_haptic_init(aw8697);

	aw8697_ram_init(aw8697);

	ret = aw8697_ff_device_init(aw8697);
	if (ret) {
		pr_err("%s: ff device init failed,ret=%d\n", __func__, ret);
		goto err_ff_init;
	}

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;
err_ff_init:
	misc_deregister(&aw8697_haptic_misc);
	
	sysfs_remove_group(&i2c->dev.kobj, &aw8697_vibrator_attribute_group);
	#ifdef TIMED_OUTPUT
	timed_output_dev_unregister(&(aw8697->to_dev));
	#else
	devm_led_classdev_unregister(&aw8697->i2c->dev, &aw8697->cdev);
	#endif
	wake_lock_destroy(&aw8697->wake_lock);
err_vibrator_init:
	sysfs_remove_group(&i2c->dev.kobj, &aw8697_attribute_group);

	return 0;
}

static int aw8697_i2c_remove(struct i2c_client *i2c)
{
	struct aw8697 *aw8697 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	kthread_stop(aw8697->ff_task);

	input_ff_destroy(aw8697->input);

	input_unregister_device(aw8697->input);

	kmem_cache_destroy(aw8697->ff_km);
	aw8697->ff_km = NULL;

	misc_deregister(&aw8697_haptic_misc);

	sysfs_remove_group(&i2c->dev.kobj, &aw8697_vibrator_attribute_group);
#ifdef TIMED_OUTPUT
	timed_output_dev_unregister(&(aw8697->to_dev));
#else
	devm_led_classdev_unregister(&aw8697->i2c->dev, &aw8697->cdev);
#endif

	wake_lock_destroy(&aw8697->wake_lock);

	sysfs_remove_group(&i2c->dev.kobj, &aw8697_attribute_group);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw8697->irq_gpio), aw8697);

	if (gpio_is_valid(aw8697->irq_gpio))
		devm_gpio_free(&i2c->dev, aw8697->irq_gpio);
	if (gpio_is_valid(aw8697->reset_gpio))
		devm_gpio_free(&i2c->dev, aw8697->reset_gpio);

	devm_kfree(&i2c->dev, aw8697);
	devm_kfree(&i2c->dev, aw8697_rtp);
	aw8697_rtp = NULL;
	aw8697 = NULL;

	kmem_cache_destroy(rtp_cachep);
	rtp_cachep = NULL;

	destroy_workqueue(rtp_wq);

	return 0;
}

static void aw8697_pm_shutdown(struct i2c_client *i2c)
{
	struct aw8697 *aw8697 = i2c_get_clientdata(i2c);

	aw8697->trig[1].enable = 0;
	aw8697->trig[2].enable = 0;
	pr_info("%s enter\n", __func__);
	aw8697_haptic_trig_enable_config(aw8697);
}

static int aw8697_pm_suspend (struct device *dev)
{
	i2c_suspend = 1;
	pr_info("%s enter, pm_suspend_flag = %d\n", __func__, i2c_suspend);

	return 0;

}


static int aw8697_pm_resume (struct device *dev)
{

	i2c_suspend = 0;
	pr_info("%s enter, pm_suspend_flag = %d\n", __func__, i2c_suspend);
	return 0;


}

struct dev_pm_ops aw8697_pm_ops = {
	.suspend = aw8697_pm_suspend,
	.resume = aw8697_pm_resume,
};


static const struct i2c_device_id aw8697_i2c_id[] = {
	{ AW8697_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw8697_i2c_id);

static struct of_device_id aw8697_dt_match[] = {
	{ .compatible = "awinic,aw8697_haptic" },
	{ },
};

static struct i2c_driver aw8697_i2c_driver = {
	.driver = {
		.name = AW8697_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw8697_dt_match),
		.pm = &aw8697_pm_ops,
	},
	.shutdown = aw8697_pm_shutdown,
	.probe = aw8697_i2c_probe,
	.remove = aw8697_i2c_remove,
	.id_table = aw8697_i2c_id,
};
// vivo zhangxiaodong modify to remove atboot test begin
// 产线在SMT阶段，增加iic通路at测试，故移除at模式下不加载马达驱动的限制
//extern unsigned int is_atboot;
static int __init aw8697_i2c_init(void)
{
	int ret = 0;

	pr_info("aw8697 driver version %s\n", AW8697_VERSION);
/*
	if (is_atboot == 1) {
		pr_err("%s now is at mode, not load lra vibrator driver\n", __func__);
		return 0;
	} else {
		pr_err("%s driver load normal\n", __func__);
	}
*/
	ret = i2c_add_driver(&aw8697_i2c_driver);
	if (ret) {
		pr_err("aw8697 fail to add device into i2c\n");
		return ret;
	}

	return 0;
}
//late_initcall(aw8697_i2c_init);
module_init(aw8697_i2c_init);


static void __exit aw8697_i2c_exit(void)
{
	i2c_del_driver(&aw8697_i2c_driver);
}
module_exit(aw8697_i2c_exit);


MODULE_DESCRIPTION("AW8697 Haptic Driver");
MODULE_LICENSE("GPL v2");
