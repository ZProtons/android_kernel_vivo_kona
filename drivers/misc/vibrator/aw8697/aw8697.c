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
#define pr_fmt(fmt) "awinic_haptics: " fmt
#define dev_fmt(fmt) "awinic_haptics: " fmt

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

#include <linux/pm_qos.h>
#include "aw8697_reg.h"
#include "aw8697.h"

#include <linux/wakelock.h>
#include "../vivo_haptic_core.h"


#undef dev_dbg
#define dev_dbg dev_err

#undef pr_debug
#define pr_debug pr_err

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW8697_I2C_NAME "aw8697_haptic"
#define AW8697_HAPTIC_NAME "awinic_haptic"  //miscdev name

#define AW8697_VERSION "v1.3.6"

#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 3
#define AW_I2C_RETRY_DELAY 2

#define PLAYBACK_INFINITELY_RAM_ID 6
#define OP_OCS_CALIBRATION_T_LENGTH 5000000

/****************************************************************************
 *
 *                  custom macro
 *
 ****************************************************************************/
#define RTP_BIN_MAX_SIZE 2000000
#define RTP_SLAB_SIZE 512

#define DUMP_DTS_CONFIG

#define BASE_SCENE_COUNT_MAX 300

/****************************************************************************
 *
 *                 static variable
 *
 ****************************************************************************/

static struct pm_qos_request pm_qos_req_vb;

static struct kmem_cache *rtp_cachep;

static struct workqueue_struct *rtp_wq;

//防止rtp调用返回上层，刚执行work时候
//中间来了一个cancel导致cancel没有效果，rtp仍然会继续播完
static bool rtp_check_flag;
static bool at_test;
static volatile int i2c_suspend;

/******************************************************
 *
 * variable
 *
 ******************************************************/


//#define AW8697_RTP_NAME_MAX        64
static char *aw8697_ram_name = "aw8697_haptic.bin";
static char *aw8697_osc_cali_file = "aw8697_osc_cali.bin";
/*
static char aw8697_rtp_name[][AW8697_RTP_NAME_MAX] = {
	{"aw8697_rtp.bin"},
	{"aw8697_rtp_lighthouse.bin"},
	{"aw8697_rtp_silk.bin"},
};
*/

static struct aw8697_container *aw8697_rtp;
static struct aw8697 *g_aw8697;

//static char osc_cali_result[128];
//static int osc_cali_result_mode;

/* Used for trigger hardware path detection, other vibrations must be shielded.
 * 1 means shielded, 0 means unshielded */
//static int at_test; //用于trigger硬件通路检测时候，屏蔽其它振动；1表示屏蔽，0表示不屏蔽

static struct aw8697_wavefrom_info waveform_list_default[1] = {

	{1, 1, 8000, 12000, false, "default"},
};

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw8697_interrupt_clear(struct aw8697 *aw8697);
static int aw8697_haptic_trig_enable_config(struct aw8697 *aw8697);
static int aw8697_haptic_set_gain(struct aw8697 *aw8697, unsigned char gain);
static int aw8697_haptic_set_bst_vol(struct aw8697 *aw8697, unsigned char bst_vol);
static void aw8697_interrupt_setup(struct aw8697 *aw8697);
static int aw8697_set_clock(struct aw8697 *aw8697, int clock_type);
static void aw8697_double_click_switch(struct aw8697 *aw8697, bool sw);


 /***********************************************************************************************
 *
 * aw8697 i2c write/read
 *
 ***********************************************************************************************/
static int aw8697_i2c_write(struct aw8697 *aw8697, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = 0;
	unsigned char cnt = 0;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw8697->bus_lock);
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw8697->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: error: addr=%#x data=%#x cnt=%d error=%d\n", __func__,
				reg_addr, reg_data, cnt, ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000, AW_I2C_RETRY_DELAY * 1000 + 500);

	}
	//pr_debug("%s: addr=%#x, data=%#x\n", __func__, reg_addr, reg_data);
	mutex_unlock(&aw8697->bus_lock);
	return ret;
}

static int aw8697_i2c_read(struct aw8697 *aw8697, unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = 0;
	unsigned char cnt = 0;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw8697->bus_lock);
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw8697->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: error: addr=%#x cnt=%d error=%d\n", __func__, reg_addr, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000, AW_I2C_RETRY_DELAY * 1000 + 500);
	}
	mutex_unlock(&aw8697->bus_lock);

	return ret;
}

static int aw8697_i2c_write_bits(struct aw8697 *aw8697, unsigned char reg_addr,
								unsigned int mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;
	int ret = 0;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	ret = aw8697_i2c_read(aw8697, reg_addr, &reg_val);
	if (ret < 0) {
		pr_err("%s i2c read failed, ret=%d\n", __func__, ret);
		return ret;
	}

	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw8697_i2c_write(aw8697, reg_addr, reg_val);
	if (ret < 0) {
		pr_err("%s: i2c write failed, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int aw8697_i2c_writes(struct aw8697 *aw8697,
		unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = 0;
	unsigned char *data = NULL;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	if ((len > RTP_SLAB_SIZE) || (rtp_cachep == NULL)) {

		data = kzalloc(len+1, GFP_KERNEL);
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
		//return rc;
	}

	mutex_unlock(&aw8697->bus_lock);

	if ((len > RTP_SLAB_SIZE) || (rtp_cachep == NULL)) {

		if (data != NULL)
			kfree(data);
	} else {
		if (data != NULL)
			kmem_cache_free(rtp_cachep, data);
	}
	return ret;
}


/*********************************************************************************************
 *
 * ram update and rtp init use aw8697_osc_cali.bin
 *
 *********************************************************************************************/
static void aw8697_rtp_loaded(const struct firmware *cont, void *context)
{
	struct aw8697 *aw8697 = context;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw8697_osc_cali_file);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded [%s] - size: %zu\n", __func__, aw8697_osc_cali_file,
				cont ? cont->size : 0);


	if (aw8697_rtp == NULL) {
		aw8697_rtp = devm_kzalloc(aw8697->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);
		if (aw8697_rtp == NULL) {
			pr_err("%s devm kzalloc failed\n", __func__);
			release_firmware(cont);
			return;
		}
	}
	memset(aw8697_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));

	if (cont->size < RTP_BIN_MAX_SIZE)
		aw8697_rtp->len = cont->size;
	else
		aw8697_rtp->len = RTP_BIN_MAX_SIZE;

	memcpy(aw8697_rtp->data, cont->data, aw8697_rtp->len);
	release_firmware(cont);

	aw8697->rtp_init = 1;
	pr_info("%s: rtp update complete\n", __func__);
}

/* This is done only once during the RAM INIT process, can use same osc file */
static int aw8697_rtp_update(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw8697_osc_cali_file, aw8697->dev, GFP_KERNEL,
				aw8697, aw8697_rtp_loaded);

}
#if 0
static void aw8697_softreset_ram_reg_init(struct aw8697 *aw8697)
{
	unsigned int shift = 0;
	//uint8_t base_addr_h = 0, base_addr_l = 0;

	//base_addr_h = aw8697->ram.base_addr>>8;
	//base_addr_l = aw8697->ram.base_addr&0x00FF;

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
		AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write(aw8697, AW8697_REG_BASE_ADDRH, aw8697->ram.base_addr>>8);
	aw8697_i2c_write(aw8697, AW8697_REG_BASE_ADDRL, aw8697->ram.base_addr&0x00FF);
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AEH,
					(unsigned char)((aw8697->ram.base_addr>>1) >> 8));//1/2 FIFO
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AEL,
					(unsigned char)((aw8697->ram.base_addr>>1)&0x00FF));//1/2 FIFO
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AFH,
					(unsigned char)((aw8697->ram.base_addr-(aw8697->ram.base_addr>>2)) >> 8));
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AFL,
					(unsigned char)((aw8697->ram.base_addr-(aw8697->ram.base_addr>>2))&0x00FF));
	shift = aw8697->ram.baseaddr_shift;
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH, aw8697->ram.base_addr>>8);
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL, aw8697->ram.base_addr&0x00FF);
	/* RAMINIT Disable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_RAMINIT_MASK, AW8697_BIT_SYSCTRL_RAMINIT_OFF);
}
#endif
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
	// ram check
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
		pr_err("%s: failed to read [%s]\n", __func__, aw8697_ram_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s enter: loaded [%s] - size: %zu\n", __func__, aw8697_ram_name,
					cont ? cont->size : 0);

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
	// need turn on trigger only used by power key
	//aw8697_haptic_trig_enable_config(aw8697);

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

	pr_info("%s enter, file_name: %s\n", __func__, file_name);

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



/********************************************************************************************
 *
 * base haptic control
 *
 ********************************************************************************************/
static void aw8697_vol_trig_switch(struct aw8697 *aw8697, bool sw)
{

	if (!aw8697->no_trigger && !aw8697->disable_trigger_force) {
		if (sw) {
			aw8697->trig[0].enable = 0;
			aw8697->trig[1].enable = 0;
			aw8697->trig[2].enable = 1;
			pr_err("%s sw=%d\n", __func__, sw);
			aw8697_haptic_trig_enable_config(aw8697);
		} else {
			aw8697->trig[0].enable = 0;
			aw8697->trig[1].enable = 0;
			aw8697->trig[2].enable = 0;
			pr_err("%s sw=%d\n", __func__, sw);
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


	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_WORK_MODE_MASK, AW8697_BIT_SYSCTRL_ACTIVE);
	aw8697_interrupt_clear(aw8697);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_EN);
	return 0;
}

static int aw8697_haptic_play_mode(struct aw8697 *aw8697, unsigned char play_mode)
{

	switch (play_mode) {
	case AW8697_HAPTIC_STANDBY_MODE:
		pr_err("%s: standby mode\n", __func__);
		aw8697->play_mode = AW8697_HAPTIC_STANDBY_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_OFF);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				AW8697_BIT_SYSCTRL_WORK_MODE_MASK&AW8697_BIT_SYSCTRL_WAVDAT_MODE_MASK, AW8697_BIT_SYSCTRL_STANDBY|AW8697_BIT_SYSCTRL_WAVDAT_MODE_1X);
		break;
	case AW8697_HAPTIC_RAM_MODE:
		pr_err("%s: ram mode\n", __func__);
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
		pr_err("%s: ram loop mode\n", __func__);
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
		pr_err("%s: rtp mode\n", __func__);
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
		pr_err("%s: trig mode\n", __func__);
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
		pr_err("%s: cont mode\n", __func__);
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
	s64 delta_ms = 0;
	pr_err("%s enter, flag=%d\n", __func__, flag);

	if (flag == true) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_GO,
			AW8697_BIT_GO_MASK, AW8697_BIT_GO_ENABLE);
			aw8697->begin = ktime_get();
	} else {
		aw8697->cancel = ktime_get();
		delta_ms = ktime_to_ms(ktime_sub(aw8697->cancel, aw8697->begin));

          if (delta_ms < 5) {
                  pr_info("%s --->delta_ms=%u ,time in start and stop is too shaot, cancel delay 5ms.\n", __func__, delta_ms);
                  mdelay(5);
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
		pr_err("%s wait for standby, reg glb_state=0x%02x\n",
			__func__, reg_val);
	}
	pr_err("%s do not enter standby automatically\n", __func__);

	return 0;
}

static int aw8697_haptic_stop(struct aw8697 *aw8697)
{

	aw8697_haptic_play_go(aw8697, false);
	aw8697_haptic_stop_delay(aw8697);
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
	//aw8697_vol_trig_switch(aw8697, true);
	//aw8697_set_clock(aw8697, AW8697_HAPTIC_CLOCK_CALI_F0);
	/* Stop pattern serial broadcasting */
	if (aw8697->play.ram_id == 0) {
		aw8697_double_click_switch(aw8697, false);
	}
	/* Set the default voltage state of the trigger */
	aw8697_haptic_set_gain(aw8697, (u8)aw8697->trigger_gain);
	aw8697_haptic_set_bst_vol(aw8697, 0x13);

	return 0;
}

static int aw8697_haptic_start(struct aw8697 *aw8697)
{

	aw8697_haptic_play_go(aw8697, true);
	/* 获取一次开始时间 */
	//aw8697->begin = ktime_get();
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
	pr_err("%s ram start\n", __func__);

	if (flag) {
		aw8697_vol_trig_switch(aw8697, false);
		aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_MODE);
		aw8697_haptic_start(aw8697);
	}
	return 0;
}


static int aw8697_haptic_play_repeat_seq(struct aw8697 *aw8697, unsigned char flag)
{
	pr_err("%s: ram loop start\n", __func__);

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

/* if flag is true, Long vibration voltage compensation */
static int aw8697_haptic_ram_vbat_comp(struct aw8697 *aw8697, bool flag)
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

static void aw8697_lra_resist_get(struct aw8697 *aw8697, unsigned char *reg_val)
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

/*
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
*/

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
	pr_err("%s: set empty irq, flag (%d)\n", __func__, flag);
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
/*
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
*/
/*
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
*/
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
	int ret = 0, retval = 0;

	pr_info("%s enter\n", __func__);
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, 400);

	mutex_lock(&aw8697->rtp_lock);
	aw8697->rtp_cnt = 0;
	retval = aw8697_haptic_rtp_get_fifo_afs(aw8697);
	if ((!retval) &&
		(aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
		pr_info("%s rtp cnt = %d\n", __func__, aw8697->rtp_cnt);

		if (aw8697_rtp->len > aw8697->rtp_cnt) {
			if ((aw8697_rtp->len - aw8697->rtp_cnt) < (aw8697->ram.base_addr)) {
				buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
			} else {
				buf_len = (aw8697->ram.base_addr);//2k 数据写入
			}

			ret = aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
				&aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
			if (ret < 0) {
				pr_err("%s: i2c write error: total length: %d, play length: %d\n",
					__func__, aw8697_rtp->len, aw8697->rtp_cnt);
				aw8697->rtp_cnt = 0;
				mutex_unlock(&aw8697->rtp_lock);
				pm_qos_remove_request(&pm_qos_req_vb);
				return -EBUSY;
			}
			aw8697->rtp_cnt += buf_len;
		}

		if (aw8697->rtp_cnt >= aw8697_rtp->len) {
			pr_err("%s complete: total length: %d, play length: %d\n", __func__,
				aw8697_rtp->len, aw8697->rtp_cnt);
			aw8697->rtp_cnt = 0;
			mutex_unlock(&aw8697->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}

	if (retval < 0) {
		pr_err("%s: i2c read error: total length: %d, play length: %d\n",
			__func__, aw8697_rtp->len, aw8697->rtp_cnt);
		//pr_err("%s retval return negtive, retval=%d\n", __func__, retval);
		aw8697->rtp_cnt = 0;
		mutex_unlock(&aw8697->rtp_lock);
		pm_qos_remove_request(&pm_qos_req_vb);
		return 0;
	}


	//vivo zhangxiaodong add end
	while ((!(retval = aw8697_haptic_rtp_get_fifo_afs(aw8697))) &&
			(aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
		pr_info("%s rtp cnt = %d\n", __func__, aw8697->rtp_cnt);

		if (aw8697_rtp->len > aw8697->rtp_cnt) {
			if ((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr>>2)) {
				buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
			} else {
				buf_len = (aw8697->ram.base_addr>>2);
			}

			ret = aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
				&aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
			if (ret < 0) {
				pr_err("%s: i2c write error: total length: %d, play length: %d\n",
					__func__, aw8697_rtp->len, aw8697->rtp_cnt);
				aw8697->rtp_cnt = 0;
				mutex_unlock(&aw8697->rtp_lock);
				pm_qos_remove_request(&pm_qos_req_vb);
				return 0;
			}

			aw8697->rtp_cnt += buf_len;

		}

		if (aw8697->rtp_cnt >= aw8697_rtp->len) {
			pr_err("%s complete total length: %d, play length: %d\n",
				__func__, aw8697_rtp->len, aw8697->rtp_cnt);
			aw8697->rtp_cnt = 0;
			mutex_unlock(&aw8697->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}

	if (retval < 0) {
		pr_err("%s: i2c read error--->2: total length: %d, play length: %d\n",
			__func__, aw8697_rtp->len, aw8697->rtp_cnt);
		aw8697->rtp_cnt = 0;
		mutex_unlock(&aw8697->rtp_lock);
		pm_qos_remove_request(&pm_qos_req_vb);
		return 0;
	}


	if ((aw8697->play_mode == AW8697_HAPTIC_RTP_MODE) && (retval >= 0)) {
		pr_err("%s open rtp irq\n", __func__);
		aw8697_haptic_set_rtp_aei(aw8697, true);
	}

	pr_info("%s exit\n", __func__);
	mutex_unlock(&aw8697->rtp_lock);
	pm_qos_remove_request(&pm_qos_req_vb);
	return 0;
}

static int aw8697_set_clock(struct aw8697 *aw8697, int clock_type)
{
	unsigned char code;

	if (clock_type == AW8697_HAPTIC_CLOCK_CALI_F0) {
		code = (unsigned char)atomic_read(&aw8697->f0_freq_cali);
		pr_err("%s f0 clock, value=%d, code=%#x\n", __func__, atomic_read(&aw8697->f0_freq_cali), code);
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, code);

	} else if (clock_type == AW8697_HAPTIC_CLOCK_CALI_OSC_STANDARD) {
		code = (unsigned char)atomic_read(&aw8697->standard_osc_freq_cali);
		pr_err("%s osc clock, value=%d, code=%#x\n", __func__, atomic_read(&aw8697->standard_osc_freq_cali), code);
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, code);
	}
	return 0;
}

#if 0
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
#endif
/*
static unsigned char aw8697_haptic_osc_read_int(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	aw8697_i2c_read(aw8697, AW8697_REG_DBGSTAT, &reg_val);
	return reg_val;
}
*/
#if 0
static int aw8697_rtp_osc_calibration(struct aw8697 *aw8697) //不做osc校准
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int buf_len = 0;
	unsigned char osc_int_state = 0;

	aw8697->rtp_cnt = 0;
	aw8697->timeval_flags = 1;
	aw8697->osc_cali_flag = 1;

	pr_info("%s enter\n", __func__);

	/* fw loaded */
	ret = request_firmware(&rtp_file, aw8697_osc_cali_file, aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__, aw8697_osc_cali_file);
		return ret;
	}

	aw8697_haptic_stop(aw8697);

	aw8697->rtp_init = 0;

	if (aw8697_rtp == NULL) {
		aw8697_rtp = devm_kzalloc(aw8697->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);
		if (aw8697_rtp == NULL) {
			pr_err("%s devm kzalloc failed\n", __func__);
			release_firmware(rtp_file);
			return -ENOMEM;
		}
	}
	memset(aw8697_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));


	aw8697_rtp->len = rtp_file->size;
	aw8697->rtp_len = rtp_file->size;

	pr_info("%s: rtp file [%s] size = %d\n", __func__,
			aw8697_osc_cali_file, aw8697_rtp->len);

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
					aw8697->start = ktime_get();
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
			aw8697->end = ktime_get();
			pr_info("%s vincent playback done aw8697->rtp_cnt= %d \n", __func__, aw8697->rtp_cnt);
			break;
		}
		aw8697->end = ktime_get();
		aw8697->microsecond = 1000 * ktime_to_us(ktime_sub(aw8697->end, aw8697->start));
		if (aw8697->microsecond > OP_OCS_CALIBRATION_T_LENGTH) {
			pr_info("%s vincent time out aw8697->rtp_cnt %d osc_int_state %02x\n",
					__func__, aw8697->rtp_cnt, osc_int_state);
			break;
			}
	}
	pm_qos_remove_request(&pm_qos_req_vb);
	enable_irq(gpio_to_irq(aw8697->irq_gpio));

	aw8697->osc_cali_flag = 0;
	aw8697->microsecond = 1000 * ktime_to_us(ktime_sub(aw8697->end, aw8697->start));
	/*calibration osc*/
	pr_info("aw8697 2018_microsecond:%ld \n", aw8697->microsecond);

	pr_info("%s exit\n", __func__);
	return 0;
}
#endif
static void aw8697_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	struct aw8697 *aw8697 = container_of(work, struct aw8697, rtp_work);
	struct aw8697_play_info *play = &aw8697->play;
	struct aw8697_wavefrom_info *effect_list = aw8697->effect_list;


	pr_info("%s enter\n", __func__);

	if (play->type != RTP_TYPE) {
		dev_err(aw8697->dev, "new not rtp effect coming\n");
		return;
	}

	/* fw loaded */
	ret = request_firmware(&rtp_file, effect_list[aw8697->rtp_file_num].rtp_file_name, aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read [%s]\n", __func__, effect_list[aw8697->rtp_file_num]);
		return ;
	}

	mutex_lock(&aw8697->rtp_lock);
	aw8697->rtp_init = 0;
	aw8697_haptic_set_rtp_aei(aw8697, false);

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


	pr_info("%s: rtp file [%s] size = %d\n", __func__,
			effect_list[aw8697->rtp_file_num].rtp_file_name, rtp_file->size);

	if (rtp_file->size < RTP_BIN_MAX_SIZE)
		aw8697_rtp->len = rtp_file->size;
	else
		aw8697_rtp->len = RTP_BIN_MAX_SIZE;
	memcpy(aw8697_rtp->data, rtp_file->data, aw8697_rtp->len);
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


/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static void aw8697_haptic_audio_cancel(struct aw8697 *aw8697);

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
        //if cmd==1,start vibrating
		if (aw8697->haptic_audio.ctr.cmd == AW8697_HAPTIC_CMD_ENABLE) {

			if (aw8697->haptic_audio.ctr.play == AW8697_HAPTIC_PLAY_ENABLE) {
				pr_info("%s: haptic_audio_play_start\n", __func__);
				mutex_lock(&aw8697->lock);
				aw8697_haptic_stop(aw8697);

				aw8697_haptic_set_wav_seq(aw8697, 0x00,
							aw8697->haptic_audio.ctr.wavseq);

				aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);

				aw8697_haptic_set_wav_loop(aw8697, 0x00,
							aw8697->haptic_audio.ctr.loop);

                aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_MODE);
                //set bost to 6000 + 19 x 142 = 8700mv,0.7V为线损
                aw8697_haptic_set_bst_vol(aw8697, 19);

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

static void aw8697_reg_init_after_softreset(struct aw8697 *aw8697)
{


	aw8697_interrupt_setup(aw8697);
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

	aw8697_reg_init_after_softreset(aw8697);

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
		aw8697_lra_resist_get(aw8697, &lra_val);
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

static int aw8697_haptic_init(struct aw8697 *aw8697)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);
	INIT_WORK(&aw8697->init_setting_work, aw8697_init_setting_work_routine);
	schedule_work(&aw8697->init_setting_work);

	return ret;
}



/*****************************************************
 *
 * vibrator
 *
 *****************************************************/

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


/******************************************************************************************
 *
 * api interface
 *
 ******************************************************************************************/

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
	if (vmax_val & 0xe0) {
		pr_err("aw8697 vmax_val=%d, gain_val=%d\n", 6000 + 0x1f * 142, gain_val);
	}else{
		pr_err("aw8697 vmax_val=%d, gain_val=%d\n", 6000 + vmax_val * 142, gain_val);
	}

	aw8697_haptic_set_bst_vol(aw8697, vmax_val);
	aw8697_haptic_set_gain(aw8697, gain_val);

	return ret;

}


static int aw8697_load_effect(struct aw8697 *aw8697, struct haptic_effect *p_effect)
{
	struct aw8697_play_info *play = &aw8697->play;
	s16 level, custom_data[CUSTOM_DATA_LEN] = {0, 0, 0};
	int real_vmax, i, j = 0, scene_effect_id = 0, scene_vmax = 0;
	struct aw8697_wavefrom_info *effect_list = aw8697->effect_list;

	memset(play, 0, sizeof(struct aw8697_play_info));

	pr_err("%s: p_effect pointer (%#x)\n", __func__, p_effect); //ignore

	//other mode is play, cancla QQfly Driver vibrator
	 mutex_lock(&aw8697->haptic_audio.lock);
	if (!aw8697->haptic_audio.haptic_audio_cancel_flag) {
		aw8697_haptic_audio_cancel(aw8697);
	}
	aw8697->ff_play_finish = false;
	mutex_unlock(&aw8697->haptic_audio.lock);

	switch (p_effect->type) {

	case HAPTIC_CONSTANT:

		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			pr_err("%s copy from user failed\n", __func__);
			return -EFAULT;
		}


		play->playLength = p_effect->length * USEC_PER_MSEC;
		level = p_effect->magnitude;
		play->vmax = level * aw8697->default_vmax / 0x7fff;
		play->type = TIME_TYPE;

		pr_err("%s: constant, length_us = %d, vmax_mv = %d, level = %d \n", __func__, play->playLength, play->vmax, level);
		aw8697_haptic_stop(aw8697);
		aw8697_haptic_set_repeat_wav_seq(aw8697, PLAYBACK_INFINITELY_RAM_ID);

		break;

	case HAPTIC_CUSTOM:

		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			pr_err("%s copy from user failed\n", __func__);
			return -EFAULT;
		}

		dev_err(aw8697->dev, "scene id %d\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
		if (custom_data[CUSTOM_DATA_EFFECT_IDX] < BASE_SCENE_COUNT_MAX) { //小于300的场景编号，从base scene列表里面找

			for (j = 0; j < aw8697->base_scene_count; j++) {
				if (aw8697->base_scene_list[j].scene_id == custom_data[CUSTOM_DATA_EFFECT_IDX])
					break;
			}

			if (j == aw8697->base_scene_count) {
				dev_err(aw8697->dev, "scene:%d not support\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
				return -EINVAL;
			}

			scene_vmax = aw8697->base_scene_list[j].real_vmax; //real_vmax为场景设计的实际电压
			scene_effect_id = aw8697->base_scene_list[j].effect_id;
		} else { //大于300的场景编号，从扩展ext scene列表里面找

			for (j = 0; j < aw8697->ext_scene_count; j++) {
				if (aw8697->ext_scene_list[j].scene_id == custom_data[CUSTOM_DATA_EFFECT_IDX])
					break;
			}

			if (j == aw8697->ext_scene_count) {
				dev_err(aw8697->dev, "scene:%d not support\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
				return -EINVAL;
			}

			scene_vmax = aw8697->ext_scene_list[j].real_vmax;
			scene_effect_id = aw8697->ext_scene_list[j].effect_id;
		}


		for (i = 0; i < aw8697->effects_count; i++) //从效果中寻找场景需要的效果
			if (aw8697->effect_list[i].idx == scene_effect_id)
				break;

		if (i == aw8697->effects_count) {
			dev_err(aw8697->dev, "scene: %d effect: %d not supported!\n",
			custom_data[CUSTOM_DATA_EFFECT_IDX], scene_effect_id);
			return -EINVAL;
		}

		//更新real vmax值，dts配置和hidl配置，取较小值
		if (scene_vmax > 0 && scene_vmax < effect_list[i].vmax)
			real_vmax = scene_vmax;
		else
			real_vmax = effect_list[i].vmax;

		dev_err(aw8697->dev, "real_vamx = %d, scene_vamx = %d, effect_vmax = %d\n", real_vmax, scene_vmax, effect_list[i].vmax);

		if (!effect_list[i].rtp_enable) {

			play->type = RAM_TYPE;
			level = p_effect->magnitude;
			play->vmax = level * real_vmax / 0x7fff;
			play->times_ms = effect_list[i].times_ms;
			play->ram_id = effect_list[i].ram_id;

			dev_err(aw8697->dev, "ram, effect_id = %d, ram_id = %d, vmax_mv = %d, length = %d, level = %d\n",
								scene_effect_id, play->ram_id, play->vmax, play->times_ms, level);
			aw8697_haptic_stop(aw8697);
			aw8697_haptic_set_wav_loop(aw8697, 0x00, 0x00);
			aw8697_haptic_set_wav_seq(aw8697, 0x00, play->ram_id);
			aw8697_set_play_vol(aw8697);
			if (aw8697->play.ram_id == 0) {
				aw8697_double_click_switch(aw8697, true);
			}
		} else {

			play->type = RTP_TYPE;
			level = p_effect->magnitude;
			play->vmax = level * real_vmax / 0x7fff;
			play->times_ms = effect_list[i].times_ms;
			strlcpy(play->rtp_file, effect_list[i].rtp_file_name, 128);
			aw8697->rtp_file_num = i;

			pr_err("%s: rtp, effect_id = %d, rtp_name: %s, vamx_mv = %d, length = %d, level = %d\n",
										__func__, scene_effect_id, play->rtp_file, play->vmax, play->times_ms, level);
			aw8697_haptic_stop(aw8697);
			aw8697_haptic_set_rtp_aei(aw8697, false);
			aw8697_haptic_set_wav_loop(aw8697, 0x00, 0x00);
			aw8697_set_play_vol(aw8697);
			aw8697_interrupt_clear(aw8697);
		}


		custom_data[CUSTOM_DATA_TIMEOUT_SEC_IDX] = play->times_ms / MSEC_PER_SEC;
		custom_data[CUSTOM_DATA_TIMEOUT_MSEC_IDX] = play->times_ms % MSEC_PER_SEC;

		if (copy_to_user(p_effect->custom_data, custom_data, sizeof(s16) * CUSTOM_DATA_LEN))	{
			pr_err("%s copy to user failed\n", __func__);
			return -EFAULT;
		}

		break;

	default:
		pr_err("%s Unsupported effect type: %d\n", __func__, p_effect->type);
		return -ENODEV;
		break;
	}

	return 0;
}

static int aw8697_playback_vib(struct aw8697 *aw8697, int val)
{
	struct aw8697_play_info *play = &aw8697->play;
	int len;
	if (val) {

		switch (play->type) {

		case RAM_TYPE:
			pr_err("%s: ---> start ram mode\n", __func__);
			aw8697_haptic_play_wav_seq(aw8697, true);
			break;

		case RTP_TYPE:
			pr_err("%s: ---> start rtp mode\n", __func__);
			queue_work(rtp_wq, &aw8697->rtp_work);
			rtp_check_flag = true;
			break;
		case TIME_TYPE:
			aw8697->activate_mode = AW8697_HAPTIC_ACTIVATE_RAM_MODE;

			aw8697->gain = 0x80; //4200为电池电压,波形已适配到2.5V，长震取最大电压
			dev_err(aw8697->dev, "play vmax = %d, aw8697 gain = %d\n", play->vmax, aw8697->gain);
			/* clip value to max */
			len = play->playLength + 10000;

			pr_err("%s: ---> start time mode, length = %d\n", __func__, len);

			if (aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_RAM_MODE) {
				aw8697_haptic_ram_vbat_comp(aw8697, false);
				aw8697_haptic_play_repeat_seq(aw8697, true);
			} else if (aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_CONT_MODE) {
				aw8697_haptic_cont(aw8697);
			} else {
				pr_err("%s: not suppoert activate mode\n", __func__);
			}
			/* run us timer */
			hrtimer_start(&aw8697->timer,
				ktime_set(len / USEC_PER_SEC, (len % USEC_PER_SEC) * NSEC_PER_USEC),
				HRTIMER_MODE_REL);

			break;

		default:

			break;

		}

	} else {
		if (hrtimer_active(&aw8697->timer)) {
			hrtimer_cancel(&aw8697->timer);
			pr_err("%s playback cancel timer\n", __func__);
		}

		/* 取消rtp work */
		if (cancel_work_sync(&aw8697->rtp_work)) {
			pr_err("%s palyback pending work cancle success\n", __func__);
		}
		//aw8697->rtp_len = 0;
		mutex_lock(&aw8697->rtp_check_lock);
		aw8697_haptic_stop(aw8697);
		rtp_check_flag = false;
		mutex_unlock(&aw8697->rtp_check_lock);
		aw8697_vol_trig_switch(aw8697, true);
		aw8697_set_clock(aw8697, AW8697_HAPTIC_CLOCK_CALI_F0);
	}
	//play finish, allow QQfly Driver vibrator
	mutex_lock(&aw8697->haptic_audio.lock);
	if (!val) {
		pr_debug("%s: haptic audio ,ff play finish\n", __func__);
		aw8697->ff_play_finish = true;

	}
	mutex_unlock(&aw8697->haptic_audio.lock);

	return 0;
}

static int aw8697_upload_sync(struct haptic_device *hp, struct haptic_effect *effect)
{
	int ret = 0;
	struct aw8697 *aw8697 = (struct aw8697 *)hp->chip;

	ret = aw8697_load_effect(aw8697, effect);

	return ret;
}

static int aw8697_erase_sync(struct haptic_device *hp)
{
	int ret = 0;
	struct aw8697 *aw8697 = (struct aw8697 *)hp->chip;

	ret = aw8697_playback_vib(aw8697, 0);
	return 0;
}
static int aw8697_playback_sync(struct haptic_device *hp, int value)
{
	struct aw8697 *aw8697 = (struct aw8697 *)hp->chip;
	int ret = 0;

	ret = aw8697_playback_vib(aw8697, !!value);

	return ret;
}

static void aw8697_set_gain_sync(struct haptic_device *hp, u16 gain)
{

	struct aw8697 *aw8697 = (struct aw8697 *)hp->chip;
	struct aw8697_play_info *play = &aw8697->play;

	pr_info("%s gain=%d\n", __func__, gain);

	if (play->type == TIME_TYPE) {
		if (gain > 0x7fff)
			gain = 0x7fff;

		aw8697->gain = ((u32)(gain * 0x80)) / 0x7fff;
		aw8697_haptic_ram_vbat_comp(aw8697, true);
	}

}

// switch trigger en
static void aw8697_trigger_en_status_change(struct aw8697 *aw8697, bool en)
{
	mutex_lock(&aw8697->trigger_en_lock);
	if (en) {
		if (!aw8697->trigger_en_status) {
			aw8697_vol_trig_switch(aw8697, true);
			aw8697->trigger_en_status = true;
			pr_err("%s: trigger_en_status=%d\n", __func__, aw8697->trigger_en_status);
		}
	} else {
		if (aw8697->trigger_en_status) {
			aw8697_vol_trig_switch(aw8697, false);
			aw8697->trigger_en_status = false;
			pr_err("%s: trigger_en_status=%d\n", __func__, aw8697->trigger_en_status);
		}
	}
	mutex_unlock(&aw8697->trigger_en_lock);

}


// set press key trigger strenght and switch
static void aw8697_set_trigger_intensity_sync(struct haptic_device *hp, int scale)
{
	struct aw8697 *aw8697 = (struct aw8697 *)hp->chip;
	uint8_t reg_val;

	pr_info("%s trigger intensity=%d\n", __func__, scale);

	if (scale > 0xff)
		scale = 0xff;

	if (aw8697->no_trigger) {
		pr_err("%s no trigger found\n", __func__);
		return;
	}

	if (scale == 0) {
		aw8697->disable_trigger_force = false;
		aw8697_trigger_en_status_change(aw8697, false);
		aw8697->disable_trigger_force = true;
	} else {
		aw8697->trigger_gain = (scale * 0x80) / 0xff;
		// Avoid voltage overruns caused by adjusting the gain in constant Vibrate
		aw8697_i2c_read(aw8697, AW8697_REG_GO, &reg_val);
		pr_err("%s go reg: %#x\n", __func__, reg_val);
		if ((reg_val & (~AW8697_BIT_GO_MASK)) == 0x00) {
			aw8697_haptic_set_gain(aw8697, (u8)aw8697->trigger_gain);
			aw8697_haptic_set_bst_vol(aw8697, 0x13);
		}

		aw8697->disable_trigger_force = false;
		aw8697_trigger_en_status_change(aw8697, true);
	}
}

static struct haptic_device hp = {

	.name = "aw8697_haptic",
	.upload = aw8697_upload_sync,
	.erase =  aw8697_erase_sync,
	.playback = aw8697_playback_sync,
	.set_gain =  aw8697_set_gain_sync,
	.set_trigger_intensity = aw8697_set_trigger_intensity_sync,

};

static void haptic_device_set_capcity(struct haptic_device *hp, int bit)
{
	if (bit > HAPTIC_CNT - 1) {
		pr_err("%s invalid bit, bit=%d\n", __func__, bit);
		return;
	}
	__set_bit(bit, hp->hap_bit);
}

static int aw8697_vibrator_init(struct aw8697 *aw8697)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);

	hrtimer_init(&aw8697->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8697->timer.function = aw8697_vibrator_timer_func;
	INIT_WORK(&aw8697->vibrator_work, aw8697_vibrator_work_routine);

	/* rtp模式要求数据写入及时，不能断流，故建立单独的工作队列，专门处理rtp数据传输，而不使用系统默认的工作队列 */
	rtp_wq = create_singlethread_workqueue("rtp_wq");
	INIT_WORK(&aw8697->rtp_work, aw8697_rtp_work_routine);

	mutex_init(&aw8697->lock);
	mutex_init(&aw8697->bus_lock);
	mutex_init(&aw8697->rtp_check_lock);
	mutex_init(&aw8697->rtp_lock);
	mutex_init(&aw8697->trigger_en_lock);
	wake_lock_init(&aw8697->wake_lock, WAKE_LOCK_SUSPEND, "vivo-aw8697-wakelock");


	ret = misc_register(&aw8697_haptic_misc);
	if (ret) {
		dev_err(aw8697->dev,  "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}

	/* register haptic miscdev */

	hp.chip = aw8697;
	hp.dev = &aw8697->i2c->dev;
	haptic_device_set_capcity(&hp, HAPTIC_MASK_BIT_SUPPORT_EFFECT);
	haptic_device_set_capcity(&hp, HAPTIC_MASK_BIT_SUPPORT_GAIN);
	haptic_device_set_capcity(&hp, HAPTIC_MASK_BIT_TRIGGER_INTENSITY);
	pr_info("%s ---> hp=%p\n", __func__, &hp);
	ret = haptic_miscdev_register("vivo_haptic", &hp);
	if (ret) {
		pr_err("%s register misc register failed, ret=%d\n", __func__, ret);
		return -EFAULT;
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

	//aw8697->perm_disable = false;

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
	pr_err("%s enter: reg SYSINT=0x%x\n", __func__, reg_val);
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

	pr_err("%s enter\n", __func__);

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
		pr_err("%s: aw8697 rtp fifo almost empty int\n", __func__);
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
					pr_err("%s total length: %d, play length: %d\n",
						__func__, aw8697_rtp->len, aw8697->rtp_cnt);
					pr_err("%s error, ret=%d\n", __func__, ret);
					aw8697_haptic_set_rtp_aei(aw8697, false);
					aw8697->rtp_cnt = 0;
					aw8697->rtp_init = 0;
					break;
				}
				aw8697->rtp_cnt += buf_len;
				if (aw8697->rtp_cnt >= aw8697_rtp->len) {
					pr_err("%s complete total length: %d, play length: %d\n",
						__func__, aw8697_rtp->len, aw8697->rtp_cnt);
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
		pr_err("%s: aw8697 rtp mode fifo full empty\n", __func__);
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
 * device tree
 *
 *****************************************************/

static aw8697_parse_per_effect_dt(struct aw8697 *aw8697, struct device_node *child,
								struct aw8697_wavefrom_info *effect_node)
{
	int rc;

	rc = of_property_read_u32(child, "awinic,effect-id", &effect_node->idx);
	if (rc < 0) {
		dev_err(aw8697->dev, "Read awinic effect-id failed, rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(child, "awinic,wf-vmax-mv", &effect_node->vmax);
	if (rc < 0) {
		dev_err(aw8697->dev, "Read awinic wf-vmax-mv failed, rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(child, "awinic,wf-length", &effect_node->times_ms);
	if (rc < 0) {
		dev_err(aw8697->dev, "Read awinic wf-length failed, rc=%d\n", rc);
		return rc;
	}

	effect_node->rtp_enable = of_property_read_bool(child, "awinic,rtp-enable");

	if (effect_node->rtp_enable) {

		rc = of_property_read_string(child, "awinic,rtp-file", &effect_node->rtp_file_name);
		if (rc < 0) {
			dev_err(aw8697->dev, "Read awinic rtp-file failed, rc=%d\n", rc);
			return rc;
		}

		effect_node->ram_id = 1;

	} else {

		rc = of_property_read_u32(child, "awinic,ram-id", &effect_node->ram_id);
		if (rc < 0) {
			dev_err(aw8697->dev, "Read awinic ram-id failed, rc=%d\n", rc);
			return rc;
		}
		effect_node->rtp_file_name = "default";
	}

	return 0;
}

static void __dump_scene_info(struct aw8697 *chip)
{
	struct scene_effect_info *base_scene_list = chip->base_scene_list;
	struct scene_effect_info *ext_scene_list = chip->ext_scene_list;
	int i;

	dev_dbg(chip->dev, "Dump base scene info begin =============\n");
	if (base_scene_list) {
		for (i = 0; i < chip->base_scene_count ; i++) {
			dev_dbg(chip->dev, "scene id: %d, effect id: %d, real_vamx: %d\n",
			base_scene_list[i].scene_id, base_scene_list[i].effect_id,
			base_scene_list[i].real_vmax);
		}
	}

	dev_dbg(chip->dev, "Dump ext scene info begin =============\n");
	if (ext_scene_list) {
		for (i = 0; i < chip->ext_scene_count ; i++) {
			dev_dbg(chip->dev, "scene id: %d, effect id: %d, real_vamx: %d\n",
			ext_scene_list[i].scene_id, ext_scene_list[i].effect_id,
			ext_scene_list[i].real_vmax);
		}
	}

}


static int aw8697_parse_scene_dt(struct aw8697 *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *scene_node;
	int ret = 0, base_scene_element_count = 0, ext_scene_element_count = 0, i;

	struct scene_effect_info *base_scene_info = NULL;
	struct scene_effect_info *ext_scene_info = NULL;

	#define SCENE_NU    1024
	u16 *temp_effect = NULL;

    temp_effect = devm_kcalloc(chip->dev, SCENE_NU, sizeof(u16), GFP_KERNEL);// 1024/3=341
	if (!temp_effect) {
		ret = -ENODEV;
		dev_err(chip->dev, "Kcalloc for temp_effect failed\n");
		goto err_out;
	}

	scene_node = of_parse_phandle(node, "scene_array", 0);
	if (!scene_node) {
		ret = -ENODEV;
		dev_err(chip->dev, "Get effect array phandle failed\n");
		goto err_out;
	}

	base_scene_element_count = of_property_count_u16_elems(scene_node, "base_scene");
	if (base_scene_element_count < 0) {
		ret = base_scene_element_count;
		dev_err(chip->dev, "Get base effect elements count failed, ret=%d\n", ret);
		goto err_out;
	}

	if (base_scene_element_count > SCENE_NU) {
		dev_err(chip->dev, "base scene overflow, %d more than %d\n", base_scene_element_count, SCENE_NU);
		goto err_out;
	}
	dev_err(chip->dev, "Base scene info elment: %d\n", base_scene_element_count);
	base_scene_info = devm_kcalloc(chip->dev, base_scene_element_count / 3, sizeof(struct scene_effect_info), GFP_KERNEL);
	if (!base_scene_info) {
		dev_err(chip->dev, "Kcalloc for base effect failed\n");
		ret = -ENOMEM;
		goto err_out;
	} else {
		ret = of_property_read_u16_array(scene_node, "base_scene", temp_effect, base_scene_element_count);
		if (ret) {
			dev_err(chip->dev, "Get base effect failed, ret=%d\n", ret);
			goto err_out;
		} else {
			for (i = 0; i < base_scene_element_count / 3; i++) {
			base_scene_info[i].scene_id = temp_effect[3 * i];
			base_scene_info[i].effect_id = temp_effect[3 * i + 1];
			base_scene_info[i].real_vmax = temp_effect[3 * i + 2];
			}
			chip->base_scene_list = base_scene_info;
			chip->base_scene_count = base_scene_element_count / 3;
		}
	}


	ext_scene_element_count = of_property_count_u16_elems(scene_node, "ext_scene");
	if (ext_scene_element_count < 0) {
		ret = ext_scene_element_count;
		dev_err(chip->dev, "Get ext effect elements count failed, ret=%d\n", ret);
		goto err_out;
	}

	if (ext_scene_element_count > SCENE_NU) {
		dev_err(chip->dev, "ext scene overflow, %d more than %d\n", ext_scene_element_count, SCENE_NU);
		goto err_out;
	}

	dev_err(chip->dev, "Ext scene info elment: %d\n", ext_scene_element_count);
	ext_scene_info = devm_kcalloc(chip->dev, ext_scene_element_count / 3, sizeof(struct scene_effect_info), GFP_KERNEL);
	if (!ext_scene_info) {
		dev_err(chip->dev, "Kcalloc for ext effect failed\n");
		ret = -ENOMEM;
		goto err_out;
	} else {
		ret = of_property_read_u16_array(scene_node, "ext_scene", temp_effect, ext_scene_element_count);
		if (ret) {
			dev_err(chip->dev, "Get ext effect failed, ret=%d\n", ret);
			goto err_out;
		} else {
			for (i = 0; i < ext_scene_element_count / 3; i++) {
				ext_scene_info[i].scene_id = temp_effect[3 * i];
				ext_scene_info[i].effect_id = temp_effect[3 * i + 1];
				ext_scene_info[i].real_vmax = temp_effect[3 * i + 2];
			}
			chip->ext_scene_list = ext_scene_info;
			chip->ext_scene_count = ext_scene_element_count / 3;
		}

	}
	devm_kfree(chip->dev, temp_effect);
#ifdef DUMP_DTS_CONFIG
	__dump_scene_info(chip);
#endif

err_out:
	of_node_put(scene_node);
	return ret;
}

/*****************************************************
 *
 * config lra_info,for gongmo
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
    pr_debug("%s aw8697->lra_information = %d \n", __func__, aw8697->lra_information);
    pr_debug("%s AW8697_HAPTIC_F0_PRE = %d \n", __func__, aw8697->lra_info.AW8697_HAPTIC_F0_PRE);
    pr_debug("%s AW8697_HAPTIC_F0_CALI_PERCEN = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_F0_CALI_PERCEN);
    pr_debug("%s AW8697_HAPTIC_CONT_DRV_LVL = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL);
    pr_debug("%s AW8697_HAPTIC_CONT_DRV_LVL_OV = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_DRV_LVL_OV);
    pr_debug("%s AW8697_HAPTIC_CONT_TD = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_TD);
    pr_debug("%s AW8697_HAPTIC_CONT_ZC_THR = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_ZC_THR);
    pr_debug("%s AW8697_HAPTIC_CONT_NUM_BRK = %d;\n", __func__, aw8697->lra_info.AW8697_HAPTIC_CONT_NUM_BRK);
    pr_debug("%s AW8697_HAPTIC_RATED_VOLTAGE = %d\n", __func__, aw8697->lra_info.AW8697_HAPTIC_RATED_VOLTAGE);
    return 0;
}

static int aw8697_parse_rst_irq_gpio_dt(struct device *dev, struct aw8697 *aw8697,
		struct device_node *np)
{
    aw8697->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw8697->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
		//return -ENODEV;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw8697->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
	if (aw8697->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
		return -ENODEV;
	} else {
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
	}

    return 0;
}

static int aw8697_parse_dt(struct device *dev, struct aw8697 *aw8697,
		struct device_node *np)
{

	struct device_node *child;

	int rc, i = 0, effect_count = 0;

	aw8697->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw8697->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
		//return -ENODEV;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw8697->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
	if (aw8697->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
		return -ENODEV;
	} else {
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
	}

	if (of_property_read_u32(np, "awinic,vmax", &aw8697->default_vmax)) {
		dev_err(dev, "%s: no default vmax\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "resistance_min", &aw8697->resistance_min)) {
		dev_err(dev, "%s: no resistance_min\n", __func__);
		return -EFAULT;
	}
	dev_info(dev, "resistance_min:%d\n", aw8697->resistance_min);

	if (of_property_read_u32(np, "resistance_max", &aw8697->resistance_max)) {
		dev_err(dev, "%s: no resistance_max\n", __func__);
		return -EFAULT;
	}
	dev_info(dev, "resistance_max:%d\n", aw8697->resistance_max);

	if (of_property_read_u32(np, "freq_min", &aw8697->freq_min)) {
		dev_err(dev, "%s: no freq_min\n", __func__);
		return -EFAULT;
	}
	dev_info(dev, "freq_min:%d\n", aw8697->freq_min);

	if (of_property_read_u32(np, "freq_max", &aw8697->freq_max)) {
		dev_err(dev, "%s: no freq_max\n", __func__);
		return -EFAULT;
	}
	dev_info(dev, "freq_max:%d\n", aw8697->freq_max);

	if (of_property_read_bool(np, "disable-trigger")) {
		dev_info(dev, "not support trigger\n");
		aw8697->no_trigger = true;
	}

      /* prase lra_info */
	if (of_property_read_u32(np, "lra_info", &aw8697->lra_information)) {
		aw8697->lra_information = 619;
        dev_err(dev, "lra_info:%d\n", aw8697->lra_information);
	}
	dev_info(dev, "lra_info:%d\n", aw8697->lra_information);

	rc = aw8697_lra_information_ctr(aw8697);
	if (rc < 0) {
		pr_err("%s: lra_information_ctr get failed\n", __func__);

	}

	//parse effect begin
	for_each_available_child_of_node(np, child) {
		if (of_find_property(child, "awinic,effect-id", NULL))
			effect_count++;
	}
	if (effect_count == 0) {
		dev_err(dev, "no dts effect configed, use default effect list\n");
		aw8697->effect_list = waveform_list_default;
		aw8697->effects_count = 1;
		return 0;
	}

	aw8697->effect_list = devm_kcalloc(aw8697->dev, effect_count,
				sizeof(*aw8697->effect_list), GFP_KERNEL);
	if (!aw8697->effect_list)
		return -ENOMEM;

	for_each_available_child_of_node(np, child) {
		if (!of_find_property(child, "awinic,effect-id", NULL))
			continue;

		rc = aw8697_parse_per_effect_dt(aw8697, child, &aw8697->effect_list[i]);
		if (rc < 0) {
			dev_err(aw8697->dev, "parse effect %d failed, rc=%d\n", i, rc);
			of_node_put(child);
			return rc;
		}
		i++;
	}

	aw8697->effects_count = i;
	dev_err(aw8697->dev, "effect count: %d, dts node: %d\n", i, effect_count);

	//parse effect end


	// dts中场景列表解析begin
	rc = aw8697_parse_scene_dt(aw8697);
	if (rc < 0) {
		dev_err(aw8697->dev, "Parse scene list failed, rc=%d\n", rc);
		return -EFAULT;
	}
	// dts中场景列表解析end

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
/*********************************************************************************************
 *                                                                                           *
 * sys group for QQflyDriver                                                                 *
 *                                                                                           *
 *********************************************************************************************/
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
	aw8697_haptic_set_gain(aw8697, 0x80);
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

static ssize_t aw8697_haptic_audio_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct aw8697 *aw8697 = g_aw8697;
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw8697->haptic_audio.cnt);
	return len;
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
			if (aw8697->ff_play_finish) {
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

/*********************************************************************************************
 *                                                                                           *
 * sys group attribute: reg                                                                  *
 *                                                                                           *
 *********************************************************************************************/
static ssize_t aw8697_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	struct aw8697_play_info *play = &aw8697->play;
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

static ssize_t aw8697_activate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	struct aw8697_play_info *play = &aw8697->play;
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 0 && val != 1)
		return count;
    
    if(val == 1){
        pr_err("%s: direct play syart, length_us = %d\n", __func__, play->playLength);
    }else{
        pr_err("%s: direct play end\n", __func__);
    }
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_set_repeat_wav_seq(aw8697, PLAYBACK_INFINITELY_RAM_ID);
	play->vmax = aw8697->default_vmax;
	aw8697->gain = 0x80;
	aw8697_playback_vib(aw8697, val);
	return count;
}

static ssize_t aw8697_iic_int_rst_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
	unsigned char reg = 0;
	pr_info("%s enter\n", __func__);
	aw8697_i2c_read(aw8697, AW8697_REG_ID, &reg);

	if (reg != AW8697_CHIPID) {
		pr_err("%s:chip id incorrect! reg = %d\n", __func__, reg);
		return snprintf(buf, PAGE_SIZE, "+IIC:\"0\"\n+INT:\"1\"\n+RST:\"1\"\n");
	}
	return snprintf(buf, PAGE_SIZE, "+IIC:\"1\"\n+INT:\"1\"\n+RST:\"1\"\n");
}

static ssize_t aw8697_at_trigger_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t len = 0;
	unsigned char reg = 0;

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	pr_info("%s enter\n", __func__);

	mutex_lock(&aw8697->lock);
	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg);
	pr_info("%s reg2=%#x\n", __func__, reg);
	if (reg & 0x01) {
		len += snprintf(buf+len, PAGE_SIZE-len, "1");
	} else {
		len += snprintf(buf+len, PAGE_SIZE-len, "0");
	}
	at_test = false; //清空屏蔽
	mutex_unlock(&aw8697->lock);

	return len;
}

static ssize_t aw8697_at_trigger_state_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0, ret = 0;
	unsigned char reg = 0;

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	pr_info("aw8697, at_trigger_state_store enter, %s\n", buf);

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		pr_err("aw8697, at_trigger_state_store kstrtoint failed, ret=%d\n", ret);
		return -EINVAL;
	}
	mutex_lock(&aw8697->lock);
	at_test = true;

	/* 清除timer*/
	if (hrtimer_active(&aw8697->timer)) {
		hrtimer_cancel(&aw8697->timer);
		pr_err("%s cancel timer\n", __func__);
	}
	aw8697_haptic_stop(aw8697);
	msleep(10);
	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg);
	pr_info("%s reg=%#x\n", __func__, reg);
	mutex_unlock(&aw8697->lock);
	return count;
}

/*use current f0,1040 lra no need to cali*/
static ssize_t aw8697_f0_offset_10_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	unsigned int databuf[1] = {0};
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;

    if (aw8697->lra_information == AW8697_LRA_1040) {
		pr_info("%s:enter;aw8697->lra_information is 1040, no need cali\n", __func__);
	} else {

    	pr_info("%s:enter;buf is %s\n", __func__, buf);
    	if (1 == sscanf(buf, "%d", &databuf[0])) {
    		mutex_lock(&aw8697->lock);

    		at_test = true;

    		aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
    		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, 0x00);
    		pr_err("%s user current f0\n", __func__);

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
    			if (f0_cali_step % 10 >= 5)
    				f0_cali_step = f0_cali_step/10 + 1 + 32;
    			else
    				f0_cali_step = f0_cali_step/10  + 32;
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
    		pr_info("%s final trim_lra=0x%02x\n", __func__, reg_val); 
            /* restore default work mode */
    		aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
    		aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
    		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
    				AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
    		aw8697_haptic_stop(aw8697);

    		mdelay(100);
    		pr_info("%s set freq to %dHZ\n", __func__, f0_limit); 

    		at_test = false;

    		mutex_unlock(&aw8697->lock);

    	}
	}
	return count;
}

static ssize_t aw8697_is_need_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	ssize_t len = 0;
	int need_cali = 1;
	char lra[] = "X-LRA 0619";

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
                  need_cali = 1;
                  break;
          default:
                  strcpy(lra, "X-LRA 0619");
                  need_cali = 1;
                  break;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "+Type:\"%s\"\n", lra);
	len += snprintf(buf+len, PAGE_SIZE-len, "+Require:\"%d\"\n", need_cali);

	return len;
}

//先校准再读f0值，再读lra_resistance值
static ssize_t aw8697_cali_f0_resis_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	unsigned char reg_val = 0;
	ssize_t len = 0;
	mutex_lock(&aw8697->lock);

	at_test = true;

	//get resistance
	aw8697_lra_resist_get(aw8697, &reg_val);
	aw8697->lra = 298 * reg_val;

	if ((aw8697->lra >= aw8697->resistance_min) && (aw8697->lra <= aw8697->resistance_max)) {
		pr_err("%s lra resistent test ok, lra=%d\n", __func__, aw8697->lra);
	} else {
		pr_err("%s lra resistent over range, lra=%d\n", __func__, aw8697->lra);
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw8697->lra >= aw8697->resistance_min && aw8697->lra <= aw8697->resistance_max) ? "ok" : "fail",
		aw8697->lra/1000, aw8697->lra%1000,
		aw8697->resistance_min/1000, aw8697->resistance_min%1000/100,
		aw8697->resistance_max/1000, aw8697->resistance_max%1000/100);


	if (aw8697->lra_information == AW8697_LRA_1040) {

		pr_info("%s:enter;aw8697->lra_information is 1040\n", __func__);

		len += snprintf(buf+len, PAGE_SIZE-len, "freqency#ok#170.0#%d.%d-%d.%d#hz\n",
			aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10);

		len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#ok#0\n",
			(aw8697->f0 >= aw8697->freq_min && aw8697->f0 <= aw8697->freq_max) ? "ok" : "fail",
			atomic_read(&aw8697->f0_freq_cali));

	} else {

        mdelay(20);

    	aw8697_haptic_f0_calibration(aw8697);

    	mdelay(200);

    	aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
    	aw8697_haptic_get_f0(aw8697);

    	len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
    		(aw8697->f0 >= aw8697->freq_min && aw8697->f0 <= aw8697->freq_max) ? "ok" : "fail",
    		aw8697->f0/10, aw8697->f0%10,
    		aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10);

    	len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#ok#0\n");
	}

	at_test = false;
	mutex_unlock(&aw8697->lock);

	return len;
}


static ssize_t aw8697_resis_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	unsigned char reg_val = 0;
	ssize_t len = 0;
	mutex_lock(&aw8697->lock);

	at_test = true;

	aw8697_lra_resist_get(aw8697, &reg_val);
	aw8697->lra = 298 * reg_val;

	if ((aw8697->lra >= aw8697->resistance_min) && (aw8697->lra <= aw8697->resistance_max)) {
		pr_err("%s lra resistent test ok, lra=%d\n", __func__, aw8697->lra);
	} else {
		pr_err("%s lra resistent over range, lra=%d\n", __func__, aw8697->lra);
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw8697->lra >= aw8697->resistance_min && aw8697->lra <= aw8697->resistance_max) ? "ok" : "fail",
		aw8697->lra/1000, aw8697->lra%1000,
		aw8697->resistance_min/1000, aw8697->resistance_min%1000/100,
		aw8697->resistance_max/1000, aw8697->resistance_max%1000/100);

	mdelay(20);

	aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
	aw8697_haptic_get_f0(aw8697);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
		(aw8697->f0 >= aw8697->freq_min && aw8697->f0 <= aw8697->freq_max) ? "ok" : "fail",
		aw8697->f0/10, aw8697->f0%10,
		aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10);

	at_test = false;

	mutex_unlock(&aw8697->lock);

	return len;
}

/* 售后工具线性马达校准项
 * AT+BK_VBR_CAL=1  
 *【指令】：cat /sys/class/leds/vibrator/cali_f0
 * Z轴线性马达不需要校准，校准反而可能会影响震动效果
 * 故offset值返回0，且不进行校准，只读f0
 */
static ssize_t aw8697_cali_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	ssize_t len = 0;
	mutex_lock(&aw8697->lock);

	at_test = true;

	if (aw8697->lra_information == AW8697_LRA_1040) {

		pr_info("%s:enter;aw8697->lra_information is 1040\n", __func__);

		len += snprintf(buf+len, PAGE_SIZE-len, "ok f0 170.0 (range:%d.%d-%d.%d)hz f0_offset=0\n",
			aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10);

	} else {

		pr_info("%s:enter;aw8697->lra_information is not 1040\n", __func__);

		aw8697_haptic_f0_calibration(aw8697);

		mdelay(200);

		aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
		aw8697_haptic_get_f0(aw8697);

		len += snprintf(buf+len, PAGE_SIZE-len, "%s f0 %d.%d (range:%d.%d-%d.%d)hz f0_offset=%d\n",
			(aw8697->f0 >= aw8697->freq_min && aw8697->f0 <= aw8697->freq_max) ? "ok" : "fail",
			aw8697->f0/10, aw8697->f0%10,
			aw8697->freq_min/10, aw8697->freq_min%10, aw8697->freq_max/10, aw8697->freq_max%10,
			atomic_read(&aw8697->f0_freq_cali));
	}

	at_test = false;

	mutex_unlock(&aw8697->lock);

	return len;
}

static ssize_t aw8697_i2c_reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);


	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8697_i2c_write(aw8697, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8697_i2c_reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

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
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);


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
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

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

//add for QQflyDriver
//static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, aw8697_haptic_audio_show, aw8697_haptic_audio_store);
//static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO, aw8697_haptic_audio_time_show, aw8697_haptic_audio_time_store);
//add for gongmo
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, NULL, aw8697_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, NULL, aw8697_activate_store);
static DEVICE_ATTR(iic_int_rst, S_IWUSR | S_IRUGO, aw8697_iic_int_rst_show, NULL);
static DEVICE_ATTR(at_trigger_state, S_IWUSR | S_IRUGO, aw8697_at_trigger_state_show,
					aw8697_at_trigger_state_store);
static DEVICE_ATTR(f0_offset_10, S_IWUSR | S_IRUGO, NULL, aw8697_f0_offset_10_store);
static DEVICE_ATTR(is_need_cali, S_IWUSR | S_IRUGO, aw8697_is_need_cali_show, NULL);
static DEVICE_ATTR(cali_f0_resis, S_IWUSR | S_IRUGO, aw8697_cali_f0_resis_show, NULL);
static DEVICE_ATTR(resis_f0, S_IWUSR | S_IRUGO, aw8697_resis_f0_show, NULL);
static DEVICE_ATTR(cali_f0, S_IWUSR | S_IRUGO, aw8697_cali_f0_show, NULL);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8697_i2c_reg_show, aw8697_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8697_i2c_ram_show, aw8697_i2c_ram_store);

static struct attribute *aw8697_attributes[] = {
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_iic_int_rst.attr,
	&dev_attr_at_trigger_state.attr,
	&dev_attr_f0_offset_10.attr,
	&dev_attr_is_need_cali.attr,
	&dev_attr_cali_f0_resis.attr,
	&dev_attr_resis_f0.attr,
	&dev_attr_cali_f0.attr,
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};

static struct attribute_group aw8697_attribute_group = {
	.attrs = aw8697_attributes
};

/******************************************************
 *
 * i2c driver probe
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
    /* init bus_lock first,it was ues in read_chipid */
    mutex_init(&aw8697->bus_lock);
	/* aw8697 rst & int */
	if (np) {
		ret = aw8697_parse_rst_irq_gpio_dt(&i2c->dev, aw8697, np);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to parse rst or int gpio node\n", __func__);
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
    /* aw8697 parse dt */
    if (np) {
            ret = aw8697_parse_dt(&i2c->dev, aw8697, np);
            if (ret) {
                dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
                return -ENODEV;
            }
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

	g_aw8697 = aw8697;

	ret = aw8697_vibrator_init(aw8697);
	if (ret) {
		pr_err("%s: vibrator init failed, ret=%d\n", __func__, ret);
		return -EFAULT;
	}

	aw8697_haptic_init(aw8697);

	aw8697_ram_init(aw8697);

	aw8697->cdev.name = "vibrator";
	aw8697->cdev.brightness_get = aw8697_haptic_brightness_get;
	aw8697->cdev.brightness_set = aw8697_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw8697->i2c->dev, &aw8697->cdev);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s: fail to create led dev\n",
				__func__);
		return ret;
	}
	ret = sysfs_create_group(&aw8697->cdev.dev->kobj, &aw8697_attribute_group);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s error creating sysfs attr files\n", __func__);
		return ret;
	 }
    //QQ fly driver node
    ret = kobject_init_and_add(&aw8697->kobjectDebug, &vibrator_debug_object_type, NULL, "vibrator");
	if (ret) {
		pr_err("%s create vibrator node error!", __func__);
		ret = -1;
		return ret;
	}

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;
}

static int aw8697_i2c_remove(struct i2c_client *i2c)
{
	struct aw8697 *aw8697 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	misc_deregister(&aw8697_haptic_misc);

	sysfs_remove_group(&i2c->dev.kobj, &aw8697_attribute_group);

//	devm_led_classdev_unregister(&aw8697->i2c->dev, &aw8697->cdev);

	wake_lock_destroy(&aw8697->wake_lock);

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
	//aw8697_haptic_trig_enable_config(aw8697);
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

static struct dev_pm_ops aw8697_pm_ops = {
	.suspend = aw8697_pm_suspend,
	.resume = aw8697_pm_resume,
};


static const struct i2c_device_id aw8697_i2c_id[] = {
	{ AW8697_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw8697_i2c_id);

static struct of_device_id aw8697_dt_match[] = {
	{ .compatible = "awinic,haptic" },
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
module_init(aw8697_i2c_init);


static void __exit aw8697_i2c_exit(void)
{
	i2c_del_driver(&aw8697_i2c_driver);
}
module_exit(aw8697_i2c_exit);


MODULE_DESCRIPTION("AW8697 Haptic Driver");
MODULE_LICENSE("GPL v2");
