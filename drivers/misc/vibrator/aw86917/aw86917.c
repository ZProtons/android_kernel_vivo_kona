/*
 * aw869xx.c
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 * Author: Peacek <hushanping@awinic.com>
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
#include <linux/vmalloc.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "aw86917_reg.h"
#include "aw86917.h"

#include <linux/wakelock.h>
#include "../vivo_haptic_core.h"


#undef dev_dbg
#define dev_dbg dev_err

#undef pr_debug
#define pr_debug pr_err

/******************************************************
 *
 * Version Marco
 *
 ******************************************************/
#define AW869XX_DRIVER_VERSION "v0.1.5"
#define AW869XX_HAPTIC_NAME "awinic_haptic"  //miscdev name

/****************************************************************************
 *
 *                  custom macro
 *
 ****************************************************************************/
#define AWINIC_RAM_UPDATE_DELAY
#define RTP_BIN_MAX_SIZE 2000000
#define RTP_SLAB_SIZE 512
#define PLAYBACK_INFINITELY_RAM_ID 6
#define DUMP_DTS_CONFIG
#define BASE_SCENE_COUNT_MAX 300

/****************************************************************************
 *
 *                 static variable
 *
 ****************************************************************************/
static struct aw869xx *g_aw869xx;
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
static char *aw869xx_ram_name = "aw8697_haptic.bin";
//static char *aw869xx_osc_cali_file = "aw869xx_osc_cali.bin";
/*
static char aw869xx_rtp_name[][AW869XX_RTP_NAME_MAX] = {
	{"aw869xx_osc_rtp_24K_5s.bin"},
	{"aw869xx_rtp.bin"},
	{"aw869xx_rtp_lighthouse.bin"},
	{"aw869xx_rtp_silk.bin"},
};
*/

static struct pm_qos_request pm_qos_req_vb;
static struct aw869xx_container *aw869xx_rtp;

//static char osc_cali_result[128];
//static int osc_cali_result_mode;

static struct aw869xx_wavefrom_info waveform_list_default[1] = {

	{1, 1, 8000, 12000, false, "default"},
};

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw869xx_haptic_bst_mode_config(struct aw869xx *aw869xx, unsigned char boost_mode);
static void aw869xx_interrupt_clear(struct aw869xx *aw869xx);
static void aw869xx_haptic_play_go(struct aw869xx *aw869xx);
static int aw869xx_haptic_stop(struct aw869xx *aw869xx);
//static int aw869xx_haptic_trig_enable_config(struct aw869xx *aw869xx);
static void aw869xx_haptic_set_gain(struct aw869xx *aw869xx, unsigned char gain);
static int aw869xx_haptic_set_bst_vol(struct aw869xx *aw869xx, unsigned char bst_vol);
static void aw869xx_interrupt_setup(struct aw869xx *aw869xx);
//static int aw869xx_set_clock(struct aw869xx *aw869xx, int clock_type);
static void aw869xx_double_click_switch(struct aw869xx *aw869xx, bool sw);
static enum hrtimer_restart aw869xx_haptic_audio_timer_func(struct hrtimer *timer);
static void aw869xx_haptic_audio_work_routine(struct work_struct *work);
static void aw869xx_haptic_audio_cancel(struct aw869xx *aw869xx);




 /***********************************************************************************************
 *
 * aw869xx i2c write/read
 *
 ***********************************************************************************************/
static int aw869xx_i2c_write(struct aw869xx *aw869xx, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = 0;
	unsigned char cnt = 0;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw869xx->bus_lock);
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw869xx->i2c, reg_addr, reg_data);
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
	mutex_unlock(&aw869xx->bus_lock);
	return ret;
}

static int aw869xx_i2c_read(struct aw869xx *aw869xx, unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = 0;
	unsigned char cnt = 0;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw869xx->bus_lock);
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw869xx->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: error: addr=%#x cnt=%d error=%d\n", __func__, reg_addr, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000, AW_I2C_RETRY_DELAY * 1000 + 500);
	}
	mutex_unlock(&aw869xx->bus_lock);

	return ret;
}

static int aw869xx_i2c_write_bits(struct aw869xx *aw869xx, unsigned char reg_addr,
								unsigned int mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;
	int ret = 0;

	if (i2c_suspend) {
		pr_err("%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	ret = aw869xx_i2c_read(aw869xx, reg_addr, &reg_val);
	if (ret < 0) {
		pr_err("%s i2c read failed, ret=%d\n", __func__, ret);
		return ret;
	}

	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw869xx_i2c_write(aw869xx, reg_addr, reg_val);
	if (ret < 0) {
		pr_err("%s: i2c write failed, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int aw869xx_i2c_writes(struct aw869xx *aw869xx,
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

	mutex_lock(&aw869xx->bus_lock);

	ret = i2c_master_send(aw869xx->i2c, data, len+1);
	if (ret < 0) {
		pr_err("%s: i2c master send error\n", __func__);
		//return rc;
	}

	mutex_unlock(&aw869xx->bus_lock);

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
 * ram update and rtp init use aw869xx_osc_cali.bin
 *
 *********************************************************************************************/
static void aw869xx_haptic_raminit(struct aw869xx *aw869xx, bool flag)
{
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL1,
				       AW869XX_BIT_SYSCTRL1_RAMINIT_MASK,
				       AW869XX_BIT_SYSCTRL1_RAMINIT_ON);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL1,
				       AW869XX_BIT_SYSCTRL1_RAMINIT_MASK,
				       AW869XX_BIT_SYSCTRL1_RAMINIT_OFF);
	}
}
/*
static void aw869xx_rtp_loaded(const struct firmware *cont, void *context)
{
	struct aw869xx *aw869xx = context;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw869xx_osc_cali_file);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded [%s] - size: %zu\n", __func__, aw869xx_osc_cali_file,
				cont ? cont->size : 0);


	if (aw869xx_rtp == NULL) {
		aw869xx_rtp = devm_kzalloc(aw869xx->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);
		if (aw869xx_rtp == NULL) {
			pr_err("%s devm kzalloc failed\n", __func__);
			release_firmware(cont);
			return;
		}
	}
	memset(aw869xx_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));

	if (cont->size < RTP_BIN_MAX_SIZE)
		aw869xx_rtp->len = cont->size;
	else
		aw869xx_rtp->len = RTP_BIN_MAX_SIZE;

	memcpy(aw869xx_rtp->data, cont->data, aw869xx_rtp->len);
	release_firmware(cont);

	aw869xx->rtp_init = 1;
	pr_info("%s: rtp update complete\n", __func__);
}

// This is done only once during the RAM INIT process, can use same osc file 
static int aw869xx_rtp_update(struct aw869xx *aw869xx)
{
	pr_info("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw869xx_osc_cali_file, aw869xx->dev, GFP_KERNEL,
				aw869xx, aw869xx_rtp_loaded);

}*/

static void aw869xx_container_update(struct aw869xx *aw869xx,
		struct aw869xx_container *aw869xx_cont)
{
	int i = 0;
	unsigned int shift = 0;
    unsigned char reg_val = 0;
	unsigned int temp = 0;

	pr_info("%s enter\n", __func__);

	mutex_lock(&aw869xx->lock);

	aw869xx->ram.baseaddr_shift = 2;
	aw869xx->ram.ram_shift = 4;

	/* RAMINIT Enable */
	aw869xx_haptic_raminit(aw869xx, true);
	/* Enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	/* base addr */
	shift = aw869xx->ram.baseaddr_shift;
	aw869xx->ram.base_addr = (unsigned int)((aw869xx_cont->data[0+shift]<<8) |
			(aw869xx_cont->data[1+shift]));
	pr_info("%s: base_addr=0x%4x\n", __func__, aw869xx->ram.base_addr);
    /*ADDRH*/
    aw869xx_i2c_write(aw869xx, AW869XX_REG_RTPCFG1, aw869xx_cont->data[0 + shift]);
    /*ADDRL*/
    aw869xx_i2c_write(aw869xx, AW869XX_REG_RTPCFG2, aw869xx_cont->data[1 + shift]);
    /* FIFO_AEH */
    aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_RTPCFG3, AW869XX_BIT_RTPCFG3_FIFO_AEH_MASK,
                   (unsigned char)(((aw869xx->ram.base_addr >> 1) >> 4) & 0xF0));
    /* FIFO AEL */
    aw869xx_i2c_write(aw869xx, AW869XX_REG_RTPCFG4,
              (unsigned char)((aw869xx->ram.base_addr >> 1) & 0x00FF));
    /* FIFO_AFH */
    aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_RTPCFG3,AW869XX_BIT_RTPCFG3_FIFO_AFH_MASK,
                   (unsigned char)(((aw869xx->ram.base_addr -(aw869xx->ram.base_addr >> 2)) >> 8) & 0x0F));
    /* FIFO_AFL */
    aw869xx_i2c_write(aw869xx, AW869XX_REG_RTPCFG5,
              (unsigned char)(((aw869xx->ram.base_addr - (aw869xx->ram.base_addr >> 2)) & 0x00FF)));
    /*
    *   unsigned int temp
    *   HIGH<byte4 byte3 byte2 byte1>LOW
    *   |_ _ _ _AF-12BIT_ _ _ _AE-12BIT|
    */
    aw869xx_i2c_read(aw869xx, AW869XX_REG_RTPCFG3, &reg_val);
    temp = ((reg_val & 0x0f) << 24) | ((reg_val & 0xf0) << 4);
    aw869xx_i2c_read(aw869xx, AW869XX_REG_RTPCFG4, &reg_val);
    temp = temp | reg_val;
    dev_info(aw869xx->dev, "%s: almost_empty_threshold = %d\n", __func__, (unsigned short)temp);
    aw869xx_i2c_read(aw869xx, AW869XX_REG_RTPCFG5, &reg_val);
    temp = temp | (reg_val << 16);
    dev_info(aw869xx->dev, "%s: almost_full_threshold = %d\n", __func__, temp >> 16);
    /* ram */
    shift = aw869xx->ram.baseaddr_shift;
    aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_RAMADDRH, AW869XX_BIT_RAMADDRH_MASK,
                   aw869xx_cont->data[0 + shift]);
    aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMADDRL, aw869xx_cont->data[1 + shift]);
    shift = aw869xx->ram.ram_shift;
    for (i = shift; i < aw869xx_cont->len; i++) {
        aw869xx->ram_update_flag = aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMDATA, aw869xx_cont->data[i]);
    }

#ifdef AW_CHECK_RAM_DATA
	shift = aw869xx->ram.baseaddr_shift;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_RAMADDRH, AW869XX_BIT_RAMADDRH_MASK,
			       aw869xx_cont->data[0 + shift]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMADDRL, aw869xx_cont->data[1 + shift]);
	shift = aw869xx->ram.ram_shift;
	for (i = shift; i < aw869xx_cont->len; i++) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_RAMDATA, &reg_val);
		dev_info(aw869xx->dev, "%s aw869xx_cont->data=0x%02X, ramdata=0x%02X\n",
		           __func__,aw869xx_cont->data[i],reg_val);
		if (reg_val != aw869xx_cont->data[i]) {
			dev_err(aw869xx->dev, "%s: ram check error addr=0x%04x, file_data=0x%02X, ram_data=0x%02X\n",
				   __func__, i, aw869xx_cont->data[i], reg_val);
			return;
		}
	}
#endif
	/* RAMINIT Disable */
	aw869xx_haptic_raminit(aw869xx, false);
	mutex_unlock(&aw869xx->lock);
	pr_info("%s exit\n", __func__);
}


static void aw869xx_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw869xx *aw869xx = context;
	struct aw869xx_container *aw869xx_fw;
	int i = 0;
	unsigned short check_sum = 0;

	if (!cont) {
		pr_err("%s: failed to read [%s]\n", __func__, aw869xx_ram_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s enter: loaded [%s] - size: %zu\n", __func__, aw869xx_ram_name,
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
		aw869xx->ram.check_sum = check_sum;
	}

	/* aw869xx ram update less then 128kB */
	aw869xx_fw = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw869xx_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw869xx_fw->len = cont->size;
	memcpy(aw869xx_fw->data, cont->data, cont->size);
	release_firmware(cont);
	aw869xx_container_update(aw869xx, aw869xx_fw);
	aw869xx->ram.len = aw869xx_fw->len;
	kfree(aw869xx_fw);
	aw869xx->ram_init = 1;
	pr_info("%s: ram firmware update complete\n", __func__);

	//aw869xx_haptic_trig_enable_config(aw869xx);
	//aw869xx_rtp_update(aw869xx);
}


static int aw869xx_ram_update(struct aw869xx *aw869xx)
{
	char file_name[128] = {0};

	aw869xx->ram_init = 0;
	aw869xx->rtp_init = 0;

	if (aw869xx->add_suffix) {
		snprintf(file_name, 128, "%s%s", "_", aw869xx_ram_name);
	} else {
		strlcpy(file_name, aw869xx_ram_name, 128);
	}

	pr_info("%s enter, file_name: %s\n", __func__, file_name);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				file_name, aw869xx->dev, GFP_KERNEL,
				aw869xx, aw869xx_ram_loaded);
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static void aw869xx_ram_work_routine(struct work_struct *work)
{
	struct aw869xx *aw869xx = container_of(work, struct aw869xx, ram_work.work);

	pr_info("%s enter\n", __func__);

	aw869xx_ram_update(aw869xx);

}
#endif

static int aw869xx_ram_init(struct aw869xx *aw869xx)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = 5000;
	INIT_DELAYED_WORK(&aw869xx->ram_work, aw869xx_ram_work_routine);
	schedule_delayed_work(&aw869xx->ram_work, msecs_to_jiffies(ram_timer_val));
#else
	aw869xx_ram_update(aw869xx);
#endif
	return 0;
}


/********************************************************************************************
 *
 * base haptic control
 *
 ********************************************************************************************/
 static void aw869xx_haptic_play_go(struct aw869xx *aw869xx)
{
	aw869xx_i2c_write(aw869xx, AW869XX_REG_PLAYCFG4,
			  AW869XX_BIT_PLAYCFG4_GO_ON);
}

static int aw869xx_haptic_stop(struct aw869xx *aw869xx)
{
	unsigned char cnt = 40;
	unsigned char reg_val = 0;
	bool force_flag = true;

	dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->play_mode = AW869XX_HAPTIC_STANDBY_MODE;
	aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
	if ((reg_val & 0x0f) == AW869XX_BIT_GLBRD5_STATE_STANDBY ||
	    (reg_val & 0x0f) == AW869XX_BIT_GLBRD5_STATE_I2S_GO) {
		force_flag = false;
		dev_info(aw869xx->dev, "%s already in standby mode! glb_state=0x%02X\n",
			    __func__, reg_val);
	} else {
		aw869xx_i2c_write(aw869xx, AW869XX_REG_PLAYCFG4, AW869XX_BIT_PLAYCFG4_STOP_ON);
		while (cnt) {
			aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
			if ((reg_val & 0x0f) == AW869XX_BIT_GLBRD5_STATE_STANDBY
			    || (reg_val & 0x0f) == AW869XX_BIT_GLBRD5_STATE_I2S_GO) {
				cnt = 0;
				force_flag = false;
				dev_info(aw869xx->dev, "%s entered standby! glb_state=0x%02X\n",
					    __func__, reg_val);
			} else {
				cnt--;
				pr_debug("%s wait for standby, glb_state=0x%02X\n", __func__, reg_val);
			}
			usleep_range(2000, 2500);
		}
	}
	if (force_flag) {
		dev_err(aw869xx->dev, "%s force to enter standby mode!\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2, AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
				       AW869XX_BIT_SYSCTRL2_STANDBY_ON);
        aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2, AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
				       AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
	}
	return 0;
}

static void aw869xx_haptic_softreset(struct aw869xx *aw869xx)
{
	dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_ID, 0xAA);
	usleep_range(3000, 3500);
}

static int aw869xx_haptic_play_mode(struct aw869xx *aw869xx,
				    unsigned char play_mode)
{
	dev_info(aw869xx->dev, "%s enter\n", __func__);

	switch (play_mode) {
	case AW869XX_HAPTIC_STANDBY_MODE:
		dev_info(aw869xx->dev, "%s: enter standby mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_STANDBY_MODE;
		aw869xx_haptic_stop(aw869xx);
		break;
	case AW869XX_HAPTIC_RAM_MODE:
		dev_info(aw869xx->dev, "%s: enter ram mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_RAM_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw869xx_haptic_bst_mode_config(aw869xx,
					       AW869XX_HAPTIC_BST_MODE_BOOST);
		break;
	case AW869XX_HAPTIC_RAM_LOOP_MODE:
		dev_info(aw869xx->dev, "%s: enter ram loop mode\n",
			    __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_RAM_LOOP_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw869xx_haptic_bst_mode_config(aw869xx,
					       AW869XX_HAPTIC_BST_MODE_BYPASS);
		break;
	case AW869XX_HAPTIC_RTP_MODE:
		dev_info(aw869xx->dev, "%s: enter rtp mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_RTP_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_RTP);
		/* bst mode */
		aw869xx_haptic_bst_mode_config(aw869xx,
					       AW869XX_HAPTIC_BST_MODE_BOOST);
		break;
	case AW869XX_HAPTIC_TRIG_MODE:
		dev_info(aw869xx->dev, "%s: enter trig mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_TRIG_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW869XX_HAPTIC_CONT_MODE:
		dev_info(aw869xx->dev, "%s: enter cont mode\n", __func__);
		aw869xx->play_mode = AW869XX_HAPTIC_CONT_MODE;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW869XX_BIT_PLAYCFG3_PLAY_MODE_CONT);
		/* bst mode */
		aw869xx_haptic_bst_mode_config(aw869xx,
					       AW869XX_HAPTIC_BST_MODE_BYPASS);
		break;
	default:
		dev_err(aw869xx->dev, "%s: play mode %d error",
			   __func__, play_mode);
		break;
	}
	return 0;
}

static int aw869xx_haptic_set_wav_seq(struct aw869xx *aw869xx,
				      unsigned char wav, unsigned char seq)
{
	aw869xx_i2c_write(aw869xx, AW869XX_REG_WAVCFG1 + wav, seq);
	return 0;
}

static int aw869xx_haptic_set_wav_loop(struct aw869xx *aw869xx,
				       unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_WAVCFG9 + (wav / 2),
				       AW869XX_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_WAVCFG9 + (wav / 2),
				       AW869XX_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
	return 0;
}

/*
static int aw869xx_haptic_set_main_loop(struct aw869xx *aw869xx,
				       unsigned char loop)
{
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_WAVCFG13,
				AW869XX_BIT_WAVCFG13_MAINLOOP_MASK, loop);
	return 0;
}
*/


static void aw869xx_haptic_set_repeat_wav_seq(struct aw869xx *aw869xx,
					      unsigned char seq)
{
	aw869xx_haptic_set_wav_seq(aw869xx, 0x00, seq);
	aw869xx_haptic_set_wav_loop(aw869xx, 0x00,
				    AW869XX_BIT_WAVLOOP_INIFINITELY);
}

static int aw869xx_haptic_set_bst_vol(struct aw869xx *aw869xx,
				      unsigned char bst_vol)
{
	if (bst_vol & 0xc0)
		bst_vol = 0x3f;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG1,
			       AW869XX_BIT_PLAYCFG1_BST_VOUT_RDA_MASK, bst_vol);
	return 0;
}

static int aw869xx_haptic_set_bst_peak_cur(struct aw869xx *aw869xx)
{
	switch (aw869xx->bst_pc) {
	case AW869XX_HAPTIC_BST_PC_L1:
		dev_info(aw869xx->dev, "%s bst pc = L1\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_BSTCFG1,
				       AW869XX_BIT_BSTCFG1_BST_PC_MASK,
				       (0 << 1));
		return 0;
	case AW869XX_HAPTIC_BST_PC_L2:
		dev_info(aw869xx->dev, "%s bst pc = L2\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_BSTCFG1,
				       AW869XX_BIT_BSTCFG1_BST_PC_MASK,
				       (5 << 1));
		return 0;
	default:
		dev_info(aw869xx->dev, "%s bst pc = L1\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_BSTCFG1,
				       AW869XX_BIT_BSTCFG1_BST_PC_MASK,
				       (0 << 1));
		break;
	}
	return 0;
}

static void aw869xx_haptic_set_gain(struct aw869xx *aw869xx, unsigned char gain)
{
	aw869xx_i2c_write(aw869xx, AW869XX_REG_PLAYCFG2, gain);
}

static int aw869xx_haptic_set_pwm(struct aw869xx *aw869xx, unsigned char mode)
{
	switch (mode) {
	case AW869XX_PWM_48K:
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW869XX_PWM_24K:
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW869XX_PWM_12K:
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
	return 0;
}


static int aw869xx_haptic_play_wav_seq(struct aw869xx *aw869xx,
				       unsigned char flag)
{
#ifdef AW_RAM_STATE_OUTPUT
	int i;
	unsigned char reg_val = 0;
#endif

	dev_info(aw869xx->dev, "%s enter\n", __func__);
	if (flag) {
		aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RAM_MODE);
		aw869xx_haptic_play_go(aw869xx);
#ifdef AW_RAM_STATE_OUTPUT
		for (i = 0; i < 100; i++) {
			aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
			if ((reg_val & 0x0f) == 0x07) {
				dev_info(aw869xx->dev, "%s RAM_GO! glb_state=0x07\n",
					    __func__);
			} else {
				dev_dbg(aw869xx->dev, "%s ram stopped, glb_state=0x%02X\n",
					   __func__, reg_val);
			}
			usleep_range(2000, 2500);

		}
#endif
	}
	return 0;
}

static int aw869xx_haptic_play_repeat_seq(struct aw869xx *aw869xx,
					  unsigned char flag)
{
	dev_info(aw869xx->dev, "%s enter\n", __func__);

	if (flag) {
		aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RAM_LOOP_MODE);
		aw869xx_haptic_play_go(aw869xx);
	}
	return 0;
}

/*****************************************************
 *
 * motor protect
 *
 *****************************************************/
static int aw869xx_haptic_swicth_motor_protect_config(struct aw869xx *aw869xx,
						      unsigned char addr,
						      unsigned char val)
{
	dev_info(aw869xx->dev, "%s enter\n", __func__);
	if (addr == 1) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG1,
				       AW869XX_BIT_DETCFG1_PRCT_MODE_MASK,
				       AW869XX_BIT_DETCFG1_PRCT_MODE_VALID);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG1,
				       AW869XX_BIT_PWMCFG1_PRC_EN_MASK,
				       AW869XX_BIT_PWMCFG1_PRC_ENABLE);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG3,
				       AW869XX_BIT_PWMCFG3_PR_EN_MASK,
				       AW869XX_BIT_PWMCFG3_PR_ENABLE);
	} else if (addr == 0) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG1,
				       AW869XX_BIT_DETCFG1_PRCT_MODE_MASK,
				       AW869XX_BIT_DETCFG1_PRCT_MODE_INVALID);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG1,
				       AW869XX_BIT_PWMCFG1_PRC_EN_MASK,
				       AW869XX_BIT_PWMCFG1_PRC_DISABLE);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG3,
				       AW869XX_BIT_PWMCFG3_PR_EN_MASK,
				       AW869XX_BIT_PWMCFG3_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG1,
				       AW869XX_BIT_PWMCFG1_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PWMCFG3,
				       AW869XX_BIT_PWMCFG3_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw869xx_i2c_write(aw869xx, AW869XX_REG_PWMCFG4, val);
	}
	return 0;
}


/*****************************************************
 *
 * offset calibration
 *
 *****************************************************/
static int aw869xx_haptic_offset_calibration(struct aw869xx *aw869xx)
{
	unsigned int cont = 2000;
	unsigned char reg_val = 0;

	dev_info(aw869xx->dev, "%s enter\n", __func__);

	aw869xx_haptic_raminit(aw869xx, true);

	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG2,
			       AW869XX_BIT_DETCFG2_DIAG_GO_MASK,
			       AW869XX_BIT_DETCFG2_DIAG_GO_ON);
	while (1) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_DETCFG2, &reg_val);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}
	if (cont == 0)
		dev_err(aw869xx->dev, "%s calibration offset failed!\n",
			   __func__);
	aw869xx_haptic_raminit(aw869xx, false);
	return 0;
}

/*****************************************************
 *
 * trig config
 *
 *****************************************************/
static int aw869xx_haptic_trig_param_init(struct aw869xx *aw869xx)
{
	dev_info(aw869xx->dev, "%s enter\n", __func__);

	aw869xx->trig[0].trig_level = aw869xx->info.trig_config[0];
	aw869xx->trig[0].trig_polar = aw869xx->info.trig_config[1];
	aw869xx->trig[0].pos_enable = aw869xx->info.trig_config[2];
	aw869xx->trig[0].pos_sequence = aw869xx->info.trig_config[3];
	aw869xx->trig[0].neg_enable = aw869xx->info.trig_config[4];
	aw869xx->trig[0].neg_sequence = aw869xx->info.trig_config[5];
	aw869xx->trig[0].trig_brk = aw869xx->info.trig_config[6];
	aw869xx->trig[0].trig_bst = aw869xx->info.trig_config[7];
	if (!aw869xx->info.is_enabled_i2s) {
		dev_info(aw869xx->dev, "%s i2s is disabled!\n", __func__);
		aw869xx->trig[1].trig_level = aw869xx->info.trig_config[8 + 0];
		aw869xx->trig[1].trig_polar = aw869xx->info.trig_config[8 + 1];
		aw869xx->trig[1].pos_enable = aw869xx->info.trig_config[8 + 2];
		aw869xx->trig[1].pos_sequence =
		    aw869xx->info.trig_config[8 + 3];
		aw869xx->trig[1].neg_enable = aw869xx->info.trig_config[8 + 4];
		aw869xx->trig[1].neg_sequence =
		    aw869xx->info.trig_config[8 + 5];
		aw869xx->trig[1].trig_brk = aw869xx->info.trig_config[8 + 6];
		aw869xx->trig[1].trig_bst = aw869xx->info.trig_config[8 + 7];

		aw869xx->trig[2].trig_level = aw869xx->info.trig_config[16 + 0];
		aw869xx->trig[2].trig_polar = aw869xx->info.trig_config[16 + 1];
		aw869xx->trig[2].pos_enable = aw869xx->info.trig_config[16 + 2];
		aw869xx->trig[2].pos_sequence =
		    aw869xx->info.trig_config[16 + 3];
		aw869xx->trig[2].neg_enable = aw869xx->info.trig_config[16 + 4];
		aw869xx->trig[2].neg_sequence =
		    aw869xx->info.trig_config[16 + 5];
		aw869xx->trig[2].trig_brk = aw869xx->info.trig_config[16 + 6];
		aw869xx->trig[2].trig_bst = aw869xx->info.trig_config[16 + 7];
	}
	return 0;
}


static int aw869xx_haptic_trig_param_config(struct aw869xx *aw869xx)
{
	dev_info(aw869xx->dev, "%s enter\n", __func__);

	if (aw869xx->trig[0].trig_level) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_MODE_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_MODE_LEVEL);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_MODE_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_MODE_EDGE);
	}
	if (aw869xx->trig[1].trig_level) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_MODE_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_MODE_LEVEL);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_MODE_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_MODE_EDGE);
	}
	if (aw869xx->trig[2].trig_level) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_MODE_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_MODE_LEVEL);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_MODE_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_MODE_EDGE);
	}
    
	if (aw869xx->trig[0].trig_polar) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_POLAR_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_POLAR_NEG);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_POLAR_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_POLAR_POS);
	}
	if (aw869xx->trig[1].trig_polar) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_POLAR_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_POLAR_NEG);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_POLAR_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_POLAR_POS);
	}
	if (aw869xx->trig[2].trig_polar) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG8_TRG3_POLAR_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_POLAR_NEG);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG8_TRG3_POLAR_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_POLAR_POS);
	}

	if (aw869xx->trig[0].pos_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG1,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG1,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[1].pos_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG2,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG2,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[2].pos_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG3,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG3,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}

	if (aw869xx->trig[0].neg_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG4,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG4,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[1].neg_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG5,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG5,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[2].neg_enable) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG6,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG6,
				       AW869XX_BIT_TRG_ENABLE_MASK,
				       AW869XX_BIT_TRG_DISABLE);
	}
	if (aw869xx->trig[0].pos_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG1,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[0].pos_sequence);
	}
	if (aw869xx->trig[0].neg_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG4,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[0].neg_sequence);
	}
	if (aw869xx->trig[1].pos_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG2,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[1].pos_sequence);
	}
	if (aw869xx->trig[1].neg_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG5,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[1].neg_sequence);
	}
	if (aw869xx->trig[2].pos_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG3,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[2].pos_sequence);
	}
	if (aw869xx->trig[2].neg_sequence) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG6,
				       AW869XX_BIT_TRG_SEQ_MASK,
				       aw869xx->trig[2].neg_sequence);
	}
	if (aw869xx->trig[0].trig_brk) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE);
	}
	if (aw869xx->trig[1].trig_brk) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE);
	}
	if (aw869xx->trig[2].trig_brk) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE);
	}
	if (aw869xx->trig[0].trig_bst) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_BST_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_BST_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG1_BST_MASK,
				       AW869XX_BIT_TRGCFG7_TRG1_BST_DISABLE);
	}
	if (aw869xx->trig[1].trig_bst) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_BST_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_BST_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG7,
				       AW869XX_BIT_TRGCFG7_TRG2_BST_MASK,
				       AW869XX_BIT_TRGCFG7_TRG2_BST_DISABLE);
	}
	if (aw869xx->trig[2].trig_bst) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_BST_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_BST_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRGCFG8,
				       AW869XX_BIT_TRGCFG8_TRG3_BST_MASK,
				       AW869XX_BIT_TRGCFG8_TRG3_BST_DISABLE);
	}
	return 0;
}

/*
static int aw869xx_haptic_trig_enable_config(struct aw869xx *aw869xx)
{

	if (!aw869xx->no_trigger) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRG_CFG2,
			AW869XX_BIT_TRGCFG2_TRG1_ENABLE_MASK, aw869xx->trig[0].enable ? AW869XX_BIT_TRGCFG2_TRG1_ENABLE:AW869XX_BIT_TRGCFG2_TRG1_DISABLE);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRG_CFG2,
			AW869XX_BIT_TRGCFG2_TRG2_ENABLE_MASK, aw869xx->trig[1].enable ? AW869XX_BIT_TRGCFG2_TRG2_ENABLE:AW869XX_BIT_TRGCFG2_TRG2_DISABLE);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRG_CFG2,
			AW869XX_BIT_TRGCFG2_TRG3_ENABLE_MASK, aw869xx->trig[2].enable ? AW869XX_BIT_TRGCFG2_TRG3_ENABLE:AW869XX_BIT_TRGCFG2_TRG3_DISABLE);
	}
	return 0;
}
*/

static void aw869xx_haptic_bst_mode_config(struct aw869xx *aw869xx,
					   unsigned char boost_mode)
{
	aw869xx->boost_mode = boost_mode;

	switch (boost_mode) {
	case AW869XX_HAPTIC_BST_MODE_BOOST:
		dev_info(aw869xx->dev, "%s haptic boost mode = boost\n",
			    __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG1,
				       AW869XX_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW869XX_BIT_PLAYCFG1_BST_MODE_BOOST);
		break;
	case AW869XX_HAPTIC_BST_MODE_BYPASS:
		dev_info(aw869xx->dev, "%s haptic boost mode = bypass\n",
			    __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG1,
				       AW869XX_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW869XX_BIT_PLAYCFG1_BST_MODE_BYPASS);
		break;
	default:
		dev_err(aw869xx->dev, "%s: boost_mode = %d error",
			   __func__, boost_mode);
		break;
	}
}

static int aw869xx_haptic_auto_brk_enable(struct aw869xx *aw869xx,
					  unsigned char flag)
{
	aw869xx->auto_brake = flag;
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
				       AW869XX_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
				       AW869XX_BIT_PLAYCFG3_BRK_DISABLE);
	}
	return 0;
}

static int aw869xx_haptic_auto_bst_enable(struct aw869xx *aw869xx,
					  unsigned char flag)
{
	aw869xx->auto_boost = flag;
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_AUTO_BST_MASK,
				       AW869XX_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
				       AW869XX_BIT_PLAYCFG3_AUTO_BST_MASK,
				       AW869XX_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
	return 0;
}


/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static int aw869xx_haptic_vbat_mode_config(struct aw869xx *aw869xx,
					   unsigned char flag)
{
	if (flag == AW869XX_HAPTIC_CONT_VBAT_HW_ADJUST_MODE) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL1,
				       AW869XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL1_VBAT_MODE_HW);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL1,
				       AW869XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
				       AW869XX_BIT_SYSCTRL1_VBAT_MODE_SW);
	}
	return 0;
}


static int aw869xx_haptic_get_vbat(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;
	unsigned int vbat_code = 0;

	aw869xx_haptic_stop(aw869xx);
	aw869xx_haptic_raminit(aw869xx, true);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG2,
			       AW869XX_BIT_DETCFG2_VBAT_GO_MASK,
			       AW869XX_BIT_DETCFG2_VABT_GO_ON);
	usleep_range(20000, 25000);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_DET_VBAT, &reg_val);
	vbat_code = (vbat_code | reg_val) << 2;
	aw869xx_i2c_read(aw869xx, AW869XX_REG_DET_LO, &reg_val);
	vbat_code = vbat_code | ((reg_val & 0x30) >> 4);
	aw869xx->vbat = 6100 * vbat_code / 1024;
	if (aw869xx->vbat > AW869XX_VBAT_MAX) {
		aw869xx->vbat = AW869XX_VBAT_MAX;
		dev_info(aw869xx->dev, "%s vbat max limit = %dmV\n",
			    __func__, aw869xx->vbat);
	}
	if (aw869xx->vbat < AW869XX_VBAT_MIN) {
		aw869xx->vbat = AW869XX_VBAT_MIN;
		dev_info(aw869xx->dev, "%s vbat min limit = %dmV\n",
			    __func__, aw869xx->vbat);
	}
	dev_info(aw869xx->dev, "%s aw869xx->vbat=%dmV, vbat_code=0x%02X\n",
		    __func__, aw869xx->vbat, vbat_code);
	aw869xx_haptic_raminit(aw869xx, false);
	return 0;
}


/* if flag is true, Long vibration voltage compensation */
static int aw869xx_haptic_ram_vbat_comp(struct aw869xx *aw869xx, bool flag)
{
	int temp_gain = 0;

	if (flag) {
		if (aw869xx->ram_vbat_comp == AW869XX_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			aw869xx_haptic_get_vbat(aw869xx);
			temp_gain = aw869xx->gain * AW869XX_VBAT_REFER / aw869xx->vbat;
			if (temp_gain > (128*AW869XX_VBAT_REFER/AW869XX_VBAT_MIN)) {
				temp_gain = 128*AW869XX_VBAT_REFER/AW869XX_VBAT_MIN;
				pr_debug("%s gain limit=%d\n", __func__, temp_gain);
			}
			aw869xx_haptic_set_gain(aw869xx, temp_gain);
		} else {
			aw869xx_haptic_set_gain(aw869xx, aw869xx->gain);
		}
	} else {
		aw869xx_haptic_set_gain(aw869xx, aw869xx->gain);
	}

	return 0;
}

static int aw869xx_haptic_get_lra_resistance(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;
	unsigned char d2s_gain_temp = 0;
	unsigned int lra_code = 0;

	aw869xx_haptic_stop(aw869xx);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSCTRL7, &reg_val);
	d2s_gain_temp = 0x07 & reg_val;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7, AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       aw869xx->info.d2s_gain);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSCTRL7, &reg_val);
	dev_info(aw869xx->dev, "%s: d2s_gain=%d\n", __func__, 0x07 & reg_val);

	aw869xx_haptic_raminit(aw869xx, true);
	/* enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	usleep_range(2000, 2500);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2, AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
			       AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG1, AW869XX_BIT_DETCFG1_RL_OS_MASK,
			       AW869XX_BIT_DETCFG1_RL);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_DETCFG2, AW869XX_BIT_DETCFG2_DIAG_GO_MASK,
			       AW869XX_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(30000, 35000);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_DET_RL, &reg_val);
	lra_code = (lra_code | reg_val) << 2;
	aw869xx_i2c_read(aw869xx, AW869XX_REG_DET_LO, &reg_val);
	lra_code = lra_code | (reg_val & 0x03);
	/* 2num */
	aw869xx->lra = (lra_code * 678 * 1000) / (1024 * 10);
	aw869xx_haptic_raminit(aw869xx, false);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7, AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       d2s_gain_temp);
	return 0;
}

static void aw869xx_haptic_misc_para_init(struct aw869xx *aw869xx)
{

	dev_info(aw869xx->dev, "%s enter\n", __func__);

	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG1,
			  aw869xx->info.bstcfg[0]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG2,
			  aw869xx->info.bstcfg[1]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG3,
			  aw869xx->info.bstcfg[2]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG4,
			  aw869xx->info.bstcfg[3]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_BSTCFG5,
			  aw869xx->info.bstcfg[4]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_SYSCTRL3,
			  aw869xx->info.sine_array[0]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_SYSCTRL4,
			  aw869xx->info.sine_array[1]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_SYSCTRL5,
			  aw869xx->info.sine_array[2]);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_SYSCTRL6,
			  aw869xx->info.sine_array[3]);
	/* d2s_gain */
	if (!aw869xx->info.d2s_gain)
		dev_err(aw869xx->dev, "%s aw869xx->info.d2s_gain = 0!\n",
			   __func__);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       aw869xx->info.d2s_gain);

	/* cont_tset */
	if (!aw869xx->info.cont_tset)
		dev_err(aw869xx->dev,
			   "%s aw869xx->info.cont_tset = 0!\n", __func__);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG13,
			       AW869XX_BIT_CONTCFG13_TSET_MASK,
			       aw869xx->info.cont_tset << 4);

	/* cont_bemf_set */
	if (!aw869xx->info.cont_bemf_set)
		dev_err(aw869xx->dev,
			   "%s aw869xx->info.cont_bemf_set = 0!\n", __func__);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG13,
			       AW869XX_BIT_CONTCFG13_BEME_SET_MASK,
			       aw869xx->info.cont_bemf_set);

	/* cont_brk_time */
	if (!aw869xx->info.cont_brk_time)
		dev_err(aw869xx->dev,
			   "%s aw869xx->info.cont_brk_time = 0!\n", __func__);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG10,
			  aw869xx->info.cont_brk_time);

	/* cont_bst_brk_gain */
	if (!aw869xx->info.cont_bst_brk_gain)
		dev_err(aw869xx->dev,
			   "%s aw869xx->info.cont_bst_brk_gain = 0!\n",
			   __func__);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG5,
			       AW869XX_BIT_CONTCFG5_BST_BRK_GAIN_MASK,
			       aw869xx->info.cont_bst_brk_gain);

	/* cont_brk_gain */
	if (!aw869xx->info.cont_brk_gain)
		dev_err(aw869xx->dev,
			   "%s aw869xx->info.cont_brk_gain = 0!\n", __func__);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG5,
			       AW869XX_BIT_CONTCFG5_BRK_GAIN_MASK,
			       aw869xx->info.cont_brk_gain);

	/* i2s enbale */
	if (aw869xx->info.is_enabled_i2s) {
		dev_info(aw869xx->dev, "%s i2s enabled!\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
				       AW869XX_BIT_SYSCTRL2_I2S_PIN_I2S);
	} else {
		dev_info(aw869xx->dev, "%s i2s disabled!\n", __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL2,
				       AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
				       AW869XX_BIT_SYSCTRL2_I2S_PIN_TRIG);
	}
}

static void aw869xx_double_click_switch(struct aw869xx *aw869xx, bool sw)
{
	if (sw) {
		aw869xx_haptic_set_wav_seq(aw869xx, 0x00, 0x01);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_WAVCFG13, AW869XX_BIT_WAVCFG13_WAITSLOT_MASK, 
			AW869XX_BIT_WAVCFG13_WAITSLOT_DIV_64);
		aw869xx_haptic_set_wav_seq(aw869xx, 0x01, ((AW869XX_DUOBLE_CLICK_DELTA / SEQ_WAIT_UNIT) & 0x7f) | 0x80);
		aw869xx_haptic_set_wav_seq(aw869xx, 0x02, 0x01);
	} else {
		aw869xx_haptic_set_wav_seq(aw869xx, 0x00, 0x00);
		aw869xx_haptic_set_wav_seq(aw869xx, 0x01, 0x00);
		aw869xx_haptic_set_wav_seq(aw869xx, 0x02, 0x00);
	}
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw869xx_haptic_set_rtp_aei(struct aw869xx *aw869xx, bool flag)
{
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM, AW869XX_BIT_SYSINTM_FF_AEM_MASK,
				       AW869XX_BIT_SYSINTM_FF_AEM_ON);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM, AW869XX_BIT_SYSINTM_FF_AEM_MASK,
				       AW869XX_BIT_SYSINTM_FF_AEM_OFF);
	}
}

/*
static void aw869xx_haptic_set_rtp_afi(struct aw869xx *aw869xx, bool flag)
{
	if (flag) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
				AW869XX_BIT_SYSINTM_FF_AF_MASK, AW869XX_BIT_SYSINTM_FF_AF_EN);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM,
				AW869XX_BIT_SYSINTM_FF_AF_MASK, AW869XX_BIT_SYSINTM_FF_AF_OFF);
	}
}
*/
/*
static unsigned char aw869xx_haptic_rtp_get_fifo_aei(struct aw869xx *aw869xx)
{
	unsigned char ret;
	unsigned char reg_val;


	if (aw869xx->osc_cali_flag == 1) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST, &reg_val);
		reg_val &= AW869XX_BIT_SYSST_FF_AES;
		ret = reg_val>>4;
	} else {
	  aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	  reg_val &= AW869XX_BIT_SYSINT_FF_AEI;
	  ret = reg_val>>4;
	}

	return ret;
}
*/
/*
static int aw869xx_haptic_rtp_get_fifo_afi(struct aw869xx *aw869xx)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;
	int rc = 0;


	if (aw869xx->osc_cali_flag == 1) {
		rc = aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST, &reg_val);
		if (rc < 0) {
			pr_info("%s failed, ret=%d, reg_val=%d", __func__, ret, reg_val);
			return -EBUSY;
		}
		reg_val &= AW869XX_BIT_SYSST_FF_AFS;
		ret = reg_val>>3;
	} else {
		rc = aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
		if (rc < 0) {
			pr_info("%s failed, ret=%d, reg_val=%d", __func__, ret, reg_val);
			return -EBUSY;
		}

		reg_val &= AW869XX_BIT_SYSINT_FF_AFI;
		ret = reg_val>>3;
	}

	return ret;
}
*/
//vivo zhangxiaodong add for rtp int begin

/* 通过SYSST状态寄存器的fifo满bit位来判断fifo是否满，代替通过SYSINT的fifo满bit位
 * 防止读SYSINT寄存器判断fifo是否满的时候，同时将fifo空的bit位清除掉了
 */

static int aw869xx_haptic_rtp_get_fifo_afs(struct aw869xx *aw869xx)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;
	int rc = 0;

	rc = aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST, &reg_val);
	if (rc < 0) {
		pr_info("%s failed, ret=%d, reg_val=%#x", __func__, ret, reg_val);
		return -EBUSY;
	}
	reg_val &= AW869XX_BIT_SYSST_FF_AFS;
	ret = reg_val >> 3;

	return ret; //0x01表示fifo满，0x00表示fifo空，负数表示i2c读取错误
}


//vivo zhangxiaodong add end

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static int aw869xx_haptic_rtp_init(struct aw869xx *aw869xx)
{
	unsigned int buf_len = 0;
	int ret = 0, retval = 0;

	pr_info("%s enter\n", __func__);
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);

	mutex_lock(&aw869xx->rtp_lock);
	aw869xx->rtp_cnt = 0;
	retval = aw869xx_haptic_rtp_get_fifo_afs(aw869xx);
	if ((!retval) &&
		(aw869xx->play_mode == AW869XX_HAPTIC_RTP_MODE)) {
		pr_info("%s rtp cnt = %d\n", __func__, aw869xx->rtp_cnt);

		if (aw869xx_rtp->len > aw869xx->rtp_cnt) {
			if ((aw869xx_rtp->len - aw869xx->rtp_cnt) < (aw869xx->ram.base_addr)) {
				buf_len = aw869xx_rtp->len-aw869xx->rtp_cnt;
			} else {
				buf_len = (aw869xx->ram.base_addr);//2k 数据写入
			}

			ret = aw869xx_i2c_writes(aw869xx, AW869XX_REG_RTPDATA,
				&aw869xx_rtp->data[aw869xx->rtp_cnt], buf_len);
			if (ret < 0) {
				pr_err("%s: i2c write error: total length: %d, play length: %d\n",
					__func__, aw869xx_rtp->len, aw869xx->rtp_cnt);
				aw869xx->rtp_cnt = 0;
				mutex_unlock(&aw869xx->rtp_lock);
				pm_qos_remove_request(&pm_qos_req_vb);
				return -EBUSY;
			}
			aw869xx->rtp_cnt += buf_len;
		}

		if (aw869xx->rtp_cnt >= aw869xx_rtp->len) {
			pr_err("%s complete: total length: %d, play length: %d\n", __func__,
				aw869xx_rtp->len, aw869xx->rtp_cnt);
			aw869xx->rtp_cnt = 0;
			mutex_unlock(&aw869xx->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}

	if (retval < 0) {
		pr_err("%s: i2c read error: total length: %d, play length: %d\n",
			__func__, aw869xx_rtp->len, aw869xx->rtp_cnt);
		//pr_err("%s retval return negtive, retval=%d\n", __func__, retval);
		aw869xx->rtp_cnt = 0;
		mutex_unlock(&aw869xx->rtp_lock);
		pm_qos_remove_request(&pm_qos_req_vb);
		return 0;
	}


	//vivo zhangxiaodong add end
	while ((!(retval = aw869xx_haptic_rtp_get_fifo_afs(aw869xx))) &&
			(aw869xx->play_mode == AW869XX_HAPTIC_RTP_MODE)) {
		pr_info("%s rtp cnt = %d\n", __func__, aw869xx->rtp_cnt);

		if (aw869xx_rtp->len > aw869xx->rtp_cnt) {
			if ((aw869xx_rtp->len-aw869xx->rtp_cnt) < (aw869xx->ram.base_addr>>2)) {
				buf_len = aw869xx_rtp->len-aw869xx->rtp_cnt;
			} else {
				buf_len = (aw869xx->ram.base_addr>>2);
			}

			ret = aw869xx_i2c_writes(aw869xx, AW869XX_REG_RTPDATA,
				&aw869xx_rtp->data[aw869xx->rtp_cnt], buf_len);
			if (ret < 0) {
				pr_err("%s: i2c write error: total length: %d, play length: %d\n",
					__func__, aw869xx_rtp->len, aw869xx->rtp_cnt);
				aw869xx->rtp_cnt = 0;
				mutex_unlock(&aw869xx->rtp_lock);
				pm_qos_remove_request(&pm_qos_req_vb);
				return 0;
			}

			aw869xx->rtp_cnt += buf_len;
		}

		if (aw869xx->rtp_cnt >= aw869xx_rtp->len) {
			pr_err("%s complete total length: %d, play length: %d\n",
				__func__, aw869xx_rtp->len, aw869xx->rtp_cnt);
			aw869xx->rtp_cnt = 0;
			mutex_unlock(&aw869xx->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}

	if (retval < 0) {
		pr_err("%s: i2c read error--->2: total length: %d, play length: %d\n",
			__func__, aw869xx_rtp->len, aw869xx->rtp_cnt);
		aw869xx->rtp_cnt = 0;
		mutex_unlock(&aw869xx->rtp_lock);
		pm_qos_remove_request(&pm_qos_req_vb);
		return 0;
	}


	if ((aw869xx->play_mode == AW869XX_HAPTIC_RTP_MODE) && (retval >= 0)) {
		pr_err("%s open rtp irq\n", __func__);
		aw869xx_haptic_set_rtp_aei(aw869xx, true);
	}

	pr_info("%s exit\n", __func__);
	mutex_unlock(&aw869xx->rtp_lock);
	pm_qos_remove_request(&pm_qos_req_vb);
	return 0;
}
/*no use
static int aw869xx_set_clock(struct aw869xx *aw869xx, int clock_type)
{
	unsigned char code;

	if (clock_type == AW869XX_HAPTIC_CLOCK_CALI_F0) {
		code = (unsigned char)atomic_read(&aw869xx->f0_freq_cali);
		pr_err("%s f0 clock, value=%d, code=%#x\n", __func__, atomic_read(&aw869xx->f0_freq_cali), code);
		aw869xx_i2c_write(aw869xx, AW869XX_REG_TRIM_LRA, code);

	} else if (clock_type == AW869XX_HAPTIC_CLOCK_CALI_OSC_STANDARD) {
		code = (unsigned char)atomic_read(&aw869xx->standard_osc_freq_cali);
		pr_err("%s osc clock, value=%d, code=%#x\n", __func__, atomic_read(&aw869xx->standard_osc_freq_cali), code);
		aw869xx_i2c_write(aw869xx, AW869XX_REG_TRIM_LRA, code);
	}
	return 0;
}*/

static void aw869xx_haptic_upload_lra(struct aw869xx *aw869xx,
				      unsigned int flag)
{
	switch (flag) {
	case WRITE_ZERO:
		dev_info(aw869xx->dev, "%s write zero to trim_lra!\n",
			    __func__);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
				       0x00);
		break;
	case F0_CALI:
		dev_info(aw869xx->dev,
			    "%s write f0_cali_data to trim_lra = 0x%02X\n",
			    __func__, aw869xx->f0_cali_data);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
				       (char)aw869xx->f0_cali_data);
		break;
	case OSC_CALI:
		dev_info(aw869xx->dev,
			    "%s write osc_cali_data to trim_lra = 0x%02X\n",
			    __func__, aw869xx->osc_cali_data);
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
				       (char)aw869xx->osc_cali_data);
		break;
	default:
		break;
	}
}

static void aw869xx_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int cnt = 200;
	unsigned char reg_val = 0;
	bool rtp_work_flag = false;
	struct aw869xx *aw869xx = container_of(work, struct aw869xx, rtp_work);
	struct aw869xx_play_info *play = &aw869xx->play;
	struct aw869xx_wavefrom_info *effect_list = aw869xx->effect_list;


	pr_info("%s enter\n", __func__);

	if (play->type != RTP_TYPE) {
		dev_err(aw869xx->dev, "new not rtp effect coming\n");
		return;
	}

	/* fw loaded */
	ret = request_firmware(&rtp_file, effect_list[aw869xx->rtp_file_num].rtp_file_name, aw869xx->dev);
	if (ret < 0) {
		pr_err("%s: failed to read [%s]\n", __func__, effect_list[aw869xx->rtp_file_num]);
		return ;
	}

	mutex_lock(&aw869xx->rtp_lock);
	aw869xx->rtp_init = 0;
	aw869xx_haptic_upload_lra(aw869xx, OSC_CALI);
	aw869xx_haptic_set_rtp_aei(aw869xx, false);
	aw869xx_interrupt_clear(aw869xx);
	aw869xx_haptic_stop(aw869xx);

	if (aw869xx_rtp == NULL) {
		aw869xx_rtp = devm_kzalloc(aw869xx->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);//TODO
		if (aw869xx_rtp == NULL) {
			pr_err("%s devm kzalloc failed\n", __func__);
			release_firmware(rtp_file);
			mutex_unlock(&aw869xx->rtp_lock);
			return;
		}
	}
	memset(aw869xx_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));


	pr_info("%s: rtp file [%s] size = %d\n", __func__,
			effect_list[aw869xx->rtp_file_num].rtp_file_name, rtp_file->size);

	if (rtp_file->size < RTP_BIN_MAX_SIZE)
		aw869xx_rtp->len = rtp_file->size;
	else
		aw869xx_rtp->len = RTP_BIN_MAX_SIZE;
	memcpy(aw869xx_rtp->data, rtp_file->data, aw869xx_rtp->len);
	release_firmware(rtp_file);

	aw869xx->rtp_init = 1;
	aw869xx->rtp_cnt = 0;
	mutex_unlock(&aw869xx->rtp_lock);
	/* gain */
	//aw869xx_haptic_ram_vbat_comp(aw869xx, false);

	/* rtp mode config */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RTP_MODE);

	/* haptic start */
	mutex_lock(&aw869xx->rtp_check_lock);
	if (rtp_check_flag) {
		//aw869xx_set_clock(aw869xx, AW869XX_HAPTIC_CLOCK_CALI_OSC_STANDARD);
		//aw869xx_vol_trig_switch(aw869xx, false);
		//aw869xx_haptic_start(aw869xx);
		aw869xx_haptic_play_go(aw869xx);
		pr_info("%s ------------------>\n", __func__);
	} else {
		pr_info("%s rtp work has cancel\n", __func__);
		mutex_unlock(&aw869xx->rtp_check_lock);
		return;
	}
	mutex_unlock(&aw869xx->rtp_check_lock);
	usleep_range(2000, 2500);
	while (cnt) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x08) {
			cnt = 0;
			rtp_work_flag = true;
			dev_info(aw869xx->dev, "%s RTP_GO! glb_state=0x08\n",
				    __func__);
		} else {
			cnt--;
			dev_dbg(aw869xx->dev,
				   "%s wait for RTP_GO, glb_state=0x%02X\n",
				   __func__, reg_val);
		}
		usleep_range(2000, 2500);
	}
	if (rtp_work_flag) {
		aw869xx_haptic_rtp_init(aw869xx);
	} else {
		/* enter standby mode */
		aw869xx_haptic_stop(aw869xx);
		dev_err(aw869xx->dev, "%s failed to enter RTP_GO status!\n",
			   __func__);
	}
}

/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static int aw869xx_haptic_cont_config(struct aw869xx *aw869xx)
{
	dev_info(aw869xx->dev, "%s enter\n", __func__);

	/* work mode */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_CONT_MODE);
	/* cont config */
	/* aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
	 **                     AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
	 **                     AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	 */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_TRACK_EN_MASK,
			       AW869XX_BIT_CONTCFG6_TRACK_ENABLE);
	/* f0 driver level */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
			       aw869xx->info.cont_drv1_lvl);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG7,
			  aw869xx->info.cont_drv2_lvl);
	/* DRV1_TIME */
	/* aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG8, 0xFF); */
	/* DRV2_TIME */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG9, 0xFF);
	/* cont play go */
	aw869xx_haptic_play_go(aw869xx);
	return 0;
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
#if 0
static int aw869xx_haptic_set_f0_preset(struct aw869xx *aw869xx)
{
	unsigned int f0_reg = 0;

	pr_debug("%s enter\n", __func__);

	f0_reg = 1000000000/(aw869xx->f0_pre*AW869XX_HAPTIC_F0_COEFF);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_F_PRE_H, (unsigned char)((f0_reg>>8)&0xff));
	aw869xx_i2c_write(aw869xx, AW869XX_REG_F_PRE_L, (unsigned char)((f0_reg>>0)&0xff));

	return 0;
}
#endif

static int aw869xx_haptic_read_lra_f0(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	dev_info(aw869xx->dev, "%s enter\n", __func__);
	/* F_LRA_F0_H */
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_CONTRD14, &reg_val);
	f0_reg = (f0_reg | reg_val) << 8;
	/* F_LRA_F0_L */
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_CONTRD15, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		dev_err(aw869xx->dev, "%s didn't get lra f0 because f0_reg value is 0!\n", __func__);
		aw869xx->f0 = aw869xx->info.f0_ref;
		return ret;
	} else {
		f0_tmp = 384000 * 10 / f0_reg;
		aw869xx->f0 = (unsigned int)f0_tmp;
		dev_info(aw869xx->dev, "%s lra_f0=%d\n", __func__, aw869xx->f0);
	}

	return ret;
}


static int aw869xx_haptic_read_cont_f0(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	dev_info(aw869xx->dev, "%s enter\n", __func__);
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_CONTRD16, &reg_val);
	f0_reg = (f0_reg | reg_val) << 8;
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_CONTRD17, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		dev_err(aw869xx->dev,
			   "%s didn't get cont f0 because f0_reg value is 0!\n",
			   __func__);
		aw869xx->cont_f0 = aw869xx->info.f0_ref;
		return ret;
	} else {
		f0_tmp = 384000 * 10 / f0_reg;
		aw869xx->cont_f0 = (unsigned int)f0_tmp;
		dev_info(aw869xx->dev, "%s cont_f0=%d\n", __func__,
			    aw869xx->cont_f0);
	}
	return ret;
}

#ifdef USE_RAM_CALI_F0
static int aw869xx_haptic_ram_get_f0(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int cnt = 200;
	bool get_f0_flag = false;
	unsigned char d2s_gain_temp = 0;
	unsigned char brk_en_temp = 0;

	dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->f0 = aw869xx->info.f0_ref;
	/* enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	/* config max d2s_gain */
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSCTRL7, &reg_val);
	d2s_gain_temp = 0x07 & reg_val;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_26);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSCTRL7, &reg_val);
	dev_info(aw869xx->dev, "%s: d2s_gain=%d\n", __func__, 0x07 & reg_val);
	/* f0 calibrate work mode */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RAM_MODE);
	/* bst mode */
	//aw869xx_haptic_bst_mode_config(aw869xx, AW869XX_HAPTIC_BST_MODE_BYPASS);
	/* enable f0 detect */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
			       AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	/* cont config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_TRACK_EN_MASK,
			       AW869XX_BIT_CONTCFG6_TRACK_ENABLE);
	/* enable auto break */
	aw869xx_i2c_read(aw869xx, AW869XX_REG_PLAYCFG3, &reg_val);
	brk_en_temp = 0x04 & reg_val;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
			       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
			       AW869XX_BIT_PLAYCFG3_BRK_ENABLE);
	/* LRA OSC Source */
	if (aw869xx->f0_cali_flag == AW869XX_HAPTIC_CALI_F0) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_LRA_TRIM_SRC_MASK,
				       AW869XX_BIT_TRIMCFG3_LRA_TRIM_SRC_REG);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_LRA_TRIM_SRC_MASK,
				       AW869XX_BIT_TRIMCFG3_LRA_TRIM_SRC_EFUSE);
	}
	aw869xx_haptic_set_wav_seq(aw869xx, 0x00, 0x01);
	aw869xx_haptic_set_wav_loop(aw869xx, 0x00, 0x0A);
	/* ram play go */
	aw869xx_haptic_play_go(aw869xx);
	/* 300ms */
	while (cnt) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x00) {
			cnt = 0;
			get_f0_flag = true;
			dev_info(aw869xx->dev,
				    "%s entered standby mode! glb_state=0x%02X\n",
				    __func__, reg_val);
		} else {
			cnt--;
			dev_dbg(aw869xx->dev,
				   "%s waitting for standby, glb_state=0x%02X\n",
				   __func__, reg_val);
		}
		usleep_range(10000, 10500);
	}
	if (get_f0_flag) {
		aw869xx_haptic_read_lra_f0(aw869xx);
	} else {
		dev_err(aw869xx->dev,
			   "%s enter standby mode failed, stop reading f0!\n",
			   __func__);
	}
	/* restore d2s_gain config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       d2s_gain_temp);
	/* restore default config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
			       AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW869XX_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
			       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK, brk_en_temp);
	return ret;
}
#endif

static int aw869xx_haptic_cont_get_f0(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int cnt = 200;
	bool get_f0_flag = false;
	unsigned char d2s_gain_temp = 0;
	unsigned char brk_en_temp = 0;

	dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->f0 = aw869xx->info.f0_ref;
	/* enter standby mode */
	aw869xx_haptic_stop(aw869xx);
	/* config max d2s_gain */
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSCTRL7, &reg_val);
	d2s_gain_temp = 0x07 & reg_val;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_26);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSCTRL7, &reg_val);
	dev_info(aw869xx->dev, "%s: d2s_gain=%d\n", __func__, 0x07 & reg_val);
	/* f0 calibrate work mode */
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_CONT_MODE);
	/* bst mode */
	//aw869xx_haptic_bst_mode_config(aw869xx, AW869XX_HAPTIC_BST_MODE_BYPASS);
	/* enable f0 detect */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
			       AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	/* cont config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_TRACK_EN_MASK,
			       AW869XX_BIT_CONTCFG6_TRACK_ENABLE);
	/* enable auto break */
	aw869xx_i2c_read(aw869xx, AW869XX_REG_PLAYCFG3, &reg_val);
	brk_en_temp = 0x04 & reg_val;
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
			       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
			       AW869XX_BIT_PLAYCFG3_BRK_ENABLE);
	/* LRA OSC Source */
	if (aw869xx->f0_cali_flag == AW869XX_HAPTIC_CALI_F0) {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_LRA_TRIM_SRC_MASK,
				       AW869XX_BIT_TRIMCFG3_LRA_TRIM_SRC_REG);
	} else {
		aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_TRIMCFG3,
				       AW869XX_BIT_TRIMCFG3_LRA_TRIM_SRC_MASK,
				       AW869XX_BIT_TRIMCFG3_LRA_TRIM_SRC_EFUSE);
	}
	/* f0 driver level */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG6,
			       AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
			       aw869xx->info.cont_drv1_lvl);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG7,
			  aw869xx->info.cont_drv2_lvl);
	/* DRV1_TIME */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG8,
			  aw869xx->info.cont_drv1_time);
	/* DRV2_TIME */
	aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG9,
			  aw869xx->info.cont_drv2_time);
	/* TRACK_MARGIN */
	if (!aw869xx->info.cont_track_margin) {
		dev_err(aw869xx->dev,
			   "%s aw869xx->info.cont_track_margin = 0!\n",
			   __func__);
	} else {
		aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG11,
				  (unsigned char)aw869xx->
				  info.cont_track_margin);
	}
	/* DRV_WIDTH */
	/*
	 * aw869xx_i2c_write(aw869xx, AW869XX_REG_CONTCFG3,
	 *                aw869xx->info.cont_drv_width);
	 */
	/* cont play go */
	aw869xx_haptic_play_go(aw869xx);
	/* 300ms */
	while (cnt) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x00) {
			cnt = 0;
			get_f0_flag = true;
			dev_info(aw869xx->dev,
				    "%s entered standby mode! glb_state=0x%02X\n",
				    __func__, reg_val);
		} else {
			cnt--;
			dev_dbg(aw869xx->dev,
				   "%s waitting for standby, glb_state=0x%02X\n",
				   __func__, reg_val);
		}
		usleep_range(10000, 10500);
	}
	if (get_f0_flag) {
		aw869xx_haptic_read_lra_f0(aw869xx);
		aw869xx_haptic_read_cont_f0(aw869xx);
	} else {
		dev_err(aw869xx->dev,
			   "%s enter standby mode failed, stop reading f0!\n",
			   __func__);
	}
	/* restore d2s_gain config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7,
			       AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       d2s_gain_temp);
	/* restore default config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_CONTCFG1,
			       AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW869XX_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_PLAYCFG3,
			       AW869XX_BIT_PLAYCFG3_BRK_EN_MASK, brk_en_temp);
	return ret;
}

static int aw869xx_haptic_f0_calibration(struct aw869xx *aw869xx)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	unsigned int f0_cali_min =
	    aw869xx->info.f0_ref * (100 - aw869xx->info.f0_cali_percent) / 100;
	unsigned int f0_cali_max =
	    aw869xx->info.f0_ref * (100 + aw869xx->info.f0_cali_percent) / 100;

	dev_info(aw869xx->dev, "%s enter\n", __func__);
	aw869xx->f0_cali_flag = AW869XX_HAPTIC_CALI_F0;
#ifndef AW_ENABLE_MULTI_CALI
	aw869xx_haptic_upload_lra(aw869xx, WRITE_ZERO);
#endif

	if (aw869xx_haptic_cont_get_f0(aw869xx)) {
		dev_err(aw869xx->dev, "%s get f0 error, user defafult f0\n",
			   __func__);
	} else {
		/* max and min limit */
		f0_limit = aw869xx->f0;
		dev_info(aw869xx->dev, "%s f0_ref = %d, f0_cali_min = %d, f0_cali_max = %d, f0 = %d\n",
			    __func__, aw869xx->info.f0_ref,
			    f0_cali_min, f0_cali_max, aw869xx->f0);

		if ((aw869xx->f0 < f0_cali_min) || aw869xx->f0 > f0_cali_max) {
			dev_err(aw869xx->dev,
				   "%s f0 calibration out of range = %d!\n",
				   __func__, aw869xx->f0);
			f0_limit = aw869xx->info.f0_ref;
			return -ERANGE;
		}
		dev_info(aw869xx->dev, "%s f0_limit = %d\n", __func__,
			    (int)f0_limit);
		/* calculate cali step */
		f0_cali_step = 100000 * ((int)f0_limit -
					 (int)aw869xx->info.f0_ref) /
		    ((int)f0_limit * 24);
		dev_info(aw869xx->dev, "%s f0_cali_step = %d\n", __func__,
			    f0_cali_step);
		if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
			if (f0_cali_step % 10 >= 5)
				f0_cali_step = 32 + (f0_cali_step / 10 + 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		} else {	/* f0_cali_step < 0 */
			if (f0_cali_step % 10 <= -5)
				f0_cali_step = 32 + (f0_cali_step / 10 - 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		}
		if (f0_cali_step > 31)
			f0_cali_lra = (char)f0_cali_step - 32;
		else
			f0_cali_lra = (char)f0_cali_step + 32;
		/* update cali step */
		aw869xx_i2c_read(aw869xx, AW869XX_REG_TRIMCFG3, &reg_val);
		aw869xx->f0_cali_data =
		    ((int)f0_cali_lra + (int)(reg_val & 0x3f)) & 0x3f;

		dev_info(aw869xx->dev,
			    "%s origin trim_lra = 0x%02X, f0_cali_lra = 0x%02X, final f0_cali_data = 0x%02X\n",
			    __func__, (reg_val & 0x3f), f0_cali_lra,
			    aw869xx->f0_cali_data);
		aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
	}
	/* restore standby work mode */
	aw869xx_haptic_stop(aw869xx);
	return ret;
}
/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw869xx_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)g_aw869xx;

	return 0;
}

static int aw869xx_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static long aw869xx_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct aw869xx *aw869xx = (struct aw869xx *)file->private_data;

	int ret = 0;

	dev_info(aw869xx->dev, "%s: cmd=0x%x, arg=0x%lx\n",
			__func__, cmd, arg);

	mutex_lock(&aw869xx->lock);

	if (_IOC_TYPE(cmd) != AW869XX_HAPTIC_IOCTL_MAGIC) {
		dev_err(aw869xx->dev, "%s: cmd magic err\n",
				__func__);
		mutex_unlock(&aw869xx->lock);
		return -EINVAL;
	}

	switch (cmd) {
	default:
		dev_err(aw869xx->dev, "%s, unknown cmd\n", __func__);
		break;
	}

	mutex_unlock(&aw869xx->lock);

	return ret;
}

static ssize_t aw869xx_file_read(struct file *filp, char *buff, size_t len, loff_t *offset)
{
	struct aw869xx *aw869xx = (struct aw869xx *)filp->private_data;
	int ret = 0;
	int i = 0;
	unsigned char reg_val = 0;
	unsigned char *pbuff = NULL;

	mutex_lock(&aw869xx->lock);

	dev_info(aw869xx->dev, "%s: len=%zu\n", __func__, len);

	switch (aw869xx->fileops.cmd) {
	case AW869XX_HAPTIC_CMD_READ_REG:
		pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
		if (pbuff != NULL) {
			for (i = 0; i < len; i++) {
				aw869xx_i2c_read(aw869xx, aw869xx->fileops.reg+i, &reg_val);
				pbuff[i] = reg_val;
			}
			for (i = 0; i < len; i++) {
				dev_info(aw869xx->dev, "%s: pbuff[%d]=0x%02x\n",
						__func__, i, pbuff[i]);
			}
			ret = copy_to_user(buff, pbuff, len);
			if (ret) {
				dev_err(aw869xx->dev, "%s: copy to user fail\n", __func__);
			}
			kfree(pbuff);
		} else {
			dev_err(aw869xx->dev, "%s: alloc memory fail\n", __func__);
		}
		break;
	default:
		dev_err(aw869xx->dev, "%s, unknown cmd %d \n", __func__, aw869xx->fileops.cmd);
		break;
	}

	mutex_unlock(&aw869xx->lock);


	return len;
}

static ssize_t aw869xx_file_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	struct aw869xx *aw869xx = (struct aw869xx *)filp->private_data;
	int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;

	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		dev_err(aw869xx->dev, "%s: alloc memory fail\n", __func__);
		return len;
	}
	ret = copy_from_user(pbuff, buff, len);
	if (ret) {
		dev_err(aw869xx->dev, "%s: copy from user fail\n", __func__);
		kfree(pbuff);
		return len;
	}

	for (i = 0; i < len; i++) {
		dev_info(aw869xx->dev, "%s: pbuff[%d]=0x%02x\n",
				__func__, i, pbuff[i]);
	}

	mutex_lock(&aw869xx->lock);

	aw869xx->fileops.cmd = pbuff[0];

	switch (aw869xx->fileops.cmd) {
	case AW869XX_HAPTIC_CMD_READ_REG:
		if (len == 2) {
			aw869xx->fileops.reg = pbuff[1];
		} else {
			dev_err(aw869xx->dev, "%s: read cmd len %zu err\n", __func__, len);
		}
		break;
	case AW869XX_HAPTIC_CMD_WRITE_REG:
		if (len > 2) {
			for (i = 0; i < len-2; i++) {
				dev_info(aw869xx->dev, "%s: write reg0x%02x=0x%02x\n",
						__func__, pbuff[1]+i, pbuff[i+2]);
				aw869xx_i2c_write(aw869xx, pbuff[1]+i, pbuff[2+i]);
			}
		} else {
			dev_err(aw869xx->dev, "%s: write cmd len %zu err\n", __func__, len);
		}
		break;
	default:
		dev_err(aw869xx->dev, "%s, unknown cmd %d \n", __func__, aw869xx->fileops.cmd);
		break;
	}

	mutex_unlock(&aw869xx->lock);

	if (pbuff != NULL) {
		kfree(pbuff);
	}
	return len;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw869xx_file_read,
	.write = aw869xx_file_write,
	.unlocked_ioctl = aw869xx_file_unlocked_ioctl,
	.open = aw869xx_file_open,
	.release = aw869xx_file_release,
};

static struct miscdevice aw869xx_haptic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW869XX_HAPTIC_NAME,
	.fops = &fops,
};

static void aw869xx_reg_init_after_softreset(struct aw869xx *aw869xx)
{


	aw869xx_interrupt_setup(aw869xx);
	aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_STANDBY_MODE);
	aw869xx_haptic_set_pwm(aw869xx, AW869XX_PWM_24K);
	/* misc value init */
	aw869xx_haptic_misc_para_init(aw869xx);
	/* set BST_ADJ */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_BSTCFG5, AW869XX_BIT_BSTCFG5_BST_ADJ_MASK,
			AW869XX_BIT_BSTCFG5_BST_ADJ_LOW);

	aw869xx_haptic_set_bst_peak_cur(aw869xx);
	
	aw869xx_haptic_swicth_motor_protect_config(aw869xx, 0x00, 0x00);

    aw869xx_haptic_auto_brk_enable(aw869xx, aw869xx->info.is_enabled_auto_brk);

	aw869xx_haptic_auto_bst_enable(aw869xx, aw869xx->info.is_enabled_auto_bst);

	aw869xx_haptic_trig_param_init(aw869xx);
	aw869xx_haptic_trig_param_config(aw869xx);
	
	aw869xx_haptic_offset_calibration(aw869xx);

	/* vbat compensation */
	aw869xx_haptic_vbat_mode_config(aw869xx,
					AW869XX_HAPTIC_CONT_VBAT_HW_ADJUST_MODE);
	aw869xx->ram_vbat_compensate = AW869XX_HAPTIC_RAM_VBAT_COMP_ENABLE;
}

static void aw869xx_init_setting_work_routine(struct work_struct *work)
{

	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	struct aw869xx *aw869xx = container_of(work, struct aw869xx, init_setting_work);
	int retry = 3;


	pr_info("%s enter\n", __func__);
	aw869xx_rtp = devm_kzalloc(aw869xx->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);
	if (aw869xx_rtp == NULL) {
		pr_err("%s devm kzalloc failed\n", __func__);
	}
    /* haptic init */
	mutex_lock(&aw869xx->lock);
	aw869xx->activate_mode = aw869xx->info.mode;
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_WAVCFG1, &reg_val);
	aw869xx->index = reg_val & 0x7F;
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_PLAYCFG2, &reg_val);
	aw869xx->gain = reg_val & 0xFF;
	dev_info(aw869xx->dev, "%s aw869xx->gain =0x%02X\n", __func__, aw869xx->gain);
	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_PLAYCFG1, &reg_val);
	aw869xx->vmax = reg_val & 0x1F;
	for (i = 0; i < AW869XX_SEQUENCER_SIZE; i++) {
		ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_WAVCFG1 + i, &reg_val);
		aw869xx->seq[i] = reg_val;
	}

    mutex_unlock(&aw869xx->lock);

	aw869xx_reg_init_after_softreset(aw869xx);

	/* f0 calibration */
	aw869xx->f0_pre = aw869xx->lra_info.AW869XX_HAPTIC_F0_PRE;
	aw869xx->cont_drv_lvl = aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL;
	aw869xx->cont_drv_lvl_ov = aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL_OV;
	aw869xx->cont_td = aw869xx->lra_info.AW869XX_HAPTIC_CONT_TD;
	aw869xx->cont_zc_thr = aw869xx->lra_info.AW869XX_HAPTIC_CONT_ZC_THR;
	aw869xx->cont_num_brk = aw869xx->lra_info.AW869XX_HAPTIC_CONT_NUM_BRK;
	/* 取消每次开机校准，改为产线校准，然后存储校准值 */
	/* f0 calibration */
	if (aw869xx->info.powerup_f0_cali) {
		mutex_lock(&aw869xx->lock);
		aw869xx_haptic_upload_lra(aw869xx, WRITE_ZERO);
		aw869xx_haptic_f0_calibration(aw869xx);
		mutex_unlock(&aw869xx->lock);
	} else {
		dev_info(aw869xx->dev,
			    "%s powerup f0 calibration is disabled\n",
			    __func__);
	}
	// 读取阻抗值，打印log，方便排除硬件问题
	while (retry--) {
		aw869xx_haptic_get_lra_resistance(aw869xx);
		if ((aw869xx->lra >= aw869xx->resistance_min && aw869xx->lra <= aw869xx->resistance_max)) {
			pr_err("%s lra resistent ok, lra=%d\n", __func__, aw869xx->lra);
			break;
		} else {
			pr_err("%s lra resistent over range, lra=%d\n", __func__, aw869xx->lra);
		}
		msleep(5);
	}

	/* 由于每次写512字节，并且频繁申请和释放，所以为rtp模式开辟单独的高速缓存，提高内存分配速度 */
	rtp_cachep = kmem_cache_create("aw869xx-hap", RTP_SLAB_SIZE + 1, 0, SLAB_HWCACHE_ALIGN, NULL);
	if (rtp_cachep == NULL) {
		pr_err("%s alloc high cache failed\n", __func__);
	} else {
		pr_err("%s alloc high cache success\n", __func__);
	}

}

static int aw869xx_haptic_init(struct aw869xx *aw869xx)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);
	INIT_WORK(&aw869xx->init_setting_work, aw869xx_init_setting_work_routine);
	schedule_work(&aw869xx->init_setting_work);

	return ret;
}



/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
static enum led_brightness aw869xx_haptic_brightness_get(struct led_classdev *cdev)
{
	struct aw869xx *aw869xx =
		container_of(cdev, struct aw869xx, cdev);

	return aw869xx->amplitude;
}

static void aw869xx_haptic_brightness_set(struct led_classdev *cdev,
				enum led_brightness level)
{
	struct aw869xx *aw869xx =
		container_of(cdev, struct aw869xx, cdev);

	aw869xx->amplitude = level;

	mutex_lock(&aw869xx->lock);

	aw869xx_haptic_stop(aw869xx);
	if (aw869xx->amplitude > 0) {
		aw869xx_haptic_ram_vbat_comp(aw869xx, false);
		aw869xx_haptic_play_wav_seq(aw869xx, aw869xx->amplitude);
	}

	mutex_unlock(&aw869xx->lock);

}

static enum hrtimer_restart aw869xx_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw869xx *aw869xx = container_of(timer, struct aw869xx, timer);

	pr_debug("%s ---> enter\n", __func__);
	aw869xx->state = 0;
	schedule_work(&aw869xx->vibrator_work);

	return HRTIMER_NORESTART;
}

static void aw869xx_vibrator_work_routine(struct work_struct *work)
{
	struct aw869xx *aw869xx = container_of(work, struct aw869xx, vibrator_work);

	dev_info(aw869xx->dev, "%s enter\n", __func__);

	mutex_lock(&aw869xx->lock);
	/* Enter standby mode */
	aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
	aw869xx_haptic_stop(aw869xx);
	if (aw869xx->state) {
		if (aw869xx->activate_mode == AW869XX_HAPTIC_ACTIVATE_RAM_MODE) {
			aw869xx_haptic_ram_vbat_comp(aw869xx, false);
			aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RAM_MODE);
			/* boost voltage */
			if (aw869xx->info.bst_vol_ram <= AW869XX_MAX_BST_VOL){
				aw869xx_haptic_set_bst_vol(aw869xx, aw869xx->info.bst_vol_ram);
			} else{
				aw869xx_haptic_set_bst_vol(aw869xx, aw869xx->vmax);
			}
			aw869xx_haptic_play_go(aw869xx);
		} else if (aw869xx->activate_mode == AW869XX_HAPTIC_ACTIVATE_RAM_LOOP_MODE) {
			aw869xx_haptic_ram_vbat_comp(aw869xx, true);
			aw869xx_haptic_play_repeat_seq(aw869xx, true);
			hrtimer_start(&aw869xx->timer, ktime_set(aw869xx->duration / 1000,
						(aw869xx->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else if (aw869xx->activate_mode == AW869XX_HAPTIC_ACTIVATE_CONT_MODE) {
			aw869xx_haptic_cont_config(aw869xx);
			/* run ms timer */
			hrtimer_start(&aw869xx->timer, ktime_set(aw869xx->duration / 1000,
						(aw869xx->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else {
			dev_err(aw869xx->dev, "%s: activate_mode error\n", __func__);
		}

	}
	mutex_unlock(&aw869xx->lock);
}


/******************************************************************************************
 *
 * api interface
 *
 ******************************************************************************************/

static aw869xx_set_play_vol(struct aw869xx *aw869xx)
{
	int ret = 0;
	u8 vmax_val;
	u8 gain_val;

	if (aw869xx->play.vmax < 6000) {
		vmax_val = 0x00;
		gain_val = aw869xx->play.vmax * 128 / 6000;
	} else {
		vmax_val = (aw869xx->play.vmax - 6000) / 79;
		vmax_val = vmax_val + 9; //添加0.7v的线损补偿
		gain_val = 0x80;
	}
	if (vmax_val & 0xc0) {
		pr_err("aw869xx vmax_val=%d, gain_val=%d\n", 6000 + 0x3f * 79, gain_val);
	}else{
		pr_err("aw869xx vmax_val=%d, gain_val=%d\n", 6000 + vmax_val * 79, gain_val);
	}

	aw869xx_haptic_set_bst_vol(aw869xx, vmax_val);
	aw869xx_haptic_set_gain(aw869xx, gain_val);

	return ret;

}


static int aw869xx_load_effect(struct aw869xx *aw869xx, struct haptic_effect *p_effect)
{
	struct aw869xx_play_info *play = &aw869xx->play;
	s16 level, custom_data[CUSTOM_DATA_LEN] = {0, 0, 0};
	int real_vmax, i, j = 0, scene_effect_id = 0, scene_vmax = 0;
	struct aw869xx_wavefrom_info *effect_list = aw869xx->effect_list;

	memset(play, 0, sizeof(struct aw869xx_play_info));

	pr_err("%s: p_effect pointer (%#x)\n", __func__, p_effect); //ignore

	//other mode is play, cancla QQfly Driver vibrator
	 mutex_lock(&aw869xx->haptic_audio.lock);
	if (!aw869xx->haptic_audio.haptic_audio_cancel_flag) {
		aw869xx_haptic_audio_cancel(aw869xx);
	}
	aw869xx->ff_play_finish = false;
	mutex_unlock(&aw869xx->haptic_audio.lock);

	switch (p_effect->type) {

	case HAPTIC_CONSTANT:

		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			pr_err("%s copy from user failed\n", __func__);
			return -EFAULT;
		}


		play->playLength = p_effect->length * USEC_PER_MSEC;
		level = p_effect->magnitude;
		play->vmax = level * aw869xx->default_vmax / 0x7fff;
		play->type = TIME_TYPE;

		pr_err("%s: constant, length_us = %d, vmax_mv = %d, level = %d \n", __func__, play->playLength, play->vmax, level);
        /* Enter standby mode */
	    aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
		aw869xx_haptic_stop(aw869xx);
		aw869xx_haptic_set_repeat_wav_seq(aw869xx, PLAYBACK_INFINITELY_RAM_ID);

		break;

	case HAPTIC_CUSTOM:

		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			pr_err("%s copy from user failed\n", __func__);
			return -EFAULT;
		}

		dev_err(aw869xx->dev, "scene id %d\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
		if (custom_data[CUSTOM_DATA_EFFECT_IDX] < BASE_SCENE_COUNT_MAX) { //小于300的场景编号，从base scene列表里面找

			for (j = 0; j < aw869xx->base_scene_count; j++) {
				if (aw869xx->base_scene_list[j].scene_id == custom_data[CUSTOM_DATA_EFFECT_IDX])
					break;
			}

			if (j == aw869xx->base_scene_count) {
				dev_err(aw869xx->dev, "scene:%d not support\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
				return -EINVAL;
			}

			scene_vmax = aw869xx->base_scene_list[j].real_vmax; //real_vmax为场景设计的实际电压
			scene_effect_id = aw869xx->base_scene_list[j].effect_id;
		} else { //大于300的场景编号，从扩展ext scene列表里面找

			for (j = 0; j < aw869xx->ext_scene_count; j++) {
				if (aw869xx->ext_scene_list[j].scene_id == custom_data[CUSTOM_DATA_EFFECT_IDX])
					break;
			}

			if (j == aw869xx->ext_scene_count) {
				dev_err(aw869xx->dev, "scene:%d not support\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
				return -EINVAL;
			}

			scene_vmax = aw869xx->ext_scene_list[j].real_vmax;
			scene_effect_id = aw869xx->ext_scene_list[j].effect_id;
		}


		for (i = 0; i < aw869xx->effects_count; i++) //从效果中寻找场景需要的效果
			if (aw869xx->effect_list[i].idx == scene_effect_id)
				break;

		if (i == aw869xx->effects_count) {
			dev_err(aw869xx->dev, "scene: %d effect: %d not supported!\n",
			custom_data[CUSTOM_DATA_EFFECT_IDX], scene_effect_id);
			return -EINVAL;
		}

		//更新real vmax值，dts配置和hidl配置，取较小值
		if (scene_vmax > 0 && scene_vmax < effect_list[i].vmax)
			real_vmax = scene_vmax;
		else
			real_vmax = effect_list[i].vmax;

		dev_err(aw869xx->dev, "real_vamx = %d, scene_vamx = %d, effect_vmax = %d\n", real_vmax, scene_vmax, effect_list[i].vmax);

		if (!effect_list[i].rtp_enable) {

			play->type = RAM_TYPE;
			level = p_effect->magnitude;
			play->vmax = level * real_vmax / 0x7fff;
			play->times_ms = effect_list[i].times_ms;
			play->ram_id = effect_list[i].ram_id;

			dev_err(aw869xx->dev, "ram, effect_id = %d, ram_id = %d, vmax_mv = %d, length = %d, level = %d\n",
								scene_effect_id, play->ram_id, play->vmax, play->times_ms, level);
            aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
			aw869xx_haptic_stop(aw869xx);
			aw869xx_haptic_set_wav_loop(aw869xx, 0x00, 0x00);
			aw869xx_haptic_set_wav_seq(aw869xx, 0x00, play->ram_id);
			aw869xx_set_play_vol(aw869xx);
			if (aw869xx->play.ram_id == 0) {
				aw869xx_double_click_switch(aw869xx, true);
			}
		} else {

			play->type = RTP_TYPE;
			level = p_effect->magnitude;
			play->vmax = level * real_vmax / 0x7fff;
			play->times_ms = effect_list[i].times_ms;
			strlcpy(play->rtp_file, effect_list[i].rtp_file_name, 128);
			aw869xx->rtp_file_num = i;

			pr_err("%s: rtp, effect_id = %d, rtp_name: %s, vamx_mv = %d, length = %d, level = %d\n",
										__func__, scene_effect_id, play->rtp_file, play->vmax, play->times_ms, level);
            aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
			aw869xx_haptic_stop(aw869xx);
			aw869xx_haptic_set_rtp_aei(aw869xx, false);
			aw869xx_haptic_set_wav_loop(aw869xx, 0x00, 0x00);
			aw869xx_set_play_vol(aw869xx);
			aw869xx_interrupt_clear(aw869xx);
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

static int aw869xx_playback_vib(struct aw869xx *aw869xx, int val)
{
	struct aw869xx_play_info *play = &aw869xx->play;
	int len;
	if (val) {
        
        /* Enter standby mode */
        aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
        aw869xx_haptic_stop(aw869xx);

		switch (play->type) {
		case RAM_TYPE:
			pr_err("%s: ---> start ram mode\n", __func__);
			aw869xx_haptic_play_wav_seq(aw869xx, true);
			break;

		case RTP_TYPE:
			pr_err("%s: ---> start rtp mode\n", __func__);
			queue_work(rtp_wq, &aw869xx->rtp_work);
			rtp_check_flag = true;
			break;
		case TIME_TYPE:
			aw869xx->activate_mode = AW869XX_HAPTIC_ACTIVATE_RAM_MODE;

			aw869xx->gain = 0x80; //4200为电池电压,波形已适配到2.5V，长震取最大电压
			dev_err(aw869xx->dev, "play vmax = %d, aw869xx gain = %d\n", play->vmax, aw869xx->gain);
			/* clip value to max */
			len = play->playLength + 10000;

			pr_err("%s: ---> start time mode, length = %d\n", __func__, len);

			if (aw869xx->activate_mode == AW869XX_HAPTIC_ACTIVATE_RAM_MODE) {
				aw869xx_haptic_ram_vbat_comp(aw869xx, false);
				aw869xx_haptic_play_repeat_seq(aw869xx, true);
			} else if (aw869xx->activate_mode == AW869XX_HAPTIC_ACTIVATE_CONT_MODE) {
				aw869xx_haptic_cont_config(aw869xx);
			} else {
				pr_err("%s: not suppoert activate mode\n", __func__);
			}
			/* run us timer */
			hrtimer_start(&aw869xx->timer,
				ktime_set(len / USEC_PER_SEC, (len % USEC_PER_SEC) * NSEC_PER_USEC),
				HRTIMER_MODE_REL);

			break;

		default:

			break;

		}

	} else {
		if (hrtimer_active(&aw869xx->timer)) {
			hrtimer_cancel(&aw869xx->timer);
			pr_err("%s playback cancel timer\n", __func__);
		}

		/* 取消rtp work */
		if (cancel_work_sync(&aw869xx->rtp_work)) {
			pr_err("%s palyback pending work cancle success\n", __func__);
		}
		//aw869xx->rtp_len = 0;
		mutex_lock(&aw869xx->rtp_check_lock);
		/* Enter standby mode */
		aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
		aw869xx_haptic_stop(aw869xx);
		if (aw869xx->play.ram_id == 0) {
			aw869xx_double_click_switch(aw869xx, false);
		}
		rtp_check_flag = false;
		mutex_unlock(&aw869xx->rtp_check_lock);
		//aw869xx_vol_trig_switch(aw869xx, true);
		//aw869xx_set_clock(aw869xx, AW869XX_HAPTIC_CLOCK_CALI_F0);
	}
	//play finish, allow QQfly Driver vibrator
	mutex_lock(&aw869xx->haptic_audio.lock);
	if (!val) {
		pr_debug("%s: haptic audio ,ff play finish\n", __func__);
		aw869xx->ff_play_finish = true;

	}
	mutex_unlock(&aw869xx->haptic_audio.lock);

	return 0;
}

static int aw869xx_upload_sync(struct haptic_device *hp, struct haptic_effect *effect)
{
	int ret = 0;
	struct aw869xx *aw869xx = (struct aw869xx *)hp->chip;

	ret = aw869xx_load_effect(aw869xx, effect);

	return ret;
}

static int aw869xx_erase_sync(struct haptic_device *hp)
{
	int ret = 0;
	struct aw869xx *aw869xx = (struct aw869xx *)hp->chip;

	ret = aw869xx_playback_vib(aw869xx, 0);
	return 0;
}
static int aw869xx_playback_sync(struct haptic_device *hp, int value)
{
	struct aw869xx *aw869xx = (struct aw869xx *)hp->chip;
	int ret = 0;

	ret = aw869xx_playback_vib(aw869xx, !!value);

	return ret;
}

static void aw869xx_set_gain_sync(struct haptic_device *hp, u16 gain)
{

	struct aw869xx *aw869xx = (struct aw869xx *)hp->chip;
	struct aw869xx_play_info *play = &aw869xx->play;

	pr_info("%s gain=%d\n", __func__, gain);

	if (play->type == TIME_TYPE) {
		if (gain > 0x7fff)
			gain = 0x7fff;

		aw869xx->gain = ((u32)(gain * 0x80)) / 0x7fff;
		aw869xx_haptic_ram_vbat_comp(aw869xx, true);
	}

}

static struct haptic_device hp = {

	.name = "aw869xx_haptic",
	.upload = aw869xx_upload_sync,
	.erase =  aw869xx_erase_sync,
	.playback = aw869xx_playback_sync,
	.set_gain =  aw869xx_set_gain_sync,

};

static void haptic_device_set_capcity(struct haptic_device *hp, int bit)
{
	if (bit > HAPTIC_CNT - 1) {
		pr_err("%s invalid bit, bit=%d\n", __func__, bit);
		return;
	}
	__set_bit(bit, hp->hap_bit);
}

static int aw869xx_vibrator_init(struct aw869xx *aw869xx)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);

	hrtimer_init(&aw869xx->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw869xx->timer.function = aw869xx_vibrator_timer_func;
	INIT_WORK(&aw869xx->vibrator_work, aw869xx_vibrator_work_routine);

	/* rtp模式要求数据写入及时，不能断流，故建立单独的工作队列，专门处理rtp数据传输，而不使用系统默认的工作队列 */
	rtp_wq = create_singlethread_workqueue("rtp_wq");
	INIT_WORK(&aw869xx->rtp_work, aw869xx_rtp_work_routine);

	mutex_init(&aw869xx->lock);
	mutex_init(&aw869xx->rtp_check_lock);
	mutex_init(&aw869xx->rtp_lock);
	wake_lock_init(&aw869xx->wake_lock, WAKE_LOCK_SUSPEND, "vivo-aw869xx-wakelock");


	ret = misc_register(&aw869xx_haptic_misc);
	if (ret) {
		dev_err(aw869xx->dev,  "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}

	/* register haptic miscdev */

	hp.chip = aw869xx;
	hp.dev = &aw869xx->i2c->dev;
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
    aw869xx->haptic_audio.delay_val = 20833; //us
    aw869xx->haptic_audio.timer_val = 20833; //us
    aw869xx->haptic_audio.haptic_audio_cancel_flag = true;
    aw869xx->ff_play_finish = true;

    hrtimer_init(&aw869xx->haptic_audio.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw869xx->haptic_audio.timer.function = aw869xx_haptic_audio_timer_func;
    INIT_WORK(&aw869xx->haptic_audio.work, aw869xx_haptic_audio_work_routine);
    INIT_LIST_HEAD(&aw869xx->haptic_audio.ctr_list);
    INIT_LIST_HEAD(&(aw869xx->haptic_audio.list));

    spin_lock_init(&aw869xx->haptic_audio.list_lock);
    mutex_init(&aw869xx->haptic_audio.lock);

	atomic_set(&aw869xx->f0_freq_cali, DEFAULT_CALI_F0);
	atomic_set(&aw869xx->standard_osc_freq_cali, DEFAULT_OSC_CALI_DATA);

	//aw869xx->perm_disable = false;

	return 0;
}


/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw869xx_interrupt_clear(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;
	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	pr_err("%s enter: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw869xx_interrupt_setup(struct aw869xx *aw869xx)
{
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

	/* edge int mode */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7, AW869XX_BIT_SYSCTRL7_INT_MODE_MASK,
			       AW869XX_BIT_SYSCTRL7_INT_MODE_EDGE);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL7, AW869XX_BIT_SYSCTRL7_INT_EDGE_MODE_MASK,
			       AW869XX_BIT_SYSCTRL7_INT_EDGE_MODE_POS);

	/* int enable */
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM, AW869XX_BIT_SYSINTM_BST_SCPM_MASK,
			       AW869XX_BIT_SYSINTM_BST_SCPM_OFF);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM, AW869XX_BIT_SYSINTM_BST_OVPM_MASK,
			       AW869XX_BIT_SYSINTM_BST_OVPM_ON);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM, AW869XX_BIT_SYSINTM_UVLM_MASK,
			       AW869XX_BIT_SYSINTM_UVLM_ON);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM, AW869XX_BIT_SYSINTM_OCDM_MASK,
			       AW869XX_BIT_SYSINTM_OCDM_ON);
	aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSINTM, AW869XX_BIT_SYSINTM_OTM_MASK,
			       AW869XX_BIT_SYSINTM_OTM_ON);
}

static irqreturn_t aw869xx_irq(int irq, void *data)
{
	struct aw869xx *aw869xx = data;
	unsigned char reg_val = 0;
	unsigned int buf_len = 0;
	int ret = 0, retval = 0;

	pr_err("%s enter\n", __func__);

	aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	if (reg_val & AW869XX_BIT_SYSINT_BST_OVPI) {
		pr_err("%s chip ov int error\n", __func__);
	}
	if (reg_val & AW869XX_BIT_SYSINT_UVLI) {
		pr_err("%s chip uvlo int error\n", __func__);
	}
	if (reg_val & AW869XX_BIT_SYSINT_OCDI) {
		pr_err("%s chip over current int error\n", __func__);
	}
	if (reg_val & AW869XX_BIT_SYSINT_OTI) {
		pr_err("%s chip over temperature int error\n", __func__);
	}
	if (reg_val & AW869XX_BIT_SYSINT_DONEI) {
		pr_info("%s chip playback done\n", __func__);
	}

	if (reg_val & AW869XX_BIT_SYSINT_FF_AEI) {
		pr_err("%s: aw869xx rtp fifo almost empty int\n", __func__);
		mutex_lock(&aw869xx->rtp_lock);
		if (aw869xx->rtp_init) {
			while ((!(retval = aw869xx_haptic_rtp_get_fifo_afs(aw869xx))) &&
					(aw869xx->play_mode == AW869XX_HAPTIC_RTP_MODE)) {
				pr_info("%s: aw869xx rtp mode fifo update, cnt=%d\n",
						__func__, aw869xx->rtp_cnt);
                if (!aw869xx_rtp) {
					pr_info("%s:aw869xx_rtp is null, break!\n", __func__);
					mutex_unlock(&aw869xx->rtp_lock);
					break;
				}
				if ((aw869xx_rtp->len-aw869xx->rtp_cnt) < (aw869xx->ram.base_addr>>2)) {
					buf_len = aw869xx_rtp->len-aw869xx->rtp_cnt;
				} else {
					buf_len = (aw869xx->ram.base_addr>>2);
				}
				ret = aw869xx_i2c_writes(aw869xx, AW869XX_REG_RTPDATA,
						&aw869xx_rtp->data[aw869xx->rtp_cnt], buf_len);
				if (ret < 0) {
					pr_err("%s total length: %d, play length: %d\n",
						__func__, aw869xx_rtp->len, aw869xx->rtp_cnt);
					pr_err("%s error, ret=%d\n", __func__, ret);
					aw869xx_haptic_set_rtp_aei(aw869xx, false);
					aw869xx->rtp_cnt = 0;
					aw869xx->rtp_init = 0;
					break;
				}
				aw869xx->rtp_cnt += buf_len;
				if (aw869xx->rtp_cnt >= aw869xx_rtp->len) {
					pr_err("%s complete total length: %d, play length: %d\n",
						__func__, aw869xx_rtp->len, aw869xx->rtp_cnt);
					pr_info("%s: rtp update complete\n", __func__);
					aw869xx_haptic_set_rtp_aei(aw869xx, false);
					aw869xx->rtp_cnt = 0;
					aw869xx->rtp_init = 0;
					break;
				}
			}
		} else {
			pr_err("%s: aw869xx rtp init = %d, init error\n", __func__, aw869xx->rtp_init);
			aw869xx_haptic_set_rtp_aei(aw869xx, false);

		}
		mutex_unlock(&aw869xx->rtp_lock);
	}

	if (reg_val & AW869XX_BIT_SYSINT_FF_AFI) {
		pr_err("%s: aw869xx rtp mode fifo full empty\n", __func__);
	}

	if ((aw869xx->play_mode != AW869XX_HAPTIC_RTP_MODE) || (retval < 0)) {
		aw869xx_haptic_set_rtp_aei(aw869xx, false);
	}

	//aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSINT, &reg_val);
	//pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	//aw869xx_i2c_read(aw869xx, AW869XX_REG_SYSST, &reg_val);
	//pr_debug("%s: reg SYSST=0x%x\n", __func__, reg_val);

	pr_debug("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/

static aw869xx_parse_per_effect_dt(struct aw869xx *aw869xx, struct device_node *child,
								struct aw869xx_wavefrom_info *effect_node)
{
	int rc;

	rc = of_property_read_u32(child, "awinic,effect-id", &effect_node->idx);
	if (rc < 0) {
		dev_err(aw869xx->dev, "Read awinic effect-id failed, rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(child, "awinic,wf-vmax-mv", &effect_node->vmax);
	if (rc < 0) {
		dev_err(aw869xx->dev, "Read awinic wf-vmax-mv failed, rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(child, "awinic,wf-length", &effect_node->times_ms);
	if (rc < 0) {
		dev_err(aw869xx->dev, "Read awinic wf-length failed, rc=%d\n", rc);
		return rc;
	}

	effect_node->rtp_enable = of_property_read_bool(child, "awinic,rtp-enable");

	if (effect_node->rtp_enable) {

		rc = of_property_read_string(child, "awinic,rtp-file", &effect_node->rtp_file_name);
		if (rc < 0) {
			dev_err(aw869xx->dev, "Read awinic rtp-file failed, rc=%d\n", rc);
			return rc;
		}

		effect_node->ram_id = 1;

	} else {

		rc = of_property_read_u32(child, "awinic,ram-id", &effect_node->ram_id);
		if (rc < 0) {
			dev_err(aw869xx->dev, "Read awinic ram-id failed, rc=%d\n", rc);
			return rc;
		}
		effect_node->rtp_file_name = "default";
	}

	return 0;
}

static void __dump_scene_info(struct aw869xx *chip)
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


static int aw869xx_parse_scene_dt(struct aw869xx *chip)
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
static int  aw869xx_lra_information_ctr(struct aw869xx *aw869xx)
{
    switch(aw869xx->lra_information)
    {
        case AW869XX_LRA_0832:
            pr_debug("%s enter AW869XX_LRA_0832 \n", __func__);
            aw869xx->lra_info.AW869XX_HAPTIC_F0_PRE = 2350;
            aw869xx->lra_info.AW869XX_HAPTIC_F0_CALI_PERCEN = 7;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL = 125;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL_OV = 155;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_TD = 0x006c;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_ZC_THR = 0x0ff1;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_NUM_BRK = 3;
            break;
        case AW869XX_LRA_0815:
            pr_debug("%s enter AW869XX_LRA_0815\n", __func__);
            aw869xx->lra_info.AW869XX_HAPTIC_F0_PRE = 1700;   // 170Hz
            aw869xx->lra_info.AW869XX_HAPTIC_F0_CALI_PERCEN = 7;       // -7%~7%
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL = 53;   // 71*6.1/256=1.69v
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL_OV = 125;    // 125*6.1/256=2.98v
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_TD = 0x009a;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_ZC_THR = 0x0ff1;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_NUM_BRK = 3;
            aw869xx->lra_info.AW869XX_HAPTIC_RATED_VOLTAGE = 1270; //mv-Vp
            break;
        default:
            pr_debug("%s enter AW869XX_LRA_DEFAULT\n", __func__);
            aw869xx->lra_info.AW869XX_HAPTIC_F0_PRE = 1700;   // 170Hz
            aw869xx->lra_info.AW869XX_HAPTIC_F0_CALI_PERCEN = 7;       // -7%~7%
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL = 71;   // 71*6.1/256=1.69v
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL_OV = 125;    // 125*6.1/256=2.98v
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_TD = 0x009a;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_ZC_THR = 0x0ff1;
            aw869xx->lra_info.AW869XX_HAPTIC_CONT_NUM_BRK = 3;
            aw869xx->lra_info.AW869XX_HAPTIC_RATED_VOLTAGE = 1700; //mv-Vp
            break;
    }
    pr_debug("%s aw869xx->lra_information = %d \n", __func__, aw869xx->lra_information);
    pr_debug("%s AW869XX_HAPTIC_F0_PRE = %d \n", __func__, aw869xx->lra_info.AW869XX_HAPTIC_F0_PRE);
    pr_debug("%s AW869XX_HAPTIC_F0_CALI_PERCEN = %d\n", __func__, aw869xx->lra_info.AW869XX_HAPTIC_F0_CALI_PERCEN);
    pr_debug("%s AW869XX_HAPTIC_CONT_DRV_LVL = %d\n", __func__, aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL);
    pr_debug("%s AW869XX_HAPTIC_CONT_DRV_LVL_OV = %d\n", __func__, aw869xx->lra_info.AW869XX_HAPTIC_CONT_DRV_LVL_OV);
    pr_debug("%s AW869XX_HAPTIC_CONT_TD = %d\n", __func__, aw869xx->lra_info.AW869XX_HAPTIC_CONT_TD);
    pr_debug("%s AW869XX_HAPTIC_CONT_ZC_THR = %d\n", __func__, aw869xx->lra_info.AW869XX_HAPTIC_CONT_ZC_THR);
    pr_debug("%s AW869XX_HAPTIC_CONT_NUM_BRK = %d;\n", __func__, aw869xx->lra_info.AW869XX_HAPTIC_CONT_NUM_BRK);
    pr_debug("%s AW869XX_HAPTIC_RATED_VOLTAGE = %d\n", __func__, aw869xx->lra_info.AW869XX_HAPTIC_RATED_VOLTAGE);
    return 0;
}

static int aw869xx_parse_rst_irq_gpio_dt(struct device *dev, struct aw869xx *aw869xx,
		struct device_node *np)
{
    aw869xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw869xx->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
		//return -ENODEV;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw869xx->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
	if (aw869xx->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
		return -ENODEV;
	} else {
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
	}

    return 0;
}

static int aw869xx_parse_dt(struct device *dev, struct aw869xx *aw869xx,
		struct device_node *np)
{

	struct device_node *child;
	int rc, val, i = 0, effect_count = 0;
    unsigned int bstcfg_temp[5] = { 0x2a, 0x24, 0x9a, 0x40, 0x91 };
	unsigned int prctmode_temp[3];
	unsigned int sine_array_temp[4] = { 0x05, 0xB2, 0xFF, 0xEF };
	unsigned int trig_config_temp[24] = { 1, 0, 1, 1, 1, 2, 0, 0,
		1, 0, 0, 1, 0, 2, 0, 0,
		1, 0, 0, 1, 0, 2, 0, 0
	};

	if (of_property_read_u32(np, "awinic,vmax", &aw869xx->default_vmax)) {
		dev_err(dev, "%s: no default vmax\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "resistance_min", &aw869xx->resistance_min)) {
		dev_err(dev, "%s: no resistance_min\n", __func__);
		return -EFAULT;
	}
	dev_info(dev, "resistance_min:%d\n", aw869xx->resistance_min);

	if (of_property_read_u32(np, "resistance_max", &aw869xx->resistance_max)) {
		dev_err(dev, "%s: no resistance_max\n", __func__);
		return -EFAULT;
	}
	dev_info(dev, "resistance_max:%d\n", aw869xx->resistance_max);

	if (of_property_read_u32(np, "freq_min", &aw869xx->freq_min)) {
		dev_err(dev, "%s: no freq_min\n", __func__);
		return -EFAULT;
	}
	dev_info(dev, "freq_min:%d\n", aw869xx->freq_min);

	if (of_property_read_u32(np, "freq_max", &aw869xx->freq_max)) {
		dev_err(dev, "%s: no freq_max\n", __func__);
		return -EFAULT;
	}
	dev_info(dev, "freq_max:%d\n", aw869xx->freq_max);

	if (of_property_read_bool(np, "disable-trigger")) {
		dev_info(dev, "not support trigger\n");
		aw869xx->no_trigger = true;
	}

      /* prase lra_info */
	if (of_property_read_u32(np, "lra_info", &aw869xx->lra_information)) {
		aw869xx->lra_information = 619;
        dev_err(dev, "lra_info:%d\n", aw869xx->lra_information);
	}
	dev_info(dev, "lra_info:%d\n", aw869xx->lra_information);

	rc = aw869xx_lra_information_ctr(aw869xx);
	if (rc < 0) {
		pr_err("%s: lra_information_ctr get failed\n", __func__);

	}

    /* prase prama which is affected by lra_info */
    val = of_property_read_u32(np, "aw86917,vib_mode", &aw869xx->info.mode);
    if (val != 0)
        dev_info(aw869xx->dev, "%s vib_mode not found\n", __func__);
    val = of_property_read_u32(np, "aw86917,vib_f0_ref", &aw869xx->info.f0_ref);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_f0_ref not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_f0_cali_percent",
                 &aw869xx->info.f0_cali_percent);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_f0_cali_percent not found\n",
                __func__);

    val = of_property_read_u32(np, "aw86917,vib_cont_drv1_lvl",
                   &aw869xx->info.cont_drv1_lvl);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_drv1_lvl not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_drv2_lvl",
                 &aw869xx->info.cont_drv2_lvl);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_drv2_lvl not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_drv1_time",
                 &aw869xx->info.cont_drv1_time);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_drv1_time not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_drv2_time",
                 &aw869xx->info.cont_drv2_time);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_drv2_time not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_drv_width",
                 &aw869xx->info.cont_drv_width);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_drv_width not found\n",
                __func__);
    val =
        of_property_read_u32(np, "vib_cont_wait_num",
                 &aw869xx->info.cont_wait_num);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_wait_num not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_bst_brk_gain",
                 &aw869xx->info.cont_bst_brk_gain);
    if (val != 0)
        dev_info(aw869xx->dev,
                "%s vib_cont_bst_brk_gain not found\n", __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_brk_gain",
                 &aw869xx->info.cont_brk_gain);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_brk_gain not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_tset", &aw869xx->info.cont_tset);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_tset not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_bemf_set",
                 &aw869xx->info.cont_bemf_set);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_bemf_set not found\n",
                __func__);
    val = of_property_read_u32(np, "aw86917,vib_d2s_gain", &aw869xx->info.d2s_gain);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_d2s_gain not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_brk_time",
                 &aw869xx->info.cont_brk_time);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_cont_brk_time not found\n",
                __func__);
    val =
        of_property_read_u32(np, "aw86917,vib_cont_track_margin",
                 &aw869xx->info.cont_track_margin);
    if (val != 0)
        dev_info(aw869xx->dev,
                "%s aw86917,vib_cont_track_margin not found\n", __func__);
    aw869xx->info.is_enabled_auto_brk =
	    of_property_read_bool(np, "aw86917,vib_is_enabled_auto_brk");
	dev_info(aw869xx->dev,
		    "%s aw869xx->info.is_enabled_auto_brk = %d\n", __func__,
		    aw869xx->info.is_enabled_auto_brk);
    aw869xx->info.is_enabled_auto_bst =
	    of_property_read_bool(np, "aw86917,vib_is_enabled_auto_bst");
	dev_info(aw869xx->dev,
		    "%s aw869xx->info.is_enabled_auto_bst = %d\n", __func__,
		    aw869xx->info.is_enabled_auto_bst);
    aw869xx->info.powerup_f0_cali =
            of_property_read_bool(np, "aw86917,vib_powerup_f0_cali");
    dev_info(aw869xx->dev,
            "%s aw869xx->info.vib_powerup_f0_cali = %d\n",
            __func__, aw869xx->info.powerup_f0_cali);
    val = of_property_read_u32_array(np, "aw86917,vib_bstcfg", bstcfg_temp,
                     ARRAY_SIZE(bstcfg_temp));
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_bstcfg not found\n",
                __func__);
    memcpy(aw869xx->info.bstcfg, bstcfg_temp, sizeof(bstcfg_temp));

    val = of_property_read_u32(np, "aw86917,vib_bst_vol_default",
                   &aw869xx->info.bst_vol_default);
    if (val != 0)
        dev_info(aw869xx->dev,
                "%s aw86917,vib_bst_vol_default not found\n", __func__);
    val = of_property_read_u32(np, "aw86917,vib_bst_vol_ram",
                   &aw869xx->info.bst_vol_ram);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_bst_vol_ram not found\n",
                __func__);
    val = of_property_read_u32(np, "aw86917,vib_bst_vol_rtp",
                   &aw869xx->info.bst_vol_rtp);
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_bst_vol_rtp not found\n",
                __func__);
    val = of_property_read_u32_array(np, "aw86917,vib_prctmode",
                         prctmode_temp,
                         ARRAY_SIZE(prctmode_temp));
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_prctmode not found\n",
                __func__);
    memcpy(aw869xx->info.prctmode, prctmode_temp, sizeof(prctmode_temp));
    val = of_property_read_u32_array(np, "aw86917,vib_sine_array", sine_array_temp,
                     ARRAY_SIZE(sine_array_temp));
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_sine_array not found\n",
                __func__);
    memcpy(aw869xx->info.sine_array, sine_array_temp,
           sizeof(sine_array_temp));
    val =
        of_property_read_u32_array(np, "aw86917,vib_trig_config", trig_config_temp,
                       ARRAY_SIZE(trig_config_temp));
    if (val != 0)
        dev_info(aw869xx->dev, "%s aw86917,vib_trig_config not found\n",
                __func__);
    memcpy(aw869xx->info.trig_config, trig_config_temp,
           sizeof(trig_config_temp));

	//parse effect begin
	for_each_available_child_of_node(np, child) {
		if (of_find_property(child, "awinic,effect-id", NULL))
			effect_count++;
	}
	if (effect_count == 0) {
		dev_err(dev, "no dts effect configed, use default effect list\n");
		aw869xx->effect_list = waveform_list_default;
		aw869xx->effects_count = 1;
		return 0;
	}

	aw869xx->effect_list = devm_kcalloc(aw869xx->dev, effect_count,
				sizeof(*aw869xx->effect_list), GFP_KERNEL);
	if (!aw869xx->effect_list)
		return -ENOMEM;

	for_each_available_child_of_node(np, child) {
		if (!of_find_property(child, "awinic,effect-id", NULL))
			continue;

		rc = aw869xx_parse_per_effect_dt(aw869xx, child, &aw869xx->effect_list[i]);
		if (rc < 0) {
			dev_err(aw869xx->dev, "parse effect %d failed, rc=%d\n", i, rc);
			of_node_put(child);
			return rc;
		}
		i++;
	}

	aw869xx->effects_count = i;
	dev_err(aw869xx->dev, "effect count: %d, dts node: %d\n", i, effect_count);

	//parse effect end


	// dts中场景列表解析begin
	rc = aw869xx_parse_scene_dt(aw869xx);
	if (rc < 0) {
		dev_err(aw869xx->dev, "Parse scene list failed, rc=%d\n", rc);
		return -EFAULT;
	}
	// dts中场景列表解析end

	return 0;
}

static int aw869xx_hw_reset(struct aw869xx *aw869xx)
{
	pr_info("%s enter\n", __func__);

	if (aw869xx && gpio_is_valid(aw869xx->reset_gpio)) {
		gpio_set_value_cansleep(aw869xx->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw869xx->reset_gpio, 1);
		msleep(2);
	} else {
		dev_err(aw869xx->dev, "%s:  failed\n", __func__);
	}
	return 0;
}


/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw869xx_read_chipid(struct aw869xx *aw869xx)
{
	int ret = -1;
	unsigned char reg = 0;

	/* hardware reset */
	aw869xx_hw_reset(aw869xx);

	ret = aw869xx_i2c_read(aw869xx, AW869XX_REG_ID, &reg);
	if (ret < 0) {
		dev_err(aw869xx->dev, "%s: failed to read register AW869XX_REG_ID: %d\n", __func__, ret);
	}
	switch (reg) {
	case AW86917_CHIPID:
		pr_info("%s aw86917 detected\n", __func__);
		aw869xx->chipid = AW86917_CHIPID;
		aw869xx->bst_pc = AW869XX_HAPTIC_BST_PC_L2;
		aw869xx_haptic_softreset(aw869xx);
		return 0;
	default:
		pr_info("%s unsupported device revision (0x%x)\n", __func__, reg);
		break;
	}

	return -EINVAL;
}
/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static enum hrtimer_restart aw869xx_haptic_audio_timer_func(struct hrtimer *timer)
{
	struct aw869xx *aw869xx = container_of(timer, struct aw869xx, haptic_audio.timer);

	pr_debug("%s enter\n", __func__);
	schedule_work(&aw869xx->haptic_audio.work);

	hrtimer_start(&aw869xx->haptic_audio.timer,
			ktime_set(aw869xx->haptic_audio.timer_val/1000000,
					(aw869xx->haptic_audio.timer_val%1000000) * 1000),
			HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void aw869xx_haptic_audio_work_routine(struct work_struct *work)
{
	struct aw869xx *aw869xx = container_of(work, struct aw869xx, haptic_audio.work);
	struct haptic_audio *haptic_audio = &(aw869xx->haptic_audio);
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

	spin_lock(&aw869xx->haptic_audio.list_lock);

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
					(AW869XX_HAPTIC_CMD_ENABLE == (AW869XX_HAPTIC_CMD_HAPTIC & p_ctr->cmd))) {

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
		spin_unlock(&aw869xx->haptic_audio.list_lock);
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

	spin_unlock(&aw869xx->haptic_audio.list_lock);

	mutex_lock(&aw869xx->haptic_audio.lock);

	memset(&aw869xx->haptic_audio.ctr, 0, sizeof(struct haptic_ctr));

	haptic_audio->ctr = temp_haptic_audio;

	if (aw869xx->haptic_audio.ctr.play) {
		pr_info("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
		 __func__, aw869xx->haptic_audio.ctr.cnt,
		 aw869xx->haptic_audio.ctr.cmd,
		 aw869xx->haptic_audio.ctr.play,
		 aw869xx->haptic_audio.ctr.wavseq,
		 aw869xx->haptic_audio.ctr.loop,
		 aw869xx->haptic_audio.ctr.gain);
	}

	if (!haptic_audio->haptic_audio_cancel_flag) {
        //if cmd==1,start vibrating
		if (aw869xx->haptic_audio.ctr.cmd == AW869XX_HAPTIC_CMD_ENABLE) {

			if (aw869xx->haptic_audio.ctr.play == AW869XX_HAPTIC_PLAY_ENABLE) {
				pr_info("%s: haptic_audio_play_start\n", __func__);
				mutex_lock(&aw869xx->lock);
                aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
                
			    aw869xx_haptic_stop(aw869xx);                

				aw869xx_haptic_set_wav_seq(aw869xx, 0x00, aw869xx->haptic_audio.ctr.wavseq);

				aw869xx_haptic_set_wav_seq(aw869xx, 0x01, 0x00);

				aw869xx_haptic_set_wav_loop(aw869xx, 0x00, aw869xx->haptic_audio.ctr.loop);                
                //set bost to 6000 + 34 x 78.89 = 8700mv,0.7V为线损
                aw869xx_haptic_set_bst_vol(aw869xx, 34);

				aw869xx_haptic_set_gain(aw869xx, aw869xx->haptic_audio.ctr.gain);

                aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_RAM_MODE);

				aw869xx_haptic_play_go(aw869xx);
				mutex_unlock(&aw869xx->lock);

			} else if (AW869XX_HAPTIC_PLAY_STOP == aw869xx->haptic_audio.ctr.play) {

				mutex_lock(&aw869xx->lock);
                aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
				aw869xx_haptic_stop(aw869xx);
				mutex_unlock(&aw869xx->lock);

			} else if (AW869XX_HAPTIC_PLAY_GAIN == aw869xx->haptic_audio.ctr.play) {

				mutex_lock(&aw869xx->lock);
				aw869xx_haptic_set_gain(aw869xx, aw869xx->haptic_audio.ctr.gain);
				mutex_unlock(&aw869xx->lock);
			}
		}
	}
	mutex_unlock(&aw869xx->haptic_audio.lock);
}

/*********************************************************************************************
 *                                                                                           *
 * sys group for QQflyDriver                                                                 *
 *                                                                                           *
 *********************************************************************************************/
static void aw869xx_haptic_audio_ctr_list_insert(struct haptic_audio
		*haptic_audio, struct haptic_ctr *haptic_ctr)
{
	spin_lock(&haptic_audio->list_lock);
	INIT_LIST_HEAD(&(haptic_ctr->list));
	list_add(&(haptic_ctr->list), &(haptic_audio->ctr_list));
	spin_unlock(&haptic_audio->list_lock);

}

static void aw869xx_haptic_audio_ctr_list_clear(struct haptic_audio *haptic_audio)
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


static void aw869xx_haptic_audio_init(struct aw869xx *aw869xx)
{

	aw869xx_haptic_set_wav_seq(aw869xx, 0x01, 0x00);

}


static void aw869xx_haptic_audio_off(struct aw869xx *aw869xx)
{
	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw869xx->lock);
    aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
	aw869xx_haptic_stop(aw869xx); 
	mutex_unlock(&aw869xx->lock);

}

static void aw869xx_haptic_audio_cancel(struct aw869xx *aw869xx)
{

	pr_info("%s: haptic_audio stop\n", __func__);

	if (hrtimer_active(&aw869xx->haptic_audio.timer)) {
		pr_info("%s: cancel haptic_audio_timer\n", __func__);
		hrtimer_cancel(&aw869xx->haptic_audio.timer); 
	}
	aw869xx->haptic_audio.ctr.cnt = 0;
	aw869xx->haptic_audio.haptic_audio_cancel_flag = true;
	aw869xx_haptic_audio_ctr_list_clear(&aw869xx->haptic_audio);
}

static ssize_t aw869xx_haptic_audio_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct aw869xx *aw869xx = g_aw869xx;
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw869xx->haptic_audio.cnt);
	return len;
}

static ssize_t aw869xx_haptic_audio_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
    //struct aw869xx *aw869xx = container_of(kobj, struct aw869xx, kobjectDebug);
	struct aw869xx *aw869xx = g_aw869xx;
	unsigned int databuf[6] = { 0 };
	struct haptic_ctr *hap_ctr = NULL;

	if (!aw869xx->ram_init) {
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

		aw869xx_haptic_audio_ctr_list_insert(&aw869xx->haptic_audio, hap_ctr);
		mutex_lock(&aw869xx->haptic_audio.lock);

		if (hap_ctr->cmd == 0xff) {

			aw869xx_haptic_audio_cancel(aw869xx);
			aw869xx_haptic_audio_off(aw869xx);
			mutex_unlock(&aw869xx->haptic_audio.lock);
			return count;

		} else {
			if (aw869xx->ff_play_finish) {
				if (!hrtimer_active(&aw869xx->haptic_audio.timer)) {

					pr_info("%s: start haptic_audio_timer\n", __func__);
					aw869xx_haptic_audio_init(aw869xx);
					hrtimer_start(&aw869xx->haptic_audio.timer,
								ktime_set(aw869xx->haptic_audio.delay_val / 1000000,
										(aw869xx->haptic_audio.delay_val % 1000000) * 1000),
										HRTIMER_MODE_REL);
				} else
					pr_info("%s: audio timer is running now\n", __func__);

				aw869xx->haptic_audio.haptic_audio_cancel_flag = false;
			}

			mutex_unlock(&aw869xx->haptic_audio.lock);
			return count;
		}


	}

	pr_err("%s: sscanf failed, buf = %s\n", __func__, buf);

	return count;

}

static struct kobj_attribute vib_haptic_audio =
	__ATTR(haptic_audio, 0644, aw869xx_haptic_audio_show, aw869xx_haptic_audio_store);

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
static ssize_t aw869xx_haptic_audio_time_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "haptic_audio.delay_val=%dus\n", aw869xx->haptic_audio.delay_val);
	len += snprintf(buf+len, PAGE_SIZE-len, "haptic_audio.timer_val=%dus\n", aw869xx->haptic_audio.timer_val);
	return len;
}

static ssize_t aw869xx_haptic_audio_time_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	unsigned int databuf[2] = {0};

	if (2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
		aw869xx->haptic_audio.delay_val = databuf[0];
		aw869xx->haptic_audio.timer_val = databuf[1];
	}
	return count;
}

static ssize_t aw869xx_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	struct aw869xx_play_info *play = &aw869xx->play;
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw869xx->duration = val;
	play->playLength = val * USEC_PER_MSEC;
	play->ram_id = PLAYBACK_INFINITELY_RAM_ID;
	play->type = TIME_TYPE;

	return count;
}

static ssize_t aw869xx_activate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	struct aw869xx_play_info *play = &aw869xx->play;
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
	/* Enter standby mode */
	aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
    aw869xx_haptic_stop(aw869xx);
	aw869xx_haptic_set_repeat_wav_seq(aw869xx, PLAYBACK_INFINITELY_RAM_ID);
    play->vmax = aw869xx->default_vmax;
    aw869xx->gain = 0x80;
	aw869xx_playback_vib(aw869xx, val);
	return count;
}

static ssize_t aw869xx_iic_int_rst_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	unsigned char reg = 0;
	pr_info("%s enter\n", __func__);
	aw869xx_i2c_read(aw869xx, AW869XX_REG_ID, &reg);

	if (reg != AW86917_CHIPID) {
		pr_err("%s:chip id incorrect! reg = %d\n", __func__, reg);
		return snprintf(buf, PAGE_SIZE, "+IIC:\"0\"\n+INT:\"1\"\n+RST:\"1\"\n");
	}
	return snprintf(buf, PAGE_SIZE, "+IIC:\"1\"\n+INT:\"1\"\n+RST:\"1\"\n");
}

/*1040信号马达不用校准，不用此节点*/
static ssize_t aw869xx_f0_offset_10_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);

	unsigned int databuf[1] = {0};
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;

    if (aw869xx->lra_information == AW869XX_LRA_1040) {
		pr_info("%s:enter;aw869xx->lra_information is 1040, no need cali\n", __func__);
	} else {
	
    	pr_info("%s:enter;buf is %s\n", __func__, buf);
    	if (1 == sscanf(buf, "%d", &databuf[0])) {
    		mutex_lock(&aw869xx->lock);

    		at_test = true;

    		aw869xx->f0_cali_flag = AW869XX_HAPTIC_CALI_F0;
    		//aw869xx_i2c_write(aw869xx, AW869XX_REG_TRIM_LRA, 0x00);
    		pr_err("%s user current f0\n", __func__);

    		switch (databuf[0]) {

    		case 0:
    			f0_limit = aw869xx->f0;
    			break;
    		case 10:
    			f0_limit = aw869xx->f0 + 100;
    			break;
    		case -10:
    			f0_limit = aw869xx->f0 - 100;
    			break;
    		default:
    			f0_limit = aw869xx->f0;
    			break;
    		}
    		pr_debug("%s f0_pre = %d\n", __func__, aw869xx->f0_pre);

    	/* calculate cali step */
    		f0_cali_step = 100000*((int)f0_limit-(int)aw869xx->f0_pre) / ((int)f0_limit*25);
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
    		atomic_set(&aw869xx->f0_freq_cali, f0_cali_lra);
    		/* update cali step */
    		//aw869xx_i2c_write(aw869xx, AW869XX_REG_TRIM_LRA, (char)f0_cali_lra);
    		//aw869xx_i2c_read(aw869xx, AW869XX_REG_TRIM_LRA, &reg_val);
    		pr_info("%s final trim_lra=0x%02x\n", __func__, reg_val); 
            /* restore default work mode */
    		aw869xx_haptic_play_mode(aw869xx, AW869XX_HAPTIC_STANDBY_MODE);
    		aw869xx->play_mode = AW869XX_HAPTIC_RAM_MODE;
    		//aw869xx_i2c_write_bits(aw869xx, AW869XX_REG_SYSCTRL,
    				//AW869XX_BIT_SYSCTRL_PLAY_MODE_MASK, AW869XX_BIT_SYSCTRL_PLAY_MODE_RAM);
    		aw869xx_haptic_stop(aw869xx);

    		mdelay(100);
    		pr_info("%s set freq to %dHZ\n", __func__, f0_limit); 

    		at_test = false;

    		mutex_unlock(&aw869xx->lock);

    	}
	}
	return count;
}

static ssize_t aw869xx_is_need_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);

	ssize_t len = 0;
	int need_cali = 1;
	char lra[] = "X-LRA 0619";

	switch (aw869xx->lra_information) {

          case AW869XX_LRA_0815:
                  strcpy(lra, "X-LRA 0815");
                  need_cali = 1;
                  break;
          case AW869XX_LRA_1040:
                  strcpy(lra, "Z-LRA 1040");
                  need_cali = 0;
                  break;
          case AW869XX_LRA_0832:
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

/* 强制进行线性马达校准项
 * AT+BK_VBR_CAL=4 
 *【指令】：cat /sys/class/leds/vibrator/cali_f0_resis
 * Z轴线性马达不需要校准，校准反而可能会影响震动效果
 * 故offset值返回0，且不进行校准，只读f0
 */

static ssize_t aw869xx_cali_f0_resis_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);

	//unsigned char reg_val = 0;
	ssize_t len = 0;
	mutex_lock(&aw869xx->lock);

	at_test = true;

	//get resistance
	aw869xx_haptic_get_lra_resistance(aw869xx);

	if ((aw869xx->lra >= aw869xx->resistance_min) && (aw869xx->lra <= aw869xx->resistance_max)) {
		pr_err("%s lra resistent test ok, lra=%d\n", __func__, aw869xx->lra);
	} else {
		pr_err("%s lra resistent over range, lra=%d\n", __func__, aw869xx->lra);
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw869xx->lra >= aw869xx->resistance_min && aw869xx->lra <= aw869xx->resistance_max) ? "ok" : "fail",
		aw869xx->lra/1000, aw869xx->lra%1000,
		aw869xx->resistance_min/1000, aw869xx->resistance_min%1000/100,
		aw869xx->resistance_max/1000, aw869xx->resistance_max%1000/100);


	if (aw869xx->lra_information == AW869XX_LRA_1040) {

		pr_info("%s:enter;aw869xx->lra_information is 1040\n", __func__);

		len += snprintf(buf+len, PAGE_SIZE-len, "freqency#ok#170.0#%d.%d-%d.%d#hz\n",
			aw869xx->freq_min/10, aw869xx->freq_min%10, aw869xx->freq_max/10, aw869xx->freq_max%10);

		len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#ok#0\n",
			(aw869xx->f0 >= aw869xx->freq_min && aw869xx->f0 <= aw869xx->freq_max) ? "ok" : "fail",
			atomic_read(&aw869xx->f0_freq_cali));

	} else {

        mdelay(20);
        
    	aw869xx_haptic_f0_calibration(aw869xx);

    	mdelay(200);

    	aw869xx->f0_cali_flag = AW869XX_HAPTIC_LRA_F0;
    	aw869xx_haptic_cont_get_f0(aw869xx);

    	len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
    		(aw869xx->f0 >= aw869xx->freq_min && aw869xx->f0 <= aw869xx->freq_max) ? "ok" : "fail",
    		aw869xx->f0/10, aw869xx->f0%10,
    		aw869xx->freq_min/10, aw869xx->freq_min%10, aw869xx->freq_max/10, aw869xx->freq_max%10);

        len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#%s#%d\n",
  		    (aw869xx->f0 >= aw869xx->freq_min && aw869xx->f0 <= aw869xx->freq_max) ? "ok" : "fail",
  		    atomic_read(&aw869xx->f0_freq_cali));
	}

	at_test = false;
	mutex_unlock(&aw869xx->lock);

	return len;
}

/* 线性马达工模测试项
 * AT+BK_VBR_TEST=1
 *【指令】：cat /sys/class/leds/vibrator/resis_f0
 * Z轴线性马达不需要校准，校准反而可能会影响震动效果
 * 测试并返回马达的阻抗（单位为Ω）和谐振频率（单位为Hz），
 * 不进行校准，不保存NV参数，执行失败返回ATNG，执行成功返回ATOK。
 */
static ssize_t aw869xx_resis_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);

	//unsigned char reg_val = 0;
	ssize_t len = 0;
	mutex_lock(&aw869xx->lock);

	at_test = true;

	aw869xx_haptic_get_lra_resistance(aw869xx);

	if ((aw869xx->lra >= aw869xx->resistance_min) && (aw869xx->lra <= aw869xx->resistance_max)) {
		pr_err("%s lra resistent test ok, lra=%d\n", __func__, aw869xx->lra);
	} else {
		pr_err("%s lra resistent over range, lra=%d\n", __func__, aw869xx->lra);
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw869xx->lra >= aw869xx->resistance_min && aw869xx->lra <= aw869xx->resistance_max) ? "ok" : "fail",
		aw869xx->lra/1000, aw869xx->lra%1000,
		aw869xx->resistance_min/1000, aw869xx->resistance_min%1000/100,
		aw869xx->resistance_max/1000, aw869xx->resistance_max%1000/100);

	mdelay(20);
    #ifdef USE_RAM_CALI_F0
    aw869xx->f0_cali_flag = AW869XX_HAPTIC_CALI_F0;
    aw869xx_haptic_upload_lra(aw869xx, F0_CALI);
    aw869xx_haptic_ram_get_f0(aw869xx);
    #endif
    aw869xx->f0_cali_flag = AW869XX_HAPTIC_LRA_F0;
    aw869xx_haptic_cont_get_f0(aw869xx);

	len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
		(aw869xx->f0 >= aw869xx->freq_min && aw869xx->f0 <= aw869xx->freq_max) ? "ok" : "fail",
		aw869xx->f0/10, aw869xx->f0%10,
		aw869xx->freq_min/10, aw869xx->freq_min%10, aw869xx->freq_max/10, aw869xx->freq_max%10);

	at_test = false;

	mutex_unlock(&aw869xx->lock);

	return len;
}

/* 售后工具线性马达校准项
 * AT+BK_VBR_CAL=1  
 *【指令】：cat /sys/class/leds/vibrator/cali_f0
 * Z轴线性马达不需要校准，校准反而可能会影响震动效果
 * 故offset值返回0，且不进行校准，只读f0
 */
static ssize_t aw869xx_cali_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);

	ssize_t len = 0;
	mutex_lock(&aw869xx->lock);

	at_test = true;

	if (aw869xx->lra_information == AW869XX_LRA_1040) {

		pr_info("%s:enter;aw869xx->lra_information is 1040\n", __func__);

		len += snprintf(buf+len, PAGE_SIZE-len, "ok f0 170.0 (range:%d.%d-%d.%d)hz f0_offset=0\n",
			aw869xx->freq_min/10, aw869xx->freq_min%10, aw869xx->freq_max/10, aw869xx->freq_max%10);

	} else {

		pr_info("%s:enter;aw869xx->lra_information is not 1040\n", __func__);

		aw869xx_haptic_f0_calibration(aw869xx);

		mdelay(200);

		aw869xx->f0_cali_flag = AW869XX_HAPTIC_LRA_F0;
    	aw869xx_haptic_cont_get_f0(aw869xx);

		len += snprintf(buf+len, PAGE_SIZE-len, "%s f0 %d.%d (range:%d.%d-%d.%d)hz f0_offset=%d\n",
			(aw869xx->f0 >= aw869xx->freq_min && aw869xx->f0 <= aw869xx->freq_max) ? "ok" : "fail",
			aw869xx->f0/10, aw869xx->f0%10,
			aw869xx->freq_min/10, aw869xx->freq_min%10, aw869xx->freq_max/10, aw869xx->freq_max%10,
			atomic_read(&aw869xx->f0_freq_cali));
	}

	at_test = false;

	mutex_unlock(&aw869xx->lock);

	return len;
}

static ssize_t aw869xx_auto_brake_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_stop(aw869xx);
	aw869xx_haptic_auto_brk_enable(aw869xx, val);
	mutex_unlock(&aw869xx->lock);

	return count;
}

static ssize_t aw869xx_auto_boost_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw869xx->lock);
	aw869xx_haptic_stop(aw869xx);
	aw869xx_haptic_auto_bst_enable(aw869xx, val);
	mutex_unlock(&aw869xx->lock);

	return count;
}


static ssize_t aw869xx_i2c_reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);


	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw869xx_i2c_write(aw869xx, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw869xx_i2c_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);

	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for (i = 0; i < AW869XX_REG_MAX; i++) {
		if (!(aw869xx_reg_access[i]&REG_RD_ACCESS))
		   continue;
		aw869xx_i2c_read(aw869xx, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}
                
static ssize_t aw869xx_ram_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	/* RAMINIT Enable */
	aw869xx_haptic_raminit(aw869xx, true);
	aw869xx_haptic_stop(aw869xx);
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMADDRH, (unsigned char)(aw869xx->ram.base_addr >> 8));
	aw869xx_i2c_write(aw869xx, AW869XX_REG_RAMADDRL, (unsigned char)(aw869xx->ram.base_addr & 0x00ff));
	len += snprintf(buf + len, PAGE_SIZE - len, "aw869xx_haptic_ram:\n");
	for (i = 0; i < aw869xx->ram.len; i++) {
		aw869xx_i2c_read(aw869xx, AW869XX_REG_RAMDATA, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02X,", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	/* RAMINIT Disable */
	aw869xx_haptic_raminit(aw869xx, false);
	return len;
}

static ssize_t aw869xx_ram_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw869xx *aw869xx = container_of(cdev, struct aw869xx, cdev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val)
		aw869xx_ram_update(aw869xx);
	return count;
}


//add for QQflyDriver
//static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, aw869xx_haptic_audio_show, aw869xx_haptic_audio_store);
static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO, aw869xx_haptic_audio_time_show, aw869xx_haptic_audio_time_store);
//add for gongmo
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, NULL, aw869xx_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, NULL, aw869xx_activate_store);
static DEVICE_ATTR(iic_int_rst, S_IWUSR | S_IRUGO, aw869xx_iic_int_rst_show, NULL);
static DEVICE_ATTR(f0_offset_10, S_IWUSR | S_IRUGO, NULL, aw869xx_f0_offset_10_store);
static DEVICE_ATTR(is_need_cali, S_IWUSR | S_IRUGO, aw869xx_is_need_cali_show, NULL);
static DEVICE_ATTR(cali_f0_resis, S_IWUSR | S_IRUGO, aw869xx_cali_f0_resis_show, NULL);
static DEVICE_ATTR(resis_f0, S_IWUSR | S_IRUGO, aw869xx_resis_f0_show, NULL);
static DEVICE_ATTR(cali_f0, S_IWUSR | S_IRUGO, aw869xx_cali_f0_show, NULL);
static DEVICE_ATTR(auto_brake, S_IWUSR | S_IRUGO, NULL, aw869xx_auto_brake_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, NULL, aw869xx_auto_boost_store);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw869xx_i2c_reg_show, aw869xx_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw869xx_ram_show, aw869xx_ram_store);

static struct attribute *aw869xx_attributes[] = {
	//&dev_attr_haptic_audio.attr,
	&dev_attr_haptic_audio_time.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_iic_int_rst.attr,
	&dev_attr_f0_offset_10.attr,
	&dev_attr_is_need_cali.attr,
	&dev_attr_cali_f0_resis.attr,
	&dev_attr_resis_f0.attr,
	&dev_attr_cali_f0.attr,
	&dev_attr_auto_brake.attr,
	&dev_attr_auto_boost.attr,
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};

static struct attribute_group aw869xx_attribute_group = {
	.attrs = aw869xx_attributes
};

/******************************************************
 *
 * i2c driver probe
 *
 ******************************************************/
static int aw869xx_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw869xx *aw869xx;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;
    unsigned char reg = 0;

	pr_info("%s enter\n", __func__);
    i2c->addr = 0x5A;
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw869xx = devm_kzalloc(&i2c->dev, sizeof(struct aw869xx), GFP_KERNEL);
	if (aw869xx == NULL)
		return -ENOMEM;

	aw869xx->dev = &i2c->dev;
	aw869xx->i2c = i2c;

	i2c_set_clientdata(i2c, aw869xx);
    /* init bus_lock first,it was ues in read_chipid */
    mutex_init(&aw869xx->bus_lock);
	/* aw869xx rst & int */
	if (np) {
		ret = aw869xx_parse_rst_irq_gpio_dt(&i2c->dev, aw869xx, np);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to parse rst or int gpio node\n", __func__);
			goto err_parse_gpio_dt;
		}
	} else {
		aw869xx->reset_gpio = -1;
		aw869xx->irq_gpio = -1;
	}

	if (gpio_is_valid(aw869xx->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw869xx->reset_gpio,
			GPIOF_OUT_INIT_HIGH, "aw869xx_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
			goto err_reset_gpio_request;
		}
	}

	if (gpio_is_valid(aw869xx->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw869xx->irq_gpio,
			GPIOF_DIR_IN, "aw869xx_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n", __func__);
			goto err_irq_gpio_request;
		}
	}

	/* aw869xx chip id */
	ret = aw869xx_read_chipid(aw869xx);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw869xx_read_chipid failed ret=%d\n", __func__, ret);
		goto err_id;
	}
    /* aw869xx parse dt */
    if (np) {
		ret = aw869xx_parse_dt(&i2c->dev, aw869xx, np);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
			goto err_id;
		}
	}
    /* chip qualify */
	ret = aw869xx_i2c_read(aw869xx, 0x64, &reg);
	if (ret < 0) {
		dev_err(&i2c->dev,
			   "%s: failed to read register 0x64: %d\n",
			   __func__, ret);
	}
	if (!(reg & 0x80)) {
		dev_err(&i2c->dev, "%s:unqualified chip!\n", __func__);
		goto err_qualify;
	}

	/* aw869xx irq */
	if (gpio_is_valid(aw869xx->irq_gpio) &&
		!(aw869xx->flags & AW869XX_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
					gpio_to_irq(aw869xx->irq_gpio),
					NULL, aw869xx_irq, irq_flags,
					"aw869xx", aw869xx);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
					__func__, gpio_to_irq(aw869xx->irq_gpio), ret);
			return -ENODEV;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw869xx->flags |= AW869XX_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw869xx);

	g_aw869xx = aw869xx;

	ret = aw869xx_vibrator_init(aw869xx);
	if (ret) {
		pr_err("%s: vibrator init failed, ret=%d\n", __func__, ret);
		return -EFAULT;
	}

	aw869xx_haptic_init(aw869xx);

	aw869xx_ram_init(aw869xx);

	aw869xx->cdev.name = "vibrator";
	aw869xx->cdev.brightness_get = aw869xx_haptic_brightness_get;
	aw869xx->cdev.brightness_set = aw869xx_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw869xx->i2c->dev, &aw869xx->cdev);
	if (ret < 0) {
		dev_err(aw869xx->dev, "%s: fail to create led dev\n",
				__func__);
		return ret;
	}
	ret = sysfs_create_group(&aw869xx->cdev.dev->kobj, &aw869xx_attribute_group);
	if (ret < 0) {
		dev_err(aw869xx->dev, "%s error creating sysfs attr files\n", __func__);
		return ret;
	 }

    //QQ fly driver node
    ret = kobject_init_and_add(&aw869xx->kobjectDebug, &vibrator_debug_object_type, NULL, "vibrator");
	if (ret) {
		pr_err("%s create vibrator node error!", __func__);
		ret = -1;
		return ret;
	}

	pr_info("%s probe completed successfully!\n", __func__);
    return 0;
 err_qualify:
 err_id:
 err_irq_gpio_request:
    if (gpio_is_valid(aw869xx->irq_gpio))
		devm_gpio_free(&i2c->dev, aw869xx->irq_gpio);
 err_reset_gpio_request:
    if (gpio_is_valid(aw869xx->reset_gpio))
                devm_gpio_free(&i2c->dev, aw869xx->reset_gpio);
 err_parse_gpio_dt:
    devm_kfree(&i2c->dev, aw869xx);
    aw869xx = NULL;

	return -ENODEV;
}

static int aw869xx_i2c_remove(struct i2c_client *i2c)
{
	struct aw869xx *aw869xx = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	misc_deregister(&aw869xx_haptic_misc);

	sysfs_remove_group(&i2c->dev.kobj, &aw869xx_attribute_group);

//	devm_led_classdev_unregister(&aw869xx->i2c->dev, &aw869xx->cdev);

	wake_lock_destroy(&aw869xx->wake_lock);

	kmem_cache_destroy(rtp_cachep);
	rtp_cachep = NULL;

	destroy_workqueue(rtp_wq);

	return 0;
}

static void aw869xx_pm_shutdown(struct i2c_client *i2c)
{
	//struct aw869xx *aw869xx = i2c_get_clientdata(i2c);

	//aw869xx->trig[1].enable = 0;
	//aw869xx->trig[2].enable = 0;
	pr_info("%s enter\n", __func__);
	//aw869xx_haptic_trig_enable_config(aw869xx);
}

static int aw869xx_pm_suspend (struct device *dev)
{
	i2c_suspend = 1;
	pr_info("%s enter, pm_suspend_flag = %d\n", __func__, i2c_suspend);

	return 0;

}


static int aw869xx_pm_resume (struct device *dev)
{

	i2c_suspend = 0;
	pr_info("%s enter, pm_suspend_flag = %d\n", __func__, i2c_suspend);
	return 0;


}

static struct dev_pm_ops aw869xx_pm_ops = {
	.suspend = aw869xx_pm_suspend,
	.resume = aw869xx_pm_resume,
};


static const struct i2c_device_id aw869xx_i2c_id[] = {
	{ AW869XX_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw869xx_i2c_id);

static struct of_device_id aw869xx_dt_match[] = {
	{ .compatible = "awinic,haptic" },
	{ },
};

static struct i2c_driver aw869xx_i2c_driver = {
	.driver = {
		.name = AW869XX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw869xx_dt_match),
		.pm = &aw869xx_pm_ops,
	},
	.shutdown = aw869xx_pm_shutdown,
	.probe = aw869xx_i2c_probe,
	.remove = aw869xx_i2c_remove,
	.id_table = aw869xx_i2c_id,
};

// 产线在SMT阶段，增加iic通路at测试，故移除at模式下不加载马达驱动的限制
//extern unsigned int is_atboot;
static int __init aw869xx_i2c_init(void)
{
	int ret = 0;

	pr_info("aw869xx driver version %s\n", AW869XX_DRIVER_VERSION);
/*
	if (is_atboot == 1) {
		pr_err("%s now is at mode, not load lra vibrator driver\n", __func__);
		return 0;
	} else {
		pr_err("%s driver load normal\n", __func__);
	}
*/
	ret = i2c_add_driver(&aw869xx_i2c_driver);
	if (ret) {
		pr_err("aw869xx fail to add device into i2c\n");
		return ret;
	}

	return 0;
}
module_init(aw869xx_i2c_init);


static void __exit aw869xx_i2c_exit(void)
{
	i2c_del_driver(&aw869xx_i2c_driver);
}
module_exit(aw869xx_i2c_exit);


MODULE_DESCRIPTION("AW869XX Haptic Driver");
MODULE_LICENSE("GPL v2");
