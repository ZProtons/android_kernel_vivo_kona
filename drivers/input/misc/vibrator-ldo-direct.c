/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
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
#define DEBUG
#define pr_fmt(fmt) "[qti_haptics] %s[L %d] :" fmt, __func__, __LINE__
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include "../../soc/vivo/inc/timed_output.h"
/*added by vivo sensor team, for vivo haptic printf pid start*/
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/export.h>
#include <asm/ptrace.h>
#include <linux/printk.h>
/*added by vivo sensor team, for vivo haptic printf pid end*/

#define VIVO_HAPTIC_DEBUG

#ifdef VIVO_HAPTIC_DEBUG
#define HAPTIC_LOG(format, args...) pr_info(format, ##args)
#else
#define HAPTIC_LOG(format, args...)
#endif


enum haptics_custom_effect_param {
	CUSTOM_DATA_EFFECT_IDX,
	CUSTOM_DATA_TIMEOUT_SEC_IDX,
	CUSTOM_DATA_TIMEOUT_MSEC_IDX,
	CUSTOM_DATA_LEN,
};

/* common definitions */
#define HAP_BRAKE_PATTERN_MAX		4
#define HAP_WAVEFORM_BUFFER_MAX		8
#define HAP_VMAX_MV_DEFAULT		1800
#define HAP_VMAX_MV_MAX			3596
#define HAP_PLAY_RATE_US_DEFAULT	5715
#define HAP_PLAY_RATE_US_MAX		20475
#define FF_EFFECT_COUNT_MAX		32

struct qti_hap_effect {
	int			id;
	u8			*pattern;
	int			pattern_length;
	u16			play_rate_us;
	u16			vmax_mv;
	u8			wf_repeat_n;
	u8			wf_s_repeat_n;
	bool			lra_auto_res_disable;
};

struct qti_hap_play_info {
	struct qti_hap_effect	*effect;
	u16			vmax_mv;
	int			length_us;
};

struct qti_hap_config {
	u16			vmax_mv;
	u16			play_rate_us;
};

struct qti_hap_chip {
	struct platform_device		*pdev;
	struct device			*dev;
	struct regmap			*regmap;
	struct input_dev		*input_dev;
	struct timed_output_dev     timed_dev;
	struct qti_hap_config		config;
	struct qti_hap_play_info	play;
	struct qti_hap_effect		*predefined;
	struct qti_hap_effect		constant;
	struct regulator		*vdd_supply;
	struct hrtimer			stop_timer;
	struct work_struct		vibrator_work_on;
	struct work_struct		vibrator_work_off;
	struct mutex	enable_lock;
	int				effects_count;
	bool				vdd_enabled;
	struct mutex vdd_mutex;
};

static int wf_repeat[8] = {1, 2, 4, 8, 16, 32, 64, 128};
static int wf_s_repeat[4] = {1, 2, 4, 8};

static int qti_haptics_load_predefined_effect(struct qti_hap_chip *chip,
		int effect_idx)
{
	struct qti_hap_play_info *play = &chip->play;

	if (effect_idx >= chip->effects_count)
		return -EINVAL;

	play->effect = &chip->predefined[effect_idx];
	return 0;
}

static inline void get_play_length(struct qti_hap_play_info *play,
		int *length_us)
{
	struct qti_hap_effect *effect = play->effect;
	int tmp;

	tmp = effect->pattern_length * effect->play_rate_us;
	tmp *= wf_s_repeat[effect->wf_s_repeat_n];
	tmp *= wf_repeat[effect->wf_repeat_n];

	*length_us = tmp;
}

static int qti_haptics_upload_effect(struct input_dev *dev,
		struct ff_effect *effect, struct ff_effect *old)
{
	struct qti_hap_chip *chip = input_get_drvdata(dev);
	struct qti_hap_config *config = &chip->config;
	struct qti_hap_play_info *play = &chip->play;
	int rc = 0, tmp, i;
	s16 level, data[CUSTOM_DATA_LEN];

	switch (effect->type) {
	case FF_CONSTANT:
		play->length_us = effect->replay.length * USEC_PER_MSEC;
		level = effect->u.constant.level;
		tmp = level * config->vmax_mv;
		play->vmax_mv = tmp / 0x7fff;
		dev_dbg(chip->dev, "upload constant effect, length = %dus\n",
				play->length_us);

		break;

	case FF_PERIODIC:
		if (chip->effects_count == 0)
			return -EINVAL;

		if (effect->u.periodic.waveform != FF_CUSTOM) {
			dev_err(chip->dev, "Only accept custom waveforms\n");
			return -EINVAL;
		}

		if (copy_from_user(data, effect->u.periodic.custom_data,
					sizeof(s16) * CUSTOM_DATA_LEN))
			return -EFAULT;

		for (i = 0; i < chip->effects_count; i++)
			if (chip->predefined[i].id ==
					data[CUSTOM_DATA_EFFECT_IDX])
				break;

		if (i == chip->effects_count) {
			dev_err(chip->dev, "predefined effect %d is NOT supported\n",
					data[0]);
			return -EINVAL;
		}

		level = effect->u.periodic.magnitude;
		tmp = level * chip->predefined[i].vmax_mv;
		play->vmax_mv = tmp / 0x7fff;

		dev_dbg(chip->dev, "upload effect %d, vmax_mv=%d\n",
				chip->predefined[i].id, play->vmax_mv);
		rc = qti_haptics_load_predefined_effect(chip, i);
		if (rc < 0) {
			dev_err(chip->dev, "Play predefined effect %d failed, rc=%d\n",
					chip->predefined[i].id, rc);
			return rc;
		}

		get_play_length(play, &play->length_us);
		data[CUSTOM_DATA_TIMEOUT_SEC_IDX] =
			play->length_us / USEC_PER_SEC;
		data[CUSTOM_DATA_TIMEOUT_MSEC_IDX] =
			(play->length_us % USEC_PER_SEC) / USEC_PER_MSEC;

		/*
		 * Copy the custom data contains the play length back to
		 * userspace so that the userspace client can wait and
		 * send stop playing command after it's done.
		 */
		if (copy_to_user(effect->u.periodic.custom_data, data,
					sizeof(s16) * CUSTOM_DATA_LEN))
			return -EFAULT;
		break;

	default:
		dev_err(chip->dev, "Unsupported effect type: %d\n",
				effect->type);
		return -EINVAL;
	}

	return 0;
}

static int qti_haptics_playback(struct input_dev *dev, int effect_id, int val)
{
	struct qti_hap_chip *chip = input_get_drvdata(dev);
	struct qti_hap_play_info *play = &chip->play;
	s64 secs;
	unsigned long nsecs;

	dev_dbg(chip->dev, "playback, val = %d length = %dus\n",
			val, play->length_us);
	if (!!val) {
			schedule_work(&chip->vibrator_work_on);
			secs = play->length_us / USEC_PER_SEC;
			nsecs = (play->length_us % USEC_PER_SEC) *
				NSEC_PER_USEC;
			hrtimer_start(&chip->stop_timer, ktime_set(secs, nsecs),
					HRTIMER_MODE_REL);
	} else {
		play->length_us = 0;
		schedule_work(&chip->vibrator_work_off);
	}

	return 0;
}

static void qti_haptics_set_gain(struct input_dev *dev, u16 gain)
{
	struct qti_hap_chip *chip = input_get_drvdata(dev);
	struct qti_hap_config *config = &chip->config;
	struct qti_hap_play_info *play = &chip->play;

	if (gain == 0)
		return;

	if (gain > 0x7fff)
		gain = 0x7fff;

	play->vmax_mv = ((u32)(gain * config->vmax_mv)) / 0x7fff;
}

static enum hrtimer_restart qti_hap_stop_timer(struct hrtimer *timer)
{
	struct qti_hap_chip *chip = container_of(timer, struct qti_hap_chip,
			stop_timer);
	chip->play.length_us = 0;
	schedule_work(&chip->vibrator_work_off);
	return HRTIMER_NORESTART;
}

static void qti_haptics_work_off(struct work_struct *work)
{
	struct qti_hap_chip *chip = container_of(work, struct qti_hap_chip, vibrator_work_off);
	int rc = 0;
	dev_err(chip->dev, "qti_haptics_work_off enter\n");
	mutex_lock(&chip->vdd_mutex);
	if (chip->vdd_supply && chip->vdd_enabled) {
		rc = regulator_disable(chip->vdd_supply);
		if (rc < 0) {
			dev_err(chip->dev, "Disable VDD supply failed, rc=%d\n", rc);
		} else {
			dev_err(chip->dev, "Disable VDD supply success");
			chip->vdd_enabled = false;
		}
	}
	mutex_unlock(&chip->vdd_mutex);
}

static void qti_haptics_work_on(struct work_struct *work)
{
	struct qti_hap_chip *chip = container_of(work, struct qti_hap_chip, vibrator_work_on);
	int rc = 0;
	dev_err(chip->dev, "qti_haptics_work_on enter, pid is %d\n", current->pid);
	mutex_lock(&chip->vdd_mutex);
	if (chip->vdd_supply && !chip->vdd_enabled) {
		int vddValue = -1;

		rc = regulator_set_load(chip->vdd_supply, 100000);
		if (rc < 0)
			dev_err(chip->dev, "failed to set load vdd vol: rc=%d\n", rc);

		rc = regulator_enable(chip->vdd_supply);
		if (rc < 0)
			dev_err(chip->dev, "failed to enable vdd: %d\n", rc);

		vddValue = regulator_get_voltage(chip->vdd_supply);
		if (vddValue < 0)
			dev_err(chip->dev, "get vdd vol failed\n");

		dev_err(chip->dev, "get vdd vol = %d\n", vddValue);

		if (rc < 0) {
			dev_err(chip->dev, "Enable VDD supply failed, rc=%d\n", rc);
		} else {
			dev_err(chip->dev, "Enable VDD supply success\n");
			chip->vdd_enabled = true;
		}
	}
	mutex_unlock(&chip->vdd_mutex);
}

static int qti_haptics_parse_dt(struct qti_hap_chip *chip)
{
	struct qti_hap_config *config = &chip->config;
	const struct device_node *node = chip->dev->of_node;
	struct device_node *child_node;
	struct qti_hap_effect *effect;
	int rc = 0, tmp, i = 0, j, m;

	dev_err(chip->dev, "enter qti haptics parse dt");

	config->vmax_mv = HAP_VMAX_MV_DEFAULT;
	rc = of_property_read_u32(node, "qcom,vmax-mv", &tmp);
	if (!rc)
		config->vmax_mv = (tmp > HAP_VMAX_MV_MAX) ?
			HAP_VMAX_MV_MAX : tmp;

	config->play_rate_us = HAP_PLAY_RATE_US_DEFAULT;
	rc = of_property_read_u32(node, "qcom,play-rate-us", &tmp);
	if (!rc)
		config->play_rate_us = (tmp >= HAP_PLAY_RATE_US_MAX) ?
			HAP_PLAY_RATE_US_MAX : tmp;


	if (of_find_property(node, "vdd-supply", NULL)) {
		chip->vdd_supply = devm_regulator_get(chip->dev, "vdd");
		if (IS_ERR(chip->vdd_supply)) {
			rc = PTR_ERR(chip->vdd_supply);
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "Failed to get vdd regulator");
			return rc;
		}
		rc = regulator_set_voltage(chip->vdd_supply, 2700000, 2700000);
		if (rc)
			dev_err(chip->dev, "failed to set vol vdd vol\n");
	} else {
		dev_err(chip->dev, "can't find vdd-supply\n");
	}

	chip->constant.pattern = devm_kcalloc(chip->dev,
			HAP_WAVEFORM_BUFFER_MAX,
			sizeof(u8), GFP_KERNEL);
	if (!chip->constant.pattern)
		return -ENOMEM;

	tmp = of_get_available_child_count(node);
	if (tmp == 0)
		return 0;

	chip->predefined = devm_kcalloc(chip->dev, tmp,
			sizeof(*chip->predefined), GFP_KERNEL);
	if (!chip->predefined)
		return -ENOMEM;

	chip->effects_count = tmp;

	for_each_available_child_of_node(node, child_node) {
		effect = &chip->predefined[i++];
		rc = of_property_read_u32(child_node, "qcom,effect-id",
				&effect->id);
		if (rc < 0) {
			dev_err(chip->dev, "Read qcom,effect-id failed, rc=%d\n",
					rc);
			return rc;
		}

		effect->vmax_mv = config->vmax_mv;
		rc = of_property_read_u32(child_node, "qcom,wf-vmax-mv", &tmp);
		if (rc < 0)
			dev_dbg(chip->dev, "Read qcom,wf-vmax-mv failed, rc=%d\n",
					rc);
		else
			effect->vmax_mv = (tmp > HAP_VMAX_MV_MAX) ?
				HAP_VMAX_MV_MAX : tmp;

		rc = of_property_count_elems_of_size(child_node,
				"qcom,wf-pattern", sizeof(u8));
		if (rc < 0) {
			dev_err(chip->dev, "Count qcom,wf-pattern property failed, rc=%d\n",
					rc);
			return rc;
		} else if (rc == 0) {
			dev_dbg(chip->dev, "qcom,wf-pattern has no data\n");
			return -EINVAL;
		}

		effect->pattern_length = rc;
		effect->pattern = devm_kcalloc(chip->dev,
				effect->pattern_length, sizeof(u8), GFP_KERNEL);
		if (!effect->pattern)
			return -ENOMEM;

		rc = of_property_read_u8_array(child_node, "qcom,wf-pattern",
				effect->pattern, effect->pattern_length);
		if (rc < 0) {
			dev_err(chip->dev, "Read qcom,wf-pattern property failed, rc=%d\n",
					rc);
			return rc;
		}

		effect->play_rate_us = config->play_rate_us;
		rc = of_property_read_u32(child_node, "qcom,wf-play-rate-us",
				&tmp);
		if (rc < 0)
			dev_dbg(chip->dev, "Read qcom,wf-play-rate-us failed, rc=%d\n",
					rc);
		else
			effect->play_rate_us = tmp;

		rc = of_property_read_u32(child_node, "qcom,wf-repeat-count",
				&tmp);
		if (rc < 0) {
			dev_dbg(chip->dev, "Read qcom,wf-repeat-count failed, rc=%d\n",
					rc);
		} else {
			for (j = 0; j < ARRAY_SIZE(wf_repeat); j++)
				if (tmp <= wf_repeat[j])
					break;

			effect->wf_repeat_n = j;
		}

		rc = of_property_read_u32(child_node, "qcom,wf-s-repeat-count",
				&tmp);
		if (rc < 0) {
			dev_dbg(chip->dev, "Read qcom,wf-s-repeat-count failed, rc=%d\n",
					rc);
		} else {
			for (j = 0; j < ARRAY_SIZE(wf_s_repeat); j++)
				if (tmp <= wf_s_repeat[j])
					break;

			effect->wf_s_repeat_n = j;
		}

	}

	for (j = 0; j < i; j++) {
		dev_dbg(chip->dev, "effect: %d\n", chip->predefined[j].id);
		dev_dbg(chip->dev, "        vmax: %d mv\n",
				chip->predefined[j].vmax_mv);
		dev_dbg(chip->dev, "        play_rate: %d us\n",
				chip->predefined[j].play_rate_us);
		for (m = 0; m < chip->predefined[j].pattern_length; m++)
			dev_dbg(chip->dev, "        pattern[%d]: 0x%x\n",
					m, chip->predefined[j].pattern[m]);
		dev_dbg(chip->dev, "    wf_repeat_n: %d\n",
				chip->predefined[j].wf_repeat_n);
		dev_dbg(chip->dev, "    wf_s_repeat_n: %d\n",
				chip->predefined[j].wf_s_repeat_n);
	}

	return 0;
}

static int qti_haptics_activate(struct qti_hap_chip *chip, int val)
{
	struct qti_hap_play_info *play = &chip->play;
	s64 secs;
	unsigned long nsecs;

	pr_info("[haptic] %s enter\n", __func__);

	dev_dbg(chip->dev, "playback, val = %d length = %dus\n",
			val, play->length_us);
	if (!!val) {
			schedule_work(&chip->vibrator_work_on);
			secs = play->length_us / USEC_PER_SEC;
			nsecs = (play->length_us % USEC_PER_SEC) *
				NSEC_PER_USEC;
			hrtimer_start(&chip->stop_timer, ktime_set(secs, nsecs),
					HRTIMER_MODE_REL);
	} else {
		play->length_us = 0;
		schedule_work(&chip->vibrator_work_off);
	}

	return 0;
}

static ssize_t qti_haptics_activate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qti_hap_chip *chip = container_of(tdev, struct qti_hap_chip, timed_dev);

	return snprintf(buf, PAGE_SIZE, "vdd_enabled = %d\n", chip->vdd_enabled);
}

static ssize_t qti_haptics_active_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qti_hap_chip *chip = container_of(tdev, struct qti_hap_chip, timed_dev);
	int rc = 0, val = 0;

	HAPTIC_LOG("enter\n");
	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val == 0) {
		qti_haptics_activate(chip, 0);
	} else {
		pr_err("qti_haptics_active, val = %d\n", val);
		qti_haptics_activate(chip, 1);
	}
	return count;
}
/*add the at interface*/

static ssize_t qti_haptics_duration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qti_hap_chip *chip = container_of(tdev, struct qti_hap_chip, timed_dev);

	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&chip->stop_timer)) {
		time_rem = hrtimer_get_remaining(&chip->stop_timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "remaining time %lld\n", time_ms);

}
static ssize_t qti_haptics_duration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qti_hap_chip *chip = container_of(tdev, struct qti_hap_chip, timed_dev);
	struct qti_hap_config *config = &chip->config;
	struct qti_hap_play_info *play = &chip->play;

	int rc = 0, val = 0;

	HAPTIC_LOG("enter\n");

	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val > 0) {
		mutex_lock(&chip->enable_lock);
		play->length_us = val * 1000;
		play->vmax_mv = config->vmax_mv;
		dev_dbg(chip->dev, "upload constant effect, length = %dus, vmax_mv=%d\n",
				play->length_us, play->vmax_mv);
		mutex_unlock(&chip->enable_lock);

	} else
		pr_err("the constant playing times is not allowed to be %d\n", val);

	return count;
}
/*end of add*/
static struct device_attribute qti_haptics_attr[] = {

	__ATTR(activate, 0664, qti_haptics_activate_show, qti_haptics_active_store),
	__ATTR(duration, 0664, qti_haptics_duration_show, qti_haptics_duration_store),

};


static void qti_haptics_enable(struct timed_output_dev *sdev, int value)
{
	struct qti_hap_chip *chip = container_of(sdev, struct qti_hap_chip, timed_dev);
	struct qti_hap_play_info *play = &chip->play;
	HAPTIC_LOG("%s enter\n");
	play->length_us = 0;
	schedule_work(&chip->vibrator_work_off);
	if (value > 0) {
		play->length_us = value * 1000;
		qti_haptics_activate(chip, 1);
	}
	HAPTIC_LOG("%s exit\n");
}

static int qti_haptics_get_time(struct timed_output_dev *sdev)
{
	struct qti_hap_chip *chip = container_of(sdev, struct qti_hap_chip, timed_dev);

	if (hrtimer_active(&chip->stop_timer)) {
		ktime_t r = hrtimer_get_remaining(&chip->stop_timer);
		return ktime_to_ms(r);
	}

	return 0;
}
/*end of add*/

static int qti_haptics_probe(struct platform_device *pdev)
{
	struct qti_hap_chip *chip;
	struct input_dev *input_dev;
	struct ff_device *ff;
	int rc = 0, effect_count_max, i;


	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	dev_err(chip->dev, "enter qti haptics probe\n");
	if (!chip)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(&pdev->dev);
	if (!input_dev)
		return -ENOMEM;

	chip->pdev = pdev;
	chip->dev = &pdev->dev;
	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "Failed to get regmap handle\n");
		return -ENXIO;
	}

	rc = qti_haptics_parse_dt(chip);
	if (rc < 0) {
		dev_err(chip->dev, "parse device-tree failed, rc=%d\n", rc);
		return rc;
	}

	mutex_init(&chip->vdd_mutex);
	hrtimer_init(&chip->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->stop_timer.function = qti_hap_stop_timer;

	INIT_WORK(&chip->vibrator_work_on, qti_haptics_work_on);
	INIT_WORK(&chip->vibrator_work_off, qti_haptics_work_off);

	input_dev->name = "qti-haptics";
	input_set_drvdata(input_dev, chip);
	chip->input_dev = input_dev;

	input_set_capability(input_dev, EV_FF, FF_CONSTANT);
	input_set_capability(input_dev, EV_FF, FF_GAIN);
	if (chip->effects_count != 0) {
		input_set_capability(input_dev, EV_FF, FF_PERIODIC);
		input_set_capability(input_dev, EV_FF, FF_CUSTOM);
	}

	if (chip->effects_count + 1 > FF_EFFECT_COUNT_MAX)
		effect_count_max = chip->effects_count + 1;
	else
		effect_count_max = FF_EFFECT_COUNT_MAX;
	rc = input_ff_create(input_dev, effect_count_max);
	if (rc < 0) {
		dev_err(chip->dev, "create FF input device failed, rc=%d\n",
				rc);
		return rc;
	}

	ff = input_dev->ff;
	ff->upload = qti_haptics_upload_effect;
	ff->playback = qti_haptics_playback;

	ff->set_gain = qti_haptics_set_gain;

	rc = input_register_device(input_dev);
	if (rc < 0) {
		dev_err(chip->dev, "register input device failed, rc=%d\n",
				rc);
		goto destroy_ff;
	}

	dev_set_drvdata(chip->dev, chip);
/*add begain*/
	chip->timed_dev.name = "qti_haptics";
	chip->timed_dev.get_time = qti_haptics_get_time;
	chip->timed_dev.enable = qti_haptics_enable;
	HAPTIC_LOG("before timed_output_dev_register\n");
	rc = timed_output_dev_register(&chip->timed_dev);
	if (rc < 0) {
		pr_err("timed_output registration failed\n");
		goto destroy_ff;
	}

	HAPTIC_LOG("before create attributes\n");
	for (i = 0; i < ARRAY_SIZE(qti_haptics_attr); i++) {
		rc = sysfs_create_file(&chip->timed_dev.dev->kobj, &qti_haptics_attr[i].attr);
		if (rc < 0) {
			pr_err("sysfs creation failed\n");
			goto sysfs_fail;
		}
	}
/*end of add*/
	return 0;
sysfs_fail:
	for (--i; i >= 0; i--)
		sysfs_remove_file(&chip->timed_dev.dev->kobj, &qti_haptics_attr[i].attr);
	timed_output_dev_unregister(&(chip->timed_dev));

destroy_ff:
	input_ff_destroy(chip->input_dev);
	return rc;
}

static int qti_haptics_remove(struct platform_device *pdev)
{
	struct qti_hap_chip *chip = dev_get_drvdata(&pdev->dev);
	int i = 0;

	input_ff_destroy(chip->input_dev);
	input_unregister_device(chip->input_dev);

	for (i = 0; i < ARRAY_SIZE(qti_haptics_attr); i++)
		sysfs_remove_file(&chip->timed_dev.dev->kobj, &qti_haptics_attr[i].attr);

	timed_output_dev_unregister(&(chip->timed_dev));
	dev_set_drvdata(chip->dev, NULL);

	return 0;
}

static void qti_haptics_shutdown(struct platform_device *pdev)
{
	struct qti_hap_chip *chip = dev_get_drvdata(&pdev->dev);

	dev_dbg(chip->dev, "Shutdown!\n");
	schedule_work(&chip->vibrator_work_off);

}

static const struct of_device_id haptics_match_table[] = {
	{ .compatible = "qcom,haptics-ldo-direct" },
	{},
};

static struct platform_driver qti_haptics_driver = {
	.driver		= {
		.name = "qcom,haptics-ldo-direct",
		.owner = THIS_MODULE,
		.of_match_table = haptics_match_table,
	},
	.probe		= qti_haptics_probe,
	.remove		= qti_haptics_remove,
	.shutdown	= qti_haptics_shutdown,
};
module_platform_driver(qti_haptics_driver);

MODULE_DESCRIPTION("QTI haptics driver");
MODULE_LICENSE("GPL v2");

