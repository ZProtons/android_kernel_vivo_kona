// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2020 The Linux Foundation. All rights reserved.
 */

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/irq.h>
#include <linux/iio/consumer.h>
#include <linux/pmic-voter.h>
#include <linux/of_batterydata.h>
#include <linux/ktime.h>
/*vivo modify*/
#include <linux/soc/qcom/fsa4480-i2c.h>
#include "vivo-smb5-lib.h"
#include "vivo-smb5-reg.h"
/*vivo end*/
#include "schgm-flash.h"
#include "step-chg-jeita.h"
#include "storm-watch.h"
#include "schgm-flash.h"

#define smblib_err(chg, fmt, ...)		\
	pr_err("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\

#define smblib_err_ratelimited(chg, fmt, ...)		\
	pr_err_ratelimited("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\

#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_err("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_err("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

#define typec_rp_med_high(chg, typec_mode)			\
	((typec_mode == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM	\
	|| typec_mode == POWER_SUPPLY_TYPEC_SOURCE_HIGH)	\
	&& (!chg->typec_legacy || chg->typec_legacy_use_rp_icl))

/* vivo add start*/
/*ex-var statement */
//static int usb_conn_fixed_temp;
extern unsigned int bsp_test_mode;
extern unsigned int ex_fg_state;
extern unsigned int power_off_charging_mode;
extern unsigned int is_atboot;
extern bool ex_fg_ffc_support;
extern bool ex_fg_support;
extern bool ex_fg_probe;
extern unsigned int ex_fg_soc;
extern unsigned int  ex_fg_i2c_error_counter;
static bool ex_fg_skip_iic_and_waitting;
extern unsigned int battery_cout_value;
extern bool enable_slave_charger;
extern bool fg_soc_really;
extern unsigned int  ex_fg_i2c_hand_step;

/*ex-function statement */
extern void charger_connect_judge(char on_or_off);
extern int fsa4480_switch_mode_event(enum fsa_function event);

enum EX_FG_STAT {
	STAT_NOT_INIT = -1,
	EX_FG_NOT_USED,
	EX_FG_USED,
};
static int ex_fg_used = STAT_NOT_INIT;

enum FG_SOC_REALLY_TYPE {
	BMS_REALLY = BIT(0),
	BQ_BMS_REALLY = BIT(1),
};

struct hvdcp_detect_voltage {
	int min_uv;
	int max_uv;
	enum hvdcp_type type;
};
static struct hvdcp_detect_voltage hvdcp_vol[] = {
	{
		.min_uv = 4150000,//4350,
		.max_uv = 7800000,
		.type	= HVDCP_TYPE_5V,
	},
	{
		.min_uv = 7800000,
		.max_uv = 10300000,
		.type	= HVDCP_TYPE_9V,
	},
	{
		.min_uv = 10300000,
		.max_uv = 14200000,
		.type	= HVDCP_TYPE_12V,
	},
};

/*local-function statement*/
static void smblib_start_apsd_rerun_work(struct smb_charger *chg);
static void smblib_usb_plugin_init(struct smb_charger *chg);
static void smblib_usb_plugin_deinit(struct smb_charger *chg);
static int get_sdp_current(struct smb_charger *chg, int *icl_ua);
static int vivo_battery_handle_ui_soc(struct smb_charger *chg, int soc);
static bool vivo_ex_fg_i2c_is_exception(struct smb_charger *chg);
static int smblib_tracking_soc(struct smb_charger *chg, int soc, bool empty_vbat);
static struct power_supply *get_cms_psy(struct smb_charger *chg);
static int smblib_drp_enable_vote_callback(struct votable *votable, void *data, int enable, const char *client);
extern char *get_bbk_board_version(void);
//VIVO_PROJECT_BASE_NAME
extern bool is_project(const char *name);
/*vivo add end*/

static void update_sw_icl_max(struct smb_charger *chg, int pst);
static int smblib_get_prop_typec_mode(struct smb_charger *chg);

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int value;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &value);
	if (rc >= 0)
		*val = (u8)value;

	return rc;
}

int smblib_batch_read(struct smb_charger *chg, u16 addr, u8 *val,
			int count)
{
	return regmap_bulk_read(chg->regmap, addr, val, count);
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	return regmap_write(chg->regmap, addr, val);
}

int smblib_batch_write(struct smb_charger *chg, u16 addr, u8 *val,
			int count)
{
	return regmap_bulk_write(chg->regmap, addr, val, count);
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	return regmap_update_bits(chg->regmap, addr, mask, val);
}

int smblib_get_iio_channel(struct smb_charger *chg, const char *propname,
					struct iio_channel **chan)
{
	int rc = 0;

	rc = of_property_match_string(chg->dev->of_node,
					"io-channel-names", propname);
	if (rc < 0)
		return 0;

	*chan = iio_channel_get(chg->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		if (rc != -EPROBE_DEFER)
			smblib_err(chg, "%s channel unavailable, %d\n",
							propname, rc);
		*chan = NULL;
	}

	return rc;
}

#define DIV_FACTOR_MICRO_V_I	1
#define DIV_FACTOR_MILI_V_I	1000
#define DIV_FACTOR_DECIDEGC	100
int smblib_read_iio_channel(struct smb_charger *chg, struct iio_channel *chan,
							int div, int *data)
{
	int rc = 0;
	*data = -ENODATA;

	if (chan) {
		rc = iio_read_channel_processed(chan, data);
		if (rc < 0) {
			smblib_err(chg, "Error in reading IIO channel data, rc=%d\n",
					rc);
			return rc;
		}

		if (div != 0)
			*data /= div;
	}

	return rc;
}

int smblib_get_jeita_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, cc_minus_ua;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_7_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}

	if (stat & BAT_TEMP_STATUS_HOT_SOFT_BIT) {
		rc = smblib_get_charge_param(chg, &chg->param.jeita_cc_comp_hot,
					&cc_minus_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n",
					rc);
			return rc;
		}
	} else if (stat & BAT_TEMP_STATUS_COLD_SOFT_BIT) {
		rc = smblib_get_charge_param(chg,
					&chg->param.jeita_cc_comp_cold,
					&cc_minus_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n",
					rc);
			return rc;
		}
	} else {
		cc_minus_ua = 0;
	}

	*cc_delta_ua = -cc_minus_ua;

	return 0;
}

int smblib_icl_override(struct smb_charger *chg, enum icl_override_mode  mode)
{
	int rc;
	u8 usb51_mode, icl_override, apsd_override;

	switch (mode) {
	case SW_OVERRIDE_USB51_MODE:
		usb51_mode = 0;
		icl_override = ICL_OVERRIDE_BIT;
		apsd_override = 0;
		break;
	case SW_OVERRIDE_HC_MODE:
		usb51_mode = USBIN_MODE_CHG_BIT;
		icl_override = 0;
		apsd_override = ICL_OVERRIDE_AFTER_APSD_BIT;
		break;
	/*vivo add*/
	case UCLC_OVERRIDE_HC_MODE:
		usb51_mode = USBIN_MODE_CHG_BIT;
		icl_override = ICL_OVERRIDE_BIT;
		apsd_override = 0;
		break;
	/*vivo end*/
	case HW_AUTO_MODE:
	default:
		usb51_mode = USBIN_MODE_CHG_BIT;
		icl_override = 0;
		apsd_override = 0;
		break;
	}

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
				USBIN_MODE_CHG_BIT, usb51_mode);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set USBIN_ICL_OPTIONS rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, CMD_ICL_OVERRIDE_REG,
				ICL_OVERRIDE_BIT, icl_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't override ICL rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, USBIN_LOAD_CFG_REG,
				ICL_OVERRIDE_AFTER_APSD_BIT, apsd_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't override ICL_AFTER_APSD rc=%d\n", rc);
		return rc;
	}

	return rc;
}

/*
 * This function does smb_en pin access, which is lock protected.
 * It should be called with smb_lock held.
 */
static int smblib_select_sec_charger_locked(struct smb_charger *chg,
					int sec_chg)
{
	int rc = 0;

	switch (sec_chg) {
	case POWER_SUPPLY_CHARGER_SEC_CP:
		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, true, 0);

		/* select Charge Pump instead of slave charger */
		rc = smblib_masked_write(chg, MISC_SMB_CFG_REG,
					SMB_EN_SEL_BIT, SMB_EN_SEL_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't select SMB charger rc=%d\n",
				rc);
			return rc;
		}
		/* Enable Charge Pump, under HW control */
		rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
					EN_CP_CMD_BIT, EN_CP_CMD_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't enable SMB charger rc=%d\n",
						rc);
			return rc;
		}
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, false, 0);
		break;
	case POWER_SUPPLY_CHARGER_SEC_PL:
		/* select slave charger instead of Charge Pump */
		rc = smblib_masked_write(chg, MISC_SMB_CFG_REG,
					SMB_EN_SEL_BIT, 0);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't select SMB charger rc=%d\n",
				rc);
			return rc;
		}
		/* Enable slave charger, under HW control */
		rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
					EN_STAT_CMD_BIT, EN_STAT_CMD_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't enable SMB charger rc=%d\n",
						rc);
			return rc;
		}
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, false, 0);

		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, false, 0);

		break;
	case POWER_SUPPLY_CHARGER_SEC_NONE:
	default:
		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, true, 0);

		/* SW override, disabling secondary charger(s) */
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, true, 0);
		break;
	}

	return rc;
}

static int smblib_select_sec_charger(struct smb_charger *chg, int sec_chg,
					int reason, bool toggle)
{
	int rc;

	mutex_lock(&chg->smb_lock);

	if (toggle && sec_chg == POWER_SUPPLY_CHARGER_SEC_CP) {
		rc = smblib_select_sec_charger_locked(chg,
					POWER_SUPPLY_CHARGER_SEC_NONE);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't disable secondary charger rc=%d\n",
				rc);
			goto unlock_out;
		}

		/*
		 * A minimum of 20us delay is expected before switching on STAT
		 * pin.
		 */
		usleep_range(20, 30);
	}

	rc = smblib_select_sec_charger_locked(chg, sec_chg);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't switch secondary charger rc=%d\n",
			rc);
		goto unlock_out;
	}

	chg->sec_chg_selected = sec_chg;
	chg->cp_reason = reason;

unlock_out:
	mutex_unlock(&chg->smb_lock);

	return rc;
}

static void smblib_notify_extcon_props(struct smb_charger *chg, int id)
{
	union extcon_property_value val;
	union power_supply_propval prop_val;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_TYPEC) {
		smblib_get_prop_typec_cc_orientation(chg, &prop_val);
		val.intval = ((prop_val.intval == 2) ? 1 : 0);
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_TYPEC_POLARITY, val);
		val.intval = true;
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_SS, val);
	} else if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		val.intval = false;
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_SS, val);
	}
}

static void smblib_notify_device_mode(struct smb_charger *chg, bool enable)
{
	if (enable)
		smblib_notify_extcon_props(chg, EXTCON_USB);

	extcon_set_state_sync(chg->extcon, EXTCON_USB, enable);
}

static void smblib_notify_usb_host(struct smb_charger *chg, bool enable)
{
	int rc = 0;

	if (enable) {
		smblib_dbg(chg, PR_OTG, "enabling VBUS in OTG mode\n");
		rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG,
					OTG_EN_BIT, OTG_EN_BIT);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't enable VBUS in OTG mode rc=%d\n", rc);
			return;
		}

		smblib_notify_extcon_props(chg, EXTCON_USB_HOST);
	} else {
		smblib_dbg(chg, PR_OTG, "disabling VBUS in OTG mode\n");
		rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG,
					OTG_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't disable VBUS in OTG mode rc=%d\n",
				rc);
			return;
		}
	}

	extcon_set_state_sync(chg->extcon, EXTCON_USB_HOST, enable);
}

/********************
 * REGISTER GETTERS *
 ********************/

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read(chg, param->reg, &val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, *val_u, val_raw);

	return rc;
}

int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}


static const s16 therm_lookup_table[] = {
	/* Index -30C~85C, ADC raw code */
	0x6C92, 0x6C43, 0x6BF0, 0x6B98, 0x6B3A, 0x6AD8, 0x6A70, 0x6A03,
	0x6990, 0x6916, 0x6897, 0x6811, 0x6785, 0x66F2, 0x6658, 0x65B7,
	0x650F, 0x6460, 0x63AA, 0x62EC, 0x6226, 0x6159, 0x6084, 0x5FA8,
	0x5EC3, 0x5DD8, 0x5CE4, 0x5BE9, 0x5AE7, 0x59DD, 0x58CD, 0x57B5,
	0x5696, 0x5571, 0x5446, 0x5314, 0x51DD, 0x50A0, 0x4F5E, 0x4E17,
	0x4CCC, 0x4B7D, 0x4A2A, 0x48D4, 0x477C, 0x4621, 0x44C4, 0x4365,
	0x4206, 0x40A6, 0x3F45, 0x3DE6, 0x3C86, 0x3B28, 0x39CC, 0x3872,
	0x3719, 0x35C4, 0x3471, 0x3322, 0x31D7, 0x308F, 0x2F4C, 0x2E0D,
	0x2CD3, 0x2B9E, 0x2A6E, 0x2943, 0x281D, 0x26FE, 0x25E3, 0x24CF,
	0x23C0, 0x22B8, 0x21B5, 0x20B8, 0x1FC2, 0x1ED1, 0x1DE6, 0x1D01,
	0x1C22, 0x1B49, 0x1A75, 0x19A8, 0x18E0, 0x181D, 0x1761, 0x16A9,
	0x15F7, 0x154A, 0x14A2, 0x13FF, 0x1361, 0x12C8, 0x1234, 0x11A4,
	0x1119, 0x1091, 0x100F, 0x0F90, 0x0F15, 0x0E9E, 0x0E2B, 0x0DBC,
	0x0D50, 0x0CE8, 0x0C83, 0x0C21, 0x0BC3, 0x0B67, 0x0B0F, 0x0AB9,
	0x0A66, 0x0A16, 0x09C9, 0x097E,
};

int smblib_get_thermal_threshold(struct smb_charger *chg, u16 addr, int *val)
{
	u8 buff[2];
	s16 temp;
	int rc = 0;
	int i, lower, upper;

	rc = smblib_batch_read(chg, addr, buff, 2);
	if (rc < 0) {
		pr_err("failed to write to 0x%04X, rc=%d\n", addr, rc);
		return rc;
	}

	temp = buff[1] | buff[0] << 8;

	lower = 0;
	upper = ARRAY_SIZE(therm_lookup_table) - 1;
	while (lower <= upper) {
		i = (upper + lower) / 2;
		if (therm_lookup_table[i] < temp)
			upper = i - 1;
		else if (therm_lookup_table[i] > temp)
			lower = i + 1;
		else
			break;
	}

	/* index 0 corresonds to -30C */
	*val = (i - 30) * 10;

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

enum {
	UNKNOWN,
	SDP,
	CDP,
	DCP,
	OCP,
	FLOAT,
	HVDCP2,
	HVDCP3,
	MAX_TYPES
};

static const struct apsd_result smblib_apsd_results[] = {
	[UNKNOWN] = {
		.name	= "UNKNOWN",
		.bit	= 0,
		.pst	= POWER_SUPPLY_TYPE_UNKNOWN
	},
	[SDP] = {
		.name	= "SDP",
		.bit	= SDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB
	},
	[CDP] = {
		.name	= "CDP",
		.bit	= CDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_CDP
	},
	[DCP] = {
		.name	= "DCP",
		.bit	= DCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[OCP] = {
		.name	= "OCP",
		.bit	= OCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[FLOAT] = {
		.name	= "FLOAT",
		.bit	= FLOAT_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_FLOAT
	},
	[HVDCP2] = {
		.name	= "HVDCP2",
		.bit	= DCP_CHARGER_BIT | QC_2P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP
	},
	[HVDCP3] = {
		.name	= "HVDCP3",
		.bit	= DCP_CHARGER_BIT | QC_3P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP_3,
	},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 apsd_stat, stat;
	const struct apsd_result *result = &smblib_apsd_results[UNKNOWN];

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return result;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return result;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return result;
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			result = &smblib_apsd_results[i];
	}

	if (apsd_stat & QC_CHARGER_BIT) {
		/* since its a qc_charger, either return HVDCP3 or HVDCP2 */
		if (result != &smblib_apsd_results[HVDCP3])
			result = &smblib_apsd_results[HVDCP2];
	}
	/*vivo add*/
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		chg->real_charger_type = result->pst;
	}

	if (result->pst == POWER_SUPPLY_TYPE_USB) {
		chg->bigdata_chg_type = POWER_SUPPLY_TYPE_USB;
	} else if (result->pst == POWER_SUPPLY_TYPE_USB_CDP) {
		chg->bigdata_chg_type = POWER_SUPPLY_TYPE_USB_CDP;
	} else if (result->pst == POWER_SUPPLY_TYPE_USB_FLOAT) {
		chg->bigdata_chg_type = POWER_SUPPLY_TYPE_USB_FLOATED;
	} else if (result->pst == POWER_SUPPLY_TYPE_USB_DCP &&
				!strncmp(result->name, "OCP", 3)) {
		chg->bigdata_chg_type = POWER_SUPPLY_TYPE_USB_OCP;
	} else if (result->pst == POWER_SUPPLY_TYPE_USB_HVDCP ||
				result->pst == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
		/* hvdcp-2.0 or hvdcp-3.0 */
		chg->bigdata_chg_type = POWER_SUPPLY_TYPE_USB_HVDCP;
	} else if (result->pst == POWER_SUPPLY_TYPE_USB_DCP) {
		chg->bigdata_chg_type = POWER_SUPPLY_TYPE_USB_DCP;
	} else {
		chg->bigdata_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}
	smblib_dbg(chg, PR_REGISTER, "detect charger type = %s\n", result->name);
	/*vivo add end*/
	return result;
}

#define INPUT_NOT_PRESENT	0
#define INPUT_PRESENT_USB	BIT(1)
#define INPUT_PRESENT_DC	BIT(2)
static int smblib_is_input_present(struct smb_charger *chg,
				   int *present)
{
	int rc;
	union power_supply_propval pval = {0, };

	*present = INPUT_NOT_PRESENT;

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		pr_err("Couldn't get usb presence status rc=%d\n", rc);
		return rc;
	}
	*present |= pval.intval ? INPUT_PRESENT_USB : INPUT_NOT_PRESENT;

	rc = smblib_get_prop_dc_present(chg, &pval);
	if (rc < 0) {
		pr_err("Couldn't get dc presence status rc=%d\n", rc);
		return rc;
	}
	*present |= pval.intval ? INPUT_PRESENT_DC : INPUT_NOT_PRESENT;

	return 0;
}

#define AICL_RANGE2_MIN_MV		5600
#define AICL_RANGE2_STEP_DELTA_MV	200
#define AICL_RANGE2_OFFSET		16
int smblib_get_aicl_cont_threshold(struct smb_chg_param *param, u8 val_raw)
{
	int base = param->min_u;
	u8 reg = val_raw;
	int step = param->step_u;


	if (val_raw >= AICL_RANGE2_OFFSET) {
		reg = val_raw - AICL_RANGE2_OFFSET;
		base = AICL_RANGE2_MIN_MV;
		step = AICL_RANGE2_STEP_DELTA_MV;
	}

	return base + (reg * step);
}

/********************
 * REGISTER SETTERS *
 ********************/
static const struct buck_boost_freq chg_freq_list[] = {
	[0] = {
		.freq_khz	= 2400,
		.val		= 7,
	},
	[1] = {
		.freq_khz	= 2100,
		.val		= 8,
	},
	[2] = {
		.freq_khz	= 1600,
		.val		= 11,
	},
	[3] = {
		.freq_khz	= 1200,
		.val		= 15,
	},
};

int smblib_set_chg_freq(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	u8 i;

	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	/* Charger FSW is the configured freqency / 2 */
	val_u *= 2;
	for (i = 0; i < ARRAY_SIZE(chg_freq_list); i++) {
		if (chg_freq_list[i].freq_khz == val_u)
			break;
	}
	if (i == ARRAY_SIZE(chg_freq_list)) {
		pr_err("Invalid frequency %d Hz\n", val_u / 2);
		return -EINVAL;
	}

	*val_raw = chg_freq_list[i].val;

	return 0;
}

int smblib_set_opt_switcher_freq(struct smb_charger *chg, int fsw_khz)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher, fsw_khz);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_buck rc=%d\n", rc);

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fsw_khz;
		/*
		 * Some parallel charging implementations may not have
		 * PROP_BUCK_FREQ property - they could be running
		 * with a fixed frequency
		 */
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_BUCK_FREQ, &pval);
	}

	return rc;
}

int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u)
			smblib_dbg(chg, PR_MISC,
				"%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);

		if (val_u > param->max_u)
			val_u = param->max_u;
		if (val_u < param->min_u)
			val_u = param->min_u;

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, val_u, val_raw);

	return rc;
}

int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;
	if (suspend)
		vote(chg->icl_irq_disable_votable, USB_SUSPEND_VOTER,
				true, 0);

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	if (!suspend)
		vote(chg->icl_irq_disable_votable, USB_SUSPEND_VOTER,
				false, 0);

	return rc;
}

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

static int smblib_usb_pd_adapter_allowance_override(struct smb_charger *chg,
					u8 allowed_voltage)
{
	int rc = 0;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		return 0;

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_OVERRIDE_REG,
						allowed_voltage);
	if (rc < 0)
		smblib_err(chg, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_OVERRIDE_REG rc=%d\n",
			allowed_voltage, rc);

	smblib_dbg(chg, PR_MISC, "set USBIN_ALLOW_OVERRIDE: %d\n",
			allowed_voltage);
	return rc;
}

#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_fsw(struct smb_charger *chg, int voltage)
{
	int rc = 0;

	if (voltage == MICRO_5V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_5V);
	else if (voltage > MICRO_5V && voltage < MICRO_9V)
		rc = smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_6V_8V);
	else if (voltage >= MICRO_9V && voltage < MICRO_12V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_9V);
	else if (voltage == MICRO_12V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_12V);
	else {
		smblib_err(chg, "Couldn't set Fsw: invalid voltage %d\n",
				voltage);
		return -EINVAL;
	}

	return rc;
}

#define CONT_AICL_HEADROOM_MV		1000
#define AICL_THRESHOLD_MV_IN_CC		5000
static int smblib_set_usb_pd_allowed_voltage(struct smb_charger *chg,
					int min_allowed_uv, int max_allowed_uv)
{
	int rc, aicl_threshold;
	u8 vbus_allowance;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		return 0;

	if (chg->pd_active == POWER_SUPPLY_PD_PPS_ACTIVE) {
		vbus_allowance = CONTINUOUS;
	} else if (min_allowed_uv == MICRO_5V && max_allowed_uv == MICRO_5V) {
		vbus_allowance = FORCE_5V;
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		vbus_allowance = FORCE_9V;
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		vbus_allowance = FORCE_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		vbus_allowance = CONTINUOUS;
	} else {
		smblib_err(chg, "invalid allowed voltage [%d, %d]\n",
				min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}

	rc = smblib_usb_pd_adapter_allowance_override(chg, vbus_allowance);
	if (rc < 0) {
		smblib_err(chg, "set CONTINUOUS allowance failed, rc=%d\n",
				rc);
		return rc;
	}

	if (vbus_allowance != CONTINUOUS)
		return 0;

	aicl_threshold = min_allowed_uv / 1000 - CONT_AICL_HEADROOM_MV;
	if (chg->adapter_cc_mode)
		aicl_threshold = min(aicl_threshold, AICL_THRESHOLD_MV_IN_CC);

	rc = smblib_set_charge_param(chg, &chg->param.aicl_cont_threshold,
							aicl_threshold);
	if (rc < 0) {
		smblib_err(chg, "set CONT_AICL_THRESHOLD failed, rc=%d\n",
							rc);
		return rc;
	}

	return rc;
}

int smblib_set_aicl_cont_threshold(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	int base = param->min_u;
	int offset = 0;
	int step = param->step_u;

	if (val_u > param->max_u)
		val_u = param->max_u;
	if (val_u < param->min_u)
		val_u = param->min_u;

	if (val_u >= AICL_RANGE2_MIN_MV) {
		base = AICL_RANGE2_MIN_MV;
		step = AICL_RANGE2_STEP_DELTA_MV;
		offset = AICL_RANGE2_OFFSET;
	};

	*val_raw = ((val_u - base) / step) + offset;

	return 0;
}

/********************
 * HELPER FUNCTIONS *
 ********************/
static bool is_cp_available(struct smb_charger *chg)
{
	if (!chg->cp_psy)
		chg->cp_psy = power_supply_get_by_name("charge_pump_master");

	return !!chg->cp_psy;
}

static bool is_cp_topo_vbatt(struct smb_charger *chg)
{
	int rc;
	bool is_vbatt;
	union power_supply_propval pval;

	if (!is_cp_available(chg))
		return false;

	rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_PARALLEL_OUTPUT_MODE, &pval);
	if (rc < 0)
		return false;

	is_vbatt = (pval.intval == POWER_SUPPLY_PL_OUTPUT_VBAT);

	smblib_dbg(chg, PR_WLS, "%s\n", is_vbatt ? "true" : "false");

	return is_vbatt;
}

#define CP_TO_MAIN_ICL_OFFSET_PC		10
int smblib_get_qc3_main_icl_offset(struct smb_charger *chg, int *offset_ua)
{
	union power_supply_propval pval = {0, };
	int rc;

	/*
	 * Apply ILIM offset to main charger's FCC if all of the following
	 * conditions are met:
	 * - HVDCP3 adapter with CP as parallel charger
	 * - Output connection topology is VBAT
	 */
	if (!is_cp_topo_vbatt(chg) || chg->hvdcp3_standalone_config
		|| ((chg->real_charger_type != POWER_SUPPLY_TYPE_USB_HVDCP_3)
		&& chg->real_charger_type != POWER_SUPPLY_TYPE_USB_HVDCP_3P5))
		return -EINVAL;

	rc = power_supply_get_property(chg->cp_psy, POWER_SUPPLY_PROP_CP_ENABLE,
					&pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get CP ENABLE rc=%d\n", rc);
		return rc;
	}

	if (!pval.intval)
		return -EINVAL;

	rc = power_supply_get_property(chg->cp_psy, POWER_SUPPLY_PROP_CP_ILIM,
					&pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get CP ILIM rc=%d\n", rc);
		return rc;
	}

	*offset_ua = (pval.intval * CP_TO_MAIN_ICL_OFFSET_PC * 2) / 100;

	return 0;
}

int smblib_get_prop_from_bms(struct smb_charger *chg,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy, psp, val);

	return rc;
}

void smblib_apsd_enable(struct smb_charger *chg, bool enable)
{
	int rc;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				BC1P2_SRC_DETECT_BIT,
				enable ? BC1P2_SRC_DETECT_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "failed to write USBIN_OPTIONS_1_CFG rc=%d\n",
				rc);
}

void smblib_hvdcp_detect_enable(struct smb_charger *chg, bool enable)
{
	int rc;
	u8 mask;

	mask = HVDCP_AUTH_ALG_EN_CFG_BIT | HVDCP_EN_BIT;
	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG, mask,
						enable ? mask : 0);
	if (rc < 0)
		smblib_err(chg, "failed to write USBIN_OPTIONS_1_CFG rc=%d\n",
				rc);
}

static void smblib_hvdcp_detect_try_enable(struct smb_charger *chg, bool enable)
{
	if (chg->hvdcp_disable || chg->pd_not_supported)
		return;
	smblib_hvdcp_detect_enable(chg, enable);
}

void smblib_hvdcp_hw_inov_enable(struct smb_charger *chg, bool enable)
{
	int rc;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT,
				enable ? HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "failed to write USBIN_OPTIONS_1_CFG rc=%d\n",
				rc);
}

void smblib_hvdcp_exit_config(struct smb_charger *chg)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0)
		return;

	if (stat & (QC_3P0_BIT | QC_2P0_BIT)) {
		/* force HVDCP to 5V */
		smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT, 0);
		smblib_write(chg, CMD_HVDCP_2_REG, FORCE_5V_BIT);

		/* rerun APSD */
		smblib_masked_write(chg, CMD_APSD_REG, APSD_RERUN_BIT,
				APSD_RERUN_BIT);
	}
}

int smblib_request_dpdm(struct smb_charger *chg, bool enable)
{
	int rc = 0;

	if (chg->pr_swap_in_progress)
		return 0;

	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
				"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			rc = PTR_ERR(chg->dpdm_reg);
			smblib_err(chg, "Couldn't get dpdm regulator rc=%d\n",
					rc);
			chg->dpdm_reg = NULL;
			return rc;
		}
	}

	mutex_lock(&chg->dpdm_lock);
	if (enable) {
		if (chg->dpdm_reg && !chg->dpdm_enabled) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't enable dpdm regulator rc=%d\n",
					rc);
			else
				chg->dpdm_enabled = true;
		}
	} else {
		if (chg->dpdm_reg && chg->dpdm_enabled) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't disable dpdm regulator rc=%d\n",
					rc);
			else
				chg->dpdm_enabled = false;
		}
	}
	mutex_unlock(&chg->dpdm_lock);

	return rc;
}

void smblib_rerun_apsd(struct smb_charger *chg)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "re-running APSD\n");

	rc = smblib_masked_write(chg, CMD_APSD_REG,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't re-run APSD rc=%d\n", rc);
}

static const struct apsd_result *smblib_update_usb_type(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active) {
		chg->real_charger_type = POWER_SUPPLY_TYPE_USB_PD;
	} else if (chg->qc3p5_detected) {
		chg->real_charger_type = POWER_SUPPLY_TYPE_USB_HVDCP_3P5;
	} else {
		/*
		 * Update real charger type only if its not FLOAT
		 * detected as as SDP
		 */
		if (!(apsd_result->pst == POWER_SUPPLY_TYPE_USB_FLOAT &&
			chg->real_charger_type == POWER_SUPPLY_TYPE_USB))
			chg->real_charger_type = apsd_result->pst;
	}
	strcpy(chg->usb_type_name,apsd_result->name);	/* vivo add */

	smblib_dbg(chg, PR_MISC, "APSD=%s PD=%d QC3P5=%d\n",
			apsd_result->name, chg->pd_active, chg->qc3p5_detected);
	return apsd_result;
}

static int smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, nb);

	if (!strcmp(psy->desc->name, "bms")) {
		if (!chg->bms_psy)
			chg->bms_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&chg->bms_update_work);
	}

	/*VIVO add*/
	if (!is_atboot && !strcmp(psy->desc->name, "bq_bms")) {
		if (ex_fg_support && !ex_fg_skip_iic_and_waitting && (!chg->bms_psy || chg->bms_psy != psy)) {
			chg->bms_psy = psy;
			smblib_err(chg, "change bms_psy=%s\n", chg->bms_psy->desc->name);
		}
	}

	if (chg->jeita_configured == JEITA_CFG_NONE)
		schedule_work(&chg->jeita_update_work);

	if (chg->sec_pl_present && !chg->pl.psy
		&& !strcmp(psy->desc->name, "parallel")) {
		chg->pl.psy = psy;
		schedule_work(&chg->pl_update_work);
	}

	if (!strcmp(psy->desc->name, "charge_pump_master")) {
		pm_stay_awake(chg->dev);
		schedule_work(&chg->cp_status_change_work);
	}

	return NOTIFY_OK;
}

static int smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->nb.notifier_call = smblib_notifier_call;
	rc = power_supply_reg_notifier(&chg->nb);
	if (rc < 0) {
		smblib_err(chg, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	*val_raw = val_u << 1;

	return 0;
}

int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw)
{
	int val_u  = val_raw * param->step_u + param->min_u;

	if (val_u > param->max_u)
		val_u -= param->max_u * 2;

	return val_u;
}

int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u - param->max_u)
		return -EINVAL;

	val_u += param->max_u * 2 - param->min_u;
	val_u %= param->max_u * 2;
	*val_raw = val_u / param->step_u;

	return 0;
}

static void smblib_uusb_removal(struct smb_charger *chg)
{
	struct smb5 *chip = container_of(chg, struct smb5, chg);/*vivo add*/
	int rc;
	struct smb_irq_data *data;
	struct storm_watch *wdata;
	int sec_charger;

	sec_charger = chg->sec_pl_present ? POWER_SUPPLY_CHARGER_SEC_PL :
				POWER_SUPPLY_CHARGER_SEC_NONE;
	smblib_select_sec_charger(chg, sec_charger, POWER_SUPPLY_CP_NONE,
					false);

	cancel_delayed_work_sync(&chg->pl_enable_work);

	if (chg->wa_flags & BOOST_BACK_WA) {
		data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
		if (data) {
			wdata = &data->storm_data;
			update_storm_count(wdata, WEAK_CHG_STORM_COUNT);
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					false, 0);
		}
	}
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);

	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false,
			0/*is_flash_active(chg) ? SDP_CURRENT_UA : SDP_100_MA*/);	//vivo change
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);
	vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
	vote(chg->usb_icl_votable, THERMAL_THROTTLE_VOTER, false, 0);
	vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
			true, 0);
	vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER, true, 0);
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);

	/* Remove SW thermal regulation WA votes */
	vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER, false, 0);
	if (chg->cp_disable_votable)
		vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
								false, 0);

	/* reset USBOV votes and cancel work */
	cancel_delayed_work_sync(&chg->usbov_dbc_work);
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	chg->dbc_usbov = false;

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usbin_forced_max_uv = 0;
	chg->usb_icl_delta_ua = 0;
	chg->pulse_cnt = 0;
	chg->uusb_apsd_rerun_done = false;
	chg->chg_param.forced_main_fcc = 0;

	del_timer_sync(&chg->apsd_timer);
	chg->apsd_ext_timeout = false;

	/* write back the default FLOAT charger configuration */
	rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				(u8)FLOAT_OPTIONS_MASK, chg->float_cfg);
	if (rc < 0)
		smblib_err(chg, "Couldn't write float charger options rc=%d\n",
			rc);

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote for USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for DCP_VOTER */
	rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote DCP from USB ICL rc=%d\n", rc);

	/*
	 * if non-compliant charger caused UV, restore original max pulses
	 * and turn SUSPEND_ON_COLLAPSE_USBIN_BIT back on.
	 */
	if (chg->qc2_unsupported_voltage) {
		rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
				HVDCP_PULSE_COUNT_MAX_QC2_MASK,
				chg->qc2_max_pulses);
		if (rc < 0)
			smblib_err(chg, "Couldn't restore max pulses rc=%d\n",
					rc);

		if (!chip->dt.disable_suspend_on_collapse) {/*vivo add condition*/
			rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
					SUSPEND_ON_COLLAPSE_USBIN_BIT,
					SUSPEND_ON_COLLAPSE_USBIN_BIT);
			if (rc < 0)
				smblib_err(chg, "Couldn't turn on SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
						rc);
		}

		chg->qc2_unsupported_voltage = QC2_COMPLIANT;
	}

	chg->qc3p5_detected = false;
	smblib_update_usb_type(chg);
}

void smblib_suspend_on_debug_battery(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval val;
	/*vivo add start*/
	if (!chg->bms_psy)
		return;
	/*vivo add end*/
	rc = smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get debug battery prop rc=%d\n", rc);
		return;
	}
	if (chg->suspend_input_on_debug_batt) {
		vote(chg->usb_icl_votable, DEBUG_BOARD_VOTER, val.intval, 0);
		vote(chg->dc_suspend_votable, DEBUG_BOARD_VOTER, val.intval, 0);
		if (val.intval)
			pr_info("Input suspended: Fake battery\n");
	} else {
		vote(chg->chg_disable_votable, DEBUG_BOARD_VOTER,
					val.intval, 0);
	}
}

int smblib_rerun_apsd_if_required(struct smb_charger *chg)
{
	union power_supply_propval val;
	int rc;

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
		return rc;
	}

	if (!val.intval)
		return 0;

	/* vivo add
		Before APSD_rerun, disable Typec-Switcher,
		make sure D+/D- voltage = 0, trigger HVDCP Adapter to reset to DCP
		��HVDCP adapter will connect D+/D- again��
	*/
	if (chg->usb_det_pin_mode)
		fsa4480_switch_mode_event(FSA_USBC_SWITCH_DISABLE);
	if (chg->audio_attached == false) {
	/* disable and re-enable DP/DM */
	rc = smblib_request_dpdm(chg, false);
	if (rc < 0)
		smblib_err(chg, "Couldn't to disable DPDM rc=%d\n", rc);
	/*vivo end*/

	rc = smblib_request_dpdm(chg, true);
	if (rc < 0)
		smblib_err(chg, "Couldn't to enable DPDM rc=%d\n", rc);
	}

	/* vivo add */
	if (chg->usb_det_pin_mode) {
		msleep(100);
		fsa4480_switch_mode_event(FSA_USBC_SWITCH_ENABLE);
	}
	/*vivo end*/

	chg->uusb_apsd_rerun_done = true;
	smblib_rerun_apsd(chg);

	return 0;
}

static int smblib_get_pulse_cnt(struct smb_charger *chg, int *count)
{
	*count = chg->pulse_cnt;
	return 0;
}

#define USBIN_25MA	25000
#define USBIN_100MA	100000
#define USBIN_150MA	150000
#define USBIN_500MA	500000
#define USBIN_900MA	900000
#define USBIN_1000MA	1000000
static int set_sdp_current(struct smb_charger *chg, int icl_ua)
{
	int rc;
	u8 icl_options;
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	smblib_dbg(chg, PR_MISC, "current = %d uA\n", icl_ua);

	/* power source is SDP */
	switch (icl_ua) {
	case USBIN_100MA:
		/* USB 2.0 100mA */
		icl_options = 0;
		break;
	case USBIN_150MA:
		/* USB 3.0 150mA */
		icl_options = CFG_USB3P0_SEL_BIT;
		break;
	case USBIN_500MA:
		/* USB 2.0 500mA */
		icl_options = USB51_MODE_BIT;
		break;
	case USBIN_900MA:
		/* USB 3.0 900mA */
		icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
	default:
		return -EINVAL;
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB &&
		apsd_result->pst == POWER_SUPPLY_TYPE_USB_FLOAT) {
		/*
		 * change the float charger configuration to SDP, if this
		 * is the case of SDP being detected as FLOAT
		 */
		rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
			FORCE_FLOAT_SDP_CFG_BIT, FORCE_FLOAT_SDP_CFG_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set float ICL options rc=%d\n",
						rc);
			return rc;
		}
	}

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
			CFG_USB3P0_SEL_BIT | USB51_MODE_BIT, icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL options rc=%d\n", rc);
		return rc;
	}

	rc = smblib_icl_override(chg, SW_OVERRIDE_USB51_MODE);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int smblib_set_icl_current(struct smb_charger *chg, int icl_ua)
{
	int rc = 0;
	enum icl_override_mode icl_override = HW_AUTO_MODE;
	/* suspend if 25mA or less is requested */
	bool suspend = (icl_ua <= USBIN_25MA);
	
	/* vivo add */
	//fuelsummary_collect_value(ID_MIC_INPUT, (icl_ua / 1000));
	smblib_dbg(chg, PR_MISC, "suspend=%d,icl_ua=%d, vote_client=%s,typec_mode=%d\n", 
				suspend, icl_ua, get_effective_client_locked(chg->usb_icl_votable), chg->typec_mode);
	/* vivo end */

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		schgm_flash_torch_priority(chg, suspend ? TORCH_BOOST_MODE :
					TORCH_BUCK_MODE);

	#if 0
	/* Do not configure ICL from SW for DAM cables */
	if (smblib_get_prop_typec_mode(chg) ==
			    POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY)
		return 0;
	#endif

	if (suspend)
		return smblib_set_usb_suspend(chg, true);

	if (icl_ua == INT_MAX)
		goto set_mode;

	/* configure current */
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB
		&& (chg->typec_legacy
		|| chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
		|| chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)) {
		rc = set_sdp_current(chg, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set SDP ICL rc=%d\n", rc);
			goto out;
		}
		/*vivo add*/
		icl_override = SW_OVERRIDE_USB51_MODE;
		/*vivo end*/
	} else {
		/*
		 * Try USB 2.0/3,0 option first on USB path when maximum input
		 * current limit is 500mA or below for better accuracy; in case
		 * of error, proceed to use USB high-current mode.
		 */
		if (icl_ua <= USBIN_500MA) {
			rc = set_sdp_current(chg, icl_ua);
			if (rc >= 0)
				goto unsuspend;
		}

		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			goto out;
		}
		icl_override = SW_OVERRIDE_HC_MODE;
		/*vivo add*/
		if (chg->cable_with_cc_float_icl_override) {
			icl_override = UCLC_OVERRIDE_HC_MODE;
		}
		/*vivo end*/
	}

set_mode:
	rc = smblib_icl_override(chg, icl_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		goto out;
	}

unsuspend:
	/* unsuspend after configuring current and override */
	rc = smblib_set_usb_suspend(chg, false);
	if (rc < 0) {
		smblib_err(chg, "Couldn't resume input rc=%d\n", rc);
		goto out;
	}

	/* Re-run AICL */
	if (icl_override != SW_OVERRIDE_HC_MODE)
		rc = smblib_run_aicl(chg, RERUN_AICL);
out:
	return rc;
}

int smblib_get_icl_current(struct smb_charger *chg, int *icl_ua)
{
	int rc;

	rc = smblib_get_charge_param(chg, &chg->param.icl_max_stat, icl_ua);
	if (rc < 0)
		smblib_err(chg, "Couldn't get HC ICL rc=%d\n", rc);

	return rc;
}

int smblib_toggle_smb_en(struct smb_charger *chg, int toggle)
{
	int rc = 0;

	if (!toggle)
		return rc;

	rc = smblib_select_sec_charger(chg, chg->sec_chg_selected,
				chg->cp_reason, true);

	return rc;
}

int smblib_get_irq_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 reg;

	if (chg->wa_flags & SKIP_MISC_PBS_IRQ_WA) {
		val->intval = 0;
		return 0;
	}

	mutex_lock(&chg->irq_status_lock);
	/* Report and clear cached status */
	val->intval = chg->irq_status;
	chg->irq_status = 0;

	/* get real time status of pulse skip irq */
	rc = smblib_read(chg, MISC_PBS_RT_STS_REG, &reg);
	if (rc < 0)
		smblib_err(chg, "Couldn't read MISC_PBS_RT_STS_REG rc=%d\n",
				rc);
	else
		val->intval |= (reg & PULSE_SKIP_IRQ_BIT);
	mutex_unlock(&chg->irq_status_lock);

	return rc;
}

/****************************
 * uUSB Moisture Protection *
 ****************************/
#define MICRO_USB_DETECTION_ON_TIME_20_MS 0x08
#define MICRO_USB_DETECTION_PERIOD_X_100 0x03
#define U_USB_STATUS_WATER_PRESENT 0x00
static int smblib_set_moisture_protection(struct smb_charger *chg,
				bool enable)
{
	int rc = 0;

	if (chg->moisture_present == enable) {
		smblib_dbg(chg, PR_MISC, "No change in moisture protection status\n");
		return rc;
	}

	if (enable) {
		chg->moisture_present = true;

		/* Disable uUSB factory mode detection */
		rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable uUSB factory mode detection rc=%d\n",
				rc);
			return rc;
		}

		/* Disable moisture detection and uUSB state change interrupt */
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable moisture detection interrupt rc=%d\n",
			rc);
			return rc;
		}

		/* Set 1% duty cycle on ID detection */
		rc = smblib_masked_write(chg,
				((chg->chg_param.smb_version == PMI632_SUBTYPE)
				? PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
				TYPEC_U_USB_WATER_PROTECTION_CFG_REG),
				EN_MICRO_USB_WATER_PROTECTION_BIT |
				MICRO_USB_DETECTION_ON_TIME_CFG_MASK |
				MICRO_USB_DETECTION_PERIOD_CFG_MASK,
				EN_MICRO_USB_WATER_PROTECTION_BIT |
				MICRO_USB_DETECTION_ON_TIME_20_MS |
				MICRO_USB_DETECTION_PERIOD_X_100);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set 1 percent CC_ID duty cycle rc=%d\n",
				rc);
			return rc;
		}

		vote(chg->usb_icl_votable, MOISTURE_VOTER, true, 0);
	} else {
		chg->moisture_present = false;
		vote(chg->usb_icl_votable, MOISTURE_VOTER, false, 0);

		/* Enable moisture detection and uUSB state change interrupt */
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable moisture detection and uUSB state change interrupt rc=%d\n",
				rc);
			return rc;
		}

		/* Disable periodic monitoring of CC_ID pin */
		rc = smblib_write(chg,
				((chg->chg_param.smb_version == PMI632_SUBTYPE)
				? PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
				TYPEC_U_USB_WATER_PROTECTION_CFG_REG), 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable 1 percent CC_ID duty cycle rc=%d\n",
				rc);
			return rc;
		}

		/* Enable uUSB factory mode detection */
		rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT,
					EN_MICRO_USB_FACTORY_MODE_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable uUSB factory mode detection rc=%d\n",
				rc);
			return rc;
		}
	}

	smblib_dbg(chg, PR_MISC, "Moisture protection %s\n",
			chg->moisture_present ? "enabled" : "disabled");
	return rc;
}

/*********************
 * VOTABLE CALLBACKS *
 *********************/
static int smblib_smb_disable_override_vote_callback(struct votable *votable,
			void *data, int disable_smb, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;

	/* Enable/disable SMB_EN pin */
	rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
			SMB_EN_OVERRIDE_BIT | SMB_EN_OVERRIDE_VALUE_BIT,
			disable_smb ? SMB_EN_OVERRIDE_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't configure SMB_EN, rc=%d\n", rc);

	return rc;
}

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		return 0;

	/* resume input if suspend is invalid */
	if (suspend < 0)
		suspend = 0;

	return smblib_set_dc_suspend(chg, (bool)suspend);
}

static int smblib_awake_vote_callback(struct votable *votable, void *data,
			int awake, const char *client)
{
	struct smb_charger *chg = data;

	if (awake)
		pm_stay_awake(chg->dev);
	else
		pm_relax(chg->dev);

	return 0;
}

static int smblib_chg_disable_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	/* vivo add */
	if (votable != NULL)
		smblib_dbg(chg, PR_MISC, "chg_disable=%d, vote_client=%s, typec_mode=%d\n",
				chg_disable, client, chg->typec_mode);
	/*vivo end*/
	//fuelsummary_collect_value(ID_MIC_STAT, !chg_disable);
	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 chg_disable ? 0 : CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}

static int smblib_hdc_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq)
		return 0;

	if (chg->irq_info[HIGH_DUTY_CYCLE_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	}

	chg->irq_info[HIGH_DUTY_CYCLE_IRQ].enabled = !disable;

	return 0;
}

static int smblib_limited_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq)
		return 0;

	if (chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(
				chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq);
	}

	chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].enabled = !disable;

	return 0;
}

static int smblib_icl_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq)
		return 0;

	if (chg->irq_info[USBIN_ICL_CHANGE_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	}

	chg->irq_info[USBIN_ICL_CHANGE_IRQ].enabled = !disable;

	return 0;
}

static int smblib_temp_change_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[TEMP_CHANGE_IRQ].irq)
		return 0;

	if (chg->irq_info[TEMP_CHANGE_IRQ].enabled && disable) {
		if (chg->irq_info[TEMP_CHANGE_IRQ].wake)
			disable_irq_wake(chg->irq_info[TEMP_CHANGE_IRQ].irq);
		disable_irq_nosync(chg->irq_info[TEMP_CHANGE_IRQ].irq);
	} else if (!chg->irq_info[TEMP_CHANGE_IRQ].enabled && !disable) {
		enable_irq(chg->irq_info[TEMP_CHANGE_IRQ].irq);
		if (chg->irq_info[TEMP_CHANGE_IRQ].wake)
			enable_irq_wake(chg->irq_info[TEMP_CHANGE_IRQ].irq);
	}

	chg->irq_info[TEMP_CHANGE_IRQ].enabled = !disable;

	return 0;
}

/*******************
 * VCONN REGULATOR *
 * *****************/

int smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 stat, orientation;

	smblib_dbg(chg, PR_OTG, "enabling VCONN\n");

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	/* VCONN orientation is opposite to that of CC */
	orientation =
		stat & TYPEC_CCOUT_VALUE_BIT ? 0 : VCONN_EN_ORIENTATION_BIT;
	rc = smblib_masked_write(chg, TYPE_C_VCONN_CONTROL_REG,
				VCONN_EN_VALUE_BIT | VCONN_EN_ORIENTATION_BIT,
				VCONN_EN_VALUE_BIT | orientation);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
			rc);
		return rc;
	}

	return 0;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	smblib_dbg(chg, PR_OTG, "disabling VCONN\n");
	rc = smblib_masked_write(chg, TYPE_C_VCONN_CONTROL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable vconn regulator rc=%d\n", rc);

	return 0;
}

int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;
	u8 cmd;

	rc = smblib_read(chg, TYPE_C_VCONN_CONTROL_REG, &cmd);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}

	return (cmd & VCONN_EN_VALUE_BIT) ? 1 : 0;
}

/*****************
 * OTG REGULATOR *
 *****************/

int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	smblib_dbg(chg, PR_OTG, "enabling OTG\n");

	rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG, OTG_EN_BIT, OTG_EN_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable OTG rc=%d\n", rc);
		return rc;
	}

	/*vivo add*/
	cancel_delayed_work(&chg->otg_monitor_work);
	chg->otg_disable_pl = false;
	chg->otg_monitor_count = 0;
	chg->drp_ongoing = true;
	schedule_delayed_work(&chg->otg_monitor_work, msecs_to_jiffies(2000));
	/*vivo end*/

	return 0;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	smblib_dbg(chg, PR_OTG, "disabling OTG\n");

	rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG, OTG_EN_BIT, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable OTG regulator rc=%d\n", rc);
		return rc;
	}

	/*vivo add*/
	cancel_delayed_work(&chg->otg_monitor_work);
	chg->otg_monitor_count = 0;
	chg->drp_timer_count = CHECK_DRP_DISABLE_WORK_COUNT;
	chg->drp_ongoing = false;
	/*vivo end*/

	return 0;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 cmd;

	rc = smblib_read(chg, DCDC_CMD_OTG_REG, &cmd);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CMD_OTG rc=%d", rc);
		return rc;
	}

	return (cmd & OTG_EN_BIT) ? 1 : 0;
}

/********************
 * BATT PSY GETTERS *
 ********************/

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	val->intval
		= (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0)
		 && get_client_vote(chg->dc_suspend_votable, USER_VOTER);
	return 0;
}

int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));

	return rc;
}

int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;
	/*vivo add start */
	union power_supply_propval batt_volt = {0,};
	static bool report_flag = false;
	bool fg_soc_really_temp = fg_soc_really;
	/* vivo add end */

	if (chg->fake_capacity >= 0) {
		val->intval = chg->fake_capacity;
		return 0;
	}

	/*vivo modify*/
	if(enable_slave_charger && vivo_ex_fg_i2c_is_exception(chg)) {
		rc = smblib_get_prop_from_qcom_fg(chg, POWER_SUPPLY_PROP_CAPACITY, val);
		smblib_get_prop_from_qcom_fg(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_volt);
	} else {
		rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CAPACITY, val);
		smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_volt);
	}

	if (!rc) {
		if (val->intval == 0 && !report_flag && batt_volt.intval >= 3400000) {
			val->intval = 1;
			smblib_err_ratelimited(chg, "soc=0,batt volt=%dmV(>3.4v),report soc=1!\n", batt_volt.intval/1000);
		} else if (val->intval == 0 && batt_volt.intval < 3400000) {
			report_flag = true;
			smblib_dbg(chg, PR_INTERRUPT, "batt_volt=%d, report soc 0!\n", batt_volt.intval);
		} else if (val->intval != 0) {
			report_flag = false;
		}

		if(enable_slave_charger && !is_atboot && !power_off_charging_mode && !vivo_ex_fg_i2c_is_exception(chg) &&
			((ex_fg_support && ex_fg_probe && ex_fg_soc == val->intval) || !ex_fg_support) && fg_soc_really_temp) {
			val->intval = vivo_battery_handle_ui_soc(chg, val->intval);
		} else if (chg->tracking_soc_enable) {
			if (ex_fg_used == EX_FG_NOT_USED)
				val->intval = smblib_tracking_soc(chg, val->intval, report_flag);
		}

		val->intval = max(0, val->intval);
		val->intval = min(100, val->intval);
	}
	/*vivo end*/
	return rc;
}

static bool is_charging_paused(struct smb_charger *chg)
{
	int rc;
	u8 val;

	rc = smblib_read(chg, CHARGING_PAUSE_CMD_REG, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHARGING_PAUSE_CMD rc=%d\n", rc);
		return false;
	}

	return val & CHARGING_PAUSE_CMD_BIT;
}

#define CUTOFF_COUNT		3
int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	union power_supply_propval vbat = {0, };/* vivo add */
	bool usb_online, dc_online;
	u8 stat;
	int rc, suspend = 0, input_present = 0;

	if (chg->fake_chg_status_on_debug_batt) {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_DEBUG_BATTERY, &pval);
		if (rc < 0) {
			pr_err_ratelimited("Couldn't get debug battery prop rc=%d\n",
					rc);
		} else if (pval.intval == 1) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			return 0;
		}
	}

	rc = smblib_get_prop_batt_health(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get batt health rc=%d\n", rc);
		return rc;
	}
	/*
	 * The charger status register shows charging even though the battery
	 * is discharging when the over voltage condition is hit. Report power
	 * supply state as NOT_CHARGING when the battery health reports
	 * over voltage.
	 */
	if (pval.intval == POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	/*
	 * If SOC = 0 and we are discharging with input connected, report
	 * the battery status as DISCHARGING.
	 */
	smblib_is_input_present(chg, &input_present);
	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (!rc && pval.intval == 0 && input_present) {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
		if (!rc && pval.intval > 0) {
			if (chg->cutoff_count > CUTOFF_COUNT) {
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
				return 0;
			}
			chg->cutoff_count++;
		} else {
			chg->cutoff_count = 0;
		}
	} else {
		chg->cutoff_count = 0;
	}

	if (chg->dbc_usbov) {
		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't get usb present prop rc=%d\n", rc);
			return rc;
		}

		rc = smblib_get_usb_suspend(chg, &suspend);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't get usb suspend rc=%d\n", rc);
			return rc;
		}

		/*
		 * Report charging as long as USBOV is not debounced and
		 * charging path is un-suspended.
		 */
		if (pval.intval && !suspend) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			return 0;
		}
	}

	rc = smblib_get_prop_usb_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc online property rc=%d\n",
			rc);
		return rc;
	}
	dc_online = (bool)pval.intval;

	smblib_get_prop_from_bms(chg,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat);/* vivo add */

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!usb_online && !dc_online) {
		switch (stat) {
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			chg->charge_full = true;	/* vivo add */
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		return rc;
	}

	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		/* vivo add */
		chg->charge_full = true;
		chg->vbat_in_iterm = vbat.intval / 1000;
		/* vivo end */
		break;
	case DISABLE_CHARGE:
	case PAUSE_CHARGE:
		if(chg->usb_temporary_present)	/* vivo add */
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	if (is_charging_paused(chg)) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	/*
	 * If charge termination WA is active and has suspended charging, then
	 * continue reporting charging status as FULL.
	 */
	if (is_client_vote_enabled_locked(chg->usb_icl_votable,
						CHG_TERMINATION_VOTER)) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if (val->intval != POWER_SUPPLY_STATUS_CHARGING)
		return 0;

	if (!usb_online && dc_online
		&& chg->fake_batt_status == POWER_SUPPLY_STATUS_FULL) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
				rc);
			return rc;
	}

	stat &= ENABLE_TRICKLE_BIT | ENABLE_PRE_CHARGING_BIT |
						ENABLE_FULLON_MODE_BIT;

	if (!stat)
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc;
	int effective_fv_uv;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
		if (!rc) {
			/*
			 * If Vbatt is within 40mV above Vfloat, then don't
			 * treat it as overvoltage.
			 */
			effective_fv_uv = get_effective_result_locked(
							chg->fv_votable);
			if (pval.intval >= effective_fv_uv + 40000) {
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
				smblib_err(chg, "battery over-voltage vbat_fg = %duV, fv = %duV\n",
						pval.intval, effective_fv_uv);
				goto done;
			}
		}
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_7_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

done:
	return rc;
}

int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

int smblib_get_prop_system_temp_level_max(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_levels;
	return 0;
}

int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val)
{
	u8 stat;
	int rc;

	if (chg->fake_input_current_limited >= 0) {
		val->intval = chg->fake_input_current_limited;
		return 0;
	}

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n", rc);
		return rc;
	}
	val->intval = (stat & SOFT_ILIMIT_BIT) || chg->is_hdc;
	return 0;
}

int smblib_get_prop_batt_iterm(struct smb_charger *chg,
		union power_supply_propval *val)
{
	int rc, temp;
	u8 stat, buf[2];

	/*
	 * Currently, only ADC comparator-based termination is supported,
	 * hence read only the threshold corresponding to ADC source.
	 * Proceed only if CHGR_ITERM_USE_ANALOG_BIT is 0.
	 */
	rc = smblib_read(chg, CHGR_ENG_CHARGING_CFG_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHGR_ENG_CHARGING_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	if (stat & CHGR_ITERM_USE_ANALOG_BIT) {
		val->intval = -EINVAL;
		return 0;
	}

	rc = smblib_batch_read(chg, CHGR_ADC_ITERM_UP_THD_MSB_REG, buf, 2);

	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHGR_ADC_ITERM_UP_THD_MSB_REG rc=%d\n",
				rc);
		return rc;
	}

	temp = buf[1] | (buf[0] << 8);
	temp = sign_extend32(temp, 15);

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		temp = DIV_ROUND_CLOSEST(temp * ITERM_LIMITS_PMI632_MA,
					ADC_CHG_ITERM_MASK);
	else
		temp = DIV_ROUND_CLOSEST(temp * ITERM_LIMITS_PM8150B_MA,
					ADC_CHG_ITERM_MASK);

	val->intval = temp;

	return rc;
}

int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	val->intval = (stat == TERMINATE_CHARGE);

	/*vivo add*/
	if (chg->force_charge_full_by_high_voltage) {
		val->intval = true;
		smblib_err(chg, "force_charge_full_by_high_voltage=true, force charge_done.\n", rc);
	}
	/*vivo end*/

	return 0;
}

int smblib_get_batt_current_now(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;

	rc = smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_CURRENT_NOW, val);
	/*if (!rc)
		val->intval *= (-1);*//*vivo delete*/

	return rc;
}

/***********************
 * BATTERY PSY SETTERS *
 ***********************/

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

	/* vote 0mA when suspended */
	rc = vote(chg->usb_icl_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_capacity = val->intval;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_batt_status(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	/* Faking battery full */
	if (val->intval == POWER_SUPPLY_STATUS_FULL)
		chg->fake_batt_status = val->intval;
	else
		chg->fake_batt_status = -EINVAL;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	chg->system_temp_level = val->intval;

	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable,
			THERMAL_DAEMON_VOTER, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON_VOTER, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

int smblib_set_prop_input_current_limited(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	chg->fake_input_current_limited = val->intval;
	return 0;
}

int smblib_set_prop_rechg_soc_thresh(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;
	u8 new_thr = DIV_ROUND_CLOSEST(val->intval * 255, 100);

	rc = smblib_write(chg, CHARGE_RCHG_SOC_THRESHOLD_CFG_REG,
			new_thr);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write to RCHG_SOC_THRESHOLD_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	chg->auto_recharge_soc = val->intval;

	return rc;
}

int smblib_run_aicl(struct smb_charger *chg, int type)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
								rc);
		return rc;
	}

	/* USB is suspended so skip re-running AICL */
	if (stat & USBIN_SUSPEND_STS_BIT)
		return rc;

	smblib_dbg(chg, PR_MISC, "re-running AICL\n");

	stat = (type == RERUN_AICL) ? RERUN_AICL_BIT : RESTART_AICL_BIT;
	rc = smblib_masked_write(chg, AICL_CMD_REG, stat, stat);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to AICL_CMD_REG rc=%d\n",
				rc);
	return 0;
}

static int smblib_dp_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 increment */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_INCREMENT_BIT,
			SINGLE_INCREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static int smblib_dm_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 decrement */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_DECREMENT_BIT,
			SINGLE_DECREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

int smblib_force_vbus_voltage(struct smb_charger *chg, u8 val)
{
	int rc;

	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, val, val);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static void smblib_hvdcp_set_fsw(struct smb_charger *chg, int bit)
{
	switch (bit) {
	case QC_5V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_5V);
		break;
	case QC_9V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_9V);
		break;
	case QC_12V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_12V);
		break;
	default:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_removal);
		break;
	}
}

#define QC3_PULSES_FOR_6V	5
#define QC3_PULSES_FOR_9V	20
#define QC3_PULSES_FOR_12V	35
static int smblib_hvdcp3_set_fsw(struct smb_charger *chg)
{
	int pulse_count, rc;

	rc = smblib_get_pulse_cnt(chg, &pulse_count);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
		return rc;
	}

	if (pulse_count < QC3_PULSES_FOR_6V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_5V);
	else if (pulse_count < QC3_PULSES_FOR_9V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_6V_8V);
	else if (pulse_count < QC3_PULSES_FOR_12V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_9V);
	else
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_12V);

	return 0;
}

static void smblib_hvdcp_adaptive_voltage_change(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_CHANGE_STATUS rc=%d\n", rc);
			return;
		}

		smblib_hvdcp_set_fsw(chg, stat & QC_2P0_STATUS_MASK);
		vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3
		|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3P5) {
		rc = smblib_hvdcp3_set_fsw(chg);
		if (rc < 0)
			smblib_err(chg, "Couldn't set QC3.0 Fsw rc=%d\n", rc);
	}

	power_supply_changed(chg->usb_main_psy);
}

int smblib_dp_dm(struct smb_charger *chg, int val)
{
	int target_icl_ua, rc = 0;
	union power_supply_propval pval;
	u8 stat;

	switch (val) {
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		/*
		 * Pre-emptively increment pulse count to enable the setting
		 * of FSW prior to increasing voltage.
		 */
		chg->pulse_cnt++;

		rc = smblib_hvdcp3_set_fsw(chg);
		if (rc < 0)
			smblib_err(chg, "Couldn't set QC3.0 Fsw rc=%d\n", rc);

		rc = smblib_dp_pulse(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't increase pulse count rc=%d\n",
				rc);
			/*
			 * Increment pulse count failed;
			 * reset to former value.
			 */
			chg->pulse_cnt--;
		}

		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DP_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		rc = smblib_dm_pulse(chg);
		if (!rc && chg->pulse_cnt)
			chg->pulse_cnt--;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DM_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		target_icl_ua = get_effective_result(chg->usb_icl_votable);
		if (target_icl_ua < 0) {
			/* no client vote, get the ICL from charger */
			rc = power_supply_get_property(chg->usb_psy,
					POWER_SUPPLY_PROP_HW_CURRENT_MAX,
					&pval);
			if (rc < 0) {
				smblib_err(chg, "Couldn't get max curr rc=%d\n",
					rc);
				return rc;
			}
			target_icl_ua = pval.intval;
		}

		/*
		 * Check if any other voter voted on USB_ICL in case of
		 * voter other than SW_QC3_VOTER reset and restart reduction
		 * again.
		 */
		if (target_icl_ua != get_client_vote(chg->usb_icl_votable,
							SW_QC3_VOTER))
			chg->usb_icl_delta_ua = 0;

		chg->usb_icl_delta_ua += 100000;
		vote(chg->usb_icl_votable, SW_QC3_VOTER, true,
						target_icl_ua - 100000);
		smblib_dbg(chg, PR_PARALLEL, "ICL DOWN ICL=%d reduction=%d\n",
				target_icl_ua, chg->usb_icl_delta_ua);
		break;
	case POWER_SUPPLY_DP_DM_FORCE_5V:
		rc = smblib_force_vbus_voltage(chg, FORCE_5V_BIT);
		if (rc < 0)
			pr_err("Failed to force 5V\n");
		break;
	case POWER_SUPPLY_DP_DM_FORCE_9V:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			smblib_err(chg, "Couldn't set 9V: unsupported\n");
			return -EINVAL;
		}

		/* If we are increasing voltage to get to 9V, set FSW first */
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read QC_CHANGE_STATUS_REG rc=%d\n",
					rc);
			break;
		}

		if (stat & QC_5V_BIT) {
			/* Force 1A ICL before requesting higher voltage */
			vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER,
					true, 1000000);
			smblib_hvdcp_set_fsw(chg, QC_9V_BIT);
		}

		rc = smblib_force_vbus_voltage(chg, FORCE_9V_BIT);
		if (rc < 0)
			pr_err("Failed to force 9V\n");
		break;
	case POWER_SUPPLY_DP_DM_FORCE_12V:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_12V) {
			smblib_err(chg, "Couldn't set 12V: unsupported\n");
			return -EINVAL;
		}

		/* If we are increasing voltage to get to 12V, set FSW first */
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read QC_CHANGE_STATUS_REG rc=%d\n",
					rc);
			break;
		}

		if ((stat & QC_9V_BIT) || (stat & QC_5V_BIT)) {
			/* Force 1A ICL before requesting higher voltage */
			vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER,
					true, 1000000);
			smblib_hvdcp_set_fsw(chg, QC_12V_BIT);
		}

		rc = smblib_force_vbus_voltage(chg, FORCE_12V_BIT);
		if (rc < 0)
			pr_err("Failed to force 12V\n");
		break;
	case POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3P5:
		chg->qc3p5_detected = true;
		smblib_update_usb_type(chg);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
	default:
		break;
	}

	return rc;
}

int smblib_disable_hw_jeita(struct smb_charger *chg, bool disable)
{
	int rc;
	u8 mask;

	/*
	 * Disable h/w base JEITA compensation if s/w JEITA is enabled
	 */
	mask = JEITA_EN_COLD_SL_FCV_BIT
		| JEITA_EN_HOT_SL_FCV_BIT
		| JEITA_EN_HOT_SL_CCC_BIT
		| JEITA_EN_COLD_SL_CCC_BIT
		| JEITA_EN_HARDLIMIT_BIT /*vivo add  for disable jeita*/
		| JEITA_EN_FVCOMP_IN_CV_BIT
		| JEITA_EN_AFP_COLD
		| JEITA_EN_AFP_HOT;/*vivo end*/
	rc = smblib_masked_write(chg, JEITA_EN_CFG_REG, mask,
			disable ? 0 : mask);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure s/w jeita rc=%d\n",
				rc);
		return rc;
	}

	return 0;
}

static int smblib_set_sw_thermal_regulation(struct smb_charger *chg,
						bool enable)
{
	int rc = 0;

	if (!(chg->wa_flags & SW_THERM_REGULATION_WA))
		return rc;

	if (enable) {
		/*
		 * Configure min time to quickly address thermal
		 * condition.
		 */
		rc = smblib_masked_write(chg, SNARL_BARK_BITE_WD_CFG_REG,
			SNARL_WDOG_TIMEOUT_MASK, SNARL_WDOG_TMOUT_62P5MS);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure snarl wdog tmout, rc=%d\n",
					rc);
			return rc;
		}

		/*
		 * Schedule SW_THERM_REGULATION_WORK directly if USB input
		 * is suspended due to SW thermal regulation WA since WDOG
		 * IRQ won't trigger with input suspended.
		 */
		if (is_client_vote_enabled(chg->usb_icl_votable,
						SW_THERM_REGULATION_VOTER)) {
			vote(chg->awake_votable, SW_THERM_REGULATION_VOTER,
								true, 0);
			schedule_delayed_work(&chg->thermal_regulation_work, 0);
		}
	} else {
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, false, 0);
	}

	smblib_dbg(chg, PR_MISC, "WDOG SNARL INT %s\n",
				enable ? "Enabled" : "Disabled");

	return rc;
}

static int smblib_update_thermal_readings(struct smb_charger *chg)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	if (!chg->pl.psy)
		chg->pl.psy = power_supply_get_by_name("parallel");

	rc = smblib_read_iio_channel(chg, chg->iio.die_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->die_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE TEMP channel, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_read_iio_channel(chg, chg->iio.connector_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->connector_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CONN TEMP channel, rc=%d\n", rc);
		return rc;
	}

	/*
	rc = smblib_read_iio_channel(chg, chg->iio.skin_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->skin_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SKIN TEMP channel, rc=%d\n", rc);
		return rc;
	}
	*/

	if (chg->sec_chg_selected == POWER_SUPPLY_CHARGER_SEC_CP) {
		if (is_cp_available(chg)) {
			rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_CP_DIE_TEMP, &pval);
			if (rc < 0) {
				smblib_err(chg, "Couldn't get smb1390 charger temp, rc=%d\n",
					rc);
				return rc;
			}
			chg->smb_temp = pval.intval;
		} else {
			smblib_dbg(chg, PR_MISC, "Coudln't find cp_psy\n");
			chg->smb_temp = -ENODATA;
		}
	} else if (chg->pl.psy && chg->sec_chg_selected ==
					POWER_SUPPLY_CHARGER_SEC_PL) {
		rc = power_supply_get_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CHARGER_TEMP, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get smb1355 charger temp, rc=%d\n",
					rc);
			return rc;
		}
		chg->smb_temp = pval.intval;
	} else {
		chg->smb_temp = -ENODATA;
	}

	return rc;
}

/* SW thermal regulation thresholds in deciDegC */
#define DIE_TEMP_RST_THRESH		1000
#define DIE_TEMP_REG_H_THRESH		800
#define DIE_TEMP_REG_L_THRESH		600

#define CONNECTOR_TEMP_SHDN_THRESH	700
#define CONNECTOR_TEMP_RST_THRESH	600
#define CONNECTOR_TEMP_REG_H_THRESH	550
#define CONNECTOR_TEMP_REG_L_THRESH	500

#define SMB_TEMP_SHDN_THRESH		1400
#define SMB_TEMP_RST_THRESH		900
#define SMB_TEMP_REG_H_THRESH		800
#define SMB_TEMP_REG_L_THRESH		600

#define SKIN_TEMP_SHDN_THRESH		700
#define SKIN_TEMP_RST_THRESH		600
#define SKIN_TEMP_REG_H_THRESH		550
#define SKIN_TEMP_REG_L_THRESH		500

#define THERM_REG_RECHECK_DELAY_1S	1000	/* 1 sec */
#define THERM_REG_RECHECK_DELAY_8S	8000	/* 8 sec */
static int smblib_process_thermal_readings(struct smb_charger *chg)
{
	int rc = 0, wdog_timeout = SNARL_WDOG_TMOUT_8S;
	u32 thermal_status = TEMP_BELOW_RANGE;
	bool suspend_input = false, disable_smb = false;

	/*
	 * Following is the SW thermal regulation flow:
	 *
	 * TEMP_SHUT_DOWN_LEVEL: If either connector temp or skin temp
	 * exceeds their respective SHDN threshold. Need to suspend input
	 * and secondary charger.
	 *
	 * TEMP_SHUT_DOWN_SMB_LEVEL: If smb temp exceed its SHDN threshold
	 * but connector and skin temp are below it. Need to suspend SMB.
	 *
	 * TEMP_ALERT_LEVEL: If die, connector, smb or skin temp exceeds it's
	 * respective RST threshold. Stay put and monitor temperature closely.
	 *
	 * TEMP_ABOVE_RANGE or TEMP_WITHIN_RANGE or TEMP_BELOW_RANGE: If die,
	 * connector, smb or skin temp exceeds it's respective REG_H or REG_L
	 * threshold. Unsuspend input and SMB.
	 */
	if (chg->connector_temp > CONNECTOR_TEMP_SHDN_THRESH ||
		chg->skin_temp > SKIN_TEMP_SHDN_THRESH) {
		thermal_status = TEMP_SHUT_DOWN;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		suspend_input = true;
		disable_smb = true;
		goto out;
	}

	if (chg->smb_temp > SMB_TEMP_SHDN_THRESH) {
		thermal_status = TEMP_SHUT_DOWN_SMB;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		disable_smb = true;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_RST_THRESH ||
			chg->skin_temp > SKIN_TEMP_RST_THRESH ||
			chg->smb_temp > SMB_TEMP_RST_THRESH ||
			chg->die_temp > DIE_TEMP_RST_THRESH) {
		thermal_status = TEMP_ALERT_LEVEL;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_REG_H_THRESH ||
			chg->skin_temp > SKIN_TEMP_REG_H_THRESH ||
			chg->smb_temp > SMB_TEMP_REG_H_THRESH ||
			chg->die_temp > DIE_TEMP_REG_H_THRESH) {
		thermal_status = TEMP_ABOVE_RANGE;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_REG_L_THRESH ||
			chg->skin_temp > SKIN_TEMP_REG_L_THRESH ||
			chg->smb_temp > SMB_TEMP_REG_L_THRESH ||
			chg->die_temp > DIE_TEMP_REG_L_THRESH) {
		thermal_status = TEMP_WITHIN_RANGE;
		wdog_timeout = SNARL_WDOG_TMOUT_8S;
	}
out:
	smblib_dbg(chg, PR_MISC, "Current temperatures: \tDIE_TEMP: %d,\tCONN_TEMP: %d,\tSMB_TEMP: %d,\tSKIN_TEMP: %d\nTHERMAL_STATUS: %d\n",
			chg->die_temp, chg->connector_temp, chg->smb_temp,
			chg->skin_temp, thermal_status);

	if (thermal_status != chg->thermal_status) {
		chg->thermal_status = thermal_status;
		/*
		 * If thermal level changes to TEMP ALERT LEVEL, don't
		 * enable/disable main/parallel charging.
		 */
		if (chg->thermal_status == TEMP_ALERT_LEVEL)
			goto exit;

		vote(chg->smb_override_votable, SW_THERM_REGULATION_VOTER,
				disable_smb, 0);

		/*
		 * Enable/disable secondary charger through votables to ensure
		 * that if SMB_EN pin get's toggled somehow, secondary charger
		 * remains enabled/disabled according to SW thermal regulation.
		 */
		if (!chg->cp_disable_votable)
			chg->cp_disable_votable = find_votable("CP_DISABLE");
		if (chg->cp_disable_votable)
			vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
							disable_smb, 0);

		vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER,
							disable_smb, 0);
		smblib_dbg(chg, PR_MISC, "Parallel %s as per SW thermal regulation\n",
				disable_smb ? "disabled" : "enabled");

		/*
		 * If thermal level changes to TEMP_SHUT_DOWN_SMB, don't
		 * enable/disable main charger.
		 */
		if (chg->thermal_status == TEMP_SHUT_DOWN_SMB)
			goto exit;

		/* Suspend input if SHDN threshold reached */
		vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER,
							suspend_input, 0);
		vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER,
							suspend_input, 0);
		smblib_dbg(chg, PR_MISC, "USB/DC %s as per SW thermal regulation\n",
				suspend_input ? "suspended" : "unsuspended");
	}
exit:
	/*
	 * On USB suspend, WDOG IRQ stops triggering. To continue thermal
	 * monitoring and regulation until USB is plugged out, reschedule
	 * the SW thermal regulation work without releasing the wake lock.
	 */
	if (is_client_vote_enabled(chg->usb_icl_votable,
					SW_THERM_REGULATION_VOTER)) {
		schedule_delayed_work(&chg->thermal_regulation_work,
				msecs_to_jiffies(THERM_REG_RECHECK_DELAY_1S));
		return 0;
	}

	rc = smblib_masked_write(chg, SNARL_BARK_BITE_WD_CFG_REG,
			SNARL_WDOG_TIMEOUT_MASK, wdog_timeout);
	if (rc < 0)
		smblib_err(chg, "Couldn't set WD SNARL timer, rc=%d\n", rc);

	vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, false, 0);
	return rc;
}

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_voltage_wls_output(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE) {
		val->intval = 0;
		return 0;
	}

	rc = smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_dc_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE) {
		val->intval = 0;
		return 0;
	}

	if (get_client_vote(chg->dc_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	if (is_client_vote_enabled(chg->dc_suspend_votable,
						CHG_TERMINATION_VOTER)) {
		rc = smblib_get_prop_dc_present(chg, val);
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	/*smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);*/  /*vivo delete*/

	val->intval = (stat & USE_DCIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);

	return rc;
}

int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.dc_icl, &val->intval);
}

int smblib_get_prop_dc_voltage_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;
	val->intval = MICRO_12V;

	if (!chg->wls_psy)
		chg->wls_psy = power_supply_get_by_name("wireless");

	if (chg->wls_psy) {
		rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX,
				val);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't get VOLTAGE_MAX, rc=%d\n",
					rc);
			return rc;
		}
	}

	return 0;
}

int smblib_get_prop_dc_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);
		return rc;
	}

	return rc;
}

/*******************
 * DC PSY SETTERS *
 *******************/

int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	chg->dcin_icl_user_set = true;
	return smblib_set_charge_param(chg, &chg->param.dc_icl, val->intval);
}

#define DCIN_AICL_RERUN_DELAY_MS	5000
int smblib_set_prop_voltage_wls_output(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_set_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);

	smblib_dbg(chg, PR_WLS, "%d\n", val->intval);

	/*
	 * When WLS VOUT goes down, the power-constrained adaptor may be able
	 * to supply more current, so allow it to do so - unless userspace has
	 * changed DCIN ICL value already due to thermal considerations.
	 */
	if (!chg->dcin_icl_user_set && (val->intval > 0) &&
			(val->intval < chg->last_wls_vout)) {
		alarm_start_relative(&chg->dcin_aicl_alarm,
				ms_to_ktime(DCIN_AICL_RERUN_DELAY_MS));
	}

	chg->last_wls_vout = val->intval;

	return rc;
}

int smblib_set_prop_dc_reset(struct smb_charger *chg)
{
	int rc;

	rc = vote(chg->dc_suspend_votable, VOUT_VOTER, true, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't suspend DC rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_EN_MASK,
				DCIN_EN_OVERRIDE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set DCIN_EN_OVERRIDE_BIT rc=%d\n",
			rc);
		return rc;
	}

	rc = smblib_write(chg, DCIN_CMD_PON_REG, DCIN_PON_BIT | MID_CHG_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write %d to DCIN_CMD_PON_REG rc=%d\n",
			DCIN_PON_BIT | MID_CHG_BIT, rc);
		return rc;
	}

	/* Wait for 10ms to allow the charge to get drained */
	usleep_range(10000, 10010);

	rc = smblib_write(chg, DCIN_CMD_PON_REG, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't clear DCIN_CMD_PON_REG rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_EN_MASK, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't clear DCIN_EN_OVERRIDE_BIT rc=%d\n",
			rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, VOUT_VOTER, false, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't unsuspend  DC rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_MISC, "Wireless charger removal detection successful\n");
	return rc;
}

/*******************
 * USB PSY GETTERS *
 *******************/

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_usb_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote_locked(chg->usb_icl_votable, USER_VOTER) == 0) {
		val->intval = false;
		return rc;
	}

	if (is_client_vote_enabled_locked(chg->usb_icl_votable,
					CHG_TERMINATION_VOTER)) {
		rc = smblib_get_prop_usb_present(chg, val);
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	/* vivo add start */
	if(chg->usb_temporary_present)
		val->intval = chg->usb_temporary_present;
	else if (chg->dchg_status == DIRECT_CHARGER_PREPARE ||
			chg->dchg_status == DIRECT_CHARGER_IS_CHARGERING) {
			val->intval = 1;
	} else /* vivo add end */
		val->intval = (stat & USE_USBIN_BIT) &&
		      	(stat & VALID_INPUT_POWER_SOURCE_STS_BIT);

	/* vivo add start */
	if (power_off_charging_mode && chg->usb_present)
		val->intval = 1;
	/* vivo add end */

	return rc;
}

int smblib_get_usb_online(struct smb_charger *chg,
			union power_supply_propval *val)
{
	int rc;

	rc = smblib_get_prop_usb_online(chg, val);
	if (!val->intval)
		goto exit;

	if (((chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) ||
		(chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB))
		&& (chg->real_charger_type == POWER_SUPPLY_TYPE_USB))
		val->intval = 0;
	else
		val->intval = 1;

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN)
		val->intval = 0;

exit:
	return rc;
}

int smblib_get_prop_usb_voltage_max_design(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			val->intval = MICRO_5V;
			break;
		} else if (chg->qc2_unsupported_voltage ==
				QC2_NON_COMPLIANT_12V) {
			val->intval = MICRO_9V;
			break;
		}
		/* else, fallthrough */
	case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
	case POWER_SUPPLY_TYPE_USB_PD:
		if (chg->chg_param.smb_version == PMI632_SUBTYPE)
			val->intval = MICRO_9V;
		else
			val->intval = MICRO_12V;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_usb_voltage_max(struct smb_charger *chg,
					union power_supply_propval *val)
{
	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			val->intval = MICRO_5V;
			break;
		} else if (chg->qc2_unsupported_voltage ==
				QC2_NON_COMPLIANT_12V) {
			val->intval = MICRO_9V;
			break;
		}
		/* else, fallthrough */
	case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		if (chg->chg_param.smb_version == PMI632_SUBTYPE)
			val->intval = MICRO_9V;
		else
			val->intval = MICRO_12V;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		val->intval = chg->voltage_max_uv;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

#define HVDCP3_STEP_UV	200000
#define HVDCP3P5_STEP_UV	20000
static int smblib_estimate_adaptor_voltage(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	int step_uv = HVDCP3_STEP_UV;

	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		val->intval = MICRO_12V;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
		step_uv = HVDCP3P5_STEP_UV;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		val->intval = MICRO_5V + (step_uv * chg->pulse_cnt);
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		/* Take the average of min and max values */
		val->intval = chg->voltage_min_uv +
			((chg->voltage_max_uv - chg->voltage_min_uv) / 2);
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

static int smblib_read_mid_voltage_chan(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.mid_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.mid_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read MID channel rc=%d\n", rc);
		return rc;
	}

	/*
	 * If MID voltage < 1V, it is unreliable.
	 * Figure out voltage from registers and calculations.
	 */
	if (val->intval < 1000000)
		return smblib_estimate_adaptor_voltage(chg, val);

	return 0;
}

static int smblib_read_usbin_voltage_chan(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.usbin_v_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN channel rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	int rc, ret = 0;
	u8 reg, adc_ch_reg;

	mutex_lock(&chg->adc_lock);

	if (chg->wa_flags & USBIN_ADC_WA) {
		/* Store ADC channel config in order to restore later */
		rc = smblib_read(chg, BATIF_ADC_CHANNEL_EN_REG, &adc_ch_reg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read ADC config rc=%d\n", rc);
			ret = rc;
			goto unlock;
		}

		/* Disable all ADC channels except IBAT channel */
		rc = smblib_write(chg, BATIF_ADC_CHANNEL_EN_REG,
						IBATT_CHANNEL_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable ADC channels rc=%d\n",
						rc);
			ret = rc;
			goto unlock;
		}
	}

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb presence status rc=%d\n", rc);
		ret = -ENODATA;
		goto restore_adc_config;
	}

	/*
	 * Skip reading voltage only if USB is not present and we are not in
	 * OTG mode.
	 */
	if (!pval.intval) {
		rc = smblib_read(chg, DCDC_CMD_OTG_REG, &reg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read CMD_OTG rc=%d", rc);
			goto restore_adc_config;
		}

		if (!(reg & OTG_EN_BIT))
			goto restore_adc_config;
	}

	/*
	 * For PM8150B, use MID_CHG ADC channel because overvoltage is observed
	 * to occur randomly in the USBIN channel, particularly at high
	 * voltages.
	 */
	if (chg->chg_param.smb_version == PM8150B_SUBTYPE)
		rc = smblib_read_mid_voltage_chan(chg, val);
	else
		rc = smblib_read_usbin_voltage_chan(chg, val);
	if (rc < 0) {
		smblib_err(chg, "Failed to read USBIN over vadc, rc=%d\n", rc);
		ret = rc;
	}

restore_adc_config:
	 /* Restore ADC channel config */
	if (chg->wa_flags & USBIN_ADC_WA) {
		rc = smblib_write(chg, BATIF_ADC_CHANNEL_EN_REG, adc_ch_reg);
		if (rc < 0)
			smblib_err(chg, "Couldn't write ADC config rc=%d\n",
						rc);
	}

unlock:
	mutex_unlock(&chg->adc_lock);

	return ret;
}

int smblib_get_prop_vph_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.vph_v_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.vph_v_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read vph channel rc=%d\n", rc);
		return rc;
	}

	return 0;
}

bool smblib_rsbux_low(struct smb_charger *chg, int r_thr)
{
	int r_sbu1, r_sbu2;
	bool ret = false;
	int rc;

	if (!chg->iio.sbux_chan)
		return false;

	/* disable crude sensors */
	rc = smblib_masked_write(chg, TYPE_C_CRUDE_SENSOR_CFG_REG,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT,
			0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable crude sensor rc=%d\n", rc);
		return false;
	}

	/* select SBU1 as current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, SEL_SBU1_ISRC_VAL);
	if (rc < 0) {
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	rc = iio_read_channel_processed(chg->iio.sbux_chan, &r_sbu1);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	if (r_sbu1 < r_thr) {
		ret = true;
		goto cleanup;
	}

	/* select SBU2 as current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, SEL_SBU2_ISRC_VAL);
	if (rc < 0) {
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	rc = iio_read_channel_processed(chg->iio.sbux_chan, &r_sbu2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	if (r_sbu2 < r_thr)
		ret = true;
cleanup:
	/* enable crude sensors */
	rc = smblib_masked_write(chg, TYPE_C_CRUDE_SENSOR_CFG_REG,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable crude sensor rc=%d\n", rc);

	/* disable current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);

	return ret;
}

int smblib_get_prop_charger_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int temp, rc;
	int input_present;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return rc;

	if (input_present == INPUT_NOT_PRESENT)
		return -ENODATA;

	if (chg->iio.temp_chan) {
		rc = iio_read_channel_processed(chg->iio.temp_chan,
				&temp);
		if (rc < 0) {
			pr_err("Error in reading temp channel, rc=%d\n", rc);
			return rc;
		}
		val->intval = temp / 100;
	} else {
		return -ENODATA;
	}

	return rc;
}

int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
					 union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	val->intval = 0;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		return 0;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (stat & CC_ATTACHED_BIT)
		val->intval = (bool)(stat & CC_ORIENTATION_BIT) + 1;

	return rc;
}

static const char * const smblib_typec_mode_name[] = {
	[POWER_SUPPLY_TYPEC_NONE]		  = "NONE",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]	  = "SOURCE_DEFAULT",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]	  = "SOURCE_MEDIUM",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]	  = "SOURCE_HIGH",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]	  = "NON_COMPLIANT",
	[POWER_SUPPLY_TYPEC_SINK]		  = "SINK",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]   = "SINK_POWERED_CABLE",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] = "SINK_DEBUG_ACCESSORY",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]   = "SINK_AUDIO_ADAPTER",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]   = "POWERED_CABLE_ONLY",
};

static int smblib_get_prop_ufp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_SNK_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_1 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_1 = 0x%02x\n", stat);

	switch (stat & DETECTED_SRC_TYPE_MASK) {
	case SNK_RP_STD_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	case SNK_RP_1P5_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case SNK_RP_3P0_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	case SNK_RP_SHORT_BIT:
		return POWER_SUPPLY_TYPEC_NON_COMPLIANT;
	case SNK_DAM_500MA_BIT:
	case SNK_DAM_1500MA_BIT:
	case SNK_DAM_3000MA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int smblib_get_prop_dfp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (chg->lpd_stage == LPD_STAGE_COMMIT)
		return POWER_SUPPLY_TYPEC_NONE;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_SRC_STATUS_REG = 0x%02x\n", stat);

	switch (stat & DETECTED_SNK_TYPE_MASK) {
	case AUDIO_ACCESS_RA_RA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
	case SRC_DEBUG_ACCESS_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	case SRC_RD_RA_VCONN_BIT:
		return POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
	case SRC_RD_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_SINK;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int smblib_get_prop_typec_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
				rc);
		return 0;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_MISC_STATUS_REG = 0x%02x\n", stat);

	if (stat & SNK_SRC_MODE_BIT)
		return smblib_get_prop_dfp_mode(chg);
	else
		return smblib_get_prop_ufp_mode(chg);
}

inline int smblib_get_usb_prop_typec_mode(struct smb_charger *chg,
				union power_supply_propval *val)
{
	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		val->intval = POWER_SUPPLY_TYPEC_NONE;
	else
		val->intval = chg->typec_mode;

	return 0;
}

int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return 0;
	}

	rc = smblib_read(chg, TYPE_C_MODE_CFG_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MODE_CFG_REG rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_MODE_CFG_REG = 0x%02x\n",
		   ctrl);

	if (ctrl & TYPEC_DISABLE_CMD_BIT) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return rc;
	}

	switch (ctrl & (EN_SRC_ONLY_BIT | EN_SNK_ONLY_BIT)) {
	case 0:
		val->intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		break;
	case EN_SRC_ONLY_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		break;
	case EN_SNK_ONLY_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SINK;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		smblib_err(chg, "unsupported power role 0x%02lx\n",
			ctrl & (EN_SRC_ONLY_BIT | EN_SNK_ONLY_BIT));
		return -EINVAL;
	}

	chg->power_role = val->intval;
	return rc;
}

static inline bool typec_in_src_mode(struct smb_charger *chg)
{
	return (chg->typec_mode > POWER_SUPPLY_TYPEC_NONE &&
		chg->typec_mode < POWER_SUPPLY_TYPEC_SOURCE_DEFAULT);
}

int smblib_get_prop_typec_select_rp(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc, rp;
	u8 stat;

	if (!typec_in_src_mode(chg))
		return -ENODATA;

	rc = smblib_read(chg, TYPE_C_CURRSRC_CFG_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_CURRSRC_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	switch (stat & TYPEC_SRC_RP_SEL_MASK) {
	case TYPEC_SRC_RP_STD:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_STD;
		break;
	case TYPEC_SRC_RP_1P5A:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_1P5A;
		break;
	case TYPEC_SRC_RP_3A:
	case TYPEC_SRC_RP_3A_DUPLICATE:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_3A;
		break;
	default:
		return -EINVAL;
	}

	val->intval = rp;

	return 0;
}

int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	int rc = 0, buck_scale = 1, boost_scale = 1;

	if (chg->iio.usbin_i_chan) {
		rc = iio_read_channel_processed(chg->iio.usbin_i_chan,
				&val->intval);
		if (rc < 0) {
			pr_err("Error in reading USBIN_I channel, rc=%d\n", rc);
			return rc;
		}

		/*
		 * For PM8150B, scaling factor = reciprocal of
		 * 0.2V/A in Buck mode, 0.4V/A in Boost mode.
		 * For PMI632, scaling factor = reciprocal of
		 * 0.4V/A in Buck mode, 0.8V/A in Boost mode.
		 */
		switch (chg->chg_param.smb_version) {
		case PMI632_SUBTYPE:
			buck_scale = 40;
			boost_scale = 80;
			break;
		default:
			buck_scale = 20;
			boost_scale = 40;
			break;
		}

		if (chg->otg_present || smblib_get_prop_dfp_mode(chg) !=
				POWER_SUPPLY_TYPEC_NONE) {
			val->intval = DIV_ROUND_CLOSEST(val->intval * 100,
								boost_scale);
			return rc;
		}

		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get usb present status,rc=%d\n",
				rc);
			return -ENODATA;
		}

		/* If USB is not present, return 0 */
		if (!pval.intval)
			val->intval = 0;
		else
			val->intval = DIV_ROUND_CLOSEST(val->intval * 100,
								buck_scale);
	} else {
		val->intval = 0;
		rc = -ENODATA;
	}

	return rc;
}

int smblib_get_prop_low_power(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (chg->sink_src_mode != SRC_MODE)
		return -ENODATA;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return rc;
	}

	val->intval = !(stat & SRC_HIGH_BATT_BIT);

	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

int smblib_get_prop_input_current_max(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	int icl_ua = 0, rc;

	rc = smblib_get_charge_param(chg, &chg->param.usb_icl, &icl_ua);
	if (rc < 0)
		return rc;

	if (is_override_vote_enabled_locked(chg->usb_icl_votable) &&
					icl_ua < USBIN_1000MA) {
		val->intval = USBIN_1000MA;
		return 0;
	}

	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

int smblib_get_prop_input_voltage_settled(struct smb_charger *chg,
						union power_supply_propval *val)
{
	int rc, pulses;
	int step_uv = HVDCP3_STEP_UV;

	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
		step_uv = HVDCP3P5_STEP_UV;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		rc = smblib_get_pulse_cnt(chg, &pulses);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return 0;
		}
		val->intval = MICRO_5V + step_uv * pulses;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		val->intval = chg->voltage_min_uv;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = chg->pd_hard_reset;
	return 0;
}

int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = chg->ok_to_pd;
	return 0;
}

int smblib_get_prop_smb_health(struct smb_charger *chg)
{
	int rc;
	int input_present;
	union power_supply_propval prop = {0, };

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return rc;

	if ((input_present == INPUT_NOT_PRESENT) || (!is_cp_available(chg)))
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_CP_DIE_TEMP, &prop);
	if (rc < 0)
		return rc;

	if (prop.intval > SMB_TEMP_RST_THRESH)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (prop.intval > SMB_TEMP_REG_H_THRESH)
		return POWER_SUPPLY_HEALTH_HOT;

	if (prop.intval > SMB_TEMP_REG_L_THRESH)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_prop_die_health(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	int input_present;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return rc;

	if (input_present == INPUT_NOT_PRESENT)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		if (chg->die_temp == -ENODATA)
			return POWER_SUPPLY_HEALTH_UNKNOWN;

		if (chg->die_temp > DIE_TEMP_RST_THRESH)
			return POWER_SUPPLY_HEALTH_OVERHEAT;

		if (chg->die_temp > DIE_TEMP_REG_H_THRESH)
			return POWER_SUPPLY_HEALTH_HOT;

		if (chg->die_temp > DIE_TEMP_REG_L_THRESH)
			return POWER_SUPPLY_HEALTH_WARM;

		return POWER_SUPPLY_HEALTH_COOL;
	}

	rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE_TEMP_STATUS_REG, rc=%d\n",
				rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & DIE_TEMP_RST_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (stat & DIE_TEMP_UB_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (stat & DIE_TEMP_LB_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_die_health(struct smb_charger *chg,
			union power_supply_propval *val)
{
	if (chg->die_health == -EINVAL)
		val->intval = smblib_get_prop_die_health(chg);
	else
		val->intval = chg->die_health;

	return 0;
}

int smblib_get_prop_scope(struct smb_charger *chg,
			union power_supply_propval *val)
{
	int rc;
	union power_supply_propval pval;

	val->intval = POWER_SUPPLY_SCOPE_UNKNOWN;
	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0)
		return rc;

	val->intval = pval.intval ? POWER_SUPPLY_SCOPE_DEVICE
		: chg->otg_present ? POWER_SUPPLY_SCOPE_SYSTEM
		: POWER_SUPPLY_SCOPE_UNKNOWN;

	return 0;
}

static int smblib_get_typec_connector_temp_status(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	return POWER_SUPPLY_HEALTH_COOL;	//vivo skip (PIN: PM8150B_CONN_THERM  usb_conn_temp Heat control in Qcom. default setting)

	if (chg->connector_health != -EINVAL)
		return chg->connector_health;

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		if (chg->connector_temp == -ENODATA)
			return POWER_SUPPLY_HEALTH_UNKNOWN;

		if (chg->connector_temp > CONNECTOR_TEMP_RST_THRESH)
			return POWER_SUPPLY_HEALTH_OVERHEAT;

		if (chg->connector_temp > CONNECTOR_TEMP_REG_H_THRESH)
			return POWER_SUPPLY_HEALTH_HOT;

		if (chg->connector_temp > CONNECTOR_TEMP_REG_L_THRESH)
			return POWER_SUPPLY_HEALTH_WARM;

		return POWER_SUPPLY_HEALTH_COOL;
	}

	rc = smblib_read(chg, CONNECTOR_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CONNECTOR_TEMP_STATUS_REG, rc=%d\n",
				rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & CONNECTOR_TEMP_RST_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (stat & CONNECTOR_TEMP_UB_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (stat & CONNECTOR_TEMP_LB_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_skin_temp_status(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (!chg->en_skin_therm_mitigation)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	rc = smblib_read(chg, SKIN_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SKIN_TEMP_STATUS_REG, rc=%d\n",
				rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & SKIN_TEMP_RST_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (stat & SKIN_TEMP_UB_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (stat & SKIN_TEMP_LB_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_prop_connector_health(struct smb_charger *chg)
{
	bool dc_present, usb_present;
	int input_present;
	int rc;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	dc_present = input_present & INPUT_PRESENT_DC;
	usb_present = input_present & INPUT_PRESENT_USB;

	if (usb_present)
		return smblib_get_typec_connector_temp_status(chg);

	/*
	 * In PM8150B, SKIN channel measures Wireless charger receiver
	 * temp, used to regulate DC ICL.
	 */
	if (chg->chg_param.smb_version == PM8150B_SUBTYPE && dc_present)
		return smblib_get_skin_temp_status(chg);

	return POWER_SUPPLY_HEALTH_COOL;
}

static int get_rp_based_dcp_current(struct smb_charger *chg, int typec_mode)
{
	int rp_ua;

	switch (typec_mode) {
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		rp_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	/* fall through */
	default:
		rp_ua = DCP_CURRENT_UA;
	}

	return rp_ua;
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, icl;
	int curr_ua = 0;/*vivo add*/

	smblib_dbg(chg, PR_MISC, "typec log: pd active=%d, current=%d\n", chg->pd_active, val->intval);

	if (chg->pd_active) {
		icl = get_client_vote(chg->usb_icl_votable, PD_VOTER);
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
		if (val->intval != icl)
			power_supply_changed(chg->usb_psy);

		/*vivo add*/
		curr_ua = max(get_client_vote_locked(chg->usb_icl_votable, USER_DEFAULT_INPUT_VOTER), val->intval);
		curr_ua = (curr_ua > chg->total_input_current_max_ua) ? chg->total_input_current_max_ua : curr_ua;

		if (curr_ua != get_client_vote_locked(chg->usb_icl_votable, USER_DEFAULT_INPUT_VOTER)) {
			smblib_dbg(chg, PR_MISC, "typec log: PD active and increase USER_DEFAULT_INPUT_VOTER current to %d\n", curr_ua);
			rc = vote(chg->usb_icl_votable, USER_DEFAULT_INPUT_VOTER, true, curr_ua);
		}
		/*vivo end*/
	} else {
		rc = -EPERM;
	}

	return rc;
}

static int smblib_handle_usb_current(struct smb_charger *chg,
					int usb_current)
{
	int rc = 0, rp_ua, typec_mode;
	union power_supply_propval val = {0, };

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
		return 0;/*vivo add,not need qcom method*/
		if (usb_current == -ETIMEDOUT) {
			if ((chg->float_cfg & FLOAT_OPTIONS_MASK)
						== FORCE_FLOAT_SDP_CFG_BIT) {
				/*
				 * Confiugure USB500 mode if Float charger is
				 * configured for SDP mode.
				 */
				rc = vote(chg->usb_icl_votable,
					SW_ICL_MAX_VOTER, true, USBIN_500MA);
				if (rc < 0)
					smblib_err(chg,
						"Couldn't set SDP ICL rc=%d\n",
						rc);
				return rc;
			}

			if (chg->connector_type ==
					POWER_SUPPLY_CONNECTOR_TYPEC) {
				/*
				 * Valid FLOAT charger, report the current
				 * based of Rp.
				 */
				typec_mode = smblib_get_prop_typec_mode(chg);
				rp_ua = get_rp_based_dcp_current(chg,
								typec_mode);
				rc = vote(chg->usb_icl_votable,
						SW_ICL_MAX_VOTER, true, rp_ua);
				if (rc < 0)
					return rc;
			} else {
				rc = vote(chg->usb_icl_votable,
					SW_ICL_MAX_VOTER, true, DCP_CURRENT_UA);
				if (rc < 0)
					return rc;
			}
		} else {
			/*
			 * FLOAT charger detected as SDP by USB driver,
			 * charge with the requested current and update the
			 * real_charger_type
			 */
			chg->real_charger_type = POWER_SUPPLY_TYPE_USB;
			rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
						true, usb_current);
			if (rc < 0)
				return rc;
			rc = vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER,
							false, 0);
			if (rc < 0)
				return rc;
		}
	} else {
		rc = smblib_get_prop_usb_present(chg, &val);
		if (!rc && !val.intval)
			return 0;

		/* if flash is active force 500mA */
		if ((usb_current < SDP_CURRENT_UA) && is_flash_active(chg))
			usb_current = SDP_CURRENT_UA;

		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
							usb_current);
		if (rc < 0) {
			pr_err("Couldn't vote ICL USB_PSY_VOTER rc=%d\n", rc);
			return rc;
		}

		rc = vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		if (rc < 0) {
			pr_err("Couldn't remove SW_ICL_MAX vote rc=%d\n", rc);
			return rc;
		}

	}

	return 0;
}

int smblib_set_prop_sdp_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc = 0;

	if (!chg->pd_active) {
		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get usb present rc = %d\n",
						rc);
			return rc;
		}

		/* handle the request only when USB is present */
		if (pval.intval)
			rc = smblib_handle_usb_current(chg, val->intval);
	} else if (chg->system_suspend_supported) {
		if (val->intval <= USBIN_25MA)
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, true, val->intval);
		else
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, false, 0);
	}

	/*vivo add*/
	pr_err("pd active=%d, current=%d, system_suspend_supported=%d\n",
	chg->pd_active, val->intval, chg->system_suspend_supported);
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB &&
		val->intval > get_client_vote_locked(chg->usb_icl_votable, USER_DEFAULT_INPUT_VOTER)) {
		rc = vote(chg->usb_icl_votable, USER_DEFAULT_INPUT_VOTER, true, val->intval);
		pr_err("re-vote usb_icl(USER_DEFAULT_INPUT_VOTER) to %d\n", val->intval);
	}
	/*vivo end*/
	return rc;
}

int smblib_set_prop_boost_current(struct smb_charger *chg,
					const union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
				val->intval <= chg->boost_threshold_ua ?
				chg->chg_freq.freq_below_otg_threshold :
				chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0) {
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);
		return rc;
	}

	chg->boost_current_ua = val->intval;
	return rc;
}

int smblib_set_prop_usb_voltage_max_limit(struct smb_charger *chg,
					const union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };

	/* Exit if same value is re-configured */
	if (val->intval == chg->usbin_forced_max_uv)
		return 0;

	smblib_get_prop_usb_voltage_max_design(chg, &pval);

	if (val->intval >= MICRO_5V && val->intval <= pval.intval) {
		chg->usbin_forced_max_uv = val->intval;
		smblib_dbg(chg, PR_MISC, "Max VBUS limit changed to: %d\n",
				val->intval);
	} else if (chg->usbin_forced_max_uv) {
		chg->usbin_forced_max_uv = 0;
	} else {
		return 0;
	}

	power_supply_changed(chg->usb_psy);

	return 0;
}

void smblib_typec_irq_config(struct smb_charger *chg, bool en)
{
	if (en == chg->typec_irq_en)
		return;

	if (en) {
		enable_irq(
			chg->irq_info[TYPEC_ATTACH_DETACH_IRQ].irq);
		enable_irq(
			chg->irq_info[TYPEC_CC_STATE_CHANGE_IRQ].irq);
		enable_irq(
			chg->irq_info[TYPEC_OR_RID_DETECTION_CHANGE_IRQ].irq);
	} else {
		disable_irq_nosync(
			chg->irq_info[TYPEC_ATTACH_DETACH_IRQ].irq);
		disable_irq_nosync(
			chg->irq_info[TYPEC_CC_STATE_CHANGE_IRQ].irq);
		disable_irq_nosync(
			chg->irq_info[TYPEC_OR_RID_DETECTION_CHANGE_IRQ].irq);
	}

	chg->typec_irq_en = en;
}

#define PR_LOCK_TIMEOUT_MS	1000
int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				     const union power_supply_propval *val)
{
	int rc = 0;
	u8 power_role;
	enum power_supply_typec_mode typec_mode;
	bool snk_attached = false, src_attached = false, is_pr_lock = false;
	/*vivo add*/
	int type;

	smblib_dbg(chg, PR_MISC, "power role set val=%d,drp_backup_state=%d,usb_det_pin_mode=%d\n",
				val->intval, chg->drp_backup_state, chg->usb_det_pin_mode);
	chg->drp_backup_state = val->intval;
	type = val->intval;

	if (!chg->usb_det_pin_mode) {
		if (chg->drp_enable) {
			type = POWER_SUPPLY_TYPEC_PR_DUAL;
			smblib_dbg(chg, PR_MISC, "drp_enable=1, only set drp mode!\n");
		} else {
			type = POWER_SUPPLY_TYPEC_PR_SINK;
			smblib_dbg(chg, PR_MISC, "drp_enable=0, only set sink mode!\n");
		}
	} else if (type >= POWER_SUPPLY_TYPEC_PR_DELTA) {
		type -= POWER_SUPPLY_TYPEC_PR_DELTA;
		smblib_dbg(chg, PR_MISC, "usb_det_pin_mode:set type=%d!\n", type);
	} else {
		smblib_dbg(chg, PR_MISC, "usb_det_pin_mode:not vivo pri-mode, not allow set typec power role!\n");
		return 0;
	}
	/*vivo end*/

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		return 0;

	smblib_dbg(chg, PR_MISC, "power role change: %d --> %d!",
			chg->power_role, type);/*vivo modify*/

	if (chg->power_role == type) {/*vivo modify*/
		smblib_dbg(chg, PR_MISC, "power role already in %d, ignore!",
				chg->power_role);
		return 0;
	}

	typec_mode = smblib_get_prop_typec_mode(chg);
	if (typec_mode >= POWER_SUPPLY_TYPEC_SINK &&
			typec_mode <= POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER)
		snk_attached = true;
	else if (typec_mode >= POWER_SUPPLY_TYPEC_SOURCE_DEFAULT &&
			typec_mode <= POWER_SUPPLY_TYPEC_SOURCE_HIGH)
		src_attached = true;

	/*
	 * If current power role is in DRP, and type-c is already in the
	 * mode (source or sink) that's being requested, it means this is
	 * a power role locking request from USBPD driver. Disable type-c
	 * related interrupts for locking power role to avoid the redundant
	 * notifications.
	 */
	if ((chg->power_role == POWER_SUPPLY_TYPEC_PR_DUAL) &&
		((src_attached && type == POWER_SUPPLY_TYPEC_PR_SINK) ||
		(snk_attached && type == POWER_SUPPLY_TYPEC_PR_SOURCE)))/*vivo modify*/
		is_pr_lock = true;

	smblib_dbg(chg, PR_MISC, "snk_attached = %d, src_attached = %d, is_pr_lock = %d\n",
			snk_attached, src_attached, is_pr_lock);
	cancel_delayed_work(&chg->pr_lock_clear_work);
	spin_lock(&chg->typec_pr_lock);
	if (!chg->pr_lock_in_progress && is_pr_lock) {
		smblib_dbg(chg, PR_MISC, "disable type-c interrupts for power role locking\n");
		smblib_typec_irq_config(chg, false);
		schedule_delayed_work(&chg->pr_lock_clear_work,
					msecs_to_jiffies(PR_LOCK_TIMEOUT_MS));
	} else if (chg->pr_lock_in_progress && !is_pr_lock) {
		smblib_dbg(chg, PR_MISC, "restore type-c interrupts after exit power role locking\n");
		smblib_typec_irq_config(chg, true);
	}

	chg->pr_lock_in_progress = is_pr_lock;
	spin_unlock(&chg->typec_pr_lock);

	switch (type) {/*vivo modify*/
	case POWER_SUPPLY_TYPEC_PR_NONE:
		power_role = TYPEC_DISABLE_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_DUAL:
		power_role = chg->typec_try_mode;
		break;
	case POWER_SUPPLY_TYPEC_PR_SINK:
		power_role = EN_SNK_ONLY_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_SOURCE:
		power_role = EN_SRC_ONLY_BIT;
		break;
	default:
		smblib_err(chg, "power role %d not supported\n", type);/*vivo modify*/
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
				TYPEC_POWER_ROLE_CMD_MASK | TYPEC_TRY_MODE_MASK,
				power_role);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			power_role, rc);
		return rc;
	}

	chg->power_role = type;/*vivo modify*/
	return rc;
}

int smblib_set_prop_typec_select_rp(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (!typec_in_src_mode(chg)) {
		smblib_err(chg, "Couldn't set curr src: not in SRC mode\n");
		return -EINVAL;
	}

	if (val->intval < TYPEC_SRC_RP_MAX_ELEMENTS) {
		rc = smblib_masked_write(chg, TYPE_C_CURRSRC_CFG_REG,
				TYPEC_SRC_RP_SEL_MASK,
				val->intval);
		if (rc < 0)
			smblib_err(chg, "Couldn't write to TYPE_C_CURRSRC_CFG rc=%d\n",
					rc);
		return rc;
	}

	return -EINVAL;
}

int smblib_set_prop_pd_voltage_min(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, min_uv;

	min_uv = min(val->intval, chg->voltage_max_uv);
	if (chg->voltage_min_uv == min_uv)
		return 0;

	rc = smblib_set_usb_pd_allowed_voltage(chg, min_uv,
					       chg->voltage_max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_min_uv = min_uv;
	power_supply_changed(chg->usb_main_psy);

	return rc;
}

int smblib_set_prop_pd_voltage_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, max_uv;

	max_uv = max(val->intval, chg->voltage_min_uv);
	if (chg->voltage_max_uv == max_uv)
		return 0;

	rc = smblib_set_usb_pd_fsw(chg, max_uv);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set FSW for voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	rc = smblib_set_usb_pd_allowed_voltage(chg, chg->voltage_min_uv,
					       max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = max_uv;
	power_supply_changed(chg->usb_main_psy);

	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);

	int rc = 0;
	int sec_charger, typec_mode;
 	
	/* vivo add log */
	smblib_dbg(chg, PR_INTERRUPT, "typec log: pd_active change from %d -> %d.\n", chg->pd_active, val->intval);

	/*
	 * Ignore repetitive notification while PD is active, which
	 * is caused by hard reset.
	 */
	if (chg->pd_active && chg->pd_active == val->intval)
		return 0;

	chg->pd_active = val->intval;

	smblib_apsd_enable(chg, !chg->pd_active);

	update_sw_icl_max(chg, apsd->pst);

	if (chg->pd_active) {
		vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
				false, 0);
		vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER,
				false, 0);

		/*
		 * Enforce 100mA for PD until the real vote comes in later.
		 * It is guaranteed that pd_active is set prior to
		 * pd_current_max
		 */
		vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_100MA);
		vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);

		/*
		 * For PPS, Charge Pump is preferred over parallel charger if
		 * present.
		 */
		if (chg->pd_active == POWER_SUPPLY_PD_PPS_ACTIVE
						&& chg->sec_cp_present) {
			rc = smblib_select_sec_charger(chg,
						POWER_SUPPLY_CHARGER_SEC_CP,
						POWER_SUPPLY_CP_PPS, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't enable secondary charger rc=%d\n",
					rc);
		}
	} else {
		vote(chg->usb_icl_votable, PD_VOTER, false, 0);
		vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
				true, 0);
		vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER,
				true, 0);

		sec_charger = chg->sec_pl_present ?
						POWER_SUPPLY_CHARGER_SEC_PL :
						POWER_SUPPLY_CHARGER_SEC_NONE;
		rc = smblib_select_sec_charger(chg, sec_charger,
						POWER_SUPPLY_CP_NONE, false);
		if (rc < 0)
			dev_err(chg->dev,
				"Couldn't enable secondary charger rc=%d\n",
					rc);

		/* PD hard resets failed, proceed to detect QC2/3 */
		if (chg->ok_to_pd) {
			chg->ok_to_pd = false;
			smblib_hvdcp_detect_try_enable(chg, true);
		}
	}

	smblib_usb_pd_adapter_allowance_override(chg,
			!!chg->pd_active ? FORCE_5V : FORCE_NULL);
	smblib_update_usb_type(chg);

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB &&
			!chg->ok_to_pd) {
		typec_mode = smblib_get_prop_typec_mode(chg);
		if (typec_rp_med_high(chg, typec_mode))
			vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	}

	power_supply_changed(chg->usb_psy);
	return rc;
}

int smblib_set_prop_ship_mode(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "Set ship mode: %d!!\n", !!val->intval);

	rc = smblib_masked_write(chg, SHIP_MODE_REG, SHIP_MODE_EN_BIT,
			!!val->intval ? SHIP_MODE_EN_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't %s ship mode, rc=%d\n",
				!!val->intval ? "enable" : "disable", rc);

	return rc;
}

int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc = 0;

	if (chg->pd_hard_reset == val->intval)
		return rc;

	chg->pd_hard_reset = val->intval;
	rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
			EXIT_SNK_BASED_ON_CC_BIT,
			(chg->pd_hard_reset) ? EXIT_SNK_BASED_ON_CC_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set EXIT_SNK_BASED_ON_CC rc=%d\n",
				rc);

	return rc;
}

#define JEITA_SOFT			0
#define JEITA_HARD			1
static int smblib_update_jeita(struct smb_charger *chg, u32 *thresholds,
								int type)
{
	int rc;
	u16 temp, base;

	base = CHGR_JEITA_THRESHOLD_BASE_REG(type);

	temp = thresholds[1] & 0xFFFF;
	temp = ((temp & 0xFF00) >> 8) | ((temp & 0xFF) << 8);
	rc = smblib_batch_write(chg, base, (u8 *)&temp, 2);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't configure Jeita %s hot threshold rc=%d\n",
			(type == JEITA_SOFT) ? "Soft" : "Hard", rc);
		return rc;
	}

	temp = thresholds[0] & 0xFFFF;
	temp = ((temp & 0xFF00) >> 8) | ((temp & 0xFF) << 8);
	rc = smblib_batch_write(chg, base + 2, (u8 *)&temp, 2);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't configure Jeita %s cold threshold rc=%d\n",
			(type == JEITA_SOFT) ? "Soft" : "Hard", rc);
		return rc;
	}

	smblib_dbg(chg, PR_MISC, "%s Jeita threshold configured\n",
				(type == JEITA_SOFT) ? "Soft" : "Hard");

	return 0;
}

static int smblib_charge_inhibit_en(struct smb_charger *chg, bool enable)
{
	int rc;

	rc = smblib_masked_write(chg, CHGR_CFG2_REG,
					CHARGER_INHIBIT_BIT,
					enable ? CHARGER_INHIBIT_BIT : 0);
	return rc;
}

static int smblib_soft_jeita_arb_wa(struct smb_charger *chg)
{
	union power_supply_propval pval;
	int rc = 0;
	bool soft_jeita;

	rc = smblib_get_prop_batt_health(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get battery health rc=%d\n", rc);
		return rc;
	}

	/*vivo add for remove this function*/
	return 0;
	/*vivo end*/

	/* Do nothing on entering hard JEITA condition */
	if (pval.intval == POWER_SUPPLY_HEALTH_COLD ||
		pval.intval == POWER_SUPPLY_HEALTH_HOT)
		return 0;

	if (chg->jeita_soft_fcc[0] < 0 || chg->jeita_soft_fcc[1] < 0 ||
		chg->jeita_soft_fv[0] < 0 || chg->jeita_soft_fv[1] < 0)
		return 0;

	soft_jeita = (pval.intval == POWER_SUPPLY_HEALTH_COOL) ||
			(pval.intval == POWER_SUPPLY_HEALTH_WARM);

	/* Do nothing on entering soft JEITA from hard JEITA */
	if (chg->jeita_arb_flag && soft_jeita)
		return 0;

	/* Do nothing, initial to health condition */
	if (!chg->jeita_arb_flag && !soft_jeita)
		return 0;

	/* Entering soft JEITA from normal state */
	if (!chg->jeita_arb_flag && soft_jeita) {
		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, true, 0);

		rc = smblib_charge_inhibit_en(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable charge inhibit rc=%d\n",
					rc);

		rc = smblib_update_jeita(chg, chg->jeita_soft_hys_thlds,
					JEITA_SOFT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't configure Jeita soft threshold rc=%d\n",
				rc);

		if (pval.intval == POWER_SUPPLY_HEALTH_COOL) {
			vote(chg->fcc_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fcc[0]);
			vote(chg->fv_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fv[0]);
		} else {
			vote(chg->fcc_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fcc[1]);
			vote(chg->fv_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fv[1]);
		}

		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
		chg->jeita_arb_flag = true;
	} else if (chg->jeita_arb_flag && !soft_jeita) {
		/* Exit to health state from soft JEITA */

		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, true, 0);

		rc = smblib_charge_inhibit_en(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable charge inhibit rc=%d\n",
					rc);

		rc = smblib_update_jeita(chg, chg->jeita_soft_thlds,
							JEITA_SOFT);
		if (rc < 0)
			smblib_err(chg, "Couldn't configure Jeita soft threshold rc=%d\n",
				rc);

		vote(chg->fcc_votable, JEITA_ARB_VOTER, false, 0);
		vote(chg->fv_votable, JEITA_ARB_VOTER, false, 0);
		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
		chg->jeita_arb_flag = false;
	}

	smblib_dbg(chg, PR_MISC, "JEITA ARB status %d, soft JEITA status %d\n",
			chg->jeita_arb_flag, soft_jeita);
	return rc;
}

/************************
 * USB MAIN PSY GETTERS *
 ************************/
int smblib_get_prop_fcc_delta(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc, jeita_cc_delta_ua = 0;

	if (chg->sw_jeita_enabled) {
		val->intval = 0;
		return 0;
	}

	rc = smblib_get_jeita_cc_delta(chg, &jeita_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc delta rc=%d\n", rc);
		jeita_cc_delta_ua = 0;
	}

	val->intval = jeita_cc_delta_ua;
	return 0;
}

/************************
 * USB MAIN PSY SETTERS *
 ************************/
int smblib_get_charge_current(struct smb_charger *chg,
				int *total_current_ua)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);
	union power_supply_propval val = {0, };
	int rc = 0, typec_source_rd, current_ua;
	bool non_compliant;
	u8 stat;

	if (chg->pd_active) {
		*total_current_ua =
			get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
		return rc;
	}

	rc = smblib_read(chg, LEGACY_CABLE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return rc;
	}
	non_compliant = stat & TYPEC_NONCOMP_LEGACY_CABLE_STATUS_BIT;

	/* get settled ICL */
	rc = smblib_get_prop_input_current_settled(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		return rc;
	}

	typec_source_rd = smblib_get_prop_ufp_mode(chg);

	/* QC 2.0/3.0 adapter */
	if (apsd_result->bit & (QC_3P0_BIT | QC_2P0_BIT)) {
		*total_current_ua = HVDCP_CURRENT_UA;
		return 0;
	}

	if (non_compliant && !chg->typec_legacy_use_rp_icl) {
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_UA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = DCP_CURRENT_UA;
			break;
		default:
			current_ua = 0;
			break;
		}

		*total_current_ua = max(current_ua, val.intval);
		return 0;
	}

	switch (typec_source_rd) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_UA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = chg->default_icl_ua;
			break;
		default:
			current_ua = 0;
			break;
		}
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		current_ua = TYPEC_MEDIUM_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		current_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_NON_COMPLIANT:
	case POWER_SUPPLY_TYPEC_NONE:
	default:
		current_ua = 0;
		break;
	}

	*total_current_ua = max(current_ua, val.intval);
	return 0;
}

#define IADP_OVERHEAT_UA	500000
int smblib_set_prop_thermal_overheat(struct smb_charger *chg,
						int therm_overheat)
{
	int icl_ua = 0;

	if (chg->thermal_overheat == !!therm_overheat)
		return 0;

	/* Configure ICL to 500mA in case system health is Overheat */
	if (therm_overheat)
		icl_ua = IADP_OVERHEAT_UA;

	if (!chg->cp_disable_votable)
		chg->cp_disable_votable = find_votable("CP_DISABLE");

	if (chg->cp_disable_votable) {
		vote(chg->cp_disable_votable, OVERHEAT_LIMIT_VOTER,
							therm_overheat, 0);
		vote(chg->usb_icl_votable, OVERHEAT_LIMIT_VOTER,
							therm_overheat, icl_ua);
	}

	chg->thermal_overheat = !!therm_overheat;
	return 0;
}

/**********************
 * INTERRUPT HANDLERS *
 **********************/

irqreturn_t default_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	return IRQ_HANDLED;
}

irqreturn_t smb_en_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc, input_present;

	if (!chg->cp_disable_votable) {
		chg->cp_disable_votable = find_votable("CP_DISABLE");
		if (!chg->cp_disable_votable)
			return IRQ_HANDLED;
	}

	if (chg->pd_hard_reset) {
		vote(chg->cp_disable_votable, BOOST_BACK_VOTER, true, 0);
		return IRQ_HANDLED;
	}

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0) {
		pr_err("Couldn't get usb presence status rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (input_present) {
		/*
		 * Add some delay to enable SMB1390 switcher after SMB_EN
		 * pin goes high
		 */
		usleep_range(1000, 1100);
		vote(chg->cp_disable_votable, BOOST_BACK_VOTER, false, 0);
	}

	return IRQ_HANDLED;
}

irqreturn_t sdam_sts_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	mutex_lock(&chg->irq_status_lock);
	chg->irq_status |= PULSE_SKIP_IRQ_BIT;
	mutex_unlock(&chg->irq_status_lock);

	power_supply_changed(chg->usb_main_psy);

	return IRQ_HANDLED;
}

#define CHG_TERM_WA_ENTRY_DELAY_MS		300000		/* 5 min */
#define CHG_TERM_WA_EXIT_DELAY_MS		60000		/* 1 min */
static void smblib_eval_chg_termination(struct smb_charger *chg, u8 batt_status)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_REAL_CAPACITY, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SOC value, rc=%d\n", rc);
		return;
	}

	/*
	 * Post charge termination, switch to BSM mode triggers the risk of
	 * over charging as BATFET opening may take some time post the necessity
	 * of staying in supplemental mode, leading to unintended charging of
	 * battery. Trigger the charge termination WA once charging is completed
	 * to prevent overcharing.
	 */
	if ((batt_status == TERMINATE_CHARGE) && (pval.intval == 100)) {
		chg->cc_soc_ref = 0;
		chg->last_cc_soc = 0;
		chg->term_vbat_uv = 0;
		alarm_start_relative(&chg->chg_termination_alarm,
				ms_to_ktime(CHG_TERM_WA_ENTRY_DELAY_MS));
	} else if (pval.intval < 100) {
		/*
		 * Reset CC_SOC reference value for charge termination WA once
		 * we exit the TERMINATE_CHARGE state and soc drops below 100%
		 */
		chg->cc_soc_ref = 0;
		chg->last_cc_soc = 0;
		chg->term_vbat_uv = 0;
	}
}

irqreturn_t chg_state_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
				rc);
		return IRQ_HANDLED;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (chg->wa_flags & CHG_TERMINATION_WA)
		smblib_eval_chg_termination(chg, stat);

	/*vivo add*/
	if (stat == TERMINATE_CHARGE && chg->recharge_flag) {
		smblib_dbg(chg, PR_INTERRUPT, "recharge done, reset recharge_flag to false.\n");
		chg->recharge_flag = false;
	}
	/*vivo end*/
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t batt_temp_changed_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->jeita_configured != JEITA_CFG_COMPLETE)
		return IRQ_HANDLED;

	rc = smblib_soft_jeita_arb_wa(chg);
	if (rc < 0) {
		smblib_err(chg, "Couldn't fix soft jeita arb rc=%d\n",
				rc);
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

irqreturn_t batt_psy_changed_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

#define AICL_STEP_MV		200
#define MAX_AICL_THRESHOLD_MV	4800
irqreturn_t usbin_uv_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata;
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);
	int rc;
	u8 stat = 0, max_pulses = 0;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if ((chg->wa_flags & WEAK_ADAPTER_WA)
			&& is_storming(&irq_data->storm_data)) {

		if (chg->aicl_max_reached) {
			smblib_dbg(chg, PR_MISC,
					"USBIN_UV storm at max AICL threshold\n");
			return IRQ_HANDLED;
		}

		smblib_dbg(chg, PR_MISC, "USBIN_UV storm at threshold %d\n",
				chg->aicl_5v_threshold_mv);

		/* suspend USBIN before updating AICL threshold */
		vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER, true, 0);

		/* delay for VASHDN deglitch */
		msleep(20);

		if (chg->aicl_5v_threshold_mv > MAX_AICL_THRESHOLD_MV) {
			/* reached max AICL threshold */
			chg->aicl_max_reached = true;
			goto unsuspend_input;
		}

		/* Increase AICL threshold by 200mV */
		rc = smblib_set_charge_param(chg, &chg->param.aicl_5v_threshold,
				chg->aicl_5v_threshold_mv + AICL_STEP_MV);
		if (rc < 0)
			dev_err(chg->dev,
				"Error in setting AICL threshold rc=%d\n", rc);
		else
			chg->aicl_5v_threshold_mv += AICL_STEP_MV;

		rc = smblib_set_charge_param(chg,
				&chg->param.aicl_cont_threshold,
				chg->aicl_cont_threshold_mv + AICL_STEP_MV);
		if (rc < 0)
			dev_err(chg->dev,
				"Error in setting AICL threshold rc=%d\n", rc);
		else
			chg->aicl_cont_threshold_mv += AICL_STEP_MV;

unsuspend_input:
		/* Force torch in boost mode to ensure it works with low ICL */
		if (chg->chg_param.smb_version == PMI632_SUBTYPE)
			schgm_flash_torch_priority(chg, TORCH_BOOST_MODE);

		if (chg->aicl_max_reached) {
			smblib_dbg(chg, PR_MISC,
				"Reached max AICL threshold resctricting ICL to 100mA\n");
			vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER,
					true, USBIN_100MA);
			smblib_run_aicl(chg, RESTART_AICL);
		} else {
			smblib_run_aicl(chg, RESTART_AICL);
			vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER,
					false, 0);
		}

		wdata = &chg->irq_info[USBIN_UV_IRQ].irq_data->storm_data;
		reset_storm_count(wdata);
	}

	if (!chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data)
		return IRQ_HANDLED;

	wdata = &chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data->storm_data;
	reset_storm_count(wdata);

	/* Workaround for non-QC2.0-compliant chargers follows */
	if (!chg->qc2_unsupported_voltage &&
			apsd->pst == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't read CHANGE_STATUS_REG rc=%d\n", rc);

		if (stat & QC_5V_BIT)
			return IRQ_HANDLED;

		rc = smblib_read(chg, HVDCP_PULSE_COUNT_MAX_REG, &max_pulses);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't read QC2 max pulses rc=%d\n", rc);

		chg->qc2_max_pulses = (max_pulses &
				HVDCP_PULSE_COUNT_MAX_QC2_MASK);

		if (stat & QC_12V_BIT) {
			chg->qc2_unsupported_voltage = QC2_NON_COMPLIANT_12V;
			rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
					HVDCP_PULSE_COUNT_MAX_QC2_MASK,
					HVDCP_PULSE_COUNT_MAX_QC2_9V);
			if (rc < 0)
				smblib_err(chg, "Couldn't force max pulses to 9V rc=%d\n",
						rc);

		} else if (stat & QC_9V_BIT) {
			chg->qc2_unsupported_voltage = QC2_NON_COMPLIANT_9V;
			rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
					HVDCP_PULSE_COUNT_MAX_QC2_MASK,
					HVDCP_PULSE_COUNT_MAX_QC2_5V);
			if (rc < 0)
				smblib_err(chg, "Couldn't force max pulses to 5V rc=%d\n",
						rc);

		}

		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
				SUSPEND_ON_COLLAPSE_USBIN_BIT,
				0);
		if (rc < 0)
			smblib_err(chg, "Couldn't turn off SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
					rc);

		smblib_rerun_apsd(chg);
	}

	return IRQ_HANDLED;
}

/*#define USB_WEAK_INPUT_UA	1400000*//*vivo remove to vivo-smb-lib.h*/
#define ICL_CHANGE_DELAY_MS	1000
irqreturn_t icl_change_irq_handler(int irq, void *data)
{
	u8 stat;
	int rc, settled_ua, delay = ICL_CHANGE_DELAY_MS;
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->mode == PARALLEL_MASTER) {
		/*
		 * Ignore if change in ICL is due to DIE temp mitigation.
		 * This is to prevent any further ICL split.
		 */
		if (chg->hw_die_temp_mitigation) {
			rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &stat);
			if (rc < 0) {
				smblib_err(chg,
					"Couldn't read DIE_TEMP rc=%d\n", rc);
				return IRQ_HANDLED;
			}
			if (stat & (DIE_TEMP_UB_BIT | DIE_TEMP_LB_BIT)) {
				smblib_dbg(chg, PR_PARALLEL,
					"skip ICL change DIE_TEMP %x\n", stat);
				return IRQ_HANDLED;
			}
		}

		rc = smblib_read(chg, AICL_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n",
					rc);
			return IRQ_HANDLED;
		}

		rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
					&settled_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
			return IRQ_HANDLED;
		}

		/* If AICL settled then schedule work now */
		if (settled_ua == get_effective_result(chg->usb_icl_votable))
			delay = 0;

		cancel_delayed_work_sync(&chg->icl_change_work);
		schedule_delayed_work(&chg->icl_change_work,
						msecs_to_jiffies(delay));
	}

	return IRQ_HANDLED;
}

static int smblib_role_switch_failure(struct smb_charger *chg)
{
	int rc = 0;
	union power_supply_propval pval = {0, };

	if (!chg->use_extcon)
		return 0;

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb presence status rc=%d\n",
				rc);
		return rc;
	}

	/*
	 * When role switch fails notify the
	 * current charger state to usb driver.
	 */
	if (pval.intval) {
		smblib_dbg(chg, PR_MISC, " Role reversal failed, notifying device mode to usb driver.\n");
		smblib_notify_device_mode(chg, true);
	}

	return rc;
}

static int typec_partner_register(struct smb_charger *chg)
{
	int typec_mode, rc = 0;

	mutex_lock(&chg->typec_lock);

	if (!chg->typec_port || chg->pr_swap_in_progress)
		goto unlock;

	if (!chg->typec_partner) {
		if (chg->sink_src_mode == AUDIO_ACCESS_MODE)
			chg->typec_partner_desc.accessory =
					TYPEC_ACCESSORY_AUDIO;
		else
			chg->typec_partner_desc.accessory =
					TYPEC_ACCESSORY_NONE;

		chg->typec_partner = typec_register_partner(chg->typec_port,
				&chg->typec_partner_desc);
		if (IS_ERR(chg->typec_partner)) {
			rc = PTR_ERR(chg->typec_partner);
			pr_err("failed to register typec_partner rc=%d\n", rc);
			goto unlock;
		}
	}

	typec_mode = smblib_get_prop_typec_mode(chg);

	if (typec_mode >= POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
			|| typec_mode == POWER_SUPPLY_TYPEC_NONE) {

		if (chg->typec_role_swap_failed) {
			rc = smblib_role_switch_failure(chg);
			if (rc < 0)
				smblib_err(chg, "Failed to role switch rc=%d\n",
					rc);
			chg->typec_role_swap_failed = false;
		}

		typec_set_data_role(chg->typec_port, TYPEC_DEVICE);
		typec_set_pwr_role(chg->typec_port, TYPEC_SINK);
	} else {
		typec_set_data_role(chg->typec_port, TYPEC_HOST);
		typec_set_pwr_role(chg->typec_port, TYPEC_SOURCE);
	}

unlock:
	mutex_unlock(&chg->typec_lock);
	return rc;
}

static void typec_partner_unregister(struct smb_charger *chg)
{
	mutex_lock(&chg->typec_lock);

	if (!chg->typec_port)
		goto unlock;

	if (chg->typec_partner && !chg->pr_swap_in_progress) {
		smblib_dbg(chg, PR_MISC, "Un-registering typeC partner\n");
		typec_unregister_partner(chg->typec_partner);
		chg->typec_partner = NULL;
	}

unlock:
	mutex_unlock(&chg->typec_lock);
}

static void smblib_micro_usb_plugin(struct smb_charger *chg, bool vbus_rising)
{
	int rc = 0;

	if (!vbus_rising) {
		smblib_update_usb_type(chg);
		smblib_notify_device_mode(chg, false);
		smblib_uusb_removal(chg);
		typec_partner_unregister(chg);
	} else {
		rc = typec_partner_register(chg);
		if (rc < 0)
			smblib_err(chg, "Couldn't register partner rc =%d\n",
					rc);
	}
}

void smblib_usb_plugin_hard_reset_locked(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	bool vbus_rising;
	struct smb_irq_data *data;
	struct storm_watch *wdata;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (vbus_rising) {
		/* Remove FCC_STEPPER 1.5A init vote to allow FCC ramp up */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER, false, 0);
		/*vivo add for usb plugin*/
		smblib_usb_plugin_init(chg);
		/*vivo end*/
	} else {
		if (chg->wa_flags & BOOST_BACK_WA) {
			data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				update_storm_count(wdata,
						WEAK_CHG_STORM_COUNT);
				vote(chg->usb_icl_votable, BOOST_BACK_VOTER,
						false, 0);
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
						false, 0);
			}
		}
		/*vivo add for usb remove*/
		smblib_usb_plugin_deinit(chg);
		/*vivo end*/
		/* Force 1500mA FCC on USB removal if fcc stepper is enabled */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER,
							true, 1500000);
	}
	/* vivo add */
	chg->usb_present = vbus_rising;
	chg->usb_temporary_present = vbus_rising;
	/* vivo end */
	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");
}

#define PL_DELAY_MS	30000
void smblib_usb_plugin_locked(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	bool vbus_rising;
	struct smb_irq_data *data;
	struct storm_watch *wdata;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	smblib_set_opt_switcher_freq(chg, vbus_rising ? chg->chg_freq.freq_5V :
						chg->chg_freq.freq_removal);

	if (vbus_rising) {
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		vote(chg->awake_votable, DETACH_DETECT_VOTER, false, 0);
		if (chg->audio_attached == false) {
		rc = smblib_request_dpdm(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't to enable DPDM rc=%d\n", rc);
		}
		/* Enable SW Thermal regulation */
		rc = smblib_set_sw_thermal_regulation(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't start SW thermal regulation WA, rc=%d\n",
				rc);

		/* Remove FCC_STEPPER 1.5A init vote to allow FCC ramp up */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER, false, 0);

		/* Schedule work to enable parallel charger */
		/*vote(chg->awake_votable, PL_DELAY_VOTER, true, 0);
		schedule_delayed_work(&chg->pl_enable_work,
					msecs_to_jiffies(PL_DELAY_MS));*//*vivo  delete*/
		/*vivo add for usb plugin*/
		smblib_usb_plugin_init(chg);
		/*vivo end*/
	} else {
		/* Disable SW Thermal Regulation */
		rc = smblib_set_sw_thermal_regulation(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't stop SW thermal regulation WA, rc=%d\n",
				rc);

		if (chg->wa_flags & BOOST_BACK_WA) {
			data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				update_storm_count(wdata,
						WEAK_CHG_STORM_COUNT);
				vote(chg->usb_icl_votable, BOOST_BACK_VOTER,
						false, 0);
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
						false, 0);
			}
		}

		/* Force 1500mA FCC on removal if fcc stepper is enabled */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER,
							true, 1500000);

		if (chg->wa_flags & WEAK_ADAPTER_WA) {
			chg->aicl_5v_threshold_mv =
					chg->default_aicl_5v_threshold_mv;
			chg->aicl_cont_threshold_mv =
					chg->default_aicl_cont_threshold_mv;

			smblib_set_charge_param(chg,
					&chg->param.aicl_5v_threshold,
					chg->aicl_5v_threshold_mv);
			smblib_set_charge_param(chg,
					&chg->param.aicl_cont_threshold,
					chg->aicl_cont_threshold_mv);
			chg->aicl_max_reached = false;

			if (chg->chg_param.smb_version == PMI632_SUBTYPE)
				schgm_flash_torch_priority(chg,
						TORCH_BUCK_MODE);

			data = chg->irq_info[USBIN_UV_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				reset_storm_count(wdata);
			}
			vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER,
					false, 0);
		}

		rc = smblib_request_dpdm(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable DPDM rc=%d\n", rc);

		smblib_update_usb_type(chg);
		/*vivo add for usb remove*/
		smblib_usb_plugin_deinit(chg);
		/*vivo end*/
	}

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		smblib_micro_usb_plugin(chg, vbus_rising);

	vote(chg->temp_change_irq_disable_votable, DEFAULT_VOTER,
						!vbus_rising, 0);

	/* vivo add */
	chg->usb_present = vbus_rising;
	chg->usb_temporary_present = vbus_rising;
	/* vivo end */
	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");
}

irqreturn_t usb_plugin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	if (chg->pd_hard_reset)
		smblib_usb_plugin_hard_reset_locked(chg);
	else
		smblib_usb_plugin_locked(chg);

	return IRQ_HANDLED;
}

static void smblib_handle_slow_plugin_timeout(struct smb_charger *chg,
					      bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: slow-plugin-timeout %s\n",
		   rising ? "rising" : "falling");
}

static void smblib_handle_sdp_enumeration_done(struct smb_charger *chg,
					       bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: sdp-enumeration-done %s\n",
		   rising ? "rising" : "falling");
}

#define APSD_EXTENDED_TIMEOUT_MS	400
/* triggers when HVDCP 3.0 authentication has finished */
static void smblib_handle_hvdcp_3p0_auth_done(struct smb_charger *chg,
					      bool rising)
{
	const struct apsd_result *apsd_result;
	int rc;

	if (!rising)
		return;

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, true, 0);

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);

	if (apsd_result->bit & QC_3P0_BIT) {
		/* for QC3, switch to CP if present */
		if (chg->sec_cp_present) {
			rc = smblib_select_sec_charger(chg,
				POWER_SUPPLY_CHARGER_SEC_CP,
				POWER_SUPPLY_CP_HVDCP3, false);
			if (rc < 0)
				dev_err(chg->dev,
				"Couldn't enable secondary chargers  rc=%d\n",
					rc);
		}

		/* QC3.5 detection timeout */
		if (!chg->apsd_ext_timeout &&
				!timer_pending(&chg->apsd_timer)) {
			smblib_dbg(chg, PR_MISC,
				"APSD Extented timer started at %lld\n",
				jiffies_to_msecs(jiffies));

			mod_timer(&chg->apsd_timer,
				msecs_to_jiffies(APSD_EXTENDED_TIMEOUT_MS)
				+ jiffies);
		}
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
}

static void smblib_handle_hvdcp_check_timeout(struct smb_charger *chg,
					      bool rising, bool qc_charger)
{
	u32 hvdcp_ua = 0;

	if (rising) {

		if (qc_charger) {
			hvdcp_ua = (chg->real_charger_type ==
					POWER_SUPPLY_TYPE_USB_HVDCP) ?
					chg->chg_param.hvdcp2_max_icl_ua :
					HVDCP_CURRENT_UA;

			/* enable HDC and ICL irq for QC2/3 charger */
			vote(chg->limited_irq_disable_votable,
					CHARGER_TYPE_VOTER, false, 0);
			vote(chg->hdc_irq_disable_votable,
					CHARGER_TYPE_VOTER, false, 0);
			vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					hvdcp_ua);
		} else {
			/* A plain DCP, enforce DCP ICL if specified */
			vote(chg->usb_icl_votable, DCP_VOTER,
				chg->dcp_icl_ua != -EINVAL, chg->dcp_icl_ua);
		}
	}

	/*vivo add start*/
	/*vivo add end*/
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s %s\n", __func__,
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP is detected */
static void smblib_handle_hvdcp_detect_done(struct smb_charger *chg,
					    bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-detect-done %s\n",
		   rising ? "rising" : "falling");
}

static void update_sw_icl_max(struct smb_charger *chg, int pst)
{
	int typec_mode;
	int rp_ua;

	/* while PD is active it should have complete ICL control */
	if (chg->pd_active)
		return;

	if (chg->typec_mode == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 500000);
		return;
	}

	/*
	 * HVDCP 2/3, handled separately
	 */
	if (pst == POWER_SUPPLY_TYPE_USB_HVDCP
			|| pst == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		return;

	/* TypeC rp med or high, use rp value */
	typec_mode = smblib_get_prop_typec_mode(chg);
	if (typec_rp_med_high(chg, typec_mode)) {
		rp_ua = get_rp_based_dcp_current(chg, typec_mode);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, rp_ua);
		return;
	}

	/*vivo add*/
	/*no need qcom defuat setting and use vivo, return*/
	smblib_dbg(chg, PR_MISC, "no need qcom defuat setting, return.\n");
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
	return;
	/*vivo end*/

	/* rp-std or legacy, USB BC 1.2 */
	switch (pst) {
	case POWER_SUPPLY_TYPE_USB:
		/*
		 * USB_PSY will vote to increase the current to 500/900mA once
		 * enumeration is done.
		 */
		if (!is_client_vote_enabled(chg->usb_icl_votable,
						USB_PSY_VOTER)) {
			/* if flash is active force 500mA */
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
					is_flash_active(chg) ?
					SDP_CURRENT_UA : SDP_100_MA);
		}
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					CDP_CURRENT_UA);
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		rp_ua = get_rp_based_dcp_current(chg, typec_mode);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, rp_ua);
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		/*
		 * limit ICL to 100mA, the USB driver will enumerate to check
		 * if this is a SDP and appropriately set the current
		 */
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					SDP_100_MA);
		break;
	case POWER_SUPPLY_TYPE_UNKNOWN:
	default:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					SDP_100_MA);
		break;
	}
}

static void smblib_handle_apsd_done(struct smb_charger *chg, bool rising)
{
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	apsd_result = smblib_update_usb_type(chg);

	update_sw_icl_max(chg, apsd_result->pst);

	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		if (chg->use_extcon)
			smblib_notify_device_mode(chg, true);
		break;
	case OCP_CHARGER_BIT:
	case DCP_CHARGER_BIT:
		break;
	default:
		break;
	}
	/*vivo add start */
	if (!chg->usb_present)
		chg->apsd_done_before_usb_present = true;
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: apsd-done rising; %s detected, usb_present=%d(%d).\n",
		   apsd_result->name, chg->usb_present, chg->apsd_done_before_usb_present);
	/* vivo add end */
}

irqreturn_t usb_source_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;
	/* vivo add start */
	bool is_present;
	int current_limit_ua;
	int start_hvdcp_work_delay = 10000;
	int sdp_icl_ua = 0;
	/* vivo add end */
	/* PD session is ongoing, ignore BC1.2 and QC detection */
	if (chg->pd_active)
		return IRQ_HANDLED;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	/* vivo add start */
	is_present = (stat & APSD_DTC_STATUS_DONE_BIT);
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x,is_present=%d\n", stat,is_present);
	/* vivo add end */
	/* vivo delete this rerun_apsd */
	if (0 && (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		&& (stat & APSD_DTC_STATUS_DONE_BIT)
		&& !chg->uusb_apsd_rerun_done) {
		/*
		 * Force re-run APSD to handle slow insertion related
		 * charger-mis-detection.
		 */
		chg->uusb_apsd_rerun_done = true;
		smblib_rerun_apsd_if_required(chg);
		return IRQ_HANDLED;
	}

	smblib_handle_apsd_done(chg,
		(bool)(stat & APSD_DTC_STATUS_DONE_BIT));

	smblib_handle_hvdcp_detect_done(chg,
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_check_timeout(chg,
		(bool)(stat & HVDCP_CHECK_TIMEOUT_BIT),
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	smblib_hvdcp_adaptive_voltage_change(chg);
	/* vivo add start */
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, false, 0);

	if(is_present){

		if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB){
			smblib_dbg(chg, PR_MISC, "usb type \n");
			get_sdp_current(chg, &sdp_icl_ua);
			current_limit_ua = (sdp_icl_ua > DEFAULT_SDP_UA) ? sdp_icl_ua : DEFAULT_SDP_UA;
			if (chg->usbsel_gpio) {
				gpio_direction_output(chg->usbsel_gpio,0);
				smblib_dbg(chg, PR_MISC, "usbsel=%d\n",gpio_get_value(chg->usbsel_gpio));
			}
			smblib_start_apsd_rerun_work(chg);
		}else if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_CDP){
			smblib_dbg(chg, PR_MISC, "cdp type \n");
			current_limit_ua = DEFAULT_CDP_UA;
			if (chg->usbsel_gpio) {
				gpio_direction_output(chg->usbsel_gpio,0);
				smblib_dbg(chg, PR_MISC, "usbsel=%d\n",gpio_get_value(chg->usbsel_gpio));
			}
			smblib_start_apsd_rerun_work(chg);
		}else if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT){
			smblib_dbg(chg, PR_MISC, "float type \n");
			/* temporarily change to 500ma */
			current_limit_ua = DEFAULT_SDP_UA;
			smblib_start_apsd_rerun_work(chg);
		}else if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP &&
					!strncmp(chg->usb_type_name, "OCP", 3)){
			smblib_dbg(chg, PR_MISC, "other type \n");
			current_limit_ua = DEFAULT_OTHER_UA;
		} else if (chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN) {
			smblib_dbg(chg, PR_MISC, "unkown type \n");
			current_limit_ua = DEFAULT_SDP_UA;
			chg->bigdata_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;

			smblib_start_apsd_rerun_work(chg);
		} else{
			if(chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP ||
					chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3){
				/* hvdcp-2.0 or hvdcp-3.0 */
				smblib_dbg(chg, PR_MISC, "%s type\n",chg->usb_type_name);
				current_limit_ua = chg->total_input_current_max_ua;
				if(chg->detect_icl_ibat_ua == SMBCHG_DETECT_ICL_IBAT_UA){
					//vote(chg->fcc_votable, USER_DEFAULT_IBAT_VOTER, true, 1000000);
					vote(chg->usb_icl_votable, USER_CTRL_PL_VOTER, true, 1400000);
					smblib_dbg(chg, PR_MISC, "first plugin to hvdcp,set input:1.4A.\n");
				}
				/* force_9v_hvdcp(chg); */
				start_hvdcp_work_delay = HVDCP_CHARGE_MONITOR_MS;
			}else{
				smblib_dbg(chg, PR_MISC, "dcp type\n");
				current_limit_ua = chg->total_input_current_max_ua;
				start_hvdcp_work_delay = HVDCP_CHARGE_MONITOR_MS+5000;
			}
			if (chg->usb_present) {
				cancel_delayed_work(&chg->start_hvdcp_charge_monitor_work);
				schedule_delayed_work(&chg->start_hvdcp_charge_monitor_work,
							msecs_to_jiffies(start_hvdcp_work_delay));
			}
			/*  usb conn protect function */
			if (chg->usb_conn_pro_enable) {
				schedule_delayed_work(&chg->start_usb_conn_pro_monitor_work,
						msecs_to_jiffies(8000));
			}
		}
		/*
		if (chg->is_audio_adapter == true) {
			current_limit_ua = DEFAULT_SDP_UA;
		}
		*/
		smblib_dbg(chg, PR_MISC, "Type %d: setting uA = %d, TypeC_mode=%s.\n",
			chg->real_charger_type, current_limit_ua, smblib_typec_mode_name[chg->typec_mode]);
		rc = vote(chg->usb_icl_votable, USER_DEFAULT_INPUT_VOTER, true,
					current_limit_ua);

		//chg->hvdcp_type = smblib_get_hvdcp_type(chg);
		//power_supply_changed(chg->batt_psy);
		//smblib_dbg(chg, PR_INTERRUPT, "### usbin type = %d\n", chg->hvdcp_type);
	}else{

	}
	/* vivo end */
	power_supply_changed(chg->usb_psy);

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_INTERRUPT, "APSD_STATUS = 0x%02x\n", stat);

	return IRQ_HANDLED;
}

enum alarmtimer_restart smblib_lpd_recheck_timer(struct alarm *alarm,
						ktime_t time)
{
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
							lpd_recheck_timer);
	int rc;

	if (chg->lpd_reason == LPD_MOISTURE_DETECTED) {
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
			return ALARMTIMER_NORESTART;
		}
		chg->moisture_present = false;
		power_supply_changed(chg->usb_psy);
	} else {
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT,
					TYPEC_WATER_DETECTION_INT_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set TYPE_C_INTERRUPT_EN_CFG_2_REG rc=%d\n",
					rc);
			return ALARMTIMER_NORESTART;
		}
	}

	chg->lpd_stage = LPD_STAGE_NONE;
	chg->lpd_reason = LPD_NONE;

	return ALARMTIMER_NORESTART;
}

#define RSBU_K_300K_UV	3000000
static bool smblib_src_lpd(struct smb_charger *chg)
{
	union power_supply_propval pval;
	bool lpd_flag = false;
	u8 stat;
	int rc;

	if (chg->lpd_disabled)
		return false;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return false;
	}

	switch (stat & DETECTED_SNK_TYPE_MASK) {
	case SRC_DEBUG_ACCESS_BIT:
		if (smblib_rsbux_low(chg, RSBU_K_300K_UV))
			lpd_flag = true;
		break;
	case SRC_RD_RA_VCONN_BIT:
	case SRC_RD_OPEN_BIT:
	case AUDIO_ACCESS_RA_RA_BIT:
	default:
		break;
	}

	if (lpd_flag) {
		chg->lpd_stage = LPD_STAGE_COMMIT;
		pval.intval = POWER_SUPPLY_TYPEC_PR_SINK;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0)
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
		chg->lpd_reason = LPD_MOISTURE_DETECTED;
		chg->moisture_present =  true;
		vote(chg->usb_icl_votable, LPD_VOTER, true, 0);
		alarm_start_relative(&chg->lpd_recheck_timer,
						ms_to_ktime(60000));
		power_supply_changed(chg->usb_psy);
	} else {
		chg->lpd_reason = LPD_NONE;
		chg->typec_mode = smblib_get_prop_typec_mode(chg);
	}

	return lpd_flag;
}

static void typec_src_fault_condition_cfg(struct smb_charger *chg, bool src)
{
	int rc;
	u8 mask = USBIN_MID_COMP_FAULT_EN_BIT | USBIN_COLLAPSE_FAULT_EN_BIT;

	rc = smblib_masked_write(chg, OTG_FAULT_CONDITION_CFG_REG, mask,
					src ? 0 : mask);
	if (rc < 0)
		smblib_err(chg, "Couldn't write OTG_FAULT_CONDITION_CFG_REG rc=%d\n",
			rc);
}

static void typec_sink_insertion(struct smb_charger *chg)
{
	int rc;
	smblib_dbg(chg, PR_INTERRUPT, "typec log\n"); /* vivo add */
	typec_src_fault_condition_cfg(chg, true);
	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
					chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);

	if (chg->use_extcon) {
		smblib_notify_usb_host(chg, true);
		chg->otg_present = true;
	}

	if (!chg->pr_swap_in_progress)
		chg->ok_to_pd = (!(chg->pd_disabled) || chg->early_usb_attach)
					&& !chg->pd_not_supported;
}

static void typec_src_insertion(struct smb_charger *chg)
{
	int rc = 0;
	u8 stat;

	smblib_dbg(chg, PR_INTERRUPT, "typec log\n");	/* vivo add */
	if (chg->pr_swap_in_progress) {
		/*vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);*//*vivo delete*/
		return;
	}

	rc = smblib_read(chg, LEGACY_CABLE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
			rc);
		return;
	}

	chg->typec_legacy = stat & TYPEC_LEGACY_CABLE_STATUS_BIT;
	chg->ok_to_pd = (!(chg->typec_legacy || chg->pd_disabled)
			|| chg->early_usb_attach) && !chg->pd_not_supported;

	/* allow apsd proceed to detect QC2/3 */
	if (!chg->ok_to_pd)
		smblib_hvdcp_detect_try_enable(chg, true);
}

static void typec_ra_ra_insertion(struct smb_charger *chg)
{
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 500000);
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	chg->ok_to_pd = false;
	smblib_hvdcp_detect_enable(chg, true);
}

static const char * const dr_mode_text[] = {
	"ufp", "dfp", "none"
};

static int smblib_force_dr_mode(struct smb_charger *chg, int mode)
{
	int rc = 0;

	switch (mode) {
	case TYPEC_PORT_SNK:
		rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
			TYPEC_POWER_ROLE_CMD_MASK, EN_SNK_ONLY_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable snk, rc=%d\n", rc);
			return rc;
		}
		break;
	case TYPEC_PORT_SRC:
		rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
			TYPEC_POWER_ROLE_CMD_MASK, EN_SRC_ONLY_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable src, rc=%d\n", rc);
			return rc;
		}
		break;
	case TYPEC_PORT_DRP:
		rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
			TYPEC_POWER_ROLE_CMD_MASK, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable DRP, rc=%d\n", rc);
			return rc;
		}
		break;
	default:
		smblib_err(chg, "Power role %d not supported\n", mode);
		return -EINVAL;
	}

	chg->dr_mode = mode;

	return rc;
}

int smblib_typec_port_type_set(const struct typec_capability *cap,
					enum typec_port_type type)
{
	struct smb_charger *chg = container_of(cap,
					struct smb_charger, typec_caps);
	int rc = 0;

	mutex_lock(&chg->typec_lock);

	if ((chg->pr_swap_in_progress) || (type == TYPEC_PORT_DRP)) {
		smblib_dbg(chg, PR_MISC, "Ignoring port type request type = %d swap_in_progress = %d\n",
				type, chg->pr_swap_in_progress);
		goto unlock;
	}

	chg->pr_swap_in_progress = true;

	rc = smblib_force_dr_mode(chg, type);
	if (rc < 0) {
		chg->pr_swap_in_progress = false;
		smblib_err(chg, "Failed to force mode, rc=%d\n", rc);
		goto unlock;
	}

	smblib_dbg(chg, PR_MISC, "Requested role %s\n",
				type ? "SINK" : "SOURCE");

	/*
	 * As per the hardware requirements,
	 * schedule the work with required delay.
	 */
	if (!(delayed_work_pending(&chg->role_reversal_check))) {
		cancel_delayed_work_sync(&chg->role_reversal_check);
		schedule_delayed_work(&chg->role_reversal_check,
			msecs_to_jiffies(ROLE_REVERSAL_DELAY_MS));
		vote(chg->awake_votable, TYPEC_SWAP_VOTER, true, 0);
	}

unlock:
	mutex_unlock(&chg->typec_lock);
	return rc;
}

static void smblib_typec_role_check_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					role_reversal_check.work);
	int rc = 0;

	mutex_lock(&chg->typec_lock);

	switch (chg->dr_mode) {
	case TYPEC_PORT_SNK:
		if (chg->typec_mode < POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) {
			smblib_dbg(chg, PR_MISC, "Role reversal not latched to UFP in %d msecs. Resetting to DRP mode\n",
						ROLE_REVERSAL_DELAY_MS);
			rc = smblib_force_dr_mode(chg, TYPEC_PORT_DRP);
			if (rc < 0)
				smblib_err(chg, "Failed to set DRP mode, rc=%d\n",
						rc);
		} else {
			chg->power_role = POWER_SUPPLY_TYPEC_PR_SINK;
			typec_set_pwr_role(chg->typec_port, TYPEC_SINK);
			typec_set_data_role(chg->typec_port, TYPEC_DEVICE);
			smblib_dbg(chg, PR_MISC, "Role changed successfully to SINK");
		}
		break;
	case TYPEC_PORT_SRC:
		if (chg->typec_mode >= POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
			|| chg->typec_mode == POWER_SUPPLY_TYPEC_NONE) {
			smblib_dbg(chg, PR_MISC, "Role reversal not latched to DFP in %d msecs. Resetting to DRP mode\n",
						ROLE_REVERSAL_DELAY_MS);
			chg->pr_swap_in_progress = false;
			chg->typec_role_swap_failed = true;
			rc = smblib_force_dr_mode(chg, TYPEC_PORT_DRP);
			if (rc < 0)
				smblib_err(chg, "Failed to set DRP mode, rc=%d\n",
					rc);
		} else {
			chg->power_role = POWER_SUPPLY_TYPEC_PR_SOURCE;
			typec_set_pwr_role(chg->typec_port, TYPEC_SOURCE);
			typec_set_data_role(chg->typec_port, TYPEC_HOST);
			smblib_dbg(chg, PR_MISC, "Role changed successfully to SOURCE");
		}
		break;
	default:
		pr_debug("Already in DRP mode\n");
		break;
	}

	chg->pr_swap_in_progress = false;
	vote(chg->awake_votable, TYPEC_SWAP_VOTER, false, 0);
	mutex_unlock(&chg->typec_lock);
}

static void typec_sink_removal(struct smb_charger *chg)
{
	int rc;
	smblib_dbg(chg, PR_INTERRUPT, "typec log\n"); /* vivo add */
	typec_src_fault_condition_cfg(chg, false);
	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
					chg->chg_freq.freq_removal);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_removal rc=%d\n", rc);

	if (chg->use_extcon) {
		if (chg->otg_present)
			smblib_notify_usb_host(chg, false);
		chg->otg_present = false;
	}
}

static void typec_src_removal(struct smb_charger *chg)
{
	struct smb5 *chip = container_of(chg, struct smb5, chg);/*vivo add*/
	int rc;
	struct smb_irq_data *data;
	struct storm_watch *wdata;
	int sec_charger;
	u8 val[2] = {0};

	smblib_dbg(chg, PR_INTERRUPT, "typec log usb_present=%d\n", chg->usb_present);	/* vivo add */
	sec_charger = chg->sec_pl_present ? POWER_SUPPLY_CHARGER_SEC_PL :
				POWER_SUPPLY_CHARGER_SEC_NONE;

	rc = smblib_select_sec_charger(chg, sec_charger, POWER_SUPPLY_CP_NONE,
					false);
	if (rc < 0)
		dev_err(chg->dev,
			"Couldn't disable secondary charger rc=%d\n", rc);

	chg->qc3p5_detected = false;
	typec_src_fault_condition_cfg(chg, false);
	smblib_hvdcp_detect_try_enable(chg, false);
	smblib_update_usb_type(chg);

	if (chg->wa_flags & BOOST_BACK_WA) {
		data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
		if (data) {
			wdata = &data->storm_data;
			update_storm_count(wdata, WEAK_CHG_STORM_COUNT);
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					false, 0);
		}
	}

	cancel_delayed_work_sync(&chg->pl_enable_work);

	/* reset input current limit voters */
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false,
			0 /*is_flash_active(chg) ? SDP_CURRENT_UA : SDP_100_MA*/);	//vivo change
	vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);
	vote(chg->usb_icl_votable, CTM_VOTER, false, 0);
	vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
	vote(chg->usb_icl_votable, THERMAL_THROTTLE_VOTER, false, 0);
	vote(chg->usb_icl_votable, LPD_VOTER, false, 0);

	/* reset usb irq voters */
	vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
			true, 0);
	vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER, true, 0);
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);

	/* reset parallel voters */
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->pl_disable_votable, PL_FCC_LOW_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);

	/* Remove SW thermal regulation WA votes */
	vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER, false, 0);
	if (chg->cp_disable_votable)
		vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
								false, 0);

	/* reset USBOV votes and cancel work */
	cancel_delayed_work_sync(&chg->usbov_dbc_work);
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	chg->dbc_usbov = false;

	chg->pulse_cnt = 0;
	chg->usb_icl_delta_ua = 0;
	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usbin_forced_max_uv = 0;
	chg->chg_param.forced_main_fcc = 0;

	/* Reset all CC mode votes */
	vote(chg->fcc_main_votable, MAIN_FCC_VOTER, false, 0);
	chg->adapter_cc_mode = 0;
	chg->thermal_overheat = 0;
	vote_override(chg->fcc_votable, CC_MODE_VOTER, false, 0);
	vote_override(chg->usb_icl_votable, CC_MODE_VOTER, false, 0);
	vote(chg->cp_disable_votable, OVERHEAT_LIMIT_VOTER, false, 0);
	vote(chg->usb_icl_votable, OVERHEAT_LIMIT_VOTER, false, 0);

	/* write back the default FLOAT charger configuration */
	rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				(u8)FLOAT_OPTIONS_MASK, chg->float_cfg);
	if (rc < 0)
		smblib_err(chg, "Couldn't write float charger options rc=%d\n",
			rc);

	if (chg->sdam_base) {
		rc = smblib_write(chg,
			chg->sdam_base + SDAM_QC_DET_STATUS_REG, 0);
		if (rc < 0)
			pr_err("Couldn't clear SDAM QC status rc=%d\n", rc);

		rc = smblib_batch_write(chg,
			chg->sdam_base + SDAM_QC_ADC_LSB_REG, val, 2);
		if (rc < 0)
			pr_err("Couldn't clear SDAM ADC status rc=%d\n", rc);
	}

	if (!chg->pr_swap_in_progress) {
		rc = smblib_usb_pd_adapter_allowance_override(chg, FORCE_NULL);
		if (rc < 0)
			smblib_err(chg, "Couldn't set FORCE_NULL rc=%d\n", rc);

		rc = smblib_set_charge_param(chg,
				&chg->param.aicl_cont_threshold,
				chg->default_aicl_cont_threshold_mv);
		if (rc < 0)
			smblib_err(chg, "Couldn't restore aicl_cont_threshold, rc=%d",
					rc);
	}
	/*
	 * if non-compliant charger caused UV, restore original max pulses
	 * and turn SUSPEND_ON_COLLAPSE_USBIN_BIT back on.
	 */
	if (chg->qc2_unsupported_voltage) {
		rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
				HVDCP_PULSE_COUNT_MAX_QC2_MASK,
				chg->qc2_max_pulses);
		if (rc < 0)
			smblib_err(chg, "Couldn't restore max pulses rc=%d\n",
					rc);

		if (!chip->dt.disable_suspend_on_collapse) {/*vivo add condition*/
			rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
					SUSPEND_ON_COLLAPSE_USBIN_BIT,
					SUSPEND_ON_COLLAPSE_USBIN_BIT);
			if (rc < 0)
				smblib_err(chg, "Couldn't turn on SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
						rc);
		}

		chg->qc2_unsupported_voltage = QC2_COMPLIANT;
	}

	if (chg->use_extcon)
		smblib_notify_device_mode(chg, false);

	chg->typec_legacy = false;

	del_timer_sync(&chg->apsd_timer);
	chg->apsd_ext_timeout = false;
}

static void typec_mode_unattached(struct smb_charger *chg)
{
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);	//VIVO debug: disconnect Cable, this way setting 100mA ICL
}

static void smblib_handle_rp_change(struct smb_charger *chg, int typec_mode)
{
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);

	smblib_dbg(chg, PR_INTERRUPT, "typec log\n");	/* vivo add */
	/*
	 * We want the ICL vote @ 100mA for a FLOAT charger
	 * until the detection by the USB stack is complete.
	 * Ignore the Rp changes unless there is a
	 * pre-existing valid vote or FLOAT is configured for
	 * SDP current.
	 */
	if (apsd->pst == POWER_SUPPLY_TYPE_USB_FLOAT) {
		if (get_client_vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER)
					<= USBIN_100MA
			|| (chg->float_cfg & FLOAT_OPTIONS_MASK)
					== FORCE_FLOAT_SDP_CFG_BIT)
			return;
	}

	update_sw_icl_max(chg, apsd->pst);

	smblib_dbg(chg, PR_MISC, "CC change old_mode=%d new_mode=%d\n",
						chg->typec_mode, typec_mode);
}

static void smblib_lpd_launch_ra_open_work(struct smb_charger *chg)
{
	u8 stat;
	int rc;

	if (chg->lpd_disabled)
		return;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
			rc);
		return;
	}

	smblib_err(chg, "typec_lpd: stat=0x%02x(%d), lpd_stage=%d\n",
		stat, (stat & TYPEC_TCCDEBOUNCE_DONE_STATUS_BIT), chg->lpd_stage);
	if (!(stat & TYPEC_TCCDEBOUNCE_DONE_STATUS_BIT)
			&& chg->lpd_stage == LPD_STAGE_NONE) {
		smblib_err(chg, "typec_lpd: lpd_ra_open_work after 300ms\n");
		chg->lpd_stage = LPD_STAGE_FLOAT;
		cancel_delayed_work_sync(&chg->lpd_ra_open_work);
		vote(chg->awake_votable, LPD_VOTER, true, 0);
		schedule_delayed_work(&chg->lpd_ra_open_work,
						msecs_to_jiffies(300));
	}
}

irqreturn_t typec_or_rid_detection_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		if (chg->uusb_moisture_protection_enabled) {
			/*
			 * Adding pm_stay_awake as because pm_relax is called
			 * on exit path from the work routine.
			 */
			pm_stay_awake(chg->dev);
			schedule_work(&chg->moisture_protection_work);
		}

		cancel_delayed_work_sync(&chg->uusb_otg_work);
		/*
		 * Skip OTG enablement if RID interrupt triggers with moisture
		 * protection still enabled.
		 */
		if (!chg->moisture_present) {
			vote(chg->awake_votable, OTG_DELAY_VOTER, true, 0);
			smblib_dbg(chg, PR_INTERRUPT, "Scheduling OTG work\n");
			schedule_delayed_work(&chg->uusb_otg_work,
				msecs_to_jiffies(chg->otg_delay_ms));
		}

		goto out;
	}

	if (chg->pr_swap_in_progress || chg->pd_hard_reset)
		goto out;

	smblib_lpd_launch_ra_open_work(chg);

	if (chg->usb_psy)
		power_supply_changed(chg->usb_psy);

out:
	return IRQ_HANDLED;
}

irqreturn_t typec_state_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int typec_mode;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		smblib_dbg(chg, PR_INTERRUPT,
				"Ignoring for micro USB\n");
		return IRQ_HANDLED;
	}

	typec_mode = smblib_get_prop_typec_mode(chg);
	if (chg->sink_src_mode != UNATTACHED_MODE
			&& (typec_mode != chg->typec_mode))
		smblib_handle_rp_change(chg, typec_mode);

	/*vivo add start*/
	if (typec_mode != POWER_SUPPLY_TYPEC_NONE && chg->cable_with_cc_float) {
			chg->typec_present = 0;
			chg->typec_mode = POWER_SUPPLY_TYPEC_NONE;
			chg->cable_with_cc_float = false;
			chg->cable_with_cc_float_icl_rerun = false;
			chg->cable_with_cc_float_icl_override = false;
	}
	/*vivo add end*/
	chg->typec_mode = typec_mode;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: cc-state-change; Type-C %s detected\n",
				smblib_typec_mode_name[chg->typec_mode]);

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static void smblib_lpd_clear_ra_open_work(struct smb_charger *chg)
{
	if (chg->lpd_disabled)
		return;

	cancel_delayed_work_sync(&chg->lpd_detach_work);
	chg->lpd_stage = LPD_STAGE_FLOAT_CANCEL;
	cancel_delayed_work_sync(&chg->lpd_ra_open_work);
	vote(chg->awake_votable, LPD_VOTER, false, 0);
}

#define TYPEC_DETACH_DETECT_DELAY_MS 2000
irqreturn_t typec_attach_detach_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	bool attached = false;
	int rc;

	/* IRQ not expected to be executed for uUSB, return */
	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		return IRQ_HANDLED;

	rc = smblib_read(chg, TYPE_C_STATE_MACHINE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s, stat=0x%02x, sink_src_mode=%d\n", irq_data->name, stat, chg->sink_src_mode);

	attached = !!(stat & TYPEC_ATTACH_DETACH_STATE_BIT);

	if (attached) {
		smblib_lpd_clear_ra_open_work(chg);

		rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
				rc);
			return IRQ_HANDLED;
		}

		if (smblib_get_prop_dfp_mode(chg) ==
				POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
			chg->sink_src_mode = AUDIO_ACCESS_MODE;
			typec_ra_ra_insertion(chg);
		} else if (stat & SNK_SRC_MODE_BIT) {
			if (smblib_src_lpd(chg))
				return IRQ_HANDLED;
			chg->sink_src_mode = SRC_MODE;
			typec_sink_insertion(chg);
		} else {
			chg->sink_src_mode = SINK_MODE;
			typec_src_insertion(chg);
		}

		rc = typec_partner_register(chg);
		if (rc < 0)
			smblib_err(chg, "failed to register partner rc =%d\n",
					rc);

		/*vivo add */
		if (!chg->usb_det_pin_mode && chg->drp_enable) {
			smblib_dbg(chg, PR_INTERRUPT, "drp enable,attach!\n");
		}
		/*vivo end */
	} else {
		switch (chg->sink_src_mode) {
		case SRC_MODE:
			typec_sink_removal(chg);
			break;
		case SINK_MODE:
		case AUDIO_ACCESS_MODE:
			typec_src_removal(chg);
			break;
		case UNATTACHED_MODE:
		default:
			typec_mode_unattached(chg);
			break;
		}

		if (!chg->pr_swap_in_progress) {
			chg->ok_to_pd = false;
			chg->sink_src_mode = UNATTACHED_MODE;
			chg->early_usb_attach = false;
			smblib_apsd_enable(chg, true);
		}

		/*
		 * Restore DRP mode on type-C cable disconnect if role
		 * swap is not in progress, to ensure forced sink or src
		 * mode configuration is reset properly.
		 */
		mutex_lock(&chg->typec_lock);

		if (chg->typec_port && !chg->pr_swap_in_progress) {

			/*
			 * Schedule the work to differentiate actual removal
			 * of cable and detach interrupt during role swap,
			 * unregister the partner only during actual cable
			 * removal.
			 */
			cancel_delayed_work(&chg->pr_swap_detach_work);
			vote(chg->awake_votable, DETACH_DETECT_VOTER, true, 0);
			schedule_delayed_work(&chg->pr_swap_detach_work,
				msecs_to_jiffies(TYPEC_DETACH_DETECT_DELAY_MS));
			smblib_force_dr_mode(chg, TYPEC_PORT_DRP);
			/*
			 * To handle cable removal during role
			 * swap failure.
			 */
			chg->typec_role_swap_failed = false;
		}

		mutex_unlock(&chg->typec_lock);

		if (chg->lpd_stage == LPD_STAGE_FLOAT_CANCEL)
			schedule_delayed_work(&chg->lpd_detach_work,
					msecs_to_jiffies(1000));

		/*vivo add*/
		if (!chg->usb_det_pin_mode && chg->drp_enable) {
			smblib_dbg(chg, PR_INTERRUPT, "drp enable,detach!\n");
		}
		/*vivo end*/
	}

	rc = smblib_masked_write(chg, USB_CMD_PULLDOWN_REG,
			EN_PULLDOWN_USB_IN_BIT,
			attached ?  0 : EN_PULLDOWN_USB_IN_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't configure pulldown on USB_IN rc=%d\n",
				rc);

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static void dcin_aicl(struct smb_charger *chg)
{
	int rc, icl, icl_save;
	int input_present;
	bool aicl_done = true;

	/*
	 * Hold awake votable to prevent pm_relax being called prior to
	 * completion of this work.
	 */
	vote(chg->awake_votable, DCIN_AICL_VOTER, true, 0);

increment:
	mutex_lock(&chg->dcin_aicl_lock);

	rc = smblib_get_charge_param(chg, &chg->param.dc_icl, &icl);
	if (rc < 0)
		goto err;

	if (icl == chg->wls_icl_ua) {
		/* Upper limit reached; do nothing */
		smblib_dbg(chg, PR_WLS, "hit max ICL: stop\n");

		rc = smblib_is_input_present(chg, &input_present);
		if (rc < 0 || !(input_present & INPUT_PRESENT_DC))
			aicl_done = false;

		goto unlock;
	}

	icl = min(chg->wls_icl_ua, icl + DCIN_ICL_STEP_UA);
	icl_save = icl;

	rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl);
	if (rc < 0)
		goto err;

	mutex_unlock(&chg->dcin_aicl_lock);

	smblib_dbg(chg, PR_WLS, "icl: %d mA\n", (icl / 1000));

	/* Check to see if DC is still present before and after sleep */
	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0 || !(input_present & INPUT_PRESENT_DC)) {
		aicl_done = false;
		goto unvote;
	}

	/*
	 * Wait awhile to check for any DCIN_UVs (the UV handler reduces the
	 * ICL). If the adaptor collapses, the ICL read after waking up will be
	 * lesser, indicating that the AICL process is complete.
	 */
	msleep(500);

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0 || !(input_present & INPUT_PRESENT_DC)) {
		aicl_done = false;
		goto unvote;
	}

	mutex_lock(&chg->dcin_aicl_lock);

	rc = smblib_get_charge_param(chg, &chg->param.dc_icl, &icl);
	if (rc < 0)
		goto err;

	if (icl < icl_save) {
		smblib_dbg(chg, PR_WLS, "done: icl: %d mA\n", (icl / 1000));
		goto unlock;
	}

	mutex_unlock(&chg->dcin_aicl_lock);

	goto increment;

err:
	aicl_done = false;
unlock:
	mutex_unlock(&chg->dcin_aicl_lock);
unvote:
	vote(chg->awake_votable, DCIN_AICL_VOTER, false, 0);
	chg->dcin_aicl_done = aicl_done;
}

static void dcin_aicl_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						dcin_aicl_work);
	dcin_aicl(chg);
}

static enum alarmtimer_restart dcin_aicl_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
					dcin_aicl_alarm);

	smblib_dbg(chg, PR_WLS, "rerunning DCIN AICL\n");

	pm_stay_awake(chg->dev);
	schedule_work(&chg->dcin_aicl_work);

	return ALARMTIMER_NORESTART;
}

static void dcin_icl_decrement(struct smb_charger *chg)
{
	int rc, icl;
	ktime_t now = ktime_get();

	rc = smblib_get_charge_param(chg, &chg->param.dc_icl, &icl);
	if (rc < 0) {
		smblib_err(chg, "reading DCIN ICL failed: %d\n", rc);
		return;
	}

	if (icl == DCIN_ICL_MIN_UA) {
		/* Cannot possibly decrease ICL any further - do nothing */
		smblib_dbg(chg, PR_WLS, "hit min ICL: stop\n");
		return;
	}

	/* Reduce ICL by 100 mA if 3 UVs happen in a row */
	if (ktime_us_delta(now, chg->dcin_uv_last_time) > (200 * 1000)) {
		chg->dcin_uv_count = 0;
	} else if (chg->dcin_uv_count >= 3) {
		icl -= DCIN_ICL_STEP_UA;

		smblib_dbg(chg, PR_WLS, "icl: %d mA\n", (icl / 1000));
		rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl);
		if (rc < 0) {
			smblib_err(chg, "setting DCIN ICL failed: %d\n", rc);
			return;
		}

		chg->dcin_uv_count = 0;
	}

	chg->dcin_uv_last_time = now;
}

irqreturn_t dcin_uv_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	mutex_lock(&chg->dcin_aicl_lock);

	chg->dcin_uv_count++;
	smblib_dbg(chg, (PR_WLS | PR_INTERRUPT), "DCIN UV count: %d\n",
			chg->dcin_uv_count);
	dcin_icl_decrement(chg);

	mutex_unlock(&chg->dcin_aicl_lock);

	return IRQ_HANDLED;
}

irqreturn_t dc_plugin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	union power_supply_propval pval;
	int input_present;
	bool dcin_present, vbus_present;
	int rc, wireless_vout = 0, wls_set = 0;
	int sec_charger;

	rc = smblib_get_prop_vph_voltage_now(chg, &pval);
	if (rc < 0)
		return IRQ_HANDLED;

	/* 2*VPH, with a granularity of 100mV */
	wireless_vout = ((pval.intval * 2) / 100000) * 100000;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return IRQ_HANDLED;

	dcin_present = input_present & INPUT_PRESENT_DC;
	vbus_present = input_present & INPUT_PRESENT_USB;

	if (!chg->cp_ilim_votable)
		chg->cp_ilim_votable = find_votable("CP_ILIM");

	if (dcin_present && !vbus_present) {
		cancel_work_sync(&chg->dcin_aicl_work);

		/* Reset DCIN ICL to 100 mA */
		mutex_lock(&chg->dcin_aicl_lock);
		rc = smblib_set_charge_param(chg, &chg->param.dc_icl,
				DCIN_ICL_MIN_UA);
		mutex_unlock(&chg->dcin_aicl_lock);
		if (rc < 0)
			return IRQ_HANDLED;

		smblib_dbg(chg, (PR_WLS | PR_INTERRUPT), "reset: icl: 100 mA\n");

		/*
		 * Remove USB's CP ILIM vote - inapplicable for wireless
		 * parallel charging.
		 */
		if (chg->cp_ilim_votable)
			vote(chg->cp_ilim_votable, ICL_CHANGE_VOTER, false, 0);

		if (chg->sec_cp_present) {
			/*
			 * If CP output topology is VBATT, limit main charger's
			 * FCC share and let the CPs handle the rest.
			 */
			if (is_cp_topo_vbatt(chg))
				vote(chg->fcc_main_votable,
					WLS_PL_CHARGING_VOTER, true, 800000);

			rc = smblib_get_prop_batt_status(chg, &pval);
			if (rc < 0)
				smblib_err(chg, "Couldn't read batt status rc=%d\n",
						rc);

			wls_set = (pval.intval == POWER_SUPPLY_STATUS_FULL) ?
				MICRO_5V : wireless_vout;

			pval.intval = wls_set;
			rc = smblib_set_prop_voltage_wls_output(chg, &pval);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't set dc voltage to 2*vph  rc=%d\n",
					rc);

			rc = smblib_select_sec_charger(chg,
					POWER_SUPPLY_CHARGER_SEC_CP,
					POWER_SUPPLY_CP_WIRELESS, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't enable secondary chargers  rc=%d\n",
					rc);
		} else {
			/*
			 * If no secondary charger is present, commence
			 * wireless charging at 5 V by default.
			 */
			pval.intval = 5000000;
			rc = smblib_set_prop_voltage_wls_output(chg, &pval);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't set dc voltage to 5 V rc=%d\n",
					rc);
		}

		schedule_work(&chg->dcin_aicl_work);
	} else {
		if (chg->cp_reason == POWER_SUPPLY_CP_WIRELESS) {
			sec_charger = chg->sec_pl_present ?
					POWER_SUPPLY_CHARGER_SEC_PL :
					POWER_SUPPLY_CHARGER_SEC_NONE;
			rc = smblib_select_sec_charger(chg, sec_charger,
					POWER_SUPPLY_CP_NONE, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't disable secondary charger rc=%d\n",
					rc);
		}

		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->fcc_main_votable, WLS_PL_CHARGING_VOTER, false, 0);

		chg->last_wls_vout = 0;
		chg->dcin_aicl_done = false;
		chg->dcin_icl_user_set = false;
	}

	/*
	 * Vote for 1500mA FCC upon WLS detach and remove vote upon attach if
	 * FCC stepper is enabled.
	 */
	if (chg->fcc_stepper_enable && !vbus_present)
		vote(chg->fcc_votable, FCC_STEPPER_VOTER, !dcin_present,
				dcin_present ? 0 : 1500000);

	if (chg->dc_psy)
		power_supply_changed(chg->dc_psy);

	smblib_dbg(chg, (PR_WLS | PR_INTERRUPT), "dcin_present= %d, usbin_present= %d, cp_reason = %d\n",
			dcin_present, vbus_present, chg->cp_reason);

	return IRQ_HANDLED;
}

irqreturn_t high_duty_cycle_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	chg->is_hdc = true;
	/*
	 * Disable usb IRQs after the flag set and re-enable IRQs after
	 * the flag cleared in the delayed work queue, to avoid any IRQ
	 * storming during the delays
	 */
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, true, 0);

	schedule_delayed_work(&chg->clear_hdc_work, msecs_to_jiffies(60));

	return IRQ_HANDLED;
}

static void smblib_bb_removal_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bb_removal_work.work);

	vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
	vote(chg->awake_votable, BOOST_BACK_VOTER, false, 0);
}

#define BOOST_BACK_UNVOTE_DELAY_MS		750
#define BOOST_BACK_STORM_COUNT			3
#define WEAK_CHG_STORM_COUNT			8
irqreturn_t switcher_power_ok_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata = &irq_data->storm_data;
	int rc, usb_icl;
	u8 stat;

	if (!(chg->wa_flags & BOOST_BACK_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	/* skip suspending input if its already suspended by some other voter */
	usb_icl = get_effective_result(chg->usb_icl_votable);
	smblib_err(chg, "stat=%d,usb_icl=%d\n", stat, usb_icl);
	if ((stat & USE_USBIN_BIT) && usb_icl >= 0 && usb_icl <= USBIN_25MA)
		return IRQ_HANDLED;

	if (stat & USE_DCIN_BIT)
		return IRQ_HANDLED;

	if (is_storming(&irq_data->storm_data)) {
		/* This could be a weak charger reduce ICL */
		if (!is_client_vote_enabled(chg->usb_icl_votable,
						WEAK_CHARGER_VOTER) || (chg->weak_charger_count < 7 && !chg->is_dchg_adapter)) { /* vivo modify */
			smblib_err(chg,
				"Weak charger detected: voting %dmA ICL\n",
				chg->weak_chg_icl_ua / 1000);
			/* vivo modify */
			chg->chg_abnormal_flag |= CHG_WEAK_CHARGER_ERROR;
			chg->weak_charger_count++;
			if (chg->weak_charger_count < 4)
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					true, chg->weak_chg_icl_ua);
			/* vivo end */
			/*
			 * reset storm data and set the storm threshold
			 * to 3 for reverse boost detection.
			 */
			update_storm_count(wdata, BOOST_BACK_STORM_COUNT);
			/* vivo add */
			cancel_delayed_work(&chg->weak_charger_voter_clear_work);
			if (chg->weak_charger_count < 4) {
				schedule_delayed_work(&chg->weak_charger_voter_clear_work,
						msecs_to_jiffies(5000));
			} else if (chg->weak_charger_count == 5) {
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER, true, WEAK_CHG_CUR_400_UA);
				chg->chg_abnormal_flag |= CHG_WEAK_CHARGER_400_ERROR;
			} else if (chg->weak_charger_count == 6) {
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER, true, WEAK_CHG_CUR_300_UA);
				chg->chg_abnormal_flag |= CHG_WEAK_CHARGER_300_ERROR;
			} else if (chg->weak_charger_count == 7) {
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER, true, WEAK_CHG_CUR_200_UA);
				chg->chg_abnormal_flag |= CHG_WEAK_CHARGER_200_ERROR;
			}
			/* vivo end*/
		} else {
			smblib_err(chg,
				"Reverse boost detected: voting 0mA to suspend input\n");
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true, 0);
			vote(chg->awake_votable, BOOST_BACK_VOTER, true, 0);
			/*
			 * Remove the boost-back vote after a delay, to avoid
			 * permanently suspending the input if the boost-back
			 * condition is unintentionally hit.
			 */
			schedule_delayed_work(&chg->bb_removal_work,
				msecs_to_jiffies(BOOST_BACK_UNVOTE_DELAY_MS));
			/* vivo add */
			chg->chg_abnormal_flag |= CHG_BOOST_BACK_ERROR;
			/* vivo end*/
		}
	}

	return IRQ_HANDLED;
}

irqreturn_t wdog_snarl_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, true, 0);
		schedule_delayed_work(&chg->thermal_regulation_work, 0);
	}

	power_supply_changed(chg->batt_psy);

	return IRQ_HANDLED;
}

irqreturn_t wdog_bark_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_write(chg, BARK_BITE_WDOG_PET_REG, BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't pet the dog rc=%d\n", rc);

	return IRQ_HANDLED;
}

static void smblib_die_rst_icl_regulate(struct smb_charger *chg)
{
	int rc;
	u8 temp;

	rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE_TEMP_STATUS_REG rc=%d\n",
				rc);
		return;
	}

	/* Regulate ICL on die temp crossing DIE_RST threshold */
	vote(chg->usb_icl_votable, DIE_TEMP_VOTER,
				temp & DIE_TEMP_RST_BIT, 500000);
}

/*
 * triggered when DIE or SKIN or CONNECTOR temperature across
 * either of the _REG_L, _REG_H, _RST, or _SHDN thresholds
 */
irqreturn_t temp_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_die_rst_icl_regulate(chg);

	return IRQ_HANDLED;
}

static void smblib_usbov_dbc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						usbov_dbc_work.work);

	smblib_dbg(chg, PR_MISC, "Resetting USBOV debounce\n");
	chg->dbc_usbov = false;
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
}

#define USB_OV_DBC_PERIOD_MS		1000
irqreturn_t usbin_ov_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (!(chg->wa_flags & USBIN_OV_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	/*
	 * On specific PMICs, OV IRQ triggers for very small duration in
	 * interim periods affecting charging status reflection. In order to
	 * differentiate between OV IRQ glitch and real OV_IRQ, add a debounce
	 * period for evaluation.
	 */
	if (stat & USBIN_OV_RT_STS_BIT) {
		chg->dbc_usbov = true;
		vote(chg->awake_votable, USBOV_DBC_VOTER, true, 0);
		schedule_delayed_work(&chg->usbov_dbc_work,
				msecs_to_jiffies(USB_OV_DBC_PERIOD_MS));
	} else {
		cancel_delayed_work_sync(&chg->usbov_dbc_work);
		chg->dbc_usbov = false;
		vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	}

	smblib_dbg(chg, PR_MISC, "USBOV debounce status %d\n",
				chg->dbc_usbov);
	return IRQ_HANDLED;
}

/**************
 * Additional USB PSY getters/setters
 * that call interrupt functions
 ***************/

int smblib_get_prop_pr_swap_in_progress(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->pr_swap_in_progress;
	return 0;
}

#define DETACH_DETECT_DELAY_MS 20
int smblib_set_prop_pr_swap_in_progress(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;
	u8 stat = 0, orientation;

	/*vivo add*/
	if (!chg->usb_det_pin_mode && !chg->drp_enable) {
		smblib_dbg(chg, PR_MISC, "drp_enable=0, skip...[value=%d]\n", val->intval);
		return 0;
	}
	if (chg->usb_det_pin_mode && !get_effective_result_locked(chg->drp_mode_votable)) {
		smblib_dbg(chg, PR_MISC, "usb_det_pin_mode: votable result drp_enable=0, skip...[value=%d]\n", val->intval);
		return 0;
	}
	/*vivo end*/

	smblib_dbg(chg, PR_MISC, "Requested PR_SWAP %d\n", val->intval);
	chg->pr_swap_in_progress = val->intval;

	/* check for cable removal during pr_swap */
	if (!chg->pr_swap_in_progress) {
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		vote(chg->awake_votable, DETACH_DETECT_VOTER, true, 0);
		schedule_delayed_work(&chg->pr_swap_detach_work,
				msecs_to_jiffies(DETACH_DETECT_DELAY_MS));
	}

	rc = smblib_masked_write(chg, TYPE_C_DEBOUNCE_OPTION_REG,
			REDUCE_TCCDEBOUNCE_TO_2MS_BIT,
			val->intval ? REDUCE_TCCDEBOUNCE_TO_2MS_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set tCC debounce rc=%d\n", rc);

	rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
			BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT,
			val->intval ? BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set exit state cfg rc=%d\n", rc);

	if (chg->pr_swap_in_progress) {
		rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
				rc);
		}

		orientation =
			stat & CC_ORIENTATION_BIT ? TYPEC_CCOUT_VALUE_BIT : 0;
		rc = smblib_masked_write(chg, TYPE_C_CCOUT_CONTROL_REG,
			TYPEC_CCOUT_SRC_BIT | TYPEC_CCOUT_BUFFER_EN_BIT
					| TYPEC_CCOUT_VALUE_BIT,
			TYPEC_CCOUT_SRC_BIT | TYPEC_CCOUT_BUFFER_EN_BIT
					| orientation);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
				rc);
		}
	} else {
		rc = smblib_masked_write(chg, TYPE_C_CCOUT_CONTROL_REG,
			TYPEC_CCOUT_SRC_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
				rc);
			return rc;
		}

		/* enable DRP */
		rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable DRP rc=%d\n", rc);
			return rc;
		}
		chg->power_role = POWER_SUPPLY_TYPEC_PR_DUAL;
		smblib_dbg(chg, PR_MISC, "restore power role: %d\n",
				chg->power_role);
	}

	return 0;
}

/***************
 * Work Queues *
 ***************/
static void smblib_pr_lock_clear_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pr_lock_clear_work.work);

	spin_lock(&chg->typec_pr_lock);
	if (chg->pr_lock_in_progress) {
		smblib_dbg(chg, PR_MISC, "restore type-c interrupts\n");
		smblib_typec_irq_config(chg, true);
		chg->pr_lock_in_progress = false;
	}
	spin_unlock(&chg->typec_pr_lock);
}

static void smblib_pr_swap_detach_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pr_swap_detach_work.work);
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATE_MACHINE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read STATE_MACHINE_STS rc=%d\n", rc);
		goto out;
	}
	smblib_dbg(chg, PR_REGISTER, "STATE_MACHINE_STS %x\n", stat);
	if (!(stat & TYPEC_ATTACH_DETACH_STATE_BIT)) {
		rc = smblib_request_dpdm(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable DPDM rc=%d\n", rc);

		if (chg->typec_port)
			typec_partner_unregister(chg);
	}
out:
	vote(chg->awake_votable, DETACH_DETECT_VOTER, false, 0);
}

static void smblib_uusb_otg_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						uusb_otg_work.work);
	int rc;
	u8 stat;
	bool otg;

	rc = smblib_read(chg, TYPEC_U_USB_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_3 rc=%d\n", rc);
		goto out;
	}
	otg = !!(stat & U_USB_GROUND_NOVBUS_BIT);
	if (chg->otg_present != otg) {
		if (otg) {
			/* otg cable inserted */
			if (chg->typec_port) {
				typec_partner_register(chg);
				typec_set_data_role(chg->typec_port,
							TYPEC_HOST);
				typec_set_pwr_role(chg->typec_port,
							TYPEC_SOURCE);
			}
		} else if (chg->typec_port) {
			/* otg cable removed */
			typec_set_data_role(chg->typec_port, TYPEC_DEVICE);
			typec_set_pwr_role(chg->typec_port, TYPEC_SINK);
			typec_partner_unregister(chg);
		}

		smblib_notify_usb_host(chg, otg);
	} else {
		goto out;
	}

	chg->otg_present = otg;
	if (!otg)
		chg->boost_current_ua = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
				otg ? chg->chg_freq.freq_below_otg_threshold
					: chg->chg_freq.freq_removal);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);

	smblib_dbg(chg, PR_REGISTER, "TYPE_C_U_USB_STATUS = 0x%02x OTG=%d\n",
			stat, otg);
	power_supply_changed(chg->usb_psy);

out:
	vote(chg->awake_votable, OTG_DELAY_VOTER, false, 0);
}

static void bms_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bms_update_work);

	smblib_suspend_on_debug_battery(chg);

	if (chg->batt_psy)
		power_supply_changed(chg->batt_psy);
}

static void pl_update_work(struct work_struct *work)
{
	union power_supply_propval prop_val;
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_update_work);
	int rc;

	if (chg->smb_temp_max == -EINVAL) {
		rc = smblib_get_thermal_threshold(chg,
					SMB_REG_H_THRESHOLD_MSB_REG,
					&chg->smb_temp_max);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't get charger_temp_max rc=%d\n",
					rc);
			return;
		}
	}

	prop_val.intval = chg->smb_temp_max;
	rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
				&prop_val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_CHARGER_TEMP_MAX rc=%d\n",
				rc);
		return;
	}

	if (chg->sec_chg_selected == POWER_SUPPLY_CHARGER_SEC_CP)
		return;

	smblib_select_sec_charger(chg, POWER_SUPPLY_CHARGER_SEC_PL,
				POWER_SUPPLY_CP_NONE, false);
}

static void clear_hdc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						clear_hdc_work.work);

	chg->is_hdc = false;
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);
}

static void smblib_icl_change_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							icl_change_work.work);
	int rc, settled_ua;

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &settled_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
		return;
	}

	power_supply_changed(chg->usb_main_psy);

	smblib_dbg(chg, PR_INTERRUPT, "icl_settled=%d\n", settled_ua);
}

static void smblib_pl_enable_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							pl_enable_work.work);

	smblib_dbg(chg, PR_PARALLEL, "timer expired, enabling parallel\n");
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);
}

static void smblib_thermal_regulation_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						thermal_regulation_work.work);
	int rc;

	rc = smblib_update_thermal_readings(chg);
	if (rc < 0)
		smblib_err(chg, "Couldn't read current thermal values %d\n",
					rc);

	rc = smblib_process_thermal_readings(chg);
	if (rc < 0)
		smblib_err(chg, "Couldn't run sw thermal regulation %d\n",
					rc);
}

#define MOISTURE_PROTECTION_CHECK_DELAY_MS 300000		/* 5 mins */
static void smblib_moisture_protection_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						moisture_protection_work);
	int rc;
	bool usb_plugged_in;
	u8 stat;

	/*
	 * Hold awake votable to prevent pm_relax being called prior to
	 * completion of this work.
	 */
	vote(chg->awake_votable, MOISTURE_VOTER, true, 0);

	/*
	 * Disable 1% duty cycle on CC_ID pin and enable uUSB factory mode
	 * detection to track any change on RID, as interrupts are disable.
	 */
	rc = smblib_write(chg, ((chg->chg_param.smb_version == PMI632_SUBTYPE) ?
			PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
			TYPEC_U_USB_WATER_PROTECTION_CFG_REG), 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable periodic monitoring of CC_ID rc=%d\n",
			rc);
		goto out;
	}

	rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT,
					EN_MICRO_USB_FACTORY_MODE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable uUSB factory mode detection rc=%d\n",
			rc);
		goto out;
	}

	/*
	 * Add a delay of 100ms to allow change in rid to reflect on
	 * status registers.
	 */
	msleep(100);

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		goto out;
	}
	usb_plugged_in = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	/* Check uUSB status for moisture presence */
	rc = smblib_read(chg, TYPEC_U_USB_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_U_USB_STATUS_REG rc=%d\n",
				rc);
		goto out;
	}

	/*
	 * Factory mode detection happens in case of USB plugged-in by using
	 * a different current source of 2uA which can hamper moisture
	 * detection. Since factory mode is not supported in kernel, factory
	 * mode detection can be considered as equivalent to presence of
	 * moisture.
	 */
	if (stat == U_USB_STATUS_WATER_PRESENT || stat == U_USB_FMB1_BIT ||
			stat == U_USB_FMB2_BIT || (usb_plugged_in &&
			stat == U_USB_FLOAT1_BIT)) {
		smblib_set_moisture_protection(chg, true);
		alarm_start_relative(&chg->moisture_protection_alarm,
			ms_to_ktime(MOISTURE_PROTECTION_CHECK_DELAY_MS));
	} else {
		smblib_set_moisture_protection(chg, false);
		rc = alarm_cancel(&chg->moisture_protection_alarm);
		if (rc < 0)
			smblib_err(chg, "Couldn't cancel moisture protection alarm\n");
	}

out:
	vote(chg->awake_votable, MOISTURE_VOTER, false, 0);
}

static enum alarmtimer_restart moisture_protection_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
					moisture_protection_alarm);

	smblib_dbg(chg, PR_MISC, "moisture Protection Alarm Triggered %lld\n",
			ktime_to_ms(now));

	/* Atomic context, cannot use voter */
	pm_stay_awake(chg->dev);
	schedule_work(&chg->moisture_protection_work);

	return ALARMTIMER_NORESTART;
}

static void smblib_chg_termination_work(struct work_struct *work)
{
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(work, struct smb_charger,
						chg_termination_work);
	int rc, input_present, delay = CHG_TERM_WA_ENTRY_DELAY_MS;
	int vbat_now_uv, max_fv_uv;

	/*
	 * Hold awake votable to prevent pm_relax being called prior to
	 * completion of this work.
	 */
	vote(chg->awake_votable, CHG_TERMINATION_VOTER, true, 0);

	rc = smblib_is_input_present(chg, &input_present);
	if ((rc < 0) || !input_present)
		goto out;

	rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_REAL_CAPACITY, &pval);
	if ((rc < 0) || (pval.intval < 100)) {
		vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
		goto out;
	}

	/* Get the battery float voltage */
	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_VOLTAGE_MAX,
				&pval);
	if (rc < 0)
		goto out;

	max_fv_uv = pval.intval;

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CHARGE_FULL,
					&pval);
	if (rc < 0)
		goto out;

	/*
	 * On change in the value of learned capacity, re-initialize the
	 * reference cc_soc value due to change in cc_soc characteristic value
	 * at full capacity. Also, in case cc_soc_ref value is reset,
	 * re-initialize it.
	 */
	if (pval.intval != chg->charge_full_cc || !chg->cc_soc_ref) {
		chg->charge_full_cc = pval.intval;

		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
		if (rc < 0)
			goto out;

		/*
		 * Store the Vbat at the charge termination to compare with
		 * the current voltage to see if the Vbat is increasing after
		 * charge termination in BSM.
		 */
		chg->term_vbat_uv = pval.intval;
		vbat_now_uv = pval.intval;

		rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CC_SOC,
					&pval);
		if (rc < 0)
			goto out;

		chg->cc_soc_ref = pval.intval;
	} else {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
		if (rc < 0)
			goto out;

		vbat_now_uv = pval.intval;

		rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CC_SOC,
					&pval);
		if (rc < 0)
			goto out;
	}

	/*
	 * In BSM a sudden jump in CC_SOC is not expected. If seen, its a
	 * good_ocv or updated capacity, reject it.
	 */
	if (chg->last_cc_soc && pval.intval > (chg->last_cc_soc + 100)) {
		/* CC_SOC has increased by 1% from last time */
		chg->cc_soc_ref = pval.intval;
		smblib_dbg(chg, PR_MISC, "cc_soc jumped(%d->%d), reset cc_soc_ref\n",
				chg->last_cc_soc, pval.intval);
	}
	chg->last_cc_soc = pval.intval;

	/*
	 * Suspend/Unsuspend USB input to keep cc_soc within the 0.5% to 0.75%
	 * overshoot range of the cc_soc value at termination and make sure that
	 * vbat is indeed rising above vfloat.
	 */
	if (pval.intval < DIV_ROUND_CLOSEST(chg->cc_soc_ref * 10050, 10000)) {
		vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
		delay = CHG_TERM_WA_ENTRY_DELAY_MS;
	} else if ((pval.intval > DIV_ROUND_CLOSEST(chg->cc_soc_ref * 10075,
								10000))
		  && ((vbat_now_uv > chg->term_vbat_uv) &&
		     (vbat_now_uv > max_fv_uv))) {

		if (input_present & INPUT_PRESENT_USB)
			vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER,
					true, 0);
		if (input_present & INPUT_PRESENT_DC)
			vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER,
					true, 0);
		delay = CHG_TERM_WA_EXIT_DELAY_MS;
	}

	smblib_dbg(chg, PR_MISC, "Chg Term WA readings: cc_soc: %d, cc_soc_ref: %d, delay: %d vbat_now %d term_vbat %d\n",
			pval.intval, chg->cc_soc_ref, delay, vbat_now_uv,
			chg->term_vbat_uv);
	alarm_start_relative(&chg->chg_termination_alarm, ms_to_ktime(delay));
out:
	vote(chg->awake_votable, CHG_TERMINATION_VOTER, false, 0);
}

static enum alarmtimer_restart chg_termination_alarm_cb(struct alarm *alarm,
								ktime_t now)
{
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
							chg_termination_alarm);

	smblib_dbg(chg, PR_MISC, "Charge termination WA alarm triggered %lld\n",
			ktime_to_ms(now));

	/* Atomic context, cannot use voter */
	pm_stay_awake(chg->dev);
	schedule_work(&chg->chg_termination_work);

	return ALARMTIMER_NORESTART;
}

static void apsd_timer_cb(struct timer_list *tm)
{
	struct smb_charger *chg = container_of(tm, struct smb_charger,
							apsd_timer);

	smblib_dbg(chg, PR_MISC, "APSD Extented timer timeout at %lld\n",
			jiffies_to_msecs(jiffies));

	chg->apsd_ext_timeout = true;
}

#define SOFT_JEITA_HYSTERESIS_OFFSET	0x200
static void jeita_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						jeita_update_work);
	struct device_node *node = chg->dev->of_node;
	struct device_node *batt_node, *pnode;
	union power_supply_propval val;
	int rc, tmp[2], max_fcc_ma, max_fv_uv;
	u32 jeita_hard_thresholds[2];
	u16 addr;
	u8 buff[2];
	/*vivo add*/
	/* COLD = -5 DegC, HOT = 60 DegC */
	jeita_hard_thresholds[0] = 0x5dce;
	jeita_hard_thresholds[1] = 0x14a6;

	rc = smblib_update_jeita(chg, jeita_hard_thresholds,
				JEITA_HARD);
	if (rc < 0) {
		smblib_err(chg, "Couldn't configure Hard Jeita rc=%d\n",
				rc);
		goto out;
	}

	goto finish;
	/*vivo end*/

	batt_node = of_find_node_by_name(node, "qcom,battery-data");
	if (!batt_node) {
		smblib_err(chg, "Batterydata not available\n");
		goto out;
	}

	/* if BMS is not ready, defer the work */
	if (!chg->bms_psy)
		return;

	rc = smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_RESISTANCE_ID, &val);
	if (rc < 0) {
		smblib_err(chg, "Failed to get batt-id rc=%d\n", rc);
		goto out;
	}

	/* if BMS hasn't read out the batt_id yet, defer the work */
	if (val.intval <= 0)
		return;

	pnode = of_batterydata_get_best_profile(batt_node,
					val.intval / 1000, NULL);
	if (IS_ERR(pnode)) {
		rc = PTR_ERR(pnode);
		smblib_err(chg, "Failed to detect valid battery profile %d\n",
				rc);
		goto out;
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-hard-thresholds",
				jeita_hard_thresholds, 2);
	if (!rc) {
		rc = smblib_update_jeita(chg, jeita_hard_thresholds,
					JEITA_HARD);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure Hard Jeita rc=%d\n",
					rc);
			goto out;
		}
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-thresholds",
				chg->jeita_soft_thlds, 2);
	if (!rc) {
		rc = smblib_update_jeita(chg, chg->jeita_soft_thlds,
					JEITA_SOFT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure Soft Jeita rc=%d\n",
					rc);
			goto out;
		}

		rc = of_property_read_u32_array(pnode,
					"qcom,jeita-soft-hys-thresholds",
					chg->jeita_soft_hys_thlds, 2);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get Soft Jeita hysteresis thresholds rc=%d\n",
					rc);
			goto out;
		}
	} else {
		/* Populate the jeita-soft-thresholds */
		addr = CHGR_JEITA_THRESHOLD_BASE_REG(JEITA_SOFT);
		rc = smblib_batch_read(chg, addr, buff, 2);
		if (rc < 0) {
			pr_err("failed to read 0x%4X, rc=%d\n", addr, rc);
			goto out;
		}

		chg->jeita_soft_thlds[1] = buff[1] | buff[0] << 8;

		rc = smblib_batch_read(chg, addr + 2, buff, 2);
		if (rc < 0) {
			pr_err("failed to read 0x%4X, rc=%d\n", addr + 2, rc);
			goto out;
		}

		chg->jeita_soft_thlds[0] = buff[1] | buff[0] << 8;

		/*
		 * Update the soft jeita hysteresis 2 DegC less for warm and
		 * 2 DegC more for cool than the soft jeita thresholds to avoid
		 * overwriting the registers with invalid values.
		 */
		chg->jeita_soft_hys_thlds[0] =
			chg->jeita_soft_thlds[0] - SOFT_JEITA_HYSTERESIS_OFFSET;
		chg->jeita_soft_hys_thlds[1] =
			chg->jeita_soft_thlds[1] + SOFT_JEITA_HYSTERESIS_OFFSET;
	}

	chg->jeita_soft_fcc[0] = chg->jeita_soft_fcc[1] = -EINVAL;
	chg->jeita_soft_fv[0] = chg->jeita_soft_fv[1] = -EINVAL;
	max_fcc_ma = max_fv_uv = -EINVAL;

	of_property_read_u32(pnode, "qcom,fastchg-current-ma", &max_fcc_ma);
	of_property_read_u32(pnode, "qcom,max-voltage-uv", &max_fv_uv);

	if (max_fcc_ma <= 0 || max_fv_uv <= 0) {
		smblib_err(chg, "Incorrect fastchg-current-ma or max-voltage-uv\n");
		goto out;
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-fcc-ua",
					tmp, 2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get fcc values for soft JEITA rc=%d\n",
				rc);
		goto out;
	}

	max_fcc_ma *= 1000;
	if (tmp[0] > max_fcc_ma || tmp[1] > max_fcc_ma) {
		smblib_err(chg, "Incorrect FCC value [%d %d] max: %d\n", tmp[0],
			tmp[1], max_fcc_ma);
		goto out;
	}
	chg->jeita_soft_fcc[0] = tmp[0];
	chg->jeita_soft_fcc[1] = tmp[1];

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-fv-uv", tmp,
					2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get fv values for soft JEITA rc=%d\n",
				rc);
		goto out;
	}

	if (tmp[0] > max_fv_uv || tmp[1] > max_fv_uv) {
		smblib_err(chg, "Incorrect FV value [%d %d] max: %d\n", tmp[0],
			tmp[1], max_fv_uv);
		goto out;
	}
	chg->jeita_soft_fv[0] = tmp[0];
	chg->jeita_soft_fv[1] = tmp[1];

	rc = smblib_soft_jeita_arb_wa(chg);
	if (rc < 0) {
		smblib_err(chg, "Couldn't fix soft jeita arb rc=%d\n",
				rc);
		goto out;
	}

finish:/*vivo add*/
	chg->jeita_configured = JEITA_CFG_COMPLETE;
	return;

out:
	chg->jeita_configured = JEITA_CFG_FAILURE;
}

static void smblib_lpd_ra_open_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							lpd_ra_open_work.work);
	union power_supply_propval pval;
	u8 stat;
	int rc;
	smblib_err(chg, "typec_lpd: pr_swap_in_progress=%d, pd_hard_reset=%d, lpd_stage=%d, lpd_reason=%d\n",
		chg->pr_swap_in_progress, chg->pd_hard_reset, chg->lpd_stage, chg->lpd_reason);

	if (chg->pr_swap_in_progress || chg->pd_hard_reset) {
		chg->lpd_stage = LPD_STAGE_NONE;
		goto out;
	}

	if (chg->lpd_stage != LPD_STAGE_FLOAT)
		goto out;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
			rc);
		goto out;
	}

	/* quit if moisture status is gone or in attached state */
	if (!(stat & TYPEC_WATER_DETECTION_STATUS_BIT)
			|| (stat & TYPEC_TCCDEBOUNCE_DONE_STATUS_BIT)) {
		chg->lpd_stage = LPD_STAGE_NONE;
		goto out;
	}

	chg->lpd_stage = LPD_STAGE_COMMIT;

	/* Enable source only mode */
	pval.intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
	rc = smblib_set_prop_typec_power_role(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set typec source only mode rc=%d\n",
					rc);
		goto out;
	}

	/* Wait 1.5ms to get SBUx ready */
	usleep_range(1500, 1510);

	if (smblib_rsbux_low(chg, RSBU_K_300K_UV)) {
		/* Moisture detected, enable sink only mode */
		pval.intval = POWER_SUPPLY_TYPEC_PR_SINK;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set typec sink only rc=%d\n",
				rc);
			goto out;
		}

		chg->lpd_reason = LPD_MOISTURE_DETECTED;
		chg->moisture_present =  true;
		vote(chg->usb_icl_votable, LPD_VOTER, true, 0);

	} else {
		/* Floating cable, disable water detection irq temporarily */
		smblib_err(chg, "typec_lpd:Floating cable, disable water detection irq temporarily ..\n");	//vivo add
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set TYPE_C_INTERRUPT_EN_CFG_2_REG rc=%d\n",
					rc);
			goto out;
		}

		/* restore DRP mode */
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
			goto out;
		}

		chg->lpd_reason = LPD_FLOATING_CABLE;
	}

	/* recheck in 60 seconds */
	alarm_start_relative(&chg->lpd_recheck_timer, ms_to_ktime(60000));
out:
	vote(chg->awake_votable, LPD_VOTER, false, 0);
}

static void smblib_lpd_detach_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							lpd_detach_work.work);

	if (chg->lpd_stage == LPD_STAGE_FLOAT_CANCEL)
		chg->lpd_stage = LPD_STAGE_NONE;
}

static void smblib_cp_status_change_work(struct work_struct *work)
{
	int rc;
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(work, struct smb_charger,
			cp_status_change_work);

	if (!chg->cp_psy)
		chg->cp_psy = power_supply_get_by_name("charge_pump_master");

	if (!chg->cp_psy)
		goto relax;

	if (chg->cp_topo == -EINVAL) {
		rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_PARALLEL_OUTPUT_MODE, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read cp topo rc=%d\n", rc);
			goto relax;
		}

		chg->cp_topo = pval.intval;

		if (chg->cp_topo == POWER_SUPPLY_PL_OUTPUT_VBAT &&
				chg->cp_reason == POWER_SUPPLY_CP_WIRELESS)
			vote(chg->fcc_main_votable, WLS_PL_CHARGING_VOTER, true,
					800000);
	}
relax:
	pm_relax(chg->dev);
}

static int smblib_create_votables(struct smb_charger *chg)
{
	int rc = 0;

	chg->fcc_votable = find_votable("FCC");
	if (chg->fcc_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FCC votable rc=%d\n", rc);
		return rc;
	}

	chg->fcc_main_votable = find_votable("FCC_MAIN");
	if (chg->fcc_main_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FCC Main votable rc=%d\n", rc);
		return rc;
	}

	chg->fv_votable = find_votable("FV");
	if (chg->fv_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FV votable rc=%d\n", rc);
		return rc;
	}

	chg->usb_icl_votable = find_votable("USB_ICL");
	if (chg->usb_icl_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find USB_ICL votable rc=%d\n", rc);
		return rc;
	}

	chg->pl_disable_votable = find_votable("PL_DISABLE");
	if (chg->pl_disable_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find votable PL_DISABLE rc=%d\n", rc);
		return rc;
	}

	chg->pl_enable_votable_indirect = find_votable("PL_ENABLE_INDIRECT");
	if (chg->pl_enable_votable_indirect == NULL) {
		rc = -EINVAL;
		smblib_err(chg,
			"Couldn't find votable PL_ENABLE_INDIRECT rc=%d\n",
			rc);
		return rc;
	}

	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);

	chg->smb_override_votable = create_votable("SMB_EN_OVERRIDE",
				VOTE_SET_ANY,
				smblib_smb_disable_override_vote_callback, chg);
	if (IS_ERR(chg->smb_override_votable)) {
		rc = PTR_ERR(chg->smb_override_votable);
		chg->smb_override_votable = NULL;
		return rc;
	}

	chg->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					smblib_dc_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->dc_suspend_votable)) {
		rc = PTR_ERR(chg->dc_suspend_votable);
		chg->dc_suspend_votable = NULL;
		return rc;
	}

	chg->awake_votable = create_votable("AWAKE", VOTE_SET_ANY,
					smblib_awake_vote_callback,
					chg);
	if (IS_ERR(chg->awake_votable)) {
		rc = PTR_ERR(chg->awake_votable);
		chg->awake_votable = NULL;
		return rc;
	}

	chg->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					smblib_chg_disable_vote_callback,
					chg);
	if (IS_ERR(chg->chg_disable_votable)) {
		rc = PTR_ERR(chg->chg_disable_votable);
		chg->chg_disable_votable = NULL;
		return rc;
	}

	chg->limited_irq_disable_votable = create_votable(
				"USB_LIMITED_IRQ_DISABLE",
				VOTE_SET_ANY,
				smblib_limited_irq_disable_vote_callback,
				chg);
	if (IS_ERR(chg->limited_irq_disable_votable)) {
		rc = PTR_ERR(chg->limited_irq_disable_votable);
		chg->limited_irq_disable_votable = NULL;
		return rc;
	}

	chg->hdc_irq_disable_votable = create_votable("USB_HDC_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_hdc_irq_disable_vote_callback,
					chg);
	if (IS_ERR(chg->hdc_irq_disable_votable)) {
		rc = PTR_ERR(chg->hdc_irq_disable_votable);
		chg->hdc_irq_disable_votable = NULL;
		return rc;
	}

	chg->icl_irq_disable_votable = create_votable("USB_ICL_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_icl_irq_disable_vote_callback,
					chg);
	if (IS_ERR(chg->icl_irq_disable_votable)) {
		rc = PTR_ERR(chg->icl_irq_disable_votable);
		chg->icl_irq_disable_votable = NULL;
		return rc;
	}

	chg->temp_change_irq_disable_votable = create_votable(
			"TEMP_CHANGE_IRQ_DISABLE", VOTE_SET_ANY,
			smblib_temp_change_irq_disable_vote_callback, chg);
	if (IS_ERR(chg->temp_change_irq_disable_votable)) {
		rc = PTR_ERR(chg->temp_change_irq_disable_votable);
		chg->temp_change_irq_disable_votable = NULL;
		return rc;
	}

	chg->drp_mode_votable = create_votable("DRP_MODE", VOTE_SET_ANY,
					smblib_drp_enable_vote_callback, chg);
	if (IS_ERR(chg->drp_mode_votable)) {
		rc = PTR_ERR(chg->drp_mode_votable);
		chg->drp_mode_votable = NULL;
		return rc;
	}


	return rc;
}

static void smblib_destroy_votables(struct smb_charger *chg)
{
	if (chg->dc_suspend_votable)
		destroy_votable(chg->dc_suspend_votable);
	if (chg->usb_icl_votable)
		destroy_votable(chg->usb_icl_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->chg_disable_votable)
		destroy_votable(chg->chg_disable_votable);
}

static void smblib_iio_deinit(struct smb_charger *chg)
{
	if (!IS_ERR_OR_NULL(chg->iio.usbin_v_chan))
		iio_channel_release(chg->iio.usbin_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_i_chan))
		iio_channel_release(chg->iio.usbin_i_chan);
	if (!IS_ERR_OR_NULL(chg->iio.temp_chan))
		iio_channel_release(chg->iio.temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.sbux_chan))
		iio_channel_release(chg->iio.sbux_chan);
	if (!IS_ERR_OR_NULL(chg->iio.vph_v_chan))
		iio_channel_release(chg->iio.vph_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.die_temp_chan))
		iio_channel_release(chg->iio.die_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.connector_temp_chan))
		iio_channel_release(chg->iio.connector_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.skin_temp_chan))
		iio_channel_release(chg->iio.skin_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.smb_temp_chan))
		iio_channel_release(chg->iio.smb_temp_chan);
	/* vivo add */
	if (!IS_ERR_OR_NULL(chg->iio.board_temp_chan))
		iio_channel_release(chg->iio.board_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.bat_board_temp_chan))
		iio_channel_release(chg->iio.bat_board_temp_chan);
	/* vivo end */
}

/***************************
 *         vivo function                   *
 ***************************/
//VIVO add:  copy below fun from Qcom original driver: smb5-lib.c :: smblib_get_prop_batt_status()
int smblib_get_prop_batt_status_ex(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_online, dc_online;
	u8 stat;
	int rc, suspend = 0;
	if (chg->fake_chg_status_on_debug_batt) {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_DEBUG_BATTERY, &pval);
		if (rc < 0) {
			pr_err_ratelimited("Couldn't get debug battery prop rc=%d\n",
					rc);
		} else if (pval.intval == 1) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			return 0;
		}
	}

	if (chg->dbc_usbov) {
		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't get usb present prop rc=%d\n", rc);
			return rc;
		}

		rc = smblib_get_usb_suspend(chg, &suspend);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't get usb suspend rc=%d\n", rc);
			return rc;
		}

		/*
		 * Report charging as long as USBOV is not debounced and
		 * charging path is un-suspended.
		 */
		if (pval.intval && !suspend) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			return 0;
		}
	}

	rc = smblib_get_prop_usb_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc online property rc=%d\n",
			rc);
		return rc;
	}
	dc_online = (bool)pval.intval;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!usb_online && !dc_online) {
		switch (stat) {
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		return rc;
	}

	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case DISABLE_CHARGE:
	case PAUSE_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	if (is_charging_paused(chg)) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	/*
	 * If charge termination WA is active and has suspended charging, then
	 * continue reporting charging status as FULL.
	 */
	if (is_client_vote_enabled_locked(chg->usb_icl_votable,
						CHG_TERMINATION_VOTER)) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if (val->intval != POWER_SUPPLY_STATUS_CHARGING)
		return 0;

	if (!usb_online && dc_online
		&& chg->fake_batt_status == POWER_SUPPLY_STATUS_FULL) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
				rc);
			return rc;
	}

	stat &= ENABLE_TRICKLE_BIT | ENABLE_PRE_CHARGING_BIT |
						ENABLE_FULLON_MODE_BIT;

	if (!stat)
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

/*get power psy point*/
static bool is_parallel_charger_available(struct smb_charger *chg)
{
	if (!chg->pl.psy)
		chg->pl.psy = power_supply_get_by_name("parallel");

	if (!chg->pl.psy)
		return false;

	return true;
}

int smblib_keep_chg_soc(struct smb_charger *chg)
{
	bool enable_chg = true;
	bool enable_power_path = true;
	union power_supply_propval pval = {0,};
	union power_supply_propval batt_soc = {0, };

	if (is_atboot)
		return 0;

	smblib_get_prop_batt_capacity(chg, &batt_soc);

	if (chg->keep_chg_soc == 100 || chg->factory_mode_state) {
		enable_chg = true;
		enable_power_path = true;
	} else {
		if (batt_soc.intval < chg->keep_chg_soc) {
			enable_chg = true;
			enable_power_path = true;
		} else if (batt_soc.intval == chg->keep_chg_soc) {
			enable_chg = false;
			enable_power_path = true;
		} else {
			enable_chg = false;
			enable_power_path = false;
		}
	}

	if (enable_chg && is_parallel_charger_available(chg)) {
		pval.intval = 1;
		power_supply_set_property(chg->pl.psy,
						POWER_SUPPLY_PROP_PRESENT, &pval);
		msleep(300);
	}

	vote(chg->chg_disable_votable, KEEP_CHG_SOC,
			!enable_chg, 0);
	vote(chg->usb_icl_votable, KEEP_CHG_SOC,
		!enable_power_path, 0);

	/* disable direct charge */
	if (!chg->dchg_disable_votable)
		chg->dchg_disable_votable = find_votable("DCHG_DISABLE");
	if (chg->dchg_disable_votable)
		vote(chg->dchg_disable_votable, KEEP_CHG_SOC, !enable_chg, 0);

	if (!enable_chg && is_parallel_charger_available(chg)) {
		msleep(300);
		pval.intval = 0;
		power_supply_set_property(chg->pl.psy,
						POWER_SUPPLY_PROP_PRESENT, &pval);
	}

	smblib_dbg(chg, PR_MISC, "%d.%d, keep_chg_soc=%d, soc=%d, chg->factory_mode_state=%d\n",
		 enable_chg, enable_power_path, chg->keep_chg_soc, batt_soc.intval, chg->factory_mode_state);
	//power_supply_changed(chg->usb_psy);
	return 0;
}

/* add for ffc start */
#define RAW_ITERM(iterm_ma, max_range)				\
		div_s64((int64_t)iterm_ma * ADC_CHG_ITERM_MASK, max_range)
static int vivo_set_chg_term_current(struct smb_charger *chg, int term_current)
{
	u8 *buf;
	int rc = 0;
	s16 raw_hi_thresh, max_limit_ma;
	union power_supply_propval val = {0,};

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		max_limit_ma = ITERM_LIMITS_PMI632_MA;
	else
		max_limit_ma = ITERM_LIMITS_PM8150B_MA;

	if (term_current < (-1 * max_limit_ma)
		|| term_current > max_limit_ma) {
		dev_err(chg->dev, "ITERM threshold out of range rc=%d\n", rc);
		return -EINVAL;
	}

	/*
	 * Conversion:
	 *	raw (A) = (term_current * ADC_CHG_ITERM_MASK) / max_limit_ma
	 * Note: raw needs to be converted to big-endian format.
	 */

	if (chg->term_current_thresh_hi_ma != term_current) {
		raw_hi_thresh = RAW_ITERM(term_current,
					max_limit_ma);
		raw_hi_thresh = sign_extend32(raw_hi_thresh, 15);
		buf = (u8 *)&raw_hi_thresh;
		raw_hi_thresh = buf[1] | (buf[0] << 8);

		rc = smblib_batch_write(chg, CHGR_ADC_ITERM_UP_THD_MSB_REG,
				(u8 *)&raw_hi_thresh, 2);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't configure ITERM threshold HIGH rc=%d\n",
					rc);
			return rc;
		}

		smblib_get_prop_batt_iterm(chg, &val);
		if (val.intval == term_current) {
			chg->term_current_thresh_hi_ma = term_current;
			smblib_dbg(chg, PR_MISC, "setting term_current=%d normal...\n", term_current);
		} else {
			chg->term_current_thresh_hi_ma = 0;
			smblib_dbg(chg, PR_MISC, "setting term_current error!!! want set = %d, but read back =%d...\n", term_current, val.intval);
		}
	}
	return rc;
}

static int vivo_set_chg_term_voltage(struct smb_charger *chg, int term_volt_uv)
{
	int rc = 0;

	vote(chg->fv_votable, HW_LIMIT_VOTER,
		term_volt_uv > 0, term_volt_uv);

	return rc;
}
/* add for ffc end */
static bool vivo_ex_fg_i2c_is_exception(struct smb_charger *chg)
{
	/*bq27750_IIC_ERROR = BIT(1) */
	if (ex_fg_support && (ex_fg_i2c_hand_step == 4 || ex_fg_i2c_hand_step >= 7
		|| ex_fg_i2c_error_counter > 20) && (ex_fg_state & BIT(1))) {
		smblib_dbg(chg, PR_MISC, "ex fg i2c is exception!\n");
		return true;
	}else {
		return false;
	}
}

static int smblib_quit_collapse_status(struct smb_charger *chg)
{
	struct smb5 *chip = container_of(chg, struct smb5, chg);/*vivo add*/
	int rc;
	smblib_dbg(chg, PR_MISC, "\n");

	/* quit collapse status */
	rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
					SUSPEND_ON_COLLAPSE_USBIN_BIT, 0);
	if (rc) {
		pr_err("failed to read USBIN_AICL_OPTIONS_CFG_REG ->SUSPEND_ON_COLLAPSE_USBIN_BIT, rc=%d\n", rc);
		return rc;
	}
	msleep(100);
	rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
					SUSPEND_ON_COLLAPSE_USBIN_BIT, SUSPEND_ON_COLLAPSE_USBIN_BIT);
	if (rc) {
		pr_err("failed to read USBIN_AICL_OPTIONS_CFG_REG ->SUSPEND_ON_COLLAPSE_USBIN_BIT, rc=%d\n", rc);
		return rc;
	}

	/* Force 5V HVDCP */
	rc = smblib_force_vbus_voltage(chg, FORCE_5V_BIT);
	if (rc < 0) {
		pr_err("Failed to force 5V\n");
		return rc;
	}
	msleep(100);

	/* rerun aicl */
	smblib_run_aicl(chg, RESTART_AICL);
	msleep(100);

	if (chip->dt.disable_suspend_on_collapse) {
		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
					SUSPEND_ON_COLLAPSE_USBIN_BIT, 0);
		if (rc) {
			smblib_err(chg, "failed to read USBIN_AICL_OPTIONS_CFG_REG ->SUSPEND_ON_COLLAPSE_USBIN_BIT, rc=%d\n", rc);
		}
	}

	return 0;
}

/*dchg function*/
static struct power_supply *get_mcu_psy(struct smb_charger *chg)
{
	if (!chg->direct_charger_enable)
		return NULL;

	if (chg->mcu_psy)
		return chg->mcu_psy;
	chg->mcu_psy = power_supply_get_by_name("mcu");
	if (!chg->mcu_psy)
		smblib_err(chg,"mcu_psy not found\n");
	return chg->mcu_psy;
}

static void smblib_usbsel_config(struct smb_charger *chg,bool high)
{
	union power_supply_propval usbsel = {0,};
	struct power_supply *mcu_psy = get_mcu_psy(chg);

	if (!chg->direct_charger_enable) {
		smblib_dbg(chg, PR_MISC,"!direct_charger_enable\n");
		return;
	}

	usbsel.intval = high;
	if (!mcu_psy) {
		smblib_err(chg,"mcu_psy not found\n");
		return;
	}
	power_supply_set_property(chg->mcu_psy,
	POWER_SUPPLY_PROP_USBSEL,&usbsel);
}

int smblib_get_handshake_status(struct smb_charger *chg)
{
	union power_supply_propval handshake_status = {0,};
	struct power_supply *mcu_psy = get_mcu_psy(chg);

	if (!chg->direct_charger_enable) {
		smblib_dbg(chg, PR_MISC,"!direct_charger_enable\n");
		return 0;
	}

	if (!mcu_psy) {
		smblib_err(chg,"mcu_psy not found\n");
		return 0;
	}
	power_supply_get_property(chg->mcu_psy,
	POWER_SUPPLY_PROP_HANDSHAKE_STATUS,&handshake_status);

	return handshake_status.intval;
}
/*dchg function end*/

/* vivo add for cc pin short protect */
static void smblib_handle_plugin_removal(struct smb_charger *chg)
{
	/* reset input current limit voters */
	//vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, true, 100000);
	vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	//vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);
	vote(chg->usb_icl_votable, CTM_VOTER, false, 0);

	/* reset hvdcp voters */
	//vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER, true, 0);
	//vote(chg->hvdcp_disable_votable_indirect, PD_INACTIVE_VOTER, true, 0);

	/* reset power delivery voters */
	//vote(chg->pd_allowed_votable, PD_VOTER, false, 0);
	//vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, true, 0);
	//vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER, true, 0);


	/* reset parallel voters */
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->pl_disable_votable, PL_FCC_LOW_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);
}

static void smblib_usb_plugin_init(struct smb_charger *chg)
{
	struct smb5 *chip = container_of(chg, struct smb5, chg);
	union power_supply_propval pval = {0,};

	// charger_connect_judge(1);
	if(ex_fg_ffc_support) {
		vivo_set_chg_term_voltage(chg, chip->dt.batt_profile_fv_uv);
		vivo_set_chg_term_current(chg, chip->dt.term_current_thresh_hi_ma);
		if(chg->dchg_disable_votable)
			vote(chg->dchg_disable_votable, FFC_VOTER, false, 0);
	}
	chg->usb_conndoned = false;
	chg->detect_icl_ibat_ua = SMBCHG_DETECT_ICL_IBAT_UA;
	vote(chg->fcc_votable, USB_FIRST_ICL_DONE_VOTER, true, chg->detect_icl_ibat_ua);
	chg->primary_fastchg_ua = chg->primary_fastchg_max_ua;
	chg->parallel_fastchg_ua = chg->parallel_fastchg_max_ua;
	chg->is_dchg_adapter = false;
	chg->last_dischg_change_record_tm = 0;
	chg->last_qg_monitor_work_count = -1;
	chg->last_chg_drop_change_record_tm = 0;
	chg->chg_less_than_consume = false;

	if (chg->dchg_disable_votable)
		vote(chg->dchg_disable_votable, KEEP_CHG_SOC, false, 0);

	if (chg->mode == PARALLEL_MASTER && !wake_lock_active(&chg->chg_wake_lock)){
		if(!bsp_test_mode || power_off_charging_mode){
			smblib_dbg(chg, PR_MISC, "get wakelock\n");
			wake_lock(&chg->chg_wake_lock);
		}else{
			smblib_dbg(chg, PR_MISC, "bsp test ignore get wakelock\n");
		}
	}

	if(is_parallel_charger_available(chg)){
		pval.intval = 1;
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_PRESENT, &pval);
	}
	vote(chg->pl_disable_votable, USER_CTRL_PL_VOTER, true, 0);

	cancel_delayed_work(&chg->split_current_monitor_work);
	schedule_delayed_work(&chg->split_current_monitor_work,
					msecs_to_jiffies((chg->direct_charger_enable) ? 100 : 5000));//dchg project, skip this work quickly

	chg->bigdata_chg_voltage_uv = 0;
	chg->ffc_pmi_charging_count = 0;
	chg->engine = POWER_SUPPLY_ENGINE_NORMAL;
	cancel_delayed_work(&chg->charge_monitor_work);
	schedule_delayed_work(&chg->charge_monitor_work, msecs_to_jiffies(5000));
}

static void smblib_usb_plugin_deinit(struct smb_charger *chg)
{
		struct smb5 *chip = container_of(chg, struct smb5, chg);
		union power_supply_propval pval = {0,};

		// charger_connect_judge(0);
		if(ex_fg_ffc_support) {
			vivo_set_chg_term_voltage(chg, chip->dt.batt_profile_fv_uv);
			vivo_set_chg_term_current(chg, chip->dt.term_current_thresh_hi_ma);
			chg->ffc_full_chg_iterm_delta_ma = 0;
			if(chg->dchg_disable_votable)
				vote(chg->dchg_disable_votable, FFC_VOTER, false, 0);
		}
		cancel_delayed_work(&chg->charge_monitor_work);

		chg->charge_monitor_count = 0;

		cancel_delayed_work(&chg->split_current_monitor_work);
		chg->split_current_monitor_status = false;

		vote(chg->pl_disable_votable, USER_CTRL_PL_VOTER, false, 0);
		cancel_delayed_work(&chg->apsd_rerun_work);
		chg->apsd_rerun_status = WORK_STATUS_IDLE;
		chg->apsd_rerun_retries = 0;
		strcpy(chg->usb_type_name,smblib_apsd_results[UNKNOWN].name);
		cancel_delayed_work(&chg->apsd_ctrl_pl_work);
		vote(chg->pl_disable_votable, APSD_CTRL_PL_VOTER, false, 0);

		cancel_delayed_work(&chg->start_hvdcp_charge_monitor_work);
		cancel_delayed_work(&chg->hvdcp_charge_monitor_work);
		chg->hvdcp_charge_monitor_status = WORK_STATUS_IDLE;
		chg->hvdcp_5v_retried = false;
		chg->hvdcp_5v_retries = 0;
		chg->hvdcp_9v_retries = 0;
		chg->chg_abnormal_flag = 0;
		smblib_handle_plugin_removal(chg);
		vote(chg->usb_icl_votable, USB_FIRST_ICL_DONE_VOTER, false, 0);
		vote(chg->fcc_votable, USB_FIRST_ICL_DONE_VOTER, false, 0);
		vote(chg->pl_disable_votable, USER_SET_IBAT_VOTER, false, 0);
		vote(chg->usb_icl_votable, USER_DEFAULT_INPUT_VOTER, false, 0);
		vote(chg->fcc_votable, USER_DEFAULT_IBAT_VOTER, false, 0);

		vote(chg->fcc_votable, USER_SET_IBAT_VOTER, false, 0);
		vote(chg->usb_icl_votable, USER_SET_INPUT_VOTER, false, 0);
		vote(chg->chg_disable_votable, USER_CTRL_CHG_VOTER, false, 0);
		vote(chg->chg_disable_votable, BSP_CTRL_CHG_VOTER, false, 0);
		vote(chg->chg_disable_votable, BQFG_CTRL_CHG_VOTER, false, 0);
		vote(chg->usb_icl_votable, BSP_CTRL_CHG_VOTER, false, 0);
		vote(chg->usb_icl_votable, DCHG_CTRL_CHG_VOTER, false, 0);

		vote(chg->usb_icl_votable, USER_CTRL_PL_VOTER, false, 0);
		vote(chg->pl_disable_votable, PL_STATUS_VOTER, false, 0);
		vote(chg->usb_icl_votable, PL_STATUS_VOTER, false, 0);

		vote(chg->usb_icl_votable, HVDCP_CTRL_ICL_VOTER, false, 0);

		if (chg->dchg_disable_votable)
			vote(chg->dchg_disable_votable, KEEP_CHG_SOC, false, 0);

		if(is_parallel_charger_available(chg)){
			pval.intval = 0;
			power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_PRESENT, &pval);
		}
		chg->user_batt_current_ua = 0;
		chg->user_input_current_ua = 0;
		chg->hvdcp_type = HVDCP_TYPE_UNKNOWN;

		chg->bigdata_chg_voltage_uv = 0;
		chg->bigdata_chg_type = 0;

		chg->recharge_flag = 0;
		chg->charge_full = false;
		chg->ffc_charge_full = false;
		chg->charge_full_force_5v = false;
		chg->charge_full_retries = 0;

		chg->last_change_record_tm = 0;
		chg->tracking_dischg_delta_soc = 0;
		cancel_delayed_work(&chg->usb_psy_voter_clear_work);
		cancel_delayed_work(&chg->weak_charger_voter_clear_work);
		chg->weak_charger_count = 0;

		chg->apsd_rerun_first = false;
		chg->apsd_done_before_usb_present = false;

		cancel_delayed_work(&chg->otg_monitor_work);

		if (chg->usb_conn_pro_flag == 0) {
			smblib_dbg(chg, PR_MISC, "irq usb plugout\n");
			cancel_delayed_work(&chg->usb_conn_pro_monitor_work);
			chg->usb_conn_pro_monitor_count = 0;
			chg->usb_conn_pro_monitor_status = WORK_STATUS_IDLE;
			if (wake_lock_active(&chg->usb_conn_pro_wake_lock)) {
				smblib_dbg(chg, PR_MISC, "usb conn pro unlock-\n");
				wake_unlock(&chg->usb_conn_pro_wake_lock);
			}
		}

		if (chg->cable_with_cc_float) {
			chg->typec_present = 0;
			chg->typec_mode = POWER_SUPPLY_TYPEC_NONE;
		}
		chg->cable_with_cc_float = false;
		chg->cable_with_cc_float_icl_rerun = false;
		chg->cable_with_cc_float_icl_override = false;

		chg->force_charge_full_count = 0;
		chg->force_charge_full_by_high_voltage = false;

		chg->vbus_check_count = 0;

		chg->dchg_status = DIRECT_CHARGER_UNKNOW;
		chg->ffc_pmi_charging_count = 0;
		chg->is_dchg_adapter = false;
		chg->engine = POWER_SUPPLY_ENGINE_NORMAL;
		if (chg->direct_charger_enable) {
			smblib_usbsel_config(chg, 0);

			pval.intval = 0;
			if (chg->bms_psy)
				power_supply_set_property(chg->bms_psy,
				POWER_SUPPLY_PROP_CHARGER_REMOVE, &pval);

			if (chg->mcu_psy)
				power_supply_set_property(chg->mcu_psy,
				POWER_SUPPLY_PROP_CHARGER_REMOVE, &pval);
		}

		if (chg->usbsel_gpio) {
			gpio_direction_output(chg->usbsel_gpio,1);
			smblib_dbg(chg, PR_MISC, "usbsel=%d\n",gpio_get_value(chg->usbsel_gpio));
		}

		/* when cable disconnect in LCO off state, PM suspend quickly and cause the LCD can not power on normal,
		* add 2s delay wake lock for Android LCD control*/
		wake_lock_timeout(&chg->usb_disconnect_wake_lock, 2*HZ);
		if (chg->mode == PARALLEL_MASTER && wake_lock_active(&chg->chg_wake_lock)) {
			smblib_dbg(chg, PR_MISC, "releasing wakelock\n");
			wake_unlock(&chg->chg_wake_lock);
		}
		//fake bat_full clear soft_term_time
		if(chg->soft_term_enable && chg->soft_term_time > 0){
			cancel_delayed_work_sync(&chg->soft_term_timer_work);
			schedule_delayed_work(&chg->soft_term_timer_work, 0);
		}else{
			chg->soft_term_time = 0;
		}
		//ensure that the power_role matches the result of drp_mode_votable
		if (!get_effective_result_locked(chg->drp_mode_votable)) {
			smblib_get_prop_typec_power_role(chg, &pval);
			if (pval.intval == POWER_SUPPLY_TYPEC_PR_DUAL) {
				smblib_dbg(chg, PR_MISC, "usb_det_pin_mode: votable result drp_enable=0. but current in PR_DUAL, so we change it to PR_SINK\n");
				pval.intval = POWER_SUPPLY_TYPEC_PR_SINK_VIVO;
				smblib_set_prop_typec_power_role(chg, &pval);
			}
		}
		if (POWER_SUPPLY_TYPE_UNKNOWN != chg->real_charger_type)
			smblib_update_usb_type(chg);
}

irqreturn_t smblib_handle_aicl_done(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc, settled_ua;

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &settled_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s : %d\n", irq_data->name,settled_ua);

	return IRQ_HANDLED;
}

int smblib_set_batt_current_ua_user(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc = 0;
	int current_ua = val->intval * 1000;

	if(!chg->split_current_monitor_status){
		smblib_dbg(chg, PR_MISC, "apsd or aicl is not ok,next 10s time set current\n");
		return -EAGAIN;
	}

	smblib_dbg(chg, PR_MISC, "###### User setting FCC to %d(ua)\n", current_ua);

	if(current_ua > chg->total_fcc_current_max_ua){
		smblib_dbg(chg, PR_MISC, "User FCC is more than the total_fcc_current_max_ua(%d)\n", chg->total_fcc_current_max_ua);
		current_ua = chg->total_fcc_current_max_ua;
	}

	if (current_ua == 0) {
		smblib_dbg(chg, PR_MISC, "User disable charger\n");
		chg->user_batt_current_ua = 0;
		rc = vote(chg->chg_disable_votable, USER_CTRL_CHG_VOTER, true, 0);
		if (rc < 0)
			pr_err("Couldn't disable battchg, rc %d\n", rc);
		return rc;
	}

	if (!chg->user_batt_current_ua) {
		smblib_dbg(chg, PR_MISC, "User enable charger\n");
		rc = vote(chg->chg_disable_votable, USER_CTRL_CHG_VOTER, false, 0);
		if (rc < 0){
			pr_err("Couldn't enable battchg, rc %d\n", rc);
			return rc;
		}
	}

	rc = vote(chg->fcc_votable, USER_SET_IBAT_VOTER, true, current_ua);
	if (rc < 0){
		smblib_err(chg, "Fcc couldn't vote en rc %d\n", rc);
		return rc;
	}

	chg->user_batt_current_ua = current_ua;
	msleep(1000);

	return 0;
}

int smblib_set_input_current_ua_user(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc = 0;
	int current_ua = val->intval * 1000;
	int icl_ua;

	if (chg->direct_charger_enable) {
		if (chg->dchg_status == DIRECT_CHARGER_PREPARE ||
			chg->dchg_status == DIRECT_CHARGER_IS_CHARGERING) {
			smblib_dbg(chg, PR_MISC, "dchg running,pmi input current not adjusted %d ua\n", current_ua);
			return 0;
		}
	}

	if(!chg->split_current_monitor_status){
		smblib_dbg(chg, PR_MISC, "apsd or aicl is not ok,next 10s time set current\n");
		return -EAGAIN;
	}

	smblib_dbg(chg, PR_MISC, "###### User setting input to %d(ua)\n", current_ua);

	if(current_ua > chg->total_input_current_max_ua){
		smblib_dbg(chg, PR_MISC, "User input is more than the total_input_current_max_ua(%d)\n", chg->total_input_current_max_ua);
		current_ua = chg->total_input_current_max_ua;
	}

	if (chg->user_input_current_ua == current_ua) {
		smblib_dbg(chg, PR_MISC, "user input current set same value,return\n");
		return 0;
	}

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &icl_ua);

	smblib_dbg(chg, PR_MISC, "User setting ICL to %d icl_ua=%d,\n",
										current_ua,icl_ua);

	rc = vote(chg->usb_icl_votable, USER_SET_INPUT_VOTER, true, current_ua);
	if (rc < 0){
		pr_err("Input couldn't vote, rc %d\n", rc);
		return rc;
	}

	chg->user_input_current_ua = current_ua;
	msleep(1000);
	power_supply_changed(chg->batt_psy);
	return 0;
}

int smblib_set_bsp_chg_cutoff(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	bool enable = val->intval;
	union power_supply_propval pval = {0,};

	smblib_dbg(chg, PR_MISC, "bsp cut-off:%d,\n", enable);

	if (enable && is_parallel_charger_available(chg)) {
		pval.intval = 1;
		power_supply_set_property(chg->pl.psy,
						POWER_SUPPLY_PROP_PRESENT, &pval);
		msleep(300);
	}

	vote(chg->chg_disable_votable, BSP_CTRL_CHG_VOTER,
			!enable, 0);
	vote(chg->usb_icl_votable, BSP_CTRL_CHG_VOTER,
		!enable, 0);

	if (!enable && is_parallel_charger_available(chg)) {
		msleep(300);
		pval.intval = 0;
		power_supply_set_property(chg->pl.psy,
						POWER_SUPPLY_PROP_PRESENT, &pval);
	}

	chg->chg_enabled = !!enable;
	power_supply_changed(chg->usb_psy);

	return 0;
}

int smblib_set_hw_charge_timeout(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;
	u8 reg;
	bool enable = val->intval;

	smblib_dbg(chg, PR_MISC, "hw charge timeout:%d,\n", enable);

#if 0
	if (enable == chg->chg_safety_timer_en)
		return 0;
#endif
	if (enable)
		reg = PRE_CHARGE_SAFETY_TIMER_EN_BIT|FAST_CHARGE_SAFETY_TIMER_EN_BIT;
	else
		reg = 0;

	rc = smblib_masked_write(chg, CHGR_SAFETY_TIMER_ENABLE_CFG_REG,
				CHGR_SAFETY_TIMER_ENABLE_CFG_MASK, reg);
	if (rc) {
		pr_err("failed to write CHGR_SAFETY_TIMER_ENABLE_CFG_REG, rc=%d\n", rc);
		return rc;
	}

	chg->chg_safety_timer_en = enable;

	return 0;
}

/*dchg function*/
int smblib_set_dchg_charging(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	bool enable = val->intval;
	u8 aicl_stat = 0;


	if (!chg->usb_present) {
		smblib_dbg(chg, PR_MISC, "try to set dchg charging(%d) when usb absent, skip it!!\n", enable);
		return 0;
	}

	smblib_dbg(chg, PR_MISC, "dchg charging start:%d,\n", enable);
	#if 0
	vote(chg->chg_disable_votable, DCHG_CTRL_CHG_VOTER,
			!enable, 0);
	#endif
	vote(chg->usb_icl_votable, DCHG_CTRL_CHG_VOTER,
		!enable, 0);
	if (enable) {   /* after Enable PMI charging, confirm the PMI quit collapse status. */
		smblib_read(chg, AICL_STATUS_REG, &aicl_stat);
		if (aicl_stat & ICL_IMIN_BIT) {
			smblib_dbg(chg, PR_MISC, "from Dchg -> PMI chg, quit pmi collapse status(0x%02x).\n", aicl_stat);
			smblib_quit_collapse_status(chg);
		}
	}

	smblib_dbg(chg, PR_MISC, "dchg charging end:%d,\n", enable);
	chg->chg_enabled = !!enable;
	//power_supply_changed(chg->usb_psy);

	return 0;
}

static int get_sdp_current(struct smb_charger *chg, int *icl_ua)
{
	int rc;
	u8 icl_options;
	bool usb3 = false;

	rc = smblib_read(chg, USBIN_ICL_OPTIONS_REG, &icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL options rc=%d\n", rc);
		return rc;
	}

	usb3 = (icl_options & CFG_USB3P0_SEL_BIT);

	if (icl_options & USB51_MODE_BIT)
		*icl_ua = usb3 ? USBIN_900MA : USBIN_500MA;
	else
		*icl_ua = usb3 ? USBIN_150MA : USBIN_100MA;

	return rc;
}

int smblib_get_prop_from_qcom_fg(struct smb_charger *chg,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int rc;
	struct power_supply		*qc_bms_psy;

	qc_bms_psy = power_supply_get_by_name("bms");

	if (!qc_bms_psy) {
		smblib_err(chg, "no qcom bms psy found\n");
		return -EINVAL;
	}

	rc = power_supply_get_property(qc_bms_psy, psp, val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get qcom bms psp %d,rc=%d\n", psp, rc);
		return -EINVAL;
	}
	return rc;
}

int smblib_get_prop_batt_capacity_mah(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc = 0;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, val);
	if(val->intval <= 0){
		smblib_err(chg, "Couldn't get batt_capacity_mah,use default mah\n");
		val->intval = DEFAULT_BATT_CAPACITY_MAH;
	}

	return rc;
}

int smblib_get_prop_batt_voltage_ocv(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_OCV, val);
	return rc;
}

#define DEFAULT_CHG_EST_VBAT_DIFF	0
int smblib_get_prop_batt_chg_est_vbat_diff(struct smb_charger *chg)
{
	union power_supply_propval val = {0, };
	int temp, rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				   POWER_SUPPLY_PROP_CHG_EST_VBAT_DIFF, &val);
	temp = val.intval;
	if (rc) {
		smblib_dbg(chg, PR_MISC, "Couldn't get chg est vbatdiff rc = %d, use default value\n", rc);
		temp = DEFAULT_CHG_EST_VBAT_DIFF;
	}
	return temp;
}

#define DEFAULT_BOARD_TEMP  250
int smblib_get_primary_board_temp(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc = 0;
	int board_temp = 0;

	rc = smblib_read_iio_channel(chg, chg->iio.board_temp_chan,
				DIV_FACTOR_DECIDEGC, &board_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SKIN TEMP channel, rc=%d, use default temp\n", rc);
		val->intval = DEFAULT_BOARD_TEMP;
		return rc;
	}
	val->intval = board_temp;
	pr_err("board_temp=%d\n", board_temp);
	return 0;
}

int smblib_get_parallel_board_temp(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc = 0;
	int smb_temp = 0;
	struct power_supply *mcu_psy;

#if 0
	#if 0
	/* now battery NTC and parallel NTC are common. */
	union power_supply_propval batt_temp = {0,};	
	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_TEMP, &batt_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BAT TEMP channel, rc=%d, use default temp\n", rc);
		val->intval = DEFAULT_BOARD_TEMP;
		return rc;
	}
	val->intval = batt_temp.intval;
	#else
	val->intval = -40;
	return 0;
	rc = smblib_read_iio_channel(chg, chg->iio.smb_temp_chan,
				DIV_FACTOR_DECIDEGC, &smb_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SKIN TEMP channel, rc=%d, use default temp\n", rc);
		val->intval = DEFAULT_BOARD_TEMP;
		return rc;
	}
	val->intval = smb_temp;
	pr_info("smb_temp=%d\n", smb_temp);
	#endif
#else 
	if (chg->direct_charger_enable) {
		mcu_psy = get_mcu_psy(chg);
		if (!mcu_psy) {
			val->intval = DEFAULT_BOARD_TEMP;
			smblib_err(chg, "mcu_psy not found\n");
			return 0;
		}

		rc = power_supply_get_property(mcu_psy,
				POWER_SUPPLY_PROP_MASTER_BAT_CONN_TEMP, val);
		if (rc < 0) {
			smblib_dbg(chg, PR_MISC, "Couldn't get master bat conn temp fail rc = %d, use default value\n", rc);
			val->intval = DEFAULT_BOARD_TEMP;
		}
	} else {
		val->intval = -40;
		rc = smblib_read_iio_channel(chg, chg->iio.smb_temp_chan,
					DIV_FACTOR_DECIDEGC, &smb_temp);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read SKIN TEMP channel, rc=%d, use default temp\n", rc);
			val->intval = DEFAULT_BOARD_TEMP;
			return rc;
		}
		val->intval = smb_temp;
		pr_info("smb_temp=%d\n", smb_temp);
	}

	pr_err("parallel_board_temp=%d\n", val->intval);
#endif
	return rc;
}

#define DEFAULT_USB_CONN_TEMP	250
int smblib_get_usb_conn_pro_temp(struct smb_charger *chg,
		union power_supply_propval *val)
{
	int rc = 0;
	struct power_supply *mcu_psy;
	union power_supply_propval usb_conn_temp = {0,};

	if (chg->direct_charger_enable) {
		mcu_psy = get_mcu_psy(chg);
		if (!mcu_psy) {
			val->intval = DEFAULT_USB_CONN_TEMP;
			chg->connector_temp = DEFAULT_USB_CONN_TEMP;
			smblib_err(chg, "mcu_psy not found\n");
			return 0;
		}

		rc = power_supply_get_property(mcu_psy,
			POWER_SUPPLY_PROP_USB_CONN_TEMP, &usb_conn_temp);
		if (rc < 0) {
			smblib_dbg(chg, PR_MISC, "Couldn't get usb conn temp fail rc = %d, use default value\n", rc);
			val->intval = DEFAULT_USB_CONN_TEMP;
			chg->connector_temp = DEFAULT_USB_CONN_TEMP;
		} else {
			val->intval = usb_conn_temp.intval;
			chg->connector_temp = usb_conn_temp.intval;
		}
	} else {
		rc = smblib_read_iio_channel(chg, chg->iio.connector_temp_chan,
					DIV_FACTOR_DECIDEGC, &chg->connector_temp);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read SKIN TEMP channel, rc=%d, use default temp\n", rc);
			val->intval = DEFAULT_BOARD_TEMP;
			return rc;
		}
		val->intval = chg->connector_temp;
	}

	pr_err("connector_temp=%d\n", chg->connector_temp);
	return 0;
}

int smblib_get_bat_board_temp(struct smb_charger *chg,
		union power_supply_propval *val)
{
	int rc = 0;
	int bat_board_temp = 0;
	struct power_supply *mcu_psy;

	if (chg->direct_charger_enable) {
		mcu_psy = get_mcu_psy(chg);
		if (!mcu_psy) {
			val->intval = DEFAULT_BOARD_TEMP;
			smblib_err(chg, "mcu_psy not found\n");
			return 0;
		}

		rc = power_supply_get_property(mcu_psy,
				POWER_SUPPLY_PROP_BAT_BOARD_TEMP, val);
		if (rc < 0) {
			smblib_dbg(chg, PR_MISC, "Couldn't get bat board temp fail rc = %d, use default value\n", rc);
			val->intval = DEFAULT_BOARD_TEMP;
		}
	} else {
		rc = smblib_read_iio_channel(chg, chg->iio.bat_board_temp_chan,
								DIV_FACTOR_DECIDEGC, &bat_board_temp);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read BAT_BOARD TEMP channel, rc=%d, use default temp\n", rc);
			val->intval = DEFAULT_BOARD_TEMP;
		} else {
			val->intval = bat_board_temp;
		}
	}

	pr_err("bat_board_temp=%d\n", val->intval);
	return 0;
}

/*common function*/
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}
static int calculate_delta_time(unsigned long *time_stamp, int *delta_time_s)
{
	unsigned long now_tm_sec = 0;

	/* default to delta time = 0 if anything fails */
	*delta_time_s = 0;

	if (get_current_time(&now_tm_sec)) {
		pr_err("RTC read failed\n");
		return 0;
	}

	*delta_time_s = (now_tm_sec - *time_stamp);

	/* remember this time */
	*time_stamp = now_tm_sec;
	return 0;
}

static int calculate_delta_qg_count(int *last_qg_cnt, int *cur_qg_cnt, int *delta_qg_cnt)
{
	*delta_qg_cnt = *cur_qg_cnt >= *last_qg_cnt ? (*cur_qg_cnt - *last_qg_cnt) : (3600 - *last_qg_cnt + *cur_qg_cnt);
	return 0;
}
/* add for vivo super FlashCharge */
int smblib_get_prop_batt_fcc(struct smb_charger *chg, union power_supply_propval *val)
{
	int rc = 0;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy, POWER_SUPPLY_PROP_FCC, val);
	if (rc < 0) {
		rc = smblib_get_prop_batt_capacity_mah(chg, val);
		if (rc < 0)
			smblib_err(chg, "Couldn't get capacity_mah rc=%d,used default mah\n", rc);
	}

	return rc;
}
int smblib_get_prop_batt_engine(struct smb_charger *chg, union power_supply_propval *val)
{
	int rc = 0;
	union power_supply_propval health = {0,};
	int engine = POWER_SUPPLY_ENGINE_NORMAL;

	rc = smblib_get_prop_batt_health(chg, &health);
	if (rc < 0) {
		val->intval = 0;
		smblib_err(chg, "Couldn't get health rc=%d\n", rc);
		return rc;
	}

	if (!chg->direct_charger_enable &&
			chg->hvdcp_type >= HVDCP_TYPE_9V &&
			health.intval == POWER_SUPPLY_HEALTH_GOOD) {
		engine = POWER_SUPPLY_ENGINE_DUAL;
		pr_err("dual engine chg\n");
	} else if (chg->direct_charger_enable &&
			smblib_get_handshake_status(chg) &&
			health.intval == POWER_SUPPLY_HEALTH_GOOD) {
		if (smblib_get_handshake_status(chg) == VIVO_FLASH_CHARGE_STATUS_SUPER)
			engine = POWER_SUPPLY_ENGINE_SUPER;
		else
			engine = POWER_SUPPLY_ENGINE_DUAL;

		chg->is_dchg_adapter = true;
		pr_err("direct engine chg %d \n", engine);
	}

	val->intval = chg->engine = engine;
	return rc;
}

int smblib_get_prop_batt_soc_decimal(struct smb_charger *chg, union power_supply_propval *val)
{
	int rc = 0, decimal = 0;
	union power_supply_propval mah = {0,};
	union power_supply_propval coul = {0,};
	union power_supply_propval batt_soc = {0,};

	#if 1
	if (chg->engine != POWER_SUPPLY_ENGINE_SUPER) {
		val->intval = 0;
		smblib_err(chg, "It's not super flashcharge\n");
		return rc;
	}
	#endif
	rc = smblib_get_prop_batt_fcc(chg, &mah);
	if (rc < 0) {
		val->intval = 0;
		smblib_err(chg, "Couldn't get fcc rc=%d\n", rc);
		return rc;
	}

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CHARGE_COUNTER, &coul);
	coul.intval = coul.intval /1000;
	rc = smblib_get_prop_batt_capacity(chg,  &batt_soc);

	/*add for ui soc start */
	if(chg->soc_decimal_from_user > 0) {
		chg->last_vbus_change_sec = chg->last_vbus_change_record_tm;
		if(!chg->last_vbus_change_sec) {
			  get_current_time(&chg->last_vbus_change_record_tm);
			  chg->last_vbus_change_sec = chg->last_vbus_change_record_tm;
		}
		calculate_delta_time(&chg->last_vbus_change_sec, &chg->vbus_change_delta_time_s);
	}
	/* add for ui soc end */

	if(batt_soc.intval == 100) {
		chg->soc_decimal = 10000;
	}else if(chg->battery_capacity_is_exception || batt_soc.intval  >=98 ||batt_soc.intval  <= 3) {
		chg->soc_decimal = batt_soc.intval * 100 + ((uint)jiffies % 50);
	}else if(chg->vbus_change_delta_time_s > 8 || chg->soc_decimal_from_user <= 0) {
		if (mah.intval > 0)
			decimal = ((coul.intval * 10000) / mah.intval) % 100;
		else
			decimal = ((uint)jiffies % 50);
		chg->soc_decimal = (batt_soc.intval * 100) + decimal;
	} else if(chg->soc_decimal_from_user > 0) {
		chg->soc_decimal = chg->soc_decimal_from_user;
	}else {
		chg->soc_decimal = batt_soc.intval * 100;
	}
	val->intval = chg->soc_decimal;
	val->intval = max(0, val->intval);
	val->intval = min(10000, val->intval);
	smblib_err(chg,"coul[%d], batt_soc=%d, decimal[%d], "
			"soc_decimal[%d,%d],mah=%d,battery_capacity_is_exception=%d,vbus_change_delta_time_s=%d\n",
			coul.intval, batt_soc.intval, decimal, chg->soc_decimal,
			chg->soc_decimal_from_user, mah, chg->battery_capacity_is_exception, chg->vbus_change_delta_time_s);

	return rc;
}

int smblib_get_prop_batt_charging_speed(struct smb_charger *chg, union power_supply_propval *val)
{
	int rc = 0, decimal = 0,step = POWER_SUPPLY_CHARGING_SPEED_NORMAL;
	union power_supply_propval tbat = {0,};
	union power_supply_propval batt_soc = {0,};
	#if 1
	if (chg->engine != POWER_SUPPLY_ENGINE_SUPER) {
		val->intval = POWER_SUPPLY_CHARGING_SPEED_NORMAL;
		smblib_err(chg, "It's not super flashcharge\n");
		return rc;
	}
	#endif
	rc = smblib_get_prop_from_qcom_fg(chg, POWER_SUPPLY_PROP_TEMP, &tbat);
	rc = smblib_get_prop_batt_capacity(chg,  &batt_soc);

	if (tbat.intval >= 150 && tbat.intval <= 450) {
		if (batt_soc.intval < 80)
			step = POWER_SUPPLY_CHARGING_SPEED_4GEAR;
		else if (batt_soc.intval < 90)
			step = POWER_SUPPLY_CHARGING_SPEED_3GEAR;
		else
			step = POWER_SUPPLY_CHARGING_SPEED_1GEAR;
	} else if (tbat.intval >= 100 && tbat.intval < 150) {
		if (batt_soc.intval < 80)
			step = POWER_SUPPLY_CHARGING_SPEED_3GEAR;
		else if (batt_soc.intval < 90)
			step = POWER_SUPPLY_CHARGING_SPEED_2GEAR;
		else
			step = POWER_SUPPLY_CHARGING_SPEED_1GEAR;
	} else {
		step = POWER_SUPPLY_CHARGING_SPEED_1GEAR;
	}

	val->intval = step;
	smblib_err(chg,"tbat=%d, batt_soc=%d, decimal=%d, step=%d\n",
			tbat.intval, batt_soc.intval, decimal, step);
	return rc;
}

int smblib_set_prop_batt_soc_decimal(struct smb_charger *chg, int soc_decimal)
{
	int rc = 0;
	u8 ui_soc = 0;
	#if 1
	if (chg->engine != POWER_SUPPLY_ENGINE_SUPER) {
		smblib_err(chg, "It's not super flashcharge\n");
		return rc;
	}
	if (!fg_soc_really) {
		smblib_err(chg, "fg_soc_really=%d, skip..\n", fg_soc_really);
		return rc;
	}
	#endif

	/*add for ui soc start */
	chg->last_vbus_change_record_tm = 0;
	chg->last_vbus_change_sec = chg->last_vbus_change_record_tm;
	if(!chg->last_vbus_change_sec) {
		  get_current_time(&chg->last_vbus_change_record_tm);
		  chg->last_vbus_change_sec = chg->last_vbus_change_record_tm;
	}
	/* add for ui soc end */
	chg->soc_decimal_from_user = soc_decimal;
	ui_soc = (u8) (chg->soc_decimal_from_user /100);
	smblib_err(chg, "super flashcharge result soc_decimal:%d,ui_soc=%d,chg.ui_soc=%d\n", soc_decimal, ui_soc, chg->ui_soc);

	if(chg->ui_soc < 0)
		chg->ui_soc = ui_soc;

	if(abs(ui_soc - chg->ui_soc) >= 5) {
		chg->soc_decimal_from_user = chg->ui_soc * 100;
		ui_soc = chg->ui_soc;
	}

	if(chg->ui_soc != ui_soc) {
		chg->ui_soc = ui_soc;
		if(chg->batt_psy)
			power_supply_changed(chg->batt_psy);
	}

	return rc;
}

int smblib_disable_chg_by_fg(struct smb_charger *chg, const union power_supply_propval *val)
{
	bool enable = val->intval;
	union power_supply_propval pval = {0,};

	pr_err("enable=%d\n", enable);
	if (!chg->dchg_disable_votable)
		chg->dchg_disable_votable = find_votable("DCHG_DISABLE");

	if (enable) {
		chg->fg_disable_charging = true;
		if (chg->dchg_status == DIRECT_CHARGER_PREPARE
			|| chg->dchg_status == DIRECT_CHARGER_IS_CHARGERING) {
			if (chg->mcu_psy)
				power_supply_set_property(chg->mcu_psy,
							POWER_SUPPLY_PROP_DCHG_CALL_START, &pval);

			if(chg->dchg_disable_votable)
				vote(chg->dchg_disable_votable, FG_CTRL_DCHG_VOTER, true, 0);
			msleep(100);
			vote(chg->usb_icl_votable, DCHG_CTRL_CHG_VOTER, !enable, 0);
		}
	       vote(chg->chg_disable_votable, FG_CTRL_SCHG_VOTER, enable, 0);
	} else {
		//vote(chg->usb_icl_votable, DCHG_CTRL_CHG_VOTER,
		//!enable, 0);
		vote(chg->chg_disable_votable, FG_CTRL_SCHG_VOTER,
				enable, 0);
		if(chg->dchg_disable_votable)
			vote(chg->dchg_disable_votable, FG_CTRL_DCHG_VOTER, false, 0);
		chg->fg_disable_charging = false;
	}

	power_supply_changed(chg->usb_psy);

	return 0;
}

static int vivo_battery_handle_ui_soc(struct smb_charger *chg, int soc)
{
	union power_supply_propval chg_status = {0,};
	int  soc_smooth_time_limit = 30; //30s
	int rc = 0;

	mutex_lock(&chg->ui_soc_lock);
	if(chg->ui_soc <= 0)
		chg->ui_soc = soc;

	rc = power_supply_get_property(chg->batt_psy,
			POWER_SUPPLY_PROP_STATUS_EX, &chg_status);
	if (rc) {
		pr_err("couldn't get chg_status,rc=%d\n", rc);
		chg->ui_soc = soc;
		chg->soc_refer = chg->ui_soc;
		chg->last_change_record_tm = 0;
		mutex_unlock(&chg->ui_soc_lock);
		return chg->ui_soc;
	}

	if(chg->ui_soc != soc) {
		//count current time and delta time
		chg->last_ui_soc_change_sec = chg->last_change_record_tm;
		if(!chg->last_ui_soc_change_sec) {
			  get_current_time(&chg->last_change_record_tm);
			  chg->last_ui_soc_change_sec = chg->last_change_record_tm;
		}
		calculate_delta_time(&chg->last_ui_soc_change_sec, &chg->ui_soc_delta_time_s);

		if(chg_status.intval == POWER_SUPPLY_STATUS_FULL) {
			chg->last_change_record_tm = 0;
			if(chg->charge_monitor_count >= 2 && chg->ui_soc <= soc)
				chg->ui_soc = soc;
		}else if(chg_status.intval == POWER_SUPPLY_STATUS_CHARGING) {
			if (chg->ui_soc > soc && 0 == soc) {
				if (chg->ui_soc_delta_time_s >= soc_smooth_time_limit) {
					chg->ui_soc = chg->ui_soc - 1;
					get_current_time(&chg->last_change_record_tm);
				}
			} else {
				chg->last_change_record_tm = 0;
				if(chg->ui_soc <= soc && chg->charge_monitor_count >= 2)
					chg->ui_soc = soc;
				else if((chg->ui_soc - soc) >= 3)
					chg->ui_soc = chg->ui_soc - 1;
				else
					pr_info("stay last ui_soc\n");
			}
		}else {
				pr_warn("ui_soc_delta_time_s=%d,soc_smooth_time_limit=%d\n",
						chg->ui_soc_delta_time_s,soc_smooth_time_limit);
				if (chg->ui_soc_delta_time_s >= soc_smooth_time_limit && chg->ui_soc > soc) {
					chg->ui_soc = chg->ui_soc - 1;
					get_current_time(&chg->last_change_record_tm);
				}
		}
	}else{
		chg->last_change_record_tm = 0;
	}
	if(abs(chg->ui_soc -soc) >= 2)
		chg->battery_capacity_is_exception = true;
	else
		chg->battery_capacity_is_exception = false;
	chg->soc_refer = chg->ui_soc;
	pr_info("ui_soc_delta_time_s=%d,chg_status=%d,chg.ui_soc=%d,soc=%d,soc_decimal_from_user=%d\n",
			chg->ui_soc_delta_time_s, chg_status.intval, chg->ui_soc, soc, chg->soc_decimal_from_user);
	mutex_unlock(&chg->ui_soc_lock);
	return chg->ui_soc;
}
/* FlashCharge end */

static int smblib_tracking_soc(struct smb_charger *chg, int soc, bool empty_vbat)
{
	static bool is_charging_change = false;
	static bool is_discharging_change;
	static int high_current_cnt;
	union power_supply_propval chg_status = {0, };
	union power_supply_propval param_status = {0, };
	union power_supply_propval qg_count = {0, };
	union power_supply_propval curr_ua = {0, };
	union power_supply_propval vbatt_uv = {0, };
	union power_supply_propval bat_temp = {0, };
	int rc = 0, raw_ui_soc, dischg_update_case;

	rc = smblib_get_prop_from_qcom_fg(chg, POWER_SUPPLY_PROP_BATTERY_PARAM_STATUS, &param_status);
	if (rc) {
		pr_err("Couldn't get param_status, rc = %d\n", rc);
		return soc;
	}

	if (!param_status.intval) {
		pr_err("batt param is not exist!\n");
		return soc;
	}

	if (param_status.intval && chg->ui_soc < 0) {
		chg->ui_soc = soc;
		smblib_dbg(chg, PR_MISC, "init ui_soc=%d\n", chg->ui_soc);
		return chg->ui_soc;
	}

	mutex_lock(&chg->ui_soc_lock);
	raw_ui_soc = chg->ui_soc;/* record the initial ui_soc at entry */
	dischg_update_case = 0;
	smblib_dbg(chg, PR_MISC, "initial: ui_soc=%d, soc=%d, raw_ui_soc=%d\n",
			chg->ui_soc, soc, raw_ui_soc);

	rc = power_supply_get_property(chg->batt_psy,
			POWER_SUPPLY_PROP_STATUS_EX, &chg_status);
	if (rc) {
		pr_err("couldn't get chg_status,rc=%d\n", rc);
		chg->ui_soc = soc;
		mutex_unlock(&chg->ui_soc_lock);
		return chg->ui_soc;
	}

	/*handle charging ui_soc*/
	if (chg_status.intval == POWER_SUPPLY_STATUS_CHARGING ||
		chg_status.intval == POWER_SUPPLY_STATUS_FULL) {
		if (chg->ui_soc < soc) {
			chg->last_ui_soc_change_sec = chg->last_change_record_tm;
			if (!chg->last_ui_soc_change_sec) {
				  get_current_time(&chg->last_change_record_tm);
				  chg->last_ui_soc_change_sec = chg->last_change_record_tm;
			}

			calculate_delta_time(&chg->last_ui_soc_change_sec, &chg->ui_soc_delta_time_s);

			/*ui soc ++ when (x)s  timeout*/
			smblib_dbg(chg, PR_MISC, "ui_soc_delta_time_s=%d,tracking_step_second=%d\n",
						chg->ui_soc_delta_time_s, chg->tracking_step_second);
			if (chg->ui_soc_delta_time_s >= chg->tracking_step_second) {
				chg->ui_soc++;
				get_current_time(&chg->last_change_record_tm);
			}
			is_charging_change = true;
		} else {
			chg->last_change_record_tm = 0;
			is_charging_change = false;
			if (!is_discharging_change)
			chg->ui_soc = soc;
			else if (raw_ui_soc - soc > chg->tracking_dischg_delta_soc + 2) /* soc difference gets larger */
				chg->chg_less_than_consume = true;
			if (chg->chg_less_than_consume && raw_ui_soc > soc) {
				chg->last_ui_soc_change_sec = chg->last_chg_drop_change_record_tm;
				if (!chg->last_ui_soc_change_sec) {
					get_current_time(&chg->last_chg_drop_change_record_tm);
					chg->last_ui_soc_change_sec = chg->last_chg_drop_change_record_tm;
				}
				calculate_delta_time(&chg->last_ui_soc_change_sec, &chg->ui_soc_delta_time_s);
				if (chg->ui_soc_delta_time_s >= 60) {
					chg->ui_soc = raw_ui_soc - 1;
					get_current_time(&chg->last_chg_drop_change_record_tm);
				}
				smblib_dbg(chg, PR_MISC, "charging current less than comsumption, "
							"raw_ui_soc=%d, ui_soc=%d, soc=%d\n",
							raw_ui_soc, chg->ui_soc, soc);
			} else {
				chg->chg_less_than_consume = false;
			}
		}
	} else {
		chg->last_change_record_tm = 0;

		if (is_charging_change && chg->ui_soc < soc) {
			smblib_dbg(chg, PR_MISC, "last time,in charing change state,stay ui_soc\n");
		} else {
			if (!chg->tracking_dischg_enable)
			chg->ui_soc = soc;
			is_charging_change = false;
		}
	}

	if (chg->tracking_dischg_enable) {
		if (chg_status.intval == POWER_SUPPLY_STATUS_DISCHARGING ||
			chg_status.intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			chg->chg_less_than_consume = false;
			rc = smblib_get_prop_from_qcom_fg(chg, POWER_SUPPLY_PROP_QG_COUNT, &qg_count);
			if (rc) {
				pr_err("Couldn't get qg_count, rc = %d\n", rc);
				mutex_unlock(&chg->ui_soc_lock);
				return soc;
			}
			rc = smblib_get_prop_from_qcom_fg(chg, POWER_SUPPLY_PROP_CURRENT_NOW, &curr_ua);
			if (rc) {
				pr_err("Couldn't get current_now, rc = %d\n", rc);
				mutex_unlock(&chg->ui_soc_lock);
				return soc;
			}
			rc = smblib_get_prop_from_qcom_fg(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbatt_uv);
			if (rc) {
				pr_err("Couldn't get vbatt, rc = %d\n", rc);
				mutex_unlock(&chg->ui_soc_lock);
				return soc;
			}
			rc = smblib_get_prop_from_qcom_fg(chg, POWER_SUPPLY_PROP_TEMP, &bat_temp);
			if (rc) {
				pr_err("Couldn't get bat temp, rc = %d\n", rc);
				mutex_unlock(&chg->ui_soc_lock);
				return soc;
			}
			if (curr_ua.intval > chg->tracking_dischg_current_threshold)
				high_current_cnt++;
			else
				high_current_cnt--;
			high_current_cnt = max(0, high_current_cnt);
			if (raw_ui_soc > soc) {
				/* soc drop condition */
				chg->last_ui_soc_change_sec = chg->last_dischg_change_record_tm;
				if (!chg->last_ui_soc_change_sec) {
					get_current_time(&chg->last_dischg_change_record_tm);
					chg->last_ui_soc_change_sec = chg->last_dischg_change_record_tm;
				}
				calculate_delta_time(&chg->last_ui_soc_change_sec, &chg->ui_soc_delta_time_s);
				if (chg->last_qg_monitor_work_count < 0) {
					chg->last_qg_monitor_work_count = qg_count.intval;
				}
				calculate_delta_qg_count(&chg->last_qg_monitor_work_count, &qg_count.intval, &chg->ui_soc_delta_qg_cnt);
				if ((chg->ui_soc_delta_time_s >= chg->tracking_dischg_step_second ||
					chg->ui_soc_delta_qg_cnt >= chg->tracking_dischg_step_cnt ||
					high_current_cnt >= 300) &&
					bat_temp.intval > 0 &&
					vbatt_uv.intval > chg->tracking_dischg_volt_threshold) {
					chg->ui_soc = raw_ui_soc - 1;
					get_current_time(&chg->last_dischg_change_record_tm);
					chg->last_qg_monitor_work_count = qg_count.intval;
					high_current_cnt = 0;
					dischg_update_case = 1;
				} else if (vbatt_uv.intval <= chg->tracking_dischg_volt_threshold &&
					chg->ui_soc_delta_time_s >= chg->tracking_dischg_low_volt_step_second &&
					bat_temp.intval > 0) {
					chg->ui_soc = raw_ui_soc - 1;
					get_current_time(&chg->last_dischg_change_record_tm);
					chg->last_qg_monitor_work_count = qg_count.intval;
					high_current_cnt = 0;
					dischg_update_case = 2;
				} else if ((empty_vbat || bat_temp.intval <= 0) && chg->ui_soc_delta_time_s >= 10) {
					chg->ui_soc = raw_ui_soc - 1;
					get_current_time(&chg->last_dischg_change_record_tm);
					chg->last_qg_monitor_work_count = qg_count.intval;
					high_current_cnt = 0;
					dischg_update_case = 3;
				} else {
					chg->ui_soc = raw_ui_soc;
				}
				is_discharging_change = true;
				chg->tracking_dischg_delta_soc = raw_ui_soc - soc;
			} else {
				chg->ui_soc_delta_qg_cnt = 0;
				chg->ui_soc_delta_time_s = 0;
				is_discharging_change = false;
				if (!is_charging_change)
					chg->ui_soc = soc;
			}
			smblib_dbg(chg, PR_MISC, "dischg:raw_ui_soc=%d,soc=%d,delta_qg_count=%d[%d,%d],delta_time_s=%d[%d], high_cur_cnt=%d\n",
							raw_ui_soc, soc,
							chg->ui_soc_delta_qg_cnt, chg->last_qg_monitor_work_count, qg_count.intval,
							chg->ui_soc_delta_time_s, chg->last_ui_soc_change_sec, high_current_cnt);
		} else {
			chg->last_dischg_change_record_tm = 0;
			chg->last_qg_monitor_work_count = -1;
			high_current_cnt = 0;
			if (is_discharging_change && raw_ui_soc > soc)
				smblib_dbg(chg, PR_MISC, "discharging time not enough, stay ui_soc, dischg_delta_soc=%d\n",
						chg->tracking_dischg_delta_soc);
			else {
				is_discharging_change = false;
				chg->tracking_dischg_delta_soc = 0;
			}
		}
	}
	smblib_dbg(chg, PR_MISC, "ui_soc=%d,soc=%d,is_charging_change=%d,is_discharging_change=%d,chg_status=%d,dischg_update_case=%d\n",
					chg->ui_soc, soc, is_charging_change, is_discharging_change, chg_status.intval, dischg_update_case);

	mutex_unlock(&chg->ui_soc_lock);

	return chg->ui_soc;
}

static int smblib_get_hvdcp_type(struct smb_charger *chg)
{
	union power_supply_propval usbin_uv = {0,};
	int hvdcp_type;
	int index,rc;

	hvdcp_type = HVDCP_TYPE_UNKNOWN;

	rc = smblib_get_prop_usb_voltage_now(chg, &usbin_uv);
	if (rc < 0){
		pr_err("read usbin err rc=%d\n",rc);
		return 0;
	}
	chg->bigdata_chg_voltage_uv = usbin_uv.intval;
	for (index = ARRAY_SIZE(hvdcp_vol)-1;index >= 0 ;index--) {
		if (is_between(hvdcp_vol[index].min_uv,hvdcp_vol[index].max_uv,usbin_uv.intval)) {
			smblib_dbg(chg, PR_MISC, "vbus_uv=%d min_uv=%d,max_uv=%d\n",
					usbin_uv.intval,hvdcp_vol[index].min_uv,hvdcp_vol[index].max_uv);
			hvdcp_type = hvdcp_vol[index].type;
		}
	}

	return hvdcp_type;
}

int smblib_reset_inhibit_status(struct smb_charger *chg)
{
	int rc;
	u8 stat = 0;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	smblib_dbg(chg, PR_MISC, "before: 0x%02x = 0x%02x\n",BATTERY_CHARGER_STATUS_1_REG,stat);

	if((stat&BATTERY_CHARGER_STATUS_MASK) == 0x6){//detect inhibit status
		rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				CHARGING_ENABLE_CMD_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable charge rc=%d\n", rc);
			return rc;
		}
		msleep(100);
		rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				CHARGING_ENABLE_CMD_BIT, CHARGING_ENABLE_CMD_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable charge rc=%d\n", rc);
			return rc;
		}
		chg->charge_full = false;
	}
	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	smblib_dbg(chg, PR_MISC, "after: 0x%02x = 0x%02x\n",BATTERY_CHARGER_STATUS_1_REG,stat);

	return rc;
}

static void smblib_soft_term_timer_work(struct work_struct *work)
{
	int work_period_ms = msecs_to_jiffies(5000);
	union power_supply_propval batt_soc;
	struct smb_charger *chg = container_of(work,
					struct smb_charger,
					soft_term_timer_work.work);

	smblib_get_prop_batt_capacity(chg, &batt_soc);

	if(chg->soft_term_time > 0)
		chg->soft_term_time--;
	smblib_dbg(chg, PR_MISC, "!!! discharging !!! %d \n",chg->soft_term_time);

	if(batt_soc.intval >= chg->soft_term_capacity_limit && chg->soft_term_time > 0){
		schedule_delayed_work(&chg->soft_term_timer_work,
			round_jiffies_relative(work_period_ms));
	}else if(batt_soc.intval < chg->soft_term_capacity_limit){
		chg->soft_term_time = 0;
	}
}

static int smblib_termination_by_soft(struct smb_charger *chg, int chg_status)
{
	union power_supply_propval batt_soc;

	if(!chg->soft_term_enable){
		smblib_dbg(chg, PR_MISC, "soft charging terminate isn't enabled\n");
		return 0;
	}

	if(chg_status == POWER_SUPPLY_STATUS_CHARGING){
		smblib_get_prop_batt_capacity(chg, &batt_soc);
		if(batt_soc.intval >= chg->soft_term_capacity_limit){
			cancel_delayed_work_sync(&chg->soft_term_timer_work);
			smblib_dbg(chg, PR_MISC, "!!! charging !!! %d \n",chg->soft_term_time);
			if(chg->soft_term_time++ >= chg->soft_term_timer_limit){
				chg->soft_term_time--;
				smblib_dbg(chg, PR_MISC, "charging terminate(not hardware irq)\n");
				//msleep(5000);
				chg->charge_full = true;
				power_supply_changed(chg->batt_psy);
			}
			smblib_dbg(chg, PR_MISC, "soft_term_capacity_limit %d, soft_term_timer_limit %d \n",
							chg->soft_term_capacity_limit,chg->soft_term_timer_limit);
		}
	}else{
	}

	return 0;
}

static int smblib_recharge_by_soft(struct smb_charger *chg,int chg_status)
{
	int rc;
	union power_supply_propval charge_done = {0,};

	if (!chg->direct_charger_enable)
		return 0;

	if (!chg->recharge_flag) {
		return 0;
	}
	smblib_get_prop_batt_charge_done(chg, &charge_done);

	if(chg_status == POWER_SUPPLY_STATUS_FULL && charge_done.intval){
		smblib_dbg(chg, PR_MISC,"recharger now !(not hardware irq)\n");
		rc = vote(chg->chg_disable_votable, BQFG_CTRL_CHG_VOTER, true, 0);
		if (rc < 0) {
			smblib_dbg(chg, PR_MISC,"Couldn't disable battchg, rc %d\n", rc);
		}
		msleep(3000);
		rc = vote(chg->chg_disable_votable, BQFG_CTRL_CHG_VOTER, false, 0);
		if (rc < 0) {
			smblib_dbg(chg, PR_MISC,"Couldn't enable battery charging=%d\n", rc);
		}
		power_supply_changed(chg->batt_psy);
	}else{
		smblib_dbg(chg, PR_MISC,"not recharger\n");
	}

	return 0;
}
/*
static int smblib_charge_full_force_5v(struct smb_charger *chg, int chg_status, int chg_volt_v)
{
	if(chg_status == POWER_SUPPLY_STATUS_FULL) {
		if (chg_volt_v > 5) {
			smblib_dbg(chg, PR_MISC,"input adjust to 5v\n");
			chg->charge_full_force_5v = true;
			chg->charge_full_retries = 0;
			msleep(300);
			force_5v_hvdcp(chg);
		}else{

		}
	}else {
		if (chg->charge_full_force_5v && chg_volt_v <= 5) {
			if(chg->charge_full_retries < 6) {
				chg->charge_full_retries++;
				smblib_dbg(chg, PR_MISC,"input adjust to 9v,retries=%d\n",chg->charge_full_retries);
				force_9v_hvdcp(chg);
			}
		}else {
			smblib_dbg(chg, PR_MISC,"input adjust is not need\n");
			chg->charge_full_force_5v = false;
			chg->charge_full_retries = 0;
		}
	}

	return 0;
}
*/
/*
static void smblib_check_pl_abnormal_status(struct smb_charger *chg)
{
	union power_supply_propval pval = {0, };

	if(is_parallel_charger_available(chg)){
		power_supply_get_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CHECK_SLAVE_CHARGER_STATUS, &pval);

		switch(pval.intval){
			case PL_STATUS_OK:
				smblib_dbg(chg, PR_MISC, "pl status ok\n");
				chg->pl_status = PL_STATUS_OK;
				break;
			case PL_STATUS_I2C_ERROR:
				smblib_dbg(chg, PR_MISC, "pl i2c error\n");
				chg->pl_status = PL_STATUS_I2C_ERROR;
				chg->chg_abnormal_flag |= CHG_PL_STATUS_I2C_ERROR;
				//fuelsummary_collect_value(ID_SIC_I2C_ERR, 1);
				if(smblib_get_hvdcp_type(chg)>= HVDCP_TYPE_9V){
					vote(chg->pl_disable_votable, PL_STATUS_VOTER, true, 0);
					msleep(100);
					vote(chg->usb_icl_votable, PL_STATUS_VOTER, true, 1400000);
				}
				break;
			default:
				smblib_dbg(chg, PR_MISC, "pl status default\n");
				chg->pl_status = PL_STATUS_OK;
		}

		if(chg->pl_status == PL_STATUS_OK){
			vote(chg->pl_disable_votable, PL_STATUS_VOTER, false, 0);
			vote(chg->usb_icl_votable, PL_STATUS_VOTER, false, 0);
		}
	}
}
*/
static int smblib_dump_reg(struct smb_charger *chg, char *name, u16 addr)
{
	int rc;
	u8 stat = 0;

	rc = smblib_read(chg, addr, &stat);
	smblib_dbg(chg, PR_MISC, "%s:0x%02x = 0x%02x\n",name,addr,stat);
	return rc;
}

static void smblib_dump_regs(struct smb_charger *chg)
{
	smblib_dbg(chg, PR_MISC, "######## dump reg start #########\n");
	/*charge*/
	smblib_dump_reg(chg, "SCHG_P_CHGR_BATTERY_CHARGER_STATUS_1", 0x1006);
	smblib_dump_reg(chg, "SCHG_P_CHGR_BATTERY_CHARGER_STATUS_2", 0x1007);
	smblib_dump_reg(chg, "SCHG_P_CHGR_BATTERY_CHARGER_STATUS_3", 0x1009);
	smblib_dump_reg(chg, "SCHG_P_CHGR_BATTERY_CHARGER_STATUS_4", 0x100A);
	smblib_dump_reg(chg, "SCHG_P_CHGR_BATTERY_CHARGER_STATUS_5", 0x100B);
	smblib_dump_reg(chg, "SCHG_P_CHGR_BATTERY_CHARGER_STATUS_6", 0x100C);
	smblib_dump_reg(chg, "SCHG_P_CHGR_BATTERY_CHARGER_STATUS_7", 0x100D);
	smblib_dump_reg(chg, "SCHG_P_CHGR_BATTERY_CHARGER_STATUS_8", 0x100E);
	smblib_dump_reg(chg, "SCHG_P_CHGR_INT_RT_STS", 0x1010);
	smblib_dump_reg(chg, "SCHG_P_CHGR_CHARGING_ENABLE_CMD", 0x1042);
	smblib_dump_reg(chg, "SCHG_P_CHGR_CHGR_CFG1", 0x1050);
	smblib_dump_reg(chg, "SCHG_P_CHGR_CHGR_CFG2", 0x1051);
	smblib_dump_reg(chg, "SCHG_P_CHGR_CHARGER_ENABLE_CFG", 0x1052);
	smblib_dump_reg(chg, "SCHG_P_CHGR_CFG", 0x1053);
	smblib_dump_reg(chg, "SCHG_P_CHGR_CC_STATE_INT_EN", 0x1054);
	smblib_dump_reg(chg, "SCHG_P_CHGR_FAST_CHARGE_CURRENT_CFG", 0x1061);
	smblib_dump_reg(chg, "SCHG_P_CHGR_CHARGE_CURRENT_TERMINATION_CFG", 0x1062);
	smblib_dump_reg(chg, "SCHG_P_CHGR_TCCC_CHARGE_CURRENT_TERMINATION_CFG", 0x1063);
	smblib_dump_reg(chg, "SCHG_P_CHGR_MAX_FAST_CHARGE_CURRENT_CFG", 0x1066);
	smblib_dump_reg(chg, "SCHG_P_CHGR_ADC_ITERM_UP_THD_MSB_REG", 0x1067);
	smblib_dump_reg(chg, "SCHG_P_CHGR_ADC_ITERM_UP_THD_LSB_REG", 0x1068);
	smblib_dump_reg(chg, "SCHG_P_CHGR_ADC_ITERM_LO_THD_MSB_REG", 0x1069);
	smblib_dump_reg(chg, "SCHG_P_CHGR_ADC_ITERM_LO_THD_LSB_REG", 0x106A);

	smblib_dump_reg(chg, "SCHG_P_CHGR_FLOAT_VOLTAGE_CFG", 0x1070);
	smblib_dump_reg(chg, "SCHG_P_CHGR_MAX_FLOAT_VOLTAGE_CFG", 0x1076);
	smblib_dump_reg(chg, "SCHG_P_CHGR_RCHG_SOC_THRESHOLD_CFG", 0x107D);
	smblib_dump_reg(chg, "SCHG_P_CHGR_JEITA_EN_CFG", 0x1090);
	smblib_dump_reg(chg, "SCHG_P_CHGR_JEITA_CCCOMP_CFG", 0x1093);
	smblib_dump_reg(chg, "SCHG_P_CHGR_SAFETY_TIMER_ENABLE_CFG", 0x10A0);
	smblib_dump_reg(chg, "SCHG_P_CHGR_FAST_CHARGE_SAFETY_TIMER_CFG", 0x10A2);
	smblib_dump_reg(chg, "SCHG_P_CHGR_STEP_CHG_MODE_CFG", 0x10B0);

	/*dcdc*/
	smblib_dump_reg(chg, "SCHG_P_DCDC_ICL_MAX_STATUS", 0x1106);
	smblib_dump_reg(chg, "SCHG_P_DCDC_ICL_STATUS", 0x1107);
	smblib_dump_reg(chg, "SCHG_P_DCDC_AICL_ICL_STATUS", 0x1108);
	smblib_dump_reg(chg, "SCHG_P_DCDC_THERMAL_ICL_STATUS", 0x1109);
	smblib_dump_reg(chg, "SCHG_P_DCDC_AICL_STATUS", 0x110A);
	smblib_dump_reg(chg, "SCHG_P_DCDC_POWER_PATH_STATUS", 0x110B);
	smblib_dump_reg(chg, "SCHG_P_DCDC_ANALOG_LOOP_STATUS", 0x110C);
	smblib_dump_reg(chg, "SCHG_P_DCDC_OTG_STATUS", 0x110D);
	smblib_dump_reg(chg, "SCHG_P_DCDC_CMD_OTG", 0x1140);
	smblib_dump_reg(chg, "SCHG_P_DCDC_BAT_UVLO_THRESHOLD_CFG", 0x1151);
	smblib_dump_reg(chg, "SCHG_P_DCDC_OTG_CURRENT_LIMIT_CFG", 0x1152);
	smblib_dump_reg(chg, "SCHG_P_DCDC_VSYSMIN_CFG", 0x1183);

	/*batif*/
	smblib_dump_reg(chg, "SCHG_P_BATIF_BATTERY_MISSING_STATUS", 0x1206);
	smblib_dump_reg(chg, "SCHG_P_BATIF_INT_RT_STS", 0x1210);
	smblib_dump_reg(chg, "SCHG_P_BATIF_INT_LATCHED_STS", 0x1218);
	smblib_dump_reg(chg, "SCHG_P_BATIF_LOW_BATT_DETECT_EN_CFG", 0x1260);
	smblib_dump_reg(chg, "SCHG_P_BATIF_LOW_BATT_THRESHOLD_CFG", 0x1261);
	smblib_dump_reg(chg, "SCHG_P_BATIF_BAT_FET_CFG", 0x1262);

	/*usb*/
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_INPUT_STATUS", 0x1306);
	smblib_dump_reg(chg, "SCHG_P_USB_APSD_STATUS", 0x1307);
	smblib_dump_reg(chg, "SCHG_P_USB_APSD_RESULT_STATUS", 0x1308);
	smblib_dump_reg(chg, "SCHG_P_USB_QC_CHANGE_STATUS", 0x1309);
	smblib_dump_reg(chg, "SCHG_P_USB_INT_RT_STS", 0x1310);
	smblib_dump_reg(chg, "SCHG_P_USB_CMD_IL", 0x1340);
	smblib_dump_reg(chg, "SCHG_P_USB_CMD_ICL_OVERRIDE", 0x1342);
	smblib_dump_reg(chg, "SCHG_P_USB_TYPE_C_CFG", 0x1358);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_ADAPTER_ALLOW_CFG", 0x1360);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_OPTIONS_1_CFG", 0x1362);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_LOAD_CFG", 0x1365);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_ICL_OPTIONS", 0x1366);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_SOURCE_CHANGE_INTRPT_ENB", 0x1369);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_CURRENT_LIMIT_CFG", 0x1370);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_AICL_OPTIONS_CFG", 0x1380);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_5V_AICL_THRESHOLD_CFG", 0x1381);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_9V_AICL_THRESHOLD_CFG", 0x1382);
	smblib_dump_reg(chg, "SCHG_P_USB_USBIN_CONT_AICL_THRESHOLD_CFG", 0x1384);

	/*typeC*/
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_SNK_STATUS", 0x1506);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_SRC_STATUS", 0x1508);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_MISC_STATUS", 0x150B);
	smblib_dump_reg(chg, "SCHG_P_LEGACY_CABLE_STATUS_REG", 0x150D);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_MODE_CFG", 0x1544);

	/*misc*/
	smblib_dump_reg(chg, "SCHG_P_MISC_WDOG_STATUS", 0x160C);
	smblib_dump_reg(chg, "SCHG_P_MISC_WD_CFG", 0x1651);
	smblib_dump_reg(chg, "SCHG_P_MISC_SNARL_BARK_BITE_WD_CFG", 0x1653);
	smblib_dump_reg(chg, "SCHG_P_MISC_SMB_CFG(stat)", 0x1690);

	smblib_dbg(chg, PR_MISC, "######## dump reg end #########\n");
}

static void smblib_dump_otg_regs(struct smb_charger *chg)
{
	smblib_dbg(chg, PR_MISC, "@@@@@@@ dump otg reg start @@@@@@@\n");
	/*otg*/
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_SNK_STATUS", 0x1506);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_SNK_DEBUG_ACCESS_STATUS", 0x1507);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_SRC_STATUS", 0x1508);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_STATE_MACHINE_STATUS", 0x1509);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_SENSOR_SM_STATUS", 0x150a);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_MISC_STATUS", 0x150b);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_TRY_SNK_SRC_STATUS", 0x150c);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_LEGACY_CABLE_STATUS", 0x150d);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_CC_STATUS", 0x150e);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_U_USB_STATUS", 0x150f);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_MODE_CFG", 0x1544);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_INTERRUPT_EN_CFG_1", 0x155e);
	smblib_dump_reg(chg, "SCHG_P_TYPEC_TYPE_C_INTERRUPT_EN_CFG_2", 0x1560);

	smblib_dbg(chg, PR_MISC, "@@@@@@@ dump reg end @@@@@@@\n");
}

static bool smblib_check_otg_detach_loose(struct smb_charger *chg)
{
	int rc;
	u8 stat[2];

	rc = smblib_read(chg, 0x150b, &stat[0]);
	smblib_dbg(chg, PR_MISC, "[0] 0x%02x = 0x%02x\n", 0x150b, stat[0]);
	msleep(1000);
	rc = smblib_read(chg, 0x150b, &stat[1]);
	smblib_dbg(chg, PR_MISC, "[1] 0x%02x = 0x%02x\n", 0x150b, stat[1]);
	if (stat[0] == stat[1] && (stat[0] == 0xca || stat[0] == 0xc8)) {
		smblib_dbg(chg, PR_MISC, "otg detach loose.....\n");
		return true;
	}

	return false;
}

static void smblib_otg_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						otg_monitor_work.work);
	/*union power_supply_propval pval = {0, };*/
	int rc;
	bool otg_detach = false;

	smblib_dbg(chg, PR_MISC, "@@@@@@@ start otg monitor work @@@@@@@\n");

	smblib_dump_otg_regs(chg);

	/*WA:disable pl chg and watch dog*/
	/*if (!chg->usb_present && !chg->otg_disable_pl && is_parallel_charger_available(chg)) {
		pval.intval = true;
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_WATCH_DOG_STATUS, &pval);
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_OTG_DISABLE_PL, &pval);
		if (rc < 0)
			smblib_err(chg, "Couldn't change slave suspend state rc=%d\n", rc);
		chg->otg_disable_pl = true;
	}*/

	/*WA:otg irq detach loose*/
	otg_detach = smblib_check_otg_detach_loose(chg);
	if (otg_detach) {
		rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
					TYPEC_POWER_ROLE_CMD_MASK | TYPEC_TRY_MODE_MASK,
					EN_SNK_ONLY_BIT);
		smblib_dbg(chg, PR_MISC, "force to exit otg mode!\n");
		chg->otg_monitor_count = 0;
		chg->drp_timer_count = CHECK_DRP_DISABLE_WORK_COUNT;
		chg->drp_ongoing = false;
		chg->power_role = POWER_SUPPLY_TYPEC_PR_SINK;
		return;
	}

	/*WA:cancel monitor work failure*/
	if (!chg->drp_ongoing) {
		smblib_dbg(chg, PR_MISC, "force to exit monitor work!\n");
		return;
	}

	if (36000 == chg->otg_monitor_count)
		chg->otg_monitor_count = 1;
	else
		chg->otg_monitor_count++;
	fuelsummary_collect_value(ID_INT__OTG_CNT, chg->otg_monitor_count);
	smblib_dbg(chg, PR_MISC, "usb_present=%d,otg_disable_pl=%d,otg_monitor_count=%d\n", chg->usb_present, chg->otg_disable_pl, chg->otg_monitor_count);

	schedule_delayed_work(&chg->otg_monitor_work,
					msecs_to_jiffies(OTG_MONITOR_WORK_PEROID_MS));

	return;
}

/*confirm DRP disable or not when Type-C switch (OTG switch) Disable.*/
static void smblib_check_drp_disable_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger, check_drp_disable_work.work);
	int rc = 0;
	u8 power_role_now = 0;

	if (chg->drp_ongoing) {
		smblib_dbg(chg, PR_MISC, "OTG is using now!\n");
		goto retry;
	}

	if (!chg->drp_ongoing && chg->drp_timer_count-- <= 0) {
		smblib_dbg(chg, PR_MISC, "OTG switch timeout,close switch\n");
		chg->drp_enable = false;
		chg->drp_ongoing = false;
		rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
					TYPEC_POWER_ROLE_CMD_MASK | TYPEC_TRY_MODE_MASK,
					EN_SNK_ONLY_BIT);
		if (rc < 0) {
			smblib_dbg(chg, PR_MISC, "Couldn't write TYPE_C_MODE_CFG_REG, rc=%d\n", rc);
			goto retry;
		} else {
			chg->power_role = POWER_SUPPLY_TYPEC_PR_SINK;
		}
		if (wake_lock_active(&chg->drp_disable_wake_lock))
			wake_unlock(&chg->drp_disable_wake_lock);
		return;
	}

	rc = smblib_read(chg, TYPE_C_MODE_CFG_REG, &power_role_now);
	if (rc < 0) {
		smblib_dbg(chg, PR_MISC, "Couldn't read TYPE_C_MODE_CFG_REG, rc=%d\n", rc);
	}
	smblib_dbg(chg, PR_MISC, "OTG switch,timer count=%d,power_role_now=%d,power_role=%d\n",
				chg->drp_timer_count, power_role_now, chg->power_role);

retry:
	schedule_delayed_work(&chg->check_drp_disable_work, msecs_to_jiffies(CHECK_DRP_DISABLE_WORK_PEROID_MS));
}

static void smblib_weak_charger_voter_clear_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						weak_charger_voter_clear_work.work);

	smblib_dbg(chg, PR_MISC, "######## start smblib weak_charger_voter clear work #########\n");

	if (chg->weak_charger_count < 4) {
		vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER, false, 0);
	}

	return;
}

static void smblib_usb_psy_voter_clear_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						usb_psy_voter_clear_work.work);

	smblib_dbg(chg, PR_MISC, "######## start smblib usb_psy_voter clear work #########\n");
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);

	return;
}

static void smblib_kick_watch_dog(struct smb_charger *chg)
{
	int rc;

	if (enable_slave_charger) {
		rc = smblib_write(chg, BARK_BITE_WDOG_PET_REG, BARK_BITE_WDOG_PET_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't pet the dog rc=%d\n", rc);
	}
}

static void smblib_double_check_vbus_status(struct smb_charger *chg)
{
	int rc = 0;
	u8 stat;
	bool vbus_rising = chg->usb_present;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
	} else {
		vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
		if (chg->usb_present && vbus_rising != chg->usb_present) {
			if (++chg->vbus_check_count > 1) {
				smblib_dbg(chg, PR_MISC, "vbus monitor error, recall plugin handler!\n");
				smblib_usb_plugin_locked(chg);
				return;
			}
		}
	}
}

static void smblib_double_check_cc_float_cable(struct smb_charger *chg)
{
	bool vbus_rising = chg->usb_present;

	/*WA: for CC float cable*/
	if (vbus_rising && !is_atboot) {
		if (chg->connector_type == POWER_SUPPLY_CONNECTOR_TYPEC &&
			(chg->typec_mode == POWER_SUPPLY_TYPEC_NONE || chg->typec_mode == POWER_SUPPLY_TYPEC_NON_COMPLIANT)) {
			smblib_dbg(chg, PR_MISC, "######## CC float cable detect: force updae typec_mode to source_default #########\n");
			chg->typec_mode = POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
			chg->cable_with_cc_float = true;
			chg->typec_present = 1;
			chg->cable_with_cc_float_icl_rerun = false;
			power_supply_changed(chg->usb_psy);
		}
		if (chg->cable_with_cc_float && !chg->cable_with_cc_float_icl_rerun && (chg->usb_conndoned || chg->real_charger_type == POWER_SUPPLY_TYPE_USB_DCP)) {
			smblib_dbg(chg, PR_MISC, "######## CC float cable detect: rerun usb_icl current(usb_conndoned=%d, real_type=%d) \n", chg->usb_conndoned, chg->real_charger_type);
			chg->cable_with_cc_float_icl_rerun = true;
			rerun_election(chg->usb_icl_votable);
		}
	}
}

static void smblib_duble_check_safety_timeout_status(struct smb_charger *chg)
{
	bool vbus_rising = chg->usb_present;
	u8 stat_1 = 0, stat_2 = 0;
	int rc = 0;

	/*WA: duble check Safety Timeout status*/
	if (!chg->chg_safety_timer_en) {
		smblib_dbg(chg, PR_MISC, "check safety Timer timeout status.\n");
		rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat_1);
		if (rc < 0)
			smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n", rc);

		rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat_2);
		if (rc < 0)
			smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n", rc);

		if ((stat_1 & BATTERY_CHARGER_STATUS_MASK) == DISABLE_CHARGE &&
			(stat_2 & CHARGER_ERROR_STATUS_SFT_EXPIRE_BIT) &&
			vbus_rising) {
			smblib_dbg(chg, PR_MISC, "safety Timer timeout error happen, status_1=0x%02x, status_2=0x%02x\n", stat_1, stat_2);
			vote(chg->chg_disable_votable, SAFETY_TIMER_RESENT_VOTER, true, 0);
			vote(chg->usb_icl_votable, SAFETY_TIMER_RESENT_VOTER, true, 0);
			msleep(200);
			vote(chg->chg_disable_votable, SAFETY_TIMER_RESENT_VOTER, false, 0);
			vote(chg->usb_icl_votable, SAFETY_TIMER_RESENT_VOTER, false, 0);
		}
	}
}

static int smblib_ffc_get_data(struct smb_charger *chg, struct power_supply *psy,
			    enum power_supply_property psp, int *data, int type)
{
	union power_supply_propval pval = {0,};
	int rc, i, val = 0;
	bool go_on = false;

	if (!psy) {
		smblib_err(chg, "psy is null\n");
		return 0;
	}

	rc = power_supply_get_property(psy, psp, &pval);
	if (rc) {
		pr_err("couldn't get data,rc=%d\n", rc);
		return 0;
	}

	/*need process data*/
	switch (type) {
	case FFC_DATA_TYPE_IBAT:
		i = chg->ffc_full_chg_ibat_rc[ROW] - 1;
		if (is_between(chg->ffc_full_chg_ibat_table[i].ibat, 0, pval.intval/1000)) {
			go_on = true;
		}
		break;
	case FFC_DATA_TYPE_VBAT_EX:
		if (pval.intval >= chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_EX]) {
			go_on = true;
		}
		break;
	case FFC_DATA_TYPE_VBAT:
		if (pval.intval >= chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_TERM]) {
			go_on = true;
		}
		break;
	default:
		break;
	}
	val = pval.intval;

	if (go_on) {
		for (i = 0; i < 2; i++) {
			msleep(1100);
			power_supply_get_property(psy, psp, &pval);
			val += pval.intval;
		}
		val = val / 3;
	}

	*data = val;
	smblib_dbg(chg, PR_MISC, "%s:val=%d\n", ffc_data_type_strings[type], val);

	return go_on;
}

static int smblib_ffc_update_param(struct smb_charger *chg, int bat_temp, int *ffc_full_chg_ibat_ma)
{
	int index, rc, *tmp_vbat_table;
	union power_supply_propval pval = {0,};
	static bool determine_multiple_ffc_full_chg_ibat_table_config;
	struct ffc_full_chg_ibat_table *tmp_ibat_table;

	if (!determine_multiple_ffc_full_chg_ibat_table_config) {
		smblib_dbg(chg, PR_MISC, "start to determine ffc-full-chg-ibat-table config according to bat-id\n");
		if (chg->multiple_ffc_full_chg_ibat_table_config) {
			if (!chg->cms_psy) {
				chg->cms_psy = get_cms_psy(chg);
				smblib_dbg(chg, PR_MISC, "chg->cms_psy %s\n", chg->cms_psy ? "found" : "not found");
			}

			rc = power_supply_get_property(chg->cms_psy, POWER_SUPPLY_PROP_BATTERY_ID, &pval);
			if (rc || !chg->multiple_batid || !chg->cms_psy)
				smblib_err(chg, "Couldn't get bat-id or multiple batids unavailable or cms not found. "
								"Failed to configure multiple ffc-full-chg-ibat-table\n");
			else {
				if (pval.intval == chg->multiple_batid[0])
					smblib_dbg(chg, PR_MISC, "Match first batid, use ffc-full-chg-ibat-table, bid=%d\n", pval.intval);
				else if (pval.intval == chg->multiple_batid[1]) {
					tmp_ibat_table = chg->ffc_full_chg_ibat_table;
					chg->ffc_full_chg_ibat_table = chg->ffc_full_chg_ibat_table_sec;
					chg->ffc_full_chg_ibat_table_sec = tmp_ibat_table;
					smblib_dbg(chg, PR_MISC, "Match second batid, use ffc-full-chg-ibat-table-second, bid=%d\n", pval.intval);
				} else if (pval.intval == chg->multiple_batid[2]) {
					/* new ffc full chg ibat table */
					tmp_ibat_table = chg->ffc_full_chg_ibat_table;
					chg->ffc_full_chg_ibat_table = chg->ffc_full_chg_ibat_table_third;
					chg->ffc_full_chg_ibat_table_third = tmp_ibat_table;
					/* new ffc float voltage max */
					chg->ffc_fv_max_uv = chg->ffc_fv_max_uv_sec;
					/* new ffc full chg vbat table */
					tmp_vbat_table = chg->ffc_full_chg_vbat_table;
					chg->ffc_full_chg_vbat_table = chg->ffc_full_chg_vbat_table_sec;
					chg->ffc_full_chg_vbat_table_sec = tmp_vbat_table;

					chg->is_high_volt_battery = true;

					smblib_dbg(chg, PR_MISC, "Match third batid, use ffc-full-chg-ibat-table-third and"
							" ffc-full-chg-vbat-table-second, bid=%d, ffc_fv=%d, is_high_volt_batt=%d\n",
							pval.intval, chg->ffc_fv_max_uv, chg->is_high_volt_battery);
				} else
					smblib_dbg(chg, PR_MISC, "Don't match any batid, use ffc-full-chg-ibat-table by default, bid=%d\n", pval.intval);
			}
		} else
			smblib_dbg(chg, PR_MISC, "no multiple ffc-full-chg-ibat-table config, skip\n");

		determine_multiple_ffc_full_chg_ibat_table_config = true;
	}

	/*get ffc full chg ibat current*/
	for (index = 0; index < chg->ffc_full_chg_ibat_rc[ROW]; index++) {
		if (is_between(chg->ffc_full_chg_ibat_table[index].tmin, chg->ffc_full_chg_ibat_table[index].tmax,
			bat_temp)) {
			*ffc_full_chg_ibat_ma = chg->ffc_full_chg_ibat_table[index].ibat;
			chg->ffc_full_chg_iterm_delta_ma = chg->ffc_full_chg_ibat_table[index].ibat_delta;
			break;
		}
	}

	/* compatible ex fg and qcom fg */
	switch (chg->ffc_param_tuning_enable) {
	case 1963:
		/*for project:1963*/
		if (!ex_fg_support) {
			if (chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT] != chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_EX]) {
				chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT] = chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_EX];
			}
		}
		break;
	case 2073:
		/*for project:2073*/
		if (ex_fg_support) {
			chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT] = 4480000;
		}
		break;
	default:
		smblib_dbg(chg, PR_MISC, "nothing to do.\n");
	}

	smblib_dbg(chg, PR_MISC, "ffc param:ffc_chg_term_current_ma=%d,ffc_fv_max_uv=%d,ffc_full_chg_ibat_ma=%d,ffc_full_chg_ibat_delta_ma=%d,ffc_full_chg_vbat_uv=%d,ffc_full_chg_vbat_ex_uv=%d,ffc_full_chg_vbat_term_uv=%d\n",
		chg->ffc_chg_term_current_ma, chg->ffc_fv_max_uv, *ffc_full_chg_ibat_ma, chg->ffc_full_chg_iterm_delta_ma, chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT], chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_EX], chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_TERM]);

	return 0;
}

static void smblib_ffc_adjust_function(struct smb_charger *chg, int *work_period_ms)
{
	struct smb5 *chip = container_of(chg, struct smb5, chg);
	int rc = 0, ibat_ma = 0, vbat_uv = 0, fg_vbat_uv = 0, fg_capacity = 0, bat_temp = 0;
	int total_current = chg->primary_fastchg_max_ua / 1000 + chg->parallel_fastchg_max_ua / 1000 + 100;
	int ffc_full_chg_ibat_ma = -1180;
	struct power_supply *qc_bms_psy = power_supply_get_by_name("bms");

	if (!ex_fg_ffc_support) {
		smblib_dbg(chg, PR_MISC, "not support ffc!\n");
		return;
	}

	/* add for ffc charging */
	if (chg->bms_psy &&
		smblib_get_handshake_status(chg)) {

		/*get bat  temp*/
		rc += smblib_ffc_get_data(chg, qc_bms_psy,
					POWER_SUPPLY_PROP_TEMP, &bat_temp, FFC_DATA_TYPE_BAT_TEMP);

		smblib_dbg(chg, PR_MISC, "bat_temp=%d,dchg_status=%d,charge_full=%d,ffc_charge_full=%d\n",
			bat_temp, chg->dchg_status, chg->charge_full, chg->ffc_charge_full);

		if (bat_temp <= chg->ffc_temperature_range[0] || bat_temp >= chg->ffc_temperature_range[1]
			|| vivo_ex_fg_i2c_is_exception(chg)) {/*bq27750_IIC_ERROR = BIT(1) */
			vivo_set_chg_term_voltage(chg, chip->dt.batt_profile_fv_uv);
			vivo_set_chg_term_current(chg, chip->dt.term_current_thresh_hi_ma);
			smblib_dbg(chg, PR_MISC, "charigng not in ffc mode\n");
		} else {
			/*get charging current*/
			rc += smblib_ffc_get_data(chg, chg->bms_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ma, FFC_DATA_TYPE_IBAT);
			ibat_ma = ibat_ma / 1000;

			/*get vbat from qcom fuel gauge*/
			rc += smblib_ffc_get_data(chg, qc_bms_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv, FFC_DATA_TYPE_VBAT_EX);

			/*get soc from ext fuel gauge*/
			rc += smblib_ffc_get_data(chg, chg->bms_psy,
					POWER_SUPPLY_PROP_CAPACITY, &fg_capacity, FFC_DATA_TYPE_CAPACITY);

			/*get vbat from bq27750*/
			rc += smblib_ffc_get_data(chg, chg->bms_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &fg_vbat_uv, FFC_DATA_TYPE_VBAT);
			if (rc > 0) {
				*work_period_ms = CHARGER_MONITOR_WORK_PEROID_MS / 2;
			}

			/*update ffc param*/
			smblib_ffc_update_param(chg, bat_temp, &ffc_full_chg_ibat_ma);

			smblib_dbg(chg, PR_MISC, "ibat_ma=%d,flag=%d,capcity=%d,bat_temp=%d,vbat_uv=%d,fg_vbat_uv=%d,cout_gpio=%d,dchg_status=%d,charge_full=%d,ffc_charge_full=%d,ffc_pmi_charging_count=%d,is_high_volt_batt=%d\n",
				ibat_ma, is_between(ffc_full_chg_ibat_ma, 0, ibat_ma), fg_capacity, bat_temp, vbat_uv, fg_vbat_uv, battery_cout_value, chg->dchg_status, chg->charge_full,
				chg->ffc_charge_full, chg->ffc_pmi_charging_count, chg->is_high_volt_battery);

			if (!chg->ffc_charge_full && chg->dchg_status == DIRECT_CHARGER_UNKNOW
				 && !chg->ffc_pmi_charging_count) {
				vivo_set_chg_term_voltage(chg, chg->ffc_fv_max_uv);
				vivo_set_chg_term_current(chg, chg->ffc_chg_term_current_ma);
				chg->ffc_pmi_charging_count = 1;
				smblib_dbg(chg, PR_MISC, "ffc step0\n");
			} else if (!chg->ffc_charge_full && chg->dchg_status == DIRECT_CHARGER_IS_CHARGERING
				 && is_between(-(total_current), 0, ibat_ma)) {
				vivo_set_chg_term_voltage(chg, chg->ffc_fv_max_uv);
				vivo_set_chg_term_current(chg, chg->ffc_chg_term_current_ma);

				if ((is_between(ffc_full_chg_ibat_ma, 0, ibat_ma) && (fg_vbat_uv >= chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT] || (battery_cout_value >= 1 && !chg->ignore_cout && !chg->is_high_volt_battery)))
					|| (is_between(chg->ffc_chg_term_current_ma, 0, ibat_ma) && (fg_capacity == 100 || fg_vbat_uv >= chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_TERM]))) {
					vivo_set_chg_term_voltage(chg, chip->dt.batt_profile_fv_uv);
					vivo_set_chg_term_current(chg, chip->dt.term_current_thresh_hi_ma);
					/* disable direct charge */
					if (!chg->dchg_disable_votable)
						chg->dchg_disable_votable = find_votable("DCHG_DISABLE");
					if (chg->dchg_disable_votable)
						vote(chg->dchg_disable_votable, FFC_VOTER, true, 0);

					chg->ffc_charge_full = true;
					smblib_dbg(chg, PR_MISC, "battery charging full in direct charge mode\n");
				}
				chg->ffc_pmi_charging_count = 1;
				smblib_dbg(chg, PR_MISC, "ffc step1\n");
			} else if (!chg->ffc_charge_full
				&& chg->direct_charger_enable
				&& (chg->dchg_status == DIRECT_CHARGER_END || chg->dchg_status == DIRECT_CHARGER_UNKNOW)
				&& chg->ffc_pmi_charging_count++ > 1
				&& ((is_between(ffc_full_chg_ibat_ma, 0, ibat_ma) && (vbat_uv >= chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_EX] || fg_vbat_uv >= chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT] || (battery_cout_value >= 1 && !chg->ignore_cout && !chg->is_high_volt_battery)))
				|| (is_between(chg->ffc_chg_term_current_ma, 0, ibat_ma) && (fg_capacity == 100 || fg_vbat_uv >= chg->ffc_full_chg_vbat_table[FFC_FULL_CHG_VBAT_TERM])))) {
				vivo_set_chg_term_voltage(chg, chip->dt.batt_profile_fv_uv);
				vivo_set_chg_term_current(chg, chip->dt.term_current_thresh_hi_ma);
				chg->ffc_charge_full = true;
				smblib_dbg(chg, PR_MISC, "ffc step2\n");
			} else if (chg->ffc_charge_full == true) {
				vivo_set_chg_term_voltage(chg, chip->dt.batt_profile_fv_uv);
				vivo_set_chg_term_current(chg, chip->dt.term_current_thresh_hi_ma);
				smblib_dbg(chg, PR_MISC, "ffc step3\n");
			}
		}
	} else {
		vivo_set_chg_term_voltage(chg, chip->dt.batt_profile_fv_uv);
		vivo_set_chg_term_current(chg, chip->dt.term_current_thresh_hi_ma);
		smblib_dbg(chg, PR_MISC, "batt_profile_fv_uv=%d,term_current_thresh_hi_ma=%d\n",
					chip->dt.batt_profile_fv_uv, chip->dt.term_current_thresh_hi_ma);
	}

	if (chg->ffc_pmi_charging_count >= INT_MAX)
		chg->ffc_pmi_charging_count = 1;

}

static void smblib_force_rerun_apsd_for_first(struct smb_charger *chg)
{
	/* force apsd_rerun_work delay 300s when upgrade system software with usbin */
	if (chg->apsd_rerun_first && !power_off_charging_mode &&
		chg->charge_monitor_count >= 60 &&
		(chg->real_charger_type == POWER_SUPPLY_TYPE_USB ||
		chg->real_charger_type == POWER_SUPPLY_TYPE_USB_CDP ||
		chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT ||
		chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN)) {
			chg->apsd_rerun_first = false;
			chg->apsd_rerun_status = WORK_STATUS_IDLE;
			smblib_start_apsd_rerun_work(chg);
	} else if (chg->apsd_rerun_first && !power_off_charging_mode &&
			   chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
			chg->apsd_rerun_first = false;
			chg->apsd_rerun_status = WORK_STATUS_IDLE;
			smblib_start_apsd_rerun_work(chg);
	}
}

static void smblib_force_charge_full_ui_for_high_vbat(struct smb_charger *chg, int vbat_voltage, int chg_status)
{
	struct smb5 *chip = container_of(chg, struct smb5, chg);
	union power_supply_propval fastchg_ma = {0,};
	int ibat_ma = 0;
	int float_voltage = get_effective_result_locked(chg->fv_votable);

	/*force charge full UI when Battery Voltage too high (use for the fake battery)*/
	if (!ex_fg_ffc_support) {
		power_supply_get_property(chg->bms_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &fastchg_ma);
		ibat_ma = fastchg_ma.intval /1000;
		smblib_dbg(chg, PR_MISC, "float_voltage=%d,ibat_ma=%d\n", float_voltage, ibat_ma);
		if (chg->dchg_status == DIRECT_CHARGER_END && float_voltage == chip->dt.batt_profile_fv_uv &&
			vbat_voltage > float_voltage + 15000 && ibat_ma > 0 &&
			(chg_status == POWER_SUPPLY_STATUS_CHARGING)) {
			if (chg->force_charge_full_count++ >= 5) {
				chg->force_charge_full_count = 5;
				chg->charge_full = true;
				chg->force_charge_full_by_high_voltage = true;
			}
			smblib_dbg(chg, PR_MISC, "force_charge_full_count=%d, force_charge_full_by_high_voltage=%d, charge_full=%d\n",
					chg->force_charge_full_count, chg->force_charge_full_by_high_voltage, chg->charge_full);
		} else {
			chg->force_charge_full_count = 0;
			chg->force_charge_full_by_high_voltage = false;
		}
	}
}

static bool smblib_charger_determin_float_voltage(struct smb_charger *chg)
{
	struct smb5 *chip = container_of(chg, struct smb5, chg);
	union power_supply_propval batid = {0,};
	int rc;
	int tmp_fv_uv;

	if (chg->high_vbat_batid <= 0)
		return true;

	if (!chg->cms_psy)
		chg->cms_psy = power_supply_get_by_name("cms");
	rc = power_supply_get_property(chg->cms_psy, POWER_SUPPLY_PROP_BATTERY_ID, &batid);
	if (rc || !chg->cms_psy) {
		smblib_dbg(chg, PR_MISC, "get batid failed or cms_psy not found, will check later\n");
		return false;
	}

	tmp_fv_uv = chip->dt.batt_profile_fv_uv;
	if (batid.intval == chg->high_vbat_batid)
		chip->dt.batt_profile_fv_uv = chg->float_volt_sec;

	smblib_dbg(chg, PR_MISC, "high voltage battery id = %d, fv: %d -> %d\n",
				chg->high_vbat_batid, tmp_fv_uv, chip->dt.batt_profile_fv_uv);
	return true;
}

static void smblib_charge_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						charge_monitor_work.work);
	union power_supply_propval usb_input_current = {0, };
	union power_supply_propval vbat_voltage = {0, };
	union power_supply_propval chg_status = {0, };
	union power_supply_propval pval = {0, };
	union power_supply_propval usbin_uv = {0,};
	static bool determine_fv_max;

	int work_period_ms = CHARGER_MONITOR_WORK_PEROID_MS;
	const struct apsd_result *apsd_result;

	smblib_dbg(chg, PR_MISC, "######## start charger monitor work #########\n");
	/* dump regs */
	smblib_dump_regs(chg);

	/* dump votable */
	get_votable_info(chg->fcc_votable);
	get_votable_info(chg->usb_icl_votable);
	get_votable_info(chg->fv_votable);
	get_votable_info(chg->awake_votable);
	get_votable_info(chg->chg_disable_votable);
	//get_votable_info(chg->pl_disable_votable);
	//get_votable_info(chg->pl_enable_votable_indirect);
	// get_votable_info(chg->usb_suspend_votable);
	//get_votable_info(chg->pd_disallowed_votable_indirect);
	//get_votable_info(chg->pd_allowed_votable);
	//get_votable_info(chg->hvdcp_disable_votable_indirect);
	//get_votable_info(chg->hvdcp_enable_votable);
	//get_votable_info(chg->apsd_disable_votable);
	//get_votable_info(chg->hvdcp_hw_inov_dis_votable);
	if (!determine_fv_max)
		determine_fv_max = smblib_charger_determin_float_voltage(chg);

	/* dump pl regs */
	if (!chg->direct_charger_enable && is_parallel_charger_available(chg)) {
		power_supply_get_property(chg->pl.psy,
						POWER_SUPPLY_PROP_DUMP_REG, &pval);
	}
	smblib_dbg(chg, PR_MISC, "########  #########\n");

	/* get charge data */
	smblib_get_primary_board_temp(chg, &pval);
	smblib_get_prop_usb_voltage_now(chg, &usbin_uv);
	smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_voltage);
	power_supply_get_property(chg->batt_psy, POWER_SUPPLY_PROP_STATUS, &chg_status);
	//fuelsummary_collect_value(ID_TERM, (chg_status.intval == POWER_SUPPLY_STATUS_FULL));
	/* kick watch dog for dual-direct-charge */
	smblib_kick_watch_dog(chg);

	if (chg->keep_chg_soc < 100)
		smblib_keep_chg_soc(chg);

	/*WA: double check Vbus status for loose plugout irq*/
	smblib_double_check_vbus_status(chg);

	/*WA: for CC float cable*/
	smblib_double_check_cc_float_cable(chg);
	fuelsummary_collect_value(ID_BIT__CC_ERR, chg->cable_with_cc_float);
	/*WA: duble check Safety Timeout status*/
	smblib_duble_check_safety_timeout_status(chg);

	/* check usbin vbus voltage */
	chg->chg_voltage_v = smblib_get_hvdcp_type(chg);

	/* check charge terminate by soft */
	smblib_termination_by_soft(chg, chg_status.intval);

	/* check charge recharge by soft */
	smblib_recharge_by_soft(chg, chg_status.intval);

	/* charge full force 9v to 5v for 9v2a-case*/
	/*
	if (get_effective_result_locked(chg->hvdcp_enable_votable))
		smblib_charge_full_force_5v(chg, chg_status.intval, chg->chg_voltage_v);
	*/

	/* get charge usb current now */
	smblib_get_prop_usb_current_now(chg, &usb_input_current);

	/* check pl status and maybe voter current */
	//smblib_check_pl_abnormal_status(chg);

	/* clear usb temporary present after 60s */
	if (5 == chg->charge_monitor_count) {
		smblib_dbg(chg, PR_MISC, "clear usb temporary present...\n");
		chg->usb_temporary_present = 0;
		power_supply_changed(chg->batt_psy);
		power_supply_changed(chg->usb_psy);
	}

	/* when disable pl,voter input 1.4a for 9v2a-case */
	if (!chg->direct_charger_enable && get_effective_result_locked(chg->pl_disable_votable) &&
				smblib_get_hvdcp_type(chg) >= HVDCP_TYPE_9V) {
		vote(chg->usb_icl_votable, HVDCP_CTRL_ICL_VOTER, true, 1400000);
	} else {
		vote(chg->usb_icl_votable, HVDCP_CTRL_ICL_VOTER, false, 0);
	}

	/* when fcc <10000 ,disable pl  for 9v2a-case */
	if(!chg->direct_charger_enable && get_effective_result_locked(chg->fcc_votable) < 1000000){
		vote(chg->pl_disable_votable, USER_SET_IBAT_VOTER, true, 0);
	} else {
		vote(chg->pl_disable_votable, USER_SET_IBAT_VOTER, false, 0);
	}

	/* when charge 1min, USB_PSY_VOTER vote false */
	if (is_client_vote_enabled_locked(chg->usb_icl_votable, USB_PSY_VOTER) &&
		chg->charge_monitor_count > 6 &&
		get_client_vote_locked(chg->usb_icl_votable, USB_PSY_VOTER) < DEFAULT_SDP_UA) {
		schedule_delayed_work(&chg->usb_psy_voter_clear_work,
				msecs_to_jiffies(8000));
	}

	/* typec mode is none && current 0, reset usb-mode 900ma */
	if (2 == chg->charge_monitor_count && chg->connector_type == POWER_SUPPLY_CONNECTOR_TYPEC &&
			chg->cable_with_cc_float) {
		apsd_result = smblib_get_apsd_result(chg);
		if (apsd_result->pst != POWER_SUPPLY_TYPE_USB) {
			smblib_dbg(chg, PR_MISC, "override icl : uclc mode...\n");
			chg->cable_with_cc_float_icl_override = true;
			rerun_election(chg->usb_icl_votable);
		}
	}

	/* force apsd_rerun_work delay 300s->60s when upgrade system software with usbin */
	smblib_force_rerun_apsd_for_first(chg);

	/* force charge full UI when Battery Voltage too high (use for the fake battery) */
	smblib_force_charge_full_ui_for_high_vbat(chg, vbat_voltage.intval, chg_status.intval);

	/* ffc adjust function: term current,vfloat */
	smblib_ffc_adjust_function(chg, &work_period_ms);

	if(36000 == chg->charge_monitor_count)
		chg->charge_monitor_count = 1;
	else
		chg->charge_monitor_count++;

	smblib_dbg(chg, PR_MISC, "count=%d,bigdata_chg_voltage_uv=%d,hvdcp_type=%d,usb_input_current=%d,usbin_uV=%d,usb_temporary_present=%d,chg_abnormal_flag=0x%02x,vbat_in_iterm=%d,vbat=%duV,charge_full=%d,ffc_charge_full=%d\n",
				chg->charge_monitor_count, chg->bigdata_chg_voltage_uv, chg->hvdcp_type, usb_input_current.intval, usbin_uv.intval,
				chg->usb_temporary_present, chg->chg_abnormal_flag, chg->vbat_in_iterm, vbat_voltage.intval,
				chg->charge_full, chg->ffc_charge_full);

	schedule_delayed_work(&chg->charge_monitor_work,
				msecs_to_jiffies(work_period_ms));
	return;
}

static void smblib_apsd_ctrl_pl_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						apsd_ctrl_pl_work.work);
	int icl_uv;

	smblib_dbg(chg, PR_MISC, "######## start smblib apsd ctrl pl work #########\n");

	smblib_get_charge_param(chg, &chg->param.icl_stat, &icl_uv);
	smblib_dbg(chg, PR_MISC, "get now icl_uv = %d \n",icl_uv);

	smblib_dbg(chg, PR_MISC, "######## apsd enable pl #########\n");

	vote(chg->pl_disable_votable, APSD_CTRL_PL_VOTER, false, 0);

	return;
}

static void smblib_split_current_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						split_current_monitor_work.work);
	int icl_uv;
	union power_supply_propval pval = {0,};

	smblib_dbg(chg, PR_MISC, "######## start split current monitor work #########\n");

	smblib_get_charge_param(chg, &chg->param.icl_stat, &icl_uv);
	smblib_dbg(chg, PR_MISC, "get now icl_uv = %d \n",icl_uv);

	smblib_dbg(chg, PR_MISC, "######## user enable pl #########\n");
	if(is_parallel_charger_available(chg)){
		pval.intval = 1;
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_INIT_SLAVE_CHARGER, &pval);
		msleep(500);
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_INPUT_SUSPEND, &pval);
	}

	chg->detect_icl_ibat_ua = 1000000;
	vote(chg->pl_disable_votable, USER_CTRL_PL_VOTER, false, 0);
	msleep(100);
	vote(chg->fcc_votable, USB_FIRST_ICL_DONE_VOTER, false, 0);
	vote(chg->usb_icl_votable, USER_CTRL_PL_VOTER, false, 0);

	chg->split_current_monitor_status = true;

	/*typec switch mode for detect ID irq valid or invalid*/
	/*
	if (chg->usb_det_pin_mode == USB_DET_PIN_MODE_IDLE ||
			chg->usb_det_pin_mode == USB_DET_PIN_MODE_INVALID) {
		smblib_dbg(chg, PR_MISC, "usb_det_pin_mode: detect id irq invalid, always set drp!\n");
		chg->usb_det_pin_mode = USB_DET_PIN_MODE_INVALID;
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL_VIVO;
		smblib_set_prop_typec_power_role(chg, &pval);
		power_supply_changed(chg->usb_psy);
	} else {
		smblib_dbg(chg, PR_MISC, "usb_det_pin_mode: detect id irq valid...\n");
	}
	*/
	return;
}

static void smblib_start_apsd_rerun_work(struct smb_charger *chg)
{
	unsigned int work_period = 0;

	if((chg->usb_present || chg->apsd_done_before_usb_present) &&
		 chg->apsd_rerun_status == WORK_STATUS_IDLE &&
		chg->hvdcp_charge_monitor_status == WORK_STATUS_IDLE){
		chg->apsd_rerun_status = WORK_STATUS_START;
		chg->apsd_rerun_retries = 0;

		if(chg->real_charger_type == POWER_SUPPLY_TYPE_USB)
			work_period = APSD_RERUN_FOR_USB_MS;
		else
			work_period = is_atboot ? APSD_RERUN_MS_AT : APSD_RERUN_MS;
		smblib_dbg(chg, PR_MISC, "rerun apsd after %dms,charger_type=%d\n", work_period, chg->real_charger_type);
		cancel_delayed_work(&chg->apsd_rerun_work);
		if (power_off_charging_mode || !chg->apsd_rerun_first)
			schedule_delayed_work(&chg->apsd_rerun_work,
					msecs_to_jiffies(work_period));
		else
			smblib_dbg(chg, PR_MISC, "start rerun apsd, cancelled(%d,%d)\n",
				power_off_charging_mode, chg->apsd_rerun_first);
	}
}

static void smblib_apsd_rerun_work(struct work_struct *work)
{
	int rc;
	struct smb_charger *chg = container_of(work, struct smb_charger,
						apsd_rerun_work.work);

	const struct apsd_result *apsd_result;
	union power_supply_propval is_present = {0,};
	int current_limit_ua;
	bool rerun_secceful = false;
	int usbsel_direct = 0;
	unsigned int work_period = 0;

	if(chg->apsd_rerun_status == WORK_STATUS_IDLE){
		smblib_dbg(chg, PR_MISC, "rerun not request\n");
		goto abort;
	}

	rc = smblib_get_prop_usb_present(chg, &is_present);
	if(rc < 0){
		smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
	}

	if(!is_present.intval){
		chg->apsd_rerun_status = WORK_STATUS_IDLE;
		smblib_dbg(chg, PR_MISC, "usb not present\n");
		goto abort;
	}

	smblib_dbg(chg, PR_MISC, "usb conndoned=%d\n",chg->usb_conndoned);
	if (chg->usb_conndoned) {
		chg->apsd_rerun_status = WORK_STATUS_END;
		smblib_dbg(chg, PR_MISC, "usb conndoned,not rerun\n");
		goto abort;
	}

	if (chg->cable_with_cc_float && chg->real_charger_type == POWER_SUPPLY_TYPE_USB) {
		chg->apsd_rerun_status = WORK_STATUS_END;
		smblib_dbg(chg, PR_MISC, "cable_with_cc_float,not rerun\n");
		goto abort;
	}

	apsd_result = smblib_get_apsd_result(chg);
	smblib_dbg(chg, PR_MISC, "inserted type = %d (%s) retries=%d\n",
					apsd_result->pst,apsd_result->name,chg->apsd_rerun_retries);

	if (chg->apsd_rerun_retries++ >= (is_atboot ? APSD_RERUN_RETRIES_MAX_AT : APSD_RERUN_RETRIES_MAX)) {
		smblib_dbg(chg, PR_MISC, "retries max,maybe charge type is true\n");
		rerun_secceful = true;
	}

	switch (apsd_result->pst){
	case POWER_SUPPLY_TYPE_USB:
		if(rerun_secceful){/* retry max */
			current_limit_ua = DEFAULT_SDP_UA;
			chg->chg_abnormal_flag |= CHG_RERUN_USB_ERROR;
			if(!chg->usb_conndoned){
				/*chg->real_charger_type = POWER_SUPPLY_TYPE_USB_DCP;*/
				vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
				smblib_dbg(chg, PR_MISC, "sdp usb_conndone is false,set dcp(500ma) to dwc3\n");
			}
		}
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		if(rerun_secceful){/* retry max */
			current_limit_ua = DEFAULT_CDP_UA;
			chg->chg_abnormal_flag |= CHG_RERUN_CDP_ERROR;
			if(!chg->usb_conndoned){
				/*chg->real_charger_type = POWER_SUPPLY_TYPE_USB_DCP;*/
				vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
				smblib_dbg(chg, PR_MISC, "cdp usb_conndone is false,set dcp(500ma) to dwc3\n");
			}
		}
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		if (rerun_secceful) {/* retry max */
			current_limit_ua = DEFAULT_FLOAT_UA;
			chg->chg_abnormal_flag |= CHG_RERUN_FLOAT_ERROR;
			if(!chg->usb_conndoned) {
				vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
				//vote(chg->usb_icl_votable, LEGACY_UNKNOWN_VOTER, false, 0);
				smblib_dbg(chg, PR_MISC, "float usb_conndone is false,set (900mA) to dwc3\n");
			}
		}
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_HVDCP:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		if(!strncmp(apsd_result->name, "OCP", 3)){
			if (rerun_secceful) {/* retry max */
				current_limit_ua = DEFAULT_OTHER_UA;
				chg->chg_abnormal_flag |= CHG_RERUN_OTHER_ERROR;
			}
		}else{
			current_limit_ua = chg->total_input_current_max_ua;
			rerun_secceful = true;
		}
		break;
	default:
		/* POWER_SUPPLY_TYPE_UNKNOWN */
		current_limit_ua = DEFAULT_OTHER_UA;
		break;
	}

	if(rerun_secceful){
		if (chg->cable_with_cc_float && !chg->cable_with_cc_float_icl_rerun) {
			smblib_dbg(chg, PR_MISC, "######## CC float cable detect: rerun usb_icl current(usb_conndoned=%d, real_type=%d) \n", chg->usb_conndoned, chg->real_charger_type);
			chg->cable_with_cc_float_icl_rerun = true;
			rerun_election(chg->usb_icl_votable);
		}

		if(apsd_result->pst == POWER_SUPPLY_TYPE_USB ||
			apsd_result->pst == POWER_SUPPLY_TYPE_USB_CDP){
			usbsel_direct = 0;
		} else {
			usbsel_direct = 1;
		}

		if(chg->usbsel_gpio){
		   gpio_direction_output(chg->usbsel_gpio,usbsel_direct);
		   smblib_dbg(chg, PR_MISC, "usbsel=%d,usb_supply_type=%d\n",
						gpio_get_value(chg->usbsel_gpio),apsd_result->pst);
		}

		if (apsd_result->pst == POWER_SUPPLY_TYPE_USB_DCP ||
			apsd_result->pst == POWER_SUPPLY_TYPE_USB_FLOAT ||
			(apsd_result->pst == POWER_SUPPLY_TYPE_USB && !chg->usb_conndoned)) {
			if (chg->direct_charger_enable) {
				smblib_dbg(chg, PR_MISC, "inserted type =%d(%s), rerun to %d, usb_conndoned=%d. enable usbsel for dchg.\n", apsd_result->pst, apsd_result->name, chg->real_charger_type, chg->usb_conndoned);
				smblib_usbsel_config(chg,1);
			}
			/*  usb conn protect function */
			if (chg->usb_conn_pro_enable) {
				if (power_off_charging_mode && apsd_result->pst == POWER_SUPPLY_TYPE_USB) {
					smblib_dbg(chg, PR_MISC, " power_off_charging_mode && USB cable, skip start_usb_conn_pro_monitor_work!!!\n");
				} else
					schedule_delayed_work(&chg->start_usb_conn_pro_monitor_work,
							msecs_to_jiffies(0));
			}

		}

		smblib_dbg(chg, PR_MISC, "rerun %s detected setting mA = %d\n",
			apsd_result->name, current_limit_ua);
		rc = vote(chg->usb_icl_votable, USER_DEFAULT_INPUT_VOTER, true,
					current_limit_ua);

		chg->apsd_rerun_status = WORK_STATUS_END;
		goto abort;
	}

	smblib_rerun_apsd_if_required(chg);

	if(chg->real_charger_type == POWER_SUPPLY_TYPE_USB)
		work_period = APSD_RERUN_PERIOD_MS;
	else
		work_period = is_atboot ? APSD_RERUN_PERIOD_MS_AT : APSD_RERUN_PERIOD_MS;
	smblib_dbg(chg, PR_MISC, "rerun apsd after %dms, charger_type=%d\n", work_period, chg->real_charger_type);
	schedule_delayed_work(&chg->apsd_rerun_work,
				msecs_to_jiffies(work_period));
	return;
abort:
	if(!is_present.intval){
		smblib_dbg(chg, PR_MISC, "usb not present,not rerun\n");
		if(chg->usbsel_gpio){
		   gpio_direction_output(chg->usbsel_gpio,1);
		   smblib_dbg(chg, PR_MISC, "usbsel=%d\n",gpio_get_value(chg->usbsel_gpio));
		}
	}

	chg->apsd_rerun_retries = 0;
	smblib_dbg(chg, PR_MISC, "apsd rerun end\n");
}

static void smblib_start_hvdcp_charge_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						start_hvdcp_charge_monitor_work.work);

	if(chg->usb_present && chg->hvdcp_charge_monitor_status == WORK_STATUS_IDLE){
		chg->hvdcp_charge_monitor_status = WORK_STATUS_START;
		chg->hvdcp_5v_retries = 0;
		chg->hvdcp_9v_retries = 0;
		smblib_dbg(chg, PR_MISC, "hvdcp charge monitor work start\n");
		cancel_delayed_work(&chg->hvdcp_charge_monitor_work);
		schedule_delayed_work(&chg->hvdcp_charge_monitor_work,0);
	}
}

#define HVDCP_CHARGER_MONITER_WORK_PEROID_MS	10000
#define HVDCP_RECHECK_DELAY_MS	500
#define SWITCH_HVDCP_RETRIES_MAX 6
#define SWITCH_DCP_RETRIES_MAX 3
static void smblib_hvdcp_charge_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						hvdcp_charge_monitor_work.work);
	int rc,hvdcp_type;
	int delay = HVDCP_CHARGER_MONITER_WORK_PEROID_MS;
	union power_supply_propval is_present = {0,};
	u8 aicl_stat = 0;

	rc = smblib_get_prop_usb_present(chg, &is_present);
	if(rc < 0){
		smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
	}

	if(!is_present.intval) {
		chg->hvdcp_charge_monitor_status = WORK_STATUS_IDLE;
		chg->hvdcp_5v_retried = false;
		chg->hvdcp_5v_retries = 0;
		chg->hvdcp_9v_retries = 0;
		smblib_dbg(chg, PR_MISC, "usb not present\n");
		return;
	}

	hvdcp_type = smblib_get_hvdcp_type(chg);
	if (chg->hvdcp_charge_monitor_status == WORK_STATUS_START) {
		chg->hvdcp_charge_monitor_status = WORK_STATUS_ONGOING;
		chg->hvdcp_type = hvdcp_type;
		/* @vivo add for Dual-engine */
		power_supply_changed(chg->batt_psy);
		smblib_dbg(chg, PR_MISC, "### usbin type = %d\n", chg->hvdcp_type);
	}

	if (chg->direct_charger_enable &&
		(chg->dchg_status == DIRECT_CHARGER_PREPARE || chg->dchg_status == DIRECT_CHARGER_IS_CHARGERING)) {
		smblib_dbg(chg, PR_MISC, "direct charging, skip hvdcp_charge_monitor.\n");
		goto out;
	}

	/* check and quit collapse status */
	if ((chg->direct_charger_enable && smblib_get_handshake_status(chg)) || !chg->direct_charger_enable) {
		rc = smblib_read(chg, AICL_STATUS_REG, &aicl_stat);
		if (aicl_stat & ICL_IMIN_BIT) {
			smblib_quit_collapse_status(chg);
			delay = 5000;
			msleep(300);
			power_supply_changed(chg->batt_psy);
			goto out;
		}
	}

	if (chg->direct_charger_enable) {
		smblib_dbg(chg, PR_MISC, "direct_charger_enable, no need 5V-9V rerun.\n");
		goto out;
	}

	if (chg->hvdcp_type == HVDCP_TYPE_5V) {
		/*rerun for dcp:always 5v*/
		if (hvdcp_type == HVDCP_TYPE_5V) {
			if (!chg->hvdcp_5v_retried) {
				chg->hvdcp_5v_retries++;
				if (chg->hvdcp_5v_retries == SWITCH_DCP_RETRIES_MAX) {
					smblib_dbg(chg, PR_MISC, "try to rerun hvdcp_5v to 9v\n");

					vote(chg->pl_disable_votable, APSD_CTRL_PL_VOTER, true, 0);
					schedule_delayed_work(&chg->apsd_ctrl_pl_work,
									msecs_to_jiffies(5000));
					msleep(300);
					smblib_rerun_apsd_if_required(chg);
					chg->hvdcp_5v_retried = true;
					chg->hvdcp_5v_retries = 0;
					delay = 5000;
				}
			} else if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
				chg->hvdcp_charge_monitor_status = WORK_STATUS_IDLE;
				chg->chg_abnormal_flag |= CHG_5V_FLOAT_ERROR;
				smblib_dbg(chg, PR_MISC, "dcp change to float,retry rerun apsd\n");
				smblib_start_apsd_rerun_work(chg);
				return;
			}
		} else if (hvdcp_type == HVDCP_TYPE_9V){
			/*rerun for hvdcp 5v-9v-5v*/
			smblib_dbg(chg, PR_MISC, "rerun for hvdcp 5v->9v->5v,successful\n");
			chg->hvdcp_type = HVDCP_TYPE_9V;
			chg->hvdcp_5v_retries = 0;
			power_supply_changed(chg->batt_psy);
		}
	}

	/*rerun for hvdcp 9v-5v-9v */
	if (chg->hvdcp_type == HVDCP_TYPE_9V && !chg->charge_full_force_5v) {
		if (hvdcp_type == HVDCP_TYPE_5V) {
			if (chg->hvdcp_9v_retries < SWITCH_HVDCP_RETRIES_MAX) {
				smblib_dbg(chg, PR_MISC, "try to rerun for 9v->5v->9v\n");
				chg->hvdcp_9v_retries++;
				smblib_rerun_apsd_if_required(chg);
			} else {
				smblib_dbg(chg, PR_MISC, "rerun failed for 9v->5v->9v\n");
				if((chg->real_charger_type == POWER_SUPPLY_TYPE_USB ||
					chg->real_charger_type == POWER_SUPPLY_TYPE_USB_CDP) &&
					!chg->usb_conndoned){
					/*chg->real_charger_type = POWER_SUPPLY_TYPE_USB_DCP;*/
					chg->chg_abnormal_flag |= CHG_9V_USB_ERROR;
					vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
					smblib_dbg(chg, PR_MISC, "d+d- is ground,unrecoverable!\n");
				} else if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
					/*chg->hvdcp_charge_monitor_status = WORK_STATUS_IDLE;*/
					chg->chg_abnormal_flag |= CHG_9V_FLOAT_ERROR;
					smblib_dbg(chg, PR_MISC, "hvdcp change to float\n");
					/*smblib_start_apsd_rerun_work(chg);
					    return;
					*/
				}
			}
		} else if (hvdcp_type == HVDCP_TYPE_9V) {
			if (chg->hvdcp_9v_retries > 0) {
				smblib_dbg(chg, PR_MISC, "rerun for hvdcp 9v->5v->9v,successful\n");
				chg->hvdcp_9v_retries = 0;
				chg->hvdcp_type = HVDCP_TYPE_9V;
				power_supply_changed(chg->batt_psy);
			}
		}
	}

out:
	smblib_dbg(chg, PR_MISC, "chg->hvdcp_type=%d,hvdcp_type=%d,hvdcp_5v_retries=%d,hvdcp_9v_retries=%d\n",
			chg->hvdcp_type,hvdcp_type,chg->hvdcp_5v_retries,chg->hvdcp_9v_retries);
	schedule_delayed_work(&chg->hvdcp_charge_monitor_work,
			msecs_to_jiffies(delay));
}

static struct power_supply *get_cms_psy(struct smb_charger *chg)
{
	if (chg->cms_psy)
		return chg->cms_psy;
	chg->cms_psy = power_supply_get_by_name("cms");
	if (!chg->cms_psy)
		smblib_err(chg, "cms_psy not found\n");
	return chg->cms_psy;
}

#define USB_CONN_PRO_MONITOR_WORK_PEROID_MS	10000
static void smblib_start_usb_conn_pro_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			start_usb_conn_pro_monitor_work.work);

	if (!is_atboot && chg->usb_present && chg->usb_conn_pro_gpio &&
			chg->usb_conn_pro_monitor_status == WORK_STATUS_IDLE) {
		smblib_dbg(chg, PR_MISC, "usb conn pro monitor work start\n");
		cancel_delayed_work(&chg->usb_conn_pro_monitor_work);
		chg->usb_conn_pro_monitor_status = WORK_STATUS_START;
		chg->usb_conn_pro_flag = 0;
		chg->usb_conn_pro_monitor_count = 0;
		schedule_delayed_work(&chg->usb_conn_pro_monitor_work, 0);
	}
}
static void smblib_usb_conn_pro_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			usb_conn_pro_monitor_work.work);

	int work_period_ms = USB_CONN_PRO_MONITOR_WORK_PEROID_MS;
	struct power_supply *mcu_psy = get_mcu_psy(chg);
	struct power_supply *cms_psy = get_cms_psy(chg);
	union power_supply_propval usb_conn_temp = {0,};
	union power_supply_propval dchg_enable = {0,};
	union power_supply_propval health_status = {0,};
	union power_supply_propval is_present = {0,};
	union power_supply_propval cms_health_now = {0,};
	static int last_cms_health = 0;
	union power_supply_propval batt_temp_now = {0,};
	int rc = 0;
	int usb_trigger_threshold = 0, usb_release_threshold = 0;

	if (is_atboot) {
		smblib_err(chg, "is_atboot, exit monitor work\n");
		chg->usb_conn_pro_monitor_status = WORK_STATUS_IDLE;
		return;
	}

	if (!cms_psy) {
		smblib_err(chg, "cms_psy null,exit monitor work\n");
		chg->usb_conn_pro_monitor_status = WORK_STATUS_IDLE;
		return;
	}

	if (!chg->usb_present && chg->usb_conn_pro_flag == 0) {
		smblib_err(chg, "usb plugout,exit monitor work\n");
		last_cms_health = 0;
		chg->usb_conn_pro_monitor_count = 0;
		chg->usb_conn_pro_monitor_status = WORK_STATUS_IDLE;
		if (wake_lock_active(&chg->usb_conn_pro_wake_lock)) {
			smblib_dbg(chg, PR_MISC, "usb conn pro unlock-\n");
			wake_unlock(&chg->usb_conn_pro_wake_lock);
		}
		return;
	}
	power_supply_get_property(chg->batt_psy,POWER_SUPPLY_PROP_TEMP, &batt_temp_now);
	power_supply_get_property(cms_psy, POWER_SUPPLY_PROP_HEALTH_STATUS, &cms_health_now);
	smblib_get_usb_conn_pro_temp(chg, &usb_conn_temp);

	usb_trigger_threshold = batt_temp_now.intval >= 260 ? 700 : 600;
	usb_release_threshold = batt_temp_now.intval >= 260 ? 620 : 520;

	smblib_dbg(chg, PR_MISC, "batt_temp=%d, usb_conn_temp=%d(%d,%d), flag=%d, work_count=%d, usb_conn_pro_gpio=%d, cms_health_now=0x%x.\n",
			batt_temp_now.intval, usb_conn_temp.intval, usb_trigger_threshold, usb_release_threshold, chg->usb_conn_pro_flag,
			chg->usb_conn_pro_monitor_count, gpio_get_value(chg->usb_conn_pro_gpio), cms_health_now.intval);

	if (((cms_health_now.intval & 0x4000) && (last_cms_health & 0x4000)) || chg->usb_conn_pro_flag == 1) {	//need check this 0x4000 in cms
		work_period_ms = 300;
	}


	if (usb_conn_temp.intval >= usb_trigger_threshold && chg->usb_conn_pro_flag != 1 && (last_cms_health & 0x4000)) {
		/* cms discharge(direct & pmi), check enable */
		if (chg->direct_charger_enable && mcu_psy) {
			power_supply_get_property(mcu_psy,
					POWER_SUPPLY_PROP_DCHG_ENABLED, &dchg_enable);
			if (dchg_enable.intval != 0) {
				smblib_dbg(chg, PR_MISC, "Direct charging enable, retry\n");
				work_period_ms = 2500;
				goto retry;
			}
		}

#if 0
		msleep(200);
		/*  show tips */
		health_status.intval = 0x4000;
		power_supply_set_property(cms_psy, POWER_SUPPLY_PROP_HEALTH_STATUS, &health_status);
		power_supply_changed(cms_psy);
		msleep(200);
#endif

		/*  mark flag=1 */
		chg->usb_conn_pro_flag = 1;
		/*  gpio 21 pull up */
		gpio_direction_output(chg->usb_conn_pro_gpio, 1);
		chg->usb_vbus_pull_down_count++;	//VIVO shixiaosheng : please monitor this flag to Bigdata

		fuelsummary_collect_value(ID_BIT__VBUS_PULL, chg->usb_vbus_pull_down_count);

		/* timer T */
		chg->usb_conn_pro_monitor_count = ((8 * 60 * 1000) / 300);//8min = 8 *60*1000/300ms
		/*keep lock */
		if (!wake_lock_active(&chg->usb_conn_pro_wake_lock)) {
			smblib_dbg(chg, PR_MISC, "usb conn pro lock+\n");
			wake_lock(&chg->usb_conn_pro_wake_lock);
		}
	}

	if (chg->usb_conn_pro_flag == 1 && (usb_conn_temp.intval < usb_release_threshold ||
				chg->usb_conn_pro_monitor_count == 0)) {
		//resume charge

		/* gpio 21 pull down */
		gpio_direction_output(chg->usb_conn_pro_gpio, 0);
		/*  clear flag=1 */
		chg->usb_conn_pro_flag = 0;

		msleep(2000);
		rc = smblib_get_prop_usb_present(chg, &is_present);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
			is_present.intval = chg->usb_present;
		}
		if (!is_present.intval) {
			smblib_dbg(chg, PR_MISC, "usb not present,clear health_status and exit monitor work\n");
			chg->usb_conn_pro_monitor_count = 0;
			chg->usb_conn_pro_monitor_status = WORK_STATUS_IDLE;
			health_status.intval = 0;
			power_supply_set_property(cms_psy, POWER_SUPPLY_PROP_HEALTH_STATUS, &health_status);
			power_supply_changed(cms_psy);
		} else {
			if (chg->usb_conn_pro_monitor_count == 0) {
				smblib_dbg(chg, PR_MISC, "usb conn pro timer T timeout,exit monitor work\n");
				chg->usb_conn_pro_monitor_status = WORK_STATUS_END;
			} else {
				smblib_dbg(chg, PR_MISC, "usb conn temp become normal,restart monitor work\n");
				chg->usb_conn_pro_monitor_count = 0;
				chg->usb_conn_pro_monitor_status = WORK_STATUS_IDLE;
			}
		}
		/* release lock */
		if (wake_lock_active(&chg->usb_conn_pro_wake_lock)) {
			smblib_dbg(chg, PR_MISC, "usb conn pro unlock-\n");
			wake_unlock(&chg->usb_conn_pro_wake_lock);
		}
		last_cms_health = 0;
		return;
	}

	if (--chg->usb_conn_pro_monitor_count < 0) {
		chg->usb_conn_pro_monitor_count = 0;
	}
retry:
	last_cms_health = cms_health_now.intval;
	schedule_delayed_work(&chg->usb_conn_pro_monitor_work,
			msecs_to_jiffies(work_period_ms));
}

#define OTG_ATTACHED_MODE 1
#define AUDIO_ATTACHED_MODE 2
#define UNKNOWN_SINK_ATTACHED_MODE 3
static void smblib_otg_audio_mode_check(struct smb_charger *chg, bool enable, const char *client)
{
	if (!!chg->otg_or_audio_attach != enable) {
		if (enable) {
			if (!strncmp(client, "OTG_DRP_VOTER", 13))
				chg->otg_or_audio_attach = OTG_ATTACHED_MODE;
			else if (!strncmp(client, "AUDIO_DRP_VOTER", 15))
				chg->otg_or_audio_attach = AUDIO_ATTACHED_MODE;
			else
				chg->otg_or_audio_attach = UNKNOWN_SINK_ATTACHED_MODE;
		} else
			chg->otg_or_audio_attach = enable;

		//fuelsummary_collect_value(ID_OTG_AUDIO, chg->otg_or_audio_attach);

		smblib_dbg(chg, PR_MISC, "otg or audio mode attach -> %d\n",
				chg->otg_or_audio_attach);
	}
}

static int smblib_drp_enable_vote_callback(struct votable *votable,
				void *data, int enable, const char *client)
{
	int rc;
	struct smb_charger *chg = data;
	union power_supply_propval pval;

	if (votable != NULL)
		smblib_dbg(chg, PR_MISC, "drp_dual_mode=%d, vote_client=%s\n",
				enable, client);

	if (enable)
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL_VIVO;
	else
		pval.intval = POWER_SUPPLY_TYPEC_PR_SINK_VIVO;
	rc = smblib_set_prop_typec_power_role(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			pval.intval, rc);
		return rc;
	}

	smblib_otg_audio_mode_check(chg, enable, client);
	return 0;
}
/* vivo end */
int smblib_init(struct smb_charger *chg)
{
	union power_supply_propval prop_val;
	int rc = 0;
	char *board_version = NULL;

	mutex_init(&chg->smb_lock);
	mutex_init(&chg->irq_status_lock);
	mutex_init(&chg->dcin_aicl_lock);
	mutex_init(&chg->dpdm_lock);
	spin_lock_init(&chg->typec_pr_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->pl_update_work, pl_update_work);
	INIT_WORK(&chg->jeita_update_work, jeita_update_work);
	INIT_WORK(&chg->dcin_aicl_work, dcin_aicl_work);
	INIT_WORK(&chg->cp_status_change_work, smblib_cp_status_change_work);
	INIT_DELAYED_WORK(&chg->clear_hdc_work, clear_hdc_work);
	INIT_DELAYED_WORK(&chg->icl_change_work, smblib_icl_change_work);
	INIT_DELAYED_WORK(&chg->pl_enable_work, smblib_pl_enable_work);
	INIT_DELAYED_WORK(&chg->uusb_otg_work, smblib_uusb_otg_work);
	INIT_DELAYED_WORK(&chg->bb_removal_work, smblib_bb_removal_work);
	INIT_DELAYED_WORK(&chg->lpd_ra_open_work, smblib_lpd_ra_open_work);
	INIT_DELAYED_WORK(&chg->lpd_detach_work, smblib_lpd_detach_work);
	INIT_DELAYED_WORK(&chg->thermal_regulation_work,
					smblib_thermal_regulation_work);
	INIT_DELAYED_WORK(&chg->usbov_dbc_work, smblib_usbov_dbc_work);
	INIT_DELAYED_WORK(&chg->pr_swap_detach_work,
					smblib_pr_swap_detach_work);
	INIT_DELAYED_WORK(&chg->pr_lock_clear_work,
					smblib_pr_lock_clear_work);
	timer_setup(&chg->apsd_timer, apsd_timer_cb, 0);

	INIT_DELAYED_WORK(&chg->role_reversal_check,
					smblib_typec_role_check_work);

	if ((is_project("PD2073") || is_project("PD2073F_EX"))){
		board_version = get_bbk_board_version();
		if (board_version[0] == '1')
			ex_fg_used = EX_FG_USED;
		else
			ex_fg_used = EX_FG_NOT_USED;
	} else {
		ex_fg_used = EX_FG_NOT_USED;
	}

	if (chg->wa_flags & CHG_TERMINATION_WA) {
		INIT_WORK(&chg->chg_termination_work,
					smblib_chg_termination_work);

		if (alarmtimer_get_rtcdev()) {
			alarm_init(&chg->chg_termination_alarm, ALARM_BOOTTIME,
						chg_termination_alarm_cb);
		} else {
			smblib_err(chg, "Couldn't get rtc device\n");
			return -ENODEV;
		}
	}

	if (chg->uusb_moisture_protection_enabled) {
		INIT_WORK(&chg->moisture_protection_work,
					smblib_moisture_protection_work);

		if (alarmtimer_get_rtcdev()) {
			alarm_init(&chg->moisture_protection_alarm,
				ALARM_BOOTTIME, moisture_protection_alarm_cb);
		} else {
			smblib_err(chg, "Failed to initialize moisture protection alarm\n");
			return -ENODEV;
		}
	}

	if (alarmtimer_get_rtcdev()) {
		alarm_init(&chg->dcin_aicl_alarm, ALARM_REALTIME,
				dcin_aicl_alarm_cb);
	} else {
		smblib_err(chg, "Failed to initialize dcin aicl alarm\n");
		return -ENODEV;
	}

	chg->fake_capacity = -EINVAL;
	chg->fake_input_current_limited = -EINVAL;
	chg->fake_batt_status = -EINVAL;
	chg->sink_src_mode = UNATTACHED_MODE;
	chg->jeita_configured = false;
	chg->sec_chg_selected = POWER_SUPPLY_CHARGER_SEC_NONE;
	chg->cp_reason = POWER_SUPPLY_CP_NONE;
	chg->thermal_status = TEMP_BELOW_RANGE;
	chg->typec_irq_en = true;
	chg->cp_topo = -EINVAL;
	chg->dr_mode = TYPEC_PORT_DRP;

	/* vivo add */
	INIT_DELAYED_WORK(&chg->charge_monitor_work, smblib_charge_monitor_work);
	INIT_DELAYED_WORK(&chg->split_current_monitor_work, smblib_split_current_monitor_work);
	INIT_DELAYED_WORK(&chg->apsd_rerun_work, smblib_apsd_rerun_work);
	INIT_DELAYED_WORK(&chg->start_hvdcp_charge_monitor_work, smblib_start_hvdcp_charge_monitor_work);
	INIT_DELAYED_WORK(&chg->hvdcp_charge_monitor_work, smblib_hvdcp_charge_monitor_work);
	INIT_DELAYED_WORK(&chg->soft_term_timer_work, smblib_soft_term_timer_work);
	INIT_DELAYED_WORK(&chg->apsd_ctrl_pl_work, smblib_apsd_ctrl_pl_work);
	INIT_DELAYED_WORK(&chg->usb_psy_voter_clear_work, smblib_usb_psy_voter_clear_work);
	INIT_DELAYED_WORK(&chg->weak_charger_voter_clear_work, smblib_weak_charger_voter_clear_work);
	INIT_DELAYED_WORK(&chg->start_usb_conn_pro_monitor_work, smblib_start_usb_conn_pro_monitor_work);
	INIT_DELAYED_WORK(&chg->usb_conn_pro_monitor_work, smblib_usb_conn_pro_monitor_work);
	INIT_DELAYED_WORK(&chg->otg_monitor_work, smblib_otg_monitor_work);
	INIT_DELAYED_WORK(&chg->check_drp_disable_work, smblib_check_drp_disable_work);

	wake_lock_init(&chg->drp_disable_wake_lock, WAKE_LOCK_SUSPEND, "smblib_drp_disable_wl");

	chg->drp_enable = 0;
	chg->drp_backup_state = POWER_SUPPLY_TYPEC_PR_SINK;

	strcpy(chg->usb_type_name,smblib_apsd_results[UNKNOWN].name);
	chg->split_current_monitor_status = false;
	chg->hvdcp_type = HVDCP_TYPE_UNKNOWN;
	chg->apsd_rerun_status = WORK_STATUS_IDLE;
	chg->apsd_rerun_retries = 0;
	chg->hvdcp_charge_monitor_status = WORK_STATUS_IDLE;
	chg->hvdcp_5v_retries = 0;
	chg->hvdcp_9v_retries = 0;
	chg->chg_safety_timer_en = true;
	chg->ui_soc = -1;
	chg->apsd_rerun_first = true;
	chg->usbsel_gpio = 0;
	mutex_init(&chg->ui_soc_lock);
	chg->charge_full_force_5v = false;
	chg->charge_full_retries = 0;
	chg->usb_conn_pro_monitor_status = WORK_STATUS_IDLE;
	chg->usb_conn_pro_flag = 0;
	chg->usb_conn_pro_monitor_count = 0;
	/* add for super flash charge start*/
	chg->soc_decimal_from_user = 0;
	chg->battery_capacity_is_exception = false;
	chg->ui_soc_diff_flag = 0;
	chg->last_vbus_change_record_tm = 0;
	chg->engine = POWER_SUPPLY_ENGINE_NORMAL;
	chg->term_current_thresh_hi_ma = 0;
	/* add for super flash charge end */
	chg->keep_chg_soc = 100;
	chg->factory_mode_state = 0;
	chg->last_qg_monitor_work_count = -1;
	chg->last_dischg_change_record_tm = 0;
	chg->last_chg_drop_change_record_tm = 0;
	chg->chg_less_than_consume = false;
	chg->tracking_dischg_delta_soc = 0;
	wake_lock_init(&chg->usb_conn_pro_wake_lock, WAKE_LOCK_SUSPEND, "usb_conn_pro_wake_lock");
	wake_lock_init(&chg->chg_wake_lock, WAKE_LOCK_SUSPEND, "smblib_chg_wk");
	wake_lock_init(&chg->usb_disconnect_wake_lock, WAKE_LOCK_SUSPEND, "smblib_usb_disconnect_wk");
	/* vivo end */

	switch (chg->mode) {
	case PARALLEL_MASTER:
		rc = qcom_batt_init(&chg->chg_param);
		if (rc < 0) {
			smblib_err(chg, "Couldn't init qcom_batt_init rc=%d\n",
				rc);
			return rc;
		}

		rc = qcom_step_chg_init(chg->dev, chg->step_chg_enabled,
						chg->sw_jeita_enabled, false);
		if (rc < 0) {
			smblib_err(chg, "Couldn't init qcom_step_chg_init rc=%d\n",
				rc);
			return rc;
		}

		rc = smblib_create_votables(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		chg->bms_psy = power_supply_get_by_name("bms");

		if (chg->sec_pl_present) {
			chg->pl.psy = power_supply_get_by_name("parallel");
			if (chg->pl.psy) {
				if (chg->sec_chg_selected
					!= POWER_SUPPLY_CHARGER_SEC_CP) {
					rc = smblib_select_sec_charger(chg,
						POWER_SUPPLY_CHARGER_SEC_PL,
						POWER_SUPPLY_CP_NONE, false);
					if (rc < 0)
						smblib_err(chg, "Couldn't config pl charger rc=%d\n",
							rc);
				}

				if (chg->smb_temp_max == -EINVAL) {
					rc = smblib_get_thermal_threshold(chg,
						SMB_REG_H_THRESHOLD_MSB_REG,
						&chg->smb_temp_max);
					if (rc < 0) {
						dev_err(chg->dev, "Couldn't get charger_temp_max rc=%d\n",
								rc);
						return rc;
					}
				}

				prop_val.intval = chg->smb_temp_max;
				rc = power_supply_set_property(chg->pl.psy,
					POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
					&prop_val);
				if (rc < 0) {
					dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_CHARGER_TEMP_MAX rc=%d\n",
							rc);
					return rc;
				}
			}
		}

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}
		/*vivo add*/
		if (chg->usb_conn_pro_gpio > 0) {
			rc = gpio_request(chg->usb_conn_pro_gpio, "ucp_gpio");
			if (rc) {
				pr_err("failed to request usb_conn_pro_gpio,rc=%d\n", rc);
				return rc;
			}
			//Init usb_conn_pro_gpio state.
			gpio_direction_output(chg->usb_conn_pro_gpio, 0);
			smblib_dbg(chg, PR_MISC, "usb_conn_pro_gpio=%d\n", gpio_get_value(chg->usb_conn_pro_gpio));
		}
	
		smblib_err(chg,"bms=%s\n", chg->bms_psy_name);
		if (chg->bms_psy_name) {
			chg->bms_psy = power_supply_get_by_name(chg->bms_psy_name);
			if (!chg->bms_psy) {
				smblib_dbg(chg, PR_MISC, "get bms_psy (%s) fail\n", chg->bms_psy_name);
			}
		}

		if (!chg->bms_psy) {
			chg->bms_psy = power_supply_get_by_name("bms");
		}
		/*vivo end*/
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}
	/* add for vivo super FlashCharge */

	smblib_err(chg,"smblib_init over\n");
	/* FlashCharge end */

	return rc;
}

int smblib_deinit(struct smb_charger *chg)
{
	switch (chg->mode) {
	case PARALLEL_MASTER:
		if (chg->uusb_moisture_protection_enabled) {
			alarm_cancel(&chg->moisture_protection_alarm);
			cancel_work_sync(&chg->moisture_protection_work);
		}
		if (chg->wa_flags & CHG_TERMINATION_WA) {
			alarm_cancel(&chg->chg_termination_alarm);
			cancel_work_sync(&chg->chg_termination_work);
		}
		del_timer_sync(&chg->apsd_timer);
		cancel_work_sync(&chg->bms_update_work);
		cancel_work_sync(&chg->jeita_update_work);
		cancel_work_sync(&chg->pl_update_work);
		cancel_work_sync(&chg->dcin_aicl_work);
		cancel_work_sync(&chg->cp_status_change_work);
		cancel_delayed_work_sync(&chg->clear_hdc_work);
		cancel_delayed_work_sync(&chg->icl_change_work);
		cancel_delayed_work_sync(&chg->pl_enable_work);
		cancel_delayed_work_sync(&chg->uusb_otg_work);
		cancel_delayed_work_sync(&chg->bb_removal_work);
		cancel_delayed_work_sync(&chg->lpd_ra_open_work);
		cancel_delayed_work_sync(&chg->lpd_detach_work);
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		cancel_delayed_work_sync(&chg->usbov_dbc_work);
		cancel_delayed_work_sync(&chg->role_reversal_check);
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		power_supply_unreg_notifier(&chg->nb);
		smblib_destroy_votables(chg);
		qcom_step_chg_deinit();
		qcom_batt_deinit();
		/* vivo add */
		wake_lock_destroy(&chg->chg_wake_lock);
		wake_lock_destroy(&chg->usb_disconnect_wake_lock);
		mutex_destroy(&chg->ui_soc_lock);
		wake_lock_destroy(&chg->usb_conn_pro_wake_lock);
		/* vivo end */
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

	return 0;
}
