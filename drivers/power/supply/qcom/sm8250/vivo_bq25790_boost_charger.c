/*
 * BQ25790 battery charging driver
 *
 * Copyright (C) 2020 vivo Co.Ltd.
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#define pr_fmt(fmt)	"[bq25790] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>
#include <asm/unaligned.h>
#include <linux/jiffies.h>


extern int usbin_vbus_uv;
extern int cms_is_enable;
extern unsigned int is_atboot;

extern int pd_check_pdo(int request_vbus_mv);
extern int  pd_select_pdo_for_external_charger(int pdo);


/*Register*/
#define BQ2579X_SYS_MIN_VOLT				0x00
#define BQ2579X_SYS_MIN_VOLT_MASK                 0x3F
#define BQ2579X_SYS_MIN_VOLT_SHIFT                0
#define BQ2579X_SYS_MIN_VOLT_BASE                 2500
#define BQ2579X_SYS_MIN_VOLT_LSB                   250

#define BQ2579X_REG_CHARGE_VOLT             0x01
#define BQ2579X_VREG_MASK                         0x7FF
#define BQ2579X_VREG_SHIFT                        0
#define BQ2579X_VREG_BASE                         0
#define BQ2579X_VREG_LSB                           10

#define BQ2579X_REG_CHARGE_CURRENT                0x03
#define BQ2579X_ICHG_MASK                         0x1FF
#define BQ2579X_ICHG_SHIFT                        0
#define BQ2579X_ICHG_BASE                          0
#define BQ2579X_ICHG_LSB                            10

#define BQ2579X_REG_VINDPM                        	0x05
#define BQ2579X_VINDPM_TH_MASK                   0xFF
#define BQ2579X_VINDPM_TH_SHIFT                  0
#define BQ2579X_VINDPM_TH_BASE                   0
#define BQ2579X_VINDPM_TH_LSB                     100

#define BQ2579X_REG_IINDPM                        	0x06
#define BQ2579X_IINDPM_TH_MASK                    0x1FF
#define BQ2579X_IINDPM_TH_SHIFT                   0
#define BQ2579X_IINDPM_TH_BASE                     0
#define BQ2579X_IINDPM_TH_LSB                       10

#define BQ2579X_REG_PRECHG	                   0x08
#define BQ2579X_IPRECHG_MASK                      0x1F
#define BQ2579X_IPRECHG_SHIFT                     0
#define BQ2579X_IPRECHG_BASE                      0
#define BQ2579X_IPRECHG_LSB                       40

#define BQ2579X_REG_TERM_RST                      0x09
#define BQ2579X_ITERM_MASK                        0x1F
#define BQ2579X_ITERM_SHIFT                       0
#define BQ2579X_ITERM_BASE                        0
#define BQ2579X_ITERM_LSB                         40

#define BQ2579X_RE_CHG_CTRL                        0x0A
#define BQ2579X_RECHG_VOLT_MASK                   0x0F
#define BQ2579X_RECHG_VOLT_SHIFT                  0
#define BQ2579X_RECHG_VOLT_BASE                   50
#define BQ2579X_RECHG_VOLT_LSB                    50

#define BQ2579X_REG_OTG_VLIM_CTRL                      0x0B
#define BQ2579X_OTG_VLIM_MASK                     0x7FF
#define BQ2579X_OTG_VLIM_SHIFT                    0
#define BQ2579X_OTG_VLIM_BASE                     2800
#define BQ2579X_OTG_VLIM_LSB                       10

#define BQ2579X_REG_OTG_ILIM_CTRL                      0x0D
#define BQ2579X_OTG_ILIM_MASK                     0x7F
#define BQ2579X_OTG_ILIM_SHIFT                    0
#define BQ2579X_OTG_ILIM_BASE                     0
#define BQ2579X_OTG_ILIM_LSB                        40

#define BQ2579X_REG_Timer_CTRL                    0x0E

#define BQ2579X_REG_CHG_CTRL0                     0x0F
#define BQ2579X_CHARGE_EN_MASK                    0x20
#define BQ2579X_CHARGE_EN_SHIFT                   5
#define BQ2579X_CHARGE_DISABLE                    0
#define BQ2579X_CHARGE_ENABLE                     1
#define BQ2579X_EN_ICO_MASK                       0x10
#define BQ2579X_FORCE_ICO_MASK                    0x08
#define BQ2579X_EN_HIZ_MASK                       0x04
#define BQ2579X_EN_HIZ_SHIFT                       2
#define BQ2579X_HIZ_DISABLE                         0
#define BQ2579X_HIZ_ENABLE                          1

#define BQ2579X_REG_CHG_CTRL1                     0x10
#define BQ2579X_WDT_TIMER_MASK                    0x07
#define BQ2579X_WDT_TIMER_SHIFT                   0
#define BQ2579X_WDT_TIMER_DISABLE                 0
#define BQ2579X_WDT_TIMER_40S                     5
#define BQ2579X_WDT_TIMER_80S                     6
#define BQ2579X_WDT_TIMER_160S                   7
#define BQ2579X_WDT_RESET_MASK                    0x8
#define BQ2579X_WDT_RESET_SHIFT                   3
#define BQ2579X_WDT_RESET_DISABLE                 0
#define BQ2579X_WDT_RESET                         	1
#define BQ2579X_VAC_OVP_MASK	                  0x30
#define BQ2579X_VAC_OVP_SHIFT	                  4
#define BQ2579X_VAC_OVP_26V		                  (0 << BQ2579X_VAC_OVP_SHIFT)
#define BQ2579X_VAC_OVP_22V		                  (1 << BQ2579X_VAC_OVP_SHIFT)
#define BQ2579X_VAC_OVP_12V		                  (2 << BQ2579X_VAC_OVP_SHIFT)
#define BQ2579X_VAC_OVP_7V		                  (3 << BQ2579X_VAC_OVP_SHIFT)


#define BQ2579X_REG_CHG_CTRL2                     0x11
#define BQ2579X_DPDM_DETECT_MASK                  0x80
#define BQ2579X_DPDM_DETECT_SHIFT                 7
#define BQ2579X_DPDM_DETECT_DISABLE               0
#define BQ2579X_DPDM_DETECT_ENABLE                1
#define BQ2579X_CHG_CTRL2_EN_12V                  0x20
#define BQ2579X_CHG_CTRL2_EN_9V                   0x10
#define BQ2579X_CHG_CTRL2_HVDCP_EN                0x08


#define BQ2579X_REG_CHG_CTRL3                     0x12
#define BQ2579X_REG_CHG_EN_OTG_ENABLE             0x40
#define BQ2579X_REG_CHG_DIS_OTG_OOA		          0x02
#define BQ2579X_REG_CHG_DIS_FWD_OOA		          0x01



#define BQ2579X_REG_CHG_CTRL4                     0x13
#define BQ2579X_REG_CHG_EN_ACDRV2	              0x80
#define BQ2579X_REG_CHG_EN_ACDRV1	              0x40

#define BQ2579X_REG_CHG_CTRL5                     0x14
#define BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK     0x02	//HW ILIM PIN register connect to 1.75A
#define BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_SHIFT	  1

#define BQ2579X_REG_CHG_STATUS0                  0x1B
#define BQ2579X_REG_CHG_STATUS0_IINDPM_STAT      BIT(7)
#define BQ2579X_REG_CHG_STATUS0_VINDPM_STAT      BIT(6)
#define BQ2579X_PG_STAT_MASK                        0x8
#define BQ2579X_VBUS_PRESENT_MASK              0x1

#define BQ2579X_REG_CHG_STATUS1                  0x1C
#define BQ2579X_BC1P2_DONE_STAT_MASK        0x1
#define BQ2579X_VBUS_STAT_MASK                    0x1E
#define BQ2579X_VBUS_STAT_SHIFT                   1
#define BQ2579X_VBUS_NO_INPUT                     0
#define BQ2579X_VBUS_USB_SDP                      1
#define BQ2579X_VBUS_USB_CDP                      2
#define BQ2579X_VBUS_USB_DCP                      3
#define BQ2579X_VBUS_HV_DCP                        4
#define BQ2579X_VBUS_USB_UNKNOWN            5
#define BQ2579X_VBUS_USB_NON_STANDARD   6
#define BQ2579X_VBUS_OTG_MODE                    7
#define BQ2579X_VBUS_NO_QUALIFIED_ADAPTOR        8
#define BQ2579X_VBUS_DIRECT_POWER             11
#define BQ2579X_VBUS_BACKUP_OTG_MODE      12
#define BQ2579X_CHRG_STAT_MASK                   0xE0
#define BQ2579X_CHRG_STAT_SHIFT                  5
#define BQ2579X_CHRG_STAT_IDLE                    0
#define BQ2579X_CHRG_STAT_TRICKLE               1
#define BQ2579X_CHRG_STAT_PRECHG                  2
#define BQ2579X_CHRG_STAT_FAST                    3
#define BQ2579X_CHRG_STAT_TAPER                   4
#define BQ2579X_CHRG_STAT_RSVED                  5
#define BQ2579X_CHRG_STAT_TOPOFF                6
#define BQ2579X_CHRG_STAT_DONE                    7

#define BQ2579X_REG_CHG_STATUS2                  0x1D
#define BQ2579X_REG_CHG_STATUS3                  0x1E
#define BQ2579X_REG_CHG_STATUS4                  0x1F
#define BQ2579X_REG_FAULT_STATUS0		0x20
#define BQ2579X_REG_FAULT_STATUS1		0x21

#define BQ2579X_REG_CHG_FLAG0                     0x22
#define BQ2579X_REG_CHG_FLAG0_VINDPM_FLAG         BIT(6)

#define BQ2579X_REG_CHG_FLAG1                     0x23

#define BQ2579X_REG_CHG_FLAG2                     0x24
#define BQ2579X_DPDM_DONE_FLAG_MASK                     0x40

#define BQ2579X_REG_CHG_FLAG3                     0x25
#define BQ2579X_REG_CHG_FAULT_FLAG0         0x26
#define BQ2579X_REG_CHG_FAULT_FLAG1         0x27


#define BQ2579X_REG_CHG_INT_MASK0                 0x28
#define BQ2579X_REG_CHG_INT_MASK1                 0x29
#define BQ2579X_REG_CHG_INT_MASK2                 0x2A
#define BQ2579X_REG_CHG_INT_MASK3                 0x2B
#define BQ2579X_REG_CHG_FAULT_INT_MASK0     0x2C
#define BQ2579X_REG_CHG_FAULT_INT_MASK1     0x2D

#define BQ2579X_REG_ADC_CTRL                            0x2E
#define BQ2579X_ADC_SCAN_EN_MASK                  0x80
#define BQ2579X_ADC_SCAN_EN_SHIFT                 7
#define BQ2579X_ADC_SCAN_DISABLE                  0
#define BQ2579X_ADC_SCAN_ENABLE                   1
#define BQ2579X_ADC_SCAN_RATE_MASK                0x40
#define BQ2579X_ADC_SCAN_RATE_SHIFT               6
#define BQ2579X_ADC_SCAN_CONTINUOUS               0
#define BQ2579X_ADC_SCAN_ONESHOT                  1
#define BQ2579X_ADC_SCAN_BITS_MASK                0x30
#define BQ2579X_ADC_SCAN_BITS_SHIFT               4
#define BQ2579X_ADC_SCAN_15BITS                   0
#define BQ2579X_ADC_SCAN_14BITS                   1
#define BQ2579X_ADC_SCAN_13BITS                   2
#define BQ2579X_ADC_SCAN_12BITS                   3

#define BQ2579X_REG_IBUS_ADC                    	   0x31
#define BQ2579X_ICHG_ADC_LB_MASK                  0xFF
#define BQ2579X_ICHG_ADC_LB_SHIFT                 0
#define BQ2579X_IBUS_ADC_LB_BASE                  0
#define BQ2579X_IBUS_ADC_LB_LSB                   1

#define BQ2579X_REG_ICHG_ADC                    0x33
#define BQ2579X_ICHG_ADC_LB_MASK                  0xFF
#define BQ2579X_ICHG_ADC_LB_SHIFT                 0
#define BQ2579X_ICHG_ADC_LB_BASE                  0
#define BQ2579X_ICHG_ADC_LB_LSB                   1

#define BQ2579X_REG_VBUS_ADC                    0x35
#define BQ2579X_VBUS_ADC_LB_MASK                  0xFF
#define BQ2579X_VBUS_ADC_LB_SHIFT                 0
#define BQ2579X_VBUS_ADC_LB_BASE                  0
#define BQ2579X_VBUS_ADC_LB_LSB                   1

#define BQ2579X_REG_VBAT_ADC                    0x3B
#define BQ2579X_VBAT_ADC_LB_MASK                  0xFF
#define BQ2579X_VBAT_ADC_LB_SHIFT                 0
#define BQ2579X_VBAT_ADC_LB_BASE                  0
#define BQ2579X_VBAT_ADC_LB_LSB                   1

#define BQ2579X_REG_VSYS_ADC	                   0x3D
#define BQ2579X_VSYS_ADC_LB_MASK                  0xFF
#define BQ2579X_VSYS_ADC_LB_SHIFT                 0
#define BQ2579X_VSYS_ADC_LB_BASE                  0
#define BQ2579X_VSYS_ADC_LB_LSB                   1

#define BQ2579X_REG_TS_ADC                      	0x3F
#define BQ2579X_TS_ADC_LB_MASK                    0xFF
#define BQ2579X_TS_ADC_LB_SHIFT                   0
#define BQ2579X_TS_ADC_LB_BASE                    0
#define BQ2579X_TS_ADC_LB_LSB                     (25/256)

#define BQ2579X_REG_TDIE_ADC                    0x41
#define BQ2579X_TDIE_ADC_LB_MASK                  0xFF
#define BQ2579X_TDIE_ADC_LB_SHIFT                 0
#define BQ2579X_TDIE_ADC_LB_BASE                  0
#define BQ2579X_TDIE_ADC_LB_LSB                   (1/2)

#define BQ2579X_REG_DP_ADC                    0x43
#define BQ2579X_DP_ADC_LB_MASK                  0xFF
#define BQ2579X_DP_ADC_LB_SHIFT                 0
#define BQ2579X_DP_ADC_LB_BASE                  0
#define BQ2579X_DP_ADC_LB_LSB                    1

#define BQ2579X_REG_DM_ADC                    0x45
#define BQ2579X_DM_ADC_LB_MASK                  0xFF
#define BQ2579X_DM_ADC_LB_SHIFT                 0
#define BQ2579X_DM_ADC_LB_BASE                  0
#define BQ2579X_DM_ADC_LB_LSB                    1

#define BQ2579X_REG_DPDM_DRIVER                0x47
#define BQ2579X_REG_DP_DAC		               0xE0
#define BQ2579X_REG_DM_DAC		               0x1C


#define BQ2579X_REG_PART_NUM                      0x48
#define BQ2579X_PART_NO_MASK                      0x38
#define BQ2579X_PART_NO_SHIFT                     3
#define BQ2579X_REVISION_MASK                     0x07
#define BQ2579X_REVISION_SHIFT                    0

static const char * const chg_state[] = {
	"Idle(dis-charging)",
	"trickle(charging)",
	"prechg(charging)",
	"Fast(charging)",
	"Taper(charging)",
	"RSVED",
	"TopOFF(charging)",
	"DONE(FULL)",
};

struct bq2579x_platform_data {

	int vlim;
	int ilim;
	int ichg;
	int iprechg;

	int vreg;
	int iterm;

	int recharge_volt_offset;

	int otg_volt;
	int otg_current;
	int int_gpio;
	
};

#define	bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#define bq_log	pr_err

enum QC__dp_dm_voltage_mode {
	QC__dp_dm_voltage_mode_UNKNOW = 0,
	DP_0p6__DM_0p6 = 12,	//12V
	DP_3p3__DM_0p6 = 9,		//9V
	DP_0p6__DM_3p3 = 4,		//QC3.0 Continuous mode: direct charging
	DP_3p3__DM_3p3 = 20,	//20V
	DP_0p6__DM_0   = 5,		//5V
};

enum {
	DEFAULT	= BIT(0),
	USER	= BIT(1),
};

enum {
	BQ25790 = 0x01,
	BQ2579X = 0x02
};

enum {
	CHARGE_STATE_NOT_CHARGING,
	CHARGE_STATE_PRECHARGE,
	CHARGE_STATE_FASTCHARGE,
	CHARGE_STATE_OTG,
};

enum wakeup_src {
	WAKEUP_SRC_MONITOR = 0,
	WAKEUP_SRC_JEITA,
	WAKEUP_SRC_MAX,
};

enum int_mask {
	INT_MASK_ADC_DONE	= 0x00000080,
	INT_MASK_IINDPM		= 0x00000040,
	INT_MASK_VINDPM		= 0x00000020,
	INT_MASK_TREG		= 0x00000010,
	INT_MASK_WDT		= 0x00000008,

	INT_MASK_PG		= 0x00008000,
	INT_MASK_VBUS		= 0x00001000,
	INT_MASK_TS		= 0x00000400,
	INT_MASK_ICO		= 0x00000200,
	INT_MASK_VSYS		= 0x00000100,

	INT_MASK_VBUS_OVP	= 0x00800000,
	INT_MASK_TSHUT		= 0x00400000,
	INT_MASK_BAT_OVP	= 0x00200000,
	INT_MASK_TMR		= 0x00100000,
	INT_MASK_SYS_SHORT	= 0x00080000,
	INT_MASK_OTG		= 0x00010000,
};

struct bq2579x_otg_regulator {
	struct regulator_desc rdesc;
	struct regulator_dev *rdev;
};


struct bq2579x {
	struct device *dev;
	struct i2c_client *client;

	int part_no;
	int revision;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex profile_change_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool usb_present;

	bool power_good;
	int cable_type;
	bool bc1p2_done_stat;

	bool batt_full;

	int vbus_volt;
	int vbat_volt;
	int vsys_volt;
	int ibus_curr;
	int ichg_curr;
	int die_temp;
	int ts_temp;

	bool software_jeita_supported;
	bool jeita_active;

	bool batt_hot;
	bool batt_cold;
	bool batt_warm;
	bool batt_cool;

	int batt_hot_degc;
	int batt_warm_degc;
	int batt_cool_degc;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;

	int batt_cool_ma;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;

	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	int usb_psy_ma;
	int charge_state;
	int charging_disabled_status;


	bool charge_enabled;	/* Register bit status */
	bool otg_enabled;
	bool otg_vbus_collapse;
	int otg_vbus_collapse_count;
	bool vindpm_triggered;
	bool iindpm_triggered;

	int icl_ma;
	int ivl_mv;

	int chg_ma;
	int chg_mv;

	int fault_status;

	int dev_id;

	enum power_supply_type supply_type;

	struct delayed_work monitor_work;

	struct bq2579x_otg_regulator otg_vreg;

	struct bq2579x_platform_data *platform_data;

	struct delayed_work discharge_jeita_work; /*normal no charge mode*/
	struct delayed_work charge_jeita_work; /*charge mode jeita work*/

	struct delayed_work charge_monitor_work; /*charge mode jeita work*/

	struct delayed_work otg_monitor_work; /*OTG mode jeita work*/
	struct delayed_work weak_chg_reset_work;
	int weak_chg_count;

	struct delayed_work ico_check_work;

	struct alarm jeita_alarm;

	struct power_supply *usb_psy;
	struct power_supply *mcu_psy;
	struct power_supply *bms_psy;
	struct power_supply *batt_psy;
	struct power_supply *boost_charger_psy;
	struct power_supply_desc boost_charger_desc;

	int user_ichg;	//mA
	int ichg_spec;
	int user_ilim;	//mA
	int ilim_spec;	//mA base cable_type
	int ilim_spec_max; //after vbus connect, save the max ilim_spec value.
	int user_iterm;

	int ico_low_current_detect;

	unsigned long	cable_connect_start_time;

	int hvdcp_9V_enable;
	bool force_hvdcp_disable;
	int hvdcp_9v_try_count;
	int force_dp_dm_voltage_mode;

	bool is_pd_active;
	bool is_pd_request_success;
	bool pd_type_9v;

	struct notifier_block	psy_nb;
	int typec_mode;

	int vbat_limit_mV;
	bool is_ffc_running;

	int recharge_volt_offset;
};

enum {
	ADC_IBUS,
	ADC_ICHG,
	ADC_VBUS,
	ADC_VAC1,
	ADC_VAC2,
	ADC_VBAT,
	ADC_VSYS,
	ADC_TS,
	ADC_TDIE,
	ADC_DP,
	ADC_DM,
	ADC_MAX_NUM,
};

static void bq2579x_registers_dump(struct bq2579x *bq);
static int bq2579x_get_prop_charge_status(struct bq2579x *bq);
static int bq2579x_get_cable_type(struct bq2579x *bq);
static int bq2579x_update_charging_profile(struct bq2579x *bq);
static void bq2579x_determine_initial_status(struct bq2579x *bq);
static int bq2579x_set_vac_ovp(struct bq2579x *bq, int ovp_mv);
static int bq2579x_otg_regulator_is_enable(struct regulator_dev *rdev);
static int bq2579x_set_HVDCP_9V_control(struct bq2579x *bq, int enable);
static inline bool is_device_suspended(struct bq2579x *bq);
static int bq2579x_charge_enable(struct bq2579x *bq, bool enable);


static struct power_supply* get_power_supply(struct power_supply **psy, const char *name) {

	if (name == NULL) {
		pr_debug("name=NULL error!!!\n");
		return NULL;
	}
		
	if(!*psy)
		*psy = power_supply_get_by_name(name);

	if(!*psy) {
		pr_debug("%s power_supply not registed\n", name);
	}
	return *psy;
}

static int __bq2579x_read_reg(struct bq2579x *bq, u8 reg, u8 *data)
{
	s32 ret = 0;
	static int i2c_continue_error_count = 0;
	static long skip_i2c__start_jiffies = 0;

	if (is_device_suspended(bq)) {
		pr_debug(" is_device_suspended, skip iic read...\n");
		i2c_continue_error_count = 0;
		return -EINVAL;
	}

	if (i2c_continue_error_count > 0 && (jiffies - skip_i2c__start_jiffies) / HZ < 11) {	//11s
		bq_err("i2c error (0x%02X), skip i2c, during = %d s!!! \n", reg, (jiffies - skip_i2c__start_jiffies) / HZ);
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		if (i2c_continue_error_count++ > 3) {
			skip_i2c__start_jiffies = jiffies;
			i2c_continue_error_count = 1;
		}
		pm_relax(bq->dev);
		return ret;
	}
	i2c_continue_error_count = 0;

	*data = (u8)ret;

	return 0;
}

static int __bq2579x_write_reg(struct bq2579x *bq, int reg, u8 val)
{
	s32 ret = 0;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		bq_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		pm_relax(bq->dev);
		return ret;
	}


	return 0;
}

static int bq2579x_read_reg(struct bq2579x *bq, u8 reg, u8 *data)
{
	int ret = 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2579x_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2579x_read_word(struct bq2579x *bq, u8 reg, s16 *data)
{
	s32 ret = 0;

	/*mutex_lock(&bq->i2c_rw_lock);*/
	ret = i2c_smbus_read_word_data(bq->client, reg);
	/*mutex_unlock(&bq->i2c_rw_lock);*/
	/*
	bq_log("bq2579x read word  0x%x:0x%x\n", reg, ret, swab16(ret));
	if (ret >= 0)
		data = (s16)get_unaligned_be16(&ret);
	*/
	if(ret >= 0)
		*data = swab16(ret);

	return ret;
}

static int bq2579x_write_word(struct bq2579x *bq, u8 reg, u16 value)
{
	s32 ret = 0;

	ret = i2c_smbus_write_word_data(bq->client, reg, swab16(value));
	if (ret < 0) {
		bq_log("bq2579x write i2c fail!\n");
		return ret;
	}

	return 0;
}

static int bq2579x_write_reg(struct bq2579x *bq, u8 reg, u8 data)
{
	int ret = 0;


	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2579x_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	if (ret)
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}
static int bq2579x_update_word_bits(struct bq2579x *bq, u8 reg,
					u16 mask, u16 data)
{
	int ret = 0;
	u16 tmp;

	mutex_lock(&bq->i2c_rw_lock);
	ret = bq2579x_read_word(bq, reg, &tmp);
	if (ret < 0) {
		bq_err("Failed: reg=%02x, ret=%d\n", reg, ret);
		goto out;
	}
	tmp &= ~mask;
	tmp |= data & mask;
	/*bq_err("tmp=0x%x:0x%x,0x%x\n", reg, tmp, swab16(tmp));*/
	ret = bq2579x_write_word(bq, reg, tmp);
	if (ret < 0)
		bq_err("Failed: reg=%02x, ret=%d\n", reg, ret);
out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2579x_update_bits(struct bq2579x *bq, u8 reg,
					u8 mask, u8 data)
{
	int ret = 0;
	u8 tmp;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2579x_read_reg(bq, reg, &tmp);
	if (ret < 0) {
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2579x_write_reg(bq, reg, tmp);
	if (ret < 0)
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2579x_set_charge_voltage(struct bq2579x *bq, int volt)
{
	int ret = 0;
	u16 reg_val;

	bq->vbat_limit_mV = volt;

	if (volt < BQ2579X_VREG_BASE)
		volt = BQ2579X_VREG_BASE;

	volt -= BQ2579X_VREG_BASE;
	reg_val = volt / BQ2579X_VREG_LSB;
	reg_val <<= BQ2579X_VREG_SHIFT;

	ret = bq2579x_update_word_bits(bq, BQ2579X_REG_CHARGE_VOLT,
				BQ2579X_VREG_MASK, reg_val);

	bq_log("Vreg=%dmV (reg_val=0x%02x, ret=%d)\n", volt+BQ2579X_VREG_BASE, reg_val, ret);
	return ret;
}

static int bq2579x_set_hiz_mode(struct bq2579x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2579X_HIZ_DISABLE;
	else
		reg_val = BQ2579X_HIZ_ENABLE;

	reg_val <<= BQ2579X_EN_HIZ_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL0,
				BQ2579X_EN_HIZ_MASK, reg_val);

	bq_log("HIZ_enable=%d (reg_val=0x%02x, ret=%d)\n", enable, reg_val, ret);

	/*bq2579x_registers_dump(bq);*/
	return ret;
}


static int bq2579x_get_hiz_mode(struct bq2579x *bq, u8 *status)
{
	int ret;
	u8 reg_val;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_CTRL0, &reg_val);

	if (!ret)
		*status = !!(reg_val & BQ2579X_EN_HIZ_MASK);

	return ret;
}

static int bq2579x_en_ico_mode(struct bq2579x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = 0;
	else
		reg_val = BQ2579X_EN_ICO_MASK;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL0,
				BQ2579X_EN_ICO_MASK, reg_val);

	bq_log("EN_ICO=%d (reg_val=0x%02x, ret=%d)\n", enable, reg_val, ret);

	/*bq2579x_registers_dump(bq);*/
	return ret;
}
#if 1
static int bq2579x_force_ico(struct bq2579x *bq)
{
	int ret;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL0,
				BQ2579X_FORCE_ICO_MASK | BQ2579X_EN_ICO_MASK, BQ2579X_FORCE_ICO_MASK | BQ2579X_EN_ICO_MASK);

	bq_log("FORCE_ICO (ret=%d)\n", ret);

	/*bq2579x_registers_dump(bq);*/
	return ret;
}
#endif
/*
static int bq2579x_set_ilim_pin(struct bq2579x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2579X_ILIM_PIN_DISABLE;
	else
		reg_val = BQ2579X_ILIM_PIN_ENABLE;

	reg_val <<= BQ2579X_EN_ILIM_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHARGE_CURRENT,
				BQ2579X_EN_ILIM_MASK, reg_val);

	return ret;
}
*/
static int bq2579x_get_charge_current(struct bq2579x *bq)
{
	int ret;
	u16 reg_val = 0;
	int ichg = 0;

	ret = bq2579x_read_word(bq, BQ2579X_REG_CHARGE_CURRENT, &reg_val);

	if (ret >= 0) {
		ichg = ((reg_val & BQ2579X_ICHG_MASK) >> BQ2579X_ICHG_SHIFT ) * BQ2579X_ICHG_LSB + BQ2579X_ICHG_BASE;
	}

	bq_log("curr=%dmA (reg_val=0x%04x, ret=%d)\n", ichg, reg_val, ret);
	return ichg;
}

static int bq2579x_set_charge_current(struct bq2579x *bq, int curr)
{
	int ret;
	u16 reg_val;

	if (curr > bq->ichg_spec)
		curr = bq->ichg_spec;

	if (curr < BQ2579X_ICHG_BASE)
		curr = BQ2579X_ICHG_BASE;

	curr -= BQ2579X_ICHG_BASE;
	reg_val = curr / BQ2579X_ICHG_LSB;
	reg_val <<= BQ2579X_ICHG_SHIFT;

	ret = bq2579x_update_word_bits(bq, BQ2579X_REG_CHARGE_CURRENT,
				BQ2579X_ICHG_MASK, reg_val);

	bq_log("curr=%dmA (reg_val=0x%02x, ret=%d %s)\n", curr+BQ2579X_ICHG_BASE, reg_val, ret, ret<0? "error":"");
	return ret;
}


static int bq2579x_set_input_volt_limit(struct bq2579x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2579X_VINDPM_TH_BASE)
		volt = BQ2579X_VINDPM_TH_BASE;

	volt -= BQ2579X_VINDPM_TH_BASE;
	reg_val = volt / BQ2579X_VINDPM_TH_LSB;
	reg_val <<= BQ2579X_VINDPM_TH_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_VINDPM,
				BQ2579X_VINDPM_TH_MASK, reg_val);

	bq_log("VINDPM=%dmV (reg_val=0x%02x, ret=%d)\n", volt+BQ2579X_VINDPM_TH_BASE, reg_val, ret);
	return ret;
}

static int bq2579x_get_input_volt_limit(struct bq2579x *bq)
{
	int ret;
	u8 reg_val;
	int vindpm_mv = 0;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_VINDPM, &reg_val);
	if (!ret) {
		vindpm_mv = reg_val * BQ2579X_VINDPM_TH_LSB;
	}

	bq_log("VINDPM=%dmV (reg_val=0x%02x, ret=%d)\n", vindpm_mv, reg_val, ret);
	return vindpm_mv;
}

#if 0
static int bq2579x_set_ico_mode(struct bq2579x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2579X_ICO_DISABLE;
	else
		reg_val = BQ2579X_ICO_ENABLE;

	reg_val <<= BQ2579X_EN_ICO_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_IINDPM,
				BQ2579X_EN_ICO_MASK, reg_val);

	return ret;
}
#endif
#if 1
static int bq2579x_get_input_current_limit(struct bq2579x *bq)
{
	int ret;
	u16 reg_val = 0;
	int ilimit = 0;

	ret = bq2579x_read_word(bq, BQ2579X_REG_IINDPM, &reg_val);

	if (ret >= 0) {
		ilimit = ((reg_val & BQ2579X_IINDPM_TH_MASK) >> BQ2579X_IINDPM_TH_SHIFT ) * BQ2579X_IINDPM_TH_LSB + BQ2579X_IINDPM_TH_BASE;
	}

	bq_log("input_curr=%dmA (reg_val=0x%04x, ret=%d)\n", ilimit, reg_val, ret);
	return ilimit;
}

static int bq2579x_set_input_current_limit(struct bq2579x *bq, int curr)
{
	int ret;
	u16 reg_val;
	u16 read_back_reg_val = 0;

//	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL5, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK, 0);	//disable EXT_ILIM PIN
	//ret = bq2579x_read_word(bq, BQ2579X_REG_IINDPM, &read_back_reg_val);

	if (curr > bq->ilim_spec)
		curr = bq->ilim_spec;

	if (curr < BQ2579X_IINDPM_TH_BASE)
		curr = BQ2579X_IINDPM_TH_BASE;

	curr -= BQ2579X_IINDPM_TH_BASE;
	reg_val = curr / BQ2579X_IINDPM_TH_LSB;
	reg_val <<= BQ2579X_IINDPM_TH_SHIFT;

	//if (read_back_reg_val != reg_val) {
		ret = bq2579x_update_word_bits(bq, BQ2579X_REG_IINDPM,
					BQ2579X_IINDPM_TH_MASK, reg_val);

		ret = bq2579x_read_word(bq, BQ2579X_REG_IINDPM, &read_back_reg_val);
		bq_log("input_curr=%dmA (reg_val=0x%02x, ret=%d %s), read_back_reg_val=0x%x\n", curr+BQ2579X_IINDPM_TH_BASE, reg_val, ret, ret<0 ? "error":"", read_back_reg_val);
	//}
	return ret;
}

static int bq2579x_set_force_dpdm_detect(struct bq2579x *bq)
{
	int ret;
//	int i = 4;

	bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL5, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK);	//Enable EXT_ILIM PIN before BC1.2

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL2, BQ2579X_DPDM_DETECT_MASK, BQ2579X_DPDM_DETECT_ENABLE << BQ2579X_DPDM_DETECT_SHIFT);

#if 0
		if (bq->hvdcp_9V_enable) {
			while (i--) {
				bq2579x_set_input_current_limit(bq, 700);
				msleep(500);
				bq2579x_get_cable_type(bq);
				if (bq->cable_type == POWER_SUPPLY_TYPE_USB_HVDCP)
					break;
			}
			if (bq->usb_present && bq->hvdcp_9V_enable && bq->cable_type != POWER_SUPPLY_TYPE_USB_HVDCP) {
				//force D+=3.3V//D-=0.6V to force 9V vbus
				bq->force_dp_dm_voltage_mode = DP_3p3__DM_0p6;
				ret |= bq2579x_update_bits(bq, BQ2579X_REG_DPDM_DRIVER, BQ2579X_REG_DP_DAC | BQ2579X_REG_DM_DAC, 0xC8);
			}
		} else { 
			ret |= bq2579x_update_bits(bq, BQ2579X_REG_DPDM_DRIVER, BQ2579X_REG_DP_DAC | BQ2579X_REG_DM_DAC, 0);
			bq->force_dp_dm_voltage_mode = QC__dp_dm_voltage_mode_UNKNOW;
		}
#endif
	bq_log("bq2579x_set_force_dpdm_detect (ret=%d)\n", ret);
	return ret;
}
static void bq2579x_set_DPDM_reset_to_0_HIZ(struct bq2579x *bq)
{
	bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL2, BQ2579X_CHG_CTRL2_EN_9V | BQ2579X_CHG_CTRL2_HVDCP_EN | BQ2579X_DPDM_DETECT_MASK, 0);
	bq2579x_update_bits(bq, BQ2579X_REG_DPDM_DRIVER, BQ2579X_REG_DP_DAC | BQ2579X_REG_DM_DAC, 0x24);	//Dp=0/Dm=0
	msleep(50);
//	bq2579x_update_bits(bq, BQ2579X_REG_DPDM_DRIVER, BQ2579X_REG_DP_DAC | BQ2579X_REG_DM_DAC, 0xE4);	//dp/dm short to 0V
//	msleep(50);
	bq2579x_update_bits(bq, BQ2579X_REG_DPDM_DRIVER, BQ2579X_REG_DP_DAC | BQ2579X_REG_DM_DAC, 0);		//Dp/Dm=HiZ
	msleep(500);
	bq_log("bq2579x_set_DPDM_reset_to_0_HIZ.\n");
}

static int bq2579x_set_HVDCP_9V_control(struct bq2579x *bq, int enable)
{
	int ret;
	int i = 3;

	bq_log("bq2579x_set_HVDCP_9V_control change from %d --> %d. \n", bq->hvdcp_9V_enable, enable);
	if (bq->hvdcp_9V_enable != enable) {
		bq->hvdcp_9V_enable = enable;

		//bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL5, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK);	//Enable EXT_ILIM PIN before BC1.2
		ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL2, BQ2579X_CHG_CTRL2_EN_9V | BQ2579X_CHG_CTRL2_HVDCP_EN | BQ2579X_DPDM_DETECT_MASK, enable ? (BQ2579X_CHG_CTRL2_EN_9V | BQ2579X_CHG_CTRL2_HVDCP_EN | BQ2579X_DPDM_DETECT_MASK) : BQ2579X_DPDM_DETECT_MASK);
		bq2579x_registers_dump(bq);

		cancel_delayed_work(&bq->ico_check_work);
		schedule_delayed_work(&bq->ico_check_work, msecs_to_jiffies(1000));	//boysxs debug

		if (0) {
			if (bq->hvdcp_9V_enable > 0) {
				bq2579x_set_input_current_limit(bq, 700);	//avoid adapter vbus collospe
				while (i--) {
					msleep(500);
					bq2579x_get_cable_type(bq);
					if (bq->cable_type == POWER_SUPPLY_TYPE_USB_HVDCP)
						break;
				}
				if (bq->usb_present && bq->hvdcp_9V_enable > 0 && bq->cable_type != POWER_SUPPLY_TYPE_USB_HVDCP) {
					//force D+=3.3V//D-=0.6V to force 9V vbus
					bq->force_dp_dm_voltage_mode = DP_3p3__DM_0p6;
					ret |= bq2579x_update_bits(bq, BQ2579X_REG_DPDM_DRIVER, BQ2579X_REG_DP_DAC | BQ2579X_REG_DM_DAC, 0xC8);	//D+=3.3V//D-=0.6V
				}
			} else { 
				ret |= bq2579x_update_bits(bq, BQ2579X_REG_DPDM_DRIVER, BQ2579X_REG_DP_DAC | BQ2579X_REG_DM_DAC, 0);	//Dp/Dm=HiZ
				bq->force_dp_dm_voltage_mode = QC__dp_dm_voltage_mode_UNKNOW;
			}

			bq2579x_set_input_current_limit(bq, bq->user_ilim);	//avoid adapter vbus collospe
		}

		bq_log("bq2579x_set_HVDCP_9V_control %s, force_dp_dm_voltage_mode=%d, cable_type=%d (ret=%d)\n", enable ? "enable" : "disable", bq->force_dp_dm_voltage_mode, bq->cable_type, ret);
	}
	return ret;
}

static int bq2579x_set_hw_charge_timeout(struct bq2579x *bq, bool enable)
{
	u8 reg_val;
	int ret = 0;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_Timer_CTRL, &reg_val);
	bq_log("ret=%d, BQ2579X_REG_Timer_CTRL=0x%x. setting chg_timer %s. Start\n", ret, reg_val, enable ? "enable" : "Disable");

	if (!!(BIT(3) & reg_val) != enable) {
		bq2579x_update_bits(bq, BQ2579X_REG_Timer_CTRL, BIT(3), enable ? BIT(3) : 0);

		ret = bq2579x_read_reg(bq, BQ2579X_REG_Timer_CTRL, &reg_val);
		bq_log("ret=%d, BQ2579X_REG_Timer_CTRL=0x%x. setting chg_timer %s. End\n", ret, reg_val, enable ? "enable" : "Disable");

		if (bq->charge_enabled) {
			ret = bq2579x_charge_enable(bq, false);
			ret = bq2579x_charge_enable(bq, true);
		}
	}

	return 0;
}


static void bq2579x_PD_select_PDO(struct bq2579x *bq)
{
	int ret = 0;
	int match_pdo_num = pd_check_pdo(9000);

	ret = pd_select_pdo_for_external_charger(match_pdo_num);

	bq->is_pd_request_success = !!(ret == 0);

	if (bq->is_pd_request_success) {
		bq->ilim_spec_max = 2000;
		bq->ilim_spec = 2000;
	}
	bq_log("ret=%d. is_pd_request_success=%d\n", ret, bq->is_pd_request_success);
}

static void bq2579x_PD_reset_PDO_5V(struct bq2579x *bq)
{
	int ret = 0;
	int match_pdo_num = pd_check_pdo(5000);

	ret = pd_select_pdo_for_external_charger(match_pdo_num);

	bq_log("ret=%d\n", ret);
}

#endif
static int bq2579x_set_prechg_current(struct bq2579x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2579X_IPRECHG_BASE)
		curr = BQ2579X_IPRECHG_BASE;

	curr -= BQ2579X_IPRECHG_BASE;
	reg_val = curr / BQ2579X_IPRECHG_LSB;
	reg_val <<= BQ2579X_IPRECHG_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_PRECHG,
				BQ2579X_IPRECHG_MASK, reg_val);

	bq_log("pre_curr=%dmA (reg_val=0x%02x, ret=%d)\n", curr+BQ2579X_IPRECHG_BASE, reg_val, ret);
	return ret;
}

static int bq2579x_get_term_current(struct bq2579x *bq)
{
	int ret;
	u8 reg_val;
	int iterm = 0;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_TERM_RST, &reg_val);

	if (!ret) {
		iterm  = ((reg_val & BQ2579X_ITERM_MASK) >> BQ2579X_ITERM_SHIFT) * 50 + 50;
	} else {
		iterm = bq->user_iterm;
	}

	bq_log("term_curr=%dmA (reg_val=0x%02x, ret=%d)\n", iterm, reg_val, ret);
	return iterm;
}

static int bq2579x_set_term_current(struct bq2579x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2579X_ITERM_BASE)
		curr = BQ2579X_ITERM_BASE;

	curr -= BQ2579X_ITERM_BASE;
	reg_val = curr / BQ2579X_ITERM_LSB;
	reg_val <<= BQ2579X_ITERM_SHIFT;

	if (reg_val > BQ2579X_ITERM_MASK)
		reg_val = BQ2579X_ITERM_MASK;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_TERM_RST,
				BQ2579X_ITERM_MASK, reg_val);

	bq_log("term_curr=%dmA (reg_val=0x%02x, ret=%d)\n", curr+BQ2579X_ITERM_BASE, reg_val, ret);
	return ret;
}

static int bq2579x_set_wdt_timer(struct bq2579x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = BQ2579X_WDT_TIMER_DISABLE;
	else if (time == 40)
		reg_val = BQ2579X_WDT_TIMER_40S;
	else if (time == 80)
		reg_val = BQ2579X_WDT_TIMER_80S;
	else
		reg_val = BQ2579X_WDT_TIMER_160S;

	reg_val <<= BQ2579X_WDT_TIMER_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL1,
				BQ2579X_WDT_TIMER_MASK, reg_val);

	bq_log("wdt_timer=%ds (reg_val=0x%02x, ret=%d)\n", time, reg_val, ret);
	return ret;
}
#if 0
static int bq2579x_enable_safety_timer(struct bq2579x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2579X_SAFETY_TIMER_DISABLE;
	else
		reg_val = BQ2579X_SAFETY_TIMER_ENABLE;

	reg_val <<= BQ2579X_SAFETY_TIMER_EN_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL1,
				BQ2579X_SAFETY_TIMER_EN_MASK, reg_val);

	return ret;
}

static int bq2579x_set_safety_timer(struct bq2579x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 5)
		reg_val = BQ2579X_SAFETY_TIMER_5H;
	else if (time == 8)
		reg_val = BQ2579X_SAFETY_TIMER_8H;
	else if (time == 12)
		reg_val = BQ2579X_SAFETY_TIMER_12H;
	else
		reg_val = BQ2579X_SAFETY_TIMER_20H;

	reg_val <<= BQ2579X_SAFETY_TIMER_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL1,
				BQ2579X_SAFETY_TIMER_MASK, reg_val);

	return ret;
}
#endif
static int bq2579x_charge_enable(struct bq2579x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2579X_CHARGE_DISABLE;
	else
		reg_val = BQ2579X_CHARGE_ENABLE;

	reg_val <<= BQ2579X_CHARGE_EN_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL0,
				BQ2579X_CHARGE_EN_MASK, reg_val);

	bq_log("charger_enable=%d (reg_val=0x%02x, ret=%d)\n", enable, reg_val, ret);
	return ret;
}

static int bq2579x_set_recharge_voltage(struct bq2579x *bq, int offset)
{
	int ret;
	u8 reg_val;

	bq->recharge_volt_offset = offset;

	reg_val = ((offset - BQ2579X_RECHG_VOLT_BASE) < 0 ? 0 : (offset - BQ2579X_RECHG_VOLT_BASE)) / BQ2579X_RECHG_VOLT_LSB;

	reg_val <<= BQ2579X_RECHG_VOLT_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_RE_CHG_CTRL,
				BQ2579X_RECHG_VOLT_MASK, reg_val);

	bq_log("set_recharge_voltage_offset=%d (reg_val=0x%02x, ret=%d)\n", offset, reg_val, ret);
	return ret;
}

static int bq2579x_reset_wdt(struct bq2579x *bq)
{
	int ret;
	u8 reg_val;

	reg_val = BQ2579X_WDT_RESET << BQ2579X_WDT_RESET_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL1,
				BQ2579X_WDT_RESET_MASK, reg_val);

	return ret;
}
#if 0
static int bq2579x_set_topoff_timer(struct bq2579x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = BQ2579X_TOPOFF_TIMER_DISABLE;
	else if (time == 15)
		reg_val = BQ2579X_TOPOFF_TIMER_15M;
	else if (time == 30)
		reg_val = BQ2579X_TOPOFF_TIMER_30M;
	else
		reg_val = BQ2579X_TOPOFF_TIMER_45M;

	reg_val <<= BQ2579X_TOPOFF_TIMER_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL3,
				BQ2579X_TOPOFF_TIMER_MASK, reg_val);

	return ret;
}
#endif
static int bq2579x_set_vac_ovp(struct bq2579x *bq, int ovp_mv)
{
	int ret;
	u8 reg_val;

	if (ovp_mv >= 26000)	//sxs need check this way.
		reg_val = BQ2579X_VAC_OVP_26V;
	else if(ovp_mv >= 22000)
		reg_val = BQ2579X_VAC_OVP_22V;
	else if(ovp_mv >= 12000)
		reg_val = BQ2579X_VAC_OVP_12V;
	else
		reg_val = BQ2579X_VAC_OVP_7V;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL1,
				BQ2579X_VAC_OVP_MASK, reg_val);

	reg_val = 0;
	bq2579x_read_reg(bq, BQ2579X_REG_CHG_CTRL1, &reg_val);
	bq_log("ovp_mv=%d, reg_val=0x%x\n", ovp_mv, reg_val);
	return ret;
}

static int bq2579x_set_sys_min_volt(struct bq2579x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2579X_SYS_MIN_VOLT_BASE)
		volt = BQ2579X_SYS_MIN_VOLT_BASE;

	volt -= BQ2579X_SYS_MIN_VOLT_BASE;
	reg_val = volt / BQ2579X_SYS_MIN_VOLT_LSB;
	reg_val <<= BQ2579X_SYS_MIN_VOLT_SHIFT;
	bq_log("reg_val=%d\n", reg_val);
	ret = bq2579x_update_bits(bq, BQ2579X_SYS_MIN_VOLT,
				BQ2579X_SYS_MIN_VOLT_MASK, reg_val);

	return ret;
}

static int bq2579x_set_otg_current_limit(struct bq2579x *bq, int curr)
{
	int ret;
	u16 reg_val;

	if (curr < BQ2579X_OTG_ILIM_BASE)
		curr = BQ2579X_OTG_ILIM_BASE;

	curr -= BQ2579X_OTG_ILIM_BASE;
	reg_val = curr / BQ2579X_OTG_ILIM_LSB;
	reg_val <<= BQ2579X_OTG_ILIM_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_OTG_ILIM_CTRL,
				BQ2579X_OTG_ILIM_MASK, reg_val);

	bq_log("otg_curr=%dmA (reg_val=0x%02x, ret=%d)\n", curr+BQ2579X_OTG_ILIM_BASE, reg_val, ret);
	return ret;
}

static int bq2579x_set_otg_volt_limit(struct bq2579x *bq, int volt)
{
	int ret;
	u16 reg_val;

	if (volt < BQ2579X_OTG_VLIM_BASE)
		volt = BQ2579X_OTG_VLIM_BASE;

	volt -= BQ2579X_OTG_VLIM_BASE;
	reg_val = volt / BQ2579X_OTG_VLIM_LSB;
	reg_val <<= BQ2579X_OTG_VLIM_SHIFT;

	ret = bq2579x_update_word_bits(bq, BQ2579X_REG_OTG_VLIM_CTRL,
				BQ2579X_OTG_VLIM_MASK, reg_val);

	bq_log("otg_volt=%dmV (reg_val=0x%02x, ret=%d)\n", volt+BQ2579X_OTG_VLIM_BASE, reg_val, ret);
	return ret;
}
#if 0
static int bq2579x_get_ico_limit(struct bq2579x *bq, int *curr)
{
	int ret;
	u8 reg_val;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_ICO_LIMIT, &reg_val);
	if (ret)
		return ret;

	*curr = reg_val & BQ2579X_ICO_ILIM_MASK;
	*curr >>= BQ2579X_ICO_ILIM_SHIFT;
	*curr *= BQ2579X_ICO_ILIM_LSB;
	*curr += BQ2579X_ICO_ILIM_BASE;

	return 0;
}
#endif
/*
static int bq2579x_set_int_mask(struct bq2579x *bq, unsigned int mask)
{
	int ret;
	u8 reg_val;

	reg_val = mask & 0xFF;
	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_INT_MASK0, 0xFF, reg_val);
	if (ret < 0)
		return ret;

	reg_val = (mask >> 8) & 0xFF;
	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_INT_MASK1, 0xFF, reg_val);
	if (ret < 0)
		return ret;

	reg_val = (mask >> 16) & 0xFF;
	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_INT_MASK2, 0xFF, reg_val);
	if (ret < 0)
		return ret;

	reg_val = (mask >> 24) & 0xFF;
	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_INT_MASK3, 0xFF, reg_val);
	if (ret < 0)
		return ret;

	reg_val = (mask >> 32) & 0xFF;
	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_FAULT_INT_MASK0, 0xFF, reg_val);
	if (ret < 0)
		return ret;

	reg_val = (mask >> 40) & 0xFF;
	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_FAULT_INT_MASK1, 0xFF, reg_val);

	return ret;
}
*/
static int bq2579x_enable_adc_scan(struct bq2579x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2579X_ADC_SCAN_DISABLE;
	else
		reg_val = BQ2579X_ADC_SCAN_ENABLE;

	reg_val <<= BQ2579X_ADC_SCAN_EN_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_ADC_CTRL,
				BQ2579X_ADC_SCAN_EN_MASK, reg_val);
	return ret;
}

static bool bq2579x_is_adc_enable(struct bq2579x *bq)
{
	int ret;
	u8 value = 0;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_ADC_CTRL, &value);
	if (ret)
		return false;

	return !!(value & BQ2579X_ADC_SCAN_EN_MASK);
}

static int bq2579x_set_adc_scan_mode(struct bq2579x *bq, bool oneshot)
{
	int ret;
	u8 reg_val;

	if (oneshot == false)
		reg_val = BQ2579X_ADC_SCAN_CONTINUOUS;
	else
		reg_val = BQ2579X_ADC_SCAN_ONESHOT;

	reg_val <<= BQ2579X_ADC_SCAN_RATE_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_ADC_CTRL,
				BQ2579X_ADC_SCAN_RATE_MASK, reg_val);

	return ret;
}

static int bq2579x_set_adc_scan_bits(struct bq2579x *bq, int bits)
{
	int ret;
	u8 reg_val;

	if (bits == 15)
		reg_val = BQ2579X_ADC_SCAN_15BITS;
	else if (bits == 14)
		reg_val = BQ2579X_ADC_SCAN_14BITS;
	else if (bits == 13)
		reg_val = BQ2579X_ADC_SCAN_13BITS;
	else
		reg_val = BQ2579X_ADC_SCAN_12BITS;

	reg_val <<= BQ2579X_ADC_SCAN_BITS_SHIFT;

	ret = bq2579x_update_bits(bq, BQ2579X_REG_ADC_CTRL,
				BQ2579X_ADC_SCAN_BITS_MASK, reg_val);

	return ret;
}

#define ADC_RES_REG_BASE	0x31

static int bq2579x_read_adc_data(struct bq2579x *bq, u8 channel, int *val)
{
	int ret;
	s16 res;
	u8 reg;

	if (channel >= ADC_MAX_NUM)
		return -EINVAL;
	reg = ADC_RES_REG_BASE + (channel << 1);

	ret = bq2579x_read_word(bq, reg, &res);
	if (ret >= 0)
		*val = (int)res;

	return ret;
}

static int bq2579x_read_bus_volt(struct bq2579x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2579x_read_adc_data(bq, ADC_VBUS, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2579X_VBUS_ADC_LB_LSB;
		*volt += BQ2579X_VBUS_ADC_LB_BASE;
	}

	bq_log("volt=%d mV\n", *volt);
	return ret;
}

static int bq2579x_read_bus_curr(struct bq2579x *bq, int *curr)
{
	int ret;
	int reg_val;

	ret = bq2579x_read_adc_data(bq, ADC_IBUS, &reg_val);

	if (ret >= 0) {
		*curr = reg_val * BQ2579X_IBUS_ADC_LB_LSB;
		*curr += BQ2579X_IBUS_ADC_LB_BASE;
	}

	//bq_log("ibus=%d, ret=%d\n", *curr, ret);
	return ret;
}

static int bq2579x_read_bat_volt(struct bq2579x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2579x_read_adc_data(bq, ADC_VBAT, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2579X_VBAT_ADC_LB_LSB;
		*volt += BQ2579X_VBAT_ADC_LB_BASE;
	}

	pr_err_ratelimited("vbat=%d mV\n", *volt);
	return ret;
}

static int bq2579x_read_bat_curr(struct bq2579x *bq, int *curr)
{
	int ret;
	int reg_val;

	ret = bq2579x_read_adc_data(bq, ADC_ICHG, &reg_val);

	if (ret >= 0) {
		*curr = reg_val * BQ2579X_ICHG_ADC_LB_LSB;
		*curr += BQ2579X_ICHG_ADC_LB_BASE;
	}

	return ret;
}
#if 0
static int bq2579x_read_sys_volt(struct bq2579x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2579x_read_adc_data(bq, ADC_VSYS, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2579X_VSYS_ADC_LB_LSB;
		*volt += BQ2579X_VSYS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2579x_read_ts_temp(struct bq2579x *bq, int *temp)
{
	int ret;
	int reg_val;

	ret = bq2579x_read_adc_data(bq, ADC_TS, &reg_val);

	if (ret >= 0) {
		*temp = reg_val * 25 / 256;	//BQ2579X_TS_ADC_LB_LSB;
		*temp += BQ2579X_TS_ADC_LB_BASE;
	}

	return ret;
}
#endif

static int bq2579x_read_die_temp(struct bq2579x *bq, int *temp)
{
	int ret;
	int reg_val;

	ret = bq2579x_read_adc_data(bq, ADC_TDIE, &reg_val);

	if (ret >= 0) {
		*temp = reg_val * 1 / 2;	//BQ2579X_TDIE_ADC_LB_LSB;
		*temp += BQ2579X_TDIE_ADC_LB_BASE;
	}

	return ret;
}


static int bq2579x_charging_disable(struct bq2579x *bq, int reason,
					int disable)
{

	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	bq_log("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2579x_charge_enable(bq, false);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2579x_charge_enable(bq, true);

	if (ret) {
		bq_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
							ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	return ret;
}

#if 0
static struct power_supply *get_bms_psy(struct bq2579x *bq)
{
	if (bq->bms_psy)
		return bq->bms_psy;
	bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->bms_psy)
		pr_debug("bms power supply not found\n");

	return bq->bms_psy;
}
#endif

static int bq2579x_get_prop_charge_type(struct bq2579x *bq)
{
	u8 val = 0;

	if (is_device_suspended(bq))
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS1, &val);
	val &= BQ2579X_CHRG_STAT_MASK;
	val >>= BQ2579X_CHRG_STAT_SHIFT;

	switch (val) {
	case BQ2579X_CHRG_STAT_FAST:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2579X_CHRG_STAT_PRECHG:
	case BQ2579X_CHRG_STAT_TRICKLE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2579X_CHRG_STAT_DONE:
	case BQ2579X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	case BQ2579X_CHRG_STAT_TAPER:
	case BQ2579X_CHRG_STAT_TOPOFF:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}
#if 0
static int bq2579x_get_prop_batt_present(struct bq2579x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2579x_get_batt_property(bq,
			POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret)
		bq->batt_present = batt_prop.intval;

	return ret;

}

static int bq2579x_get_prop_batt_full(struct bq2579x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2579x_get_batt_property(bq,
			POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret)
		bq->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);

	return ret;
}
#endif
static int bq2579x_get_prop_charge_status(struct bq2579x *bq)
{
	int ret;
	u8 status;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS1, &status);
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	mutex_lock(&bq->data_lock);
	bq->charge_state = (status & BQ2579X_CHRG_STAT_MASK) >> BQ2579X_CHRG_STAT_SHIFT;
	mutex_unlock(&bq->data_lock);

	switch (bq->charge_state) {
	case BQ2579X_CHRG_STAT_TRICKLE:
	case BQ2579X_CHRG_STAT_PRECHG:
	case BQ2579X_CHRG_STAT_FAST:
	case BQ2579X_CHRG_STAT_TAPER:
	case BQ2579X_CHRG_STAT_TOPOFF:
		return POWER_SUPPLY_STATUS_CHARGING;
	case BQ2579X_CHRG_STAT_DONE:
		return POWER_SUPPLY_STATUS_FULL;
	case BQ2579X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

static enum power_supply_property bq2579x_charger_props[] = {

	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,

//	POWER_SUPPLY_PROP_VOLTAGE_MAX,	//Vreg
//	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,	//FCC charge current
//	POWER_SUPPLY_PROP_CURRENT_MAX,	//input current limit

	POWER_SUPPLY_PROP_INPUT_SUSPEND,	// suspend HIZ
	POWER_SUPPLY_PROP_CHARGING_ENABLED,	// chg enable

//	POWER_SUPPLY_PROP_RERUN_AICL,	//ICO value(VIDPM/INDPM)
//	POWER_SUPPLY_PROP_RERUN_APSD,	//force DP/DM detect for BC1.2

	POWER_SUPPLY_PROP_REAL_TYPE,
//	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
//	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHG_VOLTAGE,
};

static int bq2579x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{

	struct bq2579x *bq = power_supply_get_drvdata(psy);
	u8 status = 0;
	static int charge_status_backup = 0, charge_type_backup = 0;	// use for limit log 

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq->ichg_curr;	//mA
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		bq2579x_get_hiz_mode(bq, &status);
		val->intval = status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2579x_get_prop_charge_type(bq);
		if (charge_type_backup != val->intval)
			bq_log("POWER_SUPPLY_PROP_CHARGE_TYPE:%d --> %d\n", charge_type_backup, val->intval);
		charge_type_backup = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2579x_get_prop_charge_status(bq);
		if (charge_status_backup != val->intval)
			bq_log("POWER_SUPPLY_PROP_STATUS:%d --> %d\n", charge_status_backup, val->intval);
		charge_status_backup = val->intval;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = bq->cable_type;
		//bq_log("POWER_SUPPLY_PROP_REAL_TYPE:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:	//uA
		bq2579x_read_bus_curr(bq, &bq->ibus_curr);
		val->intval = bq->ibus_curr * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		//val->intval = bq->platform_data->vreg[0] * 1000;	//uV
		val->intval = 9000000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = bq2579x_get_term_current(bq);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:	//mA
		val->intval = bq2579x_get_input_current_limit(bq);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:	//mA
		val->intval = bq2579x_get_charge_current(bq);
		break;
	case POWER_SUPPLY_PROP_CHG_VOLTAGE:
		val->intval = 0;
		if ((bq->typec_mode > POWER_SUPPLY_TYPEC_NONE && bq->usb_present) || is_atboot) {
			if (!bq2579x_is_adc_enable(bq)) {
				bq2579x_enable_adc_scan(bq, true);
				msleep(50);
			}
			bq2579x_read_bat_volt(bq, &bq->vbat_volt);
			val->intval = bq->vbat_volt * 1000;	//return uV
		} else if (bq->typec_mode == POWER_SUPPLY_TYPEC_NONE && !bq->usb_present && bq2579x_is_adc_enable(bq))
			bq2579x_enable_adc_scan(bq, false);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 0;
		if ((bq->typec_mode > POWER_SUPPLY_TYPEC_NONE && bq->usb_present) || is_atboot) {
			if (!bq2579x_is_adc_enable(bq)) {
				bq2579x_enable_adc_scan(bq, true);
				msleep(50);
			}
			bq2579x_read_bus_volt(bq, &bq->vbus_volt);
			val->intval = bq->vbus_volt * 1000;	//return uV
		} else if (bq->typec_mode == POWER_SUPPLY_TYPEC_NONE && !bq->usb_present && bq2579x_is_adc_enable(bq))
			bq2579x_enable_adc_scan(bq, false);
		break;
	case POWER_SUPPLY_PROP_USB_OTG:
		val->intval = bq->otg_enabled;
		break;
	case POWER_SUPPLY_PROP_IS_FFC_RUNNING:
		if (bq->vbat_limit_mV >= 9000 /*&& (BQ2579X_CHRG_STAT_IDLE != bq->charge_state)*/) {
			val->intval = 1;
		} else {
			val->intval = 0;
		}
		break;
	default:
		return -EINVAL;

	}

	return 0;
}

static int bq2579x_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq2579x *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2579x_charging_disable(bq, USER, !val->intval);
		//power_supply_changed(bq->boost_charger_psy);
		bq_log("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n", val->intval ? "enable" : "disable");
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		if (!bq->otg_enabled) {
			bq2579x_set_hiz_mode(bq, !!val->intval);
			bq_log("POWER_SUPPLY_PROP_INPUT_SUSPEND: %s\n", !!val->intval ? "suspend(HIZ)" : "disable");
		} else
			bq_log("OTG enable, skip EN_HIZ setting.(%s)\n", !!val->intval ? "suspend(HIZ)" : "disable");
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:	//mA
		bq->user_ichg = val->intval;
		bq2579x_set_charge_current(bq, bq->user_ichg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (bq->ico_low_current_detect > 0) {
			bq_log("ico_low_current_detect=%d, skip it, val=%d", bq->ico_low_current_detect, val->intval);
			break;
		}

		if (val->intval > 0) {
			bq->ilim_spec_max = max(bq->ilim_spec_max, ((val->intval > bq->platform_data->ilim) ? bq->platform_data->ilim : val->intval));
			bq->ilim_spec = bq->ilim_spec_max;
			bq_log("update ilim_spec = %d, val->intval=%d", bq->ilim_spec, val->intval);
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:	//mA
		bq->user_ilim = val->intval;
		bq2579x_set_input_current_limit(bq, bq->user_ilim);
		break;
	case POWER_SUPPLY_PROP_RERUN_APSD:
		if (!bq2579x_otg_regulator_is_enable(bq->otg_vreg.rdev) && bq->hvdcp_9V_enable <= 0)	// if OTG enable, skip DPDM
			bq2579x_set_force_dpdm_detect(bq);
		else
			bq_log("skip force dpdm detect because otg_enabled=%d, hvdcp_9V_enable=%d.", bq->otg_enabled, bq->hvdcp_9V_enable);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		if (val->intval >= 50 && val->intval <= 800)
			bq->user_iterm = val->intval;
		else
			bq->user_iterm = bq->platform_data->iterm;
		break;
	case POWER_SUPPLY_PROP_CHARGER_REMOVE:
		bq_log("trigger Charger: %s\n", val->intval ? "Connect" : "disconnect");
		bq2579x_determine_initial_status(bq);
		break;
	case POWER_SUPPLY_PROP_SETTING_USBIN_OVP:
		bq2579x_set_vac_ovp(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_DIRECT_CHG_BYPASS_TO_SWITCH_CHG:
		bq->ilim_spec = val->intval > 20000 ? val->intval / 1000 : val->intval;	//need mA
		//bq2579x_set_input_current_limit(bq, bq->user_ichg);
		break;
	case POWER_SUPPLY_PROP_HVDCP_ENABLE:
		if (!bq->force_hvdcp_disable) {
			bq_log("### bq2579x_set_HVDCP_9V_control %d ###\n", val->intval);
			bq2579x_set_HVDCP_9V_control(bq, val->intval);
		} else
			bq_log("### bq2579x_set_HVDCP_9V_control %d , skip it because force_hvdcp_disable!!!###\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_DDPM_RESET:
		if (0)
			bq2579x_set_DPDM_reset_to_0_HIZ(bq);
		break;
	case POWER_SUPPLY_PROP_HW_CHG_TIMEOUT:
		bq_log("### hw charge timeout %d ###\n", val->intval);
		bq2579x_set_hw_charge_timeout(bq, !!(val->intval));
		break;
	case POWER_SUPPLY_PROP_FFC_TERMINATION_VOLTAGE:
		if (val->intval <= 0)	//reset
			ret = bq2579x_set_charge_voltage(bq, bq->platform_data->vreg);	//Vreg: 8860mV
		else
			ret = bq2579x_set_charge_voltage(bq, val->intval);	//Vbat /mV
		if (ret < 0)
			bq_err("couldn't set charge voltage ret=%d\n", ret);
		break;
	case POWER_SUPPLY_PROP_FFC_TERMINATION_CURRENT:
		if (val->intval <= 0)	//reset
			ret = bq2579x_set_term_current(bq, bq->platform_data->iterm);	// Iterm = 100mA
		else
			ret = bq2579x_set_term_current(bq, val->intval);	//ibat_term /mA
		if (ret)
			bq_err("Failed to set termination current, ret = %d\n", ret);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2579x_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int bq2579x_update_charging_profile(struct bq2579x *bq)
{
	int ilimit_spec_temp = 100;

	mutex_lock(&bq->profile_change_lock);

	if (bq->ico_low_current_detect > 0) {
		bq_err("ico_low_current_detect, skip current change\n");
		goto OUT;
	}

	if (bq->cable_type == POWER_SUPPLY_TYPE_USB) {
		ilimit_spec_temp = 500;
	} else if (bq->cable_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
		ilimit_spec_temp = 900;
	} else if (bq->cable_type == POWER_SUPPLY_TYPE_USB_CDP) {
		ilimit_spec_temp = 1500;
	} else if (bq->cable_type == POWER_SUPPLY_TYPE_USB_OCP) {
		ilimit_spec_temp = 1200;
	} else if (bq->cable_type == POWER_SUPPLY_TYPE_USB_DCP || bq->cable_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		ilimit_spec_temp = bq->platform_data->ilim;
	}
	
	bq->ilim_spec_max = max(bq->ilim_spec_max, ilimit_spec_temp);
	bq->ilim_spec = bq->ilim_spec_max;

	if (bq->weak_chg_count > 0) {
		bq->ichg_spec = max(100, bq->platform_data->ichg - (bq->weak_chg_count * 50));
		bq->ilim_spec = min(bq->ilim_spec, max(500, (bq->ichg_spec * 85 / 50)));
	}
	
	bq2579x_set_charge_current(bq, bq->user_ichg);
	bq2579x_set_input_current_limit(bq, bq->user_ilim);

	bq2579x_charging_disable(bq, DEFAULT, false);

OUT:
	mutex_unlock(&bq->profile_change_lock);

	bq_err("cable_type=%d : ilim_spec=%d(%d), user_ilim=%d; ichg_spec=%d, user_ichg=%d, weak_chg_count=%d\n", bq->cable_type, bq->ilim_spec, ilimit_spec_temp, bq->user_ilim, bq->ichg_spec, bq->user_ichg, bq->weak_chg_count);
	return 0;
}

#if 0
static void bq2579x_external_power_changed(struct power_supply *psy)
{
	struct bq2579x *bq = power_supply_get_drvdata(psy);

	union power_supply_propval prop = {0,};
	int ret, current_limit = 0;


	ret = power_supply_get_property(bq->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		bq_err("could not read USB current_max property, ret=%d\n", ret);
	else
		current_limit = prop.intval / 1000;

	bq_log("current_limit = %d\n", current_limit);

	if (bq->usb_psy_ma != current_limit) {
		bq->usb_psy_ma = current_limit;
		bq2579x_update_charging_profile(bq);
	}

	ret = power_supply_get_property(bq->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		bq_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
		bq_log("usb online status =%d\n", prop.intval);

	ret = 0;
	bq2579x_get_prop_charge_status(bq);
	if (bq->usb_present && bq->charge_state != BQ2579X_CHRG_STAT_IDLE) {
		if (prop.intval == 0) {
			bq_log("set usb online\n");
			//ret = power_supply_set_online(bq->usb_psy, true);	//sxs
		}
	} else {
		if (prop.intval == 1) {
			bq_log("set usb offline\n");
			//ret = power_supply_set_online(bq->usb_psy, false);	//sxs
		}
	}

	if (ret < 0)
		bq_err("could not set usb online state, ret=%d\n", ret);

}
#endif

static int bq25790_power_supply_notifier(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct bq2579x *bq = container_of(nb, struct bq2579x, psy_nb);
	union power_supply_propval val;
	int ret;

	if (ptr != bq->usb_psy || evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	ret = power_supply_get_property(bq->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_MODE, &val);
	if (ret) {
		bq_err("Unable to read USB TYPEC_MODE: %d\n", ret);
		return ret;
	}

	bq->typec_mode = val.intval;

	return 0;
}

static int bq2579x_psy_register(struct bq2579x *bq)
{
	int ret;
	struct power_supply_config batt_psy_cfg = {};

	bq->boost_charger_desc.name = "boost_charger";
	bq->boost_charger_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	bq->boost_charger_desc.properties = bq2579x_charger_props;
	bq->boost_charger_desc.num_properties = ARRAY_SIZE(bq2579x_charger_props);
	bq->boost_charger_desc.get_property = bq2579x_charger_get_property;
	bq->boost_charger_desc.set_property = bq2579x_charger_set_property;
//	bq->boost_charger_desc.external_power_changed = bq2579x_external_power_changed;
	bq->boost_charger_desc.property_is_writeable = bq2579x_charger_is_writeable;

	batt_psy_cfg.drv_data = bq;
	batt_psy_cfg.of_node = bq->dev->of_node;
	batt_psy_cfg.num_supplicants = 0;

	bq->boost_charger_psy = power_supply_register(bq->dev,
				&bq->boost_charger_desc,
				&batt_psy_cfg);
	if (IS_ERR(bq->boost_charger_psy)) {
		bq_err("couldn't register battery psy, ret = %ld\n",
				PTR_ERR(bq->boost_charger_psy));
		return ret;
	}

	return 0;
}

static void bq2579x_psy_unregister(struct bq2579x *bq)
{
	power_supply_unregister(bq->boost_charger_psy);
}

#if 1
static int bq2579x_otg_enable(struct bq2579x *bq, bool enable)
{
	int ret;
	u8 en_otg, en_acdrv;

	ret = bq2579x_set_hiz_mode(bq, false);

	ret = bq2579x_charge_enable(bq, !enable);

	if (enable == false) {
		en_otg = 0;
		en_acdrv = 0;
		bq->otg_vbus_collapse = false;
	} else {
		en_otg = BQ2579X_REG_CHG_EN_OTG_ENABLE | BQ2579X_REG_CHG_DIS_OTG_OOA;
		en_acdrv = BQ2579X_REG_CHG_EN_ACDRV1;
	}

	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL3,
				BQ2579X_REG_CHG_EN_OTG_ENABLE | BQ2579X_REG_CHG_DIS_OTG_OOA, en_otg);

	ret |= bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL4,
				BQ2579X_REG_CHG_EN_ACDRV1, en_acdrv);

	//bq2579x_registers_dump(bq);
	return ret;
}

static int bq2579x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2579x *bq = rdev_get_drvdata(rdev);

	if (bq->usb_present) {
		bq_err("usb_present, skip otg enable...\n");
	}

	bq->otg_enabled = true;

	cancel_delayed_work(&bq->otg_monitor_work);
	schedule_delayed_work(&bq->otg_monitor_work, msecs_to_jiffies(1000));

	ret = bq2579x_otg_enable(bq, true);
	if (ret) {
		bq->otg_enabled = false;
		bq_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq_log("bq2579x OTG mode Enabled!\n");
	}

	return ret;
}

static int bq2579x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2579x *bq = rdev_get_drvdata(rdev);

	bq->otg_enabled = false;
	bq->otg_vbus_collapse = false;
	bq->otg_vbus_collapse_count = 0;

	ret = bq2579x_otg_enable(bq, false);
	if (ret) {
		bq_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		cancel_delayed_work(&bq->otg_monitor_work);
		bq_log("bq2579x OTG mode Disabled\n");
	}

	return ret;
}


static int bq2579x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 status;
	struct bq2579x *bq = rdev_get_drvdata(rdev);

	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS1, &status);
	if (ret)
		return ret;
	status &= BQ2579X_VBUS_STAT_MASK;
	status >>= BQ2579X_VBUS_STAT_SHIFT;

	return (status == BQ2579X_VBUS_OTG_MODE) ? 1 : 0;

}

struct regulator_ops bq2579x_otg_reg_ops = {
	.enable		= bq2579x_otg_regulator_enable,
	.disable	= bq2579x_otg_regulator_disable,
	.is_enabled	= bq2579x_otg_regulator_is_enable,
};

static int bq2579x_regulator_init(struct bq2579x *bq)
{
	int ret = 0;
	struct regulator_config cfg = {};

	bq->otg_vreg.rdesc.owner = THIS_MODULE;
	bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
	bq->otg_vreg.rdesc.ops = &bq2579x_otg_reg_ops;
	bq->otg_vreg.rdesc.of_match = "ti,boost-chg-vbus";
	bq->otg_vreg.rdesc.name = "ti,otg_vbus";

	cfg.dev = bq->dev;
	cfg.driver_data = bq;
	cfg.of_node = bq->dev->of_node;

	bq->otg_vreg.rdev = devm_regulator_register(bq->dev, &bq->otg_vreg.rdesc, &cfg);
	if (IS_ERR(bq->otg_vreg.rdev)) {
		ret = PTR_ERR(bq->otg_vreg.rdev);
		bq->otg_vreg.rdev = NULL;
		if (ret != -EPROBE_DEFER)
			dev_err(bq->dev, "OTG reg failed, rc=%d\n", ret);
	}

	bq_log("regualtor name = %s, ret=%d,\n", bq->otg_vreg.rdesc.name, ret);
	return ret;
}
#endif

static struct bq2579x_platform_data *bq2579x_parse_dt(struct device *dev,
						struct bq2579x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct bq2579x_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq2579x_platform_data),
						GFP_KERNEL);
	if (!pdata)
		return NULL;

	ret = of_property_read_u32(np, "ti,bq2579x,vreg", &pdata->vreg);
	if (ret) {
		pdata->vreg = 8800;
		bq_err("Failed to read node of ti,bq2579x,vreg (%d)\n", pdata->vreg);		
	}

	ret = of_property_read_u32(np, "ti,bq2579x,termination-current", &pdata->iterm);
	if (ret) {
		pdata->iterm = 100;
		bq_err("Failed to read node of ti,bq2579x,termination-current (%d)\n", pdata->iterm);		
	}

	ret = of_property_read_u32(np, "ti,bq2579x,vlim", &pdata->vlim);
	if (ret) {
		pdata->vlim = 4400;
		bq_err("Failed to read node of ti,bq2579x,vlim (%d)\n", pdata->vlim);		
	}

	ret = of_property_read_u32(np, "ti,bq2579x,ilim", &pdata->ilim);
	if (ret) {
		pdata->ilim = 500;
		bq_err("Failed to read node of ti,bq2579x,ilim (%d)\n", pdata->ilim);		
	}

	ret = of_property_read_u32(np, "ti,bq2579x,ichg", &pdata->ichg);
	if (ret) {
		pdata->ichg = 2000;
		bq_err("Failed to read node of ti,bq2579x,ichg (%d)\n", pdata->ichg);
	}

	ret = of_property_read_u32(np, "ti,bq2579x,precharge-current", &pdata->iprechg);
	if (ret) {
		pdata->iprechg = 400;	//sxs need check the spec.
		bq_err("Failed to read node of ti,bq2579x,precharge-current (%d)\n", pdata->iprechg);
	}

	ret = of_property_read_u32(np, "ti,bq2579x,recharge-volt-offset", &pdata->recharge_volt_offset);
	if (ret) {
		pdata->recharge_volt_offset = 100;	//mv
		bq_err("Failed to read node of ti,bq2579x,recharge-volt-offset (%d)\n", pdata->recharge_volt_offset);		
	}


	ret = of_property_read_u32(np, "ti,bq2579x,otg-voltage", &pdata->otg_volt);
	if (ret) {
		pdata->otg_volt = 5100;
		bq_err("Failed to read node of ti,bq2579x,otg-voltage (%d)\n", pdata->otg_volt);
	}

	ret = of_property_read_u32(np, "ti,bq2579x,otg-current", &pdata->otg_current);
	if (ret) {
		pdata->otg_current = 1500;
		bq_err("Failed to read node of ti,bq2579x,otg-current (%d)\n", pdata->otg_current);
	}

	pdata->int_gpio = of_get_named_gpio(np, "vivo,int-gpio", 0);
	if (pdata->int_gpio < 0) {
		pr_err("failed to get int-gpio.\n");
	}

	bq_err("Ilim=%d, Ichg=%d, Iprechg=%d, Vlim=%d, otg_volt=%d, otg_curr=%d, recharge_volt_offset=%d\n",
			pdata->ilim, pdata->ichg, pdata->iprechg, pdata->vlim, pdata->otg_volt, pdata->otg_current, pdata->recharge_volt_offset);	
	return pdata;
}

static int bq2579x_init_device(struct bq2579x *bq)
{
	int ret;
	/* unsigned int mask;*/
	/*bq2579x_registers_dump(bq);*/

	bq2579x_update_bits(bq, BQ2579X_REG_DPDM_DRIVER, BQ2579X_REG_DP_DAC | BQ2579X_REG_DM_DAC, 0);	//Dp/Dm=HiZ
	bq2579x_set_wdt_timer(bq, 0);	//disable wdt first.

	ret = bq2579x_set_sys_min_volt(bq, 7000);/* set vsysmin to 7000mV */
	if (ret < 0)
		bq_err("couldn't set vsys min fail, ret=%d\n", ret);

	if (1)	//boysxs debug
		bq2579x_en_ico_mode(bq, true);	//En ICO

//	ret = bq2579x_set_input_volt_limit(bq, bq->platform_data->vlim);	//Vdpm: 4.5V
//	if (ret < 0)
//		bq_err("couldn't set input voltage limit, ret=%d\n", ret);

	ret = bq2579x_set_charge_voltage(bq, bq->platform_data->vreg);	//Vreg: 8860mV
	if (ret < 0)
		bq_err("couldn't set charge voltage ret=%d\n", ret);

	bq2579x_get_cable_type(bq);
	if (bq->cable_type != POWER_SUPPLY_TYPE_UNKNOWN)
		bq2579x_update_charging_profile(bq);
	else {
		ret = bq2579x_set_charge_current(bq, bq->platform_data->ichg);	// Ichg = 4000mA check in dtsi
		if (ret)
			bq_err("Failed to set fast current, ret = %d\n", ret);

		ret = bq2579x_set_input_current_limit(bq, bq->platform_data->ilim);	// Ilim = 2000mA
		if (ret)
			bq_err("Failed to set Ilimt current, ret = %d\n", ret);
	}

//	ret = bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL5, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK, 0);	//disable EXT_ILIM PIN

	ret = bq2579x_set_prechg_current(bq, bq->platform_data->iprechg);	//Iprechg = 400mA
	if (ret)
		bq_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq2579x_set_term_current(bq, bq->platform_data->iterm);	// Iterm = 100mA
	if (ret)
		bq_err("Failed to set termination current, ret = %d\n", ret);

	ret = bq2579x_set_otg_volt_limit(bq, bq->platform_data->otg_volt);	// Votg = 5.1V
	if (ret)
		bq_err("Failed to set otg voltage, ret = %d\n", ret);

	ret = bq2579x_set_otg_current_limit(bq, bq->platform_data->otg_current);	// Iotg = 2A
	if (ret)
		bq_err("Failed to set otg current, ret = %d\n", ret);

	bq2579x_set_hw_charge_timeout(bq, true);	//CHG_TMR default 12hour

	bq2579x_set_hiz_mode(bq, false);
	ret = bq2579x_charge_enable(bq, true);
	if (ret) {
		bq_err("Failed to enable charger, ret = %d\n", ret);
	} else {
		bq->charge_enabled = true;
		bq_log("Charger Enabled Successfully!\n");
	}

	ret = bq2579x_set_recharge_voltage(bq, bq->platform_data->recharge_volt_offset);	//recharge_volt = VREG - 200
	if (ret)
		bq_err("Failed to set_recharge_voltage, ret = %d\n", ret);
	/* bq2579x_set_ilim_pin(bq, true); */
	/*
	mask = INT_MASK_ADC_DONE | INT_MASK_VSYS | INT_MASK_TREG |
		INT_MASK_TS | INT_MASK_TSHUT | INT_MASK_SYS_SHORT |
		INT_MASK_IINDPM | INT_MASK_VINDPM;
	bq2579x_set_int_mask(bq, mask);
	*/

	bq2579x_set_adc_scan_mode(bq, false);
	bq2579x_set_adc_scan_bits(bq, 15);
	bq2579x_enable_adc_scan(bq, false);
	bq2579x_set_vac_ovp(bq, 12000);			// OVP 12V

	bq2579x_update_bits(bq, BQ2579X_REG_CHG_INT_MASK0, 0x80, 0x80);	//disable IINDPM flag

	bq2579x_registers_dump(bq);

	return 0;
}

static int bq2579x_detect_device(struct bq2579x *bq)
{
	int ret;
	u8 data;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_PART_NUM, &data);

	if (ret == 0) {
		bq->part_no = data & BQ2579X_PART_NO_MASK;
		bq->part_no >>= BQ2579X_PART_NO_SHIFT;
		bq->revision = data & BQ2579X_REVISION_MASK;
		bq->revision >>= BQ2579X_REVISION_SHIFT;
	}

	return ret;
}

static void bq2579x_update_status(struct bq2579x *bq)
{

	bq2579x_read_bus_volt(bq, &bq->vbus_volt);
	bq2579x_read_bat_volt(bq, &bq->vbat_volt);
	bq2579x_read_bus_curr(bq, &bq->ibus_curr);
	bq2579x_read_bat_curr(bq, &bq->ichg_curr);
//	bq2579x_read_ts_temp(bq, &bq->ts_temp);
	bq2579x_read_die_temp(bq, &bq->die_temp);

	bq_log("vbus:%d, vbat:%d, ibus:%d, ichg:%d, die_temp=%d.\n",
		bq->vbus_volt, bq->vbat_volt,
		bq->ibus_curr, bq->ichg_curr,
		bq->die_temp);

}

static int bq2579x_get_cable_type(struct bq2579x *bq)
{
	u8 status, vbus_stat;
	int ret;
	static int hv_dcp_error_count = 0;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS1, &status);
	if (ret) {
		bq_err("fail to read CHG_STATUS2, ret=%d.\n", ret);
		return ret;
	}

	bq->bc1p2_done_stat = !!(status & BQ2579X_BC1P2_DONE_STAT_MASK);
	vbus_stat = (status & BQ2579X_VBUS_STAT_MASK) >> BQ2579X_VBUS_STAT_SHIFT;

	if (is_atboot && bq->usb_present && !bq->bc1p2_done_stat && vbus_stat == BQ2579X_VBUS_NO_QUALIFIED_ADAPTOR && ((jiffies - bq->cable_connect_start_time) * 1000)/HZ >= 500 && cms_is_enable == 0) {
		if (bq->cable_type != POWER_SUPPLY_TYPE_USB_DCP) {
			bq->ichg_spec = bq->platform_data->ichg;
			bq->user_ichg = bq->platform_data->ichg;
			bq->ilim_spec = bq->platform_data->ilim;
			bq->user_ilim = bq->platform_data->ilim;
		}
		bq->cable_type = POWER_SUPPLY_TYPE_USB_DCP;	//force DCP for SMT test.
	} else {
		if (bq->usb_present && !bq->bc1p2_done_stat && BQ2579X_VBUS_NO_QUALIFIED_ADAPTOR == vbus_stat && ((jiffies - bq->cable_connect_start_time) * 1000)/HZ < 500) {	//<500ms
			bq->cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
			goto OUT;
		}

		switch (vbus_stat) {
			case BQ2579X_VBUS_NO_INPUT:
				bq->cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
				break;
			case BQ2579X_VBUS_NO_QUALIFIED_ADAPTOR:	//SMT will connect Vbus direct USB_IN no by Vac-MOS. force this cable to USB, for ADC
			case BQ2579X_VBUS_USB_SDP:
				bq->cable_type = POWER_SUPPLY_TYPE_USB;
				hv_dcp_error_count = 0;
				break;
			case BQ2579X_VBUS_USB_CDP:
				bq->cable_type = POWER_SUPPLY_TYPE_USB_CDP;
				break;
			case BQ2579X_VBUS_USB_DCP:
				bq->cable_type = POWER_SUPPLY_TYPE_USB_DCP;
				hv_dcp_error_count = 0;
				break;
			case BQ2579X_VBUS_HV_DCP:
				if (bq->hvdcp_9V_enable <= 0) {
					bq->cable_type = POWER_SUPPLY_TYPE_USB_DCP;
					if (((jiffies - bq->cable_connect_start_time) * 1000)/HZ >= 30000 && hv_dcp_error_count < 3) {
						hv_dcp_error_count++;
						bq2579x_set_force_dpdm_detect(bq);
					}
					bq_err("cable_type detect error to HVDCP, we change it to DCP type. %d..\n", hv_dcp_error_count);
				} else
					bq->cable_type = POWER_SUPPLY_TYPE_USB_HVDCP;
				break;
			case BQ2579X_VBUS_USB_NON_STANDARD:
				bq->cable_type = POWER_SUPPLY_TYPE_USB_OCP;
				break;
			case BQ2579X_VBUS_USB_UNKNOWN:
			//case BQ2579X_VBUS_NO_QUALIFIED_ADAPTOR:
			case BQ2579X_VBUS_DIRECT_POWER:
			default:
				if (bq->hvdcp_9V_enable > 0/*&& bq->force_dp_dm_voltage_mode*/ && bq->vbus_volt > 8000) {
					if (bq->cable_type != POWER_SUPPLY_TYPE_USB_DCP && bq->weak_chg_count <= 0) {
						bq->ichg_spec = bq->platform_data->ichg;
						bq->ilim_spec = bq->platform_data->ilim;
					}
				}
				
				bq->cable_type = POWER_SUPPLY_TYPE_USB_FLOAT;
				break;
		}
	}

OUT:
	bq_err("CHG_STATUS2=0x%02x (VBUS_STAT=%d,bc1p2_done_stat=%d), cable_type=%d, usb_present=%d, is_atboot=%d, cms_is_enable=%d, hvdcp_9V_enable=%d, force_dp_dm_voltage_mode=%d.\n", status, vbus_stat, bq->bc1p2_done_stat, bq->cable_type, bq->usb_present, is_atboot, cms_is_enable, bq->hvdcp_9V_enable, bq->force_dp_dm_voltage_mode);
	return bq->cable_type;
}

static irqreturn_t bq2579x_charger_interrupt(int irq, void *dev_id)
{
	struct bq2579x *bq = dev_id;
	u8 chg_flag0 = 0, chg_flag1 = 0, chg_flag2 = 0, chg_flag3 = 0;
	u8 chg_fault_flag0 = 0, chg_fault_flag1 = 0;
	u8 chg_status0 = 0, chg_status1 = 0, chg_status2 = 0, chg_status3 = 0, chg_status4 = 0;
	u8 fault_status0 = 0, fault_status1 = 0;
	int ret;
	int power_good = 0;
	int input_current_limit = 0, input_current_limit_turn_down_step = BQ2579X_IINDPM_TH_LSB * 5;
	static int last_cable_type = 0;


	//bq_err(" start \n");
	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_dbg(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		bq_err("irq complete!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS0, &chg_status0);
//	bq_err("BQ2579X_REG_CHG_STATUS0 value=0x%x,ret=%d!\n", chg_status0, ret);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_STATUS0 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}

	mutex_lock(&bq->data_lock);
	power_good = !!(chg_status0 & BQ2579X_PG_STAT_MASK);
	#if 0
	if(bq->power_good <= 0  && power_good <= 0) {
		bq_err("charger interrupt trigger,but power not good!");
		mutex_unlock(&bq->data_lock);
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
		
	}
	#endif

	bq->power_good = power_good;
	mutex_unlock(&bq->data_lock);

	/*1. read the interrupt Flag*/
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_FLAG0, &chg_flag0);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_FLAG0 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_FLAG1, &chg_flag1);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_FLAG1 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_FLAG2, &chg_flag2);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_FLAG2 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_FLAG3, &chg_flag3);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_FLAG3 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_FAULT_FLAG0, &chg_fault_flag0);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_FAULT_FLAG0 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_FAULT_FLAG1, &chg_fault_flag1);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_FAULT_FLAG1 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	/*2. read the interrupt status*/
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS0, &chg_status0);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_STATUS0 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}	
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS1, &chg_status1);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_STATUS1 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}	
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS2, &chg_status2);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_STATUS2 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS3, &chg_status3);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_STATUS3 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS4, &chg_status4);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_STATUS4 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_FAULT_STATUS0, &fault_status0);
	if (ret < 0) {
		bq_err("BQ2579X_REG_FAULT_STATUS0 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	ret = bq2579x_read_reg(bq, BQ2579X_REG_FAULT_STATUS1, &fault_status1);
	if (ret < 0) {
		bq_err("BQ2579X_REG_FAULT_STATUS1 read fail!");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}

	bq_err("flag[0x%02x 0x%02x 0x%02x 0x%02x],fault flag[0x%02x, 0x%02x], status[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x].\n",
		chg_flag0, chg_flag1, chg_flag2, chg_flag3, chg_fault_flag0, chg_fault_flag1, chg_status0, chg_status1, chg_status2, chg_status3, chg_status4, fault_status0, fault_status1);

	/*Check VINDPM: */
	if (bq->usb_present && (chg_flag0 & BQ2579X_REG_CHG_FLAG0_VINDPM_FLAG) && (chg_status0 & BQ2579X_REG_CHG_STATUS0_VINDPM_STAT)) {
		input_current_limit = bq2579x_get_input_current_limit(bq);
		if (input_current_limit > 500) {
			input_current_limit = input_current_limit - input_current_limit_turn_down_step;

			if (input_current_limit < 500)
				input_current_limit = 500;
			bq->ilim_spec = min(bq->ilim_spec, input_current_limit);
			bq2579x_set_input_current_limit(bq, bq->user_ilim);
			msleep(10);
			bq_err("VINDPM detect, step down input_current_limit=%d.\n", bq->ilim_spec);
		}
	}

	/*BC1.2 result change: */
	if (chg_flag2 & BQ2579X_DPDM_DONE_FLAG_MASK) {
		bq2579x_get_cable_type(bq);
		if (bq->cable_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			bq2579x_update_charging_profile(bq);
		}
		if (last_cable_type != bq->cable_type && bq->cable_type == POWER_SUPPLY_TYPE_USB)
			power_supply_changed(bq->usb_psy);

		last_cable_type = bq->cable_type;
		bq_err("Vbus_STAT change(BC1.2 result change), cable_type=%d, bc1p2_done_stat=%d\n", bq->cable_type, bq->bc1p2_done_stat);
	}

	/*Vbus interrupt change: */
	if (!bq->power_good && bq->usb_present) {
		bq->usb_present = false;
		bq2579x_set_HVDCP_9V_control(bq, false);
		bq->force_hvdcp_disable = false;
		bq->hvdcp_9v_try_count = 0;
		cancel_delayed_work(&bq->charge_monitor_work);
		cancel_delayed_work(&bq->weak_chg_reset_work);
		cancel_delayed_work(&bq->ico_check_work);
		schedule_delayed_work(&bq->weak_chg_reset_work, msecs_to_jiffies(400));
		/*bq2579x_registers_dump(bq);*/

		bq2579x_enable_adc_scan(bq, false);
		bq2579x_set_wdt_timer(bq, 0);

		if (!bq->otg_enabled && bq2579x_otg_regulator_is_enable(bq->otg_vreg.rdev)) {
			cancel_delayed_work(&bq->otg_monitor_work);
			bq2579x_otg_enable(bq, false);
			bq2579x_set_hiz_mode(bq, false);
			bq->otg_vbus_collapse = false;
			bq->otg_vbus_collapse_count = 0;
		}

		/*reset voter*/
		bq2579x_charging_disable(bq, USER, false);
		bq2579x_charging_disable(bq, DEFAULT, false);
		bq2579x_set_charge_voltage(bq, bq->platform_data->vreg);	//Vreg: 8860mV
		bq2579x_set_term_current(bq, bq->platform_data->iterm);	// Iterm = 100mA

		bq->cable_connect_start_time = 0;
		bq->charge_state = 0;	//chg_state[bq->charge_state]
		bq->cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
		bq->bc1p2_done_stat = false;
		bq->pd_type_9v = false;
		bq->is_pd_request_success = false;

		bq->user_ichg = bq->platform_data->ichg;
		bq->ichg_spec = bq->platform_data->ichg;
		bq->user_ilim = bq->platform_data->ilim;
		bq->ilim_spec = bq->platform_data->ilim;
		bq->ilim_spec_max = 0;
		bq->ico_low_current_detect = 0;
		bq2579x_set_input_volt_limit(bq, bq->platform_data->vlim);	//Vdpm: 4.6V


		bq2579x_en_ico_mode(bq, true);	//En ICO
		bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL5, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK);	//Enable EXT_ILIM PIN before BC1.2
		bq_err("#### usb removed, set usb present = %d\n", bq->usb_present);
	} else if (bq->power_good && !bq->usb_present) {
		cancel_delayed_work(&bq->weak_chg_reset_work);
		bq->weak_chg_count = min(INT_MAX, bq->weak_chg_count + 1);
		bq->usb_present = true;
		bq->cable_connect_start_time = jiffies;
		bq->ilim_spec_max = 0;

		if (bq2579x_otg_regulator_is_enable(bq->otg_vreg.rdev)) {
			//if usb_present, OTG need to disable.
			bq->otg_enabled = false;
			cancel_delayed_work(&bq->otg_monitor_work);
			bq2579x_otg_enable(bq, false);
			bq2579x_set_hiz_mode(bq, false);
			bq->otg_vbus_collapse = false;
			bq->otg_vbus_collapse_count = 0;
		}

		bq->user_ichg = bq->platform_data->ichg;
		bq->ichg_spec = bq->platform_data->ichg;
		bq->user_ilim = bq->platform_data->ilim;
		bq->ilim_spec = bq->platform_data->ilim;
		bq->ilim_spec_max = 0;

		bq2579x_set_vac_ovp(bq, 12000); //init
		bq2579x_enable_adc_scan(bq, true);
		bq2579x_get_cable_type(bq);
		if (bq->cable_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			bq2579x_update_charging_profile(bq);
		}

		bq2579x_set_wdt_timer(bq, 80);
		bq2579x_update_status(bq);
	//	bq2579x_set_input_volt_limit(bq, bq->platform_data->vlim);	//Vdpm: 4.6V

		cancel_delayed_work(&bq->charge_monitor_work);
		schedule_delayed_work(&bq->charge_monitor_work, msecs_to_jiffies(2500));

		cancel_delayed_work(&bq->ico_check_work);
		schedule_delayed_work(&bq->ico_check_work, msecs_to_jiffies(800));	//boysxs debug
		bq_err("#### usb plugged in, set usb present = %d\n", bq->usb_present);
	}

	bq2579x_update_status(bq);

	mutex_unlock(&bq->irq_complete);
	power_supply_changed(bq->boost_charger_psy);
	//bq_err(" end \n");
	return IRQ_HANDLED;
}

#define CHARGER_MONITOR_WORK_PEROID_MS	10000
static void bq2579x_charge_monitor_work(struct work_struct *work)
{
	struct bq2579x *bq = container_of(work,
			struct bq2579x, charge_monitor_work.work);
	int ichg = bq2579x_get_charge_current(bq);
	int ilim = bq2579x_get_input_current_limit(bq);
	int ret = 0;
	unsigned int work_delay_ms = CHARGER_MONITOR_WORK_PEROID_MS;
	union power_supply_propval usb_enum = {0,};
	union power_supply_propval vivo_adapter_handshake = {0,};
	union power_supply_propval capcaity = {0,}, capacity_raw = {0, };
	union power_supply_propval val = {0,}, pval = {0,};
	u8 hiz_status = 0;
	static int hvdcp_9V_waitting_count = 0;

	if (!bq->usb_present) {
		bq_err(" usb_present=%d, skip monitor work.\n", bq->usb_present);
		return;
	}

	bq2579x_get_cable_type(bq);
	bq2579x_update_status(bq);

	if (bq->hvdcp_9V_enable > 0) {
		if (bq->vbus_volt <= 7800)
			hvdcp_9V_waitting_count++;
		else {
			bq->hvdcp_9v_try_count = 0;
			hvdcp_9V_waitting_count = 0;
		}
	} else {
		hvdcp_9V_waitting_count = 0;
	}

	if (bq->vbus_volt > 7800 && bq->vbus_volt <= 10000) { //9V vbus, setting VDPM=7.8V
		if (bq2579x_get_input_volt_limit(bq) < 7800) {
			bq2579x_set_input_volt_limit(bq, 7800); 
			bq2579x_force_ico(bq);
		}
	} else {
		if (bq2579x_get_input_volt_limit(bq) != bq->platform_data->vlim) {
			bq2579x_set_input_volt_limit(bq, bq->platform_data->vlim);	//Vdpm: 4.6V
			//bq2579x_force_ico(bq);
		}
	}

	bq2579x_get_hiz_mode(bq, &hiz_status);
	power_supply_get_property(bq->usb_psy, POWER_SUPPLY_PROP_PD_ACTIVE, &val);
	bq->is_pd_active = val.intval;

	if (get_power_supply(&bq->mcu_psy, "mcu") && get_power_supply(&bq->usb_psy, "usb") && get_power_supply(&bq->batt_psy, "battery")) {
		ret = power_supply_get_property(bq->usb_psy, POWER_SUPPLY_PROP_USB_CONNDONED, &usb_enum);
		if (ret < 0)
			bq_err("could not read POWER_SUPPLY_PROP_USB_CONNDONED property, ret=%d\n", ret);
	
		ret = power_supply_get_property(bq->mcu_psy, POWER_SUPPLY_PROP_ADAPTER_HANDSHAKE, &vivo_adapter_handshake);
		if (ret < 0)
			bq_err("could not read POWER_SUPPLY_PROP_ADAPTER_HANDSHAKE property, ret=%d\n", ret);
	
		
		ret = power_supply_get_property(bq->batt_psy, POWER_SUPPLY_PROP_CAPACITY, &capcaity);
		if (ret < 0)
			bq_err("could not read POWER_SUPPLY_PROP_CAPACITY property, ret=%d\n", ret);
	}

	if (get_power_supply(&bq->bms_psy, "bq_bms")) {
		ret = power_supply_get_property(bq->bms_psy, POWER_SUPPLY_PROP_CAPACITY_RAW, &capacity_raw);
		if (ret < 0)
			bq_err("could not read POWER_SUPPLY_PROP_CAPACITY_RAW property, ret=%d\n", ret);
		
	}

	if (capcaity.intval == 100 && capacity_raw.intval == 100 && bq->vbat_volt > 8700) {
		if (bq->recharge_volt_offset != 150) {
			ret = bq2579x_set_recharge_voltage(bq, 150);	//recharge_volt = VREG - 150
			if (ret)
				bq_err("Failed to set_recharge_voltage, ret = %d\n", ret);
		}
	} else {
		if (bq->recharge_volt_offset != bq->platform_data->recharge_volt_offset) {
			ret = bq2579x_set_recharge_voltage(bq, bq->platform_data->recharge_volt_offset);
			if (ret)
				bq_err("Failed to set_recharge_voltage, ret = %d\n", ret);
		}
	}

	power_supply_get_property(bq->usb_psy, POWER_SUPPLY_PROP_PE_START, &val);
	if (!val.intval && bq->usb_present && bq->usb_psy && bq->cable_type != POWER_SUPPLY_TYPE_USB && bq->cable_type != POWER_SUPPLY_TYPE_USB_CDP && vivo_adapter_handshake.intval <= 0) {
		pval.intval = 1;
		power_supply_set_property(bq->usb_psy, POWER_SUPPLY_PROP_PE_START, &pval);	//start PD
		power_supply_changed(bq->usb_psy);
	}

	if ((vivo_adapter_handshake.intval == 1) && (bq->is_pd_request_success)) {
		bq->pd_type_9v = false;
	} else if (bq->is_pd_request_success && bq->vbus_volt > 8000) {
		bq->pd_type_9v = true;
	}
	if (bq->is_pd_active && !bq->is_pd_request_success) {
		if (get_power_supply(&bq->mcu_psy, "mcu") && get_power_supply(&bq->usb_psy, "usb") && get_power_supply(&bq->batt_psy, "battery")) {
			if ((usb_enum.intval == 0 && vivo_adapter_handshake.intval < 0) && BQ2579X_CHRG_STAT_DONE != bq->charge_state && capcaity.intval < 100)	//usb enum fail && vivo_adapter_handshake Fail
				bq2579x_PD_select_PDO(bq);
		}
	} else if (!bq->is_pd_request_success && bq->hvdcp_9V_enable <= 0 && !bq->force_hvdcp_disable && (bq->cable_type == POWER_SUPPLY_TYPE_USB_DCP || (bq->hvdcp_9v_try_count > 0 && bq->cable_type == POWER_SUPPLY_TYPE_USB_FLOAT && bq->user_ilim >= 1600))) {
		if (get_power_supply(&bq->mcu_psy, "mcu") && get_power_supply(&bq->usb_psy, "usb") && get_power_supply(&bq->batt_psy, "battery")) {
			if ((usb_enum.intval == 0 && vivo_adapter_handshake.intval < 0) && BQ2579X_CHRG_STAT_DONE != bq->charge_state && capcaity.intval < 100)	//usb enum fail && vivo_adapter_handshake Fail
				bq2579x_set_HVDCP_9V_control(bq, true);
		}
	}

	if ((bq->is_pd_request_success && bq->vbus_volt > 8000) &&
		((BQ2579X_CHRG_STAT_DONE == bq->charge_state) || (bq->charge_enabled && hiz_status == 0 && bq->vbat_volt > 8840 && bq->ichg_curr < 400 && bq->ichg_curr > 20))) {
		bq_err("BQ25790 Charge Full, try to reset to PDO 5V!!!\n");
		bq2579x_PD_reset_PDO_5V(bq);
	}

	if ((bq->hvdcp_9V_enable > 0 && bq->vbus_volt > 8000) &&
		((BQ2579X_CHRG_STAT_DONE == bq->charge_state) || (bq->charge_enabled && hiz_status == 0 && bq->vbat_volt > 8840 && bq->ichg_curr < 400 && bq->ichg_curr > 20))) {
		bq2579x_set_HVDCP_9V_control(bq, false);
		bq_err("BQ25790 Charge Full, force disable HVDCP!!!\n");
		bq->force_hvdcp_disable = true;
	}

	/*HVDCP 9V can't rise up normal, so we retry it*/
	if (hvdcp_9V_waitting_count >= 2) {
		bq2579x_set_HVDCP_9V_control(bq, false);
		bq_err("BQ25790 HVDCP 9V can't rise up normal, so we retry it!!!\n");
		if (bq->hvdcp_9v_try_count++ >= 2) {
			bq->hvdcp_9v_try_count = 2;	//try 3 times
			bq->force_hvdcp_disable = true;
		}
	}
	if (bq->hvdcp_9v_try_count && !bq->force_hvdcp_disable)
		work_delay_ms /= 3;


	bq2579x_reset_wdt(bq);

	if (!hiz_status) {
		if (abs(min(bq->ichg_spec, bq->user_ichg) - ichg) > BQ2579X_ICHG_LSB) {
			bq2579x_set_charge_current(bq, bq->user_ichg);
			bq_err(" ichg no match (ichg=%d, user_ichg=%d), force update charge current\n", ichg, bq->user_ichg);
		}

		if (abs(min(bq->ilim_spec, bq->user_ilim) - ilim) > BQ2579X_IINDPM_TH_LSB) {
			if (bq->ilim_spec > 900 && (vivo_adapter_handshake.intval > 0 || (vivo_adapter_handshake.intval < 0 && bq->force_hvdcp_disable) || (bq->hvdcp_9V_enable > 0 && bq->vbus_volt > 8000) || bq->pd_type_9v ||
				(usb_enum.intval && bq->cable_type == POWER_SUPPLY_TYPE_USB_CDP) ||
				(bq->cable_type == POWER_SUPPLY_TYPE_USB_OCP) ||
				bq->is_pd_request_success))
				bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL5, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK, 0);	//disable EXT_ILIM PIN before BC1.2
			bq2579x_set_input_current_limit(bq, bq->user_ilim);
			bq2579x_force_ico(bq);
			bq_err(" ilim no match (ilim=%d, user_ilim=%d), force update input current limit\n", ilim, bq->user_ilim);
		}
	}

	bq2579x_registers_dump(bq);

	bq_err("%4lds: is_pdactive=%d, charge_enabled=%d, chg_state=%s, hiz_status=%d, cable_type=%d(vbus_volt=%d), ilim_spec=%d, user_ilim=%d; ichg_spec=%d, user_ichg=%d, weak_chg_count=%d, usb_enum=%d, vivo_adapter_handshake=%d, capcaity=%d(%d) hvdcp_9V_enable=%d(force_HVDCP_disable=%d, hvdcp_9v_try_count=%d), force_dp_dm_voltage_mode=%d.\n",
		((jiffies - bq->cable_connect_start_time))/HZ, bq->is_pd_active,
		bq->charge_enabled, chg_state[bq->charge_state], hiz_status, bq->cable_type, bq->vbus_volt, bq->ilim_spec, bq->user_ilim, bq->ichg_spec, bq->user_ichg, bq->weak_chg_count, usb_enum.intval, vivo_adapter_handshake.intval, capcaity.intval, capacity_raw.intval, bq->hvdcp_9V_enable, bq->force_hvdcp_disable, bq->hvdcp_9v_try_count, bq->force_dp_dm_voltage_mode);

	schedule_delayed_work(&bq->charge_monitor_work, msecs_to_jiffies(work_delay_ms));
}

#define OTG_MONITOR_WORK_PEROID_MS	1000
static void bq2579x_otg_monitor_work(struct work_struct *work)
{
	struct bq2579x *bq = container_of(work,
			struct bq2579x, otg_monitor_work.work);
	static int otg_work_count = 0;
	int ret = 0;
	u8 chg_status0 = 0;
	static int iindpm_count = 0;

	bq_err("bq2579x_otg_monitor_work~\n");

	if (!bq->otg_enabled) {
		bq2579x_otg_enable(bq, false);
		bq->otg_vbus_collapse = false;
		bq->otg_vbus_collapse_count = 0;
		bq_err("disable OTG ~\n");
		return;
	}
	bq2579x_reset_wdt(bq);

	if ((otg_work_count % 10) == 0) {		
		bq2579x_registers_dump(bq);
	}

	if (otg_work_count++ >= INT_MAX)
		otg_work_count = 1;

	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS0, &chg_status0);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_STATUS0 read fail!");
	} else {
		bq_err("BQ2579X_REG_CHG_STATUS0 value=0x%x,ret=%d!\n", chg_status0, ret);
		if (chg_status0 & BQ2579X_REG_CHG_STATUS0_IINDPM_STAT) {
			if (iindpm_count++ >= 1) {
				bq->otg_vbus_collapse = true;
				bq->otg_vbus_collapse_count++;
				bq2579x_otg_enable(bq, false);
				bq_err("BQ2579X_REG_CHG_STATUS0 value=0x%x, IINDPM trigger, otg_vbus_collapse=%d, disable OTG!!\n", chg_status0, bq->otg_vbus_collapse);
			}
		} else
			iindpm_count = 0;
	}

	if (bq->otg_enabled && bq->otg_vbus_collapse && bq->otg_vbus_collapse_count < 3) {
		bq_err("restart OTG, because otg_vbus_collapse_count=%d.\n", bq->otg_vbus_collapse_count);
		bq2579x_otg_enable(bq, true);
	}

	schedule_delayed_work(&bq->otg_monitor_work, msecs_to_jiffies(OTG_MONITOR_WORK_PEROID_MS));
}

static void bq2579x_weak_chg_reset_work(struct work_struct *work)
{
	struct bq2579x *bq = container_of(work,
			struct bq2579x, weak_chg_reset_work.work);

	bq->weak_chg_count = -1;
	bq_err("cable_type=%d : ilim_spec=%d, user_ilim=%d; ichg_spec=%d, user_ichg=%d, weak_chg_count=%d\n", bq->cable_type, bq->ilim_spec, bq->user_ilim, bq->ichg_spec, bq->user_ichg, bq->weak_chg_count);
}

#define ICO_PROGRESS	1
#define ICO_DETECTED	2
static void bq2579x_ico_check_work(struct work_struct *work)
{
	struct bq2579x *bq = container_of(work,
			struct bq2579x, ico_check_work.work);
	int ret = 0;
	u8 chg_status2 = 0;
	//int vindpm_mv = 0;
	int ico_stat = 0;
	union power_supply_propval vivo_adapter_handshake = {0,};

	if (bq->mcu_psy) {
		ret = power_supply_get_property(bq->mcu_psy, POWER_SUPPLY_PROP_ADAPTER_HANDSHAKE, &vivo_adapter_handshake);
		if (ret < 0)
			bq_err("could not read POWER_SUPPLY_PROP_ADAPTER_HANDSHAKE property, ret=%d\n", ret);
		else if (vivo_adapter_handshake.intval > 0) {
			bq_err("vivo_adapter_handshake!! it is vivo adapter, skip ICO work\n");
			return;
		}
	}

	ret = bq2579x_read_reg(bq, BQ2579X_REG_CHG_STATUS2, &chg_status2);
	if (ret < 0) {
		bq_err("BQ2579X_REG_CHG_STATUS2 read fail!");
		return;
	}
	ico_stat = (chg_status2 & 0xC0) >> 6;

	if (ico_stat == ICO_PROGRESS) {
		bq2579x_get_prop_charge_status(bq);
		/*
		vindpm_mv = bq2579x_get_input_volt_limit(bq) + 100;
		if (vindpm_mv > 5000)
			vindpm_mv = 5000;
		if (vindpm_mv < bq->platform_data->vlim)
			vindpm_mv = bq->platform_data->vlim;

		ret = bq2579x_set_input_volt_limit(bq, vindpm_mv);	//Vdpm: 4.5V
		if (ret < 0)
			bq_err("couldn't set input voltage limit, ret=%d\n", ret);
		*/

		if (BQ2579X_CHRG_STAT_FAST == bq->charge_state) {
			bq->ilim_spec_max = min(bq->ilim_spec_max, 1000);
			if (bq->ilim_spec_max <= 0)
				bq->ilim_spec_max = 1000;
			bq_err("Fast charging and ICO_progress, force ilim_spec=%d-->%dmA. \n", bq->ilim_spec, bq->ilim_spec_max);
			bq->ilim_spec = bq->ilim_spec_max;
			bq->ico_low_current_detect = bq->ilim_spec;
			bq2579x_set_input_current_limit(bq, bq->user_ilim);
		}
	}
	//vindpm_mv = bq2579x_get_input_volt_limit(bq);


	bq_err("charge_state=%d, ico_stat=%d, ilim_spec=%d. \n", bq->charge_state, ico_stat, bq->ilim_spec);
	cancel_delayed_work(&bq->ico_check_work);
	/*
	if (vindpm_mv < 5000) {
		schedule_delayed_work(&bq->ico_check_work, msecs_to_jiffies(500));
	}*/
}

static void bq2579x_determine_initial_status(struct bq2579x *bq)
{
	bq_err(" \n");
	bq2579x_charger_interrupt(bq->client->irq, bq);

	if (bq->usb_present && (bq->cable_type == POWER_SUPPLY_TYPE_UNKNOWN || bq->cable_type == POWER_SUPPLY_TYPE_USB_HVDCP))	//sxs debug
		bq2579x_set_force_dpdm_detect(bq);
}
/*
Ex:
Normal read: bq2579x Reg: 0x0:0x12 0x1:0x376 0x3:0x78 0x5:0x2e 0x6:0xc8 0x8:0xca 0x9:0x3 0xa:0x63 0xb:0xe6 0xd:0x32
8bit read: bq2579x Reg: 0x0:0x12 0x1:0x3 0x2:0x76 0x3:0x0 0x4:0x78 0x5:0x2e 0x6:0x0 0x7:0xc8 0x8:0xca 0x9:0x3 0xa:0x63 0xb:0x0 0xc:0xe6 0xd:0x32
-->
0x1:0x376  ==> 0x1:0x3 0x2:0x76  big End momery.
*/
static void bq2579x_registers_dump(struct bq2579x *bq)
{
	u8 addr;
	u8 val;
	u8 buf[800];
	int idx = 0;
	int ret;
	u16 res;

	idx += scnprintf(buf + idx, PAGE_SIZE - idx, "%s: ", "bq2579x Reg");
	for (addr = 0x0; addr <= 0x48;) { 
		if((addr == 0x1 || addr == 0x3 || addr == 0x6 || addr == 0xB ||addr == 0x19) ||(addr>= 0x31 && addr <= 0x45)) {
			ret = bq2579x_read_word(bq, addr, &res);
			if (ret >= 0) {
				idx += scnprintf(buf + idx, PAGE_SIZE - idx, "0x%x:0x%x ", addr, res);
			}
			 addr += 2;
		}else {
			ret = bq2579x_read_reg(bq, addr, &val);
			if (ret == 0) {
				idx += scnprintf(buf + idx, PAGE_SIZE - idx, "0x%x:0x%x ", addr, val);
			}
			 addr++;
		}
	}
	//idx += scnprintf(buf + idx, PAGE_SIZE - idx, "\n");
	bq_err(" %s", buf);
}
static ssize_t bq2579x_show_registers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2579x *bq = power_supply_get_drvdata(psy);
	u8 addr;
	u8 val;
	int idx = 0;
	int ret;

	idx += scnprintf(buf + idx, PAGE_SIZE - idx, "%s:\n", "bq2579x Reg");
	for (addr = 0x0; addr <= 0x48; addr++) { 
		ret = bq2579x_read_reg(bq, addr, &val);
		if (ret == 0) {
			idx += scnprintf(buf + idx, PAGE_SIZE - idx, "Reg[0x%2x] = 0x%02x\n", addr, val);
		}
	}

	return idx;
}

static ssize_t bq2579x_store_registers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2579x *bq = power_supply_get_drvdata(psy);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	bq_err("reg=0x%02x, val=0x%02x.\n", reg, val);
	if (ret == 2 && reg <= 0x48)
		bq2579x_write_reg(bq, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR,
			bq2579x_show_registers,
			bq2579x_store_registers);

static struct attribute *bq2579x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2579x_attr_group = {
	.attrs = bq2579x_attributes,
};

static int bq2579x_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq2579x *bq;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;

	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		bq_err("USB supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("bq_bms");
	if (!bms_psy) {
		bq_err("bms supply not found.\n");
		//return -EPROBE_DEFER;
	}

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2579x), GFP_KERNEL);
	if (!bq) {
		bq_err("dev kzalloc error!!!\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->usb_psy = usb_psy;
	bq->bms_psy = bms_psy;

	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	ret = bq2579x_detect_device(bq);
	if (ret) {
		bq_err("No bq2579x device found!\n");
		//return -ENODEV;
	}


	if (client->dev.of_node)
		bq->platform_data = bq2579x_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;

	if (!bq->platform_data) {
		bq_err("No platform data provided.\n");
		return -EINVAL;
	}

	/*copy platform_data to driver data*/
	bq->user_ichg = bq->platform_data->ichg;
	bq->ichg_spec = bq->platform_data->ichg;
	bq->user_ilim = bq->platform_data->ilim;
	bq->ilim_spec = bq->platform_data->ilim;
	bq->hvdcp_9V_enable = -1;

	bq->psy_nb.notifier_call = bq25790_power_supply_notifier;
	power_supply_reg_notifier(&bq->psy_nb);

	bq->weak_chg_count = -1;
	ret = bq2579x_init_device(bq);
	if (ret) {
		bq_err("Failed to init device\n");
		return ret;
	}
	ret = bq2579x_psy_register(bq);
	if (ret)
		return ret;

	ret = bq2579x_regulator_init(bq);
	if (ret) {
		bq_err("Couldn't initialize bq2579x regulator ret=%d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&bq->charge_monitor_work, bq2579x_charge_monitor_work);
	INIT_DELAYED_WORK(&bq->otg_monitor_work, bq2579x_otg_monitor_work);
	INIT_DELAYED_WORK(&bq->weak_chg_reset_work, bq2579x_weak_chg_reset_work);
	INIT_DELAYED_WORK(&bq->ico_check_work, bq2579x_ico_check_work);
	if (bq->platform_data->int_gpio > 0) {
		ret = gpio_request_one(bq->platform_data->int_gpio, GPIOF_DIR_IN,"charger_int");
		if (ret) {
			pr_err("failed to request int_gpio\n");
		}
		client->irq = gpio_to_irq(bq->platform_data->int_gpio);
	}
	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL,
				bq2579x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2579x charger irq", bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret =%d\n",
				client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}
	device_init_wakeup(bq->dev, 1);
	ret = sysfs_create_group(&bq->boost_charger_psy->dev.kobj, &bq2579x_attr_group);
	if (ret)
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
	bq2579x_determine_initial_status(bq);

	bq_err("bq2579x probe successfully, Part Num:%d, Revision:%d\n",
				bq->part_no, bq->revision);

	return 0;
err_1:
	bq2579x_psy_unregister(bq);
	if (bq->otg_vreg.rdev)
		devm_regulator_unregister(bq->dev, bq->otg_vreg.rdev);

	return ret;
}


static inline bool is_device_suspended(struct bq2579x *bq)
{
	return !bq->resume_completed;
}

static int bq2579x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2579x *bq = i2c_get_clientdata(client);
	int ret = 0;

	if (bq->typec_mode == POWER_SUPPLY_TYPEC_NONE && !bq->usb_present && bq2579x_is_adc_enable(bq)) {
		ret = bq2579x_enable_adc_scan(bq, false);
		bq_err("disable adc. ret=%d..\n", ret);
	}

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq2579x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2579x *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq2579x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2579x *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq2579x_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

//	power_supply_changed(bq->boost_charger_psy);

	return 0;
}
static int bq2579x_charger_remove(struct i2c_client *client)
{
	struct bq2579x *bq = i2c_get_clientdata(client);

	//alarm_try_to_cancel(&bq->jeita_alarm);

	//cancel_delayed_work_sync(&bq->charge_jeita_work);
	//cancel_delayed_work_sync(&bq->discharge_jeita_work);

	if (bq->otg_vreg.rdev)
		regulator_unregister(bq->otg_vreg.rdev);

	bq2579x_psy_unregister(bq);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);

	sysfs_remove_group(&bq->dev->kobj, &bq2579x_attr_group);

	return 0;
}


static void bq2579x_charger_shutdown(struct i2c_client *client)
{
	struct bq2579x *bq = i2c_get_clientdata(client);

	bq2579x_set_wdt_timer(bq, 40);
	bq2579x_enable_adc_scan(bq, false);
	bq2579x_charge_enable(bq, true);
	bq2579x_set_hiz_mode(bq, false);
	bq2579x_set_HVDCP_9V_control(bq, false);

	bq2579x_set_charge_current(bq, 2000);	// Ibat = 2A
	bq2579x_set_input_current_limit(bq, 500);	// Ilim = 500mA

	bq2579x_update_bits(bq, BQ2579X_REG_CHG_CTRL5, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK, BQ2579X_REG_CHG_CTRL5_EN_EXTILIM_MASK);	//Enable EXT_ILIM PIN
	dev_err(bq->dev, " bq2579x_charger_shutdown shutdown done!!!\n");
}

static const struct of_device_id bq2579x_charger_match_table[] = {
	{.compatible = "ti,bq25790-charger",},
	{.compatible = "ti,bq2579X-charger",},
	{},
};
MODULE_DEVICE_TABLE(of, bq2579x_charger_match_table);

static const struct i2c_device_id bq2579x_charger_id[] = {
	{ "bq25790-charger", BQ25790 },
	{ "bq2579X-charger", BQ2579X },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2579x_charger_id);

static const struct dev_pm_ops bq2579x_pm_ops = {
	.resume		= bq2579x_resume,
	.suspend_noirq = bq2579x_suspend_noirq,
	.suspend	= bq2579x_suspend,
};
static struct i2c_driver bq2579x_charger_driver = {
	.driver	= {
		.name	= "bq2579x-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq2579x_charger_match_table,
		.pm		= &bq2579x_pm_ops,
	},
	.id_table	= bq2579x_charger_id,

	.probe		= bq2579x_charger_probe,
	.remove		= bq2579x_charger_remove,
	.shutdown	= bq2579x_charger_shutdown,
};

module_i2c_driver(bq2579x_charger_driver);

MODULE_DESCRIPTION("TI BQ2579x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("VIVO_CHG");
