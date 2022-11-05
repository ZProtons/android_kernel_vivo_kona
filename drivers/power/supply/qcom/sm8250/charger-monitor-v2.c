/*
 * Copyright (C) 2017 VIVO Co., Ltd.
 * YangZhiHui <yangzhihui@vivo.com.cn>
 *
 * This driver is used to monitor charging procedure. CMS = Charging Monitor System
 *
 */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
/*platform specified headers*/
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/batterydata-lib.h>
#include <linux/jiffies.h>
#include <linux/sched/clock.h>
#include <linux/pmic-voter.h>
#include "charger-monitor-v2.h"


/*******************
 * cms  var statement  *
 *******************/
extern unsigned int is_atboot;
extern unsigned int power_off_charging_mode;
extern unsigned int bsp_test_mode;
extern bool ex_fg_ffc_support;
extern bool ex_fg_support;
extern bool ex_fg_power_on_i2c_try_success;

static int cms_is_enable = 1;
static int __debug_mask = PR_INFO | PR_ERROR | PR_WARN;

//QCOM suggest sechedule delayed work on WQ_UNBOUND flag
static inline bool schedule_delayed_work__on_system_unbound_wq(struct delayed_work *dwork,
					 unsigned long delay)
{
	return queue_delayed_work(system_unbound_wq, dwork, delay);
}

/***********************
 *  cms  function statement  *
 ***********************/
 #if 1
 extern bool is_panel_backlight_on(void);
 #else
 inline bool is_panel_backlight_on(void) { return 1; };
 #endif
static void cms_reset_voters(struct cms *chip);
static bool cms_check_fastchg_current(struct cms *chip);
static bool get_fuelsummary_user_custom(struct cms *chip);
static int cms_init_adc(struct cms *chip);
static void cms_notify_changed(struct cms *chip, int bit, bool val);
static void cms_check_chg_error_state(struct cms *chip);

static const char* const map_state_string(int state, const char* const* str, int array_size)
{
	if (!str) {
		pr_err("str is NULL! Return empty string\n");
		return "";
	}
	if (state < 0 || state >= array_size) {
		pr_err("state[%d] out of range[size:%d]! Return empty string\n", state, array_size);
		return "";
	}
	return str[state];
}


/*********************
 *  cms common function *
 *********************/
static int cms_ensure_psy_available(struct cms *chip)
{
	if(!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");

	if(!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("bms");

	if(!chip->usb_psy)
		chip->usb_psy = power_supply_get_by_name("usb");

	if(chip->direct_charger_enable && !chip->mcu_psy)
		chip->mcu_psy = power_supply_get_by_name("mcu");

	if(!chip->fuelsummary_psy)
		chip->fuelsummary_psy = power_supply_get_by_name("fuelsummary");

	if(chip->batt_psy && chip->usb_psy && chip->bms_psy && (!chip->direct_charger_enable || chip->mcu_psy))
		return 1;

	if(!chip->batt_psy)
		cms_print(chip,PR_WARN,"battery psy not found\n");

	if(!chip->bms_psy)
		cms_print(chip,PR_WARN,"bms psy not found\n");

	if(!chip->usb_psy)
		cms_print(chip,PR_WARN,"usb psy not found\n");

	if(chip->direct_charger_enable && !chip->mcu_psy)
		cms_print(chip,PR_WARN,"mcu psy not found\n");

	if(!chip->fuelsummary_psy)
		cms_print(chip,PR_WARN,"fuelsummary_psy not found\n");

	return 0;
}

static int cms_start_bat_id_det(struct cms *chip)
{
	if(chip->battery_rc.state != BAT_ID_IDLE){
		cms_print(chip,PR_DBG,"chip->battery_rc.state:%s\n",
			bat_id_det_state_strings[chip->battery_rc.state]);
		return -EBUSY;
	}
	chip->battery_rc.timestamp = jiffies_to_msecs(jiffies);
	chip->battery_rc.v1 = 0;
	chip->battery_rc.v2 = 0;
	chip->battery_rc.state = BAT_ID_START;
	schedule_delayed_work__on_system_unbound_wq(&chip->bat_id_det_work, 0);
	return 0;
}
/*
*platform adaptive needed
*return physical battery id adc value
*/
static int64_t cms_adc_read(struct cms *chip, int type)
{
	int rc;
	int adc_result = 0;

	if(!chip->iio.cms_battid_chan){
		cms_print(chip, PR_ERROR, "cms vadc_dev is null\n");
		return 0;
	}

	mutex_lock(&chip->adc_lock);
	rc = iio_read_channel_processed(chip->iio.cms_battid_chan, &adc_result);
	if (rc < 0) {
		cms_print(chip, PR_ERROR, "adc reading error battid channel, rc = %d\n", rc);
		mutex_unlock(&chip->adc_lock);
		return rc;
	}
	cms_print(chip, PR_DBG, "%s adc_result = %d\n", cms_adc_type_strings[type], adc_result);
	mutex_unlock(&chip->adc_lock);

	return adc_result;
}

/*
*platform adaptive needed
* @out, 0:input,1:output.
* @up, 0:down,1:up
*/
static void cms_bat_id_gpio_set(struct cms *chip,
	int out, int up)
{
	if(!out){
		if(gpio_direction_input(chip->bat_id_gpio))
			cms_print(chip,PR_ERROR,"failed to set gpio input\n");
	}else{
		if(gpio_direction_output(chip->bat_id_gpio, up))
			cms_print(chip,PR_ERROR,"failed to set gpio %s\n", up ? "up":"down");
	}
}

/*return battery id calculated by rc value*/
static int cms_calculate_rc(struct cms *chip)
{
	int index;
	struct battery_rc *rc = &chip->battery_rc;
	if(chip->bat_det_method == BAT_DET_BY_R){
		if(chip->use_new_rc_param){
			for(index = 0; index < ARRAY_SIZE(new_r_items); index++){
				if(is_between(new_r_items[index].v1min, new_r_items[index].v1max, rc->v1))
					return new_r_items[index].id;
			}
		}else{
			for(index = 0; index < ARRAY_SIZE(r_items); index++){
				if(is_between(r_items[index].v1min, r_items[index].v1max, rc->v1))
					return r_items[index].id;
			}
		}
    } else if (chip->bat_det_method == BAT_DET_BY_RC ){
    	if(chip->use_new_rc_param){
			for(index = 0; index < ARRAY_SIZE(new_rc_items); index++){
				if(is_between(new_rc_items[index].v1min, new_rc_items[index].v1max, rc->v1)
					&& is_between(new_rc_items[index].v2min, new_rc_items[index].v2max, rc->v2))
					return new_rc_items[index].id;
			}
    	}else{
			for(index = 0; index < ARRAY_SIZE(rc_items); index++){
				if(is_between(rc_items[index].v1min, rc_items[index].v1max, rc->v1)
					&& is_between(rc_items[index].v2min, rc_items[index].v2max, rc->v2))
					return rc_items[index].id;
			}
		}
	}
	return 0;
}

static int cms_get_batid_by_ex_fg(struct cms *chip)
{
	union power_supply_propval batt_id = {0,};
	bool power_on_timeout = ((u64)(local_clock()/1000000000)) > 60;// 60s

	if(chip->no_bat_warning) {
		batt_id.intval = 1;
		return batt_id.intval;
	}

	if (!chip->ex_bms_psy) {
		chip->ex_bms_psy = power_supply_get_by_name((char *)chip->ex_bms_psy_name);
		if (!chip->ex_bms_psy) {
			cms_print(chip,PR_ERROR,"ex_bms psy not found!\n");
			//return 0;
			goto OUT;
		}
	}

	if (chip->ex_bms_psy) {
		power_supply_get_property(chip->ex_bms_psy,
			POWER_SUPPLY_PROP_BATTERY_ID, &batt_id);

		if (batt_id.intval > B_MAX)
			batt_id.intval = B_MAX;
	}

	if (chip->battery_rc.bat_id == B_UNKOWN && batt_id.intval != B_UNKOWN) {
		chip->battery_rc.bat_id = batt_id.intval;
	}

OUT:
	if (!power_on_timeout && !batt_id.intval) {
		chip->ex_fg_fake_batt_id_start = true;
		chip->battery_rc.bat_id = B_THE_1_SUPPLIER;	//use the fake batt_id
		batt_id.intval = B_THE_1_SUPPLIER;
	} else if (chip->ex_fg_fake_batt_id_start) {
		chip->ex_fg_fake_batt_id_start = false;
		chip->battery_rc.bat_id = batt_id.intval;
		if (!chip->no_bat_warning)
			cms_notify_changed(chip, HEALTH_STATUS_BAT_INVALID,
				chip->battery_rc.bat_id == B_UNKOWN);
	}

	cms_print(chip,PR_INFO,"batt_id.intval=%d(%d), power_on_timeout=%d, ex_fg_fake_batt_id_start=%d.\n",
		batt_id.intval, chip->battery_rc.bat_id, power_on_timeout, chip->ex_fg_fake_batt_id_start);
	return batt_id.intval;
}

static int cms_get_batid_by_eprom(struct cms *chip)
{
	union power_supply_propval batt_id = {0,};

	if(chip->no_bat_warning) {
		batt_id.intval = 1;
		return batt_id.intval;
	}

	if (!chip->batid_psy) {
		chip->batid_psy = power_supply_get_by_name("batid");
		if (!chip->batid_psy) {
			cms_print(chip,PR_ERROR,"batid psy not found!\n");
			return 0;
		}
	}

	if (chip->batid_psy) {
		power_supply_get_property(chip->batid_psy,
			POWER_SUPPLY_PROP_BATTERY_ID, &batt_id);
	}

	if (chip->battery_rc.bat_id == B_UNKOWN && batt_id.intval != B_UNKOWN) {
		chip->battery_rc.bat_id = batt_id.intval;
	}
	cms_print(chip,PR_DBG,"batt_id.intval=%d\n",batt_id.intval);
	return batt_id.intval;
}

static void cms_notify_changed(struct cms *chip, int bit, bool val)
{
	int changed = 0;

	/* disable notifications when in exhibition mode */
	if (chip->exhibition_mode) {
		cms_print(chip, PR_INFO, "exhibition mode, notify [BIT(%d), %d] skipped\n", bit, val);
		return;
	}

#if 1	//PD1824/PD1916/PD1922 BQ27750 IIC error issue: use for keep charging current and charging UI.
	if (chip->no_bat_id_keep_charging) {
		if(bit == HEALTH_STATUS_BAT_INVALID
			&& chip->bat_det_method == BAT_DET_BY_EX_FG)
		{

			if(power_off_charging_mode)
				val = false;
			else
				bit = HEALTH_STATUS_CHG_ERR;
		}
	}
#endif

	if(!val && test_and_clear_bit(bit, &chip->health_status)){
		changed = 1;
	}else if(val && !test_and_set_bit(bit, &chip->health_status)){
		changed = 1;
	}

	if(changed) {
		cms_print(chip,PR_INFO,"health_status=0x%X\n", chip->health_status);
		power_supply_changed(chip->cms_psy);
	}
}

static void cms_battery_det_done(struct cms *chip)
{
	cms_print(chip,PR_DBG,"battery id=%s\n", battery_id_strings[chip->battery_rc.bat_id]);
	/*schedule another check*/
	if(chip->battery_rc.counter == 1){
		cms_start_bat_id_det(chip);
	}
	/*another battery check is finished*/
	if(chip->battery_rc.counter > 1){
		if (!chip->no_bat_warning)
			cms_notify_changed(chip, HEALTH_STATUS_BAT_INVALID,
				chip->battery_rc.bat_id == B_UNKOWN);
		if (chip->battery_rc.bat_id != B_UNKOWN) {
			chip->pon_batt_pluged = true;
		}
	}
	if(chip->complete)
		complete(chip->complete);
}

extern unsigned int is_atboot;

static void cms_bat_id_det_work(struct work_struct *work)
{
	int32_t cur_msecs = 0;
	int32_t delay_ms = 0;
	int32_t	bid	= B_UNKOWN;

	struct cms *chip = container_of(work,
						 struct cms,
						 bat_id_det_work.work);

    mutex_lock(&chip->battid_lock);
	if (is_atboot) {
		bid = 1;
		cms_print(chip, PR_INFO, "atboot bid = %d \n", bid);
		chip->battery_rc.bat_id = bid;
		chip->battery_rc.state = BAT_ID_IDLE;
		chip->battery_rc.counter = 2;
		cms_battery_det_done(chip);
		mutex_unlock(&chip->battid_lock);
		return ;
	}
	if(chip->bat_det_method == BAT_DET_BY_EX_FG){
		bid = cms_get_batid_by_ex_fg(chip);
		if(chip->no_bat_warning)
			bid = 1;
		cms_print(chip,PR_INFO,"bid = %d \n",bid);
		chip->battery_rc.bat_id = bid;
		chip->battery_rc.state = BAT_ID_IDLE;
		chip->battery_rc.counter = 2;
		cms_battery_det_done(chip);
		mutex_unlock(&chip->battid_lock);
		return ;
	} else if(chip->bat_det_method == BAT_DET_BY_EPROM){
		bid = cms_get_batid_by_eprom(chip);
		if(chip->no_bat_warning)
			bid = 1;
		cms_print(chip,PR_INFO,"bid = %d \n",bid);
		chip->battery_rc.bat_id = bid;
		chip->battery_rc.state = BAT_ID_IDLE;
		chip->battery_rc.counter = 2;
		cms_battery_det_done(chip);
		mutex_unlock(&chip->battid_lock);
		return ;
	} else if (chip->bat_det_method == BAT_DET_BY_R) {
		switch(chip->battery_rc.state){
        case BAT_ID_START:
			if(chip->bat_id_gpio > 0){
	            cms_bat_id_gpio_set(chip,1,1);
				msleep(5);
	            chip->battery_rc.v1 = div_s64(cms_adc_read(chip,CMS_BAT_ID),1000);
	            cms_bat_id_gpio_set(chip,0,0);
			}else{
				chip->battery_rc.v1 = div_s64(cms_adc_read(chip,CMS_BAT_ID),1000);
			}
			bid = cms_calculate_rc(chip);
			if(chip->no_bat_warning)
				bid = 1;
            cms_print(chip,PR_INFO,"battery_rc.counter=%d,battery_rc.v1=%lld,bid=%d\n",chip->battery_rc.counter,chip->battery_rc.v1,bid);
			if(chip->battery_rc.counter == 0){
				chip->battery_rc.bat_id = bid;
			}else if(chip->battery_rc.bat_id == B_UNKOWN){
				chip->battery_rc.bat_id = bid;
			}
			chip->battery_rc.state = BAT_ID_IDLE;
            chip->battery_rc.counter++;
			cms_battery_det_done(chip);
            break;
          default:
		    cms_print(chip,PR_ERROR,"not possible state!\n");
		    break;
        }
   }
   else if (chip->bat_det_method == BAT_DET_BY_RC) {
		switch(chip->battery_rc.state){
		case BAT_ID_START:
			if(chip->bat_id_gpio > 0){
				/*Set gpio output and discharge rc*/
				cms_bat_id_gpio_set(chip,1,0);
				msleep(50);
				/*Start charging rc*/
				cms_bat_id_gpio_set(chip,1,1);
			}

			chip->battery_rc.timestamp = jiffies_to_msecs(jiffies);
			/*Schedule charging rc stat*/
			chip->battery_rc.state = BAT_ID_CHARGING_RC;
			schedule_delayed_work__on_system_unbound_wq(&chip->bat_id_det_work,
				msecs_to_jiffies(BAT_ID_RC_CHARGE_PERIOD_MS-10));
			break;
		case BAT_ID_CHARGING_RC:
			/*Calculate an accurate delay*/
			cur_msecs = jiffies_to_msecs(jiffies);
			delay_ms = BAT_ID_RC_CHARGE_PERIOD_MS -
				(cur_msecs - chip->battery_rc.timestamp);
			/*Schedule may be delay*/
			if(delay_ms	> 0)
				mdelay(delay_ms);
			chip->battery_rc.v1 = div_s64(cms_adc_read(chip,CMS_BAT_ID),1000);
			/*Set gpio input*/

			if(chip->bat_id_gpio > 0){
				cms_bat_id_gpio_set(chip,0,0);
			}

			chip->battery_rc.timestamp = jiffies_to_msecs(jiffies);
			chip->battery_rc.state = BAT_ID_READING_RC;
			/*Schedule reading rc*/
			schedule_delayed_work__on_system_unbound_wq(&chip->bat_id_det_work,
				msecs_to_jiffies(BAT_ID_READING_RC_PERIOD_MS-10));
			break;
		case BAT_ID_READING_RC:
			cur_msecs = jiffies_to_msecs(jiffies);
			delay_ms = BAT_ID_READING_RC_PERIOD_MS -
				(cur_msecs - chip->battery_rc.timestamp);
			if(delay_ms > 0)
				mdelay(delay_ms);
			msleep(5);
			chip->battery_rc.v2 = div_s64(cms_adc_read(chip,CMS_BAT_ID),1000);
			chip->battery_rc.timestamp = jiffies_to_msecs(jiffies);
			bid = cms_calculate_rc(chip);
			if(chip->battery_rc.counter == 0){
				chip->battery_rc.bat_id = bid;
			}else if(chip->battery_rc.bat_id == B_UNKOWN){
				chip->battery_rc.bat_id = bid;
			}
			if(chip->no_bat_warning)
				chip->battery_rc.bat_id = 1;

			chip->battery_rc.state = BAT_ID_DONE;
			schedule_delayed_work__on_system_unbound_wq(&chip->bat_id_det_work,0);
			break;
		case BAT_ID_DONE:
			chip->battery_rc.state = BAT_ID_IDLE;
			chip->battery_rc.counter++;
			cms_battery_det_done(chip);
			break;
		default:
			cms_print(chip,PR_ERROR,"not possible state!\n");
			break;
		}
	}
    mutex_unlock(&chip->battid_lock);

	cms_print(chip,PR_INFO,"state:%s, timestamp:%llu, v1:%lld, v2:%lld, battery_id:%s, delay_ms:%u\n",
		bat_id_det_state_strings[chip->battery_rc.state],
		chip->battery_rc.timestamp, chip->battery_rc.v1,
		chip->battery_rc.v2,battery_id_strings[chip->battery_rc.bat_id],delay_ms);
}

/*call it in the lock*/
static int cms_is_battery_present(struct cms *chip)
{
	int64_t bat_id_voltage = 0;

    if (chip->bat_det_method == BAT_DET_BY_EX_FG) {
		return 1;
	}
    if (chip->bat_det_method == BAT_DET_BY_EPROM) {
		return cms_get_batid_by_eprom(chip);
	}

	if (atomic_read(&chip->in_batid)) {
		cms_print(chip,PR_DBG,"in_batid detecting\n");
		return 1;
	}
	atomic_set(&chip->in_batid, 1);
	if(chip->bat_id_gpio > 0){
		cms_bat_id_gpio_set(chip,1,1);
		bat_id_voltage = div_s64(cms_adc_read(chip,CMS_BAT_ID),1000);
		cms_bat_id_gpio_set(chip,0,0);
	}else{
		bat_id_voltage = div_s64(cms_adc_read(chip,CMS_BAT_ID),1000);
	}
	atomic_set(&chip->in_batid, 0);
	cms_print(chip,PR_DBG,"bat_id_voltage:%lld mV\n",bat_id_voltage);
	if(chip->no_bat_warning)
		return 1;

	if((BAT_ID_REF_VOLTAGE * 90) / 100 <= bat_id_voltage)
		return 0;
	return 1;
}

static int cms_is_battery_pluged(struct cms *chip)
{
	int batid_done,temp;
	int rc = 1;
	static int count = 0;
	mutex_lock(&chip->status_lock);
	if(!chip->enabled || !chip->online){
		cms_print(chip,PR_DBG,"usb unpluged\n");
		mutex_unlock(&chip->status_lock);
		return rc;
	}
	mutex_unlock(&chip->status_lock);

	if (atomic_read(&chip->in_batid)) {
		cms_print(chip,PR_DBG,"in_batid=1\n");
		return rc;
	}
	if (count++ % BATTERY_PLUGED_DETECT_STEP) {
		if (count > 1000) count = 0;
		cms_print(chip,PR_DBG,"count=%d\n",count);
		return rc;
	}

	batid_done = chip->battery_rc.state == BAT_ID_IDLE && chip->battery_rc.counter > 1;
    cms_print(chip,PR_DBG,"batid_done=%d,bat_id=%d\n",batid_done,chip->battery_rc.bat_id);

	if (chip->batt_plugable == 0) {
		cms_print(chip,PR_DBG,"battery is not plugable,using defaut value\n");
		return rc;
	}
	if(batid_done && chip->pon_batt_pluged){
		temp = cms_is_battery_present(chip);
		/*battery status change from pluged to missing,unpluged by user*/
		if (!temp) {
			count = 0;
			cms_notify_changed(chip, HEALTH_STATUS_BAT_UNPLUGED, !temp);
			cms_print(chip,PR_INFO,"battery missing and plugable,shut down device\n");
			rc = 0;
			return rc;
		}
	}
	return rc;

}

static void cms_scale_input_current(struct cms *chip, int pc){

	union power_supply_propval curr_ma = {0,};
	union power_supply_propval usb_type = {0,};

	cms_ensure_psy_available(chip);

	if(!chip->batt_psy) {
		cms_print(chip,PR_ERROR,"bat_psy not registed\n");
		return;
	}
	power_supply_get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_REAL_TYPE, &usb_type);
	if(usb_type.intval == POWER_SUPPLY_TYPE_USB) {
		return;
	}
	power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_LIMIT_INPUT_CURRENT_MAX, &curr_ma);
#if 0
	if (curr_ma.intval < SCALE_INPUT_CURRENT_THREHOLD_MA) {
		pr_warn("skip scale input %d < %d\n",curr_ma.intval,SCALE_INPUT_CURRENT_THREHOLD_MA);
		return;
	}
#endif
	curr_ma.intval = curr_ma.intval * pc / 100;
	if (get_fuelsummary_user_custom(chip)) {
		cms_print(chip, PR_INFO, "input: origin=%d, custom=%d\n",
				curr_ma.intval, chip->custom_input);
		curr_ma.intval = chip->custom_input;
	}
	cms_print(chip,PR_INFO,"scale input current to %d mA\n", curr_ma.intval);
	power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_LIMIT_INPUT_CURRENT_USER, &curr_ma);
}

static void cms_scale_ibat_current(struct cms *chip, int pc)
{
	union power_supply_propval curr_ma = {0,};
	union power_supply_propval full_mah = {0,};

	cms_ensure_psy_available(chip);

	if(!chip->batt_psy) {
		cms_print(chip,PR_ERROR,"bat_psy not registed\n");
		return;
	}
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, &full_mah);
	curr_ma.intval = full_mah.intval * pc / 100;
	if (get_fuelsummary_user_custom(chip)) {
		cms_print(chip, PR_INFO, "ibat: origin=%d, custom=%d\n",
				curr_ma.intval, chip->custom_current);
		curr_ma.intval = chip->custom_current;
	}
	cms_print(chip,PR_INFO,"full design=%d mah,scale chg current to %d mA\n",full_mah.intval,curr_ma.intval);
	power_supply_set_property(chip->batt_psy,
		POWER_SUPPLY_PROP_LIMIT_BATT_CURRENT_USER, &curr_ma);
}

static void cms_scale_dchg_current(struct cms *chip, int pc)
{
	union power_supply_propval curr_ma = {0,};
	union power_supply_propval full_mah = {0,};
	union power_supply_propval dchg_enable = {0,};

	cms_ensure_psy_available(chip);

	if(!chip->mcu_psy) {
		cms_print(chip,PR_ERROR,"mcu_psy not registed\n");
		return;
	}
	power_supply_get_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_DCHG_ENABLED, &dchg_enable);
	/*
	if (dchg_enable.intval == 0) {
		cms_print(chip,PR_WARN,"Direct charging no enable, skip ...\n");
		return;
	}*/

	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, &full_mah);
	curr_ma.intval = full_mah.intval * pc / 100;
	if (get_fuelsummary_user_custom(chip)) {
		cms_print(chip, PR_INFO, "dchg: origin=%d, custom=%d\n",
				curr_ma.intval, chip->custom_current);
		curr_ma.intval = chip->custom_current;
	}
	cms_print(chip,PR_INFO,"full design=%d mah,scale Dchg current to %d mA. (dchg_enable %d)\n",full_mah.intval,curr_ma.intval,dchg_enable.intval);
	power_supply_set_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_DCHG_CURRENT, &curr_ma);
}

static void cms_update_fastchg_alloc(struct cms *chip,int primary_fastchg_ma,int parallel_fastchg_ma)
{
	union power_supply_propval primary_fastchg_now_ma = {0,};
	union power_supply_propval parallel_fastchg_now_ma = {0,};

	cms_ensure_psy_available(chip);

	if(!chip->batt_psy) {
		cms_print(chip,PR_ERROR,"batt_psy not registed\n");
		return;
	}

	cms_print(chip,PR_INFO,"primary_fastchg_ma=%d parallel_fastchg_ma=%d\n",primary_fastchg_ma,parallel_fastchg_ma);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PRIMARY_FASTCHG_ALLOC, &primary_fastchg_now_ma);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PARALLEL_FASTCHG_ALLOC, &parallel_fastchg_now_ma);
	if (primary_fastchg_now_ma.intval == primary_fastchg_ma &&
		parallel_fastchg_now_ma.intval == parallel_fastchg_ma) {
		return;
	}

	primary_fastchg_now_ma.intval = primary_fastchg_ma;
	power_supply_set_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PRIMARY_FASTCHG_ALLOC, &primary_fastchg_now_ma);
	parallel_fastchg_now_ma.intval = parallel_fastchg_ma;
	power_supply_set_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PARALLEL_FASTCHG_ALLOC, &parallel_fastchg_now_ma);

}

static int cms_get_fastchg_current(struct cms *chip)
{
	if (!chip->ac_avg_current) {
		cms_print(chip,PR_DBG,"update fastchg current from userspace\n");
		cms_check_fastchg_current(chip);
	}
	return chip->ac_avg_current;
}

static void cms_enable(struct cms *chip, int enable)
{
	mutex_lock(&chip->status_lock);
	if(chip->enabled ^ enable){
        chip->enabled = enable;
		cms_is_enable = chip->enabled;
        if(enable && chip->online && !chip->cms_work_started){
			cms_print(chip,PR_INFO,"start cms work\n");
			chip->cms_work_started = true;
            schedule_delayed_work__on_system_unbound_wq(&chip->cms_work,msecs_to_jiffies(1000));
        }else{
			cms_print(chip,PR_INFO,"disable cms work\n");
			cancel_delayed_work(&chip->cms_work);
			chip->health_status = 0;
			chip->cms_work_started = false;
			chip->factory_mode_state = 0;

			/*restore chg current scale*/
			cms_scale_ibat_current(chip, 100);
			power_supply_changed(chip->cms_psy);
		}
	}
	mutex_unlock(&chip->status_lock);
}

static int cms_map_health_status(struct cms *chip)
{
	int index = 0;
	int value = 0;

	for(index = 0; index < ARRAY_SIZE(health_status_map); index++){
		if(test_bit(health_status_map[index][0], &chip->health_status)){
			value = health_status_map[index][1];
			break;
		}
	}

	return value;
}

static int cms_get_prop_batt_id(struct cms *chip)
{
	DECLARE_COMPLETION_ONSTACK(complete);
	int bat_id = B_UNKOWN;
	int timeout;

	mutex_lock(&chip->battid_lock);
	if(chip->battery_rc.bat_id == B_UNKOWN &&
		chip->battery_rc.counter <= 1){
		chip->complete = &complete;
		mutex_unlock(&chip->battid_lock);
		timeout = wait_for_completion_timeout(&complete,
						msecs_to_jiffies(3500));
		cms_print(chip,PR_INFO,"complete, timeout:%d", timeout);
	}else{
		mutex_unlock(&chip->battid_lock);
	}

	mutex_lock(&chip->battid_lock);
	bat_id = chip->battery_rc.bat_id;
	chip->complete = 0;
	mutex_unlock(&chip->battid_lock);
	return bat_id;
}

/******************
 *   cms pys function   *
 ******************/
static int cms_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	int ret = 0;
	struct cms *chip = power_supply_get_drvdata(psy);

	switch(psp){
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = chip->enabled;
			break;
		case POWER_SUPPLY_PROP_HEALTH_STATUS:
		case POWER_SUPPLY_PROP_POWER_NOW:
			/* disable notifications when in exhibition mode */
			if (chip->exhibition_mode)
				val->intval = 0;
			else
				val->intval = chip->health_status;
			break;
		case POWER_SUPPLY_PROP_POWER_AVG:
			/* disable notifications when in exhibition mode */
			if (chip->exhibition_mode)
				val->intval = 0;
			else
				val->intval = cms_map_health_status(chip);
			break;
		case POWER_SUPPLY_PROP_MODEL_NAME:
			val->strval = battery_id_strings[chip->battery_rc.bat_id];
			break;
		case POWER_SUPPLY_PROP_BATTERY_ID:
			val->intval = cms_get_prop_batt_id(chip);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = cms_is_battery_pluged(chip);
			break;
		case POWER_SUPPLY_PROP_CURRENT_AC_AVG_NOW:
			val->intval = cms_get_fastchg_current(chip);
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = chip->batt_plugable;
			break;
		case POWER_SUPPLY_PROP_HI_POWER:
			val->intval = cms_check_fastchg_current(chip);
			break;
		case POWER_SUPPLY_PROP_SWITCH_STATE:
			val->intval = chip->switch_state;
			break;
		case POWER_SUPPLY_PROP_DISABLE_CHARGE:
			val->intval = chip->ai_disable_charge;
			pr_err("system ui get charge status value=%d\n", chip->ai_disable_charge);
			break;
		case POWER_SUPPLY_PROP_CALLING_STATE:
			val->intval = chip->calling_state;
			break;
		case POWER_SUPPLY_PROP_WEIXIN_CALLING_STATE:
			val->intval = chip->weixin_calling_state;
			break;
		case POWER_SUPPLY_PROP_FACTORY_MODE_STATE:
			val->intval = chip->factory_mode_state;
			break;
		case POWER_SUPPLY_PROP_HTCCC_ENABLE:
			val->intval = chip->htccc_enable;
			break;
		/*
		case POWER_SUPPLY_PROP_BATT_ID_DETECT_DONE:
			val->intval = (chip->battery_rc.state == BAT_ID_IDLE && chip->battery_rc.counter > 1) ? true : false;
			break;
		*/
		case POWER_SUPPLY_PROP_BSPTEST_START_CHG_SOC:
			val->intval = chip->bsptest_soc_range[0];
			break;
		case POWER_SUPPLY_PROP_BSPTEST_STOP_CHG_SOC:
			val->intval = chip->bsptest_soc_range[1];
			break;
		case POWER_SUPPLY_PROP_CHARGING_TECHNOLOGY:
			val->intval = chip->charging_technology;
			break;
		case POWER_SUPPLY_PROP_EXHIBITION_MODE:
			val->intval = chip->exhibition_mode;
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int cms_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	int ret = 0;
	struct cms *chip = power_supply_get_drvdata(psy);

	switch(psp){
		case POWER_SUPPLY_PROP_PRESENT:
			cms_enable(chip,val->intval);
			break;
		case POWER_SUPPLY_PROP_SWITCH_STATE:
			chip->switch_state = val->intval;
			cms_print(chip, PR_INFO, "system ui set chip->switch_state=%d\n", chip->switch_state);
			break;
		case POWER_SUPPLY_PROP_DISABLE_CHARGE:
			chip->ai_disable_charge = val->intval;
			pr_err("system ui set charge switch=%d\n", chip->ai_disable_charge);
			break;
		case POWER_SUPPLY_PROP_CALLING_STATE:
			chip->calling_state = val->intval;
			cms_print(chip,PR_INFO,"chip->calling_state = %d\n",chip->calling_state);
			break;
		case POWER_SUPPLY_PROP_WEIXIN_CALLING_STATE:
			chip->weixin_calling_state = (val->intval > 0) ? true : false;
			cms_print(chip,PR_INFO,"chip->weixin_calling_state = %d, val->intval=%d\n",chip->weixin_calling_state, val->intval);
			break;
		case POWER_SUPPLY_PROP_FACTORY_MODE_STATE:
			chip->factory_mode_state = val->intval;
			if(chip->mcu_psy) 
				power_supply_set_property(chip->mcu_psy,
					POWER_SUPPLY_PROP_FACTORY_MODE_STATE, val);
			if(chip->batt_psy) 
				power_supply_set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_FACTORY_MODE_STATE, val);
			cancel_delayed_work(&chip->cms_work);
			schedule_delayed_work__on_system_unbound_wq(&chip->cms_work, msecs_to_jiffies(500));
			cms_print(chip,PR_INFO,"### vivo factory_mode_state=%d ###\n",chip->factory_mode_state);
			break;
		case POWER_SUPPLY_PROP_HTCCC_ENABLE:
			chip->htccc_enable = val->intval;
			cms_print(chip,PR_INFO,"### vivo htccc_enable=%d ###\n",chip->htccc_enable);
			break;
		case POWER_SUPPLY_PROP_HEALTH_STATUS:
			if (val->intval)
				chip->health_status |= val->intval;
			else
				chip->health_status = 0;
			cms_print(chip, PR_INFO, "### vivo health_status=0x%X ###\n", (int)chip->health_status);
			break;
		case POWER_SUPPLY_PROP_BSPTEST_START_CHG_SOC:
			if (val->intval > 0 && val->intval < 100) {
				chip->bsptest_soc_range[0] = val->intval;
				cms_print(chip, PR_INFO, "### bsptest_soc_range start_chg=%d ###\n", chip->bsptest_soc_range[0]);
			}
			break;
		case POWER_SUPPLY_PROP_BSPTEST_STOP_CHG_SOC:
			if (val->intval > 0 && val->intval < 100) {
				chip->bsptest_soc_range[1] = val->intval;
				cms_print(chip, PR_INFO, "### bsptest_soc_range stop_chg=%d ###\n", chip->bsptest_soc_range[1]);
			}
			break;
		case POWER_SUPPLY_PROP_EXHIBITION_MODE:
			chip->exhibition_mode = val->intval;
			cms_print(chip, PR_INFO, "### vivo exhibition mode = %d ###\n", chip->exhibition_mode);
			break;
		default:
			ret = -EINVAL;
	}
	return ret;
}

static enum power_supply_property cms_power_props[] = {
	/* real time */
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CURRENT_AC_AVG_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_HI_POWER,
	POWER_SUPPLY_PROP_SWITCH_STATE,
	POWER_SUPPLY_PROP_DISABLE_CHARGE,
	POWER_SUPPLY_PROP_CALLING_STATE,
	POWER_SUPPLY_PROP_WEIXIN_CALLING_STATE,
	POWER_SUPPLY_PROP_FACTORY_MODE_STATE,
	POWER_SUPPLY_PROP_BATTERY_ID,
	POWER_SUPPLY_PROP_HTCCC_ENABLE,
	POWER_SUPPLY_PROP_BSPTEST_START_CHG_SOC,
	POWER_SUPPLY_PROP_BSPTEST_STOP_CHG_SOC,
	POWER_SUPPLY_PROP_CHARGING_TECHNOLOGY,
	POWER_SUPPLY_PROP_EXHIBITION_MODE,
};

static int cms_property_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	int ret = 0;
	switch(psp){
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_SWITCH_STATE:
		case POWER_SUPPLY_PROP_DISABLE_CHARGE:
		case POWER_SUPPLY_PROP_CALLING_STATE:
		case POWER_SUPPLY_PROP_WEIXIN_CALLING_STATE:
		case POWER_SUPPLY_PROP_FACTORY_MODE_STATE:
		case POWER_SUPPLY_PROP_HTCCC_ENABLE:
		case POWER_SUPPLY_PROP_BSPTEST_START_CHG_SOC:
		case POWER_SUPPLY_PROP_BSPTEST_STOP_CHG_SOC:
		case POWER_SUPPLY_PROP_EXHIBITION_MODE:
			ret = 1;
			break;
		default:
			ret = 0;
	}
	return ret;
}

static void cms_external_power_changed(struct power_supply *psy)
{
    struct cms *chip = power_supply_get_drvdata(psy);
	union power_supply_propval usb_in = {0,};
	union power_supply_propval dc_in = {0,};
	union power_supply_propval value = {0,};
	union power_supply_propval usb_conn_flag = {0,};

	if(!chip->usb_psy){
		chip->usb_psy = power_supply_get_by_name("usb");
		if(!chip->usb_psy){
			cms_print(chip,PR_ERROR,"usb psy not found!\n");
			return;
		}
	}

    power_supply_get_property(chip->usb_psy,
		POWER_SUPPLY_PROP_PRESENT, &usb_in);

    power_supply_get_property(chip->usb_psy,
		POWER_SUPPLY_PROP_DC_PRESENT, &dc_in);

	power_supply_get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_USB_CONN_FLAG, &usb_conn_flag);

	if (chip->ex_fg_fake_batt_id_start) {
		cms_get_batid_by_ex_fg(chip);	//update ex-fg batt_id
	}

    mutex_lock(&chip->status_lock);
    chip->online = usb_in.intval || dc_in.intval;

	if(!chip->online){
		chip->health_status = usb_conn_flag.intval ? 0x4000 : 0;
		chip->nr_work = 0;
		chip->continue_count = 0;
		if (chip->cms_work_started)
			chip->factory_mode_state = 0;//only reset when cable disconnect.
		chip->cms_work_started = false;
		chip->chg_begin = 0;
		chip->chg_status = POWER_SUPPLY_STATUS_UNKNOWN;
		chip->direct_charger_status = DIRECT_CHARGER_UNKNOW;
		chip->usb_conn_temp_high_count = 0;
		chip->bat_board_temp_high_count = 0;
		chip->bat_conn_temp_high_count = 0;
		chip->master_bat_conn_temp_high_count = 0;
		chip->usb_conn_heat_protect_count = 0;
		chip->adapter_power_derate_param_ready = 0;
		chip->last_step = 0;
		chip->usbid_mv = 1800;
		chip->ai_disable_charge = 0;
		chip->ai_disable_charge_state = false;
		cms_reset_voters(chip);
		cancel_delayed_work(&chip->cms_work);
		cancel_delayed_work(&chip->bsp_charging_work);
		if (chip->cms_psy)
			power_supply_changed(chip->cms_psy);
		if (bsp_test_mode) {
			if (chip->usbin_cutoff_status) {
				pr_warn("usb unplug, restore chg\n");
				value.intval = 1;
				power_supply_set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &value);
				chip->usbin_cutoff_status = false;
			}
		}
	}

    if(chip->enabled && chip->online && !chip->cms_work_started) {
		cms_print(chip,PR_INFO,"start cms work\n");
		chip->continue_count = 0;
		chip->chg_status = POWER_SUPPLY_STATUS_UNKNOWN;
		chip->chg_begin = jiffies;
		chip->cms_work_started = true;
		chip->adapter_power_derate_param_ready = 0;
		cancel_delayed_work(&chip->cms_work);
		cancel_delayed_work(&chip->bsp_charging_work);
		if (bsp_test_mode) {
			cms_print(chip,PR_INFO,"bsp_test_mode, start cms_work quickly!!!\n");
			schedule_delayed_work__on_system_unbound_wq(&chip->cms_work, msecs_to_jiffies(1000));
			schedule_delayed_work__on_system_unbound_wq(&chip->bsp_charging_work, msecs_to_jiffies(500));
		} else
        	schedule_delayed_work__on_system_unbound_wq(&chip->cms_work, msecs_to_jiffies(5000));
		/*vote for dchg disable to default value,usb plugged*/
		if (chip->direct_charger_enable) {
			if (!chip->dchg_disable_votable)
				chip->dchg_disable_votable = find_votable("DCHG_DISABLE");

			if (chip->dchg_disable_votable) {
				if (chip->calling_dchg_disable)
					vote(chip->dchg_disable_votable, CALLING_EVENT, false, 0);
				vote(chip->dchg_disable_votable, DCHG_HEAVY_LOAD_EVENT, false, 0);
			}
		}
    }
    mutex_unlock(&chip->status_lock);
	cms_print(chip, PR_INFO, "usb_in=%d,dc_in=%d,usb_conn_flag=%d, cms_work_started=%d, factory_mode_state=%d\n", usb_in.intval, dc_in.intval, usb_conn_flag.intval, chip->cms_work_started, chip->factory_mode_state);
}

static const struct power_supply_desc cms_psy_desc = {
	.name = "cms",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = cms_power_props,
	.num_properties = ARRAY_SIZE(cms_power_props),
	.get_property = cms_get_property,
	.set_property = cms_set_property,
	.property_is_writeable = cms_property_is_writeable,
	.external_power_changed = cms_external_power_changed,
};

static int cms_register_psy(struct cms *chip)
{
	struct power_supply_config cms_cfg = {};

	cms_cfg.drv_data = chip;
	cms_cfg.of_node = chip->dev->of_node;
	chip->cms_psy = devm_power_supply_register(chip->dev,
						   &cms_psy_desc,
						   &cms_cfg);
	if (IS_ERR(chip->cms_psy)) {
		cms_print(chip,PR_ERROR,"Couldn't register cms power supply\n");
		return PTR_ERR(chip->cms_psy);
	}
	return 0;
}

#if 0
/**********************
	cms fb on callback
***********************/
static int cms_fb_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	int *pdata;
	struct cms *chip =
			container_of(self, struct cms, fb_nb);
	struct fb_event *fbe = data;
	cms_print(chip,PR_INFO,"enter event=%lu\n",event);
	if(!fbe)
		goto out;
	if(!fbe->data)
		goto out;
	if(event != FB_EVENT_BLANK)
		goto out;

	pdata = (int*)fbe->data;
	if(*pdata == FB_BLANK_UNBLANK){
		chip->fb_on = true;
		chip->nr_work = 0;
	}else if(*pdata == FB_BLANK_POWERDOWN)
		chip->fb_on = false;
	cms_print(chip,PR_INFO,"leave fb_on=%d\n",chip->fb_on);
out:
	return 0;
}
#endif

/**************************
	cms check charge current
***************************/
static bool cms_is_voter_priority(struct cms *chip)
{
	if(chip->factory_mode_state)
		return true;

	return false;
}

static void cms_check_heavy_load_dchg(struct cms *chip, int dchg_status)
{
	union power_supply_propval ibus_ma = {0,};
	union power_supply_propval ibat = {0,};
	union power_supply_propval primary_board_temp = {0,};
	union power_supply_propval pval = {0,};
	int ibat_ma;
	bool is_heavy_load = false;

	if (!chip->online || !chip->direct_charger_enable ||
		power_off_charging_mode || cms_is_voter_priority(chip)) {
		cms_print(chip, PR_INFO, "skip checking heavy load, online=%d, dchg_enable=%d, pwroffchg=%d, voter_priority=%d\n",
				chip->online, chip->direct_charger_enable, power_off_charging_mode, cms_is_voter_priority(chip));
		return;
	}

	if (!chip->dchg_disable_votable) {
		chip->dchg_disable_votable = find_votable("DCHG_DISABLE");
		if (!chip->dchg_disable_votable) {
			cms_print(chip, PR_ERROR, "Couldn't find DCHG_DISABLE votable\n");
			return;
		}
	}

	power_supply_get_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_HALF_CHG_MASTER_IBUS, &ibus_ma);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &ibat);
	ibat_ma = ibat.intval / 1000;
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PRIMARY_BOARD_TEMP, &primary_board_temp);

	if (dchg_status == DIRECT_CHARGER_IS_CHARGERING && (ibus_ma.intval * 2 + ibat_ma) > 800 && primary_board_temp.intval > 450) {
		is_heavy_load = true;
		pval.intval = 1;
		power_supply_set_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_DCHG_HEAVY_LOAD, &pval);
		vote(chip->dchg_disable_votable, DCHG_HEAVY_LOAD_EVENT, true, 0);
	} else if (primary_board_temp.intval < 420 && 
					get_effective_result(chip->dchg_disable_votable) == 1 &&
					!strcmp(get_effective_client(chip->dchg_disable_votable), DCHG_HEAVY_LOAD_EVENT)) {
		pval.intval = 0;
		power_supply_set_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_DCHG_HEAVY_LOAD, &pval);
		vote(chip->dchg_disable_votable, DCHG_HEAVY_LOAD_EVENT, false, 0);
	}

	if (is_heavy_load || !strcmp(get_effective_client(chip->dchg_disable_votable), DCHG_HEAVY_LOAD_EVENT))
		cms_print(chip, PR_INFO, "ibus = %d, ibat = %d, pri_board_temp = %d, heavy_load = %d, dchg_status = %s\n",
				ibus_ma.intval, ibat_ma, primary_board_temp.intval, is_heavy_load,
				map_state_string(dchg_status, dchg_status_strings, ARRAY_SIZE(dchg_status_strings)));

}

static bool cms_check_fastchg_current(struct cms *chip)
{
	union power_supply_propval vbat_uv = {0,};
	union power_supply_propval usb_type = {0,};
	union power_supply_propval update = {0, };
	union power_supply_propval fastchg_current_ua = {0,};
	int fastchg_current_ma,vbat_mv;
	int rc;

	if (!chip->online) {
		cms_print(chip,PR_ERROR,"usb not present\n");
		return false;
	}

	if (chip->health_status) {
		cms_print(chip,PR_ERROR,"health = 0x%X,charging exception\n",(int)chip->health_status);
		return false;
	}

	rc = cms_ensure_psy_available(chip);
	if(!chip->batt_psy || !chip->usb_psy || !chip->bms_psy) {
		cms_print(chip,PR_ERROR,"!bat_psy || !usb_psy || !bms\n");
		return false;
	}
	power_supply_get_property(chip->usb_psy,
		POWER_SUPPLY_PROP_REAL_TYPE, &usb_type);

	if (!(usb_type.intval == POWER_SUPPLY_TYPE_USB_DCP ||
		usb_type.intval == POWER_SUPPLY_TYPE_USB_HVDCP)) {
		cms_print(chip,PR_ERROR,"usb_type=%d,!DCP || !HVDCP\n",usb_type.intval);
		return false;
	}

	/*update bms data*/
	update.intval = 1;
	power_supply_set_property(chip->bms_psy,
		POWER_SUPPLY_PROP_UPDATE_NOW, &update);

	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);

	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &fastchg_current_ua);

	if (fastchg_current_ua.intval < 0)
		vbat_mv = (vbat_uv.intval + fastchg_current_ua.intval * chip->vbat_r_comp / 1000) / 1000;
	else
		vbat_mv = vbat_uv.intval / 1000;

	fastchg_current_ma = fastchg_current_ua.intval / 1000;

	if (vbat_mv > CHECK_FASTCH_CURRENT_VBAT_THRESHOLD_MV && abs(fastchg_current_ma) < CHECK_FASTCH_CURRENT_IBAT_THRESHOLD_MA) {
		cms_print(chip, PR_ERROR, "vbat_mv = %d > %d,and fastchg_current=%dmA < %dmA, no in CC.\n",
			vbat_mv, CHECK_FASTCH_CURRENT_VBAT_THRESHOLD_MV, fastchg_current_ma, CHECK_FASTCH_CURRENT_IBAT_THRESHOLD_MA);
		return false;
	}

	mutex_lock(&chip->status_lock);
	if (chip->ac_avg_current != fastchg_current_ma) {
		chip->ac_avg_current = fastchg_current_ma;
	}
	mutex_unlock(&chip->status_lock);
	cms_print(chip,PR_INFO,"ac_avg_current=%dma,fastchg_current_ma=%d,factory_mode_state=%d\n",
			chip->ac_avg_current, fastchg_current_ma, chip->factory_mode_state);
	if (fastchg_current_ma > 0) {
		cms_print(chip,PR_DBG,"not in charging state || power consumed too large,fastchg_current_ma=%d > 0ma\n",fastchg_current_ma);
		return false;
	}
	if (fastchg_current_ma * (-1) < CHECK_FASTCH_CURRENT_THRESHOLD_MA) {
		cms_print(chip,PR_DBG,"fastchg_current_ma=%d<%dma\n",fastchg_current_ma,CHECK_FASTCH_CURRENT_THRESHOLD_MA);
		return false;
	}
	/*the large current test in factory mode successful,when reach here */
	return true;
}

static long cms_handle_health_status(struct cms *chip)
{
	long health = chip->health_status;

	/*clear no need abnormal status*/
	test_and_clear_bit(HEALTH_STATUS_USBID_WATER, &health);

	return health;
}

static int cms_check_charge_timeout(struct cms *chip, int chg_status_now, int dchg_status_now)
{
	union power_supply_propval hw_timeout_enable = {0,};
	long health = cms_handle_health_status(chip);
	unsigned long over_time = 0;
	bool timeout = false;
	bool count_flag = true;

	/*init charge  begin time*/
	if (chip->continue_count == 0 && chip->chg_status == POWER_SUPPLY_STATUS_UNKNOWN) {
		cms_print(chip, PR_INFO, "Charging start timing\n");
		chip->chg_begin = jiffies;
	}

	/*process count action flag*/
	if (chg_status_now == POWER_SUPPLY_STATUS_DISCHARGING ||
		chg_status_now == POWER_SUPPLY_STATUS_FULL ||
		health) {
		cms_print(chip, PR_INFO, "Charging stop timing\n");
		chip->chg_begin = jiffies;
		count_flag = false;
	}

	chip->chg_status = chg_status_now;
	chip->direct_charger_status = dchg_status_now;

	/*process timing*/
	if (count_flag) {
		over_time = chip->chg_begin + chip->chg_timeout_mins * 60 * HZ;
		timeout = time_after(jiffies, over_time);

		if (timeout) {
			if (chip->no_chg_timeout) {
				cms_print(chip, PR_INFO, "ignore charging timeout for cmcc\n");
				hw_timeout_enable.intval = false;
				power_supply_set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_HW_CHG_TIMEOUT, &hw_timeout_enable);
			} else {
				cms_print(chip, PR_INFO, "charging timeout\n");
				cms_notify_changed(chip, HEALTH_STATUS_CHG_TIMEOUT, true);
			}
		}

		cms_print(chip, PR_INFO, "charging runs %d seconds, timeout_status=%d, chg_status=%d, dchg_status=%d\n",
				jiffies_to_msecs(jiffies - chip->chg_begin) / 1000, timeout, chip->chg_status, chip->direct_charger_status);
	}

	return timeout;
}


//htccc : high temperature charger current control
static int cms_check_high_temp_chg_cur_control(struct cms *chip,int batt_temp)
{
	union power_supply_propval volt_now = {0,};
	int vbat_uv,i;
	int vbat_uv_sum = 0;
	int temp;

	cms_ensure_psy_available(chip);

	if (!chip->batt_psy) {
		cms_print(chip,PR_ERROR,"batt_psy is NULL\n");
		return 0;
	}

	if(!chip->htccc_enable){
		cms_print(chip,PR_ERROR,"htccc isn't enabled\n");
		return 0;
	}

	for(i=0; i<3; i++){
		power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &volt_now);
		vbat_uv_sum += volt_now.intval;
	}

	vbat_uv = vbat_uv_sum/3;
	temp = batt_temp/10;

	cms_print(chip,PR_INFO,"### htccc volt_now = %d,temp_now = %d ###\n",vbat_uv,temp);

	if(vbat_uv > chip->htccc_data[HTCCC_VBAT] && (temp >= chip->htccc_data[HTCCC_TEMP_L] && temp <= chip->htccc_data[HTCCC_TEMP_H])){
		cms_notify_changed(chip,HEALTH_STATUS_HTCCC,1);
	}else if(vbat_uv <= chip->htccc_data[HTCCC_VBAT] && (temp >= chip->htccc_data[HTCCC_TEMP_L] && temp <= chip->htccc_data[HTCCC_TEMP_H])){
		if(vbat_uv < (chip->htccc_data[HTCCC_VBAT] - chip->htccc_data[HTCCC_DELTA])){
			cms_notify_changed(chip,HEALTH_STATUS_HTCCC,0);
		}
	}else if(vbat_uv > chip->htccc_data[HTCCC_VBAT] && (temp < chip->htccc_data[HTCCC_TEMP_L] || temp > chip->htccc_data[HTCCC_TEMP_H])){
		if(temp <= (chip->htccc_data[HTCCC_TEMP_L]-2) || temp >= (chip->htccc_data[HTCCC_TEMP_H]+2)){
			cms_notify_changed(chip,HEALTH_STATUS_HTCCC,0);
		}
	}else{
		if((vbat_uv >= (chip->htccc_data[HTCCC_VBAT] - chip->htccc_data[HTCCC_DELTA])) &&
				(temp > (chip->htccc_data[HTCCC_TEMP_L]-2) && temp < (chip->htccc_data[HTCCC_TEMP_H]+2))){
			cms_print(chip,PR_INFO,"intermediate range,nothing to do\n");
		}else{
			cms_notify_changed(chip,HEALTH_STATUS_HTCCC,0);
		}
	}

	return 1;
}

#if 0	//sxs 20180801 : need check it later:  msm_dwc3.c
extern int get_usbid_adc(void);
#else
inline int get_usbid_adc(void) { return 1800; };
#endif
static int cms_check_usb_connecter_protect(struct cms *chip, int batt_cap_mah, int batt_temp)
{
	union power_supply_propval usb_conn_temp = {0,};
	int current_ma = batt_cap_mah;
	int batt_scale = 100;
	int try_again = 0;
	int i = 0;

	if (!chip->online) {
		cms_print(chip, PR_WARN, "usb isn't online!\n");
		return 0;
	}

	if (!chip->usb_connecter_protect_enable) {
		cms_print(chip, PR_WARN, "usb_connecter_protect isn't enabled!\n");
		return 0;
	}

	if (cms_is_voter_priority(chip)) {
		cms_print(chip, PR_WARN, "usb_connecter_protect isn't priority!\n");
		return 0;
	}

	/* 1. usb_id water protect */
	if (chip->usb_connecter_protect_enable & USB_ID_PROTECT) {
		if ((chip->continue_count % 3) == 0) {
usb_id:
			chip->usbid_mv = get_usbid_adc();
			if (chip->usbid_mv  <= chip->usb_id_protect_data[USBID_WATER_TRIGGER]) {
				if (!test_bit(HEALTH_STATUS_USBID_WATER, &chip->health_status) && !try_again) {
					msleep(500);
					try_again = 1;
					goto usb_id;
				}
				cms_notify_changed(chip, HEALTH_STATUS_USBID_WATER, true);
				current_ma = chip->usb_id_protect_data[USBID_WATER_ACTION];
				batt_scale = (current_ma * 100) / batt_cap_mah + 1;
				vote(chip->user_ibat_votable, USBID_WATER_EVENT, true, batt_scale);
			} else if (chip->usbid_mv  > chip->usb_id_protect_data[USBID_WATER_RELEASE]) {
				if (test_bit(HEALTH_STATUS_USBID_WATER, &chip->health_status) && !try_again) {
					msleep(500);
					try_again = 1;
					goto usb_id;
				}
				cms_notify_changed(chip, HEALTH_STATUS_USBID_WATER, false);
				vote(chip->user_ibat_votable, USBID_WATER_EVENT, false, 0);
			}
		}
	} else {
                chip->usbid_mv = get_usbid_adc();
                if (chip->usbid_mv <= chip->usb_id_protect_data[USBID_WATER_TRIGGER]) {
                        //fuelsummary_collect_value(ID_BIT__USB_WATER, 1);
                } else {
                        //fuelsummary_collect_value(ID_BIT__USB_WATER, 0);
                }
	}

	/* 2. usb_conn_temp */
	if (chip->usb_connecter_protect_enable & USB_CONN_TEMP_HEAT_PROTECT) {

		for (i = 0; i < chip->usb_conn_heat_protect_data_rc[ROW]; i++) {
			if (is_between(chip->usb_conn_heat_protect_data[i].Tmin, chip->usb_conn_heat_protect_data[i].Tmax, batt_temp)) {
				break;
			}
		}

		if (chip->usb_psy) {
			power_supply_get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_USB_CONN_TEMP, &usb_conn_temp);
		}
		if (usb_conn_temp.intval >= chip->usb_conn_heat_protect_data[i].trigger_data) {
			if (chip->usb_conn_heat_protect_count >= 2) {
				batt_scale = 0;
				cms_notify_changed(chip, HEALTH_STATUS_USB_CONN_HEAT, true);
				vote(chip->user_ibat_votable, USB_CONN_HEAT_EVENT, true, batt_scale);
			} else
				chip->usb_conn_heat_protect_count++;
		} else if (usb_conn_temp.intval < chip->usb_conn_heat_protect_data[i].release_data) {
			chip->usb_conn_heat_protect_count = 0;
			cms_notify_changed(chip, HEALTH_STATUS_USB_CONN_HEAT, false);
			vote(chip->user_ibat_votable, USB_CONN_HEAT_EVENT, false, 0);
		}
	}

	cms_print(chip, PR_WARN, "usb_connecter_protect_enable=0x%x, usbid_mv=%dmV, batt_temp=%d, usb_conn_temp=%d. (batt_temp=[%d,%d],usb_conn_temp=[%d,%d]). usb_conn_heat_protect_count=%d\n",
		chip->usb_connecter_protect_enable, chip->usbid_mv, batt_temp, usb_conn_temp.intval,
		chip->usb_conn_heat_protect_data[i].Tmin, chip->usb_conn_heat_protect_data[i].Tmax, chip->usb_conn_heat_protect_data[i].trigger_data, chip->usb_conn_heat_protect_data[i].release_data, chip->usb_conn_heat_protect_count);
	return 0;
}

static int cms_check_dchg_ntc_protect(struct cms *chip)
{
	union power_supply_propval usb_conn_temp = {0,};
	union power_supply_propval bat_board_temp = {0,};
	union power_supply_propval bat_conn_temp = {0,};
	union power_supply_propval master_bat_conn_temp = {0,};
	int batt_scale = 100;

    if (!chip->direct_charger_enable || !chip->dchg_ntc_enable || !chip->mcu_psy)
		return 0;

	power_supply_get_property(chip->usb_psy,
		POWER_SUPPLY_PROP_USB_CONN_TEMP, &usb_conn_temp);
	power_supply_get_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_BAT_BOARD_TEMP, &bat_board_temp);
	power_supply_get_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_BAT_CONN_TEMP, &bat_conn_temp);
	power_supply_get_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_MASTER_BAT_CONN_TEMP, &master_bat_conn_temp);
	/* add for factory charging test */
	chip->usb_connector_temp = usb_conn_temp.intval;
	chip->battery_board_temp = bat_board_temp.intval;
	chip->sub_bat_connector_temp = bat_conn_temp.intval;
	chip->main_bat_connector_temp = master_bat_conn_temp.intval;

	/*wait 100s after the MCU 90s handshake & MCU cable_R calculate, because PMI suspend will affect the MCU Cable_R calculate.*/
//	if (chip->continue_count > 10) {
		if (usb_conn_temp.intval >= chip->dchg_ntc_data[USB_CONN_TEMP]) {
			if (chip->usb_conn_temp_high_count >= 1) {	//加快 USB_CONN_NTC 保护速度
				batt_scale = 0;
				cms_print(chip,PR_WARN,"usb conn temp warning (%d). \n", usb_conn_temp.intval);
			} else
				chip->usb_conn_temp_high_count++;
		} else
			chip->usb_conn_temp_high_count = 0;

		if (bat_board_temp.intval >= chip->dchg_ntc_data[BAT_BOARD_TEMP]) {
			if (chip->bat_board_temp_high_count >= 2) {
				batt_scale = 0;
				cms_print(chip,PR_WARN,"bat board temp warning (%d). \n", bat_board_temp.intval);
			} else
				chip->bat_board_temp_high_count++;
		} else
			chip->bat_board_temp_high_count = 0;

		if (bat_conn_temp.intval >= chip->dchg_ntc_data[BAT_CONN_TEMP]) {
			if (chip->bat_conn_temp_high_count >= 2) {
				batt_scale = 0;
				cms_print(chip,PR_WARN,"bat conn temp warning (%d). \n", bat_conn_temp.intval);
			} else
				chip->bat_conn_temp_high_count++;
		} else
			chip->bat_conn_temp_high_count = 0;

		if (master_bat_conn_temp.intval >= chip->dchg_ntc_data[MASTER_BAT_CONN_TEMP]) {
			if (chip->master_bat_conn_temp_high_count >= 2) {
				batt_scale = 0;
				cms_print(chip,PR_WARN,"pcb conn temp warning (%d). \n", master_bat_conn_temp.intval);
			} else
				chip->master_bat_conn_temp_high_count++;
		} else
			chip->master_bat_conn_temp_high_count = 0;

		vote(chip->user_ibat_votable, DCHG_NTC_EVENT, !batt_scale, 0);
//	}

	cms_print(chip,PR_INFO," bat_board_temp=%d, usb_conn_temp=%d, master_bat_conn_temp=%d, bat_conn_temp=%d. count=[%d,%d,%d,%d]. NTC %s.\n",
		bat_board_temp.intval, usb_conn_temp.intval, master_bat_conn_temp.intval, bat_conn_temp.intval,
		chip->usb_conn_temp_high_count, chip->bat_board_temp_high_count, chip->bat_conn_temp_high_count, chip->master_bat_conn_temp_high_count,
		batt_scale ? "normal" : "error");

	if (0 != batt_scale && (chip->usb_conn_temp_high_count || chip->bat_board_temp_high_count || chip->bat_conn_temp_high_count || chip->master_bat_conn_temp_high_count))
		return 1;	//NTC high temp checking...
	else
		return 0;
}

static int cms_check_dchg_adapter_cooldown(struct cms *chip)
{
	union power_supply_propval adapter_temp = {0,};
	union power_supply_propval dchg_enable = {0,};
	union power_supply_propval adapter_power_derate = {0,};
	union power_supply_propval adapter_power_derate_already = {0,};
	union power_supply_propval adapter_power = {0,};
	static union power_supply_propval batt_full_mah = {0,};
	static int trigger_temp, release_temp, active_current;
	int batt_scale = 100;
	int adapter_current;

    if (!chip->direct_charger_enable || !chip->mcu_psy)
		return 0;

	power_supply_get_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_DCHG_ENABLED, &dchg_enable);

	if (dchg_enable.intval == 0) {
		cms_print(chip,PR_WARN,"Direct charging no enable, skip ...\n");
		vote(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE, false, 0);
		return 0;
	}
	if(chip->adapter_power_derate_enable){
		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_ADAPTER_TEMP, &adapter_temp);
		chip->adapter_temp = adapter_temp.intval; /* add for factory charging test */

		if (!batt_full_mah.intval) {
			power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, &batt_full_mah);
		}

		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_ADAPTER_POWER_DERATE, &adapter_power_derate);

		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_ADAPTER_POWER_DERATE_ALREADY, &adapter_power_derate_already);

		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_ADAPTER_POWER, &adapter_power);

		if(adapter_power_derate_already.intval && !chip->adapter_power_derate_param_ready){
			//usb plugin, only set once param
			//adapter_powerderate param already && never set param ready
			if(adapter_power_derate.intval != 0){
				//adapter_powerderate param read ok
				if((adapter_power.intval & 0x0F) == 0x03){
					//33W adapter
					trigger_temp = 800;
					release_temp = trigger_temp - 50;
					active_current = 3400;
				}else{
					if((adapter_power.intval & 0x0F) == 0x03){
						adapter_current = 3000;
					}else if((adapter_power.intval & 0x0F) == 0x02){
						adapter_current = 2250;
					}else if((adapter_power.intval & 0x0F) == 0x01){
						adapter_current = 4000;
					}else{
						adapter_current = 2250;
					}
					trigger_temp = (int)(adapter_power_derate.intval)&0xff;
					active_current = (((int)((adapter_power_derate.intval >> 8)&0xff)) * adapter_current * 2)/100;
					release_temp = (int)(adapter_power_derate.intval >> 16)&0xff;
					pr_err("notice: trigger %d, release %d, active %d adapter_current %d\n", trigger_temp, release_temp, active_current, adapter_current);
				}
			}else{
				//adapter_powerderate param read fail
				if((adapter_power.intval & 0x0F) == 0x03){
					//33W adapter
					trigger_temp = 800;
					release_temp = trigger_temp - 50;
					active_current = 3400;
				}else if((adapter_power.intval & 0x0F) == 0x02){
					//22.5W adapter
					trigger_temp = 900;
					release_temp = trigger_temp - 50;
					active_current = 3400;
				}else if((adapter_power.intval & 0x0F) == 0x01){
					//44W adapter
					trigger_temp = 1000;
					release_temp = trigger_temp - 50;
					active_current = 5000;
				}else{
					trigger_temp = 900;
					release_temp = trigger_temp - 50;
					active_current = 3400;
				}
			}
			chip->adapter_power_derate_param_ready = 1;
		}

		if(chip->adapter_power_derate_param_ready){
			if (adapter_temp.intval > trigger_temp) {
				batt_scale = (active_current * 100) / batt_full_mah.intval;
				vote(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE, true, batt_scale);
			} else if (adapter_temp.intval < release_temp) {
				batt_scale = 100;
				vote(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE, false, 0);
			}
			cms_print(chip,PR_INFO,"power derate adapter_temp=%d[%d, %d, %d], batt_full_mah=%d, batt_scale=%d(%dmA).  Dchg Adapter is %s.\n",
			adapter_temp.intval, trigger_temp, release_temp, active_current,
			batt_full_mah.intval, batt_scale, (batt_scale * batt_full_mah.intval / 100),
			(is_client_vote_enabled_locked(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE))  ? "cooldown-ing" : "normal");

		}else{
			if (adapter_temp.intval > chip->adapter_cooldown_data[HEAT_TRIGGER]) {
				batt_scale = (chip->adapter_cooldown_data[IBAT_ACTION] * 100) / batt_full_mah.intval;
				vote(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE, true, batt_scale);
			} else if (adapter_temp.intval < chip->adapter_cooldown_data[HEAT_RELEASE]) {
				batt_scale = 100;
				vote(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE, false, 0);
			}
			cms_print(chip,PR_INFO," adapter_temp=%d[%d, %d, %d], batt_full_mah=%d, batt_scale=%d(%dmA).  Dchg Adapter is %s.\n",
			adapter_temp.intval, chip->adapter_cooldown_data[HEAT_TRIGGER], chip->adapter_cooldown_data[HEAT_RELEASE], chip->adapter_cooldown_data[IBAT_ACTION],
			batt_full_mah.intval, batt_scale, (batt_scale * batt_full_mah.intval / 100),
			(is_client_vote_enabled_locked(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE))  ? "cooldown-ing" : "normal");
		}
	}
	else
	{
		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_ADAPTER_TEMP, &adapter_temp);

		if (!batt_full_mah.intval) {
			power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, &batt_full_mah);
		}

		if (adapter_temp.intval > chip->adapter_cooldown_data[HEAT_TRIGGER]) {
			batt_scale = (chip->adapter_cooldown_data[IBAT_ACTION] * 100) / batt_full_mah.intval;
			vote(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE, true, batt_scale);
		} else if (adapter_temp.intval < chip->adapter_cooldown_data[HEAT_RELEASE]) {
			batt_scale = 100;
			vote(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE, false, 0);
		}

		cms_print(chip,PR_INFO," adapter_temp=%d[%d, %d, %d], batt_full_mah=%d, batt_scale=%d(%dmA).  Dchg Adapter is %s.\n",
			adapter_temp.intval, chip->adapter_cooldown_data[HEAT_TRIGGER], chip->adapter_cooldown_data[HEAT_RELEASE], chip->adapter_cooldown_data[IBAT_ACTION],
			batt_full_mah.intval, batt_scale, (batt_scale * batt_full_mah.intval / 100),
			(is_client_vote_enabled_locked(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE))  ? "cooldown-ing" : "normal");
	}
	return 1;
}

#if 0
static int cms_check_dchg_cableR_protect(struct cms *chip)
{
	union power_supply_propval dchg_cableR = {0,};
	union power_supply_propval dchg_enable = {0,};
	static union power_supply_propval batt_full_mah = {0,};
	int batt_scale = 100;
	int dchg_current_max = 4500;
		return 0;//add for test
    if (!chip->direct_charger_enable || !chip->mcu_psy)
		return 0;

	power_supply_get_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_DCHG_ENABLED, &dchg_enable);

	if (dchg_enable.intval == 0) {
		cms_print(chip,PR_WARN,"Direct charging no enable, skip ...\n");
		vote(chip->user_ibat_votable, DCHG_CABLER_CURRENTDOWN_CHARGE, false, 0);
		return 0;
	}

	power_supply_get_property(chip->mcu_psy,
		POWER_SUPPLY_PROP_DCHG_CABLER, &dchg_cableR);

	if (!batt_full_mah.intval) {
		power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, &batt_full_mah);
	}

	if (dchg_cableR.intval < 470) {
		dchg_current_max = 4500;
		batt_scale = (dchg_current_max * 100) / batt_full_mah.intval + 1;
		vote(chip->user_ibat_votable, DCHG_CABLER_CURRENTDOWN_CHARGE, true, batt_scale);
	} else /* if (dchg_cableR.intval <= 580) */ {
		dchg_current_max = 3600;
		batt_scale = (dchg_current_max * 100) / batt_full_mah.intval + 1;
		vote(chip->user_ibat_votable, DCHG_CABLER_CURRENTDOWN_CHARGE, true, batt_scale);
	}

	cms_print(chip,PR_INFO," dchg_cableR=%d,  batt_full_mah=%d, batt_scale=%d(%dmA).\n",
		dchg_cableR.intval, batt_full_mah.intval, batt_scale, (batt_scale * batt_full_mah.intval));
	return 1;
}
#endif

static int cms_check_abnormal_status(struct cms *chip, int batt_temp, int volt_now, int volt_max, int chg_status_now, int dchg_status_now)
{
	bool battid_done = false;
	int batt_present = 0;
	bool batt_ov = false;
	int index;
	int batt_scale = 100;

	if(!chip->online) {
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	for(index = 0; index < chip->normal_tc_rc[ROW]; index++){
		if(is_between(chip->normal_tc_data[index].min, chip->normal_tc_data[index].max,
			batt_temp)){
			batt_scale = chip->normal_tc_data[index].data;
			break;
		}
	}

	/*over heat*/
	cms_notify_changed(chip,HEALTH_STATUS_BAT_WARM,
	    index < chip->normal_tc_rc[ROW] / 2 && !batt_scale);

	/*cold*/
	cms_notify_changed(chip,HEALTH_STATUS_BAT_COLD,
		index > chip->normal_tc_rc[ROW] / 2 && !batt_scale);

	/*charging timeout*/
	cms_check_charge_timeout(chip, chg_status_now, dchg_status_now);

	/*htccc */
	cms_check_high_temp_chg_cur_control(chip,batt_temp);

	/*batt id check */
	mutex_lock(&chip->battid_lock);
	battid_done = chip->battery_rc.state == BAT_ID_IDLE && chip->battery_rc.counter > 1;
	if(chip->no_bat_warning)
		battid_done = 0;

	if(battid_done){
		/*check battery missing*/
		batt_present = cms_is_battery_present(chip);
		cms_notify_changed(chip, HEALTH_STATUS_BAT_MISSING, !batt_present);
		if(batt_present) {
                       if(chip->bat_det_method == BAT_DET_BY_EX_FG
                               && !is_atboot && ((!chip->battery_rc.bat_id && chip->battery_rc.counter > 1) || chip->ex_fg_fake_batt_id_start)) {
                                       chip->battery_rc.bat_id = cms_get_batid_by_ex_fg(chip);
                                       //if( chip->battery_rc.bat_id > 0)
                                       //        chip->health_status = 0;
                       }
			cms_notify_changed(chip, HEALTH_STATUS_BAT_INVALID,
				chip->battery_rc.bat_id == B_UNKOWN);
			volt_max = max(volt_max, 4380000);
			/*sw bat_ov || hw bat_ov*/
			batt_ov = (volt_now-volt_max) > BAT_HEALTH_OV_DELTA_MV*1000;
			if(batt_ov)
				cms_print(chip,PR_ERROR,"volt_now=%d,volt_max=%d\n", volt_now, volt_max);
			cms_notify_changed(chip,HEALTH_STATUS_BAT_OV,batt_ov);
		} else {
			if (chip->batt_plugable && chip->pon_batt_pluged) {
				cms_notify_changed(chip, HEALTH_STATUS_BAT_UNPLUGED, !batt_present);
				cms_print(chip,PR_WARN,"battery missing and plugable,shut down device\n");
			}
			cms_print(chip,PR_WARN,"battery missing\n");
		}
	}
	mutex_unlock(&chip->battid_lock);

	return batt_scale;
}

static int cms_check_abnormal_charge_current(struct cms *chip, int batt_temp)
{
	int batt_scale = 100;

	if (!chip->online) {
		cms_print(chip, PR_WARN, "usb isn't online!\n");
		return 0;
	}

	/* abnormal charge,scale set 0 */
    if ((chip->enabled && ((chip->bat_det_method != BAT_DET_BY_EX_FG || !chip->no_bat_id_keep_charging) && chip->battery_rc.bat_id == B_UNKOWN)) ||
		test_bit(HEALTH_STATUS_CHG_TIMEOUT, &chip->health_status) ||
		test_bit(HEALTH_STATUS_BAT_OV, &chip->health_status) ||
		test_bit(HEALTH_STATUS_HTCCC, &chip->health_status))
		batt_scale = 0;

	vote(chip->user_ibat_votable, ABNORMAL_EVENT, !batt_scale, 0);

#if 1	//PD1824/PD1916/PD1922 BQ27750 IIC error issue: use for keep charging current and charging UI.
	if (chip->no_bat_id_keep_charging) {
		if (!is_atboot) {
			if (chip->ex_fg_fake_batt_id_start ||
				(chip->bat_det_method == BAT_DET_BY_EX_FG && chip->battery_rc.bat_id == B_UNKOWN)) {
				vote(chip->user_ibat_votable, EX_FG_FAKE_BATT_ID_START, true, 20);	//keep charging UI in lower charging current
			} else {
				vote(chip->user_ibat_votable, EX_FG_FAKE_BATT_ID_START, false, 0);
			}
		}
	}
#endif
	return batt_scale;
}

static int cms_get_current_by_board_temp(struct cms *chip,struct tc_data_item *current_item,int len,int temp_now,
											int fast_chg_state, char *name)
{
	int index;
	int current_ma = 0;

	for(index = 0; index < len; index++){
		if(is_between(current_item[index].min, current_item[index].max,
			temp_now)){
			current_ma = current_item[index].data;
			break;
		}
	}

	cms_print(chip,PR_INFO,"%s(fast_chg_state=%d),temp=%d,get current_ma=%d\n", name, fast_chg_state, temp_now, current_ma);
	return current_ma;
}

static int cms_get_intell_charge_current(struct cms *chip,int fb_state,int pri_temp,int par_temp,
						int *primary_fastchg_ma, int *parallel_fastchg_ma, int batt_cap_mah, int fast_chg_state)
{
	struct tc_data_item *pri_data,*par_data;
	int pri_len,par_len,batt_scale,current_ma;

	switch(fb_state){
		case FB_ON:
			if (chip->ex_intell_charge_enable && !fast_chg_state) {
				pri_data = chip->ex_primary_fbon_tc_data;
				pri_len = chip->ex_primary_fbon_tc_rc[ROW];
				par_data = chip->ex_parallel_fbon_tc_data;
				par_len = chip->ex_parallel_fbon_tc_rc[ROW];
			} else {
				pri_data = chip->primary_fbon_tc_data;
				pri_len = chip->primary_fbon_tc_rc[ROW];
				par_data = chip->parallel_fbon_tc_data;
				par_len = chip->parallel_fbon_tc_rc[ROW];
			}
			break;
		case FB_OFF:
			if (chip->ex_intell_charge_enable && !fast_chg_state) {
				pri_data = chip->ex_primary_fboff_tc_data;
				pri_len = chip->ex_primary_fboff_tc_rc[ROW];
				par_data = chip->ex_parallel_fboff_tc_data;
				par_len = chip->ex_parallel_fboff_tc_rc[ROW];
			} else {
				pri_data = chip->primary_fboff_tc_data;
				pri_len = chip->primary_fboff_tc_rc[ROW];
				par_data = chip->parallel_fboff_tc_data;
				par_len = chip->parallel_fboff_tc_rc[ROW];
			}
			break;
		default:
			cms_print(chip,PR_ERROR,"fb_state is error\n");
			return 0;
	}

	current_ma = cms_get_current_by_board_temp(chip,pri_data,pri_len,
									pri_temp, fast_chg_state, "primary");
	*primary_fastchg_ma = min(*primary_fastchg_ma, current_ma);

	if(!chip->parallel_temp_enable[fb_state])
		par_temp = pri_temp;

	current_ma = cms_get_current_by_board_temp(chip,par_data, par_len,
									par_temp, fast_chg_state, "parallel");
	if(chip->direct_charger_enable) {
		*parallel_fastchg_ma = 0;
		if(current_ma > 0)
			*primary_fastchg_ma = min(current_ma, *primary_fastchg_ma);
		batt_scale =  (*primary_fastchg_ma + *parallel_fastchg_ma)*100/batt_cap_mah + 1;
	}else {
		*parallel_fastchg_ma = min(*parallel_fastchg_ma,current_ma);
		batt_scale = (*primary_fastchg_ma + *parallel_fastchg_ma)*100/batt_cap_mah+1;
	}
	cms_print(chip,PR_INFO,"primary_ma=%d,parallel_ma=%d,current_ma=%d,batt_scale=%d,batt_cap_mah=%d\n",
		*primary_fastchg_ma, *parallel_fastchg_ma, current_ma, batt_scale, batt_cap_mah);
	return batt_scale;
}

static int cms_get_step_charge_current(struct cms *chip,int index)
{
	union power_supply_propval volt_now = {0,};
	union power_supply_propval current_now = {0,};
	int vbat_uv,i;
	int vbat_uv_sum = 0;
	int batt_scale = 0;
	//int volt_delta = 50000;
	int r_comp = chip->vbat_r_comp;

	cms_ensure_psy_available(chip);

	if (!chip->batt_psy) {
		cms_print(chip,PR_ERROR,"batt_psy is NULL\n");
		return 0;
	}

	for(i=0; i<3; i++){
		power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &volt_now);
		power_supply_get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &current_now);
		if (current_now.intval > 0)
			current_now.intval = 0;
		vbat_uv_sum += (volt_now.intval + current_now.intval * r_comp / 1000);
	}
	vbat_uv = vbat_uv_sum/3;

	if (!chip->normal_tc_data[index].volt2) {	//two step
		if(vbat_uv <= chip->normal_tc_data[index].volt1){
			if(chip->last_step == 1) {
				batt_scale = chip->normal_tc_data[index].data_ex1;
			}else {
				batt_scale = chip->normal_tc_data[index].data;
				chip->last_step = 0;
			}
		}else if(vbat_uv > chip->normal_tc_data[index].volt1){
			batt_scale = chip->normal_tc_data[index].data_ex1;
			chip->last_step = 1;
		}
	} else {	//three step
		if(vbat_uv <= chip->normal_tc_data[index].volt1){
			if(chip->last_step == 2) {
				batt_scale = chip->normal_tc_data[index].data_ex2;
			}else if(chip->last_step == 1) {
				batt_scale = chip->normal_tc_data[index].data_ex1;
			}else {
				batt_scale = chip->normal_tc_data[index].data;
				chip->last_step = 0;
			}
		}else if(is_between(chip->normal_tc_data[index].volt1, chip->normal_tc_data[index].volt2, vbat_uv)){
			if(chip->last_step == 2) {
				batt_scale = chip->normal_tc_data[index].data_ex2;
			}else {
				batt_scale = chip->normal_tc_data[index].data_ex1;
				chip->last_step = 1;
			}
		}else if(vbat_uv > chip->normal_tc_data[index].volt2){
			if(chip->last_step == 0) {
				batt_scale = chip->normal_tc_data[index].data_ex1;
				chip->last_step = 1;
			}else if(chip->last_step == 1) {
				batt_scale = chip->normal_tc_data[index].data_ex2;
				chip->last_step = 2;
			}else {
				batt_scale = chip->normal_tc_data[index].data_ex2;
				chip->last_step = 2;
			}
		}
	}

	cms_print(chip, PR_INFO, "### step batt_scale = %d,volt = %d,r_comp = %d,last_step = %d ###\n",
							batt_scale, vbat_uv, r_comp, chip->last_step);

	return batt_scale;
}

static int cms_check_normal_charge_current(struct cms *chip,int batt_temp)
{
	int index;
	int batt_scale = 100, input_scale = 100;
	int *normal_data;

	if(!chip->online) {
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	if(cms_is_voter_priority(chip)){
		cms_print(chip,PR_WARN,"normal charge isn't priority!\n");
		return 0;
	}

	for(index = 0; index < chip->normal_tc_rc[ROW]; index++){
		if(is_between(chip->normal_tc_data[index].min, chip->normal_tc_data[index].max,
			batt_temp)){
			if(!chip->normal_tc_data[index].volt1){	// only one step.
				batt_scale = chip->normal_tc_data[index].data;
				chip->last_step = 0;
			}else{
				batt_scale = cms_get_step_charge_current(chip,index);
			}
			break;
		}
	}

	vote(chip->user_ibat_votable, NORMAL_CHARGE, true, batt_scale);
	vote(chip->user_input_votable, NORMAL_CHARGE, true, input_scale);

	normal_data = (int *)&chip->normal_tc_data[index];
	cms_print(chip,PR_INFO,"normal charge,input_scale:%d, batt_scale:%d. (%d: %d, %d, %d, %d, %d, %d, %d)\n", input_scale,batt_scale, index, normal_data[0], normal_data[1], normal_data[2], normal_data[3], normal_data[4], normal_data[5], normal_data[6]);

	return batt_scale;
}

static int cms_check_intell_charge_current(struct cms *chip, int batt_cap_mah, int fast_chg_state)
{
	bool system_start = ((u64)(local_clock()/1000000000)) > 150;// 2.5min
	int batt_scale = 0, intell_sum_ma = 0, exhibition_sum_ma = 0;
	union power_supply_propval primary_board_temp_now = {0,};
	union power_supply_propval parallel_board_temp_now = {0,};

	if (bsp_test_mode && fast_chg_state == 0) {
		return 0;
	}

	if(!chip->online) {
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	if (!chip->intell_charge_enable) {
		cms_print(chip,PR_WARN,"intell charge isn't enabled!\n");
		return 0;
	}

	//adjust based on board temperature after system power on for 3min
	if(!system_start) {
		cms_print(chip,PR_WARN,"waiting,until system power on for 3min!\n");
		return 0;
	}

	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PRIMARY_BOARD_TEMP, &primary_board_temp_now);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PARALLEL_BOARD_TEMP, &parallel_board_temp_now);
	/* add for factory charging test */
	chip->main_board_temp = primary_board_temp_now.intval;
	chip->sub_board_temp = parallel_board_temp_now.intval;

	if(cms_is_voter_priority(chip)){
		cms_print(chip,PR_WARN,"intell charge isn't priority!\n");
		return 0;
	}

	if(chip->fb_on){
		//if (!power_off_charging_mode) {
		if (1) {
			batt_scale = cms_get_intell_charge_current(chip,FB_ON,primary_board_temp_now.intval,parallel_board_temp_now.intval,
							&chip->intell_charge_primary_result_ma, &chip->intell_charge_parallel_result_ma, batt_cap_mah, fast_chg_state);
		}
	}else{
		batt_scale = cms_get_intell_charge_current(chip,FB_OFF,primary_board_temp_now.intval,parallel_board_temp_now.intval,
							&chip->intell_charge_primary_result_ma, &chip->intell_charge_parallel_result_ma, batt_cap_mah, fast_chg_state);
	}
	cms_print(chip,PR_INFO,"fb %s primary:%d, parallel:%d, scale:%d, fast_chg_state=%d\n", chip->fb_on?"on":"off",
							chip->intell_charge_primary_result_ma, chip->intell_charge_parallel_result_ma, batt_scale, fast_chg_state);

	/* when in exhibition mode, select a lower current between intell_current and exhibit_current */
	intell_sum_ma = chip->intell_charge_primary_result_ma + chip->intell_charge_parallel_result_ma;
	exhibition_sum_ma = chip->exhibition_mode_current_primary + chip->exhibition_mode_current_parallel;
	if (chip->exhibition_mode && intell_sum_ma > exhibition_sum_ma) {
		chip->intell_charge_primary_result_ma = chip->exhibition_mode_current_primary;
		chip->intell_charge_parallel_result_ma = chip->exhibition_mode_current_parallel;
		batt_scale = (chip->intell_charge_primary_result_ma + chip->intell_charge_parallel_result_ma) * 100 / batt_cap_mah + 1;
		cms_print(chip, PR_INFO, "exhibition mode, current set to [%d, %d], batt_scale = %d\n",
				chip->intell_charge_primary_result_ma, chip->intell_charge_parallel_result_ma, batt_scale);
	}

	cms_update_fastchg_alloc(chip,chip->intell_charge_primary_result_ma,chip->intell_charge_parallel_result_ma);
	vote(chip->user_ibat_votable, INTELL_CHARGE, true, batt_scale);

	return batt_scale;
}

static int cms_check_fixed_charge_current(struct cms *chip,int batt_cap_mah)
{
	int batt_scale = 0;
	int primary_fastchg_ma = 0,parallel_fastchg_ma = 0;

	if(!chip->online) {
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	if (!chip->fixed_scale_enable) {
		cms_print(chip,PR_WARN,"fixed scale isn't enabled!\n");
		return 0;
	}

	if(cms_is_voter_priority(chip)){
		cms_print(chip,PR_WARN,"fixed scale isn't priority!\n");
		return 0;
	}

	/*adjust max fastchg after FIXED_TIMER min*/
	if (chip->fixed_scale_data[FIXED_TIMER] > 0 &&
		/*chip->nr_work > chip->fixed_scale_data[FIXED_TIMER] &&*/
		chip->continue_count > chip->fixed_scale_data[FIXED_TIMER]) {
		if (chip->intell_charge_enable){
			if((chip->intell_charge_primary_result_ma + chip->intell_charge_parallel_result_ma)
					> (chip->fixed_scale_data[FIXED_PRIMARY] + chip->fixed_scale_data[FIXED_PARALLEL])){
				primary_fastchg_ma = chip->fixed_scale_data[FIXED_PRIMARY];
				parallel_fastchg_ma = chip->fixed_scale_data[FIXED_PARALLEL];
			}else{
				cms_print(chip,PR_INFO,"intell charge control,no need change\n");
				return 0;
			}
		}else{
			primary_fastchg_ma = chip->fixed_scale_data[FIXED_PRIMARY];
			parallel_fastchg_ma = chip->fixed_scale_data[FIXED_PARALLEL];
		}
		batt_scale = (primary_fastchg_ma + parallel_fastchg_ma)*100/batt_cap_mah+1;
		cms_update_fastchg_alloc(chip,primary_fastchg_ma,parallel_fastchg_ma);
		vote(chip->user_ibat_votable, FIXED_CHARGE, true, batt_scale);

		cms_print(chip,PR_INFO,"%dmin primary:%d, parallel:%d, scale:%d\n", chip->fixed_scale_data[FIXED_TIMER]*10/60, primary_fastchg_ma, parallel_fastchg_ma, batt_scale);
	}

	return batt_scale;
}

static int cms_check_fbon_charge_current(struct cms *chip,int chg_volt)
{
	int batt_scale = 0;

	if(!chip->online) {
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	if (!chip->fbon_scale_enable) {
		cms_print(chip,PR_WARN,"fbon scale isn't enabled!\n");
		return 0;
	}

	if(cms_is_voter_priority(chip)){
		cms_print(chip,PR_WARN,"fbon scale isn't priority!\n");
		return 0;
	}

	/* scale input and ibat current after FBON_TIMER s */
	if(chip->fb_on && chip->nr_work > chip->fbon_scale_data[FBON_TIMER]) {
		if (!power_off_charging_mode) {
			cms_print(chip,PR_INFO,"fbon scale \n");
			vote(chip->user_ibat_votable, FBON_EVENT, true, chip->fbon_scale_data[FBON_IBAT]);
			if(chg_volt > 5)
				vote(chip->user_input_votable, FBON_EVENT, true, chip->fbon_scale_data[FBON_INPUT]);
			batt_scale = chip->fbon_scale_data[FBON_IBAT];
		}
	}else{
		vote(chip->user_ibat_votable, FBON_EVENT, false, 0);
		vote(chip->user_input_votable, FBON_EVENT, false, 0);
	}

	return batt_scale;
}
/*
static int cms_check_switch_charge_current(struct cms *chip)
{
	int batt_scale = 0;

	if(!chip->online) {
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	if (!chip->switch_scale_enable) {
		cms_print(chip,PR_WARN,"switch scale isn't enabled!\n");
		return 0;
	}

	if(cms_is_voter_priority(chip)){
		cms_print(chip,PR_WARN,"switch scale isn't priority!\n");
		return 0;
	}

	//fast charge switch
	if(!chip->switch_state){
		cms_print(chip,PR_INFO,"switch scale \n");
		vote(chip->user_ibat_votable, SWITCH_EVENT, true, chip->switch_scale_data[SWITCH_IBAT]);
		vote(chip->user_input_votable, SWITCH_EVENT, true, chip->switch_scale_data[SWITCH_INPUT]);
		batt_scale = chip->switch_scale_data[SWITCH_IBAT];
	}else{
		vote(chip->user_ibat_votable, SWITCH_EVENT, false, 0);
		vote(chip->user_input_votable, SWITCH_EVENT, false, 0);
	}

	return batt_scale;
}
*/
static int cms_ai_switch_charge_current(struct cms *chip)
{
	int batt_scale = 0;
	union power_supply_propval value = {0,};

	if(!chip->online) {
		chip->ai_disable_charge = 0;
		chip->ai_disable_charge_state = false;
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	if (!chip->ai_charge_enable) {
		cms_print(chip,PR_WARN,"ai charge isn't enabled!\n");
		return 0;
	}

	if(cms_is_voter_priority(chip)){
		cms_print(chip,PR_WARN,"switch scale isn't priority!\n");
		return 0;
	}

	//ai charge switch
	cms_print(chip,PR_INFO,"ai switch scale enable=%d,ai_disable_charge_state=%d\n", chip->ai_disable_charge, chip->ai_disable_charge_state);
	if(chip->ai_disable_charge == 1 && !chip->ai_disable_charge_state) {
		vote(chip->dchg_disable_votable, AI_SWITCH_EVENT, true, 0);
		vote(chip->user_ibat_votable, AI_SWITCH_EVENT, true, batt_scale);
		chip->ai_disable_charge_state = true;
	}else if(chip->ai_disable_charge == 2 && !chip->ai_disable_charge_state) {
		if(chip->usb_psy) {
			value.intval = 1;
			power_supply_set_property(chip->usb_psy,
						POWER_SUPPLY_AI_CHARGE_STAE, &value);
		}
		vote(chip->dchg_disable_votable, AI_SWITCH_EVENT, true, 0);
		vote(chip->user_ibat_votable, AI_SWITCH_EVENT, true, batt_scale);
		chip->ai_disable_charge_state = true;
	}else if((chip->ai_disable_charge == 3 || chip->ai_disable_charge == 0) && chip->ai_disable_charge_state) {
		if(chip->usb_psy) {
			value.intval = 0;
			power_supply_set_property(chip->usb_psy,
						POWER_SUPPLY_AI_CHARGE_STAE, &value);
		}
		vote(chip->user_ibat_votable, AI_SWITCH_EVENT, false, 0);
		vote(chip->dchg_disable_votable, AI_SWITCH_EVENT, false, 0);
		chip->ai_disable_charge_state = false;
	}

	return batt_scale;
}

static int cms_check_weixin_calling_charge_current(struct cms *chip)
{
	int batt_scale = 0;

	if(!chip->online) {
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	if (!chip->weixin_calling_scale_enable) {
		cms_print(chip,PR_WARN,"weixin_calling_scale_enable isn't enabled!\n");
		return 0;
	}

	if(cms_is_voter_priority(chip)){
		cms_print(chip,PR_WARN,"weixin_calling_scale isn't priority!\n");
		return 0;
	}

	if(chip->weixin_calling_state){
		cms_print(chip,PR_INFO,"weixin_calling_state scale \n");
		vote(chip->user_ibat_votable, WEIXIN_CALLING_EVENT, true, chip->weixin_calling_scale_data[CALLING_IBAT]);
		vote(chip->user_input_votable, WEIXIN_CALLING_EVENT, true, chip->weixin_calling_scale_data[CALLING_INPUT]);
		batt_scale = chip->weixin_calling_scale_data[CALLING_IBAT];
	}else{
		vote(chip->user_ibat_votable, WEIXIN_CALLING_EVENT, false, 0);
		vote(chip->user_input_votable, WEIXIN_CALLING_EVENT, false, 0);
	}

	return batt_scale;
}

static int cms_check_calling_charge_current(struct cms *chip)
{
	int batt_scale = 0;
	union power_supply_propval pval = {0,};

	if(!chip->online) {
		cms_print(chip,PR_WARN,"usb isn't online!\n");
		return 0;
	}

	if (!chip->calling_scale_enable) {
		cms_print(chip,PR_WARN,"calling scale isn't enabled!\n");
		return 0;
	}

	if (power_off_charging_mode) {
		cms_print(chip, PR_WARN, "power off charing mode,not calling scale\n");
		return 0;
	}

	if(cms_is_voter_priority(chip)){
		cms_print(chip, PR_WARN, "calling scale isn't priority!\n");
		return 0;
	}

	if (chip->direct_charger_enable && chip->calling_dchg_disable) {
		if (!chip->dchg_disable_votable)
			chip->dchg_disable_votable = find_votable("DCHG_DISABLE");

		if (chip->dchg_disable_votable == NULL) {
			cms_print(chip, PR_INFO, "Couldn't find DCHG_DISABLE votable\n");
			return 0;
		}
	}

	//input and ibat scale base on calling
	if(chip->calling_state){
		cms_print(chip, PR_INFO, "start calling scale\n");
		if (chip->dchg_disable_votable) {
			if(chip->mcu_psy)
					power_supply_set_property(chip->mcu_psy,
						POWER_SUPPLY_PROP_DCHG_CALL_START, &pval);
			vote(chip->dchg_disable_votable, CALLING_EVENT, true, 0);
		} else {
			vote(chip->user_ibat_votable, CALLING_EVENT, true, chip->calling_scale_data[CALLING_IBAT]);
			vote(chip->user_input_votable, CALLING_EVENT, true, chip->calling_scale_data[CALLING_INPUT]);
			batt_scale = chip->calling_scale_data[CALLING_IBAT];
		}

	} else {
		cms_print(chip, PR_INFO, "stop calling scale\n");
		if (chip->dchg_disable_votable) {
			vote(chip->dchg_disable_votable, CALLING_EVENT, false, 0);
		} else {
			vote(chip->user_ibat_votable, CALLING_EVENT, false, 0);
			vote(chip->user_input_votable, CALLING_EVENT, false, 0);
		}
	}

	return batt_scale;
}

static int cms_check_factory_mode_charge_current(struct cms *chip,int batt_cap_mah,
							int primary_fastchg_max_ma,int parallel_fastchg_max_ma)
{
	int batt_scale = 100;
	static bool run = false;

	//ibat scale base on factory mode
	if(!chip->factory_mode_state){
		cms_print(chip,PR_WARN,"factory mode isn't enabled!\n");
		if(run){
			force_active_votable(chip->user_ibat_votable,"factory mode",false,0);
			force_active_votable(chip->user_input_votable, "factory mode", false, 0);
			rerun_election(chip->user_ibat_votable);
			rerun_election(chip->user_input_votable);
			run = false;
		}
		return 0;
	}

	run = true;
	cms_print(chip,PR_INFO,"In factory mode,set max ibat current \n");
	cms_update_fastchg_alloc(chip,primary_fastchg_max_ma,parallel_fastchg_max_ma);
	batt_scale = (primary_fastchg_max_ma + parallel_fastchg_max_ma)*100/batt_cap_mah+1;
	force_active_votable(chip->user_ibat_votable,"factory mode",true,batt_scale);
	force_active_votable(chip->user_input_votable, "factory mode", true, 100);

	return batt_scale;
}


static bool get_fuelsummary_user_custom(struct cms *chip)
{
	union power_supply_propval user_custom = {0,};
	union power_supply_propval custom_input = {0,};
	union power_supply_propval custom_current = {0,};

	if (!chip->fuelsummary_psy) {
		chip->fuelsummary_psy = power_supply_get_by_name("fuelsummary");
		if (!chip->fuelsummary_psy) {
			cms_print(chip, PR_WARN, "fuelsummary_psy not found\n");
			goto exit;
		}
	}

	power_supply_get_property(chip->fuelsummary_psy,
			POWER_SUPPLY_PROP_FUEL_USER_CUSTOM,
			&user_custom);
	power_supply_get_property(chip->fuelsummary_psy,
			POWER_SUPPLY_PROP_FUEL_CUSTOM_INPUT,
			&custom_input);
	power_supply_get_property(chip->fuelsummary_psy,
			POWER_SUPPLY_PROP_FUEL_CUSTOM_CURRENT,
			&custom_current);

	chip->custom_input = custom_input.intval;
	chip->custom_current = custom_current.intval;
exit:
	return (0 != user_custom.intval);
}


static void cms_check_usbin_cutoff_status(struct cms *chip, int soc){
	static bool current_status;
	union power_supply_propval batt_profile_load = {0,};
	union power_supply_propval value = {0,};
	union power_supply_propval chg_status = {0,};
	int rc;
	static bool force_start_flag = false;

	if (!bsp_test_mode) {
		return;
	}

	rc = cms_ensure_psy_available(chip);
	if (!chip->batt_psy || !chip->usb_psy || !chip->bms_psy) {
		cms_print(chip,PR_ERROR,"!bat_psy || !usb_psy || !bms\n");
		return;
	}

	power_supply_get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_BATTERY_PARAM_STATUS,
			&batt_profile_load);

	if (!(batt_profile_load.intval)) {
		cms_print(chip, PR_WARN, "BMS: battery profile not load\n");
		return;
	}

	if (soc < chip->bsptest_soc_range[0]) {	//start charge
		current_status = false;
	} else if (soc > chip->bsptest_soc_range[1]) {	//stop charge
		current_status = true;
	} else {
		//return;
		goto OUT;
	}

	if (chip->batt_psy) {
		power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_STATUS_EX, &chg_status);
	}

	if ((chip->usbin_cutoff_status != current_status || chg_status.intval == POWER_SUPPLY_STATUS_CHARGING) && current_status) {
		force_start_flag = false;
		cms_print(chip, PR_WARN, "usbin stop chg\n");
		value.intval = 0;
		power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &value);

		vote(chip->user_ibat_votable, DEBUGFS_EVENT, true, 0);

		if (chip->direct_charger_enable) {
			if (!chip->dchg_disable_votable)
				chip->dchg_disable_votable = find_votable("DCHG_DISABLE");
			if (chip->dchg_disable_votable) {
				vote(chip->dchg_disable_votable, DEBUGFS_EVENT, true, 0);
			}
		}

		chip->usbin_cutoff_status = current_status;
	}

	if (((chip->usbin_cutoff_status != current_status || chg_status.intval == POWER_SUPPLY_STATUS_DISCHARGING) && !current_status) ||
		(!chip->usbin_cutoff_status && soc < 5 && !force_start_flag)) {
		force_start_flag = true;
		chip->usbin_cutoff_status = current_status;
		cms_print(chip, PR_WARN, "usbin start chg\n");
		value.intval = 1;
		power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &value);

		vote(chip->user_ibat_votable, DEBUGFS_EVENT, false, 0);

		if (chip->direct_charger_enable) {
			if (!chip->dchg_disable_votable)
				chip->dchg_disable_votable = find_votable("DCHG_DISABLE");
			if (chip->dchg_disable_votable) {
				vote(chip->dchg_disable_votable, DEBUGFS_EVENT, false, 0);
			}
		}
	}

OUT:
	cms_print(chip, PR_WARN, "soc:%d, usbin_cutoff_status:%d, current_status:%d\n", soc, chip->usbin_cutoff_status, current_status);
}

static void cms_bsp_charging_work(struct work_struct *work)
{
	struct cms *chip = container_of(work, struct cms, bsp_charging_work.work);
	int work_period_ms = msecs_to_jiffies(21000);
	union power_supply_propval batt_soc = {0,};

	if (chip->batt_psy)
		power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, &batt_soc);

	cms_print(chip,PR_INFO," cms_bsp_charging_work START-----\n");

	/*functions for BSPTEST control keep the battery level hold on ~= [45%, 55%].*/
	cms_check_usbin_cutoff_status(chip, batt_soc.intval);

	cancel_delayed_work(&chip->bsp_charging_work);
	schedule_delayed_work__on_system_unbound_wq(&chip->bsp_charging_work,
		round_jiffies_relative(work_period_ms));

}

/**********************
 * cms main work function *
 **********************/
static void cms_main_work(struct work_struct *work)
{
	struct cms *chip = container_of(work, struct cms, cms_work.work);
	int work_period_ms = msecs_to_jiffies(CMS_MAIN_WORK_PERIOD_MS);
	static int pre_batt_soc = -1;

	union power_supply_propval batt_temp_now = {0,};
	union power_supply_propval volt_now = {0,};
	union power_supply_propval volt_max = {0,};
	union power_supply_propval batt_soc = {0,};
	union power_supply_propval batt_cap_mah = {0,};
	union power_supply_propval usb_type = {0,};
	union power_supply_propval chg_volt_v = {0,};
	union power_supply_propval chg_status = {0,};
	union power_supply_propval pmi_status = {0,};
	union power_supply_propval primary_fastchg_max_ma = {0,};
	union power_supply_propval parallel_fastchg_max_ma = {0,};
	union power_supply_propval dchg_status = {0, };
	union power_supply_propval scene = {0,};
	union power_supply_propval dchg_support = {0,};
	int fast_chg_state = 0;
	int ntc_high_temp_checking = 0;

	if (!chip->online){
		cms_print(chip,PR_WARN,"usb isn't online\n");
		return;
	}

	cms_ensure_psy_available(chip);

	if(!chip->batt_psy || !chip->usb_psy){
		cms_print(chip,PR_WARN,"batt or usb psy isn't exist\n");
		goto out;
	}

	if (!chip->fb_on && is_panel_backlight_on()) {
		chip->nr_work = 0;
		cms_print(chip, PR_INFO, "cms check the lcd on.\n");
	} else if (chip->fb_on && !is_panel_backlight_on()) {
		cms_print(chip, PR_INFO, "cms check the lcd off.\n");
	}
	chip->fb_on = is_panel_backlight_on();

	cms_print(chip,PR_INFO,"===============%s===============\n",chip->name);

	/* get psy data */
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_CAPACITY, &batt_soc);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_TEMP, &batt_temp_now);
	chip->battery_temp = batt_temp_now.intval; /* add for factory charging test */
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, &batt_cap_mah);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &volt_now);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &volt_max);
	power_supply_get_property(chip->usb_psy,
		POWER_SUPPLY_PROP_REAL_TYPE, &usb_type);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_CHG_VOLTAGE_EX, &chg_volt_v);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PMI_STATUS, &pmi_status);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PRIMARY_FASTCHG_MAX, &primary_fastchg_max_ma);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PARALLEL_FASTCHG_MAX, &parallel_fastchg_max_ma);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_STATUS, &chg_status);
	power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_DCHG_STATUS, &dchg_status);
	if(chip->direct_charger_enable && chip->mcu_psy) {
		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_HANDSHAKE_STATUS, &dchg_support);
		fast_chg_state = (dchg_support.intval > 0) ? 1 : 0;	//vivo Dchg
	} else
		fast_chg_state = (chg_volt_v.intval >= 9) ? 1 : 0;	// > QC2.0

	/* initial variables */
	chip->intell_charge_primary_result_ma = primary_fastchg_max_ma.intval;
	chip->intell_charge_parallel_result_ma = parallel_fastchg_max_ma.intval;

	/* cms report soc */
	if(chip->bms_psy){
		if(pre_batt_soc != batt_soc.intval){
			cms_print(chip,PR_INFO,"cms report soc,pre_batt_soc=%d,batt_soc=%d\n",pre_batt_soc,batt_soc.intval);
			pre_batt_soc = batt_soc.intval;
			power_supply_changed(chip->bms_psy);
		}
	}

	/* function for check abnormal status */
	cms_check_abnormal_status(chip, batt_temp_now.intval, volt_now.intval,
				volt_max.intval, chg_status.intval, dchg_status.intval);

	/* ensure pmi status */
	if(!pmi_status.intval && !chip->direct_charger_enable){
		cms_print(chip,PR_INFO,"pmi_status:%d direct_charger_enable=%d,isn't readly!\n", pmi_status.intval, chip->direct_charger_enable);
		goto out;
	}

	/* function for abnormal charge handling */
	cms_check_abnormal_charge_current(chip, batt_temp_now.intval);

	/* function for check USB connecter protect */
	cms_check_usb_connecter_protect(chip, batt_cap_mah.intval, batt_temp_now.intval);

	/* function for  direct charging NTC protect */
	ntc_high_temp_checking = cms_check_dchg_ntc_protect(chip);

	/* function for  direct charging Adapter Cool Down Action */
	cms_check_dchg_adapter_cooldown(chip);

	/* function for  direct charging CableR current down */
	//cms_check_dchg_cableR_protect(chip);	//move it to STM32 driver side.

	/* function for regulating charge current */
	cms_check_normal_charge_current(chip,batt_temp_now.intval);
	if (chip->continue_count >= 2 || fast_chg_state)	//wait fast_chg_state update normal
		cms_check_intell_charge_current(chip, batt_cap_mah.intval, fast_chg_state);
	cms_check_fixed_charge_current(chip,batt_cap_mah.intval);
	cms_check_fbon_charge_current(chip, chg_volt_v.intval);
	/*cms_check_switch_charge_current(chip);*/
	cms_ai_switch_charge_current(chip);
	cms_check_weixin_calling_charge_current(chip);
	cms_check_calling_charge_current(chip);
	cms_check_factory_mode_charge_current(chip,batt_cap_mah.intval,
				primary_fastchg_max_ma.intval,parallel_fastchg_max_ma.intval);

	/* function for heavy load during dchg, determine whether to switch to PMIC */
	cms_check_heavy_load_dchg(chip, dchg_status.intval);

	/*functions for large current test in factory mode*/
	cms_check_fastchg_current(chip);

	/*reset after 360*10/60 mins=1h*/
	if (chip->nr_work++ >= 360) {
		chip->nr_work = 0;
	}

	/*reset after 3600*10/60 mins=10h*/
	if (chip->continue_count++ >= 3600)
		chip->continue_count = 0;

	cms_print(chip, PR_INFO, "vbat=%d,continue_count=%d,nr_work=%d,fb_on=%d,temp=%d,chg_status=%d,usb_type=%d,chg_volt_v=%d,input_scale(%s)=%d,batt_scale(%s)=%d,batid=%d(%d),health=0x%X,batt_soc=%d,bsptest=%d[%d, %d],fast_chg_state=%d,dchg_support=%d\n",
		volt_now.intval, chip->continue_count, chip->nr_work, chip->fb_on, batt_temp_now.intval, chg_status.intval, usb_type.intval, chg_volt_v.intval, get_effective_client(chip->user_input_votable),
		get_effective_result(chip->user_input_votable), get_effective_client(chip->user_ibat_votable), get_effective_result(chip->user_ibat_votable), chip->battery_rc.bat_id, chip->no_bat_id_keep_charging, (int)chip->health_status, batt_soc.intval, bsp_test_mode, chip->bsptest_soc_range[0], chip->bsptest_soc_range[1], fast_chg_state, dchg_support.intval);
out:
	if(!chip->health_status)
		work_period_ms *= 2;

	if (ntc_high_temp_checking)	//quickly for NTC high temp checking process...
		work_period_ms = msecs_to_jiffies(2000);

	if (chip->fuelsummary_psy)
		power_supply_set_property(chip->fuelsummary_psy,
				POWER_SUPPLY_PROP_FEX_RUNNING, &scene);

	mutex_lock(&chip->status_lock);
	if(chip->enabled && chip->online){
		schedule_delayed_work__on_system_unbound_wq(&chip->cms_work,
			round_jiffies_relative(work_period_ms));
	}
	mutex_unlock(&chip->status_lock);
}

/*********************
 *  cms votable function   *
 *********************/
static int cms_user_ibat_vote_callback(struct votable *votable, void *data,
			int scale, const char *client)
{
	struct cms *chip = data;
	int rc = 0;

	if (!client || scale < 0)
		return -EINVAL;

	cms_print(chip,PR_INFO,"user ibat,voter:%s,scale:%d\n",client,scale);

	cms_scale_ibat_current(chip, scale);

	if (chip->direct_charger_enable)
		cms_scale_dchg_current(chip, scale);

	return rc;
}

static int cms_user_input_vote_callback(struct votable *votable, void *data,
			int scale, const char *client)
{
	struct cms *chip = data;
	int rc = 0;

	if (!client || scale < 0)
		return -EINVAL;

	cms_print(chip,PR_INFO,"user input,voter:%s,scale:%d\n",client,scale);

	cms_scale_input_current(chip, scale);

	return rc;
}

static int cms_create_votables(struct cms *chip)
{
	int rc = 0;

	chip->user_ibat_votable = create_votable("USER_IBAT", VOTE_MIN,
					cms_user_ibat_vote_callback,
					chip);
	if (IS_ERR(chip->user_ibat_votable)) {
		rc = PTR_ERR(chip->user_ibat_votable);
		return rc;
	}

	chip->user_input_votable = create_votable("USER_INPUT", VOTE_MIN,
						cms_user_input_vote_callback,
						chip);
	if (IS_ERR(chip->user_input_votable)) {
		rc = PTR_ERR(chip->user_input_votable);
		return rc;
	}

	return rc;
}

static void cms_reset_voters(struct cms *chip)
{
	if (chip->user_ibat_votable){
		vote(chip->user_ibat_votable, AI_SWITCH_EVENT, false, 0);
		vote(chip->user_ibat_votable, CALLING_EVENT, false, 0);
		vote(chip->user_ibat_votable, WEIXIN_CALLING_EVENT, false, 0);
		vote(chip->user_ibat_votable, SWITCH_EVENT, false, 0);
		vote(chip->user_ibat_votable, FBON_EVENT, false, 0);
		vote(chip->user_ibat_votable, ABNORMAL_EVENT, false, 0);
		vote(chip->user_ibat_votable, DCHG_NTC_EVENT, false, 0);
		vote(chip->user_ibat_votable, DEBUGFS_EVENT, false, 0);
		vote(chip->user_ibat_votable, USBID_WATER_EVENT, false, 0);
		vote(chip->user_ibat_votable, USB_CONN_HEAT_EVENT, false, 0);
		vote(chip->user_ibat_votable, NORMAL_CHARGE, false, 0);
		vote(chip->user_ibat_votable, INTELL_CHARGE, false, 0);
		vote(chip->user_ibat_votable, FIXED_CHARGE, false, 0);
		vote(chip->user_ibat_votable, DCHG_ADAPTER_COOLDOWN_CHARGE, false, 0);
		vote(chip->user_ibat_votable, DCHG_CABLER_CURRENTDOWN_CHARGE, false, 0);
		vote(chip->user_ibat_votable, EX_FG_FAKE_BATT_ID_START, false, 0);
	}
	if (chip->user_input_votable){
		vote(chip->user_input_votable, CALLING_EVENT, false, 0);
		vote(chip->user_input_votable, WEIXIN_CALLING_EVENT, false, 0);
		vote(chip->user_input_votable, SWITCH_EVENT, false, 0);
		vote(chip->user_input_votable, FBON_EVENT, false, 0);
		vote(chip->user_input_votable, NORMAL_CHARGE, false, 0);
		vote(chip->user_input_votable, INTELL_CHARGE, false, 0);
		vote(chip->user_input_votable, FIXED_CHARGE, false, 0);
	}
	if (chip->direct_charger_enable) {
		if (!chip->dchg_disable_votable)
			chip->dchg_disable_votable = find_votable("DCHG_DISABLE");
		if (chip->dchg_disable_votable) {
			vote(chip->dchg_disable_votable, AI_SWITCH_EVENT, false, 0);
			vote(chip->dchg_disable_votable, DCHG_HEAVY_LOAD_EVENT, false, 0);
			if (bsp_test_mode)
				vote(chip->dchg_disable_votable, DEBUGFS_EVENT, false, 0);
		}
	}
}

static void cms_destroy_votables(struct cms *chip)
{
	if (chip->user_ibat_votable)
		destroy_votable(chip->user_ibat_votable);
	if (chip->user_input_votable)
		destroy_votable(chip->user_input_votable);
}

/****************************************************
 * 				cms debug class 					*
*****************************************************/

/************************************************
 *	node:  charge_test_param
 *	function:  factory charging test parameters
 ************************************************/
static ssize_t charge_test_param_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct cms *chip = container_of(c, struct cms,
						cms_debug_class);

	return scnprintf(buf, PAGE_SIZE, "charge test param threshold:%d,%d,%d,%d,%d,%d,%d,%d\n",
			chip->charge_param_judge_threshold[0], chip->charge_param_judge_threshold[1], 
			chip->charge_param_judge_threshold[2], chip->charge_param_judge_threshold[3], 
			chip->charge_param_judge_threshold[4], chip->charge_param_judge_threshold[5],
			chip->charge_param_judge_threshold[6], chip->charge_param_judge_threshold[7]);
}
static CLASS_ATTR_RO(charge_test_param);


/************************************************
 *	node:  charge_test
 *	function:  factory charging test results
 ************************************************/
static ssize_t charge_test_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct cms *chip = container_of(c, struct cms,
						cms_debug_class);

	cms_check_chg_error_state(chip);

	return scnprintf(buf, PAGE_SIZE, "%s\n", chip->chg_state);
}

static ssize_t charge_test_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct cms *chip = container_of(c, struct cms,
						cms_debug_class);
	int ret = count;
	union power_supply_propval value = {0,};
	union power_supply_propval supported_adapter = {0,};

	cms_ensure_psy_available(chip);
	if (!chip->batt_psy || !chip->mcu_psy) {
		cms_print(chip, PR_ERROR, "bat_psy or mcu_psy not registed\n");
		return ret;
	}

	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_SUPPORTED_ADAPTER, &supported_adapter);

	if (supported_adapter.intval == 2 || supported_adapter.intval == 4) { /* add for half charge test */
		if (!strncmp(buf, "disable-fast-charge", 19)) {
			value.intval = 1;
			power_supply_set_property(chip->mcu_psy,
				POWER_SUPPLY_PROP_FACTORY_10W_CHARGE_TEST, &value);
		}
	} else if (supported_adapter.intval == 3) {
		/* reserve for 9v/2A test */
	}

	return ret;
}
static CLASS_ATTR_RW(charge_test);


/************************************************
 *	node:  debug
 *	function:  debug info read/write
 ************************************************/
static ssize_t debug_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct cms *chip = container_of(c, struct cms,
						cms_debug_class);


	return scnprintf(buf, PAGE_SIZE, "present:%d, health:0x%02X\n",
			chip->enabled, (uint32_t)chip->health_status);
}

static ssize_t debug_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct cms *chip = container_of(c, struct cms,
						cms_debug_class);
	int ret = count;
	union power_supply_propval value = {0,};

	cms_ensure_psy_available(chip);

	if (!strncmp(buf, "health-warm", 11)) {
		cms_notify_changed(chip, HEALTH_STATUS_BAT_WARM, 1);
	} else if (!strncmp(buf, "health-cold", 11)) {
		cms_notify_changed(chip, HEALTH_STATUS_BAT_COLD, 1);
	} else if (!strncmp(buf, "health-bat-ov", 13)) {
		cms_notify_changed(chip, HEALTH_STATUS_BAT_OV, 1);
	} else if (!strncmp(buf, "health-chg-ov", 13)) {
		cms_notify_changed(chip, HEALTH_STATUS_CHG_OV, 1);
	} else if (!strncmp(buf, "health-chg-tmout", 16)) {
		cms_notify_changed(chip, HEALTH_STATUS_CHG_TIMEOUT, 1);
	} else if (!strncmp(buf, "health-missing", 14)) {
		cms_notify_changed(chip, HEALTH_STATUS_BAT_MISSING, 1);
	} else if (!strncmp(buf, "health-invalid", 14)) {
		cms_notify_changed(chip, HEALTH_STATUS_BAT_INVALID, 1);
	} else if (!strncmp(buf, "usbid_water", 11)) {
		cms_notify_changed(chip, HEALTH_STATUS_USBID_WATER, 1);
	} else if (!strncmp(buf, "usb_conn_heat", 13)) {
		cms_notify_changed(chip, HEALTH_STATUS_USB_CONN_HEAT, 1);
	} else if (!strncmp(buf, "chg_err", 7)) {
		cms_notify_changed(chip, HEALTH_STATUS_CHG_ERR, 1);
	} else if (!strncmp(buf, "health-ok", 9)) {
		chip->health_status = 0;
		power_supply_changed(chip->cms_psy);
	} else if (!strncmp(buf, "chg-cutoff", 10)) {
		cancel_delayed_work(&chip->debugfs_write_work);
		chip->debugfs_write_flag = DWF_CHG_CUTOFF;
		schedule_delayed_work(&chip->debugfs_write_work, 0);
	} else if (!strncmp(buf, "chg-open", 8)) {
		cancel_delayed_work(&chip->debugfs_write_work);
		chip->debugfs_write_flag = DWF_CHG_OPEN;
		schedule_delayed_work(&chip->debugfs_write_work, 0);
	} else if (!strncmp(buf, "sealed", 6)) {
		value.intval = 1;
		cms_print(chip, PR_INFO, "sealed\n");
		if (chip->ex_bms_psy) {
			power_supply_set_property(chip->ex_bms_psy,
			POWER_SUPPLY_PROP_AUTHENTIC, &value);
		} else {
			cms_print(chip, PR_INFO, "external bms psy not found\n");
		}
	} else if (!strncmp(buf, "unsealed", 8)) {
		value.intval = 0;
		cms_print(chip, PR_INFO, "unsealed\n");
		if (chip->ex_bms_psy) {
			power_supply_set_property(chip->ex_bms_psy,
			POWER_SUPPLY_PROP_AUTHENTIC, &value);
		} else {
			cms_print(chip, PR_INFO, "external bms psy not found\n");
		}
	} else if (!strncmp(buf, "no-bat-warning", 14)) {
		cms_print(chip, PR_INFO, "no-bat-warning\n");
		chip->no_bat_warning = 1;
		chip->battery_rc.bat_id = 1;
		chip->battery_rc.state = BAT_ID_IDLE;
		chip->battery_rc.counter = 2;
		chip->health_status = 0;
	} else if (strstr(buf, "LG4") != NULL || !strncmp(buf, "no-chg-timeout", 14)) {
		cms_print(chip, PR_INFO, "no-chg-timeout\n");
		chip->no_chg_timeout = true;
		if (chip->batt_psy) {
			value.intval = false;
			power_supply_set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_HW_CHG_TIMEOUT, &value);
		}
	} else if (!strncmp(buf, "fg-cutoff", 9)) {
		cancel_delayed_work(&chip->debugfs_write_work);
		chip->debugfs_write_flag = DWF_FG_CUTOFF;
		schedule_delayed_work__on_system_unbound_wq(&chip->debugfs_write_work, 0);
	} else if (!strncmp(buf, "fg-open", 7)) {
		cancel_delayed_work(&chip->debugfs_write_work);
		chip->debugfs_write_flag = DWF_FG_OPEN;
		schedule_delayed_work__on_system_unbound_wq(&chip->debugfs_write_work, 0);
	} else {
		ret = -EINVAL;
	}

	return ret;
}
static CLASS_ATTR_RW(debug);

/************************************************
 *	node:  power_type
 *	function:  get battery detection method
 ************************************************/
static ssize_t power_type_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct cms *chip = container_of(c, struct cms,
						cms_debug_class);


	return scnprintf(buf, PAGE_SIZE, "batdet:%d\n", chip->bat_det_method);
}
static CLASS_ATTR_RO(power_type);



static struct attribute *cms_debug_class_attrs[] = {
	&class_attr_debug.attr,
	&class_attr_power_type.attr,
	&class_attr_charge_test.attr,
	&class_attr_charge_test_param.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cms_debug_class);

static int cms_debug_class_init(struct cms *chip)
{
	int rc;

	chip->cms_debug_class.name = "cms_debug";
	chip->cms_debug_class.class_groups = cms_debug_class_groups;
	rc = class_register(&chip->cms_debug_class);
	if (rc < 0)
		cms_print(chip, PR_ERROR, "Failed to create cms_debug_class, rc=%d\n", rc);
	else
		cms_print(chip, PR_INFO, "Successfully create cms_debug_class!\n");

	return rc;
}
/****************************************************
 * 				cms debug class end					*
*****************************************************/



#if 1 //debugfs remove
/****************
      cms debug fs
*****************/
static int cms_debugfs_show(struct seq_file *s, void *unused)
{
	struct cms *chip = s->private;

	seq_printf(s, "present:%d, health:0x%02X\n",
		chip->enabled, (uint32_t)chip->health_status);
	return 0;
}

static int cms_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, cms_debugfs_show, inode->i_private);
}
static ssize_t cms_debugfs_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	int ret = count;
	char buf[32];
	struct seq_file *s = file->private_data;
	struct cms *chip = s->private;
	union power_supply_propval value = {0,};

	cms_ensure_psy_available(chip);
	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if(!strncmp(buf, "health-warm", 11)){
		cms_notify_changed(chip, HEALTH_STATUS_BAT_WARM, 1);
	}else if(!strncmp(buf, "health-cold", 11)){
		cms_notify_changed(chip, HEALTH_STATUS_BAT_COLD, 1);
	}else if(!strncmp(buf, "health-bat-ov", 13)){
		cms_notify_changed(chip, HEALTH_STATUS_BAT_OV, 1);
	}else if(!strncmp(buf, "health-chg-ov", 13)){
		cms_notify_changed(chip, HEALTH_STATUS_CHG_OV, 1);
	}else if(!strncmp(buf, "health-chg-tmout", 16)){
		cms_notify_changed(chip, HEALTH_STATUS_CHG_TIMEOUT, 1);
	}else if(!strncmp(buf, "health-missing", 14)){
		cms_notify_changed(chip, HEALTH_STATUS_BAT_MISSING, 1);
	}else if(!strncmp(buf, "health-invalid", 14)){
		cms_notify_changed(chip, HEALTH_STATUS_BAT_INVALID, 1);
	} else if (!strncmp(buf, "usbid_water", 11)) {
		cms_notify_changed(chip, HEALTH_STATUS_USBID_WATER, 1);
	} else if (!strncmp(buf, "usb_conn_heat", 13)) {
		cms_notify_changed(chip, HEALTH_STATUS_USB_CONN_HEAT, 1);
	} else if (!strncmp(buf, "chg_err", 7)) {
		cms_notify_changed(chip, HEALTH_STATUS_CHG_ERR, 1);
	}else if(!strncmp(buf, "health-ok", 9)){
		chip->health_status = 0;
		power_supply_changed(chip->cms_psy);
	}else if(!strncmp(buf, "chg-cutoff", 10)){
		cancel_delayed_work(&chip->debugfs_write_work);
		chip->debugfs_write_flag = DWF_CHG_CUTOFF;
		schedule_delayed_work__on_system_unbound_wq(&chip->debugfs_write_work, bsp_test_mode ? round_jiffies_relative(500) : 0);
	}else if(!strncmp(buf, "chg-open", 8)){
		cancel_delayed_work(&chip->debugfs_write_work);
		chip->debugfs_write_flag = DWF_CHG_OPEN;
		schedule_delayed_work__on_system_unbound_wq(&chip->debugfs_write_work, bsp_test_mode ? round_jiffies_relative(500) : 0);
	} else if (!strncmp(buf,"sealed",6)) {
		value.intval = 1;
		cms_print(chip,PR_INFO,"sealed\n");
		if(chip->ex_bms_psy) {
			power_supply_set_property(chip->ex_bms_psy,
			POWER_SUPPLY_PROP_AUTHENTIC, &value);
		} else {
			cms_print(chip,PR_INFO,"external bms psy not found\n");
		}
	} else if (!strncmp(buf,"unsealed",8)) {
		value.intval = 0;
		cms_print(chip,PR_INFO,"unsealed\n");
		if(chip->ex_bms_psy) {
			power_supply_set_property(chip->ex_bms_psy,
			POWER_SUPPLY_PROP_AUTHENTIC, &value);
		} else {
			cms_print(chip,PR_INFO,"external bms psy not found\n");
		}
	}else if(!strncmp(buf, "no-bat-warning", 14)){
		cms_print(chip,PR_INFO,"no-bat-warning\n");
		chip->no_bat_warning = 1;
		chip->battery_rc.bat_id = 1;
		chip->battery_rc.state = BAT_ID_IDLE;
		chip->battery_rc.counter = 2;
		chip->health_status = 0;
	}else if(strstr(buf, "LG4") != NULL || !strncmp(buf, "no-chg-timeout", 14)){
		cms_print(chip,PR_INFO,"no-chg-timeout\n");
		chip->no_chg_timeout = true;
		value.intval = false;
		power_supply_set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_HW_CHG_TIMEOUT, &value);
	} else if (!strncmp(buf, "fg-cutoff", 9)) {
		cancel_delayed_work(&chip->debugfs_write_work);
		chip->debugfs_write_flag = DWF_FG_CUTOFF;
		schedule_delayed_work__on_system_unbound_wq(&chip->debugfs_write_work, 0);
	} else if (!strncmp(buf, "fg-open", 7)) {
		cancel_delayed_work(&chip->debugfs_write_work);
		chip->debugfs_write_flag = DWF_FG_OPEN;
		schedule_delayed_work__on_system_unbound_wq(&chip->debugfs_write_work, 0);
	}  else {
		ret = -EINVAL;
	}

	return ret;
}

const struct file_operations cms_debug_fops = {
	.open = cms_debugfs_open,
	.read = seq_read,
	.write = cms_debugfs_write,
	.llseek = seq_lseek,
	.release = single_release,
};
/* add power type debug fs*/
static int cms_power_type_fs_show(struct seq_file *s, void *unused)
{
	struct cms *chip = s->private;

	seq_printf(s, "batdet:%d\n", chip->bat_det_method);

	return 0;
}

static int cms_power_type_fs_open(struct inode *inode, struct file *file)
{
	return single_open(file, cms_power_type_fs_show, inode->i_private);
}

const struct file_operations power_type_fs_debug_fops = {
	.open = cms_power_type_fs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static const char * const charge_test_item_title[] = {
	"cableR",
	"M_ibus",
	"S_ibus",
	"BAT_NTC",
	"M_BOARD_NTC",
	"SUB_BOARD_NTC",
	"USB_CON_NTC",
	"MAIN_BAT_CON_NTC",
	"SUB_BAT_CON_NTC",
	"BAT_BOARD_NTC",
	"Unkown test item",
};
enum charge_test_item {
	cableR,
	M_ibus,
	S_ibus,
	BAT_NTC,
	M_BOARD_NTC,
	SUB_BOARD_NTC,
	USB_CON_NTC,
	MAIN_BAT_CON_NTC,
	SUB_BAT_CON_NTC,
	BAT_BOARD_NTC,
	Unkown_test_item
};
static void cms_check_chg_error_state(struct cms *chip)
{
	int pos = 0;
	union power_supply_propval supported_adapter = {0,};
	union power_supply_propval master_ibus = {0,};
	union power_supply_propval slave_ibus = {0,};
	union power_supply_propval calbe_r = {0,};
	union power_supply_propval main_board_temp_now = {0,};
	union power_supply_propval sub_board_temp_now = {0,};
	int health_state = 0;
	int index = 0;
	int value = 0;
	int len = 0;

	if(!chip->charge_test_param_rc || !chip->charge_test_param_data){
		cms_print(chip,PR_ERROR,"charge test param data load fail!\n");
		return;
	}
	len = chip->charge_test_param_rc[ROW];

	cms_ensure_psy_available(chip);
	if(!chip->batt_psy) {
		cms_print(chip,PR_ERROR,"bat_psy or mcu_psy not registed\n");
		return;
	}
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_SUPPORTED_ADAPTER, &supported_adapter);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PRIMARY_BOARD_TEMP, &main_board_temp_now);
	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_PARALLEL_BOARD_TEMP, &sub_board_temp_now);
	chip->main_board_temp = main_board_temp_now.intval;
	chip->sub_board_temp = sub_board_temp_now.intval;

	if(chip->direct_charger_enable && chip->mcu_psy) {
		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_HALF_CHG_MASTER_IBUS, &master_ibus);
		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_HALF_CHG_SLAVE_IBUS, &slave_ibus);
		power_supply_get_property(chip->mcu_psy,
			POWER_SUPPLY_PROP_DCHG_CABLER, &calbe_r);
	}

	memset(chip->chg_state, 0, sizeof(chip->chg_state));
	for(index = 0; index < len; index++) {
			if(index>=0 && index < Unkown_test_item) {
				 if(chip->charge_test_param_data[index].enable_switch) {
					switch(index) {
						case cableR:
							value = calbe_r.intval; /* cable resistance */
							break;
						case M_ibus:
							value = master_ibus.intval - slave_ibus.intval; /* master ibus */
							break;
						case S_ibus:
							value = slave_ibus.intval; /* slave ibus */
							break;
						case BAT_NTC:
							value = chip->battery_temp / 10;
							break;
						case M_BOARD_NTC:
							value = chip->main_board_temp / 10;
							break;
						case SUB_BOARD_NTC:
							value = chip->sub_board_temp / 10;
							break;
						case USB_CON_NTC:
							value = chip->usb_connector_temp / 10;
							break;
						case MAIN_BAT_CON_NTC:
							value = chip->main_bat_connector_temp / 10;
							break;
						case SUB_BAT_CON_NTC:
							value = chip->sub_bat_connector_temp / 10;
							break;
						case BAT_BOARD_NTC:
							value = chip->battery_board_temp / 10;
							break;

						default:
							value = 0;
					}
					pos += scnprintf(chip->chg_state + pos, sizeof(chip->chg_state) -pos, "%s:%d[%d, %d],", charge_test_item_title[index], value, chip->charge_test_param_data[index].min_value, chip->charge_test_param_data[index].max_value);
				}
				else {
					pos += scnprintf(chip->chg_state + pos, sizeof(chip->chg_state) -pos, "%s:%s,", charge_test_item_title[index], "NULL");
				}
			}
			else
			{
					pos += scnprintf(chip->chg_state + pos, sizeof(chip->chg_state) -pos, "%s:%s,", "Unkown test item", "NULL");
			}
	}

	health_state = cms_map_health_status(chip);
	if(health_state == 0x0008) /* HEALTH_STATUS_BAT_OV */
		pos += scnprintf(chip->chg_state + pos, sizeof(chip->chg_state) -pos, "battery over voltage|");
	if(health_state == 0x0100) /* HEALTH_STATUS_BAT_INVALID */
		pos += scnprintf(chip->chg_state + pos, sizeof(chip->chg_state) -pos, "battery detect fail|");
	if(health_state == 0x0040) /* HEALTH_STATUS_BAT_MISSING */
		pos += scnprintf(chip->chg_state + pos, sizeof(chip->chg_state) -pos, "battery detect fail|");
	if(health_state == 0x1000) /* HEALTH_STATUS_BAT_UNPLUGED */
		pos += scnprintf(chip->chg_state + pos, sizeof(chip->chg_state) -pos, "battery detect fail");

	cms_print(chip, PR_INFO, "supported_adapter=%d,chg_state=%s.\n", supported_adapter, chip->chg_state);
}


static int cms_charge_test_fs_show(struct seq_file *s, void *unused)
{
	struct cms *chip = s->private;
	cms_check_chg_error_state(chip);
	seq_printf(s, "%s\n", chip->chg_state);

	return 0;
}
static int cms_charge_test_fs_open(struct inode *inode, struct file *file)
{
	return single_open(file, cms_charge_test_fs_show, inode->i_private);
}
static ssize_t cms_charge_test_fs_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	int ret = count;
	char buf[32];
	struct seq_file *s = file->private_data;
	struct cms *chip = s->private;
	union power_supply_propval value = {0,};
	union power_supply_propval supported_adapter = {0,};

	memset(buf, 0x00, sizeof(buf));
	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	cms_ensure_psy_available(chip);
	if(!chip->batt_psy || !chip->mcu_psy) {
		cms_print(chip,PR_ERROR,"bat_psy or mcu_psy not registed\n");
		return ret;
	}

	power_supply_get_property(chip->batt_psy,
		POWER_SUPPLY_PROP_SUPPORTED_ADAPTER, &supported_adapter);

	if(supported_adapter.intval == 2 || supported_adapter.intval == 4 || supported_adapter.intval == 5) { /* add for half charge test */
		if(!strncmp(buf, "disable-fast-charge", 19)) {
			value.intval = 1;
			power_supply_set_property(chip->mcu_psy,
				POWER_SUPPLY_PROP_FACTORY_10W_CHARGE_TEST, &value);
			msleep(500);	//wait MCU prepare action.
		}
	} else if(supported_adapter.intval == 3) {
		/* reserve for 9v/2A test */
	}
	return ret;
}

const struct file_operations charge_test_fs_debug_fops = {
	.open = cms_charge_test_fs_open,
	.read = seq_read,
	.write = cms_charge_test_fs_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int cms_charge_test_param_fs_show(struct seq_file *s, void *unused)
{
	struct cms *chip = s->private;

	seq_printf(s, "charge test param threshold:%d,%d,%d,%d,%d,%d,%d,%d\n", chip->charge_param_judge_threshold[0], chip->charge_param_judge_threshold[1], chip->charge_param_judge_threshold[2],
	chip->charge_param_judge_threshold[3], chip->charge_param_judge_threshold[4], chip->charge_param_judge_threshold[5], chip->charge_param_judge_threshold[6], chip->charge_param_judge_threshold[7]);

	return 0;
}

static int cms_charge_test_param_fs_open(struct inode *inode, struct file *file)
{
	return single_open(file, cms_charge_test_param_fs_show, inode->i_private);
}

const struct file_operations charge_test_param_fs_debug_fops = {
	.open = cms_charge_test_param_fs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#if 1 //debugfs remove
static void cms_create_debugfs_entries(struct cms *chip)
{
	chip->dent = debugfs_create_dir("cms", NULL);
	if(!chip->dent){
		cms_print(chip,PR_ERROR,"failed to create debugfs\n");
		return;
	}

	if (!debugfs_create_file("debug", S_IRUGO | S_IWUSR,
				chip->dent, chip, &cms_debug_fops))
		goto error;

	if (!debugfs_create_file("power_type", S_IRUGO,
			chip->dent, chip, &power_type_fs_debug_fops))
		goto error;

	if (!debugfs_create_file("charge_test", S_IRUGO | S_IWUSR,
			chip->dent, chip, &charge_test_fs_debug_fops))
		goto error;

	if (!debugfs_create_file("charge_test_param", S_IRUGO,
			chip->dent, chip, &charge_test_param_fs_debug_fops))
		goto error;

	return;
error:
	debugfs_remove_recursive(chip->dent);
	chip->dent = 0;
}
#endif

static void cms_debugfs_write_work(struct work_struct *work)
{
	struct cms *chip = container_of(work, struct cms, debugfs_write_work.work);
	union power_supply_propval propval = {0,};

	cms_print(chip, PR_INFO, "debugfs_write_flag:%d\n", chip->debugfs_write_flag);

	switch (chip->debugfs_write_flag) {
	case DWF_CHG_CUTOFF:
		propval.intval = 0;
		cms_print(chip, PR_INFO, "chg-cutoff\n");
		if (chip->batt_psy) {
			power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &propval);
			if (bsp_test_mode) {
				chip->usbin_cutoff_status = true;
			}
		} else {
			cms_print(chip, PR_ERROR, "Battery psy not found\n");
		}
		vote(chip->user_ibat_votable, DEBUGFS_EVENT, true, 0);
		break;
	case DWF_CHG_OPEN:
		propval.intval = 1;
		cms_print(chip, PR_INFO, "chg-open\n");
		if (chip->batt_psy) {
			power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &propval);
			if (bsp_test_mode) {
				chip->usbin_cutoff_status = false;
			}
		} else {
			cms_print(chip, PR_ERROR, "Battery psy not found\n");
		}

		vote(chip->user_ibat_votable, DEBUGFS_EVENT, false, 0);
		break;
	case DWF_FG_CUTOFF:
		propval.intval = 0;
		cms_print(chip, PR_INFO, "fg-cutoff\n");
		if (chip->bms_psy) {
			power_supply_set_property(chip->bms_psy,
			POWER_SUPPLY_PROP_FG_MONITOR_WORK, &propval);
		} else {
			cms_print(chip, PR_ERROR, "fg psy not found\n");
		}
		break;
	case DWF_FG_OPEN:
		propval.intval = 1;
		cms_print(chip, PR_INFO, "fg-open\n");
		if (chip->bms_psy) {
			power_supply_set_property(chip->bms_psy,
			POWER_SUPPLY_PROP_FG_MONITOR_WORK, &propval);
		} else {
			cms_print(chip, PR_ERROR, "fg psy not found\n");
		}
		break;
	default:
		cms_print(chip, PR_INFO, "debugfs_write_flag default!\n");
		break;
	}

	chip->debugfs_write_flag = DWF_CHG_UNKNOWN;
}
/****************
    cms device tree
*****************/
static void *cms_of_get_property_data(struct cms *chip, struct device_node *node, const char * prop_name,
					int rows,int cols,int type,char *tip)
{
	//struct device_node *node = chip->dev->of_node;
	struct tc_data_item  *ori_data;
	struct tc_data_item_ex  *ori_data_ex;
	struct usb_conn_heat_action_item *ori_usb_conn_heat_action;
	struct property *prop;
	const __be32 *data;
	int *prop_data = NULL;
	char *default_tip = "default";
	int size, i;

	if(!tip){
		tip = default_tip;
	}

	if(rows*cols) {
		prop = of_find_property(node, prop_name, NULL);
		if (!prop) {
			cms_print(chip,PR_ERROR,"prop %s is not found\n", prop_name);
			goto out;
		}
		data = prop->value;
		size = prop->length/sizeof(int);
		if (size != cols * rows) {
			cms_print(chip,PR_ERROR,"%s: %s data size mismatch, %dx%d != %d\n",
					node->name, prop_name, cols, rows, size);
			goto out;
		}

		prop_data = kzalloc(sizeof(int)*size, GFP_KERNEL);
		if (!prop_data) {
			cms_print(chip,PR_ERROR,"%s kzalloc failed %d\n", __func__, __LINE__);
			goto out;
		}

		if(type == TWO_DIMENSION){
			ori_data = (struct tc_data_item  *)prop_data;
			for (i = 0; i < rows; i++) {
				ori_data[i].min = be32_to_cpup(data++);
				ori_data[i].max = be32_to_cpup(data++);
				ori_data[i].data = be32_to_cpup(data++);
				cms_print(chip,PR_INFO,"%s :min = %d , max = %d , data = %d\n", tip, ori_data[i].min, \
					ori_data[i].max, ori_data[i].data);
			}
		}else if(type == THREE_DIMENSION){
			ori_data_ex = (struct tc_data_item_ex  *)prop_data;
			for (i = 0; i < rows; i++) {
				ori_data_ex[i].min = be32_to_cpup(data++);
				ori_data_ex[i].max = be32_to_cpup(data++);
				ori_data_ex[i].data = be32_to_cpup(data++);
				ori_data_ex[i].volt1 = be32_to_cpup(data++);
				ori_data_ex[i].data_ex1 = be32_to_cpup(data++);
				ori_data_ex[i].volt2 = be32_to_cpup(data++);
				ori_data_ex[i].data_ex2 = be32_to_cpup(data++);
				cms_print(chip,PR_INFO,"%s :min = %d , max = %d , data = %d , volt1 = %d , data_ex1 = %d, volt2 = %d , data_ex2 = %d\n", tip, ori_data_ex[i].min, \
					ori_data_ex[i].max, ori_data_ex[i].data,ori_data_ex[i].volt1,ori_data_ex[i].data_ex1,ori_data_ex[i].volt2,ori_data_ex[i].data_ex2);
			}
		}else if(type == FOUR_DIMENSION){
			ori_usb_conn_heat_action = (struct usb_conn_heat_action_item  *)prop_data;
			for (i = 0; i < rows; i++) {
				ori_usb_conn_heat_action[i].Tmin = be32_to_cpup(data++);
				ori_usb_conn_heat_action[i].Tmax = be32_to_cpup(data++);
				ori_usb_conn_heat_action[i].trigger_data = be32_to_cpup(data++);
				ori_usb_conn_heat_action[i].release_data = be32_to_cpup(data++);
				cms_print(chip,PR_INFO,"%s :Tmin = %d , Tmax = %d , trigger_data = %d , release_data = %d .\n", tip, ori_usb_conn_heat_action[i].Tmin, \
					ori_usb_conn_heat_action[i].Tmax, ori_usb_conn_heat_action[i].trigger_data, ori_usb_conn_heat_action[i].release_data);
			}
		}else{
			for (i = 0; i < cols; i++) {
				prop_data[i] = be32_to_cpup(data++);
				cms_print(chip,PR_INFO,"%s :data = %d\n", tip, prop_data[i]);
			}
		}
	}else{
		cms_print(chip,PR_ERROR,"%s :rows or lows is error\n", tip);
	}

out:
	return prop_data;
}

extern char *get_bbk_board_version(void);
struct device_node *cms_find_support_board_node(struct cms *chip)
{
	struct device_node *node, *match_node = NULL;
	const char *support_board_version = NULL;
	char *board_version = NULL;
	int rc = 0;
	int i = 0;
	int bit_count = 0;
	int max_bit_count = 0;

	board_version= get_bbk_board_version();
	for_each_child_of_node(chip->dev->of_node, node) {
		rc = of_property_read_string(node, "support-board-version",
								&support_board_version);
		if (!rc) {
			if (!strncmp(support_board_version, "xxxxxxx", 7) || !strncmp(support_board_version, "XXXXXXX", 7)) {	//default board
				if (match_node == NULL) {
					match_node = node;
					cms_print(chip, PR_ERROR, "node=%s: support_board_version=%s, bbk_board_version=%s, match...\n", node->name, support_board_version, board_version);
				}
				continue;
			}

			bit_count = 0;			
			for (i = 0; i < 7; i++) {	//7bits bbk_board_version
				if (support_board_version[i] != 'x' && support_board_version[i] != 'X') {
					if (support_board_version[i] != board_version[i]) {
						bit_count = 0;
						break;
					} else
						bit_count++;
				}
			}
			if (bit_count > max_bit_count) {
				max_bit_count = bit_count;
				match_node = node;
				cms_print(chip, PR_ERROR, "node=%s: support_board_version=%s, bbk_board_version=%s, match...\n", node->name, support_board_version, board_version);
			}
		}
	}

	if (match_node == NULL)
		cms_print(chip, PR_ERROR, "Can not match the child_node in cms_v2..\n");

	return match_node;
}

static int cms_parse_dt(struct cms *chip)
{
	struct device_node *parent_node = chip->dev->of_node;
	struct device_node *node = NULL;
	int rc;

	if (!parent_node) {
		cms_print(chip,PR_ERROR,"device tree info. missing\n");
		return -EINVAL;
	}

	node = cms_find_support_board_node(chip);
	if (!node) {
		node = parent_node;
	}

	rc = of_property_read_u32(node, "vivo,bat-det-method",&chip->bat_det_method);
	if (rc) {
		chip->bat_det_method = BAT_DET_BY_R;
		cms_print(chip,PR_WARN,"bat-det-method not defined, default by R used\n");
	}

	if (chip->bat_det_method == BAT_DET_BY_R ||
		chip->bat_det_method == BAT_DET_BY_RC) {
		chip->bat_id_gpio = of_get_named_gpio(node, "vivo,bat-id-gpio", 0);
		if (chip->bat_id_gpio <= 0) {
			cms_print(chip,PR_ERROR,"bat-id-gpio not defined\n");
			return -EINVAL;
		}
		chip->use_new_rc_param = of_property_read_bool(node, "vivo,use-new-rc-param");
		if(chip->use_new_rc_param){
			cms_print(chip,PR_WARN,"use new rc param\n");
		}
	}

	rc = of_property_read_string(node, "vivo,ex-bms-psy-name",
						&chip->ex_bms_psy_name);
	if (rc)
		chip->ex_bms_psy_name = "bms";

	rc = of_property_read_u32(node, "vivo,battery-plugable",
						&chip->batt_plugable);
	if (rc)
		chip->batt_plugable = BAT_PLUGABLE_DEFAULT;

	chip->no_bat_warning = of_property_read_bool(node, "vivo,no-bat-warning");
	chip->no_bat_id_keep_charging = of_property_read_bool(node, "vivo,no-bat-id-keep-charging");
	chip->no_chg_timeout = of_property_read_bool(node, "vivo,no-chg-timeout");

	rc = of_property_read_u32(node, "vivo,chg-timeout-mins",
							&chip->chg_timeout_mins);
	if (rc)
		chip->chg_timeout_mins = CHG_TIMEOUT_MINS_DEFAULT;

	chip->htccc_enable = of_property_read_bool(node, "vivo,htccc-enable");
	//fuelsummary_of_property_put("vivo,htccc-enable", PARAMS_TYPE_BOOL, &(chip->htccc_enable));
	if(chip->htccc_enable) {
		chip->htccc_data = (int*)cms_of_get_property_data(chip, node,
				"vivo,htccc-data", 1, 4, ONE_DIMENSION, NULL);
		if(!chip->htccc_data){
			chip->htccc_enable = false;
		}
	}
	//fuelsummary_of_property_puts("vivo,htccc-data", STATIC_FIXED, PARAMS_TYPE_INT, chip->htccc_data, 1, 4);

	chip->fbon_scale_enable = of_property_read_bool(node, "vivo,fbon-scale-enable");
	//fuelsummary_of_property_put("vivo,fbon-scale-enable", PARAMS_TYPE_BOOL, &(chip->fbon_scale_enable));
	if(chip->fbon_scale_enable) {
		chip->fbon_scale_data = (int*)cms_of_get_property_data(chip, node,
				"vivo,fbon-scale-data", 1, 3, ONE_DIMENSION, NULL);
		if(!chip->fbon_scale_data){
			chip->fbon_scale_enable = false;
		}
	}
	//fuelsummary_of_property_puts("vivo,fbon-scale-data", STATIC_FIXED, PARAMS_TYPE_INT, chip->fbon_scale_data, 1, 3);

	chip->ai_charge_enable = of_property_read_bool(node, "vivo,ai-charge-enable");

	chip->calling_scale_enable = of_property_read_bool(node, "vivo,calling-scale-enable");
	if(chip->calling_scale_enable) {
		chip->calling_scale_data = (int*)cms_of_get_property_data(chip, node,
				"vivo,calling-scale-data", 1, 2, ONE_DIMENSION, NULL);
		if(!chip->calling_scale_data){
			chip->calling_scale_enable = false;
		}
		chip->calling_dchg_disable = of_property_read_bool(node, "vivo,calling-dchg-disable");
	}
	
	chip->weixin_calling_scale_enable = of_property_read_bool(node, "vivo,weixin-calling-scale-enable");
	if(chip->weixin_calling_scale_enable) {
		chip->weixin_calling_state = false;
		chip->weixin_calling_scale_data = (int*)cms_of_get_property_data(chip, node,
				"vivo,weixin-calling-scale-data", 1, 2, ONE_DIMENSION, "weixin-calling-scale-data");
		if(!chip->weixin_calling_scale_data){
			chip->weixin_calling_scale_enable = false;
		}
	}
	
	chip->fixed_scale_enable = of_property_read_bool(node, "vivo,fixed-scale-enable");
	if(chip->fixed_scale_enable) {
		chip->fixed_scale_data = (int*)cms_of_get_property_data(chip, node,
				"vivo,fixed-scale-data", 1, 3, ONE_DIMENSION, NULL);
		if(!chip->fixed_scale_data){
			chip->fixed_scale_enable = false;
		}
	}

	rc = of_property_read_u32(node, "vivo,vbat-r-comp", &chip->vbat_r_comp);
	if (rc)
		chip->vbat_r_comp = 0;
	
	rc = of_property_read_u32(node, "vivo,exhibition-mode-current-primary", &chip->exhibition_mode_current_primary);
	if (rc)
		chip->exhibition_mode_current_primary = 2000;
	
	rc = of_property_read_u32(node, "vivo,exhibition-mode-current-parallel", &chip->exhibition_mode_current_parallel);
	if (rc)
		chip->exhibition_mode_current_parallel = 0;
	
	cms_print(chip, PR_INFO, "exhibition mode pri = %d, pl = %d\n", chip->exhibition_mode_current_primary, chip->exhibition_mode_current_parallel);

	/* normal charge battery param */
	chip->normal_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,normal-tc-row-col", 1, 2, ONE_DIMENSION, NULL);
	if(chip->normal_tc_rc) {
		chip->normal_tc_data = (struct tc_data_item_ex *)cms_of_get_property_data(chip, node,
					"vivo,normal-tc-data",chip->normal_tc_rc[ROW],
					chip->normal_tc_rc[COL], THREE_DIMENSION, "normal_tc_data");
	}

	if(!chip->normal_tc_rc || !chip->normal_tc_data){
		chip->normal_tc_data = default_normal_tc_data;
		chip->normal_tc_rc[ROW] = ARRAY_SIZE(default_normal_tc_data);
		chip->normal_tc_rc[COL] = 7;
	} else {
		
	}

	/* intell-charge battery param */
	chip->intell_charge_enable = of_property_read_bool(node, "vivo,intell-charge-enable");
	if(chip->intell_charge_enable) {
		chip->parallel_temp_enable = (int*)cms_of_get_property_data(chip, node,
				"vivo,parallel-temp-enable", 1, 2, ONE_DIMENSION, "parallel-temp-enable");
		if(!chip->parallel_temp_enable){
			chip->parallel_temp_enable = default_parallel_temp_enable;
		}
		//fuelsummary_of_property_puts("vivo,parallel-temp-enable", STATIC_FIXED, PARAMS_TYPE_INT, chip->parallel_temp_enable, 1, 2);

		/* fbon battery param */
		chip->primary_fbon_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,primary-fbon-tc-row-col", 1, 2, ONE_DIMENSION, "primary-fbon-tc-row-col");
		if(chip->primary_fbon_tc_rc) {
			chip->primary_fbon_tc_data = (struct tc_data_item *)cms_of_get_property_data(chip, node,
						"vivo,primary-fbon-tc-data",chip->primary_fbon_tc_rc[ROW],
						chip->primary_fbon_tc_rc[COL], TWO_DIMENSION, "primary_fbon_tc_data");
		}
		if(!chip->primary_fbon_tc_rc || !chip->primary_fbon_tc_data){
			chip->primary_fbon_tc_data = default_primary_fbon_tc_data;
			chip->primary_fbon_tc_rc[ROW] = ARRAY_SIZE(default_primary_fbon_tc_data);
			chip->primary_fbon_tc_rc[COL] = 3;
		} else {

		}

		chip->parallel_fbon_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,parallel-fbon-tc-row-col", 1, 2, ONE_DIMENSION, NULL);
		if(chip->parallel_fbon_tc_rc) {
			chip->parallel_fbon_tc_data = (struct tc_data_item *)cms_of_get_property_data(chip, node,
						"vivo,parallel-fbon-tc-data",chip->parallel_fbon_tc_rc[ROW],
						chip->parallel_fbon_tc_rc[COL], TWO_DIMENSION, "parallel_fbon_tc_data");
		}
		if(!chip->parallel_fbon_tc_rc || !chip->parallel_fbon_tc_data){
			chip->parallel_fbon_tc_data = default_parallel_fbon_tc_data;
			chip->parallel_fbon_tc_rc[ROW] = ARRAY_SIZE(default_parallel_fbon_tc_data);
			chip->parallel_fbon_tc_rc[COL] = 3;

		} else {

		}

		/* fboff battery param */
		chip->primary_fboff_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,primary-fboff-tc-row-col", 1, 2, ONE_DIMENSION, "primary-fboff-tc-row-col");
		if(chip->primary_fboff_tc_rc) {
			chip->primary_fboff_tc_data = (struct tc_data_item *)cms_of_get_property_data(chip, node,
						"vivo,primary-fboff-tc-data",chip->primary_fboff_tc_rc[ROW],
						chip->primary_fboff_tc_rc[COL], TWO_DIMENSION, "primary_fboff_tc_data");
		}
		if(!chip->primary_fboff_tc_rc || !chip->primary_fboff_tc_data){
			chip->primary_fboff_tc_data = default_primary_fboff_tc_data;
			chip->primary_fboff_tc_rc[ROW] = ARRAY_SIZE(default_primary_fboff_tc_data);
			chip->primary_fboff_tc_rc[COL] = 3;

		} else {

		}

		chip->parallel_fboff_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,parallel-fboff-tc-row-col", 1, 2, ONE_DIMENSION, "parallel-fboff-tc-row-col");
		if(chip->parallel_fboff_tc_rc) {
			chip->parallel_fboff_tc_data = (struct tc_data_item *)cms_of_get_property_data(chip, node,
						"vivo,parallel-fboff-tc-data",chip->parallel_fboff_tc_rc[ROW],
						chip->parallel_fboff_tc_rc[COL], TWO_DIMENSION, "parallel_fboff_tc_data");
		}
		if(!chip->parallel_fboff_tc_rc || !chip->parallel_fboff_tc_data){
			chip->parallel_fboff_tc_data = default_parallel_fboff_tc_data;
			chip->parallel_fboff_tc_rc[ROW] = ARRAY_SIZE(default_parallel_fboff_tc_data);
			chip->parallel_fboff_tc_rc[COL] = 3;

		} else {

		}
	}

	/* 5v-intell-charge battery param */
	chip->ex_intell_charge_enable = of_property_read_bool(node, "vivo,ex-intell-charge-enable");
	if (chip->ex_intell_charge_enable) {
		chip->ex_parallel_temp_enable = (int*)cms_of_get_property_data(chip, node,
				"vivo,ex-parallel-temp-enable", 1, 2, ONE_DIMENSION, "ex-parallel-temp-enable");
		if (!chip->ex_parallel_temp_enable) {
			chip->ex_parallel_temp_enable = default_parallel_temp_enable;
		}

		/* fbon battery param */
		chip->ex_primary_fbon_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,ex-primary-fbon-tc-row-col", 1, 2, ONE_DIMENSION, "ex-primary-fbon-tc-row-col");
		if (chip->ex_primary_fbon_tc_rc) {
			chip->ex_primary_fbon_tc_data = (struct tc_data_item *)cms_of_get_property_data(chip, node,
						"vivo,ex-primary-fbon-tc-data", chip->ex_primary_fbon_tc_rc[ROW],
						chip->ex_primary_fbon_tc_rc[COL], TWO_DIMENSION, "ex-primary_fbon_tc_data");
		}
		if (!chip->ex_primary_fbon_tc_rc || !chip->ex_primary_fbon_tc_data) {
			chip->ex_primary_fbon_tc_data = default_primary_fbon_tc_data;
			chip->ex_primary_fbon_tc_rc[ROW] = ARRAY_SIZE(default_primary_fbon_tc_data);
			chip->ex_primary_fbon_tc_rc[COL] = 3;
		}

		chip->ex_parallel_fbon_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,ex-parallel-fbon-tc-row-col", 1, 2, ONE_DIMENSION, NULL);
		if (chip->ex_parallel_fbon_tc_rc) {
			chip->ex_parallel_fbon_tc_data = (struct tc_data_item *)cms_of_get_property_data(chip, node,
						"vivo,ex-parallel-fbon-tc-data", chip->ex_parallel_fbon_tc_rc[ROW],
						chip->ex_parallel_fbon_tc_rc[COL], TWO_DIMENSION, "ex-parallel_fbon_tc_data");
		}
		if (!chip->ex_parallel_fbon_tc_rc || !chip->ex_parallel_fbon_tc_data) {
			chip->ex_parallel_fbon_tc_data = default_parallel_fbon_tc_data;
			chip->ex_parallel_fbon_tc_rc[ROW] = ARRAY_SIZE(default_parallel_fbon_tc_data);
			chip->ex_parallel_fbon_tc_rc[COL] = 3;
		}

		/* fboff battery param */
		chip->ex_primary_fboff_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,ex-primary-fboff-tc-row-col", 1, 2, ONE_DIMENSION, "ex-primary-fboff-tc-row-col");
		if (chip->ex_primary_fboff_tc_rc) {
			chip->ex_primary_fboff_tc_data = (struct tc_data_item *)cms_of_get_property_data(chip, node,
						"vivo,ex-primary-fboff-tc-data", chip->ex_primary_fboff_tc_rc[ROW],
						chip->ex_primary_fboff_tc_rc[COL], TWO_DIMENSION, "ex-primary_fboff_tc_data");
		}
		if (!chip->ex_primary_fboff_tc_rc || !chip->ex_primary_fboff_tc_data) {
			chip->ex_primary_fboff_tc_data = default_primary_fboff_tc_data;
			chip->ex_primary_fboff_tc_rc[ROW] = ARRAY_SIZE(default_primary_fboff_tc_data);
			chip->ex_primary_fboff_tc_rc[COL] = 3;
		}

		chip->ex_parallel_fboff_tc_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,ex-parallel-fboff-tc-row-col", 1, 2, ONE_DIMENSION, "ex-parallel-fboff-tc-row-col");
		if (chip->ex_parallel_fboff_tc_rc) {
			chip->ex_parallel_fboff_tc_data = (struct tc_data_item *)cms_of_get_property_data(chip, node,
						"vivo,ex-parallel-fboff-tc-data", chip->ex_parallel_fboff_tc_rc[ROW],
						chip->ex_parallel_fboff_tc_rc[COL], TWO_DIMENSION, "ex-parallel_fboff_tc_data");
		}
		if (!chip->ex_parallel_fboff_tc_rc || !chip->ex_parallel_fboff_tc_data) {
			chip->ex_parallel_fboff_tc_data = default_parallel_fboff_tc_data;
			chip->ex_parallel_fboff_tc_rc[ROW] = ARRAY_SIZE(default_parallel_fboff_tc_data);
			chip->ex_parallel_fboff_tc_rc[COL] = 3;
		}
	}

	/* direct charging param */
	chip->direct_charger_enable = of_property_read_bool(node, "vivo,direct-charger-enable");
	if (chip->direct_charger_enable) {

		chip->dchg_ntc_enable = of_property_read_bool(node, "vivo,dchg-ntc-protect-enable");
		//fuelsummary_of_property_put("vivo,dchg-ntc-protect-enable", PARAMS_TYPE_BOOL, &(chip->dchg_ntc_enable));
		if (chip->dchg_ntc_enable) {
			chip->dchg_ntc_data = (int*)cms_of_get_property_data(chip, node,
				"vivo,dchg-ntc-protect-data", 1, 4, ONE_DIMENSION, "dchg-ntc-protect-data");
			if(!chip->dchg_ntc_data){
				chip->dchg_ntc_data = default_dchg_ntc_protect_data;
			}
			//fuelsummary_of_property_puts("vivo,dchg-ntc-protect-data", STATIC_FIXED, PARAMS_TYPE_INT, chip->dchg_ntc_data, 1, 4);
		}
	}

	/* usb connecter protect param */
	rc = of_property_read_u32(node, "vivo,usb-connecter-protect-enable", &chip->usb_connecter_protect_enable);
	//fuelsummary_of_property_put("vivo,usb-connecter-protect-enable", PARAMS_TYPE_BOOL, &(chip->usb_connecter_protect_enable));
	if (rc) {
		chip->usb_connecter_protect_enable = USB_PROTECT_NONE;
		cms_print(chip,PR_WARN,"vivo,usb-connecter-protect-enable not defined\n");
	}
	if (chip->usb_connecter_protect_enable) {
		cms_print(chip,PR_WARN,"usb_connecter_protect_enable = 0x%x.\n", chip->usb_connecter_protect_enable);
		/* usb_id water protect */
		chip->usb_id_protect_data = (int *)cms_of_get_property_data(chip, node,
					"vivo,usb-id-protect-data", 1, 3, ONE_DIMENSION, "usb-id-protect-data");
		if (!chip->usb_id_protect_data) {
			chip->usb_id_protect_data = default_usb_id_protect_data;
		}
		//fuelsummary_of_property_puts("vivo,usb-id-protect-data", STATIC_FIXED, PARAMS_TYPE_INT, chip->usb_id_protect_data, 1, 3);

		/* usb_conn_temp heat protect */
		chip->usb_conn_heat_protect_data_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,usb-conn-heat-protect-data-row-col", 1, 2, ONE_DIMENSION, NULL);
		if (chip->usb_conn_heat_protect_data_rc) {
			chip->usb_conn_heat_protect_data = (struct usb_conn_heat_action_item *)cms_of_get_property_data(chip, node,
					"vivo,usb-conn-heat-protect-data", chip->usb_conn_heat_protect_data_rc[ROW],
					chip->usb_conn_heat_protect_data_rc[COL], FOUR_DIMENSION, "usb-conn-heat-protect-data");
		}

		if (!chip->usb_conn_heat_protect_data_rc || !chip->usb_conn_heat_protect_data) {
			chip->usb_conn_heat_protect_data = default_usb_conn_temp_heat_data;
			chip->usb_conn_heat_protect_data_rc[ROW] = ARRAY_SIZE(default_usb_conn_temp_heat_data);
			chip->usb_conn_heat_protect_data_rc[COL] = 4;
		}

#if 0
		for (rc = 0; rc < chip->usb_conn_heat_protect_data_rc[ROW]; rc++) {
			cms_print(chip,PR_INFO,"usb-conn-heat-protect-data :Tmin = %d, Tmax = %d, trigger = %d, release = %d.\n",
				chip->usb_conn_heat_protect_data[rc].Tmin, chip->usb_conn_heat_protect_data[rc].Tmax, chip->usb_conn_heat_protect_data[rc].trigger_data, chip->usb_conn_heat_protect_data[rc].release_data);
		}
#endif
	}

	chip->adapter_cooldown_data = (int *)cms_of_get_property_data(chip, node,
				"vivo,adapter-cooldown-data", 1, 3, ONE_DIMENSION, "adapter-cooldown-data");
	if (!chip->adapter_cooldown_data) {
		chip->adapter_cooldown_data = default_adapter_cooldown_data;
	}

	chip->adapter_power_derate_enable = of_property_read_bool(node, "vivo,adapter-power-derate-enable");

	rc = of_property_read_u32_array(node, "vivo,bsptest-soc-range",
				chip->bsptest_soc_range, 2);
	if (rc < 0) {
		chip->bsptest_soc_range[0] = 45;
		chip->bsptest_soc_range[1] = 55;
	} else {
		if (chip->bsptest_soc_range[1] < chip->bsptest_soc_range[0] ||
			chip->bsptest_soc_range[0] < 0 || chip->bsptest_soc_range[0] > 100 ||
			chip->bsptest_soc_range[1] < 0 || chip->bsptest_soc_range[1] > 100) {
			cms_print(chip,PR_WARN,"bsptest_soc_range=[%d, %d], dsti setting value error , use the default vaule.\n", chip->bsptest_soc_range[0], chip->bsptest_soc_range[1]);
			chip->bsptest_soc_range[0] = 45;
			chip->bsptest_soc_range[1] = 55;
		}
	}
	cms_print(chip,PR_WARN,"bsptest_soc_range=[%d, %d]\n", chip->bsptest_soc_range[0], chip->bsptest_soc_range[1]);

	/* Get charge current threshold */
	chip->charge_param_judge_threshold = (int *)cms_of_get_property_data(chip, node,
				"vivo,charge-param-judge-threshold", 1, 8, ONE_DIMENSION, "charge-param-judge-threshold");
	if (!chip->charge_param_judge_threshold) {
		chip->charge_param_judge_threshold = default_charge_param_judge_threshold;
	}
	cms_print(chip,PR_WARN,"charge_param_judge_threshold[%d, %d, %d, %d, %d, %d, %d, %d]\n",chip->charge_param_judge_threshold[0], chip->charge_param_judge_threshold[1], chip->charge_param_judge_threshold[2],
	chip->charge_param_judge_threshold[3], chip->charge_param_judge_threshold[4], chip->charge_param_judge_threshold[5], chip->charge_param_judge_threshold[6], chip->charge_param_judge_threshold[7]);

	/* Get charge test param */
	chip->charge_test_param_rc = (int*)cms_of_get_property_data(chip, node,
				"vivo,charge-test-param-row-col", 1, 2, ONE_DIMENSION, NULL);
	if(chip->charge_test_param_rc) {
		chip->charge_test_param_data = (struct charge_test_param_item *)cms_of_get_property_data(chip, node,
					"vivo,charge-test-param-data", chip->charge_test_param_rc[ROW],
					chip->charge_test_param_rc[COL], TWO_DIMENSION, "charge_test_param_data");
	}

	if(!chip->charge_test_param_rc || !chip->charge_test_param_data) {
		pr_err("charge test param data load fail!\n");
	}

	rc = of_property_read_u32(node, "vivo,charging-technology", &chip->charging_technology);
	if (rc) {
		if (chip->direct_charger_enable) {
			chip->charging_technology = CHG4__SINGLE_IC_DIV2_CHARGING;
		} else
			chip->charging_technology = CHG1__NORMAL_CHARGING;
	}
	cms_print(chip,PR_WARN,"charging_technology=%d, rc=%d\n", chip->charging_technology, rc);

	return 0;
}

static void cms_release_property_data(struct cms *chip)
{
	if(chip->htccc_enable){
		kfree(chip->htccc_data);
	}
	if(chip->fbon_scale_enable){
		kfree(chip->fbon_scale_data);
	}

	if(chip->calling_scale_enable){
		kfree(chip->calling_scale_data);
	}
	if(chip->fixed_scale_enable){
		kfree(chip->fixed_scale_data);
	}
	if(chip->normal_tc_data != default_normal_tc_data){
		kfree(chip->normal_tc_data);
		kfree(chip->normal_tc_rc);
	}
	if(chip->dchg_ntc_enable && chip->dchg_ntc_data != default_dchg_ntc_protect_data) {
		kfree(chip->dchg_ntc_data);
	}
	if (chip->usb_connecter_protect_enable) {
		if (chip->usb_id_protect_data != default_usb_id_protect_data) {
			kfree(chip->usb_id_protect_data);
		}
		if (chip->usb_conn_heat_protect_data != default_usb_conn_temp_heat_data) {
			kfree(chip->usb_conn_heat_protect_data);
			kfree(chip->usb_conn_heat_protect_data_rc);
		}
	}
	if(chip->intell_charge_enable){
		if(chip->parallel_temp_enable != default_parallel_temp_enable){
			kfree(chip->parallel_temp_enable);
		}
		if(chip->primary_fboff_tc_data != default_primary_fboff_tc_data){
			kfree(chip->primary_fboff_tc_data);
			kfree(chip->primary_fboff_tc_rc);
		}
		if(chip->parallel_fboff_tc_data != default_parallel_fboff_tc_data){
			kfree(chip->parallel_fboff_tc_data);
			kfree(chip->parallel_fboff_tc_rc);
		}
		if(chip->primary_fbon_tc_data != default_primary_fbon_tc_data){
			kfree(chip->primary_fbon_tc_data);
			kfree(chip->primary_fbon_tc_rc);
		}
		if(chip->parallel_fbon_tc_data != default_parallel_fbon_tc_data){
			kfree(chip->parallel_fbon_tc_data);
			kfree(chip->parallel_fbon_tc_rc);
		}
	}
	if (chip->charge_test_param_data) {
		kfree(chip->charge_test_param_data);
	}
	if(chip->charge_test_param_rc) {
		kfree(chip->charge_test_param_rc);
	}
}

static int cms_init_adc(struct cms *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct iio_channel *chan;
	char *propname = "cms_batt_id";
	int rc = 0;

	rc = of_property_match_string(node,
					"io-channel-names", propname);
	if (rc < 0)
		return 0;/*maybe not support iio-chan,try to ex-fg*/

	chan = iio_channel_get(chip->dev, propname);
	if (IS_ERR(chan)) {
		rc = PTR_ERR(chan);
		if (rc != -EPROBE_DEFER)
			cms_print(chip, PR_ERROR, "%s channel unavailable, %d\n",
							propname, rc);
		chan = NULL;
	}
	chip->iio.cms_battid_chan = chan;

	return rc;
}

static void cms_deinit_adc(struct cms *chip)
{
	if (!IS_ERR_OR_NULL(chip->iio.cms_battid_chan))
		iio_channel_release(chip->iio.cms_battid_chan);
}

static int cms_notifier_call(struct notifier_block *psy_nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct cms *chip = container_of(psy_nb, struct cms, psy_nb);
	static int ex_fg_bat_id_try_detect = true;

	/*Try to detect bq_bms bat_id again after bq_bms IC normal*/
	if (!is_atboot && chip->bat_det_method == BAT_DET_BY_EX_FG && !strcmp(psy->desc->name, "bq_bms")) {
		if (ex_fg_support && ex_fg_power_on_i2c_try_success && ex_fg_bat_id_try_detect &&
			chip->battery_rc.counter > 1 && chip->battery_rc.bat_id == B_UNKOWN) {
			ex_fg_bat_id_try_detect = false;
			cms_start_bat_id_det(chip);
			cms_print(chip, PR_ERROR, "cms_start_bat_id_det because bq_bms normal.\n");
		}
	}

	return NOTIFY_OK;
}

static int cms_register_notifier(struct cms *chip)
{
	int rc = 0;

	chip->psy_nb.notifier_call = cms_notifier_call;
	rc = power_supply_reg_notifier(&chip->psy_nb);
	if (rc < 0) {
		cms_print(chip, PR_ERROR, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int cms_probe(struct platform_device *pdev)
{
	int ret;
	struct cms *chip;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("cms alloc fail!\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->enabled = 1;
	cms_is_enable = chip->enabled;
	chip->debug_mask = &__debug_mask;
	chip->name = "cms_v2";

	chip->direct_charger_status = DIRECT_CHARGER_UNKNOW;
	chip->usb_conn_temp_high_count = 0;
	chip->bat_board_temp_high_count = 0;
	chip->bat_conn_temp_high_count = 0;
	chip->master_bat_conn_temp_high_count = 0;
	chip->usb_conn_heat_protect_count = 0;
	chip->usbid_mv = 1800;
	chip->usbin_cutoff_status = false;
	chip->ex_fg_fake_batt_id_start = false;
	chip->no_bat_id_keep_charging = false;
	chip->adapter_power_derate_param_ready = false;
	chip->exhibition_mode = 0;

	platform_set_drvdata(pdev,chip);
	cms_create_debugfs_entries(chip);//debugfs remove

	cms_print(chip,PR_INFO,"sart\n");

	ret = cms_init_adc(chip);
	if (ret < 0) {
		cms_print(chip,PR_ERROR,"Couldn't get adc rc=%d\n", ret);
		goto err_adc;
	}

	ret = cms_create_votables(chip);
	if (ret < 0) {
		cms_print(chip,PR_ERROR,"Couldn't create votables rc=%d\n",
			ret);
		goto err_votables;
	}

	ret = cms_parse_dt(chip);
	if (ret < 0) {
		cms_print(chip,PR_ERROR,"Unable to parse DT nodes\n");
		goto err_parse_dt;
	}

	switch (chip->bat_det_method) {
		case BAT_DET_BY_EX_FG:
			cms_print(chip,PR_INFO,"use ex_fg detect bat-id \n");
			chip->ex_bms_psy = power_supply_get_by_name((char *)chip->ex_bms_psy_name);
			if (!chip->ex_bms_psy) {
				cms_print(chip, PR_ERROR, "ex_fg psy not found!\n");
			}
			break;
		case BAT_DET_BY_EPROM:
			cms_print(chip,PR_INFO,"use eprom detect bat-id \n");
			chip->batid_psy = power_supply_get_by_name("batid");
			if (!chip->batid_psy) {
				cms_print(chip, PR_ERROR, "batid psy not found; defer probe\n");
			}
			break;
		case BAT_DET_BY_R:
		case BAT_DET_BY_RC:
			cms_print(chip,PR_INFO,"use %s detect bat-id \n", (chip->bat_det_method == BAT_DET_BY_R) ? "R" : "RC");
			if (chip->bat_id_gpio > 0) {
				ret = gpio_request(chip->bat_id_gpio, "vivo_battery_id");
				if (ret) {
					cms_print(chip,PR_ERROR,"gpio_request failed for %d ret=%d\n",
						chip->bat_id_gpio, ret);
					goto err_bat_id_gpio;
				}
			}
			break;
	}

	mutex_init(&chip->status_lock);
	mutex_init(&chip->battid_lock);
	mutex_init(&chip->adc_lock);

	INIT_DELAYED_WORK(&chip->bat_id_det_work, cms_bat_id_det_work);
	INIT_DELAYED_WORK(&chip->cms_work, cms_main_work);
	INIT_DELAYED_WORK(&chip->bsp_charging_work, cms_bsp_charging_work);
	INIT_DELAYED_WORK(&chip->debugfs_write_work, cms_debugfs_write_work);

	/* cms debug class */
	ret = cms_debug_class_init(chip);
	if (ret < 0) {
		cms_print(chip, PR_ERROR, "Unable to init cms debug class\n");
		goto err_psy;
	}
		
	ret = cms_register_psy(chip);
	if(ret)
		goto err_psy;

	if (!is_atboot) {
		cms_register_notifier(chip);
	}

	mutex_lock(&chip->battid_lock);
	cms_start_bat_id_det(chip);
	mutex_unlock(&chip->battid_lock);

	/*below fb logic is no usefull in new SDM670/SDM845 platform*/
#if 0
	chip->fb_nb.notifier_call = cms_fb_callback;
	ret = fb_register_client(&chip->fb_nb);
	if(ret)
		cms_print(chip,PR_ERROR,"failed to register fb client\n");
#endif

	cms_print(chip,PR_INFO,"done\n");

	return 0;
err_psy:
	mutex_destroy(&chip->adc_lock);
	mutex_destroy(&chip->status_lock);
	mutex_destroy(&chip->battid_lock);
	if (chip->bat_id_gpio > 0)
		gpio_free(chip->bat_id_gpio);
err_bat_id_gpio:
err_parse_dt:
	cms_release_property_data(chip);
err_votables:
	cms_destroy_votables(chip);
	cms_deinit_adc(chip);
err_adc:
	if(chip->dent){
		debugfs_remove_recursive(chip->dent);
		chip->dent = 0;
	}
	kfree(chip);
	return ret;
}

static int cms_remove(struct platform_device *pdev)
{
	struct cms *chip = platform_get_drvdata(pdev);
	cms_print(chip,PR_INFO,"\n");
	cms_destroy_votables(chip);
	cms_release_property_data(chip);
	cms_deinit_adc(chip);
	kfree(chip);
	return 0;
}

static void cms_shutdown(struct platform_device *pdev)
{
	struct cms *chip = platform_get_drvdata(pdev);
	cms_print(chip,PR_INFO,"\n");
}

static const struct of_device_id cms_match_table[] = {
	{ .compatible = "vivo,charger-monitor-v2", },
	{ },
};

static struct platform_driver cms_driver = {
	.driver = {
		.name = "charger-monitor-v2",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cms_match_table),
	},
	.probe = cms_probe,
	.remove = cms_remove,
	.shutdown	= cms_shutdown,
};

module_platform_driver(cms_driver);

MODULE_DESCRIPTION("Charger Monitor System v2");
MODULE_LICENSE("GPL v2");
