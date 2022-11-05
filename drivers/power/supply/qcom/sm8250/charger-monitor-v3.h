#ifndef __CHARGER_MONITOR_V3_H
#define __CHARGER_MONITOR_V3_H

/* voter define */
#define CALLING_EVENT		"CALLING_EVENT"
#define WEIXIN_CALLING_EVENT		"WEIXIN_CALLING_EVENT"
#define SWITCH_EVENT		"SWITCH_EVENT"
#define AI_SWITCH_EVENT	"AI_SWITCH_EVENT"
#define FBON_EVENT			"FBON_EVENT"
#define ABNORMAL_EVENT		"ABNORMAL_EVENT"
#define DCHG_NTC_EVENT		"DCHG_NTC_EVENT"
#define DEBUGFS_EVENT		"DEBUGFS_EVENT"
#define USBID_WATER_EVENT		"USBID_WATER_EVENT"
#define USB_CONN_HEAT_EVENT		"USB_CONN_HEAT_EVENT"
#define NORMAL_CHARGE		"NORMAL_CHARGE"
#define INTELL_CHARGE		"INTELL_CHARGE"
#define FIXED_CHARGE		"FIXED_CHARGE"
#define DCHG_ADAPTER_COOLDOWN_CHARGE		"DCHG_ADAPTER_COOLDOWN_CHARGE"
#define DCHG_CABLER_CURRENTDOWN_CHARGE		"DCHG_CABLER_CURRENTDOWN_CHARGE"
#define EX_FG_FAKE_BATT_ID_START		"EX_FG_FAKE_BATT_ID_START"


/* */
#define CMS_MAIN_WORK_PERIOD_MS 5000
#define BAT_ID_RC_CHARGE_PERIOD_MS 2000
#define BAT_ID_READING_RC_PERIOD_MS 145
#define BAT_ID_REF_VOLTAGE 1800
#define BAT_HEALTH_OV_DELTA_MV	100
#define BAT_PLUGABLE_DEFAULT 0
#define BATTERY_PLUGED_DETECT_STEP 1
#define CHG_TIMEOUT_MINS_DEFAULT 600
#define DEFAULT_BATTERY_FLOAT_VOLTAGE_UV	4380000
#define CHECK_FASTCH_CURRENT_VBAT_THRESHOLD_MV	4250
#define CHECK_FASTCH_CURRENT_IBAT_THRESHOLD_MA	2500
#define CHECK_FASTCH_CURRENT_THRESHOLD_MA	500

/* debug define */
#define cms_print(chip, reason, fmt, ...)			\
	do {							\
		if (*chip->debug_mask & (reason))		\
			pr_err("%s: %s: " fmt, chip->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chip->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

enum print_reason {
	PR_INFO 	= BIT(0),
	PR_ERROR	= BIT(1),
	PR_WARN 	= BIT(2),
	PR_DBG		= BIT(3),
};

enum bat_id_det_state{
	BAT_ID_IDLE = 0,
	BAT_ID_START,
	BAT_ID_CHARGING_RC,
	BAT_ID_READING_RC,
	BAT_ID_DONE,
};

enum battery_id{
	B_UNKOWN = 0,
	B_THE_1_SUPPLIER,
	B_THE_2_SUPPLIER,
	B_THE_3_SUPPLIER,
	B_THE_4_SUPPLIER,
	B_THE_5_SUPPLIER,
	B_THE_6_SUPPLIER,
	B_THE_7_SUPPLIER,
	B_MAX,
};

enum bat_det_method {
	BAT_DET_BY_R,
	BAT_DET_BY_RC,
	BAT_DET_BY_EPROM,
	BAT_DET_BY_EX_FG,
	BAT_DET_BY_MAX,
};

enum fastchg_condition {
	FASTCHG_CONDITION_FB_ON,
	FASTCHG_CONDITION_FB_OFF,
	FASTCHG_CONDITION_MAX,
};

enum
{
  CMS_TEMP_LCM			= 0x00U,
  CMS_TEMP_PARALLEL		= 0x01U,
  CMS_BAT_ID			= 0x02U,
};

enum
{
	ONE_DIMENSION = 0,
	TWO_DIMENSION,
	THREE_DIMENSION,
	FOUR_DIMENSION,
};

enum
{
	ROW = 0,
	COL = 1,
};

enum{
	FB_OFF = 0,
	FB_ON,
};

enum
{
	FBON_TIMER = 0,
	FBON_INPUT,
	FBON_IBAT,
};

enum
{
	SWITCH_INPUT = 0,
	SWITCH_IBAT,
};

enum
{
	CALLING_INPUT = 0,
	CALLING_IBAT,
};

enum
{
	FIXED_TIMER = 0,
	FIXED_PRIMARY,
	FIXED_PARALLEL,
};

enum
{
	HTCCC_TEMP_L = 0,
	HTCCC_TEMP_H,
	HTCCC_VBAT,
	HTCCC_DELTA,
};

enum DIRECT_CHARGER_STATUS{
	DIRECT_CHARGER_UNKNOW = 0x0,
	DIRECT_CHARGER_PREPARE = 0x1,
	DIRECT_CHARGER_IS_CHARGERING = 0x2,
	DIRECT_CHARGER_END = 0x3,
	DIRECT_CHARGER_IS_BYPASS_TO_SWITCH_IC = 0x4,
};

enum DIRECT_CHARGER_NTC{
	BAT_BOARD_TEMP = 0x0,
	USB_CONN_TEMP = 0x1,
	MASTER_BAT_CONN_TEMP = 0x2,
	BAT_CONN_TEMP = 0x3,
};

enum HEAT_PROTECT {
	HEAT_TRIGGER = 0x0,
	HEAT_RELEASE = 0x1,
	IBAT_ACTION = 0x2,
};

enum USBID_WATER_PROTECT {
	USBID_WATER_TRIGGER = 0x0,
	USBID_WATER_RELEASE = 0x1,
	USBID_WATER_ACTION = 0x2,
};

enum DEBUGFS_WRITE_FLAG {
	DWF_CHG_UNKNOWN = 0x0,
	DWF_CHG_CUTOFF = 0x1,
	DWF_CHG_OPEN = 0x2,
	DWF_FG_CUTOFF = 0x3,
	DWF_FG_OPEN = 0x4,
};

struct battery_rc{
	uint64_t					timestamp;
	int64_t 					v1;
	int64_t 					v2;
	int32_t						bat_id;
	int32_t						counter;
	enum bat_id_det_state	state;
};

struct tc_data_item{
	int		min;
	int		max;
	int		data;
};

struct tc_data_item_ex{
	int		min;
	int		max;
	int		data;
	int		volt1;
	int		data_ex1;
	int		volt2;
	int		data_ex2;
};

struct r_item{
	int		v1min;
	int		v1max;
	int		id;
};

struct rc_item{
	int		v1min;
	int		v1max;
	int		v2min;
	int		v2max;
	int		id;
};

struct cms_iio {
	struct iio_channel	*cms_battid_chan;
};

struct usb_conn_heat_action_item{
	int		Tmin;
	int		Tmax;
	int		trigger_data;
	int		release_data;
};

struct charge_test_param_item{
	int		enable_switch;
	int		min_value;
	int		max_value;
};

struct cms{
	struct device *dev;
	int *debug_mask;
	char *name;
	int enabled;
	int online;
	int bat_id_gpio;
	int vadc_chan;
	int battery_present;
	int ac_avg_current;
	int batt_plugable;
	int bat_det_method;
	bool use_new_rc_param;
	long health_status;
	struct mutex status_lock;
	struct mutex battid_lock;
	struct mutex adc_lock;
	struct dentry *dent;
	struct power_supply	*cms_psy;
	struct power_supply	*batt_psy;
	struct power_supply *bms_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*batid_psy;
	struct power_supply	*mcu_psy;
	struct delayed_work	bat_id_det_work;
	struct delayed_work	cms_work;
	struct delayed_work	bsp_charging_work;
	struct battery_rc battery_rc;
	struct cms_iio iio;
	struct notifier_block fb_nb;
	struct notifier_block	psy_nb;
	bool fb_notify_enable;
	bool fb_on;
	bool no_bat_warning;
	bool no_bat_id_keep_charging;
	bool no_chg_timeout;
	bool cms_work_started;
	bool pon_batt_pluged;
	int nr_work;
	int continue_count;
	void *complete;
	atomic_t in_batid;
	int factory_mode_state;
	bool usbin_cutoff_status;

	bool htccc_enable;
	int *htccc_data;

	bool fbon_scale_enable;
	int *fbon_scale_data;

	bool ai_charge_enable;
	int ai_disable_charge;
	bool ai_disable_charge_state;

	bool calling_scale_enable;
	bool calling_state;
	int *calling_scale_data;
	bool calling_dchg_disable;

	bool weixin_calling_scale_enable;
	bool weixin_calling_state;
	int *weixin_calling_scale_data;

	bool fixed_scale_enable;
	int *fixed_scale_data;

	struct votable *user_ibat_votable;
	struct votable *user_input_votable;

	int *normal_tc_rc;
	struct tc_data_item_ex *normal_tc_data;

	bool intell_charge_enable;
	int *parallel_temp_enable;
	int *batt_fbon_tc_rc;
	struct tc_data_item *batt_fbon_tc_data;
	int *primary_fbon_tc_rc;
	struct tc_data_item *primary_fbon_tc_data;
	int *parallel_fbon_tc_rc;
	struct tc_data_item *parallel_fbon_tc_data;
	int *batt_fboff_tc_rc;
	struct tc_data_item *batt_fboff_tc_data;
	int *primary_fboff_tc_rc;
	struct tc_data_item *primary_fboff_tc_data;
	int *parallel_fboff_tc_rc;
	struct tc_data_item *parallel_fboff_tc_data;

	bool ex_intell_charge_enable;
	int *ex_parallel_temp_enable;
	int *ex_batt_fbon_tc_rc;
	struct tc_data_item *ex_batt_fbon_tc_data;
	int *ex_primary_fbon_tc_rc;
	struct tc_data_item *ex_primary_fbon_tc_data;
	int *ex_parallel_fbon_tc_rc;
	struct tc_data_item *ex_parallel_fbon_tc_data;
	int *ex_batt_fboff_tc_rc;
	struct tc_data_item *ex_batt_fboff_tc_data;
	int *ex_primary_fboff_tc_rc;
	struct tc_data_item *ex_primary_fboff_tc_data;
	int *ex_parallel_fboff_tc_rc;
	struct tc_data_item *ex_parallel_fboff_tc_data;

	bool bypass_intell_charge_enable;
	int *bypass_batt_fbon_tc_rc;
	struct tc_data_item *bypass_batt_fbon_tc_data;
	int *bypass_primary_fbon_tc_rc;
	struct tc_data_item *bypass_primary_fbon_tc_data;
	int *bypass_parallel_fbon_tc_rc;
	struct tc_data_item *bypass_parallel_fbon_tc_data;
	int *bypass_batt_fboff_tc_rc;
	struct tc_data_item *bypass_batt_fboff_tc_data;
	int *bypass_primary_fboff_tc_rc;
	struct tc_data_item *bypass_primary_fboff_tc_data;
	int *bypass_parallel_fboff_tc_rc;
	struct tc_data_item *bypass_parallel_fboff_tc_data;

	/* add for equalizing_charge begin*/
	bool switch_state;
	bool eq_intell_charge_enable;
	int *eq_batt_fbon_tc_rc;
	struct tc_data_item *eq_batt_fbon_tc_data;
	int *eq_primary_fbon_tc_rc;
	struct tc_data_item *eq_primary_fbon_tc_data;
	int *eq_parallel_fbon_tc_rc;
	struct tc_data_item *eq_parallel_fbon_tc_data;
	int *eq_batt_fboff_tc_rc;
	struct tc_data_item *eq_batt_fboff_tc_data;
	int *eq_primary_fboff_tc_rc;
	struct tc_data_item *eq_primary_fboff_tc_data;
	int *eq_parallel_fboff_tc_rc;
	struct tc_data_item *eq_parallel_fboff_tc_data;
	/* add for equalizing_charge end*/
	int intell_charge_primary_result_ma;
	int intell_charge_parallel_result_ma;

	struct power_supply *ex_bms_psy;
	const char *ex_bms_psy_name;

	int chg_status;
	int chg_timeout_mins;
	unsigned long chg_begin;

	bool	direct_charger_enable;
	int direct_charger_status;
	struct votable		*dchg_disable_votable;

	/* dchg ntc protect */
	bool dchg_ntc_enable;
	int *dchg_ntc_data;
	int	bat_board_temp_high_count;
	int	usb_conn_temp_high_count;
	int	master_bat_conn_temp_high_count;
	int	bat_conn_temp_high_count;

	/* usb connecter protect */
	int	usb_connecter_protect_enable;
	int usbid_mv;
	int *usb_id_protect_data;
	int *usb_conn_heat_protect_data_rc;
	struct usb_conn_heat_action_item *usb_conn_heat_protect_data;
	int usb_conn_heat_protect_count;

	/* step charge */
	int last_step;

	/* battery voltage compensate */
	int vbat_r_comp;
	struct power_supply *fuelsummary_psy;
	int custom_input;
	int custom_current;

	/* debugfs_write_work */
	struct delayed_work	debugfs_write_work;
	int debugfs_write_flag;

	int *adapter_cooldown_data;
	int adapter_power_derate_enable;

	bool ex_fg_fake_batt_id_start;

	int bsptest_soc_range[2];
	int battery_temp;
	int main_board_temp;
	int sub_board_temp;
	int usb_connector_temp;
	int main_bat_connector_temp;
	int sub_bat_connector_temp;
	int battery_board_temp;
	int adapter_temp;
	char	chg_state[400];
	int *charge_param_judge_threshold;
	int *charge_test_param_rc;
	struct charge_test_param_item *charge_test_param_data;

	int exhibition_mode;
	int exhibition_mode_current_primary;
	int exhibition_mode_current_parallel;

	int charging_technology;
	int normal_charge_batt_scale;

	bool force_bypass_to_direct_charging;

	/* cms debug class */
	struct class cms_debug_class;
};

enum FAST_CHG_STATE {
	fast_chg_state_NONE = 0,			//normal charging: 5V/2A  9V/2A
	fast_chg_state_VFCP,			//VIVO Fast Charge
	fast_chg_state_VFCP_bypass,		//VIVO Fast Charge bypass mode: 1:1 mode
};

enum CHARGING_TECHNOLOGY_LIST {
	CHG0__CHARGING_TECHNOLOGY_NONE = 0,

	CHG1__NORMAL_CHARGING,								//5V/2A normal charging
	CHG2__QC2_CHARGING,									//9V/2A	QC2.0 charging
	CHG3__LOW_VOLTAGE_DIRECT_CHARGING,					//5V/4.5A direct charging
	CHG4__SINGLE_IC_DIV2_CHARGING,						//single IC Div/2 charging
	CHG5__DUAL_IC_DIV2_CHARGING,						//Dual IC Div/2 charging
	CHG6__DUAL_IC_DUAL_BATTERY_DIV2_CHARGING,			//Dual IC Div/2 + Dual Battery charging

	CHGX__CHARGING_TECHNOLOGY_MAX,
};

/*enum values must be matched with values in charger.c*/
enum{
	HEALTH_STATUS_MIN = 0,
	HEALTH_STATUS_CHG_OV = HEALTH_STATUS_MIN,
	HEALTH_STATUS_BAT_WARM,
	HEALTH_STATUS_CHG_OC,
	HEALTH_STATUS_BAT_OV,
	HEALTH_STATUS_CHG_TIMEOUT,
	HEALTH_STATUS_BAT_COLD,
	HEALTH_STATUS_BAT_MISSING,
	HEALTH_STATUS_CHG_UV,
	HEALTH_STATUS_BAT_INVALID,
	HEALTH_STATUS_CHG_ERR,
	//HEALTH_STATUS_BAT_UV,
	//HEALTH_STATUS_CHG_DONE,
	HEALTH_STATUS_BAT_UNPLUGED = 12,
	HEALTH_STATUS_HTCCC = 13,
	HEALTH_STATUS_USB_CONN_HEAT = 14,
	HEALTH_STATUS_USBID_WATER = 15,
	HEALTH_STATUS_MAX,
};

static int health_status_map[][2] ={
	{HEALTH_STATUS_CHG_OV,		0x0001},
	{HEALTH_STATUS_BAT_WARM,	0x0002},
	{HEALTH_STATUS_CHG_OC,		0x0004},
	{HEALTH_STATUS_BAT_OV,		0x0008},
	{HEALTH_STATUS_CHG_TIMEOUT,	0x0010},
	{HEALTH_STATUS_BAT_COLD,	0x0020},
	{HEALTH_STATUS_BAT_MISSING,	0x0040},
	{HEALTH_STATUS_CHG_UV,		0x0080},
	{HEALTH_STATUS_BAT_INVALID,	0x0100},
	{HEALTH_STATUS_CHG_ERR,0x0200},
	//{HEALTH_STATUS_BAT_UV,0x0400},
	//{HEALTH_STATUS_CHG_DONE,0x0800},
	{HEALTH_STATUS_BAT_UNPLUGED,0x1000},
	{HEALTH_STATUS_HTCCC,		0x2000},
	{HEALTH_STATUS_USB_CONN_HEAT,		0x4000},
	{HEALTH_STATUS_USBID_WATER,		0x8000},
};

static const char * const bat_id_det_state_strings[] = {
	"BAT_ID_IDLE",
	"BAT_ID_START",
	"BAT_ID_CHARGING_RC",
	"BAT_ID_READING_RC",
	"BAT_ID_DONE",
};

static const char * const cms_adc_type_strings[] = {
	"lcm temperature",
	"parallel temperature",
	"battery id",
};

static const char * const battery_id_strings[] = {
	"unkown battery supplier",
	"the 1 battery supplier : 33k--(20kohm)",
	"the 2 battery supplier : 100k--(50kohm)",
	"the 3 battery supplier : 10k--(100kohm)",
	"the 4 battery supplier : 33K,4.7uf--(200kohm)",
	"the 5 battery supplier : 33k,10uf--(500kohm)",
	"the 6 battery supplier : 100k,2.2uf--(0kohm)",
	"the 7 battery supplier : 100k,10uf--(-699kohm)",
	"out of the battery supplier range",
};

static int default_dchg_ntc_protect_data[]= {650, 700, 1000, 1000};

static struct tc_data_item_ex default_normal_tc_data[] = {
	{
		.min	= 550,
		.max	= 8888,
		.data	= 0,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
	{
		.min	= 450,
		.max	= 549,
		.data	= 50,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
	{
		.min	= 200,
		.max	= 449,
		.data	= 70,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
	{
		.min	= 150,
		.max	= 199,
		.data	= 45,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
	{
		.min	= 100,
		.max	= 149,
		.data	= 30,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
	{
		.min	= 50,
		.max	= 99,
		.data	= 25,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
	{
		.min	= 0,
		.max	= 49,
		.data	= 20,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
	{
		.min	= -80,
		.max	= -1,
		.data	= 10,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
	{
		.min	= -8888,
		.max	= -81,
		.data	= 0,
		.volt1	= 0,
		.data_ex1 = 0,
		.volt2	= 0,
		.data_ex2 = 0,
	},
};
static int default_parallel_temp_enable[]= {0,0};

static struct tc_data_item default_primary_fbon_tc_data[]= {
	{
		.min	= 471,
		.max	= 8888,
		.data	= 500,
	},
	{
		.min	= 451,
		.max	= 470,
		.data	= 500,
	},
	{
		.min	= 421,
		.max	= 450,
		.data	= 1200,

	},
	{
		.min	= -8888,
		.max	= 420,
		.data	= 1800,
	},
};

static struct tc_data_item default_parallel_fbon_tc_data[]= {
	{
		.min	= 411,
		.max	= 8888,
		.data	= 512,
	},
	{
		.min	= 391,
		.max	= 410,
		.data	= 832,
	},
	{
		.min	= 371,
		.max	= 390,
		.data	= 1024,
	},
	{
		.min	= -8888,
		.max	= 370,
		.data	= 1408,
	},
};

static struct tc_data_item default_primary_fboff_tc_data[]= {
	{
		.min	= 441,
		.max	= 8888,
		.data	= 500,
	},
	{
		.min	= 411,
		.max	= 440,
		.data	= 300,
	},
	{
		.min	= 381,
		.max	= 410,
		.data	= 700,

	},
	{
		.min	= -8888,
		.max	= 380,
		.data	= 1100,
	},
};

static struct tc_data_item default_parallel_fboff_tc_data[]= {
	{
		.min	= 441,
		.max	= 8888,
		.data	= 0,
	},
	{
		.min	= 411,
		.max	= 440,
		.data	= 1024,
	},
	{
		.min	= 381,
		.max	= 410,
		.data	= 1024,

	},
	{
		.min	= -8888,
		.max	= 380,
		.data	= 1024,
	},
};

static int default_adapter_cooldown_data[] = {900, 850, 3400};	/* 500K, 2M, fcc */

enum usb_connecter_protect_type {
	USB_PROTECT_NONE = 0,
	USB_CONN_TEMP_HEAT_PROTECT = BIT(0),
	USB_ID_PROTECT = BIT(1),
};

static int default_usb_id_protect_data[] = {1100, 1547, 500};	/* 500K, 2M, fcc */
/*
*max vbus of adapter:5v, max ibus of adapter:2A
*min voltage of battery for half chg:3500mV,max voltage of battery for half charge:4100mV,
*5V2A charge min current threshold: 1000mA,5V2A charge max current threshold:2100mA,
*flash charge min current threshold:5000mA,flash charge max current threshold:11000mA
*bypass 11V5A: min current threshold 3500mA, max current threshold:6000mA
*/
static int default_charge_param_judge_threshold[] = {5, 2, 3500, 4100, 500, 2100, 3000, 8100, 3500, 6000};
static struct usb_conn_heat_action_item default_usb_conn_temp_heat_data[] = {
	{
		.Tmin	= 260,
		.Tmax	= 8888,
		.trigger_data	= 670,
		.release_data	= 620,
	},
	{
		.Tmin	= -8888,
		.Tmax	= 259,
		.trigger_data	= 570,
		.release_data	= 520,
	},
};

static const struct r_item new_r_items[] = {
	//3.3Kom
	{
		.v1min	= 78,
		.v1max	= 178,
		.id		= B_THE_1_SUPPLIER
	},
	//10Kom
	{
		.v1min	= 251,
		.v1max	= 372,
		.id		= B_THE_2_SUPPLIER
	},
	//18Kom
	{
		.v1min	= 431,
		.v1max	= 568,
		.id		= B_THE_3_SUPPLIER
	},
	//33Kom
	{
		.v1min	= 649,
		.v1max	= 818,
		.id		= B_THE_4_SUPPLIER
	},
	//56Kom
	{
		.v1min	= 882,
		.v1max	= 1087,
		.id		= B_THE_5_SUPPLIER
	},
	//120Kom
	{
		.v1min	= 1171,
		.v1max	= 1407,
		.id		= B_THE_6_SUPPLIER
	},
	//390Kom
	{
		.v1min	= 1482,
		.v1max	= 1751,
		.id		= B_THE_7_SUPPLIER
	},
};

static const struct rc_item new_rc_items[] = {
	//33Kom+4.7uf
	{
		.v1min	= 639,
		.v1max	= 848,
		.v2min	= 158,
		.v2max	= 364,
		.id		= B_THE_1_SUPPLIER,
	},
	//33Kom+10uf
	{
		.v1min	= 639,
		.v1max	= 848,
		.v2min	= 386,
		.v2max	= 571,
		.id		= B_THE_2_SUPPLIER,
	},
	//120Kom+10uf
	{
		.v1min	= 1161,
		.v1max	= 1427,
		.v2min	= 973,
		.v2max	= 1261,
		.id		= B_THE_3_SUPPLIER,
	},
	//120Kom+2.2uf
	{
		.v1min	= 1161,
		.v1max	= 1427,
		.v2min	= 560,
		.v2max	= 878,
		.id		= B_THE_4_SUPPLIER,
	},
};


static const struct r_item r_items[] = {
	{
		.v1min	= 746,
		.v1max	= 966,
		.id		= B_THE_1_SUPPLIER
	},
	{
		.v1min	= 1117,
		.v1max	= 1398,
		.id		= B_THE_2_SUPPLIER
	},
	{
		.v1min	= 447,
		.v1max	= 619,
		.id		= B_THE_3_SUPPLIER
	},
};

static const struct rc_item rc_items[] = {
	{
		.v1min	= 746,
		.v1max	= 966,
		.v2min	= 0,
		.v2max	= 129,
		.id		= B_THE_1_SUPPLIER,//33k  20kohm
	},
	{
		.v1min	= 1117,
		.v1max	= 1398,
		.v2min	= 0,
		.v2max	= 129,
		.id		= B_THE_2_SUPPLIER,//100k  50kohm
	},
	{
		.v1min	= 447,
		.v1max	= 619,
		.v2min	= 0,
		.v2max	= 129,
		.id		= B_THE_3_SUPPLIER,//10k  100kohm
	},
	{
		.v1min	= 746,
		.v1max	= 966,
		.v2min	= 129,
		.v2max	= 329,
		.id		= B_THE_4_SUPPLIER,//33K,4.7uf  200kohm
	},
	{
		.v1min	= 746,
		.v1max	= 966,
		.v2min	= 337,
		.v2max	= 509,
		.id		= B_THE_5_SUPPLIER,//33k,10uf  500kohm
	},
	{
		.v1min	= 1117,
		.v1max	= 1398,
		.v2min	= 250, //406,
		.v2max	= 719,
		.id		= B_THE_6_SUPPLIER,//100k,2.2uf  0kohm
	},
	{
		.v1min	= 1117,
		.v1max	= 1398,
		.v2min	= 825,
		.v2max	= 1113,
		.id		= B_THE_7_SUPPLIER,//100k,10uf  -699kohm
	},
};

#endif /* __CHARGER_MONITOR_V3_H */
