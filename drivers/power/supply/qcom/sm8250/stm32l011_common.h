/*
  **********************************************************************************
  *  Please make sure "AP side" & "MCU side" use the same file(stm32l011_common.h) *
  *    Use for High-Voltage Dual Battery Charging                                  *
  **********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L011_COMMON
#define __STM32L011_COMMON

#if defined(CONFIG_STM32l011X4)	//AP kernel Kconfig/Makefile will defined it.
	#define AP_SIDE
	#if defined(MCU_SIDE)
		#undef MCU_SIDE
	#endif
#else
	#define MCU_SIDE
	#if defined(AP_SIDE)
		#undef AP_SIDE
	#endif
#endif

/*****************************************************************************************************************************
 *	[VIVO Adapter List]:                                                                                                     *
 *		1. QC2.0:	9V/2A 			---> Dual Engine Fast Charging(for dual IC) / Fast Charging(for single IC).			     *
 *		2. 22.5W:	10V/2.25A   	---> vivo FlashCharge                                                                    *
 *		3. 33W:		11V/3A          ---> vivo FlashCharge 2.0                                                                *
 *		4. 44W:		11V/4A          ---> vivo Super FlashCharge                                                              *
 *		5. 55W:		11V/5A          ---> vivo Super FlashCharge 2.0                                                          *
 *		6. 120W:	20V/6A			---> vivo Super FlashCharge 3.0 (VFCP 3.0)                                               *
******************************************************************************************************************************/
/*****************************************************************************************************************************
 	[VIVO protocol CMD:0x01]
		L_4bit = 0001(1)	-->　10-11V/4A
 		L_4bit = 0010(2)	-->　10V/2.25A
 		L_4bit = 0011(3)	-->　10-11V/3A
 		L_4bit = 0101(5)	-->　10-11V/5A
 		L_4bit = 0110(6)	-->　20V/6A	 new value
 		L_4bit = 1001(9)	-->　10-11V/6A
 		L_4bit = 1010(A)	-->　20V/6A	 old value
******************************************************************************************************************************/
#define IS_VIVO_SUPER_FLASHCHARGE(adapter_power)	\
	((adapter_power & 0x0F) == 0x01 || (adapter_power & 0x0F) == 0x05 || (adapter_power & 0x0F) == 0x06 || (adapter_power & 0x0F) == 0x09 || (adapter_power & 0x0F) == 0x0A)

#define GET_VIVO_ADAPTER_VOLTAGE_MAX(adapter_power)	\
	(((adapter_power & 0x0F) == 0x0A || (adapter_power & 0x0F) == 0x06) ? 20 : ((adapter_power & 0x0F) == 0x02 ? 10 : ((adapter_power & 0x0F) > 0 ? 11 : 5)))

#ifndef BIT
	#define BIT(x) (1 << x)
#endif

#ifndef GENMASK_8BIT
/*
 * GENMASK_8BIT(4, 1) gives us the 8bit vector 0001,1110.
 */
#define GENMASK_8BIT(h, l) \
	(((~0UL) - (1UL << (l)) + 1) & (~0UL >> (8 - 1 - (h))))
#endif

//SMB1396
static const uint16_t smb1396_dchg_ic_register_dump[] = {
	0x2609, 0x2610, 0x2618, 0x2630, 0x2631, 0x2632, 0x2634, 0x2635, 0x2636, 0x2637, 0x2639, 0x2641,
	0x2642, 0x2643, 0x2644, 0x2645, 0x2646, 0x2652, 0x2653, 0x2655, 0x2656, 0x265C, 0x265D, 0x2660,
	0x2663, 0x2666, 0x2669, 0x266C, 0x266D, 0x2672, 0x267A, 0x2682, 0x2683, 0x26CF, 0x2710, 0x27C3,
};
#define  smb1396_dchg_ic_register_dump_len 	(sizeof(smb1396_dchg_ic_register_dump)/sizeof(uint16_t))

//BQ25890
static const uint16_t *bq25890_dchg_ic_register_dump = NULL;
#define  bq25890_dchg_ic_register_dump_len 			0x38

/* MCU information  **********************************************************************************V*/
#if defined(MCU_SIDE)
//#define DCHG_IC_SMB1396	1	//define in the Keil tool build CMD config: Options for Target --> C/C++ --> Misc Controls : -DDCHG_IC_SMB1396

#define dchg_null_default(value)
#define dchg_get_vbus_default()			hdchg.map->vbus
#define dchg_get_vbat_default()			hdchg.map->vbat
#define dchg_get_ibus_default(index)	hdchg.map->ibus[index]	//please update the ADC before.

//Dchg IC api
#if DCHG_IC_SMB1396
#define dchg_hw_init		smb1396_Hw_Init
#define dchg_dump_register	smb1396_dump_register
#define dchg_get_vin_status	smb1396_div2_windown_status
#define dchg_get_vbus		dchg_get_vbus_default
#define dchg_get_vbat		dchg_get_vbat_default()
#define dchg_get_ibus		dchg_get_ibus_default

#define dchg_otg_mode		dchg_null_default
#define dchg_chg_enable		smb1396_Chg_Enable	//NOTE: dchg_chg_enable() --> dchg_chg_is_enable() --> dchg_get_vin_status();
#define dchg_chg_is_enable	smb1396_Chg_Is_Enable
#define dchg_ic_register_dump	smb1396_dchg_ic_register_dump
#define dchg_ic_register_dump_len	smb1396_dchg_ic_register_dump_len

#elif DCHG_IC_BQ25890
#define dchg_hw_init		bq25980_Hw_Init
#define dchg_dump_register	bq25980_dump_register
#define dchg_get_vin_status	bq25890_div2_windown_status
#define dchg_get_vbus		hdchg.map->vbus = Bq25980_Get_VbusADC((hdchg.map->dchgIC_i2cRexp[M] > 20) ? DCHG_IC__SLAVE_BOARD : DCHG_IC__MAIN_BOARD)
#define dchg_get_vbat		hdchg.map->vbat = Bq25980_Get_VbatADC((hdchg.map->dchgIC_i2cRexp[M] > 20) ? DCHG_IC__SLAVE_BOARD : DCHG_IC__MAIN_BOARD)	//dchg_get_vbat_default
#define dchg_get_ibus		dchg_get_ibus_default

#define dchg_otg_mode		Bq25980_otg_mode
#define dchg_chg_enable		Bq25980_Chg_Enable	//NOTE: dchg_chg_enable() --> dchg_chg_is_enable() --> dchg_get_vin_status();
#define dchg_chg_is_enable	Bq25980_Chg_Is_Enable
#define dchg_ic_register_dump	bq25890_dchg_ic_register_dump
#define dchg_ic_register_dump_len	bq25890_dchg_ic_register_dump_len
#endif

#endif
/* MCU information  **********************************************************************************A*/

#define VIVO_DIRECT_CHARGER_IC_COUNT	2

enum VIVO_DIRECT_CHARGER_IC_index
{
	DCHG_IC__MAIN_BOARD = 0, 
	DCHG_IC__SLAVE_BOARD = (VIVO_DIRECT_CHARGER_IC_COUNT-1),

	DCHG_IC_MAX,
};

static const char * const VIVO_DIRECT_CHARGER_IC_string[] = {
	"MAIN__BOARD",
	"SLAVE_BOARD",
};

#define VIVO_DIRECT_CHARGER_IC_COUNT	2

#define MASTER_DCHG		DCHG_IC__MAIN_BOARD
#define SLAVE_DCHG		DCHG_IC__SLAVE_BOARD
#define M	MASTER_DCHG
#define S	SLAVE_DCHG

/* MCU register MAP-----------------------------------------------------------V*/
struct mcu_register_map	//NOTE: STM32 is little-endian memory; ARM CPU also little-endian momory.
{
 	/* Please make sure:  sizeof(struct mcu_register_map) < 0xFF */
 	/* usefull register please put in before for AP read() easy.*/

	/*[Read/Write Register]********************************************/
	uint8_t IRA;	//FSM
	uint8_t IRB;	//Exit status
	uint8_t IRC;	//Error status
	uint8_t IRD;	//Error status
	uint8_t R04;	// Control between AP/MCU
	uint8_t R05;	// status

	uint16_t reqIbus;
	uint16_t reqVbus;

	uint16_t adapter_vbus;
	uint16_t adapter_ibus;

	uint16_t vbus;	//mV	: MCU adc or get from AP side
	uint16_t vbat;	//mV	: MCU adc from battery BTB
	uint16_t fg_vbat;	//mV
	uint16_t vcell1;	//mV
	uint16_t vcell2;	//mV
	uint16_t ibus[VIVO_DIRECT_CHARGER_IC_COUNT];	//mA

	/* NOTE: only main_board/main_charger Charging, and then Slave_board/Slave_charger can charging...*/
	uint8_t charging;	//charging status: BIT(DCHG_IC__MAIN_BOARD) | BIT(DCHG_IC__SLAVE_BOARD)
	uint8_t enabled;	//enable status: come from AP side, to control DCHG IC enable or not:	BIT(DCHG_IC__MAIN_BOARD) | BIT(DCHG_IC__SLAVE_BOARD)

	uint16_t	dp;			// Voltage=mV
	uint16_t	dm;			// Voltage=mV
	uint16_t cable_mohm;

	uint8_t current_down_count;
	uint16_t current_down_start;	//mA, backup the current_down start value

	uint8_t main_chg_status;
	uint8_t dchg_status;
	uint8_t hv_div2_charge_bypass_mode;

	uint8_t	start_ic_order[2];	//[0] = first start ic number; [1] = second start ic number.
	uint8_t vbat_rb_status;	//check_vbat_reverse_back_status
	//---------------------------------------------------------------- above registers AP will read quickly.
	uint8_t enabled_cnt[VIVO_DIRECT_CHARGER_IC_COUNT];
	uint8_t vin_status[VIVO_DIRECT_CHARGER_IC_COUNT];	//Vin/2-Vout = Too High/Too low

	uint8_t otg_mode;

	uint16_t ap_to_mcu_ibat;		//ibat_current_mA
	uint16_t termination_vbat;	// compare with FG IC Vbat
	uint16_t termination_ibat;	// Ibat term Value

	uint16_t dchgIC_i2cRexp[VIVO_DIRECT_CHARGER_IC_COUNT];
	uint16_t dchgIC_i2cWexp[VIVO_DIRECT_CHARGER_IC_COUNT];
	uint16_t UartDpDm_Short_Exp;
	uint16_t UartRxExp;
	uint16_t UartRxTimeOutExp;
	uint32_t AdapterHealthStatus;
	//---------------------------------------------------------------- above registers AP will read slowly.

	uint16_t usbConnTemp;	//adc value
	uint16_t batBtbTemp;	//adc value
	uint16_t adapterConnTemp;
	uint16_t adapterTemp;

	uint16_t adapterIbus_max;
	uint8_t adapter_power;
	uint8_t adapter_power_derating[4];	/*cmd_0xD: T_trigger; I_trigger_rate; T_recover; I_recover_rate*/
	uint8_t adapter_version;
	uint8_t adapter_manufacture_code;
	uint8_t adapter_vendor;

	/*VFCP_CMD_0x20*/
	uint16_t adapter_HV_ibus_max;	//High(20V) voltage Ibus MAX value mA;
	uint16_t adapter_LV_ibus_max;	//Low(11V) voltage Ibus MAX value mA;
	uint16_t adapter_HV_vbus_max;	//mV;
	uint16_t adapter_LV_vbus_max;	//mV;

	uint8_t phandshakeid[4];
	uint8_t mcu_dchg_ic_master_slave_mode[VIVO_DIRECT_CHARGER_IC_COUNT];
	uint8_t mcu_dchg_ic_device_rev[VIVO_DIRECT_CHARGER_IC_COUNT];
	uint8_t mcu_dchg_ic_hw_id;	//setting from AP side.
	uint8_t reserve[5];	//Reverve: use for different project for compatibility
	uint8_t mcu_version;
	uint8_t mcu_dchg_ic_vendor;

	uint8_t mcu_dchg_ic_dump[VIVO_DIRECT_CHARGER_IC_COUNT];	//READ: re-map to dchg_ic_register_dump_value..  Write: AP_TO_MCU_CONTROL_TRIGGER will trigger update.

	/*[Read and reset Register]********************************************/
	uint8_t ap_ctl_mcu_uart_status;			// return the uart_CMD value to indicate the UART RX OK. otherwise [0xF0: unknow_error;   0xF1:UART_TX_error;  0xF2: UART_RX_error;  0xF3: DP_DM_short_error

	/*[Write only Register]********************************************/
	uint16_t ibus_max;	//mA
	uint8_t ap_ctl_mcu_uart_tx_data[30];	// length = 30, {cmd, tx_data_lenght, tx_data1(value1), ...}

	/*[Read only Register]********************************************/
	uint8_t ap_ctl_mcu_uart_rx_data[50];	// length = 50,  {rx_raw_data1(0xAA), rx_raw_data2(uart_cmd_rx_lenght), rx_raw_data3(0x08), rx_raw_data4(cmd), rx_raw_data5(status), ...};  [START]+5=0xB4=value
	uint8_t uart_rx_raw_data;	//Read only: this register will map to the mcu UartxRingBuff[128], in order AP to read adapter RX data

	uint8_t mcu_regerster_buffer_size;	//please make sure this is the last one, and struct size < 0xFF, because We use the 8bits IIC CMD.
};

#define MCU_REGISTER_BUFFERSIZE_SIZE	(sizeof(struct mcu_register_map))

#define MCU_REG__IN_AP_SIDE(x)		((uint8_t )(((uint8_t * )&(chip->mcu.x) - (uint8_t * )&chip->mcu) & 0xFF))
#define MCU_REG__IN_MCU_SIDE(x)		((uint8_t )(((uint8_t * )&(hdchg.map->x) - (uint8_t * )hdchg.map) & 0xFF))
#if defined(AP_SIDE)
	#define MCU_REG__ADDRESS(x)	MCU_REG__IN_AP_SIDE(x)
#else
	#define MCU_REG__ADDRESS(x)	MCU_REG__IN_MCU_SIDE(x)
#endif


#define MCU_REG__NAME(x)		(#x)
#if defined(AP_SIDE)
#define MCU_REG__VALUE(x)		(chip->mcu.x)
#define MCU_REG__DUMP(x)		(#x), (chip->mcu.x)
#else
#define MCU_REG__VALUE(x)		(hdchg.map->x)
#define MCU_REG__DUMP(x)		(#x), (hdchg.map->x)
#endif

#define LOG_D	"%s:%d "
#define LOG_X	"%s:0x%X "

#define LOG_D5	LOG_D LOG_D LOG_D LOG_D LOG_D
#define LOG_D10	LOG_D5 LOG_D5
#define LOG_D20	LOG_D10 LOG_D10

#define LOG_X5	LOG_X LOG_X LOG_X LOG_X LOG_X
#define LOG_X10	LOG_X5 LOG_X5
#define LOG_X20	LOG_X10 LOG_X10
//pr_err("%s:%d\n", MCU_REG__DUMP(IRA));

/*IRQ:*/
//**********************************************************************************
#define MCU_IRQ_A_REG				0x00
#define EVENT_INIT_DONE				BIT(7)
#define EVENT_HANDSHAKE_SUCCESS		BIT(6)
#define EVENT_HANDSHAKE_FAIL		BIT(5)
#define EVENT_PMI_SUSPEND			BIT(4)
#define EVENT_VBAT_GOOD				BIT(3)
#define EVENT_HANDSHAKEID_ERROR		BIT(2)
#define EVENT_POWER_MATCHED			BIT(1)

#define MCU_IRQ_B_REG				0x01
#define EXIT_DCHG					BIT(0)
#define EXIT_HIGH_VBAT				BIT(1)		//After  Dchg, Vbat too high
#define EXIT_HIGH_VBAT_NO_ENTRY_DCHG	BIT(2)	//Before Dchg, Vbat already too high
#define EXIT_NO_PMISUSPEND			BIT(3)
#define EXIT_NO_VBUSMATCHED			BIT(4)
#define EXIT_NO_ADAPTERINFOMATCHED	BIT(5)
#define EXIT_CHG_PROTECTED			BIT(6)		//Dchg IC error, can not enable normally.
#define EXIT_VBAT_REVERSE_BACK		BIT(7)

#define MCU_IRQ_C_REG				0x02
#define EXIT_OTP_ADAPTER			BIT(0)
#define EXIT_OTP_ADAPTER_CONN		BIT(1)
#define EXIT_OTP_USB_CONN			BIT(2)
#define EXIT_OTP_PCB_BTB			BIT(3)
#define EXIT_OTP_BAT_BTB			BIT(4)
#define EXIT_OTP_BAT				BIT(5)
#define EXIT_CABLEROM_HIGH			BIT(6)

#define MCU_IRQ_D_REG				0x03
#define EXIT_AP_CLOSE_MCU			BIT(0)
#define EXIT_VBAT_WEAK				BIT(1)
#define EXIT_VDPDM_HIGH				BIT(2)
#define EXIT_NOT_SANITY				BIT(3)
#define EXIT_UART_ERROR				BIT(4)
#define EXIT_DCHG_IC_I2C_ERROR		BIT(5)
#define EXIT_MCU_IWD				BIT(6)
#define EXIT_MCU_IBUS_HIGH			BIT(7)

#define MCU_REG_04					0x04
#define MCU_PMI_SUSPENDED			BIT(0)
#define MCU_SET_CURRENT				BIT(1)
#define MCU_DCHG_SWITCH_OFF			BIT(2)	//Exit Dchg state machine
#define MCU_SET_AT_BOOT_MODE		BIT(3)
#define MCU_SET_COUT_TRIGGER		BIT(4)	//Battery board have the COUT interrupt to monitor the Vbat>4.5V(or 9V)
#define MCU_SET_USBSEL_ENABLE		BIT(5)
#define MCU_SET_OTG_MODE			BIT(6)
#define MCU_SET_HIGH_VBAT_NO_ENTRY_DCHG			BIT(7)


#define MCU_REG_05					0x05
#define EXIT_ADAPTER_POWER_NO_MATCH	BIT(0)
#define EXIT_ADAPTER_HIGH_VOL_CONFIG_ERROR	BIT(1)
#define EXIT_ADAPTER_IDENTIFICATION_ERROR	BIT(2)
#define EXIT_SYSTEM_LOAD_GREATER_THAN_CHARGING_CURRENT	BIT(3)
#define EXIT_BQ_VBAT_ERROR							BIT(4)
#define EVENT_MAIN_BOARD_CHG_ENABLE_LONG_TIME		BIT(5)
#define EVENT_SLAVE_BOARD_CHG_ENABLE_LONG_TIME		BIT(6)
#define EVENT_ADAPTER_IBUS_LOW						BIT(7)
//**********************************************************************************

//CV data:
#if defined(DCHG_IC_SMB1396)
#define VBAT_SCALE_INIT		   (hdchg.map->hv_div2_charge_bypass_mode == HIGH_VOLTAGE_DIV2_CHARGING__BYPASS__TO_DIRECT_CHARGING ? 102 : 205)
#define VBAT_SCALE_INIT_MAX    (hdchg.map->hv_div2_charge_bypass_mode == HIGH_VOLTAGE_DIV2_CHARGING__BYPASS__TO_DIRECT_CHARGING ? 110 : 220)
#elif defined(DCHG_IC_BQ25890)
#define VBAT_SCALE_INIT		   (hdchg.map->hv_div2_charge_bypass_mode == HIGH_VOLTAGE_DIV2_CHARGING__BYPASS__TO_DIRECT_CHARGING ? 102 : 205)
#define VBAT_SCALE_INIT_MAX    (hdchg.map->hv_div2_charge_bypass_mode == HIGH_VOLTAGE_DIV2_CHARGING__BYPASS__TO_DIRECT_CHARGING ? 110 : 220)
#endif
#define CURRENT_DOWN_STEP		100	//Ibus - 100 mA

/*battery BTB voltage (or get from Charger IC)*/
#define BTB_VBAT_ADC_CV	9200		//bq25980_vbat > 9200;
#define CHG_IC_VBAT_ADC_CV	9270	//bq25790_vbat > 9270

/*FG iC battery voltage*/
#define FG_VBAT_ADC_CV_MAX						9100
#define FG_VBAT_ADC_CV_DEFAULT					9000
#define FG_VBAT_ADC_CV_MIN						8800
#define FG_VBAT_ADC_CV							((MCU_REG__VALUE(termination_vbat) < FG_VBAT_ADC_CV_MIN || MCU_REG__VALUE(termination_vbat) > FG_VBAT_ADC_CV_MAX) ? FG_VBAT_ADC_CV_DEFAULT : MCU_REG__VALUE(termination_vbat))

#define IS_FFC_RUNING							((FG_VBAT_ADC_CV >= 9000) ? 1 : 0)

#define IBAT_CURRENT_EXIT_VALUE_MAX			3000
#define IBAT_CURRENT_EXIT_VALUE_DEFAULT		1600
#define IBAT_CURRENT_EXIT_VALUE_MIN			700
#define IBAT_CURRENT_EXIT_VALUE				((hdchg.map->termination_ibat < IBAT_CURRENT_EXIT_VALUE_MIN) ? IBAT_CURRENT_EXIT_VALUE_MIN : hdchg.map->termination_ibat)
#define IBUS_CURRENT_EXIT_VALUE				(hdchg.map->hv_div2_charge_bypass_mode == HIGH_VOLTAGE_DIV2_CHARGING__BYPASS__TO_DIRECT_CHARGING ? IBAT_CURRENT_EXIT_VALUE : (IBAT_CURRENT_EXIT_VALUE / 2))

#define IS_CV 	((hdchg.map->vbat > BTB_VBAT_ADC_CV) || (hdchg.map->fg_vbat > FG_VBAT_ADC_CV) || (hdchg.map->vcell1 > (FG_VBAT_ADC_CV/2)) || (hdchg.map->vcell2 > (FG_VBAT_ADC_CV/2)) || (hdchg.map->R04 & MCU_SET_COUT_TRIGGER))
#define IS_CV_OFFSET(x) 	((hdchg.map->vbat > (BTB_VBAT_ADC_CV + (x))) || (hdchg.map->fg_vbat > (FG_VBAT_ADC_CV + (x))))


/*========================================[CableR]=============================================start*/
#define CABLE_RESISTANCE_MAX 620	//mO
/*========================================[CableR]=============================================end*/

#define ADAPTER_IBUS_TOO_LOW_VALUE 100	//mA
typedef enum
{
	VBAT_REVERSE_BACK_STATUS__UART_TIMEOUT = BIT(0),	//UART TRx timeout
}VBAT_REVERSE_BACK_STATUS;

typedef enum
{
	MCU_UART_STATUS_UNKNOW_ERROR = 0xF0,
	MCU_UART_STATUS_TX_ERROR = 0xF1,
	MCU_UART_STATUS_RX_ERROR = 0xF2,
	MCU_UART_STATUS_DP_DM_SHORT_ERROR = 0xF3,
}MCU_UART_STATUS;

typedef enum
{
	AP_TO_MCU_CONTROL_NONE = 0,
	AP_TO_MCU_CONTROL_TRIGGER = 0xAA,
	AP_TO_MCU_CONTROL_TRIGGER_DONE = 0xBB,
} AP_TO_MCU_CONTROL__STATUS;

/* State Machine [FSM]--------------------------------------------------------V*/
enum VIVO_DIRECT_CHARGER_PROTOCOL__STATE_MACHINE
{
	CHARGER_INITED = 0x00,
	CHARGER_HANDSHAKING,
	CHARGER_ADAPTER_POWER_MATCH,
	CHARGER_IDENTIFICATION,
	CHARGER_WAIT_CABLE_ID_DETECT,
	CHARGER_HIGH_VOL_MODE_REQUEST,
	CHARGER_DIRECT_CHARGING,

	CHARGER_EXIT_DIRECT_CHARGING,
	CHARGER_HANDSHAKING_FAIL,
	CHARGER_IDLE,
};

enum VIVO_DIRECT_CHARGER_IC__STATE_MACHINE
{
	DCHG_INIT = 0x00,
	DCHG_CHECK_VBAT_GOOD,
	DCHG_WAIT_PMISUSPEND,
	DCHG_REQUEST_VBUS_ADJUST,

	DCHG_RUNNING_BYPASS_TO_SWITCH_IC,
	DCHG_RUNNING,

	DCHG_STOP,
};
/* State Machine [FSM]-------------------------------------------------------A*/

//[VIN/2 - Vout] 
enum {
	VIN_NORMAL = 0,
	VIN_TOO_LOW = 1,
	VIN_TOO_HIGH = 2,

	VIN_TOO_HIGH__CONV_OCP = 3,
};

enum DChg_adapter_Vendor
{
	VENDOR_VIVO,
	VENDOR_DIALOG,
	VENDOR_NXP,
	VENDOR_RICHTEK,
	VENDOR_INJOINIC,
};
	
static const char * const adapter_vendor[] = {
	"VIVO",
	"DIALOG",
	"NXP",
	"RICHTEK",
	"INJOINIC",
};

/*Dchg IC vendor*/
enum MCU_DIRECT_CHARGER_IC_VENDOR{
	DCHG_IC_QCOM_SMB1396 = 0,
	DCHG_IC_TI_BQ25980,
	DCHG_IC_DEBUG,
	DCHG_IC_VENDOR_ERROR,
};

static const char * const DCHG_IC_VENDOR[] = {
	"QCOM_SMB1396",
	"TI_BQ25980",
	"debug",
	"ERROR",
};

enum HIGH_VOLTAGE_DIV2_CHARGING__BYPASS_MODE{
	HIGH_VOLTAGE_DIV2_CHARGING__BYPASS__NONE = 0,				/*High_voltage_Div/2 charging : 2:1 mode*/
	HIGH_VOLTAGE_DIV2_CHARGING__BYPASS__TO_SWITCH_IC = 1,		/*High_voltage_Div/2 charging  --> bypass to  Switch charging*/
	HIGH_VOLTAGE_DIV2_CHARGING__BYPASS__TO_DIRECT_CHARGING = 2,	/*High_voltage_Div/2 charging  --> bypass to  Direct charging  1:1 moode*/
};

static const char * const dchg_bypass_mode[] = {
	"No bypass -> HV_Div 2:1mode",
	"bypass to switch IC",
	"bypass to direct_charging 1:1mode",
};

#endif /* __STM32L011_COMMON */
