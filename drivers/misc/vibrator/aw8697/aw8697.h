#ifndef _AW8697_H_
#define _AW8697_H_

/*********************************************************
 *
 * aw8697.h
 *
 ********************************************************/
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/leds.h>

#include <linux/wakelock.h>


/*********************************************************
 *
 * marco
 *
 ********************************************************/
#define AW8697_CHIPID                   0x97

#define MAX_I2C_BUFFER_SIZE                 65536 //no use

#define AW8697_SEQUENCER_SIZE               8
#define AW8697_SEQUENCER_LOOP_SIZE          4

#define AW8697_RTP_I2C_SINGLE_MAX_NUM       512 //no use

#define HAPTIC_MAX_TIMEOUT                  10000 //no use

#define AW8697_VBAT_REFER                   4200
#define AW8697_VBAT_MIN                     3000
#define AW8697_VBAT_MAX                     4500

/* trig config */
#define AW8697_TRIG_NUM                     3
#define AW8697_TRG1_ENABLE                  0
#define AW8697_TRG2_ENABLE                  0
#define AW8697_TRG3_ENABLE                  1

/*
 * trig default high level
 * ___________           _________________
 *           |           |
 *           |           |
 *           |___________|
 *        first edge
 *                   second edge
 *
 *
 * trig default low level
 *            ___________
 *           |           |
 *           |           |
 * __________|           |_________________
 *        first edge
 *                   second edge
 */
#define AW8697_TRG1_DEFAULT_LEVEL           1       // 1: high level; 0: low level
#define AW8697_TRG2_DEFAULT_LEVEL           1       // 1: high level; 0: low level
#define AW8697_TRG3_DEFAULT_LEVEL           1       // 1: high level; 0: low level

#define AW8697_TRG1_DUAL_EDGE               1       // 1: dual edge; 0: first edge
#define AW8697_TRG2_DUAL_EDGE               1       // 1: dual edge; 0: first edge
#define AW8697_TRG3_DUAL_EDGE               1       // 1: dual edge; 0: first edge

#define AW8697_TRG1_FIRST_EDGE_SEQ          1       // trig1: first edge waveform seq
#define AW8697_TRG1_SECOND_EDGE_SEQ         1       // trig1: second edge waveform seq
#define AW8697_TRG2_FIRST_EDGE_SEQ          1       // trig2: first edge waveform seq
#define AW8697_TRG2_SECOND_EDGE_SEQ         1       // trig2: second edge waveform seq
#define AW8697_TRG3_FIRST_EDGE_SEQ          1       // trig3: first edge waveform seq
#define AW8697_TRG3_SECOND_EDGE_SEQ         1       // trig3: second edge waveform seq


#if AW8697_TRG1_ENABLE
#define AW8697_TRG1_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG1_ENABLE
#else
#define AW8697_TRG1_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG1_DISABLE
#endif

#if AW8697_TRG2_ENABLE
#define AW8697_TRG2_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG2_ENABLE
#else
#define AW8697_TRG2_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG2_DISABLE
#endif

#if AW8697_TRG3_ENABLE
#define AW8697_TRG3_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG3_ENABLE
#else
#define AW8697_TRG3_DEFAULT_ENABLE          AW8697_BIT_TRGCFG2_TRG3_DISABLE
#endif

#if AW8697_TRG1_DEFAULT_LEVEL
#define AW8697_TRG1_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG1_POLAR_POS
#else
#define AW8697_TRG1_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG1_POLAR_NEG
#endif

#if AW8697_TRG2_DEFAULT_LEVEL
#define AW8697_TRG2_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG2_POLAR_POS
#else
#define AW8697_TRG2_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG2_POLAR_NEG
#endif

#if AW8697_TRG3_DEFAULT_LEVEL
#define AW8697_TRG3_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG3_POLAR_POS
#else
#define AW8697_TRG3_DEFAULT_POLAR           AW8697_BIT_TRGCFG1_TRG3_POLAR_NEG
#endif

#if AW8697_TRG1_DUAL_EDGE
#define AW8697_TRG1_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG1_EDGE_POS_NEG
#else
#define AW8697_TRG1_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG1_EDGE_POS
#endif

#if AW8697_TRG2_DUAL_EDGE
#define AW8697_TRG2_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG2_EDGE_POS_NEG
#else
#define AW8697_TRG2_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG2_EDGE_POS
#endif

#if AW8697_TRG3_DUAL_EDGE
#define AW8697_TRG3_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG3_EDGE_POS_NEG
#else
#define AW8697_TRG3_DEFAULT_EDGE            AW8697_BIT_TRGCFG1_TRG3_EDGE_POS
#endif

/***************************************************************************************
 *
 *                             macro vivo
 *
  ***************************************************************************************/

#define AW8697_BSTCFG_PEAKCUR_LIMIT     0x07
#define AW8697_DEFAULT_PEAKCUR          AW8697_BIT_BSTCFG_PEAKCUR_3P5A

#define AW8697_CONT_PLAYBACK_MODE       AW8697_BIT_CONT_CTRL_CLOSE_PLAYBACK

#define AW8697_HAPTIC_F0_COEFF              260     //2.604167

/* motor config */
#define AW8697_LRA_0619                    619
#define AW8697_LRA_0832                    832
#define AW8697_LRA_1040                    1040
#define AW8697_LRA_0815                    815

#define DEFAULT_CALI_F0 0
#define DEFAULT_OSC_CALI_DATA 0
#define DEFAULT_OSC_CALI_TIMES_US 5000000

#define BOUNDARY_SEQ_WAIT_US  162000 //ms
#define MAX_SEQ_WAIT_US  1270000 //ms
#define SEQ_WAIT_UNIT    1280 //us
#define AW8697_DUOBLE_CLICK_DELTA      30000 //us
#define AW8697_QUADRA_CLICK_DELTA      50000 //us


/*****************************************************************************************
 *
 *                        enum type
 *
 *****************************************************************************************/
enum aw8697_flags {
	AW8697_FLAG_NONR = 0,
	AW8697_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw8697_haptic_read_write {
	AW8697_HAPTIC_CMD_READ_REG = 0,
	AW8697_HAPTIC_CMD_WRITE_REG = 1,
};


enum aw8697_haptic_work_mode {
	AW8697_HAPTIC_STANDBY_MODE = 0,
	AW8697_HAPTIC_RAM_MODE = 1,
	AW8697_HAPTIC_RTP_MODE = 2,
	AW8697_HAPTIC_TRIG_MODE = 3,
	AW8697_HAPTIC_CONT_MODE = 4,
	AW8697_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw8697_haptic_bst_mode {
	AW8697_HAPTIC_BYPASS_MODE = 0,
	AW8697_HAPTIC_BOOST_MODE = 1,
};

enum aw8697_haptic_activate_mode {
  AW8697_HAPTIC_ACTIVATE_RAM_MODE = 0,
  AW8697_HAPTIC_ACTIVATE_CONT_MODE = 1,
};


enum aw8697_haptic_cont_vbat_comp_mode {
	AW8697_HAPTIC_CONT_VBAT_SW_COMP_MODE = 0,
	AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE = 1,
};

enum aw8697_haptic_ram_vbat_comp_mode {
	AW8697_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
	AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw8697_haptic_f0_flag {
	AW8697_HAPTIC_LRA_F0 = 0,
	AW8697_HAPTIC_CALI_F0 = 1,
};

enum aw8697_haptic_pwm_mode {
	AW8697_PWM_48K = 0,
	AW8697_PWM_24K = 1,
	AW8697_PWM_12K = 2,
};

/************************* audio with game **************************/
enum aw8697_haptic_play {
	AW8697_HAPTIC_PLAY_NULL = 0,
	AW8697_HAPTIC_PLAY_ENABLE = 1,
	AW8697_HAPTIC_PLAY_STOP = 2,
	AW8697_HAPTIC_PLAY_GAIN = 8,
};

enum aw8697_haptic_cmd {
	AW8697_HAPTIC_CMD_NULL = 0,
	AW8697_HAPTIC_CMD_ENABLE = 1,
	AW8697_HAPTIC_CMD_HAPTIC = 0x0f,
	AW8697_HAPTIC_CMD_TP = 0x10,
	AW8697_HAPTIC_CMD_SYS = 0xf0,
	AW8697_HAPTIC_CMD_STOP = 255,
};

enum aw8697_haptic_cali_lra {
	AW8697_HAPTIC_F0_CALI_LRA = 1,
	AW8697_HAPTIC_RTP_CALI_LRA = 2,
};
enum cali_result {
	CALI_NARMAL = 0,
	CALI_NO_NEED = 1,
	CALI_OVER_RANGE = 2,

};

enum play_type {
	RAM_TYPE = 0,
	RTP_TYPE = 1,
	TIME_TYPE = 2,
	CONT_TYPE = 3,
};

enum aw8697_haptic_clock_type {
	AW8697_HAPTIC_CLOCK_CALI_F0 = 0,
	AW8697_HAPTIC_CLOCK_CALI_OSC_STANDARD = 1,

};

enum haptics_custom_effect_param {
	CUSTOM_DATA_EFFECT_IDX,
	CUSTOM_DATA_TIMEOUT_SEC_IDX,
	CUSTOM_DATA_TIMEOUT_MSEC_IDX,
	CUSTOM_DATA_LEN,
};

/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct lra_info_for_cali {
	u32 AW8697_HAPTIC_F0_PRE;  // 170Hz
	u32 AW8697_HAPTIC_F0_CALI_PERCEN;      // -7%~7%
	u32 AW8697_HAPTIC_CONT_DRV_LVL;   // 71*6.1/256=1.69v
	u32 AW8697_HAPTIC_CONT_DRV_LVL_OV;    // 125*6.1/256=2.98v
	u32 AW8697_HAPTIC_CONT_TD;
	u32 AW8697_HAPTIC_CONT_ZC_THR;
	u32 AW8697_HAPTIC_CONT_NUM_BRK;
	u32 AW8697_HAPTIC_RATED_VOLTAGE; //mv-Vp
 };

struct aw8697_play_info {
	enum play_type type;
	u32 ram_id;//用于ram模式
	u32 vmax;
	u32 times_ms;//用于返回ram和rtp模式波形时长
	u32 playLength;//用于时长播放
	char rtp_file[128];//用于rtp模式
};

//effect scene struct
struct scene_effect_info {
	u16		scene_id;
	u16		effect_id;
	u16		real_vmax;
};

//数据类型用u32，与dts的数据类型保持一致
struct aw8697_wavefrom_info {
	u32 idx;
	u32 ram_id;
	u32 vmax;
	u32 times_ms;
	bool rtp_enable;
	const char *rtp_file_name;
};
struct fileops {
	unsigned char cmd;
	unsigned char reg;
	unsigned char ram_addrh;
	unsigned char ram_addrl;
};

struct ram {
	unsigned int len;
	unsigned int check_sum;
	unsigned int base_addr;
	unsigned char version;
	unsigned char ram_shift;
	unsigned char baseaddr_shift;
};

struct haptic_ctr{
	unsigned char cnt;
	unsigned char cmd;
	unsigned char play;
	unsigned char wavseq;
	unsigned char loop;
	unsigned char gain;
	struct list_head list;
};

struct haptic_audio{
	struct mutex lock;
	spinlock_t list_lock;
	struct hrtimer timer;
	struct work_struct work;
	int delay_val;
	int timer_val;
	unsigned char cnt;
	struct haptic_ctr data[256];
	struct haptic_ctr ctr;
	struct list_head ctr_list;
	struct list_head list;
    bool haptic_audio_cancel_flag;
};

struct trig{
	unsigned char enable;
	unsigned char default_level;
	unsigned char dual_edge;
	unsigned char frist_seq;
	unsigned char second_seq;
};

struct aw8697_dts_info {
	unsigned int mode;
	unsigned int f0_pre;
	unsigned int f0_cali_percen;
	unsigned int cont_drv_lvl;
	unsigned int cont_drv_lvl_ov;
	unsigned int cont_td;
	unsigned int cont_zc_thr;
	unsigned int cont_num_brk;
	unsigned int f0_coeff;
	unsigned int f0_trace_parameter[4];
	unsigned int bemf_config[4];
	unsigned int sw_brake;
	unsigned int tset;
	unsigned int r_spare;
	unsigned int bstdbg[6];
	unsigned int parameter1;
};

struct aw8697 {
    struct kobject kobjectDebug;
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;

	struct mutex lock;
	struct mutex rtp_lock;
	struct hrtimer timer;
	struct work_struct vibrator_work;
	struct work_struct rtp_work;
	struct work_struct init_setting_work;
	struct delayed_work ram_work;
	struct led_classdev cdev;

	struct fileops fileops;
	struct ram ram;


	ktime_t start, end;
	unsigned int timeval_flags;
	unsigned int osc_cali_flag;
	s64 microsecond;
	unsigned int sys_frequency;
	unsigned int rtp_len;

	int reset_gpio;
	int irq_gpio;


	unsigned char hwen_flag;
	unsigned char flags;
	unsigned char chipid;

	unsigned char play_mode;

	unsigned char activate_mode;

	unsigned char auto_boost;

	int state;
	int duration;
	int amplitude;
	int index;
	int vmax;
	int gain;

	unsigned char seq[AW8697_SEQUENCER_SIZE];
	unsigned char loop[AW8697_SEQUENCER_SIZE];

	unsigned int rtp_cnt;
	unsigned int rtp_file_num;

	volatile unsigned char rtp_init;
	unsigned char ram_init;

	unsigned int f0;
	unsigned int f0_pre;
	unsigned int cont_f0;
	unsigned int cont_td;
	unsigned int cont_zc_thr;
	unsigned char cont_drv_lvl;
	unsigned char cont_drv_lvl_ov;
	unsigned char cont_num_brk;
	unsigned char max_pos_beme;
	unsigned char max_neg_beme;
	unsigned char f0_cali_flag;

	unsigned char ram_vbat_comp;
	unsigned int vbat;
	unsigned int lra;

	struct trig trig[AW8697_TRIG_NUM];

	struct haptic_audio haptic_audio;
	struct aw8697_dts_info info;
	unsigned int effect;
	unsigned int effect_strength;

	struct aw8697_play_info	play;

	struct mutex bus_lock;
	struct mutex rtp_check_lock;
	struct wake_lock wake_lock;

	bool ff_play_finish;

	atomic_t standard_osc_freq_cali; //标准的osc时钟频率，osc校准获取
	atomic_t f0_freq_cali; //校准f0时获取的时钟频率

	// Play and stop time interval
	ktime_t begin;
	ktime_t cancel;

// DTS Information

	u32 resistance_min;
	u32 resistance_max;
	u32 freq_min;
	u32 freq_max;

	u32 lra_information;  //板子马达型号
	struct lra_info_for_cali lra_info; //板子马达校准配置

	bool no_trigger;
	int trigger_gain;
	bool disable_trigger_force; //setting trgger swtich close
	struct mutex trigger_en_lock;
	bool trigger_en_status;
	bool add_suffix;

	// dts effect
	int effects_count;
	struct aw8697_wavefrom_info *effect_list;
	int default_vmax;

	struct scene_effect_info *base_scene_list;
	int base_scene_count;
	struct scene_effect_info *ext_scene_list;
	int ext_scene_count;

};

struct aw8697_container{
	unsigned int len;
	unsigned char data[];
};




/*********************************************************
 *
 * ioctl
 *
 ********************************************************/
struct aw8697_seq_loop {
	unsigned char loop[AW8697_SEQUENCER_SIZE];
};

struct aw8697_que_seq {
	unsigned char index[AW8697_SEQUENCER_SIZE];
};


#define AW8697_HAPTIC_IOCTL_MAGIC         'h'

#define AW8697_HAPTIC_SET_QUE_SEQ         _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 1, struct aw8697_que_seq*)
#define AW8697_HAPTIC_SET_SEQ_LOOP        _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 2, struct aw8697_seq_loop*)
#define AW8697_HAPTIC_PLAY_QUE_SEQ        _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 3, unsigned int)
#define AW8697_HAPTIC_SET_BST_VOL         _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 4, unsigned int)
#define AW8697_HAPTIC_SET_BST_PEAK_CUR    _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 5, unsigned int)
#define AW8697_HAPTIC_SET_GAIN            _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 6, unsigned int)
#define AW8697_HAPTIC_PLAY_REPEAT_SEQ     _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 7, unsigned int)


#endif

