#ifndef __AW8697_CONFIG_H__
#define __AW8697_CONFIG_H__

#define AW8697_CHIPID                   0x97

#define AW8697_BSTCFG_PEAKCUR_LIMIT     0x07
#define AW8697_DEFAULT_PEAKCUR          AW8697_BIT_BSTCFG_PEAKCUR_3P5A

#define AW8697_CONT_PLAYBACK_MODE       AW8697_BIT_CONT_CTRL_CLOSE_PLAYBACK

#define AW8697_RTP_NAME_MAX        64

#define AW8697_DUOBLE_CLICK_DELTA      30000 //us
#define AW8697_QUADRA_CLICK_DELTA      50000 //us
//数据类型用u32，与dts的数据类型保持一致
struct aw8697_wavefrom_ram_info {
	u32 idx;
	u32 ram_id;
	u32 vmax;
	u32 times_us; //us
};

struct aw8697_wavefrom_rtp_info {
	u32 idx;
	//char rtp_name[AW8697_RTP_NAME_MAX];
	const char *rtp_name;
	u32 vmax;
	u32 times_ms;
};

static struct aw8697_wavefrom_ram_info waveform_ram_table_default[1] = {
/* 原生 CLICK、DOUBLE CLICK、TICK、THUD、POP、HEAVY CLICK */
	{0, 1, 8000, 12000},
};

static struct aw8697_wavefrom_rtp_info waveform_rtp_table_default[1] = {
	/* osc校准文件 */
	{201, "aw8697_rtp.bin", 8000, 5000},
};


#endif
