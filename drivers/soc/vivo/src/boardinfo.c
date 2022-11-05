/*
 * Copyright (C) 2016 vivo Co., Ltd.
 * YangChun <yangchun@vivo.com.cn>
 *
 * This driver is used to export hardware board information for users
 *
**/

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/sys_soc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <soc/qcom/socinfo.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/soc/qcom/smem.h>
#define BOARD_REV_LEN 16
#define BOARD_NAME_LEN 24
#define VIVO_VENDOR_LEN 8
#define VIVO_CPU_TYPE_LEN 10
#define FREQ_STR_LEN 8
#define INVALID_CPU_FREQ "0"
#define INVALID_CPU_TYPE "unkown"
#define CPU_REVISION_ADDR 0x000A607C
#define VIVO_HW_VERSION_MASK (0xF<<28)
#define VIVO_PM_STATUS_LEN 32

//store in shared memory
struct boardinfo_smem{
	uint32_t board_id;
	char board_rev[BOARD_REV_LEN];
	uint32_t size;
	uint32_t type;
	char board_name[BOARD_NAME_LEN];
	uint32_t ddr_manufacture_id;
	uint32_t lcm_id;
	uint32_t dp_status;	/*add wuyuanhan, dp image load or not.*/
	char pmic_status[VIVO_PM_STATUS_LEN];
	uint32_t reserved;//make len as a multiple of 8
} *boardinfo_smem;

struct boardinfo_ext{
	char vendor[VIVO_VENDOR_LEN];
	unsigned int cpu_freq;
	char cpu_type[VIVO_CPU_TYPE_LEN];
    char user_cpu_freq[FREQ_STR_LEN];				// max cpu_freq|string|unit GHz|for setting app display to user... 2018/10/25  wuyuanhan, 
	unsigned int core_num;
} *boardinfo_ext;

typedef struct freq_base_map {
	uint32_t board_id;
	uint32_t act_freq;
	char user_freq[FREQ_STR_LEN];
    char cpu_type[VIVO_CPU_TYPE_LEN];
} freq_base_map_t;

#if 0

//ά??ԭ??:
//1???û???ʾCPUƵ??��Դ??cpuоƬ?ֲ? /sys/bus/soc/devices/soc1/user_cpu_freq????λGHz.
//2??ʵ??CPU????Ƶ?ʣ???/sys/bus/cpu/devices/cpu7/cpufreq/cpuinfo_max_freq & /sys/bus/soc/devices/soc1/cpu_freq Ϊ׼
//3??freq_maps ӳ????֧??һ??boardid??ʹ?ò?ͬƵ?ʲ?ͬCPU????????
//4??ÿ??Boarid ????Ҫ???ã???Ȼ??ȡ????CPUƵ??Ϊ0??CPU?ͺ?Ϊ unkonw.
//5??map??ƥ??ԭ??:
//   5.1 ƥ??boardid??δ?ҵ? ??????ЧƵ??"0"
//   5.2 ֻƥ?䵽boardid??δ???õ?ʵ?ʹ???Ƶ??cpu,??ȡĬ????ʾcpuƵ?ʣ?CPU?ͺ???Ϣ
//   5.3 ƥ?䵽boardid&???õ?ʵ?ʹ???Ƶ??cpu,??Ӧʵ??Ƶ??cpuƵ?ʵ???ʾCPUƵ?ʣ?CPU?ͺ???Ϣ
//
///* -----------------  1.  ??ƽ̨?漰????????CPU?ֲ? CPU????Ƶ????Ϣ                   ----------------------- */
//sm6150:
//2xKryo 360 Gold 2.0GHz
//
//sm7150:
//2xKryo 360 Gold 2.2GHz
//
///* -----------------  2.  ??ƽ̨xbl/sbl?? board_id ??Ϣ??????????board_idʱ????Ҫͬ??????  ------------------ */
//static vivo_board_id_t board_ids[] = {
//	{0,"SDM670",},
//	{1,"PD1832F_EX",},
//

#endif

static freq_base_map_t freq_maps[] = {
	{0, 0, "2.84", "865"},
	{1, 0, "2.84", "865"},//TD1906
	{2, 0, "2.84", "865"},//PD1950
	{3, 0, "2.84", "865"},//PD1955
	{4, 0, "2.84", "865"},//PD1950F_EX
	{5, 0, "2.84", "865"},//PD1955F_EX
	{6, 0, "2.84", "865"},//EXP1933
	{7, 0, "2.84", "865"},//ETD1913
	{10, 0, "2.84", "865"},//ETD1921
	{11, 0, "2.84", "865"},//PD1981
	{12, 0, "2.84", "865"},//PD1981F_EX
	{13, 0, "2.84", "865"},//PD2011
	{14, 0, "2.84", "865"},//PD2024
	{15, 0, "2.84", "865"},//PD2024F_EX
	{16, 0, "2.84", "865"},//PD2025
	{17, 0, "2.84", "865"},//PD2025F_EX
	{18, 0, "2.84", "865"},//PD2059
	{19, 0, "3.2", "870"},//PD2055
	{20, 0, "3.2", "870"},//PD2046F_EX
	{21, 0, "3.2", "870"},//PD2059F_EX
	{128, 0, "2.4", "765G"},//SM7250
	{129, 0, "2.4", "765G"},//PD1963      SM7250
	{130, 0, "2.4", "765G"},//PD2001      SM7250
	{131, 0, "2.4", "765G"},//PD2001F_EX  SM7250
	{132, 0, "2.4", "765G"},//PD2005      SM7250
	{133, 0, "2.4", "765G"},//PD2005F_EX  SM7250
	{134, 0, "2.4", "765G"},//PD2012      SM7250
	{135, 0, "2.4", "765G"},//PD2020      SM7250
	{136, 0, "2.4", "765G"},//PD2020F_EX  SM7250
	{164, 0, "2.4", "765G"},//PD2005 -b    SM7250
	{165, 0, "2.4", "765G"},//PD2005F_EX -b SM7250
	{138, 0, "2.8", "768G"},//PD2043  SM7250_AC
	{139, 0, "2.8", "768G"},//PD2073  SM7250_AC
	{140, 0, "2.8", "768G"},//PD2077  SM7250_AC
	{141, 0, "2.8", "768G"},//PD2073F SM7250_AC
};

static char *hwid = "12345678";
static char hwid_buff[40] = {
	0x6e, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x03,\
	0x12, 0x34, 0x56, 0x78,\
	0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00,\
};
static ssize_t vivo_show_hwid(struct device *dev, struct device_attribute *attr, char *buf)
{
	snprintf(hwid_buff + 8, 32, "%s", hwid);
	memcpy(buf, hwid_buff, sizeof(hwid_buff));
	return sizeof(hwid_buff);
}

static ssize_t vivo_show_board_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", boardinfo_smem->board_id);
}

static ssize_t vivo_show_board_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_smem->board_name);
}
static ssize_t vivo_show_ddr_manufacture_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", boardinfo_smem->ddr_manufacture_id);
}
static ssize_t vivo_show_vendor(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->vendor);
}
static ssize_t vivo_show_cpu_freq(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", boardinfo_ext->cpu_freq);
}
static ssize_t vivo_show_cpu_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->cpu_type);
}
static ssize_t vivo_show_user_cpu_freq(struct device *dev, struct device_attribute *attr, char *buf)
{ 
    return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->user_cpu_freq);
}

static ssize_t vivo_show_core_num(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", boardinfo_ext->core_num);
}

static struct device_attribute vivo_vendor = __ATTR(vendor, S_IRUGO, vivo_show_vendor,  NULL);

static struct device_attribute vivo_board_id = __ATTR(board_id, S_IRUGO, vivo_show_board_id, NULL);

static struct device_attribute vivo_board_name = __ATTR(board_name, S_IRUGO, vivo_show_board_name, NULL);

static struct device_attribute vivo_ddrinfo = __ATTR(ddrinfo, S_IRUGO, vivo_show_ddr_manufacture_id,  NULL);

static struct device_attribute vivo_cpu_freq = __ATTR(cpu_freq, S_IRUGO, vivo_show_cpu_freq, NULL);

static struct device_attribute vivo_cpu_user_freq = __ATTR(user_cpu_freq, S_IRUGO, vivo_show_user_cpu_freq, NULL);

static struct device_attribute vivo_cpu_type = __ATTR(cpu_type, S_IRUGO, vivo_show_cpu_type, NULL);	

static struct device_attribute vivo_core_num = __ATTR(core_num, S_IRUGO, vivo_show_core_num, NULL);

static struct device_attribute vivo_hwid = __ATTR(hwid, S_IRUGO, vivo_show_hwid, NULL);

static void __init populate_soc_sysfs_files(struct device *vivo_soc_device)
{
	device_create_file(vivo_soc_device, &vivo_board_id);
	device_create_file(vivo_soc_device, &vivo_board_name);
	device_create_file(vivo_soc_device, &vivo_vendor);
	device_create_file(vivo_soc_device, &vivo_ddrinfo);
	device_create_file(vivo_soc_device, &vivo_cpu_freq);
    device_create_file(vivo_soc_device, &vivo_cpu_user_freq);
	device_create_file(vivo_soc_device, &vivo_cpu_type);
	device_create_file(vivo_soc_device, &vivo_core_num);
	device_create_file(vivo_soc_device, &vivo_hwid);
	return;
}
#if 0
static int vivo_get_max_freq(unsigned int cpu_id)
{
	int max_freq = 0;
	int cur_freq = 0;
	int i = 0 ;
	struct cpufreq_frequency_table *table = NULL;
	struct cpufreq_policy *policy = NULL;
	policy = cpufreq_cpu_get(cpu_id);
	if (policy == NULL) {
		return 0;
	}
	table = cpufreq_frequency_get_table(policy->cpu);
	cpufreq_cpu_put(policy);
	if (table == NULL) {
		pr_err("vivo get frequency of CPU%u fail\n", cpu_id);
		return 0;
	}
	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		cur_freq = table[i].frequency;
		if (cur_freq == CPUFREQ_ENTRY_INVALID)
			continue;
		if (cur_freq > max_freq) {
			max_freq = cur_freq;
		}
		//pr_err("CPU%u:table[%d]=%d\n",cpu_id,i,cur_freq);
	}
	return max_freq;
}


static unsigned int vivo_get_cpu_freq(void)
{
	int cpu_id = 0;
	unsigned int max_freq = 0;
	unsigned int cur_freq = 0;
	int num_cpus = num_possible_cpus();
	for (cpu_id = 0; cpu_id < num_cpus; cpu_id++) {
		if (cpu_online(cpu_id)) {
			cur_freq = vivo_get_max_freq(cpu_id);
			if (cur_freq > max_freq) {
				max_freq = cur_freq;
			}
		}
	}
	
	return max_freq;
}
#endif
static void get_user_cpu_freq_and_type(void)
{
    
	int i = 0;
	int default_index = -1;
 
	for (i = 0; i < (sizeof (freq_maps) / sizeof (freq_maps[0])); i++) {
		if (freq_maps[i].board_id == boardinfo_smem->board_id) {
			if ((freq_maps[i].act_freq == boardinfo_ext->cpu_freq) && (freq_maps[i].act_freq != 0)) {				
				pr_err("vivo board_info:Set user cpu max freq : %sGHz\n", freq_maps[i].user_freq); 
				strlcpy(boardinfo_ext->user_cpu_freq, freq_maps[i].user_freq, FREQ_STR_LEN);
				strlcpy(boardinfo_ext->cpu_type, freq_maps[i].cpu_type, VIVO_CPU_TYPE_LEN);
				return;
			} else if (freq_maps[i].act_freq == 0) {
			    default_index = i;
			}
		}
	}
	
	if (default_index >= 0) {
		pr_err("vivo board_info:Set user cpu max freq : %sGHz\n", freq_maps[i].user_freq);
		strlcpy(boardinfo_ext->user_cpu_freq, freq_maps[default_index].user_freq, VIVO_CPU_TYPE_LEN);
		strlcpy(boardinfo_ext->cpu_type, freq_maps[default_index].cpu_type, VIVO_CPU_TYPE_LEN);
		return;
	} else {
		strlcpy(boardinfo_ext->user_cpu_freq, INVALID_CPU_FREQ, FREQ_STR_LEN);
		strlcpy(boardinfo_ext->cpu_type, INVALID_CPU_TYPE, VIVO_CPU_TYPE_LEN);
		pr_err("vivo board_info: error: Need to set cpu max freq for user!!!\n"); 
		return;
	}
}

/*add by hkc begin*/
char *vivo_get_pmic_status(void)
{
	printk("PM OCP check: %s\n", boardinfo_smem->pmic_status);
	return boardinfo_smem->pmic_status;
}
EXPORT_SYMBOL(vivo_get_pmic_status);
/*add by hkc end*/

static void vivo_boardinfo_ext_init(void)
{
	
	boardinfo_ext = kzalloc(sizeof(*boardinfo_ext), GFP_KERNEL);
	if (!boardinfo_ext) {
		pr_err("boardinfo_ext alloc failed!\n");
		return;
	}
    //cpu max frequency
    boardinfo_ext->cpu_freq = cpuinfo_max_freq_cached;//vivo_get_cpu_freq();
    
	//user frequency & type of cpu
	get_user_cpu_freq_and_type();
	//core number
	boardinfo_ext->core_num = num_possible_cpus();
	
	//vendor
	strlcpy(boardinfo_ext->vendor, "vivo", VIVO_VENDOR_LEN); //vivo
	pr_err("vivo cpu_freq:%u user_cpu_freq:%sGHz core_num=%u cpu_type=%s\n", boardinfo_ext->cpu_freq,
			boardinfo_ext->user_cpu_freq,
			boardinfo_ext->core_num,
			boardinfo_ext->cpu_type);
}
static int __init vivo_boardinfo_init_sysfs(void)
{
	struct device *vivo_soc_device;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;

	if (!boardinfo_smem) {
		pr_err("No boardinfo found!\n");
		return -ENODEV;
	}

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		pr_err("Soc Device alloc failed!\n");
		return -ENOMEM;
	}

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR_OR_NULL(soc_dev)) {
		kfree(soc_dev_attr);
		pr_err("Soc device register failed\n");
		return -EIO;
	}
		
	vivo_soc_device = soc_device_to_device(soc_dev);
	
	populate_soc_sysfs_files(vivo_soc_device);
	
	//extra information init
	vivo_boardinfo_ext_init();
	
	return 0;
}
late_initcall(vivo_boardinfo_init_sysfs);

char *vivo_get_projeck_name(void)
{
	printk("vivo_get_projeck_name: %s\n", boardinfo_smem->board_name);
	return boardinfo_smem->board_name;
}

static void vivo_boardinfo_print(void)
{
	pr_info("board_id=%d, board_version=%s, type=%d, board_name:%s\n",
		boardinfo_smem->board_id, boardinfo_smem->board_rev, 
		boardinfo_smem->type, boardinfo_smem->board_name);
}
#define SMEM_ID_VENDOR0 134
int __init vivo_boardinfo_init(void)
{
	static bool boardinfo_init_done;
	size_t  size;
	if (boardinfo_init_done)
		return 0;

	boardinfo_smem = (struct boardinfo_smem *)qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_ID_VENDOR0, &size);

	if (IS_ERR_OR_NULL(boardinfo_smem))
		BUG_ON("Can't find SMEM_ID_VENDOR0 for vivo boardinfo!\n");

	vivo_boardinfo_print();
	vivo_get_pmic_status();
	boardinfo_init_done = true;

	hwid = boardinfo_smem->board_rev;

	return 0;
}
subsys_initcall(vivo_boardinfo_init);
