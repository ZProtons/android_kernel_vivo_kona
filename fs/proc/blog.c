#if !defined(VIVO_PROJECT_MODEL)
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/sched/clock.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/reboot.h>
#include <linux/moduleparam.h>

#include "blog.h"
#include "steplog.h"


//#define STEPLOG_DEBUG
#ifdef STEPLOG_DEBUG
	#define blog_dbg pr_info
#else
	static int blog_dbg_enable;
	#define blog_dbg(fmt, ...) do { \
		if (blog_dbg_enable) \
			pr_info(pr_fmt(fmt), ##__VA_ARGS__); \
		else \
			pr_debug(pr_fmt(fmt), ##__VA_ARGS__); \
	} while (0)
	module_param(blog_dbg_enable, int, 0600);
#endif

#define BATCH_SIZE                  4096
#define BOOT_STEP_TIMEOUT           300 // S

#define MODULE_NAME_LEN             64
#define STEPLOG_STR_MAX_LEN         (MODULE_NAME_LEN + 10) /* module_name 4095 post */

#define ARD_MODULE_INFO_BYTE_SIZE   (SL_STAGE2_SIZE * MODULE_NAME_LEN)
#define ARD_MODULE_INFO_NUM         SL_STAGE2_SIZE

#define BLOG_PARTITION              "logdump"
#define BLOG_DEV                    "/dev/block/by-name/logdump"


struct step_buffer {
	char *start;
	unsigned int size;
	unsigned int wp;
	unsigned int rp;
	unsigned int ring_offset;
	spinlock_t lock;
};

struct blog_header blog_hdr;

static char kstep_buffer[BLOG_KERNEL_STEP_BYTE_SIZE];
static struct step_buffer kstep = {
	.start = kstep_buffer,
	.size = BLOG_KERNEL_STEP_BYTE_SIZE,
	.wp = 0,
	.rp = 0,
	.ring_offset = BLOG_KERNEL_STEP_BYTE_SIZE / 2,
	.lock = __SPIN_LOCK_UNLOCKED(kstep.lock),
};

static struct step_buffer ustep = {
	.size = BLOG_USER_STEP_BYTE_SIZE,
	.wp = 0,
	.rp = 0,
	.ring_offset = BLOG_USER_STEP_BYTE_SIZE / 2,
	.lock = __SPIN_LOCK_UNLOCKED(ustep.lock),
};

static char *module_list;
static DEFINE_SPINLOCK(ard_module_info_lock);
static const char *const state2str[] = {"none", "pre", "post"};
static int boot_complete;
static int boot_timeout;
static int reboot_start;
extern int kernel_init_done;
extern unsigned int recoverymode;
extern unsigned int power_off_charging_mode;
extern unsigned int is_atboot;


#ifdef VIVO_STEP_LOG
void save_step_log(unsigned int stage1, unsigned int stage2, unsigned int step, unsigned int state)
{}
#endif

static u32 get_blog_hdr_crc(struct blog_header *hdr)
{
	int i;
	u32 offset, sum = 0, *ptr = NULL, size = 0;

	offset = (u32)((u32 *)(&hdr->crc) + 1) - (u32)hdr;
	ptr = (u32 *)((u8 *)hdr + offset);
	size = sizeof(struct blog_header) - offset;

	for (i = 0; i < size / sizeof(u32); i++)
		sum += ptr[i];

	return sum;
}

#ifdef BLOG_MAKE_HDR_IN_KERNEL
static void blog_info_init(struct blog_info *log, u32 type, u32 offset, u32 size)
{
	log->count = (u32)-1;
	log->boot_complete = 0;
	log->stage[type].start = offset;
	log->stage[type].size = size;
	log->stage[type].ring_offset = size / 2;
	log->stage[type].wptr = 0;
}

static int blog_header_init(void)
{
	int i;
	u32 offset, size;

	blog_hdr.magic = BLOG_MAGIC_NUM;
	blog_hdr.version = BLOG_VERSION;
	blog_hdr.count = (u32)-1;
	blog_hdr.flush_mode = BLOG_LINE_FLUSH;
	blog_hdr.control = BLOG_CTL_HIGH_FREQ_CHK;

	offset = 4096; // header size
	for (i = 0; i < BLOG_NUM_MAX; i++) {
		size = BLOG_SBL_LOG_BYTE_SIZE;
		blog_info_init(&blog_hdr.log[i], BLOG_SBL, offset, size);
		offset += size;

		size = BLOG_UEFI_LOG_BYTE_SIZE;
		blog_info_init(&blog_hdr.log[i], BLOG_UEFI, offset, size);
		offset += size;

		size = BLOG_KERNEL_STEP_BYTE_SIZE;
		blog_info_init(&blog_hdr.log[i], BLOG_KERNEL, offset, size);
		offset += size;

		size = BLOG_USER_STEP_BYTE_SIZE;
		blog_info_init(&blog_hdr.log[i], BLOG_ARD, offset, size);
		offset += size;
	}

	return 0;
}
#endif

static struct blog_info *get_blog_info(int idx)
{
	int i;

	for (i = 0; i < BLOG_NUM_MAX; i++)
		if (blog_hdr.log[i].count == blog_hdr.count)
			break;

	i = (i + BLOG_NUM_MAX - idx) % BLOG_NUM_MAX;
	return &blog_hdr.log[i];
}

static DEFINE_MUTEX(partition_rw_mutex);

extern int sd_partition_rw(const char *part_name, int write, loff_t offset,
						void *buffer, size_t len);

static int read_log_partition(unsigned long pos, unsigned char *buf, size_t count)
{
	int ret;

	mutex_lock(&partition_rw_mutex);

	ret = sd_partition_rw(BLOG_PARTITION, 0, pos + BLOG_PARTITION_OFFSET, buf, count);
	if (ret < 0)
		blog_dbg("[%s][%d] sd_raw_partition_read ret %d\n", __func__, __LINE__, ret);

	mutex_unlock(&partition_rw_mutex);

	return 0;
}

static int write_log_partition(unsigned long pos, unsigned char *buf, size_t count)
{
	int ret;

	mutex_lock(&partition_rw_mutex);

	ret = sd_partition_rw(BLOG_PARTITION, 1, pos + BLOG_PARTITION_OFFSET, buf, count);
	if (ret < 0)
		blog_dbg("[%s][%d] sd_raw_partition_write ret %d\n", __func__, __LINE__, ret);

	mutex_unlock(&partition_rw_mutex);

	return 0;
}

static int get_blog_header_from_partition(void)
{
	return read_log_partition(0, (unsigned char *)&blog_hdr, sizeof(struct blog_header));
}

static int set_blog_header_to_partition(void)
{
	blog_hdr.crc = get_blog_hdr_crc(&blog_hdr);
	return write_log_partition(0, (unsigned char *)&blog_hdr, sizeof(struct blog_header));
}

static int write_step_to_partition(unsigned int type, unsigned int pos, char *buf, unsigned int size)
{
	struct blog_info *info = get_blog_info(0);
	
	return write_log_partition(info->stage[type].start + pos, buf, size);
}

static int find_stage2_id(const char *name, unsigned long step)
{
	int i, ret = -EINVAL;

	if (strlen(name) >= MODULE_NAME_LEN)
		return -EINVAL;

	spin_lock(&ard_module_info_lock);
	for (i = 0; i < ARD_MODULE_INFO_NUM && *(module_list + i * MODULE_NAME_LEN); i++) {
		if (!strncmp(name, module_list + i * MODULE_NAME_LEN, MODULE_NAME_LEN)) {
			ret = i;
			break;
		}
	}

	if (i >= ARD_MODULE_INFO_NUM)
		ret = -ENOSPC;
	else if (!(*(module_list + i * MODULE_NAME_LEN)) && !step) {
		strlcpy(module_list + i * MODULE_NAME_LEN, name, MODULE_NAME_LEN);
		ret = i;
	}
	spin_unlock(&ard_module_info_lock);

	return ret;
}

static int find_state_id(const char *state)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(state2str); i++)
		if (!strncmp(state, state2str[i], 5))
			return i;

	return -EINVAL;
}

static int check_blog_header(void)
{
	int ret;

	if (blog_hdr.magic == BLOG_MAGIC_NUM &&
			blog_hdr.version == BLOG_VERSION)
		return 0;

	ret = get_blog_header_from_partition();
	if (ret)
		return ret;

	blog_dbg("[%s][%d] magic %x version %d\n", __func__, __LINE__, blog_hdr.magic, blog_hdr.version);
	if (blog_hdr.magic == BLOG_MAGIC_NUM &&
			blog_hdr.version == BLOG_VERSION)
		return 0;

#ifdef BLOG_MAKE_HDR_IN_KERNEL
	blog_header_init();

	ret = set_blog_header_to_partition();
	if (ret)
		return ret;

	return 0;
#else
	return -EINVAL;
#endif
}

static unsigned int get_real_ptr(struct step_buffer *step, unsigned int ptr)
{
	return (ptr < step->size) ? ptr :
		(step->ring_offset + (ptr - step->size) % (step->size - step->ring_offset));
}

static unsigned int get_real_wp(struct step_buffer *step)
{
	return get_real_ptr(step, step->wp);
}

static unsigned int get_real_rp(struct step_buffer *step)
{
	return get_real_ptr(step, step->rp);
}

static void update_step_partition(unsigned int type, struct step_buffer *step)
{
	int write = 0, ret = -1;
	unsigned int wp, wp_bak, rp;

	ret = check_blog_header();
	if (ret)
		return;

	if (blog_hdr.control & BLOG_CTL_PAUSE)
		return;

	if (blog_hdr.flush_mode == BLOG_LINE_FLUSH)
		write = 1;
	else if (type == BLOG_KERNEL &&
			blog_hdr.flush_mode == BLOG_FINAL_FLUSH &&
			kernel_init_done == 1)
		write = 1;
	else if (type == BLOG_ARD &&
			blog_hdr.flush_mode == BLOG_FINAL_FLUSH &&
			boot_complete == 1)
		write = 1;

	if (!write)
		return;

#ifndef BLOG_SUPPORT_PARTIAL_RING_BUFFER
	if (step->wp >= step->size) {
		blog_dbg("[%s][%d] type %d buffer full, it do not support partial ring buffer!\n",
				__func__, __LINE__, type);
		return;
	}
#endif

	wp = step->wp;
	rp = step->rp;

	wp_bak = wp;
	wp = get_real_wp(step);
	rp = get_real_rp(step);

	wp = get_real_ptr(step, wp + 1);
	if (wp > rp) {
		ret = write_step_to_partition(type, rp, step->start + rp, wp - rp);
	} else if (wp < rp) {
		ret = write_step_to_partition(type, rp, step->start + rp, kstep.size - rp);
		ret += write_step_to_partition(type, step->ring_offset,
				step->start + step->ring_offset, wp - step->ring_offset);
	}

	if (!ret)
		step->rp = wp_bak;
}

static void update_step_partition_all(struct work_struct *dummy)
{
	update_step_partition(BLOG_KERNEL, &kstep);
	update_step_partition(BLOG_ARD, &ustep);
}

static void fill_end_mark(struct step_buffer *step, unsigned int wp)
{
	unsigned int real_ptr;
	
	real_ptr = get_real_ptr(step, wp);
	*(step->start + real_ptr) = 0;
}

static void copy_to_step(struct step_buffer *step, const char *buf, unsigned int len)
{
	unsigned int real_wp, str_len;

	spin_lock(&step->lock);
	real_wp = get_real_wp(step);
	str_len = len;
	len += 1;
	if (real_wp + len > step->size) {
		unsigned int size = step->size - real_wp;

		memcpy(step->start + real_wp, buf, size);
		memcpy(step->start + step->ring_offset, buf + size, len - size);
	} else {
		memcpy(step->start + real_wp, buf, len);
	}
	step->wp += str_len;
	fill_end_mark(step, step->wp);
	spin_unlock(&step->lock);
}

void kstep_log(const char *fmt, ...)
{
	va_list args;
	u64 clk = local_clock();
	char tmp[512];
	int len;

	len = snprintf(tmp, sizeof(tmp) - 1, "[%6d.%06d] ", clk / 1000000000, clk / 1000 % 1000000);
	va_start(args, fmt);
	len += vsnprintf(tmp + len, sizeof(tmp) - 1 - len, fmt, args);
	va_end(args);

	blog_dbg("kstep_m: %s", tmp);

	if (clk / 1000000000 > BOOT_STEP_TIMEOUT && !reboot_start)
		return;

	copy_to_step(&kstep, tmp, len);
	update_step_partition(BLOG_KERNEL, &kstep);
}

static DECLARE_WORK(update_step_partition_work, update_step_partition_all);

static ssize_t steplog_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	return 0;
}

/*
 * format:
 * echo "zygote 1" > /proc/blog/step
 * echo "zygote 3 pre" > /proc/blog/step
 * echo "zygote 3 post" > /proc/blog/step
 */
static ssize_t steplog_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char str[STEPLOG_STR_MAX_LEN * 3] = {0};
	char *state_str = str + STEPLOG_STR_MAX_LEN;
	char *s2_name = str + STEPLOG_STR_MAX_LEN * 2;
	int s2_id = 0, state = 0;
	u32 step, mod_step = 0, len = 0;
	u64 clk = local_clock();
	int ret;

	if (!count || count >= STEPLOG_STR_MAX_LEN) {
		pr_err("[%s][%d] count is wrong!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (copy_from_user(str, buf, count)) {
		pr_err("[%s][%d] copy_from_user fail!\n", __func__, __LINE__);
		return -EFAULT;
	}

	blog_dbg("[%s][%d] user set: %s\n", __func__, __LINE__, str);
	ret = sscanf(str, "%s %d %s", s2_name, &mod_step, state_str);
	if (ret < 2) {
		pr_err("[%s][%d] get parameter fail!\n", __func__, __LINE__);
		return -EINVAL;
	} else if (ret >= 3)
		state = find_state_id(state_str);

	s2_id = find_stage2_id(s2_name, mod_step);
	blog_dbg("[%s][%d] parse: %s, %d, %s\n", __func__, __LINE__, s2_name, mod_step, state_str);

	if (INVAILD_STAGE2(s2_id) || INVAILD_STEP(mod_step) || INVAILD_STATE(state)) {
		pr_err("[%s][%d] parameter is invalid (%d %d %d)!\n",
				__func__, __LINE__, s2_id, mod_step, state);
		return -EINVAL;
	}

	if (!s2_id && mod_step == INIT_RB_START && !reboot_start)
		reboot_start = 1;

	if (!boot_timeout && clk / 1000000000 > BOOT_STEP_TIMEOUT) {
		boot_timeout = 1;
		pr_info("[%s][%d] boot_timeout = 1\n", __func__, __LINE__);
	}

	if ((boot_complete || boot_timeout) && !reboot_start)
		return -EPIPE;

	step = STEP_LOG(SL_S1_ANDROID_BOOT, s2_id, mod_step, state);
	len = snprintf(str, STEPLOG_STR_MAX_LEN * 2,
			"[%6d.%06d] %08x %s %u %s\n", clk / 1000000000, clk / 1000 % 1000000,
			step, s2_name, mod_step, (state > 0) ? state2str[state] : "");
	
	copy_to_step(&ustep, str, len);

	blog_dbg("%s", str);

	if (!s2_id && !boot_complete && mod_step == INIT_BOOT_COMPLETE) {
		struct blog_info *info = get_blog_info(0);

		boot_complete = 1;
		info->boot_complete = 1;
		set_blog_header_to_partition();
		pr_info("[%s][%d] boot_complete = 1\n", __func__, __LINE__);
	}
	
	//schedule_work(&update_step_partition_work);
	//update_step_partition_all(0);
	update_step_partition(BLOG_ARD, &ustep);

	return count;
}

static const struct file_operations step_proc_fops = {
	.read       = steplog_read,
	.write      = steplog_write,
	.llseek     = seq_lseek,
};

static int steplog_reboot_notify(struct notifier_block *nb, 
		unsigned long code, void *unused)
{
	struct blog_info *info = get_blog_info(0);

	if (info->boot_complete) {
		if (blog_hdr.control & BLOG_CTL_PAUSE) {
			blog_hdr.control &= ~BLOG_CTL_PAUSE;
			blog_hdr.control |= BLOG_CTL_RESUME;
			set_blog_header_to_partition();
		} else if (blog_hdr.flush_mode != BLOG_NO_FLUSH) {
			info->boot_complete = BLOG_NORMAL_REBOOT;
			set_blog_header_to_partition();
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = steplog_reboot_notify,
};


static int state_full_info;
static unsigned int blog_print_mask = ~0;

static int print_blog_content(struct seq_file *m, struct blog_desc *desc)
{
	unsigned int offset, size, len;
	char *tmp = vzalloc(BATCH_SIZE+1);

	if (!tmp)
		return -ENOMEM;

	offset = desc->start;
	size = desc->size;
	while (size) {
		len = size > BATCH_SIZE ? BATCH_SIZE : size;
		read_log_partition(offset, tmp, len);
		seq_puts(m, tmp);
		if (strlen(tmp) < BATCH_SIZE)
			break;
		offset += len;
		size -= len;
	}

	vfree(tmp);

	return 0;	
}

static int blog_proc_show(struct seq_file *m, void *v)
{
	struct blog_info *info = NULL;
	int i = (int)m->private;
	int ret;

	ret = check_blog_header();
	if (ret) {
		pr_err("[%s][%d] check_blog_header fail!\n", __func__, __LINE__);
		return ret;
	}

	info = get_blog_info(i);

	seq_printf(m, "blog%d:\n", i);
	seq_printf(m, "\tcount: %d\n", info->count);
	seq_printf(m, "\tboot_complete: %d\n", info->boot_complete);

	if (blog_print_mask & (1 << BLOG_SBL)) {
		seq_printf(m, "\nsbl log:\n");
		print_blog_content(m, &info->stage[BLOG_SBL]);
	}

	if (blog_print_mask & (1 << BLOG_UEFI)) {
		seq_printf(m, "\n\nUEFI log:\n");
		print_blog_content(m, &info->stage[BLOG_UEFI]);
	}

	if (blog_print_mask & (1 << BLOG_KERNEL)) {
		seq_printf(m, "\n\nKernel step:\n");
		print_blog_content(m, &info->stage[BLOG_KERNEL]);
	}

	if (blog_print_mask & (1 << BLOG_ARD)) {
		seq_printf(m, "\n\nAndroid step:\n");
		print_blog_content(m, &info->stage[BLOG_ARD]);
	}

	return 0;
}

static int blog_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, blog_proc_show, PDE_DATA(inode));
}

static const struct file_operations blog_proc_fops = {
	.open       = blog_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static ssize_t state_proc_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
	char flag;
	int update_hdr = 0;

	if (count <= 0) 
		return 0;	

	if (get_user(flag, buffer))
		return -EFAULT;

	switch (flag) {
	case '0':
		blog_print_mask = 0;
		break;
	case '1' ... '9':
		blog_print_mask |= (1 << (flag - '1'));
		break;
	case 'd':
		state_full_info = 1;
		break;
	case 'n': 
		blog_hdr.flush_mode = BLOG_NO_FLUSH;
		update_hdr = 1;
		break;
	case 'l': 
		blog_hdr.flush_mode = BLOG_LINE_FLUSH;
		update_hdr = 1;
		break;
	case 'f': 
		blog_hdr.flush_mode = BLOG_FINAL_FLUSH;
		update_hdr = 1;
		break;
	case 'h':
		blog_hdr.control ^= BLOG_CTL_HIGH_FREQ_CHK;
		update_hdr = 1;
		break;
	case 'c':
		memset(&blog_hdr, 0, sizeof(blog_hdr));
		update_hdr = 1;
		break;
	default:
		break;
	}

	if (update_hdr)
		set_blog_header_to_partition();

	return count;
}

static int get_blog_used_size(struct blog_desc *desc)
{
	unsigned int offset, size, len, str_len;
	char end_mark;
	char *tmp = vzalloc(BATCH_SIZE+1);

	if (!tmp)
		return -ENOMEM;

	offset = desc->start;
	size = desc->size;
	
	read_log_partition(offset + size - 1, &end_mark, 1);
	if (end_mark)
		return size;
	
	while (size) {
		len = size > BATCH_SIZE ? BATCH_SIZE : size;
		read_log_partition(offset, tmp, len);
		str_len = strlen(tmp);
		if (str_len < BATCH_SIZE)
			break;
		offset += len;
		size -= len;
	}

	vfree(tmp);

	size = size ? (desc->size - size + str_len) : desc->size;

	return size;
}

static int state_proc_show(struct seq_file *m, void *v)
{
	int i;
	struct blog_info *info;
	
	if (state_full_info) {
		unsigned int blog_size = 4096;

		// statistic blog buffer
		seq_printf(m, "statistic:\n");
		for (i = 0; i < BLOG_NUM_MAX; i++) {
			info = get_blog_info(i);
			seq_printf(m, "\t%d: SBL(%d/%d) UEFI(%d/%d) KERNEL(%d/%d) ARD(%d/%d)\n", i,
					get_blog_used_size(&info->stage[BLOG_SBL]), info->stage[BLOG_SBL].size,
					get_blog_used_size(&info->stage[BLOG_UEFI]), info->stage[BLOG_UEFI].size,
					get_blog_used_size(&info->stage[BLOG_KERNEL]), info->stage[BLOG_KERNEL].size,
					get_blog_used_size(&info->stage[BLOG_ARD]), info->stage[BLOG_ARD].size);
			blog_size += info->stage[BLOG_SBL].size + info->stage[BLOG_UEFI].size +
				info->stage[BLOG_KERNEL].size + info->stage[BLOG_ARD].size;
		}
		seq_printf(m, "\tblog size: 0x%08x\n", blog_size);

		// blog header
		get_blog_header_from_partition();
		seq_printf(m, "\nblog_header:\n");
		seq_printf(m, "\tmagic: 0x%08x\n", blog_hdr.magic);
		seq_printf(m, "\tversion: %d\n", blog_hdr.version);
		seq_printf(m, "\tcrc: 0x%08x\n", blog_hdr.crc);
		seq_printf(m, "\tcount: %d\n", blog_hdr.count);
		seq_printf(m, "\tflush_mode: %08x\n", blog_hdr.flush_mode);
		seq_printf(m, "\tcontrol: %08x\n", blog_hdr.control);
		for (i = 0; i < BLOG_NUM_MAX; i++) {
			info = &blog_hdr.log[i];
			seq_printf(m, "\nblog_info[%d]:\n", i);
			seq_printf(m, "\tcount: %d\n", info->count);
			seq_printf(m, "\tboot_complete: 0x%08x\n", info->boot_complete);
			seq_printf(m, "\tstage[SBL] start: 0x%08x size: 0x%08x ring_offset: 0x%08x wptr: 0x%08x\n",
					info->stage[BLOG_SBL].start, info->stage[BLOG_SBL].size,
					info->stage[BLOG_SBL].ring_offset, info->stage[BLOG_SBL].wptr);
			seq_printf(m, "\tstage[UEFI] start: 0x%08x size: 0x%08x ring_offset: 0x%08x wptr: 0x%08x\n",
					info->stage[BLOG_UEFI].start, info->stage[BLOG_UEFI].size,
					info->stage[BLOG_UEFI].ring_offset, info->stage[BLOG_UEFI].wptr);
			seq_printf(m, "\tstage[KERNEL] start: 0x%08x size: 0x%08x ring_offset: 0x%08x wptr: 0x%08x\n",
					info->stage[BLOG_KERNEL].start, info->stage[BLOG_KERNEL].size,
					info->stage[BLOG_KERNEL].ring_offset, info->stage[BLOG_KERNEL].wptr);
			seq_printf(m, "\tstage[ARD] start: 0x%08x size: 0x%08x ring_offset: 0x%08x wptr: 0x%08x\n",
					info->stage[BLOG_ARD].start, info->stage[BLOG_ARD].size,
					info->stage[BLOG_ARD].ring_offset, info->stage[BLOG_ARD].wptr);
		}

		seq_printf(m, "\ndebug info:\n");
		seq_printf(m, "\tstate_full_info: 0x%x\n", state_full_info);
		seq_printf(m, "\tblog_print_mask: 0x%x\n", blog_print_mask);
		seq_printf(m, "\tboot_complete: 0x%x\n", boot_complete);
		seq_printf(m, "\tboot_timeout: 0x%x\n", boot_timeout);
		seq_printf(m, "\treboot_start: 0x%x\n", reboot_start);
		seq_printf(m, "\tkernel_init_done: 0x%x\n", kernel_init_done);
	} else {
		for (i = 0; i < BLOG_NUM_MAX; i++) {
			info = get_blog_info(i);
			seq_printf(m, "[%d] boot_complete: 0x%08x\n", i, info->boot_complete);
		}
	}

	return 0;
}

static int state_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, state_proc_show, PDE_DATA(inode));
}

static const struct file_operations state_proc_fops = {
	.open       = state_proc_open,
	.write      = state_proc_write,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

const char *blog_name[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};

static int __init proc_steplog_init(void)
{
	struct proc_dir_entry *blog_entry;
	unsigned long i = 0;

	blog_entry = proc_mkdir("blog", NULL);
	if (!blog_entry) {
		pr_err("%s: failed to create /proc/blog entry\n", __func__);
		return -EPERM;
	}

	for (i = 0; i < BLOG_NUM_MAX; i++) {
		BUG_ON(ARRAY_SIZE(blog_name) < BLOG_NUM_MAX);
		proc_create_data(blog_name[i], 0400, blog_entry, &blog_proc_fops, (void *)i);
	}
	proc_create("state", 0600, blog_entry, &state_proc_fops);

	// ard module list
	module_list = vzalloc(ARD_MODULE_INFO_BYTE_SIZE);
	if (!module_list) {
		pr_err("%s: Failed to alloc module_list memory for steplog\n", __func__);
		return -ENOMEM;
	}
	pr_info("steplog module_list @%p\n", module_list);
	strlcpy(module_list, "init", MODULE_NAME_LEN);

	// user step log buffer 
	ustep.start = vzalloc(ustep.size);
	if (!ustep.start) {
		pr_err("%s: Failed to alloc user step memory for steplog\n", __func__);
		return -ENOMEM;
	}
	pr_info("steplog user step @%p\n", module_list);

	proc_create("step", 0620, blog_entry, &step_proc_fops);

	register_reboot_notifier(&reboot_notifier);

	return 0;
}

fs_initcall(proc_steplog_init);

#define BLOG_TEST_MODE
#ifdef BLOG_TEST_MODE
#define HUNG_STAGE_BOOT 1
#define HUNG_STAGE_SHUTDOWN 2

#define BLOG_NUM_MAX 10
#define COTINUOUS_HUNG_COUNT 6
#define DEBUG_MODE 1
#define TEST_MODE
#define LOG_SIZE 200
#define EXP_STAGE_XBL 1
#define EXP_STAGE_ABL 2
#define EXP_STAGE_KERNEL 3
#define EXP_TYPE_CRASH 1
#define EXP_TYPE_HUNG 2
#define EXP_COUNTER 5

struct hung_detect_header {
	u8 hung_flag;
	u8 boot_fail_flag;
	u8 pmic_wdt_flag;
	u8 magic;
	u8 reserved[4];
};
#define offset_of(header, member) ((char *)(&(header.member)) - (char *)(&header))
#define TEST_OFFSET offset_of(header, reserved)
struct test_header {
	u8 exp_type;
	u8 exp_stage;
	u8 exp_times;
	u8 reserved;
};
struct hung_detect_header header;

struct test_header *test_header;
void try_to_trigger_hung(u8 hung_stage)
{
	int ret = 0;

	pr_err("[%s][%d] hung_stage %d\n", __func__, __LINE__, hung_stage);
	pr_err("exp_type = %d, exp_stage = %d, exp_times = %d",
			test_header->exp_type, test_header->exp_stage,
			test_header->exp_times);
	if (test_header && test_header->exp_stage == EXP_STAGE_KERNEL) {
		if (test_header->exp_type == EXP_TYPE_HUNG) {
			if (test_header->exp_times == hung_stage) {
				test_header->exp_type = 0;
				mutex_lock(&partition_rw_mutex);
				ret = sd_partition_rw(BLOG_PARTITION, 1, TEST_OFFSET, test_header,
					sizeof(struct test_header));
				mutex_unlock(&partition_rw_mutex);
				if (ret < 0)
					pr_err("[%s][%d] sd_raw_partition_write ret %d\n", __func__, __LINE__, ret);

				pr_err("hang start...");
				while (1)
					;
				pr_err("hang fail...");
			}
		}
	}
}

void  hung_detect_init(void)
{
	int ret = 0;
	test_header = (struct test_header *)(((u8 *)&header) + TEST_OFFSET);

	/* init header first*/
	ret = sd_partition_rw(BLOG_PARTITION, 0, 0, &header, sizeof(header));
	if (ret < 0)
		pr_err("[%s][%d] sd_partition_rw ret %d\n", __func__, __LINE__, ret);

	try_to_trigger_hung(HUNG_STAGE_BOOT);
}
#else
void try_to_trigger_hung(u8 hung_stage) {}
void  hung_detect_init(void) {}
#endif

#endif
