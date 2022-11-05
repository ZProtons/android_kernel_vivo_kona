/*===========================================================================
* VENDOR_EDIT
* kme/pmic_wdt.c
*
* VIVO Kernel PMIC WDT Engine(Pmic Wdt )
*
* Copyright (C) 2017 VIVO Technology Co., Ltd
* ------------------------------ Revision History: --------------------------
* <version>           <date>               <author>                  <desc>
* Revision 1.0        2019-07-14       hankecai@vivo.com         Created file
* Revision 1.1        2019-12-21       hankecai@vivo.com         Modified file
*===========================================================================*/

#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
#include <linux/typecheck.h>
#include <linux/percpu-defs.h>
#include <linux/kobject.h>
#include <linux/seq_file.h>
#include <linux/ratelimit.h>
#include <linux/mutex.h>
#include <linux/sched/debug.h>

/*===========================================================================
*
*			Public part declarations
*
*===========================================================================*/
enum BOOT_STAGE {
	B_NONE			= 0x00,
	B_BOOTLOADER	= 0x01,
	B_KERNEL		= 0x02,
	B_FRAMEWORK 	= 0x03,
};

enum WDT_CMD_CTL {
	CMD_STOP				= 0,
	CMD_REBOOT				= -1,
	CMD_FORCE_OFFLINE_ON	= -101,
	CMD_FORCE_OFFLINE_OFF	= -100,
};

static bool wdt_force_offline;
module_param_named(wdt_force_offline, wdt_force_offline, bool, 0644);

extern int kme_merge_group(const struct attribute_group *);
extern void kme_remove_group(const struct attribute_group *);

static long enable_monitor;
static int boot_stage_flags = B_NONE;
static int max_crash_cnt_limit = 20;
static DEFINE_MUTEX(g_wdt_mutex_lock);

static int vivo_pmic_wdt_enable(void);
static int vivo_pmic_wdt_disable(void);
static int vivo_pmic_wdt_cfg(long timeout);
static int vivo_pmic_wdt_force_offline(bool force_offline);

int vivo_pmic_wdt_pet(void);
int vivo_pmic_wdt_start(long timeout);
int vivo_pmic_wdt_start_kernel(long timeout);
int vivo_pmic_wdt_stop(void);
int vivo_pmic_wdt_reboot(void);
void vivo_pmic_wdt_setup(bool g_force_offline_init);

EXPORT_SYMBOL(vivo_pmic_wdt_start);
EXPORT_SYMBOL(vivo_pmic_wdt_start_kernel);
EXPORT_SYMBOL(vivo_pmic_wdt_stop);
EXPORT_SYMBOL(vivo_pmic_wdt_pet);
EXPORT_SYMBOL(vivo_pmic_wdt_reboot);
EXPORT_SYMBOL(vivo_pmic_wdt_setup);

/*common ops of pmic wdt module*/
struct pmic_wdt_ops {
	int (*wdt_enable) (void);
	int (*wdt_disable) (void);
	int (*wdt_pet) (void);
	int (*wdt_cfg) (long timeout);
	int (*wdt_reboot) (void);
	int (*wdt_force_offline)(bool offline);
};

/*setup ops*/
static struct pmic_wdt_ops *pmic_wdt_common_ops;
static void vivo_pmic_wdt_ops_setup(struct pmic_wdt_ops *ops)
{
	pmic_wdt_common_ops = ops;
}

/*enable wdt*/
static int vivo_pmic_wdt_enable(void)
{
	int rc = 0;

	if (!pmic_wdt_common_ops)
		return 0;

	if (pmic_wdt_common_ops->wdt_enable)
		rc = pmic_wdt_common_ops->wdt_enable();

	return rc;
}

/*disable wdt*/
static int vivo_pmic_wdt_disable(void)
{
	int rc = 0;

	if (!pmic_wdt_common_ops)
		return 0;

	if (pmic_wdt_common_ops->wdt_disable)
		rc = pmic_wdt_common_ops->wdt_disable();

	return rc;
}

/*cfg wdt timeout*/
static int vivo_pmic_wdt_cfg(long timeout)
{
	int rc = 0;

	if (!pmic_wdt_common_ops)
		return 0;

	if (pmic_wdt_common_ops->wdt_cfg)
		rc = pmic_wdt_common_ops->wdt_cfg(timeout);

	return rc;
}

/*force_offline wdt*/
static int vivo_pmic_wdt_force_offline(bool force_offline)
{
	int rc = 0;

	if (!pmic_wdt_common_ops)
		return 0;

	if (pmic_wdt_common_ops->wdt_force_offline)
		rc = pmic_wdt_common_ops->wdt_force_offline(force_offline);

	return rc;
}

/*start pmic wdt*/
int vivo_pmic_wdt_start(long timeout)
{
	int rc = 0;

	mutex_lock(&g_wdt_mutex_lock);
	if (wdt_force_offline) {
		printk("pmic_wdt state: force_offline!\n");
		goto unlock;
	}

	printk("pmic_wdt_start: 0x%02x|%ld\n",
				boot_stage_flags, enable_monitor);

	rc = vivo_pmic_wdt_disable();
	rc = vivo_pmic_wdt_cfg(timeout);
	rc = vivo_pmic_wdt_enable();

unlock:
	mutex_unlock(&g_wdt_mutex_lock);
	return rc;
}

/*start pmic wdt from kernel*/
int vivo_pmic_wdt_start_kernel(long timeout)
{
	int rc = 0;

	if (wdt_force_offline) {
		printk("pmic_wdt state: force_offline!\n");
		return rc;
	}

	boot_stage_flags = B_KERNEL;
	enable_monitor = timeout;
	rc = vivo_pmic_wdt_start(timeout);

	return rc;
}

/*stop pmic wdt*/
int vivo_pmic_wdt_stop(void)
{
	int rc = 0;

	mutex_lock(&g_wdt_mutex_lock);
	if (wdt_force_offline) {
		printk("pmic_wdt state: force_offline!\n");
		goto unlock;
	}

	printk("pmic_wdt_stop: 0x%02x|%ld\n",
				boot_stage_flags, enable_monitor);

	rc = vivo_pmic_wdt_disable();
	enable_monitor = 0;

unlock:
	mutex_unlock(&g_wdt_mutex_lock);
	return rc;
}

/*pet wdt*/
int vivo_pmic_wdt_pet(void)
{
	int rc = 0;

	mutex_lock(&g_wdt_mutex_lock);
	if (wdt_force_offline) {
		printk("pmic_wdt state: force_offline!\n");
		goto unlock;
	}

	printk("pmic_wdt_pet...\n");
	if (!pmic_wdt_common_ops)
		goto unlock;

	if (pmic_wdt_common_ops->wdt_pet)
		rc = pmic_wdt_common_ops->wdt_pet();

unlock:
	mutex_unlock(&g_wdt_mutex_lock);
	return rc;
}

/*reboot system*/
int vivo_pmic_wdt_reboot(void)
{
	int rc = 0;

	mutex_lock(&g_wdt_mutex_lock);
	printk("pmic_wdt_reboot...\n");
	if (!pmic_wdt_common_ops)
		goto unlock;

	if (pmic_wdt_common_ops->wdt_reboot)
		rc = pmic_wdt_common_ops->wdt_reboot();

unlock:
	mutex_unlock(&g_wdt_mutex_lock);
	return rc;
}

/*sys/kernel/kme/wdt/enable_monitor*/
static ssize_t enable_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%ld\n", enable_monitor);
}

extern unsigned int power_off_charging_mode;
extern unsigned int is_normal_mode;
/*sys/kernel/kme/wdt/enable_monitor*/
static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int ret = 0;
	long temp_enable = 0;

	if (power_off_charging_mode)
		return count;

	if (is_normal_mode == 0)
		return count;

	/*validity check*/
	ret = kstrtol(buf, 10, &temp_enable);
	if (ret)
		return ret;

	/*take this value*/
	enable_monitor = temp_enable;

	/*Set flags to be framework*/
	boot_stage_flags = B_FRAMEWORK;

	/*reboot immediately if -1 or cnt excced*/
	if ((enable_monitor == CMD_REBOOT) ||
		(max_crash_cnt_limit == 0)) {
		vivo_pmic_wdt_reboot();
		return count;
	}

	/*force_offline ON, will take effect at next boot*/
	if (enable_monitor == CMD_FORCE_OFFLINE_ON) {
		vivo_pmic_wdt_force_offline(1);
		return count;
	}

	/*force_online OFF, will take effect at next boot*/
	if (enable_monitor == CMD_FORCE_OFFLINE_OFF) {
		vivo_pmic_wdt_force_offline(0);
		return count;
	}

	/*stop pmic wdt if 0*/
	if (enable_monitor == CMD_STOP) {
		vivo_pmic_wdt_stop();
		return count;
	}

	/*start pmic wdt if larger than 0*/
	max_crash_cnt_limit--;
	vivo_pmic_wdt_start(enable_monitor);
	return count;
}

/*create node /sys/kernel/kme/wdt*/
static struct kobj_attribute enable_attr =
	__ATTR(enable_monitor, 0640, enable_show, enable_store);

static struct attribute *wdt_attrs[] = {
	&enable_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name = "wdt",
	.attrs = wdt_attrs,
};
/*===========================================================================
*
*			Platform implementation part declarations
*
*===========================================================================*/
#ifdef CONFIG_VIVO_PMIC_WDT

#define WDT_MIN_TIME (1)
#define WDT_MAX_TIME (120)
#define WDT_MAXIMUN_MARGIN (20)

static int wdt_maximum_time;
static int wdt_minimum_time;
static int wdt_interval_time;

static int wdt_pet_interval;
static int wdt_pet_cylcle;
static int wdt_pet_remainder;
static struct timer_list wdt_pet_timer;

static const char * const boot_state[] = {
	[B_NONE] = "None",
	[B_BOOTLOADER] = "pmic_wdt(Bootlader|reboot,system_block)",
	[B_KERNEL] = "pmic_wdt(Kernel|reboot,system_block)",
	[B_FRAMEWORK] = "pmic_wdt(Framework|reboot,system_block)",
};

/*extern func from qpnp_pon module*/
extern int qpnp_pon_wd_pet(void);
extern int qpnp_pon_wd_enable(bool enable, u8 flags);
extern int qpnp_pon_wd_cfg_timer(u8 s1_timer, u8 s2_timer);
extern int qpnp_pon_wd_set_force_offline(bool force_offline);
extern int qpnp_pon_wd_set_test_mode(bool test_mode);

/*test_mode_ctl for wdt*/
static int wdt_test_mode_ctl;
static int wdt_test_mode_ctl_set(const char *val, const struct kernel_param *kp);
module_param_call(wdt_test_mode_ctl, wdt_test_mode_ctl_set, param_get_int,
			&wdt_test_mode_ctl, 0644);

/*test_mode_ctl set*/
static int wdt_test_mode_ctl_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	int old_val = wdt_test_mode_ctl;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	if (wdt_test_mode_ctl == 1)
		qpnp_pon_wd_set_test_mode(1);
	else if (wdt_test_mode_ctl == 0)
		qpnp_pon_wd_set_test_mode(0);
	else
		wdt_test_mode_ctl = old_val;

	return 0;
}

void pmic_wdt_dump_debug_info(void)
{
	show_state_filter(TASK_UNINTERRUPTIBLE);
	dump_stack();
	printk("%s: 0x%02x|%s\n", __func__, boot_stage_flags, boot_state[boot_stage_flags]);
	mdelay(50);
}

int pmic_wdt_set_emergency_reset(void)
{
	int rc = 0;

	rc = qpnp_pon_wd_cfg_timer(0, 0);
	rc = qpnp_pon_wd_enable(1, boot_stage_flags);

	return rc;
}

/*pmic pet timer start*/
void pmic_wdt_pet_timer_start(void)
{
	mod_timer(&wdt_pet_timer, jiffies + wdt_pet_interval*HZ);

	printk("%s: wdt_maximum_time:%d wdt_interval_time:%d\n",
						__func__, wdt_maximum_time, wdt_pet_interval);

	printk("%s: wdt_pet_cylcle:%d wdt_pet_remainder:%d\n",
						__func__, wdt_pet_cylcle, wdt_pet_remainder);
}

/*pmic pet timer stop*/
void pmic_wdt_pet_timer_stop(void)
{
	if (timer_pending(&wdt_pet_timer))
		del_timer_sync(&wdt_pet_timer);

	printk("%s: wdt_pet_cylcle:%d wdt_pet_remainder:%d\n",
						__func__, wdt_pet_cylcle, wdt_pet_remainder);
}

/*pmic pet timer function*/
void pmic_wdt_pet_timer_fn(struct timer_list *unused)
{
	if (wdt_pet_cylcle)
		wdt_pet_cylcle = wdt_pet_cylcle - 1;

	printk("%s: 0x%02x|wdt_pet_cylcle:%d wdt_pet_remainder:%d\n", 
				__func__, boot_stage_flags, wdt_pet_cylcle, wdt_pet_remainder);

	if (wdt_pet_cylcle > 0) {
		wdt_pet_interval = wdt_interval_time;
		goto keepalive;
	}

	if (wdt_pet_remainder > wdt_minimum_time) {
		wdt_pet_interval = wdt_pet_remainder;
		wdt_pet_remainder = 0;
		goto keepalive;
	} else {
		if (power_off_charging_mode == 1)
			return;

		pmic_wdt_dump_debug_info();
		pmic_wdt_set_emergency_reset();
		return;
	}

keepalive:
	qpnp_pon_wd_pet();
	mod_timer(&wdt_pet_timer, jiffies + wdt_pet_interval*HZ);
	return;
}

/*pmic wdt timer setup*/
void pmic_wdt_timer_setup(void)
{
	timer_setup(&wdt_pet_timer, pmic_wdt_pet_timer_fn, 0);
}

/*pmic wdt enable*/
int pmic_wdt_enable(void)
{
	int rc = 0;

	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);
	rc = qpnp_pon_wd_enable(1, boot_stage_flags);

	pmic_wdt_pet_timer_start();

	return rc;
}

/*pmic wdt disable*/
int pmic_wdt_disable(void)
{
	int rc = 0;

	/*auto set if boot_stage_flags = 0*/
	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);
	rc = qpnp_pon_wd_enable(0, B_NONE);
	pmic_wdt_pet_timer_stop();

	wdt_pet_cylcle = 0;
	wdt_pet_remainder = 0;

	return rc;
}

/*pmic wdt pet*/
int pmic_wdt_pet(void)
{
	int rc = 0;

	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);
	rc = qpnp_pon_wd_pet();

	return rc;
}

/*pmic wdt cfg*/
int pmic_wdt_cfg(long timeout)
{
	int rc = 0;

	if (timeout < wdt_minimum_time)
		return -EINVAL;

	wdt_pet_cylcle = timeout/wdt_interval_time;
	wdt_pet_remainder = timeout%wdt_interval_time;

	if (wdt_pet_cylcle > 0) {
		wdt_pet_interval = wdt_interval_time;
	} else {
		wdt_pet_interval = timeout;
		wdt_pet_remainder = wdt_minimum_time;
		wdt_pet_cylcle = 1;
	}

	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, timeout);
	rc = qpnp_pon_wd_cfg_timer(wdt_maximum_time, 0);

	return rc;
}

/*pmic wdt reboot*/
int pmic_wdt_reboot(void)
{
	int rc = 0;

	/*do immediately low-level reboot*/
	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);

	pmic_wdt_dump_debug_info();
	rc = pmic_wdt_set_emergency_reset();

	return rc;
}

/*pmic wdt force_offline*/
int pmic_wdt_force_offline(bool force_offline)
{
	int rc = 0;

	/*force_offline*/
	printk("%s: 0x%02x|%ld\n", __func__, boot_stage_flags, enable_monitor);
	if (force_offline) {
		printk("pmic_wdt force_offline true from remote!\n");
		rc = qpnp_pon_wd_set_force_offline(1);
	} else {
		printk("pmic_wdt force_offline false from remote!\n");
		rc = qpnp_pon_wd_set_force_offline(0);
	}

	return rc;
}

/*pmic wdt ops*/
struct pmic_wdt_ops qcom_pmic_wdt_ops = {
	.wdt_enable = pmic_wdt_enable,
	.wdt_disable = pmic_wdt_disable,
	.wdt_pet = pmic_wdt_pet,
	.wdt_cfg = pmic_wdt_cfg,
	.wdt_reboot = pmic_wdt_reboot,
	.wdt_force_offline = pmic_wdt_force_offline,
};

static bool __vivo_wdt_setup_done;
void vivo_pmic_wdt_setup(bool g_force_offline_init)
{
	if (__vivo_wdt_setup_done)
		return;

	wdt_force_offline = g_force_offline_init;
	printk("pmic_wdt force_offline_init[%d]\n", g_force_offline_init);

	wdt_minimum_time = WDT_MIN_TIME;
	wdt_maximum_time = WDT_MAX_TIME;
	if (wdt_maximum_time > WDT_MAXIMUN_MARGIN) {
		wdt_interval_time = wdt_maximum_time - WDT_MAXIMUN_MARGIN;
	} else {
		wdt_interval_time = 1;
	}
	printk("%s: wdt_maximum_time:%d wdt_interval_time:%d\n",
						__func__, wdt_maximum_time, wdt_interval_time);

	pmic_wdt_timer_setup();
	vivo_pmic_wdt_ops_setup(&qcom_pmic_wdt_ops);

	__vivo_wdt_setup_done = true;
}
#endif	/*CONFIG_VIVO_PMIC_WDT*/
/*===========================================================================
*
*			Module entry initialization
*
*===========================================================================*/

static int __init vivo_pmic_wdt_init(void)
{
	int ret;

	ret = kme_merge_group(&attr_group);

#ifdef CONFIG_VIVO_PMIC_WDT
	if (ret) {
		vivo_pmic_wdt_setup(0);
		vivo_pmic_wdt_disable();
	}
#endif /*CONFIG_VIVO_PMIC_WDT*/

	return ret;
}

static void __exit vivo_pmic_wdt_exit(void)
{
	kme_remove_group(&attr_group);
}

module_init(vivo_pmic_wdt_init);
module_exit(vivo_pmic_wdt_exit);
MODULE_LICENSE("GPL v2");
