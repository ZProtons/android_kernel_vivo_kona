/*
 * vivo bsp sysfs directory implementation
 * It's highly recommended that all bsp developed features
 * export their sysfs interfaces under /sys/kernel/bsp_stability/
 *
 * Copyright (C) 2017 luochucheng <luochucheng@vivo.com>
 * Copyright (C) 2077 VIVO Inc.
 *
 * Released under the GPL version 2 only.
 *
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define MAX_ENV_LEN 256
#define TAG "[kernel_monitor_engine]"

static int enable_monitor = 1;

struct kobject *kme_kobj;
static struct work_struct async_work;
static char **g_envp_ext;
static spinlock_t g_lock;


/*
 * report all kinds of bottlenecks to user space through uevent
 * @argument:@envp_ext : a string array to send to userspace
 * the report format should be like this:
 * char *envp_ext[] ={
 * 	"subsystem=kernel_monitor_engine",
 * 	"exception=rt_throttling",
 *	"process=process_name",
 *	"processid = pid",
 * 	NULL,
 * 	};
 */
int kernel_monitor_report(char *envp_ext[])
{
	unsigned long flags;

	spin_lock_irqsave(&g_lock, flags);
	g_envp_ext = (char **)envp_ext;
	schedule_work(&async_work);
	spin_unlock_irqrestore(&g_lock, flags);
	return 0;
}
EXPORT_SYMBOL(kernel_monitor_report);

/*
 * report task hung in disk sleep stat, usually caused by deadlock
 * @argument:@t : the task that has been detected hung in d state continually
 * @argument:@timeout: the time inteval of hung
 */
int kernel_monitor_report_dhung(struct task_struct *t, long timeout)
{
	char envp_ext[6][MAX_ENV_LEN];
	char *envp[6] = {NULL};
	int idx = 0;
	pr_err(TAG"%s\n", __func__);
	snprintf(envp_ext[0], MAX_ENV_LEN, "subsystem=kernel_monitor_engine");
	snprintf(envp_ext[1], MAX_ENV_LEN, "exception=hung_task");
	snprintf(envp_ext[2], MAX_ENV_LEN, "process=%s", t->comm);
	snprintf(envp_ext[3], MAX_ENV_LEN, "processid=%d", t->pid);
	snprintf(envp_ext[4], MAX_ENV_LEN, "timeout=%ld", timeout);

	for (idx = 0; idx < 5; idx++) {
		envp[idx] = envp_ext[idx];
		pr_err(TAG"%s\n", envp[idx]);
	}
	return kernel_monitor_report(envp);
}
EXPORT_SYMBOL(kernel_monitor_report_dhung);

/*
 * report scheduler block by preemption disable
 * @argument:@duration: how long the current cpu is hung.
 */
int kernel_monitor_report_soft_lock(unsigned duration)
{
	char envp_ext[6][MAX_ENV_LEN];
	char *envp[6] = {NULL};
	int idx = 0;
	pr_err(TAG"%s\n", __func__);
	snprintf(envp_ext[0], MAX_ENV_LEN, "subsystem=kernel_monitor_engine");
	snprintf(envp_ext[1], MAX_ENV_LEN, "exception=soft_lock");
	snprintf(envp_ext[2], MAX_ENV_LEN, "process=%s", current->comm);
	snprintf(envp_ext[3], MAX_ENV_LEN, "processid=%d", task_pid_nr(current));
	snprintf(envp_ext[4], MAX_ENV_LEN, "timeout=%u", duration);
	for (idx = 0; idx < 5; idx++) {
		envp[idx] = envp_ext[idx];
		pr_err(TAG"%s\n", envp[idx]);
	}
	return kernel_monitor_report(envp);
}
EXPORT_SYMBOL(kernel_monitor_report_soft_lock);

/*
 * report scheduler block by interrupt disable
 * @argument:@duration: how long the current interrupt is disabled.
 */
int kernel_monitor_report_hard_lock(int this_cpu)
{
	char envp_ext[3][MAX_ENV_LEN];
	char *envp[6] = {NULL};
	int idx = 0;
	pr_err(TAG"%s\n", __func__);
	snprintf(envp_ext[0], MAX_ENV_LEN, "subsystem=kernel_monitor_engine");
	snprintf(envp_ext[1], MAX_ENV_LEN, "exception=hard_lock");
	snprintf(envp_ext[2], MAX_ENV_LEN, "cpu=%d", this_cpu);
	for (idx = 0; idx < 3; idx++) {
		envp[idx] = envp_ext[idx];
		pr_err(TAG"%s\n", envp[idx]);
	}
	return kernel_monitor_report(envp);
}
EXPORT_SYMBOL(kernel_monitor_report_hard_lock);

/*
 * report rcu stall events
 * @argument:@t: the task cause rcu stall.
 * @argument:@cpu: current cpu the task is running on.
 */
int kernel_monitor_report_rcu_stall(struct task_struct *t, int cpu)
{
	char envp_ext[6][MAX_ENV_LEN];
	char *envp[6] = {NULL};
	int idx = 0;
	pr_err(TAG"%s\n", __func__);
	snprintf(envp_ext[0], MAX_ENV_LEN, "subsystem=kernel_monitor_engine");
	snprintf(envp_ext[1], MAX_ENV_LEN, "exception=rcu_stall");
	snprintf(envp_ext[2], MAX_ENV_LEN, "process=%s", t->comm);
	snprintf(envp_ext[3], MAX_ENV_LEN, "processid=%d", task_pid_nr(t));
	snprintf(envp_ext[4], MAX_ENV_LEN, "cpu=%d", cpu);
	for (idx = 0; idx < 5; idx++) {
		envp[idx] = envp_ext[idx];
		pr_err(TAG"%s\n", envp[idx]);
	}
	return kernel_monitor_report(envp);
}
EXPORT_SYMBOL(kernel_monitor_report_rcu_stall);

/*
 * report rt throttle events
 * @argument:@t: the rt process causes rt throttle.
 * @argument:@cpu: current cpu the task is running on.
 */
int kernel_monitor_report_rt_throttle(struct task_struct *t, int cpu)
{
	char envp_ext[6][MAX_ENV_LEN];
	char *envp[6] = {NULL};
	int idx = 0;
	pr_err(TAG"%s\n", __func__);
	snprintf(envp_ext[0], MAX_ENV_LEN, "subsystem=kernel_monitor_engine");
	snprintf(envp_ext[1], MAX_ENV_LEN, "exception=rt_throttle");
	snprintf(envp_ext[2], MAX_ENV_LEN, "process=%s", t->comm);
	snprintf(envp_ext[3], MAX_ENV_LEN, "processid=%d", task_pid_nr(t));
	snprintf(envp_ext[4], MAX_ENV_LEN, "cpu=%d", cpu);
	for (idx = 0; idx < 5; idx++) {
		envp[idx] = envp_ext[idx];
		pr_err(TAG"%s\n", envp[idx]);
	}
	return kernel_monitor_report(envp);
}
EXPORT_SYMBOL(kernel_monitor_report_rt_throttle);



static ssize_t enable_monitor_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return snprintf(buf, 2, "%d\n", enable_monitor);
}

static ssize_t enable_monitor_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int ret;

	ret = kstrtoint(buf, 10, &enable_monitor);
	if (ret < 0)
		return ret;

	return count;
}

/* Sysfs attributes cannot be world-writable. */
static struct kobj_attribute enable_monitor_attribute =
__ATTR(enable_monitor, 0664, enable_monitor_show, enable_monitor_store);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *attrs[] = {
	&enable_monitor_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

int  kme_merge_group(const struct attribute_group *group)
{
	if (kme_kobj) {
		return sysfs_create_group(kme_kobj, group);
	}
	return 0;
}

void  kme_remove_group(const struct attribute_group *group)
{
	if (kme_kobj) {
		return sysfs_remove_group(kme_kobj, group);
	}
}

int kme_add_link_to_group(struct kobject *target_kobj,
				      const char *target_name)
{
	if (kme_kobj) {
		return __compat_only_sysfs_link_entry_to_kobj(kme_kobj,
				target_kobj, target_name);
	}
	return 0;
}

void async_func(struct work_struct *work)
{
	if (enable_monitor) {
		kobject_uevent_env(kme_kobj, KOBJ_CHANGE, g_envp_ext);
	}
}
static int __init bsp_stability_init(void)
{
	int retval = 0;

	kme_kobj = kobject_create_and_add("kme", kernel_kobj);
	if (!kme_kobj)
		return -ENOMEM;

	retval = kme_merge_group(&attr_group);
	if (retval)
		kobject_put(kme_kobj);

	spin_lock_init(&g_lock);
	INIT_WORK(&async_work, async_func);

	return retval;
}

static void __exit bsp_stability_exit(void)
{
	kme_remove_group(&attr_group);
	kobject_put(kme_kobj);
}

subsys_initcall(bsp_stability_init);

module_exit(bsp_stability_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("luochucheng <luochucheng@vivo.com>");
