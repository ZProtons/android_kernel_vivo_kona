/*
 * kme/irq_storm.c
 *
 * VIVO Kernel Monitor Engine(Irq Storm)
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 * rongqianfeng@vivo.com>
*/
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
#include <linux/typecheck.h>
#include <linux/percpu-defs.h>
#include <linux/kobject.h>
#include <linux/seq_file.h>
#include <linux/ratelimit.h>

#undef TAG
#define TAG "irq_storm: "
#define MAX_ENV_LEN 256
#define EXPIRES_SEC 10   /*default interval 10sec*/
#define DEFAULT_RATE 1000 /*per sec irq rate*/

static unsigned int expires_sec = EXPIRES_SEC;
static unsigned long abn_rate = DEFAULT_RATE * EXPIRES_SEC;
static int enable = 1;
static bool is_first_handler = true;
static struct timer_list irq_timer;
extern unsigned int bsp_test_mode;
static DEFINE_RATELIMIT_STATE(irq_storm, DEFAULT_RATELIMIT_INTERVAL,
								DEFAULT_RATELIMIT_BURST);

#define __overflow_sub(_a, _b, _c)			\
	do {									\
		if (typecheck(unsigned long, _a) &&	\
			typecheck(unsigned long, _b))	\
		_c = (long)((_a) - (_b));			\
	} while (0)

extern int kernel_monitor_report(char *envp_ext[]);
extern int kme_merge_group(const struct attribute_group *);
extern void  kme_remove_group(const struct attribute_group *);

bool irq_storm_monitor_is_enable(void)
{
	if (unlikely(enable && bsp_test_mode))
		return true;
	else
		return false;
}

unsigned int irq_storm_monitor_get_expires(void)
{
	return expires_sec;
}

static int kernel_report_irq_storm(int irq, unsigned long rate, unsigned long count)
{
	char envp_ext[6][MAX_ENV_LEN];
	char *envp[6] = {NULL};
	int i, index = 0;

	snprintf(envp_ext[index++], MAX_ENV_LEN, "subsystem=kernel_monitor_engine");
	snprintf(envp_ext[index++], MAX_ENV_LEN, "exception=irq_storm");
	snprintf(envp_ext[index++], MAX_ENV_LEN, "irq_number=%d", irq);
	snprintf(envp_ext[index++], MAX_ENV_LEN, "abn_rate=%lu/%ds", rate, expires_sec);
	snprintf(envp_ext[index++], MAX_ENV_LEN, "irqs_count=%lu", count);

	for (i = 0; i < index; i++) {
		envp[i] = envp_ext[i];
		if (__ratelimit(&irq_storm))
			pr_err(KERN_ERR "irq_storm:[ERROR] %s\n", envp[i]);
	}
	return kernel_monitor_report(envp);
}

static void irq_storm_handler(unsigned long time)
{
	int irq, j;
	struct irq_desc *desc;
	unsigned long flags, count, rate;

	if (!irq_storm_monitor_is_enable())
		return;

	pr_info(TAG"in irq_storm_handler\n");

	for_each_irq_desc(irq, desc) {
		count = 0;
		raw_spin_lock_irqsave(&desc->lock, flags);
		for_each_online_cpu(j)
			count += kstat_irqs_cpu(irq, j);
		if (!count) {
			raw_spin_unlock_irqrestore(&desc->lock, flags);
			continue;
		}

		/*let the irqs rate eq 0 after enable timer*/
		if (is_first_handler) {
			desc->irqs_rate_sum = 0;
			desc->kstat_irqs_sum = count;
			raw_spin_unlock_irqrestore(&desc->lock, flags);
			continue;
		}
		/*assumed the sum of irqs of all cups will overflow*/
		__overflow_sub(count, desc->kstat_irqs_sum, desc->irqs_rate_sum);
		desc->kstat_irqs_sum = count;
		rate = desc->irqs_rate_sum;
		raw_spin_unlock_irqrestore(&desc->lock, flags);

		if ((rate > abn_rate) && (irq > 100))
			kernel_report_irq_storm(irq, rate, count);
	}
	if (is_first_handler)
		is_first_handler = false;

	pr_info(TAG"out irq_storm_handler\n");

	if (!timer_pending(&irq_timer))
		mod_timer(&irq_timer, jiffies + msecs_to_jiffies(expires_sec * 1000));
}

static void clean_desc_rate(void)
{
	int irq;
	struct irq_desc *desc;
	unsigned long flags;

	for_each_irq_desc(irq, desc) {
		raw_spin_lock_irqsave(&desc->lock, flags);
		desc->irqs_rate_sum = 0;
		desc->kstat_irqs_sum = 0;
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}

static ssize_t rate_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", abn_rate);
}

static ssize_t rate_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int ret;

	ret = kstrtoul(buf, 10, &abn_rate);
	if (ret <= 0) {
		abn_rate = DEFAULT_RATE;
		return ret;
	}

	return count;
}

static ssize_t enable_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", enable);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	int temp_enable;

	ret = kstrtoint(buf, 10, &temp_enable);

	if (temp_enable < 0)
		return ret;
	if (!temp_enable) {
		enable = 0;
		del_timer_sync(&irq_timer);
		clean_desc_rate();
	} else {
		enable = 1;
		if (!timer_pending(&irq_timer) && irq_storm_monitor_is_enable()) {
			is_first_handler = true;
			mod_timer(&irq_timer, jiffies + msecs_to_jiffies(expires_sec * 1000));
		}
	}

	return count;
}

static ssize_t expires_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", expires_sec);
}

static ssize_t expires_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	unsigned int temp_expires;

	ret = kstrtoint(buf, 10, &temp_expires);

	if (temp_expires <= 0)
		return ret;
	else
		expires_sec = temp_expires;

	abn_rate = DEFAULT_RATE * expires_sec;

	return count;
}

static struct kobj_attribute rate_attr =
	__ATTR(abnormal_rate, 0640, rate_show, rate_store);
static struct kobj_attribute enable_attr =
	__ATTR(enable, 0640, enable_show, enable_store);
static struct kobj_attribute expires_attr =
	__ATTR(expires_sec, 0640, expires_show, expires_store);

static struct attribute *irq_attrs[] = {
	&rate_attr.attr,
	&enable_attr.attr,
	&expires_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name = "irq",
	.attrs = irq_attrs,
};

static void irq_timer_setup(unsigned char nsec)
{
	init_timer(&irq_timer);
	irq_timer.function = irq_storm_handler;
	irq_timer.expires = jiffies + HZ * nsec;
	add_timer(&irq_timer);
	is_first_handler = true;
}

static int __init irq_storm_mnt_init(void)
{
	int ret;

	ret = kme_merge_group(&attr_group);
	if (ret)
		return ret;

	if (irq_storm_monitor_is_enable())
		irq_timer_setup(expires_sec);

	return ret;
}

static void __exit irq_storm_mnt_exit(void)
{
	del_timer_sync(&irq_timer);
	clean_desc_rate();
	kme_remove_group(&attr_group);
}

module_init(irq_storm_mnt_init);
module_exit(irq_storm_mnt_exit);
MODULE_LICENSE("GPL v2");
