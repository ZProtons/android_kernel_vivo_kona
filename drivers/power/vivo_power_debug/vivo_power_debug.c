
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define POWER_MONITOR_PERIOD_MS	5000
#define DRV_NAME "vivo_power_debug"

int power_debug_switch = 1;
struct delayed_work power_debug_work;

//extern int msm_show_resume_irq_mask; //used to print the resume irq


extern void global_print_active_locks(void);

static void power_debug_work_func(struct work_struct *work)
{

	//print wakelocks
	global_print_active_locks();
	//wakelock_stats_show_debug();
	schedule_delayed_work(&power_debug_work,
			  round_jiffies_relative(msecs_to_jiffies
						(POWER_MONITOR_PERIOD_MS)));

}

static int power_debug_work_control(int on)
{
	int ret;

	if (on == 1) {
		if (power_debug_switch == 1) {
			ret = 1;
		} else {
			power_debug_switch = 1;
			//msm_show_resume_irq_mask=1;
			INIT_DELAYED_WORK(&power_debug_work,  power_debug_work_func);
			schedule_delayed_work(&power_debug_work,
			  round_jiffies_relative(msecs_to_jiffies
						(POWER_MONITOR_PERIOD_MS)));
			pr_info("enable power_debug_work.\n");
		}
	} else {
		if (power_debug_switch == 0) {
			ret = 1;
		} else {
			power_debug_switch = 0;
			//msm_show_resume_irq_mask=0;
			cancel_delayed_work(&power_debug_work);         
			pr_info("disable power_debug_work.\n");
		}

	}
	return ret;
}


static ssize_t po_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	snprintf(buf, sizeof(buf), "%u\n", power_debug_switch);
	return 1;
}
static ssize_t po_info_store(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t count)
{

	unsigned int val;

	if (sscanf(buf, "%u", &val) == 1) {
		if (power_debug_work_control(val))
			return count;
	}
	return -EINVAL;
}

static DEVICE_ATTR(switch, 0644, po_info_show, po_info_store);
static struct kobject *po_kobject;

static int power_debug_init(void)
{
	int ret;

	po_kobject = kobject_create_and_add(DRV_NAME, NULL);
	if (po_kobject == NULL) {
		ret = -ENOMEM;
		goto err1;
	}

	ret = sysfs_create_file(po_kobject, &dev_attr_switch.attr);
	if (ret)
		goto err;

	//if(power_debug_switch) {
	 //msm_show_resume_irq_mask=1; //on in default, deleted is allow.
	//}

	INIT_DELAYED_WORK(&power_debug_work,  power_debug_work_func);

	if (power_debug_switch == 1) {
	  //msm_show_resume_irq_mask=1; //on in default, deleted is allow.
	  schedule_delayed_work(&power_debug_work,
			  round_jiffies_relative(msecs_to_jiffies
						(POWER_MONITOR_PERIOD_MS)));
	}    
	return 0;

err:
	kobject_del(po_kobject);
err1:
	pr_info("Failed to create sys file\n");
	return ret;
}

static void __exit power_debug_exit(void)
{
	sysfs_remove_file(po_kobject,
						&dev_attr_switch.attr);
	power_debug_work_control(0);
}

module_init(power_debug_init);

module_exit(power_debug_exit);

MODULE_AUTHOR("vivo-hdl");
MODULE_DESCRIPTION("vivo power debug driver");
MODULE_LICENSE("GPL");

