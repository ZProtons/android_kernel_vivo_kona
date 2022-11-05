/***************************************************************************************
  wangyuanliang created @ 2013/4/17
  duanxufang  changed for SDM439, update to v2.0  2018/8/7
***************************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/bbk_drivers_info.h>

#include <soc/qcom/socinfo.h>

#include <linux/of_gpio.h>
#include <linux/delay.h>

#ifdef CONFIG_VIVO_REGDUMP
#include <linux/vivo-regdump.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#endif

struct bbk_driver_callback {
	struct list_head list;
	struct bbk_drivers_callback_handler *handler;
} drivers_callbacks;

#ifdef CONFIG_VIVO_REGDUMP
static bool do_regdump = false;
static unsigned long last_press = 0;
static struct delayed_work vivo_regdump_dwork;
#endif

static unsigned int vivo_snd_card_info[2] = {0, 0};

int bbk_drivers_log_switch_register_callback(struct bbk_drivers_callback_handler *handler) 
{
	struct bbk_driver_callback *new_callback = NULL;
	
	new_callback = kzalloc(sizeof(struct bbk_driver_callback), GFP_KERNEL);
	if (!new_callback) {
		pr_err("%s: Failed at allocate callback struct\n", __func__);
		return -ENOMEM;
	}

	new_callback->handler = handler;
	INIT_LIST_HEAD(&new_callback->list);
	list_add_tail(&new_callback->list, &drivers_callbacks.list);

	return 0;
}
EXPORT_SYMBOL_GPL(bbk_drivers_log_switch_register_callback);

void bbk_drivers_log_switch_unregister_callback(char *callback_name)
{
	struct bbk_driver_callback *entry;

	if (!list_empty(&drivers_callbacks.list)) {
		list_for_each_entry(entry, &drivers_callbacks.list, list)
			if (!strcmp(entry->handler->name, callback_name)) {
				list_del(&entry->list);
				kfree(entry);
				return;
			}
	}
}
EXPORT_SYMBOL_GPL(bbk_drivers_log_switch_unregister_callback);

void bbk_drivers_log_switch_do_callback(bool is_siwtch_on)
{
	struct bbk_driver_callback *entry;

	pr_err("%s: called(%d)\n", __func__, is_siwtch_on);
	if (!list_empty(&drivers_callbacks.list)) {
		list_for_each_entry(entry, &drivers_callbacks.list, list)
			entry->handler->callback(is_siwtch_on);
	}
}

static struct volume_key_trace_info {
	int key_traced_number;
	unsigned long first_down_time;
} volume_key_trace;

static bool has_switched_on = false;

static void bbk_drivers_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	unsigned long now = jiffies;
	unsigned long max_time_length = 3 * HZ; 

#ifdef CONFIG_VIVO_REGDUMP
	if (code == KEY_VOLUMEUP) {
		if (value) {
			do_regdump = true;
			last_press = now;
			schedule_delayed_work(&vivo_regdump_dwork,
								msecs_to_jiffies(3200));
		} else {
			do_regdump = false;
			/* cancel_delayed_work_sync(&vivo_regdump_dwork); */
		}
	}
#endif

	if (value == 0) {
		return;
	}

	if (now - volume_key_trace.first_down_time < max_time_length) {
		switch (volume_key_trace.key_traced_number) {
			case 0:
				/* should not happen */
				if (code == KEY_VOLUMEUP) {
					volume_key_trace.key_traced_number++;
				} else {
					/* nothing to do */
				}
			break;
		case 1:
			if (code == KEY_VOLUMEUP) {
				if (has_switched_on) {
					bbk_drivers_log_switch_do_callback(false);
					has_switched_on = false;
				}
				volume_key_trace.key_traced_number = 0;
				volume_key_trace.first_down_time = 0;
			} else if (code == KEY_VOLUMEDOWN) {
				volume_key_trace.key_traced_number++;
			} else {
				/* should not happen */
			}
			break;
		case 2:
			if (code == KEY_VOLUMEUP) {
				volume_key_trace.key_traced_number++;
			} else if (code == KEY_VOLUMEDOWN) {
				volume_key_trace.key_traced_number = 0;
				volume_key_trace.first_down_time = 0;
			}
			break;
		case 3:
			if (code == KEY_VOLUMEUP) {
				volume_key_trace.key_traced_number = 1;
				volume_key_trace.first_down_time = now;
			} else {
				if (!has_switched_on) {
					bbk_drivers_log_switch_do_callback(true);
					/* for depend mistouch do not clear trace info
						so it can't turn off log immediately in this 3s
					*/
					volume_key_trace.key_traced_number++;
					has_switched_on = true;
				}
			}
			break;
		default:
			/* nothing to do */
			break;
		}
	} else {
		if (code == KEY_VOLUMEUP) {
			volume_key_trace.key_traced_number = 1;
			volume_key_trace.first_down_time = now;
			//has_switched_on = false;//shijianxing M
		} else {
			/* nothing to do */
		}
	}
}

static int bbk_drivers_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{	
	struct input_handle *handle;
	int error = -1;

	pr_err("%s: input device %s connecting\n", __func__, dev->name);

	if ((strcmp(dev->name, "gpio-keys")) && (strcmp(dev->name, "qpnp_pon")))  {
		pr_err("%s: %s: Not volume key input device\n", 
					__func__, dev->name);
		return error;
	}

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "bbk_drivers_log";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void bbk_drivers_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id bbk_drivers_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler bbk_drivers_input_handler = {
	.event		= bbk_drivers_input_event,
	.connect	= bbk_drivers_input_connect,
	.disconnect	= bbk_drivers_input_disconnect,
	.name		= "bbk_drivers_log_siwtch",
	.id_table	= bbk_drivers_ids,
};

static struct bbk_devices_info_list {
	struct list_head list;
	struct bbk_device_info dev_info;
} devices_list;

int bbk_driver_register_device_info(char *device_type, char *device_name) 
{
	struct bbk_devices_info_list *new_info;
	int type_length, name_length;

	type_length = strlen(device_type);
	name_length = strlen(device_name);
	if (type_length > NAME_LENGTH || name_length > NAME_LENGTH) {
		pr_err("%s: Invalid device_type or device_name\n", __func__);
		return -EINVAL;
	}

	new_info = kzalloc(sizeof(struct bbk_devices_info_list), GFP_KERNEL);
	if (!new_info) {
		pr_err("%s: Can't allocat new device info list\n", __func__);
		return -ENOMEM;
	}

	strlcpy(new_info->dev_info.device_type, device_type, 
		sizeof(new_info->dev_info.device_type));
	strlcpy(new_info->dev_info.device_name, device_name, 
		sizeof(new_info->dev_info.device_name));
	INIT_LIST_HEAD(&new_info->list);
	list_add_tail(&new_info->list, &devices_list.list);

	return 0;
}
EXPORT_SYMBOL_GPL(bbk_driver_register_device_info);

void bbk_driver_unregister_device_info(char *device_type, char *device_name)
{
	struct bbk_devices_info_list *entry;

	if (!list_empty(&devices_list.list)) {
		list_for_each_entry(entry, &devices_list.list, list)
			if (!strcmp(entry->dev_info.device_type, device_type) 
					&& !strcmp(entry->dev_info.device_name, device_name)) {
				list_del(&entry->list);
				kfree(entry);
			}
	}
}
EXPORT_SYMBOL_GPL(bbk_driver_unregister_device_info);

static char bbk_board_version[16] = "version_invalid";
static char bbk_model_version[64] = "version_invalid";

char *get_bbk_board_version(void)
{
	return bbk_board_version;
}
EXPORT_SYMBOL_GPL(get_bbk_board_version);

int get_bbk_hw_subtype(void)
{
	struct device_node *root;
	int subtype[2];

	root = of_find_node_by_path("/");
	if (!root) {
		pr_err(" Can't find root of device tree\n");
		return -ENODEV;
	}
	
	if (of_property_read_u32_array(root, "qcom,board-id", subtype, 2) != 0) {
		pr_err(" Can't find prop qcom,board-id \n");
		return -ENODEV;
	}
	
	return subtype[1];
}
EXPORT_SYMBOL_GPL(get_bbk_hw_subtype);


/* liukangfei add for pd1624
 *return 1 mean tp dvddh 3.3v
 *return 0 mean tp dvddh 3v */
int bbk_touch_voltage_detect(void)
{
	if (bbk_board_version[1] == '1')
		return 1;
	else if (bbk_board_version[1] == '0')
		return 0;
		
	return 1;
}
EXPORT_SYMBOL_GPL(bbk_touch_voltage_detect);

static __init int set_bbk_board_version(char *str)
{
	strlcpy(bbk_board_version, str, sizeof(bbk_board_version));
	pr_err("bbk board version is %s\n", bbk_board_version);
	return 0;
}
early_param("bbk_board_version", set_bbk_board_version);

static __init int set_bbk_model_version(char *str)
{
	strlcpy(bbk_model_version, str, sizeof(bbk_model_version));
	pr_err("bbk model version is %s\n", bbk_model_version);
	return 0;
}
early_param("bbk_model_version", set_bbk_model_version);

static char mpp_char_val[16] = "00";
static __init int set_mpp_char_val(char *str)
{
	strlcpy(mpp_char_val, str, sizeof(mpp_char_val));
	pr_err("mpp char is %s\n", mpp_char_val);
	return 0;
}
early_param("vol_mpp_char", set_mpp_char_val);

static char mpp_dig_val[16] = "00";
static __init int set_mpp_dig_val(char *str)
{
	strlcpy(mpp_dig_val, str, sizeof(mpp_dig_val));
	pr_err("mpp digital is %s\n", mpp_dig_val);
	return 0;
}
early_param("vol_mpp_dig", set_mpp_dig_val);

static ssize_t devices_board_vsersion_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", bbk_board_version);
}

static ssize_t devices_mpps_value_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", bbk_model_version);
}

static ssize_t devices_model_value_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", bbk_model_version);
}

static ssize_t devices_list_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	struct bbk_devices_info_list *entry;
	int count = 0;

	if (!list_empty(&devices_list.list)) {
		/* count += sprintf(&buf[count], "%-16.s%-16.s\n", "Device type", "Device name"); */
		list_for_each_entry(entry, &devices_list.list, list) {
			count += snprintf(&buf[count], PAGE_SIZE, "A = ");
			count += snprintf(&buf[count], PAGE_SIZE, "%s\n", entry->dev_info.device_type);
			/* count += sprintf(&buf[count], "%-16.s\n", entry->dev_info.device_name); */
		}
	} else {
		count += snprintf(buf, PAGE_SIZE, "Device list is empty\n");
	}

	return count;
}

int set_vivo_snd_card_info(unsigned int mask, unsigned int value)
{
	unsigned int temp = 0;
	
	temp = vivo_snd_card_info[0];
	temp &= (~mask);
	temp |= (value & mask);
	vivo_snd_card_info[0] = temp;
	return 0;
}
EXPORT_SYMBOL_GPL(set_vivo_snd_card_info);

int set_vivo_fm_info(unsigned int mask, unsigned int value)
{
	unsigned int temp = 0;
	
	temp = vivo_snd_card_info[1];
	temp &= (~mask);
	temp |= (value & mask);
	vivo_snd_card_info[1] = temp;
	return 0;
}
EXPORT_SYMBOL_GPL(set_vivo_fm_info);


static ssize_t snd_card_info_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	int count = 0;
	
	count = sizeof(vivo_snd_card_info);
	pr_info("%s, %u, %u, %d\n", __func__, vivo_snd_card_info[0],
		vivo_snd_card_info[1], count);
	memcpy(buf, (char *)(&vivo_snd_card_info), count);
	pr_info("%s() 0x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x\n", __func__, buf[0],
		buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	
	return count;
}


static struct debug_sysfs_entry bbk_devices_list = 
	__ATTR(devices_list, S_IRUGO, 
			devices_list_show, NULL);
static struct debug_sysfs_entry board_version = 
	__ATTR(board_version, S_IRUGO, 
			devices_board_vsersion_show, NULL);
static struct debug_sysfs_entry mpps_value = 
	__ATTR(mpps_value, S_IRUGO, 
			devices_mpps_value_show, NULL);
static struct debug_sysfs_entry model_value = 
	__ATTR(model_value, S_IRUGO, 
			devices_model_value_show, NULL);
static struct debug_sysfs_entry snd_card_info = 
	__ATTR(snd_card_info, S_IRUGO, 
			snd_card_info_show, NULL);

static struct attribute *sys_attrs[] = {
	&bbk_devices_list.attr,
	&board_version.attr,
	&mpps_value.attr,
	&model_value.attr,
	&snd_card_info.attr,
	
	NULL
};

static ssize_t debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t debug_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops debug_object_sysfs_ops = {
	.show = debug_object_show,
	.store = debug_object_store,
};
static struct kobj_type debug_object_type = {
	.sysfs_ops	= &debug_object_sysfs_ops,
	.release	= debug_object_release,
	.default_attrs = sys_attrs,
};

static struct kobject kobject_debug;

static int creat_sys_files(void) 
{ 
	int ret = 0; 
	
	ret = kobject_init_and_add(&kobject_debug, &debug_object_type,
					NULL, "devs_list");
	if (ret) { 
		pr_err("%s: Create kobjetct error!\n", __func__);
		return ret;
	}
	return 0;
}

int devs_create_sys_files(const struct attribute *attr)
{
	int ret = 0;
	
	ret = sysfs_create_file(&kobject_debug, attr);
	if (ret) {
		pr_err("%s: Create %s sys files error!", __func__, attr->name);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(devs_create_sys_files);


#ifdef CONFIG_VIVO_REGDUMP
static void vivo_audio_regdump(struct work_struct *work)
{
	if (do_regdump && (jiffies - last_press >= 3*HZ))
		vivo_audio_regdump_do_callback();
}
#endif

static int __init bbk_drivers_log_switch_init(void)
{
	int error;
	
#ifdef CONFIG_VIVO_REGDUMP
	INIT_DELAYED_WORK(&vivo_regdump_dwork, vivo_audio_regdump);
#endif
	INIT_LIST_HEAD(&drivers_callbacks.list);
	volume_key_trace.key_traced_number = 0;
	volume_key_trace.first_down_time = jiffies;
	error = input_register_handler(&bbk_drivers_input_handler);
	if (error) {
		pr_err("%s: register input handler failed\n", __func__);
		return error;
	}
	
	INIT_LIST_HEAD(&devices_list.list);
	error = creat_sys_files();
	if (error) {
		pr_err("%s: creat sysfs files failed\n", __func__);
		return error;
	}
	return 0;
}

static void __exit bbk_drivers_log_switch_exit(void)
{
	/*nothing to do */
}

arch_initcall(bbk_drivers_log_switch_init);
module_exit(bbk_drivers_log_switch_exit);
