

#define pr_fmt(fmt) "haptic_core: %s:" fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "vivo_haptic_core.h"

static DEFINE_MUTEX(haptic_lock);
static LIST_HEAD(haptic_list);
static DEFINE_SPINLOCK(haptic_list_lock);

//static atomic_t currentUserRef;

static int haptic_effect_from_user(struct haptic_effect *event, const char __user *buffer)
{
	if (copy_from_user(event, buffer, sizeof(struct haptic_effect)))
		return -EFAULT;

	return 0;
}

static int haptic_write_event_from_user(struct haptic_write_event *event, const char __user *buffer)
{
	if (copy_from_user(event, buffer, sizeof(struct haptic_write_event)))
		return -EFAULT;

	return 0;
}


/*******************************************************************************************
 *
 * haptic fops
 *
 *******************************************************************************************/
static int haptic_open(struct inode *inode, struct file *file)
{
	struct haptic_misc *hap_misc_dev = NULL;
	struct haptic_device *hp = NULL;


	file->private_data = NULL;

	spin_lock(&haptic_list_lock);
	list_for_each_entry(hap_misc_dev, &haptic_list, list) {

		if (hap_misc_dev->devt == inode->i_rdev) {
			pr_err(" dev_t_hap=%#x, dev_t_inode=%#x, dev struct found\n", hap_misc_dev->devt, inode->i_rdev);
			file->private_data = hap_misc_dev;
			hp = (struct haptic_device *)hap_misc_dev->private_data;
			break;
		}

	}
	spin_unlock(&haptic_list_lock);


	if (!(file->private_data) || !(hp)) {
		pr_err("no device found\n");
		return -ENODEV;
	}

	pr_err("hap misc dev open count = %d\n", hap_misc_dev->open);
	if (!hap_misc_dev->open) {

		if (hp->init_dev) {
			hp->init_dev(hp);
		}

		hap_misc_dev->open++;

	}

	return 0;
}

static ssize_t haptic_read(struct file *file, char __user *buff, size_t len, loff_t *offset)
{

	return len;
}

/* 用于加载效果和开始播放振动 */
static ssize_t haptic_write(struct file *file, const char __user *buff, size_t len, loff_t *off)
{

	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	struct haptic_device *hp = (struct haptic_device *)hap_misc_dev->private_data;
	int ret = 0;
	struct haptic_write_event event;

	pr_err("enter, len = %d\n", len);

	ret = mutex_lock_interruptible(&hap_misc_dev->lock);
	if (ret) {
		pr_err("lock not correct\n");
		return ret;
	}

	if (haptic_write_event_from_user(&event, buff)) {
		pr_err("write copy form user error\n");
		mutex_unlock(&hap_misc_dev->lock);
		return -EFAULT;
	}

	switch (event.type) {

	case HAPTIC_PLAY_EVENT:
		ret = hp->playback(hp, 1);
		if (ret < 0) {
			pr_err("playback failed, ret=%d\n", ret);
			mutex_unlock(&hap_misc_dev->lock);
			return -EFAULT;
		}
		break;

	case HAPTIC_GAIN_EVENT:
		hp->set_gain(hp, event.value);
		break;

	default:
		pr_err("haptic write invalid type\n");
	}

	mutex_unlock(&hap_misc_dev->lock);

	return len;
}

static int haptic_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	return 0;
}

static int check_effect_access(struct haptic_misc *hap_misc_dev, int user_id, struct file *file)
{

	return 0;
}

static long haptic_do_ioctl(struct file *file, unsigned int cmd, void __user *p, int compat_mode)
{
	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	struct haptic_device *hp = (struct haptic_device *)hap_misc_dev->private_data;
	struct haptic_effect effect;

	uint8_t __user *ip = (uint8_t __user *)p;
	int user_id = (int)(unsigned long)p;
	int ret = 0;
	int id = 0;

	pr_err("cmd=0x%x, arg=0x%lx, misc_dev_pointer=%p\n", cmd, p, hap_misc_dev);


	if (_IOC_TYPE(cmd) != HAPTIC_IOCTL_MAGIC) {
		pr_err("cmd magic err\n");
		return -EINVAL;
	}


	switch (cmd) {

	case HAPTIC_UPLOAD:

		if (haptic_effect_from_user(&effect, p)) {
			pr_err("upload copy not correct\n");
			return -EFAULT;
		}

		if (effect.id == -1) {

			for (id = 0; id < hap_misc_dev->max_effects; id++)
				if (!hap_misc_dev->effect_owners[id])
					break;

			if (id >= hap_misc_dev->max_effects) {
				pr_err("no spec correct\n");
				return -ENOSPC;
			}

			effect.id = id;
			pr_err("alloc effect id = %d, max_effects = %d\n", effect.id, hap_misc_dev->max_effects);
			
		} else {

			pr_err("effect id is not zero\n");
		}

		ret = hp->upload(hp, &effect);
		if (ret < 0) {
			pr_err("upload effect failed, ret=%d\n", ret);
			return -EFAULT;
		}

		hap_misc_dev->effect_owners[id] = file;

		if (put_user(effect.id, &(((struct haptic_effect __user *)p)->id))) {
			pr_err("put user not correct\n");
			return -EFAULT;
		}

		break;

	case HAPTIC_PLAYBACK:
		break;

	case HAPTIC_STOP: //停止振动
		pr_err("haptic stop id = %d\n", user_id);
		ret = check_effect_access(hap_misc_dev, user_id, file);
		if (ret) {
			pr_err("stop not access\n");
			return ret;
		}
		hap_misc_dev->effect_owners[user_id] = NULL;
		ret = hp->erase(hp);
		if (ret < 0) {
			pr_err("stop play failed, ret=%d\n", ret);

			return -EFAULT;
		}
		break;

	case HAPTIC_GAIN:
		break;

	case HAPTIC_SUPPORT_BITMASK:
		put_user(hp->hap_bit[0], ip);
		break;

	case HAPTIC_TRIGGER_INTENSITY:
		hp->set_trigger_intensity(hp, user_id);
		break;

	default:
		pr_err("unknown cmd\n");
		break;

	}

//	pr_debug("---->ret=%d\n", ret);
	return ret;
}

static long haptic_ioctl_handler(struct file *file, unsigned int cmd, void __user *p, int compat_mode)
{
	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	int retval;

	retval = mutex_lock_interruptible(&hap_misc_dev->lock);
	if (retval)
		return retval;

	retval = haptic_do_ioctl(file, cmd, p, compat_mode);

	mutex_unlock(&hap_misc_dev->lock);
	return retval;
}

static long haptic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return haptic_ioctl_handler(file, cmd, (void __user *)arg, 0);
}

#ifdef CONFIG_COMPAT
static long haptic_ioctl_compat(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)file->private_data;
	struct haptic_device *hp = (struct haptic_device *)hap_misc_dev->private_data;
	int ret = 0;


	pr_debug("32bit compat mode, to be deal\n");

	// stop vibrator direct

	mutex_lock_interruptible(&hap_misc_dev->lock);
	ret = hp->erase(hp);
	mutex_unlock(&hap_misc_dev->lock);

	return -EPERM;
	//return haptic_ioctl_handler(file, cmd, compat_ptr(arg), 1);
}
#endif


static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = haptic_open,
	.read = haptic_read,
	.write = haptic_write,
	.unlocked_ioctl = haptic_ioctl,

#ifdef CONFIG_COMPAT
	.compat_ioctl = haptic_ioctl_compat,
#endif

	.release = haptic_release,
};

/************************************************************************************
 * func:  register a haptic misc device for vibrator
 * @name: haptic misc device name(matching hidl)
 * @hp:   vibrator control interface struct
 *
 ***********************************************************************************/

int haptic_miscdev_register(const char *name, struct haptic_device *hp)
{
	int ret = 0;
	int max_effects = 16;

	struct haptic_misc *hap_misc_dev;

	pr_info("enter, hp=%p, name=%s\n", hp, name);

	mutex_lock(&haptic_lock);
	hap_misc_dev = devm_kzalloc(hp->dev, sizeof(struct haptic_misc) + max_effects * sizeof(struct file *), GFP_KERNEL);
	if (!hap_misc_dev) {
		pr_err("kzalloc failed\n");
		mutex_unlock(&haptic_lock);
		return -ENOMEM;
	}

	hap_misc_dev->hap_dev.fops = &fops;
	hap_misc_dev->hap_dev.minor = MISC_DYNAMIC_MINOR;
	hap_misc_dev->hap_dev.name = name;

	ret = misc_register(&hap_misc_dev->hap_dev);
	if (ret) {
		pr_err("misc register fail, ret=%d\n", ret);
		devm_kfree(hp->dev, hap_misc_dev);
		hap_misc_dev = NULL;
		mutex_unlock(&haptic_lock);
		return ret;
	}

	hap_misc_dev->max_effects = max_effects;
	hap_misc_dev->private_data = hp;
	hap_misc_dev->devt = MKDEV(MISC_MAJOR, hap_misc_dev->hap_dev.minor);
	hp->private = hap_misc_dev;


	INIT_LIST_HEAD(&hap_misc_dev->list);
	mutex_init(&hap_misc_dev->lock);

	spin_lock(&haptic_list_lock);
	list_add(&hap_misc_dev->list, &haptic_list);
	spin_unlock(&haptic_list_lock);

	pr_info("end, hap_misc_dev=%p, name=%s\n", hap_misc_dev, name);

	mutex_unlock(&haptic_lock);
	return 0;

}

EXPORT_SYMBOL_GPL(haptic_miscdev_register);


int haptic_miscdev_unregister(struct haptic_device *hp)
{
	struct haptic_misc *hap_misc_dev = (struct haptic_misc *)hp->private;

	if (!hap_misc_dev) {
		misc_deregister(&hap_misc_dev->hap_dev);

		spin_lock(&haptic_list_lock);
		list_del(&hap_misc_dev->list);
		spin_unlock(&haptic_list_lock);
	}
	return 0;

}

EXPORT_SYMBOL_GPL(haptic_miscdev_unregister);

static int __init haptic_miscdev_init(void)
{
	return 0;
}

device_initcall(haptic_miscdev_init);
MODULE_LICENSE("GPL");
