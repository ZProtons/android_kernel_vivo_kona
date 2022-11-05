#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>
#include <linux/pmic-voter.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
#include <linux/extcon-provider.h>
#else
#include <linux/extcon.h>
#endif

extern ssize_t dwc3_msm_otg_mode(void *mdwc, char *buf);
extern int dwc3_msm_otg_vbus(void *mdwc);

#define DWC3_VUSB_HOST_DISABLED_TIME (300*1000)
#define DWC3_VUSB_DETECT_CYCLE_TIME (500)

#define DWC3_VUSB_ID_DEBOUNCE_MS 50
#define DWC3_VUSB_ID_DEBOUNCE_TIMES 5
#define DWC3_VUSB_ID_ADC_TIMES 10
#define DWC3_VUSB_ID_TRIGGER_MV 110
#define DWC3_VUSB_ID_TRIGGERED_MV 200
#define DWC3_VUSB_ID_DEFAULT_MV 1800

struct dwc3_vusb {
	struct device *dev;
	struct class *dwc3_vusb_class;
	struct device *dwc3_vusb_device;
	struct platform_device *usbc_pdev;
	struct power_supply *usb_psy;
	struct mutex switch_mutex;

	struct extcon_dev *edev;
	struct workqueue_struct *workq;
	struct wake_lock host_wake_lock;
	struct delayed_work host_disabled_work;
	int mode;

	struct delayed_work id_detect_work;
	struct iio_channel *id_adc_chan;
	int id_pull;
	int id_trigger_mv;
	int id_debounce;
	int id_times;
	int id_adc_times;
	bool id_state;

	struct delayed_work typec_detect_work;
	struct votable *drp_mode_votable;
	bool cc_state;
	bool typec_hw_det;

	bool host_enable;
};

static unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static int dwc3_vusb_read_id_voltage(struct dwc3_vusb *dwc3_vusb, int *temp)
{
	int ret = 0, total = 0, voltage, i;

	if (!dwc3_vusb->id_adc_chan) {
		dwc3_vusb->id_adc_chan = iio_channel_get(dwc3_vusb->dev, "id_pin");
		if (IS_ERR(dwc3_vusb->id_adc_chan)) {
			ret = PTR_ERR(dwc3_vusb->id_adc_chan);
			pr_err("%s: %d: get id_adc_chan fail: %d\n", __func__, __LINE__, ret);
			return ret;
		}
	}

	for (i = 0; i < dwc3_vusb->id_adc_times; i++) {
		ret = iio_read_channel_processed(dwc3_vusb->id_adc_chan, &voltage);
		if (ret < 0) {
			pr_err("%s: %d: read id adc fail: %d\n", ret);
			return ret;
		}
		voltage /= 1000;
		total += voltage;
	}

	*temp = total / dwc3_vusb->id_adc_times;

	return ret;
}

static ssize_t dwc3_vusb_type_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	ssize_t size;
	struct dwc3_vusb *dwc3_vusb = dev_get_drvdata(dev);

	if (dwc3_vusb->mode)
		size = snprintf(buf, PAGE_SIZE, "%s", "MicroB\n");
	else
		size = snprintf(buf, PAGE_SIZE, "%s", "TypeC\n");

	return size;
}

static ssize_t dwc3_vusb_type_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	pr_err("%s: %d: not support\n", __func__, __LINE__);
	return -EPERM;
}
static DEVICE_ATTR(usb_type, S_IRUGO | S_IWUSR, dwc3_vusb_type_show, dwc3_vusb_type_store);

static ssize_t dwc3_vusb_otg_mode_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct dwc3_vusb *dwc3_vusb = dev_get_drvdata(dev);
	ssize_t size;

	mutex_lock(&dwc3_vusb->switch_mutex);
	if (!dwc3_vusb->usbc_pdev) {
		pr_err("%s: %d: usbc pdev is null\n", __func__, __LINE__);
		mutex_unlock(&dwc3_vusb->switch_mutex);
		return -ENODEV;
	}

	size = dwc3_msm_otg_mode(dev_get_drvdata(&dwc3_vusb->usbc_pdev->dev), buf);

	/* Typec hw auto detect */
	if (dwc3_vusb->typec_hw_det) {
		if (!strncmp(buf, "peripheral", 10) || !strncmp(buf, "none", 4)) {
			union power_supply_propval pval = {0,};

			if (!dwc3_vusb->usb_psy) {
				dwc3_vusb->usb_psy = power_supply_get_by_name("usb");
				if (!dwc3_vusb->usb_psy) {
					pr_err("%s: %d: usb_psy is null\n", __func__, __LINE__);
					mutex_unlock(&dwc3_vusb->switch_mutex);
					return -ENODEV;
				}
			}
			power_supply_get_property(dwc3_vusb->usb_psy,
						  POWER_SUPPLY_PROP_USB_DET_PIN_MODE, &pval);
			if (pval.intval == USB_DET_PIN_MODE_DET)
				size = snprintf(buf, PAGE_SIZE, "%s", "ccopened\n");
		}
	}
	mutex_unlock(&dwc3_vusb->switch_mutex);

	return size;
}

static ssize_t dwc3_vusb_otg_mode_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t size)
{
	pr_err("%s: %d: not support\n", __func__, __LINE__);
	return -EPERM;
}
static DEVICE_ATTR(otg_mode, S_IRUGO | S_IWUSR, dwc3_vusb_otg_mode_show, dwc3_vusb_otg_mode_store);

static ssize_t dwc3_vusb_host_mode_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	ssize_t size;
	struct dwc3_vusb *dwc3_vusb = dev_get_drvdata(dev);

	mutex_lock(&dwc3_vusb->switch_mutex);
	if (dwc3_vusb->mode || dwc3_vusb->typec_hw_det) {
		/* Micro USB and Typec hw auto detect */
		if (dwc3_vusb->host_enable)
			size = snprintf(buf, PAGE_SIZE, "%s", "enabled\n");
		else
			size = snprintf(buf, PAGE_SIZE, "%s", "disabled\n");
	} else {
		/* Typec switch */
		union power_supply_propval pval = {0,};

		if (!dwc3_vusb->usb_psy) {
			dwc3_vusb->usb_psy = power_supply_get_by_name("usb");
			if (!dwc3_vusb->usb_psy) {
				pr_err("%s: %d: usb_psy is null\n", __func__, __LINE__);
				mutex_unlock(&dwc3_vusb->switch_mutex);
				return -ENODEV;
			}
		}
		power_supply_get_property(dwc3_vusb->usb_psy,
					  POWER_SUPPLY_PROP_TYPEC_SWITCH, &pval);
		if (pval.intval == 0)
			size = snprintf(buf, PAGE_SIZE, "%s", "enabled\n");
		else
			size = snprintf(buf, PAGE_SIZE, "%s", "disabled\n");
	}
	mutex_unlock(&dwc3_vusb->switch_mutex);

	return size;
}

static ssize_t dwc3_vusb_host_mode_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct dwc3_vusb *dwc3_vusb = dev_get_drvdata(dev);
	char value[32];
	bool is_enable;

	if (sscanf(buf, "%s", value) != 1)
		return -EINVAL;

	if (!strncmp(value, "enabled", 7))
		is_enable = true;
	else if (!strncmp(value, "disabled", 8))
		is_enable = false;
	else
		return -EINVAL;

	mutex_lock(&dwc3_vusb->switch_mutex);
	if (dwc3_vusb->mode || dwc3_vusb->typec_hw_det) {
		/* Micro USB and Typec hw auto detect */
		if (dwc3_vusb->host_enable == is_enable) {
			pr_info("%s: %d: already %d, ignore\n", __func__, __LINE__, dwc3_vusb->host_enable);
			mutex_unlock(&dwc3_vusb->switch_mutex);
			return size;
		}
	} else {
		/* Typec switch */
		union power_supply_propval pval = {0,};

		if (!dwc3_vusb->usb_psy) {
			dwc3_vusb->usb_psy = power_supply_get_by_name("usb");
			if (!dwc3_vusb->usb_psy) {
				pr_err("%s: %d: usb_psy is null\n", __func__, __LINE__);
				mutex_unlock(&dwc3_vusb->switch_mutex);
				return -ENODEV;
			}
		}
		power_supply_get_property(dwc3_vusb->usb_psy,
					  POWER_SUPPLY_PROP_TYPEC_SWITCH, &pval);
		if (pval.intval == !is_enable) {
			mutex_unlock(&dwc3_vusb->switch_mutex);
			return size;
		}
	}

	pr_info("%s: %d: otg %s\n", __func__, __LINE__, is_enable ? "enabled" : "disabled");

	if (dwc3_vusb->mode || dwc3_vusb->typec_hw_det) {
		/* Micro USB */
		if (dwc3_vusb->mode) {
			if (is_enable) {
				wake_lock(&dwc3_vusb->host_wake_lock);
				dwc3_vusb->id_trigger_mv = DWC3_VUSB_ID_TRIGGER_MV;
				dwc3_vusb->id_state = true;
				gpio_direction_output(dwc3_vusb->id_pull, 1);
				msleep(dwc3_vusb->id_debounce);
				queue_delayed_work(dwc3_vusb->workq, &dwc3_vusb->id_detect_work, 0);
				queue_delayed_work(dwc3_vusb->workq, &dwc3_vusb->host_disabled_work,
						   msecs_to_jiffies(DWC3_VUSB_HOST_DISABLED_TIME));
			} else {
				if (!dwc3_vusb->host_enable) {
					pr_info("%s: %d: otg already disabled\n", __func__, __LINE__);
					mutex_unlock(&dwc3_vusb->switch_mutex);
					return -EINVAL;
				}

				cancel_delayed_work(&dwc3_vusb->id_detect_work);
				cancel_delayed_work(&dwc3_vusb->host_disabled_work);

				if (!dwc3_vusb->id_state) {
					union power_supply_propval pval = {0,};

					extcon_set_state_sync(dwc3_vusb->edev, EXTCON_USB_HOST, false);
					if (!dwc3_vusb->usb_psy) {
						dwc3_vusb->usb_psy = power_supply_get_by_name("usb");
						if (!dwc3_vusb->usb_psy) {
							pr_err("%s: %d: usb_psy is null\n", __func__, __LINE__);
							mutex_unlock(&dwc3_vusb->switch_mutex);
							return -ENODEV;
						}
					}
					pval.intval = 0;
//					power_supply_set_property(dwc3_vusb->usb_psy,
//								  POWER_SUPPLY_PROP_OTG_VBUS_ENABLE, &pval);
				}
				gpio_direction_output(dwc3_vusb->id_pull, 0);
				msleep(dwc3_vusb->id_debounce);
				if (wake_lock_active(&dwc3_vusb->host_wake_lock))
					wake_unlock(&dwc3_vusb->host_wake_lock);
			}
			dwc3_vusb->host_enable = is_enable;
		}

		/* Typec hw auto detect */
		if (dwc3_vusb->typec_hw_det) {
			int vote_val = 0;

			if (!dwc3_vusb->usb_psy) {
				dwc3_vusb->usb_psy = power_supply_get_by_name("usb");
				if (!dwc3_vusb->usb_psy) {
					pr_err("%s: %d: usb_psy is null\n", __func__, __LINE__);
					mutex_unlock(&dwc3_vusb->switch_mutex);
					return -ENODEV;
				}
			}
			if (!dwc3_vusb->drp_mode_votable) {
				dwc3_vusb->drp_mode_votable = find_votable("DRP_MODE");
				if (!dwc3_vusb->drp_mode_votable) {
					pr_err("%s: %d: drp_mode_votable is null\n", __func__, __LINE__);
					mutex_unlock(&dwc3_vusb->switch_mutex);
					return -ENODEV;
				};
			}

			if (is_enable) {
				wake_lock(&dwc3_vusb->host_wake_lock);
				dwc3_vusb->cc_state = true;

				queue_delayed_work(dwc3_vusb->workq, &dwc3_vusb->typec_detect_work, 0);
				queue_delayed_work(dwc3_vusb->workq, &dwc3_vusb->host_disabled_work,
						   msecs_to_jiffies(DWC3_VUSB_HOST_DISABLED_TIME));
			} else {
				if (!dwc3_vusb->host_enable) {
					pr_info("%s: %d: otg already disabled\n", __func__, __LINE__);
					mutex_unlock(&dwc3_vusb->switch_mutex);
					return -EINVAL;
				}

				cancel_delayed_work(&dwc3_vusb->typec_detect_work);
				cancel_delayed_work(&dwc3_vusb->host_disabled_work);

				if (wake_lock_active(&dwc3_vusb->host_wake_lock))
					wake_unlock(&dwc3_vusb->host_wake_lock);
			}

			/* check again */
			vote_val = get_client_vote_locked(dwc3_vusb->drp_mode_votable, "OTG_DRP_VOTER");
			if (vote_val != is_enable)
				vote(dwc3_vusb->drp_mode_votable, "OTG_DRP_VOTER", is_enable, 1);
			else
				pr_info("%s: %d: already set drp mode votable to %d, ignore\n",
					__func__, __LINE__, vote_val);

			dwc3_vusb->host_enable = is_enable;
		}
	} else {
		/* Typec switch */
		union power_supply_propval pval = {0,};

		if (!dwc3_vusb->usb_psy) {
			dwc3_vusb->usb_psy = power_supply_get_by_name("usb");
			if (!dwc3_vusb->usb_psy) {
				pr_err("%s: %d: usb_psy is null\n", __func__, __LINE__);
				mutex_unlock(&dwc3_vusb->switch_mutex);
				return -ENODEV;
			}
		}
		pval.intval = is_enable;
		power_supply_set_property(dwc3_vusb->usb_psy,
					  POWER_SUPPLY_PROP_TYPEC_SWITCH, &pval);
	}
	mutex_unlock(&dwc3_vusb->switch_mutex);

	return size;
}
static DEVICE_ATTR(host_mode, S_IRUGO | S_IWUSR, dwc3_vusb_host_mode_show, dwc3_vusb_host_mode_store);

static ssize_t dwc3_vusb_usbid_value_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	ssize_t size;
	struct dwc3_vusb *dwc3_vusb = dev_get_drvdata(dev);
	int voltage, ret;

	mutex_lock(&dwc3_vusb->switch_mutex);
	if (!dwc3_vusb->mode) {
		pr_err("%s: %d: default mv = %d\n", __func__, __LINE__, DWC3_VUSB_ID_DEFAULT_MV);
		size = snprintf(buf, PAGE_SIZE, "%d\n", DWC3_VUSB_ID_DEFAULT_MV);
		mutex_unlock(&dwc3_vusb->switch_mutex);
		return size;
	}

	if (!dwc3_vusb->host_enable) {
		gpio_direction_output(dwc3_vusb->id_pull, 1);
		msleep(dwc3_vusb->id_debounce);
	}
	ret = dwc3_vusb_read_id_voltage(dwc3_vusb, &voltage);
	if (ret < 0) {
		mutex_unlock(&dwc3_vusb->switch_mutex);
		return ret;
	}
	pr_info("%s: %d: id voltage = %d\n", __func__, __LINE__, voltage);
	size = snprintf(buf, PAGE_SIZE, "%d\n", voltage);

	if (!dwc3_vusb->host_enable) {
		gpio_direction_output(dwc3_vusb->id_pull, 0);
		msleep(dwc3_vusb->id_debounce);
	}
	mutex_unlock(&dwc3_vusb->switch_mutex);

	return size;
}

static ssize_t dwc3_vusb_usbid_value_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	pr_err("%s: %d: not support\n", __func__, __LINE__);
	return -EPERM;
}
static DEVICE_ATTR(usbid_value, S_IRUGO | S_IWUSR, dwc3_vusb_usbid_value_show, dwc3_vusb_usbid_value_store);

static struct device_attribute *dwc3_vusb_attributes[] = {
	&dev_attr_usb_type,
	&dev_attr_otg_mode,
	&dev_attr_host_mode,
	&dev_attr_usbid_value,
};

static void dwc3_vusb_id_detect_work(struct work_struct *work)
{
	struct dwc3_vusb *dwc3_vusb = container_of(work,
						   struct dwc3_vusb,
						   id_detect_work.work);
	int i, ret, voltage = 0;
	union power_supply_propval pval = {0,};
	bool id;

	mutex_lock(&dwc3_vusb->switch_mutex);
	if (!dwc3_vusb->host_enable) {
		pr_info("%s: %d: otg is disabled\n", __func__, __LINE__);
		mutex_unlock(&dwc3_vusb->switch_mutex);
		return;
	}

	ret = dwc3_vusb_read_id_voltage(dwc3_vusb, &voltage);
	if (ret < 0)
		goto end;

	id = (voltage > dwc3_vusb->id_trigger_mv);
	pr_info("%s: %d: voltage = %d, id_trigger_mv = %d, id = %d, id_state = %d\n",
		__func__, __LINE__, voltage, dwc3_vusb->id_trigger_mv, id, dwc3_vusb->id_state);

	if (id == dwc3_vusb->id_state)
		goto end;

	for (i = 0; i < dwc3_vusb->id_times; i++) {
		ret = dwc3_vusb_read_id_voltage(dwc3_vusb, &voltage);
		if (ret < 0)
			goto end;
		id = (voltage > dwc3_vusb->id_trigger_mv);
		if (id == dwc3_vusb->id_state) {
			pr_info("%s: %d: id debounce invalid: %d\n", voltage);
			goto end;
		}
		msleep(dwc3_vusb->id_debounce);
	}

	if (!dwc3_vusb->usb_psy) {
		dwc3_vusb->usb_psy = power_supply_get_by_name("usb");
		if (!dwc3_vusb->usb_psy) {
			pr_err("%s: %d: usb_psy is null\n", __func__, __LINE__);
			goto end;
		}
	}

	if (id) {
		pr_info("%s: %d: OTG Plugout\n", __func__, __LINE__);
		extcon_set_state_sync(dwc3_vusb->edev, EXTCON_USB_HOST, false);
		pval.intval = 0;
//		power_supply_set_property(dwc3_vusb->usb_psy,
//					  POWER_SUPPLY_PROP_OTG_VBUS_ENABLE, &pval);
	} else {
		if (dwc3_vusb->usbc_pdev) {
			if (dwc3_msm_otg_vbus(dev_get_drvdata(&dwc3_vusb->usbc_pdev->dev))) {
				pr_info("%s: %d: vbus present, ignore\n", __func__, __LINE__);
				goto end;
			}
		}
		pr_info("%s: %d: OTG Plugin\n", __func__, __LINE__);
		pval.intval = 1;
//		power_supply_set_property(dwc3_vusb->usb_psy,
//					  POWER_SUPPLY_PROP_OTG_VBUS_ENABLE, &pval);
		extcon_set_state_sync(dwc3_vusb->edev, EXTCON_USB_HOST, true);
	}

	dwc3_vusb->id_trigger_mv = id ? DWC3_VUSB_ID_TRIGGER_MV : DWC3_VUSB_ID_TRIGGERED_MV;
	dwc3_vusb->id_state = id;
	if (id)
		queue_delayed_work(dwc3_vusb->workq, &dwc3_vusb->host_disabled_work,
				   msecs_to_jiffies(DWC3_VUSB_HOST_DISABLED_TIME));
	else
		cancel_delayed_work(&dwc3_vusb->host_disabled_work);
end:
	queue_delayed_work(dwc3_vusb->workq, &dwc3_vusb->id_detect_work,
			   msecs_to_jiffies(DWC3_VUSB_DETECT_CYCLE_TIME));
	mutex_unlock(&dwc3_vusb->switch_mutex);
}

static void dwc3_vusb_typec_detect_work(struct work_struct *work)
{
	struct dwc3_vusb *dwc3_vusb = container_of(work,
						   struct dwc3_vusb,
						   typec_detect_work.work);
	union power_supply_propval pval = {0,};
	bool cc;

	mutex_lock(&dwc3_vusb->switch_mutex);
	if (!dwc3_vusb->host_enable) {
		pr_info("%s: %d: otg is disabled\n", __func__, __LINE__);
		mutex_unlock(&dwc3_vusb->switch_mutex);
		return;
	}

	power_supply_get_property(dwc3_vusb->usb_psy,
				  POWER_SUPPLY_PROP_TYPEC_MODE, &pval);
	if (pval.intval == POWER_SUPPLY_TYPEC_SINK ||
	    pval.intval == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER ||
	    pval.intval == POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE ||
	    pval.intval == POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY ||
	    pval.intval == POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY)
		cc = false;
	else
		cc = true;

	pr_info("%s: %d: cc = %d(%d), cc_state = %d\n", __func__, __LINE__, cc, pval.intval, dwc3_vusb->cc_state);

	if (cc == dwc3_vusb->cc_state)
		goto end;

	dwc3_vusb->cc_state = cc;
	if (cc)
		queue_delayed_work(dwc3_vusb->workq, &dwc3_vusb->host_disabled_work,
				   msecs_to_jiffies(DWC3_VUSB_HOST_DISABLED_TIME));
	else
		cancel_delayed_work(&dwc3_vusb->host_disabled_work);
end:
	queue_delayed_work(dwc3_vusb->workq, &dwc3_vusb->typec_detect_work,
			   msecs_to_jiffies(DWC3_VUSB_DETECT_CYCLE_TIME));
	mutex_unlock(&dwc3_vusb->switch_mutex);
}

static void dwc3_vusb_host_disabled_work(struct work_struct *work)
{
	struct dwc3_vusb *dwc3_vusb = container_of(work,
						   struct dwc3_vusb,
						   host_disabled_work.work);
	mutex_lock(&dwc3_vusb->switch_mutex);
	if (!dwc3_vusb->host_enable) {
		pr_info("%s: %d: otg already disabled\n", __func__, __LINE__);
		mutex_unlock(&dwc3_vusb->switch_mutex);
		return;
	}

	pr_info("%s: %d: otg disabled\n", __func__, __LINE__);

	if (dwc3_vusb->mode) {
		cancel_delayed_work(&dwc3_vusb->id_detect_work);
		if (!dwc3_vusb->id_state) {
			union power_supply_propval pval = {0,};

			extcon_set_state_sync(dwc3_vusb->edev, EXTCON_USB_HOST, false);
			if (!dwc3_vusb->usb_psy) {
				dwc3_vusb->usb_psy = power_supply_get_by_name("usb");
				if (!dwc3_vusb->usb_psy) {
					pr_err("%s: %d: usb_psy is null\n", __func__, __LINE__);
					mutex_unlock(&dwc3_vusb->switch_mutex);
					return;
				}
			}
			pval.intval = 0;
//			power_supply_set_property(dwc3_vusb->usb_psy,
//						  POWER_SUPPLY_PROP_OTG_VBUS_ENABLE, &pval);
		}
		gpio_direction_output(dwc3_vusb->id_pull, 0);
		msleep(dwc3_vusb->id_debounce);
	}

	if (dwc3_vusb->typec_hw_det) {
		int vote_val = 0;

		cancel_delayed_work(&dwc3_vusb->typec_detect_work);
		vote_val = get_client_vote_locked(dwc3_vusb->drp_mode_votable, "OTG_DRP_VOTER");
		if (vote_val)
			vote(dwc3_vusb->drp_mode_votable, "OTG_DRP_VOTER", false, 1);
		else
			pr_info("%s: %d: already set drp mode votable to %d, ignore\n",
				__func__, __LINE__, vote_val);
	}

	dwc3_vusb->host_enable = false;
	if (wake_lock_active(&dwc3_vusb->host_wake_lock))
		wake_unlock(&dwc3_vusb->host_wake_lock);
	mutex_unlock(&dwc3_vusb->switch_mutex);
}

static int dwc3_vusb_file_interface_create(struct dwc3_vusb *dwc3_vusb)
{
	int ret = 0, i;

	dwc3_vusb->dwc3_vusb_class = class_create(THIS_MODULE, "dwc3_vusb");
	if (IS_ERR_OR_NULL(dwc3_vusb->dwc3_vusb_class)) {
		ret = PTR_ERR(dwc3_vusb->dwc3_vusb_class);
		pr_err("%s: %d: fail to create dwc3_vusb class: %d\n",
		       __func__, __LINE__, ret);
		return ret;
	}

	dwc3_vusb->dwc3_vusb_device = device_create(dwc3_vusb->dwc3_vusb_class, NULL,
						    MKDEV(0, 0), NULL, "usb0");
	if (IS_ERR_OR_NULL(dwc3_vusb->dwc3_vusb_device)) {
		ret = PTR_ERR(dwc3_vusb->dwc3_vusb_device);
		pr_err("%s: %d: fail to create usb0 device: %d\n",
		       __func__, __LINE__, ret);
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(dwc3_vusb_attributes); i++) {
		ret = device_create_file(dwc3_vusb->dwc3_vusb_device,
					 dwc3_vusb_attributes[i]);
		if (ret) {
			pr_err("%s: %d: fail to create attr file: %d\n",
			       __func__, __LINE__, ret);
			goto err1;
		}
	}

	dev_set_drvdata(dwc3_vusb->dwc3_vusb_device, dwc3_vusb);

	return 0;
err1:
	while (--i >= 0)
		device_remove_file(dwc3_vusb->dwc3_vusb_device, dwc3_vusb_attributes[i]);
	device_destroy(dwc3_vusb->dwc3_vusb_class, dwc3_vusb->dwc3_vusb_device->devt);
	dwc3_vusb->dwc3_vusb_device = NULL;
err:
	class_destroy(dwc3_vusb->dwc3_vusb_class);
	dwc3_vusb->dwc3_vusb_class = NULL;
	return ret;
}

static void dwc3_vusb_file_interface_destroy(struct dwc3_vusb *dwc3_vusb)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dwc3_vusb_attributes); i++)
		device_remove_file(dwc3_vusb->dwc3_vusb_device, dwc3_vusb_attributes[i]);
	device_destroy(dwc3_vusb->dwc3_vusb_class, dwc3_vusb->dwc3_vusb_device->devt);
	dwc3_vusb->dwc3_vusb_device = NULL;
	class_destroy(dwc3_vusb->dwc3_vusb_class);
	dwc3_vusb->dwc3_vusb_class = NULL;
}

static int dwc3_vusb_parse_dts(struct platform_device *pdev,
			       struct dwc3_vusb *dwc3_vusb)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *usbc_np;
	int ret = 0;

	if (of_property_read_u32(np, "mode", &dwc3_vusb->mode))
		dwc3_vusb->mode = 0;
	pr_info("%s: %d: switch mode %d\n", __func__, __LINE__, dwc3_vusb->mode);

	usbc_np = of_parse_phandle(np, "usbc", 0);
	if (usbc_np) {
		dwc3_vusb->usbc_pdev = of_find_device_by_node(usbc_np);
		if (!dwc3_vusb->usbc_pdev)
			pr_err("%s: %d: fail to get usbc pdev\n", __func__, __LINE__);
		of_node_put(usbc_np);
	} else
		pr_err("%s: %d: fail to get usbc np\n", __func__, __LINE__);

	/* Typec USB */
	if (!dwc3_vusb->mode) {
		/* hw auto detect */
		dwc3_vusb->typec_hw_det = of_property_read_bool(np, "vivo,typec-hw-det");
		pr_info("%s: %d: typec-hw-det = %d\n", __func__, __LINE__, dwc3_vusb->typec_hw_det);
	}
	/* Micro USB */
	if (dwc3_vusb->mode) {
		if (of_property_read_u32(np, "id-debounce", &dwc3_vusb->id_debounce))
			dwc3_vusb->id_debounce = DWC3_VUSB_ID_DEBOUNCE_MS;
		if (of_property_read_u32(np, "id-times", &dwc3_vusb->id_times))
			dwc3_vusb->id_times = DWC3_VUSB_ID_DEBOUNCE_TIMES;
		if (of_property_read_u32(np, "id-adc-times", &dwc3_vusb->id_adc_times))
			dwc3_vusb->id_times = DWC3_VUSB_ID_ADC_TIMES;
		pr_info("%s: %d: id-debounce = %d, id-times = %d, id-adc-times = %d\n",
			__func__, __LINE__, dwc3_vusb->id_debounce, dwc3_vusb->id_times, dwc3_vusb->id_adc_times);
	}

	return ret;
}

static int dwc3_vusb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dwc3_vusb *dwc3_vusb;
	int ret = 0;

	if (!np)
		return -EINVAL;

	dwc3_vusb = devm_kzalloc(dev, sizeof(*dwc3_vusb), GFP_KERNEL);
	if (!dwc3_vusb)
		return -ENOMEM;
	dwc3_vusb->dev = dev;

	ret = dwc3_vusb_parse_dts(pdev, dwc3_vusb);
	if (ret) {
		pr_err("%s: %d: fail to parse dwc3_vusb dts: %d\n",
		       __func__, __LINE__, ret);
		goto err;
	}

	dwc3_vusb->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR_OR_NULL(dwc3_vusb->edev)) {
		ret = PTR_ERR(dwc3_vusb->edev);
		pr_err("%s: %d: allocate usb extcon device fail: %d\n", __func__, __LINE__, ret);
		goto err;
	}
	ret = devm_extcon_dev_register(dev, dwc3_vusb->edev);
	if (ret < 0) {
		pr_err("%s: %d: register usb extcon device fail: %d\n", __func__, __LINE__, ret);
		goto err;
	}

	mutex_init(&dwc3_vusb->switch_mutex);
	if (dwc3_vusb->mode || dwc3_vusb->typec_hw_det) {
		wake_lock_init(&dwc3_vusb->host_wake_lock, WAKE_LOCK_SUSPEND, "host_wake_lock");
		INIT_DELAYED_WORK(&dwc3_vusb->host_disabled_work, dwc3_vusb_host_disabled_work);

		if (dwc3_vusb->mode)
			INIT_DELAYED_WORK(&dwc3_vusb->id_detect_work, dwc3_vusb_id_detect_work);
		if (dwc3_vusb->typec_hw_det)
			INIT_DELAYED_WORK(&dwc3_vusb->typec_detect_work, dwc3_vusb_typec_detect_work);

		dwc3_vusb->workq = create_singlethread_workqueue("dwc3_vusb_workq");
		if (!dwc3_vusb->workq) {
			ret = -ENOMEM;
			pr_err("%s: %d: create workq fail\n", __func__, __LINE__);
			goto err;
		}

		if (dwc3_vusb->mode) {
			dwc3_vusb->id_pull = of_get_named_gpio(np, "id-pull-gpio", 0);
			if (gpio_is_valid(dwc3_vusb->id_pull)) {
				ret = devm_gpio_request_one(dev, dwc3_vusb->id_pull,
							    GPIOF_OUT_INIT_LOW, "id_pull");
				if (ret) {
					pr_err("%s: %d: request id-volt fail: %d\n", __func__, __LINE__, ret);
					ret = 0;
					goto err1;
				}
			} else {
				pr_err("%s: %d: get id-volt fail: %d\n", __func__, __LINE__, dwc3_vusb->id_pull);
				goto err1;
			}
		}
	}

	ret = dwc3_vusb_file_interface_create(dwc3_vusb);
	if (ret)
		goto err1;

	platform_set_drvdata(pdev, dwc3_vusb);
	pr_info("%s: %d: finish\n", __func__, __LINE__);

	return 0;
err1:
	destroy_workqueue(dwc3_vusb->workq);
	dwc3_vusb->workq = NULL;
err:
	return ret;
}

static int dwc3_vusb_remove(struct platform_device *pdev)
{
	struct dwc3_vusb *dwc3_vusb = platform_get_drvdata(pdev);

	if (dwc3_vusb->dwc3_vusb_class)
		dwc3_vusb_file_interface_destroy(dwc3_vusb);
	if (dwc3_vusb->mode || dwc3_vusb->typec_hw_det) {
		if (dwc3_vusb->mode)
			cancel_delayed_work_sync(&dwc3_vusb->id_detect_work);
		if (dwc3_vusb->typec_hw_det)
			cancel_delayed_work_sync(&dwc3_vusb->typec_detect_work);
		cancel_delayed_work_sync(&dwc3_vusb->host_disabled_work);
		if (dwc3_vusb->workq) {
			destroy_workqueue(dwc3_vusb->workq);
			dwc3_vusb->workq = NULL;
		}
		if (dwc3_vusb->id_adc_chan) {
			iio_channel_release(dwc3_vusb->id_adc_chan);
			dwc3_vusb->id_adc_chan = NULL;
		}
	}
	if (dwc3_vusb->usb_psy) {
		power_supply_put(dwc3_vusb->usb_psy);
		dwc3_vusb->usb_psy = NULL;
	}

	return 0;
}

static const struct of_device_id dwc3_vusb_dt_ids[] = {
	{ .compatible = "v,dwc3-vusb", },
	{ },
};
MODULE_DEVICE_TABLE(of, dwc3_vusb_dt_ids);

static struct platform_driver dwc3_vusb_driver = {
	.probe		= dwc3_vusb_probe,
	.remove		= dwc3_vusb_remove,
	.driver		= {
		.name	= "dwc3_vusb",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(dwc3_vusb_dt_ids),
	},
};

static int __init dwc3_vusb_init(void)
{
	return platform_driver_register(&dwc3_vusb_driver);
}

static void __exit dwc3_vusb_exit(void)
{
	platform_driver_unregister(&dwc3_vusb_driver);
}

module_init(dwc3_vusb_init);
module_exit(dwc3_vusb_exit);

MODULE_DESCRIPTION("DWC3 VUSB");
MODULE_LICENSE("GPL");
