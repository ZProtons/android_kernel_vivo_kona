#define pr_fmt(fmt)     "[FP_KERN] " KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include "fp_id.h"

#define MAX_TIMES		7

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

/*
 *static const struct vreg_config const vreg_conf[] = {
 *	{ "vcc_fpc", 1800000UL, 1800000UL, 10, },
 *	{ "vcc_goodix", 2800000UL, 2800000UL, 10, },
 *};
 */

static const char * const pctl_names[] = {
	"fp_id_gpio_up",
	"fp_id_gpio_down",
};

static int fp_gpio = -1;
static int fp_id = -1;
static int fp_up = -1;
static int fp_down = -1;
static int count_reset_high_ground = -1;
static int count_reset_high_suspend = -1;
static int count_reset_low_ground = -1;
static int count_reset_low_suspend = -1;
const char *fp_project_name;
//static int count_ground = 0;//fp_id pin connect to ground
//static int count_suspend = 0;//fp_id pin suspend
//struct regulator *fp_vreg[ARRAY_SIZE(vreg_conf)];
struct pinctrl *fp_pinctrl;
struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];

#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
struct attribute fp_id_attr = {
	.name = "fp_id",
	.mode = DEVFS_MODE_RO,
};
static struct attribute *our_own_sys_attrs[] = {
	&fp_id_attr,
	NULL,
};

/*
 *static int vreg_setup(struct device *dev, const char *name,
 *	bool enable)
 *{
 *	size_t i;
 *	int rc;
 *	struct regulator *vreg;
 *
 *	printk("vreg_setup start\n");
 *	for (i = 0; i < ARRAY_SIZE(fp_vreg); i++) {
 *		const char *n = vreg_conf[i].name;
 *		if (!strncmp(n, name, strlen(n)))
 *			goto found;
 *	}
 *	dev_err(dev, "Regulator %s not found\n", name);
 *	return -EINVAL;
 *found:
 *	vreg = fp_vreg[i];
 *	if (enable) {
 *		if (!vreg) {
 *			vreg = regulator_get(dev, name);
 *			if (IS_ERR(vreg)) {
 *				dev_err(dev, "Unable to get  %s\n", name);
 *				return -ENODEV;
 *			}
 *		}
 *		printk("vreg_setup 111111\n");
 *		if (regulator_count_voltages(vreg) > 0) {
 *			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
 *					vreg_conf[i].vmax);
 *			if (rc)
 *				dev_err(dev,
 *					"Unable to set voltage on %s, %d\n",
 *					name, rc);
 *		}
 *		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
 *		if (rc < 0)
 *			dev_err(dev, "Unable to set current on %s, %d\n",
 *					name, rc);
 *		rc = regulator_enable(vreg);
 *		if (rc) {
 *			dev_err(dev, "error enabling %s: %d\n", name, rc);
 *			regulator_put(vreg);
 *			vreg = NULL;
 *		}
 *		fp_vreg[i] = vreg;
 *	} else {
 *		if (vreg) {
 *			if (regulator_is_enabled(vreg)) {
 *				regulator_disable(vreg);
 *				dev_dbg(dev, "disabled %s\n", name);
 *			}
 *			regulator_put(vreg);
 *			fp_vreg[i] = NULL;
 *		}
 *		rc = 0;
 *	}
 *	return rc;
 *}
 */
static int select_pin_ctl(struct device *dev, const char *name)
{
	size_t i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];

		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fp_pinctrl, pinctrl_state[i]);
			if (rc)
				dev_err(dev, "bio_fp_error cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "bio_fp_error %s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

static ssize_t fp_id_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	char *fp_frame_id;

	if (fp_id == FPC_FPC1022)
		fp_frame_id = "fpc_1022";
	else if (fp_id == FPC_FPC1245)
		fp_frame_id = "fpc_1245";
	else if (fp_id == GOODIX_GF5116M)
		fp_frame_id = "goodix_5116";
	else if (fp_id == GOODIX_GF52X6)
		fp_frame_id = "goodix_5216";
	else if (fp_id == GOODIX_GF318M)
		fp_frame_id = "goodix_318m";
	else if (fp_id == GOODIX_GF3208)
		fp_frame_id = "goodix_3208b";
	else if (fp_id == GOODIX_GF5269)
		fp_frame_id = "goodix_5269";
	else if (fp_id == GOODIX_GF5288)
		fp_frame_id = "goodix_5288";
	else if (fp_id == GOODIX_GF3658)
		fp_frame_id = "goodix_3658";
	else if (fp_id == GOODIX_GF3626)
		fp_frame_id = "sidefp_goodix_3626";
	else if (fp_id == GOODIX_GF3636)
		fp_frame_id = "sidefp_goodix_3636";
	else if (fp_id == FPC_FPC1540)
		fp_frame_id = "sidefp_fpc_1540";
	else if (fp_id == GOODIX_GF9518)
		fp_frame_id = "udfp_goodix_gf9518";
	else if (fp_id == GOODIX_GF9518N)
		fp_frame_id = "udfp_goodix2_gf9518";
	else if (fp_id == SYNAPTICS_FS9501)
		fp_frame_id = "udfp_syna_fs9501";
	else if (fp_id == GOODIX_GF9608)
		fp_frame_id = "udfp_goodix_gf9608";
	else if (fp_id == GOODIX_GF9578)
		fp_frame_id = "udfp_goodix_gf9578";
	else
		fp_frame_id = "default";

	printk("fp_project_name:%s, get_fp_id=%d, fp_frame_id=%s, fp_up=%d, fp_down=%d, fp_gpio=%d.\n", fp_project_name, get_fp_id(), fp_frame_id, fp_up, fp_down, fp_gpio);
	printk("%s: count_reset_high_ground=%d, count_reset_high_suspend=%d, count_reset_low_ground=%d, count_reset_low_suspend=%d.\n", __func__, count_reset_high_ground, count_reset_high_suspend, count_reset_low_ground, count_reset_low_suspend);
	return snprintf(buf, strlen(fp_frame_id)+2, "%s\n", fp_frame_id);
}

static ssize_t fp_id_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	/* nothing to do temply */
	printk("fp_id cannot be writed.\n");
	return 0;
}

int get_fp_id(void)
{
	printk("get_fp_id , fp_id=%d, fp_up=%d, fp_down=%d\n", fp_id, fp_up, fp_down);
	return fp_id;
}
EXPORT_SYMBOL(get_fp_id);

static void fp_id_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops fp_id_object_sysfs_ops = {
	.show = fp_id_object_show,
	.store = fp_id_object_store,
};
static struct kobj_type fp_id_object_type = {
	.sysfs_ops	= &fp_id_object_sysfs_ops,
	.release	= fp_id_object_release,
	.default_attrs = our_own_sys_attrs,
};

struct kobject kobj;

static int
fp_id_probe(struct platform_device *pdev)
{

	int ret;
	int i;
	ret = kobject_init_and_add(&kobj, &fp_id_object_type, NULL, "fp_id");
	if (ret) {
		printk("%s: Create fp_id error!\n", __func__);
		return -ERRORFP;
	}
	ret = of_property_read_string(pdev->dev.of_node, "vivo,project-name", &fp_project_name);
	if (ret) {
		printk("%s:vivo,project-name property do not find\n", __func__);
		fp_project_name = "default";
		return -ERRORFP;
	}
	printk("%s:vivo,project-name is %s\n", __func__, fp_project_name);

	if ((!strncmp(fp_project_name, "PD2059", 6)) || (!strncmp(fp_project_name, "PD2055", 6)) || (!strncmp(fp_project_name, "PD2046", 6))) {
		fp_id = GOODIX_GF9578;
		printk("PD2067F_EX direct return GOODIX_GF9578 \n");
		return 0;
	}

	if ((!strncmp(fp_project_name, "PD2024", 6)) || (!strncmp(fp_project_name, "PD2025", 6))) {
		fp_id = GOODIX_GF9518;
		printk("%s: return gf9518  directly\n", __func__);
		return 0;
	}

	if ((!strncmp(fp_project_name, "TD1906", 6)) || (!strncmp(fp_project_name, "PD1950", 6)) || (!strncmp(fp_project_name, "PD1955", 6)) ||  (!strncmp(fp_project_name, "PD2011", 6)) || (!strncmp(fp_project_name, "PD2020", 6)) \
		|| (!strncmp(fp_project_name, "PD2001", 6)) ||  (!strncmp(fp_project_name, "PD2005", 6))) {
		fp_id = GOODIX_GF9518;
		printk("%s: return gf9518  directly\n", __func__);
		return 0;
	}

	if ((!strncmp(fp_project_name, "EXP1933", 7))) {
		fp_id = GOODIX_GF9608;
		printk("%s: return gf9608  directly\n", __func__);
		return 0;
	}

	if ((!strncmp(fp_project_name, "PD1963", 6))) {
		fp_id = GOODIX_GF3658;
		printk("%s: return gf3658  directly\n", __func__);
		return 0;
	}

	if ((!strncmp(fp_project_name, "PD2073", 6))) {
		fp_id = GOODIX_GF3636;
		printk("%s: return gf3636  directly\n", __func__);
		return 0;
	}

	fp_gpio = of_get_named_gpio(pdev->dev.of_node, "fp_id,gpios", 0);
	if (fp_gpio < 0) {
		printk("%s: get fp_id gpio failed!\n", __func__);
		return -ERRORFP;
	}
	printk("%s:fp gpio: %d \n", __func__, fp_gpio);

	ret = devm_gpio_request(&pdev->dev, fp_gpio, "fp_id, gpios");
	if (ret) {
		printk("%s: request fp_id gpio failed!\n", __func__);
		return -ERRORFP;
	}

	fp_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fp_pinctrl)) {
		if (PTR_ERR(fp_pinctrl) == -EPROBE_DEFER) {
			printk("%s: pinctrl not ready!\n", __func__);
			return -ERRORFP;
		}
		printk("%s: Target does not use pinctrl\n", __func__);
		fp_pinctrl = NULL;
		return -ERRORFP;
	}

	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fp_pinctrl, n);
		if (IS_ERR(state)) {
			printk("%s: cannot find '%s'\n", __func__, n);
			return -ERRORFP;
		}
		printk("%s: found pin control %s\n", __func__, n);
		pinctrl_state[i] = state;
	}

	ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_up");
	mdelay(5);
	if (ret)
		return -ERRORFP;
	fp_up = gpio_get_value(fp_gpio);
	printk("%s: set fp-id pull up,get gpio value = %d\n", __func__, fp_up);
	ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
	mdelay(5);
	if (ret)
		return -ERRORFP;
	fp_down = gpio_get_value(fp_gpio);
	printk("%s: set fp-id pull down,get gpio value = %d\n", __func__, fp_down);
	if (!strncmp(fp_project_name, "PD1806", 6)) {
		if (fp_up == 0) {
			fp_id = GOODIX_GF5288;
			printk("%s: gpio value: %d, return gf5288 ", __func__, fp_up);
		} else {
			fp_id = GOODIX_GF9518;
			printk("%s: gpio value: %d, return gf9518 ", __func__, fp_up);
		}
		return 0;
	} else if  (!strncmp(fp_project_name, "PD1981", 6)) {
		if ((fp_up == 1) && (fp_down == 0)) {
			fp_id = FPC_FPC1540;
		} else {
			fp_id = GOODIX_GF3626;
		}
	}

	return 0;
}

static int fp_id_remove(struct platform_device *pdev)
{
	printk("fp_id  remove.\n");
	kobject_del(&kobj);
    return 0;
}

static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp-id",},
	{},
};

static struct platform_driver fp_id_driver = {
    .probe      = fp_id_probe,
    .remove     = fp_id_remove,
    .driver = {
		.name   = "fp_id",
		.owner  = THIS_MODULE,
		.of_match_table = fp_id_match_table,
	},
};

static int __init fp_id_init(void)
{
    return platform_driver_register(&fp_id_driver);
}
module_init(fp_id_init);

static void __exit fp_id_exit(void)
{
    platform_driver_unregister(&fp_id_driver);

}
module_exit(fp_id_exit);

MODULE_AUTHOR("Xiaot BBK Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
