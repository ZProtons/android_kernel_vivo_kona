#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/log2.h>
//#include <linux/qpnp/qpnp-misc.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/qpnp/pwm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <uapi/asm-generic/unistd.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include "../inc/timed_output.h"

//#define VIB_PINCTRL_STATE    "default"
//static struct pinctrl *vib_pinctrl;

#define PWM_NEW_FRAMEWORK_API
#define VIB8846_I0_CONTROL
//#define GPIO_VALUE_DEBUG

#if defined(PWM_NEW_FRAMEWORK_API)
struct vib_pwm_setting {
	u64	period_ns;
	u64	duty_ns;
};
#endif

#define VIVO_VIB_FIRST_STAGE_TIME		70
#define VIVO_VIB_SECOND_STAGE_TIME		391

#define VIVO_VIB_FIRST_STAGE_PERIOD     40000   //1562pps-25khz
#define VIVO_VIB_SECOND_STAGE_PERIOD    26000   //2343pps-37.5khz
#define VIVO_VIB_THIRD_STAGE_PERIOD     80000   //781pps-12.5khz
#define VIVO_VIB_DEFAULT_STAGE_INDEX    0

#define VIVO_VIB_LOW_TEMPERATURE_PERIOD 80000   //781pps-12.5khz
#define VIVO_VIB_LOW_TEMPERATURE_TIME   1500   //1.5s

struct qpnp_hap {
	struct pwm_device    *pwm_dev;
	struct platform_device      *pdev;
	struct timed_output_dev     timed_dev;
	struct mutex            lock;
	char *cali_data;
	int adec;
	int dir;
	int nenable;
	int nsleep;
	int dcen;
	int timeout;
#ifdef VIB8846_I0_CONTROL
	int i0;
#endif
	bool   pwm_enabled;
	int stage;
	unsigned int push_first_stage_time;
	unsigned int push_second_stage_time;
	unsigned int push_first_stage_period;
	unsigned int push_second_stage_period;
	unsigned int push_third_stage_period;
	unsigned int push_start_stage_index;
	unsigned int pull_first_stage_time;
	unsigned int pull_second_stage_time;
	unsigned int pull_first_stage_period;
	unsigned int pull_second_stage_period;
	unsigned int pull_third_stage_period;
	unsigned int pull_start_stage_index;
	unsigned int push_pull_low_temperature_time;
	unsigned int push_pull_low_temperature_period;
	struct delayed_work delayed_work;
#if defined(PWM_NEW_FRAMEWORK_API)
	struct vib_pwm_setting pwm_setting;
#endif
};

static struct workqueue_struct *motor_work_queue;
static int vivo_step[4] = {0, 0, 0, 0};
static struct qpnp_hap *p_hap;
/*motor period */
/*static int motor_period = 54;*/
/*static int motor_period = 26;*/
/*static int motor_runtime = 1080;*/
static int current_dir;
static int dir = -1;
static int total_time;
static int true_runtime;
const char *camera_push_pull_type;

//extern unsigned int is_at_boot_dl;
//static unsigned int is_at_boot_dl = 0;


static int motor_gpio_direction_output(int gpio, int value)
{
	int rc = 0;
	if (gpio <= 0) {
		printk(KERN_ERR "ccm_vibrator: gpio(%d) set output invaild\n", gpio);
		rc = -1;
		return rc;
	}
	return gpio_direction_output(gpio, value);
}

static int motor_gpio_direction_input(int gpio)
{
	int rc = 0;
	if (gpio <= 0) {
		printk(KERN_ERR "ccm_vibrator: gpio(%d) set input invaild\n", gpio);
		rc = -1;
		return rc;
	}
	return gpio_direction_input(gpio);
}

static int qpnp_hap_parse_dt (struct qpnp_hap *hap)
{
	struct platform_device *pdev = hap->pdev;
	int rc = 0;
	int ret = 0;
	enum of_gpio_flags flags;

	/*get pwm device*/
	hap->pwm_dev = of_pwm_get(pdev->dev.of_node, NULL);
	printk(KERN_ERR "ccm_vibrator: PWM device pwm_dev:(0x%x)\n", hap->pwm_dev);
	if (IS_ERR(hap->pwm_dev)) {
		rc = PTR_ERR(hap->pwm_dev);
		printk(KERN_ERR "ccm_vibrator: Cannot get PWM device rc:(%d)\n", rc);
		hap->pwm_dev = NULL;
		return rc;
	}
	/* end */

	if (of_property_read_u32(pdev->dev.of_node, "vivo,push_first_stage_time", &hap->push_first_stage_time)) {
		hap->push_first_stage_time = VIVO_VIB_FIRST_STAGE_TIME;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,push_second_stage_time", &hap->push_second_stage_time)) {
		hap->push_second_stage_time = VIVO_VIB_SECOND_STAGE_TIME;
	}
	printk(KERN_INFO "ccm_vibrator: push_first_stage_time %d push_second_stage_time %d\n",
			hap->push_first_stage_time, hap->push_second_stage_time);
	if (of_property_read_u32(pdev->dev.of_node, "vivo,pull_first_stage_time", &hap->pull_first_stage_time)) {
		hap->pull_first_stage_time = VIVO_VIB_FIRST_STAGE_TIME;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,pull_second_stage_time", &hap->pull_second_stage_time)) {
		hap->pull_second_stage_time = VIVO_VIB_SECOND_STAGE_TIME;
	}
	printk(KERN_INFO "ccm_vibrator: push_first_stage_time %d push_second_stage_time %d\n",
			hap->pull_first_stage_time, hap->pull_second_stage_time);

	if (of_property_read_u32(pdev->dev.of_node, "vivo,push_first_stage_period", &hap->push_first_stage_period)) {
		hap->push_first_stage_period = VIVO_VIB_FIRST_STAGE_PERIOD;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,push_second_stage_period", &hap->push_second_stage_period)) {
		hap->push_second_stage_period = VIVO_VIB_SECOND_STAGE_PERIOD;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,push_third_stage_period", &hap->push_third_stage_period)) {
		hap->push_third_stage_period = VIVO_VIB_THIRD_STAGE_PERIOD;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,push_start_stage_index", &hap->push_start_stage_index)) {
		hap->push_start_stage_index = VIVO_VIB_DEFAULT_STAGE_INDEX;
	}
	printk(KERN_INFO "ccm_vibrator: push period 1st %d 2nd %d 3rd %d default index %d\n", hap->push_first_stage_period,
				hap->push_second_stage_period, hap->push_third_stage_period, hap->push_start_stage_index);

	if (of_property_read_u32(pdev->dev.of_node, "vivo,pull_first_stage_period", &hap->pull_first_stage_period)) {
		hap->pull_first_stage_period = VIVO_VIB_FIRST_STAGE_PERIOD;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,pull_second_stage_period", &hap->pull_second_stage_period)) {
		hap->pull_second_stage_period = VIVO_VIB_SECOND_STAGE_PERIOD;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,pull_third_stage_period", &hap->pull_third_stage_period)) {
		hap->pull_third_stage_period = VIVO_VIB_THIRD_STAGE_PERIOD;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,pull_start_stage_index", &hap->pull_start_stage_index)) {
		hap->pull_start_stage_index = VIVO_VIB_DEFAULT_STAGE_INDEX;
	}
	printk(KERN_INFO "ccm_vibrator: pull period 1st %d 2nd %d 3rd %d default index %d\n", hap->pull_first_stage_period,
				hap->pull_second_stage_period, hap->pull_third_stage_period, hap->pull_start_stage_index);

	if (of_property_read_u32(pdev->dev.of_node, "vivo,push_pull_low_temperature_period", &hap->push_pull_low_temperature_period)) {
		hap->push_pull_low_temperature_period = VIVO_VIB_LOW_TEMPERATURE_PERIOD;
	}
	if (of_property_read_u32(pdev->dev.of_node, "vivo,push_pull_low_temperature_time", &hap->push_pull_low_temperature_time)) {
		hap->push_pull_low_temperature_time = VIVO_VIB_LOW_TEMPERATURE_TIME;
	}
	printk(KERN_INFO "ccm_vibrator: push_pull_low_temperature_time %d period %d\n", hap->push_pull_low_temperature_period,
				hap->push_pull_low_temperature_time);


	/*adec input-PD*/
	hap->adec = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-adec", 0, &flags);
	if (hap->adec < 0) {
		/* there's no adec pin */
		printk(KERN_ERR "ccm_vibrator: there's no adec pin, the ret is %d\n", hap->adec);
	} else {
		/* use adec pin */
		rc = gpio_request(hap->adec, "vivo,motor-adec");
		if (rc < 0)
			printk(KERN_ERR "ccm_vibrator: request vivo,motot-adec error: %d\n", rc);
		rc = motor_gpio_direction_input(hap->adec);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
		gpio_set_value(hap->adec, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
	}

#ifdef VIB8846_I0_CONTROL
	hap->i0 = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-i0", 0, &flags);
	if (hap->i0 < 0) {
		/* there's no i0 pin */
		printk(KERN_ERR "ccm_vibrator: there's no i0 pin, the ret is %d\n", hap->i0);
	} else {
		/* use adec pin */
		rc = gpio_request(hap->i0, "vivo,motor-i0");
		if (rc < 0)
			printk(KERN_ERR "ccm_vibrator: request vivo,motor-i0 error: %d\n", rc);
		rc = motor_gpio_direction_output(hap->i0, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->i0, rc);
			return rc;
		}
	}
#endif

	ret = of_property_read_string(pdev->dev.of_node, "vivo,camera-push-pull-type", &camera_push_pull_type);
	if (ret) {
		printk(KERN_ERR "%s:vivo,camera-push-pull-type property do not find, set default type positive\n", __func__);
		camera_push_pull_type = "positive";
	}
	printk(KERN_ERR "%s:vivo,camera-push-pull-type is %s\n", __func__, camera_push_pull_type);

	/*dir input-PD*/
	hap->dir = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-dir", 0, &flags);
	if (hap->dir < 0) {
		printk(KERN_ERR "ccm_vibrator: parse dir gpio error, the ret is %d\n", hap->dir);
	} else {
		rc = gpio_request(hap->dir, "vivo,motor-dir");
		if (rc < 0)
			printk(KERN_ERR "ccm_vibrator: request dir gpio error: %d\n", rc);
		rc = motor_gpio_direction_input(hap->dir);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
			return rc;
		}
		gpio_set_value(hap->dir, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
			return rc;
		}
	}


	/* nenable PD*/
	hap->nenable = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-nenable", 0, &flags);
	if (hap->nenable < 0) {
		printk(KERN_ERR "ccm_vibrator: parse motor-nenable error, the ret is %d\n", hap->nenable);
		//return hap->nenable;
	} else {
		rc = gpio_request(hap->nenable, "vivo,motor-nenable");
		if (rc < 0)
			printk(KERN_ERR "ccm_vibrator: request motor-nenable gpio error: %d\n", rc);
		rc = motor_gpio_direction_input(hap->nenable);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
			return rc;
		}
		gpio_set_value(hap->nenable, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
			return rc;
		}
	}


	/* nsleep PD*/
	hap->nsleep = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-nsleep", 0, &flags);
	if (hap->nsleep < 0) {
		printk(KERN_ERR "ccm_vibrator: parse nsleep error, the ret is %d\n", hap->nsleep);
		//return hap->nsleep;
	} else {
		rc = gpio_request(hap->nsleep, "vivo,motor-nsleep");
		if (rc < 0)
			printk(KERN_ERR "ccm_vibrator: request nsleep gpio error: %d\n", rc);
		rc = motor_gpio_direction_input(hap->nsleep);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
			return rc;
		}
		gpio_set_value(hap->nsleep, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
			return rc;
		}
	}


	/* dcen output-low*/
	hap->dcen = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-dcen", 0, &flags);
	if (hap->dcen < 0) {
		printk(KERN_ERR "ccm_vibrator: parse dcen error, the ret is %d\n", hap->dcen);
		//return hap->dcen;
	} else {
		rc = gpio_request(hap->dcen, "vivo,motor-dcen");
		if (rc < 0)
			printk(KERN_ERR "ccm_vibrator: request dcen gpio error: %d\n", rc);
		rc = motor_gpio_direction_output(hap->dcen, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
			return rc;
		}
	}


	/* timeout output-low*/
	hap->timeout = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-timeout", 0, &flags);
	if (hap->timeout < 0) {
		printk(KERN_ERR "ccm_vibrator: parse timeout error, the ret is %d\n", hap->timeout);
		return hap->timeout;
	}
	rc = gpio_request(hap->timeout, "vivo,motor-timeout");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request timeout gpio error: %d\n", rc);
	rc = gpio_direction_output(hap->timeout, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		return rc;
	}


	return rc;
}

static int vivo_enable_vib(struct qpnp_hap *hap)
{
	int rc = 0;

	printk(KERN_ERR"ccm_vibrator: Enter the %s\n", __func__);

	/* 1.pull dcen high */
	rc = motor_gpio_direction_output(hap->dcen, 1);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator: %s Unable to set dcen gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
		//return rc;
	}
	/* 2.pull nENBL high hold 1.5ms*/
	rc = motor_gpio_direction_output(hap->nenable, 1);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator: %s Unable to set nenable gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		//return rc;
	}
	usleep_range(1500, 1600);
	/* 2.pull ADEC high, hold 1us */
	if (hap->adec > 0) {
		rc = motor_gpio_direction_output(hap->adec, 1);
		if (rc < 0) {
			printk(KERN_ERR "ccm_vibrator: %s Unable to set adec gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			//return rc;
		}
	}

	usleep_range(1, 3);
	/* 3.pull nsleep high, hold 1ms */
	rc = motor_gpio_direction_output(hap->nsleep, 1);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator: %s Unable to set nsleep gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		//return rc;
	}
	usleep_range(1000, 1100);
	return rc;
}

static int vivo_disable_vib(struct qpnp_hap *hap)
{
	int rc = 0;

	/* pull nenable high */
	rc = motor_gpio_direction_output(hap->nenable, 1);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator %s Unable to set nenable gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		//return rc;
	}
	/* pull timeout high, delay 0.5ms */
	rc = gpio_direction_output(hap->timeout, 1);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator %s Unable to set timeout gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		//return rc;
	}
	usleep_range(1, 3);
	/* set nenable input-PD*/
	rc = motor_gpio_direction_input(hap->nenable);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator %s Unable to set nenable gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		//return rc;
	} else {
		gpio_set_value(hap->nenable, 0);
		if (rc < 0) {
			printk(KERN_ERR "ccm_vibrator %s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
			return rc;
		}
	}

	/* pull dcen low */
	rc = motor_gpio_direction_output(hap->dcen, 0);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator %s Unable to set dcen gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
		//return rc;
	}

	usleep_range(3000, 4000);
	/* set nsleep input-PD */
	rc = motor_gpio_direction_input(hap->nsleep);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator %s Unable to set nsleep gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		//return rc;
	} else {
		gpio_set_value(hap->nsleep, 0);
		if (rc < 0) {
			printk(KERN_ERR "ccm_vibrator %s Unable to set nsleep gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
			//return rc;
		}
	}

	/* set dir input-PD*/
	rc = motor_gpio_direction_input(hap->dir);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator %s Unable to set dir gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		//return rc;
	} else {
		gpio_set_value(hap->dir, 0);
		if (rc < 0) {
			printk(KERN_ERR "ccm_vibrator %s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
			//return rc;
		}
	}

	/* set adec input-PD*/
	if (hap->adec > 0) {
		rc = motor_gpio_direction_input(hap->adec);
		if (rc < 0) {
			printk(KERN_ERR "ccm_vibrator %s Unable to set adec gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			//return rc;
		} else {
			gpio_set_value(hap->adec, 0);
			if (rc < 0) {
				printk(KERN_ERR "ccm_vibrator %s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
				//return rc;
			}
		}
	}

	usleep_range(17000, 18000);

	/*after 20ms, pull timeout low */
	rc = gpio_direction_output(hap->timeout, 0);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator %s Unable to set gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		return rc;
	}

	return 0;
}

static void stop_motor_work(struct qpnp_hap *hap)
{
	int rc;
#if defined(PWM_NEW_FRAMEWORK_API)
	struct pwm_state pstate;
#endif
	/*disable the pwm*/
#if defined(PWM_NEW_FRAMEWORK_API)
	pwm_get_state(hap->pwm_dev, &pstate);
	pstate.enabled = false;
	pstate.period = hap->pwm_setting.period_ns;
	pstate.duty_cycle = hap->pwm_setting.duty_ns;
	pstate.output_type = PWM_OUTPUT_FIXED;
	pstate.output_pattern = NULL;
	rc = pwm_apply_state(hap->pwm_dev, &pstate);
	if (rc < 0) {
		printk(KERN_ERR"ccm_vibrator: pwm_apply_state fail: rc %d\n", rc);
	} else {
		printk(KERN_ERR"ccm_vibrator: pwm_apply %d\n", hap->pwm_setting.period_ns);
	}
#else
	pwm_disable(hap->pwm_dev);
#endif
	hap->pwm_enabled = 0;
	/*reset stage */
	hap->stage = hap->pull_start_stage_index;
	total_time = 0;
	true_runtime = 0;

	/*disable motor gpio*/
	rc = vivo_disable_vib(hap);
	if (rc < 0) {
		printk(KERN_ERR"ccm_vibrator: disable vib fail");
	}
	printk(KERN_ERR"ccm_vibrator: stop_motor_work");
}

static void vivo_vib_stage_enable(struct qpnp_hap *hap, int time_ms, bool call_by_service)
{
	int rc;
	/* direction of motor's movement, 0:pop up, 1:withdraw*/
	int runtime = 0;
	int temp_period = 0;
	bool is_jammed = false;
	int first_p  = hap->pull_first_stage_period;
	int second_p = hap->pull_second_stage_period;
	int third_p  = hap->pull_third_stage_period;
#if defined(PWM_NEW_FRAMEWORK_API)
	struct pwm_state pstate;
#endif

#if defined(PWM_NEW_FRAMEWORK_API)
	pwm_get_state(hap->pwm_dev, &pstate);
#endif

	//motor_runtime = time_ms;
	dir = time_ms >> 16;
	runtime = time_ms & 0xFFFF;
	printk(KERN_ERR "ccm_vibrator: enter vivo_vib_stage_enable runtime:%d, call_by_service:%d\n", runtime, call_by_service);
	if (runtime == 0xEEEE) {
		printk(KERN_ERR "ccm_vibrator: is_jammed");
		is_jammed = true;
	} else {
		printk(KERN_ERR "ccm_vibrator: is_normal");
		is_jammed = false;
	}
	if (dir) {
		first_p = hap->pull_first_stage_period;
		second_p = hap->pull_second_stage_period;
		third_p = hap->pull_third_stage_period;
	} else {
		first_p = hap->push_first_stage_period;
		second_p = hap->push_second_stage_period;
		third_p = hap->push_third_stage_period;
	}
	printk(KERN_ERR "ccm_vibrator: stage:%d The enable value is %d, dir is %d, current_dir:%d, runtime is %d\n", hap->stage, time_ms, dir, current_dir, runtime);
	printk(KERN_ERR "ccm_vibrator: total:%d, true:%d, run:%d\n", total_time, true_runtime, runtime);

	/* 0.stop last task if there's one*/
	if (call_by_service && !is_jammed) {
		rc = cancel_delayed_work_sync(&hap->delayed_work);
		if (rc) {
			printk(KERN_ERR "ccm_vibrator: call_by_service cancel delayed work successful");
			//stop_motor_work(hap);
			//if (runtime == 0) {
			//	printk(KERN_ERR "ccm_vibrator: stop vibrator immediately");
			//	return;
			//}
			/*sleep 10ms*/
			//usleep_range(10000, 12000);
		} else {
			printk(KERN_ERR "ccm_vibrator: call_by_service no delayed work or cancel failed");
		}

		stop_motor_work(hap);
		/*sleep 10ms*/
		usleep_range(10000, 12000);

		if (!time_ms) {
			printk(KERN_ERR "ccm_vibrator: times 0 stop motor\n");
			return;
		}
		if (dir) {
			hap->stage = hap->pull_start_stage_index;
		} else {
			hap->stage = hap->push_start_stage_index;
		}
		current_dir = dir;
		total_time = runtime;
		true_runtime = 0;
	}

	if (is_jammed) {
		rc = cancel_delayed_work_sync(&hap->delayed_work);
		if (rc) {
			printk(KERN_ERR "ccm_vibrator: is_jammed delayed work successful");
			stop_motor_work(hap);
			/*sleep 10ms*/
			usleep_range(10000, 12000);
		} else {
			printk(KERN_ERR "ccm_vibrator: is_jammed cancel failed");
		}
		hap->stage = 2;
#if defined(PWM_NEW_FRAMEWORK_API)
		third_p = hap->push_pull_low_temperature_period;
#else
		third_p = 80;
#endif
		current_dir = dir;
		total_time = hap->push_pull_low_temperature_time;
		runtime = hap->push_pull_low_temperature_time;
		true_runtime = 0;
	}

	if (total_time <= 0) {
		stop_motor_work(hap);
		usleep_range(10000, 12000);
		return;
	}

	/* 1.set the motor mode */
	if (call_by_service || is_jammed) {
		pr_err("ccm_vibrator: only run in call_by_service=%d or is_jammed %d enable vib dir :%d\n", call_by_service, is_jammed, dir);
		rc = vivo_enable_vib(hap);
	}

	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator: set mode error\n");
		return;
	}
	printk(KERN_ERR "ccm_vibrator: 1\n");

	/* 2.set pwm duty cyle:100, period: 415us(2.4KHz) */
	printk(KERN_ERR "ccm_vibrator: stage %d start\n", hap->stage);
	if (hap->stage == 0) {
		/* first stage */
		if (current_dir) {
			if (runtime > hap->pull_first_stage_time) {
				runtime = hap->pull_first_stage_time;
			}
		} else {
			if (runtime > hap->push_first_stage_time) {
				runtime = hap->push_first_stage_time;
			}
		}
		temp_period = first_p;
	} else if (hap->stage == 1) {
		/* second stage */
		if (current_dir) {
			if (runtime > hap->pull_second_stage_time) {
				runtime = hap->pull_second_stage_time;
			}
		} else {
			if (runtime > hap->push_second_stage_time) {
				runtime = hap->push_second_stage_time;
			}
		}
		temp_period = second_p;
	} else if (hap->stage == 2) {
		/* third stage */
		//runtime = 250;
		temp_period = third_p;
		if (is_jammed) {
			printk(KERN_ERR "ccm_vibrator: is_jammed stage 2\n");
		}
	}
	printk(KERN_ERR "ccm_vibrator: period:%d", temp_period);
#if defined(PWM_NEW_FRAMEWORK_API)
	hap->pwm_setting.period_ns = temp_period;
	hap->pwm_setting.duty_ns = (126 * temp_period)/255;
	pstate.enabled = true;
	pstate.period = hap->pwm_setting.period_ns;
	pstate.duty_cycle = hap->pwm_setting.duty_ns;
	pstate.output_type = PWM_OUTPUT_FIXED;
	pstate.output_pattern = NULL;
#else
	rc = pwm_config_us(hap->pwm_dev, (126 * temp_period)/255, temp_period);
	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator: pwm config failed\n");
		return;
	}
#endif
	/* pwm config end */

	printk(KERN_ERR "ccm_vibrator: 2\n");

	/* 3.set direction */
	if (!strncmp(camera_push_pull_type, "negative", 8)) {
		if (dir == 1) {
			rc = motor_gpio_direction_output(hap->dir, 1);
			if (rc < 0) {
				printk(KERN_ERR "ccm_vibrator: Unable to set dir gpio: %d to 1, the rc is %d \n", hap->dir, rc);
			}
		} else if (dir == 0) {
			rc = motor_gpio_direction_output(hap->dir, 0);
			if (rc < 0) {
				printk(KERN_ERR "%s Unable to set di gpio: %d to 0, the rc is %d \n", __func__, hap->dir, rc);
			}
		}
	} else {
		if (dir == 0) {
			rc = motor_gpio_direction_output(hap->dir, 1);
			if (rc < 0) {
				printk(KERN_ERR "ccm_vibrator: Unable to set gpio: %d to 1, the rc is %d \n", hap->dir, rc);
			}
		} else if (dir == 1) {
			rc = motor_gpio_direction_output(hap->dir, 0);
			if (rc < 0) {
				printk(KERN_ERR "%s Unable to set gpio: %d to 0, the rc is %d \n", __func__, hap->dir, rc);
			}
		}
	}

	/* 4.pull nenable low, hold 200ns*/
	rc = motor_gpio_direction_output(hap->nenable, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set nenable gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
	}

	usleep_range(1, 3);
	printk(KERN_ERR "ccm_vibrator: 3\n");


	/* 5.enable pwm */
	if (1) {
	#if defined(PWM_NEW_FRAMEWORK_API)
		printk(KERN_ERR"ccm_vibrator: pwm_apply_state %d period %d\n", pstate.enabled, hap->pwm_setting.period_ns);
		rc = pwm_apply_state(hap->pwm_dev, &pstate);
	#else
		printk(KERN_ERR"ccm_vibrator: before pwm_enable\n");
		rc = pwm_enable(hap->pwm_dev);
	#endif
		if (rc < 0)
			printk(KERN_ERR"ccm_vibrator: pwm enable fail!\n");
		hap->pwm_enabled = 1;
	}

	printk(KERN_ERR "ccm_vibrator: 4: runtime :%d\n", runtime);
	true_runtime = runtime;
	rc = queue_delayed_work(motor_work_queue, &hap->delayed_work, msecs_to_jiffies(runtime));
	if (!rc) {
		printk(KERN_ERR "ccm_vibrator: The queue_work return is %d\n", rc);
		return;
	}

}

static void vivo_vib_enable(struct timed_output_dev *dev, int time_ms)
{
	struct qpnp_hap *hap = container_of(dev, struct qpnp_hap,
				 timed_dev);
	printk(KERN_ERR "ccm_vibrator: enter vivo_vib_enable\n");
	mutex_lock(&hap->lock);
	vivo_vib_stage_enable(hap, time_ms, true);
	mutex_unlock(&hap->lock);
}

/* work func */
static void motor_work_func(struct work_struct *work)
{
	struct qpnp_hap *hap = container_of(work, struct qpnp_hap, delayed_work.work);
	int time_ms;
	printk(KERN_ERR "ccm_vibrator: motor_work_func\n");
	printk(KERN_ERR "ccm_vibrator stage:%d end.total:%d, true:%d\n", hap->stage, total_time, true_runtime);
	if (hap->stage == 0 || hap->stage == 1) {
		hap->stage++;
		total_time = total_time - true_runtime;
		time_ms = ((current_dir << 16) | (total_time & 0xFFFF));
		/*disable the pwm*/
		pwm_disable(hap->pwm_dev);
		hap->pwm_enabled = 0;
		vivo_vib_stage_enable(hap, time_ms, false);
	} else if (hap->stage == 2) {
		stop_motor_work(hap);
	}
}
/* end */

/*sysfs attributes*/
static ssize_t motor_period_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char *p = NULL;
	char *temp_buf;
	unsigned long push_first_stage_period_tmp = 0, push_second_stage_period_tmp = 0, push_third_stage_period_tmp = 0;
	unsigned long pull_first_stage_period_tmp = 0, pull_second_stage_period_tmp = 0, pull_third_stage_period_tmp = 0;
	unsigned long push_pull_low_temperature_period_tmp = 0;

	temp_buf = kmemdup(buf, count + 1, GFP_KERNEL);
	if (!temp_buf)
		return -ENOMEM;
	temp_buf[count] = '\0';

	printk(KERN_ERR"ccm_vibrator: motor temp_buf is %s\n", temp_buf);

	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &push_first_stage_period_tmp);
	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &push_second_stage_period_tmp);
	p = strsep(&temp_buf, " ");
	if (p)
		kstrtoul(p, 10, &push_third_stage_period_tmp);

	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &pull_first_stage_period_tmp);
	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &pull_second_stage_period_tmp);
	p = strsep(&temp_buf, " ");
	if (p)
		kstrtoul(p, 10, &pull_third_stage_period_tmp);

	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &push_pull_low_temperature_period_tmp);

	printk(KERN_ERR"ccm_vibrator: motor push-period 1st %d 2nd %d 3rd %d (ns)\n", (int)push_first_stage_period_tmp,
								(int)push_second_stage_period_tmp, (int)push_third_stage_period_tmp);
	printk(KERN_ERR"ccm_vibrator: motor pull-period 1st %d 2nd %d 3rd %d (ns)\n", (int)pull_first_stage_period_tmp,
								(int)pull_second_stage_period_tmp, (int)pull_third_stage_period_tmp);
	printk(KERN_ERR"ccm_vibrator: motor push or pull low temperature period %d(ns)\n", (int)push_pull_low_temperature_period_tmp);

	if (push_first_stage_period_tmp != 0) {
		p_hap->push_first_stage_period = push_first_stage_period_tmp;
	}
	if (push_second_stage_period_tmp != 0) {
		p_hap->push_second_stage_period = push_second_stage_period_tmp;
	}
	if (push_third_stage_period_tmp != 0) {
		p_hap->push_third_stage_period = push_third_stage_period_tmp;
	}

	if (pull_first_stage_period_tmp != 0) {
		p_hap->pull_first_stage_period = pull_first_stage_period_tmp;
	}
	if (pull_second_stage_period_tmp != 0) {
		p_hap->pull_second_stage_period = pull_second_stage_period_tmp;
	}
	if (pull_third_stage_period_tmp != 0) {
		p_hap->pull_third_stage_period = pull_third_stage_period_tmp;
	}

	if (push_pull_low_temperature_period_tmp != 0) {
		p_hap->push_pull_low_temperature_period = push_pull_low_temperature_period_tmp;
	}


	return count;
}

static ssize_t motor_runtime_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	char *p = NULL;
	char *temp_buf;
	unsigned long push_first_stage_time_tmp = 0, push_second_stage_time_tmp = 0, push_start_stage_index_tmp = 0;
	unsigned long pull_first_stage_time_tmp = 0, pull_second_stage_time_tmp = 0, pull_start_stage_index_tmp = 0;
	unsigned long push_pull_low_temperature_time_tmp = 0;

	temp_buf = kmemdup(buf, count + 1, GFP_KERNEL);
	if (!temp_buf)
		return -ENOMEM;
	temp_buf[count] = '\0';

	printk(KERN_ERR"ccm_vibrator: motor_runtime_store temp_buf is %s\n", temp_buf);

	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &push_first_stage_time_tmp);
	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &push_second_stage_time_tmp);
	p = strsep(&temp_buf, " ");
	if (p)
		kstrtoul(p, 10, &push_start_stage_index_tmp);

	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &pull_first_stage_time_tmp);
	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &pull_second_stage_time_tmp);
	p = strsep(&temp_buf, " ");
	if (p)
		kstrtoul(p, 10, &pull_start_stage_index_tmp);

	p = strsep(&temp_buf, ", ");
	if (p)
		kstrtoul(p, 10, &push_pull_low_temperature_time_tmp);

	printk(KERN_ERR"ccm_vibrator: motor push-time 1st %d 2nd %d 3rd %d (ns)\n", (int)push_first_stage_time_tmp,
								(int)push_second_stage_time_tmp, (int)push_start_stage_index_tmp);
	printk(KERN_ERR"ccm_vibrator: motor pull-time 1st %d 2nd %d 3rd %d (ns)\n", (int)pull_first_stage_time_tmp,
								(int)pull_second_stage_time_tmp, (int)pull_start_stage_index_tmp);
	printk(KERN_ERR"ccm_vibrator: motor push or pull low temperature time %d(ns)\n", (int)push_pull_low_temperature_time_tmp);

	p_hap->push_first_stage_time = push_first_stage_time_tmp;
	p_hap->push_second_stage_time = push_second_stage_time_tmp;
	if (push_start_stage_index_tmp >= 0 && push_start_stage_index_tmp <= 2) {
		p_hap->push_start_stage_index = push_start_stage_index_tmp;
	} else {
		p_hap->push_start_stage_index = 0;//default 3 stage mode
	}

	p_hap->pull_first_stage_time = pull_first_stage_time_tmp;
	p_hap->pull_second_stage_time = pull_second_stage_time_tmp;
	if (pull_start_stage_index_tmp >= 0 && pull_start_stage_index_tmp <= 2) {
		p_hap->pull_start_stage_index = pull_start_stage_index_tmp;
	} else {
		p_hap->pull_start_stage_index = 0;//default 3 stage mode
	}

	if (push_pull_low_temperature_time_tmp != 0) {
		p_hap->push_pull_low_temperature_time = push_pull_low_temperature_time_tmp;
	}

	printk(KERN_ERR"ccm_vibrator: motor push_start_stage_index %d pull_start_stage_index %d low_temperature_time %d\n", (int)p_hap->push_start_stage_index,
								(int)p_hap->pull_start_stage_index, (int)p_hap->push_pull_low_temperature_time);
	return count;
}
/*
static int vivo_pow(int ori, int count)
{
	int i = 0;
	int sum = ori;

	for(i = 1; i < count; i++) {
		sum = ori*ori;
	}

	return sum;
}*/

static ssize_t motor_step_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long step = 0;
	int rc = 0,  i = 0;

	printk(KERN_ERR"ccm_vibrator: motor step is %s\n", buf);
	rc = kstrtoul(buf, 10, &step);
	if (rc)
		return rc;
	printk(KERN_ERR "ccm_vibrator: The step number is %lu\n", step);
	for (i = 3; i >= 0; i--) {
		vivo_step[i] = (int)step%10;
		step = step/10;
	}
	printk(KERN_ERR "ccm_vibrator: step is %d, %d, %d, %d\n", vivo_step[0], vivo_step[1], vivo_step[2], vivo_step[3]);

	return count;
}

static ssize_t cali_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk(KERN_ERR"ccm_vibrator: cali_data is %s, count:%zd\n", buf, count);
	memset(p_hap->cali_data, 0, 30);
	memcpy(p_hap->cali_data, buf, count < 30 ? count : 30);

	return count;
}
static ssize_t cali_data_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 30, "%s", p_hap->cali_data);
}
static struct device_attribute qpnp_hap_attrs[] = {
	__ATTR(period, 0664, NULL, motor_period_store),
	__ATTR(runtime, 0664, NULL, motor_runtime_store),
	__ATTR(step, 0664, NULL, motor_step_store),
	__ATTR(cali_data, 0664, cali_data_show, cali_data_store),
};
/*sysfs attributes end*/

/* get time api to know the remaining time */
static int qpnp_hap_get_time(struct timed_output_dev *dev)
{
	return 0;
}


static int qpnp_haptic_probe(struct platform_device *pdev)
{
	struct qpnp_hap *hap;
	int rc, i = 0;

	printk("ccm_vibrator: Enter the probe, platform_device name is %s\n", pdev->name);
	hap = devm_kzalloc(&pdev->dev, sizeof(*hap), GFP_KERNEL);
	if (!hap)
		return -ENOMEM;

	hap->pdev = pdev;

	dev_set_drvdata(&pdev->dev, hap);

	printk("ccm_vibrator: before qpnp_hap_parse_dt\n");
	rc = qpnp_hap_parse_dt(hap);
	if (rc < 0) {
		pr_err("DT parsing failed\n");
		return rc;
	}

	/*added for set pinctrl state*/

	hap->pwm_enabled = 0;

	mutex_init(&hap->lock);

	/* init delay work */
	motor_work_queue = create_singlethread_workqueue("motor_queue");
	if (!motor_work_queue) {
		pr_err("Fail to create gpio_work_queue\n");
		goto timed_output_fail;
	}
	INIT_DELAYED_WORK(&hap->delayed_work, motor_work_func);

	/*function*/
	hap->timed_dev.name = "motor";
	hap->timed_dev.get_time = qpnp_hap_get_time;
	hap->timed_dev.enable = vivo_vib_enable;

	/*register*/
	printk("ccm_vibrator: before timed_output_dev_register\n");
	rc = timed_output_dev_register(&hap->timed_dev);
	if (rc < 0) {
		pr_err("ccm_vibrator: timed_output registration failed\n");
		goto timed_output_fail;
	}

	printk("ccm_vibrator: before create attributes\n");
	for (i = 0; i < ARRAY_SIZE(qpnp_hap_attrs); i++) {
		rc = sysfs_create_file(&hap->timed_dev.dev->kobj,
				&qpnp_hap_attrs[i].attr);
		if (rc < 0) {
			pr_err("sysfs creation failed\n");
			goto sysfs_fail;
		}
	}

	hap->cali_data = devm_kzalloc(&pdev->dev, 30, GFP_KERNEL);
	p_hap = hap;
	printk("ccm_vibrator: Probe ok!\n");
	return 0;

timed_output_fail:
	cancel_delayed_work_sync(&hap->delayed_work);
	mutex_destroy(&hap->lock);

sysfs_fail:
   for (; i > 0; i--)
	   sysfs_remove_file(&hap->timed_dev.dev->kobj,
			   &qpnp_hap_attrs[i].attr);
   timed_output_dev_unregister(&hap->timed_dev);

	return rc;
}

/* suspend routines to turn off haptics */
#ifdef CONFIG_PM
static int qpnp_haptic_suspend(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_haptic_pm_ops, qpnp_haptic_suspend, NULL);

static int qpnp_haptic_remove(struct platform_device *pdev)
{
	struct qpnp_hap *hap = dev_get_drvdata(&pdev->dev);
	timed_output_dev_unregister(&hap->timed_dev);
	mutex_destroy(&hap->lock);

	return 0;
}

static const struct of_device_id spmi_match_table[] = {
	{ .compatible = "vivo,sensor_vib_8846", },
	{ },
};

static struct platform_driver qpnp_haptic_driver = {
	.driver	 = {
		.name	   = "vivo,sensor_vib_8846",
		.of_match_table = spmi_match_table,
		.pm	 = &qpnp_haptic_pm_ops,
	},
	.probe	  = qpnp_haptic_probe,
	.remove	 = qpnp_haptic_remove,
};

static int __init qpnp_haptic_init(void)
{
	return platform_driver_register(&qpnp_haptic_driver);
}
module_init(qpnp_haptic_init);

static void __exit qpnp_haptic_exit(void)
{
	return platform_driver_unregister(&qpnp_haptic_driver);
}
module_exit(qpnp_haptic_exit);

MODULE_DESCRIPTION("qpnp haptic driver");
MODULE_LICENSE("GPL v2");
