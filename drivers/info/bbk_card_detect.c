/*
 *  bbk_card_detect.c
 *
 * Copyright (C) 2016 Vivo, Inc.
 * Author: WangLe <wangle@vivo.com.cn>
 *
 * Description: detect SIM1 SIM2 TF-CARD inster remove state
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/bbk_drivers_info.h>

#define TAG "CARD-DETECT"

struct card_detect_data {
	bool sim1_card;
	bool sim2_card;
	bool tf_card;

	unsigned sim1_gpio_value;
	unsigned sim2_gpio_value;
	unsigned tf_card_gpio_value;

	unsigned sim1_gpio;
	unsigned sim2_gpio;
	unsigned tf_card_gpio;
	unsigned sim1_gpio_flags;
	unsigned sim2_gpio_flags;
	unsigned tf_card_gpio_flags;
};

struct card_detect_data *cd_data;

static int card_detect_gpio_state(int gpio_num)
{
	int ret = -1;
	
	if (!gpio_is_valid(gpio_num)) {
		pr_err("%s: gpio detect gpio %d is not valid\n",
			TAG, gpio_num);
		return ret;
	}

	return gpio_get_value(gpio_num);
}

static ssize_t card_slot_inserted_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	int gpio_state = -1;
	int count = 0;
	int flag = 0;

	if (!(cd_data->sim1_card || cd_data->sim2_card || cd_data->tf_card)) {
		count += snprintf(&buf[count], PAGE_SIZE,
			"can not config shrapnel detection ERROR\n");
		return count;
	}

	if (cd_data->sim1_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim1_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is invalid\n",
				cd_data->sim1_gpio);
			return -EIO;
		} else if (gpio_state != cd_data->sim1_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is open drain\n",
				cd_data->sim1_gpio);
			flag = -1;
		}	
	}

	if (cd_data->sim2_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim2_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is invalid\n", 
				cd_data->sim2_gpio);
			return -EIO;
		} else if (gpio_state != cd_data->sim2_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is open drain\n",
				cd_data->sim2_gpio);
			flag = -1;
		}	
	}

	if (cd_data->tf_card) {
		gpio_state = card_detect_gpio_state(cd_data->tf_card_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "tf card gpio %d is invalid\n", 
				cd_data->tf_card_gpio);
			return -EIO;
		} else if (gpio_state != cd_data->tf_card_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "tf card gpio %d is open drain\n", 
				cd_data->tf_card_gpio);
			flag = -1;
		}	
	}

	if (flag == 0)
		count += snprintf(&buf[count], PAGE_SIZE, "OK\n");
	else
		count += snprintf(&buf[count], PAGE_SIZE, "ERROR\n");
	
	return count;
}

static ssize_t card_slot_removed_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	int gpio_state = -1;
	int count = 0;
	int flag = 0;

	if (!(cd_data->sim1_card || cd_data->sim2_card || cd_data->tf_card)) {
		count += snprintf(&buf[count], PAGE_SIZE, "can not config shrapnel detection ERROR\n");
		return count;
	}
	
	if (cd_data->sim1_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim1_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is invalid\n", 
				cd_data->sim1_gpio);
			return -EIO;
		} else if (gpio_state == cd_data->sim1_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is short\n", 
				cd_data->sim1_gpio);
			flag = -1;
		}	
	}
	
	if (cd_data->sim2_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim2_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is invalid\n",
				cd_data->sim2_gpio);
			return -EIO;
		} else if (gpio_state == cd_data->sim2_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is short\n", 
				cd_data->sim2_gpio);
			flag = -1;
		}	
	}
	
	if (cd_data->tf_card) {
		gpio_state = card_detect_gpio_state(cd_data->tf_card_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "tf card gpio %d is invalid\n",
				cd_data->tf_card_gpio);
			return -EIO;
		} else if (gpio_state == cd_data->tf_card_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "tf card gpio %d is short\n", 
				cd_data->tf_card_gpio);
			flag = -1;
		}	
	}

	if (flag == 0)
		count += snprintf(&buf[count], PAGE_SIZE, "OK\n");
	else
		count += snprintf(&buf[count], PAGE_SIZE, "ERROR\n");
	
	return count;
}

static ssize_t card_slot_status_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	int gpio_state = -1;
	int count = 0;


	if (!(cd_data->sim1_card || cd_data->sim2_card || cd_data->tf_card)) {
		count += snprintf(&buf[count], PAGE_SIZE, "can not config shrapnel detection ERROR\n");
		return count;
	}
	
	if (cd_data->sim1_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim1_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is invalid\n", 
				cd_data->sim1_gpio);
			return -EIO;
		} else if (gpio_state == cd_data->sim1_gpio_value) {  /* short */
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-SIM1:1,", 
				cd_data->sim1_gpio);	
		} else if (gpio_state != cd_data->sim1_gpio_value) {  /*open drain */
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-SIM1:0,", 
				cd_data->sim1_gpio);
		}		
	}
	
	if (cd_data->sim2_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim2_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is invalid\n", 
				cd_data->sim2_gpio);
			return -EIO;
		} else if (gpio_state == cd_data->sim2_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-SIM2:1,", 
				cd_data->sim2_gpio);	
		} else if (gpio_state != cd_data->sim2_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-SIM2:0,", 
				cd_data->sim2_gpio);
		}		
	}
	
	if (cd_data->tf_card) {
		gpio_state = card_detect_gpio_state(cd_data->tf_card_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "tf card gpio %d is invalid\n", 
				cd_data->tf_card_gpio);
			return -EIO;
		} else if (gpio_state == cd_data->tf_card_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-TF:1", 
				cd_data->tf_card_gpio);
		} else if (gpio_state != cd_data->tf_card_gpio_value) {
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-TF:0", 
				cd_data->tf_card_gpio);
		}		
	}

	return count;
}


static struct debug_sysfs_entry card_slot_inserted_check = 
		__ATTR(card_slot_inserted_check, S_IRUGO, card_slot_inserted_show, NULL);
static struct debug_sysfs_entry card_slot_removed_check = 
		__ATTR(card_slot_removed_check, S_IRUGO, card_slot_removed_show, NULL);
static struct debug_sysfs_entry card_slot_status_check = 
		__ATTR(card_slot_status_check, S_IRUGO, card_slot_status_show, NULL);


static int card_detect_parse_dt(struct device *dev, struct card_detect_data *cdata)
{
	int ret = -1;
	struct device_node *np = dev->of_node;
	bool reverse_detect = false;
	
	cdata->sim1_card = of_property_read_bool(np, "factory-test,sim1-card");
	cdata->sim2_card = of_property_read_bool(np, "factory-test,sim2-card");
	cdata->tf_card = of_property_read_bool(np, "factory-test,tf-card");
	
	if (cdata->sim1_card) {
		cdata->sim1_gpio = of_get_named_gpio_flags(np, 
			"card-detect-sim1,gpios", 0, &cdata->sim1_gpio_flags);
		if (!gpio_is_valid(cdata->sim1_gpio)) {
			pr_err("%s: sim detect gpio %d is not valid\n", 
				TAG, cdata->sim1_gpio);
			return ret;
		}

		reverse_detect = of_property_read_bool(np, "vivo,sim1-reverse-detect");
		if (reverse_detect)
			cdata->sim1_gpio_value = 1;
		else
			cdata->sim1_gpio_value = 0;
	}
	
	if (cdata->sim2_card) {
		cdata->sim2_gpio = of_get_named_gpio_flags(np, 
			"card-detect-sim2,gpios", 0, &cdata->sim2_gpio_flags);
		if (!gpio_is_valid(cdata->sim2_gpio)) {
			pr_err("%s: sim detect gpio %d is not valid\n",
				TAG, cdata->sim2_gpio);
			return ret;
		}

		reverse_detect = of_property_read_bool(np, "vivo,sim2-reverse-detect");
		if (reverse_detect)
			cdata->sim2_gpio_value = 1;
		else
			cdata->sim2_gpio_value = 0;
	}
	
	if (cdata->tf_card) {
		cdata->tf_card_gpio = of_get_named_gpio_flags(np, 
			"card-detect-tf-card,gpios", 0, &cdata->tf_card_gpio);
		if (!gpio_is_valid(cdata->tf_card_gpio)) {
			pr_err("%s: tf detect gpio %d is not valid\n",
				TAG, cdata->tf_card_gpio);
			return ret;
		}

		reverse_detect = of_property_read_bool(np, "vivo,tf-card-reverse-detect");
		if (reverse_detect)
			cdata->tf_card_gpio_value = 1;
		else
			cdata->tf_card_gpio_value = 0;
	}

	return 0;
}

static int card_detect_probe(struct platform_device *pdev)
{

	struct card_detect_data *card_detect_data;
	int ret = 0;

	card_detect_data = kzalloc(sizeof(struct card_detect_data), GFP_KERNEL);
	if (!card_detect_data)
		return -ENOMEM;
		
	cd_data = card_detect_data;

	ret = card_detect_parse_dt(&pdev->dev, card_detect_data);
	if (ret < 0) {
		pr_err("%s: card detect parse dt fail \n", TAG);
		goto free_pdata;
	}
	
	ret = devs_create_sys_files(&card_slot_inserted_check.attr);
	if (ret < 0) {
		pr_err("%s: card inserted sys files create error \n", TAG);
		goto free_pdata;
	}
	
	ret = devs_create_sys_files(&card_slot_removed_check.attr);
	if (ret < 0) {
		pr_err("%s: card removed sys files create error \n", TAG);
		goto free_pdata;
	}
	
	ret = devs_create_sys_files(&card_slot_status_check.attr);
	if (ret < 0) {
		pr_err("%s: card removed sys files create error \n", TAG);
		goto free_pdata;
	}

	return 0;

free_pdata:
	kfree(card_detect_data);
	
	return ret;
}

static int card_detect_remove(struct platform_device *pdev)
{
	struct card_detect_data *card_detect_data = platform_get_drvdata(pdev);

	gpio_free(card_detect_data->sim1_gpio);
	gpio_free(card_detect_data->sim2_gpio);
	gpio_free(card_detect_data->tf_card_gpio);
	kfree(card_detect_data);

	return 0;
}

#ifdef CONFIG_OF              
static struct of_device_id card_match_table[] = {
	{ .compatible = "card-detect",},
	{},
};
#endif 

static struct platform_driver card_detect_driver = {
	.probe      = card_detect_probe,
	.remove     = card_detect_remove,
	.driver     = {
		.name   = "card-detect",       
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = card_match_table,
#endif
	},
};

static int __init card_detect_init(void)
{
	return platform_driver_register(&card_detect_driver);
}

static void __exit card_detect_exit(void)
{
	platform_driver_unregister(&card_detect_driver);
}

module_init(card_detect_init);
module_exit(card_detect_exit);

MODULE_AUTHOR("WangLe <wangle@vivo.com.cn>");
MODULE_DESCRIPTION("CARD state detect");
MODULE_LICENSE("GPL");

