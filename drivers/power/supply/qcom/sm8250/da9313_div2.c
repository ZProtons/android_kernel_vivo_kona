/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
//#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>



#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug pr_info

#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg dev_info

struct da9313_priv {
	struct regmap *regmap;
	struct device *dev;
};

static struct regmap_config da9313_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x34,
};

static int da9313_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct da9313_priv *chip;
	int rc = 0;
	int regs[0x34] = {0};
	int i = 0;

	chip = devm_kzalloc(&i2c->dev, sizeof(*chip),
				GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &i2c->dev;
	dev_dbg(chip->dev, "%s enter\n", __func__);

	i2c_set_clientdata(i2c, chip);

	chip->regmap = devm_regmap_init_i2c(i2c, &da9313_regmap_config);
	if (IS_ERR_OR_NULL(chip->regmap)) {
		dev_err(chip->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!chip->regmap) {
			rc = -EINVAL;
			goto OUT;
		}
		rc = PTR_ERR(chip->regmap);
		goto OUT;
	}

	for (i = 0; i < 0x34; i++) {
		regmap_read(chip->regmap, i, &regs[i]);
		pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, i, regs[i]);
	}

	dev_dbg(chip->dev, "%s leave\n", __func__);
	return 0;

OUT:
	devm_kfree(&i2c->dev, chip);
	return rc;
}

static int da9313_remove(struct i2c_client *i2c)
{
	struct da9313_priv *chip =
			(struct da9313_priv *)i2c_get_clientdata(i2c);

	if (!chip)
		return -EINVAL;

	dev_dbg(chip->dev, "%s\n", __func__);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int da9313_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct da9313_priv *chip =
			(struct da9313_priv *)i2c_get_clientdata(i2c);

	if (!chip)
		return -EINVAL;
	dev_dbg(chip->dev, "%s\n", __func__);

	return 0;
}

static int da9313_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct da9313_priv *chip =
			(struct da9313_priv *)i2c_get_clientdata(i2c);

	if (!chip)
		return -EINVAL;
	dev_dbg(chip->dev, "%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(da9313_pm_ops, da9313_suspend, da9313_resume);
#define da9313_PM_OPS (&da9313_pm_ops)
#else
#define da9313_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id da9313_i2c_dt_match[] = {
	{
		.compatible = "dialog,da9313-div2",
	},
	{}
};

static struct i2c_driver da9313_i2c_driver = {
	.driver = {
		.name = "da9313_driver",
		.of_match_table = da9313_i2c_dt_match,
		.pm = da9313_PM_OPS,
	},
	.probe = da9313_probe,
	.remove = da9313_remove,
};

static int __init da9313_init(void)
{
	int rc;

	rc = i2c_add_driver(&da9313_i2c_driver);
	if (rc)
		pr_err("da9313: Failed to register I2C driver: %d\n", rc);

	return rc;
}
module_init(da9313_init);

static void __exit da9313_exit(void)
{
	i2c_del_driver(&da9313_i2c_driver);
}
module_exit(da9313_exit);

MODULE_DESCRIPTION("Dialog DA9313 Divider");
MODULE_LICENSE("GPL v2");
