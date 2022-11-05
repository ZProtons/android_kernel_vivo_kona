/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf3208_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf3208_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;

	gf_dev->vdd_use_gpio = of_property_read_bool(gf_dev->spi->dev.of_node, "fp,vdd_use_gpio");
	gf_dev->vdd_use_pmic = of_property_read_bool(gf_dev->spi->dev.of_node, "fp,vdd_use_pmic");

	pr_info("%s vdd_use_gpio %d\n", __func__, gf_dev->vdd_use_gpio);
	pr_info("%s vdd_use_pmic %d\n", __func__, gf_dev->vdd_use_pmic);

	if (gf_dev->vdd_use_pmic) {
		gf_dev->vreg = regulator_get(&gf_dev->spi->dev, "vcc_spi");
		if (IS_ERR(gf_dev->vreg)) {
			pr_info("%s Unable to get vcc_spi\n", __func__);
			return -EINVAL;
		}
	}

	if (gf_dev->vdd_use_gpio) {
		gf_dev->vdd_en_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_vdd_en", 0);
		if (!gpio_is_valid(gf_dev->vdd_en_gpio)) {
			pr_info("VDD_EN GPIO is invalid.\n");
			return -EINVAL;
		}
		rc = gpio_request(gf_dev->vdd_en_gpio, "goodix_vdd_en");
		if (rc) {
			pr_info("Failed to request VDD_EN GPIO. rc = %d\n", rc);
			return -EINVAL;
		}
	}

	/*get reset resource */
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info("RESET GPIO is invalid.\n");
		return -EINVAL;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
		return -EINVAL;
	}

    gpio_direction_output(gf_dev->reset_gpio, 0);

	/*get irq resourece */
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_irq", 0);
	pr_info("gf::irq_gpio:%d\n", gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		return -EINVAL;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
		return -EINVAL;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	return 0;
}

void gf3208_cleanup(struct gf_dev	 *gf_dev)
{
	pr_info("[info] %s\n", __func__);
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf3208_power_on(struct gf_dev *gf_dev)
{
    int rc = 0;
	pr_info("gf:%s, entry\n", __func__);

	if (gf_dev->vdd_use_pmic) {
		if (regulator_is_enabled(gf_dev->vreg)) {
			pr_info("%s,power state:on,don't set repeatedly!\n", __func__);
			return rc;
		}

		rc = regulator_set_load(gf_dev->vreg, 600000);
		if (rc < 0) {
			pr_err("%s: regulator_set_load(uA_load=%d) failed. rc=%d\n",
				__func__, 1000, rc);
		}

		if (regulator_count_voltages(gf_dev->vreg) > 0) {
			rc = regulator_set_voltage(gf_dev->vreg, 3000000, 3000000);
				if (rc) {
					pr_info(KERN_ERR "gf3658:Unable to set voltage on vcc_spi");
				}
		}
		rc = regulator_enable(gf_dev->vreg);
	}

	if (gf_dev->vdd_use_gpio) {
		pr_info("gf:%s, entry\n", __func__);
		rc = gpio_direction_output(gf_dev->vdd_en_gpio, 1);
		if (rc) {
			pr_info ("gf5288 power on fail.\n");
			return -EINVAL;
		}
	}

	pr_info("gf:%s, exit\n", __func__);
    return rc;
}

int gf3208_power_off(struct gf_dev *gf_dev)
{
    int rc = 0;
	pr_info("gf:%s, entry\n", __func__);

	if (gf_dev->vdd_use_pmic) {
		if (gf_dev->vreg) {
			rc = regulator_set_load(gf_dev->vreg, 0);
			if (rc < 0) {
				pr_err("%s: regulator_set_load(uA_load=%d) failed. rc=%d\n",
					__func__, 0, rc);
			}
			if (regulator_is_enabled(gf_dev->vreg)) {
				gpio_direction_output(gf_dev->reset_gpio, 0);
				regulator_disable(gf_dev->vreg);
			}
			pr_info(KERN_ERR "gf3658: disable  vcc_spi %d\n", rc);
		}
	}

	if (gf_dev->vdd_use_gpio) {
		rc = gpio_direction_output(gf_dev->reset_gpio, 0);
		if (rc) {
			pr_info("gf3658 set reset gpio output fail.\n");
			return -EIO;
		}

		rc = gpio_direction_output(gf_dev->vdd_en_gpio, 0);
		if (rc) {
			pr_info ("gf5288 power off fail.\n");
			return -EINVAL;
		}
	}

	pr_info("gf:%s, exit\n", __func__);
    return rc;
}

int gf_hw_get_power_state(struct gf_dev *gf_dev)
{

	int retval = 0;
	retval = gpio_get_value(gf_dev->vdd_en_gpio);
	pr_info("gf:%s, retval=%d\n", __func__, retval);
	return retval;
}

int gf3208_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -EINVAL;
	}
	//gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(10);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf3208_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -EINVAL;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

