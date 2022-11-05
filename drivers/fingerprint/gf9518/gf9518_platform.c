/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#define pr_fmt(fmt)     "[FP_KERN] " KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf9518_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf9518_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;

	gf_dev->vdd_use_gpio = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,vdd_use_gpio");
	gf_dev->vdd_use_pmic = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,vdd_use_pmic");
	gf_dev->vddio_use_gpio = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,vddio_use_gpio");
	gf_dev->vddio_use_pmic = of_property_read_bool(gf_dev->spi->dev.of_node, "goodix,vddio_use_pmic");

	pr_info("%s vdd_use_gpio %d\n", __func__, gf_dev->vdd_use_gpio);
	pr_info("%s vdd_use_pmic %d\n", __func__, gf_dev->vdd_use_pmic);
	pr_info("%s vddio_use_gpio %d\n", __func__, gf_dev->vddio_use_gpio);
	pr_info("%s vddio_use_pmic %d\n", __func__, gf_dev->vddio_use_pmic);

	// VDD
	if (gf_dev->vdd_use_gpio) {
		gf_dev->vdd_en_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_vdd_en", 0);
		if (!gpio_is_valid(gf_dev->vdd_en_gpio)) {
			pr_info("%s VDD_EN GPIO is invalid.\n", __func__);
			return -EIO;
		} else {
			pr_info("%s VDD_EN GPIO is %d.\n", __func__, gf_dev->vdd_en_gpio);
		}
		rc = gpio_request(gf_dev->vdd_en_gpio, "goodix_vdd_en");
		if (rc) {
			pr_info("%s Failed to request VDD_EN GPIO. rc = %d\n", __func__, rc);
			return -EIO;
		} else {
			pr_info("%s Success to request VDD_EN GPIO. rc = %d\n", __func__, rc);
		}
	}

	if (gf_dev->vdd_use_pmic) {
		gf_dev->vdd = regulator_get(&gf_dev->spi->dev, "vdd");
		if (IS_ERR(gf_dev->vdd)) {
		pr_info("%s Unable to get vdd\n", __func__);
		return -EINVAL;
		} else {
			pr_info("%s Success to get vdd\n", __func__);
		}
	}

	// VDDIO
	if (gf_dev->vddio_use_gpio) {
		gf_dev->vddio_en_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_vddio_en", 0);
		if (!gpio_is_valid(gf_dev->vddio_en_gpio)) {
			pr_info("%s VDDIO_EN GPIO is invalid.\n", __func__);
			return -EIO;
		} else {
			pr_info("%s VDDIO_EN GPIO is %d.\n", __func__, gf_dev->vddio_en_gpio);
		}
		rc = gpio_request(gf_dev->vddio_en_gpio, "goodix_vddio_en");
		if (rc) {
			pr_info("%s Failed to request VDDIO_EN GPIO. rc = %d\n", __func__, rc);
			return -EIO;
		} else {
			pr_info("%s Success to request VDDIO_EN GPIO. rc = %d\n", __func__, rc);
		}
	}

	if (gf_dev->vddio_use_pmic) {
		gf_dev->vddio = regulator_get(&gf_dev->spi->dev, "vddio");
		if (IS_ERR(gf_dev->vddio)) {
			pr_info("%s Unable to get first vddio, need to get second vddio\n", __func__);
			gf_dev->vddio = regulator_get(&gf_dev->spi->dev, "vddio_s");
			if (IS_ERR(gf_dev->vddio)) {
				pr_info("%s Unable to get second vddio\n", __func__);
				return -EINVAL;
			}
		} else {
			pr_info("%s Success to get vddio\n", __func__);
		}
	}

	/*get reset resource */
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info("RESET GPIO is invalid.\n");
		return -EIO;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_info("Failed to request RESET GPIO. rc = %d\n", rc);
		return -EIO;
	}

	gpio_direction_output(gf_dev->reset_gpio, 0);

	/*get irq resourece */
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_irq", 0);
	pr_info("gf::irq_gpio:%d\n", gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		return -EIO;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_info("Failed to request IRQ GPIO. rc = %d\n", rc);
		return -EIO;
	}
	gpio_direction_input(gf_dev->irq_gpio);
	pr_info("[info] %s exit\n", __func__);
	return 0;
}

void gf9518_cleanup(struct gf_dev *gf_dev)
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

int gf9518_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

    // VDD ON
    if (gf_dev->vdd_use_gpio) {
		rc = gpio_direction_output(gf_dev->vdd_en_gpio, 1);
		if (rc) {
		pr_info("gf9518 vdd power on fail.\n");
		return -EIO;
		}
	}

	if (gf_dev->vdd_use_pmic) {
		if (regulator_is_enabled(gf_dev->vdd)) {
			pr_info("%s, vdd state:on,don't set repeatedly!\n", __func__);
			return rc;
		}

		rc = regulator_set_load(gf_dev->vdd, 600000);
		if (rc < 0) {
			pr_err("%s: vdd regulator_set_load(uA_load=%d) failed. rc=%d\n",
			__func__, 1000, rc);
		}

		if (regulator_count_voltages(gf_dev->vdd) > 0) {
			rc = regulator_set_voltage(gf_dev->vdd, 3300000, 3300000);
			if (rc) {
				pr_info(KERN_ERR "gf9518:Unable to set voltage on vdd");
			}
		}
		rc = regulator_enable(gf_dev->vdd);
	}

	// VDDIO ON
	if (gf_dev->vddio_use_gpio) {
		rc = gpio_direction_output(gf_dev->vddio_en_gpio, 1);
		if (rc) {
			pr_info("gf9518 vddio power on fail.\n");
			return -EIO;
		}
	}

	if (gf_dev->vddio_use_pmic) {
		if (regulator_is_enabled(gf_dev->vddio)) {
			pr_info("%s, vddio state:on,don't set repeatedly!\n", __func__);
			return rc;
		}

		rc = regulator_set_load(gf_dev->vddio, 600000);
		if (rc < 0) {
			pr_err("%s: vddio regulator_set_load(uA_load=%d) failed. rc=%d\n",
			__func__, 1000, rc);
		}

		if (regulator_count_voltages(gf_dev->vddio) > 0) {
			rc = regulator_set_voltage(gf_dev->vddio, 1800000, 1800000);
			if (rc) {
				pr_info(KERN_ERR "gf9518:Unable to set voltage on vddio");
			}
		}
		rc = regulator_enable(gf_dev->vddio);
	}

	pr_info("gf:%s, exit\n", __func__);

	return rc;
}

int gf9518_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

	rc = gpio_direction_output(gf_dev->reset_gpio, 0);
	if (rc) {
		pr_info("gf9518 set reset gpio output fail.\n");
		return -EIO;
	}

	// VDD OFF
	if (gf_dev->vdd_use_gpio) {
		rc = gpio_direction_output(gf_dev->vdd_en_gpio, 0);
		if (rc) {
		pr_info("gf9518 vdd power off fail.\n");
		return -EIO;
		}
	}

	if (gf_dev->vdd_use_pmic) {
		rc = regulator_set_load(gf_dev->vdd, 0);
		if (rc < 0) {
			pr_err("%s: vdd regulator_set_load(uA_load=%d) failed. rc=%d\n",
				__func__, 0, rc);
		}
		if (regulator_is_enabled(gf_dev->vdd)) {
			regulator_disable(gf_dev->vdd);
		}
		pr_info(KERN_ERR "gf9518: disable  vdd %d\n", rc);
	}

	// VDDIO OFF
	if (gf_dev->vddio_use_gpio) {
		rc = gpio_direction_output(gf_dev->vddio_en_gpio, 0);
		if (rc) {
		pr_info("gf9518 vddio power off fail.\n");
		return -EIO;
		}
	}

	if (gf_dev->vddio_use_pmic) {
	rc = regulator_set_load(gf_dev->vddio, 0);
		if (rc < 0) {
			pr_err("%s: vddio regulator_set_load(uA_load=%d) failed. rc=%d\n",
				__func__, 0, rc);
		}
		if (regulator_is_enabled(gf_dev->vddio)) {
			regulator_disable(gf_dev->vddio);
		}
		pr_info(KERN_ERR "gf9518: disable  vddio %d\n", rc);
	}

	pr_info("gf:%s, exit\n", __func__);

	return rc;
}

int gf9518_hw_get_power_state(struct gf_dev *gf_dev)
{

	int retval = 0;

	retval = gpio_get_value(gf_dev->vdd_en_gpio);
	pr_info("gf:%s, retval=%d\n", __func__, retval);

	return retval;
}

int gf9518_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -EFAULT;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);
	mdelay(10);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf9518_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -EFAULT;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

