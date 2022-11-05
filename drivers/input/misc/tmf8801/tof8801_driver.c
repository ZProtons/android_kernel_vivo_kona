/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/
/*!
 *  \file tof8801_driver.c - ToF8801 driver
 *  \brief Device driver for measuring Proximity / Distance in mm
 *  from within the AMS-TAOS TMF8801 family of devices.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/kfifo.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/i2c/ams/tof8801.h>
#include <uapi/linux/sched/types.h>
#include <linux/regulator/consumer.h>
#include "tof8801_driver.h"
#include "tof_hex_interpreter.h"
#include "tof8801_bootloader.h"
#include "tof8801_app0.h"

#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m)                                                      \
	{                                                                      \
		pr_info("%s: Mutex Lock\n", __func__);                         \
		mutex_lock_interruptible(m);                                   \
	}
#define AMS_MUTEX_UNLOCK(m)                                                    \
	{                                                                      \
		pr_info("%s: Mutex Unlock\n", __func__);                       \
		mutex_unlock(m);                                               \
	}
#else
#define AMS_MUTEX_LOCK(m)                                                      \
	{                                                                      \
		mutex_lock(m);                                                 \
	}
#define AMS_MUTEX_UNLOCK(m)                                                    \
	{                                                                      \
		mutex_unlock(m);                                               \
	}
#endif

#define TOF8801_AMS_NAME "tof8801"
static struct device *client_dev;
#define TOF_GET_FAC_CALIB_BIN_SIZE 14
static int clk_trim_firmware;
static char crosstalk_firmware[TOF_GET_FAC_CALIB_BIN_SIZE] = { 0 };
static u32 orig_crosstalk;

/* This is the salt used for decryption on an encrypted sensor */
static char tof_salt_value = TOF8801_BL_DEFAULT_SALT;

static const unsigned long tof_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};

static struct tof8801_platform_data tof_pdata = {
	.tof_name = "tof8801",
	//.fac_calib_data_fname = "tof8801_fac_calib.bin",
	//.clock_trim_data_fname = "tof8801_clock_trim.bin",
	.config_calib_data_fname = "tof8801_config_calib.bin",
	.ram_patch_fname = {
		"tof8801_firmware.bin",
		"tof8801_firmware-1.bin",
		"tof8801_firmware-2.bin",
		NULL,
	},
};

/*
 *
 * Function Declarations
 *
 */
static int tof8801_get_all_regs(struct tof_sensor_chip *tof_chip);
static void tof_ram_patch_callback(const struct firmware *cfg, void *ctx);
static int tof_switch_apps(struct tof_sensor_chip *chip, char req_app_id);
//static int tof8801_get_fac_calib_data(struct tof_sensor_chip *chip);
static int tof8801_get_config_calib_data(struct tof_sensor_chip *chip);
static int tof8801_firmware_download(struct tof_sensor_chip *chip, int startup);
static irqreturn_t tof_irq_handler(int irq, void *dev_id);
static int tof8801_enable_interrupts(struct tof_sensor_chip *chip,
				     char int_en_flags);
extern uint32_t saved_clkratioQ15[3];

/*
 *
 * Function Definitions
 *
 */
static ssize_t program_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");

		return scnprintf(buf, PAGE_SIZE, "%#x\n",
				 (chip->info_rec.record.app_id));
	}
	return -EPERM;
}

static ssize_t program_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	char req_app_id;
	int error;
	if (chip != NULL) {
		sscanf(buf, "%hhx", &req_app_id);
		TOF8801_INFO("requested app: %#x", req_app_id);
		AMS_MUTEX_LOCK(&chip->lock);
		error = tof_switch_apps(chip, req_app_id);
		if (error) {
			TOF8801_ERR("Error switching app: %d", error);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return error;
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return count;
	}
	return -EPERM;
}

static ssize_t chip_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int state;
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (!chip->pdata->gpiod_enable) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EIO;
		}
		state = gpiod_get_value(chip->pdata->gpiod_enable) ? 1 : 0;
		AMS_MUTEX_UNLOCK(&chip->lock);
		return scnprintf(buf, PAGE_SIZE, "%d\n", state);
	}
	return -EPERM;
}

static ssize_t chip_enable_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int req_state;
	int error;
	TOF8801_INFO("enter");
	if (chip != NULL) {
		error = sscanf(buf, "%d", &req_state);
		if (error != 1)
			return -EINVAL;
		AMS_MUTEX_LOCK(&chip->lock);
		if (!chip->pdata->gpiod_enable) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EIO;
		}
		if (req_state == 0) {
			if (chip->info_rec.record.app_id ==
			    TOF8801_APP_ID_APP0) {
				(void)tof8801_app0_capture(chip, 0);
			}
			gpiod_set_value(chip->pdata->gpiod_enable, 0);
		} else {
			error = tof_hard_reset(chip);
			if (error) {
				TOF8801_ERR("Error issuing Reset-HARD");
				AMS_MUTEX_UNLOCK(&chip->lock);
				return -EIO;
			}
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return count;
	}
	return -EPERM;
}

static ssize_t driver_debug_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		return scnprintf(buf, PAGE_SIZE, "%d\n", chip->driver_debug);
	}
	return -EPERM;
}

static ssize_t driver_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		int debug;
		TOF8801_INFO("enter");
		sscanf(buf, "%d", &debug);
		if (debug == 0) {
			chip->driver_debug = 0;
		} else {
			chip->driver_debug = 1;
		}
		return count;
	}
	return -EPERM;
}

static ssize_t app0_command_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i;
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		char *cmd_byte = chip->app0_app.user_cmd.anon_cmd.buf;
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_INFO("Error ToF chip app_id: %#x",
				     chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		for (i = TOF8801_APP0_CMD_IDX; i >= 0; i--) {
			len += scnprintf(buf - len, PAGE_SIZE - len, "%#x ",
					 cmd_byte[i]);
		}
		len += scnprintf(buf - len, PAGE_SIZE - len, "\n");
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t app0_command_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int num = 0;
	int i;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	char *cmd_byte;
	char *sub_string = NULL;
	int error;
	if (chip != NULL) {
		if (chip->driver_debug) {
			TOF8801_INFO("%s", buf);
		}
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		cmd_byte = chip->app0_app.user_cmd.anon_cmd.buf;
		memset(cmd_byte, 0,
		       TOF8801_APP0_MAX_CMD_SIZE); //clear previous command
		for (i = TOF8801_APP0_CMD_IDX; (i >= 0); i--) {
			sub_string = strsep((char **)&buf, " ");
			if (sub_string) {
				num = sscanf(sub_string, "%hhx",
					     (cmd_byte + i));
				if (num == 0) {
					break;
				}
			}
		}
		error = tof8801_app0_issue_cmd(chip);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return error ? -1 : count;
	}
	return -EPERM;
}

static ssize_t capture_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		int len = 0;
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%u\n",
				 chip->app0_app.cap_settings.cmd);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t capture_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	unsigned int capture;
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%u", &capture) != 1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (capture) {
			if (chip->app0_app.cap_settings.cmd == 0) {
				error = tof8801_app0_capture((void *)chip,
							     capture);
			} else {
				AMS_MUTEX_UNLOCK(&chip->lock);
				return -EBUSY;
			}
		} else {
			tof8801_app0_capture(chip, 0);
			TMF8801_cr_init(&chip->app0_app.clk_cr);
			(&chip->app0_app.clk_cr)->ratioQ15 =
				saved_clkratioQ15[2];
			error = 0;
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return error ? -1 : count;
	}
	return -EPERM;
}

static ssize_t app0_temp_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%d\n",
				 chip->app0_app.last_known_temp);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t period_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%d\n",
				 chip->app0_app.cap_settings.period);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t period_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	unsigned int value = 0;
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("%Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%u", &value) != 1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		chip->app0_app.cap_settings.period =
			(value > 0xFF) ? 0xFF : value;
		if (chip->app0_app.cap_settings.cmd != 0) {
			(void)tof8801_app0_capture((void *)chip, 0);
			error = tof8801_app0_capture((void *)chip, 1);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return error ? -1 : count;
	}
	return -EPERM;
}

static ssize_t iterations_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	unsigned int len = 0;
	unsigned int iterations = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		iterations =
			1000 * le16_to_cpup((const __le16 *)chip->app0_app
						    .cap_settings.iterations);
		len += scnprintf(buf, PAGE_SIZE, "%u\n", iterations);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t iterations_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	unsigned int value = 0;
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%u", &value) != 1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		// we need to appropriately change the clock iteration counter
		//  when the capture iterations are changed to keep the time acceptable
		tof8801_app0_set_clk_iterations(chip, value);
		// chip takes iterations in 1000s
		value /= 1000;
		*((__le16 *)chip->app0_app.cap_settings.iterations) =
			cpu_to_le16(value);
		if (chip->app0_app.cap_settings.cmd != 0) {
			(void)tof8801_app0_capture((void *)chip, 0);
			error = tof8801_app0_capture((void *)chip, 1);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return error ? -1 : count;
	}
	return -EPERM;
}

static ssize_t noise_threshold_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%d\n",
				 chip->app0_app.cap_settings.noise_thrshld);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t noise_threshold_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%hhd",
			   &chip->app0_app.cap_settings.noise_thrshld) != 1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (chip->app0_app.cap_settings.cmd != 0) {
			(void)tof8801_app0_capture((void *)chip, 0);
			error = tof8801_app0_capture((void *)chip, 1);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return error ? -1 : count;
	}
	return -EPERM;
}

static ssize_t capture_delay_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%d\n",
				 chip->app0_app.cap_settings.delay);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t capture_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	unsigned int value = 0;
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%u", &value) != 1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		chip->app0_app.cap_settings.delay =
			(value > 0xFF) ? 0xFF : value;
		if (chip->app0_app.cap_settings.cmd != 0) {
			(void)tof8801_app0_capture((void *)chip, 0);
			error = tof8801_app0_capture((void *)chip, 1);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return error ? -1 : count;
	}
	return -EPERM;
}

static ssize_t alg_setting_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error alg setting not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%x\n",
				 chip->app0_app.cap_settings.v2.alg);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t alg_setting_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error alg setting not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%hhx", &chip->app0_app.cap_settings.v2.alg) !=
		    1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (chip->app0_app.cap_settings.cmd != 0) {
			(void)tof8801_app0_capture((void *)chip, 0);
			error = tof8801_app0_capture((void *)chip, 1);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return error ? -1 : count;
	}
	return -EPERM;
}

static ssize_t gpio_setting_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error gpio setting not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%x\n",
				 chip->app0_app.cap_settings.v2.gpio);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t gpio_setting_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error gpio setting not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%hhx", &chip->app0_app.cap_settings.v2.gpio) !=
		    1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (chip->app0_app.cap_settings.cmd != 0) {
			(void)tof8801_app0_capture((void *)chip, 0);
			error = tof8801_app0_capture((void *)chip, 1);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return error ? -1 : count;
	}
	return -EPERM;
}

static ssize_t app0_clk_iterations_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error clk iterations not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%d\n",
				 chip->app0_app.clk_iterations);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t app0_clk_iterations_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error clk iterations not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%u", &chip->app0_app.clk_iterations) != 1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		/*set the tof8801_app0_freq_ratio high thresh and low thresh for factory clk trim*/
		chip->tof8801_app0_freq_ratio_loth_Q15 = 6953;
		chip->tof8801_app0_freq_ratio_hith_Q15 = 6991;
		AMS_MUTEX_UNLOCK(&chip->lock);
		return count;
	}
	return -EPERM;
}

static ssize_t app0_clk_trim_enable_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error clk trim not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len += scnprintf(buf, PAGE_SIZE, "%d\n",
				 chip->app0_app.clk_trim_enable);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t app0_clk_trim_enable_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("%s", buf);
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error clk trim not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (sscanf(buf, "%d", &chip->app0_app.clk_trim_enable) != 1) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return count;
	}
	return -EPERM;
}

static ssize_t app0_clk_trim_set_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	unsigned int len = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int trim = 0;
	int error = 0;
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		if (!tof8801_app0_is_v2(chip)) {
			TOF8801_ERR(
				"Error clk trim not supported in revision: %#x",
				chip->info_rec.record.app_ver);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		error = tof8801_app0_rw_osc_trim(chip, &trim, 0);
		if (error) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		//len += scnprintf(buf, PAGE_SIZE, "%d\n", trim);
		memcpy(buf, &trim, sizeof(trim));
		len = sizeof(trim);
		buf[len] = 0;

		/*set the tof8801_app0_freq_ratio high thresh and low thresh after saving factory clk trim value*/
		chip->tof8801_app0_freq_ratio_loth_Q15 = 6903;
		chip->tof8801_app0_freq_ratio_hith_Q15 = 7041;
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static int tof8801_rewrite_clock_trim_data(struct tof_sensor_chip *chip)
{
	int trim = 0;
	AMS_MUTEX_LOCK(&chip->lock);
	trim = clk_trim_firmware;
	TOF8801_INFO("trim = 0x%08x", trim);
	if ((trim > 511) || (trim < 0)) {
		TOF8801_ERR("Error clk trim setting is out of range [%d,%d]",
			    -256, 511);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return -EPERM;
	}

	chip->saved_clk_trim = trim; // cache value even if app0 is not running
	if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
		TOF8801_ERR("Caching trim value, ToF chip app_id: %#x",
			    chip->info_rec.record.app_id);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return -EPERM;
	}

	if (!tof8801_app0_is_v2(chip)) {
		TOF8801_ERR("Error clk trim not supported in revision: %#x",
			    chip->info_rec.record.app_ver);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return -EPERM;
	}

	if (tof8801_app0_rw_osc_trim(chip, &trim, 1)) {
		TOF8801_ERR("Error setting clock trimming");
		AMS_MUTEX_UNLOCK(&chip->lock);
		return -EPERM;
	}

	AMS_MUTEX_UNLOCK(&chip->lock);
	return 0;
}

static ssize_t app0_clk_trim_set_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		int error;
		TOF8801_INFO("%s", buf);
		error = tof8801_rewrite_clock_trim_data(chip);

		return count;
	}
	return -EPERM;
}

static ssize_t app0_diag_state_mask_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int len;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	TOF8801_INFO("enter");
	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
		TOF8801_ERR("Error ToF chip app_id: %#x",
			    chip->info_rec.record.app_id);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return -EINVAL;
	}
	len = scnprintf(buf, PAGE_SIZE, "%#x\n",
			chip->app0_app.diag_state_mask);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return len;
}

static ssize_t app0_reflectivity_count_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	int len;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		len = scnprintf(buf, PAGE_SIZE,
				"object hits: %u\nreference hits: %u\n",
				chip->app0_app.algo_results_frame.results_frame
					.results_v2.data.objectHits,
				chip->app0_app.algo_results_frame.results_frame
					.results_v2.data.referenceHits);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t app0_general_configuration_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error, i;
	int len = 0;
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		error = tof8801_app0_get_dataset(chip, GEN_CFG_DATASET);
		if (error) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return error;
		}
		for (i = 0; i < APP0_GENERAL_CONFIG_RSP_SIZE; i++) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%#x:%x\n",
					 i, chip->app0_app.dataset.buf[i]);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static void dump_all_registers(struct device *dev)
{
	int i = 0;
	uint8_t buf[100] = { 0 };
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);

	for (i = 0x00; i < 0x3b; i += 8) {
		tof_i2c_read(chip->client, i, &buf[i], 8);
		TOF8801_INFO(
			"0x%02x : 0x%02x, 0x%02x, 0x%02x , 0x%02x , 0x%02x , 0x%02x , 0x%02x , 0x%02x",
			i, buf[i], buf[i + 1], buf[i + 2], buf[i + 3],
			buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7]);
	}
	tof_i2c_read(chip->client, i, &buf[i], 3);
	TOF8801_INFO("0x%02x : 0x%02x, 0x%02x, 0x%02x", i, buf[i], buf[i + 1],
		     buf[i + 2]);
	return;
}
#if 1
static ssize_t app0_read_peak_crosstalk_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	unsigned int len = 0;
	int i = 0;
	int error = 0;
	char peak_value[10] = { 0 };
	uint16_t peak_crosstalk[5] = { 0 };
	uint average = 0;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		dump_all_registers(dev);

		TOF8801_INFO("ENTER");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EIO;
		}

		for (i = 0; i < 10; i += 2) {
			msleep(70);
			error = tof_i2c_read(chip->client, 0x30, &peak_value[i],
					     2);

			if (error) {
				TOF8801_ERR(
					"Error i2c communication failure: %d",
					error);
				AMS_MUTEX_UNLOCK(&chip->lock);
				return error;
			}
			peak_crosstalk[i / 2] = (uint16_t)(peak_value[i] << 8) |
						(uint16_t)peak_value[i + 1];
			TOF8801_INFO("peak_crosstalk[%d]=%04x", i / 2,
				     peak_crosstalk[i / 2]);
			average += peak_crosstalk[i / 2];
		}

		len = scnprintf(buf, PAGE_SIZE, "%d\n", average / 5);

		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}
#endif

#if 0
static ssize_t app0_read_peak_crosstalk_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int len = 0;
	int i = 0;
	int error = 0;
	char peak_value[3] = {0};
	uint32_t peak_crosstalk;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);

	dump_all_registers(dev);

	TOF8801_INFO("ENTER");
	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
	  TOF8801_ERR("Error ToF chip app_id: %#x", chip->info_rec.record.app_id);
	  AMS_MUTEX_UNLOCK(&chip->lock);
	  return -EIO;
	}

	error = tof_i2c_read(chip->client, 0x20, &peak_value[i], 3);

	if (error) {
	  TOF8801_ERR("Error i2c communication failure: %d", error);
	  AMS_MUTEX_UNLOCK(&chip->lock);
	  return error;
	}
	peak_crosstalk = (uint32_t)(peak_value[i+2] << 12) | (uint32_t)(peak_value[i+1] << 4) | (uint32_t)(peak_value[i] >> 4);
	TOF8801_INFO("peak_crosstalk=%d", peak_crosstalk);

	len = scnprintf(buf, PAGE_SIZE, "%d\n", peak_crosstalk);

	AMS_MUTEX_UNLOCK(&chip->lock);
	return len;
}
#endif

static ssize_t app0_apply_fac_calib_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int i;
	int len = 0;
	char *tmpbuf = (char *)&chip->ext_calib_data.fac_data;
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_INFO("Error ToF chip app_id: %#x",
				     chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		for (i = 0; i < chip->ext_calib_data.size; i++) {
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "fac_calib[%d]:%02x\n", i, tmpbuf[i]);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t app0_apply_fac_calib_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	//int error = 0;
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
#if 0
		error = tof8801_get_fac_calib_data(chip);
		if (error) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return error;
		}
#endif
		//set flag to update fac calib on next measure
		chip->app0_app.cal_update.dataFactoryConfig = 1;
		AMS_MUTEX_UNLOCK(&chip->lock);
		return count;
	}
	return -EPERM;
}

static ssize_t app0_apply_config_calib_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int i;
	int len = 0;
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		for (i = 0; i < chip->config_data.size; i++) {
			len += scnprintf(
				buf + len, PAGE_SIZE - len, "config[%d]:%02x\n",
				i, ((char *)&chip->config_data.cfg_data)[i]);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t app0_apply_config_calib_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		error = tof8801_get_config_calib_data(chip);
		if (error) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return error;
		}
		//set flag to update config calib data on next measure
		chip->app0_app.cal_update.dataConfiguration = 1;
		AMS_MUTEX_UNLOCK(&chip->lock);
		return count;
	}
	return -EPERM;
}

static ssize_t app0_apply_state_data_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int i;
	int len = 0;
	if (chip != NULL) {
		TOF8801_INFO("enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		for (i = 0; i < chip->alg_info.size; i++) {
			len += scnprintf(
				buf + len, PAGE_SIZE - len,
				"state_data[%d]:%02x\n", i,
				((char *)&chip->alg_info.alg_data.data)[i]);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t app0_apply_state_data_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	int num = 0;
	char state[11] = { 0 };
	if (chip != NULL) {
		TOF8801_INFO("ENTER");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		memset(chip->alg_info.alg_data.data, 0,
		       sizeof(chip->alg_info.alg_data));
		num = sscanf(
			buf,
			"%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
			&state[0], &state[1], &state[2], &state[3], &state[4],
			&state[5], &state[6], &state[7], &state[8], &state[9],
			&state[10]);
		memcpy(chip->alg_info.alg_data.data, state,
		       sizeof(chip->alg_info.alg_data));
		chip->alg_info.size = 11;
		if (num != 11) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return error;
		}
		//set flag to update config calib data on next measure
		chip->app0_app.cal_update.dataAlgorithmState = 1;
		AMS_MUTEX_UNLOCK(&chip->lock);
		return count;
	}
	return -EPERM;
}

static ssize_t program_version_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int len = 0;
	if (chip != NULL) {
		TOF8801_INFO("ENTER");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id == TOF8801_APP_ID_APP0) {
			len = tof8801_app0_get_version(chip, buf, PAGE_SIZE);
			if (len == 0) {
				AMS_MUTEX_UNLOCK(&chip->lock);
				return -EIO;
			}
		} else {
			len = scnprintf(buf, PAGE_SIZE, "%#hhx-0-0-0\n",
					chip->info_rec.record.app_ver);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t registers_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int per_line = 4;
	int len = 0;
	int idx, per_line_idx;
	int bufsize = PAGE_SIZE;
	int error;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		TOF8801_INFO("ENTER");
		AMS_MUTEX_LOCK(&chip->lock);
		error = tof8801_get_all_regs(chip);
		if (error) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return error;
		}

		for (idx = 0; idx < MAX_REGS; idx += per_line) {
			len += scnprintf(buf + len, bufsize - len,
					 "%#02x:", idx);
			for (per_line_idx = 0; per_line_idx < per_line;
			     per_line_idx++) {
				len += scnprintf(buf + len, bufsize - len, " ");
				len += scnprintf(
					buf + len, bufsize - len, "%02x",
					chip->shadow[idx + per_line_idx]);
			}
			len += scnprintf(buf + len, bufsize - len, "\n");
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t app0_ctrl_reg_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0, i;
	int len = 0;
	if (chip != NULL) {
		TOF8801_DBG("ENTER");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		//Read out APP0 header info: status, last cmd, TID, register contents, etc
		error = tof_i2c_read(chip->client, TOF8801_APP_ID,
				     chip->app0_app.ctrl_frame.buf,
				     sizeof(chip->app0_app.ctrl_frame.buf));
		if (error) {
			TOF8801_ERR("Error i2c communication failure: %d",
				    error);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return snprintf(buf, 4, "%d\n", -1);
		}
		for (i = 0; i < sizeof(chip->app0_app.ctrl_frame.buf); i++) {
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "%#02x:%02x\n", i,
					 chip->app0_app.ctrl_frame.buf[i]);
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t register_write_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	char preg;
	char pval;
	char pmask = -1;
	int numparams;
	int rc;
	TOF8801_INFO("ENTER");

	numparams = sscanf(buf, "%hhx:%hhx:%hhx", &preg, &pval, &pmask);
	if ((numparams < 2) || (numparams > 3))
		return -EINVAL;
	if ((numparams >= 1) && (preg < 0))
		return -EINVAL;
	if ((numparams >= 2) && (preg < 0 || preg > 0xff))
		return -EINVAL;
	if ((numparams >= 3) && (pmask < 0 || pmask > 0xff))
		return -EINVAL;

	if (client_dev != NULL) {
		if (pmask == -1) {
			rc = tof_i2c_write(to_i2c_client(client_dev), preg,
					   &pval, 1);
		} else {
			rc = tof_i2c_write_mask(to_i2c_client(client_dev), preg,
						&pval, pmask);
		}
	}
	return rc ? rc : count;
}

static ssize_t request_ram_patch_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error = 0;
	if (chip != NULL) {
		TOF8801_INFO("ENTER");
		AMS_MUTEX_LOCK(&chip->lock);
		/***** Make firmware download available to user space *****/
		error = tof8801_firmware_download(chip, 0);
		if (error) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return error;
		}
		/* enable all ToF interrupts on sensor */
		tof8801_enable_interrupts(chip,
					  IRQ_RESULTS | IRQ_DIAG | IRQ_ERROR);

		AMS_MUTEX_UNLOCK(&chip->lock);
		return count;
	}
	return -EPERM;
}

static void tof8801_fw_work_rountine(struct work_struct *work)
{
	struct tof_sensor_chip *chip =
		container_of(work, struct tof_sensor_chip, fw_work);
	int error = 0;
	if (chip != NULL) {
		TOF8801_INFO("enter.");
		AMS_MUTEX_LOCK(&chip->lock);
		error = tof8801_firmware_download(chip, 0);
		if (error) {
			AMS_MUTEX_UNLOCK(&chip->lock);
			TOF8801_INFO("firmware download failed.");
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
	}
}

static enum hrtimer_restart tof8801_fw_timer_func(struct hrtimer *timer)
{
	struct tof_sensor_chip *tof_chip =
		container_of(timer, struct tof_sensor_chip, fw_timer);

	TOF8801_INFO("enter");

	schedule_work(&tof_chip->fw_work);

	return HRTIMER_NORESTART;
}

static int tof8801_fw_init(struct tof_sensor_chip *tof_chip)
{
	int fw_timer_val = 8000;

	hrtimer_init(&tof_chip->fw_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tof_chip->fw_timer.function = tof8801_fw_timer_func;
	INIT_WORK(&tof_chip->fw_work, tof8801_fw_work_rountine);
	hrtimer_start(&tof_chip->fw_timer,
		      ktime_set(fw_timer_val / 1000,
				(fw_timer_val % 1000) * 1000000),
		      HRTIMER_MODE_REL);
	return 0;
}
static ssize_t app0_get_fac_calib_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error;
	u32 len;
	unsigned long start = jiffies;
	int timeout_flag = 0;
	char int_stat = 0;
	if (chip != NULL) {
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		//Stop any in-progress measurements
		(void)tof8801_app0_capture(chip, 0);

		//set VCSEL clock to 20Mhz before calibration----alg set

		chip->app0_app.cap_settings.v2.alg = 0xA7;
		chip->app0_app.cap_settings.period = 66; //set period to 66ms
		*((__le16 *)chip->app0_app.cap_settings.iterations) =
			cpu_to_le16(900); //set iterations to 900k
		//start and then stop the measurement to let alg set take effect
		error = tof8801_app0_capture((void *)chip, 1);
		if (error) {
			TOF8801_ERR("start capture error!!!");
			AMS_MUTEX_UNLOCK(&chip->lock);
			return 0;
		}
		AMS_MUTEX_UNLOCK(&chip->lock);

		msleep(100);

		AMS_MUTEX_LOCK(&chip->lock);
		error = tof8801_app0_capture((void *)chip, 0);
		if (error) {
			TOF8801_ERR("stop capture error!!!");
			AMS_MUTEX_UNLOCK(&chip->lock);
			return 0;
		}

		error = tof8801_app0_perform_factory_calibration(chip);
		if (error) {
			TOF8801_ERR(
				"Error starting factory calibration routine: %d",
				error);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return 0;
		}
		do {
			//spin here waiting for factory calibration to complete
			AMS_MUTEX_UNLOCK(&chip->lock);
			msleep(100);
			AMS_MUTEX_LOCK(&chip->lock);
			timeout_flag =
				((jiffies - start) >=
				 msecs_to_jiffies(APP0_FAC_CALIB_MSEC_TIMEOUT));
		} while (!timeout_flag &&
			 tof8801_app0_measure_in_progress(chip));
		AMS_MUTEX_UNLOCK(&chip->lock);
		msleep(500);
		if (chip->poll_period == 1) {
			int_stat = 0;
			int_stat = int_stat | IRQ_RESULTS;
			tof8801_app0_process_irq(chip, int_stat);
		}
		AMS_MUTEX_LOCK(&chip->lock);
		if (!tof8801_app0_measure_in_progress(chip) &&
		    chip->app0_app.cal_update.dataFactoryConfig) {
			// If calib measure complete and was successful
			if (chip->ext_calib_data.size) {
				memcpy(buf,
				       (void *)&chip->ext_calib_data.fac_data,
				       chip->ext_calib_data.size);
			}
			len = chip->ext_calib_data.size;
			buf[len] =
				0; //output is a string so we need to add null-terminating character
			TOF8801_INFO(
				"Done performing factory calibration, size: %u",
				len);
		} else {
			TOF8801_ERR(
				"Error timeout waiting on factory calibration");
			AMS_MUTEX_UNLOCK(&chip->lock);
			return 0;
		}
		AMS_MUTEX_UNLOCK(&chip->lock);
		orig_crosstalk = ((u32)buf[2] << 12) + ((u32)buf[1] << 4) +
				 ((u32)buf[0] >> 4);
		if (orig_crosstalk > 50000) {
			return 0;
		}
		return len;
	}
	return -EPERM;
}

static ssize_t app0_tof_output_read(struct file *fp, struct kobject *kobj,
				    struct bin_attribute *attr, char *buf,
				    loff_t off, size_t size)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int read;
	u32 elem_len;
	if (chip != NULL) {
		AMS_MUTEX_LOCK(&chip->lock);
		elem_len = kfifo_peek_len(&chip->tof_output_fifo);
		TOF8801_DBG("size: %u", (unsigned int)size);
		if (kfifo_len(&chip->tof_output_fifo)) {
			TOF8801_DBG("fifo read elem_len: %u", elem_len);
			read = kfifo_out(&chip->tof_output_fifo, buf, elem_len);
			TOF8801_DBG("fifo_len: %u",
				    kfifo_len(&chip->tof_output_fifo));
			AMS_MUTEX_UNLOCK(&chip->lock);
			return elem_len;
		} else {
			AMS_MUTEX_UNLOCK(&chip->lock);
			return 0;
		}
	}
	return -EPERM;
}

static ssize_t clk_trim_firmware_download_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	unsigned int len = 0;

	TOF8801_INFO("clock trim calibration value = 0x%02x",
		     clk_trim_firmware);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "clock_trim_calibration:0x%02x\n", clk_trim_firmware);

	return len;
}

static ssize_t clk_trim_firmware_download_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int error;
	if (chip != NULL) {
		TOF8801_INFO("0x%02x", *buf);
		memcpy((void *)(&clk_trim_firmware), buf,
		       sizeof(clk_trim_firmware));
		TOF8801_INFO("%s: 0x%02x", clk_trim_firmware);
		error = tof8801_rewrite_clock_trim_data(chip);
		if (error) {
			pr_err("%s error tof8801_rewrite_clock_trim_data: %d\n",
			       __func__, error);
			return error;
		}

		return count;
	}
	return -EPERM;
}

static ssize_t crosstalk_firmware_download_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	int i;
	int len = 0;
	char *tmpbuf;
	if (chip != NULL) {
		tmpbuf = (char *)&chip->ext_calib_data.fac_data;
		TOF8801_INFO("Enter");
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
			TOF8801_ERR("Error ToF chip app_id: %#x",
				    chip->info_rec.record.app_id);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EINVAL;
		}
		for (i = 0; i < chip->ext_calib_data.size; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "fac_calib[%d]:0x%02x\n", i,
					 tmpbuf[i]);
		AMS_MUTEX_UNLOCK(&chip->lock);
		return len;
	}
	return -EPERM;
}

static ssize_t crosstalk_firmware_download_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	int i;
	char *tmpbuf;
	struct tof_sensor_chip *chip = dev_get_drvdata(client_dev);
	if (chip != NULL) {
		for (i = 0; i < count; i++)
			TOF8801_INFO("buf [%d] = 0x%02x", i, buf[i]);
		memcpy(crosstalk_firmware, buf, TOF_GET_FAC_CALIB_BIN_SIZE);
		memcpy((void *)&chip->ext_calib_data.fac_data, buf,
		       TOF_GET_FAC_CALIB_BIN_SIZE);
		chip->ext_calib_data.size = TOF_GET_FAC_CALIB_BIN_SIZE;
		orig_crosstalk = ((u32)crosstalk_firmware[2] << 12) +
				 ((u32)crosstalk_firmware[1] << 4) +
				 ((u32)crosstalk_firmware[0] >> 4);
		tmpbuf = (char *)&chip->ext_calib_data.fac_data;
		for (i = 0; i < chip->ext_calib_data.size; i++)
			TOF8801_INFO("fac_calib[%d] = 0x%02x", i, tmpbuf[i]);
		return count;
	}
	return -EPERM;
}

/****************************************************************************
 * Common Sysfs Attributes
 * **************************************************************************/
/******* READ-WRITE attributes ******/
static DEVICE_ATTR_RW(program);
static DEVICE_ATTR_RW(chip_enable);
static DEVICE_ATTR_RW(driver_debug);
/******* READ-ONLY attributes ******/
static DEVICE_ATTR_RO(program_version);
static DEVICE_ATTR_RO(registers);
/******* WRITE-ONLY attributes ******/
static DEVICE_ATTR_WO(register_write);
static DEVICE_ATTR_WO(request_ram_patch);

/****************************************************************************
 * Bootloader Sysfs Attributes
 * **************************************************************************/
/******* READ-WRITE attributes ******/
/******* READ-ONLY attributes ******/
/******* WRITE-ONLY attributes ******/

/****************************************************************************
 * APP0 Sysfs Attributes
 * *************************************************************************/
/******* READ-WRITE attributes ******/
static DEVICE_ATTR_RW(app0_command);
static DEVICE_ATTR_RW(capture);
static DEVICE_ATTR_RW(period);
static DEVICE_ATTR_RW(noise_threshold);
static DEVICE_ATTR_RW(iterations);
static DEVICE_ATTR_RW(capture_delay);
static DEVICE_ATTR_RW(alg_setting);
static DEVICE_ATTR_RW(gpio_setting);
static DEVICE_ATTR_RW(app0_clk_iterations);
static DEVICE_ATTR_RW(app0_clk_trim_enable);
static DEVICE_ATTR_RW(app0_clk_trim_set);
static DEVICE_ATTR_RW(app0_apply_fac_calib);
static DEVICE_ATTR_RW(app0_apply_config_calib);
static DEVICE_ATTR_RW(app0_apply_state_data);
static DEVICE_ATTR_RW(clk_trim_firmware_download);
static DEVICE_ATTR_RW(crosstalk_firmware_download);
/******* READ-ONLY attributes ******/
static DEVICE_ATTR_RO(app0_general_configuration);
static DEVICE_ATTR_RO(app0_ctrl_reg);
static DEVICE_ATTR_RO(app0_temp);
static DEVICE_ATTR_RO(app0_diag_state_mask);
static DEVICE_ATTR_RO(app0_reflectivity_count);
static DEVICE_ATTR_RO(app0_get_fac_calib);
static DEVICE_ATTR_RO(app0_read_peak_crosstalk);
/******* WRITE-ONLY attributes ******/
/******* READ-ONLY BINARY attributes ******/
static BIN_ATTR_RO(app0_tof_output, 0);

static struct attribute *tof_common_attrs[] = {
	&dev_attr_program.attr,		  &dev_attr_chip_enable.attr,
	&dev_attr_driver_debug.attr,	  &dev_attr_program_version.attr,
	&dev_attr_registers.attr,	  &dev_attr_register_write.attr,
	&dev_attr_request_ram_patch.attr, NULL,
};
static struct attribute *tof_bl_attrs[] = {
	NULL,
};
static struct attribute *tof_app0_attrs[] = {
	&dev_attr_app0_command.attr,
	&dev_attr_capture.attr,
	&dev_attr_period.attr,
	&dev_attr_iterations.attr,
	&dev_attr_noise_threshold.attr,
	&dev_attr_capture_delay.attr,
	&dev_attr_alg_setting.attr,
	&dev_attr_gpio_setting.attr,
	&dev_attr_app0_clk_iterations.attr,
	&dev_attr_app0_clk_trim_enable.attr,
	&dev_attr_app0_clk_trim_set.attr,
	&dev_attr_app0_diag_state_mask.attr,
	&dev_attr_app0_general_configuration.attr,
	&dev_attr_app0_ctrl_reg.attr,
	&dev_attr_app0_temp.attr,
	&dev_attr_app0_reflectivity_count.attr,
	&dev_attr_app0_get_fac_calib.attr,
	&dev_attr_app0_apply_fac_calib.attr,
	&dev_attr_app0_apply_config_calib.attr,
	&dev_attr_app0_apply_state_data.attr,
	&dev_attr_app0_read_peak_crosstalk.attr,
	&dev_attr_clk_trim_firmware_download.attr,
	&dev_attr_crosstalk_firmware_download.attr,
	NULL,
};
static struct bin_attribute *tof_app0_bin_attrs[] = {
	&bin_attr_app0_tof_output,
	NULL,
};
static const struct attribute_group tof_common_group = {
	.attrs = tof_common_attrs,
};
static const struct attribute_group tof_bl_group = {
	.name = "bootloader",
	.attrs = tof_bl_attrs,
};
static const struct attribute_group tof_app0_group = {
	.name = "app0",
	.attrs = tof_app0_attrs,
	.bin_attrs = tof_app0_bin_attrs,
};
static const struct attribute_group *tof_groups[] = {
	&tof_common_group,
	&tof_bl_group,
	&tof_app0_group,
	NULL,
};

/**
 * tof_i2c_read - Read number of bytes starting at a specific address over I2C
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the received data
 * @len: number of bytes to read
 */
int tof_i2c_read(struct i2c_client *client, char reg, char *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr = client->addr;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len;
	msgs[1].buf = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/**
 * tof_i2c_write - Write nuber of bytes starting at a specific address over I2C
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the data to write
 * @len: number of bytes to write
 */
int tof_i2c_write(struct i2c_client *client, char reg, const char *buf, int len)
{
	u8 *addr_buf;
	struct i2c_msg msg;
	struct tof_sensor_chip *chip = i2c_get_clientdata(client);
	int idx = reg;
	int ret;
	char debug[120];
	u32 strsize = 0;

	addr_buf = kmalloc(len + 1, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = reg;
	memcpy(&addr_buf[1], buf, len);
	msg.flags = 0;
	msg.addr = client->addr;
	msg.buf = addr_buf;
	msg.len = len + 1;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		TOF8801_ERR("i2c_transfer failed: %d msg_len: %u", ret, len);
	}
	if (chip->driver_debug > 1) {
		strsize = scnprintf(debug, sizeof(debug), "i2c_write: ");
		for (idx = 0; (ret == 1) && (idx < msg.len); idx++) {
			strsize += scnprintf(debug + strsize,
					     sizeof(debug) - strsize, "%02x ",
					     addr_buf[idx]);
		}
		TOF8801_INFO("%s", debug);
	}

	kfree(addr_buf);
	return ret < 0 ? ret : (ret != 1 ? -EIO : 0);
}

/**
 * tof_i2c_write_mask - Write a byte to the specified address with a given bitmask
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @val: byte to write
 * @mask: bitmask to apply to address before writing
 */
int tof_i2c_write_mask(struct i2c_client *client, char reg, const char *val,
		       char mask)
{
	int ret;
	u8 temp;
	struct tof_sensor_chip *chip = i2c_get_clientdata(client);

	ret = tof_i2c_read(client, reg, &temp, 1);
	temp &= ~mask;
	temp |= *val;
	ret = tof_i2c_write(client, reg, &temp, 1);

	chip->shadow[(int)reg] = temp;

	return ret;
}

/**
 * tof8801_get_register - Return a specific register
 *
 * @chip: tof_sensor_chip pointer
 * @value: pointer to value in register
 */
int tof8801_get_register(struct i2c_client *client, char reg, char *value)
{
	return tof_i2c_read(client, reg, value, sizeof(char));
}

/**
 * tof8801_set_register - Set a specific register
 *
 * @chip: tof_sensor_chip pointer
 * @value: value to set in register
 */
int tof8801_set_register(struct i2c_client *client, char reg, const char value)
{
	return tof_i2c_write(client, reg, &value, sizeof(char));
}

/**
 * tof8801_set_register_mask - Set a specific register, with a mask
 *
 * @chip: tof_sensor_chip pointer
 * @value: value to set in register
 * @mask: mask to apply with register, i.e. value=0x1, mask=0x1 = only bit 0 set
 */
int tof8801_set_register_mask(struct i2c_client *client, char reg,
			      const char value, const char mask)
{
	return tof_i2c_write_mask(client, reg, &value, mask);
}

void tof_dump_i2c_regs(struct tof_sensor_chip *chip, char offset, char end)
{
	int per_line = 4;
	int len = 0;
	int idx, per_line_idx;
	char debug[80];

	offset &= ~(per_line - 1); // Byte boundary for nice printing
	while ((end & (per_line - 1)) != (per_line - 1))
		end += 1;
	end = (end < offset) ? (offset + per_line) : end;
	TOF8801_INFO("ENTER");
	(void)tof_i2c_read(chip->client, offset, &chip->shadow[(int)offset],
			   (end - offset));
	for (idx = offset; idx < end; idx += per_line) {
		memset(debug, 0, sizeof(debug));
		len += scnprintf(debug, sizeof(debug) - len, "%02x: ", idx);
		for (per_line_idx = 0; per_line_idx < per_line;
		     per_line_idx++) {
			len += scnprintf(debug + len, sizeof(debug) - len,
					 "%02x ",
					 chip->shadow[idx + per_line_idx]);
		}
		len = 0;
		TOF8801_INFO("%s", debug);
	}
}

/**
 * tof_standby_operation - Tell the ToF chip to wakeup/standby
 *
 * @client: the i2c client
 */
static int tof_standby_operation(struct i2c_client *client, char oper)
{
	return tof8801_set_register(client, TOF8801_STAT, oper);
}

/**
 * tof_CE_toggle - Hard reset the ToF by toggling the ChipEnable
 *
 * @client: the i2c client
 */
static int tof_CE_toggle(struct i2c_client *client)
{
	struct tof_sensor_chip *chip = i2c_get_clientdata(client);
	int error = 0;
	if (!chip->pdata->gpiod_enable) {
		//not supported in poll mode
		return -EIO;
	}
	error = gpiod_direction_output(chip->pdata->gpiod_enable, 0);
	if (error)
		return error;
	error = gpiod_direction_output(chip->pdata->gpiod_enable, 1);
	/* ToF requires 5ms to get i2c back up */
	usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);
	return error;
}

/**
 * tof_wait_for_cpu_ready_timeout - Check for CPU ready state in the ToF sensor
 *                                  for a specified period of time
 *
 * @client: the i2c client
 */
int tof_wait_for_cpu_ready_timeout(struct i2c_client *client,
				   unsigned long usec)
{
	int error = 0;
	unsigned long curr = jiffies;
	do {
		error = tof_wait_for_cpu_ready(client);
		if (error == 0) {
			return 0;
		}
	} while ((jiffies - curr) < usecs_to_jiffies(usec));
	TOF8801_ERR("Error timeout (%lu usec) waiting on cpu_ready: %d", usec,
		    error);
	return -EIO;
}

/**
 * tof_wait_for_cpu_ready - Check for CPU ready state in the ToF sensor
 *
 * @client: the i2c client
 */
int tof_wait_for_cpu_ready(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 status;

	//wait for i2c
	usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);
	while (retry++ < TOF8801_MAX_WAIT_RETRY) {
		error = tof8801_get_register(client, TOF8801_STAT, &status);
		if (error) {
			TOF8801_ERR("i2c test failed attempt %d: %d", retry,
				    error);
			continue;
		}
		if (TOF8801_STAT_CPU_READY(status)) {
			TOF8801_DBG("ToF chip CPU is ready");
			return 0;
		} else if (TOF8801_STAT_CPU_SLEEP(status)) {
			TOF8801_INFO("ToF chip in standby state, waking up");
			tof_standby_operation(client, WAKEUP);
			usleep_range(TOF8801_I2C_WAIT_USEC,
				     TOF8801_I2C_WAIT_USEC + 1);
			error = -EIO;
			continue;
		} else if (TOF8801_STAT_CPU_BUSY(status) &&
			   (retry >= TOF8801_MAX_WAIT_RETRY)) {
			return -EIO;
		}
		usleep_range(TOF8801_WAIT_UDELAY, 2 * TOF8801_WAIT_UDELAY);
	}
	return error;
}

/**
 * tof_wait_for_cpu_startup - Check for CPU ready state in the ToF sensor
 *
 * @client: the i2c client
 */
int tof_wait_for_cpu_startup(struct i2c_client *client)
{
	int retry = 0;
	int CE_retry = 0;
	int error;
	u8 status;

	while (retry++ < TOF8801_MAX_STARTUP_RETRY) {
		usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);
		error = tof8801_get_register(client, TOF8801_STAT, &status);
		if (error) {
			TOF8801_ERR("i2c test failed attempt %d: %d", retry,
				    error);
			continue;
		} else {
			TOF8801_DBG("CPU status register: %#04x value: %#04x",
				    TOF8801_STAT, status);
		}
		if (TOF8801_STAT_CPU_READY(status)) {
			TOF8801_INFO("ToF chip CPU is ready");
			return 0;
		} else if (TOF8801_STAT_CPU_SLEEP(status)) {
			TOF8801_INFO("ToF chip in standby state, waking up");
			tof_standby_operation(client, WAKEUP);
			error = -EIO;
			continue;
		} else if (TOF8801_STAT_CPU_BUSY(status) &&
			   (retry >= TOF8801_MAX_STARTUP_RETRY)) {
			if ((CE_retry < TOF8801_MAX_STARTUP_RETRY)) {
				TOF8801_INFO(
					"ToF chip still busy, try toggle CE");
				if (tof_CE_toggle(client)) {
					return -EIO;
				}
				retry = 0;
				CE_retry++;
			} else {
				return -EIO;
			}
		}
	}
	return error;
}

/**
 * tof_init_info_record - initialize info record of currently running app
 *
 * @client: the i2c client
 * @record: pointer to info_record struct where data will be placed
 */
int tof_init_info_record(struct tof_sensor_chip *chip)
{
	int error;

	error = tof_i2c_read(chip->client, TOF8801_APP_ID, chip->info_rec.data,
			     TOF8801_INFO_RECORD_SIZE);
	if (error) {
		TOF8801_ERR("read record failed: %d", error);
		goto err;
	}
	TOF8801_INFO("Read info record - Running app_id: %#x.",
		     chip->info_rec.record.app_id);
	/* re-initialize apps */
	if (chip->info_rec.record.app_id == TOF8801_APP_ID_BOOTLOADER) {
		tof8801_BL_init_app(&chip->BL_app);
	} else if (chip->info_rec.record.app_id == TOF8801_APP_ID_APP0) {
		tof8801_app0_init_app(&chip->app0_app);
	}
	return 0;
err:
	return error;
}

static int tof_switch_from_bootloader(struct tof_sensor_chip *chip,
				      char req_app_id)
{
	int error = 0;
	char *new_app_id;

	// Try to perform RAM download (if possible)
	error = tof8801_firmware_download(chip, 0);
	if (error != 0) {
		//This means either there is no firmware, or there was a failure
		error = tof8801_set_register(chip->client, TOF8801_REQ_APP_ID,
					     req_app_id);
		if (error) {
			TOF8801_ERR("Error setting REQ_APP_ID register.");
			error = -EIO;
		}
		error = tof_wait_for_cpu_ready_timeout(chip->client, 100000);
		if (error) {
			TOF8801_ERR("Error waiting for CPU ready flag.");
		}
		error = tof_init_info_record(chip);
		if (error) {
			TOF8801_ERR("Error reading info record.");
		}
	}
	new_app_id = &chip->info_rec.record.app_id;
	TOF8801_INFO("Running app_id: 0x%02x", *new_app_id);
	switch (*new_app_id) {
	case TOF8801_APP_ID_BOOTLOADER:
		TOF8801_ERR("Error: application switch failed.");
		break;
	case TOF8801_APP_ID_APP0:
		/* enable all ToF interrupts on sensor */
		tof8801_enable_interrupts(chip,
					  IRQ_RESULTS | IRQ_DIAG | IRQ_ERROR);
		break;
	case TOF8801_APP_ID_APP1:
		break;
	default:
		TOF8801_ERR("Error: Unrecognized application.");
		return -EINVAL;
	}
	return (*new_app_id == req_app_id) ? 0 : -1;
}

int tof_switch_apps(struct tof_sensor_chip *chip, char req_app_id)
{
	int error = 0;
	if (req_app_id == chip->info_rec.record.app_id)
		return 0;
	if ((req_app_id != TOF8801_APP_ID_BOOTLOADER) &&
	    (req_app_id != TOF8801_APP_ID_APP0) &&
	    (req_app_id != TOF8801_APP_ID_APP1))
		return -EINVAL;
	switch (chip->info_rec.record.app_id) {
	case TOF8801_APP_ID_BOOTLOADER:
		error = tof_switch_from_bootloader(chip, req_app_id);
		if (error) {
			/* Hard reset back to bootloader if error */
			gpiod_set_value(chip->pdata->gpiod_enable, 0);
			gpiod_set_value(chip->pdata->gpiod_enable, 1);
			error = tof_wait_for_cpu_startup(chip->client);
			if (error) {
				TOF8801_ERR("I2C communication failure: %d",
					    error);
				return error;
			}
			error = tof_init_info_record(chip);
			if (error) {
				TOF8801_ERR(
					"Read application info record failed.");
				return error;
			}
			return -EINVAL;
		}
		break;
	case TOF8801_APP_ID_APP0:
		error = tof8801_app0_switch_to_bootloader(chip);
		break;
	case TOF8801_APP_ID_APP1:
		return -EINVAL;
		break;
	}
	return error;
}

/**
 * tof_hard_reset - use GPIO Chip Enable to reset the device
 *
 * @tof_chip: tof_sensor_chip pointer
 */
int tof_hard_reset(struct tof_sensor_chip *chip)
{
	int error = 0;
	int in_app0 = (chip->info_rec.record.app_id == TOF8801_APP_ID_APP0);
	if (!chip->pdata->gpiod_enable) {
		return -EIO;
	}

	//Try to stop measurements cleanly
	if (in_app0 && (gpiod_get_value(chip->pdata->gpiod_enable) == 1)) {
		(void)tof8801_app0_capture(chip, 0);
	}
	/* toggle CE pin */
	gpiod_set_value(chip->pdata->gpiod_enable, 0);
	gpiod_set_value(chip->pdata->gpiod_enable, 1);

	error = tof_wait_for_cpu_startup(chip->client);
	if (error) {
		TOF8801_ERR("I2C communication failure: %d", error);
		return error;
	}
	error = tof_init_info_record(chip);
	if (error) {
		TOF8801_ERR("Read application info record failed.");
		return error;
	}
	// If we were in App0 before CE LOW, try to download/switch back to App0
	if (in_app0) {
		// Need to perform RAM download if CE is toggled
		error = tof_switch_apps(chip, (char)TOF8801_APP_ID_APP0);
		if (error) {
			return error;
		}
	}
	return error;
}

static void tof_set_regulator(struct tof_sensor_chip *tof_chip)
{
	int rc = 0;
	if (tof_chip == NULL)
		return;
	if (tof_chip->regulator_info.vdd_regulator != NULL) {
		rc = regulator_set_voltage(
			tof_chip->regulator_info.vdd_regulator,
			tof_chip->regulator_info.vdd_voltage,
			tof_chip->regulator_info.vdd_voltage);
			regulator_set_load(tof_chip->regulator_info.vdd_regulator, 105000);
		if (rc == 0) {
			TOF8801_INFO("Success set vdd regulator");
			rc = regulator_enable(
				tof_chip->regulator_info.vdd_regulator);
			if (rc == 0) {
				TOF8801_INFO("Success enable vdd regulator");
			} else {
				TOF8801_ERR("Failed to enable vdd regulator");
			}
		} else {
			TOF8801_ERR("Failed set vdd regulator");
		}
	}
	if (tof_chip->regulator_info.vio_regulator != NULL) {
		rc = regulator_set_voltage(
			tof_chip->regulator_info.vio_regulator,
			tof_chip->regulator_info.vio_voltage,
			tof_chip->regulator_info.vio_voltage);
		if (rc == 0) {
			TOF8801_INFO("Success set vio regulator");
			rc = regulator_enable(
				tof_chip->regulator_info.vio_regulator);
			if (rc == 0) {
				TOF8801_INFO("Success enable vio regulator");
			} else {
				TOF8801_ERR("Failed to enable vio regulator");
			}
		} else {
			TOF8801_ERR("Failed set vio regulator");
		}
	}
}

static void tof_put_regulator(struct tof_sensor_chip *tof_chip)
{
	int rc = 0;
	if (tof_chip == NULL)
		return;
	if (tof_chip->regulator_info.vdd_regulator != NULL) {
		rc = regulator_set_voltage(
			tof_chip->regulator_info.vdd_regulator,
			tof_chip->regulator_info.vdd_min_voltage,
			tof_chip->regulator_info.vdd_max_voltage);
		if (rc == 0) {
			TOF8801_INFO("Success set regulator");
		} else {
			TOF8801_ERR("Failed to set regulator");
		}
	}
	if (tof_chip->regulator_info.vio_regulator != NULL) {
		rc = regulator_set_voltage(
			tof_chip->regulator_info.vio_regulator,
			tof_chip->regulator_info.vio_min_voltage,
			tof_chip->regulator_info.vio_max_voltage);
		if (rc == 0) {
			TOF8801_INFO("Success set regulator");
		} else {
			TOF8801_ERR("Failed to set regulator");
		}
	}
}

/**
 * tof_get_gpio_config - Get GPIO config from DT
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof_get_gpio_config(struct tof_sensor_chip *tof_chip)
{
	int error;
	struct device *dev;
	struct gpio_desc *gpiod;
	struct device_node *of_node = NULL;
	struct device_node *src_node = NULL;
	uint32_t vdd_voltage, vio_voltage;
	uint32_t vdd_max_voltage, vio_max_voltage;
	uint32_t vdd_min_voltage, vio_min_voltage;

	if (!tof_chip->client)
		return -EINVAL;
	dev = &tof_chip->client->dev;

	of_node = dev->of_node;
	/* Get the enable line GPIO pin number */
	gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_ENABLE_NAME,
					GPIOD_OUT_HIGH);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		return error;
	}
	tof_chip->pdata->gpiod_enable = gpiod;

	/* Get the interrupt GPIO pin number */
	gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_INT_NAME, GPIOD_IN);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		return error;
	}
	tof_chip->pdata->gpiod_interrupt = gpiod;

	if (of_node != NULL) {
		src_node = of_parse_phandle(of_node, "tof8801_vdd-supply", 0);
		if (src_node != NULL) {
			of_property_read_u32(of_node, "tof8801_vdd-voltage",
					     &vdd_voltage);
			of_property_read_u32(of_node, "tof8801_vdd-min-voltage",
					     &vdd_min_voltage);
			of_property_read_u32(of_node, "tof8801_vdd-max-voltage",
					     &vdd_max_voltage);
			if (vdd_voltage > 0 && vdd_min_voltage >= 0 &&
			    vdd_max_voltage > 0) {
				tof_chip->regulator_info.vdd_regulator =
					regulator_get(dev, "tof8801_vdd");
				tof_chip->regulator_info.vdd_voltage =
					vdd_voltage;
				tof_chip->regulator_info.vdd_max_voltage =
					vdd_max_voltage;
				tof_chip->regulator_info.vdd_min_voltage =
					vdd_min_voltage;
				TOF8801_INFO(
					"regulator:min = %d max = %d vdd = %d",
					vdd_min_voltage, vdd_max_voltage,
					vdd_voltage);
			}
		}
		src_node = of_parse_phandle(of_node, "tof8801_vio-supply", 0);
		if (src_node != NULL) {
			of_property_read_u32(of_node, "tof8801_vio-voltage",
					     &vio_voltage);
			of_property_read_u32(of_node, "tof8801_vio-min-voltage",
					     &vio_min_voltage);
			of_property_read_u32(of_node, "tof8801_vio-max-voltage",
					     &vio_max_voltage);
			if (vio_voltage > 0 && vio_min_voltage >= 0 &&
			    vio_max_voltage > 0) {
				tof_chip->regulator_info.vio_regulator =
					regulator_get(dev, "tof8801_vio");
				tof_chip->regulator_info.vio_voltage =
					vio_voltage;
				tof_chip->regulator_info.vio_max_voltage =
					vio_max_voltage;
				tof_chip->regulator_info.vio_min_voltage =
					vio_min_voltage;
				TOF8801_INFO(
					"regulator:min = %d max = %d vio = %d",
					vio_min_voltage, vio_max_voltage,
					vio_voltage);
			}
		}
		tof_set_regulator(tof_chip);
	}
	/*
	gpiod = of_get_named_gpio(dev->of_node, "irq-gpio", 0);
	if (rc < 0) {
	  dev_err(&client->dev, "Couldn't get Interrupt GPIO.\n");
	  return error;
	}
	tof_chip->pdata->gpiod_interrupt = gpiod;
	*/
	return 0;
}

/**
 * tof_ram_patch_callback - The firmware download callback
 *
 * @cfg: the firmware cfg structure
 * @ctx: private data pointer to struct tof_sensor_chip
 */
static void tof_ram_patch_callback(const struct firmware *cfg, void *ctx)
{
	struct tof_sensor_chip *chip = ctx;
	const u8 *line;
	const u8 *line_end;
	int verify = 0;
	int result = 0;
	u32 patch_size = 0;
	u64 fwdl_time = 0;
	struct timespec start_ts = { 0 }, end_ts = { 0 };
	AMS_MUTEX_LOCK(&chip->lock);
	if (!chip) {
		TOF8801_ERR(
			"AMS-TOF Error: Ram patch callback NULL context pointer.");
	}

	if (!cfg) {
		TOF8801_ERR("Warning, firmware not available.");
		goto err_fwdl;
	}
	TOF8801_INFO("Ram patch in progress...");
	/* Assuming you can only perform ram download while in BL application */
	/* switch back to BL app to perform RAM download */
	if (chip->info_rec.record.app_id != TOF8801_APP_ID_BOOTLOADER) {
		TOF8801_INFO(
			"Current app_id: %hhx - Switching to bootloader for RAM download",
			chip->info_rec.record.app_id);
		result = tof_switch_apps(chip, (char)TOF8801_APP_ID_BOOTLOADER);
		if (result) {
			TOF8801_INFO("Error changing to bootloader app: \'%d\'",
				     result);
			goto err_fwdl;
		}
	}
	//Start fwdl timer
	getnstimeofday(&start_ts);
	/* setup encryption salt */
	result = tof8801_BL_upload_init(chip->client, &chip->BL_app,
					tof_salt_value);
	if (result) {
		TOF8801_INFO("Error setting upload salt: \'%d\'", result);
		goto err_fwdl;
	}
	//assume we have mutex already
	intelHexInterpreterInitialise();
	line = cfg->data;
	line_end = line;
	while ((line_end - cfg->data) < cfg->size) {
		line_end = strchrnul(line, '\n');
		patch_size +=
			((line_end - line) > INTEL_HEX_MIN_RECORD_SIZE) ?
				((line_end - line - INTEL_HEX_MIN_RECORD_SIZE) /
				 2) :
				0;
		result = intelHexHandleRecord(chip->client, &chip->BL_app,
					      line_end - line, line, verify);
		if (result) {
			TOF8801_ERR("Ram patch failed: %d", result);
			goto err_fwdl;
		}
		line = ++line_end;
	}
	//Stop fwdl timer
	getnstimeofday(&end_ts);
	fwdl_time =
		timespec_sub(end_ts, start_ts).tv_nsec / 1000000; //time in ms
	TOF8801_INFO("Ram patch complete, patch size: %uK, dl time: %llu ms\n",
		     ((patch_size >> 10) + 1), fwdl_time);
	//wait for i2c
	usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);
	/* resync our info record since we just switched apps */
	tof_init_info_record(chip);
err_fwdl:
	release_firmware(cfg);
	complete_all(&chip->ram_patch_in_progress);
	AMS_MUTEX_UNLOCK(&chip->lock);
}

int tof_queue_frame(struct tof_sensor_chip *chip, void *buf, int size)
{
	unsigned int fifo_len;
	unsigned int frame_size;
	int result = kfifo_in(&chip->tof_output_fifo, buf, size);
	if (result == 0) {
		if (chip->driver_debug == 1)
			TOF8801_ERR(
				"Error: Frame buffer is full, clearing buffer.");
		kfifo_reset(&chip->tof_output_fifo);
		tof8801_app0_report_error(chip, ERR_BUF_OVERFLOW, DEV_OK);
		result = kfifo_in(&chip->tof_output_fifo, buf, size);
		if (result == 0) {
			TOF8801_ERR("Error: queueing ToF output frame.");
		}
	}
	if (chip->driver_debug == 2) {
		fifo_len = kfifo_len(&chip->tof_output_fifo);
		frame_size = ((char *)buf)[DRV_FRAME_SIZE_LSB] |
			     (((char *)buf)[DRV_FRAME_SIZE_MSB] << 8);
		TOF8801_INFO("Add frame_id: 0x%x, data_size: %u",
			     ((char *)buf)[DRV_FRAME_ID], frame_size);
		TOF8801_INFO(
			"New fifo len: %u, fifo utilization: %u%%", fifo_len,
			(1000 * fifo_len / kfifo_size(&chip->tof_output_fifo)) /
				10);
	}
	return (result == size) ? 0 : -1;
}

/**
 * tof_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t tof_irq_handler(int irq, void *dev_id)
{
	struct tof_sensor_chip *tof_chip = (struct tof_sensor_chip *)dev_id;
	char int_stat = 0;
	char appid;
	int error;

	TOF8801_DBG("Enter IRQ Handler.");
	AMS_MUTEX_LOCK(&tof_chip->lock);
	//Go to appropriate IRQ handler depending on the app running
	appid = tof_chip->info_rec.record.app_id;
	switch (appid) {
	case TOF8801_APP_ID_BOOTLOADER:
		goto irq_handled;
	case TOF8801_APP_ID_APP0:
		(void)tof8801_get_register(tof_chip->client, TOF8801_INT_STAT,
					   &int_stat);
		if (tof_chip->driver_debug) {
			TOF8801_INFO("IRQ stat: %#x", int_stat);
		}
		if (int_stat != 0) {
			//Clear interrupt on ToF chip
			error = tof8801_set_register(
				tof_chip->client, TOF8801_INT_STAT, int_stat);
			if (error) {
				tof8801_app0_report_error(tof_chip, ERR_COMM,
							  DEV_OK);
			}
			tof8801_app0_process_irq(tof_chip, int_stat);
			/* Alert user space of changes */
			sysfs_notify(&tof_chip->client->dev.kobj,
				     tof_app0_group.name,
				     bin_attr_app0_tof_output.attr.name);
		}
		break;
	case TOF8801_APP_ID_APP1:
		goto irq_handled;
	}
irq_handled:
	AMS_MUTEX_UNLOCK(&tof_chip->lock);
	return IRQ_HANDLED;
}

int tof8801_app0_poll_irq_thread(void *tof_chip)
{
	struct tof_sensor_chip *chip = (struct tof_sensor_chip *)tof_chip;
	char meas_cmd = 0;
	int us_sleep = 0;
	// Poll period is interpreted in units of 5000 usec
	us_sleep = 5000;
	TOF8801_INFO("Starting ToF irq polling thread, period: %u us",
		     us_sleep);
	while (!kthread_should_stop()) {
		AMS_MUTEX_LOCK(&chip->lock);
		if (chip->app0_app.cap_settings.cmd == 0)
			tof8801_app0_capture(chip, 1);
		meas_cmd = chip->app0_app.cap_settings.cmd;
		AMS_MUTEX_UNLOCK(&chip->lock);
		if (meas_cmd) {
			(void)tof_irq_handler(0, tof_chip);
		}
		usleep_range(us_sleep, us_sleep + us_sleep / 10);
	}
	return 0;
}

/**
 * tof_request_irq - request IRQ for given gpio
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof_request_irq(struct tof_sensor_chip *tof_chip)
{
	int irq = tof_chip->client->irq;
	unsigned long default_trigger =
		irqd_get_trigger_type(irq_get_irq_data(irq));
	TOF8801_INFO("irq: %d, trigger_type: %lu", irq, default_trigger);
	return devm_request_threaded_irq(
		&tof_chip->client->dev, tof_chip->client->irq, NULL,
		tof_irq_handler, default_trigger | IRQF_SHARED | IRQF_ONESHOT,
		tof_chip->client->name, tof_chip);
}

/**
 * tof8801_get_all_regs - read all addressable I2C registers from device
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof8801_get_all_regs(struct tof_sensor_chip *tof_chip)
{
	int error, i = 0;

	memset(tof_chip->shadow, 0, MAX_REGS);
	error = tof_i2c_read(tof_chip->client, TOF8801_APP_ID, tof_chip->shadow,
			     MAX_REGS);
	if (error < 0) {
		TOF8801_ERR("Read all registers failed: %d", error);
		return error;
	}
	for (i = 0; i < MAX_REGS; i++) {
		TOF8801_INFO("Read addr = 0x%x02 data = 0x%02x", tof_chip->shadow[i]);
	}
	return 0;
}

/**
 * tof8801_enable_interrupts - enable specified interrutps
 *
 * @tof_chip: tof_sensor_chip pointer
 * @int_en_flags: OR'd flags of interrupts to enable
 */
static int tof8801_enable_interrupts(struct tof_sensor_chip *chip,
				     char int_en_flags)
{
	char flags;
	int error = tof8801_get_register(chip->client, TOF8801_INT_EN, &flags);
	flags &= TOF8801_INT_MASK;
	flags |= int_en_flags;
	if (error) {
		return error;
	}
	return tof8801_set_register(chip->client, TOF8801_INT_EN, flags);
}

static int tof8801_get_config_calib_data(struct tof_sensor_chip *chip)
{
	int error;
	const struct firmware *config_fw = NULL;
	/* Set current configuration calibration data size to 0*/
	chip->config_data.size = 0;
	///***** Check for available fac_calib to read *****/
	error = request_firmware_direct(&config_fw,
					chip->pdata->config_calib_data_fname,
					&chip->client->dev);
	if (error || !config_fw) {
		TOF8801_ERR(
			"configuration calibration data not available \'%s\': %d",
			chip->pdata->config_calib_data_fname, error);
		return 0;
	} else {
		TOF8801_INFO("Read in config_calib file: \'%s\'.",
			     chip->pdata->config_calib_data_fname);
	}
	if (config_fw->size > sizeof(chip->config_data.cfg_data)) {
		TOF8801_ERR(
			"Error: config calibration data size too large %zu > %zu (MAX)",
			config_fw->size, sizeof(chip->config_data.cfg_data));
		return 1;
	}
	memcpy((void *)&chip->config_data.cfg_data, config_fw->data,
	       config_fw->size);
	chip->config_data.size = config_fw->size;
	release_firmware(config_fw);
	return 0;
}

#if 0
static int tof8801_get_fac_calib_data(struct tof_sensor_chip *chip)
{
	int error;
	const struct firmware *calib_fw = NULL;
	/* Set current factory calibration data size to 0*/
	chip->ext_calib_data.size = 0;
	//Alg info is only valid with factory cal, so clear it as well
	chip->alg_info.size = 0;
	///***** Check for available fac_calib to read *****/
	error = request_firmware_direct(&calib_fw,
					chip->pdata->fac_calib_data_fname,
					&chip->client->dev);
	if (error || !calib_fw) {
		TOF8801_ERR("factory calibration data not available \'%s\': %d",
			    chip->pdata->fac_calib_data_fname, error);
		return 0;
	} else {
		TOF8801_INFO("Read in fac_calib file: \'%s\'.",
			     chip->pdata->fac_calib_data_fname);
	}
	if (calib_fw->size > sizeof(chip->ext_calib_data.fac_data)) {
		TOF8801_INFO(
			"Error: factory calibration data size too large %zu > %zu (MAX)",
			calib_fw->size, sizeof(chip->ext_calib_data.fac_data));
		return 1;
	}
	memcpy((void *)&chip->ext_calib_data.fac_data, calib_fw->data,
	       calib_fw->size);
	chip->ext_calib_data.size = calib_fw->size;
	release_firmware(calib_fw);
	return 0;
}
#endif
#if 0
static int tof8801_get_clock_trim_data(struct tof_sensor_chip *chip)
{
	int error;
	int trim = 0;
	const struct firmware *clock_trim_data = NULL;

	///***** Check for available fac_calib to read *****/
	error = request_firmware(&clock_trim_data,
				 chip->pdata->clock_trim_data_fname,
				 &chip->client->dev);
	if (error || !clock_trim_data) {
		TOF8801_ERR("factory calibration data not available \'%s\': %d",
			    chip->pdata->clock_trim_data_fname, error);
		return -EPERM;
	} else {
		TOF8801_INFO("Read in fac_calib file: \'%s\'.",
			     chip->pdata->clock_trim_data_fname);
	}
	if (clock_trim_data->size > sizeof(trim)) {
		TOF8801_INFO(
			"Error: factory calibration data size too large %zu > %lu (MAX)",
			clock_trim_data->size, sizeof(trim));
		return -EPERM;
	}
	memcpy(&trim, clock_trim_data->data, clock_trim_data->size);
	release_firmware(clock_trim_data);
	return trim;
}
#endif

static int tof8801_firmware_download(struct tof_sensor_chip *chip, int startup)
{
	int error;
	struct timespec start_ts = { 0 }, end_ts = { 0 };
	int mutex_locked = mutex_is_locked(&chip->lock);
	int file_idx = 0;
	getnstimeofday(&start_ts);
	/* Iterate through all Firmware(s) to find one that works. 'Works' here is
	 * defined as running APP0 after FWDL
	 */
	for (file_idx = 0; chip->pdata->ram_patch_fname[file_idx] != NULL;
	     file_idx++) {
		/*** reset completion event that FWDL is starting ***/
		reinit_completion(&chip->ram_patch_in_progress);
		if (mutex_locked) {
			AMS_MUTEX_UNLOCK(&chip->lock);
		}
		TOF8801_INFO("Trying firmware: \'%s\'...\n",
			     chip->pdata->ram_patch_fname[file_idx]);
		/***** Check for available firmware to load *****/
		error = request_firmware_nowait(
			THIS_MODULE, true,
			chip->pdata->ram_patch_fname[file_idx],
			&chip->client->dev, GFP_KERNEL, chip,
			tof_ram_patch_callback);
		if (error) {
			TOF8801_ERR("Firmware not available \'%s\': %d",
				    chip->pdata->ram_patch_fname[file_idx],
				    error);
		}
		if (!startup &&
		    !wait_for_completion_interruptible_timeout(
			    &chip->ram_patch_in_progress,
			    msecs_to_jiffies(TOF_FWDL_TIMEOUT_MSEC))) {
			TOF8801_ERR(
				"Timeout waiting for Ram Patch \'%s\' Complete",
				chip->pdata->ram_patch_fname[file_idx]);
		}
		if (mutex_locked) {
			AMS_MUTEX_LOCK(&chip->lock);
		}
		if (chip->info_rec.record.app_id == TOF8801_APP_ID_APP0) {
			// assume we are done if APP0 is running
			break;
		}
	}
	getnstimeofday(&end_ts);
	TOF8801_DBG("FWDL callback %lu ms to finish",
		    (timespec_sub(end_ts, start_ts).tv_nsec / 1000000));
	// error if App0 is not running (fwdl failed)
	return (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) ? -EIO : 0;
}

static int tof_input_dev_open(struct input_dev *dev)
{
	struct tof_sensor_chip *chip = input_get_drvdata(dev);
	int error = 0;
	TOF8801_INFO("ENTER");
	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->pdata->gpiod_enable &&
	    (gpiod_get_value(chip->pdata->gpiod_enable) == 0)) {
		/* enable the chip */
		error = gpiod_direction_output(chip->pdata->gpiod_enable, 1);
		if (error) {
			TOF8801_ERR("Chip enable failed.");
			AMS_MUTEX_UNLOCK(&chip->lock);
			return -EIO;
		} else {
			TOF8801_INFO("enable laser success");
		}
	}
	AMS_MUTEX_UNLOCK(&chip->lock);
	if (chip->poll_period == 1 && chip->app0_poll_irq == NULL) {
		chip->app0_poll_irq = kthread_run(tof8801_app0_poll_irq_thread,
						  (void *)chip, "tof-irq_poll");
		if (IS_ERR(chip->app0_poll_irq))
			TOF8801_ERR("Error starting IRQ polling thread.");
	}
	return error;
}

static void tof_input_dev_close(struct input_dev *dev)
{
	struct tof_sensor_chip *chip = input_get_drvdata(dev);
	int error = 0;
	TOF8801_INFO("ENTER");
	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->pdata->gpiod_enable &&
	    (gpiod_get_value(chip->pdata->gpiod_enable) == 1)) {
		/* enable the chip */
		error = gpiod_direction_output(chip->pdata->gpiod_enable, 0);
		if (error) {
			TOF8801_ERR("Chip disable failed.");
		} else {
			TOF8801_INFO("disable laser success");
		}
	}
	AMS_MUTEX_UNLOCK(&chip->lock);
	if (chip->app0_poll_irq != NULL) {
		kthread_stop(chip->app0_poll_irq);
		chip->app0_poll_irq = NULL;
	}
}

static int tof_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	struct tof_sensor_chip *tof_chip;
	int error = 0;

	TOF8801_INFO("I2C Address: %02x", client->addr);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TOF8801_ERR("I2C check functionality failed.");
		return -ENXIO;
	}

	tof_chip = devm_kzalloc(&client->dev, sizeof(*tof_chip), GFP_KERNEL);
	if (!tof_chip)
		return -ENOMEM;

	/***** Setup data structures *****/
	mutex_init(&tof_chip->lock);
	client->dev.platform_data = (void *)&tof_pdata;
	tof_chip->client = client;
	tof_chip->pdata = &tof_pdata;
	i2c_set_clientdata(client, tof_chip);
	client_dev = &client->dev;
	/***** Firmware sync structure initialization*****/
	init_completion(&tof_chip->ram_patch_in_progress);
	//initialize kfifo for frame output
	INIT_KFIFO(tof_chip->tof_output_fifo);
	//Setup measure timer
	timer_setup(&tof_chip->meas_timer,
		    tof8801_app0_measure_timer_expiry_callback, 0);
	//Setup input device
	tof_chip->obj_input_dev = devm_input_allocate_device(&client->dev);
	if (tof_chip->obj_input_dev == NULL) {
		TOF8801_ERR("Error allocating input_dev.");
		goto input_dev_alloc_err;
	}
	of_property_read_u32(tof_chip->client->dev.of_node,
			     TOF_PROP_NAME_POLLIO, &(tof_chip->poll_period));
	if (tof_chip->poll_period != 1)
		tof_chip->poll_period = 0;
	tof_chip->obj_input_dev->name = tof_chip->pdata->tof_name;
	tof_chip->obj_input_dev->id.bustype = BUS_I2C;
	input_set_drvdata(tof_chip->obj_input_dev, tof_chip);
	tof_chip->obj_input_dev->open = tof_input_dev_open;
	if (tof_chip->poll_period == 1)
		tof_chip->obj_input_dev->close = tof_input_dev_close;
	set_bit(EV_ABS, tof_chip->obj_input_dev->evbit);
	input_set_abs_params(tof_chip->obj_input_dev, ABS_DISTANCE, 0, 0xFF, 0,
			     0);
	error = input_register_device(tof_chip->obj_input_dev);
	if (error) {
		TOF8801_ERR("Error registering input_dev.");
		goto input_reg_err;
	}

	error = tof_get_gpio_config(tof_chip);
	if (error)
		goto gpio_err;

	/***** Set ChipEnable HIGH *****/
	if (tof_chip->pdata->gpiod_enable) {
		/* enable the chip */
		error = gpiod_direction_output(tof_chip->pdata->gpiod_enable,
					       1);
		if (error) {
			TOF8801_ERR("Chip enable failed.");
			tof_put_regulator(tof_chip);
			goto gpio_err;
		}
	}
	TOF8801_INFO("choose type:%s",
		     tof_chip->poll_period == 0 ? "interrupt" : "poll");
	if (tof_chip->poll_period == 0) {
		/*** Use Interrupt I/O instead of polled ***/
		/***** Setup GPIO IRQ handler *****/
		if (tof_chip->pdata->gpiod_interrupt) {
			error = tof_request_irq(tof_chip);
			if (error) {
				TOF8801_ERR("Interrupt request Failed.");
				goto gen_err;
			}
		}
	}

	/***** Wait until ToF is ready for commands *****/
	error = tof_wait_for_cpu_startup(client);
	if (error) {
		TOF8801_ERR("I2C communication failure: %d", error);
		goto gen_err;
	}

	tof_chip->saved_clk_trim = UNINITIALIZED_CLK_TRIM_VAL;
	//read external (manufacturer) configuration data
	tof8801_app0_default_cap_settings(&tof_chip->app0_app);
	tof_chip->tof8801_app0_freq_ratio_loth_Q15 = 6903;
	tof_chip->tof8801_app0_freq_ratio_hith_Q15 = 7041;

	error = tof_init_info_record(tof_chip);
	if (error) {
		TOF8801_ERR("Read application info record failed.");
		goto gen_err;
	}

	/* enable all ToF interrupts on sensor */
	tof8801_enable_interrupts(tof_chip, IRQ_RESULTS | IRQ_DIAG | IRQ_ERROR);
	tof8801_fw_init(tof_chip);
	TOF8801_INFO("Probe ok.\n");
	return 0;

	/***** Failure case(s), unwind and return error *****/
gen_err:
gpio_err:
	if (tof_chip->pdata->gpiod_enable)
		(void)gpiod_direction_output(tof_chip->pdata->gpiod_enable, 0);
input_dev_alloc_err:
input_reg_err:
	i2c_set_clientdata(client, NULL);
	TOF8801_ERR("Probe failed.");
	return error;
}

static int tof_suspend(struct device *dev)
{
	return 0;
}

static int tof_resume(struct device *dev)
{
	return 0;
}

static int tof_remove(struct i2c_client *client)
{
	struct tof_sensor_chip *chip = i2c_get_clientdata(client);
	char int_stat = 0;
	if (chip->info_rec.record.app_id == TOF8801_APP_ID_APP0) {
		//Stop any current measurements
		tof8801_app0_capture(chip, 0);
		(void)tof8801_get_register(chip->client, TOF8801_INT_STAT,
					   &int_stat);
		if (int_stat != 0) {
			//Clear any interrupt status
			(void)tof8801_set_register(chip->client,
						   TOF8801_INT_STAT, int_stat);
		}
	}
	if (chip->pdata->gpiod_interrupt) {
		devm_free_irq(&client->dev, client->irq, chip);
		devm_gpiod_put(&client->dev, chip->pdata->gpiod_interrupt);
	}
	if (chip->poll_period != 0) {
		(void)kthread_stop(chip->app0_poll_irq);
	}
	if (chip->pdata->gpiod_enable) {
		/* disable the chip */
		(void)gpiod_direction_output(chip->pdata->gpiod_enable, 0);
		devm_gpiod_put(&client->dev, chip->pdata->gpiod_enable);
	}
	del_timer_sync(&chip->meas_timer); /*delete measure timer*/
	TOF8801_INFO("Leave");
	i2c_set_clientdata(client, NULL);
	return 0;
}

static struct i2c_device_id tof_idtable[] = { { "tof8801", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, tof_idtable);

static const struct dev_pm_ops tof_pm_ops = {
	.suspend = tof_suspend,
	.resume = tof_resume,
};

static const struct of_device_id tof_of_match[] = { { .compatible =
							      "ams,tof8801" },
						    {} };
MODULE_DEVICE_TABLE(of, tof_of_match);

static struct i2c_driver tof_driver = {
	.driver = {
		.name = "ams-tof",
		.pm = &tof_pm_ops,
		.of_match_table = of_match_ptr(tof_of_match),
	},
	.id_table = tof_idtable,
	.probe = tof_probe,
	.remove = tof_remove,
};

static int tof8801_probe(struct platform_device *pdev)
{
	int error = 0;
	if (i2c_add_driver(&tof_driver)) {
		TOF8801_ERR("Failed to add i2c driver.");
		error = -1;
		return error;
	}

	error = sysfs_create_groups(&pdev->dev.kobj, tof_groups);
	if (error) {
		TOF8801_ERR("Error creating sysfs attribute group.");
		goto sysfs_err;
	}
	return 0;

sysfs_err:
	sysfs_remove_groups(&pdev->dev.kobj, tof_groups);
	TOF8801_ERR("sysfs_remove_groups failed.");
	return 0;
}

static int tof8801_remove(struct platform_device *pdev)
{
	i2c_del_driver(&tof_driver);
	return 0;
}

static const struct of_device_id tof8801_of_match[] = {
	{
		.compatible = "qcom,tof8801-ams",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tof8801_of_match);

static struct platform_driver tof8801_platform_driver = {
	.probe = tof8801_probe,
	.remove = tof8801_remove,
	.driver = {
		.name = TOF8801_AMS_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tof8801_of_match,
	},
};

static int __init tof8801_ams_init(void)
{
	int ret;
	TOF8801_INFO("Init start.");
	ret = platform_driver_register(&tof8801_platform_driver);
	if (ret) {
		TOF8801_INFO("Failed to register platform driver\n");
		return ret;
	}
	return 0;
}

static void __exit tof8801_ams_exit(void)
{
	platform_driver_unregister(&tof8801_platform_driver);
}

module_init(tof8801_ams_init);
module_exit(tof8801_ams_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AMS-TAOS tmf8801 ToF sensor driver");
MODULE_VERSION("3.11");
