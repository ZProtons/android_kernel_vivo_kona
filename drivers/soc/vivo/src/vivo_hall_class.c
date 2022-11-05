/*
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: vivo sensor team
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "vivo_hall: " fmt

#include <linux/init.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include "../inc/vivo_hall_class.h"

static struct class *vivo_hall_class;

static atomic_t device_count;
static atomic_t rising_hall_count;
static atomic_t rotate_hall_count;

static struct attribute *vivo_hall_attrs[] = {
	NULL,
};
ATTRIBUTE_GROUPS(vivo_hall);

ssize_t device_count_show(struct class *class, struct class_attribute *attr, char *buf)
{

	pr_err("[vivo_hall] %s enter\n");

	return snprintf(buf, 20, "%d\n", atomic_read(&device_count));

}

ssize_t rising_hall_count_show(struct class *class, struct class_attribute *attr, char *buf)
{
	pr_err("[vivo_hall] %s enter\n");

	return snprintf(buf, 20, "%d\n", atomic_read(&rising_hall_count));

}

ssize_t rotate_hall_count_show(struct class *class, struct class_attribute *attr, char *buf)
{

	pr_err("[vivo_hall] %s enter\n");

	return snprintf(buf, 20, "%d\n", atomic_read(&rotate_hall_count));

}


static CLASS_ATTR_RO(device_count);
static CLASS_ATTR_RO(rising_hall_count);
static CLASS_ATTR_RO(rotate_hall_count);

static int create_vivo_hall_class(void)
{
	int ret = 0;
	
	if (!vivo_hall_class) {
		vivo_hall_class = class_create(THIS_MODULE, "vivo_hall");
		if (IS_ERR(vivo_hall_class))
			return PTR_ERR(vivo_hall_class);
		atomic_set(&device_count, 0);
		vivo_hall_class->dev_groups = vivo_hall_groups;

		ret = class_create_file(vivo_hall_class, &class_attr_device_count);
		if (ret) {
			pr_err("[vivo_hall] create vivo hall class failed, ret=%d\n", ret);
			return ret;
		}

		ret = class_create_file(vivo_hall_class, &class_attr_rising_hall_count);
		if (ret) {
			pr_err("[vivo_hall] create vivo hall rising class failed, ret=%d\n", ret);
			return ret;
		}

		ret = class_create_file(vivo_hall_class, &class_attr_rotate_hall_count);
		if (ret) {
			pr_err("[vivo_hall] create vivo rotate hall class failed, ret=%d\n", ret);
			return ret;
		}
	}
	return 0;
}

int vivo_hall_dev_register(struct vivo_hall_dev *tdev)
{
	int ret;
	int tmp_val;

	if (!tdev || !tdev->name)
		return -EINVAL;

	ret = create_vivo_hall_class();
	if (ret < 0)
		return ret;
	if (tdev->hall_type == RISING_HALL) {
		tmp_val = atomic_inc_return(&rising_hall_count);
		pr_err("[rising hall] the hall num now is %d\n", tmp_val);
	} else if (tdev->hall_type == ROTATE_HALL) {
		tmp_val = atomic_inc_return(&rotate_hall_count);
		pr_err("[rotate hall] the hall num now is %d\n", tmp_val);
	}
	tdev->index = atomic_inc_return(&device_count);
	tdev->dev = device_create(vivo_hall_class, NULL,
		MKDEV(0, tdev->index), NULL, "%s", tdev->name);
	if (IS_ERR(tdev->dev))
		return PTR_ERR(tdev->dev);

	dev_set_drvdata(tdev->dev, tdev);
	tdev->state = 0;
	return 0;
}
EXPORT_SYMBOL_GPL(vivo_hall_dev_register);

void vivo_hall_dev_unregister(struct vivo_hall_dev *tdev)
{
	device_destroy(vivo_hall_class, MKDEV(0, tdev->index));
}
EXPORT_SYMBOL_GPL(vivo_hall_dev_unregister);

static int __init vivo_hall_init(void)
{
	return create_vivo_hall_class();
}
device_initcall(vivo_hall_init);