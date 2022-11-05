#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include "vivo_force.h"

struct force_driver_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count);
};

static ssize_t force_null_store(struct kobject *kobj,
							struct kobj_attribute *attr,  const char *buf, size_t count)
{
		return -EINVAL;
}
static ssize_t force_null_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
		return -EINVAL;
}

static ssize_t force_log_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	if (vForceGetData() == NULL) {
		return snprintf(buf, 255, "vForceGetData is NULL\n");
	}
	return snprintf(buf, 255, "logSwitch = %d\n", vForceGetData()->logSwitch);
}
static ssize_t force_log_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val;

	if (vForceGetData() == NULL) {
		VFI("vForceGetData is NULL");
		return count;
	}

	if (sscanf(buf, "%d", &val) != 1) {
		VFI("invalide number of parameters passed");
		return -EINVAL;
	}

	vForceGetData()->logSwitch = val;
	VFI("logSwitch is %d", vForceGetData()->logSwitch);

	return count;
}
static ssize_t force_version_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	struct VForce *vfd = vForceGetData();
	unsigned char *fw_ver;
	
	if (vfd == NULL) {
		VFI("vForceGetData is NULL");
		return snprintf(buf, 255, "vForceGetData is NULL\n");
	}
	fw_ver = getChipFwOrCfgVer();
	if (fw_ver)
		return snprintf(buf, 255, "FWV:%2x%2x%2x%2x VFV:0x%x\n", fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3], VIVO_FORCE_VERSION);

	return ret;
}

static ssize_t force_fw_update_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	struct VForce *vfd = vForceGetData();
	
	if (vfd == NULL) {
		VFI("vForceGetData is NULL");
		return snprintf(buf, 255, "vForceGetData is NULL\n");
	}
	if (!vForceGetData()->updateFirmware) {
		VFI("chip's update fw function not define.");
		return snprintf(buf, 255, "updateFirmware is NULL\n");
	}
	vForceGetData()->fwUpdatingFlag = 1;
	ret = vForceGetData()->updateFirmware();
	if (ret < 0) {
		VFI("updateFirmware fail, please try again.");
		return snprintf(buf, 255, "updateFirmware fail\n");
	} else {
		VFI("updateFirmware success.");
		return snprintf(buf, 255, "updateFirmware success\n");
	}
	vForceGetData()->fwUpdatingFlag = 0;

	return ret;
}

static ssize_t force_rawdata_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	VFI("get sensor rawdata.");

//	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
//		return snprintf(buf, 255, "not in ts normal mode, could not read.\n");
//	}

	if (!vForceGetData()->getRawOrDiffData) {
		return snprintf(buf, 255, "getRawOrDiffData not define in chip driver.\n");
	}

	mutex_lock(&(vForceGetData()->rawDifMutex));
	ret = vForceGetData()->getRawOrDiffData(VIVO_FORCE_RAW_DATA, buf);
	mutex_unlock(&(vForceGetData()->rawDifMutex));
	if (ret < 0) {
		VFI("get rawdata fail.");
		return ret;
	}

	return ret;
}
static ssize_t force_difdata_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	VFI("get sensor difdata.");

//	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
//		return snprintf(buf, 255, "not in ts normal mode, could not read.\n");
//	}

	if (!vForceGetData()->getRawOrDiffData) {
		return snprintf(buf, 255, "getRawOrDiffData not define in chip driver.\n");
	}

	mutex_lock(&(vForceGetData()->rawDifMutex));
	ret = vForceGetData()->getRawOrDiffData(VIVO_FORCE_DIF_DATA, buf);
	mutex_unlock(&(vForceGetData()->rawDifMutex));
	if (ret < 0) {
		VFI("get rawdata fail.");
		return ret;
	}

	return ret;
}

static ssize_t force_keyInt_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	VFI("get key switch state.");
	if (vForceGetData() == NULL) {
		VFI("vForceGetData is NULL");
		return ret;
	}
	if (!vForceGetData()->getKeySwitchState) {
		return snprintf(buf, 255, "getKeySwitchState not define in chip driver.\n");
	}

	ret = vForceGetData()->getKeySwitchState();
	if (ret < 0) {
		VFI("get key switch state fail.");
		return ret;
	}
	return snprintf(buf, 255, "KeySwitchState = %d, 1 is on, 0 is off\n", ret);

	return ret;
}
static ssize_t force_keyInt_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val;
	int ret = 0;

	if (vForceGetData() == NULL) {
		VFI("vForceGetData is NULL");
		return count;
	}
	if (!vForceGetData()->keyIntSwitch) {
		VFI("keyIntSwitch not define in chip driver.");
		return count;
	}

	if (sscanf(buf, "%d", &val) != 1) {
		VFI("invalide number of parameters passed");
		return -EINVAL;
	}	

	ret = vForceGetData()->keyIntSwitch(val);
	if (ret < 0) {
		VFI("change key int switch fail.");
		return count;
	}

	return count;
}

static ssize_t force_get_threshold_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	VFI("get key threshold.");
	if (vForceGetData() == NULL) {
		VFI("vForceGetData is NULL");
		return ret;
	}
	if (!vForceGetData()->getKeyThreshold) {
		return snprintf(buf, 255, "getKeyThreshold not define in chip driver.\n");
	}

	ret = vForceGetData()->getKeyThreshold((unsigned char *)buf);
	if (ret < 0) {
		VFI("get key threshold fail.");
		return ret;
	}
	ret += snprintf(buf + strlen(buf), 255, "\n");
	return ret;
}

static ssize_t force_change_threshold_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int which, downup, threshold;
	int ret = 0;

	if (vForceGetData() == NULL) {
		VFI("vForceGetData is NULL");
		return count;
	}
	if (!vForceGetData()->changeKeyThreshold) {
		VFI("changeKeyThreshold not define in chip driver.");
		return count;
	}

	if (sscanf(buf, "%d %d %d", &which, &downup, &threshold) != 3) {
		VFI("invalide number of parameters passed");
		return -EINVAL;
	}

	ret = vForceGetData()->changeKeyThreshold(which, downup, threshold);
	if (ret < 0) {
		VFI("change key threshold fail.");
		return count;
	}

	return count;
}

static ssize_t force_cali_coef_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	VFI("get calibration coefficients.");
	if (vForceGetData() == NULL) {
		VFI("vForceGetData is NULL");
		return ret;
	}
	if (!vForceGetData()->getCaliCoef) {
		return snprintf(buf, 255, "getCaliCoef not define in chip driver.\n");
	}

	ret = vForceGetData()->getCaliCoef((unsigned char *)buf);
	if (ret < 0) {
		VFI("get calibration coefficients fail.");
		return ret;
	}
	ret += snprintf(buf + strlen(buf), 255, "\n");
	return ret;
}
static ssize_t force_cali_coef_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int which, coef;
	int ret = 0;

	if (vForceGetData() == NULL) {
		VFI("vForceGetData is NULL");
		return count;
	}
	if (!vForceGetData()->changeCaliCoef) {
		VFI("changeCaliCoef not define in chip driver.");
		return count;
	}

	if (sscanf(buf, "%d %d", &which, &coef) != 2) {
		VFI("invalide number of parameters passed");
		return -EINVAL;
	}

	ret = vForceGetData()->changeCaliCoef(which, coef);
	if (ret < 0) {
		VFI("change key calibration coefficients fail.");
		return count;
	}

	return count;
}
static ssize_t force_service_state_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{

	int i = 0;

	if (vForceGetData() == NULL) {
		VFI("vForceGetData is NULL");
		return count;
	}

	VFI("app name is %s", buf);
	memset(vForceGetData()->service_buf, '\0', 256);
	for (i = 0; i < count; i++) {
		vForceGetData()->service_buf[i] = (unsigned char)buf[i];
	}
	if (vForceGetData()->processByPackage) {
		vForceGetData()->processByPackage(vForceGetData()->service_buf);
	}

	return count;
}

static struct force_driver_sysfs_entry force_sensor_delta =
	__ATTR(dif, 0644, force_difdata_show, force_null_store);
static struct force_driver_sysfs_entry force_sensor_rawdata =
	__ATTR(raw, 0644, force_rawdata_show, force_null_store);
static struct force_driver_sysfs_entry force_firmware_update =
	__ATTR(fwu, 0644, force_fw_update_show, force_null_store);
static struct force_driver_sysfs_entry force_firmware_version =
	__ATTR(fwver, 0644, force_version_show, force_null_store);
static struct force_driver_sysfs_entry force_ts_log_switch =
	__ATTR(log_switch, 0644, force_log_switch_show, force_log_switch_store);
static struct force_driver_sysfs_entry force_key_int_switch =
	__ATTR(key_int_switch, 0644, force_keyInt_switch_show, force_keyInt_switch_store);
static struct force_driver_sysfs_entry force_key_threshold =
	__ATTR(key_threshold, 0644, force_get_threshold_show, force_change_threshold_store);
static struct force_driver_sysfs_entry force_cali_coef =
	__ATTR(cali_coef, 0644, force_cali_coef_show, force_cali_coef_store);
static struct force_driver_sysfs_entry force_service_state =
	__ATTR(service_state, 0644, force_null_show, force_service_state_store);
	
	
static struct attribute *force_node_sys_attrs[] = {
	&force_ts_log_switch.attr,
	&force_sensor_rawdata.attr,
	&force_sensor_delta.attr,
	
	&force_firmware_version.attr,
	&force_firmware_update.attr,
	&force_key_int_switch.attr,
	&force_key_threshold.attr,
	&force_cali_coef.attr,
    &force_service_state.attr,
	NULL
};
static ssize_t force_debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}
static ssize_t force_debug_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void force_debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}
static const struct sysfs_ops force_debug_object_sysfs_ops = {
	.show = force_debug_object_show,
	.store = force_debug_object_store,
};
static struct kobj_type force_debug_object_type = {
	.sysfs_ops	= &force_debug_object_sysfs_ops,
	.release	= force_debug_object_release,
	.default_attrs = force_node_sys_attrs,
};

int vForceNodeCreate(struct VForce *vfd)
{
	int ret = 0;
	
	VFI("create vivo force node:sys/vforce/...");
	ret = kobject_init_and_add(&vfd->kobjectDebug, &force_debug_object_type,
					NULL, "vforce");
	if (ret) {
		VFI("create force node error!");
		ret = -1;
		return ret;
	}
	
	return 0;
}

void vForceNodeRemove(struct VForce *vfd)
{
	VFI("remove vivo force node:sys/vforce/...");
	kobject_del(&vfd->kobjectDebug);
	return;
}