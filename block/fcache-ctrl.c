
#define pr_fmt(fmt) "FCACHE: " fmt

#include <linux/mm.h>
#include <linux/seqlock.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/namei.h>
#include <linux/freezer.h>
#include <linux/uaccess.h>

extern unsigned int is_atboot;

unsigned long android_data_ino = 0;
unsigned long data_app_ino = 0;
unsigned long data_data_ino = 0;

unsigned long system_lib_ino = 0;
unsigned long system_lib64_ino = 0;
unsigned long system_apex_ino = 0;
unsigned long system_framework_ino = 0;
unsigned long system_fonts_ino = 0;

unsigned long SystemUI_ino = 0;
unsigned long Launcher_ino = 0;
unsigned long WebView_ino = 0;
unsigned long TrichromeLib_ino = 0;
unsigned long QtiSystemSer_ino = 0;

unsigned long vendor_lib_ino = 0;
unsigned long vendor_lib64_ino = 0;

unsigned long top_android_data_ino = 0;
unsigned long top_data_app_ino = 0;
unsigned long top_data_data_ino = 0;

unsigned long vital_android_data_ino = 0;
unsigned long vital_data_app_ino = 0;
unsigned long vital_data_data_ino = 0;

unsigned long InputDevices_ino = 0;
unsigned long InProcessNetworkStack_ino = 0;
unsigned long PermissionController_ino = 0;
unsigned long SettingsProvider_ino = 0;

unsigned long system_pro_lib_ino = 0;
unsigned long system_pro_lib64_ino = 0;
unsigned long system_pro_etc_ino = 0;
unsigned long system_pro_overlay_ino = 0;

#define TOTAL_RAM	(SZ_1G + SZ_2G + SZ_4G)
#define MAX_INODE	10000

unsigned long min_file = ((600 * SZ_1M) >> PAGE_SHIFT);
unsigned long nr_low = 0;

long max_vital = ((300 * SZ_1M) >> PAGE_SHIFT);
long max_staple = ((50 * SZ_1M) >> PAGE_SHIFT);

int mm_kswap = 10;

int fctrl_en = 0;

static int manual = 1;

#define MAX_LEN	128

static char top_app[MAX_LEN] = "NOP";
static char top_app_ver[MAX_LEN] = {};

static char vital_app[MAX_LEN] = "NOP";
static char vital_app_ver[MAX_LEN] = {};

static DEFINE_MUTEX(top_mutex);
static DEFINE_MUTEX(vital_mutex);

static bool is_dir_exist(const char *dirname, unsigned long *ino)
{
	bool ret = false;

	if ((!dirname) || (!dirname[0])) {
	} else {
		struct path file_path;
		int error;

		error = kern_path(dirname, 0, &file_path);
		if (!error) {
			if (S_ISDIR(file_path.dentry->d_inode->i_mode)) {
				*ino = file_path.dentry->d_inode->i_ino;
				ret = true;
				pr_info("%s: %s ino=%lu\n", __func__, dirname, *ino);
			}

			path_put(&file_path);
		}
	}

	return ret;
}

#if 0
static bool is_file_exist(const char *dirname, unsigned long *ino)
{
	bool ret = false;

	if ((!dirname) || (!dirname[0])) {
	} else {
		struct path file_path;
		int error;

		error = kern_path(dirname, 0, &file_path);
		if (!error) {
			if (S_ISREG(file_path.dentry->d_inode->i_mode)) {
				*ino = file_path.dentry->d_inode->i_ino;
				ret = true;
				pr_info("%s: %s ino=%lu\n", __func__, dirname, *ino);
			}

			path_put(&file_path);
		}
	}

	return ret;
}
#endif

static int top_data_app_dir(struct dir_context *ctx, const char *name,
		int namelen, loff_t offset, u64 ino, unsigned int d_type)
{
	char version[MAX_LEN] = {};

	strcat(version, top_app);
	strcat(version, "-");

	if (strstr(name, version)) {
		memset(top_app_ver, 0, MAX_LEN);
		strcat(top_app_ver, name);
		top_data_app_ino = ino;
		pr_info("%s: %.*s, ino=%lu, %s\n",
				__func__, namelen, name, ino, version);
		return -EEXIST;
	}

	return 0;
}

static int vital_data_app_dir(struct dir_context *ctx, const char *name,
		int namelen, loff_t offset, u64 ino, unsigned int d_type)
{
	char version[MAX_LEN] = {};

	strcat(version, vital_app);
	strcat(version, "-");

	if (strstr(name, version)) {
		memset(vital_app_ver, 0, MAX_LEN);
		strcat(vital_app_ver, name);
		vital_data_app_ino = ino;
		pr_info("%s: %.*s, ino=%lu, %s\n",
				__func__, namelen, name, ino, version);
		return -EEXIST;
	}

	return 0;
}

static int fcache_store(const char *buf,
		struct dir_context ctx[],
		char *app, char *ver,
		unsigned long *ino1,
		unsigned long *ino2,
		unsigned long *ino3)
{
	char name[MAX_LEN] = {};
	struct file *fp;
	mm_segment_t fs;
	int ret = 0;

	fp = filp_open("/data/app", O_RDONLY, 0);
	if (IS_ERR(fp)) {
		ret = PTR_ERR(fp);
		goto error;
	}

	memset(app, 0, MAX_LEN);
	snprintf(app, MAX_LEN, "%s", buf);

	if (app[strlen(app)-1] == 10)
		app[strlen(app)-1] = 0;

	pr_info("%s: %s\n", __func__, app);

	fs = get_fs();
	set_fs(KERNEL_DS);
	*ino3 = 0;
	ret = iterate_dir(fp, &ctx[0]);
	set_fs(fs);

	filp_close(fp, NULL);

	if (*ino3 == 0) {
		ret = -EINVAL;
		goto error;
	}

	memset(name, 0, sizeof(name));
	strcat(name, "/storage/emulated/0/Android/data/");
	strcat(name, app);
	pr_info("%s: %s\n", __func__, name);

	if (!is_dir_exist(name, ino1)) {
		ret = -ENOENT;
		goto error;
	}

	memset(name, 0, sizeof(name));
	strcat(name, "/data/data/");
	strcat(name, app);
	pr_info("%s: %s\n", __func__, name);

	if (!is_dir_exist(name, ino2)) {
		ret = -ENOENT;
		goto error;
	}

	goto out;

error:
	memset(app, 0, MAX_LEN);
	snprintf(app, MAX_LEN, "%s", "NOP");
	*ino1 = 0;
	*ino2 = 0;
	*ino3 = 0;

out:
	return ret;
}

static int param_set_top(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	struct dir_context ctx[] = {
		{.actor = &top_data_app_dir},
	};

	mutex_lock(&top_mutex);
	ret = fcache_store(val, ctx, top_app, top_app_ver,
			&top_android_data_ino,
			&top_data_data_ino,
			&top_data_app_ino);
	mutex_unlock(&top_mutex);
	return ret;
}

static int param_get_top(char *buf, const struct kernel_param *kp)
{
	int ret = 0;

	mutex_lock(&top_mutex);
	ret = sprintf(buf, "APK=%s Adata=%lu app=%lu data=%lu\n",
			top_app, top_android_data_ino,
			top_data_app_ino, top_data_data_ino);
	mutex_unlock(&top_mutex);
	return ret;
}

static int param_set_vital(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	struct dir_context ctx[] = {
		{.actor = &vital_data_app_dir},
	};

	mutex_lock(&vital_mutex);
	ret = fcache_store(val, ctx, vital_app, vital_app_ver,
			&vital_android_data_ino,
			&vital_data_data_ino,
			&vital_data_app_ino);
	mutex_unlock(&vital_mutex);
	return ret;
}

static int param_get_vital(char *buf, const struct kernel_param *kp)
{
	int ret = 0;

	mutex_lock(&vital_mutex);
	ret = sprintf(buf, "APK=%s Adata=%lu app=%lu data=%lu\n",
			vital_app, vital_android_data_ino,
			vital_data_app_ino, vital_data_data_ino);
	mutex_unlock(&vital_mutex);
	return ret;
}

static int param_get_fctrl(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%d\n", fctrl_en);
}

static int set_max_vital_fn(const char *val, const struct kernel_param *kp)
{
	int rc = 0;
	long param;

	rc = kstrtol(val, 0, &param);
	if (rc)
		return rc;

	if (param < 0)
		return -EINVAL;

	if (param > (SZ_1G >> PAGE_SHIFT))
		return -EINVAL;

	max_vital = param;
	return 0;
}

static int get_max_vital_fn(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%ld\n", max_vital);
}

static int set_max_staple_fn(const char *val, const struct kernel_param *kp)
{
	int rc = 0;
	long param;

	rc = kstrtol(val, 0, &param);
	if (rc)
		return rc;

	if (param < 0)
		return -EINVAL;

	if (param > (SZ_512M >> PAGE_SHIFT))
		return -EINVAL;

	max_staple = param;
	return 0;
}

static int get_max_staple_fn(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%ld\n", max_staple);
}

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX    "customize."

module_param_call(top, param_set_top, param_get_top, NULL, 0644);
module_param_call(vital, param_set_vital, param_get_vital, NULL, 0644);

module_param_call(check, NULL, param_get_fctrl, NULL, 0644);

module_param_named(manual, manual, int, 0644);

module_param_named(sys_lib, system_lib_ino, ulong, 0644);
module_param_named(sys_lib64, system_lib64_ino, ulong, 0644);
module_param_named(sys_apex, system_apex_ino, ulong, 0644);
module_param_named(sys_framework, system_framework_ino, ulong, 0644);
module_param_named(sys_fonts, system_fonts_ino, ulong, 0644);

module_param_named(SystemUI, SystemUI_ino, ulong, 0644);
module_param_named(Launcher, Launcher_ino, ulong, 0644);
module_param_named(WebView, WebView_ino, ulong, 0644);
module_param_named(TrichromeLib, TrichromeLib_ino, ulong, 0644);
module_param_named(QtiSystemSer, QtiSystemSer_ino, ulong, 0644);

module_param_named(InputDevices, InputDevices_ino, ulong, 0644);
module_param_named(InProcessNetworkStack, InProcessNetworkStack_ino, ulong, 0644);
module_param_named(PermissionController, PermissionController_ino, ulong, 0644);
module_param_named(SettingsProvider, SettingsProvider_ino, ulong, 0644);

module_param_named(sys_p_lib, system_pro_lib_ino, ulong, 0644);
module_param_named(sys_p_lib64, system_pro_lib64_ino, ulong, 0644);
module_param_named(sys_p_etc, system_pro_etc_ino, ulong, 0644);
module_param_named(sys_p_overlay, system_pro_overlay_ino, ulong, 0644);

module_param_named(kswap, mm_kswap, int, 0644);

module_param_named(min_file, min_file, ulong, 0644);
module_param_named(nr_low, nr_low, ulong, 0644);

module_param_call(max_vital, set_max_vital_fn, get_max_vital_fn, NULL, 0644);
module_param_call(max_staple, set_max_staple_fn, get_max_staple_fn, NULL, 0644);

static int fcache_thread(void *data)
{
	set_user_nice(current, MIN_NICE);
	set_freezable();

	while (!kthread_should_stop()) {
		if (!system_lib_ino)
			is_dir_exist("/system/lib", &system_lib_ino);

		if (!system_lib64_ino)
			is_dir_exist("/system/lib64", &system_lib64_ino);

		if (!system_apex_ino)
			is_dir_exist("/system/apex", &system_apex_ino);

		if (!system_framework_ino)
			is_dir_exist("/system/framework", &system_framework_ino);

		if (!system_fonts_ino)
			is_dir_exist("/system/fonts", &system_fonts_ino);

#if 0
		if (!SystemUI_ino)
			is_dir_exist("/system/priv-app/SystemUI",
					&SystemUI_ino);

		if (!InputDevices_ino)
			is_dir_exist("/system/priv-app/InputDevices",
					&InputDevices_ino);

		if (!InProcessNetworkStack_ino)
			is_dir_exist("/system/priv-app/InProcessNetworkStack",
					&InProcessNetworkStack_ino);

		if (!PermissionController_ino)
			is_dir_exist("/system/priv-app/PermissionController",
					&PermissionController_ino);

		if (!SettingsProvider_ino)
			is_dir_exist("/system/priv-app/SettingsProvider",
					&SettingsProvider_ino);

		if (!Launcher_ino)
			is_dir_exist("/system/app/BBKLauncher2",
					&Launcher_ino);

		if (!WebView_ino)
			is_dir_exist("/system/product/app/WebViewGoogle",
					&WebView_ino);

		if (!TrichromeLib_ino)
			is_dir_exist("/system/product/app/TrichromeLibrary",
					&TrichromeLib_ino);

		if (!QtiSystemSer_ino)
			is_dir_exist("/system/product/app/QtiSystemService",
					&QtiSystemSer_ino);

		if (!system_pro_lib_ino)
			is_dir_exist("/system/product/lib",
					&system_pro_lib_ino);

		if (!system_pro_lib64_ino)
			is_dir_exist("/system/product/lib64",
					&system_pro_lib64_ino);

		if (!system_pro_etc_ino)
			is_dir_exist("/system/product/etc",
					&system_pro_etc_ino);

		if (!system_pro_overlay_ino)
			is_dir_exist("/system/product/overlay",
					&system_pro_overlay_ino);
#endif

#if 0
		if (!vendor_lib_ino)
			is_dir_exist("/vendor/lib", &vendor_lib_ino);

		if (!vendor_lib64_ino)
			is_dir_exist("/vendor/lib64", &vendor_lib64_ino);
#endif

#ifdef CONFIG_FCACHE_CTRL_DATA
		if (!data_app_ino)
			is_dir_exist("/data/app", &data_app_ino);

		if (!data_data_ino)
			is_dir_exist("/data/data", &data_data_ino);

		if (!android_data_ino)
			is_dir_exist("/storage/emulated/0/Android/data",
					&android_data_ino);
#endif

		if (ktime_get_boot_ns() > (NSEC_PER_SEC * 60)) {
			if (!manual)
				return 0;

			if (
#ifdef CONFIG_FCACHE_CTRL_DATA
				!android_data_ino ||
				!data_app_ino ||
				!data_data_ino ||
#endif
				!system_lib_ino ||
				!system_lib64_ino ||
				!system_apex_ino ||
				!system_framework_ino ||
				!system_fonts_ino/* ||
				!SystemUI_ino ||
				!Launcher_ino ||
				!WebView_ino ||
				!TrichromeLib_ino ||
				!QtiSystemSer_ino ||
				!InputDevices_ino ||
				!InProcessNetworkStack_ino ||
				!PermissionController_ino ||
				!SettingsProvider_ino ||
				!system_pro_lib_ino ||
				!system_pro_lib64_ino ||
				!system_pro_etc_ino ||
				!system_pro_overlay_ino*/)
				return 0;

			if (system_lib_ino > MAX_INODE ||
					system_lib64_ino > MAX_INODE ||
					system_apex_ino > MAX_INODE ||
					system_framework_ino > MAX_INODE ||
					system_fonts_ino > MAX_INODE)
				return 0;

			fctrl_en = 1;
			return 0;
		}

		msleep(300);
	}

	return 0;
}

static int __init filecache_init(void)
{
	struct task_struct *thread = NULL;
	const u64 limit = totalram_pages << PAGE_SHIFT;

	if (limit < TOTAL_RAM)
		return 0;

	if (is_atboot == 1) {
		pr_info("%s: in AT mode\n", __func__);
		return 0;
	}

	thread = kthread_run(fcache_thread, NULL, "fcache");
	if (IS_ERR(thread))
		return PTR_ERR(thread);

	return 0;
}

module_init(filecache_init);
