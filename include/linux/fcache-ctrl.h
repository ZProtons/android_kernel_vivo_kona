/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_FCACHE_CTRL_H
#define _LINUX_FCACHE_CTRL_H


extern unsigned long android_data_ino;
extern unsigned long data_app_ino;
extern unsigned long data_data_ino;

extern unsigned long system_lib_ino;
extern unsigned long system_lib64_ino;
extern unsigned long system_apex_ino;
extern unsigned long system_framework_ino;
extern unsigned long system_fonts_ino;

extern unsigned long SystemUI_ino;
extern unsigned long Launcher_ino;
extern unsigned long WebView_ino;
extern unsigned long TrichromeLib_ino;
extern unsigned long QtiSystemSer_ino;

extern unsigned long InputDevices_ino;
extern unsigned long InProcessNetworkStack_ino;
extern unsigned long PermissionController_ino;
extern unsigned long SettingsProvider_ino;

extern unsigned long system_pro_lib_ino;
extern unsigned long system_pro_lib64_ino;
extern unsigned long system_pro_etc_ino;
extern unsigned long system_pro_overlay_ino;

extern unsigned long vendor_lib_ino;
extern unsigned long vendor_lib64_ino;

extern unsigned long top_android_data_ino;
extern unsigned long top_data_app_ino;
extern unsigned long top_data_data_ino;

extern unsigned long vital_android_data_ino;
extern unsigned long vital_data_app_ino;
extern unsigned long vital_data_data_ino;

extern unsigned long min_file;
extern unsigned long nr_low;

extern long max_vital;
extern long max_staple;

extern int mm_kswap;

extern int fctrl_en;

static inline bool is_top_app(struct inode *inode)
{
#if 0
	if (inode->i_sb->s_point == DIR_DATA) {
		if (!inode->i_child_of)
			return false;

		if (inode->i_child_of == top_android_data_ino ||
				inode->i_child_of == top_data_app_ino ||
				inode->i_child_of == top_data_data_ino)
			return true;
	}
#endif

	return false;
}

static inline bool is_vital_app(struct inode *inode)
{
#ifdef CONFIG_FCACHE_CTRL_DATA
	if (inode->i_sb->s_point == DIR_DATA) {
		if (!inode->i_child_of)
			return false;

		if (inode->i_child_of == vital_android_data_ino ||
				inode->i_child_of == vital_data_app_ino ||
				inode->i_child_of == vital_data_data_ino)
			return true;
	}
#endif
	return false;
}

static inline bool is_vital_inode(struct inode *inode)
{
	if (inode->i_vital)
		return true;

	return false;
}

static inline bool is_dir_vital(unsigned long ino)
{
	if (ino == system_lib_ino ||
			ino == system_lib64_ino ||
			ino == system_apex_ino ||
			ino == system_framework_ino/* ||
			ino == system_fonts_ino ||
			ino == SystemUI_ino ||
			ino == Launcher_ino ||
			ino == WebView_ino ||
			ino == TrichromeLib_ino ||
			ino == QtiSystemSer_ino ||
			ino == InputDevices_ino ||
			ino == InProcessNetworkStack_ino ||
			ino == PermissionController_ino ||
			ino == SettingsProvider_ino ||
			ino == system_pro_lib_ino ||
			ino == system_pro_lib64_ino ||
			ino == system_pro_etc_ino ||
			ino == system_pro_overlay_ino */)
		return true;

	return false;
}

static inline bool staple_list_is_full(unsigned long nr)
{
	return nr > max_staple;
}

static inline bool vital_list_is_full(unsigned long nr)
{
	return nr > max_vital;
}

static inline bool lru_file_is_low(unsigned long nr)
{
	return nr < min_file;
}

#endif /* _LINUX_FCACHE_CTRL_H */
