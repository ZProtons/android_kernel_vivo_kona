#ifndef __WANGHAI_DEBUG__
#define __WANGHAI_DEBUG__

///////////////////////////////////////////////
/*
 * 全局控制 debug 开关;
 */
//#define ENABLE_DEBUG
//#undef ENABLE_DEBUG

/*
 * 如果要 重新 定义 LOG_TAG, 就 把 以下的代码, 放到 include <oem_debug.h> 之前;
 *
 *   #undef LOG_TAG
 *   #define LOG_TAG "<<-- xxx-debug -->>"
 */
#ifndef LOG_TAG
#define LOG_TAG "<<-- vivo-debug -->>"
#endif

///////////////////////////////////////////////

/*
 * 缺省 使用 printk 打印
 */
#ifndef print_interface
#define print_interface printk
#endif

/*
 * 强制 打印接口;
 */
#define MY__INFO(fmt, ...) \
	((void)print_interface(LOG_TAG " @%s, line:[%d], " fmt, __func__, __LINE__, ##__VA_ARGS__))

/*
 * 条件控制 打印接口;
 */
#define MY__INFO_IF(cond, fmt, ...) \
	(cond ? (MY__INFO(fmt, ##__VA_ARGS__)) : (void)0)

/*
 * 开关控制 打印接口;
 */
#ifdef ENABLE_DEBUG
#define MY__DEBUG(fmt, ...) MY__INFO(fmt, ##__VA_ARGS__)
#else
#define MY__DEBUG(fmt, ...) //
#endif

/*
 * 定义 常用调试接口 别名
 */

/* 开关控制 打印接口 */
#define my_dbg      MY__DEBUG
#define oem_dbg     MY__DEBUG
#define vivo_dbg     MY__DEBUG

/* 强制 打印接口 */
#define my_info     MY__INFO
#define vivo_info    MY__INFO

/* 条件控制 打印接口 */
#define my_info_if MY__INFO_IF
#define vivo_info_if MY__INFO_IF

#endif //__WANGHAI_DEBUG__
