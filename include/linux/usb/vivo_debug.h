#ifndef __LINUX_USB_VIVO_DEBUG_H
#define __LINUX_USB_VIVO_DEBUG_H

#ifndef USB_DEBUG_TAG
#define USB_DEBUG_TAG "vusb dbg: "
#endif

#undef dev_emerg
#undef dev_alert
#undef dev_crit
#undef dev_err
#undef dev_warn
#undef dev_notice
#undef _dev_info
#undef dev_dbg

#define dev_emerg(dev, fmt, ...) _dev_emerg(dev, USB_DEBUG_TAG fmt, ##__VA_ARGS__)
#define dev_alert(dev, fmt, ...) _dev_alert(dev, USB_DEBUG_TAG fmt, ##__VA_ARGS__)
#define dev_crit(dev, fmt, ...) _dev_crit(dev, USB_DEBUG_TAG fmt, ##__VA_ARGS__)
#define dev_err(dev, fmt, ...) _dev_err(dev, USB_DEBUG_TAG fmt, ##__VA_ARGS__)
#define dev_warn(dev, fmt, ...) _dev_warn(dev, USB_DEBUG_TAG fmt, ##__VA_ARGS__)
#define dev_notice(dev, fmt, ...) _dev_notice(dev, USB_DEBUG_TAG fmt, ##__VA_ARGS__)
#define _dev_info(dev, fmt, ...) _dev_info(dev, USB_DEBUG_TAG fmt, ##__VA_ARGS__)
#if defined(CONFIG_DYNAMIC_DEBUG)
#define dev_dbg(dev, format, ...)					\
	do {								\
		dynamic_dev_dbg(dev, USB_DEBUG_TAG format, ##__VA_ARGS__);	\
	} while (0)
#elif defined(DEBUG)
#define dev_dbg(dev, format, arg...)				\
	dev_printk(KERN_DEBUG, dev, USB_DEBUG_TAG format, ##arg)
#else
#define dev_dbg(dev, format, arg...)					\
	({								\
		if (0)							\
			dev_printk(KERN_DEBUG, dev, format, ##arg);	\
	})
#endif

#undef pr_emerg
#undef pr_alert
#undef pr_crit
#undef pr_err
#undef pr_warning
#undef pr_notice
#undef pr_info
#undef pr_debug

#define pr_emerg(fmt, ...) printk(KERN_EMERG pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#define pr_alert(fmt, ...) printk(KERN_ALERT pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#define pr_crit(fmt, ...) printk(KERN_CRIT pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#define pr_err(fmt, ...) printk(KERN_ERR pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#define pr_warning(fmt, ...) printk(KERN_WARNING pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#define pr_notice(fmt, ...) printk(KERN_NOTICE pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#define pr_info(fmt, ...) printk(KERN_INFO pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#if defined(CONFIG_DYNAMIC_DEBUG)
#define pr_debug(fmt, ...) \
	dynamic_pr_debug(USB_DEBUG_TAG fmt, ##__VA_ARGS__)
#elif defined(DEBUG)
#define pr_debug(fmt, ...) \
	printk(KERN_DEBUG pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...) \
	no_printk(KERN_DEBUG pr_fmt(USB_DEBUG_TAG fmt), ##__VA_ARGS__)
#endif

#endif
