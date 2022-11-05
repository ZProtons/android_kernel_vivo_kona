/*
 * include/linux/mem_big_order.h
 *
 * VIVO Kernel Memory Monitor
 *
 * <rongqianfeng@vivo.com>
 *
*/

#ifndef _MEM_BIG_ORDER_H
#define _MEM_BIG_ORDER_H

#ifdef CONFIG_BIG_ORDER_MONITOR
#include <linux/mmzone.h>
#include <linux/fault-inject.h>

extern int mem_big_order_monitor(gfp_t gfp_mask, unsigned int order);

#endif /*CONFIG_BIG_ORDER_MONITOR*/

#endif
