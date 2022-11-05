/*
 * mm/mem_big_data.h
 *
 * VIVO Kernel Monitor Engine(Memory)
 *
 * Copyright (C) 2020 VIVO Technology Co., Ltd
 * <rongqianfeng@vivo.com>
*/
#ifndef __MEM_BIG_DATA_H
#define __MEM_BIG_DATA_H

#include <linux/percpu-defs.h>

enum membigdata_item {
	ALLOCSTALL_TOP,
	ALLOCSTALL_EO3,
	ALLOCSTALL_GO3,
	ALLOCSTALL_GE_10ms,
	ALLOC_FAILD,
	ALLOC_FAILD_KILL,
	ALLOC_EO3,
	ALLOC_GO3,
	OOM_KILL_GEO3,
	OOM_KILL_LEO2,
	LMKD_TOP_KILL,
	VM_OOM_COUNT,
	COMPACT_MIGRATE_SUCCESS,
	COMPACT_MIGRATE_FAIL,
	CMA_MIGRATE_SUCCESS,
	CMA_MIGRATE_FAIL,
	KSWAPD_WAKEUP_COUNT,
	ION_LEAK_COUNT,
	GPU_LEAK_COUNT,
	SLAB_LEAK_COUNT,
	KSTACK_LEAK_COUNT,

	SWAP_STAY_FREE_GE0,
	SWAP_STAY_FREE_GE5,
	SWAP_STAY_FREE_GE10,
	SWAP_STAY_FREE_GE15,

	SWAP_STAY_READ_GE0,
	SWAP_STAY_READ_GE5,
	SWAP_STAY_READ_GE10,
	SWAP_STAY_READ_GE15,

	NR_MEMBIGDATA_EVENT_ITEMS
};

#ifdef CONFIG_MEM_BIGDATA

extern void mbd_time_geo3_event(u64 single_time);

extern void mbd_swap_range_out_record(struct page *page, int nr);
extern void mbd_swap_in_record(struct page *page);
extern void mbd_swap_free_record(struct page *page);
extern void mbd_swap_range_free_record(struct swap_info_struct *, unsigned long, unsigned int);

struct membigdata_state {
	unsigned long event[NR_MEMBIGDATA_EVENT_ITEMS];
};

DECLARE_PER_CPU(struct membigdata_state, membigdata_states);
/*
 * mbd counters are allowed to be racy. Use raw_cpu_ops to avoid the
 * local_irq_disable overhead.
 */
static inline void __count_mbd_event(enum membigdata_item item)
{
	raw_cpu_inc(membigdata_states.event[item]);
}

static inline void count_mbd_event(enum membigdata_item item)
{
	this_cpu_inc(membigdata_states.event[item]);
}

static inline void __count_mbd_events(enum membigdata_item item, long delta)
{
	raw_cpu_add(membigdata_states.event[item], delta);
}

static inline void count_mbd_events(enum membigdata_item item, long delta)
{
	this_cpu_add(membigdata_states.event[item], delta);
}

#else

/* Disable counters */
static inline void count_mbd_event(enum membigdata_item item)
{
}
static inline void count_mbd_events(enum membigdata_item item, long delta)
{
}
static inline void __count_mbd_event(enum membigdata_item item)
{
}
static inline void __count_mbd_events(enum membigdata_item item, long delta)
{
}

#endif /* CONFIG_MEM_BIGDATA */

#endif /*__MEM_BIG_DATA_H*/
