/*
 * mm/mem_big_data.c
 *
 * VIVO Kernel Monitor Engine(Memory)
 *
 * Copyright (C) 2020 VIVO Technology Co., Ltd
 * <rongqianfeng@vivo.com>
*/
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/percpu.h>
#include <linux/seq_file.h>
#include <linux/vmstat.h>
#include <linux/module.h>
#include <linux/atomic.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/vm_event_item.h>
#include <linux/blkdev.h>
#include "internal.h"
#include <linux/mem_big_data.h>
#include <linux/ktime.h>

#ifdef CONFIG_RSC_MEM_DEFRAG
#include <linux/vivo_rsc/rsc_internal.h>
#endif

#ifdef CONFIG_RSC_MEM_STAT
extern atomic_long_t rsc_ion_head_free_list_size;
extern atomic_t rsc_ion_pool_pages;
#endif

extern int rsc_kgsl_pool_size_total(void);

#define ONE_GB_KB (1*1024*1024UL)
#define TWO_GB_KB (2*1024*1024UL)
#define THREE_GB_KB (3*1024*1024UL)
#define FOUR_GB_KB (4*1024*1024UL)
#define FIVE_GB_KB (5*1024*1024UL)
#define SEVEN_GB_KB (7*1024*1024UL)

#define ONE_GB_PAGES (ONE_GB_KB  >> (PAGE_SHIFT - 10))
#define TWO_GB_PAGES (TWO_GB_KB >> (PAGE_SHIFT - 10))
#define THREE_GB_PAGES (THREE_GB_KB  >> (PAGE_SHIFT - 10))
#define FOUR_GB_PAGES (FOUR_GB_KB  >> (PAGE_SHIFT - 10))
#define FIVE_GB_PAGES (FIVE_GB_KB  >> (PAGE_SHIFT - 10))
#define SEVEN_GB_PAGES (SEVEN_GB_KB  >> (PAGE_SHIFT - 10))

DEFINE_PER_CPU(struct membigdata_state, membigdata_states) = { {0} };
EXPORT_PER_CPU_SYMBOL(membigdata_states);

#define MBD_SEC_PER_MIN 		60
#define MBD_SEC_IN_5MIN 		(MBD_SEC_PER_MIN * 5)
#define MBD_SEC_IN_10MIN 		(MBD_SEC_PER_MIN * 10)
#define MBD_SEC_IN_15MIN 		(MBD_SEC_PER_MIN * 15)

#define MBD_STAY_FREE_ITEMS		(SWAP_STAY_FREE_GE15 - SWAP_STAY_FREE_GE0 + 1)

static inline unsigned int mbd_relative_idx(time64_t dt)
{
	unsigned int idx = dt / MBD_SEC_IN_5MIN;
	if (idx >= MBD_STAY_FREE_ITEMS)
		idx = MBD_STAY_FREE_ITEMS - 1;
	return idx;
}

/* Parameter nr only works while out is true,
 * maybe support false value in the future.
 */
static void mbd_swap_record(struct page *page, bool out, int nr)
{
	struct swap_info_struct *sis;
	struct swap_cluster_info *ci;
	swp_entry_t entry;
	unsigned long offset, end;
	time64_t now, dt;

	entry.val = page_private(page);
	sis = swp_swap_info(entry);
	offset = swp_offset(entry);
	now = ktime_get_boottime_seconds();

	ci = mbd_lock_cluster(sis, offset);
	if (out) {
		end = offset + nr - 1;
		while (offset <= end) {
			sis->swap_timestamp[offset] = now;
			offset++;
		}
	} else {
		if (unlikely(!sis->swap_timestamp[offset])) {
			mbd_unlock_cluster(ci);
			return;
		}

		dt = now - sis->swap_timestamp[offset];
	}
	mbd_unlock_cluster(ci);

	if (!out)
		count_mbd_event(SWAP_STAY_READ_GE0 + mbd_relative_idx(dt));
}

inline void mbd_swap_range_out_record(struct page *page, int nr)
{
	mbd_swap_record(page, true, nr);
}

inline void mbd_swap_in_record(struct page *page)
{
	mbd_swap_record(page, false, 1);
}

void mbd_swap_range_free_record(struct swap_info_struct *sis, unsigned long offset,
				unsigned int nr)
{
	unsigned long head = offset;
	unsigned long end = offset + nr - 1;
	struct swap_cluster_info *ci;
	time64_t now;
	int indices[MBD_STAY_FREE_ITEMS] = {0};
	int i;

	now = ktime_get_boottime_seconds();

	ci = mbd_lock_cluster(sis, offset);
	while (offset <= end) {
		if (sis->swap_timestamp[offset]) {
			time64_t dt = now - sis->swap_timestamp[offset];
			indices[mbd_relative_idx(dt)]++;
		}
		offset++;
	}
	memset(sis->swap_timestamp + head, 0, nr * sizeof(time64_t));
	mbd_unlock_cluster(ci);

	for (i = 0; i < MBD_STAY_FREE_ITEMS; ++i) {
		if (indices[i])
			count_mbd_events(SWAP_STAY_FREE_GE0 + i, indices[i]);
	}
}

unsigned long mbd_time_geo3_count;
u64 mbd_time_geo3_total;
u64 mbd_time_geo3_peak;

inline void mbd_time_geo3_event(u64 single_time)
{
	mbd_time_geo3_count++;
	mbd_time_geo3_total += single_time;
	mbd_time_geo3_peak = max(mbd_time_geo3_peak, single_time);
}

static void sum_mbd_events(unsigned long *ret)
{
	int cpu;
	int i;

	memset(ret, 0, NR_MEMBIGDATA_EVENT_ITEMS * sizeof(unsigned long));

	for_each_online_cpu(cpu) {
		struct membigdata_state *this = &per_cpu(membigdata_states, cpu);

		for (i = 0; i < NR_MEMBIGDATA_EVENT_ITEMS; i++)
			ret[i] += this->event[i];
	}
}

static void all_mbd_events(unsigned long *ret)
{
	get_online_cpus();
	sum_mbd_events(ret);
	put_online_cpus();
}

static unsigned long all_zones_count(unsigned long *vm)
{
	unsigned long ret = 0;

#ifdef CONFIG_ZONE_DMA
	ret += vm[ALLOCSTALL_DMA];
#endif

#ifdef CONFIG_ZONE_DMA32
	ret += vm[ALLOCSTALL_DMA32];
#endif

#ifdef CONFIG_HIGHMEM
	ret += vm[ALLOCSTALL_HIGH];
#endif

	ret += vm[ALLOCSTALL_NORMAL];
	ret += vm[ALLOCSTALL_MOVABLE];
	return ret;
}

struct mbd_contig_page_info {
	unsigned long free_pages;
	unsigned long free_blocks_total;
	unsigned long free_blocks_suitable;
};

static void mbd_fill_contig_page_info(struct zone *zone,
				unsigned int suitable_order,
				struct mbd_contig_page_info *info)
{
	unsigned int order;

	info->free_pages = 0;
	info->free_blocks_total = 0;
	info->free_blocks_suitable = 0;

	for (order = 0; order < MAX_ORDER; order++) {
		unsigned long blocks;

		/* Count number of free blocks */
		blocks = zone->free_area[order].nr_free;
		info->free_blocks_total += blocks;

		/* Count free base pages */
		info->free_pages += blocks << order;

		/* Count the suitable free blocks */
		if (order >= suitable_order)
			info->free_blocks_suitable += blocks <<
						(order - suitable_order);
	}
}

static int mbd_unusable_free_index(unsigned int order,
				struct mbd_contig_page_info *info)
{
	/* No free memory is interpreted as all free memory is unusable */
	if (info->free_pages == 0)
		return 1000;

	/*
	 * Index should be a value between 0 and 1. Return a value to 3
	 * decimal places.
	 *
	 * 0 => no fragmentation
	 * 1 => high fragmentation
	 */
	return div_u64((info->free_pages - (info->free_blocks_suitable << order)) * 1000ULL, info->free_pages);

}

static void mbd_physical_frag_level(int index[][MAX_ORDER])
{
	int zones = 0;
	unsigned int order;
	struct zone *zone;
	struct mbd_contig_page_info info;

	for_each_populated_zone(zone) {
		for (order = 0; order < MAX_ORDER; ++order) {
			mbd_fill_contig_page_info(zone, order, &info);
			index[zones][order] = mbd_unusable_free_index(order, &info);
		}
		zones++;
	}
}

static void get_physical_frag_level(char *buf, int buf_size)
{
	int order, zones = 0, p = 0;
	int frag_index[MAX_NR_ZONES][MAX_ORDER] = {0};
	struct zone *zone;
	mbd_physical_frag_level(frag_index);

	for_each_populated_zone(zone) {
		p += scnprintf(buf + p, buf_size - p, "%s ", zone->name);
		for (order = 0; order < MAX_ORDER; ++order) {
			p += scnprintf(buf + p, buf_size - p, "%d.%03d ",
				frag_index[zones][order] / 1000, frag_index[zones][order] % 1000);
		}
		zones++;
	}
}

static struct timer_list mem_check_timer;
static unsigned int expires_sec = 60*30;


static void mdb_mem_check(struct timer_list *timer)
{
	unsigned long kernel_stack_limit;

	if (global_zone_page_state(NR_ION) > (totalram_pages/2))
		__count_mbd_event(ION_LEAK_COUNT);
	if (global_zone_page_state(NR_GPU) > (totalram_pages/3))
		__count_mbd_event(GPU_LEAK_COUNT);
	if (global_node_page_state(NR_SLAB_UNRECLAIMABLE) > (totalram_pages/3))
		__count_mbd_event(SLAB_LEAK_COUNT);

	if (totalram_pages > SEVEN_GB_PAGES)
		kernel_stack_limit = THREAD_SIZE*30000/1024;
	else if (totalram_pages > FIVE_GB_PAGES)
		kernel_stack_limit = THREAD_SIZE*25000/1024;
	else if (totalram_pages > THREE_GB_PAGES)
		kernel_stack_limit = THREAD_SIZE*20000/1024;
	else
		kernel_stack_limit = THREAD_SIZE*15000/1024;

	if (global_zone_page_state(NR_KERNEL_STACK_KB) > kernel_stack_limit)
		__count_mbd_event(KSTACK_LEAK_COUNT);

	if (!timer_pending(&mem_check_timer))
		mod_timer(&mem_check_timer, jiffies + msecs_to_jiffies(expires_sec * 1000));
}

static void mdb_mem_check_init(unsigned char nsec)
{
	timer_setup(&mem_check_timer, mdb_mem_check, 0);
	mem_check_timer.expires = jiffies + HZ * nsec;
	add_timer(&mem_check_timer);
}

static unsigned long mbd[NR_MEMBIGDATA_EVENT_ITEMS];
static unsigned long vm[NR_VM_EVENT_ITEMS];
static char phys_level_buf[PAGE_SIZE];
#define page_to_kb(x) ((x) << (PAGE_SHIFT - 10))

static int membigdata_show(struct seq_file *m, void *arg)
{
	unsigned long buffers = nr_blockdev_pages();
	struct sysinfo si;
	unsigned time_geo3_count;
	u64 time_geo3_total;
	u64 time_geo3_peak;

	get_physical_frag_level(phys_level_buf, PAGE_SIZE);
	all_mbd_events(mbd);
	all_vm_events(vm);
	si_swapinfo(&si);

	time_geo3_count = mbd_time_geo3_count;
	time_geo3_total = mbd_time_geo3_total;
	time_geo3_peak = mbd_time_geo3_peak;

	seq_printf(m, "uptime:                          %lu\n",
		   ((jiffies-INITIAL_JIFFIES)/CONFIG_HZ)/3600);
	seq_printf(m, "mem_total:                       %lu\n",
		   totalram_pages*4/ONE_GB_KB + 1);
	seq_printf(m, "slowpath_count:                  %lu\n",
		   all_zones_count(vm));
	seq_printf(m, "slowpath_count_top:              %lu\n",
		   mbd[ALLOCSTALL_TOP]);
	seq_printf(m, "slowpath_count_eo3:              %lu\n",
		   mbd[ALLOCSTALL_EO3]);
	seq_printf(m, "slowpath_count_go3:              %lu\n",
		   mbd[ALLOCSTALL_GO3]);
	seq_printf(m, "slowpath_stall_ge_10ms:          %lu\n",
		   mbd[ALLOCSTALL_GE_10ms]);
	seq_printf(m, "alloc_faild:                     %lu\n",
		   mbd[ALLOC_FAILD]);
	seq_printf(m, "alloc_faild_kill:                %lu\n",
		   mbd[ALLOC_FAILD_KILL]);
	seq_printf(m, "alloc_eo3:                       %lu\n",
		   mbd[ALLOC_EO3]);
	seq_printf(m, "alloc_go3:                       %lu\n",
		   mbd[ALLOC_GO3]);
	seq_printf(m, "alloc_time_geo3:                 %lu\n",
		   time_geo3_count ? (time_geo3_total / time_geo3_count) : 0);
	seq_printf(m, "alloc_time_geo3_peak:            %lu\n",
		   time_geo3_peak);
	seq_printf(m, "oom_kill:                        %lu\n",
		   vm[OOM_KILL]);
	seq_printf(m, "oom_kill_geo3:                   %lu\n",
		   mbd[OOM_KILL_GEO3]);
	seq_printf(m, "oom_kill_leo2:                   %lu\n",
		   mbd[OOM_KILL_LEO2]);
	seq_printf(m, "lmkd_top_kill:                   %lu\n",
		   mbd[LMKD_TOP_KILL]);
	seq_printf(m, "vm_oom_count:                    %lu\n",
		   mbd[VM_OOM_COUNT]);
	seq_printf(m, "physical_frag_level:             %s\n",
		   phys_level_buf);
	seq_printf(m, "compact_success:                 %lu\n",
		   vm[COMPACTSUCCESS]);
	seq_printf(m, "compact_stall:                   %lu\n",
		   vm[COMPACTSTALL]);
	seq_printf(m, "comapct_falied:                  %lu\n",
		   vm[COMPACTFAIL]);
	seq_printf(m, "compact_migrate_success_pages:   %lu\n",
		   mbd[COMPACT_MIGRATE_SUCCESS]);
	seq_printf(m, "compact_migrate_fail_pages:      %lu\n",
		   mbd[COMPACT_MIGRATE_FAIL]);
	seq_printf(m, "cma_migrate_success_pages:       %lu\n",
		   mbd[CMA_MIGRATE_SUCCESS]);
	seq_printf(m, "cma_migrate_fail_pages:          %lu\n",
		   mbd[CMA_MIGRATE_FAIL]);
	seq_printf(m, "compact_wake_count:              %lu\n",
		   vm[KCOMPACTD_WAKE]);
	seq_printf(m, "kswapd_wakeup_count:             %lu\n",
		   mbd[KSWAPD_WAKEUP_COUNT]);
	seq_printf(m, "ion_leak_count:                  %lu\n",
		   mbd[ION_LEAK_COUNT]);
	seq_printf(m, "gpu_leak_count:                  %lu\n",
		   mbd[GPU_LEAK_COUNT]);
	seq_printf(m, "slab_leak_count:                 %lu\n",
		   mbd[SLAB_LEAK_COUNT]);
	seq_printf(m, "kstack_leak_count:               %lu\n",
		   mbd[KSTACK_LEAK_COUNT]);
	seq_printf(m, "page_in:                         %lu\n",
		   vm[PGPGIN]);
	seq_printf(m, "page_out:                        %lu\n",
		   vm[PGPGOUT]);
	seq_printf(m, "swap_total:                      %lu\n",
		   si.totalswap);
	seq_printf(m, "swap_used:                       %lu\n",
		   si.totalswap - si.freeswap);
	seq_printf(m, "swap_out:                        %lu\n",
		   vm[PSWPOUT]);
	seq_printf(m, "swap_in:                         %lu\n",
		   vm[PSWPIN]);
	seq_printf(m, "swap_stay_free:                  %lu %lu %lu %lu\n",
		mbd[SWAP_STAY_FREE_GE0], mbd[SWAP_STAY_FREE_GE5], mbd[SWAP_STAY_FREE_GE10], mbd[SWAP_STAY_FREE_GE15]);
	seq_printf(m, "swap_stay_read:                  %lu %lu %lu %lu\n",
		mbd[SWAP_STAY_READ_GE0], mbd[SWAP_STAY_READ_GE5], mbd[SWAP_STAY_READ_GE10], mbd[SWAP_STAY_READ_GE15]);
	seq_printf(m, "mem_free:                        %lu\n",
		   page_to_kb(global_zone_page_state(NR_FREE_PAGES)));
	seq_printf(m, "mem_available:                   %lu\n",
		   page_to_kb(si_mem_available()));
	seq_printf(m, "buffers:                         %lu\n",
		   page_to_kb(buffers));
	seq_printf(m, "cached:                          %lu\n",
		   page_to_kb(global_node_page_state(NR_FILE_PAGES)
		   - total_swapcache_pages() - buffers));
	seq_printf(m, "active:                          %lu\n",
		   page_to_kb(global_node_page_state(NR_ACTIVE_ANON)
		   +global_node_page_state(NR_ACTIVE_FILE)));
	seq_printf(m, "inactive:                        %lu\n",
		   page_to_kb(global_node_page_state(NR_INACTIVE_ANON)
		   +global_node_page_state(NR_INACTIVE_FILE)));
	seq_printf(m, "active_anon:                     %lu\n",
		   page_to_kb(global_node_page_state(NR_ACTIVE_ANON)));
	seq_printf(m, "inactive_anon:                   %lu\n",
		   page_to_kb(global_node_page_state(NR_INACTIVE_ANON)));
	seq_printf(m, "active_file:                     %lu\n",
		   page_to_kb(global_node_page_state(NR_ACTIVE_ANON)));
	seq_printf(m, "inactive_file:                   %lu\n",
		   page_to_kb(global_node_page_state(NR_INACTIVE_ANON)));
	seq_printf(m, "dirty:                           %lu\n",
		   page_to_kb(global_node_page_state(NR_FILE_DIRTY)));
	seq_printf(m, "writeback:                       %lu\n",
		   page_to_kb(global_node_page_state(NR_WRITEBACK)));
	seq_printf(m, "anon_pages:                      %lu\n",
		   page_to_kb(global_node_page_state(NR_ANON_MAPPED)));
	seq_printf(m, "file_mapped_pages:               %lu\n",
		   page_to_kb(global_node_page_state(NR_FILE_MAPPED)));
	seq_printf(m, "slab_reclaimable:                %lu\n",
		   page_to_kb(global_node_page_state(NR_SLAB_RECLAIMABLE)));
	seq_printf(m, "slab_unreclaimable:              %lu\n",
		   page_to_kb(global_node_page_state(NR_SLAB_UNRECLAIMABLE)));
	seq_printf(m, "kernel_stack:                    %lu\n",
		   page_to_kb(global_zone_page_state(NR_KERNEL_STACK_KB)));
	seq_printf(m, "page_tables:                     %lu\n",
		   page_to_kb(global_zone_page_state(NR_PAGETABLE)));
	seq_printf(m, "vmalloc:                         %lu\n",
		   page_to_kb(vmalloc_nr_pages()));
	seq_printf(m, "ion_used:                        %lu\n",
		   page_to_kb(global_zone_page_state(NR_ION)));
	seq_printf(m, "ion_free:                        %lu\n",
		   page_to_kb((unsigned long)atomic_read(&rsc_ion_pool_pages) + (atomic_long_read(&rsc_ion_head_free_list_size) >> PAGE_SHIFT)));
	seq_printf(m, "gpu_used:                        %lu\n",
		   page_to_kb(global_zone_page_state(NR_GPU)));
	seq_printf(m, "gpu_free:                        %lu\n",
		   page_to_kb((unsigned long)rsc_kgsl_pool_size_total()));

	return 0;
}

static int __init membigdata_init(void)
{
	proc_create_single("mem_big_data", 0444, NULL, &membigdata_show);
	mdb_mem_check_init(expires_sec);
	return 0;
}

module_init(membigdata_init);
MODULE_LICENSE("GPL v2");
