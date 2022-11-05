#include <linux/mmzone.h>
#include <linux/mm.h>
#include <linux/vmstat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/slab.h>
#include <linux/kmemleak.h>
#include <linux/ratelimit.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/page-isolation.h>
#include "highatomic_pool.h"

static struct kobject *highatomic_kobj;
#define HIGHATOMIC_POOL_BLOCK			1
static unsigned long highatomic_pool_pages = HIGHATOMIC_POOL_BLOCK;

int highatomic_pool_pageblock(struct zone *zone, struct page *page)
{
	int migratetype;

	migratetype = get_pageblock_migratetype(page);
	if (highatomic_pool_enabled(zone) &&
		is_highatomic_pool(migratetype))
		return 1;
	else
		return 0;
}

static int pageblock_is_unuseble(unsigned long start_pfn, unsigned long end_pfn)
{
	unsigned long pfn;
	int migratetype;

	for (pfn = start_pfn; pfn < end_pfn; pfn++) {
		migratetype = get_pageblock_migratetype(pfn_to_page(pfn));
		if (!pfn_valid_within(pfn) || PageReserved(pfn_to_page(pfn))
			/*||is_unmovable_isolate_sord(migratetype) ||is_unmovable_isolate_bord(migratetype)*/)
			return 1;
	}
	return 0;
}

#ifdef CONFIG_SMP
/*
 * flush the item stat, this function is
 * __mod_zone_page_state remove stat_threshold.
 */
static void update_zone_page_state(struct zone *zone, enum zone_stat_item item)
{
	struct per_cpu_pageset __percpu *pcp = zone->pageset;
	s8 __percpu *p = pcp->vm_stat_diff + item;
	long x;

	x = __this_cpu_read(*p);

	zone_page_state_add(x, zone, item);
	x = 0;
	__this_cpu_write(*p, x);
}
#else
/*
 * We do not maintain differentials in a single processor configuration.
 * The functions will do nothing.
 */
static void update_zone_page_state(struct zone *zone, enum zone_stat_item item)
{
	return;
}
#endif

/*
 * Mark a number of pageblocks as MIGRATE_HIGHATOMIC_POOL
 */
int init_highatomic_pool(struct zone *zone, int migrate_type)
{
	unsigned long start_pfn, pfn, end_pfn, block_end_pfn;
	struct page *page;
	int block_migratetype;
	unsigned long *zone_nr_block;
	long long block_count, old_block_count;
	int pages_moved = 0;

	if (is_highatomic_pool(migrate_type)) {
		zone_nr_block = &(zone->nr_highatomic_block);
	} else {
		pr_err("unknown migrate type!\n");
		return -EINVAL;
	}

	/* just setup the unmovable-isolate once when enable */
	if (!highatomic_pool_enabled(zone) || min_wmark_pages(zone) == 0)
		return -EPERM;

	if (highatomic_pool_enable)
		block_count = highatomic_pool_pages;
	else
		block_count = 0;

	old_block_count = *zone_nr_block;
	if (block_count == old_block_count)
		return 0;

	/*
	 * Get the start pfn, end pfn and the number of blocks to unmovable_isolate
	 * We have to be careful to be aligned to pageblock_nr_pages to
	 * make sure that we always check pfn_valid for the first page in
	 * the block.
	 */
	start_pfn = zone->zone_start_pfn;
	end_pfn = zone_end_pfn(zone);
	start_pfn = roundup(start_pfn, pageblock_nr_pages);

	for (pfn = start_pfn; pfn < end_pfn; pfn += pageblock_nr_pages) {
		if (!pfn_valid(pfn))
			continue;
		page = pfn_to_page(pfn);

		/* Watch out for overlapping nodes */
		if (page_to_nid(page) != zone_to_nid(zone))
			continue;

		block_migratetype = get_pageblock_migratetype(page);

		/* Only test what is necessary when the reserves are not met */
		if (block_count > 0) {
			/*
			 * Blocks with reserved pages will never free, skip
			 * them.
			 */
			block_end_pfn = min(pfn + pageblock_nr_pages, end_pfn);
			if (pageblock_is_unuseble(pfn, block_end_pfn))
				continue;

			/* If this block is migrate_type, account for it */
			if (block_migratetype == migrate_type) {
				block_count--;
				continue;
			}

			if (block_migratetype == MIGRATE_MOVABLE) {
				set_pageblock_migratetype(page,
							migrate_type);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
				pages_moved = move_freepages_block(zone, page,
							migrate_type, NULL);
#else
				pages_moved = move_freepages_block(zone, page,
							migrate_type);
#endif
				__mod_zone_page_state(zone, NR_FREE_HIGHATOMIC_POOL_PAGES, pages_moved);
				block_count--;
				continue;
			}
		} else {
			break;
		}
	}

	update_zone_page_state(zone, NR_FREE_HIGHATOMIC_POOL_PAGES);

	if (!block_count) {
		pr_info("init highatomic pool reserved %d block page for hard alloc successes",
			HIGHATOMIC_POOL_BLOCK);
		*zone_nr_block = HIGHATOMIC_POOL_BLOCK;
	} else {
		pr_err("init highatomic pool reserved %d block page for hard alloc failed",
			HIGHATOMIC_POOL_BLOCK);
	}

	return block_count;
}


static ssize_t pool_size_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", 
		highatomic_pool_pages*pageblock_nr_pages);
}

static ssize_t free_pages_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	unsigned long free_pages;

#ifdef CONFIG_HIGHATOMIC_POOL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		free_pages = global_zone_page_state(NR_FREE_HIGHATOMIC_POOL_PAGES);
#else
		free_pages = global_page_state(NR_FREE_HIGHATOMIC_POOL_PAGES);
#endif
#endif
	return snprintf(buf, PAGE_SIZE, "%lu\n", free_pages);
}

static ssize_t pool_enable_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", highatomic_pool_enable);
}

static ssize_t pool_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int ret;

	ret = kstrtouint(buf, 10, &highatomic_pool_enable);
	if (ret < 0)
		return ret;

	return count;
}

static struct kobj_attribute pool_size_attr =
	__ATTR(pool_size, 0444, pool_size_show, NULL);

static struct kobj_attribute free_pages_attr =
	__ATTR(free_pages, 0444, free_pages_show, NULL);

static struct kobj_attribute pool_enable_attr =
	__ATTR(enable, 0664, pool_enable_show, pool_enable_store);


static struct attribute *highatomic_pool_attrs[] = {
	&pool_size_attr.attr,
	&free_pages_attr.attr,
	&pool_enable_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name = "highatomic_pool",
	.attrs = highatomic_pool_attrs,
};

static int __init highatomic_pool_init(void)
{
	int ret;

	highatomic_kobj = kobject_create_and_add("kmem", kernel_kobj);
	if (!highatomic_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(highatomic_kobj, &attr_group);
	if (ret)
		kobject_put(highatomic_kobj);

	return ret;
}

static void __exit highatomic_pool_exit(void)
{
	sysfs_remove_group(highatomic_kobj, &attr_group);
	kobject_put(highatomic_kobj);
}

module_init(highatomic_pool_init);
module_exit(highatomic_pool_exit);
MODULE_LICENSE("GPL v2");

