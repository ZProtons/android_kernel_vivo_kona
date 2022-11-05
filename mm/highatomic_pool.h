/* 
 * mm/highatomic_pool.h
 */

#ifndef HIGHATOMIC_POOL_H
#define HIGHATOMIC_POOL_H

#ifdef CONFIG_HIGHATOMIC_POOL
#include <linux/mmzone.h>

static int __read_mostly highatomic_pool_enable = 1;

extern int highatomic_pool_pageblock(struct zone *zone, struct page *page);
extern int init_highatomic_pool(struct zone *zone, int migrate_type);

static inline int dma_zone(struct zone *zone)
{
	if (zone != NULL && zone_idx(zone) == ZONE_NORMAL)
		return 1;
	else
		return 0;
}

static inline int highatomic_pool_enabled (struct zone *zone)
{
	if (highatomic_pool_enable && dma_zone(zone))
		return 1;
	else
		return 0;
}

static inline void __mod_zone_page_state_internel(struct zone *zone,
	enum zone_stat_item item, long delta)
{
	return __mod_zone_page_state(zone, item, delta);
}

#else /*CONFIG_HIGHATOMIC_POOL*/
static inline int highatomic_pool_enabled (struct zone *zone) {return 0};
static inline int highatomic_pool_pageblock(struct zone *zone, struct page *page) {return 0};

static inline void __mod_zone_page_state_internel(struct zone *zone,
	enum zone_stat_item item, long delta) {}

#endif /*CONFIG_HIGHATOMIC_POOL*/

#endif
