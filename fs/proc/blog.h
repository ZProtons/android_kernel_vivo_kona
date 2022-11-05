#ifndef __BLOG_H__
#define __BLOG_H__

#include <linux/types.h>

/*
 * blog:
 *
 * +==============================
 * | blog_header
 * +==============================
 * | sbl log (32KB)
 * +-----------------------------
 * | uefi log (128KB)
 * +-----------------------------
 * | kernel_step (1MB)
 * +-----------------------------
 * | android step (1MB)
 * +==============================
 * | sbl log (32KB)
 * +-----------------------------
 * | ...
 */

#define BLOG_PARTITION_OFFSET       0x1000

#define BLOG_MAGIC_NUM              0x11061793
#define BLOG_VERSION                0
#define BLOG_NUM_MAX                10

#define BLOG_SBL                    0
#define BLOG_UEFI                   1
#define BLOG_KERNEL                 2
#define BLOG_ARD                    3
#define BLOG_REC_MAX                4

#define BLOG_NO_FLUSH               0
#define BLOG_LINE_FLUSH             1
#define BLOG_FINAL_FLUSH            2

#define BLOG_NORMAL_REBOOT          0xF

#define BLOG_CTL_PAUSE              (1 << 0)
#define BLOG_CTL_RESUME             (1 << 1)
#define BLOG_CTL_HIGH_FREQ_CHK      (1 << 31)

#define BLOG_SBL_LOG_BYTE_SIZE      (32 * 1024)
#define BLOG_UEFI_LOG_BYTE_SIZE     (128 * 1024)
#define BLOG_KERNEL_STEP_BYTE_SIZE  (1 * 1024 * 1024)
#define BLOG_USER_STEP_BYTE_SIZE    (1 * 1024 * 1024)

/*
 * partial ring buffer:
 * |    non-ring    |      ring       |
 * |----------------|-----------------|
 * start            ring offset       end
 */
struct blog_desc {
	u32 start;          // BYTE
	u32 size;           // BYTE
	u32 ring_offset;    // BYTE, partial ring offset
	u32 wptr;           // BYTE
};

struct blog_info {
	u32 count;
	u32 boot_complete;
	struct blog_desc stage[BLOG_REC_MAX];
};

struct blog_header {
	u32 magic;
	u32 version;
	u32 crc;
	u32 count;
	u32 flush_mode;
	u32 control;
	struct blog_info log[BLOG_NUM_MAX];
};

#endif
