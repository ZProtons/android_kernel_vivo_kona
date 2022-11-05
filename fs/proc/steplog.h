#ifndef __STEPLOG__H__
#define __STEPLOG__H__

/*
 * Step log:
 *
 * Step log format:
 *       stage1  | stage2  | step   | reserve | state
 * Bits: 31 - 28 | 27 - 20 | 19 - 8 | 7 - 2   | 1 - 0
 *
 * stage1(S1)(4 bits): Major stage, e.g. pbl, xbl, uefi, abl, kernel
 * stage2(S2)(8 bits): Minor stage, e.g. ddr init, flash init, load image
 * step(SP)(12 bits): The step of stage2 if need, e.g. load image1, load image2
 * reserve(RV)(6 bits): reservation
 * state(ST)(2 bits): pre or post action
 */

#define SL_STAGE1_SIZE      (1 << 4)
#define SL_STAGE1_MASK      (SL_STAGE1_SIZE - 1)
#define SL_STAGE1_SHIFT     28
#define SL_STAGE2_SIZE      (1 << 8)
#define SL_STAGE2_MASK      (SL_STAGE2_SIZE - 1)
#define SL_STAGE2_SHIFT     20
#define SL_STEP_SIZE        (1 << 12)
#define SL_STEP_MASK        (SL_STEP_SIZE - 1)
#define SL_STEP_SHIFT       8
#define SL_RESERVE_SIZE     (1 << 6)
#define SL_RESERVE_MASK     (SL_RESERVE_SIZE - 1)
#define SL_RESERVE_SHIFT    2
#define SL_STATE_SIZE       (1 << 2)
#define SL_STATE_MASK       (SL_STATE_SIZE - 1)
#define SL_STATE_SHIFT      0

#define STEP_LOG(stage1, stage2, step, state) \
	((((stage1) & SL_STAGE1_MASK) << SL_STAGE1_SHIFT) | \
	 (((stage2) & SL_STAGE2_MASK) << SL_STAGE2_SHIFT) | \
	 (((step) & SL_STEP_MASK) << SL_STEP_SHIFT) | \
	 (((state) & SL_STATE_MASK) << SL_STATE_SHIFT))

/* stage1 definition */
#define SL_S1_UNKNOWN               0
#define SL_S1_PBL                   1
#define SL_S1_XBL                   2
#define SL_S1_UEFI                  3
#define SL_S1_ABL                   4
#define SL_S1_KERNEL_BOOT           5
#define SL_S1_ANDROID_BOOT          6
#define SL_S1_KERNEL_SHUTDOWN       15

#define INVAILD_STAGE2(x)           ((unsigned int)(x) >= SL_STAGE2_SIZE)
#define INVAILD_STEP(x)             ((unsigned int)(x) >= SL_STEP_SIZE)
#define INVAILD_STATE(x)            ((unsigned int)(x) >= SL_STATE_SIZE)

#define INIT_START		             0
#define INIT_FIRST_STAGE1            1
#define INIT_FIRST_STAGE2            2
#define INIT_PROPERTY_INIT           3
#define INIT_SELINUX_INIT            4
#define INIT_EARLY_INIT              5
#define INIT_BOOT_COMPLETE           300

#define INIT_RB_START                4000
#define INIT_RB_SYNC                 4001
#define INIT_RB_SHUTDOWN_CRITICAL    4002
#define INIT_RB_TURNOFF_BACKLIGHT    4003
#define INIT_RB_BOOTANIM             4004
#define INIT_RB_SHUTDOWN_SERVICES    4005
#define INIT_RB_KILL_SERVICES        4006
#define INIT_RB_SHUTDOWN_VOID        4007
#define INIT_RB_KILL_AFTER_APP       4008
#define INIT_RB_UMOUNT               4009
#define INIT_RB_FSCK                 4010
#define INIT_RB_SYSCALL              4011

#ifdef VIVO_STEP_LOG
void save_step_log(unsigned int stage1, unsigned int stage2, unsigned int step, unsigned int state);
#else
static inline void save_step_log(unsigned int stage1, unsigned int stage2, unsigned int step, unsigned int state) {}
#endif

#endif
