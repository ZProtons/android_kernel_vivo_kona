#ifndef __HAPTIC_CORE_H__
#define __HAPTIC_CORE_H__

#define HAPTIC_IOCTL_MAGIC                'h'

#define HAPTIC_UPLOAD                     _IOWR(HAPTIC_IOCTL_MAGIC, 11, struct haptic_effect*)
#define HAPTIC_PLAYBACK                   _IOWR(HAPTIC_IOCTL_MAGIC, 12, int)
#define HAPTIC_STOP                       _IOWR(HAPTIC_IOCTL_MAGIC, 13, int)
#define HAPTIC_GAIN                       _IOWR(HAPTIC_IOCTL_MAGIC, 14, int)
#define HAPTIC_TRIGGER_INTENSITY          _IOW(HAPTIC_IOCTL_MAGIC, 15, int)

#define HAPTIC_SUPPORT_BITMASK            _IOWR(HAPTIC_IOCTL_MAGIC, 16, unsigned char *)


/* support bit mask*/
#define HAPTIC_MASK_BIT_SUPPORT_EFFECT    0x00
#define HAPTIC_MASK_BIT_SUPPORT_GAIN      0x01
#define HAPTIC_MASK_BIT_TRIGGER_INTENSITY 0x07

#define HAPTIC_CNT                        0x08

enum haptic_effect_type {
	HAPTIC_CONSTANT,
	HAPTIC_CUSTOM
};

enum haptic_write_event_type {
	HAPTIC_PLAY_EVENT,
	HAPTIC_GAIN_EVENT,
};

struct haptic_write_event {
	__s16           type;
	__s16           code;
	__s32           value;
};

struct haptic_effect {
	enum haptic_effect_type      type;
	__s16                 magnitude;
	__s16                 length;
	__s16                 id;
	__s16 __user         *custom_data;

};


struct haptic_device {
	const char *name;

	int (*upload)(struct haptic_device *hp, struct haptic_effect *effect);
	int (*erase)(struct haptic_device *hp);
	int (*playback)(struct haptic_device *hp, int value);
	void (*set_gain)(struct haptic_device *hp, u16 gain);
	void (*set_trigger_intensity)(struct haptic_device *hp, int gain);

	void (*init_dev)(struct haptic_device *hp);
	unsigned long hap_bit[BITS_TO_LONGS(HAPTIC_CNT)];
	struct device *dev; //指向probe中的dev，做绑定作用
	void *chip; //指向芯片信息结构体
	void *private;

};

struct haptic_misc {
	struct miscdevice     hap_dev;
	struct mutex          lock;
	dev_t                 devt;
	int                   open; //打开设备节点的次数
//	bool                  exist;
	struct list_head      list;
	void                  *private_data;
	int                   max_effects;
	struct file           *effect_owners[];
};


extern int haptic_miscdev_register(const char *name, struct haptic_device *hp);
extern int haptic_miscdev_unregister(struct haptic_device *hp);

#endif

