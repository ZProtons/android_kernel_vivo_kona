#ifndef _LINUX_VIVO_HALL_H_
#define _LINUX_VIVO_HALL_H_

enum hall_type {
	RISING_HALL = 0,
	ROTATE_HALL
};

struct vivo_hall_dev {
	const char	*name;

	struct device	*dev;
	int		index;
	int		state;
	enum hall_type hall_type;
};

int vivo_hall_dev_register(struct vivo_hall_dev *dev);
void vivo_hall_dev_unregister(struct vivo_hall_dev *dev);

#endif