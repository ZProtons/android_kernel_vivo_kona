#ifndef _VIVO_FORCE_H_
#define _VIVO_FORCE_H_

#include <linux/wait.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/fb.h>

#define VIVO_FORCE_VERSION 0x180210

#define VIVO_FORCE_INTERFACE_NODEFINE (-500)
#define VIVO_FORCE_RAW_DATA 0
#define VIVO_FORCE_DIF_DATA 1

extern int getForceLogSwitch(void);

#define VFE(fmt, param...) \
do {\
	printk(KERN_ERR "[%s]VFORCE_ERR^%d^"fmt, __func__, __LINE__, ##param);\
} while (0)

#define VFI(fmt, param...) \
do {\
	printk(KERN_ERR "[%s]VFORCE_INF^%d^"fmt, __func__, __LINE__, ##param);\
} while (0)

#define VFD(fmt, param...) \
do {\
	if (getForceLogSwitch()) {\
		printk(KERN_ERR "[%s]VFORCE_DBG^%d^"fmt, __func__, __LINE__, ##param);\
	} \
} while (0)

struct VForce {
	struct kobject kobjectDebug;
	struct i2c_client *client;
	struct input_dev *inputDev;
	struct mutex rawDifMutex;
	/* input lock */
	struct mutex inputReportMutex;
	/* iic and reset lock */
	struct mutex i2cResetMutex;
	struct mutex sensorRawDiffGetMutex;
	struct mutex rw_mutex;
	struct wakeup_source wakeLock;
	struct work_struct game_worker;
	struct work_struct switch_worker;
	void *privateData1;
	void *privateData2;
	atomic_t chipState;
	int lcdState;
	int iicErrorNum;
	int logSwitch;
	int isProbeComplete;
	int fwUpdatingFlag;
	//struct VtsFwManager vtsFwManager;
	
	unsigned char* (*getChipFwOrCfgVer)(void);
	int (*getHeaderFileFwOrCfgVer)(int fwOrCfg, unsigned char *fw);
	int (*getRawOrDiffData)(int which, char *buf);
	int (*idleEnableOrDisable)(int state);	/* force test */
	int (*otherInfo)(unsigned char *buf);
	
	int (*updateFirmware)(void);
	int (*getI2cBusState)(void);
	int (*keyIntSwitch)(unsigned char state);
	int (*getKeySwitchState)(void);
	int (*getKeyThreshold)(unsigned char *threshold);
	int (*changeKeyThreshold)(int which, int downup, int threshold);
	int (*changeCaliCoef)(int which, int coef);
	int (*getCaliCoef)(unsigned char *coef);
	int (*processByPackage)(unsigned char *app_name);
	int channelNum;
	int i2cCheckDelay;
	int gameMode;
	int lastIntFlag;
	int intSwitch;
	char service_buf[256];
	
	
};

extern struct VForce *vForceGetData(void);
extern struct VForce *vForceAlloc(void);
extern int vForceNodeCreate(struct VForce *vfd);
extern int vForceInit(struct VForce* vfd);
extern int vForceDeinit(void);
extern unsigned char* getChipFwOrCfgVer(void);
#endif
