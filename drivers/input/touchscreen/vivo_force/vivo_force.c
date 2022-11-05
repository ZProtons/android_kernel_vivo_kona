#include "vivo_force.h"
#include "vivo_force_node.h"

static struct VForce *vForceData;
struct VForce *vForceGetData(void)
{
	if (vForceData == NULL) {
		VFI("vForceData is NULL");
	}
	return vForceData;
}

struct VForce *vForceAlloc(void)
{
	struct VForce *vfd = NULL;
	vfd = (struct VForce *)kzalloc(sizeof(struct VForce), GFP_KERNEL);
	if (!vfd) {
		VFE("fail kzalloc vfd.");
		return NULL;
	}
	
	return vfd;
}

int vForceInit(struct VForce* vfd)
{
	int ret = 0;
	if (vfd == NULL) {
		VFE("vForceData is NULL");
		ret = -1;
		return ret;
	}
	
	vForceData = vfd;
	
	mutex_init(&(vForceData->rawDifMutex));
	mutex_init(&(vForceData->rw_mutex));
	wakeup_source_init(&(vForceData->wakeLock), "presskey");
	
	ret = vForceNodeCreate(vfd);
	
	return ret;
}

int vForceDeinit(void)
{
	int ret = 0;
	if (vForceData == NULL) {
		VFE("vForceData is NULL");
		ret = -1;
		return ret;
	}
	mutex_destroy(&(vForceData->rawDifMutex));
	mutex_destroy(&(vForceData->rw_mutex));
	vForceNodeRemove(vForceData);
	wakeup_source_trash(&(vForceData->wakeLock));
	kfree(vForceData);
	vForceData = NULL;
	return 0;
}

int getForceLogSwitch(void)
{
	if (vForceData == NULL) {
		return 1;
	} else {
		return vForceData->logSwitch;
	}
	
	return 0;
}

unsigned char* getChipFwOrCfgVer(void)
{
	if (vForceData == NULL) {
		return NULL;
	} else if (vForceData->getChipFwOrCfgVer == NULL) {
		return NULL;
	}
	
	return vForceData->getChipFwOrCfgVer();
}