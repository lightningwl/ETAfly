#ifndef CONTROL_H
#define CONTROL_H

#include "sys.h"
#include "atti_esti.h"



static void PidParamInit(void);
static void ClearAllPidInt(void);
static void SetAllPidParam(void);
static void AttiCtrl(struct EULAR_t atti_d, float dt);

static void MotorCommandUpdate(const struct XYZ_t moment, const float force);
static void RatePidUpdate(const struct XYZ_t rate_d, const float dt);
static void AttiPidUpdate(const struct EULAR_t atti_d, const float dt);
static void VelzPidUpdate(const float vzd, const float dt);
static void VelxyPidUpdate(const float vxd, const float vyd, const float dt);

void ControlTask(void *arg);


uint32_t GetControlTaskRunTimeApi(void);
struct PID_t *PointXratePidApi(void);
struct PID_P_t *PointRollPidApi(void);
struct PID_P_t *PointPitchPidApi(void);
struct PID_P_t *PointYawPidApi(void);
struct PID_t *PointVzPidApi(void);
struct PID_P_t *PointPyPidApi(void);
struct PID_t *PointVyPidApi(void);
struct PID_P_t *PointPxPidApi(void);
struct PID_t *PointVxPidApi(void);
struct PID_P_t *PointPzPidApi(void);
struct PID_t *PointYratePidApi(void);
struct PID_t *PointZratePidApi(void);

uint16_t *PointMotorDutyApi(void);

#endif
