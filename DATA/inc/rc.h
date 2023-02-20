#ifndef RC_H
#define RC_H


#include "sys.h"
#include "uavmath.h"
#include "communicate.h"


#define RC_PITCH_MAX							25							// 最大目标俯仰角为 30°
#define RC_ROLL_MAX								25							// 最大滚转角 30°
#define RC_THR_MAX								200							// 油门最大PWM波
#define RC_THR_MIN								0								// 油门最小PWM波
#define RC_YAW_RATE_MAX						60							// 最大偏航角速度 deg/s
#define RC_YAW_RATE_LOCK  				5								// 偏航角速度小于该值，锁定偏航角
#define RC_VELX_MAX								3								// x方向最大速度 3m/s
#define RC_VELY_MAX								3								// y方向最大速度 3m/s
#define RC_VELZ_MAX								1								// z方向最大速度 1m/s




void RcCmd2Rc(const struct RC_CMD_t *cmd, struct RC_t *rc);
static void RcMapping(struct RC_t rc, struct RCM_t *rcm);

static void SetLockState(const struct RC_t rc, enum LOCKSTATE_e *lockstate);
static void SetFlyMode(const struct RC_t rc, enum FLYMODE_e *flymode);

void RcTask(void *arg);

struct RC_t GetRcApi(void);
struct RC_t *PointRcApi(void);
struct RCM_t GetRcmApi(void);
struct RCM_t *PointRcmApi(void);
enum FLYMODE_e GetFlyModeApi(void);
enum LOCKSTATE_e GetLockStateApi(void);

#endif


