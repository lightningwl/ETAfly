#ifndef RC_H
#define RC_H


#include "sys.h"
#include "uavmath.h"
#include "communicate.h"


#define RC_PITCH_MAX							25							// ���Ŀ�긩����Ϊ 30��
#define RC_ROLL_MAX								25							// ����ת�� 30��
#define RC_THR_MAX								200							// �������PWM��
#define RC_THR_MIN								0								// ������СPWM��
#define RC_YAW_RATE_MAX						60							// ���ƫ�����ٶ� deg/s
#define RC_YAW_RATE_LOCK  				5								// ƫ�����ٶ�С�ڸ�ֵ������ƫ����
#define RC_VELX_MAX								3								// x��������ٶ� 3m/s
#define RC_VELY_MAX								3								// y��������ٶ� 3m/s
#define RC_VELZ_MAX								1								// z��������ٶ� 1m/s




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


