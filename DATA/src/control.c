#include "control.h"
#include "uavmath.h"
#include "dof10.h"
#include "motor.h"
#include "atti_esti.h"
#include "controller.h"
#include "communicate.h"
#include "nrf.h"
#include "pos_esti.h"
#include "rc.h"
#include "FreeRTOS.h"
#include "task.h"


const float basic_thr = 90.0f;
const float roll_pitch_bias[2] = {0.0f, 0.0f};	// 四个螺旋桨水平时，测得的滚转角与俯仰角 (deg)
/* 
 * PID控制器参数
 * Kp  Ki  Kd
 * e_threshold   o_threshold   i_threshold   d_threshold
 */
static struct PID_PARAM_t param_vz	  = {36.0f, 0.05f, 0.00f,   2.0f, 50.0f, 20.0f, 	10.0};
 
static struct PID_PARAM_t param_xrate = {0.15f, 0.2f, 0.08f,		300.0f, 100.0f, 10.0f, 30.0f}; 
static struct PID_PARAM_t param_yrate = {0.15f, 0.2f, 0.08f, 		300.0f, 100.0f, 10.0f, 30.0f}; 
static struct PID_PARAM_t param_zrate = {0.6f, 0.5f, 0.00f, 		300.0f, 200.0f, 10.0f, 60.0f}; 


uint16_t duty_[4];

/*
 * Kp, desire, measure, output, o_threshold;
 */
struct PID_P_t pid_pz_		= {0.8f, 0.0f, 0.0f, 0.0f, 2.0f};					// 高度 P 控制
struct PID_t pid_vz_;																								// z轴速度 PID 控制

struct PID_P_t pid_roll_  = {7.0f, 0.0f, 0.0f, 0.0f, 300.0f};				// 姿态 P 控制
struct PID_P_t pid_pitch_ = {7.0f, 0.0f, 0.0f, 0.0f, 300.0f};
struct PID_P_t pid_yaw_   = {3.0f, 0.0f, 0.0f, 0.0f, 300.0f};

struct PID_t pid_xrate_;																						// 角速度 PID 控制
struct PID_t pid_yrate_;
struct PID_t pid_zrate_;

/*
 * 控制任务
 */

void ControlTask(void *arg)
{
	portTickType last_t;
	static uint16_t k = 0;

	struct EULAR_t eular_d;
	struct XYZ_t rate_d;
	struct XYZ_t moment = {0, 0, 0};
	float force;
	struct RCM_t *rcm = PointRcmApi();
	
	float cos_roll, cos_pitch;
	float cos_roll_pitch;
	float yaw_lock = 0.0;	// 需要初始化获取yaw，海拔等
	
	uint8_t ctrl_state = 0;
	enum FLYMODE_e flymode = FLY_STABILIZE;

	float vzd = 0.0f;	
	float posz_hold = 0;
	struct XYZ_t *pose;
	
	
	while (GetInitEularApi().valid != 1)	// 等待姿态角有效
	{
		vTaskDelay(1000);
		printf("Cannot Get Initial Attitude, control attitude is rejected\r\n");
	}
	yaw_lock = GetInitEularApi().yaw;			// 获取初始偏航角
	
	last_t = xTaskGetTickCount();
		
	while (1)
	{
		vTaskDelayUntil(&last_t, 1);
		
		switch (ctrl_state)
		{
			case 0:
				SetMotorsStopApi();												// 电机停转				
				PidParamInit();														// PID复位		
				ClearAllPidInt();													// 清除所有积分
			
				if (GetInitEularApi().valid == 1 && GetLockStateApi() == FLY_UNLOCK)
					ctrl_state = 1;
				break;
			
			case 1:
					SetFourMotorsSpinSlowly();										// 电机缓缓转动	
					ctrl_state = 2;		
					
				break;
		
			case 2:
				if (GetLockStateApi() == FLY_LOCKED)
				{
					ctrl_state = 0;
					break;
				}
							
				rcm = PointRcmApi();
				flymode = GetFlyModeApi();
				pose = PointFusePosApi();				
				
				
				/* 1. 角速度控制 333Hz */
				if (k % 3 == 0)
				{
					cos_roll = cosf(pid_roll_.desire * DEG2RAD);
					cos_pitch = cosf(pid_pitch_.desire * DEG2RAD);
					cos_roll_pitch = sqrtf(cos_roll * cos_pitch);
					
					rate_d.x = pid_roll_.output;
					rate_d.y = cos_roll * pid_pitch_.output;
					
					if (fabs(rcm->yaw_rate) < 0.5f*RC_YAW_RATE_MAX)		// 摇杆在中间附近
						rate_d.z = cos_pitch * cos_roll * pid_yaw_.output;
					else
						rate_d.z = rcm->yaw_rate;												// 遥控器映射的偏航角速度
					
					RatePidUpdate(rate_d, 0.003f);
					
					
					/* 力/力矩控制 */
					moment.x = pid_xrate_.output;
					moment.y = pid_yrate_.output;
					moment.z = pid_zrate_.output;
										

					if (cos_roll_pitch < 0.866f)
						cos_roll_pitch = 0.866f;
					
					if (flymode != FLY_STABILIZE)
						force = (-pid_vz_.output + basic_thr) / cos_roll_pitch;			// z轴控制器输出。非线性补偿，避免姿态变化时竖直方向升力不足
					else	
						force = rcm->thr;																// 遥控器映射的油门

					MotorCommandUpdate(moment, force);								// 控制电机转动
				}
				
				/* 2. 角度控制 200Hz*/
				if (k % 5 == 0)
				{
					eular_d.roll = rcm->roll;
					eular_d.pitch = rcm->pitch;
			
					if (fabs(rcm->yaw_rate) < 0.6f*RC_YAW_RATE_MAX)	// 摇杆在中间附近
						eular_d.yaw = yaw_lock;													// 锁定偏航角		
					else
						yaw_lock = pid_yaw_.measure;										// 更新偏航角
					
					AttiPidUpdate(eular_d, 0.005f);							
				}				
				
				/* 3. 速度控制 100Hz*/
				if (flymode != FLY_STABILIZE && k % 10 == 0)
				{
					// z轴速度控制 
					VelzPidUpdate(vzd, 0.01f);
				}
				
				/* 4. 位置控制 50Hz*/
				if (flymode != FLY_STABILIZE && k % 20 == 0)
				{
					// z轴位置控制
					if (fabs(rcm->vz) < 0.5f * RC_VELZ_MAX)
					{
						pid_pz_.desire = posz_hold;
						pid_pz_.measure = pose->z;
						PidP_Update(&pid_pz_);
						
						vzd = pid_pz_.output;
					}
					else 
					{
						vzd = rcm->vz;
						posz_hold = pose->z;
					}

				}
				
				/* 5. 更新悬停位置为当前位置	*/
				if (flymode == FLY_STABILIZE && k % 10 == 0)
				{					
					posz_hold = pose->z;						
				}
				
				k ++;
				k %= 100;
						
				break;
		}
	}
}


/*
 * 重置所有PID
 */
static void PidParamInit(void)
{
	ResetPid(&pid_vz_,    &param_vz);
	
	ResetPid(&pid_xrate_, &param_xrate);
	ResetPid(&pid_yrate_, &param_yrate);
	ResetPid(&pid_zrate_, &param_zrate);
}


/*
 * 清除所有PID的积分
 */
static void ClearAllPidInt(void)
{
	pid_vz_.e_int     = 0.0f;
	
	pid_xrate_.e_int  = 0.0f;
	pid_yrate_.e_int  = 0.0f;
	pid_zrate_.e_int  = 0.0f;
}

/*
 * 速度环Z方向PID更新
 */
static void VelzPidUpdate(const float vzd, const float dt)
{	
	pid_vz_.desire = vzd;
	pid_vz_.dt = dt;
	pid_vz_.measure =  GetFuseVelApi().z;

	PidUpdate(&pid_vz_);
}


/*
 * 姿态控制
 */
static void AttiPidUpdate(const struct EULAR_t atti_d, const float dt)
{
	struct EULAR_t atti_m = GetAttiEularApi();


					
	pid_roll_.desire = atti_d.roll;
	pid_roll_.measure = atti_m.roll - roll_pitch_bias[0];
	PidP_Update(&pid_roll_);
	
	pid_pitch_.desire = atti_d.pitch;
	pid_pitch_.measure = atti_m.pitch - roll_pitch_bias[1];
	PidP_Update(&pid_pitch_);	
	
	pid_yaw_.desire = atti_d.yaw;
	pid_yaw_.measure = atti_m.yaw;
	
	if (pid_yaw_.desire - pid_yaw_.measure > 180.0f)				// 避免±180°不连续问题
		pid_yaw_.desire = pid_yaw_.desire - 360.0f;
	else if (pid_yaw_.desire - pid_yaw_.measure < -180.0f)
		pid_yaw_.measure = pid_yaw_.measure - 360.0f;
				
					
	PidP_Update(&pid_yaw_);		
}



/*
 * 角速度控制
 */
static void RatePidUpdate(const struct XYZ_t rate_d, const float dt)
{	
	struct XYZ_t rate_m = GetDof6LpfDataApi().gyro;
	
	pid_xrate_.desire  	= rate_d.x; 					
	pid_xrate_.measure 	= rate_m.x;
	pid_xrate_.dt 			= dt;
	PidUpdate(&pid_xrate_);
	
	pid_yrate_.desire 	= rate_d.y;
	pid_yrate_.measure 	= rate_m.y;
	pid_yrate_.dt	 			= dt;
	PidUpdate(&pid_yrate_);
	
	
	pid_zrate_.desire		= rate_d.z;
	pid_zrate_.measure 	= rate_m.z;
	pid_zrate_.dt 			= dt;
	PidUpdate(&pid_zrate_);
}

/*
 * 控制分配及控制电机转动
 */
static void MotorCommandUpdate(const struct XYZ_t moment, const float force)
{	
	duty_[0] = (uint16_t)(-moment.x + moment.y + moment.z + force);	// 分配电机1
	duty_[1] = (uint16_t)( moment.x + moment.y - moment.z + force);	// 分配电机2 
	duty_[2] = (uint16_t)( moment.x - moment.y + moment.z + force);	// 分配电机3
	duty_[3] = (uint16_t)(-moment.x - moment.y - moment.z + force);	// 分配电机4	
	
	SetMotorDutyApi(MOTOR1, duty_[0]);			// 调用API，设置电机1占空比
	SetMotorDutyApi(MOTOR2, duty_[1]);			// 调用API，设置电机2占空比
	SetMotorDutyApi(MOTOR3, duty_[2]);			// 调用API，设置电机3占空比
	SetMotorDutyApi(MOTOR4, duty_[3]);			// 调用API，设置电机4占空比		
}


/* 
 * API接口
 */
struct PID_t *PointXratePidApi(void)
{
	return &pid_xrate_;
}

struct PID_t *PointYratePidApi(void)
{
	return &pid_yrate_;
}

struct PID_t *PointZratePidApi(void)
{
	return &pid_zrate_;
}


struct PID_P_t *PointRollPidApi(void)
{
	return &pid_roll_;
}

struct PID_P_t *PointPitchPidApi(void)
{
	return &pid_pitch_;
}

struct PID_P_t *PointYawPidApi(void)
{
	return &pid_yaw_;
}

struct PID_P_t *PointPzPidApi(void)
{
	return &pid_pz_;
}


struct PID_t *PointVzPidApi(void)
{
	return &pid_vz_;
}

uint16_t *PointMotorDutyApi(void)
{
	return &duty_[0];
}

