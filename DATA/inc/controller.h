#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "sys.h"


/*
 * 单纯P控制
 */
struct PID_P_t
{
	float Kp;
	float desire;
	float measure;
	float output;
	float o_threshold;
};


/*
 * PID参数
 */
struct PID_PARAM_t
{
	float Kp;
	float Ki;
	float Kd;
	
	float e_threshold;	// 误差门限，例如 e_threshold = 0.1，最终会将误差限制于 ±0.1 之间，下同
	float o_threshold;	// 输出结果门限
	float i_threshold;	// 积分门限
	float d_threshold;	// 积分门限	
};



/*
 * PID 结构体
 */
struct PID_t
{
	struct PID_PARAM_t param;
	
	float e_in;					// 期望与反馈的误差
	float e_diff;				// 误差差分
	float e_int;				// 误差积分

	float desire;				// 期望值
	float measure;			// 测量值
	float output;				// 输出值
	float measure_last; // 上一次输出值

	float dt;						// 时间间隔，单位：s
};


void PidP_Update(struct PID_P_t *p);

void ClearPidInt(struct PID_t *pid);
void SetPidParam(struct PID_t *pid, const struct PID_PARAM_t *param);
void ResetPid(struct PID_t *pid, const struct PID_PARAM_t *param);
void PidUpdate(struct PID_t *pid);


#endif







