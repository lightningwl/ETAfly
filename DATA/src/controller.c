#include "controller.h"
#include "uavmath.h"


/************************************* P控制 *********************************************/
/*
 * PID 单P控制
 */
void PidP_Update(struct PID_P_t *p)
{
	p->output = p->Kp * (p->desire - p->measure);
	LimitFloat(&p->output, -p->o_threshold, p->o_threshold);
}


/************************************* PID控制 *********************************************/


/*
 * 清除PID的积分
 */
void ClearPidInt(struct PID_t *pid)
{
	pid->e_int = 0.0f;
}


/*
 * 设置PID参数
 */
void SetPidParam(struct PID_t *pid, const struct PID_PARAM_t *param)
{
	pid->param.Kp = param->Kp;
	pid->param.Ki = param->Ki;
	pid->param.Kd = param->Kd;
	
	pid->param.e_threshold = param->e_threshold;
	pid->param.o_threshold = param->o_threshold;
	pid->param.i_threshold = param->i_threshold;
	pid->param.d_threshold = param->d_threshold;
}

/*
 * PID复位
 */
void ResetPid(struct PID_t *pid, const struct PID_PARAM_t *param)
{
	SetPidParam(pid, param);
	
	pid->desire 				= 0.0f;
	pid->dt							= 0.0f;
	pid->e_diff 				= 0.0f;
	pid->e_in 					= 0.0f;
	pid->e_int 					= 0.0f;
	pid->measure 				= 0.0f;
	pid->measure_last 	= 0.0f;
	pid->output 				= 0.0f;
}


/*
 * PID更新
 */
void PidUpdate(struct PID_t *pid)
{
	LimitFloat(&pid->dt, 0.001f, 0.01f);											
	
	pid->e_in = pid->desire - pid->measure;
	LimitFloat(&pid->e_in, -pid->param.e_threshold, pid->param.e_threshold);	
	
	pid->e_int += pid->e_in * pid->dt;
	LimitFloat(&pid->e_int, -pid->param.i_threshold, pid->param.i_threshold);				// 误差积分限幅
	
	pid->e_diff = 0.9f * pid->e_diff + 0.1f * (pid->measure - pid->measure_last) / (pid->dt);
	LimitFloat(&pid->e_diff, -pid->param.d_threshold, pid->param.d_threshold);	
	
	pid->output = pid->param.Kp * (pid->e_in 
					+ pid->param.Ki * pid->e_int 
					- pid->param.Kd * pid->e_diff);
	LimitFloat(&pid->output, -pid->param.o_threshold, pid->param.o_threshold);				// 输出限幅
	
	/* 更新测量结果 */
	pid->measure_last = pid->measure;
}

