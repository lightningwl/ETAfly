#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "sys.h"


/*
 * ����P����
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
 * PID����
 */
struct PID_PARAM_t
{
	float Kp;
	float Ki;
	float Kd;
	
	float e_threshold;	// ������ޣ����� e_threshold = 0.1�����ջὫ��������� ��0.1 ֮�䣬��ͬ
	float o_threshold;	// ����������
	float i_threshold;	// ��������
	float d_threshold;	// ��������	
};



/*
 * PID �ṹ��
 */
struct PID_t
{
	struct PID_PARAM_t param;
	
	float e_in;					// �����뷴�������
	float e_diff;				// �����
	float e_int;				// ������

	float desire;				// ����ֵ
	float measure;			// ����ֵ
	float output;				// ���ֵ
	float measure_last; // ��һ�����ֵ

	float dt;						// ʱ��������λ��s
};


void PidP_Update(struct PID_P_t *p);

void ClearPidInt(struct PID_t *pid);
void SetPidParam(struct PID_t *pid, const struct PID_PARAM_t *param);
void ResetPid(struct PID_t *pid, const struct PID_PARAM_t *param);
void PidUpdate(struct PID_t *pid);


#endif







