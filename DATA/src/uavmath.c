#include <math.h>
#include "uavmath.h"
#include "sys.h"

/* 
 * 限制范围，单精度浮点			
 */
void LimitFloat(float *num, float nmin, float nmax)
{
	if (*num < nmin)
		*num = nmin;
	else if (*num > nmax)
		*num = nmax;
}


/* 
 * 限制范围，16位无符号整型		
 */
void LimitUint16(uint16_t *num, uint16_t nmin, uint16_t nmax)
{
	if (*num < nmin)
		*num = nmin;
	else if (*num > nmax)
		*num = nmax;
}

void LimitInt16(int16_t *num, int16_t nmin, int16_t nmax)
{
	if (*num < nmin)
		*num = nmin;
	else if (*num > nmax)
		*num = nmax;
}


/*
 * 限制增量范围
 */
float DiffLimitInc(const float xk, const float xk_1, const float max_value)
{
	float ret = 0.0f;
	float dx;
  dx = xk - xk_1;
	
	if (dx > max_value)
		ret = xk_1 + max_value;
	else if (dx < -max_value)
		ret = xk_1 - max_value;
	else
		ret = xk;
	
	return ret;
}

/*
 * 限制增量范围
 */
int16_t DiffLimitInt16Inc(const int16_t xk, const int16_t xk_1, const int16_t max_inc)
{
	int16_t ret = 0;
	int16_t dx;
  dx = xk - xk_1;
	
	if (dx > max_inc)
		ret = xk_1 + max_inc;
	else if (dx < -max_inc)
		ret = xk_1 - max_inc;
	else
		ret = xk;
	
	return ret;
}


/* 
 * 线性映射，模型：y = kx + b 							
 * 输入：[x_left, x_right]，起始区间范围					
 *		[y_left, y_right]，目标区间范围					
 * 输出：将 x 从起始区间线性映射到目标区间得到 y			
 */
float LinearMapping(const float x_left, const float x_right, const float y_left, const float y_right, const float x)
{
	float k, b;					
	float y;
	k = (y_right - y_left) / (x_right - x_left);
	b = y_left - k * x_left;
	y = k * x + b;
	
	return y;
}



/* 
 * 设置低通滤波器截止频率			
 */
void SetLpfCutoffFreq(struct LPF_t *f, float sample_freq, float cutoff_freq)
{

    float fr ;
    float ohm ;
    float c ;
    f->cutoff_frequency = cutoff_freq;
    fr = sample_freq / f->cutoff_frequency;
    ohm = tanf(PI/fr);
    c = 1.0f + 2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
    f->b0 = ohm*ohm/c;
    f->b1 = 2.0f*f->b0;
    f->b2 = f->b0;
    f->a1 = 2.0f*(ohm*ohm-1.0f)/c;
    f->a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
    f->delay_element_1 = f->delay_element_2 = 0;
}

/* 
 * 应用低通滤波器函数						
 */
float ApplyLpf(struct LPF_t *f,float sample)
{
    float output;
    float delay_element_0 = sample - f->delay_element_1 * f->a1 - f->delay_element_2 * f->a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) 
    {
        delay_element_0 = sample;
    }
    output = delay_element_0 * f->b0 + f->delay_element_1 * f->b1 + f->delay_element_2 * f->b2;

    f->delay_element_2 = f->delay_element_1;
    f->delay_element_1 = delay_element_0;

    return output;
}

/* 
 * 初始化均值滤波器
 */

void MfInit(struct MF_t *mf, float delta)
{
	mf->len = 0;
	mf->sum = 0;
	mf->mean = 0;
	mf->delta = delta;
}

float ApplyMf(struct MF_t *mf, float value)
{
	uint8_t i;
	
	if (mf->len < MF_MAX_LENGTH)
	{
		mf->sum += value;
		mf->len ++;
	}
	else
	{
		for (i=0; i<MF_MAX_LENGTH-1; i++)
			mf->value[i] = mf->value[i+1];
		mf->value[MF_MAX_LENGTH-1] = value;	
	}
	mf->mean = mf->sum / mf->len;
	return mf->mean;
}


/*
 * 字符串转化为双精度浮点数
 * 注意不要输入特殊符号（除小数点）
 */

double Str2Double(uint8_t *str, uint8_t len)
{
	double num = 0;							// 整数部分数值
	double dec = 0;							// 小数部分数值
	uint8_t k;
	uint8_t dot_label = len;
	
	for (k=0; k<len; k++)
	{
		if (str[k] == '\0')				// 字符串结束，break
			break;
		else if (str[k] == '.')		// 找到小数点，记下位置
			dot_label = k;
		
		if (str[0] != '-')					// 正数
		{
			if (k < dot_label)
			{
					num = 10 * num + (str[k] - '0');// 整数部分数值
			}
			else if (k>dot_label)
			{
					dec = dec + powf(0.1, k - dot_label) * (str[k] - '0');// 小数部分数值
			}
		}
		else 												// 负数
		{
			if (k==0)
				continue;
			else if (k < dot_label)
			{
					num = 10 * num + (str[k] - '0');// 整数部分数值
			}
			else if (k>dot_label)
			{
					dec = dec + powf(0.1, k - dot_label) * (str[k] - '0');// 小数部分数值
			}
		}
		
	}
	if (str[0] != '-')					// 正数
		return (num+dec);
	else
		return -(num+dec);
}
/*
 * 字符串转化为单精度浮点数
 * 注意不要输入特殊符号（除小数点）
 * 注意输入最大最小数数量级不要超过8！ 1234.5678 合理， 12345.6789 不合理
 */
float Str2Float(char *str, uint8_t len)
{
	uint32_t num = 0;
  uint32_t abit = 0;	
	
	float dec = 0;							// 小数部分数值
	uint8_t k;
	uint8_t dot_label = len;
	
	for (k=0; k<len; k++)
	{
		if (str[k] == '\0')
			break;
		else if (str[k] == '.')
			dot_label = k;
		
		
		if (k < dot_label)
		{
				num = 10 * num + (str[k] - '0'); // 整数部分数值
		}
		else if (k>dot_label)
		{
			  dec = dec + powf(0.1, k - dot_label) * (str[k] - '0');	// 小数部分数值
		}
	}
		
	for (k=0; k<dot_label; k++)
	{
		abit = (num % powi(10, k+1));
		num = num - abit;
		dec += (float)abit;	
	}
	return dec;	
}

/*
 * 整数指数运算
 */
uint32_t powi(uint32_t a, uint8_t b)
{
	uint8_t k;
	uint32_t ret = 1;
	
	for (k=0; k<b; k++)
		ret *= a;
	
	return ret;
}


/*
 * 差分限幅
 * xk -- 此次测量值
 * xk_1 -- 上一次测量值
 *
 * if (xk - xk_1)大于 err_vel, 认为数据错误，返回 xk_1
 * elseif 数据大于 max_vel, 限制两次差异，返回 xk_1 + max_vel
 * else 返回 xk
 * 小于 -err_vel, -max_vel 类似
 */
float DiffLimit(const float xk, const float xk_1, const float max_value, const float err_value)
{
	float ret = 0.0f;
	float dx;
  dx = xk - xk_1;
	
	if (dx > err_value)
		ret = xk_1;
	else if (dx > max_value)
		ret = xk_1 + max_value;
	else if (dx < -err_value)
		ret = xk_1;
	else if (dx < -max_value)
		ret = xk_1 - max_value;
	else
		ret = xk;
	
	return ret;
}

void DiffLimit2(const float xk, float *xk_1, const float max_value, const float err_value)
{
	float dx;
  dx = xk - *xk_1;
	
	if (dx > err_value || dx < -err_value)
		return;
	else if (dx > max_value)
		*xk_1 = *xk_1 + max_value;
	else if (dx < -max_value)
		*xk_1 = *xk_1 - max_value;
	else
		*xk_1 = xk;
}

/*
 * 角度转化为弧度
 */
void Dof3Deg2Rad(struct XYZ_t *dof3)
{
	dof3->x *= DEG2RAD;
	dof3->y *= DEG2RAD;
	dof3->z *= DEG2RAD;
}

/*
 * 弧度转化为角度
 */
void Dof3Rad2Deg(struct XYZ_t *dof3)
{
	dof3->x *= RAD2DEG;
	dof3->y *= RAD2DEG;
	dof3->z *= RAD2DEG;
}

/*
 * 求均值
 */
float Mean(float *data, uint8_t len)
{
	uint8_t k;
	float mean = 0;
	
	if (len == 0)
		return 0.0f;
	
	for (k=0; k<len; k++)
	{
		mean += data[k];
	}
	
	mean /= len;
	
	return mean;
}


/*
 * 求方差
 * 注意：使用无偏估计，分母为 (n-1) 而不是 n
 */
float Var(float *data, uint8_t len)
{
	uint8_t k;
	float mean;
	float var = 0;
	
	if (len <= 1)
		return 0.0f;
	
	mean = Mean(data, len);
	
	for (k=0; k<len; k++)
	{
		var += (data[k] - mean) * (data[k] - mean);
	}
	
	var /= (len - 1);
	
	return var;
}

/*
 * 求标准差 
 */
float Std(float *data, uint8_t len)
{
	return sqrtf(Var(data, len));
}

/*
 * 四元数模长归一化
 */
void NormQuat(struct QUAT_t *quat)
{
	float norm = sqrtf(	quat->q0 * quat->q0 
										+ quat->q1 * quat->q1 
										+ quat->q2 * quat->q2
										+ quat->q3 * quat->q3);
	
	norm = 1.0f / norm;
	
	quat->q0 *= norm;
	quat->q1 *= norm;
	quat->q2 *= norm;
	quat->q3 *= norm;
}

/*
 * 四元数转欧拉角(旋转顺序XYZ，单位：deg)
 */
struct EULAR_t Quat2Eular(struct QUAT_t quat)
{
	struct EULAR_t eular;
	
	NormQuat(&quat);
	
	eular.roll  = RAD2DEG * atan2f(2.0f*(quat.q2*quat.q3+quat.q0*quat.q1), quat.q0*quat.q0-quat.q1*quat.q1-quat.q2*quat.q2+quat.q3*quat.q3);
	eular.pitch = RAD2DEG * asinf(2.0f*(quat.q0*quat.q2-quat.q1*quat.q3) );
	eular.yaw   = RAD2DEG * atan2f(2.0f*(quat.q1*quat.q2+quat.q0*quat.q3), (quat.q0*quat.q0+quat.q1*quat.q1-quat.q2*quat.q2-quat.q3*quat.q3));
	
	return eular;
}

/*
 * 欧拉角(旋转顺序XYZ，单位：deg)转四元数（模为1）
 */
struct QUAT_t Eular2Quat(const struct EULAR_t eular)
{
	struct EULAR_t c;
	struct EULAR_t s;
	struct QUAT_t quat;
	
	c.roll = cosf(eular.roll * DEG2RAD * 0.5f);
	c.pitch = cosf(eular.pitch * DEG2RAD * 0.5f);
	c.yaw = cosf(eular.yaw * DEG2RAD * 0.5f);
	
	s.roll = sinf(eular.roll * DEG2RAD * 0.5f);
	s.pitch = sinf(eular.pitch * DEG2RAD * 0.5f);
	s.yaw = sinf(eular.yaw * DEG2RAD * 0.5f);	

	quat.q0 = c.roll * c.pitch * c.yaw + s.roll * s.pitch * s.yaw;	
	quat.q1 = s.roll * c.pitch * c.yaw - c.roll * s.pitch * s.yaw;
	quat.q2 = c.roll * s.pitch * c.yaw + s.roll * c.pitch * s.yaw;
	quat.q3 = c.roll * c.pitch * s.yaw - s.roll * s.pitch * c.yaw;

	NormQuat(&quat);
	
	return quat;
}

/*
 * 取符号函数
 */
float Sign(float x)
{
	if (x > 1e-6f)
		return 1.0f;
	else if (x < -1e-6f)
		return -1.0f;
	else
		return 0.0f;
}

/*
 * 三维向量模长
 */
float Norm3(const struct XYZ_t *vec)
{
	return sqrtf(vec->x * vec->x   +   vec->y * vec->y    +    vec->z * vec->z);
}



/*
 * 单位四元数转旋转矩阵
 */
void Quat2Rbe(const struct QUAT_t *quat, float Rbe[3][3])
{
	Rbe[0][0] = 1-2*(quat->q2*quat->q2 + quat->q3*quat->q3);					Rbe[0][1] = 2*(quat->q1*quat->q2 		- quat->q0*quat->q3);				Rbe[0][2] = 2*(quat->q1*quat->q3 		+ quat->q0*quat->q2);
	Rbe[1][0] = 2*(quat->q1*quat->q2 	 + quat->q0*quat->q3);					Rbe[1][1] = 1-2*(quat->q1*quat->q1 	+ quat->q3*quat->q3);				Rbe[1][2] = 2*(quat->q2*quat->q3		- quat->q0*quat->q1);
	Rbe[2][0] = 2*(quat->q1*quat->q3   - quat->q0*quat->q2);					Rbe[2][1] = 2*(quat->q2*quat->q3 		+ quat->q0*quat->q1);				Rbe[2][2] = 1-2*(quat->q1*quat->q1	+ quat->q2*quat->q2);	
}

void Eular2Rbe(const struct EULAR_t *eular, float Rbe[3][3])
{
	float c_pitch = cosf(eular->pitch * DEG2RAD);
	float s_pitch = sinf(eular->pitch * DEG2RAD);
	float c_roll = cosf(eular->roll * DEG2RAD);
	float s_roll = sinf(eular->roll * DEG2RAD);
	float c_yaw = cosf(eular->yaw * DEG2RAD);
	float s_yaw = sinf(eular->yaw * DEG2RAD);
	
	Rbe[0][0] = c_pitch * c_yaw;					Rbe[0][1] = c_yaw * s_pitch * s_roll - s_yaw * c_roll;				Rbe[0][2] = c_yaw * s_pitch * c_roll + s_yaw * s_roll;
	Rbe[1][0] = c_pitch * s_yaw;					Rbe[1][1] = s_yaw * s_pitch * s_roll + c_yaw * c_roll;				Rbe[1][2] = s_yaw * s_pitch * c_roll - c_yaw * s_roll;
	Rbe[2][0] = -s_pitch;									Rbe[2][1] = s_roll * c_pitch;																	Rbe[2][2] = c_roll * c_pitch;	
}


/*
 * 去除死区函数
 */
void RemoveDeadBand(float *input, float deadband)
{
	if (*input > deadband)
		*input -= deadband;
	else if (*input < -deadband)
		*input += deadband;
	else
		*input = 0.0f;
}

/*
 * 去除三轴死区函数
 */
void RemoveDeadBandVector3(struct XYZ_t *input, float deadband)
{
	RemoveDeadBand(&input->x, deadband);
	RemoveDeadBand(&input->y, deadband);
	RemoveDeadBand(&input->z, deadband);
}

