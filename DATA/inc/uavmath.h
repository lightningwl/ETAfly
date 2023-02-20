#ifndef UAVMATH_H
#define UAVMATH_H

#include <math.h>
#include "sys.h"
#include "arm_math.h"

#define GRAVITY						9.8f							// 加速度大小

#if USING_GPS == 1
	#define EARTH_RADIUS		6378137.0					// 地球半径
#else
	#define EARTH_RADIUS		6371000.0					// 地球半径
#endif

#define RAD2DEG   				57.295779513082323f
#define DEG2RAD   				0.017453292519943f

#define UWB_LAT_START 		22.3061285			//使用uwb定位，原点A0 纬度 
#define UWB_LONG_START 		113.5404365 		//使用uwb定位，原点A0 经度

#ifndef true
	#define true 1
#endif

#ifndef false
	#define false 0
#endif

	

enum FLYMODE_e
{
	FLY_STABILIZE = 0,		// 自稳模式 stabilize
	FLY_ALTHOLD = 1,			// 定高模式 altitude hold
	FLY_LOITER = 2,				// 定点模式 loiter
	FLY_VELOCITY = 3,			// 速度模式
	FLY_POSITION = 4,			// 位置模式
	FLY_OFFBOARD = 5			// 外部控制模式
};



enum LOCKSTATE_e
{
	FLY_LOCKED = 0,				// 上锁状态
	FLY_UNLOCK = 1				// 解锁状态
};

/*
 * 传感器ID
 */
enum SENSOR_ID_e
{
	ACC_ID = 1,					// 加速度计ID
	GYRO_ID = 2,				// 陀螺仪ID
	MAG_ID = 3,					// 磁力计ID
	BARO_ID = 4					// 气压计ID
};


struct XYZ_t
{
	float x;
	float y;
	float z;
};


struct BARO_t
{
	uint16_t C[6];
	float temperature;
	float pressure;
	float alti;
};

struct KARMAN_t
{
	float A;		// 状态转移矩阵
	float P;		// 协方差初始值
	float Q;		// 协同噪声
	float R;		// 测量噪声
	float H;		// 转换到测量的矩阵
	float K;		// 卡尔曼增益
	float x;		// 此次估计结果
	float x_last;	// 上次估计结果
	float z;		// 此次测量值
};


struct LPF_t
{
	float  			cutoff_frequency;
	float           a1;
	float           a2;
	float           b0;
	float           b1;
	float           b2;
	float           delay_element_1;        // buffered sample -1
	float           delay_element_2;        // buffered sample -2
};



struct ROTATE_t
{
	uint8_t valid;
	
	float R[3][3];
};



#define MF_MAX_LENGTH 6							// 均值滤波数据长度
struct MF_t
{
	uint8_t len;
	float value[MF_MAX_LENGTH];
	float sum;										// 数的和
	float delta;									// 相邻时刻数据允许变化大小（均值滤波对异常过大/过小数据很敏感，必须做好限幅）
	float mean;										// 数据均值
};



struct EULAR_t 
{
	uint8_t valid;			// 数据有效性
	
	float roll;
	float pitch;
	float yaw;
};

struct QUAT_t
{
	uint8_t valid;			// 数据有效性
	
	float q0;
	float q1;
	float q2;
	float q3;
};

struct GPS_t 
{
	uint8_t valid;	// 有效性，1有效，0无效
	float tim;			// 时间(hhmmss:ms)
	
	double lat;			// 当前纬度（°）
	double lon; 		// 当前经度（°）
	float alt;			// 海拔高度（m）

	float vel_N;		// 北向速度(m/s)
	float vel_E;		// 东向速度(m/s)
	float vel_D;		// 向下速度(m/s)
	
	float vel_g;		// 地速（m/s）
	float angle;		// 方位角（°）注意要GPS移动时才能获取数据，否则为0
};

struct JOYSTICK_t
{
	uint16_t roll;				// 滚转通道的值
	uint16_t pitch;				// 俯仰通道的值
	uint16_t thr;					// 油门通道的值
	uint16_t yaw;					// 偏航通道的值
};

struct KEY_t
{
	uint8_t key1;
	uint8_t key2;
	uint8_t key3;
	uint8_t key4;
};

struct RC_t
{
	struct JOYSTICK_t joy;
	struct KEY_t key;
};

struct RCM_t						// Remote Control Mapping
{
	float roll;
	float pitch;
	float yaw_rate;
	float thr;
	
	float vx;
	float vy;
	float vz;
	
	uint8_t ch5;
	uint8_t ch6;
	uint8_t ch7;
	uint8_t ch8;
};



void LimitFloat(float *num, float nmin, float nmax);
void LimitUint16(uint16_t *num, uint16_t nmin, uint16_t nmax);
void LimitInt16(int16_t *num, int16_t nmin, int16_t nmax);

float LinearMapping(const float x_left, const float x_right, const float y_left, const float y_right, const float x);

void SetLpfCutoffFreq(struct LPF_t *f, float sample_freq, float cutoff_freq);
float ApplyLpf(struct LPF_t *f,float sample);

void MfInit(struct MF_t *mf, float delta);
float ApplyMf(struct MF_t *mf, float value);
double Str2Double(uint8_t *str, uint8_t len);
float Str2Float(char *str, uint8_t len);
uint32_t powi(uint32_t a, uint8_t b);
float DiffLimit(const float xk, const float xk_1, const float max_value, const float err_value);
void DiffLimit2(const float xk, float *xk_1, const float max_value, const float err_value);
float DiffLimitInc(const float xk, const float xk_1, const float max_value);
int16_t DiffLimitInt16Inc(const int16_t xk, const int16_t xk_1, const int16_t max_inc);

void Int2String(int16_t num, uint8_t *str);

void Dof3Rad2Deg(struct XYZ_t *dof3);
void Dof3Deg2Rad(struct XYZ_t *dof3);

float Mean(float *data, uint8_t len);
float Var(float *data, uint8_t len);
float Std(float *data, uint8_t len);
struct EULAR_t Quat2Eular(const struct QUAT_t quat);
struct QUAT_t Eular2Quat(const struct EULAR_t eular);
float Sign(float x);
float Norm3(const struct XYZ_t *vec);
void Quat2Rbe(const struct QUAT_t *quat, float Rbe[3][3]);

	
void Eular2Rbe(const struct EULAR_t *eular, float Rbe[3][3]);
void RemoveDeadBand(float *input, float deadband);
void RemoveDeadBandVector3(struct XYZ_t *input, float deadband);



#endif
