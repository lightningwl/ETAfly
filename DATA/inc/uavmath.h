#ifndef UAVMATH_H
#define UAVMATH_H

#include <math.h>
#include "sys.h"
#include "arm_math.h"

#define GRAVITY						9.8f							// ���ٶȴ�С

#if USING_GPS == 1
	#define EARTH_RADIUS		6378137.0					// ����뾶
#else
	#define EARTH_RADIUS		6371000.0					// ����뾶
#endif

#define RAD2DEG   				57.295779513082323f
#define DEG2RAD   				0.017453292519943f

#define UWB_LAT_START 		22.3061285			//ʹ��uwb��λ��ԭ��A0 γ�� 
#define UWB_LONG_START 		113.5404365 		//ʹ��uwb��λ��ԭ��A0 ����

#ifndef true
	#define true 1
#endif

#ifndef false
	#define false 0
#endif

	

enum FLYMODE_e
{
	FLY_STABILIZE = 0,		// ����ģʽ stabilize
	FLY_ALTHOLD = 1,			// ����ģʽ altitude hold
	FLY_LOITER = 2,				// ����ģʽ loiter
	FLY_VELOCITY = 3,			// �ٶ�ģʽ
	FLY_POSITION = 4,			// λ��ģʽ
	FLY_OFFBOARD = 5			// �ⲿ����ģʽ
};



enum LOCKSTATE_e
{
	FLY_LOCKED = 0,				// ����״̬
	FLY_UNLOCK = 1				// ����״̬
};

/*
 * ������ID
 */
enum SENSOR_ID_e
{
	ACC_ID = 1,					// ���ٶȼ�ID
	GYRO_ID = 2,				// ������ID
	MAG_ID = 3,					// ������ID
	BARO_ID = 4					// ��ѹ��ID
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
	float A;		// ״̬ת�ƾ���
	float P;		// Э�����ʼֵ
	float Q;		// Эͬ����
	float R;		// ��������
	float H;		// ת���������ľ���
	float K;		// ����������
	float x;		// �˴ι��ƽ��
	float x_last;	// �ϴι��ƽ��
	float z;		// �˴β���ֵ
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



#define MF_MAX_LENGTH 6							// ��ֵ�˲����ݳ���
struct MF_t
{
	uint8_t len;
	float value[MF_MAX_LENGTH];
	float sum;										// ���ĺ�
	float delta;									// ����ʱ����������仯��С����ֵ�˲����쳣����/��С���ݺ����У����������޷���
	float mean;										// ���ݾ�ֵ
};



struct EULAR_t 
{
	uint8_t valid;			// ������Ч��
	
	float roll;
	float pitch;
	float yaw;
};

struct QUAT_t
{
	uint8_t valid;			// ������Ч��
	
	float q0;
	float q1;
	float q2;
	float q3;
};

struct GPS_t 
{
	uint8_t valid;	// ��Ч�ԣ�1��Ч��0��Ч
	float tim;			// ʱ��(hhmmss:ms)
	
	double lat;			// ��ǰγ�ȣ��㣩
	double lon; 		// ��ǰ���ȣ��㣩
	float alt;			// ���θ߶ȣ�m��

	float vel_N;		// �����ٶ�(m/s)
	float vel_E;		// �����ٶ�(m/s)
	float vel_D;		// �����ٶ�(m/s)
	
	float vel_g;		// ���٣�m/s��
	float angle;		// ��λ�ǣ��㣩ע��ҪGPS�ƶ�ʱ���ܻ�ȡ���ݣ�����Ϊ0
};

struct JOYSTICK_t
{
	uint16_t roll;				// ��תͨ����ֵ
	uint16_t pitch;				// ����ͨ����ֵ
	uint16_t thr;					// ����ͨ����ֵ
	uint16_t yaw;					// ƫ��ͨ����ֵ
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
