/*
 * ���ļ�ʵ��
 * 1. ��ȡʮ�ᴫ�������ݣ�ͨ��IIC��ȡ���ٶȼƣ�����������ѹ������
 * 2. �Ծ��ᴫ����У׼�����ٶ�+������+�����ƣ�
 * 3. �����ᴫ������ͨ�˲������ٶ�+�����ǣ�
 *
 * ���ļ����ⲿ�ṩ
 * 1. ԭʼ��ʮ�ᴫ��������
 * 2. У׼��ľ��ᴫ��������
 * 3. �˲�������ᴫ��������
 */

#include "dof10.h"
#include "iic.h"
#include "delay.h"
#include <string.h>
#include "uavmath.h"
#include "mpu9250.h"
#include "bmp280.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>





#define LPF_ACC_SAMPLE_FREQ				500.0f			// ���ٶȵ�ͨ�˲�����Ƶ��
#define LPF_ACC_XY_CUTTOF_FREQ		20.0f				// ���ٶȵ�ͨ�˲�����Ƶ��
#define LPF_ACC_Z_CUTTOF_FREQ			20.0f				// ���ٶȵ�ͨ�˲�����Ƶ��

#define LPF_GYRO_SAMPLE_FREQ			500.0f			// �����ǵ�ͨ�˲�����Ƶ��
#define LPF_GYRO_CUTTOF_FREQ			50.0f				// �����ǵ�ͨ�˲�����Ƶ��



struct DOF10_t raw_dof10_;										// ԭʼ��ʮ�ᴫ��������
struct DOF9_t  cali_dof9_;										// У׼��ľ��ᴫ��������
struct DOF6_t  lpf_dof6_;											// ��ͨ�˲�������ᴫ��������


// 3
struct XYZ_t acc_offset_ 		= {0.7880f,    0.2451f,    0.1814f};	// ƫ��
struct XYZ_t acc_scale_ 		= {1.0013f,    0.9985f,    0.9889f};	// �߶�

struct XYZ_t gyro_offset_ 	= {0.0f, 0.0f, 0.0f};						// ������ƫ��

struct XYZ_t mag_offset_ 		= {-1.5542f,    7.3584f,   -1.2493f};					// ������ƫ������
struct XYZ_t mag_scale_ 		= {0.3873f,    0.3951f,    0.3430f};					// �����Ƴߴ�����

/*
// 2
struct XYZ_t acc_offset_ 		= {0.2515f,    0.1515f,    0.2067f};	// ƫ��
struct XYZ_t acc_scale_ 		= {1.0008f,    0.9988f,    0.9868f};	// �߶�

struct XYZ_t gyro_offset_ 	= {0.0f, 0.0f, 0.0f};						// ������ƫ��

struct XYZ_t mag_offset_ 		= {4.3337f,   -0.8319f,   -1.1915f};					// ������ƫ������
struct XYZ_t mag_scale_ 		= {0.4507f,    0.4068f,    0.4025f};					// �����Ƴߴ�����

*/
/*
 * ���ٶȼ�У׼������任
 * У׼��ʽ������У׼
 * ����ϵ��ǰ����
 */
static struct XYZ_t CalibrateAcc(const struct XYZ_t raw)
{
	struct XYZ_t cali;

	cali.x = -acc_scale_.x * (raw.x - acc_offset_.x);	// acc_scale_, acc_offset_ Ϊȫ�ֱ���	
	cali.y =  acc_scale_.y * (raw.y - acc_offset_.y);	// �밲װλ���й�				
	cali.z =  acc_scale_.z * (raw.z - acc_offset_.z);
	
	return cali;
}


/*
 * ������У׼������任
 * У׼��ʽ����ȥ��ֵȥ����ƫ
 * ����ϵ��ǰ����
 */
static struct  XYZ_t CalibrateGyro(const struct XYZ_t raw)
{
	struct XYZ_t cali;
	
	cali.x = (raw.x - gyro_offset_.x);	// �����Ƕ����������ת������֤��ͷΪ x �ᣬ����90��Ϊ y �ᣬ����Ϊ z ��					
	cali.y = -(raw.y - gyro_offset_.y);	// acc_scale_, gyro_offset_ Ϊȫ�ֱ���
	cali.z = -(raw.z - gyro_offset_.z);
	
	return cali;
}

/*
 * ������У׼������任
 * У׼��ʽ������У׼
 * ����ϵ��ǰ����
 */
static struct XYZ_t CalibrateMag(const struct XYZ_t raw)
{
	struct XYZ_t cali;
	
	cali.x = mag_scale_.y*(raw.y - mag_offset_.y);	// mag_scale_, mag_offset_ Ϊȫ�ֱ���				
	cali.y = -mag_scale_.y*(raw.x - mag_offset_.x);	
	cali.z = mag_scale_.z*(raw.z - mag_offset_.z);
	
	return cali;
}
 


/*
 * ������ٶȵ�ͨ�˲�
 */
static struct XYZ_t AccApplyLpf(const struct XYZ_t acc)
{
	struct XYZ_t acc_f = {0,0,0};
	static uint8_t acc_lpf_state = 0;
	
	static struct LPF_t lpf_accx;
	static struct LPF_t lpf_accy;
	static struct LPF_t lpf_accz;
	
	
	if (acc_lpf_state == 0)
	{
		SetLpfCutoffFreq(&lpf_accx, LPF_ACC_SAMPLE_FREQ, LPF_ACC_XY_CUTTOF_FREQ);
		SetLpfCutoffFreq(&lpf_accy, LPF_ACC_SAMPLE_FREQ, LPF_ACC_XY_CUTTOF_FREQ);
		SetLpfCutoffFreq(&lpf_accz, LPF_ACC_SAMPLE_FREQ, LPF_ACC_Z_CUTTOF_FREQ);		
		
		acc_lpf_state = 1;
	}
	else
	{
		acc_f.x = ApplyLpf(&lpf_accx, acc.x);
		acc_f.y = ApplyLpf(&lpf_accy, acc.y);
		acc_f.z = ApplyLpf(&lpf_accz, acc.z);
	}
	return acc_f;
}



/*
 * ���������ǵ�ͨ�˲�
 */
static struct XYZ_t GyroApplyLpf(const struct XYZ_t gyro)
{
	struct XYZ_t gyro_f = {0,0,0};
	static uint8_t gyro_lpf_state = 0;
	
	static struct LPF_t lpf_gyrox;
	static struct LPF_t lpf_gyroy;
	static struct LPF_t lpf_gyroz;
	
	if (gyro_lpf_state == 0)
	{
		SetLpfCutoffFreq(&lpf_gyrox, LPF_GYRO_SAMPLE_FREQ, LPF_GYRO_CUTTOF_FREQ);
		SetLpfCutoffFreq(&lpf_gyroy, LPF_GYRO_SAMPLE_FREQ, LPF_GYRO_CUTTOF_FREQ);
		SetLpfCutoffFreq(&lpf_gyroz, LPF_GYRO_SAMPLE_FREQ, LPF_GYRO_CUTTOF_FREQ);		
		
		gyro_lpf_state = 1;
	}
	else
	{
		gyro_f.x = ApplyLpf(&lpf_gyrox, gyro.x);
		gyro_f.y = ApplyLpf(&lpf_gyroy, gyro.y);
		gyro_f.z = ApplyLpf(&lpf_gyroz, gyro.z);
	}
	return gyro_f;
}



/*
 * �ȴ�IMU������Ч
 * �жϷ��������ٶ�/���ٶȲ�Ϊ0������ĳһ��ֵ
 */
uint8_t WaitImuValid(void)
{
	uint8_t k = 0;
	struct XYZ_t acc;
	struct XYZ_t gyro; 
	
	while (1)
	{
		acc = GetMpu9250AccApi();
		gyro = GetMpu9250GyroApi();
		
		if (Norm3(&acc) > 7.0f && Norm3(&gyro) > 1e-5f)	// ģ����ĳ�����������ʼ��ֵλ0
			return 1;
		
		if (k % 100 == 1)
			printf("wait imu valid ...\r\n");
		
		k ++;
		
		vTaskDelay(2);
	}
	return 0;
}




/*
 * ��ȡ������ƫ��
 * 1����ȡ�ɹ�
 * 0����ȡ��
 */
static uint8_t SetGyroOffsetAuto(struct XYZ_t *gyro_offsets)
{
	uint8_t k = 0;
	struct XYZ_t gyro_offset = {0,0,0};
	struct XYZ_t gyro;
	
	while (1)
	{
		gyro = GetMpu9250GyroApi();
		
		if (Norm3(&gyro) > 1e-6f && Norm3(&gyro) < 10.0f)		// ������ϵ�����Ϊ0���������˻�����
		{
			gyro_offset.x += gyro.x;
			gyro_offset.y += gyro.y;
			gyro_offset.z += gyro.z;
			k ++;
					
			gyro_offsets->x = gyro_offset.x / k;
			gyro_offsets->y = gyro_offset.y / k;
			gyro_offsets->z = gyro_offset.z / k;
		}
		if (k >= 200)
			return 1;
		
		vTaskDelay(2);
	}
	return 0;
}



/*
 * ��ȡʮ������
 */
void Dof10Task(void *arg)
{
	uint8_t k = 0;
	uint8_t state = 0;	
	
	portTickType t_last;
	
	IIC_Init();																		// IIC��ʼ��	
	MPU9250_Init();																// MPU9250��ʼ��
	
	if (WaitImuValid() == 1)											// �ȴ�������Ч
		state = 1;
	if (SetGyroOffsetAuto(&gyro_offset_) == 1)
		state = 2;
	
	t_last = xTaskGetTickCount();

	while (1)
	{		
		vTaskDelayUntil(&t_last, 2);													// 500Hz
		
		raw_dof10_.acc 	= GetMpu9250AccApi();									// ��ȡ���ٶ�����
		cali_dof9_.acc 	= CalibrateAcc(raw_dof10_.acc);				// У׼���ٶ�����
		lpf_dof6_.acc  	= AccApplyLpf(cali_dof9_.acc);				// ���ٶȵ�ͨ�˲�
		
		raw_dof10_.gyro = GetMpu9250GyroApi();								// ��ȡ����������
		cali_dof9_.gyro = CalibrateGyro(raw_dof10_.gyro);			// У׼����������
		lpf_dof6_.gyro 	= GyroApplyLpf(cali_dof9_.gyro);			// ���ٶȵ�ͨ�˲�
		
		if (k % 20 == 1)
		{
			raw_dof10_.mag = GetMpu9250MagApi();								// ��ȡ����������
			cali_dof9_.mag = CalibrateMag(raw_dof10_.mag);			// У׼����������			
		}
		if (k % 5 == 0)
		{
			raw_dof10_.alti = GetBmp280AltiApi();								// ��ȡ��ѹ������
		}
		
		k ++;
		k %= 100;		
	}
}



/*
 * ��ȡʮ��ԭʼ����
 */
struct DOF10_t GetDof10RawDataApi(void)
{
	return raw_dof10_;
}

/*
 * ָ�뷽ʽ��ȡʮ��ԭʼ����
 */
struct DOF10_t *PointDof10RawDataApi(void)
{
	return &raw_dof10_;
}



/*
 * ��ȡУ׼���������
 */
struct DOF9_t GetDof9CaliDataApi(void)
{
	return cali_dof9_;
}


/*
 * ָ�뷽ʽ��ȡУ׼���������
 */
struct DOF9_t *PointDof9CaliDataApi(void)
{
	return &cali_dof9_;
}






/*
 * ��ȡ��ͨ�˲�����������
 */
struct DOF6_t GetDof6LpfDataApi(void)
{
	return lpf_dof6_;
}


/*
 * ָ�뷽ʽ��ȡ��ͨ�˲�����������
 */
struct DOF6_t *PointDof6LpfDataApi(void)
{
	return &lpf_dof6_;
}

