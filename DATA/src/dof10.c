/*
 * 此文件实现
 * 1. 获取十轴传感器数据，通过IIC获取加速度计，陀螺仪与气压计数据
 * 2. 对九轴传感器校准（加速度+陀螺仪+磁力计）
 * 3. 对六轴传感器低通滤波（加速度+陀螺仪）
 *
 * 此文件向外部提供
 * 1. 原始的十轴传感器数据
 * 2. 校准后的九轴传感器数据
 * 3. 滤波后的六轴传感器数据
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





#define LPF_ACC_SAMPLE_FREQ				500.0f			// 加速度低通滤波采样频率
#define LPF_ACC_XY_CUTTOF_FREQ		20.0f				// 加速度低通滤波截至频率
#define LPF_ACC_Z_CUTTOF_FREQ			20.0f				// 加速度低通滤波截至频率

#define LPF_GYRO_SAMPLE_FREQ			500.0f			// 陀螺仪低通滤波采样频率
#define LPF_GYRO_CUTTOF_FREQ			50.0f				// 陀螺仪低通滤波截至频率



struct DOF10_t raw_dof10_;										// 原始的十轴传感器数据
struct DOF9_t  cali_dof9_;										// 校准后的九轴传感器数据
struct DOF6_t  lpf_dof6_;											// 低通滤波后的六轴传感器数据


// 3
struct XYZ_t acc_offset_ 		= {0.7880f,    0.2451f,    0.1814f};	// 偏移
struct XYZ_t acc_scale_ 		= {1.0013f,    0.9985f,    0.9889f};	// 尺度

struct XYZ_t gyro_offset_ 	= {0.0f, 0.0f, 0.0f};						// 陀螺仪偏移

struct XYZ_t mag_offset_ 		= {-1.5542f,    7.3584f,   -1.2493f};					// 磁力计偏移因子
struct XYZ_t mag_scale_ 		= {0.3873f,    0.3951f,    0.3430f};					// 磁力计尺寸因子

/*
// 2
struct XYZ_t acc_offset_ 		= {0.2515f,    0.1515f,    0.2067f};	// 偏移
struct XYZ_t acc_scale_ 		= {1.0008f,    0.9988f,    0.9868f};	// 尺度

struct XYZ_t gyro_offset_ 	= {0.0f, 0.0f, 0.0f};						// 陀螺仪偏移

struct XYZ_t mag_offset_ 		= {4.3337f,   -0.8319f,   -1.1915f};					// 磁力计偏移因子
struct XYZ_t mag_scale_ 		= {0.4507f,    0.4068f,    0.4025f};					// 磁力计尺寸因子

*/
/*
 * 加速度计校准与坐标变换
 * 校准方式：六面校准
 * 坐标系：前右下
 */
static struct XYZ_t CalibrateAcc(const struct XYZ_t raw)
{
	struct XYZ_t cali;

	cali.x = -acc_scale_.x * (raw.x - acc_offset_.x);	// acc_scale_, acc_offset_ 为全局变量	
	cali.y =  acc_scale_.y * (raw.y - acc_offset_.y);	// 与安装位置有关				
	cali.z =  acc_scale_.z * (raw.z - acc_offset_.z);
	
	return cali;
}


/*
 * 陀螺仪校准与坐标变换
 * 校准方式：减去均值去除零偏
 * 坐标系：前右下
 */
static struct  XYZ_t CalibrateGyro(const struct XYZ_t raw)
{
	struct XYZ_t cali;
	
	cali.x = (raw.x - gyro_offset_.x);	// 负号是对坐标轴进行转换，保证机头为 x 轴，向右90°为 y 轴，向下为 z 轴					
	cali.y = -(raw.y - gyro_offset_.y);	// acc_scale_, gyro_offset_ 为全局变量
	cali.z = -(raw.z - gyro_offset_.z);
	
	return cali;
}

/*
 * 磁力计校准与坐标变换
 * 校准方式：椭球校准
 * 坐标系：前右下
 */
static struct XYZ_t CalibrateMag(const struct XYZ_t raw)
{
	struct XYZ_t cali;
	
	cali.x = mag_scale_.y*(raw.y - mag_offset_.y);	// mag_scale_, mag_offset_ 为全局变量				
	cali.y = -mag_scale_.y*(raw.x - mag_offset_.x);	
	cali.z = mag_scale_.z*(raw.z - mag_offset_.z);
	
	return cali;
}
 


/*
 * 三轴加速度低通滤波
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
 * 三轴陀螺仪低通滤波
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
 * 等待IMU数据有效
 * 判断方法：加速度/角速度不为0，大于某一阈值
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
		
		if (Norm3(&acc) > 7.0f && Norm3(&gyro) > 1e-5f)	// 模大于某个数，避免初始数值位0
			return 1;
		
		if (k % 100 == 1)
			printf("wait imu valid ...\r\n");
		
		k ++;
		
		vTaskDelay(2);
	}
	return 0;
}




/*
 * 获取陀螺仪偏移
 * 1：获取成功
 * 0：获取中
 */
static uint8_t SetGyroOffsetAuto(struct XYZ_t *gyro_offsets)
{
	uint8_t k = 0;
	struct XYZ_t gyro_offset = {0,0,0};
	struct XYZ_t gyro;
	
	while (1)
	{
		gyro = GetMpu9250GyroApi();
		
		if (Norm3(&gyro) > 1e-6f && Norm3(&gyro) < 10.0f)		// 避免刚上电数据为0，避免无人机在振动
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
 * 获取十轴数据
 */
void Dof10Task(void *arg)
{
	uint8_t k = 0;
	uint8_t state = 0;	
	
	portTickType t_last;
	
	IIC_Init();																		// IIC初始化	
	MPU9250_Init();																// MPU9250初始化
	
	if (WaitImuValid() == 1)											// 等待数据有效
		state = 1;
	if (SetGyroOffsetAuto(&gyro_offset_) == 1)
		state = 2;
	
	t_last = xTaskGetTickCount();

	while (1)
	{		
		vTaskDelayUntil(&t_last, 2);													// 500Hz
		
		raw_dof10_.acc 	= GetMpu9250AccApi();									// 获取加速度数据
		cali_dof9_.acc 	= CalibrateAcc(raw_dof10_.acc);				// 校准加速度数据
		lpf_dof6_.acc  	= AccApplyLpf(cali_dof9_.acc);				// 加速度低通滤波
		
		raw_dof10_.gyro = GetMpu9250GyroApi();								// 获取陀螺仪数据
		cali_dof9_.gyro = CalibrateGyro(raw_dof10_.gyro);			// 校准陀螺仪数据
		lpf_dof6_.gyro 	= GyroApplyLpf(cali_dof9_.gyro);			// 角速度低通滤波
		
		if (k % 20 == 1)
		{
			raw_dof10_.mag = GetMpu9250MagApi();								// 获取磁力计数据
			cali_dof9_.mag = CalibrateMag(raw_dof10_.mag);			// 校准磁力计数据			
		}
		if (k % 5 == 0)
		{
			raw_dof10_.alti = GetBmp280AltiApi();								// 获取气压计数据
		}
		
		k ++;
		k %= 100;		
	}
}



/*
 * 获取十轴原始数据
 */
struct DOF10_t GetDof10RawDataApi(void)
{
	return raw_dof10_;
}

/*
 * 指针方式获取十轴原始数据
 */
struct DOF10_t *PointDof10RawDataApi(void)
{
	return &raw_dof10_;
}



/*
 * 获取校准后九轴数据
 */
struct DOF9_t GetDof9CaliDataApi(void)
{
	return cali_dof9_;
}


/*
 * 指针方式获取校准后九轴数据
 */
struct DOF9_t *PointDof9CaliDataApi(void)
{
	return &cali_dof9_;
}






/*
 * 获取低通滤波后六轴数据
 */
struct DOF6_t GetDof6LpfDataApi(void)
{
	return lpf_dof6_;
}


/*
 * 指针方式获取低通滤波后六轴数据
 */
struct DOF6_t *PointDof6LpfDataApi(void)
{
	return &lpf_dof6_;
}

