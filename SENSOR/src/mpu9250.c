#include "sys.h"
#include "mpu9250.h"
#include "iic.h"
#include "delay.h"
#include "math.h"
#include "uavmath.h"

/*
 * MPU9250初始化函数
 */
uint8_t MPU9250_Init(void)
{
	uint8_t res = 0;
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_PWR_MGMT1_REG, 0x80);//复位MPU9250
	DelayMs(100);
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_PWR_MGMT2_REG, 0x00);//唤醒MPU9250
	MPU9250_SetGyroFsr(MPU9250_GYRO_FSR);
	MPU9250_SetAccelFsr(MPU9250_ACCEL_FSR);
	MPU9250_SetRate(MPU9250_RATE);
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_INT_EN_REG, 0x00);//关闭所有中断
	
	
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_USER_CTRL_REG, 0x00);//I2C主模式关闭
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_FIFO_EN_REG, 0x00);//关闭FIFO
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_INTBP_CFG_REG, 0x82);//INT引脚低电平有效，开启bypass模式，可以直//读取MPU6500的ID接读取磁力计
	
	res = IIC_ReadAddrRegByte(MPU6500_ADDR, MPU9250_DEVICE_ID_REG);
	
	
	if (res == MPU6500_ID)
	{
		IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_PWR_MGMT1_REG, 0x01);//设置CLKSEL,PLL X轴为参考
		IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_PWR_MGMT2_REG, 0x00);//加速度与陀螺仪都工作
		MPU9250_SetRate(MPU9250_RATE);
	}
	else
	{
		printf("MPU9250 Init failed 1, res = 0x%X\r\n", res);
		return 1;
	}
	
	res = IIC_ReadAddrRegByte(AK8963_ADDR, MPU9250_MAG_WIA);//读取AK8963 ID   
	if (res == AK8963_ID)
	{
		IIC_WriteAddrRegByte(AK8963_ADDR, MPU9250_MAG_CNTL1, 0x11);		//设置AK8963为连续测量模式
		printf("MPU9250 Init success.\r\n");
	}
	else
	{
		printf("MPU9250 Init failed 2, res = 0x%X\r\n", res);
		return 1;
	}
	return 0;
}




/* 设置MPU9250陀螺仪传感器满量程范围					
 * fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps	
 * 返回值:0,设置成功									
 *    其他,设置失败 									
 */
uint8_t MPU9250_SetGyroFsr(uint8_t fsr)
{
	return IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_GYRO_CFG_REG, fsr<<3);	//设置陀螺仪满量程范围  
}



/* 设置MPU92506050加速度传感器满量程范围	
 * fsr:0,±2g;1,±4g;2,±8g;3,±16g		
 * 返回值:0,设置成功						
 *    其他,设置失败 						
 */

uint8_t MPU9250_SetAccelFsr(uint8_t fsr)
{
	return IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_ACCEL_CFG_REG, fsr<<3);	//设置加速度传感器满量程范围  
}



/* 设置MPU92506050的数字低通滤波器		
 * lpf:数字低通滤波频率(Hz)				
 * 返回值:0,设置成功						
 *    其他,设置失败 						
 */

uint8_t MPU9250_SetLPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)
		data=1;
	else if(lpf>=98)
		data=2;
	else if(lpf>=42)
		data=3;
	else if(lpf>=20)
		data=4;
	else if(lpf>=10)
		data=5;
	else 
		data=6; 
	return IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_CFG_REG, data);			//设置数字低通滤波器  
}

/* 设置MPU9250的采样率(假定Fs=1KHz)		
 * rate:4~1000(Hz)						
 * 返回值:0,设置成功						
 *    其他,设置失败 						
 */

uint8_t MPU9250_SetRate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)
		rate=1000;
	if(rate<4)
		rate=4;
	data = 1000 / rate - 1;
	data = IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_SAMPLE_RATE_REG, data);	//设置数字低通滤波器
 	return MPU9250_SetLPF(rate/2);											//自动设置LPF为采样率的一半
}

/* 得到温度值			
 * 返回值:温度值（°C）	
 * 此处为使用到			
 */
float GetMpu9250TempeApi(void)
{
	uint8_t buf[2]; 
	short raw_tempe;		// 温度的原始数据
	float tempe;			// 温度值
	IIC_ReadAddrRegBytes(MPU6500_ADDR, MPU9250_TEMP_OUTH_REG, 2, buf); 
	raw_tempe = ((uint16_t)buf[0]<<8) | buf[1];  
	tempe = 36.53 + ((double)raw_tempe) / 340.0;  
	return tempe;
}






/*
 * 使用IIC获取加速度数据
 * 返回：三轴加速度(m/s^2)
 */
struct XYZ_t GetMpu9250AccApi(void)
{
	uint8_t buf[6], res;  
	short raw_acc[3];				// 原始的加速度数据，此处必须用short, 取值范围 -32767~32736，占2字节。int取值范围也相同，但是占4字节
	float fsr;						// 满量程
	struct XYZ_t acc;				// 加速度(单位：m/s^2)
	const float temp = 1.0f / 32767.0f;
	
	memset(buf, 0, 6);
	res = IIC_ReadAddrRegBytes(MPU6500_ADDR, MPU9250_ACCEL_XOUTH_REG, 6, buf);
		
	if(res==0)
	{
		raw_acc[0] = ((uint16_t)buf[0]<<8) | buf[1];  			// x 方向加速度原始值
		raw_acc[1] = ((uint16_t)buf[2]<<8) | buf[3];  			// y 方向加速度原始值  
		raw_acc[2] = ((uint16_t)buf[4]<<8) | buf[5];  			// z 方向加速度原始值
	} 	

	if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_2g)						// 选择量程
		fsr = 2.0f * GRAVITY;
	else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_4g)
		fsr = 4.0f * GRAVITY;
	else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_8g)
		fsr = 8.0f * GRAVITY;
	else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_16g)
		fsr = 16.0f * GRAVITY;

	acc.x = (float)raw_acc[0] * fsr * temp;		// x 方向加速度
	acc.y = (float)raw_acc[1] * fsr * temp;		// y 方向加速度
	acc.z = (float)raw_acc[2] * fsr * temp;		// z 方向加速度	
	
	return acc;
}

/*
 * 使用IIC获取陀螺仪数据
 * 返回：三轴角速度(deg/s)
 */
struct XYZ_t GetMpu9250GyroApi(void)
{
	uint8_t res;
	uint8_t buf[6];					// 6bytes
	float fsr;
	const float temp = 1.0f / 32767.0f;
	short raw_gyro[3] = {0,0,0};
	struct XYZ_t gyro;				// 加速度(单位：m/s^2)
	
	res = IIC_ReadAddrRegBytes(MPU6500_ADDR, MPU9250_GYRO_XOUTH_REG, 6, buf);
	if(res==0)
	{
		raw_gyro[0] = ((uint16_t)buf[0]<<8) | buf[1];  			// 原始 gyro x
		raw_gyro[1] = ((uint16_t)buf[2]<<8) | buf[3];  			// 原始 gyro y
		raw_gyro[2] = ((uint16_t)buf[4]<<8) | buf[5];			// 原始 gyro z
	} 	
	
	if (MPU9250_GYRO_FSR == MPU9250_GYRO_FSR_250)
		fsr = 250.0;
	else if (MPU9250_GYRO_FSR == MPU9250_GYRO_FSR_500)
		fsr = 500.0;
	else if (MPU9250_GYRO_FSR == MPU9250_GYRO_FSR_1000)
		fsr = 1000.0;
	else if (MPU9250_GYRO_FSR == MPU9250_GYRO_FSR_2000)
		fsr = 2000.0;
	
	gyro.x = (float)raw_gyro[0] * fsr * temp;
	gyro.y = (float)raw_gyro[1] * fsr * temp;	// 乘以 fsr 除以 32767
	gyro.z = (float)raw_gyro[2] * fsr * temp;	
	
	return gyro;
}

/*
 * 获取磁力计数据接口
 */
struct XYZ_t GetMpu9250MagApi(void)
{
	uint8_t buf[6] ={0};
	uint8_t res;
	short mag_raw[3];
	const float fsr = 0.01f;
	struct XYZ_t mag;
	
	res = IIC_ReadAddrRegBytes(AK8963_ADDR, MPU9250_MAG_XOUT_L, 6, buf);	// IIC读取磁力计原始数据
	if (res == 0)
	{
		mag_raw[0] = ((uint16_t)buf[1]<<8) | buf[0];		// 磁力计 x方向原始数据
		mag_raw[1] = ((uint16_t)buf[3]<<8) | buf[2];		// 磁力计 y方向原始数据
		mag_raw[2] = ((uint16_t)buf[5]<<8) | buf[4];		// 磁力计 z方向原始数据
		
		
	}
	IIC_WriteAddrRegByte(AK8963_ADDR, MPU9250_MAG_CNTL1, 0x11);		// 连续测量模式	

	mag.x = (float)mag_raw[0] * fsr;
	mag.y = (float)mag_raw[1] * fsr;
	mag.z = (float)mag_raw[2] * fsr;
	
	return mag;
}




