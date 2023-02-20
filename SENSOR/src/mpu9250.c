#include "sys.h"
#include "mpu9250.h"
#include "iic.h"
#include "delay.h"
#include "math.h"
#include "uavmath.h"

/*
 * MPU9250��ʼ������
 */
uint8_t MPU9250_Init(void)
{
	uint8_t res = 0;
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_PWR_MGMT1_REG, 0x80);//��λMPU9250
	DelayMs(100);
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_PWR_MGMT2_REG, 0x00);//����MPU9250
	MPU9250_SetGyroFsr(MPU9250_GYRO_FSR);
	MPU9250_SetAccelFsr(MPU9250_ACCEL_FSR);
	MPU9250_SetRate(MPU9250_RATE);
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_INT_EN_REG, 0x00);//�ر������ж�
	
	
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_USER_CTRL_REG, 0x00);//I2C��ģʽ�ر�
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_FIFO_EN_REG, 0x00);//�ر�FIFO
	IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_INTBP_CFG_REG, 0x82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ//��ȡMPU6500��ID�Ӷ�ȡ������
	
	res = IIC_ReadAddrRegByte(MPU6500_ADDR, MPU9250_DEVICE_ID_REG);
	
	
	if (res == MPU6500_ID)
	{
		IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_PWR_MGMT1_REG, 0x01);//����CLKSEL,PLL X��Ϊ�ο�
		IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_PWR_MGMT2_REG, 0x00);//���ٶ��������Ƕ�����
		MPU9250_SetRate(MPU9250_RATE);
	}
	else
	{
		printf("MPU9250 Init failed 1, res = 0x%X\r\n", res);
		return 1;
	}
	
	res = IIC_ReadAddrRegByte(AK8963_ADDR, MPU9250_MAG_WIA);//��ȡAK8963 ID   
	if (res == AK8963_ID)
	{
		IIC_WriteAddrRegByte(AK8963_ADDR, MPU9250_MAG_CNTL1, 0x11);		//����AK8963Ϊ��������ģʽ
		printf("MPU9250 Init success.\r\n");
	}
	else
	{
		printf("MPU9250 Init failed 2, res = 0x%X\r\n", res);
		return 1;
	}
	return 0;
}




/* ����MPU9250�����Ǵ����������̷�Χ					
 * fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps	
 * ����ֵ:0,���óɹ�									
 *    ����,����ʧ�� 									
 */
uint8_t MPU9250_SetGyroFsr(uint8_t fsr)
{
	return IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_GYRO_CFG_REG, fsr<<3);	//���������������̷�Χ  
}



/* ����MPU92506050���ٶȴ����������̷�Χ	
 * fsr:0,��2g;1,��4g;2,��8g;3,��16g		
 * ����ֵ:0,���óɹ�						
 *    ����,����ʧ�� 						
 */

uint8_t MPU9250_SetAccelFsr(uint8_t fsr)
{
	return IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_ACCEL_CFG_REG, fsr<<3);	//���ü��ٶȴ����������̷�Χ  
}



/* ����MPU92506050�����ֵ�ͨ�˲���		
 * lpf:���ֵ�ͨ�˲�Ƶ��(Hz)				
 * ����ֵ:0,���óɹ�						
 *    ����,����ʧ�� 						
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
	return IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_CFG_REG, data);			//�������ֵ�ͨ�˲���  
}

/* ����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)		
 * rate:4~1000(Hz)						
 * ����ֵ:0,���óɹ�						
 *    ����,����ʧ�� 						
 */

uint8_t MPU9250_SetRate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)
		rate=1000;
	if(rate<4)
		rate=4;
	data = 1000 / rate - 1;
	data = IIC_WriteAddrRegByte(MPU6500_ADDR, MPU9250_SAMPLE_RATE_REG, data);	//�������ֵ�ͨ�˲���
 	return MPU9250_SetLPF(rate/2);											//�Զ�����LPFΪ�����ʵ�һ��
}

/* �õ��¶�ֵ			
 * ����ֵ:�¶�ֵ����C��	
 * �˴�Ϊʹ�õ�			
 */
float GetMpu9250TempeApi(void)
{
	uint8_t buf[2]; 
	short raw_tempe;		// �¶ȵ�ԭʼ����
	float tempe;			// �¶�ֵ
	IIC_ReadAddrRegBytes(MPU6500_ADDR, MPU9250_TEMP_OUTH_REG, 2, buf); 
	raw_tempe = ((uint16_t)buf[0]<<8) | buf[1];  
	tempe = 36.53 + ((double)raw_tempe) / 340.0;  
	return tempe;
}






/*
 * ʹ��IIC��ȡ���ٶ�����
 * ���أ�������ٶ�(m/s^2)
 */
struct XYZ_t GetMpu9250AccApi(void)
{
	uint8_t buf[6], res;  
	short raw_acc[3];				// ԭʼ�ļ��ٶ����ݣ��˴�������short, ȡֵ��Χ -32767~32736��ռ2�ֽڡ�intȡֵ��ΧҲ��ͬ������ռ4�ֽ�
	float fsr;						// ������
	struct XYZ_t acc;				// ���ٶ�(��λ��m/s^2)
	const float temp = 1.0f / 32767.0f;
	
	memset(buf, 0, 6);
	res = IIC_ReadAddrRegBytes(MPU6500_ADDR, MPU9250_ACCEL_XOUTH_REG, 6, buf);
		
	if(res==0)
	{
		raw_acc[0] = ((uint16_t)buf[0]<<8) | buf[1];  			// x ������ٶ�ԭʼֵ
		raw_acc[1] = ((uint16_t)buf[2]<<8) | buf[3];  			// y ������ٶ�ԭʼֵ  
		raw_acc[2] = ((uint16_t)buf[4]<<8) | buf[5];  			// z ������ٶ�ԭʼֵ
	} 	

	if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_2g)						// ѡ������
		fsr = 2.0f * GRAVITY;
	else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_4g)
		fsr = 4.0f * GRAVITY;
	else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_8g)
		fsr = 8.0f * GRAVITY;
	else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_16g)
		fsr = 16.0f * GRAVITY;

	acc.x = (float)raw_acc[0] * fsr * temp;		// x ������ٶ�
	acc.y = (float)raw_acc[1] * fsr * temp;		// y ������ٶ�
	acc.z = (float)raw_acc[2] * fsr * temp;		// z ������ٶ�	
	
	return acc;
}

/*
 * ʹ��IIC��ȡ����������
 * ���أ�������ٶ�(deg/s)
 */
struct XYZ_t GetMpu9250GyroApi(void)
{
	uint8_t res;
	uint8_t buf[6];					// 6bytes
	float fsr;
	const float temp = 1.0f / 32767.0f;
	short raw_gyro[3] = {0,0,0};
	struct XYZ_t gyro;				// ���ٶ�(��λ��m/s^2)
	
	res = IIC_ReadAddrRegBytes(MPU6500_ADDR, MPU9250_GYRO_XOUTH_REG, 6, buf);
	if(res==0)
	{
		raw_gyro[0] = ((uint16_t)buf[0]<<8) | buf[1];  			// ԭʼ gyro x
		raw_gyro[1] = ((uint16_t)buf[2]<<8) | buf[3];  			// ԭʼ gyro y
		raw_gyro[2] = ((uint16_t)buf[4]<<8) | buf[5];			// ԭʼ gyro z
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
	gyro.y = (float)raw_gyro[1] * fsr * temp;	// ���� fsr ���� 32767
	gyro.z = (float)raw_gyro[2] * fsr * temp;	
	
	return gyro;
}

/*
 * ��ȡ���������ݽӿ�
 */
struct XYZ_t GetMpu9250MagApi(void)
{
	uint8_t buf[6] ={0};
	uint8_t res;
	short mag_raw[3];
	const float fsr = 0.01f;
	struct XYZ_t mag;
	
	res = IIC_ReadAddrRegBytes(AK8963_ADDR, MPU9250_MAG_XOUT_L, 6, buf);	// IIC��ȡ������ԭʼ����
	if (res == 0)
	{
		mag_raw[0] = ((uint16_t)buf[1]<<8) | buf[0];		// ������ x����ԭʼ����
		mag_raw[1] = ((uint16_t)buf[3]<<8) | buf[2];		// ������ y����ԭʼ����
		mag_raw[2] = ((uint16_t)buf[5]<<8) | buf[4];		// ������ z����ԭʼ����
		
		
	}
	IIC_WriteAddrRegByte(AK8963_ADDR, MPU9250_MAG_CNTL1, 0x11);		// ��������ģʽ	

	mag.x = (float)mag_raw[0] * fsr;
	mag.y = (float)mag_raw[1] * fsr;
	mag.z = (float)mag_raw[2] * fsr;
	
	return mag;
}




