#include <stdlib.h>
#include <math.h>
#include "bmp280.h"
#include "delay.h"
#include "iic.h"
 

/*
 * BMP280��ȡ�Ĵ�������
 */
static uint8_t BMP280_ReadRegister(uint8_t reg_addr)
{
    return IIC_ReadAddrRegByte(BMP280_ADDRESS, reg_addr);
}
 
/*
 * BMP280д
 */
static void BMP280_WriteRegister(uint8_t reg_addr, uint8_t reg_data)
{
	IIC_WriteAddrRegByte(BMP280_ADDRESS, reg_addr, reg_data);
}
 
/**
 * ��bmp280_init()������Ĭ�ϳ�ʼ��t_standbyΪ0.5ms��
 * �¶Ⱥ���ѹ�Ĳ���������Ϊ��ͣ�
 * �˲���ϵ����Ϊ��ͣ�
 * ���ҽ���sleep mode��
 */
struct BMP280_t BMP280_Init(void)
{
	uint8_t bmp280_id;
	uint8_t lsb, msb;
	uint8_t ctrlmeas_reg, config_reg;
	struct BMP280_t bmp280;

	bmp280_id = BMP280_ReadRegister(BMP280_CHIPID_REG);
	if(bmp280_id == 0x58) 
	{
		bmp280.mode = BMP280_SLEEP_MODE;													// ģʽ
		bmp280.t_sb = BMP280_T_SB2;																// ����ʱ��			
		bmp280.p_oversampling = BMP280_P_MODE_2;									// ��ѹ��������
		bmp280.t_oversampling = BMP280_T_MODE_2;									// �¶ȹ�������
		bmp280.filter_coefficient = BMP280_FILTER_MODE_2;					// �˲�����
	} 
	else
	{
		printf("BMP280 ID error, current id = 0x%X\r\n", bmp280_id);
	}

	/* read the temperature calibration parameters */
	lsb = BMP280_ReadRegister(BMP280_DIG_T1_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_T1_MSB_REG);
	bmp280.T1 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_T2_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_T2_MSB_REG);
	bmp280.T2 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_T3_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_T3_MSB_REG);
	bmp280.T3 = msb << 8 | lsb;

	/* read the pressure calibration parameters */
	lsb = BMP280_ReadRegister(BMP280_DIG_P1_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P1_MSB_REG);
	bmp280.P1 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_P2_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P2_MSB_REG);
	bmp280.P2 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_P3_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P3_MSB_REG);
	bmp280.P3 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_P4_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P4_MSB_REG);
	bmp280.P4 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_P5_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P5_MSB_REG);
	bmp280.P5 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_P6_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P6_MSB_REG);
	bmp280.P6 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_P7_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P7_MSB_REG);
	bmp280.P7 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_P8_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P8_MSB_REG);
	bmp280.P8 = msb << 8 | lsb;
	lsb = BMP280_ReadRegister(BMP280_DIG_P9_LSB_REG);
	msb = BMP280_ReadRegister(BMP280_DIG_P9_MSB_REG);
	bmp280.P9 = msb << 8 | lsb;

	BMP280_Reset(&bmp280);

	ctrlmeas_reg = bmp280.t_oversampling << 5 | bmp280.p_oversampling << 2 | bmp280.mode;
	config_reg = bmp280.t_sb << 5 | bmp280.filter_coefficient << 2;

	BMP280_WriteRegister(BMP280_CTRLMEAS_REG, ctrlmeas_reg);
	BMP280_WriteRegister(BMP280_CONFIG_REG, config_reg);

	DelayMs(10);
	printf("BMP280 Init success\r\n");
	
	return bmp280;
}
 
/*
 * BMP280��λ
 */
void BMP280_Reset(struct BMP280_t *bmp280)
{
    BMP280_WriteRegister(BMP280_RESET_REG, BMP280_RESET_VALUE);
}
 
/*
 * BMP280����������Чʱ��
 */
void BMP280_SetStandbyTime(struct BMP280_t *bmp280, BMP280_T_SB t_standby)
{
    uint8_t config_reg;
 
    bmp280->t_sb = t_standby;
    config_reg = bmp280->t_sb << 5 | bmp280->filter_coefficient << 2;
 
    BMP280_WriteRegister(BMP280_CONFIG_REG, config_reg);
}
 
/*
 * BMP280���ù���ģʽ
 */
void BMP280_SetWorkMode(struct BMP280_t *bmp280, BMP280_WORK_MODE mode)
{
    uint8_t ctrlmeas_reg;
 
    bmp280->mode = mode;
    ctrlmeas_reg = bmp280->t_oversampling << 5 | bmp280->p_oversampling << 2 | bmp280->mode;
 
    BMP280_WriteRegister(BMP280_CTRLMEAS_REG, ctrlmeas_reg);
}
 
/*
 * BMP280�����¶ȹ�������
 */
void BMP280_SetTemperatureOversampling(struct BMP280_t *bmp280, BMP280_T_OVERSAMPLING t_osl)
{
    uint8_t ctrlmeas_reg;
 
    bmp280->t_oversampling = t_osl;
    ctrlmeas_reg = bmp280->t_oversampling << 5 | bmp280->p_oversampling << 2 | bmp280->mode;
 
    BMP280_WriteRegister(BMP280_CTRLMEAS_REG, ctrlmeas_reg);
}
 
/*
 * BMP280������ѹ��������
 */
void BMP280_SetPressureOversampling(struct BMP280_t *bmp280, BMP280_P_OVERSAMPLING p_osl)
{
    uint8_t ctrlmeas_reg;
 
    bmp280->t_oversampling = p_osl;
    ctrlmeas_reg = bmp280->t_oversampling << 5 | bmp280->p_oversampling << 2 | bmp280->mode;
 
    BMP280_WriteRegister(BMP280_CTRLMEAS_REG, ctrlmeas_reg);
}
 
/*
 * BMP280�����˲�ϵ��
 */
void BMP280_SetFilter(struct BMP280_t *bmp280, BMP280_FILTER_COEFFICIENT f_coefficient)
{
    uint8_t config_reg;
 
    bmp280->filter_coefficient = f_coefficient;
    config_reg = bmp280->t_sb << 5 | bmp280->filter_coefficient << 2;
 
    BMP280_WriteRegister(BMP280_CONFIG_REG, config_reg);
}
 
/* 
 * �¶Ȳ���
 */
static double BMP280_CompensateTemperatureDouble(struct BMP280_t *bmp280, int32_t adc_T)
{
    double var1, var2, temperature;
 
    var1 = (((double) adc_T) / 16384.0 - ((double) bmp280->T1) / 1024.0)
            * ((double) bmp280->T2);
    var2 = ((((double) adc_T) / 131072.0 - ((double) bmp280->T1) / 8192.0)
            * (((double) adc_T) / 131072.0 - ((double) bmp280->T1) / 8192.0))
            * ((double) bmp280->T3);
    bmp280->t_fine = (int32_t) (var1 + var2);
    temperature = (var1 + var2) / 5120.0;
 
    return temperature;
}
 
/* 
 * ��ѹ����
 */
static double BMP280_CompensatePressureDouble(struct BMP280_t *bmp280, int32_t adc_P)
{
    double var1, var2, pressure;
 
    var1 = ((double) bmp280->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) bmp280->P6) / 32768.0;
    var2 = var2 + var1 * ((double) bmp280->P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) bmp280->P4) * 65536.0);
    var1 = (((double) bmp280->P3) * var1 * var1 / 524288.0
            + ((double) bmp280->P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) bmp280->P1);
 
    if (var1 == 0.0) {
        return 0; 															// avoid exception caused by division by zero
    }
 
    pressure = 1048576.0 - (double) adc_P;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double) bmp280->P9) * pressure * pressure / 2147483648.0;
    var2 = pressure * ((double) bmp280->P8) / 32768.0;
    pressure = pressure + (var1 + var2 + ((double) bmp280->P7)) / 16.0;
 
    return pressure;
}
 
/* 
 * ��ȡ�¶�
 * ��λ(��)
 */
double GetBmp280TemperatureApi(struct BMP280_t *bmp280)
{
    uint8_t lsb, msb, xlsb;
    int32_t adc_T;
    double temperature;
 
    xlsb = BMP280_ReadRegister(BMP280_TEMPERATURE_XLSB_REG);
    lsb = BMP280_ReadRegister(BMP280_TEMPERATURE_LSB_REG);
    msb = BMP280_ReadRegister(BMP280_TEMPERATURE_MSB_REG);
 
    adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4);
    temperature = BMP280_CompensateTemperatureDouble(bmp280, adc_T);
 
    return temperature;
}
 
/*
 * ��ȡBMP280������ѹ�ӿ�
 * ��λ(Pa)
 */
double GetBmp280PressureApi(struct BMP280_t *bmp280)
{
    uint8_t lsb, msb, xlsb;
    int32_t adc_P;
    double pressure;
 
    xlsb = BMP280_ReadRegister(BMP280_PRESSURE_XLSB_REG);
    lsb = BMP280_ReadRegister(BMP280_PRESSURE_LSB_REG);
    msb = BMP280_ReadRegister(BMP280_PRESSURE_MSB_REG);
 
    adc_P = (msb << 12) | (lsb << 4) | (xlsb >> 4);
    pressure = BMP280_CompensatePressureDouble(bmp280, adc_P);
 
    return pressure;
}


/*
 * ����BMP280������Ϊnormal mode��
 * ��ʹ�øýӿ�ֱ�Ӷ�ȡ�¶Ⱥ���ѹ��
 */
void GetBmp280TempeperaturePressureApi(struct BMP280_t *bmp280, double *temperature, double *pressure)
{
		BMP280_SetWorkMode(bmp280, BMP280_FORCED_MODE);

    *temperature = GetBmp280TemperatureApi(bmp280);
    *pressure = GetBmp280PressureApi(bmp280);
}
 
/*
 * ��ȡBMP280��������
 */
float GetBmp280AltiApi(void)
{
	static struct BMP280_t bmp280;
	static uint8_t times = 0;
	double temperature = 0;
	double pressure = 0;
	float alti;	
	
	if (times == 0)
	{
		bmp280 = BMP280_Init();
		times = 1;
	}
	
	GetBmp280TempeperaturePressureApi(&bmp280, &temperature, &pressure);
	
	alti = (float)(44306.0 * (1.0 - pow(0.001*pressure / 101.325, 0.1902587)));	// ����
	
	return alti;
}


