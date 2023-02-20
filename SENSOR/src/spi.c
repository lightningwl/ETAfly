#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"

SPI_HandleTypeDef hspi2;


/* 
 * SPI2 ��ʼ��
 * SPI2 ���ڶ�ȡNRF����
 */
void SPI2_Init(void)
{
  hspi2.Instance	 						= SPI2;
  hspi2.Init.Mode 						= SPI_MODE_MASTER;
  hspi2.Init.Direction 				= SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize 				= SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity 			= SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase 				= SPI_PHASE_1EDGE;
  hspi2.Init.NSS 							= SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit 				= SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode 					= SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation 	= SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial 		= 10;
  HAL_SPI_Init(&hspi2);
}


/*
 * SPI �ײ�����
 * �ᱻ HAL_SPI_Init() ����
 * PB13 -- SPI2_SCK
 * PB14 -- SPI2_MISO
 * PB15 -- SPI2_MOSI
 */ 
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
	if(spiHandle->Instance==SPI2)
  {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin 			= GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode 			= GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull 			= GPIO_PULLUP;
    GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin 			= GPIO_PIN_14;
    GPIO_InitStruct.Mode 			= GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull 			= GPIO_PULLUP;
    GPIO_InitStruct.Speed	 		= GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}


/*
 * SPI�ٶ����ú���
 * SPI�ٶ�=fAPB1/��Ƶϵ��
 * @ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
 * fAPB1ʱ��һ��Ϊ42Mhz��
 */
void SPI2_SetSpeedApi(uint8_t pres)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));	//�ж���Ч��
	__HAL_SPI_DISABLE(&hspi2);            													//�ر�SPI
	hspi2.Instance->CR1 &= 0XFFC7;          												//λ3-5���㣬�������ò�����
	hspi2.Instance->CR1 |= pres;																		//����SPI�ٶ�
	__HAL_SPI_ENABLE(&hspi2);             													//ʹ��SPI
}


/*
 * SPI2��д����
 */
uint8_t SPI2_ReadWriteApi(uint8_t dat)
{
	uint8_t recv = 0x00;
	
	HAL_SPI_TransmitReceive(&hspi2, &dat, &recv, 1, 1000);
	
  return recv;
}

