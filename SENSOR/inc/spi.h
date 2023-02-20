#ifndef SPI_H
#define SPI_H

#include "sys.h"


extern SPI_HandleTypeDef hspi2;			// NRF24L01 ��ȡ����ʹ��

void SPI2_Init(void);

uint8_t SPI2_ReadWriteApi(uint8_t dat);
void SPI2_SetSpeedApi(uint8_t pres);

#endif

