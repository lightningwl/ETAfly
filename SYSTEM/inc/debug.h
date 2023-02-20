#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	


void USART1_Init(void);

void PrintDataTask(void *arg);									// 打印数据接口
void Usart1RecvDataTask(void *arg);							// 串口1接收数据
void PrintDataApi(uint8_t *data, uint8_t len);	// 发送数据接口

#endif
