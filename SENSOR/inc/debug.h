#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	


#define USART_MAX_RX_LEN  			32  	//定义最大接收字节数 200	  	
#define USART_BUFFER_SIZE   		1			//缓存大小


#define ANO_DATA_MAX_LEN				32


/*
 * 匿名上位机数据帧
 */
 

struct ANO_t
{
	uint16_t head;										// 帧头
	uint8_t func;											// 功能字
	uint8_t len;											// 长度
		
	int16_t data[ANO_DATA_MAX_LEN];		// 数据
	uint8_t sum;											// 校验和
};





void USART1_Init(void);

void Usart1RecvDataTask(void *arg);
void AnoDebugTask(void *arg);			// 使用匿名上位机 ANO_PC 调试
void PrintDataTask(void *arg);			// 打印数据接口





#endif
