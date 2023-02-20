#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	


#define USART_MAX_RX_LEN  			32  	//�����������ֽ��� 200	  	
#define USART_BUFFER_SIZE   		1			//�����С


#define ANO_DATA_MAX_LEN				32


/*
 * ������λ������֡
 */
 

struct ANO_t
{
	uint16_t head;										// ֡ͷ
	uint8_t func;											// ������
	uint8_t len;											// ����
		
	int16_t data[ANO_DATA_MAX_LEN];		// ����
	uint8_t sum;											// У���
};





void USART1_Init(void);

void Usart1RecvDataTask(void *arg);
void AnoDebugTask(void *arg);			// ʹ��������λ�� ANO_PC ����
void PrintDataTask(void *arg);			// ��ӡ���ݽӿ�





#endif
