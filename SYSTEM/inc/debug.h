#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	


void USART1_Init(void);

void PrintDataTask(void *arg);									// ��ӡ���ݽӿ�
void Usart1RecvDataTask(void *arg);							// ����1��������
void PrintDataApi(uint8_t *data, uint8_t len);	// �������ݽӿ�

#endif
