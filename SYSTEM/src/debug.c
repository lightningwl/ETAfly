#include "debug.h"
#include "delay.h"
#include <string.h>
#include "sys.h"

/**************************************************************************************************************/
/*****************************************  PART1�����ڽ��յײ�����	  *****************************************/

static uint16_t usart_rx_sta=0;    						//����״̬, bit15: ������ɱ�־;	bit14: ���յ�0x0d; bit13~0: ���յ�����Ч�ֽ���Ŀ
static uint8_t usart_rx_buf[USART_MAX_RX_LEN];     		//���ջ���,���USART_MAXRX_LEN���ֽ�.
static uint8_t usart_buf[USART_BUFFER_SIZE];				//HAL��ʹ�õĴ��ڽ��ջ���

UART_HandleTypeDef usart1; 							//UART���

/* ��ʼ��IO ʹ�ô���1 		*/
/* bound:������				*/
void DebugInit(uint32_t bound)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
		
	DEBUG_ENCLK();											// ʹ��ʱ��
	
	GPIO_Initure.Pin		= DEBUG_PIN;					// Pin
	GPIO_Initure.Mode		= GPIO_MODE_AF_PP;				// �����������
	GPIO_Initure.Pull		= GPIO_PULLUP;					// ����
	GPIO_Initure.Speed		= GPIO_SPEED_FAST;				// ����
	GPIO_Initure.Alternate	= GPIO_AF7_USART1;				// ����ΪUSART1
	HAL_GPIO_Init(DEBUG_GPIO,&GPIO_Initure);	   					// ��ʼ��
		
	//UART ��ʼ������	
	usart1.Instance			= USART1;				// USART1
	usart1.Init.BaudRate	= bound;				// ������
	usart1.Init.WordLength	= UART_WORDLENGTH_8B;   // �ֳ�Ϊ8λ���ݸ�ʽ
	usart1.Init.StopBits	= UART_STOPBITS_1;	    // һ��ֹͣλ
	usart1.Init.Parity		= UART_PARITY_NONE;		// ����żУ��λ
	usart1.Init.HwFlowCtl	= UART_HWCONTROL_NONE;  // ��Ӳ������
	usart1.Init.Mode		= UART_MODE_TX_RX;		// �շ�ģʽ
	usart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&usart1);					    	// HAL_UART_Init()��ʹ��UART2
	
	HAL_NVIC_EnableIRQ(USART1_IRQn);						// ʹ��USART1�ж�ͨ��
	HAL_NVIC_SetPriority(USART1_IRQn, 3, 4);				// ��ռ���ȼ�3�������ȼ�3	
	HAL_UART_Receive_IT(&usart1, (uint8_t *)usart_buf, USART_BUFFER_SIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
}

 
/* ����1�жϷ������		*/
void USART1_IRQHandler(void)                	
{ 
	if((usart_rx_sta&0x8000)==0)							//����δ���
	{
		if(usart_rx_sta&0x4000)								//���յ���0x0d
		{
			if(usart_buf[0]!=0x0a)
				usart_rx_sta=0;								//���մ���,���¿�ʼ
			else 
				usart_rx_sta|=0x8000;						//��������� 
		}
		else 												//��û�յ�0X0D
		{	
			if(usart_buf[0]==0x0d)
				usart_rx_sta|=0x4000;
			else
			{
				usart_rx_buf[usart_rx_sta & 0X3FFF]=usart_buf[0] ;
				usart_rx_sta++;
				if(usart_rx_sta>(USART_MAX_RX_LEN-1))
					usart_rx_sta=0;							//�������ݴ���,���¿�ʼ����	  
			}		 
		}
	}
	HAL_UART_IRQHandler(&usart1);							//����HAL���жϴ����ú���
	HAL_UART_Receive_IT(&usart1, (uint8_t *)usart_buf, USART_BUFFER_SIZE);//һ�δ������֮�����¿����жϲ�����usart_bufΪ1
} 


/* ������ӡ��������						*/
/* ����ʹ�ã��ܶ����ݶ���Ϊʹ�ã����ؾ���	*/
void PrintDataTask(void *arg)
{
	while (1)
	{
		delay_ms(200);
	}
}


/* �����ݷ��͵�������λ��������ʾ����											*/
/* һ�οɴ���Ÿ����ݣ���Ϊ0x02�����ְ���9�����ݣ��ɲο�����ͨ��Э��			*/
/* ע�⣬�˴�Ҫ��ң����CH5�� 1�������ݲ��ϴ�����2���ǽ���ֹͣ�����ᴫ������	*/
void AnoDebugTask(void *arg)
{
	while (1)
	{
		delay_ms(1000);
	}
}


/********************************************************************/
/* 							һЩ��ʱ���ù��ĵĺ���					*/
#if 1
#pragma import(__use_no_semihosting)   
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (uint8_t) ch;      
	return ch;
}


#endif 


