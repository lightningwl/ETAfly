#include "debug.h"
#include "delay.h"
#include <string.h>
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dof10.h"
#include "communicate.h"
#include "atti_esti.h"
#include "pos_esti.h"
#include "debug.h"
#include "rc.h"
#include "control.h"

#define USART1_RX_MAX_LEN						64
#define USART1_TX_MAX_LEN						512


volatile uint8_t tc_flag = 0;																		// ���������ɱ�־λ��
																																// �����ڽ�������ж� IDLE ʱ��˵�����ݴ�����ɣ���1
																																// ���������ݺ���0���ɼ�����������
uint8_t usart1_rx_buf[USART1_RX_MAX_LEN];


/*****************************************  PART1�����ڽ��յײ�����	  *****************************************/

#define USART1_BAUD_RATE	921600
		
												
#define USART1_PIN			GPIO_PIN_9 | GPIO_PIN_10
#define USART1_GPIO			GPIOA
#define USART1_ENCLK()	__HAL_RCC_GPIOA_CLK_ENABLE();	\
												__HAL_RCC_USART1_CLK_ENABLE();					//ʹ��GPIOAʱ��,USART1ʱ��	


UART_HandleTypeDef 	usart1; 																		//UART���
DMA_HandleTypeDef  	usart1_tx_dma;      												//DMA���
DMA_HandleTypeDef 	usart1_rx_dma;


/* 
 * ��ʼ��IO ʹ�ô���1 		
 * bound:������				
 */
void USART1_Init(void)
{
  /* GPIO�˿����� */
	GPIO_InitTypeDef GPIO_Initure;
		
	USART1_ENCLK();																													// ʹ��ʱ��
	
	GPIO_Initure.Pin				= USART1_PIN;																		// Pin
	GPIO_Initure.Mode				= GPIO_MODE_AF_PP;															// �����������
	GPIO_Initure.Pull				= GPIO_PULLUP;																	// ����
	GPIO_Initure.Speed			= GPIO_SPEED_FAST;															// ����
	GPIO_Initure.Alternate	= GPIO_AF7_USART1;															// ����ΪUSART1
	HAL_GPIO_Init(USART1_GPIO, &GPIO_Initure);	   													// ��ʼ��
			
	/* UART1 TX DMA����--DMA2 Stream7 Channel4 */
	__HAL_RCC_DMA2_CLK_ENABLE();																						//DMA2ʱ��ʹ��	
	__HAL_LINKDMA(&usart1,hdmatx,usart1_tx_dma);    												//��DMA��USART1��ϵ����(����DMA)
	
	usart1_tx_dma.Instance					=	DMA2_Stream7;                         //������ѡ��
	usart1_tx_dma.Init.Channel			=	DMA_CHANNEL_4;                        //ͨ��ѡ��
	usart1_tx_dma.Init.Direction		=	DMA_MEMORY_TO_PERIPH;             		//�洢��������
	usart1_tx_dma.Init.PeriphInc		=	DMA_PINC_DISABLE;                 		//���������ģʽ
	usart1_tx_dma.Init.MemInc				=	DMA_MINC_ENABLE;                     	//�洢������ģʽ
	usart1_tx_dma.Init.PeriphDataAlignment	=	DMA_PDATAALIGN_BYTE;    			//�������ݳ���:8λ
	usart1_tx_dma.Init.MemDataAlignment			=	DMA_MDATAALIGN_BYTE;      		//�洢�����ݳ���:8λ
	usart1_tx_dma.Init.Mode					=	DMA_NORMAL;                           //������ͨģʽ
	usart1_tx_dma.Init.Priority			=	DMA_PRIORITY_MEDIUM;               		//�е����ȼ�
	usart1_tx_dma.Init.FIFOMode			=	DMA_FIFOMODE_DISABLE;              
	usart1_tx_dma.Init.FIFOThreshold=	DMA_FIFO_THRESHOLD_FULL;      
	usart1_tx_dma.Init.MemBurst			=	DMA_MBURST_SINGLE;                 		//�洢��ͻ�����δ���
	usart1_tx_dma.Init.PeriphBurst	=	DMA_PBURST_SINGLE;              			//����ͻ�����δ���
	
	HAL_DMA_DeInit(&usart1_tx_dma);   
	HAL_DMA_Init(&usart1_tx_dma);	
	
	/* UAR1 RX DMA */
	usart1_rx_dma.Instance 					= DMA2_Stream5;
	usart1_rx_dma.Init.Channel 			= DMA_CHANNEL_4;
	usart1_rx_dma.Init.Direction 		= DMA_PERIPH_TO_MEMORY;
	usart1_rx_dma.Init.PeriphInc 		= DMA_PINC_DISABLE;
	usart1_rx_dma.Init.MemInc 			= DMA_MINC_ENABLE;
	usart1_rx_dma.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
	usart1_rx_dma.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
	usart1_rx_dma.Init.Mode 				= DMA_CIRCULAR;
	usart1_rx_dma.Init.Priority 		= DMA_PRIORITY_LOW;
	usart1_rx_dma.Init.FIFOMode 		= DMA_FIFOMODE_DISABLE;
	
	HAL_DMA_DeInit(&usart1_rx_dma);
	HAL_DMA_Init(&usart1_rx_dma);
	__HAL_LINKDMA(&usart1, hdmarx, usart1_rx_dma);
	
	/* UART ��ʼ������	*/
	usart1.Instance						= USART1;												// USART1
	usart1.Init.BaudRate			= USART1_BAUD_RATE;										// ������
	usart1.Init.WordLength		= UART_WORDLENGTH_8B;   				// �ֳ�Ϊ8λ���ݸ�ʽ
	usart1.Init.StopBits			= UART_STOPBITS_1;	    				// һ��ֹͣλ
	usart1.Init.Parity				= UART_PARITY_NONE;							// ����żУ��λ
	usart1.Init.HwFlowCtl			= UART_HWCONTROL_NONE;  				// ��Ӳ������
	usart1.Init.Mode					= UART_MODE_TX_RX;							// �շ�ģʽ
	usart1.Init.OverSampling 	= UART_OVERSAMPLING_16;
	HAL_UART_Init(&usart1);					    											// HAL_UART_Init()��ʹ��UART1
	
	HAL_NVIC_EnableIRQ(USART1_IRQn);													// ʹ��USART1�ж�ͨ��
	HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);									// ���ȼ�		
}


/* 
 * USART1 DMA �жϺ���		
 */
void DMA2_Stream5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&usart1_rx_dma);
}




/* 
 * ����1�жϷ�����		
 */
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&usart1);

	if((__HAL_UART_GET_FLAG(&usart1,UART_FLAG_IDLE) != RESET))	// �����·���У�������һ֡��
	{ 
		__HAL_UART_DISABLE_IT(&usart1, UART_IT_IDLE);							// �رմ��ڿ����ж�
		HAL_UART_DMAStop(&usart1);
		__HAL_UART_CLEAR_IDLEFLAG(&usart1);												// �����·���б�־
		tc_flag = 1;																							// ���ݽ������
	}
}






/*
 * ��ӡ��������
 */
void PrintDataTask(void *arg)
{				
	struct DOF10_t *dof10 = PointDof10RawDataApi();
	struct DOF6_t *dof6 = PointDof6LpfDataApi();
	struct EULAR_t *eular = PointAttiEularApi();
	struct XYZ_t *fpos = PointFusePosApi();
	struct XYZ_t *rpos = PointRawPosApi();
	struct RC_t *rc = PointRcApi();
	struct RCM_t *rcm = PointRcmApi();
	struct XYZ_t *vel = PointFuseVelApi();
	struct XYZ_t *pos = PointFusePosApi();
	struct XYZ_t *pos_raw = PointRawPosApi();
	struct XYZ_t *acce = PointEarthAccApi();
		
	uint16_t *pwm = PointMotorDutyApi();
	portTickType last_t = xTaskGetTickCount();
	
	while (1)
	{
		vTaskDelayUntil(&last_t, 2);
		//SendImuData(dof10->acc.z*100, dof6->acc.z*100, acce->z*100, dof6->gyro.x*100, dof6->gyro.y*100, dof6->gyro.z*100);
		//SendEularAngle(eular->roll*100, eular->pitch*100, eular->yaw*100);
		SendEularAngle(rcm->roll*100, rcm->pitch*100, rcm->yaw_rate*100);
		//SendFourPwmDuty(pwm);
		//SendMagBaroTempeData(pos_raw->z*100, pos->z*100, vel->z*100, 1, 1);		// ����������������ݣ���������߶����ٶ�
		

		//printf("%d %d %d %d %d %d\r\n", rc->joy.roll, rc->joy.pitch, rc->joy.thr, rc->joy.yaw, GetLockStateApi(), GetFlyModeApi());
//	printf("%.3f %.3f %.3f\r\n", dof10->acc.x, dof10->acc.y, dof10->acc.z);
//		printf("%.3f %.3f %.3f\r\n", dof10->mag.x, dof10->mag.y, dof10->mag.z);
	}
}




/*
 * ���ݽ�������
 */
void Usart1RecvDataTask(void *arg)
{
	portTickType last_t;
	
	__HAL_UART_ENABLE_IT(&usart1, UART_IT_IDLE);																				// �������ڿ����ж�
  HAL_UART_Receive_DMA(&usart1, (uint8_t*)usart1_rx_buf, USART1_RX_MAX_LEN);
	
	last_t = xTaskGetTickCount();
	
	while (1)
	{
		vTaskDelayUntil(&last_t, 2);
		
		if (tc_flag == 1)
		{	
			printf("receive data: %s\r\n", usart1_rx_buf);
			memset(usart1_rx_buf, 0, USART1_RX_MAX_LEN);
			
			__HAL_UART_ENABLE_IT(&usart1, UART_IT_IDLE);																		// �������ڿ����ж�
			HAL_UART_Receive_DMA(&usart1, (uint8_t*)usart1_rx_buf, USART1_RX_MAX_LEN);			// �������ݽ���
		
			tc_flag = 0;
		}
	}
}



/*
 * ��ӡ����API
 */
void PrintDataApi(uint8_t *data, uint8_t len)
{
	uint8_t t_delay = 0;
	
	HAL_UART_Transmit_DMA(&usart1, data, len);									//��������
	t_delay = (uint8_t)((uint32_t)(10 * 1000 * (uint32_t)len / (uint32_t)USART1_BAUD_RATE) + 1);						// 1Byte=10Bit 1s=1000ms  
	vTaskDelay(t_delay);																				// ����ʱ��ȥ�������£��ٵȴ����ݷ�����
	
	while(1)
	{
		if(__HAL_DMA_GET_FLAG(&usart1_tx_dma,DMA_FLAG_TCIF3_7))		//�ȴ�DMA2_Steam7�������
		{
			__HAL_DMA_CLEAR_FLAG(&usart1_tx_dma,DMA_FLAG_TCIF3_7);	//���DMA2_Steam7������ɱ�־
			HAL_UART_DMAStop(&usart1);      												//��������Ժ�رմ���DMA
			break; 
		}
	}
}

/********************************************************************/
/* 							һЩ��ʱ���ù��ĵĺ���																*/

#if 1
#pragma import(__use_no_semihosting)   

/*
 * ��׼����Ҫ��֧�ֺ���   
 */             
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;  

/* 
 * ����_sys_exit()�Ա���ʹ�ð�����ģʽ    
 */
void _sys_exit(int x) 
{ 
	x = x; 
} 

/* 
 * �ض���fputc���� 
 * ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ��� 
 */
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (uint8_t) ch;      
	return ch;
}


#endif 
