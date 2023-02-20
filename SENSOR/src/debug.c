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


volatile uint8_t tc_flag = 0;																		// 输出传输完成标志位，
																																// 当串口进入空闲中断 IDLE 时，说明数据传输完成，置1
																																// 处理完数据后，置0，可继续接收数据
uint8_t usart1_rx_buf[USART1_RX_MAX_LEN];


/*****************************************  PART1：串口接收底层配置	  *****************************************/

#define USART1_BAUD_RATE	921600
		
												
#define USART1_PIN			GPIO_PIN_9 | GPIO_PIN_10
#define USART1_GPIO			GPIOA
#define USART1_ENCLK()	__HAL_RCC_GPIOA_CLK_ENABLE();	\
												__HAL_RCC_USART1_CLK_ENABLE();					//使能GPIOA时钟,USART1时钟	


UART_HandleTypeDef 	usart1; 																		//UART句柄
DMA_HandleTypeDef  	usart1_tx_dma;      												//DMA句柄
DMA_HandleTypeDef 	usart1_rx_dma;


/* 
 * 初始化IO 使用串口1 		
 * bound:波特率				
 */
void USART1_Init(void)
{
  /* GPIO端口设置 */
	GPIO_InitTypeDef GPIO_Initure;
		
	USART1_ENCLK();																													// 使能时钟
	
	GPIO_Initure.Pin				= USART1_PIN;																		// Pin
	GPIO_Initure.Mode				= GPIO_MODE_AF_PP;															// 复用推挽输出
	GPIO_Initure.Pull				= GPIO_PULLUP;																	// 上拉
	GPIO_Initure.Speed			= GPIO_SPEED_FAST;															// 高速
	GPIO_Initure.Alternate	= GPIO_AF7_USART1;															// 复用为USART1
	HAL_GPIO_Init(USART1_GPIO, &GPIO_Initure);	   													// 初始化
			
	/* UART1 TX DMA配置--DMA2 Stream7 Channel4 */
	__HAL_RCC_DMA2_CLK_ENABLE();																						//DMA2时钟使能	
	__HAL_LINKDMA(&usart1,hdmatx,usart1_tx_dma);    												//将DMA与USART1联系起来(发送DMA)
	
	usart1_tx_dma.Instance					=	DMA2_Stream7;                         //数据流选择
	usart1_tx_dma.Init.Channel			=	DMA_CHANNEL_4;                        //通道选择
	usart1_tx_dma.Init.Direction		=	DMA_MEMORY_TO_PERIPH;             		//存储器到外设
	usart1_tx_dma.Init.PeriphInc		=	DMA_PINC_DISABLE;                 		//外设非增量模式
	usart1_tx_dma.Init.MemInc				=	DMA_MINC_ENABLE;                     	//存储器增量模式
	usart1_tx_dma.Init.PeriphDataAlignment	=	DMA_PDATAALIGN_BYTE;    			//外设数据长度:8位
	usart1_tx_dma.Init.MemDataAlignment			=	DMA_MDATAALIGN_BYTE;      		//存储器数据长度:8位
	usart1_tx_dma.Init.Mode					=	DMA_NORMAL;                           //外设普通模式
	usart1_tx_dma.Init.Priority			=	DMA_PRIORITY_MEDIUM;               		//中等优先级
	usart1_tx_dma.Init.FIFOMode			=	DMA_FIFOMODE_DISABLE;              
	usart1_tx_dma.Init.FIFOThreshold=	DMA_FIFO_THRESHOLD_FULL;      
	usart1_tx_dma.Init.MemBurst			=	DMA_MBURST_SINGLE;                 		//存储器突发单次传输
	usart1_tx_dma.Init.PeriphBurst	=	DMA_PBURST_SINGLE;              			//外设突发单次传输
	
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
	
	/* UART 初始化设置	*/
	usart1.Instance						= USART1;												// USART1
	usart1.Init.BaudRate			= USART1_BAUD_RATE;										// 波特率
	usart1.Init.WordLength		= UART_WORDLENGTH_8B;   				// 字长为8位数据格式
	usart1.Init.StopBits			= UART_STOPBITS_1;	    				// 一个停止位
	usart1.Init.Parity				= UART_PARITY_NONE;							// 无奇偶校验位
	usart1.Init.HwFlowCtl			= UART_HWCONTROL_NONE;  				// 无硬件流控
	usart1.Init.Mode					= UART_MODE_TX_RX;							// 收发模式
	usart1.Init.OverSampling 	= UART_OVERSAMPLING_16;
	HAL_UART_Init(&usart1);					    											// HAL_UART_Init()会使能UART1
	
	HAL_NVIC_EnableIRQ(USART1_IRQn);													// 使能USART1中断通道
	HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);									// 优先级		
}


/* 
 * USART1 DMA 中断函数		
 */
void DMA2_Stream5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&usart1_rx_dma);
}




/* 
 * 串口1中断服务函数		
 */
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&usart1);

	if((__HAL_UART_GET_FLAG(&usart1,UART_FLAG_IDLE) != RESET))	// 如果线路空闲（接收完一帧）
	{ 
		__HAL_UART_DISABLE_IT(&usart1, UART_IT_IDLE);							// 关闭串口空闲中断
		HAL_UART_DMAStop(&usart1);
		__HAL_UART_CLEAR_IDLEFLAG(&usart1);												// 清除线路空闲标志
		tc_flag = 1;																							// 数据接收完成
	}
}






/*
 * 打印数据任务
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
		//SendMagBaroTempeData(pos_raw->z*100, pos->z*100, vel->z*100, 1, 1);		// 本来传输磁力计数据，用来传输高度与速度
		

		//printf("%d %d %d %d %d %d\r\n", rc->joy.roll, rc->joy.pitch, rc->joy.thr, rc->joy.yaw, GetLockStateApi(), GetFlyModeApi());
//	printf("%.3f %.3f %.3f\r\n", dof10->acc.x, dof10->acc.y, dof10->acc.z);
//		printf("%.3f %.3f %.3f\r\n", dof10->mag.x, dof10->mag.y, dof10->mag.z);
	}
}




/*
 * 数据接收任务
 */
void Usart1RecvDataTask(void *arg)
{
	portTickType last_t;
	
	__HAL_UART_ENABLE_IT(&usart1, UART_IT_IDLE);																				// 开启串口空闲中断
  HAL_UART_Receive_DMA(&usart1, (uint8_t*)usart1_rx_buf, USART1_RX_MAX_LEN);
	
	last_t = xTaskGetTickCount();
	
	while (1)
	{
		vTaskDelayUntil(&last_t, 2);
		
		if (tc_flag == 1)
		{	
			printf("receive data: %s\r\n", usart1_rx_buf);
			memset(usart1_rx_buf, 0, USART1_RX_MAX_LEN);
			
			__HAL_UART_ENABLE_IT(&usart1, UART_IT_IDLE);																		// 开启串口空闲中断
			HAL_UART_Receive_DMA(&usart1, (uint8_t*)usart1_rx_buf, USART1_RX_MAX_LEN);			// 启动数据接收
		
			tc_flag = 0;
		}
	}
}



/*
 * 打印数据API
 */
void PrintDataApi(uint8_t *data, uint8_t len)
{
	uint8_t t_delay = 0;
	
	HAL_UART_Transmit_DMA(&usart1, data, len);									//启动传输
	t_delay = (uint8_t)((uint32_t)(10 * 1000 * (uint32_t)len / (uint32_t)USART1_BAUD_RATE) + 1);						// 1Byte=10Bit 1s=1000ms  
	vTaskDelay(t_delay);																				// 先延时，去干其他事，再等待数据发送完
	
	while(1)
	{
		if(__HAL_DMA_GET_FLAG(&usart1_tx_dma,DMA_FLAG_TCIF3_7))		//等待DMA2_Steam7传输完成
		{
			__HAL_DMA_CLEAR_FLAG(&usart1_tx_dma,DMA_FLAG_TCIF3_7);	//清除DMA2_Steam7传输完成标志
			HAL_UART_DMAStop(&usart1);      												//传输完成以后关闭串口DMA
			break; 
		}
	}
}

/********************************************************************/
/* 							一些暂时不用关心的函数																*/

#if 1
#pragma import(__use_no_semihosting)   

/*
 * 标准库需要的支持函数   
 */             
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;  

/* 
 * 定义_sys_exit()以避免使用半主机模式    
 */
void _sys_exit(int x) 
{ 
	x = x; 
} 

/* 
 * 重定义fputc函数 
 * 注意,读取USARTx->SR能避免莫名其妙的错误 
 */
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (uint8_t) ch;      
	return ch;
}


#endif 
