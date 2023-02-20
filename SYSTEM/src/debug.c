#include "debug.h"
#include "delay.h"
#include <string.h>
#include "sys.h"

/**************************************************************************************************************/
/*****************************************  PART1：串口接收底层配置	  *****************************************/

static uint16_t usart_rx_sta=0;    						//接收状态, bit15: 接收完成标志;	bit14: 接收到0x0d; bit13~0: 接收到的有效字节数目
static uint8_t usart_rx_buf[USART_MAX_RX_LEN];     		//接收缓冲,最大USART_MAXRX_LEN个字节.
static uint8_t usart_buf[USART_BUFFER_SIZE];				//HAL库使用的串口接收缓冲

UART_HandleTypeDef usart1; 							//UART句柄

/* 初始化IO 使用串口1 		*/
/* bound:波特率				*/
void DebugInit(uint32_t bound)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
		
	DEBUG_ENCLK();											// 使能时钟
	
	GPIO_Initure.Pin		= DEBUG_PIN;					// Pin
	GPIO_Initure.Mode		= GPIO_MODE_AF_PP;				// 复用推挽输出
	GPIO_Initure.Pull		= GPIO_PULLUP;					// 上拉
	GPIO_Initure.Speed		= GPIO_SPEED_FAST;				// 高速
	GPIO_Initure.Alternate	= GPIO_AF7_USART1;				// 复用为USART1
	HAL_GPIO_Init(DEBUG_GPIO,&GPIO_Initure);	   					// 初始化
		
	//UART 初始化设置	
	usart1.Instance			= USART1;				// USART1
	usart1.Init.BaudRate	= bound;				// 波特率
	usart1.Init.WordLength	= UART_WORDLENGTH_8B;   // 字长为8位数据格式
	usart1.Init.StopBits	= UART_STOPBITS_1;	    // 一个停止位
	usart1.Init.Parity		= UART_PARITY_NONE;		// 无奇偶校验位
	usart1.Init.HwFlowCtl	= UART_HWCONTROL_NONE;  // 无硬件流控
	usart1.Init.Mode		= UART_MODE_TX_RX;		// 收发模式
	usart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&usart1);					    	// HAL_UART_Init()会使能UART2
	
	HAL_NVIC_EnableIRQ(USART1_IRQn);						// 使能USART1中断通道
	HAL_NVIC_SetPriority(USART1_IRQn, 3, 4);				// 抢占优先级3，子优先级3	
	HAL_UART_Receive_IT(&usart1, (uint8_t *)usart_buf, USART_BUFFER_SIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}

 
/* 串口1中断服务程序		*/
void USART1_IRQHandler(void)                	
{ 
	if((usart_rx_sta&0x8000)==0)							//接收未完成
	{
		if(usart_rx_sta&0x4000)								//接收到了0x0d
		{
			if(usart_buf[0]!=0x0a)
				usart_rx_sta=0;								//接收错误,重新开始
			else 
				usart_rx_sta|=0x8000;						//接收完成了 
		}
		else 												//还没收到0X0D
		{	
			if(usart_buf[0]==0x0d)
				usart_rx_sta|=0x4000;
			else
			{
				usart_rx_buf[usart_rx_sta & 0X3FFF]=usart_buf[0] ;
				usart_rx_sta++;
				if(usart_rx_sta>(USART_MAX_RX_LEN-1))
					usart_rx_sta=0;							//接收数据错误,重新开始接收	  
			}		 
		}
	}
	HAL_UART_IRQHandler(&usart1);							//调用HAL库中断处理公用函数
	HAL_UART_Receive_IT(&usart1, (uint8_t *)usart_buf, USART_BUFFER_SIZE);//一次处理完成之后，重新开启中断并设置usart_buf为1
} 


/* 连续打印数据任务						*/
/* 调试使用，很多数据定义为使用，不必纠结	*/
void PrintDataTask(void *arg)
{
	while (1)
	{
		delay_ms(200);
	}
}


/* 将数据发送到匿名上位机，可显示波形											*/
/* 一次可传输九个数据，因为0x02功能字包括9个数据，可参考匿名通信协议			*/
/* 注意，此处要将遥控器CH5打到 1档，数据才上传，打到2档是紧急停止，不会传输数据	*/
void AnoDebugTask(void *arg)
{
	while (1)
	{
		delay_ms(1000);
	}
}


/********************************************************************/
/* 							一些暂时不用关心的函数					*/
#if 1
#pragma import(__use_no_semihosting)   
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
//注意,读取USARTx->SR能避免莫名其妙的错误   	
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (uint8_t) ch;      
	return ch;
}


#endif 


