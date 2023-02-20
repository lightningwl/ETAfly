#include "rgbled.h"
#include "FreeRTOS.h"
#include "task.h"


#define LED_NUM			4																			// 灯珠数量
#define BUF_SIZE		24 * LED_NUM+2												// 缓冲区大小 RGB888

#define TIM_ARR			105-1																	// 84M / 105 = 800kHz = 1.25us
#define TIM_1  			71																		// 高电平时间 71 / 105 * 1.25 = 0.845
#define TIM_0 			34																		// 低电平时间 34 / 105 * 1.25 = 0.405


GPIO_InitTypeDef 		GPIO_Initure;													// GPIO初始化结构体
TIM_HandleTypeDef 	TIM3_Handler;         								// 定时器1 PWM句柄 
TIM_OC_InitTypeDef 	TIM3_CHxHandler;	    								// 定时器1通道x句柄
DMA_HandleTypeDef 	DMA_InitStructure;										// DMA初始化结构体

static uint16_t led_buf[BUF_SIZE] = {0};									// LED数据存储缓冲区


/* 
 * TIM3 CH3
 * PB0
 * DMA1 CH5 Stream7
 */
void TIM3_PWM_Init(void)
{ 

	// 1. 开启时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();	
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	
	// 2. GPIO配置
	GPIO_Initure.Pin				= GPIO_PIN_0; 
	GPIO_Initure.Mode				= GPIO_MODE_AF_PP;  					
	GPIO_Initure.Pull				= GPIO_PULLUP;          	
	GPIO_Initure.Speed			= GPIO_SPEED_HIGH;     		
	GPIO_Initure.Alternate	= GPIO_AF2_TIM3;			
  HAL_GPIO_Init(GPIOB, &GPIO_Initure);	
	
	// 3. 定时器配置
	TIM3_Handler.Instance						= TIM3;            					// 定时器3
	TIM3_Handler.Init.Prescaler			= 1-1;       								// 定时器分频 21，得到 84M/21=4M 时钟
	TIM3_Handler.Init.CounterMode		= TIM_COUNTERMODE_UP;				// 向上计数模式
	TIM3_Handler.Init.Period				= TIM_ARR;          				// 自动重装载值 5，得到PWM频率 4M/5=800kHz
	TIM3_Handler.Init.ClockDivision	= TIM_CLOCKDIVISION_DIV1;		// 分频因子
	HAL_TIM_PWM_Init(&TIM3_Handler);       											// 初始化PWM
	
	// 4. PWM配置
	TIM3_CHxHandler.OCMode					= TIM_OCMODE_PWM1; 					// 模式选择PWM1
	TIM3_CHxHandler.Pulse						= TIM_1;         						// 设置比较值,此值用来确定占空比，默认为 1ms时间，用于驱动电调
	TIM3_CHxHandler.OCPolarity			= TIM_OCPOLARITY_HIGH; 			// 输出比较极性为高
	TIM3_CHxHandler.OCFastMode 			= TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&TIM3_Handler, &TIM3_CHxHandler, TIM_CHANNEL_3);	//配置TIM3通道3


	// 5. DMA配置
	DMA_InitStructure.Instance 									= DMA1_Stream7;
	DMA_InitStructure.Init.Channel  						= DMA_CHANNEL_5;
	DMA_InitStructure.Init.Direction 						= DMA_MEMORY_TO_PERIPH;
	DMA_InitStructure.Init.PeriphInc 						= DMA_PINC_DISABLE;
	DMA_InitStructure.Init.MemInc 							= DMA_MINC_ENABLE;
	DMA_InitStructure.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_HALFWORD;
	DMA_InitStructure.Init.MemDataAlignment 		= DMA_PDATAALIGN_HALFWORD;
	DMA_InitStructure.Init.Mode 								= DMA_NORMAL;
	DMA_InitStructure.Init.Priority 						= DMA_PRIORITY_MEDIUM;
	DMA_InitStructure.Init.FIFOMode 						= DMA_FIFOMODE_DISABLE;
	
	HAL_DMA_Init(&DMA_InitStructure);	
	
	__HAL_LINKDMA(&TIM3_Handler, hdma[TIM_DMA_ID_CC3], DMA_InitStructure);

	HAL_NVIC_SetPriority(TIM3_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}


void DMA1_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&DMA_InitStructure);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TIM3_Handler);
}

/* 
 * 中断回调函数
 * 设定的pwm通过DMA发送完成后会调用 
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
  __HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, 0); //占空比清0，若不清会导致灯珠颜色不对 
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
}



/*
 * 设置RGB灯颜色 
 */
void SetLedColor(uint8_t (*color)[3], uint16_t len)
{
	uint8_t i;
	uint16_t memaddr = 0;

	led_buf[memaddr++] = 0;
	// 获取数据
	while (len)
	{	
		led_buf[memaddr++] = 0;
		for(i=0; i<8; i++) // GREEN
		{
			led_buf[memaddr++] = ((color[0][1]<<i) & 0x0080) ? TIM_1:TIM_0;
		}
		for(i=0; i<8; i++) // RED
		{
			led_buf[memaddr++] = ((color[0][0]<<i) & 0x0080) ? TIM_1:TIM_0;
		}
		for(i=0; i<8; i++) // BLUE
		{
			led_buf[memaddr++] = ((color[0][2]<<i) & 0x0080) ? TIM_1:TIM_0;
		}
		len--;
	}
	led_buf[memaddr] = 0;
	
	HAL_TIM_PWM_Start_DMA(&TIM3_Handler, TIM_CHANNEL_3, (uint32_t*)led_buf, BUF_SIZE);
}


/*
 * RGB颜色设置任务
 */
void RgbLedTask(void *arg)
{
	uint8_t r[][3] = {255,0,0};
	uint8_t g[][3] = {0,255,0};
	uint8_t b[][3] = {0,0,255};

	
	TIM3_PWM_Init();

	while (1)
	{
		SetLedColor(r, 4);
		vTaskDelay(500);
		
		SetLedColor(g, 4);
		vTaskDelay(500);
		
		SetLedColor(b, 4);
		vTaskDelay(500);
	}
}






