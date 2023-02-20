#include "rgbled.h"
#include "FreeRTOS.h"
#include "task.h"


#define LED_NUM			4																			// ��������
#define BUF_SIZE		24 * LED_NUM+2												// ��������С RGB888

#define TIM_ARR			105-1																	// 84M / 105 = 800kHz = 1.25us
#define TIM_1  			71																		// �ߵ�ƽʱ�� 71 / 105 * 1.25 = 0.845
#define TIM_0 			34																		// �͵�ƽʱ�� 34 / 105 * 1.25 = 0.405


GPIO_InitTypeDef 		GPIO_Initure;													// GPIO��ʼ���ṹ��
TIM_HandleTypeDef 	TIM3_Handler;         								// ��ʱ��1 PWM��� 
TIM_OC_InitTypeDef 	TIM3_CHxHandler;	    								// ��ʱ��1ͨ��x���
DMA_HandleTypeDef 	DMA_InitStructure;										// DMA��ʼ���ṹ��

static uint16_t led_buf[BUF_SIZE] = {0};									// LED���ݴ洢������


/* 
 * TIM3 CH3
 * PB0
 * DMA1 CH5 Stream7
 */
void TIM3_PWM_Init(void)
{ 

	// 1. ����ʱ��
	__HAL_RCC_GPIOB_CLK_ENABLE();	
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	
	// 2. GPIO����
	GPIO_Initure.Pin				= GPIO_PIN_0; 
	GPIO_Initure.Mode				= GPIO_MODE_AF_PP;  					
	GPIO_Initure.Pull				= GPIO_PULLUP;          	
	GPIO_Initure.Speed			= GPIO_SPEED_HIGH;     		
	GPIO_Initure.Alternate	= GPIO_AF2_TIM3;			
  HAL_GPIO_Init(GPIOB, &GPIO_Initure);	
	
	// 3. ��ʱ������
	TIM3_Handler.Instance						= TIM3;            					// ��ʱ��3
	TIM3_Handler.Init.Prescaler			= 1-1;       								// ��ʱ����Ƶ 21���õ� 84M/21=4M ʱ��
	TIM3_Handler.Init.CounterMode		= TIM_COUNTERMODE_UP;				// ���ϼ���ģʽ
	TIM3_Handler.Init.Period				= TIM_ARR;          				// �Զ���װ��ֵ 5���õ�PWMƵ�� 4M/5=800kHz
	TIM3_Handler.Init.ClockDivision	= TIM_CLOCKDIVISION_DIV1;		// ��Ƶ����
	HAL_TIM_PWM_Init(&TIM3_Handler);       											// ��ʼ��PWM
	
	// 4. PWM����
	TIM3_CHxHandler.OCMode					= TIM_OCMODE_PWM1; 					// ģʽѡ��PWM1
	TIM3_CHxHandler.Pulse						= TIM_1;         						// ���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ��Ϊ 1msʱ�䣬�����������
	TIM3_CHxHandler.OCPolarity			= TIM_OCPOLARITY_HIGH; 			// ����Ƚϼ���Ϊ��
	TIM3_CHxHandler.OCFastMode 			= TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&TIM3_Handler, &TIM3_CHxHandler, TIM_CHANNEL_3);	//����TIM3ͨ��3


	// 5. DMA����
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
 * �жϻص�����
 * �趨��pwmͨ��DMA������ɺ����� 
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
  __HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, 0); //ռ�ձ���0��������ᵼ�µ�����ɫ���� 
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
}



/*
 * ����RGB����ɫ 
 */
void SetLedColor(uint8_t (*color)[3], uint16_t len)
{
	uint8_t i;
	uint16_t memaddr = 0;

	led_buf[memaddr++] = 0;
	// ��ȡ����
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
 * RGB��ɫ��������
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






