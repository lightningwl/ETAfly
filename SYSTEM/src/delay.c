#include "delay.h"
#include "sys.h"
#include "FreeRTOS.h"					 
#include "task.h"

static uint32_t time_us=0;							//us延时倍乘数
static u16 time_ms=0;				        	//ms延时倍乘数,在os下,代表每个节拍的ms数


extern void xPortSysTickHandler(void);
void SysTick_Handler(void)
{
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xPortSysTickHandler();
	}
	//HAL_IncTick();						// F429需要, F407不需要
}
			   
//初始化延迟函数
void DelayInit(u8 SYSCLK)
{
	uint32_t reload;
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	time_us = SYSCLK;
	reload = SYSCLK;
	reload *= 1000000 / configTICK_RATE_HZ;
	time_ms = 1000 / configTICK_RATE_HZ;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->LOAD = reload;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void DelayUs(uint32_t nus)
{
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD;
	ticks = nus * time_us;
	told = SysTick->VAL;
	while(1)
	{
		tnow = SysTick->VAL;
		if (tnow != told)
		{
			if (tnow < told)
				tcnt += (told - tnow);
			else
				tcnt += (reload - tnow + told);
			told = tnow;
			if (tcnt >= ticks)
				break;
		}
	};
}

void DelayMs(uint32_t nms)
{
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		if (nms >= time_ms)
		{
			vTaskDelay(nms / time_ms);
		}
		nms %= time_ms;
	}
	DelayUs((uint32_t)(nms*1000));
}

