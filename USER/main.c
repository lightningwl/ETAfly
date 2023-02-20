#include <math.h>
#include "sys.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "rgbled.h"
#include "dof10.h"
#include "atti_esti.h"
#include "pos_esti.h"
#include "motor.h"
#include "rc.h"
#include "control.h"

int main(void)
{
	HAL_Init();
	SysClockInit(336, 8, 2, 7);																									// 168M, 外部晶振为8M
	DelayInit(168);
	

	USART1_Init();
	TIM1_PWM_Init();
	
	printf("hello world\r\n");
	
	taskENTER_CRITICAL();																												// 进入临界区，避免创建任务被打断

	xTaskCreate(Dof10Task, "Dof10Task", 512,	NULL, 5, NULL);										// 十轴传感器任务
	xTaskCreate(AttiEstiTask, "AttiEstiTask", 1024,	NULL, 5, NULL);							// 姿态估计任务
	xTaskCreate(RcTask, "RcTask", 256,	NULL, 4, NULL);													// 遥控器任务
	xTaskCreate(PosEstiTask, "PosEstiTask", 512,	NULL, 3, NULL);								// 位置估计任务

	xTaskCreate(ControlTask, "ControlTask", 512,	NULL, 4, NULL);								// 控制任务
	xTaskCreate(PrintDataTask, "PrintDataTask", 512,	NULL, 1, NULL);						// 数据打印任务
	xTaskCreate(RgbLedTask, "RgbLedTask", 128,	NULL, 1, NULL);									// RGBLED任务

	
	taskEXIT_CRITICAL();																												// 退出临界区	
	vTaskStartScheduler();																											// 开始任务调度
				
	return 0;
}


