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
	SysClockInit(336, 8, 2, 7);																									// 168M, �ⲿ����Ϊ8M
	DelayInit(168);
	

	USART1_Init();
	TIM1_PWM_Init();
	
	printf("hello world\r\n");
	
	taskENTER_CRITICAL();																												// �����ٽ��������ⴴ�����񱻴��

	xTaskCreate(Dof10Task, "Dof10Task", 512,	NULL, 5, NULL);										// ʮ�ᴫ��������
	xTaskCreate(AttiEstiTask, "AttiEstiTask", 1024,	NULL, 5, NULL);							// ��̬��������
	xTaskCreate(RcTask, "RcTask", 256,	NULL, 4, NULL);													// ң��������
	xTaskCreate(PosEstiTask, "PosEstiTask", 512,	NULL, 3, NULL);								// λ�ù�������

	xTaskCreate(ControlTask, "ControlTask", 512,	NULL, 4, NULL);								// ��������
	xTaskCreate(PrintDataTask, "PrintDataTask", 512,	NULL, 1, NULL);						// ���ݴ�ӡ����
	xTaskCreate(RgbLedTask, "RgbLedTask", 128,	NULL, 1, NULL);									// RGBLED����

	
	taskEXIT_CRITICAL();																												// �˳��ٽ���	
	vTaskStartScheduler();																											// ��ʼ�������
				
	return 0;
}


