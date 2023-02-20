/***************************************************************************

  ��� 4·PWM
  TIM1 CH1 -- PE9
  TIM1 CH2 -- PE11
  TIM1 CH3 -- PE13
  TIM1 CH4 -- PB14
 
****************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "sys.h"
#include "delay.h"



#define TIM1_PWM_PIN				GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14
#define TIM1_PWM_GPIO				GPIOE
#define TIM1_PWM_ENCLK()		__HAL_RCC_TIM1_CLK_ENABLE();	\
														__HAL_RCC_GPIOE_CLK_ENABLE();										
			
#define TIM1_PWM_PSR				2 - 1													// Ԥ����ϵ�����õ���Ƶʱ�� 168M / 3 = 56M
#define TIM1_PWM_ARR				200 - 1												// �Զ���װֵ���õ�����Ƶ�� 56M / 200 = 280k Hz



#define PWM_DUTY_MIN		  	0															// PWM���������Сռ�ձ�
#define PWM_DUTY_MAX		  	200														// PWM����������ռ�ձ�
#define PWM_DUTY_SAFE 			10														// ���΢΢ת��


TIM_HandleTypeDef TIM1_Handler;         									//��ʱ��1 PWM��� 
TIM_OC_InitTypeDef TIM1_CHxHandler;	    									//��ʱ��1ͨ��x���

/* 
 * TIM1 4· PWM�������ʼ�� 		
 */

void TIM1_PWM_Init(void)
{ 
	TIM1_Handler.Instance						= TIM1;            					// ��ʱ��3
	TIM1_Handler.Init.Prescaler			= TIM1_PWM_PSR;       			// ��ʱ����Ƶ 45���õ� 84M/42=2M ʱ��
	TIM1_Handler.Init.CounterMode		= TIM_COUNTERMODE_UP;				// ���ϼ���ģʽ
	TIM1_Handler.Init.Period				= TIM1_PWM_ARR;          		// �Զ���װ��ֵ 5000���õ�PWMƵ�� 2M/5k=400Hz
	TIM1_Handler.Init.ClockDivision	= TIM_CLOCKDIVISION_DIV1;		// ��Ƶ����
	HAL_TIM_PWM_Init(&TIM1_Handler);       											// ��ʼ��PWM
	
	TIM1_CHxHandler.OCMode					= TIM_OCMODE_PWM1; 					// ģʽѡ��PWM1
	TIM1_CHxHandler.Pulse						= PWM_DUTY_MIN;         		// ���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ��Ϊ 1msʱ�䣬�����������
	TIM1_CHxHandler.OCPolarity			= TIM_OCPOLARITY_HIGH; 			// ����Ƚϼ���Ϊ��

	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CHxHandler,TIM_CHANNEL_1);	//����TIM3ͨ��1
	HAL_TIM_PWM_Start(&TIM1_Handler, TIM_CHANNEL_1);													//����PWMͨ��1
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CHxHandler,TIM_CHANNEL_2);	//����TIM3ͨ��2
  HAL_TIM_PWM_Start(&TIM1_Handler, TIM_CHANNEL_2);													//����PWMͨ��2
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CHxHandler,TIM_CHANNEL_3);	//����TIM3ͨ��3
  HAL_TIM_PWM_Start(&TIM1_Handler, TIM_CHANNEL_3);													//����PWMͨ��3
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CHxHandler,TIM_CHANNEL_4);	//����TIM3ͨ��4
  HAL_TIM_PWM_Start(&TIM1_Handler, TIM_CHANNEL_4);													//����PWMͨ��4																										
}

/* 
 * ��ʱ���ײ�������ʱ��ʹ�ܣ���������	
 * �˺����ᱻHAL_TIM_PWM_Init()����	
 */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_Initure;
	
	TIM1_PWM_ENCLK();
	
	GPIO_Initure.Pin				= TIM1_PWM_PIN; 
	GPIO_Initure.Mode				= GPIO_MODE_AF_PP;  					
	GPIO_Initure.Pull				= GPIO_PULLUP;          	
	GPIO_Initure.Speed			= GPIO_SPEED_HIGH;     		
	GPIO_Initure.Alternate	= GPIO_AF1_TIM1;			
  HAL_GPIO_Init(TIM1_PWM_GPIO, &GPIO_Initure);
}


/* 
 * ����ָ�����ռ�ձ�										
 * motor: ѡ����������MOTOR1, MOTOR2, MOTOR3, MOTOR4	
 * ������1ms-2ms��������Χ��Ч				
 * pwm ��λ��ms������Ϊ1000��Ϊ1ms
 */

void SetMotorDutyApi(enum MOTOR_m motor, uint16_t pwm)
{
	if (pwm > PWM_DUTY_MAX)														// ��������� 2ms
		pwm = PWM_DUTY_MAX;
	else if (pwm <= PWM_DUTY_MIN)											// ��С������ 1ms
		pwm = PWM_DUTY_MIN;
	
	if (motor == MOTOR1)
		TIM1->CCR2 = pwm;																// �趨 CH1 �����Ƚ�ֵ
	else if (motor == MOTOR2)
		TIM1->CCR1 = pwm;																// �趨 CH2 �����Ƚ�ֵ
	else if (motor == MOTOR3)
		TIM1->CCR4 = pwm;																// �趨 CH3 �����Ƚ�ֵ
	else if (motor == MOTOR4) 
		TIM1->CCR3 = pwm;																// �趨 CH4 �����Ƚ�ֵ	
}

/* 
 * �����ĸ����ͣת���������ʱ�Ͽ���� (^_^)				
 */
void SetMotorsStopApi(void)
{
	SetMotorDutyApi(MOTOR1, PWM_DUTY_MIN);						// ����API�����õ��1ռ�ձ�
	SetMotorDutyApi(MOTOR2, PWM_DUTY_MIN);						// ����API�����õ��2ռ�ձ�
	SetMotorDutyApi(MOTOR3, PWM_DUTY_MIN);						// ����API�����õ��3ռ�ձ�
	SetMotorDutyApi(MOTOR4, PWM_DUTY_MIN);						// ����API�����õ��4ռ�ձ�		
}

/*
 * �ĸ������������ת��
 */
void SetFourMotorsSpinSlowly(void)
{
	SetMotorDutyApi(MOTOR1, PWM_DUTY_SAFE);						// ����API�����õ��1ռ�ձ�
	vTaskDelay(500);	
	SetMotorDutyApi(MOTOR2, PWM_DUTY_SAFE);						// ����API�����õ��2ռ�ձ�
	vTaskDelay(500);
	SetMotorDutyApi(MOTOR3, PWM_DUTY_SAFE);						// ����API�����õ��3ռ�ձ�
	vTaskDelay(500);
	SetMotorDutyApi(MOTOR4, PWM_DUTY_SAFE);						// ����API�����õ��4ռ�ձ�
	vTaskDelay(500);	
}

/*
 * ���õ��ռ�ձȣ��ĸ����ռ�ձ���ͬ
 * duty:ms
 */
void SetMotorsDutyApi(uint16_t duty)
{
	SetMotorDutyApi(MOTOR1, duty);		// ����API�����õ��1ռ�ձ�
	SetMotorDutyApi(MOTOR2, duty);		// ����API�����õ��2ռ�ձ�
	SetMotorDutyApi(MOTOR3, duty);		// ����API�����õ��3ռ�ձ�
	SetMotorDutyApi(MOTOR4, duty);		// ����API�����õ��4ռ�ձ�				
}


/*
 * �����������
 */
void MotorTestTask(void *arg)
{
	TIM1_PWM_Init();
	
	while (1)
	{
		SetFourMotorsSpinSlowly();		// �ĸ��������ת��
		vTaskDelay(1000);
		SetMotorsDutyApi(0);					// ռ�ձ�Ϊ0�����ͣת
		vTaskDelay(3000);
	}
}
