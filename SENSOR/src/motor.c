/***************************************************************************

  输出 4路PWM
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
			
#define TIM1_PWM_PSR				2 - 1													// 预分配系数，得到分频时钟 168M / 3 = 56M
#define TIM1_PWM_ARR				200 - 1												// 自动重装值，得到计数频率 56M / 200 = 280k Hz



#define PWM_DUTY_MIN		  	0															// PWM驱动电调最小占空比
#define PWM_DUTY_MAX		  	200														// PWM驱动电调最大占空比
#define PWM_DUTY_SAFE 			10														// 电机微微转动


TIM_HandleTypeDef TIM1_Handler;         									//定时器1 PWM句柄 
TIM_OC_InitTypeDef TIM1_CHxHandler;	    									//定时器1通道x句柄

/* 
 * TIM1 4路 PWM波输出初始化 		
 */

void TIM1_PWM_Init(void)
{ 
	TIM1_Handler.Instance						= TIM1;            					// 定时器3
	TIM1_Handler.Init.Prescaler			= TIM1_PWM_PSR;       			// 定时器分频 45，得到 84M/42=2M 时钟
	TIM1_Handler.Init.CounterMode		= TIM_COUNTERMODE_UP;				// 向上计数模式
	TIM1_Handler.Init.Period				= TIM1_PWM_ARR;          		// 自动重装载值 5000，得到PWM频率 2M/5k=400Hz
	TIM1_Handler.Init.ClockDivision	= TIM_CLOCKDIVISION_DIV1;		// 分频因子
	HAL_TIM_PWM_Init(&TIM1_Handler);       											// 初始化PWM
	
	TIM1_CHxHandler.OCMode					= TIM_OCMODE_PWM1; 					// 模式选择PWM1
	TIM1_CHxHandler.Pulse						= PWM_DUTY_MIN;         		// 设置比较值,此值用来确定占空比，默认为 1ms时间，用于驱动电调
	TIM1_CHxHandler.OCPolarity			= TIM_OCPOLARITY_HIGH; 			// 输出比较极性为高

	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CHxHandler,TIM_CHANNEL_1);	//配置TIM3通道1
	HAL_TIM_PWM_Start(&TIM1_Handler, TIM_CHANNEL_1);													//开启PWM通道1
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CHxHandler,TIM_CHANNEL_2);	//配置TIM3通道2
  HAL_TIM_PWM_Start(&TIM1_Handler, TIM_CHANNEL_2);													//开启PWM通道2
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CHxHandler,TIM_CHANNEL_3);	//配置TIM3通道3
  HAL_TIM_PWM_Start(&TIM1_Handler, TIM_CHANNEL_3);													//开启PWM通道3
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CHxHandler,TIM_CHANNEL_4);	//配置TIM3通道4
  HAL_TIM_PWM_Start(&TIM1_Handler, TIM_CHANNEL_4);													//开启PWM通道4																										
}

/* 
 * 定时器底层驱动，时钟使能，引脚配置	
 * 此函数会被HAL_TIM_PWM_Init()调用	
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
 * 设置指定电机占空比										
 * motor: 选择电机，可填MOTOR1, MOTOR2, MOTOR3, MOTOR4	
 * 正脉宽：1ms-2ms，超出范围无效				
 * pwm 单位：ms，设置为1000即为1ms
 */

void SetMotorDutyApi(enum MOTOR_m motor, uint16_t pwm)
{
	if (pwm > PWM_DUTY_MAX)														// 最大正脉宽 2ms
		pwm = PWM_DUTY_MAX;
	else if (pwm <= PWM_DUTY_MIN)											// 最小正脉宽 1ms
		pwm = PWM_DUTY_MIN;
	
	if (motor == MOTOR1)
		TIM1->CCR2 = pwm;																// 设定 CH1 计数比较值
	else if (motor == MOTOR2)
		TIM1->CCR1 = pwm;																// 设定 CH2 计数比较值
	else if (motor == MOTOR3)
		TIM1->CCR4 = pwm;																// 设定 CH3 计数比较值
	else if (motor == MOTOR4) 
		TIM1->CCR3 = pwm;																// 设定 CH4 计数比较值	
}

/* 
 * 设置四个电机停转，情况紧急时赶快调用 (^_^)				
 */
void SetMotorsStopApi(void)
{
	SetMotorDutyApi(MOTOR1, PWM_DUTY_MIN);						// 调用API，设置电机1占空比
	SetMotorDutyApi(MOTOR2, PWM_DUTY_MIN);						// 调用API，设置电机2占空比
	SetMotorDutyApi(MOTOR3, PWM_DUTY_MIN);						// 调用API，设置电机3占空比
	SetMotorDutyApi(MOTOR4, PWM_DUTY_MIN);						// 调用API，设置电机4占空比		
}

/*
 * 四个电机依次慢速转动
 */
void SetFourMotorsSpinSlowly(void)
{
	SetMotorDutyApi(MOTOR1, PWM_DUTY_SAFE);						// 调用API，设置电机1占空比
	vTaskDelay(500);	
	SetMotorDutyApi(MOTOR2, PWM_DUTY_SAFE);						// 调用API，设置电机2占空比
	vTaskDelay(500);
	SetMotorDutyApi(MOTOR3, PWM_DUTY_SAFE);						// 调用API，设置电机3占空比
	vTaskDelay(500);
	SetMotorDutyApi(MOTOR4, PWM_DUTY_SAFE);						// 调用API，设置电机4占空比
	vTaskDelay(500);	
}

/*
 * 设置电机占空比，四个电机占空比相同
 * duty:ms
 */
void SetMotorsDutyApi(uint16_t duty)
{
	SetMotorDutyApi(MOTOR1, duty);		// 调用API，设置电机1占空比
	SetMotorDutyApi(MOTOR2, duty);		// 调用API，设置电机2占空比
	SetMotorDutyApi(MOTOR3, duty);		// 调用API，设置电机3占空比
	SetMotorDutyApi(MOTOR4, duty);		// 调用API，设置电机4占空比				
}


/*
 * 电机测试任务
 */
void MotorTestTask(void *arg)
{
	TIM1_PWM_Init();
	
	while (1)
	{
		SetFourMotorsSpinSlowly();		// 四个电机依次转动
		vTaskDelay(1000);
		SetMotorsDutyApi(0);					// 占空比为0，电机停转
		vTaskDelay(3000);
	}
}
