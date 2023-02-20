#ifndef MOTOR_H
#define MOTOR_H
#include "sys.h"

/* 
 * 四个电机枚举	
 */
enum MOTOR_m
{
	MOTOR1 = 0,
	MOTOR2 = 1,
	MOTOR3 = 2,
	MOTOR4 = 3
};


void TIM1_PWM_Init(void);												

void SetMotorDutyApi(enum MOTOR_m motor, uint16_t pwm);
void SetMotorsStopApi(void);
void SetFourMotorsSpinSlowly(void);
void SetMotorsDutyApi(uint16_t duty);

void MotorTestTask(void *arg);

#endif
