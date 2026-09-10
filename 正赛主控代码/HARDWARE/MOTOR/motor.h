#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f4xx.h"
/*
R1接驱动板MOTORA       		//有编码器
L1接驱动板MOTORC
R2接驱动板MOTORD
L2接驱动板MOTORB					//有编码器
*/
#define R1PWM TIM1->CCR1
#define L2PWM TIM1->CCR2
#define L1PWM TIM1->CCR3
#define R2PWM TIM1->CCR4

#define R1IN1 PAout(15)
#define R1IN2 PEout(9)
#define L1IN1 PDout(11)
#define L1IN2 PBout(10)
#define R2IN1 PCout(12)
#define R2IN2 PDout(0)
#define L2IN1 PBout(4)
#define L2IN2 PBout(5)

void Motor_Init(void);
void TIM1_Motor_PWM_Init(u16 arr, u16 psc);
void Motor_Set_Speed(void);		//赋值顺序为L1--R1--L2--R2;
int myabs(int number);
#endif

