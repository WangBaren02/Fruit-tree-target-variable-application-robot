#ifndef __SERVO_H
#define __SERVO_H
#include "stm32f4xx.h"
/***********************
L1接舵机1						135°时轮子朝前
R1接舵机2						45°时轮子朝前
L2接舵机3						55°时轮子朝前
R2接舵机4						130°时轮子朝前
5接上方的OpenMV    	180°时看右边
6接转盘
8接最下方的小舵机		30°左右是合上
***********************/

#define SERVO1_PWM TIM4->CCR1
#define SERVO2_PWM TIM4->CCR2
#define SERVO3_PWM TIM4->CCR3
#define SERVO4_PWM TIM4->CCR4
#define SERVO5_PWM TIM8->CCR1
#define SERVO6_PWM TIM8->CCR2
#define SERVO7_PWM TIM8->CCR3
#define SERVO8_PWM TIM8->CCR4

void TIM4_Servo_PWM_Init(u32 arr, u16 psc);
void TIM8_Servo_PWM_Init(u32 arr, u16 psc);

void SERVO1_Set_Angle(float angle);
void SERVO2_Set_Angle(float angle);
void SERVO3_Set_Angle(float angle);
void SERVO4_Set_Angle(float angle);
void SERVO5_Set_Angle(float angle);
void SERVO6_Set_Angle(float angle);
void SERVO7_Set_Angle(float angle);
void SERVO8_Set_Angle(float angle);

void Servo_Set_Angle(void);			//设置舵机的角度

#endif

