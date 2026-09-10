#ifndef __PWM_H
#define __PWM_H
#include "sys.h"

#define SERVO1_PWM TIM5->CCR3
#define SERVO2_PWM TIM5->CCR4
#define SERVO3_PWM TIM4->CCR1

void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM5_PWM_Init(u16 arr,u16 psc);
void servo1(float angle);
void servo2(float angle);
void servo3(float angle);

#endif
