#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

 
extern int zhongduan;
void TIM1_Int_Init(u16 arr,u16 psc);
void TIM2_Cap_Init(void);
void TIM3_Cap_Init(void);
void delay_ctrl(void);
void TIM8_Int_Init(u16 arr,u16 psc);

#endif
