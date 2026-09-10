#include "pwm.h"
#include "io.h"
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数

//servo1  PA2
//servo2  PA3


void TIM5_PWM_Init(u16 arr,u16 psc)	//转盘
{  
	GPIO_InitTypeDef GPIO_InitStrue;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStrue.GPIO_Pin=GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStrue.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStrue);
	
	TIM_TimeBaseStructure.TIM_Period = arr-1; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OC3Init(TIM5,&TIM_OCInitStructure);    //通道3
	TIM_OC4Init(TIM5,&TIM_OCInitStructure);   //通道4
	
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_Cmd(TIM5,ENABLE);
}

void servo1(float angle)    //pwm是设定的角度，这里pwm有效值0~270，下同,两枚
{
	u16 angle_CCR;
	if(angle >= 240)
		angle = 240;
	else if(angle <= 3)
		angle = 3;
	angle_CCR = angle * 7.4 + 500.0f;
	TIM_SetCompare3(TIM5,angle_CCR);
}

void servo2(float angle)		//
{
	u16 angle_CCR;
	if(angle >= 240)
		angle = 240;
	else if(angle <= 3)
		angle = 3;
	angle_CCR = angle * 7.4 + 500.0f;
	TIM_SetCompare4(TIM5,angle_CCR);
}


