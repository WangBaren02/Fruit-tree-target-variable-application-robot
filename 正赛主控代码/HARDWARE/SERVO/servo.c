#include "servo.h"
#include "sys.h"
#include "control.h"
/**************************************************************************
名称:TIM4舵机PWM初始化函数
参数:u32 arr 自动重装载值,u16 psc 预分频系数
作用:初始化
**************************************************************************/
void TIM4_Servo_PWM_Init(u32 arr, u16 psc)
{
	//初始化结构体变量
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef					TIM_OCInitStructure;
	
	//RCC时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	//IO端口复用
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	//IO初始化
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//定时器初始化
	TIM_TimeBaseStructure.TIM_Period 				= arr-1;
	TIM_TimeBaseStructure.TIM_Prescaler 		= psc-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	//OC模式初始化
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

/**************************************************************************
名称:TIM8舵机PWM初始化函数
参数:u32 arr 自动重装载值,u16 psc 预分频系数
作用:初始化
**************************************************************************/
void TIM8_Servo_PWM_Init(u32 arr, u16 psc)
{
	
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef					TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr-1;
	TIM_TimeBaseStructure.TIM_Prescaler = psc-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM8,ENABLE);	 //高级定时器(TIM1、TIM8)独有
	
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_Cmd(TIM8, ENABLE);		
}

/**************************************************************************
名称:舵机1角度与CCR转换函数
参数:float angle	目标角度
作用:计算出旋转到指定角度所需要设置的定时器CCR
**************************************************************************/
void SERVO1_Set_Angle(float angle)
{
	u16 angle_CCR;
	if(angle >= 220)
		angle = 220;
	else if(angle <= 5)
		angle = 5;
	angle_CCR = angle * 7.4 + 500.0f;		//180°舵机时angle * 11.1 (11.1 = 2000/180)
	SERVO1_PWM = angle_CCR;
}
/**************************************************************************
名称:舵机2角度与CCR转换函数
参数:float angle	目标角度
作用:计算出旋转到指定角度所需要设置的定时器CCR
**************************************************************************/
void SERVO2_Set_Angle(float angle)
{
	u16 angle_CCR;
	if(angle >= 220)
		angle = 220;
	else if(angle <= 5)
		angle = 5;
	angle_CCR = angle * 7.4 + 500.0f;
	SERVO2_PWM = angle_CCR;
}
/**************************************************************************
名称:舵机3角度与CCR转换函数
参数:float angle	目标角度
作用:计算出旋转到指定角度所需要设置的定时器CCR
**************************************************************************/
void SERVO3_Set_Angle(float angle)
{
	u16 angle_CCR;
	if(angle >= 220)
		angle = 220;
	else if(angle <= 5)
		angle = 5;
	angle_CCR = angle * 7.4 + 500.0f;
	SERVO3_PWM = angle_CCR;
}
/**************************************************************************
名称:舵机4角度与CCR转换函数
参数:float angle	目标角度
作用:计算出旋转到指定角度所需要设置的定时器CCR
**************************************************************************/
void SERVO4_Set_Angle(float angle)
{
	u16 angle_CCR;
	if(angle >= 220)
		angle = 220;
	else if(angle <= 5)
		angle = 5;
	angle_CCR = angle * 7.4 + 500.0f;
	SERVO4_PWM = angle_CCR;
}
/**************************************************************************
名称:舵机5角度与CCR转换函数
参数:float angle	目标角度
作用:计算出旋转到指定角度所需要设置的定时器CCR
**************************************************************************/
void SERVO5_Set_Angle(float angle)
{
	u16 angle_CCR;
	if(angle >= 220)
		angle = 220;
	else if(angle <= 5)
		angle = 5;
	angle_CCR = angle * 7.4 + 500.0f;
	SERVO5_PWM = angle_CCR;
}
/**************************************************************************
名称:舵机6角度与CCR转换函数
参数:float angle	目标角度
作用:计算出旋转到指定角度所需要设置的定时器CCR
**************************************************************************/
void SERVO6_Set_Angle(float angle)
{
	u16 angle_CCR;
	if(angle >= 220)
		angle = 220;
	else if(angle <= 5)
		angle = 5;
	angle_CCR = angle * 7.4 + 500;
	SERVO6_PWM = angle_CCR;
}
/**************************************************************************
名称:舵机7角度与CCR转换函数
参数:float angle	目标角度
作用:计算出旋转到指定角度所需要设置的定时器CCR
**************************************************************************/
void SERVO7_Set_Angle(float angle)
{
	u16 angle_CCR;
	if(angle >=220)
		angle = 220;
	else if(angle <= 5)
		angle = 5;
	angle_CCR = angle * 7.4 + 500.0f;
	SERVO7_PWM = angle_CCR;
}
/**************************************************************************
名称:舵机8角度与CCR转换函数
参数:float angle	目标角度
作用:计算出旋转到指定角度所需要设置的定时器CCR
**************************************************************************/
void SERVO8_Set_Angle(float angle)
{
	u16 angle_CCR;
	if(angle >=220)
		angle = 220;
	else if(angle <= 5)
		angle = 5;
	angle_CCR = angle * 7.4 + 500.0f;
	SERVO8_PWM = angle_CCR;
}

void Servo_Set_Angle(void)			//设置舵机的角度
{
	SERVO1_Set_Angle(Servo1_Base + Servo1_Increase);
	SERVO2_Set_Angle(Servo2_Base + Servo2_Increase);
	SERVO3_Set_Angle(Servo3_Base + Servo3_Increase);
	SERVO4_Set_Angle(Servo4_Base + Servo4_Increase);
}
