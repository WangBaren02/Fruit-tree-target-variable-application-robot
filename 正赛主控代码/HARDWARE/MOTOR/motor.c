#include "motor.h"
#include "sys.h"
#include "control.h"
#include "oled.h"

/*****************************************
1表示车头，2表示车尾，L(left)左,R(right)右。
               L1-----------R1
               |						|
               |						|
               |						|
               L2-----------R2
*******************************************/
/****
	* @brief  电机普通IO的初始化
	* @param  无
	* @note   AIN2--PE9
						BIN2--PA15
						CIN2--PB10
						DIN2--PD12
	* @retval 无
	* @bug
    * @todo
	*/
void Motor_Init(void)
{
    //初始化结构体变量
    GPIO_InitTypeDef GPIO_InitStructure;

    //RCC时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    //IO初始化
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;					//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//推挽
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		//PE9
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;		//PA15
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;		//PB10
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;		//PD11
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;			//PB4
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;			//PB5
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//PC12
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			//PD0
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    TIM1_Motor_PWM_Init(7200, 2);
	
	GPIO_SetBits(GPIOE,GPIO_Pin_9);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
	GPIO_ResetBits(GPIOD,GPIO_Pin_11);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
	
	GPIO_SetBits(GPIOC,GPIO_Pin_12);
	GPIO_ResetBits(GPIOD,GPIO_Pin_0);
	
	
}

void TIM1_Motor_PWM_Init(u16 arr, u16 psc)
{
    //初始化结构体变量
    GPIO_InitTypeDef					GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
    TIM_OCInitTypeDef					TIM_OCInitStructure;

    //RCC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    //IO复用
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

    //IO初始化
    GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //定时器初始化
    TIM_TimeBaseStructure.TIM_Period 				= arr - 1;
    TIM_TimeBaseStructure.TIM_Prescaler 		= psc - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_High;

    /***********解决普通输出时的互补BUG************/
    TIM_OCInitStructure.TIM_OCNIdleState	= TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OutputNState	= TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity 	= TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    /************使能预装载寄存器****************/
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);                                                    //允许在定时器工作时向ARR的缓冲器中写入新值，以便在更新事件发生时载入覆盖以前的值

    TIM_CtrlPWMOutputs(TIM1, ENABLE);                                                      /************************/
    TIM_Cmd(TIM1, ENABLE);
		TIM_SetCompare1(TIM1,4000);
		TIM_SetCompare2(TIM1,4000);
		TIM_SetCompare3(TIM1,4000);
		TIM_SetCompare4(TIM1,4000);
}

void Motor_Set_Speed(void)
{
    float speedL1 = SpeedL1_Base + SpeedL1_Increase + SpeedL1_Increase_9250;
    float speedR1 = SpeedR1_Base + SpeedR1_Increase + SpeedR1_Increase_9250;
    float speedL2 = SpeedL2_Base + SpeedL2_Increase + SpeedL2_Increase_9250;
    float speedR2 = SpeedR2_Base + SpeedR2_Increase + SpeedR2_Increase_9250;

    //接驱动板电机A-----------------------------------------
    if(speedR1 >= 0)
    {
        R1PWM = speedR1;
        R1IN1 = 1;
        R1IN2 = 0;
    }

    if(speedR1 < 0)
    {
        R1PWM = myabs(speedR1);
        R1IN1 = 0;
        R1IN2 = 1;
    }

//接驱动板电机B-----------------------------------------
    if(speedL2 >= 0)
    {
        L2PWM = speedL2;
        L2IN1 = 1;
        L2IN2 = 0;
    }

    if(speedL2 < 0)
    {
        L2PWM = myabs(speedL2);
        L2IN1 = 0;
        L2IN2 = 1;
    }

//接驱动板电机C-----------------------------------------
    if(speedL1 >= 0)
    {
        L1PWM = speedL1;
        L1IN1 = 0;
        L1IN2 = 1;
    }

    if(speedL1 < 0)
    {
        L1PWM = myabs(speedL1);
        L1IN1 = 1;
        L1IN2 = 0;
    }

//接驱动板电机D-----------------------------------------
    if(speedR2 >= 0)
    {
        R2PWM = speedR2;
        R2IN1 = 0;
        R2IN2 = 1;
    }

    if(speedR2 < 0)
    {
        R2PWM = myabs(speedR2);
        R2IN1 = 1;
        R2IN2 = 0;
    }
}

int myabs(int number)
{
    //变量
    int temp = 0;

    //操作
    if( number >= 0 )
        temp = number;

    if( number < 0)
        temp = -number;

    //返回值
    return temp;
}


