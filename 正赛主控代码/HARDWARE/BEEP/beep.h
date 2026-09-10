#ifndef __BEEP_H
#define	__BEEP_H

/****
	*****************************************************************************
	* @file    beep.h
	* @author  YSW
	* @version V1.0
	* @date    2021-7-6
	* @brief   LED函数
	* @license Copyright 2021. All rights reserved. 

	*****************************************************************************
	* @note  	 	无
	* @bug
	* @todo	
	* @details 
	*
	*****************************************************************************
	*/
#include "stm32f4xx.h"

/****
	*****************************************************************************
										        CONSTANTS & MACROS
	*****************************************************************************
	* @brief  
	*/
#define BEEP_PIN                  GPIO_Pin_7                
#define BEEP_GPIO_PORT            GPIOD                     
#define BEEP_GPIO_CLK             RCC_AHB1Periph_GPIOD

/****
	*****************************************************************************
										        CONSTANTS & MACROS
	*****************************************************************************
	* @brief  直接操作寄存器的方法
	*/
#define	digitalHi(p,i)				{p->BSRRL=i;}		//设置为高电平
#define digitalLo(p,i)			 	{p->BSRRH=i;}		//输出低电平
#define digitalToggle(p,i)	 		{p->ODR ^=i;}		//输出反转状态

/****
	*****************************************************************************
										        CONSTANTS & MACROS
	*****************************************************************************
	* @brief  蜂鸣器状态
	*/
#define BEEP_TOGGLE			digitalToggle(BEEP_GPIO_PORT,BEEP_PIN)
#define BEEP_ON			  	digitalHi(BEEP_GPIO_PORT,BEEP_PIN)
#define BEEP_OFF			digitalLo(BEEP_GPIO_PORT,BEEP_PIN)

void BEEP_Init(void);
void BIBI(void);

#endif /* __LED_H */
