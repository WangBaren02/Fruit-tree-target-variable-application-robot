#ifndef __LAMP_H
#define	__LAMP_H

/****
	*****************************************************************************
	* @file    lamp.h
	* @author  YSW
	* @version V1.0
	* @date    2021-7-6
	* @brief   指示灯函数
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
#define LAMP_PIN                  GPIO_Pin_4             
#define LAMP_GPIO_PORT            GPIOD                     
#define LAMP_GPIO_CLK             RCC_AHB1Periph_GPIOD

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
#define LAMP_TOGGLE			digitalToggle(LAMP_GPIO_PORT,LAMP_PIN)
#define LAMP_ON			  	digitalHi(LAMP_GPIO_PORT,LAMP_PIN)
#define LAMP_OFF			digitalLo(LAMP_GPIO_PORT,LAMP_PIN)

void LAMP_Init(void);


#endif /* __LAMP_H */

