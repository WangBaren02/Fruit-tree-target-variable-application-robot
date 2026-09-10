#ifndef __LED_H
#define	__LED_H
/****
	*****************************************************************************
	* @file    bsp_led.c
	* @author  杨少文
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
	* @brief  LED引脚宏定义
	*/
#define LED1_PIN                  GPIO_Pin_13                 
#define LED1_GPIO_PORT            GPIOC
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOC


#define LED2_PIN                  GPIO_Pin_4
#define LED2_GPIO_PORT            GPIOC
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOC


#define LED3_PIN                  GPIO_Pin_5
#define LED3_GPIO_PORT            GPIOC
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOC

/****
	*****************************************************************************
										        CONSTANTS & MACROS
	*****************************************************************************
	* @brief  直接操作寄存器的方法控制IO 
	*/

#define	digitalHi(p,i)			 			{p->BSRRL=i;}									//设置为高电平
#define digitalLo(p,i)			 			{p->BSRRH=i;}									//输出低电平
#define digitalToggle(p,i)	 	 			{p->ODR ^=i;}									//输出反转状态

/****
	*****************************************************************************
										        CONSTANTS & MACROS
	*****************************************************************************
	* @brief   定义控制IO的宏
	*/

#define LED1_TOGGLE			digitalToggle(LED1_GPIO_PORT,LED1_PIN)		//闪烁
#define LED1_ON			 		digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF				digitalLo(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE			digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON			  	digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF				digitalLo(LED2_GPIO_PORT,LED2_PIN)

#define LED3_TOGGLE			digitalToggle(LED3_GPIO_PORT,LED3_PIN)
#define LED3_ON			  	digitalHi(LED3_GPIO_PORT,LED3_PIN)
#define LED3_OFF				digitalLo(LED3_GPIO_PORT,LED3_PIN) 


void LED_Init(void);
#endif /* __LED_H */
