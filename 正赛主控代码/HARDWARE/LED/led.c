#include "led.h"
/****
	*****************************************************************************
	* @file    led.c
	* @author  
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
	
/****
	* @brief  LED初始化 
	* @param  无
	* @note   初始化的时候把灯都打开了
	* @retval 无
	* @bug
  * @todo	
	*/
void LED_Init(void)
{		
	
	GPIO_InitTypeDef GPIO_InitStructure;																					//定义一个GPIO_InitTypeDef类型的结构体
	RCC_AHB1PeriphClockCmd ( LED1_GPIO_CLK|LED2_GPIO_CLK|LED3_GPIO_CLK, ENABLE); 	//开启LED相关的GPIO外设时钟
	
	GPIO_InitStructure.GPIO_Pin = LED1_PIN|LED2_PIN|LED3_PIN;											//选择要控制的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   															//设置引脚模式为输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;																//设置引脚的输出类型为推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;																	//设置引脚为空状态
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 														//设置引脚速率为100MHz  

	GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);																//调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO
//	GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
//	GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

}


