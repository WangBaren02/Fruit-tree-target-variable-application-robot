#include "lamp.h"

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

/****
	* @brief  指示灯的初始化
	* @param  无
	* @note   无
	* @retval 无
	* @bug
    * @todo	
	*/
void LAMP_Init(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;								//定义一个GPIO_InitTypeDef类型的结构体
	RCC_AHB1PeriphClockCmd ( LAMP_GPIO_CLK, ENABLE);					//开启LED相关的GPIO外设时钟 
	
	GPIO_InitStructure.GPIO_Pin = LAMP_PIN;								//选择要控制的GPIO引脚		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  						//设置引脚模式为输出模式 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//设置引脚的输出类型为推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;					//设置引脚为空	  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 					//设置引脚速率为2MHz  
	
	GPIO_Init(LAMP_GPIO_PORT, &GPIO_InitStructure);						//调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO 	
	LAMP_OFF;
}
