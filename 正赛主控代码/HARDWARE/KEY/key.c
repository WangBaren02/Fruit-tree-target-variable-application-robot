/****
	*****************************************************************************
	* @file    bsp_key.c
	* @author  huang 
	* @version V1.0
	* @date    2020-10-13
	* @brief   串口DMA的初始化及上位机的应用
	* @license Copyright 2020 YRCK. All rights reserved. 

	*****************************************************************************
	* @note		按键初始化，按键的扫描
				USART1_RX---PC10
				USART1_TX---PC11	
	* @bug
	* @todo	
	* @details 
	*
	*****************************************************************************
	*/
#include "key.h" 
#include "delay.h"
#include "sys.h"

/****
	*****************************************************************************
										        CONSTANTS & MACROS
	*****************************************************************************
	* @brief  按键引脚宏定义
	*/
#define KEY1_PIN                  GPIO_Pin_0                
#define KEY1_GPIO_PORT            GPIOE                     
#define KEY1_GPIO_CLK             RCC_AHB1Periph_GPIOE

#define KEY2_PIN                  GPIO_Pin_1                
#define KEY2_GPIO_PORT            GPIOE                     
#define KEY2_GPIO_CLK             RCC_AHB1Periph_GPIOE

#define KEY3_PIN                  GPIO_Pin_2
#define KEY3_GPIO_PORT            GPIOE
#define KEY3_GPIO_CLK             RCC_AHB1Periph_GPIOE

#define KEY4_PIN                  GPIO_Pin_3
#define KEY4_GPIO_PORT            GPIOE
#define KEY4_GPIO_CLK             RCC_AHB1Periph_GPIOE

/*******************************************************/


/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
	
void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(KEY1_GPIO_CLK|KEY2_GPIO_CLK|KEY3_GPIO_CLK|KEY4_GPIO_CLK,ENABLE);//开启按键GPIO口的时钟
	
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN;							//选择按键的引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;					//设置引脚为输入模式	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					//设置引脚不上拉也不下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 				//设置引脚速率为2MHz 
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
	
	GPIO_InitStructure.GPIO_Pin = KEY2_PIN;							//选择按键的引脚
	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
	
	GPIO_InitStructure.GPIO_Pin = KEY3_PIN;							//选择按键的引脚
	GPIO_Init(KEY3_GPIO_PORT, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
	
	GPIO_InitStructure.GPIO_Pin = KEY4_PIN;							//选择按键的引脚
	GPIO_Init(KEY4_GPIO_PORT, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
}

/****
	* @brief	初始化按键扫描函数
	* @param	mode：0,不支持连续按;1,支持连续按;
	* @retval	返回相应按键的编号
	* @bug
	* @todo	
	*	@todo	注意此函数有响应优先级,KEY1>KEY2>KEY3
	*/
u8 Key_Scan(void)
{  
	 static u8 key_up=1;
	 if(key_up&&((KEY1==0)||(KEY2==0)||(KEY3==0)||(KEY4==0)))
	 {
		 delay_ms(10);
		 key_up=0;
		 if(KEY1==0)      return KEY1_PRES;
		 else if(KEY2==0) return KEY2_PRES;
		 else if(KEY3==0) return KEY3_PRES; 
		 else if(KEY4==0) return KEY4_PRES; 
	 }
    else if(KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1)key_up=1;
	return 0;	
}



/*********************************************END OF FILE**********************/
