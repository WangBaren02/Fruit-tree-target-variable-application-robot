#include "io.h"
//板间通讯引脚对应
//主控板	作业板	
//	E7	->	B4
//	E8	->	B5
//	E14	->	B6
//	E15	->	B7

void IO_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE,ENABLE);//开启按键GPIO口的时钟

	
//板间通讯
	
	//左行信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;						
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//设置引脚为输出模式	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//设置引脚输出模式为推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 	//设置引脚速率为100MHz 
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	//右行信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;						
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//设置引脚为输出模式	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//设置引脚输出模式为推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 	//设置引脚速率为100MHz 	
	GPIO_ResetBits(GPIOE,GPIO_Pin_8);
	//作业模式控制信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;						
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//设置引脚为输出模式	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//设置引脚输出模式为推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 	//设置引脚速率为100MHz 
	GPIO_Init(GPIOE, &GPIO_InitStructure);   				
	GPIO_ResetBits(GPIOE,GPIO_Pin_14);
	//减速信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;				//设置引脚为输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;		//浮空
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 	//设置引脚速率为100MHz 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
//光电管
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;						//车身左侧光电管
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;				//设置引脚为输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;		//浮空
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 	//设置引脚速率为100MHz 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;					//车身右侧光电管
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;				//设置引脚为输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;		//浮空
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 	//设置引脚速率为100MHz 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}
