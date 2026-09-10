#include "io.h"

void IO_Init(void)
{
/* 1.打开外设时钟(为了低功耗，需要分别打开) */
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

//摄像头通讯
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;		// 引脚openmv4、5、6为左，11，12、15为右来的，分别为double、green、red
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 模式：输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//光电管	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_9;		//PC4左、PC9右
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 模式：输入模式
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//水泵
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		// 水泵上
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 模式：输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		// 水泵下
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 模式：输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	
//板间通讯
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;		// 主控板给作业板左行的信息
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;						// 模式：输入模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		// 主控板给作业板右行的信息
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;						// 模式：输入模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   //主控板给作业板的控制作业信号
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 模式：输入模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;         //作业板给主控板的减速信息
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_7);
	
//LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;         //LED1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;         //LED2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_1);
}

