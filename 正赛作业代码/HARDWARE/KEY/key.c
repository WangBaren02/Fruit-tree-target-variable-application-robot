#include "key.h"
#include "delay.h" 

//按键初始化函数 
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA,PORTC时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;//PB9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置模式：输入模式
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOB9
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;//PC13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置模式：输入模式
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC13
} 
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//返回值：
//0，没有任何按键按下
//KEY0_PRES，KEY0按下
//KEY1_PRES，KEY1按下
//注意此函数有响应优先级,KEY0>KEY1>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode) key_up=1;  //支持连按		  
	if(key_up&&(KEY_B9==0||KEY_C13==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY_B9==0) return KEY_B9_PRES;
		else if(KEY_C13==0) return KEY_C13_PRES;
	}else if(KEY_B9==1&&KEY_C13==1) key_up=1; 	     
	return 0;// 无按键按下
}
