#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"   	  
 

#define KEY_B9  	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)	//读取按键0
#define KEY_C13  	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)//读取按键1 
 

#define KEY_B9_PRES		1		//KEY0  
#define KEY_C13_PRES	2		//KEY1 


void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8 mode);  	//按键扫描函数					    
#endif
