#ifndef __SOW_H
#define __SOW_H
#include "sys.h"

#define Right_green GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)	//右侧摄像头检测到绿色
#define Right_red 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)	//右侧摄像头检测到红色

#define Left_green 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)		//左侧摄像头检测到绿色
#define Left_red 		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)		//左侧摄像头检测到红色

#define Light_Left 	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)		//左侧光电管
#define Light_Right GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)		//右侧光电管

#define	WORK_FLAG		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)		//主控板给作业板的控制作业信号
#define	Left_Flag 	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)		//板间通讯：左行	
#define	Right_Flag	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)		//板间通讯：右行

#define	Deteted2Biu_Left	165		//左行时，从光电管检测到树到喷水的延迟时间,50
#define	Deteted2Biu_Right	165		//右行时，从光电管检测到树到喷水的延迟时间,50

#define	BiuBiu_time_UP		6.5			//上水泵的喷洒时间：8
#define	BiuBiu_time_DOWN	10		//下水泵的喷洒时间：12
#define BiuBiu_interval		1			//上水泵喷射开始到结束的模拟时间	
#define	BiuBiu_time_Green 10		//绿树（树干）喷水时间：12

#define	Pump_Up_ON 		GPIO_SetBits(GPIOC,GPIO_Pin_5)		//打开上水泵
#define	Pump_Up_OFF 	GPIO_ResetBits(GPIOC,GPIO_Pin_5)	//关闭上水泵
#define	Pump_Down_ON 	GPIO_SetBits(GPIOA,GPIO_Pin_8)		//打开下水泵
#define	Pump_Down_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_8)	//关闭下水泵

#define	SlowDown_ON 	GPIO_SetBits(GPIOB,GPIO_Pin_7)		//板间通讯：开启减速模式
#define SlowDown_OFF	GPIO_ResetBits(GPIOB,GPIO_Pin_7)	//板间通讯：关闭减速模式

#define Led1_ON				GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define Led1_OFF			GPIO_SetBits(GPIOB,GPIO_Pin_0)
#define Led2_ON				GPIO_ResetBits(GPIOB,GPIO_Pin_1)
#define Led2_OFF			GPIO_SetBits(GPIOB,GPIO_Pin_1)

//extern uint8_t flag_Left,flag_Right;

void Select_LR(void);

void Select_Color(uint8_t zuo,uint8_t you); 

void Water(uint8_t Red,uint8_t Green);

void Light_Detect(void);

void Clear_RESET(void);
#endif
