#include "sow.h"
#include "delay.h"
#include "stm32f10x.h"

///////////////////////////作业

uint8_t flag_Left = 0,flag_Right = 0;

uint8_t flag_Green = 0,flag_Red = 0;

uint8_t RightLightLast = 1;
uint8_t LeftLightLast = 1;

void Select_LR(void)
{
	uint8_t flag_Left_Current=0,flag_Right_Current=0;
	flag_Left_Current = Left_Flag;	//板间通讯：左行
	flag_Right_Current = Right_Flag;	//板间通讯：右行
	delay_ms(5);
	flag_Left = Left_Flag;
	flag_Right = Right_Flag;
	if((flag_Left == flag_Left_Current) && (flag_Right == flag_Right_Current))
		return;
	else Select_LR();
	
//	flag_Left = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4);	//板间通讯：左行
//	flag_Right = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5);	//板间通讯：右行
}

void Select_Color(uint8_t Left,uint8_t Right)   //颜色识别
{
	if((!Left) && (Right))	//正在右行，读取右边摄像头的颜色信息
	{
		if(Right_green && !Right_red)	
		{
			flag_Green=1;
			return;
		}
		else if(!Right_green && Right_red)	
		{
			flag_Red=1;
			return;
		}
	}
	else if((Left)&&(!Right))	//正在左行，读取左边摄像头的颜色信息
	{
		if(Left_green && !Left_red)	
		{
			flag_Green=1;
			return;
		}	
		else if(!Left_green && Left_red)
		{
			flag_Red=1;
			return;
		}
	}
}

void Water(uint8_t Red,uint8_t Green)    //4是L，9是R
{
	if(Green)
	{
		delay_ms(BiuBiu_interval);
		Pump_Down_ON;								//开启下水泵
		delay_ms(BiuBiu_time_Green);	//下水泵喷水时长
		Pump_Down_OFF;							//关闭下水泵
	}
	else if(Red)
	{
		Pump_Up_ON;									//开启上水泵
		delay_ms(BiuBiu_time_UP);		//上水泵喷水时长
		Pump_Up_OFF;								//关闭上水泵
		
		Pump_Down_ON;								//开启下水泵
		delay_ms(BiuBiu_time_DOWN);	//下水泵喷水时长
		Pump_Down_OFF;							//关闭下水泵
	}
	return ;
}

void Light_Detect(void)		//垄中光电管减速及喷水逻辑
{
	uint8_t Current_RightLight,Current_LeftLight;
	Current_RightLight = Light_Right;
	Current_LeftLight = Light_Left;
	
	if(flag_Right && Current_LeftLight && !Current_RightLight && RightLightLast) 		//右行，右侧光电管1->0检测到果树
	{
		SlowDown_ON;	//作业板给主控板的减速信息
		Select_Color(flag_Left,flag_Right);
		delay_ms(Deteted2Biu_Right);		//右行时，从识别到颜色到喷水的延迟时间
		Water(flag_Red,flag_Green);//Water(1,0);	//
		SlowDown_OFF;	//停止减速
		flag_Red = flag_Green = 0;
	}
	else if(flag_Left && !Current_LeftLight && Current_RightLight && LeftLightLast)	//左行，左侧光电管1->0检测到果树
	{
		SlowDown_ON;	//作业板给主控板的减速信息
		Select_Color(flag_Left,flag_Right);
		delay_ms(Deteted2Biu_Left);		//左行时，从识别到颜色到喷水的延迟时间
		Water(flag_Red,flag_Green);//Water(1,0);	//
		SlowDown_OFF;	//停止减速
		flag_Red = flag_Green = 0;
	}
	
	RightLightLast = Current_RightLight;
	LeftLightLast = Current_LeftLight;
	return ;
}

void Clear_RESET(void)	//作业板处于待机模式时，所有信号复位
{
	flag_Green = 0,flag_Red = 0;
	RightLightLast = 1;
	LeftLightLast = 1;
	SlowDown_OFF;		//减速停止
	Pump_Up_OFF;		//关闭上水泵
	Pump_Down_OFF;	//关闭下水泵
}
