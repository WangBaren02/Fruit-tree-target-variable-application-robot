#include "delay.h"
#include "sys.h"
#include "bmp.h"
#include "stdio.h"
#include "io.h"
#include "pwm.h"
#include "timer.h"
#include "key.h"
#include "sow.h"

int main(void)
{
		u8 key;
		IO_Init();
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
		TIM5_PWM_Init(20000,72);
		delay_init();	 //延时函数初始化:	有问题，延迟时间大概是八分之一，最长24000（3s） 

			while(1)
			{
				key = KEY_Scan(0);

				//正式比赛
				if(WORK_FLAG)
				{
					Select_LR();
					Light_Detect();
					servo1(0);
				}
				else
				{
					Clear_RESET();
					servo1(270);
				}
				
				
				 //摄像头颜色测试				
         if ((Right_red) && (!Right_green))
				 {
						Led1_ON;  
						Led2_ON;  
				 }
				 else if ((!Right_red) && (Right_green))
				 {
						Led1_ON;  
				 }
				 else
				 {
						Led1_OFF;  
						Led2_OFF; 
				 }
				
//				//舵机测试				
//				if(key == KEY_B9_PRES)
//				{
//					servo1(270);
//					Led1_ON;
//				}
//				
//				if(key == KEY_C13_PRES)
//				{
//					servo1(0);
//					Led1_OFF;
//				}
				
				//喷水测试				
				if(key == KEY_B9_PRES)
				{
//					Pump_Down_ON;								//开启下水泵
//					Pump_Up_ON;									//开启上水泵
//								
//					delay_ms(BiuBiu_time_UP);		//下水泵喷水时长
//					Pump_Down_OFF;							//关闭下水泵
//							
//					delay_ms(BiuBiu_interval);	//上水泵多于下水泵的喷水时长
//					Pump_Up_OFF;								//关闭上水泵
					
					
					Pump_Down_ON;									//开启下水泵
					delay_ms(BiuBiu_time_Green);	//下水泵喷水时长
					Pump_Down_OFF;								//关闭下水泵
				}
				
				if(key == KEY_C13_PRES)
				{
//					Pump_Up_ON;		//上水泵
//					Pump_Down_ON;;	//下水泵
//					delay_ms(200);	//上：8~9
//					Pump_Up_OFF;	//关闭上水泵
//					Pump_Down_OFF;							//关闭下水泵
////					Pump_Up_ON;			//上水泵
////					Pump_Down_ON;;	//下水泵
//					
//					
//					
					Pump_Up_ON;									//开启上水泵
					delay_ms(BiuBiu_time_UP);		//下水泵喷水时长
					Pump_Up_OFF;								//关闭下水泵
				}
				
//				//光电管测试
//				if(Light_Left)	//左侧光电管
//					Led1_ON;
//				else
//					Led1_OFF;
//				
//				if(Light_Right)	//右侧光电管
//					Led2_ON;
//				else
//					Led2_OFF;

//				//左右行检测				
//				if(flag_Right) Led1_ON;
//				else Led1_OFF;
//				
//				if(flag_Left) Led2_ON;
//				else Led2_OFF;
			
			}
}
