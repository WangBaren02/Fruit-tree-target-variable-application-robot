#include "stdio.h"
#include "string.h"
#include "sys.h"
#include "delay.h"
#include "stm32f4xx.h"
#include "control.h"
//#include "lamp.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "oled.h"
#include "usart1_dma.h"
#include "usart2_dma.h"
#include "usart3_dma.h"
#include "uart4_dma.h"
#include "timer7.h"
#include "servo.h"
#include "ultrasonic.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "io.h"

float start_yaw=0;

int main(void)
{
	//系统初始化
	u8 key;
	u8 Add_Encoder_Flag = 0;
	float Add_Encoder_Oled = 0;
	float yaw_t;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	//BEEP_Init();
	Key_Init();
	IO_init();
	LED_Init();
	TIM4_Servo_PWM_Init(20000,84);
	Ultrasonic_Init();
	OLED_Init();
	OLED_Display_On();
	
	OLED_Clear();
	Motor_Init();
	TIM7_Init(1000,84);
	TIM3_EncoderB_Init(0xFFFF,1);
	TIM5_EncoderA_Init(0xFFFF,1);
	PID_Init();
	Motor_Stop();
	Motor_Set_Speed();
	USART1_Init(115200);	//单片机与陀螺仪上位机通讯串口
	USART2_Init(9600);		//单片机与陀螺仪通讯串口

	Servo1_Increase = 0;
	Servo2_Increase = 0;
	Servo3_Increase = 0;
	Servo4_Increase = 0;
	SpeedL1_Increase = 0;
	SpeedR1_Increase = 0;
	SpeedL2_Increase = 0;
	SpeedR2_Increase = 0;
	SpeedL1_Increase_9250 = 0;
	SpeedR1_Increase_9250 = 0;
	SpeedL2_Increase_9250 = 0;
	SpeedR2_Increase_9250 = 0;
//	LED1_ON;
	
	Servo_Forw();
//	Servo_Turn();
	Servo_Set_Angle();
	while(1)
	{
		key = Key_Scan();
		if(key == 1)	//尽量不要改里面的东西，看懂了再改
		{
			start_YAW=yaw;					//记录下当前指南针原始中心角度
			if(start_YAW>-180 && start_YAW<-135) mode=2;
			else if(start_YAW<180 && start_YAW>135) mode=3;

			delay_ms(100);			//等待yaw稳定
			
			start_yaw=yaw;
			PID_9250_Forward_motor.desired = start_yaw;
			PID_9250_Forward.desired = start_yaw;
			PID_9250_Right.desired = start_yaw;
			PID_9250_Left.desired = start_yaw;
			PID_9250_Right_slow.desired = start_yaw;
			PID_9250_Left_slow.desired = start_yaw;
			PID_9250_Back.desired = start_yaw;
			PID_9250_Back_last.desired = start_yaw;
			PID_9250_Back_last_motor.desired = start_yaw;
			
			GPIO_ResetBits(GPIOE,GPIO_Pin_14);	//板间通讯，给作业板关闭作业模式信号
			
			flag_mission_start = 1;
			flag_delay_1500ms = 1;	//step0时开始2s后以75%速度继续行驶
		}
		
		if(key == 2)
		{
//			switch (Add_Encoder_Flag)
//			{
//				case 1: Add_Encoder_2 	+= 1.8; Add_Encoder_Oled += 1.8; break;
//				case 2: Add_Encoder_4 	+= 1.6; Add_Encoder_Oled += 1.6; break;
//				case 3: Add_Encoder_6 	+= 1.8; Add_Encoder_Oled += 1.8; break;
//				case 4: Add_Encoder_8 	+= 1.6; Add_Encoder_Oled += 1.6; break;
//				case 5: Add_Encoder_10 	+= 1.8; Add_Encoder_Oled += 1.8; break;
//				case 6: Add_Encoder_Last+= 2.0; Add_Encoder_Oled += 2.0; break;
//				default:;
//			}
////调编码器用		
//			switch (Add_Encoder_Flag)
//			{
//				case 1: Add_Encoder_2 	-= 0.3; Add_Encoder_Oled -= 0.3; break;
//				case 2: Add_Encoder_4 	-= 0.2; Add_Encoder_Oled -= 0.2; break;
//				case 3: Add_Encoder_6 	-= 0.3; Add_Encoder_Oled -= 0.3; break;
//				case 4: Add_Encoder_8 	-= 0.2; Add_Encoder_Oled -= 0.2; break;
//				case 5: Add_Encoder_10 	-= 0.3; Add_Encoder_Oled -= 0.3; break;
//				case 6: Add_Encoder_Last-= 1.0; Add_Encoder_Oled -= 1.0; break;
//				default:;
//			}
			
			switch (Add_Encoder_Flag)
			{
				case 1: Add_Encoder_2 	+= 0.3; Add_Encoder_Oled += 0.3; break;
				case 2: Add_Encoder_4 	+= 0.2; Add_Encoder_Oled += 0.2; break;
				case 3: Add_Encoder_6 	+= 0.3; Add_Encoder_Oled += 0.3; break;
				case 4: Add_Encoder_8 	+= 0.2; Add_Encoder_Oled += 0.2; break;
				case 5: Add_Encoder_10 	+= 0.3; Add_Encoder_Oled += 0.3; break;
				case 6: Add_Encoder_Last+= 1.0; Add_Encoder_Oled += 1.0; break;
				default:;
			}
			
//			PID_F1_Right_Servo.kd +=0.1;
//			PID_F2_Right_Servo.kd +=0.1;
//			PID_F1_Right_Servo.kd += 0.1;
//			PID_F2_Right_Servo.kd += 0.1;
//			flag_step += 2;
//			PID_9250_Right.kd+=100;
//			PID_F1_Left_Servo.kp+=0.1;
//			PID_F1_Left_Servo.kp+=0.1;
		}
		
		if(key == 3)
		{
			if(Add_Encoder_Flag < 6)
			{
				Add_Encoder_Flag ++;
			}
			else if(Add_Encoder_Flag == 6)
			{
				Add_Encoder_Flag = 0;
			}
			Add_Encoder_Oled = 0; 
//			PID_9250_Right.kd+=50;
//			PID_F1_Right_Servo.kd +=0.05;
//			PID_F2_Right_Servo.kd +=0.05;
//			PID_F1_Right_Servo.kd += 0.05;
//			PID_F2_Right_Servo.kd += 0.05;
//			start_YAW=yaw;					//记录下当前指南针原始中心角度
//			if(start_YAW>-180 && start_YAW<-135)mode=2;
//			else if(start_YAW<180 && start_YAW>135)mode=3;
//			PID_9250_Right.kd+=2;
		}
		
		if(key == 4)
		{
			switch (Add_Encoder_Flag)
			{
				case 1: Add_Encoder_2 	+= 0.8; Add_Encoder_Oled += 0.8; break;
				case 2: Add_Encoder_4 	+= 0.55; Add_Encoder_Oled += 0.55; break;
				case 3: Add_Encoder_6 	+= 0.8; Add_Encoder_Oled += 0.8; break;
				case 4: Add_Encoder_8 	+= 0.55; Add_Encoder_Oled += 0.55; break;
				case 5: Add_Encoder_10 	+= 0.8; Add_Encoder_Oled += 0.8; break;
				case 6: Add_Encoder_Last+= 2.0; Add_Encoder_Oled += 2.0; break;
				default:;
			}
//			PID_9250_Right.kd+=10;
//			PID_F1_Right_Servo.kd +=0.01;
//			PID_F2_Right_Servo.kd +=0.01;
//			PID_F1_Right_Servo.kd += 0.01;
//			PID_F2_Right_Servo.kd += 0.01;
//			delay_ms(100);
//			Motor_Forw();
//			Motor_Right();
//			Motor_Left();
//			Motor_Back();
//			Motor_Set_Speed();
//			Servo_Forw();
//			Servo_Set_Angle();
		}
		
		
		delay_us(20);
//		OLED_Show3FNum(10,0,yaw ,4,2,12);
//		OLED_ShowIntNum(64,0,flag_step,2,12);
		//OLED_ShowIntNum(84,0,flag_huanlong,1,12);
		//OLED_ShowIntNum(96,0,flag_chulong,1,12);
		
//		OLED_Show3FNum(64,0 ,	PID_9250_Back.kp,3,1,12);
//		OLED_Show3FNum(10,12 ,	PID_9250_Back.kd,3,1,12);
		
		
//		OLED_Show3FNum(0,36,PID_9250_Right.kp,5,2,12);
//		OLED_Show3FNum(48,36,PID_9250_Right.kd,5,2,12);
/*********************************************************/
		OLED_Show3FNum(60,36,ultra_F1.DIS ,5,2,12);	//接口5
		OLED_Show3FNum(0,36,ultra_F2.DIS ,5,2,12);	//接口6

		OLED_Show3FNum(60,24,ultra_L1.DIS ,5,2,12);	//接口1
		OLED_Show3FNum(60,12,ultra_L2.DIS,5,2,12);	//接口2
		
		OLED_Show3FNum(0,24,ultra_R1.DIS ,5,2,12);	//接口3
		OLED_Show3FNum(0,12,ultra_R2.DIS ,5,2,12);	//接口4
		
//		OLED_Show3FNum(0,48,PID_F1_Right_Servo.kp ,5,2,12);
//		OLED_Show3FNum(60,48,PID_F1_Right_Servo.kd ,5,2,12);
		OLED_Show3FNum(0,48,Add_Encoder_Flag ,5,2,12);
		OLED_Show3FNum(60,48,Add_Encoder_Oled ,5,2,12);
    
//		OLED_Show3FNum(10,48,RightLightLast ,1,0,12);
//		OLED_Show3FNum(24,48,LeftLightLast ,1,0,12);
//		OLED_Show3FNum(36,48,RightLight ,1,0,12);
//		OLED_Show3FNum(48,48,LeftLight ,1,0,12);
//		OLED_Show3FNum(10,48,PID_F1_Left_Servo.kp ,1,3,12);
//		OLED_Show3FNum(48,48,PID_F1_Left_Servo.kd ,1,3,12);
	  OLED_Refresh_Gram();
	}
}
