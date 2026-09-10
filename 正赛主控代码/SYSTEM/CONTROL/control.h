#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f4xx.h"
#include "ultrasonic.h"

#define LeftLight GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)		//RX3--PD9
#define RightLight GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)	//RX4--PC11

extern float pitch, roll, yaw;
extern short aacx, aacy, aacz;
extern short gyrox, gyroy, gyroz;

extern float Add_Encoder_2;
extern float Add_Encoder_4;
extern float Add_Encoder_6;
extern float Add_Encoder_8;
extern float Add_Encoder_10;
extern float Add_Encoder_Last;

extern u8 flag_step;
extern u8 flag_huanlong;
extern u8 flag_chulong;
extern u8 flag_mission_start,flag_delay_1500ms;		//按键按下后，进入控制函数的标志
extern uint8_t RightLightLast;
extern uint8_t LeftLightLast;
//根据不同步数选择舵机、电机的基本角度、速度
extern float Servo1_Base, Servo2_Base, Servo3_Base, Servo4_Base;		
extern float SpeedL1_Base, SpeedR1_Base, SpeedL2_Base, SpeedR2_Base;
//根据PID的结果得到的舵机、电机的增量
extern float Servo1_Increase, Servo2_Increase, Servo3_Increase, Servo4_Increase;
extern float Servo5_Increase, Servo6_Increase, Servo7_Increase, Servo8_Increase;
extern float SpeedL1_Increase, SpeedR1_Increase, SpeedL2_Increase, SpeedR2_Increase;
extern float SpeedL1_Increase_9250, SpeedR1_Increase_9250, SpeedL2_Increase_9250, SpeedR2_Increase_9250;
//调试基础电机速度
//extern float Motor_Forw_test_L1,Motor_Forw_test_L2,Motor_Forw_test_R1,Motor_Forw_test_R2;

void Detect_Flag_Step(void); 				//判断现在运行到了第几步 一共7步 根据不同的步数选择舵机基本角度、电机基本速度、PID
void Detect_Flag_Slowdown(void);		//检测车是否需要减速
void Select_Servo_Angle(void);		 	//根据当前运行的步数来选择舵机的基本角度
void Select_Motor_Speed(void);			//根据当前运行的步数来选择电机的基本速度
void Select_PID(void);							//根据当前运行的步数来选择用那个PID
void Servo_Forw_pre(void);					//车轮向前准备（新加）
void Servo_Forw(void);							//车轮向前
void Servo_Turn(void);							//车轮转向
void Motor_SlowDown(void);					//车子减速
void Motor_Forw(void);							//车轮向前转
void Motor_Back(void);							//车轮向前转，换垄
void Motor_Back_last(void);					//车轮向前转,出垄
void Motor_Right(void);							//车轮向前转
void Motor_Left(void);							//车轮向前转
void Motor_Left_last(void);					//车轮向左，回长垄
void Motor_Stop(void);							//车停止
void HuanLong(void);								//换垄，利用编码器
void ChuLong(void);									//出笼，利用编码器
void IQR_Delay(unsigned int n);	  	//中断中用的延时函数
void Detect_Flag_Mv();

uint8_t LightStop();

#endif

