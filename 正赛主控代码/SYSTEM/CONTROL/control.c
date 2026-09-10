/*
2025 张瀚文
		        F1 头 F2
			   L1----------R1
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 L2---------- R2
				       尾 
*/
#include "stdio.h"
#include "string.h"
#include "control.h"
#include "sys.h"
#include "delay.h"
//#include "lamp.h"
#include "led.h"
#include "usart1_dma.h"
#include "usart2_dma.h"
#include "usart3_dma.h"
#include "uart4_dma.h"
#include "ultrasonic.h"
#include "oled.h"
#include "encoder.h"
#include "param.h"
#include "motor.h"
#include "pid.h"
#include "servo.h"
#include "io.h"

#define N 0.98
#define Move 180          //直行停车预留量 160
#define move 320          //右行停车预留量 320
#define movez 320         //左行停车预留量 320
#define movez_last 240		//出垄前左行停车预留量

u8 parity	= 0;								//判断奇数偶
float pitch, roll;
extern float yaw;
short aacx, aacy, aacz;
short gyrox, gyroy, gyroz;
u8 flag_step = 0;					//判断现在运行到了第几步的标志
u8 flag_yaw_new = 0;
float Add_Encoder_2 					= 0;
float Add_Encoder_4 					= 0;
float Add_Encoder_6 					= 0;
float Add_Encoder_8 					= 0;
float Add_Encoder_10 					= 0;
float Add_Encoder_Last 				= 0;

char PIDOUT[20];
//int a=0;
/***********************************张瀚文 2024********************************************
0： 入长垄          
1： 右行
2： 换垄
3： 左行
4： 换垄
5： 右行
6： 换垄
7： 左行
8： 换垄
9： 右行
10：换垄
11：左行
12：出垄
13：停车
***********************************************************************************************/
u8 flag_mission_start 		= 0;				//按键按下后，进入控制函数的标志
u8 flag_huanlong 				  = 0;				//换垄标志，此flag=1时正在换垄
u8 flag_chulong 				  = 0;				//出笼标志，此flag=1时已经出笼
u8 flag_slowdown 				  = 0;				//减速标志，此flag=1时表示需要减速
u8 flag_zuowuslowdown 		= 0;				//减速标志，此flag=1时表示需要减速
u8 flag_found_zuowu				= 0;				//检测作物的标志，此flag=1时表示检测到作物，开始减速
u8 flag_delay					    = 0;				//延时标志，此flag=1时表示需要延时
u8 flag_delay_200ms				= 0;				//延时标志，此flag=1时表示需要延时200ms
u8 flag_delay_500ms				= 0;				//延时标志，此flag=1时表示需要延时500ms
u8 flag_delay_1000ms			= 0;				//延时标志，此flag=1时表示需要延时1000ms
u8 flag_delay_1500ms			= 0;				//延时标志，此flag=1时表示需要延时1500ms
u8 flag_delay_zuowu				= 0;				//延时标志，此flag=1时作物表示需要延时
u8 Color_SLOW							= 0;
u8 flag_step0SLOW					= 0;
uint8_t RightLightLast = 2;
uint8_t LeftLightLast = 2;

//float N_F_R = 1.0;
//根据不同步数选择舵机、电机的基本角度、速度
float Servo1_Base = 135, Servo2_Base = 135, Servo3_Base = 135, Servo4_Base = 135;
float Servo5_Base = 0, Servo6_Base = 0, Servo7_Base = 0, Servo8_Base = 0;
float SpeedL1_Base = 0, SpeedR1_Base = 0, SpeedL2_Base = 0, SpeedR2_Base = 0;
float SpeedL1_Base_c = 0, SpeedR1_Base_c = 0, SpeedL2_Base_c = 0, SpeedR2_Base_c = 0;
//根据PID的结果得到的舵机、电机的增量
float Servo1_Increase = 0, Servo2_Increase = 0, Servo3_Increase = 0, Servo4_Increase = 0;
float Servo5_Increase = 0, Servo6_Increase = 0, Servo7_Increase = 0, Servo8_Increase = 0;
float SpeedL1_Increase = 0, SpeedR1_Increase = 0, SpeedL2_Increase = 0, SpeedR2_Increase = 0;
float SpeedL1_Increase_9250 = 0, SpeedR1_Increase_9250 = 0, SpeedL2_Increase_9250 = 0, SpeedR2_Increase_9250 = 0;
//调试基础电机速度
//float Motor_Forw_test_L1 = 0, Motor_Forw_test_L2 = 0, Motor_Forw_test_R1 = 0, Motor_Forw_test_R2 = 0;
void TIM7_IRQHandler()
{

    static u16 time_1ms;
    static u16 time_5ms;
    static u16 time_10ms;
    static u16 time_25ms;
    static u16 time_50ms;
    static u16 time_200ms;
    static u16 time_500ms;
    static u16 time_1000ms;
    static u16 time_1500ms;
    static u16 time_zuowu;
    static u16 lamp_1000ms;

    if(TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) //溢出中断
    {
        if(flag_mission_start == 1)		//按键按下后1、5、10、50ms的任务才能进行
        {
            time_1ms++;
            time_5ms++;
            time_10ms++;
            time_50ms++;
        }

        //下面的则不受按键的影响
        time_25ms++;


        if(flag_delay_200ms)								//延时专用
        {
            time_200ms++;
        }

        if(flag_delay_500ms)
        {
            time_500ms++;
        }

        if(flag_delay_1000ms)
        {
            time_1000ms++;
        }

        if(flag_delay_1500ms)
        {
            time_1500ms++;
        }

        if(flag_delay_zuowu)
        {
            time_zuowu++;
        }

//----------------------------------------------------------------------
        if(time_1ms == 1)
        {
            time_1ms = 0;

        }

//----------------------------------------------------------------------
        if(time_5ms == 5)
        {
					time_5ms = 0;
					Detect_Flag_Step();				//判断运行到第几步了

					Detect_Flag_Mv();
					flag_delay = (flag_delay_200ms || flag_delay_500ms || flag_delay_1000ms);

					if(!flag_delay)						//在不需要延时的时候才运行下面的步骤
					{
							Detect_Flag_Slowdown();		//判断是否需要减速
							Select_Servo_Angle();			//根据步数选择舵机基本角度
							Select_Motor_Speed();			//根据步数选择电机基本速度
							Select_PID();							//根据当前运行的步数来选择用那个PID

							Servo_Set_Angle();				//设置舵机角度
							Motor_Set_Speed();				//设置电机转速

							if(!flag_huanlong)
							{
									TIM5->CNT = 0;
									TIM3->CNT = 0;
									Encoders.disA = 0;
									Encoders.disB = 0;
							}

							if(flag_huanlong)
							{
									Read_Encoder_Cnt();
							}
					}
					if(!flag_delay)
					{
						if(flag_huanlong&&!flag_chulong)
						{
							HuanLong();								//换垄，当换完垄后，flag_step会+1
						}

						if(flag_chulong)
						{
							ChuLong();
						}
					 }

        }
//----------------------------------------------------------------------
        if(time_10ms == 10)
        {
          time_10ms = 0;
					sprintf(PIDOUT,"%f\n",yaw);
//					sprintf(PIDOUT,"%f\n",ultra_F1.DIS);
//					sprintf(PIDOUT,"%f,%f\n",ultra_F1.DIS,yaw);
//					sprintf(PIDOUT,"%f,%f\n",ultra_L1.DIS,ultra_L2.DIS);
					USART1_DMATransfer((uint32_t *)PIDOUT,strlen(PIDOUT));
        }

//----------------------------------------------------------------------
        if(time_25ms == 25)
        {
            time_25ms = 0;

            Ultra_Trig();
						//Yaw_new();
        }

//----------------------------------------------------------------------
        if(time_50ms == 50)
        {
            time_50ms = 0;
        }

//----------------------------------------------------------------------
        if(time_200ms == 200)
        {
            time_200ms = 0;

            flag_delay_200ms = 0;
        }

//----------------------------------------------------------------------
        if(time_500ms == 450)
        {
            time_500ms = 0;

            flag_delay_500ms = 0;
        }

//----------------------------------------------------------------------
        if(time_1000ms == 250)	//250	//换垄时停止的时间
        {
            time_1000ms = 0;

            flag_delay_1000ms = 0;
        }
//----------------------------------------------------------------------
        if(time_1500ms == 2150)	//2150//第0步运行time_1500ms秒后开始以60%速度行驶，2s+后开始减速
        {
            time_1500ms = 0;

            flag_delay_1500ms = 0;
						if(flag_step==0 && flag_step0SLOW==0)
						{
							flag_step0SLOW=1;
							SpeedL1_Base *= 0.625;	//0.75
							SpeedR1_Base *= 0.625;
							SpeedL2_Base *= 0.625;
							SpeedR2_Base *= 0.625;
							
							Motor_Set_Speed();
						}
        }

//----------------------------------------------------------------------
        if(time_zuowu == 400)
        {
					time_zuowu = 0;
        }

    }

    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);			//清除中断标志位
}


void Detect_Flag_Step(void)
{
//第0步转换到第1步-----------------------------------------------------------------
    if((flag_step == 0) && (flag_delay_1500ms == 0) && ((ultra_F1.DIS < F1 + Move) && (ultra_F2.DIS < F2 + Move)))
    {
        flag_step = 1;
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
        Servo_Set_Angle();
				flag_delay_1000ms = 1;
				return;
    }

//----------------------------------------------------------------------

//第1步转换到第2步-----------------------------------------------------------------
		if((flag_step == 1) && (LightStop()))
    {
        flag_step = 2;
        //清除STEP==1时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        Motor_Stop();
        Motor_Set_Speed();				//设置电机转速
				Select_Servo_Angle();			//根据步数选择舵机基本角度
        Servo_Set_Angle();
        flag_delay_1000ms = 1;
        flag_huanlong = 1;
//				GPIO_ResetBits(GPIOE,GPIO_Pin_14);	//板间通讯，给作业板关闭作业模式信号
				return;
    }

//----------------------------------------------------------------------

//第3步转换到第4步-----------------------------------------------------------------
		if((flag_step == 3) && (LightStop()))
    {
				flag_step = 4;
        //清除STEP==2时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
        Servo_Set_Angle();
        flag_delay_1000ms = 1;
				flag_huanlong = 1;
//				GPIO_ResetBits(GPIOE,GPIO_Pin_14);	//板间通讯，给作业板关闭作业模式信号
				return;
    }

//----------------------------------------------------------------------

//第5步转换到第6步-----------------------------------------------------------------
		if((flag_step == 5) && (LightStop()))
    {
				flag_step = 6;
        //清除STEP==3时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
        Servo_Set_Angle();
        flag_delay_1000ms = 1;
        flag_huanlong = 1;
//				GPIO_ResetBits(GPIOE,GPIO_Pin_14);	//板间通讯，给作业板关闭作业模式信号
				return;
    }

//----------------------------------------------------------------------

//第7步转换到第8步-----------------------------------------------------------------
		if((flag_step == 7) && (LightStop()))
    {
        //清除STEP==7时的PID输出
        flag_step = 8;
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
				Servo_Set_Angle();
        flag_delay_1000ms = 1;
        flag_huanlong = 1;
//				GPIO_ResetBits(GPIOE,GPIO_Pin_14);	//板间通讯，给作业板关闭作业模式信号
				return;
    }

//----------------------------------------------------------------------

//第9步转换到第10步-----------------------------------------------------------------
		if((flag_step == 9) && (LightStop()))
    {
				flag_step = 10;
        //清除STEP==10时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();	
				Select_Servo_Angle();			//根据步数选择舵机基本角度
				Servo_Set_Angle();
        flag_delay_1000ms = 1;
				flag_huanlong = 1;
//				GPIO_ResetBits(GPIOE,GPIO_Pin_14);	//板间通讯，给作业板结束作业模式信号
				return;
    }

//----------------------------------------------------------------------

//----------------------------------------------------------------------

//第11步转换到第12步-----------------------------------------------------------------
		if((flag_step == 11) && ((ultra_L1.DIS < L1_L + movez_last) && (ultra_L2.DIS < L2_L + movez_last)))
    {
        //清除STEP==11时的PID输出
				flag_step = 12;
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
        SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
//				delay_ms(160);					//车会飘移
				Select_Servo_Angle();		//根据步数选择舵机基本角度
				Servo_Set_Angle();
				flag_delay_1000ms = 1;
        flag_huanlong = 1;
        flag_chulong = 1;
				GPIO_ResetBits(GPIOE,GPIO_Pin_14);	//板间通讯，给作业板结束作业模式信号
				return;
    }

		 if(flag_step == 13)
    {
        //清除STEP==13时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
        SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				return;
    }
}

void Select_Servo_Angle(void)		 							//根据当前运行的步数来选择舵机基本角度
{
    if((flag_step == 0) || (flag_step == 2) || (flag_step == 4)|| (flag_step == 6) || (flag_step == 8) || (flag_step == 10) || (flag_step == 12))
    {
        Servo_Forw();//Servo_Turn();															//轮子向前
    }

    if((flag_step == 1) || (flag_step == 3) || (flag_step == 5)|| (flag_step == 7) || (flag_step == 9) || (flag_step == 11))
    {
        Servo_Turn();															//轮子转向
    }
}

void Select_Motor_Speed(void)									//根据当前运行的步数来选择电机的基本速度
{
    if(flag_step == 0 && flag_delay_1500ms==1)
    {
        Motor_Forw();//Motor_Right();//Motor_Back();//Motor_Left();
    }

    if((flag_step == 1) || (flag_step == 5) || (flag_step == 9))
    {
        Motor_Right();
    }

    if((flag_step == 3) || (flag_step == 7))
    {
        Motor_Left();
    }

		if(flag_step == 11)
		{
				Motor_Left_last();
		}
		
    if(flag_slowdown)
    {
        Motor_SlowDown();
    }

    if(flag_step == 13)
    {
        Motor_Stop();
    }
}

void Select_PID(void)
{
	
    if(flag_step == 0)
    {
			if((ultra_L1.DIS<150)&&(ultra_L2.DIS<150))
			{
				Control_L1_For_Servo(ultra_L1.DIS);	//用L1超声波的值控制1、2舵机
			  Control_L2_For_Servo(ultra_L2.DIS);	//用L2超声波的值控制3、4舵机
			}
			else if(ultra_L1.DIS<150)
			{
				Control_L1_For_Servo(ultra_L1.DIS);	//用L1超声波的值控制1、2舵机
			}
			Contrl_9250_Forward_motor(yaw);	//第0步未入垄时不用超声波
			
//调PID
//			Contrl_9250_Forward_motor(yaw);
//			Contrl_9250_Right(yaw);	//用yaw角控制车右行，四个差速电机
//			Contrl_9250_Left(yaw);	//用yaw角控制车左行，四个差速电机
//			Contrl_9250_Back_motor(yaw);	//用yaw角控制四个电机差速后退
			
//			Control_L1_For_Servo(ultra_L1.DIS);	//用L1超声波的值控制1、2舵机
//			Control_L2_For_Servo(ultra_L2.DIS);	//用L2超声波的值控制3、4舵机
			
//			Control_F1_Right_Servo(ultra_F1.DIS);	//用F1超声波控制舵机2、4
//			Control_F2_Right_Servo(ultra_F2.DIS);	//用F2超声波控制舵机1、3
//			Control_F1_Left_Servo(ultra_F1.DIS);	//用F1超声波控制2、4舵机
//			Control_F2_Left_Servo(ultra_F2.DIS);	//用F2超声波控制1、3舵机
//			Control_L2_Back_Servo(ultra_L2.DIS);	//用L2超声波控制3、4舵机
//		  Control_L1_Back_Servo(ultra_L1.DIS);	//用L1超声波控制1、2舵机
			return;
    }
		
		
		
		if(flag_step == 1)	
		{
			SpeedL1_Increase = 0;
			SpeedR1_Increase = 0;
			SpeedL2_Increase = 0;
			SpeedR2_Increase = 0;
			
//			EdgeFilter(ultra_F1.DIS,1);
//			EdgeFilter(ultra_F2.DIS,2);
			
			Control_F1_Right_Servo(ultra_F1.DIS);	//用F1超声波控制舵机2、4
			Control_F2_Right_Servo(ultra_F2.DIS);	//用F2超声波控制舵机1、3
			Contrl_9250_Right(yaw);	//用yaw角控制车右行，四个差速电机
			return;
		}
		
		
		
    if( ((flag_step == 5) || (flag_step == 9)) && (!flag_huanlong) )	//右行
		{
				if((ultra_F1.DIS<150)&&(ultra_F2.DIS<150))
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
				
				Control_F2_Right_Servo(ultra_F2.DIS);	//用F2超声波控制舵机1、3
				Control_F1_Right_Servo(ultra_F1.DIS);	//用F1超声波控制舵机2、4
			}
			else if(ultra_F2.DIS<150)
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
				
				Control_F2_Right_Servo(ultra_F2.DIS);	//用F2超声波控制舵机1、3
			}
			else
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
			}
			Contrl_9250_Right(yaw);	//用yaw角控制车右行，四个差速电机
			return;
    }
		
		
		
    if( ((flag_step == 3) || (flag_step == 7) || (flag_step == 11)) && (!flag_huanlong) )	//左行
    {
//			if( (ultra_R1.DIS>240 || (ultra_R2.DIS>240)) && ((ultra_L2.DIS>240) || (ultra_L1.DIS>240)))
			if((ultra_F1.DIS<150)&&(ultra_F2.DIS<150))
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
				
//				EdgeFilter(ultra_F1.DIS,1);
//				EdgeFilter(ultra_F2.DIS,2);
				
				Control_F1_Left_Servo(ultra_F1.DIS);	//用F1超声波控制2、4舵机
				Control_F2_Left_Servo(ultra_F2.DIS);	//用F2超声波控制1、3舵机
			}
			else if(ultra_F1.DIS<150)
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
				
				Control_F1_Left_Servo(ultra_F1.DIS);	//用F1超声波控制2、4舵机
			}
			else
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
			}
			Contrl_9250_Left(yaw);	//用yaw角控制车左行，四个差速电机
			return;
    }
		
		
		
    if( ((flag_step == 2) || (flag_step == 6) || (flag_step == 10)) && flag_huanlong )	//靠右侧换垄
    {
			Contrl_9250_Back_motor(yaw);	//用yaw角控制四个电机差速后退
			Control_R2_Back_Servo(ultra_R2.DIS);	//用R2超声波控制3、4舵机
			Control_R1_Back_Servo(ultra_R1.DIS);	//用R1超声波控制1、2舵机
			return;
    }
		
		
		
    if( ((flag_step == 4) || (flag_step == 8)) && flag_huanlong )	//靠左侧换垄
		{
			Contrl_9250_Back_motor(yaw);	//用yaw角控制四个电机差速后退
			Control_L2_Back_Servo(ultra_L2.DIS);	//用L2超声波控制3、4舵机
		  Control_L1_Back_Servo(ultra_L1.DIS);	//用L1超声波控制1、2舵机
			return;
		}
		
		
		
//		if( (flag_step == 11) && (!flag_huanlong) )	//最后一条垄左行
//    {		
//			if((ultra_F1.DIS<100) && (ultra_F2.DIS<100))
//			{
//				SpeedL1_Increase = 0;
//				SpeedR1_Increase = 0;
//				SpeedL2_Increase = 0;
//				SpeedR2_Increase = 0;
//				
//				Control_F1_Left_Last_Servo(ultra_F1.DIS);	//用F1超声波控制2、4舵机
//				Control_F2_Left_Last_Servo(ultra_F2.DIS);	//用F2超声波控制1、3舵机
//			}
//			else if(ultra_F1.DIS<100)
//			{
//				SpeedL1_Increase = 0;
//				SpeedR1_Increase = 0;
//				SpeedL2_Increase = 0;
//				SpeedR2_Increase = 0;
//				
//				Control_F1_Left_Last_Servo(ultra_F1.DIS);	//用F1超声波控制2、4舵机
//			}
//			Contrl_9250_Left_Last(yaw);	//用yaw角控制车左行，四个差速电机
//			return;
//    }
		
		

		if( flag_step == 12 )
		{
				if((ultra_L1.DIS<150)&&(ultra_L2.DIS<150))
				{
					Control_L2_Backlast_Servo(ultra_L2.DIS);	//用L2超声波的值控制3、4舵机，出垄
					Control_L1_Backlast_Servo(ultra_L1.DIS);	//用L1超声波的值控制1、2舵机，出垄
					Contrl_9250_Back_last_motor(yaw);					//出垄用yaw角控制四个电机
				}
				else if(ultra_L1.DIS<150)
				{
					Control_L1_Backlast_Servo(ultra_L1.DIS);	//用L1超声波的值控制1、2舵机，出垄
					Contrl_9250_Back_last_motor(yaw);					//出垄用yaw角控制四个电机
				}
				else
				{
					Servo1_Increase = 0;
					Servo2_Increase = 0;
					Servo3_Increase = 0;
					Servo4_Increase = 0;
					SpeedL1_Increase = 0;
					SpeedR1_Increase = 0;
					SpeedL2_Increase = 0;
					SpeedR2_Increase = 0;
					Contrl_9250_Back_last_motor(yaw);					//出垄用yaw角控制四个电机
				}
				return;
		}
}

void Detect_Flag_Slowdown(void)		//检测车是否需要减速,光电管识别到树则减速
{
    flag_slowdown = 0;
		flag_found_zuowu = 0;
		Color_SLOW = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15);
		
    if(((flag_step == 1) || (flag_step == 5) || (flag_step == 9)) && (!flag_huanlong) && (Color_SLOW))
    {
      flag_slowdown = 1;
			flag_found_zuowu = 1;
			return;
    }

    if(((flag_step == 3) || (flag_step == 7) || (flag_step == 11)) && (!flag_huanlong) && (Color_SLOW))
    {
      flag_slowdown = 1;
			flag_found_zuowu = 1;
			return;
    }
		
//		if(flag_step==10 && ultra_L1.DIS<550 && ultra_L2.DIS<550)
//		{
//			SpeedL1_Base *= 0.7;
//      SpeedR1_Base *= 0.7;
//      SpeedL2_Base *= 0.7;
//      SpeedR2_Base *= 0.7;
//		}
}

void HuanLong(void)								//换垄，利用编码器
{
    static float	disA_desired, disB_desired;//调距离，编码器
		GPIO_SetBits(GPIOE,GPIO_Pin_14);	//板间通讯，给作业板开启作业模式信号
    Servo_Forw();
    Motor_Back();

		if(flag_step == 2) {disA_desired = -29.5 - Add_Encoder_2;}	//比中间多一点（按有一个毛毯可以跑）：27.1
		
    if(flag_step == 4) {disA_desired = -16.7 - Add_Encoder_4;}	//14.0

    if(flag_step == 6) {disA_desired = -28.0 - Add_Encoder_6;}	//26.6

    if(flag_step == 8) {disA_desired = -15.2 - Add_Encoder_8;}	//14.0
		
		if(flag_step == 10){disA_desired = -31.2 - Add_Encoder_10;}	//28.9
		

    if(Encoders.disA < disA_desired)
    {
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
        Motor_Stop();
        Motor_Set_Speed();							//设置电机转速
        Servo_Turn();
        Servo_Set_Angle();							//设置舵机角度
        flag_delay_1000ms = 1;
        flag_step++;
        flag_huanlong = 0;
        Encoders.disA = 0;
        Encoders.disB = 0;
    }
}

void ChuLong(void)									//出笼，利用编码器
{
    Servo_Forw();
    Motor_Back_last();

    if(Encoders.disA < (-15 - Add_Encoder_Last))
    {
        Motor_Stop();
        Motor_Set_Speed();							//设置电机转速
        flag_delay_1000ms = 1;
        flag_step++;
        flag_huanlong = 0;
        flag_chulong = 0;
        Encoders.disA = 0;
        Encoders.disB = 0;
				flag_mission_start = 0;
    }
}
/*****************************************四轮舵机控制********************************************/
//对应车子所标

void Servo_Forw_pre(void)							//车轮向前的准备
{
    Servo1_Base = 118;
    Servo2_Base = 120;
    Servo3_Base = 130;
    Servo4_Base = 123;
}

void Servo_Forw(void)							//车轮向前
{
    Servo1_Base = 117;			//值变大，顺时针转
    Servo2_Base = 126;
    Servo3_Base = 137;
    Servo4_Base = 91;
}

void Servo_Turn(void)							//车轮转向
{
    Servo1_Base = 205;		//+90
    Servo2_Base = 35;			//-90
    Servo3_Base = 49;
    Servo4_Base = 182;
}
/*****************************************四轮电机控制********************************************/

void Motor_Forw(void)								//前行
{
    SpeedL1_Base = 6000;
    SpeedR1_Base = 6000;
    SpeedL2_Base = 6000;	//5000
    SpeedR2_Base = 6000;	//5000
}

void Motor_Back(void)								//后行,换垄
{
    SpeedL1_Base = -3542;
    SpeedR1_Base = -3500;	//3500
    SpeedL2_Base = -3500;
    SpeedR2_Base = -3542;	//3542
}

void Motor_Back_last(void)					//后行，出垄
{
    SpeedL1_Base = -4554;	//4554
    SpeedR1_Base = -4500;	//4500
    SpeedL2_Base = -4500;
    SpeedR2_Base = -4554;
}

void Motor_Right(void)							//右行
{
    SpeedL1_Base = 5000*N;
    SpeedR1_Base = -5000*N;
    SpeedL2_Base = -5000*N;
    SpeedR2_Base = 5000*N;
}
void Motor_Left(void)								//左行
{
    SpeedL1_Base = -5000*N;
    SpeedR1_Base = 5000*N;
    SpeedL2_Base = 5000*N;
    SpeedR2_Base = -5000*N;
}

void Motor_Left_last(void)					//左行，最后一条垄
{
    SpeedL1_Base = -5000;
    SpeedR1_Base = 5000;
    SpeedL2_Base = 5000;
    SpeedR2_Base = -5000;
}

void Motor_Stop(void)								//车停止
{
    SpeedL1_Base = 0;
    SpeedR1_Base = 0;
    SpeedL2_Base = 0;
    SpeedR2_Base = 0;
}

void Motor_SlowDown(void)						//减速
{
    if(flag_found_zuowu)
    {
        SpeedL1_Base *= 0.15;
        SpeedR1_Base *= 0.15;
        SpeedL2_Base *= 0.15;
        SpeedR2_Base *= 0.15;	//0.3
    }	
}

void IQR_Delay(unsigned int n)		//中断中用的延时函数
{
    unsigned int i, j;

    for(j = 0; j < 50 * n; j++)
    {
        for(i = 0; i < 400; i++);
    }
}

void Detect_Flag_Mv()	//传输给作业板，作业板再传输给摄像头
{
	if((flag_step == 5||flag_step==9)) //&& (!RightLight))	//右行，右摄像头的探测信息有效
	{
//		Mv1_1 = 0;
//		Mv1_2 = 1;
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		GPIO_SetBits(GPIOE,GPIO_Pin_8);
	}
	else if((flag_step == 3||flag_step == 7||flag_step == 11)) //&& (!LeftLight))		//左行，左摄像头的探测信息有效
	{
//		Mv1_1 = 1;
//		Mv1_2 = 0;
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
	}
	else
	{
//		Mv1_1 = 0;
//		Mv1_2 = 0;
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
	}
}

uint8_t LightStop()		//垄中光电管停车，当对应光电管从1变到0时停车
{
	uint8_t Current_RightLight,Current_LeftLight;
	Current_RightLight = RightLight;
	Current_LeftLight = LeftLight;
	if((flag_step==1 || flag_step==5 || flag_step==9) && Current_RightLight==1 && LeftLightLast==0 && RightLightLast==0) //右行，右侧光电管0->1，
	{
		RightLightLast = Current_RightLight;
		LeftLightLast = Current_LeftLight;
		delay_ms(34);
		return 1;
	}
	else if((flag_step==3 || flag_step==7) && Current_LeftLight==1 && RightLightLast==0 && LeftLightLast==0)	//左行，左边光电管0->1
	{
		RightLightLast = Current_RightLight;
		LeftLightLast = Current_LeftLight;
		delay_ms(35);
		return 1;
	}
//	else if(flag_step==11 && Current_LeftLight==1 && LeftLightLast==0 && RightLightLast==0)	//11->12左侧（同侧）光电管
//	{
//		RightLightLast = Current_RightLight;
//		LeftLightLast = Current_LeftLight;
//		delay_ms(7);
//		return 1;
//	}
	RightLightLast = Current_RightLight;
	LeftLightLast = Current_LeftLight;
	return 0;
}
