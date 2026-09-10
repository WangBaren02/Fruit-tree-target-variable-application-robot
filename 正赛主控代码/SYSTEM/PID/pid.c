#include "pid.h"
#include "math.h"
#include "motor.h"
#include "control.h"

#define n 1
#define speed_r_l 100
#define speed_r 80
#define speed_l 80

#define speed_r_9250 5
#define speed_l_9250 5

#define speed_r_9250_s 0.6
#define speed_l_9250_s 0.6

#define speed_r_back 120
#define speed_l_back 120

#define speed_r_j 100
#define speed_l_j 100

#define speed_r_r 100
#define speed_l_r 100

//电机A和电机B的转速闭环PID
PidObject PID_PWMA;
PidObject PID_PWMB;

//直走时的9250闭环
PidObject PID_9250_Forward;
PidObject PID_9250_Forward_motor;
//向右走的9250闭环
PidObject PID_9250_Right;
//向左走的9250闭环
PidObject PID_9250_Left;
PidObject PID_9250_Left_Last;

//降速向右走的9250闭环
PidObject PID_9250_Right_slow;
//降速向左走的9250闭环
PidObject PID_9250_Left_slow;

//向后走的9250闭环
PidObject PID_9250_Back;
PidObject PID_9250_Back_motor;
//出垄的9250闭环
PidObject PID_9250_Back_last;
PidObject PID_9250_Back_last_motor;

//靠左前行时舵机进行闭环
PidObject PID_L1_For_Servo;
PidObject PID_L2_For_Servo;
//靠左后退时舵机进行闭环
PidObject PID_L1_Back_Servo;
PidObject PID_L2_Back_Servo;

PidObject PID_L1_Back_Last_Servo;
PidObject PID_L2_Back_Last_Servo;
//靠右前行时舵机进行闭环
PidObject PID_R1_For_Servo;
//靠右后退时舵机进行闭环
PidObject PID_R2_Back_Servo;
PidObject PID_R1_Back_Servo;

//舵机左行右行
PidObject PID_F1_Left_Servo;
PidObject PID_F2_Left_Servo;

PidObject PID_F1_Left_Last_Servo;
PidObject PID_F2_Left_Last_Servo;

PidObject PID_F1_Right_Servo;
PidObject PID_F2_Right_Servo;

//电机左行右行
PidObject PID_F1_Left_Motor;
PidObject PID_F2_Right_Motor;

PidObject PID_L1_For_Motor;
PidObject PID_L2_For_Motor;

//换垄时，用舵机保持和垄的固定距离
PidObject PID_Back_L_Servo;
PidObject PID_Back_R_Servo;

PidObject PID_L1_Left_Servo;
PidObject PID_R1_Right_Servo;

void PID_Init(void)
{
	PID_R1_Right_Servo.desired = R1 - R2;
	//比例
	PID_R1_Right_Servo.kp = 0.3;
	//积分
	PID_R1_Right_Servo.ki = 0;
	PID_R1_Right_Servo.integ_sep = 0;
	PID_R1_Right_Servo.dt = 0;
	PID_R1_Right_Servo.iLimit = 0;
	//微分
	PID_R1_Right_Servo.kd = 0.2;
	//输出限幅
	PID_R1_Right_Servo.outputLimit = 10;
	
	PID_L1_Left_Servo.desired = L1 - L2;
	//比例
	PID_L1_Left_Servo.kp = 0.3;
	//积分
	PID_L1_Left_Servo.ki = 0;
	PID_L1_Left_Servo.integ_sep = 0;
	PID_L1_Left_Servo.dt = 0;
	PID_L1_Left_Servo.iLimit = 0;
	//微分
	PID_L1_Left_Servo.kd = 0.2;
	//输出限幅
	PID_L1_Left_Servo.outputLimit = 10;
/*
*************************************
* 		电机A转速闭环PID参数初始化			
*************************************
*/	
	PID_PWMA.desired = 50;				//电机A的转速期望
	PID_PWMA.kp = 10;							//电机A转速闭环P
	PID_PWMA.ki = 0;
	PID_PWMA.integ_sep = 0;
	PID_PWMA.dt = 0;
	PID_PWMA.iLimit = 0;
	
/*
*************************************
* 		直走时9250PID参数初始化			
*************************************
*/	
	PID_9250_Forward.desired = 0;
	//比例
	PID_9250_Forward.kp = 0.7;	//done
	//积分
	PID_9250_Forward.ki = 0;
	PID_9250_Forward.integ_sep = 0;
	PID_9250_Forward.dt = 0;
	PID_9250_Forward.iLimit = 0;
	//微分
	PID_9250_Forward.kd = 1.25;	//done
	//输出限幅
	PID_9250_Forward.outputLimit = 6;		//6
	
	
	PID_9250_Forward_motor.desired = 0;
	//比例
	PID_9250_Forward_motor.kp = 260;	//5000：200,6000:280
	//积分
	PID_9250_Forward_motor.ki = 0;
	PID_9250_Forward_motor.integ_sep = 0;
	PID_9250_Forward_motor.dt = 0;
	PID_9250_Forward_motor.iLimit = 2000*n;
	//微分
	PID_9250_Forward_motor.kd = 150;	//100
	//输出限幅
	PID_9250_Forward_motor.outputLimit = 2800*n;	//1800
/*
*************************************
* 		右行时9250PID参数初始化
*************************************
*/
	PID_9250_Right.desired = 0;
	//比例
	PID_9250_Right.kp = 98;//80
	//积分
	PID_9250_Right.ki = 0;
	PID_9250_Right.integ_sep = 0;
	PID_9250_Right.dt = 0;
	PID_9250_Right.iLimit = 600*n;	
	//微分
	PID_9250_Right.kd = 200;	//70
	//输出限幅
	PID_9250_Right.outputLimit = 1800*n;	//800
	
	PID_9250_Right_slow.desired = 0;
	//比例
	PID_9250_Right_slow.kp = 400;
	//积分
	PID_9250_Right_slow.ki = 0;
	PID_9250_Right_slow.integ_sep = 2;
	PID_9250_Right_slow.dt = 1;
	PID_9250_Right_slow.iLimit = 600*n;
	//微分
	PID_9250_Right_slow.kd = 300;
	//输出限幅
	PID_9250_Right_slow.outputLimit = 1400*n;	//800
	
	
/*
*************************************
* 		左行时9250PID参数初始化			
*************************************
*/	
	PID_9250_Left.desired = 0;
	//比例
	PID_9250_Left.kp = 98;
	//积分
	PID_9250_Left.ki = 0;
	PID_9250_Left.integ_sep = 0;
	PID_9250_Left.dt = 0;
	PID_9250_Left.iLimit = 600*n;
	//微分
	PID_9250_Left.kd = 200;	//50
	//输出限幅
	PID_9250_Left.outputLimit = 1800*n;
	
	
	PID_9250_Left_Last.desired = 0;
	//比例
	PID_9250_Left_Last.kp = 100;
	//积分
	PID_9250_Left_Last.ki = 0;
	PID_9250_Left_Last.integ_sep = 0;
	PID_9250_Left_Last.dt = 0;
	PID_9250_Left_Last.iLimit = 600*n;
	//微分
	PID_9250_Left_Last.kd = 70;	
	//输出限幅
	PID_9250_Left_Last.outputLimit = 800*n;	//800
	
	
	PID_9250_Left_slow.desired = 0;
	//比例
	PID_9250_Left_slow.kp = 400;
	//积分
	PID_9250_Left_slow.ki = 0;
	PID_9250_Left_slow.integ_sep = 2;
	PID_9250_Left_slow.dt = 1;
	PID_9250_Left_slow.iLimit = 600*n;
	//微分
	PID_9250_Left_slow.kd = 300;
	//输出限幅
	PID_9250_Left_slow.outputLimit = 800*n;

/*
*************************************
* 		向后走时9250PID参数初始化			
*************************************
*/
/****************换垄****************/
	PID_9250_Back.desired = 0;
	//比例
	PID_9250_Back.kp = 0.72;
	//积分
	PID_9250_Back.ki = 0;
	PID_9250_Back.integ_sep = 0;
	PID_9250_Back.dt = 0;
	PID_9250_Back.iLimit = 0;
	//微分
	PID_9250_Back.kd = 1.5;
	//输出限幅
	PID_9250_Back.outputLimit = 15;
	
	
	PID_9250_Back_motor.desired = 0;
	//比例
	PID_9250_Back_motor.kp = 150;
	//积分
	PID_9250_Back_motor.ki = 0;
	PID_9250_Back_motor.integ_sep = 0;
	PID_9250_Back_motor.dt = 0;
	PID_9250_Back_motor.iLimit = 2000*n;
	//微分
	PID_9250_Back_motor.kd = 50;
	//输出限幅
	PID_9250_Back_motor.outputLimit = 1200*n;
	
/****************出垄****************/
	PID_9250_Back_last.desired = 0;
	//比例
	PID_9250_Back_last.kp = 0.6;
	//积分
	PID_9250_Back_last.ki = 0;
	PID_9250_Back_last.integ_sep = 0;
	PID_9250_Back_last.dt = 0;
	PID_9250_Back_last.iLimit = 0;
	//微分
	PID_9250_Back_last.kd = 1.5;
	//输出限幅
	PID_9250_Back_last.outputLimit = 30;
	
	
	PID_9250_Back_last_motor.desired = 0;
	//比例
	PID_9250_Back_last_motor.kp = 240;
	//积分
	PID_9250_Back_last_motor.ki = 0;
	PID_9250_Back_last_motor.integ_sep = 0;
	PID_9250_Back_last_motor.dt = 0;
	PID_9250_Back_last_motor.iLimit = 600*n;
	//微分
	PID_9250_Back_last_motor.kd = 100;
	//输出限幅
	PID_9250_Back_last_motor.outputLimit = 600*n;
/*
*************************************
* 		靠左前行时pid初始化			超声波
*************************************
*/
	PID_L1_For_Servo.desired = L1_F;
	//比例
	PID_L1_For_Servo.kp = 0.135;	//0.096
	//积分
	PID_L1_For_Servo.ki = 0;
	PID_L1_For_Servo.integ_sep = 0;
	PID_L1_For_Servo.dt = 0;
	PID_L1_For_Servo.iLimit = 0;
	//微分
	PID_L1_For_Servo.kd = 0.2;		//0.2
	//输出限幅
	PID_L1_For_Servo.outputLimit = 12;	//8
	
	PID_L2_For_Servo.desired = L2_F;
	//比例
	PID_L2_For_Servo.kp = 0.135;
	//积分
	PID_L2_For_Servo.ki = 0;
	PID_L2_For_Servo.integ_sep = 0;
	PID_L2_For_Servo.dt = 0;
	PID_L2_For_Servo.iLimit = 0;
	//微分
	PID_L2_For_Servo.kd = 0.2;
	//输出限幅
	PID_L2_For_Servo.outputLimit = 12;
/*
*************************************
* 		靠左后退时pid初始化		  超声波
*************************************
*/
  PID_L1_Back_Servo.desired = L1;
	//比例
	PID_L1_Back_Servo.kp = 0.2;
	//积分
	PID_L1_Back_Servo.ki = 0;
	PID_L1_Back_Servo.integ_sep = 0;
	PID_L1_Back_Servo.dt = 0;
	PID_L1_Back_Servo.iLimit = 0;
	//微分
	PID_L1_Back_Servo.kd = 0.2;
	//输出限幅
	PID_L1_Back_Servo.outputLimit = 5;           //5

	PID_L2_Back_Servo.desired = L2;
	//比例
	PID_L2_Back_Servo.kp = 0.2;
	//积分
	PID_L2_Back_Servo.ki = 0;
	PID_L2_Back_Servo.integ_sep = 0;
	PID_L2_Back_Servo.dt = 0;
	PID_L2_Back_Servo.iLimit = 0;
	//微分
	PID_L2_Back_Servo.kd = 0.2;
	//输出限幅
	PID_L2_Back_Servo.outputLimit = 5;  

	PID_L1_Back_Last_Servo.desired = L1_L;
	//比例
	PID_L1_Back_Last_Servo.kp = 0.2;
	//积分
	PID_L1_Back_Last_Servo.ki = 0;
	PID_L1_Back_Last_Servo.integ_sep = 0;
	PID_L1_Back_Last_Servo.dt = 0;
	PID_L1_Back_Last_Servo.iLimit = 0;
	//微分
	PID_L1_Back_Last_Servo.kd = 0.2;
	//输出限幅
	PID_L1_Back_Last_Servo.outputLimit = 5;           //5

	PID_L2_Back_Last_Servo.desired = L2_L;
	//比例
	PID_L2_Back_Last_Servo.kp = 0.2;
	//积分
	PID_L2_Back_Last_Servo.ki = 0;
	PID_L2_Back_Last_Servo.integ_sep = 0;
	PID_L2_Back_Last_Servo.dt = 0;
	PID_L2_Back_Last_Servo.iLimit = 0;
	//微分
	PID_L2_Back_Last_Servo.kd = 0.2;
	//输出限幅
	PID_L2_Back_Last_Servo.outputLimit = 5; 
	
/*
*************************************
* 		靠右前行时pid初始化		  超声波
*************************************
*/
	PID_R1_For_Servo.desired = R1;
	//比例
	PID_R1_For_Servo.kp = 0.1;
	//积分
	PID_R1_For_Servo.ki = 0.0015;
	PID_R1_For_Servo.integ_sep = 0.005;
	PID_R1_For_Servo.dt = 0.0001;
	PID_R1_For_Servo.iLimit = 4;
	//微分
	PID_R1_For_Servo.kd = 0;
	//输出限幅
	PID_R1_For_Servo.outputLimit = 10;
	
/*
*************************************
* 		靠右后退时pid初始化		  超声波
*************************************
*/
  PID_R1_Back_Servo.desired = R1;
	//比例
	PID_R1_Back_Servo.kp = 0.2;
	//积分
	PID_R1_Back_Servo.ki = 0;
	PID_R1_Back_Servo.integ_sep = 0;
	PID_R1_Back_Servo.dt = 0;
	PID_R1_Back_Servo.iLimit = 0;
	//微分
	PID_R1_Back_Servo.kd = 0.2;
	//输出限幅
	PID_R1_Back_Servo.outputLimit = 5;	


	PID_R2_Back_Servo.desired = R2;
	//比例
	PID_R2_Back_Servo.kp = 0.2;
	//积分
	PID_R2_Back_Servo.ki = 0;
	PID_R2_Back_Servo.integ_sep = 0;
	PID_R2_Back_Servo.dt = 0;
	PID_R2_Back_Servo.iLimit = 0;
	//微分
	PID_R2_Back_Servo.kd = 0.2;
	//输出限幅
	PID_R2_Back_Servo.outputLimit = 5;	



/*
*************************************
* 		超声波控制舵机左行pid			
*************************************
*/
//	PID_F1_Left_Servo.desired = 129;
//	//比例
//	PID_F1_Left_Servo.kp = 0.35;
//	//积分
//	PID_F1_Left_Servo.ki = 0.002;
//	PID_F1_Left_Servo.integ_sep = 30;
//	PID_F1_Left_Servo.dt = 1;
//	PID_F1_Left_Servo.iLimit = 4;
//	//微分
//	PID_F1_Left_Servo.kd = 0;
//	//输出限幅
//	PID_F1_Left_Servo.outputLimit = 10;	
//	
//	
//	PID_F2_Left_Servo.desired = 119;
//	//比例
//	PID_F2_Left_Servo.kp = 0.35;
//	//积分
//	PID_F2_Left_Servo.ki = 0.002;
//	PID_F2_Left_Servo.integ_sep = 30;
//	PID_F2_Left_Servo.dt = 1;
//	PID_F2_Left_Servo.iLimit = 4;
//	//微分
//	PID_F2_Left_Servo.kd = 0;
//	//输出限幅
//	PID_F2_Left_Servo.outputLimit = 10;	
	
	PID_F1_Left_Servo.desired = F1;
	//比例
	PID_F1_Left_Servo.kp = 0.32;	//0.26
	//积分
	PID_F1_Left_Servo.ki = 0;
	PID_F1_Left_Servo.integ_sep = 0;
	PID_F1_Left_Servo.dt = 0;
	PID_F1_Left_Servo.iLimit = 0;
	//微分
	PID_F1_Left_Servo.kd = 0.25;
	//输出限幅
	PID_F1_Left_Servo.outputLimit = 8;//5
	
	
	PID_F2_Left_Servo.desired = F2;
	//比例
	PID_F2_Left_Servo.kp = 0.32;
	//积分
	PID_F2_Left_Servo.ki = 0;
	PID_F2_Left_Servo.integ_sep = 0;
	PID_F2_Left_Servo.dt = 0;
	PID_F2_Left_Servo.iLimit = 0;
	//微分
	PID_F2_Left_Servo.kd = 0.25;
	//输出限幅
	PID_F2_Left_Servo.outputLimit = 8;
	
	
	PID_F1_Left_Last_Servo.desired = F1;
	//比例
	PID_F1_Left_Last_Servo.kp = 0.26;
	//积分
	PID_F1_Left_Last_Servo.ki = 0;
	PID_F1_Left_Last_Servo.integ_sep = 0;
	PID_F1_Left_Last_Servo.dt = 0;
	PID_F1_Left_Last_Servo.iLimit = 0;
	//微分
	PID_F1_Left_Last_Servo.kd = 0.25;
	//输出限幅
	PID_F1_Left_Last_Servo.outputLimit = 8;//5
	
	PID_F2_Left_Last_Servo.desired = F1;
	//比例
	PID_F2_Left_Last_Servo.kp = 0.18;
	//积分
	PID_F2_Left_Last_Servo.ki = 0;
	PID_F2_Left_Last_Servo.integ_sep = 0;
	PID_F2_Left_Last_Servo.dt = 0;
	PID_F2_Left_Last_Servo.iLimit = 0;
	//微分
	PID_F2_Left_Last_Servo.kd = 0.1;
	//输出限幅
	PID_F2_Left_Last_Servo.outputLimit = 5;//5
/*
*************************************
* 		F12超声波控制舵机右行pid			
*************************************
*/
  PID_F1_Right_Servo.desired = F1;
	//比例
	PID_F1_Right_Servo.kp = 0.32;//0.26
	//积分
	PID_F1_Right_Servo.ki = 0;
	PID_F1_Right_Servo.integ_sep = 0;
	PID_F1_Right_Servo.dt = 0;
	PID_F1_Right_Servo.iLimit = 0;
	//微分
	PID_F1_Right_Servo.kd = 0.25;//0.1
	//输出限幅
	PID_F1_Right_Servo.outputLimit = 8;	//8


	PID_F2_Right_Servo.desired = F2;
	//比例
	PID_F2_Right_Servo.kp = 0.32;
	//积分
	PID_F2_Right_Servo.ki = 0;
	PID_F2_Right_Servo.integ_sep = 0;
	PID_F2_Right_Servo.dt = 0;
	PID_F2_Right_Servo.iLimit = 0;
	//微分
	PID_F2_Right_Servo.kd = 0.25;
	//输出限幅
	PID_F2_Right_Servo.outputLimit = 8;
/*
*************************************
* 		F1超声波控制电机左行pid			
*************************************
*/
	PID_F1_Left_Motor.desired = F1;
	//比例
	PID_F1_Left_Motor.kp = 400;
	//积分
	PID_F1_Left_Motor.ki = 3;
	PID_F1_Left_Motor.integ_sep = 10;
	PID_F1_Left_Motor.dt = 1;
	PID_F1_Left_Motor.iLimit = 0;
	//微分
	PID_F1_Left_Motor.kd = 0;
	//输出限幅
	PID_F1_Left_Motor.outputLimit = 300;	
/*
*************************************
* 		F2超声波控制电机右行pid			
*************************************
*/
	PID_F2_Right_Motor.desired = F2;
	//比例
	PID_F2_Right_Motor.kp = 400;
	//积分
	PID_F2_Right_Motor.ki = 3;
	PID_F2_Right_Motor.integ_sep = 10;
	PID_F2_Right_Motor.dt = 1;
	PID_F2_Right_Motor.iLimit = 300;
	//微分
	PID_F2_Right_Motor.kd = 0;
	//输出限幅
	PID_F2_Right_Motor.outputLimit = 300;	
	
/*



*************************************
* 		L1超声波控制电机前行pid			
*************************************
*/
	PID_L1_For_Motor.desired = L1;
	//比例
	PID_L1_For_Motor.kp = 400;
	//积分
	PID_L1_For_Motor.ki = 3;
	PID_L1_For_Motor.integ_sep = 10;
	PID_L1_For_Motor.dt = 1;
	PID_L1_For_Motor.iLimit = 800;
	//微分
	PID_L1_For_Motor.kd = 0;
	//输出限幅
	PID_L1_For_Motor.outputLimit = 1000;	
/*
*************************************
* 		L2超声波控制电机前行pid			
*************************************
*/
	PID_L2_For_Motor.desired = L2;
	//比例
	PID_L2_For_Motor.kp = 400;
	//积分
	PID_L2_For_Motor.ki = 3;
	PID_L2_For_Motor.integ_sep = 10;
	PID_L2_For_Motor.dt = 1;
	PID_L2_For_Motor.iLimit = 800;
	//微分
	PID_L2_For_Motor.kd = 0;
	//输出限幅
	PID_L2_For_Motor.outputLimit = 1000;	
	
/*
*************************************
* 		左侧换垄时，前面两舵机PID参数初始化			
*************************************
*/		
	PID_Back_L_Servo.desired = 70;
	//比例
	PID_Back_L_Servo.kp = 0.4;
	//积分
	PID_Back_L_Servo.ki = 0;
	PID_Back_L_Servo.integ_sep = 0;
	PID_Back_L_Servo.dt = 0;
	PID_Back_L_Servo.iLimit = 0;
	//微分
	PID_Back_L_Servo.kd = 0;
	//输出限幅
	PID_Back_L_Servo.outputLimit = 10;
	
/*
*************************************
* 		右侧换垄时，前面两舵机PID参数初始化			
*************************************
*/		
	PID_Back_R_Servo.desired = 70;
	//比例
	PID_Back_R_Servo.kp = 0.4;
	//积分
	PID_Back_R_Servo.ki = 0;
	PID_Back_R_Servo.integ_sep = 0;
	PID_Back_R_Servo.dt = 0;
	PID_Back_R_Servo.iLimit = 0;
	//微分
	PID_Back_R_Servo.kd = 0;
	//输出限幅
	PID_Back_R_Servo.outputLimit = 10;
}

void Pid_Update(PidObject* pid, const float error)
{
	float output;
	pid->error = error;
	
	if((pid->error < pid->integ_sep) && (pid->error > -pid->integ_sep))
		pid->integ += pid->error * pid->dt;
	else pid->integ = 0;
	
	if(pid->integ > pid->iLimit )
		pid->integ = pid->iLimit;
	else if(pid->integ < -pid->iLimit)
		pid->integ = -pid->iLimit;
	
	pid->deriv = pid->error - pid->preError;
	
	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;
	
	output = pid->outP + pid->outI + pid->outD;
	
	if(pid->outputLimit!=0)
	{
		if(output > pid->outputLimit)
			output = pid->outputLimit;
		else if(output < -pid->outputLimit)
			output = -pid->outputLimit;
	}
	
	pid->preError = pid->error;
	pid->out = output;
}

void Control_L1_For_Servo(float L1_dis)				//用L1超声波的值控制1、2舵机
{
	PID_L1_For_Servo.error = PID_L1_For_Servo.desired - L1_dis;
	Pid_Update(&PID_L1_For_Servo, PID_L1_For_Servo.error);
	Servo1_Increase = PID_L1_For_Servo.out;
	Servo2_Increase = PID_L1_For_Servo.out;
}

void Control_L2_For_Servo(float L2_dis)				//用L2超声波的值控制3、4舵机
{
	PID_L2_For_Servo.error = PID_L2_For_Servo.desired - L2_dis;
	Pid_Update(&PID_L2_For_Servo, PID_L2_For_Servo.error);
	Servo3_Increase = PID_L2_For_Servo.out;
	Servo4_Increase = PID_L2_For_Servo.out;
}


void Control_L1_Back_Servo(float L1_dis)				//用L1超声波的值控制1、2舵机
{
	PID_L1_Back_Servo.error = PID_L1_Back_Servo.desired - L1_dis;
	Pid_Update(&PID_L1_Back_Servo, PID_L1_Back_Servo.error);
	Servo1_Increase = (-PID_L1_Back_Servo.out);
	Servo2_Increase = (-PID_L1_Back_Servo.out);
	
//	SpeedL2_Increase = -speed_l_back*PID_L1_Back_Servo.out;
//	SpeedR2_Increase = +speed_l_back*PID_L1_Back_Servo.out; 
}

void Control_L2_Back_Servo(float L2_dis)				//用L2超声波的值控制3、4舵机
{
	PID_L2_Back_Servo.error = PID_L2_Back_Servo.desired - L2_dis;
	Pid_Update(&PID_L2_Back_Servo, PID_L2_Back_Servo.error);
	Servo3_Increase = (-PID_L2_Back_Servo.out);
	Servo4_Increase = (-PID_L2_Back_Servo.out);
	
//  SpeedL1_Increase = +speed_l_back*PID_L2_Back_Servo.out;
//	SpeedR1_Increase = -speed_l_back*PID_L2_Back_Servo.out;
}

//void Control_L1_Backlast_Servo(float L1_dis)				//用L1超声波的值控制1、2舵机,出垄
//{
//	PID_L1_Back_Servo.error = PID_L1_Back_Servo.desired - 23 - L1_dis;
//	Pid_Update(&PID_L1_Back_Servo, PID_L1_Back_Servo.error);
//	Servo1_Increase = (-PID_L1_Back_Servo.out);
//	Servo2_Increase = (-PID_L1_Back_Servo.out);
//}

//void Control_L2_Backlast_Servo(float L2_dis)				//用L2超声波的值控制3、4舵机，出垄
//{
//	PID_L2_Back_Servo.error = PID_L2_Back_Servo.desired - 23 - L2_dis;
//	Pid_Update(&PID_L2_Back_Servo, PID_L2_Back_Servo.error);
//	Servo3_Increase = (-PID_L2_Back_Servo.out);
//	Servo4_Increase = (-PID_L2_Back_Servo.out);
//}

void Control_L1_Backlast_Servo(float L1_dis)				//用L1超声波的值控制1、2舵机,出垄
{
	PID_L1_Back_Last_Servo.error = PID_L1_Back_Last_Servo.desired - L1_dis;
	Pid_Update(&PID_L1_Back_Last_Servo, PID_L1_Back_Last_Servo.error);
	Servo1_Increase = (-PID_L1_Back_Last_Servo.out);
	Servo2_Increase = (-PID_L1_Back_Last_Servo.out);
}

void Control_L2_Backlast_Servo(float L2_dis)				//用L2超声波的值控制3、4舵机，出垄
{
	PID_L2_Back_Last_Servo.error = PID_L2_Back_Last_Servo.desired - L2_dis;
	Pid_Update(&PID_L2_Back_Last_Servo, PID_L2_Back_Last_Servo.error);
	Servo3_Increase = (-PID_L2_Back_Last_Servo.out);
	Servo4_Increase = (-PID_L2_Back_Last_Servo.out);
}

void Control_R1_For_Servo(float R1_dis)				//用R1超声波的值控制1、2舵机
{
	PID_R1_For_Servo.error = PID_R1_For_Servo.desired - R1_dis;
	Pid_Update(&PID_R1_For_Servo, PID_R1_For_Servo.error);
	Servo1_Increase = PID_R1_For_Servo.out;
	Servo2_Increase = PID_R1_For_Servo.out;                                                //直走  右
}

void Control_R2_Back_Servo(float R2_dis)				//用R2超声波控制3、4舵机
{
	PID_R2_Back_Servo.error = PID_R2_Back_Servo.desired - R2_dis;
	Pid_Update(&PID_R2_Back_Servo, PID_R2_Back_Servo.error);  
	Servo3_Increase = PID_R2_Back_Servo.out;
	Servo4_Increase = PID_R2_Back_Servo.out;        
	
//	SpeedL1_Increase = -speed_r_back*PID_R2_Back_Servo.out;
//	SpeedR1_Increase = +speed_r_back*PID_R2_Back_Servo.out; 

//  Servo7_Increase = PID_R2_Back_Servo.out;
//	Servo8_Increase = PID_R2_Back_Servo.out;	//后退  右
}

void Control_R1_Back_Servo(float R1_dis)				//用R1超声波控制1、2舵机
{
	PID_R1_Back_Servo.error = PID_R1_Back_Servo.desired - R1_dis;
	Pid_Update(&PID_R1_Back_Servo, PID_R1_Back_Servo.error);
	Servo1_Increase = PID_R1_Back_Servo.out;
	Servo2_Increase = PID_R1_Back_Servo.out;   
	
//	SpeedL2_Increase = +speed_r_back*PID_R1_Back_Servo.out;
//	SpeedR2_Increase = -speed_r_back*PID_R1_Back_Servo.out; 

//	Servo5_Increase = PID_R1_Back_Servo.out;
//	Servo6_Increase = PID_R1_Back_Servo.out; //后退  右
}


void Control_F1_Left_Servo(float F1_dis)				//左行用F1超声波控制1、3舵机
{
	PID_F1_Left_Servo.error =PID_F1_Left_Servo.desired - F1_dis;
	Pid_Update(&PID_F1_Left_Servo, PID_F1_Left_Servo.error);
	Servo1_Increase = (-PID_F1_Left_Servo.out);	//-
	Servo3_Increase = (-PID_F1_Left_Servo.out);
//  SpeedL1_Increase = +speed_l*PID_F1_Left_Servo.error;
//	SpeedR1_Increase = -1.1*speed_l*PID_F1_Left_Servo.out; 	
//	SpeedL2_Increase = -speed_l*PID_F1_Left_Servo.out;
//	SpeedR2_Increase = +speed_l*PID_F1_Left_Servo.error; 
}

void Control_F2_Left_Servo(float F2_dis)				//左行用F2超声波控制2、4舵机
{
	PID_F2_Left_Servo.error =PID_F2_Left_Servo.desired - F2_dis;
	Pid_Update(&PID_F2_Left_Servo, PID_F2_Left_Servo.error);
	Servo2_Increase = (-PID_F2_Left_Servo.out);
	Servo4_Increase = (-PID_F2_Left_Servo.out);
//  SpeedL1_Increase = +0.8*speed_l*PID_F2_Left_Servo.out;
//	SpeedR1_Increase = -0.7*speed_l*PID_F2_Left_Servo.error;	
//	SpeedL2_Increase = -0.7*speed_l*PID_F2_Left_Servo.error;
//	SpeedR2_Increase = +0.7*speed_l*PID_F2_Left_Servo.out;
}


void Control_F1_Left_Last_Servo(float F1_dis)		//最后一个左行用F1超声波控制1、3舵机
{
	PID_F1_Left_Last_Servo.error =PID_F1_Left_Last_Servo.desired - F1_dis;
	Pid_Update(&PID_F1_Left_Last_Servo, PID_F1_Left_Last_Servo.error);
	Servo1_Increase = (-PID_F1_Left_Last_Servo.out);	//-
	Servo3_Increase = (-PID_F1_Left_Last_Servo.out);
//  SpeedL1_Increase = +speed_l*PID_F1_Left_Servo.error;
//	SpeedR1_Increase = -1.1*speed_l*PID_F1_Left_Servo.out; 	
//	SpeedL2_Increase = -speed_l*PID_F1_Left_Servo.out;
//	SpeedR2_Increase = +speed_l*PID_F1_Left_Servo.error; 
}

void Control_F2_Left_Last_Servo(float F2_dis)				//左行用F2超声波控制2、4舵机
{
	PID_F2_Left_Last_Servo.error =PID_F2_Left_Last_Servo.desired - F2_dis;
	Pid_Update(&PID_F2_Left_Last_Servo, PID_F2_Left_Last_Servo.error);
	Servo2_Increase = (-PID_F2_Left_Last_Servo.out);
	Servo4_Increase = (-PID_F2_Left_Last_Servo.out);
//  SpeedL1_Increase = +0.8*speed_l*PID_F2_Left_Servo.out;
//	SpeedR1_Increase = -0.7*speed_l*PID_F2_Left_Servo.error;	
//	SpeedL2_Increase = -0.7*speed_l*PID_F2_Left_Servo.error;
//	SpeedR2_Increase = +0.7*speed_l*PID_F2_Left_Servo.out;
}

void Control_F1_Right_Servo(float F1_dis)			//右行用F1超声波控制1、3舵机
{
	PID_F1_Right_Servo.error = PID_F1_Right_Servo.desired - F1_dis;
	Pid_Update(&PID_F1_Right_Servo, PID_F1_Right_Servo.error);
	Servo1_Increase = PID_F1_Right_Servo.out;
	Servo3_Increase = PID_F1_Right_Servo.out;
//  SpeedL1_Increase = -0.7*speed_r*PID_F1_Right_Servo.error;
//	SpeedR1_Increase = +speed_r*PID_F1_Right_Servo.out;
//	SpeedL2_Increase = +speed_r*PID_F1_Right_Servo.out;
//	SpeedR2_Increase = -0.7*speed_r*PID_F1_Right_Servo.error; 
}

void Control_F2_Right_Servo(float F2_dis)			//右行用F2超声波控制2、4舵机
{
	PID_F2_Right_Servo.error = PID_F2_Right_Servo.desired - F2_dis;
	Pid_Update(&PID_F2_Right_Servo, PID_F2_Right_Servo.error);
	Servo2_Increase = PID_F2_Right_Servo.out;
	Servo4_Increase = PID_F2_Right_Servo.out;
//  SpeedL1_Increase = -speed_r*PID_F2_Right_Servo.out;
//	SpeedR1_Increase = +speed_r*PID_F2_Right_Servo.error; 	
//	SpeedL2_Increase = +speed_r*PID_F2_Right_Servo.error;
//	SpeedR2_Increase = -speed_r*PID_F2_Right_Servo.out;
}

void Control_R1_Right_Servo(float D_dis,u8 step)			//用R1超声波控制舵机和电机
{
	PID_R1_Right_Servo.error = PID_R1_Right_Servo.desired - D_dis;
	Pid_Update(&PID_R1_Right_Servo, PID_R1_Right_Servo.error);
if((step == 1)||(step == 5)||(step == 9))
	{
		if(PID_R1_Right_Servo.error>0)
		{
			Servo2_Increase = -PID_R1_Right_Servo.out;
	    Servo4_Increase = -PID_R1_Right_Servo.out;
			Servo1_Increase = -0.7*PID_R1_Right_Servo.out;
	    Servo3_Increase = -0.7*PID_R1_Right_Servo.out;
      SpeedL1_Increase = -speed_r_l*PID_R1_Right_Servo.out;
	    SpeedR1_Increase = +0.7*speed_r_l*PID_R1_Right_Servo.out; 
	    SpeedL2_Increase = +0.7*speed_r_l*PID_R1_Right_Servo.out;
	    SpeedR2_Increase = -speed_r_l*PID_R1_Right_Servo.out; 				
		}      
    else
    {
		  Servo1_Increase = -0.7*PID_R1_Right_Servo.out;
	    Servo3_Increase = -0.7*PID_R1_Right_Servo.out;
			Servo2_Increase = -PID_R1_Right_Servo.out;
	    Servo4_Increase = -PID_R1_Right_Servo.out;
      SpeedL1_Increase = -speed_r_l*PID_R1_Right_Servo.out;
	    SpeedR1_Increase = +0.7*speed_r_l*PID_R1_Right_Servo.out; 
	    SpeedL2_Increase = +0.7*speed_r_l*PID_R1_Right_Servo.out;
	    SpeedR2_Increase = -speed_r_l*PID_R1_Right_Servo.out; 			
		}    	
  }
	else
	{
		if(PID_L1_Left_Servo.error<0)
		{
		  Servo1_Increase = -PID_R1_Right_Servo.out;
	    Servo3_Increase = -PID_R1_Right_Servo.out;
			Servo2_Increase = -0.7*PID_R1_Right_Servo.out;
	    Servo4_Increase = -0.7*PID_R1_Right_Servo.out;
      SpeedL1_Increase = -0.7*speed_r_l*PID_R1_Right_Servo.out;
	    SpeedR1_Increase = +speed_r_l*PID_R1_Right_Servo.out; 
	    SpeedL2_Increase = +speed_r_l*PID_R1_Right_Servo.out;
	    SpeedR2_Increase = -0.7*speed_r_l*PID_R1_Right_Servo.out; 						
		}      
    else
    {
		  Servo1_Increase = -PID_R1_Right_Servo.out;
	    Servo3_Increase = -PID_R1_Right_Servo.out;
			Servo2_Increase = -0.7*PID_R1_Right_Servo.out;
	    Servo4_Increase = -0.7*PID_R1_Right_Servo.out;
      SpeedL1_Increase = -0.7*speed_r_l*PID_R1_Right_Servo.out;
	    SpeedR1_Increase = +speed_r_l*PID_R1_Right_Servo.out; 
	    SpeedL2_Increase = +speed_r_l*PID_R1_Right_Servo.out;
	    SpeedR2_Increase = -0.7*speed_r_l*PID_R1_Right_Servo.out; 		
		}    	
	}   	
}

void Control_L1_Left_Servo(float D_dis,u8 step)			//用L1超声波控制舵机和电机
{
	PID_L1_Left_Servo.error = PID_L1_Left_Servo.desired - D_dis;
	Pid_Update(&PID_L1_Left_Servo, PID_L1_Left_Servo.error);
	if((step == 1)||(step == 5)||(step == 9))
	{
		if(PID_L1_Left_Servo.error<0)
		{
			Servo1_Increase = 0.7*PID_L1_Left_Servo.out;
	    Servo3_Increase = 0.7*PID_L1_Left_Servo.out;  
			Servo2_Increase = PID_L1_Left_Servo.out;
	    Servo4_Increase = PID_L1_Left_Servo.out;  
			SpeedL1_Increase = +speed_r_l*PID_L1_Left_Servo.out;
	    SpeedR1_Increase = -0.7*speed_r_l*PID_L1_Left_Servo.out; 
	    SpeedL2_Increase = -0.7*speed_r_l*PID_L1_Left_Servo.out;
	    SpeedR2_Increase = +speed_r_l*PID_L1_Left_Servo.out; 	
		}      
    else
    {
			Servo1_Increase = 0.7*PID_L1_Left_Servo.out;
	    Servo3_Increase = 0.7*PID_L1_Left_Servo.out;  
			Servo2_Increase = PID_L1_Left_Servo.out;
	    Servo4_Increase = PID_L1_Left_Servo.out; 	
			SpeedL1_Increase = +speed_r_l*PID_L1_Left_Servo.out;
	    SpeedR1_Increase = -0.7*speed_r_l*PID_L1_Left_Servo.out; 
	    SpeedL2_Increase = -0.7*speed_r_l*PID_L1_Left_Servo.out;
	    SpeedR2_Increase = +speed_r_l*PID_L1_Left_Servo.out;
		}    	
  }
	else
	{
		if(PID_L1_Left_Servo.error>0)
		{
			Servo1_Increase = PID_L1_Left_Servo.out;
	    Servo3_Increase = PID_L1_Left_Servo.out;  
			Servo2_Increase = 0.7*PID_L1_Left_Servo.out;
	    Servo4_Increase = 0.7*PID_L1_Left_Servo.out; 
      SpeedL1_Increase = +0.7*speed_r_l*PID_L1_Left_Servo.out;
	    SpeedR1_Increase = -speed_r_l*PID_L1_Left_Servo.out; 
	    SpeedL2_Increase = -speed_r_l*PID_L1_Left_Servo.out;
	    SpeedR2_Increase = +0.7*speed_r_l*PID_L1_Left_Servo.out;			
		}      
    else
    {
			Servo1_Increase = PID_L1_Left_Servo.out;
	    Servo3_Increase = PID_L1_Left_Servo.out;  
			Servo2_Increase = 0.7*PID_L1_Left_Servo.out;
	    Servo4_Increase = 0.7*PID_L1_Left_Servo.out; 
			SpeedL1_Increase = +0.7*speed_r_l*PID_L1_Left_Servo.out;
	    SpeedR1_Increase = -speed_r_l*PID_L1_Left_Servo.out; 
	    SpeedL2_Increase = -speed_r_l*PID_L1_Left_Servo.out;
	    SpeedR2_Increase = +0.7*speed_r_l*PID_L1_Left_Servo.out;
		}    	
	}
}

void Control_Back_L_Servo(float L1_dis, float L2_dis)	//左侧换垄时，用超声波值控制与垄道的距离固定
{
	//用L1的值控制前面两个超声波
	PID_Back_L_Servo.error = PID_Back_L_Servo.desired - L1_dis;
	Pid_Update(&PID_Back_L_Servo, PID_Back_L_Servo.error);
	Servo1_Increase = -PID_Back_L_Servo.out;
	Servo2_Increase = -PID_Back_L_Servo.out;
	//用L2的值控制后面两个超声波
	PID_Back_L_Servo.error = PID_Back_L_Servo.desired - L2_dis;
	Pid_Update(&PID_Back_L_Servo, PID_Back_L_Servo.error);
	Servo3_Increase = -PID_Back_L_Servo.out;
	Servo4_Increase = -PID_Back_L_Servo.out;	
}

void Control_Back_R_Servo(float R1_dis, float R2_dis)	//右侧换垄时，用超声波值控制与垄道的距离固定
{
	//用L1的值控制前面两个超声波
	PID_Back_R_Servo.error = PID_Back_R_Servo.desired - R1_dis;
	Pid_Update(&PID_Back_R_Servo, PID_Back_R_Servo.error);
	Servo1_Increase = PID_Back_R_Servo.out;
	Servo2_Increase = PID_Back_R_Servo.out;
	//用L2的值控制后面两个超声波
	PID_Back_R_Servo.error = PID_Back_R_Servo.desired - R2_dis;
	Pid_Update(&PID_Back_R_Servo, PID_Back_R_Servo.error);
	Servo3_Increase = PID_Back_R_Servo.out;
	Servo4_Increase = PID_Back_R_Servo.out;	
}


//超声波对电机的PID控制

void Control_F1_Left_Motor(float F2_dis)
{
	PID_F1_Left_Motor.error = PID_F1_Left_Motor.desired - F2_dis;
	Pid_Update(&PID_F1_Left_Motor, PID_F1_Left_Motor.error);      
  SpeedL1_Increase = +PID_F1_Left_Motor.out;
	SpeedR1_Increase = -PID_F1_Left_Motor.out; 
	SpeedL2_Increase = -PID_F1_Left_Motor.out;
	SpeedR2_Increase = +PID_F1_Left_Motor.out; 	//电机左行
}

void Control_F2_Right_Motor(float F2_dis)
{
	PID_F2_Right_Motor.error = PID_F2_Right_Motor.desired - F2_dis;
	Pid_Update(&PID_F2_Right_Motor, PID_F2_Right_Motor.error);        
  SpeedL1_Increase = -PID_F2_Right_Motor.out;
	SpeedR1_Increase = +PID_F2_Right_Motor.out; 
	SpeedL2_Increase = +PID_F2_Right_Motor.out;
	SpeedR2_Increase = -PID_F2_Right_Motor.out; 	//电机右行
}

void Control_L1_For_Motor(float L1_dis)
{
	PID_L1_For_Motor.error = PID_L1_For_Motor.desired - L1_dis;
	Pid_Update(&PID_L1_For_Motor, PID_L1_For_Motor.error);      
  SpeedL1_Increase = -PID_L1_For_Motor.out;
	SpeedR1_Increase = +PID_L1_For_Motor.out; 
	SpeedL2_Increase = +PID_L1_For_Motor.out;
	SpeedR2_Increase = -PID_L1_For_Motor.out; 	//电机左行
}

void Control_L2_For_Motor(float L2_dis)
{
	PID_L2_For_Motor.error = PID_L2_For_Motor.desired - L2_dis;
	Pid_Update(&PID_L2_For_Motor, PID_L2_For_Motor.error);        
  SpeedL1_Increase = -PID_L2_For_Motor.out;
	SpeedR1_Increase = +PID_L2_For_Motor.out; 
	SpeedL2_Increase = +PID_L2_For_Motor.out;
	SpeedR2_Increase = -PID_L2_For_Motor.out; 	//电机右行
}


//陀螺仪对舵机的PID控制

void Contrl_9250_Forward(float angle)					//用yaw角控制车直走，只控制两前轮
{
	PID_9250_Forward.error = angle - PID_9250_Forward.desired;
	Pid_Update(&PID_9250_Forward, PID_9250_Forward.error);
	Servo1_Increase = PID_9250_Forward.out;
	Servo2_Increase = PID_9250_Forward.out;
	Servo3_Increase = 0;
	Servo4_Increase = 0;
}

void Contrl_9250_Forwardl(float angle)					//用yaw角控制车直走
{
	PID_9250_Forward.error = PID_9250_Forward.desired - angle;
	Pid_Update(&PID_9250_Forward, PID_9250_Forward.error);
	Servo1_Increase = PID_9250_Forward.out;
	Servo2_Increase = PID_9250_Forward.out; 
  Servo3_Increase = PID_9250_Forward.out;
	Servo4_Increase = PID_9250_Forward.out;  
}

void Contrl_9250_Back(float angle)					//用yaw角控制两个舵机，后退
{
	PID_9250_Back.error = angle - PID_9250_Right.desired;
	Pid_Update(&PID_9250_Back, PID_9250_Back.error);
//	Servo1_Increase = PID_9250_Back.out;
//	Servo2_Increase = PID_9250_Back.out; 
	Servo3_Increase = PID_9250_Back.out;
	Servo4_Increase = PID_9250_Back.out;
}

void Contrl_9250_Back_last(float angle)					//用yaw角控制车3、4舵机
{
	PID_9250_Back_last.error = angle - PID_9250_Right.desired;
	Pid_Update(&PID_9250_Back_last, PID_9250_Back_last.error);
//	Servo1_Increase = PID_9250_Back_last.out;
//	Servo2_Increase = PID_9250_Back_last.out;
	Servo3_Increase = PID_9250_Back_last.out;
	Servo4_Increase = PID_9250_Back_last.out;
}


//陀螺仪对电机的PID控制

void Contrl_9250_Forward_motor(float angle)					//用yaw角控制车直走
{
	PID_9250_Forward_motor.error = PID_9250_Forward_motor.desired - angle;
	Pid_Update(&PID_9250_Forward_motor, PID_9250_Forward_motor.error);
  SpeedL1_Increase = -PID_9250_Forward_motor.out;
	SpeedR1_Increase = +PID_9250_Forward_motor.out; 
	SpeedL2_Increase = -PID_9250_Forward_motor.out;
	SpeedR2_Increase = +PID_9250_Forward_motor.out;
}

void Contrl_9250_Right(float angle)					//用yaw角控制车右行，四个差速电机
{
	PID_9250_Right.error = angle - PID_9250_Right.desired;
	Pid_Update(&PID_9250_Right, PID_9250_Right.error);
	SpeedL1_Increase_9250 = +speed_r_9250*PID_9250_Right.out;
	SpeedR1_Increase_9250 = -speed_r_9250*PID_9250_Right.out;
	SpeedL2_Increase_9250 = +speed_r_9250*PID_9250_Right.out;
	SpeedR2_Increase_9250 = -speed_r_9250*PID_9250_Right.out;
}
void Contrl_9250_Left(float angle)					//用yaw角控制车左行，四个差速电机
{
	PID_9250_Left.error = angle - PID_9250_Left.desired;
	Pid_Update(&PID_9250_Left, PID_9250_Left.error); 
	SpeedL1_Increase_9250 = +speed_l_9250*PID_9250_Left.out;
	SpeedR1_Increase_9250 = -speed_l_9250*PID_9250_Left.out; 
	SpeedL2_Increase_9250 = +speed_l_9250*PID_9250_Left.out;
	SpeedR2_Increase_9250 = -speed_l_9250*PID_9250_Left.out; 	
}

void Contrl_9250_Left_Last(float angle)
{
	PID_9250_Left_Last.error = angle - PID_9250_Left_Last.desired;
	Pid_Update(&PID_9250_Left_Last, PID_9250_Left_Last.error); 
	SpeedL1_Increase_9250 = +speed_l_9250*PID_9250_Left_Last.out;
	SpeedR1_Increase_9250 = -speed_l_9250*PID_9250_Left_Last.out; 
	SpeedL2_Increase_9250 = +speed_l_9250*PID_9250_Left_Last.out;
	SpeedR2_Increase_9250 = -speed_l_9250*PID_9250_Left_Last.out; 	
}	

void Contrl_9250_Back_motor(float angle)					//用yaw角控制四个电机差速后退
{
	PID_9250_Back_motor.error = PID_9250_Back_motor.desired - angle;
	Pid_Update(&PID_9250_Back_motor, PID_9250_Back_motor.error);
	SpeedL1_Increase = -PID_9250_Back_motor.out;
	SpeedR1_Increase = +PID_9250_Back_motor.out;
	SpeedL2_Increase = -PID_9250_Back_motor.out;
	SpeedR2_Increase = +PID_9250_Back_motor.out; 
}

void Contrl_9250_Right_slow(float angle)					//用yaw角控制车右行
{
	PID_9250_Right_slow.error = angle - PID_9250_Right.desired;
	Pid_Update(&PID_9250_Right_slow, PID_9250_Right_slow.error);
	SpeedL1_Increase_9250 = +speed_r_9250_s*PID_9250_Right_slow.out;
	SpeedR1_Increase_9250 = -speed_r_9250_s*PID_9250_Right_slow.out; 
	SpeedL2_Increase_9250 = +speed_r_9250_s*PID_9250_Right_slow.out;
	SpeedR2_Increase_9250 = -speed_r_9250_s*PID_9250_Right_slow.out; 
}
void Contrl_9250_Left_slow(float angle)					//用yaw角控制车左行
{
	PID_9250_Left_slow.error = PID_9250_Left_slow.desired - angle;
	Pid_Update(&PID_9250_Left_slow, PID_9250_Left_slow.error); 
	SpeedL1_Increase_9250 = +1.1*speed_l_9250_s*PID_9250_Left_slow.out;
	SpeedR1_Increase_9250 = -1.1*speed_l_9250_s*PID_9250_Left_slow.out; 
	SpeedL2_Increase_9250 = +speed_l_9250_s*PID_9250_Left_slow.out;
	SpeedR2_Increase_9250 = -speed_l_9250_s*PID_9250_Left_slow.out; 	
}

void Contrl_9250_Back_last_motor(float angle)					//出垄用yaw角控制四个电机
{
	PID_9250_Back_last_motor.error = PID_9250_Right.desired - angle;
	Pid_Update(&PID_9250_Back_last_motor, PID_9250_Back_last_motor.error);
	SpeedL1_Increase = -PID_9250_Back_last_motor.out;
	SpeedR1_Increase = +PID_9250_Back_last_motor.out;
	SpeedL2_Increase = -PID_9250_Back_last_motor.out;
	SpeedR2_Increase = +PID_9250_Back_last_motor.out; 
}


//混合控制

void Control_R1_Right_Servo_new(float D_dis,u8 step)			//用前面第二个超声波值控制右边两个舵机
{
	PID_R1_Right_Servo.error = PID_R1_Right_Servo.desired - D_dis;
	Pid_Update(&PID_R1_Right_Servo, PID_R1_Right_Servo.error);
if((step == 1)||(step == 5)||(step == 9))
	{
		if(PID_R1_Right_Servo.error>0)
		{
			Servo2_Increase = -PID_R1_Right_Servo.out;
	    Servo4_Increase = -PID_R1_Right_Servo.out;
			Servo1_Increase = -0.9*PID_R1_Right_Servo.out;
	    Servo3_Increase = -0.9*PID_R1_Right_Servo.out;
      SpeedL1_Increase = +speed_r_r*PID_R1_Right_Servo.out;
	    SpeedR1_Increase = -speed_r_r*0.9*PID_R1_Right_Servo.out; 
	    SpeedL2_Increase = -speed_r_r*0.9*PID_R1_Right_Servo.out;
	    SpeedR2_Increase = +speed_r_r*PID_R1_Right_Servo.out; 				
		}      
    else
    {
		  Servo1_Increase = -0.9*PID_R1_Right_Servo.out;
	    Servo3_Increase = -0.9*PID_R1_Right_Servo.out;
			Servo2_Increase = -PID_R1_Right_Servo.out;
	    Servo4_Increase = -PID_R1_Right_Servo.out;
      SpeedL1_Increase = +speed_r_r*PID_R1_Right_Servo.out;
	    SpeedR1_Increase = -speed_r_r*0.9*PID_R1_Right_Servo.out; 
	    SpeedL2_Increase = -speed_r_r*0.9*PID_R1_Right_Servo.out;
	    SpeedR2_Increase = +speed_r_r*PID_R1_Right_Servo.out; 			
		}    	
  }
	else
	{
		if(PID_R1_Right_Servo.error<0)
		{
		  Servo1_Increase = -PID_R1_Right_Servo.out;
	    Servo3_Increase = -PID_R1_Right_Servo.out;
			Servo2_Increase = -0.9*PID_R1_Right_Servo.out;
	    Servo4_Increase = -0.9*PID_R1_Right_Servo.out;
      SpeedL1_Increase = +speed_r_j*0.9*PID_R1_Right_Servo.out;
	    SpeedR1_Increase = -speed_r_j*PID_R1_Right_Servo.out; 
	    SpeedL2_Increase = -speed_r_j*PID_R1_Right_Servo.out;
	    SpeedR2_Increase = +speed_r_j*0.9*PID_R1_Right_Servo.out; 						
		}      
    else
    {
		  Servo1_Increase = -PID_R1_Right_Servo.out;
	    Servo3_Increase = -PID_R1_Right_Servo.out;
			Servo2_Increase = -0.9*PID_R1_Right_Servo.out;
	    Servo4_Increase = -0.9*PID_R1_Right_Servo.out;
      SpeedL1_Increase = +speed_r_j*0.9*PID_R1_Right_Servo.out;
	    SpeedR1_Increase = -speed_r_j*PID_R1_Right_Servo.out; 
	    SpeedL2_Increase = -speed_r_j*PID_R1_Right_Servo.out;
	    SpeedR2_Increase = +speed_r_j*0.9*PID_R1_Right_Servo.out; 		
		}    	
	}   	
}

void Control_L1_Left_Servo_new(float D_dis,u8 step)			//用前面第二个超声波值控制右边两个舵机
{
	PID_L1_Left_Servo.error = PID_L1_Left_Servo.desired - D_dis;
	Pid_Update(&PID_L1_Left_Servo, PID_L1_Left_Servo.error);
	if((step == 1)||(step == 5)||(step == 9))
	{
		if(PID_L1_Left_Servo.error<0)
		{
			Servo1_Increase = 0.9*PID_L1_Left_Servo.out;
	    Servo3_Increase = 0.9*PID_L1_Left_Servo.out;  
			Servo2_Increase = PID_L1_Left_Servo.out;
	    Servo4_Increase = PID_L1_Left_Servo.out;  
			SpeedL1_Increase = -speed_l_j*PID_L1_Left_Servo.out;
	    SpeedR1_Increase = +speed_l_j*0.9*PID_L1_Left_Servo.out; 
	    SpeedL2_Increase = +speed_l_j*0.9*PID_L1_Left_Servo.out;
	    SpeedR2_Increase = -speed_l_j*PID_L1_Left_Servo.out; 	
		}      
    else
    {
			Servo1_Increase = 0.9*PID_L1_Left_Servo.out;
	    Servo3_Increase = 0.9*PID_L1_Left_Servo.out;  
			Servo2_Increase = PID_L1_Left_Servo.out;
	    Servo4_Increase = PID_L1_Left_Servo.out; 	
			SpeedL1_Increase = -speed_l_j*PID_L1_Left_Servo.out;
	    SpeedR1_Increase = +speed_l_j*0.9*PID_L1_Left_Servo.out; 
	    SpeedL2_Increase = +speed_l_j*0.9*PID_L1_Left_Servo.out;
	    SpeedR2_Increase = -speed_l_j*PID_L1_Left_Servo.out;
		}    	
  }
	else
	{
		if(PID_L1_Left_Servo.error>0)
		{
			Servo1_Increase = PID_L1_Left_Servo.out;
	    Servo3_Increase = PID_L1_Left_Servo.out;  
			Servo2_Increase = 0.9*PID_L1_Left_Servo.out;
	    Servo4_Increase = 0.9*PID_L1_Left_Servo.out; 
      SpeedL1_Increase = -speed_l*0.9*PID_L1_Left_Servo.out;
	    SpeedR1_Increase = +speed_l*PID_L1_Left_Servo.out; 
	    SpeedL2_Increase = +speed_l*PID_L1_Left_Servo.out;
	    SpeedR2_Increase = -speed_l*0.9*PID_L1_Left_Servo.out;			
		}      
    else
    {
			Servo1_Increase = PID_L1_Left_Servo.out;
	    Servo3_Increase = PID_L1_Left_Servo.out;  
			Servo2_Increase = 0.9*PID_L1_Left_Servo.out;
	    Servo4_Increase = 0.9*PID_L1_Left_Servo.out; 
			SpeedL1_Increase = -speed_l*0.9*PID_L1_Left_Servo.out;
	    SpeedR1_Increase = +speed_l*PID_L1_Left_Servo.out; 
	    SpeedL2_Increase = +speed_l*PID_L1_Left_Servo.out;
	    SpeedR2_Increase = -speed_l*0.9*PID_L1_Left_Servo.out;
		}    	
	}
}