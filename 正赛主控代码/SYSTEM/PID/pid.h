#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

typedef struct
{
	float desired;			//< set point
	float input;				//< input
	float error;        //< error
	float preError;    //< previous error
	float integ;        //< integral
	float integ_sep;		//< integral separation
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	float dt;           //< delta-time dt
	float out;					//< out
} PidObject;


void PID_Init(void);
void Pid_Update(PidObject* pid, const float error);
//电机A和电机B的转速闭环PID
extern PidObject PID_PWMA;				//电机A转速闭环
extern PidObject PID_PWMB;				//电机B转速闭环

//直走时的9250闭环
extern PidObject PID_9250_Forward;
extern PidObject PID_9250_Forward_motor;
//向右走的9250闭环
extern PidObject PID_9250_Right;
//向左走的9250闭环
extern PidObject PID_9250_Left;
extern PidObject PID_9250_Left_Last;

extern PidObject PID_9250_Right_slow;
//向左走的9250闭环
extern PidObject PID_9250_Left_slow;
//向后走的9250闭环
extern PidObject PID_9250_Back;
extern PidObject PID_9250_Back_motor;

extern PidObject PID_9250_Back_last;
extern PidObject PID_9250_Back_last_motor;
//L1 L2超声波控制舵机左前左后
extern PidObject PID_L1_For_Servo;
extern PidObject PID_L2_For_Servo;

extern PidObject PID_L1_Back_Servo;
extern PidObject PID_L2_Back_Servo;

extern PidObject PID_L1_Back_Last_Servo;
extern PidObject PID_L2_Back_Last_Servo;
//R1 R2超声波控制舵机右前右后
extern PidObject PID_R1_For_Servo;
extern PidObject PID_R2_Back_Servo;
extern PidObject PID_R1_Back_Servo;
//F1 F2超声波控制舵机左行右行
extern PidObject PID_F1_Left_Servo;
extern PidObject PID_F2_Left_Servo;

extern PidObject PID_F1_Left_Last_Servo;
extern PidObject PID_F2_Left_Last_Servo;

extern PidObject PID_F1_Right_Servo;
extern PidObject PID_F2_Right_Servo;
//换垄时，用舵机保持和垄的固定距离
extern PidObject PID_Back_L_Servo;
extern PidObject PID_Back_R_Servo;
//F1 F2超声波控制电机左行右行
extern PidObject PID_F1_Left_Motor;
extern PidObject PID_F2_Right_Motor;

extern PidObject PID_L1_For_Motor;
extern PidObject PID_L2_For_Motor;

#define F1 60	//60
#define F2 60

#define L1_F 75	//靠左前行，即step= 0时，80
#define L2_F 75	//靠左前行，即step= 0时，80
#define L1_L 52.5	//靠左后行，即step=12时，55
#define L2_L 52.5	//靠左后行，即step=12时，55
#define L1 75
#define L2 75
#define R1 75//75
#define R2 75

float Control_PWMA(float cnt_A);
float Control_PWMB(float cnt_B);
//9250
void Contrl_9250_Forward(float angle);
void Contrl_9250_Forwardl(float angle);
void Contrl_9250_Forward_motor(float angle);
void Contrl_9250_Left(float angle);
void Contrl_9250_Left_Last(float angle);
void Contrl_9250_Right(float angle);
void Contrl_9250_Left_slow(float angle);
void Contrl_9250_Right_slow(float angle);
void Contrl_9250_Back(float angle);
void Contrl_9250_Back_motor(float angle);
void Contrl_9250_Back_last(float angle);
void Contrl_9250_Back_last_motor(float angle);	
//超声波
void Control_L1_For_Servo(float L1_dis);
void Control_L2_For_Servo(float L2_dis);
void Control_L1_Back_Servo(float L1_dis);
void Control_L2_Back_Servo(float L2_dis);
void Control_L1_Backlast_Servo(float L1_dis);
void Control_L2_Backlast_Servo(float L2_dis);
void Control_R1_For_Servo(float R1_dis);
void Control_R2_Back_Servo(float R2_dis);
void Control_R1_Back_Servo(float R1_dis);
void Control_F1_Left_Servo(float F1_dis);
void Control_F2_Right_Servo(float F2_dis);
void Control_F2_Left_Servo(float F2_dis);
void Control_F1_Right_Servo(float F1_dis);
void Control_F1_Left_Last_Servo(float F1_dis);
void Control_F2_Left_Last_Servo(float F2_dis);

void Control_L1_Left_Servo(float D_dis,u8 step);
void Control_R1_Right_Servo(float D_dis,u8 step);

void Control_L1_Left_Servo_new(float D_dis,u8 step);
void Control_R1_Right_Servo_new(float D_dis,u8 step);

void Control_F1_Left_Motor(float F1_dis);
void Control_F2_Right_Motor(float F2_dis);

void Control_L1_For_Motor(float L1_dis);
void Control_L2_For_Motor(float L2_dis);

void Control_Back_L_Servo(float L1_dis, float L2_dis);	//左侧换垄时，用超声波值控制与垄道的距离固定
void Control_Back_R_Servo(float R1_dis, float R2_dis);	//右侧换垄时，用超声波值控制与垄道的距离固定
#endif
