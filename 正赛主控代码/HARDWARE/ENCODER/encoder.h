#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f4xx.h"

typedef struct
{
	float cntA;
	float cntB;
	float	speedA;
	float speedB;
	float disA;
	float disB;
	
}EncoderStruct;

//编码器滤波结构体
typedef struct
{
	unsigned char 	Complete;
	unsigned char   Count;
	float   				Cache;	        //计算中间缓存值
	float 		 			Last_Data;	    //上一次捕获值
	float  					Data_His[10];		//数据历史值
}EncoderFilter_Struct;

extern EncoderStruct Encoders;

void TIM3_EncoderB_Init(u16 arr, u16 psc);
void TIM5_EncoderA_Init(u16 arr, u16 psc);

void Read_Encoder_Cnt(void);
void Solve_EncoData(float time);
void Encoder_Filter(EncoderFilter_Struct *filterStruct,float newDatas,unsigned char FilterNum,float *Out);


#endif

