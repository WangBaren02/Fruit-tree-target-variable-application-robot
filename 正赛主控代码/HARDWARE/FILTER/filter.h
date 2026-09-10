/*
******************************************************************************************************************
*Filename      :filter.h
*Programmer(s) :chu
*Description   :filter function
******************************************************************************************************************
*/

#ifndef __FILTER_H
#define __FILTER_H
#include "stm32f4xx.h"
/*
******************************************************************************************************************
*                                            CONSTANTS & MACROS
*Description : 变量及宏定义声明
*Notes       : none
******************************************************************************************************************
*/


//#define MedNumSum  5       //中值滤波的数据总数
//#define MedNumGet  2       //最后取哪一个数据输出
//#define MeanSum   20       
//#define EnhanceMeanSum 10  // 增强型均值滤波算法
//#define FilterNum  5        //滑动加权滤波缓存数组大小


typedef struct
{
	float data[25];
	unsigned char storeNum;
}FilterStruct;

typedef struct
{
	float data[100];
	unsigned char storeNum;
}FilterStruct_Float;

typedef struct
{
	unsigned char 			Complete;
	unsigned char       Count;
	short int  					Cache;	        //计算中间缓存值
	short int 					Last_Data;	    //上一次捕获值
	short int  					Data_His[15];		//数据历史值
}Filter_Struct;


//typedef struct{
//	float gain;
//	float IIR_A[3];   		
//	float IIR_B[3];   
//	float x_temp[3];  
//	float y_temp[3];
//}IIR2_para;


/*
******************************************************************************************************************
*                                            FUNCTION PROTOTYPES
******************************************************************************************************************
*/
extern FilterStruct Ultr_F1;
extern FilterStruct Ultr_F2;
extern FilterStruct Ultr_B1;
extern FilterStruct Ultr_B2;
extern FilterStruct Ultr_FR;
extern FilterStruct Ultr_FL;

extern float medFilter(FilterStruct *filterStruct,short int newDatas,unsigned char MedNumSum,unsigned char MedNumGet);
extern float meanFilter(FilterStruct *filterStruct,float newDatas,unsigned char MeanSum);
extern float meanFilter_Float(FilterStruct_Float *filterStruct,float newDatas,unsigned char MeanSum);
extern float enhanceMeanFilter(FilterStruct *filterStruct,short int newDatas,unsigned char EnhanceMeanSum);
extern float MeanFilterLimit(FilterStruct *filterStruct,short int newDatas,unsigned char EnhanceMeanSum);
void Average_Filter(Filter_Struct *filterStruct,short int newDatas,unsigned char FilterNum,u32 *Out);
void Enaverage_Filter(Filter_Struct *filterStruct,short int newDatas,unsigned char FilterNum,short int *Out);
float KalmanFilter(float inData);
void EdgeFilter(float ultra_data,unsigned int num);

//void gyro_IIRFilter(IIR2_para *gyro,float input);

#endif


