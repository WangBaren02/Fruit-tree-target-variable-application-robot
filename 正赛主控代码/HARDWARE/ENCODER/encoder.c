#include "encoder.h"
#include "oled.h"
#include "sys.h"
#include "led.h"

//cntA;cntB;speedA;speedB;disA; disB;
EncoderStruct Encoders={0,0,0,0,0,0};

//滤波的结构体
EncoderFilter_Struct EncoAspeed_Filter = {0,0,0,0,{0}};
EncoderFilter_Struct EncoBspeed_Filter = {0,0,0,0,{0}};
/*
********************************************************************************************************************
*                  void TIM3_EncoderB_Init(u16 arr,u16 psc)
*
*Description : 定时器3的输入捕获配置
*Arguments   : 溢出时间为：（arr+1）*（psc+1）/84M
*Returns     : none
*Notes       : arr:自动重装值,psc:时钟预分频数
********************************************************************************************************************
*/
void TIM3_EncoderB_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;
	TIM_TimeBaseStructure.TIM_Period = arr - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);	
}
	
/*
********************************************************************************************************************
*              void TIM5_EncoderA_Init(u16 arr,u16 psc)
*
*Description : 定时器5的输入捕获配置
*Arguments   : 溢出时间为：（arr+1）*（psc+1）/84M
*Returns     : none
*Notes       : arr:自动重装值,psc:时钟预分频数
********************************************************************************************************************
*/
void TIM5_EncoderA_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;
	TIM_TimeBaseStructure.TIM_Period = arr - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);//使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM5, 0);
	TIM_Cmd(TIM5, ENABLE);
}	

/*
********************************************************************************************************************
*                  void Read_Encoder_Cnt(void)
*
*Description : 左右编码器测速函数
*Arguments   : none
*Returns     : none
*Notes       : none
********************************************************************************************************************
*/
void Read_Encoder_Cnt(void)
{
	Encoders.cntA = (short)TIM5->CNT;
	Encoders.cntB = (short)TIM3->CNT;
	Encoders.disA += (Encoders.cntA*3.14f*10.0f/3168.0f);
	Encoders.disB += (Encoders.cntB*3.14f*10.0f/3168.0f);
	TIM5->CNT = 0;
	TIM3->CNT = 0;
}
/*
********************************************************************************************************************
*                  void Solve_EncoData(void)
*
*Description : 编码器解算距离、速度函数
*Arguments   : none
*Returns     : none
*Notes       : 3168：电机正转一圈，CNT计数值  3.14：Pi   10.0：轮子直径/cm  time：解算频率/s
********************************************************************************************************************
*/
void Solve_EncoData(float time)
{
	float speedA=0;
	float speedB=0;
	Read_Encoder_Cnt();
	Encoders.disA += (Encoders.cntA/3168.0f)*3.14f*10.0f;						//单位厘米 cm
	Encoders.disB += (Encoders.cntB/3168.0f)*3.14f*10.0f;						
	speedA =- Encoders.disA/time;																		//单位厘米 cm/s
	speedB =- Encoders.disB/time;
	Encoder_Filter(&EncoAspeed_Filter,speedA,5,&Encoders.speedA);
	Encoder_Filter(&EncoBspeed_Filter,speedB,5,&Encoders.speedB);
//	OLED_Show3FNum(0,0,-Encoders.disA,5,2,12);
//	OLED_Show3FNum(60,0,Encoders.disB,5,2,12);
//	OLED_Show3FNum(0,16,Encoders.speedA,5,2,12);
//	OLED_Show3FNum(60,16,-Encoders.speedB,5,2,12);
//	OLED_Refresh_Gram();
}
/******************************************************************************************************************
*             void Encoder_Filter(Filter_Struct *filterStruct,float newDatas,unsigned char FilterNum,float *Out)
*
*Description : 增强型均值滤波  去掉一个最大值和最小值
*Returns     : 当前的数据
*Notes       : 输入的数据在程序中完成缓存
*              
							*filterStruct		选择滤波的结构体
							newDatas				新数据
							FilterNum				滤波样本总数
							*Out						输出值
*******************************************************************************************************************
*/
void Encoder_Filter(EncoderFilter_Struct *filterStruct,float newDatas,unsigned char FilterNum,float *Out)
{
	unsigned int i = 0;
	float FilterSum  = 0;
	static float max,min;
	
	filterStruct->Data_His[filterStruct->Count++] = newDatas;
	
	if( filterStruct->Complete )                //已经采集完最初的AdcFilterNum个数据
	{
		max = min = filterStruct->Data_His[0];    //初始化
	//找到最大最小值
		for(i =1;i < FilterNum; i ++)
		{
			if(filterStruct->Data_His[i] > max)
			{
				max = filterStruct->Data_His[i];
			}
			
			if(filterStruct->Data_His[i] < min)
			{
				min = filterStruct->Data_His[i];
			}
		}
		
		for(i = 0; i < FilterNum; i++)
		{
			FilterSum += filterStruct->Data_His[i];
		}
		FilterSum -= max + min;
		filterStruct->Cache = FilterSum / (FilterNum-2);
	}
	
	if(filterStruct->Complete  == 1)
	{
		if(filterStruct->Cache < 5000)
		{
			*Out = filterStruct->Cache;
			filterStruct->Last_Data 	 = filterStruct->Cache;
		}
		else
			*Out = filterStruct->Last_Data;
	}
	if(filterStruct->Count == FilterNum)
	{
		filterStruct->Complete   = 1;
		filterStruct->Count = 0;
	}
}

