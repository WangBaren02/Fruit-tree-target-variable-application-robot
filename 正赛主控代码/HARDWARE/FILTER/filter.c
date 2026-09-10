/*
******************************************************************************************************************
*Filename      : filter.c
*Programmer(s) : chu
*Description   : 滤波
******************************************************************************************************************
*/
#include "filter.h"
#include "ultrasonic.h"
#include "control.h"
/*
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
！！！！！    滤波结构体定义事例      ！！！！！

FilterStruct ????? = {{0},0};
Filter_Struct ????? = {0,0,0,0,{0}};

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/
//FilterStruct laser= {{0},0};
extern FilterStruct Ultr_F1;
extern FilterStruct Ultr_F2;
extern FilterStruct Ultr_L1;
extern FilterStruct Ultr_L2;
extern FilterStruct Ultr_R1;
extern FilterStruct Ultr_R2;
//FilterStruct_Float mpu_6050= {{0},0};
/******************************************************************************************************************
*      medFilter(FilterStruct *filterStruct,short int newDatas,unsigned char MedNumSum,unsigned char MedNumGet)
*
*Description : 连续中值滤波算法
*Returns     : 当前的数据
*Notes       : 输入的数据在程序中完成缓存
*              
							*filterStruct		选择滤波的结构体
							newDatas				新数据
							MedNumSum				滤波样本总数
							MedNumGet				选择第几个数据输出
*******************************************************************************************************************
*/
float medFilter(FilterStruct *filterStruct,short int newDatas,unsigned char MedNumSum,unsigned char MedNumGet)
{
	unsigned char i,j;// 循环变量
	short int dataFilter[MedNumSum];
	short int dataTemp = 0;
	//存取数据
	filterStruct->data[filterStruct->storeNum++] = newDatas;
	if(filterStruct->storeNum >= MedNumSum)    
	{
		filterStruct->storeNum = 0;
	}
	
	for(i = 0;i < MedNumSum;i ++)   
	{
		dataFilter[i] = filterStruct->data[i];    //数据复制
	}
	
	// 用冒泡法对数组进行排序，从小到大排序
	for (j = 0; j < MedNumSum - 1; j ++)
	{
		for (i = 0; i < MedNumSum - j - 1; i ++)
		{
			if (dataFilter[i] > dataFilter[i + 1])
			{
				// 互换
				dataTemp = dataFilter[i];
				dataFilter[i] = dataFilter[i + 1];
				dataFilter[i + 1] = dataTemp;
			}
		}
	}
	return dataFilter[MedNumGet];
}


/******************************************************************************************************************
*        			  meanFilter(FilterStruct *filterStruct,short int newDatas,unsigned char MeanSum)
*
*Description : 滑动均值滤波算法
*Returns     : 当前的数据
*Notes       : 输入的数据在程序中完成缓存
*              
							*filterStruct		选择滤波的结构体
							newDatas				新数据
							MeanSum					滤波样本总数
*******************************************************************************************************************
*/
float meanFilter(FilterStruct *filterStruct,float newDatas,unsigned char MeanSum)
{
	float dataSum = 0;
	unsigned char i = 0;
	filterStruct->data[filterStruct->storeNum++] = newDatas;  
	if(filterStruct->storeNum >= MeanSum)
	{
		filterStruct->storeNum = 0;
	}
	
	for(i = 0;i < MeanSum;i++)
	{
		dataSum += filterStruct->data[i];
	}
	return dataSum/MeanSum;
}

/******************************************************************************************************************
*        			  meanFilter_Float(FilterStruct *filterStruct,short int newDatas,unsigned char MeanSum)
*
*Description : 滑动均值滤波算法(输入为浮点型)
*Returns     : 当前的数据
*Notes       : 输入的数据在程序中完成缓存
*              
							*filterStruct		选择滤波的结构体
							newDatas				新数据
							MeanSum					滤波样本总数
*******************************************************************************************************************
*/
float meanFilter_Float(FilterStruct_Float *filterStruct,float newDatas,unsigned char MeanSum)
{
	float dataSum = 0;
	unsigned char i = 0;
	filterStruct->data[filterStruct->storeNum++] = newDatas;  
	if(filterStruct->storeNum >= MeanSum)
	{
		filterStruct->storeNum = 0;
	}
	
	for(i = 0;i < MeanSum;i++)
	{
		dataSum += filterStruct->data[i];
	}
	return dataSum/MeanSum;
}

/******************************************************************************************************************
*          enhanceMeanFilter(FilterStruct *filterStruct,short int newDatas,unsigned char EnhanceMeanSum)
*
*Description : 增强型滑动均值滤波算法    去掉一个最大值和最小值
*Returns     : 当前的数据
*Notes       : 输入的数据在程序中完成缓存
*              
							*filterStruct		选择滤波的结构体
							newDatas				新数据
							EnhanceMeanSum	滤波样本总数
*******************************************************************************************************************
*/
float enhanceMeanFilter(FilterStruct *filterStruct,short int newDatas,unsigned char EnhanceMeanSum)
{
	float dataSum = 0;
	unsigned char i = 0;
	static short int max,min;
	filterStruct->data[filterStruct->storeNum++] = newDatas;  
	if(filterStruct->storeNum >= EnhanceMeanSum)
	{
		filterStruct->storeNum = 0;
	}
	
	max = min = filterStruct->data[0];    //初始化
	//找到最大最小值
	for(i =1;i < EnhanceMeanSum; i ++)
	{
		if(filterStruct->data[i] > max)
		{
			max = filterStruct->data[i];
		}
		
		if(filterStruct->data[i] < min)
		{
			min = filterStruct->data[i];
		}
	}
	
	for(i = 0;i < EnhanceMeanSum;i ++)
	{
		dataSum += filterStruct->data[i];
	}
	dataSum -= (max + min); 
	return dataSum/(EnhanceMeanSum-2);
}
/******************************************************************************************************************
*          enhanceMeanFilter(FilterStruct *filterStruct,short int newDatas,unsigned char EnhanceMeanSum)
*
*Description : 增强型滑动均值滤波算法    去掉一个最大值和最小值
*Returns     : 当前的数据
*Notes       : 输入的数据在程序中完成缓存
*              
							*filterStruct		选择滤波的结构体
							newDatas				新数据
							EnhanceMeanSum	滤波样本总数
*******************************************************************************************************************
*/
float MeanFilterLimit(FilterStruct *filterStruct,short int newDatas,unsigned char EnhanceMeanSum)
{
	float dataSum = 0;
	unsigned char i = 0;
	static short int max1,max2;
	filterStruct->data[filterStruct->storeNum++] = newDatas;
	if(filterStruct->storeNum >= EnhanceMeanSum)
	{
		filterStruct->storeNum = 0;
	}
	
	max1 = max2 = filterStruct->data[0];    //初始化
	//找到最大次大值
	for(i =1;i < EnhanceMeanSum; i ++)
	{
		if(filterStruct->data[i] > max1)
		{
			max1 = filterStruct->data[i];
		}
	}
	for(i =1;i < EnhanceMeanSum; i ++)
	{
		if(filterStruct->data[i] > max2 && filterStruct->data[i] < max1)
		{
			max2 = filterStruct->data[i];
		}
	}
	for(i = 0;i < EnhanceMeanSum;i ++)
	{
		dataSum += filterStruct->data[i];
	}
	dataSum-=(max1+max2);
	return dataSum/(EnhanceMeanSum-2);
}

/******************************************************************************************************************
*             Average_Filter(Filter_Struct *filterStruct,short int newDatas,unsigned char FilterNum)
*
*Description : 均值滤波
*Returns     : 当前的数据
*Notes       : 输入的数据在程序中完成缓存
*              
							*filterStruct		选择滤波的结构体
							newDatas				新数据
							FilterNum				滤波样本总数
							*Out						输出值
*******************************************************************************************************************
*/
void Average_Filter(Filter_Struct *filterStruct,short int newDatas,unsigned char FilterNum,u32 *Out)
{
	unsigned int i = 0;
	float FilterSum  = 0;
	
	filterStruct->Data_His[filterStruct->Count++] = newDatas;
	
	if( filterStruct->Complete )                //已经采集完最初的AdcFilterNum个数据
	{
		for(i = 0; i < FilterNum; i++)
		{
			FilterSum += filterStruct->Data_His[i];
		}
		filterStruct->Cache = FilterSum / (FilterNum);
	}
	
	if(filterStruct->Complete  == 1)
	{
		if(filterStruct->Cache < 5000)
		{
			*Out 	 = filterStruct->Cache;
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


/******************************************************************************************************************
*             Enaverage_Filter(Filter_Struct *filterStruct,short int newDatas,unsigned char FilterNum)
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
void Enaverage_Filter(Filter_Struct *filterStruct,short int newDatas,unsigned char FilterNum,short int *Out)
{
	unsigned int i = 0;
	float FilterSum  = 0;
	static unsigned short int max,min;
	
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
/**
  ******************************************************************************
  * @brief  卡尔曼滤波器 函数 szj
  * @param  inData - 输入值
  * @return 滤波后的值
  * @note   r值固定，q值越大，代表越信任测量值，q值无穷大，代表只用测量值。
  *                  q值越小，代表越信任模型预测值，q值为0，代表只用模型预测值。
  *         q:过程噪声，q增大，动态响应变快，收敛稳定性变坏；反之。控制误差
  *         r:测量噪声，r增大，动态响应变慢，收敛稳定性变好；反之。控制响应速度
  ******************************************************************************
  */
float KalmanFilter(float inData)
{
	static float kalman = 0; //上次卡尔曼值(估计出的最优值)
	static float p = 10;
	float q = 0.003; //q:过程噪声
	float r = 0.01; //r:测量噪声
	float kg = 0; //kg:卡尔曼增益

	p += q;
	kg = p / (p + r); //计算卡尔曼增益
	kalman = kalman + (kg * (inData - kalman)); //计算本次滤波估计值
	p = (1 - kg) * p; //更新测量方差

	return kalman; //返回估计值
}
/*
  ******************************************************************************
	分步维持滤波
	step：当前步数		ultra_data：当前超声波模块的返回值（已完成滑动均值滤波后）		num：超声波编号（滤具体哪个超声波）
	
		         F1头F2
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

							1头2
					3----------5
					4----------6
							 尾
							 
	第一组：num=1，F1和F2		1，5，9，3，7，10
	第二组：num=2，R1和R2		2，6
	第三组：num=3，L1和L2		0||4，8
	以下四个为超声波上一个的值（经滑动均值滤波和分布维持滤波后）
	float ultra_F1DISLast=0;
	float ultra_F2DISLast=0;
	float ultra_L1DISLast=0;
	float ultra_L2DISLast=0;
	float ultra_R1DISLast=0;
	float ultra_R2DISLast=0;
  ******************************************************************************
*/
void EdgeFilter(float ultra_data,unsigned int num)
{
	if(num==1)
	{
		if(abs(ultra_F1.DIS-ultra_F1DISLast)>80)
		{ultra_F1.DIS=ultra_F1DISLast;return;}
		else {ultra_F1DISLast=ultra_F1.DIS;return;}
	}
	
	if(num==2)
	{
		if(abs(ultra_F2.DIS-ultra_F2DISLast)>80)
		{ultra_F2.DIS=ultra_F2DISLast;return;}
		else {ultra_F2DISLast=ultra_F2.DIS;return;}
	}
	
	if(num==3)
	{
		if(abs(ultra_L1.DIS-ultra_L1DISLast)>80)
		{ultra_L1.DIS=ultra_L1DISLast;return;}
		else {ultra_L1DISLast=ultra_L1.DIS;return;}
	}
	
	if(num==4)
	{
		if(abs(ultra_L2.DIS-ultra_L2DISLast)>80)
		{ultra_L2.DIS=ultra_L2DISLast;return;}
		else {ultra_L2DISLast=ultra_L2.DIS;return;}
	}
	
	if(num==5)
	{
		if(abs(ultra_R1.DIS-ultra_R1DISLast)>80)
		{ultra_R1.DIS=ultra_R1DISLast;return;}
		else {ultra_R1DISLast=ultra_R1.DIS;return;}
	}
	
	if(num==6)
	{
		if(abs(ultra_R2.DIS-ultra_R2DISLast)>80)
		{ultra_R2.DIS=ultra_R2DISLast;return;}
		else {ultra_R2DISLast=ultra_R2.DIS;return;}
	}	
}
