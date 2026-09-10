#include "ultrasonic.h"
#include "sys.h"
#include "led.h"
#include "oled.h"
#include "filter.h"
/******************************************************************************
	定义超声波的初始值变量											
******************************************************************************/
/*	u8  CAPTURE_STA;               //通道输入捕获标志，高两位做捕获标志，低6位做溢出标志		
	u16 CAPTURE_UPVAL;               //通道上升沿数
	u16 CAPTURE_DOWNVAL;             //通道下降沿数
	u32	CAPTURE_VAL;	     				   //通道输入捕获值
	
	float DIS; 									     //超声波距离值
	float Last_DIS;
	float max;
	float min;
	u8  CAPTURE_SELECT; */
Ultra_cap_t ultra_L1 = {0, 0, 0, 0, 0.0f, 0.0f, 2550.0, 0.0, 0};   	//PE15
Ultra_cap_t ultra_L2 = {0, 0, 0, 0, 0.0f, 0.0f, 2550.0, 0.0, 0};		//PD11
Ultra_cap_t ultra_R1 = {0, 0, 0, 0, 0.0f, 0.0f, 2550.0, 0.0, 0};		//PE2
Ultra_cap_t ultra_R2 = {0, 0, 0, 0, 0.0f, 0.0f, 2550.0, 0.0, 0};		//PD10
Ultra_cap_t ultra_F1 = {0, 0, 0, 0, 0.0f, 0.0f, 2550.0, 0.0, 0};		//PB10
Ultra_cap_t ultra_F2 = {0, 0, 0, 0, 0.0f, 0.0f, 2550.0, 0.0, 0};		//PA8

FilterStruct Ultr_F1= {{0},0};
FilterStruct Ultr_F2= {{0},0};
FilterStruct Ultr_L1= {{0},0};
FilterStruct Ultr_L2= {{0},0};
FilterStruct Ultr_R1= {{0},0};
FilterStruct Ultr_R2= {{0},0};

float ultra_F1DISLast=0;
float ultra_F2DISLast=0;
float ultra_L1DISLast=0;
float ultra_L2DISLast=0;
float ultra_R1DISLast=0;
float ultra_R2DISLast=0;

void Ultrasonic_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(Left1_trig_GPIO_CLK, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = Left1_trig_PIN;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Left1_trig_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(Left1_trig_GPIO_PORT,Left1_trig_PIN);	
	
	RCC_AHB1PeriphClockCmd(Left2_trig_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = Left2_trig_PIN;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Left2_trig_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(Left2_trig_GPIO_PORT,Left2_trig_PIN);
	
	RCC_AHB1PeriphClockCmd(Right1_trig_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = Right1_trig_PIN;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Right1_trig_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(Right1_trig_GPIO_PORT,Right1_trig_PIN);
	
	RCC_AHB1PeriphClockCmd(Right2_trig_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = Right2_trig_PIN;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Right2_trig_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(Right2_trig_GPIO_PORT,Right2_trig_PIN);
	
	RCC_AHB1PeriphClockCmd(Front1_trig_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = Front1_trig_PIN;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Front1_trig_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(Front1_trig_GPIO_PORT,Front1_trig_PIN);
	
	RCC_AHB1PeriphClockCmd(Front2_trig_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = Front2_trig_PIN;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Front2_trig_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(Front2_trig_GPIO_PORT,Front2_trig_PIN);
	
	Ultra_TIM2_Init(0xFFFF, 84 - 1);
	Ultra_TIM9_Init(0xFFFF, 168 - 1);

}


void Ultra_TIM2_Init(u32 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	                    		//使能TIM2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);                       	//使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                       	//使能GPIOA时钟
	
	TIM_DeInit(TIM2);	                                                        		//重新初始化TIM2为默认状态	
	
	GPIO_InitStructure.GPIO_Pin = Left1_echo_PIN;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                  //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                          //速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                                //下拉
	GPIO_Init(Left1_echo_GPIO_PORT, &GPIO_InitStructure);                         //初始化PA2
	
	GPIO_InitStructure.GPIO_Pin = Left2_echo_PIN;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                  //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                          //速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                               	//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                               	//下拉
	GPIO_Init(Left2_echo_GPIO_PORT, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = Right1_echo_PIN;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                 	//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                         	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                               	//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                               	//下拉
	GPIO_Init(Right1_echo_GPIO_PORT, &GPIO_InitStructure);   
	
	GPIO_InitStructure.GPIO_Pin = Right2_echo_PIN;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                  //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                          //速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                                //下拉
	GPIO_Init(Right2_echo_GPIO_PORT, &GPIO_InitStructure);    
	
	GPIO_PinAFConfig(Left1_echo_GPIO_PORT,Left1_echo_GPIO_PinSource,GPIO_AF_TIM2);	//PA2 复用为TIM2 CH3
	GPIO_PinAFConfig(Left2_echo_GPIO_PORT,Left2_echo_GPIO_PinSource,GPIO_AF_TIM2);	//PA3 复用为TIM2 CH4
	GPIO_PinAFConfig(Right1_echo_GPIO_PORT,Right1_echo_GPIO_PinSource,GPIO_AF_TIM2);//PA5 复用为TIM2 CH1
	GPIO_PinAFConfig(Right2_echo_GPIO_PORT,Right2_echo_GPIO_PinSource,GPIO_AF_TIM2);//PB3 复用为TIM2 CH2
	
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void Ultra_TIM9_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM9_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);	                    	//使能TIM9时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);                       	//使能GPIOE时钟
	
	TIM_DeInit(TIM9);
	
	GPIO_InitStructure.GPIO_Pin = Front1_echo_PIN|Front2_echo_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                   	//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                           	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                 	//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                                 	//下拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);                                         	//初始化PB11 
	
	GPIO_PinAFConfig(Front1_echo_GPIO_PORT,Front1_echo_GPIO_PinSource,GPIO_AF_TIM9);//PE5复用为TIM9 CH1
	GPIO_PinAFConfig(Front2_echo_GPIO_PORT,Front2_echo_GPIO_PinSource, GPIO_AF_TIM9);//PE6 复用为TIM9 CH2
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
	
	TIM9_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM9_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM9_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM9_ICInitStructure.TIM_ICFilter = 0;
	
	TIM9_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM9, &TIM9_ICInitStructure);
	TIM9_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM9, &TIM9_ICInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM9,TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
	TIM_Cmd(TIM9, ENABLE);
}

/******************************************************************************
	* @brief  定时器2中断服务函数处理 
	* @param  无
	* @retval 无
	* @bug
	* @todo	
	*****************************************************************************
	*/
void TIM2_IRQHandler(void)
{
	if((ultra_L1.CAPTURE_STA & 0x80) == 0)
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		{
			if(ultra_L1.CAPTURE_STA & 0x40)
			{
				if((ultra_L1.CAPTURE_STA & 0x02) == 0x02)
				{
					ultra_L1.CAPTURE_STA = 0;
					TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising);
					ultra_L1.CAPTURE_VAL = 0xFFFFFFFF;
				}
				else ultra_L1.CAPTURE_STA++;			
			}
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
		{
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
			if(ultra_L1.CAPTURE_STA & 0x40)
			{
				ultra_L1.CAPTURE_DOWNVAL = TIM_GetCapture3(TIM2);
				ultra_L1.CAPTURE_VAL = ultra_L1.CAPTURE_DOWNVAL - ultra_L1.CAPTURE_UPVAL + (ultra_L1.CAPTURE_STA&0X3F)*0XFFFF;
				ultra_L1.DIS = ultra_L1.CAPTURE_VAL * 340.0f / 2000.0f;
				ultra_L1.DIS = Limit(ultra_L1.DIS,10,2550);
//				ultra_L1.DIS = enhanceMeanFilter(&Ultr_L1,ultra_L1.DIS,filter_nub);
				ultra_L1.DIS = MeanFilterLimit(&Ultr_L1,ultra_L1.DIS,filter_nub);
				ultra_L1.CAPTURE_STA = 0;		                                    //捕获标志位清零
				TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising);             //设置为上升沿捕获		  
			}
			else
			{
				ultra_L1.CAPTURE_UPVAL = TIM_GetCapture3(TIM2);		              //获取上升沿数据
				ultra_L1.CAPTURE_STA |= 0X40;		                                //标记已捕获到上升沿
				TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling);            //设置为下降沿捕获
			}
		}
	}
//------------------------------------------------------------------------------------------------------------------------
	if((ultra_L2.CAPTURE_STA & 0X80) == 0)
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		{
			if(ultra_L2.CAPTURE_STA & 0x40)
			{
				if((ultra_L2.CAPTURE_STA & 0x02) == 0x02)
				{
					ultra_L2.CAPTURE_STA = 0;
					TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising);
					ultra_L2.CAPTURE_VAL = 0xFFFFFFFF;
				}
				else ultra_L2.CAPTURE_STA++;
			}
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
		{
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
			if(ultra_L2.CAPTURE_STA & 0x40)
			{
				ultra_L2.CAPTURE_DOWNVAL = TIM_GetCapture4(TIM2);
				ultra_L2.CAPTURE_VAL = ultra_L2.CAPTURE_DOWNVAL - ultra_L2.CAPTURE_UPVAL + (ultra_L2.CAPTURE_STA&0X3F)*0XFFFF;
				ultra_L2.DIS = ultra_L2.CAPTURE_VAL * 340.0f / 2000.0f;
				ultra_L2.DIS = Limit(ultra_L2.DIS,10,2550);
//				ultra_L2.DIS=enhanceMeanFilter(&Ultr_L2,ultra_L2.DIS,filter_nub);
				ultra_L2.DIS = MeanFilterLimit(&Ultr_L2,ultra_L2.DIS,filter_nub);
//				ultra_L2.DIS=KalmanFilter(ultra_L2.DIS);
				ultra_L2.CAPTURE_STA = 0;
				TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising);
			}
			else
			{
				ultra_L2.CAPTURE_UPVAL = TIM_GetCapture4(TIM2);
				ultra_L2.CAPTURE_STA |= 0x40;
				TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling);
			}
		}
	}
	
//----------------------------------------------------------------------------------------------------------------------------
	if((ultra_R1.CAPTURE_STA & 0X80) == 0)
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		{
			if(ultra_R1.CAPTURE_STA & 0x40)
			{
				if((ultra_R1.CAPTURE_STA & 0x02) == 0x02)
				{
					ultra_R1.CAPTURE_STA = 0;
					TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising);
					ultra_R1.CAPTURE_VAL = 0xFFFFFFFF;
				}
				else ultra_R1.CAPTURE_STA++;
			}
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
		{
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
			if(ultra_R1.CAPTURE_STA & 0x40)
			{
				ultra_R1.CAPTURE_DOWNVAL = TIM_GetCapture1(TIM2);
				ultra_R1.CAPTURE_VAL = ultra_R1.CAPTURE_DOWNVAL - ultra_R1.CAPTURE_UPVAL + (ultra_R1.CAPTURE_STA&0X3F)*0XFFFF;
				ultra_R1.DIS = ultra_R1.CAPTURE_VAL * 340.0f / 2000.0f;
				ultra_R1.DIS = Limit(ultra_R1.DIS,10,2550);
//				ultra_R1.DIS = enhanceMeanFilter(&Ultr_R1,ultra_R1.DIS,filter_nub);
				ultra_R1.DIS = MeanFilterLimit(&Ultr_R1,ultra_R1.DIS,filter_nub);
				ultra_R1.CAPTURE_STA = 0;
				TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising);
			}
			else
			{
				ultra_R1.CAPTURE_UPVAL = TIM_GetCapture1(TIM2);
				ultra_R1.CAPTURE_STA |= 0x40;
				TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling);
			}
		}
	}
	
//------------------------------------------------------------------------------------------------------------------------------
	if((ultra_R2.CAPTURE_STA & 0X80) == 0)
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		{
			if(ultra_R2.CAPTURE_STA & 0x40)
			{
				if((ultra_R2.CAPTURE_STA & 0x02) == 0x02)
				{
					ultra_R2.CAPTURE_STA = 0;
					TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising);
					ultra_R2.CAPTURE_VAL = 0xFFFFFFFF;
				}
				else ultra_R2.CAPTURE_STA++;
			}
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
		{
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
			if(ultra_R2.CAPTURE_STA & 0x40)
			{
				ultra_R2.CAPTURE_DOWNVAL = TIM_GetCapture2(TIM2);
				ultra_R2.CAPTURE_VAL = ultra_R2.CAPTURE_DOWNVAL - ultra_R2.CAPTURE_UPVAL + (ultra_R2.CAPTURE_STA&0X3F)*0XFFFF;
				ultra_R2.DIS = ultra_R2.CAPTURE_VAL * 340.0f / 2000.0f;
				ultra_R2.DIS = Limit(ultra_R2.DIS,10,2550);
//				ultra_R2.DIS = enhanceMeanFilter(&Ultr_R2,ultra_R2.DIS,filter_nub);
				ultra_R2.DIS = MeanFilterLimit(&Ultr_R2,ultra_R2.DIS,filter_nub);
				ultra_R2.CAPTURE_STA = 0;
				TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising);
			}
			else
			{
				ultra_R2.CAPTURE_UPVAL = TIM_GetCapture2(TIM2);
				ultra_R2.CAPTURE_STA |= 0x40;
				TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling);
			}
		}
	}
//------------------------------------------------------------------------------------------------------------------------------

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4 ); //清除中断标志位	
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	if((ultra_F1.CAPTURE_STA & 0x80) == 0)
	{
		if(TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)
		{
			if(ultra_F1.CAPTURE_STA & 0x40)
			{
				if((ultra_F1.CAPTURE_STA & 0x02) == 0x02)
				{
					ultra_F1.CAPTURE_STA = 0;
					TIM_OC1PolarityConfig(TIM9, TIM_ICPolarity_Rising);
					ultra_F1.CAPTURE_VAL = 0xFFFFFFFF;
				}
				else ultra_F1.CAPTURE_STA++;			
			}
		}
		if(TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET)
		{
			TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);
			if(ultra_F1.CAPTURE_STA & 0x40)
			{
				ultra_F1.CAPTURE_DOWNVAL = TIM_GetCapture1(TIM9);
				ultra_F1.CAPTURE_VAL = ultra_F1.CAPTURE_DOWNVAL - ultra_F1.CAPTURE_UPVAL + (ultra_F1.CAPTURE_STA&0X3F)*0XFFFF;
				ultra_F1.DIS = ultra_F1.CAPTURE_VAL * 340.0f / 2000.0f;
				ultra_F1.DIS = Limit(ultra_F1.DIS,30,2550);
//				ultra_F1.DIS = enhanceMeanFilter(&Ultr_F1,ultra_F1.DIS,filter_nub);
				ultra_F1.DIS = MeanFilterLimit(&Ultr_F1,ultra_F1.DIS,filter_nub);
				ultra_F1.CAPTURE_STA = 0;		                                    //捕获标志位清零
				TIM_OC1PolarityConfig(TIM9, TIM_ICPolarity_Rising);                 //设置为上升沿捕获		  
			}
			else
			{
				ultra_F1.CAPTURE_UPVAL = TIM_GetCapture1(TIM9);		                //获取上升沿数据
				ultra_F1.CAPTURE_STA |= 0X40;		                                //标记已捕获到上升沿
				TIM_OC1PolarityConfig(TIM9, TIM_ICPolarity_Falling);                //设置为下降沿捕获
			}
		}
	}
//------------------------------------------------------------------------------------------------------------------------------
	if((ultra_F2.CAPTURE_STA & 0X80) == 0)
	{
		if(TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)
		{
			if(ultra_F2.CAPTURE_STA & 0x40)
			{
				if((ultra_F2.CAPTURE_STA & 0x02) == 0x02)
				{
					ultra_F2.CAPTURE_STA = 0;
					TIM_OC2PolarityConfig(TIM9, TIM_ICPolarity_Rising);
					ultra_F2.CAPTURE_VAL = 0xFFFFFFFF;
				}
				else ultra_F2.CAPTURE_STA++;
			}
		}
		if(TIM_GetITStatus(TIM9, TIM_IT_CC2) != RESET)
		{
			TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);
			if(ultra_F2.CAPTURE_STA & 0x40)
			{
				ultra_F2.CAPTURE_DOWNVAL = TIM_GetCapture2(TIM9);
				ultra_F2.CAPTURE_VAL = ultra_F2.CAPTURE_DOWNVAL - ultra_F2.CAPTURE_UPVAL + (ultra_F2.CAPTURE_STA&0X3F)*0XFFFF;
				ultra_F2.DIS = ultra_F2.CAPTURE_VAL * 340.0f / 2000.0f;
				ultra_F2.DIS = Limit(ultra_F2.DIS,30,2550);
//				ultra_F2.DIS = enhanceMeanFilter(&Ultr_F2,ultra_F2.DIS,filter_nub);
				ultra_F2.DIS = MeanFilterLimit(&Ultr_F2,ultra_F2.DIS,filter_nub);
				ultra_F2.CAPTURE_STA = 0;
				TIM_OC2PolarityConfig(TIM9, TIM_ICPolarity_Rising);
			}
			else
			{
				ultra_F2.CAPTURE_UPVAL = TIM_GetCapture2(TIM9);
				ultra_F2.CAPTURE_STA |= 0x40;
				TIM_OC2PolarityConfig(TIM9, TIM_ICPolarity_Falling);
			}
		}
	}
	
//------------------------------------------------------------------------------------------------------------------------------

	
	TIM_ClearITPendingBit(TIM9, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2); //清除中断标志位	
}

void Delay_ULTRA(unsigned char n)
{
	unsigned int i,j;
	for(j = 0;j < n;j++)
	{
		for(i=0;i<40;i++);
	}
}

void Ultra_Trig(void)
{
	TRIG_L1;
	TRIG_L2;
	TRIG_R1;
	TRIG_R2;
	TRIG_F1;
	TRIG_F2;
}

/******************************************************************************
	* @brief  超声波数据处理函数 包括对数据的计算 滤波 和 尖峰处理
	* @param  无
	* @retval 无
	* @bug
	* @todo	
	*****************************************************************************
	*/
void ultra_Filter(Ultra_cap_t *data)
{
	data->Last_DIS = data->DIS;
		if(data->DIS > data->max)
	{
		data->DIS=data->Last_DIS;
	}	
		if(data->DIS < data->min)
	{
		data->DIS=data->Last_DIS;
	}
}
