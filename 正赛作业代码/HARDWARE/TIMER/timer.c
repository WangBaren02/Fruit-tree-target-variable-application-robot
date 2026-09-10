#include "timer.h"
#include "led.h"
#include "usart.h"

int zhongduan;

void TIM1_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc;  
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
 
	TIM_ITConfig(  
		TIM1, //TIM2
		TIM_IT_Update  | TIM_IT_Trigger, 
		ENABLE  
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
 
	TIM_Cmd(TIM1, ENABLE);  //ê1?üTIMxíaéè
							 
}
 

void TIM2_Cap_Init(void)                                        
{    
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
    
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;             
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;         
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA,GPIO_Pin_0);                 
    
	
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;          
    TIM_TimeBaseStructure.TIM_Prescaler =0;           
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	
    TIM_ITRxExternalClockConfig(TIM2,TIM_TS_ETRF);         
    TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_SetCounter(TIM2, 0);        
    
    TIM_Cmd(TIM2,ENABLE );                            
}
 
void TIM3_Cap_Init(void)                                        
{    
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  
    
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;             
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;         
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD,GPIO_Pin_2);                 
    
	
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;          
    TIM_TimeBaseStructure.TIM_Prescaler =0;           
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	
    TIM_ITRxExternalClockConfig(TIM3,TIM_TS_ETRF);         
    TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_SetCounter(TIM3, 0);        
    
    TIM_Cmd(TIM3,ENABLE );                            
}

void TIM1_UP_IRQHandler(void)   
{
	
}

void delay_ctrl(void)
{
	//GPIO_ResetBits(GPIOB,GPIO_Pin_15); 						 //拉低，让车减速
	zhongduan = 1;
	TIM_Cmd(TIM8,ENABLE); //启动定时器
	
}

void TIM8_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); 
 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc;  
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
	TIM_ClearFlag(TIM8,TIM_FLAG_Update);
 
	TIM_ITConfig(  
		TIM8, //TIM2
		TIM_IT_Update  | TIM_IT_Trigger, 
		ENABLE  
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  
							 
}
void TIM8_UP_IRQHandler(void)   
{
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) 
		{
	      TIM_ClearITPendingBit(TIM8, TIM_IT_Update  );  
			//GPIO_ResetBits(GPIOB,GPIO_Pin_15); 						 //拉高，让车恢复
			zhongduan = 0;
			TIM_Cmd(TIM8,DISABLE); //关闭定时器
		}
}
