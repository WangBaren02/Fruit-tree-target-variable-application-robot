#ifndef __ULTRA_H
#define __ULTRA_H

#include "stm32f4xx.h"
/*
1--L1
2--L2
3--R1
4--R2
5--F1
6--F2
*/
#define filter_nub	5
#define Limit(x, min, max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )//限幅
/* 引脚宏定义 */
/* Left1宏定义 PE15:trig1  PA0:echo1	TIM5 */
#define Left1_trig_PIN                   	GPIO_Pin_12                
#define Left1_trig_GPIO_PORT             	GPIOE                      
#define Left1_trig_GPIO_CLK              	RCC_AHB1Periph_GPIOE	

#define Left2_trig_PIN                   	GPIO_Pin_13                
#define Left2_trig_GPIO_PORT             	GPIOE                      
#define Left2_trig_GPIO_CLK             	RCC_AHB1Periph_GPIOE

#define Right1_trig_PIN                  	GPIO_Pin_0                 
#define Right1_trig_GPIO_PORT            	GPIOB                    
#define Right1_trig_GPIO_CLK             	RCC_AHB1Periph_GPIOB	

#define Right2_trig_PIN                  	GPIO_Pin_1                
#define Right2_trig_GPIO_PORT            	GPIOD                     
#define Right2_trig_GPIO_CLK             	RCC_AHB1Periph_GPIOD	

#define Front1_trig_PIN                   	GPIO_Pin_2               
#define Front1_trig_GPIO_PORT             	GPIOD                   
#define Front1_trig_GPIO_CLK              	RCC_AHB1Periph_GPIOD

#define Front2_trig_PIN                    	GPIO_Pin_3              
#define Front2_trig_GPIO_PORT              	GPIOD                    
#define Front2_trig_GPIO_CLK               	RCC_AHB1Periph_GPIOD

/******************************************************************************/
#define Left1_echo_PIN                   	GPIO_Pin_2                
#define Left1_echo_GPIO_PORT             	GPIOA                      
#define Left1_echo_GPIO_CLK              	RCC_AHB1Periph_GPIOA
#define Left1_echo_GPIO_PinSource					GPIO_PinSource2

#define Left2_echo_PIN                   	GPIO_Pin_3        
#define Left2_echo_GPIO_PORT             	GPIOA                     
#define Left2_echo_GPIO_CLK              	RCC_AHB1Periph_GPIOA
#define Left2_echo_GPIO_PinSource					GPIO_PinSource3

#define Right1_echo_PIN                  	GPIO_Pin_5                
#define Right1_echo_GPIO_PORT            	GPIOA                     
#define Right1_echo_GPIO_CLK             	RCC_AHB1Periph_GPIOA
#define Right1_echo_GPIO_PinSource				GPIO_PinSource5

#define Right2_echo_PIN                  	GPIO_Pin_3                
#define Right2_echo_GPIO_PORT            	GPIOB                  
#define Right2_echo_GPIO_CLK             	RCC_AHB1Periph_GPIOB
#define Right2_echo_GPIO_PinSource				GPIO_PinSource3

#define Front1_echo_PIN                  	GPIO_Pin_5               
#define Front1_echo_GPIO_PORT            	GPIOE                  
#define Front1_echo_GPIO_CLK             	RCC_AHB1Periph_GPIOE
#define Front1_echo_GPIO_PinSource				GPIO_PinSource5

#define Front2_echo_PIN                  	GPIO_Pin_6                 
#define Front2_echo_GPIO_PORT            	GPIOE                    
#define Front2_echo_GPIO_CLK             	RCC_AHB1Periph_GPIOE
#define Front2_echo_GPIO_PinSource				GPIO_PinSource6

#define 		TRIG_L1 		{PEout(12)=1;Delay_ULTRA(60);PEout(12)=0;}	//40
#define 		TRIG_L2 		{PEout(13)=1;Delay_ULTRA(60);PEout(13)=0;}
#define 		TRIG_R1 		{PBout(0)=1;Delay_ULTRA(60);PBout(0)=0;}
#define 		TRIG_R2 		{PDout(1)=1;Delay_ULTRA(60);PDout(1)=0;}
#define 		TRIG_F1 		{PDout(2)=1;Delay_ULTRA(60);PDout(2)=0;}
#define 		TRIG_F2 		{PDout(3)=1;Delay_ULTRA(60);PDout(3)=0;}

typedef struct
{
	u8  CAPTURE_STA;                 //通道输入捕获标志，高两位做捕获标志，低6位做溢出标志		
	u16 CAPTURE_UPVAL;               //通道上升沿数
	u16 CAPTURE_DOWNVAL;             //通道下降沿数
	u32	CAPTURE_VAL;	     					 //通道输入捕获值
	
	float DIS; 											 //超声波距离值
	float Last_DIS;
	float max;
	float min;
	u8  CAPTURE_SELECT;                        
} Ultra_cap_t;

extern Ultra_cap_t ultra_L1;
extern Ultra_cap_t ultra_L2;
extern Ultra_cap_t ultra_R1;
extern Ultra_cap_t ultra_R2;
extern Ultra_cap_t ultra_F1;
extern Ultra_cap_t ultra_F2;

extern float ultra_F1DISLast;
extern float ultra_F2DISLast;
extern float ultra_L1DISLast;
extern float ultra_L2DISLast;
extern float ultra_R1DISLast;
extern float ultra_R2DISLast;

void Ultrasonic_Init(void);
void Ultra_TIM2_Init(u32 arr, u16 psc);
void Ultra_TIM9_Init(u16 arr, u16 psc);
void Delay_ULTRA(unsigned char n);
void ultra_Filter(Ultra_cap_t *data);

void Ultra_TEST(void);
void Ultra_Trig(void);
#endif

