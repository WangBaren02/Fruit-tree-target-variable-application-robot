#include "tcs3200.h"
#include "delay.h"
#include "usart.h"

 
float RGB_Scale[6];         
int count1=0;  
int count2=0; 
int cnt[6];             
int flag1=0;
int flag2=0;
 
 
//S0-----PA4,S1-----PA5,S2-----PA6,S3-----PA7,LED-----PC5,OUT-----PA0
 
void TCS_GPIO_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);	 
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;				 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	 GPIO_Init(GPIOA, &GPIO_InitStructure);					
	 GPIO_SetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_5);	
	 GPIO_ResetBits(GPIOA,GPIO_Pin_4);	
				
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;				 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	 GPIO_Init(GPIOC, &GPIO_InitStructure);					
	 GPIO_SetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);	
	 GPIO_ResetBits(GPIOC,GPIO_Pin_0);
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    	
 GPIO_Init(GPIOC, &GPIO_InitStructure);	  				
 GPIO_SetBits(GPIOC,GPIO_Pin_5); 					
}
 
void filter1(int s2,int s3) 
{
    if(s2==0&&s3==0){
			S2=0;S3=0;
		}
    if(s2==0&&s3==1){
		  S2=0;S3=1;
		}
    if(s2==1&&s3==0){
			S2=1;S3=0;
		}
    if(s2==1&&s3==1){
		  S2=1;S3=1;
		}			
}
 void filter2(int s21,int s31) 
{
    if(s21==0&&s31==0){
			S21=0;S31=0;
		}
    if(s21==0&&s31==1){
		  S21=0;S31=1;
		}
    if(s21==1&&s31==0){
			S21=1;S31=0;
		}
    if(s21==1&&s31==1){
		  S21=1;S31=1;
		}			
}
void TSC_WB_1(int s2, int s3)
{
  count1 = 0;
	if(flag1<3)
		flag1 ++;   
	else 
		flag1=0;
	
   filter1(s2, s3); 
}

void TSC_WB_2(int s21, int s31)
{
  count2 = 0;
	if(flag2<3)
		flag2 ++;   
	else 
		flag2=0;
	
   filter2(s21, s31); 
}

int color_choice(int R,int G,int B)
{
	float g_rate;
	int color;
	g_rate = (float)G/(R + G +B);
	if ( (R>=1000 && G>=1.45*B)||(G>=1.2*R && G>=1.2*B) ) //»ÆÉ«ºÍÂÌÉ«
		color =YELLOW;
	else if((R<1000) && (R>=1.3*B) &&(R>=1.3*G)) 
		color =RED;
	else 
		color = NONE;
	
	return color;
}
