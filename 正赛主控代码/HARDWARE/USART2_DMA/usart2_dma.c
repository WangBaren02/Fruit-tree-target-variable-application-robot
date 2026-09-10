#include "usart2_dma.h"
#include "stdio.h"
#include "oled.h"

u8 rec_data_u2[U2_DATA_LEN]={0};   // ????
u8 send_data_u2[U2_DATA_LEN]={0};  // ????

float yaw=0;
short yaw_H=0;
short yaw_L=0;
short yaw_temp=0;
float yaw_test=0;

int flag_YAW=0;
float start_YAW=0;
u8 mode=0;			//mode=0和mode=1无事发生

void USART2_Init(int Baud)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;    //DMA1???6
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  //USART2_TX
	DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);//????
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)send_data_u2;//????
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//dma????
	DMA_InitStructure.DMA_BufferSize = U2_DATA_LEN;//??DMA??????????
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//??DMA???????,????
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//??DMA???????

	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//??????
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;//??????
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//??DMA?????
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//??DMA?????
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;    //????FIFO?????????????? : ???FIFO??
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;  	//???FIFO????
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; //???Burst????????
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //???Burst???????? */

	DMA_Init(DMA1_Stream6,&DMA_InitStructure);//??DMA1???
	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);//????

  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);//?????DMA???? 
	DMA_Cmd(DMA1_Stream6,DISABLE);//????
  //USART2_RX
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//DMA????
	
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);//????
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rec_data_u2;//????
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//dma????
	DMA_InitStructure.DMA_BufferSize = U2_DATA_LEN;//??DMA??????????
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//??DMA???????,????
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//??DMA???????
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//??????
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//??????
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//??DMA?????
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//??DMA?????
	
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);//??DMA1???	
	DMA_Cmd(DMA1_Stream5,ENABLE);//????

	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);//?????DMA??

  //???????
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_BaudRate = Baud;
	//?????
  USART_Init(USART2,&USART_InitStructure);
	//????
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //?????????
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //??????
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;              //???????
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //????
  NVIC_Init(&NVIC_InitStructure);

	//??DMA????
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	//??DMA????
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);

	//????
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
   
  USART_Cmd(USART2, ENABLE);//????

  //??IO?
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  //????:???
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //??:????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  //??????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  //????:???
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//??????
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*
********************************************************************************************************************
*                  void  USART2_IRQHandler(void)
*
*Description : ??2???? 
*Arguments   : BufferSRC:????????;BufferSize:???????
*Returns     : none
*Notes       : none
********************************************************************************************************************
*/
void USART2_DMATransfer(uint32_t *BufferSRC, uint32_t BufferSize)//UASRT DMA????
{
	DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
	DMA1_Stream6->NDTR = BufferSize;    //??BUFFER??
	DMA1_Stream6->M0AR = (uint32_t)BufferSRC;   //????
	DMA1_Stream6->CR |= DMA_SxCR_EN;	//??DMA,????
}



/*
********************************************************************************************************************
*                  void  USART2_IRQHandler(void)
*
*Description : ??2???? 
*Arguments   : none
*Returns     : none
*Notes       : none
********************************************************************************************************************
*/
void USART2_IRQHandler(void)
{
	int t;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		t= USART_ReceiveData(USART2);
		if((rec_data_u2[0]==0X55)&&(rec_data_u2[1]==0X53))
			{
				yaw_H=rec_data_u2[7];
				yaw_L=rec_data_u2[6];
				yaw_temp=(yaw_H<<8)|yaw_L;
				yaw_test=yaw_temp/32768.0f*180.0f;
				
//				if(flag_YAW==0)	//手动yaw清零
//				{
//					flag_YAW=1;
//					start_YAW=yaw;
//				}
				yaw_test=yaw_test-start_YAW;
				if(mode==2 && yaw_test>180)yaw_test=yaw_test - 360;
				else if(mode==3 && yaw_test<-180)yaw_test=yaw_test + 360;
			}
			yaw=yaw_test;
			USART_ClearITPendingBit(USART2, USART_IT_IDLE);         //??????
			DMA1_S5_Reset();                                       //??DMA??,????????
   }
	else if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		DMA1_Stream6->CR &= (uint32_t)(~DMA_SxCR_EN);   //??DMA,????
	}
}

/*
********************************************************************************************************************
*                  void DMA1_Stream6_IRQHandler(void)
*
*Description : ??2?????? 
*Arguments   : none
*Returns     : none
*Notes       : none
********************************************************************************************************************
*/
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) != RESET)
    {
        /* ????? */
        DMA_ClearFlag(DMA1_Stream6,DMA_IT_TCIF6);
        /* ??DMA */
        DMA_Cmd(DMA1_Stream6,DISABLE);
        /* ????????,???????????? */
        USART_ITConfig(USART2,USART_IT_TC,ENABLE);
    }
}

/*
********************************************************************************************************************
*                  void DMA1_S5_Reset(void)
*
*Description : ??2???? 
*Arguments   : none
*Returns     : none
*Notes       : none
********************************************************************************************************************
*/

void DMA1_S5_Reset(void)//DMA1_Stream6 ????
{ 
	DMA_ClearFlag(DMA1_Stream5,DMA_IT_TCIF5|DMA_FLAG_HTIF5|DMA_FLAG_TEIF5|DMA_FLAG_DMEIF5|DMA_FLAG_FEIF5);  
	
	DMA_Cmd(DMA1_Stream5,DISABLE); //??USART2 TX DMA1 ??????

 	//DMA_SetCurrDataCounter(DMA1_Channel5,Uart1_DMA_Len);//DMA???DMA?????
	DMA1_Stream5->NDTR = U2_DATA_LEN;
 	//DMA_Cmd(DMA1_Channel5, ENABLE);                    
	DMA_Cmd(DMA1_Stream5,ENABLE); //??USART2 TX DMA1 ??????
	/* ????? */
   
}
