/****
	*****************************************************************************
	* @file    usartx_dma.c
	* @author  杨少文
	* @version V1.0
	* @date    2021-7-7
	* @brief   串口DMA的初始化及上位机的应用
	* @license Copyright 2021. All rights reserved. 

	*****************************************************************************
	* @note		串口DMA的初始化 上位机的应用 中断标志位需要进行修改 没有进行宏定义
				串口1的初始化 
				USART1_RX---PC10
				USART1_TX---PC11	
	* @bug
	* @todo	
	* @details 
	*
	*****************************************************************************
	*/
#include "usart1_dma.h"
#include "sys.h"
/****
	*****************************************************************************
										        CONSTANTS & MACROS
	*****************************************************************************
	* @brief  串口名 DMA名 GPIO名
	*/
#define USARTx                          	USART1
#define USARTx_CLK                      	RCC_APB2Periph_USART1
#define USARTx_CLK_INIT                 	RCC_APB2PeriphClockCmd
#define USARTx_IRQn                     	USART1_IRQn
#define USARTx_IRQHandler               	USART1_IRQHandler

#define USARTx_TX_PIN                   	GPIO_Pin_6          
#define USARTx_TX_GPIO_PORT             	GPIOB                     
#define USARTx_TX_GPIO_CLK              	RCC_AHB1Periph_GPIOB
#define USARTx_TX_SOURCE                	GPIO_PinSource6
#define USARTx_TX_AF                    	GPIO_AF_USART1

#define USARTx_RX_PIN                   	GPIO_Pin_7       
#define USARTx_RX_GPIO_PORT             	GPIOB                   
#define USARTx_RX_GPIO_CLK              	RCC_AHB1Periph_GPIOB
#define USARTx_RX_SOURCE                	GPIO_PinSource7
#define USARTx_RX_AF                    	GPIO_AF_USART1

#define USARTx_DMA                      	DMA2
#define USARTx_DMAx_CLK                 	RCC_AHB1Periph_DMA2
#define DMACLEAR                        	DMA2->HIFCR = DMA_Stream7_IT_MASK;				//TX	



#define USART_RX_DMA_Stream 							DMA2_Stream2
#define USART_RX_DMA_Channel   						DMA_Channel_4									
#define USART_RX_DMA_PeripheralBaseAddr 	(uint32_t) (&(USARTx->DR))

#define USART_TX_DMA_Stream 							DMA2_Stream7
#define USART_TX_DMA_Channel   						DMA_Channel_4
#define USART_TX_DMA_IRQn									DMA2_Stream7_IRQn
#define USART_TX_DMA_IRQHandler						DMA2_Stream7_IRQHandler
#define USART_TX_DMA_PeripheralBaseAddr 	(uint32_t) (&(USARTx->DR))

u8 rec_data_u1[U1_DATA_LEN];      													//接收数据地址
u8 send_data_u1[U1_DATA_LEN];     													//发送数据地址

/****
	* @brief  串口的初始化
	* @param  无
	* @note   无
	* @retval 无
	* @bug
    * @todo	
	*/
void USART1_Init(int Baud)
{
	NVIC_InitTypeDef NVIC_InitStructure;											//定义中断结构体
 	GPIO_InitTypeDef GPIO_InitStructure;											//定义IO初始化结构体
	USART_InitTypeDef USART_InitStructure;										//定义串口结构体
	DMA_InitTypeDef DMA_InitStructure;												//定义DMA结构体
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);								//打开串口对应的外设时钟

	//**********************串口 发送 DMA 设置（DMA1_Stream6）**************************
	
    RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);					//启动DMA时钟

	NVIC_InitStructure.NVIC_IRQChannel = USART_TX_DMA_IRQn; 	//DMA发送中断设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(USART_TX_DMA_Stream );												 //DMA通道配置
	DMA_InitStructure.DMA_Channel = USART_TX_DMA_Channel;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART_TX_DMA_PeripheralBaseAddr;	//外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)send_data_u1;							//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;											//dma传输方向
	DMA_InitStructure.DMA_BufferSize = U1_DATA_LEN;															//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;						//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;											//设置DMA的内存递增模式

	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//外设数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;					//内存数据字长
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;																//设置DMA的传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;													//设置DMA的优先级别
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;    									//指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;  					//指定了FIFO阈值水平
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 								//指定的Burst转移配置内存传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 				//指定的Burst转移配置外围转移

	DMA_Init(USART_TX_DMA_Stream , &DMA_InitStructure);													//配置DMA1的通道
	
	DMA_ITConfig(USART_TX_DMA_Stream ,DMA_IT_TC,ENABLE);												//使能中断

    USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);															//使能串口的DMA发送接口
	
	DMA_Cmd(USART_TX_DMA_Stream,DISABLE);																				//关闭通道

	//**********************串口 接受 DMA 设置**************************
    RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);														//DMA通道配置
	
	DMA_DeInit(USART_RX_DMA_Stream);																						//DMA通道配置
	DMA_InitStructure.DMA_Channel = USART_TX_DMA_Channel;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART_TX_DMA_PeripheralBaseAddr;	//外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rec_data_u1;							//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;											//dma传输方向
	DMA_InitStructure.DMA_BufferSize = U1_DATA_LEN;															//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;						//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;											//设置DMA的内存递增模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//外设数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;							//内存数据字长
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;															//设置DMA的传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;											//设置DMA的优先级别
	
	DMA_Init(USART_RX_DMA_Stream, &DMA_InitStructure);													//配置DMA1的通道
	
	DMA_Cmd(USART_RX_DMA_Stream,ENABLE);																				//使能通道

	USART_DMACmd(USARTx,USART_DMAReq_Rx,ENABLE);																//使能串口的DMA接收

  USART_InitStructure.USART_WordLength = USART_WordLength_8b;									//初始化串口参数
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_BaudRate = Baud;
  USART_Init(USARTx,&USART_InitStructure);																		//初始化串口

  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;               						//通道设置为串口中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       						//中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;              						//中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 						//打开中断
  NVIC_Init(&NVIC_InitStructure);

	USART_DMACmd(USARTx,USART_DMAReq_Tx,ENABLE);																//采用DMA方式发送
	USART_DMACmd(USARTx,USART_DMAReq_Rx,ENABLE);																//采用DMA方式接收

	
	USART_ITConfig(USARTx,USART_IT_TC,DISABLE);																	//中断配置
	USART_ITConfig(USARTx,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USARTx,USART_IT_TXE,DISABLE);
	USART_ITConfig(USARTx,USART_IT_IDLE,ENABLE);
 
  USART_Cmd(USARTx, ENABLE);																									//启动串口
 
	RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK, ENABLE);													//设置IO口时钟
	GPIO_PinAFConfig(USARTx_TX_GPIO_PORT,USARTx_TX_SOURCE,USARTx_TX_AF);
	GPIO_PinAFConfig(USARTx_RX_GPIO_PORT,USARTx_RX_SOURCE,USARTx_RX_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	 															//管脚模式:输出口
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	    												//类型:推挽模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	 															//上拉下拉设置
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;													//IO口速度
	GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;  															//管脚指定
	GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);												//初始化

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;																//管脚模式:输入口
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	 													//上拉下拉设置
	GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;    														//管脚指定
	GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);      									//初始化
}

/****
	* @brief  串口发送
	* @param  无
	* @note   DMA_FLAG_TCIF7 没有进行宏定义 需要自行改动 数字与流的数字相同  TX
	* @retval 无
	* @bug
  * @todo	
	*/
void USART1_DMATransfer(uint32_t *BufferSRC, uint32_t BufferSize)							//UASRT DMA发送设置
{
	DMA_ClearFlag(USART_TX_DMA_Stream,DMA_FLAG_TCIF7);
	USART_TX_DMA_Stream->NDTR = BufferSize;    																	//配置BUFFER大小
	USART_TX_DMA_Stream->M0AR = (uint32_t)BufferSRC;   													//配置地址
	USART_TX_DMA_Stream->CR |= DMA_SxCR_EN;																			//打开DMA,开始发送
}



/****
	* @brief  串口接收中断
	* @param  无
	* @note   Uart1_Rec_Len 自行改动 串口几就是几
	* @retval 无
	* @bug
  * @todo	
	*/
void USART1_IRQHandler(void)
{

	int t;
	if(USART_GetITStatus(USARTx, USART_IT_IDLE) != RESET)  											//接收中断
	{
		t= USART_ReceiveData(USARTx);																							//读取数据 注意：这句必须要，否则不能够清除中断标志位。
		//Usart1_Rec_Len = U1_DATA_LEN - USART_RX_DMA_Stream->NDTR;
		//***********帧数据处理函数************//
		
		//*************************************//
		DMA2_S2_Reset();                                       										//恢复DMA指针，等待下一次的接收
  }
	else if(USART_GetITStatus(USARTx, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USARTx, USART_IT_TC);
		USART_TX_DMA_Stream->CR &= (uint32_t)(~DMA_SxCR_EN);   										//关闭DMA,发送完成
	}
}

/****
	* @brief  串口发送中断
	* @param  无
	* @note   DMA_IT_TCIF7 自行改动 流几就是几
	* @retval 无
	* @bug
  * @todo	
	*/
void USART_TX_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(USART_TX_DMA_Stream,DMA_IT_TCIF7) != RESET)
    {        
      DMA_ClearFlag(USART_TX_DMA_Stream,DMA_IT_TCIF7);												// 清除标志位       
      DMA_Cmd(USART_TX_DMA_Stream,DISABLE);																		//关闭DMA       
      USART_ITConfig(USARTx,USART_IT_TC,ENABLE);      												//打开发送完成中断,确保最后一个字节发送成功 
    }
}

/****
	* @brief  DMA重置函数
	* @param  无
	* @note   标志位 自行改动 流几就是几
	* @retval 无
	* @bug
  * @todo	
	*/

void DMA2_S2_Reset(void)
{ 
	DMA_ClearFlag(USART_RX_DMA_Stream,DMA_IT_TCIF2|DMA_FLAG_HTIF2|DMA_FLAG_TEIF2|DMA_FLAG_DMEIF2|DMA_FLAG_FEIF2);  
	DMA_Cmd(USART_RX_DMA_Stream,DISABLE); 																			//关闭USART2 TX DMA1 所指示的通道
	USART_RX_DMA_Stream->NDTR = U1_DATA_LEN;            
	DMA_Cmd(USART_RX_DMA_Stream,ENABLE); 																				//使能USART2 TX DMA1 所指示的通道 
}

/****
	* @brief  上位机函数 V7版本 
	* @param  无
	* @note   无
	* @retval 无
	* @bug
  * @todo	
	*/
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))      
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
void USART1_Upper_f_Computer(int16_t data1,int16_t data2,int16_t data3,int16_t data4,int16_t data5,int16_t data6,int16_t data7,int16_t data8,int16_t data9,int16_t data10)
{
	u8 _cnt = 0;	
	static u8 Send_DATA[50]; 
	
	Send_DATA[_cnt++] = 0xAA;    
	Send_DATA[_cnt++] = 0xFF;   
	Send_DATA[_cnt++] = 0xF1;   
	Send_DATA[_cnt++] = 20;   

	Send_DATA[_cnt++] = BYTE0(data1);   
	Send_DATA[_cnt++] = BYTE1(data1); 
	
	Send_DATA[_cnt++] = BYTE0(data2);   
	Send_DATA[_cnt++] = BYTE1(data2); 	

	Send_DATA[_cnt++] = BYTE0(data3);   
	Send_DATA[_cnt++] = BYTE1(data3); 	

	Send_DATA[_cnt++] = BYTE0(data4);   
	Send_DATA[_cnt++] = BYTE1(data4); 	
	
	Send_DATA[_cnt++] = BYTE0(data5);   
	Send_DATA[_cnt++] = BYTE1(data5); 	

	Send_DATA[_cnt++] = BYTE0(data6);   
	Send_DATA[_cnt++] = BYTE1(data6); 	
	
	Send_DATA[_cnt++] = BYTE0(data7);   
	Send_DATA[_cnt++] = BYTE1(data7); 	

	Send_DATA[_cnt++] = BYTE0(data8);   
	Send_DATA[_cnt++] = BYTE1(data8); 	

	Send_DATA[_cnt++] = BYTE0(data9);   
	Send_DATA[_cnt++] = BYTE1(data9); 

	Send_DATA[_cnt++] = BYTE0(data10);   
	Send_DATA[_cnt++] = BYTE1(data10); 
	
	u8 sum = 0;	
	u8 add_sum = 0;		
	for(u8 i = 0;i < Send_DATA[3]+4; i++)
	{
		sum += Send_DATA[i];	
		add_sum += sum;
	}

	Send_DATA[_cnt++] = sum;   
	Send_DATA[_cnt++] = add_sum; 	

	USART1_DMATransfer((u32*)Send_DATA, _cnt);
}
