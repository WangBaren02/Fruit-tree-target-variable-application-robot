#ifndef __USART3_DMA_H
#define __USART3_DMA_H	 
#include "sys.h"
/****
	*****************************************************************************
	* @file    usartx_dma.c
	* @author  杨少文
	* @version V1.0
	* @date    2021-7-7
	* @brief   串口DMA驱动
	* @license Copyright 2021. All rights reserved. 

	*****************************************************************************
	* @note  	 串口_DMA
	* @bug
	* @todo	
	* @details 
	*
	*****************************************************************************
	*/
/****
	*****************************************************************************
										        CONSTANTS & MACROS
	*****************************************************************************
	* @brief  数据长度和接收发送的数组
	*/
#define U3_DATA_LEN  								15

extern u8 rec_data_u3[U3_DATA_LEN];   										// 接收数据
extern u8 send_data_u3[U3_DATA_LEN];   										// 接收数据
/*
*******************************************************************************
*                            FUNCTION PROTOTYPES
*                                串口函数
*******************************************************************************
*/
extern void USART3_Init(int Baud);
extern void USART3_DMATransfer(uint32_t *BufferSRC, uint32_t BufferSize);			//UASRT DMA发送设置
extern void DMA1_S1_Reset(void);																							//DMA1_Stream2 接收重置	RX
extern void USART3_Upper_f_Computer(int16_t data1,int16_t data2,int16_t data3,int16_t data4,int16_t data5,int16_t data6,int16_t data7,int16_t data8,int16_t data9,int16_t data10);

#endif /* __USART1_DMA_H */
