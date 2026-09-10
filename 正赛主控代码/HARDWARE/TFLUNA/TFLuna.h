#ifndef __TFLUNA_H
#define __TFLUNA_H
#include "stdio.h"	
#include "sys.h" 

#define LASER 				0	//使能（1）/禁止（0）串口2接收

#if 		LASER
#define USART3_REC_LEN  	200 	//定义最大接收字节数 200

typedef struct {

	uint16_t Dist;
	uint16_t AMP;
	uint8_t RecieveFlag;

}lidar;

extern lidar BenewakeLidar;


extern u8  USART3_RX_BUF[USART3_REC_LEN];		//接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
extern u16 USART3_RX_STA;         				//接收状态标记

//如果想串口中断接收，请不要注释以下宏定义
void laser_uart3_init(u32 bound);
//void USART3_send_byte(uint8_t byte);
//void USART3_Send_bytes(uint8_t *Buffer, uint8_t Length);
//void send_3out(uint8_t *data,uint8_t length,uint8_t send);
//void send_out(int16_t *data,uint8_t length,uint8_t send);
//void display(int16_t *num,u8 send,u8 count);
//void CHeck(u8 *RX_BUF);
//void USART3_Send(uint8_t *Buffer, uint8_t Length);
//void send_com(u8 data);
//void Mea_dis(void);
//extern u8 Receive_ok;
#endif

#endif


