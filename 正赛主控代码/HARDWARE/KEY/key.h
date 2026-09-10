#ifndef __KEY_H
#define	__KEY_H

#include "stm32f4xx.h"



#define KEY1 		PEin(0)   	                                                                             	//PB13
#define KEY2 		PEin(1)		                                                                         		//PG2
#define KEY3  	PEin(2)		                                                                         	//PA12
#define KEY4		PEin(3)

#define KEY1_PRES                 1
#define KEY2_PRES                 2
#define KEY3_PRES                 3
#define KEY4_PRES                 4


void Key_Init(void);
u8   Key_Scan(void);



#endif /* __KEY_H */

