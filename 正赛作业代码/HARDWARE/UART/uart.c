#include "usart.h"
#include "stdio.h"

 u16 odd_Cx_1=0,odd_Cy_1=0,odd_flag_left=0,odd_flag_right=0;
 u16 even_Cx_1=0,even_Cy_1=0,even_flag_left=0,even_flag_right=0;
		u16 run_flag;
		u8 speed_ctrl;
		
		u8 odd_speed_ctrl;
		u8 even_speed_ctrl;
		
		u8 collect_ctrl_1;
		u8 collect_ctrl_2;
		

		
//void openmv_get(void)
//{
//	odd_flag_left  = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10);
//	odd_flag_right = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
//	odd_speed_ctrl = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
//	
//	even_flag_left  = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);
//	even_flag_right = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11);
//	even_speed_ctrl = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12);
//}
void openmv_get(void)
{
	odd_flag_left  = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);
	odd_flag_right = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
	odd_speed_ctrl = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
	
	even_flag_left  = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);
	even_flag_right = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
	even_speed_ctrl = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
}

void USART3_Init(void)      //OPENMV1
{
	  //USART3_TX:PB   10
	  //USART3_RX:PB   11
		GPIO_InitTypeDef GPIO_InitStructure;     //串口端口配置结构体变量
		USART_InitTypeDef USART_InitStructure;   //串口参数配置结构体变量
		NVIC_InitTypeDef NVIC_InitStructure;     //串口中断配置结构体变量

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //打开PA端口时钟

		//USART3_TX   PB10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          		 //PA9
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  		 //设定IO口的输出速度为50MHz
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	   		 //复用推挽输出
		GPIO_Init(GPIOB, &GPIO_InitStructure);             	 	 //初始化PA9
		//USART3_RX	  PB11
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;             //PA10
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
		GPIO_Init(GPIOB, &GPIO_InitStructure);                 //初始化PA10 

		//USART3 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;  //抢占优先级0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		    //子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			      //IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	                          //根据指定的参数初始化VIC寄存器

		//USART 初始化设置
		USART_InitStructure.USART_BaudRate = 115200;                  //串口波特率为115200
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;        //一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;           //无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                  //收发模式
		USART_Init(USART3, &USART_InitStructure);                     //初始化串口1
	
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //使能中断
		USART_Cmd(USART3, ENABLE);                     //使能串口1

  	//如下语句解决第1个字节无法正确发送出去的问题
	  USART_ClearFlag(USART3, USART_FLAG_TC);        //清串口1发送标志 
		
}

void UART4_Init(void)  //OPENMV2
{
	  //USART3_TX:PB   10
	  //USART3_RX:PB   11
		GPIO_InitTypeDef GPIO_InitStructure;     //串口端口配置结构体变量
		USART_InitTypeDef USART_InitStructure;   //串口参数配置结构体变量
		NVIC_InitTypeDef NVIC_InitStructure;     //串口中断配置结构体变量

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);   //打开PA端口时钟

		//USART4_TX   PC10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          		 //PA9
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  		 //设定IO口的输出速度为50MHz
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	   		 //复用推挽输出
		GPIO_Init(GPIOC, &GPIO_InitStructure);             	 	 //初始化PA9
		//USART4_RX	  PC11
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;             //PA10
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
		GPIO_Init(GPIOC, &GPIO_InitStructure);                 //初始化PA10 

		//USART4 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;  //抢占优先级0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		    //子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			      //IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	                          //根据指定的参数初始化VIC寄存器

		//USART 初始化设置
		USART_InitStructure.USART_BaudRate = 115200;                  //串口波特率为115200
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;        //一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;           //无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                  //收发模式
		USART_Init(UART4, &USART_InitStructure);                     //初始化串口1
	
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //使能中断
		USART_Cmd(UART4, ENABLE);                     //使能串口1

  	//如下语句解决第1个字节无法正确发送出去的问题
	  USART_ClearFlag(UART4, USART_FLAG_TC);        //清串口1发送标志 
		
}

//------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------
//void NVIC_Configuration(void)      //配置主从单片机通信                                                         
//{
//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);					/* 嵌套向量中断控制器组选择 */
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  				/* 配置USART为中断源 */
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  		/* 抢断优先级*/
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  			/* 子优先级 */
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  				/* 使能中断 */
//    NVIC_Init(&NVIC_InitStructure);  								/* 初始化配置NVIC */
//}

//void USART_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	
//    //打开串口GPIO的时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//	//打开串口外设的时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	//将USART Tx的GPIO配置为推挽复用模式GPIOA2
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//  	//将USART Rx的GPIO配置为浮空输入模式GPIOA10
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//    // 配置串口的工作参数
//	USART_InitStructure.USART_BaudRate = 115200;						// 配置波特率
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 配置帧数据字长8或9
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 配置停止位.5/1/1.5/2
//	USART_InitStructure.USART_Parity = USART_Parity_No ;							// 配置校验位，奇偶无
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 配置硬件流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 配置工作模式，收发一起
//	USART_Init(USART2, &USART_InitStructure);									// 完成串口的初始化配置
//	NVIC_Configuration();															// 串口中断优先级配置
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);							// 使能串口接收中断
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);									// 使能空闲中断
//	USART_Cmd(USART2, ENABLE);	    										// 使能串口
//}

//------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------
//发送函数
//void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
//{
//	/* 发送一个字节数据到USART */
//	USART_SendData(pUSARTx,ch);
//	/* 等待发送数据寄存器为空 */
//	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
//}

///****************** 发送8位的数组 ************************/
//void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
//{
//    uint8_t i;
//    for(i=0; i<num; i++)
//    {
//        /* 发送一个字节数据到USART */
//        Usart_SendByte(pUSARTx,array[i]);
//    }
//    /* 等待发送完成 */
//    while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
//}

///*****************  发送字符串 **********************/
//void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
//{
//    unsigned int k=0;
//    do 
//    {
//        Usart_SendByte( pUSARTx, *(str + k) );
//        k++;
//    } while(*(str + k)!='\0');
//    /* 等待发送完成 */
//    while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
//    {}
//}

///*****************  发送一个16位数 **********************/
//void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
//{
//    uint8_t temp_h, temp_l;
//    /* 取出高八位 */
//    temp_h = (ch&0XFF00)>>8;
//    /* 取出低八位 */
//    temp_l = ch&0XFF;
//    /* 发送高八位 */
//    USART_SendData(pUSARTx,temp_h);	
//    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
//    /* 发送低八位 */
//    USART_SendData(pUSARTx,temp_l);	
//    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
//}



//------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------
//接受openmv传输的信号
//USART3 全局中断服务函数
void USART3_IRQHandler(void)		 
{
		u8 com_data; 
		u8 i;
		static u8 RxCounter1=0;
		static u16 RxBuffer1[20]={0};
		static u8 RxState = 0;	
		static u8 RxFlag1 = 0;


		if( USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)  	   //接收中断  
		{
				USART_ClearITPendingBit(USART3,USART_IT_RXNE);   //清除中断标志
				com_data = USART_ReceiveData(USART3);
			
				if(RxState==0&&com_data==0x2C)  //0x2c帧头
				{
					RxState=1;
					RxBuffer1[RxCounter1++]=com_data;
				}
		
				else if(RxState==1&&com_data==0x12)  //0x12帧头
				{
					RxState=2;
					RxBuffer1[RxCounter1++]=com_data;
				}
		
				else if(RxState==2)
				{
					RxBuffer1[RxCounter1++]=com_data;

					if(RxCounter1>=20||com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
					{
						RxState=3;
						RxFlag1=1;
						odd_Cx_1= (RxBuffer1[RxCounter1-10]<<8)+(RxBuffer1[RxCounter1-11]);
						odd_Cy_1= (RxBuffer1[RxCounter1-8]<<8)+(RxBuffer1[RxCounter1-9]);
						odd_flag_left =(RxBuffer1[RxCounter1-6]<<8)+(RxBuffer1[RxCounter1-7]);
						odd_flag_right=(RxBuffer1[RxCounter1-4]<<8)+(RxBuffer1[RxCounter1-5]);
						odd_speed_ctrl=(RxBuffer1[RxCounter1-2]<<8)+(RxBuffer1[RxCounter1-3]);

					}
				}
		
				else if(RxState==3)		//检测是否接受到结束标志
				{
						if(RxBuffer1[RxCounter1-1] == 0x5B)
						{
									USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);//关闭DTSABLE中断
									if(RxFlag1)
									{
								
//									printf("(%d,%d)\r\n",Cx,Cy);
									}
									RxFlag1 = 0;
									RxCounter1 = 0;
									RxState = 0;
									USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
						}
						else   //接收错误
						{
									RxState = 0;
									RxCounter1=0;
									for(i=0;i<20;i++)
									{
											RxBuffer1[i]=0x00;      //将存放数据数组清零
									}
						}
				} 
	
				else   //接收异常
				{
						RxState = 0;
						RxCounter1=0;
						for(i=0;i<20;i++)
						{
								RxBuffer1[i]=0x00;      //将存放数据数组清零
						}
				}
				
	

		}

		
}
//USART3 全局中断服务函数
void UART4_IRQHandler(void)		 
{

		u8 com_data; 
		u8 i;
		static u8 RxCounter1=0;
		static u16 RxBuffer1[20]={0};
		static u8 RxState = 0;	
		static u8 RxFlag1 = 0;


		if( USART_GetITStatus(UART4,USART_IT_RXNE)!=RESET)  	   //接收中断  
		{
				USART_ClearITPendingBit(UART4,USART_IT_RXNE);   //清除中断标志
				com_data = USART_ReceiveData(UART4);
			
				if(RxState==0&&com_data==0x2C)  //0x2c帧头
				{
					RxState=1;
					RxBuffer1[RxCounter1++]=com_data;
				}
		
				else if(RxState==1&&com_data==0x12)  //0x12帧头
				{
					RxState=2;
					RxBuffer1[RxCounter1++]=com_data;
				}
		
				else if(RxState==2)
				{
					RxBuffer1[RxCounter1++]=com_data;

					if(RxCounter1>=20||com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
					{
						RxState=3;
						RxFlag1=1;
						even_Cx_1= (RxBuffer1[RxCounter1-10]<<8)+(RxBuffer1[RxCounter1-11]);
						even_Cy_1= (RxBuffer1[RxCounter1-8]<<8)+(RxBuffer1[RxCounter1-9]);
						even_flag_left =(RxBuffer1[RxCounter1-6]<<8)+(RxBuffer1[RxCounter1-7]);
						even_flag_right=(RxBuffer1[RxCounter1-4]<<8)+(RxBuffer1[RxCounter1-5]);
						even_speed_ctrl=(RxBuffer1[RxCounter1-2]<<8)+(RxBuffer1[RxCounter1-3]);
					}
				}
		
				else if(RxState==3)		//检测是否接受到结束标志
				{
						if(RxBuffer1[RxCounter1-1] == 0x5B)
						{
									USART_ITConfig(UART4,USART_IT_RXNE,DISABLE);//关闭DTSABLE中断
									if(RxFlag1)
									{
							
//									printf("(%d,%d)\r\n",Cx,Cy);
									}
									RxFlag1 = 0;
									RxCounter1 = 0;
									RxState = 0;
									USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);
						}
						else   //接收错误
						{
									RxState = 0;
									RxCounter1=0;
									for(i=0;i<20;i++)
									{
											RxBuffer1[i]=0x00;      //将存放数据数组清零
									}
						}
				} 
	
				else   //接收异常
				{
						RxState = 0;
						RxCounter1=0;
						for(i=0;i<20;i++)
						{
								RxBuffer1[i]=0x00;      //将存放数据数组清零
						}
				}
				
	

		}

		
}

//------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------
//接受主机串口通信传输的信号
//将串口接收到的数据发送出去
//在stm32f01x_it.c中编写中断服务函数
//串口中断服务函数
//void USART2_IRQHandler(void)
//{


//    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)	//如果收到一个数据
//    {		
//        run_flag = USART_ReceiveData(USART2);				//就将这个数据传给ucTemp
//        Usart_SendByte(USART2,speed_ctrl);    				//将接收到的数据发送出去
//    }
//}


//将串口接收一帧的数据并发送出去
//在stm32f01x_it.c中编写中断服务函数
//串口中断服务函数
//uint8_t buf_rx[20];
//uint8_t i = 0,j = 0;
//void USART2_IRQHandler(void)
//{
//    uint8_t ucTemp;
//    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)	//如果收到一个数据
//    {		
//        if(i < 20)												//无论发过来多少，只接收20个数据
//            buf_rx[i++] = USART_ReceiveData(USART2);			//就将这个数据传给buf_rx
//    }	 
//    if(USART_GetITStatus(USART2,USART_IT_IDLE) != RESET)		//接收到一帧数据
//    {
//        j++;
//        USART2->SR;
//        USART2->DR;
//        printf("接收中断次数 %d\r\n",i);
//        printf("空闲中断次数 %d\r\n",j);
//        i = 0;
//        printf("1帧 %s\r\n",buf_rx);
//        memset(buf_rx,0,sizeof(buf_rx));
//    }
//}
