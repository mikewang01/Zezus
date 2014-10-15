/***************************************************************
 * Name:      USART.H
 * Purpose:   code for USART DEVICE
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/
////////////////////////////////////////////////////////////////////////////////// 
#ifndef __USART_H
#define __USART_H

#include "kernel-includes.h"
#include "drivers-includes.h"

#define USART_RX_BUFFER_SIZE 64
//inherented from fathee class os_device_t 
typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
	u32 pclk2 ;
	u32 bound ;
	void *data;
}usart_device;


typedef struct
{
	u16 length;
	char data[USART_RX_BUFFER_SIZE];
}usart_data;



#define usart_send_message(x,y ,z) send_message((x),SYSTEM_EVENT , DEV_MSG , USART1_RX ,(void*)(y), z) //x taskid to recieve ,y data of message ,z tyoe of data
	
extern u8 USART_RX_BUF[64];     //接收缓冲,最大63个字节.末字节为换行符 
extern u8 USART_RX_STA;         //接收状态标记	

//如果想串口中断接收，请不要注释以下宏定义
//#define EN_USART1_RX //使能串口1接收
os_err_t usart_register(u16 task_id);

#endif	   
















