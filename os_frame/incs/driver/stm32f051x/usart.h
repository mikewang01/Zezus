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
	
extern u8 USART_RX_BUF[64];     //���ջ���,���63���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART_RX_STA;         //����״̬���	

//����봮���жϽ��գ��벻Ҫע�����º궨��
//#define EN_USART1_RX //ʹ�ܴ���1����
os_err_t usart_register(u16 task_id);

#endif	   
















