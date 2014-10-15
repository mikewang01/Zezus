/***************************************************************
 * Name:      USART.H
 * Purpose:   code for USART DEVICE
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/
////////////////////////////////////////////////////////////////////////////////// 
#ifndef __KEY_H_
#define __KEY_H_
#include "kernel-includes.h"
#include "drivers-includes.h"
	
 
enum keys{key1=1,key2,key3};
enum keystatus{unkown=0,key_down,key_released};

typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
}key_device;


typedef struct
{
	u8  key;
	u8  status;
}key_data;



typedef struct 
{
  u8 key1_trigger:2;
	u8 key2_trigger:2;
	u8 key3_trigger:2;
	u8 key4_trigger:2;
} key_trigger;


#define keys_send_message(x,y ,z) send_message((x),SYSTEM_EVENT , DEV_MSG , KEY_STATUS_CHANGED ,(void*)(y), z) //x taskid to recieve ,y data of message ,z size of data
	

os_err_t keys_register(u16 task_id);

#endif	   
















