/***************************************************************
 * Name:      USART.H
 * Purpose:   code for USART DEVICE
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/
////////////////////////////////////////////////////////////////////////////////// 
#ifndef __RELAY_H_
#define __RELAY_H_
#include "kernel-includes.h"
#include "drivers-includes.h"
#include "corefunc.h"	
 
enum relays{relay1=1,relay2,relay3,relay4};
enum relays_trigger{relay_closed=0,relay_opened=1};
typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
	u8  relay1_status:2;
	u8  relay2_status:2;
	u8  relay3_status:2;
	u8  relay4_status:2;
}relay_device;


#define RELAY_1 PBout(11)


	

os_err_t relays_register(u16 task_id);

#endif	   
















