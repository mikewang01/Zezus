/***************************************************************
 * Name:      htu20.H
 * Purpose:   code for USART DEVICE
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/
////////////////////////////////////////////////////////////////////////////////// 
#ifndef __I2C_H
#define __I2C_H

#include "kernel-includes.h"
#include "drivers-includes.h"

#define I2C_TX_BUFFER_SIZE 64
#define TRIGGER_TEMP_MEASUREMENT 0Xe3
#define TRIGGER_HUMI_MEASUREMENT 0Xe5
#define WRITE_USER_REGISTER      0XE6
#define READ_USER_REGISTER       0XE7
#define SOFT_RESET               0XFE
#define HTU20_ADRESS             0X40




enum htu20_command{htu20_convert_start , htu20_wakeup , htu20_sleep};
//inherented from fathee class os_device_t 
typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
	u32 clk ;
	u8 dev_adress;
	void *data;
}htu20_device;


typedef struct
{
	u16 temper;
  u16 humidity;
	u16 ampfifier_times;
	u8 user_res_content;
}htu20_data;



#endif	   
















