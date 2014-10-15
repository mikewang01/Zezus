#ifndef __ME2CH2O_H_
#define __ME2CH2O_H_


#include "corefunc.h"
#include "kernel-includes.h"
#include "drivers-includes.h"
#include "stm32f0xx_gpio.h"



typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
 	
}adc_device;


/*data stuct store with correction coefficient
equation: y=a+b*x
*/
typedef struct
{
	u32 para_a;  /**/
	u32 para_b;  /**/
	
}correction_para;



enum sensor_order
{
	get_correction_para     , /*get correction parameter for the sensor*/
	set_correction_para     , /*set correction parameter fo rthe sensor*/
	update_correction_sensor, /*update the new parameter from eeprom*/
	sensor_open             , /*open sensor to make it work*/
	sensor_close            , /*close sensor to make it power saving*/
	
};

os_err_t me2_ch20_register(u16 task_id);

#endif
