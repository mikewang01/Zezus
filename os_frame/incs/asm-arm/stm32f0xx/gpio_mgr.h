
#ifndef __GPIO_MGR_H_
#define __GPIO_MGR_H_ 
/***************************************************************
 * Name:      gpio_mgr.h
 * Purpose:   code for io struct
 * Author:    mikewang(s)
 * Created:   2014-10-9
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/
 
/*********************************************************************
 * INCLUDES
 */
#include "cpu.h"
#include "stm32f0xx_gpio.h"

/* Exported macros --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

   
   
/** 
  * @brief  gpio configuration Structure definition  
  */ 

enum gpio_port
{
    PORT_A=0,
	  PORT_B,
	  PORT_C,
	  PORT_D
};


enum gpio_num
{
   IO_0=0,
	 IO_1,
	 IO_2,
	 IO_3,
	 IO_4,
	 IO_5,
	 IO_6,
	 IO_7,
	 IO_8,
	 IO_9,
	 IO_10,
	 IO_11,
	 IO_12,
	 IO_13,
	 IO_14,
	 IO_15,
	 IO_16
		
};



/*define io manager structor*/
typedef struct
{
  u8 gpio_num;
	GPIOMode_TypeDef gpio_mode;
	u8 gpio_af;
	GPIOOType_TypeDef gpio_output_type;
  GPIOSpeed_TypeDef gpio_speed;
	GPIOPuPd_TypeDef gpio_pupd;
	
}io_config_t;


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Initialization and Configuration functions *********************************/
void os_gpio_init();
#endif





























