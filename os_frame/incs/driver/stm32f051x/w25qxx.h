#ifndef __W25QXX_H__
#define __W25QXX_H__			    
/***************************************************************
 * Name:      w25qxx.h
 * Purpose:   code for winbond w25qxx serials spi flash
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
#include "kernel-includes.h"
#include "drivers-includes.h"



/* Exported types ------------------------------------------------------------*/
enum flash_operation
{
  flash_write_enable,
	flash_write_disable,
	flash_page_program,
	flash_read_sr,
	flash_write_sr,
	flash_power_up,
	flash_power_down,
	flash_chip_erase,
	flash_block_erase,
	flash_sector_erase,
	flash_read_device_id,
	flash_read_unique_id,
	flash_read_data,
	flash_fast_read,
	
	
};

enum flash_state
{
		flash_work=0,
	  flash_sleep

};

/*flash information struct*/
typedef struct 
{
	const u8* name;
	u16 flash_id;      /*indentify diffrent device*/
	u16 page_size;     /*corresponed page size in unit Byte*/
	u8 sector_size;     /*corresponed page size in unit KB*/
	u8 pages_perblock;/*page numbers every block contained*/
	u16 block_num;     /*block num this chip contained*/
}flash_inf_t;

/*flash device struct*/
typedef struct
{
	os_device_t os_device;   /*parent object to manage*/ 
 	u16 flash_id;            /*FLASH  id to identify diffrent flash*/ 
	u16 register_taskid;     /*task_id which this device registered*/
	u8  flash_state;
	const flash_inf_t * flash_info;
}flash_device_t;



/*W25X serial supported id-------------------------------------------------------*/	   

 

/* Exported functions ------------------------------------------------------- */
os_err_t w25qxx_register(u16 task_id);

#endif
















