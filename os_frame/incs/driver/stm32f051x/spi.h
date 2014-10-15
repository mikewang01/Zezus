/***************************************************************
 * Name:      spi.c
 * Purpose:   code for spi bus
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */


#ifndef _SPI_H
#define _SPI_H
/* Includes ------------------------------------------------------------------*/
#include "cpu.h"
#include "kernel-includes.h"
#include "drivers-includes.h"
/* Exported constants --------------------------------------------------------*/

enum spimode{spi_rx_only=0 , spi_tx_only , spi_full_duplex};
enum SPICLK_DIV{SPICLK_DIV_2 , SPICLK_DIV_4 , SPICLK_DIV_8 , SPICLK_DIV_16 , SPICLK_DIV_32 , SPICLK_DIV_64 , SPICLK_DIV_128 , SPICLK_DIV_256};
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	os_device_t os_device;
	u8 mode;
	u16 register_taskid;
	u32 pclk2 ;
	u32 prescale ;
	void *data;
}spi_device;


				
extern os_err_t spi2_register(u16 task_id);
extern os_err_t spi1_register(u16 task_id);

#endif

