/***************************************************************
 * Name:      USART.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */


#ifndef __CONFIG_H__
#define __CONFIG_H__

#define  OS_NAME_MAX 10
#define  OS_USING_DEVICE 1

#define  SYSCLK      72
#define  PLL_FACTOR (SYSCLK/8)
//usart config part
#define USART_TX_BUFFER_SIZE 64
#define USART_RX_BUFFER_SIZE 64

#define  BOUND  115200

//systick part
#define OS_TICKS_PER_SEC 1000


//device name 
#define USART1_NAME "usart1"
#define KEY_NAME "keys"

//clock manager
#define CLOCK_IN_ROM // determine if the management struct stored in rom or sram

//crc module
#define OS_USING_CRC16 

//#define  _STM32F0XXX_ 
#define  _STM32F1XXX_
#endif  

