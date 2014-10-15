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

#define _EXAM_ASSERT_TEST_


typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned  int  u32;

#define  OS_NAME_MAX 10
#define  OS_USING_DEVICE 1

#define  SYSCLK      48
#define  PLL_FACTOR (SYSCLK/12)
//usart config part
#define USART_TX_BUFFER_SIZE 64
#define USART_RX_BUFFER_SIZE 64

#define  BOUND  115200

//systick part
#define OS_TICKS_PER_SEC 1000


//device name 
//#define USART1_NAME "usart1"
#define KEY_NAME "keys"

//clock manager
#define CLOCK_IN_ROM // determine if the management struct stored in rom or sram

//crc module
#define OS_USING_CRC16 


/* console device config*/
#define _DEBUG_OUTPUT_ 1
#define  DEVICE_DEBUG
#if (_DEBUG_OUTPUT_==1)

#define  _CONSOLE_DEVICE_USART1_OPEN
//#define  _CONSOLE_DEVICE_LCD_OPEN

/*debug information outpust swith*/
#define  __DEBUG


#ifdef _EXAM_ASSERT_TEST_   //  ???????

#define   EXAM_ASSERT( condition )\
		    if(condition) \
              NULL;\
		    else   \
				exam_assert( __FILE__, __LINE__ )  
#else   
		
#define EXAM_ASSERT(condition)   NULL 

#define 
				
#endif   /* end of ASSERT */
				
				

#ifndef __DEBUG

#define DEBUG(format,...) 

#else

#define DEBUG(format,...) printf("FILE: "__FILE__", LINE: %d: "format"\r\n", __LINE__, ##__VA_ARGS__)

#endif

 
#endif

				


#define  _STM32F0XXX_ 










#endif  

