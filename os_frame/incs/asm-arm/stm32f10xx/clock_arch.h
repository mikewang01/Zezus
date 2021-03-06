
#ifndef __CLOCK_ARCH_H_
#define __CLOCK_ARCH_H_ 
/***************************************************************
 * Name:      systick.h
 * Purpose:   code for systick initiate
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/
#ifdef _STM32F1XXX_

#include <stm32f10x.h>

#endif

#ifdef _STM32F0XXX_

#include <stm32f0xx.h>

#endif


#ifdef CLOCK_IN_ROM


typedef struct 
{
  u8 status:2;
  u8 last_status:2;
}status_record;
			
typedef struct 
{
  status_record gpioa;
	status_record gpiob;
	status_record gpioc;
	status_record gpiod;
	status_record gpioe;
	status_record usart1;
	status_record usart2;
	status_record afio;
	status_record adc1;
	status_record adc2;
	status_record spi1;
	status_record tim1;
	status_record tim2;
	status_record dma1;
} clock_status_record;
#endif

#endif





























