#ifndef __SYSTICK_H_
#define __SYSTICK_H_ 
/***************************************************************
 * Name:      systick.h
 * Purpose:   code for systick initiate
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/
#include <stm32f10x.h>


void sys_tick_init(u8 sysclk);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























