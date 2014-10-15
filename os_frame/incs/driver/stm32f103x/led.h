#ifndef __LED_H
#define __LED_H	 
#include "corefunc.h"
//Mini STM32开发板
//LED驱动代码			 
//正点原子@ALIENTEK
//2010/5/27

//LED端口定义
#define LED1 PFout(6)// Pf6
#define LED2 PFout(7)// Pf6	
#define LED3 PBout(11)// Pf6
#define LED4 PCout(13)
void LED_Init(void);//初始化		 				    
#endif

















