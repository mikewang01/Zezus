/***************************************************************
 * Name:      LENTHMEASURE.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "kernel-includes.h"
#include "drivers-includes.h"
/*********************************************************************
* MACROS
*/
#define OS_TICKS_PER_SEC 1000


/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
//static u8  fac_us=0;//us延时倍乘数			   
//static u16 fac_ms=0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数


/*********************************************************************
 * EXTERNAL VARIABLES
 */
/*********************************************************************
 * FUNCTIONS
 */



/*********************************************************************
 * @fn      void sys_tick_init
 *
 * @brief   SYSTICK的时钟固定为HCLK时钟的1/8
 *					SYSCLK:系统时钟
 *					SYSTICK的时钟固定为HCLK时钟的1/8
 * 			    @param SYSCLK system clock from clock, take the union of Mhz 
 *			
 * 
 *   @return none 
 */			   
void LED_Init(void)
{
		RCC->APB2ENR|=1<<4;    //使能PORTC时
		GPIOC->CRH&=0XFF0FFFFF; 
		GPIOC->CRH|=0X00300000;//PC13 推挽输出  
		
		
		RCC->APB2ENR|=1<<3;    //使能PORTB时
		GPIOB->CRH&=0XFFFF0FFF; 
		GPIOB->CRH|=0X00003000;//Pb11 推挽输出  
		
		
		RCC->APB2ENR|=1<<7;    //使能PORTF时钟	   	 
		GPIOF->CRH&=0XFFFFFFF0; 
		GPIOF->CRH|=0X00000003;//PA8 推挽输出   	 
		
		GPIOF->CRL&=0x00ffffff;
		GPIOF->CRL|=0X33000000;
		GPIOF->ODR|=1<<8|1<<7|1<<6;      //PA8 输出高
}






