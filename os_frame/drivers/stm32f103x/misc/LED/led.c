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
//static u8  fac_us=0;//us��ʱ������			   
//static u16 fac_ms=0;//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��


/*********************************************************************
 * EXTERNAL VARIABLES
 */
/*********************************************************************
 * FUNCTIONS
 */



/*********************************************************************
 * @fn      void sys_tick_init
 *
 * @brief   SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
 *					SYSCLK:ϵͳʱ��
 *					SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
 * 			    @param SYSCLK system clock from clock, take the union of Mhz 
 *			
 * 
 *   @return none 
 */			   
void LED_Init(void)
{
		RCC->APB2ENR|=1<<4;    //ʹ��PORTCʱ
		GPIOC->CRH&=0XFF0FFFFF; 
		GPIOC->CRH|=0X00300000;//PC13 �������  
		
		
		RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ
		GPIOB->CRH&=0XFFFF0FFF; 
		GPIOB->CRH|=0X00003000;//Pb11 �������  
		
		
		RCC->APB2ENR|=1<<7;    //ʹ��PORTFʱ��	   	 
		GPIOF->CRH&=0XFFFFFFF0; 
		GPIOF->CRH|=0X00000003;//PA8 �������   	 
		
		GPIOF->CRL&=0x00ffffff;
		GPIOF->CRL|=0X33000000;
		GPIOF->ODR|=1<<8|1<<7|1<<6;      //PA8 �����
}






