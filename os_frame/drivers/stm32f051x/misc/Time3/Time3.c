 /***************************************************************
 * Name:      TIMES.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */

 #include "led.h" 
 #include "Time3.h"
 /*********************************************************************
* MACROS
*/



/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */



/*********************************************************************
 * EXTERNAL VARIABLES
 */
/*********************************************************************
 * FUNCTIONS
 */



/*********************************************************************
 * @fn      void sys_tick_init
 *
 * @brief  init time3 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */			   
 void Time3_Init()
{
  RCC->APB1ENR|=1<<1; //TIM13 timer clock enable
  TIM3->CR1|=1<<7;//TIMx_ARR register is buffered
  TIM3->CR1|=1<<4;//Counter used as downcounter
  TIM3->PSC=7200; // 10khz	  
  TIM3->ARR=10;	 //1ms		  不准确还需改正，
  TIM3->EGR|=1;	   //Update generation
  TIM3->DIER|=1;  //Update interrupt enable
  TIM3->CR1|=1<<0; //  Counter enable
  MY_NVIC_Init(0,0,TIM3_IRQn,2);//抢占1，子优先级3，组2
}









