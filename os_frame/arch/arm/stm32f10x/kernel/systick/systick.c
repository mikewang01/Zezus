
/***************************************************************
 * Name:      systick.c
 * Purpose:   code for systick initiate
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
* includes
*/
#include "kernel-includes.h"
#include "drivers-includes.h"


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
static u8  fac_us=0;//us延时倍乘数			   
static u16 fac_ms=0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数


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
void sys_tick_init(u8 sysclk)
{

	u32 reload;

 	SysTick->CTRL&=~(1<<2);	//SYSTICK使用外部时钟源	 
	fac_us=SYSCLK/8;		//不论是否使用ucos,fac_us都需要使用
	reload=SYSCLK/8;		//每秒钟的计数次数 单位为K	   
	reload*=1000000/OS_TICKS_PER_SEC;//根据OS_TICKS_PER_SEC设定溢出时间
							//reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右	
	fac_ms=1000/OS_TICKS_PER_SEC;//代表ucos可以延时的最少单位	   
	SysTick->CTRL|=1<<1;   	//开启SYSTICK中断
	SysTick->LOAD=reload; 	//每1/OS_TICKS_PER_SEC秒中断一次	
	SysTick->CTRL|=1<<0;   	//开启SYSTICK    

}								    


/*********************************************************************
 * @fn      void delay_us
 *
 * @brief   this function is mainly used to obtain an accurate counter
 * 			    
						@param SYSCLK system clock from clock, take the union of Mhz 
 *			
 * 
 *   @return none 
 */	    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;	//LOAD的值	    	 
	ticks=nus*fac_us; 			//需要的节拍数	  		 
	tcnt=0;
//	OSSchedLock();				//阻止ucos调度，防止打断us延时
	told=SysTick->VAL;        	//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;//时间超过/等于要延迟的时间,则退出.
		}  
	};
//	OSSchedUnlock();			//开启ucos调度 									    
}
//延时nms
//nms:要延时的ms数
void delay_ms(u16 nms)
{	

	delay_us((u32)(nms*1000));	//普通方式延时 
}



extern void systick_process(void);

void SysTick_Handler()
{
		systick_process();
}































