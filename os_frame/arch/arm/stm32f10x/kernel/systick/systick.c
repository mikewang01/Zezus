
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
static u8  fac_us=0;//us��ʱ������			   
static u16 fac_ms=0;//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��


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
void sys_tick_init(u8 sysclk)
{

	u32 reload;

 	SysTick->CTRL&=~(1<<2);	//SYSTICKʹ���ⲿʱ��Դ	 
	fac_us=SYSCLK/8;		//�����Ƿ�ʹ��ucos,fac_us����Ҫʹ��
	reload=SYSCLK/8;		//ÿ���ӵļ������� ��λΪK	   
	reload*=1000000/OS_TICKS_PER_SEC;//����OS_TICKS_PER_SEC�趨���ʱ��
							//reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��1.86s����	
	fac_ms=1000/OS_TICKS_PER_SEC;//����ucos������ʱ�����ٵ�λ	   
	SysTick->CTRL|=1<<1;   	//����SYSTICK�ж�
	SysTick->LOAD=reload; 	//ÿ1/OS_TICKS_PER_SEC���ж�һ��	
	SysTick->CTRL|=1<<0;   	//����SYSTICK    

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
	u32 reload=SysTick->LOAD;	//LOAD��ֵ	    	 
	ticks=nus*fac_us; 			//��Ҫ�Ľ�����	  		 
	tcnt=0;
//	OSSchedLock();				//��ֹucos���ȣ���ֹ���us��ʱ
	told=SysTick->VAL;        	//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};
//	OSSchedUnlock();			//����ucos���� 									    
}
//��ʱnms
//nms:Ҫ��ʱ��ms��
void delay_ms(u16 nms)
{	

	delay_us((u32)(nms*1000));	//��ͨ��ʽ��ʱ 
}



extern void systick_process(void);

void SysTick_Handler()
{
		systick_process();
}































