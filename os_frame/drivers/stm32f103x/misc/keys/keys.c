
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
#include <stm32f10x.h>
 
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
static key_trigger key_trigger_status={0};
static key_device *key_dev=NULL;

/*********************************************************************
 * LOCAL functions
 */

static os_err_t  keys_init   (os_device_t* dev);

	/*********************************************************************
 * @fn      os_err_t os_device_init_all
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
 os_err_t keys_register(u16 task_id)
{
	key_dev=osmalloc(sizeof(key_device));
	

	key_dev->os_device.type= OS_Device_Class_Misc;
	
	key_dev->os_device.device_id=OS_DEVICE_KEY_ID;
	
	key_dev->register_taskid=task_id;
	
	key_dev->os_device.init=keys_init;
	
	key_dev->os_device.open=NULL;
	
	key_dev->os_device.write=NULL;
	
	
	return os_device_register(&(key_dev->os_device), KEY_NAME, OS_DEVICE_FLAG_INACTIVATED);
	
}

 
//end
//////////////////////////////////////////////////////////////////

	/*********************************************************************
 * @fn      os_err_t keys_init
 *
 * @brief   This function initiate KEY DEVICES INCLUDING PIN A8 B15 
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */


static os_err_t  keys_init   (os_device_t* dev)
{  	 

	RCC->APB2ENR |=1<<2|1<<3;     //enbale PORTA  PAORB CLOCK  
	GPIOA->CRH   &=0XFFFFFFF0;//PA8
	GPIOA->CRH   |=0X00000008;   
	GPIOB->CRH   &=0X0FFFFFFF;//pb15	  
	GPIOB->CRH   |=0X80000000; 				   
	GPIOA->ODR   |=1<<8;	   //pull up
	GPIOB->ODR   |=1<<15;	   //PB15 pull up
  key_trigger_status.key1_trigger=FTIR;
  key_trigger_status.key2_trigger=FTIR;
//	Ex_NVIC_Config(GPIO_A,0, RTIR); //?????
	Ex_NVIC_Config(GPIO_A, 8 , key_trigger_status.key1_trigger);//?????
	Ex_NVIC_Config(GPIO_B, 15,key_trigger_status.key2_trigger);//?????

	MY_NVIC_Init(2,3,EXTI9_5_IRQn,2);//??2,????1,?2
	MY_NVIC_Init(2,3,EXTI15_10_IRQn,2);//??2,????1,?2
	
	//MY_NVIC_Init(0,3,USART1_IRQn,2);//组2，最低优先级 
  return SUCCESS;
	

}



static u16 count=0;
void EXTI9_5_IRQHandler(void)
{
	
	key_data key_status={0};
  key_status.key=key1;
	if(key_trigger_status.key1_trigger==FTIR)
	{
	   key_trigger_status.key1_trigger=RTIR;
			
		 key_status.status=key_down;
	}else
	{
			key_trigger_status.key1_trigger=FTIR;
		  key_status.status=key_released;
	}
	keys_send_message(key_dev->register_taskid,&key_status,sizeof(key_data));
	Ex_NVIC_Config(GPIO_A,8,key_trigger_status.key1_trigger);
	count++;
	EXTI->PR=1<<8;   
	
}
void EXTI15_10_IRQHandler(void)
{	
	key_data key_status={0};
  key_status.key=key2;
  if(key_trigger_status.key2_trigger==FTIR)
	{
	 key_trigger_status.key2_trigger=RTIR;
		 key_status.status=key_down;
	}else
	{
			key_trigger_status.key2_trigger=FTIR;
			key_status.status=key_released;

	}
	keys_send_message(key_dev->register_taskid,&key_status,sizeof(key_data));
	Ex_NVIC_Config(GPIO_B,15,key_trigger_status.key2_trigger);
	
	
	EXTI->PR=1<<15;     //??LINE15???????  
}

//PWR_EnterSTOPMode


