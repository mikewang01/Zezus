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


#ifndef _HAL_TIMER_H
#define _HAL_TIMER_H
/* Includes ------------------------------------------------------------------*/
#include "cpu.h"


/* Exported constants --------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/
typedef struct timer
{
	u8  type; //timer type like alarm,expired clock ...
	u8  task_id;//the owner it belongs to 
	u8  event;  // when time is up, the mcu gonna send this msg to corespond object
	u32 start_time;//the time when this counter established
	u32 destination_time;////the time when this counter ended
	u32 interval;//
	void (*triger_callback)(void);
	struct timer *next;

}stimer;


typedef  struct 
{
	u8 tasid;
	stimer ptimer;//inherit the property from parent
	

}task_timer;

//local MACROS 
#define TIMER_EVENT u16


/*timer types macros
x: taskid
y: message_type
z: detailed event
a: data to transfer
b: data legth
*/
#define timer_send_message(x, z ,a , b)\
				 send_message(x,SYSTEM_EVENT, TIMER_MSG, z ,a , b)
				



void timer_process(void);
u8 os_timer_period(u8 taskid ,TIMER_EVENT event, u32  ticks_expired , void (*triger_callback)(void));
u8 os_timer_expired(u8 taskid ,TIMER_EVENT event, u32  ticks_expired , void (*triger_callback)(void));
u8 del_timer_struct(u8 task_id,TIMER_EVENT event);
#endif

