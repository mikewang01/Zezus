
/***************************************************************
 * Name:      MEM.H
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/
#ifndef _OS_H
#define _OS_H
/*********************************************************************
 * INCLUDES
 */
#include "cpu.h"



/* Exported macros --------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/

typedef void (*Task)();
typedef  unsigned char  os_err_t;

typedef struct
{
	u16 State;
	u8  Delay;
	u32 task_tick;
	void *ptr;
} PCB;

/* Exported functions ------------------------------------------------------------*/

void OS_Init(void);

void Shedule(void);
PCB* OS_get_taskstate(void);
u32 OS_get_tick(void);
void os_show_version(void);


#define task_ticks_inc(taskid) do{ \
																		(OS_get_taskstate())[taskid].task_tick++;\
                                 }while(0)


void  exam_assert( char * file_name, unsigned int line_no );


#endif

