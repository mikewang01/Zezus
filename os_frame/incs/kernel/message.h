#ifndef __MESSAGE_H
#define __MESSAGE_H


#include "cpu.h"

#include "OS.H"

//message struct 
typedef struct msg
{
	u8 type;
	u32 event;
	u16 length;
	void * ptr;
	struct msg * next;
}smessage,*psmessage;

os_err_t send_message(u8 task_id,u16 pcb_type,u16 message_type , u16 event,void *data, u16 length);
os_err_t get_message(PCB* task_state , smessage** msg);
os_err_t delete_message(u8 task_id,u8 message_type);

//define message type here
#define TIME_EXPIRED 0X01


#define GET_TASKSATE_BYID(TASKID,PTASKSTATE)  \
		do{ \
		  PTASKSTATE=OS_get_taskstate();\
		  PTASKSTATE=PTASKSTATE+TASKID;\
		}while(0)


	

#endif

