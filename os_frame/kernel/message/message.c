/***************************************************************
 * Name:      message.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "kernel-includes.h"




/*********************************************************************
 * @fn      get_message
 *
 * @brief   This function is used for task to get message and message number.
 *
 * @param   task_state pointer ,msg pointer
 *
 * @return  message numbers
 */
os_err_t get_message(PCB* task_state , smessage** msg)
{
    u8 msg_num=0;
    /*get cuurent message ptr from task PCB*/	
    *msg=	(smessage*)task_state->ptr;
	
    /*if no message existed, return 0*/	
    if(msg==NULL)
    {
        return 0;
    }
    
		/*statistic  message num that hook in PCB*/
    msg_num++;
    while((*msg)->next!=NULL)
    {
        msg_num++;
        (*msg)=(*msg)->next;
    }
    
    /*copy message ptr to uer pointer */
    *msg=(smessage*)task_state->ptr;
    
    
    return msg_num;
}

/*********************************************************************
 * @fn      delete_message
 *
 * @brief   This function is used for task detele message after used.
 *
 * @param   task_id,message_type.
 *
 * @return  SUCCESS ,ERROR
 */
enum{locked=0,released};
static  u8 message_lock=released;
os_err_t delete_message(u8 task_id,u8 message_type)
{
    PCB * ptaskstate=NULL;
    smessage* msg=NULL;
	
	  /*get task state by task_id*/
    GET_TASKSATE_BYID(task_id,ptaskstate);
	
	  /*get message ptr*/
    msg=(smessage*)ptaskstate->ptr;
    
	  /*wait until other thread release this lock*/
    while(message_lock==locked);	
	
	  /*lock up message*/
    message_lock=locked;
      
    ptaskstate->ptr=NULL;
	  
	  /*remove state mark in case making thread running again*/
    ptaskstate->State^=message_type;
	
    while(msg!=NULL)
    {
        smessage* temp=msg;
        msg=msg->next;
        if(temp->ptr!=NULL)
        {
           /*free message memory taht has been occupied*/ 
					 if(osfree((void*)(temp->ptr))==ERROR)
            {
                goto fault;
            }
        }
				/*erroe happens return fault*/
        if(osfree((void*)(temp))==ERROR)
        {
            goto fault;
        }
        
    }
    
    /*release message lock*/
    message_lock=released;
    
    return SUCCESS;	 
fault:
    message_lock=released;
    return ERROR;
}




/*********************************************************************
 * @fn      send_message
 *
 * @brief   This function is used for sending messge to specific task.
 *
 * @param   task_id,message_type.
 *
 * @return  SUCCESS ,ERROR
 */

os_err_t send_message(u8 task_id , u16 pcb_type , u16 message_type , u16 event,void *data, u16 length)
{
    smessage* msg=(smessage*)osmalloc(sizeof(smessage));
    u8 *data_temp=NULL;
    
    smessage* msg_tmp=NULL;
    PCB *  taskstate_tmp=NULL;
    u16 circle=0;
    if(message_lock==locked)
    {
        return ERROR;
    }
    message_lock=locked;
    
    /*copy data in buffer into newly allocated space*/
    if(length!=0)
    {
        data_temp=(u8*)osmalloc(sizeof(u8)*length);
        osmemcpy(data_temp ,data , length);
    }
    /*if there is no message return erroe*/
    if(msg==NULL)
    {
        goto fault;
    }	
    
    osmemset(msg,0,sizeof(smessage));
		
		/*set event group type*/
    msg->type= message_type; 
		/*set message length*/
    msg->length= length;
		/*set user data ptr*/
    msg->ptr=data_temp;
    /*set specific event*/
    msg->event=event;
		/*set next ptr to NULL*/
    msg->next=NULL;
    
    
		/*get the task PCB corresponded to task_id*/
    taskstate_tmp=OS_get_taskstate();
		/*set task to run */
    taskstate_tmp[task_id].State|=pcb_type;
    
		/*if this is the fist message*/
    if(taskstate_tmp[task_id].ptr==NULL)
    {
        taskstate_tmp[task_id].ptr=(void*)msg;
    }
    else /*there are some message existed in PCB ptr area*/
    {
        msg_tmp=(smessage*)taskstate_tmp[task_id].ptr;
        
        /*find the end of the message chain*/
        while(msg_tmp->next!=NULL)
        {
            msg_tmp=msg_tmp->next;
					/*in case it is a dead loop*/
            circle++;
            if(circle>30)
            {
                break;
            }
        }
        
        
        msg_tmp->next=msg;
    }
		
		/*release message lock*/
    message_lock=released;
    return SUCCESS;
    
fault:
    message_lock=released;
    return ERROR;
}



