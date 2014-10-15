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
static stimer * delay_header=NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */

u8 add_timer_struct(u8 task_id, u8 timer_types , TIMER_EVENT event , u32 interval, void (*triger_callback)(void))
{
    
    stimer *p_stimer=(stimer*)osmalloc(sizeof(stimer));
    if(p_stimer==NULL)
    {
        goto fault;
    }
    p_stimer->task_id=task_id;
    p_stimer->type=timer_types; 
    p_stimer->event=event; 
    p_stimer->triger_callback=triger_callback;
    
    p_stimer->interval=interval;
    p_stimer->start_time=OS_get_tick();
    p_stimer->destination_time=p_stimer->start_time+ p_stimer->interval;
    p_stimer->next=NULL;
    
    if(p_stimer==NULL)
    {
        goto fault;
    }
    
    if(delay_header==NULL)
    {
        delay_header=p_stimer;
    }
    else
    {
        stimer *stimer_i=delay_header;
        while(stimer_i->next!=NULL)
        {
            stimer_i=stimer_i->next;
        }
        stimer_i->next=p_stimer;
    }
    
    return SUCCESS;
fault:
    return ERROR;
}

/*********************************************************************
 * @fn      del_struct
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */

u8 del_struct(stimer *stimer_prev , stimer *stimer_i)
{
    if(stimer_i==NULL)//this means ,only one timer attached to delay_header
    {
        delay_header=NULL;
        return osfree(stimer_i);
    }
    
    if(stimer_prev==NULL)
    {
        delay_header=stimer_i->next;
    }else
    {
        stimer_prev->next=stimer_i->next;
    }
    return osfree(stimer_i);
    
}
/*********************************************************************
 * @fn      del_timer_struct
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
u8 del_timer_struct(u8 task_id,TIMER_EVENT event)
{
    stimer *stimer_i=NULL;
    stimer *stimer_prev=NULL;
    if(delay_header==NULL)
    {
        goto fault;
    }
    stimer_i=delay_header;
    while(stimer_i!=NULL)
    {
        if(stimer_i->task_id==task_id)
        {
            if(stimer_i->event==event)
            {
                return del_struct(stimer_prev , stimer_i);
                
            }
        }
        stimer_prev=stimer_i;
        stimer_i=stimer_i->next;
    }
    
fault:
    return ERROR;
}
/*********************************************************************
 * @fn      os_timer_expired
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
u8 os_timer_expired(u8 taskid ,TIMER_EVENT event, u32  ticks_expired , void (*triger_callback)(void))
{
    stimer *p_stimer=delay_header;
    while(p_stimer!=NULL)
    {
        if(p_stimer->task_id==taskid)
        {
            if(p_stimer->event==event)
            {
                return SUCCESS;
                
            }
        }
        p_stimer=p_stimer->next;
    }
    return add_timer_struct(taskid, TIMER_EXPIRED , event ,ticks_expired, (*triger_callback));
}

/*********************************************************************
 * @fn      os_timer_period
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
u8 os_timer_period(u8 taskid ,TIMER_EVENT event, u32  ticks_expired , void (*triger_callback)(void))
{
    stimer *p_stimer=delay_header;
    while(p_stimer!=NULL)
    {
        if(p_stimer->task_id==taskid)
        {
            if(p_stimer->event==event)
            {
                return SUCCESS;
                
            }
        }
        p_stimer=p_stimer->next;
    }
    return add_timer_struct(taskid, TIMER_PERIOD , event ,ticks_expired, (*triger_callback));
    
}

/*********************************************************************
 * @fn      timer_process()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */

void timer_process()
{
    stimer *p_stimer=delay_header;
    stimer *stimer_prev=NULL;
    if(p_stimer==NULL)
    {
        return;
    }
    while(p_stimer!=NULL)
    {
        p_stimer->start_time++;
        if(p_stimer->start_time>=p_stimer->destination_time)
        {
            if(p_stimer->triger_callback!=NULL)
            {
                p_stimer->triger_callback();
            }
            //#ifdef   _DEBUG_
            if(timer_send_message( p_stimer->task_id , p_stimer->event, NULL,0)==ERROR)
            {
                
            }else
            {
                
            }
            //#endif
            if(p_stimer->type==TIMER_EXPIRED)
            {
                del_struct(stimer_prev , p_stimer);
                return;
            }
            else if(p_stimer->type==TIMER_PERIOD)
            {
                p_stimer->start_time=OS_get_tick();
                p_stimer->destination_time=p_stimer->start_time+p_stimer->interval;
            }
        }
        stimer_prev=p_stimer;
        p_stimer=p_stimer->next;
    }
    
}

/*********************************************************************
 * @fn      _list_timer()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
os_err_t _list_timer()
{
    
    stimer *timer_list=delay_header;
    if(timer_list==NULL)
    {
        return ERROR;	
    }
    printf("\r\ntaskid   periodic   timeout    flag\r\n");
    printf("-------- ---------- ---------- -----------\r\n");
    for (timer_list = timer_list->next; timer_list != NULL; timer_list = timer_list->next)
    {
        
        printf("0x%08x 0x%08x 0x%08x ",
               timer_list->task_id,
               timer_list->interval,
               timer_list->start_time+timer_list->interval);
        if (timer_list->type & TIMER_PERIOD)
            printf("timer_interval\r\n");
        else
            printf("timer_exipred\r\n");
    }
    
    printf("current tick:0x%08x\r\n", OS_get_tick());
    
    return 0;
}


