/***************************************************************
 * Name:      usertask.c
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
#include "usertask.h"
#include "stdio.h"
#include "gpio_mgr.h"
#include "w25qxx.h"
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
 * @fn      Task_Led()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void call_back_test()
{
    
}
/*********************************************************************
 * @fn      Task_Led()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
enum unit{unit_us=0,unit_ms,unit_s};
struct relay_order
{
    u8  unit;
    u8  relay_num;
    u16 time_open;
    u16 crc;
};

void process_event(u16 taskid , u16 msg_num , smessage* msg)
{
    //	  os_device_t *usart1_dev= os_device_get("usart1");
    //		if(usart1_dev==NULL)
    //		{
    //				return ;
    //		}
    //	 
    do{
        switch(msg->type)
        {
        case IPC_MSG: 
            if(((msg->event)&START_TICK)==START_TICK)
            {
                //	 os_device_open(usart1_dev, 1); //make sure  usart1 
                //os_device_write(dev,0,"fuckyou",10);
                // u8 taskid ,TIMER_EVENT event, u32  ticks_expired , void (*triger_callback
                os_timer_period(taskid,TIMEOUT_500MS,500,NULL);
                //  os_timer_period(taskid,TIMEOUT_250MS,250,NULL);
                os_timer_period(taskid,TIMEOUT_125MS,125,NULL);
                //				 os_timer_period(taskid,TIMEOUT_100MS,100,NULL);
            } 
            break;  
        case TIMER_MSG: 
            if((msg->event&TIMEOUT_500MS)==TIMEOUT_500MS)
            {
                
                u8 temp[8]={0,0};
                u8 led2[2]={0 , led_1};
                os_device_t *dev = os_device_get("me2_ch2o");
                temp[0]=0X01;
                //os_device_control(dev , 	WRITE_USER_REGISTER ,  temp    );
                os_device_read(dev , 0 , temp , 2);
                //	 os_device_control(dev , TRIGGER_HUMI_MEASUREMENT,     temp );
                //	 os_device_control(dev , TRIGGER_TEMP_MEASUREMENT , temp );
                //htu20_read_temper(0x40 , temp , 3);
                
                //	 printf("%d %d \r\n",((htu20_data*)temp)->temper ,((htu20_data*)temp)->humidity);
                //
                
                
                
                
                	  printf("dty=%02dug/m3\r\n",*((u16*)temp));
                // temp[1]+=1;
                
                leds_control((os_device_t*)1, led_trigger, led2);
            }else if((msg->event&TIMEOUT_250MS)==TIMEOUT_250MS)
            {
                
                
            }
            else if((msg->event&TIMEOUT_125MS)==TIMEOUT_125MS)
            {
                static	 u8 temp[2]={0,0x00}; static char a='0';
                // leds_control((os_device_t*)1, led_trigger, temp);
                //os_device_t *dev= os_device_get("LCD");
                //temp[1]+=1;
                //printf("how are %d\r\n",temp[1]);
                
            }					 
            else if((msg->event&TIMEOUT_RELAY1)==TIMEOUT_RELAY1)
            {
                u8 arg[2]={1,relay1};
                os_device_t *dev= os_device_get("RELAY");
                os_device_open(dev,0);
                os_device_control(dev,relay_closed,arg);
                os_device_close(dev);
                //os_device_open(usart1_dev, 1); //make sure  usart1 
            }		
            
            
            break;
        case DEV_MSG: 
            if((msg->event&USART1_RX)==USART1_RX)
            {
                
                usart_data *usart1_data=(usart_data*)msg->ptr;
                struct relay_order *relay_data=(struct relay_order*)usart1_data->data;
                
                if(relay_data->relay_num==relay1)
                {
                    extern u8 crc16_on_off;
                    u8 arg[2]={1,relay1};
#ifdef  OS_USING_CRC16
                    u16 crc=crc16((u8*)relay_data,sizeof(struct relay_order)-sizeof(u16));
                    if(crc==relay_data->crc||crc16_on_off)
#endif
                    {
                        os_device_t *dev= os_device_get("RELAY");
                        os_device_open(dev,0);
                        os_device_control(dev,relay_opened,arg);
                        //relay_data->time_open
                        os_timer_expired(taskid,TIMEOUT_RELAY1,relay_data->time_open,NULL);
                    }
                }
                
                
                //										 os_device_write(usart1_dev,0,usart1_data->data,usart1_data->length);
                //                   os_device_close(usart1_dev);
                
            }
            if((msg->event&KEY_STATUS_CHANGED)==KEY_STATUS_CHANGED)
            {
                key_data *key_status=(key_data*)msg->ptr;	
                u8 arg[2]={1,relay1};
                os_device_t *dev= os_device_get("RELAY");
                os_device_open(dev,0);
                
                if(key_status->key==key1)
                {
                    if((key_status->status)==key_down)
                    {
                        os_device_control(dev,relay_opened,arg);
                        
                    }
                    else if((key_status->status)==key_released)
                    {
                        os_device_control(dev,relay_closed,arg);
                    }
                    
                }
                if(key_status->key==key2)
                {
                    if((key_status->status)==key_down)
                    {
                        os_device_control(dev,relay_opened,arg);
                    }
                    else if((key_status->status)==key_released)
                    {
                        os_device_control(dev,relay_closed,arg);
                    }
                    
                }
                os_device_close(dev);
            }
            break;
        }
        msg=msg->next;
    }
    while((--msg_num)>0);
}


/*********************************************************************
 * @fn      task_demo_init()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */

extern os_err_t usart1_register(u16 task_id);
extern os_err_t lcd_register(u16 task_id);
extern os_err_t me2_ch20_register(u16 task_id);
os_err_t htu20_register(u16 task_id);
void  task_demo_init()
{
    u8 this_task_id=Current_Task;
	  os_gpio_init();
    led_init();
    spi2_register(this_task_id);
    spi1_register(this_task_id);
    me2_ch20_register(this_task_id);
    htu20_register(this_task_id);
    
    // usart_register(this_task_id);
    //	  keys_register(this_task_id);
    usart1_register(this_task_id);
    //	   relays_register(this_task_id);
    lcd_register(this_task_id);
    w25qxx_register(this_task_id);
    os_device_init_all();
    // 
    os_show_version();
    //printf("how are %f\r\n",22.2);
    send_message(this_task_id,SYSTEM_EVENT,IPC_MSG,START_TICK,NULL,(u16)0);
}


/*********************************************************************
 * @fn      Task_Led()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void task_demo()
{
    
    u8 My_Id=Current_Task;
    PCB* task_state=0; 
    get_self_taskstate(My_Id,task_state);
    
    if(((task_state->State)&SYSTEM_EVENT)==SYSTEM_EVENT)
    {
        
        smessage* msg_get;
        u8 msg_num=get_message(task_state , &msg_get);
        process_event(My_Id, msg_num , msg_get);
        /*
                do something
        */
        
        // delete_message(My_Id,SYSTEM_EVENT);
    }
    
    delete_message(My_Id,SYSTEM_EVENT);
    task_ticks_inc(My_Id);
}




