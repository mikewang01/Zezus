
/************************************************************
Copyright (C), 2008-2014, Colorful Sea Tech. Co., Ltd.
FileName:       manufacture_task.c
Author:      MikeWang   Version : 0.0          Date: 2014/8/18
Description:      s task to process the order from the host sofware      
Version:  0.0        
Function List:   



task init function;
   void  task_manufacture_init()

process the message from other device ,thread ...;
   static void     process_msg(u16 taskid , u16 msg_num , smessage* msg);

process the detailed order distributed from host;
    static os_err_t process_order(data_packge package);

1. -------History:         
<author>   <time>    <version >    <desc>
Mike      14/8/18      0.0       build this moudle  
***********************************************************/

/*********************************************************************
 * INCLUDES
 */ 

#include "kernel-includes.h"
#include "drivers-includes.h"
#include "manufacture_task.h"
#include "stdio.h"
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
const u8 mac_adress[] ={0x11 , 0x22 , 0x33 , 0x44 ,0x55 ,0x66 ,0x77 , 0x88};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */
static void     process_msg(u16 taskid , u16 msg_num , smessage* msg);
static os_err_t process_order(data_packge package);

/*********************************************************************
 * @fn      task_demo_init()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void  task_manufacture_init()
{
    u8 this_task_id=Current_Task;
	  eeprom_register(this_task_id);
    os_device_init_all();
    
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
void task_manufacture()
{
    
    u8 My_Id=Current_Task;
    PCB* task_state=0; 
	  /*get self PCB to lead a further process */
    get_self_taskstate(My_Id,task_state);
    
    // printf("taskid = %d" , My_Id );
    /*it actully get some message recieve and process it*/
    if(((task_state->State)&SYSTEM_EVENT)==SYSTEM_EVENT)
    {
        
        smessage* msg_get;
			
        /*get detailed message through PCB*/
        u8 msg_num=get_message(task_state , &msg_get);
       // printf("get message %d", msg_num);
			
        /*process the message has been recieved*/
        process_msg(My_Id, msg_num , msg_get);
    }
    
    /*delete messge recieved or memmory leakage gonna happen*/
    delete_message(My_Id,SYSTEM_EVENT);
    
    /*statistic the the oppupation of cpu*/
    task_ticks_inc(My_Id);
}



/*********************************************************************
 * @fn      process_msg()
 *
 * @brief   process the ,message this task has received.
 *
 * @param   u16    taskid  ;task_id of task calling this function
 *          u16   msg_num  ;total message number recieved
 *          smessage* msg  ;recieved message ptr
 *
 * @return  none
 */

static void process_msg(u16 taskid , u16 msg_num , smessage* msg)
{
    do{
        
        switch(msg->type)
        {
        /* inter process communication message*/
        case IPC_MSG: 
				 {
            // u8 buffer[10]={1,2,3,4,5,6,7,8};
					   os_device_t *dev = os_device_get("EEPROM");
						 /*device open fialed  return NULL*/
						 if(dev == NULL)
						 {
							 printf("open failed");
						 }
						 
					  os_device_open(dev, 1); //make sure  usart1 
          //  os_device_write(dev, 2 , buffer , 8);
					//   os_device_read(dev , 0 , buffer , 9);
				//		 os_device_close(dev);
				//		 printf("%s" , buffer);
						 
            /*CALIBRATION_COMMAND  event ,process it*/
            if((msg->event&CALIBRATION_COMMAND) == CALIBRATION_COMMAND )
            {
                
                data_packge *calibration_data=(data_packge*)msg->ptr;
                
                process_order(*calibration_data);
             //   printf("order = %d" , calibration_data->order );
            }
         }break;
				 
        /*timer related message*/
        case TIMER_MSG: 
            if((msg->event&TIMEOUT_500MS)==TIMEOUT_500MS)
            {
                
                
                
            }else if((msg->event&TIMEOUT_250MS)==TIMEOUT_250MS)
            {
                
                
            }
            else if((msg->event&TIMEOUT_125MS)==TIMEOUT_125MS)
            {
                
                
            }					 
            else if((msg->event&TIMEOUT_RELAY1)==TIMEOUT_RELAY1)
            {
                
            }		
            
            
            break;
            
        /*device related message*/
        case DEV_MSG: 
            
            
            break;
        }
        
        /*get next message from message queue*/
        msg=msg->next;
    }while((--msg_num)>0);
}


/*********************************************************************
 * @fn      void process_order(data_packge package)
 *
 * @brief   This function mainly process the order distributed by host
 *
 * @param   void
 *
 * @return  none
 */

extern u16 sensor_get_voltage(void);

static os_err_t process_order(data_packge package)
{
    os_device_t *dev_serial = os_device_get("USART1");
    
 
     /*if opened failed return error*/
		EXAM_ASSERT( (dev_serial!=NULL) );
    
    /*order of get_mac adress*/
    if(package.order == get_mac)
    {
        
			 
				
		  #ifdef _TASK_DEBUG_
			    printf("%d [%s]:%d",package.order , __FILE__ , __LINE__ );
			#endif
			
			 // os_device_control(dev, set_correction_para , temp);
			
			  /*get payload length*/
        package.payload.lenth = sizeof(uid_96bis);//sizeof(mac_adress);
        
			
			  /*copy corresponed mac adress to pay load */
        os_memmove(package.payload.pay_load , (uid_96bis*)(UID_BASE_ADRESS) ,  package.payload.lenth);
			
        /*send the response data*/
        return os_device_write(dev_serial , 0 , (u8*)(&package) , sizeof(data_packge)); 
        
    }else if(package.order == get_vol) /*order of get_vol to get curent volatge cpu sampled*/
    {
        /*set payload lenth*/
        package.payload.lenth = sizeof(u16);
        
        /*fullfill the corresponed data area*/
        *((u16*)package.payload.pay_load) = sensor_get_voltage();
        
        /*send  response data*/
        return os_device_write(dev_serial , 0 , (u8*)(&package) , sizeof(data_packge));
			
    }else if(package.order == get_dsy)/*order of get_dsy to get curent density cpu sampled*/
		{
			   u16 density=0;
			   
			  
			   os_device_t *dev = os_device_get("me2_ch2o");
			
			#ifdef _TASK_DEBUG_
			    printf("%d [%s]:%d",package.order , __FILE__ , __LINE__ );
			#endif
			
			    /*if opened fialed return error*/
			    EXAM_ASSERT( (dev!=NULL) );
      
				 /*get data lenth*/
				 package.payload.lenth = sizeof(u16);
				 
		     /*read data from me_ch2o device*/ 	
          os_device_read(dev , 0 , (void*)&density , package.payload.lenth);
				 
			   /*copy corresponed mac adress to pay load */
         os_memmove(package.payload.pay_load , (void*)&density , package.payload.lenth);
				 
				 	/*save new parameter to eeprom*/
//			   os_device_control(dev, update_correction_sensor , NULL);
				 
				  /*send  coresponded data*/
         return os_device_write(dev_serial , 0 , (u8*)(&package) , sizeof(data_packge));
				 
		}else if(package.order == set_para) /*set_parameter order distributed by host software*/
		{
			  
			  u8 argument[sizeof(correction_para)+2] = {0};
			  os_device_t *dev = os_device_get("me2_ch2o");
		
	      
			  /*if opened failed return error*/
			  EXAM_ASSERT( (dev!=NULL) );
				/*size of the pakage*/
			  argument[0] = package.payload.lenth;
			  os_memmove((argument+1) , (void*)&package.payload.pay_load , package.payload.lenth);
			  
				/*save new parameter to eeprom*/
			  os_device_control(dev, set_correction_para , argument);
				
			  /*send  the same data to response*/
        return os_device_write(dev_serial , 0 , (u8*)(&package) , sizeof(data_packge));
        
    }else if(package.order == get_para) /*get parameter from device*/
    {
			  u8 argument[sizeof(correction_para)+2] = {0};
				os_device_t *dev = os_device_get("me2_ch2o");
				 /*if opened failed return error*/
			  EXAM_ASSERT( (dev!=NULL) );
				
				/*save new parameter to eeprom*/
			  os_device_control(dev, get_correction_para , argument);
				
				/*copy the data lenth*/
				package.payload.lenth = argument[0];
				/*cpoy the detailed data  from  buffuer*/
				os_memmove((void*)&package.payload.pay_load , argument+1 , package.payload.lenth );
				
			  /*send  the same data to response*/
        return os_device_write(dev_serial , 0 , (u8*)(&package) , sizeof(data_packge));
			  
    }else if(package.order == get_hard_version) /*get hardware version*/
		{
				
		}
		else if(package.order == get_soft_version) /*get sorfware versiom*/
		{
      u32 version;
			/*get detailed version by macros*/
			version = GET_VERSION();
			
			package.payload.lenth = sizeof(u32);
			/*copy data from buffer to package payload*/
			os_memmove( (void*)&package.payload.pay_load , &version , package.payload.lenth );
			
			/*send  corresponded data to host*/
      return os_device_write(dev_serial , 0 , (u8*)(&package) , sizeof(data_packge));
			
    }
    return ERROR;
}


