
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

#define RELAY_NAME  "RELAY"

/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static relay_device *relay_dev=NULL;

/*********************************************************************
 * LOCAL functions
 */

static os_err_t  relays_init      (os_device_t* dev);
static os_err_t relay_open_close  (u8 relay_num,u8 cmd);
static os_err_t  relays_open      (os_device_t* dev, u16 oflag);
static os_err_t  relays_close     (os_device_t* dev);
static os_size_t  relays_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);

static os_err_t  relays_control(os_device_t* dev, u8 cmd, void *args);
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
 os_err_t relays_register(u16 task_id)
{
	 relay_dev=osmalloc(sizeof(key_device));
	
	 relay_dev->os_device.type= OS_Device_Class_Misc;
	
	relay_dev->os_device.device_id=OS_DEVICE_KEY_ID;
	
	relay_dev->register_taskid=task_id;
	
	relay_dev->os_device.init=relays_init;
	
	relay_dev->os_device.open=relays_open;
	
	relay_dev->os_device.write=relays_write;
	
	relay_dev->os_device.control=relays_control;
	
	relay_dev->os_device.close = relays_close;
	
	
	return os_device_register(&(relay_dev->os_device), 	RELAY_NAME, OS_DEVICE_FLAG_INACTIVATED);
	
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


static os_err_t  relays_init   (os_device_t* dev)
{  	 

	
	 os_clock_open("GPIOB");
		GPIOB->CRH&=0XFFFF0FFF; 
		GPIOB->CRH|=0X00003000;//Pb11 ÍÆÍìÊä³ö  
   os_clock_close("GPIOB");
	return SUCCESS;
	

}


	/*********************************************************************
 * @fn      os_err_t relays_open
 *
 * @brief   This function initiate KEY DEVICES INCLUDING PIN A8 B15 
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */


static os_err_t  relays_open   (os_device_t* dev, u16 oflag)
{  	 

	
	os_clock_open("GPIOB");
  return SUCCESS;
	

}


	/*********************************************************************
 * @fn      os_err_t relays_close
 *
 * @brief   This function initiate KEY DEVICES INCLUDING PIN A8 B15 
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */


static os_err_t  relays_close   (os_device_t* dev)
{  	 

	
	 os_clock_close("GPIOB");
  return SUCCESS;
	

}

/*********************************************************************
 * @fn      os_err_t relays_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart1
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  relays_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
{
		
	  
	  if(pos>=size||buffer==NULL||size>USART_TX_BUFFER_SIZE)
		{
			return  NULL;
		}else
		{
			 
		
		}
	  
		
  return SUCCESS;
}	

/*********************************************************************
 * @fn      os_err_t relays_control(os_device_t* dev, u8 cmd, void *args)
 *
 * @brief   This function is write driver for uart1
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_err_t  relays_control(os_device_t* dev, u8 cmd, void *args)
{
   if(dev==NULL)
		{
			return  NULL;
		}else
		{
			 os_device_t *__mptr = (dev);
       relay_device * relay_dev=(relay_device *)( (char *)__mptr - offsetof(usart_device,os_device));

    	 switch(((u8*)args)[1])
			 {
				 case relay1: if(relay_open_close(relay1,cmd)==ERROR) goto fault; relay_dev->relay1_status=cmd; break;
				 case relay2: if(relay_open_close(relay2,cmd)==ERROR) goto fault; relay_dev->relay2_status=cmd;break;
				 case relay3: if(relay_open_close(relay3,cmd)==ERROR) goto fault; relay_dev->relay3_status=cmd;break;
				 case relay4: if(relay_open_close(relay4,cmd)==ERROR) goto fault; relay_dev->relay4_status=cmd;break;
				 default: return ERROR;
			 };
		
		}
    return  SUCCESS;
		fault:
		return ERROR;

}

/*********************************************************************
 * @fn      os_err_t relay_open_close(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart1
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_err_t relay_open_close(u8 relay_num,u8 cmd)
{
		 
	  if(cmd!=relay_opened&&cmd!=relay_closed)
		{
				return ERROR;
		}
	  if(relay_num==relay1)
		 {
		    RELAY_1=cmd;
		 }else if(relay_num==relay2)
		 {
       
		 }	 
			else if(relay_num==relay3)
			{
			
			}
			else if(relay_num==relay4)
			{
			
			}
	return  SUCCESS;
}

/*********************************************************************
 * @fn      _relay_control()
 *
 * @brief   This function expose operation to the shell.
 *
 * @param   void
 *
 * @return  none
 */
os_err_t _relay_control(u8 relay_no,u8 cmd)
{

    relay_open_close(relay_no,cmd);
    return 0;
}
 