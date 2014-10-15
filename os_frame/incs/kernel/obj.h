/***************************************************************
 * Name:      USART.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/



#ifndef _OBJ_H_
#define _OBJ_H_
/*********************************************************************
 * INCLUDES
 */
 
#include "cpu.h"


typedef u32  os_off_t;
typedef u8   os_err_t;
typedef u32  os_size_t;

enum os_object_class_type
			{  
				OS_Object_Class_Thread = 0, 
				
				OS_Object_Class_Clock,
				
				#ifdef OS_USING_SEMAPHORE    
			     OS_Object_Class_Semaphore,                      
   			#endif
				
				#ifdef OS_USING_MUTEX    
					OS_Object_Class_Mutex,       
				#endif
				
				#ifdef OS_USING_EVENT    
					OS_Object_Class_Event,         
			  #endif
				
				#ifdef OS_USING_MAILBOX 
				
				OS_Object_Class_MailBox, 
				
				#endif
				#ifdef OS_USING_MESSAGEQUEUE
				
				OS_Object_Class_MessageQueue,
				
				#endif
				
				#ifdef OS_USING_MEMPOOL 
				OS_Object_Class_MemPool,   
				#endif
				
				#ifdef OS_USING_DEVICE	
				OS_Object_Class_Device,    
				#endif  
				
				OS_Object_Class_Timer, 
				
				#ifdef OS_USING_MODULE
				OS_Object_Class_Module,  	
				#endif    
				
				OS_Object_Class_Unknown, 
				
				OS_Object_Class_Static = 0x80           
			}; 
			

struct os_object
	{
		char       				name[OS_NAME_MAX];//name   
    u8 				 				type;//kernel object type
    u8 				 				flag;//kernel object flasg           
    struct os_object *next;//obejct list
	};			   

/*********************************************************************
 * function decaration
 */
struct os_object * os_object_find(struct os_object *object_dev_list,const char * name);
os_err_t os_object_init(struct os_object **object_list , struct os_object *obj, enum os_object_class_type class_type,  const char *name);	
os_err_t os_object_detach(struct os_object **object_list_header, struct os_object *obj);
	
#endif

