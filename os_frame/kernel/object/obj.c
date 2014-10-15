/***************************************************************
 * Name:      obj.c
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
#include "string.h"

// ***************************************************************************
// ******************** START OF global variable DECLARATIONS *******************
// ***************************************************************************




/***************************************************************************
 ***************************************************************************
 **************** START OF PRIVATE  PROCEDURE IMPLEMENTATIONS **************
 ***************************************************************************/
/*********************************************************************
 * @fn      OS_err_t os_object_detach
 *
 * @brief   This function registers a device driver with specified name
 *
 * 			    @param dev the pointer of device driver structure 
 *					@param name the device driver's name
 *					@param flags the flag of device
 * 
 *   @return the error code, OS_EOK on initialization successfully. 
 */
struct os_object * os_object_find(struct os_object *object_dev_list,const char * name)
{
    struct os_object *obj=object_dev_list;
    
    if(obj==NULL)
    {
        return NULL; //no device in list
    }
    
    while(obj!=NULL)
    {
        if(strcmp(name,obj->name)==0)
            return obj;
        
        obj=obj->next;
        
        
    }
    return NULL;
}
/*********************************************************************
 * @fn      OS_err_t os_object_detach
 *
 * @brief   This function registers a device driver with specified name
 *
 * 			    @param dev the pointer of device driver structure 
 *					@param name the device driver's name
 *					@param flags the flag of device
 * 
 *   @return the error code, OS_EOK on initialization successfully. 
 */
os_err_t os_object_init(struct os_object **object_list , struct os_object *obj, enum os_object_class_type class_type ,const char *name)
{
    if(obj==NULL||name==NULL)
    {
        return ERROR;
    }
    
    if((sizeof(name))>OS_NAME_MAX)
    {
        return ERROR;
    }
    strcpy(obj->name , name);
    obj->type=class_type;
    obj->next=NULL;
    
    if(*object_list==NULL)
    {
        *object_list=obj;
    }else
    {
        struct os_object *object_temp=*object_list;
        while((object_temp)->next!=NULL)
        {
            (object_temp)=(object_temp)->next;
        }
        
        (object_temp)->next=obj;
    }
    return SUCCESS;
}

/*********************************************************************
 * @fn      OS_err_t os_object_detach
 *
 * @brief   This function registers a device driver with specified name
 *
 * 			    @param dev the pointer of device driver structure 
 *					@param name the device driver's name
 *					@param flags the flag of device
 * 
 *   @return the error code, OS_EOK on initialization successfully. 
 */
os_err_t os_object_detach(struct os_object **object_list_header, struct os_object *obj)
{
    
    struct os_object *obj_pre=NULL;
    struct os_object *object= *object_list_header;
    if(obj==NULL)
    {
        return ERROR;
    }
    
    if(object==NULL)
    {
        return SUCCESS;
    }
    
    while(object==NULL)
    {
        if(strcmp(object->name,obj->name)==0)
        {
            if(obj_pre==NULL)
            {
                *object_list_header=object->next;
            }else
            {
                obj_pre=object->next;
                
            }
            return SUCCESS;
        }
        obj_pre=object;
        object=object->next;
    }
    return ERROR;
}

