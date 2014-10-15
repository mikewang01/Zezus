
/************************************************************
Copyright (C), 2008-2014, Colorful Sea Tech. Co., Ltd.
FileName:       mtd.c
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
Mike      14/10/11      0.0       build this moudle  
***********************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "kernel-includes.h"




/*********************************************************************
* MACROS
*/

#define BOARD_FLASH_SIZE  (32*1024*1024)
#define SIZE_OF_1K      1024
/*********************************************************************
 * TYPEDEFS
 */

enum mtd_flash_mask
{
    partion_read_allowed  =0x01,
    partion_write_allowed =0x02,
    partion_erase_allowed =0x04
    
};

typedef struct mtd_partition {  
    char *name;            /* identifier string */    
    u32 size;              /* partition size */      
    u32 offset;            /* offset within the master MTD space */              	               
    u32 mask_flags;        /* master MTD flags to mask out for this partition */               
}mtd_partition_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
//#define 
static mtd_partition_t flash_layouts[]=
{
    {
        .name    = "fw_1",
        .offset  = 0,
        .size = 64*SIZE_OF_1K,
        .mask_flags =partion_read_allowed,
        
    },
    {
        .name    = "fw_2",
        .offset  = 64*SIZE_OF_1K,
        .size = 64*SIZE_OF_1K,
        .mask_flags =partion_read_allowed,
        
    },
    {
        .name    = "gbk12",
        .offset  = 128*SIZE_OF_1K,
        .size = 564*SIZE_OF_1K,
        .mask_flags =partion_read_allowed,
        
    },
    {
        .name    = "gbk16",
        .offset  = 692*SIZE_OF_1K,
        .size = 752*SIZE_OF_1K,
        .mask_flags =partion_read_allowed,
        
    },
    {
        .name    = "uni2gbk",
        .offset  = 14444*SIZE_OF_1K,
        .size =    11*SIZE_OF_1K,
        .mask_flags =partion_read_allowed,
        
    },
    {
        .name    = "logo",
        .offset  = 14456*SIZE_OF_1K,
        .size =    1*SIZE_OF_1K,
        .mask_flags =partion_read_allowed,
        
    }
};

/*********************************************************************
 * LOCAL function declarement
 */


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