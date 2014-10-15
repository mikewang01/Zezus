/***************************************************************
 * Name:      task.c
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
#include "manufacture_task.h"
#include "usertask.h"
#include "Graphics\Graphics.h"

//****************************************************************************
//*************** START OF PRIVATE TYPE AND SYMBOL DEFINITIONS ***************
//**************************************************************************** 

void Task_Free(void);


void  Task_Free_Init(void);

void  task_gui_init();

void  task_gui();
// ***************************************************************************
// ******************** START OF PRIVATE DATA DECLARATIONS *******************
// ***************************************************************************




// ***************************************************************************
// ******************** START OF global variable DECLARATIONS *******************
// ***************************************************************************

u8 Current_Task=0;

const Task My_Task[]={
   
    task_demo,        /*user demo task*/
	  task_manufacture, /*manufacture test process task*/
	  task_gui_init,
    Task_Free		     /*free time task*/
};

const Task My_Task_Init[]=
{
    
    task_demo_init,
  	task_manufacture_init,
	  task_gui,
    Task_Free_Init,
};

u16 T_NUM=(sizeof( My_Task )/sizeof( My_Task[0] ));

/***************************************************************************
 ***************************************************************************
 **************** START OF PRIVATE  PROCEDURE IMPLEMENTATIONS **************
 ***************************************************************************/


/*********************************************************************
 * FUNCTIONS
 *********************************************************************/



/*********************************************************************
 * @fn      void Task_Free_Init()
 *
 * @brief   come to run when cpu is free
 *
 * @param   void
 *
 * @return  none
 */

void Task_Free_Init()
{
    
    //send_message(3,SYSTEM_EVENT,IPC_MSG,START_TICK,NULL,(u16)0);
}				


/*********************************************************************
 * @fn      void Task_Free_Init()
 *
 * @brief   come to run when cpu is free
 *
 * @param   void
 *
 * @return  none
 */
void Task_Free()
{
    u8 My_Id=Current_Task;
    // LED4=1;
    //os_clock_close("ALL");
    //	 Sys_Standby(sleepmode);
    //os_clock_restore("ALL");
    
    //   LED4=!LED4;
    GOLDraw();
    task_ticks_inc(My_Id);
}


/*********************************************************************
 * @fn      Task_Led_Init()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */

void board_hardware_initiate()
{
    
}
/*********************************************************************
 * @fn     void System_Tick_Hook()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */

void System_Tick_Hook()
{
    
    
    
}






