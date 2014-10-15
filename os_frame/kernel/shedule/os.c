/***************************************************************
 * Name:      os.c
 * Purpose:   code for operating system
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/


/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "kernel-includes.h"
#include "systick.h"
extern u16 T_NUM;//amont of task 


// ***************************************************************************
// ******************** START OF global variable DECLARATIONS *******************
// ***************************************************************************

u32 tick_tok=0;
#define Running 99
#define Suspend 98

/***************************************************************************
 ***************************************************************************
 **************** START OF PRIVATE  PROCEDURE IMPLEMENTATIONS **************
 ***************************************************************************/

extern u8 Current_Task;
extern const Task My_Task[];
extern const Task My_Task_Init[];


static PCB *Task_State;	//process control block


/*********************************************************************
 * @fn      os_err_t rt_device_set_tx_complete
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */	

const char *const corp_logo=
        " \
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n \
        @~~~~~~~~~~~~~~~~#~~~~~~~~#~~~#~~~~~~~~@\r\n \
        @~##########~~~~#######~~~~#~~#######~~@\r\n \
        @~~~~~#~~~~~~~~#~~~~~~#~~~~~~#~~~~~~~~~@\r\n \
        @~~~~~#~~~~~~~#~~~~~~#~~~~~~#~######~~~@\r\n \
        @~~~~~#~~~~~~~~#########~~#~~~#~~~~#~~~@\r\n \
        @~~########~~~~#~~~#~~~#~~~#~~#~#~~#~~~@\r\n \
        @~~~~#~~~~#~~~~#~~~#~~~#~~~~#########~~@\r\n \
        @~~~~#~~~~#~~~~#########~~~~~#~~~~~#~~~@\r\n \
        @~~~~#~~~~#~~~~#~~~~~~~~~~~~~#~~#~~#~~~@\r\n \
        @~~~#~~~~~#~~~~#~~~~~~~~#~~#~########~~@\r\n \
        @~~~#~~~~~#~~~~#~~~~~~~~#~#~~~~~~~~#~~~@\r\n \
        @~###########~~~#########~~~~~~~~##~~~~@\r\n \
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
        void os_show_version(void)
{
        printf("\r\n%s",corp_logo);
        printf("\r\n\\ | /\r\n");
        printf("-WSH-    WSH Operating System\r\n");
        printf("/ | \\     %d.%d.%d build %s\r\n",
               VER_MAJOR, VER_MINOR, REVISION, __DATE__);
        printf(" 20012 - 2013 Copyright by MikeWang\r\n");
        }
        
        /*********************************************************************
         * @fn      OS_Init()
         *
         * @brief   This function  init all functions invoked in My_Task_Init and establish
                    Task_State structor for each peocess.
         *
         * @param   void
         *
         * @return  none
         */
        extern  os_err_t clock_add_to_manager_list(void);
void OS_Init()
{
    u8 count;
    
    Task_State=(PCB*)osmalloc(sizeof(PCB)*T_NUM);	//申请 动态的任务管理块
    sys_tick_init(SYSCLK);	     //?????
    mem_init();	 
    clock_add_to_manager_list();
    //	
    for(count=0;count<T_NUM;count++)
    {
        Current_Task=count;
        Task_State[count].ptr=NULL;
        My_Task_Init[count]();	
    }
    
}





/*********************************************************************
 * @fn      OS_get_taskstate
 *
 * @brief   delay Task and Suspend itself
 *
 * @param   Task_Id
 *
 * @return  none
 */
PCB* OS_get_taskstate()
{
    return Task_State;
}

/*********************************************************************
 * @fn      OS_get_tick()
 *
 * @brief   get current tick count 
 *
 * @param   
 *
 * @return  current_tick count
 */
u32 OS_get_tick()
{
    return tick_tok;
}




void Shedule()
{
     u8 idx=0;
    while(1)
    {
        
    do {
			   /* Task is highest priority that is ready.*/
				if (Task_State[idx].State)
				 {
					 break;
				 }
			} while (++idx < T_NUM);
			
			/*make sure that one task is ready if no ,no function gonna be excecuted*/
	     if(idx < T_NUM)
			 {
					Current_Task=idx;
				  My_Task[idx]();
				 
			 }  
			 /*clear task index or result gonna be a unkown error*/
       idx=0;			 
     }
            
        
 }



void systick_process()
{
    
    
    // u8 count;
#ifdef System_Hook
    System_Tick_Hook();
#endif
    tick_tok++;
    timer_process();
    
}



/*********************************************************************
 * @fn      exam_assert()
 *
 * @brief   check 
 *
 * @param   
 *
 * @return  current_tick count
 */
void  exam_assert( char * file_name, unsigned int line_no )
{
	printf( "\n[EXAM]Assert failed: %s, line %u\n", 
	file_name, line_no );
	abort( );
}


