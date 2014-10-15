
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
#include "Graphics\Graphics.h"
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
 void task_gui();
void  task_gui_init() 
{
    u8 this_task_id=Current_Task;
	  InitGraph();
	//  task_gui();
    
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
GOL_SCHEME    altScheme;
extern void lcd_refresh_buffer(void);
void task_gui()
{
    
    u8 My_Id=Current_Task ,counter;
          PCB* task_state=0;
    u16 width,height;
	  GOL_SCHEME *pScheme;
    BUTTON *buttons[3];
    WORD state;
          SetColor(WHITE);
        /* for Truly display */
	   altScheme.Color0 =	WHITE;
     altScheme.Color1 = WHITE;
	   altScheme.EmbossDkColor =  WHITE;
	   altScheme.EmbossLtColor = WHITE;
     altScheme.ColorDisabled = WHITE;

    altScheme.TextColor1 = WHITE;
    altScheme.TextColor0 = WHITE;
    altScheme.TextColorDisabled =WHITE;
        for(counter=0; counter<GetMaxX(); counter+=20){
            Line(counter,0,GetMaxX()-1-counter,GetMaxY()-1);
        }	
				 lcd_refresh_buffer();
				 DelayMs(4000);
				 SetColor(BLACK);				
				 ClearDevice();
				
				
				  for(counter=10; counter<GetMaxY()>>1; counter+=10){
            Circle(GetMaxX()>>1,GetMaxY()>>1,counter);
        }
				
							
				 lcd_refresh_buffer();
				 DelayMs(4000);
				 SetColor(BLACK);				
				 ClearDevice();
				
					SetColor(WHITE);

        for(counter=0; counter<GetMaxX(); counter+=20){
            Line(counter,0,GetMaxX()-1-counter,GetMaxY()-1);
        }

       			
				 lcd_refresh_buffer();
				 DelayMs(4000);
				 SetColor(BLACK);				
				 ClearDevice();

        SetColor(WHITE);

        for(counter=10; counter<GetMaxY()>>1; counter+=10){
            Circle(GetMaxX()>>1,GetMaxY()>>1,counter);
        }
				

         lcd_refresh_buffer();
				 DelayMs(4000);
				 SetColor(BLACK);				
				 ClearDevice();
				

        SetColor(WHITE);
        FillCircle(GetMaxX()>>1,GetMaxY()>>1,60);
        SetColor(WHITE);
        FillCircle(GetMaxX()>>1,GetMaxY()>>1,40);
        SetColor(WHITE);
        FillCircle(GetMaxX()>>1,GetMaxY()>>1,20);
        
				
				 lcd_refresh_buffer();
				 DelayMs(4000);
				 SetColor(BLACK);				
				 ClearDevice();
				
          
				SetFont((void*)&GOLFontDefault);
        SetColor(WHITE);
        width = GetTextWidth("Microchip Tech.",(void*)&GOLFontDefault);
        height = GetTextHeight((void*)&GOLFontDefault);
       
        OutTextXY( (GetMaxX()-width)>>1,
                   (GetMaxY()-height)>>1,
                    "Microchip Tech.");
         lcd_refresh_buffer();
				 DelayMs(4000);
				 SetColor(BLACK);				
				 ClearDevice();
				
				


    pScheme = GOLCreateScheme();
    state = BTN_DRAW;
		pScheme ->EmbossLtColor =BLACK;
		pScheme ->EmbossDkColor =WHITE;
		pScheme -> CommonBkColor =BLACK;
		pScheme ->Color0 =BLACK;
    pScheme ->TextColor0 =WHITE;
    buttons[0] = BtnCreate(1,0,0,50,30,10, state, NULL, "ON", pScheme);
    // check if button 0 is created
    if (buttons[0] == NULL)     
       return ;
    buttons[1] = BtnCreate(2,60,0,110,30,10, state, NULL, "OFF", pScheme);
    // check if button 0 is created
    if (buttons[0] == NULL)     
       return ;
		
		#define ID_MYEDITBOX    101
		EDITBOX *pEb;

		pEb = EbCreate(ID_MYEDITBOX,    // ID
								 10,                // left
								 35,                // top
								 100,               // right
								 64,                // bottom
								 EB_DRAW,           // redraw after creation                    
								 "HELLO",              // no text yet
								 8,                 // display only four characters
								 pScheme);          // pointer to the style scheme         

		if( pEb == NULL )
		{
				// MEMORY ERROR. Object was not created.
		}

		//LbCreate(3, 30, 35, 58, 50, state , "LIST BOX", pScheme);
    GOLDraw();
	

//				
//				
//	  /*get self PCB to lead a further process */
//    get_self_taskstate(My_Id,task_state);
//    
//    // printf("taskid = %d" , My_Id );
//    /*it actully get some message recieve and process it*/
//    if(((task_state->State)&SYSTEM_EVENT)==SYSTEM_EVENT)
//    {
//        
//        smessage* msg_get;
//			
//        /*get detailed message through PCB*/
//        u8 msg_num=get_message(task_state , &msg_get);
//       // printf("get message %d", msg_num);
//			
//        /*process the message has been recieved*/
//        process_msg(My_Id, msg_num , msg_get);
//    }
//    
//    /*delete messge recieved or memmory leakage gonna happen*/
//    delete_message(My_Id,SYSTEM_EVENT);
    
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
    
   
}


