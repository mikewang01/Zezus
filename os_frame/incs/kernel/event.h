//----------------------------------------------------------------------------
// Tenway Corp. Company Confidential Strictly Private                         
//                                                                            
// $RCSfile: Version.h,v $                                                    
// $Revision: 1.0 $                                                           
// $Author: Wenbing Ma $                                                      
// $Date: 2013/4/16 15:55:34 $                                                
//                                                                            
// ---------------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// ---------------------------------------------------------------------------
// Copyright 2013-2014 (c) Tenway Corp.                                       
//                                                                            
// Tenway owns the sole copyright to this software. Under international       
// copyright laws you (1) may not make a copy of this software except for     
// the purposes of maintaining a single archive copy, (2) may not derive      
// works herefrom, (3) may not distribute this work to others. These rights   
// are provided for information clarification, other restrictions of rights   
// may apply as well.                                                         
//                                                                            
// This is an unpublished work.                                               
// ---------------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>> WARRANTEE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// ---------------------------------------------------------------------------
// Tenway Corp. MAKES NO WARRANTY OF ANY KIND WITH REGARD TO THE USE OF       
// THIS SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR      
// PURPOSE.                                                                   
// ---------------------------------------------------------------------------

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __EVENT_H__
#define __EVENT_H__
/* Includes ------------------------------------------------------------------*/

#include "cpu.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

// PCB related event
//#define DEVICE_EVENT       0X01
#define SYSTEM_EVENT       0x04
//#define TIME_EVENT			   0x08

// message related type to determine  which group event areas belongs to
#define IPC_MSG    0x01 //interprocess communication meg group
#define TIMER_MSG  0X02// timer message group
#define DEV_MSG    0X03



// specific timer EVENT
#define TIMER_EXPIRED  0X01
#define TIMER_PERIOD	 0X02

/* U can define your own timer event here*/
#define TIMEOUT_500MS  0x03
#define TIMEOUT_400MS  0x04
#define TIMEOUT_250MS  0x05
#define TIMEOUT_125MS  0x06
#define TIMEOUT_100MS  0x07

#define TIMEOUT_RELAY1 0X08
// specific IPC message 
#define START_TICK   0x01

/* U can define your ipc event here*/
// specific DEVICE EVENT

#define USART1_RX    					0X01
#define KEY_STATUS_CHANGED    0X02
#define CALIBRATION_COMMAND   0X03


#endif

