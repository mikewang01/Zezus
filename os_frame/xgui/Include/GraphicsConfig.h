/*********************************************************************
 * Module for Microchip Graphics Library
 * This file contains compile time options for the Graphics Library. 
 *********************************************************************
 * FileName:        GraphicsConfig.h
 * Dependencies:    none
 * Processor:       PIC24/PIC30/PIC32
 * Compiler:        C30 V3.00, C32
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright © 2008 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Anton Alkhimenok		10/28/2007
 ********************************************************************/

#ifndef _GRAPHICSCONFIG_H
#define _GRAPHICSCONFIG_H

//////////////////// COMPILE OPTIONS AND DEFAULTS ////////////////////

////////////  HARDWARE PROFILE /////////////
#include "HardwareProfile.h"

/////////// PORTRAIT LCD ORIENTATION ////////
//#define USE_PORTRAIT

//// GRAPHICS PICTAIL VERSION SELECTION /////
#define GRAPHICS_PICTAIL_VERSION   2

///// BLOCKING/NON-BLOCKING SELECTION ///////
#define USE_NONBLOCKING_CONFIG // Comment this line to use blocking configuretion

///////////// GOL FOCUS SUPPORT /////////////
//#define USE_FOCUS

//////////// INPUT DEVICES USED /////////////
//#define USE_TOUCHSCREEN
//#define USE_KEYBOARD
//#define USE_MOUSE

/////////////// WIDGETS USED ////////////////
//#define USE_BUTTON
//#define USE_WINDOW
//#define USE_CHECKBOX
//#define USE_RADIOBUTTON
//#define USE_EDITBOX
//#define USE_LISTBOX
//#define USE_SLIDER // required for scrool bar
//#define USE_PROGRESSBAR
//#define USE_STATICTEXT
//#define USE_PICTURE
//#define USE_GROUPBOX
//#define USE_ROUNDDIAL
//#define USE_METER
//#define USE_CUSTOM

////////////// UNICODE SUPPORT //////////////
//#define USE_MULTIBYTECHAR

/////////////// FONT OPTIONS ////////////////
// Support for fonts located in internal flash
#define USE_FONT_FLASH
// Support for fonts located in external memory
//#define USE_FONT_EXTERNAL

////////////// BITMAP OPTIONS ///////////////
// Support for bitmaps located in internal flash
#define USE_BITMAP_FLASH
// Support for bitmaps located in external memory
//#define USE_BITMAP_EXTERNAL

///////////////// DRIVER ////////////////////

//#include "Graphics\LGDP4531_R61505_S6D0129_S6D0139_SPFD5408.h"
//#include "Graphics\SSD1906.h"   // Driver for Solomon Systech. SSD1906 controller 
#include "Graphics\SSD1303.h"   // Driver for SSD1303 (monochrome OLED display) controller
//#include "Graphics\SH1101A.h"	  // Driver for the HS1101A (monochrome OLED display) controller
//#include "Graphics\HIT1270L.h"  // Landscape driver for Densitron HIT1270 controller
//#include "Graphics\SSD1339.h"   // Driver for Solomon Systech. SSD1339 controller 
        // GOL layer 
#define USE_BUTTON

//#define USE_WINDOW
  

//#define USE_GROUPBOX
   

#define USE_STATICTEXT

//#define USE_SLIDER

//#define USE_CHECKBOX

//#define USE_RADIOBUTTON

#define USE_PICTURE

#define USE_PROGRESSBAR

#define USE_EDITBOX

#define USE_LISTBOX

#define USE_ROUNDDIAL

//#define USE_METER

#define USE_GRID



#endif // _GRAPHICSCONFIG_H
