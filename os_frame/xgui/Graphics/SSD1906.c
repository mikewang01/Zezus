/*****************************************************************************
 *  Module for Microchip Graphics Library
 *  Solomon Systech. SSD1906 LCD controller driver
 *****************************************************************************
 * FileName:        SSD1906.c
 * Dependencies:    Graphics.h
 * Processor:       PIC24, PIC32
 * Compiler:       	MPLAB C30, MPLAB C32
 * Linker:          MPLAB LINK30, MPLAB LINK32
 * Company:         Microchip Technology Incorporated
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
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Anton Alkhimenok     12/19/07
 * Anton Alkhimenok     01/31/08    combined portrait and landscape, PIC32 support	
 *****************************************************************************/
#include "Graphics\Graphics.h"

// Color
WORD  _color;
// Clipping region control
SHORT _clipRgn;
// Clipping region borders
SHORT _clipLeft;
SHORT _clipTop;
SHORT _clipRight;
SHORT _clipBottom;

/////////////////////// LOCAL FUNCTIONS PROTOTYPES ////////////////////////////
#ifdef __PIC32MX
DWORD _address; // current video buffer address for PIC32 implementation
#define SetAddress(address) _address = address
#else
void SetAddress(DWORD address);
#endif

void SetReg(WORD index, BYTE value);
BYTE GetReg(WORD index);

void PutImage1BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch);
void PutImage4BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch);
void PutImage8BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch);
void PutImage16BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch);

void PutImage1BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch);
void PutImage4BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch);
void PutImage8BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch);
void PutImage16BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch);

/*********************************************************************
* Function:  void  DelayMs(WORD time)
*
* PreCondition: none
*
* Input: time - delay in ms
*
* Output: none
*
* Side Effects: none
*
* Overview: delays execution on time specified in ms
*
* Note: delay is defined for 16MIPS
*
********************************************************************/
#ifdef __PIC32MX
void  DelayMs(WORD time)
{
    while(time--)
    {
        unsigned int int_status;

        int_status = INTDisableInterrupts();
        OpenCoreTimer(GetSystemClock() / 2000);     // core timer is at 1/2 system clock
        INTRestoreInterrupts(int_status);

        mCTClearIntFlag();

        while(!mCTGetIntFlag());

    }

    mCTClearIntFlag();
}
#else
#define DELAY_1MS 16000/5  // for 16MIPS
void  DelayMs(WORD time){
unsigned delay;
	while(time--)
		for(delay=0; delay<DELAY_1MS; delay++);	
}
#endif

/*********************************************************************
* Function:  void SetAddress(DWORD address)
*
* PreCondition: none
*
* Input: address - address 
*
* Output: none
*
* Side Effects: none
*
* Overview: sets the address for read/write operations
*
* Note: none
*
********************************************************************/
#ifndef __PIC32MX
void SetAddress(DWORD address){

    PMADDR = ((DWORD_VAL)address).w[0];
    A17_LAT_BIT = 0;
    if(((DWORD_VAL)address).w[1])
        A17_LAT_BIT = 1;

}
#endif

#ifdef __PIC32MX
/*********************************************************************
* Function:  void  WriteData(WORD value)
*
* PreCondition: none
*
* Input:  value - value to be written
*
* Output: none
*
* Side Effects: none
*
* Overview: writes 16-bit word into video buffer at current address.
*           The address pointer is incremented after the operation.
*
* Note: none
*
********************************************************************/
void WriteData(WORD value){

    RS_LAT_BIT = 1;        // set RS line to low for register space access

    A17_LAT_BIT = 0;                                  // set address[17]    
    if(((DWORD_VAL)_address).w[1]&0x0001)
        A17_LAT_BIT = 1;
    PMADDR = ((DWORD_VAL)_address).w[0];              // set address[16:1]
    A0_LAT_BIT = 1;                                   // set address[0]
    PMDIN1 = ((WORD_VAL)value).v[0];                  // write low byte
    _address++;
    PMPWaitBusy();         // wait for the transmission end

    A0_LAT_BIT = 0;                                   // set address[0]
    PMDIN1 = ((WORD_VAL)value).v[1];                  // write high byte
    PMPWaitBusy();         // wait for the transmission end
}
#endif

/*********************************************************************
* Function:  void  SetReg(WORD index, BYTE value)
*
* PreCondition: none
*
* Input: index - register number
*        value - value to be set
*
* Output: none
*
* Side Effects: none
*
* Overview: sets graphics controller register (byte access)
*
* Note: none
*
********************************************************************/
void  SetReg(WORD index, BYTE value){

#ifndef __PIC32MX
    PMMODEbits.MODE16 = 0; // turn off 16 bit data mode
    PMCONbits.PTBEEN = 0;  // disable BE line for 16-bit access
#endif

    RS_LAT_BIT = 0;        // set RS line to low for register space access
    CS_LAT_BIT = 0;        // enable SSD1906 

    A17_LAT_BIT = 0;            // set address[17]    
    PMADDR = index>>1;          // set address[16:1]
    A0_LAT_BIT = index&0x0001;  // set address[0]


    PMDIN1 = value;        // write value
    PMPWaitBusy();            // wait for the transmission end
  
    CS_LAT_BIT = 1;        // disable SSD1906

#ifndef __PIC32MX
    PMCONbits.PTBEEN = 1;  // enable BE line for 16-bit access
    PMMODEbits.MODE16 = 1; // restore 16 bit data mode
#endif
}

/*********************************************************************
* Function:  BYTE GetReg(WORD index)
*
* PreCondition: none
*
* Input: index - register number
*
* Output: none
*
* Side Effects: none
*
* Overview: returns graphics controller register value (byte access)
*
* Note: none
*
********************************************************************/
BYTE  GetReg(WORD index){
BYTE value;

#ifndef __PIC32MX
    PMMODEbits.MODE16 = 0; // turn off 16 bit data mode
    PMCONbits.PTBEEN = 0;  // disable BE line for 16-bit access
#endif

    RS_LAT_BIT = 0;        // set RS line to low for register space access
    CS_LAT_BIT = 0;        // enable SSD1906 

    A17_LAT_BIT = 0;            // set address[17]    
    PMADDR = index>>1;          // set address[16:1]
    A0_LAT_BIT = index&0x0001;  // set address[0]

    value = PMDIN1;        // start transmission, read dummy value
    PMPWaitBusy();         // wait for the transmission end

    CS_LAT_BIT = 1;        // disable SSD1906

    PMCONbits.PMPEN  = 0;  // suspend PMP
    value = PMDIN1;        // read value
    PMCONbits.PMPEN  = 1;  // resume PMP

#ifndef __PIC32MX
    PMCONbits.PTBEEN = 1;  // enable BE line for 16-bit access
    PMMODEbits.MODE16 = 1; // restore 16 bit data mode
#endif

    return value;
}

/*********************************************************************
* Function:  void ResetDevice()
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: resets LCD, initializes PMP
*
* Note: none
*
********************************************************************/
void ResetDevice(void){
BYTE bReg;

    RST_LAT_BIT = 0;       // hold in reset by default
    RST_TRIS_BIT = 0;      // enable RESET line

    A0_TRIS_BIT = 0;       // enable A0  line for byte access
    A17_TRIS_BIT = 0;      // enable A17 line

    RS_TRIS_BIT = 0;       // enable RS line 

    CS_LAT_BIT = 1;        // SSD1906 is not selected by default
    CS_TRIS_BIT = 0;       // enable 1906 CS line    
    
    // PMP setup 
    PMMODE = 0; PMAEN = 0; PMCON = 0;
    PMMODEbits.MODE   = 2;  // Intel 80 master interface
#ifdef __PIC32MX            
    PMMODEbits.WAITB  = 3;
    PMMODEbits.WAITM  = 13;
    PMMODEbits.WAITE  = 3;
    PMMODEbits.INCM   = 0;  // auto increment address
#else
    PMMODEbits.WAITB  = 1;
    PMMODEbits.WAITM  = 5;
    PMMODEbits.WAITE  = 1;
    PMMODEbits.INCM   = 1;  // auto increment address
#endif

#ifdef __PIC32MX
    PMMODEbits.MODE16 = 0;  // 16 bit data are multiplexed
#else
    PMMODEbits.MODE16 = 1;  // 16 bit data are multiplexed
#endif

    PMAENbits.PTEN0 = 1;   // enable low address latch
    PMAENbits.PTEN1 = 1;   // enable high address latch

#ifndef __PIC32MX
    PMCONbits.PTBEEN = 1;   // enable data latch line
#endif
    PMCONbits.ADRMUX = 2;   // address is multiplexed on data bus
    PMCONbits.CSF    = 0;
    PMCONbits.ALP    = 1;  // set address latch control polarity 
    PMCONbits.PTRDEN = 1;  // enable RD line
    PMCONbits.PTWREN = 1;  // enable WR line
    PMCONbits.PMPEN  = 1;  // enable PMP

    DelayMs(40);
    RST_LAT_BIT = 1;       // release from reset
    DelayMs(20);

    SetReg(REG_MEMCLK_CONFIG,0x00);             // Reg    4h 
    SetReg(REG_PCLK_CONFIG,0x32);               // Reg    5h 
	SetReg(REG_PANEL_TYPE,0x61);                // Reg   10h 
	SetReg(REG_MOD_RATE,0x00);                  // Reg   11h 
	SetReg(REG_HORIZ_TOTAL,0x22);               // Reg   12h 
	SetReg(REG_HDP,0x1d);                       // Reg   14h 
	SetReg(REG_HDP_START_POS0,0x11);            // Reg   16h 
	SetReg(REG_HDP_START_POS1,0x00);            // Reg   17h 
	SetReg(REG_VERT_TOTAL0,0x46);               // Reg   18h 
	SetReg(REG_VERT_TOTAL1,0x01);               // Reg   19h 
	SetReg(REG_VDP0,0x3f);                      // Reg   1ch 
	SetReg(REG_VDP1,0x01);                      // Reg   1dh 
	SetReg(REG_VDP_START_POS0,0x05);            // Reg   1eh 
	SetReg(REG_VDP_START_POS1,0x00);            // Reg   1fh 
	SetReg(REG_HSYNC_PULSE_WIDTH,0x04);         // Reg   20h 
	SetReg(REG_HSYNC_PULSE_START_POS0,0x00);    // Reg   22h 
	SetReg(REG_HSYNC_PULSE_START_POS1,0x00);    // Reg   23h 
	SetReg(REG_VSYNC_PULSE_WIDTH,0x00);         // Reg   24h 
	SetReg(REG_VSYNC_PULSE_START_POS0,0x00);    // Reg   26h 
	SetReg(REG_VSYNC_PULSE_START_POS1,0x00);    // Reg   27h 

    SetReg(REG_DISPLAY_MODE,0x84);              // Reg   70h

#ifdef USE_PORTRAIT
	SetReg(REG_SPECIAL_EFFECTS,0x40);           // Reg   71h
	SetReg(REG_MAIN_WIN_DISP_START_ADDR0,0x00); // Reg   74h
	SetReg(REG_MAIN_WIN_ADDR_OFFSET0,0x78);     // Reg   78h 
#else
	SetReg(REG_SPECIAL_EFFECTS,0x41);           // Reg   71h
	SetReg(REG_MAIN_WIN_DISP_START_ADDR0,0x9f); // Reg   74h
	SetReg(REG_MAIN_WIN_ADDR_OFFSET0,0xa0);     // Reg   78h 
#endif

	SetReg(REG_MAIN_WIN_DISP_START_ADDR1,0x00); // Reg   75h
	SetReg(REG_MAIN_WIN_DISP_START_ADDR2,0x00); // Reg   76h
	SetReg(REG_MAIN_WIN_ADDR_OFFSET1,0x00);     // Reg   79h
	SetReg(REG_FLOAT_WIN_DISP_START_ADDR0,0x00);// Reg   7ch
	SetReg(REG_FLOAT_WIN_DISP_START_ADDR1,0x00);// Reg   7dh
	SetReg(REG_FLOAT_WIN_DISP_START_ADDR2,0x00);// Reg   7eh
	SetReg(REG_FLOAT_WIN_ADDR_OFFSET0,0x50);    // Reg   80h
	SetReg(REG_FLOAT_WIN_ADDR_OFFSET1,0x00);    // Reg   81h
	SetReg(REG_FLOAT_WIN_X_START_POS0,0x00);    // Reg   84h
	SetReg(REG_FLOAT_WIN_X_START_POS1,0x00);    // Reg   85h
	SetReg(REG_FLOAT_WIN_Y_START_POS0,0x00);    // Reg   88h
	SetReg(REG_FLOAT_WIN_Y_START_POS1,0x00);    // Reg   89h
	SetReg(REG_FLOAT_WIN_X_END_POS0,0x4f);      // Reg   8ch
	SetReg(REG_FLOAT_WIN_X_END_POS1,0x00);      // Reg   8dh
	SetReg(REG_FLOAT_WIN_Y_END_POS0,0xef);      // Reg   90h
	SetReg(REG_FLOAT_WIN_Y_END_POS1,0x00);      // Reg   91h
	SetReg(REG_POWER_SAVE_CONFIG,0x00);         // Reg   a0h
	SetReg(REG_SOFTWARE_RESET,0x00);            // Reg   a2h
	SetReg(REG_SCRATCH_PAD0,0x00);              // Reg   a4h
	SetReg(REG_SCRATCH_PAD1,0x00);              // Reg   a5h
	SetReg(REG_GPIO_CONFIG0,0x00);              // Reg   a8h
	SetReg(REG_GPIO_CONFIG1,0x80);              // Reg   a9h
	SetReg(REG_GPIO_STATUS_CONTROL0,0x00);      // Reg   ach
	SetReg(REG_GPIO_STATUS_CONTROL1,0x00);      // Reg   adh
	SetReg(REG_PWM_CV_CLOCK_CONTROL,0x00);      // Reg   b0h
	SetReg(REG_PWM_CV_CLOCK_CONFIG,0x00);       // Reg   b1h
	SetReg(REG_CV_CLOCK_BURST_LENGTH,0x00);     // Reg   b2h
	SetReg(REG_PWM_CLOCK_DUTY_CYCLE,0x00);      // Reg   b3h

    DelayMs(500);

	while (!(GetReg(REG_POWER_SAVE_CONFIG) & 0x80)); // SSD1906 vertical sync signal exists

	SetReg(REG_GPIO_STATUS_CONTROL1,0x80);	    // set GPO high to turn on LCD panel
    SetReg(REG_DISPLAY_MODE,0x04);		        // enable RAM content to screen
}

/*********************************************************************
* Function: void PutPixel(SHORT x, SHORT y)
*
* PreCondition: none
*
* Input: x,y - pixel coordinates
*
* Output: none
*
* Side Effects: none
*
* Overview: puts pixel
*
* Note: none
*
********************************************************************/
void PutPixel(SHORT x, SHORT y){
DWORD address;
    if(_clipRgn){
        if(x<_clipLeft)
            return;
        if(x>_clipRight)
            return;
        if(y<_clipTop)
            return;
        if(y>_clipBottom)
            return;
    }
    address = (DWORD)LINE_MEM_PITCH*y + x;
    SetAddress(address);
    CS_LAT_BIT = 0;
    WriteData(_color);
    CS_LAT_BIT = 1;
}

/*********************************************************************
* Function: WORD GetPixel(SHORT x, SHORT y)
*
* PreCondition: none
*
* Input: x,y - pixel coordinates 
*
* Output: pixel color
*
* Side Effects: none
*
* Overview: returns pixel color at x,y position
*
* Note: none
*
********************************************************************/
WORD GetPixel(SHORT x, SHORT y){
DWORD address;
WORD  value;

    address = (long)LINE_MEM_PITCH*y + x;
    SetAddress(address);

    CS_LAT_BIT = 0;        // enable SSD1906

#ifdef __PIC32MX
    A17_LAT_BIT = 0;                                  // set address[17]    
    if(((DWORD_VAL)_address).w[1]&0x0001)
        A17_LAT_BIT = 1;
    PMADDR = ((DWORD_VAL)_address).w[0];              // set address[16:1]
    A0_LAT_BIT = 1;                                   // set address[0]
    ((WORD_VAL)value).v[0] = PMDIN;  // start transmission, read dummy value
    PMPWaitBusy();         // wait for the transmission end
    A0_LAT_BIT = 0;                                   // set address[0]
    ((WORD_VAL)value).v[0] = PMDIN;  // start transmission, read low byte           
#else
    value = PMDIN1;        // start transmission, read dummy value
#endif
    PMPWaitBusy();         // wait for the transmission end
    CS_LAT_BIT = 1;        // disable SSD1906

    PMCONbits.PMPEN  = 0;  // suspend PMP
#ifdef __PIC32MX
    ((WORD_VAL)value).v[0] = PMDIN;  // read high byte           
#else
    value = PMDIN1;        // read value
#endif
    PMCONbits.PMPEN  = 1;  // resume PMP

    return value;
}

/*********************************************************************
* Function: void Bar(SHORT left, SHORT top, SHORT right, SHORT bottom)
*
* PreCondition: none
*
* Input: left,top - top left corner coordinates,
*        right,bottom - bottom right corner coordinates
*
* Output: none
*
* Side Effects: none
*
* Overview: draws rectangle filled with current color
*
* Note: none
*
********************************************************************/
void Bar(SHORT left, SHORT top, SHORT right, SHORT bottom){
DWORD address;
register SHORT  x,y;

    if(_clipRgn){
        if(left<_clipLeft)
           left = _clipLeft;
        if(right>_clipRight)
           right= _clipRight;
        if(top<_clipTop)
           top = _clipTop;
        if(bottom>_clipBottom)
           bottom = _clipBottom;
    }

    address = (DWORD)LINE_MEM_PITCH*top + left;

    CS_LAT_BIT = 0;
    for(y=top; y<bottom+1; y++){
        SetAddress(address);
        for(x=left; x<right+1; x++){
            WriteData(_color);
        }
        address += LINE_MEM_PITCH;
    }
    CS_LAT_BIT = 1;
}

/*********************************************************************
* Function: void ClearDevice(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: clears screen with current color 
*
* Note: none
*
********************************************************************/
void ClearDevice(void){
DWORD     counter;

    SetAddress(0);
    CS_LAT_BIT = 0;
    for(counter=0; counter<(DWORD)(GetMaxX()+1)*(GetMaxY()+1); counter++){
        WriteData(_color);
    }
    CS_LAT_BIT = 1;
}

/*********************************************************************
* Function: void PutImage(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner,
*        bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage(SHORT left, SHORT top, void* bitmap, BYTE stretch){
FLASH_BYTE* flashAddress;
BYTE colorDepth;
WORD colorTemp;

    // Save current color
    colorTemp = _color;

    switch(*((SHORT*)bitmap))
    {
#ifdef USE_BITMAP_FLASH
        case FLASH:
            // Image address
            flashAddress = ((BITMAP_FLASH*)bitmap)->address;
            // Read color depth
            colorDepth = *(flashAddress+1);
            // Draw picture
            switch(colorDepth){
                case 1:
                    PutImage1BPP(left, top, flashAddress, stretch);
                    break;
                case 4:
                    PutImage4BPP(left, top, flashAddress, stretch);
                    break;
                case 8:
                    PutImage8BPP(left, top, flashAddress, stretch);
                    break;
                case 16:
                    PutImage16BPP(left, top, flashAddress, stretch);
                    break;
            }
            break;
#endif
#ifdef USE_BITMAP_EXTERNAL
        case EXTERNAL:
            // Get color depth
            ExternalMemoryCallback(bitmap, 1, 1, &colorDepth);
            // Draw picture
            switch(colorDepth){
                case 1:
                    PutImage1BPPExt(left, top, bitmap, stretch);
                    break;
                case 4:
                    PutImage4BPPExt(left, top, bitmap, stretch);
                    break;
                case 8:
                    PutImage8BPPExt(left, top, bitmap, stretch);
                    break;
                case 16:
                    PutImage16BPPExt(left, top, bitmap, stretch);
                    break;
                default:
                    break;
            }
            break;
#endif
        default:
            break;
    }

    // Restore current color
    _color = colorTemp;
}

#ifdef USE_BITMAP_FLASH
/*********************************************************************
* Function: void PutImage1BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner,
*        bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage1BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch){
register DWORD address;
register FLASH_BYTE* flashAddress;
register FLASH_BYTE* tempFlashAddress;
BYTE temp;
WORD sizeX, sizeY;
WORD x,y;
BYTE stretchX,stretchY;
WORD pallete[2];
BYTE mask;

    // Move pointer to size information
    flashAddress = bitmap + 2;

    // Set start address
    address = (long)LINE_MEM_PITCH*top+ left;

    // Read image size
    sizeY = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;
    sizeX = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;
    pallete[0] = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;
    pallete[1] = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;

    CS_LAT_BIT = 0;
    for(y=0; y<sizeY; y++){
        tempFlashAddress = flashAddress;
        for(stretchY = 0; stretchY<stretch; stretchY++){
            flashAddress = tempFlashAddress;
            SetAddress(address);
            mask = 0;
            for(x=0; x<sizeX; x++){

                // Read 8 pixels from flash
                if(mask == 0){
                    temp = *flashAddress;
                    flashAddress++;
                    mask = 0x80;
                }
                
                // Set color
                if(mask&temp){
                    SetColor(pallete[1]);
                }else{
                    SetColor(pallete[0]);
                }

                // Write pixel to screen
                for(stretchX=0; stretchX<stretch; stretchX++){
                    WriteData(_color);
                }

                // Shift to the next pixel
                mask >>= 1;
           }
           address += LINE_MEM_PITCH; 
        }
    }
    CS_LAT_BIT = 1;
}

/*********************************************************************
* Function: void PutImage4BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs 16 color image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage4BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch){
register DWORD address;
register FLASH_BYTE* flashAddress;
register FLASH_BYTE* tempFlashAddress;
WORD sizeX, sizeY;
register WORD x,y;
BYTE temp;
register BYTE stretchX,stretchY;
WORD pallete[16];
WORD counter;

    // Move pointer to size information
    flashAddress = bitmap + 2;

    // Set start address
    address = (long)LINE_MEM_PITCH*top+ left;

    // Read image size
    sizeY = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;
    sizeX = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;

    // Read pallete
    for(counter=0;counter<16;counter++){
        pallete[counter] = *((FLASH_WORD*)flashAddress);
        flashAddress += 2;
    }

    CS_LAT_BIT = 0;     
    for(y=0; y<sizeY; y++){
        tempFlashAddress = flashAddress;
        for(stretchY = 0; stretchY<stretch; stretchY++){
            flashAddress = tempFlashAddress;
            SetAddress(address);
            for(x=0; x<sizeX; x++){
                // Read 2 pixels from flash
                if(x&0x0001){
                    // second pixel in byte
                    SetColor(pallete[temp>>4]);
                }else{
                    temp = *flashAddress;
                    flashAddress++;
                    // first pixel in byte
                    SetColor(pallete[temp&0x0f]);
                }

                // Write pixel to screen       
                for(stretchX=0; stretchX<stretch; stretchX++){
                    WriteData(_color);
                }

                // Shift to the next pixel
                //temp >>= 4;
            }
            address += LINE_MEM_PITCH;
        }
    }
    CS_LAT_BIT = 1;
}

/*********************************************************************
* Function: void PutImage8BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs 256 color image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage8BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch){
register DWORD address;
register FLASH_BYTE* flashAddress;
register FLASH_BYTE* tempFlashAddress;
WORD sizeX, sizeY;
WORD x,y;
BYTE temp;
BYTE stretchX, stretchY;
WORD pallete[256];
WORD counter;

    // Move pointer to size information
    flashAddress = bitmap + 2;

    // Set start address
    address = (long)LINE_MEM_PITCH*top+ left;

    // Read image size
    sizeY = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;
    sizeX = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;

    // Read pallete
    for(counter=0;counter<256;counter++){
        pallete[counter] = *((FLASH_WORD*)flashAddress);
        flashAddress += 2;
    }

    CS_LAT_BIT = 0;     
    for(y=0; y<sizeY; y++){
        tempFlashAddress = flashAddress;
        for(stretchY = 0; stretchY<stretch; stretchY++){
            flashAddress = tempFlashAddress;
            SetAddress(address);
            for(x=0; x<sizeX; x++){
                // Read pixels from flash
                temp = *flashAddress;
                flashAddress++;

                // Set color
                SetColor(pallete[temp]);

                // Write pixel to screen       
                for(stretchX=0; stretchX<stretch; stretchX++){
                    WriteData(_color);
                }
            }
            address += LINE_MEM_PITCH;
        }
    }
    CS_LAT_BIT = 1;
}

/*********************************************************************
* Function: void PutImage16BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs hicolor image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage16BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch){
register DWORD address;
register FLASH_WORD* flashAddress;
register FLASH_WORD* tempFlashAddress;
WORD sizeX, sizeY;
register WORD x,y;
WORD temp;
register BYTE stretchX,stretchY;

    // Move pointer to size information
    flashAddress = (FLASH_WORD*)bitmap + 1;

    // Set start address
    address = (long)LINE_MEM_PITCH*top+ left;

    // Read image size
    sizeY = *flashAddress;
    flashAddress++;
    sizeX = *flashAddress;
    flashAddress++;

    CS_LAT_BIT = 0;     
    for(y=0; y<sizeY; y++){
        tempFlashAddress = flashAddress;
        for(stretchY = 0; stretchY<stretch; stretchY++){
            flashAddress = tempFlashAddress;
            SetAddress(address);
            for(x=0; x<sizeX; x++){
                // Read pixels from flash
                temp = *flashAddress;
                flashAddress++;

                // Set color
                SetColor(temp);

                // Write pixel to screen       
                for(stretchX=0; stretchX<stretch; stretchX++){
                    WriteData(_color);
                }
            }
            address += LINE_MEM_PITCH;
        }
    }
    CS_LAT_BIT = 1;
}
#endif

#ifdef USE_BITMAP_EXTERNAL
/*********************************************************************
* Function: void PutImage1BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in external memory
*
********************************************************************/
void PutImage1BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch){
register DWORD      address;
register DWORD      memOffset;
BITMAP_HEADER       bmp;
WORD                pallete[2];
BYTE                lineBuffer[(SCREEN_HOR_SIZE/8)+1];
BYTE*               pData; 
SHORT               byteWidth;

BYTE                temp;
BYTE                mask;
WORD                sizeX, sizeY;
WORD                x,y;
BYTE                stretchX, stretchY;

    // Set start address
    address = (long)LINE_MEM_PITCH*top+ left;

    // Get bitmap header
    ExternalMemoryCallback(bitmap, 0, sizeof(BITMAP_HEADER), &bmp);

    // Get pallete (2 entries)
    ExternalMemoryCallback(bitmap, sizeof(BITMAP_HEADER), 2*sizeof(WORD), pallete);

    // Set offset to the image data
    memOffset = sizeof(BITMAP_HEADER) + 2*sizeof(WORD);

    // Line width in bytes
    byteWidth = bmp.width>>3;
    if(bmp.width&0x0007)
        byteWidth++;

    // Get size
    sizeX = bmp.width;
    sizeY = bmp.height;

    for(y=0; y<sizeY; y++){
        // Get line
        ExternalMemoryCallback(bitmap, memOffset, byteWidth, lineBuffer);
        memOffset += byteWidth;

        CS_LAT_BIT = 0;
        for(stretchY = 0; stretchY<stretch; stretchY++){
            pData = lineBuffer;
            SetAddress(address);
            mask = 0;
            for(x=0; x<sizeX; x++){

                // Read 8 pixels from flash
                if(mask == 0){
                    temp = *pData++;
                    mask = 0x80;
                }
                
                // Set color
                if(mask&temp){
                    SetColor(pallete[1]);
                }else{
                    SetColor(pallete[0]);
                }

                // Write pixel to screen
                for(stretchX=0; stretchX<stretch; stretchX++){
                    WriteData(_color);
                }

                // Shift to the next pixel
                mask >>= 1;
           }
           address += LINE_MEM_PITCH; 
        }
        CS_LAT_BIT = 1;
    }

}

/*********************************************************************
* Function: void PutImage4BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in external memory
*
********************************************************************/
void PutImage4BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch){
register DWORD      address;
register DWORD      memOffset;
BITMAP_HEADER       bmp;
WORD                pallete[16];
BYTE                lineBuffer[(SCREEN_HOR_SIZE/2)+1];
BYTE*               pData; 
SHORT               byteWidth;

BYTE                temp;
WORD                sizeX, sizeY;
WORD                x,y;
BYTE                stretchX, stretchY;

    // Set start address
    address = (long)LINE_MEM_PITCH*top+ left;

    // Get bitmap header
    ExternalMemoryCallback(bitmap, 0, sizeof(BITMAP_HEADER), &bmp);

    // Get pallete (16 entries)
    ExternalMemoryCallback(bitmap, sizeof(BITMAP_HEADER), 16*sizeof(WORD), pallete);

    // Set offset to the image data
    memOffset = sizeof(BITMAP_HEADER) + 16*sizeof(WORD);

    // Line width in bytes
    byteWidth = bmp.width>>1;
    if(bmp.width&0x0001)
        byteWidth++;

    // Get size
    sizeX = bmp.width;
    sizeY = bmp.height;

    for(y=0; y<sizeY; y++){

        // Get line
        ExternalMemoryCallback(bitmap, memOffset, byteWidth, lineBuffer);
        memOffset += byteWidth;
        CS_LAT_BIT = 0;
        for(stretchY = 0; stretchY<stretch; stretchY++){

            pData = lineBuffer;
            SetAddress(address);

            for(x=0; x<sizeX; x++){

                // Read 2 pixels from flash
                if(x&0x0001){
                    // second pixel in byte
                    SetColor(pallete[temp>>4]);
                }else{
                    temp = *pData++;
                    // first pixel in byte
                    SetColor(pallete[temp&0x0f]);
                }

                // Write pixel to screen       
                for(stretchX=0; stretchX<stretch; stretchX++){
                    WriteData(_color);
                }
           }
           address += LINE_MEM_PITCH; 
        }
        CS_LAT_BIT = 1;
    }
}

/*********************************************************************
* Function: void PutImage8BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in external memory
*
********************************************************************/
void PutImage8BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch){
register DWORD      address;
register DWORD      memOffset;
BITMAP_HEADER       bmp;
WORD                pallete[256];
BYTE                lineBuffer[SCREEN_HOR_SIZE];
BYTE*               pData; 

BYTE                temp;
WORD                sizeX, sizeY;
WORD                x,y;
BYTE                stretchX, stretchY;

    // Set start address
    address = (long)LINE_MEM_PITCH*top+ left;

    // Get bitmap header
    ExternalMemoryCallback(bitmap, 0, sizeof(BITMAP_HEADER), &bmp);

    // Get pallete (256 entries)
    ExternalMemoryCallback(bitmap, sizeof(BITMAP_HEADER), 256*sizeof(WORD), pallete);

    // Set offset to the image data
    memOffset = sizeof(BITMAP_HEADER) + 256*sizeof(WORD);

    // Get size
    sizeX = bmp.width;
    sizeY = bmp.height;

    for(y=0; y<sizeY; y++){

        // Get line
        ExternalMemoryCallback(bitmap, memOffset, sizeX, lineBuffer);
        memOffset += sizeX;
        CS_LAT_BIT = 0;
        for(stretchY = 0; stretchY<stretch; stretchY++){

            pData = lineBuffer;
            SetAddress(address);

            for(x=0; x<sizeX; x++){

                temp = *pData++;
                SetColor(pallete[temp]);                    

                // Write pixel to screen       
                for(stretchX=0; stretchX<stretch; stretchX++){
                    WriteData(_color);
                }
           }
           address += LINE_MEM_PITCH; 
        }
        CS_LAT_BIT = 1;
    }
}

/*********************************************************************
* Function: void PutImage16BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch)
*
* PreCondition: none
*
* Input: left,top - left top image corner, bitmap - image pointer,
*        stretch - image stretch factor
*
* Output: none
*
* Side Effects: none
*
* Overview: outputs monochrome image starting from left,top coordinates
*
* Note: image must be located in external memory
*
********************************************************************/
void PutImage16BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch){
register DWORD      address;
register DWORD      memOffset;
BITMAP_HEADER       bmp;
WORD                lineBuffer[SCREEN_HOR_SIZE];
WORD*               pData; 
WORD                byteWidth;

WORD                temp;
WORD                sizeX, sizeY;
WORD                x,y;
BYTE                stretchX, stretchY;

    // Set start address
    address = (long)LINE_MEM_PITCH*top+ left;

    // Get bitmap header
    ExternalMemoryCallback(bitmap, 0, sizeof(BITMAP_HEADER), &bmp);

    // Set offset to the image data
    memOffset = sizeof(BITMAP_HEADER);

    // Get size
    sizeX = bmp.width;
    sizeY = bmp.height;

    byteWidth = sizeX<<1;

    for(y=0; y<sizeY; y++){
        // Get line
        ExternalMemoryCallback(bitmap, memOffset, byteWidth, lineBuffer);
        memOffset += byteWidth;
        CS_LAT_BIT = 0;
        for(stretchY = 0; stretchY<stretch; stretchY++){

            pData = lineBuffer;
            SetAddress(address);

            for(x=0; x<sizeX; x++){

                temp = *pData++;
                SetColor(temp);                    

                // Write pixel to screen       
                for(stretchX=0; stretchX<stretch; stretchX++){
                    WriteData(_color);
                }

           }

           address += LINE_MEM_PITCH; 

        }
        CS_LAT_BIT = 1;
    }
}
#endif
