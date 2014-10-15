/*****************************************************************************
 *  Module for Microchip Graphics Library
 *  Sino Wealth Microelectronic SH1101A OLED controller driver  
 *  Landscape orientation
 *****************************************************************************
 * FileName:        SH1101A.c
 * Dependencies:    Graphics.h
 * Processor:       PIC24
 * Compiler:       	MPLAB C30
 * Linker:          MPLAB LINK30
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
 * Rodger Richey		03/10/07	Original
 * Paolo Tamayo			12/20/07	Ported to PIC24 Kit
 *****************************************************************************/
#include "Graphics\Graphics.h"

// Color
BYTE    _color;
// Clipping region control
SHORT _clipRgn;
// Clipping region borders
SHORT _clipLeft;
SHORT _clipTop;
SHORT _clipRight;
SHORT _clipBottom;

/////////////////////// LOCAL FUNCTIONS PROTOTYPES ////////////////////////////
void PutImage1BPP(SHORT left, SHORT top, FLASH_BYTE* bitmap, BYTE stretch);
void PutImage1BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch);

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
#if (GetSystemClock() == 32000000)
#define DELAY_1MS 32000/9
#else
#define DELAY_1MS 16000/9
#endif
void  DelayMs(WORD time){
unsigned delay;
	while(time--)
		for(delay=0; delay<DELAY_1MS; delay++);	
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

    // Set reset pin as output
    RST_TRIS_BIT = 0;
    // Hold in reset
    RST_LAT_BIT = 0;
    
    // PMP setup 
    PMMODEbits.MODE  = 0b10;	// Master 2, controller pins should be set BS0=0,BS1=1,BS2=1 on board
#if (GetSystemClock() == 32000000)
    PMMODEbits.WAITB = 0b01;	// 1 tcy data set up
    PMMODEbits.WAITM = 0b010;	// 2 tcy additional wait 
    PMMODEbits.WAITE = 0b00;	// 1 tcy data hold 
#else
    PMMODEbits.WAITB = 0b01;	// 1 tcy data set up
    PMMODEbits.WAITM = 0b001;	// 1 tcy additional wait 
    PMMODEbits.WAITE = 0b00;	// 1 tcy data hold 
#endif

    PMAENbits.PTEN0  = 1;		// Address line 0 is enabled
    PMAENbits.PTEN14 = 1;		// Address line 14 is enabled and will be used as CS1 as set in PMMODEbits.MODE
    PMCONbits.CSF    = 0b10;	// set CS1 as chip select
    PMCONbits.PTRDEN = 1;		// enable read strobe
    PMCONbits.PTWREN = 1;		// enable write strobe
    PMCONbits.PMPEN  = 1;		// enable PMP module

    // Reset controller
    RST_LAT_BIT = 1;
    DelayMs(20);
    RST_LAT_BIT = 0;
    DelayMs(20);
    RST_LAT_BIT = 1;
    DelayMs(20);

	// Setup Display
	WriteCommand(0xAE);			// turn off the display (AF=ON, AE=OFF)
	
	WriteCommand(0xDB);			// set  VCOMH
	WriteCommand(0x23);			 

	WriteCommand(0xD9);			// set  VP
	WriteCommand(0x22);			 

	//////////////////////////////
	// User Set Up
	//////////////////////////////

	// Re-map
	WriteCommand(0xA1);			// [A0]:column address 0 is map to SEG0
								// [A1]:column address 131 is map to SEG0

	// COM Output Scan Direction
	WriteCommand(0xC8);			// C0 is COM0 to COMn, C8 is COMn to COM0

	// COM Pins Hardware Configuration
	WriteCommand(0xDA);			// set pins hardware configuration
	WriteCommand(0x12);

	// Multiplex Ratio
	WriteCommand(0xA8);			// set multiplex ratio
	WriteCommand(0x3F);			// set to 64 mux

	// Display Clock Divide
	WriteCommand(0xD5);			// set display clock divide
	WriteCommand(0xA0);			// set to 100Hz

	// Contrast Control Register
	WriteCommand(0x81);			// Set contrast control
	WriteCommand(0x60);			// display 0 ~ 127; 2C

	// Display Offset
	WriteCommand(0xD3);			// set display offset
	WriteCommand(0x00);			// no offset
	
	//Normal or Inverse Display
	WriteCommand(0xA6);			// Normal display

	WriteCommand(0xAD);			// Set DC-DC
	WriteCommand(0x8B);			// 8B=ON, 8A=OFF 
	
	// Display ON/OFF
	WriteCommand(0xAF);			// AF=ON, AE=OFF
	DelayMs(150);

	// Entire Display ON/OFF
	WriteCommand(0xA4);			// A4=ON

	// Display Start Line
	WriteCommand(0x40);			// Set display start line

	// Lower Column Address
	WriteCommand(0x00+OFFSET);	// Set lower column address

	// Higher Column Address
	WriteCommand(0x10);			// Set higher column address

    DelayMs(1);
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
void PutPixel(SHORT x, SHORT y) {
BYTE page, add, lAddr, hAddr;
BYTE mask, display;

	// check if point is in clipping region
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

	// Assign a page address
    	 if(y < 8)  page = 0xB0;
	else if(y < 16) page = 0xB1;
	else if(y < 24) page = 0xB2;
	else if(y < 32) page = 0xB3;
	else if(y < 40) page = 0xB4;
	else if(y < 48) page = 0xB5;
	else if(y < 56) page = 0xB6;
	else            page = 0xB7;

	add = x+OFFSET;
	lAddr = 0x0F & add;				// Low address
	hAddr = 0x10 | (add >> 4);		// High address

	// Calculate mask from rows basically do a y%8 and remainder is bit position
	add = y>>3;						// Divide by 8
	add <<= 3;						// Multiply by 8
	add = y - add;					// Calculate bit position
	mask = 1 << add;				// Left shift 1 by bit position

	SetAddress(page, lAddr, hAddr);	// Set the address (sets the page, 
									// lower and higher column address pointers)

	ReadData(display);				// Read to initiate Read transaction on PMP
	ReadData(display);				// Dummy Read (requirement for data synchronization in the controller)
	ReadData(display);				// Read data from display buffer

	if(_color > 0)					// If non-zero for pixel on
		display |= mask;			// or in mask
	else							// If 0 for pixel off
		display &= ~mask;			// and with inverted mask
		
	WriteCommand(lAddr);			// Set column address low
	WriteCommand(hAddr);			// Set column address high

	WriteData(display);				// restore the byte with manipulated bit
}

/*********************************************************************
* Function: BYTE GetPixel(SHORT x, SHORT y)
*
* PreCondition: none
*
* Input: x,y - pixel coordinates 
*
* Output: pixel color
*
* Side Effects: none
*
* Overview: return pixel color at x,y position
*
* Note: none
*
********************************************************************/
BYTE GetPixel(SHORT x, SHORT y){
BYTE page, add, lAddr, hAddr;
BYTE mask, temp, display;

	// check if point is in clipping region
    if(_clipRgn){
        if(x<_clipLeft)
            return 0;
        if(x>_clipRight)
            return 0;
        if(y<_clipTop)
            return 0;
        if(y>_clipBottom)
            return 0;
    }

	// Assign a page address
    	 if(y < 8)  page = 0xB0;
	else if(y < 16) page = 0xB1;
	else if(y < 24) page = 0xB2;
	else if(y < 32) page = 0xB3;
	else if(y < 40) page = 0xB4;
	else if(y < 48) page = 0xB5;
	else if(y < 56) page = 0xB6;
	else            page = 0xB7;

	add = x+OFFSET;
	lAddr = 0x0F & add;				// Low address
	hAddr = 0x10 | (add >> 4);		// High address

	// Calculate mask from rows basically do a y%8 and remainder is bit position
	temp = y>>3;					// Divide by 8
	temp <<= 3;						// Multiply by 8
	temp = y - temp;				// Calculate bit position
	mask = 1 << temp;				// Left shift 1 by bit position

	SetAddress(page, lAddr, hAddr);	// Set the address (sets the page, 
									// lower and higher column address pointers)
	ReadData(display);				// Read to initiate Read transaction on PMP
	ReadData(display);				// Dummy Read (requirement for data synchronization in the controller)
	ReadData(display);				// Read data from display buffer

	return (display & mask);		// mask all other bits and return the result	
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
	BYTE i,j;

	for(i=0xB0;i<0xB8;i++) {			// Go through all 8 pages
		SetAddress(i,0x00,0x10);
		for(j=0;j<132;j++) {			// Write to all 132 bytes
			WriteData(_color);			
		}
	}
}

/*********************************************************************
* Function: void PutImage(SHORT left, SHORT top, void* bitmap, BYTE stretch)
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
* Overview: outputs image starting from left,top coordinates
*
* Note: image must be located in flash
*
********************************************************************/
void PutImage(SHORT left, SHORT top, void* bitmap, BYTE stretch){
FLASH_BYTE* flashAddress;
BYTE colorTemp;

    //top = GetMaxY() - top; 

    // Save current color
    colorTemp = _color;

    switch(*((SHORT*)bitmap))
    {
#ifdef USE_BITMAP_FLASH
        case FLASH:
            // Image address
            flashAddress = ((BITMAP_FLASH*)bitmap)->address;
            PutImage1BPP(left, top, flashAddress, stretch);
            break;
#endif
#ifdef USE_BITMAP_EXTERNAL
        case EXTERNAL:
            // Get color depth
            ExternalMemoryCallback(bitmap, 1, 1, &colorDepth);
            // Draw picture
            PutImage1BPPExt(left, top, bitmap, stretch);
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
* Input: left,top - left top image corner, bitmap - image pointer,
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
register WORD_VAL address;
register FLASH_BYTE* flashAddress;

register BYTE pos, temp;

WORD sizeX, sizeY;
WORD x,y;
//BYTE stretchX,stretchY;
WORD pallete[2];

    // Move pointer to size information
    flashAddress = bitmap + 2;

    // Set start address
    address.Val = (LINE_MEM_PITCH*left) + top;

    // Read image size
    sizeY = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;
    sizeX = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;
    pallete[0] = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;
    pallete[1] = *((FLASH_WORD*)flashAddress);
    flashAddress += 2;


	for (y=0; y<sizeY;y++) {
		for(x=0;x<sizeX;x+=8) {
	        temp = *flashAddress;
	        
			for(pos=x;pos<(8+x);pos++) {
				if (pos >= sizeX) {
					x = pos;
					break;
				}
				if ((temp&0x80) != 0)
	            	_color = 0;
	            else	
	            	_color = 1;
	            PutPixel(pos+left,y+top);
	            temp = temp << 1;	
	    	} 
	        flashAddress++;
		}
		//address.Val += 1; 
	}
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
* Note: image must be located in flash
*
********************************************************************/
void PutImage1BPPExt(SHORT left, SHORT top, void* bitmap, BYTE stretch){
register DWORD_VAL  address;
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
    address.Val = (long)LINE_MEM_PITCH*left+ top;

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

        for(stretchY = 0; stretchY<stretch; stretchY++){
            pData = lineBuffer;
            SetAddress(address.v[2],address.v[1],address.v[0]);
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
                    WriteData(_color.v[1],_color.v[0]);
                }

                // Shift to the next pixel
                mask >>= 1;
           }
           address.Val -= 1; 
        }
    }
}
#endif
