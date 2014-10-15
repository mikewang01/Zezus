/*****************************************************************************
 *  Module for Microchip Graphics Library
 *  Solomon Systech SSD1303 LCD controller driver
 *  Landscape orientation
 *****************************************************************************
 * FileName:        SSD1303.c
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
 * Paolo Tamayo			11/13/07	Ported to PIC24 Kit
 *****************************************************************************/
#include "Graphics.h"

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
#define DELAY_1MS 16000/9
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

    
    DelayMs(1);
}


void ScreenSaver(BYTE enable)
{
	
	return;
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
extern BYTE lcd_read_point(SHORT x,SHORT y);
extern BYTE lcd_drawpoint(SHORT x,SHORT y,BYTE pixel);
void PutPixel(SHORT x, SHORT y) {
BYTE mask, display;


 	mask = 1;				// Left shift 1 by bit position

/* Read data from display buffer*/
  display = lcd_read_point(x ,y);

	if(_color > 0)					// If non-zero for pixel on
		display |= mask;			// or in mask
	else							// If 0 for pixel off
		display &= ~mask;			// and with inverted mask
	  lcd_drawpoint(x , y,display);
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

BYTE mask, display;


	mask = 1 ;				// Left shift 1 by bit position

  display = lcd_read_point(x ,y);
	return (display & mask);		// mask all other bits and return the result	
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
/*
void Bar(SHORT left, SHORT top, SHORT right, SHORT bottom){
DWORD_VAL address;
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

    top = GetMaxY() - top;
    bottom = GetMaxY() - bottom;
 
    address.Val = (DWORD)LINE_MEM_PITCH*left + top;

    CS_LAT_BIT = 0;
    for(y=bottom; y<top+1; y++){
        SetAddress(address.v[2],address.v[1],address.v[0]);
        for(x=left; x<right+1; x++){
            WriteData(_color.v[1],_color.v[0]);
        }
        address.Val -= 1;
    }
    CS_LAT_BIT = 1;
}
*/
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
extern void clear_screen(BYTE data) ;
void ClearDevice(void){

	clear_screen(_color);
	
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
        case FLASH_I:
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




/*****************************************************************

USER SPACE
*************************************************************/
#define SIG_STATE_SET   0
#define SIG_STATE_DRAW  1
WORD GOLDrawCallback(){

        return 1;
    
}           


WORD GOLMsgCallback(WORD objMsg, OBJ_HEADER* pObj, GOL_MSG *pMsg){
  return 1;
}   


WORD RdiaDraw(
    ROUNDDIAL * pDia
){  return 1;}
	
void RdiaMsgDefault(
    WORD translatedMsg, 
    ROUNDDIAL * pDia, 
    GOL_MSG* pMsg
){  }
	
WORD RdiaTranslateMsg(
    ROUNDDIAL * pDia, 
    GOL_MSG * pMsg
){  return 1;}
	
WORD ExternalMemoryCallback(
    EXTDATA* memory, 
    LONG offset, 
    WORD nCount, 
    void* buffer
){  return 1;}	
	
	
	


