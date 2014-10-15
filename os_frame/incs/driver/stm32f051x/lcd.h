/***************************************************************
 * Name:      spi.c
 * Purpose:   code for spi bus
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */


#ifndef _LCD_H
#define _LCD_H
/* Includes ------------------------------------------------------------------*/
#include "cpu.h"
#include "kernel-includes.h"
#include "drivers-includes.h"
/* Exported constants --------------------------------------------------------*/

#define __FONT_16X8_
//#define _FONT6X8_
#define _LCD_BUFFER_USING_

#define LCD_X_SOLUTON        128
#define LCD_Y_SOLUTON        64


#ifdef __FONT_16X8_
 #define FONT_Y_MODEL 16
 #define FONT_X_MODEL 8
#endif


#ifdef _FONT6X8_
 #define FONT_Y_MODEL 6
 #define FONT_X_MODEL 8
#endif



enum com_data{lcd_command=0, lcd_x_inv , lcd_y_inv , lcd_clear , lcd_pwrdown , lcd_wakeup , lcd_refresh};

enum lcd_status{lcd_normal, lcd_reset , lcd_sleep};

/*pixel fill mode*/
enum lcd_pixel{pixel_clear=0x00 ,pixel_fill=0x01 };


typedef struct
{
		u16 x_cord;
	  u16 y_cord;
}curor_pos;

typedef struct
{
		u16 x_pixel;
	  u16 y_pixel;
}solution;

typedef struct
{
		u8 x_direction;
	  u8 y_direction;
}scan_mode;

/* Exported types ------------------------------------------------------------*/

typedef struct
{
  u8 font_type;
	u8 (*font_ptr)[FONT_Y_MODEL];
	u8 font_x_model;
	u8 font_y_model;
} font_property;



typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
	solution lcd_solution;
	scan_mode lcd_scan_mode;
	font_property font;
  u8 current_status;
  curor_pos  cursor_record;	
	u8 (*buffer)[LCD_Y_SOLUTON>>3];
}lcd_device;



/*lcd command list*/
#define  LCD_COLADDR_LSB		     0x01 //set CA[0-3]
#define  LCD_COLADDR_MSB 		   0x10 // set   CA[4-7]
#define  LCD_POWER_CTRL  		   0X28  // SET PC[0-2]
#define  LCD_SET_SCOLLLINE      0x40  //set sl[5:0]
#define  LCD_PAGE_ADDRESS       0XB0  //SET PA[3:0]
#define  LCD_SET_VLCD_RADIO     0X20  // SET PC[5£º3]
#define  LCD_ALL_PIXEL_ON       0XA4  //SET DC[1]
#define  LCD_INVERSE_DIS        0xA6  //SET DC[0]
#define  LCD_DISPLAY_EN			   0XAE  //SET DC[2]
#define  LCD_SEG_DIRECTION      0XA0  //SET LC[0]
#define  LCD_COM_DIRECTION      0Xc0  //SET LC[1]
#define  LCD_SYSTEM_RESET       0XE2  //SYSTEM RESET
#define  LCD_NOP						     0XE3  //NO OPERATION
#define  LCD_BIAS_RATIO         0XA2 //SET BR
#define  LCD_SET_CURSORUPDATE   0XE0 //AC3=1,CR=CA
#define  LCD_RESET_CURSORUPDATE 0XEE //AC3=0,CR=CA

#define  LCD_LEFT2RIGHT       0X00    
#define  LCD_RIGHT2LEFT       0X01
#define  LCD_UP2DOWN          0X08
#define  LCD_DOWN2UP          0X00

#define  LCD_DISPLAY_ON       0x01
#define  LCD_DISPLAY_OFF      0X00

#define  lcd_software_reset() do{transfer_command(LCD_SYSTEM_RESET);}while(0)

/*x : hrizondirection LCD_LEFT2RIGHT OR LCD_RIGHT2LEFT y: vertical scan direction LCD_UP2DOWN OR LCD_DOWN2UP*/
#define  lcd_set_scroll_directon(x,y) do{transfer_command(LCD_COM_DIRECTION|y);transfer_command(LCD_SEG_DIRECTION|x);}while(0)


/*x : onoff parameter LCD_DISPLAY_ON or LCD_DISPLAY_OFF*/
#define  lcd_display_on_off(x) do{transfer_command(LCD_DISPLAY_EN|x);}while(0)

#define  lcd_enter_powersavemode()do{ lcd_display_on_off(LCD_DISPLAY_OFF)\
																			transfer_command(LCD_ALL_PIXEL_ON);\
																		 }while(0)


#define  lcd_powercontraist_set()do{\
														transfer_command(0x2c);\
														transfer_command(0x2e);\
														transfer_command(0x2f);\
														transfer_command(0x23);\
														transfer_command(0x81);\
														transfer_command(0x28);\
														transfer_command(0xa2);\
													}while(0)

/*x : collum adress ranging from 0 to 131*/
#define   lcd_collumaddress_set(x) do{\
																		transfer_command(LCD_COLADDR_LSB | (x&0x0f));\
																		transfer_command(LCD_COLADDR_MSB | ((x>>4)&0x0f));\
                                  }while(0)

/*y : page adress ranging from 0 to 8*/

#define   lcd_pageaddress_set(y)  do{\
																		transfer_command(LCD_PAGE_ADDRESS | y);\
                                  }while(0)


/*x : page adress ranging from 0 to 131*/
/*y : page adress ranging from 0 to 8*/
#define lcd_position_set(x , y) do{\
																		transfer_command(LCD_PAGE_ADDRESS + (y));\
																		transfer_command(LCD_COLADDR_MSB | ((x>>4)&0x0f));\
	                                  transfer_command(LCD_COLADDR_LSB | (x&0x0f));\
                                  }while(0)

extern os_err_t lcd_register(u16 task_id);


#endif

