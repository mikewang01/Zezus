
/************************************************************
Copyright (C), 2008-2014, Colorful Sea Tech. Co., Ltd.
FileName:       manufacture_task.c
Author:      MikeWang   Version : 0.0          Date: 2014/8/18
Description:  this file is aimed at managing  ios used in this project  
Version:  0.0        
Function List:   


task init function;
   void  os_gpio_init()
   
   
   
1. -------History:         
<author>   <time>    <version >    <desc>
Mike      14/8/18      0.0       create this file  
***********************************************************/

/*********************************************************************
 * INCLUDES
 */ 

#include "kernel-includes.h"
#include "drivers-includes.h"
#include "manufacture_task.h"
#include "stdio.h"
#include "gpio_mgr.h"
/*********************************************************************
* MACROS
*/


#define  DEFAULT_GPIO_AF     GPIO_AF_0
#define  DEFAULT_GPIO_OTYPE  GPIO_OType_OD
#define  DEFAULT_GPIO_SPEED  GPIO_Speed_2MHz
#define  DEFAULT_GPIO_PUPD   GPIO_PuPd_NOPULL

/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
/*define io manager structor
typedef struct
{
  u8 gpio_num;
    u8 gpio_mode;
    u8 gpio_output_type;
  u8 gpio_speed;
    u8 gpio_pupd;
    
}io_config_t;*/
static const io_config_t porta_gpio_config[] =
{
    {IO_0, GPIO_Mode_IN,  DEFAULT_GPIO_AF, DEFAULT_GPIO_OTYPE, DEFAULT_GPIO_SPEED, GPIO_PuPd_UP},      //PA0******* INPUT    :PALY_ON PIN TO WAKE UP DEVICE
    {IO_1, GPIO_Mode_AN,  DEFAULT_GPIO_AF, DEFAULT_GPIO_OTYPE, DEFAULT_GPIO_SPEED, GPIO_PuPd_NOPULL},  //PA1******* ADC     :BAT DETECT PIN
    {IO_2, GPIO_Mode_AF,  GPIO_AF_1,       GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PA2*******        USART2_TX
    {IO_3, GPIO_Mode_AF,  GPIO_AF_1,       GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PA3*******        USART2_RX
    {IO_4, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,      GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PA4*******  OUTPUT :AMP_ENABLE
    {IO_5, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,      GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PA4*******  OUTPUT :SENSOR_SHORT CONTROL SIGNAL
    {IO_6, GPIO_Mode_AN,  DEFAULT_GPIO_AF, DEFAULT_GPIO_OTYPE, DEFAULT_GPIO_SPEED, GPIO_PuPd_NOPULL},  //PA6******* ADC:AIR_SENSOR_ADC
    {IO_7, GPIO_Mode_AN,  DEFAULT_GPIO_AF, DEFAULT_GPIO_OTYPE, DEFAULT_GPIO_SPEED, GPIO_PuPd_NOPULL},  //PA7******* ADC:ADC_REF
    {IO_8, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,      GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PA8*******  OUTPUT :LED_B
    {IO_9, GPIO_Mode_AF,  GPIO_AF_1,       GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PA9*******       USART1_TX
    {IO_10, GPIO_Mode_AF, GPIO_AF_1,      GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PA10******       USART1_RX 
    {IO_11, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,      GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PA11*******  OUTPUT :LED_R
    {IO_12, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,      GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PA12******  OUTPUT :DIS_TS_PENABLE
    
};

static const io_config_t portb_gpio_config[] =
{
    {IO_0, GPIO_Mode_IN,  DEFAULT_GPIO_AF, DEFAULT_GPIO_OTYPE,  DEFAULT_GPIO_SPEED, GPIO_PuPd_UP},      //PB0******* INPUT    :CHARGE_OK SIGNAL FROM POWER MANAGEMENT IC
    {IO_1, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,       GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PB1*******  OUTPUT :  PWR_HOLD
    {IO_2, GPIO_Mode_IN,  DEFAULT_GPIO_AF, DEFAULT_GPIO_OTYPE,  DEFAULT_GPIO_SPEED, GPIO_PuPd_UP},      //PB0******* INPUT    :#BATLOW
    {IO_3, GPIO_Mode_AF,  GPIO_AF_0,       GPIO_OType_PP,       GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PB3*******     MB_SPI1_SCK 
    {IO_4, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,       GPIO_Speed_50MHz,    GPIO_PuPd_UP},     //PB4*******  OUTPUT :  MB_SPI1_DC
    {IO_5, GPIO_Mode_AF,  GPIO_AF_0,       GPIO_OType_PP,       GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PB5*******     MB_SPI1_MOSI 
    {IO_6, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,       GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PB6*******  OUTPUT :  OLED RESET SIGNAL
    {IO_7, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,       GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PB7*******  OUTPUT :LED_G
    {IO_8, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,       GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PB8*******  OUTPUT :WD_ENABLE
    {IO_9, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,       GPIO_Speed_2MHz,    GPIO_PuPd_UP},     //PB9*******  OUTPUT :WD_FEED
    {IO_10, GPIO_Mode_IN,  DEFAULT_GPIO_AF, DEFAULT_GPIO_OTYPE, DEFAULT_GPIO_SPEED, GPIO_PuPd_UP},      //PB10******* INPUT    :USB_PLUG_DET
    {IO_11, GPIO_Mode_IN,  DEFAULT_GPIO_AF, DEFAULT_GPIO_OTYPE, DEFAULT_GPIO_SPEED, DEFAULT_GPIO_PUPD},      //PB11******* FLOAT PIN
    {IO_12, GPIO_Mode_OUT, DEFAULT_GPIO_AF, GPIO_OType_PP,      GPIO_Speed_50MHz,    GPIO_PuPd_UP},     //PB12*******  OUTPUT :  MB_SPI_FLASH_CS
    {IO_13, GPIO_Mode_AF,  GPIO_AF_0,       GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PB13*******     MB_SPI2_SCK 
    {IO_14, GPIO_Mode_AF,  GPIO_AF_0,       GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PB14*******     MB_SPI2_MISO
    {IO_15, GPIO_Mode_AF,  GPIO_AF_0,       GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PB15*******     MB_SPI2_MOSI
    
};

static const io_config_t portf_gpio_config[] =
{
    {IO_6, GPIO_Mode_AF,  GPIO_AF_1,       GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PF6*******     I2C2_SCL
    {IO_7, GPIO_Mode_AF,  GPIO_AF_1,       GPIO_OType_PP,      GPIO_Speed_50MHz,   GPIO_PuPd_UP},     //PF7*******     I2C2_SDA
    
};



/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
*/

/*********************************************************************
 * @fn      os_gpio_init()
 *
 * @brief   This function initiate vvery detailed configured gpio listed in conifg arry
 *
 * @param   void
 *
 * @return  none
 */
void os_gpio_init()
{
    u8 gpio_num_total=0; 
    u8 i=0;
    
    /*gpio initiate struct*/
    GPIO_InitTypeDef    GPIO_InitStructure;
    
    
    /*PORTA INITILAZE PEOCESS*/
    /*total gpio number needed initializing invoked in  gpio arry*/	 
    gpio_num_total = sizeof(porta_gpio_config)/sizeof(porta_gpio_config[0]);
    /*if there are pins need configuring , open the corresponed clock*/
    if(gpio_num_total >0)
    {
        os_clock_open("GPIOA");
    }
    for(i=0 ; i<gpio_num_total ; i++)	
    {
        
        GPIO_InitStructure.GPIO_Pin   =  BIT_SHIFT(porta_gpio_config[i].gpio_num);
        GPIO_InitStructure.GPIO_Mode  =  porta_gpio_config[i].gpio_mode;
        GPIO_InitStructure.GPIO_OType =  porta_gpio_config[i].gpio_output_type;
        GPIO_InitStructure.GPIO_Speed =  porta_gpio_config[i].gpio_speed;
        GPIO_InitStructure.GPIO_PuPd = porta_gpio_config[i].gpio_pupd ;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        /*if cuurent pin need to configure as af pin ,specific af mode initialization is needed here*/
        if(porta_gpio_config[i].gpio_mode==GPIO_Mode_AF)
        {
            GPIO_PinAFConfig(GPIOA , porta_gpio_config[i].gpio_num , porta_gpio_config[i].gpio_af); 
        }
    }
    
    /*PORTB INITILAZE PEOCESS*/		
    /*total gpio number needed initializing invoked in  gpio arry*/	 
    gpio_num_total = sizeof(portb_gpio_config)/sizeof(portb_gpio_config[0]);
    /*if there are pins need configuring , open the corresponed clock*/
    if(gpio_num_total >0)
    {
        os_clock_open("GPIOB");
    }
    for(i=0 ; i<gpio_num_total ; i++)	
    {
        
        GPIO_InitStructure.GPIO_Pin   =  BIT_SHIFT(portb_gpio_config[i].gpio_num);
        GPIO_InitStructure.GPIO_Mode  =  portb_gpio_config[i].gpio_mode;
        GPIO_InitStructure.GPIO_OType =  portb_gpio_config[i].gpio_output_type;
        GPIO_InitStructure.GPIO_Speed =  portb_gpio_config[i].gpio_speed;
        GPIO_InitStructure.GPIO_PuPd = portb_gpio_config[i].gpio_pupd ;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        /*if cuurent pin need to configure as af pin ,specific af mode initialization is needed here*/
        if(portb_gpio_config[i].gpio_mode==GPIO_Mode_AF)
        {
            GPIO_PinAFConfig(GPIOB , portb_gpio_config[i].gpio_num , portb_gpio_config[i].gpio_af); 
        }
    }	
    
    /*PORTF INITILAZE PEOCESS*/			
    
    /*total gpio number needed initializing invoked in  gpio arry*/	 
    gpio_num_total = sizeof(portf_gpio_config)/sizeof(portf_gpio_config[0]);
    /*if there are pins need configuring , open the corresponed clock*/
    if(gpio_num_total >0)
    {
        os_clock_open("GPIOF");
    }
    for(i=0 ; i<gpio_num_total ; i++)	
    {
        
        GPIO_InitStructure.GPIO_Pin   = BIT_SHIFT(portf_gpio_config[i].gpio_num);
        GPIO_InitStructure.GPIO_Mode  =  portf_gpio_config[i].gpio_mode;
        GPIO_InitStructure.GPIO_OType =  portf_gpio_config[i].gpio_output_type;
        GPIO_InitStructure.GPIO_Speed =  portf_gpio_config[i].gpio_speed;
        GPIO_InitStructure.GPIO_PuPd = portf_gpio_config[i].gpio_pupd ;
        GPIO_Init(GPIOF, &GPIO_InitStructure);
        /*if cuurent pin need to configure as af pin ,specific af mode initialization is needed here*/
        if(portf_gpio_config[i].gpio_mode==GPIO_Mode_AF)
        {
            GPIO_PinAFConfig(GPIOF , portf_gpio_config[i].gpio_num , portf_gpio_config[i].gpio_af); 
        }
    }
    
}









