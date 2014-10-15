/***************************************************************
 * Name:      LENTHMEASURE.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "led.h"

/*********************************************************************
* MACROS
*/
#define OS_TICKS_PER_SEC 1000


/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
 

static GPIO_TypeDef *led_ports[]={LED1_PORT , LED2_PORT };//, LED2_PORT , LED3_PORT};

static const u8 led_pins[]={LED1_PIN , LED2_PIN};

/*********************************************************************
 * EXTERNAL VARIABLES
 */
/*********************************************************************
 * FUNCTIONS
 */



/*********************************************************************
 * @fn      void led_init
 *
 * @brief   SYSTICK的时钟固定为HCLK时钟的1/8
 *					SYSCLK:系统时钟
 *					SYSTICK的时钟固定为HCLK时钟的1/8
 * 			    @param SYSCLK system clock from clock, take the union of Mhz 
 *			
 * 
 *   @return none 
 */	


void led_init(void)
{
    
//    os_clock_open("GPIOA"); 
//    os_clock_open("GPIOB"); 
//    /* Speed mode configuration */
//    LED1_PORT->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (LED1_PIN * 2));
//    LED1_PORT->OSPEEDR |= ((u32)(GPIO_Speed_2MHz) <<(LED1_PIN * 2));
//    
//    /* Output mode configuration */
//    LED1_PORT->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((u16) LED1_PIN));
//    
//    LED1_PORT->OTYPER |= (u16)(GPIO_OType_PP) << ((u16) LED1_PIN);
//    
//    LED1_PORT->MODER  &= ~(GPIO_MODER_MODER0 << ((u16) LED1_PIN * 2));
//    
//    LED1_PORT->MODER |= ((u32)GPIO_Mode_OUT) << ((u16) LED1_PIN * 2);
//    
//    /* Pull-up Pull down resistor configuration */
//    LED1_PORT->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((u16)(LED1_PIN * 2)));
//    
//    LED1_PORT->PUPDR |= GPIO_PUPDR_PUPDR0 << ((u16) LED1_PIN * 2);
    
//    
//    /* Speed mode configuration */
//    POWR_PORT->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (POWR_PIN * 2));
//    POWR_PORT->OSPEEDR |= ((u32)(GPIO_Speed_2MHz) <<(POWR_PIN * 2));
//    
//    /* Output mode configuration */
//    POWR_PORT->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((u16) POWR_PIN));
//    
//    POWR_PORT->OTYPER |= (u16)(GPIO_OType_PP) << ((u16) POWR_PIN);
//    
//    POWR_PORT->MODER  &= ~(GPIO_MODER_MODER0 << ((u16) POWR_PIN * 2));
//    
//    POWR_PORT->MODER |= ((u32)GPIO_Mode_OUT) << ((u16) POWR_PIN * 2);
//    
//    /* Pull-up Pull down resistor configuration */
//    POWR_PORT->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((u16)(POWR_PIN * 2)));
//    
//    POWR_PORT->PUPDR |= GPIO_PUPDR_PUPDR0 << ((u16) POWR_PIN * 2);
//    
    POWR_PORT->BSRR = 1<<POWR_PIN;
//		
//		
//		/* Speed mode configuration */
//    OLED_POWER_PORT->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (OLED_POWER_PIN * 2));
//    OLED_POWER_PORT->OSPEEDR |= ((u32)(GPIO_Speed_2MHz) <<(OLED_POWER_PIN * 2));
//    
//    /* Output mode configuration */
//    OLED_POWER_PORT->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((u16) OLED_POWER_PIN));
//    
//    OLED_POWER_PORT->OTYPER |= (u16)(GPIO_OType_PP) << ((u16) OLED_POWER_PIN);
//    
//    OLED_POWER_PORT->MODER  &= ~(GPIO_MODER_MODER0 << ((u16) OLED_POWER_PIN * 2));
//    
//    OLED_POWER_PORT->MODER |= ((u32)GPIO_Mode_OUT) << ((u16) OLED_POWER_PIN * 2);
//    
//    /* Pull-up Pull down resistor configuration */
//    OLED_POWER_PORT->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((u16)(OLED_POWER_PIN * 2)));
//    
//    OLED_POWER_PORT->PUPDR |= GPIO_PUPDR_PUPDR0 << ((u16) OLED_POWER_PIN * 2);
//    
    OLED_POWER_PORT->BSRR = 1<<OLED_POWER_PIN;
    
    
    
}



/*********************************************************************
 * @fn      os_err_t relays_control(os_device_t* dev, u8 cmd, void *args)
 *
 * @brief   This function is write driver for uart1
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

os_err_t  leds_control(os_device_t* dev, u8 cmd, void *args)
{
    u8 led_num=((u8*)args)[1];
    
    if(led_num < led_1&&led_num > led_2)
    {
        return ERROR;
    }
    
    if(dev==NULL)
    {
        return ERROR;
    }
    
    switch(cmd)
    {
    case led_on:  	led_ports[led_num]->BRR  = BIT_SHIFT(led_pins[led_num]) ;	 break;
    case led_off: 	led_ports[led_num]->BSRR = BIT_SHIFT(led_pins[led_num]) ;   break;
        
    case led_trigger: if((led_ports[led_num]->ODR)&BIT_SHIFT(led_pins[led_num]))
        {
            led_ports[led_num]->ODR&=~BIT_SHIFT(led_pins[led_num]);
            
        }else
        {
            led_ports[led_num]->ODR|=BIT_SHIFT(led_pins[led_num]);
        }
        break;
    default:break;
        
    }
    return SUCCESS;
}


