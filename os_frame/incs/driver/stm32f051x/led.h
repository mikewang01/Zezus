#ifndef __LED_H
#define __LED_H	 
#include "corefunc.h"
#include "kernel-includes.h"
#include "drivers-includes.h"
#include "stm32f0xx_gpio.h"

//Mini STM32开发板
//LED驱动代码			 
//正点原子@ALIENTEK
//2010/5/27

#define POWR_PORT GPIOB
#define POWR_PIN  1
#define POWR_ACTIVE 1


#define OLED_POWER_PORT GPIOA
#define OLED_POWER_PIN  12
#define OLED_POWER_ACTIVE 1

#define LED1_PORT GPIOA
#define LED1_PIN  11
#define LED1_ACTIVE 0

#define LED2_PORT GPIOA
#define LED2_PIN  8
#define LED2_ACTIVE 0


#define LED3_PORT GPIOB
#define LED3_PIN  7
#define LED3_ACTIVE 0

//LED端口定义


 enum led_command{led_on=0 , led_off, led_trigger};
 enum led_list{led_1=0 , led_2 ,led_3};

#define LED_MIN_NUM    led_1
#define LED_MAX_NUM  	led_3
 
 #define BIT_SHIFT(x)  (1<<x)
 
void led_init(void);//初始化	
 
os_err_t  leds_control(os_device_t* dev, u8 cmd, void *args);
 
#endif

















