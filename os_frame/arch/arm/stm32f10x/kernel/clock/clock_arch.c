//----------------------------------------------------------------------------
// WSH Corp. Company Confidential Strictly Private
//
// $RCSfile: task.c,v $
// $Revision: 0.1 $
// $Author: Mike Wang $
// $Date: 2014/5/10 19:37:23 $
// $Description: Task file
//
// ---------------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<
// ---------------------------------------------------------------------------
// Copyright 2013-2014 (c) WSH Corp.
//
// WSH Corp. owns the sole copyright to this software. Under international
// copyright laws you (1) may not make a copy of this software except for
// the purposes of maintaining a single archive copy, (2) may not derive
// works herefrom, (3) may not distribute this work to others. These rights
// are provided for information clarification, other restrictions of rights
// may apply as well.
//
// This is an unpublished work.
// ---------------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>> WARRANTEE <<<<<<<<<<<<<<<<<<<<<
// ---------------------------------------------------------------------------
// WSH Corp. MAKES NO WARRANTY OF ANY KIND WITH REGARD TO THE USE OF
// THIS SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR
// PURPOSE.
// ---------------------------------------------------------------------------


/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "kernel-includes.h"
#include "drivers-includes.h"

// ***************************************************************************
// ******************** START OF global variable DECLARATIONS *******************
// **************************************************************************



 
 
 static os_err_t gpioa_clock_enable(void);
 static os_err_t gpioa_clock_disable(void);


 static os_err_t gpiob_clock_enable(void);
 static os_err_t gpiob_clock_disable(void);

 static os_err_t gpioc_clock_enable(void);
 static os_err_t gpioc_clock_disable(void);

 static os_err_t gpiod_clock_enable(void);
 static os_err_t gpiod_clock_disable(void);

 static os_err_t gpioe_clock_enable(void);
 static os_err_t gpioe_clock_disable(void);
 
static os_err_t usart1_clock_enable(void);
static os_err_t usart1_clock_disable(void);


static os_err_t usart2_clock_enable(void);
static os_err_t usart2_clock_disable(void);
		
static os_err_t afio_clock_enable(void);
static os_err_t afio_clock_disable(void);

static os_err_t adc1_clock_enable(void);
static os_err_t adc1_clock_disable(void);

static os_err_t adc2_clock_enable(void);
static os_err_t adc2_clock_disable(void);

static os_err_t tim1_clock_enable(void);
static os_err_t tim1_clock_disable(void);

static os_err_t spi1_clock_enable(void);
static os_err_t spi1_clock_disable(void);

static os_err_t dma1_clock_enable(void);
static os_err_t dma1_clock_disable(void);

#ifdef CLOCK_IN_ROM

clock_status_record clock_status={0};

#endif
const Peripheral_clock clock_list[]=
{
   {
			{
				"GPIOA",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  gpioa_clock_enable,
		  gpioa_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.gpioa),
#endif
	},
   {
			{
				"GPIOB",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  gpiob_clock_enable,
		  gpiob_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.gpiob),
#endif
	},
   {
			{
				"GPIOC",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  gpioc_clock_enable,
		  gpioc_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.gpioc),
#endif
	},
	   {
			{
				"GPIOD",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
				
			},
		  clock_closed,
			clock_closed,
		  gpiod_clock_enable,
		  gpiod_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.gpiod),
#endif
	},
	
		{
			{
				"GPIOE",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  gpioe_clock_enable,
		  gpioe_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.gpioe),
#endif
	},	
		{
			{
				"USART1",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  usart1_clock_enable,
		  usart1_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.usart1),
#endif
	},
		{
			{
				"USART2",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  usart2_clock_enable,
		  usart2_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.usart2),
#endif
	},  
		{
			{
				"AFIO",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  afio_clock_enable,
		  afio_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.afio),
#endif
	 },
		{
			{
				"ADC1",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  adc1_clock_enable,
		  adc1_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.adc1),
#endif
	},
		{
			{
				"ADC2",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  adc2_clock_enable,
		  adc2_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.adc2),
#endif
	},
		{
			{
				"SPI1",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  spi1_clock_enable,
		  spi1_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.spi1),
#endif
	},
		{
			{
				"TIME1",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  tim1_clock_enable,
		  tim1_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.tim1),
#endif
	},
				{
			{
				"DMA1",
				 OS_Object_Class_Clock,
				 OS_DEVICE_FLAG_INACTIVATED,
				 NULL,
			},
		  clock_closed,
			clock_closed,
		  dma1_clock_enable,
		  dma1_clock_disable,
#ifdef CLOCK_IN_ROM
			&(clock_status.dma1),
#endif
	},
		
	
};
static u16 clock_num=(sizeof( clock_list )/sizeof( clock_list[0] ));


/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      clock_add_to_manager_list
 *
 * @brief   copy specific amount of data from source adress to destination adress.
 *
* @param    
 *
 * @return  none
 */

os_err_t clock_add_to_manager_list()
{
	 u16 i=0;
	 Peripheral_clock *clock_add=NULL;
#ifdef CLOCK_IN_ROM
	 clock_add =(Peripheral_clock *)clock_list;
	 os_clock_register(clock_add , clock_list->parent.name);
#else	
	for(i=0 ; i<clock_num ; i++)
	{
		 clock_add=(Peripheral_clock*)osmalloc(sizeof(Peripheral_clock));
		
			if(clock_add==NULL)	
			{
				goto fault;
			}
		 osmemcpy(clock_add,(void*)(clock_list+i),sizeof(Peripheral_clock));
		 os_clock_register(clock_add , clock_list->parent.name);
	}
#endif
	
 return SUCCESS;
		fault:
	return ERROR;
}



/*********************************************************************
 * @fn     static os_err_t  gpioa_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t gpioa_clock_enable()
{
		RCC->APB2ENR |=1<<2;     //enbale PORTA  PAORB CLOCK  
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpioa_clock_enable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t gpioa_clock_disable()
{
		RCC->APB2ENR &=~(1<<2);     //enbale PORTA  PAORB CLOCK  
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiob_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t gpiob_clock_enable()
{
		RCC->APB2ENR |=1<<3;    
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiob_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t gpiob_clock_disable()
{
		RCC->APB2ENR &=~(1<<3);     
	  return SUCCESS;
}

/*********************************************************************
 * @fn     static os_err_t  gpioc_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t gpioc_clock_enable()
{
		RCC->APB2ENR |=1<<4;     //enbale PORTA  PAORB CLOCK  
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpioc_clock_enable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t gpioc_clock_disable()
{
		RCC->APB2ENR &=~(1<<4);     //enbale PORTA  PAORB CLOCK  
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiod_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t gpiod_clock_enable()
{
		RCC->APB2ENR |=1<<5;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiod_clock_enable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t gpiod_clock_disable()
{
		RCC->APB2ENR &=~(1<<5);     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiod_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t gpioe_clock_enable()
{
		RCC->APB2ENR |=1<<6;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiod_clock_enable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t gpioe_clock_disable()
{
		RCC->APB2ENR &=~(1<<6);     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiod_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t gpiof_clock_enable()
{
		RCC->APB2ENR |=1<<7;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiod_clock_enable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t gpiof_clock_disable()
{
		RCC->APB2ENR &=~(1<<7);     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  afio_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t afio_clock_enable()
{
		RCC->APB2ENR |=1<<0;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  afio_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t afio_clock_disable()
{
		RCC->APB2ENR &=~(1<<0);     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  usart1_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t adc1_clock_enable()
{
		RCC->APB2ENR |=1<<9;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiod_clock_enable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t adc1_clock_disable()
{
		RCC->APB2ENR &=~(1<<9);     
	  return SUCCESS;
}

/*********************************************************************
 * @fn     static os_err_t  adc2_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t adc2_clock_enable()
{
		RCC->APB2ENR |=1<<10;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  adc2_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t adc2_clock_disable()
{
		RCC->APB2ENR &=~(1<<10);     
	  return SUCCESS;
}



/*********************************************************************
 * @fn     static os_err_t  tim1_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t tim1_clock_enable()
{
		RCC->APB2ENR |=1<<10;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  tim1_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t tim1_clock_disable()
{
		RCC->APB2ENR &=~(1<<10);     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  spi1_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */
static os_err_t spi1_clock_enable()
{
		RCC->APB2ENR |=1<<10;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  spi1_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t spi1_clock_disable()
{
		RCC->APB2ENR &=~(1<<10);     
	  return SUCCESS;
}






/*********************************************************************
 * @fn     static os_err_t  usart1_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t usart1_clock_enable()
{
		RCC->APB2ENR |=1<<14;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  gpiod_clock_enable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t usart1_clock_disable()
{
		RCC->APB2ENR &=~(1<<14);     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  usart2_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t usart2_clock_enable()
{
		RCC->APB1ENR |=1<<17;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  usart2_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t usart2_clock_disable()
{
		RCC->APB1ENR &=~(1<<17);     
	  return SUCCESS;
}




//APB1 peripheral clock enable register(RCC_APB1ENR)
/*********************************************************************
 * @fn     static os_err_t  tim2_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t tim2_clock_enable()
{
		RCC->APB1ENR |=1<<0;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  tim2_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t tim2_clock_disable()
{
		RCC->APB1ENR &=~(1<<0);     
	  return SUCCESS;
}

/*********************************************************************
 * @fn     static os_err_t  tim3_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t tim3_clock_enable()
{
		RCC->APB1ENR |=1<<1;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  tim3_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t tim3_clock_disable()
{
		RCC->APB1ENR &=~(1<<1);     
	  return SUCCESS;
}




/*********************************************************************
 * @fn     static os_err_t  tim4_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t tim4_clock_enable()
{
		RCC->APB1ENR |=1<<2;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  tim4_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t tim4_clock_disable()
{
		RCC->APB1ENR &=~(1<<2);     
	  return SUCCESS;
}

/*********************************************************************
 * @fn     static os_err_t  tim4_clock_enable();
 *
 * @brief   enable gpioa clock
 *
 * @param  none
 *
 * @return  none
 */

static os_err_t dma1_clock_enable()
{
		RCC->AHBENR|=1<<0;     
	  return SUCCESS;
}


/*********************************************************************
 * @fn     static os_err_t  tim4_clock_disable();
 *
 * @brief   disable gpioa clock
 *
 * @param   none
 *
 * @return  none
 */

static os_err_t dma1_clock_disable()
{
		RCC->AHBENR &=~(1<<0);     
	  return SUCCESS;
}





