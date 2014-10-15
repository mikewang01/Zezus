/***************************************************************
 * Name:      SPI1.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "stdio.h"
#include "spi.h"
#include "kernel-includes.h"
#include "drivers-includes.h"
#include "stm32f0xx_spi.h"
/*********************************************************************
* MACROS
*/



/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static spi_device *spi_dev=NULL;

/*********************************************************************
 * LOCAL function declarement
 */


static  os_err_t   spi1_init   (os_device_t* dev);

static  os_size_t  spi1_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);

static  os_err_t   spi1_rx_indicator(os_device_t* dev,os_size_t size);

static  os_err_t   spi1_open(os_device_t* dev, u16 oflag);

/*********************************************************************
 * @fn      os_err_t os_device_init_all
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
#define SPI1_NAME  "SPI1"

os_err_t spi1_register(u16 task_id)
{
    
    spi_dev=osmalloc(sizeof(spi_device));
    
    spi_dev->pclk2=SYSCLK;
    
    spi_dev->prescale=SPICLK_DIV_2;
    
    spi_dev->os_device.type = OS_Device_Class_SPIBUS;
    
    spi_dev->os_device.device_id = OS_DEVICE_USART_ID;
    
    spi_dev->register_taskid = task_id;
    
    spi_dev->mode =  spi_tx_only;
    
    spi_dev->os_device.init  = spi1_init;
    
    spi_dev->os_device.open  = spi1_open;
    
    spi_dev->os_device.write = spi1_write;
    
   // spi_dev->os_device.rx_indicate = spi1_rx_indicator;
    
    return os_device_register(&(spi_dev->os_device), SPI1_NAME, OS_DEVICE_FLAG_INACTIVATED);
    
}
/*********************************************************************
 * @fn      os_err_t uart1_init
 *
 * @brief   This function initiate device uaart1
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
* nition!! before U configure any register ,opening clock first
 */
#define SPI1_PORT      GPIOB
#define SPI1_PIN_MOSI  5
#define SPI1_PIN_MISO  4
#define SPI1_PIN_SCK   3



static os_err_t  spi1_init   (os_device_t* dev)
{  	 
    
    u32 tmpreg;
    SPI_InitTypeDef   SPI_InitStruct;
    os_clock_open("SPI1");
//    os_clock_open("GPIOB");
    
//    gpio_pin_altset(SPI1_PORT , SPI1_PIN_MOSI , GPIO_AF_0);
//    
//    gpio_pin_altset(SPI1_PORT , SPI1_PIN_SCK  , GPIO_AF_0);
//    
//    /* Speed mode configuration */
//    gpio_speed_set(SPI1_PORT , SPI1_PIN_MOSI , GPIO_Speed_50MHz );
//    
//    gpio_speed_set(SPI1_PORT , SPI1_PIN_SCK  , GPIO_Speed_50MHz );
//    
//    
//    /* Output mode configuration */
//    gpio_outtype_set(SPI1_PORT , SPI1_PIN_MOSI , GPIO_OType_PP);
//    
//    gpio_outtype_set(SPI1_PORT , SPI1_PIN_SCK , GPIO_OType_PP);
//    
//    
//    gpio_outmode_set(SPI1_PORT , SPI1_PIN_MOSI , GPIO_Mode_AF);
//    gpio_outmode_set(SPI1_PORT , SPI1_PIN_MISO , GPIO_Mode_AF);
//    gpio_outmode_set(SPI1_PORT , SPI1_PIN_SCK ,  GPIO_Mode_AF);
//    
//    /* Pull-up Pull down resistor configuration */
//    gpio_pupdr_set(SPI1_PORT , SPI1_PIN_MOSI , GPIO_PuPd_UP);
//    
//    gpio_pupdr_set(SPI1_PORT , SPI1_PIN_SCK , GPIO_PuPd_UP);
//    
//    
//    
//    if(spi_dev->mode != spi_tx_only)
//    {
//        gpio_pin_altset(SPI1_PORT ,  SPI1_PIN_MISO , GPIO_AF_0);
//        gpio_speed_set(SPI1_PORT ,   SPI1_PIN_MISO , GPIO_Speed_50MHz ); 
//        gpio_outtype_set(SPI1_PORT , SPI1_PIN_MISO , GPIO_OType_PP);
//        gpio_pupdr_set(SPI1_PORT ,   SPI1_PIN_MISO , GPIO_PuPd_UP);
//        
//    }	
//    
//    
    /*---------------------------- SPIx CR1 Configuration ------------------------*/
    /* Get the SPIx CR1 value */
    tmpreg = SPI1->CR1;
    /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, CPOL and CPHA bits */
    tmpreg &= ((uint16_t)0x3040);
    /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
  master/slave mode, CPOL and CPHA */
    /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
    /* Set SSM, SSI bit according to SPI_NSS values */
    /* Set LSBFirst bit according to SPI_FirstBit value */
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    /* Set CPOL bit according to SPI_CPOL value */
    /* Set CPHA bit according to SPI_CPHA value */
    tmpreg |= (uint16_t)((uint32_t) 1<<15 | 1<<14|1<<9|0<<7|(spi_dev->prescale<<3)|1<<1|1<<0);  
    /* Write to SPIx CR1 */
    SPI1->CR1 = (tmpreg); 
    /* 8 	BITS DATA FORMAT */
    SPI1->CRCPR = 7;
    
    SPI1->CR2  = (7<<8);
    
    /*  	master mode */
    SPI1->CR1 |= ((uint16_t)0x0104);
    
    /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
    SPI1->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
    /* ENABLE SPI1 */	
    SPI1->CR1 |=SPI_CR1_SPE; //ENABLE SPI1
    
    //	NVIC_SetPriority(USART1_IRQn , 0x00);
    //	NVIC_EnableIRQ(USART1_IRQn);
    
    
    
    
    //	 os_clock_close("SPI1");	
    os_clock_open("SPI1");
    os_clock_open("GPIOB");
    
    return SUCCESS;
    
    
}


/*********************************************************************
 * @fn      os_err_t uart2_open
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
static os_err_t  spi1_open(os_device_t* dev, u16 oflag)
{
    os_clock_open("SPI1");
    os_clock_open("GPIOB");
    
    return SUCCESS;	
}

/*********************************************************************
 * @fn      os_err_t uart2_close
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
os_err_t  spi1_close(os_device_t* dev, u16 oflag)
{
    
    os_clock_close("SPI1");
    os_clock_close("GPIOB");
    
    return SUCCESS;		
    
}
/*********************************************************************
 * @fn      os_err_t SPI1_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart2
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  spi1_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
{
    u32 spixbase = (u32)SPI1; 
    spixbase += 0x0C;
    
    if(pos>=size||buffer==NULL||size>USART_TX_BUFFER_SIZE)
    {
        return  NULL;
    }else
    {
        char* temp=(char*)buffer; 
        if(pos>0)
        {
            temp=temp+pos;
        }				 
        
        while(size>0)
        {
            
            *(__IO u8 *) spixbase = *(char*)temp;
            temp++;
            size--;
            while(!(SPI1->SR&(1<<1)));//Transmit data register empty
        }
    }
    
    return SUCCESS;
}	

/*********************************************************************
 * @fn      os_err_t uart2_rx_indicator
 *
 * @brief   This function is write driver for uart2
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_err_t  SPI1_rx_indicator(os_device_t* dev,os_size_t size)
{
    
    return SUCCESS;
}

/*********************************************************************
 * @fn      void USART2_IRQHandler
 *
 * @brief   This function is irq for usart1
 *
 * 			    @param none 
 *			
 * 
 *   @return none 
 */  
void SPI1_IRQHandler(void)
{
    
    
    
    
} 



