/***************************************************************
 * Name:      spi2.c
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


static  os_err_t   spi2_init   (os_device_t* dev);

static  os_size_t  spi2_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);

static os_size_t  spi2_read(os_device_t* dev, os_off_t pos, void *buffer, os_size_t size);

static  os_err_t   spi2_rx_indicator(os_device_t* dev,os_size_t size);

static  os_err_t   spi2_open(os_device_t* dev, u16 oflag);

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
#define SPI2_NAME "SPI2"

os_err_t spi2_register(u16 task_id)
{
    
    spi_dev=osmalloc(sizeof(spi_device));
    
    spi_dev->pclk2=SYSCLK;
    
    spi_dev->prescale=SPICLK_DIV_4;
    
    spi_dev->os_device.type = OS_Device_Class_SPIBUS;
    
    spi_dev->os_device.device_id = OS_DEVICE_USART_ID;
    
    spi_dev->register_taskid = task_id;
    
    spi_dev->mode =  spi_tx_only;
    
    spi_dev->os_device.init  = spi2_init;
    
    spi_dev->os_device.open  = spi2_open;
    
    spi_dev->os_device.write = spi2_write;
    
    spi_dev->os_device.read = spi2_read;
    
    spi_dev->os_device.rx_indicate = spi2_rx_indicator;
    
    return os_device_register(&(spi_dev->os_device), SPI2_NAME, OS_DEVICE_FLAG_INACTIVATED);
    
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
#define SPI2_PORT GPIOB
#define SPI2_PIN_MOSI  15
#define SPI2_PIN_MISO  14
#define SPI2_PIN_SCK   13



static os_err_t  spi2_init   (os_device_t* dev)
{  	 
    
    u32 tmpreg;
    SPI_InitTypeDef   SPI_InitStruct;
    os_clock_open("SPI2");
    
    
    
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//配置spi方向
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;//配置spi模式
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;//配置数据格式
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;//配置时钟高电平稳态
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;//配置时钟bit位捕获方式
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;//设置nss管脚软件管理
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//设置spi波特率分频值
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;//指定数据传输从msb位开始
    SPI_InitStruct.SPI_CRCPolynomial = 7;//指定用于CRC计算的值
    SPI_Init(SPI2, &SPI_InitStruct);//调入结构体
    SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);//设置接收缓冲
    SPI_Cmd(SPI2, ENABLE); /*!< SD_SPI enable */
    //SPI_SendData8
    //    /*---------------------------- SPIx CR1 Configuration ------------------------*/
    //    /* Get the SPIx CR1 value */
    //    tmpreg = SPI2->CR1;
    //    /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, CPOL and CPHA bits */
    //    tmpreg &= ((uint16_t)0x3040);
    //    /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
    //  master/slave mode, CPOL and CPHA */
    //    /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
    //    /* Set SSM, SSI bit according to SPI_NSS values */
    //    /* Set LSBFirst bit according to SPI_FirstBit value */
    //    /* Set BR bits according to SPI_BaudRatePrescaler value */
    //    /* Set CPOL bit according to SPI_CPOL value */
    //    /* Set CPHA bit according to SPI_CPHA value */
    //    tmpreg |= (uint16_t)((uint32_t) 0<<15 | 0<<14|1<<9|0<<7|(spi_dev->prescale<<3)|1<<1|1<<0);  
    //    /* Write to SPIx CR1 */
    //    SPI2->CR1 = (tmpreg); 
    //    /* 8 	BITS DATA FORMAT */
    //    SPI2->CRCPR = 7;
    //    
    //    SPI2->CR2  = (7<<8);
    //    
    //    /*  	master mode */
    //    SPI2->CR1 |= ((uint16_t)0x0104);
    //    
    //    /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
    //    SPI2->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
    //    /* ENABLE SPI2 */	
    //    SPI2->CR1 |=SPI_CR1_SPE; //ENABLE SPI2
    //		
    
    os_clock_open("SPI2");
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
static os_err_t  spi2_open(os_device_t* dev, u16 oflag)
{
    os_clock_open("SPI2");
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
os_err_t  spi2_close(os_device_t* dev, u16 oflag)
{
    
    os_clock_close("SPI2");
    os_clock_close("GPIOB");
    
    return SUCCESS;		
    
}
/*********************************************************************
 * @fn      os_err_t spi2_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart2
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  spi2_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
{
    u32 spixbase = (u32)SPI2; 
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
            
            while((SPI2->SR&SPI_SR_TXE) == RESET);//Transmit data register empty
            *(__IO u8 *) spixbase = *(char*)temp;
            temp++;
            size--;
        }
    }
    
    return SUCCESS;
}	

/*********************************************************************
 * @fn      os_err_t spi2_read(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart2
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  spi2_read(os_device_t* dev, os_off_t pos, void *buffer, os_size_t size)
{
    u32 spixbase = (u32)SPI2; 
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
            
            
            while((SPI2->SR&SPI_SR_RXNE)== RESET);//recived register no empty
            *(char*)temp = *(__IO u8 *) spixbase ;
            temp++;
            size--;
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

static os_err_t  spi2_rx_indicator(os_device_t* dev,os_size_t size)
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
void SPI2_IRQHandler(void)
{
    
    
    
    
} 



