/***************************************************************
 * Name:      htu20.c
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
#include "htu20d.h"
#include "kernel-includes.h"
#include "drivers-includes.h"
#include "stm32f0xx_spi.h"
/*********************************************************************
* MACROS
*/


/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will 
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   

#define HTU20_FLAG_TIMEOUT         ((u32)0x1000)
#define HTU20_LONG_TIMEOUT         ((u32)(10 * HTU20_FLAG_TIMEOUT))



/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static htu20_device *htu20_dev=NULL;

/*********************************************************************
 * LOCAL function declarement
 */


os_err_t   htu20_init   (os_device_t* dev);

static  os_size_t  htu20_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);

static  os_err_t   htu20_rx_indicator(os_device_t* dev,os_size_t size);

static  os_err_t   htu20_open(os_device_t* dev, u16 oflag);

static  os_err_t   htu20_control(os_device_t* dev, u8 cmd, void *args);

static  os_err_t   htu20_send_data(u16 deviceAddr , u8 command , u8 *buffer );
static  os_err_t   htu20_read_data(u16 deviceAddr , u8 command , u8 *buffer , size_t size);

static  u8 calcrc_8(u8 *p,u8 len);
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
#define HTU20_NAME "HTU20"
#define HTU20_CLK_DEFAULT 1

os_err_t htu20_register(u16 task_id)
{
    
    htu20_dev=osmalloc(sizeof(htu20_device));
    
    htu20_dev->clk =HTU20_CLK_DEFAULT;
    
    //	htu20_dev->prescale=SPICLK_DIV_4;
    
    htu20_dev->os_device.type   = OS_Device_Class_I2CBUS;
    
    htu20_dev->os_device.device_id = OS_DEVICE_USART_ID;
    
    htu20_dev->register_taskid   =  task_id;
    
    
    htu20_dev->os_device.init    =  htu20_init;
    
    htu20_dev->os_device.open    =  htu20_open;
    
    htu20_dev->os_device.write   =  htu20_write;
    
    htu20_dev->os_device.control =  htu20_control;
    
    htu20_dev->dev_adress        =  HTU20_ADRESS;
    
    htu20_dev->os_device.rx_indicate = htu20_rx_indicator;
    
    return os_device_register(&(htu20_dev->os_device), HTU20_NAME, OS_DEVICE_FLAG_INACTIVATED);
    
}
/*********************************************************************
 * @fn      os_err_t htu20_init
 *
 * @brief   This function initiate device uaart1
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
* nition!! before U configure any register ,opening clock first
 */
#define HTU20_PORT      GPIOF
#define HTU20_PIN_SCL   6
#define HTU20_PIN_SDA   7
#define HTU20_BUS       I2C2

os_err_t  htu20_init   (os_device_t* dev)
{  	 
    
    
    
    GPIO_InitTypeDef  GPIO_InitStruct; 
    I2C_InitTypeDef  I2C_InitStruct; 
    
    
    os_clock_open("I2C2");
    os_clock_open("GPIOF");
    
    /*!< GPIO configuration */  
    /*!< Configure sEE_I2C pins: SCL */
    GPIO_InitStruct.GPIO_Pin = BIT_SHIFT(HTU20_PIN_SCL) | BIT_SHIFT(HTU20_PIN_SDA);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(HTU20_PORT , &GPIO_InitStruct);
    
    /*!< Configure sEE_I2C pins: SDA & SCL*/
    GPIO_Init(HTU20_PORT , &GPIO_InitStruct);
    
    /* Connect PXx to I2C2_SCL*/
    GPIO_PinAFConfig( HTU20_PORT , HTU20_PIN_SCL , GPIO_AF_1); 
    /* Connect PXx to I2C2_SDA*/
    GPIO_PinAFConfig( HTU20_PORT , HTU20_PIN_SDA , GPIO_AF_1);	 
    
    /*************************************************************************************/
    I2C_DeInit(HTU20_BUS);
    /* I2C configuration */
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStruct.I2C_DigitalFilter = 0x00;
    I2C_InitStruct.I2C_OwnAddress1 =0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	
    I2C_InitStruct.I2C_Timing = 0x10805E89;
    
    /* I2C Peripheral Enable */
    I2C_Cmd(HTU20_BUS , ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(HTU20_BUS , &I2C_InitStruct);
    
    
    
    return SUCCESS;
    
}


/*********************************************************************
 * @fn      os_err_t htu20_open
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
static os_err_t  htu20_open(os_device_t* dev, u16 oflag)
{
    os_clock_open("I2C");
    os_clock_open("GPIOF");	
    
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
os_err_t  htu20_close(os_device_t* dev, u16 oflag)
{
    
    os_clock_close("htu20");
    os_clock_close("GPIOB");
    
    return SUCCESS;		
    
}
/*********************************************************************
 * @fn      os_err_t htu20_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart2
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  htu20_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
{
    
    
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
            
            
        }
    }
    
    return SUCCESS;
}	
/*********************************************************************
 * @fn      me2_ch20_read
 *
 * @brief   This function read data from sensor
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data ;ignored here
*									 buffer: data need tansfering ;  attention::data is stored in 16 bits mode  
* 								 size: data size tansfered
 *   @return number of data readed
 */

os_size_t htu20_read   (os_device_t* dev, os_off_t pos, void *buffer, os_size_t size)
{
    u16 i=0;
    if(dev==NULL)
    {
        return ERROR;
    }
    
    while(size >0)
    {
        
        
    }
    
    return  i;	
    
}

/*********************************************************************
 * @fn      os_err_t htu20_control(os_device_t* dev, u8 cmd, void *args)
 *
 * @brief   This function is write driver for uart1
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_err_t  htu20_control(os_device_t* dev, u8 cmd, void *args)
{
    //    u8 adc_num=((u8*)args)[1];
    u8 buffer[10]={0};
    u32 temp;
    if(dev==NULL)
    {
        return ERROR;
    }
    switch(cmd)
    {
    /*trigger and read humidity data from sensor*/			
    case TRIGGER_HUMI_MEASUREMENT: if(SUCCESS == htu20_read_data(htu20_dev->dev_adress , TRIGGER_HUMI_MEASUREMENT , buffer , 3))
        {
            temp = (buffer)[0]<<8|(buffer[1]&(~0x03));
            ((htu20_data *)args)->humidity = ((temp*12500)>>16)-600; 
            ((htu20_data *)args)->ampfifier_times = 100;
        }break;
        /*trigger and read temperature data from sensor*/			
    case TRIGGER_TEMP_MEASUREMENT: if(SUCCESS == htu20_read_data(htu20_dev->dev_adress , TRIGGER_TEMP_MEASUREMENT , buffer , 3))
        {
            temp = (buffer)[0]<<8|(buffer[1]&(~0x03));
            ((htu20_data *)args)->temper =  ((temp*17572)>>16)-4685;
            ((htu20_data *)args)->ampfifier_times = 100;
        }break;
        /*read user register data*/			
    case READ_USER_REGISTER:       if(SUCCESS == htu20_read_data(htu20_dev->dev_adress , READ_USER_REGISTER       , buffer , 1))
        {
            ((htu20_data *)args)->user_res_content = buffer[0];
            
        }break;
        
    case WRITE_USER_REGISTER:      htu20_send_data(htu20_dev->dev_adress , WRITE_USER_REGISTER      , &(((htu20_data *)args)->user_res_content) );  break;
        /*reset sensor via software*/			
    case SOFT_RESET:  						 htu20_send_data(htu20_dev->dev_adress , SOFT_RESET , buffer );          break;    
    default:break;
        
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

static os_err_t  htu20_rx_indicator(os_device_t* dev,os_size_t size)
{
    
    return SUCCESS;
}

/*********************************************************************
 * @fn      static os_err_t htu20_read_data(u8 command)
 *
 * @brief   This function offers the api for reading operation
 *
 * 			    @param deviceaddr :slave adress
 *                 command:    specific read command
 *								 buffer:     store the data recived
 *			           size:       num of data need recieving
 * 
 *   @return none 
 */  

static os_err_t htu20_read_data(u16 deviceAddr , u8 command , u8 *buffer , size_t size)
{
    u16 count=0;
    u16  count_down;
    /*route of sending conv*/	
    I2C_NumberOfBytesConfig(HTU20_BUS, 1);
    
    I2C_SlaveAddressConfig(HTU20_BUS, (deviceAddr<<1));
    
    I2C_MasterRequestConfig(HTU20_BUS, I2C_Direction_Transmitter);
    
    I2C_GenerateSTART(HTU20_BUS, ENABLE);           //START
    
    /* tigger temper convertion */	
    I2C_SendData(HTU20_BUS , command);  //
    
    count_down = HTU20_LONG_TIMEOUT;
    
    while(!I2C_GetFlagStatus(HTU20_BUS, I2C_FLAG_TXE)) //???????,???NACK,????
    {
        if(I2C_GetFlagStatus(HTU20_BUS,I2C_FLAG_NACKF))
        {
            I2C_ClearFlag(HTU20_BUS,I2C_FLAG_NACKF);
            I2C_ClearFlag(HTU20_BUS,I2C_FLAG_STOPF);
            return ERROR;
        }
        if(count_down-- == 0)
        {
            return ERROR;
        }
        
    }
    
    
    count_down = HTU20_LONG_TIMEOUT;	
    do{
        /*wait for the return data from sensor*/	 
        I2C_ClearFlag(HTU20_BUS , I2C_FLAG_NACKF);
        
        I2C_ClearFlag(HTU20_BUS , I2C_FLAG_STOPF);
        
        I2C_NumberOfBytesConfig(HTU20_BUS , size);
        
        
        I2C_MasterRequestConfig(HTU20_BUS , I2C_Direction_Receiver);
        
        I2C_GenerateSTART(HTU20_BUS , ENABLE);           //START
        
        if(count_down-- == 0)
        {
            return ERROR;
        }			 
        
        
    }while(I2C_GetFlagStatus(HTU20_BUS ,I2C_FLAG_NACKF)==SET);
    
    
    /*get specific size of  data*/
    while(size>0)
    {	
        
        count_down = HTU20_LONG_TIMEOUT;
        
        while(!I2C_GetFlagStatus(HTU20_BUS, I2C_FLAG_RXNE))
        {
            if(count_down-- == 0)
                return  ERROR;
        }
        
        buffer[count]=I2C_ReceiveData(HTU20_BUS);
        size --;
        count ++;
    }	
    
    count_down = HTU20_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(HTU20_BUS ,  I2C_FLAG_TC) == RESET)
    {
        if(count_down-- == 0)
            return  ERROR;
    }
    
    
    I2C_GenerateSTOP(HTU20_BUS, ENABLE); 
    
    count_down = HTU20_LONG_TIMEOUT;		
    while(I2C_GetFlagStatus(HTU20_BUS, I2C_ISR_STOPF) == RESET)
    {
        if(count_down-- == 0)
            return  ERROR;
    }   
    
    /* Clear STOPF flag */
    I2C_ClearFlag(HTU20_BUS, I2C_ICR_STOPCF);
    
    //if(buffer[3] == calcrc_8(buffer , 2))		
    return  SUCCESS;
    
    
}


/*********************************************************************
 * @fn      static os_err_t htu20_send_data(u8 command)
 *
 * @brief   This function offers the api for reading operation
 *
 * 			    @param deviceaddr :slave adress
 *                 command:    specific WRITE OR CTL command
 *								 buffer:     specific data to be send only needed in write user register mode
 *			           size:       num of data need recieving
 * 
 *   @return none 
 */  

static os_err_t htu20_send_data(u16 deviceAddr , u8 command , u8 *buffer )
{
    u16  count_down;
    
    /*route of sending conv*/	
    if(command == WRITE_USER_REGISTER)
    {
        I2C_NumberOfBytesConfig(HTU20_BUS, 2);
    }
    else if(command == SOFT_RESET)
    {
        I2C_NumberOfBytesConfig(HTU20_BUS, 1);
    }else
    {
        return ERROR;
    }
    
    
    I2C_SlaveAddressConfig(HTU20_BUS, (deviceAddr<<1));
    
    I2C_MasterRequestConfig(HTU20_BUS, I2C_Direction_Transmitter);
    
    I2C_GenerateSTART(HTU20_BUS, ENABLE);           //START
    
    /* tigger temper convertion */	
    I2C_SendData(HTU20_BUS , command);  //
    
    count_down = HTU20_LONG_TIMEOUT;
    
    while(!I2C_GetFlagStatus(HTU20_BUS, I2C_FLAG_TXE)) //???????,???NACK,????
    {
        if(I2C_GetFlagStatus(HTU20_BUS,I2C_FLAG_NACKF))
        {
            I2C_ClearFlag(HTU20_BUS,I2C_FLAG_NACKF);
            I2C_ClearFlag(HTU20_BUS,I2C_FLAG_STOPF);
            return ERROR;
        }
        if(count_down-- == 0)
        {
            return ERROR;
        }
        
    }
    
    
    /*send user register content*/
    if(command == WRITE_USER_REGISTER)
    {	
        /* tigger temper convertion */	
        I2C_SendData(HTU20_BUS , buffer[0]);  //
        
        count_down = HTU20_LONG_TIMEOUT;
        
        while(!I2C_GetFlagStatus(HTU20_BUS, I2C_FLAG_TXE)) //???????,???NACK,????
        {
            if(I2C_GetFlagStatus(HTU20_BUS,I2C_FLAG_NACKF))
            {
                I2C_ClearFlag(HTU20_BUS,I2C_FLAG_NACKF);
                I2C_ClearFlag(HTU20_BUS,I2C_FLAG_STOPF);
                return ERROR;
            }
            if(count_down-- == 0)
            {
                return ERROR;
            }
            
        }
    }else if(command == SOFT_RESET)
    {
        /*no data need to be sent by this operation*/
    }
    
    count_down = HTU20_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(HTU20_BUS ,  I2C_FLAG_TC) == RESET)
    {
        if(count_down-- == 0)
            return  ERROR;
    }
    
    
    I2C_GenerateSTOP(HTU20_BUS, ENABLE); 
    
    count_down = HTU20_LONG_TIMEOUT;		
    while(I2C_GetFlagStatus(HTU20_BUS, I2C_ISR_STOPF) == RESET)
    {
        if(count_down-- == 0)
            return  ERROR;
    }   
    
    /* Clear STOPF flag */
    I2C_ClearFlag(HTU20_BUS, I2C_ICR_STOPCF);
    
    return  SUCCESS;
    
}

/*********************************************************************
 * @fn      u8 calcrc_8(u8 *p,u8 len)
 *
 * @brief   This function offers function of crc8
 *
 * 			    @param deviceaddr :slave adress
 *                 command:    specific WRITE OR CTL command
 *								 buffer:     specific data to be send only needed in write user register mode
 *			           size:       num of data need recieving
 * 
 *   @return none 
 */  

u8 calcrc_1byte(u8 abyte)  
{  
    u8 i,crc_1byte;   
    crc_1byte=0;                //??crc_1byte???0
    for(i = 0; i < 8; i++)  
    {  
        if(((crc_1byte^abyte)&0x01))  
        {  
            crc_1byte^=0x18;   
            crc_1byte>>=1;  
            crc_1byte|=0x80;  
        }        
        else   
            crc_1byte>>=1; 
        abyte>>=1;        
    } 
    return crc_1byte; 
} 

u8 calcrc_8(u8 *p,u8 len)
{
    u8 crc=0;
    while(len--) //len??????????
    {
        crc=calcrc_1byte(crc^*p++);
    }
    return crc;  //??????crc?0,???????
}
