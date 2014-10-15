
/***************************************************************
 * Name:      USART.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "kernel-includes.h"
#include "drivers-includes.h"
/*********************************************************************
* MACROS
*/

#define USART2RX_DMA1_Channelx DMA1_Channel5
#define USART2TX_DMA1_Channelx DMA1_Channel4

/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */

static u8 USART2_RX_BUF[USART_TX_BUFFER_SIZE];
static u8 USART2_TX_BUF[USART_TX_BUFFER_SIZE];
static usart_device *usart_dev=NULL;

/*********************************************************************
 * LOCAL function declarement
 */

static	void       usart2_dma1_channel5_init(void);	
static  os_err_t   uart2_init   (os_device_t* dev);
static  void       usart2_dma1_channel4_init(void);
static  os_size_t  uart2_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);
static  os_err_t   uart2_open(os_device_t* dev, u16 oflag);


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
#define USART2_NAME "USART2"
os_err_t usart_register(u16 task_id)
{
    
    usart_dev=osmalloc(sizeof(usart_device));
    
    
    usart_dev->pclk2=SYSCLK;
    usart_dev->bound=BOUND;
    
    usart_dev->os_device.type = OS_Device_Class_Char;
    
    usart_dev->os_device.device_id = OS_DEVICE_USART_ID;
    
    usart_dev->register_taskid = task_id;
    
    usart_dev->os_device.init  = uart2_init;
    
    usart_dev->os_device.open  = uart2_open;
    usart_dev->os_device.write = uart2_write;
    
    
    return os_device_register(&(usart_dev->os_device), USART2_NAME, OS_DEVICE_FLAG_INACTIVATED);
}


/*********************************************************************
 * @fn      os_err_t os_device_init_all
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
* nition!! before U configure any register ,opening clock first
 */
#define USART2_REMAP
os_err_t  uart2_init   (os_device_t* dev)
{  	 
    float temp;
    u16 mantissa;
    u16 fraction;	
    os_device_t *__mptr = (dev);
    usart_device * usart_dev=(usart_device *)( (char *)__mptr - offsetof(usart_device,os_device));	
    
    temp=(float)(usart_dev->pclk2*1000000)/(usart_dev->bound*16);//得到USARTDIV
    mantissa=temp;				 //得到整数部分
    fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
    mantissa+=fraction;
    
    os_clock_open("USART1");
    
#ifdef USART2_REMAP
    //	os_clock_open("GPIOB");
    //	GPIOB->CRL&=0X00FFFffF; 
    //	GPIOB->CRL|=0X8b000000;//IO状态设置
    //	RCC->APB2RSTR|=1<<14;   //复位串口1
    //  AFIO->MAPR|=1<<2;
    //	os_clock_close("GPIOB");
#else
    os_clock_open("GPIOA");
    GPIOA->CRH&=0XFFFFF00F; 
    GPIOA->CRH|=0X000008B0;//IO状态设置
    RCC->APB2RSTR|=1<<14;   //复位串口1
    // os_clock_close("GPIOA");
#endif	
    RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
    
    
    USART1->CR3|=1<<7;//DMA enable transmitter
    USART1->CR3|=1<<6;//DMA enable receiver
    
    //波特率设置
    USART1->BRR=mantissa; // 波特率设置	 
    USART1->CR1|=0X200C;  //1位停止,无校验位.
    
    os_clock_open("DMA1");		
    usart2_dma1_channel4_init();
    usart2_dma1_channel5_init();
    os_clock_close("DMA1");	
    
    USART1->CR1|=1<<4;// IDLE interrupt enable
    MY_NVIC_Init(0,1,USART1_IRQn,2);//组2，最低优先级
    
    os_clock_close("USART1");
    
    return SUCCESS;
    
    
}

/*********************************************************************
 * @fn      os_err_t uart1_open
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
static os_err_t  uart2_open(os_device_t* dev, u16 oflag)
{
    os_clock_open("AFIO");	
    os_clock_open("DMA1");	
    os_clock_open("USART1");
    os_clock_open("GPIOA");
    os_clock_open("GPIOB");
    return SUCCESS;	
}

/*********************************************************************
 * @fn      os_err_t uart1_close
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
os_err_t  uart2_close(os_device_t* dev, u16 oflag)
{
    os_clock_close("AFIO");	
    os_clock_close("DMA1");	
    os_clock_close("USART1");
    os_clock_close("GPIOA");
    os_clock_close("GPIOB");
    return SUCCESS;		
    
}
/*********************************************************************
 * @fn      os_err_t uart1_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart1
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  uart2_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
{
    
    if(pos>=size||buffer==NULL||size>USART_TX_BUFFER_SIZE)
    {
        return  NULL;
    }else
    {
        
        osmemcpy(USART2_TX_BUF,(char *)buffer, size);
        USART2TX_DMA1_Channelx->CNDTR=size;//DMA channel x number of data register
        USART2TX_DMA1_Channelx->CMAR=(unsigned long)(USART2_TX_BUF);
        USART2TX_DMA1_Channelx->CCR|=1<<0;          //开启DMA传输
    }
    
    return SUCCESS;
}	
/*********************************************************************
 * @fn      os_err_t usart1_dma1_channel4_init
 *
 * @brief   This function initiate usart tx dma chnneel4
 *
 * 			    @param none 
 *			
 * 
 *   @return none
 */
static void usart2_dma1_channel4_init()
{
    
    
    //	USART1TX_DMA1_Channelx->CCR=0;//reset
    //	USART1TX_DMA1_Channelx->CCR|=0<<10;//Memory size 0:8bits,16-bits	 2 32bits
    //	USART1TX_DMA1_Channelx->CCR|=0<<8;//Peripheral size 0:8bits,1:16-bits	 2 32bits
    //	USART1TX_DMA1_Channelx->CCR|=1<<7;//Memory increment mode enabled
    //	USART1TX_DMA1_Channelx->CCR|=0<<12;//
    //	//USART1RX_DMA1_Channelx->CCR|=1<<5;//Memory Circular  mode enabled
    //	
    //	USART1TX_DMA1_Channelx->CCR&=~(1<<6);	//Peripheral increment mode disabled
    //	USART1TX_DMA1_Channelx->CCR|=(1<<4);//write to peripheral
    //	USART1TX_DMA1_Channelx->CCR|=1<<1;//Transfer complete interrupt enable
    //	//USART1TX_DMA1_Channelx->CNDTR=USART_TX_BUFFER_SIZE;//DMA channel x number of data register
    //  //USART1TX_DMA1_Channelx->CMAR=(unsigned long)(USART1_RX_BUF);
    ////	 USART1TX_DMA1_Channelx->CPAR=(unsigned long)(&(USART1->DR));
    //	// MY_NVIC_Init(1,1,DMA1_Channel4_IRQn ,2);//抢占1，子优先级3，组2
}

/*********************************************************************
 * @fn      os_err_t usart1_dma1_channel5_init
 *
 * @brief   This function initiate usart rx dma chnneel5
 *
 * 			    @param none 
 *			
 * 
 *   @return none 
 */
static void usart2_dma1_channel5_init()
{
    
    
    USART2RX_DMA1_Channelx->CCR=0;//reset
    USART2RX_DMA1_Channelx->CCR|=0<<10;//Memory size 0:8bits,16-bits	 2 32bits
    USART2RX_DMA1_Channelx->CCR|=0<<8;//Peripheral size 0:8bits,1:16-bits	 2 32bits
    USART2RX_DMA1_Channelx->CCR|=1<<7;//Memory increment mode enabled
    USART2RX_DMA1_Channelx->CCR|=2<<12;//
    USART2RX_DMA1_Channelx->CCR|=1<<5;//Memory Circular  mode enabled
    
    USART2RX_DMA1_Channelx->CCR&=~(1<<6);	//Peripheral increment mode disabled
    USART2RX_DMA1_Channelx->CCR&=~(1<<4);//Read from peripheral
    //USART1RX_DMA1_Channelx->CCR|=1<<1;//Transfer complete interrupt enable
    USART2RX_DMA1_Channelx->CNDTR=USART_TX_BUFFER_SIZE;//DMA channel x number of data register
    USART2RX_DMA1_Channelx->CMAR=(unsigned long)(USART2_RX_BUF);
    //	USART1RX_DMA1_Channelx->CPAR=(unsigned long)(&(USART1->DR));
    
    USART2RX_DMA1_Channelx->CCR|=1<<0;          //开启DMA传输
    
}

/*********************************************************************
 * @fn      void DMA1_Channel4_IRQHandler
 *
 * @brief   This function is irq for dma1_channel4
 *
 * 			    @param none 
 *			
 * 
 *   @return none 
 */
void DMA1_Channel4_IRQHandler()
{
    
    USART2TX_DMA1_Channelx->CCR&=~(1<<0);         
    if(DMA1->ISR&(1<<13))
    {
        
        
    }
    DMA1->IFCR|=3<<12;
}


/*********************************************************************
 * @fn      void USART1_IRQHandler
 *
 * @brief   This function is irq for usart1
 *
 * 			    @param none 
 *			
 * 
 *   @return none 
 */    
void USART2_IRQHandler(void)
{
    
    volatile  u32 uart1_status;
    u16 data_length;
    usart_data usart_message_data;
    
    //if(USART1->SR&(1<<4))//idle interrupt
    {	 
        
        
        USART2RX_DMA1_Channelx->CCR&=(~0x01); //disable dma templaroly
        
        data_length=USART_TX_BUFFER_SIZE-USART2RX_DMA1_Channelx->CNDTR; //get the length of data
        
        
        
        usart_message_data.length=data_length;
        osmemcpy(usart_message_data.data, USART2_RX_BUF , data_length);
        
        if(usart_dev==NULL)
        {
            goto end;  
        }
        
        usart_send_message(usart_dev->register_taskid , &usart_message_data ,usart_message_data.length+sizeof(usart_message_data.length));
        
        
        
        // uart1_status=USART1->SR;  
        // uart1_status=USART1->DR;
        // uart1_status=USART1->SR;
        
        //	send_message(usart_dev->register_taskid,DEVICE_EVENT,USART_MSG , NULL,(void*)(usart_message_data), sizeof(usart_message_data));
end:
        USART2RX_DMA1_Channelx->CNDTR=USART_RX_BUFFER_SIZE;
        //	  USART1RX_DMA1_Channelx->CPAR=(unsigned long)(&USART1->DR);
        USART2RX_DMA1_Channelx->CMAR=(unsigned long)(USART2_RX_BUF);	
        USART2RX_DMA1_Channelx->CCR|=1<<0; 	
    }  	
    
    
    //	USART1->SR^=uart1_status;								 
} 



