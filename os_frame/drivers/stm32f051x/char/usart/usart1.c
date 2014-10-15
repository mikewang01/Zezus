
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
#include "stdio.h"
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
 * LOCAL function declarement
 */


static  os_err_t   uart1_init   (os_device_t* dev);

static  os_size_t  uart1_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);

static  os_err_t   uart1_rx_indicator(os_device_t* dev,os_size_t size);

static  os_err_t   uart1_open(os_device_t* dev, u16 oflag);

/*********************************************************************
 * LOCAL VARIABLES
 */


static usart_device *console_dev=NULL;


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
#define USART1_NAME "USART1"
os_err_t usart1_register(u16 task_id)
{
    
    console_dev=osmalloc(sizeof(usart_device));
    
    
    console_dev->pclk2=SYSCLK;
    console_dev->bound=BOUND;
    
    console_dev->os_device.type = OS_Device_Class_Char;
    
    console_dev->os_device.device_id = OS_DEVICE_USART_ID;
    
    console_dev->register_taskid = task_id;
    
    console_dev->os_device.init  = uart1_init;
    
    console_dev->os_device.open  = uart1_open;
    console_dev->os_device.write = uart1_write;
    console_dev->os_device.rx_indicate = uart1_rx_indicator;
    
    return os_device_register(&(console_dev->os_device), USART1_NAME, OS_DEVICE_FLAG_INACTIVATED);
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
#define UART1_PORT GPIOA
#define UART1_PORT_TX  9
#define UART1_PORT_RX  10

os_err_t  uart1_init   (os_device_t* dev)
{  	 
    
    u32 temp_1,temp_2, tmpreg;
    u32 integerdivider,fractionaldivider;	
    
    os_device_t *__mptr = (dev);
    usart_device * console_dev=(usart_device *)( (char *)__mptr - offsetof(usart_device,os_device));	
    
    
    os_clock_open("USART1");
    if ((USART1->CR1 & USART_CR1_OVER8) != 0)
    {
        /* Integer part computing in case Oversampling mode is 8 Samples */
        integerdivider = ((25 * console_dev->pclk2*1000000) / (2 * (console_dev->bound)));    
    }
    else /* if ((USART1->CR1 & CR1_OVER8_Set) == 0) */
    {
        /* Integer part computing in case Oversampling mode is 16 Samples */
        integerdivider = ((25 * console_dev->pclk2*1000000) / (4 * (console_dev->bound)));    
    }
    tmpreg = (integerdivider / 100) << 4;
    
    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
    
    /* Implement the fractional part in the register */
    if ((USART1->CR1 & USART_CR1_OVER8) != 0)
    {
        tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
    }
    else /* if ((USART1->CR1 & CR1_OVER8_Set) == 0) */
    {
        tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
    }
    
    /* Write to USART BRR */
    
    
    
    /* Disable USART */
    USART1->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE); 
    USART1->BRR = (uint16_t)tmpreg;
    
//#ifdef USART1_REMAP
//    os_clock_open("GPIOB");
//    GPIOB->CRL&=0X00FFFffF; 
//    GPIOB->CRL|=0X8b000000;//IO×´Ì¬ÉèÖÃ
//    RCC->APB1RSTR|=(1<<17); //¸´Î»´®¿Ú2
//    AFIO->MAPR|=1<<2;
//    os_clock_close("GPIOB");
//#else
//    os_clock_open("GPIOA");
//    
//    /* alternate functions  configuration */ 
//    temp_1 = ((u32)(GPIO_AF_1) << ((u32)((u32)UART1_PORT_TX & (u32)0x07) * 4));    
//    UART1_PORT->AFR[UART1_PORT_TX >> 0x03] &= ~((u32)0xF << ((u32)((u32)UART1_PORT_TX & (u32)0x07) * 4));
//    temp_2 = UART1_PORT->AFR[UART1_PORT_TX >> 0x03] | temp_1;
//    UART1_PORT->AFR[UART1_PORT_TX >> 0x03] = temp_2;
//    
//    temp_1 = ((u32)(GPIO_AF_1) << ((u32)((u32)UART1_PORT_RX & (u32)0x07) * 4));    
//    UART1_PORT->AFR[UART1_PORT_RX >> 0x03] &= ~((u32)0xF << ((u32)((u32)UART1_PORT_RX & (u32)0x07) * 4));
//    temp_2 = UART1_PORT->AFR[UART1_PORT_RX >> 0x03] | temp_1;
//    UART1_PORT->AFR[UART1_PORT_RX >> 0x03] = temp_2;
//    
//    /* Speed mode configuration */
//    UART1_PORT->OSPEEDR &= ~((GPIO_Speed_50MHz << (UART1_PORT_TX <<1))|(GPIO_Speed_50MHz<<(UART1_PORT_TX<<1)));
//    UART1_PORT->OSPEEDR |= ((GPIO_Speed_50MHz << (UART1_PORT_TX <<1))|(GPIO_Speed_50MHz<<(UART1_PORT_TX<<1)));
//    
//    /* Output mode configuration */
//    UART1_PORT->OTYPER &= ~(((GPIO_OTYPER_OT_0) << ((uint16_t)UART1_PORT_TX))|((GPIO_OTYPER_OT_0) << ((uint16_t)UART1_PORT_RX)));
//    UART1_PORT->OTYPER |=  (((GPIO_OType_PP) << ((uint16_t)UART1_PORT_TX))|((GPIO_OType_PP) << ((uint16_t)UART1_PORT_RX)));
//    
//    UART1_PORT->MODER  &= ~((GPIO_MODER_MODER0 << (UART1_PORT_TX <<1))|(GPIO_MODER_MODER0<<(UART1_PORT_TX<<1)));
//    
//    UART1_PORT->MODER |= ((GPIO_Mode_AF << (UART1_PORT_TX <<1))|(GPIO_Mode_AF<<(UART1_PORT_RX<<1)));
//    
//    /* Pull-up Pull down resistor configuration */
//    
//    UART1_PORT->PUPDR &= ~((GPIO_PUPDR_PUPDR0 << (UART1_PORT_TX <<1))|(GPIO_PUPDR_PUPDR0<<(UART1_PORT_TX<<1)));
//    
//    UART1_PORT->PUPDR |=  ((GPIO_PuPd_UP << (UART1_PORT_TX <<1))|(GPIO_PuPd_UP<<(UART1_PORT_TX<<1)));
//    
//    os_clock_close("GPIOA");
//#endif	
    
    
    /*---------------------------- USART CR2 Configuration -----------------------*/
    tmpreg = USART1->CR2;
    
    /* Clear STOP[13:12] bits */
    tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);
    
    /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
    /* Set STOP[13:12] bits according to USART_StopBits value */
    tmpreg |= (uint32_t)0;
    
    /* Write to USART CR2 */
    USART1->CR2 = tmpreg;
    /*---------------------------- USART CR1 Configuration -----------------------*/
    tmpreg = USART1->CR1;
    /* Clear M, PCE, PS, TE and RE bits */
    tmpreg &= (uint32_t)~(((uint32_t)(USART_CR1_M | USART_CR1_PCE | \
                                      USART_CR1_PS | USART_CR1_TE | \
                                      USART_CR1_RE)));
    
    /* Configure the USART Word Length, Parity and mode ----------------------- */
    /* Set the M bits according to USART_WordLength value */
    /* Set PCE and PS bits according to USART_Parity value */
    /* Set TE and RE bits according to USART_Mode value */
    tmpreg |= (uint32_t)USART_CR1_RXNEIE|USART_CR1_RE|USART_CR1_TE|USART_CR1_UE;
    
    /* Write to USART CR1 */
    USART1->CR1 = tmpreg;
    
    NVIC_SetPriority(USART1_IRQn , 0x00);
    NVIC_EnableIRQ(USART1_IRQn);
    
    
    os_clock_close("USART1");	
    os_clock_open("USART1");
//    os_clock_open("GPIOA");
    
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
static os_err_t  uart1_open(os_device_t* dev, u16 oflag)
{
    os_clock_open("USART1");
    os_clock_open("GPIOA");
    
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
os_err_t  uart1_close(os_device_t* dev, u16 oflag)
{
    
    os_clock_close("USART1");
    os_clock_close("GPIOA");
    
    return SUCCESS;		
    
}
/*********************************************************************
 * @fn      os_err_t uart2_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart2
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  uart1_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
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
            USART1->TDR=*(char*)temp;
            temp++;
            size--;
            while(!(USART1->ISR&(1<<7)));//Transmit data register empty
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
extern os_err_t  Q_Sh_rx_callback(char data);
extern os_err_t  calibration_rx_callback(u8 data);
static os_err_t  uart1_rx_indicator(os_device_t* dev,os_size_t size)
{
    
      return  Q_Sh_rx_callback(*((char*)dev->user_data));
     //return calibration_rx_callback(*((char*)dev->user_data));
    
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
void USART1_IRQHandler(void)
{
    
    u32 uart1_status;
    u8 usart_recieved;
    if(USART1->ISR&(1<<5))//Read data register not empty
    {	 
        usart_recieved=USART1->RDR;
        console_dev->os_device.user_data=(void*)&usart_recieved;
        console_dev->os_device.rx_indicate(&(console_dev->os_device), 1);
        
        USART1->RQR|=(1<<3);
    }
		
		if(USART1->ISR&(1<<4)) //idle
		{
       usart_recieved=USART1->RDR; 	
       uart1_status=USART1->ISR;
    }
    usart_recieved=USART1->RDR; 	
    uart1_status=USART1->ISR;
    USART1->ISR  =uart1_status;
    
    
} 




#ifdef  _CONSOLE_DEVICE_USART1_OPEN
struct __FILE  
{  
    int handle;
    os_device_t* dev;	
};  
FILE __stdout;  

void sys_exit(int x)  
{  
    x = x;  
}
int fputc(int ch, FILE *f)
{
    console_dev->os_device.write(&(console_dev->os_device),0,&ch,1);
    return ch;
}
#endif
