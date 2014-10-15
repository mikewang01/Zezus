
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

#define USART1RX_DMA1_Channelx DMA1_Channel5
#define USART1TX_DMA1_Channelx DMA1_Channel4

/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */


static usart_device *console_dev=NULL;

/*********************************************************************
 * LOCAL function declarement
 */


static  os_err_t   uart2_init   (os_device_t* dev);

static  os_size_t  uart2_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);

static  os_err_t   uart2_rx_indicator(os_device_t* dev,os_size_t size);

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
os_err_t usart2_register(u16 task_id)
{
	
	console_dev=osmalloc(sizeof(usart_device));
	
	
	console_dev->pclk2=SYSCLK;
	console_dev->bound=BOUND;
	
	console_dev->os_device.type = OS_Device_Class_Char;
	
	console_dev->os_device.device_id = OS_DEVICE_USART_ID;
	
	console_dev->register_taskid = task_id;
	
	console_dev->os_device.init  = uart2_init;
	
	console_dev->os_device.open  = uart2_open;
	console_dev->os_device.write = uart2_write;
  console_dev->os_device.rx_indicate = uart2_rx_indicator;
	  
	return os_device_register(&(console_dev->os_device), USART2_NAME, OS_DEVICE_FLAG_INACTIVATED);
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
os_err_t  uart2_init   (os_device_t* dev)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	
  os_device_t *__mptr = (dev);
	usart_device * console_dev=(usart_device *)( (char *)__mptr - offsetof(usart_device,os_device));	
  
	temp=(float)(console_dev->pclk2*500000)/(console_dev->bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction;

	os_clock_open("USART2");
  

	
#ifdef USART2_REMAP
	os_clock_open("GPIOB");
	GPIOB->CRL&=0X00FFFffF; 
	GPIOB->CRL|=0X8b000000;//IO状态设置
		RCC->APB1RSTR|=(1<<17); //复位串口2
  AFIO->MAPR|=1<<2;
	os_clock_close("GPIOB");
#else
	os_clock_open("GPIOA");
	GPIOA->CRL&=0XFFFF00FF; //PA2 TX P A3 RX
	GPIOA->CRL|=0X00008B00;//IO状态设置
	RCC->APB1RSTR|=(1<<17);   //复位串口2
  os_clock_close("GPIOA");
#endif	
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	


//波特率设置
 	USART2->BRR=mantissa; // 波特率设置	 
	USART2->CR1|=0X200C;  //1位停止,无校验位.
	

	USART2->CR1|=1<<5;//RXNE interrupt enable
	//USART2->CR1|=1<<4;// IDLE interrupt enable
	MY_NVIC_Init(3,3,USART2_IRQn,2);//组2，最低优先级
	
  os_clock_close("USART2");
	
	 os_clock_open("USART2");
	 os_clock_open("GPIOA");
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
static os_err_t  uart2_open(os_device_t* dev, u16 oflag)
{
	 os_clock_open("USART2");
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
os_err_t  uart2_close(os_device_t* dev, u16 oflag)
{
	
    os_clock_close("USART2");
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

static os_size_t  uart2_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
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
					USART2->DR=*(char*)temp;
				  temp++;
				  size--;
				 while(!(USART2->SR&(1<<7)));//Transmit data register empty
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
static os_err_t  uart2_rx_indicator(os_device_t* dev,os_size_t size)
{
	
  return  Q_Sh_rx_callback(*((char*)dev->user_data));

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
void USART2_IRQHandler(void)
{

  u32 uart2_status;
  u8 usart_recieved;
	if(USART2->SR&(1<<5))//Read data register not empty
	{	 
		usart_recieved=USART2->DR;
		console_dev->os_device.user_data=(void*)&usart_recieved;
		console_dev->os_device.rx_indicate(&(console_dev->os_device), 1);
		
		USART2->SR|=(1<<5);
	}  	
	uart2_status=USART2->SR;
	USART2->SR^=uart2_status;
		  
						 
} 





struct __FILE  
{  
	int handle;  
};  
FILE __stdout;  

_sys_exit(int x)  
{  
	x = x;  
}
int fputc(int ch, FILE *f)
{
	console_dev->os_device.write(&(console_dev->os_device),0,&ch,1);
	return ch;
}


