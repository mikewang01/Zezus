 /***************************************************************
 * Name:      ADC.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-05-15
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



/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL VARIABLES
 */
/*********************************************************************
 * FUNCTIONS
 */



/*********************************************************************
 * @fn      void Timer2_Init
 *
 * @brief  init time2 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */			   

//开启中断,并映射 
void Timer2_Init(void)
{	

	RCC->APB1ENR|=1<<0;//TIM2时钟使能    
	TIM2->PSC=17;		//预分频器18,得到4Mhz的计数时钟
	TIM2->ARR=90;		//自动重装
	TIM2->CNT=90;		//44Khz ADC采样率										
	TIM2->DIER|=1<<0;   //允许更新中断	
	TIM2->DIER|=1<<6;   //允许触发中断
	TIM2->SR|=(1<<0);		 //清楚中断标志
	TIM2->CR1=1<<7;         //ARPE使能 
	TIM2->CCMR1&=~(3<<8);  //CC2 channel is configured as output
	TIM2->CCMR1|=(1<<11);	 //Output compare 2 preload enable
	TIM2->CCMR1|=(6<<12); //PWM mode 1 
	TIM2->CCR2=90;     //TIMx capture/compare register 2
	TIM2->CCER|=1<<4; //Capture/Compare 2 output enable
	TIM2->CR1|=1<<0;  //使能定时器2				 					  										  
} 
/*********************************************************************
 * @fn      void Timer2_Init
 *
 * @brief  init time2 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */	
void ADC_GPIO_Init(void)
{
	RCC->APB2ENR|=1<<2;		//先使能外设IO PORTB时钟
	GPIOA->CRL&=0xfffff0ff;	//PA2模拟输入
}
/*********************************************************************
 * @fn      void Timer2_Init
 *
 * @brief  init time2 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */	
void ADC1_Init()
{
	RCC->CFGR|=(ADC_CLK_12<<14);			//ADC时钟12M
	RCC->APB2ENR|=(1<<9);					//使能ADC1时钟
	ADC1->CR1|=(1<<8);						//模式：独立，扫描

	ADC1->CR2|=(0<<1)|(1<<8);				//连续转换,DMA使能
	ADC1->CR2|=(1<<20);	// External trigger conversion mode for regular channels
	ADC1->CR2|=(3<<17);//011: Timer 2 CC2 event
	//ADC1->CR2|=(0<<1)|(0<<8);				//单次转换,DMA不使能
	ADC1->SMPR1|=(5<<0)|(5<<3)|(5<<6);		//采样周期为55.5个ADC_CLK
	ADC1->SQR1|=(0<<20);					//共转换1个通道
	ADC1->SQR3|=(2<<0);						//依次转换通道2  PA2
	ADC1->CR2|=(1<<0);					//使能ADC1
	ADC1->CR2|=(1<<0);					//使能ADC1,必须要执行两次才可以启动ADC，否则不启动，i don't know why
	ADC1->CR2|=(1<<3);					//复位校准
	while(ADC1->CR2&(1<<3));			//等待复位校准结束
	ADC1->CR2|=(1<<2);					//AD校准
	while((ADC1->CR2)&(1<<2));			//等待AD校准结束
}

/*********************************************************************
 * @fn      void Timer2_Init
 *
 * @brief  init time2 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */	
void DMA_Init4()
{
   
	RCC->AHBENR|=1<<0;//开启DMA1时钟
	DMA1_Channel1->CCR=0;//reset
	DMA1_Channel1->CCR|=2<<10;//Memory size 16-bits	 2 32bits
	DMA1_Channel1->CCR|=1<<8;//Peripheral size1  16-bits  2 32bit
	DMA1_Channel1->CCR|=1<<7;//Memory increment mode enabled
	DMA1_Channel1->CCR&=~(1<<6);	//Peripheral increment mode disabled
	DMA1_Channel1->CCR&=~(1<<4);//Read from peripheral
	DMA1_Channel1->CCR|=1<<1;//Transfer complete interrupt enable
//	DMA1_Channel1->CNDTR=NPT;//DMA channel x number of data register

	DMA1_Channel1->CPAR=(unsigned long)(&ADC1->DR);
	//DMA1_Channel1->CMAR=(unsigned long)(lBUFIN);
	MY_NVIC_Init(1,2,DMA1_Channel1_IRQn ,2);//抢占1，子优先级3，组2
	DMA1_Channel1->CCR|=1<<0;          //开启DMA传输
}
/*********************************************************************
 * @fn      void Timer2_Init
 *
 * @brief  init time2 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */	
//开启一次DMA传输
void MYDMA_Enable()
{
	DMA1_Channel1->CCR&=~(1<<0);       //关闭DMA传输 
//	DMA1_Channel1->CNDTR=NPT; //DMA1,传输数据量 
	DMA1_Channel1->CCR|=1<<0;          //开启DMA传输
} 

/*********************************************************************
 * @fn      void Timer2_Init
 *
 * @brief  init time2 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */	
 void ADC_int(void)
{
	ADC_GPIO_Init();
	Timer2_Init();
	ADC1_Init();
	DMA_Init4();
//	Timer2_Init();	
}


/*********************************************************************
 * @fn      void Timer2_Init
 *
 * @brief  init time2 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */	
u16 ADC_Get(void)
{
   
	ADC1->CR2|=(1<<0);			//启动转换
	ADC1->CR2|=(1<<0);			//启动转换
	while((ADC1->SR&(1<<1))==0);	//等待转换结束
	return (ADC1->DR);	//让值减小
}


/*********************************************************************
 * @fn      void Timer2_Init
 *
 * @brief  init time2 to interrupt every 10ms 100HZ
 *				
 * 			    @param NONE 
 *			
 * 
 *   @return none 
 */	
void DMAChannel1_IRQHandler()
{
   
   if(DMA1->ISR&(1<<1))
   {
   	  // MYDMA_Enable();
	   
	 
   }
	DMA1->IFCR|=1<<1;
}

 	 	

