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

//�����ж�,��ӳ�� 
void Timer2_Init(void)
{	

	RCC->APB1ENR|=1<<0;//TIM2ʱ��ʹ��    
	TIM2->PSC=17;		//Ԥ��Ƶ��18,�õ�4Mhz�ļ���ʱ��
	TIM2->ARR=90;		//�Զ���װ
	TIM2->CNT=90;		//44Khz ADC������										
	TIM2->DIER|=1<<0;   //���������ж�	
	TIM2->DIER|=1<<6;   //���������ж�
	TIM2->SR|=(1<<0);		 //����жϱ�־
	TIM2->CR1=1<<7;         //ARPEʹ�� 
	TIM2->CCMR1&=~(3<<8);  //CC2 channel is configured as output
	TIM2->CCMR1|=(1<<11);	 //Output compare 2 preload enable
	TIM2->CCMR1|=(6<<12); //PWM mode 1 
	TIM2->CCR2=90;     //TIMx capture/compare register 2
	TIM2->CCER|=1<<4; //Capture/Compare 2 output enable
	TIM2->CR1|=1<<0;  //ʹ�ܶ�ʱ��2				 					  										  
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
	RCC->APB2ENR|=1<<2;		//��ʹ������IO PORTBʱ��
	GPIOA->CRL&=0xfffff0ff;	//PA2ģ������
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
	RCC->CFGR|=(ADC_CLK_12<<14);			//ADCʱ��12M
	RCC->APB2ENR|=(1<<9);					//ʹ��ADC1ʱ��
	ADC1->CR1|=(1<<8);						//ģʽ��������ɨ��

	ADC1->CR2|=(0<<1)|(1<<8);				//����ת��,DMAʹ��
	ADC1->CR2|=(1<<20);	// External trigger conversion mode for regular channels
	ADC1->CR2|=(3<<17);//011: Timer 2 CC2 event
	//ADC1->CR2|=(0<<1)|(0<<8);				//����ת��,DMA��ʹ��
	ADC1->SMPR1|=(5<<0)|(5<<3)|(5<<6);		//��������Ϊ55.5��ADC_CLK
	ADC1->SQR1|=(0<<20);					//��ת��1��ͨ��
	ADC1->SQR3|=(2<<0);						//����ת��ͨ��2  PA2
	ADC1->CR2|=(1<<0);					//ʹ��ADC1
	ADC1->CR2|=(1<<0);					//ʹ��ADC1,����Ҫִ�����βſ�������ADC������������i don't know why
	ADC1->CR2|=(1<<3);					//��λУ׼
	while(ADC1->CR2&(1<<3));			//�ȴ���λУ׼����
	ADC1->CR2|=(1<<2);					//ADУ׼
	while((ADC1->CR2)&(1<<2));			//�ȴ�ADУ׼����
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
   
	RCC->AHBENR|=1<<0;//����DMA1ʱ��
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
	MY_NVIC_Init(1,2,DMA1_Channel1_IRQn ,2);//��ռ1�������ȼ�3����2
	DMA1_Channel1->CCR|=1<<0;          //����DMA����
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
//����һ��DMA����
void MYDMA_Enable()
{
	DMA1_Channel1->CCR&=~(1<<0);       //�ر�DMA���� 
//	DMA1_Channel1->CNDTR=NPT; //DMA1,���������� 
	DMA1_Channel1->CCR|=1<<0;          //����DMA����
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
   
	ADC1->CR2|=(1<<0);			//����ת��
	ADC1->CR2|=(1<<0);			//����ת��
	while((ADC1->SR&(1<<1))==0);	//�ȴ�ת������
	return (ADC1->DR);	//��ֵ��С
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

 	 	
