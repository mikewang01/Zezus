#ifndef __COREFUNC_H
#define __COREFUNC_H	  
#ifdef _STM32F1XXX_

#include <stm32f10x.h>

#endif

#ifdef _STM32F0XXX_

#include <stm32f0xx.h>

#endif

#include "obj.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//ϵͳʱ�ӳ�ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2013/12/26
//�汾��V1.9
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.4�޸�˵��
//��NVIC KO��,û��ʹ���κο��ļ�!
//������JTAG_Set����
//V1.5 20120322
//����void INTX_DISABLE(void)��void INTX_ENABLE(void)��������
//V1.6 20120412
//1,����MSR_MSP����												    
//2,�޸�VECT_TAB_RAM��Ĭ��ƫ��,����Ϊ0X6000.
//V1.7 20120818
//1,���ucos֧�����ú�SYSTEM_SUPPORT_UCOS
//2,�޸���ע��
//3,ȥ���˲����ú���BKP_Write
//V1.8 20131120
//1,�޸�ͷ�ļ�Ϊstm32f10x.h,����ʹ��stm32f10x_lib.h�������ͷ�ļ�
//V1.9 20131226
//1,�޸�ͷ�ļ�ΪMY_NVIC_Init�������ִ�����֧�������Ŵ���63���жϵ�����
////////////////////////////////////////////////////////////////////////////////// 	

//0,��֧��ucos
//1,֧��ucos
#define SYSTEM_SUPPORT_UCOS		0		//����ϵͳ�ļ����Ƿ�֧��UCOS
																	    
	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����
/////////////////////////////////////////////////////////////////
//Ex_NVIC_Configר�ö���
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 

#define FTIR   1  //�½��ش���
#define RTIR   2  //�����ش���
								   

//JTAGģʽ���ö���
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	

/////////////////////////////////////////////////////////////////  
void Stm32_Clock_Init(u8 PLL);  //ʱ�ӳ�ʼ��  
void Sys_Soft_Reset(void);      //ϵͳ��λ
void Sys_Standby(u8 mode);         //����ģʽ 	

void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);//����NVIC����

void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);//�����ж�

void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);//�ⲿ�ж����ú���(ֻ��GPIOA~G)

void JTAG_Set(u8 mode);
//////////////////////////////////////////////////////////////////////////////
//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ

enum {BUSY=2,OS_ENOSYS};

__STATIC_INLINE  void gpio_pin_altset(GPIO_TypeDef *portx , u8 pin_num ,u32 altfunction)
{
    u32 temp_1,temp_2;
	  temp_1 = ((u32)(altfunction) << ((u32)((u32)pin_num & (u32)0x07) * 4));    
		portx->AFR[pin_num >> 0x03] &= ~((u32)0xF << ((u32)((u32)pin_num & (u32)0x07) * 4));
		temp_2 = portx->AFR[pin_num >> 0x03] | temp_1;
		portx->AFR[pin_num >> 0x03] = temp_2;
	
}

__STATIC_INLINE  void gpio_speed_set(GPIO_TypeDef *portx , u8 pin_num ,u32 gpio_speed)
{
       portx->OSPEEDR &=~(gpio_speed << (pin_num <<1));
       portx->OSPEEDR |= (gpio_speed << (pin_num <<1));
}


__STATIC_INLINE  void gpio_outtype_set(GPIO_TypeDef *portx , u8 pin_num ,u32 output_type)
{
		  portx->OTYPER &=~(GPIO_OTYPER_OT_0 << ((uint16_t)pin_num));
	    portx->OTYPER |= (((output_type)   << ((uint16_t)pin_num)));
}

__STATIC_INLINE  void gpio_outmode_set(GPIO_TypeDef *portx , u8 pin_num ,u32 output_mode)
{
         portx->MODER  &= ~((GPIO_MODER_MODER0 << (pin_num <<1)));
         portx->MODER |= ((output_mode << (pin_num <<1)));

}

__STATIC_INLINE  void gpio_pupdr_set(GPIO_TypeDef *portx , u8 pin_num ,u32 pupdr_mode)
{
		     portx->PUPDR &= ~((GPIO_PUPDR_PUPDR0 << (pin_num <<1)));
				 portx->PUPDR |=  ((pupdr_mode      << (pin_num <<1)));	
}


												
#endif











