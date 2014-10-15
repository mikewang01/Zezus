#line 1 "..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C"







 



 
#line 1 "..\\incs\\driver\\stm32f051x\\me2_ch2o.h"




#line 1 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"
#line 8 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"







#line 1 "..\\incs\\kernel\\obj.h"







 







 
 
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\cpu.h"







 





#line 1 "..\\incs\\asm-arm\\stm32f0xx\\config.h"







 



 








typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned  int  u32;



























 




 






#line 73 "..\\incs\\asm-arm\\stm32f0xx\\config.h"
 

















#line 15 "..\\incs\\asm-arm\\stm32f0xx\\cpu.h"









#line 1 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"










































 



 



 
    








  


 
  


 










 










 
   








 







 







 




























 
#line 154 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"



 



 




 





 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  HardFault_IRQn              = -13,     
  SVC_IRQn                    = -5,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  RTC_IRQn                    = 2,       
  FLASH_IRQn                  = 3,       
  RCC_IRQn                    = 4,       
  EXTI0_1_IRQn                = 5,       
  EXTI2_3_IRQn                = 6,       
  EXTI4_15_IRQn               = 7,       
  TS_IRQn                     = 8,       
  DMA1_Channel1_IRQn          = 9,       
  DMA1_Channel2_3_IRQn        = 10,      
  DMA1_Channel4_5_IRQn        = 11,      
  ADC1_COMP_IRQn              = 12,      
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,      
  TIM1_CC_IRQn                = 14,      
  TIM2_IRQn                   = 15,      
  TIM3_IRQn                   = 16,      
  TIM6_DAC_IRQn               = 17,      
  TIM14_IRQn                  = 19,      
  TIM15_IRQn                  = 20,      
  TIM16_IRQn                  = 21,      
  TIM17_IRQn                  = 22,      
  I2C1_IRQn                   = 23,      
  I2C2_IRQn                   = 24,      
  SPI1_IRQn                   = 25,      
  SPI2_IRQn                   = 26,      
  USART1_IRQn                 = 27,      
  USART2_IRQn                 = 28,      
  CEC_IRQn                    = 30       
} IRQn_Type;



 

#line 1 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
 







 

























 
























 




 


 

 













#line 100 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"


 







#line 125 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

#line 1 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 127 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
#line 1 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 



#line 292 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"



#line 685 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 128 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
#line 1 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 271 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 307 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 634 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 


#line 129 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"








 
#line 154 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

 






 
#line 170 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
  else {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}




 








#line 219 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"
#line 1 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\system_stm32f0xx.h"

























 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 220 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"
#line 221 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"



   

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;



    



 

typedef struct
{
  volatile uint32_t ISR;           
  volatile uint32_t IER;           
  volatile uint32_t CR;            
  volatile uint32_t CFGR1;         
  volatile uint32_t CFGR2;         
  volatile uint32_t SMPR;          
  uint32_t   RESERVED1;        
  uint32_t   RESERVED2;        
  volatile uint32_t TR;            
  uint32_t   RESERVED3;        
  volatile uint32_t CHSELR;        
  uint32_t   RESERVED4[5];     
   volatile uint32_t DR;           
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CCR;
} ADC_Common_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;            
  volatile uint32_t CFGR;          
  volatile uint32_t TXDR;          
  volatile uint32_t RXDR;          
  volatile uint32_t ISR;           
  volatile uint32_t IER;           
}CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;      
} COMP_TypeDef;




 

typedef struct
{
  volatile uint32_t DR;        
  volatile uint8_t  IDR;       
  uint8_t   RESERVED0;     
  uint16_t  RESERVED1;     
  volatile uint32_t CR;        
  uint32_t  RESERVED2;     
  volatile uint32_t INIT;      
} CRC_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;            
  volatile uint32_t SWTRIGR;       
  volatile uint32_t DHR12R1;       
  volatile uint32_t DHR12L1;       
  volatile uint32_t DHR8R1;        
       uint32_t RESERVED[6];   
  volatile uint32_t DOR1;          
       uint32_t RESERVED1;     
  volatile uint32_t SR;            
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;        
  volatile uint32_t CR;            
  volatile uint32_t APB1FZ;        
  volatile uint32_t APB2FZ;        
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;           
  volatile uint32_t CNDTR;         
  volatile uint32_t CPAR;          
  volatile uint32_t CMAR;          
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;           
  volatile uint32_t IFCR;          
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;           
  volatile uint32_t EMR;           
  volatile uint32_t RTSR;          
  volatile uint32_t FTSR;          
  volatile uint32_t SWIER;         
  volatile uint32_t PR;            
}EXTI_TypeDef;



 
typedef struct
{
  volatile uint32_t ACR;           
  volatile uint32_t KEYR;          
  volatile uint32_t OPTKEYR;       
  volatile uint32_t SR;            
  volatile uint32_t CR;            
  volatile uint32_t AR;            
  volatile uint32_t RESERVED;      
  volatile uint32_t OBR;           
  volatile uint32_t WRPR;          
} FLASH_TypeDef;




 
typedef struct
{
  volatile uint16_t RDP;           
  volatile uint16_t USER;          
  uint16_t RESERVED0;          
  uint16_t RESERVED1;          
  volatile uint16_t WRP0;          
  volatile uint16_t WRP1;          
} OB_TypeDef;
  



 

typedef struct
{
  volatile uint32_t MODER;         
  volatile uint16_t OTYPER;        
  uint16_t RESERVED0;          
  volatile uint32_t OSPEEDR;       
  volatile uint32_t PUPDR;         
  volatile uint16_t IDR;           
  uint16_t RESERVED1;          
  volatile uint16_t ODR;           
  uint16_t RESERVED2;          
  volatile uint32_t BSRR;          
  volatile uint32_t LCKR;          
  volatile uint32_t AFR[2];        
  volatile uint16_t BRR;           
  uint16_t RESERVED3;          
}GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CFGR1;        
       uint32_t RESERVED;     
  volatile uint32_t EXTICR[4];    
  volatile uint32_t CFGR2;        
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;       
  volatile uint32_t CR2;       
  volatile uint32_t OAR1;      
  volatile uint32_t OAR2;      
  volatile uint32_t TIMINGR;   
  volatile uint32_t TIMEOUTR;  
  volatile uint32_t ISR;       
  volatile uint32_t ICR;       
  volatile uint32_t PECR;      
  volatile uint32_t RXDR;      
  volatile uint32_t TXDR;      
}I2C_TypeDef;




 
typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
  volatile uint32_t WINR;  
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;




 
typedef struct
{
  volatile uint32_t CR;          
  volatile uint32_t CFGR;        
  volatile uint32_t CIR;         
  volatile uint32_t APB2RSTR;    
  volatile uint32_t APB1RSTR;    
  volatile uint32_t AHBENR;      
  volatile uint32_t APB2ENR;     
  volatile uint32_t APB1ENR;     
  volatile uint32_t BDCR;         
  volatile uint32_t CSR;         
  volatile uint32_t AHBRSTR;     
  volatile uint32_t CFGR2;       
  volatile uint32_t CFGR3;       
  volatile uint32_t CR2;         
} RCC_TypeDef;



 

typedef struct
{                           
  volatile uint32_t TR;          
  volatile uint32_t DR;          
  volatile uint32_t CR;          
  volatile uint32_t ISR;         
  volatile uint32_t PRER;        
       uint32_t RESERVED0;   
       uint32_t RESERVED1;   
  volatile uint32_t ALRMAR;      
       uint32_t RESERVED2;   
  volatile uint32_t WPR;         
  volatile uint32_t SSR;         
  volatile uint32_t SHIFTR;      
  volatile uint32_t TSTR;        
  volatile uint32_t TSDR;        
  volatile uint32_t TSSSR;       
  volatile uint32_t CAL;         
  volatile uint32_t TAFCR;       
  volatile uint32_t ALRMASSR;    
       uint32_t RESERVED3;   
       uint32_t RESERVED4;   
  volatile uint32_t BKP0R;       
  volatile uint32_t BKP1R;       
  volatile uint32_t BKP2R;       
  volatile uint32_t BKP3R;       
  volatile uint32_t BKP4R;       
} RTC_TypeDef;




 
  
typedef struct
{
  volatile uint16_t CR1;       
  uint16_t  RESERVED0;     
  volatile uint16_t CR2;       
  uint16_t  RESERVED1;     
  volatile uint16_t SR;        
  uint16_t  RESERVED2;     
  volatile uint16_t DR;        
  uint16_t  RESERVED3;     
  volatile uint16_t CRCPR;     
  uint16_t  RESERVED4;     
  volatile uint16_t RXCRCR;    
  uint16_t  RESERVED5;     
  volatile uint16_t TXCRCR;    
  uint16_t  RESERVED6;      
  volatile uint16_t I2SCFGR;   
  uint16_t  RESERVED7;     
  volatile uint16_t I2SPR;     
  uint16_t  RESERVED8;         
} SPI_TypeDef;




 
typedef struct
{
  volatile uint16_t CR1;              
  uint16_t      RESERVED0;        
  volatile uint16_t CR2;              
  uint16_t      RESERVED1;        
  volatile uint16_t SMCR;             
  uint16_t      RESERVED2;        
  volatile uint16_t DIER;             
  uint16_t      RESERVED3;        
  volatile uint16_t SR;               
  uint16_t      RESERVED4;        
  volatile uint16_t EGR;              
  uint16_t      RESERVED5;        
  volatile uint16_t CCMR1;            
  uint16_t      RESERVED6;        
  volatile uint16_t CCMR2;            
  uint16_t      RESERVED7;        
  volatile uint16_t CCER;             
  uint16_t      RESERVED8;        
  volatile uint32_t CNT;              
  volatile uint16_t PSC;              
  uint16_t      RESERVED10;       
  volatile uint32_t ARR;              
  volatile uint16_t RCR;              
  uint16_t      RESERVED12;       
  volatile uint32_t CCR1;             
  volatile uint32_t CCR2;             
  volatile uint32_t CCR3;             
  volatile uint32_t CCR4;             
  volatile uint16_t BDTR;             
  uint16_t      RESERVED17;       
  volatile uint16_t DCR;              
  uint16_t      RESERVED18;       
  volatile uint16_t DMAR;             
  uint16_t      RESERVED19;       
  volatile uint16_t OR;               
  uint16_t      RESERVED20;       
} TIM_TypeDef;



 
typedef struct
{
  volatile uint32_t CR;         
  volatile uint32_t IER;        
  volatile uint32_t ICR;         
  volatile uint32_t ISR;        
  volatile uint32_t IOHCR;      
  volatile uint32_t RESERVED1;  
  volatile uint32_t IOASCR;     
  volatile uint32_t RESERVED2;  
  volatile uint32_t IOSCR;      
  volatile uint32_t RESERVED3;  
  volatile uint32_t IOCCR;      
  volatile uint32_t RESERVED4;  
  volatile uint32_t IOGCSR;     
  volatile uint32_t IOGXCR[6];  
} TSC_TypeDef;



 
  
typedef struct
{
  volatile uint32_t CR1;      
  volatile uint32_t CR2;      
  volatile uint32_t CR3;     
  volatile uint16_t BRR;     
  uint16_t  RESERVED1;     
  volatile uint16_t GTPR;    
  uint16_t  RESERVED2;   
  volatile uint32_t RTOR;      
  volatile uint16_t RQR;     
  uint16_t  RESERVED3;   
  volatile uint32_t ISR;     
  volatile uint32_t ICR;     
  volatile uint16_t RDR;     
  uint16_t  RESERVED4;   
  volatile uint16_t TDR;     
  uint16_t  RESERVED5;   
} USART_TypeDef;




 
typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;




 
  


 





 




#line 685 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

#line 698 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

#line 710 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"









 
  


   

#line 739 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

#line 752 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

#line 764 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"









 



 
  
  

 
    
 
 
 
 
 
 
 
 
 
#line 798 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 
#line 809 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 






 
#line 848 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 



 





 


 


 
#line 888 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 




 
 
 
 
 

 




 
#line 918 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 


 
#line 939 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 954 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
 
 
 
 
 
 
#line 981 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"
 
#line 1001 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
 
 
 
 
 


 


 






 


 
 
 
 
 
 











 


 


 


 


 


 



 
 
 
 
 

 


#line 1085 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 



 
#line 1099 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 





 
 
 
 
 

 
#line 1133 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1155 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1165 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"















 


 


 


 
 
 
 
 
 
#line 1219 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1245 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1266 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1287 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1308 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1329 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
 
 
 
 

 





 


 


 



                                                               




 





 
#line 1375 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 




#line 1390 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 


 


 

 



 



 



 



 
 
 
 
 
 
#line 1472 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1490 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1540 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1590 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1608 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1626 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1660 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1679 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1689 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1699 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1717 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
 
 
 
 

 
#line 1746 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1759 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 




 




 






 






 
#line 1802 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 1813 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 


 


 
 
 
 
 
 


 





 


 




 


 
 
 
 
 

 











 
#line 1875 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"



 








 
 
 
 
 

 
#line 1904 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
 








 








 






#line 1940 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 











 









 












#line 1991 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 





#line 2006 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2028 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2039 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2053 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2065 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2076 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2090 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 












 








   
#line 2125 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2133 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
 






#line 2158 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
 



 




 






 
 
 
 
 
 
#line 2209 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2239 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2259 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2273 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 



 
#line 2319 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 


 



 
#line 2358 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2378 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 
#line 2396 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2416 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2424 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 


 


 


 


 
 
 
 
 
 
#line 2463 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2481 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2498 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 


 


 


 
#line 2526 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 




 
 
 
 
 
 
#line 2550 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 







 







  







 







 




 







 







 







 







 





 







 






 






 






 




 







 






 






 






 




 





 
 
 
 
 
 
















 









#line 2740 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 



























 
#line 2785 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2799 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2809 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 




























 





















 




























 





















 
#line 2928 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 


 


 


 


 


 


 


 
#line 2963 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"





#line 2974 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 2982 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

#line 2989 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 





 
 
 
 
 
 
#line 3035 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 3059 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 3085 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 



 




 



 






 
#line 3129 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 
#line 3143 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"

 


 


 
 
 
 
 

 
#line 3165 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"



 
#line 3177 "d:\\Keil\\ARM\\Inc\\ST\\STM32F0xx\\stm32f0xx.h"







 




 

 

  







 


 









 

  

 

 
#line 25 "..\\incs\\asm-arm\\stm32f0xx\\cpu.h"






#line 19 "..\\incs\\kernel\\obj.h"


typedef u32  os_off_t;
typedef u8   os_err_t;
typedef u32  os_size_t;

enum os_object_class_type
			{  
				OS_Object_Class_Thread = 0, 
				
				OS_Object_Class_Clock,
				



				



				



				
#line 53 "..\\incs\\kernel\\obj.h"
				



				

				OS_Object_Class_Device,    

				
				OS_Object_Class_Timer, 
				



				
				OS_Object_Class_Unknown, 
				
				OS_Object_Class_Static = 0x80           
			}; 
			

struct os_object
	{
		char       				name[10];
    u8 				 				type;
    u8 				 				flag;
    struct os_object *next;
	};			   



 
struct os_object * os_object_find(struct os_object *object_dev_list,const char * name);
os_err_t os_object_init(struct os_object **object_list , struct os_object *obj, enum os_object_class_type class_type,  const char *name);	
os_err_t os_object_detach(struct os_object **object_list_header, struct os_object *obj);
	


#line 16 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"

































																	    
	 







#line 65 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"

#line 73 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"
 
























#line 105 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"



								   







void Stm32_Clock_Init(u8 PLL);  
void Sys_Soft_Reset(void);      
void Sys_Standby(u8 mode);         

void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);

void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);

void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);

void JTAG_Set(u8 mode);


void WFI_SET(void);		
void INTX_DISABLE(void);
void INTX_ENABLE(void);	
void MSR_MSP(u32 addr);	

enum {BUSY=2,OS_ENOSYS};

static __inline  void gpio_pin_altset(GPIO_TypeDef *portx , u8 pin_num ,u32 altfunction)
{
    u32 temp_1,temp_2;
	  temp_1 = ((u32)(altfunction) << ((u32)((u32)pin_num & (u32)0x07) * 4));    
		portx->AFR[pin_num >> 0x03] &= ~((u32)0xF << ((u32)((u32)pin_num & (u32)0x07) * 4));
		temp_2 = portx->AFR[pin_num >> 0x03] | temp_1;
		portx->AFR[pin_num >> 0x03] = temp_2;
	
}

static __inline  void gpio_speed_set(GPIO_TypeDef *portx , u8 pin_num ,u32 gpio_speed)
{
       portx->OSPEEDR &=~(gpio_speed << (pin_num <<1));
       portx->OSPEEDR |= (gpio_speed << (pin_num <<1));
}


static __inline  void gpio_outtype_set(GPIO_TypeDef *portx , u8 pin_num ,u32 output_type)
{
		  portx->OTYPER &=~(((uint32_t)0x00000001) << ((uint16_t)pin_num));
	    portx->OTYPER |= (((output_type)   << ((uint16_t)pin_num)));
}

static __inline  void gpio_outmode_set(GPIO_TypeDef *portx , u8 pin_num ,u32 output_mode)
{
         portx->MODER  &= ~((((uint32_t)0x00000003) << (pin_num <<1)));
         portx->MODER |= ((output_mode << (pin_num <<1)));

}

static __inline  void gpio_pupdr_set(GPIO_TypeDef *portx , u8 pin_num ,u32 pupdr_mode)
{
		     portx->PUPDR &= ~((((uint32_t)0x00000003) << (pin_num <<1)));
				 portx->PUPDR |=  ((pupdr_mode      << (pin_num <<1)));	
}


												












#line 6 "..\\incs\\driver\\stm32f051x\\me2_ch2o.h"
#line 1 "..\\incs\\kernel\\kernel-includes.h"
 



































#line 38 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\device.h"







 





 

#line 17 "..\\incs\\kernel\\device.h"
#line 18 "..\\incs\\kernel\\device.h"
#line 19 "..\\incs\\kernel\\device.h"



 






 







 
typedef u32 size_t;






					



 
 				 

   
	enum os_device_class_type  
	{  
			OS_Device_Class_Char = 0,                              
	    OS_Device_Class_Block,                                 
	    OS_Device_Class_NetIf,                                 
	    OS_Device_Class_MTD,                                   
	    OS_Device_Class_CAN,                                   
	    OS_Device_Class_RTC,                                   
	    OS_Device_Class_Sound,                                 
	    OS_Device_Class_Graphic,                               
	    OS_Device_Class_I2CBUS,                                
	    OS_Device_Class_USBDevice,                             
	    OS_Device_Class_USBHost,                               
	    OS_Device_Class_SPIBUS,                                
	    OS_Device_Class_SPIDevice,                             
	    OS_Device_Class_SDIO,                                  
	    OS_Device_Class_PM,                                   
      OS_Device_Class_Pipe,																  
      OS_Device_Class_Portal,															  
		  OS_Device_Class_Misc,																   
	    OS_Device_Class_Unknown                                
	};  
	
	
enum{
	OS_DEVICE_FLAG_ACTIVATED  = 0X01,
	OS_DEVICE_FLAG_INACTIVATED= 0X02,
	RT_DEVICE_FLAG_STANDALONE = 0X04,
	RT_DEVICE_OFLAG_OPEN      = 0X08,
	RT_DEVICE_OFLAG_CLOSE     =	0X10
};

enum{
	OS_DEVICE_USART_ID=0x01,
	OS_DEVICE_KEY_ID,

};


 
 

	

   
	

	
	
typedef struct rt_device  os_device_t;
struct rt_device  
 	{  
  	    struct os_object          parent;                     
	  
  	    enum os_device_class_type type;                      
  	    u16               flag;                       
  	    u16               open_flag;                 
	  
  	    u8                device_id;                  
	  
  	       
  	    os_err_t (*rx_indicate)(os_device_t* dev, os_size_t size);  
  	    os_err_t (*tx_complete)(os_device_t* dev, void *buffer);   
 	  
 	       
  	    os_err_t  (*init)   (os_device_t* dev);
  	    os_err_t  (*open)   (os_device_t* dev, u16 oflag);
  	    os_err_t  (*close)  (os_device_t* dev);
   	    os_size_t (*read)   (os_device_t* dev, os_off_t pos, void *buffer, os_size_t size);
        os_size_t (*write)  (os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);
	      os_err_t  (*control)(os_device_t* dev, u8 cmd, void *args);
	  




	  
	    void                     *user_data;                 
	};  




   


 
   

   




 
os_err_t os_device_register(os_device_t *dev, const char  *name,  u16 flags); 
	
os_err_t os_device_unregister(os_device_t *dev);
	
os_err_t os_device_init_all(void);

os_err_t os_device_open(os_device_t *dev, u16 oflag);

os_err_t os_device_close(os_device_t* dev);

os_size_t os_device_read(os_device_t *dev,  
	                         os_off_t    pos,  
	                         void       *buffer,  
	                         os_size_t   size); 
													 
	os_size_t os_device_write(os_device_t *dev,  
	                          os_off_t    pos,  
	                          const void *buffer,  
	                          os_size_t   size);

os_err_t os_device_control(os_device_t *dev, u8 cmd, void *arg);

os_err_t os_device_set_rx_indicate(os_device_t *dev,   os_err_t (*rx_ind)(os_device_t *dev, os_size_t size));

os_err_t os_device_set_tx_complete(os_device_t *dev,  os_err_t (*tx_done)(os_device_t *dev, void *buffer)) ;	
os_device_t* os_device_get(const char * name) ;														


 









#line 39 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\hal_timer.h"







 



 




 
#line 19 "..\\incs\\kernel\\hal_timer.h"


 



 
typedef struct timer
{
	u8  type; 
	u8  task_id;
	u8  event;  
	u32 start_time;
	u32 destination_time;
	u32 interval;
	void (*triger_callback)(void);
	struct timer *next;

}stimer;


typedef  struct 
{
	u8 tasid;
	stimer ptimer;
	

}task_timer;











 


				



void timer_process(void);
u8 os_timer_period(u8 taskid ,u16 event, u32  ticks_expired , void (*triger_callback)(void));
u8 os_timer_expired(u8 taskid ,u16 event, u32  ticks_expired , void (*triger_callback)(void));
u8 del_timer_struct(u8 task_id,u16 event);


#line 40 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\version.h"







 



 




 



 



































 

#line 41 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\mem.h"







 








 
#line 19 "..\\incs\\kernel\\mem.h"


 










 



 

struct _m_mallco_dev
{
	void (*init)(void);     
	u8 (*perused)(void);         
	u8  membase[1*1024];   
	u16 memmap[1*1024/16];  
	u8  memrdy;        
	u16 mem_current_used;
	u16 mem_max_used;
};

 
extern struct _m_mallco_dev mallco_dev;  

void osmemset(void *s,u8 c,u32 count);  
void osmemcpy(void *des,void *src,u32 n);

void mem_init(void);      
static u32 mem_malloc(u32 size);     
static u8 mem_free(u32 offset);     
u8 mem_perused(void);      


u8 osfree(void *ptr);       
void *osmalloc(u32 size);     
void *osrealloc(void *ptr,u32 size);  
void *os_memmove(void *dest, const void *src, u32 n); 
u16 crc16(u8 *ptr,u8 len);



#line 42 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\OS.H"








 




 
#line 16 "..\\incs\\kernel\\OS.H"



 


 



 

typedef void (*Task)();
typedef  unsigned char  os_err_t;

typedef struct
{
	u16 State;
	u8  Delay;
	u32 task_tick;
	void *ptr;
} PCB;

 

void OS_Init(void);

void Shedule(void);
PCB* OS_get_taskstate(void);
u32 OS_get_tick(void);
void os_show_version(void);







void  exam_assert( char * file_name, unsigned int line_no );




#line 43 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\message.h"




#line 6 "..\\incs\\kernel\\message.h"

#line 8 "..\\incs\\kernel\\message.h"


typedef struct msg
{
	u8 type;
	u32 event;
	u16 length;
	void * ptr;
	struct msg * next;
}smessage,*psmessage;

os_err_t send_message(u8 task_id,u16 pcb_type,u16 message_type , u16 event,void *data, u16 length);
os_err_t get_message(PCB* task_state , smessage** msg);
os_err_t delete_message(u8 task_id,u8 message_type);












	



#line 44 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\event.h"






























 



 

#line 38 "..\\incs\\kernel\\event.h"


 
 

















 










 









#line 45 "..\\incs\\kernel\\kernel-includes.h"
#line 46 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\clkmng.h"
 


































#line 1 "..\\incs\\kernel\\kernel-includes.h"
 































#line 51 "..\\incs\\kernel\\kernel-includes.h"

#line 37 "..\\incs\\kernel\\clkmng.h"
#line 1 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"







 




#line 14 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\clock_arch.h"










 
#line 13 "..\\incs\\asm-arm\\stm32f0xx\\clock_arch.h"





typedef struct 
{
  u8 status:2;
  u8 last_status:2;
}status_record;
			
typedef struct 
{
  status_record gpioa;
	status_record gpiob;
	status_record gpioc;
	status_record gpiod;
	status_record gpioe;
	status_record gpiof;
	status_record usart1;
	status_record usart2;
	status_record afio;
	status_record adc1;
	status_record adc2;
	status_record spi1;
	status_record spi2;
	status_record i2c2;
	status_record tim1;
	status_record tim2;
	status_record dma1;
} clock_status_record;
































#line 15 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 16 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\usart.h"







 




#line 14 "..\\incs\\driver\\stm32f051x\\usart.h"
#line 1 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"







 

#line 30 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
















#line 15 "..\\incs\\driver\\stm32f051x\\usart.h"



typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
	u32 pclk2 ;
	u32 bound ;
	void *data;
}usart_device;


typedef struct
{
	u16 length;
	char data[64];
}usart_data;




	
extern u8 USART_RX_BUF[64];     
extern u8 USART_RX_STA;         



os_err_t usart_register(u16 task_id);


















#line 17 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\led.h"
#line 4 "..\\incs\\driver\\stm32f051x\\led.h"
#line 5 "..\\incs\\driver\\stm32f051x\\led.h"
#line 6 "..\\incs\\driver\\stm32f051x\\led.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_gpio.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_gpio.h"



 



 
 












 
typedef enum
{
  GPIO_Mode_IN   = 0x00,  
  GPIO_Mode_OUT  = 0x01,  
  GPIO_Mode_AF   = 0x02,  
  GPIO_Mode_AN   = 0x03   
}GPIOMode_TypeDef;





 



 
typedef enum
{
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;





 



 
typedef enum
{
  GPIO_Speed_Level_1  = 0x01,  
  GPIO_Speed_Level_2  = 0x02,  
  GPIO_Speed_Level_3  = 0x03   
}GPIOSpeed_TypeDef;





 



 
typedef enum
{
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;





 



 
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;




 



 
typedef struct
{
  uint32_t GPIO_Pin;              
 
                                       
  GPIOMode_TypeDef GPIO_Mode;     
 

  GPIOSpeed_TypeDef GPIO_Speed;   
 

  GPIOOType_TypeDef GPIO_OType;   
 

  GPIOPuPd_TypeDef GPIO_PuPd;     
 
}GPIO_InitTypeDef;

 



 



 
#line 183 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_gpio.h"



#line 202 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_gpio.h"



 



 
#line 226 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_gpio.h"

#line 243 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_gpio.h"


 



 



 





 




 



 




 



 




 



 









 



 







 



 

 
 
 
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

 
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);

 
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);








 



 

 
#line 7 "..\\incs\\driver\\stm32f051x\\led.h"




















 enum led_command{led_on=0 , led_off, led_trigger};
 enum led_list{led_1=0 , led_2 };



 

 
void led_init(void);
 
os_err_t  leds_control(os_device_t* dev, u8 cmd, void *args);
 


















#line 18 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\Time3.h"




 void Time3_Init(void);





#line 19 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f103x\\ADC.h"

















#line 26 "..\\incs\\driver\\stm32f103x\\ADC.h"


#line 38 "..\\incs\\driver\\stm32f103x\\ADC.h"



void ADC1_Init(void);
void ADC_int(void);

void MYDMA_Enable(void);
#line 20 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\keys.h"







 

#line 13 "..\\incs\\driver\\stm32f051x\\keys.h"
#line 14 "..\\incs\\driver\\stm32f051x\\keys.h"
	
 
enum keys{key1=1,key2,key3};
enum keystatus{unkown=0,key_down,key_released};

typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
}key_device;


typedef struct
{
	u8  key;
	u8  status;
}key_data;



typedef struct 
{
  u8 key1_trigger:2;
	u8 key2_trigger:2;
	u8 key3_trigger:2;
	u8 key4_trigger:2;
} key_trigger;



	

os_err_t keys_register(u16 task_id);


















#line 21 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\relay.h"







 

#line 13 "..\\incs\\driver\\stm32f051x\\relay.h"
#line 14 "..\\incs\\driver\\stm32f051x\\relay.h"

 
enum relays{relay1=1,relay2,relay3,relay4};
enum relays_trigger{relay_closed=0,relay_opened=1};
typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
	u8  relay1_status:2;
	u8  relay2_status:2;
	u8  relay3_status:2;
	u8  relay4_status:2;
}relay_device;





	

os_err_t relays_register(u16 task_id);


















#line 22 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 23 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\spi.h"







 



 




 
#line 19 "..\\incs\\driver\\stm32f051x\\spi.h"
#line 20 "..\\incs\\driver\\stm32f051x\\spi.h"
#line 21 "..\\incs\\driver\\stm32f051x\\spi.h"
 

enum spimode{spi_rx_only=0 , spi_tx_only , spi_full_duplex};
enum SPICLK_DIV{SPICLK_DIV_2 , SPICLK_DIV_4 , SPICLK_DIV_8 , SPICLK_DIV_16 , SPICLK_DIV_32 , SPICLK_DIV_64 , SPICLK_DIV_128 , SPICLK_DIV_256};
 
typedef struct
{
	os_device_t os_device;
	u8 mode;
	u16 register_taskid;
	u32 pclk2 ;
	u32 prescale ;
	void *data;
}spi_device;


				
extern os_err_t spi2_register(u16 task_id);




#line 24 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\htu20d.h"







 




#line 14 "..\\incs\\driver\\stm32f051x\\htu20d.h"
#line 15 "..\\incs\\driver\\stm32f051x\\htu20d.h"

#line 23 "..\\incs\\driver\\stm32f051x\\htu20d.h"




enum htu20_command{htu20_convert_start , htu20_wakeup , htu20_sleep};

typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
	u32 clk ;
	u8 dev_adress;
	void *data;
}htu20_device;


typedef struct
{
	u16 temper;
  u16 humidity;
	u16 ampfifier_times;
	u8 user_res_content;
}htu20_data;




















#line 25 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\eeprom.h"


























  

 



 
#line 35 "..\\incs\\driver\\stm32f051x\\eeprom.h"
#line 36 "..\\incs\\driver\\stm32f051x\\eeprom.h"
#line 37 "..\\incs\\driver\\stm32f051x\\eeprom.h"
 
 


 




 






 



 


 




 



 


 


 

 
typedef struct
{
	os_device_t os_device; 
	u16 register_taskid;   
	u32 pclk2 ;            
	u32 bound ;
}eeprom_device;

 
 
os_err_t eeprom_register(u16 task_id);


 
#line 26 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\driver\\stm32f051x\\me2_ch2o.h"
#line 27 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_flash.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_flash.h"



 



 

 



  
typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;

 
  


  
  


  







  



 
   





  



 
  




 



   




 



 
  

#line 131 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_flash.h"







 



  



  


  
 






  



 







 



 







 



 







 



 







 



 








     



 








  
  


  





 






  



  




  



  

 
 
  


   
 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_PrefetchBufferCmd(FunctionalState NewState);
FlagStatus FLASH_GetPrefetchBufferStatus(void);

 
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);

 
void FLASH_OB_Unlock(void);
void FLASH_OB_Lock(void);
void FLASH_OB_Launch(void);
FLASH_Status FLASH_OB_Erase(void);
FLASH_Status FLASH_OB_EnableWRP(uint32_t OB_WRP);
FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP);
FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
FLASH_Status FLASH_OB_BOOTConfig(uint8_t OB_BOOT1);
FLASH_Status FLASH_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG);
FLASH_Status FLASH_OB_SRAMParityConfig(uint8_t OB_SRAM_Parity);
FLASH_Status FLASH_OB_WriteUser(uint8_t OB_USER);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
uint8_t FLASH_OB_GetUser(void);
uint32_t FLASH_OB_GetWRP(void);
FlagStatus FLASH_OB_GetRDP(void);

 
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);









 



  

 
#line 28 "..\\incs\\driver\\stm32f051x\\drivers-includes.h"


















#line 38 "..\\incs\\kernel\\clkmng.h"
enum clock_status{clock_closed=0,clock_opened};

typedef struct
{
		struct os_object parent;
	  u8 status;
	  u8 last_status;
	  os_err_t (*clock_enable)(void);
    os_err_t (*clock_disable)(void);

	 status_record *pstatus_record;

} Peripheral_clock;


enum {sleepmode,stopmode,standbymode};

os_err_t os_clock_register(Peripheral_clock *clock,   
                            const char  *name 
                           ); 
os_err_t os_clock_open(const char * name);
os_err_t os_clock_close(const char * name);
os_err_t os_clock_restore(const char * name);	




#line 47 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\task.h"





void System_Tick_Hook(void);









extern u8 Current_Task;



				

				




#line 48 "..\\incs\\kernel\\kernel-includes.h"
#line 49 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 













#line 38 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 129 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 948 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 50 "..\\incs\\kernel\\kernel-includes.h"


#line 7 "..\\incs\\driver\\stm32f051x\\me2_ch2o.h"
#line 8 "..\\incs\\driver\\stm32f051x\\me2_ch2o.h"
#line 9 "..\\incs\\driver\\stm32f051x\\me2_ch2o.h"



typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
 	
}adc_device;




 
typedef struct
{
	u32 para_a;   
	u32 para_b;   
	
}correction_para;



enum sensor_order
{
	get_correction_para     ,  
	set_correction_para     ,  
	update_correction_sensor,  
	sensor_open             ,  
	sensor_close            ,  
	
};

os_err_t me2_ch20_register(u16 task_id);

#line 14 "..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C"
#line 15 "..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"

























 

 



 
 
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"



 



 

 



 
  
typedef struct
{
  uint32_t ADC_Resolution;                  
 

  FunctionalState ADC_ContinuousConvMode;   

 

  uint32_t ADC_ExternalTrigConvEdge;        

 

  uint32_t ADC_ExternalTrigConv;            

 

  uint32_t ADC_DataAlign;                   
 

  uint32_t  ADC_ScanDirection;              

 
}ADC_InitTypeDef;


 



  




  







  



  












  



  











  



  

 



 


 


 









  



  
  







 



  
  







  



  
  







  
    


  
  
#line 220 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"


#line 241 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"


  
  


  

#line 257 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"

#line 266 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"


  



  
  




  



  
  
#line 303 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"









  
  


  
  
#line 324 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"
 










  



  
  
#line 347 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"









#line 362 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_adc.h"


 
  


  

 
  

 
void ADC_DeInit(ADC_TypeDef* ADCx);

  
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_JitterCmd(ADC_TypeDef* ADCx, uint32_t ADC_JitterOff, FunctionalState NewState);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_AutoPowerOffCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_WaitModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog_Channel);
void ADC_AnalogWatchdogSingleChannelCmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_TempSensorCmd(FunctionalState NewState);
void ADC_VrefintCmd(FunctionalState NewState);
void ADC_VbatCmd(FunctionalState NewState);

 
void ADC_ChannelConfig(ADC_TypeDef* ADCx, uint32_t ADC_Channel, uint32_t ADC_SampleTime);
void ADC_ContinuousModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_OverrunModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint32_t ADC_GetCalibrationFactor(ADC_TypeDef* ADCx);
void ADC_StopOfConversion(ADC_TypeDef* ADCx);
void ADC_StartOfConversion(ADC_TypeDef* ADCx);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);

 
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMARequestModeConfig(ADC_TypeDef* ADCx, uint32_t ADC_DMARequestMode);

 
void ADC_ITConfig(ADC_TypeDef* ADCx, uint32_t ADC_IT, FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint32_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint32_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint32_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint32_t ADC_IT);









  



  

 
#line 35 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_cec.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_cec.h"



 



 
 
  


 
typedef struct
{
  uint32_t CEC_SignalFreeTime;     
 
  uint32_t CEC_RxTolerance;        
 
  uint32_t CEC_StopReception;      
 
  uint32_t CEC_BitRisingError;     
 
  uint32_t CEC_LongBitPeriodError; 
 
  uint32_t CEC_BRDNoGen;           
 
  uint32_t CEC_SFTOption;          
 

}CEC_InitTypeDef;

 



 



 
#line 88 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_cec.h"

#line 97 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_cec.h"


 



 







 



 







 



 







 



 







 



 








 



 







 



 




 



 
#line 199 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_cec.h"



#line 215 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_cec.h"


 



 
#line 235 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_cec.h"



#line 251 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_cec.h"


 



 

 
 

 
void CEC_DeInit(void);

 
void CEC_Init(CEC_InitTypeDef* CEC_InitStruct);
void CEC_StructInit(CEC_InitTypeDef* CEC_InitStruct);
void CEC_Cmd(FunctionalState NewState);
void CEC_ListenModeCmd(FunctionalState NewState);
void CEC_OwnAddressConfig(uint8_t CEC_OwnAddress);
void CEC_OwnAddressClear(void);

 
void CEC_SendData(uint8_t Data);
uint8_t CEC_ReceiveData(void);
void CEC_StartOfMessage(void);
void CEC_EndOfMessage(void);

 
void CEC_ITConfig(uint16_t CEC_IT, FunctionalState NewState);
FlagStatus CEC_GetFlagStatus(uint16_t CEC_FLAG);
void CEC_ClearFlag(uint32_t CEC_FLAG);
ITStatus CEC_GetITStatus(uint16_t CEC_IT);
void CEC_ClearITPendingBit(uint16_t CEC_IT);









 



 

 
#line 36 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_crc.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_crc.h"



 



 

 
 



 












 

 
 
 
void CRC_DeInit(void);
void CRC_ResetDR(void);
void CRC_ReverseInputDataSelect(uint32_t CRC_ReverseInputData);
void CRC_ReverseOutputDataCmd(FunctionalState NewState);
void CRC_SetInitRegister(uint32_t CRC_InitValue);

 
uint32_t CRC_CalcCRC(uint32_t CRC_Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);

 
void CRC_SetIDRegister(uint8_t CRC_IDValue);
uint8_t CRC_GetIDRegister(void);









 



 

 
#line 37 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_comp.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_comp.h"



 



 

 



 
  
typedef struct
{

  uint32_t COMP_InvertingInput;     
 

  uint32_t COMP_Output;             
 

  uint32_t COMP_OutputPol;           
 

  uint32_t COMP_Hysteresis;         
 

  uint32_t COMP_Mode;               

 

}COMP_InitTypeDef;

 
   


  



 






 


  



 

#line 105 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_comp.h"

#line 113 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_comp.h"


  
  


 

#line 129 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_comp.h"


#line 139 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_comp.h"


  



 








  



 

 











 



 

 











 



  

 


 




  



  

 
 

 
void COMP_DeInit(void);

 
void COMP_Init(uint32_t COMP_Selection, COMP_InitTypeDef* COMP_InitStruct);
void COMP_StructInit(COMP_InitTypeDef* COMP_InitStruct);
void COMP_Cmd(uint32_t COMP_Selection, FunctionalState NewState);
void COMP_SwitchCmd(FunctionalState NewState);
uint32_t COMP_GetOutputLevel(uint32_t COMP_Selection);

 
void COMP_WindowCmd(FunctionalState NewState);

 
void COMP_LockConfig(uint32_t COMP_Selection);









  



 

 
#line 38 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dac.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dac.h"
 


 



 

 



 
  
typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;

 



 



 
  
#line 81 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dac.h"

#line 89 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dac.h"
                                 


 



 







 
  


 






 



 

#line 127 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dac.h"


 



 





 



  
  





  




  
  

  




  



  

 
 

 
void DAC_DeInit(void);

 
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);

 
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);

 
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState);
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG);
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG);
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT);
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT);









  



  

 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dbgmcu.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dbgmcu.h"



 



  
  
 




 





#line 68 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dbgmcu.h"









  

 
  

  
uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);

  
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB2PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);









  



  

 
#line 40 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dma.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dma.h"



 



 
 



 
typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_MemoryBaseAddr;      

  uint32_t DMA_DIR;                
 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_M2M;                
 
}DMA_InitTypeDef;

 



 









 








 



 








 



 








 



 










 



 










 



 







 



 












 



 








 



 







#line 242 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dma.h"



#line 255 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dma.h"



 



 
#line 283 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dma.h"



#line 296 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_dma.h"



 



 





 



 

 
 

 
void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);

 
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);

  
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber);
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);

 
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
FlagStatus DMA_GetFlagStatus(uint32_t DMA_FLAG);
void DMA_ClearFlag(uint32_t DMA_FLAG);
ITStatus DMA_GetITStatus(uint32_t DMA_IT);
void DMA_ClearITPendingBit(uint32_t DMA_IT);









 



 

 
#line 41 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_exti.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_exti.h"



 



 
 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 

  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
 
}EXTI_InitTypeDef;

 



 


 

#line 142 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_exti.h"



#line 156 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_exti.h"



 



 

 
 
 
void EXTI_DeInit(void);

 
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);

 
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);








 



 

 
#line 42 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 43 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 44 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_syscfg.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_syscfg.h"



 



 
 
 



  
  


  













 



  
#line 91 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_syscfg.h"

#line 108 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_syscfg.h"


 



  











 



  





  








 



  












 



  










 



 







 



 

 
 

 
void SYSCFG_DeInit(void);

  
void SYSCFG_MemoryRemapConfig(uint32_t SYSCFG_MemoryRemap);
void SYSCFG_DMAChannelRemapConfig(uint32_t SYSCFG_DMARemap, FunctionalState NewState);
void SYSCFG_I2CFastModePlusConfig(uint32_t SYSCFG_I2CFastModePlus, FunctionalState NewState);
void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);
void SYSCFG_BreakConfig(uint32_t SYSCFG_Break);
FlagStatus SYSCFG_GetFlagStatus(uint32_t SYSCFG_Flag);
void SYSCFG_ClearFlag(uint32_t SYSCFG_Flag);









  



  

 
#line 45 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"



 



 

 



 

typedef struct
{
  uint32_t I2C_Timing;              
 

  uint32_t I2C_AnalogFilter;        
 

  uint32_t I2C_DigitalFilter;       
 

  uint32_t I2C_Mode;                
 

  uint32_t I2C_OwnAddress1;         
 

  uint32_t I2C_Ack;                 
 

  uint32_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;

 




 



                                         




 








 



 




 



 










 



 








 



 








  



 




 



 








 



 







 



 




 




 





 



 

#line 219 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"

#line 228 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"



 



 





 



 

#line 258 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"

#line 270 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"


 



 

#line 285 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"





 



 

#line 311 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"



#line 322 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"



 




 

#line 345 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"


                               
#line 355 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_i2c.h"
                               



 



 





                              



                               



 



 






                              




                               



 



 

 
 


 
void I2C_DeInit(I2C_TypeDef* I2Cx);
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint32_t I2C_IT, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StopModeCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint16_t Address, uint8_t Mask);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SlaveByteControlCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SlaveAddressConfig(I2C_TypeDef* I2Cx, uint16_t Address);
void I2C_10BitAddressingModeCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

 
void I2C_AutoEndCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_ReloadCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_NumberOfBytesConfig(I2C_TypeDef* I2Cx, uint8_t Number_Bytes);
void I2C_MasterRequestConfig(I2C_TypeDef* I2Cx, uint16_t I2C_Direction);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_10BitAddressHeaderCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetAddressMatched(I2C_TypeDef* I2Cx);
uint16_t I2C_GetTransferDirection(I2C_TypeDef* I2Cx);
void I2C_TransferHandling(I2C_TypeDef* I2Cx, uint16_t Address, uint8_t Number_Bytes, uint32_t ReloadEndMode, uint32_t StartStopMode);

 
void I2C_SMBusAlertCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_ClockTimeoutCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_ExtendedClockTimeoutCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_IdleClockTimeoutCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_TimeoutAConfig(I2C_TypeDef* I2Cx, uint16_t Timeout);
void I2C_TimeoutBConfig(I2C_TypeDef* I2Cx, uint16_t Timeout);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECRequestCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);

 
uint32_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);

 
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);

 
void I2C_DMACmd(I2C_TypeDef* I2Cx, uint32_t I2C_DMAReq, FunctionalState NewState);

 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);










 



 

 
#line 46 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_iwdg.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_iwdg.h"



 



 

 
 



 



 







 



 

#line 85 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_iwdg.h"


 



 












 



 

 
 

 
void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);
void IWDG_SetWindowValue(uint16_t WindowValue);

 
void IWDG_Enable(void);

 
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);









 



 

 
#line 47 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_pwr.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_pwr.h"



 



  

 

 



  



  

#line 68 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_pwr.h"







 



 







 

 


 







 



 




 


 



 




 


 



 












 



 

 
 

 
void PWR_DeInit(void);

 
void PWR_BackupAccessCmd(FunctionalState NewState);

 
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_PVDCmd(FunctionalState NewState);

 
void PWR_WakeUpPinCmd(uint32_t PWR_WakeUpPin, FunctionalState NewState);

 
void PWR_EnterSleepMode(uint8_t PWR_SLEEPEntry);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);

 
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);









 



 

 
#line 48 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"



 



 

 

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK_Frequency;
  uint32_t ADCCLK_Frequency;
  uint32_t CECCLK_Frequency;
  uint32_t I2C1CLK_Frequency;
  uint32_t USART1CLK_Frequency;
}RCC_ClocksTypeDef;

 



 



 









  
 


 



 




  



 

#line 121 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"


 



 
#line 144 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"

#line 153 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"


 
 


 

#line 167 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"


 



 

#line 189 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"


  



 

#line 205 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"


 
  


 










 



 








 



 








 



 











 
       


 

#line 277 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"












 
  


 








 



 










 



 

#line 329 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"


 
  


 

#line 347 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"






 



 

#line 368 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"





  



 

#line 391 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"




 



 

#line 409 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"







  



 
#line 435 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"

#line 443 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rcc.h"






 



 

 
 

 
void RCC_DeInit(void);

 
void RCC_HSEConfig(uint8_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_AdjustHSI14CalibrationValue(uint8_t HSI14CalibrationValue);
void RCC_HSI14Cmd(FunctionalState NewState);
void RCC_HSI14ADCRequestCmd(FunctionalState NewState);
void RCC_LSEConfig(uint32_t RCC_LSE);
void RCC_LSEDriveConfig(uint32_t RCC_LSEDrive);
void RCC_LSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_PREDIV1Config(uint32_t RCC_PREDIV1_Div);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCOSource);

 
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLKConfig(uint32_t RCC_HCLK);
void RCC_ADCCLKConfig(uint32_t RCC_ADCCLK);
void RCC_CECCLKConfig(uint32_t RCC_CECCLK);
void RCC_I2CCLKConfig(uint32_t RCC_I2CCLK);
void RCC_USARTCLKConfig(uint32_t RCC_USARTCLK);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

 
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);

void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);

void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);

 
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);









 



  

 
#line 49 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"

























 

 
#line 80 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"

 
#line 40 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"


 



  

 



  
typedef struct
{
  uint32_t RTC_HourFormat;   
 
  
  uint32_t RTC_AsynchPrediv; 
 
  
  uint32_t RTC_SynchPrediv;  
 
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t RTC_Hours;    


 

  uint8_t RTC_Minutes;  
 
  
  uint8_t RTC_Seconds;  
 

  uint8_t RTC_H12;      
 
}RTC_TimeTypeDef; 



 
typedef struct
{
  uint8_t RTC_WeekDay; 
 
  
  uint8_t RTC_Month;   
 

  uint8_t RTC_Date;     
 
  
  uint8_t RTC_Year;     
 
}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef RTC_AlarmTime;      

  uint32_t RTC_AlarmMask;            
 

  uint32_t RTC_AlarmDateWeekDaySel;  
 
  
  uint8_t RTC_AlarmDateWeekDay;      



 
}RTC_AlarmTypeDef;

 



  




  






  



  

 


  




  




  



  







  



  






  



  




  



  
#line 209 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"



  



  
  
#line 232 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"


  




  
#line 248 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"



  




  








  




  
#line 278 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"



  



  






  



  
#line 346 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"


  



  
  




  



  






  



  


 





  



  






  




  






  



  
#line 419 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"
                                          


  



  
#line 434 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"



  



  




  



  











  



  
#line 475 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"



  



  


#line 495 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"


  



  
#line 526 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"
                                           


 

  

  
#line 542 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"







 



  
#line 560 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"





 



  







  



  






  



  




  



 

#line 613 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"


  



  






  



  
#line 642 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"

#line 650 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"



  



  
#line 664 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_rtc.h"









  



  


 
 
 
ErrorStatus RTC_DeInit(void);


 
ErrorStatus RTC_Init(RTC_InitTypeDef* RTC_InitStruct);
void RTC_StructInit(RTC_InitTypeDef* RTC_InitStruct);
void RTC_WriteProtectionCmd(FunctionalState NewState);
ErrorStatus RTC_EnterInitMode(void);
void RTC_ExitInitMode(void);
ErrorStatus RTC_WaitForSynchro(void);
ErrorStatus RTC_RefClockCmd(FunctionalState NewState);
void RTC_BypassShadowCmd(FunctionalState NewState);

 
ErrorStatus RTC_SetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_TimeStructInit(RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_GetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
uint32_t RTC_GetSubSecond(void);
ErrorStatus RTC_SetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);
void RTC_DateStructInit(RTC_DateTypeDef* RTC_DateStruct);
void RTC_GetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);

 
void RTC_SetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_AlarmStructInit(RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_GetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
ErrorStatus RTC_AlarmCmd(uint32_t RTC_Alarm, FunctionalState NewState);
void RTC_AlarmSubSecondConfig(uint32_t RTC_Alarm, uint32_t RTC_AlarmSubSecondValue, uint8_t RTC_AlarmSubSecondMask);
uint32_t RTC_GetAlarmSubSecond(uint32_t RTC_Alarm);

 
void RTC_DayLightSavingConfig(uint32_t RTC_DayLightSaving, uint32_t RTC_StoreOperation);
uint32_t RTC_GetStoreOperation(void);

 
void RTC_OutputConfig(uint32_t RTC_Output, uint32_t RTC_OutputPolarity);

 
void RTC_CalibOutputCmd(FunctionalState NewState);
void RTC_CalibOutputConfig(uint32_t RTC_CalibOutput);
ErrorStatus RTC_SmoothCalibConfig(uint32_t RTC_SmoothCalibPeriod, 
                                  uint32_t RTC_SmoothCalibPlusPulses,
                                  uint32_t RTC_SmouthCalibMinusPulsesValue);

 
void RTC_TimeStampCmd(uint32_t RTC_TimeStampEdge, FunctionalState NewState);
void RTC_GetTimeStamp(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_StampTimeStruct, RTC_DateTypeDef* RTC_StampDateStruct);
uint32_t RTC_GetTimeStampSubSecond(void);

 
void RTC_TamperTriggerConfig(uint32_t RTC_Tamper, uint32_t RTC_TamperTrigger);
void RTC_TamperCmd(uint32_t RTC_Tamper, FunctionalState NewState);
void RTC_TamperFilterConfig(uint32_t RTC_TamperFilter);
void RTC_TamperSamplingFreqConfig(uint32_t RTC_TamperSamplingFreq);
void RTC_TamperPinsPrechargeDuration(uint32_t RTC_TamperPrechargeDuration);
void RTC_TimeStampOnTamperDetectionCmd(FunctionalState NewState);
void RTC_TamperPullUpCmd(FunctionalState NewState);

 
void RTC_WriteBackupRegister(uint32_t RTC_BKP_DR, uint32_t Data);
uint32_t RTC_ReadBackupRegister(uint32_t RTC_BKP_DR);

 
void RTC_OutputTypeConfig(uint32_t RTC_OutputType);
 
 
ErrorStatus RTC_SynchroShiftConfig(uint32_t RTC_ShiftAdd1S, uint32_t RTC_ShiftSubFS);

 
void RTC_ITConfig(uint32_t RTC_IT, FunctionalState NewState);
FlagStatus RTC_GetFlagStatus(uint32_t RTC_FLAG);
void RTC_ClearFlag(uint32_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint32_t RTC_IT);
void RTC_ClearITPendingBit(uint32_t RTC_IT);









  



  

 
#line 50 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"
#line 40 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 



 

 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 
  
  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;




 

typedef struct
{
  uint16_t I2S_Mode;         
 

  uint16_t I2S_Standard;     
 

  uint16_t I2S_DataFormat;   
 

  uint16_t I2S_MCLKOutput;   
 

  uint32_t I2S_AudioFreq;    
 

  uint16_t I2S_CPOL;         
 
}I2S_InitTypeDef;

 



 








 
  
#line 136 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 



 







 



 

#line 182 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 



 







 



 







 



 







 



 







 



 

#line 254 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 



 







 
  


 

#line 282 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 



 

#line 300 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 



 

#line 316 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 



 







 



 

#line 346 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"






 



 







 



 







 



 






 



 

#line 401 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 


 







 



 






 



 







 



 



















 




  








  



  







  




 

#line 503 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"



#line 512 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_spi.h"


 



 




 



 

 
 

 
void SPI_I2S_DeInit(SPI_TypeDef* SPIx);
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_TIModeCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_NSSPulseModeCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_RxFIFOThresholdConfig(SPI_TypeDef* SPIx, uint16_t SPI_RxFIFOThreshold);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);

 
void SPI_SendData8(SPI_TypeDef* SPIx, uint8_t Data);
void SPI_I2S_SendData16(SPI_TypeDef* SPIx, uint16_t Data);
uint8_t SPI_ReceiveData8(SPI_TypeDef* SPIx);
uint16_t SPI_I2S_ReceiveData16(SPI_TypeDef* SPIx);

 
void SPI_CRCLengthConfig(SPI_TypeDef* SPIx, uint16_t SPI_CRCLength);
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);

 
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);
void SPI_LastDMATransferCmd(SPI_TypeDef* SPIx, uint16_t SPI_LastDMATransfer);

 
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
uint16_t SPI_GetTransmissionFIFOStatus(SPI_TypeDef* SPIx);
uint16_t SPI_GetReceptionFIFOStatus(SPI_TypeDef* SPIx);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);









 



 

 
#line 51 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"
#line 40 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


 



  

 




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint32_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef;       



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint32_t TIM_Pulse;         

 

  uint16_t TIM_OCPolarity;    
 

  uint16_t TIM_OCNPolarity;   

 

  uint16_t TIM_OCIdleState;   

 

  uint16_t TIM_OCNIdleState;  

 
} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;




 

typedef struct
{

  uint16_t TIM_OSSRState;        
 

  uint16_t TIM_OSSIState;        
 

  uint16_t TIM_LOCKLevel;        
  

  uint16_t TIM_DeadTime;         

 

  uint16_t TIM_Break;            
 

  uint16_t TIM_BreakPolarity;    
 

  uint16_t TIM_AutomaticOutput;  
 
} TIM_BDTRInitTypeDef;



 

 

  


 

#line 189 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"

 


 





 




 
#line 212 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"

 
#line 220 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"

 





 





                                      
 





 






 
#line 255 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"

 

                                     



  



 

#line 288 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


 



 







  



 






#line 322 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"



  



 

#line 337 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


 



 

#line 355 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 479 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 







 



 







  



 







  



 







  



 

#line 541 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 

#line 557 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 

#line 573 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 

#line 590 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"

#line 599 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 

#line 647 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  




 

#line 692 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 

#line 708 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"



  



 

#line 725 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 

#line 753 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 







  



  






 



 







  



 







  



 

#line 814 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  




 

#line 832 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"



  



 

#line 847 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 908 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 

#line 924 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


  



 







  
  


 

#line 968 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"
                               
                               



  




 




  



 




 



 







 


 











 



 

#line 1043 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_tim.h"


 



 
  
 
  

 
void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint32_t Autoreload);
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint32_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint32_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint32_t Compare4);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_SelectOCREFClear(TIM_TypeDef* TIMx, uint16_t TIM_OCReferenceClear);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
uint32_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture4(TIM_TypeDef* TIMx);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);

 
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);


 
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);

                    
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_RemapConfig(TIM_TypeDef* TIMx, uint16_t TIM_Remap);










  



 

 
#line 52 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"
#line 40 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"


 



  

 

   
   


  

typedef struct
{
  uint32_t USART_BaudRate;            


 

  uint32_t USART_WordLength;          
 

  uint32_t USART_StopBits;            
 

  uint32_t USART_Parity;              




 
 
  uint32_t USART_Mode;                
 

  uint32_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  

typedef struct
{
  uint32_t USART_Clock;             
 

  uint32_t USART_CPOL;              
 

  uint32_t USART_CPHA;              
 

  uint32_t USART_LastBit;           

 
} USART_ClockInitTypeDef;

 



  








  







  



  

#line 137 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"


  



  

#line 151 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"


  



  







  



  

#line 180 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"


  



  
  






  



 
  






  



 







 



 







 
  


 








  



 







  



 







 



  







  



  

#line 291 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"


  



 
  







 



 







  



 







  



 








  



 







  



 







  


 














  



 
#line 423 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"

#line 430 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"


  









 

#line 459 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"

#line 466 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"

#line 474 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"

#line 481 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_usart.h"


 



 









  



  

 
 

 
void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_DirectionModeCmd(USART_TypeDef* USARTx, uint32_t USART_DirectionMode, FunctionalState NewState);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_MSBFirstCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_DataInvCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_InvPinCmd(USART_TypeDef* USARTx, uint32_t USART_InvPin, FunctionalState NewState);
void USART_SWAPPinCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ReceiverTimeOutCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetReceiverTimeOut(USART_TypeDef* USARTx, uint32_t USART_ReceiverTimeOut);

 
void USART_STOPModeCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_StopModeWakeUpSourceConfig(USART_TypeDef* USARTx, uint32_t USART_WakeUpSource);

 
void USART_AutoBaudRateCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_AutoBaudRateConfig(USART_TypeDef* USARTx, uint32_t USART_AutoBaudRate);
void USART_AutoBaudRateNewRequest(USART_TypeDef* USARTx);

 
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

 
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_MuteModeWakeUpConfig(USART_TypeDef* USARTx, uint32_t USART_WakeUp);
void USART_MuteModeCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_AddressDetectionConfig(USART_TypeDef* USARTx, uint32_t USART_AddressLength);
 
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint32_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
void USART_SetAutoRetryCount(USART_TypeDef* USARTx, uint8_t USART_AutoCount);
void USART_SetBlockLength(USART_TypeDef* USARTx, uint8_t USART_BlockLength);

 
void USART_IrDAConfig(USART_TypeDef* USARTx, uint32_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_DECmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_DEPolarityConfig(USART_TypeDef* USARTx, uint32_t USART_DEPolarity);
void USART_SetDEAssertionTime(USART_TypeDef* USARTx, uint32_t USART_DEAssertionTime);
void USART_SetDEDeassertionTime(USART_TypeDef* USARTx, uint32_t USART_DEDeassertionTime);

 
void USART_DMACmd(USART_TypeDef* USARTx, uint32_t USART_DMAReq, FunctionalState NewState);
void USART_DMAReceptionErrorConfig(USART_TypeDef* USARTx, uint32_t USART_DMAOnError);

 
void USART_ITConfig(USART_TypeDef* USARTx, uint32_t USART_IT, FunctionalState NewState);
void USART_RequestCmd(USART_TypeDef* USARTx, uint32_t USART_Request, FunctionalState NewState);
void USART_OverrunDetectionConfig(USART_TypeDef* USARTx, uint32_t USART_OVRDetection);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint32_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint32_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint32_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint32_t USART_IT);









  



  

 
#line 53 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_wwdg.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_wwdg.h"
#line 40 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_wwdg.h"


 



  
 
 



  
  


  
  
#line 68 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_wwdg.h"



  



  

 
 
   
void WWDG_DeInit(void);

 
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);

 
void WWDG_Enable(uint8_t Counter);

 
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  

 
#line 54 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_misc.h"


























 

 







 
#line 39 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_misc.h"
#line 40 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_misc.h"


 



 

 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;             


 

  uint8_t NVIC_IRQChannelPriority;     

 

  FunctionalState NVIC_IRQChannelCmd;  

    
} NVIC_InitTypeDef;






 

 



 



 

#line 93 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xx_misc.h"


 



 




 



 







 



 

 
  

void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 

 
#line 55 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"

 
 

 
 

 
#line 78 "..\\incs\\asm-arm\\stm32f0xx\\stm32f0xxlib_conf.h"



 
#line 16 "..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C"
#line 17 "..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C"
#line 18 "..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C"
#line 19 "..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C"


 



















 





 

 





 





 




 

static  adc_device *adc_dev = 0;

static  correction_para correct_coefficient = {0};


 
static u16  RegularConvData_Tab[20];


 



 
static os_err_t  	me2_ch20_init   (os_device_t* dev);

static os_err_t  me2_ch20_control(os_device_t* dev, u8 cmd, void *args);

os_size_t me2_ch20_read   (os_device_t* dev, os_off_t pos, void *buffer, os_size_t size);


static os_err_t adc1_dma_init(void);

static os_err_t me2_ch20_ctlpin_init(void);

static os_err_t set_me2_ch20_state(u8 operation);

static u16 adc_avr_cal(void);










 



os_err_t me2_ch20_register(u16 task_id)
{
    adc_dev = (adc_device*)osmalloc(sizeof(adc_device));
    
    
    adc_dev->os_device.type= OS_Device_Class_Char;
    
    adc_dev->os_device.device_id=OS_DEVICE_KEY_ID;
    
    adc_dev->register_taskid = task_id;
    
    adc_dev->os_device.init = me2_ch20_init;
    
    adc_dev->os_device.open = 0;
    
    adc_dev->os_device.write   = 0;
    adc_dev->os_device.read    = me2_ch20_read;
    adc_dev->os_device.control = me2_ch20_control;
    
    return os_device_register(&(adc_dev->os_device), "me2_ch2o", OS_DEVICE_FLAG_INACTIVATED);
    
}









 	

static os_err_t  	me2_ch20_init   (os_device_t* dev)
{
    
    os_clock_open("GPIOA"); 
    os_clock_open("GPIOB");
	
     
    adc1_dma_init();
    me2_ch20_ctlpin_init();
	
	   
    correct_coefficient.para_a = 0;
	  correct_coefficient.para_b = 0;
	
    return SUCCESS;
    
    
}













 

static os_err_t  me2_ch20_control(os_device_t* dev, u8 cmd, void *args)
{
    
    if(dev==0)
    {
        return ERROR;
    }
    switch(cmd)
    {
    case get_correction_para:   
         
        ((u8*)args)[0] = sizeof(correction_para);
         
        os_memmove((u8*)args+1 , &correct_coefficient , ((u8*)args)[0] );
        break;
    case set_correction_para:  
    {
        u8 data_num = ((u8*)args)[0];
        correction_para temp; 
        os_memmove(&temp , ((u8*)args)+1) , data_num);
        if(data_num != sizeof(correction_para))
        {
            return ERROR;
        }
        
         
        if(temp->para_a ==correct_coefficient.para_a &&  temp->para_b ==correct_coefficient.para_b) 
        {
            return SUCCESS;
        }else  
        {
            
            
            os_device_t *dev = os_device_get("EEPROM"); 
            
            correct_coefficient = *temp;
             
            if(dev == 0)
            {

                printf("EEPROM open failed[%s]:%d","..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C", 216 );

                return ERROR;
            }
            
             
            if(os_device_write(dev, 0X02 ,&correct_coefficient , data_num) < data_num)
            {
              

                printf("sensor para write failed[%s]:%d","..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C", 226 );

							 return ERROR;
            }
        }
        
        
    }
        break;
		case update_correction_sensor:
		{
         os_device_t *dev = os_device_get("EEPROM"); 
		         
            if(dev == 0)
            {

                printf("EEPROM open failed[%s]:%d","..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C", 242 );

                return ERROR;
            }
						
		         
            if(os_device_read(dev, 0X02 ,&correct_coefficient , sizeof(correction_para)) < sizeof(correction_para))
						{

                printf("sensor para read failed[%s]:%d","..\\drivers\\stm32f051x\\misc\\me2_ch2o\\me2_ch2o.C", 251 );

							 return ERROR;  
						}
		}  		
		   break;
    case sensor_close :         break;
    case sensor_open :          break;			
    default:break;
        
    }
    return SUCCESS;
}











 

os_size_t me2_ch20_read   (os_device_t* dev, os_off_t pos, void *buffer, os_size_t size)
{
    u16 i=0;
    if(dev==0)
    {
        return ERROR;
    }
    
    while(size >0)
    {
        
        ((u16*)buffer)[i] = adc_avr_cal();
        i++;
        size--;
    }
    
    return  i;	
    
}







 

static os_err_t adc1_dma_init(void)
{
    ADC_InitTypeDef     ADC_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;
    DMA_InitTypeDef     DMA_InitStructure;
    u16 count_time=0;
     
    ADC_DeInit(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)));
    
     
    RCC_AHBPeriphClockCmd(((uint32_t)0x00040000), ENABLE);
    
     
    RCC_APB2PeriphClockCmd(((uint32_t)0x00000200), ENABLE);
    
     
    RCC_AHBPeriphClockCmd(((uint32_t)0x00000001) , ENABLE);
    
     
    GPIO_InitStructure.GPIO_Pin = (1<<0) | (1<<1);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)), &GPIO_InitStructure);
    
     
    DMA_DeInit(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x00000000) + 0x00000008)));
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(0x40012440);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RegularConvData_Tab;
    DMA_InitStructure.DMA_DIR = ((uint32_t)0x00000000);
    DMA_InitStructure.DMA_BufferSize = 20;
    DMA_InitStructure.DMA_PeripheralInc = ((uint32_t)0x00000000);
    DMA_InitStructure.DMA_MemoryInc = ((uint32_t)0x00000080);
    DMA_InitStructure.DMA_PeripheralDataSize = ((uint32_t)0x00000100);
    DMA_InitStructure.DMA_MemoryDataSize = ((uint32_t)0x00000400);
    DMA_InitStructure.DMA_Mode = ((uint32_t)0x00000020);
    DMA_InitStructure.DMA_Priority = ((uint32_t)0x00002000);
    DMA_InitStructure.DMA_M2M = ((uint32_t)0x00000000);
    DMA_Init(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x00000000) + 0x00000008)), &DMA_InitStructure);
     
    DMA_Cmd(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x00000000) + 0x00000008)), ENABLE);
    
     
    ADC_DMARequestModeConfig(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)), ((uint32_t)0x00000002));
    
     
    ADC_DMACmd(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)), ENABLE);
    
     
    ADC_StructInit(&ADC_InitStructure);
    
     
    ADC_InitStructure.ADC_Resolution = ((uint32_t)0x00000000);
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ((uint32_t)0x00000000);
    ADC_InitStructure.ADC_DataAlign = ((uint32_t)0x00000000);
    ADC_InitStructure.ADC_ScanDirection = ((uint32_t)0x00000004);
    ADC_Init(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)), &ADC_InitStructure);
    
     
    ADC_ChannelConfig(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)), ((uint32_t)0x00000100) , ((uint32_t)0x00000005));
    
    
     
    ADC_ChannelConfig(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)), ((uint32_t)0x00000200) , ((uint32_t)0x00000005));
    
    
     
    ADC_GetCalibrationFactor(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)));
    
     
    ADC_Cmd(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)), ENABLE);
    
     
    while(!ADC_GetFlagStatus(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)), ((uint32_t)0x01000001))&&count_time<2000)
    {
        
    }
    ADC_StartOfConversion(((ADC_TypeDef *) (((uint32_t)0x40000000) + 0x00012400)));
     
    if(count_time<2000)
    {
        return  SUCCESS;
    }
    
    return ERROR;
    
}










 
static os_err_t set_me2_ch20_state(u8 operation)
{
     	
    if(operation == sensor_close)
    {
        ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) -> BRR = (1<<2);
    }
    else if(operation == sensor_open )
    {
        ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) -> BSRR = (1<<2);
    }
    return SUCCESS;
}








 

static os_err_t me2_ch20_ctlpin_init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    
     
    GPIO_InitStructure.GPIO_Pin   =  (1<<2);
    GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  =  GPIO_PuPd_UP ;
    GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_Level_2 ;
    GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)), &GPIO_InitStructure);
    set_me2_ch20_state(sensor_open);
    
    return SUCCESS;
    
}









 
u16 sensor_get_voltage(void);
static u16 adc_avr_cal()
{
   
     u16  HCHOConvertedValue = 0;
	
		   
		if(correct_coefficient.para_a == 0 && correct_coefficient.para_b ==0)
		{
		 		
			  if(me2_ch20_control( &(adc_dev->os_device) , update_correction_sensor , 0 ) == ERROR)
				{
					
					correct_coefficient.para_a = 0;
					correct_coefficient.para_b = 190;

            printf("me2ch2o update failed!");  

        }
		}
		
	     
    HCHOConvertedValue = ((sensor_get_voltage() -1235)*correct_coefficient.para_b);
		
		 
    if(HCHOConvertedValue < correct_coefficient.para_a)
    {
        return 0;
    }
    else   
    {  
        return ((HCHOConvertedValue*10 - correct_coefficient.para_a)/100 );
    }
  
    
}








 

u16 sensor_get_voltage()
{
  
	  u16  ADC1RefConvertedValue = 0;
    u16  HCHOConvertedValue = 0;
   
    u8   index;
    u32  ADC1RefConvertedSum = 0, HCHOConvertedSum = 0;
    u32  ADC1RefConvertedValueMin = 0xfffe, ADC1RefConvertedValueMax = 0;
    u32  HCHOConvertedValueMin = 0xfffe, HCHOConvertedVoltageMax = 0;
	
      
    while((DMA_GetFlagStatus(((uint32_t)0x00000002))) == RESET );
    
     
    for(index = 0; index < 20; ++index)
    { 
        if(index & 1)
        {
            ADC1RefConvertedSum += RegularConvData_Tab[index];
            if(RegularConvData_Tab[index] > ADC1RefConvertedValueMax)
            {
                ADC1RefConvertedValueMax = RegularConvData_Tab[index];
            }
            if(RegularConvData_Tab[index] < ADC1RefConvertedValueMin)
            {
                ADC1RefConvertedValueMin = RegularConvData_Tab[index];
            }
        }
        else
        {
            HCHOConvertedSum +=  RegularConvData_Tab[index];
            if(RegularConvData_Tab[index] > HCHOConvertedVoltageMax)
            {
                HCHOConvertedVoltageMax = RegularConvData_Tab[index];
            }
            if(RegularConvData_Tab[index] < HCHOConvertedValueMin)
            {
                HCHOConvertedValueMin = RegularConvData_Tab[index];
            }
        }
    }
    
     
    HCHOConvertedSum = HCHOConvertedSum - HCHOConvertedVoltageMax - HCHOConvertedValueMin;
		
	   
    ADC1RefConvertedSum = ADC1RefConvertedSum - ADC1RefConvertedValueMax - ADC1RefConvertedValueMin;
    
		 
    ADC1RefConvertedValue = ADC1RefConvertedSum / ((20 >> 1) - 2);
		
		 
    HCHOConvertedValue    = HCHOConvertedSum / ((20 >> 1) - 2);
    
    
     
    DMA_ClearFlag(((uint32_t)0x00000002));
    
		
		 
    if(((HCHOConvertedValue * 2520 / ADC1RefConvertedValue)<1235))
    {
        return 1235;
    }
    
     
    return ((HCHOConvertedValue * 2520 / ADC1RefConvertedValue));
}

