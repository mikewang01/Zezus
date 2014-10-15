#line 1 "..\\arch\\arm\\stm32f05x\\kernel\\corefunc\\system_stm32f0xx.c"















































































 



 



   
  


 

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







 




 

 

  







 


 









 

  

 

 
#line 95 "..\\arch\\arm\\stm32f05x\\kernel\\corefunc\\system_stm32f0xx.c"
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\config.h"







 



 








typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned  int  u32;



























 




 






#line 73 "..\\incs\\asm-arm\\stm32f0xx\\config.h"
 

















#line 96 "..\\arch\\arm\\stm32f05x\\kernel\\corefunc\\system_stm32f0xx.c"


 



 



 



 


 



 



 



 
uint32_t SystemCoreClock    = 48*1000000;
volatile const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};



 



 

static void SetSysClock(void);



 



 







 
void SystemInit (void)
{    
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR |= (uint32_t)0x00000001;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR &= (uint32_t)0xF8FFB80C;
  
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR &= (uint32_t)0xFEF6FFFF;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR &= (uint32_t)0xFFFBFFFF;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR &= (uint32_t)0xFFC0FFFF;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR2 &= (uint32_t)0xFFFFFFF0;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR3 &= (uint32_t)0xFFFFFEAC;
  
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR2 &= (uint32_t)0xFFFFFFFE;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CIR = 0x00000000;
  
 
  SetSysClock();
}



























 
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, prediv1factor = 0;

   
  tmp = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR & ((uint32_t)0x0000000C);
  
  switch (tmp)
  {
    case 0x00:   
      SystemCoreClock = ((uint32_t)8000000);
      break;
    case 0x04:   
      SystemCoreClock = ((uint32_t)8000000);
      break;
    case 0x08:   
       
      pllmull = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR & ((uint32_t)0x003C0000);
      pllsource = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR & ((uint32_t)0x00010000);
      pllmull = ( pllmull >> 18) + 2;
      
      if (pllsource == 0x00)
      {
         
        SystemCoreClock = (((uint32_t)8000000) >> 1) * pllmull;
      }
      else
      {
        prediv1factor = (((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR2 & ((uint32_t)0x0000000F)) + 1;
         
        SystemCoreClock = (((uint32_t)8000000) / prediv1factor) * pllmull; 
      }      
      break;
    default:  
      SystemCoreClock = ((uint32_t)8000000);
      break;
  }
   
   
  tmp = AHBPrescTable[((((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR & ((uint32_t)0x000000F0)) >> 4)];
   
  SystemCoreClock >>= tmp;  
}








 
static void SetSysClock(void)
{
  volatile uint32_t StartUpCounter = 0, HSEStatus = 0;
  
   
       
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR |= ((uint32_t)((uint32_t)0x00010000));
 
   
  do
  {
    HSEStatus = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR & ((uint32_t)0x00020000);
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != ((uint16_t)0x0500)));

  if ((((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR & ((uint32_t)0x00020000)) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
     
    ((FLASH_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00002000))->ACR |= ((uint32_t)0x00000010);
    ((FLASH_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00002000))->ACR |= (uint32_t)((uint32_t)0x00000001);
 
     
    ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR |= (uint32_t)((uint32_t)0x00000000);
      
     
    ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR |= (uint32_t)((uint32_t)0x00000000);

     
    ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR &= (uint32_t)((uint32_t)~(((uint32_t)0x00010000) | ((uint32_t)0x00020000) | ((uint32_t)0x003C0000)));
    ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR |= (uint32_t)(((uint32_t)0x00010000) | ((uint32_t)0x00000000) | ((uint32_t)0x00100000));
            
     
    ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR |= ((uint32_t)0x01000000);

     
    while((((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CR & ((uint32_t)0x02000000)) == 0)
    {
    }

     
    ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR &= (uint32_t)((uint32_t)~(((uint32_t)0x00000003)));
    ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR |= (uint32_t)((uint32_t)0x00000002);    

     
    while ((((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x00020000) + 0x00001000))->CFGR & (uint32_t)((uint32_t)0x0000000C)) != (uint32_t)0x08)
    {
    }
  }
  else
  { 
 
  }  
}





 



 



 

 
