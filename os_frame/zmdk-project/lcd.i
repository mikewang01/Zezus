#line 1 "..\\drivers\\stm32f051x\\misc\\lcd\\lcd.c"







 



 

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



 

#line 15 "..\\drivers\\stm32f051x\\misc\\lcd\\lcd.c"
#line 1 "..\\incs\\driver\\stm32f051x\\spi.h"







 



 




 
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






#line 19 "..\\incs\\driver\\stm32f051x\\spi.h"
#line 1 "..\\incs\\kernel\\kernel-includes.h"
 



































#line 38 "..\\incs\\kernel\\kernel-includes.h"
#line 1 "..\\incs\\kernel\\device.h"







 





 

#line 17 "..\\incs\\kernel\\device.h"
#line 1 "..\\incs\\kernel\\obj.h"







 







 
 
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










 
#line 1 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"
#line 8 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"



#line 12 "..\\incs\\asm-arm\\stm32f0xx\\corefunc.h"



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







 



 


#line 42 "..\\incs\\driver\\stm32f051x\\spi.h"

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




#line 6 "..\\incs\\driver\\stm32f051x\\me2_ch2o.h"
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
#line 50 "..\\incs\\kernel\\kernel-includes.h"


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




#line 16 "..\\drivers\\stm32f051x\\misc\\lcd\\lcd.c"

#line 18 "..\\drivers\\stm32f051x\\misc\\lcd\\lcd.c"
#line 19 "..\\drivers\\stm32f051x\\misc\\lcd\\lcd.c"
#line 1 "..\\incs\\driver\\stm32f051x\\lcd.h"







 



 




 
#line 19 "..\\incs\\driver\\stm32f051x\\lcd.h"
#line 20 "..\\incs\\driver\\stm32f051x\\lcd.h"
#line 21 "..\\incs\\driver\\stm32f051x\\lcd.h"
 






















enum com_data{lcd_command=0, lcd_x_inv , lcd_y_inv , lcd_clear , lcd_pwrdown , lcd_wakeup , lcd_refresh};

enum lcd_status{lcd_normal, lcd_reset , lcd_sleep};

 
enum lcd_pixel{pixel_fill , pixel_clear};


typedef struct
{
		u16 x_cord;
	  u16 y_cord;
}curor_pos;

typedef struct
{
		u16 x_pixel;
	  u16 y_pixel;
}solution;

typedef struct
{
		u8 x_direction;
	  u8 y_direction;
}scan_mode;

 

typedef struct
{
  u8 font_type;
	u8 (*font_ptr)[16];
	u8 font_x_model;
	u8 font_y_model;
} font_property;



typedef struct
{
	os_device_t os_device;
	u16 register_taskid;
	solution lcd_solution;
	scan_mode lcd_scan_mode;
	font_property font;
  u8 current_status;
  curor_pos  cursor_record;	
	u8 (*buffer)[64>>3];
}lcd_device;



 
#line 113 "..\\incs\\driver\\stm32f051x\\lcd.h"











 



 







#line 145 "..\\incs\\driver\\stm32f051x\\lcd.h"

 





 






 
 






extern os_err_t lcd_register(u16 task_id);




#line 20 "..\\drivers\\stm32f051x\\misc\\lcd\\lcd.c"


 









 

enum font_type{font6x8_t , font16x8_t};



 




 
static lcd_device  *lcd_dev=0;
static os_device_t *bus_dev=0;









static const  unsigned char font16x8[][16];	






 










static u8 lcd_dram_buffer[128][64>>3]={0};



 
static  os_err_t   transfer_data       (u8 data);

static  os_err_t   transfer_command    (u8 cmd); 

static  void       clear_screen        (u8 data); 


static  os_err_t   lcd_init            (os_device_t* dev);

static  os_size_t  lcd_write           (os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);

static os_err_t    lcd_control         (os_device_t* dev, u8 cmd, void *args);

static  os_err_t   lcd_open            (os_device_t* dev, u16 oflag);

static os_err_t    lcd_showstring      (u8 x,u8 y ,  const u8 *p , u16 size_1);

static void       lcd_refresh_buffer   (void);









 

os_err_t lcd_register(u16 task_id)
{
    
    lcd_dev=osmalloc(sizeof(lcd_device));
    
    
    lcd_dev->os_device.type = OS_Device_Class_MTD;
    
    lcd_dev->os_device.device_id = OS_DEVICE_USART_ID;
    
    lcd_dev->register_taskid = task_id;
    
    lcd_dev->lcd_solution.x_pixel=128;
    
    lcd_dev->lcd_solution.y_pixel=64;
    
    lcd_dev->lcd_scan_mode.x_direction=0X00;
    
    lcd_dev->lcd_scan_mode.y_direction=0X00;
    
    lcd_dev->cursor_record.x_cord=0;
    lcd_dev->cursor_record.y_cord=0;
    
         
#line 139 "..\\drivers\\stm32f051x\\misc\\lcd\\lcd.c"
    

    
    lcd_dev->font.font_type = font16x8_t;
    lcd_dev->font.font_ptr  = (u8 (*)[16])font16x8;
    lcd_dev->font.font_x_model = 8;
    lcd_dev->font.font_y_model = 16;

    

    
    
       

    lcd_dev->buffer = lcd_dram_buffer;



    
    
    lcd_dev->current_status = lcd_reset;
    
    lcd_dev->os_device.init  = lcd_init;
    
    lcd_dev->os_device.open  = lcd_open;
    
    lcd_dev->os_device.write = lcd_write;
    
    lcd_dev->os_device.control = lcd_control;
    
    
    return os_device_register(&(lcd_dev->os_device), "LCD", OS_DEVICE_FLAG_INACTIVATED);
    
}










 
static os_err_t  lcd_init   (os_device_t* dev)
{  	 
    

    os_clock_open("GPIOB");
    
    
    
     
    gpio_speed_set(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) , 12 ,  GPIO_Speed_Level_3 );
    
    gpio_speed_set(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) , 14  , GPIO_Speed_Level_3 );
    
    
     
    gpio_outtype_set(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) , 12 , GPIO_OType_PP);
    
    gpio_outtype_set(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) , 14 , GPIO_OType_PP);
    
    
    gpio_outmode_set(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) , 12 , GPIO_Mode_OUT);
    
    gpio_outmode_set(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) , 14 ,  GPIO_Mode_OUT);
    
     
    gpio_pupdr_set(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) , 12 , GPIO_PuPd_UP);
    
    gpio_pupdr_set(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400)) , 14 , GPIO_PuPd_UP);
    
    bus_dev= os_device_get("SPI2");
    
    if(bus_dev==0)
    {
        transfer_command(0xe2);     
    }
    
    do{transfer_command(0XE2);}while(0);
    
    do{ transfer_command(0x2c); transfer_command(0x2e); transfer_command(0x2f); transfer_command(0x23); transfer_command(0x81); transfer_command(0x28); transfer_command(0xa2); }while(0);
    
    do{transfer_command(0Xc0|lcd_dev->lcd_scan_mode . y_direction);transfer_command(0XA0|lcd_dev->lcd_scan_mode . x_direction);}while(0);
    
    do{transfer_command(0XAE|0x01);}while(0);
    
    lcd_dev->current_status = lcd_normal ;
    
    
    
    
    
    
    
    return SUCCESS;
    
    
}











 
static os_err_t  lcd_open(os_device_t* dev, u16 oflag)
{
    os_clock_open("SPI2");
    os_clock_open("GPIOB");
    
    return SUCCESS;	
}










 
static os_err_t  lcd_close(os_device_t* dev, u16 oflag)
{
    
    os_clock_close("SPI2");
    os_clock_close("GPIOB");
    
    return SUCCESS;		
    
}










 

static os_size_t  lcd_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
{
    
    if(pos>=size||buffer==0||size>64)
    {
        return  0;
    }else
    {
        lcd_showstring((pos/128)*8 , pos%128   , buffer , size );
        lcd_refresh_buffer();
    }
    
    return SUCCESS;
}	












 

static os_err_t  lcd_control(os_device_t* dev, u8 cmd, void *args)
{
    u8 clear_data=((u8*)args)[1];
    
    if(dev==0)
    {
        return ERROR;
    }
    switch(cmd)
    {
    case lcd_command:  	  break;
        
    case lcd_x_inv: 	          
        if(lcd_dev->lcd_scan_mode.x_direction == 0X00 )
        {
            lcd_dev->lcd_scan_mode.x_direction = 0X01;
        }else
        {
            lcd_dev->lcd_scan_mode.x_direction = 0X00;
        }
        do{transfer_command(0Xc0|lcd_dev->lcd_scan_mode . y_direction);transfer_command(0XA0|lcd_dev->lcd_scan_mode . x_direction);}while(0);				
        break;
        
    case lcd_y_inv:                
        if(lcd_dev->lcd_scan_mode.y_direction == 0X08 )
        {
            lcd_dev->lcd_scan_mode.y_direction = 0X00 ;
        }else
        {
            lcd_dev->lcd_scan_mode.y_direction = 0X08;
        }
        do{transfer_command(0Xc0|lcd_dev->lcd_scan_mode . y_direction);transfer_command(0XA0|lcd_dev->lcd_scan_mode . x_direction);}while(0);				
        break; 
        
    case lcd_clear:      clear_screen(clear_data); break;
        
    case lcd_pwrdown:     break;
        
    case lcd_wakeup:     break;
         															
    case lcd_refresh:  lcd_refresh_buffer(); break;
        
    default:break;
        
    }
    return SUCCESS;
}











 


static os_err_t transfer_command(u8 cmd) 
{ 
    u8 status;
     	
    ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400))->BRR=(1<<12);
    
     	
    ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400))->BRR=(1<<14);
    
    
    status=os_device_write(bus_dev, 0, &cmd, 1);   
    
    
    return status;
} 












 
static os_err_t transfer_data(u8 data) 
{ 
    u8 status;
    
    ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400))->BRR=(1<<12);
    
    ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400))->BSRR=(1<<14);
    
    status=os_device_write(bus_dev, 0, &data, 1);  
    
    
    return status;
} 











 

static void clear_screen(u8 data) 
{
    unsigned char i,n;
    ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x08000000) + 0x00000400))->BRR=(1<<12);	
    for(i=0 ; i<((lcd_dev->lcd_solution.y_pixel)>>3) ; i++)  
    {  
         
        do{ transfer_command(0XB0 + (i)); transfer_command(0x10 | ((0>>4)&0x0f)); transfer_command(0x00 | (0&0x0f)); }while(0);
        
        for(n=0 ; n<lcd_dev->lcd_solution.x_pixel ; n++)
            
            lcd_dev->buffer[n][i]=data; 
        transfer_data(lcd_dev->buffer[n][i]);
        
        
    }   
    
}











 



static void lcd_refresh_buffer(void)
{
    u16 i,n;		    
    for(i=0 ; i< ((lcd_dev->lcd_solution.y_pixel)>>3) ; i++)  
    {  
         
        do{ transfer_command(0XB0 + (i)); transfer_command(0x10 | ((0>>4)&0x0f)); transfer_command(0x00 | (0&0x0f)); }while(0);
        for(n=0 ; n<lcd_dev->lcd_solution.x_pixel ; n++)
            
            transfer_data(lcd_dev->buffer[n][i]);
        
    }   
}












 


os_err_t lcd_drawpoint(u16 x,u16 y,u8 pixel)
{
    u8 pos,bx,temp=0;
    
    if(x>=(lcd_dev->lcd_solution.x_pixel)||y>=(lcd_dev->lcd_solution.y_pixel))
        return ERROR;
    
    pos=7-y/8;
    bx=y%8;
    temp=1<<(7-bx);
    
    
     	
    if(lcd_dev->buffer!=0)
    {	 if(pixel == pixel_fill)
            lcd_dev->buffer[x][pos] |= temp;	
        else
            lcd_dev->buffer[x][pos] &= ~temp;
    }	
    return SUCCESS;	
}









 

os_err_t lcd_showchar(u8 x,u8 y,u16 chr )
{      			    
    u8 temp,t,t1;
    u8 y0=y;
    chr=chr-' ';
    for(t=0 ; t<(lcd_dev->font.font_y_model) ;  t++)
    {   
        
        temp =	lcd_dev->font.font_ptr[chr][t];	
        for(t1=0; t1<8 ; t1++)
        {
            if(temp&0x80)lcd_drawpoint(x,y,pixel_fill);
            temp<<=1;
            y++;
            if((y-y0)==lcd_dev->font.font_y_model)
            {
                y=y0;
                x++;
                break;
            }
        }  	 
    }  
    
    return SUCCESS;		
}









 























 
static os_err_t lcd_showstring(u8 x,u8 y ,  const u8 *p , u16 size_1)
{
               
    if(size_1<=0)
    {
        return ERROR;
    }
    
    while(size_1>0)
    {       
        if(x>(lcd_dev->lcd_solution.x_pixel-lcd_dev->font.font_x_model))
        {  
            x=0;
            y+=lcd_dev->font.font_y_model;
        }
        
        if(y>((lcd_dev->lcd_solution.y_pixel - lcd_dev->font.font_y_model)))
        {
            y=x=0;



            clear_screen(0x00);

        }
        
        lcd_showchar(x,y,*p);
        
        x+=lcd_dev->font.font_x_model;
        
        p++;
        
        size_1--;
        
    } 
     		
    lcd_dev->cursor_record.x_cord=x;
    lcd_dev->cursor_record.y_cord=y;
    
    return SUCCESS;
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
    u8 ch_temp=ch;
    if(ch_temp=='\n')
    {		
        lcd_dev->cursor_record.y_cord += lcd_dev->font.font_y_model;
        
    }else if(ch_temp=='\r')
    {
        lcd_dev->cursor_record.x_cord = 0;
    }else
    {
        lcd_showstring(lcd_dev->cursor_record.x_cord , lcd_dev->cursor_record.y_cord , &ch_temp , 1  );
        lcd_refresh_buffer();
    }
    return ch;
}











 


static const  unsigned char font16x8[95][16]={	  
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xCC,0x00,0x0C,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x08,0x00,0x30,0x00,0x60,0x00,0x08,0x00,0x30,0x00,0x60,0x00,0x00,0x00}, 
    {0x02,0x20,0x03,0xFC,0x1E,0x20,0x02,0x20,0x03,0xFC,0x1E,0x20,0x02,0x20,0x00,0x00}, 
    {0x00,0x00,0x0E,0x18,0x11,0x04,0x3F,0xFF,0x10,0x84,0x0C,0x78,0x00,0x00,0x00,0x00}, 
    {0x0F,0x00,0x10,0x84,0x0F,0x38,0x00,0xC0,0x07,0x78,0x18,0x84,0x00,0x78,0x00,0x00}, 
    {0x00,0x78,0x0F,0x84,0x10,0xC4,0x11,0x24,0x0E,0x98,0x00,0xE4,0x00,0x84,0x00,0x08}, 
    {0x08,0x00,0x68,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xE0,0x18,0x18,0x20,0x04,0x40,0x02,0x00,0x00}, 
    {0x00,0x00,0x40,0x02,0x20,0x04,0x18,0x18,0x07,0xE0,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x02,0x40,0x02,0x40,0x01,0x80,0x0F,0xF0,0x01,0x80,0x02,0x40,0x02,0x40,0x00,0x00}, 
    {0x00,0x80,0x00,0x80,0x00,0x80,0x0F,0xF8,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x00}, 
    {0x00,0x01,0x00,0x0D,0x00,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x80}, 
    {0x00,0x00,0x00,0x0C,0x00,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x06,0x00,0x18,0x00,0x60,0x01,0x80,0x06,0x00,0x18,0x00,0x20,0x00}, 
    {0x00,0x00,0x07,0xF0,0x08,0x08,0x10,0x04,0x10,0x04,0x08,0x08,0x07,0xF0,0x00,0x00}, 
    {0x00,0x00,0x08,0x04,0x08,0x04,0x1F,0xFC,0x00,0x04,0x00,0x04,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x0E,0x0C,0x10,0x14,0x10,0x24,0x10,0x44,0x11,0x84,0x0E,0x0C,0x00,0x00}, 
    {0x00,0x00,0x0C,0x18,0x10,0x04,0x11,0x04,0x11,0x04,0x12,0x88,0x0C,0x70,0x00,0x00}, 
    {0x00,0x00,0x00,0xE0,0x03,0x20,0x04,0x24,0x08,0x24,0x1F,0xFC,0x00,0x24,0x00,0x00}, 
    {0x00,0x00,0x1F,0x98,0x10,0x84,0x11,0x04,0x11,0x04,0x10,0x88,0x10,0x70,0x00,0x00}, 
    {0x00,0x00,0x07,0xF0,0x08,0x88,0x11,0x04,0x11,0x04,0x18,0x88,0x00,0x70,0x00,0x00}, 
    {0x00,0x00,0x1C,0x00,0x10,0x00,0x10,0xFC,0x13,0x00,0x1C,0x00,0x10,0x00,0x00,0x00}, 
    {0x00,0x00,0x0E,0x38,0x11,0x44,0x10,0x84,0x10,0x84,0x11,0x44,0x0E,0x38,0x00,0x00}, 
    {0x00,0x00,0x07,0x00,0x08,0x8C,0x10,0x44,0x10,0x44,0x08,0x88,0x07,0xF0,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x0C,0x03,0x0C,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x80,0x01,0x40,0x02,0x20,0x04,0x10,0x08,0x08,0x10,0x04,0x00,0x00}, 
    {0x02,0x20,0x02,0x20,0x02,0x20,0x02,0x20,0x02,0x20,0x02,0x20,0x02,0x20,0x00,0x00}, 
    {0x00,0x00,0x10,0x04,0x08,0x08,0x04,0x10,0x02,0x20,0x01,0x40,0x00,0x80,0x00,0x00}, 
    {0x00,0x00,0x0E,0x00,0x12,0x00,0x10,0x0C,0x10,0x6C,0x10,0x80,0x0F,0x00,0x00,0x00}, 
    {0x03,0xE0,0x0C,0x18,0x13,0xE4,0x14,0x24,0x17,0xC4,0x08,0x28,0x07,0xD0,0x00,0x00}, 
    {0x00,0x04,0x00,0x3C,0x03,0xC4,0x1C,0x40,0x07,0x40,0x00,0xE4,0x00,0x1C,0x00,0x04}, 
    {0x10,0x04,0x1F,0xFC,0x11,0x04,0x11,0x04,0x11,0x04,0x0E,0x88,0x00,0x70,0x00,0x00}, 
    {0x03,0xE0,0x0C,0x18,0x10,0x04,0x10,0x04,0x10,0x04,0x10,0x08,0x1C,0x10,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x10,0x04,0x10,0x04,0x10,0x04,0x08,0x08,0x07,0xF0,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x11,0x04,0x11,0x04,0x17,0xC4,0x10,0x04,0x08,0x18,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x11,0x04,0x11,0x00,0x17,0xC0,0x10,0x00,0x08,0x00,0x00,0x00}, 
    {0x03,0xE0,0x0C,0x18,0x10,0x04,0x10,0x04,0x10,0x44,0x1C,0x78,0x00,0x40,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x10,0x84,0x00,0x80,0x00,0x80,0x10,0x84,0x1F,0xFC,0x10,0x04}, 
    {0x00,0x00,0x10,0x04,0x10,0x04,0x1F,0xFC,0x10,0x04,0x10,0x04,0x00,0x00,0x00,0x00}, 
    {0x00,0x03,0x00,0x01,0x10,0x01,0x10,0x01,0x1F,0xFE,0x10,0x00,0x10,0x00,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x11,0x04,0x03,0x80,0x14,0x64,0x18,0x1C,0x10,0x04,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x10,0x04,0x00,0x04,0x00,0x04,0x00,0x04,0x00,0x0C,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x1F,0x00,0x00,0xFC,0x1F,0x00,0x1F,0xFC,0x10,0x04,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x0C,0x04,0x03,0x00,0x00,0xE0,0x10,0x18,0x1F,0xFC,0x10,0x00}, 
    {0x07,0xF0,0x08,0x08,0x10,0x04,0x10,0x04,0x10,0x04,0x08,0x08,0x07,0xF0,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x10,0x84,0x10,0x80,0x10,0x80,0x10,0x80,0x0F,0x00,0x00,0x00}, 
    {0x07,0xF0,0x08,0x18,0x10,0x24,0x10,0x24,0x10,0x1C,0x08,0x0A,0x07,0xF2,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x11,0x04,0x11,0x00,0x11,0xC0,0x11,0x30,0x0E,0x0C,0x00,0x04}, 
    {0x00,0x00,0x0E,0x1C,0x11,0x04,0x10,0x84,0x10,0x84,0x10,0x44,0x1C,0x38,0x00,0x00}, 
    {0x18,0x00,0x10,0x00,0x10,0x04,0x1F,0xFC,0x10,0x04,0x10,0x00,0x18,0x00,0x00,0x00}, 
    {0x10,0x00,0x1F,0xF8,0x10,0x04,0x00,0x04,0x00,0x04,0x10,0x04,0x1F,0xF8,0x10,0x00}, 
    {0x10,0x00,0x1E,0x00,0x11,0xE0,0x00,0x1C,0x00,0x70,0x13,0x80,0x1C,0x00,0x10,0x00}, 
    {0x1F,0xC0,0x10,0x3C,0x00,0xE0,0x1F,0x00,0x00,0xE0,0x10,0x3C,0x1F,0xC0,0x00,0x00}, 
    {0x10,0x04,0x18,0x0C,0x16,0x34,0x01,0xC0,0x01,0xC0,0x16,0x34,0x18,0x0C,0x10,0x04}, 
    {0x10,0x00,0x1C,0x00,0x13,0x04,0x00,0xFC,0x13,0x04,0x1C,0x00,0x10,0x00,0x00,0x00}, 
    {0x08,0x04,0x10,0x1C,0x10,0x64,0x10,0x84,0x13,0x04,0x1C,0x04,0x10,0x18,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0xFE,0x40,0x02,0x40,0x02,0x40,0x02,0x00,0x00}, 
    {0x00,0x00,0x30,0x00,0x0C,0x00,0x03,0x80,0x00,0x60,0x00,0x1C,0x00,0x03,0x00,0x00}, 
    {0x00,0x00,0x40,0x02,0x40,0x02,0x40,0x02,0x7F,0xFE,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x20,0x00,0x40,0x00,0x40,0x00,0x40,0x00,0x20,0x00,0x00,0x00}, 
    {0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01}, 
    {0x00,0x00,0x40,0x00,0x40,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x98,0x01,0x24,0x01,0x44,0x01,0x44,0x01,0x44,0x00,0xFC,0x00,0x04}, 
    {0x10,0x00,0x1F,0xFC,0x00,0x88,0x01,0x04,0x01,0x04,0x00,0x88,0x00,0x70,0x00,0x00}, 
    {0x00,0x00,0x00,0x70,0x00,0x88,0x01,0x04,0x01,0x04,0x01,0x04,0x00,0x88,0x00,0x00}, 
    {0x00,0x00,0x00,0x70,0x00,0x88,0x01,0x04,0x01,0x04,0x11,0x08,0x1F,0xFC,0x00,0x04}, 
    {0x00,0x00,0x00,0xF8,0x01,0x44,0x01,0x44,0x01,0x44,0x01,0x44,0x00,0xC8,0x00,0x00}, 
    {0x00,0x00,0x01,0x04,0x01,0x04,0x0F,0xFC,0x11,0x04,0x11,0x04,0x11,0x00,0x18,0x00}, 
    {0x00,0x00,0x00,0xD6,0x01,0x29,0x01,0x29,0x01,0x29,0x01,0xC9,0x01,0x06,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x00,0x84,0x01,0x00,0x01,0x00,0x01,0x04,0x00,0xFC,0x00,0x04}, 
    {0x00,0x00,0x01,0x04,0x19,0x04,0x19,0xFC,0x00,0x04,0x00,0x04,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x00,0x03,0x00,0x01,0x01,0x01,0x19,0x01,0x19,0xFE,0x00,0x00,0x00,0x00}, 
    {0x10,0x04,0x1F,0xFC,0x00,0x24,0x00,0x40,0x01,0xB4,0x01,0x0C,0x01,0x04,0x00,0x00}, 
    {0x00,0x00,0x10,0x04,0x10,0x04,0x1F,0xFC,0x00,0x04,0x00,0x04,0x00,0x00,0x00,0x00}, 
    {0x01,0x04,0x01,0xFC,0x01,0x04,0x01,0x00,0x01,0xFC,0x01,0x04,0x01,0x00,0x00,0xFC}, 
    {0x01,0x04,0x01,0xFC,0x00,0x84,0x01,0x00,0x01,0x00,0x01,0x04,0x00,0xFC,0x00,0x04}, 
    {0x00,0x00,0x00,0xF8,0x01,0x04,0x01,0x04,0x01,0x04,0x01,0x04,0x00,0xF8,0x00,0x00}, 
    {0x01,0x01,0x01,0xFF,0x00,0x85,0x01,0x04,0x01,0x04,0x00,0x88,0x00,0x70,0x00,0x00}, 
    {0x00,0x00,0x00,0x70,0x00,0x88,0x01,0x04,0x01,0x04,0x01,0x05,0x01,0xFF,0x00,0x01}, 
    {0x01,0x04,0x01,0x04,0x01,0xFC,0x00,0x84,0x01,0x04,0x01,0x00,0x01,0x80,0x00,0x00}, 
    {0x00,0x00,0x00,0xCC,0x01,0x24,0x01,0x24,0x01,0x24,0x01,0x24,0x01,0x98,0x00,0x00}, 
    {0x00,0x00,0x01,0x00,0x01,0x00,0x07,0xF8,0x01,0x04,0x01,0x04,0x00,0x00,0x00,0x00}, 
    {0x01,0x00,0x01,0xF8,0x00,0x04,0x00,0x04,0x00,0x04,0x01,0x08,0x01,0xFC,0x00,0x04}, 
    {0x01,0x00,0x01,0x80,0x01,0x70,0x00,0x0C,0x00,0x10,0x01,0x60,0x01,0x80,0x01,0x00}, 
    {0x01,0xF0,0x01,0x0C,0x00,0x30,0x01,0xC0,0x00,0x30,0x01,0x0C,0x01,0xF0,0x01,0x00}, 
    {0x00,0x00,0x01,0x04,0x01,0x8C,0x00,0x74,0x01,0x70,0x01,0x8C,0x01,0x04,0x00,0x00}, 
    {0x01,0x01,0x01,0x81,0x01,0x71,0x00,0x0E,0x00,0x18,0x01,0x60,0x01,0x80,0x01,0x00}, 
    {0x00,0x00,0x01,0x84,0x01,0x0C,0x01,0x34,0x01,0x44,0x01,0x84,0x01,0x0C,0x00,0x00}, 
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x3E,0xFC,0x40,0x02,0x40,0x02}, 
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x40,0x02,0x40,0x02,0x3E,0xFC,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x60,0x00,0x80,0x00,0x80,0x00,0x40,0x00,0x40,0x00,0x20,0x00,0x20,0x00}, 
}; 

