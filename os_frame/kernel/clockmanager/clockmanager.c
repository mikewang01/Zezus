
/***************************************************************
 * Name:      clockmanager.c
 * Purpose:   code for clock management 
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */ 


#include "kernel-includes.h"
#include "string.h"

static struct os_object *clock_list;
/*********************************************************************
 * @fn      OS_err_t OS_device_register
 *
 * @brief   This function registers a device driver with specified name
 *
 * 			    @param dev the pointer of device driver structure 
 *					@param name the device driver's name
 *					@param flags the flag of device
 * 
 *   @return the error code, OS_EOK on initialization successfully. 
 */
os_err_t os_clock_register(Peripheral_clock *clock,   const char  *name)   
{   
    if (clock == NULL)   
        return ERROR;   
    
    
#ifdef CLOCK_IN_ROM
    clock_list=&clock->parent;
#else
    if (os_object_find(clock_list,name) != NULL)//if this device has been registored return error  
        return ERROR;   
    clock->status = clock_closed;
    clock->last_status = clock_closed;
    
    os_object_init(&clock_list, &(clock->parent), OS_Object_Class_Clock , name);//???????,?????????????????????,??????   
#endif    
    return SUCCESS;   
}


/*********************************************************************
 * @fn      os_err_t rt_device_set_tx_complete
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */	
Peripheral_clock* os_clock_get(const char * name)  
{  
    if(clock_list==NULL)
    {
        return NULL;
    }	else
    {
        
#ifdef CLOCK_IN_ROM	
        struct os_object *obj=clock_list;
        while(obj!=NULL)
        {
            
            if(strcmp(name,obj->name)==0)
            {
                struct os_object *__mptr = (obj);
                return ((Peripheral_clock *)( (char *)__mptr - offsetof(Peripheral_clock,parent)));
            }
            
            obj++;			
            
        }
#else
        struct os_object *obj=os_object_find(clock_list,name);
        if(obj!=NULL)
        {
            struct os_object *__mptr = (obj);
            return ((Peripheral_clock *)( (char *)__mptr - offsetof(Peripheral_clock,parent)));
        }
#endif
    }
    return NULL;
} 




/*********************************************************************
 * @fn      os_err_t rt_device_set_tx_complete
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */	
os_err_t os_clock_open(const char * name)  
{  
    if(name==NULL)
    {
        return NULL;
    }else if(strcmp(name,"ALL")==0) // this means open all clock registered
    {
        
        struct os_object *__mptr=clock_list;
#ifdef CLOCK_IN_ROM		
        while(__mptr!=NULL)
        {
            Peripheral_clock *phr_clock=((Peripheral_clock *)( (char *)__mptr - offsetof(Peripheral_clock,parent)));
            if(phr_clock->clock_enable!=NULL&&phr_clock->pstatus_record->status!=clock_opened)
            {
                phr_clock->clock_enable();
                
                phr_clock->pstatus_record->last_status=phr_clock->pstatus_record->status; //save last status ;
                phr_clock->pstatus_record->status=clock_opened;
            }
            __mptr++;
        }
#else
        while(__mptr!=NULL)
        {
            Peripheral_clock *phr_clock=((Peripheral_clock *)( (char *)__mptr - offsetof(Peripheral_clock,parent)));
            
            
            
            if(phr_clock->clock_enable!=NULL&&phr_clock->status!=clock_opened)
            {
                phr_clock->clock_enable();
                
                
                phr_clock->last_status=phr_clock->status; //save last status ;
                phr_clock->status=clock_opened;
            }
            __mptr=__mptr->next;
        }
        
        
#endif
    }else// initiate a specific device here
    {
        Peripheral_clock *phr_clock=os_clock_get(name);
        
        if(phr_clock!=NULL)
        {
            
            if(phr_clock->clock_enable!=NULL)
            {
#ifdef CLOCK_IN_ROM	
                
                if(phr_clock->pstatus_record->status!=clock_opened)
#else
                if(phr_clock->status!=clock_opened)
#endif
                {
                    phr_clock->clock_enable();
                    
                    
#ifdef CLOCK_IN_ROM	
                    
                    (phr_clock->pstatus_record->last_status)=phr_clock->pstatus_record->status; //save last status ;
                    (phr_clock->pstatus_record->status)=clock_opened;
#else	
                    phr_clock->last_status=phr_clock->status; //save last status ;
                    phr_clock->status=clock_opened;
#endif
                }
            }
        }
        
    }
    return SUCCESS;
} 


/*********************************************************************
 * @fn      os_err_t rt_device_set_tx_complete
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */	
os_err_t os_clock_close(const char * name)  
{  
    if(name==NULL)
    {
        return NULL;
    }else if(strcmp(name,"ALL")==0) // this means open all clock registered
    {
        struct os_object *__mptr=clock_list;
#ifdef CLOCK_IN_ROM		
        while(__mptr!=NULL)
        {
            Peripheral_clock *phr_clock=((Peripheral_clock *)( (char *)__mptr - offsetof(Peripheral_clock,parent)));
            if(phr_clock->clock_disable!=NULL&&phr_clock->pstatus_record->status!=clock_closed)
            {
                phr_clock->clock_disable();
                
                phr_clock->pstatus_record->last_status=phr_clock->pstatus_record->status; //save last status ;
                phr_clock->pstatus_record->status=clock_closed;
            }
            __mptr++;
        }
#else
        while(__mptr!=NULL)
        {
            Peripheral_clock *phr_clock=((Peripheral_clock *)( (char *)__mptr - offsetof(Peripheral_clock,parent)));
            
            //save last status 
            if(phr_clock->clock_disable!=NULL&&phr_clock->status!=clock_closed)
            {
                phr_clock->clock_disable();
                phr_clock->last_status=phr_clock->status; 
                phr_clock->status=clock_closed; 
            }
            
            __mptr=__mptr->next;
        }
#endif
    }else// initiate a specific device here
    {
        Peripheral_clock *phr_clock=os_clock_get(name);
        
        if(phr_clock!=NULL)
        {
            
            if(phr_clock->clock_disable!=NULL)
            {
#ifdef CLOCK_IN_ROM	
                if(phr_clock->pstatus_record->status!=clock_closed)
#else
                if(phr_clock->status!=clock_closed)
#endif
                { 
                    
                    phr_clock->clock_disable();
#ifdef CLOCK_IN_ROM	
                    phr_clock->pstatus_record->last_status=phr_clock->pstatus_record->status; //save last status ;
                    phr_clock->pstatus_record->status=clock_closed;
#else	
                    phr_clock->last_status=phr_clock->status; 
                    phr_clock->status=clock_closed;
#endif
                }
            }
        }
    }
    return SUCCESS;
}


/*********************************************************************
 * @fn      os_err_t rt_device_set_tx_complete
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */	
os_err_t os_clock_restore(const char * name)  
{  
    if(name==NULL)
    {
        return NULL;
    }else if(strcmp(name,"ALL")==0) // this means open all clock registered
    {
        struct os_object *__mptr=clock_list;
        u8 temp;
#ifdef CLOCK_IN_ROM		
        while(__mptr!=NULL)
        {
            Peripheral_clock *phr_clock=((Peripheral_clock *)( (char *)__mptr - offsetof(Peripheral_clock,parent)));
            if(phr_clock->pstatus_record->last_status==clock_opened)
            {
                if(phr_clock->clock_enable!=NULL)
                {
                    phr_clock->clock_enable();
                }		
            }else
            {
                
                if(phr_clock->clock_disable!=NULL)
                {
                    phr_clock->clock_disable();
                }	
            }	
            temp                   =	 phr_clock->pstatus_record->last_status;					 
            phr_clock->pstatus_record->last_status =   phr_clock->pstatus_record->status;
            phr_clock->pstatus_record->status      =   temp;		
            __mptr                 =   __mptr->next;
            
            __mptr++;
        }
#else
        while(__mptr!=NULL)
        {
            Peripheral_clock *phr_clock=((Peripheral_clock *)( (char *)__mptr - offsetof(Peripheral_clock,parent)));
            u8 temp;
            if(phr_clock->last_status==clock_opened)
            {
                if(phr_clock->clock_enable!=NULL)
                {
                    phr_clock->clock_enable();
                }		
            }else
            {
                
                if(phr_clock->clock_disable!=NULL)
                {
                    phr_clock->clock_disable();
                }	
            }	
            
            temp                   =	 phr_clock->last_status;					 
            phr_clock->last_status =   phr_clock->status;
            phr_clock->status      =   temp;		
            __mptr                 =   __mptr->next;
        }
#endif
    }else// initiate a specific device here
    {  
        Peripheral_clock *phr_clock=os_clock_get(name);
        u8 temp;
#ifdef CLOCK_IN_ROM		
        if(phr_clock->pstatus_record->last_status==clock_opened)
        {
            if(phr_clock->clock_enable!=NULL)
            {
                phr_clock->clock_enable();
            }		
        }else
        {
            
            if(phr_clock->clock_disable!=NULL)
            {
                phr_clock->clock_disable();
            }	
        }	
        
        temp                   =	 phr_clock->pstatus_record->last_status;					 
        phr_clock->pstatus_record->last_status =   phr_clock->pstatus_record->status;
        phr_clock->pstatus_record->status      =   temp;
        
#else
        
        if(phr_clock->last_status==clock_opened)
        {
            if(phr_clock->clock_enable!=NULL)
            {
                phr_clock->clock_enable();
            }		
        }else
        {
            
            if(phr_clock->clock_disable!=NULL)
            {
                phr_clock->clock_disable();
            }	
        }	
        
        temp                   =	 phr_clock->last_status;					 
        phr_clock->last_status =   phr_clock->status;
        phr_clock->status      =   temp;
#endif								
    }
    return SUCCESS;
}


void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)	 
{ 	   	 
    
}
//设置NVIC分组
//NVIC_Group:NVIC分组 0~4 总共5组 		   
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
    u32 temp,temp1;	  
    temp1=(~NVIC_Group)&0x07;//取后三位
    temp1<<=8;
    temp=SCB->AIRCR;  //读取先前的设置
    temp&=0X0000F8FF; //清空先前分组
    temp|=0X05FA0000; //写入钥匙
    temp|=temp1;	   
    SCB->AIRCR=temp;  //设置分组	    	  				   
}
//设置NVIC 
//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority       :响应优先级
//NVIC_Channel           :中断编号
//NVIC_Group             :中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分:
//组0:0位抢占优先级,4位响应优先级
//组1:1位抢占优先级,3位响应优先级
//组2:2位抢占优先级,2位响应优先级
//组3:3位抢占优先级,1位响应优先级
//组4:4位抢占优先级,0位响应优先级
//NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先	   
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
    u32 temp;	
    MY_NVIC_PriorityGroupConfig(NVIC_Group);//设置分组
    temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
    temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
    temp&=0xf;//取低四位  
    NVIC->ISER[NVIC_Channel/32]|=(1<<NVIC_Channel%32);//使能中断位(要清除的话,相反操作就OK) 
    NVIC->IP[NVIC_Channel]|=temp<<4;//设置响应优先级和抢断优先级   	    	  				   
} 
//外部中断配置函数
//只针对GPIOA~G;不包括PVD,RTC和USB唤醒这三个
//参数:
//GPIOx:0~6,代表GPIOA~G
//BITx:需要使能的位;
//TRIM:触发模式,1,下升沿;2,上降沿;3，任意电平触发
//该函数一次只能配置1个IO口,多个IO口,需多次调用
//该函数会自动开启对应中断,以及屏蔽线   	    
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
{
    
} 	  
//不能在这里执行所有外设复位!否则至少引起串口不工作.		    
//把所有时钟寄存器复位		  
void MYRCC_DeInit(void)
{	
    
}


//系统软复位   
void Sys_Soft_Reset(void)
{   
    SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 		 
//JTAG模式设置,用于设置JTAG的模式
//mode:jtag,swd模式设置;00,全使能;01,使能SWD;10,全关闭;	   
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00		  
void JTAG_Set(u8 mode)
{
    
} 



void WFI_SET(void)

{
    
    __WFI();
    
}



void Sys_Standby(u8 var)
{
    
    
    
    RCC->APB1ENR |= 1<<28;             //open power reggitor 
    
    switch(var)
        
    {
    
    case sleepmode:{ break; }           //WFI??????
        
    case stopmode:{                    //PDDS+LPDS+SLEEPDEEP+WFI??????
        SCB->SCR |= 1<<2;      //??SLEEPDEEP? (SYS->CTRL)          
        
        PWR->CR  |= 1<<0;      //LPDS??   
        
        
        break;         
    }
        
    case standbymode:{                    //PDDS+SLEEPDEEP+WFI??????
        SCB->SCR |= 1<<2;      //??SLEEPDEEP? (SYS->CTRL)
        
        PWR->CR|=1<<1;          //PDDS??  
        
        break;         
        
    }
        
    }
    
    PWR->CR  |= 1<<2;                  //clear Wake-up flags
    PWR->CSR |= 1<<8;                  //设置WKUP用于唤醒
    
    WFI_SET();                      //??WFI??       
}


























