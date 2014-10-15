
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
//����NVIC����
//NVIC_Group:NVIC���� 0~4 �ܹ�5�� 		   
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
    u32 temp,temp1;	  
    temp1=(~NVIC_Group)&0x07;//ȡ����λ
    temp1<<=8;
    temp=SCB->AIRCR;  //��ȡ��ǰ������
    temp&=0X0000F8FF; //�����ǰ����
    temp|=0X05FA0000; //д��Կ��
    temp|=temp1;	   
    SCB->AIRCR=temp;  //���÷���	    	  				   
}
//����NVIC 
//NVIC_PreemptionPriority:��ռ���ȼ�
//NVIC_SubPriority       :��Ӧ���ȼ�
//NVIC_Channel           :�жϱ��
//NVIC_Group             :�жϷ��� 0~4
//ע�����ȼ����ܳ����趨����ķ�Χ!����������벻���Ĵ���
//�黮��:
//��0:0λ��ռ���ȼ�,4λ��Ӧ���ȼ�
//��1:1λ��ռ���ȼ�,3λ��Ӧ���ȼ�
//��2:2λ��ռ���ȼ�,2λ��Ӧ���ȼ�
//��3:3λ��ռ���ȼ�,1λ��Ӧ���ȼ�
//��4:4λ��ռ���ȼ�,0λ��Ӧ���ȼ�
//NVIC_SubPriority��NVIC_PreemptionPriority��ԭ����,��ֵԽС,Խ����	   
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
    u32 temp;	
    MY_NVIC_PriorityGroupConfig(NVIC_Group);//���÷���
    temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
    temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
    temp&=0xf;//ȡ����λ  
    NVIC->ISER[NVIC_Channel/32]|=(1<<NVIC_Channel%32);//ʹ���ж�λ(Ҫ����Ļ�,�෴������OK) 
    NVIC->IP[NVIC_Channel]|=temp<<4;//������Ӧ���ȼ����������ȼ�   	    	  				   
} 
//�ⲿ�ж����ú���
//ֻ���GPIOA~G;������PVD,RTC��USB����������
//����:
//GPIOx:0~6,����GPIOA~G
//BITx:��Ҫʹ�ܵ�λ;
//TRIM:����ģʽ,1,������;2,�Ͻ���;3�������ƽ����
//�ú���һ��ֻ������1��IO��,���IO��,���ε���
//�ú������Զ�������Ӧ�ж�,�Լ�������   	    
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
{
    
} 	  
//����������ִ���������踴λ!�����������𴮿ڲ�����.		    
//������ʱ�ӼĴ�����λ		  
void MYRCC_DeInit(void)
{	
    
}


//ϵͳ��λ   
void Sys_Soft_Reset(void)
{   
    SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 		 
//JTAGģʽ����,��������JTAG��ģʽ
//mode:jtag,swdģʽ����;00,ȫʹ��;01,ʹ��SWD;10,ȫ�ر�;	   
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
    PWR->CSR |= 1<<8;                  //����WKUP���ڻ���
    
    WFI_SET();                      //??WFI??       
}


























