//----------------------------------------------------------------------------
// WSH Corp. Company Confidential Strictly Private
//
// $RCSfile: task.c,v $
// $Revision: 0.1 $
// $Author: Mike Wang $
// $Date: 2014/5/10 19:37:23 $
// $Description: Task file
//
// ---------------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<
// ---------------------------------------------------------------------------
// Copyright 2013-2014 (c) WSH Corp.
//
// WSH Corp. owns the sole copyright to this software. Under international
// copyright laws you (1) may not make a copy of this software except for
// the purposes of maintaining a single archive copy, (2) may not derive
// works herefrom, (3) may not distribute this work to others. These rights
// are provided for information clarification, other restrictions of rights
// may apply as well.
//
// This is an unpublished work.
// ---------------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>> WARRANTEE <<<<<<<<<<<<<<<<<<<<<
// ---------------------------------------------------------------------------
// WSH Corp. MAKES NO WARRANTY OF ANY KIND WITH REGARD TO THE USE OF
// THIS SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR
// PURPOSE.
// ---------------------------------------------------------------------------


/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "kernel-includes.h"
#include "drivers-includes.h"
// ***************************************************************************
// ******************** START OF global variable DECLARATIONS *******************
// **************************************************************************
struct _m_mallco_dev mallco_dev=
{
	mem_init, //�ڴ��ʼ��
	mem_perused,//�ڴ�ʹ���
	0,   //�ڴ��
	0,   //�ڴ����״̬��
	0,     //�ڴ����δ����
	0,
	0
};


/*********************************************************************
 * FUNCTIONS
 *********************************************************************/



/*********************************************************************
 * @fn      osmemcpy
 *
 * @brief   copy specific amount of data from source adress to destination adress.
 *
* @param   *des:Ŀ�ĵ�ַ *src:Դ��ַ n: number of the data gonna be coppied
 *
 * @return  none
 */
void osmemcpy(void *des,void *src,u32 n) 
{ 
    u8 *xdes=des;
		u8 *xsrc=src;
    while(n--)*xdes++=*xsrc++; 
} 

/*********************************************************************
 * @fn      osmemset
 *
 * @brief   set memory to a specific data
 *
 * @param   *s:�ڴ��׵�ַ c :Ҫ���õ�ֵ count:��Ҫ���õ��ڴ��С(�ֽ�Ϊ��λ)
 *
 * @return  none
 */
void osmemset(void *s,u8 c,u32 count) 
{ 
    u8 *xs = s; 
    while(count--)*xs++=c; 
}  

/**
 * This function will move memory content from source address to destination
 * address.
 *
 * @param dest the address of destination memory
 * @param src  the address of source memory
 * @param n the copied length
 *
 * @return the address of destination memory
 */
void *os_memmove(void *dest, const void *src, u32 n)
{
    char *tmp = (char *)dest, *s = (char *)src;

    if (s < tmp && tmp < s + n)
    {
        tmp += n;
        s += n;

        while (n--)
            *(--tmp) = *(--s);
    }
    else
    {
        while (n--)
            *tmp++ = *s++;
    }

    return dest;
}

/*********************************************************************
 * @fn      mem_init
 *
 * @brief   initiate the memory manager structor and memory buffer
 *
 * @param   void
 *
 * @return  none
 */
void mem_init(void) 
{ 
    osmemset(mallco_dev.membase, 0, sizeof(mallco_dev.membase));//�ڴ�������������� 
    mallco_dev.memrdy=1;//�ڴ�����ʼ��OK 
} 




/*********************************************************************
 * @fn      mem_perused
 *
 * @brief   get memory usage in percent
 *
 * @param   void
 *
 * @return  memory usage %
 */
static u16 max_used=0;
u8 mem_perused(void) 
{ 
    u16 used=0; 
    u32 i; 
    for(i=0;i<MEM_ALLOC_TABLE_SIZE;i++) 
    { 
        if(mallco_dev.memmap[i])used++;
    } 
		if(used>max_used)
		{
			max_used=used;
		}
    return ((used*100)/MEM_ALLOC_TABLE_SIZE); 
} 

/*********************************************************************
 * @fn      mem_malloc 
 *
 * @brief   allocate  memory from buffer (internal function)
 *
 * @param   void
 *
 * @return  memory adress
 */
static u32 mem_malloc(u32 size) 
{ 
    signed long offset=0; 
    u16 nmemb; //��Ҫ���ڴ���� 
	  u16 cmemb=0;//�������ڴ����
    u32 i; 

    if(!mallco_dev.memrdy)mallco_dev.init();//δ��ʼ��,��ִ�г�ʼ��
    if(size==0)return 0XFFFFFFFF;//����Ҫ����

    nmemb=size/MEM_BLOCK_SIZE;   //��ȡ��Ҫ����������ڴ����

    if(size%MEM_BLOCK_SIZE)nmemb++;
	
         __disable_irq();
	
    for(offset=MEM_ALLOC_TABLE_SIZE-1;offset>=0;offset--)//���������ڴ������ 
    {    
			if(!mallco_dev.memmap[offset])cmemb++; //�������ڴ��������
			else cmemb=0;       //�����ڴ������
			
			if(cmemb==nmemb)      //�ҵ�������nmemb�����ڴ��
			{
								for(i=0;i<nmemb;i++)      //��ע�ڴ��ǿ�
								{ 
										mallco_dev.memmap[offset+i]=nmemb; 
								} 
								
								mallco_dev.mem_current_used+=nmemb;
								
								if(mallco_dev.mem_current_used>mallco_dev.mem_max_used) //record the max mem have been used
								{
										mallco_dev.mem_max_used=mallco_dev.mem_current_used;
								}
								  goto sucess;
								
			}
			
    } 
	
  	          goto fault;
sucess:
		__enable_irq();
		return (offset*MEM_BLOCK_SIZE);//����ƫ�Ƶ�ַ 
fault:
		__enable_irq();
		return 0XFFFFFFFF;//ƫ�Ƴ�����.		
} 

/*********************************************************************
 * @fn      mem_free 
 *
 * @brief   allocate  memory from buffer (internal function)
 *
 * @param   offset:�ڴ��ַƫ�� 
 *
 * @return  SUCESS ,ERROR
 */
u8 mem_free(u32 offset) 
{ 
    int i; 
    if(!mallco_dev.memrdy)//δ��ʼ��,��ִ�г�ʼ��
		{
				 mallco_dev.init();   
         return ERROR;//δ��ʼ�� 
    } 
       __disable_irq();
    if(offset<MAX_MEM_SIZE)//ƫ�����ڴ����.
    { 
        int index=offset/MEM_BLOCK_SIZE;//ƫ�������ڴ����� 
        int nmemb=mallco_dev.memmap[index];   //�ڴ������
        for(i=0;i<nmemb;i++)     //�ڴ������
        { 
            mallco_dev.memmap[index+i]=0; 
        }
				
        mallco_dev.mem_current_used-=nmemb;
				
				goto sucess;
        
    }else 
		{
		   goto fault;
		}			
sucess:
		__enable_irq();
		return SUCCESS; 
fault:
		__enable_irq();
		return ERROR;//ƫ�Ƴ�����.
} 
/*********************************************************************
 * @fn      mem_free 
 *
 * @brief   allocate  memory from buffer (internal function)
 *
 * @param   offset:�ڴ��ַƫ�� 
 *
 * @return  SUCESS ,ERROR
 */
u8 osfree(void *ptr) 
{ 
	u32 offset; 
    if(ptr==NULL)return ERROR;//��ַΪ0. 
    offset=(u32)ptr-(u32)(mallco_dev.membase); 
   return mem_free(offset);//�ͷ��ڴ�    
} 
/*********************************************************************
 * @fn      osmalloc 
 *
 * @brief   allocate  memory from buffer (internal function)
 *
 * @param   offset:�ڴ��ַƫ�� 
 *
 * @return  SUCESS ,ERROR
 */
void *osmalloc(u32 size) 
{ 
    u32 offset; 
    offset=mem_malloc(size);
	 
    if(offset==0XFFFFFFFF)return NULL; 
    else return (void*)((u32)&mallco_dev.membase+offset); 
} 
/*********************************************************************
 * @fn      osrealloc 
 *
 * @brief   allocate  memory from buffer (internal function)
 *
 * @param   offset:�ڴ��ַƫ�� 
 *
 * @return  SUCESS ,ERROR
 */
void *osrealloc(void *ptr,u32 size) 
{ 
    u32 offset; 
    offset=mem_malloc(size);
	 
    if(offset==0XFFFFFFFF)return NULL;    
    else 
    { 
        osmemcpy((void*)((u32)&mallco_dev.membase+offset),ptr,size);//�������ڴ����ݵ����ڴ�  
        osfree(ptr);               //�ͷž��ڴ�
        return (void*)((u32)&mallco_dev.membase+offset);          //�������ڴ��׵�ַ
    } 
}

 /*********************************************************************
 * @fn      crc16()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */

u16 crc16(u8 *ptr,u8 len) // ptr ?????,len ?????
{
	u8 i;
	u16 crc;
	while(len--)
	{
		for(i=0x80; i!=0; i>>=1)
		{
			if((crc&0x8000)!=0) {crc<<=1; crc^=0x1021;} 
			else crc<<=1; 
			if((*ptr&i)!=0) crc^=0x1021; 
		}
			ptr++;
		}
		return(crc);
}


 
 /*********************************************************************
 * @fn      _list_timer()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
os_err_t _list_mem()
{

   
	
    printf("\r\ntotalmem    blocksize   totalblock  mem_cur_used  mem max_used\r\n");
        printf("----------  ----------  ----------  -----------   -----------\r\n");
   
       printf("0x%08x  0x%08x  0x%08x  0x%08x   0x%08x\r\n",
                   MAX_MEM_SIZE,
                   MEM_BLOCK_SIZE,
									 MEM_ALLOC_TABLE_SIZE,
                   mallco_dev.mem_current_used,
									 mallco_dev.mem_max_used
							);
      
    return 0;
}


