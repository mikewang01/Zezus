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
	mem_init, //内存初始化
	mem_perused,//内存使用�
	0,   //内存池
	0,   //内存管理状态表
	0,     //内存管理未就绪
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
* @param   *des:目的地址 *src:源地址 n: number of the data gonna be coppied
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
 * @param   *s:内存首地址 c :要设置的值 count:需要设置的内存大小(字节为单位)
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
    osmemset(mallco_dev.membase, 0, sizeof(mallco_dev.membase));//内存池素有数据清零 
    mallco_dev.memrdy=1;//内存管理初始化OK 
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
    u16 nmemb; //需要的内存块数 
	  u16 cmemb=0;//连续空内存块数
    u32 i; 

    if(!mallco_dev.memrdy)mallco_dev.init();//未初始化,先执行初始化
    if(size==0)return 0XFFFFFFFF;//不需要分配

    nmemb=size/MEM_BLOCK_SIZE;   //获取需要分配的连续内存块数

    if(size%MEM_BLOCK_SIZE)nmemb++;
	
         __disable_irq();
	
    for(offset=MEM_ALLOC_TABLE_SIZE-1;offset>=0;offset--)//搜索整个内存控制区 
    {    
			if(!mallco_dev.memmap[offset])cmemb++; //连续空内存块数增加
			else cmemb=0;       //连续内存块清零
			
			if(cmemb==nmemb)      //找到了连续nmemb个空内存块
			{
								for(i=0;i<nmemb;i++)      //标注内存块非空
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
		return (offset*MEM_BLOCK_SIZE);//返回偏移地址 
fault:
		__enable_irq();
		return 0XFFFFFFFF;//偏移超区了.		
} 

/*********************************************************************
 * @fn      mem_free 
 *
 * @brief   allocate  memory from buffer (internal function)
 *
 * @param   offset:内存地址偏移 
 *
 * @return  SUCESS ,ERROR
 */
u8 mem_free(u32 offset) 
{ 
    int i; 
    if(!mallco_dev.memrdy)//未初始化,先执行初始化
		{
				 mallco_dev.init();   
         return ERROR;//未初始化 
    } 
       __disable_irq();
    if(offset<MAX_MEM_SIZE)//偏移在内存池内.
    { 
        int index=offset/MEM_BLOCK_SIZE;//偏移所在内存块号码 
        int nmemb=mallco_dev.memmap[index];   //内存块数量
        for(i=0;i<nmemb;i++)     //内存块清零
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
		return ERROR;//偏移超区了.
} 
/*********************************************************************
 * @fn      mem_free 
 *
 * @brief   allocate  memory from buffer (internal function)
 *
 * @param   offset:内存地址偏移 
 *
 * @return  SUCESS ,ERROR
 */
u8 osfree(void *ptr) 
{ 
	u32 offset; 
    if(ptr==NULL)return ERROR;//地址为0. 
    offset=(u32)ptr-(u32)(mallco_dev.membase); 
   return mem_free(offset);//释放内存    
} 
/*********************************************************************
 * @fn      osmalloc 
 *
 * @brief   allocate  memory from buffer (internal function)
 *
 * @param   offset:内存地址偏移 
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
 * @param   offset:内存地址偏移 
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
        osmemcpy((void*)((u32)&mallco_dev.membase+offset),ptr,size);//拷贝旧内存内容到新内存  
        osfree(ptr);               //释放旧内存
        return (void*)((u32)&mallco_dev.membase+offset);          //返回新内存首地址
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


