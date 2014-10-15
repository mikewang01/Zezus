/***************************************************************
 * Name:      MEM.H
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-06-12
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/


#ifndef __MALLOC_H
#define __MALLOC_H


/*********************************************************************
 * INCLUDES
 */
#include "cpu.h"


/* Exported macros --------------------------------------------------------*/

#ifndef NULL
#define NULL 0
#endif

#define MEM_BLOCK_SIZE   16          //内存块大小为32字节
#define MAX_MEM_SIZE     1*1024       //最大管理内存 1K
#define MEM_ALLOC_TABLE_SIZE MAX_MEM_SIZE/MEM_BLOCK_SIZE //内存表大小


/* Exported constants --------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/
//内存管理控制器
struct _m_mallco_dev
{
	void (*init)(void);     //初始化
	u8 (*perused)(void);         //内存使用率
	u8  membase[MAX_MEM_SIZE];   //内存池
	u16 memmap[MEM_ALLOC_TABLE_SIZE];  //内存管理状态表
	u8  memrdy;        //内存管理是否就绪
	u16 mem_current_used;
	u16 mem_max_used;
};

/* Exported functions ------------------------------------------------------------*/
extern struct _m_mallco_dev mallco_dev;  //在mallco.c里面定义

void osmemset(void *s,u8 c,u32 count);  //设置内存
void osmemcpy(void *des,void *src,u32 n);//复制内存 

void mem_init(void);      //内存管理初始化函数(外/内部调用)
static u32 mem_malloc(u32 size);     //内存分配(内部调用)
static u8 mem_free(u32 offset);     //内存释放(内部调用)
u8 mem_perused(void);      //获得内存使用率(外/内部调用)
////////////////////////////////////////////////////////////////////////////////
//用户调用函数
u8 osfree(void *ptr);       //内存释放(外部调用)
void *osmalloc(u32 size);     //内存分配(外部调用)
void *osrealloc(void *ptr,u32 size);  //重新分配内存(外部调用)
void *os_memmove(void *dest, const void *src, u32 n); 
u16 crc16(u8 *ptr,u8 len);

#endif

