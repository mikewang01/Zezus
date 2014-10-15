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

#define MEM_BLOCK_SIZE   16          //�ڴ���СΪ32�ֽ�
#define MAX_MEM_SIZE     1*1024       //�������ڴ� 1K
#define MEM_ALLOC_TABLE_SIZE MAX_MEM_SIZE/MEM_BLOCK_SIZE //�ڴ���С


/* Exported constants --------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/
//�ڴ���������
struct _m_mallco_dev
{
	void (*init)(void);     //��ʼ��
	u8 (*perused)(void);         //�ڴ�ʹ����
	u8  membase[MAX_MEM_SIZE];   //�ڴ��
	u16 memmap[MEM_ALLOC_TABLE_SIZE];  //�ڴ����״̬��
	u8  memrdy;        //�ڴ�����Ƿ����
	u16 mem_current_used;
	u16 mem_max_used;
};

/* Exported functions ------------------------------------------------------------*/
extern struct _m_mallco_dev mallco_dev;  //��mallco.c���涨��

void osmemset(void *s,u8 c,u32 count);  //�����ڴ�
void osmemcpy(void *des,void *src,u32 n);//�����ڴ� 

void mem_init(void);      //�ڴ�����ʼ������(��/�ڲ�����)
static u32 mem_malloc(u32 size);     //�ڴ����(�ڲ�����)
static u8 mem_free(u32 offset);     //�ڴ��ͷ�(�ڲ�����)
u8 mem_perused(void);      //����ڴ�ʹ����(��/�ڲ�����)
////////////////////////////////////////////////////////////////////////////////
//�û����ú���
u8 osfree(void *ptr);       //�ڴ��ͷ�(�ⲿ����)
void *osmalloc(u32 size);     //�ڴ����(�ⲿ����)
void *osrealloc(void *ptr,u32 size);  //���·����ڴ�(�ⲿ����)
void *os_memmove(void *dest, const void *src, u32 n); 
u16 crc16(u8 *ptr,u8 len);

#endif

