#ifndef __MANUFACTURE_TASK_H_
#define __MANUFACTURE_TASK_H_

#include "kernel-includes.h"
#include "drivers-includes.h"


void  task_manufacture_init(void);
void task_manufacture(void);

#define PAYLOAD_LENTH   14
#define MCU_ID_LENTH    12

#define  UID_BASE_ADRESS (((u32*)(0x1FFFF7AC)))


/*all order are from the host side*/
enum serial_order
{
    get_mac =1        ,
    get_hard_version ,
    get_soft_version ,
    get_vol          ,
    get_dsy          ,
    set_para_a       ,
    set_para_b       ,
    get_para_a       ,
    get_para_b       ,
	  get_para				 ,
	  set_para         ,
    comm_ack
};


 typedef struct
 {
   u16 lenth;
   u8 pay_load[PAYLOAD_LENTH];

 }packge_payload;

 typedef struct
 {
    u8 stf[3]; /*3 bits fram start bits 0XFF 0X55 0X7E*/
    u8 order;
    packge_payload payload;
    u16 crc_16;
	  u8  pre_end;
    u8  end_flag;	 
 }data_packge;

 typedef struct
 {
    u8 stf;
    u8 order;
    u16 crc_16;
 }ack_packge;



typedef struct
{
		u32 ID0;
	  u32 ID1;
	  u32 ID2;
} uid_96bis;


				


#endif

