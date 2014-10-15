
/*********************************************************************
 * INCLUDES
 */

/************************************************************
Copyright (C), 2008-2014, Colorful Sea Tech. Co., Ltd.
FileName:       w25qxx.c
Author:      MikeWang   Version : 0.0          Date: 2014/10/9
Description:      code for flash driver     
Version:  0.0        
Function List:  

static  os_err_t   w25qxx_init   (os_device_t* dev);

static  os_size_t  w25qxx_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);


static  os_err_t   w25qxx_open(os_device_t* dev, u16 oflag);

static os_err_t    w25qxx_control(os_device_t* dev, u8 cmd, void *args);

static  void       w25qxx_write_sr(u8 sr);   
static  void       w25qxx_write_enable(void);  
static  void       w25qxx_write_disable(void);   
static  void       w25qxx_write_sr(u8 sr);
static  u8         w25qxx_read_sr(void) ;
static  u16        w25qxx_read_id(void);
static  u8         w25qxx_readwrite_byte(u8 data);
static void        w25qxx_erase_sector(u32 dst_addr);
static void        w25qxx_wait_busy(void) ;
static void        w25qxx_erase_chip(void);
static void        w25qxx_erase_sector(u32 dst_sector);
static void        w25qxx_wait_busy(void) ;
static void        spi_flash_powerdown(void);
static void        w25qxx_flash_wakeup(void);
static void        w25qxx_write_content(u8* p_buffer,u32 write_addr,u16 bytes_write);
static void        w25qxx_read_content(u8* p_buffer,u32 read_addr,u16 bytes_to_read);
    
1. -------History:         
<author>   <time>    <version >      <desc>
Mike      2014/10/9     0.0       build this moudle  
***********************************************************/
#include "stdio.h"
#include "spi.h"
#include "kernel-includes.h"
#include "drivers-includes.h"
#include "stm32f0xx_spi.h"
#include "w25qxx.h"
/*********************************************************************
* MACROS
*/

/*winbond w25qxx serials flash command list-------------------------------------------------------*/
#define W25X_WriteEnable		    0x06 
#define W25X_WriteDisable		  0x04 
#define W25X_ReadStatusReg		  0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData		     	0x03 
#define W25X_FastReadData	  	0x0B 
#define W25X_FastReadDual	  	0x3B 
#define W25X_PageProgram	     	0x02 
#define W25X_BlockErase		  	0xD8 
#define W25X_SectorErase	    	0x20 
#define W25X_ChipErase			    0xC7 
#define W25X_PowerDown			    0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			    0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		  0x9F 
#define W25X_NOP_ACTION        0X00
#define W25X_FILL_ACTION       0XFF
#define  W25QXX_CS_PORT  GPIOB
#define  W25QXX_CS_PIN		12
#define  SIZE_1K         1024



#define IS_FLASH_SLEEP()  (w25qxx_dev->flash_state == flash_sleep)
/*pull flash cs pin to  high*/
#define  w25qxx_cs_high() do{\
    W25QXX_CS_PORT->BSRR = BIT_SHIFT(W25QXX_CS_PIN);\
    }while(0)

/*pull flash cs pin to  low*/
#define  w25qxx_cs_low() do{\
    W25QXX_CS_PORT->BRR = BIT_SHIFT(W25QXX_CS_PIN);\
    }while(0)
/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static flash_device_t *w25qxx_dev=NULL;
static os_device_t *bus_dev=NULL;
static  const flash_inf_t flash_table_list[] = 
{
    /*flash W25Q16 */
    {
        (const u8*)"W25Q16",
        0xef14 ,
        256,
        4,
        16,
        32,		 
    },
    /*flash W25Q32*/ 
    {
        (const u8*)"W25Q32",
        0xef15 ,
        256,
        4,
        16,
        64,		 
    },
    /*flash W25Q64*/ 
    {
        (const u8*)"W25Q64",
        0xef16 ,
        256,
        4,
        16,
        128,		 
    },
    /*flash W25Q80*/ 
    {
        (const u8*)"W25Q80",
        0xef13 ,
        256,
        4,
        16,
        256,		 
    },
    
};	

/*********************************************************************
 * LOCAL function declarement
 */


static  os_err_t   w25qxx_init   (os_device_t* dev);

static  os_size_t  w25qxx_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);


static  os_err_t   w25qxx_open(os_device_t* dev, u16 oflag);

static os_err_t    w25qxx_control(os_device_t* dev, u8 cmd, void *args);




static  void       w25qxx_write_sr(u8 sr);   
static  void       w25qxx_write_enable(void);  
static  void       w25qxx_write_disable(void);   
static  void       w25qxx_write_sr(u8 sr);
static  u8         w25qxx_read_sr(void) ;
static  u16        w25qxx_read_id(void);
static  u8         w25qxx_readwrite_byte(u8 data);
static void        w25qxx_erase_sector(u32 dst_addr);
static void        w25qxx_wait_busy(void) ;
static void        w25qxx_erase_chip(void);
static void        w25qxx_erase_sector(u32 dst_sector);
static void        w25qxx_wait_busy(void) ;
static void        spi_flash_powerdown(void);
static void        w25qxx_flash_wakeup(void);
static void        w25qxx_write_content(u8* p_buffer,u32 write_addr,u32 bytes_write);
static void        w25qxx_read_content(u8* p_buffer,u32 read_addr,u32 bytes_to_read);
/*********************************************************************
 * @fn      os_err_t os_device_init_all
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
#define W25QXX_NAME "w25qxx"

os_err_t w25qxx_register(u16 task_id)
{
    
    w25qxx_dev=osmalloc(sizeof(flash_device_t));
    
    
    w25qxx_dev->os_device.type = OS_Device_Class_SPIBUS;
    
    w25qxx_dev->os_device.device_id = OS_DEVICE_USART_ID;
    
    w25qxx_dev->flash_id        = 0;
    
    w25qxx_dev->flash_state     = flash_sleep;
    
    w25qxx_dev->register_taskid = task_id;
    
    w25qxx_dev->os_device.init  = w25qxx_init;
    
    w25qxx_dev->os_device.open  = w25qxx_open;
    
    w25qxx_dev->os_device.write = w25qxx_write;
    
    w25qxx_dev->os_device.control = w25qxx_control;
    
    w25qxx_dev->os_device.rx_indicate = NULL;
    
    return os_device_register(&(w25qxx_dev->os_device), W25QXX_NAME, OS_DEVICE_FLAG_INACTIVATED);
    
}
/*********************************************************************
 * @fn      os_err_t uart1_init
 *
 * @brief   This function initiate device uaart1
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
* nition!! before U configure any register ,opening clock first
 */


static os_err_t  w25qxx_init   (os_device_t* dev)
{  	 
    
    
    u8 i=0;
    bus_dev= os_device_get("SPI2");
    
    if(bus_dev ==NULL)
    {
        return ERROR;
    }
    EXAM_ASSERT(bus_dev !=NULL );
    
    /*if flash is in sleep mode ,wake it up first*/
    if(IS_FLASH_SLEEP())
    {
        w25qxx_control(NULL ,flash_power_up, NULL);
    }
    w25qxx_dev->flash_id   = w25qxx_read_id();
    w25qxx_dev->flash_id   = w25qxx_read_id();
    DEBUG("flashid = %04x\r\n",w25qxx_dev->flash_id);
    
    /*find out the detailed flash parametor according to flash specifi id */
    for(i=0 ; i< sizeof(flash_table_list)/sizeof(flash_table_list[0]); i++)
    {
        
        if(w25qxx_dev->flash_id == flash_table_list[i].flash_id)
        {
            w25qxx_dev->flash_info = ((flash_table_list+i));
            break;
        }			 
    }
    /*if corresponed device is not found,return error*/
    EXAM_ASSERT(w25qxx_dev->flash_info !=NULL );
    //		w25qxx_write_content(s,34658,sizeof(s));
    //		w25qxx_read_content(buffer , 34658 , 23);
    //		printf("%s\r\n",buffer);
    //		while(1);
    return SUCCESS;
    
    
}


/*********************************************************************
 * @fn      os_err_t uart2_open
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
static os_err_t  w25qxx_open(os_device_t* dev, u16 oflag)
{
    
    return SUCCESS;	
}

/*********************************************************************
 * @fn      os_err_t uart2_close
 *
 * @brief   This function initiate all the device registered
 *
 * 			    @param none 
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
os_err_t  w25qxx_close(os_device_t* dev, u16 oflag)
{
    
    
    return SUCCESS;		
    
}
/*********************************************************************
 * @fn      os_err_t w25qxx_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart2
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  w25qxx_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
{
    
    
    /*if flash is in sleep mode ,wake it up first*/
    if(IS_FLASH_SLEEP())
    {
        w25qxx_control(NULL ,flash_power_up, NULL);
    }
		
    if(pos>=size||buffer==NULL||size>USART_TX_BUFFER_SIZE)
    {
        return  NULL;
    }else
    {
        
			
			  char* temp=(char*)buffer; 
        if(pos>0)
        {
            temp=temp+pos;
        }				 
        
        while(size>0)
        {
            
            
        }
    }
    
    return SUCCESS;
}	

/*********************************************************************
 * @fn      os_err_t w25qxx_read(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function is write driver for uart2
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_size_t  w25qxx_read(os_device_t* dev, os_off_t pos, void *buffer, os_size_t size)
{
    
			/*if flash is in sleep mode ,wake it up first*/
			if(IS_FLASH_SLEEP())
			{
					w25qxx_control(NULL ,flash_power_up, NULL);
			} 
			
    if(pos>=size||buffer==NULL||size>USART_TX_BUFFER_SIZE)
    {
        return  NULL;
    }else
    {
        char* temp=(char*)buffer; 
        if(pos>0)
        {
            temp=temp+pos;
        }				 
        
        while(size>0)
        {
            
            
            
        }
    }
    
    return SUCCESS;
}	


/*********************************************************************
 * @fn      os_err_t relays_control(os_device_t* dev, u8 cmd, void *args)
 *
 * @brief   This function is write driver for uart1
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data
*									 buffer: data need tansfering
* 								 size: data size tansfered
 *   @return number of data transfered
 */

static os_err_t  w25qxx_control(os_device_t* dev, u8 cmd, void *args)
{
    
    if(dev==NULL)
    {
        return ERROR;
    }
    switch(cmd)
    {
    
    case  flash_write_enable:     w25qxx_write_enable();
																	break;
			
    case  flash_write_disable:    w25qxx_write_disable();
		                              break;	
			
    case 	flash_page_program: 		break;
																	
			
    case  flash_read_sr:					((u8*)args)[1] = w25qxx_read_sr();
																	break;
    case  flash_write_sr:					 w25qxx_write_sr(((u8*)args)[1]);  
																	break;
    case	flash_power_up:			
															DEBUG("start waking chip up...");
																w25qxx_flash_wakeup();
																w25qxx_dev->flash_state = flash_work ;	
															DEBUG("chip waking done!");		
																	break;
		
     /*enter sleep mode to save power*/    
    case	flash_power_down:		spi_flash_powerdown();
																	break;
    case	flash_chip_erase:		DEBUG("start erasing chip...");
																w25qxx_erase_chip(); 
																w25qxx_dev->flash_state = flash_sleep;
															DEBUG("chip erasing done!");					
																	break;
		
    case	flash_block_erase:			break;
																	
    case	flash_sector_erase:			
																	w25qxx_erase_sector(((u16*)((u8*)(args)+1))[0]);
																	break;
																	
    case	flash_read_device_id:		((u16*)args)[1] = w25qxx_read_id();
																	break;
		
    case	flash_read_unique_id:		break;
		
    case	flash_read_data:				w25qxx_read_content(((u8*)(args))+((u8*)(args))[0] , ((u32*)((u8*)(args)+1))[0] ,((u16*)((u8*)(args)+5))[0]);  		
																	break;
																	
    case	flash_fast_read:				break;
    default:break;
        
    }
    return SUCCESS;
}

/*********************************************************************
 * @fn      static u8 w25qxx_readwrite_byte(u8 data)
 
 *
 * @brief   This function is intended to write one byte to spi device and recieve data at the same time
 *
* 			    @param data: data to transfer
 *   @return data recieved
 */
static u8 w25qxx_readwrite_byte(u8 data)
{
    u8 status=os_device_write(bus_dev, 0, &data, 1);   
    /*if write failed ,return error*/
    EXAM_ASSERT(status == SUCCESS );
    
    status=os_device_read(bus_dev, 0, &data, 1); 
    /*if write failed ,return error*/
    EXAM_ASSERT(status == SUCCESS );
    
    return data;
    
}



/*********************************************************************
 * @fn      static  u8 w25qxx_read_sr(void) 
 
 *
 * @brief   read status register content
 *					BIT7  6   5   4   3   2   1   0
 *					SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *					SPR:default0, satus resistor proteced bit woking with WP
 *					TB,BP2,BP1,BP0:FLASH area protected setup
 *					WEL:write  enable locked 
 *					BUSY:busy bit(1,busy;0,idle)
 *					default:0x00
 * 			    @param data: data to transfer
 *   @return data recieved
 */
static  u8 w25qxx_read_sr(void)  
{  
    u8 byte=0;   
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_ReadStatusReg);    //发送读取状态寄存器命令    
    byte=w25qxx_readwrite_byte(W25X_FILL_ACTION);             //读取一个字节  
    w25qxx_cs_high();                            //取消片选     
    return byte;   
} 

/*********************************************************************
 * @fn      static  void w25qxx_write_sr(u8 sr)
 
 *
 * @brief   write status register content
 *					BIT7  6   5   4   3   2   1   0
 *					SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *					SPR:default0, satus resistor proteced bit woking with WP
 *					TB,BP2,BP1,BP0:FLASH area protected setup
 *					WEL:write  enable locked 
 *					BUSY:busy bit(1,busy;0,idle)
 *					default:0x00
 * 			    @param data: data to transfer
 *   @return data recieved
 */
static  void w25qxx_write_sr(u8 sr)     
{   
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_WriteStatusReg);   //发送写取状态寄存器命令    
    w25qxx_readwrite_byte(sr);               //写入一个字节  
    w25qxx_cs_high();                            //取消片选     	      
}   


/*********************************************************************
 * @fn      static  void w25qxx_write_enable(void) 
 
 *
 * @brief   write enable function abbtained by setting WEL bit in sr
 * 			    @param void
 *   @return void
 */  
static  void w25qxx_write_enable(void)   
{
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_WriteEnable);      //发送写使能  
    w25qxx_cs_high();                            //取消片选     	      
} 



/*********************************************************************
 * @fn      static  void w25qxx_write_disable(void) 
 
 *
 * @brief   write enable function abbtained by clearing WEL bit in sr
 * 			    @param void
 *   @return void
 */  
static  void w25qxx_write_disable(void)   
{  
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_WriteDisable);     //发送写禁止指令    
    w25qxx_cs_high();                            //取消片选     	      
} 


/*********************************************************************
 * @fn      static u16 w25qxx_read_id(void)
 
 *
 * @brief   read specfic manufacture id from flash ic 
 *          0XEF13:Indicate  W25Q80  
 *					0XEF14:Indicate  W25Q16    
 *					0XEF15:Indicate  W25Q32  
 *					0XEF16:Indicate  W25Q64   	 
 * 			    @param void
 *   @return manufaacture id 
 */

static u16 w25qxx_read_id(void)
{
    u16 Temp = 0;	  
    w25qxx_cs_low();				    
    w25qxx_readwrite_byte(W25X_ManufactDeviceID);//发送读取ID命令	    
    w25qxx_readwrite_byte(W25X_NOP_ACTION); 	    
    w25qxx_readwrite_byte(W25X_NOP_ACTION); 	    
    w25qxx_readwrite_byte(W25X_NOP_ACTION); 	 			   
    Temp|=w25qxx_readwrite_byte(W25X_FILL_ACTION)<<8;  
    Temp|=w25qxx_readwrite_byte(W25X_FILL_ACTION);	 
    w25qxx_cs_high();				    
    return Temp;
}  

/*********************************************************************
 * @fn      static void w25qxx_read_content(u8* p_buffer,u32 read_addr,u16 bytes_to_read) 
 
 *
 * @brief   read a specific mount of data from a specific adress   	 
 * 			    @param  u8* p_buffer:  buffer pointer
 *                  u32 read_addr: data start adress(24bit)
 *                  u16 bytes_to_read: (max:65535bytes)
 *                                       
 *   @return manufaacture id 
 */
static void w25qxx_read_content(u8* p_buffer,u32 read_addr,u32 bytes_to_read)   
{ 
    u16 i;   										    
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_ReadData);         //发送读取命令   
    w25qxx_readwrite_byte((u8)((read_addr)>>16));  //发送24bit地址    
    w25qxx_readwrite_byte((u8)((read_addr)>>8));   
    w25qxx_readwrite_byte((u8)read_addr);   
    for(i=0;i<bytes_to_read;i++)
    { 
        p_buffer[i]=w25qxx_readwrite_byte(W25X_FILL_ACTION);   //循环读数  
    }
    w25qxx_cs_high();  				    	      
}  

/*********************************************************************
 * @fn      static void w25qxx_write_page(u8* p_buffer,u32 write_addr,u16 bytes_write)
 
 *
 * @brief   write data less then page size into specific adress   	 
 * 			    @param  u8* p_buffer:  buffer pointer
 *                  u32 write_addr: data start adress(24bit)
 *                  u16 bytes_write: (max:65535bytes) which should less the the amount of left writeable sapce in one page
 *                                        
 *   @return none
 */
static void w25qxx_write_page(u8* p_buffer,u32 write_addr,u16 bytes_write)
{
    u16 i;  
    w25qxx_write_enable();                  //SET WEL 
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_PageProgram);      //发送写页命令   
    w25qxx_readwrite_byte((u8)((write_addr)>>16)); //发送24bit地址    
    w25qxx_readwrite_byte((u8)((write_addr)>>8));   
    w25qxx_readwrite_byte((u8)write_addr);   
    for(i=0;i<bytes_write;i++)w25qxx_readwrite_byte(p_buffer[i]);//循环写数  
    w25qxx_cs_high();                            //取消片选 
    w25qxx_wait_busy();					   //等待写入结束
} 

/*********************************************************************
 * @fn      static void w25qxx_write_nocheck(u8* p_buffer,u32 write_addr,u16 bytes_write)
 
 *
 * @brief   write data into detailed adress changing page automaticly without checking if area is writebale or not 	 
 * 			    @param  u8* p_buffer:  buffer pointer
 *                  u32 write_addr: data start adress(24bit)
 *                  u16 bytes_write: (max:65535bytes) which should less the the amount of left writeable sapce in one page
 *                                       
 *   @return none
 */


static void w25qxx_write_nocheck(u8* p_buffer,u32 write_addr,u16 bytes_write)   
{ 			 		 
    u16 pageremain;	   
    pageremain = (w25qxx_dev->flash_info->page_size) - write_addr%(w25qxx_dev->flash_info->page_size); //单页剩余的字节数		 	    
    if( bytes_write <= pageremain )
    {
        pageremain=bytes_write;//不大于(w25qxx_dev->flash_info->page_size)个字节
    }
    while(1)
    {	   
        w25qxx_write_page(p_buffer,write_addr,pageremain);
        if(bytes_write==pageremain)break;//写入结束了
        else //bytes_write>pageremain
        {
            p_buffer+=pageremain;
            write_addr+=pageremain;	
            
            bytes_write-=pageremain;			  //减去已经写入了的字节数
            if(bytes_write>(w25qxx_dev->flash_info->page_size))pageremain=(w25qxx_dev->flash_info->page_size); //一次可以写入(w25qxx_dev->flash_info->page_size)个字节
            else pageremain=bytes_write; 	  //不够(w25qxx_dev->flash_info->page_size)个字节了
        }
    };	    
} 


/*********************************************************************
 * @fn      static void w25qxx_write_nocheck(u8* p_buffer,u32 write_addr,u16 bytes_write)
 
 *
 * @brief   write data into detailed adress changing page automatically with function of erasing	 
 * 			    @param   u8* p_buffer:  buffer pointer
 *                   u32 write_addr: data start adress(24bit)
 *                   u16 bytes_write: (max:65535bytes) which should less the the amount of left writeable sapce in one page
 *                                       
 *   @return none
 */

static void w25qxx_write_content(u8* p_buffer,u32 write_addr,u32 bytes_write)  
{ 
    
    
#define CHECK_AMOUNT 64
    u32 cycles =0;
    u32 sec_pos =0;
    u16 sec_off =0;
    u16 sec_remain =0;	   
    u32 i =0;    
    u8  spi_flash_buffer[CHECK_AMOUNT+1]={0};	
    u32 buffer_pos=0;   
    
    
    /*calculate sector offset */	
    sec_pos=write_addr/(w25qxx_dev->flash_info->sector_size*SIZE_1K); 
    /*calculate page offset */	
    sec_off=write_addr%(w25qxx_dev->flash_info->sector_size*SIZE_1K);
    /*calculate left space in sector*/	
    sec_remain=(w25qxx_dev->flash_info->sector_size*SIZE_1K)-sec_off;   
    
    /*if bytes need to write is less then one page size which is 4kbyte*/
    if(bytes_write<=sec_remain)
    {
        sec_remain=bytes_write;
    }
    
    /*if offset existed in one sector*/
    if(sec_off > 0)
    {
			
        u32 j;
        u16 sector_offset_temp = sec_off;
        u16 sec_remain_temp = sec_remain;
			
			  DEBUG("ENTER FLASH HEADER OFFSET");
			
        cycles=sec_remain_temp/CHECK_AMOUNT;
        
        for(j=0 ; j<cycles ; j++)
        {
            w25qxx_read_content(spi_flash_buffer,sec_pos*(w25qxx_dev->flash_info->sector_size*SIZE_1K)+sector_offset_temp , CHECK_AMOUNT);
            sector_offset_temp +=CHECK_AMOUNT;
            sec_remain_temp    -=CHECK_AMOUNT;
            for(i=0 ; i<CHECK_AMOUNT ; i++)//校验数据
            {
                if(spi_flash_buffer[i]!=0XFF)
                {
                    w25qxx_erase_sector(sec_pos);//擦除这个扇区
                    goto save_data_head;
                }
            }
        }
        
        if(sec_remain_temp >0 )
        {
            w25qxx_read_content(spi_flash_buffer,sec_pos*(w25qxx_dev->flash_info->sector_size*SIZE_1K)+sector_offset_temp , sec_remain_temp+3);
            for(i=0 ; i<sec_remain_temp ; i++)//校验数据
            {
                if(spi_flash_buffer[i]!=0XFF)
                {
                    w25qxx_erase_sector(sec_pos);//擦除这个扇区
                    goto save_data_head;
                }
            }
            
        }
save_data_head:
         DEBUG("save_data_head");
        w25qxx_write_nocheck(p_buffer+buffer_pos , sec_pos*(w25qxx_dev->flash_info->sector_size*SIZE_1K)+sec_off ,sec_remain);//写入整个扇区
        /*move buffer pointer to next point*/
        buffer_pos+=sec_remain;
        write_addr+=sec_remain;
        bytes_write-=sec_remain;				//字节数递减
        /*mov sector positon to next sector*/
        sec_pos ++;
        sec_off = 0;
        sec_remain =(w25qxx_dev->flash_info->sector_size*SIZE_1K)-sec_off;
    }
    
    /*full sector checkout*/		
    while((bytes_write/(w25qxx_dev->flash_info->sector_size*SIZE_1K))>0)
    {	
        
			  DEBUG("full sector checkout");
			
        u32 j;
        u16 sector_offset_temp = sec_off;
        cycles=sec_remain/CHECK_AMOUNT; 
        for(j=0 ; j<cycles ; j++)
        {
            w25qxx_read_content(spi_flash_buffer,sec_pos*(w25qxx_dev->flash_info->sector_size*SIZE_1K)+sector_offset_temp , CHECK_AMOUNT);
            sector_offset_temp +=CHECK_AMOUNT;
            sec_remain -=CHECK_AMOUNT;
            for(i=0 ; i<CHECK_AMOUNT ; i++)//校验数据
            {
                if(spi_flash_buffer[i]!=0XFF)
                {
                    w25qxx_erase_sector(sec_pos);//擦除这个扇区
                    goto save_data_body;
                }
            }
        }
        
save_data_body:
        DEBUG("save_data_body");
        w25qxx_write_nocheck(p_buffer+buffer_pos , sec_pos*(w25qxx_dev->flash_info->sector_size*SIZE_1K) ,(w25qxx_dev->flash_info->sector_size*SIZE_1K));//写入整个扇区
        /*move buffer pointer to next point*/
        buffer_pos+=(w25qxx_dev->flash_info->sector_size*SIZE_1K);
        write_addr+=(w25qxx_dev->flash_info->sector_size*SIZE_1K);
        bytes_write-=(w25qxx_dev->flash_info->sector_size*SIZE_1K);				//字节数递减		
        /*mov sector positon to next sector*/
				DEBUG("sector %d saved",sec_pos);
        sec_pos ++;
        sec_off = 0;
        sec_remain =(w25qxx_dev->flash_info->sector_size*SIZE_1K)-sec_off;
        
        
        
        
    }	
		
    /*tail sector checkout*/	
    if(bytes_write>0)
    {
        u32 j;
        u16 sector_offset_temp = sec_off;
        sec_remain =bytes_write;
        cycles=sec_remain/CHECK_AMOUNT;
        
        for(j=0 ; j<cycles ; j++)
        {
            w25qxx_read_content(spi_flash_buffer,sec_pos*(w25qxx_dev->flash_info->sector_size*SIZE_1K)+sector_offset_temp , CHECK_AMOUNT);
            sector_offset_temp +=CHECK_AMOUNT;
            sec_remain -=CHECK_AMOUNT;
            for(i=0 ; i<CHECK_AMOUNT ; i++)//校验数据
            {
                if(spi_flash_buffer[i]!=0XFF)
                {
                    w25qxx_erase_sector(sec_pos);//擦除这个扇区
                    goto save_data_tail;
                }
            }
        }
        
        if(sec_remain >0 )
        {
            w25qxx_read_content(spi_flash_buffer,sec_pos*(w25qxx_dev->flash_info->sector_size*SIZE_1K)+sector_offset_temp , sec_remain);
            for(i=0 ; i<sec_remain ; i++)//校验数据
            {
                if(spi_flash_buffer[i]!=0XFF)
                {
                    w25qxx_erase_sector(sec_pos);//擦除这个扇区
                    goto save_data_head;
                }
            }
            
        }
        
save_data_tail:
        
        sec_remain = bytes_write;  
        w25qxx_write_nocheck(p_buffer+buffer_pos , sec_pos*(w25qxx_dev->flash_info->sector_size*SIZE_1K) ,sec_remain);//写入整个扇区
        /*move buffer pointer to next point*/
        buffer_pos+=sec_remain;
        write_addr+=sec_remain;
        bytes_write-=sec_remain;				//字节数递减
        /*mov sector positon to next sector*/
        sec_pos ++;
        sec_off = 0;
        sec_remain =(w25qxx_dev->flash_info->sector_size*SIZE_1K)-sec_off;
        
        
    }
    
    
}

/*********************************************************************
 * @fn     static void w25qxx_erase_chip(void)
 
 *
 * @brief   erase full chip	 
 * 			    @param  voif
 
 *          @return nonr
 */
static void w25qxx_erase_chip(void)  
{                                   
    w25qxx_write_enable();                  //SET WEL 
    w25qxx_wait_busy();   
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_ChipErase);        //发送片擦除命令  
    w25qxx_cs_high();                            //取消片选     	      
    w25qxx_wait_busy();   				   //等待芯片擦除结束
}  

/*********************************************************************
 * @fn     static void w25qxx_erase_sector(u32 dst_addr)  
 
 *
 * @brief   erase a specific sector 
 * 			    @param  u32 dst_sector : sector need erasing (at lesr 150 ms per sector)
 
 *          @return none
 */

static void w25qxx_erase_sector(u32 dst_sector) 
{  
    //监视falsh擦除情况,测试用   
    DEBUG("sector %x erased",dst_sector); 
    dst_sector*=(w25qxx_dev->flash_info->sector_size*SIZE_1K);
    w25qxx_write_enable();                  //SET WEL 	 
    w25qxx_wait_busy();   
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_SectorErase);      //发送扇区擦除指令 
    w25qxx_readwrite_byte((u8)((dst_sector)>>16));  //发送24bit地址    
    w25qxx_readwrite_byte((u8)((dst_sector)>>8));   
    w25qxx_readwrite_byte((u8)dst_sector);  
    w25qxx_cs_high();                            //取消片选     	      
    w25qxx_wait_busy();   				   //等待擦除完成
}  


/*********************************************************************
 * @fn     static void w25qxx_wait_busy(void)  
 
 *
 * @brief   wait while is chip is busy
 * 			    @param  none
 
 *          @return none
 */

static void w25qxx_wait_busy(void)  
{   
    while((w25qxx_read_sr()&0x01)==0x01);   // 等待BUSY位清空
}  


/*********************************************************************
 * @fn     static void spi_flash_powerdown(void)   
 
 *
 * @brief   make flash enter power saving mode
 * 			    @param  none
 
 *          @return none
 */
static void spi_flash_powerdown(void)  
{ 
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_PowerDown);        //发送掉电命令  
    w25qxx_cs_high();                            //取消片选     	      
    // delay_us(3);                               //等待TPD  
}  

/*********************************************************************
 * @fn    static void w25qxx_flash_wakeup(void)    
 
 *
 * @brief   wake flash up from sleepping
 * 			    @param  none
 
 *          @return none
 */
static void w25qxx_flash_wakeup(void)   
{  
    w25qxx_cs_low();                            //使能器件   
    w25qxx_readwrite_byte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
    w25qxx_cs_high();                            //取消片选     	      
    //delay_us(3);                               //等待TRES1
}   


/*********************************************************************
 * @fn    os_err_t _flash_read(u32 adress , u32 lenth)   
 
 *
 * @brief   api function for shell
 * 			    @param  none
 *
 *          @return none
 */

os_err_t _flash_read(u32 adress , u32 lenth)
{

	u8 buffer[CHECK_AMOUNT];
	u32 i=0 ,j=0;
	u32 cycles = lenth/CHECK_AMOUNT;
	
	if(lenth%CHECK_AMOUNT > 0)
	{
		cycles++;	
	}
	
	for(i=0 ; i<cycles ; i++ )
	{
	  w25qxx_read_content(buffer, adress ,CHECK_AMOUNT);
	  for(j =0; j<CHECK_AMOUNT ; j++)
		{			
			if(j%16 == 0)
			{
					printf("\r\n");
				  printf("0X%08X: ",adress+j);
			}
			printf("%02X ",buffer[j]);
		}	
		adress +=CHECK_AMOUNT;
		lenth  -=CHECK_AMOUNT;
		
	}
	return  SUCCESS;
}


/*********************************************************************
 * @fn    _flash_cp(u32 adress , u32 lenth)   
 
 *
 * @brief   copy data from sram to flash
 * 			    @param  none
 *
 *          @return none
 */

os_err_t _flash_cp(u32 ram_adrr , u32 flash_adress , u32 lenth)
{


	w25qxx_write_content(((u8*)ram_adrr) , flash_adress, lenth) ;
	
	return  SUCCESS;
}




















