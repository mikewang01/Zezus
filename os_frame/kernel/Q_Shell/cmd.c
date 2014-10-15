#include "Q_Shell.h"
#include "string.h"
#include "stdio.h"
#include "kernel-includes.h"

unsigned char  crc16_on_off=0;
unsigned short Var2;
unsigned int   Var3;

QSH_VAR_REG(crc16_on_off,"crc16_on_off on:0 off:1","u8");
QSH_VAR_REG(Var2,"unsigned char  Var2","u16");
QSH_VAR_REG(Var3,"unsigned char  Var3","u32");

/*********************************************************************
 * @fn      Add()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
unsigned int Add(unsigned a,unsigned b)
{
    return (a+b);
}

QSH_FUN_REG(Add, "unsigned int Add(unsigned a,unsigned b)");
/*********************************************************************
 * @fn      PutString()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
unsigned int PutString(char *Str)
{
    if(*Str=='\0')
        return 0;
    printf("%s\r\n",Str);
    return 1;
}

QSH_FUN_REG(PutString, "unsigned int PutString(char *Str)");

/*********************************************************************
 * @fn      list_timer()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
extern os_err_t _list_timer(void);
long list_timer(void)
{
    return _list_timer();
}
QSH_FUN_REG(list_timer, "long list_timer()");

/*********************************************************************
 * @fn      list_timer()
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
extern os_err_t _list_device(void);
long list_device(void)
{
    return _list_device();
}
QSH_FUN_REG(list_device, "long list_device()");


/*********************************************************************
 * @fn      relay_control()
 *
 * @brief   This function expose operation to the shell.
 *
 * @param   void
 *
 * @return  none
 */
extern os_err_t _relay_control(u8 relay_no,u8 cmd);
os_err_t relay_control(u8 relay_no,u8 cmd)
{
    
    _relay_control(relay_no,cmd);
    return 0;
}
QSH_FUN_REG(relay_control, "os_err_t relay_control(u8 relay_no,u8 cmd) relayno:1~4;cmd 1:open 0 closed");


/*********************************************************************
 * @fn      _list_mem()
 *
 * @brief   This function expose operation to the shell.
 *
 * @param   void
 *
 * @return  none
 */
extern os_err_t _list_mem(void);
os_err_t list_mem(void)
{
    
    _list_mem();
    return 0;
}
QSH_FUN_REG(list_mem, "os_err_t list_mem()");


/*********************************************************************
 * @fn      _mem_read(u32 adress , u32 lenth)
 *
 * @brief   This function expose operation to the shell.
 *
 * @param   void
 *
 * @return  none
 */
extern os_err_t _mem_read(u32 adress , u32 lenth);
os_err_t mem_read(u32 adress , u32 lenth)
{
    
    _mem_read(adress , lenth);
    return 0;
}
QSH_FUN_REG(mem_read, "os_err_t mem_read(u32 adress , u32 lenth)");
/*********************************************************************
 * @fn      _flash_read()
 *
 * @brief   This function expose operation to the shell.
 *
 * @param   void
 *
 * @return  none
 */
extern os_err_t _flash_read(u32 adress , u32 lenth);
os_err_t flash_read(u32 adress , u32 lenth)
{
    _flash_read(adress , lenth);
    return 0;
	
}
QSH_FUN_REG(flash_read, "os_err_t flash_read(u32 adress , u32 lenth)");


/*********************************************************************
 * @fn      _flash_read()
 *
 * @brief   This function is implented to read data from ram to flash.
 *
 * @param   void
 *
 * @return  none
 */
extern os_err_t _flash_cp(u32 ram_adrr , u32 flash_adress , u32 lenth);
os_err_t flash_cp(u32 ram_adrr , u32 flash_adress , u32 lenth)
{
    _flash_cp( ram_adrr ,  flash_adress ,  lenth);
    return 0;
	
}
QSH_FUN_REG(flash_cp, "os_err_t flash_cp(u32 ram_adrr , u32 flash_adress , u32 lenth)");





