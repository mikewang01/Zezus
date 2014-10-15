
/************************************************************
Copyright (C), 2008-2014, Colorful Sea Tech. Co., Ltd.
FileName:       EEPROM.c
Author:      MikeWang   Version : 0.0          Date: 2014/8/18
Description:      Provide interface of eeprom to up-level software      
Version:  0.0        
Function List:   


1. -------History:         
<author>   <time>    <version >    <desc>
Mike      14/8/18      0.0       build this moudle  
***********************************************************/

/** @addtogroup STM32F0xx_EEPROM_Emulation
  * @{
  */ 

/* Includes ------------------------------------------------------------------*/
#include "kernel-includes.h"
#include "drivers-includes.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static eeprom_device *eeprom_simulation=NULL;
/* Global variable used to store variable value in read sequence */
u16 DataVar = 0;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
u16 VirtAddVarTab[NB_OF_VAR];

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/* system related functions*/
static  os_err_t   eeprom_init   (os_device_t* dev);

static  os_size_t  eeprom_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size);

static  os_size_t  eeprom_read(os_device_t* dev, os_off_t pos, void *buffer, os_size_t size);

static  os_err_t   eeprom_open(os_device_t* dev, u16 oflag);

static  os_err_t   eeprom_close(os_device_t* dev);

/*driver related functions*/
static u16 EE_Init(void);
static u16 EE_ReadVariable(u16 VirtAddress, u16* Data);
static u16 EE_WriteVariable(u16 VirtAddress, u16 Data);
static FLASH_Status EE_Format(void);
static u16 EE_VerifyPageFullWriteVariable(u16 VirtAddress, u16 Data);
static u16 EE_PageTransfer(u16 VirtAddress, u16 Data);
static u16 EE_FindValidPage(u8 Operation);


/*********************************************************************
 * @fn     os_err_t usart1_register(u16 task_id)
 *
 * @brief   This function initiate  device structor and insert into manage chain
 *
 * 			    @param task_id
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
#define 	EEPROM_NAME "EEPROM"

os_err_t eeprom_register(u16 task_id)
{
    /*allocate specific amount memory*/
    eeprom_simulation=osmalloc(sizeof(usart_device));
    /*if memory alloated failed */
    if(eeprom_simulation == NULL)
    {
        

        DEBUG("eeprom allocated failed");

        return ERROR;
        
    }
    
    eeprom_simulation->pclk2=SYSCLK;
    
    eeprom_simulation->bound=BOUND;
    
    /*initilize device type*/
    eeprom_simulation->os_device.type = OS_Device_Class_Char;
    
    /*registor device id*/
    eeprom_simulation->os_device.device_id = OS_DEVICE_USART_ID;
    
    /*registor task id*/
    eeprom_simulation->register_taskid = task_id;
    
    /*initiate the function poniter to the device structor*/
    eeprom_simulation->os_device.init  = eeprom_init;
    
    /*open function poniter initialized*/
    eeprom_simulation->os_device.open  = eeprom_open;
    
    /*close function poniter initialized*/
    eeprom_simulation->os_device.close  = eeprom_close;
    
    /*write function poniter initialized*/
    eeprom_simulation->os_device.write = eeprom_write;
    
    /*read function poniter initialized*/
    eeprom_simulation->os_device.read  = eeprom_read; 
    
    /*return registor status*/
    return os_device_register(&(eeprom_simulation->os_device), EEPROM_NAME, OS_DEVICE_FLAG_INACTIVATED);
}



/*********************************************************************
 * @fn    static  os_err_t   eeprom_init   (os_device_t* dev);
 *
 * @brief   This function initiate  eeprom resistors
 *
 * 			    @param task_id
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
static  os_err_t   eeprom_init   (os_device_t* dev)
{
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock();
    /* EEPROM Init */
    EE_Init();
    
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
static os_err_t  eeprom_open(os_device_t* dev, u16 oflag)
{
    
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock();
    
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
static os_err_t  eeprom_close(os_device_t* dev)
{
    
    /* Unlock the Flash Program Erase controller */
    FLASH_Lock();
    
    return SUCCESS;		
    
}


/*********************************************************************
 * @fn    static  os_size_t  eeprom_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
 *
 * @brief   This function  flush buffer into specific adress in dev 
 *
 * 			    @param  os_device_t* dev   : device pointer
 *                   os_off_t pos      : position offset
 *                  const void *buffer : buffer stored with data
 *                  os_size_t size     : num of buffer needed to be flushed in bytes
 *			
 * 
 *   @return the error code, success on initialization successfully. 
 */
static  os_size_t  eeprom_write(os_device_t* dev, os_off_t pos, const void *buffer, os_size_t size)
{
    
    u16 data_num     = 0;
    u16 write_status = 0;
    u16 size_16bit   = size/sizeof(u16);
    u16 buffer_temp[50]={0};
    
    /*copy data from user erea to kernel eara*/
    osmemcpy((u8*)buffer_temp , (u8*)buffer ,size);
    
    /*if size is a odd number then icn size_16bit*/
    if(size_16bit*sizeof(u16) < size)
    {
        size_16bit++;
    }
    
    if(pos>=size || buffer==NULL)
    {
        return  NULL;
    }else
    {
        for ( ; data_num < size_16bit ; data_num++ )
        {
            /*flush buffer in to eeprom in position in accodance with pos*/
            write_status = EE_WriteVariable( pos + data_num , ((buffer_temp)[data_num]));
            
            /*write error happens return 0*/
            if(write_status != FLASH_COMPLETE)
            {

                DEBUG("EEPROM write failed");  

                return 0;
            }
            
            
        }
        
        
    }
    
    /*read successfully*/	
    return (data_num+1)*sizeof(u16);
}

/*********************************************************************
 * @fn      eeprom_read
 *
 * @brief   This function read data from specific position indicated by pos from eeprom dev
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data ;ignored here
*									 buffer: data need tansfering ;  attention::data is stored in 16 bits mode  
* 								 size: data size tansfered in bytes
 *   @return number of data readed
 */

os_size_t eeprom_read   (os_device_t* dev, os_off_t pos, void *buffer, os_size_t size)
{
    
    u16 data_num = 0;
    u16 read_status = SUCCESS;
    u16 size_16bit   = size/sizeof(u16);
    u16 buffer_temp[50]={0};
    
    /*if size is a odd number then icn size_16bit*/
    if(size_16bit*sizeof(u16) < size)
    {
        size_16bit++;
    }
    
    if(dev==NULL)
    {
        return ERROR;
    }
    
    for(data_num =0 ; data_num < size_16bit ; data_num++)
    {
        /*read data form specific position*/
        read_status = EE_ReadVariable(pos + data_num , buffer_temp + data_num);
        
        /*if read error occurs return 0*/
        if(read_status == ERROR || read_status == NO_VALID_PAGE )
        {
					

					
					  DEBUG("EEPROM Read failed");

            return 0;
        }
    }	  
    
    /*copy data from kernel eara to user erea */
    osmemcpy((u8*)buffer , (u8*)buffer_temp ,size);
    
		/*greater or equal than size*/
    return data_num*sizeof(u16) ;	
    
}

/**
  * @brief  Restore the pages to a known good state in case of page's status
  *   corruption after a power loss.
  * @param  None.
  * @retval - Flash error code: on write Flash error
  *         - FLASH_COMPLETE: on success
  */
static u16 EE_Init(void)
{
    u16 PageStatus0 = 6, PageStatus1 = 6;
    u16 VarIdx = 0;
    u16 EepromStatus = 0, ReadStatus = 0;
    int16_t x = -1;
    u16  FlashStatus;
    
    /* Get Page0 status */
    PageStatus0 = (*(__IO u16*)PAGE0_BASE_ADDRESS);
    /* Get Page1 status */
    PageStatus1 = (*(__IO u16*)PAGE1_BASE_ADDRESS);
    
    /* Check for invalid header states and repair if necessary */
    switch (PageStatus0)
    {
    case ERASED:
        if (PageStatus1 == VALID_PAGE) /* Page0 erased, Page1 valid */
        {
            /* Erase Page0 */
            FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDRESS);
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        else if (PageStatus1 == RECEIVE_DATA) /* Page0 erased, Page1 receive */
        {
            /* Erase Page0 */
            FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDRESS);
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
            /* Mark Page1 as valid */
            FlashStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        else /* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
        {
            /* Erase both Page0 and Page1 and set Page0 as valid page */
            FlashStatus = EE_Format();
            /* If erase/program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        break;
        
    case RECEIVE_DATA:
        if (PageStatus1 == VALID_PAGE) /* Page0 receive, Page1 valid */
        {
            /* Transfer data from Page1 to Page0 */
            for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++)
            {
                if (( *(__IO u16*)(PAGE0_BASE_ADDRESS + 6)) == VirtAddVarTab[VarIdx])
                {
                    x = VarIdx;
                }
                if (VarIdx != x)
                {
                    /* Read the last variables' updates */
                    ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
                    /* In case variable corresponding to the virtual address was found */
                    if (ReadStatus != 0x1)
                    {
                        /* Transfer the variable to the Page0 */
                        EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
                        /* If program operation was failed, a Flash error code is returned */
                        if (EepromStatus != FLASH_COMPLETE)
                        {
                            return EepromStatus;
                        }
                    }
                }
            }
            /* Mark Page0 as valid */
            FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
            /* Erase Page1 */
            FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDRESS);
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        else if (PageStatus1 == ERASED) /* Page0 receive, Page1 erased */
        {
            /* Erase Page1 */
            FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDRESS);
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
            /* Mark Page0 as valid */
            FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        else /* Invalid state -> format eeprom */
        {
            /* Erase both Page0 and Page1 and set Page0 as valid page */
            FlashStatus = EE_Format();
            /* If erase/program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        break;
        
    case VALID_PAGE:
        if (PageStatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
        {
            /* Erase both Page0 and Page1 and set Page0 as valid page */
            FlashStatus = EE_Format();
            /* If erase/program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        else if (PageStatus1 == ERASED) /* Page0 valid, Page1 erased */
        {
            /* Erase Page1 */
            FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDRESS);
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        else /* Page0 valid, Page1 receive */
        {
            /* Transfer data from Page0 to Page1 */
            for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++)
            {
                if ((*(__IO u16*)(PAGE1_BASE_ADDRESS + 6)) == VirtAddVarTab[VarIdx])
                {
                    x = VarIdx;
                }
                if (VarIdx != x)
                {
                    /* Read the last variables' updates */
                    ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
                    /* In case variable corresponding to the virtual address was found */
                    if (ReadStatus != 0x1)
                    {
                        /* Transfer the variable to the Page1 */
                        EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
                        /* If program operation was failed, a Flash error code is returned */
                        if (EepromStatus != FLASH_COMPLETE)
                        {
                            return EepromStatus;
                        }
                    }
                }
            }
            /* Mark Page1 as valid */
            FlashStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
            /* Erase Page0 */
            FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDRESS);
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
        }
        break;
        
    default:  /* Any other state -> format eeprom */
        /* Erase both Page0 and Page1 and set Page0 as valid page */
        FlashStatus = EE_Format();
        /* If erase/program operation was failed, a Flash error code is returned */
        if (FlashStatus != FLASH_COMPLETE)
        {
            return FlashStatus;
        }
        break;
    }
    
    return FLASH_COMPLETE;
}

/**
  * @brief  Returns the last stored variable data, if found, which correspond to
  *   the passed virtual address
  * @param  VirtAddress: Variable virtual address
  * @param  Data: Global variable contains the read variable value
  * @retval Success or error status:
  *           - sucess: if variable was found
  *           - error: if the variable was not found
  *           - NO_VALID_PAGE: if no valid page was found.
  */
static u16 EE_ReadVariable(u16 VirtAddress, u16* Data)
{
    u16 ValidPage = PAGE0;
    u16 AddressValue = 0x5555, ReadStatus = ERROR;
    u32 Address = 0x08010000, PageStartAddress = 0x08010000;
    
    /* Get active Page for read operation */
    ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);
    
    /* Check if there is no valid page */
    if (ValidPage == NO_VALID_PAGE)
    {
        return  NO_VALID_PAGE;
    }
    
    /* Get the valid Page start Address */
    PageStartAddress = (u32)(EEPROM_START_ADDRESS + (u32)(ValidPage * PAGE_SIZE));
    
    /* Get the valid Page end Address */
    Address = (u32)((EEPROM_START_ADDRESS - 2) + (u32)((1 + ValidPage) * PAGE_SIZE));
    
    /* Check each active page address starting from end */
    while (Address > (PageStartAddress + 2))
    {
        /* Get the current location content to be compared with virtual address */
        AddressValue = (*(__IO u16*)Address);
        
        /* Compare the read address with the virtual address */
        if (AddressValue == VirtAddress)
        {
            /* Get content of Address-2 which is variable value */
            *Data = (*(__IO u16*)(Address - 2));
            
            /* In case variable value is read, reset ReadStatus flag */
            ReadStatus = SUCCESS;
            
            break;
        }
        else
        {
            /* Next address location */
            Address = Address - 4;
        }
    }
    
    /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
    return ReadStatus;
}

/**
  * @brief  Writes/upadtes variable data in EEPROM.
  * @param  VirtAddress: Variable virtual address
  * @param  Data: 16 bit data to be written
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
static u16 EE_WriteVariable(u16 VirtAddress, u16 Data)
{
    u16 Status = 0;
    
    /* Write the variable virtual address and value in the EEPROM */
    Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
    
    /* In case the EEPROM active page is full */
    if (Status == PAGE_FULL)
    {
        /* Perform Page transfer */
        Status = EE_PageTransfer(VirtAddress, Data);
    }
    
    /* Return last operation status */
    return Status;
}

/**
  * @brief  Erases PAGE0 and PAGE1 and writes VALID_PAGE header to PAGE0
  * @param  None
  * @retval Status of the last operation (Flash write or erase) done during
  *         EEPROM formating
  */
static FLASH_Status EE_Format(void)
{
    FLASH_Status FlashStatus = FLASH_COMPLETE;
    
    /* Erase Page0 */
    FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDRESS);
    
    /* If erase operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE)
    {
        return FlashStatus;
    }
    
    /* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */
    FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
    
    /* If program operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE)
    {
        return FlashStatus;
    }
    
    /* Erase Page1 */
    FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDRESS);
    
    /* Return Page1 erase operation status */
    return FlashStatus;
}

/**
  * @brief  Find valid Page for write or read operation
  * @param  Operation: operation to achieve on the valid page.
  *   This parameter can be one of the following values:
  *     @arg READ_FROM_VALID_PAGE: read operation from valid page
  *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
  * @retval Valid page number (PAGE0 or PAGE1) or NO_VALID_PAGE in case
  *   of no valid page was found
  */
static u16 EE_FindValidPage(u8 Operation)
{
    u16 PageStatus0 = 6, PageStatus1 = 6;
    
    /* Get Page0 actual status */
    PageStatus0 = (*(__IO u16*)PAGE0_BASE_ADDRESS);
    
    /* Get Page1 actual status */
    PageStatus1 = (*(__IO u16*)PAGE1_BASE_ADDRESS);
    
    /* Write or read operation */
    switch (Operation)
    {
    case WRITE_IN_VALID_PAGE:   /* ---- Write operation ---- */
        if (PageStatus1 == VALID_PAGE)
        {
            /* Page0 receiving data */
            if (PageStatus0 == RECEIVE_DATA)
            {
                return PAGE0;         /* Page0 valid */
            }
            else
            {
                return PAGE1;         /* Page1 valid */
            }
        }
        else if (PageStatus0 == VALID_PAGE)
        {
            /* Page1 receiving data */
            if (PageStatus1 == RECEIVE_DATA)
            {
                return PAGE1;         /* Page1 valid */
            }
            else
            {
                return PAGE0;         /* Page0 valid */
            }
        }
        else
        {
            return NO_VALID_PAGE;   /* No valid Page */
        }
        
    case READ_FROM_VALID_PAGE:  /* ---- Read operation ---- */
        if (PageStatus0 == VALID_PAGE)
        {
            return PAGE0;           /* Page0 valid */
        }
        else if (PageStatus1 == VALID_PAGE)
        {
            return PAGE1;           /* Page1 valid */
        }
        else
        {
            return NO_VALID_PAGE ;  /* No valid Page */
        }
        
    default:
        return PAGE0;             /* Page0 valid */
    }
}

/**
  * @brief  Verify if active page is full and Writes variable in EEPROM.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
static u16 EE_VerifyPageFullWriteVariable(u16 VirtAddress, u16 Data)
{
    FLASH_Status FlashStatus = FLASH_COMPLETE;
    u16 ValidPage = PAGE0;
    u32 Address = 0x08010000, PageEndAddress = 0x080107FF;
    
    /* Get valid Page for write operation */
    ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);
    
    /* Check if there is no valid page */
    if (ValidPage == NO_VALID_PAGE)
    {
        return  NO_VALID_PAGE;
    }
    
    /* Get the valid Page start Address */
    Address = (u32)(EEPROM_START_ADDRESS + (u32)(ValidPage * PAGE_SIZE));
    
    /* Get the valid Page end Address */
    PageEndAddress = (u32)((EEPROM_START_ADDRESS - 2) + (u32)((1 + ValidPage) * PAGE_SIZE));
    
    /* Check each active page address starting from begining */
    while (Address < PageEndAddress)
    {
        /* Verify if Address and Address+2 contents are 0xFFFFFFFF */
        if ((*(__IO u32*)Address) == 0xFFFFFFFF)
        {
            /* Set variable data */
            FlashStatus = FLASH_ProgramHalfWord(Address, Data);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE)
            {
                return FlashStatus;
            }
            /* Set variable virtual address */
            FlashStatus = FLASH_ProgramHalfWord(Address + 2, VirtAddress);
            /* Return program operation status */
            return FlashStatus;
        }
        else
        {
            /* Next address location */
            Address = Address + 4;
        }
    }
    
    /* Return PAGE_FULL in case the valid page is full */
    return PAGE_FULL;
}

/**
  * @brief  Transfers last updated variables data from the full Page to
  *   an empty one.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
static u16 EE_PageTransfer(u16 VirtAddress, u16 Data)
{
    FLASH_Status FlashStatus = FLASH_COMPLETE;
    u32 NewPageAddress = 0x080103FF, OldPageAddress = 0x08010000;
    u16 ValidPage = PAGE0, VarIdx = 0;
    u16 EepromStatus = 0, ReadStatus = 0;
    
    /* Get active Page for read operation */
    ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);
    
    if (ValidPage == PAGE1)       /* Page1 valid */
    {
        /* New page address where variable will be moved to */
        NewPageAddress = PAGE0_BASE_ADDRESS;
        
        /* Old page address where variable will be taken from */
        OldPageAddress = PAGE1_BASE_ADDRESS;
    }
    else if (ValidPage == PAGE0)  /* Page0 valid */
    {
        /* New page address where variable will be moved to */
        NewPageAddress = PAGE1_BASE_ADDRESS;
        
        /* Old page address where variable will be taken from */
        OldPageAddress = PAGE0_BASE_ADDRESS;
    }
    else
    {
        return NO_VALID_PAGE;       /* No valid Page */
    }
    
    /* Set the new Page status to RECEIVE_DATA status */
    FlashStatus = FLASH_ProgramHalfWord(NewPageAddress, RECEIVE_DATA);
    /* If program operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE)
    {
        return FlashStatus;
    }
    
    /* Write the variable passed as parameter in the new active page */
    EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
    /* If program operation was failed, a Flash error code is returned */
    if (EepromStatus != FLASH_COMPLETE)
    {
        return EepromStatus;
    }
    
    /* Transfer process: transfer variables from old to the new active page */
    for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++)
    {
        if (VirtAddVarTab[VarIdx] != VirtAddress)  /* Check each variable except the one passed as parameter */
        {
            /* Read the other last variable updates */
            ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
            /* In case variable corresponding to the virtual address was found */
            if (ReadStatus != 0x1)
            {
                /* Transfer the variable to the new active page */
                EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
                /* If program operation was failed, a Flash error code is returned */
                if (EepromStatus != FLASH_COMPLETE)
                {
                    return EepromStatus;
                }
            }
        }
    }
    
    /* Erase the old Page: Set old Page status to ERASED status */
    FlashStatus = FLASH_ErasePage(OldPageAddress);
    /* If erase operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE)
    {
        return FlashStatus;
    }
    
    /* Set new Page status to VALID_PAGE status */
    FlashStatus = FLASH_ProgramHalfWord(NewPageAddress, VALID_PAGE);
    /* If program operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE)
    {
        return FlashStatus;
    }
    
    /* Return last operation flash status */
    return FlashStatus;
}

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
