/***************************************************************
 * Name:      LENTHMEASURE.c
 * Purpose:   code for lenth sample  to store and send
 * Author:    mikewang(s)
 * Created:   2014-05-15
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */
 #include "kernel-includes.h"
#include "drivers-includes.h"
#include "me2_ch2o.h"
#include <stm32f0xx.h>
#include "stm32f0xxlib_conf.h"
#include "stdio.h"
#include "stm32f0xx_spi.h"
/*********************************************************************
* MACROS
*/
#define  ADC_SENSOR_PORT     GPIOA
#define  ADC_SENSOR_PIN      6
#define  ADC_SENSOR_CHANNEL  ADC_CHSELR_CHSEL6


#define  ADC_REF_PORT        GPIOA
#define  ADC_REF_PIN         7
#define  ADC_REF_CHANNEL     ADC_CHSELR_CHSEL7


#define  SENSOR_POWER_CTL_PORT GPIOA
#define  SENSOR_POWER_CTL_PIN   4

#define  SENSOR_SHORT_CTL_PORT GPIOA
#define  SENSOR_SHORT_CTL_PIN   5	


#define CH2O_DMA_NUM  DMA1
#define CH2O_DMA_CHANNEL  DMA1_Channel1

#define ADCBUF_SIZE 6

/*smoth filter liner lentt*/
#define SMOOTH_FILTER_LENTH  10

/*Y = a +b*x*/
#define DEFUALT_PAR_A  (-10860)
#define DEFUALT_PAR_B  408


#define ABS(x)  ( (x)>0?(x):-(x) )
#define SENSOR_REF_VOL  1230
#define ADC_REF_VOL     2525
/*********************************************************************
 * MACROS
 */

/*start adress stroring the struct data*/
#define  CORRECTION_PARA_START_ADRRESS   0X02
#define  CORRECTION_PARA_NUM             (sizeof(correction_para))

/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */

static  adc_device *adc_dev = NULL;

static  correction_para correct_coefficient = {0};


/*adc sample buffer for dma*/
static u16  RegularConvData_Tab[ADCBUF_SIZE] = {0};
static u16  smooth_filter_tab[SMOOTH_FILTER_LENTH] = {0};

static int  zero_density_voltage = 0;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * local FUNCTIONS decalaratio
 */
static os_err_t  	me2_ch20_init   (os_device_t* dev);

static os_err_t  me2_ch20_control(os_device_t* dev, u8 cmd, void *args);

os_size_t me2_ch20_read   (os_device_t* dev, os_off_t pos, void *buffer, os_size_t size);


static os_err_t adc1_dma_init(void);

static os_err_t me2_ch20_ctlpin_init(void);

static os_err_t set_me2_ch20_state(u8 operation);

static u16 adc_avr_cal(void);

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

#define  SENSOR_DEVICE_NAME  "me2_ch2o"

os_err_t me2_ch20_register(u16 task_id)
{
    adc_dev = (adc_device*)osmalloc(sizeof(adc_device));
    
    
    adc_dev->os_device.type= OS_Device_Class_Char;
    
    adc_dev->os_device.device_id=OS_DEVICE_KEY_ID;
    
    adc_dev->register_taskid = task_id;
    
    adc_dev->os_device.init = me2_ch20_init;
    
    adc_dev->os_device.open = NULL;
    /*set write ptr to NULL*/
    adc_dev->os_device.write   = NULL;
    adc_dev->os_device.read    = me2_ch20_read;
    adc_dev->os_device.control = me2_ch20_control;
    
    return os_device_register(&(adc_dev->os_device), SENSOR_DEVICE_NAME, OS_DEVICE_FLAG_INACTIVATED);
    
}

/*********************************************************************
 * @fn      os_err-t 	me2_ch20_init   (os_device_t* dev)
 *
 * @brief   initiate sensor system invoking adc1 dma1 gpio
 * 			   
 *			
 * 
 *   @return none 
 */	

static os_err_t  	me2_ch20_init   (os_device_t* dev)
{
    
    os_clock_open("GPIOA"); 
    os_clock_open("GPIOB");
	
	  osmemset(smooth_filter_tab , 0 , SMOOTH_FILTER_LENTH*sizeof(u16));
	
    /*init hardware system*/
    adc1_dma_init();
    me2_ch20_ctlpin_init();
	
	  /*intiate correction parameter*/
    correct_coefficient.para_a = 0;
	  correct_coefficient.para_b = 0;
	  zero_density_voltage = 0;
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

static os_err_t  me2_ch20_control(os_device_t* dev, u8 cmd, void *args)
{
    
    if(dev==NULL)
    {
        return ERROR;
    }
    switch(cmd)
    {
    case get_correction_para:   
        /*set the fist byte to the amount the packet contains*/
        ((u8*)args)[0] = sizeof(correction_para);
        /*copy reqired data then return */
        os_memmove((u8*)args+1 , &correct_coefficient , ((u8*)args)[0] );
        break;
    case set_correction_para:  
    {   
			  correction_para temp;//= (correction_para*)(((u8*)args)+1);
        u8 data_num = ((u8*)args)[0];
			  
			  /*make sure the parameter  legal*/
		    EXAM_ASSERT(data_num <= sizeof(correction_para));	
			
        os_memmove(&temp , (((u8*)args)+1) , data_num);
			
        if(data_num != sizeof(correction_para))
        {
            return ERROR;
        }
        
        /*if coeeficient stroed is the same as th elast one this is no needed to  updade */
        if(temp.para_a ==correct_coefficient.para_a &&  temp.para_b ==correct_coefficient.para_b) 
        {
            return SUCCESS;
        }else /*refresh current coefficeint and store in to eeprom*/
        {
            
            
            os_device_t *dev = os_device_get("EEPROM"); 
            
					  /*update current coefficence*/
            correct_coefficient = temp;
            /*check  if opened successfully*/
            if(dev == NULL)
            {
							
                DEBUG("EEPROM open failed");
					
                return ERROR;
            }
            
            /*store data into specific adress failed*/
            if(os_device_write(dev, CORRECTION_PARA_START_ADRRESS ,&correct_coefficient , data_num) < data_num)
            {
              
							
                DEBUG("sensor para write failed");
			
							 return ERROR;
            }
        }
        
        
    }
        break;
		case update_correction_sensor:
		{
         os_device_t *dev = os_device_get("EEPROM"); 
		        /*check  if opened successfully*/
            if(dev == NULL)
            {
           
							  DEBUG("EEPROM open failed");
                return ERROR;
            }
						
		        /*read data from eeprom failed*/
            if(os_device_read(dev, CORRECTION_PARA_START_ADRRESS ,&correct_coefficient , sizeof(correction_para)) < sizeof(correction_para))
						{
   
                DEBUG("sensor para read failed");				 
							  return ERROR;  
						}
		}  		
		   break;
    case sensor_close :         break;
    case sensor_open :          break;			
    default:break;
        
    }
    return SUCCESS;
}

/*********************************************************************
 * @fn      me2_ch20_read
 *
 * @brief   This function read data from sensor
 *
* 			    @param dev: device poniter 
*								   pos: offset in the data ;ignored here
*									 buffer: data need tansfering ;  attention::data is stored in 16 bits mode  
* 								 size: data size tansfered
 *   @return number of data readed
 */

os_size_t me2_ch20_read   (os_device_t* dev, os_off_t pos, void *buffer, os_size_t size)
{
    u16 i=0;
    if(dev==NULL)
    {
        return ERROR;
    }
    //	set_me2_ch20_state(sensor_open);
    while(size >0)
    {
        
        ((u16*)buffer)[i] = adc_avr_cal();
        i++;
        size--;
    }
    //		set_me2_ch20_state(sensor_close);
    return  i;	
    
}
/*********************************************************************
 * @fn      ADC1_DMA_Init(void)
 *
 * @brief   This function is write driver for adc1 and dma1
 *
* 			    @param dev: none
 *   @return success or error
 */

static os_err_t adc1_dma_init(void)
{
    ADC_InitTypeDef     ADC_InitStructure;
    DMA_InitTypeDef     DMA_InitStructure;
    u16 count_time=0;
    /* ADC1 DeInit */
    ADC_DeInit(ADC1);

    
    /* ADC1 Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    /* DMA1 clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

 /* GPIOB Periph clock enable */
	/*moved to gpio mgr.c*/
//   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//    /* Configure ADC Channel11 as analog input */
//    GPIO_InitStructure.GPIO_Pin = BIT_SHIFT(ADC_SENSOR_PIN) | BIT_SHIFT(ADC_REF_PIN);
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init(ADC_SENSOR_PORT, &GPIO_InitStructure);
    
    /* DMA1 Channel1 Config */
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(0x40012440);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RegularConvData_Tab;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ADCBUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    /* DMA1 Channel1 enable */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    /* ADC DMA request in circular mode */
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
    
    /* Enable ADC_DMA */
    ADC_DMACmd(ADC1, ENABLE);
    
    /* Initialize ADC structure */
    ADC_StructInit(&ADC_InitStructure);
    
    /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    /* Convert the ADC1 Channel 1 with 55.5 Cycles as sampling time */
    ADC_ChannelConfig(ADC1, ADC_SENSOR_CHANNEL , ADC_SampleTime_55_5Cycles);
    
    
    /* Convert the ADC1 temperature sensor  with 55.5 Cycles as sampling time */
    ADC_ChannelConfig(ADC1, ADC_REF_CHANNEL , ADC_SampleTime_55_5Cycles);
    
    
    /* ADC Calibration */
    ADC_GetCalibrationFactor(ADC1);
    
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    
    /* Wait the ADCEN falg */
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)&&count_time<2000)
    {
        
    }
    ADC_StartOfConversion(ADC1);
    /*check out if this has been initiated sucessfully*/
    if(count_time<2000)
    {
        return  SUCCESS;
    }
    
    return ERROR;
    
}



/*********************************************************************
 * @fn      sensor shut down(void)
 *
 * @brief   This function is write driver for adc1 and dma1
 *
* 			    @param dev: none
 *   @return success or error
 */
static os_err_t set_me2_ch20_state(u8 operation)
{
    /*shut down operation to save power*/	
    if(operation == sensor_close)
    {
        SENSOR_POWER_CTL_PORT -> BRR = BIT_SHIFT(SENSOR_POWER_CTL_PIN);
    }
    else if(operation == sensor_open )
    {
        SENSOR_POWER_CTL_PORT -> BSRR = BIT_SHIFT(SENSOR_POWER_CTL_PIN);
    }
    return SUCCESS;
}

/*********************************************************************
 * @fn      ADC1_DMA_Init(void)
 *
 * @brief   This function is write driver for adc1 and dma1
 *
* 			    @param dev: none
 *   @return success or error
 */

static os_err_t me2_ch20_ctlpin_init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    
    /* Configure sensor ctl port as output */
    GPIO_InitStructure.GPIO_Pin   =  BIT_SHIFT(SENSOR_POWER_CTL_PIN);
    GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  =  GPIO_PuPd_UP ;
    GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_2MHz ;
    GPIO_Init(SENSOR_POWER_CTL_PORT, &GPIO_InitStructure);
    set_me2_ch20_state(sensor_open);
    
    return SUCCESS;
    
}

/*********************************************************************
 * @fn      static u16 adc_avr_cal()
 *
 * @brief   This function is sample data from ref pin and sensor pin 
 *					minimazing the error by average filter 
 *
 * 			    @param  none
 *   @return the voltage appeapered in sensor pin amplified by 1000 times 
 */
u16 sensor_get_voltage(void);

static u16 adc_avr_cal()
{
   
     int  HCHOConvertedValue = 0;
	   u16  sensor_voltage = 0;
		/*if the correction parameter still not updated*/  
		if(correct_coefficient.para_a == 0 && correct_coefficient.para_b ==0)
		{
		/*update parameter failed using default parameter*/		
			 if(me2_ch20_control( &(adc_dev->os_device) , update_correction_sensor , NULL ) == ERROR)
				{
					
					correct_coefficient.para_a = DEFUALT_PAR_A;
					correct_coefficient.para_b = DEFUALT_PAR_B;
					
          
            DEBUG("me2ch2o update failed!\r\n");  
        
        }
				
				zero_density_voltage = (ABS((int)(correct_coefficient.para_a))/correct_coefficient.para_b);
			
				
		}
		
	   /*sample the voltage produced by sensor*/
		sensor_voltage = sensor_get_voltage() -SENSOR_REF_VOL;
		
		
//		if((sensor_voltage+10) < (ABS(zero_density_voltage)) && sensor_voltage >0)
//		{
////			u8 arg[sizeof(correction_para)+2]={0};
////			arg[0] = sizeof(correction_para);
////			
// 			zero_density_voltage = sensor_voltage;
////			
////			/*adjust the zero point dymatically*/
//			correct_coefficient.para_a = (0-(int)(zero_density_voltage*(correct_coefficient.para_b)));
////			
////			osmemcpy(&arg[1], &correct_coefficient , sizeof(correction_para) );
//			
//			/*save new parameter to eeprom*/
//		//	if(me2_ch20_control( &(adc_dev->os_device) , set_correction_para , arg ) == SUCCESS)
//	//		{
//				printf("zero point adjustment finished");
//  //    }
//			
//    }
		/*the coefficient has been amplifier by 100 times*/
    HCHOConvertedValue = ((sensor_voltage)*correct_coefficient.para_b);
		HCHOConvertedValue += ((int)correct_coefficient.para_a);
		/*if lower then a, negative number gonna created which is not allowable*/

    if(HCHOConvertedValue < 0)
    {
        return 0;
    }
    else  /*return final result*/
    {  
        return ((u32)HCHOConvertedValue/100);
    }
  
    
}
/*********************************************************************
 * @fn      u16 sensor_get_voltage()
 *
 * @brief   This function is sample data from ref pin and sensor pin 
 *					minimazing the error by average filter 
 *
 * 			    @param  none
 *   @return the voltage appeapered in sensor pin amplified by 1000 times 
 */

u16 sensor_get_voltage()
{
  
	/*record position to strore data in smoth liner*/
	  static u8  smooth_positon =0;
	  u16  ADC1RefConvertedValue = 0;
    u16  HCHOConvertedValue = 0;
   
    u8   index;
    u32  ADC1RefConvertedSum = 0, HCHOConvertedSum = 0;
    u32  ADC1RefConvertedValueMin = 0xfffe, ADC1RefConvertedValueMax = 0;
    u32  HCHOConvertedValueMin = 0xfffe, HCHOConvertedVoltageMax = 0;
	
     /* Test DMA1 TC flag */
    while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET );
    
    /*average filter*/
    for(index = 0; index < ADCBUF_SIZE; ++index)
    { 
        if(index & 1)
        {
            ADC1RefConvertedSum += RegularConvData_Tab[index];
            if(RegularConvData_Tab[index] > ADC1RefConvertedValueMax)
            {
                ADC1RefConvertedValueMax = RegularConvData_Tab[index];
            }
            if(RegularConvData_Tab[index] < ADC1RefConvertedValueMin)
            {
                ADC1RefConvertedValueMin = RegularConvData_Tab[index];
            }
        }
        else
        {
            HCHOConvertedSum +=  RegularConvData_Tab[index];
            if(RegularConvData_Tab[index] > HCHOConvertedVoltageMax)
            {
                HCHOConvertedVoltageMax = RegularConvData_Tab[index];
            }
            if(RegularConvData_Tab[index] < HCHOConvertedValueMin)
            {
                HCHOConvertedValueMin = RegularConvData_Tab[index];
            }
        }
    }
    
    /*get rid of the max value and min value from sensor*/
    HCHOConvertedSum = HCHOConvertedSum - HCHOConvertedVoltageMax - HCHOConvertedValueMin;
		
	  /*get rid of the max value and min value from refence*/
    ADC1RefConvertedSum = ADC1RefConvertedSum - ADC1RefConvertedValueMax - ADC1RefConvertedValueMin;
    
		/*get average voltage value for sensor*/
    ADC1RefConvertedValue = ADC1RefConvertedSum / ((ADCBUF_SIZE >> 1) - 2);
		
		/*get average voltage value for refernce*/
    HCHOConvertedValue    = HCHOConvertedSum / ((ADCBUF_SIZE >> 1) - 2);
    
    
    /* Clear DMA TC flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
    
		HCHOConvertedValue = HCHOConvertedValue * ADC_REF_VOL / ADC1RefConvertedValue;
		
		smooth_filter_tab[smooth_positon++] = HCHOConvertedValue;
		
		/*warp round when comed to the end of queue*/
		if(smooth_positon >= SMOOTH_FILTER_LENTH)
		{
			smooth_positon =0;
		}
		
		/*get the average value from smooth queue*/
		for(index = 0 , ADC1RefConvertedSum=0; index < SMOOTH_FILTER_LENTH ; index++)
	  {
		   	ADC1RefConvertedSum += smooth_filter_tab[index];
	  }
		
		
		HCHOConvertedValue = ADC1RefConvertedSum/SMOOTH_FILTER_LENTH;
		
		/*if the voltage sampled for sensor is loewer than reference*/
    if(HCHOConvertedValue < SENSOR_REF_VOL)
    {
        return SENSOR_REF_VOL;
    }
    
    /*return final result*/
    return (HCHOConvertedValue);
}

