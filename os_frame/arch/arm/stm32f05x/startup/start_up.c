/***************************************************************
 * Name:      test.c
 * Purpose:   system gonna be started from here
 * Author:    mikewang(s)
 * Created:   2014-06-13
 * Copyright: mikewang(mikewang01@hotmail.com)
 * License:
 **************************************************************/

/*********************************************************************
 * INCLUDES
 */

//#include "kernel-includes.h"
//#include "drivers-includes.h"
/*********************************************************************
* MACROS
*/



/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
 
 
/*********************************************************************
 * EXTERNAL VARIABLES
 */
 
/*********************************************************************
 * FUNCTIONS
 */



/*********************************************************************
 * @fn      int main(void)
 *
 * @brief   This function start the shedule machine for system
 *
 * 			    @param none
 * 
 *   @return the error code, OS_EOK on initialization successfully. 
 */
 
#include "stdio.h"
#include "Q_Shell.h"
#include "system_stm32f0xx.h"
#include "led.h"

int main(void)
{				  
	 

	 OS_Init();
	 Shedule();
	 while(1);
		 
}

























