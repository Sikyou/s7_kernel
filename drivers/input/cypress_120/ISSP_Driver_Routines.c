// filename: ISSP_Driver_Routines.c

#include "ISSP_Revision.h"
#ifdef PROJECT_REV_110

/********************************************************************************
* Copyright (c) 2008 Cypress Semiconductor Corporation. All rights reserved.	*
*																				*
*********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress)			*
* and is protected by and subject to worldwide patent protection (United		*
* States and foreign), United States copyright laws and international 			*
* treaty provisions. Cypress hereby grants to licensee a personal, 				*
* non-exclusive, non-transferable license to copy, use, modify, create 			*
* derivative works of, and compile the Cypress Source Code and derivative 		*
* works for the sole purpose of creating custom software in support of 			*
* licensee product to be used only in conjunction with a Cypress integrated 	*
* circuit as specified in the applicable agreement. Any reproduction, 			*
* modification, translation, compilation, or representation of this 			*
* software except as specified above is prohibited without the express 			*
* written permission of Cypress.												*
*																				*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND,EXPRESS OR IMPLIED,			* 
* WITH REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED		* 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.			*
* Cypress reserves the right to make changes without further notice to the		*
* materials described herein. Cypress does not assume any liability arising		*
* out of the application or use of any product or circuit described herein.		*
* Cypress does not authorize its products for use as critical components in		*
* life-support systems where a malfunction or failure may reasonably be			*
* expected to result in significant injury to the user. The inclusion of		*
* Cypress’ product in a life-support systems application implies that the		*
* manufacturer assumes all risk of such use and in doing so indemnifies			*
* Cypress against all charges.													*
*																				*
* Use may be limited by and subject to the applicable Cypress software license  *
* license agreement																*
*********************************************************************************
*																				*
* You may not use this file except in compliance with the Cypress Semiconductor *
* Corporation IP Library License and Usage Agreement. Please obtain a copy of   *
* the Agreement at http://www.cypress.com/IP_Library_License_Agreement.pdf and  *
* read it before using this file. 												*
*																				*
* This file and all files distributed under the Agreement are distributed on an *
* 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS, IMPLIED OR 		*
* STATUTORY, REGARDING THE SUFFICIENCY, ACCURACY OR COMPLETENESS OF THE 		*
* INFORMATION AND CYPRESS HEREBY DISCLAIMS ALL SUCH WARRANTIES, INCLUDING 		*
* WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 			*
* PARTICULAR PURPOSE, AND NON-INFRINGEMENT. Cypress Semiconductor Corporation 	*
* reserves the right to make changes to this file and any files distributed 	*
* under the Agreement without further notice.									*
*																				*
* Please see the License and Usage Agreement for the specific language 			*
* governing rights and limitations under the Agreement.							*
********************************************************************************/

/********************************************************************************
* File Name: Issp_Driver_Routines.c		     									*
*									       										*
*   Purpose: main																*
*									       										*
*********************************************************************************

*********************************************************************************
* Authors:									       								*
*-------------------------------------------------------------------------------*
* Int Name                          Company                                     *
* --- ----------------------------	--------------------------------------------*
* MDS Michael D. Sherwood     		J Gordon Electronic Design ( JGED )         *
*									       										*
*********************************************************************************

*********************************************************************************
* Revision History:							       								*
*-------------------------------------------------------------------------------*
*  REV  *   DATE     *  BY   *            ECO NUMBER - DESCRIPTION             	*
*-------------------------------------------------------------------------------*
*       *            *       *                                                  *
* 0.1.0 * 03/25/2008 *  MDS  * Modifed											*
*       *            *       *                                                  *
*       *            *       *                                                  *
********************************************************************************/

/*******************************************************************************
* Include files
********************************************************************************/
//#include <m8c.h>        		// part specific constants and macros
//#include "PSoCAPI.h"    		// PSoC API definitions for all User Modules

#include "ISSP_Driver_Routines.h"	// ISSP driver routines
#include <mach/gpio.h>
#include <linux/delay.h>



// ((((((((((((((((((((( LOW-LEVEL ISSP SUBROUTINE SECTION ))))))))))))))))))))
// ((((                        PROCESSOR_SPECIFIC                          ))))   
// ((((The subroutines in this section will need to be modified in the     ))))
// (((( final version of this program. The details of the low-level        ))))
// (((( routines will depend on the specific Host processor used.          ))))
// (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))

/*******************************************************************************
* Functions
********************************************************************************/

/*
 ** IsspDriverDelay
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     Number of loops
 *
 *  DESCRIPTION:    This delay uses a simple "nop" loop. With the CPU running at
 *					24MHz, each pass of the loop is about 1 usec plus an overhead
 *					of about 3 usec. ( total delay = (n + 3) * 1 usec )
 *					To adjust delays and to adapt delays when porting this
 *					application, see the ISSP_Delays.h file.
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
#define CYPRESS_I2C_ADDR   		(0x26)//(0x05)//
#define CYPRESS_DECT_INT_GPIO   (28) // (28)
#define CYPRESS_RST_GPIO   		(158)
#define GPIO_SCL  (95)
#define GPIO_SDA  (96)


void IsspDriverDelay( unsigned char n )
{
    while( n )
    {        
        n -= 1;
    }
}

/*
 ** IsspDriverSDATACheck
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Check SDATA pin for high or low logic level and return value
 *					to calling routine.
 *
 *  RETURNS:        0 if the pin was low.
 *				    1 if the pin was high.
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
unsigned char IsspDriverSDATACheck( void )
{	
    if ( gpio_get_value(GPIO_SDA) )    		// Port 0
    {
        return( 1 );
    }
    else
    {
        return( 0 );
    }    
}
        
/*
 ** IsspDriverSCLKHigh
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set the SCLK pin High
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverSCLKHigh( void )
{
	gpio_set_value(GPIO_SCL, 1);
	//udelay(4);
}

/*
 ** IsspDriverSCLKLow
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set the Clock pin Low
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverSCLKLow( void )
{
    gpio_set_value(GPIO_SCL, 0);
	//udelay(4);
}

/*
 ** IsspDriverSetSCLKStrong
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set SCLK to an output (Strong drive mode)
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverSetSCLKStrong( void )
{
    gpio_tlmm_config(GPIO_CFG(GPIO_SCL, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
}

/*
 ** IsspDriverSetSDATAHigh
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set SDATA pin high
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverSetSDATAHigh( void )
{
	gpio_set_value(GPIO_SDA, 1);
	udelay(4);
}

/*
 ** IsspDriverSetSDATALow
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set SDATA pin low
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverSetSDATALow( void )
{
   	gpio_set_value(GPIO_SDA, 0);
	udelay(4);
}
	
/*
 ** IsspDriverSetSDATAHiZ
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set SDATA pin to an input (HighZ drive mode).
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverSetSDATAHiZ( void )
{
	gpio_tlmm_config(GPIO_CFG(GPIO_SDA, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
}

/*
 ** IsspDriverSetSDATAStrong
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set SDATA for transmission (Strong drive mode) -- as opposed
 *					to being set to High Z for receiving data.
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverSetSDATAStrong( void )
{
    gpio_tlmm_config(GPIO_CFG(GPIO_SDA, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
}

/*
 ** IsspDriverSetXRESStrong
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set external reset (XRes) to an output (Strong drive mode).
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverSetXRESStrong( void )
{    
    gpio_tlmm_config(GPIO_CFG(CYPRESS_RST_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
}

/*
 ** IsspDriverAssertXRES
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set XRES pin High
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverAssertXRES( void )
{
   	gpio_set_value(CYPRESS_RST_GPIO, 1);
}

/*
 ** IsspDriverDeassertXRES
 *
 *  FILENAME: 		Issp_Driver_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Set XRES pin low
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
void IsspDriverDeassertXRES( void )
{
       	gpio_set_value(CYPRESS_RST_GPIO, 0);
}

#endif  //(PROJECT_REV_110)

//end of file ISSP_Drive_Routines.c
