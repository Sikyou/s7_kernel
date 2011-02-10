// filename: ISSP_Driver_Routines.h

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
* File Name: Issp_Driver_Routines.h		     									*
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
#ifndef _INC_ISSP_DRIVER_ROUTINES_H_
#define _INC_ISSP_DRIVER_ROUTINES_H_

#include "ISSP_Directives.h"

#define ISSP_SDATA_PIN   ( 0x01 )		// P0.0
#define ISSP_XRES_PIN    ( 0x40	)		// P0.6
#define ISSP_SCLK_PIN    ( 0x80	)		// P0.7

void IsspDriverSCLKHigh( void );
void IsspDriverSCLKLow( void );
void IsspDriverSetSCLKStrong( void );

void IsspDriverSetSDATAHigh( void );
void IsspDriverSetSDATALow( void );
unsigned char IsspDriverSDATACheck( void );
void IsspDriverSetSDATAHiZ( void );
void IsspDriverSetSDATAStrong( void );

void IsspDriverAssertXRES( void );
void IsspDriverDeassertXRES( void );
void IsspDriverSetXRESStrong( void );

void IsspDriverDelay( unsigned char );

#endif  //(_INC_ISSP_DRIVER_ROUTINES_H_)
#endif  //(PROJECT_REV_)

//end of file ISSP_Driver_Routines.h
