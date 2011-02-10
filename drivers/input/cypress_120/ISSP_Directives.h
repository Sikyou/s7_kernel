// filename: ISSP_Directives.h

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
* File Name: Issp_Directives.h			     									*
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

// --------------------- Compiler Directives ----------------------------------
#ifndef _INC_ISSP_DIRECTIVES_H_
#define _INC_ISSP_DIRECTIVES_H_

// This application uses reset programming mode Reset programming mode uses the
// external reset pin (Xres) to enter programming mode. 

// This directive causes the proper Initialization vector #3 to be sent
// to the Target, based on what the Target Vdd programming voltage will
// be. Either 5V (if #define enabled) or 3.3V (if #define disabled).
//#define TARGET_VOLTAGE_IS_5V

// This Directive will enable the Block-Verify Function.
// !CAUTION! Enabing this function consumes 65-Bytes of RAM!
// Otherwise, the firmware operates from a 16-Byte buffer instead.
#define USE_BLOCK_VERIFY

// This directive will program the Flash-Security Bytes, if desired.
// NOTE: The USE_SECURITY feature can only be enabled if the USE_BLOCK_VERIFY is also
//		enabled. This is ONLY because the routines for doing the security-byte writes
//		with the 16-Byte buffer have not been created.

#ifdef USE_BLOCK_VERIFY
	#define USE_SECURITY	// Enable, or not, as needed.
#endif

// If your compiler does not support the 0b data type which is not a C standard.
// Please use these macros
//#define Ob( x )  ( (unsigned)Ob_( 0 ## x ## uL ) )
//#define Ob_( x ) ( ( x & 1 ) | ( x >> 2 & 2 ) | ( x >> 4 & 4 ) | ( x >> 6 & 8 ) | \
    			 ( x >> 8 & 16 ) | ( x >> 10 & 32 ) | ( x >> 12 & 64 ) | ( x >> 14 & 128 ) )

//-----------------------------------------------------------------------------
// The directives below enable support for various PSoC devices. The root part
// number to be programmed should be un-commented so that its value becomes
// defined.  All other devices should be commented out.
// Select one device to be supported below:
//-----------------------------------------------------------------------------

// **** CY8C20x24 devices ****
//#define CY8C20224
//#define CY8C20324
//#define CY8C20424
//#define CY8C20524

// **** CY8C20x34 devices ****
//#define CY8C20234
//#define CY8C20334

#define CY8C24794    -----------------------nielimin 00164272--------
// **** CY8C21x23 devices ****
//#define CY8C21123
//#define CY8C21223
//#define CY8C21323
//#define CY8C21002

// **** CY8C21x34 devices ****
//#define CY8C21234
//#define CY8C21334
//#define CY8C21434
//#define CY8C21534
//#define CY8C21634
//#define CY8C21001

// **** CY8C24x23A devices ****
//#define CY8C24123A
//#define CY8C24223A
//#define CY8C24423A
//#define CY8C24000A

// **** CY8C24x94 devices ****
//#define CY8C24794
//#define CY8C24894
//#define CY8C24994
//#define CY8C24094

// **** CY8C27x34 devices ****
//#define CY8C27143
//#define CY8C27243
//#define CY8C27443
//#define CY8C27543
//#define CY8C27643
//#define CY8C27002

// **** CY8C29x66 devices ****
//#define CY8C29466
//#define CY8C29566
//#define CY8C29666
//#define CY8C29866
//#define CY8C29002

//////////////////////////////////////////////////////////////////////////////////////////
// Targets ID's

//-----------------------------------------------------------------------------
// This section sets the Family and target part ID that has been selected.
// These are used to simplify other conditional compilation blocks.
//-----------------------------------------------------------------------------

#ifdef CY8C20224
    #define CY8C20x24
	#define TARGET_ID ( 0x0868 )
#endif

#ifdef CY8C20324
    #define CY8C20x24
	#define TARGET_ID ( 0x0869 )
#endif

#ifdef CY8C20324
    #define CY8C20x24
	#define TARGET_ID ( 0x0869 )
#endif

#ifdef CY8C20524
    #define CY8C20x24
	#define TARGET_ID ( 0x086e )
#endif

#ifdef CY8C20234
    #define CY8C20x34
	#define TARGET_ID ( 0x0068 )
#endif

#ifdef CY8C20334
    #define CY8C20x34
	#define TARGET_ID ( 0x0069 )
#endif
									// NOTE....
#ifdef CY8C20434					// During test with a CY3210- 20x34 Evaluation pod, the silicon
    #define CY8C20x34				// ID read in as 0x6b, but the Cypress documentation states
	#define TARGET_ID ( 0x006b )	// 0x6a
#endif

#ifdef CY8C21123
    #define CY8C21x23
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C21223
    #define CY8C21x23
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C21323
    #define CY8C21x23
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C21002
    #define CY8C21x23
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C21234
    #define CY8C21x34
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C21334
    #define CY8C21x34
	#define TARGET_ID ( 0x0037 )
#endif

#ifdef CY8C21434
    #define CY8C21x34
	#define TARGET_ID ( 0x0038 )
#endif

#ifdef CY8C21534
    #define CY8C21x34
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C21634
    #define CY8C21x34
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C21001
    #define CY8C21x34
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C24123A
    #define CY8C24x23A
	#define TARGET_ID ( 0x0012 )
#endif

#ifdef CY8C24223A
    #define CY8C24x23A
	#define TARGET_ID ( 0x0013 )
#endif

#ifdef CY8C24423A
    #define CY8C24x23A
	#define TARGET_ID ( 0x0014 )
#endif

#ifdef CY8C24000A
    #define CY8C24x23A
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C24794
    #define CY8C24x94
	#define TARGET_ID ( 0x001d )
#endif

#ifdef CY8C24894
    #define CY8C24x94
	#define TARGET_ID ( 0x001f )
#endif

#ifdef CY8C24994
    #define CY8C24x94
	#define TARGET_ID ( 0x0059 )
#endif

#ifdef CY8C24094
    #define CY8C24x94
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C27143
    #define CY8C27x43
	#define TARGET_ID ( 0x0009 )
#endif

#ifdef CY8C27243
    #define CY8C27x43
	#define TARGET_ID ( 0x000a )
#endif

#ifdef CY8C27443
    #define CY8C27x43
	#define TARGET_ID ( 0x000b )
#endif

#ifdef CY8C27543
    #define CY8C27x43
	#define TARGET_ID ( 0x000c )
#endif

#ifdef CY8C27643
    #define CY8C27x43
	#define TARGET_ID ( 0x000d )
#endif

#ifdef CY8C27002
    #define CY8C27x43
	#define TARGET_ID ( 0x0000 )
#endif

#ifdef CY8C29466
    #define CY8C29x66
	#define TARGET_ID ( 0x002a )
#endif

#ifdef CY8C29566
    #define CY8C29x66
	#define TARGET_ID ( 0x002b )
#endif

#ifdef CY8C29666
    #define CY8C29x66
	#define TARGET_ID ( 0x002c )
#endif

#ifdef CY8C29866
    #define CY8C29x66
	#define TARGET_ID ( 0x002d )
#endif

#ifdef CY8C29002
    #define CY8C29x66
	#define TARGET_ID ( 0x0000 )
#endif

//-----------------------------------------------------------------------------
// The directives below are used to define various sets of vectors that differ
// for more than one set of PSoC parts.
//-----------------------------------------------------------------------------
// **** Select a Checksum Setup Vector ****
// **** Select a Program Block Vector ****

#ifdef CY8C20x24
    #define CHECKSUM_SETUP_24_29
    #define PROGRAM_BLOCK_21_24_29
#endif

#ifdef CY8C20x34
    #define CHECKSUM_SETUP_24_29
    #define PROGRAM_BLOCK_21_24_29
#endif

#ifdef CY8C21x23
    #define CHECKSUM_SETUP_21_27
    #define PROGRAM_BLOCK_21_24_29
#endif

#ifdef CY8C21x34
    #define CHECKSUM_SETUP_21_27
    #define PROGRAM_BLOCK_21_24_29
#endif

#ifdef CY8C24x23A
    #define CHECKSUM_SETUP_24_24A
    #define PROGRAM_BLOCK_21_24_29
#endif

#ifdef CY8C24x94
    #define CHECKSUM_SETUP_24_29
    #define PROGRAM_BLOCK_21_24_29
#endif

#ifdef CY8C27x43
    #define CHECKSUM_SETUP_21_27
    #define PROGRAM_BLOCK_27
#endif

#ifdef CY8C29x66
    #define CHECKSUM_SETUP_24_29
    #define PROGRAM_BLOCK_21_24_29
#endif

//-----------------------------------------------------------------------------
// The directives below are used to control switching banks if the device
// has multiple banks of Flash.
//-----------------------------------------------------------------------------
// **** Select a Checksum Setup Vector ****
#ifdef CY8C24x94
    #define MULTI_BANK
#endif
#ifdef CY8C29x66
    #define MULTI_BANK
#endif

// ----------------------------------------------------------------------------
#endif  //(_INC_ISSP_DIRECTIVES_H_)
#endif  //(PROJECT_REV_)

//end of file ISSP_Directives.h
