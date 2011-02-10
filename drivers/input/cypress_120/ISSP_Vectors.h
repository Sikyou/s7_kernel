// filename: ISSP_Vectors.h

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
* File Name: Issp_Vectors.h				     									*
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
#ifndef _INC_ISSP_VECTORS_H_
#define _INC_ISSP_VECTORS_H_

#include "ISSP_Defs.h"


// -------------------------------------- PSoC "Diamond" 27xxx Devices -------------------------------------------------
// Modifying these tables is NOT recommendended. Doing so will all but guarantee an ISSP error, unless updated
// vectors have been recommended or provided by Cypress MicroSystems.
// ----------------------------------------------------------------------------------------------------------------------

//////////////////////////////////////////////////////////////////////////////////////////
// Target configuration
//const unsigned int target_id = TARGET_ID;								// Target ID
const unsigned int target_id = 0x071f;								// Target ID
const unsigned char number_of_banks = NUM_BANKS;						// Number of banks
const unsigned int blocks_per_bank = BLOCKS_PER_BANK;					// Number of blocks per bank
// nielimin  00164272
//const unsigned char number_of_banks = 2;						// Number of banks
//const unsigned int blocks_per_bank = 128;

const unsigned char security_bytes_per_bank = SECURITY_BYTES_PER_BANK;	// Number of security bytes per bank

//////////////////////////////////////////////////////////////////////////////////////////
// Initialize Vectors

const int num_bits_init1 = 396;		//400; NOTE: Last Byte padded with four 0's
const char init1_v[] =
{
// 20xxxx,21xxx,22xxx,24xxx,27xxx,29xxx
	0b11001010,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,
	0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,
	0b00001101,0b11101110,0b00000001,0b11110111,0b10110000,0b00000111,0b10011111,0b00000111,
	0b01011110,0b01111100,0b10000001,0b11111101,0b11101010,0b00000001,0b11110111,0b10100000,
	0b00011111,0b10011111,0b01110000,0b00011111,0b01111100,0b10011000,0b01111101,0b11110100,
	0b10000001,0b11110111,0b10000000,0b01001111,0b11011111,0b00000000,0b00011111,0b01111111,
	0b10001001,0b01110000
};

const int num_bits_init2 = 286;		//288; NOTE: Last Byte padded with two 0's
const char init2_v[] =
{
	0b11011110,0b11100000,0b00011111,0b01111011,0b00000000,0b01111001,0b11110000,0b01110101,
	0b11100111,0b11001000,0b00011111,0b11011110,0b10100000,0b00011111,0b01111010,0b00000001,
	0b11111001,0b11110111,0b00000001,0b11110111,0b11001001,0b10000111,0b11011111,0b01001000,
	0b00011110,0b01111101,0b00000000,0b11111101,0b11100000,0b00001101,0b11110111,0b11000000,
	0b00000111,0b11011111,0b11100010,0b01011100
};

#ifdef TARGET_VOLTAGE_IS_5V
	const int num_bits_init3_5v = 836;	//840; NOTE: Last Byte padded with four 0's
	const char init3_5v[] =
	{
		0b11011110,0b11100000,0b00011111,0b01111010,0b00000001,0b11111101,0b11101010,0b00000001,
		0b11110111,0b10110000,0b01000111,0b11011111,0b00001010,0b00111111,0b01111100,0b11111110,
		0b01111101,0b11110100,0b01100001,0b11110111,0b11111000,0b10010111,0b00000000,0b00000000,
		0b00000011,0b01111011,0b10000000,0b01111101,0b11101000,0b00000111,0b11110111,0b10101000,
		0b00000111,0b11011110,0b11000001,0b00011111,0b01111100,0b00110000,0b01111101,0b11110011,
		0b11010101,0b11110111,0b11010001,0b10000111,0b11011110,0b11100010,0b00011111,0b01111111,
		0b10001001,0b01110000,0b00000000,0b00000000,0b00110111,0b10111000,0b00000111,0b11011110,
		0b10000000,0b01111111,0b01111010,0b10000000,0b01111101,0b11101100,0b00010001,0b11110111,
		0b11000010,0b10001111,0b11011111,0b00111111,0b10111111,0b01111101,0b00011000,0b01111101,
		0b11111110,0b00100101,0b11000000,0b00000000,0b00000000,0b11011110,0b11100000,0b00011111,
		0b01111010,0b00000001,0b11111101,0b11101010,0b00000001,0b11110111,0b10110000,0b01000111,
		0b11011111,0b00001100,0b00011111,0b01111100,0b11110100,0b01111101,0b11110100,0b01100001,
		0b11110111,0b10111000,0b10000111,0b11011111,0b11100010,0b01011100,0b00000000,0b00000000,
		0b00000000
	};
#else
	const int num_bits_init3_3v = 836;	//840; NOTE: Last Byte padded with four 0's
	const char init3_3v[] =
	{
		0b11011110,0b11100000,0b00011111,0b01111010,0b00000001,0b11111101,0b11101010,0b00000001,
		0b11110111,0b10110000,0b01000111,0b11011111,0b00001010,0b00111111,0b01111100,0b11111100,
		0b01111101,0b11110100,0b01100001,0b11110111,0b11111000,0b10010111,0b00000000,0b00000000,
		0b00000011,0b01111011,0b10000000,0b01111101,0b11101000,0b00000111,0b11110111,0b10101000,
		0b00000111,0b11011110,0b11000001,0b00011111,0b01111100,0b00110000,0b01111101,0b11110011,
		0b11010101,0b11110111,0b11010001,0b10000111,0b11011110,0b11100010,0b00011111,0b01111111,
		0b10001001,0b01110000,0b00000000,0b00000000,0b00110111,0b10111000,0b00000111,0b11011110,
		0b10000000,0b01111111,0b01111010,0b10000000,0b01111101,0b11101100,0b00010001,0b11110111,
		0b11000010,0b10001111,0b11011111,0b00111111,0b00111111,0b01111101,0b00011000,0b01111101,
		0b11111110,0b00100101,0b11000000,0b00000000,0b00000000,0b11011110,0b11100000,0b00011111,
		0b01111010,0b00000001,0b11111101,0b11101010,0b00000001,0b11110111,0b10110000,0b01000111,
		0b11011111,0b00001100,0b00011111,0b01111100,0b11110100,0b01111101,0b11110100,0b01100001,
		0b11110111,0b10111000,0b10000111,0b11011111,0b11100010,0b01011100,0b00000000,0b00000000,
		0b00000000
	};
#endif /* TARGET_VOLTAGE_IS_5V */

//////////////////////////////////////////////////////////////////////////////////////////
// ID setup vectors
const int num_bits_id_setup = 330;	//336; NOTE: Last Byte padded with six 0's
// 20x24, 20x34, 21xxx, 24xxx, 27xxx and 29xxx
const char id_setup_v[] =
{
//
	0b11011110,0b11100010,0b00011111,0b01110000,0b00000001,0b01111101,0b11101110,0b00000001,
	0b11110111,0b10110000,0b00000111,0b10011111,0b00000111,0b01011110,0b01111100,0b10000001,
	0b11111101,0b11101010,0b00000001,0b11110111,0b10100000,0b00011111,0b10011111,0b01110000,
	0b00011111,0b01111100,0b10011000,0b01111101,0b11110100,0b10000001,0b11100111,0b11010000,
	0b00000111,0b11011110,0b00000000,0b11011111,0b01111100,0b00000000,0b01111101,0b11111110,
	0b00100101,0b11000000
};

//////////////////////////////////////////////////////////////////////////////////////////
// Erase all vectors
const int num_bits_erase_all = 308;		//312; NOTE: Last Byte padded with four 0's.
const char erase_all_v[] =
{
	0b10011111,0b10000010,0b10111110,0b01111111,0b00101011,0b01111101,0b11101110,0b00000001,
	0b11110111,0b10110000,0b00000111,0b10011111,0b00000111,0b01011110,0b01111100,0b10000001,
	0b11111101,0b11101010,0b00000001,0b11110111,0b10100000,0b00011111,0b10011111,0b01110000,
	0b00011111,0b01111100,0b10011000,0b01111101,0b11110100,0b10000001,0b11110111,0b10000000,
	0b00101111,0b11011111,0b00000000,0b00011111,0b01111111,0b10001001,0b01110000
};


//////////////////////////////////////////////////////////////////////////////////////////
// Read ID vectors
const char read_id_v[] =
{
	0b10111111,0b00000000,0b11011111,0b10010000,0b00000000
};

//////////////////////////////////////////////////////////////////////////////////////////
// Set bank number vectors
const char  set_bank_number[] =
{
	0b11011110,0b11100010,0b00011111,0b01111101,0b00000000	// 33-MSBs
};	

const char  set_bank_number_end[] =
{
	0b11111011,0b11011100,0b00000011,0b10000000				// 25-MSBs
};	


//////////////////////////////////////////////////////////////////////////////////////////
// Write byte vectors
const char	write_byte_start = 0b10010000;	// 5-MSBs
const char	write_byte_end = 0b11100000;	// 3-MSBs

//////////////////////////////////////////////////////////////////////////////////////////
// Set Block vectors
const char	set_block_number[] =
{
	0b10011111, 0b01000000, 0b11100000		// 11-MSBits
};

const char	set_block_number_end = 0b11100000;							// 3 MSBits

//////////////////////////////////////////////////////////////////////////////////////////
// Program Block vectors

#ifdef PROGRAM_BLOCK_27	
/* block programming vector for the 27K part family				*/
const int num_bits_program_block = 308;		//312; NOTE: Last Byte padded with four 0's.
const char program_block[] =
{
	0b10011111,0b10000010,0b10111110,0b01111111,0b00101011,0b01111101,0b11101110,0b00000001,
	0b11110111,0b10110000,0b00000111,0b10011111,0b00000111,0b01011110,0b01111100,0b10000001,
	0b11111101,0b11101010,0b00000001,0b11110111,0b10100000,0b00011111,0b10011111,0b01110000,
	0b00011111,0b01111100,0b10011000,0b01111101,0b11110100,0b10000001,0b11110111,0b10000000,
	0b00010111,0b11011111,0b00000000,0b00011111,0b01111111,0b10001001,0b01110000
};
#endif /* PROGRAM_BLOCK_27 */

#ifdef PROGRAM_BLOCK_21_24_29	
/* block programming vector for 21K, 24K and 29k part families	*/			
const int num_bits_program_block = 308;		//312; NOTE: Last Byte padded with four 0's.
const unsigned char program_block[] =
{
    0x9F, 0x8A, 0x9E, 0x7F, 0x2B, 0x7D, 0xEE, 0x01,
    0xF7, 0xB0, 0x07, 0x9F, 0x07, 0x5E, 0x7C, 0x81,
    0xFD, 0xEA, 0x01, 0xF7, 0xA0, 0x1F, 0x9F, 0x70,
    0x1F, 0x7C, 0x98, 0x7D, 0xF4, 0x81, 0xF7, 0x80,
    0x17, 0xDF, 0x00, 0x1F, 0x7F, 0x89, 0x70
                     
//  0b10011111,0b10001010,0b10011110,0b01111111,0b00101011,0b01111101,0b11101110,0b00000001,
//  0b11110111,0b10110000,0b00000111,0b10011111,0b00000111,0b01011110,0b01111100,0b10000001,
//  0b11111101,0b11101010,0b00000001,0b11110111,0b10100000,0b00011111,0b10011111,0b01110000,
//  0b00011111,0b01111100,0b10011000,0b01111101,0b11110100,0b10000001,0b11110111,0b10000000,
//  0b00010111,0b11011111,0b00000000,0b00011111,0b01111111,0b10001001,0b01110000
};
#endif /* PROGRAM_BLOCK_21_24_29 */
	
//////////////////////////////////////////////////////////////////////////////////////////
// Wait vectors
const char	num_bits_wait_and_poll_end = 40;
const char	wait_and_poll_end[] =
{
	0b00000000,0b00000000,0b00000000,0b00000000,0b00000000		// forty '0's per the spec
};				
	
//////////////////////////////////////////////////////////////////////////////////////////
// Checksum vectors
#ifdef CHECKSUM_SETUP_21_27
const int num_bits_checksum = 286;
const char checksum_v[] =
{
	0b11011110,0b11100000,0b00011111,0b01111011,0b00000000,0b01111001,0b11110000,0b01110101,
	0b11100111,0b11001000,0b00011111,0b11011110,0b10100000,0b00011111,0b01111010,0b00000001,
	0b11111001,0b11110111,0b00000001,0b11110111,0b11001001,0b10000111,0b11011111,0b01001000,
	0b00011110,0b01111101,0b00000000,0b01111101,0b11100000,0b00001111,0b11110111,0b11000000,
	0b00000111,0b11011111,0b11100010,0b01011100
};
#endif /* CHECKSUM_SETUP_21_27 */

#ifdef CHECKSUM_SETUP_24_29
const int num_bits_checksum = 286;
const char checksum_v[] =
{
	0b11011110,0b11100000,0b00011111,0b01111011,0b00000000,0b01111001,0b11110000,0b01110101,
	0b11100111,0b11001000,0b00011111,0b11011110,0b10100000,0b00011111,0b01111010,0b00000001,
	0b11111001,0b11110111,0b00000001,0b11110111,0b11001001,0b10000111,0b11011111,0b01001000,
	0b00011110,0b01111101,0b00000000,0b01111101,0b11100000,0b00001111,0b11110111,0b11000000,
	0b00000111,0b11011111,0b11100010,0b01011100
};
#endif /* CHECKSUM_SETUP_24_29 */
	
//////////////////////////////////////////////////////////////////////////////////////////
// Secure vectors
const int num_bits_secure = 308;
const char secure_v[] =
{
	0b10011111,0b10000010,0b10111110,0b01111111,0b00101011,0b01111101,0b11101110,0b00000001,
	0b11110111,0b10110000,0b00000111,0b10011111,0b00000111,0b01011110,0b01111100,0b10000001,
	0b11111101,0b11101010,0b00000001,0b11110111,0b10100000,0b00011111,0b10011111,0b01110000,
	0b00011111,0b01111100,0b10011000,0b01111101,0b11110100,0b10000001,0b11110111,0b10000000,
	0b00100111,0b11011111,0b00000000,0b00011111,0b01111111,0b10001001,0b01110000
};

//////////////////////////////////////////////////////////////////////////////////////////
// Read Checksum vectors

//	10111111001ZDDDDDDDDZ110111111000ZDDDDDDDDZ1
//	where DDDDDDDDDDDDDDDD= Device Checksum data out
// 0b10111111,0b001-ZDDDD,0bDDDDZ-110,0b11111-100,0b0-ZDDDDDD,0bDDZ-1

//								+0			+1			+2					+3
//								   |----- 11 ------|      |------ 12 -----|	    1
const char read_checksum_v[] = { 0b10111111, 0b00100000,0b11011111,0b10000000,0b10000000 };

//////////////////////////////////////////////////////////////////////////////////////////
// Read byte vectors

//	10110aaaaaaZDDDDDDDDZ1
//	where DDDDDDDD= data out, aaaaaa=address (6 bits)

//	  +0	    +1			+2
//	  |-5-|	      |------- 0 -----| 1
//	0b10110aaa, 0baaaZDDDD, 0bDDDDZ 1
//	0b10110000, 0b00000000, 0b00000100
// consolidated to 2-Bytes:
const char read_byte_v[] = { 0b10110000, 0b10000000 };
//								  ||-- MSBs of Address are always '10'. So..either write these bits as part of the
//											command, OR, start the address at 0x80 and count-up from there!

//////////////////////////////////////////////////////////////////////////////////////////
// Block verify vectors

#ifdef USE_BLOCK_VERIFY
	const int num_bits_verify_setup = 264;
	const char verify_setup_v[] =
	{
		0b11011110,0b11100000,0b00011111,0b01111011,0b00000000,0b01111001,0b11110000,0b01110101,
		0b11100111,0b11001000,0b00011111,0b11011110,0b10100000,0b00011111,0b01111010,0b00000001,
		0b11111001,0b11110111,0b00000001,0b11110111,0b11001001,0b10000111,0b11011111,0b01001000,
		0b00011111,0b01111000,0b00000000,0b11111101,0b11110000,0b00000001,0b11110111,0b11111000,
		0b10010111
	};

	#ifdef USE_SECURITY

//////////////////////////////////////////////////////////////////////////////////////////
// Security vectors
		const int num_bits_security = 308;
		const char security_v[] =
		{
			0b10011111,0b10000010,0b10111110,0b01111111,0b00101011,0b01111101,0b11101110,0b00000001,
			0b11110111,0b10110000,0b00000111,0b10011111,0b00000111,0b01011110,0b01111100,0b10000001,
			0b11111101,0b11101010,0b00000001,0b11110111,0b10100000,0b00011111,0b10011111,0b01110000,
			0b00011111,0b01111100,0b10011000,0b01111101,0b11110100,0b10000001,0b11110111,0b10000000,
			0b00100111,0b11011111,0b00000000,0b00011111,0b01111111,0b10001001,0b01110000
		};
	#endif	// USE_SECURITY
#endif		// USE_BLOCK_VERIFY

#endif //(_INC_ISSP_VECTORS_H_)

//end of file ISSP_Vectors.h
