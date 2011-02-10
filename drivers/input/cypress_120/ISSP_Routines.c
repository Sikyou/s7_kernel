// filename: ISSP_Routines.c

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
* File Name: main.c						     									*
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
* 0.1.0 * 03/25/2008 *  MDS  * Create											*
*       *            *       *                                                  *
*       *            *       *                                                  *
********************************************************************************/

/*******************************************************************************
* Include files
********************************************************************************/
//#include <m8c.h>        		// part specific constants and macros
//#include "PSoCAPI.h"    		// PSoC API definitions for all User Modules

#include "ISSP_Defs.h"
#include "ISSP_Delays.h"
#include "ISSP_Vectors.h"
#include "ISSP_Routines.h"
#include "ISSP_Driver_Routines.h"
#include <linux/kernel.h>
#include <linux/delay.h>

// Include your target specific .h file here !!!
#include "Test_Records.h"

/*******************************************************************************
* Defines
********************************************************************************/

// ISSP process states
enum enIsspProcess_States
{
	ISSP_INITIALIZE = 0,
	ISSP_IDENTIFY,
	ISSP_ERASE,
	ISSP_PROGRAM,
	ISSP_VERIFY,
	ISSP_SECURITY,
	ISSP_CHECKSUM,
	ISSP_RESTART,
	ISSP_DONE
};

//
// ISSP .hex record types
//
enum enHexRecordDefines
{
	ISSP_RECORD_LENGTH = 0,						// 0
	ISSP_RECORD_ADDRESS_MSB,					// 1
	ISSP_RECORD_ADDRESS_LSB,					// 2
	ISSP_RECORD_TYPE,							// 3
	ISSP_RECORD_DATA							// 4
};

#define RECORD_OVERHEAD		( 5 )				// Record overhead

//
// ISSP .hex record offsets
//
enum enHexRecordTypes
{
	ISSP_RECORD_TYPE_DATA = 0,					// 0
	ISSP_RECORD_TYPE_END_OF_FILE,				// 1
	ISSP_RECORD_TYPE_EXTENDED_SEGMENT_ADDR,		// 2
	ISSP_RECORD_TYPE_START_SEGMENT_ADDR,		// 3
	ISSP_RECORD_TYPE_EXTENDED_LINEAR_ADDR,		// 4
	ISSP_RECORD_TYPE_START_LINEAR_ADDR			// 5
};
#define CYPRESS120_DEVICE_DUG	1
#if (CYPRESS120_DEVICE_DUG)
#define CYPRESS120_DEV_DUG(format,arg...)		printk(KERN_ALERT format, ## arg); 
#define CYPRESS120_DEV_INFO(format,arg...)		printk(KERN_ALERT format, ## arg); 
#define CYPRESS120_DEV_ERR(format,arg...)		printk(KERN_ALERT format, ## arg); 
#else
#define CYPRESS120_DEV_DUG(format,arg...)		do { (void)(format); } while (0)
#define CYPRESS120_DEV_INFO(format,arg...)		do { (void)(format); } while (0)
#define CYPRESS120_DEV_ERR(format,arg...)		do { (void)(format); } while (0)
#endif
/*******************************************************************************
* Local Function Prototypes
********************************************************************************/
void IsspSendByte( unsigned char bCurrByte, unsigned char bSize );
void IsspSendVector( const unsigned char* bVect, unsigned int iNumBits );
void IsspRunClock( unsigned int iNumCycles );
void IsspSetBankNumber( unsigned char bBankNumber );

signed char IsspTargetBankChecksum( unsigned int* pCSum );
signed char IsspDetectHiLoTransition( void );
signed char IsspPowerCycleInitializeTarget( void );
signed char IsspProgramTargetBlock( unsigned char bBlockNumber );
signed char IsspVerifyTargetBlock( unsigned char bBlockNumber, const unsigned char *pHexData );
signed char IsspSecureTargetFlash( const unsigned char *pHexData );

unsigned char IsspReceiveBit( void );
unsigned char IsspReceiveByte( void );
unsigned char IsspLoadTarget( const unsigned char *pHexData );

/********************************************************************************
* Globals
*********************************************************************************/
unsigned int  iCSum;		// the target checksum

/********************************************************************************
* Functions
*********************************************************************************/

/*
 ** IsspProgram
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     none
 *
 *  DESCRIPTION:    Initializes, Verifies Silicon Id, Erases, Programs, Verifies
 *					Secures, Checksums and restarts the target device
 *
 *  RETURNS:        ISSP_PASS or one of the ISSP Error Codes
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
signed char IsspProgram( void )
{
unsigned char bProcessState = ISSP_INITIALIZE;

unsigned char bStatus = ISSP_PASS;

	while ( ( ISSP_PASS == bStatus ) && ( ISSP_DONE != bProcessState ) )
	{
		IsspDriverDelay( 0xff );
		
		switch ( bProcessState )
		{
		default:
			bStatus = ISSP_ERROR;
			break;

		case ISSP_INITIALIZE: 			
			CYPRESS120_DEV_DUG("---IsspProgram ISSP_INITIALIZE----\n");
			// Implements the intialization vectors for the device.
			if ( ISSP_PASS == ( bStatus = IsspXRESInitializeTarget() ) )
			{
				CYPRESS120_DEV_DUG("---IsspXRESInitializeTarget return =%d----\n", bStatus);
				bProcessState = ISSP_IDENTIFY;
			}
			break;

		case ISSP_IDENTIFY:
			CYPRESS120_DEV_DUG("---IsspProgram ISSP_IDENTIFY----\n");
			// Verify the device ID
			if ( ISSP_PASS == ( bStatus = IsspIdentifyTarget() ) )
			{
				CYPRESS120_DEV_DUG("---IsspIdentifyTarget return =%d----\n", bStatus);
				bProcessState = ISSP_ERASE;
			}
			break;

		case ISSP_ERASE: 			
			CYPRESS120_DEV_DUG("---IsspProgram ISSP_ERASE----\n");
			// Erase Target
			if ( ISSP_PASS == ( bStatus = IsspEraseTarget() ) )
			{
				CYPRESS120_DEV_DUG("---IsspEraseTarget return =%d----\n", bStatus);
				bProcessState = ISSP_PROGRAM;
			}
			break;

		case ISSP_PROGRAM: 	
			CYPRESS120_DEV_DUG("---IsspProgram ISSP_PROGRAM----\n");
			// Program Target
			if ( ISSP_PASS == ( bStatus = IsspProgramTarget() ) )
			{
				CYPRESS120_DEV_DUG("---IsspProgramTarget return =%d----\n", bStatus);
				//bProcessState = ISSP_VERIFY;   //nielimin  00164272
				bProcessState = ISSP_SECURITY;
			}
			break;
		#if 0   
		case ISSP_VERIFY: 	
			CYPRESS120_DEV_DUG("---IsspProgram ISSP_VERIFY----\n");
			// Program Target
			if ( ISSP_PASS == ( bStatus = IsspVerifyTarget() ) )
			{
				CYPRESS120_DEV_DUG("---IsspVerifyTarget return =%d----\n", bStatus);
				bProcessState = ISSP_SECURITY;
			}
			break;
		#endif
		case ISSP_SECURITY:
			CYPRESS120_DEV_DUG("---IsspProgram ISSP_SECURITY----\n");
			// Secure Target
			if ( ISSP_PASS == ( bStatus = IsspSecureTarget() ) )
			{
				CYPRESS120_DEV_DUG("---IsspSecureTarget return =%d----\n", bStatus);
				bProcessState = ISSP_CHECKSUM;
			}
			break;
		
		case ISSP_CHECKSUM: 	
			CYPRESS120_DEV_DUG("---IsspProgram ISSP_CHECKSUM----\n");
			if ( ISSP_PASS == ( bStatus = IsspCheckSumTarget() ) )
			{
				CYPRESS120_DEV_DUG("---IsspCheckSumTarget return =%d----\n", bStatus);
				bProcessState = ISSP_RESTART;
			}
			break;

		case ISSP_RESTART: 	
			CYPRESS120_DEV_DUG("---IsspProgram ISSP_RESTART----\n");
			IsspReStartTarget();
			bProcessState = ISSP_DONE;
			break;
		}					
	} 
	CYPRESS120_DEV_DUG("---IsspProgram return =%d----\n", bStatus);
	return ( bStatus );
}

/*
 ** IsspChecksum
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     Checksum location pointer
 *
 *  DESCRIPTION:    Initializes, Verifies Silicopn Id and Checksums the target device
 *
 *  RETURNS:        ISSP_PASS or one of the ISSP Error Codes
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Create
 */
signed char IsspChecksum( unsigned int *pCSum )
{
unsigned char bProcessState = ISSP_INITIALIZE;

unsigned char bStatus = ISSP_PASS;

	while ( ( ISSP_PASS == bStatus ) && ( ISSP_DONE != bProcessState ) )
	{
		IsspDriverDelay( 0xff );
		
		switch ( bProcessState )
		{
		default:
			bStatus = ISSP_ERROR;
			break;

		case ISSP_INITIALIZE: 			
			// Implements the intialization vectors for the device.
			if ( ISSP_PASS == ( bStatus = IsspXRESInitializeTarget() ) )
			{
				bProcessState = ISSP_IDENTIFY;
			}
			break;

		case ISSP_IDENTIFY: 			
			// Verify the device ID
			if ( ISSP_PASS == ( bStatus = IsspIdentifyTarget() ) )
			{
				bProcessState = ISSP_CHECKSUM;
			}
			break;
		
		case ISSP_CHECKSUM: 			
			if ( ISSP_PASS == ( bStatus = IsspCheckSumTarget() ) )
			{
				bProcessState = ISSP_RESTART;
			}
			break;

		case ISSP_RESTART: 			
			IsspReStartTarget();

			*pCSum = iCSum;
			bProcessState = ISSP_DONE;
			break;
		}					
	} 

	return ( bStatus );
}

/* ((((((((((((((((((((( HIGH-LEVEL ISSP ROUTINE SECTION ))))))))))))))))))))))
   (( These functions are mostly made of calls to the low level routines     ))
   (( above.  This should isolate the processor-specific changes so that     ))
   (( these routines do not need to be modified.                             ))
   (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))*/

/*
 ** IsspXRESInitializeTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     none
 *
 *  DESCRIPTION:    Implements the intialization vectors for the device.
 *
 *  RETURNS:        ISSP_PASS or ISSP_INIT_ERROR
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspXRESInitializeTarget( void )
{
unsigned char bError;

    // Configure the pins for initialization
    IsspDriverSetSDATAHiZ();
    IsspDriverSetSCLKStrong();

    IsspDriverSCLKLow();
    IsspDriverSetXRESStrong();
  
    // Cycle reset and put the device in programming mode when it exits reset
    IsspDriverAssertXRES();
    IsspDriverDelay( XRES_CLK_DELAY );
    IsspDriverDeassertXRES();

    // !!! NOTE: 
    //  The timing spec that requires that the first Init-Vector happen within
    //  1 msec after the reset/power up. For this reason, it is not advisable
    //  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the 
    //  Init-Vector instructions below. Doing so could introduce excess delay
    //  and cause the target device to exit ISSP Mode.

    // Send Initialization Vectors and detect Hi-Lo transition on SDATA
    IsspSendVector( init1_v, num_bits_init1 ); 

    if ( bError = IsspDetectHiLoTransition() )
    {
        return( ISSP_INIT_ERROR );
    }

    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );

    // Send Initialize 2 Vector
    IsspSendVector( init2_v, num_bits_init2 );

    if ( bError = IsspDetectHiLoTransition() )
    {
        return( ISSP_INIT_ERROR );
    }
    
    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );      

#ifdef TARGET_VOLTAGE_IS_5V
	IsspSendVector( init3_5v, num_bits_init3_5v );         	// Target Vdd = 5v
#else
	IsspSendVector( init3_3v, num_bits_init3_3v );          // Target Vdd = 3.3v
#endif

    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );

    // NOTE: DO NOT not wait for HiLo on SDATA after vector Init-3
    //       it does not occur (per spec).
    return( ISSP_PASS );
}

/*
 ** IsspIdentifyTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     none
 *
 *  DESCRIPTION:    Reads and verifies target silicon ID
 *
 *  RETURNS:        ISSP_PASS or ISSP_SiID_ERROR
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspIdentifyTarget( void )
{
	unsigned char bError;

	unsigned int iTargetID;

    // Send ID-Setup vector set
    IsspSendVector( id_setup_v, num_bits_id_setup );
    
    if ( bError = IsspDetectHiLoTransition() )
    {
    	CYPRESS120_DEV_DUG("---[%s] IsspDetectHiLoTransition error----\n",__FUNCTION__);
        return( ISSP_SiID_ERROR );
    }
    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );     

    //Send Read ID vector and get Target ID
    IsspSendVector( read_id_v, 11 );      	// Read-MSB Vector is the first 11-Bits
    IsspRunClock( 2 );                    	// Two SCLK cycles between write & read

    iTargetID = ( IsspReceiveByte() << 8 );

    IsspRunClock( 1 );
    IsspSendVector( read_id_v + 2, 12 );    // 12 bits starting from the 3rd character

    IsspRunClock( 2 );                    	// Read-LSB Command
    iTargetID |= IsspReceiveByte();

    IsspRunClock( 1 );
    IsspSendVector( read_id_v + 4, 1 );     // 1 bit starting from the 5th character

	//return( ISSP_PASS );				// Correct part ID  nielimin 00164272
	
	if ( iTargetID == target_id )
 	{ 
 		return( ISSP_PASS );				// Correct part ID
 	}
 	else
 	{
    	CYPRESS120_DEV_DUG("---[%s] iTargetID =0x%x, target_id=0x%x----\n",__FUNCTION__,iTargetID,target_id);
 		return( ISSP_SiID_ERROR );
 	}
}

/*
 ** IsspEraseTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     none
 *
 *  DESCRIPTION:    Perform a bulk erase of the target device.
 *
 *  RETURNS:        ISSP_PASS or ISSP_ERASE_ERROR
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspEraseTarget( void )
{
	unsigned char bError;

	IsspSendVector( erase_all_v, num_bits_erase_all );

    if ( bError = IsspDetectHiLoTransition() )
    {
        return( ISSP_ERASE_ERROR );
    }

    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );

    return( ISSP_PASS );
}

/*
 ** IsspProgramTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     none
 *
 *  DESCRIPTION:    Perform programming of the target device.
 *
 *  RETURNS:        ISSP_PASS or one of the ISSP Error Codes
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspProgramTarget( void )
{
	unsigned char bBankNumber,
	bStatus = ISSP_PASS;

	unsigned int iBlockNum;

	const unsigned char *pHexData = caPsocIsspTable;	
   	CYPRESS120_DEV_DUG("---[%s] number_of_banks = %d, blocks_per_bank = %d----\n",__FUNCTION__,number_of_banks,blocks_per_bank);
	for ( bBankNumber = 0; bBankNumber < number_of_banks; bBankNumber++ )
	//for ( bBankNumber = 0; bBankNumber < 1; bBankNumber++ )
	{   
		//CYPRESS120_DEV_DUG("---[%s] bBankNumber = %d----\n", __FUNCTION__, bBankNumber);

		IsspSetBankNumber( bBankNumber );	// Set the bank number

		for ( iBlockNum = 0; iBlockNum < blocks_per_bank; iBlockNum++ )
		{  
			// Check the record type
			if ( pHexData[ ISSP_RECORD_TYPE ] != ISSP_RECORD_TYPE_DATA )
			{
				return ( bStatus );			// We are done !
			}
		
			// Load the target data
			(void)IsspLoadTarget( &pHexData[ ISSP_RECORD_DATA ] );

			// send programming cmd to target
			if ( bStatus = IsspProgramTargetBlock( (unsigned char)iBlockNum ) )
			{
				return ( bStatus );			// ERROR, We are done !
			}
			
			pHexData += *pHexData + RECORD_OVERHEAD;
		}
	} 
    return( bStatus );
}

/*
 ** IsspLoadTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     Pointer to the recorxd data
 *
 *  DESCRIPTION:    Transfers data from array in Host to RAM buffer in the target.
 *
 *  RETURNS:        Returns the checksum of the record data.
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
unsigned char IsspLoadTarget( const unsigned char *pHexData )
{
unsigned char bChecksumData = 0,
			  bTemp,
			  bTargetAddress = 0,
			  bTargetDataIndex = 0;
    // Set SDATA to Strong Drive here because IsspSendByte() does not
    IsspDriverSetSDATAStrong();

    // Transfer the temporary RAM array into the target.
    // In this section, a 64-unsigned char array was specified by #define, so the entire
    // 64-unsigned chars are written in this loop.
    while( bTargetDataIndex < TARGET_DATABUFF_LEN )
    {   
        bTemp = pHexData[ bTargetDataIndex ];		
		//if(0 == bTargetDataIndex)
		//	CYPRESS120_DEV_DUG("pHexData[0] = 0x%02x\n ",bTemp   );
		//CYPRESS120_DEV_DUG("bTargetAddress = 0x%02x ",  bTargetAddress>>2 );
        bChecksumData += bTemp;

        IsspSendByte( write_byte_start, 5 );     
        IsspSendByte( bTargetAddress, 6 );
        IsspSendByte( bTemp, 8 );
        IsspSendByte( write_byte_end, 3 );
               
        // !!!NOTE:
        // IsspSendByte() uses MSbits, so inc by '4' to put the 0..63 address into
        // the six MSBit locations.
        //
        // This can be confusing, but check the logic:
        //   The address is only 6-Bits long. The IsspSendByte() subroutine will
        // send however-many bits, BUT...always reads them bits from left-to-
        // right. So in order to pass a value of 0..63 as the address using
        // IsspSendByte(), we have to left justify the address by 2-Bits.
        //   This can be done easily by incrementing the address each time by
        // '4' rather than by '1'.

        bTargetAddress += 4;
        bTargetDataIndex++;
    }
	//CYPRESS120_DEV_DUG("\n");
    return ( bChecksumData );
}

/*
 ** IsspProgramTargetBlock
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     Block Number
 *
 *  DESCRIPTION:    Program one block with data that has been loaded into a RAM buffer
 *					in the target device.
 *
 *  RETURNS:        ISSP_PASS or ISSP_BLOCK_ERROR
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspProgramTargetBlock( unsigned char bBlockNumber )
{
	unsigned char bError;
	//CYPRESS120_DEV_DUG("---[%s]  bBlockNumber = %d----\n", __FUNCTION__, bBlockNumber);
	
    // Send the block-select vector.
    IsspSendVector( set_block_number, 11 );

    // Set the drive here because IsspSendByte() does not.
    IsspDriverSetSDATAStrong();
    IsspSendByte( bBlockNumber, 8 );
    IsspSendByte( set_block_number_end, 3 );

    // Send the program-block vector. Vector selection based on device type	*/
  	IsspSendVector( program_block, num_bits_program_block );
    
    // wait for acknowledge from target.
    if ( bError = IsspDetectHiLoTransition() )
    {
        return( ISSP_BLOCK_ERROR );
    }

    // Send the Wait-For-Poll-End vector
    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );
	mdelay(100);
    return( ISSP_PASS );
}

/*
 ** IsspVerifyTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Perform verification of the target device.
 *
 *  RETURNS:        ISSP_PASS or one of the ISSP Error Codes
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspVerifyTarget( void )
{
unsigned char bBankNumber,
			  bStatus = ISSP_PASS;

unsigned int iBlockNum;

const unsigned char *pHexData = caPsocIsspTable;	

	for ( bBankNumber = 0; bBankNumber < number_of_banks; bBankNumber++ )
	{   
		IsspSetBankNumber( bBankNumber );	// Set the bank number

		for ( iBlockNum = 0; iBlockNum < blocks_per_bank; iBlockNum++ )
		{  
			// Check the record type
			if ( pHexData[ ISSP_RECORD_TYPE ] != ISSP_RECORD_TYPE_DATA )
			{
				return ( bStatus );			// We are done !
			}
			//CYPRESS120_DEV_DUG("---[%s]  bBankNumber = %d iBlockNum = %d ----\n",
			//__FUNCTION__, bBankNumber, iBlockNum );
			// Verify the target block
			if ( bStatus = IsspVerifyTargetBlock( (unsigned char)iBlockNum, &pHexData[ ISSP_RECORD_DATA ] ) )
			{
				return ( bStatus );			// ERROR, We are done !
			}
			
			pHexData += *pHexData + RECORD_OVERHEAD;
		}
	} 
    return( bStatus );
}

/*
 ** IsspVerifyTargetBlock
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     Block Number, Host record character array pointer
 *
 *  DESCRIPTION:    Verify the block just written to. This can be done byte-by-byte
 *					before the protection bits are set.
 *
 *  RETURNS:        ISSP_PASS or ISSP_BLOCK_ERROR
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspVerifyTargetBlock( unsigned char bBlockNumber, const unsigned char *pHexData )
{
unsigned char bError,
			  bTargetDataIN,
			  bTargetAddress = 0,
			  bTargetDataIndex = 0;
	mdelay(100);
    IsspSendVector( set_block_number, 11 );

    // Set the drive here because IsspSendByte() does not
    IsspDriverSetSDATAStrong();
    IsspSendByte( bBlockNumber, 8 );
    IsspSendByte( set_block_number_end, 3 );
     
    IsspSendVector( verify_setup_v, num_bits_verify_setup );
    
    if ( bError = IsspDetectHiLoTransition() )
    {
        return( ISSP_BLOCK_ERROR );
    }

    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );     
     
    while( bTargetDataIndex < TARGET_DATABUFF_LEN )
    {	
    	
        //Send Read unsigned char vector and then get a byte from Target
        IsspSendVector( read_byte_v, 5 );

        // Set the drive here because IsspSendByte() does not
        IsspDriverSetSDATAStrong();
        IsspSendByte( bTargetAddress, 6 );
		udelay(5);
        IsspRunClock(2);       // Run two SCLK cycles between writing and reading
        IsspDriverSetSDATAHiZ();     // Set to HiZ so Target can drive SDATA

        bTargetDataIN = IsspReceiveByte();
        IsspRunClock( 1 );

        IsspSendVector( read_byte_v + 1, 1 );     // Send the Read Vector End

		//CYPRESS120_DEV_DUG("---[%s]  bTargetDataIN = 0x%02x pHexData[ %d ]=0x%02x ----\n",
		//	__FUNCTION__,bTargetDataIN,bTargetDataIndex,pHexData[ bTargetDataIndex ]);
		
        // Test the unsigned char that was read from the Target against the original
        // value (already in the 64-unsigned char array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
        if ( bTargetDataIN != pHexData[ bTargetDataIndex ] )
        {
            return( ISSP_BLOCK_ERROR );
		}
		
        bTargetDataIndex++;

        // Increment the address by four to accomodate 6-Bit addressing
        // (puts the 6-bit address into MSBit locations for "IsspSendByte()").
        bTargetAddress += 4;
    }
    return( ISSP_PASS );
}

/*
 ** IsspSecureTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Perform securing of the target device.
 *
 *  RETURNS:        ISSP_PASS or one of the ISSP Error Codes
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspSecureTarget( void )
{
unsigned char bBankNumber,
			  bStatus = ISSP_PASS;

const unsigned char *pHexData = caPsocIsspTable;

	// Find the last data record
	while ( pHexData[ ISSP_RECORD_TYPE ] == ISSP_RECORD_TYPE_DATA )
	{
		pHexData += pHexData[ 0 ] + RECORD_OVERHEAD;
	}

	// Found last data record
	// Find the first secure record
	while ( pHexData[ ISSP_RECORD_TYPE ] != ISSP_RECORD_TYPE_DATA )
	{
		pHexData += pHexData[ 0 ] + RECORD_OVERHEAD;
	}

	// Found security record

	for ( bBankNumber = 0; bBankNumber < number_of_banks; bBankNumber++ )
	{   
		IsspSetBankNumber( bBankNumber );	// Set the bank number

		if ( security_bytes_per_bank == 32 )
		{
			if ( bBankNumber & 0x01 )
 			{
				// send secure cmd to target
				bStatus = IsspSecureTargetFlash( &pHexData[ ISSP_RECORD_DATA + security_bytes_per_bank ] );

				// Adjust to next record
				pHexData += *pHexData + RECORD_OVERHEAD;
			}
			else
			{
				// send secure cmd to target
				bStatus = IsspSecureTargetFlash( &pHexData[ ISSP_RECORD_DATA ] );
			}
		}
		else
		{
			// send secure cmd to target
			bStatus = IsspSecureTargetFlash( &pHexData[ ISSP_RECORD_DATA ] );

			// Adjust to next record
			pHexData += *pHexData + RECORD_OVERHEAD;
		}	

		if ( bStatus != ISSP_PASS )
		{
			return ( bStatus );				// ERROR, We are done !
		}
	} 
    return( bStatus );
}

/*
 ** IsspSecureTargetFlash
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     Host record character array pointer
 *
 *  DESCRIPTION:    This can be called multiple times with different SecurityTypes
 *					as needed for particular Flash Blocks..
 *
 *  RETURNS:        ISSP_PASS or ISSP_SECURITY_ERROR
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspSecureTargetFlash( const unsigned char *pHexData )
{
unsigned char bTemp,
			  bError,
			  bTargetAddress = 0,
			  bTargetDataIndex = 0;

    // Transfer the temporary RAM array into the target
    IsspDriverSetSDATAStrong();

    while( bTargetDataIndex < security_bytes_per_bank )
    {     
        bTemp = pHexData[ bTargetDataIndex ];
        IsspSendByte( write_byte_start, 5 );     
        IsspSendByte( bTargetAddress, 6 );
        IsspSendByte( bTemp, 8 );
        IsspSendByte( write_byte_end, 3 );
           
        // IsspSendBytes() uses MSBits, so increment the address by '4' to put
        // the 0..n address into the six MSBit locations
        bTargetAddress += 4;
        bTargetDataIndex++;
    }

    IsspSendVector( security_v, num_bits_security );

    if ( bError = IsspDetectHiLoTransition() )
    {
        return( ISSP_SECURITY_ERROR );
    }

    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );
    
    return( ISSP_PASS );
}

/*
 ** IsspCheckSumTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Perform checksum verification of the target device.
 *
 *  RETURNS:        ISSP_PASS or one of the ISSP Error Codes
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspCheckSumTarget( void )
{
unsigned char bBankNumber,
			  bStatus = ISSP_PASS;

const unsigned char *pHexData = caPsocIsspTable;

	iCSum = 0;

	for ( bBankNumber = 0; bBankNumber < number_of_banks; bBankNumber++ )
	{   
		IsspSetBankNumber( bBankNumber );	// Set the bank number

		// send checksum cmd to target
		if ( bStatus = IsspTargetBankChecksum( &iCSum ) )
		{
			return ( bStatus );			// ERROR, We are done !
		}
	} 

	// Find the checksum record
	if ( ( *pHexData                    != 0x02 ) &&					// Record data length
		 ( pHexData[ ISSP_RECORD_TYPE ] != ISSP_RECORD_TYPE_DATA ) )	// Record type
	{
		pHexData += *pHexData + RECORD_OVERHEAD;
	}

    return( bStatus );
}

/*
 ** IsspTargetBankChecksum
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     Checksum location pointer
 *
 *  DESCRIPTION:    Reads and adds the target bank checksum to the referenced accumulator.
 *
 *  RETURNS:        ISSP_PASS or ISSP_VERIFY_ERROR
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspTargetBankChecksum( unsigned int *pCSum )
{
	unsigned char bError;

    IsspSendVector( checksum_v, num_bits_checksum ); 

    if ( bError = IsspDetectHiLoTransition() )
    {
        return( ISSP_VERIFY_ERROR );
    }
	
    IsspSendVector( wait_and_poll_end, num_bits_wait_and_poll_end );     

    //Send Read Checksum vector and get Target Checksum
    IsspSendVector( read_checksum_v, 11 );     // first 11-bits is ReadCKSum-MSB
    IsspRunClock( 2 );                         // Two SCLKs between write & read

    *pCSum += (int)IsspReceiveByte() << 8;
    IsspRunClock( 1 );                         // See Fig. 6

    IsspSendVector(read_checksum_v + 2, 12 );  // 12 bits starting from 3rd character
    IsspRunClock( 2 );                         // Read-LSB Command

    *pCSum += IsspReceiveByte();
    IsspRunClock( 1 );

    IsspSendVector( read_checksum_v + 3, 1 );  // Send the final bit of the command

    return( ISSP_PASS );    
}    

/*
 ** IsspSetBankNumber
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     Bank Number
 *
 *  DESCRIPTION:    Set the bank number in the target device.
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
#ifdef MULTI_BANK
void IsspSetBankNumber( unsigned char bBankNumber )
{
    // Send the bank-select vector.
    IsspSendVector( set_bank_number, 33 );

    // Set the drive here because IsspSendByte() does not.
    IsspDriverSetSDATAStrong();
    IsspSendByte( bBankNumber, 8 );
    IsspSendVector( set_bank_number_end, 25 );
	mdelay(100);
}
#endif /* MULTI_BANK */

/*
 ** IsspReStartTarget
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    After programming, the target PSoC must be reset to take it
 *					out of programming mode. This routine performs a reset.
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
void IsspReStartTarget( void )
{
    // Assert XRES, then release, then disable XRES-Enable
    IsspDriverAssertXRES();
    IsspDriverDelay( XRES_CLK_DELAY );
    IsspDriverDeassertXRES();
}


/* ((((((((((((((((((((( LOW-LEVEL ISSP SUBROUTINE SECTION ))))))))))))))))))))
   (( The subroutines in this section use functions from the C file          ))
   (( ISSP_Drive_Routines.c. The functions in that file interface to the     ))
   (( processor specific hardware. So, these functions should work as is, if ))
   (( the routines in ISSP_Drive_Routines.c are correctly converted.         ))
   (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))*/

/*
 ** IsspRunClock
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     Number of cycles
 *
 *  DESCRIPTION:    Run Clock without sending/receiving bits. Use this when
 *					transitioning from write to read and read to write "num_cycles"
 *					is number of SCLK cycles, not number of counter	cycles.
 *
 *	NOTE...			SCLK cannot run faster than the specified maximum frequency
 *					of 8MHz. Some processors may need to have delays added after
 *					setting SCLK low and setting SCLK high in order to not exceed
 *					this specification. The maximum frequency of SCLK should be
 *					measured as part of validation of the final program
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
void IsspRunClock( unsigned int iNumCycles )
{
int i;

    for( i = 0; i < iNumCycles; i++ )
    {
		IsspDriverSCLKLow();		
		IsspDriverSCLKHigh();
    }
    // function exits with CLK high.
}

/*
 ** IsspReceiveBit
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Clocks the SCLK pin (high-low-high) and reads the status of
 *					the SDATA pin after the rising edge.
 *
 *	NOTE...			SCLK cannot run faster than the specified maximum frequency
 *					of 8MHz. Some processors may need to have delays added after
 *					setting SCLK low and setting SCLK high in order to not exceed
 *					this specification. The maximum frequency of SCLK should be
 *					measured as part of validation of the final program
 *
 *  RETURNS:        0 if SDATA was low
 *					1 if SDATA was high
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
unsigned char IsspReceiveBit( void )
{	
	IsspDriverSCLKLow();		
	IsspDriverSCLKHigh();

	return ( IsspDriverSDATACheck() );
}          

/*
 ** IsspReceiveByte
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Calls ReceiveBit 8 times to receive one byte.
 *
 *  RETURNS:        The 8-bit values recieved.
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
unsigned char IsspReceiveByte( void )
{
	unsigned char b,
	bCurrentByte = 0;
    for ( b = 0; b < 8; b++ )
    {           
        bCurrentByte = ( bCurrentByte << 1 ) + IsspReceiveBit();
    }
    return( bCurrentByte );
}          

/*
 ** IsspSendByte
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     bCurrentByte - the byte that contains the bits to be sent.
 *					bSize        - the number of bits to be sent.
 *								   Valid values are 1 to 8.
 *
 *  DESCRIPTION:    This routine sends up to one byte of a vector, one bit at a time.
 *
 *
 *	NOTE...			SCLK cannot run faster than the specified maximum frequency
 *					of 8MHz. Some processors may need to have delays added after
 *					setting SCLK low and setting SCLK high in order to not exceed
 *					this specification. The maximum frequency of SCLK should be
 *					measured as part of validation of the final program
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
void IsspSendByte( unsigned char bCurrentByte, unsigned char bSize )
{
	unsigned char b;
	
    for ( b = 0; b < bSize; b++ )
    {
        if ( bCurrentByte & 0x80 )
        {
            // Send a '1'
            IsspDriverSetSDATAHigh();		
        }
        else
        {
            // Send a '0'	
            IsspDriverSetSDATALow();		
        }
 
        IsspDriverSCLKHigh();	
		IsspDriverSCLKLow();

        bCurrentByte = bCurrentByte << 1;
    }
}

/*
 ** IsspSendVector
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     bVect    - a pointer to the vector to be sent.
 *    				iNumBits - the number of bits to be sent.
 *
 *  DESCRIPTION:    This routine sends the vector specifed. All vectors constant
 *					strings found in ISSP_Vectors.h.  The data line is returned
 *					to HiZ after the vector is sent.
 *
 *  RETURNS:        None
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
void IsspSendVector( const unsigned char* bVect, unsigned int iNumBits )
{
    IsspDriverSetSDATAStrong();
   	
    while( iNumBits > 0 )
    {
        if ( iNumBits >= 8 )
        {
            IsspSendByte( *(bVect), 8 );
            iNumBits -= 8;
            bVect++;
        }
        else
        {
            IsspSendByte( *(bVect), iNumBits );
            iNumBits = 0;
        }
    }

    IsspDriverSetSDATAHiZ();
}

/*
 ** IsspDetectHiLoTransition
 *
 *  FILENAME: 		Issp_Routines.c
 *
 *  PARAMETERS:     None
 *
 *  DESCRIPTION:    Waits for transition from SDATA = 1 to SDATA = 0. Has a 100 msec
 *					timeout. TRANSITION_TIMEOUT is a loop counter for a 100 msec
 *					timeout when waiting for a high-to-low transition. This is used
 *					in the polling loop of IsspDetectHiLoTransition(). The timing
 *					of the while(1) loops can be calculated and the number of loops
 *					is counted, using iTimer, to determine when 100 msec has passed.
 *
 *	NOTE...			SCLK cannot run faster than the specified maximum frequency
 *					of 8MHz. Some processors may need to have delays added after
 *					setting SCLK low and setting SCLK high in order to not exceed
 *					this specification. The maximum frequency of SCLK should be
 *					measured as part of validation of the final program
 *
 *  RETURNS:        ISSP_PASS or ISSP_ERROR
 *
 *  Date       Int Description
 *  ---------- --- ---------------------------------------------------------------
 *  03/25/2008 MDS Modified
 *
 */
signed char IsspDetectHiLoTransition( void )
{
// nTimer breaks out of the while loops if the wait in the two loops totals
// more than 100 msec.  Making this static makes the loop run a faster.
// This is really a processor/compiler dependency and it not needed.
static unsigned int iTimer;
    
// NOTE:
// These loops look unconventional, but it is necessary to check SDATA_PIN
// as shown because the transition can be missed otherwise, due to the
// length of the SDATA Low-High-Low after certain commands.

    // Generate clocks for the target to pull SDATA High
     iTimer = TRANSITION_TIMEOUT;

     while( 1 )
     {
        IsspDriverSCLKLow();

        if ( IsspDriverSDATACheck() )      		// exit once SDATA goes HI
		{
            break;
        }
        
        IsspDriverSCLKHigh();
        
        // If the wait is too long then timeout
        if ( iTimer-- == 0 )
        {
            return ( ISSP_ERROR );
        }
    }
    
    // Generate Clocks and wait for Target to pull SDATA Low again
    iTimer = TRANSITION_TIMEOUT;              // reset the timeout counter

    while( 1 )
    {
        IsspDriverSCLKLow();

        if ( !IsspDriverSDATACheck() )			   	// exit once SDATA returns LOW
        { 
            break;
        }

        IsspDriverSCLKHigh();

        // If the wait is too long then timeout
        if ( iTimer-- == 0 )
        {
            return ( ISSP_ERROR );
        }
    }
    return ( ISSP_PASS );
}

#endif  //(PROJECT_REV_)

// end of file ISSP_Routines.c
