
#include "ISSP_Revision.h"
#ifdef PROJECT_REV_110

#ifndef _INC_ISSP_DEFS_H_
#define _INC_ISSP_DEFS_H_

#include "ISSP_Directives.h"

#define TARGET_DATABUFF_LEN    64 


#ifdef CY8C20x24
    #define NUM_BANKS          			( unsigned char )(   1 )
    #define BLOCKS_PER_BANK    			( unsigned int  )( 128 )
    #define SECURITY_BYTES_PER_BANK		( unsigned char )(  64 )
#endif

#ifdef CY8C20x34
    #define NUM_BANKS          			( unsigned char )(   1 )
    #define BLOCKS_PER_BANK    			( unsigned int  )( 128 )
    #define SECURITY_BYTES_PER_BANK		( unsigned char )(  64 )
#endif

#ifdef CY8C21x23
    #define NUM_BANKS          			( unsigned char )(   1 )
    #define BLOCKS_PER_BANK    			( unsigned int  )(  64 )
    #define SECURITY_BYTES_PER_BANK		( unsigned char )(  64 )
#endif

#ifdef CY8C21x34
    #define NUM_BANKS           		( unsigned char )(   1 )
    #define BLOCKS_PER_BANK   			( unsigned int  )( 128 )
    #define SECURITY_BYTES_PER_BANK		( unsigned char )(  64 )
#endif

#ifdef CY8C24x23A
    #define NUM_BANKS         			( unsigned char )(   1 )
    #define BLOCKS_PER_BANK   			( unsigned int  )(  64 )
    #define SECURITY_BYTES_PER_BANK		( unsigned char )(  64 )
#endif

#ifdef CY8C24x94
    #define NUM_BANKS         			( unsigned char )(   2 )
    #define BLOCKS_PER_BANK   			( unsigned int  )( 128 )
    #define SECURITY_BYTES_PER_BANK		( unsigned char )(  32 )
#endif

#ifdef CY8C27x43
    #define NUM_BANKS         			( unsigned char )(   1 )
    #define BLOCKS_PER_BANK   			( unsigned int  )( 256 )
    #define SECURITY_BYTES_PER_BANK		( unsigned char )(  64 )
#endif

#ifdef CY8C29x66
    #define NUM_BANKS         			( unsigned char )(   4 )
    #define BLOCKS_PER_BANK   			( unsigned int  )( 128 )
    #define SECURITY_BYTES_PER_BANK		( unsigned char )(  32 )
#endif

#endif //(_INC_ISSP_DEFS_H_)

#endif //(PROJECT_REV_)

