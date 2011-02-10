// filename: ISSP_Delays.h

#include "ISSP_Revision.h"
#ifdef PROJECT_REV_110

#ifndef _INC_ISSP_DELAYS_H_
#define _INC_ISSP_DELAYS_H_


#define DELAY_M    1

#define DELAY_B    3

#define TRANSITION_TIMEOUT     67400  

#define XRES_CLK_DELAY    250

#define POWER_CYCLE_DELAY ( ( 150 - DELAY_B ) / DELAY_M )

#define DELAY100us        ( ( 100 - DELAY_B ) / DELAY_M )

#endif //(_INC_ISSP_DELAYS_H_)
#endif //(PROJECT_REV_)

//end of file ISSP_Delays.h
