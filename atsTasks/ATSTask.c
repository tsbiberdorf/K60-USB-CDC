/*
 * ADCTask.c
 *
 *  Created on: Oct 14, 2013
 *      Author: tsbiberdorf
 */

#include <stdio.h>
#include <time.h>
#include "arm_cm4.h"
#include "Initialize.h"

/*
 * to be placed in Vectors.c
PE_ISR(isrSWI);
PE_ISR(isrPDB);
PE_ISR(isrADC0);
*/
//    (tIsrFunc)&isrADC0,                /* 0x49  0x00000124   0   ivINT_ADC0                     used by PE */
//    (tIsrFunc)&isrPDB,                 /* 0x58  0x00000160   0   ivINT_PDB0                     used by PE */
//    (tIsrFunc)&isrSWI,          /* 0x6E  0x000001B8   -   ivINT_SWI                      unused by PE */


 
void isrPDB()
{
	PDB0_SC &= ~PDB_SC_PDBIF_MASK ;  // clear interrupt mask
	int32_t  x;
	x++;
	SetPTD12();
	ClrPTD12();
}

void isrADC0()
{
	int32_t  x;
	static int32_t swiTriggerCnt = 0;
	x = ADC0_RA;
	SetPTD13();
	if( ++swiTriggerCnt > 100 )
		{
			swiTriggerCnt = 0;
			
			/* trigger SWI IRQ */
//			NVICSTIR = (INT_SWI-16);
		}
	ClrPTD13();
}

void isrSWI()
{
	int32_t  x;
	x++;
	
}

void atsTask()
{
	struct tm myTime,*ptrMyTime;
	time_t timeStamp;
	int32_t  x;
	x++;
	timeStamp++;
	
	ptrMyTime = &myTime;
	ptrMyTime = gmtime(&timeStamp);
	
	if(x>500)
	{
		NegYellowLED();
		x=0;
	}
}
