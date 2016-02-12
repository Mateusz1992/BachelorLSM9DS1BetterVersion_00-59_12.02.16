/*
 * Main.c
 *
 *  Created on: 08-02-2016
 *      Author: Mateusz
 */


#include <DAVE3.h>			//Declarations from DAVE3 Code Generation (includes SFR declaration)

#include "LSM9DS1_Driver.h"
#include "Timer.h"

void timerHandlerReadByte(void *T);


int main(void)
{
//	status_t status;		// Declaration of return variable for DAVE3 APIs (toggle comment if required)


	DAVE_Init();			// Initialization of DAVE Apps


	initLSM9DS1();
	calibrate(TRUE);

	startMeasurements();

	while(1)
	{
		readAndSendMeasurements(NULL);
	}
	return 0;
}
