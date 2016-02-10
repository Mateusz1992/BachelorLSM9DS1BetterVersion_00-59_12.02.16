/*
 * Timer.c
 *
 *  Created on: 09-02-2016
 *      Author: Mateusz
 */

#include "Timer.h"

void makeTimer(uint32_t period, SYSTM001_TimerType TimerType, SYSTM001_TimerCallBackPtr TimerCallBack, void *pCallBackArgPtr, uint32_t *status, handle_t *timerID)
{
	*timerID = SYSTM001_CreateTimer(period,TimerType,TimerCallBack, pCallBackArgPtr);

	if(*timerID != 0)
	{
		*status = SYSTM001_StartTimer(*timerID);
	}
}


void removeTimer(uint32_t *status, handle_t *timerID)
{
	if(*timerID != 0)
	{
		*status = SYSTM001_StopTimer(*timerID);

		if(*status == DAVEApp_SUCCESS)
		{
			SYSTM001_DeleteTimer(*timerID);
			*timerID = 0;
		}
	}
}
