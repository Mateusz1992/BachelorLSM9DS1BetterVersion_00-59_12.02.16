/*
 * Timer.h
 *
 *  Created on: 09-02-2016
 *      Author: Mateusz
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <DAVE3.h>

void makeTimer(uint32_t period, SYSTM001_TimerType TimerType, SYSTM001_TimerCallBackPtr TimerCallBack, void *pCallBackArgPtr, uint32_t *status, handle_t *timerID);
void removeTimer(uint32_t *status, handle_t *timerID);

#endif /* TIMER_H_ */
