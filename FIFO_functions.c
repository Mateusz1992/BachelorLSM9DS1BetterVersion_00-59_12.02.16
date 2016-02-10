/*
 * FIFO_functions.c
 *
 *  Created on: 10-02-2016
 *      Author: Mateusz
 */
#include "FIFO_functions.h"

void flushFIFO(void)
{
	USIC_CH_TypeDef* I2CRegs = I2C001_Handle0.I2CRegs;
	USIC_FlushTxFIFO(I2CRegs);
}

void clearErrorFlags(void)
{
	if(USIC1_CH1->PSR_IICMode & (USIC_CH_PSR_IICMode_ERR_Msk | USIC_CH_PSR_IICMode_NACK_Msk))
	{
		// Clear error bits
		USIC1_CH1->PSCR |= 0x3FF;
		// Flush transmit FIFO buffer
		USIC1_CH1->TRBSCR |= USIC_CH_TRBSCR_FLUSHTB_Msk;
		// Modify Transmit Data Valid
		WR_REG(USIC1_CH1->FMR, USIC_CH_FMR_MTDV_Msk, USIC_CH_FMR_MTDV_Pos, 2);
	}
}

