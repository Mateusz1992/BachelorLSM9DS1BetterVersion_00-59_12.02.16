/*
 * Main.c
 *
 *  Created on: 08-02-2016
 *      Author: Mateusz
 */


#include <DAVE3.h>			//Declarations from DAVE3 Code Generation (includes SFR declaration)

#include "LSM9DS1_Driver.h"
#include "Timer.h"

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
#define TRUE 1
#define FALSE 0

void initLSM9DS1(void);
void timerHandlerReadByte(void *T);
void timerHandlerReadByte1(void *T);


typedef struct
{
	uint8_t addressDevice;
	uint8_t subAddress;
}deviceAddress;

typedef struct
{
	uint8_t dane[6];
	deviceAddress adr;
}addressAndData;

volatile bool readingAllowed = TRUE;

accel pomiary[20];

#define DELAY 10000

int main(void)
{
//	status_t status;		// Declaration of return variable for DAVE3 APIs (toggle comment if required)


	DAVE_Init();			// Initialization of DAVE Apps


	int counter = 0;
	handle_t TimerId;
	uint32_t Status = SYSTM001_ERROR;

	addressAndData adrAndData;
	adrAndData.adr.addressDevice = 0x6B;
	adrAndData.adr.subAddress =  OUT_X_L_XL;

	initLSM9DS1();
	calibrate(TRUE);

	//readAccel1();
	//makeTimer(100, SYSTM001_PERIODIC, timerHandlerReadByte1, &a, &Status, &TimerId);
	TimerId=SYSTM001_CreateTimer(5,SYSTM001_PERIODIC,timerHandlerReadByte1,&adrAndData);
	SYSTM001_StartTimer(TimerId);
	while(1)
	{
		if(!readingAllowed)
		{


			int16_t accelX = (a.dane[1] << 8) | a.dane[0]; // Store x-axis values into gx

			int16_t accelY = (a.dane[3] << 8) | a.dane[2]; // Store y-axis values into gy

			int16_t accelZ = (a.dane[5] << 8) | a.dane[4]; // Store z-axis values into gz

			if (_autoCalc) //kalibracja
			{
				accelX -= aBiasRaw[X_AXIS];
				accelX -= aBiasRaw[Y_AXIS];
				accelX -= aBiasRaw[Z_AXIS];
			}

			accelX = calcAccel(accelX);
			accelY = calcAccel(accelY);
			accelZ = calcAccel(accelZ);

			pomiary[counter].ax = accelX;
			pomiary[counter].ay = accelY;
			pomiary[counter].az = accelZ;
			counter++;
			readingAllowed = TRUE;
		}

		if(counter >= 20)
		{
			counter = 0;
		}

	}
	return 0;
}

void initLSM9DS1(void)
{
	init(IMU_MODE_I2C, LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1));

	settings.device.commInterface = IMU_MODE_I2C;
	settings.device.mAddress = LSM9DS1_M;
	settings.device.agAddress = LSM9DS1_AG;

	if(!begin())
	{
		int k = 0;
	}
}

void timerHandlerReadByte1(void *T)
{
	static volatile uint32_t stageOfReading = 0;
	static uint8_t whichByte = 0;
	addressAndData *address = (addressAndData*)T;

	if(readingAllowed == TRUE)
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

		USIC_CH_TypeDef* I2CRegs = I2C001_Handle0.I2CRegs;

		I2C001_DataType data1;
		data1.Data1.TDF_Type = I2C_TDF_MStart;
		uint8_t adr = address->adr.addressDevice;
		data1.Data1.Data = ((adr << 1) | I2C_WRITE);
		while(!I2C001_WriteData(&I2C001_Handle0,&data1))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}

		stageOfReading++;
		delay(DELAY);

		I2C001_DataType data2;
		data2.Data1.TDF_Type = I2C_TDF_MTxData;
		uint8_t subAdr = (address->adr.subAddress + whichByte);
		data2.Data1.Data = subAdr;
		while(!I2C001_WriteData(&I2C001_Handle0,&data2))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}

		stageOfReading++;
		delay(DELAY);

		I2C001_DataType data3;
		data3.Data1.TDF_Type = I2C_TDF_MRStart;
		uint8_t adr1 = address->adr.addressDevice;
		data3.Data1.Data = ((adr1 << 1) | I2C_READ);
		while(!I2C001_WriteData(&I2C001_Handle0,&data3))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}

		stageOfReading++;
		delay(DELAY);

		I2C001_DataType data4;
		data4.Data1.TDF_Type = I2C_TDF_MRxAck1;
		data4.Data1.Data = ubyteFF;
		while(!I2C001_WriteData(&I2C001_Handle0,&data4))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}

		stageOfReading++;
		delay(DELAY);

		I2C001_DataType data5;
		data5.Data1.TDF_Type = I2C_TDF_MStop;
		data5.Data1.Data = ubyteFF;
		while(!I2C001_WriteData(&I2C001_Handle0,&data5))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}

		stageOfReading++;
		delay(DELAY);

		int k = 0;

		uint16_t buffer = 0;
		if(I2C001_ReadData(&I2C001_Handle0,&buffer))
		{
			k++;
		}
		else
		{
			k--;
		}
		delay(DELAY);
		address->dane[whichByte] = (uint8_t)buffer;
		stageOfReading = 0;
		whichByte++;

		if(whichByte == 6)
		{
			readingAllowed = FALSE;

			whichByte = 0;
			stageOfReading = 0;
		}
	}
}
