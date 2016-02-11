/*
 * Main.c
 *
 *  Created on: 08-02-2016
 *      Author: Mateusz
 */


#include <DAVE3.h>			//Declarations from DAVE3 Code Generation (includes SFR declaration)

#include "LSM9DS1_Driver.h"
#include "Timer.h"
#include "FIFO_functions.h"

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
#define TRUE 1
#define FALSE 0

void initLSM9DS1(void);
void timerHandlerReadByte(void *T);
void timerHandlerReadByte1(void *T);

void receiveByte(uint8_t adr, uint8_t subAdr, uint8_t *buffer);


typedef struct
{
	uint8_t addressDevice[2];
	uint8_t subAddress[3];
}deviceAddress;

typedef struct
{
	uint8_t dane[18];
	deviceAddress adr;
}addressAndData;

volatile bool readingAllowed = TRUE;

accel pomiary[100];
accel pomiary1[100];


#define DELAY 10000

int main(void)
{
//	status_t status;		// Declaration of return variable for DAVE3 APIs (toggle comment if required)


	DAVE_Init();			// Initialization of DAVE Apps


	int counter = 0;
	handle_t TimerId;
	uint32_t Status = SYSTM001_ERROR;

	addressAndData adrAndData;
	adrAndData.adr.addressDevice[0] = 0x6B;
	adrAndData.adr.addressDevice[1] = 0x1E;
	adrAndData.adr.subAddress[0] =  OUT_X_L_XL; //subaddres for accel
	adrAndData.adr.subAddress[1] =  OUT_X_L_G; //sub address for gyroscope
	adrAndData.adr.subAddress[2] =  OUT_X_L_M;

	initLSM9DS1();
	calibrate(TRUE);

	//readAccel1();
	//makeTimer(100, SYSTM001_PERIODIC, timerHandlerReadByte1, &a, &Status, &TimerId);
	TimerId=SYSTM001_CreateTimer(2,SYSTM001_PERIODIC,timerHandlerReadByte1,&adrAndData);
	SYSTM001_StartTimer(TimerId);
	while(1)
	{
		if(!readingAllowed)
		{

			int16_t accelX = (adrAndData.dane[1] << 8) | adrAndData.dane[0]; // Store x-axis values into gx

			int16_t accelY = (adrAndData.dane[3] << 8) | adrAndData.dane[2]; // Store y-axis values into gy

			int16_t accelZ = (adrAndData.dane[5] << 8) | adrAndData.dane[4]; // Store z-axis values into gz

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

			int16_t gyroX = (adrAndData.dane[7] << 1) | adrAndData.dane[6];
			int16_t gyroY = (adrAndData.dane[9] << 1) | adrAndData.dane[8];
			int16_t gyroZ = (adrAndData.dane[11] << 1) | adrAndData.dane[10];

			if (_autoCalc) //kalibracja
			{
				gyroX -= gBiasRaw[X_AXIS];
				gyroY -= gBiasRaw[Y_AXIS];
				gyroZ -= gBiasRaw[Z_AXIS];
			}
			gyroX = calcGyro(gyroX);
			gyroY = calcGyro(gyroY);
			gyroZ = calcGyro(gyroZ);

			pomiary1[counter].ax = gyroX;
			pomiary1[counter].ay = gyroY;
			pomiary1[counter].az = gyroZ;
			counter++;
			readingAllowed = TRUE;
		}

		if(counter >= 100)
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
	static uint8_t whichDevice = 0;
	addressAndData *address = (addressAndData*)T;

	if(readingAllowed == TRUE)
	{
		if(0 == whichDevice) //accel
		{
			receiveByte(address->adr.addressDevice[0], (address->adr.subAddress[0] + whichByte), &(address->dane[whichByte]));
			whichByte++;

			if(whichByte == 6)
			{
				//readingAllowed = FALSE;

				whichDevice++;

				whichByte = 0;
				stageOfReading = 0;
			}
		}
		else if(1 == whichDevice) //gyro
		{
			receiveByte(address->adr.addressDevice[0], (address->adr.subAddress[1] + whichByte), &(address->dane[whichByte + 6]));
			whichByte++;

			if(whichByte == 6)
			{
				//readingAllowed = FALSE;

				whichDevice++;

				whichByte = 0;
				stageOfReading++;
			}
		}
		else if(2 == whichDevice)
		{
			receiveByte(address->adr.addressDevice[1], (address->adr.subAddress[2] + whichByte), &(address->dane[whichByte + 12]));
			whichByte++;

			if(whichByte == 6)
			{
				readingAllowed = FALSE;

				whichDevice = 0;

				whichByte = 0;
				stageOfReading++;
			}
		}

	}
}


void receiveByte(uint8_t adr, uint8_t subAdr, uint8_t *buffer)
{
	clearErrorFlags();

	I2C001_DataType data1;
	data1.Data1.TDF_Type = I2C_TDF_MStart;

	data1.Data1.Data = ((adr << 1) | I2C_WRITE);
	while(!I2C001_WriteData(&I2C001_Handle0,&data1))
	{
		flushFIFO();
	}

	delay(DELAY);

	I2C001_DataType data2;
	data2.Data1.TDF_Type = I2C_TDF_MTxData;

	data2.Data1.Data = subAdr;
	while(!I2C001_WriteData(&I2C001_Handle0,&data2))
	{
		flushFIFO();
	}

	delay(DELAY);

	I2C001_DataType data3;
	data3.Data1.TDF_Type = I2C_TDF_MRStart;
	//uint8_t adr1 = address->adr.addressDevice;
	data3.Data1.Data = ((adr << 1) | I2C_READ);
	while(!I2C001_WriteData(&I2C001_Handle0,&data3))
	{
		flushFIFO();
	}

	delay(DELAY);

	I2C001_DataType data4;
	data4.Data1.TDF_Type = I2C_TDF_MRxAck1;
	data4.Data1.Data = ubyteFF;
	while(!I2C001_WriteData(&I2C001_Handle0,&data4))
	{
		flushFIFO();
	}

	delay(DELAY);

	I2C001_DataType data5;
	data5.Data1.TDF_Type = I2C_TDF_MStop;
	data5.Data1.Data = ubyteFF;
	while(!I2C001_WriteData(&I2C001_Handle0,&data5))
	{
		flushFIFO();
	}

	delay(DELAY);

	int k = 0;
	uint16_t bufferToRead = 0;
	if(I2C001_ReadData(&I2C001_Handle0,&bufferToRead))
	{
		k++;
	}
	else
	{
		k--;
	}

	delay(DELAY);
	*buffer = (uint8_t)bufferToRead;

}
