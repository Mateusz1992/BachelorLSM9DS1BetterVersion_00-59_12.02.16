/*
 * LSM9DS1_Driver.c
 *
 *  Created on: 08-02-2016
 *      Author: Mateusz
 */
#include "LSM9DS1_Driver.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

float magSensitivity[4] = {0.00014, 0.00029, 0.00043, 0.00058};

void init(interface_mode interface, uint8_t xgAddr, uint8_t mAddr)
{
	//measurementsLSMRead = 0;

	settings.device.commInterface = interface;
	settings.device.agAddress = xgAddr;
	settings.device.mAddress = mAddr;

	settings.gyro.enabled = TRUE;
	settings.gyro.enableX = TRUE;
	settings.gyro.enableY = TRUE;
	settings.gyro.enableZ = TRUE;
	// gyro scale can be 245, 500, or 2000
	settings.gyro.scale = 245;
	// gyro sample rate: value between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	settings.gyro.sampleRate = 3;
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	settings.gyro.bandwidth = 0;
	settings.gyro.lowPowerEnable = FALSE;

	settings.gyro.HPFEnable = FALSE;
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is TRUE.
	settings.gyro.HPFCutoff = 0;
	settings.gyro.flipX = FALSE;
	settings.gyro.flipY = FALSE;
	settings.gyro.flipZ = FALSE;
	settings.gyro.orientation = 0;
	settings.gyro.latchInterrupt = TRUE;

	settings.accel.enabled = TRUE;
	settings.accel.enableX = TRUE;
	settings.accel.enableY = TRUE;
	settings.accel.enableZ = TRUE;
	// accel scale can be 2, 4, 8, or 16
	settings.accel.scale = 2;
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	settings.accel.sampleRate = 6;
	// Accel cutoff freqeuncy can be any value between -1 - 3.
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	settings.accel.bandwidth = -1;
	settings.accel.highResEnable = FALSE;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	settings.accel.highResBandwidth = 0;

	settings.mag.enabled = TRUE;
	// mag scale can be 4, 8, 12, or 16
	settings.mag.scale = 4;
	// mag data rate can be 0-7
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	settings.mag.sampleRate = 7;
	settings.mag.tempCompensationEnable = FALSE;
	// magPerformance can be any value between 0-3
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	settings.mag.XYPerformance = 3;
	settings.mag.ZPerformance = 3;
	settings.mag.lowPowerEnable = FALSE;
	// magOperatingMode can be 0-2
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	settings.mag.operatingMode = 0;

	settings.temp.enabled = TRUE;

	for (int i=0; i<3; i++)
	{
		gBias[i] = 0;
		aBias[i] = 0;
		mBias[i] = 0;
		gBiasRaw[i] = 0;
		aBiasRaw[i] = 0;
		mBiasRaw[i] = 0;
	}

	_autoCalc = FALSE;
}

uint16_t begin(void)
{
	//! Todo: don't use _xgAddress or _mAddress, duplicating memory
	_xgAddress = settings.device.agAddress;
	_mAddress = settings.device.mAddress;

	constrainScales();

	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	if (settings.device.commInterface == IMU_MODE_I2C)	// If we're using I2C
		initI2C();	// Initialize I2C
	else if (settings.device.commInterface == IMU_MODE_SPI) 	// else, if we're using SPI
		initSPI();	// Initialize SPI

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t mTest = mReadByte(WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I

	uint16_t whoAmICombined = (xgTest << 8) | mTest;

	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
	{
		return 0;
	}

	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.

	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.

	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;

}

void constrainScales()
{
	if ((settings.gyro.scale != 245) && (settings.gyro.scale != 500) && (settings.gyro.scale != 2000))
	{
		settings.gyro.scale = 245;
	}

	if ((settings.accel.scale != 2) && (settings.accel.scale != 4) && (settings.accel.scale != 8) && (settings.accel.scale != 16))
	{
		settings.accel.scale = 2;
	}

	if ((settings.mag.scale != 4) && (settings.mag.scale != 8) && (settings.mag.scale != 12) && (settings.mag.scale != 16))
	{
		settings.mag.scale = 4;
	}
}


void initI2C(void)
{
	;
}

void initSPI(void)
{
	;
}

void calcgRes()
{
	gRes = ((float) settings.gyro.scale) / 32768.0;
}

void calcaRes()
{
	aRes = ((float) settings.accel.scale) / 32768.0;
}


void calcmRes()
{
	//mRes = ((float) settings.mag.scale) / 32768.0;
	switch (settings.mag.scale)
	{
		case 4:
			mRes = magSensitivity[0];
			break;
		case 8:
			mRes = magSensitivity[1];
			break;
		case 12:
			mRes = magSensitivity[2];
			break;
		case 16:
			mRes = magSensitivity[3];
			break;
	}

}

void delay(int d)
{
	for(int i = 0; i < d; i++) i++;
}

uint8_t I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint32_t stageOfReading = 0;

	//deviceAddress address = *((deviceAddress*)T);

		I2C001_DataType data1;
		data1.Data1.TDF_Type = I2C_TDF_MStart;
		data1.Data1.Data = ((address<<1) | I2C_WRITE);
		while(!I2C001_WriteData(&I2C001_Handle0,&data1));

		delay(10000);

		I2C001_DataType data2;
		data2.Data1.TDF_Type = I2C_TDF_MTxData;
		data2.Data1.Data = subAddress;
		while(!I2C001_WriteData(&I2C001_Handle0,&data2));

		delay(10000);


		I2C001_DataType data3;
		data3.Data1.TDF_Type = I2C_TDF_MRStart;
		data3.Data1.Data = ((address<<1) | I2C_READ);
		while(!I2C001_WriteData(&I2C001_Handle0,&data3));

		delay(10000);


		I2C001_DataType data4;
		data4.Data1.TDF_Type = I2C_TDF_MRxAck1;
		data4.Data1.Data = ubyteFF;
		while(!I2C001_WriteData(&I2C001_Handle0,&data4));

		delay(10000);

		I2C001_DataType data5;
		data5.Data1.TDF_Type = I2C_TDF_MStop;
		data5.Data1.Data = ubyteFF;
		while(!I2C001_WriteData(&I2C001_Handle0,&data5));
		stageOfReading++;

		delay(10000);

		uint16_t DataReceive1 = 0;
		if(I2C001_ReadData(&I2C001_Handle0,&DataReceive1))
		{
			stageOfReading++;
		}
		else
		{
			stageOfReading--;
		}

		return (uint8_t)DataReceive1;
}


uint8_t mReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadByte(_mAddress, subAddress);
	/*else if (settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadByte(_mAddress, subAddress);*/
}

uint8_t xgReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadByte(_xgAddress, subAddress);
	/*else if (settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadByte(_xgAddress, subAddress);*/
}

void initGyro(void)
{
	uint8_t tempRegValue = 0;

	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection

	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (settings.gyro.enabled)
	{
		tempRegValue = (settings.gyro.sampleRate & 0x07) << 5;
	}

	switch (settings.gyro.scale)
	{
		case 500:
			tempRegValue |= (0x1 << 3);
			break;
		case 2000:
			tempRegValue |= (0x3 << 3);
			break;
		// Otherwise we'll set it to 245 dps (0x0 << 4)
	}
	tempRegValue |= (settings.gyro.bandwidth & 0x3);
	xgWriteByte(CTRL_REG1_G, tempRegValue);

	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	xgWriteByte(CTRL_REG2_G, 0x00);

	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (settings.gyro.HPFEnable)
	{
		tempRegValue |= ((1<<6) | (settings.gyro.HPFCutoff & 0x0F));
	}
	xgWriteByte(CTRL_REG3_G, tempRegValue);

	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (settings.gyro.enableY) tempRegValue |= (1<<4);
	if (settings.gyro.enableX) tempRegValue |= (1<<3);
	if (settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
	xgWriteByte(CTRL_REG4, tempRegValue);

	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (settings.gyro.flipX) tempRegValue |= (1<<5);
	if (settings.gyro.flipY) tempRegValue |= (1<<4);
	if (settings.gyro.flipZ) tempRegValue |= (1<<3);
	xgWriteByte(ORIENT_CFG_G, tempRegValue);
}


void xgWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
	{
		I2CwriteByte(_xgAddress, subAddress, data);
	}
}

void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	/*addressTimerWrite = address;
	subAddressTimerWrite = subAddress;
	dataWrite = data;*/

	//makeTimer(50, SYSTM001_PERIODIC, timerHandlerI2CwriteByte, NULL, &WriteTimerStatus, &WriteTimerId);


		I2C001_DataType data1;
		data1.Data1.TDF_Type = I2C_TDF_MStart;
		data1.Data1.Data = ((address<<1) | I2C_WRITE);
		while(!I2C001_WriteData(&I2C001_Handle0,&data1));

		delay(10000);

		I2C001_DataType data2;
		data2.Data1.TDF_Type = I2C_TDF_MTxData;
		data2.Data1.Data = subAddress;
		while(!I2C001_WriteData(&I2C001_Handle0,&data2));

		delay(10000);

		I2C001_DataType data3;
		data3.Data1.TDF_Type = I2C_TDF_MTxData;
		data3.Data1.Data = data;
		while(!I2C001_WriteData(&I2C001_Handle0,&data3));

		delay(10000);

		I2C001_DataType data4;
		data4.Data1.TDF_Type = I2C_TDF_MStop;
		data4.Data1.Data = ubyteFF;
		while(!I2C001_WriteData(&I2C001_Handle0,&data4));

		delay(10000);
}

void initAccel(void)
{
	uint8_t tempRegValue = 0;

	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (settings.accel.enableZ) tempRegValue |= (1<<5);
	if (settings.accel.enableY) tempRegValue |= (1<<4);
	if (settings.accel.enableX) tempRegValue |= (1<<3);

	xgWriteByte(CTRL_REG5_XL, tempRegValue);

	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (settings.accel.enabled)
	{
		tempRegValue |= ((settings.accel.sampleRate & 0x07) << 5);
	}
	switch (settings.accel.scale)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			break;
		// Otherwise it'll be set to 2g (0x0 << 3)
	}
	if (settings.accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (settings.accel.bandwidth & 0x03);
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);

	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
	}
	xgWriteByte(CTRL_REG7_XL, tempRegValue);
}

void initMag(void)
{
	uint8_t tempRegValue = 0;

	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (settings.mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (settings.mag.sampleRate & 0x7) << 2;
	mWriteByte(CTRL_REG1_M, tempRegValue);

	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = 0;
	switch (settings.mag.scale)
	{
	case 8:
		tempRegValue |= (0x1 << 5);
		break;
	case 12:
		tempRegValue |= (0x2 << 5);
		break;
	case 16:
		tempRegValue |= (0x3 << 5);
		break;
	// Otherwise we'll default to 4 gauss (00)
	}
	mWriteByte(CTRL_REG2_M, tempRegValue); // +/-4Gauss

	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= (settings.mag.operatingMode & 0x3);
	mWriteByte(CTRL_REG3_M, tempRegValue); // Continuous conversion mode

	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (settings.mag.ZPerformance & 0x3) << 2;
	mWriteByte(CTRL_REG4_M, tempRegValue);

	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	mWriteByte(CTRL_REG5_M, tempRegValue);
}

void mWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
	{
		return I2CwriteByte(_mAddress, subAddress, data);
	}
}

void enableFIFO(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if(enable)
	{
		temp |= (1<<1);
	}
	else
	{
		temp &= ~(1<<1);
	}

	xgWriteByte(CTRL_REG9, temp);
}

void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

void calibrate(bool autoCalc)
{
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	uint8_t samples = 0;
	int ii;
	int32_t aBiasRawTemp[3] = {0, 0, 0};
	int32_t gBiasRawTemp[3] = {0, 0, 0};

	// Turn on FIFO and set threshold to 32 samples
	enableFIFO(TRUE);
	setFIFO(FIFO_THS, 0x1F);
	/*while (samples < 29)
	{*/
		samples = (xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
		//samples = 10;
	//}
	for(ii = 0; ii < samples ; ii++)
	{	// Read the gyro data stored in the FIFO
		readGyro1();
		gBiasRawTemp[0] += gx;
		gBiasRawTemp[1] += gy;
		gBiasRawTemp[2] += gz;

		readAccel1();
		aBiasRawTemp[0] += ax;
		aBiasRawTemp[1] += ay;
		aBiasRawTemp[2] += az - (int16_t)(1./aRes); // Assumes sensor facing up!
	}
	for (ii = 0; ii < samples/*3*/; ii++)
	{
		gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
		gBias[ii] = calcGyro(gBiasRaw[ii]);
		aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		aBias[ii] = calcAccel(aBiasRaw[ii]);
	}

	enableFIFO(FALSE);
	setFIFO(FIFO_OFF, 0x00);

	if (autoCalc) _autoCalc = TRUE;
}


void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		I2CreadBytes(_xgAddress, subAddress, dest, count);
		//I2CreadBytes1(_xgAddress, subAddress, dest, count);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		SPIreadBytes(_xgAddress, subAddress, dest, count);
}

void SPIreadBytes(uint8_t csPin, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	;
}

void readGyro1(void)
{
	uint8_t i = 0; //licznik dla czytania
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	uint8_t subAddr = OUT_X_L_G;


	while(i < 6)
	{
		subAddr = OUT_X_L_G;
		subAddr = subAddr + i;
		temp[i] = I2CreadBytes(_xgAddress, subAddr, NULL, 0);
		delay(10000);
		i++;
	}

	gx = ((int8_t)temp[1] << 8) | (int8_t)temp[0]; // Store x-axis values into gx

	gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy

	gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz



	if (_autoCalc) //kalibracja
	{
		gx -= gBiasRaw[X_AXIS];
		gy -= gBiasRaw[Y_AXIS];
		gz -= gBiasRaw[Z_AXIS];
	}
	/*gx = calcGyro(gx);
	gy = calcGyro(gy);
	gz = calcGyro(gz);*/
}

void readAccel1(void)
{
	uint8_t i = 0; //licznik dla czytania
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	uint8_t subAddr = OUT_X_L_XL;

	while(i < 6)
	{
		subAddr = OUT_X_L_XL;
		subAddr = subAddr + i;
		temp[i] = I2CreadBytes(_xgAddress, subAddr, NULL, 0);
		i++;
	}

	ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az = (temp[5] << 8) | temp[4]; // Store z-axis values into az

	if (_autoCalc) //kalibracja
	{
		ax -= aBiasRaw[X_AXIS];
		ay -= aBiasRaw[Y_AXIS];
		az -= aBiasRaw[Z_AXIS];
	}

	/*ax = calcAccel(ax);
	ay = calcAccel(ay);
	az = calcAccel(az);*/
}

void readAccelToSensor(accel *pomiar)
{
	uint8_t i = 0; //licznik dla czytania
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	uint8_t subAddr = OUT_X_L_XL;

	while(i < 6)
	{
		subAddr = OUT_X_L_XL;
		subAddr = subAddr + i;
		temp[i] = I2CreadBytes(_xgAddress, subAddr, NULL, 0);
		i++;
	}

	ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az = (temp[5] << 8) | temp[4]; // Store z-axis values into az

	if (_autoCalc) //kalibracja
	{
		ax -= aBiasRaw[X_AXIS];
		ay -= aBiasRaw[Y_AXIS];
		az -= aBiasRaw[Z_AXIS];
	}

	ax = calcAccel(ax);
	ay = calcAccel(ay);
	az = calcAccel(az);

	pomiar->ax = ax;
	pomiar->ay = ay;
	pomiar->az = az;
}

uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
		USIC_CH_TypeDef* I2CRegs = I2C001_Handle0.I2CRegs;

		I2C001_DataType data1;
		data1.Data1.TDF_Type = I2C_TDF_MStart;
		data1.Data1.Data = ((address<<1) | I2C_WRITE);
		while(!I2C001_WriteData(&I2C001_Handle0,&data1))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}
		delay(10000);

		I2C001_DataType data2;
		data2.Data1.TDF_Type = I2C_TDF_MTxData;
		data2.Data1.Data = (subAddress);
		while(!I2C001_WriteData(&I2C001_Handle0,&data2))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}
		delay(10000);

		I2C001_DataType data3;
		data3.Data1.TDF_Type = I2C_TDF_MRStart;
		data3.Data1.Data = ((address<<1) | I2C_READ);
		while(!I2C001_WriteData(&I2C001_Handle0,&data3))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}
		delay(10000);

		I2C001_DataType data4;
		data4.Data1.TDF_Type = I2C_TDF_MRxAck1;
		data4.Data1.Data = ubyteFF;
		while(!I2C001_WriteData(&I2C001_Handle0,&data4))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}
		delay(10000);

		I2C001_DataType data5;
		data5.Data1.TDF_Type = I2C_TDF_MStop;
		data5.Data1.Data = ubyteFF;
		while(!I2C001_WriteData(&I2C001_Handle0,&data5))
		{
			USIC_FlushTxFIFO(I2CRegs);
		}
		delay(10000);

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
		delay(10000);
		return (uint8_t)buffer;
}

uint8_t magAvailable(lsm9ds1_axis axis)
{
	uint8_t status;
	status = mReadByte(STATUS_REG_M);

	return ((status & (1<<axis)) >> axis);
}

void readMag1(void)
{
	//for(int kl = 0; kl < 10; kl++){
	uint8_t temp[6]; // We'll read six bytes from the mag into temp
	uint8_t subAddress = OUT_X_L_M;
	uint8_t i = 0;

	while(i < 6)
	{
		temp[i] = I2CreadBytes(_mAddress, subAddress, NULL, 0);
		i++;
	}

	mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz

	/*mx = calcMag(mx);
	my = calcMag(my);
	mz = calcMag(mz);*?
	/*}*/


}


void calibrateMag(bool loadIn)
{
	int i, j;
	int16_t magMin[3] = {0, 0, 0};
	int16_t magMax[3] = {0, 0, 0}; // The road warrior

	for (i=0; i<128; i++)
	{
		//tu nie wiem
		while (!magAvailable(i))
			;
		readMag1();
		int16_t magTemp[3] = {0, 0, 0};
		magTemp[0] = mx;
		magTemp[1] = my;
		magTemp[2] = mz;
		for (j = 0; j < 3; j++)
		{
			if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}
	}
	for (j = 0; j < 3; j++)
	{
		mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
		mBias[j] = calcMag(mBiasRaw[j]);
		if (loadIn)
			magOffset(j, mBiasRaw[j]);
	}
}

float calcMag(int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return ceil(mRes * mag);
	//return mag;
}

void magOffset(uint8_t axis, int16_t offset)
{
	if (axis > 2)
		return;
	uint8_t msb, lsb;
	msb = (offset & 0xFF00) >> 8;
	lsb = offset & 0x00FF;
	mWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
	mWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
}

uint8_t accelAvailable(void)
{
	uint8_t status = xgReadByte(STATUS_REG_1);

	return (status & (1<<0));
}

uint8_t gyroAvailable(void)
{
	uint8_t status = xgReadByte(STATUS_REG_1);

	return ((status & (1<<1)) >> 1);
}

uint8_t tempAvailable(void)
{
	uint8_t status = xgReadByte(STATUS_REG_1);

	return ((status & (1<<2)) >> 2);
}

int16_t readAccel(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;
	uint8_t subAddress = OUT_X_L_XL + (2 * axis);
	int i = 0;
	//xgReadBytes(OUT_X_L_XL + (2 * axis), temp, 2);
	while(i < 2)
	{
		subAddress = subAddress + i;

		temp[i] = I2CreadBytes(_xgAddress, subAddress, NULL, 0);

		i++;
	}

	value = (temp[1] << 8) | temp[0];

	if (_autoCalc)
		value -= aBiasRaw[axis];

	return value;
}

int16_t readMag(lsm9ds1_axis axis)
{
	uint8_t temp[2];

	int i = 0;
	uint8_t subAddress = OUT_X_L_M + (2 * axis);

	while(i < 2)
	{
		subAddress = subAddress + i;

		temp[i] = I2CreadBytes(_mAddress, subAddress, NULL, 0);

		i++;
	}

	int16_t value = (temp[1] << 8) | temp[0];
	return value;
}

int16_t readTemp(void)
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp

	int i  = 0;
	uint8_t subAddress = OUT_TEMP_L;

	while(i < 2)
	{
		subAddress = subAddress + i;

		temp[i] = I2CreadBytes(_xgAddress, subAddress, NULL, 0);

		i++;
	}

	//xgReadBytes(OUT_TEMP_L, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L
	temperature = (temp[1] << 8) | temp[0];

	return temperature;
}

int16_t readGyro(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;

	int i  = 0;
	uint8_t subAddress = OUT_X_L_G + (2 * axis);

	while(i < 2)
	{
		subAddress = subAddress + i;

		temp[i] = I2CreadBytes(_xgAddress, subAddress, NULL, 0);
		i++;
	}

	value = (temp[1] << 8) | temp[0];

	if (_autoCalc)
		value -= gBiasRaw[axis];

	return value;
}

float calcGyro(int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return round(gRes * gyro);
	//return gyro;
}

float calcAccel(int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return round(aRes * accel);
	//return accel;
}


void setGyroScale(uint16_t gScl)
{
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = xgReadByte(CTRL_REG1_G);
	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0xE7;
	switch (gScl)
	{
		case 500:
			ctrl1RegValue |= (0x1 << 3);
			settings.gyro.scale = 500;
			break;
		case 2000:
			ctrl1RegValue |= (0x3 << 3);
			settings.gyro.scale = 2000;
			break;
		default: // Otherwise we'll set it to 245 dps (0x0 << 4)
			settings.gyro.scale = 245;
			break;
	}
	xgWriteByte(CTRL_REG1_G, ctrl1RegValue);

	calcgRes();
}

void setAccelScale(uint8_t aScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = xgReadByte(CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;

	switch (aScl)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			settings.accel.scale = 4;
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			settings.accel.scale = 8;
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			settings.accel.scale = 16;
			break;
		default: // Otherwise it'll be set to 2g (0x0 << 3)
			settings.accel.scale = 2;
			break;
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);

	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void setMagScale(uint8_t mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);

	switch (mScl)
	{
		case 8:
			temp |= (0x1 << 5);
			settings.mag.scale = 8;
			break;
		case 12:
			temp |= (0x2 << 5);
			settings.mag.scale = 12;
			break;
		case 16:
			temp |= (0x3 << 5);
			settings.mag.scale = 16;
			break;
		default: // Otherwise we'll default to 4 gauss (00)
			settings.mag.scale = 4;
			break;
	}

	// And write the new register value back into CTRL_REG6_XM:
	mWriteByte(CTRL_REG2_M, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	//mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void setGyroODR(uint8_t gRate)
{
	// Only do this if gRate is not 0 (which would disable the gyro)
	if ((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG1_G);
		// Then mask out the gyro ODR bits:
		temp &= 0xFF^(0x7 << 5);
		temp |= (gRate & 0x07) << 5;
		// Update our settings struct
		settings.gyro.sampleRate = gRate & 0x07;
		// And write the new register value back into CTRL_REG1_G:
		xgWriteByte(CTRL_REG1_G, temp);
	}
}

void setAccelODR(uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		settings.accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		xgWriteByte(CTRL_REG6_XL, temp);
	}
}

void setMagODR(uint8_t mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	settings.mag.sampleRate = mRate & 0x07;
	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
}

void configInt(interrupt_select interupt, uint8_t generator, h_lactive activeLow, pp_od pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
	// those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	xgWriteByte(interupt, generator);

	// Configure CTRL_REG8
	uint8_t temp;
	temp = xgReadByte(CTRL_REG8);

	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);

	if (pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);

	xgWriteByte(CTRL_REG8, temp);
}


void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn)
{
	uint8_t temp = 0;

	temp = threshold & 0x7F;
	if (sleepOn) temp |= (1<<7);
	xgWriteByte(ACT_THS, temp);

	xgWriteByte(ACT_DUR, duration);
}


uint8_t getInactivity(void)
{
	uint8_t temp = xgReadByte(STATUS_REG_0);
	temp &= (0x10);
	return temp;
}

void configAccelInt(uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	xgWriteByte(INT_GEN_CFG_XL, temp);
}

void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_X_XL + axis, threshold);

	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_XL, temp);
}

uint8_t getAccelIntSrc(void)
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_XL);

	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}

	return 0;
}

void configGyroInt(uint8_t generator, bool aoi, bool latch)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	xgWriteByte(INT_GEN_CFG_G, temp);
}


void configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	uint8_t buffer[2];
	buffer[0] = (threshold & 0x7F00) >> 8;
	buffer[1] = (threshold & 0x00FF);
	// Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
	xgWriteByte(INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);

	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_G, temp);
}


uint8_t getGyroIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_G);

	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}

	return 0;
}


void configMagInt(uint8_t generator, h_lactive activeLow, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);
	// IEA bit is 0 for active-low, 1 for active-high.
	if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) config |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (generator != 0) config |= (1<<0);

	mWriteByte(INT_CFG_M, config);
}


void configMagThs(uint16_t threshold)
{
	// Write high eight bits of [threshold] to INT_THS_H_M
	mWriteByte(INT_THS_H_M, (uint8_t)((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	mWriteByte(INT_THS_L_M, (uint8_t)(threshold & 0x00FF));
}

uint8_t getMagIntSrc(void)
{
	uint8_t intSrc = mReadByte(INT_SRC_M);

	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc & 0xFE);
	}

	return 0;
}

void sleepGyro(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	xgWriteByte(CTRL_REG9, temp);
}


uint8_t getFIFOSamples(void)
{
	return (xgReadByte(FIFO_SRC) & 0x3F);
}
