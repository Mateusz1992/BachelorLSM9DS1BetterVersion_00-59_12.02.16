/*
 * LSM9DS1_drivers.h
 *
 *  Created on: 28-12-2015
 *      Author: Mateusz
 */

#ifndef LSM9DS1_DRIVERS_H_
#define LSM9DS1_DRIVERS_H_

#define LSM9DS1_AG_ADDR(sa0)	((sa0) == 0 ? 0x6A : 0x6B)
#define LSM9DS1_M_ADDR(sa1)		((sa1) == 0 ? 0x1C : 0x1E)

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include <DAVE3.h>
#include <stdio.h>

extern int accelMeasurementsNum;

extern char measurementsLSMRead;

extern int indexAccel;
extern char lk[20];

handle_t TimerId;
uint32_t Status;

handle_t WriteTimerId;
uint32_t WriteTimerStatus;

handle_t TimerIdReadBytes;
uint32_t StatusReadBytes;




struct IMUSettings settings;

float gBias[3], aBias[3], mBias[3];
int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

// _autoCalc keeps track of whether we're automatically subtracting off
// accelerometer and gyroscope bias calculated in calibrate().
bool _autoCalc;

// x_mAddress and gAddress store the I2C address or SPI chip select pin
// for each sensor.
uint8_t _mAddress, _xgAddress;

// gRes, aRes, and mRes store the current resolution for each sensor.
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15).
float gRes, aRes, mRes;

// gRes, aRes, and mRes store the current resolution for each sensor.
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15).
/*float gRes, aRes, mRes;*/


// We'll store the gyro, accel, and magnetometer readings in a series of
// public class variables. Each sensor gets three variables -- one for each
// axis. Call readGyro(), readAccel(), and readMag() first, before using
// these variables!
// These values are the RAW signed 16-bit readings from the sensors.
int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer

int16_t temperature; // Chip temperature



typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
}accel;

void readAccelToSensor(accel *pomiar);

typedef enum {
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
}lsm9ds1_axis;


void myDelay(volatile unsigned int delay);

void startMeasurements(void);

void readAndSendMeasurements(void (*sendFunction)(char *str));

void initAdrAndSubAdr(void);

void initLSM9DS1(void);
// init() -- Sets up gyro, accel, and mag settings to default.
// - interface - Sets the interface mode (IMU_MODE_I2C or IMU_MODE_SPI)
// - xgAddr - Sets either the I2C address of the accel/gyro or SPI chip
//   select pin connected to the CS_XG pin.
// - mAddr - Sets either the I2C address of the magnetometer or SPI chip
//   select pin connected to the CS_M pin.
void init(interface_mode interface, uint8_t xgAddr, uint8_t mAddr);

// begin() -- Initialize the gyro, accelerometer, and magnetometer.
// This will set up the scale and output rate of each sensor. The values set
// in the IMUSettings struct will take effect after calling this function.
uint16_t begin(void);

//////////////////////
// Helper Functions //
//////////////////////
void constrainScales();

// calcgRes() -- Calculate the resolution of the gyroscope.
// This function will set the value of the gRes variable. gScale must
// be set prior to calling this function.
void calcgRes();

// calcmRes() -- Calculate the resolution of the magnetometer.
// This function will set the value of the mRes variable. mScale must
// be set prior to calling this function.
void calcmRes();

// calcaRes() -- Calculate the resolution of the accelerometer.
// This function will set the value of the aRes variable. aScale must
// be set prior to calling this function.
void calcaRes();

void initI2C(void);

void initSPI(void);

// I2CreadByte() -- Read a single byte from a register over I2C.
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to be read from.
// Output:
//	- The byte read from the requested address.
uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);

// gReadByte() -- Reads a byte from a specified gyroscope register.
// Input:
// 	- subAddress = Register to be read from.
// Output:
// 	- An 8-bit value read from the requested address.
uint8_t mReadByte(uint8_t subAddress);

// SPIreadByte() -- Read a single byte from a register over SPI.
// Input:
//	- csPin = The chip select pin of the slave device.
//	- subAddress = The register to be read from.
// Output:
//	- The byte read from the requested address.
uint8_t SPIreadByte(uint8_t csPin, uint8_t subAddress);

// xmReadByte() -- Read a byte from a register in the accel/mag sensor
// Input:
//	- subAddress = Register to be read from.
// Output:
//	- An 8-bit value read from the requested register.
uint8_t xgReadByte(uint8_t subAddress);


// xmReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the accelerometer/magnetometer.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

// initGyro() -- Sets up the gyroscope to begin reading.
// This function steps through all five gyroscope control registers.
// Upon exit, the following parameters will be set:
//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled.
//		95 Hz ODR, 12.5 Hz cutoff frequency.
//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
//		set to 7.2 Hz (depends on ODR).
//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
//		active high). Data-ready output enabled on DRDY_G.
//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
void initGyro(void);

// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void xgWriteByte(uint8_t subAddress, uint8_t data);

// I2CwriteByte() -- Write a byte out of I2C to a register in the device
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to be written to.
//	- data = Byte to be written to the register.
void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);

// initAccel() -- Sets up the accelerometer to begin reading.
// This function steps through all accelerometer related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
//		all axes enabled.
//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
void initAccel(void);

// initMag() -- Sets up the magnetometer to begin reading.
// This function steps through all magnetometer-related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
//		requests don't latch. Temperature sensor disabled.
//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
void initMag(void);

// gWriteByte() -- Write a byte to a register in the gyroscope.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void mWriteByte(uint8_t subAddress, uint8_t data);

// enableFIFO() - Enable or disable the FIFO
// Input:
//	- enable: true = enable, false = disable.
void enableFIFO(bool enable);

// setFIFO() - Configure FIFO mode and Threshold
// Input:
//	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
//	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
//	- fifoThs: FIFO threshold level setting
//	  Any value from 0-0x1F is acceptable.
void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);

// readGyro() -- Read the gyroscope output registers.
// This function will read all six gyroscope output registers.
// The readings are stored in the class' gx, gy, and gz variables. Read
// those _after_ calling readGyro().
void readGyro1(void);

// readAccel() -- Read the accelerometer output registers.
// This function will read all six accelerometer output registers.
// The readings are stored in the class' ax, ay, and az variables. Read
// those _after_ calling readAccel().
void readAccel1(void);

void readAccel1v1(accel *a);

void toAscii(int16_t accel, int *index);

void calibrate(bool autoCalc);

// magAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Input:
//	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
//	  on one specific axis. Or ALL_AXIS (default) to check for new data
//	  on all axes.
// Output:	1 - New data available
//			0 - No new data available
uint8_t magAvailable(lsm9ds1_axis axis);

// readMag() -- Read the magnetometer output registers.
// This function will read all six magnetometer output registers.
// The readings are stored in the class' mx, my, and mz variables. Read
// those _after_ calling readMag().
void readMag1(void);

void calibrateMag(bool loadIn);

void magOffset(uint8_t axis, int16_t offset);

// calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
// This function reads in a signed 16-bit value and returns the scaled
// Gs. This function relies on mScale and mRes being correct.
// Input:
//	- mag = A signed 16-bit raw reading from the magnetometer.
float calcMag(int16_t mag);

// accelAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t accelAvailable(void);

// gyroAvailable() -- Polls the gyroscope status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t gyroAvailable(void);


// gyroAvailable() -- Polls the temperature status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t tempAvailable(void);

// int16_t readAccel(axis) -- Read a specific axis of the accelerometer.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t readAccel(lsm9ds1_axis axis);

// int16_t readMag(axis) -- Read a specific axis of the magnetometer.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t readMag(lsm9ds1_axis axis);

// readTemp() -- Read the temperature output register.
// This function will read two temperature output registers.
// The combined readings are stored in the class' temperature variables. Read
// those _after_ calling readTemp().
int16_t readTemp(void);

// int16_t readGyro(axis) -- Read a specific axis of the gyroscope.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t readGyro(lsm9ds1_axis axis);

// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
// This function reads in a signed 16-bit value and returns the scaled
// DPS. This function relies on gScale and gRes being correct.
// Input:
//	- gyro = A signed 16-bit raw reading from the gyroscope.
float calcGyro(int16_t gyro);

// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
// This function reads in a signed 16-bit value and returns the scaled
// g's. This function relies on aScale and aRes being correct.
// Input:
//	- accel = A signed 16-bit raw reading from the accelerometer.
float calcAccel(int16_t accel);

// setGyroScale() -- Set the full-scale range of the gyroscope.
// This function can be called to set the scale of the gyroscope to
// 245, 500, or 200 degrees per second.
// Input:
// 	- gScl = The desired gyroscope scale. Must be one of three possible
//		values from the gyro_scale.
void setGyroScale(uint16_t gScl);

// setAccelScale() -- Set the full-scale range of the accelerometer.
// This function can be called to set the scale of the accelerometer to
// 2, 4, 6, 8, or 16 g's.
// Input:
// 	- aScl = The desired accelerometer scale. Must be one of five possible
//		values from the accel_scale.
void setAccelScale(uint8_t aScl);


// setMagScale() -- Set the full-scale range of the magnetometer.
// This function can be called to set the scale of the magnetometer to
// 2, 4, 8, or 12 Gs.
// Input:
// 	- mScl = The desired magnetometer scale. Must be one of four possible
//		values from the mag_scale.
void setMagScale(uint8_t mScl);

// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
// Input:
//	- gRate = The desired output rate and cutoff frequency of the gyro.
void setGyroODR(uint8_t gRate);

// setAccelODR() -- Set the output data rate of the accelerometer
// Input:
//	- aRate = The desired output rate of the accel.
void setAccelODR(uint8_t aRate);

// setMagODR() -- Set the output data rate of the magnetometer
// Input:
//	- mRate = The desired output rate of the mag.
void setMagODR(uint8_t mRate);


// configInt() -- Configure INT1 or INT2 (Gyro and Accel Interrupts only)
// Input:
//	- interrupt = Select INT1 or INT2
//	  Possible values: XG_INT1 or XG_INT2
//	- generator = Or'd combination of interrupt generators.
//	  Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
//	  INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
//	- activeLow = Interrupt active configuration
//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
//	- pushPull =  Push-pull or open drain interrupt configuration
//	  Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
void configInt(interrupt_select interupt, uint8_t generator, h_lactive activeLow, pp_od pushPull);

// configInactivity() -- Configure inactivity interrupt parameters
// Input:
//	- duration = Inactivity duration - actual value depends on gyro ODR
//	- threshold = Activity Threshold
//	- sleepOn = Gyroscope operating mode during inactivity.
//	  true: gyroscope in sleep mode
//	  false: gyroscope in power-down
void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn);

// getGyroIntSrc() -- Get status of inactivity interrupt
uint8_t getInactivity(void);

// configAccelInt() -- Configure Accelerometer Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
//	- andInterrupts = AND/OR combination of interrupt events
//	  true: AND combination
//	  false: OR combination
void configAccelInt(uint8_t generator, bool andInterrupts);

// configAccelThs() -- Configure the threshold of an accelereomter axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-255.
//	  Multiply by 128 to get the actual raw accel value.
//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//	- duration = Duration value must be above or below threshold to trigger interrupt
//	- wait = Wait function on duration counter
//	  true: Wait for duration samples before exiting interrupt
//	  false: Wait function off
void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait);

// getGyroIntSrc() -- Get contents of accelerometer interrupt source register
uint8_t getAccelIntSrc(void);

// configGyroInt() -- Configure Gyroscope Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
//	- aoi = AND/OR combination of interrupt events
//	  true: AND combination
//	  false: OR combination
//	- latch: latch gyroscope interrupt request.
void configGyroInt(uint8_t generator, bool aoi, bool latch);

// configGyroThs() -- Configure the threshold of a gyroscope axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
//	  Value is equivalent to raw gyroscope value.
//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//	- duration = Duration value must be above or below threshold to trigger interrupt
//	- wait = Wait function on duration counter
//	  true: Wait for duration samples before exiting interrupt
//	  false: Wait function off
void configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait);


// getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
uint8_t getGyroIntSrc();

// configMagInt() -- Configure Magnetometer Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZIEN, YIEN, XIEN
//	- activeLow = Interrupt active configuration
//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
//	- latch: latch gyroscope interrupt request.
void configMagInt(uint8_t generator, h_lactive activeLow, bool latch);

// configMagThs() -- Configure the threshold of a gyroscope axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
//	  Value is equivalent to raw magnetometer value.
void configMagThs(uint16_t threshold);

// getGyroIntSrc() -- Get contents of magnetometer interrupt source register
uint8_t getMagIntSrc(void);

// sleepGyro() -- Sleep or wake the gyroscope
// Input:
//	- enable: True = sleep gyro. False = wake gyro.
void sleepGyro(bool enable);

// getFIFOSamples() - Get number of FIFO samples
uint8_t getFIFOSamples(void);

uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);

uint8_t I2CreadBytes1(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);

void SPIreadBytes(uint8_t csPin, uint8_t subAddress, uint8_t * dest, uint8_t count);

void mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

void timerHandlerReadByte(void *T);

void timerHandlerReceiveOneMeasurementEachSensor(void *T);

void receiveByte(uint8_t adr, uint8_t subAdr, uint8_t *buffer);

#endif /* LSM9DS1_DRIVERS_H_ */
