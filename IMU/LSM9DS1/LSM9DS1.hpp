#pragma once
#ifndef LSM9DS1_HPP_
#define LSM9DS1_HPP_

/* C/C++ Includes */
#include <stdint.h>

/* Library Includes */
#include "LSM9DS1_Types.h"
#include "LSM9DS1_Registers.h"

/* Thor Includes */
#include "Thor/include/thor_config.h"
#include "Thor/include/thor_definitions.h"
#include "Thor/include/gpio.h"
#include "Thor/include/spi.h"

#ifdef USING_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif


#define LSM9DS1_AG_ADDR(sa0)	((sa0) == 0 ? 0x6A : 0x6B)
#define LSM9DS1_M_ADDR(sa1)		((sa1) == 0 ? 0x1C : 0x1E)

/* Highest sampling rates supported (Hz)*/
#define LSM9DS1_M_MAX_BW 75
#define LSM9DS1_X_MAX_BW 952
#define LSM9DS1_G_MAX_BW 952

enum lsm9ds1_axis {
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
};

class LSM9DS1
{
public:
	IMUSettings settings;
	
	// We'll store the gyro, accel, and magnetometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readGyro(), readAccel(), and readMag() first, before using
	// these variables!
	// These values are the RAW signed 16-bit readings from the sensors.
	int16_t gx, gy, gz;  // x, y, and z axis readings of the gyroscope
	int16_t ax, ay, az;  // x, y, and z axis readings of the accelerometer
	int16_t mx, my, mz;  // x, y, and z axis readings of the magnetometer
    int16_t temperature;  // Chip temperature
	float gBias[3], aBias[3], mBias[3];
	int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];
	
	float aRaw[3];		/* Raw converted output of accelerometer in m/s^2 */
	float aNorm[3];		/* Normalized accelerometer data in m/s^2, where 9.8 m/s^2 == 1.0 g */
	
	float gRaw[3];		/* Raw converted output of gyroscope in dps */
	float mRaw[3];		/* Raw converted output of magnetometer in gauss */
	
	LSM9DS1(SPIClass_sPtr spi_instance, GPIOClass_sPtr xg_ss_pin, GPIOClass_sPtr m_ss_pin);
		
	// begin() -- Initialize the gyro, accelerometer, and magnetometer.
	// This will set up the scale and output rate of each sensor. The values set
	// in the IMUSettings struct will take effect after calling this function.
	uint16_t begin();
	
	void selfTest();
	
	void calibrate(bool autoCalc = true);
	void calibrateMag(bool loadIn = true);
	void magOffset(uint8_t axis, int16_t offset);
	
	// accelAvailable() -- Polls the accelerometer status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	uint8_t accelAvailable();
	
	// gyroAvailable() -- Polls the gyroscope status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	uint8_t gyroAvailable();
	
	// tempAvailable() -- Polls the temperature status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	uint8_t tempAvailable();
	
	// magAvailable() -- Polls the accelerometer status register to check
	// if new data is available.
	// Input:
	//	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
	//	  on one specific axis. Or ALL_AXIS (default) to check for new data
	//	  on all axes.
	// Output:	1 - New data available
	//			0 - No new data available
	uint8_t magAvailable(lsm9ds1_axis axis = ALL_AXIS);
	
	// readGyro() -- Read the gyroscope output registers.
	// This function will read all six gyroscope output registers.
	// The readings are stored in the class' gx, gy, and gz variables. Read
	// those _after_ calling readGyro().
	void readGyro();
	
	
	// readAccel() -- Read the accelerometer output registers.
	// This function will read all six accelerometer output registers.
	// The readings are stored in the class' ax, ay, and az variables. Read
	// those _after_ calling readAccel().
	void readAccel();
	
	
	// readMag() -- Read the magnetometer output registers.
	// This function will read all six magnetometer output registers.
	// The readings are stored in the class' mx, my, and mz variables. Read
	// those _after_ calling readMag().
	void readMag();
	

	// readTemp() -- Read the temperature output register.
	// This function will read two temperature output registers.
	// The combined readings are stored in the class' temperature variables. Read
	// those _after_ calling readTemp().
	void readTemp();
	

	void calcGyro();
		
	void calcAccel();
	
	void calcMag();
	
	void read9Dof();	/* Reads the Accel/Gyro/Mag sensors */
	void calc9Dof();	/* Calculates useable data for Accel/Gyro/Mag sensors */
	
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
	
	// configInactivity() -- Configure inactivity interrupt parameters
	// Input:
	//	- duration = Inactivity duration - actual value depends on gyro ODR
	//	- threshold = Activity Threshold
	//	- sleepOn = Gyroscope operating mode during inactivity.
	//	  true: gyroscope in sleep mode
	//	  false: gyroscope in power-down
	void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn);
	
	// configAccelInt() -- Configure Accelerometer Interrupt Generator
	// Input:
	//	- generator = Interrupt axis/high-low events
	//	  Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
	//	- andInterrupts = AND/OR combination of interrupt events
	//	  true: AND combination
	//	  false: OR combination
	void configAccelInt(uint8_t generator, bool andInterrupts = false);
	
	// configAccelThs() -- Configure the threshold of an accelereomter axis
	// Input:
	//	- threshold = Interrupt threshold. Possible values: 0-255.
	//	  Multiply by 128 to get the actual raw accel value.
	//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
	//	- duration = Duration value must be above or below threshold to trigger interrupt
	//	- wait = Wait function on duration counter
	//	  true: Wait for duration samples before exiting interrupt
	//	  false: Wait function off
	void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration = 0, bool wait = 0);
	
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
	void configInt(interrupt_select interupt,
		uint8_t generator,
		h_lactive activeLow = INT_ACTIVE_LOW,
		pp_od pushPull = INT_PUSH_PULL);
				   
	// configMagInt() -- Configure Magnetometer Interrupt Generator
	// Input:
	//	- generator = Interrupt axis/high-low events
	//	  Any OR'd combination of ZIEN, YIEN, XIEN
	//	- activeLow = Interrupt active configuration
	//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
	//	- latch: latch gyroscope interrupt request.
	void configMagInt(uint8_t generator, h_lactive activeLow, bool latch = true);
	
	// configMagThs() -- Configure the threshold of a gyroscope axis
	// Input:
	//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
	//	  Value is equivalent to raw magnetometer value.
	void configMagThs(uint16_t threshold);
	
	// getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
	uint8_t getGyroIntSrc();
	
	// getGyroIntSrc() -- Get contents of accelerometer interrupt source register
	uint8_t getAccelIntSrc();
	
	// getGyroIntSrc() -- Get contents of magnetometer interrupt source register
	uint8_t getMagIntSrc();
	
	// getGyroIntSrc() -- Get status of inactivity interrupt
	uint8_t getInactivity();
	
	// sleepGyro() -- Sleep or wake the gyroscope
	// Input:
	//	- enable: True = sleep gyro. False = wake gyro.
	void sleepGyro(bool enable = true);
	
	// enableFIFO() - Enable or disable the FIFO
	// Input:
	//	- enable: true = enable, false = disable.
	void enableFIFO(bool enable = true);
	
	// setFIFO() - Configure FIFO mode and Threshold
	// Input:
	//	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
	//	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
	//	- fifoThs: FIFO threshold level setting
	//	  Any value from 0-0x1F is acceptable.
	void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);
	
	// getFIFOSamples() - Get number of FIFO samples
	uint8_t getFIFOSamples();
		

protected:	
	
	GPIOClass_sPtr csPinXG, csPinM;
	SPIClass_sPtr spi;
	
	uint8_t _dummyBuffer[32];
	
	
	// x_mAddress and gAddress store the I2C address or SPI chip select pin
	// for each sensor.
	uint8_t _mAddress, _xgAddress;
	
	// gRes, aRes, and mRes store the current resolution for each sensor. 
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	float gRes, aRes, mRes;
	
	// _autoCalc keeps track of whether we're automatically subtracting off
	// accelerometer and gyroscope bias calculated in calibrate().
	bool _autoCalc;
	
	// init() -- Sets up gyro, accel, and mag settings to default.
	// - interface - Sets the interface mode (IMU_MODE_I2C or IMU_MODE_SPI)
	// - xgAddr - Sets either the I2C address of the accel/gyro or SPI chip 
	//   select pin connected to the CS_XG pin.
	// - mAddr - Sets either the I2C address of the magnetometer or SPI chip 
	//   select pin connected to the CS_M pin.
	void initSettings(interface_mode interface, uint8_t xgAddr, uint8_t mAddr);
	
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
	void initGyro();
	
	// initAccel() -- Sets up the accelerometer to begin reading.
	// This function steps through all accelerometer related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
	//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
	//		all axes enabled.
	//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
	//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
	void initAccel();
	
	// initMag() -- Sets up the magnetometer to begin reading.
	// This function steps through all magnetometer-related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
	//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
	//		requests don't latch. Temperature sensor disabled.
	//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
	//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
	//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
	void initMag();
	
	// gReadByte() -- Reads a byte from a specified gyroscope register.
	// Input:
	// 	- subAddress = Register to be read from.
	// Output:
	// 	- An 8-bit value read from the requested address.
	uint8_t mReadByte(uint8_t subAddress);
	
	// gReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the gyroscope.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	uint8_t mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	
	// gWriteByte() -- Write a byte to a register in the gyroscope.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void mWriteByte(uint8_t subAddress, uint8_t data);
	
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
	uint8_t xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	
	// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void xgWriteByte(uint8_t subAddress, uint8_t data);
	
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
	
	//////////////////////
	// Helper Functions //
	//////////////////////
	void constrainScales();
	
	///////////////////
	// SPI Functions //
	///////////////////
	// initSPI() -- Initialize the SPI hardware.
	// This function will setup all SPI pins and related hardware.
	void initSPI();
	
	// SPIwriteByte() -- Write a byte out of SPI to a register in the device
	// Input:
	//	- csPin = The chip select pin of the slave device.
	//	- subAddress = The register to be written to.
	//	- data = Byte to be written to the register.
	void SPIwriteByte(const GPIOClass_sPtr& csPin, uint8_t subAddress, uint8_t data);
	
	// SPIreadByte() -- Read a single byte from a register over SPI.
	// Input:
	//	- csPin = The chip select pin of the slave device.
	//	- subAddress = The register to be read from.
	// Output:
	//	- The byte read from the requested address.
	uint8_t SPIreadByte(const GPIOClass_sPtr& csPin, uint8_t subAddress);
	
	// SPIreadBytes() -- Read a series of bytes, starting at a register via SPI
	// Input:
	//	- csPin = The chip select pin of a slave device.
	//	- subAddress = The register to begin reading.
	// 	- * dest = Pointer to an array where we'll store the readings.
	//	- count = Number of registers to be read.
	// Output: No value is returned by the function, but the registers read are
	// 		all stored in the *dest array given.
	uint8_t SPIreadBytes(const GPIOClass_sPtr& csPin,
		uint8_t subAddress, 
		uint8_t * dest,
		uint8_t count);
	
	///////////////////
	// I2C Functions //
	///////////////////
	// initI2C() -- Initialize the I2C hardware.
	// This function will setup all I2C pins and related hardware.
	void initI2C();
	
	// I2CwriteByte() -- Write a byte out of I2C to a register in the device
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be written to.
	//	- data = Byte to be written to the register.
	void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
	
	// I2CreadByte() -- Read a single byte from a register over I2C.
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be read from.
	// Output:
	//	- The byte read from the requested address.
	uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);
	
	// I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to begin reading.
	// 	- * dest = Pointer to an array where we'll store the readings.
	//	- count = Number of registers to be read.
	// Output: No value is returned by the function, but the registers read are
	// 		all stored in the *dest array given.
	uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
};


#endif