/* C/C++ Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/utilities.hpp>
#include <Chimera/logging.hpp>

/* Driver Includes */
#include "LSM9DS1.hpp"

using namespace Chimera::GPIO;
using namespace Chimera::SPI;
using namespace Chimera::Logging;

// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2  0.000061f
#define SENSITIVITY_ACCELEROMETER_4  0.000122f
#define SENSITIVITY_ACCELEROMETER_8  0.000244f
#define SENSITIVITY_ACCELEROMETER_16 0.000732f
#define SENSITIVITY_GYROSCOPE_245    0.00875f
#define SENSITIVITY_GYROSCOPE_500    0.0175f
#define SENSITIVITY_GYROSCOPE_2000   0.07f
#define SENSITIVITY_MAGNETOMETER_4   0.00014f
#define SENSITIVITY_MAGNETOMETER_8   0.00029f
#define SENSITIVITY_MAGNETOMETER_12  0.00043f
#define SENSITIVITY_MAGNETOMETER_16  0.00058f

#define GRAVITATIONAL_STANDARD_ACCELERATION 9.80665f


/*--------------------------------------
 * Initialization & Calibration 
 *-------------------------------------*/
LSM9DS1::LSM9DS1(uint8_t spiPeripheral, Chimera::GPIO::Port SSPort_xg, uint8_t SSPin_xg, Chimera::GPIO::Port SSPort_mag, uint8_t SSPin_mag)
{
	spi = Chimera::make_unique<Chimera::SPI::SPIClass>(spiPeripheral);
	csPinXG = Chimera::make_unique<Chimera::GPIO::GPIOClass>(SSPort_xg, SSPin_xg);
	csPinM = Chimera::make_unique<Chimera::GPIO::GPIOClass>(SSPort_mag, SSPin_mag);
	
	initSettings(IMU_MODE_SPI, LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0));
	
	memset(_dummyBuffer, 0, sizeof(_dummyBuffer) / sizeof(_dummyBuffer[0]));
}

void LSM9DS1::initSettings(interface_mode interface, uint8_t xgAddr, uint8_t mAddr)
{
	settings.device.commInterface = interface;
	settings.device.agAddress = xgAddr;
	settings.device.mAddress = mAddr;


	settings.gyro.enabled = true;
	settings.gyro.enableX = true;
	settings.gyro.enableY = true;
	settings.gyro.enableZ = true;


	/* Gyro scale can be 245, 500, or 2000*/
	settings.gyro.scale = 2000;
	
	// gyro sample rate: value between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	settings.gyro.sampleRate = 6;
	
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	settings.gyro.bandwidth = 3;
	
	settings.gyro.lowPowerEnable = false;
	
	settings.gyro.HPFEnable = true; //false
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is true.
	
	/* See Table 52 in Datasheet */
	settings.gyro.HPFCutoff = 0; //0
	
	
	settings.gyro.flipX = false;
	settings.gyro.flipY = false;
	settings.gyro.flipZ = false;
	settings.gyro.orientation = 0;
	settings.gyro.latchInterrupt = true;

	settings.accel.enabled = true;
	settings.accel.enableX = true;
	settings.accel.enableY = true;
	settings.accel.enableZ = true;
	// accel scale can be 2, 4, 8, or 16
	settings.accel.scale = 8;
	
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	settings.accel.sampleRate = 6;
	
	// Accel cutoff freqeuncy can be any value between -1 - 3. 
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	settings.accel.bandwidth = 3; //-1
	
	settings.accel.highResEnable = true;
	
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	settings.accel.highResBandwidth = 2;

	settings.mag.enabled = true;
	
	// mag scale can be 4, 8, 12, or 16
	settings.mag.scale = 8;
	
	// mag data rate can be 0-7
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	settings.mag.sampleRate = 7;
	settings.mag.tempCompensationEnable = false;
	// magPerformance can be any value between 0-3
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	settings.mag.XYPerformance = 3;
	settings.mag.ZPerformance = 3;
	settings.mag.lowPowerEnable = false;
	
	// magOperatingMode can be 0-2
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	settings.mag.operatingMode = 0;

	settings.temp.enabled = true;
	for (int i = 0; i < 3; i++)
	{
		gBias[i] = 0;
		aBias[i] = 0;
		mBias[i] = 0;
		gBiasRaw[i] = 0;
		aBiasRaw[i] = 0;
		mBiasRaw[i] = 0;
	}
	_autoCalc = true;
}

uint16_t LSM9DS1::begin()
{	
	/* Set the sensor scalings, then calculate the appropriate resolution */
	constrainScales();
	calcgRes();			// Calculate DPS / ADC tick		(rotation rate)
	calcmRes();			// Calculate  Gs / ADC tick		(magnetic field)
	calcaRes();			// Calculate   g / ADC tick		(gravity)
	
	
	if (settings.device.commInterface == IMU_MODE_I2C)
	{
		initI2C();
	}
	else if (settings.device.commInterface == IMU_MODE_SPI)
	{
		initSPI();
	}
		
	/* Verify connection by reading each WHO_AM_I register */
	uint8_t mTest = mReadByte(WHO_AM_I_M); 		// Magnetometer
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG); 	// Accel/Gyro

	volatile uint16_t whoAmICombined = (xgTest << 8) | mTest;
	volatile uint16_t whoAmIExpected = ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP);
	
	if (whoAmICombined != whoAmIExpected)
	{
		Console.log(Level::ERROR, "LSM9DS1 WHO_AM_I registers did not return valid readings\r\n");
		Console.log(Level::ERROR, "\tActual: %#04x\r\n", whoAmICombined);
		Console.log(Level::ERROR, "\tExpected: %#04x\r\n", whoAmIExpected);
		return 0;
	}
	else
	{
		Console.log(Level::INFO, "LSM9DS1 Device Found: %#04x\r\n", whoAmICombined);
	}

	/* Initialize all the internal registers according to values set in initSettings() */
	initGyro();
	initAccel();
	initMag();
	
	return whoAmICombined;
}

void LSM9DS1::initGyro()
{
	Console.log(Level::INFO, "LSM9DS1: Initializing Gyro\r\n");

	uint8_t tempRegValue = 0x00;
	
	/* CTRL_REG1_G (Default value: 0x00) */
	if(settings.gyro.enabled)
		tempRegValue = (settings.gyro.sampleRate & 0x07) << 5;
	
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
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (xgReadByte(CTRL_REG1_G) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG1_G write & read value not matched\r\n");
	#endif
	
	
	
	/* CTRL_REG2_G (Default value: 0x00) */
	tempRegValue = 0x00;
	xgWriteByte(CTRL_REG2_G, tempRegValue);	
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (xgReadByte(CTRL_REG2_G) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG2_G write & read value not matched\r\n");
	#endif
	
	/* CTRL_REG3_G (Default value: 0x00) */
	tempRegValue = 0x00;
	tempRegValue = settings.gyro.lowPowerEnable ? (1 << 7) : 0;
	if (settings.gyro.HPFEnable)
	{
		tempRegValue |= (1 << 6) | (settings.gyro.HPFCutoff & 0x0F);
	}
	xgWriteByte(CTRL_REG3_G, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (xgReadByte(CTRL_REG3_G) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG3_G write & read value not matched\r\n");
	#endif
	
	/* CTRL_REG4 (Default value: 0x38) */
	tempRegValue = 0x00;
	if (settings.gyro.enableZ) tempRegValue |= (1 << 5);
	if (settings.gyro.enableY) tempRegValue |= (1 << 4);
	if (settings.gyro.enableX) tempRegValue |= (1 << 3);
	if (settings.gyro.latchInterrupt) tempRegValue |= (1 << 1);
	xgWriteByte(CTRL_REG4, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (xgReadByte(CTRL_REG4) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG4 write & read value not matched\r\n");
	#endif
	
	/* ORIENT_CFG_G (Default value: 0x00) */
	tempRegValue = 0x00;
	if (settings.gyro.flipX) tempRegValue |= (1 << 5);
	if (settings.gyro.flipY) tempRegValue |= (1 << 4);
	if (settings.gyro.flipZ) tempRegValue |= (1 << 3);
	xgWriteByte(ORIENT_CFG_G, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (xgReadByte(ORIENT_CFG_G) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: ORIENT_CFG_G write & read value not matched\r\n");
	#endif
}

void LSM9DS1::initAccel()
{
	Console.log(Level::INFO, "LSM9DS1: Initializing Accelerometer\r\n");

	uint8_t tempRegValue = 0x00;
	
	/* CTRL_REG5_XL (0x1F) (Default value: 0x38) */
	if (settings.accel.enableZ) tempRegValue |= (1 << 5);
	if (settings.accel.enableY) tempRegValue |= (1 << 4);
	if (settings.accel.enableX) tempRegValue |= (1 << 3);
	
	xgWriteByte(CTRL_REG5_XL, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
	volatile auto tempResult = xgReadByte(CTRL_REG5_XL);
	if (tempResult != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG5_XL write & read value not matched\r\n");
	#endif
	
	/* CTRL_REG6_XL (0x20) (Default value: 0x00) */
	tempRegValue = 0x00;
	if(settings.accel.enabled) tempRegValue |= (settings.accel.sampleRate & 0x07) << 5;

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
		tempRegValue |= (1 << 2);  // Set BW_SCAL_ODR
		tempRegValue |= (settings.accel.bandwidth & 0x03);
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (xgReadByte(CTRL_REG6_XL) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG6_XL write & read value not matched.\r\n");
	#endif
	
	
	/* CTRL_REG7_XL (0x21) (Default value: 0x00) */
	tempRegValue = 0x00;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1 << 7);  // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
	}
	xgWriteByte(CTRL_REG7_XL, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (xgReadByte(CTRL_REG7_XL) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG7_XL write & read value not matched\r\n");
	#endif
}

void LSM9DS1::initMag()
{
	Console.log(Level::INFO, "LSM9DS1: Initializing Magnetometer\r\n");

	uint8_t tempRegValue = 0x00;
	
	/* CTRL_REG1_M (Default value: 0x10) */
	if(settings.mag.tempCompensationEnable) tempRegValue |= (1 << 7);
	tempRegValue |= (settings.mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (settings.mag.sampleRate & 0x7) << 2;
	mWriteByte(CTRL_REG1_M, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (mReadByte(CTRL_REG1_M) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG1_M write & read value not matched\r\n");
	#endif
	
	
	/* CTRL_REG2_M (Default value 0x00) */
	tempRegValue = 0x00;
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
	mWriteByte(CTRL_REG2_M, tempRegValue);   // +/-4Gauss
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (mReadByte(CTRL_REG2_M) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG2_M write & read value not matched\r\n");
	#endif
	
	/* CTRL_REG3_M (Default value: 0x03) */
	tempRegValue = 0x00;
	if (settings.mag.lowPowerEnable) tempRegValue |= (1 << 5);
	tempRegValue |= (settings.mag.operatingMode & 0x3);
	mWriteByte(CTRL_REG3_M, tempRegValue);   // Continuous conversion mode
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (mReadByte(CTRL_REG3_M) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG3_M write & read value not matched\r\n");
	#endif
	
	/* CTRL_REG4_M (Default value: 0x00) */
	tempRegValue = 0x00;
	tempRegValue = (settings.mag.ZPerformance & 0x3) << 2;
	mWriteByte(CTRL_REG4_M, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (mReadByte(CTRL_REG4_M) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG4_M write & read value not matched\r\n");
	#endif
	
	/* CTRL_REG5_M (Default value: 0x00) */
	tempRegValue = 0x00;
	mWriteByte(CTRL_REG5_M, tempRegValue);
	
	#ifdef DEBUG
	Chimera::delayMilliseconds(1);
		
	if (mReadByte(CTRL_REG5_M) != tempRegValue)
		Console.log(Level::WARN, "LSM9DS1: CTRL_REG5_M write & read value not matched\r\n");
	#endif
}

void LSM9DS1::selfTest()
{
	//TODO
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void LSM9DS1::calibrate(bool autoCalc)
{  
	uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t samples = 0;
	int ii;
	int32_t aBiasRawTemp[3] = { 0, 0, 0 };
	int32_t gBiasRawTemp[3] = { 0, 0, 0 };
	
	// Turn on FIFO and set threshold to 32 samples
	enableFIFO(true);
	setFIFO(FIFO_THS, 0x1F);
	while (samples < 0x1F)
	{
		samples = (xgReadByte(FIFO_SRC) & 0x3F);  // Read number of stored samples
	}
	for (ii = 0; ii < samples; ii++) 
	{
		// Read the gyro data stored in the FIFO
		readGyro();
		gBiasRawTemp[0] += gx;
		gBiasRawTemp[1] += gy;
		gBiasRawTemp[2] += gz;
		readAccel();
		aBiasRawTemp[0] += ax;
		aBiasRawTemp[1] += ay;
		aBiasRawTemp[2] += az - (int16_t)(1.0f/ aRes);  // Assumes sensor facing up!
	}  
	for (ii = 0; ii < 3; ii++)
	{
		gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
		gBias[ii] = gRes * gBiasRaw[ii];
		aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		aBias[ii] = aRes * aBiasRaw[ii];
	}
	
	enableFIFO(false);
	setFIFO(FIFO_OFF, 0x00);
	
	if (autoCalc) _autoCalc = true;
}

void LSM9DS1::calibrateMag(bool loadIn)
{
	int i, j;
	int16_t magMin[3] = { 0, 0, 0 };
	int16_t magMax[3] = { 0, 0, 0 };  // The road warrior
	
	for(i = 0 ; i < 128 ; i++)
	{
		while (!magAvailable());

		readMag();
		int16_t magTemp[3] = { 0, 0, 0 };
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
		mBias[j] = mRes * mBiasRaw[j];
		if (loadIn)
			magOffset(j, mBiasRaw[j]);
	}
	
}

void LSM9DS1::magOffset(uint8_t axis, int16_t offset)
{
	if (axis > 2)
		return;

	uint8_t msb, lsb;
	msb = (offset & 0xFF00) >> 8;
	lsb = offset & 0x00FF;

	mWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
	mWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
}


/*--------------------------------------
 * Status Functions
 *-------------------------------------*/
uint8_t LSM9DS1::accelAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return (status & (1 << 0));
}

uint8_t LSM9DS1::gyroAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return ((status & (1 << 1)) >> 1);
}

uint8_t LSM9DS1::tempAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return ((status & (1 << 2)) >> 2);
}

uint8_t LSM9DS1::magAvailable(lsm9ds1_axis axis)
{
	uint8_t status;
	status = mReadByte(STATUS_REG_M);
	
	return ((status & (1 << axis)) >> axis);
}


/*--------------------------------------
 * Take Measurements
 *-------------------------------------*/
void LSM9DS1::read9Dof()
{
	readAccel();
	readGyro();
	readMag();
}

void LSM9DS1::readAccel()
{	
	uint8_t readBuffer[6] = { 0, 0, 0, 0, 0, 0 };
	xgReadBytes(OUT_X_L_XL, readBuffer, 6);
	
	ax = ((readBuffer[1] << 8) | readBuffer[0]);
	ay = ((readBuffer[3] << 8) | readBuffer[2]);
	az = ((readBuffer[5] << 8) | readBuffer[4]);
	
	if (_autoCalc)
	{
		ax -= aBiasRaw[X_AXIS];
		ay -= aBiasRaw[Y_AXIS];
		az -= aBiasRaw[Z_AXIS];
	}
}

void LSM9DS1::readGyro()
{
	uint8_t readBuffer[6] = { 0, 0, 0, 0, 0, 0 };
	xgReadBytes(OUT_X_L_G, readBuffer, 6);
	
	gx = ((readBuffer[1] << 8) | readBuffer[0]);
	gy = ((readBuffer[3] << 8) | readBuffer[2]);
	gz = ((readBuffer[5] << 8) | readBuffer[4]);
	
	if (_autoCalc)
	{
		gx -= gBiasRaw[X_AXIS];
		gy -= gBiasRaw[Y_AXIS];
		gz -= gBiasRaw[Z_AXIS];
	}
}

void LSM9DS1::readMag()
{
	uint8_t readBuffer[6] = { 0, 0, 0, 0, 0, 0 };
	mReadBytes(OUT_X_L_M, readBuffer, 6);
	
	mx = ((readBuffer[1] << 8) | readBuffer[0]);
	my = ((readBuffer[3] << 8) | readBuffer[2]);
	mz = ((readBuffer[5] << 8) | readBuffer[4]);
	
	if (_autoCalc)
	{
		mx -= mBiasRaw[X_AXIS];
		my -= mBiasRaw[Y_AXIS];
		mz -= mBiasRaw[Z_AXIS];
	}
}

void LSM9DS1::readTemp()
{
	temperature = ((int16_t)xgReadByte(OUT_TEMP_H) << 8 | xgReadByte(OUT_TEMP_L));
}


/*--------------------------------------
 * Raw Measurement Conversion
 *-------------------------------------*/
void LSM9DS1::calc9Dof()
{
	calcAccel();
	calcGyro();
	calcMag();
}

void LSM9DS1::calcAccel()
{
	aNorm[0] = aRes * ax;
	aNorm[1] = aRes * ay;
	aNorm[2] = aRes * az;
	
	aRaw[0] = aNorm[0] * GRAVITATIONAL_STANDARD_ACCELERATION;
	aRaw[1] = aNorm[1] * GRAVITATIONAL_STANDARD_ACCELERATION;
	aRaw[2] = aNorm[2] * GRAVITATIONAL_STANDARD_ACCELERATION;
}

void LSM9DS1::calcGyro()
{
	gRaw[0] = gRes * gx;
	gRaw[1] = gRes * gy;
	gRaw[2] = gRes * gz;
}

void LSM9DS1::calcMag()
{
	mRaw[0] = mRes * mx;
	mRaw[1] = mRes * my;
	mRaw[2] = mRes * mz;
}


/*--------------------------------------
 * Measurement Scale Selection 
 *-------------------------------------*/
void LSM9DS1::setGyroScale(uint16_t gScl)
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

void LSM9DS1::setAccelScale(uint8_t aScl)
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

void LSM9DS1::setMagScale(uint8_t mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);

	// Then mask out the mag scale bits:
	temp &= 0xFF ^ (0x3 << 5);
	
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
	
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}


/*--------------------------------------
 * Output Data Rate Selection
 *-------------------------------------*/
void LSM9DS1::setGyroODR(uint8_t gRate)
{
	// Only do this if gRate is not 0 (which would disable the gyro)
	if((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG1_G);

		// Then mask out the gyro ODR bits:
		temp &= 0xFF ^ (0x7 << 5);
		temp |= (gRate & 0x07) << 5;

		// Update our settings struct
		settings.gyro.sampleRate = gRate & 0x07;

		// And write the new register value back into CTRL_REG1_G:
		xgWriteByte(CTRL_REG1_G, temp);
	}
}

void LSM9DS1::setAccelODR(uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if((aRate & 0x07) != 0)
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

void LSM9DS1::setMagODR(uint8_t mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);

	// Then mask out the mag ODR bits:
	temp &= 0xFF ^ (0x7 << 2);

	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	settings.mag.sampleRate = mRate & 0x07;

	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
}


/*--------------------------------------
 * Resolution Calculation
 *-------------------------------------*/
void LSM9DS1::calcgRes()
{
	switch (settings.gyro.scale)
	{
	case 245:
		gRes = SENSITIVITY_GYROSCOPE_245;
		break;
	case 500:
		gRes = SENSITIVITY_GYROSCOPE_500;
		break;
	case 2000:
		gRes = SENSITIVITY_GYROSCOPE_2000;
		break;
	default:
		break;
	}
}

void LSM9DS1::calcaRes()
{
	switch (settings.accel.scale)
	{
	case 2:
		aRes = SENSITIVITY_ACCELEROMETER_2;
		break;
	case 4:
		aRes = SENSITIVITY_ACCELEROMETER_4;
		break;
	case 8:
		aRes = SENSITIVITY_ACCELEROMETER_8;
		break;
	case 16:
		aRes = SENSITIVITY_ACCELEROMETER_16;
		break;
	default:
		break;
	}
}

void LSM9DS1::calcmRes()
{
	switch (settings.mag.scale)
	{
	case 4:
		mRes = SENSITIVITY_MAGNETOMETER_4;
		break;
	case 8:
		mRes = SENSITIVITY_MAGNETOMETER_8;
		break;
	case 12:
		mRes = SENSITIVITY_MAGNETOMETER_12;
		break;
	case 16:
		mRes = SENSITIVITY_MAGNETOMETER_16;
		break;
	}	
}

/*--------------------------------------
 * Interrupt Settings 
 *-------------------------------------*/
void LSM9DS1::configInt(interrupt_select interrupt, uint8_t generator, h_lactive activeLow, pp_od pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interrupt] should already be one of those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	xgWriteByte(interrupt, generator);
	
	// Configure CTRL_REG8
	uint8_t temp;
	temp = xgReadByte(CTRL_REG8);
	
	if (activeLow) 
		temp |= (1 << 5);
	else 
		temp &= ~(1 << 5);
	
	if (pushPull) 
		temp &= ~(1 << 4);
	else 
		temp |= (1 << 4);
	
	xgWriteByte(CTRL_REG8, temp);
}

void LSM9DS1::configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn)
{
	uint8_t temp = 0;
	
	temp = threshold & 0x7F;
	if (sleepOn) 
		temp |= (1 << 7);

	xgWriteByte(ACT_THS, temp);
	xgWriteByte(ACT_DUR, duration);
}

void LSM9DS1::configAccelInt(uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create the [generator] value.
	uint8_t temp = generator;

	if (andInterrupts) 
		temp |= 0x80;

	xgWriteByte(INT_GEN_CFG_XL, temp);
}

void LSM9DS1::configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL. Axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_X_XL + axis, threshold);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);

	if (wait) 
		temp |= 0x80;

	xgWriteByte(INT_GEN_DUR_XL, temp);
}

void LSM9DS1::configGyroInt(uint8_t generator, bool aoi, bool latch)
{
	// Use variables from accel_interrupt_generator, OR'd together to create the [generator] value.
	uint8_t temp = generator;

	if (aoi) 
		temp |= 0x80;

	if (latch) 
		temp |= 0x40;

	xgWriteByte(INT_GEN_CFG_G, temp);
}

void LSM9DS1::configMagInt(uint8_t generator, h_lactive activeLow, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);

	// IEA bit is 0 for active-low, 1 for active-high.
	if(activeLow == INT_ACTIVE_HIGH) 
		config |= (1 << 2);

	// IEL bit is 0 for latched, 1 for not-latched
	if(!latch) 
		config |= (1 << 1);

	// As long as we have at least 1 generator, enable the interrupt
	if(generator != 0) 
		config |= (1 << 0);
	
	mWriteByte(INT_CFG_M, config);
}

void LSM9DS1::configMagThs(uint16_t threshold)
{
	// Write high eight bits of [threshold] to INT_THS_H_M
	mWriteByte(INT_THS_H_M, uint8_t((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	mWriteByte(INT_THS_L_M, uint8_t(threshold & 0x00FF));
}

void LSM9DS1::configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
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
	if (wait) 
		temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_G, temp);
}

uint8_t LSM9DS1::getInactivity()
{
	uint8_t temp = xgReadByte(STATUS_REG_0);
	temp &= (0x10);
	return temp;
}

uint8_t LSM9DS1::getAccelIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_XL);
	
	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1 << 6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

uint8_t LSM9DS1::getGyroIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_G);
	
	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1 << 6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

uint8_t LSM9DS1::getMagIntSrc()
{
	uint8_t intSrc = mReadByte(INT_SRC_M);
	
	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1 << 0))
	{
		return (intSrc & 0xFE);
	}
	
	return 0;
}

/*--------------------------------------
 * FIFO
 *-------------------------------------*/
void LSM9DS1::enableFIFO(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);

	if (enable) 
		temp |= (1 << 1);
	else 
		temp &= ~(1 << 1);

	xgWriteByte(CTRL_REG9, temp);
}

void LSM9DS1::setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t LSM9DS1::getFIFOSamples()
{
	return (xgReadByte(FIFO_SRC) & 0x3F);
}

/*--------------------------------------
 * MISC
 *-------------------------------------*/
void LSM9DS1::constrainScales()
{
	if ((settings.gyro.scale != 245) && (settings.gyro.scale != 500) && 
		(settings.gyro.scale != 2000))
	{
		settings.gyro.scale = 245;
	}
		
	if ((settings.accel.scale != 2) && (settings.accel.scale != 4) &&
		(settings.accel.scale != 8) && (settings.accel.scale != 16))
	{
		settings.accel.scale = 2;
	}
		
	if ((settings.mag.scale != 4) && (settings.mag.scale != 8) &&
		(settings.mag.scale != 12) && (settings.mag.scale != 16))
	{
		settings.mag.scale = 4;
	}
}

void LSM9DS1::sleepGyro(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);

	if (enable) 
		temp |= (1 << 6);
	else 
		temp &= ~(1 << 6);

	xgWriteByte(CTRL_REG9, temp);
}

/*--------------------------------------
 * Low Level I/O
 *-------------------------------------*/
void LSM9DS1::xgWriteByte(uint8_t subAddress, uint8_t data)
{
	if(settings.device.commInterface == IMU_MODE_I2C)
		I2CwriteByte(_xgAddress, subAddress, data);
	else if(settings.device.commInterface == IMU_MODE_SPI)
		SPIwriteByte(csPinXG, subAddress, data);
}

void LSM9DS1::mWriteByte(uint8_t subAddress, uint8_t data)
{
	if(settings.device.commInterface == IMU_MODE_I2C)
		return I2CwriteByte(_mAddress, subAddress, data);
	else if(settings.device.commInterface == IMU_MODE_SPI)
		return SPIwriteByte(csPinM, subAddress, data);
}

uint8_t LSM9DS1::xgReadByte(uint8_t subAddress)
{
	if(settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadByte(_xgAddress, subAddress);
	else if(settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadByte(csPinXG, subAddress);
}

uint8_t LSM9DS1::xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	if(settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadBytes(_xgAddress, subAddress, dest, count);
	else if(settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadBytes(csPinXG, subAddress, dest, count);
}

uint8_t LSM9DS1::mReadByte(uint8_t subAddress)
{
	if(settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadByte(_mAddress, subAddress);
	else if(settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadByte(csPinM, subAddress);
}

uint8_t LSM9DS1::mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	if(settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadBytes(_mAddress, subAddress, dest, count);
	else if(settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadBytes(csPinM, subAddress, dest, count);
}

void LSM9DS1::initSPI()
{
	csPinXG->mode(OUTPUT_PUSH_PULL); 
	csPinXG->write(HIGH);

	csPinM->mode(OUTPUT_PUSH_PULL); 
	csPinM->write(HIGH);
	
	//Note: Max freq for LSM9DS1 is ~10MHz
	//2EDGE, LO: works 
	//2EDGE, HI: works 

	Chimera::SPI::Setup settings;
	settings.mode = MASTER;
	settings.bitOrder = MSB_FIRST;
	settings.dataSize = DATASIZE_8BIT;
	settings.clockMode = MODE2;
	settings.clockFrequency = 6000000;
	
	spi->begin(settings);

	/* Even though FreeRTOS is supported, SPI is still kept in blocking mode because reading 
	 * a full set of data only takes tens of uS. This is far shorter than the tick rate of FreeRTOS. */
	spi->setMode(SubPeripheral::TXRX, SubPeripheralMode::BLOCKING);
}

void LSM9DS1::SPIwriteByte(const GPIOClass_sPtr& csPin, uint8_t subAddress, uint8_t data)
{
	uint8_t command[] = { static_cast<uint8_t>(subAddress & 0x3F), data };
	
	csPin->write(LOW);
	spi->write(command, 2);
	csPin->write(HIGH);
}

uint8_t LSM9DS1::SPIreadByte(const GPIOClass_sPtr& csPin, uint8_t subAddress)
{
	uint8_t temp;
	SPIreadBytes(csPin, subAddress, &temp, 1);
	return temp;
}

uint8_t LSM9DS1::SPIreadBytes(const GPIOClass_sPtr& csPin, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	/* Indicate a read by setting MSB to 1. For Accelerometer and Gyroscope,
	 * the read address will be automatically incremented if the IF_ADD_INC bit
	 * is set in CTRL_REG_8, which it is by default. */
	uint8_t rAddress = 0x80 | (subAddress & 0x3F);
	
	/* Reading multiple bytes from Magnetometer requires explicitly setting bit
	 * two. Apparently it is not affected by IF_ADD_INC. */
	if((csPin == csPinM) && count > 1)
		rAddress |= 0x40;
	
	/* Read out the data using SPI transmit/receive */
	csPin->write(LOW);
	spi->write(&rAddress, 1, false);			
	spi->write(_dummyBuffer, dest, count, true);
	csPin->write(HIGH);
	
	return count;
}

void LSM9DS1::initI2C()
{
	//Wire.begin(); 	// Initialize I2C library
}

void LSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	//Wire.beginTransmission(address);   // Initialize the Tx buffer
	//Wire.write(subAddress);            // Put slave register address in Tx buffer
	//Wire.write(data);                  // Put data in Tx buffer
	//Wire.endTransmission();            // Send the Tx buffer
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data;  // `data` will store the register data	
	
	//Wire.beginTransmission(address);          // Initialize the Tx buffer
	//Wire.write(subAddress); 	                 // Put slave register address in Tx buffer
	//Wire.endTransmission(false);              // Send the Tx buffer, but send a restart to keep connection alive
	//Wire.requestFrom(address, (uint8_t) 1);   // Read one byte from slave register address 
	//
	//data = Wire.read();                       // Fill Rx buffer with result
	return data;                              // Return data read from slave register
}

uint8_t LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	//byte retVal;
	//Wire.beginTransmission(address);       // Initialize the Tx buffer
	//// Next send the register to be read. OR with 0x80 to indicate multi-read.
	//Wire.write(subAddress | 0x80);         // Put slave register address in Tx buffer
	//retVal = Wire.endTransmission(false);  // Send Tx buffer, send a restart to keep connection alive
	//if(retVal != 0) // endTransmission should return 0 on success
	//	return 0;
	//
	//retVal = Wire.requestFrom(address, count);   // Read bytes from slave register address 
	//if(retVal != count)
	//	return 0;
	//
	//for (int i = 0; i < count;)
	//	dest[i++] = Wire.read();
	
	return count;
}
