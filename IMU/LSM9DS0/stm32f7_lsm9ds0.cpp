#include "stm32f7_lsm9ds0.h"

using namespace ThorDef::SPI;
using namespace ThorDef::GPIO;

/*--------------------------------------------------------
* Constructor & Destructor
*--------------------------------------------------------*/
LSM9DS0::LSM9DS0(SPIClass_sPtr spi_instance, GPIOClass_sPtr xm_ss_pin, GPIOClass_sPtr g_ss_pin, LSM9DS0_Settings lsm_settings)
{
	settings = lsm_settings;
	
	/* Initialize the Slave Selects */
	xm_CSPin = xm_ss_pin;
	g_CSPin = g_ss_pin;
	xm_CSPin->mode(OUTPUT_PP); xm_CSPin->write(HIGH);
	g_CSPin->mode(OUTPUT_PP); g_CSPin->write(HIGH);
	
	/* Initialize SPI */
	spi = spi_instance;
	
	SPI_InitTypeDef settings = Defaults::SPI::dflt_SPI_Init;
	
	settings.CLKPhase = SPI_PHASE_2EDGE;
	settings.CLKPolarity = SPI_POLARITY_HIGH;
	
	spi->setSSMode(SS_MANUAL_CONTROL);
	spi->attachSettings(settings);
	spi->reInitialize();
	spi->begin(EXTERNAL_SLAVE_SELECT);
	
	spi->setTxModeBlock();
	spi->setRxModeBlock();
}
	
LSM9DS0::~LSM9DS0()
{
}

/*--------------------------------------------------------
* Public Functions
*--------------------------------------------------------*/
bool LSM9DS0::initialize(bool calibrate_on_setup)
{
	/*----------------------------------
	 * Check for the correct sensor
	 *---------------------------------*/
	bool sensorPresent = true;
	
	/* Gyro */
	clr_cmd_pkt();
	cmd_pkt[0] = WHO_AM_I_G | LSM_READ_BIT;
	read_pkt(GYRO, cmd_pkt, rcv_pkt, 2);
	
	if (rcv_pkt[1] != 0xD4)
		sensorPresent = false;

	/* Accel/Mag */
	cmd_pkt[0] = WHO_AM_I_XM | LSM_READ_BIT;
	read_pkt(ACCEL, cmd_pkt, rcv_pkt, 2);
	
	if (rcv_pkt[1] != 0x49)
		sensorPresent = false;
	
	
	/*----------------------------------
	 * Get the correct resolution
	 *---------------------------------*/
	calc_gRes(); // Calculate DPS / ADC tick
	calc_mRes(); // Calculate  Gs / ADC tick
	calc_aRes(); // Calculate   g / ADC tick

	
	/*----------------------------------
	 * Setup Each Sensor
	 *---------------------------------*/
	init_gyro();
	setODR_gyro(settings.odr.gyro);
	setScale_gyro(settings.scale.gyro);
	
	init_accel();
	setODR_accel(settings.odr.accel);
	setScale_accel(settings.scale.accel);
	
	init_mag();
	setODR_mag(settings.odr.mag);
	setScale_mag(settings.scale.mag);
	
	if (calibrate_on_setup)
	{
		calibrate(GYRO);
		calibrate(ACCEL);
		calibrate(MAG);
	}
	
	return sensorPresent;
}

void LSM9DS0::readAll()
{
	readDevice(ACCEL); 
	readDevice(GYRO); 
	readDevice(MAG); 
	//readDevice(TEMP);
}

void LSM9DS0::readAccel()
{
	readDevice(ACCEL);
}

void LSM9DS0::readGyro()
{
	readDevice(GYRO); 
}

void LSM9DS0::readMag()
{
	readDevice(MAG); 
}

void LSM9DS0::readTemp()
{
	//readDevice(TEMP);
}

void LSM9DS0::calcAll()
{
	calcAccel();
	calcGyro();
	calcMag();
}

void LSM9DS0::calcAccel()
{
	calc_accel();
}

void LSM9DS0::calcGyro()
{
	calc_gyro();
}

void LSM9DS0::calcMag()
{
	calc_mag(); 
}

void LSM9DS0::calcTemp()
{
	//TODO
}

void LSM9DS0::calibrate(sensor_t chip)
{
	/*---------------------------------------
	* GYRO
	*---------------------------------------*/
	if (chip == GYRO)
	{
		clr_cmd_pkt();
		int16_t _gyro_bias[3] = { 0, 0, 0 };
		uint8_t samples;

		// Grab the current register settings
		cmd_pkt[0] = CTRL_REG5_G | LSM_READ_BIT;
		read_pkt(GYRO, cmd_pkt, rcv_pkt, 2);

		// Enable the FIFO and then wait for it to take effect
		cmd_pkt[0] = CTRL_REG5_G;
		cmd_pkt[1] = rcv_pkt[1] | 0x40;
		write_pkt(GYRO, cmd_pkt, 2);
		HAL_Delay(20);

		// Enable gyro FIFO stream mode and set watermark at 32 samples
		cmd_pkt[0] = FIFO_CTRL_REG_G;
		cmd_pkt[1] = 0x20 | 0x1F;
		write_pkt(GYRO, cmd_pkt, 2);
		HAL_Delay(1000);

		// Read number of stored samples
		cmd_pkt[0] = FIFO_SRC_REG_G | LSM_READ_BIT;
		cmd_pkt[1] = 0x00;
		read_pkt(GYRO, cmd_pkt, rcv_pkt, 2);
		samples = (rcv_pkt[1] & 0x1F);

		// Read the gyro data stored in the FIFO
		clr_cmd_pkt();
		cmd_pkt[0] = OUT_X_L_G | LSM_READ_BIT | LSM_AUTO_INCR_BIT;

		for (uint32_t i = 0; i < samples; i++) {
			read_pkt(GYRO, cmd_pkt, rcv_pkt, 7);
			_gyro_bias[0] += (((int16_t)rcv_pkt[2] << 8) | rcv_pkt[1]);
			_gyro_bias[1] += (((int16_t)rcv_pkt[4] << 8) | rcv_pkt[3]);
			_gyro_bias[2] += (((int16_t)rcv_pkt[6] << 8) | rcv_pkt[5]);
		}

		// average the data
		_gyro_bias[0] /= samples;
		_gyro_bias[1] /= samples;
		_gyro_bias[2] /= samples;

		// Properly scale the data to get deg/s
		settings.bias.gyro.x = (float)_gyro_bias[0] * settings.res.gyro;
		settings.bias.gyro.y = (float)_gyro_bias[1] * settings.res.gyro;
		settings.bias.gyro.z = (float)_gyro_bias[2] * settings.res.gyro;

		// Grab the current register settings
		cmd_pkt[0] = CTRL_REG5_G | LSM_READ_BIT;
		read_pkt(GYRO, cmd_pkt, rcv_pkt, 2);

		// Disable the FIFO and then wait for it to take effect
		cmd_pkt[0] = CTRL_REG5_G;
		cmd_pkt[1] = rcv_pkt[1] & ~0x40;
		write_pkt(GYRO, cmd_pkt, 2);
		HAL_Delay(20);

		// Enable gyro bypass mode
		cmd_pkt[0] = FIFO_CTRL_REG_G;
		cmd_pkt[1] = 0x00;
		write_pkt(GYRO, cmd_pkt, 2);
	}

	/*---------------------------------------
	* ACCELEROMETER
	*---------------------------------------*/
	if (chip == ACCEL)
	{
		clr_cmd_pkt();
		uint16_t _accel_bias[3] = { 0, 0, 0 };
		uint8_t samples;

		// Grab the current register settings
		cmd_pkt[0] = CTRL_REG0_XM | LSM_READ_BIT;
		read_pkt(ACCEL, cmd_pkt, rcv_pkt, 2);

		// Enable accelerometer FIFO and wait for it to take effect
		cmd_pkt[0] = CTRL_REG0_XM;
		cmd_pkt[1] = rcv_pkt[1] | 0x40;
		write_pkt(ACCEL, cmd_pkt, 2);
		HAL_Delay(20);

		// Enable accelerometer FIFO stream mode and set watermark at 32 samples
		cmd_pkt[0] = FIFO_CTRL_REG;
		cmd_pkt[1] = 0x20 | 0x1F;
		write_pkt(ACCEL, cmd_pkt, 2);
		HAL_Delay(1000);

		// Read number of stored samples
		cmd_pkt[0] = FIFO_SRC_REG | LSM_READ_BIT;
		cmd_pkt[1] = 0x00;
		read_pkt(ACCEL, cmd_pkt, rcv_pkt, 2);
		samples = (rcv_pkt[1] & 0x1F);

		// Read the accelerometer data stored in the FIFO
		clr_cmd_pkt();
		cmd_pkt[0] = OUT_X_L_A | LSM_READ_BIT | LSM_AUTO_INCR_BIT;

		for (uint32_t i = 0; i < samples; i++) {
			read_pkt(ACCEL, cmd_pkt, rcv_pkt, 7);
			_accel_bias[0] += (((int16_t)rcv_pkt[2] << 8) | rcv_pkt[1]);
			_accel_bias[1] += (((int16_t)rcv_pkt[4] << 8) | rcv_pkt[3]);
			_accel_bias[2] += (((int16_t)rcv_pkt[6] << 8) | rcv_pkt[5]) - (int16_t)(1.0f / settings.res.accel); // Assumes sensor facing up!
		}

		// average the data
		_accel_bias[0] /= samples;
		_accel_bias[1] /= samples;
		_accel_bias[2] /= samples;

		// Properly scale data to get gs
		settings.bias.accel.x = (float)_accel_bias[0] * settings.res.accel;
		settings.bias.accel.y = (float)_accel_bias[1] * settings.res.accel;
		settings.bias.accel.z = (float)_accel_bias[2] * settings.res.accel;

		// Grab the current register settings
		cmd_pkt[0] = CTRL_REG0_XM | LSM_READ_BIT;
		read_pkt(ACCEL, cmd_pkt, rcv_pkt, 2);

		// Disable the FIFO and then wait for it to take effect
		cmd_pkt[0] = CTRL_REG0_XM;
		cmd_pkt[1] = rcv_pkt[1] & ~0x40;
		write_pkt(ACCEL, cmd_pkt, 2);
		HAL_Delay(20);

		// Enable accelerometer bypass mode
		cmd_pkt[0] = FIFO_CTRL_REG;
		cmd_pkt[1] = 0x00;
		write_pkt(ACCEL, cmd_pkt, 2);
	}

	/*---------------------------------------
	* MAGNETOMETER
	*---------------------------------------*/
	if (chip == MAG)
	{
	}

	/*---------------------------------------
	* TEMPERATURE
	*---------------------------------------*/
	if (chip == TEMP)
	{
	}
}

/*--------------------------------------------------------
* Private Functions
*--------------------------------------------------------*/
void LSM9DS0::readDevice(sensor_t chip)
{
	clr_cmd_pkt();
	clr_rcv_pkt();

	switch (chip)
	{
	case ACCEL:
		// Read out 6 bytes of data, starting from OUT_X_L_A
		cmd_pkt[0] = OUT_X_L_A | LSM_READ_BIT | LSM_AUTO_INCR_BIT; //
		read_pkt(ACCEL, cmd_pkt, rcv_pkt, 7); //First byte received is garbage. Ignore.

		rawData.accel.x = (rcv_pkt[2] << 8) | rcv_pkt[1];
		rawData.accel.y = (rcv_pkt[4] << 8) | rcv_pkt[3];
		rawData.accel.z = (rcv_pkt[6] << 8) | rcv_pkt[5];
		break;

	case GYRO:
		// Read out 6 bytes of data, starting from OUT_X_L_G
		cmd_pkt[0] = OUT_X_L_G | LSM_READ_BIT | LSM_AUTO_INCR_BIT; //
		read_pkt(GYRO, cmd_pkt, rcv_pkt, 7); //First byte received is garbage. Ignore.

		rawData.gyro.x = (rcv_pkt[2] << 8) | rcv_pkt[1];
		rawData.gyro.y = (rcv_pkt[4] << 8) | rcv_pkt[3];
		rawData.gyro.z = (rcv_pkt[6] << 8) | rcv_pkt[5];
		break;

	case MAG:
		// Read out 6 bytes of data, starting from OUT_X_L_M
		cmd_pkt[0] = OUT_X_L_M | LSM_READ_BIT | LSM_AUTO_INCR_BIT; //
		read_pkt(MAG, cmd_pkt, rcv_pkt, 7); //First byte received is garbage. Ignore.

		rawData.mag.x = (rcv_pkt[2] << 8) | rcv_pkt[1];
		rawData.mag.y = (rcv_pkt[4] << 8) | rcv_pkt[3];
		rawData.mag.z = (rcv_pkt[6] << 8) | rcv_pkt[5];
		break;

	case TEMP:
		// Read out 2 bytes of data, starting from OUT_TEMP_L_XM
		cmd_pkt[0] = OUT_TEMP_L_XM | LSM_READ_BIT | LSM_AUTO_INCR_BIT; //
		read_pkt(TEMP, cmd_pkt, rcv_pkt, 3); //First byte received is garbage. Ignore.

		//temperature_data = (((int16_t)rcv_pkt[2] << 12) | rcv_pkt[1] << 4) >> 4; // Temperature is a 12-bit signed integer
		break;

	default: break;
	}
}

void LSM9DS0::calc_aRes()
{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:
	settings.res.accel = settings.scale.accel == A_SCALE_16G ? 16.0f / 32768.0f :
		(((float)settings.scale.accel + 1.0f) * 2.0f) / 32768.0f;
}

void LSM9DS0::calc_mRes()
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	settings.res.mag = settings.scale.mag == M_SCALE_2GS ? 2.0f / 32768.0f :
		(float)(settings.scale.mag << 2) / 32768.0f;
}

void LSM9DS0::calc_gRes()
{
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	switch (settings.scale.gyro)
	{
	case G_SCALE_245DPS:
		settings.res.gyro = 245.0f / 32768.0f;
		break;
	case G_SCALE_500DPS:
		settings.res.gyro = 500.0f / 32768.0f;
		break;
	case G_SCALE_2000DPS:
		settings.res.gyro = 2000.0f / 32768.0f;
		break;
	}
}

void LSM9DS0::calc_gyro()
{
	data.gyro.x = (settings.res.gyro * (float)rawData.gyro.x);
	data.gyro.y = (settings.res.gyro * (float)rawData.gyro.y);
	data.gyro.z = (settings.res.gyro * (float)rawData.gyro.z);
}

void LSM9DS0::calc_accel()
{
	data.accel.x = (settings.res.accel * (float)rawData.accel.x * 9.8f);
	data.accel.y = (settings.res.accel * (float)rawData.accel.y * 9.8f);
	data.accel.z = (settings.res.accel * (float)rawData.accel.z * 9.8f);
}

void LSM9DS0::calc_mag()
{
	data.mag.x = (settings.res.mag * (float)rawData.mag.x);
	data.mag.y = (settings.res.mag * (float)rawData.mag.y);
	data.mag.z = (settings.res.mag * (float)rawData.mag.z);
}

void LSM9DS0::write_pkt(sensor_t chip, uint8_t *cmd_buffer, size_t length)
{
	clr_CS(chip);
	spi->write(cmd_buffer, length);
	set_CS(chip);
}

void LSM9DS0::read_pkt(sensor_t chip, uint8_t *cmd_buffer, uint8_t *rcv_buffer, size_t length)
{
	memset(rcv_buffer, 0x00, sizeof(rcv_buffer) / sizeof(rcv_buffer[0]));
	
	clr_CS(chip);
	spi->write(cmd_buffer, rcv_buffer, length);
	set_CS(chip);
}

void LSM9DS0::set_CS(sensor_t chip)
{
	switch (chip)
	{
	case ACCEL:
	case MAG:
		xm_CSPin->write(HIGH);
		break;

	case GYRO:
		g_CSPin->write(HIGH);

	case TEMP:
	default:
		break;
	}
}

void LSM9DS0::clr_CS(sensor_t chip)
{
	switch (chip)
	{
	case ACCEL:
	case MAG:
		xm_CSPin->write(LOW);
		break;

	case GYRO:
		g_CSPin->write(LOW);

	case TEMP:
	default:
		break;
	}
}

void LSM9DS0::init_gyro()
{
	/* Queue up the packet structure to send out */
	memset(cmd_pkt, 0, sizeof(cmd_pkt) / sizeof(cmd_pkt[0]));

	cmd_pkt[0] = CTRL_REG1_G;
	cmd_pkt[1] = 0x0F;
	cmd_pkt[2] = CTRL_REG2_G;
	cmd_pkt[3] = 0x00;
	cmd_pkt[4] = CTRL_REG3_G;
	cmd_pkt[5] = 0x88;
	cmd_pkt[6] = CTRL_REG4_G;
	cmd_pkt[7] = 0x00;
	cmd_pkt[8] = CTRL_REG5_G;
	cmd_pkt[9] = 0x00;

	write_pkt(GYRO, cmd_pkt, 2);
	write_pkt(GYRO, &cmd_pkt[2], 2);
	write_pkt(GYRO, &cmd_pkt[4], 2);
	write_pkt(GYRO, &cmd_pkt[6], 2);
	write_pkt(GYRO, &cmd_pkt[8], 2);
}

void LSM9DS0::init_accel()
{
	/* Queue up the packet structure to send out */
	memset(cmd_pkt, 0, sizeof(cmd_pkt) / sizeof(cmd_pkt[0]));

	cmd_pkt[0] = CTRL_REG0_XM;
	cmd_pkt[1] = 0x00;
	cmd_pkt[2] = CTRL_REG1_XM;
	cmd_pkt[3] = 0x57;
	cmd_pkt[4] = CTRL_REG2_XM;
	cmd_pkt[5] = 0x00;
	cmd_pkt[6] = CTRL_REG3_XM;
	cmd_pkt[7] = 0x04;

	write_pkt(ACCEL, cmd_pkt, 2);
	write_pkt(ACCEL, &cmd_pkt[2], 2);
	write_pkt(ACCEL, &cmd_pkt[4], 2);
	write_pkt(ACCEL, &cmd_pkt[6], 2);
}

void LSM9DS0::init_mag()
{
	/* Queue up the packet structure to send out */
	memset(cmd_pkt, 0, sizeof(cmd_pkt) / sizeof(cmd_pkt[0]));

	cmd_pkt[0] = CTRL_REG5_XM;
	cmd_pkt[1] = 0x94;
	cmd_pkt[2] = CTRL_REG6_XM;
	cmd_pkt[3] = 0x00;
	cmd_pkt[4] = CTRL_REG7_XM;
	cmd_pkt[5] = 0x00;
	cmd_pkt[6] = CTRL_REG4_XM;
	cmd_pkt[7] = 0x04;
	cmd_pkt[8] = INT_CTRL_REG_M;
	cmd_pkt[9] = 0x09;

	write_pkt(MAG, cmd_pkt, 2);
	write_pkt(MAG, &cmd_pkt[2], 2);
	write_pkt(MAG, &cmd_pkt[4], 2);
	write_pkt(MAG, &cmd_pkt[6], 2);
	write_pkt(MAG, &cmd_pkt[8], 2);
}

void LSM9DS0::setScale_gyro(gyro_scale gScl)
{
	clr_cmd_pkt();
	clr_rcv_pkt();

	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	cmd_pkt[0] = CTRL_REG4_G | LSM_READ_BIT; //Ensure we ask for a read
	read_pkt(GYRO, cmd_pkt, rcv_pkt, 2);

	// Then mask out the gyro scale bits and shift in new scale bits:
	rcv_pkt[1] &= 0xFF ^ (0x3 << 4);
	rcv_pkt[1] |= gScl << 4;

	// And write the new register value back into CTRL_REG4_G:
	cmd_pkt[0] &= ~LSM_READ_BIT; //Ensure we are writing
	cmd_pkt[1] = rcv_pkt[1];
	write_pkt(GYRO, cmd_pkt, 2);

	// Update class variables and calculate a new resolution
	settings.scale.gyro = gScl;
	calc_gRes();
}

void LSM9DS0::setScale_accel(accel_scale aScl)
{
	clr_cmd_pkt();
	clr_rcv_pkt();

	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	cmd_pkt[0] = CTRL_REG2_XM | LSM_READ_BIT; //Ensure we ask for a read
	read_pkt(ACCEL, cmd_pkt, rcv_pkt, 2);

	// Then mask out the accel scale bits and shift in new scale bits:
	rcv_pkt[1] &= 0xFF ^ (0x3 << 3);
	rcv_pkt[1] |= aScl << 3;

	// And write the new register value back into CTRL_REG2_XM:
	cmd_pkt[0] &= ~LSM_READ_BIT; //Ensure we are writing
	cmd_pkt[1] = rcv_pkt[1];
	write_pkt(ACCEL, cmd_pkt, 2);

	// Update class variable and calculate a new resolution
	settings.scale.accel = aScl;
	calc_aRes();
}

void LSM9DS0::setScale_mag(mag_scale mScl)
{
	clr_cmd_pkt();
	clr_rcv_pkt();

	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	cmd_pkt[0] = CTRL_REG6_XM | LSM_READ_BIT; //Ensure we ask for a read
	read_pkt(MAG, cmd_pkt, rcv_pkt, 2);

	// Then mask out the accel scale bits and shift in new scale bits:
	rcv_pkt[1] &= 0xFF ^ (0x3 << 5);
	rcv_pkt[1] |= mScl << 5;

	// And write the new register value back into CTRL_REG6_XM:
	cmd_pkt[0] &= ~LSM_READ_BIT; //Ensure we are writing
	cmd_pkt[1] = rcv_pkt[1];
	write_pkt(MAG, cmd_pkt, 2);

	// Update class variable and calculate a new resolution
	settings.scale.mag = mScl;
	calc_mRes();
}

void LSM9DS0::setODR_gyro(gyro_odr gODR)
{
	clr_cmd_pkt();
	clr_rcv_pkt();

	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	cmd_pkt[0] = CTRL_REG1_G | LSM_READ_BIT; //Ensure we ask for a read
	read_pkt(GYRO, cmd_pkt, rcv_pkt, 2);

	// Then mask out the gyro ODR bits:
	rcv_pkt[1] &= 0xFF ^ (0xF << 4);
	rcv_pkt[1] |= (gODR << 4);

	// And write the new register value back into CTRL_REG1_G:
	cmd_pkt[0] &= ~LSM_READ_BIT; //Ensure we are writing
	cmd_pkt[1] = rcv_pkt[1];
	write_pkt(GYRO, cmd_pkt, 2);

	// Update class variable
	settings.odr.gyro = gODR;
}

void LSM9DS0::setODR_accel(accel_odr aODR)
{
	clr_cmd_pkt();
	clr_rcv_pkt();

	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	cmd_pkt[0] = CTRL_REG1_XM | LSM_READ_BIT; //Ensure we ask for a read
	read_pkt(ACCEL, cmd_pkt, rcv_pkt, 2);

	// Then mask out the accel ODR bits:
	rcv_pkt[1] &= 0xFF ^ (0xF << 4);
	rcv_pkt[1] |= (aODR << 4);

	// And write the new register value back into CTRL_REG1_XM:
	cmd_pkt[0] &= ~LSM_READ_BIT; //Ensure we are writing
	cmd_pkt[1] = rcv_pkt[1];
	write_pkt(ACCEL, cmd_pkt, 2);

	// Update class variable
	settings.odr.accel = aODR;
}

void LSM9DS0::setODR_mag(mag_odr mODR)
{
	clr_cmd_pkt();
	clr_rcv_pkt();

	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	cmd_pkt[0] = CTRL_REG5_XM | LSM_READ_BIT; //Ensure we ask for a read
	read_pkt(MAG, cmd_pkt, rcv_pkt, 2);

	// Then mask out the mag ODR bits:
	rcv_pkt[1] &= 0xFF ^ (0x7 << 2);
	rcv_pkt[1] |= (mODR << 2);

	// And write the new register value back into CTRL_REG5_XM:
	cmd_pkt[0] &= ~LSM_READ_BIT; //Ensure we are writing
	cmd_pkt[1] = rcv_pkt[1];
	write_pkt(MAG, cmd_pkt, 2);

	// Update class variable
	settings.odr.mag = mODR;
}

void LSM9DS0::setABW_accel(accel_abw aBW)
{
	clr_cmd_pkt();
	clr_rcv_pkt();

	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	cmd_pkt[0] = CTRL_REG2_XM | LSM_READ_BIT; //Ensure we ask for a read
	read_pkt(ACCEL, cmd_pkt, rcv_pkt, 2);

	// Then mask out the accel ABW bits:
	rcv_pkt[1] &= 0xFF ^ (0x3 << 7);
	rcv_pkt[1] |= (aBW << 7);

	// And write the new register value back into CTRL_REG2_XM:
	cmd_pkt[0] &= ~LSM_READ_BIT; //Ensure we are writing
	cmd_pkt[1] = rcv_pkt[1];
	write_pkt(ACCEL, cmd_pkt, 2);

	// Update class variable
	settings.abw_accel = aBW;
}

void LSM9DS0::clr_cmd_pkt()
{
	memset(cmd_pkt, 0x00, sizeof(cmd_pkt) / sizeof(cmd_pkt[0]));
}

void LSM9DS0::clr_rcv_pkt()
{
	memset(rcv_pkt, 0x00, sizeof(rcv_pkt) / sizeof(rcv_pkt[0]));
}