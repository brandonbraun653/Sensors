#include <stm32f7xx_hal.h>
#include <stm32_hal_legacy.h>

/* Thor Dependencies */
#include "core.h"

/* Sensor Dependencies */
#include "stm32f7_lsm9ds0.h"

/* Sensor Pinouts for SPI Use: 
VIN: 3-5V input
SCL:	CLK
SDA:	MOSI
CSG:	Gyro CS
SDOG:	Gyro MISO
CSXM:	Accelerometer & Magnetometer CS
SDOXM:	Accelerometer & Magnetometer MISO


Let's use SPI3 on the Nucleo Board and see how it works. CN11
CLK:  Yellow Cable
MISO: Blue Cable
MOSI: Brown Cable
CSG:  Orange Cable -> PF6
CSXM: White Cable  -> PF7

*/


int main(void)
{
	HAL_Init();
	SystemClockConfig();

	LSM9DS0_Settings* sensor_settings = new LSM9DS0_Settings;

	/* Select the protocol used */
	sensor_settings->interfaceMode = MODE_SPI;

	/* Fill in the Accelerometer and Mag details */
	sensor_settings->xmAddress = 0x1D;
	sensor_settings->xmCSPin = PIN_7;
	sensor_settings->xmCSPort = GPIOF;
	
	sensor_settings->gAddress = 0x6B; 
	sensor_settings->gCSPin = PIN_6;
	sensor_settings->gCSPort = GPIOF;

	sensor_settings->scale.accel = A_SCALE_2G;
	sensor_settings->scale.mag = M_SCALE_2GS;
	sensor_settings->scale.gyro = G_SCALE_245DPS;

	sensor_settings->odr.accel = A_ODR_50;
	sensor_settings->odr.mag = M_ODR_50;
	sensor_settings->odr.gyro = G_ODR_190_BW_125;
	

	/* Create the new sensor object */
	LSM9DS0 sensor(sensor_settings);
	sensor.initialize();

	for (;;)
	{
		sensor.read(ACCEL);
		sensor.read(GYRO);
		sensor.read(MAG);

		sensor.calc(ACCEL);
		sensor.calc(GYRO);
		sensor.calc(MAG);


		//HAL_Delay(1);
	}
}
