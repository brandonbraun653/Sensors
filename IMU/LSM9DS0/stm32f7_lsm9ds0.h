#pragma once
#ifndef STM_LSM9DS0_H_
#define STM_LSM9DS0_H_

#include <stdlib.h>
#include <stdint.h>

#if __has_include("stm32f7xx_hal.h")
#include "stm32f7xx_hal.h"
#elif __has_include("stm32f4xx_hal.h")
#include "stm32f4xx_hal.h"
#endif
/* TODO: Seriously need to improve this...later*/
#include "C:/git/Microcontrollers/thor_stm32/include/thor.h"
#include "C:/git/Microcontrollers/thor_stm32/include/thor_config.h"
#include "C:/git/Microcontrollers/thor_stm32/include/thor_definitions.h"
#include "C:/git/Microcontrollers/thor_stm32/include/gpio.h"
#include "C:/git/Microcontrollers/thor_stm32/include/spi.h"

#define LSM_READ_BIT (1u << 7)
#define LSM_AUTO_INCR_BIT (1u << 6)

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G			0x0F
#define CTRL_REG1_G			0x20
#define CTRL_REG2_G			0x21
#define CTRL_REG3_G			0x22
#define CTRL_REG4_G			0x23
#define CTRL_REG5_G			0x24
#define REFERENCE_G			0x25
#define STATUS_REG_G		0x27
#define OUT_X_L_G			0x28
#define OUT_X_H_G			0x29
#define OUT_Y_L_G			0x2A
#define OUT_Y_H_G			0x2B
#define OUT_Z_L_G			0x2C
#define OUT_Z_H_G			0x2D
#define FIFO_CTRL_REG_G		0x2E
#define FIFO_SRC_REG_G		0x2F
#define INT1_CFG_G			0x30
#define INT1_SRC_G			0x31
#define INT1_THS_XH_G		0x32
#define INT1_THS_XL_G		0x33
#define INT1_THS_YH_G		0x34
#define INT1_THS_YL_G		0x35
#define INT1_THS_ZH_G		0x36
#define INT1_THS_ZL_G		0x37
#define INT1_DURATION_G		0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM		0x05
#define OUT_TEMP_H_XM		0x06
#define STATUS_REG_M		0x07
#define OUT_X_L_M			0x08
#define OUT_X_H_M			0x09
#define OUT_Y_L_M			0x0A
#define OUT_Y_H_M			0x0B
#define OUT_Z_L_M			0x0C
#define OUT_Z_H_M			0x0D
#define WHO_AM_I_XM			0x0F
#define INT_CTRL_REG_M		0x12
#define INT_SRC_REG_M		0x13
#define INT_THS_L_M			0x14
#define INT_THS_H_M			0x15
#define OFFSET_X_L_M		0x16
#define OFFSET_X_H_M		0x17
#define OFFSET_Y_L_M		0x18
#define OFFSET_Y_H_M		0x19
#define OFFSET_Z_L_M		0x1A
#define OFFSET_Z_H_M		0x1B
#define REFERENCE_X			0x1C
#define REFERENCE_Y			0x1D
#define REFERENCE_Z			0x1E
#define CTRL_REG0_XM		0x1F
#define CTRL_REG1_XM		0x20
#define CTRL_REG2_XM		0x21
#define CTRL_REG3_XM		0x22
#define CTRL_REG4_XM		0x23
#define CTRL_REG5_XM		0x24
#define CTRL_REG6_XM		0x25
#define CTRL_REG7_XM		0x26
#define STATUS_REG_A		0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D
#define FIFO_CTRL_REG		0x2E
#define FIFO_SRC_REG		0x2F
#define INT_GEN_1_REG		0x30
#define INT_GEN_1_SRC		0x31
#define INT_GEN_1_THS		0x32
#define INT_GEN_1_DURATION	0x33
#define INT_GEN_2_REG		0x34
#define INT_GEN_2_SRC		0x35
#define INT_GEN_2_THS		0x36
#define INT_GEN_2_DURATION	0x37
#define CLICK_CFG			0x38
#define CLICK_SRC			0x39
#define CLICK_THS			0x3A
#define TIME_LIMIT			0x3B
#define TIME_LATENCY		0x3C
#define TIME_WINDOW			0x3D
#define ACT_THS				0x3E
#define ACT_DUR				0x3F

/*-----------------------------------
* Sensor Specific Structures
*-----------------------------------*/
typedef enum
{
	A_SCALE_2G,	// 000:  2g
	A_SCALE_4G,	// 001:  4g
	A_SCALE_6G,	// 010:  6g
	A_SCALE_8G,	// 011:  8g
	A_SCALE_16G	// 100:  16g
} accel_scale;

typedef enum
{
	G_SCALE_245DPS,		// 00:  245 degrees per second
	G_SCALE_500DPS,		// 01:  500 dps
	G_SCALE_2000DPS,	// 10:  2000 dps
} gyro_scale;

typedef enum
{
	M_SCALE_2GS,	// 00:  2Gs
	M_SCALE_4GS, 	// 01:  4Gs
	M_SCALE_8GS,	// 10:  8Gs
	M_SCALE_12GS,	// 11:  12Gs
} mag_scale;

typedef enum
{
	A_POWER_DOWN, 	// Power-down mode (0x0)
	A_ODR_3125,		// 3.125 Hz	(0x1)
	A_ODR_625,		// 6.25 Hz (0x2)
	A_ODR_125,		// 12.5 Hz (0x3)
	A_ODR_25,		// 25 Hz (0x4)
	A_ODR_50,		// 50 Hz (0x5)
	A_ODR_100,		// 100 Hz (0x6)
	A_ODR_200,		// 200 Hz (0x7)
	A_ODR_400,		// 400 Hz (0x8)
	A_ODR_800,		// 800 Hz (9)
	A_ODR_1600		// 1600 Hz (0xA)
} accel_odr;

typedef enum
{							// ODR (Hz) --- Cutoff
	G_ODR_95_BW_125  = 0x0, //   95         12.5
	G_ODR_95_BW_25   = 0x1, //   95          25
	// 0x2 and 0x3 define the same data rate and bandwidth
	G_ODR_190_BW_125 = 0x4, //   190        12.5
	G_ODR_190_BW_25  = 0x5, //   190         25
	G_ODR_190_BW_50  = 0x6, //   190         50
	G_ODR_190_BW_70  = 0x7, //   190         70
	G_ODR_380_BW_20  = 0x8, //   380         20
	G_ODR_380_BW_25  = 0x9, //   380         25
	G_ODR_380_BW_50  = 0xA, //   380         50
	G_ODR_380_BW_100 = 0xB, //   380         100
	G_ODR_760_BW_30  = 0xC, //   760         30
	G_ODR_760_BW_35  = 0xD, //   760         35
	G_ODR_760_BW_50  = 0xE, //   760         50
	G_ODR_760_BW_100 = 0xF, //   760         100
} gyro_odr;

typedef enum
{
	M_ODR_3125,	// 3.125 Hz (0x00)
	M_ODR_625,	// 6.25 Hz (0x01)
	M_ODR_125,	// 12.5 Hz (0x02)
	M_ODR_25,	// 25 Hz (0x03)
	M_ODR_50,	// 50 (0x04)
	M_ODR_100,	// 100 Hz (0x05)
} mag_odr;

typedef enum
{
	A_ABW_773,		// 773 Hz (0x0)
	A_ABW_194,		// 194 Hz (0x1)
	A_ABW_362,		// 362 Hz (0x2)
	A_ABW_50,		//  50 Hz (0x3)
} accel_abw;

/*-----------------------------------
* User Specific Structures
*-----------------------------------*/
struct LSM9DS0_Settings
{
	/* Settings Data */
	struct LSM_ODR
	{
		accel_odr accel;
		gyro_odr gyro;
		mag_odr mag;
	} odr;
	
	struct LSM_Scales
	{
		gyro_scale 	gyro;
		accel_scale accel;
		mag_scale 	mag;
	} scale;
	
	struct LSM_Resolution
	{
		float gyro;
		float accel;
		float mag;
	} res;
	
	struct LSM_Bias
	{
		struct vec3f
		{
			float x = 0.0;
			float y = 0.0;
			float z = 0.0;
		};
		
		vec3f accel;
		vec3f gyro;
		vec3f mag;
	} bias;
	
	accel_abw abw_accel;

	/* Chip Addressses */
	uint8_t xmAddress = 0x1D;
	uint8_t gAddress = 0x6B;
};

typedef enum
{
	ACCEL = (1u << 1),
	GYRO = (1u << 2),
	MAG = (1u << 3),
	TEMP = (1u << 4),
	ALL = (ACCEL | GYRO | MAG | TEMP)
} sensor_t;

/*-----------------------------------
* Primary Class Definition
*-----------------------------------*/
class LSM9DS0
{
public:
	/*----------------------------
	* Main interface functions
	*----------------------------*/
	bool initialize(bool calibrate_on_setup = false);
	void calibrate(sensor_t chip);
	
	void readAll();
	void readAccel();
	void readGyro();
	void readMag();
	void readTemp();
	
	void calcAll();
	void calcAccel();
	void calcGyro();
	void calcMag();
	void calcTemp();
	
	/* Sets the measurement scales */
	void setScale_gyro(gyro_scale gScl);
	void setScale_accel(accel_scale aScl);
	void setScale_mag(mag_scale mScl);

	/* Sets the output data rates */
	void setODR_gyro(gyro_odr gODR);
	void setODR_accel(accel_odr aODR);
	void setODR_mag(mag_odr mODR);
	void setABW_accel(accel_abw aBW);

	/*----------------------------
	* Latest Sensor Data (float)
	*----------------------------*/
	struct _sensorData
	{
		struct vec3f
		{
			float x = 0.0;
			float y = 0.0;
			float z = 0.0;
		};
		
		vec3f accel;
		vec3f gyro;
		vec3f mag;
	} data;
	

	/*----------------------------
	* Constructor/Destructor
	*----------------------------*/
	LSM9DS0(SPIClass_sPtr spi_instance, GPIOClass_sPtr xm_ss_pin, GPIOClass_sPtr g_ss_pin, LSM9DS0_Settings lsm_settings);
	~LSM9DS0();

private:
	/*----------------------------
	* Objects
	*----------------------------*/
	LSM9DS0_Settings settings;
	SPIClass_sPtr spi;
	GPIOClass_sPtr xm_CSPin, g_CSPin;

	/*----------------------------
	* Data Handlers
	*----------------------------*/
	/* Raw Sensor Data */
	
	
	struct _rawData
	{
		struct LSM_RawData
		{
			int16_t x;
			int16_t y;
			int16_t z;
		};
		
		LSM_RawData gyro;
		LSM_RawData accel;
		LSM_RawData mag;	
	} rawData;

	/* Buffers for sending and receiving data */
	uint8_t cmd_pkt[32];
	uint8_t rcv_pkt[32];

	/*----------------------------
	* Helper Functions
	*----------------------------*/

	/* Reads sensor data */
	void readDevice(sensor_t chip);

	/* Resolution Calculations */
	void calc_gRes();
	void calc_mRes();
	void calc_aRes();

	/* Converts from raw data to float */
	void calc_gyro();
	void calc_accel();
	void calc_mag();

	/* Initializes sensors */
	void init_gyro();
	void init_accel();
	void init_mag();

	/* Hardware Level Interfacing */
	void set_CS(sensor_t chip);
	void clr_CS(sensor_t chip);
	void write_pkt(sensor_t chip, uint8_t *cmd_buffer, size_t length);
	void read_pkt(sensor_t chip, uint8_t *cmd_buffer, uint8_t *rcv_buffer, size_t length);

	/* Buffer Management */
	void clr_cmd_pkt();
	void clr_rcv_pkt();
};

#endif /* !STM_LSM9DS0_H_ */
