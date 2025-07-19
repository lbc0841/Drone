#include "qmi8658.h"

// SDO/SA0 = VDD -> QMI8658_ADDRESS = 0x6A
#define QMI8658_ADDRESS (0x6A << 1)


// Setup and Control Registers --------------------------------------------------------------------------
#define CTRL1 0x02
#define CTRL2 0x03
#define CTRL3 0x04
// 0x05 -> Reserved
#define CTRL5 0x06
// 0x07 -> Reserved
#define CTRL7 0x08
#define CTRL8 0x09
#define CTRL9 0x0A


// Data Output Registers --------------------------------------------------------------------------------
#define AX_L 0x35
#define GX_L 0x3B


uint8_t imuData[12] = {0};

// Calibration variables
AngularVelocity angCalibration = {0};
double scaleFactor_acc = 0;
double scaleFactor_ang = 0;


// I2C Callback
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c != &hi2c1) return;

	QMI8658_HandleImuData();

	if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
	{
		HAL_StatusTypeDef status = QMI8658_ReadRegisters_IT(AX_L, imuData, 12);
		if (status != HAL_OK) deviceError = 1;
	}
	else deviceError = 1;
}


// Read/Write Registers
HAL_I2C_StateTypeDef QMI8658_ReadRegisters(uint8_t memAddress, uint8_t* data, int size)
{
	return HAL_I2C_Mem_Read(
			&hi2c1, QMI8658_ADDRESS, memAddress, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}
HAL_I2C_StateTypeDef QMI8658_WriteRegisters(uint8_t memAddress, uint8_t data)
{
	uint8_t size = 1;
	return HAL_I2C_Mem_Write(
			&hi2c1, QMI8658_ADDRESS, memAddress, I2C_MEMADD_SIZE_8BIT, &data, size, HAL_MAX_DELAY);
}

HAL_I2C_StateTypeDef QMI8658_ReadRegisters_IT(uint8_t memAddress, uint8_t* data, int size)
{
	return HAL_I2C_Mem_Read_IT(
			&hi2c1, QMI8658_ADDRESS, memAddress, I2C_MEMADD_SIZE_8BIT, data, size);
}


void QMI8658_Init()
{
	/* CTRL1 :
	 * 7	: 0 - Enables 4-wire SPI interface
	 * 6	: 1 - Serial interface (SPI, I2C, I3C) address auto increment
	 * 5	: 0 - Serial interface (SPI, I2C, I3C) read data Little-Endian
	 * 4	: 0 - INT2 pin is high-Z mode
	 * 3	: 0 - INT1 pin is high-Z mode
	 * 2	: 0 - FIFO interrupt is mapped to INT2 pin
	 * 1	: 0 - Reserved
	 * 0	: 0 - Enable internal high-speed oscillator */
	QMI8658_WriteRegisters(CTRL1, 0b01000000);
	HAL_Delay(10);


	/* CTRL2 :
	 * 7	: 0 - Disable Accelerometer Self-Test
	 * 6:4	: 000 - Accelerometer Full-scale = ±2 g
	 * 3:0	: 0100 - Accelerometer Output Data Rate = 500 Hz */
	QMI8658_WriteRegisters(CTRL2, 0b00000100);
	scaleFactor_acc = (2*9.81)/32768.0;
	HAL_Delay(10);


	/* CTRL3 :
	 * 7	: 0 -  Disable Gyro self-Test
	 * 6:4	: 010 -  Gyroscope Full-scale = ±64 dps
	 * 3:0	: 0011 - Gyroscope Output Data Rate = 896.8 Hz */
	QMI8658_WriteRegisters(CTRL3, 0b00100011);
	scaleFactor_ang = 64.0/32768.0;
	HAL_Delay(10);


	QMI8658_WriteRegisters(CTRL5, 0x11);
	HAL_Delay(10);


	/* CTRL7 :
	 * 7	: 0 - Disable SyncSample mode
	 * 6	: 0 - Reserved
	 * 5	: 1 - DRDY(Data Ready) is disabled, is blocked from the INT2 pin
	 * 4	: 0 - Gyroscope in Full Mode (Drive and Sense are enabled).
	 * 3:2	: 0 - Reserved
	 * 1	: 1 - Enable Gyroscope.
	 * 0	: 1 - Enable Accelerometer */
	QMI8658_WriteRegisters(CTRL7, 0b00100011);
	HAL_Delay(10);


	HAL_Delay(30);
	QMI8658_Calibration();
}


#define CAL_AVG_TIME 20
void QMI8658_Calibration()
{
	AngularVelocity angSum;

	for(int i=0; i<CAL_AVG_TIME; i++)
	{
		QMI8658_ReadRegisters(AX_L, imuData, 12);

		uint16_t gx = (( uint16_t) imuData[7] ) << 8 | imuData[6];
		uint16_t gy = (( uint16_t) imuData[9] ) << 8 | imuData[8];
		uint16_t gz = (( uint16_t) imuData[11]) << 8 | imuData[10];

		angSum.x += uint2int_16bit(gx)*scaleFactor_ang;
		angSum.y += uint2int_16bit(gy)*scaleFactor_ang;
		angSum.z += uint2int_16bit(gz)*scaleFactor_ang;

		HAL_Delay(10);
	}

	angCalibration.x = angSum.x/CAL_AVG_TIME;
	angCalibration.y = angSum.y/CAL_AVG_TIME;
	angCalibration.z = angSum.z/CAL_AVG_TIME;
}

void QMI8658_StartReadImuData()
{
	HAL_StatusTypeDef status = QMI8658_ReadRegisters_IT(AX_L, imuData, 12);
	if (status != HAL_OK) deviceError = 1;
}

void QMI8658_HandleImuData()
{
	// merge lower 8 bits & upper 8 bits
	uint16_t ax = (( uint16_t) imuData[1]) << 8 | imuData[0];
	uint16_t ay = (( uint16_t) imuData[3]) << 8 | imuData[2];
	uint16_t az = (( uint16_t) imuData[5]) << 8 | imuData[4];

	uint16_t gx = (( uint16_t) imuData[7] ) << 8 | imuData[6];
	uint16_t gy = (( uint16_t) imuData[9] ) << 8 | imuData[8];
	uint16_t gz = (( uint16_t) imuData[11]) << 8 | imuData[10];

	// calculate acceleration & angularVelocity
	acceleration.x = uint2int_16bit(ax)*scaleFactor_acc;
	acceleration.y = uint2int_16bit(ay)*scaleFactor_acc;
	acceleration.z = uint2int_16bit(az)*scaleFactor_acc;

	angularVelocity.x = uint2int_16bit(gx)*scaleFactor_ang - angCalibration.x;
	angularVelocity.y = uint2int_16bit(gy)*scaleFactor_ang - angCalibration.y;
	angularVelocity.z = uint2int_16bit(gz)*scaleFactor_ang - angCalibration.z;
}


/* QMI8658 :
 |                                           _____
 |                                          |___  |
 |                                             / /
 |                                            / /
 |                                           / /__
 |                                          |_____|
 |
 |											   n
 |                                            MIL
 |                                          ,IMtIL
 |     __   __                                WM
 |     \ \ / /                                WM
 |      \ V /                                 WM
 |       | |                                  WM
 |       |_|                                  WM
 |                  MMMWIL_                   WM
 |                  M$CEbv                    WM
 |                  P  'TI#WC                 WW                                       __   __
 |                         .tB#n              WM                                       \ \ / /
 |                           .t$$n            WM                              7o.       \ V /
 |                               EbL          WM   ...::88,             _,,ansb0Md>     / Λ \
 |                                .2MB   inca MM 6686ZA6IUIIL     ..,;msanYCo6z6MP     /_/ \_\
 |                    ..:ivYCtzzAL  '76#L  87 MM 0PT^*'     .,oac6Q2Stzn^'      P
 |           .vUE$MMP  __  ..IoznStt   I#IL   MM    _.nn#$$QQbb'''   _,.
 |           .  .:    |__|  ..tXtCCCCX   Yi798AA2zCCY70RT''   _.,atroanva.
 |           ........        .XCCCXCCCXCL  7c;cvYY77P''    _...SoSoSzoA69S;L
 |            ,..........7onnCC7C7C7C7X7Cc   ia'    ..t1CCXC77vc;;ii::,,,,:v
 |               ..:......:v71nn7XYC7C7CCnic._   .YM;vii0i::::::,:,::::::i:Y
 |                  .,.;....;v7YI2U2I2zno11CC77YYcY;v;v;v;vvYvYvYvcvY;v;;iii
 |                      .......cAZU6IUIIz2zIoUo2z2oS1nXC7Yvvii:0VWP
 |                         .......$WWQQbbAUnCcvi:,,.
 |                            .....;:,.
 */
