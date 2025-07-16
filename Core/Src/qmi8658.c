#include "qmi8658.h"

// SDO/SA0 = VDD -> QMI8658_ADDRESS = 0x6A
#define QMI8658_ADDRESS 0x6A << 1


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

// Calibration ------------------------------------------------------------------------------------------
#define AVERAGE_TIME 20


// Calibration Definitions ------------------------------------------------------------------------------
#define AVERAGE_TIME 20

// Calibration variables
AngularVelocity angCalibrationValue;
double scaleFactor_acc = 0;
double scaleFactor_ang = 0;


// I2C Callback -----------------------------------------------------------------------------------------
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c != &hi2c1) return;
//	QMI8658_HandleInertiaData(inertiaData, &acceleration, &angularVelocity);
}


// Read/Write Registers ---------------------------------------------------------------------------------

// Polling
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

// DMA
HAL_I2C_StateTypeDef QMI8658_ReadRegistersDMA(uint8_t memAddress, uint8_t* data, int size)
{
	return HAL_I2C_Mem_Read_DMA(
			&hi2c1, QMI8658_ADDRESS, memAddress, I2C_MEMADD_SIZE_8BIT, data, size);
}
HAL_I2C_StateTypeDef QMI8658_WriteRegistersDMA(uint8_t memAddress, uint8_t data)
{
	uint8_t size = 1;
	return HAL_I2C_Mem_Write_DMA(
			&hi2c1, QMI8658_ADDRESS, memAddress, I2C_MEMADD_SIZE_8BIT, &data, size);
}

// QMI8658_Init -----------------------------------------------------------------------------------------
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


//	QMI8658_WriteRegisters(CTRL8, 0x00);
//	QMI8658_WriteRegisters(CTRL9, 0x00);


	HAL_Delay(30);
	QMI8658_Calibration(AVERAGE_TIME);
}


void QMI8658_Calibration(int avgTime)
{
	Acceleration accTemp;
	AngularVelocity angTemp;

	Acceleration accSum;
	AngularVelocity angSum;

	for(int i=0; i<AVERAGE_TIME; i++)
	{
		QMI8658_GetInertiaData_NoCalibration(&accTemp, &angTemp);


		accSum.x += accTemp.x;
		accSum.y += accTemp.y;
		accSum.z += (accTemp.z-9.81);

		angSum.x += angTemp.x;
		angSum.y += angTemp.y;
		angSum.z += angTemp.z;

		HAL_Delay(10);
	}

	angCalibrationValue.x = angSum.x/AVERAGE_TIME;
	angCalibrationValue.y = angSum.y/AVERAGE_TIME;
	angCalibrationValue.z = angSum.z/AVERAGE_TIME;
}


void QMI8658_GetInertiaData(Acceleration* acc, AngularVelocity* ang)
{
	uint8_t data[12];
	QMI8658_ReadRegisters(AX_L, data, 12);

	// Merge lower 8 bits & upper 8 bits
	uint16_t ax = (( uint16_t) data[1]) << 8 | data[0];
	uint16_t ay = (( uint16_t) data[3]) << 8 | data[2];
	uint16_t az = (( uint16_t) data[5]) << 8 | data[4];

	uint16_t gx = (( uint16_t) data[7] ) << 8 | data[6];
	uint16_t gy = (( uint16_t) data[9] ) << 8 | data[8];
	uint16_t gz = (( uint16_t) data[11]) << 8 | data[10];


	acc->x = uint2int_16bit(ax)*scaleFactor_acc;
	acc->y = uint2int_16bit(ay)*scaleFactor_acc;
	acc->z = uint2int_16bit(az)*scaleFactor_acc;

	ang->x = uint2int_16bit(gx)*scaleFactor_ang - angCalibrationValue.x;
	ang->y = uint2int_16bit(gy)*scaleFactor_ang - angCalibrationValue.y;
	ang->z = uint2int_16bit(gz)*scaleFactor_ang - angCalibrationValue.z;

}

void QMI8658_GetInertiaData_NoCalibration(Acceleration* acc, AngularVelocity* ang)
{
	uint8_t data[12];
	QMI8658_ReadRegisters(AX_L, data, 12);

	// Merge lower 8 bits & upper 8 bits
	uint16_t ax = (( uint16_t) data[1]) << 8 | data[0];
	uint16_t ay = (( uint16_t) data[3]) << 8 | data[2];
	uint16_t az = (( uint16_t) data[5]) << 8 | data[4];

	uint16_t gx = (( uint16_t) data[7] ) << 8 | data[6];
	uint16_t gy = (( uint16_t) data[9] ) << 8 | data[8];
	uint16_t gz = (( uint16_t) data[11]) << 8 | data[10];

	acc->x = uint2int_16bit(ax)*scaleFactor_acc;
	acc->y = uint2int_16bit(ay)*scaleFactor_acc;
	acc->z = uint2int_16bit(az)*scaleFactor_acc;

	ang->x = uint2int_16bit(gx)*scaleFactor_ang;
	ang->y = uint2int_16bit(gy)*scaleFactor_ang;
	ang->z = uint2int_16bit(gz)*scaleFactor_ang;
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
