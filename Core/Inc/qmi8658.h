#ifndef INC_QMI8658_H_
#define INC_QMI8658_H_

#include "drone_utils.h"
#include "i2c.h"
#include "constant.h"

extern Acceleration acceleration;
extern AngularVelocity angularVelocity;

extern volatile int deviceError;

HAL_I2C_StateTypeDef QMI8658_ReadRegisters(uint8_t memAddress, uint8_t* data, int size);
HAL_I2C_StateTypeDef QMI8658_WriteRegisters(uint8_t memAddress, uint8_t data);
HAL_I2C_StateTypeDef QMI8658_ReadRegisters_IT(uint8_t memAddress, uint8_t* data, int size);

void QMI8658_Init();
void QMI8658_Calibration();
void QMI8658_StartReadImuData();
void QMI8658_HandleImuData();


#endif /* INC_QMI8658_H_ */
