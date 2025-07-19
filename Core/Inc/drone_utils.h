#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include <math.h>
#include "constant.h"

extern volatile int deviceError;

int uint2int_16bit(uint16_t input);
uint16_t int2uint_16bit(int input);

int uint2int_8bit(uint8_t input);
uint8_t int2uint_8bit(int input);

int IsDeviceError(Angle currAngle, Angle targetAngle, PidGain pidGain, PidState pidState);

void inertiaData2Angle_noFilter(Acceleration acc, AngularVelocity ang, Angle* angle);
void inertiaData2Angle_balanceFilter(Acceleration acc, AngularVelocity ang, Angle* currentAngle);

void limitMotorSpeed(MotorSpeed* speed);

int enablePid(MotorSpeed speed);
void singlePidController(AngularVelocity ang, Angle currentAngle, Angle targetAngle, PidGain pidGain, PidState* pidState,
                         MotorSpeed baseSpeed, MotorSpeed* speed);

#endif /* INC_UTILS_H_ */
