#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "constant.h"

int uint2int_16bit(uint16_t input);
uint16_t int2uint_16bit(int input);

int uint2int_8bit(uint8_t input);
uint8_t int2uint_8bit(int input);

void inertiaData2Angle_noFilter(Acceleration acc, AngularVelocity ang, Angle* angle);
void inertiaData2Angle_balanceFilter(Acceleration acc, AngularVelocity ang,
											Angle* lastAngle,Angle* currentAngle);

void limitMotorSpeed(MotorSpeed* speed);

bool enablePid(MotorSpeed speed);
void singlePidController(Acceleration acc, AngularVelocity ang,
		Angle currentAngle, Angle targetAngle, PidK k, MotorSpeed baseSpeed, MotorSpeed* speed);


#endif /* INC_UTILS_H_ */
