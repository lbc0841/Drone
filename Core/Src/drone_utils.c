#include "drone_utils.h"

// PWM
#define COUNTER_PERIOD 1024


// Conversion Functions
int uint2int_16bit(uint16_t input)
{
	int output = input;

	if(input > 32767)
	{
		output = - (input ^ 0xFFFF) -1;
	}

	return output;
}

uint16_t int2uint_16bit(int input)
{
	return (uint16_t)input;
}

int uint2int_8bit(uint8_t input)
{
	int output = input;

	if(input > 127)
	{
		output = - (input ^ 0xFF) -1;
	}

	return output;
}

uint8_t int2uint_8bit(int input)
{
	return (uint8_t)input;
}

int IsDeviceError(Angle currAngle, Angle targetAngle, PidGain pidGain, PidState pidState)
{
	if (isnan(currAngle.pitch) || isnan(currAngle.roll) || isnan(currAngle.yaw) ||
		isnan(targetAngle.pitch) || isnan(targetAngle.roll) || isnan(targetAngle.yaw) ||
		isnan(pidGain.p) || isnan(pidGain.i) || isnan(pidGain.d) ||
		isnan(pidState.p.pitch) || isnan(pidState.p.roll) || isnan(pidState.p.yaw) ||
		isnan(pidState.i.pitch) || isnan(pidState.i.roll) || isnan(pidState.i.yaw) ||
		isnan(pidState.d.pitch) || isnan(pidState.d.roll) || isnan(pidState.d.yaw))
	{
	    return 1;
	}

	return 0;
}


// Inertia Data to Angle
void inertiaData2Angle_noFilter(Acceleration acc, AngularVelocity ang, Angle* angle)
{
    angle->pitch = atan(acc.x / sqrt(pow(acc.y, 2) + pow(acc.z, 2)));
    angle->roll  = atan(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2)));
}

void inertiaData2Angle_balanceFilter(Acceleration acc, AngularVelocity ang, Angle* currentAngle)
{
    Angle angle_acc = {0};
    Angle angle_gyro = {0};

    double sqrtYZ = sqrt(pow(acc.y, 2) + pow(acc.z, 2));
    double sqrtXZ = sqrt(pow(acc.x, 2) + pow(acc.z, 2));
    angle_acc.pitch = (sqrtYZ!=0) ? atanf(acc.x / sqrtYZ) * 180 / M_PI : 0;
    angle_acc.roll =  (sqrtXZ!=0) ? atanf(acc.y / sqrtXZ) * 180 / M_PI : 0;

    double dt = 0.01;
    angle_gyro.pitch = currentAngle->pitch + ang.y * dt;
    angle_gyro.roll  = currentAngle->roll + ang.x * dt;

    double k = 0.3;
    currentAngle->pitch = k * angle_acc.pitch + (1 - k) * angle_gyro.pitch;
    currentAngle->roll  = k * angle_acc.roll + (1 - k) * angle_gyro.roll;
}

void limitMotorSpeed(MotorSpeed* speed)
{
    // Front Left
    if (speed->fl > COUNTER_PERIOD) speed->fl = COUNTER_PERIOD - 1;
    else if (speed->fl < 0) speed->fl = 0;

    // Front Right
    if (speed->fr > COUNTER_PERIOD) speed->fr = COUNTER_PERIOD - 1;
    else if (speed->fr < 0) speed->fr = 0;

    // Back Left
    if (speed->bl > COUNTER_PERIOD) speed->bl = COUNTER_PERIOD - 1;
    else if (speed->bl < 0) speed->bl = 0;

    // Back Right
    if (speed->br > COUNTER_PERIOD) speed->br = COUNTER_PERIOD - 1;
    else if (speed->br < 0) speed->br = 0;
}

// PID Controller
int enablePid(MotorSpeed speed)
{
    if(speed.fr > 100 && speed.fl > 100 && speed.br > 100 && speed.bl > 100) return 1;
    else return 0;
}

// Single PID controller for drone stabilization
float i_max = 160;
void singlePidController(AngularVelocity ang, Angle currentAngle, Angle targetAngle, PidGain pidGain, PidState* pidState,
                         MotorSpeed baseSpeed, MotorSpeed* speed)
{
    Angle error = {0};
    error.pitch = targetAngle.pitch - currentAngle.pitch;
    error.roll  = targetAngle.roll  - currentAngle.roll;

//    p.pitch =  error.pitch;
//    i.pitch += error.pitch;
//    d.pitch =  0 - ang.y;
//    if (i.pitch > i_max)  i.pitch = i_max;
//    if (i.pitch < -i_max) i.pitch = -i_max;

    pidState->p.roll =  error.roll;
    pidState->i.roll += error.roll;
    pidState->d.roll =  0 - ang.x;
    if (pidState->i.roll > i_max)  pidState->i.roll = i_max;
    if (pidState->i.roll < -i_max) pidState->i.roll = -i_max;

    // calculate PID
    Pid pid = {0};
//    pid.pitch = -(pidGain.p * p.pitch + pidGain.i * i.pitch + pidGain.d * d.pitch);
//    pid.roll = -(pidGain.p * pidState->p.roll  + pidGain.i * pidState->i.roll  + pidGain.d * pidState->d.roll);
    pid.roll = 0 * pidState->p.roll;
    // Apply PID corrections
//    speed->fl = baseSpeed.fl + pid.pitch - pid.roll - pid.yaw;
//    speed->fr = baseSpeed.fr + pid.pitch + pid.roll + pid.yaw;
//    speed->bl = baseSpeed.bl - pid.pitch - pid.roll + pid.yaw;
//    speed->br = baseSpeed.br - pid.pitch + pid.roll - pid.yaw;

    speed->fl = baseSpeed.fl + pid.roll;
    speed->fr = baseSpeed.fr;
    speed->bl = baseSpeed.bl;
    speed->br = baseSpeed.br;
}
