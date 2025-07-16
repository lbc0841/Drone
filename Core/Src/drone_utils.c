#include "drone_utils.h"

// PWM
#define COUNTER_PERIOD 1024

Angle p, i, d;



// Conversion Functions ---------------------------------------------------------------------------------

// Converts 16-bit unsigned integer to signed integer
// Handles two's complement for negative values
int uint2int_16bit(uint16_t input)
{
	int output = input;

	if(input > 32767)
	{
		output = - (input ^ 0xFFFF) -1;
	}

	return output;
}

// Converts signed integer to 16-bit unsigned integer
uint16_t int2uint_16bit(int input)
{
	return (uint16_t)input;
}

// Converts 8-bit unsigned integer to signed integer
// Handles two's complement for negative values
int uint2int_8bit(uint8_t input)
{
	int output = input;

	if(input > 127)
	{
		output = - (input ^ 0xFF) -1;
	}

	return output;
}

// Converts signed integer to 8-bit unsigned integer
uint8_t int2uint_8bit(int input)
{
	return (uint8_t)input;
}

// Inertia Data to Angle Conversion ---------------------------------------------------------------------

// Calculates pitch and roll angles from acceleration data without filtering
// Uses atan to compute angles based on gravitational components
void inertiaData2Angle_noFilter(Acceleration acc, AngularVelocity ang, Angle* angle)
{
    angle->pitch = atan(acc.x / sqrt(pow(acc.y, 2) + pow(acc.z, 2)));
    angle->roll  = atan(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2)));
}

// Balance filter combining accelerometer and gyroscope data
// Uses complementary filter approach with weighting factor k
void inertiaData2Angle_balanceFilter(Acceleration acc, AngularVelocity ang,
                                    Angle* lastAngle, Angle* currentAngle)
{
    Angle angle_acc, angle_gyro;

    // Calculate angles from accelerometer data (in degrees)
    angle_acc.pitch  = atanf(acc.x / sqrt(pow(acc.y, 2) + pow(acc.z, 2))) * 180 / M_PI;
    angle_acc.roll  = atanf(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2))) * 180 / M_PI;

    // Time step for gyroscope integration
    float dt = 0.03; // 30ms loop time
    // Integrate gyroscope data to get angle
    angle_gyro.pitch = lastAngle->pitch + ang.y * dt;
    angle_gyro.roll  = lastAngle->roll + ang.x * dt;

    // Complementary filter with weighting factor
    float k = 0.3; // Accelerometer weight (30%)
    currentAngle->pitch = k * angle_acc.pitch + (1 - k) * angle_gyro.pitch;
    currentAngle->roll  = k * angle_acc.roll + (1 - k) * angle_gyro.roll;

    // Update last angle for next iteration
    lastAngle->pitch = currentAngle->pitch;
    lastAngle->roll  = currentAngle->roll;
}

// Kalman Filter Implementation --------------------------------------------------------------------------

// Structure to hold Kalman filter state
typedef struct {
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Estimated angle
    float p; // Estimation error covariance
    float k; // Kalman gain
} KalmanFilter;

// Initialize Kalman filter with default parameters
void kalmanFilterInit(KalmanFilter* filter)
{
    filter->q = 0.0001; // Process noise
    filter->r = 0.03;   // Measurement noise
    filter->x = 0;      // Initial angle
    filter->p = 1;      // Initial error covariance
    filter->k = 0;      // Initial Kalman gain
}

// Kalman filter update for one axis
void kalmanFilterUpdate(KalmanFilter* filter, float measurement, float rate, float dt)
{
    // Prediction step
    filter->x = filter->x + dt * rate; // Predict angle using gyroscope
    filter->p = filter->p + filter->q; // Update error covariance

    // Update step
    filter->k = filter->p / (filter->p + filter->r); // Calculate Kalman gain
    filter->x = filter->x + filter->k * (measurement - filter->x); // Update estimate
    filter->p = (1 - filter->k) * filter->p; // Update error covariance
}

// Kalman filter for angle calculation
void inertiaData2Angle_kalmanFilter(Acceleration acc, AngularVelocity ang,
                                   Angle* lastAngle, Angle* currentAngle)
{
    static KalmanFilter pitchFilter, rollFilter;
    static bool initialized = false;

    // Initialize filters on first call
    if (!initialized)
    {
        kalmanFilterInit(&pitchFilter);
        kalmanFilterInit(&rollFilter);
        initialized = true;
    }

    // Calculate angles from accelerometer (in degrees)
    float accPitch = atanf(acc.x / sqrt(pow(acc.y, 2) + pow(acc.z, 2))) * 180 / M_PI;
    float accRoll = atanf(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2))) * 180 / M_PI;

    // Time step
    float dt = 0.03; // 30ms loop time

    // Update Kalman filters
    kalmanFilterUpdate(&pitchFilter, accPitch, ang.x, dt);
    kalmanFilterUpdate(&rollFilter, accRoll, ang.y, dt);

    // Set current angles
    currentAngle->pitch = pitchFilter.x;
    currentAngle->roll = rollFilter.x;

    // Update last angle
    lastAngle->pitch = currentAngle->pitch;
    lastAngle->roll = currentAngle->roll;
}


// Motor Speed Limiting ----------------------------------------------------------------------------------

// Limits motor speeds to valid PWM range [0, COUNTER_PERIOD-1]
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

// PID Controller ---------------------------------------------------------------------------------------

// Checks if PID should be enabled based on minimum motor speed threshold
bool enablePid(MotorSpeed speed)
{
//    return speed.fr > 256 &&
//           speed.fl > 256 &&
//           speed.br > 256 &&
//           speed.bl > 256;

    return true;
}


// Single PID controller for drone stabilization
float i_max = 160;
void singlePidController(Acceleration acc, AngularVelocity ang,
                         Angle currentAngle, Angle targetAngle, PidK k,
                         MotorSpeed baseSpeed, MotorSpeed* speed)
{
    Angle error;

    // Calculate angle errors
    error.pitch = targetAngle.pitch - currentAngle.pitch;
    error.roll  = targetAngle.roll  - currentAngle.roll;

    // Update PID terms
    p.pitch =  error.pitch;
    i.pitch += error.pitch;
    d.pitch =  0 - ang.y; // Gyro rate as derivative term
    if (i.pitch > i_max)  i.pitch = i_max;
    if (i.pitch < -i_max) i.pitch = -i_max;

    p.roll =  error.roll;
    i.roll += error.roll;
    d.roll =  0 - ang.x; // Gyro rate as derivative term
    if (i.roll > i_max)  i.roll = i_max;
    if (i.roll < -i_max) i.roll = -i_max;

    // Calculate PID outputs
    Pid pid;
//    pid.pitch = -(k.p * p.pitch + k.i * i.pitch + k.d * d.pitch);

    pid.roll  = -(k.p * p.roll  + k.i * i.roll  + k.d * d.roll);
    pid.pitch = 0;
    pid.yaw = 0; // Yaw control not implemented

    // Apply PID corrections
    if (enablePid(baseSpeed))
    {
//        speed->fl = baseSpeed.fl + pid.pitch - pid.roll + pid.yaw;
//        speed->fr = baseSpeed.fr + pid.pitch + pid.roll + pid.yaw;
//        speed->bl = baseSpeed.bl - pid.pitch - pid.roll + pid.yaw;
//        speed->br = baseSpeed.br - pid.pitch + pid.roll + pid.yaw;

        speed->fl = baseSpeed.fl - pid.roll;
        speed->fr = baseSpeed.fr + pid.roll;
        speed->bl = baseSpeed.bl - pid.roll;
        speed->br = baseSpeed.br + pid.roll;
    }
    else
    {
        // Maintain base speeds if PID not enabled
        speed->fl = baseSpeed.fl;
        speed->fr = baseSpeed.fr;
        speed->bl = baseSpeed.bl;
        speed->br = baseSpeed.br;
    }
}
