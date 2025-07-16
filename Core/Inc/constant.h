#ifndef INC_CONSTANT_H_
#define INC_CONSTANT_H_

// Struct -----------------------------------------------------------------------------------------------
typedef struct
{
	float x;
	float y;
	float z;
} Acceleration;

typedef struct
{
	float x;
	float y;
	float z;
} AngularVelocity;

typedef struct
{
	float pitch;
	float roll;
	float yaw;
} Angle;

typedef struct
{
	float pitch;
	float roll;
	float yaw;
} Pid;

typedef struct
{
	int fl;
	int fr;
	int bl;
	int br;
} MotorSpeed;

typedef struct
{
	float p;
	float i;
	float d;
} PidGain;

#endif /* INC_CONSTANT_H_ */
