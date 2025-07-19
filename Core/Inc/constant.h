#ifndef INC_CONSTANT_H_
#define INC_CONSTANT_H_

// Struct -----------------------------------------------------------------------------------------------
typedef struct
{
	double x;
	double y;
	double z;
} Acceleration;

typedef struct
{
	double x;
	double y;
	double z;
} AngularVelocity;

typedef struct
{
	double pitch;
	double roll;
	double yaw;
} Angle;

typedef struct
{
	int fl;
	int fr;
	int bl;
	int br;
} MotorSpeed;

typedef struct
{
	double p;
	double i;
	double d;
} PidGain;

typedef struct
{
    Angle p;
	Angle i;
	Angle d;
} PidState;

typedef struct
{
	double pitch;
	double roll;
	double yaw;
} Pid;

#endif /* INC_CONSTANT_H_ */
