#ifndef INC_QMI8658_H_
#define INC_QMI8658_H_

#include "drone_utils.h"
#include "i2c.h"
#include "constant.h"

extern Acceleration acceleration;
extern AngularVelocity angularVelocity;

void QMI8658_Init();
void QMI8658_Calibration(int avgTime);

void QMI8658_GetInertiaData(Acceleration* acc, AngularVelocity* ang);
void QMI8658_GetInertiaData_NoCalibration(Acceleration* acc, AngularVelocity* ang);


#endif /* INC_QMI8658_H_ */
