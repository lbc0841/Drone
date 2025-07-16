#ifndef INC_E104BT52_H_
#define INC_E104BT52_H_

#include "drone_utils.h"
#include "constant.h"
#include "gpio.h"
#include "usart.h"
#include "motor.h"

extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern MotorSpeed motorBaseSpeed;
extern PidGain pidGain;

void E104BT52_Init();
void E104BT52_TranmdSetup();

void E104BT52_ReceiveMotorSpeed();
void E104BT52_ReceivePID();

void E104BT52_TransmitInertiaData(Acceleration acc, AngularVelocity ang, Angle angle);

#endif /* INC_E104BT52_H_ */
