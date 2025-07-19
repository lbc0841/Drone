#include "e104bt52.h"

uint8_t receiveData[30];

void E104BT52_Init()
{
    // Reset the module
    HAL_GPIO_WritePin(BT52_RST_GPIO_Port, BT52_RST_Pin, GPIO_PIN_SET);

    E104BT52_TranmdSetup();
    HAL_Delay(20);

    // Start UART DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


// AT+TRANMD=0 -> not Transparent transmission
// AT+TRANMD=1 -> Transparent transmission
// Sets the module to transparent transmission mode (AT+TRANMD=1)
void E104BT52_TranmdSetup()
{
    char command[11] = "AT+TRANMD=1";
    HAL_UART_Transmit(&huart2, (uint8_t*)command, 11, HAL_MAX_DELAY);
}


// UART Callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart2 && receiveData[0] == 0xAA)
    {
    	if (receiveData[1] == 0x01) E104BT52_ReceiveMotorSpeed();
    	else if (receiveData[1] == 0x05) E104BT52_ReceivePID();
    }

    // Restart DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(huart, receiveData, sizeof(receiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void E104BT52_ReceiveMotorSpeed()
{
    motorBaseSpeed.fl = receiveData[2] * 7;
    motorBaseSpeed.fr = receiveData[3] * 7;
    motorBaseSpeed.bl = receiveData[4] * 7;
    motorBaseSpeed.br = receiveData[5] * 7;
}


#define P_SCALER 0.06
#define I_SCALER 0.004
#define D_SCALER 0.04

void E104BT52_ReceivePID()
{
	pidGain.p = receiveData[2] * P_SCALER;
	pidGain.i = receiveData[3] * I_SCALER;
	pidGain.d = receiveData[4] * D_SCALER;
}


void E104BT52_TransmitInertiaData(Acceleration acc, AngularVelocity ang, Angle angle)
{
    uint8_t transmitData[19] = {
        0xAA, // Header
        0x02, // Data type
        int2uint_8bit((int)acc.x),
        int2uint_8bit((acc.x - (int)acc.x) * 10),
        int2uint_8bit((int)acc.y),
        int2uint_8bit((acc.y - (int)acc.y) * 10),
        int2uint_8bit((int)acc.z),
        int2uint_8bit((acc.z - (int)acc.z) * 10),
        int2uint_8bit((int)ang.x),
        int2uint_8bit((ang.x - (int)ang.x) * 10),
        int2uint_8bit((int)ang.y),
        int2uint_8bit((ang.y - (int)ang.y) * 10),
        int2uint_8bit((int)ang.z),
        int2uint_8bit((ang.z - (int)ang.z) * 10),
        int2uint_8bit((int)angle.pitch),
        int2uint_8bit((angle.pitch - (int)angle.pitch) * 10),
        int2uint_8bit((int)angle.roll),
        int2uint_8bit((angle.roll - (int)angle.roll) * 10),
        0x00 // Checksum (not implemented)
    };

    HAL_UART_Transmit(&huart2, transmitData, 19, HAL_MAX_DELAY);
}

