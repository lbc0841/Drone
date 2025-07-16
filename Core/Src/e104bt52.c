#include "e104bt52.h"

// Buffer for receiving data via UART
uint8_t receiveData[30];


// E104BT52_Init -----------------------------------------------------------------------------------------

// Initializes the E104-BT52 Bluetooth module
void E104BT52_Init()
{
    // Reset the module
    HAL_GPIO_WritePin(BT52_RST_GPIO_Port, BT52_RST_Pin, GPIO_PIN_SET);

    // Configure transmission mode
    E104BT52_TranmdSetup();
    HAL_Delay(20);

    // Start UART DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveData, sizeof(receiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


// E104BT52_TranmdSetup ----------------------------------------------------------------------------------

// AT+TRANMD=0 -> not Transparent transmission
// AT+TRANMD=1 -> Transparent transmission
// Sets the module to transparent transmission mode (AT+TRANMD=1)
void E104BT52_TranmdSetup()
{
    char command[11] = "AT+TRANMD=1";
    HAL_UART_Transmit(&huart2, (uint8_t*)command, 11, HAL_MAX_DELAY);
}


// UART Callback -----------------------------------------------------------------------------------------
// UART receive callback for handling incoming data
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Validate UART and header
    if (huart != &huart2 || receiveData[0] != 0xAA)
    {
        // Restart DMA reception
        HAL_UARTEx_ReceiveToIdle_DMA(huart, receiveData, sizeof(receiveData));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
        return;
    }

    // Process motor speed command
    if (receiveData[1] == 0x01)
    {
        E104BT52_ReceiveMotorSpeed();
    }
    // Process motor speed command
    else if (receiveData[1] == 0x05)
    {
        E104BT52_ReceivePID();
    }

    // Restart DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(huart, receiveData, sizeof(receiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

// E104BT52_ReceiveMotorSpeed ----------------------------------------------------------------------------

// Processes received motor speed data
void E104BT52_ReceiveMotorSpeed()
{
    // Scale received values to motor PWM range
    motorBaseSpeed.fl = receiveData[2] * 7;
    motorBaseSpeed.fr = receiveData[3] * 7;
    motorBaseSpeed.bl = receiveData[4] * 7;
    motorBaseSpeed.br = receiveData[5] * 7;
}


// E104BT52_ReceivePID ----------------------------------------------------------------------------

// Processes received motor speed data
void E104BT52_ReceivePID()
{
	pidK.p = receiveData[2] * 0.04;
	pidK.i = receiveData[3] * 0.003;
	pidK.d = receiveData[4] * 0.02;
}


// E104BT52_TransmitInertia ------------------------------------------------------------------------------
// Transmits inertia data (acceleration, angular velocity, angles) via UART
void E104BT52_TransmitInertiaData(Acceleration acc, AngularVelocity ang, Angle angle)
{
    uint8_t transmitData[19] = {
        0xAA, // Header
        0x02, // Data type
        int2uint_8bit((int)acc.x),                // Acc X integer part
        int2uint_8bit((acc.x - (int)acc.x) * 10), // Acc X fractional part
        int2uint_8bit((int)acc.y),                // Acc Y integer part
        int2uint_8bit((acc.y - (int)acc.y) * 10), // Acc Y fractional part
        int2uint_8bit((int)acc.z),                // Acc Z integer part
        int2uint_8bit((acc.z - (int)acc.z) * 10), // Acc Z fractional part
        int2uint_8bit((int)ang.x),                // Gyro X integer part
        int2uint_8bit((ang.x - (int)ang.x) * 10), // Gyro X fractional part
        int2uint_8bit((int)ang.y),                // Gyro Y integer part
        int2uint_8bit((ang.y - (int)ang.y) * 10), // Gyro Y fractional part
        int2uint_8bit((int)ang.z),                // Gyro Z integer part
        int2uint_8bit((ang.z - (int)ang.z) * 10), // Gyro Z fractional part
        int2uint_8bit((int)angle.pitch),          // Pitch integer part
        int2uint_8bit((angle.pitch - (int)angle.pitch) * 10), // Pitch fractional
        int2uint_8bit((int)angle.roll),           // Roll integer part
        int2uint_8bit((angle.roll - (int)angle.roll) * 10),   // Roll fractional
        0x00 // Checksum (not implemented)
    };

    HAL_UART_Transmit(&huart2, transmitData, 19, HAL_MAX_DELAY);
}

