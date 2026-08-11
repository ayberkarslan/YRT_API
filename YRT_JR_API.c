/*
* @file YRT_JR_API.c
* @author Ali Okan Yeşilyurt
* @brief Implementation of the YRT JR API functions
* @version BETA
* @copyright Copyright (c) 2026
*/

#include "YRT_JR_DEF.h"
#include "YRT_JR_API.h"

/**
 * @brief Get the current time in milliseconds
 * @return Current time in milliseconds
 */
uint32_t YRT_GetTime_ms()
{
#if YRT_JR_IS_RTOS_ENABLED
    return osKernelGetTickCount();
#else
    return HAL_GetTick();
#endif
}


/**
 * @brief Delay function
 * @param ms Delay time in milliseconds
 */
void YRT_JR_Delay(uint32_t ms)
{
    #if YRT_JR_IS_RTOS_ENABLED
    osDelay(ms);
    #else
    HAL_Delay(ms);
    #endif
}

/**
 * @brief CAN BUS Functions
 */
YRT_CAN_Dev_t YRT_JR_CAN_Dev;
extern HAL_CAN_HandleTypeDef hcan1;
CAN_FilterTypeDef canFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

YRT_JR_Result_t YRT_JR_CAN_Init(YRT_CAN_Dev_t *can_dev)
{
    canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	canFilterConfig.FilterBank = 0;
	canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK)
	{
		return YRT_JR_CAN_ERROR;
	}

    if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return YRT_JR_CAN_ERROR;
    }

    return YRT_JR_OK;
}

/**
 * @brief CAN RX FIFO0 message pending callback
 * @param hcan Pointer to the CAN handle
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if (hcan->Instance == CAN1)
    {
        CAN_Msg_t newMsg;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &newMsg.header, (uint8_t*)newMsg.data);
        osMessageQueuePut(canRxQueueHandle, &newMsg, 0, 0);
    }
}

/** 
 * @brief Initialize GPIO
 * @param gpio GPIO structure
 * @param port GPIO port
 * @param pin GPIO pin
 * @param mode GPIO mode
 * @return Result of the initialization
 */
YRT_JR_Result_t YRT_JR_GPIO_Init(YRT_JR_GPIO *gpio, GPIO_TypeDef *port, uint16_t pin, YRT_JR_OP_MODE mode)
{
    gpio->port = port;
    gpio->pin = pin;
    gpio->mode = mode;
    gpio->state = YRT_JR_OFF;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = (mode == YRT_JR_MODE_ACTIVE) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    return YRT_JR_OK;
}

/**
 * @brief Set the state of a GPIO pin
 * @param gpio GPIO structure
 * @param state Desired state
 */
void YRT_JR_GPIO_SetState(YRT_JR_GPIO *gpio, YRT_JR_DEV_STATE state)
{
    gpio->state = state;
    if (state == YRT_JR_ON)
    {
        HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_SET);
    }
    else if (state == YRT_JR_OFF)
    {
        HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_RESET);
    }
}

/** 
 * @brief Toggle the state of a GPIO pin
 * @param gpio GPIO structure
 */
void YRT_JR_GPIO_Toggle(YRT_JR_GPIO *gpio) {
    if (gpio == NULL || gpio->port == NULL) return;
    HAL_GPIO_TogglePin(gpio->port, gpio->pin);
}

/** 
 * @brief Send data over serial
 * @param dev Serial device structure
 * @param data Data to send
 * @param len Length of data to send
 * @return Result of the operation
 */
YRT_JR_Result_t YRT_JR_Serial_Send(YRT_JR_SERIAL_DEV *dev, uint8_t *data, uint16_t len) {
    if (dev == NULL || data == NULL || len == 0) return YRT_ERROR;

    switch (dev->protocol) {
        case YRT_SERIAL_UART:
            HAL_UART_Transmit(dev->huart, data, len, HAL_MAX_DELAY);
            break;
            
        case YRT_SERIAL_USB:
            CDC_Transmit_FS(data, len);
            break;

        case YRT_SERIAL_CAN:
#if YRT_JR_IS_CAN_ENABLED
            {  
        uint16_t sent_bytes = 0;
        TxHeader.StdId = dev->target_id;
        TxHeader.ExtId = 0;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.RTR = CAN_RTR_DATA;

        while (sent_bytes < len) {
            uint8_t dlc = (len - sent_bytes > 8) ? 8 : (len - sent_bytes);
            TxHeader.DLC = dlc;         
            HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &data[sent_bytes], &TxMailbox);
            sent_bytes += dlc;
            }
                dev->bytesSent += len;
            }
#else
            return YRT_ERROR; // CAN makrosu kapalıysa hata dön
#endif
            break;
    }
    
    return YRT_OK;
}

/** 
 * @brief Receive data over serial
 * @param dev Serial device structure
 * @param data Buffer to store received data
 * @param len Length of data to receive
 * @return Result of the operation
 */
YRT_JR_Result_t YRT_JR_Serial_Receive(YRT_JR_SERIAL_DEV *dev, uint8_t *data, uint16_t len) {
    if (dev == NULL || data == NULL || len == 0) return YRT_ERROR;

    switch (dev->protocol) {
        case YRT_SERIAL_UART:
            HAL_UART_Receive(dev->huart, data, len, HAL_MAX_DELAY);
            break;
            
        case YRT_SERIAL_USB:
            // USB receive implementation here
            break;

        case YRT_SERIAL_CAN:
#if YRT_JR_IS_CAN_ENABLED
            {
                uint16_t received_bytes = 0;
                while (received_bytes < len) {
                    CAN_Msg_t msg;
                    osMessageQueueGet(canRxQueueHandle, &msg, NULL, osWaitForever);
                    uint8_t dlc = (len - received_bytes > 8) ? 8 : (len - received_bytes);
                    memcpy(data + received_bytes, msg.data, dlc);
                    received_bytes += dlc;
                }
                dev->bytesReceived += len;
            }
#else 
            return YRT_ERROR; // CAN makrosu kapalıysa hata dön
#endif
            break;
    }
    
    return YRT_OK;
}

/** 
 * @brief Calculate CRC
 * @param buf Buffer containing data
 * @param len Length of data
 * @return Calculated CRC
 */
#if YRT_JR_IS_CRC_ENABLED   
uint16_t YRT_CRCCalculator(uint8_t  *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    char i = 0;

    while (len--)
    {
        crc ^= (*buf++);

        for (i = 0; i < 8; i++)
        {
            if (crc & 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
        
    }
    return crc;
}
#endif /* YRT_JR_IS_CRC_ENABLED */

