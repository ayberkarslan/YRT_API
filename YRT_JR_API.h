/**
 * @file YRT_JR_API.h
 * @author Ali Okan Yeşilyurt
 * @brief Header file for the YRT JR API functions
 * @version BETA
 * @copyright Copyright (c) 2026
 */

#ifndef YRT_JR_API_H
#define YRT_JR_API_H

#include "YRT_JR_DEF.h"
#include "math.h"


#define YRT_JR_ENABLED 1
#define YRT_JR_DISABLED 0

#define YRT_JR_MCU_STM32F7 YRT_JR_DISABLED
#define YRT_JR_MCU_STM32F4 YRT_JR_ENABLED

#define YRT_JR_IS_IMU_ENABLED YRT_JR_ENABLED
#define YRT_JR_IS_BARO_ENABLED YRT_JR_ENABLED
#define YRT_JR_IS_GPS_ENABLED YRT_JR_DISABLED
#define YRT_JR_IS_SD_ENABLED YRT_JR_ENABLED
#define YRT_JR_IS_CAN_ENABLED YRT_JR_ENABLED
#define YRT_JR_IS_RFM98_ENABLED YRT_JR_DISABLED
#define YRT_JR_IS_RFMT_ENABLED YRT_JR_DISABLED
#define YRT_JR_IS_CAN_ENABLED YRT_JR_ENABLED
#define YRT_JR_IS_RTOS_ENABLED YRT_JR_ENABLED
#define YRT_JR_IS_CRC_ENABLED YRT_JR_ENABLED
#if YRT_JR_MCU_STM32F7
    #include "stm32f7xx_hal.h" 
#elif YRT_JR_MCU_STM32F4
    #include "stm32f4xx_hal.h"
#endif

void YRT_JR_Delay(uint32_t ms);

/**
 * @brief CAN BUS
 */
#if YRT_JR_IS_CAN_ENABLED
#include "can.h"
#include "stm32f4xx_hal_can.h"

extern CAN_TxHeaderTypedef YRT_JR_CAN_TxHeader;
extern CAN_RxHeaderTypeDef YRT_JR_CAN_RxHeader;
// TODO: Burası ne alaka anlamadım kontrol edeceğim.
/**
 * @brief CAN device structure
 */
typedef enum {
    YRT_JR_CAN_FIFO0 = 0,
    YRT_JR_CAN_FIFO1 = 1
} YRT_JR_CAN_FIFO_NUM_t;

typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_FilterTypeDef *CAN_FilterConfig;
    CAN_TxHeaderTypeDef *CAN_TxHeader;
    CAN_RxHeaderTypeDef *CAN_RxHeader;
    YRT_JR_CAN_FIFO_NUM_t fifo_num;
} YRT_JR_CAN_DEV_t;

YRT_JR_Result_t YRT_JR_CAN_Init(YRT_JR_CAN_DEV_t *can_dev);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
#endif

/**
 * @brief GPIO Definition
 */

typedef enum {
    YRT_JR_MODE_ACTIVE,
    YRT_JR_MODE_INACTIVE
}YRT_JR_OP_MODE;

typedef enum {
    YRT_JR_OFF,
    YRT_JR_ON,
    YRT_JR_ERROR,
    YRT_JR_BLINK
}YRT_JR_DEV_STATE;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    YRT_JR_OP_MODE mode;
    YRT_JR_DEV_STATE state;
} YRT_JR_GPIO;

YRT_JR_Result_t YRT_JR_GPIO_Init(YRT_JR_GPIO *gpio, GPIO_TypeDef *port, uint16_t pin, YRT_JR_OP_MODE mode);
void YRT_JR_GPIO_SetState(YRT_JR_GPIO *gpio, YRT_JR_DEV_STATE state);
void YRT_JR_GPIO_Toggle(YRT_JR_GPIO *gpio);

/**
 * @brief LED-BUZZER 
 */

typedef struct {
    YRT_JR_GPIO USR_LED1;
    YRT_JR_GPIO USR_LED2;
    YRT_JR_GPIO BUZZER;
}YRT_JR_LED_BUZZER_DEV;
// TODO: Morse ve pyro kodu kısmını anlarsam buraya entegre edeceğim.

/** 
 * @brief Serial Protocol Buffer and Functions
 */

typedef enum {
    YRT_SERIAL_UART,
    YRT_SERIAL_USB,
    YRT_SERIAL_CAN,
}YRT_JR_SERIAL_PROTOCOL;

typedef struct {
    YRT_JR_SERIAL_PROTOCOL protocol;
    uint8_t *buffer;
    uint16_t bufferSize;
    uint16_t bytesSent;
    uint8_t bytesReceived;
    uint32_t target_id;
}YRT_JR_SERIAL_DEV;

/**
 * @brief Serial Send and Receive Functions
 */
void YRT_JR_CreateSerialTxBuffer(YRT_JR_SERIAL_DEV *dev, YRT_JR_SERIAL_PROTOCOL protocol, uint8_t *buffer, uint16_t bufferSize);
YRT_JR_Result_t YRT_JR_Serial_Send(YRT_JR_SERIAL_DEV *dev, YRT_JR_SERIAL_PROTOCOL protocol, uint8_t *buffer, uint16_t bufferSize);
YRT_JR_Result_t YRT_JR_Serial_Receive(YRT_JR_SERIAL_DEV *dev, uint8_t *data, uint16_t len);

#if YRT_JR_IS_IMU_ENABLED

/**
 * @brief IMU enums
 */

typedef enum 
{
    // config
    YRT_JR_ACC_RANGE_2G,
    YRT_JR_ACC_RANGE_3G,
    YRT_JR_ACC_RANGE_4G,
    YRT_JR_ACC_RANGE_6G,
    YRT_JR_ACC_RANGE_8G,
    YRT_JR_ACC_RANGE_12G,
    YRT_JR_ACC_RANGE_16G,
    YRT_JR_ACC_RANGE_24G,
    YRT_JR_ACC_RANGE_32G,
} YRT_JR_ACC_RANGE;

typedef enum
{
    YRT_JR_GYR_RANGE_125DPS,
    YRT_JR_GYR_RANGE_250DPS,
    YRT_JR_GYR_RANGE_500DPS,
    YRT_JR_GYR_RANGE_1000DPS,
    YRT_JR_GYR_RANGE_2000DPS,
} YRT_JR_GYR_RANGE;

typedef enum
{
    YRT_JR_IMU_SR_1_5625,
    YRT_JR_IMU_SR_3_125,
    YRT_JR_IMU_SR_6_25,
    YRT_JR_IMU_SR_12_5,
    YRT_JR_IMU_SR_25,
    YRT_JR_IMU_SR_50,
    YRT_JR_IMU_SR_100,
    YRT_JR_IMU_SR_200,
    YRT_JR_IMU_SR_400,
    YRT_JR_IMU_SR_800,
} YRT_JR_IMU_SR;

typedef enum 
{
    // get uncalibrated raw data for test
    IMU_TEST,

    // use only acc and gyr
    IMU_ACC_GYR,

    // use acc, gyr and fusion euler
    IMU_ACC_GYR_EULER,

    // use acc, gyr and fusion quaternion
    IMU_ACC_GYR_QUATERNION,

    // use acc, gyr, quat and fusion euler
    IMU_ACC_GYR_QUAT_EULER,

} YRT_JR_IMU_MODE;

/** 
 * @brief IMU device structure
 */

#define YRT_JR_IMU_DEV_BHI360 YRT_JR_DISABLED
#define YRT_JR_IMU_DEV_BMI088 YRT_JR_ENABLED
#define YRT_JR_IMU_DEV_BNO055 YRT_JR_DISABLED

typedef struct YRT_IMU_DEV
{
    I2C_HandleTypeDef *hi2c;

    // interrupt pin (for bhi360)
#if YRT_JR_IMU_DEV_BHI360
    GPIO_TypeDef *IMU_IRQ_PORT;
    uint16_t IMU_IRQ_PIN;
    // address (for bhi360)
    uint8_t dev_address;
    /*
     * sample rate (for bhi360)
     * 1.5625, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800 [Hz] supported
     * default 100 Hz
     */
    YRT_IMU_SR sample_rate;
    // report latency '0' for immediate report (for bhi360)
    uint32_t report_latency_ms;
#endif

#if YRT_JR_IMU_DEV_BNO055
    // config (for bno055)
//    bno055_opmode_t imu_mode;
    YRT_ACC_RANGE acc_range; //bno055 not used this config
    YRT_GYR_RANGE gyr_range; //bno055 not used this config
#endif

#if YRT_JR_IMU_DEV_BMI088
    // address (for bmi088)
    uint8_t acc_address;
    uint8_t gyro_address;
    uint8_t mag_address;
#endif
    uint16_t offset_val;

} YRT_IMU_DEV;

YRT_JR_Result_t YRT_JR_IMU_Init(YRT_IMU_DEV *imu_dev);
YRT_JR_Result_t YRT_JR_IMU_Read(YRT_IMU_DEV *imu_dev);
// TODO: IMU Read fonksiyonu parametreleri değişebilir eski apide voidmiş. 
YRT_JR_Result_t YRT_JR_IMU_Config(YRT_IMU_DEV *imu_dev, uint16_t samples);
#endif /* YRT_JR_IS_IMU_ENABLED */

#if YRT_JR_IS_BARO_ENABLED

typedef enum 
{
    YRT_JR_BAROMETER_STANDARD,
    YRT_JR_BAROMETER_ULTRA_HIGH_RES,

} YRT_JR_BAROMETER_CONFIG;

typedef enum
{
    YRT_JR_BAROMETER_ODR_240_HZ,
    YRT_JR_BAROMETER_ODR_218_5_HZ,
    YRT_JR_BAROMETER_ODR_199_1_HZ,
    YRT_JR_BAROMETER_ODR_179_2_HZ,
    YRT_JR_BAROMETER_ODR_160_HZ,
    YRT_JR_BAROMETER_ODR_149_3_HZ,
    YRT_JR_BAROMETER_ODR_140_HZ,
    YRT_JR_BAROMETER_ODR_129_8_HZ,
    YRT_JR_BAROMETER_ODR_120_HZ,
    YRT_JR_BAROMETER_ODR_110_1_HZ,
    YRT_JR_BAROMETER_ODR_100_2_HZ,
    YRT_JR_BAROMETER_ODR_89_6_HZ,
    YRT_JR_BAROMETER_ODR_80_HZ,
    YRT_JR_BAROMETER_ODR_70_HZ,
    YRT_JR_BAROMETER_ODR_60_HZ,
    YRT_JR_BAROMETER_ODR_50_HZ,
    YRT_JR_BAROMETER_ODR_45_HZ,
    YRT_JR_BAROMETER_ODR_40_HZ,
    YRT_JR_BAROMETER_ODR_35_HZ,
    YRT_JR_BAROMETER_ODR_30_HZ,
    YRT_JR_BAROMETER_ODR_25_HZ,
    YRT_JR_BAROMETER_ODR_20_HZ,
    YRT_JR_BAROMETER_ODR_15_HZ,
    YRT_JR_BAROMETER_ODR_10_HZ,
    YRT_JR_BAROMETER_ODR_05_HZ,
    YRT_JR_BAROMETER_ODR_04_HZ,
    YRT_JR_BAROMETER_ODR_03_HZ,
    YRT_JR_BAROMETER_ODR_02_HZ,
    YRT_JR_BAROMETER_ODR_01_HZ,
    YRT_JR_BAROMETER_ODR_0_5_HZ,
    YRT_JR_BAROMETER_ODR_0_250_HZ,
    YRT_JR_BAROMETER_ODR_0_125_HZ
} YRT_JR_BAROMETER_ODR;

typedef enum
{
    YRT_JR_BAROMETER_OSR_1X,
    YRT_JR_BAROMETER_OSR_2X,
    YRT_JR_BAROMETER_OSR_4X,
    YRT_JR_BAROMETER_OSR_8X,
    YRT_JR_BAROMETER_OSR_16X,
    YRT_JR_BAROMETER_OSR_32X,
    YRT_JR_BAROMETER_OSR_64X,
    YRT_JR_BAROMETER_OSR_128X,
    YRT_JR_BAROMETER_OSR_256X,
    YRT_JR_BAROMETER_OSR_512X,
    YRT_JR_BAROMETER_OSR_1024X,
    YRT_JR_BAROMETER_OSR_2048X,
    YRT_JR_BAROMETER_OSR_4096X,
} YRT_JR_BAROMETER_OSR;

typedef enum
{
    YRT_JR_BAROMETER_IIR_OFF,
    YRT_JR_BAROMETER_IIR_1,
    YRT_JR_BAROMETER_IIR_3,
    YRT_JR_BAROMETER_IIR_7,
    YRT_JR_BAROMETER_IIR_15,
    YRT_JR_BAROMETER_IIR_31,
    YRT_JR_BAROMETER_IIR_63,
    YRT_JR_BAROMETER_IIR_127
} YRT_JR_BAROMETER_IIR;

/**
 * @brief Barometer device structure
 */

typedef struct {
    YRT_JR_BAROMETER_CONFIG config;
    YRT_JR_BAROMETER_ODR odr;
    YRT_JR_BAROMETER_OSR osr;
    YRT_JR_BAROMETER_IIR iir;
    uint8_t address;
}YRT_JR_BARO_DEV;

/**
 * @brief Barometer functions
 */

YRT_JR_Result_t YRT_JR_BARO_Init(YRT_JR_BARO_DEV *baro_dev);
YRT_JR_Result_t YRT_JR_BARO_Read(YRT_JR_BARO_DEV *baro_dev, YRT_Baro_Data_t *baro_data);
YRT_JR_Result_t YRT_JR_BARO_Config(YRT_JR_BARO_DEV *baro_dev);
YRT_JR_Result_t YRT_JR_BARO_ReadAltitude(YRT_JR_BARO_DEV *baro_dev);
YRT_JR_Result_t YRT_JR_BARO_ReadBasePressure(YRT_JR_BARO_DEV *baro_dev);
float YRT_JR_BARO_CalculateAltitude(float pressure, float base_pressure);

#endif /* YRT_JR_IS_BARO_ENABLED */

#if YRT_JR_IS_CRC_ENABLED
#include "crc.h"
uint16_t YRT_CRCCalculator(uint8_t  *buf, uint16_t len);
#endif /* YRT_JR_IS_CRC_ENABLED */

#endif /* YRT_JR_API_H */

//TODO: Kalan modüllere vs ne kodu yazacağımıza tekrar bakarız.