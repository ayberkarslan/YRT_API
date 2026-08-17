/*

*/

#ifndef YRT_API_H
#define YRT_API_H

#include "cmsis_os.h"

typedef enum {
    YRT_OK = 0,
    YRT_ERR,
    YRT_IMU_ERR,
    YRT_BARO_ERR,
	YRT_CAN_ERR,
	YRT_COMM_ERR,
    YRT_TIMEOUT,
	YRT_INVALID_PARAM
} YRT_Status_t;

#define YRT_ENABLED 1
#define YRT_DISABLED 0

#define YRT_IS_RTOS_ENABLED YRT_ENABLED
#define YRT_IS_LORA_ENABLED YRT_ENABLED
#define YRT_IS_IMU_ENABLED YRT_ENABLED
#define YRT_IS_BARO_ENABLED YRT_ENABLED
#define YRT_IS_SERIAL_ENABLED YRT_ENABLED

#if YRT_IS_SERIAL_ENABLED
#define YRT_IS_CAN_ENABLED YRT_DISABLED
#endif

#include "stm32f4xx_hal.h"
#include "math.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#if YRT_IS_BARO_ENABLED

#define YRT_BARO_BMP280_SELECTED YRT_DISABLED
#define YRT_BARO_MS5611_SELECTED YRT_ENABLED

#endif



//IMU
#if YRT_IS_IMU_ENABLED

#define YRT_IMU_BNO055_SELECTED YRT_DISABLED
#define YRT_IMU_MPU6050_SELECTED YRT_DISABLED
#define YRT_IMU_BMI088_SELECTED YRT_ENABLED




    
typedef struct {
    float x;
    float y;
    float z;
} YRT_Vec3;

typedef struct {
    float x;
    float y;
    float z;
    float w;
} YRT_Vec4;

uint32_t YRT_Get_Time(void);  //kendisi aşağıda, hata vermesin diye üste alanzi


#if YRT_IMU_BNO055_SELECTED
#include "bno055.h"
bno055_t bno055;

typedef struct {
    bno055_pwr_t power_mod;

    bno055_acc_range_t acc_range;
    bno055_acc_band_t acc_bandwidth;
    bno055_acc_mode_t acc_mode;

    bno055_gyr_range_t gyro_range;
    bno055_gyr_band_t gyro_bandwidth;
    bno055_gyr_mode_t gyro_mode;

    bno055_mag_rate_t mag_rate;
    bno055_mag_mode_t mag_mode;
    bno055_mag_pwr_t mag_power_mode;

    I2C_HandleTypeDef* hi2c;
    u8 dev_address;
} YRT_IMU_Config_t;

typedef struct {
    YRT_Vec4 quaternion;
    YRT_Vec3 euler;
    YRT_Vec3 lineer_accel;
    YRT_Vec3 accel;
    YRT_Vec3 grav;
    YRT_Vec3 gyro;
    YRT_Vec3 mag;
} YRT_IMU_Data_t;



#elif YRT_IMU_MPU6050_SELECTED
      #include "MPU6050.h"
    extern Struct_MPU6050 MPU6050;



#elif YRT_IMU_BMI088_SELECTED
#include "BMI088.h"

typedef struct {
    bmi088_imu_acc_bwp_osr_t acc_bwp_osr;
    bmi088_imu_acc_odr_t acc_odr;
    bmi088_imu_acc_range_t acc_range;
    bmi088_imu_gyro_range_t gyro_range;
    bmi088_imu_gyro_bw_t gyro_filter_bw;

    I2C_HandleTypeDef *hi2c;
} YRT_IMU_Config_t;

typedef struct {
    YRT_Vec3 accel;
    YRT_Vec3 gyro;
} YRT_IMU_Data_t;

#else
    //Farkli sensorun kutuphanesi
#endif



typedef struct {
    YRT_IMU_Config_t config;
    YRT_IMU_Data_t data;
} YRT_IMU_t;



/**
 * @brief Seçilen imuyu kurma fonksiyonu.
 * 
 * @param yrt_imu 
 * @return YRT_Status_t 
 */
YRT_Status_t YRT_IMU_Init(YRT_IMU_t *yrt_imu);

/**
 * @brief Bizim structımızı seçilen kütüphanenin structına uydurma fonksiyonu.
 * 
 * @param yrt_imu 
 * @return YRT_Status_t 
 */
YRT_Status_t YRT_IMU_Config(YRT_IMU_t *yrt_imu);

/**
 * @brief IMU veri okuma fonksiyonu.
 * 
 * @param yrt_imu 
 * @return YRT_Status_t 
 */
YRT_Status_t YRT_IMU_Read(YRT_IMU_t *yrt_imu);
#endif




//GPS
#define YRT_IS_GPS_ENABLED YRT_ENABLED


//GPS STRUCTS
typedef struct{
    void *huart;
}YRT_GPS_Config_t;


typedef struct{
        float latitude;       // enlem
        float longitude;      // boylam
        float altitude;       // rakım (m)
        float speed_kmh;      // hız (km/h)
        uint8_t satellites;   // uydu sayısı
        uint8_t fix_status;   // 1:OK 0:YOK
    }YRT_GPS_t;


YRT_Status_t YRT_GPS_Init(const YRT_GPS_Config_t *config);
YRT_Status_t YRT_GPS_Read(YRT_GPS_t *out_data);





//BARO
#if YRT_IS_BARO_ENABLED

#if YRT_BARO_BMP280_SELECTED
#include "bmp280.h"

typedef struct {

    BMP280_Mode baro_mode;
    BMP280_Filter filter;
    BMP280_Oversampling osr_press;
    BMP280_Oversampling osr_temp;
    BMP280_StandbyTime standby;
    I2C_HandleTypeDef* hi2c;
    uint8_t dev_address;

} YRT_BARO_Config_t;

typedef struct {
    float pressure;
    float temperature;
    float altitude;
    float ground_pressure;
} YRT_BARO_Data_t;


#elif YRT_BARO_MS5611_SELECTED
    #include "ms5611_yrt.h"

typedef struct {

	void *hi2c;
	float lp_coef;
	MS5611_OSR_t osr;

} YRT_BARO_Config_t;

typedef struct {
    float    altitude;
    float    pressure;
    float    temperature;
    uint32_t raw_pressure;
    uint32_t raw_temperature;
} YRT_BARO_Data_t;

#else
    //Farkli sensorun kutuphanesi
#endif

typedef struct {
    YRT_BARO_Config_t config;
    YRT_BARO_Data_t data;
} YRT_BARO_t;

/**
 * @brief Seçilen barometreyi kurma fonksiyonu.
 * 
 * @param yrt_baro 
 * @return YRT_Status_t 
 */
YRT_Status_t YRT_Baro_Init(YRT_BARO_t* yrt_baro);

/**
 * @brief Bizim structımızı seçilen kütüphanenin structına uydurma fonksiyonu.
 * 
 * @param baro_config 
 * @return YRT_Status_t 
 */
YRT_Status_t YRT_Baro_Config(YRT_BARO_t* baro_config);

/**
 * @brief Barometreden veri okuma fonksiyonu.
 * 
 * @param yrt_baro 
 * @return YRT_Status_t 
 */
YRT_Status_t YRT_Baro_Read(YRT_BARO_t* yrt_baro);

#endif

//LORA
#if YRT_IS_LORA_ENABLED 


#include "LoRa.h"

   typedef struct {
        int      frequency;
        uint8_t  spredingFactor;
        uint8_t  bandWidth;
        uint8_t  crcRate;
        uint8_t  power;
        uint8_t  overCurrentProtection;
        uint16_t preamble;
    
        void     *hspi;          // Hangi SPI?
        void     *nss_port;      // Hangi Port?
        uint16_t nss_pin;        // Hangi Pin?
        void     *reset_port;    
        uint16_t reset_pin;      
        void     *dio0_port;     
        uint16_t dio0_pin;       
    } yrt_LoRa_conf_t;

//lora prototypes

YRT_Status_t yrt_LoRa_init(const yrt_LoRa_conf_t *config);

YRT_Status_t yrt_LoRa_Transmit(const uint8_t *data, uint8_t length, uint16_t timeout);

YRT_Status_t yrt_LoRa_Receive(uint8_t *send_data, uint8_t length, uint16_t timeout);

#endif


/* #if YRT_IS_GPS_ENABLED

does nothing because we have uart
⢀⡴⠑⡄⠀⠀⠀⠀⠀⠀⠀⣀⣀⣤⣤⣤⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ 
⠸⡇⠀⠿⡀⠀⠀⠀⣀⡴⢿⣿⣿⣿⣿⣿⣿⣿⣷⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠑⢄⣠⠾⠁⣀⣄⡈⠙⣿⣿⣿⣿⣿⣿⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⢀⡀⠁⠀⠀⠈⠙⠛⠂⠈⣿⣿⣿⣿⣿⠿⡿⢿⣆⠀⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⢀⡾⣁⣀⠀⠴⠂⠙⣗⡀⠀⢻⣿⣿⠭⢤⣴⣦⣤⣹⠀⠀⠀⢀⢴⣶⣆ 
⠀⠀⢀⣾⣿⣿⣿⣷⣮⣽⣾⣿⣥⣴⣿⣿⡿⢂⠔⢚⡿⢿⣿⣦⣴⣾⠁⠸⣼⡿ 
⠀⢀⡞⠁⠙⠻⠿⠟⠉⠀⠛⢹⣿⣿⣿⣿⣿⣌⢤⣼⣿⣾⣿⡟⠉⠀⠀⠀⠀⠀ 
⠀⣾⣷⣶⠇⠀⠀⣤⣄⣀⡀⠈⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀ 
⠀⠉⠈⠉⠀⠀⢦⡈⢻⣿⣿⣿⣶⣶⣶⣶⣤⣽⡹⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⠉⠲⣽⡻⢿⣿⣿⣿⣿⣿⣿⣷⣜⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣷⣶⣮⣭⣽⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⣀⣀⣈⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠇⠀⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⠀⠀⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀ 
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⠻⠿⠿⠿⠿⠛⠉

*/

void YRT_Delay(uint32_t period);

//CAN
#if YRT_IS_CAN_ENABLED

#include "can.h"
#include "cmsis_os.h"

typedef enum {
    YRT_CAN_FIFO0 = 0,
    YRT_CAN_FIFO1 = 1
} YRT_CAN_FIFO_NUM_t;

typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} CAN_Msg_t;

typedef struct {
    CAN_HandleTypeDef    *hcan;          // Hangi CAN? Cevabi BPC_SET.c'de.
    YRT_CAN_FIFO_NUM_t fifo_num;
} YRT_CAN_DEV_t;

#if YRT_IS_RTOS_ENABLED
extern osMessageQueueId_t canRxQueueHandle;
#endif

/**
 * @brief CAN filtresini kurup CAN'i baslatir.
 *
 * @param can_dev
 * @return YRT_Status_t
 */
YRT_Status_t YRT_CAN_Init(YRT_CAN_DEV_t *can_dev);

#endif

//GPIO
typedef enum {
    YRT_MODE_ACTIVE,
    YRT_MODE_INACTIVE
} YRT_OP_MODE;

typedef enum {
    YRT_DEV_OFF,
    YRT_DEV_ON,
    YRT_DEV_ERROR,
    YRT_DEV_BLINK
} YRT_DEV_STATE;

typedef struct {
    GPIO_TypeDef     *port;
    uint16_t          pin;
    YRT_OP_MODE    mode;
    YRT_DEV_STATE  state;
} YRT_GPIO;

typedef struct {
    YRT_GPIO USR_LED1;
    YRT_GPIO USR_LED2;
    YRT_GPIO BUZZER;
} YRT_LED_BUZZER_DEV;

// TODO: Morse ve pyro kodu kismini anlarsam buraya entegre edecegim.

YRT_Status_t YRT_GPIO_Init(YRT_GPIO *gpio, GPIO_TypeDef *port, uint16_t pin, YRT_OP_MODE mode);
void YRT_GPIO_SetState(YRT_GPIO *gpio, YRT_DEV_STATE state);
void YRT_GPIO_Toggle(YRT_GPIO *gpio);

//SERIAL
#if YRT_IS_SERIAL_ENABLED

typedef enum {
    YRT_SERIAL_UART,
    YRT_SERIAL_USB,
    YRT_SERIAL_CAN
} YRT_SERIAL_PROTOCOL;

typedef struct {
    YRT_SERIAL_PROTOCOL  protocol;
    UART_HandleTypeDef     *huart;       // Hangi UART? Cevabi BPC_SET.c'de.
#if YRT_IS_CAN_ENABLED
    CAN_HandleTypeDef      *hcan;        // Hangi CAN?  Cevabi BPC_SET.c'de.
#endif
    uint32_t                target_id;
    uint32_t                bytesSent;
    uint32_t                bytesReceived;
} YRT_SERIAL_DEV;

YRT_Status_t YRT_Serial_Send(YRT_SERIAL_DEV *dev, uint8_t *data, uint16_t len);
YRT_Status_t YRT_Serial_Receive(YRT_SERIAL_DEV *dev, uint8_t *data, uint16_t len);

#endif

#endif

