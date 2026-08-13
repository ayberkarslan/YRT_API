#ifndef YRT_API_H
#define YRT_API_H

#include <stdint.h>




#define YRT_ENABLED 1
#define YRT_DISABLED 0





    typedef enum {
        YRT_OK = 0,            // her şey relax
        YRT_ERROR,             // genel
        YRT_ERROR_TIMEOUT,     
        YRT_ERROR_I2C,         // i2c hat hatası veya cevap yok
        YRT_ERROR_SPI,         // spi
        YRT_ERROR_NOT_READY  
    } YRT_Status_t;






uint32_t YRT_Get_Time(void);
YRT_Status_t YRT_Delay(uint32_t period);





//FREERTOS
#define YRT_IS_FREERTOS_ENABLED YRT_ENABLED

//LORA
#define YRT_IS_LORA_ENABLED YRT_ENABLED

//IMU
#define YRT_IMU_BNO055_SELECTED YRT_DISABLED
#define YRT_IMU_MPU6050_SELECTED YRT_DISABLED
#define YRT_IMU_BMI088_SELECTED YRT_DISABLED




typedef struct{

void *hi2c;

}YRT_IMU_conf_t;



typedef struct {

    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature;

   } yrt_IMU_Data_t;

YRT_Status_t YRT_IMU_Init(const YRT_IMU_conf_t *config);
YRT_Status_t YRT_IMU_Read_All(yrt_IMU_Data_t *out_data);




//BARO

#define YRT_BARO_BMP280_SELECTED YRT_DISABLED
#define YRT_BARO_MS5611_SELECTED YRT_DISABLED



float YRT_Baro_ReadAltitude(void);








//FREERTOS
#if IS_FREERTOS_ENABLED

    include "cmsis_os.h"

#endif





//IMU
#if YRT_IS_IMU_ENABLED

#if YRT_IMU_BNO055_SELECTED
    #include "bno055_yrt.h"

#elif YRT_IMU_MPU6050_SELECTED
    #include "MPU6050.h"
    extern Struct_MPU6050 MPU6050;


#elif YRT_IMU_BMI088_SELECTED
    #include "bmi088_yrt.h"

#else
//another libs 

#endif

#endif




//BARO
#if YRT_IS_BARO_ENABLED



/*
typedef enum {
    YRT_BARO_MODE_SLEEP,
    YRT_BARO_MODE_FORCED,
    YRT_BARO_MODE_NORMAL
} YRT_BARO_Mode_t;

typedef enum {
    YRT_BARO_FILTER_OFF,
    YRT_BARO_FILTER_2,
    YRT_BARO_FILTER_4,
    YRT_BARO_FILTER_8,
    YRT_BARO_FILTER_16
} YRT_BARO_Filter_t;

typedef enum {
    YRT_BARO_OS_SKIPPED,
    YRT_BARO_OS_ULTRA_LOW_POWER,
    YRT_BARO_OS_LOW_POWER,
    YRT_BARO_OS_STANDARD,
    YRT_BARO_OS_HIGH_RES,
    YRT_BARO_OS_ULTRA_HIGH_RES
} YRT_BARO_Oversampling_t;

typedef enum {
    YRT_BARO_STANDBY_05,
    YRT_BARO_STANDBY_62,
    YRT_BARO_STANDBY_125,
    YRT_BARO_STANDBY_250,
    YRT_BARO_STANDBY_500,
    YRT_BARO_STANDBY_1000,
    YRT_BARO_STANDBY_2000,
    YRT_BARO_STANDBY_4000
} YRT_BARO_StandbyTime;

typedef struct {
    I2C_HandleTypeDef hi2c;
    uint8_t dev_address;

    YRT_BARO_Mode_t baro_mode;
    YRT_BARO_Filter_t filter;
    YRT_BARO_Oversampling_t osr;
    YRT_BARO_StandbyTime standby;
} YRT_BARO_CONFIG_t;  */


typedef struct{

        void *hi2c;

}yrt_Baro_Config_t ;



typedef struct {
        float    altitude; 
        float    pressure;    
        float    temperature;  
        uint32_t raw_pressure;  
        uint32_t raw_temperature;
}yrt_Baro_Data_t;






YRT_Status_t YRT_Baro_Init(const  yrt_Baro_Config_t *config);

YRT_Status_t YRT_Baro_Read_All(yrt_Baro_Data_t *out_data);

float YRT_Baro_Get_Altitude(void);





#if YRT_BARO_MS5611_SELECTED
    #include "ms5611_yrt.h"

#elif YRT_BARO_BMP280_SELECTED
    #include "bmp280_yrt.h"

#else
//another libs 

#endif

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

YRT_Status_t yrt_delay(uint8_t period);


#endif

















#endif