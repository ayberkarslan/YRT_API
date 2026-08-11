
#ifndef YRT_API_H
#define YRT_API_H





#define YRT_ENABLED
#define YRT_DISABLED

#define YRT_IS_LORA_ENABLED YRT_ENABLED









//IMU
#if YRT_IS_IMU_ENABLED

#define YRT_IMU_BNO055_SELECTED YRT_DISABLED
#define YRT_IMU_MPU6050_SELECTED YRT_DISABLED
#define YRT_IMU_BMI088_SELECTED YRT_DISABLED

#include "bno055.h"

#endif




//BARO
#if YRT_IS_BARO_ENABLED

#define YRT_BARO_BMP280_SELECTED YRT_DISABLED
#define YRT_BARO_MS5611_SELECTED YRT_DISABLED







#endif



//LORA
#if LORA_ENABLED 


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

uint8_t yrt_LoRa_init(const yrt_LoRa_conf_t *config);

uint8_t yrt_LoRa_Transmit(const uint8_t *data, uint8_t length, uint16_t timeout);

uint8_t yrt_LoRa_Receive(uint8_t *send_data, uint8_t length, uint16_t timeout);

uint8_t yrt_delay(uint8_t period);


#endif





#











#endif