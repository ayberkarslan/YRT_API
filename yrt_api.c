# include "yrt_api.h"



//LORA
#if YRT_IS_LORA_ENABLED

    static LoRa my_lora;


    //LoRa init
    uint16_t yrt_LoRa_init(const yrt_LoRa_conf_t *config){


        my_lora = newLoRa();

        my_lora.CS_port         = config -> nss_port;
        my_lora.CS_pin          = config -> nss_pin;
        my_lora.reset_port      = config -> reset_port;
        my_lora.reset_pin       = config -> reset_pin;
        my_lora.DIO0_port       = config -> dio0_port;
        my_lora.DIO0_pin        = config -> dio0_pin;
        my_lora.hSPIx           = config -> hspi;

        my_lora.frequency             = config -> frequency;             // default = 433 MHz
        my_lora.spredingFactor        = config -> spredingFactor;            // default = SF_7
        my_lora.bandWidth             = config -> bandWidth;       // default = BW_125KHz
        my_lora.crcRate               = config -> crcRate ;          // default = CR_4_5
        my_lora.power                 = config -> power;      // default = 20db
        my_lora.overCurrentProtection = config -> overCurrentProtection;             // default = 100 mA
        my_lora.preamble              = config -> preamble;              // default = 8;


        return LoRa_init(&my_lora);
    
    }



    // LoRa transmit
    uint8_t yrt_LoRa_Transmit(const uint8_t *data, uint8_t length, uint16_t timeout){

    LoRa_transmit(&my_lora, (uint8_t*)data, length, timeout);

    }


    // LoRa receive
    uint8_t yrt_LoRa_Receive(uint8_t *send_data, uint8_t length, uint16_t timeout){

        LoRa_startReceiving(&my_lora);
        //uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length);
        //şimdilik dursun burası, asıl receive fonksiyonunu ekleyeceğiz
    
    }


#endif











//BARO
#if YRT_BARO_MS5611_SELECTED

        typedef struct {
            void *hi2c;

        } yrt_Baro_conf_t;
    

        
    
uint8_t YRT_Baro_Init(const yrt_Baro_conf_t *config){

    #if YRT_BARO_MS5611_SELECTED
    //MS5611
    MS5611_Init(&hi2c1, &ucus_sensoru);

    #elif YRT_BARO_BMP280_SELECTED
    //BMP280

    #else

    //Farklı sensör varsa burayı doldur
    #endif

}




float YRT_Baro_ReadAltitude(void);

#endif





//IMU
#if  YRT_IS_IMU_ENABLED


        typedef struct {
            void *hi2c;   

        } yrt_IMU_conf_t;
    

        
    
uint8_t YRT_Baro_Init(const yrt_Baro_conf_t *config){

   #if YRT_IMU_BNO055_SELECTED
   //BNO055
   
   BNO055_Init(&hi2c1, &ucus_sensoru);

  
    #elif   YRT_IMU_MPU6050_SELECTED
    //MPU6050


    #elif YRT_IMU_BMI088_SELECTED
    //BMI088


    #else
    //Farklı sensör varsa burayı 
    
    #endif

}




float YRT_Baro_ReadAltitude(void);

#endif











