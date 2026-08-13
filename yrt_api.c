# include "yrt_api.h"


//LORA
#if YRT_IS_LORA_ENABLED


    static LoRa my_lora;


    //LoRa init
    YRT_Status_t yrt_LoRa_init(const yrt_LoRa_conf_t *config){


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
        my_lora.bandWidth             = config -> bandWidth;       // defsault = BW_125KHz
        my_lora.crcRate               = config -> crcRate ;          // default = CR_4_5
        my_lora.power                 = config -> power;      // default = 20db
        my_lora.overCurrentProtection = config -> overCurrentProtection;             // default = 100 mA
        my_lora.preamble              = config -> preamble;              // default = 8;


           // 1. CS pinini HIGH yap (SPI haberleşmesi için falling edge şart)
           HAL_GPIO_WritePin(my_lora.CS_port, my_lora.CS_pin, GPIO_PIN_SET);
           
           // 2. Çipi Resetle (CubeMX reset pinini LOW başlattığı için çip kilitli kalıyor)
           LoRa_reset(&my_lora);
           
           // 200 dönerse LORA_OK demektir
            if (LoRa_init(&my_lora) == 200) { 
                return YRT_OK; // Bizim sisteme göre "Başarılı (0)"
            }
            return YRT_ERROR; // Çip bulunamadıysa (404)
    
    }



    // LoRa transmit
    
  	YRT_Status_t yrt_LoRa_Transmit(const uint8_t *data, uint8_t length, uint16_t timeout) {
            
            // Kütüphane 1 döndürürse başarılı demektir.
            if (LoRa_transmit(&my_lora, (uint8_t*)data, length, timeout) == 1) {
                return YRT_OK; // Bizim sistemimize göre "Başarılı (0)" döndür
            }
             
            return YRT_ERROR_TIMEOUT; // Hata olduysa
	}

    // LoRa receive
    YRT_Status_t yrt_LoRa_Receive(uint8_t *receive_data, uint8_t length, uint16_t timeout){

        //LoRa_startReceiving(&my_lora);
        //uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length);
        //şimdilik dursun burası, asıl receive fonksiyonunu ekleyeceğiz
      uint32_t start_time = YRT_Get_Time(); // Kronometreyi başlat
            uint8_t received_bytes = 0;
            
            LoRa_startReceiving(&my_lora); // Telsizi dinleme moduna al

            // Timeout süresi dolana kadar sürekli havayı dinle
            while ( (YRT_Get_Time() - start_time) < timeout ) {
                
                // Havadan gelen paketi çekmeyi dene
                received_bytes = LoRa_receive(&my_lora, receive_data, length);
                
                if (received_bytes > 0) {
                    return YRT_OK; // Veri yakalandı ve başarıyla alındı!
                }
                
                YRT_Delay(1); // İşlemciyi boğmamak için 1 milisaniye nefesal
            }

            return YRT_ERROR_TIMEOUT; // Süre doldu, kimse bir şey göndermedi
       
    
    }





#endif











//BARO
    #if YRT_IS_BARO_ENABLED
    
        #if YRT_BARO_MS5611_SELECTED
            
            static MS5611_t ms5611_dev; 
            static I2C_HandleTypeDef *baro_i2c; 
    
          
            static uint8_t ms5611_i2c_write(uint8_t addr, uint8_t *data, uint16_t len) {
                return HAL_I2C_Master_Transmit(baro_i2c, addr, data, len, 100);
            }
        
            
            static uint8_t ms5611_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
                return HAL_I2C_Master_Receive(baro_i2c, addr, data, len, 100);
            }

        #elif YRT_BARO_BMP280_SELECTED
            // static BMP280_t bmp280_dev  mesela
        #endif
    
    

        // evrensel init fonksiyonu
        YRT_Status_t YRT_Baro_Init(const yrt_Baro_conf_t *config){
    
            #if YRT_BARO_MS5611_SELECTED
              
                baro_i2c = (I2C_HandleTypeDef *)config->hi2c;
                
             
                ms5611_dev.i2c_send = ms5611_i2c_write;
                ms5611_dev.i2c_receive = ms5611_i2c_read;
                ms5611_dev.delay_ms = yrt_delay; 
                
                
                if(MS5611_Init(&ms5611_dev) == YRT_OK) return YRT_OK;
                return YRT_ERROR;
    


            #elif YRT_BARO_BMP280_SELECTED
                // BMP280 kodları buraya
                return YRT_OK;
            #else
                return YRT_ERROR;
            #endif
        }



        


        YRT_Status_t YRT_Baro_Read_All(yrt_Baro_Data_t *out_data) {

            #if YRT_BARO_MS5611_SELECTED
                
                if (MS5611_ReadRawData(&ms5611_dev) == YRT_OK) {
                    MS5611_Calculate(&ms5611_dev);

                    out_data->altitude_m      = ms5611_dev.altitude;
                    out_data->pressure_pa     = ms5611_dev.pressure;
                    out_data->temperature_c   = ms5611_dev.temperature;
                    out_data->raw_pressure    = ms5611_dev.D1;
                    out_data->raw_temperature = ms5611_dev.D2;
                    return YRT_OK;
                }
                return YRT_ERROR_I2C;
                



            #elif YRT_BARO_BMP280_SELECTED
		// BMP280 okuma işlemi
                return YRT_OK;
            #else
                return YRT_ERROR;
            #endif
        }



        float YRT_Baro_Get_Altitude(void) {
            yrt_Baro_Data_t temp_data;
            if (YRT_Baro_Read_All(&temp_data) == YRT_OK) {
                return temp_data.altitude_m;
            }
            return 0.0f; // Hata
        }

    #endif // YRT_IS_BARO_ENABLED








//IMU
#if  YRT_IS_IMU_ENABLED


        typedef struct {
            void *hi2c;   

        } yrt_IMU_conf_t;
    

        
    
YRT_Status_t YRT_IMU_Init(const yrt_IMU_conf_t *config){

   #if YRT_IMU_BNO055_SELECTED
   //BNO055
   
   //BNO055_Init(&hi2c1, &ucus_sensoru);

  
    #elif   YRT_IMU_MPU6050_SELECTED
    //MPU6050

    MPU6050_Initialization();

    return YRT_OK;

  // bunu okuma yapmak için kullanacağız üst katmanlarda
  //  MPU6050_ProcessData(&MPU6050);
    #elif YRT_IMU_BMI088_SELECTED
    //BMI088


    #else

 

    //Farklı sensör varsa buraya
    
    #endif

   return YRT_ERROR;
}




YRT_Status_t YRT_IMU_Read_All(yrt_IMU_Data_t *out_data){


#if YRT_IMU_MPU6050_SELECTED


MPU6050_ProcessData(&MPU6050);


    out_data ->accel_x = MPU6050.acc_x;
    out_data ->accel_y = MPU6050.acc_y;
    out_data ->accel_z = MPU6050.acc_z;
    out_data ->gyro_x  = MPU6050.gyro_x;
    out_data ->gyro_y  = MPU6050.gyro_y;
    out_data ->gyro_z  = MPU6050.gyro_z;
    out_data ->temperature = MPU6050.temperature;
 

    return YRT_OK;

#endif



#if YRT_IMU_BNO055_SELECTED

#endif



#if YRT_IMU_BMI088_SELECTED

#endif



return YRT_ERROR;


}












#endif

#include "cmsis_os.h"

uint32_t YRT_Get_Time(void) {
    return osKernelSysTick();
}

YRT_Status_t YRT_Delay(uint32_t period) {
    osDelay(period);
    return YRT_OK;
}











