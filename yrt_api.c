#include "yrt_api.h"
#include <string.h>
#include <stdlib.h>//LORA
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


           // cs high (spi başlaması için falling edge şart)
           HAL_GPIO_WritePin(my_lora.CS_port, my_lora.CS_pin, GPIO_PIN_SET);
           
           // önce reset at
           LoRa_reset(&my_lora);
           
           // 200 dönerse LORA_OK demektir
                if (LoRa_init(&my_lora) == 200) { 
                    return YRT_OK; 
                }
            return YRT_ERROR;
    }



    // LoRa transmit
    
  	YRT_Status_t yrt_LoRa_Transmit(const uint8_t *data, uint8_t length, uint16_t timeout) {
            
            // ÖNEMLİ LAN - Eski interruptan kalan bayrakları temizliyoruz ki LoRa kendini kilitlemesin (açılış parazitinde yapabiliyor bunu)
            LoRa_write(&my_lora, RegIrqFlags, 0xFF);

            //kütüphane 1 döndürürse başarılı demek
            if (LoRa_transmit(&my_lora, (uint8_t*)data, length, timeout) == 1) {
                return YRT_OK; 
            }
             
            return YRT_ERROR_TIMEOUT; 
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

#endif // YRT_IS_IMU_ENABLED

#if YRT_IS_GPS_ENABLED

static void *gps_uart;
static uint8_t gps_rx_buffer[100]; //GNGGA bufferi



//GPS



YRT_Status_t YRT_GPS_Init(const YRT_GPS_Config_t *config){
    if(config == NULL || config->huart == NULL){
       return YRT_ERROR;
    }
    gps_uart = config->huart;

    // --- UART HATA TEMIZLEME ---
    __HAL_UART_CLEAR_OREFLAG((UART_HandleTypeDef*)gps_uart);
    __HAL_UART_CLEAR_FEFLAG((UART_HandleTypeDef*)gps_uart);
    
    HAL_Delay(100);

    HAL_UART_Receive_DMA(gps_uart, gps_rx_buffer, 100);
    return YRT_OK;
}













    void YRT_GPS_Parse(const char *sentence, int field_index, char *out_buffer) {
        // DMA circular buffer olduğu için kelimeler dizinin sonundan başına bölünebilir!
        // Bunu çözmek için diziyi kendi arkasına kopyalayarak (200 harf) düz bir dizi yapıyoruz.
        char safe_buffer[201];
        memcpy(safe_buffer, sentence, 100);
        memcpy(safe_buffer + 100, sentence, 100); // Peşine aynısını bir daha ekle
        safe_buffer[200] = '\0';

        // Buffer'ın içinde GNGGA veya GPGGA arıyoruz
        char *start = strstr(safe_buffer, "$GNGGA");
        if (start == NULL) {
            start = strstr(safe_buffer, "$GPGGA");
        }
        
        // Eğer GGA cümlesi bulunamadıysa boş dön
        if (start == NULL) {
            out_buffer[0] = '\0';
            return;
        }

        int i = 0;             // GNGGA kelimesinin başından itibaren harf sayacı
        int current_comma = 0; // Virgül sayacı
        int out_i = 0;         // Çıkarttığımız karakterin bufferdaki gideceği sırası

        // İstenilen virgüle kadar cümleyi sar (Kelimeleri atla)
        while (start[i] != '\0' && current_comma < field_index) {
            if (start[i] == ',') {
                current_comma++;
            }
            i++;
        }

        // İstenilen virgüle ulaşınca kopyalamaya başla - diğer virgüle veya yıldıza kadar
        while (start[i] != '\0' && start[i] != ',' && start[i] != '*') {
            out_buffer[out_i] = start[i];
            out_i++;
            i++;
        }

        // Stringi kapatıyoruz
        out_buffer[out_i] = '\0';
    }











        YRT_Status_t YRT_GPS_Read(YRT_GPS_t *out_data) {
        char gecici_yazi[20];
        float raw_value;
        int degrees;
        float minutes;

        // fix kalitesi (6)
        YRT_GPS_Parse((const char *)gps_rx_buffer, 6, gecici_yazi);
        out_data->fix_status = atoi(gecici_yazi); // atoi: yazıyı tam sayıya
        
        // uydulara henüz kilitlenmemişse saçma sapan veriler okumamak için direkt çıktık
        if (out_data->fix_status == 0) {
            return YRT_ERROR; 
        }

        // uydu sayısı (7)
        YRT_GPS_Parse((const char*)gps_rx_buffer, 7, gecici_yazi);
        out_data->satellites = atoi(gecici_yazi);

        // rakım (9)
        YRT_GPS_Parse((const char*)gps_rx_buffer, 9, gecici_yazi);
        out_data->altitude = atof(gecici_yazi); // atof: yazıyı ondalıklı sayıya çevirir

        // enlem lat (2) -> DDMM.MMMM formatı
        YRT_GPS_Parse((const char*)gps_rx_buffer, 2, gecici_yazi);
        raw_value = atof(gecici_yazi);         // Örneğin 4100.1234 gelir
        degrees = (int)(raw_value / 100);      // 4100'ü 100'e bölüp tam kısmını aldık: 41 derece
        minutes = raw_value - (degrees * 100); // 4100.1234 - 4100 = 0.1234 dakika kalır
        out_data->latitude = degrees + (minutes / 60.0); //google maps formatı

        // boylam long (4)
        YRT_GPS_Parse((const char*)gps_rx_buffer, 4, gecici_yazi);
        raw_value = atof(gecici_yazi);         // Örneğin 02800.5678 gelir
        degrees = (int)(raw_value / 100);      // 28 derece
        minutes = raw_value - (degrees * 100); // 0.5678 dakika
        out_data->longitude = degrees + (minutes / 60.0);

        return YRT_OK;
    }







/*
 YRT_Status_t yrt_GPS_Read(yrt_GPS_Data_t *out_data){

    char gecici_yazi[20]; // tazıyı alacağımız buffer

    //9. virgül sonrası mesela
    YRT_GPS_Parse(gps_rx_buffer, 9, gecici_yazi); 
    // yazıyı float yapıp apiye at
    out_data->altitude = atof(gecici_yazi); 

       out_data->longitude = ;
       out_data->latitude =  ;
       return YRT_OK;
 }

*/





#endif










#include "cmsis_os.h"

uint32_t YRT_Get_Time(void) {
    return osKernelSysTick();
}


YRT_Status_t YRT_Delay(uint32_t period) {
    osDelay(period);
    return YRT_OK;
}











