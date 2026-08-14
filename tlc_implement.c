#include "yrt_api.h"
#include "LoRa.h"
#include <stdint.h>
#include "stm32f4xx_hal.h" // HAL katmanı
#include "tlc_implement.h"



//LoRa pin configurations
#define LoRa_NSS_GPIO_Port             GPIOA
#define LoRa_NSS_Pin                   GPIO_PIN_4
#define LoRa_RESET_GPIO_Port           GPIOB
#define LoRa_RESET_Pin                 GPIO_PIN_0
#define LoRa_DIO0_GPIO_Port            GPIOC
#define LoRa_DIO0_Pin                  GPIO_PIN_4

//LoRa configurations
#define LoRa_frequency                 433
#define LoRa_spreadingFactor           SF_7
#define LoRa_bandWidth                 BW_125KHz
#define LoRa_crcRate                   CR_4_5
#define LoRa_power                     POWER_11db // SX1278'in desteklediği en düşük güç seviyesi (Yaklaşık 2 dBm, pil dostu)
#define LoRa_overCurrentProtection     100
#define LoRa_preamble                  10





// cubemx'in oluşturduğu spi donanım pini
extern SPI_HandleTypeDef hspi1; 
extern UART_HandleTypeDef huart4;    
extern uint16_t init_durumu;



    
    // kartı hazırladığımız yer
    void TLC_Board_Init(void) {
        
        // apideki structu burada inşa ediyoruz.
        yrt_LoRa_conf_t myLoRa;
        YRT_GPS_Config_t myGPS;
        myGPS.huart = &huart4;
    
        

    myLoRa.frequency             = LoRa_frequency;             // default = 433 MHz
    myLoRa.spredingFactor        = LoRa_spreadingFactor;       // default = SF_7
    myLoRa.bandWidth             = LoRa_bandWidth;             // default = BW_125KHz
    myLoRa.crcRate               = LoRa_crcRate;               // default = CR_4_5
    myLoRa.power                 = LoRa_power;                 // default = 20db
    myLoRa.overCurrentProtection = LoRa_overCurrentProtection; // default = 100 mA
    myLoRa.preamble              = LoRa_preamble;   
        
    myLoRa.nss_port              = LoRa_NSS_GPIO_Port;
    myLoRa.nss_pin               = LoRa_NSS_Pin;
    myLoRa.reset_port            = LoRa_RESET_GPIO_Port;
    myLoRa.reset_pin             = LoRa_RESET_Pin;
    myLoRa.dio0_port             = LoRa_DIO0_GPIO_Port;
    myLoRa.dio0_pin              = LoRa_DIO0_Pin;
    myLoRa.hspi                  = &hspi1;
        
        

    //Initler
        yrt_LoRa_init(&myLoRa);
        YRT_GPS_Init(&myGPS);
        

         
    init_durumu = yrt_LoRa_init(&myLoRa);
    
    }