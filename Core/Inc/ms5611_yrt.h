//ayberkauditore

#ifndef MS5611_YRT_H
#define MS5611_YRT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <math.h> //basınçtan irtifa hesaplarken lazım olacak




#define MS5611_I2C_ADDR  (0x77 << 1) // ama eğer csb vddye bağlıysa adres 0x76

#define MS5611_COMMAND_RESET   0x1E
#define MS5611_COMMAND_C1 0xA2
#define MS5611_COMMAND_C2 0xA4
#define MS5611_COMMAND_C3 0xA6
#define MS5611_COMMAND_C4 0xA8
#define MS5611_COMMAND_C5 0xAA
#define MS5611_COMMAND_C6 0xAC

/*  BUNLARA GEREK YOK ARTIK KULLANICIYA SEÇTİRDİK OSR'YI
#define MS5611_COMMAND_D1  0x48 // ham basınç,D1 dedik  okuma emri
#define MS5611_COMMAND_D2  0x58 // sıcaklık, D2 dedik okuma emri
*/

#define MS5611_COMMAND_ADC 0x00 // adcden gelen verileri çekme emri

typedef enum{
YRT_OK_KEMAL, // 0, her şey yolunda
YRT_ERROR_KEMAL, //1, sensör ya da i2c bağlantı hatası

}YRT_STATUS_KEMAL_t;


/*burada direkt D1 ve D2 olarak ayrı ayrı yazabilirdik iki enum'a ama D1 0x40'dan;
 D2 ise 0x50'den başlıyor sonrasında her OSR değeri artışı için +0x02 artış gösteriyor
  bunları en son toplayacağız. (D1 için 0x40 ile D2 için 0x50 ile)*/
typedef enum {
        MS5611_OSR_256  = 0x00, // en hızlı, az hassasiyet
        MS5611_OSR_512  = 0x02,
        MS5611_OSR_1024 = 0x04,
        MS5611_OSR_2048 = 0x06,
        MS5611_OSR_4096 = 0x08  // en yavaş, en hassas
}MS5611_OSR_t;


typedef struct {

        uint16_t C1;
        uint16_t C2;
        uint16_t C3;
        uint16_t C4;
        uint16_t C5;
        uint16_t C6;

        uint32_t D1;
        uint32_t D2;

        float temperature;
        float pressure;


        MS5611_OSR_t osr_value;

        float reference_pressure;
        float altitude;


        float filter_value;
        float filtered_altitude;

        uint8_t i2c_adress;

        uint8_t (*i2c_send)(uint8_t device_address, uint8_t *data, uint16_t length);
        
        uint8_t (*i2c_receive)(uint8_t device_address, uint8_t *data, uint16_t length);

        void (*delay_ms)(uint32_t period);


}MS5611_t;

YRT_STATUS_KEMAL_t MS5611_Init( MS5611_t *sensor_data);

YRT_STATUS_KEMAL_t MS5611_ReadRawData(MS5611_t *sensor_data);

void MS5611_Calculate(MS5611_t *sensor_data);

void MS5611_SetReferencePressure(MS5611_t *sensor_data);

#endif
