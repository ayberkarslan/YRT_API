#include "ms5611_yrt.h"

/*YRT_STATUS_t MS5611_Delay(int ms){
#if IS_RTOS_ENABLED
osDelay(ms);
#else
HAL_Delay(ms);
#endif
     
}
*/


// artık hata kodu döndürecekleri için fonksiyon tipini değiştirdik
YRT_STATUS_KEMAL_t MS5611_Init(MS5611_t *sensor_data){

      
  uint8_t cmd; //sensöre göndereceğimiz komutlar için
  uint8_t buffer[2]; // kalibrasyon verilerini tutacak buffer


//  sensor_data->osr_value = MS5611_OSR_4096; // varsayılan olarak hassas ama yavaş
//  sensor_data->filter_value= 1.0f; // varsayılan olarak low pass kapalı ama kullanıcı isterse katsayıyı değiştirerek aktif edebilir.
  sensor_data->reference_pressure = 1013.25f; //irtifa için basınç refernası (deniz seviyesi)

  cmd = MS5611_COMMAND_RESET; //çnce kalbirasyon verilerini PROM'dan çekeceğiz, bunun için sensöre reset atacağız.






  if ( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
      return YRT_ERROR_KEMAL;
  } // komutu gönderdik

  sensor_data->delay_ms(3);
 // MS5611_Delay(3); //sensörün en az 2.8ms beklemesi lazım 


  //şimdi sırayla kalibrasyon verilerini alacağız

  //c1
  cmd = MS5611_COMMAND_C1;

  if ( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0 ){

      return YRT_ERROR_KEMAL;

  }//yine 1 byte c1 alma isteği


 

  if(  sensor_data->i2c_receive(MS5611_I2C_ADDR, buffer, 2) != 0){ // c1'i aldık

      return YRT_ERROR_KEMAL;

  }
  sensor_data->C1 = (buffer[0] << 8) | buffer[1]; // 2 byte'lık c1'i buffera atadık.

  //c2
  cmd = MS5611_COMMAND_C2;
  if( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
      return YRT_ERROR_KEMAL;
  }
  if(  sensor_data->i2c_receive(MS5611_I2C_ADDR, buffer, 2) != 0){
    return YRT_ERROR_KEMAL;
  }
  sensor_data->C2 = (buffer[0] << 8) | buffer[1];
    
  //c3
  cmd = MS5611_COMMAND_C3;
  if( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
        return YRT_ERROR_KEMAL;
  }
  
  if(  sensor_data->i2c_receive(MS5611_I2C_ADDR, buffer, 2) != 0){
    return YRT_ERROR_KEMAL;
  }
  sensor_data->C3 = (buffer[0] << 8) | buffer[1];

  //c4
  cmd = MS5611_COMMAND_C4;
  if( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
    return YRT_ERROR_KEMAL;
  } 
  if(  sensor_data->i2c_receive(MS5611_I2C_ADDR, buffer, 2) != 0){
    return YRT_ERROR_KEMAL;
  } 
  sensor_data->C4 = (buffer[0] << 8) | buffer[1];

  //c5
  cmd = MS5611_COMMAND_C5;
  if( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
    return YRT_ERROR_KEMAL;
  }  
  if(  sensor_data->i2c_receive(MS5611_I2C_ADDR, buffer, 2) != 0){
    return YRT_ERROR_KEMAL;
  }
  sensor_data->C5 = (buffer[0] << 8) | buffer[1];

  //c6
  cmd = MS5611_COMMAND_C6;
  if( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
    return YRT_ERROR_KEMAL;
  }
  if(  sensor_data->i2c_receive(MS5611_I2C_ADDR, buffer, 2) != 0){
    return YRT_ERROR_KEMAL;
  }
  sensor_data->C6 = (buffer[0] << 8) | buffer[1];


  return YRT_OK_KEMAL;
}


// artık hata kodu döndürecekleri için fonksiyon tipini değiştirdik
YRT_STATUS_KEMAL_t MS5611_ReadRawData( MS5611_t *sensor_data){

uint8_t buffer[3];
uint8_t cmd;

/* d1 oku dedik burada yani sensör ham basıncı ölçecek ve bunu adc registerine koyacak 
ama ek bir adım olarak bu sakladığı veriyi çekip buffera koyacağız */
cmd = 0x40 + sensor_data->osr_value; //osr değerini de seçtiriyoruz artık
if ( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
  return YRT_ERROR_KEMAL;
}


//değişken osr değerleri için değişken bekleme süreleri seçtik
   switch(sensor_data->osr_value) {
                case MS5611_OSR_4096: sensor_data->delay_ms(10); break;
                case MS5611_OSR_2048: sensor_data->delay_ms(5);  break;
                case MS5611_OSR_1024: sensor_data->delay_ms(3);  break;
                case MS5611_OSR_512:  sensor_data->delay_ms(2);  break;
                case MS5611_OSR_256:  sensor_data->delay_ms(1);  break;
		default: sensor_data->delay_ms(10); break;
		}


//ek adım bu
cmd= MS5611_COMMAND_ADC;
if( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
  return YRT_ERROR_KEMAL;
}
if(sensor_data->i2c_receive(MS5611_I2C_ADDR, buffer, 3) != 0){
  return YRT_ERROR_KEMAL;
}
sensor_data->D1 = (buffer[0] << 16) | (buffer[1]<<8) | buffer[2];

//şimdi ham sıcaklık 
cmd = 0x50 + sensor_data->osr_value;
if( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
  return YRT_ERROR_KEMAL;
}
//değişken osr değerleri için değişken bekleme süreleri seçtik
//MS5611_Delay(10);
   switch(sensor_data->osr_value) {
                case MS5611_OSR_4096: sensor_data->delay_ms(10); break;
                case MS5611_OSR_2048: sensor_data->delay_ms(5);  break;
                case MS5611_OSR_1024: sensor_data->delay_ms(3);  break;
                case MS5611_OSR_512:  sensor_data->delay_ms(2);  break;
                case MS5611_OSR_256:  sensor_data->delay_ms(1);  break;
		default: sensor_data->delay_ms(10); break;
		}

cmd = MS5611_COMMAND_ADC;
if( sensor_data->i2c_send(MS5611_I2C_ADDR, &cmd, 1) != 0){
  return YRT_ERROR_KEMAL;
}
if(sensor_data->i2c_receive(MS5611_I2C_ADDR, buffer, 3) != 0){
  return YRT_ERROR_KEMAL;
}
sensor_data->D2 = (buffer[0]<<16) | (buffer[1]<<8) | buffer[2];


return YRT_OK_KEMAL;

}


void MS5611_Calculate(MS5611_t *sensor_data){

int32_t TEMP; //datasheette verilenlere göre
int32_t dT;
int64_t OFF;
int64_t SENS;
int32_t P;

int64_t T2;
int64_t OFF2;
int64_t SENS2;




dT= sensor_data->D2 - ((uint32_t)sensor_data->C5*256);

TEMP=2000 + (((int64_t)dT*sensor_data->C6)>>23); // >>23 yerine direkt sayıyı da yazabilirdik (8388608) ama bunun yerine ikilik sistemde 23 bit sağa da kaydırırsak aynı hesaba gelir.

OFF=((int64_t)sensor_data->C2<<16) + (((int64_t)sensor_data->C4*dT)>>7);

SENS=((int64_t)sensor_data->C1<<15) + (((int64_t)sensor_data->C3*dT)>>8);

//(D1 * SENS / 2^21 - OFF) / 2^15


if(TEMP<2000){

       
    T2=((int64_t)dT*dT)>>31;
OFF2 = 5*(((int64_t)TEMP-2000)*(TEMP-2000)) / 2;
SENS2= 5*(((int64_t)TEMP-2000)*(TEMP-2000)) / 4;


   if(TEMP<-1500){

 OFF2=OFF2 + 7*(((int64_t)TEMP+1500)*(TEMP+1500));
 SENS2=SENS2 + 11*(((int64_t)TEMP+1500)*(TEMP+1500))/2;
   }



TEMP= TEMP-T2;
OFF = OFF - OFF2;
SENS = SENS - SENS2;

}

P = ((((int64_t)sensor_data->D1*SENS)>>21) - OFF) >> 15;

sensor_data->temperature = (float)TEMP /100.0f; // derece C'ye çevirdik küsüratıyla
sensor_data->pressure = (float)P /100.0f; //mbara çevirdik
sensor_data->altitude = 44330.0f * (1.0f-pow((sensor_data->pressure / sensor_data->reference_pressure),0.190263f));

 sensor_data->filtered_altitude = (sensor_data->filtered_altitude * (1.0f - sensor_data->filter_value)) + (sensor_data->altitude * sensor_data->filter_value);

}



void MS5611_SetReferencePressure(MS5611_t *sensor_data){

sensor_data->reference_pressure = sensor_data->pressure;
}


