/**
 * @file    yrt_daq_api.h
 * @brief   YRT STARDUSTv2 DAQ Kartı Donanım Soyutlama Katmanı (API)
 * @details Modüler yapıda tasarlanmıştır. İhtiyaç duyulmayan alt modüller
 *          anahtarlar üzerinden (0 yapılarak) devre dışı bırakılabilir.
 */

#ifndef YRT_DAQ_API_H
#define YRT_DAQ_API_H

#include "stm32f4xx_hal.h"

/* ==================================================================== */
/* YRT STANDART DURUM BİLDİRİMLERİ                                      */
/* ==================================================================== */
/**
 * @brief YRT sistemleri için evrensel hata ve durum dönüş tipi.
 * @details HAL kütüphanesine olan bağımlılığı izole etmek için kullanılır.
 */
typedef enum {
    YRT_OK = 0,            /**< İşlem başarıyla tamamlandı */
    YRT_ERROR = 1,         /**< İşlem sırasında hata oluştu */
    YRT_ERROR_TIMEOUT = 2, /**< İşlem zaman aşımına uğradı (CAN mailbox vb.) */
    YRT_ERROR_PARAM = 3    /**< Geçersiz parametre (NULL pointer, sınır aşımı vb.) */
} YRT_Status_t;

/* ==================================================================== */
/* MODÜL AKTİF/PASİF ANAHTARLARI (1 = açık, 0 = kapalı)                 */
/* ==================================================================== */
#define YRT_SYSTEM_ENABLE       1
#define YRT_USE_SENSOR_MODULE   1
#define YRT_USE_VALVE_MODULE    1
#define YRT_USE_SDCARD_MODULE   1
#define YRT_USE_CAN_MODULE      1

/* ==================================================================== */

#if YRT_SYSTEM_ENABLE == 1

/* --- SENSÖR MODÜLÜ TANIMLAMALARI --- */
#if YRT_USE_SENSOR_MODULE == 1
#include "ads1115.h"

#define PT_VOLTAGE_TO_BAR_COEF  10.0f

YRT_Status_t YRT_DAQ_API_Init_Sensors(I2C_HandleTypeDef *hi2c_ptr);
YRT_Status_t YRT_DAQ_API_Read_PT1_Pressure(float *pressure);
YRT_Status_t YRT_DAQ_API_Read_PT2_Pressure(float *pressure);
YRT_Status_t YRT_DAQ_API_Read_PT3_Pressure(float *pressure);
#endif /* YRT_USE_SENSOR_MODULE */


/* --- VALF VE BİLDİRİM MODÜLÜ TANIMLAMALARI --- */
#if YRT_USE_VALVE_MODULE == 1
#define IGN_VALVE_PORT   YRT_IGN_VALVE_GPIO_Port
#define IGN_VALVE_PIN    YRT_IGN_VALVE_Pin
#define REL_VALVE_PORT   YRT_REL_VALVE_GPIO_Port
#define REL_VALVE_PIN    YRT_REL_VALVE_Pin
#define USR_LED1_PORT    YRT_USR_LED1_GPIO_Port
#define USR_LED1_PIN     YRT_USR_LED1_Pin
#define BUZZER_PORT      YRT_BUZZER_GPIO_Port
#define BUZZER_PIN       YRT_BUZZER_Pin

void YRT_DAQ_API_Init_Valves(void);
void YRT_DAQ_API_IgnitionValve_Open(void);
void YRT_DAQ_API_IgnitionValve_Close(void);
void YRT_DAQ_API_ReleaseValve_Open(void);
void YRT_DAQ_API_ReleaseValve_Close(void);
void YRT_DAQ_API_Buzzer_Set(uint8_t state);
void YRT_DAQ_API_SystemError_Alert(void);
void YRT_DAQ_API_SystemError_Clear(void);
#endif /* YRT_USE_VALVE_MODULE */


/* --- SD KART LOGLAMA MODÜLÜ TANIMLAMALARI --- */
#if YRT_USE_SDCARD_MODULE == 1
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

#define YRT_LOG_FILENAME "FLIGHT_DATA.csv"

YRT_Status_t YRT_SD_API_Init(void);
YRT_Status_t YRT_SD_API_LogData(uint32_t timestamp_ms, float pt1, float pt2, float pt3);
void         YRT_SD_API_DeInit(void);
#endif /* YRT_USE_SDCARD_MODULE */


/* --- CAN BUS HABERLEŞME MODÜLÜ TANIMLAMALARI --- */
#if YRT_USE_CAN_MODULE == 1

/**
 * @def YRT_CAN_MAX_MSG_LEN
 * @brief SADECE ALICI (RX/reassembly) tarafı için geçerli olan tampon boyutu.
 *        Gönderim (TX) tarafında bu sınır UYGULANMAZ — TX veriyi RAM'de
 *        tutmadan parça parça doğrudan gönderir. Eğer alıcı tarafta da bu
 *        kütüphaneyi kullanıp büyük mesajları birleştirecekseniz, gönderdiğiniz
 *        en büyük mesaj boyutu kadar bu değeri büyütmeniz gerekir.
 */
#define YRT_CAN_MAX_MSG_LEN   32

/**
 * @brief   CAN Donanımı için filtreleri ayarlar ve modülü başlatır.
 * @param   hcan_ptr Kullanılacak CAN hattının adresi (örn: &hcan1)
 * @retval  YRT_Status_t Başarılı ise YRT_OK döner.
 */
YRT_Status_t YRT_CAN_API_Init(CAN_HandleTypeDef *hcan_ptr);

/**
 * @brief   İSTENİLEN UZUNLUKTA (1 byte ile 896 byte arası) veriyi CAN hattına yollar.
 * @details 8 byte'a kadar tek frame olarak, 8 byte'ı aşan veriler otomatik
 *          olarak sequence header'lı parçalara bölünüp (multi-frame) gönderilir.
 *          Örn: 14 byte -> 2 frame, 100 byte -> 15 frame, hepsi otomatik.
 *          Kullanıcı uzunluk için hiçbir ek ayar yapmaz, sadece `length` parametresini
 *          doğru verir; fonksiyon geri kalanını halleder.
 * @param   can_id  Gönderilecek mesajın CAN ID'si
 * @param   data    Gönderilecek verinin dizisi (pointer)
 * @param   length  Verinin toplam uzunluğu, byte cinsinden. Donanımsal üst sınır
 *                  896 byte'tır (sequence numarası 7-bit olduğu için: 128 frame x 7 byte).
 *                  896 byte'ı aşan veriler için bu fonksiyonu birden fazla kez çağırın.
 * @param   timeout mailbox boşalması için beklenecek maksimum süre (ms)
 * @retval  YRT_OK / YRT_ERROR / YRT_ERROR_TIMEOUT / YRT_ERROR_PARAM
 */
YRT_Status_t YRT_CAN_API_Transmit(uint32_t can_id, const uint8_t *data, uint16_t length, uint32_t timeout);

/**
 * @brief   3 basınç değerini (PT1,PT2,PT3) 6 byte'lık pakete sıkıştırıp CAN'a yollar.
 * @details İçeride YRT_CAN_API_Transmit() çağırır (6 byte <= 8 olduğu için tek frame gider).
 *          Bu fonksiyon geriye dönük uyumluluk için korunmuştur.
 * @retval  YRT_Status_t İletim başarılıysa YRT_OK, hata varsa YRT_ERROR
 */
YRT_Status_t YRT_CAN_API_SendTelemetry(float pt1, float pt2, float pt3);

#endif /* YRT_USE_CAN_MODULE */


/* --- GENEL SİSTEM YÖNETİMİ --- */
YRT_Status_t YRT_DAQ_API_Init_System(I2C_HandleTypeDef *hi2c_ptr, CAN_HandleTypeDef *hcan_ptr);

#endif /* YRT_SYSTEM_ENABLE */

#endif /* YRT_DAQ_API_H */
