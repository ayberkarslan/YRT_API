/**
 * @file    yrt_daq_api.c
 * @brief   YRT STARDUSTv2 DAQ Kartı Fonksiyon İşlemleri
 * @details Modüler yapıdaki alt donanım görevlerini yürütür.
 */

#include "yrt_daq_api.h"
#include <string.h>

#if YRT_SYSTEM_ENABLE == 1

/* ==================================================================== */
/* 1. SENSÖR MODÜLÜ KODLARI                                             */
/* ==================================================================== */
#if YRT_USE_SENSOR_MODULE == 1

static I2C_HandleTypeDef *daq_i2c;

YRT_Status_t YRT_DAQ_API_Init_Sensors(I2C_HandleTypeDef *hi2c_ptr) {
    if (hi2c_ptr == NULL) return YRT_ERROR_PARAM;
    daq_i2c = hi2c_ptr;

    if (ADS1115_Init(daq_i2c, ADS1115_DATA_RATE_128, ADS1115_PGA_TWO) != HAL_OK) {
        return YRT_ERROR;
    }
    return YRT_OK;
}

YRT_Status_t YRT_DAQ_API_Read_PT1_Pressure(float *pressure) {
    if (pressure == NULL) return YRT_ERROR_PARAM;
    float raw_voltage = 0.0f;
    if (ADS1115_readSingleEnded(ADS1115_MUX_AIN1, &raw_voltage) == HAL_OK) {
        *pressure = (raw_voltage * PT_VOLTAGE_TO_BAR_COEF);
        return YRT_OK;
    }
    return YRT_ERROR;
}

YRT_Status_t YRT_DAQ_API_Read_PT2_Pressure(float *pressure) {
    if (pressure == NULL) return YRT_ERROR_PARAM;
    float raw_voltage = 0.0f;
    if (ADS1115_readSingleEnded(ADS1115_MUX_AIN2, &raw_voltage) == HAL_OK) {
        *pressure = (raw_voltage * PT_VOLTAGE_TO_BAR_COEF);
        return YRT_OK;
    }
    return YRT_ERROR;
}

YRT_Status_t YRT_DAQ_API_Read_PT3_Pressure(float *pressure) {
    if (pressure == NULL) return YRT_ERROR_PARAM;
    float raw_voltage = 0.0f;
    if (ADS1115_readSingleEnded(ADS1115_MUX_AIN3, &raw_voltage) == HAL_OK) {
        *pressure = (raw_voltage * PT_VOLTAGE_TO_BAR_COEF);
        return YRT_OK;
    }
    return YRT_ERROR;
}

#endif /* YRT_USE_SENSOR_MODULE */


/* ==================================================================== */
/* 2. VALF VE BİLDİRİM MODÜLÜ KODLARI                                   */
/* ==================================================================== */
#if YRT_USE_VALVE_MODULE == 1

void YRT_DAQ_API_Init_Valves(void) {
    YRT_DAQ_API_IgnitionValve_Close();
    YRT_DAQ_API_ReleaseValve_Close();
    YRT_DAQ_API_Buzzer_Set(1);
    HAL_Delay(100);
    YRT_DAQ_API_Buzzer_Set(0);
}

void YRT_DAQ_API_IgnitionValve_Open(void) { HAL_GPIO_WritePin(IGN_VALVE_PORT, IGN_VALVE_PIN, GPIO_PIN_SET); }
void YRT_DAQ_API_IgnitionValve_Close(void) { HAL_GPIO_WritePin(IGN_VALVE_PORT, IGN_VALVE_PIN, GPIO_PIN_RESET); }
void YRT_DAQ_API_ReleaseValve_Open(void) { HAL_GPIO_WritePin(REL_VALVE_PORT, REL_VALVE_PIN, GPIO_PIN_SET); }
void YRT_DAQ_API_ReleaseValve_Close(void) { HAL_GPIO_WritePin(REL_VALVE_PORT, REL_VALVE_PIN, GPIO_PIN_RESET); }
void YRT_DAQ_API_Buzzer_Set(uint8_t state) { HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void YRT_DAQ_API_SystemError_Alert(void) {
    HAL_GPIO_WritePin(USR_LED1_PORT, USR_LED1_PIN, GPIO_PIN_SET);
    YRT_DAQ_API_Buzzer_Set(1);
}

void YRT_DAQ_API_SystemError_Clear(void) {
    HAL_GPIO_WritePin(USR_LED1_PORT, USR_LED1_PIN, GPIO_PIN_RESET);
    YRT_DAQ_API_Buzzer_Set(0);
}

#endif /* YRT_USE_VALVE_MODULE */


/* ==================================================================== */
/* 3. SD KART MODÜLÜ KODLARI                                            */
/* ==================================================================== */
#if YRT_USE_SDCARD_MODULE == 1

static FATFS yrt_fs;
static FIL yrt_file;
static FRESULT yrt_fres;

YRT_Status_t YRT_SD_API_Init(void) {
    yrt_fres = f_mount(&yrt_fs, "", 1);
    if (yrt_fres != FR_OK) return YRT_ERROR;

    yrt_fres = f_open(&yrt_file, YRT_LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE);
    if (yrt_fres != FR_OK) return YRT_ERROR;

    char header[] = "ZAMAN(ms),PT1(Bar),PT2(Bar),PT3(Bar)\n";
    UINT bytesWrote;
    yrt_fres = f_write(&yrt_file, header, strlen(header), &bytesWrote);
    f_close(&yrt_file);

    if (yrt_fres != FR_OK || bytesWrote == 0) return YRT_ERROR;
    return YRT_OK;
}

YRT_Status_t YRT_SD_API_LogData(uint32_t timestamp_ms, float pt1, float pt2, float pt3) {
    char log_buffer[100];
    UINT bytesWrote;
    snprintf(log_buffer, sizeof(log_buffer), "%lu,%.2f,%.2f,%.2f\n", timestamp_ms, pt1, pt2, pt3);

    yrt_fres = f_open(&yrt_file, YRT_LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE);
    if (yrt_fres != FR_OK) return YRT_ERROR;

    yrt_fres = f_write(&yrt_file, log_buffer, strlen(log_buffer), &bytesWrote);
    f_close(&yrt_file);

    if (yrt_fres != FR_OK || bytesWrote == 0) return YRT_ERROR;
    return YRT_OK;
}

void YRT_SD_API_DeInit(void) {
    f_mount(NULL, "", 0);
}

#endif /* YRT_USE_SDCARD_MODULE */


/* ==================================================================== */
/* 4. CAN BUS HABERLEŞME MODÜLÜ KODLARI                                 */
/* ==================================================================== */
#if YRT_USE_CAN_MODULE == 1

/* Multi-frame (8 byte üstü) paketleme için sabitler.
   Her frame'in 0. byte'ı bir "sequence header": [bit7: son parça mı][bit6:0: sıra no]
   Kalan 7 byte gerçek veri taşır. Bu sayede alıcı taraf parçaları doğru
   sırada birleştirebilir, kayıp/karışık frame'i tespit edebilir. */
#define YRT_SEQ_LAST_FRAME_BIT      0x80u
#define YRT_SEQ_INDEX_MASK          0x7Fu
#define YRT_MULTIFRAME_PAYLOAD_LEN  7u

/* Sequence numarası 7-bit olduğu için (0-127), TX tarafında RAM buffer
   gerekmese de gönderilebilecek gerçek üst sınır: 128 frame x 7 byte = 896 byte.
   Bu, RX tarafındaki YRT_CAN_MAX_MSG_LEN (reassembly buffer boyutu) ile
   KARIŞTIRILMAMALI — TX'in kendi buffer'ı yok, sadece frame sayacı sınırlı. */
#define YRT_CAN_TX_HARD_LIMIT   (128u * YRT_MULTIFRAME_PAYLOAD_LEN)

static CAN_HandleTypeDef *daq_can;

YRT_Status_t YRT_CAN_API_Init(CAN_HandleTypeDef *hcan_ptr) {
    if (hcan_ptr == NULL) return YRT_ERROR_PARAM;
    daq_can = hcan_ptr;

    /* Donanımsal CAN Filtresini Ayarla */
    CAN_FilterTypeDef canFilterConfig = {0};
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000;
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(daq_can, &canFilterConfig) != HAL_OK) return YRT_ERROR;

    /* CAN Modülünü Başlat */
    if (HAL_CAN_Start(daq_can) != HAL_OK) return YRT_ERROR;

    return YRT_OK;
}

/**
 * @brief   Gönderim
 * @details 8 byte'a kadar veriler hiçbir ek header eklenmeden TEK FRAME olarak,
 *          8 byte'ı aşan veriler (örn. 14 byte) otomatik olarak birden fazla
 *          frame'e bölünerek gönderilir. Kullanıcı uzunluk konusunda hiçbir
 *          şey düşünmeden bu fonksiyonu çağırabilir.
 */
YRT_Status_t YRT_CAN_API_Transmit(uint32_t can_id, const uint8_t *data, uint16_t length, uint32_t timeout) {
    /* NOT: Gönderim tarafı veriyi parça parça, kaynak pointer'dan doğrudan okuyup
       yolladığı için (RAM'de tüm veriyi tutan büyük bir buffer'a ihtiyaç YOK),
       burada YRT_CAN_MAX_MSG_LEN gibi keyfi bir üst sınır uygulanmaz.
       Gerçek sınır, sequence header'ın 7-bit olmasından gelir:
       128 frame x 7 byte/frame = maksimum 896 byte tek çağrıda gönderilebilir.
       896 byte'ı da aşan bir kullanım durumunda, kendi üst katmanınızda veriyi
       birden fazla YRT_CAN_API_Transmit() çağrısına bölmeniz gerekir. */
    if (data == NULL || length == 0 || length > YRT_CAN_TX_HARD_LIMIT) {
        return YRT_ERROR_PARAM;
    }

    uint32_t start_time = HAL_GetTick();

    CAN_TxHeaderTypeDef txHeader;
    txHeader.StdId = can_id;
    txHeader.ExtId = 0x01;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;

    /* --- TEK FRAME YOLU: 8 byte veya altı, hiçbir header eklenmez --- */
    if (length <= 8) {
        uint8_t payload[8] = {0};
        memcpy(payload, data, length);
        txHeader.DLC = (uint8_t)length;

        while (HAL_CAN_GetTxMailboxesFreeLevel(daq_can) == 0) {
            if ((HAL_GetTick() - start_time) > timeout) return YRT_ERROR_TIMEOUT;
        }

        uint32_t txMailbox;
        if (HAL_CAN_AddTxMessage(daq_can, &txHeader, payload, &txMailbox) != HAL_OK) {
            return YRT_ERROR;
        }
        return YRT_OK;
    }

    /* --- 8 byte üstü veriler için --- */
    uint16_t bytes_sent = 0;
    uint8_t seq = 0;

    while (bytes_sent < length) {
        uint16_t remaining = (uint16_t)(length - bytes_sent);
        uint8_t chunk_len = (remaining > YRT_MULTIFRAME_PAYLOAD_LEN)
                                ? YRT_MULTIFRAME_PAYLOAD_LEN
                                : (uint8_t)remaining;
        uint8_t is_last = (remaining <= YRT_MULTIFRAME_PAYLOAD_LEN) ? 1 : 0;

        uint8_t payload[8] = {0};
        payload[0] = (uint8_t)((seq & YRT_SEQ_INDEX_MASK) | (is_last ? YRT_SEQ_LAST_FRAME_BIT : 0));
        memcpy(&payload[1], &data[bytes_sent], chunk_len);

        txHeader.DLC = (uint8_t)(chunk_len + 1);

        while (HAL_CAN_GetTxMailboxesFreeLevel(daq_can) == 0) {
            if ((HAL_GetTick() - start_time) > timeout) return YRT_ERROR_TIMEOUT;
        }

        uint32_t txMailbox;
        if (HAL_CAN_AddTxMessage(daq_can, &txHeader, payload, &txMailbox) != HAL_OK) {
            return YRT_ERROR;
        }

        bytes_sent += chunk_len;
        seq = (uint8_t)((seq + 1) & YRT_SEQ_INDEX_MASK);
    }

    return YRT_OK;
}

/**
 * @brief   Eski 6-byte'lık telemetri fonksiyonu — artık jenerik Transmit üzerine kurulu.
 *          Geriye dönük uyumluluk için imzası aynı bırakıldı (pt1,pt2,pt3).
 */
YRT_Status_t YRT_CAN_API_SendTelemetry(float pt1, float pt2, float pt3) {
    uint8_t payload[6] = {0};

    uint16_t pt1_scaled = (uint16_t)(pt1 * 100.0f);
    uint16_t pt2_scaled = (uint16_t)(pt2 * 100.0f);
    uint16_t pt3_scaled = (uint16_t)(pt3 * 100.0f);

    payload[0] = (pt1_scaled >> 8) & 0xFF;
    payload[1] = pt1_scaled & 0xFF;
    payload[2] = (pt2_scaled >> 8) & 0xFF;
    payload[3] = pt2_scaled & 0xFF;
    payload[4] = (pt3_scaled >> 8) & 0xFF;
    payload[5] = pt3_scaled & 0xFF;

    /* Supervisor ID: 0x100, 6 byte olduğu için tek frame gider */
    return YRT_CAN_API_Transmit(0x100, payload, sizeof(payload), 50);
}

#endif /* YRT_USE_CAN_MODULE */


/* ==================================================================== */
/* GENEL SİSTEM BAŞLATMA FONKSİYONU                                     */
/* ==================================================================== */
YRT_Status_t YRT_DAQ_API_Init_System(I2C_HandleTypeDef *hi2c_ptr, CAN_HandleTypeDef *hcan_ptr) {
    YRT_Status_t status = YRT_OK;

    #if YRT_USE_SENSOR_MODULE == 1
    if (YRT_DAQ_API_Init_Sensors(hi2c_ptr) != YRT_OK) status = YRT_ERROR;
    #endif

    #if YRT_USE_VALVE_MODULE == 1
    YRT_DAQ_API_Init_Valves();
    #endif

    #if YRT_USE_SDCARD_MODULE == 1
    if (YRT_SD_API_Init() != YRT_OK) status = YRT_ERROR;
    #endif

    #if YRT_USE_CAN_MODULE == 1
    if (YRT_CAN_API_Init(hcan_ptr) != YRT_OK) status = YRT_ERROR;
    #endif

    return status;
}

#endif /* YRT_SYSTEM_ENABLE == 1 */
