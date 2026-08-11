/**
* @file YRT_JR_DEF.h
* @author Ali Okan Yeşilyurt
* @brief Header file for YRT JR definitions
* @version BETA
* @copyright Copyright (c) 2026
*/
#ifndef YRT_JR_DEF_H
#define YRT_JR_DEF_H

#include <stdint.h>

/**
* @brief CAN TAGS
*/
#define SSC_CAN_ID 0X101
#define MUQU_CAN_ID 0X102
#define PRC_CAN_ID 0X103
// TODO: Kartların adını öğrenince ID'leri güncelle.

#define FLOAT_TO_UINT16(X) (uint16_t)(X * 1000)
#define UINT16_TO_FLOAT(X) (float)(X / 1000.0f)

/**
 * @brief RESULT CODES
 */
typedef enum {
    YRT_JR_OK = 0,
    YRT_JR_ERROR = 1,
    YRT_JR_TIMEOUT = 2,
    YRT_JR_INVALID_PARAM = 3,
    YRT_JR_INIT_ERROR = 4,
    YRT_JR_COMM_ERROR = 5,
    YRT_JR_BARO_ERROR = 6,
    YRT_JR_BARO_HARD_ERROR = 7,
    YRT_JR_IMU_ERROR = 8,
    YRT_JR_IMU_HARD_ERROR = 9,
    YRT_JR_CAN_ERROR = 10,
    YRT_JR_SD_ERROR = 11,
} YRT_JR_Result_t;

/**
 * @brief IMU datas
 */
typedef struct {
    int16_t accel_x;        
    int16_t accel_y;        
    int16_t accel_z;      
    float   accel_z_avg;    
    float   accel_total;    
} YRT_IMU_Data_t;

/**
 * @brief Barometer datas
 */
typedef struct {
    float pressure_current;     
    float pressure_filtered;   
    float pressure_initial;     
    
    float temperature_current; 
    float temperature_filtered;
    
    float altitude_current;     
    float altitude_filtered;    
    float altitude_initial;    
    float altitude_relative;    
    float altitude_max;         
} YRT_Baro_Data_t;

typedef struct {
    float latitude;
    float longitude;
    float altitude;
} YRT_GPS_Data_t;

/**
 * @brief Flight state datas
 */
typedef struct {
    float    vertical_velocity; 
    uint32_t previous_time;    
    uint32_t apogee_time;       
    uint32_t fall_counter;      
} YRT_FlightState_t;

/**
 * @brief Telemetry  packet with pragma 
 */
#pragma pack(push, 1)
typedef struct {
    uint32_t          timestamp_ms; 
    YRT_IMU_Data_t    imu;          
    YRT_Baro_Data_t   baro;        
    YRT_FlightState_t state;
    YRT_GPS_Data_t     gps;        
    uint16_t          crc;          
} YRT_TelePacket_t;
#pragma pack(pop)


/**
 * @brief Conventer unions
 */
typedef union {
    float u32;
    uint8_t u8[4];
} float_to_u8_4;

typedef union
{
    double u64;
    uint8_t u8[8];
} double_to_u8;
//TODO: Bu dönüştürücülerin sayısı ihtiyaca göre artacak.

#endif /* YRT_JR_DEF_H */