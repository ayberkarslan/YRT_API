/**
* @file YRT_DEF.h
* @author Ali Okan Yeşilyurt
* @brief Header file for YRT definitions
* @version BETA
* @copyright Copyright (c) 2026
*/
#ifndef YRT_DEF_H
#define YRT_DEF_H

#include <stdint.h>

/**
* @brief CAN TAGS
*/
#define SSC_CAN_ID 0X101
#define MUQU_CAN_ID 0X102
#define PRC_CAN_ID 0X103
// TODO: Kartların adını öğrenince ID'leri güncelle.

#define FLOAT_TO_UINT16(X) ((uint16_t)((X) * 1000.0f))
#define UINT16_TO_FLOAT(X) ((float)((X) / 1000.0f))

/**
 * @brief RESULT CODES
 */
typedef enum {
    YRT_OK = 0,
    YRT_ERROR = 1,
    YRT_TIMEOUT = 2,
    YRT_INVALID_PARAM = 3,
    YRT_INIT_ERROR = 4,
    YRT_COMM_ERROR = 5,
    YRT_BARO_ERROR = 6,
    YRT_BARO_HARD_ERROR = 7,
    YRT_IMU_ERROR = 8,
    YRT_IMU_HARD_ERROR = 9,
    YRT_CAN_ERROR = 10,
    YRT_JR_SD_ERROR = 11,
} YRT_Status_t;

/**
 * @brief IMU datas
 */
typedef struct {
    float   accel_x;        /* m/s^2 */
    float   accel_y;        /* m/s^2 */
    float   accel_z;        /* m/s^2 */
    float   accel_z_avg;    /* m/s^2, filtreli */
    float   accel_total;    /* m/s^2, bileske */
} YRT_Flight_IMU_t;

typedef struct {
    float x; 
    float y; 
    float z; 
} YRT_Flight_Gyro_t;

/**
 * @brief Barometer datas
 */
typedef struct {
    float pressure_current;     
    float pressure_start;
    float pressure_filtered;   
    float pressure_initial;     
    
    float temperature_current; 
    float temperature_filtered;
    
    float altitude_current;     
    float altitude_filtered;    
    float altitude_initial;    
    float altitude_relative; 
    float altitude_start;   
    float altitude_max;         
} YRT_Flight_Baro_t;

typedef struct {
    float latitude;
    float longitude;
    float altitude;
} YRT_Flight_GPS_t;

/**
 * @brief Flight state datas
 */
typedef struct {
    float    vertical_velocity; 
    uint32_t previous_time;    
    uint32_t landing_time;       
    uint32_t fall_counter;      
} YRT_FlightSpec_t;

/**
 * @brief Telemetry  packet with pragma 
 */
#pragma pack(push, 1)
typedef struct {
    uint32_t          timestamp_ms; 
    YRT_Flight_IMU_t    imu;          
    YRT_Flight_Baro_t   baro;        
    YRT_FlightSpec_t state;
    YRT_Flight_GPS_t     gps;        
    YRT_Flight_Gyro_t euler;
    uint16_t          crc;          
} YRT_FlightData_t;
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

#endif /* YRT_DEF_H */
