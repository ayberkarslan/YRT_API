//BMI088.h

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "stdint.h"

#define GFORCE 9.80665

#define BMI088_ACC_I2C_ADDR (0x18 << 1)
#define BMI088_GYR_I2C_ADDR (0x68 << 1)
#define BMI088_I2C_TIMEOUT 100

#define BMI088_ACC_CHIP_ID        0x00
#define BMI088_ACC_ERR_REG        0x02
#define BMI088_ACC_STATUS         0x03
#define BMI088_ACC_DATA           0x12
#define BMI088_TEMP_DATA          0x22
#define BMI088_ACC_CONF           0x40
#define BMI088_ACC_RANGE          0x41
#define BMI088_INT1_IO_CONF       0x53
#define BMI088_INT2_IO_CONF       0x54
#define BMI088_INT1_INT2_MAP_DATA 0x58
#define BMI088_ACC_SELF_TEST      0x6D
#define BMI088_ACC_PWR_CONF       0x7C
#define BMI088_ACC_PWR_CTRL       0x7D
#define BMI088_ACC_SOFTRESET      0x7E

#define BMI088_RST_CMD			  0xB6

#define BMI088_GYR_CHIP_ID         		0x00
#define BMI088_GYR_DATA             	0x02
#define BMI088_GYR_INT_STAT_1			0x0A
#define BMI088_GYR_RANGE            	0x0F
#define BMI088_GYR_BANDWIDTH        	0x10
#define BMI088_GYR_LPM1              	0x11
#define BMI088_GYR_SOFTRESET         	0x14
#define BMI088_GYR_INT_CTRL          	0x15
#define BMI088_GYR_INT3_INT4_IO_CONF 	0x16
#define BMI088_GYR_INT3_INT4_IO_MAP  	0x18
#define BMI088_GYR_SELF_TEST		  	0x3C

typedef enum {
	BMI088_acc_bwp_osr2 = 0x08,
	BMI088_acc_bwp_osr4 = 0x09,
	BMI088_acc_bwp_osr_normal = 0x0A
} bmi088_imu_acc_bwp_osr_t;

typedef enum {
	BMI088_acc_odr12_5 = 0x05,
	BMI088_acc_odr25 = 0x06,
	BMI088_acc_odr50 = 0x07,
	BMI088_acc_odr100 = 0x08,
	BMI088_acc_odr200 = 0x09,
	BMI088_acc_odr400 = 0x0A,
	BMI088_acc_odr800 = 0x0B,
	BMI088_acc_odr1600 = 0x0C
} bmi088_imu_acc_odr_t;

typedef enum {
	BMI088_acc_range_3g = 0x00,
	BMI088_acc_range_6g = 0x01,
	BMI088_acc_range_12g = 0x02,
	BMI088_acc_range_24g = 0x03
} bmi088_imu_acc_range_t;

typedef enum {
	BMI088_gyro_range_2000dps = 0x00,
	BMI088_gyro_range_1000dps = 0x01,
	BMI088_gyro_range_500dps = 0x02,
	BMI088_gyro_range_250dps = 0x03,
	BMI088_gyro_range_125dps = 0x04
} bmi088_imu_gyro_range_t;

typedef enum {
	BMI088_gyro_filter_bandwidth_532 = 0x00,
	BMI088_gyro_filter_bandwidth_230 = 0x01,
	BMI088_gyro_filter_bandwidth_116 = 0x02,
	BMI088_gyro_filter_bandwidth_47 = 0x03,
	BMI088_gyro_filter_bandwidth_23 = 0x04,
	BMI088_gyro_filter_bandwidth_12 = 0x05,
	BMI088_gyro_filter_bandwidth_64 = 0x06,
	BMI088_gyro_filter_bandwidth_32 = 0x07
} bmi088_imu_gyro_bw_t;

typedef struct {
	I2C_HandleTypeDef *I2Chandle;
	bmi088_imu_acc_bwp_osr_t acc_bwp_osr;
	bmi088_imu_acc_odr_t acc_odr;
	bmi088_imu_acc_range_t acc_range;
	bmi088_imu_gyro_range_t gyro_range;
	bmi088_imu_gyro_bw_t gyro_bw;
	GPIO_TypeDef *intAccPinBank;
	uint16_t intAccPin;
	GPIO_TypeDef *intGyrPinBank;
	uint16_t intGyrPin;
	float acc_scale;
	float gyro_scale;
	float acc[3];
	float gyr[3];

	void (*delay_ms)(uint32_t period);
} BMI088IMU;

uint8_t BMI088_Init(BMI088IMU *imu);
uint8_t BMI088_ReadAcc(BMI088IMU *imu);
uint8_t BMI088_ReadGyr(BMI088IMU *imu);

#endif /* INC_BMI088_H_ */
