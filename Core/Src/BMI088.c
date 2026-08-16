//BMI088.c
//mehmetauditore

#include "BMI088.h"

uint8_t BMI088_Init(BMI088IMU *imu) {
	
	uint8_t txBuf[2];

	if(imu->acc_bwp_osr == 0x00) imu->acc_bwp_osr = 0x0A;
	if(imu->acc_odr == 0x00) imu->acc_odr = 0x05;

	/*Acc soft reset*/
	uint8_t rst = BMI088_RST_CMD;
	txBuf[0] = BMI088_ACC_SOFTRESET; txBuf[1] = rst; 
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);
	imu->delay_ms(50);

	/*Gyro soft reset*/
	txBuf[0] = BMI088_GYR_SOFTRESET; txBuf[1] = rst;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT); 
	imu->delay_ms(50);

	imu->acc[0] = 0.0f;
	imu->acc[1] = 0.0f;
	imu->acc[2] = 0.0f;
	imu->gyr[0] = 0.0f;
	imu->gyr[1] = 0.0f;
	imu->gyr[2] = 0.0f;

	/*
	 * ACCELEROMETER
	 */

	/*Chip ID check*/
	uint8_t chipID;
	HAL_I2C_Mem_Read(imu->I2Chandle, BMI088_ACC_I2C_ADDR, BMI088_ACC_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &chipID, 1, BMI088_I2C_TIMEOUT);

	if (chipID != 0x1E) {
		return 2;
	}

	/*Configure the accelerometer register
	 * [7:4] bits for bandwith of the accelerometer low pass filter
	 * 0x08 for OSR4, 0x09 for OSR2, 0x0A reset value
	 * [3:0] bits for set output data rate ODR
	 * 0x05 for 12.5Hz
	 * 0x06 for 25Hz
	 * 0x07 for 50Hz
	 * 0x08 for 100Hz
	 * 0x09 for 200Hz
	 * 0x0A for 400Hz
	 * 0x0B for 800Hz
	 * 0x0C for 1600Hz
	*/
	uint8_t accConf = ((imu->acc_bwp_osr<<4) | (imu->acc_odr));
	txBuf[0] = BMI088_ACC_CONF; txBuf[1] = accConf;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Configure the accelerometer's range
	 * 0x00 for +-3g
	 * 0x01 for +-6g
	 * 0x02 for +-12g
	 * 0x03 for +-24g
	*/

	uint8_t accRange = imu->acc_range;
	txBuf[0] = BMI088_ACC_RANGE; 
	txBuf[1] = accRange;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	switch (accRange)
	{
		case BMI088_acc_range_3g:
		imu->acc_scale = 0.00089798534f;
		break;

		case BMI088_acc_range_6g:
		imu->acc_scale = 0.00179597069f;
		break;

		case BMI088_acc_range_12g:
		imu->acc_scale = 0.00359194139f;
		break;

		case BMI088_acc_range_24g:
		imu->acc_scale = 0.00718388279f;
		break;
		default:
	}

	/*Configure the input/output pin INT1 and INT2
	 * in register 0x53(INT1_IO_CONF
	 * 5. bit for Enable INT1 as an input pin.
	 * 4. bit for Enable INT1 as an output pin.
	 * 3. bit for Pin behavior 0 for Push-Pull, 1 for Open-Drain.
	 * 2. bit for Active state 0 for Active-low, 1 for Active-high.
	 * it is same for INT2
	*/
	uint8_t intConf = 0x0A;
	txBuf[0] = BMI088_INT1_IO_CONF; txBuf[1] = intConf;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	txBuf[0] = BMI088_INT2_IO_CONF;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Map data Interrup register
	 * 7. bit for Map data ready interrupt to pin INT2
	 * 3. bit for Map data ready interrupt to pin INT1
	*/
	txBuf[0] = BMI088_INT1_INT2_MAP_DATA; txBuf[1] = 0x44;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);
	/*Setting accelerometer to active mode
	 * 0x03 for suspend mode
	 * 0x00 for active mode
	*/
	txBuf[0] = BMI088_ACC_PWR_CONF; txBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);
	/*Switch the accelerometer on
	 * 0x00 Accelerometer off
	 * 0x04 Accelerometer on
	 */
	txBuf[0] = BMI088_ACC_PWR_CTRL; txBuf[1] = 0x04;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_ACC_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*
	 * GYROSCOPE
	 */

	/*Chip ID check*/
	HAL_I2C_Mem_Read(imu->I2Chandle, BMI088_GYR_I2C_ADDR, BMI088_GYR_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &chipID, 1, BMI088_I2C_TIMEOUT);

	if (chipID != 0x0F) {
			return 2;
	}
	/* Setting the Gyroscope range
	 * 0x00 for +-2000 deg/s
	 * 0x01 for +-1000 deg/s
	 * 0x02 for +-500 deg/s
	 * 0x03 for +-250 deg/s
	 * 0x04 for +-125 deg/s
	*/
	uint8_t gyrRange = imu->gyro_range;
	txBuf[0] = BMI088_GYR_RANGE; txBuf[1] = gyrRange;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

		switch (gyrRange)
	{
		case BMI088_gyro_range_2000dps:
		imu->gyro_scale = 2000.0f / 32768.0f;
		break;
		
		case BMI088_gyro_range_1000dps:
		imu->gyro_scale = 1000.0f / 32768.0f;
		break;

		case BMI088_gyro_range_500dps:
		imu->gyro_scale = 500.0f / 32768.0f;
		break;

		case BMI088_gyro_range_250dps:
		imu->gyro_scale = 250.0f / 32768.0f;
		break;
		
		case BMI088_gyro_range_125dps:
		imu->gyro_scale = 125.0f / 32768.0f;
		break;
		default:
	}

	/*Setting the Gyroscope Bandwidth
	 * 0x00 ODR(2000Hz) Filter Bandwidth(532 Hz)
	 * 0x01 ODR(2000Hz) Filter Bandwidth(230 Hz)
	 * 0x02 ODR(1000Hz) Filter Bandwidth(116 Hz)
	 * 0x03 ODR(400Hz) Filter Bandwidth(47 Hz)
	 * 0x04 ODR(200Hz) Filter Bandwidth(23 Hz)
	 * 0x05 ODR(100Hz) Filter Bandwidth(12 Hz)
	 * 0x06 ODR(200Hz) Filter Bandwidth(64 Hz)
	 * 0x07 ODR(100Hz) Filter Bandwidth(32 Hz)
	 */
	uint8_t gyrBandwidth = imu->gyro_bw;
	txBuf[0] = BMI088_GYR_BANDWIDTH; txBuf[1] = gyrBandwidth;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Gyroscope power mode
	 * 0x00 for normal mode
	 * 0x01 for suspend mode
	 * 0x02 for deep suspend mode
	 */
	txBuf[0] = BMI088_GYR_LPM1; txBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Gyroscope Interrupt Control
	 * 0x00 for No data ready interrupt is triggered
	 * 0x80 for Enables the new data interrupt to be trigerred on new data
	 */
	txBuf[0] = BMI088_GYR_INT_CTRL; txBuf[1] = 0x80;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Set the electrical and logical properties of the interrupt pins
	 * 4. bit configure pin4 as Push-pull if it is 0, and Open-drain if it is 1
	 * 3. bit configure pin4's state if it is 0 Active low,if it is 1 Active High
	 * 2. bit configure pin3 as Push-pull if it is 0, and Open-drain if it is 1
	 * 1. bit configure pin3's state if it is 0 Active low,if it is 1 Active High
	 */
	txBuf[0] = BMI088_GYR_INT3_INT4_IO_CONF; txBuf[1] = 0x05;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	/*Map the data ready interrupt pin to one of the interrupt pins INT3 and INT4
	 * 0x00 Data ready interrupt is not mapped to any INT pin
	 * 0x01 Data ready interrupt is mapped to INT3 pin
	 * 0x80 Data ready interrupt is mapped to INT4 pin
	 * 0x81 Data ready interrupt is mapped to INT3 and INT4 pins
	 */
	txBuf[0] = BMI088_GYR_INT3_INT4_IO_MAP; txBuf[1] = 0x81;
	HAL_I2C_Master_Transmit(imu->I2Chandle, BMI088_GYR_I2C_ADDR, txBuf, 2, BMI088_I2C_TIMEOUT);

	return 0;
}

uint8_t BMI088_ReadAcc(BMI088IMU *imu) {
	uint8_t rslt;
	uint8_t rxBuf[6];
	rslt = HAL_I2C_Mem_Read(imu->I2Chandle, BMI088_ACC_I2C_ADDR, BMI088_ACC_DATA, I2C_MEMADD_SIZE_8BIT, rxBuf, 6, BMI088_I2C_TIMEOUT);

	int16_t accX = rxBuf[1];
			accX <<= 8;
			accX |= rxBuf[0];

	int16_t accY = rxBuf[3];
			accY <<= 8;
			accY |= rxBuf[2];

	int16_t accZ = rxBuf[5];
			accZ <<= 8;
			accZ |= rxBuf[4];

	/* Scale (to m/s^2) and re-map axes 
	acc = (reading/sensitivity)* 9.806 = reading * value
	values:	
	3g:  0.00089798534f
	6g:	 0.00179597069f
	12g: 0.00359194139f
	24g: 0.00718388279f
	*/

	imu->acc[0] = accZ * imu->acc_scale;
	imu->acc[1] = accX * imu->acc_scale;
	imu->acc[2] = accY * imu->acc_scale;

	return rslt;
}

uint8_t BMI088_ReadGyr(BMI088IMU *imu)
{
    uint8_t data[6];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        imu->I2Chandle,
        BMI088_GYR_I2C_ADDR,
        BMI088_GYR_DATA,
        I2C_MEMADD_SIZE_8BIT,
        data,
        sizeof(data),
        BMI088_I2C_TIMEOUT
    );

    if (status != HAL_OK)
    {
        return 1U;
    }

    int16_t raw_x =
        (int16_t)(((uint16_t)data[1] << 8) | data[0]);

    int16_t raw_y =
        (int16_t)(((uint16_t)data[3] << 8) | data[2]);

    int16_t raw_z =
        (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    /* GYR_RANGE registerı gerçekten ±2000 dps ise
    const float scale = 2000.0f / 32768.0f; */

    imu->gyr[0] = (float)raw_x * imu->gyro_scale;
    imu->gyr[1] = (float)raw_y * imu->gyro_scale;
    imu->gyr[2] = (float)raw_z * imu->gyro_scale;

    return 0U;
}
