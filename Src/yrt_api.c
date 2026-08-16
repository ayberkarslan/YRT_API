#include "yrt_api.h"


#if YRT_IS_IMU_ENABLED

#if YRT_IMU_BNO055_SELECTED
bno055_t bno055;
#elif YRT_IMU_MPU6050_SELECTED

#elif YRT_IMU_BMI088_SELECTED
BMI088IMU bmi088;
#else

#endif
#endif

#if YRT_IS_BARO_ENABLED

#if YRT_BARO_BMP280_SELECTED
BMP280_HandleTypedef bmp280;
#elif YRT_BARO_MS5611_SELECTED

static MS5611_t ms5611_dev;
static I2C_HandleTypeDef *baro_i2c;


static uint8_t ms5611_i2c_write(uint8_t addr, uint8_t *data, uint16_t len) {
    return HAL_I2C_Master_Transmit(baro_i2c, addr, data, len, 100);
}


static uint8_t ms5611_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    return HAL_I2C_Master_Receive(baro_i2c, addr, data, len, 100);
}

#else

#endif
#endif

//LORA
#if YRT_IS_LORA_ENABLED

    static LoRa my_lora;


    //LoRa init
    uint16_t yrt_LoRa_init(const yrt_LoRa_conf_t *config){


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
        my_lora.bandWidth             = config -> bandWidth;       // default = BW_125KHz
        my_lora.crcRate               = config -> crcRate ;          // default = CR_4_5
        my_lora.power                 = config -> power;      // default = 20db
        my_lora.overCurrentProtection = config -> overCurrentProtection;             // default = 100 mA
        my_lora.preamble              = config -> preamble;              // default = 8;

        return LoRa_init(&my_lora);
    }



    // LoRa transmit
    uint8_t yrt_LoRa_Transmit(const uint8_t *data, uint8_t length, uint16_t timeout){

    LoRa_transmit(&my_lora, (uint8_t*)data, length, timeout);

    }


    // LoRa receive
    uint8_t yrt_LoRa_Receive(uint8_t *send_data, uint8_t length, uint16_t timeout){

        LoRa_startReceiving(&my_lora);
        //uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length);
        //şimdilik dursun burası, asıl receive fonksiyonunu ekleyeceğiz
    }


#endif

//IMU
#if YRT_IS_IMU_ENABLED

void yrt_delay(uint32_t period)
{
#if YRT_IS_RTOS_ENABLED
    osDelay(period);
#else
    HAL_Delay(period);
#endif
}

YRT_Status_t YRT_IMU_Config(YRT_IMU_t *yrt_imu)
{

    #if YRT_IMU_BNO055_SELECTED
    //BNO055


    bno055.i2c = yrt_imu->config.hi2c;
    bno055.addr = yrt_imu->config.dev_address;

    #elif YRT_IMU_MPU6050_SELECTED
    //MPU6050

    #elif YRT_IMU_BMI088_SELECTED
    //BMI088

    bmi088.I2Chandle = yrt_imu->config.hi2c;
    bmi088.acc_bwp_osr = yrt_imu->config.acc_bwp_osr;
    bmi088.acc_odr = yrt_imu->config.acc_odr;
    bmi088.acc_range = yrt_imu->config.acc_range;
    bmi088.gyro_bw = yrt_imu->config.gyro_filter_bw;
    bmi088.gyro_range = yrt_imu->config.gyro_range;
    bmi088.delay_ms = yrt_delay;

    #else

    //Farklı sensör varsa burayı doldur
    #endif

    return YRT_OK;
}
    
YRT_Status_t YRT_IMU_Init(YRT_IMU_t *yrt_imu)
{

    #if YRT_IMU_BNO055_SELECTED 
    //BNO055

    YRT_IMU_Config(imu_config);

    if(bno055_init(&bno055) != BND_OK)
    {
        return YRT_IMU_ERR;
    }

    yrt_delay(100);
    if (bno055_set_unit(&bno055, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS, BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG) != BNO_OK)
    {
        return YRT_IMU_ERR;
    }

    bno055_set_pwr_mode(&bno055, imu_config->config.power_mod);
    bno055_acc_conf(&bno055, yrt_imu->config.acc_range, yrt_imu->config.acc_bandwidth, yrt_imu->config.acc_mode);
    bno055_gyr_conf(&bno055, yrt_imu->config.gyro_range, yrt_imu->config.gyro_bandwidth, yrt_imu->config.gyro_mode);
    bno055_mag_conf(&bno055, yrt_imu->config.mag_rate, yrt_imu->config.mag_mode, yrt_imu->config.mag_power_mode);

    #elif YRT_IMU_MPU6050_SELECTED 
    //MPU6050

    #elif YRT_IMU_BMI088_SELECTED 
    //BMI088

    YRT_IMU_Config(yrt_imu);
    if (BMI088_Init(&bmi088) != HAL_OK)
    {
    	return YRT_IMU_ERR;
    }

    #else

    //Farklı sensör varsa burayı doldur
    #endif

    return YRT_OK;
}

YRT_Status_t YRT_IMU_Read(YRT_IMU_t *yrt_imu)
{

    #if YRT_IMU_BNO055_SELECTED
    //BNO055
    
    bno055_vec3_t acc;
    bno055_vec3_t lin_acc;
    bno055_vec3_t gyro;
    bno055_vec3_t mag;
    bno055_vec3_t grav;
    bno055_euler_t euler;
    bno055_vec4_t quat;

    bno055_acc(&bno055, &acc);
    bno055_linear_acc(&bno055, &lin_acc);
    bno055_gyro(&bno055, gyro);
    bno055_mag(&bno055, &mag);
    bno055_grav(&bno055, &grav);
    bno055_euler(&bno055, &euler);
    bno055_quaternion(&bno055, &quat);

    acc.x = yrt_imu->data.accel.x;
    acc.y = yrt_imu->data.accel.y;
    acc.z = yrt_imu->data.accel.z;

    lin_acc.x = yrt_imu->data.lineer_accel.x;
    lin_acc.y = yrt_imu->data.lineer_accel.y;
    lin_acc.z = yrt_imu->data.lineer_accel.z;

    gyro.x = yrt_imu->data.gyro.x;
    gyro.y = yrt_imu->data.gyro.y;
    gyro.z = yrt_imu->data.gyro.z;

    mag.x = yrt_imu->data.mag.x;
    mag.y = yrt_imu->data.mag.y;
    mag.z = yrt_imu->data.mag.z;

    grav.x = yrt_imu->data.grav.x;
    grav.y = yrt_imu->data.grav.y;
    grav.z = yrt_imu->data.grav.z;

    euler.roll = yrt_imu->data.euler.x;
    euler.pitch = yrt_imu->data.euler.y;
    euler.yaw = yrt_imu->data.euler.z;

    quat.x = yrt_imu->data.quaternion.x;
    quat.y = yrt_imu->data.quaternion.y;
    quat.z = yrt_imu->data.quaternion.z;
    quat.w = yrt_imu->data.quaternion.w;

    #elif YRT_IMU_MPU6050_SELECTED
    //MPU6050

    #elif YRT_IMU_BMI088_SELECTED
    //BMI088

    if (BMI088_ReadAcc(&bmi088) != HAL_OK)
    {
    	return YRT_IMU_ERR;
    }

    yrt_imu->data.accel.x = bmi088.acc[0];
    yrt_imu->data.accel.y = bmi088.acc[1];
    yrt_imu->data.accel.z = bmi088.acc[2];
    
    if (BMI088_ReadGyr(&bmi088) != HAL_OK)
    {
    	return YRT_IMU_ERR;
    }

    yrt_imu->data.gyro.x = bmi088.gyr[0];
    yrt_imu->data.gyro.y = bmi088.gyr[1];
    yrt_imu->data.gyro.z = bmi088.gyr[2];

    #else

    //Farklı sensör varsa burayı doldur
    #endif

    return YRT_OK;
}

#endif

//BARO
#if YRT_IS_BARO_ENABLED
    
YRT_Status_t YRT_Baro_Config(YRT_BARO_t* baro_config)
{

    #if YRT_BARO_MS5611_SELECTED
    //MS5611

	if(baro_config->config.lp_coef == 0.0f) baro_config->config.lp_coef = 1.0f;

	ms5611_dev.osr_value = baro_config->config.osr;
	ms5611_dev.filter_value = baro_config->config.lp_coef;
    baro_i2c = (I2C_HandleTypeDef *)baro_config->config.hi2c;


    ms5611_dev.i2c_send = ms5611_i2c_write;
    ms5611_dev.i2c_receive = ms5611_i2c_read;
    ms5611_dev.delay_ms = yrt_delay;

    return YRT_OK;

    #elif YRT_BARO_BMP280_SELECTED
    //BMP280
    bmp280.i2c = baro_config->config.hi2c;
    bmp280.addr = baro_config->config.dev_address;
    bmp280.id = 0x58;
    bmp280.params.mode = baro_config->config.baro_mode;
    bmp280.params.filter = baro_config->config.filter;
    bmp280.params.oversampling_pressure = baro_config->config.osr_press;
    bmp280.params.oversampling_temperature = baro_config->config.osr_temp;
    bmp280.params.standby = baro_config->config.standby;

    return YRT_OK;

    #else

    //Farklı sensör varsa burayı doldur
    #endif

    return YRT_BARO_ERR;
}
    
YRT_Status_t YRT_Baro_Init(YRT_BARO_t* yrt_baro)
{
    YRT_Status_t rslt = YRT_BARO_ERR;

    #if YRT_BARO_MS5611_SELECTED
    //MS5611

    if ((rslt = YRT_Baro_Config(yrt_baro)) != YRT_OK)
    {
    	return rslt;
    }

    if(MS5611_Init(&ms5611_dev) != YRT_OK_KEMAL)
    {
    	return YRT_IMU_ERR;
    }

    if(YRT_Baro_Read(yrt_baro) != YRT_OK)
    {
    	return YRT_IMU_ERR;
    }

    MS5611_SetReferencePressure(&ms5611_dev);

    #elif YRT_BARO_BMP280_SELECTED
    //BMP280

    if(YRT_Baro_Config(yrt_baro) != true)
    {
        return rslt;
    }

    if (!bmp280_init(&bmp280, &bmp280.params))
    {
		return YRT_BARO_ERR;
	}

    if (bmp280_read_float(&bmp280, NULL, &(yrt_baro->data.ground_pressure), NULL) != true)
    {
    	return YRT_BARO_ERR;
    }
    
    #else

    //Farklı sensör varsa burayı doldur
    #endif

    return YRT_OK;
}

YRT_Status_t YRT_Baro_Read(YRT_BARO_t* yrt_baro)
{

    #if YRT_BARO_MS5611_SELECTED
    //MS5611

    if (MS5611_ReadRawData(&ms5611_dev) == YRT_OK_KEMAL) {
        MS5611_Calculate(&ms5611_dev);

        yrt_baro->data.altitude		= ms5611_dev.filtered_altitude;
        yrt_baro->data.pressure		= ms5611_dev.pressure;
        yrt_baro->data.temperature	= ms5611_dev.temperature;
        return YRT_OK;
    }
    return YRT_BARO_ERR;

    #elif YRT_BARO_BMP280_SELECTED
    //BMP280

    if (bmp280_read_float(&bmp280, &(yrt_baro->data.temperature), &(yrt_baro->data.pressure), NULL) != true)
    {
    	return YRT_BARO_ERR;
    }
    yrt_baro->data.altitude = YRT_CalculateAltitude(yrt_baro->data.ground_pressure, yrt_baro->data.pressure, yrt_baro->data.temperature);

    #else

    //Farklı sensör varsa burayı doldur
    #endif

    return YRT_OK;
}

float YRT_CalculateAltitude(float p, float pi, float ti)
{
    const float R = 287.05;  // Specific gas constant for dry air (J/(kg*K))
    const float L = 0.0065;  // Temperature lapse rate (K/m)
    const float g = 9.80665; // Acceleration due to gravity (m/s^2)
    // #define p11 214.225  // Standard pressure at 11 km (Pa)

    const float MOLAR_MASS = 0.0289644;             // Molar mass of Earth's air in kg/mol
    const float UNIVERSAL_GAS_CONSTANT = 8.3144598; // Universal gas constant in J/(mol·K)

    float alt = (ti + 273.15) / L * (1 - pow(p / pi, R * L / g));

    // Calculate altitude above 11km
    // temperature set to -56.5c(216.65K) as simulations shows this in 11km
    if (alt > 11000)
    {
        // calculate 11km pressure. It not constant due to ti
        const float exponent = (g * MOLAR_MASS) / (UNIVERSAL_GAS_CONSTANT * L);
        const float p11 = pi * pow((1 - (L * 11000) / (ti + 273.15)), exponent);

        // float T = (ti + 273.15 ) - L * 11000;
        // float p11 = pi * pow((T / (ti + 273.15 )), (-g * M_E / (R * L)));
        alt = 11000 + (R * 216.65 / g) * log(p11 / p);
    }
    return alt;
}

#endif
