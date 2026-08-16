#include "YRT_API.h"
#include <string.h>

/**
 * @brief Bekleme. Scheduler baslamadan once osDelay calismaz; sensor init'i
 *        main() icinde yapilirsa bu koruma olmadan beklemeler hic olmaz.
 */
void yrt_delay(uint32_t period)
{
#if YRT_IS_RTOS_ENABLED
    if (osKernelGetState() == osKernelRunning) {
        osDelay(period);
    } else {
        HAL_Delay(period);
    }
#else
    HAL_Delay(period);
#endif
}


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

static MS5611_HandleTypeDef ms5611_dev;
static I2C_HandleTypeDef *baro_i2c;


static uint8_t ms5611_i2c_write(uint8_t dev_addr, uint8_t cmd) {
    return (uint8_t)HAL_I2C_Master_Transmit(baro_i2c, dev_addr, &cmd, 1, 100);
}


static uint8_t ms5611_i2c_read(uint8_t dev_addr, uint8_t cmd, uint8_t *data, uint16_t size) {
    /* MS5611'de once komut yazilir, sonra okunur. Komutsuz okuma cop dondurur
       ama sensor calisiyor gibi gorunur. */
    if (HAL_I2C_Master_Transmit(baro_i2c, dev_addr, &cmd, 1, 100) != HAL_OK) {
        return 1;
    }
    return (uint8_t)HAL_I2C_Master_Receive(baro_i2c, dev_addr, data, size, 100);
}

#else

#endif
#endif

//LORA
#if YRT_IS_LORA_ENABLED

    static LoRa my_lora;


    //LoRa init
    uint8_t yrt_LoRa_init(const yrt_LoRa_conf_t *config){


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

    return LoRa_transmit(&my_lora, (uint8_t*)data, length, timeout);

    }


    // LoRa receive
    uint8_t yrt_LoRa_Receive(uint8_t *send_data, uint8_t length, uint16_t timeout){

        LoRa_startReceiving(&my_lora);
        //uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length);
        //şimdilik dursun burası, asıl receive fonksiyonunu ekleyeceğiz
        return 0;
    }


#endif

//IMU
#if YRT_IS_IMU_ENABLED

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
        return YRT_IMU_ERROR;
    }

    yrt_delay(100);
    if (bno055_set_unit(&bno055, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS, BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG) != BNO_OK)
    {
        return YRT_IMU_ERROR;
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
    	return YRT_IMU_ERROR;
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

    yrt_imu->data.accel.x = acc.x;
    yrt_imu->data.accel.y = acc.y;
    yrt_imu->data.accel.z = acc.z;

    yrt_imu->data.lineer_accel.x = lin_acc.x;
    yrt_imu->data.lineer_accel.y = lin_acc.y;
    yrt_imu->data.lineer_accel.z = lin_acc.z;

    yrt_imu->data.gyro.x = gyro.x;
    yrt_imu->data.gyro.y = gyro.y;
    yrt_imu->data.gyro.z = gyro.z;

    yrt_imu->data.mag.x = mag.x;
    yrt_imu->data.mag.y = mag.y;
    yrt_imu->data.mag.z = mag.z;

    yrt_imu->data.grav.x = grav.x;
    yrt_imu->data.grav.y = grav.y;
    yrt_imu->data.grav.z = grav.z;

    yrt_imu->data.euler.x = euler.roll;
    yrt_imu->data.euler.y = euler.pitch;
    yrt_imu->data.euler.z = euler.yaw;

    yrt_imu->data.quaternion.x = quat.x;
    yrt_imu->data.quaternion.y = quat.y;
    yrt_imu->data.quaternion.z = quat.z;
    yrt_imu->data.quaternion.w = quat.w;

    #elif YRT_IMU_MPU6050_SELECTED
    //MPU6050

    #elif YRT_IMU_BMI088_SELECTED
    //BMI088

    if (BMI088_ReadAcc(&bmi088) != HAL_OK)
    {
    	return YRT_IMU_ERROR;
    }

    yrt_imu->data.accel.x = bmi088.acc[0];
    yrt_imu->data.accel.y = bmi088.acc[1];
    yrt_imu->data.accel.z = bmi088.acc[2];
    
    if (BMI088_ReadGyr(&bmi088) != HAL_OK)
    {
    	return YRT_IMU_ERROR;
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

    baro_i2c = (I2C_HandleTypeDef *)baro_config->config.hi2c;
    if (baro_i2c == NULL) return YRT_INVALID_PARAM;

    ms5611_dev.Transmit = ms5611_i2c_write;
    ms5611_dev.Receive  = ms5611_i2c_read;
    ms5611_dev.Delay    = yrt_delay;

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

    return YRT_BARO_ERROR;
}
    
YRT_Status_t YRT_Baro_Init(YRT_BARO_t* yrt_baro)
{
    YRT_Status_t rslt = YRT_BARO_ERROR;

    #if YRT_BARO_MS5611_SELECTED
    //MS5611

    if ((rslt = YRT_Baro_Config(yrt_baro)) != YRT_OK)
    {
    	return rslt;
    }

    if(MS5611_init(&ms5611_dev, yrt_baro->config.dev_address) != MS5611_OK)
    {
    	return YRT_BARO_ERROR;
    }

    /* Yer referansi (dara) alinir; 50 ornek + yeterlilik kontrolu surucude. */
    if(MS5611_daraAl(&ms5611_dev) != MS5611_OK)
    {
    	return YRT_BARO_ERROR;
    }

    if(YRT_Baro_Read(yrt_baro) != YRT_OK)
    {
    	return YRT_BARO_ERROR;
    }

    #elif YRT_BARO_BMP280_SELECTED
    //BMP280

    if((rslt = YRT_Baro_Config(yrt_baro)) != YRT_OK)
    {
        return rslt;
    }

    if (!bmp280_init(&bmp280, &bmp280.params))
    {
		return YRT_BARO_ERROR;
	}

    if (bmp280_read_float(&bmp280, NULL, &(yrt_baro->data.ground_pressure), NULL) != true)
    {
    	return YRT_BARO_ERROR;
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

    if (MS5611_veriOku(&ms5611_dev) == MS5611_OK) {

        yrt_baro->data.altitude		    = ms5611_dev.irtifa;
        yrt_baro->data.pressure		    = ms5611_dev.basinc;
        yrt_baro->data.temperature	    = ms5611_dev.sicaklik;
        yrt_baro->data.raw_pressure     = ms5611_dev.D1;
        yrt_baro->data.raw_temperature  = ms5611_dev.D2;
        return YRT_OK;
    }
    return YRT_BARO_ERROR;

    #elif YRT_BARO_BMP280_SELECTED
    //BMP280

    if (bmp280_read_float(&bmp280, &(yrt_baro->data.temperature), &(yrt_baro->data.pressure), NULL) != true)
    {
    	return YRT_BARO_ERROR;
    }
    yrt_baro->data.altitude = YRT_CalculateAltitude(yrt_baro->data.ground_pressure, yrt_baro->data.pressure, yrt_baro->data.temperature);

    #else

    //Farklı sensör varsa burayı doldur
    #endif

    return YRT_OK;
}

float YRT_CalculateAltitude(float p, float pi, float ti)
{
    const float R = 287.05f;  // Specific gas constant for dry air (J/(kg*K))
    const float L = 0.0065f;  // Temperature lapse rate (K/m)
    const float g = 9.80665f; // Acceleration due to gravity (m/s^2)
    // #define p11 214.225  // Standard pressure at 11 km (Pa)

    const float MOLAR_MASS = 0.0289644f;             // Molar mass of Earth's air in kg/mol
    const float UNIVERSAL_GAS_CONSTANT = 8.3144598f; // Universal gas constant in J/(mol·K)

    /* f sonekleri ve powf/logf sart: F446 FPU'su sadece single precision,
       double islem yazilim emulasyonuna duser. */
    float alt = (ti + 273.15f) / L * (1.0f - powf(p / pi, R * L / g));

    // Calculate altitude above 11km
    // temperature set to -56.5c(216.65K) as simulations shows this in 11km
    if (alt > 11000.0f)
    {
        // calculate 11km pressure. It not constant due to ti
        const float exponent = (g * MOLAR_MASS) / (UNIVERSAL_GAS_CONSTANT * L);
        const float p11 = pi * powf((1.0f - (L * 11000.0f) / (ti + 273.15f)), exponent);

        // float T = (ti + 273.15 ) - L * 11000;
        // float p11 = pi * pow((T / (ti + 273.15 )), (-g * M_E / (R * L)));
        alt = 11000.0f + (R * 216.65f / g) * logf(p11 / p);
    }
    return alt;
}

#endif
/* ==========================================================================
 *  Cevre birim katmani — CAN / GPIO / Serial
 *  Not: Hangi CAN, hangi UART, hangi pin sorularinin cevabi burada DEGIL,
 *       BPC_SET.c icinde. Bu katman sadece "nasil" bilir.
 * ========================================================================== */

/**
 * @brief CAN BUS Functions
 */
#if YRT_IS_CAN_ENABLED

static CAN_FilterTypeDef   canFilterConfig;
static CAN_TxHeaderTypeDef TxHeader;
static uint32_t            TxMailbox;

YRT_Status_t YRT_CAN_Init(YRT_JR_CAN_DEV_t *can_dev)
{
    if (can_dev == NULL || can_dev->hcan == NULL) return YRT_INVALID_PARAM;

    canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	canFilterConfig.FilterBank = 0;
	canFilterConfig.FilterFIFOAssignment = (can_dev->fifo_num == YRT_JR_CAN_FIFO0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(can_dev->hcan, &canFilterConfig) != HAL_OK)
	{
		return YRT_CAN_ERROR;
	}

    if(HAL_CAN_Start(can_dev->hcan) != HAL_OK)
    {
        return YRT_CAN_ERROR;
    }

    /* Bu olmadan HAL_CAN_RxFifo0MsgPendingCallback hic tetiklenmez. */
    if(HAL_CAN_ActivateNotification(can_dev->hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return YRT_CAN_ERROR;
    }

    return YRT_OK;
}

/**
 * @brief CAN RX FIFO0 message pending callback
 * @param hcan Pointer to the CAN handle
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if (hcan->Instance == CAN1)
    {
        CAN_Msg_t newMsg;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &newMsg.header, newMsg.data);
#if YRT_IS_RTOS_ENABLED
        if (canRxQueueHandle != NULL) {
            osMessageQueuePut(canRxQueueHandle, &newMsg, 0, 0);
        }
#endif
    }
}

#endif

/**
 * @brief Initialize GPIO
 * @param gpio GPIO structure
 * @param port GPIO port
 * @param pin GPIO pin
 * @param mode GPIO mode
 * @return Result of the initialization
 */
YRT_Status_t YRT_GPIO_Init(YRT_JR_GPIO *gpio, GPIO_TypeDef *port, uint16_t pin, YRT_JR_OP_MODE mode)
{
    if (gpio == NULL || port == NULL) return YRT_INVALID_PARAM;

    gpio->port = port;
    gpio->pin = pin;
    gpio->mode = mode;
    gpio->state = YRT_JR_OFF;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = (mode == YRT_JR_MODE_ACTIVE) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    return YRT_OK;
}

/**
 * @brief Set the state of a GPIO pin
 * @param gpio GPIO structure
 * @param state Desired state
 */
void YRT_GPIO_SetState(YRT_JR_GPIO *gpio, YRT_JR_DEV_STATE state)
{
    if (gpio == NULL || gpio->port == NULL) return;

    gpio->state = state;
    if (state == YRT_JR_ON)
    {
        HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_SET);
    }
    else if (state == YRT_JR_OFF)
    {
        HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief Toggle the state of a GPIO pin
 * @param gpio GPIO structure
 */
void YRT_GPIO_Toggle(YRT_JR_GPIO *gpio) {
    if (gpio == NULL || gpio->port == NULL) return;
    HAL_GPIO_TogglePin(gpio->port, gpio->pin);
}

/**
 * @brief Send data over serial
 * @param dev Serial device structure
 * @param data Data to send
 * @param len Length of data to send
 * @return Result of the operation
 */
#if YRT_IS_SERIAL_ENABLED

YRT_Status_t YRT_Serial_Send(YRT_JR_SERIAL_DEV *dev, uint8_t *data, uint16_t len) {
    if (dev == NULL || data == NULL || len == 0) return YRT_ERROR;

    switch (dev->protocol) {
        case YRT_SERIAL_UART:
            if (dev->huart == NULL) return YRT_INVALID_PARAM;
            if (HAL_UART_Transmit(dev->huart, data, len, HAL_MAX_DELAY) != HAL_OK) return YRT_COMM_ERROR;
            dev->bytesSent += len;
            break;

        case YRT_SERIAL_USB:
#if YRT_IS_USB_ENABLED
            if (CDC_Transmit_FS(data, len) != USBD_OK) return YRT_COMM_ERROR;
            dev->bytesSent += len;
#else
            return YRT_ERROR; // USB makrosu kapaliysa hata don
#endif
            break;

        case YRT_SERIAL_CAN:
#if YRT_IS_CAN_ENABLED
            {
        if (dev->hcan == NULL) return YRT_INVALID_PARAM;

        uint16_t sent_bytes = 0;
        TxHeader.StdId = dev->target_id;
        TxHeader.ExtId = 0;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.RTR = CAN_RTR_DATA;

        while (sent_bytes < len) {
            uint8_t dlc = (len - sent_bytes > 8) ? 8 : (uint8_t)(len - sent_bytes);
            TxHeader.DLC = dlc;
            if (HAL_CAN_AddTxMessage(dev->hcan, &TxHeader, &data[sent_bytes], &TxMailbox) != HAL_OK)
            {
                return YRT_CAN_ERROR;
            }
            sent_bytes += dlc;
            }
                dev->bytesSent += len;
            }
#else
            return YRT_ERROR; // CAN makrosu kapaliysa hata don
#endif
            break;

        default:
            return YRT_INVALID_PARAM;
    }

    return YRT_OK;
}

/**
 * @brief Receive data over serial
 * @param dev Serial device structure
 * @param data Buffer to store received data
 * @param len Length of data to receive
 * @return Result of the operation
 */
YRT_Status_t YRT_Serial_Receive(YRT_JR_SERIAL_DEV *dev, uint8_t *data, uint16_t len) {
    if (dev == NULL || data == NULL || len == 0) return YRT_ERROR;

    switch (dev->protocol) {
        case YRT_SERIAL_UART:
            if (dev->huart == NULL) return YRT_INVALID_PARAM;
            if (HAL_UART_Receive(dev->huart, data, len, HAL_MAX_DELAY) != HAL_OK) return YRT_COMM_ERROR;
            dev->bytesReceived += len;
            break;

        case YRT_SERIAL_USB:
            // USB receive implementation here
            return YRT_ERROR;

        case YRT_SERIAL_CAN:
#if (YRT_IS_CAN_ENABLED && YRT_IS_RTOS_ENABLED)
            {
                uint16_t received_bytes = 0;
                while (received_bytes < len) {
                    CAN_Msg_t msg;
                    osMessageQueueGet(canRxQueueHandle, &msg, NULL, osWaitForever);
                    uint8_t dlc = (len - received_bytes > 8) ? 8 : (uint8_t)(len - received_bytes);
                    memcpy(data + received_bytes, msg.data, dlc);
                    received_bytes += dlc;
                }
                dev->bytesReceived += len;
            }
#else
            return YRT_ERROR; // CAN makrosu kapaliysa hata don
#endif
            break;

        default:
            return YRT_INVALID_PARAM;
    }

    return YRT_OK;
}

#endif
