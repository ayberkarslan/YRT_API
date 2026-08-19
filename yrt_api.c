//YRT_API_C

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




void YRT_Delay(uint32_t period)
{
#if YRT_IS_RTOS_ENABLED
    osDelay(period);
#else
    HAL_Delay(period);
#endif
}


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
    YRT_Status_t yrt_LoRa_init(const yrt_LoRa_conf_t *config){


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
        my_lora.bandWidth             = config -> bandWidth;       // defsault = BW_125KHz
        my_lora.crcRate               = config -> crcRate ;          // default = CR_4_5
        my_lora.power                 = config -> power;      // default = 20db
        my_lora.overCurrentProtection = config -> overCurrentProtection;             // default = 100 mA
        my_lora.preamble              = config -> preamble;              // default = 8;


        // cs high (spi başlaması için falling edge şart)
    
    //LoRa çalışmazsa dene -> HAL_GPIO_WritePin(my_lora.CS_port, my_lora.CS_pin, GPIO_PIN_SET);
           
        // önce reset at

    //LoRa çalışmazsa dene -> LoRa_reset(&my_lora);
           
           // 200 dönerse LORA_OK demektir
                if (LoRa_init(&my_lora) == 200) { 
                    return YRT_OK; 
                }
            return YRT_ERR;
    }

        // LoRa transmit

    	YRT_Status_t yrt_LoRa_Transmit(const uint8_t *data, uint8_t length, uint16_t timeout) {
            
            // ÖNEMLİ LAN - Eski interruptan kalan bayrakları temizliyoruz ki LoRa kendini kilitlemesin (açılış parazitinde yapabiliyor bunu)
            LoRa_write(&my_lora, RegIrqFlags, 0xFF);

            //kütüphane 1 döndürürse başarılı demek
            if (LoRa_transmit(&my_lora, (uint8_t*)data, length, timeout) == 1) {
                return YRT_OK; 
            }
             
            return YRT_TIMEOUT; 
	}


     // LoRa receive
    YRT_Status_t yrt_LoRa_Receive(uint8_t *receive_data, uint8_t length, uint16_t timeout){

        //LoRa_startReceiving(&my_lora);
        //uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length);
        //şimdilik dursun burası, asıl receive fonksiyonunu ekleyeceğiz
      uint32_t start_time = YRT_Get_Time(); // Kronometreyi başlat
            uint8_t received_bytes = 0;
            
            LoRa_startReceiving(&my_lora); // Telsizi dinleme moduna al

            // Timeout süresi dolana kadar sürekli havayı dinle
            while ( (YRT_Get_Time() - start_time) < timeout ) {
                
                // Havadan gelen paketi çekmeyi dene
                received_bytes = LoRa_receive(&my_lora, receive_data, length);
                
                if (received_bytes > 0) {
                    return YRT_OK; // Veri yakalandı ve başarıyla alındı!
                }
                
                YRT_Delay(1); // İşlemciyi boğmamak için 1 milisaniye nefesal
            }

            return YRT_TIMEOUT; // Süre doldu, kimse bir şey göndermedi
       
    
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
    bmi088.delay_ms = YRT_Delay;

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

    YRT_Delay(100);
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
   MPU6050_Initialization();

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

MPU6050_ProcessData(&MPU6050);


    out_data ->accel_x = MPU6050.acc_x;
    out_data ->accel_y = MPU6050.acc_y;
    out_data ->accel_z = MPU6050.acc_z;
    out_data ->gyro_x  = MPU6050.gyro_x;
    out_data ->gyro_y  = MPU6050.gyro_y;
    out_data ->gyro_z  = MPU6050.gyro_z;
    out_data ->temperature = MPU6050.temperature;
 

    return YRT_OK;

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
    ms5611_dev.delay_ms = YRT_Delay;

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

    if(MS5611_Init(&ms5611_dev) != YRT_OK)
    {
    	return YRT_BARO_ERR;
    }

    if(YRT_Baro_Read(yrt_baro) != YRT_OK)
    {
    	return YRT_BARO_ERR;
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

    if (MS5611_ReadRawData(&ms5611_dev) == YRT_OK) {
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



//CAN BUS
#if YRT_IS_CAN_ENABLED

static CAN_FilterTypeDef   canFilterConfig;


YRT_Status_t YRT_CAN_Init(YRT_CAN_DEV_t *can_dev)
{
    if (can_dev == NULL || can_dev->hcan == NULL) return YRT_ERR;

    canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	canFilterConfig.FilterBank = 0;
	canFilterConfig.FilterFIFOAssignment = (can_dev->fifo_num == YRT_CAN_FIFO0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(can_dev->hcan, &canFilterConfig) != HAL_OK)
	{
		return YRT_CAN_ERR;
	}

    if(HAL_CAN_Start(can_dev->hcan) != HAL_OK)
    {
        return YRT_CAN_ERR;
    }

    /* bu olmadan HAL_CAN_RxFifo0MsgPendingCallback hic tetiklenmez. */
    if(HAL_CAN_ActivateNotification(can_dev->hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return YRT_CAN_ERR;
    }

    return YRT_OK;
}

YRT_Status_t YRT_CAN_Send(YRT_CAN_DEV_t *can_dev, uint32_t target_id, uint8_t *data, uint16_t len)
{
    CAN_TxHeaderTypeDef TxHeader = {0};
    uint32_t            TxMailbox = 0;
    if (can_dev == NULL || can_dev->hcan == NULL || data == NULL || len == 0) return YRT_ERR;

    uint16_t sent_bytes = 0;
    TxHeader.StdId = target_id;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    while (sent_bytes < len) {
        // en fazla 8 byte aldık 
        uint8_t dlc = (len - sent_bytes > 8) ? 8 : (uint8_t)(len - sent_bytes);
        TxHeader.DLC = dlc;

        // mailboxların boşalmasını bekliyor
       uint32_t timeout = 0;
            while (HAL_CAN_GetTxMailboxesFreeLevel(can_dev->hcan) == 0) {
                YRT_Delay(1); // 1 ms bekle ve tekrar kontrol et
                timeout++;
                if (timeout > 10) return YRT_TIMEOUT; // kilitlenmeyi çözen satır
            }

        if (HAL_CAN_AddTxMessage(can_dev->hcan, &TxHeader, &data[sent_bytes], &TxMailbox) != HAL_OK)
        {
            return YRT_CAN_ERR;
        }
        sent_bytes += dlc;
    }
    return YRT_OK;
}

YRT_Status_t YRT_CAN_Receive(YRT_CAN_DEV_t *can_dev, uint32_t *sender_id, uint8_t *data, uint16_t *len)
{
#if YRT_IS_RTOS_ENABLED
    if (canRxQueueHandle == NULL) return YRT_ERR;
    CAN_Msg_t msg;

    if (xQueueReceive((QueueHandle_t)canRxQueueHandle, &msg, 0) == pdPASS){
        if (sender_id != NULL) *sender_id = msg.header.StdId;
        if (len != NULL) *len = msg.header.DLC;
        if (data != NULL) memcpy(data, msg.data, msg.header.DLC);
        return YRT_OK;
    }
    return YRT_ERR; // Kuyruk boş
#else
    // RTOS kapalıysa direkt donanım FIFO'sundan kontrol et
    uint32_t fifo = (can_dev->fifo_num == YRT_CAN_FIFO0) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    if (HAL_CAN_GetRxFifoFillLevel(can_dev->hcan, fifo) > 0) {
        CAN_RxHeaderTypeDef rxHeader;
        if (HAL_CAN_GetRxMessage(can_dev->hcan, fifo, &rxHeader, data) == HAL_OK) {
            if (sender_id != NULL) *sender_id = rxHeader.StdId;
            if (len != NULL) *len = rxHeader.DLC;
            return YRT_OK;
        }
    }
    return YRT_ERR;
#endif
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
            xQueueSendFromISR((QueueHandle_t)canRxQueueHandle, &newMsg, NULL);
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
YRT_Status_t YRT_GPIO_Init(YRT_GPIO *gpio, GPIO_TypeDef *port, uint16_t pin, YRT_OP_MODE mode)
{
    if (gpio == NULL || port == NULL) return YRT_ERR;

    gpio->port = port;
    gpio->pin = pin;
    gpio->mode = mode;
    gpio->state = YRT_DEV_OFF;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = (mode == YRT_MODE_ACTIVE) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    return YRT_OK;
}

/**
 * @brief Set the state of a GPIO pin
 * @param gpio GPIO structure
 * @param state Desired state
 */
void YRT_GPIO_SetState(YRT_GPIO *gpio, YRT_DEV_STATE state)
{
    if (gpio == NULL || gpio->port == NULL) return;

    gpio->state = state;
    if (state == YRT_DEV_ON)
    {
        HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_SET);
    }
    else if (state == YRT_DEV_OFF)
    {
        HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief Toggle the state of a GPIO pin
 * @param gpio GPIO structure
 */
void YRT_GPIO_Toggle(YRT_GPIO *gpio) {
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

YRT_Status_t YRT_Serial_Send(YRT_SERIAL_DEV *dev, uint8_t *data, uint16_t len) {
    if (dev == NULL || data == NULL || len == 0) return YRT_ERR;

    switch (dev->protocol) {
        case YRT_SERIAL_UART:
            if (dev->huart == NULL) return YRT_ERR;
            if (HAL_UART_Transmit(dev->huart, data, len, HAL_MAX_DELAY) != HAL_OK) return YRT_COMM_ERR;
            dev->bytesSent += len;
            break;

        case YRT_SERIAL_USB:
#if YRT_IS_USB_ENABLED
            if (CDC_Transmit_FS(data, len) != USBD_OK) return YRT_COMM_ERROR;
            dev->bytesSent += len;
#else
            return YRT_ERR; // USB makrosu kapaliysa hata don
#endif
            break;

        case YRT_SERIAL_CAN:
#if YRT_IS_CAN_ENABLED
            {
        if (dev->hcan == NULL) return YRT_ERR;

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
                return YRT_CAN_ERR;
            }
            sent_bytes += dlc;
            }
                dev->bytesSent += len;
            }
#else
            return YRT_ERR; // CAN makrosu kapaliysa hata don
#endif
            break;

        default:
            return YRT_ERR;
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
YRT_Status_t YRT_Serial_Receive(YRT_SERIAL_DEV *dev, uint8_t *data, uint16_t len) {
    if (dev == NULL || data == NULL || len == 0) return YRT_ERR;

    switch (dev->protocol) {
        case YRT_SERIAL_UART:
            if (dev->huart == NULL) return YRT_ERR;
            if (HAL_UART_Receive(dev->huart, data, len, HAL_MAX_DELAY) != HAL_OK) return YRT_COMM_ERR;
            dev->bytesReceived += len;
            break;

        case YRT_SERIAL_USB:
            // USB receive implementation here
            return YRT_ERR;

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
            return YRT_ERR; // CAN makrosu kapaliysa hata don
#endif
            break;

        default:
            return YRT_ERR;
    }

    return YRT_OK;
}
#endif


//ALL GPS FUNCTIONS
#if YRT_IS_GPS_ENABLED

static void *gps_uart;
static uint8_t gps_rx_buffer[100]; //GNGGA bufferi



//GPS



YRT_Status_t YRT_GPS_Init(const YRT_GPS_Config_t *config){
    if(config == NULL || config->huart == NULL){
       return YRT_ERR;
    }
    gps_uart = config->huart;

    // --- UART HATA TEMIZLEME ---
    __HAL_UART_CLEAR_OREFLAG((UART_HandleTypeDef*)gps_uart);
    __HAL_UART_CLEAR_FEFLAG((UART_HandleTypeDef*)gps_uart);
    
    HAL_Delay(100);

    HAL_UART_Receive_DMA(gps_uart, gps_rx_buffer, 100);
    return YRT_OK;
}













    void YRT_GPS_Parse(const char *sentence, int field_index, char *out_buffer) {
        // circular bufferda veri kaybında bufferin baştan yanlış dizilmesini önelemek için aldığımız bufferi
        // kopyalayıp kendi ardına ekleyerek kaybı önledik
        char safe_buffer[201];
        memcpy(safe_buffer, sentence, 100);
        memcpy(safe_buffer + 100, sentence, 100); // peşine aynısını kopyaladık
        safe_buffer[200] = '\0';

        // buffer şçinde GNGGA veya GPGGA arıyoruz.
        char *start = strstr(safe_buffer, "$GNGGA");
        if (start == NULL) {
            start = strstr(safe_buffer, "$GPGGA");
        }
        
        // GGA ifadesi yoksa başa dönüyoruz
        if (start == NULL) {
            out_buffer[0] = '\0';
            return;
        }

        int i = 0;             // GNGGA kelimesinin başından itibaren harf sayacı
        int current_comma = 0;
        int out_i = 0;         

        //  istenilen virgül kadar kelime atlıyoruz (spesifik verileri almak için)
        while (start[i] != '\0' && current_comma < field_index) {
            if (start[i] == ',') {
                current_comma++;
            }
            i++;
        }





        
        // istenilen spesifik veriye ulaşınca kopyalamayı başlattık (yıldız ya da virgül işaretine kadar sürüyor)
        while (start[i] != '\0' && start[i] != ',' && start[i] != '*') {
            out_buffer[out_i] = start[i];
            out_i++;
            i++;
        }

        //  kapattık
        out_buffer[out_i] = '\0';
    }











        YRT_Status_t YRT_GPS_Read(YRT_GPS_t *out_data) {
        char gecici_yazi[20];
        float raw_value;
        int degrees;
        float minutes;

        // fix kalitesi 
        YRT_GPS_Parse((const char *)gps_rx_buffer, 6, gecici_yazi);
        out_data->fix_status = atoi(gecici_yazi); // atoi: yazıyı tam sayıya
        
        // uydulara henüz kilitlenmemişse saçma sapan veriler okumamak için direkt çıktık
        if (out_data->fix_status == 0) {
            return YRT_ERR; 
        }

        // uydu sayısı
        YRT_GPS_Parse((const char*)gps_rx_buffer, 7, gecici_yazi);
        out_data->satellites = atoi(gecici_yazi);

        // rakım (9)
        YRT_GPS_Parse((const char*)gps_rx_buffer, 9, gecici_yazi);
        out_data->altitude = atof(gecici_yazi); // atof: yazıyı ondalıklı sayıya 

        // enlem lat (2) -> DDMM.MMMM formatı
        YRT_GPS_Parse((const char*)gps_rx_buffer, 2, gecici_yazi);
        raw_value = atof(gecici_yazi);         // Örneğin 4100.1234 gelir
        degrees = (int)(raw_value / 100);      // 4100'ü 100'e bölüp tam kısmını aldık: 41 derece
        minutes = raw_value - (degrees * 100); // 4100.1234 - 4100 = 0.1234 dakika kalır
        out_data->latitude = degrees + (minutes / 60.0); //google maps formatı

        // boylam long (4)
        YRT_GPS_Parse((const char*)gps_rx_buffer, 4, gecici_yazi);
        raw_value = atof(gecici_yazi);         // Örneğin 02800.5678 gelir
        degrees = (int)(raw_value / 100);      // 28 derece
        minutes = raw_value - (degrees * 100); // 0.5678 dakika
        out_data->longitude = degrees + (minutes / 60.0);

        return YRT_OK;
    }



#endif


uint32_t YRT_Get_Time(void) {
    
    #if YRT_IS_RTOS_ENABLED
        return osKernelSysTick();
    #else
        return HAL_GetTick();
    #endif
}