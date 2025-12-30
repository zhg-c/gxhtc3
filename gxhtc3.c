#include "gxhtc3.h"

void float_to_str(char *buf, float num) {
    int integer_part = (int)num;
    int decimal_part = (int)((num - integer_part) * 100 + 0.5);
    
    if (decimal_part >= 100) {
        integer_part += 1;
        decimal_part -= 100;
    } else if (num < 0 && decimal_part > 0) {
        decimal_part = 100 - decimal_part;
        integer_part -= 1;
    }
    
    sprintf(buf, "%d.%02d", integer_part, decimal_part);
}

void HAL_Delay_us(uint32_t us);
/**
 * @brief CRC8校验（手册表15：多项式0x31，初始值0xFF，无反射）
 * @param data：待校验的2字节数据
 * @param crc：传感器返回的CRC值
 * @retval HAL_StatusTypeDef：HAL_OK=校验通过，HAL_ERROR=校验失败
 */
static HAL_StatusTypeDef GXHTC3_CheckCRC(uint8_t *data, uint8_t crc) {
    uint8_t crc_calc = 0xFFU; // 初始值0xFF
    for (uint8_t i = 0; i < 2U; i++) {
        crc_calc ^= data[i];  // 异或当前数据字节
        // 8位循环移位计算
        for (uint8_t j = 0; j < 8U; j++) {
            if ((crc_calc & 0x80U) != 0U) {
                crc_calc = (crc_calc << 1U) ^ 0x31U; // 多项式0x31（x^8+x^5+x^4+1）
            } else {
                crc_calc <<= 1U;
            }
        }
    }
    return (crc_calc == crc) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 发送16位命令（通用函数：唤醒/休眠/测量命令均为16位）
 * @param hi2c：I2C句柄指针
 * @param cmd：16位命令（MSB在前，LSB在后）
 * @retval HAL_StatusTypeDef：通信结果
 */
static HAL_StatusTypeDef GXHTC3_SendCmd(I2C_HandleTypeDef *hi2c, uint16_t cmd) {
    uint8_t cmd_buf[2U] = {
        (uint8_t)(cmd >> 8U),  // 命令高8位（MSB）
        (uint8_t)(cmd & 0xFFU) // 命令低8位（LSB）
    };
    // 发送命令：START → 写地址 → ACK → 命令MSB → ACK → 命令LSB → ACK → STOP
    return HAL_I2C_Master_Transmit(
        hi2c,
        GXHTC3_I2C_ADDR_WRITE,
        cmd_buf,
        2U,
        GXHTC3_I2C_TIMEOUT_MS
    );
}

/**
 * @brief 初始化GXHTC3（唤醒+读序列号，验证传感器在线）
 * @param hi2c：I2C句柄指针
 * @param sensor_id：输出参数，存储传感器16位序列号（可NULL，仅用于验证）
 * @retval HAL_StatusTypeDef：初始化结果
 */
HAL_StatusTypeDef GXHTC3_Init(I2C_HandleTypeDef *hi2c, uint8_t *sensor_id) {
    HAL_StatusTypeDef status;
    uint8_t id_buf[3U] = {0U}; // 序列号：2字节数据 + 1字节CRC

    // 1. 发送唤醒命令（手册5.2：休眠状态必须先唤醒）
    status = GXHTC3_SendCmd(hi2c, GXHTC3_CMD_WAKEUP);
    if (status != HAL_OK) return status;
    HAL_Delay_us(GXHTC3_WAKEUP_DELAY_US); // 唤醒后等待稳定（至少40us）



    // 2. 验证传感器在线（可选，增强鲁棒性）
    status = HAL_I2C_IsDeviceReady(hi2c, GXHTC3_I2C_ADDR_WRITE, 3U, GXHTC3_I2C_TIMEOUT_MS);
    if (status != HAL_OK) return status;



    // 3. 读取序列号（手册5.9：验证通信正确性）
    if (sensor_id != NULL) {
        // 3.1 发送读ID命令
        status = GXHTC3_SendCmd(hi2c, GXHTC3_CMD_READ_ID);
        if (status != HAL_OK) return status;
        // 3.2 读取3字节数据（2字节ID + 1字节CRC）
        status = HAL_I2C_Master_Receive(
            hi2c,
            GXHTC3_I2C_ADDR_READ,
            id_buf,
            3U,
            GXHTC3_I2C_TIMEOUT_MS
        );
        if (status != HAL_OK) return status;



        // 3.3 CRC校验（可选，确保ID正确）
        status = GXHTC3_CheckCRC(&id_buf[0U], id_buf[2U]);
        if (status != HAL_OK) return HAL_ERROR;



        // 3.4 存储16位序列号
        sensor_id[0U] = id_buf[0U];
        sensor_id[1U] = id_buf[1U];
    }

    return HAL_OK;
}

/**
 * @brief 读取温湿度数据（核心函数，遵循手册图7时序）
 * @param hi2c：I2C句柄指针
 * @param data：输出参数，存储解析后的温湿度数据
 * @retval HAL_StatusTypeDef：读取结果
 */
HAL_StatusTypeDef GXHTC3_ReadHumidityTemp(I2C_HandleTypeDef *hi2c, GXHTC3_DataTypedef *data) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[6U] = {0U}; // 原始数据：湿度(MSB/LSB/CRC) + 温度(MSB/LSB/CRC)
    uint16_t hum_raw, temp_raw;

    // 1. 唤醒传感器（若此前进入休眠，必须唤醒）
    status = GXHTC3_SendCmd(hi2c, GXHTC3_CMD_WAKEUP);
    if (status != HAL_OK) goto exit_fail;
    HAL_Delay_us(GXHTC3_WAKEUP_DELAY_US);



    // 2. 发送测量命令（选择湿度先出+Clock Stretching开启，手册表11）
    status = GXHTC3_SendCmd(hi2c, GXHTC3_CMD_MEASURE_HUM);
    if (status != HAL_OK) goto exit_fail;
    // HAL_Delay(GXHTC3_MEASURE_DELAY_MS); // Clock Stretching开启时无需延时，传感器会拉低SCL等待

    // 3. 读取6字节原始数据（手册5.6：湿度3字节 + 温度3字节）
    status = HAL_I2C_Master_Receive(
        hi2c,
        GXHTC3_I2C_ADDR_READ,
        raw_data,
        6U,
        GXHTC3_I2C_TIMEOUT_MS
    );
    if (status != HAL_OK) goto exit_fail;

        

    // 4. CRC校验（手册5.10：确保数据无传输错误）
    // 4.1 湿度数据校验（前3字节：raw_data[0]=MSB, raw_data[1]=LSB, raw_data[2]=CRC）
    status = GXHTC3_CheckCRC(&raw_data[0U], raw_data[2U]);
    if (status != HAL_OK) goto exit_fail;
    // 4.2 温度数据校验（后3字节：raw_data[3]=MSB, raw_data[4]=LSB, raw_data[5]=CRC）
    status = GXHTC3_CheckCRC(&raw_data[3U], raw_data[5U]);
    if (status != HAL_OK) goto exit_fail;

   

    // 5. 数据解析（手册5.11：公式转换）
    // 5.1 湿度转换：RH = 100 * S_RH / 2^16
    hum_raw = (uint16_t)(raw_data[0U] << 8U) | raw_data[1U];
    data->humidity = (float)hum_raw * 100.0f / 65535.0f;
    // 5.2 温度转换：T = -45 + 175 * S_T / 2^16
    temp_raw = (uint16_t)(raw_data[3U] << 8U) | raw_data[4U];
    data->temperature = (float)temp_raw * 175.0f / 65535.0f - 45.0f;
    // 5.3 标记数据有效
    data->is_valid = 1U;
    
    // 6. （可选）进入休眠（降低功耗，手册5.2：空闲时建议休眠）
    GXHTC3_EnterSleep(hi2c);
    return HAL_OK;

exit_fail:
    // 读取失败，标记数据无效
    data->is_valid = 0U;
    GXHTC3_EnterSleep(hi2c); // 失败也尝试休眠，避免高功耗
    return status;
}

/**
 * @brief 软复位GXHTC3（手册5.7：强制恢复到空闲状态）
 * @param hi2c：I2C句柄指针
 * @retval HAL_StatusTypeDef：复位结果
 */
HAL_StatusTypeDef GXHTC3_SoftReset(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;
    // 发送软复位命令
    status = GXHTC3_SendCmd(hi2c, GXHTC3_CMD_SOFT_RESET);
    if (status != HAL_OK) return status;
    HAL_Delay_us(500U); // 软复位后稳定时间（手册表5：最大500us）
    return HAL_OK;
}

/**
 * @brief 让GXHTC3进入休眠（手册5.2：降低功耗至0.3μA）
 * @param hi2c：I2C句柄指针
 * @retval HAL_StatusTypeDef：休眠结果
 */
HAL_StatusTypeDef GXHTC3_EnterSleep(I2C_HandleTypeDef *hi2c) {
    // 发送休眠命令
    return GXHTC3_SendCmd(hi2c, GXHTC3_CMD_SLEEP);
}

/**
 * @brief 微秒级延时（基于SysTick）
 * @param us：延时时间（单位：us，最大支持约1.6秒，满足传感器需求）
 * @note 原理：利用SysTick的计数寄存器，计算微秒对应的时钟周期
 */
void HAL_Delay_us(uint32_t us) {
    uint32_t start_tick, current_tick;
    uint32_t tick_per_us = SystemCoreClock / 1000000U;  // 每微秒的SysTick计数（如168MHz→168）
    
    // 获取当前SysTick计数（HAL库的SysTick时钟为SystemCoreClock/8，需修正）
    start_tick = SysTick->VAL;
    // 计算需要等待的总计数（SysTick为递减计数，需处理溢出）
    uint32_t wait_ticks = us * tick_per_us;
    
    do {
        current_tick = SysTick->VAL;
    } while ((start_tick - current_tick + (start_tick < current_tick ? SysTick->LOAD + 1 : 0)) < wait_ticks);
}