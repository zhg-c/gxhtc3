#ifndef GXHTC3_HAL_H
#define GXHTC3_HAL_H

#include "stm32f1xx_hal.h"  // 替换为你的MCU对应的HAL头文件（如stm32f4xx_hal.h）

/**
 * @brief GXHTC3关键参数定义（源自手册第5章）
 */
// 1. I2C地址（手册表8：二进制01110000 → 7位地址0x70）
#define GXHTC3_I2C_ADDR_7BIT    0x70U
#define GXHTC3_I2C_ADDR_WRITE   ((GXHTC3_I2C_ADDR_7BIT << 1) | 0x00U)  // 写地址（最后1位0）
#define GXHTC3_I2C_ADDR_READ    ((GXHTC3_I2C_ADDR_7BIT << 1) | 0x01U)  // 读地址（最后1位1）

// 2. 核心命令（手册表9~14，修正此前错误命令码）
#define GXHTC3_CMD_WAKEUP       0x3517U    // 唤醒命令（表10）
#define GXHTC3_CMD_SLEEP        0xB098U    // 休眠命令（表9）
#define GXHTC3_CMD_SOFT_RESET   0x805DU    // 软复位命令（表12）
#define GXHTC3_CMD_READ_ID      0xEFC8U    // 读序列号命令（表14）
// 测量命令（表11：选择Clock Stretching开启+湿度在前，适配大多数场景）
#define GXHTC3_CMD_MEASURE_HUM  0x5C24U    // 正常模式：湿度先出+Clock Stretching开启
#define GXHTC3_CMD_MEASURE_TEMP 0x7CA2U    // 正常模式：温度先出+Clock Stretching开启

// 3. 时序参数（手册表5、表6，确保稳定通信）
#define GXHTC3_WAKEUP_DELAY_US  50U        // 唤醒后稳定时间（手册表5：>40us）
#define GXHTC3_MEASURE_DELAY_MS 12U        // 测量转换时间（手册表5：最大11ms，留1ms冗余）
#define GXHTC3_I2C_TIMEOUT_MS   1000U      // I2C通信超时时间（适配Clock Stretching）

/**
 * @brief 温湿度数据结构体
 */
typedef struct {
    float humidity;    // 湿度值（单位：%RH，范围0~100）
    float temperature; // 温度值（单位：°C，范围-45~130）
    uint8_t is_valid;  // 数据有效性标记（1=有效，0=无效）
} GXHTC3_DataTypedef;

/**
 * @brief 函数声明
 */
// 初始化相关
HAL_StatusTypeDef GXHTC3_Init(I2C_HandleTypeDef *hi2c, uint8_t *sensor_id);
// 数据读取相关
HAL_StatusTypeDef GXHTC3_ReadHumidityTemp(I2C_HandleTypeDef *hi2c, GXHTC3_DataTypedef *data);
// 工具函数
HAL_StatusTypeDef GXHTC3_SoftReset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef GXHTC3_EnterSleep(I2C_HandleTypeDef *hi2c);
void float_to_str(char *buf, float num);

#endif // GXHTC3_HAL_H