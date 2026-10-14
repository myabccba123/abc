//
// MPU6050 底层驱动（I2C1：SCL=PB6，SDA=PB7）
//

#pragma once

#include <stdint.h>

/** MPU6050 数据（已换算为物理量） */
typedef struct {
    float accel[3];  /**< 加速度，单位 g */
    float gyro[3];   /**< 角速度，单位 °/s（已去除零偏） */
    float temp;      /**< 温度，单位 ℃ */
} bsp_mpu6050_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 MPU6050，量程默认 ±2g / ±250°/s，数字低通 44Hz
 */
void bsp_mpu6050_init(void);

/**
 * @brief 读取 MPU6050 数据
 * @return 换算后的加速度、角速度与温度
 */
bsp_mpu6050_data_t bsp_mpu6050_read(void);

/**
 * @brief 陀螺仪零偏校准，调用期间请保持静止
 * @param samples 采样次数
 */
void bsp_mpu6050_calibrate(uint16_t samples);

#ifdef __cplusplus
}
#endif