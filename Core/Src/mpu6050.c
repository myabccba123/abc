//
// MPU6050 底层驱动（I2C1：SCL=PB6，SDA=PB7）
//

#include "mpu6050.h"

#include "i2c.h"

// 寄存器地址
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define MPU6050_ADDR    (0x68 << 1)  // AD0 接地，7 位地址 0x68
#define MPU6050_TIMEOUT 100          // I2C 超时时间，单位 ms

// 满量程对应的灵敏度
#define MPU6050_ACCEL_SEN 16384.0f  // ±2g：      LSB/g
#define MPU6050_GYRO_SEN  131.0f    // ±250°/s：  LSB/(°/s)

static float gyro_bias[3];  // 陀螺仪零偏，单位 °/s

/**
 * @brief 写单个寄存器
 */
static uint8_t write_reg(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, MPU6050_TIMEOUT) == HAL_OK;
}

/**
 * @brief 连续读寄存器
 */
static uint8_t read_reg(uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, MPU6050_TIMEOUT) == HAL_OK;
}

void bsp_mpu6050_init(void) {
    HAL_Delay(50);

    write_reg(MPU6050_REG_PWR_MGMT_1, 0x80);   // 复位
    HAL_Delay(100);
    write_reg(MPU6050_REG_PWR_MGMT_1, 0x01);   // 唤醒，时钟源选择 X 轴陀螺 PLL
    HAL_Delay(10);

    write_reg(MPU6050_REG_CONFIG, 0x03);        // 数字低通滤波 44Hz
    write_reg(MPU6050_REG_SMPLRT_DIV, 0x09);    // 采样率 = 1kHz / (1 + 9) = 100Hz
    write_reg(MPU6050_REG_GYRO_CONFIG, 0x00);   // 陀螺仪 ±250°/s
    write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00);  // 加速度计 ±2g
}

bsp_mpu6050_data_t bsp_mpu6050_read(void) {
    bsp_mpu6050_data_t ret = {0};
    uint8_t buf[14];

    // 读取：加速度(6) + 温度(2) + 角速度(6)
    if (read_reg(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
        int16_t temp = (int16_t)((buf[6] << 8) | buf[7]);
        int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

        ret.accel[0] = (float)ax / MPU6050_ACCEL_SEN;
        ret.accel[1] = (float)ay / MPU6050_ACCEL_SEN;
        ret.accel[2] = (float)az / MPU6050_ACCEL_SEN;

        ret.gyro[0] = (float)gx / MPU6050_GYRO_SEN - gyro_bias[0];
        ret.gyro[1] = (float)gy / MPU6050_GYRO_SEN - gyro_bias[1];
        ret.gyro[2] = (float)gz / MPU6050_GYRO_SEN - gyro_bias[2];

        ret.temp = (float)temp / 340.0f + 36.53f;
    }

    return ret;
}

void bsp_mpu6050_calibrate(uint16_t samples) {
    if (samples == 0) {
        return;
    }

    uint8_t buf[6];
    float sum[3] = {0};

    for (uint16_t i = 0; i < samples; i++) {
        if (read_reg(MPU6050_REG_GYRO_XOUT_H, buf, sizeof(buf))) {
            sum[0] += (float)(int16_t)((buf[0] << 8) | buf[1]) / MPU6050_GYRO_SEN;
            sum[1] += (float)(int16_t)((buf[2] << 8) | buf[3]) / MPU6050_GYRO_SEN;
            sum[2] += (float)(int16_t)((buf[4] << 8) | buf[5]) / MPU6050_GYRO_SEN;
        }
        HAL_Delay(1);
    }

    gyro_bias[0] = sum[0] / (float)samples;
    gyro_bias[1] = sum[1] / (float)samples;
    gyro_bias[2] = sum[2] / (float)samples;
}