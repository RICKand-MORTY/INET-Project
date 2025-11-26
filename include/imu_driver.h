#pragma once
#include "stm32l4xx_hal.h"


extern I2C_HandleTypeDef hi2c1;

#define LSM6DSL_ADDR    (0x6A << 1)   // I2C 地址
#define WHO_AM_I_REG    0x0F
#define CTRL1_XL        0x10
#define CTRL2_G         0x11
#define CTRL3_C         0x12
#define OUTX_L_XL       0x28   // 加速度数据寄存器起始地址

// 三轴加速度结构体
typedef struct {
    float ax;
    float ay;
    float az;
} AccelData;

typedef struct {
    float gx;
    float gy;
    float gz;
} GyroData;

// 外部 I2C 句柄：请在工程的 HW 初始化代码中初始化并配置 hi2c1
extern I2C_HandleTypeDef hi2c1;

void imu_init(void);
// 初始化 IMU 寄存器（注意：本函数不创建或配置 I2C 硬件句柄，hi2c1 必须在外部初始化）

AccelData imu_read_accel(void);
GyroData imu_read_gyro(void);