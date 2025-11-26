#include "stm32l4xx_hal.h"
#include "imu_driver.h"

#define WHO_AM_I_REG     0x0F
#define CTRL1_XL         0x10
#define CTRL2_G          0x11
#define CTRL3_C          0x12
#define STATUS_REG       0x1E
#define OUTX_L_XL        0x28

#define LSM6DSL_ADDR     (0x6A << 1)

extern I2C_HandleTypeDef hi2c2;

void imu_init(void)
{
    uint8_t whoami = 0;
    HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, WHO_AM_I_REG,
                     I2C_MEMADD_SIZE_8BIT, &whoami, 1, HAL_MAX_DELAY);
    if (whoami != 0x6A) {
        return; // error
    }

    // reset device
    uint8_t ctrl3 = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, CTRL3_C,
                      I2C_MEMADD_SIZE_8BIT, &ctrl3, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // BDU=1, IF_INC=1
    ctrl3 = 0x48;
    HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, CTRL3_C,
                      I2C_MEMADD_SIZE_8BIT, &ctrl3, 1, HAL_MAX_DELAY);

    // config acc：ODR=52Hz, ±2g
    uint8_t accel_cfg = 0x30;
    HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, CTRL1_XL,
                      I2C_MEMADD_SIZE_8BIT, &accel_cfg, 1, HAL_MAX_DELAY);

    // config gyro：ODR=52Hz, ±250 dps
    uint8_t gyro_cfg = 0x30;
    HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, CTRL2_G,
                      I2C_MEMADD_SIZE_8BIT, &gyro_cfg, 1, HAL_MAX_DELAY);

    HAL_Delay(20);
}

AccelData imu_read_accel(void)
{
    AccelData out = {0};

    uint8_t status = 0;
    HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, STATUS_REG,
                     I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
    if ((status & 0x01) == 0) {
        return out; // no new data
    }

    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, OUTX_L_XL,
                     I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    int16_t raw_ax = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_ay = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_az = (int16_t)(data[5] << 8 | data[4]);

    const float s_g = 0.000061f; // g/LSB
    out.ax = raw_ax * s_g;
    out.ay = raw_ay * s_g;
    out.az = raw_az * s_g;

    return out;
}

#define OUTX_L_G         0x22   // gyro output register

GyroData imu_read_gyro(void)
{
    GyroData out = {0};

    uint8_t status = 0;
    HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, STATUS_REG,
                     I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
    if ((status & 0x02) == 0) {
        return out; // no new data
    }

    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, OUTX_L_G,
                     I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    int16_t raw_gx = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_gy = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_gz = (int16_t)(data[5] << 8 | data[4]);

    // sensitivity: ±250 dps → 8.75 mdps/LSB = 0.00875 dps/LSB
    const float s_dps = 0.00875f;
    out.gx = raw_gx * s_dps;
    out.gy = raw_gy * s_dps;
    out.gz = raw_gz * s_dps;

    return out;
}