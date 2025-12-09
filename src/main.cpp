#include <mbed.h>
#include "stm32l4xx_hal.h"
#include "imu_driver.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_rcc_ex.h"
#include "filter.h"
#include "fft_analysis.h"
#include <arm_math.h>
// ==== 新增：BLE 接口封装 ====（lyt修改）
#include "ble_service.h" 

I2C_HandleTypeDef hi2c2;   // I2C2
UART_HandleTypeDef huart1;
float32_t z_buffer[FFT_SIZE];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);

#define WINDOW_SEC      3           // 3s for window
#define WINDOW_SAMPLES  (SAMPLE_RATE * WINDOW_SEC) // 156点，需<=FFT_SIZE
static float32_t accel_buf[FFT_SIZE];   // buffer for accelerometer
static float32_t gyro_buf[FFT_SIZE];    // buffer for gyroscope
static float32_t fused_buf[FFT_SIZE];    // buffer for gyroscope
static int sample_idx = 0;

static int stationary_windows = 0; // count of consecutive no-step windows
static bool had_steps = false; // record if there were steps in previous window

// ==== 新增：三个症状 flag + 总体 state ====(BLE part)（lyt修改）
static int tremor_flag     = 0;   // 0/1: 是否检测到 tremor
static int dyskinesia_flag = 0;   // 0/1: 是否检测到 dyskinesia
static int fog_flag        = 0;   // 0/1: 是否检测到 FOG
static int state           = 0;   // 0/1/2/3：整体状态编码

int main(void)
{
    HAL_Init();
    //SystemClock_Config();

    //MX_GPIO_Init();
    MX_I2C2_Init();
    //MX_USART1_UART_Init();

    imu_init(); 

    // BLE Part
    // ==== BLE 初始化 ====
    // NOTE[TEAM-BLE]:
    //   这里调用我们封装好的 BLE 初始化函数：
    //   - 在 ble_service.cpp 中会完成 BLE.init()、添加 Service、开始广播
    //   - 手机上会看到一个名为 "PD-State" 的 BLE 设备
    ble_init();

    /*

    //Test the accelerometer

    while (1)
    {
        AccelData accel = imu_read_accel();
        accel = filter_accel_lowpass(accel);
        char buf[100];
        int len = snprintf(buf, sizeof(buf),
                           "AX=%.3f g, AY=%.3f g, AZ=%.3f g\r\n",
                           accel.ax, accel.ay, accel.az);
        //printf("%s", buf);
        fft_analyze(z_buffer, FFT_SIZE);
        HAL_Delay(20);
    }
    */

   /*
    //Test the gyroscope
    while (1) {
        AccelData accel = imu_read_accel();
        GyroData gyro   = imu_read_gyro();

        printf("AX=%.3f g, AY=%.3f g, AZ=%.3f g | GX=%.2f dps, GY=%.2f dps, GZ=%.2f dps\r\n",
               accel.ax, accel.ay, accel.az,
               gyro.gx, gyro.gy, gyro.gz);

        HAL_Delay(20);
    }
        */
     // 参数：融合权重
    const float alpha = 0.7f;  // 可调节：0.7 表示加速度计占主导

    while (1)
    {
        // Read accelerometer and gyroscope
        AccelData accel = imu_read_accel();
        GyroData  gyro  = imu_read_gyro();

        // Apply simple low-pass filter
        accel = filter_accel_lowpass(accel);

        // Compute magnitude of accelerometer vector
        float accel_mag = sqrtf(accel.ax * accel.ax +
                                accel.ay * accel.ay +
                                accel.az * accel.az);

        // Compute magnitude of gyroscope vector
        float gyro_mag = sqrtf(gyro.gx * gyro.gx +
                            gyro.gy * gyro.gy +
                            gyro.gz * gyro.gz);

        // --- 融合加速度计和陀螺仪 ---
        float fused_mag = alpha * accel_mag + (1.0f - alpha) * gyro_mag;

        // Store into buffer
        if (sample_idx < FFT_SIZE) {
            fused_buf[sample_idx] = fused_mag;  // 使用融合后的值
            sample_idx++;
        }

        // When one window is full, perform FFT analysis
        if (sample_idx >= WINDOW_SAMPLES) 
        {
            printf("=== Window analysis start ===\r\n");

            tremor_flag     = 0;
            dyskinesia_flag = 0;

            // --- Step 1: Stationary check ---
            bool stationary = is_stationary(fused_buf, WINDOW_SAMPLES);

            // --- Step 2: Tremor/Dyskinesia detection ---
            if (!stationary) {
                if (fft_compute(fused_buf, WINDOW_SAMPLES)) {
                    float trem_energy = fft_get_band_energy(2.0f, 3.0f);
                    float dysk_energy = fft_get_band_energy(4.0f, 5.0f);
                    float step_energy = fft_get_band_energy(3.0f, 5.0f); 
                    float total_energy = fft_get_band_energy(0.5f, 10.0f);

                    if (total_energy > 0.0f) {
                        printf("trem_energy / total_energy = %.3f, dysk_energy / total_energy = %.3f, step_energy / total_energy = %.3f\r\n",
                            trem_energy / total_energy,
                            dysk_energy / total_energy,
                            step_energy / total_energy);

                        if (trem_energy / total_energy > 0.1f) {
                            tremor_flag = 1;
                            printf("Tremor detected (3-5Hz)\r\n");
                        }
                        if (dysk_energy / total_energy > 0.1f) {
                            dyskinesia_flag = 1;
                            printf("Dyskinesia detected (5-7Hz)\r\n");
                        }
                        if (step_energy / total_energy > 0.2f) {
                            had_steps = true;
                            printf("Walking detected\r\n");
                        }
                    }
                }
            } else {
                printf("Stationary: skip tremor/dyskinesia detection\r\n");
            }

            // --- Step 3: FOG detection ---
            fog_flag = 0;
            if (had_steps) {
                if (stationary) {
                    stationary_windows++;
                    if (stationary_windows >= 2) {
                        printf("FOG detected (Freezing of Gait)\r\n");
                        fog_flag = 1;
                        had_steps = false;          
                        stationary_windows = 0;
                    }
                } else {
                    stationary_windows = 0; 
                }
            }

            // --- Step 4: 状态编码 ---
            if (fog_flag) {
                state = 3;
            } else if (tremor_flag) {
                state = 1;
            } else if (dyskinesia_flag) {
                state = 2;
            } else {
                state = 0;
            }

            // --- Step 5: BLE 广播 ---
            ble_update(state, tremor_flag, dyskinesia_flag, fog_flag);

            sample_idx = 0;
        }

        ble_process();
        HAL_Delay(1000 / SAMPLE_RATE);
    }
}

static void MX_I2C2_Init(void)
{
    // enable I2C2 clock
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PB10 = I2C2_SCL, PB11 = I2C2_SDA
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // init I2C2
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x00707CBB; // 400kHz
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        printf("I2C2 Init Error\r\n");
    }

    // 使能模拟滤波器
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        printf("I2C2 Analog Filter Error\r\n");
    }
}
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PB10 = I2C2_SCL, PB11 = I2C2_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

