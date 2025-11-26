#include <mbed.h>
#include "stm32l4xx_hal.h"
#include "imu_driver.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_rcc_ex.h"
#include "filter.h"
#include "fft_analysis.h"
#include <arm_math.h>

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
static int sample_idx = 0;

static int stationary_windows = 0; // count of consecutive no-step windows
static bool had_steps = false; // record if there were steps in previous window

int main(void)
{
    HAL_Init();
    //SystemClock_Config();

    //MX_GPIO_Init();
    MX_I2C2_Init();
    //MX_USART1_UART_Init();

    imu_init(); 

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

        // Store into buffer
        if (sample_idx < FFT_SIZE) {
            accel_buf[sample_idx] = accel_mag;
            gyro_buf[sample_idx]  = gyro_mag;
            sample_idx++;
        }

        // When one window is full, perform FFT analysis
        if (sample_idx >= WINDOW_SAMPLES) 
        {
            printf("=== Window analysis start ===\r\n");


            // --- Step 1: Stationary check ---
            bool stationary = is_stationary(accel_buf, WINDOW_SAMPLES);

            // --- Step 2: Tremor/Dyskinesia detection ---
            if (!stationary) {
                if (fft_compute(accel_buf, WINDOW_SAMPLES)) {
                    float trem_energy = fft_get_band_energy(3.0f, 5.0f);
                    float dysk_energy = fft_get_band_energy(5.0f, 7.0f);
                    float step_energy = fft_get_band_energy(1.0f, 2.5f); 
                    float total_energy = fft_get_band_energy(0.5f, 10.0f);

                    if (total_energy > 0.0f) {
                        if (trem_energy / total_energy > 0.1f) {
                            printf("Tremor detected (3-5Hz)\r\n");
                        }
                        if (dysk_energy / total_energy > 0.1f) {
                            printf("Dyskinesia detected (5-7Hz)\r\n");
                        }
                        if (step_energy / total_energy > 0.2f) {
                            had_steps = true; // walking detected in this window
                            printf("Walking detected\r\n");
                        }
                    }
                }
            } else {
                printf("Stationary: skip tremor/dyskinesia detection\r\n");
            }

            // --- Step 3: FOG detection ---
            // FOG detection logic: if there were steps in previous windows,
            if (had_steps) {
                if (stationary) {
                    stationary_windows++;
                    if (stationary_windows >= 2) { // stationary for 2 consecutive windows
                        printf("FOG detected (Freezing of Gait)\r\n");
                        had_steps = false;          
                        stationary_windows = 0;
                    }
                } else {
                    stationary_windows = 0; 
                }
            }
            // Reset sampling
            sample_idx = 0;
        }

        HAL_Delay(1000 / SAMPLE_RATE); // Control sampling frequency
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

