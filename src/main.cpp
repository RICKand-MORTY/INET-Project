#include "mbed.h"
#include "lsm6dsl_reg.h"
#include "arm_math.h"

#define ARM_MATH_CM4
#define __FPU_PRESENT 1

constexpr int SAMPLE_SIZE = 256;  // FFT sample size
constexpr int BUFFER_SIZE = 156; // 52Hz * 3 seconds = 156 samples

static float32_t accel_magnitude[BUFFER_SIZE] = {0}; // Circular buffer for 3 seconds of data
static float32_t fft_input[SAMPLE_SIZE];
static float32_t output[2 * SAMPLE_SIZE];

DigitalOut led_tremor(PB_14);     // LED for tremor detection
DigitalOut led_dyskinesia(PA_5);   // LED for dyskinesia detection

DigitalOut led_strong(PC_9);    // LED for strong signal detection

// I2C & Serial
I2C i2c(PB_11, PB_10);     // I2C2: SDA = PB11, SCL = PB10
BufferedSerial pc(USBTX, USBRX, 115200);

// FileHandle *mbed::mbed_override_console(int) {
//     return &pc;
// }

// LSM6DSL
#define LSM6DSL_I2C_ADDR  0x6A  // 7-bit I2C address
stmdev_ctx_t dev_ctx;

// Generic I2C read/write functions
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    char data[17];
    if (len > 16) return -1;
    data[0] = reg;
    memcpy(&data[1], bufp, len);
    return i2c.write(((int)(intptr_t)handle << 1), data, len + 1) == 0 ? 0 : -1;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    char reg_addr = reg;
    if (i2c.write(((int)(intptr_t)handle << 1), &reg_addr, 1, true) != 0) return -1;
    return i2c.read(((int)(intptr_t)handle << 1), (char*)bufp, len) == 0 ? 0 : -1;
}

void analyze_motion(const float* magnitudes, int sample_size, float sampling_rate);

void test_fft_accelerometer() {
    int16_t data_raw[3];
    int buffer_index = 0;

    printf("Initializing FFT on live accelerometer data...\n");

    // Setup sensor context
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = (void*)LSM6DSL_I2C_ADDR;

    thread_sleep_for(500); // Wait for the sensor to stabilize

    uint8_t whoami = 0;
    if (lsm6dsl_device_id_get(&dev_ctx, &whoami) != 0 || whoami != 0x6A) {
        printf("Device not found or ID mismatch. WHO_AM_I = 0x%X\n", whoami);
        return;
    }

    lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    uint8_t rst;
    do {
        lsm6dsl_reset_get(&dev_ctx, &rst);
    } while (rst);

    lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_52Hz);
    lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);

    printf("Starting continuous motion detection...\n");

    while (true) {
        lsm6dsl_status_reg_t status;
        lsm6dsl_status_reg_get(&dev_ctx, &status);
        if (status.xlda) {
            lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw);
            float x = lsm6dsl_from_fs2g_to_mg(data_raw[0]) / 1000.0f;
            float y = lsm6dsl_from_fs2g_to_mg(data_raw[1]) / 1000.0f;
            float z = lsm6dsl_from_fs2g_to_mg(data_raw[2]) / 1000.0f;
            float magnitude = sqrtf(x * x + y * y + z * z);

            // Update circular buffer
            accel_magnitude[buffer_index] = magnitude;
            buffer_index = (buffer_index + 1) % BUFFER_SIZE;

            // Prepare FFT input (last SAMPLE_SIZE samples from the buffer)
            for (int i = 0; i < SAMPLE_SIZE; i++) {
                int index = (buffer_index + BUFFER_SIZE - SAMPLE_SIZE + i) % BUFFER_SIZE;

                // Check for buffer index out of bounds
                 if (index < 0 || index >= BUFFER_SIZE) {
                    printf("Buffer index out of bounds: %d (buffer_index: %d, i: %d)\n", index, buffer_index, i);
                    return;
                }

                fft_input[i] = accel_magnitude[index];
            }

            // Perform FFT
            arm_rfft_fast_instance_f32 fft_instance;
            if (arm_rfft_fast_init_f32(&fft_instance, SAMPLE_SIZE) != ARM_MATH_SUCCESS) {
                printf("FFT initialization failed.\n");
                return;
            }

            arm_rfft_fast_f32(&fft_instance, fft_input, output, 0);

            // Calculate magnitudes
            float magnitudes[SAMPLE_SIZE / 2];
            for (int i = 0; i < SAMPLE_SIZE / 2; i++) {
                float real = output[2 * i];
                float imag = output[2 * i + 1];
                magnitudes[i] = sqrtf(real * real + imag * imag);
            }

            // Analyze motion
            analyze_motion(magnitudes, SAMPLE_SIZE, 52.0f);
        }

        thread_sleep_for(20); // Sampling interval (approx. 52Hz)
    }
}


void analyze_motion(const float* magnitudes, int sample_size, float sampling_rate) {
    float frequency_resolution = sampling_rate / sample_size;

    // Thresholds (tunable based on real-world tests)
    float tremor_threshold = 14.0f;
    float dyskinesia_threshold = 20.0f;
    float peak_threshold = 80.0f;
    int tremor_count = 0;
    int dyskinesia_count = 0;

    float peak_amplitude_tremor = 0.0f;      // Peak amplitude in Tremor range
    float peak_amplitude_dyskinesia = 0.0f;  // Peak amplitude in Dyskinesia range

    for (int i = 1; i < sample_size / 2; i++) {
        float freq = i * frequency_resolution;
        float amp = magnitudes[i];

        if (freq >= 3.0f && freq <= 5.0f) {   // 3-5 Hz
            if (amp >= tremor_threshold) {
                tremor_count++;
            }
            if (amp > peak_amplitude_tremor) {
                peak_amplitude_tremor = amp;
            }
        } else if (freq > 5.0f && freq <= 7.0f) {    // 5-7 Hz
            if (amp > peak_amplitude_dyskinesia) {
                peak_amplitude_dyskinesia = amp;
            }
            if (amp >= dyskinesia_threshold) {
                dyskinesia_count++;
            }
        }
    }

    if (tremor_count >= 2) {
        // printf("Tremor Detected\n");
        led_tremor = 1; // Turn on tremor LED
        led_dyskinesia = 0; // Turn off dyskinesia LED
    } else if (dyskinesia_count >= 3) {
        // printf("Dyskinesia Detected\n");
        led_tremor = 1;
        led_dyskinesia = 1; 
    } else {
        // printf("No abnormal motion detected\n");
        led_tremor = 0; // Turn off tremor LED
        led_dyskinesia = 0; // Turn off dyskinesia LED
        led_strong = 0; // Turn led for strong signal off
    }

    if (peak_amplitude_tremor >= peak_threshold || peak_amplitude_dyskinesia >= peak_threshold) {
        led_strong = 1; // Turn on strong signal LED
    } else {
        led_strong = 0; // Turn off strong signal LED
    }
}

int main() {
    thread_sleep_for(1000);
    printf("FFT Start\n");

    while (true) {
        test_fft_accelerometer();
        thread_sleep_for(10); 
    }
}