#pragma once
#include <arm_math.h>

#define SAMPLE_RATE   52       // sample rate (Hz)
#define FFT_SIZE      256      // 2^N points FFT

bool fft_compute(const float32_t *input, int length);
float32_t fft_get_band_max(float32_t f_low, float32_t f_high);
bool detect_tremor(void);
bool detect_dyskinesia(void);
bool is_stationary(const float32_t *buf, int length);
float32_t fft_get_band_energy(float32_t f_low, float32_t f_high);
