#include "fft_analysis.h"
#include <arm_math.h>
#include <stdio.h>

static float32_t fft_input[2 * FFT_SIZE];   // complex input: interleaved [real, imag]
static float32_t fft_output[FFT_SIZE];      // magnitude spectrum

/*
Perform FFT on the input signal and calculate the amplitude spectrum.
Supports length <= FFT_SIZE and automatically performs zero padding.
*/
bool fft_compute(const float32_t *input, int length)
{
    if (length <= 0) {
        printf("Invalid input length\r\n");
        return false;
    }

    // copy and zero-pad
    int copyN = (length < FFT_SIZE) ? length : FFT_SIZE;
    for (int i = 0; i < FFT_SIZE; i++) {
        float32_t x = (i < copyN) ? input[i] : 0.0f; // zero-padding
        fft_input[2*i]   = x;      // real
        fft_input[2*i+1] = 0.0f;   // imag
    }

    // initialize CFFT instance
    arm_cfft_instance_f32 cfft_instance;
    arm_cfft_init_f32(&cfft_instance, FFT_SIZE);
    arm_cfft_f32(&cfft_instance, fft_input, 0, 1);

    // compute magnitude spectrum
    arm_cmplx_mag_f32(fft_input, fft_output, FFT_SIZE);

    return true;
}

// Search for the maximum amplitude within the specified frequency range
float32_t fft_get_band_max(float32_t f_low, float32_t f_high)
{
    float32_t freq_res = (float32_t)SAMPLE_RATE / FFT_SIZE; // Δf
    int start_bin = (int)(f_low / freq_res);
    int end_bin   = (int)(f_high / freq_res);

    if (start_bin < 0) start_bin = 0;
    if (end_bin >= FFT_SIZE/2) end_bin = FFT_SIZE/2 - 1; // up to Nyquist

    float32_t max_val = 0.0f;
    for (int i = start_bin; i <= end_bin; i++) {
        if (fft_output[i] > max_val) {
            max_val = fft_output[i];
        }
    }
    return max_val;
}

// Tremor (3–5Hz)
bool detect_tremor(void)
{
    float32_t max_val = fft_get_band_max(3.0f, 5.0f);
    return (max_val > 3.0f); // threshold
}

//Dyskinesia (5–7Hz)
bool detect_dyskinesia(void)
{
    float32_t max_val = fft_get_band_max(5.0f, 7.0f);
    return (max_val > 3.0f); // threshold
}
/*
    detect if the signal is stationary based on variance (static condition)
*/
bool is_stationary(const float32_t *buf, int length)
{
    float mean = 0, var = 0;
    for (int i = 0; i < length; i++) mean += buf[i];
    mean /= length;
    for (int i = 0; i < length; i++) {
        float diff = buf[i] - mean;
        var += diff * diff;
    }
    var /= length;
    return (var < 0.01f); // threshold
}

/*
Sum up the values of the FFT amplitude spectrum within the specified 
frequency range to obtain the total energy of that range.
*/
float32_t fft_get_band_energy(float32_t f_low, float32_t f_high)
{
    float32_t freq_res = (float32_t)SAMPLE_RATE / FFT_SIZE;
    int start_bin = (int)(f_low / freq_res);
    int end_bin   = (int)(f_high / freq_res);
    if (end_bin >= FFT_SIZE/2) end_bin = FFT_SIZE/2 - 1;

    float32_t energy = 0.0f;
    for (int i = start_bin; i <= end_bin; i++) {
        energy += fft_output[i];
    }
    return energy;
}