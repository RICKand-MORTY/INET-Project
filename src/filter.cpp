#include "imu_driver.h"

#define FILTER_LEN 8

/* Moving Average */
AccelData filter_accel_moving_average(AccelData in)
{
    static float ax_buf[FILTER_LEN] = {0};
    static float ay_buf[FILTER_LEN] = {0};
    static float az_buf[FILTER_LEN] = {0};
    static int idx = 0;

    AccelData out = {0};

    // store new data
    ax_buf[idx] = in.ax;
    ay_buf[idx] = in.ay;
    az_buf[idx] = in.az;
    idx = (idx + 1) % FILTER_LEN;

    // calculate average
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    for (int i = 0; i < FILTER_LEN; i++) {
        sum_ax += ax_buf[i];
        sum_ay += ay_buf[i];
        sum_az += az_buf[i];
    }

    out.ax = sum_ax / FILTER_LEN;
    out.ay = sum_ay / FILTER_LEN;
    out.az = sum_az / FILTER_LEN;

    return out;
}

#define ALPHA 0.1f  // filter coefficient

/*Exponential Moving Average*/
AccelData filter_accel_lowpass(AccelData in)
{
    static AccelData filtered = {0};
    AccelData out = {0};

    filtered.ax = filtered.ax + ALPHA * (in.ax - filtered.ax);
    filtered.ay = filtered.ay + ALPHA * (in.ay - filtered.ay);
    filtered.az = filtered.az + ALPHA * (in.az - filtered.az);

    out = filtered;
    return out;
}
