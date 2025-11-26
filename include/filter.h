#pragma once
#include "imu_driver.h"

AccelData filter_accel_moving_average(AccelData in);

AccelData filter_accel_lowpass(AccelData in);
