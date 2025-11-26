## NYU INET-Group 40

**Member:** 

Lyu, Yitan

Min, Tianzan

Pan, Jason

Qu, Weikai

Raj, Shivangi



### TODO List

- [x] IMU (Accelerometer, gyroscope)

- [x] FFT

- [x] Detect tremors

- [x] Detect dyskinesia

- [x] Detect FOG

- [ ] Robustness of detection

- [ ] Bluetooth

  ...

  

### Directory and File

include: Hearders

lib: CSMIS-DSP

Src: source files

​	fft_analysis.cpp: FFT achievements

​	filter.cpp: Moving average and other filter

​	imu_drier.cpp: Setup IMU and read accelerometer and gyroscope

​	main.cpp: Main function



### Description

In the main function, first initialize HAL, I2C, and IMU. Then, perform a looping detection, analyzing data every 3 seconds. If the sample variance is below the threshold, it is determined to be stationary, and tremor/dyskinesia analysis is skipped. Otherwise, tremor and dyskinesia are assessed based on the energy of the windowed data. For FOG detection, a gait pattern must first be detected. If the subject remains stationary within consecutive windows, it indicates the occurrence of FOG.
