#ifndef CHASSIS_IMU_H
#define CHASSIS_IMU_H

#include <stdint.h>

#define CHASSIS_LSM6DS3TR_C 1
#define CHASSIS_MPU6500     2
#ifndef CHASSIS_IMU
#define CHASSIS_IMU CHASSIS_LSM6DS3TR_C
#endif

#if CHASSIS_IMU == CHASSIS_LSM6DS3TR_C
#define IMU_G_PER_COUNT   0.000061f
#define IMU_DPS_PER_COUNT 0.00875f
#elif CHASSIS_IMU == CHASSIS_MPU6500
#define IMU_G_PER_COUNT   (1.0f / 16384.0f)
#define IMU_DPS_PER_COUNT (1.0f / 131.0f)
#else
#error Unknown CHASSIS_IMU selection
#endif

typedef struct {
    uint32_t seq, read_tick_ms;
    int16_t gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw, temp_raw;
} ChassisImuSample;

/* Task context. Returns 1 once a complete sample is available, including during bias collection. */
int ChassisImu_GetLatest(ChassisImuSample *sample);

#endif
