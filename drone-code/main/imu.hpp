#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} imu_raw_data_t;

typedef struct {
    float ax, ay, az; // in m/s^2
    float gx, gy, gz; // in rad/s
    float temp_c;
} imu_data_t;

typedef struct {
    imu_data_t data;
    uint64_t timestamp; // in microseconds
} timestamped_imu_data_t;


void imu_i2c_init(void);
void imu_scan(void);
int  imu_init(void);
int  imu_read_raw(imu_raw_data_t *out);
imu_data_t imu_unit_convert(imu_raw_data_t *raw);
int  imu_read(imu_data_t *out);
void imu_task(void *arg);

#endif
