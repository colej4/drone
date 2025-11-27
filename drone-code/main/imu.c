#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "imu.h"
#include "math_helpers.h"

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SDA_IO      21
#define I2C_MASTER_SCL_IO      22
#define I2C_MASTER_FREQ_HZ     100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS  100

#define MPU6050_ADDR           0x68


static esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        dev_addr,
        buf,
        sizeof(buf),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
}

static esp_err_t i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        dev_addr,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
}

// ====== Public API ======

void imu_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0
    ));

    printf("I2C bus initialized on SDA=%d SCL=%d\n", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
}

int imu_init(void)
{
    // Small delay after power-up
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up device: PWR_MGMT_1 (0x6B) = 0x00
    if (i2c_write_reg(MPU6050_ADDR, 0x6B, 0x00) != ESP_OK) return -1;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Sample rate divider: SMPLRT_DIV (0x19) = 0x00 → 1 kHz when DLPF enabled
    if (i2c_write_reg(MPU6050_ADDR, 0x19, 0x00) != ESP_OK) return -1;

    // DLPF config: CONFIG (0x1A) = 0x03 → ~42 Hz
    if (i2c_write_reg(MPU6050_ADDR, 0x1A, 0x03) != ESP_OK) return -1;

    // Gyro config: GYRO_CONFIG (0x1B) = 0x18 → ±2000 dps
    if (i2c_write_reg(MPU6050_ADDR, 0x1B, 0x18) != ESP_OK) return -1;

    // Accel config: ACCEL_CONFIG (0x1C) = 0x08 → ±4g
    if (i2c_write_reg(MPU6050_ADDR, 0x1C, 0x08) != ESP_OK) return -1;

    vTaskDelay(pdMS_TO_TICKS(10));


    return 0;
}

int imu_read_raw(imu_raw_data_t *out)
{
    if (!out) return -1;

    uint8_t buf[14] = {0};
    esp_err_t err = i2c_read_regs(MPU6050_ADDR, 0x3B, buf, sizeof(buf));
    if (err != ESP_OK) {
        printf("imu_read_raw failed: %s\n", esp_err_to_name(err));
        return -1;
    }

    out->ax   = (int16_t)((buf[0] << 8) | buf[1]);
    out->ay   = (int16_t)((buf[2] << 8) | buf[3]);
    out->az   = (int16_t)((buf[4] << 8) | buf[5]);
    out->temp = (int16_t)((buf[6] << 8) | buf[7]);
    out->gx   = (int16_t)((buf[8] << 8) | buf[9]);
    out->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    out->gz   = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;
}

imu_data_t imu_unit_convert(imu_raw_data_t *raw)
{
    imu_data_t data;

    const float accel_scale = G / 8192.0f; // m/s² per LSB
    data.ax = raw->ax * accel_scale;
    data.ay = raw->ay * accel_scale;
    data.az = raw->az * accel_scale;

    // Gyroscope: ±2000 dps → 16.4 LSB/dps
    const float gyro_scale = (M_PI / 180.0f) / 16.4f; // rad/s per LSB
    data.gx = raw->gx * gyro_scale;
    data.gy = raw->gy * gyro_scale;
    data.gz = raw->gz * gyro_scale;

    // Temperature: Temp (°C) = (raw_temp / 340) + 36.53
    data.temp_c = (raw->temp / 340.0f) + 36.53f;

    return data;
}

int imu_read(imu_data_t *out)
{
    if (!out) return -1;

    imu_raw_data_t raw;
    if (imu_read_raw(&raw) != 0) {
        return -1;
    }

    *out = imu_unit_convert(&raw);
    return 0;
}

void imu_task(void *arg)
{
    imu_data_t data;

    while (1) {
        if (imu_read(&data) == 0) {
            printf("ACC: %6f %6f %6f  GYRO: %6f %6f %6f  TEMP: %6f\n",
                   data.ax, data.ay, data.az,
                   data.gx, data.gy, data.gz,
                   data.temp_c);
        }
        vTaskDelay(pdMS_TO_TICKS(1));   // 10 Hz
    }
}