#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/timer.h"

#include "imu.h"
#include "math_helpers.h"

//esp logging
#include "esp_log.h"
static const char* TAG = "imu";

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SDA_IO      21
#define I2C_MASTER_SCL_IO      22
#define I2C_MASTER_FREQ_HZ     100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS  100

#define ACCEL_BIAS_X 0.504476
#define ACCEL_BIAS_Y 0.024793
#define ACCEL_BIAS_Z 0.042967

#define MPU6050_ADDR           0x68

static float accel_scale_bias = 0.0f;
static Quaternion imu_to_drone_quat = (Quaternion){0.0f, 0.0f, 0.0f, 0.0f};
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;


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

    ESP_LOGI(TAG, "I2C bus initialized on SDA=%d SCL=%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
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
        ESP_LOGE(TAG, "imu_read_raw failed: %s", esp_err_to_name(err));
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

void apply_bias_correction(imu_data_t* data)
{
    data->ax -= ACCEL_BIAS_X;
    data->ay -= ACCEL_BIAS_Y;
    data->az -= ACCEL_BIAS_Z;
}

int imu_read(imu_data_t *out)
{
    if (!out) return -1;

    imu_raw_data_t raw;
    if (imu_read_raw(&raw) != 0) {
        return -1;
    }

    *out = imu_unit_convert(&raw);
    apply_bias_correction(out);
    return 0;
}

void calibrate_imu() {
    const int num_samples = 4000;
    imu_data_t data;
    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f, 
          gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    ESP_LOGI(TAG, "Calibrating IMU");

    for (int i = 0; i < num_samples; i++) {
        if (imu_read(&data) == 0) {
            ax_sum += data.ax;
            ay_sum += data.ay;
            az_sum += data.az;
            gx_sum += data.gx;
            gy_sum += data.gy;
            gz_sum += data.gz;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }


    float ax = ax_sum / num_samples;
    float ay = ay_sum / num_samples;
    float az = az_sum / num_samples;

    gyro_bias_x = gx_sum / num_samples;
    gyro_bias_y = gy_sum / num_samples;
    gyro_bias_z = gz_sum / num_samples;

    float mag = sqrtf(ax*ax + ay*ay + az*az);
    float gx = ax/mag, gy = ay/mag, gz = az/mag;
    accel_scale_bias = mag / G;
    const float ux=0, uy=0, uz=1; //unit vec up

    // Cross and dot
    float cx = gy*uz - gz*uy;
    float cy = gz*ux - gx*uz;
    float cz = gx*uy - gy*ux;
    float dot = gx*ux + gy*uy + gz*uz;

    float axis_mag = sqrtf(cx*cx + cy*cy + cz*cz);

    if (axis_mag < 1e-6f) {
        imu_to_drone_quat.w = 1;
        imu_to_drone_quat.x = 0;
        imu_to_drone_quat.y = 0;
        imu_to_drone_quat.z = 0;
    } else {
        float angle = acosf(dot);

        cx /= axis_mag;
        cy /= axis_mag;
        cz /= axis_mag;

        float s = sinf(angle/2.0f);
        imu_to_drone_quat.w = cosf(angle/2.0f);
        imu_to_drone_quat.x = cx * s;
        imu_to_drone_quat.y = cy * s;
        imu_to_drone_quat.z = cz * s;
    }

    ESP_LOGD(TAG, "Quaternion to rotate IMU to drone frame: w=%f x=%f y=%f z=%f",
           imu_to_drone_quat.w,
           imu_to_drone_quat.x,
           imu_to_drone_quat.y,
           imu_to_drone_quat.z);
    ESP_LOGD(TAG, "Gyro biases: gx=%f gy=%f gz=%f",
           gyro_bias_x,
           gyro_bias_y,
           gyro_bias_z);
    ESP_LOGD(TAG, "Accel scale bias: %f", accel_scale_bias);
    ESP_LOGI(TAG, "IMU calibration complete.");
}



int imu_read_calibrated(imu_data_t *out)
{
    if (!out) return -1;

    imu_raw_data_t raw;
    if (imu_read_raw(&raw) != 0) {
        return -1;
    }

    *out = imu_unit_convert(&raw);
    apply_bias_correction(out);
    out->ax /= accel_scale_bias;
    out->ay /= accel_scale_bias;
    out->az /= accel_scale_bias;


    Vector3 accel_vec = {out->ax, out->ay, out->az};

    accel_vec = quatrotate(imu_to_drone_quat, accel_vec);

    out->ax = accel_vec.x;
    out->ay = accel_vec.y;
    out->az = accel_vec.z;

    Vector3 gyro_vec = {out->gx - gyro_bias_x, out->gy - gyro_bias_y, out->gz - gyro_bias_z};
    gyro_vec = quatrotate(imu_to_drone_quat, gyro_vec);
    out->gx = gyro_vec.x;
    out->gy = gyro_vec.y;
    out->gz = gyro_vec.z;

    return 0;
}



void imu_task(void *arg)
{
    imu_data_t data;

    QueueHandle_t send_queue = *(QueueHandle_t*)arg;


    calibrate_imu();

    while (1) {
        if (imu_read_calibrated(&data) == 0) {
            timestamped_imu_data_t ts_data;
            ts_data.data = data;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &ts_data.timestamp);
            // data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
            BaseType_t result = xQueueSendToBack(send_queue, &ts_data, 0);
            if (result != pdTRUE) {
                ESP_LOGW(TAG, "IMU task: queue full");
            }
        } else {
            ESP_LOGE(TAG, "IMU read error");
        }

        vTaskDelay(pdMS_TO_TICKS(1));   // 1000 Hz
    }
}