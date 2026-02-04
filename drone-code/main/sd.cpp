#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/unistd.h>

#include "esp_err.h"
#include "esp_log.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "sd";

#define MOUNT_POINT "/sdcard"

#define CS   19
#define SCK  18
#define MOSI 5
#define MISO 17

#define LOG_LINE_MAX   256
#define LOG_QUEUE_LEN  128

typedef struct {
    uint16_t n;
    char line[LOG_LINE_MAX];
} log_item_t;

static QueueHandle_t   s_log_q = nullptr;
static TaskHandle_t    s_log_task = nullptr;
static vprintf_like_t  s_prev_vprintf = nullptr;
static volatile uint32_t s_dropped = 0;

static FILE *s_log_file = nullptr;

static int log_capture_vprintf(const char *fmt, va_list ap)
{
    log_item_t item;
    int n = vsnprintf(item.line, sizeof(item.line), fmt, ap);

    if (n < 0) n = 0;
    if (n >= (int)sizeof(item.line)) n = (int)sizeof(item.line) - 1;
    item.n = (uint16_t)n;

    if (s_log_q) {
        if (xQueueSend(s_log_q, &item, 0) != pdTRUE) {
            s_dropped++;
        }
    }

    return n;
}

static void sd_log_task(void *arg)
{
    const char *path = (const char *)arg;

    s_log_file = fopen(path, "a");
    if (!s_log_file) {
        vTaskDelete(nullptr);
        return;
    }

    setvbuf(s_log_file, nullptr, _IOFBF, 4096);

    log_item_t item;
    TickType_t last_flush = xTaskGetTickCount();

    while (true) {
        if (xQueueReceive(s_log_q, &item, pdMS_TO_TICKS(100))) {
            if (item.n) {
                fwrite(item.line, 1, item.n, s_log_file);
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_flush) >= pdMS_TO_TICKS(1000)) {
            fflush(s_log_file);
            fsync(fileno(s_log_file));
            last_flush = now;
        }
    }
}

static esp_err_t sd_log_start(const char *filepath)
{
    if (!filepath || filepath[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (s_log_task) return ESP_ERR_INVALID_STATE;

    s_log_q = xQueueCreate(LOG_QUEUE_LEN, sizeof(log_item_t));
    if (!s_log_q) return ESP_ERR_NO_MEM;

    s_prev_vprintf = esp_log_set_vprintf(log_capture_vprintf);

    BaseType_t ok = xTaskCreate(
        sd_log_task,
        "sd_log_task",
        4096,
        (void *)filepath,
        tskIDLE_PRIORITY,
        &s_log_task
    );

    if (ok != pdPASS) {
        esp_log_set_vprintf(s_prev_vprintf);
        s_prev_vprintf = nullptr;

        vQueueDelete(s_log_q);
        s_log_q = nullptr;
        s_log_task = nullptr;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t mount_sd_and_start_logging(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 8;
    mount_config.allocation_unit_size = 16 * 1024;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = MOSI;
    bus_cfg.miso_io_num = MISO;
    bus_cfg.sclk_io_num = SCK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 1000;

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    sdspi_device_config_t sdspi_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    sdspi_config.gpio_cs = (gpio_num_t)CS;
    sdspi_config.host_id = SPI2_HOST;

    sdmmc_card_t *card = nullptr;

    ESP_LOGI(TAG, "Mounting filesystem at %s", MOUNT_POINT);
    err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &sdspi_config, &mount_config, &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_vfs_fat_sdspi_mount failed: %s", esp_err_to_name(err));
        spi_bus_free(SPI2_HOST);
        return err;
    }

    ESP_LOGI(TAG, "Filesystem mounted");

    err = sd_log_start(MOUNT_POINT "/log.txt");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sd_log_start failed: %s", esp_err_to_name(err));
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        spi_bus_free(SPI2_HOST);
        return err;
    }

    ESP_LOGI(TAG, "SD log capture started: %s", MOUNT_POINT "/log.txt");
    return ESP_OK;
}
