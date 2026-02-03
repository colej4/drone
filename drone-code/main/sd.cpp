#include <string.h>
#include <iostream>
#include <fstream>
using namespace std;

#include "esp_err.h"
#include "esp_log.h"


#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "freertos/FreeRTOS.h"

static const char *TAG = "sd";

#define MOUNT_POINT "/sdcard"

#define CS 19
#define SCK 18
#define MOSI 5
#define MISO 17

esp_err_t mount_sd(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = true;
    mount_config.max_files = 20;
    mount_config.allocation_unit_size = 16 * 1024;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = MOSI;
    bus_cfg.miso_io_num = MISO;
    bus_cfg.sclk_io_num = SCK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA));

    sdspi_device_config_t sdspi_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    sdspi_config.gpio_cs = (gpio_num_t)CS;
    sdspi_config.host_id = SPI2_HOST;

    const char mount_point[] = MOUNT_POINT;

    sdmmc_card_t *card;

    ESP_LOGI(TAG, "Mounting filesystem");
    ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(mount_point, &host, &sdspi_config, &mount_config, &card));

    ESP_LOGI(TAG, "Filesystem mounted");
    const char *file_test = MOUNT_POINT"/test.txt";
    ESP_LOGI(TAG, "Opening file %s", file_test);
    ofstream f;
    f.open(file_test);
    if (!f.is_open()) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    f << "Hello SD Card!" << std::endl;
    f.close();
    ESP_LOGI(TAG, "File written successfully");
    return ESP_OK;
}