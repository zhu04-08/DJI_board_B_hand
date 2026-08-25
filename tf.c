/*
 * tf.c - TF/SD Card SPI Driver Implementation
 * Targeted for ESP32-S3 (ESP-IDF)
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "tf.h"

static const char *TAG = "TF_CARD";

static sdmmc_card_t *s_card = NULL;
static bool s_is_mounted = false;

// 内部辅助函数：拼接生成正确的 ESP VFS 挂载绝对路径
static void get_full_path(const char *rel_path, char *full_path, size_t max_len)
{
    if (!rel_path || !full_path || max_len == 0) return;

    if (strncmp(rel_path, TF_MOUNT_POINT, strlen(TF_MOUNT_POINT)) == 0) {
        snprintf(full_path, max_len, "%s", rel_path);
    } else if (rel_path[0] == '/') {
        snprintf(full_path, max_len, "%s%s", TF_MOUNT_POINT, rel_path);
    } else {
        snprintf(full_path, max_len, "%s/%s", TF_MOUNT_POINT, rel_path);
    }
}

esp_err_t tf_init(void)
{
    if (s_is_mounted) {
        ESP_LOGW(TAG, "TF card is already mounted.");
        return ESP_OK;
    }

    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing TF Card via SPI Host %d...", TF_SPI_HOST);
    ESP_LOGI(TAG, "SPI Pin Config: CS=%d, MOSI=%d, CLK=%d, MISO=%d",
             TF_PIN_CS, TF_PIN_MOSI, TF_PIN_CLK, TF_PIN_MISO);

    // 1. FATFS 文件系统挂载配置
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true, // 若未格式化则自动格式化为 FAT32
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // 2. SPI 总线引脚配置
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = TF_PIN_MOSI,
        .miso_io_num = TF_PIN_MISO,
        .sclk_io_num = TF_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

#ifndef SDSPI_DEFAULT_DMA
#define SDSPI_DEFAULT_DMA SPI_DMA_CH_AUTO
#endif

    ret = spi_bus_initialize(TF_SPI_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus (%s)", esp_err_to_name(ret));
        return ret;
    }

    // 3. SDSPI 驱动接口配置
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = TF_SPI_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = TF_PIN_CS;
    slot_config.host_id = TF_SPI_HOST;

    // 4. 挂载 FAT 文件系统到 VFS
    ret = esp_vfs_fat_sdspi_mount(TF_MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount FAT filesystem. Formatting may be required.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card (%s). Please check physical connection and pull-up resistors.", esp_err_to_name(ret));
        }
        return ret;
    }

    s_is_mounted = true;
    ESP_LOGI(TAG, "TF Card mounted successfully at %s", TF_MOUNT_POINT);

    // 打印 TF 卡硬件属性信息
    sdmmc_card_print_info(stdout, s_card);

    return ESP_OK;
}

esp_err_t tf_deinit(void)
{
    if (!s_is_mounted) {
        ESP_LOGW(TAG, "TF card is not mounted.");
        return ESP_OK;
    }

    esp_err_t ret = esp_vfs_fat_sdcard_unmount(TF_MOUNT_POINT, s_card);
    if (ret == ESP_OK) {
        s_card = NULL;
        s_is_mounted = false;
        spi_bus_free(TF_SPI_HOST);
        ESP_LOGI(TAG, "TF Card unmounted successfully and SPI bus freed.");
    } else {
        ESP_LOGE(TAG, "Failed to unmount TF Card (%s)", esp_err_to_name(ret));
    }
    return ret;
}

bool tf_is_mounted(void)
{
    return s_is_mounted;
}

esp_err_t tf_write_file(const char *path, const char *data, size_t len)
{
    if (!s_is_mounted) {
        ESP_LOGE(TAG, "Cannot write file, TF card not mounted.");
        return ESP_ERR_INVALID_STATE;
    }

    char full_path[256];
    get_full_path(path, full_path, sizeof(full_path));

    ESP_LOGI(TAG, "Writing file: %s", full_path);
    FILE *f = fopen(full_path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", full_path);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, len, f);
    fclose(f);

    if (written != len) {
        ESP_LOGE(TAG, "Write length mismatch: expected %zu, actual %zu", len, written);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tf_append_file(const char *path, const char *data, size_t len)
{
    if (!s_is_mounted) {
        ESP_LOGE(TAG, "Cannot append file, TF card not mounted.");
        return ESP_ERR_INVALID_STATE;
    }

    char full_path[256];
    get_full_path(path, full_path, sizeof(full_path));

    FILE *f = fopen(full_path, "ab");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for appending: %s", full_path);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, len, f);
    fclose(f);

    if (written != len) {
        ESP_LOGE(TAG, "Append length mismatch: expected %zu, actual %zu", len, written);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tf_read_file(const char *path, char *buf, size_t max_len, size_t *out_len)
{
    if (!s_is_mounted) {
        ESP_LOGE(TAG, "Cannot read file, TF card not mounted.");
        return ESP_ERR_INVALID_STATE;
    }

    char full_path[256];
    get_full_path(path, full_path, sizeof(full_path));

    FILE *f = fopen(full_path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", full_path);
        return ESP_FAIL;
    }

    size_t read_bytes = fread(buf, 1, max_len, f);
    fclose(f);

    if (out_len) {
        *out_len = read_bytes;
    }

    return ESP_OK;
}

esp_err_t tf_list_dir(const char *path)
{
    if (!s_is_mounted) {
        ESP_LOGE(TAG, "Cannot list directory, TF card not mounted.");
        return ESP_ERR_INVALID_STATE;
    }

    char full_path[256];
    get_full_path(path, full_path, sizeof(full_path));

    ESP_LOGI(TAG, "Listing directory: %s", full_path);
    DIR *dir = opendir(full_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", full_path);
        return ESP_FAIL;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        char entry_path[512];
        snprintf(entry_path, sizeof(entry_path), "%s/%s", full_path, entry->d_name);

        struct stat st;
        if (stat(entry_path, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                ESP_LOGI(TAG, "  [DIR]  %s", entry->d_name);
            } else {
                ESP_LOGI(TAG, "  [FILE] %s (%ld bytes)", entry->d_name, st.st_size);
            }
        } else {
            ESP_LOGI(TAG, "  [ITEM] %s", entry->d_name);
        }
    }
    closedir(dir);

    return ESP_OK;
}

// ============================================================================
// 光谱数据存储专用 API 实现
// ============================================================================

esp_err_t tf_save_spectrum_csv(const char *filename, const spectrum_data_t *spec)
{
    if (!s_is_mounted || !spec || !spec->wavelengths || !spec->intensities) {
        ESP_LOGE(TAG, "Invalid arguments or card not mounted for CSV save.");
        return ESP_ERR_INVALID_ARG;
    }

    char full_path[256];
    get_full_path(filename, full_path, sizeof(full_path));

    FILE *f = fopen(full_path, "w");
if (!f) {
    // 增加 errno 和 strerror 打印
    ESP_LOGE(TAG, "Failed to create CSV file: %s (Errno: %d, Msg: %s)", 
             full_path, errno, strerror(errno));
    return ESP_FAIL;
}

    // 写入光谱配置及元数据头 (CSV Header)
    fprintf(f, "# Spectrum Data Export\n");
    fprintf(f, "# Sample ID: %s\n", spec->sample_id ? spec->sample_id : "Unknown");
    fprintf(f, "# Integration Time (ms): %lu\n", (unsigned long)spec->integration_ms);
    fprintf(f, "# Gain: %u\n", spec->gain);
    fprintf(f, "# Data Points: %zu\n", spec->num_points);
    if (spec->timestamp) {
        fprintf(f, "# Timestamp: %s\n", spec->timestamp);
    }
    fprintf(f, "Wavelength(nm),Intensity\n");

    // 逐行写入波长与数值
    for (size_t i = 0; i < spec->num_points; i++) {
        fprintf(f, "%.2f,%.4f\n", spec->wavelengths[i], spec->intensities[i]);
    }

    fclose(f);
    ESP_LOGI(TAG, "Saved spectrum CSV successfully: %s (%zu points)", full_path, spec->num_points);
    return ESP_OK;
}

esp_err_t tf_save_spectrum_binary(const char *filename, const spectrum_data_t *spec)
{
    if (!s_is_mounted || !spec || !spec->wavelengths || !spec->intensities) {
        ESP_LOGE(TAG, "Invalid arguments or card not mounted for BIN save.");
        return ESP_ERR_INVALID_ARG;
    }

    char full_path[256];
    get_full_path(filename, full_path, sizeof(full_path));

    FILE *f = fopen(full_path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to create binary spectrum file: %s (Errno: %d, Msg: %s)", 
                full_path, errno, strerror(errno));
        return ESP_FAIL;
    }

    // 写入魔数与数据头
    uint32_t magic = 0x53504543; // 标志位 'SPEC'
    uint32_t num_pts = (uint32_t)spec->num_points;
    fwrite(&magic, sizeof(uint32_t), 1, f);
    fwrite(&num_pts, sizeof(uint32_t), 1, f);
    fwrite(&spec->integration_ms, sizeof(uint32_t), 1, f);

    // 批量二进制写入波长与光强数据
    fwrite(spec->wavelengths, sizeof(float), spec->num_points, f);
    fwrite(spec->intensities, sizeof(float), spec->num_points, f);

    fclose(f);
    ESP_LOGI(TAG, "Saved spectrum BIN successfully: %s", full_path);
    return ESP_OK;
}

esp_err_t tf_append_spectrum_matrix_csv(const char *filename, const spectrum_data_t *spec, bool is_header)
{
    if (!s_is_mounted || !spec || !spec->intensities) {
        ESP_LOGE(TAG, "Invalid arguments or card not mounted for Matrix CSV append.");
        return ESP_ERR_INVALID_ARG;
    }

    char full_path[256];
    get_full_path(filename, full_path, sizeof(full_path));

    FILE *f = fopen(full_path, is_header ? "w" : "a");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open spectrum matrix file: %s", full_path);
        return ESP_FAIL;
    }

    // 若需要写表头，写入波长标题行
    if (is_header && spec->wavelengths) {
        fprintf(f, "Sample_ID,Timestamp,Integration_ms,Gain");
        for (size_t i = 0; i < spec->num_points; i++) {
            fprintf(f, ",%.2fnm", spec->wavelengths[i]);
        }
        fprintf(f, "\n");
    }

    // 追加一行单帧光谱完整数据
    fprintf(f, "%s,%s,%lu,%u",
            spec->sample_id ? spec->sample_id : "N/A",
            spec->timestamp ? spec->timestamp : "N/A",
            (unsigned long)spec->integration_ms,
            spec->gain);

    for (size_t i = 0; i < spec->num_points; i++) {
        fprintf(f, ",%.4f", spec->intensities[i]);
    }
    fprintf(f, "\n");

    fclose(f);
    return ESP_OK;
}