/*
 * tf.h - TF/SD Card SPI Driver & Spectral Data Storage Interface
 * Targeted for ESP32-S3 (ESP-IDF)
 */

#ifndef TF_H
#define TF_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// SPI 引脚配置 (ESP32-S3)
// ============================================================================
#define TF_PIN_CS       GPIO_NUM_5   // CS (片选引脚)
#define TF_PIN_MOSI     GPIO_NUM_7   // MOSI (主出从入)
#define TF_PIN_CLK      GPIO_NUM_6   // CLK (时钟)
#define TF_PIN_MISO     GPIO_NUM_4   // MISO (主入从出)

// SPI 控制器选择 (ESP32-S3 包含 SPI2_HOST 和 SPI3_HOST 可用)
#define TF_SPI_HOST     SPI3_HOST    // 推荐使用 SPI3_HOST (独立于其他 SPI 外设)
#define TF_MOUNT_POINT  "/sdcard"    // VFS 挂载点

// ============================================================================
// 光谱数据结构体定义
// ============================================================================
typedef struct {
    const char *sample_id;      // 样品编号/名称 (例如 "Sample_A01")
    uint32_t integration_ms;    // 积分时间 (毫秒)
    uint16_t gain;              // 增益 / 放大倍数
    const float *wavelengths;   // 波长数据数组 (单位: nm, 如 350.0 ~ 1000.0)
    const float *intensities;   // 光强/采样数值数组
    size_t num_points;          // 采样点数 / 光谱通道数 (例如 2048)
    const char *timestamp;      // 时间戳或采集时间字符串 (可选, 传 NULL 忽略)
} spectrum_data_t;

// ============================================================================
// TF 卡通用文件操作 API
// ============================================================================

/**
 * @brief 初始化 TF 卡 (挂载 FATFS 文件系统)
 * @return ESP_OK 成功，其他为错误码
 */
esp_err_t tf_init(void);

/**
 * @brief 卸载 TF 卡并释放 SPI 资源
 * @return ESP_OK 成功
 */
esp_err_t tf_deinit(void);

/**
 * @brief 查询 TF 卡是否处于已挂载状态
 * @return true 已挂载，false 未挂载
 */
bool tf_is_mounted(void);

/**
 * @brief 写入文件 (覆盖写模式)
 * @param path 相对路径 (如 "test.txt") 或绝对路径 (如 "/sdcard/test.txt")
 * @param data 准备写入的数据内存指针
 * @param len 数据字节数
 * @return ESP_OK 成功
 */
esp_err_t tf_write_file(const char *path, const char *data, size_t len);

/**
 * @brief 追加数据到文件末尾
 * @param path 文件路径
 * @param data 追加数据指针
 * @param len 追加字节数
 * @return ESP_OK 成功
 */
esp_err_t tf_append_file(const char *path, const char *data, size_t len);

/**
 * @brief 读取文件内容
 * @param path 文件路径
 * @param buf 读取缓冲区
 * @param max_len 缓冲区容量
 * @param out_len 实际读取的字节数指针
 * @return ESP_OK 成功
 */
esp_err_t tf_read_file(const char *path, char *buf, size_t max_len, size_t *out_len);

/**
 * @brief 遍历并打印指定目录下的文件及文件夹
 * @param path 目录路径，如 "/" 或 "/sdcard"
 * @return ESP_OK 成功
 */
esp_err_t tf_list_dir(const char *path);

// ============================================================================
// 光谱数据存储专用 API
// ============================================================================

/**
 * @brief 保存单帧光谱数据为标准 CSV 文件 (纵向列表格式: 波长,光强)
 * @param filename 文件名 (如 "spectrum_001.csv")
 * @param spec 光谱数据指针
 * @return ESP_OK 成功
 */
esp_err_t tf_save_spectrum_csv(const char *filename, const spectrum_data_t *spec);

/**
 * @brief 保存单帧光谱数据为二进制文件 (适合高速写入及占用空间优化)
 * @param filename 文件名 (如 "spectrum_001.bin")
 * @param spec 光谱数据指针
 * @return ESP_OK 成功
 */
esp_err_t tf_save_spectrum_binary(const char *filename, const spectrum_data_t *spec);

/**
 * @brief 追加单帧光谱到矩阵 CSV 文件 (横向数据行格式，适合连续时序光谱记录)
 * @param filename 文件名 (如 "spectrum_matrix.csv")
 * @param spec 光谱数据指针
 * @param is_header 是否写入表头 (首帧传 true，后续帧传 false)
 * @return ESP_OK 成功
 */
esp_err_t tf_append_spectrum_matrix_csv(const char *filename, const spectrum_data_t *spec, bool is_header);

#ifdef __cplusplus
}
#endif

#endif // TF_H