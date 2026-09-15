#include <stdio.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "sdspi.h"
#include "boardA.h"
#include "sc16is752.h"

static const char *TAG = "TF_CARD";
static sdmmc_card_t *s_card = NULL;
FILE *g_fp_a = NULL;
FILE *g_fp_b = NULL;
uint32_t g_fc_a  = 0;
uint32_t g_fc_b  = 0;

bool tf_init_flag = false;

esp_err_t tf_init(void)
{
    if(tf_init_flag){
        return ESP_OK;
    }
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,  //调试阶段自动格式化为FAT32
        .max_files = 4,                  //读写分别占用2个，最大占用4个
        .allocation_unit_size = 16 *1024 //16KB读写快
    };
    spi_bus_config_t bus_config = {
        .mosi_io_num = TF_PIN_MOSI,
        .miso_io_num = TF_PIN_MISO,
        .sclk_io_num = TF_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2048,
    };
    esp_err_t ret;
    ret = spi_bus_initialize(TF_SPI_HOST,&bus_config,SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus (%s)", esp_err_to_name(ret));
        return ret;
    }
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = TF_SPI_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = TF_PIN_CS;
    slot_config.host_id = TF_SPI_HOST;

    ret = esp_vfs_fat_sdspi_mount(TF_MOUNT_POINT,&host,&slot_config,&mount_config,&s_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount FAT filesystem. Formatting may be required.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card (%s). Please check physical connection and pull-up resistors.", esp_err_to_name(ret));
        }
        return ret;
    }
    tf_init_flag = true;
    return ESP_OK;
}

esp_err_t tf_deinit(void)
{
    if (!tf_init_flag){
        return ESP_OK;
    }
    if (g_fp_a) { fflush(g_fp_a); fclose(g_fp_a); g_fp_a = NULL; }
    if (g_fp_b) { fflush(g_fp_b); fclose(g_fp_b); g_fp_b = NULL; }
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(TF_MOUNT_POINT,s_card);
    if (ret == ESP_OK) {
        s_card = NULL;
        tf_init_flag = false;
        spi_bus_free(TF_SPI_HOST);
        ESP_LOGI(TAG, "TF Card unmounted successfully and SPI bus freed.");
    } else {
        ESP_LOGE(TAG, "Failed to unmount TF Card (%s)", esp_err_to_name(ret));
    }
    return ret;
}

static void csv_write_header(FILE *fp, uint16_t wl_start, uint16_t wl_end)
{
    if (!fp) return;

    fprintf(fp, "channel_id,frame_count,utc_sec,"
                "latitude,longitude,altitude,"
                "exposure_status,exposure_time_us,");

    for (int i = 0; i < 47; i++) fprintf(fp, "photometric_%d,", i);

    fprintf(fp, "blue_light_hazard,");

    for (int i = 0; i < 3;  i++) fprintf(fp, "nir_%d,",  i);
    for (int i = 0; i < 16; i++) fprintf(fp, "plant_%d,", i);

    fprintf(fp, "spectral_coeff,");

    /* 波长列：起止波长，步进 1nm */
    for (int wl = wl_start; wl <= wl_end; wl++) {
        if (wl == wl_end) fprintf(fp, "%dnm\n", wl);
        else              fprintf(fp, "%dnm,", wl);
    }

    fflush(fp);
}

/* ---------- 打开文件并写表头 ---------- */
esp_err_t csv_init(void)
{
    if (!tf_init_flag) {
        ESP_LOGE(TAG, "TF not ready");
        return ESP_FAIL;
    }
    if (wavelength_start[0] == 0 || wavelength_end[0] == 0 ||
        wavelength_start[1] == 0 || wavelength_end[1] == 0) {
        ESP_LOGE(TAG, "wavelength not ready");
        return ESP_FAIL;
    }

    g_fp_a = fopen("/sdcard/A.csv", "a");
    g_fp_b = fopen("/sdcard/B.csv", "a");

    if (!g_fp_a || !g_fp_b) {
        ESP_LOGE(TAG, "fopen csv failed");
        if (g_fp_a) fclose(g_fp_a);
        if (g_fp_b) fclose(g_fp_b);
        g_fp_a = g_fp_b = NULL;
        return ESP_FAIL;
    }

    //文件为空才写表头
    fseek(g_fp_a, 0, SEEK_END);
    if(ftell(g_fp_a) == 0){
        csv_write_header(g_fp_a, wavelength_start[0], wavelength_end[0]);
    }
    fseek(g_fp_b, 0, SEEK_END);
    if(ftell(g_fp_b) == 0){
        csv_write_header(g_fp_b, wavelength_start[1], wavelength_end[1]);
    }

    g_fc_a = 0;
    g_fc_b = 0;

    ESP_LOGI(TAG, "A.csv header: %u~%unm, B.csv header: %u~%unm",
             wavelength_start[0], wavelength_end[0],
             wavelength_start[1], wavelength_end[1]);
    return ESP_OK;
}

/* 写一行 CSV */
void csv_write_row(FILE *fp, uint8_t channel,uint32_t frame_no,const tf_save_msg_t *msg,const uint16_t *spec, uint16_t npts)
{
    /* ---- 固定字段 ---- */
    fprintf(fp, "%c,%" PRIu32 ",%" PRIu32 ","
                "%.6f,%.6f,%.2f,"
                "%u,%" PRIu32 ",",
            (channel == 0) ? 'A' : 'B',
            frame_no,
            B_state.utc_sec,             // 来自 B_state
            (double)B_state.latitude,            // 来自 B_state
            (double)B_state.longitude,           // 来自 B_state
            (double)B_state.alt_rel,     // 来自 B_state
            msg->exposure_status,
            msg->exposure_time_us);

    /* ---- 47 个光度学参数 ---- */
    for (int i = 0; i < 47; i++)
        fprintf(fp, "%.4f,", msg->photometric[i]);

    /* ---- 蓝光危害 ---- */
    fprintf(fp, "%.4f,", msg->blue_light_hazard);

    /* ---- 近红外 3 个 ---- */
    for (int i = 0; i < 3; i++)
        fprintf(fp, "%.4f,", msg->nir[i]);

    /* ---- 植物光照 16 个 ---- */
    for (int i = 0; i < 16; i++)
        fprintf(fp, "%.4f,", msg->plant[i]);

    /* ---- 光谱系数 ---- */
    fprintf(fp, "%d,", msg->spectral_coeff);

    /* ---- 光谱数据（原始 uint16） ---- */
    for (int i = 0; i < npts; i++) {
        if (i == npts - 1) fprintf(fp, "%u\n", spec[i]);
        else               fprintf(fp, "%u,",  spec[i]);
    }
}

bool parse_spectrum_frame(const rx_frame_t *f,tf_save_msg_t *msg,uint16_t *spec_buf,uint16_t spec_buf_max)
{
    if (f->len < 9) return false;

    const uint8_t *p   = f->data + 6;
    size_t         len = f->len - 9;
    size_t         off = 0;

    msg->channel_id = (f->channel == 0) ? 'A' : 'B';

    if (off + 1 > len) return false;
    msg->exposure_status = p[off]; off += 1;

    if (off + 4 > len) return false;
    memcpy(&msg->exposure_time_us, p + off, 4); off += 4;

    if (off + 47*4 > len) return false;
    memcpy(msg->photometric, p + off, 47*4); off += 47*4;

    if (off + 4 > len) return false;
    memcpy(&msg->blue_light_hazard, p + off, 4); off += 4;

    if (off + 3*4 > len) return false;
    memcpy(msg->nir, p + off, 3*4); off += 3*4;

    if (off + 16*4 > len) return false;
    memcpy(msg->plant, p + off, 16*4); off += 16*4;

    if (off + 2 > len) return false;
    memcpy(&msg->spectral_coeff, p + off, 2); off += 2;

    size_t remain = len - off;
    if (remain < 2 || (remain & 1)) return false;
    uint16_t npts = remain / 2;
    if (npts > spec_buf_max) return false;

    for (uint16_t i = 0; i < npts; i++) {
        memcpy(&spec_buf[i], p + off, 2);
        off += 2;
    }

    msg->num_points  = npts;
    msg->intensities = spec_buf;   // 指向调用者缓冲，不拷贝

    msg->utc_sec   = B_state.utc_sec;
    msg->latitude  = B_state.latitude;
    msg->longitude = B_state.longitude;
    msg->altitude  = B_state.alt_rel;
    return true;
}