#ifndef SDSPI_H
#define SDSPI_H

#include <stdint.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

#include "sc16is752.h"

#define TF_PIN_CS       GPIO_NUM_5   // CS (片选引脚)
#define TF_PIN_MOSI     GPIO_NUM_7   // MOSI (主出从入)
#define TF_PIN_CLK      GPIO_NUM_6   // CLK (时钟)
#define TF_PIN_MISO     GPIO_NUM_4   // MISO (主入从出)

#define TF_SPI_HOST     SPI3_HOST
#define TF_MOUNT_POINT  "/sdcard"

extern FILE *g_fp_a;
extern FILE *g_fp_b;
extern uint32_t g_fc_a;
extern uint32_t g_fc_b;
extern bool tf_init_flag;

typedef struct {
    char channel_id;                        // 通道 'A' 或 'B'
    uint32_t frame_count;                   // 帧序号
    uint32_t utc_sec;                       // 时间戳
    float latitude, longitude, altitude;    // 位置信息
    uint8_t exposure_status;                // 曝光状态 0=正常 1=过曝 2=欠曝
    uint32_t exposure_time_us;              // 曝光时间(μs)
    float photometric[47];                  // 47个光度学参数
    float blue_light_hazard;                // 蓝光危害 Eb(W/m²)
    float nir[3];                           // 近红外参数
    float plant[16];                        // 植物光照参数
    int16_t spectral_coeff;                 // 光谱系数 N，实际值=原始值/10^N
    uint16_t num_points;                    // 光谱点数
    uint16_t *intensities;             //光谱数据
}tf_save_msg_t;

esp_err_t tf_init(void);
esp_err_t tf_deinit(void);
esp_err_t csv_init(void);
bool parse_spectrum_frame(const rx_frame_t *f,tf_save_msg_t *msg,uint16_t *spec_buf,uint16_t spec_buf_max);
void csv_write_row(FILE *fp, uint8_t channel,uint32_t frame_no,const tf_save_msg_t *msg,const uint16_t *spec, uint16_t npts);

#endif // SDSPI_H