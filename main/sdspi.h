#ifndef SDSPI_H
#define SDSPI_H

#include <stdint.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

#define TF_PIN_CS       GPIO_NUM_5   // CS (片选引脚)
#define TF_PIN_MOSI     GPIO_NUM_7   // MOSI (主出从入)
#define TF_PIN_CLK      GPIO_NUM_6   // CLK (时钟)
#define TF_PIN_MISO     GPIO_NUM_4   // MISO (主入从出)

extern int rx_tail_a;
extern int rx_tail_b;

typedef struct {
    char channel_id;                        // 通道 'A' 或 'B'
    uint8_t numble;                         // 帧序号
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
    float *intensities;                     // 光谱数据（动态分配）
}tf_save_msg_t;

tf_save_msg_t save_a;
tf_save_msg_t save_b;

#endif // SDSPI_H