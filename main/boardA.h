#ifndef BOARD_A_H
#define BOARD_A_H

#include <stdint.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define BOARD_A_UART_PORT      UART_NUM_1
#define BOARD_A_TX_PIN         GPIO_NUM_35   // ESP32-S3 TX -> Board A RX
#define BOARD_A_RX_PIN         GPIO_NUM_36   // ESP32-S3 RX <- Board A TX
#define BOARD_A_BAUDRATE      115200

typedef struct {
    uint8_t  b_state;
    uint8_t  actual_capture;
    uint8_t  err_code;
    uint8_t  storage_free_pct;
    uint32_t frame_count;
    uint32_t session_id;
    uint8_t  safe_power_off;
    uint8_t  rsv;
} payload_0x81_t; //状态上报(B->A) 14字节

extern payload_0x81_t payload_0x81;
typedef struct {
    //0xA0握手应答数据
    uint8_t proto_ver;
    uint16_t b_fw_ver;
    //0x81状态上报数据
    uint8_t  b_ready;
    uint8_t  actual_capture;
    uint8_t  err_code;
    uint8_t  storage_free_pct;
    uint32_t frame_count;
    uint32_t session_id;
    uint8_t  safe_power_off;
    //A板数据帧有效内容
    int32_t latitude;
    int32_t longitude;
    int32_t alt_rel;
    uint32_t utc_sec;
    //0x90通用应答数据
    uint8_t start_result;
    uint8_t stop_result;
    uint8_t power_result;
    //用于比对该帧是否有效
    uint8_t  last_rx_cmd;
    uint8_t  last_rx_seq;
}boardB_state;

extern bool SC16IS752_uart_flag;
extern boardB_state B_state;

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);
void boardA_init(void);
void boardA_cmd_process(uint8_t cmd, uint8_t seq, uint8_t *payload_buf);
#endif //BOARD_A_H