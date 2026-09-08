#include <stdio.h>

#include "esp_log.h"
#include "boardA.h"

static boardB_state B_state = {
    .proto_ver = 0x01,
    .b_fw_ver = 0x01,
    .b_ready = 2,               // 默认故障
    .actual_capture = 0,        // 默认未采集
    .err_code = 0,              //默认无故障
    .storage_free_pct = 100,     // 默认剩余 100% 空间
    .frame_count = 0,           //默认已采集帧数为0
    .session_id = 0,            //默认无任务
    .safe_power_off = 0,        //默认不可断电
    .last_rx_cmd = 0xFF,
    .last_rx_seq = 0xFF
};

//CRC16算法
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    return crc;
}


void boardA_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = BOARD_A_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(BOARD_A_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_A_UART_PORT, BOARD_A_TX_PIN, BOARD_A_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(BOARD_A_UART_PORT, 1024, 512, 0, NULL, 0));
}

//指令处理函数
void boardA_cmd_process(uint8_t cmd, uint8_t seq, uint8_t *payload_buf)
{
    uint8_t tx_buf[64];
    B_state.last_rx_seq = seq;
    B_state.last_rx_cmd = cmd;
    tx_buf[0] = 0xAA;tx_buf[1] = 0x55;
    if (cmd ==  0x20){
        tx_buf[2] = 0x05;
        tx_buf[3] = 0xA0;
        tx_buf[4] = seq;
        tx_buf[5] = B_state.proto_ver;
        tx_buf[6] = B_state.b_ready;
        tx_buf[7] = B_state.b_fw_ver & 0xFF;
        tx_buf[8] = (B_state.b_fw_ver >> 8) & 0xFF;
        tx_buf[9] = seq;
        tx_buf[10] = crc16_ccitt(tx_buf[2],8) & 0xFF;
        tx_buf[11] = (crc16_ccitt(tx_buf[2],8) >> 8) & 0xFF;
        uart_write_bytes(BOARD_A_UART_PORT,tx_buf,12);
    }
    if (cmd == 0x01){
        if(payload_buf[29] & 0x01 == 0x01){
            B_state.latitude = payload_buf[0] | (payload_buf[1] >> 8)
                        | (payload_buf[2] >> 16) | (payload_buf[3] >> 24);
            B_state.longitude = payload_buf[4] | (payload_buf[5] >> 8)
                        | (payload_buf[6] >> 16) | (payload_buf[7] >>24);
        }
        if (payload_buf[29] & 0x02 == 0x02){
            B_state.alt_rel = payload_buf[8] | (payload_buf[9] >> 8)
                        | (payload_buf[10] >> 16) | (payload_buf[11] >>24);
        }
        if (payload_buf[29] & 0x04 == 0x04){
            B_state.utc_sec = payload_buf[12] | (payload_buf[13] >> 8)
                        | (payload_buf[14] >> 16) | (payload_buf[15] >>24);
        }
    }
    if (cmd == 0x10){
        B_state.session_id = payload_buf[2] | (payload_buf[3] >> 8)
                        | (payload_buf[4] >> 16) | (payload_buf[5] >>24);
        tx_buf[2] = 0x03;
        tx_buf[3] = 0x90;
        tx_buf[4] = seq;
        tx_buf[5] = cmd;
        tx_buf[6] = seq;
        tx_buf[7] = B_state.start_result;
        tx_buf[8] = crc16_ccitt(tx_buf[2],6) & 0xFF;
        tx_buf[9] = (crc16_ccitt(tx_buf[2],6) >> 8) & 0xFF;
        uart_write_bytes(BOARD_A_UART_PORT,tx_buf,10);
    }
    if (cmd == 0x11){
        B_state.session_id = payload_buf[2] | (payload_buf[3] >> 8)
                        | (payload_buf[4] >> 16) | (payload_buf[5] >>24);
        tx_buf[2] = 0x03;
        tx_buf[3] = 0x90;
        tx_buf[4] = seq;
        tx_buf[5] = cmd;
        tx_buf[6] = seq;
        tx_buf[7] = B_state.stop_result;
        tx_buf[8] = crc16_ccitt(tx_buf[2],6) & 0xFF;
        tx_buf[9] = (crc16_ccitt(tx_buf[2],6) >> 8) & 0xFF;
        uart_write_bytes(BOARD_A_UART_PORT,tx_buf,10);
    }
    if (cmd == 0x30){
        tx_buf[2] = 0x03;
        tx_buf[3] = 0x90;
        tx_buf[4] = seq;
        tx_buf[5] = cmd;
        tx_buf[6] = seq;
        tx_buf[7] = B_state.power_result;
        tx_buf[8] = crc16_ccitt(tx_buf[2],6) & 0xFF;
        tx_buf[9] = (crc16_ccitt(tx_buf[2],6) >> 8) & 0xFF;
        uart_write_bytes(BOARD_A_UART_PORT,tx_buf,10);
    }
}