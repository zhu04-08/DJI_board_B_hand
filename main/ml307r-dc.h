#ifndef ML307R_DC_H
#define ML307R_DC_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define ML307_UART_PORT       UART_NUM_2
#define ML307_TX_PIN          GPIO_NUM_18
#define ML307_RX_PIN          GPIO_NUM_17
#define ML307_BAUD_RATE       230400
#define UART_BUF_SIZE         1024
#define ML307R_CONN_ID        0
#define MQTT_TOPIC_A          "data_up"

#define MQTT_PKT_MAX   1024
#define MQTT_PKT_NUM   2       // 一帧分两个包

extern QueueHandle_t mqtt_q;
extern TaskHandle_t mqtt_task_handle;
typedef struct {
    uint8_t  channel;                  // 'A' 或 'B'
    uint32_t frame_no;                 // 帧序号
    uint16_t total_pkts;               // 总包数
    uint16_t pkt_idx;                  // 当前包序号
    uint16_t payload_len;              // 实际负载长度
    uint8_t  payload[MQTT_PKT_MAX];    // 负载
} mqtt_pkt_t;

void ml307_uart_init(void);
bool ml307r_mqtt_connect(void);

#endif //ML307R_DC_H