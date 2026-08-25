#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define ML307_UART_PORT       UART_NUM_1
#define ML307_TX_PIN          GPIO_NUM_18
#define ML307_RX_PIN          GPIO_NUM_17
#define ML307_BAUD_RATE       115200
#define UART_BUF_SIZE         1024

static const char *TAG = "uart_bridge";                      //日志标签

// 自动发送的 AT 指令列表（ML307R‑DC OneMO 指令集，每条已经自带\r\n）
static const char *at_commands[] = {
    "AT\r\n",                                                // 模组基础握手
    "AT+CPIN?\r\n",                                          // 查询SIM卡状态，期待返回READY
    "AT+CEREG?\r\n",                                         // 查询4G蜂窝注册状态
    "AT+CGDCONT=1,\"IP\",\"cmnet\"\r\n",                      // 设置APN cmnet
    "AT+CGACT=1,1\r\n",                                      // 激活PDP网络上下文
    // -------- MQTT参数配置 connect_id=0 --------
    "AT+MQTTCFG=\"version\",0,4\r\n",                        // version=4 → MQTT3.1.1
    "AT+MQTTCFG=\"keepalive\",0,60\r\n",                     // MQTT保活心跳60s
    "AT+MQTTCFG=\"clean\",0,1\r\n",                          // CleanSession=1，新建会话
    "AT+MQTTCFG=\"ssl\",0,0\r\n",                            // ssl=0：明文1883端口；加密8883改为1
    // MQTT连接命令：host,port,clientID,username,password
    "AT+MQTTCONN=0,\"mqtt-mgnt.torchbearer.tech\",1883,\"ml307r_dev01\",\"zhh\",\"88888888\"\r\n",
    // 订阅下行主题 data/down QoS=0
    "AT+MQTTSUB=0,\"data_down\",0\r\n",
    "AT+MQTTPUB=0,\"data_up\",0,0,0,41,\"{\\\"test\\\":\\\"hello_ml307r\\\",\\\"value\\\":666}\"\r\n",
};

#define NUM_AT_COMMANDS (sizeof(at_commands) / sizeof(at_commands[0]))

static void ml307_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = ML307_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(ML307_UART_PORT, UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ML307_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ML307_UART_PORT, ML307_TX_PIN, ML307_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void auto_at_test_task(void *arg)
{
    uint8_t data[UART_BUF_SIZE];
    int cmd_index = 0;

    while (1) {
        const char *cmd = at_commands[cmd_index];
        ESP_LOGI(TAG, "Sending: %s", cmd);

        // 清空接收缓冲区
        uart_flush_input(ML307_UART_PORT);

        // 发送指令
        uart_write_bytes(ML307_UART_PORT, cmd, strlen(cmd));

        // 循环读取，最多持续 3000ms
        int total_len = 0;
        TickType_t start_ticks = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start_ticks) < pdMS_TO_TICKS(3000)) {
            int len = uart_read_bytes(ML307_UART_PORT, data + total_len,
                                      sizeof(data) - total_len - 1,
                                      pdMS_TO_TICKS(200));
            if (len > 0) {
                total_len += len;
                // 继续读取，直到200ms内无新数据
                start_ticks = xTaskGetTickCount(); // 重置超时，等待更多数据
            } else {
                // 200ms 无数据，认为响应结束
                if (total_len > 0) break;
            }
        }

        if (total_len > 0) {
            data[total_len] = '\0';
            ESP_LOGI(TAG, "Response (%d bytes):", total_len);
            printf("%s\n", (char *)data);
        } else {
            ESP_LOGW(TAG, "No response for command: %s", cmd);
        }

        // 切换到下一条命令，循环
        cmd_index = (cmd_index + 1) % NUM_AT_COMMANDS;
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每条命令之间间隔 1 秒
    }
}

// 透传任务：USB -> UART1
static void usb_to_uart_task(void *arg)
{
    uint8_t data[128];
    while (1) {
        int len = fread(data, 1, sizeof(data), stdin);
        if (len > 0) {
            uart_write_bytes(ML307_UART_PORT, (const char *)data, len);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// 透传任务：UART1 -> USB
static void uart_to_usb_task(void *arg)
{
    uint8_t data[UART_BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(ML307_UART_PORT, data, sizeof(data), pdMS_TO_TICKS(20));
        if (len > 0) {
            fwrite(data, 1, len, stdout);
            fflush(stdout);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void app_main(void)
{
    ml307_uart_init();

    // 禁用标准输入输出的缓冲
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    ESP_LOGI(TAG, "UART Bridge with Auto AT Test started.");
    ESP_LOGI(TAG, "ML307 UART: TX=%d, RX=%d, Baud=%d", ML307_TX_PIN, ML307_RX_PIN, ML307_BAUD_RATE);

    // 创建自动测试任务（优先级高于透传任务）
    xTaskCreate(auto_at_test_task, "auto_at_test", 4096, NULL, 6, NULL);

    // 创建透传任务（如果你仍想手动输入测试）
    xTaskCreate(usb_to_uart_task, "usb_to_uart", 4096, NULL, 5, NULL);
    xTaskCreate(uart_to_usb_task, "uart_to_usb", 4096, NULL, 5, NULL);
}