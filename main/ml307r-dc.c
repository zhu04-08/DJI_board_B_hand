#include <stdio.h>

#include "ml307r-dc.h"
QueueHandle_t mqtt_q = NULL;
TaskHandle_t mqtt_task_handle = NULL;

void ml307_uart_init(void)
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

/* 发 AT 命令 + 等 OK */
static esp_err_t send_at_cmd(const char *cmd, const char *expected_resp, uint32_t timeout_ms)
{
    uint8_t response_buf[128];
    uart_flush_input(ML307_UART_PORT);
    uart_write_bytes(ML307_UART_PORT, cmd, strlen(cmd));

    int total_len = 0;
    TickType_t start_ticks = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_ticks) < pdMS_TO_TICKS(timeout_ms)) {
        int len = uart_read_bytes(ML307_UART_PORT, response_buf + total_len,
                                  sizeof(response_buf) - total_len - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            total_len += len;
            response_buf[total_len] = '\0';
            if (expected_resp && strstr((char *)response_buf, expected_resp) != NULL) {
                return ESP_OK;
            }
        }
    }

    if (expected_resp == NULL && total_len > 0) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

/* 建立 MQTT 连接（按 ML307R 手册） */
bool ml307r_mqtt_connect(void)
{
    char cmd[192];

    /* ---- 1. AT 握手 ---- */
    if (send_at_cmd("AT\r\n", "OK", 1000) != ESP_OK) return false;
    if (send_at_cmd("AT+CPIN?\r\n", "READY", 2000) != ESP_OK) return false;
    if (send_at_cmd("AT+CEREG?\r\n", "OK", 2000) != ESP_OK) return false;
    if (send_at_cmd("AT+CGDCONT=1,\"IP\",\"cmnet\"\r\n", "OK", 2000) != ESP_OK) return false;
    if (send_at_cmd("AT+CGACT=1,1\r\n", "OK", 5000) != ESP_OK) return false;

    send_at_cmd("AT+MQTTCFG=\"version\",0,4\r\n", "OK", 1000);
    send_at_cmd("AT+MQTTCFG=\"keepalive\",0,60\r\n", "OK", 1000);
    send_at_cmd("AT+MQTTCFG=\"clean\",0,1\r\n", "OK", 1000);
    send_at_cmd("AT+MQTTCFG=\"ssl\",0,0\r\n", "OK", 1000);
    
    const char *conn_cmd = "AT+MQTTCONN=0,\"mqtt-mgnt.torchbearer.tech\",1883,\"ml307r_dev01\",\"zhh\",\"88888888\"\r\n";
    if (send_at_cmd(conn_cmd, "OK", 1000) != ESP_OK) {
        return false;
    }
    return true;
}
