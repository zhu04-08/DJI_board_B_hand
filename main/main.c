#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "math.h"
#include "time.h"

#include "sc16is752.h"
#include "sdspi.h"
#include "ml307r-dc.h"
#include "boardA.h"

#define TAG "MAIN"

bool SC16IS752_uart_flag = false;
bool SC16IS752_init_flag = false;

static void boardA_heartbeat_task(void *pvParameters)
{
    while(1){
        uint8_t tx_buf[64];
        tx_buf[0] = 0xAA;
        tx_buf[1] = 0x55;
        tx_buf[2] = 0x0E;
        tx_buf[3] = 0x81;
        tx_buf[4] = B_state.last_rx_seq;
        tx_buf[5] = B_state.b_ready;
        tx_buf[6] = B_state.actual_capture;
        tx_buf[7] = B_state.err_code;
        tx_buf[8] = B_state.storage_free_pct;
        tx_buf[9] = B_state.frame_count & 0xFF;
        tx_buf[10] = (B_state.frame_count >> 8) & 0xFF;
        tx_buf[11] = (B_state.frame_count >> 16) & 0xFF;
        tx_buf[12] = (B_state.frame_count >> 24) & 0xFF;
        tx_buf[13] = B_state.session_id & 0xFF;
        tx_buf[14] = (B_state.session_id >> 8) & 0xFF;
        tx_buf[15] = (B_state.session_id >> 16) & 0xFF;
        tx_buf[16] = (B_state.session_id >> 24) & 0xFF;
        tx_buf[17] = B_state.safe_power_off;
        tx_buf[18] = 0;
        tx_buf[19] = crc16_ccitt(tx_buf[2],8) & 0xFF;
        tx_buf[20] = (crc16_ccitt(tx_buf[2],8) >> 8) & 0xFF;
        uart_write_bytes(BOARD_A_UART_PORT,tx_buf,21);
        //1Hz间隔发送
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void boardA_interaction_task(void *pvParameters)
{
    uint8_t uart1_rx_buf[128];
    uint8_t basic_buf[8];
    uint8_t basic_idx = 0;
    uint8_t payload_buf[64];
    uint8_t payload_idx = 0;
    uint8_t crc_buf[64];
    uint8_t crc = 0;
    while(1){
        int uart1_rx_head = uart_read_bytes(BOARD_A_UART_PORT,uart1_rx_buf,128,pdMS_TO_TICKS(100));
        int uart1_rx_tail = 0;
        while (uart1_rx_head > uart1_rx_tail){
            if(payload_idx != 0){
                if(uart1_rx_head >= basic_buf[2] - payload_idx + 1){
                    memcpy(payload_buf[payload_idx],uart1_rx_buf,basic_buf[2] - payload_idx);
                    memcpy(&crc_buf[3+payload_idx],uart1_rx_buf,basic_buf[2] - payload_idx);
                    crc = uart1_rx_buf[uart1_rx_tail + basic_buf[2]+5];
                    uint16_t calc_crc = crc16_ccitt(crc_buf,basic_buf[2]+3);
                    if(calc_crc == crc && (B_state.last_rx_cmd != basic_buf[3] || B_state.last_rx_seq != basic_buf[4])){
                        //指令处理并应答
                        boardA_cmd_process(basic_buf[3],basic_buf[4],payload_buf);
                    }
                    payload_idx = 0;
                }else{
                    memcpy(payload_buf[payload_idx],uart1_rx_buf,uart1_rx_head);
                    memcpy(&crc_buf[3+payload_idx],uart1_rx_buf,uart1_rx_buf);
                    uint16_t calc_crc = crc16_ccitt(crc_buf,basic_buf[2]+3);
                    payload_idx += uart1_rx_head;
                }
            }
            if(basic_idx != 0){
                if(uart1_rx_head >=5 - basic_idx){
                    memcpy(&basic_buf,uart1_rx_buf,5-basic_idx);
                    memcpy(&crc_buf,basic_buf[2],3);
                    uart1_rx_tail = 5 -basic_idx;
                    basic_idx = 0;
                }else{
                    memcpy(&basic_buf,uart1_rx_buf,5-basic_idx);
                    basic_idx += uart1_rx_head;
                }
            }
            if (payload_idx == 0 && basic_idx == 0){
                for(int i = uart1_rx_tail; i+1 < uart1_rx_head; i++){
                    if(uart1_rx_buf[i] == 0xAA && uart1_rx_buf[i+1] == 0x55){
                        if(i+5 < uart1_rx_head){
                            memcpy(&basic_buf,uart1_rx_buf[i],5);
                            memcpy(&crc_buf,basic_buf[2],3);
                            uart1_rx_tail = i;
                            break;
                        }else{
                            memcpy(&basic_buf,uart1_rx_buf[i],uart1_rx_head - i);
                            basic_idx = uart1_rx_head - i;
                            uart1_rx_tail = uart1_rx_head;
                            break;
                        }
                    }
                }
                if (uart1_rx_buf[uart1_rx_tail] == 0xAA && uart1_rx_head >= uart1_rx_tail + basic_buf[2] + 5){
                    memcpy(&payload_buf,uart1_rx_buf[uart1_rx_tail+5],basic_buf[2]);
                    memcpy(&crc_buf[3],uart1_rx_buf[uart1_rx_tail+5],basic_buf[2]);
                    crc = uart1_rx_buf[uart1_rx_tail + basic_buf[2]+5];
                    uint16_t calc_crc = crc16_ccitt(crc_buf,basic_buf[2]+3);
                    if(calc_crc == crc && (B_state.last_rx_cmd != basic_buf[3] || B_state.last_rx_seq != basic_buf[4])){
                        //指令处理并应答
                        boardA_cmd_process(basic_buf[3],basic_buf[4],payload_buf);
                    }
                }else{
                    if (uart1_rx_buf[uart1_rx_tail] == 0xAA && uart1_rx_head > uart1_rx_tail +4){
                        memcpy(&basic_buf,uart1_rx_buf[uart1_rx_tail],uart1_rx_head - uart1_rx_tail);
                        payload_idx = uart1_rx_head - uart1_rx_tail;
                    }
                }
            }
        }
    }
}

static void main_init(void)
{
    //板B应答初始化
    B_state.b_ready = 0; //板B状态设置为初始化
    boardA_init();
    // SC16IS752初始化
    SC16IS752_init(&dev, SC16IS752_PIN_RESET);
    if (wavelength_start[0] == 0 || wavelength_end[0] == 0 || wavelength_start[1] == 0 || wavelength_end[1] == 0) {
        //复位fifo
        SC16IS752_FIFOReset(&dev, SC16IS752_CHANNEL_A, 1); 
        SC16IS752_FIFOReset(&dev, SC16IS752_CHANNEL_B, 1); 
        rx_head_a = 0;
        rx_head_b = 0;
        //发送获取波长范围指令
        for(int i = 0; i < 9; i++) {
            SC16IS752_write(&dev, SC16IS752_CHANNEL_A, cmd_wavelength[i]);
            SC16IS752_write(&dev, SC16IS752_CHANNEL_B, cmd_wavelength[i]);
            }
        vTaskDelay(pdMS_TO_TICKS(200));
        //读取波长范围数据
        int avail_a = SC16IS752_available(&dev, SC16IS752_CHANNEL_A);
        int avail_b = SC16IS752_available(&dev, SC16IS752_CHANNEL_B);
        if (avail_a == 0x0F) {
            int read = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_A,&rx_buf_a[rx_head_a],avail_a);
            if (read > 0) rx_head_a += read;
        }
        if (avail_b == 0x0F) {
            int read = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_B,&rx_buf_b[rx_head_b],avail_b);
            if (read > 0) rx_head_b += read;
        }
        int16_t sum_a = 0, sum_b = 0;
        for (int i = 0; i < 10; i++) {
            sum_a += rx_buf_a[i];
            sum_b += rx_buf_b[i];
            }
        if (sum_a && 0xFF == rx_buf_a[10]){
            wavelength_start[0] = (rx_buf_a[7] << 8) | rx_buf_a[6];
            wavelength_end[0] = (rx_buf_a[9] << 8) | rx_buf_a[8];
        }
        if (sum_b && 0xFF == rx_buf_b[10]){
            wavelength_start[1] = (rx_buf_b[7] << 8) | rx_buf_b[6];
            wavelength_end[1] = (rx_buf_b[9] << 8) | rx_buf_b[8];
        }
    } else{
        SC16IS752_init_flag = true;
    }
    if(SC16IS752_init_flag == true) B_state.b_ready = 1;
}

//板A指令接收任务
static void board_a_cmd_receive_task(void *pvParameters)
{

}

//SC16IS752 串口轮询采集光谱数据
static void SC16IS752_data_get_task(void *pvParameters)
{
    while (SC16IS752_init_flag) {
        if(!SC16IS752_uart_flag){
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        //复位fifo
        SC16IS752_FIFOReset(&dev, SC16IS752_CHANNEL_A, 1); 
        SC16IS752_FIFOReset(&dev, SC16IS752_CHANNEL_B, 1); 
        rx_head_a = 0;
        rx_head_b = 0;
        //发送连续采集指令
        for(int i = 0; i < 9; i++) {
            SC16IS752_write(&dev, SC16IS752_CHANNEL_A, cmd_continue_spectrum[i]);
            SC16IS752_write(&dev, SC16IS752_CHANNEL_B, cmd_continue_spectrum[i]);
        }
        ESP_LOGI(TAG, "已发送连续采集指令，进入数据接收循环");
        //数据接收
        while (SC16IS752_uart_flag) {
            bool data_received = false;
            //数据接收，存入环形缓冲区
            int avail_a = SC16IS752_available(&dev, SC16IS752_CHANNEL_A);
            if (avail_a > 0) {
                data_received = true;
                int space_a = 4096 - rx_head_a;
                if (avail_a <= space_a){
                    int read = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_A,&rx_buf_a[rx_head_a],avail_a);
                    if (read > 0) rx_head_a = (rx_head_a + read) % 4096;
                }
                else {
                    int read_1 = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_A,&rx_buf_a[rx_head_a],space_a);
                    int read_2 = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_A,&rx_buf_a[0],avail_a - space_a);
                    if (read_1 > 0 && read_2 > 0) rx_head_a = read_2;
                }
            }
            int avail_b = SC16IS752_available(&dev, SC16IS752_CHANNEL_B);
            if (avail_b > 0) {
                data_received = true;
                int space_b = 4096 - rx_head_b;
                if (avail_b <= space_b){
                    int read = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_B,&rx_buf_b[rx_head_b],avail_b);
                    if (read > 0) rx_head_b = (rx_head_b + read) % 4096;
                }
                else {
                    int read_1 = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_B,&rx_buf_b[rx_head_b],space_b);
                    int read_2 = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_B,&rx_buf_b[0],avail_b - space_b);
                    if (read_1 > 0 && read_2 > 0) rx_head_b = read_2;
                }
            }
            //无数据时短暂延时，喂狗
            if (!data_received) {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
    }
        
}

//光谱数据处理 + tf卡存储
static void data_process_save_task(void *pvParameters)
{

}

void app_main(void)
{
    //创建SC16IS752串口轮询采集任务
}