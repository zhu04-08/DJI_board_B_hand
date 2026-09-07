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

static void main_init(void)
{
    // SC16IS752初始化
    bool SC16IS752_init_flag = false;
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
}

//SC16IS752 串口轮询采集光谱数据
static void SC16IS752_data_get_task(void *pvParameters)
{
    while (1) {
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
static void data_process_save_task(void *pvparameters)
{
    
}

void app_main(void)
{
    //创建SC16IS752串口轮询采集任务
}