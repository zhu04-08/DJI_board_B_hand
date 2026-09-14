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
bool tf_init_flag = false;
tf_save_msg_t safe_msg_a;
tf_save_msg_t safe_msg_b;

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
    uint16_t crc = 0;
    while(1){
        int uart1_rx_head = uart_read_bytes(BOARD_A_UART_PORT,uart1_rx_buf,128,pdMS_TO_TICKS(100));
        int uart1_rx_tail = 0;
        while (uart1_rx_head > uart1_rx_tail){
            if(payload_idx != 0){
                if(uart1_rx_head >= basic_buf[2] - payload_idx + 2){
                    memcpy(&payload_buf[payload_idx],uart1_rx_buf,basic_buf[2] - payload_idx);
                    memcpy(&crc_buf[3+payload_idx],uart1_rx_buf,basic_buf[2] - payload_idx);
                    crc = uart1_rx_buf[basic_buf[2] - payload_idx + 1]
                        |((uint16_t)uart1_rx_buf[basic_buf[2] - payload_idx + 2] << 8);
                    uint16_t calc_crc = crc16_ccitt(crc_buf,basic_buf[2]+3);
                    if(calc_crc == crc && (B_state.last_rx_cmd != basic_buf[3] || B_state.last_rx_seq != basic_buf[4])){
                        //指令处理并应答
                        boardA_cmd_process(basic_buf[3],basic_buf[4],payload_buf);
                    }
                    uart1_rx_tail = basic_buf[2] - payload_idx + 2;
                    payload_idx = 0;
                }else{
                    memcpy(&payload_buf[payload_idx],uart1_rx_buf,uart1_rx_head);//payload_buf中包含了crc的内容，不影响处理payload处理
                    memcpy(&crc_buf[3+payload_idx],uart1_rx_buf,uart1_rx_head);
                    payload_idx += uart1_rx_head;
                    uart1_rx_tail = uart1_rx_head;
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
                    uart1_rx_tail = uart1_rx_head;
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
                if (uart1_rx_buf[uart1_rx_tail] == 0xAA && uart1_rx_head >= uart1_rx_tail + basic_buf[2] + 6){
                    memcpy(&payload_buf,&uart1_rx_buf[uart1_rx_tail+5],basic_buf[2]);
                    memcpy(&crc_buf[3],&uart1_rx_buf[uart1_rx_tail+5],basic_buf[2]);
                    crc = uart1_rx_buf[uart1_rx_tail + basic_buf[2]+5]
                        |((uint16_t)uart1_rx_buf[uart1_rx_tail + basic_buf[2] + 6] << 8);
                    uint16_t calc_crc = crc16_ccitt(crc_buf,basic_buf[2]+3);
                    if(calc_crc == crc && (B_state.last_rx_cmd != basic_buf[3] || B_state.last_rx_seq != basic_buf[4])){
                        //指令处理并应答
                        boardA_cmd_process(basic_buf[3],basic_buf[4],payload_buf);
                    }
                }else{
                    if (uart1_rx_buf[uart1_rx_tail] == 0xAA && uart1_rx_head > uart1_rx_tail +4){
                        memcpy(payload_buf,&uart1_rx_buf[uart1_rx_tail+4],uart1_rx_head - uart1_rx_tail - 4);
                        memcpy(basic_buf,&uart1_rx_buf[uart1_rx_tail],5);
                        payload_idx = uart1_rx_head - uart1_rx_tail;
                    }else{
                        memcpy(basic_buf,&uart1_rx_buf[uart1_rx_tail],uart1_rx_head - uart1_rx_tail);
                        basic_idx = uart1_rx_head - uart1_rx_tail;
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
    tf_init();
    // SC16IS752初始化
    SC16IS752_init(&dev, SC16IS752_PIN_RESET);
    if (wavelength_start[0] == 0 || wavelength_end[0] == 0 || wavelength_start[1] == 0 || wavelength_end[1] == 0) {
        uint8_t buf_a[16] = {0};
        uint8_t buf_b[16] = {0};
        //复位fifo
        SC16IS752_FIFOReset(&dev, SC16IS752_CHANNEL_A, 1); 
        SC16IS752_FIFOReset(&dev, SC16IS752_CHANNEL_B, 1); 
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
            int read = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_A,&buf_a[0],avail_a);
            if (read > 0){
                uint8_t sum = 0;
                for (int i = 0; i < 10; i++) sum += buf_a[i];
                if (sum == buf_a[10]) {                       // ★ 用 uint8_t 比较
                    wavelength_start[0] = (buf_a[7] << 8) | buf_a[6];
                    wavelength_end[0]   = (buf_a[9] << 8) | buf_a[8];
                }
            }
        }
        if (avail_b == 0x0F) {
            int read = SC16IS752_read_bytes(&dev,SC16IS752_CHANNEL_B,&buf_b[0],avail_b);
            if (read > 0){
                uint8_t sum = 0;
                for (int i = 0; i < 10; i++) sum += buf_b[i];
                if (sum == buf_b[10]) {                       // ★ 用 uint8_t 比较
                    wavelength_start[1] = (buf_b[7] << 8) | buf_b[6];
                    wavelength_end[1]   = (buf_b[9] << 8) | buf_b[8];
                }
            }
        }
        if (wavelength_start[0] && wavelength_end[0] &&
            wavelength_start[1] && wavelength_end[1]) {
            SC16IS752_init_flag = true;
        } else {
            ESP_LOGE(TAG, "wavelength read failed: A=[%u,%u] B=[%u,%u]",
                     wavelength_start[0], wavelength_end[0],
                     wavelength_start[1], wavelength_end[1]);
        }
    } else{
        SC16IS752_init_flag = true;
    }
    if (SC16IS752_init_flag && tf_init_flag) {

        if (csv_init() == ESP_OK) {

            xTaskCreate(data_process_save_task,   // 任务函数
                        "csv_save",               // 任务名
                        4096,                     // 栈大小
                        NULL,                     // 参数
                        5,                        // 优先级（低于接收任务 10）
                        NULL);                    // 句柄（不需要）

            B_state.b_ready = 1;
            ESP_LOGI("MAIN", "System ready: TF + SC16IS752 + CSV");
        } else {
            ESP_LOGE("MAIN", "csv_init failed, system NOT ready");
            B_state.b_ready = 2;   // 故障状态
        }
    } else {
        ESP_LOGE("MAIN", "init incomplete: uart=%d tf=%d",
                 SC16IS752_init_flag, tf_init_flag);
        B_state.b_ready = 2;
    }
}

//SC16IS752 串口轮询采集光谱数据
static void SC16IS752_data_get_task(void *pvParameters)
{
    SC16IS752_irq_bind_task(xTaskGetCurrentTaskHandle());
    uint8_t tmp[64];
    const int ch_list[2] = {SC16IS752_CHANNEL_A,SC16IS752_CHANNEL_B};
    rx_ctx_t *ctx_list[2] = {&rx_a,&rx_b};
  
    while (SC16IS752_init_flag) {
        if(!SC16IS752_uart_flag){
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        //先屏蔽IRQ引脚
        SC16IS752_irq_enable(false);
        //复位fifo
        SC16IS752_FIFOReset(&dev, SC16IS752_CHANNEL_A, 1); 
        SC16IS752_FIFOReset(&dev, SC16IS752_CHANNEL_B, 1);
        memset(&rx_a,0,sizeof(rx_a));
        memset(&rx_b,0,sizeof(rx_b));
        ulTaskNotifyTake(pdTRUE, 0);
        //发送连续采集指令
        for(int i = 0; i < 9; i++) {
            SC16IS752_write(&dev, SC16IS752_CHANNEL_A, cmd_continue_spectrum[i]);
            SC16IS752_write(&dev, SC16IS752_CHANNEL_B, cmd_continue_spectrum[i]);
        }
        SC16IS752_irq_enable(true);
        ESP_LOGI(TAG, "已发送连续采集指令，进入数据接收循环");
        //数据接收
        while (SC16IS752_uart_flag) {
            ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20));

            for(int ch = 0;ch < 2;ch++){
                rx_ctx_t *c = ctx_list[ch];
                size_t n;

                while(SC16IS752_available(&dev,ch_list[ch]) > 0){
                    int n = SC16IS752_read_bytes(&dev,ch_list[ch],tmp,sizeof(tmp));
                    if(n <= 0) break;
                    for(size_t i = 0;i < n; i++){
                        uint8_t b = tmp[i];
                        switch (c->state)
                        {
                        case RXS_SEARCH:
                            if (c->hdr_prev) {
                                c->hdr_prev = 0;
                                if (b == FRAME_HDR2) {
                                    c->buf[0]   = FRAME_HDR1;
                                    c->buf[1]   = FRAME_HDR2;
                                    c->received = 2;
                                    c->state    = RXS_LEN;
                                    break;
                                }
                                if (b != FRAME_HDR1) break;
                            }
                            if (b == FRAME_HDR1) c->hdr_prev = 1;
                            break;

                        case RXS_LEN:
                            c->buf[c->received++] = b;
                            if (c->received == 5) {
                                c->total_len = (uint32_t)c->buf[2]
                                             | ((uint32_t)c->buf[3] << 8)
                                             | ((uint32_t)c->buf[4] << 16);
                                if (c->total_len < 9 ||
                                    c->total_len > 2048) {
                                    c->state    = RXS_SEARCH;
                                    c->hdr_prev = 0;
                                } else {
                                    c->state = RXS_BODY;
                                }
                            }
                            break;

                        case RXS_BODY:
                            c->buf[c->received++] = b;
                            if (c->received == c->total_len) {
                                uint8_t sum = 0;
                                for (uint32_t k = 0;
                                     k < c->total_len - 3; k++)
                                    sum += c->buf[k];

                                bool ok =
                                    (sum == c->buf[c->total_len - 3]) &&
                                    (c->buf[c->total_len - 2] == FRAME_TAIL1) &&
                                    (c->buf[c->total_len - 1] == FRAME_TAIL2);

                                if (ok) {
                                    rx_frame_t f;
                                    f.channel = (uint8_t)ch;
                                    f.len     = (uint16_t)c->total_len;
                                    memcpy(f.data, c->buf, c->total_len);
                                    xQueueSend(frame_q, &f, 0);  // ★ 永不阻塞
                                }
                                c->state    = RXS_SEARCH;
                                c->hdr_prev = 0;
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
        
}

//光谱数据处理任务 + tf卡存储
static void data_process_save_task(void *pvParameters)
{
    /* 预分配解析缓冲（最大 1024 点） */
    uint16_t *spec_buf = malloc(1024 * sizeof(uint16_t));
    if (!spec_buf) {
        ESP_LOGE(TAG, "spec_buf malloc failed");
        vTaskDelete(NULL);
    }

    rx_frame_t frame;
    while (1) {
        /* 从接收任务的队列拿一帧 */
        if (xQueueReceive(frame_q, &frame, portMAX_DELAY) != pdTRUE)
            continue;

        /* 解析：把原始帧拆成 头结构 + 光谱数组 */
        tf_save_msg_t hdr = {0};
        if (!parse_spectrum_frame(&frame, &hdr, spec_buf, 1024)) {
            ESP_LOGW(TAG, "parse failed ch=%d len=%u",
                     frame.channel, frame.len);
            continue;
        }

        /* 选择目标文件 + 帧号自增 */
        FILE    *fp;
        uint32_t frame_no;
        if (frame.channel == 0) {
            fp       = g_fp_a;
            frame_no = ++g_fc_a;
        } else {
            fp       = g_fp_b;
            frame_no = ++g_fc_b;
        }
        if (!fp) continue;

        /* 写一行 */
        csv_write_row(fp, frame.channel, frame_no, &hdr,
                      spec_buf, hdr.num_points);

        /* 每 10 帧 flush 一次，兼顾速度与掉电保护 */
        if ((frame_no % 10) == 0) {
            fflush(fp);
            fsync(fileno(fp));
        }

        /* 每 50 帧打印一次进度 */
        if ((frame_no % 50) == 1) {
            ESP_LOGI(TAG, "ch=%c saved %" PRIu32 " frames (pts=%u)",
                     (frame.channel == 0) ? 'A' : 'B',
                     frame_no, hdr.num_points);
        }
    }
}

void app_main(void)
{
    //创建SC16IS752串口轮询采集任务
}