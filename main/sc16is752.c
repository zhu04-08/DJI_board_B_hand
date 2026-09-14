#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "sc16is752.h"

#define TAG "SC16IS752"
#define SPI_FREQ 4000000 // 4 MHz

SC16IS752_t dev;

static TaskHandle_t rx_task_handle = NULL;
static volatile bool irq_armed = false;
static bool irq_isr_installed = false;

//光谱采集任务定义变量
const uint8_t cmd_wavelength[] = {
    0xCC, 0x01, 0x09, 0x00, 0x00, 0x0F, 0xE5, 0x0D, 0x0A
};//获取光谱波长指令
const uint8_t cmd_continue_spectrum[] = {
    0xCC, 0x01, 0x09, 0x00, 0x00, 0x33, 0x09, 0x0D, 0x0A
};//连续获取光谱指令
const uint8_t cmd_stop_spectrum[] = {
    0xCC, 0x01, 0x09, 0x00, 0x00, 0x04, 0xDA, 0x0D, 0x0A
};//停止获取光谱指令 

//光谱波长数组
uint16_t wavelength_start[2] = {0, 0};//0->A通道，1->B通道
uint16_t wavelength_end[2] = {0, 0};//0->A通道，1->B通道

uint8_t SC16IS752_ReadRegister(SC16IS752_t *dev,uint8_t channel, uint8_t reg_addr)
{
    uint8_t result = 0;
    //SPI读取：命令字节高位置一表示读操作，低三位包含通道和寄存器地址
    unsigned char spi_data[2];
    spi_data[0] = 0x80 | (reg_addr << 3) | (channel << 1); 
    spi_data[1] = 0xff;
    //配置spi事务
    spi_transaction_t SPITransaction;
    memset(&SPITransaction, 0, sizeof(SPITransaction));
    SPITransaction.length = 16; 
    SPITransaction.tx_buffer = spi_data;
    SPITransaction.rx_buffer = spi_data;
    //执行spi传输
    esp_err_t espRc = spi_device_transmit(dev->spi_device_handle, &SPITransaction);
    if (espRc == ESP_OK) {
        result = spi_data[1];
    } else {
        ESP_LOGE(TAG, "ReadRegister reg_addr=0x%02x failed. code: 0x%02x", reg_addr, espRc);
        ESP_LOGE(TAG, "ReadRegister %s", esp_err_to_name(espRc));
    }
    return result;
}

void SC16IS752_WriteRegister(SC16IS752_t *dev, uint8_t channel, uint8_t reg_addr, uint8_t val)
{
    // SPI 写入：命令字节最高位为0，低三位包含通道和寄存器地址
    unsigned char spi_data[2];
    spi_data[0] = (reg_addr << 3 | channel << 1);
    spi_data[1] = val;

    spi_transaction_t SPITransaction;
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = 2 * 8;
    SPITransaction.tx_buffer = spi_data;

    esp_err_t espRc = spi_device_transmit(dev->spi_device_handle, &SPITransaction);
    if (espRc == ESP_OK) {
        ESP_LOGD(TAG, "WriteRegister reg_addr=0x%02x val=0x%02x successfully", reg_addr, val);
    } else {
        ESP_LOGE(TAG, "WriteRegister reg_addr=0x%02x val=0x%02x failed. code: 0x%02x", reg_addr, val, espRc);
        ESP_LOGE(TAG, "WriteRegister %s", esp_err_to_name(espRc));
    }
}

int16_t SC16IS752_SetBaudrate(SC16IS752_t *dev, uint8_t channel, uint32_t baudrate)
{
    uint16_t divisor;
    uint8_t prescaler;
    uint32_t actual_baudrate;
    int16_t error;
    uint8_t temp_lcr;

    if ((SC16IS752_ReadRegister(dev, channel, SC16IS752_REG_MCR) & 0x80) == 0) {
        prescaler = 1;
    } else {
        prescaler = 4;
    }

    uint32_t divisor1 = dev->crystal_freq / prescaler;
    uint32_t divisor2 = baudrate * 16;

    if (divisor2 > divisor1) {
        ESP_LOGE(TAG, "Baudrate %" PRIu32 " not supported", baudrate);
        return 0;
    }

    double wk = (double)divisor1 / (double)divisor2;
    divisor = wk + 0.999;   // 向上取整
    ESP_LOGD(TAG, "baudrate=%" PRIu32 " divisor=%d", baudrate, divisor);

    temp_lcr = SC16IS752_ReadRegister(dev, channel, SC16IS752_REG_LCR);
    temp_lcr |= 0x80;
    SC16IS752_WriteRegister(dev, channel, SC16IS752_REG_LCR, temp_lcr);
    SC16IS752_WriteRegister(dev, channel, SC16IS752_REG_DLL, (uint8_t)divisor);
    SC16IS752_WriteRegister(dev, channel, SC16IS752_REG_DLH, (uint8_t)(divisor >> 8));
    temp_lcr &= 0x7F;
    SC16IS752_WriteRegister(dev, channel, SC16IS752_REG_LCR, temp_lcr);

    actual_baudrate = (divisor1 / divisor) / 16;
    error = baudrate - actual_baudrate;
    ESP_LOGD(TAG, "actual_baudrate=%" PRIu32 " error=%d", actual_baudrate, error);
    if (error != 0) {
        ESP_LOGW(TAG, "baudrate=%" PRIu32 " actual_baudrate=%" PRIu32, baudrate, actual_baudrate);
    }

    return error;
}

void SC16IS752_FIFOEnable(SC16IS752_t *dev, uint8_t channel, uint8_t fifo_enable)
{
    uint8_t temp_fcr = SC16IS752_ReadRegister(dev, channel, SC16IS752_REG_FCR);
    if (fifo_enable == 0) {
        temp_fcr &= 0xFE; // disable FIFO
    } else {
        temp_fcr |= 0x01; // enable FIFO
    }
    SC16IS752_WriteRegister(dev, channel, SC16IS752_REG_FCR, temp_fcr);
}

void SC16IS752_FIFOReset(SC16IS752_t *dev, uint8_t channel, uint8_t rx_fifo)
{
    uint8_t temp_fcr = SC16IS752_ReadRegister(dev, channel, SC16IS752_REG_FCR);
    if (rx_fifo == 0) {
        temp_fcr |= 0x04;   // 复位发送 FIFO
    } else {
        temp_fcr |= 0x02;   // 复位接收 FIFO
    }
    SC16IS752_WriteRegister(dev, channel, SC16IS752_REG_FCR, temp_fcr);
}

void SC16IS752_SetLine(SC16IS752_t *dev, uint8_t channel, uint8_t data_length, uint8_t parity_select, uint8_t stop_length)
{
    uint8_t temp_lcr = SC16IS752_ReadRegister(dev, channel, SC16IS752_REG_LCR);
    temp_lcr &= 0xC0;   // 保留 DLAB 和 Break 控制位，清除低6位

    switch (data_length) {
        case 5: break;
        case 6: temp_lcr |= 0x01; break;
        case 7: temp_lcr |= 0x02; break;
        case 8: temp_lcr |= 0x03; break;
        default: temp_lcr |= 0x03; break;
    }

    if (stop_length == 2) {
        temp_lcr |= 0x04;
    }

    switch (parity_select) {
        case 0: break;                       // 无校验
        case 1: temp_lcr |= 0x08; break;     // 奇校验
        case 2: temp_lcr |= 0x18; break;     // 偶校验
        case 3: temp_lcr |= 0x03; break;     // 强制1
        case 4: break;                       // 强制0
        default: break;
    }

    SC16IS752_WriteRegister(dev, channel, SC16IS752_REG_LCR, temp_lcr);
}

static void IRAM_ATTR SC16IS752_IRQ_ISR(void *arg)
{
    BaseType_t hp = pdFALSE;
    if(rx_task_handle && irq_armed){
        vTaskNotifyGiveFromISR(rx_task_handle,&hp);
        portYIELD_FROM_ISR(hp);
    }
}
void SC16IS752_init(SC16IS752_t *dev, int16_t reset_pin)
{
    dev->address_sspin = SC16IS752_PIN_CS;
    dev->peek_flag = 0;
    dev->crystal_freq = SC16IS752_CRYSRAL_FREQ; // 1.8432 MHz
    //硬件复位
    if (reset_pin >= 0) {
        gpio_reset_pin(reset_pin);
        gpio_set_direction(reset_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(reset_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(reset_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(reset_pin, 1);
    }
    //配置spi模式
    spi_bus_config_t busconfig = {
        .miso_io_num = SC16IS752_PIN_MISO,
        .mosi_io_num = SC16IS752_PIN_MOSI,
        .sclk_io_num = SC16IS752_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busconfig, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t devconfig = {
        .clock_speed_hz = SPI_FREQ,
        .mode = 0,
        .spics_io_num = SC16IS752_PIN_CS,
        .queue_size = 7,
    };
    spi_device_handle_t spi_device_handle;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devconfig, &spi_device_handle));
    dev->spi_device_handle = spi_device_handle;
    //初始化双通道
    SC16IS752_FIFOEnable(dev, SC16IS752_CHANNEL_A, 1);
    SC16IS752_FIFOEnable(dev, SC16IS752_CHANNEL_B, 1);
    SC16IS752_SetBaudrate(dev, SC16IS752_CHANNEL_A, SC16IS752_UART_BAUDRATE);
    SC16IS752_SetBaudrate(dev, SC16IS752_CHANNEL_B, SC16IS752_UART_BAUDRATE);
    //配置IRQ引脚
    gpio_config_t irq_cfg = {
        .pin_bit_mask = (1ULL << SC16IS752_PIN_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&irq_cfg));
    if(!irq_isr_installed){
        esp_err_t err = gpio_install_isr_service(0);
        if(err == ESP_OK || err == ESP_ERR_INVALID_STATE){
            irq_isr_installed = true;
        }else{
            ESP_ERROR_CHECK(err);
        }
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(SC16IS752_PIN_IRQ,SC16IS752_IRQ_ISR,NULL));
    //设置串口帧形式
    SC16IS752_SetLine(dev, SC16IS752_CHANNEL_A, 8, 0, 1);
    SC16IS752_SetLine(dev, SC16IS752_CHANNEL_B, 8, 0, 1);
}


int SC16IS752_read_bytes(SC16IS752_t *dev, uint8_t channel, uint8_t *buf, int len)
{
    if (len == 0) return 0;
    if (len > 64) len = 64; // 限制最大读取长度为64字节
    uint8_t tx_buf[65];//首位置零为SPI读命令，后面全为0xFF产生时钟以获取字节
    uint8_t rx_buf[65];//首位无意义，后面为FIFO中的数据
    tx_buf[0] = 0x80 | (SC16IS752_REG_RHR << 3 | channel << 1);
    memset(&tx_buf[1], 0xFF, len);
    spi_transaction_t SPITranscaction;
    memset(&SPITranscaction,0,sizeof(SPITranscaction));//先将结构体初始化
    SPITranscaction.length = (len + 1) * 8;
    SPITranscaction.tx_buffer = tx_buf;
    SPITranscaction.rx_buffer = rx_buf;

    esp_err_t espRc = spi_device_transmit(dev->spi_device_handle,&SPITranscaction);
    if (espRc == ESP_OK){
        memcpy(buf,&rx_buf[1],len);
        return len;
    }
    return 0;
    
}

void SC16IS752_write(SC16IS752_t *dev, uint8_t channel, uint8_t val)
{
    uint8_t tmp_lsr;
    // 等待发送保持寄存器为空（LSR[5]=1）
    do {
        tmp_lsr = SC16IS752_ReadRegister(dev, channel, SC16IS752_REG_LSR);
        esp_rom_delay_us(1);
    } while ((tmp_lsr & 0x20) == 0);

    SC16IS752_WriteRegister(dev, channel, SC16IS752_REG_THR, val);
}

int SC16IS752_available(SC16IS752_t *dev, uint8_t channel)
{
    return SC16IS752_ReadRegister(dev, channel, SC16IS752_REG_RXLVL);
}
void SC16IS752_irq_bind_task(TaskHandle_t task)
{
    rx_task_handle = task;
}

void SC16IS752_irq_enable(bool en)
{
    irq_armed = en;
}