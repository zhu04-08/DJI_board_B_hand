#ifndef MAIN_SC16IS752_H
#define MAIN_SC16IS752_H

#include <stdint.h>
#include <driver/spi_master.h>

#define SC16IS752_PIN_SCLK  12
#define SC16IS752_PIN_MOSI  11
#define SC16IS752_PIN_MISO  13
#define SC16IS752_PIN_CS    10
#define SC16IS752_PIN_IRQ   14
#define SC16IS752_PIN_RESET 9
#define SC16IS752_CRYSRAL_FREQ  1843200UL
#define SC16IS752_UART_BAUDRATE 115200

#define SC16IS752_CHANNEL_A 0x00
#define SC16IS752_CHANNEL_B 0x01

// General Registers
#define SC16IS752_REG_RHR       (0x00)
#define SC16IS752_REG_THR       (0x00)
#define SC16IS752_REG_IER       (0x01)
#define SC16IS752_REG_FCR       (0x02)
#define SC16IS752_REG_IIR       (0x02)
#define SC16IS752_REG_LCR       (0x03)
#define SC16IS752_REG_MCR       (0x04)
#define SC16IS752_REG_LSR       (0x05)
#define SC16IS752_REG_MSR       (0x06)
#define SC16IS752_REG_SPR       (0x07)
#define SC16IS752_REG_TCR       (0x06)
#define SC16IS752_REG_TLR       (0x07)
#define SC16IS752_REG_TXLVL     (0x08)
#define SC16IS752_REG_RXLVL     (0x09)
#define SC16IS752_REG_IODIR     (0x0A)
#define SC16IS752_REG_IOSTATE   (0x0B)
#define SC16IS752_REG_IOINTENA  (0x0C)
#define SC16IS752_REG_IOCONTROL (0x0E)
#define SC16IS752_REG_EFCR      (0x0F)

// Special Registers
#define SC16IS752_REG_DLL       (0x00)
#define SC16IS752_REG_DLH       (0x01)

// Enhanced Registers
#define SC16IS752_REG_EFR       (0x02)
#define SC16IS752_REG_XON1      (0x04)
#define SC16IS752_REG_XON2      (0x05)
#define SC16IS752_REG_XOFF1     (0x06)
#define SC16IS752_REG_XOFF2     (0x07)

// Interrupt Enable Register bits
#define SC16IS752_INT_CTS       (0x80)
#define SC16IS752_INT_RTS       (0x40)
#define SC16IS752_INT_XOFF      (0x20)
#define SC16IS752_INT_SLEEP     (0x10)
#define SC16IS752_INT_MODEM     (0x08)
#define SC16IS752_INT_LINE      (0x04)
#define SC16IS752_INT_THR       (0x02)
#define SC16IS752_INT_RHR       (0x01)


typedef struct {
    uint8_t address_sspin; // SPI CS PIN 
    long crystal_freq; // SC16IS752芯片晶振大小
    int peek_buf;
    uint8_t peek_flag; // 预读标志位
    spi_device_handle_t spi_device_handle; // SPI device handle
}SC16IS752_t;
SC16IS752_t dev;

extern const uint8_t cmd_wavelength[];
extern const uint8_t cmd_continue_spectrum[];
extern const uint8_t cmd_stop_spectrum[];
extern uint16_t wavelength_start[2];//0->A通道，1->B通道
extern uint16_t wavelength_end[2];
extern uint8_t rx_buf_a[4096];
extern uint8_t rx_buf_b[4096];
extern int rx_head_a;
extern int rx_head_b;

uint8_t SC16IS752_ReadRegister(SC16IS752_t *dev,uint8_t channel, uint8_t reg_addr);
void SC16IS752_WriteRegister(SC16IS752_t *dev, uint8_t channel, uint8_t reg_addr, uint8_t val);
int16_t SC16IS752_SetBaudrate(SC16IS752_t *dev, uint8_t channel, uint32_t baudrate);
void SC16IS752_FIFOEnable(SC16IS752_t *dev, uint8_t channel, uint8_t fifo_enable);
void SC16IS752_FIFOReset(SC16IS752_t *dev, uint8_t channel, uint8_t rx_fifo);
void SC16IS752_SetLine(SC16IS752_t *dev, uint8_t channel, uint8_t data_length, uint8_t parity_select, uint8_t stop_length);
void SC16IS752_init(SC16IS752_t *dev, int16_t reset_pin);
int SC16IS752_read_bytes(SC16IS752_t *dev, uint8_t channel, uint8_t *buf, int len);
void SC16IS752_write(SC16IS752_t *dev, uint8_t channel, uint8_t val);
int SC16IS752_available(SC16IS752_t *dev, uint8_t channel);


#endif // MAIN_SC16IS752_H