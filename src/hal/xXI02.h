#ifndef xXI02_h
#define xXI02_h

#include <bcm2835.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <ifaddrs.h>
#include <netpacket/packet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <unistd.h>

#define SC18IS602_ADDRESS_000   0x50
#define SC18IS602_ADDRESS_001   0x52
#define SC18IS602_ADDRESS_010   0x54
#define SC18IS602_ADDRESS_011   0x56
#define SC18IS602_ADDRESS_100   0x58
#define SC18IS602_ADDRESS_101   0x5A
#define SC18IS602_ADDRESS_110   0x5C
#define SC18IS602_ADDRESS_111   0x5E

#define SC18IS601_CONFIG_SPI   0xF0
#define SC18IS601_CLEAR_INT    0xF1
#define SC18IS601_IDLE         0xF2
#define SC18IS601_GPIO_WRITE   0xF4
#define SC18IS601_GPIO_READ    0xF5
#define SC18IS601_GPIO_ENABLE  0xF6
#define SC18IS601_GPIO_CONFIG  0xF7

#define SC18IS602_CLOCK_1843K  0x00
#define SC18IS602_CLOCK_461K   0x01
#define SC18IS602_CLOCK_115K   0x02
#define SC18IS602_CLOCK_58K    0x03

#define SC18IS602_SPI_MODE0    0x00
#define SC18IS602_SPI_MODE1    0x01
#define SC18IS602_SPI_MODE2    0x02
#define SC18IS602_SPI_MODE3    0x03

#define SC18IS602_SS_0         0x00
#define SC18IS602_SS_1         0x01
#define SC18IS602_SS_2         0x02
#define SC18IS602_SS_3         0x03

#define SC18IS601_GPIO_MODE_QUASI_BID   0x00
#define SC18IS601_GPIO_MODE_PUSH_PULL   0x01
#define SC18IS601_GPIO_MODE_INPUT_ONLY 0x02
#define SC18IS601_GPIO_MODE_OPEN_DRAIN  0x03


class xXI02 {
  public:
    xXI02();
    xXI02(uint8_t addr);
    bool begin();
    bool begin(uint8_t ss_pin);
    void reset(void);
    void pinFunction(uint8_t pin, uint8_t function);
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t state);
    bool digitalRead(uint8_t pin);
    void setBitOrder(uint8_t order);
    void setClockDivider(uint8_t divider);
    void setDataMode(uint8_t mode);
    void end(void);
    uint8_t transfer(uint8_t value);
    uint8_t transfer(uint8_t* buffer, uint8_t length);
  private:
    uint8_t i2c_addr;
    uint8_t gpio_config;
    uint8_t gpio_enable;
    uint8_t gpio_write;
    uint8_t spi_interface;
    uint8_t ss_pin;

    void writeRegister(uint8_t cmd, uint8_t data);
    uint8_t readBytes(uint8_t* buffer, uint8_t length);
    void writeBytes(uint8_t* buffer, uint8_t length);
    uint8_t readByte(void);
};
#endif xXI02_h
