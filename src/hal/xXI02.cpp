#include "xXI02.h"

xXI02::xXI02(void)
{
  i2c_addr = SC18IS602_ADDRESS_111 >> 1;
}
xXI02::xXI02(uint8_t addr
{
  i2c_addr = addr >> 1;
}

bool xXI02::begin(void)
{
  bcm2835_i2c_begin();
  bcm2835_i2c_set_baudrate(10000);
  bcm2835_i2c_setSlaveAddress(i2c_addr);
  reset();
  return true;
}

bool xXI02::begin(uint8_t pin)
{
  bcm2835_i2c_begin();
  bcm2835_i2c_set_baudrate(10000);
  bcm2835_i2c_setSlaveAddress(i2c_addr);
  ss_pin = pin;
  reset();
  pinFunction(ss_pin, 0);
  setClockDivider(SC18IS602_CLOCK_1843K);
  setDataMode(SC18IS602_SPI_MODE0);
  setBitOrder(LSBFIRST); // actually MSBFIRST. 
  return true;
}
void xXI02::pinMode(uint8_t pin, uint8_t mode)
{
  pinFunction(pin, 1); // use pin as GPIO
  uint8_t pos;
  if (mode == OUTPUT) pos = SC18IS601_GPIO_MODE_PUSH_PULL << (pin * 2); // 01 01 01 10
  else pos = SC18IS601_GPIO_MODE_INPUT_ONLY << (pin * 2);
  gpio_config &= ~(1 << pin * 2);
  gpio_config = gpio_config | pos;

  writeRegister(SC18IS601_GPIO_CONFIG, gpio_config);
}

void xXI02::digitalWrite(uint8_t pin, uint8_t state)
{
  if (state == LOW) gpio_write &= ~(1 << pin);
  else gpio_write |= 1 << pin;
  writeRegister(SC18IS601_GPIO_WRITE, gpio_write);
}

bool xXI02::digitalRead(uint8_t pin)
{
  writeRegister(SC18IS601_GPIO_READ, NULL);
  uint8_t val = readByte() & 0x0F;
  bool status = val & (1 << pin);
  SerialUSB.println(status);
  return status;
}

uint8_t xXI02::transfer(uint8_t value)
{
  unsigned char writebuf[2];
  unsigned char readbuf[1];
  writebuf[0] = 0x01 << ss_pin;
  writebuf[1] = value;
  bcm2835_i2c_write_read_rs((char * )&writebuf, 2, (char * )&readbuf, 1);
  return readbuf[0];
}

uint8_t xXI02::transfer(uint8_t * buffer, uint8_t length)
{
  // TO DO
}
void xXI02::writeRegister(uint8_t cmd, uint8_t data)
{
  uint8_t buf[2];
  buf[0] = cmd;
  buf[1] = data;
  bcm2835_i2c_write((const char * )&buf, 2);
}

void xXI02::writeBytes(uint8_t * buffer, uint8_t length)
{
  // TO DO
  return;
}

uint8_t xXI02::readBytes(uint8_t * buffer, uint8_t length)
{
  // TO DO
  return 1;
}

uint8_t xXI02::readByte(void)
{
  char buf[1];
  bcm2835_i2c_read((char*)buf,1);
  return buf[0];
}

void xXI02::reset(void)
{
  for (uint8_t i = 0; i < 4; i++) {
    pinFunction(i, 1);
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
}

void xXI02::pinFunction(uint8_t pin, uint8_t function)
{
  if (function) gpio_enable |= 1 << pin; // set pin for GPIO
  else gpio_enable &= ~(1 << pin); // set pin as slave select
  SerialUSB.print("gpio_enable: ");
  SerialUSB.println(gpio_enable, BIN);

  writeRegister(SC18IS601_GPIO_ENABLE, gpio_enable);
}

void xXI02::setBitOrder(uint8_t order)
{
  spi_interface &= 0x0F; // clear order bit
  order = (order << 6) & 0x0F;
  spi_interface |= order;
  writeRegister(SC18IS601_CONFIG_SPI, spi_interface);
}

void xXI02::setDataMode(uint8_t mode)
{
  spi_interface &= 0xF3; // clear mode bits
  mode = (mode << 2) & 0xF3;
  spi_interface |= mode;
  writeRegister(SC18IS601_CONFIG_SPI, spi_interface);
}

void xXI02::setClockDivider(uint8_t divider)
{
  spi_interface &= 0xFC; // clear clock bits
  spi_interface |= divider;
  writeRegister(SC18IS601_CONFIG_SPI, spi_interface);
}

void xXI02::end(void)
{
  return;
}
