#pragma once

#include "stm32f4xx_hal.h"
#include <cstddef>
#include <cstdint>

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim11;

void Board_Init();
void Board_Service();
void Board_DelayUs(uint32_t microseconds);
void Board_ReinitI2c1();
void Board_SetWatchdogCallback(void (*callback)());
void Board_WatchdogStart();
void Board_WatchdogStop();
void Board_BuzzerStart(uint16_t frequency_hz, uint16_t duration_ms);
void Board_BuzzerStop();

class HalUartPort {
 public:
  explicit HalUartPort(UART_HandleTypeDef *handle) : handle_(handle) {}
  bool begin(uint32_t baudrate);
  void end();
  int available() const;
  int read();
  std::size_t write(const uint8_t *data, std::size_t length);
  std::size_t write(uint8_t byte) { return write(&byte, 1U); }
  void flush();
  uint32_t overflowCount() const { return overflow_count_; }
  void irqRxComplete();
  void irqError();
 private:
  static constexpr uint16_t kRxSize = 2048U;
  UART_HandleTypeDef *handle_;
  volatile uint16_t rx_head_{0U};
  volatile uint16_t rx_tail_{0U};
  uint8_t rx_byte_{0U};
  uint8_t rx_buffer_[kRxSize]{};
  volatile uint32_t overflow_count_{0U};
};

extern HalUartPort gVescUart;
extern HalUartPort gGnssUart;
