#pragma once

#include <Arduino.h>

class VescGateway {
 public:
  void begin();
  void poll();
  bool handleHostCommand(const char *command);

 private:
  enum class Owner : uint8_t { RUNTIME = 0, MAINTENANCE = 1 };
  static constexpr uint32_t kBaud = 2000000;
  static constexpr size_t kChunkBytes = 48;
  static constexpr uint32_t kRxIdleFlushMs = 2;
  static constexpr uint32_t kStatusPeriodMs = 1000;

  Uart uart_{PB7, PB6};  // RX=PB7, TX=PB6 (USART1 AF7)
  Owner owner_{Owner::RUNTIME};
  uint8_t rx_chunk_[kChunkBytes]{};
  size_t rx_len_{0};
  uint32_t last_rx_ms_{0};
  uint32_t last_status_ms_{0};
  uint32_t rx_bytes_{0};
  uint32_t tx_bytes_{0};
  uint32_t rejected_bytes_{0};

  static int hexNibble(char c);
  static const char *ownerName(Owner owner);
  bool forwardHex(const char *hex, Owner source);
  void flushRx();
  void publishStatus(bool force = false);
};
