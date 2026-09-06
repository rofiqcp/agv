#pragma once

#include <Arduino.h>

class VescGateway {
 public:
  void begin();
  void poll();
  bool handleHostCommand(const char *command);
  bool maintenanceMode() const { return owner_ == Owner::MAINTENANCE; }

 private:
  enum class Owner : uint8_t { RUNTIME = 0, MAINTENANCE = 1 };
  static constexpr uint32_t kBaud = 1000000;
  static constexpr size_t kChunkBytes = 512;
  static constexpr size_t kRxFrameBytes = 768;   // F103 VESC_MAX_FRAME <= 707 bytes
  static constexpr size_t kRxBufferBytes = 1536; // noise + at least one whole frame
  static constexpr uint32_t kRxFrameTimeoutMs = 50;
  static constexpr uint32_t kStatusPeriodMs = 1000;

  Uart uart_{PB7, PB6};  // RX=PB7, TX=PB6 (USART1 AF7)
  Owner owner_{Owner::RUNTIME};
  uint8_t rx_chunk_[kRxBufferBytes]{};
  size_t rx_len_{0};
  uint32_t last_rx_ms_{0};
  uint32_t last_status_ms_{0};
  uint32_t rx_bytes_{0};
  uint32_t tx_bytes_{0};
  uint32_t rejected_bytes_{0};
  uint32_t rx_frames_{0};
  uint32_t rx_frame_errors_{0};

  static int hexNibble(char c);
  static uint16_t crc16(const uint8_t *data, size_t len);
  static const char *ownerName(Owner owner);
  bool forwardHex(const char *hex, Owner source);
  void publishRxFrame(const uint8_t *data, size_t len);
  void serviceRxFrames();
  void publishStatus(bool force = false);
};
