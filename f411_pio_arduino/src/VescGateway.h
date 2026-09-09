#pragma once

#include <Arduino.h>

class VescGateway {
 public:
  void begin();
  void poll();
  bool handleHostCommand(const char *command);
  bool maintenanceMode() const { return owner_ == Owner::MAINTENANCE; }
  // Safety input NEO3 memiliki jalur langsung F411 -> F103, tidak bergantung
  // scheduler ROS. Selama aktif, hold E-stop diperbarui periodik.
  void setSafetyStop(bool active);
  bool safetyStopActive() const { return safety_stop_active_; }

 private:
  enum class Owner : uint8_t { RUNTIME = 0, MAINTENANCE = 1 };
  static constexpr uint32_t kBaud = 115200;
  static constexpr size_t kChunkBytes = 512;
  static constexpr size_t kRxFrameBytes = 768;   // F103 VESC_MAX_FRAME <= 707 bytes
  static constexpr size_t kRxBufferBytes = 1536; // noise + at least one whole frame
  static constexpr uint32_t kRxFrameTimeoutMs = 150;
  static constexpr uint32_t kStatusPeriodMs = 1000;
  static constexpr uint32_t kRuntimeNoValidFrameRecoverMs = 1200;
  static constexpr uint32_t kRuntimeRecoverCooldownMs = 1200;
  static constexpr uint8_t kCommMotorEstop = 159U;  // VESC 6.00 COMM_MOTOR_ESTOP
  static constexpr uint16_t kSafetyRefreshHoldMs = 250U;
  static constexpr uint16_t kSafetyReleaseHoldMs = 600U;
  static constexpr uint32_t kSafetyRefreshPeriodMs = 50U;

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
  uint32_t usb_drop_frames_{0};
  uint32_t last_valid_frame_ms_{0};
  uint32_t last_runtime_tx_ms_{0};
  uint32_t last_recovery_ms_{0};
  uint32_t recovery_tx_marker_{0};
  uint32_t uart_recovery_count_{0};
  uint8_t recovery_streak_{0};
  bool ever_valid_frame_{false};
  bool safety_stop_active_{false};
  uint32_t last_safety_stop_ms_{0};

  static int hexNibble(char c);
  static uint16_t crc16(const uint8_t *data, size_t len);
  static const char *ownerName(Owner owner);
  bool forwardHex(const char *hex, Owner source);
  bool writeUsbBounded(const uint8_t *data, size_t len, uint32_t timeout_ms);
  void publishRxFrame(const uint8_t *data, size_t len);
  void serviceRxFrames();
  void publishStatus(bool force = false);
  void recoverRuntimeUart(uint32_t now);
  void recoveryTick(uint32_t now);
  bool sendSafetyStop(uint16_t hold_ms);
};
