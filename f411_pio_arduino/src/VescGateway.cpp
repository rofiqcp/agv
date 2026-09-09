#include "VescGateway.h"

#include <string.h>
#include <stdio.h>

int VescGateway::hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

uint16_t VescGateway::crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0U;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

const char *VescGateway::ownerName(Owner owner) {
  return owner == Owner::MAINTENANCE ? "MAINTENANCE" : "RUNTIME";
}

bool VescGateway::sendSafetyStop(uint16_t hold_ms) {
  // VESC 6.00 COMM_MOTOR_ESTOP = 159, payload uint16 big-endian.
  // Jalur ini dibentuk lokal di F411 sehingga tidak bergantung pada ROS/USB.
  uint8_t payload[3] = {kCommMotorEstop, static_cast<uint8_t>(hold_ms >> 8U), static_cast<uint8_t>(hold_ms)};
  uint8_t frame[8] = {2U, 3U, payload[0], payload[1], payload[2], 0U, 0U, 3U};
  const uint16_t crc = crc16(payload, sizeof(payload));
  frame[5] = static_cast<uint8_t>(crc >> 8U);
  frame[6] = static_cast<uint8_t>(crc);
  const size_t written = uart_.write(frame, sizeof(frame));
  tx_bytes_ += static_cast<uint32_t>(written);
  return written == sizeof(frame);
}

void VescGateway::setSafetyStop(bool active) {
  const uint32_t now = millis();
  if (active) {
    safety_stop_active_ = true;
    if (last_safety_stop_ms_ == 0U ||
        static_cast<uint32_t>(now - last_safety_stop_ms_) >= kSafetyRefreshPeriodMs) {
      (void)sendSafetyStop(kSafetyRefreshHoldMs);
      last_safety_stop_ms_ = now;
    }
    return;
  }
  if (safety_stop_active_) {
    // Hold terakhir lebih panjang dari hard watchdog F103 (maks 500 ms), sehingga
    // command stale yang sempat masuk saat safety aktif sudah pasti kedaluwarsa.
    (void)sendSafetyStop(kSafetyReleaseHoldMs);
  }
  safety_stop_active_ = false;
  last_safety_stop_ms_ = 0U;
}

void VescGateway::begin() {
  uart_.begin(kBaud);
  // STM32 Arduino defaults UART and USB CDC to the same NVIC priority (1).
  // USART1 is the dedicated F103 VESC link. Keep its IRQ at the highest
  // peripheral priority even at the standard 115200 baud so motor-active USB/HMI
  // traffic cannot delay RX service. USB CDC to the mini PC remains 1 Mbaud.
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  owner_ = Owner::RUNTIME;
  last_rx_ms_ = millis();
  last_valid_frame_ms_ = last_rx_ms_;
  last_runtime_tx_ms_ = 0U;
  last_recovery_ms_ = 0U;
  recovery_tx_marker_ = 0U;
  recovery_streak_ = 0U;
  ever_valid_frame_ = false;
  last_status_ms_ = 0;
  publishStatus(true);
}

bool VescGateway::writeUsbBounded(const uint8_t *data, size_t len, uint32_t timeout_ms) {
  if (data == nullptr || len == 0U) return false;
  const uint32_t start = millis();
  size_t sent = 0U;
  while (sent < len && static_cast<uint32_t>(millis() - start) < timeout_ms) {
    const int room = Serial.availableForWrite();
    if (room <= 0) { delay(1); continue; }
    const size_t chunk = std::min(len - sent, static_cast<size_t>(room));
    const size_t n = Serial.write(data + sent, chunk);
    if (n == 0U) { delay(1); continue; }
    sent += n;
  }
  return sent == len;
}

bool VescGateway::forwardHex(const char *hex, Owner source) {
  if (safety_stop_active_) {
    // Saat safety aktif hanya paket E-stop lokal yang boleh mencapai F103.
    // Buang seluruh host stream agar command lama tidak menumpuk untuk diputar
    // kembali setelah tombol dilepas.
    rejected_bytes_ += static_cast<uint32_t>(strlen(hex) / 2U);
    return false;
  }
  if (source != owner_) {
    size_t rejected = strlen(hex) / 2U;
    rejected_bytes_ += static_cast<uint32_t>(rejected);
    Serial.print(F("VESC:ERR:OWNER:"));
    Serial.println(ownerName(owner_));
    return false;
  }
  const size_t chars = strlen(hex);
  if (chars == 0U || (chars & 1U) != 0U || chars > kChunkBytes * 2U) {
    Serial.println(F("VESC:ERR:HEX_LENGTH"));
    return false;
  }
  uint8_t bytes[kChunkBytes];
  const size_t count = chars / 2U;
  for (size_t i = 0; i < count; ++i) {
    const int hi = hexNibble(hex[i * 2U]);
    const int lo = hexNibble(hex[i * 2U + 1U]);
    if (hi < 0 || lo < 0) {
      Serial.println(F("VESC:ERR:HEX_DATA"));
      return false;
    }
    bytes[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  const size_t written = uart_.write(bytes, count);
  tx_bytes_ += static_cast<uint32_t>(written);
  if (source == Owner::RUNTIME && written == count) last_runtime_tx_ms_ = millis();
  if (written != count) {
    Serial.println(F("VESC:ERR:UART_TX"));
    return false;
  }
  return true;
}

void VescGateway::publishRxFrame(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0U || len > kRxFrameBytes) return;
  static const char hex[] = "0123456789ABCDEF";
  // Keep the raw VESC packet intact and only envelope it for the shared USB CDC.
  // The previous implementation issued two Serial.write() calls per VESC byte;
  // at 50-Hz RT polling that created thousands of tiny USB writes per second and
  // competed with GNSS/HMI traffic. Build one line and submit it in one call.
  static char line[8U + (kRxFrameBytes * 2U) + 2U];
  memcpy(line, "VESC:RX:", 8U);
  size_t out = 8U;
  for (size_t i = 0; i < len; ++i) {
    const uint8_t b = data[i];
    line[out++] = hex[b >> 4];
    line[out++] = hex[b & 0x0F];
  }
  line[out++] = '\n';
  if (owner_ == Owner::RUNTIME) {
    if (Serial.availableForWrite() < static_cast<int>(out) ||
        Serial.write(reinterpret_cast<const uint8_t *>(line), out) != out) ++usb_drop_frames_;
    return;
  }
  // Maintenance/config replies may exceed one CDC queue. Stream them with a
  // bounded deadline: reliable while the host is present, but never an infinite block.
  // Jangan tahan cooperative main-loop ratusan ms hanya karena host USB macet.
  // 20 ms cukup untuk frame maintenance normal pada USB CDC 1 Mbps; host dapat
  // retry jika queue penuh, sementara polling safety PB12 tetap bounded.
  if (!writeUsbBounded(reinterpret_cast<const uint8_t *>(line), out, 20U)) ++usb_drop_frames_;
}

void VescGateway::serviceRxFrames() {
  // Port of upstream VESC comm/packet.c packet_process_byte()/try_decode_packet
  // semantics. Decode only from the current read pointer. On invalid framing,
  // discard exactly one byte and retry; on a plausible partial frame, preserve
  // it until more UART bytes arrive. This avoids the previous "scan all offsets"
  // heuristic, which could mistake payload bytes for a new partial header and
  // discard a complete following RT-data reply.
  while (rx_len_ > 0U) {
    const uint8_t start = rx_chunk_[0];
    size_t data_start = 0U;
    size_t payload = 0U;

    if (start == 2U) {
      data_start = 2U;
      if (rx_len_ < data_start) return;
      payload = rx_chunk_[1];
      if (payload < 1U) {
        ++rx_frame_errors_;
        memmove(rx_chunk_, rx_chunk_ + 1U, --rx_len_);
        continue;
      }
    } else if (start == 3U) {
      data_start = 3U;
      if (rx_len_ < data_start) return;
      payload = (static_cast<size_t>(rx_chunk_[1]) << 8U) | rx_chunk_[2];
      // Upstream rejects a long header for a short payload.
      if (payload < 255U) {
        ++rx_frame_errors_;
        memmove(rx_chunk_, rx_chunk_ + 1U, --rx_len_);
        continue;
      }
    } else if (start == 4U) {
      data_start = 4U;
      if (rx_len_ < data_start) return;
      payload = (static_cast<size_t>(rx_chunk_[1]) << 16U) |
                (static_cast<size_t>(rx_chunk_[2]) << 8U) | rx_chunk_[3];
      // F103 VESC_MAX_PAYLOAD is far below the 24-bit packet range. Keeping the
      // upstream short-header rule also makes random legacy byte streams resync.
      if (payload < 65535U) {
        ++rx_frame_errors_;
        memmove(rx_chunk_, rx_chunk_ + 1U, --rx_len_);
        continue;
      }
    } else {
      ++rx_frame_errors_;
      memmove(rx_chunk_, rx_chunk_ + 1U, --rx_len_);
      continue;
    }

    const size_t total = data_start + payload + 3U;
    if (total > kRxFrameBytes) {
      ++rx_frame_errors_;
      memmove(rx_chunk_, rx_chunk_ + 1U, --rx_len_);
      continue;
    }
    if (rx_len_ < total) return;

    const uint16_t expected = static_cast<uint16_t>(
      (static_cast<uint16_t>(rx_chunk_[data_start + payload]) << 8U) |
      rx_chunk_[data_start + payload + 1U]);
    if (rx_chunk_[total - 1U] != 3U ||
        crc16(rx_chunk_ + data_start, payload) != expected) {
      ++rx_frame_errors_;
      memmove(rx_chunk_, rx_chunk_ + 1U, --rx_len_);
      continue;
    }

    ++rx_frames_;
    last_valid_frame_ms_ = millis();
    ever_valid_frame_ = true;
    recovery_streak_ = 0U;
    recovery_tx_marker_ = tx_bytes_;
    publishRxFrame(rx_chunk_, total);
    const size_t remain = rx_len_ - total;
    if (remain > 0U) memmove(rx_chunk_, rx_chunk_ + total, remain);
    rx_len_ = remain;
  }
}

void VescGateway::recoverRuntimeUart(uint32_t now) {
  rx_len_ = 0U;
  uart_.end();
  uart_.begin(kBaud);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  ++uart_recovery_count_;
  if (recovery_streak_ < 0xffU) ++recovery_streak_;
  last_recovery_ms_ = now;
  last_valid_frame_ms_ = now;
  last_rx_ms_ = now;
  recovery_tx_marker_ = tx_bytes_;
}

void VescGateway::recoveryTick(uint32_t now) {
  if (owner_ != Owner::RUNTIME || last_runtime_tx_ms_ == 0U) return;
  if (static_cast<uint32_t>(now - last_runtime_tx_ms_) > 500U) return;
  if (static_cast<uint32_t>(now - last_valid_frame_ms_) < kRuntimeNoValidFrameRecoverMs) return;
  if (static_cast<uint32_t>(now - last_recovery_ms_) < kRuntimeRecoverCooldownMs) return;
  if (tx_bytes_ == recovery_tx_marker_) return;
  recoverRuntimeUart(now);
  // Kegagalan UART ESC tidak boleh mereset seluruh F411 karena board yang sama
  // juga membawa HMI, GNSS, magnetometer, dan jalur E-stop. Recovery tetap lokal
  // pada USART1; F103 watchdog 300 ms memadamkan aktuator bila command terputus.
  // Application watchdog tetap menjadi otoritas reset untuk hang F411 nyata.
}

void VescGateway::publishStatus(bool force) {
  const uint32_t now = millis();
  if (!force && static_cast<uint32_t>(now - last_status_ms_) < kStatusPeriodMs) return;
  last_status_ms_ = now;
  char line[320];
  const int n = snprintf(line, sizeof(line),
    "VESC:STAT:mode=%s,baud=%lu,rx=%lu,tx=%lu,reject=%lu,frames=%lu,frame_err=%lu,"
    "usb_drop=%lu,valid_age_ms=%lu,recover=%lu,recover_streak=%u,ever_valid=%u,safety=%u,age_ms=%lu,rx_lvl=%d,tx_lvl=%d,brr=%lX,cr1=%lX,sr=%lX\n",
    ownerName(owner_), static_cast<unsigned long>(kBaud),
    static_cast<unsigned long>(rx_bytes_), static_cast<unsigned long>(tx_bytes_),
    static_cast<unsigned long>(rejected_bytes_), static_cast<unsigned long>(rx_frames_),
    static_cast<unsigned long>(rx_frame_errors_), static_cast<unsigned long>(usb_drop_frames_),
    static_cast<unsigned long>(now - last_valid_frame_ms_), static_cast<unsigned long>(uart_recovery_count_),
    static_cast<unsigned>(recovery_streak_), ever_valid_frame_ ? 1U : 0U, safety_stop_active_ ? 1U : 0U, static_cast<unsigned long>(now - last_rx_ms_), digitalRead(PB7), digitalRead(PB6),
    static_cast<unsigned long>(USART1->BRR), static_cast<unsigned long>(USART1->CR1),
    static_cast<unsigned long>(USART1->SR));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(line)) return;
  if (Serial.availableForWrite() < n) { ++usb_drop_frames_; return; }
  if (Serial.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(n)) != static_cast<size_t>(n)) {
    ++usb_drop_frames_;
  }
}

bool VescGateway::handleHostCommand(const char *command) {
  if (strncmp(command, "VESC:", 5) != 0) return false;
    if (strcmp(command, "VESC:STATUS") == 0) {
    publishStatus(true);
    return true;
  }
      if (strcmp(command, "VESC:LINECHECK") == 0) {
    uart_.end();
    pinMode(PB7, INPUT_PULLDOWN);
    delay(3);
    const int rxPd = digitalRead(PB7);
    pinMode(PB7, INPUT_PULLUP);
    delay(3);
    const int rxPu = digitalRead(PB7);
    Serial.print(F("VESC:LINE:rx_pd="));
    Serial.print(rxPd);
    Serial.print(F(",rx_pu="));
    Serial.print(rxPu);
    Serial.print(F(",class="));
    if (rxPd == 1 && rxPu == 1) Serial.println(F("DRIVEN_HIGH"));
    else if (rxPd == 0 && rxPu == 0) Serial.println(F("DRIVEN_LOW"));
    else Serial.println(F("FLOATING"));
    uart_.begin(kBaud);
    return true;
  }
  if (strcmp(command, "VESC:MODE:RUNTIME") == 0 || strcmp(command, "VESC:MODE:NORMAL") == 0) {
    owner_ = Owner::RUNTIME;
    recovery_streak_ = 0U;
    last_valid_frame_ms_ = millis();
    recovery_tx_marker_ = tx_bytes_;
    while (uart_.available() > 0) (void)uart_.read();
    rx_len_ = 0U;
    Serial.println(F("VESC:MODE:RUNTIME"));
    publishStatus(true);
    return true;
  }
  if (strcmp(command, "VESC:MODE:MAINTENANCE") == 0) {
    owner_ = Owner::MAINTENANCE;
    recovery_streak_ = 0U;
    while (uart_.available() > 0) (void)uart_.read();
    rx_len_ = 0U;
    Serial.println(F("VESC:MODE:MAINTENANCE"));
    publishStatus(true);
    return true;
  }
  if (strncmp(command, "VESC:TX:R:", 10) == 0) return forwardHex(command + 10, Owner::RUNTIME);
  if (strncmp(command, "VESC:TX:M:", 10) == 0) return forwardHex(command + 10, Owner::MAINTENANCE);
  Serial.println(F("VESC:ERR:COMMAND"));
  return true;
}

void VescGateway::poll() {
  const uint32_t now = millis();
  if (rx_len_ > 0U && static_cast<uint32_t>(now - last_rx_ms_) > kRxFrameTimeoutMs) {
    rx_frame_errors_ += static_cast<uint32_t>(rx_len_);
    rx_len_ = 0U;
  }

  size_t budget = 640U;
  while (budget-- > 0U && uart_.available() > 0) {
    const int value = uart_.read();
    if (value < 0) break;
    if (rx_len_ >= kRxBufferBytes) {
      serviceRxFrames();
      if (rx_len_ >= kRxBufferBytes) {
        ++rx_frame_errors_;
        memmove(rx_chunk_, &rx_chunk_[1], --rx_len_);
      }
    }
    rx_chunk_[rx_len_++] = static_cast<uint8_t>(value);
    ++rx_bytes_;
    last_rx_ms_ = millis();
  }
  serviceRxFrames();
  recoveryTick(now);
  if (owner_ == Owner::RUNTIME) publishStatus(false);
}
