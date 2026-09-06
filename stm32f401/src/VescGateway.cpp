#include "VescGateway.h"

#include <string.h>

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

void VescGateway::begin() {
  uart_.begin(kBaud);
  // STM32 Arduino defaults UART and USB CDC to the same NVIC priority (1).
  // At 1 Mbaud a new byte arrives every ~10 us, so USB must not delay USART1
  // byte service. VESC transport gets the highest peripheral priority; USB stays
  // at 1 and I2C at 2. This only affects the F411 bridge, not F103 motor timing.
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  owner_ = Owner::RUNTIME;
  last_rx_ms_ = millis();
  last_status_ms_ = 0;
  publishStatus(true);
}

bool VescGateway::forwardHex(const char *hex, Owner source) {
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
  const size_t written = Serial.write(reinterpret_cast<const uint8_t *>(line), out);
  if (written == out) ++rx_frames_;
  else rx_frame_errors_ += static_cast<uint32_t>(out - written);
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

    publishRxFrame(rx_chunk_, total);
    const size_t remain = rx_len_ - total;
    if (remain > 0U) memmove(rx_chunk_, rx_chunk_ + total, remain);
    rx_len_ = remain;
  }
}

void VescGateway::publishStatus(bool force) {
  const uint32_t now = millis();
  if (!force && static_cast<uint32_t>(now - last_status_ms_) < kStatusPeriodMs) return;
  last_status_ms_ = now;
  Serial.print(F("VESC:STAT:mode="));
  Serial.print(ownerName(owner_));
  Serial.print(F(",baud="));
  Serial.print(kBaud);
  Serial.print(F(",rx="));
  Serial.print(rx_bytes_);
  Serial.print(F(",tx="));
  Serial.print(tx_bytes_);
  Serial.print(F(",reject="));
  Serial.print(rejected_bytes_);
  Serial.print(F(",frames="));
  Serial.print(rx_frames_);
  Serial.print(F(",frame_err="));
  Serial.print(rx_frame_errors_);
  Serial.print(F(",age_ms="));
  Serial.print(static_cast<uint32_t>(now - last_rx_ms_));
  Serial.print(F(",rx_lvl="));
  Serial.print(digitalRead(PB7));
  Serial.print(F(",tx_lvl="));
  Serial.print(digitalRead(PB6));
  Serial.print(F(",brr="));
  Serial.print(USART1->BRR, HEX);
  Serial.print(F(",cr1="));
  Serial.print(USART1->CR1, HEX);
  Serial.print(F(",sr="));
  Serial.println(USART1->SR, HEX);
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
    while (uart_.available() > 0) (void)uart_.read();
    rx_len_ = 0U;
    Serial.println(F("VESC:MODE:RUNTIME"));
    publishStatus(true);
    return true;
  }
  if (strcmp(command, "VESC:MODE:MAINTENANCE") == 0) {
    owner_ = Owner::MAINTENANCE;
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
  if (owner_ == Owner::RUNTIME) publishStatus(false);
}
