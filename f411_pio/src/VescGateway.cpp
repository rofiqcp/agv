#include "VescGateway.h"

#include <cstring>
#include <cstdio>
#include <algorithm>

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
  gVescUart.begin(kBaud);
  // Native STM32Cube priorities keep the VESC UART above USB CDC.
  // USART1 is the dedicated F103 VESC link. Keep its IRQ at the highest
  // peripheral priority even at the standard 115200 baud so motor-active USB/HMI
  // traffic cannot delay RX service. USB CDC to the mini PC remains 1 Mbaud.
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  owner_ = Owner::RUNTIME;
  last_rx_ms_ = HAL_GetTick();
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
  const uint32_t start = HAL_GetTick();
  size_t sent = 0U;
  while (sent < len && static_cast<uint32_t>(HAL_GetTick() - start) < timeout_ms) {
    const int room = gUsb.availableForWrite();
    if (room <= 0) { HAL_Delay(1U); continue; }
    const size_t chunk = std::min(len - sent, static_cast<size_t>(room));
    const size_t n = gUsb.write(data + sent, chunk);
    if (n == 0U) { HAL_Delay(1U); continue; }
    sent += n;
  }
  return sent == len;
}

bool VescGateway::forwardHex(const char *hex, Owner source) {
  if (source != owner_) {
    size_t rejected = strlen(hex) / 2U;
    rejected_bytes_ += static_cast<uint32_t>(rejected);
    char owner_line[48];
    std::snprintf(owner_line, sizeof(owner_line), "VESC:ERR:OWNER:%s", ownerName(owner_));
    (void)gUsb.writeLine(owner_line);
    return false;
  }
  const size_t chars = strlen(hex);
  if (chars == 0U || (chars & 1U) != 0U || chars > kChunkBytes * 2U) {
    (void)gUsb.writeLine("VESC:ERR:HEX_LENGTH");
    return false;
  }
  uint8_t bytes[kChunkBytes];
  const size_t count = chars / 2U;
  for (size_t i = 0; i < count; ++i) {
    const int hi = hexNibble(hex[i * 2U]);
    const int lo = hexNibble(hex[i * 2U + 1U]);
    if (hi < 0 || lo < 0) {
      (void)gUsb.writeLine("VESC:ERR:HEX_DATA");
      return false;
    }
    bytes[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  const size_t written = gVescUart.write(bytes, count);
  tx_bytes_ += static_cast<uint32_t>(written);
  if (source == Owner::RUNTIME && written == count) last_runtime_tx_ms_ = HAL_GetTick();
  if (written != count) {
    (void)gUsb.writeLine("VESC:ERR:UART_TX");
    return false;
  }
  return true;
}

void VescGateway::publishRxFrame(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0U || len > kRxFrameBytes) return;
  static const char hex[] = "0123456789ABCDEF";
  // Keep the raw VESC packet intact and only envelope it for the shared USB CDC.
  // The previous implementation issued many tiny USB writes per VESC byte;
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
    if (gUsb.availableForWrite() < static_cast<int>(out) ||
        gUsb.write(reinterpret_cast<const uint8_t *>(line), out) != out) ++usb_drop_frames_;
    return;
  }
  // Maintenance/config replies may exceed one CDC queue. Stream them with a
  // bounded deadline: reliable while the host is present, but never an infinite block.
  if (!writeUsbBounded(reinterpret_cast<const uint8_t *>(line), out, 400U)) ++usb_drop_frames_;
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
    last_valid_frame_ms_ = HAL_GetTick();
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
  gVescUart.end();
  gVescUart.begin(kBaud);
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
  if (ever_valid_frame_ && recovery_streak_ >= kRuntimeRecoverBeforeReset) NVIC_SystemReset();
}

void VescGateway::publishStatus(bool force) {
  const uint32_t now = HAL_GetTick();
  if (!force && static_cast<uint32_t>(now - last_status_ms_) < kStatusPeriodMs) return;
  last_status_ms_ = now;
  char line[320];
  const int n = snprintf(line, sizeof(line),
    "VESC:STAT:mode=%s,baud=%lu,rx=%lu,tx=%lu,reject=%lu,frames=%lu,frame_err=%lu,"
    "usb_drop=%lu,valid_age_ms=%lu,recover=%lu,age_ms=%lu,rx_lvl=%d,tx_lvl=%d,brr=%lX,cr1=%lX,sr=%lX\n",
    ownerName(owner_), static_cast<unsigned long>(kBaud),
    static_cast<unsigned long>(rx_bytes_), static_cast<unsigned long>(tx_bytes_),
    static_cast<unsigned long>(rejected_bytes_), static_cast<unsigned long>(rx_frames_),
    static_cast<unsigned long>(rx_frame_errors_), static_cast<unsigned long>(usb_drop_frames_),
    static_cast<unsigned long>(now - last_valid_frame_ms_), static_cast<unsigned long>(uart_recovery_count_),
    static_cast<unsigned long>(now - last_rx_ms_),
    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET ? 1 : 0,
    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET ? 1 : 0,
    static_cast<unsigned long>(USART1->BRR), static_cast<unsigned long>(USART1->CR1),
    static_cast<unsigned long>(USART1->SR));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(line)) return;
  if (gUsb.availableForWrite() < n) { ++usb_drop_frames_; return; }
  if (gUsb.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(n)) != static_cast<size_t>(n)) {
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
    gVescUart.end();
    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_7; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &gpio); HAL_Delay(3U);
    const int rxPd = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET ? 1 : 0;
    gpio.Pull = GPIO_PULLUP; HAL_GPIO_Init(GPIOB, &gpio); HAL_Delay(3U);
    const int rxPu = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET ? 1 : 0;
    const char *line_class = (rxPd == 1 && rxPu == 1) ? "DRIVEN_HIGH" :
                             ((rxPd == 0 && rxPu == 0) ? "DRIVEN_LOW" : "FLOATING");
    char line[96];
    std::snprintf(line, sizeof(line), "VESC:LINE:rx_pd=%d,rx_pu=%d,class=%s", rxPd, rxPu, line_class);
    (void)gUsb.writeLine(line);
    (void)gVescUart.begin(kBaud);
    return true;
  }
  if (strcmp(command, "VESC:MODE:RUNTIME") == 0 || strcmp(command, "VESC:MODE:NORMAL") == 0) {
    owner_ = Owner::RUNTIME;
    recovery_streak_ = 0U;
    last_valid_frame_ms_ = HAL_GetTick();
    recovery_tx_marker_ = tx_bytes_;
    while (gVescUart.available() > 0) (void)gVescUart.read();
    rx_len_ = 0U;
    (void)gUsb.writeLine("VESC:MODE:RUNTIME");
    publishStatus(true);
    return true;
  }
  if (strcmp(command, "VESC:MODE:MAINTENANCE") == 0) {
    owner_ = Owner::MAINTENANCE;
    recovery_streak_ = 0U;
    while (gVescUart.available() > 0) (void)gVescUart.read();
    rx_len_ = 0U;
    (void)gUsb.writeLine("VESC:MODE:MAINTENANCE");
    publishStatus(true);
    return true;
  }
  if (strncmp(command, "VESC:TX:R:", 10) == 0) return forwardHex(command + 10, Owner::RUNTIME);
  if (strncmp(command, "VESC:TX:M:", 10) == 0) return forwardHex(command + 10, Owner::MAINTENANCE);
  (void)gUsb.writeLine("VESC:ERR:COMMAND");
  return true;
}

void VescGateway::poll() {
  const uint32_t now = HAL_GetTick();
  if (rx_len_ > 0U && static_cast<uint32_t>(now - last_rx_ms_) > kRxFrameTimeoutMs) {
    rx_frame_errors_ += static_cast<uint32_t>(rx_len_);
    rx_len_ = 0U;
  }

  size_t budget = 640U;
  while (budget-- > 0U && gVescUart.available() > 0) {
    const int value = gVescUart.read();
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
    last_rx_ms_ = HAL_GetTick();
  }
  serviceRxFrames();
  recoveryTick(now);
  if (owner_ == Owner::RUNTIME) publishStatus(false);
}
