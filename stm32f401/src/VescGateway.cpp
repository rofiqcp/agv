#include "VescGateway.h"

#include <string.h>

int VescGateway::hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

const char *VescGateway::ownerName(Owner owner) {
  return owner == Owner::MAINTENANCE ? "MAINTENANCE" : "RUNTIME";
}

void VescGateway::begin() {
  uart_.begin(kBaud);
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

void VescGateway::flushRx() {
  if (rx_len_ == 0U) return;
  static const char hex[] = "0123456789ABCDEF";
  Serial.print(F("VESC:RX:"));
  for (size_t i = 0; i < rx_len_; ++i) {
    const uint8_t b = rx_chunk_[i];
    Serial.write(hex[b >> 4]);
    Serial.write(hex[b & 0x0F]);
  }
  Serial.println();
  rx_len_ = 0U;
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
  size_t budget = 640U;
  while (budget-- > 0U && uart_.available() > 0) {
    const int value = uart_.read();
    if (value < 0) break;
    rx_chunk_[rx_len_++] = static_cast<uint8_t>(value);
    ++rx_bytes_;
    last_rx_ms_ = millis();
    if (rx_len_ >= kChunkBytes) flushRx();
  }
  if (rx_len_ > 0U && static_cast<uint32_t>(millis() - last_rx_ms_) >= kRxIdleFlushMs) flushRx();
  if (owner_ == Owner::RUNTIME) publishStatus(false);
}
