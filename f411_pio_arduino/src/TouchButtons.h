// ============================================================================
// TouchButtons.h — satu input layer untuk HOME, LEFT/RIGHT/OK, UP/DOWN.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "Config.h"

extern TFT_eSPI tft;

struct TouchEvent {
  enum Type : uint8_t { NONE = 0, PRESS, REPEAT, RELEASE } type{NONE};
  SoftKey key{SoftKey::NONE};
};

static bool touchWasDown = false;
static SoftKey touchActiveKey = SoftKey::NONE;
static uint32_t touchPressMs = 0;
static uint32_t touchLastRepeatMs = 0;

inline void resetTouchState() {
  touchWasDown = false;
  touchActiveKey = SoftKey::NONE;
  touchPressMs = 0;
  touchLastRepeatMs = 0;
}

inline void beginTouch() {
  pinMode(PIN_TOUCH_CS, OUTPUT);
  digitalWrite(PIN_TOUCH_CS, HIGH);
  uint16_t calData[5] = {300, 3600, 300, 3600, 1};
  tft.setTouch(calData);
  resetTouchState();
}

inline void correctTouchXY(uint16_t sx, uint16_t sy, uint16_t& tx, uint16_t& ty) {
  tx = map(static_cast<int>(sy), 0, H - 1, 0, W - 1);
  ty = map(static_cast<int>(sx), 0, W - 1, H - 1, 0);
  tx = static_cast<uint16_t>(constrain(static_cast<int>(tx), 0, W - 1));
  ty = static_cast<uint16_t>(constrain(static_cast<int>(ty), 0, H - 1));
}

inline bool hit(int px, int py, int x, int y, int w, int h) {
  return px >= x && px < (x + w) && py >= y && py < (y + h);
}

inline SoftKey touchKeyAt(int x, int y) {
  // HOME hitbox 52x44: visual icon tetap kecil tetapi sentuhan jauh lebih toleran.
  if (hit(x, y, HOME_TOUCH_X, HOME_TOUCH_Y, HOME_TOUCH_W, HOME_TOUCH_H)) return SoftKey::HOME;
  if (hit(x, y, SOFTKEY_X0, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::LEFT;
  if (hit(x, y, SOFTKEY_X0 + SOFTKEY_W + SOFTKEY_GAP, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::RIGHT;
  if (hit(x, y, SOFTKEY_X0 + 2 * (SOFTKEY_W + SOFTKEY_GAP), SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H)) return SoftKey::OK;
  if (hit(x, y, RIGHT_RAIL_X, RIGHT_UP_Y, RIGHT_RAIL_W, RIGHT_KEY_H)) return SoftKey::UP;
  if (hit(x, y, RIGHT_RAIL_X, RIGHT_DOWN_Y, RIGHT_RAIL_W, RIGHT_KEY_H)) return SoftKey::DOWN;
  return SoftKey::NONE;
}

inline TouchEvent pollTouch(bool railEnabled) {
  TouchEvent ev{};
  uint16_t sx = 0, sy = 0;
  const bool down = tft.getTouch(&sx, &sy, TOUCH_THRESHOLD);
  const uint32_t now = millis();

  if (!down) {
    if (touchWasDown) {
      ev.type = TouchEvent::RELEASE;
      ev.key = touchActiveKey;
      resetTouchState();
    }
    return ev;
  }

  uint16_t tx = 0, ty = 0;
  correctTouchXY(sx, sy, tx, ty);
  SoftKey key = touchKeyAt(tx, ty);
  if (!railEnabled && (key == SoftKey::UP || key == SoftKey::DOWN)) key = SoftKey::NONE;

  if (!touchWasDown) {
    touchWasDown = true;
    touchActiveKey = key;
    touchPressMs = now;
    touchLastRepeatMs = now;
    ev.type = TouchEvent::PRESS;
    ev.key = key;
    return ev;
  }

  if (key != touchActiveKey) return ev;
  const bool repeatable = key == SoftKey::UP || key == SoftKey::DOWN;
  if (repeatable && now - touchPressMs >= 450U && now - touchLastRepeatMs >= 120U) {
    touchLastRepeatMs = now;
    ev.type = TouchEvent::REPEAT;
    ev.key = key;
  }
  return ev;
}
