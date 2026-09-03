// ============================================================================
// TouchButtons.h — XPT2046 direct navigation + one-tap controls
// Preserves the existing TFT_eSPI calibration and coordinate correction.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "Config.h"
#include "BottomMenu.h"

extern TFT_eSPI tft;

struct TouchEvent {
  enum Type : uint8_t { NONE = 0, PRESS, RELEASE } type;
  ControlAction control;
  PageId navPage;
  bool home;
};

static bool touchWasDown = false;
static ControlAction touchActiveControl = CTRL_NONE;
static PageId touchActiveNav = PAGE_SPLASH;
static bool touchActiveHome = false;

inline void beginTouch() {
  pinMode(PIN_TOUCH_CS, OUTPUT);
  digitalWrite(PIN_TOUCH_CS, HIGH);
  uint16_t calData[5] = { 300, 3600, 300, 3600, 1 };
  tft.setTouch(calData);
}

// Existing panel correction retained from the working project.
inline void correctTouchXY(uint16_t sx, uint16_t sy, uint16_t& tx, uint16_t& ty) {
  tx = map((int)sy, 0, H - 1, 0, W - 1);
  ty = map((int)sx, 0, W - 1, H - 1, 0);
  tx = (uint16_t)constrain((int)tx, 0, W - 1);
  ty = (uint16_t)constrain((int)ty, 0, H - 1);
}

inline bool hit(int px, int py, int x, int y, int w, int h) {
  return px >= x && px < (x + w) && py >= y && py < (y + h);
}

inline PageId navHitPage(int x, int y) {
  if (!hit(x, y, 0, NAV_Y - 2, W, H - (NAV_Y - 2))) return PAGE_SPLASH;
  if (hit(x, y, navTileX(0), NAV_Y, NAV_W, NAV_H)) return PAGE_CAMERA;
  if (hit(x, y, navTileX(1), NAV_Y, NAV_W, NAV_H)) return PAGE_GPS;
  if (hit(x, y, navTileX(2), NAV_Y, NAV_W, NAV_H)) return PAGE_ACTUATOR;
  return PAGE_SPLASH;
}

inline ControlAction actuatorHitControl(int x, int y) {
  if (hit(x, y, ACT_FWD_X, ACT_FWD_Y, ACT_BTN_W, ACT_BTN_H)) return CTRL_FORWARD;
  if (hit(x, y, ACT_LEFT_X, ACT_LEFT_Y, ACT_BTN_W, ACT_BTN_H)) return CTRL_LEFT;
  if (hit(x, y, ACT_CENTER_X, ACT_CENTER_Y, ACT_BTN_W, ACT_BTN_H)) return CTRL_CENTER;
  if (hit(x, y, ACT_RIGHT_X, ACT_RIGHT_Y, ACT_BTN_W, ACT_BTN_H)) return CTRL_RIGHT;
  if (hit(x, y, ACT_REV_X, ACT_REV_Y, ACT_BTN_W, ACT_BTN_H)) return CTRL_REVERSE;
  if (hit(x, y, ACT_STOP_X, ACT_STOP_Y, ACT_BTN_W, ACT_BTN_H)) return CTRL_STOP;
  return CTRL_NONE;
}

inline TouchEvent pollTouch(PageId currentPage) {
  TouchEvent ev{TouchEvent::NONE, CTRL_NONE, PAGE_SPLASH, false};

  uint16_t sx = 0, sy = 0;
  bool down = tft.getTouch(&sx, &sy, TOUCH_THRESHOLD);

  if (!down) {
    if (touchWasDown) {
      touchWasDown = false;
      ev.type = TouchEvent::RELEASE;
      ev.control = touchActiveControl;
      ev.navPage = touchActiveNav;
      ev.home = touchActiveHome;
      touchActiveControl = CTRL_NONE;
      touchActiveNav = PAGE_SPLASH;
      touchActiveHome = false;
      return ev;
    }
    return ev;
  }

  uint16_t tx = 0, ty = 0;
  correctTouchXY(sx, sy, tx, ty);

  if (!touchWasDown) {
    touchWasDown = true;
    ev.type = TouchEvent::PRESS;

    // Home icon touch area on non-splash pages.
    if (currentPage != PAGE_SPLASH && hit(tx, ty, 0, 0, 28, TOP_H)) {
      touchActiveHome = true;
      ev.home = true;
      return ev;
    }

    PageId np = navHitPage(tx, ty);
    if (np != PAGE_SPLASH) {
      touchActiveNav = np;
      ev.navPage = np;
      return ev;
    }

    if (currentPage == PAGE_ACTUATOR) {
      touchActiveControl = actuatorHitControl(tx, ty);
      ev.control = touchActiveControl;
      return ev;
    }
    return ev;
  }

  // No HOLD repeat: the first tap latches the selected actuator command.
  return ev;
}
