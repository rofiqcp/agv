// ============================================================================
// TouchButtons.h — XPT2046 direct navigation + one-tap controls
// Preserves the existing HmiDisplay calibration and coordinate correction.
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include "UsbCdcPort.h"
#include "Config.h"
#include "BottomMenu.h"

extern HmiDisplay tft;

struct TouchEvent {
  enum Type : uint8_t { NONE = 0, PRESS, RELEASE } type;
  ControlAction control;
  PageId navPage;
  bool home;
  bool modeToggle;
  CameraSubPage cameraTab;
  WaypointAction waypointAction;
};

static bool touchWasDown = false;
static ControlAction touchActiveControl = CTRL_NONE;
static PageId touchActiveNav = PAGE_SPLASH;
static bool touchActiveHome = false;
static bool touchActiveModeToggle = false;
static CameraSubPage touchActiveCameraTab = CAM_NONE;
static WaypointAction touchActiveWaypoint = WP_ACTION_NONE;

inline void resetTouchState() {
  touchWasDown = false;
  touchActiveControl = CTRL_NONE;
  touchActiveNav = PAGE_SPLASH;
  touchActiveHome = false;
  touchActiveModeToggle = false;
  touchActiveCameraTab = CAM_NONE;
  touchActiveWaypoint = WP_ACTION_NONE;
}

inline void beginTouch() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  // Exact calibration path from the original HMI revision that was touchable.
  uint16_t calData[5] = { 300, 3600, 300, 3600, 1 };
  tft.setTouch(calData);
  resetTouchState();
}

// Exact coordinate correction from the original working HMI.
inline void correctTouchXY(uint16_t sx, uint16_t sy, uint16_t& tx, uint16_t& ty) {
  const int32_t mappedX = static_cast<int32_t>(sy) * (W - 1) / (H - 1);
  // Preserve the reference integer mapping/truncation order exactly.
  const int32_t mappedY = (H - 1) +
      static_cast<int32_t>(sx) * (0 - (H - 1)) / (W - 1);
  tx = static_cast<uint16_t>(std::clamp<int32_t>(mappedX, 0, W - 1));
  ty = static_cast<uint16_t>(std::clamp<int32_t>(mappedY, 0, H - 1));
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

inline CameraSubPage cameraHitTab(int x, int y) {
  if (!hit(x, y, CAM_TAB_X0, CAM_TAB_Y, W - CAM_TAB_X0, CAM_TAB_H)) return CAM_NONE;
  for (uint8_t i = 0; i < 4; ++i) {
    const int bx = CAM_TAB_X0 + i * (CAM_TAB_W + CAM_TAB_GAP);
    if (hit(x, y, bx, CAM_TAB_Y, CAM_TAB_W, CAM_TAB_H)) {
      return static_cast<CameraSubPage>(i);
    }
  }
  return CAM_NONE;
}

inline WaypointAction gpsHitWaypointAction(int x, int y) {
  if (!hit(x, y, 0, GPS_WP_Y, W, GPS_WP_H)) return WP_ACTION_NONE;
  if (hit(x, y, GPS_WP_PREV_X, GPS_WP_Y, GPS_WP_PREV_W, GPS_WP_H)) return WP_PREV;
  if (hit(x, y, GPS_WP_NAME_X, GPS_WP_Y, GPS_WP_NAME_W, GPS_WP_H)) return WP_ACTION_NONE;
  if (hit(x, y, GPS_WP_NEXT_X, GPS_WP_Y, GPS_WP_NEXT_W, GPS_WP_H)) return WP_NEXT;
  if (hit(x, y, GPS_WP_SAVE_X, GPS_WP_Y, GPS_WP_SAVE_W, GPS_WP_H)) return WP_SAVE;
  if (hit(x, y, GPS_WP_GO_X, GPS_WP_Y, GPS_WP_GO_W, GPS_WP_H)) return WP_GO;
  if (hit(x, y, GPS_WP_STOP_X, GPS_WP_Y, GPS_WP_STOP_W, GPS_WP_H)) return WP_STOP;
  return WP_ACTION_NONE;
}

inline TouchEvent pollTouch(PageId currentPage) {
  TouchEvent ev{TouchEvent::NONE, CTRL_NONE, PAGE_SPLASH, false, false, CAM_NONE, WP_ACTION_NONE};

  // Original working path: HmiDisplay validates pressure/debounce and applies
  // the calibration loaded by beginTouch(). Keep this foundation untouched.
  uint16_t sx = 0, sy = 0;
  const bool down = tft.getTouch(&sx, &sy, TOUCH_THRESHOLD);

  if (!down) {
    if (touchWasDown) {
      touchWasDown = false;
      ev.type = TouchEvent::RELEASE;
      ev.control = touchActiveControl;
      ev.navPage = touchActiveNav;
      ev.home = touchActiveHome;
      ev.modeToggle = touchActiveModeToggle;
      ev.cameraTab = touchActiveCameraTab;
      ev.waypointAction = touchActiveWaypoint;
      touchActiveControl = CTRL_NONE;
      touchActiveNav = PAGE_SPLASH;
      touchActiveHome = false;
      touchActiveModeToggle = false;
      touchActiveCameraTab = CAM_NONE;
      touchActiveWaypoint = WP_ACTION_NONE;
      return ev;
    }
    return ev;
  }

  uint16_t tx = 0, ty = 0;
  correctTouchXY(sx, sy, tx, ty);

  if (!touchWasDown) {
    touchWasDown = true;
    ev.type = TouchEvent::PRESS;

    // Edge-only diagnostics; no extra XPT2046 reads are performed here.
    char touchLog[64];
    std::snprintf(touchLog, sizeof(touchLog), "[TOUCH] PRESS CAL X=%u Y=%u", sx, sy);
    (void)gUsb.writeLine(touchLog);
    std::snprintf(touchLog, sizeof(touchLog), "[TOUCH] SCREEN X=%u Y=%u", tx, ty);
    (void)gUsb.writeLine(touchLog);

    // HOME mode card is a large finger-friendly AUTO/MANUAL toggle.
    if (currentPage == PAGE_HOME && hit(tx, ty, HOME_SIDE_X, HOME_MODE_Y, HOME_SIDE_W, HOME_SIDE_H)) {
      touchActiveModeToggle = true;
      ev.modeToggle = true;
      return ev;
    }

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

    if (currentPage == PAGE_CAMERA) {
      touchActiveCameraTab = cameraHitTab(tx, ty);
      ev.cameraTab = touchActiveCameraTab;
      return ev;
    }

    if (currentPage == PAGE_GPS) {
      touchActiveWaypoint = gpsHitWaypointAction(tx, ty);
      ev.waypointAction = touchActiveWaypoint;
      return ev;
    }

    if (currentPage == PAGE_ACTUATOR) {
      touchActiveControl = actuatorHitControl(tx, ty);
      ev.control = touchActiveControl;
      return ev;
    }
    return ev;
  }

  // No HOLD repeat: the first tap latches or triggers the selected command.
  return ev;
}
