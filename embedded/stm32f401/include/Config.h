// ============================================================================
// Config.h — ADV HMI final visual layout for STM32F411 + ILI9341 320x240
// ============================================================================
#pragma once

#include <Arduino.h>

static const int W = 320;
static const int H = 240;
static const int PIN_TOUCH_CS = PA4;

enum PageId : uint8_t {
  PAGE_SPLASH = 0,
  PAGE_HOME,
  PAGE_CAMERA,
  PAGE_GPS,
  PAGE_ACTUATOR
};

enum SystemStatus : uint8_t {
  SYS_OFF = 0,
  SYS_STARTING,
  SYS_INITIALIZING,
  SYS_READY,
  SYS_NOT_READY,
  SYS_FAULT
};

enum VehicleMode : uint8_t { MODE_AUTO = 0, MODE_MANUAL };
enum VehicleState : uint8_t { STATE_STANDBY = 0, STATE_RUNNING, STATE_STOPPED, STATE_FAULT };
enum GpsFixState : uint8_t { GPS_LOST = 0, GPS_NO_FIX, GPS_2D_FIX, GPS_3D_FIX, GPS_DEGRADED };

enum CameraSubPage : uint8_t {
  CAM_VIEW = 0,
  CAM_DETECT,
  CAM_DRIVE,
  CAM_STATUS,
  CAM_NONE = 255
};

enum WaypointAction : uint8_t {
  WP_ACTION_NONE = 0,
  WP_PREV,
  WP_NEXT,
  WP_SAVE,
  WP_GO,
  WP_STOP
};

enum NavigationStatus : uint8_t {
  NAV_IDLE = 0,
  NAV_SELECTED,
  NAV_QUEUED,
  NAV_NAVIGATING,
  NAV_ARRIVED,
  NAV_STOPPED,
  NAV_FAILED
};

static const uint8_t HMI_WAYPOINT_COUNT = 4;
static const uint8_t HMI_WAYPOINT_NAME_LEN = 20;

enum ControlAction : uint8_t {
  CTRL_NONE = 0,
  CTRL_FORWARD,
  CTRL_REVERSE,
  CTRL_LEFT,
  CTRL_RIGHT,
  CTRL_CENTER,
  CTRL_STOP,
  CTRL_SPEED_MINUS,
  CTRL_SPEED_PLUS
};

constexpr uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// Visual palette tuned to the approved 3D-printer-style concept.
static const uint16_t C_BG          = RGB565(9, 18, 30);      // deep navy
static const uint16_t C_PANEL       = RGB565(16, 30, 47);     // nav tile / dark card
static const uint16_t C_PANEL_ALT   = RGB565(27, 42, 60);     // button face
static const uint16_t C_CARD        = RGB565(245, 237, 229);  // warm off-white
static const uint16_t C_TEXT        = RGB565(246, 246, 244);
static const uint16_t C_TEXT_DIM    = RGB565(178, 187, 198);
static const uint16_t C_INK         = RGB565(10, 25, 42);
static const uint16_t C_ACCENT      = RGB565(10, 199, 229);
static const uint16_t C_READY       = RGB565(47, 198, 63);
static const uint16_t C_WARNING     = RGB565(242, 181, 56);
static const uint16_t C_FAULT       = RGB565(255, 59, 48);
static const uint16_t C_DISABLED    = RGB565(105, 119, 136);
static const uint16_t C_BORDER      = RGB565(62, 82, 105);
static const uint16_t C_CARD_LINE   = RGB565(205, 198, 190);
static const uint16_t C_SHADOW      = RGB565(5, 11, 19);
static const uint16_t C_WHITE       = 0xFFFF;
static const uint16_t C_BLACK       = 0x0000;

// Global screen geometry. All coordinates are locked for 320x240 landscape.
static const int TOP_H = 28;
static const int NAV_Y = 185;
static const int NAV_H = 51;
static const int CARD_RADIUS = 7;

// Bottom navigation.
static const int NAV_X0 = 4;
static const int NAV_W = 102;
static const int NAV_GAP = 4;

// HOME.
static const int HOME_MAIN_X = 6;
static const int HOME_MAIN_Y = 34;
static const int HOME_MAIN_W = 228;
static const int HOME_MAIN_H = 146;
static const int HOME_SIDE_X = 240;
static const int HOME_SIDE_W = 74;
static const int HOME_MODE_Y = 34;
static const int HOME_STATE_Y = 110;
static const int HOME_SIDE_H = 70;

// CAMERA submenu. Four 72x27 tabs remain comfortably finger-touchable.
static const int CAM_TAB_Y = 34;
static const int CAM_TAB_H = 30;
static const int CAM_TAB_W = 72;
static const int CAM_TAB_GAP = 5;
static const int CAM_TAB_X0 = 8;
static const int CAM_CONTENT_Y = 68;
static const int CAM_CONTENT_H = 112;

// GPS waypoint controls. The action row is 34 px tall for fingertip use.
static const int GPS_WP_Y = 140;
static const int GPS_WP_H = 36;
static const int GPS_WP_PREV_X = 12;
static const int GPS_WP_PREV_W = 34;
static const int GPS_WP_NAME_X = 49;
static const int GPS_WP_NAME_W = 74;
static const int GPS_WP_NEXT_X = 126;
static const int GPS_WP_NEXT_W = 34;
static const int GPS_WP_SAVE_X = 163;
static const int GPS_WP_SAVE_W = 48;
static const int GPS_WP_GO_X = 214;
static const int GPS_WP_GO_W = 44;
static const int GPS_WP_STOP_X = 261;
static const int GPS_WP_STOP_W = 47;

// CAMERA / GPS full card.
static const int FULL_CARD_X = 6;
static const int FULL_CARD_Y = 34;
static const int FULL_CARD_W = 308;
static const int FULL_CARD_H = 146;

// ACTUATOR.
static const int ACT_CTRL_X = 6;
static const int ACT_CTRL_Y = 34;
static const int ACT_CTRL_W = 194;
static const int ACT_CTRL_H = 146;
static const int ACT_INFO_X = 205;
static const int ACT_INFO_Y = 34;
static const int ACT_INFO_W = 109;
static const int ACT_INFO_H = 146;

static const int ACT_BTN_W = 52;
static const int ACT_BTN_H = 36;
static const int ACT_FWD_X = 77;
static const int ACT_FWD_Y = 39;
static const int ACT_LEFT_X = 15;
static const int ACT_LEFT_Y = 80;
static const int ACT_CENTER_X = 77;
static const int ACT_CENTER_Y = 80;
static const int ACT_RIGHT_X = 139;
static const int ACT_RIGHT_Y = 80;
static const int ACT_REV_X = 77;
static const int ACT_REV_Y = 122;
static const int ACT_STOP_X = 139;
static const int ACT_STOP_Y = 122;

// Conservative manual test defaults. Calibrate to the actual vehicle before motion tests.
static const int MANUAL_SPEED_DEFAULT = 20;
static const int MANUAL_SPEED_MIN = 10;
static const int MANUAL_SPEED_MAX = 50;
static const int MANUAL_SPEED_STEP = 10;
static const float STEER_MIN_DEG = -90.0f;
static const float STEER_MAX_DEG =  90.0f;

// One-tap steering presets use the STM steering command domain: -90..+90 deg.
// Physical wheel angle remains a separate calibrated quantity in ROS/ESC.
// LEFT/RIGHT are latched full-scale targets; CENTER commands 0 degrees.
static const float STEER_LEFT_PRESET_DEG  = -90.0f;
static const float STEER_RIGHT_PRESET_DEG =  90.0f;

// Touch threshold from the original working HMI. TFT_eSPI performs the
// pressure validation/debounce internally through getTouch().
static const uint16_t TOUCH_THRESHOLD = 600;

// Splash timing.
static const int PB_W = 180;
static const int PB_H = 10;
static const int PB_R = 5;
static const int PB_X = (W - PB_W) / 2;
static const int PB_Y = 168;
static const uint32_t PROGRESS_MS = 1900;
static const uint32_t READY_HOLD = 350;
static const uint16_t FRAME_MS = 33;

#ifndef HMI_DEMO_MODE
#define HMI_DEMO_MODE 0
#endif
