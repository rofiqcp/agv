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
static const float STEER_MIN_DEG = -30.0f;
static const float STEER_MAX_DEG =  30.0f;

// One-tap steering presets. Tune these to the real Ackermann/mechanical
// steering range after measuring the vehicle. LEFT/RIGHT are latched targets;
// CENTER always commands 0 degrees.
static const float STEER_LEFT_PRESET_DEG  = -15.0f;
static const float STEER_RIGHT_PRESET_DEG =  15.0f;

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
