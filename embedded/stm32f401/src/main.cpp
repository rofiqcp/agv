// ============================================================================
// ADV HMI Firmware — STM32F411CEU6 + ILI9341 320x240 + XPT2046
// Final UI architecture: HOME / CAMERA / GPS / ACTUATOR
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "Config.h"
#include "Telemetry.h"
#include "Theme.h"
#include "Icons.h"
#include "SplashScreen.h"
#include "HomePage.h"
#include "CameraPage.h"
#include "GpsPage.h"
#include "ActuatorPage.h"
#include "TouchButtons.h"

TFT_eSPI tft = TFT_eSPI();
VehicleTelemetry gTelemetry = defaultTelemetry();

static PageId currentPage = PAGE_SPLASH;
static bool splashComplete = false;
static uint8_t splashProgress = 0;
static bool splashReadyText = false;
static uint32_t splashStartMs = 0;
static uint32_t splashReadyMs = 0;
static uint32_t lastFrameMs = 0;

static bool uiDirty = false;
static uint32_t lastUiRefreshMs = 0;

// Latched manual-control state. A single tap starts/changes the command;
// releasing the finger does NOT cancel it. STOP, mode changes, faults, or
// leaving the ACTUATOR page cancel the latched drive command.
static ControlAction activeDriveControl = CTRL_NONE;   // FWD / REV / NONE
static ControlAction activeSteerControl = CTRL_NONE;   // LEFT / RIGHT / CENTER / NONE

static char serialRx[128];
static uint8_t serialRxLen = 0;
static char serial1Rx[128];
static uint8_t serial1RxLen = 0;

// ---------------------------------------------------------------------------
// Forward declarations used by page/safety helpers
// ---------------------------------------------------------------------------
static void sendDriveStop();

// ---------------------------------------------------------------------------
// Page helpers
// ---------------------------------------------------------------------------
static const char* pageName(PageId page) {
  switch (page) {
    case PAGE_SPLASH:   return "SPLASH";
    case PAGE_HOME:     return "HOME";
    case PAGE_CAMERA:   return "CAMERA";
    case PAGE_GPS:      return "GPS";
    case PAGE_ACTUATOR: return "ACTUATOR";
    default:            return "UNKNOWN";
  }
}

static void printBoth(const char* line) {
  Serial.println(line);
  Serial1.println(line);
}

static void publishPage() {
  Serial.print(F("PAGE:"));
  Serial.println(pageName(currentPage));
  Serial1.print(F("PAGE:"));
  Serial1.println(pageName(currentPage));
}

static void drawCurrentPage(bool fullDraw = true) {
  switch (currentPage) {
    case PAGE_HOME:
      if (fullDraw) drawHomePage(gTelemetry);
      else updateHomePage(gTelemetry);
      break;
    case PAGE_CAMERA:
      if (fullDraw) drawCameraPage(gTelemetry);
      else updateCameraPage(gTelemetry);
      break;
    case PAGE_GPS:
      if (fullDraw) drawGpsPage(gTelemetry);
      else updateGpsPage(gTelemetry);
      break;
    case PAGE_ACTUATOR:
      if (fullDraw) drawActuatorPage(gTelemetry, activeDriveControl, activeSteerControl);
      else updateActuatorPage(gTelemetry, activeDriveControl, activeSteerControl);
      break;
    case PAGE_SPLASH:
    default:
      break;
  }
}

static void showPage(PageId page) {
  if (page == PAGE_SPLASH) return;
  if (!splashComplete) {
    splashComplete = true;
    beginTouch();
  }

  // Manual drive is intentionally latched after one tap, but only while the
  // operator stays on the ACTUATOR page. Leaving that page performs a safe
  // software stop so the vehicle cannot keep moving while STOP is hidden.
  if (currentPage == PAGE_ACTUATOR && page != PAGE_ACTUATOR &&
      (activeDriveControl == CTRL_FORWARD || activeDriveControl == CTRL_REVERSE)) {
    sendDriveStop();
    activeDriveControl = CTRL_NONE;
  }

  currentPage = page;
  // Do not clear touchWasDown here: when navigation was triggered by a finger
  // press, keeping the down-state prevents the same held touch from re-firing.
  drawCurrentPage(true);
  publishPage();
}

static void restartSplash() {
  if (activeDriveControl == CTRL_FORWARD || activeDriveControl == CTRL_REVERSE) {
    sendDriveStop();
  }
  currentPage = PAGE_SPLASH;
  splashComplete = false;
  splashProgress = 0;
  splashReadyText = false;
  lastFrameMs = 0;
  activeDriveControl = CTRL_NONE;
  activeSteerControl = CTRL_NONE;
  touchWasDown = false;
  gTelemetry.systemStatus = SYS_INITIALIZING;
  drawSplashScreen();
  splashStartMs = millis();
  publishPage();
}

// ---------------------------------------------------------------------------
// Command output for manual actuator test
// ---------------------------------------------------------------------------
static void sendDriveCommand(const char* direction) {
  char line[48];
  snprintf(line, sizeof(line), "CMD:DRIVE:%s:%u", direction, gTelemetry.manualSpeedPct);
  printBoth(line);
  gTelemetry.state = STATE_RUNNING;
  uiDirty = true;
}

static void sendDriveStop() {
  printBoth("CMD:DRIVE:STOP");
  gTelemetry.state = STATE_STOPPED;
  uiDirty = true;
}

static void sendSteerTarget(float targetDeg) {
  targetDeg = constrain(targetDeg, STEER_MIN_DEG, STEER_MAX_DEG);
  gTelemetry.steeringTargetDeg = targetDeg;
  gTelemetry.steeringErrorDeg = gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;

  char line[48];
  snprintf(line, sizeof(line), "CMD:STEER:%.1f", targetDeg);
  printBoth(line);
  uiDirty = true;
}

static void setManualSpeed(int value) {
  value = constrain(value, MANUAL_SPEED_MIN, MANUAL_SPEED_MAX);
  gTelemetry.manualSpeedPct = (uint8_t)value;
  char line[32];
  snprintf(line, sizeof(line), "CMD:SPEED:%u", gTelemetry.manualSpeedPct);
  printBoth(line);
  uiDirty = true;
}

// ---------------------------------------------------------------------------
// Touch behavior
// ---------------------------------------------------------------------------
static bool speedAdjustEnabled() {
  return gTelemetry.systemStatus == SYS_READY && gTelemetry.mode == MODE_MANUAL;
}

static bool controlAllowed(ControlAction action) {
  if (action == CTRL_STOP) return true;
  if (action == CTRL_FORWARD || action == CTRL_REVERSE) return manualDriveEnabled(gTelemetry);
  if (action == CTRL_LEFT || action == CTRL_RIGHT || action == CTRL_CENTER) return manualSteerEnabled(gTelemetry);
  if (action == CTRL_SPEED_MINUS || action == CTRL_SPEED_PLUS) return speedAdjustEnabled();
  return false;
}

static void handleControlTap(ControlAction action) {
  if (action == CTRL_NONE) return;

  if (!controlAllowed(action)) {
    // Do not send movement commands while AUTO/not-ready/faulted.
    printBoth("ERR:MANUAL_CONTROL_LOCKED");
    return;
  }

  switch (action) {
    case CTRL_FORWARD:
      // If reversing, explicitly stop before changing direction.
      if (activeDriveControl == CTRL_REVERSE) sendDriveStop();
      sendDriveCommand("FWD");
      activeDriveControl = CTRL_FORWARD;
      break;

    case CTRL_REVERSE:
      // If moving forward, explicitly stop before changing direction.
      if (activeDriveControl == CTRL_FORWARD) sendDriveStop();
      sendDriveCommand("REV");
      activeDriveControl = CTRL_REVERSE;
      break;

    case CTRL_LEFT:
      // One tap commands a fixed left steering position.
      sendSteerTarget(STEER_LEFT_PRESET_DEG);
      activeSteerControl = CTRL_LEFT;
      break;

    case CTRL_RIGHT:
      // One tap commands a fixed right steering position.
      sendSteerTarget(STEER_RIGHT_PRESET_DEG);
      activeSteerControl = CTRL_RIGHT;
      break;

    case CTRL_CENTER:
      sendSteerTarget(0.0f);
      activeSteerControl = CTRL_CENTER;
      break;

    case CTRL_STOP:
      sendDriveStop();
      activeDriveControl = CTRL_NONE;
      break;

    case CTRL_SPEED_MINUS:
      setManualSpeed((int)gTelemetry.manualSpeedPct - MANUAL_SPEED_STEP);
      // If already driving, immediately re-issue the latched command at the new speed.
      if (activeDriveControl == CTRL_FORWARD) sendDriveCommand("FWD");
      else if (activeDriveControl == CTRL_REVERSE) sendDriveCommand("REV");
      break;

    case CTRL_SPEED_PLUS:
      setManualSpeed((int)gTelemetry.manualSpeedPct + MANUAL_SPEED_STEP);
      if (activeDriveControl == CTRL_FORWARD) sendDriveCommand("FWD");
      else if (activeDriveControl == CTRL_REVERSE) sendDriveCommand("REV");
      break;

    default:
      break;
  }

  if (currentPage == PAGE_ACTUATOR) {
    updateActuatorPage(gTelemetry, activeDriveControl, activeSteerControl);
  }
}

static void handleTouch() {
  TouchEvent ev = pollTouch(currentPage);
  if (ev.type == TouchEvent::NONE) return;

  // All controls are edge-triggered: one PRESS = one latched command.
  // RELEASE does not cancel/repeat motion commands.
  if (ev.type == TouchEvent::PRESS) {
    if (ev.home) {
      showPage(PAGE_HOME);
      return;
    }
    if (ev.navPage != PAGE_SPLASH) {
      showPage(ev.navPage);
      return;
    }
    if (currentPage == PAGE_ACTUATOR && ev.control != CTRL_NONE) {
      handleControlTap(ev.control);
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// Serial telemetry parser
// GUI/ROS2 -> HMI examples:
// SYS:READY, MODE:MANUAL, STATE:STOPPED, SPD:1.2, HEAD:32.0,
// GPS:1, FIX:3, LAT:-7.050123, LON:110.440235, SAT:17, HDOP:0.82,
// IMU:1, CAM:1, PER:1, OBJ:PERSON, DIST:3.24, CONF:92,
// DRV:1, OBS:0, STEER_TARGET:15, STEER_ACTUAL:14.8, RPM:328, ESC:1, ENC:1
// ---------------------------------------------------------------------------
static bool eqIgnoreCase(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return false;
  }
  return *a == '\0' && *b == '\0';
}

static bool parseBool(const char* s) {
  return !strcmp(s, "1") || eqIgnoreCase(s, "ON") || eqIgnoreCase(s, "READY") || eqIgnoreCase(s, "TRUE");
}

static void parseSystemStatus(const char* s) {
  if (eqIgnoreCase(s, "OFF")) gTelemetry.systemStatus = SYS_OFF;
  else if (eqIgnoreCase(s, "STARTING")) gTelemetry.systemStatus = SYS_STARTING;
  else if (eqIgnoreCase(s, "INITIALIZING")) gTelemetry.systemStatus = SYS_INITIALIZING;
  else if (eqIgnoreCase(s, "READY") || eqIgnoreCase(s, "VEHICLE READY")) gTelemetry.systemStatus = SYS_READY;
  else if (eqIgnoreCase(s, "NOT READY")) gTelemetry.systemStatus = SYS_NOT_READY;
  else if (eqIgnoreCase(s, "FAULT")) gTelemetry.systemStatus = SYS_FAULT;
}

static void parseVehicleState(const char* s) {
  if (eqIgnoreCase(s, "STANDBY")) gTelemetry.state = STATE_STANDBY;
  else if (eqIgnoreCase(s, "RUNNING")) gTelemetry.state = STATE_RUNNING;
  else if (eqIgnoreCase(s, "STOPPED") || eqIgnoreCase(s, "STOP")) gTelemetry.state = STATE_STOPPED;
  else if (eqIgnoreCase(s, "FAULT")) gTelemetry.state = STATE_FAULT;
}

static void handleSerialCommand(char* command) {
  while (*command == ' ' || *command == '\t') command++;
  if (!*command) return;

  // Navigation / health commands
  if (!strcmp(command, "GET:STATE")) {
    publishPage();
    return;
  }
  if (!strcmp(command, "PING")) {
    printBoth("ACK:PONG");
    publishPage();
    return;
  }
  if (!strcmp(command, "GOTO:HOME")) { showPage(PAGE_HOME); return; }
  if (!strcmp(command, "GOTO:CAMERA")) { showPage(PAGE_CAMERA); return; }
  if (!strcmp(command, "GOTO:GPS")) { showPage(PAGE_GPS); return; }
  if (!strcmp(command, "GOTO:ACTUATOR")) { showPage(PAGE_ACTUATOR); return; }
  if (!strcmp(command, "GOTO:SPLASH")) { restartSplash(); return; }

  // Legacy page aliases from the previous firmware, kept for old GUI compatibility.
  if (!strcmp(command, "GOTO:SENSOR")) { showPage(PAGE_GPS); return; }
  if (!strcmp(command, "GOTO:AUTO")) { showPage(PAGE_ACTUATOR); return; }
  if (!strcmp(command, "GOTO:INFO")) { showPage(PAGE_HOME); return; }

  if (!strncmp(command, "SYS:", 4)) {
    parseSystemStatus(command + 4);
    if (gTelemetry.systemStatus != SYS_READY &&
        (activeDriveControl == CTRL_FORWARD || activeDriveControl == CTRL_REVERSE)) {
      sendDriveStop();
      activeDriveControl = CTRL_NONE;
    }
    if (gTelemetry.systemStatus != SYS_READY) {
      activeSteerControl = CTRL_NONE;
    }
  } else if (!strncmp(command, "MODE:", 5)) {
    gTelemetry.mode = eqIgnoreCase(command + 5, "MANUAL") ? MODE_MANUAL : MODE_AUTO;
    if (gTelemetry.mode == MODE_AUTO) {
      if (activeDriveControl == CTRL_FORWARD || activeDriveControl == CTRL_REVERSE) {
        sendDriveStop();
      }
      activeDriveControl = CTRL_NONE;
      activeSteerControl = CTRL_NONE;
    }
  } else if (!strncmp(command, "STATE:", 6)) {
    parseVehicleState(command + 6);
    if (gTelemetry.state == STATE_STOPPED || gTelemetry.state == STATE_FAULT) {
      activeDriveControl = CTRL_NONE;
    }
  } else if (!strncmp(command, "SPD:", 4)) {
    gTelemetry.speedKmh = atof(command + 4);
  } else if (!strncmp(command, "HEAD:", 5)) {
    gTelemetry.headingDeg = atof(command + 5);
  } else if (!strncmp(command, "GPS:", 4)) {
    gTelemetry.gpsReady = parseBool(command + 4);
  } else if (!strncmp(command, "FIX:", 4)) {
    int fix = atoi(command + 4);
    if (fix <= 1) gTelemetry.gpsFix = GPS_NO_FIX;
    else if (fix == 2) gTelemetry.gpsFix = GPS_2D_FIX;
    else if (fix == 3) gTelemetry.gpsFix = GPS_3D_FIX;
    else if (fix == 4) gTelemetry.gpsFix = GPS_DEGRADED;
  } else if (!strncmp(command, "LAT:", 4)) {
    gTelemetry.latitude = atof(command + 4);
  } else if (!strncmp(command, "LON:", 4)) {
    gTelemetry.longitude = atof(command + 4);
  } else if (!strncmp(command, "SAT:", 4)) {
    gTelemetry.satellites = (uint8_t)constrain(atoi(command + 4), 0, 99);
  } else if (!strncmp(command, "HDOP:", 5)) {
    gTelemetry.hdop = atof(command + 5);
  } else if (!strncmp(command, "IMU:", 4)) {
    gTelemetry.imuReady = parseBool(command + 4);
  } else if (!strncmp(command, "CAM:", 4)) {
    gTelemetry.cameraReady = parseBool(command + 4);
  } else if (!strncmp(command, "PER:", 4)) {
    gTelemetry.perceptionReady = parseBool(command + 4);
  } else if (!strncmp(command, "OBJ:", 4)) {
    snprintf(gTelemetry.detectedObject, sizeof(gTelemetry.detectedObject), "%s", command + 4);
  } else if (!strncmp(command, "DIST:", 5)) {
    gTelemetry.objectDistanceM = atof(command + 5);
  } else if (!strncmp(command, "CONF:", 5)) {
    gTelemetry.confidencePct = atof(command + 5);
  } else if (!strncmp(command, "DRV:", 4)) {
    gTelemetry.drivableAreaClear = parseBool(command + 4);
  } else if (!strncmp(command, "OBS:", 4)) {
    gTelemetry.obstacleDetected = parseBool(command + 4);
  } else if (!strncmp(command, "STEER_TARGET:", 13)) {
    gTelemetry.steeringTargetDeg = constrain((float)atof(command + 13), STEER_MIN_DEG, STEER_MAX_DEG);
    gTelemetry.steeringErrorDeg = gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;
  } else if (!strncmp(command, "STEER_ACTUAL:", 13)) {
    gTelemetry.steeringActualDeg = atof(command + 13);
    gTelemetry.steeringErrorDeg = gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;
  } else if (!strncmp(command, "STEER_ERR:", 10)) {
    gTelemetry.steeringErrorDeg = atof(command + 10);
  } else if (!strncmp(command, "RPM:", 4)) {
    gTelemetry.motorRpm = atof(command + 4);
  } else if (!strncmp(command, "ESC:", 4)) {
    gTelemetry.escReady = parseBool(command + 4);
    if (!gTelemetry.escReady &&
        (activeDriveControl == CTRL_FORWARD || activeDriveControl == CTRL_REVERSE)) {
      sendDriveStop();
      activeDriveControl = CTRL_NONE;
    }
  } else if (!strncmp(command, "ENC:", 4)) {
    gTelemetry.encoderReady = parseBool(command + 4);
    if (!gTelemetry.encoderReady) activeSteerControl = CTRL_NONE;
  } else if (!strncmp(command, "MANUAL_SPEED:", 13)) {
    gTelemetry.manualSpeedPct = (uint8_t)constrain(atoi(command + 13), MANUAL_SPEED_MIN, MANUAL_SPEED_MAX);
  } else {
    Serial.print(F("ERR:UNKNOWN_COMMAND:"));
    Serial.println(command);
    Serial1.print(F("ERR:UNKNOWN_COMMAND:"));
    Serial1.println(command);
    return;
  }

  uiDirty = true;
}

static void pollSerialStream(Stream& io, char* rx, uint8_t& rxLen) {
  while (io.available() > 0) {
    char c = (char)io.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rx[rxLen] = '\0';
      handleSerialCommand(rx);
      rxLen = 0;
    } else if (rxLen < 127) {
      rx[rxLen++] = c;
    } else {
      rxLen = 0;
      io.println(F("ERR:COMMAND_TOO_LONG"));
    }
  }
}

static void pollSerialGui() {
  pollSerialStream(Serial, serialRx, serialRxLen);
  pollSerialStream(Serial1, serial1Rx, serial1RxLen);
}

// ---------------------------------------------------------------------------
// Display / splash
// ---------------------------------------------------------------------------
static void initDisplay() {
  pinMode(PIN_TOUCH_CS, OUTPUT);
  digitalWrite(PIN_TOUCH_CS, HIGH);
  tft.init();
  tft.setRotation(1);
  // RGB565 illustration arrays use standard byte order.
  tft.setSwapBytes(true);
  tft.fillScreen(C_BG);
  tft.setTextDatum(MC_DATUM);
}

static bool updateProgressBar() {
  if (splashComplete) return false;

  uint32_t now = millis();
  if (now - lastFrameMs < FRAME_MS) return true;
  lastFrameMs = now;

  uint32_t elapsed = now - splashStartMs;
  if (elapsed < PROGRESS_MS) {
    uint8_t target = (uint8_t)((elapsed * 100UL) / PROGRESS_MS);
    if (target != splashProgress) {
      splashProgress = target;
      int innerW = PB_W - 4;
      int fillW = (int)((splashProgress * innerW) / 100);
      tft.fillRoundRect(PB_X + 2, PB_Y + 2, innerW, PB_H - 4, PB_R - 2, C_BG);
      if (fillW > 0) tft.fillRoundRect(PB_X + 2, PB_Y + 2, fillW, PB_H - 4, PB_R - 2, C_ACCENT);
    }
    return true;
  }

  if (!splashReadyText) {
    splashReadyText = true;
    splashReadyMs = now;
    tft.fillRoundRect(PB_X + 2, PB_Y + 2, PB_W - 4, PB_H - 4, PB_R - 2, C_READY);
    tft.fillRect(70, 186, 180, 18, C_BG);
    drawUiText("System ready", W / 2, 187, C_READY, C_BG, MC_DATUM);
  }

  if (splashReadyText && now - splashReadyMs >= READY_HOLD) {
    splashComplete = true;
    beginTouch();
    if (gTelemetry.systemStatus == SYS_INITIALIZING) gTelemetry.systemStatus = SYS_NOT_READY;
    showPage(PAGE_HOME);
    return false;
  }
  return true;
}

#if HMI_DEMO_MODE
static void updateDemoTelemetry() {
  static uint32_t demoMs = 0;
  if (millis() - demoMs < 700) return;
  demoMs = millis();

  gTelemetry.systemStatus = SYS_READY;
  gTelemetry.mode = MODE_MANUAL;
  gTelemetry.state = STATE_STOPPED;
  gTelemetry.gpsReady = true;
  gTelemetry.gpsFix = GPS_3D_FIX;
  gTelemetry.latitude = -7.050123;
  gTelemetry.longitude = 110.440235;
  gTelemetry.satellites = 17;
  gTelemetry.hdop = 0.82f;
  gTelemetry.imuReady = true;
  gTelemetry.cameraReady = true;
  gTelemetry.perceptionReady = true;
  snprintf(gTelemetry.detectedObject, sizeof(gTelemetry.detectedObject), "%s", "PERSON");
  gTelemetry.objectDistanceM = 3.24f;
  gTelemetry.drivableAreaClear = true;
  gTelemetry.obstacleDetected = false;
  gTelemetry.escReady = true;
  gTelemetry.encoderReady = true;
  gTelemetry.speedKmh = 0.0f;
  gTelemetry.headingDeg += 1.0f;
  if (gTelemetry.headingDeg >= 360.0f) gTelemetry.headingDeg = 0.0f;
  gTelemetry.steeringActualDeg = gTelemetry.steeringTargetDeg - 0.2f;
  gTelemetry.steeringErrorDeg = gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;
  gTelemetry.motorRpm = 328.0f;
  uiDirty = true;
}
#endif

void setup() {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  Serial.begin(115200);
  Serial1.begin(115200);
  delay(50);
  Serial.println(F("ADV HMI visual precise TAP control — boot"));

  initDisplay();
  restartSplash();
}

void loop() {
  pollSerialGui();

  if (!splashComplete) {
    updateProgressBar();
  } else {
    handleTouch();

#if HMI_DEMO_MODE
    updateDemoTelemetry();
#endif

    // Telemetry redraw is throttled and only redraws top/content areas, not the
    // entire screen/bottom navigation, reducing flicker and SPI load.
    if (uiDirty && millis() - lastUiRefreshMs >= 80) {
      lastUiRefreshMs = millis();
      uiDirty = false;
      drawCurrentPage(false);
    }
  }

  // Heartbeat LED (PC13 active-low on many Black Pill boards)
  static uint32_t ledMs = 0;
  if (millis() - ledMs >= 500) {
    ledMs = millis();
    digitalWrite(PC13, !digitalRead(PC13));
  }
}
