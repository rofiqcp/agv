// ============================================================================
// ADV HMI Firmware — STM32F411CEU6 + ILI9341 320x240 + XPT2046
// Final UI architecture: HOME / CAMERA / GPS / ACTUATOR
// ============================================================================

#include "BoardSupport.h"
#include "UsbCdcPort.h"
#include "HmiDisplay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Config.h"
#include "Telemetry.h"
#include "Neo3Sensors.h"
#include "VescGateway.h"
#include "Theme.h"
#include "Icons.h"
#include "SplashScreen.h"
#include "HomePage.h"
#include "CameraPage.h"
#include "GpsPage.h"
#include "ActuatorPage.h"
#include "TouchButtons.h"

HmiDisplay tft;
VehicleTelemetry gTelemetry = defaultTelemetry();
Neo3Sensors gNeo3;
VescGateway gVesc;

// Application watchdog: unlike IWDG this timer is stopped before ROM-DFU, so
// firmware updates cannot be interrupted. The ISR only resets when the entire
// cooperative main loop fails to complete for several seconds.
static volatile uint32_t gMainLoopHeartbeatMs = 0U;
static volatile bool gAppWatchdogArmed = false;
static constexpr uint32_t APP_WATCHDOG_TIMEOUT_MS = 3500U;

static void appWatchdogIsr() {
  if (!gAppWatchdogArmed) return;
  const uint32_t now = HAL_GetTick();
  if (static_cast<uint32_t>(now - gMainLoopHeartbeatMs) > APP_WATCHDOG_TIMEOUT_MS) {
    NVIC_SystemReset();
  }
}

static void startAppWatchdog() {
  gMainLoopHeartbeatMs = HAL_GetTick();
  Board_SetWatchdogCallback(appWatchdogIsr);
  Board_WatchdogStart();
  gAppWatchdogArmed = false;
}

static void stopAppWatchdog() {
  gAppWatchdogArmed = false;
  Board_WatchdogStop();
}


static PageId currentPage = PAGE_SPLASH;
static CameraSubPage currentCameraTab = CAM_VIEW;
static bool splashComplete = false;
static uint8_t splashProgress = 0;
static bool splashReadyText = false;
static uint32_t splashStartMs = 0;
static uint32_t splashReadyMs = 0;
static uint32_t lastFrameMs = 0;

static bool uiDirty = false;
static uint32_t lastUiRefreshMs = 0;
static uint32_t lastRosHeartbeatMs = 0;
static uint32_t lastTouchPollMs = 0;
static uint8_t rosHeartbeatStableCount = 0;
static bool rosHeartbeatStable = false;

// Latched manual-control state. A single tap starts/changes the command;
// releasing the finger does NOT cancel it. STOP, mode changes, faults, or
// leaving the ACTUATOR page cancel the latched drive command.
static ControlAction activeDriveControl = CTRL_NONE;   // FWD / REV / NONE
static ControlAction activeSteerControl = CTRL_NONE;   // LEFT / RIGHT / CENTER / NONE

// Long VESC config/firmware frames are transported as hexadecimal commands.
// 640 bytes safely holds a 240-byte raw chunk plus namespace/terminator.
static char serialRx[640];
static size_t serialRxLen = 0;
static bool serialRxDiscarding = false;

// ---------------------------------------------------------------------------
// Forward declarations used by page/safety helpers
// ---------------------------------------------------------------------------
static void sendDriveStop();
static void publishControlState();
static void publishCameraTab();
static void enterSystemDfu();

// ---------------------------------------------------------------------------
// USB firmware-update helper
// STM32F411 system-memory bootloader starts at 0x1FFF0000 and exposes USB DFU.
// We reset first, then jump from .preinit_array before HAL/USB application startup,
// avoiding stale USB/peripheral state during erase/program operations.
// ---------------------------------------------------------------------------
static constexpr uint32_t kBootRequestMagic = 0x42465544UL;  // "DFUB"

static void enterSystemDfu() {
  stopAppWatchdog();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  for (volatile uint32_t i = 0; i < 1000U; ++i) __NOP();
  RTC->BKP0R = kBootRequestMagic;
  __DSB();
  __ISB();

  // Match the reference transaction: best-effort ACK, clean CDC disconnect,
  // then reset into the dedicated recovery bootloader at 0x08000000.
  HAL_Delay(20U);
  gUsb.end();
  NVIC_SystemReset();
  while (true) { }
}

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

static bool tryUsbLine(const char* line) {
  if (line == nullptr) return false;
  const size_t len = strnlen(line, 190U);
  if (len >= 190U) return false;
  char out[192];
  memcpy(out, line, len);
  out[len] = '\n';
  const size_t total = len + 1U;
  if (gUsb.availableForWrite() < static_cast<int>(total)) return false;
  return gUsb.write(reinterpret_cast<const uint8_t *>(out), total) == total;
}

static void printBoth(const char* line) {
  (void)tryUsbLine(line);
}

static void publishPage() {
  char line[32];
  snprintf(line, sizeof(line), "PAGE:%s", pageName(currentPage));
  printBoth(line);
}

static const char* driveControlName() {
  if (activeDriveControl == CTRL_FORWARD) return "FWD";
  if (activeDriveControl == CTRL_REVERSE) return "REV";
  return "STOP";
}

static const char* steerControlName() {
  if (activeSteerControl == CTRL_LEFT) return "LEFT";
  if (activeSteerControl == CTRL_RIGHT) return "RIGHT";
  if (activeSteerControl == CTRL_CENTER) return "CENTER";
  return "NONE";
}

static void publishCameraTab() {
  char line[32];
  snprintf(line, sizeof(line), "CAMTAB:%s", cameraTabName(currentCameraTab));
  printBoth(line);
}

static void publishLinkState() {
  printBoth(gTelemetry.rosConnected ? "LINK:ROS:ONLINE" : "LINK:ROS:OFFLINE");
}

static void publishControlState() {
  char line[40];
  snprintf(line, sizeof(line), "MODE:%s", gTelemetry.mode == MODE_MANUAL ? "MANUAL" : "AUTO");
  printBoth(line);
  snprintf(line, sizeof(line), "CTRL:DRIVE:%s", driveControlName());
  printBoth(line);
  snprintf(line, sizeof(line), "CTRL:STEER:%s", steerControlName());
  printBoth(line);
  snprintf(line, sizeof(line), "CTRL:SPEED:%u", gTelemetry.manualSpeedPct);
  printBoth(line);
}

static void drawCurrentPage(bool fullDraw = true) {
  switch (currentPage) {
    case PAGE_HOME:
      if (fullDraw) drawHomePage(gTelemetry);
      else updateHomePage(gTelemetry);
      break;
    case PAGE_CAMERA:
      if (fullDraw) drawCameraPage(gTelemetry, currentCameraTab);
      else updateCameraPage(gTelemetry, currentCameraTab);
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
  if (splashComplete && page == currentPage) {
    publishPage();
    return;
  }
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
  resetTouchState();
  uiDirty = false;
  lastUiRefreshMs = 0;
  gTelemetry.systemStatus = SYS_INITIALIZING;
  drawSplashScreen();
  splashStartMs = HAL_GetTick();
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
  targetDeg = std::clamp(targetDeg, STEER_MIN_DEG, STEER_MAX_DEG);
  gTelemetry.steeringTargetDeg = targetDeg;
  gTelemetry.steeringErrorDeg = gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;

  char line[48];
  snprintf(line, sizeof(line), "CMD:STEER:%.1f", targetDeg);
  printBoth(line);
  uiDirty = true;
}

static void setManualSpeed(int value) {
  value = std::clamp(value, MANUAL_SPEED_MIN, MANUAL_SPEED_MAX);
  gTelemetry.manualSpeedPct = (uint8_t)value;
  char line[32];
  snprintf(line, sizeof(line), "CMD:SPEED:%u", gTelemetry.manualSpeedPct);
  printBoth(line);
  uiDirty = true;
}

static void handleWaypointTap(WaypointAction action) {
  if (action == WP_ACTION_NONE) return;
  const uint8_t selected = gTelemetry.selectedWaypoint < HMI_WAYPOINT_COUNT ?
    gTelemetry.selectedWaypoint : 0;

  if (action == WP_PREV || action == WP_NEXT) {
    int next = selected;
    if (action == WP_PREV) next = (next + HMI_WAYPOINT_COUNT - 1) % HMI_WAYPOINT_COUNT;
    else next = (next + 1) % HMI_WAYPOINT_COUNT;
    gTelemetry.selectedWaypoint = static_cast<uint8_t>(next);
    char line[32];
    snprintf(line, sizeof(line), "CMD:WP:SELECT:%u", gTelemetry.selectedWaypoint);
    printBoth(line);
    updateGpsPage(gTelemetry);
    return;
  }

  char line[32];
  if (action == WP_SAVE) {
    if (!waypointSaveEnabled(gTelemetry)) {
      printBoth("ERR:WP_SAVE_GPS_NOT_READY");
      return;
    }
    snprintf(line, sizeof(line), "CMD:WP:SAVE:%u", selected);
    printBoth(line);
  } else if (action == WP_GO) {
    if (!waypointGoEnabled(gTelemetry)) {
      printBoth("ERR:WP_GO_REQUIRES_SAVED_AUTO_READY");
      return;
    }
    snprintf(line, sizeof(line), "CMD:WP:GO:%u", selected);
    printBoth(line);
  } else if (action == WP_STOP) {
    printBoth("CMD:NAV:STOP");
  }
}

// ---------------------------------------------------------------------------
// Touch behavior
// ---------------------------------------------------------------------------
static bool speedAdjustEnabled() {
  return gTelemetry.rosConnected && gTelemetry.systemStatus == SYS_READY && gTelemetry.mode == MODE_MANUAL;
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
    if (ev.modeToggle && currentPage == PAGE_HOME) {
      const bool toManual = gTelemetry.mode != MODE_MANUAL;
      gTelemetry.mode = toManual ? MODE_MANUAL : MODE_AUTO;
      if (!toManual) {
        if (activeDriveControl == CTRL_FORWARD || activeDriveControl == CTRL_REVERSE) sendDriveStop();
        activeDriveControl = CTRL_NONE;
        activeSteerControl = CTRL_NONE;
      }
      printBoth(toManual ? "CMD:MODE:MANUAL" : "CMD:MODE:AUTO");
      updateHomePage(gTelemetry);
      publishControlState();
      return;
    }
    if (ev.home) {
      showPage(PAGE_HOME);
      return;
    }
    if (ev.navPage != PAGE_SPLASH) {
      showPage(ev.navPage);
      return;
    }
    if (currentPage == PAGE_CAMERA && ev.cameraTab != CAM_NONE) {
      if (ev.cameraTab == currentCameraTab) return;
      currentCameraTab = ev.cameraTab;
      // Change only the tab strip and content card; never blank the full screen.
      drawCameraTabs(currentCameraTab);
      drawCameraContent(gTelemetry, currentCameraTab);
      publishCameraTab();
      return;
    }
    if (currentPage == PAGE_GPS && ev.waypointAction != WP_ACTION_NONE) {
      handleWaypointTap(ev.waypointAction);
      return;
    }
    if (currentPage == PAGE_ACTUATOR && ev.control != CTRL_NONE) {
      handleControlTap(ev.control);
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// USB CDC telemetry parser
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

static void sanitizeTelemetry() {
  if (!std::isfinite(gTelemetry.speedKmh)) gTelemetry.speedKmh = 0.0f;
  gTelemetry.speedKmh = std::clamp(gTelemetry.speedKmh, 0.0f, 100.0f);
  if (!std::isfinite(gTelemetry.headingDeg)) gTelemetry.headingDeg = 0.0f;
  gTelemetry.headingDeg = std::fmod(gTelemetry.headingDeg, 360.0f);
  if (gTelemetry.headingDeg < 0.0f) gTelemetry.headingDeg += 360.0f;
  if (!std::isfinite(gTelemetry.latitude) || gTelemetry.latitude < -90.0 || gTelemetry.latitude > 90.0) gTelemetry.latitude = 0.0;
  if (!std::isfinite(gTelemetry.longitude) || gTelemetry.longitude < -180.0 || gTelemetry.longitude > 180.0) gTelemetry.longitude = 0.0;
  if (!std::isfinite(gTelemetry.hdop)) gTelemetry.hdop = 0.0f;
  gTelemetry.hdop = std::clamp(gTelemetry.hdop, 0.0f, 99.9f);
  if (!std::isfinite(gTelemetry.cameraFps)) gTelemetry.cameraFps = 0.0f;
  gTelemetry.cameraFps = std::clamp(gTelemetry.cameraFps, 0.0f, 120.0f);
  if (!std::isfinite(gTelemetry.objectDistanceM) || gTelemetry.objectDistanceM < 0.0f) gTelemetry.objectDistanceM = 0.0f;
  if (!std::isfinite(gTelemetry.confidencePct)) gTelemetry.confidencePct = 0.0f;
  gTelemetry.confidencePct = std::clamp(gTelemetry.confidencePct, 0.0f, 100.0f);
  if (!std::isfinite(gTelemetry.steeringTargetDeg)) gTelemetry.steeringTargetDeg = 0.0f;
  if (!std::isfinite(gTelemetry.steeringActualDeg)) gTelemetry.steeringActualDeg = 0.0f;
  if (!std::isfinite(gTelemetry.steeringErrorDeg)) gTelemetry.steeringErrorDeg = 0.0f;
  if (!std::isfinite(gTelemetry.motorRpm)) gTelemetry.motorRpm = 0.0f;
}

static void markRosHeartbeat() {
  const uint32_t now = HAL_GetTick();
  if (!gTelemetry.rosConnected) {
    rosHeartbeatStableCount = 1;
    rosHeartbeatStable = false;
    gTelemetry.rosConnected = true;
    uiDirty = true;
    publishLinkState();
  } else {
    const uint32_t gap = (uint32_t)(now - lastRosHeartbeatMs);
    if (gap <= ROS_HEARTBEAT_STABLE_GAP_MS) {
      if (rosHeartbeatStableCount < 255) ++rosHeartbeatStableCount;
      if (rosHeartbeatStableCount >= ROS_HEARTBEAT_STABLE_COUNT) {
        rosHeartbeatStable = true;
        if (!gAppWatchdogArmed) {
          gMainLoopHeartbeatMs = HAL_GetTick();
          gAppWatchdogArmed = true;
        }
      }
    } else {
      // A busy ROS startup may delay timers. Restart the qualification window
      // without falsely declaring the link dead.
      rosHeartbeatStableCount = 1;
      rosHeartbeatStable = false;
    }
  }
  lastRosHeartbeatMs = now;
}

static void forceRosOffline() {
  const bool wasConnected = gTelemetry.rosConnected;
  if (activeDriveControl == CTRL_FORWARD || activeDriveControl == CTRL_REVERSE) sendDriveStop();
  activeDriveControl = CTRL_NONE;
  activeSteerControl = CTRL_NONE;
  gTelemetry.rosConnected = false;
  rosHeartbeatStableCount = 0;
  rosHeartbeatStable = false;
  gAppWatchdogArmed = false;
  gTelemetry.systemStatus = SYS_NOT_READY;
  gTelemetry.state = STATE_STOPPED;
  gTelemetry.speedKmh = 0.0f;
  gTelemetry.gpsReady = false;
  gTelemetry.gpsFix = GPS_LOST;
  gTelemetry.imuReady = false;
  gTelemetry.cameraReady = false;
  gTelemetry.perceptionReady = false;
  gTelemetry.cameraFps = 0.0f;
  gTelemetry.drivableAreaClear = false;
  gTelemetry.obstacleDetected = false;
  gTelemetry.escReady = false;
  gTelemetry.encoderReady = false;
  gTelemetry.motorRpm = 0.0f;
  if (gTelemetry.navigationStatus == NAV_QUEUED || gTelemetry.navigationStatus == NAV_NAVIGATING)
    gTelemetry.navigationStatus = NAV_FAILED;
  uiDirty = true;
  if (wasConnected) publishLinkState();
}

static void checkRosLinkTimeout() {
  if (!gTelemetry.rosConnected) return;
  const uint32_t timeoutMs = rosHeartbeatStable ? ROS_LINK_TIMEOUT_MS : ROS_LINK_STARTUP_TIMEOUT_MS;
  if ((uint32_t)(HAL_GetTick() - lastRosHeartbeatMs) > timeoutMs) forceRosOffline();
}

static uint32_t gDfuArmDeadlineMs = 0u;

static void handleSerialCommand(char* command) {
  while (*command == ' ' || *command == '\t') command++;
  if (!*command) return;
  // ROS intentionally repeats a complete state heartbeat. Snapshot the visible
  // telemetry so an identical heartbeat does not trigger any TFT transaction.
  const VehicleTelemetry telemetryBefore = gTelemetry;
  // Hardware gateway namespaces are handled before the HMI command namespace.
  if (!strncmp(command, "VESC:", 5)) {
    (void)gVesc.handleHostCommand(command);
    return;
  }
  if (!strncmp(command, "NEO:", 4)) {
    (void)gNeo3.handleHostCommand(command);
    return;
  }

  // Navigation / health commands
  if (!strcmp(command, "GET:STATE")) {
    publishPage();
    publishLinkState();
    publishControlState();
    publishCameraTab();
    return;
  }
  if (!strcmp(command, "PING")) {
    printBoth("ACK:PONG");
    publishPage();
    publishLinkState();
    publishControlState();
    publishCameraTab();
    return;
  }
  if (!strcmp(command, "BOOT:DFU:ARM")) {
    // Two-step software DFU: a single stale/corrupted CDC line must never reboot
    // the F411 while Nav2 is running. Confirmation is valid for only 2 seconds.
    gDfuArmDeadlineMs = HAL_GetTick() + 2000u;
    printBoth("ACK:DFU:ARMED");
    return;
  }
  if (!strcmp(command, "BOOT:DFU:CONFIRM")) {
    const uint32_t now = HAL_GetTick();
    if (gDfuArmDeadlineMs == 0u || (int32_t)(gDfuArmDeadlineMs - now) <= 0) {
      gDfuArmDeadlineMs = 0u;
      printBoth("ERR:DFU:NOT_ARMED");
      return;
    }
    gDfuArmDeadlineMs = 0u;
    if (activeDriveControl == CTRL_FORWARD || activeDriveControl == CTRL_REVERSE) {
      sendDriveStop();
      activeDriveControl = CTRL_NONE;
    }
    printBoth("ACK:DFU");
    HAL_Delay(80U);
    enterSystemDfu();
    return;
  }
  if (!strcmp(command, "BOOT:DFU")) {
    printBoth("ERR:DFU:TWO_STEP_REQUIRED");
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

  // ROS/Web mirror commands. These change visual/control state without echoing a
  // new actuator CMD back to ROS, preventing SCADA feedback loops.
  if (!strncmp(command, "REMOTE:CAMTAB:", 14)) {
    const char* tab = command + 14;
    CameraSubPage next = CAM_NONE;
    if (eqIgnoreCase(tab, "VIEW")) next = CAM_VIEW;
    else if (eqIgnoreCase(tab, "DETECT")) next = CAM_DETECT;
    else if (eqIgnoreCase(tab, "DRIVE")) next = CAM_DRIVE;
    else if (eqIgnoreCase(tab, "STATUS")) next = CAM_STATUS;
    else return;
    if (next != currentCameraTab) {
      currentCameraTab = next;
      if (currentPage == PAGE_CAMERA) {
        drawCameraTabs(currentCameraTab);
        drawCameraContent(gTelemetry, currentCameraTab);
      }
    }
    publishCameraTab();
    return;
  }
  if (!strncmp(command, "REMOTE:DRIVE:", 13)) {
    const char* action = command + 13;
    if (eqIgnoreCase(action, "STOP")) {
      activeDriveControl = CTRL_NONE;
      gTelemetry.state = STATE_STOPPED;
    } else if (gTelemetry.mode == MODE_MANUAL && gTelemetry.escReady &&
               (eqIgnoreCase(action, "FWD") || eqIgnoreCase(action, "REV"))) {
      activeDriveControl = eqIgnoreCase(action, "FWD") ? CTRL_FORWARD : CTRL_REVERSE;
      gTelemetry.state = STATE_RUNNING;
    } else {
      printBoth("ERR:REMOTE_DRIVE_LOCKED");
      return;
    }
    if (currentPage == PAGE_ACTUATOR) updateActuatorPage(gTelemetry, activeDriveControl, activeSteerControl);
    publishControlState();
    return;
  }
  if (!strncmp(command, "REMOTE:STEER:", 13)) {
    const char* action = command + 13;
    if (!(gTelemetry.mode == MODE_MANUAL && gTelemetry.escReady && gTelemetry.encoderReady)) {
      printBoth("ERR:REMOTE_STEER_LOCKED");
      return;
    }
    if (eqIgnoreCase(action, "LEFT")) activeSteerControl = CTRL_LEFT;
    else if (eqIgnoreCase(action, "RIGHT")) activeSteerControl = CTRL_RIGHT;
    else if (eqIgnoreCase(action, "CENTER")) activeSteerControl = CTRL_CENTER;
    else return;
    if (currentPage == PAGE_ACTUATOR) updateActuatorPage(gTelemetry, activeDriveControl, activeSteerControl);
    publishControlState();
    return;
  }
  if (!strncmp(command, "REMOTE:SPEED:", 13)) {
    gTelemetry.manualSpeedPct = (uint8_t)std::clamp(atoi(command + 13), MANUAL_SPEED_MIN, MANUAL_SPEED_MAX);
    if (currentPage == PAGE_ACTUATOR) updateActuatorPage(gTelemetry, activeDriveControl, activeSteerControl);
    publishControlState();
    return;
  }

  if (!strncmp(command, "ROS:", 4)) {
    if (parseBool(command + 4)) markRosHeartbeat();
    else forceRosOffline();
  } else if (!strncmp(command, "FPS:", 4)) {
    gTelemetry.cameraFps = std::max(0.0f, static_cast<float>(atof(command + 4)));
  } else if (!strncmp(command, "WPSEL:", 6)) {
    gTelemetry.selectedWaypoint = (uint8_t)std::clamp(atoi(command + 6), 0, static_cast<int>(HMI_WAYPOINT_COUNT) - 1);
  } else if (!strncmp(command, "TARGET:", 7)) {
    snprintf(gTelemetry.activeTarget, sizeof(gTelemetry.activeTarget), "%s", command + 7);
  } else if (!strncmp(command, "NAV:", 4)) {
    const char* state = command + 4;
    if (eqIgnoreCase(state, "SELECTED")) gTelemetry.navigationStatus = NAV_SELECTED;
    else if (eqIgnoreCase(state, "QUEUED") || eqIgnoreCase(state, "SENDING")) gTelemetry.navigationStatus = NAV_QUEUED;
    else if (eqIgnoreCase(state, "NAVIGATING") || eqIgnoreCase(state, "ACTIVE")) gTelemetry.navigationStatus = NAV_NAVIGATING;
    else if (eqIgnoreCase(state, "ARRIVED") || eqIgnoreCase(state, "SUCCEEDED")) gTelemetry.navigationStatus = NAV_ARRIVED;
    else if (eqIgnoreCase(state, "STOPPED") || eqIgnoreCase(state, "CANCELED")) gTelemetry.navigationStatus = NAV_STOPPED;
    else if (eqIgnoreCase(state, "FAILED") || eqIgnoreCase(state, "ABORTED") || eqIgnoreCase(state, "REJECTED")) gTelemetry.navigationStatus = NAV_FAILED;
    else gTelemetry.navigationStatus = NAV_IDLE;
  } else if (!strncmp(command, "WP", 2) && command[2] >= '0' && command[2] <= '3' && command[3] == ':') {
    const uint8_t index = (uint8_t)(command[2] - '0');
    char* payload = command + 4;
    char* colon = strchr(payload, ':');
    if (colon != nullptr) {
      *colon = '\0';
      gTelemetry.waypointSaved[index] = parseBool(payload);
      snprintf(gTelemetry.waypointName[index], HMI_WAYPOINT_NAME_LEN, "%s", colon + 1);
    }
  } else if (!strncmp(command, "SYS:", 4)) {
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
    gTelemetry.satellites = (uint8_t)std::clamp(atoi(command + 4), 0, 99);
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
    gTelemetry.steeringTargetDeg = std::clamp(static_cast<float>(atof(command + 13)), STEER_MIN_DEG, STEER_MAX_DEG);
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
    gTelemetry.manualSpeedPct = (uint8_t)std::clamp(atoi(command + 13), MANUAL_SPEED_MIN, MANUAL_SPEED_MAX);
  } else {
    char line[224];
    std::snprintf(line, sizeof(line), "ERR:UNKNOWN_COMMAND:%s", command);
    (void)gUsb.writeLine(line);
    return;
  }

  sanitizeTelemetry();
  if (std::memcmp(&telemetryBefore, &gTelemetry, sizeof(VehicleTelemetry)) != 0) {
    uiDirty = true;
  }
}

static void pollSerialGui() {
  gUsb.poll();
  while (gUsb.available() > 0) {
    const int value = gUsb.read();
    if (value < 0) break;
    const char c = static_cast<char>(value);
    if (c == '\r') continue;
    if (serialRxDiscarding) {
      if (c == '\n') serialRxDiscarding = false;
      continue;
    }
    if (c == '\n') {
      serialRx[serialRxLen] = '\0';
      if (serialRxLen > 0U) handleSerialCommand(serialRx);
      serialRxLen = 0U;
    } else if (serialRxLen + 1U < sizeof(serialRx)) {
      serialRx[serialRxLen++] = c;
    } else {
      serialRxLen = 0U;
      serialRxDiscarding = true;
      (void)gUsb.writeLine("ERR:COMMAND_TOO_LONG");
    }
  }
}

// ---------------------------------------------------------------------------
// Display / splash
// ---------------------------------------------------------------------------
static void initDisplay() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  tft.init();
  tft.setRotation(1);
  // RGB565 illustration arrays use standard byte order.
  tft.setSwapBytes(true);
  tft.fillScreen(C_BG);
  tft.setTextDatum(MC_DATUM);
}

static bool updateProgressBar() {
  if (splashComplete) return false;

  uint32_t now = HAL_GetTick();
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
    drawUiText("HMI ready", W / 2, 187, C_READY, C_BG, MC_DATUM);
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
  if (HAL_GetTick() - demoMs < 700) return;
  demoMs = HAL_GetTick();

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
  gTelemetry.cameraFps = 8.0f;
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

int main() {
  Board_Init();
  if (!gUsb.begin()) { NVIC_SystemReset(); }
  HAL_Delay(50U);
  printBoth("ADV HMI + CUAV NEO3 integrated firmware - boot");

  gNeo3.begin();
  gVesc.begin();
  initDisplay();
  restartSplash();
  startAppWatchdog();

  while (true) {

  // USB host commands are serviced first. In VESC MAINTENANCE the gateway owns
  // the USB/UART bandwidth exclusively so firmware blocks cannot be dropped by
  // GNSS/I2C/TFT traffic. Power-stage outputs remain under the F103 bootloader's
  // safe-OFF policy during an update.
  pollSerialGui();
  gVesc.poll();

  // Maintenance is entered only after the ROS arbiter verifies the vehicle is idle.
  // Give the VESC request/reply path exclusive loop priority while a maintenance
  // owner is active. This prevents TFT, GNSS/I2C and HMI telemetry from adding
  // millisecond-scale jitter to VESC Tool RT data. Runtime mode is unchanged.
  if (gVesc.maintenanceMode()) {
    for (uint8_t i = 0; i < 4U; ++i) {
      pollSerialGui();
      gVesc.poll();
    }
    gUsb.poll();
    Board_Service();
    gMainLoopHeartbeatMs = HAL_GetTick();
    continue;
  }

  // VESC traffic has first service priority, but maintenance must not suspend
  // unrelated F411 functions. GNSS, IST8310, safety I/O, touch/HMI and actuator
  // state continue to run while VESC Tool owns the motor link. This mirrors the
  // official VESC design where UART packet processing is a communication task,
  // not a global mode that blocks other application tasks.
  gNeo3.poll();
  // Cooperative equivalent of the dedicated UART thread in upstream VESC:
  // immediately drain any F103 reply that arrived while sensor work ran.
  gVesc.poll();
  pollSerialGui();
  checkRosLinkTimeout();

  if (!splashComplete) {
    updateProgressBar();
  } else {
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - lastTouchPollMs) >= TOUCH_POLL_MS) {
      lastTouchPollMs = now;
      handleTouch();
    }

#if HMI_DEMO_MODE
    updateDemoTelemetry();
#endif

    // Full-rate telemetry never means full-rate painting. Each page also caches
    // its formatted values, so only changed pixels are touched at this cadence.
    if (uiDirty && !touchWasDown && (uint32_t)(now - lastUiRefreshMs) >= DISPLAY_REFRESH_MS) {
      lastUiRefreshMs = now;
      uiDirty = false;
      drawCurrentPage(false);
    }
  }

  // Heartbeat LED (PC13 active-low on many Black Pill boards)
  static uint32_t ledMs = 0;
  if (HAL_GetTick() - ledMs >= 500) {
    ledMs = HAL_GetTick();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  gUsb.poll();
  Board_Service();
  // Feed only after one complete main-loop iteration. A blocking USB/TFT/I2C
  // call therefore cannot keep the watchdog alive accidentally.
  gMainLoopHeartbeatMs = HAL_GetTick();
  }
}
