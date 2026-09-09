// ============================================================================
// UiShell.h — renderer tunggal untuk OVERVIEW / ESC / PERCEPTION / NAVIGATION.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "Config.h"
#include "Telemetry.h"
#include "Theme.h"
#include "Icons.h"
#include "UiMenu.h"

extern TFT_eSPI tft;

inline void drawDomainDot(int x, const char* label, bool ok) {
  drawStatusDot(x, 14, healthColor(ok), 3);
  drawMicroText(label, x + 7, 10, C_TEXT_DIM, C_BG);
}

inline void drawUiTopBar(const UiState& ui, const VehicleTelemetry& d) {
  tft.fillRect(0, 0, W, TOP_H, C_BG);
  iconHome(14, 14, C_TEXT);
  tft.drawFastVLine(28, 6, 17, C_BORDER);
  char title[28];
  snprintf(title, sizeof(title), "%.24s", menuTitle(ui.menu));
  drawCompactText(title, 36, 7, C_TEXT, C_BG);
  drawDomainDot(205, "E", d.escReady && d.vescConnected);
  drawDomainDot(244, "P", d.perceptionReady);
  drawDomainDot(283, "N", d.nav2Ready && d.motionReady);
  if (d.eStop) {
    tft.fillRoundRect(126, 5, 67, 20, 4, C_FAULT);
    drawMicroText("E-STOP", 159, 10, C_WHITE, C_FAULT, TC_DATUM);
  }
}

inline int softKeyX(uint8_t index) { return SOFTKEY_X0 + index * (SOFTKEY_W + SOFTKEY_GAP); }

inline void drawSoftKey(uint8_t index, SoftKey key, const char* label, bool active = true) {
  const int x = softKeyX(index);
  const uint16_t border = key == SoftKey::OK && active ? C_ACCENT : C_BORDER;
  const uint16_t fg = active ? (key == SoftKey::OK ? C_ACCENT : C_TEXT) : C_DISABLED;
  tft.fillRoundRect(x, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H, 6, C_PANEL);
  tft.drawRoundRect(x, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H, 6, border);
  const int cx = x + SOFTKEY_W / 2;
  if (key == SoftKey::LEFT) iconArrowLeft(cx, SOFTKEY_Y + 15, fg);
  else if (key == SoftKey::RIGHT) iconArrowRight(cx, SOFTKEY_Y + 15, fg);
  else if (key == SoftKey::OK) tft.drawCircle(cx, SOFTKEY_Y + 15, 7, fg);
  drawCompactText(label, cx, SOFTKEY_Y + 30, fg, C_PANEL, TC_DATUM);
}

inline bool uiNeedsRail(const UiState& ui) {
  return menuHasChildren(ui.menu) || menuEditKey(ui.menu) != UiEditKey::NONE ||
         ui.menu == UiMenuId::ESC_STEERING_TEST || ui.menu == UiMenuId::NAV_MISSION;
}

inline void drawRightRail(const UiState& ui) {
  if (!uiNeedsRail(ui)) return;
  tft.fillRoundRect(RIGHT_RAIL_X, RIGHT_UP_Y, RIGHT_RAIL_W, RIGHT_KEY_H, 6, C_PANEL_ALT);
  tft.drawRoundRect(RIGHT_RAIL_X, RIGHT_UP_Y, RIGHT_RAIL_W, RIGHT_KEY_H, 6, C_BORDER);
  iconArrowUp(RIGHT_RAIL_X + RIGHT_RAIL_W / 2, RIGHT_UP_Y + 23, C_TEXT);
  drawMicroText("UP", RIGHT_RAIL_X + RIGHT_RAIL_W / 2, RIGHT_UP_Y + 42, C_TEXT_DIM, C_PANEL_ALT, TC_DATUM);

  tft.fillRoundRect(RIGHT_RAIL_X, RIGHT_DOWN_Y, RIGHT_RAIL_W, RIGHT_KEY_H, 6, C_PANEL_ALT);
  tft.drawRoundRect(RIGHT_RAIL_X, RIGHT_DOWN_Y, RIGHT_RAIL_W, RIGHT_KEY_H, 6, C_BORDER);
  iconArrowDown(RIGHT_RAIL_X + RIGHT_RAIL_W / 2, RIGHT_DOWN_Y + 23, C_TEXT);
  drawMicroText("DOWN", RIGHT_RAIL_X + RIGHT_RAIL_W / 2, RIGHT_DOWN_Y + 42, C_TEXT_DIM, C_PANEL_ALT, TC_DATUM);
}

inline void drawUiFooter(const UiState& ui, const VehicleTelemetry& d) {
  (void)d;
  drawSoftKey(0, SoftKey::LEFT, ui.editing ? "CANCEL" : "LEFT");
  drawSoftKey(1, SoftKey::RIGHT, "RIGHT");
  drawSoftKey(2, SoftKey::OK, ui.editing ? "APPLY" : "OK", !d.configPending);
  drawRightRail(ui);
}

inline void drawMetricRow(int y, const char* label, const char* value, uint16_t valueColor = C_INK) {
  drawSmallText(label, 18, y, C_DISABLED, C_CARD);
  drawUiText(value, 252, y - 2, valueColor, C_CARD, TR_DATUM);
  drawThinDivider(16, y + 20, 238, C_CARD_LINE);
}

inline void drawMetricFloat(int y, const char* label, float value, const char* unit, int precision = 1,
                            uint16_t valueColor = C_INK) {
  char text[30];
  snprintf(text, sizeof(text), precision == 2 ? "%.2f %s" : "%.1f %s", value, unit);
  drawMetricRow(y, label, text, valueColor);
}

inline void drawContentCard(bool rail) {
  const int width = rail ? 258 : 308;
  drawCard(6, CONTENT_Y, width, CONTENT_BOTTOM - CONTENT_Y + 1, C_CARD, C_CARD_LINE, false);
}

inline void drawOverviewCard(int x, int y, int w, int h, const char* title,
                             const char* line1, const char* line2, bool ready, bool selected) {
  const uint16_t border = selected ? C_ACCENT : C_CARD_LINE;
  drawCard(x, y, w, h, C_CARD, border, false);
  drawCompactText(title, x + 9, y + 8, selected ? C_ACCENT : C_INK, C_CARD);
  drawStatusDot(x + w - 13, y + 14, healthColor(ready), 3);
  drawMicroText(line1, x + 9, y + 31, C_INK, C_CARD);
  drawMicroText(line2, x + 9, y + 47, C_DISABLED, C_CARD);
}

inline void drawOverview(const UiState& ui, const VehicleTelemetry& d) {
  char esc1[28], esc2[28], per1[28], per2[28], nav1[28], nav2[28];
  snprintf(esc1, sizeof(esc1), "Steer %+.1f deg", d.steeringActualDeg);
  snprintf(esc2, sizeof(esc2), "Drive %.2f m/s", d.driveActualMps);
  snprintf(per1, sizeof(per1), "%.1f FPS  %s", d.cameraFps, d.detectedObject);
  snprintf(per2, sizeof(per2), "%s %.1fm", d.obstacleDetected ? "OBS" : "CLEAR", d.objectDistanceM);
  snprintf(nav1, sizeof(nav1), "%s SAT %u", gpsFixText(d.gpsFix), d.satellites);
  snprintf(nav2, sizeof(nav2), "Heading %.0f deg", d.headingDeg);

  drawOverviewCard(6, 38, 100, 73, "ESC", esc1, esc2,
                   d.escReady && d.vescConnected, ui.selectedChild == 0);
  drawOverviewCard(110, 38, 100, 73, "PERCEPTION", per1, per2,
                   d.perceptionReady, ui.selectedChild == 1);
  drawOverviewCard(214, 38, 100, 73, "NAV", nav1, nav2,
                   d.motionReady && d.nav2Ready, ui.selectedChild == 2);

  drawCard(6, 116, 308, 68, C_PANEL, C_BORDER, false);
  char state[60];
  snprintf(state, sizeof(state), "%s | %s | %.1f km/h",
           modeText(d.mode), systemStatusText(d.systemStatus), d.speedKmh);
  drawUiText(state, 16, 126, d.eStop ? C_FAULT : C_TEXT, C_PANEL);
  const char* target = navigationHasTarget(d) ? d.activeTarget : "NO TARGET";
  char mission[70];
  snprintf(mission, sizeof(mission), "%s | %.20s", navigationStatusText(d.navigationStatus), target);
  drawSmallText(mission, 16, 151, C_TEXT_DIM, C_PANEL);
  drawMicroText(d.configPending ? "CONFIG SYNC: PENDING" :
                (d.configLastOk ? "CONFIG SYNC: OK" : "CONFIG SYNC: ERROR"),
                16, 170, d.configPending ? C_WARNING : (d.configLastOk ? C_READY : C_FAULT), C_PANEL);
}

inline void drawMenuList(const UiState& ui) {
  uint8_t count = 0;
  const UiMenuId* children = menuChildren(ui.menu, count);
  drawContentCard(true);
  drawMicroText("SELECT SUBMENU", 16, 43, C_DISABLED, C_CARD);
  if (children == nullptr || count == 0) return;

  const uint8_t visible = 4;
  uint8_t first = 0;
  if (ui.selectedChild >= visible) first = static_cast<uint8_t>(ui.selectedChild - visible + 1);
  if (first + visible > count && count > visible) first = count - visible;
  for (uint8_t row = 0; row < visible && first + row < count; ++row) {
    const uint8_t index = first + row;
    const int y = 59 + row * 29;
    const bool selected = index == ui.selectedChild;
    const uint16_t fill = selected ? C_PANEL_ALT : C_CARD;
    if (selected) tft.fillRoundRect(12, y - 3, 246, 26, 5, fill);
    drawCompactText(selected ? ">" : " ", 18, y + 2, selected ? C_ACCENT : C_DISABLED, fill);
    drawCompactText(menuTitle(children[index]), 36, y + 2, selected ? C_ACCENT : C_INK, fill);
  }
  char pos[18];
  snprintf(pos, sizeof(pos), "%u / %u", static_cast<unsigned>(ui.selectedChild + 1), static_cast<unsigned>(count));
  drawMicroText(pos, 247, 171, C_DISABLED, C_CARD, TR_DATUM);
}

inline void drawEditStatus(const VehicleTelemetry& d) {
  if (d.configPending) {
    drawMicroText("WAITING ROS ACK / READBACK", 18, 164, C_WARNING, C_CARD);
  } else if (!d.configLastOk) {
    drawMicroText(d.configMessage, 18, 164, C_FAULT, C_CARD);
  } else {
    drawMicroText("SYNCED WITH ROS", 18, 164, C_READY, C_CARD);
  }
}

inline void drawEditor(const UiState& ui, const VehicleTelemetry& d, UiEditKey key) {
  drawContentCard(true);
  drawMicroText(ui.editing ? "EDIT MODE" : "CURRENT VALUE", 18, 46,
                ui.editing ? C_ACCENT : C_DISABLED, C_CARD);
  char value[32];
  const float shown = ui.editing ? ui.editValue :
    (key == UiEditKey::OPERATOR_MODE ? (d.mode == MODE_MANUAL ? 1.0F : 0.0F) :
     (key == UiEditKey::MANUAL_SPEED_PCT ? static_cast<float>(d.manualSpeedPct) :
      (key == UiEditKey::DRIVE_SCALE ? d.driveScale : (d.perceptionInference ? 1.0F : 0.0F))));
  if (key == UiEditKey::MANUAL_SPEED_PCT) snprintf(value, sizeof(value), "%.0f %%", shown);
  else if (key == UiEditKey::DRIVE_SCALE) snprintf(value, sizeof(value), "%.3f", shown);
  else if (key == UiEditKey::OPERATOR_MODE) snprintf(value, sizeof(value), "%s", shown > 0.5F ? "MANUAL" : "AUTO");
  else snprintf(value, sizeof(value), "%s", shown > 0.5F ? "ON" : "OFF");
  drawHeroText(value, 18, 70, C_INK, C_CARD);
  drawSmallText("UP / DOWN mengubah nilai", 18, 126, C_DISABLED, C_CARD);
  drawSmallText(ui.editing ? "OK = APPLY, LEFT = CANCEL" : "OK = EDIT",
                18, 145, C_DISABLED, C_CARD);
  drawEditStatus(d);
}

inline void drawEscLeaf(UiMenuId id, const UiState& ui, const VehicleTelemetry& d) {
  (void)ui;
  drawContentCard(uiNeedsRail(ui));
  char a[30], b[30];
  if (id == UiMenuId::ESC_OVERVIEW) {
    snprintf(a, sizeof(a), "%s", d.escReady ? "READY" : "NOT READY");
    drawMetricRow(48, "ESC state", a, healthColor(d.escReady));
    drawMetricFloat(77, "Steering", d.steeringActualDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(106, "Drive", d.driveActualMps, "m/s", 2, C_ACCENT);
    drawMetricRow(135, "F103 link", d.vescConnected ? "ONLINE" : "OFFLINE", healthColor(d.vescConnected));
    return;
  }
  if (id == UiMenuId::ESC_STEERING_LIVE || id == UiMenuId::ESC_STEERING_CAL) {
    drawMetricFloat(48, "Target", d.steeringTargetDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(77, "Actual", d.steeringActualDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(106, "Error", d.steeringErrorDeg, "deg", 1,
                    fabsf(d.steeringErrorDeg) < 2.0F ? C_READY : C_WARNING);
    drawMetricRow(135, "Encoder", d.encoderReady ? "READY" : "OFFLINE", healthColor(d.encoderReady));
    return;
  }
  if (id == UiMenuId::ESC_STEERING_TEST) {
    snprintf(a, sizeof(a), "%+.1f deg", ui.steeringTestTargetDeg);
    drawMetricRow(48, "Test target", a, C_ACCENT);
    drawMetricFloat(77, "Actual", d.steeringActualDeg, "deg", 1, C_INK);
    drawMetricRow(106, "Test state", d.steeringTestState,
                  strcmp(d.steeringTestState, "ACTIVE") == 0 ? C_WARNING :
                  (strcmp(d.steeringTestState, "LOCKED") == 0 ? C_FAULT : C_READY));
    drawMetricRow(135, "Safety", (!d.eStop && d.escReady && d.encoderReady) ? "READY" : "LOCKED",
                  (!d.eStop && d.escReady && d.encoderReady) ? C_READY : C_FAULT);
    drawMicroText("UP/DOWN target, OK kirim posisi", 18, 166, C_DISABLED, C_CARD);
    return;
  }
  if (id == UiMenuId::ESC_DRIVE_LIVE || id == UiMenuId::ESC_DRIVE_TEST) {
    drawMetricFloat(48, "Target", d.driveTargetMps, "m/s", 2, C_ACCENT);
    drawMetricFloat(77, "Actual", d.driveActualMps, "m/s", 2, C_ACCENT);
    snprintf(a, sizeof(a), "%.0f", d.motorErpm);
    drawMetricRow(106, "Electrical RPM", a, C_INK);
    snprintf(b, sizeof(b), "%.0f", d.motorRpm);
    drawMetricRow(135, "Mechanical RPM", b, C_INK);
    return;
  }
  if (id == UiMenuId::ESC_POWER) {
    drawMetricRow(48, "Power telemetry", d.vescConnected ? "STREAMING" : "NO LINK", healthColor(d.vescConnected));
    drawMetricRow(77, "Control", "FOC 16 kHz", C_INK);
    drawMetricRow(106, "Watchdog", "300 ms", C_READY);
    drawMetricRow(135, "Fault gate", d.systemStatus == SYS_FAULT ? "FAULT" : "CLEAR",
                  d.systemStatus == SYS_FAULT ? C_FAULT : C_READY);
    return;
  }
  if (id == UiMenuId::ESC_LINK) {
    drawMetricRow(48, "ROS <-> F411", d.rosConnected ? "ONLINE" : "OFFLINE", healthColor(d.rosConnected));
    drawMetricRow(77, "F411 <-> F103", d.vescConnected ? "ONLINE" : "OFFLINE", healthColor(d.vescConnected));
    drawMetricRow(106, "USB CDC", "1,000,000", C_INK);
    drawMetricRow(135, "F103 UART", "115,200", C_INK);
    return;
  }
  drawMetricRow(48, "ESC", d.escReady ? "PASS" : "FAIL", healthColor(d.escReady));
  drawMetricRow(77, "Encoder", d.encoderReady ? "PASS" : "FAIL", healthColor(d.encoderReady));
  drawMetricRow(106, "E-stop", d.eStop ? "FAIL" : "PASS", d.eStop ? C_FAULT : C_READY);
  drawMetricRow(135, "Vehicle", d.state == STATE_STOPPED ? "STOPPED" : "MOVING",
                d.state == STATE_STOPPED ? C_READY : C_WARNING);
}

inline void drawPerceptionLeaf(UiMenuId id, const UiState& ui, const VehicleTelemetry& d) {
  drawContentCard(uiNeedsRail(ui));
  char text[32];
  if (id == UiMenuId::PERCEPTION_OVERVIEW || id == UiMenuId::PERCEPTION_CAMERA) {
    drawMetricRow(48, "Camera", d.cameraReady ? "READY" : "OFFLINE", healthColor(d.cameraReady));
    drawMetricRow(77, "Perception", d.perceptionReady ? "HEALTHY" : "WAIT", healthColor(d.perceptionReady));
    snprintf(text, sizeof(text), "%.1f FPS", d.cameraFps);
    drawMetricRow(106, "Pipeline", text, d.cameraFps > 1.0F ? C_READY : C_WARNING);
    drawMetricRow(135, "Inference", d.perceptionInference ? "ON" : "OFF",
                  d.perceptionInference ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::PERCEPTION_DETECTION_LIVE) {
    drawMetricRow(48, "Object", d.detectedObject, C_ACCENT);
    drawMetricFloat(77, "Distance", d.objectDistanceM, "m", 2, C_INK);
    snprintf(text, sizeof(text), "%.0f %%", d.confidencePct);
    drawMetricRow(106, "Confidence", text, C_INK);
    drawMetricRow(135, "Obstacle", d.obstacleDetected ? "DETECTED" : "CLEAR",
                  d.obstacleDetected ? C_FAULT : C_READY);
    return;
  }
  if (id == UiMenuId::PERCEPTION_LANE) {
    drawMetricRow(48, "Lane state", d.laneState,
                  strcmp(d.laneState, "CLEAR") == 0 ? C_READY : C_WARNING);
    drawMetricRow(77, "Drivable", d.drivableAreaClear ? "CLEAR" : "BLOCKED",
                  d.drivableAreaClear ? C_READY : C_FAULT);
    drawMetricRow(106, "Camera metric", d.perceptionReady ? "ACTIVE" : "WAIT",
                  d.perceptionReady ? C_READY : C_WARNING);
    drawMetricRow(135, "Safety action", d.eStop ? "STOP" : "MONITOR", d.eStop ? C_FAULT : C_INK);
    return;
  }
  if (id == UiMenuId::PERCEPTION_OBSTACLE) {
    drawMetricRow(48, "Nearest", d.detectedObject, C_ACCENT);
    drawMetricFloat(77, "Forward", d.objectDistanceM, "m", 2, C_INK);
    drawMetricRow(106, "Path", d.drivableAreaClear ? "CLEAR" : "BLOCKED",
                  d.drivableAreaClear ? C_READY : C_FAULT);
    drawMetricRow(135, "Emergency", d.obstacleDetected ? "CHECK" : "CLEAR",
                  d.obstacleDetected ? C_WARNING : C_READY);
    return;
  }
  if (id == UiMenuId::PERCEPTION_PERFORMANCE) {
    snprintf(text, sizeof(text), "%.1f FPS", d.cameraFps);
    drawMetricRow(48, "Inference rate", text, d.cameraFps > 1.0F ? C_READY : C_WARNING);
    drawMetricRow(77, "Camera link", d.cameraReady ? "ONLINE" : "OFFLINE", healthColor(d.cameraReady));
    drawMetricRow(106, "Health", d.perceptionReady ? "GOOD" : "BAD", healthColor(d.perceptionReady));
    drawMetricRow(135, "ROS stream", d.rosConnected ? "LIVE" : "OFFLINE", healthColor(d.rosConnected));
    return;
  }
  drawMetricRow(48, "Camera", d.cameraReady ? "PASS" : "FAIL", healthColor(d.cameraReady));
  drawMetricRow(77, "Inference", d.perceptionInference ? "PASS" : "OFF", d.perceptionInference ? C_READY : C_WARNING);
  drawMetricRow(106, "Drivable", d.drivableAreaClear ? "PASS" : "CHECK",
                d.drivableAreaClear ? C_READY : C_WARNING);
  drawMetricRow(135, "Obstacle gate", d.eStop ? "STOP" : "PASS", d.eStop ? C_FAULT : C_READY);
}

inline void drawNavigationLeaf(UiMenuId id, const UiState& ui, const VehicleTelemetry& d) {
  drawContentCard(uiNeedsRail(ui));
  char text[40];
  if (id == UiMenuId::NAVIGATION_OVERVIEW) {
    drawMetricRow(48, "Localization", d.localizationState,
                  d.motionReady ? C_READY : C_WARNING);
    drawMetricRow(77, "Nav2", d.nav2Ready ? "READY" : "WAIT", healthColor(d.nav2Ready));
    drawMetricRow(106, "Motion gate", d.motionReady ? "OPEN" : "LOCKED", d.motionReady ? C_READY : C_FAULT);
    drawMetricRow(135, "Mission", navigationStatusText(d.navigationStatus), C_ACCENT);
    return;
  }
  if (id == UiMenuId::NAV_GNSS) {
    snprintf(text, sizeof(text), "%s / %u SAT", gpsFixText(d.gpsFix), d.satellites);
    drawMetricRow(48, "Fix", text, gpsFixColor(d.gpsFix));
    drawMetricFloat(77, "hAcc", d.haccM, "m", 2, d.haccM < 2.5F ? C_READY : C_WARNING);
    drawMetricFloat(106, "Age", d.gnssAgeSec, "s", 2, d.gnssAgeSec < 0.5F ? C_READY : C_WARNING);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(135, "Heading", text, C_INK);
    return;
  }
  if (id == UiMenuId::NAV_IMU) {
    drawMetricRow(48, "IMU", d.imuStatus, healthColor(d.imuReady));
    drawMetricFloat(77, "Gyro Z", d.gyroZRps, "rad/s", 2, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(106, "Heading", text, C_INK);
    drawMetricRow(135, "Fusion", d.motionReady ? "ACTIVE" : "WAIT", d.motionReady ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::NAV_MAG) {
    drawMetricRow(48, "IST8310", d.magReady ? "READY" : "OFFLINE", healthColor(d.magReady));
    drawMetricRow(77, "Heading source", d.gnssStatus, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(106, "Vehicle yaw", text, C_ACCENT);
    drawMetricRow(135, "Calibration", d.magReady ? "AVAILABLE" : "REQUIRED",
                  d.magReady ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::NAV_EKF_LOCAL || id == UiMenuId::NAV_EKF_GLOBAL) {
    const char* status = id == UiMenuId::NAV_EKF_LOCAL ? d.ekfLocalStatus : d.ekfGlobalStatus;
    drawMetricRow(48, id == UiMenuId::NAV_EKF_LOCAL ? "EKF local" : "EKF global", status,
                  strstr(status, "READY") != nullptr ? C_READY : C_WARNING);
    drawMetricRow(77, "Localization", d.localizationState, d.motionReady ? C_READY : C_WARNING);
    drawMetricFloat(106, "Vehicle speed", d.driveActualMps, "m/s", 2, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(135, "Yaw", text, C_ACCENT);
    return;
  }
  if (id == UiMenuId::NAV_MISSION_GO || id == UiMenuId::NAV_MISSION_SAVE ||
      id == UiMenuId::NAV_MISSION_STOP) {
    const uint8_t index = d.selectedWaypoint < HMI_WAYPOINT_COUNT ? d.selectedWaypoint : 0;
    drawMetricRow(48, "Waypoint", d.waypointName[index], d.waypointSaved[index] ? C_READY : C_WARNING);
    drawMetricRow(77, "Saved", d.waypointSaved[index] ? "YES" : "NO", d.waypointSaved[index] ? C_READY : C_WARNING);
    drawMetricRow(106, "Navigation", navigationStatusText(d.navigationStatus), C_ACCENT);
    if (id == UiMenuId::NAV_MISSION_GO) {
      drawMetricRow(135, "Action", d.mode == MODE_AUTO ? "GO ON OK" : "AUTO REQUIRED",
                    d.mode == MODE_AUTO ? C_READY : C_FAULT);
      drawMicroText("UP/DOWN pilih waypoint", 18, 166, C_DISABLED, C_CARD);
    } else if (id == UiMenuId::NAV_MISSION_SAVE) {
      drawMetricRow(135, "Action", d.gpsReady && d.state == STATE_STOPPED ? "SAVE ON OK" : "GPS/STOP REQUIRED",
                    d.gpsReady && d.state == STATE_STOPPED ? C_READY : C_FAULT);
      drawMicroText("Simpan pose map + GNSS saat ini", 18, 166, C_DISABLED, C_CARD);
    } else {
      drawMetricRow(135, "Action", "STOP ON OK", C_FAULT);
      drawMicroText("STOP selalu boleh dipanggil", 18, 166, C_DISABLED, C_CARD);
    }
    return;
  }
  if (id == UiMenuId::NAV_PLANNER || id == UiMenuId::NAV_MPPI ||
      id == UiMenuId::NAV_SMOOTHER || id == UiMenuId::NAV_COSTMAP) {
    // Jangan tampilkan parameter Nav2 hardcoded: seluruh angka harus berasal dari runtime.
    drawMetricRow(48, "Nav2 state", d.nav2Ready ? "READY" : "WAIT", healthColor(d.nav2Ready));
    drawMetricRow(77, "Localization", d.localizationState, d.motionReady ? C_READY : C_WARNING);
    if (id == UiMenuId::NAV_COSTMAP) {
      drawMetricRow(106, "Obstacle", d.obstacleDetected ? "ACTIVE" : "CLEAR",
                    d.obstacleDetected ? C_WARNING : C_READY);
      drawMetricRow(135, "Lane", d.laneState, strcmp(d.laneState, "CLEAR") == 0 ? C_READY : C_WARNING);
    } else {
      drawMetricFloat(106, "Cmd speed", d.driveTargetMps, "m/s", 2, C_ACCENT);
      drawMetricFloat(135, "Actual speed", d.driveActualMps, "m/s", 2, C_INK);
    }
    return;
  }
  if (id == UiMenuId::NAV_SAFETY) {
    drawMetricRow(48, "E-stop", d.eStop ? "ACTIVE" : "CLEAR", d.eStop ? C_FAULT : C_READY);
    drawMetricRow(77, "Localization", d.motionReady ? "PASS" : "LOCKED", d.motionReady ? C_READY : C_FAULT);
    drawMetricRow(106, "Nav2", d.nav2Ready ? "PASS" : "WAIT", healthColor(d.nav2Ready));
    drawMetricRow(135, "Autonomy", (!d.eStop && d.motionReady && d.nav2Ready) ? "ALLOWED" : "LOCKED",
                  (!d.eStop && d.motionReady && d.nav2Ready) ? C_READY : C_FAULT);
    return;
  }
  drawMetricRow(48, "GNSS", d.gpsReady ? "PASS" : "FAIL", healthColor(d.gpsReady));
  drawMetricRow(77, "IMU", d.imuReady ? "PASS" : "FAIL", healthColor(d.imuReady));
  drawMetricRow(106, "Nav2", d.nav2Ready ? "PASS" : "FAIL", healthColor(d.nav2Ready));
  drawMetricRow(135, "Safety", d.eStop ? "FAIL" : "PASS", d.eStop ? C_FAULT : C_READY);
}

inline bool isEscMenu(UiMenuId id) {
  const uint8_t value = static_cast<uint8_t>(id);
  return value >= static_cast<uint8_t>(UiMenuId::ESC_ROOT) &&
         value <= static_cast<uint8_t>(UiMenuId::ESC_TEST);
}
inline bool isPerceptionMenu(UiMenuId id) {
  const uint8_t value = static_cast<uint8_t>(id);
  return value >= static_cast<uint8_t>(UiMenuId::PERCEPTION_ROOT) &&
         value <= static_cast<uint8_t>(UiMenuId::PERCEPTION_TEST);
}
inline bool isNavigationMenu(UiMenuId id) {
  return static_cast<uint8_t>(id) >= static_cast<uint8_t>(UiMenuId::NAVIGATION_ROOT);
}

inline void drawUiContent(const UiState& ui, const VehicleTelemetry& d) {
  tft.fillRect(0, CONTENT_Y - 1, W, CONTENT_BOTTOM - CONTENT_Y + 3, C_BG);
  if (ui.menu == UiMenuId::OVERVIEW) {
    drawOverview(ui, d);
  } else if (menuEditKey(ui.menu) != UiEditKey::NONE) {
    drawEditor(ui, d, menuEditKey(ui.menu));
  } else if (menuHasChildren(ui.menu)) {
    drawMenuList(ui);
  } else if (isEscMenu(ui.menu)) {
    drawEscLeaf(ui.menu, ui, d);
  } else if (isPerceptionMenu(ui.menu)) {
    drawPerceptionLeaf(ui.menu, ui, d);
  } else if (isNavigationMenu(ui.menu)) {
    drawNavigationLeaf(ui.menu, ui, d);
  }
  drawRightRail(ui);
}

inline void drawUiFrame(const UiState& ui, const VehicleTelemetry& d, bool full) {
  if (full) {
    tft.fillScreen(C_BG);
    drawUiFooter(ui, d);
  }
  drawUiTopBar(ui, d);
  drawUiContent(ui, d);
}
