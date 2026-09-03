// ============================================================================
// TopBar.h — compact status bar used on every HMI page
// ============================================================================
#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "Config.h"
#include "Telemetry.h"
#include "Theme.h"
#include "Icons.h"

extern TFT_eSPI tft;

inline void drawTopHealth(const VehicleTelemetry& d, bool actuatorPage = false) {
  if (actuatorPage) {
    drawMicroText("ESC", 246, 10, C_TEXT, C_BG);
    drawStatusDot(277, 13, healthColor(d.escReady), 3);
    drawMicroText("ENC", 285, 10, C_TEXT, C_BG);
    drawStatusDot(314, 13, healthColor(d.encoderReady), 3);
    return;
  }

  drawMicroText("GPS", 202, 10, C_TEXT, C_BG);
  drawStatusDot(231, 13, d.gpsReady ? gpsFixColor(d.gpsFix) : C_FAULT, 3);
  drawMicroText("CAM", 239, 10, C_TEXT, C_BG);
  drawStatusDot(270, 13, healthColor(d.cameraReady), 3);
  drawMicroText("ESC", 278, 10, C_TEXT, C_BG);
  drawStatusDot(309, 13, healthColor(d.escReady), 3);
}

inline void drawTopBar(const char* title, const VehicleTelemetry& d, bool showHome,
                       bool actuatorPage = false) {
  tft.fillRect(0, 0, W, TOP_H, C_BG);

  if (showHome) {
    iconHome(14, 14, C_TEXT);
    tft.drawFastVLine(28, 6, 16, C_BORDER);
    drawUiText(title, 36, 5, C_TEXT, C_BG);
  } else {
    drawUiText(title, 8, 5, C_TEXT, C_BG);
  }

  drawTopHealth(d, actuatorPage);
}
