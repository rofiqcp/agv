// ============================================================================
// CameraPage.h — perception summary matching the approved CAMERA visual
// ============================================================================
#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "Config.h"
#include "Telemetry.h"
#include "Theme.h"
#include "TopBar.h"
#include "BottomMenu.h"
#include "Icons.h"

extern TFT_eSPI tft;

inline void drawCameraContent(const VehicleTelemetry& d) {
  drawCard(FULL_CARD_X, FULL_CARD_Y, FULL_CARD_W, FULL_CARD_H, C_CARD, C_CARD_LINE);

  // Visual illustration: camera + perception cone.
  drawCameraIllustration(FULL_CARD_X + 14, FULL_CARD_Y + 31);

  const int x = FULL_CARD_X + 112;
  drawUiText(d.cameraReady ? "Camera Ready" : "Camera Offline",
             x, FULL_CARD_Y + 9, d.cameraReady ? C_INK : C_FAULT, C_CARD);
  drawSmallText(d.perceptionReady ? "Perception Active" : "Perception Waiting",
                x, FULL_CARD_Y + 29, d.perceptionReady ? C_READY : C_WARNING, C_CARD);

  drawThinDivider(x, FULL_CARD_Y + 52, 184);

  drawSmallText("Object", x, FULL_CARD_Y + 58, C_DISABLED, C_CARD);
  drawSmallText("Distance", x + 112, FULL_CARD_Y + 58, C_DISABLED, C_CARD);

  char objectBuf[13];
  snprintf(objectBuf, sizeof(objectBuf), "%.12s", d.detectedObject);
  useFontValue();
  if (tft.textWidth(objectBuf) <= 98) {
    drawValueText(objectBuf, x, FULL_CARD_Y + 75, C_INK, C_CARD);
  } else {
    drawMicroText(objectBuf, x, FULL_CARD_Y + 85, C_INK, C_CARD);
  }

  char distBuf[12];
  if (d.objectDistanceM > 0.01f) snprintf(distBuf, sizeof(distBuf), "%.2f", d.objectDistanceM);
  else snprintf(distBuf, sizeof(distBuf), "--");
  drawValueText(distBuf, x + 112, FULL_CARD_Y + 75, C_INK, C_CARD);
  if (d.objectDistanceM > 0.01f) {
    int dw = tft.textWidth(distBuf);
    drawSmallText("m", x + 117 + dw, FULL_CARD_Y + 84, C_INK, C_CARD);
  }

  drawThinDivider(x, FULL_CARD_Y + 105, 184);

  drawSmallText("Drivable Area", x, FULL_CARD_Y + 111, C_DISABLED, C_CARD);
  drawSmallText("Obstacle", x + 112, FULL_CARD_Y + 111, C_DISABLED, C_CARD);

  const char* driveText = d.drivableAreaClear ? "CLEAR" : "BLOCKED";
  const uint16_t driveColor = d.drivableAreaClear ? C_INK : C_FAULT;
  drawUiText(driveText, x, FULL_CARD_Y + 126, driveColor, C_CARD);
  drawStatusDot(x + 90, FULL_CARD_Y + 135,
                d.drivableAreaClear ? C_READY : C_FAULT, 3);

  const char* obsText = d.obstacleDetected ? "DETECTED" : "NONE";
  const uint16_t obsColor = d.obstacleDetected ? C_FAULT : C_INK;
  if (d.obstacleDetected) {
    drawMicroText(obsText, x + 112, FULL_CARD_Y + 132, obsColor, C_CARD);
    useFontMicro();
  } else {
    drawUiText(obsText, x + 112, FULL_CARD_Y + 126, obsColor, C_CARD);
    useFontUi();
  }
  drawStatusDot(FULL_CARD_X + FULL_CARD_W - 17, FULL_CARD_Y + 135,
                d.obstacleDetected ? C_FAULT : C_READY, 3);
}

inline void drawCameraPage(const VehicleTelemetry& d) {
  tft.fillScreen(C_BG);
  drawTopBar("CAMERA", d, true, false);
  drawCameraContent(d);
  drawBottomMenu(PAGE_CAMERA);
}

inline void updateCameraPage(const VehicleTelemetry& d) {
  drawTopBar("CAMERA", d, true, false);
  drawCameraContent(d);
}
