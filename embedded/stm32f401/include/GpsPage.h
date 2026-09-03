// ============================================================================
// GpsPage.h — GNSS / localization page matching the approved GPS visual
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

inline void drawGpsContent(const VehicleTelemetry& d) {
  drawCard(FULL_CARD_X, FULL_CARD_Y, FULL_CARD_W, FULL_CARD_H, C_CARD, C_CARD_LINE);

  // Graphical GPS marker with rings, rendered from the approved visual.
  drawGpsIllustration(FULL_CARD_X + 13, FULL_CARD_Y + 42);

  char buf[32];
  snprintf(buf, sizeof(buf), "%.0f", d.headingDeg);
  drawHeroText(buf, FULL_CARD_X + 103, FULL_CARD_Y + 12, C_INK, C_CARD);
  int headingW = tft.textWidth(buf);
  drawDegreeMark(FULL_CARD_X + 107 + headingW, FULL_CARD_Y + 18, C_INK, C_CARD);
  drawSmallText("HEADING", FULL_CARD_X + 106, FULL_CARD_Y + 61, C_DISABLED, C_CARD);

  // Latitude/longitude stay compact so full 6-decimal values fit on 320 px.
  drawMicroText("LAT", FULL_CARD_X + 199, FULL_CARD_Y + 18, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.6f", d.latitude);
  drawMicroText(buf, FULL_CARD_X + 231, FULL_CARD_Y + 18, C_INK, C_CARD);

  drawMicroText("LON", FULL_CARD_X + 199, FULL_CARD_Y + 42, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.6f", d.longitude);
  drawMicroText(buf, FULL_CARD_X + 231, FULL_CARD_Y + 42, C_INK, C_CARD);

  drawThinDivider(FULL_CARD_X + 103, FULL_CARD_Y + 72, 193);

  drawSmallText("SAT", FULL_CARD_X + 106, FULL_CARD_Y + 80, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%u", d.satellites);
  drawValueText(buf, FULL_CARD_X + 106, FULL_CARD_Y + 96, C_INK, C_CARD);

  drawSmallText("HDOP", FULL_CARD_X + 151, FULL_CARD_Y + 80, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.2f", d.hdop);
  drawValueText(buf, FULL_CARD_X + 151, FULL_CARD_Y + 96, C_INK, C_CARD);

  drawSmallText("GNSS", FULL_CARD_X + 207, FULL_CARD_Y + 80, C_DISABLED, C_CARD);
  if (d.gpsFix == GPS_3D_FIX) {
    drawUiText("FIX", FULL_CARD_X + 207, FULL_CARD_Y + 101, C_READY, C_CARD);
  } else {
    drawMicroText(gpsFixText(d.gpsFix), FULL_CARD_X + 207, FULL_CARD_Y + 107,
                  gpsFixColor(d.gpsFix), C_CARD);
  }

  drawSmallText("IMU", FULL_CARD_X + 261, FULL_CARD_Y + 80, C_DISABLED, C_CARD);
  if (d.imuReady) {
    drawCompactText("READY", FULL_CARD_X + 261, FULL_CARD_Y + 104, C_READY, C_CARD);
  } else {
    drawCompactText("OFF", FULL_CARD_X + 261, FULL_CARD_Y + 104, C_FAULT, C_CARD);
  }

  drawThinDivider(FULL_CARD_X + 103, FULL_CARD_Y + 121, 193);
  drawSmallText("Speed", FULL_CARD_X + 106, FULL_CARD_Y + 128, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.1f", d.speedKmh);
  drawUiText(buf, FULL_CARD_X + 167, FULL_CARD_Y + 126, C_INK, C_CARD);
  int speedW = tft.textWidth(buf);
  drawMicroText("km/h", FULL_CARD_X + 173 + speedW, FULL_CARD_Y + 134, C_INK, C_CARD);
}

inline void drawGpsPage(const VehicleTelemetry& d) {
  tft.fillScreen(C_BG);
  drawTopBar("GPS", d, true, false);
  drawGpsContent(d);
  drawBottomMenu(PAGE_GPS);
}

inline void updateGpsPage(const VehicleTelemetry& d) {
  drawTopBar("GPS", d, true, false);
  drawGpsContent(d);
}
