// ============================================================================
// HomePage.h — ADV overview matching the approved 3D-printer-style visual
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

inline void drawHomeContent(const VehicleTelemetry& d) {
  char buf[32];

  // Large speed/vehicle card.
  drawCard(HOME_MAIN_X, HOME_MAIN_Y, HOME_MAIN_W, HOME_MAIN_H, C_CARD, C_CARD_LINE);

  snprintf(buf, sizeof(buf), "%.1f", d.speedKmh);
  drawHeroText(buf, HOME_MAIN_X + 14, HOME_MAIN_Y + 10, C_INK, C_CARD);
  drawSmallText("km/h", HOME_MAIN_X + 18, HOME_MAIN_Y + 64, C_DISABLED, C_CARD);

  const uint16_t statusColor = systemStatusColor(d.systemStatus);
  drawUiText(systemStatusText(d.systemStatus), HOME_MAIN_X + 16, HOME_MAIN_Y + 98,
             statusColor, C_CARD);

  snprintf(buf, sizeof(buf), "Heading %.0f", d.headingDeg);
  drawSmallText(buf, HOME_MAIN_X + 16, HOME_MAIN_Y + 121, C_INK, C_CARD);
  int headingW = tft.textWidth(buf);
  drawDegreeMark(HOME_MAIN_X + 19 + headingW, HOME_MAIN_Y + 122, C_INK, C_CARD);

  // Richer delivery-vehicle illustration taken from the approved visual.
  drawVehicleIllustration(HOME_MAIN_X + 150, HOME_MAIN_Y + 16);

  // MODE card.
  drawCard(HOME_SIDE_X, HOME_MODE_Y, HOME_SIDE_W, HOME_SIDE_H, C_CARD, C_CARD_LINE);
  drawMicroText("MODE", HOME_SIDE_X + HOME_SIDE_W / 2, HOME_MODE_Y + 9,
                C_DISABLED, C_CARD, TC_DATUM);
  drawUiText(modeText(d.mode), HOME_SIDE_X + HOME_SIDE_W / 2, HOME_MODE_Y + 23,
             C_INK, C_CARD, TC_DATUM);
  iconAutoArrow(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_MODE_Y + 53,
                d.mode == MODE_AUTO ? C_ACCENT : C_DISABLED, C_CARD);

  // STATE card.
  drawCard(HOME_SIDE_X, HOME_STATE_Y, HOME_SIDE_W, HOME_SIDE_H, C_CARD, C_CARD_LINE);
  drawMicroText("STATE", HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 9,
                C_DISABLED, C_CARD, TC_DATUM);
  const char* state = vehicleStateText(d.state);
  drawCompactText(state, HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 27,
                  C_INK, C_CARD, TC_DATUM);

  if (d.state == STATE_RUNNING) {
    iconAutoArrow(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 54, C_READY, C_CARD);
  } else {
    iconStop(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 54,
             d.state == STATE_FAULT ? C_FAULT : C_FAULT);
  }
}

inline void drawHomePage(const VehicleTelemetry& d) {
  tft.fillScreen(C_BG);
  drawTopBar("ADV", d, true, false);
  drawHomeContent(d);
  drawBottomMenu(PAGE_HOME); // no bottom tile highlighted while on HOME
}

inline void updateHomePage(const VehicleTelemetry& d) {
  drawTopBar("ADV", d, true, false);
  drawHomeContent(d);
}
