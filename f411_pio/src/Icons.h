// ============================================================================
// Icons.h — compact icons + approved concept illustrations
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include "Config.h"
#include "VisualAssets.h"

extern HmiDisplay tft;

inline void iconHome(int cx, int cy, uint16_t c) {
  tft.fillTriangle(cx - 8, cy - 1, cx, cy - 8, cx + 8, cy - 1, c);
  tft.fillRoundRect(cx - 6, cy - 1, 12, 9, 1, c);
  tft.fillRect(cx - 2, cy + 3, 4, 5, C_BG);
}

inline void iconCamera(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillRoundRect(cx - 12, cy - 7, 24, 16, 3, c);
  tft.fillRoundRect(cx - 6, cy - 11, 9, 5, 2, c);
  tft.fillCircle(cx, cy + 1, 7, bg);
  tft.drawCircle(cx, cy + 1, 7, c);
  tft.drawCircle(cx, cy + 1, 5, c);
  tft.fillCircle(cx, cy + 1, 2, c);
  tft.fillCircle(cx + 8, cy - 3, 1, bg);
}

inline void iconGps(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillCircle(cx, cy - 3, 8, c);
  tft.fillTriangle(cx - 7, cy, cx + 7, cy, cx, cy + 13, c);
  tft.fillCircle(cx, cy - 3, 3, bg);
}

inline void iconSteering(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.drawCircle(cx, cy, 10, c);
  tft.drawCircle(cx, cy, 8, c);
  tft.fillCircle(cx, cy, 3, c);
  tft.drawLine(cx - 8, cy - 3, cx - 2, cy, c);
  tft.drawLine(cx + 8, cy - 3, cx + 2, cy, c);
  tft.drawLine(cx, cy + 3, cx, cy + 9, c);
  tft.fillCircle(cx, cy, 1, bg);
}

inline void iconAutoArrow(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillTriangle(cx - 10, cy + 7, cx + 10, cy - 8, cx + 4, cy + 11, c);
  tft.fillTriangle(cx - 1, cy + 2, cx + 4, cy + 11, cx + 1, cy + 2, bg);
}

inline void iconStop(int cx, int cy, uint16_t c) {
  tft.fillRoundRect(cx - 6, cy - 6, 12, 12, 2, c);
}

inline void iconArrowUp(int cx, int cy, uint16_t c) {
  tft.fillTriangle(cx, cy - 9, cx - 8, cy - 1, cx + 8, cy - 1, c);
  tft.fillRoundRect(cx - 3, cy - 1, 6, 10, 2, c);
}

inline void iconArrowDown(int cx, int cy, uint16_t c) {
  tft.fillRoundRect(cx - 3, cy - 9, 6, 10, 2, c);
  tft.fillTriangle(cx - 8, cy + 1, cx + 8, cy + 1, cx, cy + 9, c);
}

inline void iconArrowLeft(int cx, int cy, uint16_t c) {
  tft.fillTriangle(cx - 9, cy, cx, cy - 8, cx, cy + 8, c);
  tft.fillRoundRect(cx - 1, cy - 3, 10, 6, 2, c);
}

inline void iconArrowRight(int cx, int cy, uint16_t c) {
  tft.fillRoundRect(cx - 9, cy - 3, 10, 6, 2, c);
  tft.fillTriangle(cx, cy - 8, cx + 9, cy, cx, cy + 8, c);
}

inline void drawVehicleIllustration(int x, int y) {
  tft.pushImage(x, y, VEHICLE_ASSET_W, VEHICLE_ASSET_H, VEHICLE_ASSET);
}

inline void drawCameraIllustration(int x, int y) {
  tft.pushImage(x, y, CAMERA_ASSET_W, CAMERA_ASSET_H, CAMERA_ASSET);
}

inline void drawGpsIllustration(int x, int y) {
  tft.pushImage(x, y, GPS_ASSET_W, GPS_ASSET_H, GPS_ASSET);
}
