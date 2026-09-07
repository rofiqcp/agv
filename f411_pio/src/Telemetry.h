// ============================================================================
// Telemetry.h — Central state model used by all HMI pages
// ============================================================================
#pragma once

#include "Config.h"

struct VehicleTelemetry {
  SystemStatus systemStatus;
  VehicleMode mode;
  VehicleState state;
  bool rosConnected;

  float speedKmh;
  float headingDeg;

  bool gpsReady;
  GpsFixState gpsFix;
  double latitude;
  double longitude;
  uint8_t satellites;
  float hdop;
  bool imuReady;

  bool cameraReady;
  bool perceptionReady;
  float cameraFps;
  char detectedObject[24];
  float objectDistanceM;
  float confidencePct;
  bool drivableAreaClear;
  bool obstacleDetected;

  float steeringTargetDeg;
  float steeringActualDeg;
  float steeringErrorDeg;
  float motorRpm;

  bool escReady;
  bool encoderReady;

  uint8_t manualSpeedPct;

  bool waypointSaved[HMI_WAYPOINT_COUNT];
  char waypointName[HMI_WAYPOINT_COUNT][HMI_WAYPOINT_NAME_LEN];
  uint8_t selectedWaypoint;
  char activeTarget[HMI_WAYPOINT_NAME_LEN];
  NavigationStatus navigationStatus;
};

inline VehicleTelemetry defaultTelemetry() {
  VehicleTelemetry t{};
  t.systemStatus = SYS_INITIALIZING;
  t.mode = MODE_AUTO;
  t.state = STATE_STOPPED;
  t.rosConnected = false;
  t.speedKmh = 0.0f;
  t.headingDeg = 0.0f;

  t.gpsReady = false;
  t.gpsFix = GPS_NO_FIX;
  t.latitude = 0.0;
  t.longitude = 0.0;
  t.satellites = 0;
  t.hdop = 0.0f;
  t.imuReady = false;

  t.cameraReady = false;
  t.perceptionReady = false;
  t.cameraFps = 0.0f;
  snprintf(t.detectedObject, sizeof(t.detectedObject), "%s", "NONE");
  t.objectDistanceM = 0.0f;
  t.confidencePct = 0.0f;
  t.drivableAreaClear = false;
  t.obstacleDetected = false;

  t.steeringTargetDeg = 0.0f;
  t.steeringActualDeg = 0.0f;
  t.steeringErrorDeg = 0.0f;
  t.motorRpm = 0.0f;
  t.escReady = false;
  t.encoderReady = false;
  t.manualSpeedPct = MANUAL_SPEED_DEFAULT;
  const char* defaults[HMI_WAYPOINT_COUNT] = {"Titik A", "Titik B", "Titik C", "Titik D"};
  for (uint8_t i = 0; i < HMI_WAYPOINT_COUNT; ++i) {
    t.waypointSaved[i] = false;
    snprintf(t.waypointName[i], HMI_WAYPOINT_NAME_LEN, "%s", defaults[i]);
  }
  t.selectedWaypoint = 0;
  snprintf(t.activeTarget, sizeof(t.activeTarget), "%s", "NONE");
  t.navigationStatus = NAV_IDLE;
  return t;
}

inline const char* systemStatusText(SystemStatus s) {
  switch (s) {
    case SYS_OFF:          return "OFF";
    case SYS_STARTING:     return "STARTING";
    case SYS_INITIALIZING: return "INITIALIZING";
    case SYS_READY:        return "VEHICLE READY";
    case SYS_NOT_READY:    return "NOT READY";
    case SYS_FAULT:        return "FAULT";
    default:               return "UNKNOWN";
  }
}

inline const char* modeText(VehicleMode mode) {
  return mode == MODE_MANUAL ? "MANUAL" : "AUTO";
}

inline const char* vehicleStateText(VehicleState s) {
  switch (s) {
    case STATE_STANDBY: return "STANDBY";
    case STATE_RUNNING: return "RUNNING";
    case STATE_STOPPED: return "STOPPED";
    case STATE_FAULT:   return "FAULT";
    default:            return "UNKNOWN";
  }
}

inline const char* gpsFixText(GpsFixState s) {
  switch (s) {
    case GPS_LOST:     return "LOST";
    case GPS_NO_FIX:   return "NO FIX";
    case GPS_2D_FIX:   return "2D FIX";
    case GPS_3D_FIX:   return "FIX";
    case GPS_DEGRADED: return "DEGRADED";
    default:           return "NO FIX";
  }
}

inline uint16_t healthColor(bool ok) {
  return ok ? C_READY : C_FAULT;
}

inline uint16_t systemStatusColor(SystemStatus s) {
  if (s == SYS_READY) return C_READY;
  if (s == SYS_FAULT || s == SYS_NOT_READY || s == SYS_OFF) return C_FAULT;
  return C_WARNING;
}

inline uint16_t stateColor(VehicleState s) {
  if (s == STATE_RUNNING) return C_READY;
  if (s == STATE_FAULT) return C_FAULT;
  return C_TEXT_DIM;
}

inline uint16_t gpsFixColor(GpsFixState s) {
  if (s == GPS_3D_FIX) return C_READY;
  if (s == GPS_2D_FIX || s == GPS_DEGRADED) return C_WARNING;
  return C_FAULT;
}

inline const char* navigationStatusText(NavigationStatus s) {
  switch (s) {
    case NAV_SELECTED:   return "TARGET DIPILIH";
    case NAV_QUEUED:     return "MENUNGGU NAV2";
    case NAV_NAVIGATING: return "NAVIGASI AKTIF";
    case NAV_ARRIVED:    return "TIBA";
    case NAV_STOPPED:    return "DIHENTIKAN";
    case NAV_FAILED:     return "NAVIGASI GAGAL";
    case NAV_IDLE:
    default:             return "BELUM ADA TARGET";
  }
}

inline bool navigationHasTarget(const VehicleTelemetry& d) {
  return strcmp(d.activeTarget, "NONE") != 0 && d.activeTarget[0] != '\0';
}
