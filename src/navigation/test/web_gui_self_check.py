#!/usr/bin/env python3
"""Dependency-light contract for the native C++ localhost web HMI."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]

def fail(msg):
    print("FAIL:", msg)
    raise SystemExit(1)

cpp = (ROOT / "web/web_server.cpp").read_text(encoding="utf-8")
html = (ROOT / "web/static/index.html").read_text(encoding="utf-8")
css = (ROOT / "web/static/styles.css").read_text(encoding="utf-8")
js = (ROOT / "web/static/app.js").read_text(encoding="utf-8")
cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
auto = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
gui = (ROOT / "launch/gui.launch.py").read_text(encoding="utf-8")

for path in (ROOT/"web/static/index.html", ROOT/"web/static/styles.css", ROOT/"web/static/app.js"):
    if not path.is_file() or path.stat().st_size < 1000:
        fail(f"web asset missing/too small: {path}")
for token in ("add_executable(agv_web_gui", "Qt5::Network", "web/static", "agv_web_gui"):
    if token not in cmake: fail(f"CMake web integration missing {token}")
for token in ("start_web_gui", "web_bind_address", "web_port", "127.0.0.1", "agv_web_gui"):
    if token not in auto: fail(f"autonomous launch web contract missing {token}")
for token in ("start_web_gui", "web_bind_address", "web_port"):
    if token not in gui: fail(f"gui launch does not forward {token}")
for endpoint in ("/api/events", "/api/state", "/api/health", "/api/camera.jpg", "/api/map.png",
                 "/api/global_costmap.png", "/api/local_costmap.png",
                 "/api/experiment/record/last.csv", "/api/navigation/goal", "/api/navigation/cancel",
                 "/api/localization/initial-pose", "/api/steering/calibration-mode",
                 "/api/esc/vesc/command", "/api/config/set", "/api/config/reset", "/api/config/reset-batch", "/api/experiment/record/start",
                 "/api/experiment/record/stop", "/api/experiment/record/status"):
    if endpoint not in cpp: fail(f"web endpoint missing {endpoint}")
for page in ("overview", "navigation", "perception", "sensors", "esc", "calibration", "tuning", "experiments", "reports", "diagnostics", "configuration"):
    if f'id="page-{page}"' not in html: fail(f"frontend page missing {page}")
for bad in ("https://", "http://cdn", "unpkg.com", "cdnjs", "jsdelivr"):
    if bad in html or bad in js or bad in css: fail(f"web GUI must remain offline/self-contained: {bad}")
if "EventSource('/api/events')" not in js:
    fail("frontend realtime SSE connection missing")
if '<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">' not in html:
    fail("responsive viewport meta missing")
if '<link rel="icon" href="data:,">' not in html:
    fail("inline favicon guard missing; browser must not emit favicon 404")
for token in ("Mobile containment", ".workbench-main>*", ".domain-tab>div", ".table-scroll{max-width:100%", ".sidebar-backdrop", ".sidebar.open + .sidebar-backdrop"):
    if token not in css: fail(f"BAB IV mobile containment missing {token}")
if 'id="sidebarBackdrop"' not in html or "$('sidebarBackdrop').onclick" not in js:
    fail("mobile sidebar backdrop behavior missing")

# BAB IV web workbench must expose the three source domains and real tuning/evidence surfaces.
for token in ("CONTROL ENGINEERING", "Tuning & Control", "data-exp=\"navigation\"",
              "data-exp=\"perception\"", "data-exp=\"steering\"",
              "id=\"tuningFields\"", "id=\"experimentGraphs\"", "id=\"experimentTables\"",
              "id=\"expMapCanvas\"", "id=\"expCameraImage\"", "id=\"expEscCanvas\"",
              "id=\"recordToggleBtn\""):
    if token not in html: fail(f"BAB IV web workbench missing {token}")
if 'id="yoloToggleBtn"' not in html or 'id="yoloToggleState"' not in html:
    fail("explicit YOLOPv2 perception toggle missing")
# NEO-3 must expose GNSS receiver link/fix separately from the IST8310 compass.
for token in ('NEO-3 GNSS M9N', 'id="gnssChip"', 'id="gnssFixChip"', 'id="gnssRate"',
              'NEO-3 Magnetometer IST8310', 'id="neo3MagChip"', 'id="neo3MagX"',
              'id="neo3MagY"', 'id="neo3MagZ"', 'id="neo3MagNorm"', 'id="neo3MagHeading"'):
    if token not in html: fail(f"NEO-3 split sensor UI missing {token}")
for token in ("connected.gnss", "qg.gnss_fix_ok", "gnssFixChip", "pvt_rate_hz",
              "connected.neo3_mag", "neo3_mag_heading_valid", "neo3MagX", "neo3MagY", "neo3MagZ"):
    if token not in js: fail(f"NEO-3 split sensor rendering missing {token}")
for token in ('/gnss/connected', '/gnss/quality', '/neo3/status', '/neo3/ist8310_connected',
              '/neo3/mag', '/neo3/mag_heading_fusion', '/neo3/mag_heading_valid'):
    if token not in cpp: fail(f"NEO-3 web ROS binding missing {token}")
per_start = html.find('id="page-perception"')
per_end = html.find('id="page-sensors"', per_start)
perception_html = html[per_start:per_end if per_end > per_start else len(html)]
if 'data-hmi-camera-tab="VIEW"' in perception_html or 'data-hmi-camera-tab="DETECT"' in perception_html:
    fail("legacy VIEW/DETECT/DRIVE/STATUS block still present in Persepsi")
for token in ("WEB_TUNING", "resolveMetricPath", "drawExperimentChart", "drawExperimentMap",
              "drawExperimentEsc", "saveTuningField", "startWebRecording", "stopWebRecording", "setRecordingUi"):
    if token not in js: fail(f"BAB IV web behavior missing {token}")
for token in ("setYamlValueAtomic", "patchExistingYamlScalar", "captureRecordingSample",
              "saveRecordingFiles", "/home/otomasi/ros/data", "navigasi", "presepsi",
              "download_url", "global_costmap_meta", "local_costmap_meta",
              "foc_thesis", "bbox_calibration",
              "localization_cpp.yaml", "mppi_closed_loop.yaml"):
    if token not in cpp: fail(f"web tuning backend missing {token}")

# Navigation tuning must preserve an immutable initial YAML baseline and expose safe reset.
for token in ("baselinePathForConfig", "ensureConfigBaseline", "baselineYamlValue",
              "applyConfigChanges", "safe_batch_restart", "batch_backups",
              "baseline_data", "RESET BASELINE", "/api/config/reset", "/api/config/reset-batch"):
    if token not in cpp: fail(f"YAML baseline/reset backend missing {token}")
for token in ('id="resetExperimentYaml"', '*.web.baseline', '.web.bak.*'):
    if token not in html: fail(f"YAML baseline/reset UI missing {token}")
for token in ("baselineConfigValue", "resetSelectedExperimentYaml", "Reset YAML tahap ini"):
    if token not in js: fail(f"YAML baseline/reset frontend missing {token}")
for name in ("vehicle.yaml", "navigation_core.yaml", "nav2_ackermann.yaml", "ekf.yaml",
             "localization_cpp.yaml", "gnss.yaml", "imu.yaml", "stage3_navigation.yaml",
             "trajectory_safety.yaml", "collision_monitor_production.yaml",
             "mppi_closed_loop.yaml", "gui_calibration.yaml"):
    baseline = ROOT / "config" / f"{name}.web.baseline"
    if not baseline.is_file() or baseline.stat().st_size < 10:
        fail(f"navigation baseline missing/empty: {baseline}")

# RViz-like browser interaction: real Nav2 costmaps, pan/zoom, goal yaw drag and auto-save/download.
for token in ("id=\"mapGoalTool\"", "id=\"mapPanTool\"", "id=\"layerGlobalCostmap\"",
              "id=\"layerLocalCostmap\"", "id=\"layerGrid\"", "id=\"mapZoomLabel\""):
    if token not in html: fail(f"RViz-like navigation control missing {token}")
for token in ("loadCostmapImage", "zoomMap", "screenToWorld", "pointerdown", "pointermove",
              "template-only report", "saveTemplateTableServer"):
    if token not in js: fail(f"RViz-like/autosave frontend behavior missing {token}")


# ESC/VESC gateway UI must expose real F411 transport, safe maintenance and desktop VESC Tool TCP.
for token in ("VESC Tool Workbench", "F411 PB6/PB7", "127.0.0.1:65102",
              "vescEnterMaintenance", "vescExitMaintenance", "vescReadMcconf",
              "vescDetectHall", "vescDetectEncoder", "vescSendTerminal", "vescRawHex"):
    if token not in html: fail(f"VESC Web workbench missing {token}")
for token in ("vescCmd", "vesc_tool_status", "tcp_client", "VESC TOOL TCP",
              "command',{command}", "renderVescTool"):
    if token not in js: fail(f"VESC Web behavior missing {token}")
for token in ("/esc/vesc/tool_command", "/stmf4/vesc/status", "/esc/vesc/tool_telemetry",
              "vesc_tool.yaml"):
    if token not in cpp: fail(f"VESC Web backend integration missing {token}")

if "Content-Security-Policy" not in cpp:
    fail("HTTP response security headers missing")
for forbidden in ("_summary.csv", "_manifest.json", "_config.json"):
    if forbidden in cpp: fail(f"CSV-only recorder must not persist {forbidden}")
for token in ("recordingCsvStem", "section_label", "lastDownloadCsv_", "recordToggleBtn"):
    hay = cpp if token != "recordToggleBtn" else html
    if token not in hay: fail(f"CSV-only recorder contract missing {token}")

for token in ("cameraEncodeMutex_", "rosShutdownGuard", "Request body too large",
              "canonicalRoot", "declare_parameter<std::int64_t>(\"port\"",
              "configuredPort > 65535"):
    if token not in cpp: fail(f"web runtime hardening missing {token}")
print("PASS web_gui_self_check")
