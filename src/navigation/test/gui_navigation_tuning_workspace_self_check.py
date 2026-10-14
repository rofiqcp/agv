#!/usr/bin/env python3
"""Static contract for BAB-IV navigation tuning workspace behavior."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMP = (ROOT / "gui/modules/experiment_components.cpp").read_text(encoding="utf-8")
WORK = (ROOT / "gui/modules/system_and_gnss_pages.cpp").read_text(encoding="utf-8")
MAIN = (ROOT / "gui/modules/main_window.cpp").read_text(encoding="utf-8")
CAT = (ROOT / "gui/agv_experiment_catalog.hpp").read_text(encoding="utf-8")
WEB_NAV = (ROOT / "web/static/navigation.js").read_text(encoding="utf-8")
WEB_EVIDENCE = (ROOT / "web/static/tuning_evidence.js").read_text(encoding="utf-8")
WEB_UI = (ROOT / "web/static/tuning_ui.js").read_text(encoding="utf-8")
WEB_PACKAGES = (ROOT / "web/static/navigation_packages.js").read_text(encoding="utf-8")
WEB_HTML = (ROOT / "web/static/index.html").read_text(encoding="utf-8")
WEB_CPP = (ROOT / "web/web_server.cpp").read_text(encoding="utf-8")
PACKAGE_TOOL = (ROOT / "tools/package_bab4_navigation.py").read_text(encoding="utf-8")
ESC_LAUNCH = (ROOT.parent / "esc/launch/esc.launch.py").read_text(encoding="utf-8")
PROFILE_BLOCK = WEB_UI.split("const NAV_BAB4_GLOBAL_PROFILE_FIELDS=[", 1)[1].split("\n];", 1)[0]
PROFILE_ESC_LINES = [line for line in PROFILE_BLOCK.splitlines() if "fileKey:'esc'" in line]

checks = {
    "YAML editor loads actual value": "if (yamlValue.isValid()) edit->setText(editorText(yamlValue));" in COMP,
    "YAML commits on editingFinished": "&QLineEdit::editingFinished" in COMP,
    "typed bool YAML support": 'fkind == QStringLiteral("bool")' in COMP,
    "typed integer YAML support": 'fkind == QStringLiteral("int")' in COMP,
    "typed float YAML support": 'fkind == QStringLiteral("float")' in COMP,
    "YAML commit event exists": "yamlParameterCommitted" in COMP,
    "graph selector exists": "graphSelector_ = new NoWheelComboBox()" in WORK,
    "Start CSV label exists": 'QStringLiteral("● Start CSV")' in WORK,
    "Stop CSV autosave label exists": 'QStringLiteral("■ Stop CSV + Auto Save")' in WORK,
    "Stop triggers evidence autosave": "saveEvidence(false);" in WORK,
    "autosave exports every table": '"table_csv_files"' in WORK and "tableCsvPaths" in WORK,
    "autosave exports every graph": "for (int i = 0; i < graphCards_.size(); ++i)" in WORK,
    "autosave status exists": "CSV AUTO-SAVED" in WORK,
    "leaf sample-rate override": 'parameterValue(QStringLiteral("sample_rate"))' in WORK,
    "menu exposes counts": 'spec.tableColumns.size()).arg(spec.graphCaptions.size())' in MAIN,
    "4.10 has zero graph UX": "0 grafik" in WORK,
    "planner tuning metadata": "GridBased.minimum_turning_radius" in CAT,
    "MPPI tuning metadata": "PathAlignCritic.cost_weight" in CAT,
    "velocity smoother tuning metadata": "velocity_smoother.ros__parameters.smoothing_frequency" in CAT,
    "goal checker tuning metadata": "goal_checker.xy_goal_tolerance" in CAT,
    "SEND GOAL starts master recorder first": "startNavigationGoalMissionRecording(goal)" in WEB_NAV,
    "SEND GOAL blocks if master recorder unavailable": "goal tidak dikirim agar data awal tidak hilang" in WEB_NAV,
    "master recorder covers 4.2 through 4.4": r"^R4\.[234]\.[1-5]$" in WEB_EVIDENCE,
    "4.4.5 manual START disabled": "4.4.5 AUTO: gunakan SEND GOAL" in WEB_EVIDENCE,
    "master stops on Nav2 terminal state": "stopNavigationGoalMissionRecording(st)" in WEB_EVIDENCE,
    "mission metadata includes cycle and goal": "goal_x_m" in WEB_EVIDENCE and "primary_experiment_id" in WEB_EVIDENCE and "cycle" in WEB_EVIDENCE,
    "one-step tuning apply button exists": 'id="tuningQuickApplyBtn"' in WEB_HTML and "APPLY TUNING" in WEB_HTML,
    "runtime match card exists": 'id="tuningMatchStatus"' in WEB_HTML and "ACTIVE MATCH" in WEB_UI,
    "one-step apply auto validates": "validateConfigItems(items,{openReview:false})" in WEB_UI,
    "runtime readback endpoint exists": '/api/config/runtime-check' in WEB_CPP and "configRuntimeState(fk,path)" in WEB_CPP,
    "SEND GOAL runtime tuning gate exists": "ensureNavigationTuningRuntimeReady" in WEB_EVIDENCE and "VERIFYING TUNING" in WEB_EVIDENCE,
    "draft blocks SEND GOAL": "DRAFT/STAGED belum di-apply" in WEB_UI,
    "restore baseline quick button exists": 'id="tuningQuickBaselineBtn"' in WEB_HTML and "RESTORE BASELINE" in WEB_HTML,
    "restore baseline is staged before apply": "nilai YAML aktif yang tersimpan sebelum tuning sesi ini" in WEB_UI and "quickRestoreBaseline" in WEB_UI,
    "discard returns current active yaml": "kembali ke CURRENT YAML aktif" in WEB_UI,
    "SEND GOAL verifies all 4.2 through 4.4 runtime": "navigationMissionRuntimeCheckItems" in WEB_UI and "4.2–4.4 ACTIVE MATCH" in WEB_UI,
    "BAB4 session baseline uses current YAML snapshot": "ensureNavigationSessionBaseline" in WEB_UI and "server_persistent_session_baseline" in WEB_CPP,
    "BAB4 restore uses persistent session snapshot": "navigation-session-baseline" in WEB_UI and "action:'restore'" in WEB_UI,
    "BAB4 hides historical baseline value": "!isNavigationBab4Tuning(x)" in WEB_UI and "AKTIF SISTEM" in WEB_UI,
    "BAB4 report YAML fields can be tuned": "reportEditable=isNavigationBab4Tuning(x)" in WEB_UI,
    "BAB4 session baseline IDs supported backend": "R4\\.[234]\\.[1-5]" in WEB_CPP,
    "BAB4 global 3-profile selector exists": "PROFILE GLOBAL • 3 PERCOBAAN" in WEB_UI and "data-nav-global-profile" in WEB_UI,
    "BAB4 global profile stages draft only": "stageNavigationGlobalProfile" in WEB_UI and "runtime belum berubah" in WEB_UI,
    "BAB4 global profile covers EKF planner goal": "NAV_BAB4_GLOBAL_PROFILE_FIELDS" in WEB_UI and "sensor_timeout" in WEB_UI and "cost_penalty" in WEB_UI and "xy_goal_tolerance" in WEB_UI,
    "BAB4 web goal requires master recorder": "master recorder BAB4 4.2-4.4 belum aktif" in WEB_CPP and "recordingId_ == QStringLiteral(\"BAB4_MISSION_4.2_4.4\")" in WEB_CPP,
    "BAB4 observed goal auto recorder exists": "goal_request_event" in WEB_CPP and "ensureBab4RecorderForObservedGoal" in WEB_CPP and "AUTO_OBSERVED_GOAL_MASTER_4.2_4.4" in WEB_CPP,
    "BAB4 browser adopts observed goal recorder": "adoptObservedGoalMasterRecorder" in WEB_EVIDENCE and "AUTO-ADOPT goal eksternal/HMI" in WEB_EVIDENCE,
    "BAB4 data package page exists": 'id="navigationDataPackagePanel"' in WEB_HTML and "R4.5.1" in (ROOT / "web/static/tuning_catalog.js").read_text(encoding="utf-8"),
    "BAB4 package is built on master stop": "buildNavigationBab4Package" in WEB_CPP and "package_saved" in WEB_CPP,
    "BAB4 package list and download API exist": "/api/navigation/data-packages" in WEB_CPP and "/api/navigation/data-package/" in WEB_CPP,
    "BAB4 package delete is isolated": "deleteNavigationDataPackage" in WEB_CPP and "delete_source" in WEB_CPP,
    "BAB4 package frontend download delete exists": "DOWNLOAD ZIP" in WEB_PACKAGES and "data-package/delete" in WEB_PACKAGES,
    "BAB4 incomplete ZIP remains downloadable for audit": "DOWNLOAD ZIP • AUDIT" in WEB_PACKAGES and "downloadable=p.zip_exists!==false" in WEB_PACKAGES,
    "BAB4 success opens package page": "navigationDataPackageOnMissionSaved" in WEB_PACKAGES and "R4.5.1" in WEB_PACKAGES,
    "BAB4 package excludes 4.1": '"scope": "4.2-4.4"' in PACKAGE_TOOL and '"4.2.1"' in PACKAGE_TOOL and '"4.4.5"' in PACKAGE_TOOL,
    "BAB4 package Windows-safe paths": "windows_safe_paths" in PACKAGE_TOOL and "max_internal_path_chars" in PACKAGE_TOOL,
    "BAB4 package exact GUI table columns": "normalize_gui_table" in PACKAGE_TOOL and "trim_table_to_terminal" in PACKAGE_TOOL and "LIVE_MISSION_TABLE_CSV" in PACKAGE_TOOL and "TABEL SESUAI GUI" in PACKAGE_TOOL,
    "BAB4 package builds per-table XLSX": "build_table_xlsx" in PACKAGE_TOOL and "Tabel_" in PACKAGE_TOOL,
    "BAB4 download is report-only 4.2-4.4": "report_only_zip" in PACKAGE_TOOL and "build_report_tree" in PACKAGE_TOOL and "BAB4_Report_{candidate}_C" in PACKAGE_TOOL,
    "BAB4 report contains real graphs without fake single-run overlay": "Grafik_" in PACKAGE_TOOL and "Do not fabricate an overlay from a single run" in PACKAGE_TOOL,
    "BAB4 master sends GUI PNG payload": "collectNavigationGoalMasterGraphPngPayload" in WEB_EVIDENCE and "browser_graph_pngs:browserGraphPngs" in WEB_EVIDENCE,
    "BAB4 master watchdog captures independent of active page": "Dedicated master watchdog" in WEB_EVIDENCE and "captureLabTelemetry();" in WEB_EVIDENCE,
    "BAB4 master records GNSS fusion sources": "gnss_base_vel_fusion" in WEB_EVIDENCE and "gnss_cog_fusion" in WEB_EVIDENCE,
    "BAB4 goal checker is position radius only": "nav2_controller::SimpleGoalChecker" in (ROOT / "config/nav2_ackermann.yaml").read_text(encoding="utf-8") and "stateful: false" in (ROOT / "config/nav2_ackermann.yaml").read_text(encoding="utf-8"),
    "Nav2 tuning live-applies around materialized temp SSOT": "liveNav2Changes" in WEB_CPP and "setRuntimeScalarParameter" in WEB_CPP and "stale temp file" in WEB_CPP,
    "Candidate ESC covariance live-applies without actuator restart": "liveEscTuningPaths" in WEB_CPP and "esc_cov_live_apply" in WEB_CPP and "odom_v_variance_base" in WEB_CPP and "odom_yaw_rate_variance_base" in WEB_CPP,
    "Candidate profiles never tune teleop or VESC motor control": len(PROFILE_ESC_LINES) == 2 and all("odom_" in line and "variance_base" in line for line in PROFILE_ESC_LINES) and all(token not in PROFILE_BLOCK.lower() for token in ("teleop", "steering_kp", "steering_ki", "steering_kd", "position_kp", "speed_kp", "current_limit", "duty", "erpm", "foc", "vesc", "drive_erpm_per_mps")),
    "Full ROS restart preserves baseline teleop wiring": 'DeclareLaunchArgument("start_teleop", default_value="true")' in ESC_LAUNCH and 'DeclareLaunchArgument("start_ackermann", default_value="true")' in ESC_LAUNCH and 'DeclareLaunchArgument("teleop_topic", default_value="/cmd_vel/teleop")' in ESC_LAUNCH and 'teleop_params' in ESC_LAUNCH and 'ackermann_params' in ESC_LAUNCH,
    "motor_teleop auto-respawns after transient crash": 'name="motor_teleop"' in ESC_LAUNCH and 'respawn=True, respawn_delay=1.0' in ESC_LAUNCH,
    "Config patcher safely collapses block YAML sequences": "Block-style ROS arrays" in WEB_CPP and "lines.erase(lines.begin() + target + 1" in WEB_CPP,
    "R4.4.5 report uses position radius pass": "Position radius pass" in (ROOT / "web/static/tuning_catalog.js").read_text(encoding="utf-8") and "Goal Checker dan Akurasi Posisi" in (ROOT / "web/static/tuning_catalog.js").read_text(encoding="utf-8"),
    "BAB4 tables validate continuous one-second rows": "navigationGoalMasterContinuityReport" in WEB_EVIDENCE and "missing_seconds" in WEB_EVIDENCE,
    "BAB4 table capture forbids synthetic backfill": "never synthesize/backfill missed seconds" in WEB_EVIDENCE and "backfilled:false" in WEB_EVIDENCE and "synthetic/backfill forbidden" in WEB_EVIDENCE,
    "BAB4 package READY requires one-Hz continuity": "continuous_1hz" in PACKAGE_TOOL and "continuity_ok" in PACKAGE_TOOL and "expected_seconds" in PACKAGE_TOOL,
    "BAB4 strict field package requires complete field graphs": "strict_real_field" in PACKAGE_TOOL and "field_graphs_complete" in PACKAGE_TOOL and "SERVER_FIELD_RENDER" in PACKAGE_TOOL,
    "BAB4 strict field package rejects fallback source": "strict_sources_ok" in PACKAGE_TOOL and "legacy_table_fallback_used" in PACKAGE_TOOL and "synthetic_fill" in PACKAGE_TOOL,
    "BAB4 RAW validates every mission second": "raw_second_counts" in PACKAGE_TOOL and "raw_missing_seconds" in PACKAGE_TOOL and "raw_recording_ok" in PACKAGE_TOOL,
    "BAB4 RAW validates terminal Nav2 state": "raw_terminal_state" in PACKAGE_TOOL and "raw_terminal_ok" in PACKAGE_TOOL and "SUCCEEDED" in PACKAGE_TOOL,
    "BAB4 continuity metadata reaches package builder": "table_continuity" in WEB_CPP and "table_continuity:continuity" in WEB_EVIDENCE,
}
failed = [name for name, ok in checks.items() if not ok]
if failed:
    for name in failed:
        print(f"FAIL: {name}")
    raise SystemExit(1)
print(f"PASS: {len(checks)} navigation tuning workspace checks")
