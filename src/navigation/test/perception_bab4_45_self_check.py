"""Static regression contract for Persepsi Bab IV 4.5 P1/P2 fixes."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent
STATIC = ROOT / "web/static"

def req(ok: bool, msg: str) -> None:
    if not ok:
        raise AssertionError(msg)

js = (STATIC / "perception_bab4_locked.js").read_text(encoding="utf-8")
heartbeat_worker = (STATIC / "perception_bab4_heartbeat_worker.js").read_text(encoding="utf-8")
css = (STATIC / "perception_bab4_locked.css").read_text(encoding="utf-8")
camera = (STATIC / "camera_geometry.js").read_text(encoding="utf-8")
index = (STATIC / "index.html").read_text(encoding="utf-8")
store = (ROOT / "web/bab4_perception_session.hpp").read_text(encoding="utf-8")
server = (ROOT / "web/web_server.cpp").read_text(encoding="utf-8")
safety = (ROOT / "src/trajectory_safety_supervisor.cpp").read_text(encoding="utf-8")
router = (ROOT / "src/cmd_vel_router.cpp").read_text(encoding="utf-8")
esc = (SRC / "esc/src/ackermann_controller_server.cpp").read_text(encoding="utf-8")
cpu = (SRC / "perception/src/astra_yolop_cpu_pt_node.cpp").read_text(encoding="utf-8")
semantic = (SRC / "perception/tools/semantic_obstacle_node.py").read_text(encoding="utf-8")
pcmake = (SRC / "perception/CMakeLists.txt").read_text(encoding="utf-8")
nav2 = (ROOT / "config/nav2_ackermann.yaml").read_text(encoding="utf-8")
safety_cfg = (ROOT / "config/trajectory_safety.yaml").read_text(encoding="utf-8")
core_js = (STATIC / "core.js").read_text(encoding="utf-8")
navigation_js = (STATIC / "navigation.js").read_text(encoding="utf-8")
exporter = (ROOT / "tools/export_perception_bab4.py").read_text(encoding="utf-8")

for token in ("function v3Refresh45Live()", "v3Render(true)", "v3Last45LiveRender",
              "setInterval(()=>{if(v3IsActive())"):
    req(token in js, f"4.5 realtime refresh contract missing: {token}")

# 4.2 formal protocol: DAY vs NIGHT x 1..5 m x 10 captures with raw-data lineage.
for token in ("V3_42_LIGHT", "function v3Key42(", "100 sampel formal",
              "lighting_condition:V3.light42", "Mean Luma",
              "Detection Rate SIANG vs MALAM"):
    req(token in js, f"4.2 day/night GUI contract missing: {token}")
req('^human_((day|night)_)?[1-5]m$' in store,
    "4.2 backend condition schema must accept explicit day/night and legacy night keys")
for token in ('human_%1_%2m', 'light=="night"&&!c42.contains(key)'):
    req(token in server, f"4.2 export readiness day/night contract missing: {token}")
for token in ("raw_all.csv", 'f"raw_{light}.csv"',
              'lights=[("night","MALAM / minim cahaya"),("day","SIANG / banyak cahaya")]',
              "graph_source_manifest.csv", "estimated_vs_reference.png",
              "lighting_comparison.csv", "distance_model_comparison.csv",
              "combined_invwidth_height", "train DAY -> test NIGHT"):
    req(token in exporter, f"4.2 raw/day-night/calibration export contract missing: {token}")
req("illumination_mean_luma" in store and "illumination_stddev_luma" in store,
    "4.2 raw rows must preserve measured illumination evidence")

# Reference -> actual -> error must be explicit in formal BAB IV data.
for token in ("reference_type","reference_source","actual_source",
              "reference_value","actual_value","signed_error","abs_error"):
    req(token in store, f"generic reference/actual/error raw contract missing: {token}")
for token in ("abs_distance_error_m","lane_offset_reference_m","centering_error_m",
              "fps_reference_hz","fps_error_hz","inference_time_reference_ms",
              "goal_reference_m","goal_actual_progress_m","goal_signed_error_m",
              "goal_endpoint_error_m","heading_signed_error_deg","steering_signed_error_deg"):
    req(token in store, f"specific reference/error field missing: {token}")
for token in ("fps_reference_vs_actual.png","fps_target_vs_actual.png",
              "inference_target_vs_actual.png","obstacle_goal_reference_vs_actual.png",
              "lane_goal_reference_vs_actual.png","lane_heading_error.png",
              "lane_steering_error.png"):
    req(token in exporter, f"reference-vs-actual graph/export missing: {token}")
for token in ("Target FPS","Actual FPS","Goal Ref [m]","Goal Error [m]",
              "Heading Err [deg]","Steer Err [deg]"):
    req(token in js, f"reference/error GUI evidence missing: {token}")

# ARMED heartbeat must run in an independent Web Worker so rendering stalls do
# not cause false watchdog stops. Backend watchdog remains the fail-safe owner.
for token in ("new Worker('/perception_bab4_heartbeat_worker.js?v=20260926-worker-v1')",
              "v3StartHeartbeatWorker(", "v3StopHeartbeatWorker(",
              "v3HandleHeartbeatWorkerMessage"):
    req(token in js, f"dedicated heartbeat-worker GUI contract missing: {token}")
for token in ("setInterval(beat,500)", "worker heartbeat timeout",
              "transport:'web_worker'", "/api/perception/final-trial/heartbeat"):
    req(token in heartbeat_worker, f"heartbeat worker contract missing: {token}")
for token in ("heartbeat_count", "heartbeat_age_ms", "heartbeat_transport",
              "heartbeat_sequence"):
    req(token in server and token in store,
        f"heartbeat telemetry/evidence contract missing: {token}")
req("finalHeartbeatCount_" in server,
    "backend heartbeat counter member missing")
req('t-finalHeartbeat_>6000' in server,
    "backend 6 s heartbeat fail-safe watchdog must remain enabled")

# 4.5A HUMAN-only planning + pre-motion planner gate + runtime hold/recovery.
for token in ('obstacle_modes = {"OBSTACLE", "OBSTACLE_GATE"}',
              "self.obstacle_memory_trial_active and class_id != 0",
              '"trial_person_only_planning_filter"',
              '"final_trial_guard_mode"'):
    req(token in semantic, f"4.5A HUMAN-only/gate semantic contract missing: {token}")
for token in ("final_obstacle_gate_active", "FINAL_TRIAL_PLANNER_GATE_STOP",
              'final_mode=="LANE"||final_mode=="OBSTACLE"||final_mode=="OBSTACLE_GATE"'):
    req(token in safety, f"planner gate safety contract missing: {token}")
for token in ('publishFinalGuard("OBSTACLE_GATE"', "WAIT_HUMAN_WORLD_MEMORY",
              "WAIT_GLOBAL_COSTMAP_POST_LATCH", "REJECT_LOOP_OR_EXCESSIVE_GLOBAL_PATH",
              "PLANNER FAIL-SAFE HOLD", "PLANNER RECOVERED -> MOTION RESUMED",
              "navPathLength<=maxReasonablePath", "navPathHeadingVar<=3.50",
              "planner_hold_count", "planner_gate_pass"):
    req(token in server, f"planner gate/loop fail-safe backend contract missing: {token}")
for token in ("trial_person_only_planning_filter", "obstacle_motion_released",
              "planner_gate_reason", "planner_gate_path_length_m",
              "planner_hold_count", "planner_hold_recovery_count",
              "planner_path_length_m", "planner_path_quality_ok"):
    req(token in store and token in exporter,
        f"planner gate evidence/export contract missing: {token}")

for token in ("obstacle_trial_1","obstacle_trial_2","obstacle_trial_3",
              "lane_trial_1","lane_trial_2","lane_trial_3"):
    req(token in js or token in store or token in server, f"4.5 three-trial schema missing: {token}")
req("goal_distance_m" in store and "5.0" in store and
    'repeatTrial?5.0' in server,
    "4.5A repeated obstacle trials must use fixed 5 m goal without distance selector")
req('semantic_safety_' in store and 'human_tracking_state' in store,
    "4.5 research row must preserve semantic safety/tracking source")
req("requireNav2Eligible" in store and 'd.value("nav2_eligible")' in store and
    'safety_human_valid' in store,
    "4.5 safety human selection must enforce nav2_eligible + tracked safety validity")
for token in ("human_track_continue_min_score", "human_track_iou_min",
              "human_track_center_distance_max", "human_track_max_without_strong_sec",
              'tracking_state"] = "STRONG"', 'tracking_state"] = "TRACKED"',
              'tracking_state"] = "HELD"', "safety_human_valid",
              "A low-score proposal can never start a track by itself",
              "semantic_center_crop_fraction", 'preprocess_mode = "CENTER_CROP"',
              "person_acquire_score_threshold", "person_acquire_min_bbox_height_fraction",
              "semantic_watchdog_loop", "semantic_watchdog_inference_timeout_sec",
              "semantic_watchdog_output_timeout_sec", "os._exit(87)", "os._exit(88)"):
    req(token in semantic, f"semantic HUMAN temporal/crop/watchdog contract missing: {token}")

for token in ("Gagal menyimpan CSV hasil ARMED; perubahan di-rollback",
              "Gagal menyimpan state hasil ARMED; CSV di-rollback"):
    req(token in store, f"ARMED transactional persistence missing: {token}")
req("Research persistence failed:" in server,
    "web runtime must invalidate trial when research persistence fails")
req('starting&&!s.value("system.autonomy_ready").toBool(false)' in server and
    "Autonomy start gate belum READY" in server,
    "4.5A ARMED start must still require NavigationCore autonomy admission")
req('mode=="obstacle"&&condition!="none"&&starting' in server and
    "vehicle has passed the obstacle" in server and
    "cloud/costmap/MPPI/avoidance/recovery evidence plus goal completion" in server,
    "4.5A human must be mandatory at admission but may leave FOV after avoidance")
req("AUTONOMY START GATE BELUM READY" in js and "Autonomy Start Gate" in js,
    "4.5A GUI must expose the NavigationCore autonomy gate for ARMED motion")

# 4.5A world-fixed obstacle memory: FOV loss must not erase the obstacle until
# the vehicle has geometrically passed it in odom.
for token in ("obstacle_memory_guard_topic", "update_obstacle_memory",
              "_base_xy_to_odom", "_odom_xy_to_base",
              "obstacle_memory_release_behind_m", "obstacle_memory_points",
              "4.5A obstacle memory LATCH", "4.5A obstacle memory RELEASE"):
    req(token in semantic, f"4.5A world-fixed obstacle-memory contract missing: {token}")
for token in ("final_obstacle_rear_keep_m", "final_obstacle_trial_active",
              "planning_min_x"):
    req(token in safety, f"4.5A rear-memory planning envelope missing: {token}")
req("final_obstacle_rear_keep_m: 0.90" in safety_cfg,
    "4.5A safety config must retain remembered obstacle behind base during pass")
for token in ("obstacle_memory_latched_seen", "obstacle_memory_released_seen",
              "obstacle_memory_base_forward_m"):
    req(token in server and token in exporter,
        f"4.5A obstacle-memory evidence missing from runtime/export: {token}")
req("World Memory" in js and "LATCH → RELEASE" in js,
    "4.5A GUI must expose obstacle-memory evidence")

for token in ("function v3ExportReady45()", "dryPass&&armedComplete",
              'Lengkapi 3 obstacle trial + 3 lane trial'):
    req(token in js, f"4.5 export frontend gate missing: {token}")
req('QString("lane_trial_%1").arg(i)' in server and 'QString("obstacle_trial_%1").arg(i)' in server,
    "4.5 export backend must require obstacle/lane Trial 1..3")
req("trialKeys" in store and "obstacle_trial_1" in store and "lane_trial_1" in store,
    "4.5 group reset must remove only current three-trial data, preserving legacy state")
req('obs_keys={"obstacle_trial_1","obstacle_trial_2","obstacle_trial_3"}' in exporter and
    'lane_keys={"lane_trial_1","lane_trial_2","lane_trial_3"}' in exporter,
    "4.5 export must isolate current trial rows from legacy conditions")
req("auto complete45=" in server and "dryPass&&armedComplete" in server,
    "4.5 export backend must require DRY RUN PASS + ARMED COMPLETE")

for token in ("Expected DRY RUN:", "Expected ARMED:", "Goal 5 m",
              "Recenter 5 m", "Tidak ada syarat jarak referensi",
              "Target Awal", "Target Akhir", "RED salah satu sisi"):
    req(token in js, f"4.5 three-trial UI label missing: {token}")
req("[1,2,3].map(i=>" in js and "lane_trial_'+i" in js,
    "4.5B UI must generate Lane Trial 1..3 dynamically")
for token in ("#v3Chart{display:block;width:100%;max-width:100%",
              "overflow-wrap:anywhere", "word-break:break-word"):
    req(token in css, f"4.5 responsive containment missing: {token}")

req("perception_bab4_locked.js?v=20260926-heartbeat-worker-v50" in index,
    "4.5 JS cache-buster must match global-replan visualization revision")
for token in ("function v3Human45Detected()", "humanDetected?'TERDETEKSI':'TIDAK'",
              "DRY RUN 4.5A is planning-only"):
    req(token in js, f"4.5 visual detection/planning-only preview contract missing: {token}")
req("safety_human_valid===true" in js,
    "4.5 GUI must keep Nav2 gating on safety-qualified tracked human")
for token in ("peakEvidence", "Evidence obstacle lengkap", "safety_human_valid===true"):
    req(token in js, f"4.5A stable capture/track gate missing: {token}")
req("bestPriority" in store and "bestPriority" in server,
    "4.5A backend/session must prioritize safety-tracked human over raw confidence")
for token in ("Jarak Aktual", "stable_forward_m", "stable_distance_m"):
    req(token in js or token in store or token in server,
        f"4.5 stable-distance evidence contract missing: {token}")
for token in ("stabilize_human_distance", "person_continuity_rank",
              "human_distance_history_size", "BBOX_JUMP_REJECTED",
              "human_distance_alpha_approach", "human_distance_alpha_recede"):
    req(token in semantic, f"semantic distance stabilizer missing: {token}")
req("HUMAN LAST " in js and "v3DrawOverlay(false)" in js and
    "Date.now()-V3.visualHuman45At<=2000" in js,
    "bbox visual hold must be display-only and excluded from evidence capture")
req("perception_bab4_locked.css?v=20260922-p3-v5" in index,
    "4.5 CSS cache-buster must match P3 revision")
req("camera_geometry.js?v=20260922-preview-v3" in index,
    "Camera frame-store cache-buster must match dropout-race fix")
for token in ("this.lastFrame?.url&&img", "queueMicrotask(()=>onFrame(this.lastFrame,img))",
              "const old=this.lastUrl;this.lastUrl=url;this.lastFrame=frame",
              "for(const t of this.activeTargets())", "URL.revokeObjectURL(old),3000"):
    req(token in camera, f"last-good-frame/dropout-race contract missing: {token}")
for token in ("const keepCamera=p.querySelector('#v3CameraHost')", "freshCamera.replaceWith(keepCamera)"):
    req(token in js, f"Bab-4 camera DOM persistence contract missing: {token}")

# 4.5B Lane Safety: three repeated 5 m recenter trials; actual side is read at START.
for token in ("Recenter 5 m", "LANE SAFETY CONTROL PATH",
              "Target Awal", "Target Akhir", "RED salah satu sisi",
              "recenter dari RED sampai GREEN/GREEN", "obj('nav_cmd_mux').raw"):
    req(token in js, f"4.5B lane 5 m UI/semantic contract missing: {token}")
for token in ("RED LEFT -> recenter RIGHT", "RED RIGHT -> recenter LEFT",
              "steering_response_seen", "steering_direction_match",
              "web_trial_authority_at_ms",
              "Command RED aktif tetapi feedback steering tidak merespons >=2 deg dalam 1.5 s",
              "4.5B recenter trial hanya menerima RED pada salah satu sisi",
              'const double target=(mode=="lane")?5.0',
              'forwardProgress>=laneTarget',
              'finalMode_=="lane"?110000:45000',
              'initial_heading_rad',
              'heading_error_rad',
              'heading_restored_once',
              'HEADING_RECOVERY',
              'READY_NAV2_HANDOFF',
              'NAV2_TO_GOAL',
              'lane_nav2_handoff',
              'lane_nav2_authority_seen',
              'Nav2/MPPI mencapai goal',
              'restoring initial heading',
              'trial_index'):
    req(token in server, f"4.5B server 5 m state-machine contract missing: {token}")
req('"/system/autonomy_motion_allowed", "system.autonomy_motion_allowed"' in server and
    "Autonomy gate aktif sebelum Lane Safety menyerahkan kontrol ke Nav2" in server and
    "lane_nav2_handoff" in server and "lane_nav2_authority_seen" in server,
    "4.5B must use WEB_TRIAL before handoff and AUTONOMY after Nav2 goal")
req("Obstacle safety stream stale — Lane Safety trial diblok" not in server,
    "4.5B lane motion must not be blocked only by stale obstacle stream")
req("laneObstacleStreamFresh&&lane.laneValid" not in js,
    "4.5B GUI must not block lane motion only because obstacle stream is stale")
for token in ("lane_dropout_hold_active",
              "LANE EVIDENCE DROPOUT -> HOLD STOP, WAIT RECOVERY",
              "Lane/drivable hilang > 1.8 s saat recenter",
              "countdown_wait_lane_evidence"):
    req(token in server, f"4.5B bounded lane-dropout hold missing: {token}")
req("laneNav2Handoff" in server and
    'mode=="lane"&&!laneNav2Handoff&&!finalFresh(s,"lane_state",1500)' in server,
    "4.5B lane-state freshness must be pre-handoff only")
req("If the exact RED/GREEN sample disappeared between the browser click" in server,
    "4.5B start must tolerate a no-evidence race while remaining STOP")
for token in ("FINAL_LANE_WARNING_LEFT_NO_STEER", "FINAL_LANE_WARNING_RIGHT_NO_STEER",
              "FINAL_LANE_RED_LEFT_RECOVERY_YELLOW", "FINAL_LANE_RED_RIGHT_RECOVERY_YELLOW"):
    req(token in safety, f"trajectory safety lane semantic mirror missing: {token}")
req('"/cmd_vel/web_trial"' in server and 'web_trial_topic, cmd_qos' in router and
    'router_source_ == "WEB_TRIAL"' in esc,
    "4.5B WEB_TRIAL -> router -> ESC actuator path contract missing")
for token in ("lane_evidence_source", "lane_classical_fallback_active",
              "lane_classical_left_samples", "lane_classical_right_samples"):
    req(token in store and token in exporter,
        f"4.5B lane evidence provenance missing from research/export data: {token}")
for token in ("applyClassicalLaneFallback", "CLASSICAL_FALLBACK",
              "lane_classical_min_samples_per_side_", "lane_classical_fallback_active_",
              "DRIVABLE_EDGE_FALLBACK", "DRIVABLE_HYBRID_BOUNDARY",
              "robustMedianGap", "left_drivable_edge_gaps",
              "left_boundary_source", "right_boundary_source",
              "drivable_edge_start_y", "best_distance == 0"):
    req(token in cpu, f"4.5B lane boundary fallback contract missing: {token}")
for token in ("lane_corridor_evidence_mode", "lane_left_boundary_source",
              "lane_right_boundary_source", "lane_left_drivable_edge_samples",
              "lane_right_drivable_edge_samples"):
    req(token in store and token in exporter,
        f"4.5B lane boundary provenance missing from research/export data: {token}")
req('rr.value("integration_pass").toBool(false)' in store and
    'r.value("integration_pass").toBool(false))armedComplete=true' in server,
    "4.5 completion/export must require integration_pass=true")

# 4.5A ZIP visual evidence: camera preview + map/costmap/MPPI combined image.
for token in ("function v3Make45VisualBundle(", "MAP / COSTMAP / MPPI EVIDENCE",
              "/api/perception/bab4/visual-evidence", "armedVisual45",
              "Snapshot avoidance dibekukan",
              "map_filename", "combined_filename"):
    req(token in js or token in store or token in server, f"4.5A visual evidence contract missing: {token}")
req("save45VisualEvidence" in store and "VISUAL_EVIDENCE" in store and
    "_map.png" in store and "_combined.png" in store,
    "4.5A backend must persist map + combined evidence into session folder")
req("dryVisual" in server and "armedVisual" in server and
    "needVisual" in server,
    "4.5A ZIP readiness must require DRY + ARMED visual evidence")

# Nav2 Humble MPPI publishes one Marker per trajectory point using marker.pose,
# not marker.points. The bridge must reconstruct candidate/optimal trajectories.
for token in ("marker.pose.position", "POSE_MARKERS_HUMBLE",
              "Candidate Trajectories", "Optimal Trajectory",
              "lastCandidateGreen", "pose_marker_count", "raw_marker_count"):
    req(token in server, f"Humble MPPI pose-marker parser contract missing: {token}")

# Legacy NO-HUMAN support remains in backend for rollback compatibility, but is not part of the new 4.5 UI/export protocol.
req('NO_HUMAN_PATH_CLEAR' in store,
    "legacy NO HUMAN backend compatibility unexpectedly removed")

req("tjDecompress2" in cpu and "TJFLAG_FASTDCT" in cpu and
    "TJFLAG_FASTDCT | TJFLAG_STOPONWARNING" not in cpu,
    "MJPEG decoder must accept recoverable warnings and reject fatal decode errors")
req("find_library(TURBOJPEG_LIBRARY NAMES turbojpeg)" in pcmake and
    "${TURBOJPEG_LIBRARY}" in pcmake,
    "TurboJPEG build/link contract missing")

# 4.5 dynamic obstacle -> global costmap -> SMAC replanning + visible old/new plan.
global_cfg = nav2.split("global_costmap:", 1)[1]
for token in ("update_frequency: 2.0", "publish_frequency: 2.0",
              "- obstacle_layer", "observation_sources: planning_points semantic_human_clearing",
              "topic: /perception/planning_relevant_points",
              "topic: /perception/semantic_clearing_points"):
    req(token in global_cfg, f"global dynamic replanning costmap contract missing: {token}")
req("local_plan_topic: /transformed_global_plan" in safety_cfg and
    "/controller_server/transformed_global_plan" not in safety_cfg,
    "trajectory safety must consume live Humble controller /transformed_global_plan")
req('pathSubscribe("/transformed_global_plan", "local_path")' in server and
    'pathSubscribe("/controller_server/transformed_global_plan", "local_path")' not in server and
    'pathSubscribe("/local_plan", "local_path")' not in server,
    "web GUI local path must use the live controller topic only")
for token in ("navPathGeometryChanged", "capturePreviousNavPath",
              "previous_nav_path", "nav_replan"):
    req(token in core_js or token in navigation_js or token in js,
        f"old/new planner visualization contract missing: {token}")
req("pathDraw('previous_nav_path'" in navigation_js and
    "path('previous_nav_path'" in js and "• replan ${+obj('nav_replan').count||0}" in js,
    "navigation and 4.5 maps must render previous planner path plus replan count")
req("core.js?v=20260923-global-replan-v2" in index and
    "navigation.js?v=20260923-global-replan-v2" in index and
    "perception_bab4_locked.js?v=20260926-heartbeat-worker-v50" in index,
    "planner visualization cache-busters not updated")

print("PASS perception_bab4_45_self_check")
print("4.5 THREE-TRIAL: obstacle goal-5m x3 + lane recenter+heading-5m x3 + safety/transactional/visual evidence contracts")
