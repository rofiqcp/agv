#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
nav = ROOT / "src/navigation"
esc = ROOT / "src/esc"
obs = (nav / "src/vehicle_dynamics_observer.cpp").read_text()
ack = (esc / "src/ackermann_controller_server.cpp").read_text()
launch = (nav / "launch/autonomous.launch.py").read_text()
web = (nav / "web/web_server.cpp").read_text()
app = (nav / "web/static/app.js").read_text()
index = (nav / "web/static/index.html").read_text()

def require(cond, message):
    if not cond:
        raise AssertionError(message)

require("/vehicle/twist_fused" in obs, "fused twist output missing")
require("/precision/dynamics_status" in obs, "precision status missing")
require("steering_offset_rad_" in obs and "offset_gain_" in obs, "offset estimator missing")
require("steering_predicted_rad_" in obs, "steering predictor missing")
require("applyYawRateFeedback" in ack, "yaw-rate feedback loop missing")
require("integration_drives_further" in ack, "PI anti-windup missing")
require("selected.nav2" in ack and "yaw_rate_feedback_min_speed_mps_" in ack, "NAV2/speed gating missing")
require("vehicle_dynamics_observer" in launch, "precision observer not launched")
require("precision_dynamics_status" in web and "yaw_rate_feedback_status" in web, "web subscriptions missing")
require("precisionDynamicsStatus" in app and "yawFeedbackStatus" in app, "web renderer missing")
require('id="precisionChip"' in index and 'id="yawFeedbackChip"' in index, "precision UI cards missing")
require("vesc_runtime_tick_" not in ack, "dead vesc_runtime_tick_ still present")
print("PASS: PX4/Autoware precision upgrade wiring")
