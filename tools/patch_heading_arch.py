from pathlib import Path
import shutil, time
p=Path('/home/sirobo/agv/src/navigation/src/mag_heading_fusion_node.cpp')
s=p.read_text(); ts=time.strftime('%Y%m%d_%H%M%S'); shutil.copy2(p,str(p)+'.bak_heading_arch2_'+ts)
def rep(a,b):
    global s
    if a not in s: raise SystemExit('MISSING:'+a[:100])
    s=s.replace(a,b,1)
rep('    declare_parameter<double>("neo3_mag_yaw_offset_rad", 1.5707963268);\n',
    '    declare_parameter<double>("neo3_mag_yaw_offset_rad", 1.5707963268);\n    declare_parameter<double>("neo3_heading_alignment_rad", 0.0);\n    declare_parameter<bool>("enable_neo3_inertial_stationary_correction", true);\n    declare_parameter<double>("neo3_inertial_correction_max_rate_dps", 0.10);\n    declare_parameter<double>("neo3_inertial_correction_max_error_rad", 0.2617993878);\n')
rep('    neo_offset_ = get_parameter("neo3_mag_yaw_offset_rad").as_double();\n',
    '    neo_offset_ = get_parameter("neo3_mag_yaw_offset_rad").as_double();\n    neo_alignment_rad_ = get_parameter("neo3_heading_alignment_rad").as_double();\n    enable_neo_inertial_stationary_correction_ = get_parameter("enable_neo3_inertial_stationary_correction").as_bool();\n    neo_inertial_correction_max_rate_rps_ = std::max(0.0, get_parameter("neo3_inertial_correction_max_rate_dps").as_double()) * kPi / 180.0;\n    neo_inertial_correction_max_error_rad_ = std::clamp(get_parameter("neo3_inertial_correction_max_error_rad").as_double(), 0.01, kPi);\n')
rep('    const bool stationary = std::isfinite(wz) && std::isfinite(an) &&\n      std::abs(wz) <= 0.03 && std::abs(an - 9.80665) <= 0.15;\n',
    '    const bool stationary = std::isfinite(wz) && std::isfinite(an) &&\n      std::abs(wz) <= 0.03 && std::abs(an - 9.80665) <= 0.15;\n    imu_stationary_ = stationary;\n')
rep('    if (source == Source::NEO3) yaw_enu = applyNeoHeadingLut(yaw_enu);\n',
    '    if (source == Source::NEO3) yaw_enu = normalizeAngle(applyNeoHeadingLut(yaw_enu) + neo_alignment_rad_);\n')
rep('    // No validated absolute magnetic heading may reach the validated-fusion\n    // output until the field campaign is explicitly certified.\n',
    '    // During stationary periods, let calibrated NEO3 Pro slowly trim the inertial predictor.\n    // This keeps long-term gyro drift bounded without letting magnetic disturbances steer motion.\n    if (enable_neo_inertial_stationary_correction_ && inertial_fresh && neo_fresh && imu_stationary_) {\n      const double err = normalizeAngle(neo_state_.heading_rad - inertial_heading_rad_);\n      if (std::abs(err) <= neo_inertial_correction_max_error_rad_) {\n        if (last_correction_time_.nanoseconds() == 0) last_correction_time_ = t;\n        const double dt = std::clamp((t - last_correction_time_).seconds(), 0.0, 0.25);\n        const double max_step = neo_inertial_correction_max_rate_rps_ * dt;\n        inertial_heading_rad_ = normalizeAngle(inertial_heading_rad_ + std::clamp(err, -max_step, max_step));\n        last_correction_time_ = t;\n      }\n    }\n\n    // No validated absolute magnetic heading may reach the validated-fusion\n    // output until the field campaign is explicitly certified.\n')
rep('  double imu_sign_{-1.0}, imu_offset_{1.5451826276}, neo_sign_{-1.0}, neo_offset_{1.5707963268};\n',
    '  double imu_sign_{-1.0}, imu_offset_{1.5451826276}, neo_sign_{-1.0}, neo_offset_{1.5707963268};\n  double neo_alignment_rad_{0.0};\n  bool enable_neo_inertial_stationary_correction_{true};\n  double neo_inertial_correction_max_rate_rps_{0.001745329252};\n  double neo_inertial_correction_max_error_rad_{0.2617993878};\n')
rep('  bool have_tilt_{false}, have_map_yaw_{false}, have_inertial_heading_{false}, consensus_valid_{false};\n',
    '  bool have_tilt_{false}, have_map_yaw_{false}, have_inertial_heading_{false}, consensus_valid_{false};\n  bool imu_stationary_{false};\n')
p.write_text(s)
print('PATCH_OK',str(p)+'.bak_heading_arch2_'+ts)
