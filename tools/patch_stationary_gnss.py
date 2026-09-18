from pathlib import Path
import shutil, datetime, yaml
ROOT=Path(__file__).resolve().parents[1]
p=ROOT/'src/navigation/src/localization_core.cpp'
ts=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
backup=p.with_name(p.name+f'.bak_stationary_gnss_{ts}')
shutil.copy2(p, backup)
s=p.read_text()
def rep(old,new):
    global s
    if old not in s:
        raise RuntimeError('pattern not found: '+old[:80])
    s=s.replace(old,new,1)
rep('    declare_parameter<double>("strict_correction_alpha", 0.20);\n    declare_parameter<bool>("freeze_stationary_map_translation", true);\n    declare_parameter<double>("strict_moving_correction_alpha", 0.03);',
    '    declare_parameter<double>("strict_correction_alpha", 0.20);\n    declare_parameter<bool>("freeze_stationary_map_translation", true);\n    declare_parameter<bool>("inflate_stationary_gnss_covariance", true);\n    declare_parameter<double>("stationary_gnss_position_variance_m2", 25.0);\n    declare_parameter<double>("strict_moving_correction_alpha", 0.03);')
rep('    freeze_stationary_map_translation_ =\n      get_parameter("freeze_stationary_map_translation").as_bool();\n    strict_moving_correction_alpha_ = std::clamp(',
    '    freeze_stationary_map_translation_ =\n      get_parameter("freeze_stationary_map_translation").as_bool();\n    inflate_stationary_gnss_covariance_ =\n      get_parameter("inflate_stationary_gnss_covariance").as_bool();\n    stationary_gnss_position_variance_m2_ = std::max(\n      0.01, get_parameter("stationary_gnss_position_variance_m2").as_double());\n    strict_moving_correction_alpha_ = std::clamp(')
rep('''    if (fix.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      odom.pose.covariance[0] = fix.position_covariance[0];
      odom.pose.covariance[7] = fix.position_covariance[4];
      odom.pose.covariance[35] = 0.10;
    } else {
      odom.pose.covariance[0] = 400.0;
      odom.pose.covariance[7] = 400.0;
      odom.pose.covariance[35] = 0.25;
    }
    gnss_map_odom_pub_->publish(odom);''', '''    if (fix.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      odom.pose.covariance[0] = fix.position_covariance[0];
      odom.pose.covariance[7] = fix.position_covariance[4];
      odom.pose.covariance[35] = 0.10;
    } else {
      odom.pose.covariance[0] = 400.0;
      odom.pose.covariance[7] = 400.0;
      odom.pose.covariance[35] = 0.25;
    }
    if (inflate_stationary_gnss_covariance_ && vehicleStationaryUnlocked()) {
      odom.pose.covariance[0] = std::max(odom.pose.covariance[0], stationary_gnss_position_variance_m2_);
      odom.pose.covariance[7] = std::max(odom.pose.covariance[7], stationary_gnss_position_variance_m2_);
    }
    gnss_map_odom_pub_->publish(odom);''')
rep('''  double strict_correction_alpha_{0.20};
  bool freeze_stationary_map_translation_{true};
  double strict_moving_correction_alpha_{0.03};''', '''  double strict_correction_alpha_{0.20};
  bool freeze_stationary_map_translation_{true};
  bool inflate_stationary_gnss_covariance_{true};
  double stationary_gnss_position_variance_m2_{25.0};
  double strict_moving_correction_alpha_{0.03};''')
p.write_text(s)
cfg=ROOT/'src/navigation/config/localization_cpp.yaml'
d=yaml.safe_load(cfg.read_text())
r=d['localization_core']['ros__parameters']
r['use_global_ekf_yaw_for_map_correction']=False
r['inflate_stationary_gnss_covariance']=True
r['stationary_gnss_position_variance_m2']=25.0
cfg.write_text(yaml.safe_dump(d, sort_keys=False))
print('PATCH_OK', backup)
print('use_global_ekf_yaw_for_map_correction', r['use_global_ekf_yaw_for_map_correction'])
print('inflate_stationary_gnss_covariance', r['inflate_stationary_gnss_covariance'])
print('stationary_gnss_position_variance_m2', r['stationary_gnss_position_variance_m2'])
