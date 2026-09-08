#pragma once
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QVariant>
#include <initializer_list>
#include <utility>
// Each leaf = one report subsection (subjudul). A single leaf may carry multiple
// report tables (Tabel 1..N) and multiple graphs (Grafik 1..N) — the GUI exposes
// them through small selectors in the right panel. No artificial `B`/`C` suffixes
// are emitted: "4.1.1" is one leaf with two tables / two graphs.
//
// Graph titles render as "Format mengacu Gambar 4.xx — Data Aktual GUI" and never
// as estimated-data labels. The catalog only carries the live ROS topic bindings
// used to build the actual GUI panels; no estimated values are stored here.

// Parameter field metadata for dynamic ExperimentParameterPanel.
struct ExperimentParameterField {
  QString key;                // "sample_rate", "frequency", "gt_x", etc
  QString label;              // "Sample Rate (Hz)", "Ground Truth X", etc
  QString kind;               // "float", "int", "string", "yaml_readonly"
  QString yamlFileKey;        // "gnss", "ekf_local", "nav2" (maps to YamlStore)
  QString yamlPath;           // "data_cuav_node.ros__parameters.navigation_rate_hz"
  QString placeholder;        // hint text
  QString lockedValue;        // for locked fields
  bool isGroundTruth = false; // yellow styling
  bool locked = false;        // read-only + locked label
  QString group;              // optional staged subgroup inside the tuning leaf
  // Compact constructor for initializer-list init (trailing defaults)
  ExperimentParameterField(QString k = {}, QString lbl = {}, QString knd = {},
    QString yf = {}, QString yp = {}, QString ph = {}, QString lv = {},
    bool gt = false, bool lck = false, QString grp = {})
  : key(k), label(lbl), kind(knd), yamlFileKey(yf), yamlPath(yp),
    placeholder(ph), lockedValue(lv), isGroundTruth(gt), locked(lck), group(grp) {}
};

// Optional per-graph rendering spec (for multi-series or scatter graphs).
struct ExperimentGraphSpec {
  QString type;               // "time_series" | "scatter"
  QStringList series;         // for time_series: keys into liveSeries
  QString xSeries;            // for scatter: telemetry key for X
  QString ySeries;            // for scatter: telemetry key for Y
  QString xLabel;             // optional display label; time-series defaults Time [s]
  QString yLabel;             // optional engineering-unit label
  ExperimentGraphSpec(QString t = {}, QStringList s = {}, QString x = {}, QString y = {},
    QString xl = {}, QString yl = {})
  : type(t), series(s), xSeries(x), ySeries(y), xLabel(xl), yLabel(yl) {}
};

struct ExperimentSpec {
  QString subsystem;
  // navigation | perception | steering
  QString groupId;
  // "4.1"
  QString groupTitle;
  // "4.1 Pengujian Sensor"
  QString id;
  // "4.1.1"  (no fake suffix)
  QString section;
  // "4.1.1 Pengujian GNSS"
  QStringList tableNames;
  // "Tabel 1", "Tabel 2", ...
  QVector<QStringList> tableColumns;
  // columns per table, report order preserved
  QStringList graphCaptions;
  // "Grafik N" captions (0..k)
  QMap<QString, QString> liveSeries;
  // Parameter fields for dynamic panel (empty = auto-generate from liveSeries)
  QVector<ExperimentParameterField> parameterFields;
  // Optional graph render specs (empty = default time-series of all liveSeries)
  QVector<ExperimentGraphSpec> graphs;
};
inline QVector<ExperimentSpec> buildExperimentCatalog(const QString &subsystem) {
  QVector<ExperimentSpec> out;
  auto add = [&](const char *subsys, const QString &groupId, const QString &groupTitle,
  const char *id, const QString &section,
  std::initializer_list<const char *> graphs,
  std::initializer_list<std::initializer_list<const char *>> tblColumns,
  std::initializer_list<std::pair<const char *, const char *>> series,
  QVector<ExperimentParameterField> params = QVector<ExperimentParameterField>(),
  QVector<ExperimentGraphSpec> graphSpecs = QVector<ExperimentGraphSpec>()) {
    if (subsystem != QString::fromLatin1(subsys)) return;
    ExperimentSpec spec;
    spec.subsystem = subsystem;
    spec.groupId = groupId;
    spec.groupTitle = groupTitle;
    spec.id = QString::fromUtf8(id);
    spec.section = section;
    for (const char *g : graphs) spec.graphCaptions << QString::fromUtf8(g);
    for (const auto &cols : tblColumns) {
      spec.tableNames << QStringLiteral("Tabel %1").arg(spec.tableColumns.size() + 1);
      QStringList list;
      for (const char *c : cols) list << QString::fromUtf8(c);
      spec.tableColumns << list;
    }
    for (const auto &item : series)
    spec.liveSeries[QString::fromUtf8(item.first)] = QString::fromUtf8(item.second);
    for (const ExperimentParameterField &p : params) spec.parameterFields << p;
    for (const ExperimentGraphSpec &g : graphSpecs) spec.graphs << g;
    out << spec;
  };
  // clang-format off
  // STAGED AUTONOMOUS COMMISSIONING WORKBENCH (N0..N17).
  // This is intentionally broader than the thesis chapter.  The order follows
  // physical dependencies: timing -> geometry -> actuators -> sensors -> EKF ->
  // localization -> planning -> control -> safety -> end-to-end certification.
  if (subsystem == QStringLiteral("navigation")) {
    auto F=[](const QString&key,const QString&label,const QString&kind,const QString&file,const QString&path,
              const QString&placeholder,const QString&group=QString(),bool locked=false){
      return ExperimentParameterField(key,label,kind,file,path,placeholder,QString(),false,locked,group);
    };
    auto RO=[&](const QString&key,const QString&label,const QString&file,const QString&path,const QString&group=QString()){
      return F(key,label,QStringLiteral("yaml_readonly"),file,path,QString(),group,false);
    };
    auto LOCK=[&](const QString&key,const QString&label,const QString&file,const QString&path,const QString&group){
      return F(key,label,QStringLiteral("yaml_readonly"),file,path,QString(),group,true);
    };
    auto P=[&](std::initializer_list<ExperimentParameterField> extras){
      QVector<ExperimentParameterField> v;
      v << ExperimentParameterField(QStringLiteral("sample_rate"),QStringLiteral("Sample Rate CSV (Hz)"),QStringLiteral("float"),{}, {},QStringLiteral("10.0"))
        << ExperimentParameterField(QStringLiteral("duration"),QStringLiteral("Durasi Run (s)"),QStringLiteral("float"),{}, {},QStringLiteral("30.0"))
        << ExperimentParameterField(QStringLiteral("variation"),QStringLiteral("Variasi / Run"),QStringLiteral("string"),{}, {},QStringLiteral("set-1"))
        << ExperimentParameterField(QStringLiteral("condition"),QStringLiteral("Kondisi / Skenario"),QStringLiteral("string"),{}, {},QStringLiteral("normal"));
      for(const auto&e:extras)v<<e;
      return v;
    };

    // N0 — timing/readiness must PASS before any estimator tuning.
    add("navigation",QStringLiteral("N0"),QStringLiteral("N0 System, Topic & Timing"),"N0.1",
      QStringLiteral("N0.1 Runtime Rate, Age, dan Sinkronisasi"),
      {"Rate topic utama terhadap Time [s]","Message age terhadap Time [s]","Inter-arrival time sensor terhadap Time [s]"},
      {{"Variasi","Mean Rate GNSS","Mean Rate IMU","Mean Rate ESC","Mean Rate EKF Local","Mean Rate EKF Global","Max Age GNSS","Max Age IMU","Max Age ESC"}},
      {{"Rate GNSS","derived.rate_gnss_hz"},{"Rate IMU","derived.rate_imu_hz"},{"Rate ESC","derived.rate_esc_hz"},{"Rate EKF Local","derived.rate_ekf_local_hz"},{"Rate EKF Global","derived.rate_ekf_global_hz"},
       {"Age GNSS","derived.age_gnss_s"},{"Age IMU","derived.age_imu_s"},{"Age ESC","derived.age_esc_s"},{"Age EKF Local","derived.age_ekf_local_s"},{"Age EKF Global","derived.age_ekf_global_s"},
       {"dt GNSS","derived.dt_gnss_s"},{"dt IMU","derived.dt_imu_s"},{"dt ESC","derived.dt_esc_s"}},
      P({RO("gnss_rate_cfg","GNSS navigation_rate_hz","gnss","data_cuav_node.ros__parameters.navigation_rate_hz","Target Rate"),
         RO("imu_rate_cfg","IMU publish_rate_hz","imu","data_imu_node.ros__parameters.publish_rate_hz","Target Rate"),
         RO("esc_rate_cfg","ESC command_rate_hz","esc","esc_ackermann.ros__parameters.command_rate_hz","Target Rate"),
         RO("local_ekf_rate_cfg","Local EKF frequency","ekf","ekf_filter_node_odom.ros__parameters.frequency","Target Rate"),
         RO("global_ekf_rate_cfg","Global EKF frequency","ekf","ekf_filter_node_map.ros__parameters.frequency","Target Rate")}),
      {{"time_series",{"Rate GNSS","Rate IMU","Rate ESC","Rate EKF Local","Rate EKF Global"},{},{},"Time [s]","Rate [Hz]"},
       {"time_series",{"Age GNSS","Age IMU","Age ESC","Age EKF Local","Age EKF Global"},{},{},"Time [s]","Message age [s]"},
       {"time_series",{"dt GNSS","dt IMU","dt ESC"},{},{},"Time [s]","Inter-arrival Δt [s]"}});

    // N1 — physical dimensions are measurement authority, not free tuning knobs.
    add("navigation",QStringLiteral("N1"),QStringLiteral("N1 Vehicle Geometry Authority"),"N1.1",
      QStringLiteral("N1.1 Dimensi Fisik dan Batas Kinematik"),{},
      {{"Parameter","Nilai","Status / Sumber"}}, {},
      P({LOCK("wheelbase","Wheelbase fisik (m)","vehicle","vehicle.ros__parameters.wheelbase_m","MEASURED / LOCKED"),
         LOCK("track_width","Track width (m)","vehicle","vehicle.ros__parameters.track_width_m","MEASURED / LOCKED"),
         LOCK("vehicle_width","Total vehicle width (m)","vehicle","vehicle.ros__parameters.total_width_m","MEASURED / LOCKED"),
         LOCK("vehicle_length","Vehicle length (m)","vehicle","vehicle.ros__parameters.vehicle_length_m","MEASURED / LOCKED"),
         LOCK("wheel_radius","Wheel radius (m)","vehicle","vehicle.ros__parameters.wheel_radius_m","MEASURED / LOCKED"),
         LOCK("footprint","Nav2 footprint","vehicle","vehicle.ros__parameters.footprint","MEASURED / LOCKED"),
         RO("max_fwd","Max forward speed (m/s)","vehicle","vehicle.ros__parameters.max_forward_speed_mps","SAFETY CEILING"),
         RO("max_rev","Max reverse speed (m/s)","vehicle","vehicle.ros__parameters.max_reverse_speed_mps","SAFETY CEILING") }));

    // N2 — longitudinal velocity/distance scale MUST be calibrated before steering/yaw trials.
    add("navigation",QStringLiteral("N2"),QStringLiteral("N2 Drive Odometry Calibration"),"N2.1",
      QStringLiteral("N2.1 RPM, Velocity dan Distance Scale"),
      {"RPM ESC selama trial","Velocity ESC raw/calibrated, GNSS, dan IMU diagnostic","Perpindahan GNSS X-Y","Scale candidate per trial"},
      {{"Pengujian","RPM set","Mean RPM ESC","Mean V ESC raw","Mean V GNSS","Mean V ESC calibrated","Mean V IMU","Delta X","Delta Y","Distance GNSS","Scale candidate","Mean hAcc","Samples"},
       {"Parameter","Nilai final","Status / sumber"}},
      {{"RPM ESC","vesc_right_values.rpm"},{"V ESC raw","esc_drive_raw"},{"V ESC calibrated","esc_odom.v"},
       {"V GNSS","gnss_vel.speed"},{"V IMU","imu_speed_kalman"},{"GNSS X","gnss_map_odom.x"},{"GNSS Y","gnss_map_odom.y"},
       {"hAcc","gnss_quality.hacc_m"}},
      P({F("test_erpm","RPM/eRPM ESC RIGHT untuk trial","float",{}, {},"200","VARIASI TRIAL"),
         F("trial_duration","Durasi rekomendasi per trial (s)","float",{}, {},"15","VARIASI TRIAL"),
         F("odom_scale","Drive odometry scale (live)","float","esc","esc_ackermann.ros__parameters.drive_odometry_calibration_scale","1.0","HASIL KALIBRASI"),
         RO("vehicle_scale","Vehicle scale authority","vehicle","vehicle.ros__parameters.drive_odometry_calibration_scale","PERSISTENT RESULT"),
         RO("odom_valid","Drive odometry calibration valid","vehicle","vehicle.ros__parameters.drive_odometry_calibration_valid","CERTIFICATION"),
         F("imu_speed_alpha","IMU accel LPF alpha","float","imu_speed","imu_speed_diagnostic.ros__parameters.accel_lpf_alpha","0.18","IMU SPEED DIAGNOSTIC"),
         F("imu_speed_gyro_gate","IMU speed gyro-assisted ZUPT","bool","imu_speed","imu_speed_diagnostic.ros__parameters.gyro_assisted_stationary_gate","true","IMU SPEED DIAGNOSTIC")}),
      {{"time_series",{"RPM ESC"},{},{},"Time [s]","VESC eRPM"},
       {"time_series",{"V ESC raw","V ESC calibrated","V GNSS","V IMU"},{},{},"Time [s]","Velocity [m/s]"},
       {"scatter",{},"gnss_map_odom.x","gnss_map_odom.y","Map X [m]","Map Y [m]"},
       {"time_series",{"hAcc"},{},{},"Time [s]","GNSS hAcc [m]"}});

    // N3 — steering/yaw calibration uses the already-calibrated longitudinal speed.
    add("navigation",QStringLiteral("N3"),QStringLiteral("N3 Steering & Ackermann Calibration"),"N3.1",
      QStringLiteral("N3.1 Straight Steering Zero dengan Gyro, Dual Magnetometer dan GNSS"),
      {"Steering target/actual saat lurus","Heading IMU, Yahboom MAG, NEO3 MAG, dan GNSS COG","Yaw-rate gyro dan model","Lintasan GNSS X-Y"},
      {{"Pengujian","RPM set","Mean V GNSS","Steering target","Mean Steering actual","Mean Gyro Z","Delta Yaw IMU","Delta Yaw IMU MAG","Delta Yaw NEO3 MAG","Delta COG GNSS","Delta X","Delta Y","Lateral drift","Center candidate","Samples"}},
      {{"Steer target","esc_steer_target"},{"Steer actual","esc_steer_actual"},{"Gyro Z","imu.gz"},{"Yaw IMU","imu.yaw_rad"},
       {"Yaw IMU MAG","imu_mag_heading.yaw_rad"},{"Yaw NEO3 MAG","neo3_mag_heading.yaw_rad"},{"Yaw GNSS COG","gnss_cog_fusion.yaw_rad"},
       {"V GNSS","gnss_vel.speed"},{"GNSS X","gnss_map_odom.x"},{"GNSS Y","gnss_map_odom.y"}},
      P({F("test_erpm","RPM/eRPM ESC RIGHT","float",{}, {},"200","VARIASI SPEED"),
         F("test_steering_deg","Steering test (deg)","float",{}, {},"0","LOCK STRAIGHT"),
         F("center","Feedback center command (deg)","float","esc","esc_ackermann.ros__parameters.steering_feedback_center_deg","0","CENTER CALIBRATION"),
         F("straight_deadband","Straight deadband (deg)","float","esc","esc_ackermann.ros__parameters.steering_straight_deadband_deg","1.0","CENTER CALIBRATION")}),
      {{"time_series",{"Steer target","Steer actual"},{},{},"Time [s]","Steering [rad]"},
       {"time_series",{"Yaw IMU","Yaw IMU MAG","Yaw NEO3 MAG","Yaw GNSS COG"},{},{},"Time [s]","Heading [rad]"},
       {"time_series",{"Gyro Z"},{},{},"Time [s]","Yaw-rate [rad/s]"},
       {"scatter",{},"gnss_map_odom.x","gnss_map_odom.y","Map X [m]","Map Y [m]"}});

    add("navigation",QStringLiteral("N3"),QStringLiteral("N3 Steering & Ackermann Calibration"),"N3.2",
      QStringLiteral("N3.2 Turning Steering Angle, Radius, Effective Wheelbase dan Scale"),
      {"Steering target/actual","Heading multi-sensor saat turning","Yaw-rate gyro vs Ackermann","Lintasan circle GNSS X-Y"},
      {{"Pengujian","RPM set","Steering set [deg]","Mean Steering actual","Mean V GNSS","Mean V ESC","Mean Gyro Z","Mean Yaw-rate model","Radius GNSS","Radius model","Effective wheelbase candidate","Scale steering candidate","Delta COG","Mean hAcc","Samples"},
       {"Parameter","Nilai final","Status"}},
      {{"Steer target","esc_steer_target"},{"Steer actual","esc_steer_actual"},{"V GNSS","gnss_vel.speed"},{"V ESC","esc_odom.v"},
       {"Gyro Z","imu.gz"},{"Yaw-rate model","esc_kinematic_yaw_rate"},{"Yaw IMU","imu.yaw_rad"},{"Yaw IMU MAG","imu_mag_heading.yaw_rad"},
       {"Yaw NEO3 MAG","neo3_mag_heading.yaw_rad"},{"Yaw GNSS COG","gnss_cog_fusion.yaw_rad"},{"GNSS X","gnss_map_odom.x"},{"GNSS Y","gnss_map_odom.y"}},
      P({F("test_erpm","RPM/eRPM ESC RIGHT","float",{}, {},"200","VARIASI SPEED"),
         F("test_steering_deg","Steering test (deg)","float",{}, {},"10","VARIASI STEERING"),
         RO("effective_wb","Effective wheelbase (m)","vehicle","vehicle.ros__parameters.effective_wheelbase_m","CALIBRATION RESULT"),
         LOCK("rmin","Minimum turning radius (m)","vehicle","vehicle.ros__parameters.minimum_turning_radius_m","CALIBRATION AUTHORITY"),
         RO("circle_valid","Circle calibration valid","vehicle","vehicle.ros__parameters.steering_circle_calibration_valid","CERTIFICATION")}),
      {{"time_series",{"Steer target","Steer actual"},{},{},"Time [s]","Steering [rad]"},
       {"time_series",{"Yaw IMU","Yaw IMU MAG","Yaw NEO3 MAG","Yaw GNSS COG"},{},{},"Time [s]","Heading [rad]"},
       {"time_series",{"Gyro Z","Yaw-rate model"},{},{},"Time [s]","Yaw-rate [rad/s]"},
       {"scatter",{},"gnss_map_odom.x","gnss_map_odom.y","Map X [m]","Map Y [m]"}});

    // N4 — IMU calibration and dynamic consistency.
    add("navigation",QStringLiteral("N4"),QStringLiteral("N4 IMU Calibration"),"N4.1",
      QStringLiteral("N4.1 Bias, Noise, Orientation dan Covariance IMU"),
      {"Gyro XYZ terhadap Time [s]","Acceleration XYZ terhadap Time [s]","Roll-Pitch-Yaw terhadap Time [s]","Gyro-Z stationary zoom terhadap Time [s]"},
      {{"Variasi","Mean Gyro X","Mean Gyro Y","Mean Gyro Z","Std Gyro Z"},{"Variasi","Mean Roll","Mean Pitch","Mean Yaw","Std Yaw"}},
      {{"Gyro X","imu.gx"},{"Gyro Y","imu.gy"},{"Gyro Z","imu.gz"},{"Accel X","imu.ax"},{"Accel Y","imu.ay"},{"Accel Z","imu.az"},
       {"Roll","imu.roll_rad"},{"Pitch","imu.pitch_rad"},{"Yaw","imu.yaw_rad"}},
      P({F("yaw_offset","Yaw offset (rad)","float","imu","data_imu_node.ros__parameters.yaw_offset_rad","0.0","Orientation Calibration"),
         F("gyro_bias_z","Gyro bias Z","float","imu","data_imu_node.ros__parameters.gyro_bias.2","0.0","Stationary Bias"),
         F("r_imu_yaw","R IMU yaw variance","float","imu","data_imu_node.ros__parameters.orientation_covariance.2","0.05","Measurement Covariance R"),
         F("r_imu_w","R IMU gyro-Z variance","float","imu","data_imu_node.ros__parameters.angular_velocity_covariance.2","0.02","Measurement Covariance R"),
         F("gyro_timeout","Gyro packet timeout (s)","float","imu","data_imu_node.ros__parameters.gyro_packet_timeout_sec","0.35","Timing")}),
      {{"time_series",{"Gyro X","Gyro Y","Gyro Z"},{},{},"Time [s]","Angular velocity [rad/s]"},
       {"time_series",{"Accel X","Accel Y","Accel Z"},{},{},"Time [s]","Acceleration [m/s²]"},
       {"time_series",{"Roll","Pitch","Yaw"},{},{},"Time [s]","Orientation [rad]"},
       {"time_series",{"Gyro Z"},{},{},"Time [s]","Gyro-Z [rad/s]"}});

    add("navigation",QStringLiteral("N4"),QStringLiteral("N4 IMU Calibration"),"N4.2",
      QStringLiteral("N4.2 Dynamic Yaw Consistency IMU vs Ackermann"),
      {"IMU gyro-Z vs kinematic yaw-rate","Heading IMU during maneuver"},
      {{"Variasi","RMSE yaw-rate","Mean Gyro Z","Mean Yaw-rate kinematic","Max residual"}},
      {{"IMU wz","imu.gz"},{"Ackermann wz","esc_kinematic_yaw_rate"},{"Yaw IMU","imu.yaw_rad"}}, P({}),
      {{"time_series",{"IMU wz","Ackermann wz"},{},{},"Time [s]","Yaw-rate [rad/s]"},
       {"time_series",{"Yaw IMU"},{},{},"Time [s]","Yaw [rad]"}});

    // N5 — GNSS quality first, motion/COG second.
    add("navigation",QStringLiteral("N5"),QStringLiteral("N5 GNSS Qualification"),"N5.1",
      QStringLiteral("N5.1 Quality, DOP, Position Stability dan Timing"),
      {"Sebaran posisi GNSS statis ΔEast-ΔNorth","HDOP, VDOP dan DOP terhadap Time [s]","Horizontal accuracy hAcc terhadap Time [s]","Jumlah satelit terhadap Time [s]"},
      {{"Variasi","Mean Satelit","Mean DOP","Mean HDOP","Mean VDOP","Mean hAcc","Mean Rate GNSS","Max Age GNSS"},{"Variasi","Std East [m]","Std North [m]","2D RMS [m]","R95 [m]","Samples"}},
      {{"Satelit","gnss_quality.sat"},{"DOP","gnss_quality.dop"},{"HDOP","gnss_quality.hdop"},{"VDOP","gnss_quality.vdop"},{"hAcc","gnss_quality.hacc_m"},
       {"Rate GNSS","derived.rate_gnss_hz"},{"Age GNSS","derived.age_gnss_s"}},
      P({F("nav_rate","Navigation rate (Hz)","float","gnss","data_cuav_node.ros__parameters.navigation_rate_hz","10","Receiver Timing"),
         F("min_sat","Minimum satellites","int","gnss","data_cuav_node.ros__parameters.min_satellites","8","Quality Gate"),
         F("max_dop","Maximum DOP","float","gnss","data_cuav_node.ros__parameters.max_dop","2.0","Quality Gate"),
         F("max_hacc","Maximum hAcc (m)","float","gnss","data_cuav_node.ros__parameters.max_hacc_m","2.5","Quality Gate"),
         F("max_sacc","Maximum sAcc (m/s)","float","gnss","data_cuav_node.ros__parameters.max_sacc_mps","0.8","Quality Gate"),
         F("fit_window","Position fit window (s)","float","gnss","data_cuav_node.ros__parameters.position_fit_window_sec","3.0","Motion Fit")}),
      {{"scatter",{},"gnss_fix.lon","gnss_fix.lat","ΔEast [m]","ΔNorth [m]"},
       {"time_series",{"HDOP","VDOP","DOP"},{},{},"Time [s]","DOP [-]"},
       {"time_series",{"hAcc"},{},{},"Time [s]","Horizontal accuracy [m]"},
       {"time_series",{"Satelit"},{},{},"Time [s]","Satellites [-]"}});

    add("navigation",QStringLiteral("N5"),QStringLiteral("N5 GNSS Qualification"),"N5.2",
      QStringLiteral("N5.2 Velocity, COG, Synchronization dan Fusion Qualification"),
      {"GNSS velocity vs wheel velocity","COG yaw terhadap Time [s]","GNSS velocity covariance"},
      {{"Variasi","RMSE V GNSS-vs-ESC","Mean V GNSS","Mean V ESC","Mean COG variance"},{"Parameter","Nilai","Gate"}},
      {{"V GNSS","gnss_base_vel_fusion.vx"},{"V ESC","esc_odom.v"},{"COG yaw","gnss_cog_fusion.yaw_rad"},{"R GNSS vx","gnss_base_vel_fusion.cov_x"},{"R COG yaw","gnss_cog_fusion.yaw_variance"}},
      P({F("sync_gap","GNSS sync max gap (s)","float","localization","localization_core.ros__parameters.gnss_sync_max_gap_sec","0.3","Synchronization"),
         F("speed_consistency","Wheel/GNSS speed residual max (m/s)","float","localization","localization_core.ros__parameters.gnss_speed_consistency_max_mps","0.2","Velocity Qualification"),
         F("cog_min_speed","COG min forward speed (m/s)","float","localization","localization_core.ros__parameters.cog_min_forward_speed_mps","0.35","COG Qualification"),
         F("cog_max_sacc","COG max sAcc (m/s)","float","localization","localization_core.ros__parameters.cog_max_sacc_mps","0.3","COG Qualification"),
         F("r_gnss_v_min","R GNSS vx min variance","float","localization","localization_core.ros__parameters.gnss_velocity_fusion_min_variance","0.0025","Measurement Covariance R"),
         F("r_gnss_v_max","R GNSS vx max variance","float","localization","localization_core.ros__parameters.gnss_velocity_fusion_max_variance","0.25","Measurement Covariance R"),
         F("r_cog_min","R COG yaw min variance","float","localization","localization_core.ros__parameters.gnss_cog_fusion_min_variance_rad2","0.001218","Measurement Covariance R"),
         F("r_cog_max","R COG yaw max variance","float","localization","localization_core.ros__parameters.gnss_cog_fusion_max_variance_rad2","0.274","Measurement Covariance R")}),
      {{"time_series",{"V GNSS","V ESC"},{},{},"Time [s]","Velocity [m/s]"},
       {"time_series",{"COG yaw"},{},{},"Time [s]","Yaw [rad]"},
       {"time_series",{"R GNSS vx"},{},{},"Time [s]","Variance [(m/s)^2]"}});

    // N6 — map reference / lever arm / alignment.
    add("navigation",QStringLiteral("N6"),QStringLiteral("N6 TF & Map Alignment"),"N6.1",
      QStringLiteral("N6.1 ENU Reference, Lever Arm dan Multi-point Map Calibration"),
      {"GNSS map trajectory","Global EKF map trajectory"},
      {{"Variasi","Mean GNSS map X","Mean GNSS map Y","Mean Global X","Mean Global Y"},{"Parameter","Nilai","Peran"}},
      {{"GNSS map X","gnss_map_odom.x"},{"GNSS map Y","gnss_map_odom.y"},{"Global X","ekf_global.x"},{"Global Y","ekf_global.y"}},
      P({F("reference_lat","Reference latitude","float","localization","localization_core.ros__parameters.reference_latitude","-7.0","Map Reference"),
         F("reference_lon","Reference longitude","float","localization","localization_core.ros__parameters.reference_longitude","110.0","Map Reference"),
         F("reference_x","Reference map X (m)","float","localization","localization_core.ros__parameters.reference_map_x_m","0","Map Reference"),
         F("reference_y","Reference map Y (m)","float","localization","localization_core.ros__parameters.reference_map_y_m","0","Map Reference"),
         F("map_yaw","Map yaw from ENU (rad)","float","localization","localization_core.ros__parameters.map_yaw_from_enu_rad","0","Map Reference"),
         F("antenna_x","GNSS antenna X lever arm (m)","float","localization","localization_core.ros__parameters.gnss_antenna_x_m","0.165","Lever Arm"),
         F("antenna_y","GNSS antenna Y lever arm (m)","float","localization","localization_core.ros__parameters.gnss_antenna_y_m","0","Lever Arm"),
         F("cal_rmse","Map calibration max RMSE (m)","float","localization","localization_core.ros__parameters.map_calibration_max_rmse_m","3.0","Multi-point Calibration")}),
      {{"scatter",{},"gnss_map_odom.x","gnss_map_odom.y","Map X [m]","Map Y [m]"},
       {"scatter",{},"ekf_global.x","ekf_global.y","Map X [m]","Map Y [m]"}});

    // N7 — LOCAL EKF CORE. P/Q/R/K/residuals are deliberately explicit.
    add("navigation",QStringLiteral("N7"),QStringLiteral("N7 Local EKF — Core Tuning"),"N7.1",
      QStringLiteral("N7.1 Fusion Architecture, Timing, Queue dan Rejection"),
      {"Local EKF velocity sources","Local EKF yaw-rate sources","Local EKF output age"},
      {{"Variasi","Frequency","Sensor timeout","Reject ESC","Reject GNSS","RMSE Residual vx ESC","RMSE Residual vx GNSS"},{"Source","Config state","Role"}},
      {{"ESC vx","esc_odom.v"},{"GNSS vx","gnss_base_vel_fusion.vx"},{"EKF vx","ekf_local.v"},{"ESC wz","esc_odom.w"},{"IMU wz","imu.gz"},{"EKF wz","ekf_local.w"},{"EKF age","derived.age_ekf_local_s"},
       {"Residual vx ESC","derived.ekf_local_res_vx_esc"},{"Residual vx GNSS","derived.ekf_local_res_vx_gnss"}},
      P({F("frequency","Frequency [Hz]","float","ekf","ekf_filter_node_odom.ros__parameters.frequency","10","Timing"),
         F("sensor_timeout","Sensor timeout [s]","float","ekf","ekf_filter_node_odom.ros__parameters.sensor_timeout","0.25","Timing"),
         F("predict","Predict to current time","bool","ekf","ekf_filter_node_odom.ros__parameters.predict_to_current_time","true","Timing"),
         F("queue_esc","ESC queue size","int","ekf","ekf_filter_node_odom.ros__parameters.odom0_queue_size","30","Buffer / Queue"),
         F("queue_imu","IMU queue size","int","ekf","ekf_filter_node_odom.ros__parameters.imu0_queue_size","50","Buffer / Queue"),
         F("queue_gnss","GNSS vx queue size","int","ekf","ekf_filter_node_odom.ros__parameters.twist0_queue_size","20","Buffer / Queue"),
         F("reject_esc","ESC twist rejection threshold","float","ekf","ekf_filter_node_odom.ros__parameters.odom0_twist_rejection_threshold","5","Rejection Gate"),
         F("reject_gnss","GNSS vx rejection threshold","float","ekf","ekf_filter_node_odom.ros__parameters.twist0_rejection_threshold","10","Rejection Gate"),
         LOCK("esc_cfg","ESC fusion matrix [vx,wz]","ekf","ekf_filter_node_odom.ros__parameters.odom0_config","ARCHITECTURE — LOCKED"),
         LOCK("imu_cfg","IMU fusion matrix [yaw,wz]","ekf","ekf_filter_node_odom.ros__parameters.imu0_config","ARCHITECTURE — LOCKED"),
         LOCK("gnss_cfg","GNSS fusion matrix [vx]","ekf","ekf_filter_node_odom.ros__parameters.twist0_config","ARCHITECTURE — LOCKED")}),
      {{"time_series",{"ESC vx","GNSS vx","EKF vx"},{},{},"Time [s]","Velocity [m/s]"},
       {"time_series",{"ESC wz","IMU wz","EKF wz"},{},{},"Time [s]","Yaw-rate [rad/s]"},
       {"time_series",{"EKF age"},{},{},"Time [s]","State age [s]"}});

    add("navigation",QStringLiteral("N7"),QStringLiteral("N7 Local EKF — Core Tuning"),"N7.2",
      QStringLiteral("N7.2 P-Q-R Local: State Uncertainty, Process Noise, Measurement Trust"),
      {"Local P position sigma","Local P velocity sigma","Local P yaw/yaw-rate sigma","Local R velocity variance"},
      {{"Variasi","Qx","Qy","Qyaw","Qvx","Qwz","Mean sigma X","Mean sigma Y","Mean sigma Yaw","Mean sigma Vx","Mean sigma Wz"},
       {"Variasi","R ESC vx","R GNSS vx","R IMU yaw","R ESC wz","R IMU wz"}},
      {{"sigma X","derived.ekf_local_sigma_x"},{"sigma Y","derived.ekf_local_sigma_y"},{"sigma Yaw","derived.ekf_local_sigma_yaw"},{"sigma Vx","derived.ekf_local_sigma_vx"},{"sigma Wz","derived.ekf_local_sigma_w"},
       {"R ESC vx","esc_odom.var_v"},{"R GNSS vx","gnss_base_vel_fusion.cov_x"},{"R IMU yaw","imu.var_yaw"},{"R ESC wz","esc_odom.var_w"},{"R IMU wz","imu.var_gz"}},
      P({F("qx","Qx","float","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.0","0.05","Q — Process Noise"),
         F("qy","Qy","float","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.16","0.05","Q — Process Noise"),
         F("qyaw","Qyaw","float","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.80","0.06","Q — Process Noise"),
         F("qvx","Qvx","float","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.96","0.025","Q — Process Noise"),
         F("qwz","Qwz","float","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.176","0.02","Q — Process Noise"),
         F("r_esc_v","R ESC vx base","float","esc","esc_ackermann.ros__parameters.odom_v_variance_base","0.03","R — Measurement Noise"),
         F("r_esc_w","R ESC yaw-rate base","float","esc","esc_ackermann.ros__parameters.odom_yaw_rate_variance_base","0.05","R — Measurement Noise"),
         F("r_imu_yaw","R IMU yaw","float","imu","data_imu_node.ros__parameters.orientation_covariance.2","0.05","R — Measurement Noise"),
         F("r_imu_w","R IMU gyro-Z","float","imu","data_imu_node.ros__parameters.angular_velocity_covariance.2","0.02","R — Measurement Noise"),
         F("r_gnss_v_min","R GNSS vx min","float","localization","localization_core.ros__parameters.gnss_velocity_fusion_min_variance","0.0025","R — Measurement Noise"),
         F("r_gnss_v_max","R GNSS vx max","float","localization","localization_core.ros__parameters.gnss_velocity_fusion_max_variance","0.25","R — Measurement Noise")}),
      {{"time_series",{"sigma X","sigma Y"},{},{},"Time [s]","σ position [m]"},
       {"time_series",{"sigma Vx"},{},{},"Time [s]","σ velocity [m/s]"},
       {"time_series",{"sigma Yaw","sigma Wz"},{},{},"Time [s]","σ angular state"},
       {"time_series",{"R ESC vx","R GNSS vx"},{},{},"Time [s]","Velocity variance [(m/s)^2]"}});

    add("navigation",QStringLiteral("N7"),QStringLiteral("N7 Local EKF — Core Tuning"),"N7.3",
      QStringLiteral("N7.3 Kalman Authority, Residual, NIS Proxy dan Drift Analyzer Local"),
      {"K_eff Local velocity","K_eff Local yaw/yaw-rate","Local velocity residual","Local angular residual","Local NIS diagnostic proxy","Local P growth / drift uncertainty"},
      {{"Variasi","Mean K vx ESC","Mean K vx GNSS","Mean K yaw IMU","Mean K wz ESC","Mean K wz IMU","P95 NIS vx ESC","P95 NIS vx GNSS"},
       {"Variasi","RMSE Residual vx ESC","RMSE Residual vx GNSS","RMSE Residual yaw IMU","RMSE Residual wz ESC","RMSE Residual wz IMU","Max P growth X","Max P growth Y"}},
      {{"K vx ESC","derived.ekf_local_k_vx_esc"},{"K vx GNSS","derived.ekf_local_k_vx_gnss"},{"K yaw IMU","derived.ekf_local_k_yaw_imu"},{"K wz ESC","derived.ekf_local_k_w_esc"},{"K wz IMU","derived.ekf_local_k_w_imu"},
       {"Residual vx ESC","derived.ekf_local_res_vx_esc"},{"Residual vx GNSS","derived.ekf_local_res_vx_gnss"},{"Residual yaw IMU","derived.ekf_local_res_yaw_imu"},{"Residual wz ESC","derived.ekf_local_res_w_esc"},{"Residual wz IMU","derived.ekf_local_res_w_imu"},
       {"NIS vx ESC","derived.ekf_local_nis_vx_esc"},{"NIS vx GNSS","derived.ekf_local_nis_vx_gnss"},{"NIS yaw IMU","derived.ekf_local_nis_yaw_imu"},{"NIS wz ESC","derived.ekf_local_nis_w_esc"},{"NIS wz IMU","derived.ekf_local_nis_w_imu"},
       {"P growth X","derived.ekf_local_p_growth_x"},{"P growth Y","derived.ekf_local_p_growth_y"},{"P growth Yaw","derived.ekf_local_p_growth_yaw"},{"P growth Vx","derived.ekf_local_p_growth_vx"},{"P growth Wz","derived.ekf_local_p_growth_w"}}, P({}),
      {{"time_series",{"K vx ESC","K vx GNSS"},{},{},"Time [s]","K_eff [0..1]"},
       {"time_series",{"K yaw IMU","K wz ESC","K wz IMU"},{},{},"Time [s]","K_eff [0..1]"},
       {"time_series",{"Residual vx ESC","Residual vx GNSS"},{},{},"Time [s]","Velocity residual [m/s]"},
       {"time_series",{"Residual yaw IMU","Residual wz ESC","Residual wz IMU"},{},{},"Time [s]","Angular residual"},
       {"time_series",{"NIS vx ESC","NIS vx GNSS","NIS yaw IMU","NIS wz ESC","NIS wz IMU"},{},{},"Time [s]","NIS proxy [-]"},
       {"time_series",{"P growth X","P growth Y","P growth Yaw","P growth Vx","P growth Wz"},{},{},"Time [s]","dP/dt"}});

    add("navigation",QStringLiteral("N7"),QStringLiteral("N7 Local EKF — Core Tuning"),"N7.4",
      QStringLiteral("N7.4 Final Validation Local EKF"),
      {"Final local velocity fusion","Final local yaw-rate fusion","Final local covariance"},
      {{"Variasi","RMSE Residual vx ESC","RMSE Residual vx GNSS","RMSE Residual wz ESC","RMSE Residual wz IMU","Mean sigma X","Mean sigma Y","Max P growth X","Max P growth Y"}},
      {{"ESC vx","esc_odom.v"},{"GNSS vx","gnss_base_vel_fusion.vx"},{"EKF vx","ekf_local.v"},{"ESC wz","esc_odom.w"},{"IMU wz","imu.gz"},{"EKF wz","ekf_local.w"},{"sigma X","derived.ekf_local_sigma_x"},{"sigma Y","derived.ekf_local_sigma_y"},{"P growth X","derived.ekf_local_p_growth_x"},{"P growth Y","derived.ekf_local_p_growth_y"}},
      P({RO("frequency","Final Frequency","ekf","ekf_filter_node_odom.ros__parameters.frequency","FROZEN CONFIG"),RO("timeout","Final sensor timeout","ekf","ekf_filter_node_odom.ros__parameters.sensor_timeout","FROZEN CONFIG"),RO("qvx","Final Qvx","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.96","FROZEN CONFIG"),RO("qwz","Final Qwz","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.176","FROZEN CONFIG")}),
      {{"time_series",{"ESC vx","GNSS vx","EKF vx"},{},{},"Time [s]","Velocity [m/s]"},
       {"time_series",{"ESC wz","IMU wz","EKF wz"},{},{},"Time [s]","Yaw-rate [rad/s]"},
       {"time_series",{"sigma X","sigma Y"},{},{},"Time [s]","σ position [m]"}});

    // N8 — GLOBAL EKF CORE.
    add("navigation",QStringLiteral("N8"),QStringLiteral("N8 Global EKF — Core Tuning"),"N8.1",
      QStringLiteral("N8.1 Fusion Architecture, Timing, Queue dan Rejection"),
      {"Global GNSS position vs EKF","Global velocity fusion","Global heading sources"},
      {{"Variasi","Frequency","Sensor timeout","Pose reject","Twist reject","RMSE Residual x GNSS","RMSE Residual y GNSS"}},
      {{"GNSS X","gnss_map_odom.x"},{"GNSS Y","gnss_map_odom.y"},{"Global X","ekf_global.x"},{"Global Y","ekf_global.y"},{"GNSS vx","gnss_base_vel_fusion.vx"},{"Global vx","ekf_global.v"},{"COG yaw","gnss_cog_fusion.yaw_rad"},{"Global yaw","ekf_global.yaw"}},
      P({F("frequency","Frequency [Hz]","float","ekf","ekf_filter_node_map.ros__parameters.frequency","10","Timing"),
         F("sensor_timeout","Sensor timeout [s]","float","ekf","ekf_filter_node_map.ros__parameters.sensor_timeout","2.0","Timing"),
         F("predict","Predict to current time","bool","ekf","ekf_filter_node_map.ros__parameters.predict_to_current_time","false","Timing"),
         F("pose_reject","GNSS pose rejection threshold","float","ekf","ekf_filter_node_map.ros__parameters.odom0_pose_rejection_threshold","12","Rejection Gate"),
         F("twist_reject","GNSS vx rejection threshold","float","ekf","ekf_filter_node_map.ros__parameters.twist0_rejection_threshold","10","Rejection Gate"),
         F("queue_xy","GNSS XY queue","int","ekf","ekf_filter_node_map.ros__parameters.odom0_queue_size","8","Buffer / Queue"),
         F("queue_v","GNSS vx queue","int","ekf","ekf_filter_node_map.ros__parameters.twist0_queue_size","12","Buffer / Queue"),
         F("queue_cog","COG queue","int","ekf","ekf_filter_node_map.ros__parameters.pose0_queue_size","10","Buffer / Queue"),
         F("queue_imu","IMU queue","int","ekf","ekf_filter_node_map.ros__parameters.imu0_queue_size","30","Buffer / Queue"),
         LOCK("xy_cfg","GNSS map fusion [x,y]","ekf","ekf_filter_node_map.ros__parameters.odom0_config","ARCHITECTURE — LOCKED"),
         LOCK("v_cfg","GNSS velocity fusion [vx]","ekf","ekf_filter_node_map.ros__parameters.twist0_config","ARCHITECTURE — LOCKED"),
         LOCK("cog_cfg","COG fusion [yaw]","ekf","ekf_filter_node_map.ros__parameters.pose0_config","ARCHITECTURE — LOCKED"),
         LOCK("imu_cfg","IMU fusion [wz only]","ekf","ekf_filter_node_map.ros__parameters.imu0_config","ARCHITECTURE — LOCKED")}),
      {{"scatter",{},"gnss_map_odom.x","gnss_map_odom.y","Map X [m]","Map Y [m]"},
       {"time_series",{"GNSS vx","Global vx"},{},{},"Time [s]","Velocity [m/s]"},
       {"time_series",{"COG yaw","Global yaw"},{},{},"Time [s]","Yaw [rad]"}});

    add("navigation",QStringLiteral("N8"),QStringLiteral("N8 Global EKF — Core Tuning"),"N8.2",
      QStringLiteral("N8.2 P-Q-R Global: Position, Velocity, Heading Trust"),
      {"Global P position sigma","Global P dynamics sigma","Global R position variance","Global R heading/velocity variance"},
      {{"Variasi","Qx","Qy","Qyaw","Qvx","Qwz","Mean sigma X","Mean sigma Y","Mean sigma Yaw"},
       {"Variasi","R GNSS X","R GNSS Y","R GNSS vx","R COG yaw","R IMU wz"}},
      {{"sigma X","derived.ekf_global_sigma_x"},{"sigma Y","derived.ekf_global_sigma_y"},{"sigma Yaw","derived.ekf_global_sigma_yaw"},{"sigma Vx","derived.ekf_global_sigma_vx"},{"sigma Wz","derived.ekf_global_sigma_w"},
       {"R GNSS X","gnss_map_odom.var_x"},{"R GNSS Y","gnss_map_odom.var_y"},{"R GNSS vx","gnss_base_vel_fusion.cov_x"},{"R COG yaw","gnss_cog_fusion.yaw_variance"},{"R IMU wz","imu.var_gz"}},
      P({F("qx","Qx","float","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.0","0.05","Q — Process Noise"),
         F("qy","Qy","float","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.16","0.05","Q — Process Noise"),
         F("qyaw","Qyaw","float","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.80","0.06","Q — Process Noise"),
         F("qvx","Qvx","float","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.96","0.025","Q — Process Noise"),
         F("qwz","Qwz","float","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.176","0.02","Q — Process Noise"),
         F("r_gnss_v_min","R GNSS vx min","float","localization","localization_core.ros__parameters.gnss_velocity_fusion_min_variance","0.0025","R — Measurement Noise"),
         F("r_gnss_v_max","R GNSS vx max","float","localization","localization_core.ros__parameters.gnss_velocity_fusion_max_variance","0.25","R — Measurement Noise"),
         F("r_cog_min","R COG yaw min","float","localization","localization_core.ros__parameters.gnss_cog_fusion_min_variance_rad2","0.001218","R — Measurement Noise"),
         F("r_cog_max","R COG yaw max","float","localization","localization_core.ros__parameters.gnss_cog_fusion_max_variance_rad2","0.274","R — Measurement Noise"),
         F("r_imu_w","R IMU gyro-Z","float","imu","data_imu_node.ros__parameters.angular_velocity_covariance.2","0.02","R — Measurement Noise")}),
      {{"time_series",{"sigma X","sigma Y"},{},{},"Time [s]","σ position [m]"},
       {"time_series",{"sigma Yaw","sigma Vx","sigma Wz"},{},{},"Time [s]","σ state"},
       {"time_series",{"R GNSS X","R GNSS Y"},{},{},"Time [s]","Position variance [m²]"},
       {"time_series",{"R GNSS vx","R COG yaw","R IMU wz"},{},{},"Time [s]","Measurement variance"}});

    add("navigation",QStringLiteral("N8"),QStringLiteral("N8 Global EKF — Core Tuning"),"N8.3",
      QStringLiteral("N8.3 Kalman Authority, Residual, NIS Proxy dan Drift Analyzer Global"),
      {"K_eff Global position","K_eff Global velocity/heading","Global position residual","Global velocity/heading residual","Global NIS diagnostic proxy","Global P growth"},
      {{"Variasi","Mean K x GNSS","Mean K y GNSS","Mean K vx GNSS","Mean K yaw COG","Mean K wz IMU","P95 NIS x GNSS","P95 NIS y GNSS"},
       {"Variasi","RMSE Residual x GNSS","RMSE Residual y GNSS","RMSE Residual vx GNSS","RMSE Residual yaw COG","Max P growth X","Max P growth Y"}},
      {{"K x GNSS","derived.ekf_global_k_x_gnss"},{"K y GNSS","derived.ekf_global_k_y_gnss"},{"K vx GNSS","derived.ekf_global_k_vx_gnss"},{"K yaw COG","derived.ekf_global_k_yaw_cog"},{"K wz IMU","derived.ekf_global_k_w_imu"},
       {"Residual x GNSS","derived.ekf_global_res_x_gnss"},{"Residual y GNSS","derived.ekf_global_res_y_gnss"},{"Residual vx GNSS","derived.ekf_global_res_vx_gnss"},{"Residual yaw COG","derived.ekf_global_res_yaw_cog"},{"Residual wz IMU","derived.ekf_global_res_w_imu"},
       {"NIS x GNSS","derived.ekf_global_nis_x_gnss"},{"NIS y GNSS","derived.ekf_global_nis_y_gnss"},{"NIS vx GNSS","derived.ekf_global_nis_vx_gnss"},{"NIS yaw COG","derived.ekf_global_nis_yaw_cog"},{"NIS wz IMU","derived.ekf_global_nis_w_imu"},
       {"P growth X","derived.ekf_global_p_growth_x"},{"P growth Y","derived.ekf_global_p_growth_y"},{"P growth Yaw","derived.ekf_global_p_growth_yaw"},{"P growth Vx","derived.ekf_global_p_growth_vx"},{"P growth Wz","derived.ekf_global_p_growth_w"}}, P({}),
      {{"time_series",{"K x GNSS","K y GNSS"},{},{},"Time [s]","K_eff [0..1]"},
       {"time_series",{"K vx GNSS","K yaw COG","K wz IMU"},{},{},"Time [s]","K_eff [0..1]"},
       {"time_series",{"Residual x GNSS","Residual y GNSS"},{},{},"Time [s]","Position residual [m]"},
       {"time_series",{"Residual vx GNSS","Residual yaw COG","Residual wz IMU"},{},{},"Time [s]","Residual"},
       {"time_series",{"NIS x GNSS","NIS y GNSS","NIS vx GNSS","NIS yaw COG","NIS wz IMU"},{},{},"Time [s]","NIS proxy [-]"},
       {"time_series",{"P growth X","P growth Y","P growth Yaw","P growth Vx","P growth Wz"},{},{},"Time [s]","dP/dt"}});

    add("navigation",QStringLiteral("N8"),QStringLiteral("N8 Global EKF — Core Tuning"),"N8.4",
      QStringLiteral("N8.4 Final Validation Global EKF"),
      {"Final raw GNSS vs global EKF trajectory","Final COG vs Global yaw","Final global covariance"},
      {{"Variasi","RMSE Residual x GNSS","RMSE Residual y GNSS","RMSE Residual yaw COG","Mean sigma X","Mean sigma Y","Mean sigma Yaw","Max P growth X","Max P growth Y"}},
      {{"COG yaw","gnss_cog_fusion.yaw_rad"},{"Global yaw","ekf_global.yaw"},{"sigma X","derived.ekf_global_sigma_x"},{"sigma Y","derived.ekf_global_sigma_y"},{"sigma Yaw","derived.ekf_global_sigma_yaw"}},
      P({RO("frequency","Final Frequency","ekf","ekf_filter_node_map.ros__parameters.frequency","FROZEN CONFIG"),RO("timeout","Final sensor timeout","ekf","ekf_filter_node_map.ros__parameters.sensor_timeout","FROZEN CONFIG"),RO("qx","Final Qx","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.0","FROZEN CONFIG"),RO("qy","Final Qy","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.16","FROZEN CONFIG")}),
      {{"scatter",{},"ekf_global.x","ekf_global.y","Map X [m]","Map Y [m]"},
       {"time_series",{"COG yaw","Global yaw"},{},{},"Time [s]","Yaw [rad]"},
       {"time_series",{"sigma X","sigma Y","sigma Yaw"},{},{},"Time [s]","σ state"}});

    // N9 — custom global correction layer.
    add("navigation",QStringLiteral("N9"),QStringLiteral("N9 LocalizationCore & map→odom"),"N9.1",
      QStringLiteral("N9.1 Quality Gate, Startup dan Fusion Qualification"),
      {"Global EKF trajectory during localization gating","GNSS quality during localization"},
      {{"Variasi","Mean Satelit","Mean DOP","Mean hAcc","Mean Global X","Mean Global Y"}},
      {{"Satelit","gnss_quality.sat"},{"DOP","gnss_quality.dop"},{"hAcc","gnss_quality.hacc_m"},{"Global X","ekf_global.x"},{"Global Y","ekf_global.y"}},
      P({F("startup_gnss","Startup GNSS samples","int","localization","localization_core.ros__parameters.startup_gnss_samples","5","Startup / Planning"),
         F("degraded_sat","Degraded planning min satellites","int","localization","localization_core.ros__parameters.degraded_min_satellites","6","Startup / Planning"),
         F("degraded_dop","Degraded planning max DOP","float","localization","localization_core.ros__parameters.degraded_max_dop","8.0","Startup / Planning"),
         F("degraded_hacc","Degraded planning max hAcc (m)","float","localization","localization_core.ros__parameters.degraded_max_hacc_m","25.0","Startup / Planning"),
         F("strict_sat","Strict minimum satellites","int","localization","localization_core.ros__parameters.strict_min_satellites","8","Motion Acquire"),
         F("strict_dop","Strict maximum DOP","float","localization","localization_core.ros__parameters.strict_max_dop","2.0","Motion Acquire"),
         F("strict_hacc","Strict maximum hAcc (m)","float","localization","localization_core.ros__parameters.strict_max_hacc_m","3.0","Motion Acquire"),
         F("hold_sec","Strict quality hold (s)","float","localization","localization_core.ros__parameters.strict_quality_hold_sec","1.5","Motion Acquire"),
         F("motion_hold_hacc","Motion hold max hAcc (m)","float","localization","localization_core.ros__parameters.motion_hold_max_hacc_m","4.5","Motion Hold"),
         F("motion_hold_dop","Motion hold max DOP","float","localization","localization_core.ros__parameters.motion_hold_max_dop","2.5","Motion Hold"),
         F("motion_hold_sat","Motion hold min satellites","int","localization","localization_core.ros__parameters.motion_hold_min_satellites","8","Motion Hold"),
         F("motion_grace","Motion degrade grace (s)","float","localization","localization_core.ros__parameters.motion_degrade_grace_sec","3.0","Motion Hold"),
         F("critical_hacc","Critical max hAcc (m)","float","localization","localization_core.ros__parameters.motion_critical_max_hacc_m","6.0","Motion Fail-Closed"),
         F("critical_dop","Critical max DOP","float","localization","localization_core.ros__parameters.motion_critical_max_dop","4.0","Motion Fail-Closed"),
         F("critical_sat","Critical min satellites","int","localization","localization_core.ros__parameters.motion_critical_min_satellites","6","Motion Fail-Closed")}),
      {{"scatter",{},"ekf_global.x","ekf_global.y","Map X [m]","Map Y [m]"},
       {"time_series",{"Satelit","DOP","hAcc"},{},{},"Time [s]","GNSS quality"}});

    add("navigation",QStringLiteral("N9"),QStringLiteral("N9 LocalizationCore & map→odom"),"N9.2",
      QStringLiteral("N9.2 map→odom Correction Position dan Yaw"),
      {"Raw GNSS/global EKF/final map X","Raw GNSS/global EKF/final map Y","Global yaw and final localization yaw"},
      {{"Variasi","Correction alpha stationary","Correction alpha moving","Max correction [m]","Yaw correction alpha","Max yaw step"}},
      {{"GNSS X","gnss_map_odom.x"},{"Global X","ekf_global.x"},{"Final map X","localization_state.map_x"},{"GNSS Y","gnss_map_odom.y"},{"Global Y","ekf_global.y"},{"Final map Y","localization_state.map_y"},{"Global yaw","ekf_global.yaw"},{"Final yaw","localization_state.yaw"}},
      P({F("alpha_stationary","Strict correction alpha stationary","float","localization","localization_core.ros__parameters.strict_correction_alpha","0.08","Position Correction"),
         F("alpha_moving","Strict correction alpha moving","float","localization","localization_core.ros__parameters.strict_moving_correction_alpha","0.01","Position Correction"),
         F("max_corr","Strict max correction (m)","float","localization","localization_core.ros__parameters.strict_max_correction_m","0.08","Position Correction"),
         F("global_ref_gate","Global EKF map-reference max error (m)","float","localization","localization_core.ros__parameters.global_odom_map_reference_max_error_m","15.0","Position Correction"),
         F("yaw_alpha","Global EKF yaw correction alpha","float","localization","localization_core.ros__parameters.global_ekf_yaw_correction_alpha","0.05","Yaw Correction"),
         F("yaw_step","Global EKF yaw max step (rad)","float","localization","localization_core.ros__parameters.global_ekf_yaw_max_step_rad","0.0087266","Yaw Correction"),
         F("yaw_innovation","Global EKF yaw max innovation (rad)","float","localization","localization_core.ros__parameters.global_ekf_yaw_max_innovation_rad","0.7854","Yaw Correction")}),
      {{"time_series",{"GNSS X","Global X","Final map X"},{},{},"Time [s]","Map X [m]"},
       {"time_series",{"GNSS Y","Global Y","Final map Y"},{},{},"Time [s]","Map Y [m]"},
       {"time_series",{"Global yaw","Final yaw"},{},{},"Time [s]","Yaw [rad]"}});

    // N10 — costmap before planner/controller tuning.
    add("navigation",QStringLiteral("N10"),QStringLiteral("N10 Costmap"),"N10.1",
      QStringLiteral("N10.1 Global Costmap Geometry & Inflation"),{},
      {{"Parameter","Nilai","Kategori"}}, {},
      P({LOCK("footprint","Footprint","nav2","global_costmap.global_costmap.ros__parameters.footprint","GEOMETRY AUTHORITY"),
         F("padding","Footprint padding (m)","float","nav2","global_costmap.global_costmap.ros__parameters.footprint_padding","0.03","Global Costmap"),
         F("resolution","Resolution (m/cell)","float","nav2","global_costmap.global_costmap.ros__parameters.resolution","0.1","Global Costmap"),
         F("inflation","Inflation radius (m)","float","nav2","global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius","0.8","Inflation"),
         F("scaling","Cost scaling factor","float","nav2","global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor","2.5","Inflation"),
         F("transform_tol","Transform tolerance (s)","float","nav2","global_costmap.global_costmap.ros__parameters.transform_tolerance","0.3","Timing") }));

    add("navigation",QStringLiteral("N10"),QStringLiteral("N10 Costmap"),"N10.2",
      QStringLiteral("N10.2 Local Costmap Obstacle, Drivable Boundary & Inflation"),{},
      {{"Parameter","Nilai","Kategori"}}, {},
      P({F("update_rate","Update frequency (Hz)","float","nav2","local_costmap.local_costmap.ros__parameters.update_frequency","8","Timing"),
         F("width","Window width (m)","float","nav2","local_costmap.local_costmap.ros__parameters.width","8","Geometry"),
         F("height","Window height (m)","float","nav2","local_costmap.local_costmap.ros__parameters.height","8","Geometry"),
         F("resolution","Resolution (m/cell)","float","nav2","local_costmap.local_costmap.ros__parameters.resolution","0.1","Geometry"),
         F("inflation","Inflation radius (m)","float","nav2","local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius","0.45","Inflation"),
         F("scaling","Cost scaling factor","float","nav2","local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor","3.0","Inflation"),
         F("obstacle_range","Obstacle max range (m)","float","nav2","local_costmap.local_costmap.ros__parameters.obstacle_layer.yolop_points.obstacle_max_range","4.0","Obstacle Layer"),
         F("persistence","Observation persistence (s)","float","nav2","local_costmap.local_costmap.ros__parameters.obstacle_layer.yolop_points.observation_persistence","0.25","Obstacle Layer") }));

    // N11 — Smac. Rmin locked to N2 physical calibration.
    add("navigation",QStringLiteral("N11"),QStringLiteral("N11 Smac Hybrid-A*"),"N11.1",
      QStringLiteral("N11.1 Search Resolution, Penalty dan Analytic Expansion"),
      {"Planning time against runs","Path length against runs"},
      {{"Variasi","Planning time","Path length","Angle bins","Downsampling factor","Cost penalty","Non-straight penalty"}},
      {{"Planning time","nav_path.planning_latency_ms"},{"Path length","nav_path.length_m"}},
      P({LOCK("rmin","Minimum turning radius (m)","nav2","planner_server.ros__parameters.GridBased.minimum_turning_radius","PHYSICAL AUTHORITY — N2"),
         F("downsample","Downsample costmap","bool","nav2","planner_server.ros__parameters.GridBased.downsample_costmap","true","Search Resolution"),
         F("factor","Downsampling factor","int","nav2","planner_server.ros__parameters.GridBased.downsampling_factor","2","Search Resolution"),
         F("bins","Angle quantization bins","int","nav2","planner_server.ros__parameters.GridBased.angle_quantization_bins","48","Search Resolution"),
         F("max_time","Maximum planning time (s)","float","nav2","planner_server.ros__parameters.GridBased.max_planning_time","2.5","Computational Limit"),
         F("cost_penalty","Cost penalty","float","nav2","planner_server.ros__parameters.GridBased.cost_penalty","2.2","Path Cost"),
         F("nonstraight","Non-straight penalty","float","nav2","planner_server.ros__parameters.GridBased.non_straight_penalty","1.2","Path Cost"),
         F("reverse","Reverse penalty","float","nav2","planner_server.ros__parameters.GridBased.reverse_penalty","3.0","Path Cost"),
         F("analytic_ratio","Analytic expansion ratio","float","nav2","planner_server.ros__parameters.GridBased.analytic_expansion_ratio","3.5","Analytic Expansion"),
         F("analytic_max","Analytic expansion max length (m)","float","nav2","planner_server.ros__parameters.GridBased.analytic_expansion_max_length","8.0","Analytic Expansion")}),
      {{"time_series",{"Planning time"},{},{},"Time [s]","Planning latency [ms]"},
       {"time_series",{"Path length"},{},{},"Time [s]","Path length [m]"}});

    add("navigation",QStringLiteral("N11"),QStringLiteral("N11 Smac Hybrid-A*"),"N11.2",
      QStringLiteral("N11.2 Smoother dan Final Global Path Quality"),
      {"Global path length","Global path heading variation"},
      {{"Variasi","Path length","Planning time","Smoother w_smooth","Smoother w_data"}},
      {{"Path length","nav_path.length_m"},{"Heading variation","nav_path.heading_variation_rad"}},
      P({F("smooth_path","Smooth path","bool","nav2","planner_server.ros__parameters.GridBased.smooth_path","true","Path Smoother"),
         F("w_smooth","Smoother w_smooth","float","nav2","planner_server.ros__parameters.GridBased.smoother.w_smooth","0.3","Path Smoother"),
         F("w_data","Smoother w_data","float","nav2","planner_server.ros__parameters.GridBased.smoother.w_data","0.2","Path Smoother"),
         F("iterations","Smoother max iterations","int","nav2","planner_server.ros__parameters.GridBased.smoother.max_iterations","200","Path Smoother")}),
      {{"time_series",{"Path length"},{},{},"Time [s]","Path length [m]"},
       {"time_series",{"Heading variation"},{},{},"Time [s]","Heading variation [rad]"}});

    // N12 — MPPI local controller.
    add("navigation",QStringLiteral("N12"),QStringLiteral("N12 MPPI Ackermann Controller"),"N12.1",
      QStringLiteral("N12.1 Horizon, Sampling dan Computational Load"),
      {"CTE against Time [s]","Velocity tracking against Time [s]","Steering tracking against Time [s]"},
      {{"Variasi","Controller frequency","model_dt","time_steps","batch_size","CTE RMSE","RMSE V","RMSE Steering"}},
      {{"CTE","derived.cte_m"},{"V command","cmd_nav.linear_x"},{"V actual","esc_drive_actual"},{"Steer target","esc_steer_target"},{"Steer actual","esc_steer_actual"}},
      P({F("frequency","Controller frequency (Hz)","float","nav2","controller_server.ros__parameters.controller_frequency","8","Timing"),
         F("dt","model_dt (s)","float","nav2","controller_server.ros__parameters.FollowPath.model_dt","0.125","Horizon"),
         F("steps","time_steps","int","nav2","controller_server.ros__parameters.FollowPath.time_steps","32","Horizon"),
         F("batch","batch_size","int","nav2","controller_server.ros__parameters.FollowPath.batch_size","1000","Sampling"),
         F("iterations","iteration_count","int","nav2","controller_server.ros__parameters.FollowPath.iteration_count","1","Sampling"),
         F("vx_std","vx_std","float","nav2","controller_server.ros__parameters.FollowPath.vx_std","0.1","Sampling"),
         F("wz_std","wz_std","float","nav2","controller_server.ros__parameters.FollowPath.wz_std","0.25","Sampling"),
         F("temperature","temperature","float","nav2","controller_server.ros__parameters.FollowPath.temperature","0.3","Optimization"),
         F("gamma","gamma","float","nav2","controller_server.ros__parameters.FollowPath.gamma","0.015","Optimization"),
         LOCK("rmin","Ackermann minimum turning radius","nav2","controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r","PHYSICAL AUTHORITY — N2")}),
      {{"time_series",{"CTE"},{},{},"Time [s]","Cross-track error [m]"},
       {"time_series",{"V command","V actual"},{},{},"Time [s]","Velocity [m/s]"},
       {"time_series",{"Steer target","Steer actual"},{},{},"Time [s]","Steering [rad]"}});

    add("navigation",QStringLiteral("N12"),QStringLiteral("N12 MPPI Ackermann Controller"),"N12.2",
      QStringLiteral("N12.2 Critic Weights dan Kinematic Constraints"),
      {"CTE after critic tuning","Heading error after critic tuning"},
      {{"Variasi","PathAlign","PathFollow","PathAngle","Cost","Goal","PreferForward","CTE RMSE","Heading RMSE"}},
      {{"CTE","derived.cte_m"},{"Heading error","derived.path_heading_error_rad"}},
      P({F("path_align","PathAlignCritic weight","float","nav2","controller_server.ros__parameters.FollowPath.PathAlignCritic.cost_weight","8","Critic Weights"),
         F("path_follow","PathFollowCritic weight","float","nav2","controller_server.ros__parameters.FollowPath.PathFollowCritic.cost_weight","4","Critic Weights"),
         F("path_angle","PathAngleCritic weight","float","nav2","controller_server.ros__parameters.FollowPath.PathAngleCritic.cost_weight","3","Critic Weights"),
         F("cost","CostCritic weight","float","nav2","controller_server.ros__parameters.FollowPath.CostCritic.cost_weight","6","Critic Weights"),
         F("goal","GoalCritic weight","float","nav2","controller_server.ros__parameters.FollowPath.GoalCritic.cost_weight","5","Critic Weights"),
         F("goal_angle","GoalAngleCritic weight","float","nav2","controller_server.ros__parameters.FollowPath.GoalAngleCritic.cost_weight","3","Critic Weights"),
         F("forward","PreferForwardCritic weight","float","nav2","controller_server.ros__parameters.FollowPath.PreferForwardCritic.cost_weight","7","Critic Weights"),
         F("deadband","VelocityDeadbandCritic weight","float","nav2","controller_server.ros__parameters.FollowPath.VelocityDeadbandCritic.cost_weight","35","Critic Weights"),
         F("vx_max","vx_max (m/s)","float","nav2","controller_server.ros__parameters.FollowPath.vx_max","0.5","Kinematic Constraints"),
         F("wz_max","wz_max (rad/s)","float","nav2","controller_server.ros__parameters.FollowPath.wz_max","0.292","Kinematic Constraints"),
         F("ax_max","ax_max (m/s²)","float","nav2","controller_server.ros__parameters.FollowPath.ax_max","0.75","Kinematic Constraints")}),
      {{"time_series",{"CTE"},{},{},"Time [s]","Cross-track error [m]"},
       {"time_series",{"Heading error"},{},{},"Time [s]","Heading error [rad]"}});

    add("navigation",QStringLiteral("N12"),QStringLiteral("N12 MPPI Ackermann Controller"),"N12.3",
      QStringLiteral("N12.3 Final MPPI Closed-loop Validation"),
      {"Final CTE","Final velocity command vs actual","Final steering command vs actual"},
      {{"Variasi","CTE RMSE","Max CTE","RMSE V","RMSE Steering","RMSE Yaw","Time-to-goal"}},
      {{"CTE","derived.cte_m"},{"V command","cmd_final.linear_x"},{"V actual","esc_drive_actual"},{"Steer target","esc_steer_target"},{"Steer actual","esc_steer_actual"}}, P({}),
      {{"time_series",{"CTE"},{},{},"Time [s]","Cross-track error [m]"},
       {"time_series",{"V command","V actual"},{},{},"Time [s]","Velocity [m/s]"},
       {"time_series",{"Steer target","Steer actual"},{},{},"Time [s]","Steering [rad]"}});

    // N13 — command shaping and mux.
    add("navigation",QStringLiteral("N13"),QStringLiteral("N13 Velocity Smoother & Command Chain"),"N13.1",
      QStringLiteral("N13.1 Velocity Smoother Accel/Decel, Deadband dan Feedback"),
      {"Raw Nav2 vs final velocity vs actual","Raw Nav2 vs final yaw command"},
      {{"Variasi","Smoothing frequency","Max accel vx","Max decel vx","Deadband vx","RMSE V","Stop error"}},
      {{"Nav raw vx","cmd_nav.linear_x"},{"Integrated vx","cmd_autonomy_integrated.linear_x"},{"Final vx","cmd_final.linear_x"},{"Actual vx","esc_drive_actual"},{"Nav raw wz","cmd_nav.angular_z"},{"Final wz","cmd_final.angular_z"}},
      P({F("frequency","Smoothing frequency (Hz)","float","nav2","velocity_smoother.ros__parameters.smoothing_frequency","20","Smoother"),
         F("feedback","Feedback mode","string","nav2","velocity_smoother.ros__parameters.feedback","OPEN_LOOP","Smoother"),
         F("accel","Max accel vx (m/s²)","float","nav2","velocity_smoother.ros__parameters.max_accel.0","0.18","Smoother"),
         F("decel","Max decel vx (m/s²)","float","nav2","velocity_smoother.ros__parameters.max_decel.0","-0.28","Smoother"),
         F("deadband","Deadband vx (m/s)","float","nav2","velocity_smoother.ros__parameters.deadband_velocity.0","0.03","Smoother"),
         F("timeout","Velocity timeout (s)","float","nav2","velocity_smoother.ros__parameters.velocity_timeout","0.4","Smoother")}),
      {{"time_series",{"Nav raw vx","Integrated vx","Final vx","Actual vx"},{},{},"Time [s]","Velocity [m/s]"},
       {"time_series",{"Nav raw wz","Final wz"},{},{},"Time [s]","Yaw command [rad/s]"}});

    add("navigation",QStringLiteral("N13"),QStringLiteral("N13 Velocity Smoother & Command Chain"),"N13.2",
      QStringLiteral("N13.2 NavigationCore Deadband, Timeout dan Authority"),
      {"Command stages linear","Command stages angular"},
      {{"Variasi","Linear deadband","Angular deadband","Autonomy timeout","Mean Actual speed","RMSE V"}},
      {{"Nav raw vx","cmd_nav.linear_x"},{"Integrated vx","cmd_autonomy_integrated.linear_x"},{"Pre-collision vx","cmd_pre_collision.linear_x"},{"Final vx","cmd_final.linear_x"},{"Actuator vx","cmd_actuator.linear_x"},
       {"Nav raw wz","cmd_nav.angular_z"},{"Integrated wz","cmd_autonomy_integrated.angular_z"},{"Final wz","cmd_final.angular_z"},{"Actuator wz","cmd_actuator.angular_z"}},
      P({F("autonomy_timeout","Autonomy timeout (s)","float","navigation_core","navigation_core.ros__parameters.autonomy_timeout_sec","0.6","Command Timing"),
         F("linear_deadband","Linear deadband (m/s)","float","navigation_core","navigation_core.ros__parameters.linear_deadband_mps","0.08","Command Deadband"),
         F("angular_deadband","Angular deadband (rad/s)","float","navigation_core","navigation_core.ros__parameters.angular_deadband_rps","0.02","Command Deadband"),
         F("min_yaw_speed","Minimum speed for yaw (m/s)","float","navigation_core","navigation_core.ros__parameters.min_speed_for_yaw_mps","0.08","Command Deadband")}),
      {{"time_series",{"Nav raw vx","Integrated vx","Pre-collision vx","Final vx","Actuator vx"},{},{},"Time [s]","Linear velocity [m/s]"},
       {"time_series",{"Nav raw wz","Integrated wz","Final wz","Actuator wz"},{},{},"Time [s]","Angular velocity [rad/s]"}});

    // N14 — safety after nominal control works.
    add("navigation",QStringLiteral("N14"),QStringLiteral("N14 Trajectory & Collision Safety"),"N14.1",
      QStringLiteral("N14.1 Trajectory Safety Slow/Stop/Avoidance"),
      {"Safety speed scale and final velocity","Command velocity through safety"},
      {{"Variasi","Hard stop distance","Slow distance","Minimum slow scale","Avoidance speed","Mean Actual speed"}},
      {{"Speed scale","trajectory_safety_state.speed_scale"},{"Nav vx","cmd_nav.linear_x"},{"Safe vx","cmd_autonomy_integrated.linear_x"},{"Actual vx","esc_drive_actual"}},
      P({F("hard_stop","Hard-stop path distance (m)","float","trajectory_safety","trajectory_safety_supervisor.ros__parameters.hard_stop_path_distance_m","0.9","Longitudinal Safety"),
         F("slow","Slow path distance (m)","float","trajectory_safety","trajectory_safety_supervisor.ros__parameters.slow_path_distance_m","2.2","Longitudinal Safety"),
         F("min_scale","Minimum slow speed scale","float","trajectory_safety","trajectory_safety_supervisor.ros__parameters.minimum_slow_speed_scale","0.35","Longitudinal Safety"),
         F("collision_horizon","Command collision horizon (m)","float","trajectory_safety","trajectory_safety_supervisor.ros__parameters.command_collision_horizon_m","2.5","Collision Projection"),
         F("lateral_margin","Command collision lateral margin (m)","float","trajectory_safety","trajectory_safety_supervisor.ros__parameters.command_collision_lateral_margin_m","0.2","Collision Projection"),
         F("avoid_speed","Avoidance speed (m/s)","float","trajectory_safety","trajectory_safety_supervisor.ros__parameters.avoidance_speed_mps","0.18","Avoidance")}),
      {{"time_series",{"Speed scale"},{},{},"Time [s]","Speed scale [-]"},
       {"time_series",{"Nav vx","Safe vx","Actual vx"},{},{},"Time [s]","Velocity [m/s]"}});

    add("navigation",QStringLiteral("N14"),QStringLiteral("N14 Trajectory & Collision Safety"),"N14.2",
      QStringLiteral("N14.2 Collision Monitor Polygon Stop/Slow"),
      {"Final command before/after collision monitor"},
      {{"Parameter","Nilai","Safety role"}},
      {{"Pre-collision vx","cmd_pre_collision.linear_x"},{"Final vx","cmd_final.linear_x"}},
      P({F("source_timeout","Collision source timeout (s)","float","collision","collision_monitor.ros__parameters.source_timeout","1.0","Collision Monitor"),
         F("stop_timeout","Stop publish timeout (s)","float","collision","collision_monitor.ros__parameters.stop_pub_timeout","1.0","Collision Monitor"),
         F("slow_ratio","Slowdown ratio","float","collision","collision_monitor.ros__parameters.PolygonSlow.slowdown_ratio","0.25","Collision Monitor"),
         RO("stop_polygon","Stop polygon","collision","collision_monitor.ros__parameters.PolygonStop.points","Geometry"),
         RO("slow_polygon","Slow polygon","collision","collision_monitor.ros__parameters.PolygonSlow.points","Geometry")}),
      {{"time_series",{"Pre-collision vx","Final vx"},{},{},"Time [s]","Velocity [m/s]"}});

    // N15 — termination semantics.
    add("navigation",QStringLiteral("N15"),QStringLiteral("N15 Goal, Progress & Failure"),"N15.1",
      QStringLiteral("N15.1 Goal Checker, Progress Checker dan Failure Tolerance"),
      {"Endpoint error during approach","Goal yaw error during approach"},
      {{"Variasi","XY tolerance","Yaw tolerance","Movement radius","Movement allowance","Endpoint error","Time-to-goal","Success"}},
      {{"Endpoint error","derived.endpoint_error_m"},{"Goal yaw error","derived.goal_yaw_error_rad"}},
      P({F("xy_tol","XY goal tolerance (m)","float","nav2","controller_server.ros__parameters.goal_checker.xy_goal_tolerance","0.75","Goal Checker"),
         F("yaw_tol","Yaw goal tolerance (rad)","float","nav2","controller_server.ros__parameters.goal_checker.yaw_goal_tolerance","0.5236","Goal Checker"),
         F("movement_radius","Required movement radius (m)","float","nav2","controller_server.ros__parameters.progress_checker.required_movement_radius","0.2","Progress Checker"),
         F("movement_time","Movement time allowance (s)","float","nav2","controller_server.ros__parameters.progress_checker.movement_time_allowance","30","Progress Checker"),
         F("failure_tolerance","Controller failure tolerance (s)","float","nav2","controller_server.ros__parameters.failure_tolerance","0.5","Failure Handling")}),
      {{"time_series",{"Endpoint error"},{},{},"Time [s]","Endpoint error [m]"},
       {"time_series",{"Goal yaw error"},{},{},"Time [s]","Yaw error [rad]"}});

    // N16 — end-to-end only after previous stages are stable.
    add("navigation",QStringLiteral("N16"),QStringLiteral("N16 Autonomous End-to-End"),"N16.1",
      QStringLiteral("N16.1 Full Autonomous Route Validation"),
      {"Actual global trajectory","Cross-track error","Velocity command vs actual","Steering target vs actual"},
      {{"Variasi","Success","CTE RMSE","Max CTE","Heading RMSE","Endpoint error","Time-to-goal","RMSE V","RMSE Steering"}},
      {{"Global X","localization_state.map_x"},{"Global Y","localization_state.map_y"},{"CTE","derived.cte_m"},{"V command","cmd_final.linear_x"},{"V actual","esc_drive_actual"},{"Steer target","esc_steer_target"},{"Steer actual","esc_steer_actual"}}, P({}),
      {{"scatter",{},"localization_state.map_x","localization_state.map_y","Map X [m]","Map Y [m]"},
       {"time_series",{"CTE"},{},{},"Time [s]","Cross-track error [m]"},
       {"time_series",{"V command","V actual"},{},{},"Time [s]","Velocity [m/s]"},
       {"time_series",{"Steer target","Steer actual"},{},{},"Time [s]","Steering [rad]"}});

    // N17 — frozen snapshot; no free tuning after certification.
    add("navigation",QStringLiteral("N17"),QStringLiteral("N17 Final Certified Configuration"),"N17.1",
      QStringLiteral("N17.1 Snapshot Konfigurasi Autonomous Terbaik"),{},
      {{"Subsystem","Parameter utama","Nilai final / Status"}}, {},
      P({RO("steering_valid","Steering calibration valid","vehicle","vehicle.ros__parameters.steering_calibration_valid","Certification"),
         RO("circle_valid","Circle calibration valid","vehicle","vehicle.ros__parameters.steering_circle_calibration_valid","Certification"),
         RO("odom_valid","Drive odometry valid","vehicle","vehicle.ros__parameters.drive_odometry_calibration_valid","Certification"),
         RO("local_qvx","Local Qvx","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.96","EKF Local"),
         RO("local_qwz","Local Qwz","ekf","ekf_filter_node_odom.ros__parameters.process_noise_covariance.176","EKF Local"),
         RO("global_qx","Global Qx","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.0","EKF Global"),
         RO("global_qy","Global Qy","ekf","ekf_filter_node_map.ros__parameters.process_noise_covariance.16","EKF Global"),
         RO("smac_rmin","Smac Rmin","nav2","planner_server.ros__parameters.GridBased.minimum_turning_radius","Planning"),
         RO("mppi_rmin","MPPI Rmin","nav2","controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r","Control"),
         RO("stage3","Stage3 production certified","navigation_core","navigation_core.ros__parameters.stage3_production_certified","Certification") }));

    return out;
  }

  /* ------------------------- NAVIGASI (legacy, unreachable for navigation) ------------------------- */
  add("navigation", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Sensor"), "4.1.1",
  QStringLiteral("4.1.1 Pengujian GNSS"),
  {
    "Sebaran posisi GNSS statis", "Kualitas satelit, pDOP, dan hAcc GNSS"
  },
  {
    {
      "Parameter", "Nilai YAML Aktual", "Fungsi"
    },
    {
      "Metrik", "Hasil"
    }
  },
  {
    {
      "Satelit", "gnss_quality.sat"
    }, {
      "DOP", "gnss_quality.dop"
    }, {
      "hAcc", "gnss_quality.hacc_m"
    }
  },
  {
    { "sample_rate", "Sample Rate (Hz)", "float", "", "", "10.0" },
    { "duration", "Durasi (s)", "float", "", "", "30.0" },
    { "variation", "Variasi / Run", "string", "", "", "variasi-1" },
    { "condition", "Kondisi", "string", "", "", "statis" },
    { "gt_x", "Ground Truth X", "float", "", "", "0.0", "", true },
    { "gt_y", "Ground Truth Y", "float", "", "", "0.0", "", true },
    { "min_satellites", "Min Satellites", "yaml_readonly", "gnss", "data_cuav_node.ros__parameters.min_satellites" },
    { "max_dop", "Max DOP", "yaml_readonly", "gnss", "data_cuav_node.ros__parameters.max_dop" },
    { "max_hacc_m", "Max hAcc (m)", "yaml_readonly", "gnss", "data_cuav_node.ros__parameters.max_hacc_m" },
    { "nav_rate", "navigation_rate_hz", "float", "gnss", "data_cuav_node.ros__parameters.navigation_rate_hz", "10.0" },
    { "nav_model", "Dynamic Model", "yaml_readonly", "gnss", "data_cuav_node.ros__parameters.dynamic_model" }
  },
  {
    { "scatter", {}, "gnss_fix.lat", "gnss_fix.lon" },
    { "time_series", { "Satelit", "DOP", "hAcc" } }
  });
  add("navigation", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Sensor"), "4.1.2",
  QStringLiteral("4.1.2 Pengujian IMU"),
  {
    "Noise gyro-Z IMU ketika kendaraan diam", "Linearitas heading IMU"
  },
  {
    {
      "Sumbu", "Mean gyro saat diam (rad/s)", "Std (rad/s)"
    },
    {
      "Heading referensi", "Heading IMU", "Error"
    }
  },
  {
    {
      "Gyro Z", "imu.gz"
    }, {
      "Yaw", "imu.yaw_rad"
    }, {
      "Yaw residual", "imu_status.yaw_residual"
    }
  });
  add("navigation", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Sensor"), "4.1.3",
  QStringLiteral("4.1.3 Pengujian Encoder Steering dan Feedback RPM"),
  {
    "Linearitas feedback RPM", "Linearitas encoder steering"
  },
  {
    {
      "RPM perintah", "RPM feedback", "Error", "Error relatif"
    },
    {
      "Steering perintah", "Feedback", "Error"
    }
  },
  {
    {
      "Steer target", "esc_steer_target"
    }, {
      "Steer actual", "esc_steer_actual"
    },
    {
      "Drive target", "esc_drive_target"
    }, {
      "Drive actual", "esc_drive_actual"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.1",
  QStringLiteral("4.2.1 Pengujian Parameter frequency"),
  {
    "Pengaruh frequency terhadap RMSE dan latency EKF lokal"
  },
  {
    {
      "frequency", "RMSE v", "RMSE yaw-rate", "Latency median", "CPU EKF", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  },
  {
    { "sample_rate", "Sample Rate (Hz)", "float", "", "", "20.0" },
    { "duration", "Durasi (s)", "float", "", "", "30.0" },
    { "variation", "Variasi / Run", "string", "", "", "frequency-1" },
    { "condition", "Kondisi", "string", "", "", "lintasan lurus" },
    { "frequency", "EKF Lokal frequency (Hz)", "float", "ekf", "ekf_filter_node_odom.ros__parameters.frequency", "20.0" },
    { "sensor_timeout", "Sensor timeout (s)", "yaml_readonly", "ekf", "ekf_filter_node_odom.ros__parameters.sensor_timeout" }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.2",
  QStringLiteral("4.2.2 Pengujian Parameter sensor_timeout"),
  {
    "Pengaruh sensor timeout EKF lokal"
  },
  {
    {
      "sensor timeout", "Episode predict-only/menit", "Error akhir 10 m", "Respons terhadap stale", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.3",
  QStringLiteral("4.2.3 Pengujian odom0_twist_rejection_threshold"),
  {
    "Pengaruh twist rejection threshold EKF lokal"
  },
  {
    {
      "Threshold", "Outlier tertolak", "Measurement valid ikut tertolak", "RMSE posisi lokal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.4",
  QStringLiteral("4.2.4 Pengujian process_noise_covariance"),
  {
    "Pengaruh process noise EKF lokal"
  },
  {
    {
      "Set", "Q(vx)", "Q(vyaw)", "Karakter"
    },
    {
      "Set Q", "RMSE v", "RMSE yaw-rate", "Waktu respons perubahan", "Noise output v", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.5",
  QStringLiteral("4.2.5 Pengujian predict_to_current_time"),
  {
    "Pengaruh predict_to_current_time EKF lokal"
  },
  {
    {
      "Mode", "RMSE v", "Median latency", "Age state saat publish", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian dan Tuning Extended Kalman Filter Lokal"), "4.2.6",
  QStringLiteral("4.2.6 Validasi Konfigurasi Akhir EKF Lokal"),
  {
    "Perbandingan odometri ESC dan EKF lokal pada lintasan lurus"
  },
  {
    {
      "Metrik validasi", "Baseline 30 Hz", "Konfigurasi tuning"
    }
  },
  {
    {
      "ESC v", "esc_odom.v"
    }, {
      "EKF v", "ekf_local.v"
    }, {
      "ESC w", "esc_odom.w"
    }, {
      "EKF w", "ekf_local.w"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.1",
  QStringLiteral("4.3.1 Pengujian Parameter frequency"),
  {
    "Pengaruh frequency EKF global"
  },
  {
    {
      "frequency", "RMSE posisi statis", "Respons dinamis", "CPU", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  },
  {
    { "sample_rate", "Sample Rate (Hz)", "float", "", "", "10.0" },
    { "duration", "Durasi (s)", "float", "", "", "60.0" },
    { "variation", "Variasi / Run", "string", "", "", "frequency-1" },
    { "condition", "Kondisi", "string", "", "", "statis" },
    { "frequency", "EKF Global frequency (Hz)", "float", "ekf", "ekf_filter_node_map.ros__parameters.frequency", "10.0" },
    { "sensor_timeout", "Sensor timeout (s)", "yaml_readonly", "ekf", "ekf_filter_node_map.ros__parameters.sensor_timeout" },
    { "pose_threshold", "odom0_pose_rejection_threshold", "yaml_readonly", "ekf", "ekf_filter_node_map.ros__parameters.odom0_pose_rejection_threshold" }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.2",
  QStringLiteral("4.3.2 Pengujian Parameter sensor_timeout"),
  {
    "Pengaruh sensor timeout EKF global"
  },
  {
    {
      "sensor timeout", "Kontinuitas output", "Peak error setelah gap", "Keterangan"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.3",
  QStringLiteral("4.3.3 Pengujian odom0_pose_rejection_threshold"),
  {
    "Pengaruh pose rejection threshold EKF global"
  },
  {
    {
      "Threshold", "Spike tertolak", "Measurement valid tertolak", "RMSE posisi", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.4",
  QStringLiteral("4.3.4 Pengujian process_noise_covariance Posisi X-Y"),
  {
    "Pengaruh process noise posisi EKF global"
  },
  {
    {
      "Qx=Qy", "Std statis", "RMSE dinamis", "Waktu respons", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.5",
  QStringLiteral("4.3.5 Pengujian predict_to_current_time"),
  {
    "Pengaruh predict_to_current_time EKF global"
  },
  {
    {
      "Mode", "RMSE dinamis", "Peak error", "Karakter output", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian dan Tuning Extended Kalman Filter Global"), "4.3.6",
  QStringLiteral("4.3.6 Validasi Konfigurasi Akhir EKF Global"),
  {
    "Perbandingan scatter GNSS map dan EKF global akhir"
  },
  {
    {
      "Metrik", "GNSS map raw", "EKF global tuning"
    }
  },
  {
    {
      "Map X", "localization_state.map_x"
    }, {
      "EKF X", "ekf_global.x"
    },
    {
      "Map Y", "localization_state.map_y"
    }, {
      "EKF Y", "ekf_global.y"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.1",
  QStringLiteral("4.4.1 Pengujian Kalibrasi Koordinat ENU terhadap Map"),
  {
    "Kalibrasi multi-titik ENU terhadap map"
  },
  {
    {
      "Titik", "Error sebelum", "Error sesudah", "ΔX sebelum", "ΔY sebelum", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.2",
  QStringLiteral("4.4.2 Pengujian startup_gnss_samples"),
  {
    "Pengaruh jumlah sampel GNSS startup"
  },
  {
    {
      "Sampel", "Waktu anchor", "Std posisi awal", "Error awal", "Anchor ulang", "Status"
    }
  },
  {
    {
      "Satelit", "gnss_quality.sat"
    }, {
      "DOP", "gnss_quality.dop"
    }, {
      "hAcc", "gnss_quality.hacc_m"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.3",
  QStringLiteral("4.4.3 Pengujian strict_correction_alpha"),
  {
    "Pengaruh strict_correction_alpha pada kondisi diam"
  },
  {
    {
      "Alpha diam", "Error posisi", "Settling time", "Max Δ map→odom", "Std posisi akhir", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.4",
  QStringLiteral("4.4.4 Pengujian strict_moving_correction_alpha"),
  {
    "Pengaruh strict_moving_correction_alpha"
  },
  {
    {
      "Alpha", "RMSE global", "Max Δ map→odom", "Settling time", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.5",
  QStringLiteral("4.4.5 Pengujian strict_max_correction_m"),
  {
    "Trade-off batas koreksi dan tracking"
  },
  {
    {
      "Batas", "Peak TF step", "Konvergensi", "Tracking RMSE", "Peak tracking", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian dan Tuning Koreksi Global LocalizationCore"), "4.4.6",
  QStringLiteral("4.4.6 Validasi Konfigurasi Akhir LocalizationCore"),
  {
    "Gambar 4.24 Perbandingan error koreksi global baseline dan hasil tuning"
  },
  {
    {
      "Metrik", "Baseline", "Hasil tuning", "Perubahan", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian dan Tuning Global Costmap"), "4.5.1",
  QStringLiteral("4.5.1 Pengujian footprint_padding"),
  {
    "Pengaruh footprint_padding terhadap path"
  },
  {
    {
      "Padding", "Min clearance", "Path length", "Koridor sukses", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian dan Tuning Global Costmap"), "4.5.2",
  QStringLiteral("4.5.2 Pengujian inflation_radius"),
  {
    "Trade-off inflation_radius dan clearance"
  },
  {
    {
      "Radius", "Min clearance", "Path length", "Planning time", "Koridor sukses", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian dan Tuning Global Costmap"), "4.5.3",
  QStringLiteral("4.5.3 Pengujian cost_scaling_factor"),
  {
    "Pengaruh cost_scaling_factor pada clearance path"
  },
  {
    {
      "Faktor", "Min clearance", "Path length", "Planning time", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian dan Tuning Global Costmap"), "4.5.4",
  QStringLiteral("4.5.4 Validasi Konfigurasi Akhir Global Costmap"),
  {
    "Ringkasan konfigurasi akhir global costmap"
  },
  {
    {
      "Konfigurasi", "Path length", "Min clearance", "Planning time", "Tracking RMSE", "Goal success"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Heading error", "derived.path_heading_error_rad"
    },
    {
      "Path length", "nav_path.length_m"
    }, {
      "Plan latency", "nav_path.planning_latency_ms"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.1",
  QStringLiteral("4.6.1 Pengujian minimum_turning_radius"),
  {
    "Pengaruh minimum_turning_radius"
  },
  {
    {
      "Rmin", "Planning time", "Path length", "Tracking RMSE", "Steering saturasi", "Status"
    }
  },
  {
    {
      "Gyro Z", "imu.gz"
    }, {
      "Yaw", "imu.yaw_rad"
    }, {
      "Yaw residual", "imu_status.yaw_residual"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.2",
  QStringLiteral("4.6.2 Pengujian downsampling_factor"),
  {
    "Trade-off downsampling dan tracking"
  },
  {
    {
      "Downsampling", "Planning time", "Min clearance", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.3",
  QStringLiteral("4.6.3 Pengujian angle_quantization_bins"),
  {
    "Pengaruh angle_quantization_bins"
  },
  {
    {
      "Bins", "Resolusi heading", "Planning time", "Tracking RMSE", "Variasi steering", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.4",
  QStringLiteral("4.6.4 Pengujian cost_penalty"),
  {
    "Pengaruh cost_penalty terhadap clearance dan panjang path"
  },
  {
    {
      "cost penalty", "Min clearance", "Path length", "Planning time", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.5",
  QStringLiteral("4.6.5 Pengujian analytic_expansion_max_length"),
  {
    "Pengaruh analytic_expansion_max_length"
  },
  {
    {
      "Max length", "Planning time", "Clearance dekat goal", "Final tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.6",
  QStringLiteral("4.6.6 Pengujian non_straight_penalty"),
  {
    "Pengaruh non_straight_penalty"
  },
  {
    {
      "Penalty", "Perubahan arah steering", "Path length", "Tracking RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian dan Tuning Smac Hybrid-A* Planner"), "4.6.7",
  QStringLiteral("4.6.7 Validasi Konfigurasi Akhir Smac Hybrid-A*"),
  {
    "Perbandingan bentuk path baseline dan hasil tuning"
  },
  {
    {
      "Parameter", "Baseline source", "Hasil tuning aktual", "Perubahan utama"
    },
    {
      "Metrik skenario gabungan", "Baseline", "Hasil tuning aktual", "Perubahan"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Heading error", "derived.path_heading_error_rad"
    },
    {
      "Path length", "nav_path.length_m"
    }, {
      "Plan latency", "nav_path.planning_latency_ms"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.1",
  QStringLiteral("4.7.1 Pengujian controller_frequency dan model_dt"),
  {
    "Pengaruh controller_frequency terhadap tracking dan CPU"
  },
  {
    {
      "Pasangan", "Tracking RMSE", "Latency", "CPU", "Miss cycle", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.2",
  QStringLiteral("4.7.2 Pengujian time_steps atau Prediction Horizon"),
  {
    "Pengaruh prediction horizon MPPI"
  },
  {
    {
      "time_steps", "CTE RMSE", "Max CTE", "Compute time", "Std steering", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.3",
  QStringLiteral("4.7.3 Pengujian PathAlignCritic cost_weight"),
  {
    "Pengaruh PathAlignCritic terhadap tracking"
  },
  {
    {
      "Weight", "CTE RMSE", "Heading RMSE", "Std steering", "Time-to-goal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.4",
  QStringLiteral("4.7.4 Pengujian PathFollowCritic cost_weight"),
  {
    "Pengaruh PathFollowCritic terhadap cross-track error"
  },
  {
    {
      "Weight", "CTE RMSE", "Max CTE", "Recovery time", "Std steering", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.5",
  QStringLiteral("4.7.5 Pengujian PathAngleCritic cost_weight"),
  {
    "Pengaruh PathAngleCritic terhadap heading error"
  },
  {
    {
      "Weight", "CTE RMSE", "Heading RMSE", "Std steering", "Time-to-goal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.6",
  QStringLiteral("4.7.6 Pengujian vx_max"),
  {
    "Trade-off vx_max terhadap waktu tempuh dan CTE"
  },
  {
    {
      "vx_max", "CTE RMSE", "Max CTE", "Waktu tempuh", "Steering saturasi", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.7",
  QStringLiteral("4.7.7 Konsistensi minimum_turning_r MPPI dengan Planner"),
  {
    "Validasi minimum_turning_radius MPPI"
  },
  {
    {
      "Rmin MPPI", "CTE RMSE", "Saturasi steering", "Radius aktual minimum", "Status"
    }
  },
  {
    {
      "Gyro Z", "imu.gz"
    }, {
      "Yaw", "imu.yaw_rad"
    }, {
      "Yaw residual", "imu_status.yaw_residual"
    }
  });
  add("navigation", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian dan Tuning MPPI Ackermann Controller"), "4.7.8",
  QStringLiteral("4.7.8 Validasi Konfigurasi Akhir MPPI"),
  {
    "Ringkasan konfigurasi akhir MPPI"
  },
  {
    {
      "Konfigurasi", "CTE RMSE", "Max CTE", "Heading RMSE", "Waktu", "Steering saturasi"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "v error", "derived.velocity_error_mps"
    },
    {
      "steer error", "derived.steering_error_rad"
    }, {
      "yaw error", "derived.yaw_error_rps"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.1",
  QStringLiteral("4.8.1 Pengujian Kecepatan Minimum Stabil Kendaraan"),
  {
    "Penentuan kecepatan minimum stabil"
  },
  {
    {
      "Command", "Actual speed", "Kontinu", "Gejala", "Status"
    }
  },
  {
    {
      "Gyro Z", "imu.gz"
    }, {
      "Yaw", "imu.yaw_rad"
    }, {
      "Yaw residual", "imu_status.yaw_residual"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.2",
  QStringLiteral("4.8.2 Pengujian Harmonisasi Low-Speed Deadband"),
  {
    "Pengaruh low-speed deadband terhadap final approach"
  },
  {
    {
      "Threshold set", "Endpoint error", "Start-stop/run", "Command dipotong", "Time-to-goal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.3",
  QStringLiteral("4.8.3 Pengujian smoothing_frequency"),
  {
    "Pengaruh smoothing_frequency pada command"
  },
  {
    {
      "Frekuensi", "Mean Δ command", "Latency", "Speed oscillation", "CTE RMSE", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.4",
  QStringLiteral("4.8.4 Pengujian max_accel dan max_decel"),
  {
    "Pengaruh acceleration/deceleration limit"
  },
  {
    {
      "Accel/Decel", "Overshoot speed", "Stop error", "Waktu", "Peak jerk", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian dan Tuning Pipeline Command dan Velocity Smoother"), "4.8.5",
  QStringLiteral("4.8.5 Validasi Konfigurasi Akhir Pipeline Command"),
  {
    "Ringkasan konfigurasi akhir pipeline command"
  },
  {
    {
      "Parameter", "Baseline", "Hasil tuning"
    }
  },
  {
    {
      "Nav v", "cmd_nav.linear_x"
    }, {
      "Integrated v", "cmd_autonomy_integrated.linear_x"
    },
    {
      "Final v", "cmd_final.linear_x"
    }, {
      "Actual v", "esc_drive_actual"
    }
  });
  add("navigation", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian Goal Checker dan Validasi Akurasi Navigasi"), "4.9.1",
  QStringLiteral("4.9.1 Pengujian xy_goal_tolerance"),
  {
    "Hubungan xy_goal_tolerance dan error posisi akhir"
  },
  {
    {
      "Tolerance", "Nav2 success", "Endpoint RMSE", "Max error", "Time-to-goal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian Goal Checker dan Validasi Akurasi Navigasi"), "4.9.2",
  QStringLiteral("4.9.2 Pengujian yaw_goal_tolerance"),
  {
    "Pengaruh yaw_goal_tolerance"
  },
  {
    {
      "Yaw tolerance", "Success", "Mean yaw error", "Time-to-goal", "Retry/reversal", "Status"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian Goal Checker dan Validasi Akurasi Navigasi"), "4.9.3",
  QStringLiteral("4.9.3 Validasi Navigasi End-to-End"),
  {
    "Sebaran posisi akhir pengujian berulang"
  },
  {
    {
      "Skenario", "CTE RMSE", "Heading RMSE", "Mean endpoint error", "Std endpoint", "Success"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian Goal Checker dan Validasi Akurasi Navigasi"), "4.9.4",
  QStringLiteral("4.9.4 Rekapitulasi Parameter Navigasi Akhir"),
  {
    "Ringkasan parameter navigasi akhir"
  },
  {
    {
      "Lapisan", "Parameter", "Kandidat hasil tuning"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  add("navigation", QStringLiteral("4.10"), QStringLiteral("4.10 Ringkasan Hubungan Hasil Pengujian"), "4.10",
  QStringLiteral("4.10 Ringkasan Hubungan Hasil Pengujian"),
  {
  },
  {
    {
      "Lapisan", "Ringkasan hasil", "Keterkaitan"
    }
  },
  {
    {
      "CTE", "derived.cte_m"
    }, {
      "Endpoint", "derived.endpoint_error_m"
    }, {
      "Heading", "derived.path_heading_error_rad"
    }
  });
  /* ------------------------- PERSEPSI ------------------------- */
  // FINAL BAB IV acquisition leaves.  The existing 4.1--4.11 perception
  // leaves below remain the detailed commissioning/tuning workspace.  These
  // four leaves intentionally mirror the narrowed thesis Chapter IV: four
  // tests x three variations, using only actual GUI/ROS data.
  add("perception", QStringLiteral("FINAL-4.1"), QStringLiteral("FINAL BAB IV — 4.1 Deteksi Obstacle YOLOPv2"), "F4.1",
  QStringLiteral("FINAL 4.1 Deteksi Obstacle — 1 m / 2 m / 3 m"),
  {"Grafik detection rate dan mean confidence terhadap jarak obstacle"},
  {{"Variasi","Jarak","Total Frame","Frame Terdeteksi","Detection Rate","Mean Confidence","False Detection","Missed Detection"}},
  {{"Target detected","derived.target_detected"},{"Raw detections","raw_detections.count"},{"Mean confidence","raw_detections.mean_confidence"}},
  {
    {"variation","Variasi","string","","","1 m / 2 m / 3 m"},
    {"condition","Kondisi/Lokasi","string","","","jalur dan pencahayaan"},
    {"gt_x","GT Forward (m)","float","","","1.00","",true},
    {"gt_y","GT Lateral (m)","float","","","0.00","",true},
    {"target_class_id","Target raw class ID","int","","","contoh: 0"},
    {"target_match_radius_m","Toleransi target (m)","float","","","0.50"},
    {"frame_target","Target unique frame","int","","","100","100",false,true},
    {"confidence_threshold","Raw confidence threshold","yaml_readonly","perception","perception.ros__parameters.confidence_threshold"},
    {"iou_threshold","NMS IoU threshold","yaml_readonly","perception","perception.ros__parameters.iou_threshold"},
    {"minimum_obstacle_confidence","Minimum obstacle confidence","yaml_readonly","perception","perception.ros__parameters.minimum_obstacle_confidence"},
    {"drivable_contact_min_fraction","Drivable contact fraction","yaml_readonly","perception","perception.ros__parameters.drivable_contact_min_fraction"},
    {"track_confirm_hits","Track confirm hits","yaml_readonly","perception","perception.ros__parameters.track_confirm_hits"},
    {"track_max_missed_frames","Track max missed","yaml_readonly","perception","perception.ros__parameters.track_max_missed_frames"},
    {"track_ema_alpha","Track EMA alpha","yaml_readonly","perception","perception.ros__parameters.track_ema_alpha"}
  });
  add("perception", QStringLiteral("FINAL-4.2"), QStringLiteral("FINAL BAB IV — 4.2 Akurasi Estimasi Posisi Obstacle"), "F4.2",
  QStringLiteral("FINAL 4.2 Estimasi Posisi — GT 1 m / 2 m / 3 m"),
  {"Grafik ground truth vs estimasi posisi","Grafik error absolut terhadap jarak"},
  {{"Variasi","Ground Truth","Mean Homography","Mean Tracked","Error Absolut","MAE","Std. Deviasi","Sampel"}},
  {{"Raw homography","obstacle_metrics.nearest_forward_homography_m"},{"Tracked forward","obstacle_metrics.nearest_forward_m"},{"Lateral","obstacle_metrics.nearest_left_m"}},
  {
    {"variation","Variasi","string","","","1 m / 2 m / 3 m"},
    {"condition","Kondisi/Lokasi","string","","","jalur dan pencahayaan"},
    {"gt_x","GT Forward (m)","float","","","1.00","",true},
    {"gt_y","GT Lateral (m)","float","","","0.00","",true},
    {"target_class_id","Target raw class ID","int","","","contoh: 0"},
    {"target_match_radius_m","Toleransi target (m)","float","","","0.70"},
    {"sample_target","Target sampel valid","int","","","30","30",false,true},
    {"metric_minimum_forward_m","Minimum forward","yaml_readonly","perception","perception.ros__parameters.metric_minimum_forward_m"},
    {"metric_maximum_forward_m","Maximum forward","yaml_readonly","perception","perception.ros__parameters.metric_maximum_forward_m"},
    {"metric_maximum_abs_left_m","Maximum abs lateral","yaml_readonly","perception","perception.ros__parameters.metric_maximum_abs_left_m"},
    {"track_ema_alpha","Track EMA alpha","yaml_readonly","perception","perception.ros__parameters.track_ema_alpha"}
  });
  add("perception", QStringLiteral("FINAL-4.3"), QStringLiteral("FINAL BAB IV — 4.3 Segmentasi Drivable Area dan Lane Line"), "F4.3",
  QStringLiteral("FINAL 4.3 Segmentasi — terang / berbayang / redup"),
  {"Grafik IoU drivable area dan lane line"},
  {{"Kondisi","IoU Drivable Area","IoU Lane Line","Lane Valid Rate","Valid Rows","Lane Confidence","Center Error","Lane State"}},
  {{"Lane valid","lane_state.valid"},{"Center error","lane_state.center_error_m"},{"Valid rows","drivable_space.valid_rows"}},
  {
    {"variation","Variasi","string","","","1 / 2 / 3"},
    {"condition","Kondisi Visual","string","","","Terang / Berbayang / Redup"},
    {"lane_threshold","Lane threshold","yaml_readonly","perception","perception.ros__parameters.lane_threshold"},
    {"lane_ema_alpha","Lane EMA alpha","yaml_readonly","perception","perception.ros__parameters.lane_ema_alpha"},
    {"publish_drivable_mask","Publish drivable mask","yaml_readonly","perception","perception.ros__parameters.publish_drivable_mask"},
    {"publish_lane_mask","Publish lane mask","yaml_readonly","perception","perception.ros__parameters.publish_lane_mask"}
  });
  add("perception", QStringLiteral("FINAL-4.4"), QStringLiteral("FINAL BAB IV — 4.4 Real-Time dan Integrasi"), "F4.4",
  QStringLiteral("FINAL 4.4 Real-Time — kosong / obstacle / near-field"),
  {"Grafik pipeline FPS","Grafik mean dan P95 processFrame","Grafik resource Jetson"},
  {
    {"Kondisi","Candidate","Path Relevant","Planning Relevant","Keputusan","Success"},
    {"Kondisi","Pipeline FPS","Mean processFrame","P95 processFrame","Capture Drop","Median Response"},
    {"Kondisi","GPU","RAM","Temperatur"}
  },
  {{"FPS","perception_performance.fps"},{"Mean ms","perception_performance.mean_ms"},{"P95 ms","perception_performance.p95_ms"},{"GPU","host.gpu_percent"},{"Temperature","host.temperature_c"}},
  {
    {"variation","Variasi","string","","","1 / 2 / 3"},
    {"condition","Skenario","string","","","Jalur kosong / Obstacle di jalur / Obstacle sangat dekat"},
    {"trial_target","Target trial","int","","","20","20",false,true},
    {"performance_window","Window performa (frame)","int","","","300","300",false,true},
    {"near_field_confirm_frames","Near-field confirm","yaml_readonly","perception","perception.ros__parameters.near_field_confirm_frames"},
    {"near_field_release_frames","Near-field release","yaml_readonly","perception","perception.ros__parameters.near_field_release_frames"}
  });

  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Pipeline Kamera dan YOLOPv2"), "4.1.1",
  QStringLiteral("4.1.1 Arsitektur Pipeline Persepsi"),
  {
    "Gambar 4.1 Diagram alir pipeline sistem persepsi visual"
  },
  {
    {
      "Tahap", "Input", "Proses", "Output", "Topic"
    }
  },
  {
    {
      "Raw detections", "raw_detections.count"
    }, {
      "Metric candidates", "obstacle_metrics.count"
    },
    {
      "Mean confidence", "raw_detections.mean_confidence"
    }, {
      "Nearest X", "obstacle_metrics.nearest_forward_m"
    }
  });
  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Pipeline Kamera dan YOLOPv2"), "4.1.2",
  QStringLiteral("4.1.2 Konfigurasi Kamera dan Runtime"),
  {
  },
  {
    {
      "Parameter", "Nilai"
    }
  },
  {
    {
      "Raw detections", "raw_detections.count"
    }, {
      "Metric candidates", "obstacle_metrics.count"
    },
    {
      "Mean confidence", "raw_detections.mean_confidence"
    }, {
      "Nearest X", "obstacle_metrics.nearest_forward_m"
    }
  });
  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Pipeline Kamera dan YOLOPv2"), "4.1.3",
  QStringLiteral("4.1.3 Pengujian Mode Capture Kamera"),
  {
    "Pipeline FPS berdasarkan mode capture"
  },
  {
    {
      "Mode Capture", "Camera Rate", "Pipeline FPS", "Mean processFrame", "P95 processFrame", "Capture Drop/5 Menit", "Frame Korup"
    }
  },
  {
    {
      "FPS", "perception_performance.fps"
    }, {
      "Mean ms", "perception_performance.mean_ms"
    },
    {
      "P95 ms", "perception_performance.p95_ms"
    }, {
      "Capture drop", "perception_performance.capture_dropped"
    }
  });
  add("perception", QStringLiteral("4.1"), QStringLiteral("4.1 Pengujian Pipeline Kamera dan YOLOPv2"), "4.1.4",
  QStringLiteral("4.1.4 Pengujian Batas Publish Visualisasi"),
  {
    "Ringkasan publish rate terhadap FPS dan drop"
  },
  {
    {
      "Publish Rate", "Pipeline FPS", "Mean processFrame", "RViz Drop", "Kualitas Monitoring"
    }
  },
  {
    {
      "FPS", "perception_performance.fps"
    }, {
      "Mean ms", "perception_performance.mean_ms"
    },
    {
      "P95 ms", "perception_performance.p95_ms"
    }, {
      "Capture drop", "perception_performance.capture_dropped"
    }
  });
  add("perception", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Deteksi Obstacle"), "4.2.1",
  QStringLiteral("4.2.1 Pengujian Minimum Obstacle Confidence"),
  {
    "Detection rate terhadap minimum obstacle confidence"
  },
  {
    {
      "Threshold", "Detection Rate", "False Candidate/Frame", "Miss Rate", "Analisis"
    }
  },
  {
    {
      "Raw detections", "raw_detections.count"
    }, {
      "Metric candidates", "obstacle_metrics.count"
    },
    {
      "Mean confidence", "raw_detections.mean_confidence"
    }, {
      "Nearest X", "obstacle_metrics.nearest_forward_m"
    }
  });
  add("perception", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Deteksi Obstacle"), "4.2.2",
  QStringLiteral("4.2.2 Pengujian Drivable-Contact Filter"),
  {
  },
  {
    {
      "Minimum Fraction", "Obstacle di Jalan Lolos", "Background Rejection", "Miss akibat Mask", "Analisis"
    }
  },
  {
    {
      "Valid rows", "drivable_space.valid_rows"
    }, {
      "Boundary points", "drivable_boundary_points.count"
    },
    {
      "Left clearance", "lane_state.left_clearance_m"
    }, {
      "Right clearance", "lane_state.right_clearance_m"
    }
  });
  add("perception", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Deteksi Obstacle"), "4.2.3",
  QStringLiteral("4.2.3 Pengujian Konfirmasi Temporal Obstacle"),
  {
  },
  {
    {
      "Confirm Hits", "False Track/Trial", "Delay Konfirmasi", "Detection Stabil"
    }
  },
  {
    {
      "Raw detections", "raw_detections.count"
    }, {
      "Metric candidates", "obstacle_metrics.count"
    },
    {
      "Mean confidence", "raw_detections.mean_confidence"
    }, {
      "Nearest X", "obstacle_metrics.nearest_forward_m"
    }
  });
  add("perception", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Deteksi Obstacle"), "4.2.4",
  QStringLiteral("4.2.4 Pengujian Toleransi Detection Dropout"),
  {
  },
  {
    {
      "Max Missed Frame", "Obstacle Flicker", "Estimasi Hold Time", "Risiko Stale"
    }
  },
  {
    {
      "Raw detections", "raw_detections.count"
    }, {
      "Metric candidates", "obstacle_metrics.count"
    },
    {
      "Mean confidence", "raw_detections.mean_confidence"
    }, {
      "Nearest X", "obstacle_metrics.nearest_forward_m"
    }
  });
  add("perception", QStringLiteral("4.2"), QStringLiteral("4.2 Pengujian Deteksi Obstacle"), "4.2.5",
  QStringLiteral("4.2.5 Pengujian Target Fisik terhadap Jarak"),
  {
    "Detection rate berdasarkan jarak target"
  },
  {
    {
      "Jarak", "Jumlah Frame", "Frame Terdeteksi", "Detection Rate", "Mean Confidence"
    },
    {
      "Jarak", "Jumlah Frame", "Frame Terdeteksi", "Detection Rate", "Mean Confidence"
    },
    {
      "Jarak", "Jumlah Frame", "Frame Terdeteksi", "Detection Rate", "Mean Confidence"
    }
  },
  {
    {
      "Raw detections", "raw_detections.count"
    }, {
      "Metric candidates", "obstacle_metrics.count"
    },
    {
      "Mean confidence", "raw_detections.mean_confidence"
    }, {
      "Nearest X", "obstacle_metrics.nearest_forward_m"
    }
  });
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Kalibrasi & Estimasi Jarak Obstacle"), "4.3.1",
  QStringLiteral("4.3.1 Kalibrasi Obstacle 1–5 Meter (Human & Motorcycle × 4 Orientasi)"),
  {
  },
  {
    {
      "Posisi Lateral", "X = 1 m", "X = 2 m", "X = 3 m"
    }
  },
  {
    {
      "Raw detections", "raw_detections.count"
    }, {
      "Metric candidates", "obstacle_metrics.count"
    },
    {
      "Mean confidence", "raw_detections.mean_confidence"
    }, {
      "Nearest X", "obstacle_metrics.nearest_forward_m"
    }
  });
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Kalibrasi & Estimasi Jarak Obstacle"), "4.3.2",
  QStringLiteral("4.3.2 Hasil Pengujian Ground-Plane Homography"),
  {
    "Perbandingan ground truth dan hasil estimasi homography", "Error posisi obstacle terhadap jarak"
  },
  {
    {
      "GT X", "GT Y", "Sistem X", "Sistem Y", "Error X", "Error Y", "Error 2D"
    }
  },
  {
    {
      "Obstacle X", "obstacle_metrics.nearest_forward_m"
    }, {
      "Obstacle Y", "obstacle_metrics.nearest_left_m"
    },
    {
      "Error 2D", "derived.obstacle_error_2d_m"
    }, {
      "Confidence", "obstacle_metrics.mean_confidence"
    }
  });
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Kalibrasi & Estimasi Jarak Obstacle"), "4.3.3",
  QStringLiteral("4.3.3 Kalibrasi Offset Posisi"),
  {
  },
  {
    {
      "Titik", "GT X", "GT Y", "Sistem X", "Sistem Y", "Offset X", "Offset Y", "Status"
    }
  },
  {
    {
      "Obstacle X", "obstacle_metrics.nearest_forward_m"
    }, {
      "Obstacle Y", "obstacle_metrics.nearest_left_m"
    },
    {
      "Error 2D", "derived.obstacle_error_2d_m"
    }, {
      "Confidence", "obstacle_metrics.mean_confidence"
    }
  });
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Kalibrasi & Estimasi Jarak Obstacle"), "4.3.4",
  QStringLiteral("4.3.4 Pengujian EMA Posisi Obstacle"),
  {
  },
  {
    {
      "Alpha", "Standar Deviasi X", "Settling Time", "RMSE 2D", "Analisis"
    }
  },
  {
    {
      "Obstacle X", "obstacle_metrics.nearest_forward_m"
    }, {
      "Obstacle Y", "obstacle_metrics.nearest_left_m"
    },
    {
      "Error 2D", "derived.obstacle_error_2d_m"
    }, {
      "Confidence", "obstacle_metrics.mean_confidence"
    }
  });
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Kalibrasi & Estimasi Jarak Obstacle"), "4.3.5",
  QStringLiteral("4.3.5 Evaluasi Metode Alternatif Bounding Box Area"),
  {
  },
  {
    {
      "Metode", "MAE X", "MAE Y", "RMSE 2D", "Status"
    }
  },
  {
    {
      "Obstacle X", "obstacle_metrics.nearest_forward_m"
    }, {
      "Obstacle Y", "obstacle_metrics.nearest_left_m"
    },
    {
      "Error 2D", "derived.obstacle_error_2d_m"
    }, {
      "Confidence", "obstacle_metrics.mean_confidence"
    }
  });
  add("perception", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian Drivable Area"), "4.4.1",
  QStringLiteral("4.4.1 Pengujian Segmentasi Drivable Area"),
  {
    "Perbandingan citra asli, ground truth, dan drivable-area segmentation"
  },
  {
    {
      "Kondisi", "IoU", "Pixel Accuracy", "False Drivable", "Analisis"
    }
  },
  {
    {
      "Valid rows", "drivable_space.valid_rows"
    }, {
      "Boundary points", "drivable_boundary_points.count"
    },
    {
      "Left clearance", "lane_state.left_clearance_m"
    }, {
      "Right clearance", "lane_state.right_clearance_m"
    }
  });
  add("perception", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian Drivable Area"), "4.4.2",
  QStringLiteral("4.4.2 Pengujian Ruang Drivable dalam Koordinat Metrik"),
  {
  },
  {
    {
      "Jarak Lookahead", "GT Lebar Bebas", "Sistem", "Error"
    }
  },
  {
    {
      "Valid rows", "drivable_space.valid_rows"
    }, {
      "Boundary points", "drivable_boundary_points.count"
    },
    {
      "Left clearance", "lane_state.left_clearance_m"
    }, {
      "Right clearance", "lane_state.right_clearance_m"
    }
  });
  add("perception", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian Lane Detection dan Lane Safety"), "4.5.1",
  QStringLiteral("4.5.1 Pengujian Lane Threshold"),
  {
  },
  {
    {
      "Threshold", "Valid Lane", "Std Center Error", "False Lane Pixel", "State Accuracy"
    }
  },
  {
    {
      "Center error", "lane_state.center_error_m"
    }, {
      "Left clearance", "lane_state.left_clearance_m"
    },
    {
      "Right clearance", "lane_state.right_clearance_m"
    }, {
      "Heading", "lane_state.heading_error_rad"
    }
  });
  add("perception", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian Lane Detection dan Lane Safety"), "4.5.2",
  QStringLiteral("4.5.2 Pengujian EMA Lane"),
  {
  },
  {
    {
      "Alpha", "Std Center Error", "Response Time", "State Chatter"
    }
  },
  {
    {
      "Center error", "lane_state.center_error_m"
    }, {
      "Left clearance", "lane_state.left_clearance_m"
    },
    {
      "Right clearance", "lane_state.right_clearance_m"
    }, {
      "Heading", "lane_state.heading_error_rad"
    }
  });
  add("perception", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian Lane Detection dan Lane Safety"), "4.5.3",
  QStringLiteral("4.5.3 Pengujian Threshold Keselamatan Lane"),
  {
  },
  {
    {
      "Set", "Warning", "Critical", "Release"
    },
    {
      "Set", "State Accuracy", "False Warning", "Missed Warning"
    }
  },
  {
    {
      "Center error", "lane_state.center_error_m"
    }, {
      "Left clearance", "lane_state.left_clearance_m"
    },
    {
      "Right clearance", "lane_state.right_clearance_m"
    }, {
      "Heading", "lane_state.heading_error_rad"
    }
  });
  add("perception", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian Lane Detection dan Lane Safety"), "4.5.4",
  QStringLiteral("4.5.4 Validasi Posisi Kendaraan terhadap Lane"),
  {
  },
  {
    {
      "Kondisi", "GT Offset", "GT Left", "GT Right", "System Left", "System Right", "Center Error", "State"
    }
  },
  {
    {
      "Center error", "lane_state.center_error_m"
    }, {
      "Left clearance", "lane_state.left_clearance_m"
    },
    {
      "Right clearance", "lane_state.right_clearance_m"
    }, {
      "Heading", "lane_state.heading_error_rad"
    }
  });
  add("perception", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian Camera Health dan Near-Field Emergency"), "4.6.1",
  QStringLiteral("4.6.1 Pengujian Kondisi Kamera"),
  {
  },
  {
    {
      "Kondisi Kamera", "Healthy Detection", "Detection Success", "Response"
    }
  },
  {
    {
      "Mean luma", "camera_health_state.mean_luma"
    }, {
      "Std luma", "camera_health_state.stddev_luma"
    },
    {
      "Gradient", "camera_health_state.mean_gradient"
    }, {
      "Healthy", "camera_healthy"
    }
  });
  add("perception", QStringLiteral("4.6"), QStringLiteral("4.6 Pengujian Camera Health dan Near-Field Emergency"), "4.6.2",
  QStringLiteral("4.6.2 Pengujian Near-Field Emergency"),
  {
  },
  {
    {
      "Kondisi", "Jarak Obstacle", "Trigger Emergency", "Success"
    }
  },
  {
    {
      "Emergency", "perception_emergency"
    }, {
      "Confidence", "near_field_state.confidence"
    },
    {
      "Drivable fraction", "near_field_state.near_field_drivable_fraction"
    }
  });
  add("perception", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian Integrasi Persepsi dengan Trajectory Safety"), "4.7.1",
  QStringLiteral("4.7.1 Pengujian Path-Relevant Obstacle"),
  {
  },
  {
    {
      "Margin", "Relevant Recall", "False Path-Relevant", "Analisis"
    }
  },
  {
    {
      "Candidate", "object_points.count"
    }, {
      "Path relevant", "path_relevant_points.count"
    },
    {
      "Planning relevant", "planning_relevant_points.count"
    }, {
      "Speed scale", "trajectory_safety_state.speed_scale"
    }
  });
  add("perception", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian Integrasi Persepsi dengan Trajectory Safety"), "4.7.2",
  QStringLiteral("4.7.2 Pengujian Planning-Relevant Obstacle"),
  {
  },
  {
    {
      "Margin", "Obstacle Masuk Costmap", "Bypass Success", "False Planning Burden"
    }
  },
  {
    {
      "Candidate", "object_points.count"
    }, {
      "Path relevant", "path_relevant_points.count"
    },
    {
      "Planning relevant", "planning_relevant_points.count"
    }, {
      "Speed scale", "trajectory_safety_state.speed_scale"
    }
  });
  add("perception", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian Integrasi Persepsi dengan Trajectory Safety"), "4.7.3",
  QStringLiteral("4.7.3 Pengujian Slowdown Distance"),
  {
  },
  {
    {
      "Distance", "Aktual Mulai Melambat", "Bypass Success", "Stop Mendadak"
    }
  },
  {
    {
      "Candidate", "object_points.count"
    }, {
      "Path relevant", "path_relevant_points.count"
    },
    {
      "Planning relevant", "planning_relevant_points.count"
    }, {
      "Speed scale", "trajectory_safety_state.speed_scale"
    }
  });
  add("perception", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian Integrasi Persepsi dengan Trajectory Safety"), "4.7.4",
  QStringLiteral("4.7.4 Pengujian Hard Stop"),
  {
  },
  {
    {
      "Set", "Immediate", "Path Stop", "Stop Success", "Residual Clearance"
    }
  },
  {
    {
      "Candidate", "object_points.count"
    }, {
      "Path relevant", "path_relevant_points.count"
    },
    {
      "Planning relevant", "planning_relevant_points.count"
    }, {
      "Speed scale", "trajectory_safety_state.speed_scale"
    }
  });
  add("perception", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian Integrasi Persepsi dengan Trajectory Safety"), "4.7.5",
  QStringLiteral("4.7.5 Validasi Skenario Integrasi"),
  {
  },
  {
    {
      "Skenario", "Candidate", "Path Relevant", "Planning Relevant", "Keputusan", "Outcome"
    }
  },
  {
    {
      "Candidate", "object_points.count"
    }, {
      "Path relevant", "path_relevant_points.count"
    },
    {
      "Planning relevant", "planning_relevant_points.count"
    }, {
      "Speed scale", "trajectory_safety_state.speed_scale"
    }
  });
  add("perception", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian Performa Real-Time"), "4.8.1",
  QStringLiteral("4.8.1 Hasil Pengujian Performa"),
  {
    "Pipeline FPS pada setiap kondisi pengujian", "Mean dan P95 waktu processFrame", "Penggunaan GPU, RAM, dan temperatur"
  },
  {
    {
      "Kondisi", "Pipeline FPS", "Mean", "P95", "Capture Drop", "GPU", "RAM", "Temperatur"
    }
  },
  {
    {
      "FPS", "perception_performance.fps"
    }, {
      "Mean ms", "perception_performance.mean_ms"
    },
    {
      "P95 ms", "perception_performance.p95_ms"
    }, {
      "Capture drop", "perception_performance.capture_dropped"
    }
  });
  add("perception", QStringLiteral("4.9"), QStringLiteral("4.9 Pengujian End-to-End Sistem Persepsi"), "4.9",
  QStringLiteral("4.9 Pengujian End-to-End Sistem Persepsi"),
  {
  },
  {
    {
      "ID", "Skenario", "Ekspektasi"
    },
    {
      "Skenario", "Success/20", "Success Rate", "Median Response"
    }
  },
  {
    {
      "Candidate", "object_points.count"
    }, {
      "Path relevant", "path_relevant_points.count"
    },
    {
      "Planning relevant", "planning_relevant_points.count"
    }, {
      "Speed scale", "trajectory_safety_state.speed_scale"
    }
  });
  add("perception", QStringLiteral("4.10"), QStringLiteral("4.10 Rekapitulasi Konfigurasi Sistem Persepsi"), "4.10",
  QStringLiteral("4.10 Rekapitulasi Konfigurasi Sistem Persepsi"),
  {
  },
  {
    {
      "Bagian", "Parameter", "Nilai"
    }
  },
  {
    {
      "Raw detections", "raw_detections.count"
    }, {
      "Metric candidates", "obstacle_metrics.count"
    },
    {
      "Mean confidence", "raw_detections.mean_confidence"
    }, {
      "Nearest X", "obstacle_metrics.nearest_forward_m"
    }
  });
  add("perception", QStringLiteral("4.11"), QStringLiteral("4.11 Analisis Keseluruhan Sistem Persepsi"), "4.11",
  QStringLiteral("4.11 Analisis Keseluruhan Sistem Persepsi"),
  {
  },
  {
    {
      "Metrik", "Hasil aktual", "Kriteria", "Status", "Catatan"
    }
  },
  {
    {
      "Candidate", "object_points.count"
    }, {
      "Path relevant", "path_relevant_points.count"
    },
    {
      "Planning relevant", "planning_relevant_points.count"
    }, {
      "Speed scale", "trajectory_safety_state.speed_scale"
    }
  });
  /* -------------------- VESC / ACKERMANN ACTUATION -------------------- */
  // Ordered by physical dependency. LEFT is steering with ABI encoder on TIM4
  // PB6/PB7; RIGHT is traction/velocity with Hall feedback. Do not tune an
  // outer loop before its sensor and inner FOC loop have passed the prior gate.
  add("steering", QStringLiteral("4.1"), QStringLiteral("4.1 Metode Pengujian VESC Ackermann"), "4.1",
      QStringLiteral("4.1 Alur Pengujian Ackermann dan Aturan Penguncian Parameter"), {},
      {{"Tahap","Motor / fungsi","Prasyarat PASS","Parameter yang boleh diubah","Parameter yang dikunci","Keluaran / handoff"}}, {}, {});

  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Kalibrasi Dasar dan Validasi Sensor Motor"), "4.2.1",
      QStringLiteral("4.2.1 Verifikasi Link F411, FW VESC LEFT/RIGHT, dan Snapshot Konfigurasi"), {},
      {{"Variasi","LEFT Vbus","LEFT Fault","RIGHT Vbus","RIGHT Fault","LEFT Reply","RIGHT Reply"}},
      {{"LEFT Vbus","vesc_left_values.vbus_v"},{"LEFT Fault","vesc_left_values.fault"},{"RIGHT Vbus","vesc_right_values.vbus_v"},{"RIGHT Fault","vesc_right_values.fault"}}, {});

  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Kalibrasi Dasar dan Validasi Sensor Motor"), "4.2.2",
      QStringLiteral("4.2.2 Kalibrasi Sensor Arus dan Tegangan Bus Dual Motor"),
      {"Arus dan Vbus LEFT/RIGHT saat validasi zero-current"},
      {{"Variasi","LEFT Motor Current","LEFT Input Current","LEFT Vbus","RIGHT Motor Current","RIGHT Input Current","RIGHT Vbus","Status"}},
      {{"LEFT Motor Current","vesc_left_values.current_motor_a"},{"LEFT Input Current","vesc_left_values.current_in_a"},{"LEFT Vbus","vesc_left_values.vbus_v"},{"RIGHT Motor Current","vesc_right_values.current_motor_a"},{"RIGHT Input Current","vesc_right_values.current_in_a"},{"RIGHT Vbus","vesc_right_values.vbus_v"}}, {},
      {{"time_series",{"LEFT Motor Current","LEFT Input Current","RIGHT Motor Current","RIGHT Input Current","LEFT Vbus","RIGHT Vbus"},{},{},"Time [s]","A / V"}});

  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Kalibrasi Dasar dan Validasi Sensor Motor"), "4.2.3",
      QStringLiteral("4.2.3 LEFT — Validasi Encoder ABI TIM4, Hard Stop, Center, dan Home"),
      {"Raw TIM4 dan posisi steering terhadap waktu"},
      {{"Variasi","Raw TIM4","LEFT Position","Steering target","Steering actual","Span","Encoder synced","Status"}},
      {{"Raw TIM4","vesc_steering_state.raw_encoder"},{"LEFT Position","vesc_left_values.position_deg"},{"Steering target","esc_steer_target"},{"Steering actual","esc_steer_actual"},{"Span","vesc_steering_state.span"}}, {},
      {{"time_series",{"Raw TIM4","LEFT Position","Steering target","Steering actual"},{},{},"Time [s]","count / deg"}});

  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Kalibrasi Dasar dan Validasi Sensor Motor"), "4.2.4",
      QStringLiteral("4.2.4 LEFT — Deteksi Encoder FOC dan Kalibrasi Sudut Elektrik"),
      {"Respons D/Q saat alignment encoder LEFT"},
      {{"Variasi","LEFT Id","LEFT Iq","LEFT Vd","LEFT Vq","LEFT Position","Raw TIM4","Status"}},
      {{"LEFT Id","vesc_left_values.id_a"},{"LEFT Iq","vesc_left_values.iq_a"},{"LEFT Vd","vesc_left_values.vd_v"},{"LEFT Vq","vesc_left_values.vq_v"},{"LEFT Position","vesc_left_values.position_deg"},{"Raw TIM4","vesc_steering_state.raw_encoder"}}, {},
      {{"time_series",{"LEFT Id","LEFT Iq","LEFT Vd","LEFT Vq","LEFT Position"},{},{},"Time [s]","A / V / deg"}});

  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Kalibrasi Dasar dan Validasi Sensor Motor"), "4.2.5",
      QStringLiteral("4.2.5 RIGHT — Validasi Hall, Detect Hall, Arah, dan Pole Pair"),
      {"Respons Hall RIGHT pada putaran commissioning rendah"},
      {{"Variasi","Command value","RIGHT RPM","RIGHT Duty","RIGHT Motor Current","RIGHT Iq","RIGHT Fault","Status"}},
      {{"Command value","vesc_command_state.value"},{"RIGHT RPM","vesc_right_values.rpm"},{"RIGHT Duty","vesc_right_values.duty"},{"RIGHT Motor Current","vesc_right_values.current_motor_a"},{"RIGHT Iq","vesc_right_values.iq_a"},{"RIGHT Fault","vesc_right_values.fault"}}, {},
      {{"time_series",{"Command value","RIGHT RPM","RIGHT Duty","RIGHT Motor Current","RIGHT Iq"},{},{},"Time [s]","command / feedback"}});

  add("steering", QStringLiteral("4.3"), QStringLiteral("4.3 Tuning Loop Arus FOC per Motor"), "4.3.1",
      QStringLiteral("4.3.1 LEFT — Tuning PI Arus FOC Steering"),
      {"LEFT Iq reference dan feedback","LEFT Id terhadap waktu"},
      {{"Variasi","LEFT Iq","LEFT Id","LEFT Motor Current","LEFT Duty","LEFT Vbus","Status"}},
      {{"LEFT Iq","vesc_left_values.iq_a"},{"LEFT Id","vesc_left_values.id_a"},{"LEFT Motor Current","vesc_left_values.current_motor_a"},{"LEFT Duty","vesc_left_values.duty"},{"LEFT Vbus","vesc_left_values.vbus_v"}}, {},
      {{"time_series",{"LEFT Iq","LEFT Motor Current","LEFT Duty"},{},{},"Time [s]","A / duty"},{"time_series",{"LEFT Id"},{},{},"Time [s]","Id [A]"}});

  add("steering", QStringLiteral("4.3"), QStringLiteral("4.3 Tuning Loop Arus FOC per Motor"), "4.3.2",
      QStringLiteral("4.3.2 RIGHT — Tuning PI Arus FOC Drive"),
      {"RIGHT Iq dan motor current","RIGHT duty dan Vbus"},
      {{"Variasi","RIGHT Iq","RIGHT Id","RIGHT Motor Current","RIGHT Duty","RIGHT RPM","RIGHT Vbus","Status"}},
      {{"RIGHT Iq","vesc_right_values.iq_a"},{"RIGHT Id","vesc_right_values.id_a"},{"RIGHT Motor Current","vesc_right_values.current_motor_a"},{"RIGHT Duty","vesc_right_values.duty"},{"RIGHT RPM","vesc_right_values.rpm"},{"RIGHT Vbus","vesc_right_values.vbus_v"}}, {},
      {{"time_series",{"RIGHT Iq","RIGHT Motor Current","RIGHT Duty"},{},{},"Time [s]","A / duty"},{"time_series",{"RIGHT RPM","RIGHT Vbus"},{},{},"Time [s]","eRPM / V"}});

  add("steering", QStringLiteral("4.4"), QStringLiteral("4.4 Tuning Outer Loop Steering dan Velocity"), "4.4.1",
      QStringLiteral("4.4.1 LEFT — Tuning PID Posisi Steering"),
      {"Target dan feedback steering","Error steering dan Iq"},
      {{"Variasi","Steering target","Steering actual","Steering error","LEFT Iq","LEFT Duty","Status"}},
      {{"Steering target","esc_steer_target"},{"Steering actual","esc_steer_actual"},{"Steering error","derived.steering_error_rad"},{"LEFT Iq","vesc_left_values.iq_a"},{"LEFT Duty","vesc_left_values.duty"}}, {},
      {{"time_series",{"Steering target","Steering actual"},{},{},"Time [s]","Steering [rad]"},{"time_series",{"Steering error","LEFT Iq"},{},{},"Time [s]","error / A"}});

  add("steering", QStringLiteral("4.4"), QStringLiteral("4.4 Tuning Outer Loop Steering dan Velocity"), "4.4.2",
      QStringLiteral("4.4.2 RIGHT — Tuning PID Speed Velocity"),
      {"RIGHT RPM command dan feedback","RIGHT current saat speed control"},
      {{"Variasi","Command value","RIGHT RPM","RIGHT Motor Current","RIGHT Iq","RIGHT Duty","Status"}},
      {{"Command value","vesc_command_state.value"},{"RIGHT RPM","vesc_right_values.rpm"},{"RIGHT Motor Current","vesc_right_values.current_motor_a"},{"RIGHT Iq","vesc_right_values.iq_a"},{"RIGHT Duty","vesc_right_values.duty"}}, {},
      {{"time_series",{"Command value","RIGHT RPM"},{},{},"Time [s]","eRPM"},{"time_series",{"RIGHT Motor Current","RIGHT Iq","RIGHT Duty"},{},{},"Time [s]","A / duty"}});

  add("steering", QStringLiteral("4.5"), QStringLiteral("4.5 Respons Step Aktuator"), "4.5.1",
      QStringLiteral("4.5.1 LEFT — Respons Step Posisi Steering ±10°, ±20°, ±28°"),
      {"Target dan feedback step steering","Error steering","Iq dan duty steering","Raw encoder TIM4"},
      {{"Variasi","Steering target","Steering actual","Steering error","LEFT Iq","LEFT Duty","Raw TIM4","Status"}},
      {{"Steering target","esc_steer_target"},{"Steering actual","esc_steer_actual"},{"Steering error","derived.steering_error_rad"},{"LEFT Iq","vesc_left_values.iq_a"},{"LEFT Duty","vesc_left_values.duty"},{"Raw TIM4","vesc_steering_state.raw_encoder"}}, {},
      {{"time_series",{"Steering target","Steering actual"},{},{},"Time [s]","Steering [rad]"},{"time_series",{"Steering error"},{},{},"Time [s]","Error [rad]"},{"time_series",{"LEFT Iq","LEFT Duty"},{},{},"Time [s]","A / duty"},{"time_series",{"Raw TIM4"},{},{},"Time [s]","TIM4 count"}});

  add("steering", QStringLiteral("4.5"), QStringLiteral("4.5 Respons Step Aktuator"), "4.5.2",
      QStringLiteral("4.5.2 RIGHT — Respons Step Duty, Current, dan RPM"),
      {"RIGHT command dan RPM","RIGHT current","RIGHT duty dan Vbus"},
      {{"Variasi","Command mode","Command value","RIGHT RPM","RIGHT Duty","RIGHT Motor Current","RIGHT Input Current","RIGHT Iq","RIGHT Vbus","Status"}},
      {{"Command value","vesc_command_state.value"},{"RIGHT RPM","vesc_right_values.rpm"},{"RIGHT Duty","vesc_right_values.duty"},{"RIGHT Motor Current","vesc_right_values.current_motor_a"},{"RIGHT Input Current","vesc_right_values.current_in_a"},{"RIGHT Iq","vesc_right_values.iq_a"},{"RIGHT Vbus","vesc_right_values.vbus_v"}}, {},
      {{"time_series",{"Command value","RIGHT RPM"},{},{},"Time [s]","command / eRPM"},{"time_series",{"RIGHT Motor Current","RIGHT Input Current","RIGHT Iq"},{},{},"Time [s]","Current [A]"},{"time_series",{"RIGHT Duty","RIGHT Vbus"},{},{},"Time [s]","duty / V"}});

  add("steering", QStringLiteral("4.6"), QStringLiteral("4.6 Validasi Steering dan Geometri Ackermann"), "4.6.1",
      QStringLiteral("4.6.1 Simetri Steering Kiri dan Kanan"),
      {"Simetri target dan feedback steering"},
      {{"Variasi","Steering target","Steering actual","Steering error","Raw TIM4","LEFT Iq"}},
      {{"Steering target","esc_steer_target"},{"Steering actual","esc_steer_actual"},{"Steering error","derived.steering_error_rad"},{"Raw TIM4","vesc_steering_state.raw_encoder"},{"LEFT Iq","vesc_left_values.iq_a"}}, {},
      {{"time_series",{"Steering target","Steering actual","Steering error"},{},{},"Time [s]","Steering [rad]"}});

  add("steering", QStringLiteral("4.6"), QStringLiteral("4.6 Validasi Steering dan Geometri Ackermann"), "4.6.2",
      QStringLiteral("4.6.2 Repeatability Steering pada Pengulangan Kiri/Center/Kanan"),
      {"Repeatability steering dan raw encoder"},
      {{"Variasi","Steering target","Steering actual","Steering error","Raw TIM4","LEFT Iq","Status"}},
      {{"Steering target","esc_steer_target"},{"Steering actual","esc_steer_actual"},{"Steering error","derived.steering_error_rad"},{"Raw TIM4","vesc_steering_state.raw_encoder"},{"LEFT Iq","vesc_left_values.iq_a"}}, {},
      {{"time_series",{"Steering target","Steering actual","Raw TIM4"},{},{},"Time [s]","rad / count"}});

  add("steering", QStringLiteral("4.6"), QStringLiteral("4.6 Validasi Steering dan Geometri Ackermann"), "4.6.3",
      QStringLiteral("4.6.3 Validasi Geometri Ackermann dengan Steering LEFT dan Velocity RIGHT"),
      {"Yaw-rate aktual terhadap model Ackermann","Steering dan kecepatan kendaraan"},
      {{"Variasi","Steering actual","Vehicle speed","Yaw rate actual","Yaw rate Ackermann","Yaw-rate error","RIGHT RPM","Status"}},
      {{"Steering actual","esc_steer_actual"},{"Vehicle speed","esc_drive_actual"},{"Yaw rate actual","esc_yaw_rate"},{"Yaw rate Ackermann","esc_kinematic_yaw_rate"},{"Yaw-rate error","derived.ackermann_yaw_error"},{"RIGHT RPM","vesc_right_values.rpm"}}, {},
      {{"time_series",{"Yaw rate actual","Yaw rate Ackermann"},{},{},"Time [s]","Yaw-rate [rad/s]"},{"time_series",{"Steering actual","Vehicle speed"},{},{},"Time [s]","steer / m/s"}});

  add("steering", QStringLiteral("4.7"), QStringLiteral("4.7 Tracking Dinamis"), "4.7.1",
      QStringLiteral("4.7.1 LEFT — Tracking Referensi Steering Sinusoidal"),
      {"Tracking steering sinusoidal","Error tracking steering","Iq steering","Duty steering"},
      {{"Variasi","Steering target","Steering actual","Steering error","LEFT Iq","LEFT Duty","Raw TIM4"}},
      {{"Steering target","esc_steer_target"},{"Steering actual","esc_steer_actual"},{"Steering error","derived.steering_error_rad"},{"LEFT Iq","vesc_left_values.iq_a"},{"LEFT Duty","vesc_left_values.duty"},{"Raw TIM4","vesc_steering_state.raw_encoder"}}, {},
      {{"time_series",{"Steering target","Steering actual"},{},{},"Time [s]","Steering [rad]"},{"time_series",{"Steering error"},{},{},"Time [s]","Error [rad]"},{"time_series",{"LEFT Iq"},{},{},"Time [s]","Iq [A]"},{"time_series",{"LEFT Duty"},{},{},"Time [s]","Duty"}});

  add("steering", QStringLiteral("4.7"), QStringLiteral("4.7 Tracking Dinamis"), "4.7.2",
      QStringLiteral("4.7.2 RIGHT — Tracking Referensi Velocity"),
      {"RIGHT command dan RPM","RIGHT current","RIGHT duty"},
      {{"Variasi","Command value","RIGHT RPM","RIGHT Motor Current","RIGHT Iq","RIGHT Duty","RIGHT Vbus"}},
      {{"Command value","vesc_command_state.value"},{"RIGHT RPM","vesc_right_values.rpm"},{"RIGHT Motor Current","vesc_right_values.current_motor_a"},{"RIGHT Iq","vesc_right_values.iq_a"},{"RIGHT Duty","vesc_right_values.duty"},{"RIGHT Vbus","vesc_right_values.vbus_v"}}, {},
      {{"time_series",{"Command value","RIGHT RPM"},{},{},"Time [s]","eRPM"},{"time_series",{"RIGHT Motor Current","RIGHT Iq"},{},{},"Time [s]","A"},{"time_series",{"RIGHT Duty"},{},{},"Time [s]","Duty"}});

  add("steering", QStringLiteral("4.8"), QStringLiteral("4.8 Penolakan Gangguan Beban"), "4.8.1",
      QStringLiteral("4.8.1 LEFT — Penolakan Gangguan Beban Steering"),
      {"Posisi steering saat gangguan","Error saat gangguan","Iq saat gangguan","Duty saat gangguan"},
      {{"Kondisi","Steering target","Steering actual","Steering error","LEFT Iq","LEFT Motor Current","LEFT Duty","Status"}},
      {{"Steering target","esc_steer_target"},{"Steering actual","esc_steer_actual"},{"Steering error","derived.steering_error_rad"},{"LEFT Iq","vesc_left_values.iq_a"},{"LEFT Motor Current","vesc_left_values.current_motor_a"},{"LEFT Duty","vesc_left_values.duty"}}, {},
      {{"time_series",{"Steering target","Steering actual"},{},{},"Time [s]","Steering [rad]"},{"time_series",{"Steering error"},{},{},"Time [s]","Error [rad]"},{"time_series",{"LEFT Iq","LEFT Motor Current"},{},{},"Time [s]","A"},{"time_series",{"LEFT Duty"},{},{},"Time [s]","Duty"}});

  add("steering", QStringLiteral("4.8"), QStringLiteral("4.8 Penolakan Gangguan Beban"), "4.8.2",
      QStringLiteral("4.8.2 RIGHT — Penolakan Gangguan Beban Drive"),
      {"RIGHT RPM saat gangguan","RIGHT current saat gangguan","RIGHT duty saat gangguan","Vbus saat gangguan"},
      {{"Kondisi","Command value","RIGHT RPM","RIGHT Motor Current","RIGHT Input Current","RIGHT Iq","RIGHT Duty","RIGHT Vbus","Status"}},
      {{"Command value","vesc_command_state.value"},{"RIGHT RPM","vesc_right_values.rpm"},{"RIGHT Motor Current","vesc_right_values.current_motor_a"},{"RIGHT Input Current","vesc_right_values.current_in_a"},{"RIGHT Iq","vesc_right_values.iq_a"},{"RIGHT Duty","vesc_right_values.duty"},{"RIGHT Vbus","vesc_right_values.vbus_v"}}, {},
      {{"time_series",{"Command value","RIGHT RPM"},{},{},"Time [s]","eRPM"},{"time_series",{"RIGHT Motor Current","RIGHT Input Current","RIGHT Iq"},{},{},"Time [s]","A"},{"time_series",{"RIGHT Duty"},{},{},"Time [s]","Duty"},{"time_series",{"RIGHT Vbus"},{},{},"Time [s]","Vbus [V]"}});

  add("steering", QStringLiteral("4.9"), QStringLiteral("4.9 Konfigurasi Akhir dan Gerbang Final"), "4.9",
      QStringLiteral("4.9 Rekapitulasi Konfigurasi Akhir LEFT/RIGHT dan Gerbang Final Ackermann"), {},
      {{"Kelompok","Parameter","Nilai final","Motor","Persistensi","Sumber keputusan","Status"}}, {}, {});

  add("steering", QStringLiteral("4.10"), QStringLiteral("4.10 Handoff Antar-Tahap"), "4.10",
      QStringLiteral("4.10 Ringkasan Handoff Sensor → FOC → Outer Loop → Ackermann"), {},
      {{"Tahap","Input yang sudah PASS","Parameter dikunci","Output / handoff","Dipakai pada"}}, {}, {});

  add("steering", QStringLiteral("4.11"), QStringLiteral("4.11 Checklist Data Laporan"), "4.11",
      QStringLiteral("4.11 Checklist Kelengkapan Data Aktual dan Penggantian Data Estimasi"), {},
      {{"Item","Subbab sumber","Data aktual wajib","File CSV / grafik","Status final"}}, {}, {});
  // Navigation BAB-IV tuning metadata. Every requested leaf gets a live
  // parameter panel backed by the SAME YAML files consumed by autonomous.launch.py.
  // Validation leaves expose the final values read-only; tuning leaves are editable.
  if (subsystem == QStringLiteral("navigation")) {
    auto addField = [&](const QString &id, const QString &key, const QString &label,
                        const QString &kind, const QString &fileKey,
                        const QString &path, const QString &placeholder = QString()) {
      for (ExperimentSpec &s : out) {
        if (s.id != id) continue;
        for (const ExperimentParameterField &existing : s.parameterFields)
          if (existing.key == key) return;
        s.parameterFields << ExperimentParameterField(key, label, kind, fileKey, path, placeholder);
        return;
      }
    };
    auto addReadOnly = [&](const QString &id, const QString &key, const QString &label,
                           const QString &fileKey, const QString &path) {
      addField(id, key, label, QStringLiteral("yaml_readonly"), fileKey, path);
    };
    auto addBase = [&](const QString &id) {
      addField(id, QStringLiteral("sample_rate"), QStringLiteral("Sample Rate CSV (Hz)"),
               QStringLiteral("float"), QString(), QString(), QStringLiteral("5.0"));
      addField(id, QStringLiteral("duration"), QStringLiteral("Durasi Run (s)"),
               QStringLiteral("float"), QString(), QString(), QStringLiteral("30.0"));
      addField(id, QStringLiteral("variation"), QStringLiteral("Variasi / Run"),
               QStringLiteral("string"), QString(), QString(), QStringLiteral("variasi-1"));
      addField(id, QStringLiteral("condition"), QStringLiteral("Kondisi"),
               QStringLiteral("string"), QString(), QString(), QStringLiteral("pengujian"));
    };
    for (const ExperimentSpec &s : out)
      if (s.id != QStringLiteral("4.10")) addBase(s.id);

    // 4.1 Sensor
    addField("4.1.2", "imu_publish_rate", "IMU publish_rate_hz", "int", "imu", "data_imu_node.ros__parameters.publish_rate_hz", "10");
    addField("4.1.2", "imu_yaw_offset", "IMU yaw_offset_rad", "float", "imu", "data_imu_node.ros__parameters.yaw_offset_rad", "0.0");
    addReadOnly("4.1.2", "imu_gyro_z_cov", "Gyro-Z covariance", "imu", "data_imu_node.ros__parameters.angular_velocity_covariance.2");
    addField("4.1.3", "esc_command_rate", "ESC command_rate_hz", "float", "esc", "esc_ackermann.ros__parameters.command_rate_hz", "50.0");
    addField("4.1.3", "esc_serial_tx_rate", "ESC serial_tx_rate_hz", "float", "esc", "esc_ackermann.ros__parameters.serial_tx_rate_hz", "50.0");
    addField("4.1.3", "nav_steer_min_speed", "Min speed steering Nav (m/s)", "float", "esc", "esc_ackermann.ros__parameters.min_speed_for_nav_steering_mps", "0.05");

    // 4.2 EKF lokal
    addField("4.2.2", "sensor_timeout", "EKF Lokal sensor_timeout (s)", "float", "ekf", "ekf_filter_node_odom.ros__parameters.sensor_timeout", "0.25");
    addField("4.2.3", "odom0_twist_rejection_threshold", "odom0_twist_rejection_threshold", "float", "ekf", "ekf_filter_node_odom.ros__parameters.odom0_twist_rejection_threshold", "5.0");
    addField("4.2.4", "q_vx", "Q(vx) process noise", "float", "ekf", "ekf_filter_node_odom.ros__parameters.process_noise_covariance.96", "0.025");
    addField("4.2.4", "q_vyaw", "Q(vyaw) process noise", "float", "ekf", "ekf_filter_node_odom.ros__parameters.process_noise_covariance.176", "0.02");
    addField("4.2.5", "predict_to_current_time", "predict_to_current_time", "bool", "ekf", "ekf_filter_node_odom.ros__parameters.predict_to_current_time", "true");
    addReadOnly("4.2.6", "frequency", "Final frequency (Hz)", "ekf", "ekf_filter_node_odom.ros__parameters.frequency");
    addReadOnly("4.2.6", "sensor_timeout", "Final sensor_timeout (s)", "ekf", "ekf_filter_node_odom.ros__parameters.sensor_timeout");
    addReadOnly("4.2.6", "twist_threshold", "Final twist rejection", "ekf", "ekf_filter_node_odom.ros__parameters.odom0_twist_rejection_threshold");
    addReadOnly("4.2.6", "q_vx", "Final Q(vx)", "ekf", "ekf_filter_node_odom.ros__parameters.process_noise_covariance.96");
    addReadOnly("4.2.6", "q_vyaw", "Final Q(vyaw)", "ekf", "ekf_filter_node_odom.ros__parameters.process_noise_covariance.176");
    addReadOnly("4.2.6", "predict", "Final predict_to_current_time", "ekf", "ekf_filter_node_odom.ros__parameters.predict_to_current_time");

    // 4.3 EKF global
    addField("4.3.2", "sensor_timeout", "EKF Global sensor_timeout (s)", "float", "ekf", "ekf_filter_node_map.ros__parameters.sensor_timeout", "2.0");
    addField("4.3.3", "odom0_pose_rejection_threshold", "odom0_pose_rejection_threshold", "float", "ekf", "ekf_filter_node_map.ros__parameters.odom0_pose_rejection_threshold", "12.0");
    addField("4.3.4", "q_x", "Q(x) process noise", "float", "ekf", "ekf_filter_node_map.ros__parameters.process_noise_covariance.0", "0.05");
    addField("4.3.4", "q_y", "Q(y) process noise", "float", "ekf", "ekf_filter_node_map.ros__parameters.process_noise_covariance.16", "0.05");
    addField("4.3.5", "predict_to_current_time", "predict_to_current_time", "bool", "ekf", "ekf_filter_node_map.ros__parameters.predict_to_current_time", "false");
    addReadOnly("4.3.6", "frequency", "Final frequency (Hz)", "ekf", "ekf_filter_node_map.ros__parameters.frequency");
    addReadOnly("4.3.6", "sensor_timeout", "Final sensor_timeout (s)", "ekf", "ekf_filter_node_map.ros__parameters.sensor_timeout");
    addReadOnly("4.3.6", "pose_threshold", "Final pose rejection", "ekf", "ekf_filter_node_map.ros__parameters.odom0_pose_rejection_threshold");
    addReadOnly("4.3.6", "q_x", "Final Q(x)", "ekf", "ekf_filter_node_map.ros__parameters.process_noise_covariance.0");
    addReadOnly("4.3.6", "q_y", "Final Q(y)", "ekf", "ekf_filter_node_map.ros__parameters.process_noise_covariance.16");
    addReadOnly("4.3.6", "predict", "Final predict_to_current_time", "ekf", "ekf_filter_node_map.ros__parameters.predict_to_current_time");

    // 4.4 LocalizationCore
    addField("4.4.1", "map_calibration_x", "Map calibration X (m)", "float", "localization", "localization_core.ros__parameters.map_calibration_x_m", "0.0");
    addField("4.4.1", "map_calibration_y", "Map calibration Y (m)", "float", "localization", "localization_core.ros__parameters.map_calibration_y_m", "0.0");
    addField("4.4.1", "map_calibration_yaw", "Map calibration yaw (rad)", "float", "localization", "localization_core.ros__parameters.map_calibration_yaw_rad", "0.0");
    addField("4.4.2", "startup_gnss_samples", "startup_gnss_samples", "int", "localization", "localization_core.ros__parameters.startup_gnss_samples", "5");
    addField("4.4.3", "strict_correction_alpha", "strict_correction_alpha", "float", "localization", "localization_core.ros__parameters.strict_correction_alpha", "0.08");
    addField("4.4.4", "strict_moving_correction_alpha", "strict_moving_correction_alpha", "float", "localization", "localization_core.ros__parameters.strict_moving_correction_alpha", "0.01");
    addField("4.4.5", "strict_max_correction_m", "strict_max_correction_m", "float", "localization", "localization_core.ros__parameters.strict_max_correction_m", "0.08");
    for (const auto &item : QVector<QPair<QString,QString>>{{"startup_gnss_samples","startup_gnss_samples"},{"strict_correction_alpha","strict_correction_alpha"},{"strict_moving_correction_alpha","strict_moving_correction_alpha"},{"strict_max_correction_m","strict_max_correction_m"}})
      addReadOnly("4.4.6", QStringLiteral("final_")+item.first, QStringLiteral("Final ")+item.first, "localization", QStringLiteral("localization_core.ros__parameters.")+item.second);

    // 4.5 Global costmap
    addField("4.5.1", "footprint_padding", "Global footprint_padding (m)", "float", "nav2", "global_costmap.global_costmap.ros__parameters.footprint_padding", "0.03");
    addField("4.5.2", "inflation_radius", "Global inflation_radius (m)", "float", "nav2", "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius", "0.8");
    addField("4.5.3", "cost_scaling_factor", "Global cost_scaling_factor", "float", "nav2", "global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor", "2.5");
    addReadOnly("4.5.4", "footprint_padding", "Final footprint_padding", "nav2", "global_costmap.global_costmap.ros__parameters.footprint_padding");
    addReadOnly("4.5.4", "inflation_radius", "Final inflation_radius", "nav2", "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius");
    addReadOnly("4.5.4", "cost_scaling_factor", "Final cost_scaling_factor", "nav2", "global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor");

    // 4.6 Smac Hybrid-A*
    addField("4.6.1", "minimum_turning_radius", "minimum_turning_radius (m)", "float", "nav2", "planner_server.ros__parameters.GridBased.minimum_turning_radius", "1.712159378317");
    addField("4.6.2", "downsampling_factor", "downsampling_factor", "int", "nav2", "planner_server.ros__parameters.GridBased.downsampling_factor", "2");
    addField("4.6.3", "angle_quantization_bins", "angle_quantization_bins", "int", "nav2", "planner_server.ros__parameters.GridBased.angle_quantization_bins", "48");
    addField("4.6.4", "cost_penalty", "cost_penalty", "float", "nav2", "planner_server.ros__parameters.GridBased.cost_penalty", "2.2");
    addField("4.6.5", "analytic_expansion_max_length", "analytic_expansion_max_length (m)", "float", "nav2", "planner_server.ros__parameters.GridBased.analytic_expansion_max_length", "8.0");
    addField("4.6.6", "non_straight_penalty", "non_straight_penalty", "float", "nav2", "planner_server.ros__parameters.GridBased.non_straight_penalty", "1.2");
    addReadOnly("4.6.7", "minimum_turning_radius", "Final minimum_turning_radius", "nav2", "planner_server.ros__parameters.GridBased.minimum_turning_radius");
    addReadOnly("4.6.7", "downsampling_factor", "Final downsampling_factor", "nav2", "planner_server.ros__parameters.GridBased.downsampling_factor");
    addReadOnly("4.6.7", "angle_bins", "Final angle_quantization_bins", "nav2", "planner_server.ros__parameters.GridBased.angle_quantization_bins");
    addReadOnly("4.6.7", "cost_penalty", "Final cost_penalty", "nav2", "planner_server.ros__parameters.GridBased.cost_penalty");
    addReadOnly("4.6.7", "analytic_length", "Final analytic_expansion_max_length", "nav2", "planner_server.ros__parameters.GridBased.analytic_expansion_max_length");
    addReadOnly("4.6.7", "non_straight", "Final non_straight_penalty", "nav2", "planner_server.ros__parameters.GridBased.non_straight_penalty");

    // 4.7 MPPI Ackermann
    addField("4.7.1", "controller_frequency", "controller_frequency (Hz)", "float", "nav2", "controller_server.ros__parameters.controller_frequency", "8.0");
    addField("4.7.1", "model_dt", "MPPI model_dt (s)", "float", "nav2", "controller_server.ros__parameters.FollowPath.model_dt", "0.125");
    addField("4.7.2", "time_steps", "MPPI time_steps", "int", "nav2", "controller_server.ros__parameters.FollowPath.time_steps", "32");
    addField("4.7.3", "path_align_weight", "PathAlignCritic cost_weight", "float", "nav2", "controller_server.ros__parameters.FollowPath.PathAlignCritic.cost_weight", "8.0");
    addField("4.7.4", "path_follow_weight", "PathFollowCritic cost_weight", "float", "nav2", "controller_server.ros__parameters.FollowPath.PathFollowCritic.cost_weight", "4.0");
    addField("4.7.5", "path_angle_weight", "PathAngleCritic cost_weight", "float", "nav2", "controller_server.ros__parameters.FollowPath.PathAngleCritic.cost_weight", "3.0");
    addField("4.7.6", "vx_max", "MPPI vx_max (m/s)", "float", "nav2", "controller_server.ros__parameters.FollowPath.vx_max", "0.5");
    addField("4.7.7", "minimum_turning_radius", "MPPI / Planner minimum_turning_r (m)", "float", "nav2", "controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r", "1.712159378317");
    addReadOnly("4.7.7", "planner_rmin", "Planner minimum_turning_radius", "nav2", "planner_server.ros__parameters.GridBased.minimum_turning_radius");
    addReadOnly("4.7.8", "controller_frequency", "Final controller_frequency", "nav2", "controller_server.ros__parameters.controller_frequency");
    addReadOnly("4.7.8", "model_dt", "Final model_dt", "nav2", "controller_server.ros__parameters.FollowPath.model_dt");
    addReadOnly("4.7.8", "time_steps", "Final time_steps", "nav2", "controller_server.ros__parameters.FollowPath.time_steps");
    addReadOnly("4.7.8", "vx_max", "Final vx_max", "nav2", "controller_server.ros__parameters.FollowPath.vx_max");
    addReadOnly("4.7.8", "rmin", "Final minimum_turning_r", "nav2", "controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r");

    // 4.8 command pipeline / velocity smoother
    for (const QString &id : {QStringLiteral("4.8.1"), QStringLiteral("4.8.2")}) {
      addField(id, "min_x_velocity_threshold", "Controller min_x_velocity_threshold", "float", "nav2", "controller_server.ros__parameters.min_x_velocity_threshold", "0.08");
      addField(id, "mppi_deadband_vx", "MPPI VelocityDeadband vx", "float", "nav2", "controller_server.ros__parameters.FollowPath.VelocityDeadbandCritic.deadband_velocities.0", "0.10");
      addField(id, "smoother_deadband_vx", "Velocity smoother deadband vx", "float", "nav2", "velocity_smoother.ros__parameters.deadband_velocity.0", "0.03");
      addField(id, "esc_nav_steering_min_speed", "ESC min speed steering Nav", "float", "esc", "esc_ackermann.ros__parameters.min_speed_for_nav_steering_mps", "0.05");
    }
    addField("4.8.3", "smoothing_frequency", "smoothing_frequency (Hz)", "float", "nav2", "velocity_smoother.ros__parameters.smoothing_frequency", "20.0");
    addField("4.8.4", "max_accel", "max_accel vx (m/s²)", "float", "nav2", "velocity_smoother.ros__parameters.max_accel.0", "0.18");
    addField("4.8.4", "max_decel", "max_decel vx (m/s²)", "float", "nav2", "velocity_smoother.ros__parameters.max_decel.0", "-0.28");
    addReadOnly("4.8.5", "smoothing_frequency", "Final smoothing_frequency", "nav2", "velocity_smoother.ros__parameters.smoothing_frequency");
    addReadOnly("4.8.5", "deadband", "Final smoother deadband vx", "nav2", "velocity_smoother.ros__parameters.deadband_velocity.0");
    addReadOnly("4.8.5", "max_accel", "Final max_accel vx", "nav2", "velocity_smoother.ros__parameters.max_accel.0");
    addReadOnly("4.8.5", "max_decel", "Final max_decel vx", "nav2", "velocity_smoother.ros__parameters.max_decel.0");

    // 4.9 goal checker / end-to-end validation
    addField("4.9.1", "xy_goal_tolerance", "xy_goal_tolerance (m)", "float", "nav2", "controller_server.ros__parameters.goal_checker.xy_goal_tolerance", "0.75");
    addField("4.9.2", "yaw_goal_tolerance", "yaw_goal_tolerance (rad)", "float", "nav2", "controller_server.ros__parameters.goal_checker.yaw_goal_tolerance", "0.523598776");
    addReadOnly("4.9.3", "xy_goal_tolerance", "Final xy_goal_tolerance", "nav2", "controller_server.ros__parameters.goal_checker.xy_goal_tolerance");
    addReadOnly("4.9.3", "yaw_goal_tolerance", "Final yaw_goal_tolerance", "nav2", "controller_server.ros__parameters.goal_checker.yaw_goal_tolerance");
    addReadOnly("4.9.4", "planner_rmin", "Planner minimum_turning_radius", "nav2", "planner_server.ros__parameters.GridBased.minimum_turning_radius");
    addReadOnly("4.9.4", "mppi_rmin", "MPPI minimum_turning_r", "nav2", "controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r");
    addReadOnly("4.9.4", "vx_max", "MPPI vx_max", "nav2", "controller_server.ros__parameters.FollowPath.vx_max");
    addReadOnly("4.9.4", "smoothing_frequency", "Velocity smoother frequency", "nav2", "velocity_smoother.ros__parameters.smoothing_frequency");
    addReadOnly("4.9.4", "xy_goal", "xy_goal_tolerance", "nav2", "controller_server.ros__parameters.goal_checker.xy_goal_tolerance");
    addReadOnly("4.9.4", "yaw_goal", "yaw_goal_tolerance", "nav2", "controller_server.ros__parameters.goal_checker.yaw_goal_tolerance");
  }
  // clang-format on
  return out;
}
