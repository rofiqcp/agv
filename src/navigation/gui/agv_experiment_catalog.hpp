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
  // Compact constructor for initializer-list init (trailing defaults)
  ExperimentParameterField(QString k = {}, QString lbl = {}, QString knd = {},
    QString yf = {}, QString yp = {}, QString ph = {}, QString lv = {},
    bool gt = false, bool lck = false)
  : key(k), label(lbl), kind(knd), yamlFileKey(yf), yamlPath(yp),
    placeholder(ph), lockedValue(lv), isGroundTruth(gt), locked(lck) {}
};

// Optional per-graph rendering spec (for multi-series or scatter graphs).
struct ExperimentGraphSpec {
  QString type;               // "time_series" | "scatter"
  QStringList series;         // for time_series: keys into liveSeries
  QString xSeries;            // for scatter: telemetry key for X
  QString ySeries;            // for scatter: telemetry key for Y
  ExperimentGraphSpec(QString t = {}, QStringList s = {}, QString x = {}, QString y = {})
  : type(t), series(s), xSeries(x), ySeries(y) {}
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
  /* ------------------------- NAVIGASI ------------------------- */
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
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Estimasi Posisi Metrik Obstacle"), "4.3.1",
  QStringLiteral("4.3.1 Pengujian Grid Ground Truth"),
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
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Estimasi Posisi Metrik Obstacle"), "4.3.2",
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
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Estimasi Posisi Metrik Obstacle"), "4.3.3",
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
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Estimasi Posisi Metrik Obstacle"), "4.3.4",
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
  add("perception", QStringLiteral("4.3"), QStringLiteral("4.3 Pengujian Estimasi Posisi Metrik Obstacle"), "4.3.5",
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
  /* ------------------------- STEERING / FOC ------------------------- */
  add("steering", QStringLiteral("4.1"), QStringLiteral("4.1 Alur Pengujian Bertahap dan Aturan Penguncian Parameter"), "4.1",
  QStringLiteral("4.1 Alur Pengujian Bertahap dan Aturan Penguncian Parameter"),
  {
  },
  {
    {
      "Tahap", "Jenis kegiatan", "Masukan yang dikunci", "Variasi/proses", "Keluaran untuk tahap berikutnya"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Tahap Kalibrasi Awal"), "4.2.1",
  QStringLiteral("4.2.1 Kalibrasi Offset Arus, Gain Arus, dan Tegangan Bus"),
  {
    "Rekaman offset arus saat PWM nonaktif"
  },
  {
    {
      "Kanal", "Mean ADC", "Std ADC", "Arus setelah koreksi", "Status"
    }
  },
  {
    {
      "Iq ref", "foc_telemetry.iq_ref_a"
    }, {
      "Iq", "foc_telemetry.iq_a"
    },
    {
      "Id", "foc_telemetry.id_a"
    }, {
      "Vbus", "foc_telemetry.vbus_v"
    }
  });
  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Tahap Kalibrasi Awal"), "4.2.2",
  QStringLiteral("4.2.2 Validasi Encoder A/B dan Kalibrasi Mekanik Steering"),
  {
    "Linearitas konversi encoder ke sudut steering"
  },
  {
    {
      "Sudut referensi", "Feedback encoder", "Error", "Arah pendekatan"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.2"), QStringLiteral("4.2 Tahap Kalibrasi Awal"), "4.2.3",
  QStringLiteral("4.2.3 Pemetaan Enam Sektor dan Kalibrasi Sudut Elektrik"),
  {
    "Pemetaan enam sektor terhadap encoder"
  },
  {
    {
      "Sektor", "Count relatif", "Δ dari sektor sebelumnya", "Status"
    }
  },
  {
    {
      "Iq ref", "foc_telemetry.iq_ref_a"
    }, {
      "Iq", "foc_telemetry.iq_a"
    },
    {
      "Id", "foc_telemetry.id_a"
    }, {
      "Vbus", "foc_telemetry.vbus_v"
    }
  });
  add("steering", QStringLiteral("4.3"), QStringLiteral("4.3 Tahap Tuning Loop Arus FOC"), "4.3",
  QStringLiteral("4.3 Tahap Tuning Loop Arus FOC"),
  {
    "Respons loop arus untuk tiga set gain"
  },
  {
    {
      "Set", "Kp_i", "Ki_i", "Rise time", "Overshoot", "Ripple Iq", "Status"
    }
  },
  {
    {
      "Iq ref", "foc_telemetry.iq_ref_a"
    }, {
      "Iq", "foc_telemetry.iq_a"
    },
    {
      "Id", "foc_telemetry.id_a"
    }, {
      "Vbus", "foc_telemetry.vbus_v"
    }
  });
  add("steering", QStringLiteral("4.4"), QStringLiteral("4.4 Pengujian 1 — Tuning Gain Kendali Posisi"), "4.4",
  QStringLiteral("4.4 Pengujian 1 — Tuning Gain Kendali Posisi"),
  {
    "Tuning gain loop posisi pada target 20°"
  },
  {
    {
      "Set", "Kp_pos", "Ki_pos", "Rise time", "Settling", "Overshoot", "e_ss", "Iq peak"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.5"), QStringLiteral("4.5 Pengujian 2 — Respons Step Posisi dan Akuisisi Data Terpadu"), "4.5",
  QStringLiteral("4.5 Pengujian 2 — Respons Step Posisi dan Akuisisi Data Terpadu"),
  {
    "Respons step 10°", "Respons step 20°", "Respons step 30°", "Ringkasan respons step"
  },
  {
    {
      "Target", "Rise time", "Settling time", "Overshoot", "e_ss", "Iq peak", "Status"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.6"), QStringLiteral("4.6 Analisis Simetri, Repeatability, dan Ackermann dari Data Step"), "4.6.1",
  QStringLiteral("4.6.1 Simetri Arah Kiri dan Kanan"),
  {
    "Simetri arah kiri dan kanan"
  },
  {
    {
      "|Target|", "e ss kiri", "e ss kanan", "Selisih rise time", "Kesimpulan"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.6"), QStringLiteral("4.6 Analisis Simetri, Repeatability, dan Ackermann dari Data Step"), "4.6.2",
  QStringLiteral("4.6.2 Repeatability"),
  {
    "Repeatability delapan pengulangan"
  },
  {
    {
      "Target", "Mean |error|", "Std", "Maksimum", "Jumlah run"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.6"), QStringLiteral("4.6 Analisis Simetri, Repeatability, dan Ackermann dari Data Step"), "4.6.3",
  QStringLiteral("4.6.3 Validasi Geometri Ackermann"),
  {
    "Validasi geometri Ackermann"
  },
  {
    {
      "δ_in", "δ_out ideal", "δ_out terukur", "Error Ackermann"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.7"), QStringLiteral("4.7 Pengujian 3 — Tracking Referensi Sinusoidal"), "4.7",
  QStringLiteral("4.7 Pengujian 3 — Tracking Referensi Sinusoidal"),
  {
    "Tracking sinus 0,1 Hz", "Tracking sinus 0,5 Hz", "Tracking sinus 1,0 Hz", "Pengaruh frekuensi terhadap tracking"
  },
  {
    {
      "Frekuensi", "RMSE", "Gain amplitudo", "Phase lag", "Iq_rms", "Iq_peak", "Status"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.8"), QStringLiteral("4.8 Pengujian 4 — Penolakan Gangguan Beban"), "4.8",
  QStringLiteral("4.8 Pengujian 4 — Penolakan Gangguan Beban"),
  {
    "Penolakan gangguan tanpa beban", "Penolakan gangguan beban sedang", "Penolakan gangguan beban maksimum", "Ringkasan penolakan gangguan"
  },
  {
    {
      "Kondisi", "Deviasi maksimum", "Recovery time", "e_ss sesudah gangguan", "Iq_peak", "Status"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.9"), QStringLiteral("4.9 Rekapitulasi Konfigurasi Akhir dan Gerbang Final"), "4.9",
  QStringLiteral("4.9 Rekapitulasi Konfigurasi Akhir dan Gerbang Final"),
  {
  },
  {
    {
      "Kelompok", "Parameter", "Kandidat aktual", "Sumber keputusan"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.10"), QStringLiteral("4.10 Ringkasan Handoff Antar-Tahap"), "4.10",
  QStringLiteral("4.10 Ringkasan Handoff Antar-Tahap"),
  {
  },
  {
    {
      "Tahap", "Tiga variasi/proses", "Keluaran yang dikunci", "Dipakai pada"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
  add("steering", QStringLiteral("4.11"), QStringLiteral("4.11 Checklist Penggantian Data Estimasi"), "4.11",
  QStringLiteral("4.11 Checklist Penggantian Data Estimasi"),
  {
  },
  {
    {
      "Item", "Yang harus diganti/dilengkapi", "Status sebelum final"
    }
  },
  {
    {
      "Target", "esc_steer_target"
    }, {
      "Actual", "esc_steer_actual"
    },
    {
      "Error", "derived.steering_error_rad"
    }, {
      "Iq", "foc_telemetry.iq_a"
    }
  });
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
