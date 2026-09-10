#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <signal.h>
#include <sys/types.h>
#include <utility>
#include <vector>

#include <QBuffer>
#include <QByteArray>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHostAddress>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QList>
#include <QMap>
#include <QPointer>
#include <QProcess>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QUrl>
#include <QUrlQuery>

#include <action_msgs/srv/cancel_goal.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "agv_experiment_catalog.hpp"

using namespace std::chrono_literals;

namespace {
constexpr double kPi = 3.14159265358979323846;

double nowMs() { return static_cast<double>(QDateTime::currentMSecsSinceEpoch()); }

QString agvRootPath() {
  const QString configured = qEnvironmentVariable("AGV_ROOT").trimmed();
  if (!configured.isEmpty()) return QDir::cleanPath(QFileInfo(configured).absoluteFilePath());
  return QDir::cleanPath(QDir::home().filePath(QStringLiteral("agv")));
}

QString agvPath(const QString &relativePath) {
  return QDir::cleanPath(QDir(agvRootPath()).filePath(relativePath));
}

QString agvPythonPath() {
  const QString configured = qEnvironmentVariable("AGV_PYTHON").trimmed();
  return configured.isEmpty() ? QStringLiteral("/usr/bin/python3") : configured;
}

double yawFromQuat(double x, double y, double z, double w) {
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}

QJsonValue scalarFromString(const QString &value) {
  const QString s = value.trimmed();
  const QString lower = s.toLower();
  if (lower == "true") return true;
  if (lower == "false") return false;
  bool okInt = false;
  const qlonglong integer = s.toLongLong(&okInt);
  if (okInt && !s.contains('.') && !s.contains('e', Qt::CaseInsensitive)) return static_cast<double>(integer);
  bool okDouble = false;
  const double number = s.toDouble(&okDouble);
  if (okDouble && std::isfinite(number)) return number;
  return s;
}

QJsonValue yamlToJson(const YAML::Node &node) {
  if (!node || node.IsNull()) return QJsonValue();
  if (node.IsScalar()) return scalarFromString(QString::fromStdString(node.Scalar()));
  if (node.IsSequence()) {
    QJsonArray array;
    for (const auto &item : node) array.append(yamlToJson(item));
    return array;
  }
  if (node.IsMap()) {
    QJsonObject object;
    for (auto it = node.begin(); it != node.end(); ++it) {
      object.insert(QString::fromStdString(it->first.as<std::string>()), yamlToJson(it->second));
    }
    return object;
  }
  return QJsonValue();
}

QJsonValue rosParameterValueToJson(const rcl_interfaces::msg::ParameterValue &value) {
  using PT = rcl_interfaces::msg::ParameterType;
  switch (value.type) {
    case PT::PARAMETER_BOOL: return value.bool_value;
    case PT::PARAMETER_INTEGER: return static_cast<double>(value.integer_value);
    case PT::PARAMETER_DOUBLE: return value.double_value;
    case PT::PARAMETER_STRING: return QString::fromStdString(value.string_value);
    case PT::PARAMETER_BYTE_ARRAY: { QJsonArray a; for (auto v : value.byte_array_value) a.append(static_cast<double>(v)); return a; }
    case PT::PARAMETER_BOOL_ARRAY: { QJsonArray a; for (auto v : value.bool_array_value) a.append(v); return a; }
    case PT::PARAMETER_INTEGER_ARRAY: { QJsonArray a; for (auto v : value.integer_array_value) a.append(static_cast<double>(v)); return a; }
    case PT::PARAMETER_DOUBLE_ARRAY: { QJsonArray a; for (auto v : value.double_array_value) a.append(v); return a; }
    case PT::PARAMETER_STRING_ARRAY: { QJsonArray a; for (const auto &v : value.string_array_value) a.append(QString::fromStdString(v)); return a; }
    default: return QJsonValue();
  }
}

bool jsonRuntimeEquivalent(const QJsonValue &a, const QJsonValue &b) {
  if (a.isDouble() && b.isDouble()) {
    const double x=a.toDouble(), y=b.toDouble();
    return std::abs(x-y) <= 1e-8 * std::max({1.0,std::abs(x),std::abs(y)});
  }
  if (a.isBool() && b.isBool()) return a.toBool()==b.toBool();
  if (a.isString() && b.isString()) return a.toString()==b.toString();
  if (a.isArray() && b.isArray()) {
    const QJsonArray aa=a.toArray(), bb=b.toArray();
    if (aa.size()!=bb.size()) return false;
    for (int i=0;i<aa.size();++i) if (!jsonRuntimeEquivalent(aa.at(i),bb.at(i))) return false;
    return true;
  }
  return a == b;
}

QString packageConfigDir(const QString &packageName, const char *envName) {
  const QByteArray envValue = qgetenv(envName);
  if (!envValue.trimmed().isEmpty()) {
    const QString path = QDir::cleanPath(QDir::home().absoluteFilePath(QString::fromUtf8(envValue)));
    if (QDir(path).exists()) return path;
    const QString expanded = QString::fromUtf8(envValue).replace("~", QDir::homePath());
    if (QDir(expanded).exists()) return QDir::cleanPath(expanded);
  }
  try {
    const QString share = QString::fromStdString(ament_index_cpp::get_package_share_directory(packageName.toStdString()));
    const QString marker = "/install/";
    const int idx = share.indexOf(marker);
    if (idx > 0) {
      const QString workspace = share.left(idx);
      const QString sourceConfig = workspace + "/src/" + packageName + "/config";
      if (QDir(sourceConfig).exists()) return sourceConfig;
    }
    const QString installed = share + "/config";
    if (QDir(installed).exists()) return installed;
  } catch (...) {
  }
  return QString();
}

QMap<QString, QString> configCandidates() {
  const QString nav = packageConfigDir("navigation", "AGV_CONFIG_DIR");
  const QString esc = packageConfigDir("esc", "AGV_ESC_CONFIG_DIR");
  const QString per = packageConfigDir("perception", "AGV_PERCEPTION_CONFIG_DIR");
  const QString hmi = packageConfigDir("stmf4", "AGV_STMF4_CONFIG_DIR");
  return {
      {"vehicle", nav + "/vehicle.yaml"},
      {"navigation_core", nav + "/navigation_core.yaml"},
      {"nav2", nav + "/nav2_ackermann.yaml"},
      {"ekf", nav + "/ekf.yaml"},
      {"localization", nav + "/localization_cpp.yaml"},
      {"gnss", nav + "/gnss.yaml"},
      {"imu", nav + "/imu.yaml"},
      {"mag_heading", nav + "/mag_heading.yaml"},
      {"imu_speed", nav + "/imu_speed.yaml"},
      {"stage3", nav + "/stage3_navigation.yaml"},
      {"trajectory_safety", nav + "/trajectory_safety.yaml"},
      {"collision", nav + "/collision_monitor_production.yaml"},
      {"mppi_closed_loop", nav + "/mppi_closed_loop.yaml"},
      {"gui", nav + "/gui_calibration.yaml"},
      {"esc", esc + "/ackermann.yaml"},
      {"teleop", esc + "/teleop.yaml"},
      {"foc_thesis", esc + "/foc_thesis.yaml"},
      {"vesc_tool", esc + "/vesc_tool.yaml"},
      {"hmi", hmi + "/hmi.yaml"},
      {"perception", per + "/astra_yolop_gpu.yaml"},
      {"bbox_calibration", per + "/bbox_obstacle_calibration.yaml"},
  };
}

QString baselinePathForConfig(const QString &filePath) {
  return filePath + QStringLiteral(".web.baseline");
}

bool ensureConfigBaseline(const QString &filePath, QString *message = nullptr) {
  if (!QFileInfo(filePath).isFile()) {
    if (message) *message = QStringLiteral("YAML source tidak ditemukan: ") + filePath;
    return false;
  }
  const QString baseline = baselinePathForConfig(filePath);
  if (QFileInfo(baseline).isFile()) {
    try { YAML::LoadFile(baseline.toStdString()); }
    catch (const std::exception &e) {
      if (message) *message = QStringLiteral("Baseline YAML invalid: ") + QString::fromUtf8(e.what());
      return false;
    }
    if (message) *message = QStringLiteral("Baseline YAML tersedia");
    return true;
  }
  if (!QFile::copy(filePath, baseline)) {
    if (message) *message = QStringLiteral("Gagal membuat baseline YAML: ") + baseline;
    return false;
  }
  try { YAML::LoadFile(baseline.toStdString()); }
  catch (const std::exception &e) {
    QFile::remove(baseline);
    if (message) *message = QStringLiteral("Baseline hasil copy invalid: ") + QString::fromUtf8(e.what());
    return false;
  }
  QFile::setPermissions(baseline, QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                                  QFileDevice::ReadGroup | QFileDevice::ReadOther);
  if (message) *message = QStringLiteral("Baseline YAML dibuat: ") + baseline;
  return true;
}

QJsonObject loadConfigSnapshot() {
  QJsonObject root;
  QJsonObject paths;
  QJsonObject files;
  paths["navigation"] = packageConfigDir("navigation", "AGV_CONFIG_DIR");
  paths["esc"] = packageConfigDir("esc", "AGV_ESC_CONFIG_DIR");
  paths["perception"] = packageConfigDir("perception", "AGV_PERCEPTION_CONFIG_DIR");
  paths["hmi"] = packageConfigDir("stmf4", "AGV_STMF4_CONFIG_DIR");

  const QMap<QString, QString> candidates = configCandidates();
  for (auto it = candidates.cbegin(); it != candidates.cend(); ++it) {
    if (it.value().startsWith('/') && QFileInfo::exists(it.value())) {
      try {
        QJsonObject entry;
        entry["path"] = it.value();
        entry["data"] = yamlToJson(YAML::LoadFile(it.value().toStdString()));
        QString baselineMessage;
        const bool baselineOk = ensureConfigBaseline(it.value(), &baselineMessage);
        entry["baseline_ok"] = baselineOk;
        entry["baseline_path"] = baselinePathForConfig(it.value());
        entry["baseline_message"] = baselineMessage;
        if (baselineOk) {
          const QString baselinePath = baselinePathForConfig(it.value());
          entry["baseline_data"] = yamlToJson(YAML::LoadFile(baselinePath.toStdString()));
          entry["baseline_mtime_ms"] = QFileInfo(baselinePath).lastModified().toMSecsSinceEpoch();
        }
        files[it.key()] = entry;
      } catch (const std::exception &e) {
        files[it.key()] = QJsonObject{{"path", it.value()}, {"error", QString::fromUtf8(e.what())}};
      }
    }
  }
  root["paths"] = paths;
  root["files"] = files;
  root["model_expected"] = agvPath(QStringLiteral("models/yolopv2.pt"));
  root["agv_root"] = agvRootPath();
  root["generated_at_ms"] = nowMs();
  return root;
}


QString uiParameterMetadataPath() {
  const QString env = qEnvironmentVariable("AGV_WEB_CONFIG_DIR").trimmed();
  if (!env.isEmpty()) {
    const QString p = QDir(env).filePath(QStringLiteral("ui_parameter_metadata.yaml"));
    if (QFileInfo(p).isFile()) return QDir::cleanPath(p);
  }
  try {
    const QString share = QString::fromStdString(ament_index_cpp::get_package_share_directory("navigation"));
    const QString installed = share + QStringLiteral("/web/config/ui_parameter_metadata.yaml");
    if (QFileInfo(installed).isFile()) return QDir::cleanPath(installed);
    const QString marker = QStringLiteral("/install/");
    const int idx = share.indexOf(marker);
    if (idx > 0) {
      const QString source = share.left(idx) + QStringLiteral("/src/navigation/web/config/ui_parameter_metadata.yaml");
      if (QFileInfo(source).isFile()) return QDir::cleanPath(source);
    }
  } catch (...) {}
  const QString fallback = agvPath(QStringLiteral("src/navigation/web/config/ui_parameter_metadata.yaml"));
  return QFileInfo(fallback).isFile() ? QDir::cleanPath(fallback) : QString();
}

QJsonObject loadUiParameterMetadata() {
  const QString path = uiParameterMetadataPath();
  if (path.isEmpty()) return QJsonObject{{"version", 2}, {"parameters", QJsonObject()}, {"error", "metadata_not_found"}};
  try {
    const QJsonValue value = yamlToJson(YAML::LoadFile(path.toStdString()));    if (!value.isObject()) return QJsonObject{{"version", 2}, {"parameters", QJsonObject()}, {"error", "metadata_root_not_object"}, {"path", path}};
    QJsonObject out = value.toObject();
    out["path"] = path;
    return out;
  } catch (const std::exception &e) {
    return QJsonObject{{"version", 2}, {"parameters", QJsonObject()}, {"error", QString::fromUtf8(e.what())}, {"path", path}};
  }
}

QString sha256File(const QString &path) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) return QString();
  QCryptographicHash hash(QCryptographicHash::Sha256);
  if (!hash.addData(&f)) return QString();
  return QString::fromLatin1(hash.result().toHex());
}

QJsonObject configRevisionState() {
  const auto candidates = configCandidates();
  QJsonObject files;
  QStringList lines;
  for (auto it = candidates.cbegin(); it != candidates.cend(); ++it) {
    if (!QFileInfo(it.value()).isFile()) continue;
    const QString revision = sha256File(it.value());
    files[it.key()] = revision;
    lines << it.key() + QStringLiteral("=") + revision;
  }
  std::sort(lines.begin(), lines.end());
  const QByteArray joined = lines.join(QStringLiteral("\n")).toUtf8();  const QString global = QString::fromLatin1(QCryptographicHash::hash(joined, QCryptographicHash::Sha256).toHex());
  return QJsonObject{{"files", files}, {"config_revision", global}, {"snapshot_at_ms", nowMs()}};
}

QString configDraftSignature(const QJsonArray &items) {
  QStringList rows;
  for (const QJsonValue &v : items) {
    if (!v.isObject()) continue;
    const QJsonObject o = v.toObject();
    const QString id = o.value("file_key").toString() + QStringLiteral(":") + o.value("path").toString();
    QJsonArray wrapped; wrapped.append(o.value("value"));
    const QString encoded = QString::fromUtf8(QJsonDocument(wrapped).toJson(QJsonDocument::Compact));
    rows << id + QStringLiteral("=") + encoded + QStringLiteral("|generated=") +
      (o.value("generated").toBool(false) ? QStringLiteral("1") : QStringLiteral("0")) +
      QStringLiteral("|proposal=") + o.value("proposal_id").toString();
  }
  std::sort(rows.begin(), rows.end());
  return QString::fromLatin1(QCryptographicHash::hash(rows.join(QStringLiteral("\n")).toUtf8(), QCryptographicHash::Sha256).toHex());
}

bool metadataNumericWithin(const QJsonObject &meta, const QJsonValue &value, QString *error, QString *warning) {
  if (!value.isDouble()) return true;
  const double n = value.toDouble();
  if (!std::isfinite(n)) { if (error) *error = QStringLiteral("Nilai numerik wajib finite"); return false; }
  if (meta.contains("hard_min") && n < meta.value("hard_min").toDouble()) { if (error) *error = QStringLiteral("Nilai di bawah hard_min"); return false; }
  if (meta.contains("hard_max") && n > meta.value("hard_max").toDouble()) { if (error) *error = QStringLiteral("Nilai di atas hard_max"); return false; }
  if (warning && ((meta.contains("recommended_min") && n < meta.value("recommended_min").toDouble()) ||
                  (meta.contains("recommended_max") && n > meta.value("recommended_max").toDouble())))
    *warning = QStringLiteral("Di luar recommended range; review evidence sebelum Apply");
  return true;
}

QString jsonScalarInlineYaml(const QJsonValue &value) {
  if (value.isNull() || value.isUndefined()) return QStringLiteral("~");
  if (value.isBool()) return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
  if (value.isDouble()) return QString::number(value.toDouble(), 'g', 16);
  if (value.isString()) {
    QJsonArray wrapper{value};
    QByteArray encoded = QJsonDocument(wrapper).toJson(QJsonDocument::Compact);
    return QString::fromUtf8(encoded.mid(1, encoded.size() - 2));
  }
  if (value.isArray()) return QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  return QString::fromUtf8(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
}

QString jsonScalarPreservingYamlType(const QJsonValue &value, QString originalScalar) {
  originalScalar = originalScalar.trimmed();
  // Preserve floating-point arrays as floating-point arrays. QJson serializes
  // 1.0 as 1, which otherwise changes a ROS 2 double_array into integer_array.
  if (value.isArray() && originalScalar.startsWith('[') && originalScalar.endsWith(']')) {
    static const QRegularExpression floatToken(QStringLiteral(R"([+-]?(?:\d+\.\d*|\d*\.\d+|\d+[eE][+-]?\d+))"));
    if (floatToken.match(originalScalar).hasMatch()) {
      QStringList items;
      for (const QJsonValue &entry : value.toArray()) {
        if (entry.isDouble()) {
          QString n = QString::number(entry.toDouble(), 'g', 16);
          if (!n.contains('.') && !n.contains('e', Qt::CaseInsensitive)) n += QStringLiteral(".0");
          items << n;
        } else {
          items << jsonScalarInlineYaml(entry);
        }
      }
      return QStringLiteral("[") + items.join(QStringLiteral(", ")) + QStringLiteral("]");
    }
  }
  if (!value.isDouble()) return jsonScalarInlineYaml(value);

  // QJson stores every JSON number as double. ROS 2 YAML, however, distinguishes
  // integer and floating-point parameter types. Keep a leaf that was written as
  // 10.0 / 1e-3 in floating-point syntax even when the edited value is exactly
  // integral; otherwise rclcpp rejects the YAML override as PARAMETER_INTEGER.
  static const QRegularExpression floatSyntax(
    QStringLiteral(R"(^[+-]?(?:\d+\.\d*|\d*\.\d+|\d+(?:[eE][+-]?\d+))(?:[eE][+-]?\d+)?$)"));
  const bool originallyFloat = floatSyntax.match(originalScalar).hasMatch();
  QString encoded = QString::number(value.toDouble(), 'g', 16);
  if (originallyFloat && !encoded.contains('.') && !encoded.contains('e', Qt::CaseInsensitive)) {
    encoded += QStringLiteral(".0");
  }
  return encoded;
}

bool patchExistingYamlScalar(const QString &filePath, const QString &yamlPath, const QJsonValue &value, QString *error) {
  QFile in(filePath);
  if (!in.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (error) *error = QStringLiteral("Tidak dapat membaca YAML");
    return false;
  }
  QStringList lines = QString::fromUtf8(in.readAll()).split('\n');
  in.close();
  const QStringList wanted = yamlPath.split('.', Qt::SkipEmptyParts);
  QVector<QPair<int, QString>> stack;
  int target = -1;
  for (int i = 0; i < lines.size(); ++i) {
    const QString raw = lines[i];
    const QString stripped = raw.trimmed();
    if (stripped.isEmpty() || stripped.startsWith('#') || stripped.startsWith('-') || !stripped.contains(':')) continue;
    const int indent = raw.size() - raw.trimmed().size();
    QString key = stripped.section(':', 0, 0).trimmed();
    key.remove('"'); key.remove('\'');
    while (!stack.isEmpty() && indent <= stack.last().first) stack.removeLast();
    QStringList full;
    for (const auto &entry : stack) full << entry.second;
    full << key;
    if (full == wanted) { target = i; break; }
    stack.push_back({indent, key});
  }
  if (target < 0) {
    if (error) *error = QStringLiteral("Path YAML tidak ditemukan untuk patch preserving-comment");
    return false;
  }
  const QString raw = lines[target];
  const int colon = raw.indexOf(':');
  if (colon < 0) return false;
  const QString prefix = raw.left(colon + 1);
  const QString rest = raw.mid(colon + 1);
  QString comment;
  bool quote = false;
  int hash = -1;
  for (int j = 0; j < rest.size(); ++j) {
    if (rest[j] == '"' && (j == 0 || rest[j - 1] != '\\')) quote = !quote;
    if (rest[j] == '#' && !quote) { hash = j; break; }
  }
  if (hash >= 0) comment = QStringLiteral(" ") + rest.mid(hash).trimmed();
  const QString originalScalar = (hash >= 0 ? rest.left(hash) : rest).trimmed();
  lines[target] = prefix + QStringLiteral(" ") + jsonScalarPreservingYamlType(value, originalScalar) + comment;
  const QByteArray candidate = lines.join('\n').toUtf8();
  try { YAML::Load(candidate.constData()); }
  catch (const std::exception &e) {
    if (error) *error = QStringLiteral("Patch menghasilkan YAML invalid: ") + QString::fromUtf8(e.what());
    return false;
  }
  QSaveFile out(filePath);
  if (!out.open(QIODevice::WriteOnly | QIODevice::Text)) {
    if (error) *error = QStringLiteral("Tidak dapat membuka YAML untuk atomic save");
    return false;
  }
  out.write(candidate);
  if (!out.commit()) {
    if (error) *error = QStringLiteral("Atomic YAML commit gagal");
    return false;
  }
  return true;
}

QJsonValue yamlPathValue(YAML::Node node, const QStringList &parts, int index = 0) {
  if (index >= parts.size() || !node) return yamlToJson(node);
  bool numeric = false;
  const int seqIndex = parts[index].toInt(&numeric);
  if (numeric && node.IsSequence()) {
    if (seqIndex < 0 || static_cast<size_t>(seqIndex) >= node.size()) return QJsonValue();
    return yamlPathValue(node[static_cast<size_t>(seqIndex)], parts, index + 1);
  }
  if (!node.IsMap()) return QJsonValue();
  return yamlPathValue(node[parts[index].toStdString()], parts, index + 1);
}

bool baselineYamlValue(const QString &fileKey, const QString &yamlPath, QJsonValue *value,
                       QString *message = nullptr) {
  const QMap<QString, QString> candidates = configCandidates();
  if (!candidates.contains(fileKey) || yamlPath.trimmed().isEmpty()) {
    if (message) *message = QStringLiteral("file_key/path baseline tidak diizinkan");
    return false;
  }
  const QString filePath = candidates.value(fileKey);
  QString baselineMessage;
  if (!ensureConfigBaseline(filePath, &baselineMessage)) {
    if (message) *message = baselineMessage;
    return false;
  }
  const QString baseline = baselinePathForConfig(filePath);
  try {
    YAML::Node current = YAML::LoadFile(baseline.toStdString());
    const QStringList parts = yamlPath.split('.', Qt::SkipEmptyParts);
    for (const QString &part : parts) {
      bool numeric = false;
      const int seqIndex = part.toInt(&numeric);
      if (numeric && current.IsSequence()) {
        if (seqIndex < 0 || static_cast<size_t>(seqIndex) >= current.size()) {
          if (message) *message = QStringLiteral("Index baseline di luar batas: ") + yamlPath;
          return false;
        }
        current = current[static_cast<size_t>(seqIndex)];
      } else {
        if (!current.IsMap() || !current[part.toStdString()]) {
          if (message) *message = QStringLiteral("Path tidak ada pada baseline: ") + yamlPath;
          return false;
        }
        current = current[part.toStdString()];
      }
    }
    if (!current) {
      if (message) *message = QStringLiteral("Nilai baseline tidak ditemukan: ") + yamlPath;
      return false;
    }
    if (value) *value = yamlToJson(current);
    if (message) *message = QStringLiteral("Nilai baseline ditemukan");
    return true;
  } catch (const std::exception &e) {
    if (message) *message = QString::fromUtf8(e.what());
    return false;
  }
}

bool currentYamlValue(const QString &fileKey, const QString &yamlPath, QJsonValue *value, QString *message = nullptr) {
  const QMap<QString, QString> candidates = configCandidates();
  if (!candidates.contains(fileKey) || yamlPath.trimmed().isEmpty()) { if (message) *message = "file_key/path YAML tidak diizinkan"; return false; }
  try {
    YAML::Node root = YAML::LoadFile(candidates.value(fileKey).toStdString());
    const QJsonValue actual = yamlPathValue(root, yamlPath.split('.', Qt::SkipEmptyParts));
    if (actual.isUndefined() || actual.isNull()) { if (message) *message = "Path YAML tidak ditemukan: " + yamlPath; return false; }
    if (value) *value = actual;
    if (message) *message = "Nilai YAML ditemukan";
    return true;
  } catch (const std::exception &e) { if (message) *message = QString::fromUtf8(e.what()); return false; }
}

bool compatibleConfigType(const QJsonValue &current, const QJsonValue &candidate) {
  if (current.isDouble()) return candidate.isDouble();
  if (current.isBool()) return candidate.isBool();
  if (current.isString()) return candidate.isString();
  if (current.isArray()) return candidate.isArray();
  if (current.isObject()) return candidate.isObject();
  return !candidate.isUndefined();
}

bool setYamlValueAtomic(const QString &fileKey, const QString &yamlPath, const QJsonValue &input,
                        QString *message, QJsonValue *savedValue = nullptr) {
  const QMap<QString, QString> candidates = configCandidates();
  if (!candidates.contains(fileKey) || yamlPath.trimmed().isEmpty()) {
    if (message) *message = QStringLiteral("file_key/path YAML tidak diizinkan");
    return false;
  }
  const QString filePath = candidates.value(fileKey);
  if (!QFileInfo(filePath).isFile()) {
    if (message) *message = QStringLiteral("File YAML tidak ditemukan: ") + filePath;
    return false;
  }
  const QStringList parts = yamlPath.split('.', Qt::SkipEmptyParts);
  if (parts.isEmpty()) return false;
  try {
    YAML::Node root = YAML::LoadFile(filePath.toStdString());
    YAML::Node current = root;
    for (int i = 0; i < parts.size() - 1; ++i) {
      bool numeric = false;
      const int seqIndex = parts[i].toInt(&numeric);
      if (numeric && current.IsSequence()) {
        if (seqIndex < 0 || static_cast<size_t>(seqIndex) >= current.size()) {
          if (message) *message = QStringLiteral("Index YAML di luar batas");
          return false;
        }
        current = current[static_cast<size_t>(seqIndex)];
      } else {
        if (!current.IsMap() || !current[parts[i].toStdString()]) {
          if (message) *message = QStringLiteral("Path YAML tidak ditemukan: ") + yamlPath;
          return false;
        }
        current = current[parts[i].toStdString()];
      }
    }
    bool lastNumeric = false;
    const int lastIndex = parts.last().toInt(&lastNumeric);
    QString patchPath = yamlPath;
    QJsonValue patchValue = input;
    if (lastNumeric && current.IsSequence()) {
      if (lastIndex < 0 || static_cast<size_t>(lastIndex) >= current.size()) {
        if (message) *message = QStringLiteral("Index YAML di luar batas");
        return false;
      }
      YAML::Node replacement = YAML::Load(jsonScalarInlineYaml(input).toStdString());
      current[static_cast<size_t>(lastIndex)] = replacement;
      patchPath = parts.mid(0, parts.size() - 1).join('.');
      patchValue = yamlToJson(current);
    } else {
      if (!current.IsMap() || !current[parts.last().toStdString()]) {
        if (message) *message = QStringLiteral("Leaf YAML tidak ditemukan: ") + yamlPath;
        return false;
      }
      YAML::Node replacement = YAML::Load(jsonScalarInlineYaml(input).toStdString());
      current[parts.last().toStdString()] = replacement;
    }
    const QString backup = filePath + QStringLiteral(".web.bak.") + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
    if (!QFile::copy(filePath, backup)) {
      if (message) *message = QStringLiteral("Gagal membuat backup YAML");
      return false;
    }
    QString patchError;
    if (!patchExistingYamlScalar(filePath, patchPath, patchValue, &patchError)) {
      QFile::remove(filePath);
      QFile::copy(backup, filePath);
      if (message) *message = patchError;
      return false;
    }
    YAML::Node verified = YAML::LoadFile(filePath.toStdString());
    const QJsonValue actual = yamlPathValue(verified, parts);
    if (savedValue) *savedValue = actual;
    if (message) *message = QStringLiteral("YAML tersimpan atomik; backup: ") + backup +
                            QStringLiteral(". Restart/lifecycle reload mungkin diperlukan.");
    return true;
  } catch (const std::exception &e) {
    if (message) *message = QString::fromUtf8(e.what());
    return false;
  }
}

QString csvEscape(QString value) {
  value.replace('"', QStringLiteral("\"\""));
  return QStringLiteral("\"") + value + QStringLiteral("\"");
}

void flattenJson(const QString &prefix, const QJsonValue &value, QMap<QString, QString> &out, int depth = 0) {
  if (depth > 4) {
    if (value.isObject()) out[prefix] = QString::fromUtf8(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
    else if (value.isArray()) out[prefix] = QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
    return;
  }
  if (value.isObject()) {
    const QJsonObject object = value.toObject();
    for (auto it = object.constBegin(); it != object.constEnd(); ++it) {
      const QString key = prefix.isEmpty() ? it.key() : prefix + QStringLiteral(".") + it.key();
      flattenJson(key, it.value(), out, depth + 1);
    }
    return;
  }
  if (value.isArray()) {
    out[prefix] = QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  } else if (value.isBool()) out[prefix] = value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
  else if (value.isDouble()) out[prefix] = QString::number(value.toDouble(), 'g', 16);
  else if (value.isString()) out[prefix] = value.toString();
  else out[prefix] = QString();
}

QString recordingCsvStem(QString section) {
  section = section.trimmed();
  if (section.isEmpty()) return QStringLiteral("run");
  const QRegularExpression sectionRx(QStringLiteral(R"(^(\d+(?:\.\d+)*)(?:\s+)?(.*)$)"));
  const QRegularExpressionMatch match = sectionRx.match(section);
  QString prefix;
  QString title = section;
  if (match.hasMatch()) {
    prefix = match.captured(1);
    title = match.captured(2).trimmed();
  }
  title.replace(QRegularExpression(QStringLiteral("\\s+")), QStringLiteral("_"));
  title.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9._-]+")), QStringLiteral("_"));
  while (title.contains(QStringLiteral("__"))) title.replace(QStringLiteral("__"), QStringLiteral("_"));
  while (title.startsWith('_')) title.remove(0, 1);
  while (title.endsWith('_')) title.chop(1);
  const QString stem = prefix.isEmpty() ? title :
    (title.isEmpty() ? prefix : prefix + QStringLiteral(".") + title);
  return stem.isEmpty() ? QStringLiteral("run") : stem.left(140);
}

QJsonObject parseJsonOrKv(const QString &raw) {
  QJsonParseError error{};
  const QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8(), &error);
  if (error.error == QJsonParseError::NoError) {
    if (doc.isObject()) return doc.object();
    if (doc.isArray()) return QJsonObject{{"items", doc.array()}, {"raw", raw}};
  }
  QJsonObject out;
  const QStringList parts = raw.split(QRegularExpression("[;,\\n]+"), Qt::SkipEmptyParts);
  for (const QString &part : parts) {
    int sep = part.indexOf('=');
    if (sep <= 0) sep = part.indexOf(':');
    if (sep <= 0) continue;
    const QString key = part.left(sep).trimmed();
    const QString value = part.mid(sep + 1).trimmed();
    if (!key.isEmpty()) out[key] = scalarFromString(value);
  }
  out["raw"] = raw;
  return out;
}

QJsonObject normalizePerceptionPerformance(QJsonObject map) {
  const auto alias = [&map](const QString &target, const QStringList &sources) {
    if (map.contains(target)) return;
    for (const QString &source : sources) if (map.contains(source)) { map[target] = map.value(source); return; }
  };
  alias("fps", {"pipeline_fps", "pipeline_fps_ema"});
  alias("mean_ms", {"pipeline_ms_per_frame", "pipeline_ms"});
  alias("p95_ms", {"pipeline_p95_ms", "pipeline_ms_per_frame", "pipeline_ms"});
  alias("capture_dropped", {"capture_dropped_total"});
  alias("rviz_dropped", {"rviz_dropped_total"});
  alias("raw_count", {"raw_detection_count", "detections"});
  alias("metric_count", {"metric_candidate_count", "object_points"});
  return map;
}

QJsonObject enrichObstacleMetrics(QJsonObject map) {
  const QJsonArray detections = map.value("detections").toArray();
  if (!map.contains("count")) map["count"] = detections.size();
  double nearest = std::numeric_limits<double>::infinity();
  double nearestLeft = std::numeric_limits<double>::quiet_NaN();
  double nearestHomography = std::numeric_limits<double>::quiet_NaN();
  double nearestScore = std::numeric_limits<double>::quiet_NaN();
  int nearestClass = -1, nearestTrack = -1;
  double confidenceSum = 0.0; int confidenceCount = 0;
  for (const auto &entry : detections) {
    const QJsonObject d = entry.toObject();
    const double forward = d.value("forward_m").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double score = d.value("score").toDouble(std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(score)) {confidenceSum += score; ++confidenceCount;}
    if (std::isfinite(forward) && forward < nearest) {
      nearest = forward; nearestLeft = d.value("left_m").toDouble(std::numeric_limits<double>::quiet_NaN());
      nearestHomography = d.value("forward_homography_m").toDouble(forward);
      nearestScore = score; nearestClass = d.value("class_id").toInt(-1); nearestTrack = d.value("track_id").toInt(-1);
    }
  }
  if (std::isfinite(nearest)) {
    map["nearest_forward_m"] = nearest; map["nearest_left_m"] = nearestLeft;
    map["nearest_forward_homography_m"] = nearestHomography; map["nearest_score"] = nearestScore;
    map["nearest_class_id"] = nearestClass; map["nearest_track_id"] = nearestTrack;
  }
  if (confidenceCount > 0) map["mean_confidence"] = confidenceSum / confidenceCount;
  return map;
}

QJsonObject parseRawDetectionSummary(const QString &raw) {
  QJsonObject map = parseJsonOrKv(raw);
  QJsonArray detections; double confidenceSum = 0.0; int confidenceCount = 0;
  const QRegularExpression cpu(QStringLiteral("d[0-9]+=cls:(-?[0-9]+),score:([-+]?[0-9]*\\.?[0-9]+)"));
  auto matches = cpu.globalMatch(raw);
  while (matches.hasNext()) {
    const auto m = matches.next(); const int cls = m.captured(1).toInt(); const double score = m.captured(2).toDouble();
    detections.append(QJsonObject{{"class_id", cls}, {"score", score}}); confidenceSum += score; ++confidenceCount;
  }
  if (!detections.isEmpty()) {map["detections"] = detections; map["count"] = detections.size();}
  if (confidenceCount > 0) map["mean_confidence"] = confidenceSum / confidenceCount;
  map["raw"] = raw;
  return map;
}

QByteArray mimeTypeForPath(const QString &path) {
  if (path.endsWith(".html")) return "text/html; charset=utf-8";
  if (path.endsWith(".css")) return "text/css; charset=utf-8";
  if (path.endsWith(".js")) return "application/javascript; charset=utf-8";
  if (path.endsWith(".json")) return "application/json; charset=utf-8";
  if (path.endsWith(".svg")) return "image/svg+xml";
  if (path.endsWith(".png")) return "image/png";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
  return "application/octet-stream";
}

QJsonObject experimentCatalogJson() {
  QJsonObject root;
  for (const QString & subsystem : {QStringLiteral("navigation"), QStringLiteral("perception"), QStringLiteral("steering")}) {
    QJsonArray entries;
    for (const ExperimentSpec &spec : buildExperimentCatalog(subsystem)) {
      QJsonObject item;
      item["subsystem"] = spec.subsystem;
      item["groupId"] = spec.groupId;
      item["groupTitle"] = spec.groupTitle;
      item["id"] = spec.id;
      item["section"] = spec.section;
      QJsonArray tableNames;
      for (const QString &name : spec.tableNames) tableNames.append(name);
      item["tableNames"] = tableNames;
      QJsonArray tableColumns;
      for (const QStringList &columns : spec.tableColumns) {
        QJsonArray row;
        for (const QString &column : columns) row.append(column);
        tableColumns.append(row);
      }
      item["tableColumns"] = tableColumns;
      QJsonArray graphCaptions;
      for (const QString &caption : spec.graphCaptions) graphCaptions.append(caption);
      item["graphCaptions"] = graphCaptions;
      QJsonObject liveSeries;
      for (auto it = spec.liveSeries.cbegin(); it != spec.liveSeries.cend(); ++it) liveSeries[it.key()] = it.value();
      item["liveSeries"] = liveSeries;
      QJsonArray parameters;
      for (const ExperimentParameterField &field : spec.parameterFields) {
        parameters.append(QJsonObject{
            {"key", field.key}, {"label", field.label}, {"kind", field.kind},
            {"yamlFileKey", field.yamlFileKey}, {"yamlPath", field.yamlPath},
            {"placeholder", field.placeholder}, {"lockedValue", field.lockedValue},
            {"isGroundTruth", field.isGroundTruth}, {"locked", field.locked}, {"group", field.group}});
      }
      item["parameterFields"] = parameters;
      QJsonArray graphs;
      for (const ExperimentGraphSpec &graph : spec.graphs) {
        QJsonArray series;
        for (const QString &s : graph.series) series.append(s);
        graphs.append(QJsonObject{{"type", graph.type}, {"series", series},
                                  {"xSeries", graph.xSeries}, {"ySeries", graph.ySeries},
                                  {"xLabel", graph.xLabel}, {"yLabel", graph.yLabel}});
      }
      item["graphs"] = graphs;
      entries.append(item);
    }
    root[subsystem] = entries;
  }
  return root;
}

}  // namespace

class WebRosBridge {
 public:
  WebRosBridge() {
    node_ = std::make_shared<rclcpp::Node>("agv_web_gui");
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, node_, false);
    bindAddress_ = QString::fromStdString(node_->declare_parameter<std::string>("bind_address", "127.0.0.1"));
    const auto configuredPort = node_->declare_parameter<std::int64_t>("port", 5000);
    // Deployment contract: the operator ROS Web must stay loopback-only on
    // localhost:5000. Reject overrides instead of silently exposing another
    // interface/port, so launch, footer, bookmarks and safety assumptions agree.
    if (bindAddress_ != "127.0.0.1" && bindAddress_ != "localhost") {
      throw std::invalid_argument("ROS Web wajib localhost (127.0.0.1)");
    }
    if (configuredPort != 5000) {
      throw std::invalid_argument("ROS Web wajib port 5000");
    }
    bindAddress_ = "127.0.0.1";
    port_ = 5000;
    readOnly_ = node_->declare_parameter<bool>("read_only", false);
    cameraJpegFps_ = std::clamp(node_->declare_parameter<double>("camera_jpeg_fps", 5.0), 0.5, 12.0);
    setupRos();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    executor_->add_node(node_);
    thread_ = std::thread([this]() {
      while (rclcpp::ok() && !stop_.load()) executor_->spin_once(100ms);
    });
    update("server", QJsonObject{{"ros", true}, {"read_only", readOnly_}, {"port", port_},
                                  {"bind_address", bindAddress_}, {"started_at_ms", nowMs()}});
  }

  ~WebRosBridge() {
    stop_.store(true);
    if (executor_) executor_->cancel();
    if (thread_.joinable()) thread_.join();
    if (executor_ && node_) executor_->remove_node(node_);
  }

  QString bindAddress() const { return bindAddress_; }
  int port() const { return port_; }
  bool readOnly() const { return readOnly_; }

  QJsonObject snapshot() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    QJsonObject copy = state_;
    copy["__updated"] = updated_;
    copy["__server_time_ms"] = nowMs();
    return copy;
  }

  QJsonObject takeDelta() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (dirty_.isEmpty()) return {};
    QJsonObject delta;
    QJsonObject timestamps;
    for (const QString &key : dirty_) {
      delta[key] = state_.value(key);
      timestamps[key] = updated_.value(key);
    }
    dirty_.clear();
    delta["__updated"] = timestamps;
    delta["__server_time_ms"] = nowMs();
    return delta;
  }

  QByteArray cameraJpeg() const {
    std::lock_guard<std::mutex> lock(mediaMutex_);
    return cameraJpeg_;
  }

  QByteArray mapPng() const {
    std::lock_guard<std::mutex> lock(mediaMutex_);
    return mapPng_;
  }

  QByteArray globalCostmapPng() const {
    std::lock_guard<std::mutex> lock(mediaMutex_);
    return globalCostmapPng_;
  }

  QByteArray localCostmapPng() const {
    std::lock_guard<std::mutex> lock(mediaMutex_);
    return localCostmapPng_;
  }

  QJsonObject configRuntimeState(const QString &fileKey, const QString &yamlPath) {
    QJsonObject out{{"file_key", fileKey}, {"path", yamlPath}};
    const auto targets = runtimeTargetsForChange(fileKey, yamlPath);
    QJsonArray targetNames;
    for (const auto &target : targets) targetNames.append(QJsonObject{{"restart_node", target.first}, {"parameter_node", target.second}});
    out["targets"] = targetNames;
    out["has_runtime_target"] = !targets.isEmpty();
    if (targets.isEmpty()) {
      out["readback_capable"] = false;
      out["status"] = "NO_RUNTIME_TARGET";
      return out;
    }
    const QString parameterName = parameterNameForYamlPath(yamlPath);
    const QJsonValue expected = expectedRuntimeValue(fileKey, yamlPath);
    QJsonArray reads;
    bool capable = !parameterName.isEmpty();
    for (const auto &target : targets) {
      const QJsonObject one = readRuntimeParameter(target.second, parameterName, expected);
      reads.append(one);
      capable = capable && one.contains("actual") &&
                (one.value("status").toString() == QStringLiteral("MATCH") || one.value("status").toString() == QStringLiteral("MISMATCH"));
    }
    out["parameter"] = parameterName;
    out["readback"] = reads;
    out["readback_capable"] = capable;
    out["status"] = capable ? QStringLiteral("READBACK_OK") : QStringLiteral("READBACK_UNAVAILABLE");
    return out;
  }

  bool vehicleStationary(QString *reason) const { return vehicleStationaryForRuntimeApply(reason); }

  QJsonObject applyConfigChange(const QString &fileKey, const QString &yamlPath) {
    QJsonObject result{{"requested", true}, {"file_key", fileKey}, {"path", yamlPath}};
    const auto targets = runtimeTargetsForChange(fileKey, yamlPath);
    if (targets.isEmpty()) {
      result["mode"] = "yaml_only";
      result["status"] = "NO_RUNTIME_TARGET";
      result["runtime_match"] = false;
      return result;
    }
    QString stationaryReason;
    if (!vehicleStationaryForRuntimeApply(&stationaryReason)) {
      result["mode"] = "restart_when_stationary";
      result["status"] = "PENDING_STATIONARY";
      result["message"] = stationaryReason;
      result["runtime_match"] = false;
      update("runtime_config_apply", result);
      return result;
    }

    QSet<QString> restartNodes;
    for (const auto &target : targets) restartNodes.insert(target.first);
    QJsonArray restartResults;
    int signaled = 0;
    for (const QString &nodeName : restartNodes) {
      const QStringList pids = exactNodeProcesses(nodeName);
      int stopped = 0;
      for (const QString &pidText : pids) {
        bool ok = false;
        const qlonglong pid = pidText.toLongLong(&ok);
        if (ok && pid > 1 && ::kill(pid_t(pid), SIGTERM) == 0) ++stopped;
      }
      signaled += stopped;
      restartResults.append(QJsonObject{{"node", nodeName}, {"matched_processes", pids.size()}, {"signaled", stopped}});
    }
    result["mode"] = "safe_restart";
    result["restart"] = restartResults;
    result["signaled_processes"] = signaled;
    if (signaled == 0) {
      result["status"] = "NEXT_START";
      result["message"] = "Node target tidak sedang berjalan; YAML akan aktif pada start berikutnya.";
      result["runtime_match"] = false;
      update("runtime_config_apply", result);
      return result;
    }

    // Tuning-critical nodes such as perception need time for model load/freeze/warm-up
    // after launch_ros respawn. Poll the parameter service instead of reporting a
    // false RUNTIME_MISMATCH while the constructor is still starting.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    const QString paramName = parameterNameForYamlPath(yamlPath);
    const QJsonValue expected = expectedRuntimeValue(fileKey, yamlPath);
    QJsonArray verify;
    bool allMatch = false;
    int verifyAttempts = 0;
    constexpr int kMaxVerifyAttempts = 8;
    for (int attempt = 1; attempt <= kMaxVerifyAttempts; ++attempt) {
      verifyAttempts = attempt;
      QJsonArray round;
      bool roundMatch = !paramName.isEmpty() && !expected.isUndefined();
      for (const auto &target : targets) {
        const QJsonObject one = readRuntimeParameter(target.second, paramName, expected);
        round.append(one);
        roundMatch = roundMatch && one.value("match").toBool(false);
      }
      verify = round;
      if (roundMatch) { allMatch = true; break; }
      if (attempt < kMaxVerifyAttempts) std::this_thread::sleep_for(std::chrono::milliseconds(900));
    }
    result["verify_attempts"] = verifyAttempts;
    result["verify"] = verify;
    result["runtime_match"] = allMatch;
    result["status"] = allMatch ? "ACTIVE_MATCH" : "RUNTIME_MISMATCH";
    result["message"] = allMatch
        ? QStringLiteral("YAML == runtime; parameter aktif setelah safe restart.")
        : QStringLiteral("Restart terkirim tetapi runtime belum MATCH; jangan mulai run tuning dulu.");
    update("runtime_config_apply", result);
    return result;
  }

  QJsonObject applyConfigChanges(const QJsonArray &changes) {
    QJsonObject result{{"requested", true}, {"change_count", changes.size()}};
    if (changes.isEmpty()) {
      result["mode"] = "batch";
      result["status"] = "NO_CHANGES";
      result["runtime_match"] = true;
      return result;
    }

    QSet<QString> restartNodes;
    bool hasRuntimeTarget = false;
    for (const QJsonValue &value : changes) {
      if (!value.isObject()) continue;
      const QJsonObject change = value.toObject();
      const auto targets = runtimeTargetsForChange(change.value("file_key").toString(), change.value("path").toString());
      if (!targets.isEmpty()) hasRuntimeTarget = true;
      for (const auto &target : targets) restartNodes.insert(target.first);
    }
    if (!hasRuntimeTarget) {
      result["mode"] = "yaml_only";
      result["status"] = "NO_RUNTIME_TARGET";
      result["runtime_match"] = false;
      return result;
    }

    QString stationaryReason;
    if (!vehicleStationaryForRuntimeApply(&stationaryReason)) {
      result["mode"] = "restart_when_stationary";
      result["status"] = "PENDING_STATIONARY";
      result["message"] = stationaryReason;
      result["runtime_match"] = false;
      update("runtime_config_apply", result);
      return result;
    }

    QJsonArray restartResults;
    int signaled = 0;
    for (const QString &nodeName : restartNodes) {
      const QStringList pids = exactNodeProcesses(nodeName);
      int stopped = 0;
      for (const QString &pidText : pids) {
        bool ok = false;
        const qlonglong pid = pidText.toLongLong(&ok);
        if (ok && pid > 1 && ::kill(pid_t(pid), SIGTERM) == 0) ++stopped;
      }
      signaled += stopped;
      restartResults.append(QJsonObject{{"node", nodeName}, {"matched_processes", pids.size()}, {"signaled", stopped}});
    }
    result["mode"] = "safe_batch_restart";
    result["restart"] = restartResults;
    result["signaled_processes"] = signaled;
    if (signaled == 0) {
      result["status"] = "NEXT_START";
      result["message"] = "Node target tidak sedang berjalan; seluruh baseline YAML akan aktif pada start berikutnya.";
      result["runtime_match"] = false;
      update("runtime_config_apply", result);
      return result;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    QJsonArray verify;
    bool allMatch = false;
    int verifyAttempts = 0;
    constexpr int kMaxVerifyAttempts = 8;
    for (int attempt = 1; attempt <= kMaxVerifyAttempts; ++attempt) {
      verifyAttempts = attempt;
      QJsonArray round;
      bool roundMatch = true;
      bool checkedAny = false;
      for (const QJsonValue &value : changes) {
        if (!value.isObject()) continue;
        const QJsonObject change = value.toObject();
        const QString fileKey = change.value("file_key").toString();
        const QString yamlPath = change.value("path").toString();
        const auto targets = runtimeTargetsForChange(fileKey, yamlPath);
        if (targets.isEmpty()) continue;
        checkedAny = true;
        const QString paramName = parameterNameForYamlPath(yamlPath);
        const QJsonValue expected = expectedRuntimeValue(fileKey, yamlPath);
        bool changeMatch = !paramName.isEmpty() && !expected.isUndefined();
        QJsonArray targetVerify;
        for (const auto &target : targets) {
          const QJsonObject one = readRuntimeParameter(target.second, paramName, expected);
          targetVerify.append(one);
          changeMatch = changeMatch && one.value("match").toBool(false);
        }
        round.append(QJsonObject{{"file_key", fileKey}, {"path", yamlPath},
                                 {"match", changeMatch}, {"verify", targetVerify}});
        roundMatch = roundMatch && changeMatch;
      }
      verify = round;
      if (checkedAny && roundMatch) { allMatch = true; break; }
      if (attempt < kMaxVerifyAttempts) std::this_thread::sleep_for(std::chrono::milliseconds(900));
    }
    result["verify_attempts"] = verifyAttempts;
    result["verify"] = verify;
    result["runtime_match"] = allMatch;
    result["status"] = allMatch ? "ACTIVE_MATCH" : "RUNTIME_MISMATCH";
    result["message"] = allMatch
        ? QStringLiteral("Semua parameter baseline yang memiliki runtime target sudah MATCH setelah satu batch restart.")
        : QStringLiteral("Batch restart selesai tetapi sebagian parameter runtime belum MATCH; jangan mulai run tuning dulu.");
    update("runtime_config_apply", result);
    return result;
  }

  bool publishGoal(double x, double y, double yawRad, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yawRad)) {
      if (message) *message = "Koordinat goal tidak valid";
      return false;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = "map";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.z = std::sin(yawRad / 2.0);
    goal.pose.orientation.w = std::cos(yawRad / 2.0);
    goalPub_->publish(goal);
    update("goal_pose", QJsonObject{{"x", x}, {"y", y}, {"yaw", yawRad}, {"source", "web"}});
    update("web_action", QJsonObject{{"ok", true}, {"action", "publish_goal"}, {"at_ms", nowMs()}});
    if (message) *message = "Goal diterbitkan ke /navigation/goal_request";
    return true;
  }

  bool publishInitialPose(double x, double y, double yawRad, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yawRad)) {
      if (message) *message = "Initial pose tidak valid";
      return false;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation.z = std::sin(yawRad / 2.0);
    pose.pose.pose.orientation.w = std::cos(yawRad / 2.0);
    pose.pose.covariance[0] = 0.01;
    pose.pose.covariance[7] = 0.01;
    pose.pose.covariance[35] = std::pow(kPi / 180.0, 2);
    initialPosePub_->publish(pose);
    update("web_action", QJsonObject{{"ok", true}, {"action", "publish_initial_pose"}, {"at_ms", nowMs()}});
    if (message) *message = "Initial pose diterbitkan ke /initialpose";
    return true;
  }

  bool cancelNavigation(QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    auto client = node_->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
    if (!client->wait_for_service(350ms)) {
      if (message) *message = "Service cancel Nav2 belum tersedia";
      return false;
    }
    auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    client->async_send_request(request, [this, client](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future) {
      try {
        const auto result = future.get();
        const bool ok = result->return_code == 0 || !result->goals_canceling.empty();
        update("web_action", QJsonObject{{"ok", ok}, {"action", "cancel_navigation"},
                                         {"return_code", result->return_code},
                                         {"goals", static_cast<double>(result->goals_canceling.size())},
                                         {"at_ms", nowMs()}});
      } catch (const std::exception &e) {
        update("web_action", QJsonObject{{"ok", false}, {"action", "cancel_navigation"},
                                         {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
      }
    });
    if (message) *message = "Permintaan cancel dikirim ke Nav2";
    return true;
  }

  bool triggerService(const QString &service, const QString &tag, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    auto client = node_->create_client<std_srvs::srv::Trigger>(service.toStdString());
    if (!client->wait_for_service(350ms)) {
      if (message) *message = "Service belum tersedia: " + service;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request, [this, client, service, tag](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        const auto result = future.get();
        update("web_action", QJsonObject{{"ok", result->success}, {"action", tag}, {"service", service},
                                         {"message", QString::fromStdString(result->message)}, {"at_ms", nowMs()}});
      } catch (const std::exception &e) {
        update("web_action", QJsonObject{{"ok", false}, {"action", tag}, {"service", service},
                                         {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
      }
    });
    if (message) *message = "Trigger dikirim ke " + service;
    return true;
  }

  bool publishHmiRequest(const QString &request, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    const QString cmd = request.trimmed();
    if (cmd.isEmpty() || cmd.size() > 96) {
      if (message) *message = "Perintah HMI tidak valid";
      return false;
    }
    std_msgs::msg::String msg;
    msg.data = cmd.toStdString();
    hmiRequestPub_->publish(msg);
    update("web_action", QJsonObject{{"ok", true}, {"action", "hmi_request"},
                                     {"command", cmd}, {"at_ms", nowMs()}});
    if (message) *message = "Perintah HMI dikirim: " + cmd;
    return true;
  }

  bool publishVescToolCommand(const QString &command, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    const QString cmd = command.trimmed();
    if (cmd.isEmpty() || cmd.size() > 512 || cmd.contains('\n') || cmd.contains('\r')) {
      if (message) *message = "Perintah VESC Tool tidak valid";
      return false;
    }
    std_msgs::msg::String msg;
    msg.data = cmd.toStdString();
    vescToolCommandPub_->publish(msg);
    update("web_action", QJsonObject{{"ok", true}, {"action", "vesc_tool_command"},
                                     {"command", cmd.left(96)}, {"at_ms", nowMs()}});
    if (message) *message = "Perintah dikirim ke VESC maintenance bridge";
    return true;
  }

  bool setPerceptionInference(bool enabled, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    auto client = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>("/perception/set_parameters_atomically");
    if (!client->wait_for_service(800ms)) {
      if (message) *message = "Node /perception belum tersedia untuk lazy inference";
      return false;
    }
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = "inference_enabled";
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter.value.bool_value = enabled;
    request->parameters.push_back(parameter);
    client->async_send_request(request,
      [this, client, enabled](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
        try {
          const auto response = future.get();
          const bool ok = response->result.successful;
          if (ok) update("perception_inference_enabled", enabled);
          update("web_action", QJsonObject{{"ok", ok}, {"action", enabled ? "perception_inference_on" : "perception_inference_off"},
                                           {"message", QString::fromStdString(response->result.reason)}, {"at_ms", nowMs()}});
        } catch (const std::exception &e) {
          update("web_action", QJsonObject{{"ok", false}, {"action", "perception_inference"},
                                           {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
        }
      });
    if (message) *message = enabled ? "YOLOPv2 inference diminta ON" : "YOLOPv2 inference diminta OFF (camera-only)";
    return true;
  }

  bool setDriveOdometryScale(double scale, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!std::isfinite(scale) || scale < 0.20 || scale > 5.0) {
      if (message) *message = "Drive scale harus finite dan 0.20..5.00";
      return false;
    }
    const double speed = scalarState("esc_drive_actual").value_or(0.0);
    if (std::abs(speed) > 0.03) {
      if (message) *message = QString("Apply scale ditolak: kendaraan masih bergerak (%1 m/s)").arg(speed,0,'f',3);
      return false;
    }
    auto client=node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
      "/esc_ackermann/set_parameters_atomically");
    if (!client->wait_for_service(700ms)) {
      if (message) *message = "Parameter service /esc_ackermann belum tersedia";
      return false;
    }
    auto req=std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    rcl_interfaces::msg::Parameter p; p.name="drive_odometry_calibration_scale";
    p.value.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE; p.value.double_value=scale;
    req->parameters.push_back(p);
    auto future=client->async_send_request(req);
    if (future.wait_for(1200ms)!=std::future_status::ready) {
      if (message) *message = "Timeout apply drive scale ke runtime";
      return false;
    }
    try {
      const auto result=future.get();
      if (!result->result.successful) { if(message)*message=QString::fromStdString(result->result.reason); return false; }
    } catch (const std::exception &e) { if(message)*message=QString::fromUtf8(e.what()); return false; }
    if (message) *message=QString("Drive scale %.8f aktif live").arg(scale,0,'f',8);
    return true;
  }

  bool publishTrialMotion(const QString &experimentId, double erpm, double steeringDeg, bool active, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!trialTwistPub_ || !trialSourcePub_) { if (message) *message = "Trial motion publisher belum siap"; return false; }
    if (!std::isfinite(erpm) || !std::isfinite(steeringDeg)) { if (message) *message = "Trial motion value tidak finite"; return false; }
    if (boolState("vesc_maintenance_active")) { if (message) *message = "Keluar dari VESC maintenance sebelum trial ROS"; return false; }
    if (active && !boolState("connected.esc_feedback")) { if (message) *message = "Trial ditolak: ESC feedback belum fresh"; return false; }
    const bool navCal = experimentId == QStringLiteral("N2.1") || experimentId == QStringLiteral("N3.1") || experimentId == QStringLiteral("N3.2");
    if (active && navCal && !objectBoolState("gnss_quality", "gnss_fix_ok")) { if (message) *message = "Trial ditolak: GNSS fix belum qualified"; return false; }
    const bool steerCal = experimentId == QStringLiteral("N3.1") || experimentId == QStringLiteral("N3.2");
    if (active && steerCal && (!boolState("connected.imu") || !boolState("connected.neo3_mag"))) { if (message) *message = "Trial steering ditolak: IMU/IST8310 belum online"; return false; }
    if (active && steerCal && !sourceConfigBool("vehicle", "vehicle.ros__parameters.drive_odometry_calibration_valid", false)) { if (message) *message = "Trial steering ditolak: drive odometry scale N2.1 belum certified"; return false; }
    const double wheelR = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.drive_wheel_radius_m", 0.145);
    const double polePairs = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.drive_motor_pole_pairs", 15.0);
    const double gear = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.drive_gear_ratio", 1.0);
    const double scale = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.drive_odometry_calibration_scale", 1.0);
    const double speedMax = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.speed_max", 0.5);
    const double yawMaxDeg = sourceConfigNumber("teleop", "motor_teleop.ros__parameters.yaw_max_deg_s", 80.0);
    const double steerLimit = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.steering_physical_operational_limit_deg", 28.0);
    const double erpmPerMps = (60.0 * gear * polePairs) / (2.0 * kPi * std::max(0.01, wheelR));
    const double speed = erpm / erpmPerMps * scale;
    if (std::abs(speed) > speedMax + 1.0e-6) {
      if (message) *message = QString("Trial ditolak: %1 eRPM = %2 m/s > speed_max %3 m/s").arg(erpm,0,'f',1).arg(speed,0,'f',3).arg(speedMax,0,'f',3);
      return false;
    }
    if (std::abs(steeringDeg) > steerLimit + 1.0e-6) {
      if (message) *message = QString("Trial ditolak: steering %1 deg > operational limit %2 deg").arg(steeringDeg,0,'f',2).arg(steerLimit,0,'f',2);
      return false;
    }
    geometry_msgs::msg::Twist cmd;
    if (active) {
      cmd.linear.x = speed;
      const double frac = steerLimit > 1.0e-9 ? steeringDeg / steerLimit : 0.0;
      cmd.angular.z = std::clamp(frac, -1.0, 1.0) * yawMaxDeg * kPi / 180.0;
    }
    std_msgs::msg::String src; src.data = active ? "WEB_TRIAL" : "STOP";
    trialTwistPub_->publish(cmd); trialSourcePub_->publish(src);
    update("web_trial_motion", QJsonObject{{"active",active},{"erpm",erpm},{"speed_mps",active?speed:0.0},{"steering_deg",active?steeringDeg:0.0},{"at_ms",nowMs()}});
    if (message) *message = active ? QString("WEB_TRIAL active: %1 eRPM, %2 deg").arg(erpm,0,'f',1).arg(steeringDeg,0,'f',2) : "WEB_TRIAL stop";
    return true;
  }

  bool setSteeringCalibrationMode(bool enabled, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (enabled) {
      const double speed = scalarState("esc_drive_actual").value_or(0.0);
      if (std::abs(speed) > 0.05) {
        if (message) *message = QString("Ditolak: kendaraan masih bergerak (%1 m/s)").arg(speed, 0, 'f', 3);
        return false;
      }
    }
    auto client = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
        "/esc_ackermann/set_parameters_atomically");
    if (!client->wait_for_service(500ms)) {
      if (message) *message = "Parameter service /esc_ackermann belum tersedia";
      return false;
    }
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = "steering_calibration_mode_enabled";
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter.value.bool_value = enabled;
    request->parameters.push_back(parameter);
    client->async_send_request(
        request, [this, client, enabled](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
          try {
            const auto result = future.get();
            update("web_action", QJsonObject{{"ok", result->result.successful},
                                             {"action", enabled ? "steering_cal_mode_on" : "steering_cal_mode_off"},
                                             {"message", QString::fromStdString(result->result.reason)}, {"at_ms", nowMs()}});
          } catch (const std::exception &e) {
            update("web_action", QJsonObject{{"ok", false}, {"action", "steering_calibration_mode"},
                                             {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
          }
        });
    if (message) *message = enabled ? "Mode kalibrasi steering diminta ON" : "Mode kalibrasi steering diminta OFF";
    return true;
  }

  // Host metrics dipublikasikan ke state web sebagai data aktual. Field yang
  // tidak tersedia (mis. GPU pada host CPU-only) tidak dibuat, sehingga UI N/A.
  void updateHostMetrics(const QJsonObject &metrics) { update("host", metrics); }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread thread_;
  std::atomic_bool stop_{false};
  QString bindAddress_;
  int port_{5000};
  bool readOnly_{false};
  double cameraJpegFps_{5.0};
  mutable std::mutex stateMutex_;
  QJsonObject state_;
  QJsonObject updated_;
  QSet<QString> dirty_;
  mutable std::mutex mediaMutex_;
  std::mutex cameraEncodeMutex_;
  std::mutex costmapEncodeMutex_;
  QByteArray cameraJpeg_;
  QByteArray mapPng_;
  QByteArray globalCostmapPng_;
  QByteArray localCostmapPng_;
  std::chrono::steady_clock::time_point lastCameraEncode_{};
  std::chrono::steady_clock::time_point lastRawCameraSeen_{};
  std::chrono::steady_clock::time_point lastCompressedCameraSeen_{};
  std::chrono::steady_clock::time_point lastGlobalCostmapEncode_{};
  std::chrono::steady_clock::time_point lastLocalCostmapEncode_{};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::TimerBase::SharedPtr tfTimer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vescToolCommandPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPosePub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hmiRequestPub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr trialTwistPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trialSourcePub_;

  QVector<QPair<QString,QString>> runtimeTargetsForChange(const QString &fileKey, const QString &path) const {
    QVector<QPair<QString,QString>> out;
    auto add=[&](const QString &restartNode,const QString &parameterNode){out.append(qMakePair(restartNode,parameterNode));};
    if (fileKey == QStringLiteral("ekf")) {
      if (path.startsWith(QStringLiteral("ekf_filter_node_odom."))) add(QStringLiteral("ekf_filter_node_odom"),QStringLiteral("/ekf_filter_node_odom"));
      else if (path.startsWith(QStringLiteral("ekf_filter_node_map."))) add(QStringLiteral("ekf_filter_node_map"),QStringLiteral("/ekf_filter_node_map"));
    } else if (fileKey == QStringLiteral("localization")) add(QStringLiteral("localization_core"),QStringLiteral("/localization_core"));
    else if (fileKey == QStringLiteral("gnss")) add(QStringLiteral("data_cuav_node"),QStringLiteral("/data_cuav_node"));
    else if (fileKey == QStringLiteral("imu")) add(QStringLiteral("data_imu_node"),QStringLiteral("/data_imu_node"));
    else if (fileKey == QStringLiteral("imu_speed")) add(QStringLiteral("imu_speed_diagnostic"),QStringLiteral("/imu_speed_diagnostic"));
    else if (fileKey == QStringLiteral("mag_heading")) add(QStringLiteral("mag_heading_fusion"),QStringLiteral("/mag_heading_fusion"));
    else if (fileKey == QStringLiteral("navigation_core")) add(QStringLiteral("navigation_core"),QStringLiteral("/navigation_core"));
    else if (fileKey == QStringLiteral("trajectory_safety")) add(QStringLiteral("trajectory_safety_supervisor"),QStringLiteral("/trajectory_safety_supervisor"));
    else if (fileKey == QStringLiteral("perception")) add(QStringLiteral("perception"),QStringLiteral("/perception"));
    else if (fileKey == QStringLiteral("collision")) add(QStringLiteral("collision_monitor"),QStringLiteral("/collision_monitor"));
    else if (fileKey == QStringLiteral("esc")) add(QStringLiteral("esc_ackermann"),QStringLiteral("/esc_ackermann"));
    else if (fileKey == QStringLiteral("hmi")) add(QStringLiteral("stmf4_hmi_bridge"),QStringLiteral("/stmf4_hmi_bridge"));
    else if (fileKey == QStringLiteral("mppi_closed_loop")) add(QStringLiteral("mppi_closed_loop_supervisor"),QStringLiteral("/mppi_closed_loop_supervisor"));
    else if (fileKey == QStringLiteral("teleop")) add(QStringLiteral("motor_teleop"),QStringLiteral("/motor_teleop"));
    else if (fileKey == QStringLiteral("nav2")) {
      if (path.startsWith(QStringLiteral("planner_server."))) add(QStringLiteral("planner_server"),QStringLiteral("/planner_server"));
      else if (path.startsWith(QStringLiteral("global_costmap."))) add(QStringLiteral("planner_server"),QStringLiteral("/global_costmap/global_costmap"));
      else if (path.startsWith(QStringLiteral("controller_server."))) add(QStringLiteral("controller_server"),QStringLiteral("/controller_server"));
      else if (path.startsWith(QStringLiteral("local_costmap."))) add(QStringLiteral("controller_server"),QStringLiteral("/local_costmap/local_costmap"));
      else if (path.startsWith(QStringLiteral("velocity_smoother."))) add(QStringLiteral("velocity_smoother"),QStringLiteral("/velocity_smoother"));
      else if (path.startsWith(QStringLiteral("behavior_server."))) add(QStringLiteral("behavior_server"),QStringLiteral("/behavior_server"));
      else if (path.startsWith(QStringLiteral("bt_navigator."))) add(QStringLiteral("bt_navigator"),QStringLiteral("/bt_navigator"));
      else if (path.startsWith(QStringLiteral("map_server."))) add(QStringLiteral("map_server"),QStringLiteral("/map_server"));
    }
    // vehicle.yaml is physical authority. The staged GUI exposes those fields
    // locked; runtime fan-out is handled by the native authority synchronizer.
    return out;
  }

  bool vehicleStationaryForRuntimeApply(QString *reason) const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    auto numberAt=[&](const QString &key)->std::optional<double>{
      const QJsonValue value=state_.value(key);
      if (value.isDouble()) return value.toDouble();
      return std::nullopt;
    };
    double cmd=0.0, esc=0.0, wz=0.0;
    const QJsonObject cmdObj=state_.value(QStringLiteral("cmd_final")).toObject();
    if (cmdObj.value(QStringLiteral("linear_x")).isDouble()) cmd=cmdObj.value(QStringLiteral("linear_x")).toDouble();
    if (cmdObj.value(QStringLiteral("angular_z")).isDouble()) wz=cmdObj.value(QStringLiteral("angular_z")).toDouble();
    if (const auto v=numberAt(QStringLiteral("esc_drive_actual"))) esc=*v;
    if (std::abs(cmd)>0.03 || std::abs(esc)>0.03 || std::abs(wz)>0.05) {
      if (reason) *reason=QStringLiteral("Kendaraan/perintah masih bergerak; safe runtime apply menunggu |v|<=0.03 m/s dan |w|<=0.05 rad/s.");
      return false;
    }
    if (reason) reason->clear();
    return true;
  }

  QStringList exactNodeProcesses(const QString &nodeName) const {
    QString bare=nodeName; if (bare.startsWith('/')) bare.remove(0,1);
    QStringList pids;
    QDir proc(QStringLiteral("/proc"));
    for (const QString &pidText : proc.entryList(QDir::Dirs|QDir::NoDotAndDotDot,QDir::Name)) {
      bool ok=false; const qlonglong pid=pidText.toLongLong(&ok);
      if (!ok || pid<=1 || pid==QCoreApplication::applicationPid()) continue;
      QFile f(QStringLiteral("/proc/")+pidText+QStringLiteral("/cmdline"));
      if (!f.open(QIODevice::ReadOnly)) continue;
      QByteArray raw=f.readAll(); raw.replace('\0',' ');
      const QString cmd=QString::fromLocal8Bit(raw);
      if (cmd.contains(QStringLiteral("__node:=")+bare) || cmd.contains(QStringLiteral("__node:=/")+bare)) pids<<pidText;
    }
    return pids;
  }

  QString parameterNameForYamlPath(const QString &yamlPath) const {
    const QString marker=QStringLiteral(".ros__parameters.");
    const int pos=yamlPath.indexOf(marker);
    if (pos<0) return QString();
    QString name=yamlPath.mid(pos+marker.size());
    const QString tail=name.section('.',-1);
    bool numeric=false; tail.toInt(&numeric);
    if (numeric) name=name.section('.',0,-2);
    return name;
  }

  QJsonValue expectedRuntimeValue(const QString &fileKey, const QString &yamlPath) const {
    const auto candidates=configCandidates();
    if (!candidates.contains(fileKey) || !QFileInfo::exists(candidates.value(fileKey))) return QJsonValue(QJsonValue::Undefined);
    QString expectedPath=yamlPath;
    const QString tail=yamlPath.section('.',-1);
    bool numeric=false; tail.toInt(&numeric);
    if (numeric) expectedPath=yamlPath.section('.',0,-2);
    try {
      YAML::Node root=YAML::LoadFile(candidates.value(fileKey).toStdString());
      return yamlPathValue(root, expectedPath.split('.',Qt::SkipEmptyParts));
    } catch (...) { return QJsonValue(QJsonValue::Undefined); }
  }

  QJsonObject readRuntimeParameter(const QString &nodeName, const QString &parameterName, const QJsonValue &expected) {
    QJsonObject out{{"node",nodeName},{"parameter",parameterName},{"expected",expected},{"match",false}};
    if (parameterName.isEmpty()) { out["status"]="NO_PARAMETER_NAME"; return out; }
    auto client=node_->create_client<rcl_interfaces::srv::GetParameters>((nodeName+QStringLiteral("/get_parameters")).toStdString());
    if (!client->wait_for_service(1200ms)) { out["status"]="SERVICE_UNAVAILABLE"; return out; }
    auto request=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(parameterName.toStdString());
    auto future=client->async_send_request(request);
    if (future.wait_for(1500ms)!=std::future_status::ready) { out["status"]="TIMEOUT"; return out; }
    try {
      const auto response=future.get();
      if (response->values.empty()) { out["status"]="EMPTY_RESPONSE"; return out; }
      const QJsonValue actual=rosParameterValueToJson(response->values.front());
      const bool match=jsonRuntimeEquivalent(expected,actual);
      out["actual"]=actual; out["match"]=match; out["status"]=match?"MATCH":"MISMATCH";
    } catch (const std::exception &e) {
      out["status"]="ERROR"; out["message"]=QString::fromUtf8(e.what());
    }
    return out;
  }

  bool rejectReadOnly(QString *message) const {
    if (message) *message = "Web GUI berjalan dalam read-only mode";
    return false;
  }

  std::optional<double> scalarState(const QString &key) const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    const QJsonValue value = state_.value(key);
    if (!value.isDouble()) return std::nullopt;
    return value.toDouble();
  }

  bool boolState(const QString &key) const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return state_.value(key).toBool(false);
  }

  bool objectBoolState(const QString &key, const QString &field) const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return state_.value(key).toObject().value(field).toBool(false);
  }

  bool sourceConfigBool(const QString &fileKey, const QString &path, bool fallback) const {
    try {
      const auto candidates = configCandidates();
      if (!candidates.contains(fileKey) || !QFileInfo::exists(candidates.value(fileKey))) return fallback;
      const YAML::Node root = YAML::LoadFile(candidates.value(fileKey).toStdString());
      const QJsonValue value = yamlPathValue(root, path.split('.', Qt::SkipEmptyParts));
      return value.isBool() ? value.toBool() : fallback;
    } catch (...) {
      return fallback;
    }
  }

  double sourceConfigNumber(const QString &fileKey, const QString &path, double fallback) const {
    try {
      const auto candidates = configCandidates();
      if (!candidates.contains(fileKey) || !QFileInfo::exists(candidates.value(fileKey))) return fallback;
      const YAML::Node root = YAML::LoadFile(candidates.value(fileKey).toStdString());
      const QJsonValue value = yamlPathValue(root, path.split('.', Qt::SkipEmptyParts));
      return value.isDouble() && std::isfinite(value.toDouble()) ? value.toDouble() : fallback;
    } catch (...) {
      return fallback;
    }
  }

  void update(const QString &channel, const QJsonValue &value) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    state_[channel] = value;
    updated_[channel] = nowMs();
    dirty_.insert(channel);
  }

  template <typename Msg, typename Callback>
  void subscribe(const std::string &topic, const rclcpp::QoS &qos, Callback callback) {
    subscriptions_.push_back(node_->create_subscription<Msg>(topic, qos, callback));
  }

  void setupRos() {
    const auto stateQos = rclcpp::QoS(1).reliable();
    const auto latchedQos = rclcpp::QoS(1).reliable().transient_local();
    const auto sensorQos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();

    const std::vector<std::pair<const char *, const char *>> bools = {
        {"/gnss/connected", "connected.gnss"}, {"/imu/connected", "connected.imu"},
        {"/neo3/ist8310_connected", "connected.neo3_mag"},
        {"/neo3/mag_heading_valid", "neo3_mag_heading_valid"},
        {"/imu/mag_heading_valid", "imu_mag_heading_valid"},
        {"/neo3/safety_switch", "neo3_safety_switch"},
        {"/hmi/connected", "connected.hmi"},
        {"/teleop/joystick_connected", "connected.joystick"},
        {"/perception/camera_connected", "connected.camera"}, {"/esc/ready", "connected.esc_ready"},
        {"/esc/armed", "connected.esc_armed"}, {"/esc/feedback_valid", "connected.esc_feedback"},
        {"/esc/drive/connected", "connected.esc_drive"}, {"/esc/steer/connected", "connected.esc_steer"},
        {"/esc/steering_calibration/controller_ready", "esc_calibration_controller_ready"},
        {"/system/autonomy_ready", "system.autonomy_ready"}, {"/system/motion_ready", "system.motion_ready"},
        {"/system/planning_localization_ready", "system.planning_localization_ready"},
        {"/system/motion_localization_ready", "system.motion_localization_ready"},
        {"/system/nav2_ready", "system.nav2_ready"},
        {"/navigation/mppi_closed_loop/ready", "mppi_closed_loop_ready"},
        {"/navigation/velocity_smoother/closed_loop_eligible", "smoother_closed_loop_eligible"},
        {"/esc/yaw_rate_feedback/active", "yaw_rate_feedback_active"},
        {"/gnss/velocity_qualified", "gnss_velocity_qualified"}, {"/gnss/cog_qualified", "gnss_cog_qualified"},
        {"/gnss/velocity_fusion_active", "gnss_velocity_fusion_active"},
        {"/gnss/cog_fusion_active", "gnss_cog_fusion_active"},
        {"/perception/camera_healthy", "camera_healthy"},
        {"/perception/emergency_stop", "perception_emergency"}};
    for (const auto &entry : bools) {
      const QString channel = QString::fromLatin1(entry.second);
      subscribe<std_msgs::msg::Bool>(entry.first, latchedQos, [this, channel](std_msgs::msg::Bool::ConstSharedPtr msg) {
        update(channel, msg->data);
      });
    }
    subscribe<std_msgs::msg::Bool>("/safety/estop", stateQos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      update("system.estop", msg->data);
    });

    subscribe<std_msgs::msg::Bool>("/stmf4/vesc/connected", latchedQos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      update("connected.vesc_transport", msg->data);
    });
    subscribe<std_msgs::msg::Bool>("/esc/vesc/maintenance_active", latchedQos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      update("vesc_maintenance_active", msg->data);
    });

    const std::vector<std::pair<const char *, const char *>> strings = {
        {"/system/localization_state", "localization_state"}, {"/system/gnss_status", "gnss_status"},
        {"/gnss/state", "gnss_driver_state"}, {"/neo3/status", "neo3_status"},
        {"/system/magnetic_heading_status", "magnetic_heading_status"},
        {"/imu/profile_status", "imu_profile_status"},
        {"/gnss/motion_diagnostics", "gnss_motion"},
        {"/gnss/motion_validation", "gnss_motion_validation"}, {"/gnss/fusion_status", "gnss_fusion_status"},
        {"/system/imu_status", "imu_status"}, {"/system/ekf_local_status", "ekf_local_status"},
        {"/system/ekf_global_status", "ekf_global_status"}, {"/system/pose_estimator", "pose_estimator"},
        {"/system/sensor_status", "sensor_status"}, {"/navigation/goal_state", "goal_state"},
        {"/navigation/mppi_closed_loop/status", "mppi_status"},
        {"/navigation/velocity_smoother/qualification", "smoother_qualification"},
        {"/precision/dynamics_status", "precision_dynamics_status"},
        {"/esc/yaw_rate_feedback/status", "yaw_rate_feedback_status"},
        {"/perception/lane_safety_state", "lane_state"}, {"/perception/lane_control_state", "lane_control"},
        {"/yolop/lane_metrics", "lane_metrics"}, {"/perception/drivable_space", "drivable_space"},
        {"/perception/camera_health_state", "camera_health_state"}, {"/perception/near_field_state", "near_field_state"},
        {"/perception/obstacle_metrics", "obstacle_metrics"}, {"/perception/raw_detections", "raw_detections"},
        {"/perception/semantic_detections", "semantic_detections"},
        {"/perception/semantic_status", "semantic_status"},
        {"/perception/performance", "perception_performance"},
        {"/navigation/trajectory_safety_state", "trajectory_safety_state"},
        {"/collision_monitor/state", "collision_monitor_state"}, {"/esc/status", "esc_status"},
        {"/hmi/page", "hmi_page"}, {"/hmi/operator_mode", "hmi_mode"},
        {"/hmi/waypoints", "hmi_waypoints"},
        {"/hmi/navigation_state", "hmi_navigation"},
        {"/hmi/manual_state", "hmi_manual"}, {"/hmi/status", "hmi_status"},
        {"/teleop/joystick_status", "joystick_status"},
        {"/navigation/cmd_mux/source", "nav_cmd_mux"},
        {"/esc/foc/telemetry", "foc_telemetry"}, {"/esc/mux/active_source", "esc_mux"},
        {"/stmf4/vesc/status", "vesc_transport_status"}, {"/stmf4/vesc/error", "vesc_transport_error"}, {"/esc/vesc/tool_status", "vesc_tool_status"},
        {"/esc/vesc/tool_telemetry", "vesc_tool_telemetry"}, {"/esc/vesc/left_values", "vesc_left_values"}, {"/esc/vesc/right_values", "vesc_right_values"}, {"/esc/vesc/config_state", "vesc_config_state"},
        {"/esc/vesc/tuning_state", "vesc_tuning_state"}, {"/esc/vesc/position_state", "vesc_position_state"},
        {"/esc/vesc/steering_state", "vesc_steering_state"}, {"/esc/vesc/rotor_state", "vesc_rotor_state"},
        {"/esc/vesc/left_rotor_state", "vesc_left_rotor_state"}, {"/esc/vesc/right_rotor_state", "vesc_right_rotor_state"},
        {"/esc/vesc/command_state", "vesc_command_state"}, {"/esc/vesc/raw_reply", "vesc_raw_reply"}};
    for (const auto &entry : strings) {
      const QString channel = QString::fromLatin1(entry.second);
      const QString topic = QString::fromLatin1(entry.first);
      const bool perceptionStream = topic.startsWith(QStringLiteral("/perception/")) ||
                                    topic.startsWith(QStringLiteral("/yolop/"));
      const bool highRateTelemetry = topic == QStringLiteral("/esc/foc/telemetry") ||
                                     topic == QStringLiteral("/esc/vesc/tool_telemetry") ||
                                     topic == QStringLiteral("/esc/vesc/rotor_state") ||
                                     topic == QStringLiteral("/esc/vesc/left_rotor_state") ||
                                     topic == QStringLiteral("/esc/vesc/right_rotor_state");
      const bool latchedStream = topic.startsWith(QStringLiteral("/hmi/")) ||
                                    topic.startsWith(QStringLiteral("/stmf4/vesc/")) ||
                                    (topic.startsWith(QStringLiteral("/esc/vesc/")) &&
                                     topic != QStringLiteral("/esc/vesc/raw_reply")) ||
                                    topic == QStringLiteral("/esc/mux/active_source");
      const rclcpp::QoS & stringQos = (perceptionStream || highRateTelemetry) ? sensorQos : (latchedStream ? latchedQos : stateQos);
      subscribe<std_msgs::msg::String>(entry.first, stringQos, [this, channel](std_msgs::msg::String::ConstSharedPtr msg) {
        const QString raw = QString::fromStdString(msg->data);
        if (channel == "goal_state") update(channel, QJsonObject{{"state", raw.trimmed().toUpper()}, {"raw", raw}});
        else if (channel == "hmi_page" || channel == "hmi_mode") update(channel, raw.trimmed().toUpper());
        else if (channel == "raw_detections") update(channel, parseRawDetectionSummary(raw));
        else if (channel == "perception_performance") update(channel, normalizePerceptionPerformance(parseJsonOrKv(raw)));
        else if (channel == "obstacle_metrics") update(channel, enrichObstacleMetrics(parseJsonOrKv(raw)));
        else update(channel, parseJsonOrKv(raw));
      });
    }

    subscribe<sensor_msgs::msg::NavSatFix>("/gnss/fix_raw", sensorQos, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
      update("gnss_fix", QJsonObject{{"lat", msg->latitude}, {"lon", msg->longitude}, {"alt", msg->altitude},
                                     {"status", msg->status.status}, {"cov_x", msg->position_covariance[0]},
                                     {"cov_y", msg->position_covariance[4]},
                                     {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
    });

    subscribe<std_msgs::msg::Float64MultiArray>("/gnss/quality", sensorQos,
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          std::vector<double> data = msg->data;
          data.resize(std::max<size_t>(45, data.size()), std::numeric_limits<double>::quiet_NaN());
          const char *keys[] = {"sat", "dop", "hacc_m", "fix_type", "source_id", "sacc_mps", "ground_speed_mps",
                                "course_enu_rad", "course_accuracy_rad", "itow_ms", "vacc_m", "vel_e_mps", "vel_n_mps",
                                "vel_d_mps", "gdop", "hdop", "vdop", "ndop", "edop", "tdop", "nav_cov_pos_valid",
                                "nav_cov_vel_valid", "pvt_rate_hz", "measurement_age_sec", "timestamp_source_code", "flags2",
                                "flags3", "nav_dop_itow_ms", "nav_dop_exact_epoch", "nav_cov_itow_ms", "nav_cov_exact_epoch",
                                "utc_valid_flags", "tacc_ns", "height_ellipsoid_m", "head_vehicle_enu_rad", "mag_declination_rad",
                                "mag_accuracy_rad", "diff_solution", "carrier_solution", "invalid_llh", "last_correction_age_code",
                                "auth_time", "head_vehicle_valid", "mag_valid", "gnss_fix_ok"};
          QJsonObject object;
          for (int i = 0; i < 45; ++i) {
            if (std::isfinite(data[static_cast<size_t>(i)])) object[keys[i]] = data[static_cast<size_t>(i)];
            else object[keys[i]] = QJsonValue();
          }
          for (const char *key : {"nav_cov_pos_valid", "nav_cov_vel_valid", "nav_dop_exact_epoch", "nav_cov_exact_epoch",
                                  "diff_solution", "invalid_llh", "auth_time", "head_vehicle_valid", "mag_valid", "gnss_fix_ok"}) {
            const QJsonValue value = object.value(key);
            object[key] = value.isDouble() && value.toDouble() > 0.5;
          }
          update("gnss_quality", object);
        });

    const auto velocitySubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<geometry_msgs::msg::TwistWithCovarianceStamped>(
          topic, sensorQos, [this, ch](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
            const double vx = msg->twist.twist.linear.x;
            const double vy = msg->twist.twist.linear.y;
            update(ch, QJsonObject{{"vx", vx}, {"vy", vy}, {"vz", msg->twist.twist.linear.z},
                                   {"speed", std::hypot(vx, vy)},
                                   {"course_enu_rad", std::hypot(vx, vy) > 1e-9 ? std::atan2(vy, vx) : 0.0},
                                   {"cov_x", msg->twist.covariance[0]}, {"cov_y", msg->twist.covariance[7]},
                                   {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
          });
    };
    velocitySubscribe("/gnss/vel", "gnss_vel");
    velocitySubscribe("/gnss/velocity_position_fit", "gnss_vel_fit");
    velocitySubscribe("/gnss/vel_map", "gnss_vel_map");
    velocitySubscribe("/gnss/base_velocity", "gnss_base_vel");
    velocitySubscribe("/gnss/base_velocity_fusion", "gnss_base_vel_fusion");
    subscribe<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/vehicle/twist_fused", sensorQos,
      [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
        update("vehicle_twist_fused", QJsonObject{
          {"vx", msg->twist.twist.linear.x}, {"wz", msg->twist.twist.angular.z},
          {"var_v", msg->twist.covariance[0]}, {"var_w", msg->twist.covariance[35]},
          {"frame_id", QString::fromStdString(msg->header.frame_id)},
          {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
      });

    subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/gnss/cog_heading_fusion", sensorQos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          const auto &q = msg->pose.pose.orientation;
          update("gnss_cog_fusion", QJsonObject{{"yaw_rad", yawFromQuat(q.x, q.y, q.z, q.w)},
                                                 {"yaw_variance", msg->pose.covariance[35]},
                                                 {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
        });

    const auto magSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<sensor_msgs::msg::MagneticField>(topic, sensorQos,
          [this, ch](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
            const double x_ut = msg->magnetic_field.x * 1.0e6;
            const double y_ut = msg->magnetic_field.y * 1.0e6;
            const double z_ut = msg->magnetic_field.z * 1.0e6;
            update(ch, QJsonObject{{"x_ut", x_ut}, {"y_ut", y_ut}, {"z_ut", z_ut},
                                   {"norm_ut", std::sqrt(x_ut*x_ut + y_ut*y_ut + z_ut*z_ut)},
                                   {"var_x_t2", msg->magnetic_field_covariance[0]},
                                   {"var_y_t2", msg->magnetic_field_covariance[4]},
                                   {"var_z_t2", msg->magnetic_field_covariance[8]},
                                   {"frame_id", QString::fromStdString(msg->header.frame_id)},
                                   {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
          });
    };
    magSubscribe("/neo3/mag", "neo3_mag");
    magSubscribe("/imu/mag", "imu_mag");

    const auto magneticHeadingSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>(topic, sensorQos,
          [this, ch](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
            const auto &q = msg->pose.pose.orientation;
            update(ch, QJsonObject{{"yaw_rad", yawFromQuat(q.x, q.y, q.z, q.w)},
                                   {"yaw_variance", msg->pose.covariance[35]},
                                   {"frame_id", QString::fromStdString(msg->header.frame_id)},
                                   {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
          });
    };
    magneticHeadingSubscribe("/neo3/mag_heading_fusion", "neo3_mag_heading");
    magneticHeadingSubscribe("/imu/mag_heading_fusion", "imu_mag_heading");

    subscribe<std_msgs::msg::Float64MultiArray>("/imu/raw_sensor_vectors", sensorQos,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() < 9U) return;
        const double ax=msg->data[0], ay=msg->data[1], az=msg->data[2];
        const double an=std::sqrt(ax*ax+ay*ay+az*az);
        QString mounting=QStringLiteral("UNKNOWN"), detail=QStringLiteral("gravity belum stabil");
        bool mountingOk=false;
        if (std::isfinite(an) && std::abs(an-9.80665)<=1.5) {
          if (az>7.2 && std::abs(ax)<4.5 && std::abs(ay)<4.5) { mounting="TOP_UP"; mountingOk=true; detail="horizontal/top-up"; }
          else if (az<-7.2) { mounting="UPSIDE_DOWN"; detail="Z sensor terbalik"; }
          else if (std::abs(ax)>7.2 || std::abs(ay)>7.2) { mounting="SIDE_MOUNTED"; detail="gravity dominan pada X/Y"; }
          else { mounting="TILT_MISMATCH"; detail="sensor terlalu miring untuk kalibrasi planar"; }
        }
        update("imu_raw_sensor", QJsonObject{{"ax",ax},{"ay",ay},{"az",az},{"acc_norm",an},
          {"gx",msg->data[3]},{"gy",msg->data[4]},{"gz",msg->data[5]},
          {"mx_lsb",msg->data[6]},{"my_lsb",msg->data[7]},{"mz_lsb",msg->data[8]},
          {"mounting_state",mounting},{"mounting_ok",mountingOk},{"mounting_detail",detail}});
      });

    subscribe<sensor_msgs::msg::Imu>("/imu/data", sensorQos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      const auto &q = msg->orientation;
      const double sinr = 2 * (q.w * q.x + q.y * q.z);
      const double cosr = 1 - 2 * (q.x * q.x + q.y * q.y);
      const double roll = std::atan2(sinr, cosr);
      const double sinp = 2 * (q.w * q.y - q.z * q.x);
      const double pitch = std::abs(sinp) >= 1 ? std::copysign(kPi / 2, sinp) : std::asin(sinp);
      update("imu", QJsonObject{{"roll_rad", roll}, {"pitch_rad", pitch}, {"yaw_rad", yawFromQuat(q.x, q.y, q.z, q.w)},
                                {"gx", msg->angular_velocity.x}, {"gy", msg->angular_velocity.y}, {"gz", msg->angular_velocity.z},
                                {"ax", msg->linear_acceleration.x}, {"ay", msg->linear_acceleration.y}, {"az", msg->linear_acceleration.z},
                                {"var_roll", msg->orientation_covariance[0]}, {"var_pitch", msg->orientation_covariance[4]},
                                {"var_yaw", msg->orientation_covariance[8]},
                                {"var_gx", msg->angular_velocity_covariance[0]}, {"var_gy", msg->angular_velocity_covariance[4]},
                                {"var_gz", msg->angular_velocity_covariance[8]},
                                {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
    });

    const auto odomSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<nav_msgs::msg::Odometry>(topic, sensorQos, [this, ch](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        update(ch, QJsonObject{{"x", p.x}, {"y", p.y}, {"yaw", yawFromQuat(q.x, q.y, q.z, q.w)},
                               {"v", msg->twist.twist.linear.x}, {"w", msg->twist.twist.angular.z},
                               {"var_x", msg->pose.covariance[0]}, {"var_y", msg->pose.covariance[7]},
                               {"var_yaw", msg->pose.covariance[35]},
                               {"var_v", msg->twist.covariance[0]}, {"var_w", msg->twist.covariance[35]},
                               {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
      });
    };
    odomSubscribe("/esc/odom", "esc_odom");
    odomSubscribe("/odometry/gnss_map", "gnss_map_odom");
    odomSubscribe("/odometry/filtered", "ekf_local");
    odomSubscribe("/odometry/filtered_map", "ekf_global");

    const auto pathSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<nav_msgs::msg::Path>(topic, sensorQos, [this, ch](nav_msgs::msg::Path::ConstSharedPtr msg) {
        QJsonArray points;
        double length = 0.0;
        double headingVariation = 0.0;
        double previousYaw = 0.0;
        bool havePrevious = false;
        const size_t stride = std::max<size_t>(1, msg->poses.size() / 800 + 1);
        for (size_t i = 0; i < msg->poses.size(); ++i) {
          const auto &p = msg->poses[i].pose.position;
          const auto &q = msg->poses[i].pose.orientation;
          const double yaw = yawFromQuat(q.x, q.y, q.z, q.w);
          if (i > 0) {
            const auto &prev = msg->poses[i - 1].pose.position;
            length += std::hypot(p.x - prev.x, p.y - prev.y);
          }
          if (havePrevious) {
            double d = yaw - previousYaw;
            while (d > kPi) d -= 2 * kPi;
            while (d < -kPi) d += 2 * kPi;
            headingVariation += std::abs(d);
          }
          previousYaw = yaw;
          havePrevious = true;
          if (i % stride == 0 || i + 1 == msg->poses.size()) points.append(QJsonArray{p.x, p.y, yaw});
        }
        update(ch, QJsonObject{{"count", static_cast<double>(msg->poses.size())}, {"length_m", length},
                               {"heading_variation_rad", headingVariation}, {"frame_id", QString::fromStdString(msg->header.frame_id)},
                               {"points", points},
                               {"measurement_stamp_sec", double(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9}});
      });
    };
    pathSubscribe("/plan", "nav_path");
    pathSubscribe("/transformed_global_plan", "local_path");
    pathSubscribe("/controller_server/transformed_global_plan", "local_path");
    pathSubscribe("/local_plan", "local_path");

    // Nav2 Humble MPPI TrajectoryVisualizer publishes a MarkerArray on the
    // controller node's relative `trajectories` topic.  Bridge candidate and
    // optimal trajectories so the browser map can show the same control output
    // that is normally inspected in RViz.
    subscribe<visualization_msgs::msg::MarkerArray>(
      "/trajectories", sensorQos,
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        QJsonArray trajectories;
        size_t pointCount = 0;
        size_t optimalCount = 0;
        for (const auto &marker : msg->markers) {
          if (marker.action == visualization_msgs::msg::Marker::DELETE ||
              marker.action == visualization_msgs::msg::Marker::DELETEALL || marker.points.empty()) continue;
          QJsonArray points;
          const size_t stride = std::max<size_t>(1, marker.points.size() / 160 + 1);
          for (size_t i = 0; i < marker.points.size(); ++i) {
            if (i % stride != 0 && i + 1 != marker.points.size()) continue;
            const auto &p = marker.points[i];
            points.append(QJsonArray{p.x, p.y, p.z});
          }
          if (points.size() < 2) continue;
          const QString ns = QString::fromStdString(marker.ns);
          const bool optimal = ns.contains(QStringLiteral("Optimal"), Qt::CaseInsensitive);
          if (optimal) ++optimalCount;
          pointCount += static_cast<size_t>(points.size());
          trajectories.append(QJsonObject{
            {"ns", ns}, {"id", marker.id}, {"frame_id", QString::fromStdString(marker.header.frame_id)},
            {"optimal", optimal}, {"points", points}});
        }
        update("mppi_trajectories", QJsonObject{
          {"count", trajectories.size()}, {"optimal_count", static_cast<double>(optimalCount)},
          {"point_count", static_cast<double>(pointCount)}, {"trajectories", trajectories}});
      });

    const auto goalSubscribe = [this, stateQos](const char *topic) {
      subscribe<geometry_msgs::msg::PoseStamped>(topic, stateQos, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        const auto &q = msg->pose.orientation;
        update("goal_pose", QJsonObject{{"x", msg->pose.position.x}, {"y", msg->pose.position.y},
                                        {"yaw", yawFromQuat(q.x, q.y, q.z, q.w)}});
      });
    };
    goalSubscribe("/navigation/goal_request");
    goalSubscribe("/goal_pose");

    const auto twistSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<geometry_msgs::msg::Twist>(topic, sensorQos, [this, ch](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        update(ch, QJsonObject{{"linear_x", msg->linear.x}, {"angular_z", msg->angular.z}});
      });
    };
    twistSubscribe("/cmd_vel_nav_raw", "cmd_nav");
    twistSubscribe("/cmd_vel/teleop", "cmd_teleop");
    twistSubscribe("/cmd_vel/pre_smoother", "cmd_pre_smoother");
    twistSubscribe("/cmd_vel/perception_advisory", "cmd_perception_advisory");
    twistSubscribe("/cmd_vel/autonomy_integrated", "cmd_autonomy_integrated");
    twistSubscribe("/cmd_vel/nav2_pre_collision", "cmd_pre_collision");
    twistSubscribe("/cmd_vel/collision_preview", "cmd_collision_preview");
    twistSubscribe("/cmd_vel", "cmd_final");
    twistSubscribe("/cmd_vel/actuator", "cmd_actuator");

    const std::vector<std::pair<const char *, const char *>> floats = {
        {"/esc/drive_target_mps", "esc_drive_target"}, {"/esc/drive_actual_mps", "esc_drive_actual"},
        {"/esc/drive_raw_mps", "esc_drive_raw"}, {"/imu/speed_kalman", "imu_speed_kalman"},
        {"/imu/forward_accel_filtered", "imu_forward_accel_filtered"}, {"/imu/forward_accel_bias", "imu_forward_accel_bias"},
        {"/esc/steering_target_rad", "esc_steer_target"},
        {"/esc/steering_command_uncalibrated_rad", "esc_steer_uncal_target"},
        {"/esc/steering_uncalibrated_rad", "esc_steer_uncalibrated"},
        {"/esc/steering_protocol_command_rad", "esc_steer_protocol_cmd"},
        {"/esc/steering_feedback_raw_rad", "esc_steer_feedback_raw"},
        {"/esc/steering_actual_rad", "esc_steer_actual"}, {"/esc/yaw_rate_actual_rps", "esc_yaw_rate"},
        {"/esc/kinematic_yaw_rate_rps", "esc_kinematic_yaw_rate"},
        {"/navigation/mppi_closed_loop/velocity_error_mps", "mppi_velocity_error"},
        {"/navigation/mppi_closed_loop/steering_error_rad", "mppi_steering_error"},
        {"/navigation/mppi_closed_loop/yaw_rate_error_rps", "mppi_yaw_error"},
        {"/precision/steering_predicted_rad", "precision_steering_predicted"},
        {"/precision/steering_offset_rad", "precision_steering_offset"},
        {"/precision/yaw_rate_innovation_rps", "precision_yaw_innovation"},
        {"/precision/twist_sync_gap_sec", "precision_twist_sync_gap"},
        {"/esc/yaw_rate_feedback/target_rps", "yaw_rate_feedback_target"},
        {"/esc/yaw_rate_feedback/measured_rps", "yaw_rate_feedback_measured"},
        {"/esc/yaw_rate_feedback/error_rps", "yaw_rate_feedback_error"},
        {"/esc/yaw_rate_feedback/correction_deg", "yaw_rate_feedback_correction_deg"}};
    for (const auto &entry : floats) {
      const QString channel = QString::fromLatin1(entry.second);
      subscribe<std_msgs::msg::Float64>(entry.first, sensorQos, [this, channel](std_msgs::msg::Float64::ConstSharedPtr msg) {
        update(channel, msg->data);
      });
    }

    subscribe<sensor_msgs::msg::CompressedImage>("/camera/astra/image_preview/compressed", sensorQos,
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        if (!msg || msg->data.empty()) return;
        const auto now = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(mediaMutex_);
          cameraJpeg_ = QByteArray(reinterpret_cast<const char *>(msg->data.data()), static_cast<int>(msg->data.size()));
        }
        {
          std::lock_guard<std::mutex> encodeLock(cameraEncodeMutex_);
          lastCompressedCameraSeen_ = now;
          lastCameraEncode_ = now;
        }
        const QString format = QString::fromStdString(msg->format);
        const QString source = format.contains(QStringLiteral("yolop_annotated")) ?
          QStringLiteral("yolop_annotated") : QStringLiteral("camera_raw");
        update("camera_frame", QJsonObject{{"encoding", "jpeg"}, {"source", source}, {"format", format}, {"at_ms", nowMs()},
                                            {"bytes", static_cast<double>(msg->data.size())}});
      });

    auto cacheCamera = [this](sensor_msgs::msg::Image::ConstSharedPtr msg, const QString &source) {
      // Raw and annotated image subscriptions may execute concurrently in the
      // MultiThreadedExecutor. Serialize throttle/encode state to avoid racing
      // lastCameraEncode_ and producing duplicate JPEG work on the mini-PC.
      std::lock_guard<std::mutex> encodeLock(cameraEncodeMutex_);
      const auto now = std::chrono::steady_clock::now();
      if (lastCompressedCameraSeen_.time_since_epoch().count() != 0 &&
          std::chrono::duration<double>(now - lastCompressedCameraSeen_).count() < 1.0) return;
      if (source == QStringLiteral("raw")) lastRawCameraSeen_ = now;
      const double elapsed = std::chrono::duration<double>(now - lastCameraEncode_).count();
      if (lastCameraEncode_.time_since_epoch().count() != 0 && elapsed < 1.0 / cameraJpegFps_) return;
      if (msg->width == 0 || msg->height == 0 || msg->step == 0) return;
      const std::uint64_t requiredBytes = static_cast<std::uint64_t>(msg->step) * static_cast<std::uint64_t>(msg->height);
      if (requiredBytes > msg->data.size()) return;
      QImage image;
      if (msg->encoding == "rgb8" && msg->step >= msg->width * 3U) {
        image = QImage(msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height),
                       static_cast<int>(msg->step), QImage::Format_RGB888).copy();
      } else if (msg->encoding == "bgr8" && msg->step >= msg->width * 3U) {
        image = QImage(msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height),
                       static_cast<int>(msg->step), QImage::Format_RGB888).rgbSwapped().copy();
      } else if ((msg->encoding == "mono8" || msg->encoding == "8UC1") && msg->step >= msg->width) {
        image = QImage(msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height),
                       static_cast<int>(msg->step), QImage::Format_Grayscale8).copy();
      }
      if (image.isNull()) return;
      QByteArray encoded;
      QBuffer buffer(&encoded);
      if (!buffer.open(QIODevice::WriteOnly) || !image.save(&buffer, "JPG", 78) || encoded.isEmpty()) return;
      {
        std::lock_guard<std::mutex> lock(mediaMutex_);
        cameraJpeg_ = encoded;
      }
      lastCameraEncode_ = now;
      update("camera_frame", QJsonObject{{"width", static_cast<int>(msg->width)}, {"height", static_cast<int>(msg->height)},
                                         {"encoding", QString::fromStdString(msg->encoding)}, {"source", source}, {"at_ms", nowMs()}});
    };
    subscribe<sensor_msgs::msg::Image>("/camera/astra/image_raw", sensorQos,
                                      [cacheCamera](sensor_msgs::msg::Image::ConstSharedPtr msg) { cacheCamera(msg, "raw"); });
    subscribe<sensor_msgs::msg::Image>("/camera/yolop/image_annotated", sensorQos,
                                      [cacheCamera](sensor_msgs::msg::Image::ConstSharedPtr msg) { cacheCamera(msg, "annotated"); });

    subscribe<nav_msgs::msg::OccupancyGrid>("/map", latchedQos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) return;
      const std::uint64_t cellCount = static_cast<std::uint64_t>(msg->info.width) * static_cast<std::uint64_t>(msg->info.height);
      if (cellCount > msg->data.size() || cellCount > 100000000ULL ||
          msg->info.width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
          msg->info.height > static_cast<unsigned int>(std::numeric_limits<int>::max())) return;
      QImage image(static_cast<int>(msg->info.width), static_cast<int>(msg->info.height), QImage::Format_Grayscale8);
      if (image.isNull()) return;
      for (unsigned int y = 0; y < msg->info.height; ++y) {
        uchar *row = image.scanLine(static_cast<int>(msg->info.height - 1 - y));
        for (unsigned int x = 0; x < msg->info.width; ++x) {
          const int8_t occ = msg->data[static_cast<size_t>(y) * msg->info.width + x];
          int shade = 118;
          if (occ >= 0) shade = std::clamp(245 - static_cast<int>(occ) * 2, 35, 245);
          row[x] = static_cast<uchar>(shade);
        }
      }
      QByteArray encoded;
      QBuffer buffer(&encoded);
      if (!buffer.open(QIODevice::WriteOnly) || !image.save(&buffer, "PNG") || encoded.isEmpty()) return;
      {
        std::lock_guard<std::mutex> lock(mediaMutex_);
        mapPng_ = encoded;
      }
      const auto &origin = msg->info.origin.position;
      update("map_meta", QJsonObject{{"width", static_cast<int>(msg->info.width)}, {"height", static_cast<int>(msg->info.height)},
                                     {"resolution", msg->info.resolution}, {"origin_x", origin.x}, {"origin_y", origin.y},
                                     {"frame_id", QString::fromStdString(msg->header.frame_id)}, {"at_ms", nowMs()}});
    });

    // RViz-like costmap layers for the Web HMI. These are the actual Nav2
    // OccupancyGrid outputs, not a fabricated inflation preview. Encoding is
    // throttled and downsampled to keep the mini-PC responsive on large maps.
    const auto cacheCostmap = [this, latchedQos](const char *topic, const char *metaKey, bool global) {
      subscribe<nav_msgs::msg::OccupancyGrid>(topic, latchedQos,
        [this, metaKey, global](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) return;
          const std::uint64_t cells = static_cast<std::uint64_t>(msg->info.width) * msg->info.height;
          if (cells > msg->data.size() || cells > 100000000ULL) return;

          std::lock_guard<std::mutex> encodeLock(costmapEncodeMutex_);
          auto &last = global ? lastGlobalCostmapEncode_ : lastLocalCostmapEncode_;
          const auto now = std::chrono::steady_clock::now();
          const double minPeriod = global ? 1.0 : 0.25;
          if (last.time_since_epoch().count() != 0 &&
              std::chrono::duration<double>(now - last).count() < minPeriod) return;

          constexpr unsigned int kMaxRaster = 1600U;
          const unsigned int largest = std::max(msg->info.width, msg->info.height);
          const unsigned int stride = std::max(1U, (largest + kMaxRaster - 1U) / kMaxRaster);
          const unsigned int outW = (msg->info.width + stride - 1U) / stride;
          const unsigned int outH = (msg->info.height + stride - 1U) / stride;
          QImage image(static_cast<int>(outW), static_cast<int>(outH), QImage::Format_RGBA8888);
          if (image.isNull()) return;
          image.fill(Qt::transparent);

          for (unsigned int oy = 0; oy < outH; ++oy) {
            uchar *row = image.scanLine(static_cast<int>(outH - 1U - oy));
            for (unsigned int ox = 0; ox < outW; ++ox) {
              int maxCost = -1;
              const unsigned int y0 = oy * stride;
              const unsigned int x0 = ox * stride;
              const unsigned int y1 = std::min(msg->info.height, y0 + stride);
              const unsigned int x1 = std::min(msg->info.width, x0 + stride);
              for (unsigned int y = y0; y < y1; ++y) {
                const size_t base = static_cast<size_t>(y) * msg->info.width;
                for (unsigned int x = x0; x < x1; ++x) maxCost = std::max(maxCost, static_cast<int>(msg->data[base + x]));
              }
              uchar *px = row + static_cast<size_t>(ox) * 4U;
              if (maxCost <= 0) { px[0]=0; px[1]=0; px[2]=0; px[3]=0; continue; }
              const int bounded = std::clamp(maxCost, 1, 100);
              const int alpha = std::clamp(42 + bounded * 2, 48, 225);
              if (bounded >= 90) { px[0]=255; px[1]=72; px[2]=88; }
              else if (bounded >= 50) { px[0]=255; px[1]=159; px[2]=64; }
              else { px[0]=181; px[1]=119; px[2]=255; }
              px[3]=static_cast<uchar>(alpha);
            }
          }

          QByteArray encoded;
          QBuffer buffer(&encoded);
          if (!buffer.open(QIODevice::WriteOnly) || !image.save(&buffer, "PNG") || encoded.isEmpty()) return;
          {
            std::lock_guard<std::mutex> lock(mediaMutex_);
            if (global) globalCostmapPng_ = encoded; else localCostmapPng_ = encoded;
          }
          last = now;
          const auto &origin = msg->info.origin.position;
          update(QString::fromLatin1(metaKey), QJsonObject{
            {"width", static_cast<int>(msg->info.width)}, {"height", static_cast<int>(msg->info.height)},
            {"image_width", static_cast<int>(outW)}, {"image_height", static_cast<int>(outH)},
            {"stride", static_cast<int>(stride)}, {"resolution", msg->info.resolution},
            {"origin_x", origin.x}, {"origin_y", origin.y},
            {"frame_id", QString::fromStdString(msg->header.frame_id)}, {"at_ms", nowMs()}});
        });
    };
    cacheCostmap("/global_costmap/costmap", "global_costmap_meta", true);
    cacheCostmap("/local_costmap/costmap", "local_costmap_meta", false);

    const auto cloudSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<sensor_msgs::msg::PointCloud2>(topic, sensorQos, [this, ch](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        update(ch, QJsonObject{{"count", static_cast<double>(msg->width) * static_cast<double>(msg->height)},
                               {"frame_id", QString::fromStdString(msg->header.frame_id)}});
      });
    };
    cloudSubscribe("/perception/object_points", "object_points");
    cloudSubscribe("/perception/path_relevant_points", "path_relevant_points");
    cloudSubscribe("/perception/planning_relevant_points", "planning_relevant_points");
    cloudSubscribe("/perception/drivable_boundary_points", "drivable_boundary_points");

    // Fixed-frame transform used to place the odom-frame local costmap exactly
    // where RViz would render it in the map frame. Failure is non-fatal while
    // localization is starting; the Web layer simply remains WAIT.
    tfTimer_ = node_->create_wall_timer(200ms, [this]() {
      if (!tfBuffer_) return;
      try {
        const auto tf = tfBuffer_->lookupTransform("map", "odom", tf2::TimePointZero);
        update("map_odom_tf", QJsonObject{{"x", tf.transform.translation.x}, {"y", tf.transform.translation.y},
          {"yaw", yawFromQuat(tf.transform.rotation.x, tf.transform.rotation.y,
                              tf.transform.rotation.z, tf.transform.rotation.w)},
          {"at_ms", nowMs()}});
      } catch (const std::exception &) {
        // Expected during STARTUP/DEGRADED localization.
      }
    });

    goalPub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/navigation/goal_request", 10);
    initialPosePub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    hmiRequestPub_ = node_->create_publisher<std_msgs::msg::String>("/hmi/request", 10);
    vescToolCommandPub_ = node_->create_publisher<std_msgs::msg::String>("/esc/vesc/tool_command", 20);
    trialTwistPub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/teleop", 10);
    trialSourcePub_ = node_->create_publisher<std_msgs::msg::String>("/teleop/active_source", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }
};

class LocalHttpServer : public QObject {
 public:
  LocalHttpServer(WebRosBridge *bridge, QObject *parent = nullptr) : QObject(parent), bridge_(bridge) {
    connect(&server_, &QTcpServer::newConnection, this, [this]() { acceptConnections(); });
    // Stream state deltas at the same 50 Hz class as actuator telemetry.
    // Browser-side rendering is independently throttled, so ingestion stays lossless/current.
    eventTimer_.setInterval(20);
    connect(&eventTimer_, &QTimer::timeout, this, [this]() { broadcastEvents(); });
    bindRetryTimer_.setInterval(1000);
    connect(&bindRetryTimer_, &QTimer::timeout, this, [this]() { retryListen(); });
    recordingTimer_.setInterval(200);
    connect(&recordingTimer_, &QTimer::timeout, this, [this]() { captureRecordingSample(); });
    hostTimer_.setInterval(1000);
    connect(&hostTimer_, &QTimer::timeout, this, [this]() { sampleHostMetrics(); });
    imuCalibrationProcess_.setProcessChannelMode(QProcess::MergedChannels);
    connect(&imuCalibrationProcess_, &QProcess::readyReadStandardOutput, this, [this]() {
      imuCalibrationLastOutput_ += QString::fromUtf8(imuCalibrationProcess_.readAllStandardOutput());
      if (imuCalibrationLastOutput_.size() > 4000) imuCalibrationLastOutput_ = imuCalibrationLastOutput_.right(4000);
    });
  }

  bool start(const QString &bindAddress, int port) {
    if (!resolveStaticRoot()) return false;
    QHostAddress address;
    if (!address.setAddress(bindAddress)) {
      if (bindAddress == "localhost") address = QHostAddress::LocalHost;
      else return false;
    }
    bindAddress_ = address;
    bindAddressText_ = bindAddress;
    bindPort_ = static_cast<quint16>(port);
    hostTimer_.start();
    sampleHostMetrics();
    if (!server_.listen(bindAddress_, bindPort_)) {
      RCLCPP_WARN(rclcpp::get_logger("agv_web_gui"),
        "Port web %s:%d belum tersedia (%s); node tetap hidup dan retry internal 1 Hz",
        bindAddress.toUtf8().constData(), port, server_.errorString().toUtf8().constData());
      bindRetryTimer_.start();
      return true;
    }
    eventTimer_.start();
    return true;
  }

  bool isListening() const { return server_.isListening(); }

 private:
  bool resolveStaticRoot() {
    try {
      staticRoot_ = QString::fromStdString(ament_index_cpp::get_package_share_directory("navigation")) + "/web/static";
    } catch (...) {
      staticRoot_.clear();
    }
    const QString envRoot = QString::fromUtf8(qgetenv("AGV_WEB_STATIC_DIR"));
    if (!envRoot.isEmpty() && QDir(envRoot).exists()) staticRoot_ = envRoot;
    const QString canonicalRoot = QFileInfo(staticRoot_).canonicalFilePath();
    if (canonicalRoot.isEmpty() || !QFileInfo(canonicalRoot + "/index.html").isFile()) {
      staticRoot_.clear();
      return false;
    }
    staticRoot_ = canonicalRoot;
    return true;
  }

  void sampleHostMetrics() {
    QJsonObject host{{"source", "local_os"}, {"at_ms", nowMs()}};
    QFile statFile(QStringLiteral("/proc/stat"));
    if (statFile.open(QIODevice::ReadOnly)) {
      const QStringList p = QString::fromLocal8Bit(statFile.readLine()).simplified().split(' ');
      if (p.size() >= 6 && p.first() == QStringLiteral("cpu")) {
        quint64 total = 0;
        for (int i = 1; i < p.size(); ++i) total += p[i].toULongLong();
        const quint64 idle = p[4].toULongLong() + (p.size() > 5 ? p[5].toULongLong() : 0ULL);
        if (hostPrevTotal_ > 0 && total > hostPrevTotal_) {
          const quint64 dt = total - hostPrevTotal_, di = idle >= hostPrevIdle_ ? idle - hostPrevIdle_ : 0ULL;
          host["cpu_percent"] = 100.0 * static_cast<double>(dt - std::min(dt, di)) / static_cast<double>(dt);
        }
        hostPrevTotal_ = total; hostPrevIdle_ = idle;
      }
    }
    double memTotalKb = 0.0, memAvailKb = 0.0;
    QFile memFile(QStringLiteral("/proc/meminfo"));
    if (memFile.open(QIODevice::ReadOnly)) while (!memFile.atEnd()) {
      const QStringList p = QString::fromLocal8Bit(memFile.readLine()).simplified().split(' ');
      if (p.size() < 2) continue;
      if (p[0] == QStringLiteral("MemTotal:")) memTotalKb = p[1].toDouble();
      else if (p[0] == QStringLiteral("MemAvailable:")) memAvailKb = p[1].toDouble();
    }
    if (memTotalKb > 0.0 && memAvailKb >= 0.0) {
      host["ram_percent"] = 100.0 * (memTotalKb - memAvailKb) / memTotalKb;
      host["ram_used_mb"] = (memTotalKb - memAvailKb) / 1024.0;
      host["ram_total_mb"] = memTotalKb / 1024.0;
    }
    double maxTemp = std::numeric_limits<double>::quiet_NaN();
    const QDir thermal(QStringLiteral("/sys/class/thermal"));
    for (const QString &zone : thermal.entryList({QStringLiteral("thermal_zone*")}, QDir::Dirs | QDir::NoDotAndDotDot)) {
      QFile f(thermal.absoluteFilePath(zone + QStringLiteral("/temp")));
      if (!f.open(QIODevice::ReadOnly)) continue;
      bool ok = false; double t = QString::fromLocal8Bit(f.readAll()).trimmed().toDouble(&ok);
      if (!ok) continue;
      if (t > 1000.0) t /= 1000.0;
      if (t > 0.0 && t < 150.0 && (!std::isfinite(maxTemp) || t > maxTemp)) maxTemp = t;
    }
    if (std::isfinite(maxTemp)) host["temperature_c"] = maxTemp;
    bool gpuAvailable = false;
    for (int card = 0; card < 4 && !gpuAvailable; ++card) {
      QFile f(QStringLiteral("/sys/class/drm/card%1/device/gpu_busy_percent").arg(card));
      if (!f.open(QIODevice::ReadOnly)) continue;
      bool ok = false; const double gpu = QString::fromLocal8Bit(f.readAll()).trimmed().toDouble(&ok);
      if (ok && gpu >= 0.0 && gpu <= 100.0) { host["gpu_percent"] = gpu; gpuAvailable = true; }
    }
    host["gpu_available"] = gpuAvailable;
    bridge_->updateHostMetrics(host);
  }

  void retryListen() {
    if (server_.isListening()) { bindRetryTimer_.stop(); return; }
    if (!server_.listen(bindAddress_, bindPort_)) return;
    bindRetryTimer_.stop();
    eventTimer_.start();
    RCLCPP_INFO(rclcpp::get_logger("agv_web_gui"), "Web GUI bind pulih otomatis: http://%s:%u",
      bindAddressText_.toUtf8().constData(), static_cast<unsigned>(bindPort_));
  }

  struct Request {
    QByteArray method;
    QString path;
    QMap<QByteArray, QByteArray> headers;
    QByteArray body;
  };

  void purgeValidationSnapshots() {
    const double now = nowMs();
    for (auto it = validationSnapshots_.begin(); it != validationSnapshots_.end();) {
      if (it.value().value("expires_at_ms").toDouble() <= now) it = validationSnapshots_.erase(it);
      else ++it;
    }
  }

  void purgeConfigProposals() {
    const double now = nowMs();
    for (auto it = configProposals_.begin(); it != configProposals_.end();) {
      if (it.value().value("expires_at_ms").toDouble() <= now) it = configProposals_.erase(it);
      else ++it;
    }
  }

  QJsonObject registerConfigProposal(const QString &sourceTask, const QJsonArray &items, const QJsonObject &evidence = {}) {
    purgeConfigProposals();
    const QJsonObject revision = configRevisionState();
    const double created = nowMs(), expires = created + 600000.0;
    const QByteArray evidenceBytes = QJsonDocument(evidence).toJson(QJsonDocument::Compact);
    const QString evidenceDigest = QString::fromLatin1(QCryptographicHash::hash(evidenceBytes,QCryptographicHash::Sha256).toHex());
    const QString seed = sourceTask + QString::number(created,'f',0) + configDraftSignature(items) + revision.value("config_revision").toString();
    const QString id = QString::fromLatin1(QCryptographicHash::hash(seed.toUtf8(),QCryptographicHash::Sha256).toHex().left(24));
    QJsonObject proposal{{"proposal_id",id},{"source_task",sourceTask},{"created_at_ms",created},{"expires_at_ms",expires},
      {"base_config_revision",revision.value("config_revision")},{"items",items},{"evidence",evidence},{"evidence_digest",evidenceDigest},
      {"validation_summary",QJsonObject{{"item_count",items.size()},{"config_write",false},{"runtime_write",false}}}};
    configProposals_[id]=proposal;
    return proposal;
  }

  bool generatedProposalItemValid(const QJsonObject &item, const QString &identity, QString *reason) {
    purgeConfigProposals();
    const QString proposalId=item.value("proposal_id").toString();
    if(proposalId.isEmpty()||!configProposals_.contains(proposalId)){if(reason)*reason="GENERATED_PROPOSAL_REQUIRED";return false;}
    const QJsonObject proposal=configProposals_.value(proposalId);
    if(proposal.value("base_config_revision").toString()!=configRevisionState().value("config_revision").toString()){
      if (reason) *reason = "GENERATED_PROPOSAL_STALE_CONFIG";
      return false;
    }
    for(const QJsonValue &v:proposal.value("items").toArray()){
      const QJsonObject p=v.toObject();const QString pid=p.value("file_key").toString()+":"+p.value("path").toString();
      if(pid==identity&&jsonRuntimeEquivalent(p.value("value"),item.value("value")))return true;
    }
    if (reason) *reason = "GENERATED_PROPOSAL_ITEM_MISMATCH";
    return false;
  }

  static QJsonArray runtimeActuals(const QJsonObject &runtime) {
    QJsonArray out;
    for (const QJsonValue &v : runtime.value("readback").toArray()) {
      const QJsonObject r = v.toObject();
      if (!r.contains("actual")) continue;
      out.append(QJsonObject{{"node", r.value("node")}, {"parameter", r.value("parameter")}, {"actual", r.value("actual")}});
    }
    return out;
  }

  QJsonObject validateConfigBatch(const QJsonArray &items, bool storeSnapshot) {
    QJsonObject result{{"ok", false}, {"valid", false}, {"can_apply", false}, {"at_ms", nowMs()}};
    if (items.isEmpty() || items.size() > 64) {
      result["message"] = "items harus berisi 1..64 parameter";
      return result;
    }
    const QJsonObject metadataRoot = loadUiParameterMetadata();
    const QJsonObject metadata = metadataRoot.value("parameters").toObject();
    const QJsonObject revisions = configRevisionState();
    const QJsonObject fileRevisions = revisions.value("files").toObject();
    QJsonArray checked, errors, warnings;
    QSet<QString> seen;
    QMap<QString, QJsonValue> draftValues;
    for (const QJsonValue &v : items) {
      if (!v.isObject()) continue;
      const QJsonObject item = v.toObject();
      const QString fk = item.value("file_key").toString().trimmed();
      const QString path = item.value("path").toString().trimmed();
      if (!fk.isEmpty() && !path.isEmpty()) draftValues[fk + QStringLiteral(":") + path] = item.value("value");
    }
    auto resolveIdentityValue = [&](const QString &identity, QJsonValue *value, QString *why) {
      if (draftValues.contains(identity)) { if (value) *value = draftValues.value(identity); return true; }
      const int sep = identity.indexOf(':');
      if (sep <= 0 || sep + 1 >= identity.size()) { if (why) *why = QStringLiteral("Dependency identity invalid: ") + identity; return false; }
      return currentYamlValue(identity.left(sep), identity.mid(sep + 1), value, why);
    };
    bool stationaryChecked = false, stationary = true;
    QString stationaryReason;
    for (const QJsonValue &v : items) {
      if (!v.isObject()) { errors.append("item harus object"); continue; }
      const QJsonObject item = v.toObject();
      const QString fk = item.value("file_key").toString().trimmed();
      const QString path = item.value("path").toString().trimmed();
      const QString identity = fk + QStringLiteral(":") + path;
      QJsonObject row{{"identity", identity}, {"file_key", fk}, {"path", path}, {"draft", item.value("value")}};
      QJsonArray itemErrors, itemWarnings;
      if (fk.isEmpty() || path.isEmpty() || seen.contains(identity)) itemErrors.append("file_key/path kosong atau duplikat");
      seen.insert(identity);
      const QJsonObject meta = metadata.value(identity).toObject();
      row["metadata"] = meta;
      row["metadata_complete"] = !meta.isEmpty() && meta.value("metadata_complete").toBool(false);
      row["risk"] = meta.value("risk"); row["apply_mode"] = meta.value("apply_mode");
      row["requires_stationary"] = meta.value("requires_stationary").toBool(false);
      if (meta.isEmpty() || !meta.value("metadata_complete").toBool(false)) itemErrors.append("METADATA_INCOMPLETE");
      if (meta.value("write_authority").toString() == QStringLiteral("calibration_generated")) {
        QString generatedReason;
        if (!item.value("generated").toBool(false) || !generatedProposalItemValid(item,identity,&generatedReason))
          itemErrors.append(generatedReason.isEmpty()?QStringLiteral("CALIBRATION_GENERATED_ONLY"):generatedReason);
      }
      if (meta.value("apply_mode").toString() == QStringLiteral("read_only")) itemErrors.append("READ_ONLY");
      QJsonValue current; QString why;
      if (!currentYamlValue(fk, path, &current, &why)) itemErrors.append(why);
      else {
        row["yaml_current"] = current;
        if (!compatibleConfigType(current, item.value("value"))) itemErrors.append("TYPE_MISMATCH");
        QString hardError, recommendedWarning;
        if (!metadataNumericWithin(meta, item.value("value"), &hardError, &recommendedWarning)) itemErrors.append(hardError);
        if (!recommendedWarning.isEmpty()) itemWarnings.append(recommendedWarning);
      }
      QJsonValue baseline;
      if (baselineYamlValue(fk, path, &baseline, nullptr)) row["baseline"] = baseline;
      row["file_revision"] = fileRevisions.value(fk);
      const QJsonObject runtime = bridge_->configRuntimeState(fk, path);
      row["runtime"] = runtime;
      if (meta.value("requires_stationary").toBool(false)) {
        if (!stationaryChecked) { stationary = bridge_->vehicleStationary(&stationaryReason); stationaryChecked = true; }
        if (!stationary) itemErrors.append(QStringLiteral("STATIONARY_REQUIRED: ") + stationaryReason);
      }
      const QJsonArray options = meta.value("options").toArray();
      if (!options.isEmpty()) {
        bool found = false; for (const QJsonValue &o : options) if (o == item.value("value")) { found = true; break; }
        if (!found) itemErrors.append("ENUM_INVALID");
      }
      if (meta.contains("list_length")) {
        const int expectedLength = meta.value("list_length").toInt(-1);
        if (!item.value("value").isArray() || expectedLength < 0 || item.value("value").toArray().size() != expectedLength)
          itemErrors.append(QStringLiteral("LIST_LENGTH_MISMATCH expected=") + QString::number(expectedLength));
      }
      if (!meta.value("array_bounds").toArray().isEmpty()) {
        if (!item.value("value").isArray()) itemErrors.append("ARRAY_BOUNDS_REQUIRE_LIST");
        else {
          const QJsonArray a = item.value("value").toArray();
          for (const QJsonValue &bv : meta.value("array_bounds").toArray()) {
            const QJsonObject bound = bv.toObject();
            const int index = bound.value("index").toInt(-1);
            if (index < 0 || index >= a.size() || !a.at(index).isDouble()) { itemErrors.append(QStringLiteral("ARRAY_INDEX_INVALID ") + QString::number(index)); continue; }
            const double n = a.at(index).toDouble();
            if (!std::isfinite(n) || (bound.contains("hard_min") && n < bound.value("hard_min").toDouble()) ||
                (bound.contains("hard_max") && n > bound.value("hard_max").toDouble()))
              itemErrors.append(QStringLiteral("ARRAY_BOUND_INVALID index=") + QString::number(index));
          }
        }
      }
      QJsonArray dependencyResults;
      for (const QJsonValue &dv : meta.value("dependencies").toArray()) {
        const QJsonObject dep = dv.toObject();
        if (dep.contains("when_candidate") && !jsonRuntimeEquivalent(item.value("value"), dep.value("when_candidate"))) continue;
        const QString depIdentity = dep.value("identity").toString();
        const QString op = dep.value("operator").toString();
        QJsonValue depValue; QString depWhy;
        bool depOk = resolveIdentityValue(depIdentity, &depValue, &depWhy);
        const QJsonValue candidate = item.value("value");
        if (depOk) {
          if (op == QStringLiteral("eq")) depOk = jsonRuntimeEquivalent(depValue, dep.value("value"));
          else if (op == QStringLiteral("neq")) depOk = !jsonRuntimeEquivalent(depValue, dep.value("value"));
          else if (depValue.isDouble() && candidate.isDouble()) {
            const double a = depValue.toDouble(), c = candidate.toDouble();
            if (op == QStringLiteral("gt_candidate")) depOk = a > c;
            else if (op == QStringLiteral("gte_candidate")) depOk = a >= c;
            else if (op == QStringLiteral("lt_candidate")) depOk = a < c;
            else if (op == QStringLiteral("lte_candidate")) depOk = a <= c;
            else if (op == QStringLiteral("abs_gte_candidate")) depOk = std::abs(a) >= c;
            else if (op == QStringLiteral("lte_abs_candidate")) depOk = a <= std::abs(c);
            else if (op == QStringLiteral("gte_candidate_plus")) depOk = c >= a + dep.value("delta").toDouble();
            else if (op == QStringLiteral("lte_candidate_minus")) depOk = c <= a - dep.value("delta").toDouble();
            else depOk = false;
          } else depOk = false;
        }
        const QString depMessage = dep.value("message").toString(
          depWhy.isEmpty() ? QStringLiteral("DEPENDENCY_FAILED: ") + depIdentity : depWhy);
        dependencyResults.append(QJsonObject{{"identity", depIdentity}, {"operator", op}, {"actual", depValue}, {"pass", depOk}, {"message", depMessage}});
        if (!depOk) itemErrors.append(depMessage);
      }
      row["dependency_results"] = dependencyResults;
      QString status = QStringLiteral("CHANGE_VALID");
      if (row.contains("yaml_current") && jsonRuntimeEquivalent(row.value("yaml_current"), item.value("value"))) status = QStringLiteral("ALREADY_MATCHES_YAML");
      else if (!itemWarnings.isEmpty()) status = QStringLiteral("RECOMMENDED_RANGE_WARNING");
      else if (meta.value("apply_mode").toString() == QStringLiteral("startup_only")) status = QStringLiteral("STARTUP_ONLY");
      else if (meta.value("apply_mode").toString() == QStringLiteral("node_restart")) status = QStringLiteral("RESTART_REQUIRED");
      if (!itemErrors.isEmpty()) status = itemErrors.contains("METADATA_INCOMPLETE") ? QStringLiteral("METADATA_INCOMPLETE") : QStringLiteral("INVALID");
      row["status"] = status; row["errors"] = itemErrors; row["warnings"] = itemWarnings;
      for (const QJsonValue &e : itemErrors) errors.append(QJsonObject{{"identity", identity}, {"message", e}});
      for (const QJsonValue &w : itemWarnings) warnings.append(QJsonObject{{"identity", identity}, {"message", w}});
      checked.append(row);
    }
    const bool valid = errors.isEmpty();
    const QString signature = configDraftSignature(items);
    result["ok"] = valid; result["valid"] = valid; result["can_apply"] = valid && !bridge_->readOnly();
    result["message"] = valid ? QStringLiteral("Validate OK; YAML belum ditulis") : QStringLiteral("Validate gagal; perbaiki item invalid");
    result["items"] = checked; result["errors"] = errors; result["warnings"] = warnings;
    result["draft_signature"] = signature; result["config_revision"] = revisions.value("config_revision");
    result["guard_state"] = QJsonObject{{"stationary", stationary}, {"reason", stationaryReason}, {"read_only", bridge_->readOnly()}};
    if (valid && storeSnapshot) {
      purgeValidationSnapshots();
      const QString seed = signature + QString::number(nowMs(), 'f', 0) + QString::number(reinterpret_cast<quintptr>(this));
      const QString id = QString::fromLatin1(QCryptographicHash::hash(seed.toUtf8(), QCryptographicHash::Sha256).toHex().left(24));
      QJsonObject snapshot{{"validation_id", id}, {"draft_signature", signature}, {"items", checked},
                           {"raw_items", items}, {"created_at_ms", nowMs()}, {"expires_at_ms", nowMs() + 120000.0},
                           {"config_revision", revisions.value("config_revision")}};
      validationSnapshots_[id] = snapshot;
      result["validation_id"] = id; result["expires_at_ms"] = snapshot.value("expires_at_ms");
    }
    return result;
  }

  QJsonObject checkValidatedApply(const QString &validationId, const QJsonArray &items) {
    purgeValidationSnapshots();
    QJsonObject out{{"ok", false}, {"code", "VALIDATION_REQUIRED"}};
    if (validationId.isEmpty() || !validationSnapshots_.contains(validationId)) {
      out["message"] = "validation_id tidak dikenal atau sudah kedaluwarsa"; return out;
    }
    const QJsonObject snap = validationSnapshots_.value(validationId);
    if (configDraftSignature(items) != snap.value("draft_signature").toString()) {
      out["code"] = "VALIDATION_PAYLOAD_CHANGED"; out["message"] = "Payload berubah setelah Validate"; return out;
    }
    QJsonArray conflicts;
    const QJsonObject currentRevisions = configRevisionState();
    const QJsonObject fileRevisions = currentRevisions.value("files").toObject();
    if (currentRevisions.value("config_revision").toString() != snap.value("config_revision").toString()) {
      conflicts.append(QJsonObject{{"identity", "*"}, {"kind", "CONFIG_REVISION_CHANGED"},
        {"validated_revision", snap.value("config_revision")}, {"current_revision", currentRevisions.value("config_revision")}});
    }
    for (const QJsonValue &v : snap.value("items").toArray()) {
      const QJsonObject row = v.toObject();
      const QString fk = row.value("file_key").toString(), path = row.value("path").toString();
      const QString identity = row.value("identity").toString();
      QJsonValue current; QString why;
      if (!currentYamlValue(fk, path, &current, &why) || !jsonRuntimeEquivalent(current, row.value("yaml_current")) ||
          fileRevisions.value(fk).toString() != row.value("file_revision").toString()) {
        conflicts.append(QJsonObject{{"identity", identity}, {"kind", "YAML_CHANGED"}, {"validated_yaml", row.value("yaml_current")}, {"current_yaml", current}});
        continue;
      }
      const QJsonObject beforeRuntime = row.value("runtime").toObject();
      if (beforeRuntime.value("readback_capable").toBool(false)) {
        const QJsonObject nowRuntime = bridge_->configRuntimeState(fk, path);
        if (!nowRuntime.value("readback_capable").toBool(false) || runtimeActuals(nowRuntime) != runtimeActuals(beforeRuntime)) {
          conflicts.append(QJsonObject{{"identity", identity}, {"kind", "EXTERNAL_RUNTIME_CHANGE"},
                                       {"validated_runtime", runtimeActuals(beforeRuntime)}, {"current_runtime", runtimeActuals(nowRuntime)}});
        }
      }
      if (row.value("requires_stationary").toBool(false)) {
        QString stationaryReason;
        if (!bridge_->vehicleStationary(&stationaryReason))
          conflicts.append(QJsonObject{{"identity", identity}, {"kind", "STATIONARY_REQUIRED"}, {"reason", stationaryReason}});
      }
    }
    if (!conflicts.isEmpty()) {
      out["code"] = "CONFIG_CONFLICT"; out["message"] = "Config/runtime berubah setelah Validate; Apply diblokir"; out["conflicts"] = conflicts;
      return out;
    }
    out["ok"] = true; out["code"] = "VALIDATED"; out["snapshot"] = snap;
    return out;
  }


  WebRosBridge *bridge_{nullptr};
  QTcpServer server_;
  QTimer eventTimer_;
  QTimer bindRetryTimer_;
  QTimer recordingTimer_;
  QTimer hostTimer_;
  QProcess imuCalibrationProcess_;
  QString imuCalibrationLastOutput_;
  quint64 hostPrevTotal_{0};
  quint64 hostPrevIdle_{0};
  QHostAddress bindAddress_;
  QString bindAddressText_;
  quint16 bindPort_{0};
  QList<QPointer<QTcpSocket>> sseClients_;
  QString staticRoot_;
  quint64 sseSequence_{0};
  int heartbeatTicks_{0};
  bool recording_{false};
  QString recordingSubsystem_;
  QString recordingId_;
  QString recordingSourceExperimentId_;
  QString recordingLabel_;
  QString recordingSectionLabel_;
  QMap<QString, QString> recordingTuningConfig_;
  QStringList recordingPaths_;
  QString recordingCandidate_;
  QString recordingVariation_;
  QString recordingCondition_;
  QString recordingStartedIso_;
  qint64 recordingStartedMs_{0};
  double recordingRateHz_{5.0};
  QVector<QMap<QString, QString>> recordingRows_;
  QByteArray lastDownloadCsv_;
  QString lastDownloadName_;
  QJsonObject recordingTrialInputs_;
  QJsonObject recordingLiveSeries_;
  QJsonArray recordingGraphs_;
  QString lastXlsxPath_;
  QString lastXlsxName_;
  QStringList lastGraphPaths_;
  QMap<QString, QJsonObject> validationSnapshots_;
  QMap<QString, QJsonObject> configProposals_;

  QString navigationSharePath() const {
    try { return QString::fromStdString(ament_index_cpp::get_package_share_directory("navigation")); }
    catch (...) { return agvPath(QStringLiteral("install/navigation/share/navigation")); }
  }

  QJsonObject imuCalibrationStatus() const {
    QJsonObject out{{"running", imuCalibrationProcess_.state()!=QProcess::NotRunning},
                    {"process_state", static_cast<int>(imuCalibrationProcess_.state())},
                    {"last_output", imuCalibrationLastOutput_.right(1200)}};
    const QString statePath=agvPath(QStringLiteral("calibration/yahboom_calibration_state.json"));
    QFile sf(statePath);
    if (sf.open(QIODevice::ReadOnly)) {
      QJsonParseError e{}; const auto d=QJsonDocument::fromJson(sf.readAll(),&e);
      if (e.error==QJsonParseError::NoError && d.isObject()) {
        for (auto it=d.object().constBegin(); it!=d.object().constEnd(); ++it) out[it.key()]=it.value();
      }
    }
    const QString fitPath=agvPath(QStringLiteral("calibration/yahboom_mag_planar_latest.yaml"));
    if (QFileInfo(fitPath).isFile()) {
      try { out["fit"] = yamlToJson(YAML::LoadFile(fitPath.toStdString())); }
      catch (const std::exception &e) { out["fit_error"]=QString::fromUtf8(e.what()); }
    }
    return out;
  }

  bool startImuCalibration(const QString &direction, QString *message) {
    if (bridge_->readOnly()) { if(message)*message="Web GUI read-only"; return false; }
    if (imuCalibrationProcess_.state()!=QProcess::NotRunning) { if(message)*message="Kalibrasi Yahboom masih berjalan"; return false; }
    QString stationary;
    if (!bridge_->vehicleStationary(&stationary)) { if(message)*message=stationary; return false; }
    const QString d=direction.trimmed().toUpper();
    if (d!="CW" && d!="CCW") { if(message)*message="direction wajib CW atau CCW"; return false; }
    const QString script=navigationSharePath()+QStringLiteral("/tools/yahboom_8dir_calibration.py");
    if (!QFileInfo(script).isFile()) { if(message)*message="Backend calibration script tidak ditemukan: "+script; return false; }
    imuCalibrationLastOutput_.clear();
    imuCalibrationProcess_.setProgram(agvPythonPath());
    imuCalibrationProcess_.setArguments({script,QStringLiteral("--direction"),d});
    imuCalibrationProcess_.setWorkingDirectory(agvRootPath());
    imuCalibrationProcess_.start();
    if (!imuCalibrationProcess_.waitForStarted(1200)) { if(message)*message="Gagal start backend kalibrasi"; return false; }
    if(message)*message=QStringLiteral("Wizard Yahboom 8 arah %1 dimulai; ikuti target animasi dan tahan diam tiap posisi.").arg(d);
    return true;
  }

  bool stopImuCalibration(QString *message) {
    if (imuCalibrationProcess_.state()==QProcess::NotRunning) { if(message)*message="Tidak ada kalibrasi aktif"; return false; }
    imuCalibrationProcess_.terminate();
    if (!imuCalibrationProcess_.waitForFinished(1200)) { imuCalibrationProcess_.kill(); imuCalibrationProcess_.waitForFinished(700); }
    const QString path=agvPath(QStringLiteral("calibration/yahboom_calibration_state.json"));
    QJsonObject st=imuCalibrationStatus(); st["status"]="STOPPED"; st["instruction"]="Dihentikan operator; hasil tidak boleh di-apply"; st["running"]=false;
    QSaveFile f(path); if(f.open(QIODevice::WriteOnly)){f.write(QJsonDocument(st).toJson(QJsonDocument::Indented));f.commit();}
    if(message)*message="Kalibrasi Yahboom dihentikan; gate apply tetap tertutup";
    return true;
  }

  bool proposeImuCalibration(QString *message, QJsonObject *result) {
    if (imuCalibrationProcess_.state()!=QProcess::NotRunning) { if(message)*message="Tunggu kalibrasi selesai"; return false; }
    const QString script=navigationSharePath()+QStringLiteral("/tools/yahboom_apply_calibration.py");
    QProcess proc; proc.setProgram(agvPythonPath());
    proc.setArguments({script,QStringLiteral("--workspace"),agvRootPath(),QStringLiteral("--propose")}); proc.start();
    if(!proc.waitForStarted(1000) || !proc.waitForFinished(6000)){proc.kill();if(message)*message="Calibration proposal backend timeout";return false;}
    QJsonParseError pe{}; const QJsonDocument doc=QJsonDocument::fromJson(proc.readAllStandardOutput().trimmed(),&pe);
    if(pe.error!=QJsonParseError::NoError || !doc.isObject()){if(message)*message="Calibration proposal backend menghasilkan response invalid";return false;}
    QJsonObject r=doc.object();
    if(proc.exitCode()!=0 || !r.value("ok").toBool(false)){if(result)*result=r;if(message)*message=r.value("message").toString("Fit tidak lolos gate");return false;}
    const QJsonArray proposalItems=r.value("proposal_items").toArray();
    if(proposalItems.isEmpty()){if(message)*message="Calibration proposal kosong";return false;}
    const QJsonObject proposal=registerConfigProposal(QStringLiteral("imu:yahboom"),proposalItems,r.value("evidence").toObject());
    r["proposal"]=proposal;r["proposal_only"]=true;r["runtime_write"]=false;r["yaml_write"]=false;
    if (result) *result = r;
    if (message) *message = "Yahboom PASS → server proposal dibuat; review Diff sebelum config transaction";
    return true;
  }

  void acceptConnections() {
    while (server_.hasPendingConnections()) {
      QTcpSocket *socket = server_.nextPendingConnection();
      socket->setProperty("request_buffer", QByteArray());
      connect(socket, &QTcpSocket::readyRead, this, [this, socket]() { readRequest(socket); });
      connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
    }
  }

  void readRequest(QTcpSocket *socket) {
    if (!socket || socket->property("sse").toBool()) return;
    QByteArray buffer = socket->property("request_buffer").toByteArray();
    buffer += socket->readAll();
    const int headerEnd = buffer.indexOf("\r\n\r\n");
    if (headerEnd < 0) {
      if (buffer.size() > 64 * 1024) {
        sendJson(socket, 413, QJsonObject{{"ok", false}, {"message", "Request headers too large"}});
      } else {
        socket->setProperty("request_buffer", buffer);
      }
      return;
    }
    if (headerEnd > 64 * 1024) {
      return sendJson(socket, 413, QJsonObject{{"ok", false}, {"message", "Request headers too large"}});
    }
    const QByteArray headersPart = buffer.left(headerEnd);
    const QList<QByteArray> lines = headersPart.split('\n');
    if (lines.isEmpty()) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Bad request"}});
    const QList<QByteArray> first = lines.first().trimmed().split(' ');
    if (first.size() < 2) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Bad request line"}});
    Request request;
    request.method = first[0].trimmed();
    const QUrl url = QUrl::fromEncoded(first[1]);
    request.path = url.path();
    for (int i = 1; i < lines.size(); ++i) {
      const QByteArray line = lines[i].trimmed();
      const int colon = line.indexOf(':');
      if (colon > 0) request.headers[line.left(colon).trimmed().toLower()] = line.mid(colon + 1).trimmed();
    }
    bool ok = false;
    const int contentLength = request.headers.value("content-length", "0").toInt(&ok);
    const int bodyStart = headerEnd + 4;
    if (!ok || contentLength < 0 || contentLength > 8 * 1024 * 1024) {
      return sendJson(socket, 413, QJsonObject{{"ok", false}, {"message", "Request body too large or invalid"}});
    }
    if (contentLength > 0 && buffer.size() < bodyStart + contentLength) {
      socket->setProperty("request_buffer", buffer);
      return;
    }
    if (contentLength > 0) request.body = buffer.mid(bodyStart, contentLength);
    handle(socket, request);
  }

  void handle(QTcpSocket *socket, const Request &request) {
    if (request.method == "GET" && request.path == "/api/events") return openSse(socket);
    if (request.method == "GET" && request.path == "/api/state") return sendJson(socket, 200, bridge_->snapshot());
    if (request.method == "GET" && request.path == "/api/health") {
      return sendJson(socket, 200, QJsonObject{{"ok", true}, {"ros", true}, {"read_only", bridge_->readOnly()},
                                               {"server_time_ms", nowMs()}});
    }
    if (request.method == "GET" && request.path == "/api/experiments") return sendJson(socket, 200, experimentCatalogJson());
    if (request.method == "GET" && request.path == "/api/config") return sendJson(socket, 200, loadConfigSnapshot());
    if (request.method == "GET" && request.path == "/api/config/state") {
      const QJsonObject snap = loadConfigSnapshot();
      const QJsonObject revisions = configRevisionState();
      return sendJson(socket, 200, QJsonObject{{"ok", true}, {"read_only", bridge_->readOnly()},
        {"model", "BASELINE/YAML/DRAFT/RUNTIME"}, {"files", snap.value("files")},
        {"file_revisions", revisions.value("files")}, {"config_revision", revisions.value("config_revision")},
        {"snapshot_at_ms", revisions.value("snapshot_at_ms")},
        {"runtime_config_apply", bridge_->snapshot().value("runtime_config_apply")}, {"at_ms", nowMs()}});
    }
    if (request.method == "GET" && request.path == "/api/config/schema") {
      const QJsonObject meta = loadUiParameterMetadata();
      return sendJson(socket, 200, QJsonObject{{"ok", !meta.contains("error")}, {"version", 2},
        {"parameters", meta.value("parameters")}, {"policy", meta.value("policy")}, {"metadata_path", meta.value("path")},
        {"capabilities", QJsonObject{{"validate", true}, {"validation_token", true}, {"optimistic_concurrency", true},
          {"atomic_batch_apply", true}, {"transaction_backup", true}, {"rollback_on_failure", true},
          {"runtime_readback", true}, {"revert", true}}}, {"at_ms", nowMs()}});
    }
    if (request.method == "GET" && request.path == "/api/imu/calibration/status") return sendJson(socket, 200, imuCalibrationStatus());
    if (request.method == "GET" && request.path == "/api/experiment/record/status") {
      return sendJson(socket, 200, recordingStatus());
    }
    if (request.method == "GET" && request.path == "/api/experiment/trials") {
      return sendJson(socket, 200, loadTrialStore());
    }
    if (request.method == "GET" && request.path == "/api/commissioning/state") return sendJson(socket,200,commissioningState());
    if (request.method == "GET" && request.path == "/api/camera.jpg") {
      const QByteArray bytes = bridge_->cameraJpeg();
      if (bytes.isEmpty()) return sendText(socket, 503, "text/plain; charset=utf-8", "Camera frame belum tersedia");
      return sendBytes(socket, 200, "image/jpeg", bytes, {{"Cache-Control", "no-store, max-age=0"}});
    }
    if (request.method == "GET" && request.path == "/api/map.png") {
      const QByteArray bytes = bridge_->mapPng();
      if (bytes.isEmpty()) return sendText(socket, 503, "text/plain; charset=utf-8", "Map belum tersedia");
      return sendBytes(socket, 200, "image/png", bytes, {{"Cache-Control", "no-store, max-age=0"}});
    }
    if (request.method == "GET" && request.path == "/api/global_costmap.png") {
      const QByteArray bytes = bridge_->globalCostmapPng();
      if (bytes.isEmpty()) return sendText(socket, 503, "text/plain; charset=utf-8", "Global costmap belum aktif");
      return sendBytes(socket, 200, "image/png", bytes, {{"Cache-Control", "no-store, max-age=0"}});
    }
    if (request.method == "GET" && request.path == "/api/local_costmap.png") {
      const QByteArray bytes = bridge_->localCostmapPng();
      if (bytes.isEmpty()) return sendText(socket, 503, "text/plain; charset=utf-8", "Local costmap belum aktif");
      return sendBytes(socket, 200, "image/png", bytes, {{"Cache-Control", "no-store, max-age=0"}});
    }
    if (request.method == "GET" && request.path == "/api/experiment/record/last.csv") {
      if (lastDownloadCsv_.isEmpty()) return sendText(socket, 404, "text/plain; charset=utf-8", "Belum ada CSV hasil Stop pada sesi server ini");
      const QByteArray disposition = QByteArray("attachment; filename=\"") + lastDownloadName_.toUtf8() + "\"";
      return sendBytes(socket, 200, "text/csv; charset=utf-8", lastDownloadCsv_,
                       {{"Cache-Control", "no-store"}, {"Content-Disposition", disposition}});
    }
    if (request.method == "GET" && request.path == "/api/experiment/record/last.xlsx") {
      QFile f(lastXlsxPath_); if(lastXlsxPath_.isEmpty()||!f.open(QIODevice::ReadOnly)) return sendText(socket,404,"text/plain; charset=utf-8","Belum ada XLSX hasil Stop");
      const QByteArray disposition=QByteArray("attachment; filename=\"")+lastXlsxName_.toUtf8()+"\"";
      return sendBytes(socket,200,"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",f.readAll(),{{"Cache-Control","no-store"},{"Content-Disposition",disposition}});
    }
    if (request.method == "GET" && request.path.startsWith("/api/experiment/record/last-graph-") && request.path.endsWith(".png")) {
      const QString mid=request.path.mid(QString("/api/experiment/record/last-graph-").size()); bool okIndex=false;const int idx=mid.left(mid.size()-4).toInt(&okIndex)-1;
      if(!okIndex||idx<0||idx>=lastGraphPaths_.size())return sendText(socket,404,"text/plain; charset=utf-8","Graph PNG tidak ditemukan");
      QFile f(lastGraphPaths_[idx]);if(!f.open(QIODevice::ReadOnly))return sendText(socket,404,"text/plain; charset=utf-8","Graph PNG tidak ditemukan");
      const QByteArray disposition=QByteArray("attachment; filename=\"")+QFileInfo(lastGraphPaths_[idx]).fileName().toUtf8()+"\"";
      return sendBytes(socket,200,"image/png",f.readAll(),{{"Cache-Control","no-store"},{"Content-Disposition",disposition}});
    }
    if (request.method == "POST") return handlePost(socket, request);
    if (request.method == "GET") return serveStatic(socket, request.path);
    sendJson(socket, 405, QJsonObject{{"ok", false}, {"message", "Method not allowed"}});
  }

  void handlePost(QTcpSocket *socket, const Request &request) {
    QJsonParseError error{};
    const QJsonDocument doc = QJsonDocument::fromJson(request.body, &error);
    if (!request.body.isEmpty() && (error.error != QJsonParseError::NoError || !doc.isObject())) {
      return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "JSON body tidak valid"}});
    }
    const QJsonObject json = doc.isObject() ? doc.object() : QJsonObject();
    QString message;
    bool ok = false;
    if (request.path == "/api/config/proposal") {
      const QString sourceTask=json.value("source_task").toString();const QJsonArray proposalItems=json.value("items").toArray();
      const QMap<QString,QSet<QString>> allowedPaths{
        {QStringLiteral("perception:homography"),QSet<QString>{QStringLiteral("perception.ros__parameters.ground_src_points"),QStringLiteral("perception.ros__parameters.ground_dst_points")}},
        {QStringLiteral("perception:obstacle-distance"),QSet<QString>{QStringLiteral("perception.ros__parameters.obstacle_distance_calibration_coefficients")}},
        {QStringLiteral("perception:lane-roi"),QSet<QString>{QStringLiteral("perception.ros__parameters.lane_safety_enabled"),QStringLiteral("perception.ros__parameters.lane_corridor_correction_gain_m_per_px"),QStringLiteral("perception.ros__parameters.lane_corridor_top_y_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_bottom_y_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_left_top_x_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_left_bottom_x_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_right_top_x_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_right_bottom_x_ratio"),QStringLiteral("perception.ros__parameters.nav2_obstacle_roi_enabled"),QStringLiteral("perception.ros__parameters.nav2_obstacle_roi_points")}}
      };
      if(!allowedPaths.contains(sourceTask)||proposalItems.isEmpty()||proposalItems.size()>32)return sendJson(socket,400,QJsonObject{{"ok",false},{"message","proposal source/items invalid"}});
      const QSet<QString> sourceAllowed=allowedPaths.value(sourceTask);
      for(const QJsonValue &v:proposalItems){const QJsonObject x=v.toObject();const QString path=x.value("path").toString();if(x.value("file_key").toString()!=QStringLiteral("perception")||!sourceAllowed.contains(path))return sendJson(socket,400,QJsonObject{{"ok",false},{"message","proposal identity tidak diizinkan untuk source_task"},{"source_task",sourceTask},{"path",path}});}
      const QJsonObject proposal=registerConfigProposal(sourceTask,proposalItems,json.value("evidence").toObject());
      return sendJson(socket,200,QJsonObject{{"ok",true},{"message","Generated proposal registered; no config write"},{"proposal",proposal},{"at_ms",nowMs()}});
    } else if (request.path == "/api/commissioning/qualification") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"code","READ_ONLY"},{"message","Read-only; qualification persistence ditolak"}});
      QJsonObject result;ok=setQualification(json,&message,&result);return sendJson(socket,ok?200:409,QJsonObject{{"ok",ok},{"message",message},{"qualification",result},{"commissioning",commissioningState()},{"at_ms",nowMs()}});
    } else if (request.path == "/api/perception/evidence") {
      QJsonObject result;
      ok = savePerceptionEvidence(json.value("label").toString(), &result, &message);
      return sendJson(socket, ok ? 200 : 409, QJsonObject{{"ok", ok}, {"message", message}, {"evidence", result}, {"at_ms", nowMs()}});
    } else if (request.path == "/api/config/validate") {
      const QJsonArray items = json.value("items").toArray();
      const QJsonObject result = validateConfigBatch(items, true);
      const int status = result.value("valid").toBool(false) ? 200 : 409;
      return sendJson(socket, status, result);
    } else if (request.path == "/api/config/apply" || request.path == "/api/config/revert") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"code","READ_ONLY"},{"message","Web GUI read-only; transactional apply ditolak"}});
      const bool useBaseline = request.path.endsWith("/revert");
      QJsonArray items = json.value("items").toArray();
      if (items.isEmpty() || items.size() > 64) return sendJson(socket,400,QJsonObject{{"ok",false},{"message","items harus berisi 1..64 parameter"}});
      if (useBaseline) {
        QJsonArray baselineItems;
        for (const QJsonValue &v : items) {
          const QJsonObject i = v.toObject(); QJsonValue baseline; QString why;
          if (!baselineYamlValue(i.value("file_key").toString(), i.value("path").toString(), &baseline, &why))
            return sendJson(socket,409,QJsonObject{{"ok",false},{"message",why}});
          QJsonObject b = i; b["value"] = baseline; baselineItems.append(b);
        }
        items = baselineItems;
      }
      const QString validationId = json.value("validation_id").toString();
      const QJsonObject gate = checkValidatedApply(validationId, items);
      if (!gate.value("ok").toBool(false)) return sendJson(socket,409,gate);
      const QMap<QString,QString> candidates = configCandidates();
      QSet<QString> seen; QMap<QString,QString> sourceFiles; QJsonArray resolved;
      for (const QJsonValue &v : items) {
        const QJsonObject item=v.toObject(); const QString fk=item.value("file_key").toString(),yp=item.value("path").toString(),id=fk+":"+yp;
        if (!candidates.contains(fk) || yp.isEmpty() || seen.contains(id)) return sendJson(socket,400,QJsonObject{{"ok",false},{"message","file_key/path tidak valid atau duplikat"}});
        seen.insert(id); QJsonValue current; QString why;
        if(!currentYamlValue(fk,yp,&current,&why)) return sendJson(socket,409,QJsonObject{{"ok",false},{"message",why},{"file_key",fk},{"path",yp}});
        const QJsonValue target=item.value("value");
        if(target.isUndefined()||!compatibleConfigType(current,target)) return sendJson(socket,409,QJsonObject{{"ok",false},{"message","Tipe target tidak kompatibel dengan YAML"},{"file_key",fk},{"path",yp}});
        sourceFiles[fk]=candidates.value(fk); resolved.append(QJsonObject{{"file_key",fk},{"path",yp},{"value",target},{"old_value",current}});
      }
      const QString transactionId = QString::fromLatin1(QCryptographicHash::hash((validationId+QString::number(nowMs(),'f',0)).toUtf8(),QCryptographicHash::Sha256).toHex().left(20));
      const QString stamp=QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
      QMap<QString,QString> txBackups; QJsonObject backupJson;
      for(auto it=sourceFiles.cbegin();it!=sourceFiles.cend();++it){
        const QString backup=it.value()+QStringLiteral(".web.txn.bak.")+stamp;
        if(!QFile::copy(it.value(),backup))return sendJson(socket,409,QJsonObject{{"ok",false},{"transaction_id",transactionId},{"message",QStringLiteral("Gagal membuat transaction backup: ")+it.value()}});
        txBackups[it.key()]=backup; backupJson[it.key()]=backup;
      }
      auto restoreFiles=[&](){bool rb=true;for(auto it=sourceFiles.cbegin();it!=sourceFiles.cend();++it){const QString b=txBackups.value(it.key());if(b.isEmpty()||!QFileInfo::exists(b)){rb=false;continue;}QFile::remove(it.value());if(!QFile::copy(b,it.value()))rb=false;}return rb;};
      QJsonArray saved; QString saveError; bool allSaved=true;
      for(const QJsonValue &v:resolved){
        const QJsonObject item=v.toObject(); QJsonValue actual; QString one;
        if(!setYamlValueAtomic(item.value("file_key").toString(),item.value("path").toString(),item.value("value"),&one,&actual)){allSaved=false;saveError=one;break;}
        saved.append(QJsonObject{{"identity",item.value("file_key").toString()+":"+item.value("path").toString()},
          {"file_key",item.value("file_key")},{"path",item.value("path")},{"saved_value",actual}});
      }
      if(!allSaved){const bool rb=restoreFiles();validationSnapshots_.remove(validationId);return sendJson(socket,409,QJsonObject{{"ok",false},{"transaction_id",transactionId},{"message",saveError},{"rolled_back",rb},{"batch_backups",backupJson},{"saved_before_failure",saved}});}
      QJsonArray runtimeChanges; for(const QJsonValue &v:resolved){const QJsonObject i=v.toObject();runtimeChanges.append(QJsonObject{{"file_key",i.value("file_key")},{"path",i.value("path")}});}
      const QJsonObject runtimeApply=bridge_->applyConfigChanges(runtimeChanges);
      const QString status=runtimeApply.value("status").toString();
      const bool match=runtimeApply.value("runtime_match").toBool(false);
      if(status=="RUNTIME_MISMATCH"){
        const bool rb=restoreFiles(); QJsonArray rollbackChanges;
        for(const QJsonValue &v:resolved){const QJsonObject i=v.toObject();rollbackChanges.append(QJsonObject{{"file_key",i.value("file_key")},{"path",i.value("path")}});}
        const QJsonObject rollbackRuntime=bridge_->applyConfigChanges(rollbackChanges); validationSnapshots_.remove(validationId);
        return sendJson(socket,409,QJsonObject{{"ok",false},{"code","RUNTIME_MISMATCH"},{"transaction_id",transactionId},{"message","Runtime verification mismatch; YAML transaction di-rollback"},{"rolled_back",rb},{"runtime_apply",runtimeApply},{"rollback_runtime",rollbackRuntime},{"batch_backups",backupJson},{"config",loadConfigSnapshot()},{"at_ms",nowMs()}});
      }
      const QJsonObject metadata=loadUiParameterMetadata().value("parameters").toObject(); QJsonArray itemResults;
      for(const QJsonValue &v:saved){
        QJsonObject one=v.toObject(); const QString identity=one.value("identity").toString();
        const QJsonObject meta=metadata.value(identity).toObject(); const QJsonObject runtime=bridge_->configRuntimeState(one.value("file_key").toString(),one.value("path").toString());
        QString itemStatus;
        if(meta.value("apply_mode").toString()==QStringLiteral("startup_only") || !runtime.value("has_runtime_target").toBool(false)) itemStatus=QStringLiteral("NEXT_START");
        else if(runtime.value("readback_capable").toBool(false)){
          bool allMatch=true;for(const QJsonValue &rv:runtime.value("readback").toArray())allMatch=allMatch&&rv.toObject().value("match").toBool(false);
          itemStatus=allMatch?QStringLiteral("ACTIVE_MATCH"):status;
        }else itemStatus=status.isEmpty()?QStringLiteral("YAML_SAVED"):status;
        one["apply_status"]=itemStatus; one["runtime"]=runtime; one["apply_mode"]=meta.value("apply_mode"); itemResults.append(one);
      }
      validationSnapshots_.remove(validationId);
      const QJsonObject revisions=configRevisionState();
      return sendJson(socket,200,QJsonObject{{"ok",true},{"code","TRANSACTION_APPLIED"},{"transaction_id",transactionId},
        {"message",useBaseline?"Validated baseline transaction selesai":"Validated config transaction selesai"},
        {"items",itemResults},{"batch_backups",backupJson},{"runtime_apply",runtimeApply},{"runtime_match",match},
        {"config_revision",revisions.value("config_revision")},{"file_revisions",revisions.value("files")},
        {"config",loadConfigSnapshot()},{"at_ms",nowMs()}});
    } else if (request.path == "/api/config/set" || request.path == "/api/config/reset" || request.path == "/api/config/reset-batch") {
      return sendJson(socket,409,QJsonObject{{"ok",false},{"code","USE_CONFIG_TRANSACTION"},
        {"message","Legacy direct config write dinonaktifkan; gunakan Draft → Validate → Diff Review → /api/config/apply atau /api/config/revert"},{"at_ms",nowMs()}});
    } else if (request.path == "/api/experiment/record/start") {
      ok = startRecording(json, &message);
      return sendJson(socket, ok ? 200 : 409, QJsonObject{{"ok", ok}, {"message", message},
                       {"recording", recordingStatus()}, {"at_ms", nowMs()}});
    } else if (request.path == "/api/experiment/record/stop") {
      QJsonObject result;
      ok = stopRecording(json, &message, &result);
      result["ok"] = ok; result["message"] = message; result["at_ms"] = nowMs();
      return sendJson(socket, ok ? 200 : 409, result);
    } else if (request.path == "/api/experiment/table/save") {
      QJsonObject result;
      ok = saveTemplateTable(json, &message, &result);
      result["ok"] = ok; result["message"] = message; result["at_ms"] = nowMs();
      return sendJson(socket, ok ? 200 : 409, result);
    } else if (request.path == "/api/experiment/trial/delete") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"message","Web GUI read-only; hapus trial ditolak"}});
      QJsonObject result; ok=deleteTrial(json,&message,&result);result["ok"]=ok;result["message"]=message;result["at_ms"]=nowMs();
      return sendJson(socket,ok?200:409,result);
    } else if (request.path == "/api/experiment/trial/optimal-scale") {
      if (json.value("apply").toBool(false)) return sendJson(socket,409,QJsonObject{{"ok",false},{"code","USE_CONFIG_TRANSACTION"},{"message","Direct optimal-scale apply dinonaktifkan; gunakan proposal_items → Validate → Diff Review → Apply"}});
      QJsonObject result;ok=calculateOptimalScaleProposal(&message,&result);
      if(ok){const double scale=result.value("scale").toDouble();const QString iso=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);QJsonArray proposalItems;
        proposalItems.append(QJsonObject{{"file_key","esc"},{"path","esc_ackermann.ros__parameters.drive_odometry_calibration_scale"},{"value",scale}});
        proposalItems.append(QJsonObject{{"file_key","vehicle"},{"path","vehicle.ros__parameters.drive_odometry_calibration_scale"},{"value",scale},{"generated",true}});
        proposalItems.append(QJsonObject{{"file_key","vehicle"},{"path","vehicle.ros__parameters.drive_odometry_calibration_valid"},{"value",true},{"generated",true}});
        proposalItems.append(QJsonObject{{"file_key","vehicle"},{"path","vehicle.ros__parameters.drive_odometry_calibration_saved_at"},{"value",iso},{"generated",true}});
        const QJsonObject evidence{{"accepted_trials",result.value("accepted_trials")},{"valid_trials",result.value("valid_trials")},{"rejected_trial_ids",result.value("rejected_trial_ids")}};
        const QJsonObject proposal=registerConfigProposal(QStringLiteral("navigation:N2.1"),proposalItems,evidence);
        result["proposal_items"]=proposalItems;result["proposal"]=proposal;result["proposal_only"]=true;}
      result["ok"]=ok;result["message"]=message;result["at_ms"]=nowMs();
      return sendJson(socket,ok?200:409,result);
    } else if (request.path == "/api/experiment/trial/motion") {
      const bool active = json.value("active").toBool(false);
      const QString subsystem = json.value("subsystem").toString();
      if (active && subsystem == QStringLiteral("navigation") && !taskQualified(QStringLiteral("steering"), QStringLiteral("4.9"))) {
        return sendJson(socket,409,QJsonObject{{"ok",false},{"message","Commissioning gate: ESC 4.9 final evidence belum tersedia"},{"at_ms",nowMs()}});
      }
      if (active && subsystem == QStringLiteral("perception")) {
        const bool esc_ready = taskQualified(QStringLiteral("steering"), QStringLiteral("4.9"));
        const bool nav_ready = taskQualified(QStringLiteral("navigation"), QStringLiteral("N16.1")) || taskQualified(QStringLiteral("navigation"), QStringLiteral("N17.1"));
        if (!esc_ready || !nav_ready) return sendJson(socket,409,QJsonObject{{"ok",false},{"message","Commissioning gate belum memenuhi ESC → Navigasi → Persepsi"},{"at_ms",nowMs()}});
      }
      ok=bridge_->publishTrialMotion(json.value("id").toString(), json.value("erpm").toDouble(0.0), json.value("steering_deg").toDouble(0.0), active, &message);
      return sendJson(socket,ok?200:409,QJsonObject{{"ok",ok},{"message",message},{"at_ms",nowMs()}});
    } else if (request.path == "/api/imu/profile/optimal") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"message","Web GUI read-only"}});
      QString stationary; if(!bridge_->vehicleStationary(&stationary)) return sendJson(socket,409,QJsonObject{{"ok",false},{"message",stationary}});
      ok=bridge_->triggerService("/imu/configure_optimal_profile","imu_optimal_profile",&message);
    } else if (request.path == "/api/imu/calibration/start") {
      ok=startImuCalibration(json.value("direction").toString(),&message);
    } else if (request.path == "/api/imu/calibration/stop") {
      ok=stopImuCalibration(&message);
    } else if (request.path == "/api/imu/calibration/proposal") {
      QJsonObject result;ok=proposeImuCalibration(&message,&result);result["ok"]=ok;result["message"]=message;result["at_ms"]=nowMs();return sendJson(socket,ok?200:409,result);
    } else if (request.path == "/api/imu/calibration/apply") {
      return sendJson(socket,409,QJsonObject{{"ok",false},{"code","USE_CONFIG_TRANSACTION"},{"message","Direct IMU apply dinonaktifkan; gunakan /api/imu/calibration/proposal → Validate → Diff Review → Apply"},{"at_ms",nowMs()}});
    } else if (request.path == "/api/perception/inference") {
      ok = bridge_->setPerceptionInference(json.value("enabled").toBool(false), &message);
    } else if (request.path == "/api/navigation/goal") {
      ok = bridge_->publishGoal(json.value("x").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                json.value("y").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                json.contains("yaw_rad") ? json.value("yaw_rad").toDouble() : json.value("yaw_deg").toDouble() * kPi / 180.0,
                                &message);
    } else if (request.path == "/api/navigation/cancel") {
      ok = bridge_->cancelNavigation(&message);
    } else if (request.path == "/api/localization/initial-pose") {
      ok = bridge_->publishInitialPose(json.value("x").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                       json.value("y").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                       json.contains("yaw_rad") ? json.value("yaw_rad").toDouble() : json.value("yaw_deg").toDouble() * kPi / 180.0,
                                       &message);
    } else if (request.path == "/api/hmi/page") {
      const QString page = json.value("page").toString().trimmed().toUpper();
      static const QSet<QString> allowed{QStringLiteral("OVERVIEW"), QStringLiteral("ESC"), QStringLiteral("PERCEPTION"), QStringLiteral("NAVIGATION")};
      if (!allowed.contains(page)) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Page HMI tidak valid"}});
      ok = bridge_->publishHmiRequest("PAGE:" + page, &message);
    } else if (request.path == "/api/hmi/waypoint/select") {
      const int index = json.value("index").toInt(-1);
      if (index < 0 || index > 3) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Waypoint index wajib 0..3"}});
      ok = bridge_->publishHmiRequest(QString("WAYPOINT:SELECT:%1").arg(index), &message);
    } else if (request.path == "/api/hmi/waypoint/save") {
      const int index = json.value("index").toInt(-1);
      QString name = json.value("name").toString().trimmed().left(18);
      name.remove(QRegularExpression(QStringLiteral("[^A-Za-z0-9 _-]")));
      if (index < 0 || index > 3) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Waypoint index wajib 0..3"}});
      const QString command = name.isEmpty() ? QString("WAYPOINT:SAVE:%1").arg(index) : QString("WAYPOINT:SAVE:%1:%2").arg(index).arg(name);
      ok = bridge_->publishHmiRequest(command, &message);
    } else if (request.path == "/api/hmi/waypoint/go") {
      const int index = json.value("index").toInt(-1);
      if (index < 0 || index > 3) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Waypoint index wajib 0..3"}});
      ok = bridge_->publishHmiRequest(QString("WAYPOINT:GO:%1").arg(index), &message);
    } else if (request.path == "/api/hmi/navigation/stop") {
      ok = bridge_->publishHmiRequest("NAV:STOP", &message);
    } else if (request.path == "/api/hmi/mode") {
      const QString mode = json.value("mode").toString().trimmed().toUpper();
      if (mode != "AUTO" && mode != "MANUAL") return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Mode HMI wajib AUTO/MANUAL"}});
      ok = bridge_->publishHmiRequest("MODE:" + mode, &message);
    } else if (request.path == "/api/hmi/control") {
      const QString action = json.value("action").toString().trimmed().toUpper();
      static const QSet<QString> drive{QStringLiteral("FWD"), QStringLiteral("REV"), QStringLiteral("STOP")};
      static const QSet<QString> steer{QStringLiteral("LEFT"), QStringLiteral("CENTER"), QStringLiteral("RIGHT")};
      if (drive.contains(action)) ok = bridge_->publishHmiRequest("DRIVE:" + action, &message);
      else if (steer.contains(action)) ok = bridge_->publishHmiRequest("STEER:" + action, &message);
      else return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Control HMI tidak valid"}});
    } else if (request.path == "/api/hmi/speed") {
      const int pct = json.value("pct").toInt(-1);
      if (pct < 10 || pct > 50 || pct % 10 != 0) return sendJson(socket, 400, QJsonObject{{"ok", false}, {"message", "Speed HMI wajib 10/20/30/40/50%"}});
      ok = bridge_->publishHmiRequest(QString("SPEED:%1").arg(pct), &message);
    } else if (request.path == "/api/hmi/sync") {
      ok = bridge_->publishHmiRequest("SYNC", &message);
    } else if (request.path == "/api/esc/vesc/command") {
      ok = bridge_->publishVescToolCommand(json.value("command").toString(), &message);
    } else if (request.path == "/api/localization/reset-calibration") {
      ok = bridge_->triggerService("/localization/reset_calibration_samples", "reset_localization_calibration", &message);
    } else if (request.path == "/api/steering/calibration-mode") {
      ok = bridge_->setSteeringCalibrationMode(json.value("enabled").toBool(false), &message);
    } else {
      return sendJson(socket, 404, QJsonObject{{"ok", false}, {"message", "Unknown API endpoint"}});
    }
    sendJson(socket, ok ? 200 : 409, QJsonObject{{"ok", ok}, {"message", message}, {"at_ms", nowMs()}});
  }

  static QString sha256File(const QString &path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    while (!f.atEnd()) hash.addData(f.read(1024 * 1024));
    return QString::fromLatin1(hash.result().toHex());
  }

  static QString gitRevision() {
    QProcess p;
    p.setWorkingDirectory(agvRootPath());
    p.start(QStringLiteral("git"), {QStringLiteral("rev-parse"), QStringLiteral("--short=12"), QStringLiteral("HEAD")});
    if (!p.waitForFinished(1200) || p.exitStatus() != QProcess::NormalExit || p.exitCode() != 0) return QStringLiteral("unknown");
    const QString out = QString::fromUtf8(p.readAllStandardOutput()).trimmed();
    return out.isEmpty() ? QStringLiteral("unknown") : out;
  }

  static QJsonObject configFingerprints() {
    const QJsonObject files = loadConfigSnapshot().value(QStringLiteral("files")).toObject();
    QJsonObject out;
    for (auto it = files.constBegin(); it != files.constEnd(); ++it) {
      const QString path = it.value().toObject().value(QStringLiteral("path")).toString();
      if (path.isEmpty() || !QFileInfo::exists(path)) continue;
      const QFileInfo fi(path);
      out[it.key()] = QJsonObject{{QStringLiteral("path"), path},
                                  {QStringLiteral("sha256"), sha256File(path)},
                                  {QStringLiteral("mtime_ms"), fi.lastModified().toMSecsSinceEpoch()},
                                  {QStringLiteral("bytes"), fi.size()}};
    }
    return out;
  }

  bool writeSessionManifest(const QString &csvPath, QJsonObject *result, QString *message) const {
    if (csvPath.isEmpty()) {
      if (message) *message = QStringLiteral("Primary CSV path kosong");
      return false;
    }
    QJsonObject runtime;
    if (bridge_) {
      const QJsonObject snap = bridge_->snapshot();
      for (const QString &key : {QStringLiteral("server"), QStringLiteral("connected"), QStringLiteral("system.motion_ready"),
                                 QStringLiteral("system.nav2_ready"), QStringLiteral("system.estop"), QStringLiteral("esc_mux"),
                                 QStringLiteral("nav_cmd_mux"), QStringLiteral("vesc_tool_status"), QStringLiteral("gnss_quality"),
                                 QStringLiteral("imu_status"), QStringLiteral("ekf_local_status"), QStringLiteral("ekf_global_status"),
                                 QStringLiteral("trajectory_safety_state"), QStringLiteral("collision_monitor_state")}) {
        if (snap.contains(key)) runtime[key] = snap.value(key);
      }
    }
    QJsonObject tuning;
    for (auto it = recordingTuningConfig_.cbegin(); it != recordingTuningConfig_.cend(); ++it) tuning[it.key()] = it.value();
    QJsonObject manifest{{QStringLiteral("schema"), QStringLiteral("adv-session-manifest-v1")},
                         {QStringLiteral("git_commit"), gitRevision()},
                         {QStringLiteral("generated_at"), QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},
                         {QStringLiteral("subsystem"), recordingSubsystem_},
                         {QStringLiteral("section_id"), recordingId_},
                         {QStringLiteral("source_experiment_id"), recordingSourceExperimentId_},
                         {QStringLiteral("label"), recordingLabel_},
                         {QStringLiteral("section_label"), recordingSectionLabel_},
                         {QStringLiteral("candidate"), recordingCandidate_},
                         {QStringLiteral("variation"), recordingVariation_},
                         {QStringLiteral("condition"), recordingCondition_},
                         {QStringLiteral("started_at"), recordingStartedIso_},
                         {QStringLiteral("stopped_at"), QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},
                         {QStringLiteral("sample_rate_hz"), recordingRateHz_},
                         {QStringLiteral("samples"), recordingRows_.size()},
                         {QStringLiteral("primary_csv"), csvPath},
                         {QStringLiteral("record_paths"), QJsonArray::fromStringList(recordingPaths_)},
                         {QStringLiteral("trial_inputs"), recordingTrialInputs_},
                         {QStringLiteral("live_series"), recordingLiveSeries_},
                         {QStringLiteral("graphs"), recordingGraphs_},
                         {QStringLiteral("tuning_snapshot"), tuning},
                         {QStringLiteral("config_fingerprints"), configFingerprints()},
                         {QStringLiteral("runtime_stop_snapshot"), runtime}};
    const QFileInfo csvInfo(csvPath);
    const QString manifestPath = QDir(csvInfo.absolutePath()).filePath(csvInfo.completeBaseName() + QStringLiteral(".manifest.json"));
    QSaveFile f(manifestPath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
      if (message) *message = QStringLiteral("Gagal membuka session manifest: ") + manifestPath;
      return false;
    }
    f.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented));
    if (!f.commit()) {
      if (message) *message = QStringLiteral("Gagal commit session manifest: ") + manifestPath;
      return false;
    }
    if (result) { (*result)[QStringLiteral("manifest_path")] = manifestPath; (*result)[QStringLiteral("manifest_saved")] = true; }
    if (message) *message = QStringLiteral("Manifest tersimpan: ") + manifestPath;
    return true;
  }

  bool savePerceptionEvidence(const QString &label, QJsonObject *result, QString *message) const {
    if (!bridge_) { if (message) *message = QStringLiteral("ROS bridge tidak tersedia"); return false; }
    const QByteArray jpeg = bridge_->cameraJpeg();
    if (jpeg.isEmpty()) { if (message) *message = QStringLiteral("Camera frame belum tersedia"); return false; }
    const QString root = agvPath(QStringLiteral("data/presepsi/evidence"));
    if (!QDir().mkpath(root)) { if (message) *message = QStringLiteral("Gagal membuat folder evidence"); return false; }
    QString safe = label.trimmed();
    safe.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_.-]+")), QStringLiteral("_"));
    if (safe.isEmpty()) safe = QStringLiteral("capture");
    const QString stem = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")) + QStringLiteral("_") + safe;
    const QString jpgPath = QDir(root).filePath(stem + QStringLiteral(".jpg"));
    const QString jsonPath = QDir(root).filePath(stem + QStringLiteral(".json"));
    QSaveFile jf(jpgPath);
    if (!jf.open(QIODevice::WriteOnly) || jf.write(jpeg) != jpeg.size() || !jf.commit()) {
      if (message) *message = QStringLiteral("Gagal menyimpan evidence JPEG");
      return false;
    }
    const QJsonObject snap = bridge_->snapshot();
    QJsonObject telemetry;
    for (const QString &key : {QStringLiteral("camera_frame"), QStringLiteral("camera_health_state"), QStringLiteral("camera_healthy"),
                               QStringLiteral("raw_detections"), QStringLiteral("obstacle_metrics"), QStringLiteral("lane_state"),
                               QStringLiteral("drivable_state"), QStringLiteral("object_points"), QStringLiteral("path_relevant_points"),
                               QStringLiteral("planning_relevant_points"), QStringLiteral("perception_performance"),
                               QStringLiteral("trajectory_safety_state"), QStringLiteral("perception_emergency"),
                               QStringLiteral("cmd_perception_advisory"), QStringLiteral("ekf_global")}) {
      if (snap.contains(key)) telemetry[key] = snap.value(key);
    }
    QJsonObject meta{{QStringLiteral("schema"), QStringLiteral("adv-perception-evidence-v1")},
                     {QStringLiteral("captured_at"), QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},
                     {QStringLiteral("label"), label}, {QStringLiteral("image"), jpgPath},
                     {QStringLiteral("git_commit"), gitRevision()}, {QStringLiteral("telemetry"), telemetry},
                     {QStringLiteral("config_fingerprints"), configFingerprints()}};
    QSaveFile mf(jsonPath);
    if (!mf.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QFile::remove(jpgPath);
      if (message) *message = QStringLiteral("Gagal membuka evidence JSON");
      return false;
    }
    mf.write(QJsonDocument(meta).toJson(QJsonDocument::Indented));
    if (!mf.commit()) {
      QFile::remove(jpgPath);
      if (message) *message = QStringLiteral("Gagal commit evidence JSON");
      return false;
    }
    if (result) *result = QJsonObject{{QStringLiteral("image_path"), jpgPath}, {QStringLiteral("metadata_path"), jsonPath}, {QStringLiteral("label"), label}};
    if (message) *message = QStringLiteral("Perception evidence tersimpan: ") + jpgPath;
    return true;
  }

  QJsonObject recordingStatus() const {
    return QJsonObject{{"active", recording_}, {"subsystem", recordingSubsystem_}, {"id", recordingId_},
                       {"source_experiment_id", recordingSourceExperimentId_},
                       {"label", recordingLabel_}, {"section_label", recordingSectionLabel_},
                       {"candidate", recordingCandidate_}, {"variation", recordingVariation_}, {"condition", recordingCondition_},
                       {"started_at", recordingStartedIso_}, {"sample_rate_hz", recordingRateHz_},
                       {"elapsed_s", recording_ && recordingStartedMs_ > 0 ? (nowMs() - recordingStartedMs_) / 1000.0 : 0.0},
                       {"samples", recordingRows_.size()}, {"record_paths", QJsonArray::fromStringList(recordingPaths_)},
                       {"report_root", agvPath(QStringLiteral("data"))}};
  }

  bool startRecording(const QJsonObject &json, QString *message) {
    if (recording_) {
      if (message) *message = QStringLiteral("Recording lain masih aktif; Stop CSV lebih dulu");
      return false;
    }
    const QString subsystem = json.value("subsystem").toString().trimmed();
    const QString id = json.value("id").toString().trimmed();
    if (!QStringList{QStringLiteral("navigation"), QStringLiteral("perception"), QStringLiteral("steering")}.contains(subsystem) || id.isEmpty()) {
      if (message) *message = QStringLiteral("subsystem/id recording tidak valid");
      return false;
    }
    if (subsystem == QStringLiteral("navigation") && !taskQualified(QStringLiteral("steering"), QStringLiteral("4.9"))) {
      if (message) *message = QStringLiteral("Commissioning gate: selesaikan ESC 4.9 Final Gate sebelum recording Navigasi");
      return false;
    }
    if (subsystem == QStringLiteral("perception")) {
      const bool esc_ready = taskQualified(QStringLiteral("steering"), QStringLiteral("4.9"));
      const bool nav_ready = taskQualified(QStringLiteral("navigation"), QStringLiteral("N16.1")) ||
                             taskQualified(QStringLiteral("navigation"), QStringLiteral("N17.1"));
      if (!esc_ready || !nav_ready) {
        if (message) *message = !esc_ready ?
          QStringLiteral("Commissioning gate: ESC 4.9 belum memiliki final evidence") :
          QStringLiteral("Commissioning gate: Navigasi N16.1/N17.1 belum memiliki final evidence");
        return false;
      }
    }
    recordingSubsystem_ = subsystem;
    recordingId_ = id;
    recordingSourceExperimentId_ = json.value("source_experiment_id").toString().trimmed();
    if (recordingSourceExperimentId_.isEmpty()) recordingSourceExperimentId_ = recordingId_;
    recordingLabel_ = json.value("label").toString().trimmed();
    if (recordingLabel_.isEmpty()) recordingLabel_ = recordingId_;
    recordingSectionLabel_ = json.value("section_label").toString().trimmed();
    if (recordingSectionLabel_.isEmpty()) recordingSectionLabel_ = recordingLabel_;
    recordingTuningConfig_.clear();
    recordingPaths_.clear();
    for (const QJsonValue &v : json.value("record_paths").toArray()) {
      const QString path = v.toString().trimmed();
      if (!path.isEmpty() && !recordingPaths_.contains(path)) recordingPaths_ << path;
    }
    const QJsonObject tuningConfig = json.value("tuning_config").toObject();
    for (auto it = tuningConfig.constBegin(); it != tuningConfig.constEnd(); ++it) {
      flattenJson(QStringLiteral("tuning_yaml.") + it.key(), it.value(), recordingTuningConfig_);
    }
    recordingTrialInputs_ = json.value("trial_inputs").toObject();
    recordingLiveSeries_ = json.value("live_series").toObject();
    recordingGraphs_ = json.value("graphs").toArray();
    recordingCandidate_ = json.value("candidate").toString().trimmed();
    if (recordingCandidate_.isEmpty()) recordingCandidate_ = QStringLiteral("baseline");
    recordingVariation_ = json.value("variation").toString().trimmed();
    recordingCondition_ = json.value("condition").toString().trimmed();
    recordingRateHz_ = std::clamp(json.value("sample_rate_hz").toDouble(5.0), 1.0, 20.0);
    recordingStartedIso_ = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    recordingStartedMs_ = nowMs();
    recordingRows_.clear();
    recording_ = true;
    recordingTimer_.start(std::max(50, static_cast<int>(std::lround(1000.0 / recordingRateHz_))));
    captureRecordingSample();
    if (message) *message = QStringLiteral("CSV recording dimulai: ") + subsystem + QStringLiteral("/") + id;
    return true;
  }

  void captureRecordingSample() {
    if (!recording_ || !bridge_) return;
    QMap<QString, QString> row;
    const qint64 sampleMs = nowMs();
    row["time_iso"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    row["elapsed_s"] = QString::number(recordingStartedMs_ > 0 ? (sampleMs - recordingStartedMs_) / 1000.0 : 0.0, 'f', 3);
    row["subsystem"] = recordingSubsystem_;
    row["section_id"] = recordingId_;
    row["source_experiment_id"] = recordingSourceExperimentId_;
    row["section_label"] = recordingSectionLabel_;
    row["candidate"] = recordingCandidate_;
    row["variation"] = recordingVariation_;
    row["condition"] = recordingCondition_;
    for (auto it = recordingTuningConfig_.cbegin(); it != recordingTuningConfig_.cend(); ++it) row[it.key()] = it.value();
    const QJsonObject snap = bridge_->snapshot();
    QMap<QString, QString> flatSnapshot;
    for (auto it = snap.constBegin(); it != snap.constEnd(); ++it) {
      if (it.key().startsWith(QStringLiteral("__"))) continue;
      flattenJson(it.key(), it.value(), flatSnapshot);
    }
    if (recordingPaths_.isEmpty()) {
      for (auto it = flatSnapshot.cbegin(); it != flatSnapshot.cend(); ++it) row[it.key()] = it.value();
    } else {
      for (const QString &path : recordingPaths_) {
        auto exact = flatSnapshot.constFind(path);
        if (exact != flatSnapshot.cend()) row[path] = exact.value();
        const QString prefix = path + QStringLiteral(".");
        for (auto it = flatSnapshot.cbegin(); it != flatSnapshot.cend(); ++it)
          if (it.key().startsWith(prefix)) row[it.key()] = it.value();
      }
    }
    recordingRows_.push_back(row);
    if (recordingRows_.size() > 250000) {
      recording_ = false;
      recordingTimer_.stop();
    }
  }

  bool saveTemplateTable(const QJsonObject &json, QString *message, QJsonObject *result) {
    const QString subsystem = json.value("subsystem").toString().trimmed();
    const QString id = json.value("id").toString().trimmed();
    const QString label = json.value("label").toString().trimmed();
    const int tableIndex = std::max(0, json.value("table_index").toInt(0));
    const QString csvText = json.value("csv").toString();
    if (!QStringList{QStringLiteral("navigation"), QStringLiteral("perception"), QStringLiteral("steering")}.contains(subsystem) ||
        id.isEmpty() || csvText.trimmed().isEmpty()) {
      if (message) *message = QStringLiteral("Template table payload tidak valid");
      return false;
    }
    const QString domain = subsystem == QStringLiteral("navigation") ? QStringLiteral("navigasi") :
                           subsystem == QStringLiteral("perception") ? QStringLiteral("presepsi") : QStringLiteral("esc");
    const QString dataRoot = QDir(agvPath(QStringLiteral("data"))).filePath(domain);
    if (!QDir().mkpath(dataRoot)) {
      if (message) *message = QStringLiteral("Gagal membuat folder data: ") + dataRoot;
      return false;
    }
    const QString stemLabel = id.isEmpty() ? label : id;
    const QString minuteStamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString baseStem = recordingCsvStem(stemLabel) + QStringLiteral("_T%1_").arg(tableIndex + 1) + minuteStamp;
    QString fileName = baseStem + QStringLiteral(".csv");
    int suffix = 2;
    while (QFileInfo::exists(QDir(dataRoot).filePath(fileName)))
      fileName = baseStem + QStringLiteral("_%1.csv").arg(suffix++, 2, 10, QChar('0'));
    const QString path = QDir(dataRoot).filePath(fileName);
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      if (message) *message = QStringLiteral("Gagal membuka template CSV: ") + path;
      return false;
    }
    file.write(csvText.toUtf8());
    if (!file.commit()) {
      if (message) *message = QStringLiteral("Gagal commit template CSV: ") + path;
      return false;
    }
    if (result) *result = QJsonObject{{"path", path}, {"table_index", tableIndex}, {"section_id", id}, {"subsystem", subsystem}};
    if (message) *message = QStringLiteral("Template table CSV tersimpan: ") + path;
    return true;
  }

  QString trialStorePath() const {
    return agvPath(QStringLiteral("data/experiment_trials.yaml"));
  }

  QString taskConfigFingerprint(const QString &subsystem, const QString &id) const {
    QSet<QString> keys;
    for (const ExperimentSpec &spec : buildExperimentCatalog(subsystem)) {
      if (spec.id != id) continue;
      for (const ExperimentParameterField &field : spec.parameterFields)
        if (!field.yamlFileKey.isEmpty()) keys.insert(field.yamlFileKey);
      break;
    }
    const bool finalGate = (subsystem == QStringLiteral("steering") && id == QStringLiteral("4.9")) ||
      (subsystem == QStringLiteral("navigation") && (id == QStringLiteral("N16.1") || id == QStringLiteral("N17.1"))) ||
      (subsystem == QStringLiteral("perception") && id == QStringLiteral("4.9"));
    if (finalGate || keys.isEmpty()) {
      if (subsystem == QStringLiteral("steering")) keys.unite(QSet<QString>{"esc","vehicle","foc_thesis","vesc_tool"});
      else if (subsystem == QStringLiteral("navigation")) keys.unite(QSet<QString>{"vehicle","navigation_core","nav2","ekf","localization","gnss","imu","mag_heading","imu_speed","trajectory_safety","collision","mppi_closed_loop","esc"});
      else if (subsystem == QStringLiteral("perception")) keys.unite(QSet<QString>{"perception","bbox_calibration","trajectory_safety","navigation_core"});
    }
    const QJsonObject revs = configRevisionState().value("files").toObject();
    QStringList rows;
    for (const QString &key : keys) if (revs.contains(key)) rows << key + QStringLiteral("=") + revs.value(key).toString();
    std::sort(rows.begin(), rows.end());
    return QString::fromLatin1(QCryptographicHash::hash(rows.join(QStringLiteral("\n")).toUtf8(), QCryptographicHash::Sha256).toHex());
  }

  QJsonObject qualificationRecord(const QString &subsystem, const QString &id) const {
    return loadTrialStore().value("qualifications").toObject().value(subsystem).toObject().value(id).toObject();
  }

  QJsonObject effectiveQualification(const QString &subsystem, const QString &id) const {
    QJsonObject q = qualificationRecord(subsystem, id);
    if (q.isEmpty()) return QJsonObject{{"subsystem",subsystem},{"task_id",id},{"status","UNREVIEWED"},{"effective_status","UNREVIEWED"}};
    const QString current = taskConfigFingerprint(subsystem, id);
    const bool same = !q.value("config_fingerprint").toString().isEmpty() && q.value("config_fingerprint").toString() == current;
    QString effective = q.value("status").toString(QStringLiteral("UNREVIEWED"));
    if (effective == QStringLiteral("PASS") && !same) effective = QStringLiteral("INVALIDATED");
    q["fingerprint_current"] = current;
    q["fingerprint_match"] = same;
    q["effective_status"] = effective;
    return q;
  }

  bool taskQualified(const QString &subsystem, const QString &id) const {
    return effectiveQualification(subsystem, id).value("effective_status").toString() == QStringLiteral("PASS");
  }

  QJsonObject commissioningState() const {
    QJsonObject out{{"ok",true},{"at_ms",nowMs()}};
    QJsonObject effective;
    const QJsonObject stored = loadTrialStore().value("qualifications").toObject();
    for (const QString &subsystem : {QStringLiteral("steering"),QStringLiteral("navigation"),QStringLiteral("perception")}) {
      QJsonObject domain;
      const QJsonObject src = stored.value(subsystem).toObject();
      for (const QString &id : src.keys()) domain[id] = effectiveQualification(subsystem,id);
      effective[subsystem] = domain;
    }
    const bool esc = taskQualified(QStringLiteral("steering"),QStringLiteral("4.9"));
    const bool nav16 = taskQualified(QStringLiteral("navigation"),QStringLiteral("N16.1"));
    const bool nav17 = taskQualified(QStringLiteral("navigation"),QStringLiteral("N17.1"));
    const bool per = taskQualified(QStringLiteral("perception"),QStringLiteral("4.9"));
    out["qualifications"] = effective;
    out["phase"] = QJsonObject{{"esc_pass",esc},{"navigation_pass",nav16||nav17},{"perception_pass",per},
      {"navigation_gate_open",esc},{"perception_gate_open",esc&&(nav16||nav17)}};
    return out;
  }

  bool setQualification(const QJsonObject &json, QString *message, QJsonObject *result) {
    const QString subsystem=json.value("subsystem").toString().trimmed();
    const QString id=json.value("id").toString().trimmed();
    const QString status=json.value("status").toString().trimmed().toUpper();
    if (!QStringList{"steering","navigation","perception"}.contains(subsystem) || id.isEmpty() ||
        !QStringList{"UNREVIEWED","PASS","FAIL","INVALIDATED"}.contains(status)) {
      if (message) *message = "qualification subsystem/id/status invalid";
      return false;
    }
    const QJsonArray trials=trialList(subsystem,id), requested=json.value("evidence_trial_ids").toArray();
    QSet<QString> existing; for(const QJsonValue &v:trials)existing.insert(v.toObject().value("trial_id").toString());
    QJsonArray evidence;
    for(const QJsonValue &v:requested){const QString tid=v.toString();if(!tid.isEmpty()&&existing.contains(tid))evidence.append(tid);else if(!tid.isEmpty()){if(message)*message="Evidence trial tidak ditemukan: "+tid;return false;}}
    if((status==QStringLiteral("PASS")||status==QStringLiteral("FAIL"))&&evidence.isEmpty()){if(message)*message=status+" membutuhkan minimal satu evidence trial yang valid";return false;}
    QJsonObject q{{"subsystem",subsystem},{"task_id",id},{"status",status},{"effective_status",status},
      {"evidence_trial_ids",evidence},{"config_fingerprint",taskConfigFingerprint(subsystem,id)},
      {"config_revision",configRevisionState().value("config_revision")},{"reviewer_source","operator_review"},
      {"reason",json.value("reason").toString()},{"metric_summary",json.value("metric_summary")},
      {"qualified_at",QDateTime::currentDateTime().toString(Qt::ISODateWithMs)}};
    QJsonObject store=loadTrialStore(),qual=store.value("qualifications").toObject(),domain=qual.value(subsystem).toObject();
    domain[id]=q;qual[subsystem]=domain;store["qualifications"]=qual;store["version"]=2;store["updated_at"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    if(!saveTrialStore(store,message))return false;
    if (result) *result = effectiveQualification(subsystem, id);
    if (message) *message = "Qualification " + subsystem + ":" + id + " → " + status;
    return true;
  }

  QJsonObject loadTrialStore() const {
    QFile f(trialStorePath());
    if (!f.open(QIODevice::ReadOnly)) return QJsonObject{{"version",1}};
    QJsonParseError e{}; const auto doc=QJsonDocument::fromJson(f.readAll(),&e);
    if (e.error!=QJsonParseError::NoError || !doc.isObject()) return QJsonObject{{"version",1}};
    return doc.object();
  }

  bool saveTrialStore(const QJsonObject &store, QString *message=nullptr) const {
    QDir().mkpath(QFileInfo(trialStorePath()).absolutePath());
    QSaveFile f(trialStorePath());
    if (!f.open(QIODevice::WriteOnly|QIODevice::Text)) { if(message)*message="Gagal membuka trial YAML"; return false; }
    f.write(QJsonDocument(store).toJson(QJsonDocument::Indented));
    if (!f.commit()) { if(message)*message="Gagal commit trial YAML"; return false; }
    return true;
  }

  QJsonArray trialList(const QString &subsystem, const QString &id) const {
    const auto store=loadTrialStore();
    return store.value(subsystem).toObject().value(id).toArray();
  }

  static std::optional<double> rowNumber(const QMap<QString,QString> &row,const QString &key) {
    bool ok=false; const double v=row.value(key).toDouble(&ok);
    if(!ok || !std::isfinite(v)) return std::nullopt;
    return v;
  }

  std::optional<double> meanRecording(const QString &key,bool stable=true) const {
    if(recordingRows_.empty()) return std::nullopt;
    int a=0,b=recordingRows_.size();
    if(stable && b>=5){a=static_cast<int>(std::floor(b*0.20));b=static_cast<int>(std::ceil(b*0.80));}
    double sum=0.0; int n=0;
    for(int i=a;i<b;++i) if(auto v=rowNumber(recordingRows_[i],key)){sum+=*v;++n;}
    if (!n) return std::nullopt;
    return sum / static_cast<double>(n);
  }

  std::optional<QPair<double,double>> firstLastRecording(const QString &key) const {
    std::optional<double> first,last;
    for(const auto &r:recordingRows_) if(auto v=rowNumber(r,key)){if(!first)first=*v;last=*v;}
    if (!first || !last) return std::nullopt;
    return qMakePair(*first, *last);
  }

  static double wrapAngle(double v) {
    while (v > kPi) v -= 2.0 * kPi;
    while (v <= -kPi) v += 2.0 * kPi;
    return v;
  }

  QJsonValue configStoredValue(const QString &fileKey, const QString &path) const {
    try {
      const auto c = configCandidates();
      if (!c.contains(fileKey)) return QJsonValue(QJsonValue::Undefined);
      const YAML::Node root = YAML::LoadFile(c.value(fileKey).toStdString());
      return yamlPathValue(root, path.split('.', Qt::SkipEmptyParts));
    } catch (...) {
      return QJsonValue(QJsonValue::Undefined);
    }
  }

  double configNumber(const QString &fileKey,const QString &path,double fallback) const {
    const QJsonValue v = configStoredValue(fileKey, path);
    return v.isDouble() && std::isfinite(v.toDouble()) ? v.toDouble() : fallback;
  }

  static void putNumber(QJsonObject &o,const QString &key,const std::optional<double> &v) {
    if(v && std::isfinite(*v)) o[key]=*v;
  }

  double maxLateralDeviation() const {
    QVector<QPair<double,double>> pts;
    for(const auto&r:recordingRows_){auto x=rowNumber(r,"gnss_map_odom.x"),y=rowNumber(r,"gnss_map_odom.y");if(x&&y)pts.push_back({*x,*y});}
    if(pts.size()<3) return 0.0;
    const double x0=pts.front().first,y0=pts.front().second,x1=pts.back().first,y1=pts.back().second;
    const double dx=x1-x0,dy=y1-y0,n=std::hypot(dx,dy); if(n<1e-6)return 0.0;
    double m=0.0; for(const auto&p:pts)m=std::max(m,std::abs(dy*(p.first-x0)-dx*(p.second-y0))/n); return m;
  }

  QJsonObject buildTrialSummary(int trialNo) const {
    QJsonObject o{{"trial_no",trialNo},{"trial_id",QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz")},
      {"subsystem",recordingSubsystem_},{"experiment_id",recordingSourceExperimentId_},{"candidate",recordingCandidate_},{"variation",recordingVariation_},
      {"condition",recordingCondition_},{"started_at",recordingStartedIso_},{"stopped_at",QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},
      {"samples",recordingRows_.size()},{"sample_rate_hz",recordingRateHz_},{"inputs",recordingTrialInputs_},
      {"config_revision",configRevisionState().value("config_revision")},
      {"task_config_fingerprint",taskConfigFingerprint(recordingSubsystem_,recordingSourceExperimentId_)}};
    auto input=[&](const char*k)->std::optional<double>{const QJsonValue v=recordingTrialInputs_.value(k); if(v.isDouble())return v.toDouble(); bool ok=false;double x=v.toString().toDouble(&ok);return ok?std::optional<double>(x):std::nullopt;};
    putNumber(o,"rpm_set",input("test_erpm")); putNumber(o,"steering_set_deg",input("test_steering_deg"));
    const auto rpm=meanRecording("vesc_right_values.rpm"),raw=meanRecording("esc_drive_raw"),esc=meanRecording("esc_odom.v"),
      gnss=meanRecording("gnss_vel.speed"),imu=meanRecording("imu_speed_kalman"),hacc=meanRecording("gnss_quality.hacc_m");
    putNumber(o,"mean_rpm_esc",rpm);putNumber(o,"mean_v_esc_raw_mps",raw);putNumber(o,"mean_v_esc_mps",esc);
    putNumber(o,"mean_v_gnss_mps",gnss);putNumber(o,"mean_v_imu_mps",imu);putNumber(o,"mean_hacc_m",hacc);
    const auto xp=firstLastRecording("gnss_map_odom.x"),yp=firstLastRecording("gnss_map_odom.y");
    if(xp&&yp){const double dx=xp->second-xp->first,dy=yp->second-yp->first;o["delta_x_m"]=dx;o["delta_y_m"]=dy;o["distance_gnss_m"]=std::hypot(dx,dy);}
    if(raw&&gnss&&std::abs(*raw)>0.02)o["scale_candidate"]=std::abs(*gnss)/std::abs(*raw);
    if(recordingSourceExperimentId_=="N3.1"||recordingSourceExperimentId_=="N3.2"){
      putNumber(o,"mean_steering_actual_rad",meanRecording("esc_steer_actual"));putNumber(o,"mean_gyro_z_rps",meanRecording("imu.gz"));
      putNumber(o,"mean_yaw_rate_model_rps",meanRecording("esc_kinematic_yaw_rate"));o["max_lateral_deviation_m"]=maxLateralDeviation();
      for(const auto &pair:QVector<QPair<QString,QString>>{{"delta_yaw_imu_rad","imu.yaw_rad"},{"delta_yaw_imu_mag_rad","imu_mag_heading.yaw_rad"},{"delta_yaw_neo3_mag_rad","neo3_mag_heading.yaw_rad"},{"delta_cog_gnss_rad","gnss_cog_fusion.yaw_rad"}})
        if(auto q=firstLastRecording(pair.second))o[pair.first]=wrapAngle(q->second-q->first);
    }
    if(recordingSourceExperimentId_=="N3.1"){
      const auto w=meanRecording("imu.gz"); const double wb=configNumber("esc","esc_ackermann.ros__parameters.wheelbase_m",0.7);
      if(w&&gnss&&std::abs(*gnss)>0.05)o["zero_steer_correction_candidate_deg"]=-std::atan(wb*(*w)/std::max(0.05,std::abs(*gnss)))*180.0/kPi;
    }
    if(recordingSourceExperimentId_=="N3.2"){
      const auto w=meanRecording("imu.gz"),wm=meanRecording("esc_kinematic_yaw_rate"),st=meanRecording("esc_steer_actual");
      if(w&&gnss&&std::abs(*w)>0.02)o["radius_gnss_gyro_m"]=std::abs(*gnss/(*w));
      if(wm&&esc&&std::abs(*wm)>0.02)o["radius_model_m"]=std::abs(*esc/(*wm));
      if(st&&w&&gnss&&std::abs(*w)>0.02){
        const double track=configNumber("esc","esc_ackermann.ros__parameters.track_width_m",0.48);
        const double wb=configNumber("esc","esc_ackermann.ros__parameters.wheelbase_m",0.70);
        const double r=std::abs(*gnss/(*w));
        const double inner=std::max(1.0e-6,r-0.5*track);
        const double l=inner*std::tan(std::abs(*st));
        if(std::isfinite(l)&&l>0)o["effective_wheelbase_candidate_m"]=l;
        const double required=std::atan(wb/inner);
        if(std::abs(*st)>0.5*kPi/180.0 && std::isfinite(required)) o["steering_scale_candidate"]=required/std::abs(*st);
      }
    }
    return o;
  }

  bool appendCurrentTrial(QJsonObject *summary,QJsonArray *trials,QString *message) {
    QJsonObject store=loadTrialStore(); QJsonObject domain=store.value(recordingSubsystem_).toObject();
    QJsonArray list=domain.value(recordingSourceExperimentId_).toArray();
    QJsonObject one=buildTrialSummary(list.size()+1); list.append(one);
    domain[recordingSourceExperimentId_]=list; store[recordingSubsystem_]=domain;
    store["updated_at"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    if(!saveTrialStore(store,message))return false;
    if (summary) *summary = one;
    if (trials) *trials = list;
    return true;
  }

  bool deleteTrial(const QJsonObject &json,QString *message,QJsonObject *result) {
    const QString subsystem=json.value("subsystem").toString(),id=json.value("id").toString(),trialId=json.value("trial_id").toString();
    if(subsystem.isEmpty()||id.isEmpty()||trialId.isEmpty()){if(message)*message="Identitas trial tidak lengkap";return false;}
    QJsonObject store=loadTrialStore(),domain=store.value(subsystem).toObject();const QJsonArray old=domain.value(id).toArray();QJsonArray list;
    bool removed=false;for(const auto&v:old){const auto o=v.toObject();if(o.value("trial_id").toString()==trialId){removed=true;continue;}list.append(o);}
    if(!removed){if(message)*message="Trial tidak ditemukan";return false;}
    for(int i=0;i<list.size();++i){auto o=list[i].toObject();o["trial_no"]=i+1;list[i]=o;}
    domain[id]=list;store[subsystem]=domain;
    QJsonObject qualifications=store.value("qualifications").toObject(),qDomain=qualifications.value(subsystem).toObject(),qual=qDomain.value(id).toObject();
    bool evidenceReferenced=false;for(const QJsonValue &ev:qual.value("evidence_trial_ids").toArray())if(ev.toString()==trialId){evidenceReferenced=true;break;}
    if(evidenceReferenced){qual["status"]="INVALIDATED";qual["effective_status"]="INVALIDATED";qual["reason"]="Referenced evidence trial deleted: "+trialId;qual["invalidated_at"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);qDomain[id]=qual;qualifications[subsystem]=qDomain;store["qualifications"]=qualifications;}
    store["updated_at"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    if(!saveTrialStore(store,message))return false;
    if (result) *result = QJsonObject{{"trials", list}, {"trial_count", list.size()}};
    if (message) *message = "Trial dihapus dari YAML";
    return true;
  }

  static double median(QVector<double> v){if(v.empty())return std::numeric_limits<double>::quiet_NaN();std::sort(v.begin(),v.end());const int n=v.size();return n%2?v[n/2]:0.5*(v[n/2-1]+v[n/2]);}

  bool calculateOptimalScaleProposal(QString *message,QJsonObject *result) {
    const QJsonArray list = trialList("navigation", "N2.1");
    struct P { double raw, gnss, scale; QString id; };
    QVector<P> valid;
    QVector<double> scales;
    for (const auto &v : list) {
      const auto o = v.toObject();
      const double raw = std::abs(o.value("mean_v_esc_raw_mps").toDouble());
      const double gnss = std::abs(o.value("mean_v_gnss_mps").toDouble());
      if (raw <= 0.03 || gnss <= 0.03) continue;
      const double k = gnss / raw;
      if (std::isfinite(k) && k >= 0.20 && k <= 5.00) {
        valid.push_back({raw, gnss, k, o.value("trial_id").toString()});
        scales.push_back(k);
      }
    }
    if (valid.size() < 3) {
      if (message) *message = "Butuh minimal 3 trial valid N2.1 untuk scale optimal";
      return false;
    }
    const double med = median(scales);
    QVector<double> dev;
    for (double k : scales) dev.push_back(std::abs(k - med));
    const double mad = median(dev);
    const double tol = std::max(0.05 * std::abs(med), 3.0 * 1.4826 * mad);
    double xy = 0.0, xx = 0.0;
    int accepted = 0;
    QJsonArray rejected;
    for (const auto &p : valid) {
      if (std::abs(p.scale - med) > tol) { rejected.append(p.id); continue; }
      xy += p.raw * p.gnss;
      xx += p.raw * p.raw;
      ++accepted;
    }
    if (accepted < 2 || xx <= 1e-9) {
      if (message) *message = "Trial valid setelah outlier rejection tidak cukup";
      return false;
    }
    const double scale = std::clamp(xy / xx, 0.20, 5.0);
    QJsonObject store = loadTrialStore();
    QJsonObject results = store.value("calibration_results").toObject();
    QJsonObject nav = results.value("navigation").toObject();
    nav["N2.1"] = QJsonObject{{"optimal_scale", scale}, {"accepted_trials", accepted},
      {"total_valid_trials", valid.size()}, {"median_candidate", med}, {"mad", mad},
      {"rejected_trial_ids", rejected}, {"applied", false}, {"proposal_only", true},
      {"updated_at", QDateTime::currentDateTime().toString(Qt::ISODateWithMs)}};
    results["navigation"] = nav;
    store["calibration_results"] = results;
    saveTrialStore(store, nullptr);
    if (result) *result = QJsonObject{{"scale", scale}, {"accepted_trials", accepted},
      {"valid_trials", valid.size()}, {"rejected_trial_ids", rejected},
      {"applied", false}, {"proposal_only", true}};
    if (message) *message = "Scale optimal berhasil dihitung sebagai proposal; belum ada YAML/runtime write";
    return true;
  }

  bool exportTrialArtifacts(const QString &csvPath,const QJsonObject &summary,const QJsonArray &trials,const QJsonObject &clientArtifacts,QJsonObject *result,QString *message) {
    const QString token=QString::number(QCoreApplication::applicationPid())+"_"+QString::number(QDateTime::currentMSecsSinceEpoch());
    const QString tmpSummary="/tmp/agv_trial_summary_"+token+".json",tmpTrials="/tmp/agv_trial_trials_"+token+".json",tmpSpec="/tmp/agv_trial_spec_"+token+".json";
    auto writeJson=[](const QString&path,const QJsonDocument&doc){QSaveFile f(path);if(!f.open(QIODevice::WriteOnly))return false;f.write(doc.toJson(QJsonDocument::Compact));return f.commit();};
    QJsonObject spec{{"live_series",recordingLiveSeries_},{"graphs",recordingGraphs_}};
    if(!writeJson(tmpSummary,QJsonDocument(summary))||!writeJson(tmpTrials,QJsonDocument(trials))||!writeJson(tmpSpec,QJsonDocument(spec))){if(message)*message="Gagal menulis temporary export spec";return false;}
    QString base=csvPath;if(base.endsWith(".csv"))base.chop(4);const QString xlsx=base+".xlsx";
    QProcess proc;proc.setProgram(agvPythonPath());
    QStringList exportArgs{agvPath(QStringLiteral("src/navigation/tools/export_trial_artifacts.py")),"--csv",csvPath,"--summary",tmpSummary,"--trials",tmpTrials,"--spec",tmpSpec,"--xlsx",xlsx,"--png-prefix",base};
    const QString dataCanonical = QFileInfo(agvPath(QStringLiteral("data"))).canonicalFilePath();
    int tableSheetCount = 0;
    for (const auto &v : clientArtifacts.value("table_csv_paths").toArray()) {
      const QString requested = QDir::cleanPath(v.toString());
      const QFileInfo info(requested); const QString canonical = info.canonicalFilePath();
      if (canonical.isEmpty() || dataCanonical.isEmpty() || !canonical.startsWith(dataCanonical + QDir::separator()) ||
          info.suffix().compare(QStringLiteral("csv"), Qt::CaseInsensitive) != 0 || !info.isFile()) continue;
      exportArgs << "--table-csv" << canonical; ++tableSheetCount;
    }
    proc.setArguments(exportArgs);
    proc.start();const bool started=proc.waitForStarted(3000);const bool done=started&&proc.waitForFinished(60000);
    const QByteArray out=proc.readAllStandardOutput(),err=proc.readAllStandardError();QFile::remove(tmpSummary);QFile::remove(tmpTrials);QFile::remove(tmpSpec);
    if(!done||proc.exitStatus()!=QProcess::NormalExit||proc.exitCode()!=0){if(message)*message=QStringLiteral("Exporter gagal: ")+QString::fromUtf8(err).left(600);return false;}
    QJsonParseError pe{};const auto doc=QJsonDocument::fromJson(out.trimmed(),&pe);if(pe.error!=QJsonParseError::NoError||!doc.isObject()){if(message)*message="Output exporter tidak valid";return false;}
    const auto artifacts=doc.object();lastXlsxPath_=artifacts.value("xlsx").toString();lastXlsxName_=QFileInfo(lastXlsxPath_).fileName();lastGraphPaths_.clear();
    for(const auto&v:artifacts.value("pngs").toArray())if(QFileInfo::exists(v.toString()))lastGraphPaths_<<v.toString();
    int browserPngCount = 0;
    const auto browserPngs = clientArtifacts.value("browser_graph_pngs").toArray();
    for (const auto &v : browserPngs) {
      const QJsonObject one = v.toObject(); const int idx = one.value("index").toInt();
      if (idx < 1 || idx > recordingGraphs_.size() || idx > 32) continue;
      QString dataUrl = one.value("data_url").toString();
      const QString prefix = QStringLiteral("data:image/png;base64,");
      if (!dataUrl.startsWith(prefix) || dataUrl.size() > 6 * 1024 * 1024) continue;
      const QByteArray bytes = QByteArray::fromBase64(dataUrl.mid(prefix.size()).toLatin1());
      if (bytes.size() < 8 || bytes.size() > 4 * 1024 * 1024 || !bytes.startsWith(QByteArray::fromHex("89504e470d0a1a0a"))) continue;
      const QString path = QStringLiteral("%1_G%2.png").arg(base).arg(idx, 2, 10, QChar('0'));
      QSaveFile f(path); if (!f.open(QIODevice::WriteOnly) || f.write(bytes) != bytes.size() || !f.commit()) continue;
      const int pos = idx - 1; if (pos < lastGraphPaths_.size()) lastGraphPaths_[pos] = path; else if (QFileInfo::exists(path)) lastGraphPaths_ << path;
      ++browserPngCount;
    }
    if(result){(*result)["xlsx_path"]=lastXlsxPath_;(*result)["xlsx_download_url"]="/api/experiment/record/last.xlsx";(*result)["table_sheet_count"]=tableSheetCount;(*result)["browser_graph_png_count"]=browserPngCount;QJsonArray urls,paths;for(int i=0;i<lastGraphPaths_.size();++i){urls.append(QString("/api/experiment/record/last-graph-%1.png").arg(i+1));paths.append(lastGraphPaths_[i]);}(*result)["graph_download_urls"]=urls;(*result)["graph_png_paths"]=paths;}
    if (message) *message = QStringLiteral("XLSX + %1 sheet tabel + %2 PNG browser (%3 fallback PNG) berhasil dibuat").arg(tableSheetCount).arg(browserPngCount).arg(lastGraphPaths_.size()-browserPngCount);
    return true;
  }

  bool saveRecordingFiles(QJsonObject *result, QString *message) {
    if (recordingRows_.isEmpty()) {
      if (message) *message = QStringLiteral("Tidak ada sampel untuk disimpan");
      return false;
    }

    const QString domain = recordingSubsystem_ == QStringLiteral("navigation") ? QStringLiteral("navigasi") :
                           recordingSubsystem_ == QStringLiteral("perception") ? QStringLiteral("presepsi") :
                           QStringLiteral("esc");
    const QString dataRoot = QDir(agvPath(QStringLiteral("data"))).filePath(domain);
    if (!QDir().mkpath(dataRoot)) {
      if (message) *message = QStringLiteral("Gagal membuat folder data: ") + dataRoot;
      return false;
    }

    QSet<QString> all;
    for (const auto &row : recordingRows_) {
      for (auto it = row.cbegin(); it != row.cend(); ++it) all.insert(it.key());
    }
    QStringList columns = all.values();
    std::sort(columns.begin(), columns.end());
    for (const QString &preferred : {QStringLiteral("condition"), QStringLiteral("variation"),
                                      QStringLiteral("section_label"), QStringLiteral("source_experiment_id"),
                                      QStringLiteral("section_id"), QStringLiteral("subsystem"),
                                      QStringLiteral("elapsed_s"), QStringLiteral("time_iso")}) {
      columns.removeAll(preferred);
    }
    columns.prepend(QStringLiteral("condition"));
    columns.prepend(QStringLiteral("variation"));
    columns.prepend(QStringLiteral("section_label"));
    columns.prepend(QStringLiteral("source_experiment_id"));
    columns.prepend(QStringLiteral("section_id"));
    columns.prepend(QStringLiteral("subsystem"));
    columns.prepend(QStringLiteral("elapsed_s"));
    columns.prepend(QStringLiteral("time_iso"));

    QByteArray csv;
    QStringList escapedHeader;
    for (const QString &column : columns) escapedHeader << csvEscape(column);
    csv += escapedHeader.join(',').toUtf8() + '\n';
    for (const auto &row : recordingRows_) {
      QStringList values;
      for (const QString &column : columns) values << csvEscape(row.value(column));
      csv += values.join(',').toUtf8() + '\n';
    }

    const QString minuteStamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString baseStem = recordingCsvStem(recordingId_) + QStringLiteral("_") + minuteStamp;
    QString fileName = baseStem + QStringLiteral(".csv");
    int suffix = 2;
    while (QFileInfo::exists(QDir(dataRoot).filePath(fileName))) {
      fileName = baseStem + QStringLiteral("_%1.csv").arg(suffix++, 2, 10, QChar('0'));
    }
    const QString primaryPath = QDir(dataRoot).filePath(fileName);
    QSaveFile csvFile(primaryPath);
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
      if (message) *message = QStringLiteral("Gagal membuka CSV: ") + primaryPath;
      return false;
    }
    csvFile.write(csv);
    if (!csvFile.commit()) {
      if (message) *message = QStringLiteral("Gagal commit CSV: ") + primaryPath;
      return false;
    }

    // Browser download memakai buffer memory yang sama; tidak membuat summary,
    // manifest, config JSON, atau folder run tambahan di AGV_ROOT/data.
    lastDownloadCsv_ = csv;
    lastDownloadName_ = fileName;
    if (result) {
      *result = QJsonObject{{"subsystem", recordingSubsystem_}, {"section_id", recordingId_},
                           {"source_experiment_id", recordingSourceExperimentId_},
                           {"section_label", recordingSectionLabel_}, {"sample_count", recordingRows_.size()},
                           {"primary_csv", primaryPath}, {"raw_csv", primaryPath},
                           {"report_root", dataRoot},
                           {"download_url", QStringLiteral("/api/experiment/record/last.csv")},
                           {"download_name", lastDownloadName_}};
    }
    if (message) {
      *message = QStringLiteral("CSV tersimpan: ") + primaryPath +
                 QStringLiteral("; download browser siap");
    }
    return true;
  }

  bool stopRecording(const QJsonObject &clientArtifacts, QString *message, QJsonObject *result) {
    if (!recording_) {
      if (message) *message = QStringLiteral("Tidak ada recording aktif");
      return false;
    }
    captureRecordingSample();
    recording_ = false;
    recordingTimer_.stop();
    QString csvMessage;
    const bool csvSaved = saveRecordingFiles(result, &csvMessage);
    QJsonObject summary; QJsonArray trials; QString trialMessage;
    const bool trialSaved = appendCurrentTrial(&summary, &trials, &trialMessage);
    bool artifactsSaved = false; QString artifactMessage;
    bool manifestSaved = false; QString manifestMessage;
    if (csvSaved && result) {
      const QString csvPath = result->value("primary_csv").toString();
      artifactsSaved = exportTrialArtifacts(csvPath, summary, trials, clientArtifacts, result, &artifactMessage);
      manifestSaved = writeSessionManifest(csvPath, result, &manifestMessage);
      (*result)["trial_summary"] = summary;
      (*result)["trials"] = trials;
      (*result)["trial_yaml"] = trialStorePath();
      (*result)["trial_saved"] = trialSaved;
      (*result)["artifacts_saved"] = artifactsSaved;
    }
    if (message) *message = csvMessage + QStringLiteral(" | ") + trialMessage + QStringLiteral(" | ") + artifactMessage + QStringLiteral(" | ") + manifestMessage;
    recordingRows_.clear(); recordingTuningConfig_.clear(); recordingPaths_.clear();
    recordingCandidate_.clear(); recordingTrialInputs_=QJsonObject(); recordingLiveSeries_=QJsonObject(); recordingGraphs_=QJsonArray();
    return csvSaved && trialSaved && artifactsSaved && manifestSaved;
  }

  void openSse(QTcpSocket *socket) {
    QByteArray headers;
    headers += "HTTP/1.1 200 OK\r\n";
    headers += "Content-Type: text/event-stream\r\n";
    headers += "Cache-Control: no-cache, no-transform\r\n";
    headers += "Connection: keep-alive\r\n";
    headers += "X-Accel-Buffering: no\r\n\r\n";
    socket->write(headers);
    socket->setProperty("sse", true);
    sseClients_.append(QPointer<QTcpSocket>(socket));
    QJsonObject snapshotObject = bridge_->snapshot();
    snapshotObject["__event_seq"] = QString::number(sseSequence_);
    const QByteArray snapshot = QJsonDocument(snapshotObject).toJson(QJsonDocument::Compact);
    socket->write("event: snapshot\ndata: " + snapshot + "\n\n");
    socket->flush();
  }

  void broadcastEvents() {
    for (int i = sseClients_.size() - 1; i >= 0; --i) {
      if (sseClients_[i].isNull() || sseClients_[i]->state() != QAbstractSocket::ConnectedState) sseClients_.removeAt(i);
    }
    QJsonObject delta = bridge_->takeDelta();
    QByteArray payload;
    if (!delta.isEmpty()) {
      ++sseSequence_;
      delta["__event_seq"] = QString::number(sseSequence_);
      payload = "data: " + QJsonDocument(delta).toJson(QJsonDocument::Compact) + "\n\n";
    }
    ++heartbeatTicks_;
    const bool heartbeat = heartbeatTicks_ >= 250;  // 5 s at 50 Hz event timer
    if (heartbeat) heartbeatTicks_ = 0;
    for (const auto &client : sseClients_) {
      if (client.isNull()) continue;
      if (client->bytesToWrite() > 2 * 1024 * 1024) continue;
      if (!payload.isEmpty()) client->write(payload);
      if (heartbeat) client->write(": heartbeat\n\n");
    }
  }

  void serveStatic(QTcpSocket *socket, QString path) {
    if (path == "/") path = "/index.html";
    if (path.contains("..")) return sendJson(socket, 403, QJsonObject{{"ok", false}, {"message", "Forbidden"}});
    if (staticRoot_.isEmpty()) return sendJson(socket, 500, QJsonObject{{"ok", false}, {"message", "Static web root tidak ditemukan"}});
    // Use lexical containment instead of canonicalFilePath() for the requested
    // file.  With colcon --symlink-install, static assets are symlinks into src/;
    // canonicalizing the file resolves outside install/share and incorrectly
    // rejects valid files as 404.  '..' is already rejected above and the
    // cleaned absolute path must remain inside the trusted static root.
    const QString cleanRoot = QDir::cleanPath(QDir(staticRoot_).absolutePath());
    const QString cleanFile = QDir::cleanPath(QDir(cleanRoot).absoluteFilePath(path.mid(1)));
    const QString rootPrefix = cleanRoot.endsWith(QDir::separator()) ? cleanRoot : cleanRoot + QDir::separator();
    if (!cleanFile.startsWith(rootPrefix) || !QFileInfo(cleanFile).isFile()) {
      return sendJson(socket, 404, QJsonObject{{"ok", false}, {"message", "Not found"}});
    }
    QFile file(cleanFile);
    if (!file.open(QIODevice::ReadOnly)) return sendJson(socket, 404, QJsonObject{{"ok", false}, {"message", "Not found"}});
    sendBytes(socket, 200, mimeTypeForPath(cleanFile), file.readAll(), {{"Cache-Control", "no-cache"}});
  }

  static QByteArray statusText(int status) {
    switch (status) {
      case 200: return "OK";
      case 400: return "Bad Request";
      case 403: return "Forbidden";
      case 404: return "Not Found";
      case 405: return "Method Not Allowed";
      case 409: return "Conflict";
      case 413: return "Payload Too Large";
      case 500: return "Internal Server Error";
      case 503: return "Service Unavailable";
      default: return "Response";
    }
  }

  void sendJson(QTcpSocket *socket, int status, const QJsonObject &json) {
    sendBytes(socket, status, "application/json; charset=utf-8", QJsonDocument(json).toJson(QJsonDocument::Compact),
              {{"Cache-Control", "no-store"}});
  }

  void sendText(QTcpSocket *socket, int status, const QByteArray &contentType, const QByteArray &text) {
    sendBytes(socket, status, contentType, text, {{"Cache-Control", "no-store"}});
  }

  void sendBytes(QTcpSocket *socket, int status, const QByteArray &contentType, const QByteArray &body,
                 const QMap<QByteArray, QByteArray> &extra = {}) {
    QByteArray headers = "HTTP/1.1 " + QByteArray::number(status) + " " + statusText(status) + "\r\n";
    headers += "Content-Type: " + contentType + "\r\n";
    headers += "Content-Length: " + QByteArray::number(body.size()) + "\r\n";
    headers += "Connection: close\r\n";
    headers += "X-Content-Type-Options: nosniff\r\n";
    headers += "Referrer-Policy: no-referrer\r\n";
    headers += "Content-Security-Policy: default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'\r\n";
    for (auto it = extra.cbegin(); it != extra.cend(); ++it) headers += it.key() + ": " + it.value() + "\r\n";
    headers += "\r\n";
    socket->write(headers);
    socket->write(body);
    socket->disconnectFromHost();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  QCoreApplication app(argc, argv);
  QCoreApplication::setApplicationName("AGV Web GUI");
  QCoreApplication::setApplicationVersion("1.0");
  int result = 1;
  try {
    WebRosBridge bridge;
    LocalHttpServer server(&bridge);
    if (!server.start(bridge.bindAddress(), bridge.port())) {
      RCLCPP_FATAL(rclcpp::get_logger("agv_web_gui"), "Web GUI gagal start: bind address/static root tidak valid");
    } else {
      if (server.isListening()) {
        RCLCPP_INFO(rclcpp::get_logger("agv_web_gui"), "Web GUI aktif: http://%s:%d%s",
                    bridge.bindAddress().toUtf8().constData(), bridge.port(), bridge.readOnly() ? " [READ ONLY]" : "");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("agv_web_gui"), "Web GUI menunggu port %s:%d; retry internal aktif tanpa process respawn",
                    bridge.bindAddress().toUtf8().constData(), bridge.port());
      }
      QTimer rosShutdownGuard;
      rosShutdownGuard.setInterval(200);
      QObject::connect(&rosShutdownGuard, &QTimer::timeout, &app, [&app]() {
        if (!rclcpp::ok()) app.quit();
      });
      rosShutdownGuard.start();
      result = app.exec();
    }
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("agv_web_gui"), "Web GUI fatal: %s", e.what());
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
