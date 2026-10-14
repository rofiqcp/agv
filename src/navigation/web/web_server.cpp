#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
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
#include <QPainter>
#include <QPen>
#include <QFont>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QList>
#include <QLockFile>
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
#include <map_msgs/msg/occupancy_grid_update.hpp>
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
#include "bab4_perception_session.hpp"

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


struct PgmImageData {
  int width = 0;
  int height = 0;
  int maxValue = 0;
  QByteArray pixels;
};

QByteArray nextPgmToken(const QByteArray &data, int *offset) {
  int i = *offset;
  while (i < data.size()) {
    const char c = data.at(i);
    if (c == '#') { while (i < data.size() && data.at(i) != '\n') ++i; continue; }
    if (std::isspace(static_cast<unsigned char>(c))) { ++i; continue; }
    break;
  }
  const int start = i;
  while (i < data.size()) {
    const char c = data.at(i);
    if (c == '#' || std::isspace(static_cast<unsigned char>(c))) break;
    ++i;
  }
  *offset = i;
  return data.mid(start, i - start);
}

bool loadPgmP5(const QString &path, PgmImageData *out, QString *message) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) { if (message) *message = QStringLiteral("PGM source tidak dapat dibuka: ") + path; return false; }
  const QByteArray data = f.readAll();
  int pos = 0; bool okW=false, okH=false, okM=false;
  if (nextPgmToken(data,&pos) != "P5") { if(message)*message=QStringLiteral("PGM wajib format P5"); return false; }
  const int w=nextPgmToken(data,&pos).toInt(&okW), h=nextPgmToken(data,&pos).toInt(&okH), m=nextPgmToken(data,&pos).toInt(&okM);
  if(pos<data.size() && std::isspace(static_cast<unsigned char>(data.at(pos)))) ++pos;
  const qint64 need=static_cast<qint64>(w)*static_cast<qint64>(h);
  if(!okW||!okH||!okM||w<=0||h<=0||m!=255||need<=0||need>100000000LL||data.size()-pos<need){if(message)*message=QStringLiteral("Header/data PGM tidak valid");return false;}
  out->width=w;out->height=h;out->maxValue=m;out->pixels=data.mid(pos,static_cast<int>(need));return true;
}

bool saveMapEditorCopy(const QJsonObject &json, QJsonObject *result, QString *message) {
  const QString navCfg=packageConfigDir("navigation","AGV_CONFIG_DIR");
  const QString mapDir=QDir::cleanPath(QDir(navCfg).absoluteFilePath(QStringLiteral("../maps/undip")));
  const QString sourceYaml=QDir(mapDir).absoluteFilePath(QStringLiteral("undip_nav2.yaml"));
  if(!QFileInfo(sourceYaml).isFile()){if(message)*message=QStringLiteral("Source map YAML tidak ditemukan: ")+sourceYaml;return false;}
  QString sourcePgm; double resolution=0.0,originX=0.0,originY=0.0,originYaw=0.0,occupied=.65,free=.196;int negate=0;QString mode=QStringLiteral("trinary");
  try{
    const YAML::Node y=YAML::LoadFile(sourceYaml.toStdString());
    sourcePgm=QString::fromStdString(y["image"].as<std::string>());if(QDir::isRelativePath(sourcePgm))sourcePgm=QDir(mapDir).absoluteFilePath(sourcePgm);
    resolution=y["resolution"].as<double>();if(y["origin"]&&y["origin"].size()>=3){originX=y["origin"][0].as<double>();originY=y["origin"][1].as<double>();originYaw=y["origin"][2].as<double>();}
    if (y["occupied_thresh"]) occupied = y["occupied_thresh"].as<double>();
    if (y["free_thresh"]) free = y["free_thresh"].as<double>();
    if (y["negate"]) negate = y["negate"].as<int>();
    if (y["mode"]) mode = QString::fromStdString(y["mode"].as<std::string>());
  }catch(const std::exception &e){if(message)*message=QStringLiteral("Gagal membaca map YAML: ")+QString::fromUtf8(e.what());return false;}
  if(!(resolution>0.0)||std::abs(originYaw)>1e-9){if(message)*message=QStringLiteral("Map editor saat ini memerlukan resolution>0 dan origin yaw=0");return false;}
  PgmImageData original; if(!loadPgmP5(sourcePgm,&original,message))return false; QByteArray edited=original.pixels;
  const QJsonArray strokes=json.value("strokes").toArray();if(strokes.size()>50000){if(message)*message=QStringLiteral("Terlalu banyak brush stroke (>50000)");return false;}
  int accepted=0;
  for(const QJsonValue &v:strokes){const QJsonObject st=v.toObject();const double x=st.value("x").toDouble(std::numeric_limits<double>::quiet_NaN()),y=st.value("y").toDouble(std::numeric_limits<double>::quiet_NaN()),radius=st.value("radius_m").toDouble(.5);const QString action=st.value("action").toString(QStringLiteral("add"));if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(radius)||radius<=0||radius>10.0||(action!="add"&&action!="erase"))continue;const int cx=static_cast<int>(std::floor((x-originX)/resolution)),cy=static_cast<int>(std::floor((y-originY)/resolution)),rr=std::max(1,static_cast<int>(std::ceil(radius/resolution)));if(cx+rr<0||cy+rr<0||cx-rr>=original.width||cy-rr>=original.height)continue;const int r2=rr*rr;for(int yy=std::max(0,cy-rr);yy<=std::min(original.height-1,cy+rr);++yy){for(int xx=std::max(0,cx-rr);xx<=std::min(original.width-1,cx+rr);++xx){const int dx=xx-cx,dy=yy-cy;if(dx*dx+dy*dy>r2)continue;const int row=original.height-1-yy,idx=row*original.width+xx;edited[idx]=(action=="add")?char(0):original.pixels.at(idx);}}++accepted;}
  qint64 changed=0;for(int i=0;i<edited.size();++i)if(edited.at(i)!=original.pixels.at(i))++changed;
  const QString outDir=QDir(mapDir).absoluteFilePath(QStringLiteral("edits"));if(!QDir().mkpath(outDir)){if(message)*message=QStringLiteral("Gagal membuat folder edits");return false;}
  const QString stamp=QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")),base=QStringLiteral("undip_nav2_edit_")+stamp;
  const QString pgmPath=QDir(outDir).absoluteFilePath(base+QStringLiteral(".pgm")),yamlPath=QDir(outDir).absoluteFilePath(base+QStringLiteral(".yaml")),previewPath=QDir(outDir).absoluteFilePath(base+QStringLiteral("_preview.png"));
  QSaveFile pgm(pgmPath);if(!pgm.open(QIODevice::WriteOnly)){if(message)*message=QStringLiteral("Gagal membuat output PGM");return false;}pgm.write(QByteArray("P5\n# ADV static map editor; runtime inflation/lethal overlays are NOT baked\n")+QByteArray::number(original.width)+" "+QByteArray::number(original.height)+"\n255\n");pgm.write(edited);if(!pgm.commit()){if(message)*message=QStringLiteral("Commit PGM gagal");return false;}
  const QString yamlText=QStringLiteral("image: \"")+QFileInfo(pgmPath).fileName()+QStringLiteral("\"\nmode: ")+mode+QStringLiteral("\nresolution: ")+QString::number(resolution,'f',11)+QStringLiteral("\norigin: [")+QString::number(originX,'f',9)+QStringLiteral(", ")+QString::number(originY,'f',9)+QStringLiteral(", ")+QString::number(originYaw,'f',9)+QStringLiteral("]\nnegate: ")+QString::number(negate)+QStringLiteral("\noccupied_thresh: ")+QString::number(occupied,'g',12)+QStringLiteral("\nfree_thresh: ")+QString::number(free,'g',12)+QStringLiteral("\n");
  QSaveFile yf(yamlPath);if(!yf.open(QIODevice::WriteOnly)||yf.write(yamlText.toUtf8())<0||!yf.commit()){QFile::remove(pgmPath);if(message)*message=QStringLiteral("Save YAML pendamping gagal");return false;}
  QImage preview(original.width,original.height,QImage::Format_Grayscale8);for(int row=0;row<original.height;++row)std::memcpy(preview.scanLine(row),edited.constData()+static_cast<qint64>(row)*original.width,original.width);preview.save(previewPath,"PNG");
  *result=QJsonObject{{"source_pgm",sourcePgm},{"source_yaml",sourceYaml},{"pgm_path",pgmPath},{"yaml_path",yamlPath},{"preview_path",previewPath},{"download_url",QStringLiteral("/api/map-editor/download/")+QFileInfo(pgmPath).fileName()},{"width",original.width},{"height",original.height},{"resolution",resolution},{"accepted_strokes",accepted},{"changed_cells",static_cast<double>(changed)},{"changed_area_m2",changed*resolution*resolution},{"active_map_changed",false}};
  if (message) *message = QStringLiteral("Map edit copy tersimpan; active map tidak diubah");
  return true;
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
  if (value.isArray() && originalScalar.isEmpty()) {
    // Block-style ROS arrays (e.g. process_noise_covariance:\n  - 0.05 ...)
    // cannot be replaced by an inline value while leaving their old "- item"
    // children in place. Collapse the complete child sequence atomically.
    const int parentIndent = raw.size() - raw.trimmed().size();
    int blockEnd = target + 1;
    QStringList originalItems;
    while (blockEnd < lines.size()) {
      const QString childRaw = lines[blockEnd];
      const QString child = childRaw.trimmed();
      if (child.isEmpty()) { ++blockEnd; continue; }
      const int childIndent = childRaw.size() - childRaw.trimmed().size();
      // yaml-cpp accepts block sequence entries at the same indentation as
      // the parent key ("key:\n- item" / indented equivalent). Keep those
      // entries in this block; stop on the next sibling mapping key.
      if (childIndent < parentIndent || (childIndent == parentIndent && !child.startsWith('-'))) break;
      if (child.startsWith('-')) {
        QString token = child.mid(1).trimmed();
        const int childHash = token.indexOf('#');
        if (childHash >= 0) token = token.left(childHash).trimmed();
        if (!token.isEmpty()) originalItems << token;
      }
      ++blockEnd;
    }
    const QString typeHint = originalItems.isEmpty()
      ? QStringLiteral("[]")
      : QStringLiteral("[") + originalItems.join(QStringLiteral(", ")) + QStringLiteral("]");
    lines[target] = prefix + QStringLiteral(" ") + jsonScalarPreservingYamlType(value, typeHint) + comment;
    if (blockEnd > target + 1) {
      lines.erase(lines.begin() + target + 1, lines.begin() + blockEnd);
    }
  } else {
    lines[target] = prefix + QStringLiteral(" ") + jsonScalarPreservingYamlType(value, originalScalar) + comment;
  }
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
    // Port tetap dikunci ke 5000, tetapi bind address boleh loopback atau
    // 0.0.0.0 agar ROS Web dapat diakses dari browser operator di LAN.
    if (bindAddress_ != "127.0.0.1" && bindAddress_ != "localhost" && bindAddress_ != "0.0.0.0") {
      QHostAddress configuredAddress;
      if (!configuredAddress.setAddress(bindAddress_)) {
        throw std::invalid_argument("ROS Web bind_address tidak valid");
      }
    }
    if (configuredPort != 5000) {
      throw std::invalid_argument("ROS Web wajib port 5000");
    }
    port_ = 5000;
    readOnly_ = node_->declare_parameter<bool>("read_only", false);
    cameraJpegFps_ = std::clamp(node_->declare_parameter<double>("camera_jpeg_fps", 5.0), 0.5, 12.0);
    cameraFullresFallbackEnabled_ = node_->declare_parameter<bool>("camera_fullres_fallback_enabled", false);
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

    // N2.1 special case: drive_erpm_per_mps is overridden by the parent autonomous
    // launch from vehicle.yaml. Respawning only /esc_ackermann would therefore reuse
    // the old launch argument. Keep the persistent YAML transaction, but apply this
    // one parameter through its proven dynamic callback after any required respawns.
    const QString driveErpmPath = QStringLiteral("esc_ackermann.ros__parameters.drive_erpm_per_mps");
    const QSet<QString> liveEscTuningPaths{
      QStringLiteral("esc_ackermann.ros__parameters.odom_v_variance_base"),
      QStringLiteral("esc_ackermann.ros__parameters.odom_yaw_rate_variance_base")
    };
    std::optional<double> driveErpmLiveTarget;
    QSet<QString> restartNodes;
    QJsonArray liveNav2Changes;
    QJsonArray liveEscTuningChanges;
    bool hasRuntimeTarget = false;
    for (const QJsonValue &value : changes) {
      if (!value.isObject()) continue;
      const QJsonObject change = value.toObject();
      const QString fileKey = change.value("file_key").toString();
      const QString path = change.value("path").toString();
      const auto targets = runtimeTargetsForChange(fileKey, path);
      if (!targets.isEmpty()) hasRuntimeTarget = true;
      if (fileKey == QStringLiteral("esc") && path == driveErpmPath) {
        const QJsonValue expected = expectedRuntimeValue(fileKey, path);
        if (expected.isDouble()) driveErpmLiveTarget = expected.toDouble();
        continue;
      }
      // BAB IV Candidate A/B only tune ESC odometry covariance. Restarting
      // esc_ackermann for those two estimation parameters would unnecessarily
      // reset the serial/teleop actuator path. Apply them live and verify so
      // motor control, steering mapping and joystick behavior stay untouched.
      if (fileKey == QStringLiteral("esc") && liveEscTuningPaths.contains(path)) {
        liveEscTuningChanges.append(change);
        continue;
      }
      // autonomous.launch.py materializes nav2_ackermann.yaml into a /tmp
      // vehicle-SSOT file once at launch. Killing planner/controller here would
      // respawn them from that stale temp file and undo a freshly written
      // Candidate A/B value. Nav2 scalar tuning is dynamically settable, so
      // apply it live and verify instead of restarting these lifecycle nodes.
      if (fileKey == QStringLiteral("nav2")) {
        liveNav2Changes.append(change);
        continue;
      }
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
    const bool hasLiveNav2 = !liveNav2Changes.isEmpty();
    const bool hasLiveEscTuning = !liveEscTuningChanges.isEmpty();
    if (driveErpmLiveTarget.has_value() && hasLiveNav2 && hasLiveEscTuning)
      result["mode"] = "safe_batch_restart_plus_live_nav2_plus_live_esc_cov_plus_drive_scale";
    else if (hasLiveNav2 && hasLiveEscTuning)
      result["mode"] = "safe_batch_restart_plus_live_nav2_plus_live_esc_cov";
    else if (hasLiveNav2)
      result["mode"] = "safe_batch_restart_plus_live_nav2";
    else if (hasLiveEscTuning)
      result["mode"] = "safe_batch_restart_plus_live_esc_cov";
    else if (driveErpmLiveTarget.has_value())
      result["mode"] = "safe_batch_restart_plus_live_drive_scale";
    else
      result["mode"] = "safe_batch_restart";
    result["restart"] = restartResults;
    result["signaled_processes"] = signaled;
    if (signaled == 0 && !restartNodes.isEmpty() && !driveErpmLiveTarget.has_value()) {
      result["status"] = "NEXT_START";
      result["message"] = "Node target tidak sedang berjalan; seluruh YAML akan aktif pada start berikutnya.";
      result["runtime_match"] = false;
      update("runtime_config_apply", result);
      return result;
    }

    if (signaled > 0) std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    if (hasLiveNav2) {
      QJsonArray liveResults;
      bool liveOk = true;
      QString liveError;
      for (const QJsonValue &value : liveNav2Changes) {
        const QJsonObject change = value.toObject();
        const QString fileKey = change.value("file_key").toString();
        const QString path = change.value("path").toString();
        const QString parameterName = parameterNameForYamlPath(path);
        const QJsonValue expected = expectedRuntimeValue(fileKey, path);
        const auto targets = runtimeTargetsForChange(fileKey, path);
        for (const auto &target : targets) {
          QString oneMessage;
          const bool oneOk = setRuntimeScalarParameter(target.second, parameterName, expected, &oneMessage);
          liveResults.append(QJsonObject{{"file_key",fileKey},{"path",path},{"node",target.second},
            {"parameter",parameterName},{"expected",expected},{"ok",oneOk},{"message",oneMessage}});
          if (!oneOk) {
            liveOk = false;
            liveError = oneMessage;
            break;
          }
        }
        if (!liveOk) break;
      }
      result["nav2_live_apply"] = liveResults;
      if (!liveOk) {
        result["status"] = "RUNTIME_MISMATCH";
        result["message"] = QStringLiteral("Nav2 live apply gagal: ") + liveError;
        result["runtime_match"] = false;
        update("runtime_config_apply", result);
        return result;
      }
    }
    if (hasLiveEscTuning) {
      QJsonArray liveResults;
      bool liveOk = true;
      QString liveError;
      for (const QJsonValue &value : liveEscTuningChanges) {
        const QJsonObject change = value.toObject();
        const QString path = change.value("path").toString();
        const QString parameterName = parameterNameForYamlPath(path);
        const QJsonValue expected = expectedRuntimeValue(QStringLiteral("esc"), path);
        QString oneMessage;
        const bool oneOk = setRuntimeScalarParameter(
          QStringLiteral("/esc_ackermann"), parameterName, expected, &oneMessage);
        liveResults.append(QJsonObject{{"file_key","esc"},{"path",path},{"node","/esc_ackermann"},
          {"parameter",parameterName},{"expected",expected},{"ok",oneOk},{"message",oneMessage}});
        if (!oneOk) {
          liveOk = false;
          liveError = oneMessage;
          break;
        }
      }
      result["esc_cov_live_apply"] = liveResults;
      if (!liveOk) {
        result["status"] = "RUNTIME_MISMATCH";
        result["message"] = QStringLiteral("ESC covariance live apply gagal: ") + liveError;
        result["runtime_match"] = false;
        update("runtime_config_apply", result);
        return result;
      }
    }
    if (driveErpmLiveTarget.has_value()) {
      QString liveMessage;
      const bool liveOk = setDriveErpmPerMps(*driveErpmLiveTarget, &liveMessage);
      result["drive_erpm_live_apply"] = QJsonObject{{"ok",liveOk},{"value",*driveErpmLiveTarget},{"message",liveMessage}};
      if (!liveOk) {
        result["status"] = "NEXT_START";
        result["message"] = QStringLiteral("YAML tersimpan, tetapi eRPM/(m/s) belum dapat diaktifkan live: ") + liveMessage;
        result["runtime_match"] = false;
        update("runtime_config_apply", result);
        return result;
      }
    }
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

  void publishFinalGuard(const QString &mode,const QString &condition) {
    std_msgs::msg::String m;m.data=QJsonDocument(QJsonObject{{"mode",mode},{"condition",condition}}).toJson(QJsonDocument::Compact).toStdString();finalGuardPub_->publish(m);
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

  bool clearGoalVisualization(QString *message) {
    update("goal_pose", QJsonObject{});
    update("nav_path", QJsonObject{{"frame_id", "map"}, {"points", QJsonArray{}}, {"length_m", 0.0}});
    update("local_path", QJsonObject{{"points", QJsonArray{}}});
    update("web_action", QJsonObject{{"ok", true}, {"action", "clear_goal_visual"}, {"at_ms", nowMs()}});
    if (message) *message = "Marker goal dan path visual dibersihkan";
    return true;
  }

  bool cancelNavigation(QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!navCancelClient_ || !navCancelClient_->service_is_ready()) {
      if (message) *message = "Service cancel Nav2 belum tersedia";
      return false;
    }
    auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    auto client = navCancelClient_;
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
    clearGoalVisualization(nullptr);
    if (message) *message = "Permintaan cancel dikirim ke Nav2; marker/path dibersihkan";
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

  bool setPerceptionLanePreview(
      double topY, double bottomY,
      double leftTopX, double leftTopY, double leftBottomX, double leftBottomY,
      double rightTopX, double rightTopY, double rightBottomX, double rightBottomY,
      QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    const auto interpX = [](double y, double x0, double y0, double x1, double y1) {
      const double t = std::clamp((y - y0) / std::max(1.0e-9, y1 - y0), 0.0, 1.0);
      return x0 + t * (x1 - x0);
    };
    const double overlapTop = std::max(leftTopY, rightTopY);
    const double overlapBottom = std::min(leftBottomY, rightBottomY);
    const bool finite = std::isfinite(topY) && std::isfinite(bottomY) &&
      std::isfinite(leftTopX) && std::isfinite(leftTopY) &&
      std::isfinite(leftBottomX) && std::isfinite(leftBottomY) &&
      std::isfinite(rightTopX) && std::isfinite(rightTopY) &&
      std::isfinite(rightBottomX) && std::isfinite(rightBottomY);
    const bool ranges = leftTopY >= 0.0 && leftTopY <= 0.95 &&
      leftBottomY >= leftTopY + 0.05 && leftBottomY <= 0.995 &&
      rightTopY >= 0.0 && rightTopY <= 0.95 &&
      rightBottomY >= rightTopY + 0.05 && rightBottomY <= 0.995 &&
      overlapBottom >= overlapTop + 0.05 &&
      leftTopX >= 0.0 && leftTopX <= 1.0 && leftBottomX >= 0.0 && leftBottomX <= 1.0 &&
      rightTopX >= 0.0 && rightTopX <= 1.0 && rightBottomX >= 0.0 && rightBottomX <= 1.0;
    const bool separated = finite && ranges &&
      interpX(overlapTop, leftTopX, leftTopY, leftBottomX, leftBottomY) + 0.005 <
        interpX(overlapTop, rightTopX, rightTopY, rightBottomX, rightBottomY) &&
      interpX(overlapBottom, leftTopX, leftTopY, leftBottomX, leftBottomY) + 0.005 <
        interpX(overlapBottom, rightTopX, rightTopY, rightBottomX, rightBottomY);
    const bool valid = finite && ranges && separated &&
      std::abs(topY - overlapTop) <= 1.0e-6 && std::abs(bottomY - overlapBottom) <= 1.0e-6;
    if (!valid) {
      if (message) *message = "Lane preview geometry tidak valid";
      return false;
    }
    const double speed = scalarState("esc_drive_actual").value_or(0.0);
    if (std::abs(speed) > 0.03) {
      if (message) *message = QString("Lane preview ditolak: kendaraan masih bergerak (%1 m/s)").arg(speed, 0, 'f', 3);
      return false;
    }
    auto client = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
      "/perception/set_parameters_atomically");
    if (!client->wait_for_service(300ms)) {
      if (message) *message = "Node /perception belum tersedia untuk lane preview";
      return false;
    }
    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    const auto addDouble = [&request](const char *name, double value) {
      rcl_interfaces::msg::Parameter p;
      p.name = name;
      p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      p.value.double_value = value;
      request->parameters.push_back(std::move(p));
    };
    addDouble("lane_corridor_top_y_ratio", topY);
    addDouble("lane_corridor_bottom_y_ratio", bottomY);
    addDouble("lane_corridor_left_top_x_ratio", leftTopX);
    addDouble("lane_corridor_left_top_y_ratio", leftTopY);
    addDouble("lane_corridor_left_bottom_x_ratio", leftBottomX);
    addDouble("lane_corridor_left_bottom_y_ratio", leftBottomY);
    addDouble("lane_corridor_right_top_x_ratio", rightTopX);
    addDouble("lane_corridor_right_top_y_ratio", rightTopY);
    addDouble("lane_corridor_right_bottom_x_ratio", rightBottomX);
    addDouble("lane_corridor_right_bottom_y_ratio", rightBottomY);
    client->async_send_request(request,
      [this, client](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
        try {
          const auto response = future.get();
          update("web_action", QJsonObject{{"ok", response->result.successful},
            {"action", "perception_lane_preview"},
            {"message", QString::fromStdString(response->result.reason)}, {"at_ms", nowMs()}});
        } catch (const std::exception &e) {
          update("web_action", QJsonObject{{"ok", false}, {"action", "perception_lane_preview"},
            {"message", QString::fromUtf8(e.what())}, {"at_ms", nowMs()}});
        }
      });
    if (message) *message = "Lane Safety runtime preview diperbarui; YAML belum berubah";
    return true;
  }

  bool setDriveErpmPerMps(double erpmPerMps, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!std::isfinite(erpmPerMps) || erpmPerMps < 100.0 || erpmPerMps > 50000.0) {
      if (message) *message = "drive_erpm_per_mps harus finite dan 100..50000 eRPM/(m/s)";
      return false;
    }
    const double speed = scalarState("esc_drive_actual").value_or(0.0);
    if (std::abs(speed) > 0.03) {
      if (message) *message = QString("Apply eRPM/m/s ditolak: kendaraan masih bergerak (%1 m/s)").arg(speed,0,'f',3);
      return false;
    }
    auto client=node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
      "/esc_ackermann/set_parameters_atomically");
    if (!client->wait_for_service(700ms)) {
      if (message) *message = "Parameter service /esc_ackermann belum tersedia";
      return false;
    }
    auto req=std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    rcl_interfaces::msg::Parameter p; p.name="drive_erpm_per_mps";
    p.value.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE; p.value.double_value=erpmPerMps;
    req->parameters.push_back(p);
    auto future=client->async_send_request(req);
    if (future.wait_for(1200ms)!=std::future_status::ready) {
      if (message) *message = "Timeout apply eRPM/m/s ke runtime";
      return false;
    }
    try {
      const auto result=future.get();
      if (!result->result.successful) { if(message)*message=QString::fromStdString(result->result.reason); return false; }
    } catch (const std::exception &e) { if(message)*message=QString::fromUtf8(e.what()); return false; }
    if (message) *message=QString("Drive calibration %1 eRPM/(m/s) aktif live").arg(erpmPerMps,0,'f',3);
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
    if (active && steerCal && !sourceConfigBool("vehicle", "vehicle.ros__parameters.drive_odometry_calibration_valid", false)) { if (message) *message = "Trial steering ditolak: drive odometry scale N2.1 belum certified"; return false; }
    const double erpmPerMps = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.drive_erpm_per_mps", 8000.0);
    const double speedMax = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.speed_max", 0.5);
    const double yawMaxDeg = sourceConfigNumber("teleop", "motor_teleop.ros__parameters.yaw_max_deg_s", 80.0);
    const double steerLimit = sourceConfigNumber("esc", "esc_ackermann.ros__parameters.steering_physical_operational_limit_deg", 28.0);
    const double speed = erpm / std::max(100.0, erpmPerMps);
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

  bool publishFinalTrialVelocity(double speedMps, double yawRateRad, bool active, QString *message) {
    if (readOnly_) return rejectReadOnly(message);
    if (!trialTwistPub_ || !trialSourcePub_) { if(message)*message="Final trial publisher belum siap"; return false; }
    if (!std::isfinite(speedMps) || !std::isfinite(yawRateRad)) { if(message)*message="Final trial command tidak finite"; return false; }
    if (active) {
      if (boolState("system.estop")) { if(message)*message="Final trial ditolak: E-stop aktif"; return false; }
      if (boolState("vesc_maintenance_active")) { if(message)*message="Final trial ditolak: maintenance aktif"; return false; }
      if (!boolState("connected.esc_feedback") || !boolState("connected.esc_ready")) {
        if(message) *message="Final trial ditolak: ESC feedback/ready belum siap";
        return false;
      }
      if (std::abs(speedMps) > 0.22 + 1.0e-9 || std::abs(yawRateRad) > 0.18 + 1.0e-9) {
        if(message)*message=QString("Final trial command melebihi limit: v=%1 w=%2").arg(speedMps,0,'f',3).arg(yawRateRad,0,'f',3);
        return false;
      }
    }

    geometry_msgs::msg::Twist cmd;
    double steeringDeg=0.0, encodedAngular=0.0;
    if (active) {
      cmd.linear.x=speedMps;
      const double wheelbase=sourceConfigNumber("vehicle","vehicle.ros__parameters.effective_wheelbase_m",0.70);
      const double track=sourceConfigNumber("vehicle","vehicle.ros__parameters.track_width_m",0.48);
      const double steerLimit=sourceConfigNumber("esc","esc_ackermann.ros__parameters.steering_physical_operational_limit_deg",28.0);
      const double yawMaxDeg=sourceConfigNumber("teleop","motor_teleop.ros__parameters.yaw_max_deg_s",80.0);
      if(std::abs(speedMps)>=0.05&&std::abs(yawRateRad)>1.0e-6){
        const double kappa=yawRateRad/speedMps;
        const double centerRadius=1.0/std::abs(kappa);
        const double innerRadius=std::max(1.0e-6,centerRadius-0.5*track);
        const double innerAngle=std::atan(wheelbase/innerRadius);
        // ROS +yaw = LEFT, physical steering +deg = RIGHT.
        steeringDeg=-std::copysign(innerAngle,kappa)*180.0/kPi;
        steeringDeg=std::clamp(steeringDeg,-steerLimit,steerLimit);
      }
      const double frac=steerLimit>1.0e-9?steeringDeg/steerLimit:0.0;
      encodedAngular=std::clamp(frac,-1.0,1.0)*yawMaxDeg*kPi/180.0;
      cmd.angular.z=encodedAngular;
    }
    std_msgs::msg::String src; src.data=active ? "WEB_TRIAL" : "STOP";
    trialTwistPub_->publish(cmd); trialSourcePub_->publish(src);
    update("web_trial_motion",QJsonObject{{"active",active},{"speed_mps",active?speedMps:0.0},
      {"requested_yaw_rate_rad_s",active?yawRateRad:0.0},{"steering_deg",active?steeringDeg:0.0},
      {"encoded_manual_angular_z",active?encodedAngular:0.0},{"at_ms",nowMs()}});
    if(message)*message=active ? QString("FINAL WEB_TRIAL v=%1 m/s steer=%2 deg (w_req=%3)")
      .arg(speedMps,0,'f',3).arg(steeringDeg,0,'f',2).arg(yawRateRad,0,'f',3) : "FINAL WEB_TRIAL stop";
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
  bool cameraFullresFallbackEnabled_{false};
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
  nav_msgs::msg::OccupancyGrid localCostmapGrid_{};
  bool localCostmapGridReady_{false};
  std::mutex costmapAlignmentMutex_;
  double alignmentOdomX_{0.0};
  double alignmentOdomY_{0.0};
  double alignmentOdomYaw_{0.0};
  bool alignmentOdomReady_{false};
  std::chrono::steady_clock::time_point alignmentOdomSeen_{};
  std::vector<std::pair<double,double>> latestPlanningPoints_{};
  std::string latestPlanningFrame_{};
  std::chrono::steady_clock::time_point latestPlanningSeen_{};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::TimerBase::SharedPtr tfTimer_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr navCancelClient_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vescToolCommandPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPosePub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hmiRequestPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr finalGuardPub_;
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
    else if (fileKey == QStringLiteral("navigation_core")) {
      if (path.startsWith(QStringLiteral("cmd_vel_router."))) add(QStringLiteral("cmd_vel_router"),QStringLiteral("/cmd_vel_router"));
      else if (path.startsWith(QStringLiteral("sensor_contract_monitor."))) add(QStringLiteral("sensor_contract_monitor"),QStringLiteral("/sensor_contract_monitor"));
      else if (path.startsWith(QStringLiteral("joint_state_visualizer."))) add(QStringLiteral("joint_state_visualizer"),QStringLiteral("/joint_state_visualizer"));
      else if (path.startsWith(QStringLiteral("agv_web_gui."))) add(QStringLiteral("agv_web_gui"),QStringLiteral("/agv_web_gui"));
      else add(QStringLiteral("navigation_core"),QStringLiteral("/navigation_core"));
    }
    else if (fileKey == QStringLiteral("trajectory_safety")) add(QStringLiteral("trajectory_safety_supervisor"),QStringLiteral("/trajectory_safety_supervisor"));
    else if (fileKey == QStringLiteral("perception")) {
      if (path.startsWith(QStringLiteral("semantic_obstacle."))) add(QStringLiteral("semantic_obstacle"),QStringLiteral("/semantic_obstacle"));
      else if (path.startsWith(QStringLiteral("bbox_static_test_logger."))) add(QStringLiteral("bbox_static_test_logger"),QStringLiteral("/bbox_static_test_logger"));
      else add(QStringLiteral("perception"),QStringLiteral("/perception"));
    }
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

  bool setRuntimeScalarParameter(const QString &nodeName, const QString &parameterName,
                               const QJsonValue &expected, QString *message) {
    if (parameterName.isEmpty() || expected.isUndefined() || expected.isArray() || expected.isObject()) {
      if (message) *message = QStringLiteral("Runtime scalar parameter tidak valid");
      return false;
    }
    auto getClient=node_->create_client<rcl_interfaces::srv::GetParameters>(
      (nodeName+QStringLiteral("/get_parameters")).toStdString());
    if(!getClient->wait_for_service(1200ms)){
      if(message)*message=QStringLiteral("GetParameters unavailable: ")+nodeName;
      return false;
    }
    auto getReq=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    getReq->names.push_back(parameterName.toStdString());
    auto getFuture=getClient->async_send_request(getReq);
    if(getFuture.wait_for(1500ms)!=std::future_status::ready){
      if(message)*message=QStringLiteral("Timeout membaca tipe runtime: ")+nodeName+QStringLiteral(" ")+parameterName;
      return false;
    }
    rcl_interfaces::msg::ParameterValue current;
    try{
      const auto response=getFuture.get();
      if(response->values.empty()){
        if(message)*message=QStringLiteral("Parameter runtime tidak ditemukan: ")+parameterName;
        return false;
      }
      current=response->values.front();
    }catch(const std::exception &e){
      if(message)*message=QString::fromUtf8(e.what());
      return false;
    }

    rcl_interfaces::msg::Parameter p;
    p.name=parameterName.toStdString();
    p.value.type=current.type;
    using PT=rcl_interfaces::msg::ParameterType;
    switch(current.type){
      case PT::PARAMETER_BOOL:
        if(!expected.isBool()){if(message)*message=QStringLiteral("Tipe bool runtime tidak cocok");return false;}
        p.value.bool_value=expected.toBool(); break;
      case PT::PARAMETER_INTEGER:
        if(!expected.isDouble()){if(message)*message=QStringLiteral("Tipe integer runtime tidak cocok");return false;}
        p.value.integer_value=static_cast<int64_t>(std::llround(expected.toDouble())); break;
      case PT::PARAMETER_DOUBLE:
        if(!expected.isDouble()){if(message)*message=QStringLiteral("Tipe double runtime tidak cocok");return false;}
        p.value.double_value=expected.toDouble(); break;
      case PT::PARAMETER_STRING:
        if(!expected.isString()){if(message)*message=QStringLiteral("Tipe string runtime tidak cocok");return false;}
        p.value.string_value=expected.toString().toStdString(); break;
      default:
        if(message)*message=QStringLiteral("Hanya scalar ROS parameter yang didukung untuk live apply");
        return false;
    }

    auto setClient=node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
      (nodeName+QStringLiteral("/set_parameters_atomically")).toStdString());
    if(!setClient->wait_for_service(1200ms)){
      if(message)*message=QStringLiteral("SetParametersAtomically unavailable: ")+nodeName;
      return false;
    }
    auto req=std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    req->parameters.push_back(p);
    auto future=setClient->async_send_request(req);
    if(future.wait_for(1800ms)!=std::future_status::ready){
      if(message)*message=QStringLiteral("Timeout live apply: ")+nodeName+QStringLiteral(" ")+parameterName;
      return false;
    }
    try{
      const auto response=future.get();
      if(!response->result.successful){
        if(message)*message=QString::fromStdString(response->result.reason);
        return false;
      }
    }catch(const std::exception &e){
      if(message)*message=QString::fromUtf8(e.what());
      return false;
    }
    const QJsonObject verify=readRuntimeParameter(nodeName,parameterName,expected);
    if(!verify.value(QStringLiteral("match")).toBool(false)){
      if(message)*message=QStringLiteral("Live apply selesai tetapi readback mismatch: ")+nodeName+QStringLiteral(" ")+parameterName;
      return false;
    }
    if(message)*message=QStringLiteral("ACTIVE_MATCH");
    return true;
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
        {"/neo3pro/rm3100_connected", "connected.rm3100"},
        {"/neo3pro/mag_heading_valid", "rm3100_heading_valid"},
        {"/neo3/safety_switch", "neo3_safety_switch"},
        {"/hmi/connected", "connected.hmi"},
        {"/teleop/joystick_connected", "connected.joystick"},
        {"/perception/camera_connected", "connected.camera"}, {"/esc/ready", "connected.esc_ready"},
        {"/esc/armed", "connected.esc_armed"}, {"/esc/feedback_valid", "connected.esc_feedback"},
        {"/esc/drive/connected", "connected.esc_drive"}, {"/esc/steer/connected", "connected.esc_steer"},
        {"/esc/steering_calibration/controller_ready", "esc_calibration_controller_ready"},
        {"/system/autonomy_ready", "system.autonomy_ready"}, {"/system/autonomy_motion_allowed", "system.autonomy_motion_allowed"},
        {"/system/motion_ready", "system.motion_ready"},
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
    // E-stop publisher is TRANSIENT_LOCAL. Use the same durability so a
    // restarted web node immediately receives the latched safety state.
    subscribe<std_msgs::msg::Bool>("/safety/estop", latchedQos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      update("system.estop", msg->data);
    });

    subscribe<std_msgs::msg::Bool>("/esc/feedback_valid", latchedQos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      // Canonical production transport is the dedicated USB-UART owned by package esc.
      // A fresh F103 ACK proves the direct native-VESC transport; F411 is not in this path.
      update("connected.vesc_transport", msg->data);
    });
    subscribe<std_msgs::msg::Bool>("/esc/vesc/maintenance_active", latchedQos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      update("vesc_maintenance_active", msg->data);
    });

    const std::vector<std::pair<const char *, const char *>> strings = {
        {"/system/localization_state", "localization_state"}, {"/system/gnss_status", "gnss_status"},
        {"/gnss/state", "gnss_driver_state"}, {"/neo3/status", "neo3_status"},
        {"/neo3pro/param", "neo3_param"},
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
        {"/esc/status", "vesc_transport_status"}, {"/esc/vesc/tool_status", "vesc_tool_status"},
        {"/esc/vesc/tool_telemetry", "vesc_tool_telemetry"}, {"/esc/vesc/left_values", "vesc_left_values"}, {"/esc/vesc/right_values", "vesc_right_values"}, {"/esc/vesc/config_state", "vesc_config_state"},
        {"/esc/vesc/tuning_state", "vesc_tuning_state"}, {"/esc/vesc/position_state", "vesc_position_state"},
        {"/esc/vesc/steering_state", "vesc_steering_state"}, {"/esc/vesc/rotor_state", "vesc_rotor_state"},
        {"/esc/vesc/left_rotor_state", "vesc_left_rotor_state"}, {"/esc/vesc/right_rotor_state", "vesc_right_rotor_state"},
        {"/esc/vesc/command_state", "vesc_command_state"}, {"/esc/vesc/detect_state", "vesc_detect_state"},
        {"/esc/vesc/raw_reply", "vesc_raw_reply"}};
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
        else if (channel == "neo3_param") {
          const QStringList parts = raw.split(',');
          QJsonObject one{{"raw",raw}};
          if (parts.size() >= 4) {
            const QString name=parts.at(2).trimmed(), text=parts.at(3).trimmed();
            bool numberOk=false; const double number=text.toDouble(&numberOk);
            one["name"]=name; one["value_text"]=text; if(numberOk) one["value"]=number;
            if (name.startsWith(QStringLiteral("COMPASS_"))) {
              static QJsonObject compassParams;
              compassParams[name]=numberOk?QJsonValue(number):QJsonValue(text);
              compassParams["last_name"]=name; compassParams["updated_at_ms"]=QDateTime::currentMSecsSinceEpoch();
              update("neo3_compass_params",compassParams);
            }
          }
          update(channel,one);
        }
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
    // RM3100 on NEO3 Pro is the only magnetometer stream used by ROS.
    magSubscribe("/neo3pro/mag", "rm3100_mag");
    magSubscribe("/neo3pro/mag_calibrated", "rm3100_mag_calibrated");

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
    magneticHeadingSubscribe("/neo3pro/mag_heading_fusion", "rm3100_heading");
    magneticHeadingSubscribe("/imu/inertial_heading", "imu_inertial_heading");
    magneticHeadingSubscribe("/heading/validated_fusion", "validated_heading");
    subscribe<std_msgs::msg::Float64>("/localization/map_yaw_from_enu", stateQos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        update("map_yaw_from_enu", msg->data);
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
        const auto &lv = msg->twist.twist.linear;
        const auto &av = msg->twist.twist.angular;
        const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
        const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        const double roll = std::atan2(sinr, cosr);
        const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);
        const double yaw = yawFromQuat(q.x, q.y, q.z, q.w);
        if (ch == QStringLiteral("ekf_local")) {
          std::lock_guard<std::mutex> alignLock(costmapAlignmentMutex_);
          alignmentOdomX_ = p.x;
          alignmentOdomY_ = p.y;
          alignmentOdomYaw_ = yaw;
          alignmentOdomReady_ = true;
          alignmentOdomSeen_ = std::chrono::steady_clock::now();
        }
        QJsonArray poseCovariance, twistCovariance;
        for (const double value : msg->pose.covariance) poseCovariance.append(value);
        for (const double value : msg->twist.covariance) twistCovariance.append(value);
        update(ch, QJsonObject{
          // Flat aliases are retained for map/render code and existing exports.
          {"x", p.x}, {"y", p.y}, {"z", p.z}, {"roll", roll}, {"pitch", pitch}, {"yaw", yaw},
          {"v", lv.x}, {"w", av.z},
          {"vx", lv.x}, {"vy", lv.y}, {"vz", lv.z}, {"wx", av.x}, {"wy", av.y}, {"wz", av.z},
          {"var_x", msg->pose.covariance[0]}, {"var_y", msg->pose.covariance[7]}, {"var_z", msg->pose.covariance[14]},
          {"var_roll", msg->pose.covariance[21]}, {"var_pitch", msg->pose.covariance[28]}, {"var_yaw", msg->pose.covariance[35]},
          {"var_v", msg->twist.covariance[0]}, {"var_w", msg->twist.covariance[35]},
          {"frame_id", QString::fromStdString(msg->header.frame_id)},
          {"child_frame_id", QString::fromStdString(msg->child_frame_id)},
          {"position", QJsonObject{{"x", p.x}, {"y", p.y}, {"z", p.z}}},
          {"orientation", QJsonObject{{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w},
                                      {"roll_rad", roll}, {"pitch_rad", pitch}, {"yaw_rad", yaw}}},
          {"linear_velocity", QJsonObject{{"x", lv.x}, {"y", lv.y}, {"z", lv.z}}},
          {"angular_velocity", QJsonObject{{"x", av.x}, {"y", av.y}, {"z", av.z}}},
          {"pose_covariance", poseCovariance}, {"twist_covariance", twistCovariance},
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
    // Humble MPPI publishes the controller-trimmed path on /transformed_global_plan.
    // Do not subscribe dead aliases into the same local_path state key.
    pathSubscribe("/transformed_global_plan", "local_path");

    // Nav2 Humble MPPI TrajectoryVisualizer publishes a MarkerArray on the
    // controller node's relative `trajectories` topic.  Bridge candidate and
    // optimal trajectories so the browser map can show the same control output
    // that is normally inspected in RViz.
    subscribe<visualization_msgs::msg::MarkerArray>(
      "/trajectories", sensorQos,
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        // Nav2 Humble MPPI TrajectoryVisualizer emits one Marker per sampled
        // trajectory point using marker.pose.position (marker.points is empty).
        // Keep compatibility with LINE_STRIP/POINTS style marker.points too.
        QJsonArray trajectories;
        size_t pointCount = 0;
        size_t optimalCount = 0;
        size_t rawMarkerCount = 0;
        size_t poseMarkerCount = 0;
        size_t pointsMarkerCount = 0;

        QJsonArray candidatePosePoints;
        QJsonArray optimalPosePoints;
        QString candidateFrame;
        QString optimalFrame;
        int candidateFirstId = -1;
        int optimalFirstId = -1;
        double lastCandidateGreen = -1.0;

        auto flushCandidate = [&]() {
          if (candidatePosePoints.size() >= 2) {
            pointCount += static_cast<size_t>(candidatePosePoints.size());
            trajectories.append(QJsonObject{
              {"ns", QStringLiteral("Candidate Trajectories")},
              {"id", candidateFirstId},
              {"frame_id", candidateFrame},
              {"optimal", false},
              {"points", candidatePosePoints}});
          }
          candidatePosePoints = QJsonArray();
          candidateFrame.clear();
          candidateFirstId = -1;
          lastCandidateGreen = -1.0;
        };

        for (const auto &marker : msg->markers) {
          if (marker.action == visualization_msgs::msg::Marker::DELETE ||
              marker.action == visualization_msgs::msg::Marker::DELETEALL) continue;
          ++rawMarkerCount;
          const QString ns = QString::fromStdString(marker.ns);
          const bool optimal = ns.contains(QStringLiteral("Optimal"), Qt::CaseInsensitive);

          if (!marker.points.empty()) {
            ++pointsMarkerCount;
            QJsonArray points;
            const size_t stride = std::max<size_t>(1, marker.points.size() / 160 + 1);
            for (size_t i = 0; i < marker.points.size(); ++i) {
              if (i % stride != 0 && i + 1 != marker.points.size()) continue;
              const auto &p = marker.points[i];
              if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
              points.append(QJsonArray{p.x, p.y, p.z});
            }
            if (points.size() < 2) continue;
            if (optimal) ++optimalCount;
            pointCount += static_cast<size_t>(points.size());
            trajectories.append(QJsonObject{
              {"ns", ns}, {"id", marker.id}, {"frame_id", QString::fromStdString(marker.header.frame_id)},
              {"optimal", optimal}, {"points", points}});
            continue;
          }

          const auto &p = marker.pose.position;
          if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
          ++poseMarkerCount;
          const QJsonArray point{p.x, p.y, p.z};

          if (optimal) {
            if (optimalPosePoints.isEmpty()) {
              optimalFrame = QString::fromStdString(marker.header.frame_id);
              optimalFirstId = marker.id;
            }
            optimalPosePoints.append(point);
            continue;
          }

          if (ns.contains(QStringLiteral("Candidate"), Qt::CaseInsensitive)) {
            const double green = marker.color.g;
            // Humble encodes progress along each candidate in color.g. It
            // rises along a trajectory and resets to 0 for the next one.
            if (!candidatePosePoints.isEmpty() && lastCandidateGreen >= 0.0 &&
                green + 1.0e-6 < lastCandidateGreen) {
              flushCandidate();
            }
            if (candidatePosePoints.isEmpty()) {
              candidateFrame = QString::fromStdString(marker.header.frame_id);
              candidateFirstId = marker.id;
            }
            candidatePosePoints.append(point);
            lastCandidateGreen = green;
          }
        }

        flushCandidate();
        if (optimalPosePoints.size() >= 2) {
          ++optimalCount;
          pointCount += static_cast<size_t>(optimalPosePoints.size());
          trajectories.append(QJsonObject{
            {"ns", QStringLiteral("Optimal Trajectory")},
            {"id", optimalFirstId},
            {"frame_id", optimalFrame},
            {"optimal", true},
            {"points", optimalPosePoints}});
        }

        update("mppi_trajectories", QJsonObject{
          {"count", trajectories.size()},
          {"optimal_count", static_cast<double>(optimalCount)},
          {"point_count", static_cast<double>(pointCount)},
          {"raw_marker_count", static_cast<double>(rawMarkerCount)},
          {"pose_marker_count", static_cast<double>(poseMarkerCount)},
          {"points_marker_count", static_cast<double>(pointsMarkerCount)},
          {"format", poseMarkerCount > 0 ? QStringLiteral("POSE_MARKERS_HUMBLE") : QStringLiteral("MARKER_POINTS")},
          {"trajectories", trajectories}});
      });

    subscribe<geometry_msgs::msg::PoseStamped>("/navigation/goal_request", stateQos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        const auto &q = msg->pose.orientation;
        const QJsonObject goal{{"x", msg->pose.position.x}, {"y", msg->pose.position.y},
                               {"yaw", yawFromQuat(q.x, q.y, q.z, q.w)}, {"at_ms", nowMs()}};
        update("goal_pose", goal);
        // Dedicated edge event used by the HTTP recorder fallback. This fires for
        // WEB, STM32 HMI, or any other publisher on /navigation/goal_request.
        update("goal_request_event", goal);
      });
    subscribe<geometry_msgs::msg::PoseStamped>("/goal_pose", stateQos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        const auto &q = msg->pose.orientation;
        update("goal_pose", QJsonObject{{"x", msg->pose.position.x}, {"y", msg->pose.position.y},
                                        {"yaw", yawFromQuat(q.x, q.y, q.z, q.w)}, {"at_ms", nowMs()}});
      });

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
    if (cameraFullresFallbackEnabled_) {
      subscribe<sensor_msgs::msg::Image>("/camera/astra/image_raw", sensorQos,
                                        [cacheCamera](sensor_msgs::msg::Image::ConstSharedPtr msg) { cacheCamera(msg, "raw"); });
      subscribe<sensor_msgs::msg::Image>("/camera/yolop/image_annotated", sensorQos,
                                        [cacheCamera](sensor_msgs::msg::Image::ConstSharedPtr msg) { cacheCamera(msg, "annotated"); });
    }

    subscribe<nav_msgs::msg::OccupancyGrid>("/map", latchedQos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) return;
      const std::uint64_t cellCount = static_cast<std::uint64_t>(msg->info.width) * static_cast<std::uint64_t>(msg->info.height);
      if (cellCount > msg->data.size() || cellCount > 100000000ULL ||
          msg->info.width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
          msg->info.height > static_cast<unsigned int>(std::numeric_limits<int>::max())) return;
      // The static map may be several thousand pixels wide. Serving the full
      // raster made Chromium decode tens of millions of pixels on every retry,
      // which could leave the map placeholder visible and make the HMI appear
      // blocked even though /map metadata was healthy. Downsample only the Web
      // preview; all world geometry continues to use the original map metadata.
      constexpr unsigned int kMaxMapRaster = 1600U;
      const unsigned int largest = std::max(msg->info.width, msg->info.height);
      const unsigned int stride = std::max(1U, (largest + kMaxMapRaster - 1U) / kMaxMapRaster);
      const unsigned int outW = (msg->info.width + stride - 1U) / stride;
      const unsigned int outH = (msg->info.height + stride - 1U) / stride;
      QImage image(static_cast<int>(outW), static_cast<int>(outH), QImage::Format_Grayscale8);
      if (image.isNull()) return;
      for (unsigned int oy = 0; oy < outH; ++oy) {
        uchar *row = image.scanLine(static_cast<int>(outH - 1U - oy));
        for (unsigned int ox = 0; ox < outW; ++ox) {
          int maxOcc = -1;
          bool sawUnknown = false;
          const unsigned int y0 = oy * stride, x0 = ox * stride;
          const unsigned int y1 = std::min(msg->info.height, y0 + stride);
          const unsigned int x1 = std::min(msg->info.width, x0 + stride);
          for (unsigned int y = y0; y < y1; ++y) {
            const size_t base = static_cast<size_t>(y) * msg->info.width;
            for (unsigned int x = x0; x < x1; ++x) {
              const int occ = static_cast<int>(msg->data[base + x]);
              if (occ < 0) sawUnknown = true; else maxOcc = std::max(maxOcc, occ);
            }
          }
          int shade = 118;
          if (maxOcc >= 0) shade = std::clamp(245 - maxOcc * 2, 35, 245);
          else if (!sawUnknown) shade = 245;
          row[ox] = static_cast<uchar>(shade);
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
                                     {"image_width", static_cast<int>(outW)}, {"image_height", static_cast<int>(outH)},
                                     {"stride", static_cast<int>(stride)}, {"resolution", msg->info.resolution},
                                     {"origin_x", origin.x}, {"origin_y", origin.y},
                                     {"frame_id", QString::fromStdString(msg->header.frame_id)}, {"at_ms", nowMs()}});
    });

    // RViz-like costmap layers for the Web HMI. Nav2 commonly publishes a
    // full OccupancyGrid once and then incremental OccupancyGridUpdate patches.
    // The web bridge merges those updates so 4.5 always visualizes the actual
    // rolling local costmap instead of a stale all-zero startup snapshot.
    const auto renderCostmap = [this](const nav_msgs::msg::OccupancyGrid &msg, const char *metaKey, bool global) {
      if (msg.info.width == 0 || msg.info.height == 0 || msg.data.empty()) return;
      const std::uint64_t cells = static_cast<std::uint64_t>(msg.info.width) * msg.info.height;
      if (cells > msg.data.size() || cells > 100000000ULL) return;

      std::lock_guard<std::mutex> encodeLock(costmapEncodeMutex_);
      auto &last = global ? lastGlobalCostmapEncode_ : lastLocalCostmapEncode_;
      const auto now = std::chrono::steady_clock::now();
      const double minPeriod = global ? 1.0 : 0.08;
      if (last.time_since_epoch().count() != 0 &&
          std::chrono::duration<double>(now - last).count() < minPeriod) return;

      constexpr unsigned int kMaxRaster = 1600U;
      const unsigned int largest = std::max(msg.info.width, msg.info.height);
      const unsigned int stride = std::max(1U, (largest + kMaxRaster - 1U) / kMaxRaster);
      const unsigned int outW = (msg.info.width + stride - 1U) / stride;
      const unsigned int outH = (msg.info.height + stride - 1U) / stride;
      QImage image(static_cast<int>(outW), static_cast<int>(outH), QImage::Format_RGBA8888);
      if (image.isNull()) return;
      image.fill(Qt::transparent);
      std::uint64_t nonzeroCells = 0;
      std::uint64_t lethalCells = 0;
      int maxObservedCost = 0;
      std::vector<std::pair<double,double>> lethalWorld;
      if (!global) lethalWorld.reserve(512);

      for (unsigned int oy = 0; oy < outH; ++oy) {
        uchar *row = image.scanLine(static_cast<int>(outH - 1U - oy));
        for (unsigned int ox = 0; ox < outW; ++ox) {
          int maxCost = -1;
          const unsigned int y0 = oy * stride;
          const unsigned int x0 = ox * stride;
          const unsigned int y1 = std::min(msg.info.height, y0 + stride);
          const unsigned int x1 = std::min(msg.info.width, x0 + stride);
          for (unsigned int y = y0; y < y1; ++y) {
            const size_t base = static_cast<size_t>(y) * msg.info.width;
            for (unsigned int x = x0; x < x1; ++x) {
              maxCost = std::max(maxCost, static_cast<int>(msg.data[base + x]));
            }
          }
          uchar *px = row + static_cast<size_t>(ox) * 4U;
          if (maxCost <= 0) { px[0]=0; px[1]=0; px[2]=0; px[3]=0; continue; }
          ++nonzeroCells;
          if (maxCost >= 99) {
            ++lethalCells;
            if (!global) {
              const double cxCell = 0.5 * (static_cast<double>(x0) + static_cast<double>(x1));
              const double cyCell = 0.5 * (static_cast<double>(y0) + static_cast<double>(y1));
              lethalWorld.emplace_back(
                msg.info.origin.position.x + cxCell * msg.info.resolution,
                msg.info.origin.position.y + cyCell * msg.info.resolution);
            }
          }
          maxObservedCost = std::max(maxObservedCost, maxCost);
          const int bounded = std::clamp(maxCost, 1, 100);
          const int alpha = std::clamp(42 + bounded * 2, 48, 225);
          // Make the Nav2 risk field explicit in BAB 4.5:
          // red = lethal/inscribed core, orange = high-cost inflation,
          // yellow = low-cost fringe.
          if (bounded >= 99) { px[0]=255; px[1]=64; px[2]=70; }
          else if (bounded >= 60) { px[0]=255; px[1]=153; px[2]=45; }
          else { px[0]=245; px[1]=204; px[2]=75; }
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
      const auto &origin = msg.info.origin.position;
      QJsonObject meta{
        {"width", static_cast<int>(msg.info.width)}, {"height", static_cast<int>(msg.info.height)},
        {"image_width", static_cast<int>(outW)}, {"image_height", static_cast<int>(outH)},
        {"stride", static_cast<int>(stride)}, {"resolution", msg.info.resolution},
        {"origin_x", origin.x}, {"origin_y", origin.y},
        {"nonzero_cells", static_cast<double>(nonzeroCells)},
        {"lethal_cells", static_cast<double>(lethalCells)},
        {"max_cost", maxObservedCost},
        {"source", global ? "full" : "full_or_updates"},
        {"frame_id", QString::fromStdString(msg.header.frame_id)}, {"at_ms", nowMs()}};

      if (!global) {
        std::vector<std::pair<double,double>> planning;
        std::string planningFrame;
        double odomX = 0.0, odomY = 0.0, odomYaw = 0.0;
        bool odomReady = false;
        std::chrono::steady_clock::time_point planningSeen{};
        std::chrono::steady_clock::time_point odomSeen{};
        {
          std::lock_guard<std::mutex> alignLock(costmapAlignmentMutex_);
          planning = latestPlanningPoints_;
          planningFrame = latestPlanningFrame_;
          planningSeen = latestPlanningSeen_;
          odomX = alignmentOdomX_;
          odomY = alignmentOdomY_;
          odomYaw = alignmentOdomYaw_;
          odomReady = alignmentOdomReady_;
          odomSeen = alignmentOdomSeen_;
        }
        const double planningAge =
          planningSeen.time_since_epoch().count() == 0 ? std::numeric_limits<double>::infinity() :
          std::chrono::duration<double>(now - planningSeen).count();
        const double odomAge =
          odomSeen.time_since_epoch().count() == 0 ? std::numeric_limits<double>::infinity() :
          std::chrono::duration<double>(now - odomSeen).count();
        const bool planningFresh = planningAge <= 1.2;
        const bool odomFresh = odomReady && odomAge <= 1.0;
        double maxNearest = 0.0;
        double minNearest = std::numeric_limits<double>::infinity();
        bool alignmentValid = planningFresh && !planning.empty() && odomFresh;
        if (alignmentValid) {
          for (const auto &p : planning) {
            double px = p.first;
            double py = p.second;
            if (planningFrame == "base_footprint" || planningFrame == "base_link") {
              const double c = std::cos(odomYaw), s = std::sin(odomYaw);
              const double bx = px, by = py;
              px = odomX + c * bx - s * by;
              py = odomY + s * bx + c * by;
            } else if (planningFrame != "odom") {
              alignmentValid = false;
              break;
            }
            double nearest = std::numeric_limits<double>::infinity();
            for (const auto &cell : lethalWorld) {
              nearest = std::min(nearest, std::hypot(px - cell.first, py - cell.second));
            }
            minNearest = std::min(minNearest, nearest);
            maxNearest = std::max(maxNearest, nearest);
          }
        }
        const double threshold = std::max(0.25, 2.5 * static_cast<double>(msg.info.resolution));
        const bool alignmentPass = alignmentValid && !lethalWorld.empty() &&
          std::isfinite(maxNearest) && maxNearest <= threshold;
        meta.insert("planning_alignment_valid", alignmentValid);
        meta.insert("planning_alignment_pass", alignmentPass);
        meta.insert("planning_alignment_threshold_m", threshold);
        meta.insert("planning_alignment_planning_count", static_cast<int>(planning.size()));
        meta.insert("planning_alignment_lethal_count", static_cast<int>(lethalWorld.size()));
        meta.insert("planning_alignment_max_m",
          std::isfinite(maxNearest) ? QJsonValue(maxNearest) : QJsonValue(QJsonValue::Null));
        meta.insert("planning_alignment_min_m",
          std::isfinite(minNearest) ? QJsonValue(minNearest) : QJsonValue(QJsonValue::Null));
        meta.insert("planning_alignment_age_sec",
          std::isfinite(planningAge) ? QJsonValue(planningAge) : QJsonValue(QJsonValue::Null));
      }
      update(QString::fromLatin1(metaKey), meta);
    };

    const auto cacheCostmap = [this, latchedQos, renderCostmap](const char *topic, const char *metaKey, bool global) {
      subscribe<nav_msgs::msg::OccupancyGrid>(topic, latchedQos,
        [this, metaKey, global, renderCostmap](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          if (!global) {
            localCostmapGrid_ = *msg;
            localCostmapGridReady_ = true;
          }
          renderCostmap(*msg, metaKey, global);
        });
    };
    cacheCostmap("/global_costmap/costmap", "global_costmap_meta", true);
    cacheCostmap("/local_costmap/costmap", "local_costmap_meta", false);

    const auto updateQos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    subscribe<map_msgs::msg::OccupancyGridUpdate>("/local_costmap/costmap_updates", updateQos,
      [this, renderCostmap](map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr patch) {
        if (!localCostmapGridReady_ || patch->width == 0 || patch->height == 0 || patch->data.empty()) return;
        const auto fullW = localCostmapGrid_.info.width;
        const auto fullH = localCostmapGrid_.info.height;
        if (patch->x < 0 || patch->y < 0) return;
        const unsigned int x0 = static_cast<unsigned int>(patch->x);
        const unsigned int y0 = static_cast<unsigned int>(patch->y);
        if (x0 + patch->width > fullW || y0 + patch->height > fullH) return;
        const size_t expected = static_cast<size_t>(patch->width) * patch->height;
        if (patch->data.size() < expected) return;

        for (unsigned int py = 0; py < patch->height; ++py) {
          const size_t src = static_cast<size_t>(py) * patch->width;
          const size_t dst = static_cast<size_t>(y0 + py) * fullW + x0;
          std::copy_n(patch->data.begin() + static_cast<std::ptrdiff_t>(src),
                      patch->width,
                      localCostmapGrid_.data.begin() + static_cast<std::ptrdiff_t>(dst));
        }
        localCostmapGrid_.header.stamp = node_->now();
        renderCostmap(localCostmapGrid_, "local_costmap_meta", false);
      });

    const auto cloudSubscribe = [this, sensorQos](const char *topic, const char *channel) {
      const QString ch = QString::fromLatin1(channel);
      subscribe<sensor_msgs::msg::PointCloud2>(topic, sensorQos, [this, ch](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        QJsonObject payload{{"count", static_cast<double>(msg->width) * static_cast<double>(msg->height)},
                            {"frame_id", QString::fromStdString(msg->header.frame_id)}};
        if (ch == QStringLiteral("planning_relevant_points")) {
          int xOffset = -1, yOffset = -1;
          for (const auto &field : msg->fields) {
            if (field.name == "x") xOffset = static_cast<int>(field.offset);
            else if (field.name == "y") yOffset = static_cast<int>(field.offset);
          }
          std::vector<std::pair<double,double>> pts;
          QJsonArray jsonPts;
          if (xOffset >= 0 && yOffset >= 0 && msg->point_step > 0) {
            const size_t count = std::min<size_t>(
              static_cast<size_t>(msg->width) * msg->height, 128U);
            pts.reserve(count);
            for (size_t i = 0; i < count; ++i) {
              const size_t base = i * msg->point_step;
              if (base + static_cast<size_t>(std::max(xOffset, yOffset)) + sizeof(float) > msg->data.size()) break;
              float x = 0.0f, y = 0.0f;
              std::memcpy(&x, msg->data.data() + base + static_cast<size_t>(xOffset), sizeof(float));
              std::memcpy(&y, msg->data.data() + base + static_cast<size_t>(yOffset), sizeof(float));
              if (!std::isfinite(x) || !std::isfinite(y)) continue;
              pts.emplace_back(static_cast<double>(x), static_cast<double>(y));
              jsonPts.append(QJsonArray{static_cast<double>(x), static_cast<double>(y)});
            }
          }
          {
            std::lock_guard<std::mutex> alignLock(costmapAlignmentMutex_);
            latestPlanningPoints_ = pts;
            latestPlanningFrame_ = msg->header.frame_id;
            latestPlanningSeen_ = std::chrono::steady_clock::now();
          }
          payload.insert("points", jsonPts);
        }
        update(ch, payload);
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
    navCancelClient_ = node_->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
    initialPosePub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    hmiRequestPub_ = node_->create_publisher<std_msgs::msg::String>("/hmi/request", 10);
    vescToolCommandPub_ = node_->create_publisher<std_msgs::msg::String>("/esc/vesc/tool_command", 20);
    finalGuardPub_ = node_->create_publisher<std_msgs::msg::String>("/perception/final_trial_guard", rclcpp::QoS(1).reliable().transient_local());
    trialTwistPub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/web_trial", 10);
    trialSourcePub_ = node_->create_publisher<std_msgs::msg::String>("/web_trial/active_source", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
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
    rm3100CalibrationProcess_.setProcessChannelMode(QProcess::MergedChannels);
    connect(&rm3100CalibrationProcess_, &QProcess::readyReadStandardOutput, this, [this]() {
      rm3100CalibrationLastOutput_ += QString::fromUtf8(rm3100CalibrationProcess_.readAllStandardOutput());
      if (rm3100CalibrationLastOutput_.size() > 6000) rm3100CalibrationLastOutput_ = rm3100CalibrationLastOutput_.right(6000);
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


  // Single server-owned Final Dataset trial. Browser only renews the lease.
  QJsonObject finalTrial_;
  QString finalToken_, finalMode_, finalCondition_, finalTargetResult_, finalSessionId_;
  qint64 finalStart_{0}, finalGo_{0}, finalHeartbeat_{0}, finalStopAt_{0}, finalStillAt_{0}, finalGreenAt_{0};
  int finalHeartbeatCount_{0};
  double finalX_{0}, finalY_{0}, finalYaw_{0}, finalPrevX_{0}, finalPrevY_{0}, finalTravel_{0}, finalObstacleDistance_{0}, finalAvoidSign_{0};
  bool finalAvoid_{false}, finalTurn_{false};
  bool finalPreviewActive_{false};
  qint64 finalPreviewExpire_{0};
  QString finalPreviewSession_;
  bool finalActive() const { const QString p=finalTrial_.value("phase").toString(); return p=="countdown"||p=="running"||p=="stopping"; }
  bool finalFresh(const QJsonObject &s,const QString &k,qint64 maxAge) const {
    const double t=s.value("__updated").toObject().value(k).toDouble(0);
    return t>0 && nowMs()-t>=0 && nowMs()-t<maxAge;
  }
  double finalNumber(const QJsonObject &s,const QString &k) const { return s.value(k).toDouble(std::numeric_limits<double>::quiet_NaN()); }
  bool finalDrivable(const QJsonObject &s) const {
    const auto lane=s.value("lane_state").toObject(),c=lane.value("corridor").toObject();
    // Motion authority requires the production lane-state validity and a valid
    // drivable area. A raw lane mask alone is never sufficient to move.
    const bool valid=lane.value("valid").toBool(false);
    const bool drivable=c.value("drivable_detected").toBool(false);
    const QString ev=c.value("evidence_mode").toString().toUpper(),rec=c.value("recommendation").toString().toUpper();
    return valid&&drivable&&!ev.contains("NO_EVIDENCE")&&rec!="NO_MASK_EVIDENCE"&&rec!="DRIVABLE_INVALID";
  }
  QJsonObject finalHumanCandidate(const QJsonObject &s) const {
    const auto sem=s.value("semantic_detections").toObject();
    // 4.5 is a safety/integration test: use only runtime detections that
    // passed semantic filtering. Low-floor evaluation_candidates remain for
    // calibration/4.2 and must never open a 4.5 motion or DRY-RUN gate.
    const QJsonArray a=sem.value("detections").toArray();
    QJsonObject best;double score=-1.0;int bestPriority=-1;
    for(const auto &v:a){
      const auto o=v.toObject();const QString n=o.value("class_name").toString().toLower();
      if(o.value("class_id").toInt(-1)!=0&&n!="person"&&n!="human")continue;
      if(o.contains("nav2_eligible")&&!o.value("nav2_eligible").toBool(false))continue;
      const double sc=o.value("score").toDouble(-1.0);
      const bool safetyTracked=o.value("safety_human_valid").toBool(false);
      if(sc<0.20&&!safetyTracked)continue;
      const QString tracking=o.value("tracking_state").toString().toUpper();
      const int priority=safetyTracked?2:((tracking=="STRONG"||tracking=="TRACKED"||tracking=="HELD")?1:0);
      if(priority>bestPriority||(priority==bestPriority&&sc>score)){bestPriority=priority;score=sc;best=o;}
    }
    return best;
  }
  bool finalHumanDetected(const QJsonObject &s) const { return !finalHumanCandidate(s).isEmpty(); }
  double finalHumanMetricDistance(const QJsonObject &s) const {
    const QJsonObject h=finalHumanCandidate(s);
    if(h.isEmpty())return std::numeric_limits<double>::quiet_NaN();
    for(const char *k:{"stable_forward_m","nav2_forward_m","calibrated_forward_m","raw_forward_m","forward_m"}){
      const QJsonValue v=h.value(k);
      if(v.isDouble()){
        const double x=v.toDouble(std::numeric_limits<double>::quiet_NaN());
        if(std::isfinite(x)&&x>0.0)return x;
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  }
  QString finalHumanReferenceError(const QJsonObject &s,const QString &condition) const {
    bool ok=false;const int ref=condition.toInt(&ok);
    if(!ok||ref<2||ref>5)return {};
    const double d=finalHumanMetricDistance(s);
    if(!std::isfinite(d))return QStringLiteral("Posisi metrik human tidak valid");
    constexpr double kToleranceM=1.0;
    if(std::abs(d-static_cast<double>(ref))>kToleranceM)
      return QString("Jarak human belum sesuai referensi %1 m — terbaca %2 m (toleransi ±%3 m)")
        .arg(ref).arg(d,0,'f',2).arg(kToleranceM,0,'f',1);
    return {};
  }
  QString finalReady(const QJsonObject &s,bool starting,const QString &mode=QString(),const QString &condition=QString(),bool requireActuation=true) const {
    for(const QString &k:{QString("connected.camera"),QString("camera_healthy")})
      if(!s.value(k).toBool(false))return k+" belum siap";
    if(requireActuation){
      for(const QString &k:{QString("connected.esc_feedback"),QString("connected.esc_ready")})
        if(!s.value(k).toBool(false))return k+" belum siap";
    }
    if(!s.contains("system.estop"))return "Status E-stop belum tersedia";
    if(s.value("system.estop").toBool()||s.value("vesc_maintenance_active").toBool())return "E-stop / maintenance aktif";
    const QString muxSource=s.value("nav_cmd_mux").toObject().value("raw").toString().toUpper();
    if(muxSource.startsWith("TELEOP"))return "Teleop aktif / release hold — lepaskan joystick sebelum ARMED";
    if(requireActuation){
      for(const QString &k:{QString("esc_odom"),QString("esc_drive_actual"),QString("esc_steer_actual")})
        if(!finalFresh(s,k,1000))return k+" stale";
    }
    if(!finalFresh(s,"trajectory_safety_state",1000))return "trajectory_safety_state stale";
    if(!finalFresh(s,"perception_performance",2500))return "perception_performance stale";
    const bool laneNav2Handoff=mode=="lane"&&!starting&&finalTrial_.value("lane_nav2_handoff").toBool(false);
    if(mode=="lane"&&!laneNav2Handoff&&!finalFresh(s,"lane_state",1500))return "lane_state stale";
    const auto ts=s.value("trajectory_safety_state").toObject();
    if(mode=="lane"){
      const bool nav2Handoff=laneNav2Handoff;
      const QString goal=s.value("goal_state").toObject().value("state").toString().toUpper();
      if(!nav2Handoff){
        if(s.value("system.autonomy_motion_allowed").toBool(false))
          return "Autonomy gate aktif sebelum Lane Safety menyerahkan kontrol ke Nav2";
        if(goal=="ACTIVE"||goal=="RUNNING"||goal=="EXECUTING"||goal=="SENDING"||goal=="QUEUED")
          return "Goal Nav2 aktif sebelum Lane Safety selesai recenter";
      }else{
        if(!s.value("system.planning_localization_ready").toBool(false))
          return "Planning localization hilang setelah Lane Safety handoff";
        if(!s.value("system.nav2_ready").toBool(false))
          return "Nav2 lifecycle tidak READY setelah Lane Safety handoff";
      }
      if(starting){
        if(!s.value("system.planning_localization_ready").toBool(false))
          return "Planning localization belum READY untuk Lane Safety -> Nav2";
        if(!s.value("system.nav2_ready").toBool(false))
          return "Nav2 lifecycle belum READY untuk Lane Safety -> Nav2";
        if(!s.value("system.autonomy_ready").toBool(false))
          return "Autonomy start gate belum READY untuk Lane Safety -> Nav2";
        if(!finalFresh(s,"ekf_global",1200))
          return "ekf_global stale";
      }
      if(condition!="gray"){
        if(s.value("perception_emergency").toBool(false)||ts.value("emergency").toBool(false))
          return "Perception emergency aktif — Lane Safety trial wajib STOP";
        // finalInitialLane() validates the actual RED/GREEN admission state.
        // Brief segmentation dropouts while running are handled fail-safe in
        // finalTick(): command zero and wait for evidence to recover.
      }
    }
    // Semantic detector is commissioned at ~2 Hz actual. Fail closed quickly
    // if fresh human evidence stops arriving; 1.5 s still tolerates >2 normal
    // inference periods while avoiding multi-second stale obstacle decisions.
    if(mode=="obstacle"&&starting&&!finalFresh(s,"semantic_detections",1200))
      return "semantic_detections stale";
    if(mode=="obstacle"&&condition!="none"&&starting){
      // Human evidence is mandatory at trial admission/countdown. Once the
      // vehicle has passed the obstacle, the person may naturally leave the
      // camera FOV; running success is then decided by the latched
      // cloud/costmap/MPPI/avoidance/recovery evidence plus goal completion.
      if(!finalHumanDetected(s))return "Human belum terdeteksi dengan confidence >= 20%";
      const QJsonObject human=finalHumanCandidate(s);
      double bboxH=finalNumber(human,"stable_bbox_height_fraction");
      if(!std::isfinite(bboxH))bboxH=finalNumber(human,"bbox_height_fraction");
      if(condition.startsWith("trial")&&
         (human.value("bottom_clipped").toBool(false)||(std::isfinite(bboxH)&&bboxH>=0.90))){
        return QStringLiteral("Human terlalu dekat / bbox saturasi untuk trial avoidance — mundurkan obstacle hingga seluruh bbox masuk frame");
      }
      const QString refErr=finalHumanReferenceError(s,condition);
      if(!refErr.isEmpty())return refErr;
    }
    if(!ts.value("final_trial_supported").toBool(false)||ts.value("commissioning_bypass").toBool(false))
      return "Safety runtime / trial watchdog belum siap";
    const bool obstacleNav=mode=="obstacle"&&condition!="1";
    if(obstacleNav){
      if(!s.value("system.planning_localization_ready").toBool(false))
        return "Planning localization belum READY — set Initial Pose / map anchor terlebih dahulu";
      if(!s.value("system.nav2_ready").toBool(false))
        return "Nav2 lifecycle planner/controller belum READY";
      if(starting&&!s.value("system.autonomy_ready").toBool(false))
        return "Autonomy start gate belum READY — motion localization / sensor admission belum lengkap";
      // MPPI trajectory is required as trial evidence after the goal is published;
      // do not require an active trajectory before a goal exists.
      if(!finalFresh(s,"ekf_global",1200))return "ekf_global stale";
      const auto mp=s.value("ekf_global").toObject();
      for(const auto k:{"x","y","yaw"})if(!std::isfinite(finalNumber(mp,k)))return "Pose map EKF tidak finite";
      if(condition!="none"&&!ts.value("metric_obstacle_safety").toBool(false))
        return "Metric obstacle safety belum aktif";
      // Strict planning-point vs local-costmap alignment is an admission check.
      // During motion the obstacle may be passed/cleared and the two asynchronous
      // streams can legitimately transition at different instants. Runtime safety
      // still fail-closes on stale obstacle/costmap data, while trial success
      // requires latched planning/costmap/MPPI/avoidance evidence.
      if(condition!="none"&&starting){
        if(!finalFresh(s,"local_costmap_meta",1200))return "local_costmap_meta stale";
        const auto cm=s.value("local_costmap_meta").toObject();
        if(!cm.value("planning_alignment_pass").toBool(false)){
          const double delta=cm.value("planning_alignment_max_m").toDouble(
            std::numeric_limits<double>::quiet_NaN());
          return std::isfinite(delta)?
            QString("Costmap tidak sinkron dengan planning points — Δ %1 m").arg(delta,0,'f',2):
            QString("Costmap sync belum tervalidasi");
        }
      }
    }
    if(requireActuation){
      if(!std::isfinite(finalNumber(s,"esc_drive_actual")))return "Feedback speed tidak finite";
      if(!std::isfinite(finalNumber(s,"esc_steer_actual")))return "Feedback steering tidak finite";
      if(starting&&std::abs(finalNumber(s,"esc_drive_actual"))>0.03)return "Kendaraan belum diam";
      const auto od=s.value("esc_odom").toObject();
      for(const auto k:{"x","y","yaw"})if(!std::isfinite(finalNumber(od,k)))return "Odometri tidak finite";
    }
    return {};
  }
  bool finalInitialLane(const QJsonObject &s,const QString &key) const {
    const auto lane=s.value("lane_state").toObject();const auto c=lane.value("corridor").toObject();
    // Canonical Final Dataset LEFT/RIGHT follows the physical/operator frame.
    const QString l=c.value("left_status").toString().toUpper(),r=c.value("right_status").toString().toUpper();
    const QString evidence=c.value("evidence_mode").toString().toUpper(),rec=c.value("recommendation").toString().toUpper();
    const bool hasVisual=c.value("drivable_detected").toBool(false)||c.value("lane_mask_detected").toBool(false)||c.value("raw_lane_mask_detected").toBool(false);
    const QString state=lane.value("state").toString("UNKNOWN").toUpper();
    const bool gray=evidence.contains("NO_EVIDENCE")||rec=="NO_MASK_EVIDENCE"||
      (!hasVisual&&(state=="UNKNOWN"||state=="LANE_LOST"))||
      ((l=="GRAY"||l=="UNKNOWN")&&(r=="GRAY"||r=="UNKNOWN")&&!hasVisual);
    if(key=="gray")return gray;
    if(!finalDrivable(s))return false;
    const bool leftLatched=c.value("left_recenter_latched").toBool(false);
    const bool rightLatched=c.value("right_recenter_latched").toBool(false);
    // Standalone YELLOW is warning-only and must not already be a latched RED recovery.
    return (key=="center"&&l=="GREEN"&&r=="GREEN")||
      (key=="left_yellow"&&l=="YELLOW"&&r=="GREEN"&&!leftLatched)||
      (key=="right_yellow"&&l=="GREEN"&&r=="YELLOW"&&!rightLatched)||
      (key=="left_red"&&l=="RED"&&r=="GREEN"&&leftLatched)||
      (key=="right_red"&&l=="GREEN"&&r=="RED"&&rightLatched);
  }
  void finalStop(const QString &result,const QString &reason) {
    if(!finalActive())return;
    if(finalTrial_.value("phase").toString()!="stopping"){
      finalTargetResult_=result;finalTrial_["reason"]=reason;finalTrial_["phase"]="stopping";finalStopAt_=nowMs();finalStillAt_=0;
      bridge_->publishFinalGuard("STOP",finalCondition_);
      QString msg;bridge_->publishFinalTrialVelocity(0.0,0.0,false,&msg);bridge_->cancelNavigation(&msg);
    }
  }
  void finalPersist() {
    const QString path=agvPath("data/perception_final_v2/trial_latest.json");QDir().mkpath(QFileInfo(path).absolutePath());QSaveFile f(path);if(f.open(QIODevice::WriteOnly)){f.write(QJsonDocument(finalTrial_).toJson());f.commit();}
  }
  QJsonObject finalRequest(const QString &action,const QJsonObject &j) {
    if(bridge_->readOnly())return {{"ok",false},{"message","GUI read-only"}};
    if(action=="preview-stop"){
      bridge_->publishFinalGuard("STOP","");
      QString msg;bridge_->publishFinalTrialVelocity(0.0,0.0,false,&msg);bridge_->cancelNavigation(&msg);
      bridge_->publishFinalGuard("IDLE","");
      finalPreviewActive_=false;finalPreviewExpire_=0;finalPreviewSession_.clear();
      return {{"ok",true},{"message","DRY RUN Nav2 preview dihentikan dengan kendaraan tetap STOP"}};
    }
    if(action=="preview-start"){
      if(finalActive())return {{"ok",false},{"message","ARMED trial masih aktif"}};
      Bab4PerceptionSessionStore sessionStore(agvRootPath(), loadConfigSnapshot());
      const QJsonObject sessionReply=sessionStore.state();
      const QString activeSession=sessionReply.value("state").toObject().value("session_id").toString();
      const QString clientSession=j.value("session_id").toString();
      if(activeSession.isEmpty()||clientSession.isEmpty()||activeSession!=clientSession)
        return {{"ok",false},{"code","SESSION_CHANGED"},{"message","Session Bab 4 berubah — refresh sebelum DRY RUN"},{"active_session_id",activeSession}};
      const QString mode=j.value("mode").toString(),key=j.value("condition").toString();
      if(mode!="obstacle"||!QSet<QString>{"none","1","2","3","4","5","trial1","trial2","trial3"}.contains(key))
        return {{"ok",false},{"message","Preview Nav2 hanya untuk obstacle legacy atau trial 1–3"}};
      // DRY RUN is planning-only and the final guard holds STOP. Require
      // localization/Nav2/costmap/semantic readiness, but not the motion/autonomy
      // start gate that is intentionally reserved for the later ARMED trial.
      const auto st=bridge_->snapshot();const QString err=finalReady(st,false,mode,key,false);
      if(!err.isEmpty())return {{"ok",false},{"message",err}};
      if(key=="none"){
        if(finalHumanDetected(st))return {{"ok",false},{"message","NO HUMAN ditolak: human masih terdeteksi"}};
      }else{
        const bool repeatTrial=QRegularExpression(QStringLiteral("^trial[1-3]$")).match(key).hasMatch();
        // The 3-trial DRY RUN is planning-only and keeps the final guard at STOP.
        // Allow preview admission before a human enters the path so the operator
        // can start the recorder first. Human + metric + cloud + costmap + MPPI
        // are still required by capture/integration PASS and by ARMED admission.
        if(!repeatTrial){
          if(!finalHumanDetected(st))
            return {{"ok",false},{"message","Human belum terdeteksi dengan confidence >= 20%"}};
          const QString refErr=finalHumanReferenceError(st,key);
          if(!refErr.isEmpty())return {{"ok",false},{"message",refErr}};
        }
      }
      QString msg;bridge_->publishFinalGuard("STOP",key);
      bridge_->publishFinalTrialVelocity(0.0,0.0,false,&msg);bridge_->cancelNavigation(&msg);
      QJsonObject out{{"ok",true},{"mode",mode},{"condition",key},{"goal_published",false}};
      if(key!="1"){
        const auto mp=st.value("ekf_global").toObject();
        const double x=finalNumber(mp,"x"),y=finalNumber(mp,"y"),yaw=finalNumber(mp,"yaw");
        const bool repeatTrial=QRegularExpression(QStringLiteral("^trial[1-3]$")).match(key).hasMatch();
        const double target=repeatTrial?5.0:(key=="none"?2.0:key.toDouble()+1.5);
        const double gx=x+std::cos(yaw)*target,gy=y+std::sin(yaw)*target;
        if(!bridge_->publishGoal(gx,gy,yaw,&msg)){
          bridge_->publishFinalGuard("IDLE","");
          return {{"ok",false},{"message",msg}};
        }
        out["goal_published"]=true;out["goal_x_map"]=gx;out["goal_y_map"]=gy;out["target_distance_m"]=target;
      }
      finalPreviewActive_=true;finalPreviewExpire_=nowMs()+8000;finalPreviewSession_=activeSession;
      out["expires_at_ms"]=static_cast<double>(finalPreviewExpire_);
      out["message"]="DRY RUN preview aktif: Nav2/MPPI boleh planning, final guard memaksa kendaraan STOP";
      return out;
    }
    if(action=="start"){
      if(finalActive())return {{"ok",false},{"message","Trial lain masih aktif"}};
      Bab4PerceptionSessionStore sessionStore(agvRootPath(), loadConfigSnapshot());
      const QJsonObject sessionReply=sessionStore.state();
      const QString activeSession=sessionReply.value("state").toObject().value("session_id").toString();
      const QString clientSession=j.value("session_id").toString();
      if(activeSession.isEmpty()||clientSession.isEmpty()||activeSession!=clientSession)
        return {{"ok",false},{"code","SESSION_CHANGED"},{"message","Session Bab 4 berubah — refresh sebelum ARMED"},{"active_session_id",activeSession}};
      const QString mode=j.value("mode").toString(),key=j.value("condition").toString(),token=j.value("token").toString();
      const int laneTrialIndex=mode=="lane"?j.value("trial_index").toInt(0):0;
      if(token.size()<20||token.size()>100||!(mode=="obstacle"||mode=="lane"))return {{"ok",false},{"message","Identitas trial tidak valid"}};
      if(mode=="lane"&&(laneTrialIndex<1||laneTrialIndex>3))return {{"ok",false},{"message","Nomor lane trial harus 1..3"}};
      QString storedCondition;
      if(mode=="obstacle"){
        const QRegularExpression trialRe(QStringLiteral("^trial([1-3])$"));const auto tm=trialRe.match(key);
        storedCondition=tm.hasMatch()?QStringLiteral("obstacle_trial_")+tm.captured(1):
          (key=="none"?QStringLiteral("obstacle_no_human"):QStringLiteral("obstacle_")+key+QStringLiteral("m"));
      } else {
        storedCondition=QStringLiteral("lane_trial_")+QString::number(laneTrialIndex);
      }
      const QJsonObject activeState=sessionReply.value("state").toObject();
      const QJsonObject stored=activeState.value("subchapters").toObject().value("4.5").toObject().value("conditions").toObject().value(storedCondition).toObject();
      bool storedArmedComplete=stored.value("armed_complete").toBool(false);
      if(!storedArmedComplete){
        for(const QJsonValue &v:stored.value("rows").toArray()){const QJsonObject rr=v.toObject();
          if(rr.value("mode").toString()=="ARMED_VEHICLE_TEST"&&rr.value("result").toString().toLower()=="complete"&&rr.value("final_vehicle_state").toString()=="STOPPED"&&rr.value("integration_pass").toBool(false)){storedArmedComplete=true;break;}
        }
      }
      if(storedArmedComplete)
        return {{"ok",false},{"message","Kondisi 4.5 sudah SELESAI — reset kondisi untuk mengulang ARMED"}};
      if(!stored.value("dry_run_complete").toBool(false)||!stored.value("dry_run_pass").toBool(false))
        return {{"ok",false},{"message","ARMED diblok: DRY RUN kondisi ini belum PASS"}};
      if(finalPreviewActive_)return {{"ok",false},{"message","DRY RUN Nav2 preview masih aktif — tunggu preview dihentikan"}};
      const auto s=bridge_->snapshot();const QString err=finalReady(s,true,mode,key);if(!err.isEmpty())return {{"ok",false},{"message",err}};
      // Bab 4.5 is itself the response qualification test. Runtime safety gates
      // above replace circular dependencies on previously reviewed qualification cards.
      const QString goal=s.value("goal_state").toObject().value("state").toString();
      if(goal.contains("ACTIVE")||goal.contains("RUNNING")||goal.contains("EXECUT"))
        return {{"ok",false},{"message","Batalkan goal navigasi yang aktif dahulu"}};
      if(mode=="lane"&&key!="left_red"&&key!="right_red")
        return {{"ok",false},{"message","4.5B recenter trial hanya menerima RED pada salah satu sisi"}};
      if(mode=="lane"&&!finalInitialLane(s,key)){
        // If the exact RED/GREEN sample disappeared between the browser click
        // and this request, admit the trial only when evidence is currently
        // unavailable. Countdown stays STOP and waits for the requested RED
        // side to become valid again. A valid-but-wrong lane state is rejected.
        if(finalDrivable(s))return {{"ok",false},{"message","Kondisi lane awal tidak sesuai"}};
      }
      double obstacleReference=std::numeric_limits<double>::infinity();
      if(mode=="obstacle"){
        if(key=="none"){
          if(finalHumanDetected(s))return {{"ok",false},{"message","NO HUMAN ditolak: human masih terdeteksi"}};
        }else{
          const bool repeatTrial=QRegularExpression(QStringLiteral("^trial[1-3]$")).match(key).hasMatch();
          if(!repeatTrial&&!QSet<QString>{"1","2","3","4","5"}.contains(key))
            return {{"ok",false},{"message","Kondisi obstacle tidak valid"}};
          if(!finalHumanDetected(s))return {{"ok",false},{"message","Human belum terdeteksi dengan confidence >= 20%"}};
          if(!repeatTrial){
            const QString refErr=finalHumanReferenceError(s,key);
            if(!refErr.isEmpty())return {{"ok",false},{"message",refErr}};
            obstacleReference=key.toDouble();
          }
        }
      }
      QString msg;if(!bridge_->cancelNavigation(&msg))return {{"ok",false},{"message",msg}};
      bridge_->publishFinalTrialVelocity(0.0,0.0,false,&msg);
      finalToken_=token;finalMode_=mode;finalCondition_=key;finalSessionId_=activeSession;finalStart_=finalHeartbeat_=nowMs();finalGo_=0;finalTravel_=0;finalAvoid_=finalTurn_=false;finalAvoidSign_=0.0;finalGreenAt_=0;finalObstacleDistance_=obstacleReference;finalHeartbeatCount_=0;
      const bool obstacleRepeat=mode=="obstacle"&&QRegularExpression(QStringLiteral("^trial[1-3]$")).match(key).hasMatch();
      const double target=(mode=="lane")?5.0:(mode=="obstacle"?(obstacleRepeat?5.0:(key=="none"?2.0:(key=="1"?0.0:key.toDouble()+1.5))):0.0);
      finalTrial_={{"phase","countdown"},{"countdown",3},{"mode",mode},{"condition",key},{"trial_index",mode=="lane"?QJsonValue(laneTrialIndex):QJsonValue()},{"reference_distance_m",std::isfinite(obstacleReference)?QJsonValue(obstacleReference):QJsonValue()},{"target_distance_m",target},{"started_at_ms",finalStart_},{"travel_m",0.0},{"response_ms",QJsonValue()},{"recovery_ms",QJsonValue()},
        {"metric_position_valid",mode!="obstacle"||key=="none"},{"object_cloud_seen",false},{"planning_cloud_seen",false},{"costmap_marking_seen",false},
        {"mppi_seen",false},{"avoidance_seen",false},{"recovery_seen",false},{"web_trial_authority_seen",false},{"web_trial_authority_at_ms",QJsonValue()},
        {"lane_nav2_handoff",false},{"lane_nav2_handoff_at_ms",QJsonValue()},{"lane_nav2_authority_seen",false},
        {"steering_response_seen",false},{"steering_direction_match",false},{"steering_response_ms",QJsonValue()},
        {"steering_neutral_maintained",true},{"peak_steering_actual_rad",QJsonValue()},{"integration_pass",false},
        {"obstacle_motion_released",false},{"planner_gate_pass",false},{"planner_gate_started_ms",QJsonValue()},
        {"planner_gate_passed_ms",QJsonValue()},{"planner_gate_reason",QStringLiteral("NOT_APPLICABLE")},
        {"planner_hold_active",false},{"planner_hold_started_ms",QJsonValue()},{"planner_hold_count",0},
        {"planner_hold_recovery_count",0},{"planner_path_age_ms",QJsonValue()},
        {"planner_path_count",0},{"planner_path_length_m",QJsonValue()},
        {"planner_path_heading_variation_rad",QJsonValue()},{"planner_path_quality_ok",false}};
      finalTrial_["heartbeat_count"]=0;
      finalTrial_["heartbeat_last_ms"]=finalHeartbeat_;
      finalTrial_["heartbeat_age_ms"]=0;
      finalTrial_["heartbeat_transport"]=QStringLiteral("start_request");
      finalTrial_["heartbeat_sequence"]=0;
      if(mode=="lane"){
        const QString dir=key=="left_red"?QStringLiteral("RIGHT_POSITIVE_PHYSICAL"):
          key=="right_red"?QStringLiteral("LEFT_NEGATIVE_PHYSICAL"):QStringLiteral("NEUTRAL_OR_WARNING_ONLY");
        finalTrial_["expected_steering_direction"]=dir;
      }
      bridge_->publishFinalGuard("STOP",key);finalPersist();return {{"ok",true},{"trial",finalTrial_}};
    }
    if(j.value("token").toString()!=finalToken_||finalToken_.isEmpty())return {{"ok",false},{"message","Token trial tidak cocok"}};
    if(j.value("session_id").toString()!=finalSessionId_||finalSessionId_.isEmpty())
      return {{"ok",false},{"code","SESSION_CHANGED"},{"message","Session ARMED tidak cocok"}};
    if(action=="heartbeat"){
      finalHeartbeat_=nowMs();
      ++finalHeartbeatCount_;
      finalTrial_["heartbeat_count"]=finalHeartbeatCount_;
      finalTrial_["heartbeat_last_ms"]=finalHeartbeat_;
      finalTrial_["heartbeat_age_ms"]=0;
      finalTrial_["heartbeat_transport"]=j.value("transport").toString(QStringLiteral("browser"));
      finalTrial_["heartbeat_sequence"]=j.value("sequence");
    }
    else if(action=="stop")finalStop("aborted",j.value("reason").toString("Operator abort"));
    else return {{"ok",false},{"message","Aksi tidak dikenal"}};
    return {{"ok",true},{"trial",finalTrial_}};
  }
  void finalTick(){
    const qint64 tickNow=nowMs();
    if(finalPreviewActive_&&tickNow>finalPreviewExpire_){
      bridge_->publishFinalGuard("STOP","");
      QString msg;bridge_->publishFinalTrialVelocity(0.0,0.0,false,&msg);bridge_->cancelNavigation(&msg);
      bridge_->publishFinalGuard("IDLE","");
      finalPreviewActive_=false;finalPreviewExpire_=0;finalPreviewSession_.clear();
    }
    if(!finalActive())return;
    const auto s=bridge_->snapshot();const qint64 t=tickNow;QString phase=finalTrial_.value("phase").toString();
    finalTrial_["heartbeat_age_ms"]=std::max<qint64>(0,t-finalHeartbeat_);
    if(phase!="stopping"){
      const QString err=finalReady(s,phase=="countdown",finalMode_,finalCondition_);
      // 4.5 browser watchdog: beri margin untuk scheduling UI/SSE/canvas tanpa
      // menghilangkan fail-safe. Pada mode final kecepatan tetap dibatasi supervisor.
      // Browser/UI rendering may occasionally stall for a few seconds under
      // camera/costmap load. Keep fail-safe supervision, but do not invalidate
      // a physical trial on a single short scheduling/network hiccup.
      if(t-finalHeartbeat_>6000)finalStop("invalid","Heartbeat browser terputus");
      else if(!err.isEmpty())finalStop("invalid",err);
      else {
        const qint64 maxTrialMs=(finalMode_=="obstacle"&&finalCondition_!="none")?75000:(finalMode_=="lane"?110000:45000);
        if(t-finalStart_>maxTrialMs)finalStop("failed","Batas waktu trial tercapai");
      }
    }
    phase=finalTrial_.value("phase").toString();
    if(phase=="countdown"){
      bridge_->publishFinalGuard("STOP",finalCondition_);
      QString stopMsg;bridge_->publishFinalTrialVelocity(0.0,0.0,false,&stopMsg);
      finalTrial_["countdown"]=std::max(0,3-static_cast<int>((t-finalStart_)/1000));
      if(t-finalStart_<3000)return;
      if(finalMode_=="lane"&&!finalInitialLane(s,finalCondition_)){
        if(!finalDrivable(s)&&t-finalStart_<6500){
          finalTrial_["countdown"]=0;
          finalTrial_["countdown_wait_lane_evidence"]=true;
          return;
        }
        finalStop("invalid","Kondisi lane berubah saat countdown");return;
      }
      if(finalMode_=="lane"){
        finalTrial_["countdown_wait_lane_evidence"]=false;
        const auto lane0=s.value("lane_state").toObject(),c0=lane0.value("corridor").toObject();
        finalTrial_["initial_lane_status"]=c0.value("left_status").toString().toUpper()+QStringLiteral("|")+c0.value("right_status").toString().toUpper();
        finalTrial_["initial_lane_state"]=lane0.value("state").toString().toUpper();
        finalTrial_["initial_lane_recommendation"]=c0.value("recommendation").toString().toUpper();
        finalTrial_["initial_left_latched"]=c0.value("left_recenter_latched").toBool(false);
        finalTrial_["initial_right_latched"]=c0.value("right_recenter_latched").toBool(false);
        finalTrial_["initial_condition_match"]=true;
        const double steer0=finalNumber(s,"esc_steer_actual");
        if((finalCondition_=="center"||finalCondition_=="left_yellow"||finalCondition_=="right_yellow")&&
           std::isfinite(steer0)&&std::abs(steer0)>5.0*kPi/180.0){
          finalStop("invalid",QString("Steering belum center untuk trial warning/straight (%1 deg)").arg(steer0*180.0/kPi,0,'f',1));return;
        }
      }
      if(finalMode_=="obstacle"){
        if(finalCondition_=="none"){
          if(finalHumanDetected(s)){finalStop("invalid","NO HUMAN berubah: human terdeteksi saat countdown");return;}
        }else{
          const QJsonObject h=finalHumanCandidate(s);
          if(h.isEmpty()){finalStop("invalid","Human hilang saat countdown");return;}
          double cx=h.value("center_x_fraction").toDouble(std::numeric_limits<double>::quiet_NaN());
          if(!std::isfinite(cx)){
            const double x1=h.value("x1").toDouble(std::numeric_limits<double>::quiet_NaN());
            const double x2=h.value("x2").toDouble(std::numeric_limits<double>::quiet_NaN());
            const double iw=s.value("semantic_detections").toObject().value("image_width").toDouble(0.0);
            if(std::isfinite(x1)&&std::isfinite(x2)&&iw>1.0)cx=((x1+x2)*0.5)/iw;
          }
          if(!std::isfinite(cx))cx=0.5;
          const double displayCx=1.0-cx;
          const double calibrated=finalHumanMetricDistance(s);
          const bool metricValid=std::isfinite(calibrated)&&calibrated>0.0;
          finalTrial_["human_start_confidence"]=h.value("score");
          finalTrial_["human_start_source_x_fraction"]=cx;
          finalTrial_["human_start_display_x_fraction"]=displayCx;
          finalTrial_["human_start_calibrated_distance_m"]=std::isfinite(calibrated)?QJsonValue(calibrated):QJsonValue();
          finalTrial_["metric_position_valid"]=metricValid;
          if(finalCondition_!="1"&&!metricValid){finalStop("invalid","Estimasi metrik human tidak valid");return;}
          const QString refErr=finalHumanReferenceError(s,finalCondition_);
          if(!refErr.isEmpty()){finalStop("invalid",refErr);return;}
        }
      }
      if(finalMode_=="obstacle"&&finalCondition_=="1"){
        const auto ts=s.value("trajectory_safety_state").toObject();
        const bool nearFieldStop=s.value("perception_emergency").toBool(false)||
          ts.value("decision").toString().toUpper().contains("EMERGENCY");
        finalGo_=t;finalTrial_["go_at_ms"]=t;finalTrial_["response_ms"]=0;
        finalTrial_["response"]="HUMAN 1M -> STOP";
        finalTrial_["command_v_mps"]=0.0;finalTrial_["command_w_rad_s"]=0.0;
        finalTrial_["forward_progress_m"]=0.0;finalTrial_["lateral_displacement_m"]=0.0;
        finalTrial_["near_field_stop_active"]=nearFieldStop;
        finalTrial_["integration_pass"]=nearFieldStop;
        finalStop(nearFieldStop?"complete":"failed",
          nearFieldStop?"Human referensi 1 m: near-field emergency aktif dan kendaraan tetap STOP":
          "Human referensi 1 m terdeteksi tetapi near-field emergency tidak aktif");return;
      }
      if(finalMode_=="lane"&&finalCondition_=="gray"){
        const double v0=finalNumber(s,"esc_drive_actual");
        const bool stopped=finalFresh(s,"esc_drive_actual",500)&&std::isfinite(v0)&&std::abs(v0)<0.03;
        finalGo_=t;finalTrial_["go_at_ms"]=t;finalTrial_["response_ms"]=0;
        finalTrial_["response"]="GRAY/GRAY -> STOP";finalTrial_["command_v_mps"]=0.0;finalTrial_["command_w_rad_s"]=0.0;
        finalTrial_["steering_neutral_maintained"]=true;finalTrial_["integration_pass"]=stopped;
        finalStop(stopped?"complete":"failed",stopped?
          "GRAY/GRAY: fail-closed tervalidasi dan kendaraan tetap STOP":
          "GRAY/GRAY terdeteksi tetapi feedback kendaraan belum STOP");return;
      }
      const auto od=s.value("esc_odom").toObject();
      finalX_=finalPrevX_=finalNumber(od,"x");finalY_=finalPrevY_=finalNumber(od,"y");
      finalYaw_=finalNumber(od,"yaw");
      if(!std::isfinite(finalYaw_)){finalStop("invalid","Heading awal odometri tidak valid");return;}
      finalGo_=t;finalTravel_=0.0;finalGreenAt_=0;
      finalTrial_["phase"]="running";finalTrial_["go_at_ms"]=t;finalTrial_["countdown"]=0;
      finalTrial_["initial_heading_rad"]=finalYaw_;finalTrial_["lane_phase"]=finalMode_=="lane"?QStringLiteral("RECENTER"):QStringLiteral("NAV2");
      if(finalMode_=="lane"){
        finalTrial_["lane_last_valid_ms"]=t;finalTrial_["lane_dropout_hold_count"]=0;
        finalTrial_["lane_dropout_max_ms"]=0;
        const auto mp0=s.value("ekf_global").toObject();
        const double mx0=finalNumber(mp0,"x"),my0=finalNumber(mp0,"y"),myaw0=finalNumber(mp0,"yaw");
        if(!std::isfinite(mx0)||!std::isfinite(my0)||!std::isfinite(myaw0)){
          finalStop("invalid","Pose map awal Lane Safety tidak valid");return;
        }
        const double target=finalTrial_.value("target_distance_m").toDouble(5.0);
        const double gx=mx0+std::cos(myaw0)*target,gy=my0+std::sin(myaw0)*target;
        finalTrial_["initial_map_x"]=mx0;finalTrial_["initial_map_y"]=my0;finalTrial_["initial_map_heading_rad"]=myaw0;
        finalTrial_["goal_x_map"]=gx;finalTrial_["goal_y_map"]=gy;finalTrial_["goal_yaw_map"]=myaw0;
        finalTrial_["goal_state"]="WAIT_RECENTER";
      }
      if(finalMode_=="obstacle"){
        const auto mp=s.value("ekf_global").toObject();
        const double mx=finalNumber(mp,"x"),my=finalNumber(mp,"y"),myaw=finalNumber(mp,"yaw");
        const double target=finalTrial_.value("target_distance_m").toDouble(0.0);
        const double gx=mx+std::cos(myaw)*target,gy=my+std::sin(myaw)*target;
        const bool humanPlannerTrial=finalCondition_!="none"&&finalCondition_!="1";
        QString msg;bridge_->publishFinalTrialVelocity(0.0,0.0,false,&msg);
        // HUMAN 4.5A: arm world-memory and let Nav2 compute while the safety
        // supervisor keeps actuator output at zero. Motion is released only
        // after a post-latch global path reaches MPPI.
        bridge_->publishFinalGuard(humanPlannerTrial?"OBSTACLE_GATE":"OBSTACLE",finalCondition_);
        if(!bridge_->publishGoal(gx,gy,myaw,&msg)){finalStop("invalid",msg);return;}
        finalTrial_["goal_x_map"]=gx;finalTrial_["goal_y_map"]=gy;finalTrial_["goal_yaw_map"]=myaw;
        finalTrial_["planner_goal_sent_ms"]=t;
        if(humanPlannerTrial){
          finalTrial_["planner_gate_started_ms"]=t;
          finalTrial_["planner_gate_reason"]="WAIT_HUMAN_WORLD_MEMORY";
          finalTrial_["obstacle_motion_released"]=false;
          finalTrial_["response"]="HOLD STOP -> WAIT HUMAN LATCH + COSTMAP + NEW PATH + MPPI";
        }else{
          finalTrial_["planner_gate_pass"]=true;
          finalTrial_["planner_gate_reason"]="NOT_REQUIRED";
          finalTrial_["obstacle_motion_released"]=true;
          finalTrial_["response"]="NAV2 GOAL ACTIVE";
        }
      }else{
        bridge_->publishFinalGuard("LANE",finalCondition_);
      }
      return;
    }
    if(phase=="running"){
      const bool laneNav2GuardReleased=finalMode_=="lane"&&
        finalTrial_.value("lane_nav2_handoff").toBool(false);
      const bool humanObstacleTrial=finalMode_=="obstacle"&&finalCondition_!="none"&&finalCondition_!="1";
      const bool obstacleMotionReleased=finalTrial_.value("obstacle_motion_released").toBool(false);
      const bool plannerHoldActive=finalTrial_.value("planner_hold_active").toBool(false);
      const bool obstaclePlannerGate=humanObstacleTrial&&(!obstacleMotionReleased||plannerHoldActive);
      const QString finalGuardMode=laneNav2GuardReleased?QStringLiteral("IDLE"):
        (finalMode_=="lane"?QStringLiteral("LANE"):
          (obstaclePlannerGate?QStringLiteral("OBSTACLE_GATE"):QStringLiteral("OBSTACLE")));
      bridge_->publishFinalGuard(finalGuardMode,laneNav2GuardReleased?QString():finalCondition_);
      const auto od=s.value("esc_odom").toObject();
      const double x=finalNumber(od,"x"),y=finalNumber(od,"y");
      const double step=std::hypot(x-finalPrevX_,y-finalPrevY_);
      if(!std::isfinite(step)||step>.30){finalStop("invalid","Lompatan odometri");return;}
      finalTravel_+=step;finalPrevX_=x;finalPrevY_=y;
      const double dx=x-finalX_,dy=y-finalY_;
      const double forwardProgress=std::cos(finalYaw_)*dx+std::sin(finalYaw_)*dy;
      const double lateralDisplacement=-std::sin(finalYaw_)*dx+std::cos(finalYaw_)*dy;
      if(!std::isfinite(forwardProgress)||!std::isfinite(lateralDisplacement)){finalStop("invalid","Progres odometri tidak finite");return;}
      finalTrial_["travel_m"]=finalTravel_;
      finalTrial_["forward_progress_m"]=forwardProgress;
      finalTrial_["lateral_displacement_m"]=lateralDisplacement;
      finalTrial_["duration_s"]=(t-finalGo_)/1000.0;

      const auto laneObj=s.value("lane_state").toObject(),lane=laneObj.value("corridor").toObject();
      const QString left=lane.value("left_status").toString("UNKNOWN").toUpper();
      const QString right=lane.value("right_status").toString("UNKNOWN").toUpper();
      const bool gg=left=="GREEN"&&right=="GREEN";
      const bool yg=left=="YELLOW"&&right=="GREEN";
      const bool rg=left=="RED"&&right=="GREEN";
      const bool gy=left=="GREEN"&&right=="YELLOW";
      const bool gr=left=="GREEN"&&right=="RED";
      const double actualSpeed=finalNumber(s,"esc_drive_actual");
      double cmdV=0.0,cmdW=0.0;QString response;bool commandValid=true;

      if(finalMode_=="lane"){
        const double laneTarget=std::max(0.5,finalTrial_.value("target_distance_m").toDouble(5.0));
        const bool nav2Handoff=finalTrial_.value("lane_nav2_handoff").toBool(false);
        const bool laneNowValid=finalDrivable(s);
        finalTrial_["lane_post_handoff_valid"]=laneNowValid;
        if(!nav2Handoff){
          if(laneNowValid){
            finalTrial_["lane_last_valid_ms"]=t;
            finalTrial_["lane_dropout_hold_active"]=false;
          }else{
            const qint64 lastValid=static_cast<qint64>(finalTrial_.value("lane_last_valid_ms").toDouble(finalGo_));
            const qint64 dropoutAge=std::max<qint64>(0,t-lastValid);
            finalTrial_["lane_dropout_hold_active"]=true;
            finalTrial_["lane_dropout_age_ms"]=dropoutAge;
            finalTrial_["lane_dropout_hold_count"]=finalTrial_.value("lane_dropout_hold_count").toInt(0)+1;
            const qint64 prevMax=static_cast<qint64>(finalTrial_.value("lane_dropout_max_ms").toDouble(0));
            finalTrial_["lane_dropout_max_ms"]=std::max(prevMax,dropoutAge);
            QString holdMsg;
            if(!bridge_->publishFinalTrialVelocity(0.0,0.0,true,&holdMsg)){finalStop("invalid",holdMsg);return;}
            finalTrial_["response"]="LANE EVIDENCE DROPOUT -> HOLD STOP, WAIT RECOVERY";
            finalTrial_["command_v_mps"]=0.0;finalTrial_["command_w_rad_s"]=0.0;
            if(dropoutAge>1800){
              finalStop("invalid",QString("Lane/drivable hilang > 1.8 s saat recenter (%1 ms)").arg(dropoutAge));return;
            }
            return;
          }
        }
        const QString muxSource=s.value("nav_cmd_mux").toObject().value("raw").toString().toUpper();
        finalTrial_["cmd_mux_source_last"]=muxSource;
        if(!nav2Handoff&&muxSource=="WEB_TRIAL"){
          if(!finalTrial_.value("web_trial_authority_seen").toBool(false))
            finalTrial_["web_trial_authority_at_ms"]=t;
          finalTrial_["web_trial_authority_seen"]=true;
        }
        if(nav2Handoff&&muxSource=="AUTONOMY")
          finalTrial_["lane_nav2_authority_seen"]=true;
        if(!nav2Handoff&&t-finalGo_>1200&&!finalTrial_.value("web_trial_authority_seen").toBool(false)){
          finalStop("invalid","WEB_TRIAL tidak memperoleh actuator authority dari cmd_vel_router");return;
        }
        const qint64 handoffAt=static_cast<qint64>(finalTrial_.value("lane_nav2_handoff_at_ms").toDouble(0));
        if(nav2Handoff&&handoffAt>0&&t-handoffAt>2500&&!finalTrial_.value("lane_nav2_authority_seen").toBool(false)){
          finalStop("invalid","Nav2 goal aktif tetapi AUTONOMY belum memperoleh actuator authority");return;
        }
        const double speedUpper=nav2Handoff?1.35:0.35;
        if(std::isfinite(actualSpeed)&&(actualSpeed>speedUpper||actualSpeed<-0.05)){
          finalStop("invalid",QString("Feedback speed di luar envelope 4.5B (%1 m/s)").arg(actualSpeed,0,'f',3));return;
        }
        if(std::abs(lateralDisplacement)>0.75){
          finalStop("failed",QString("Lateral displacement melewati batas trial (%1 m)").arg(lateralDisplacement,0,'f',2));return;
        }
        if(finalTravel_>laneTarget+1.5&&forwardProgress<laneTarget){
          finalStop("failed",QString("Travel > target+1.5 m tetapi forward progress belum %1 m").arg(laneTarget,0,'f',1));return;
        }
        const bool leftLatched=lane.value("left_recenter_latched").toBool(false);
        const bool rightLatched=lane.value("right_recenter_latched").toBool(false);
        const bool warningTrial=finalCondition_=="left_yellow"||finalCondition_=="right_yellow";
        const bool redTrial=finalCondition_=="left_red"||finalCondition_=="right_red";
        const bool straightTrial=finalCondition_=="center"||warningTrial;
        const double steerActual=finalNumber(s,"esc_steer_actual");
        const double steerTarget=finalNumber(s,"esc_steer_target");
        const double currentYaw=finalNumber(od,"yaw");
        if(!std::isfinite(currentYaw)){finalStop("invalid","Heading odometri tidak valid saat lane trial");return;}
        const double headingError=std::atan2(std::sin(finalYaw_-currentYaw),std::cos(finalYaw_-currentYaw));
        const double headingErrorDeg=headingError*180.0/kPi;
        const double headingDeadband=1.5*kPi/180.0;
        const double headingRestoreTolerance=3.0*kPi/180.0;
        const bool headingRestored=std::abs(headingError)<=headingRestoreTolerance;
        const double headingCmd=std::abs(headingError)<=headingDeadband?0.0:
          std::clamp(0.65*headingError,-0.040,0.040);
        finalTrial_["current_heading_rad"]=currentYaw;
        finalTrial_["heading_error_rad"]=headingError;
        finalTrial_["heading_error_deg"]=headingErrorDeg;
        finalTrial_["heading_restored"]=headingRestored;
        finalTrial_["steering_actual_rad"]=std::isfinite(steerActual)?QJsonValue(steerActual):QJsonValue();
        finalTrial_["steering_target_rad"]=std::isfinite(steerTarget)?QJsonValue(steerTarget):QJsonValue();
        if(std::isfinite(steerActual)){
          const QJsonValue peakV=finalTrial_.value("peak_steering_actual_rad");
          double peak=peakV.isDouble()?peakV.toDouble():0.0;
          if(!peakV.isDouble()||std::abs(steerActual)>std::abs(peak))peak=steerActual;
          finalTrial_["peak_steering_actual_rad"]=peak;
        }

        if(redTrial&&gg&&finalGreenAt_==0){
          finalGreenAt_=t;finalTrial_["recovery_ms"]=t-finalGo_;finalTrial_["recovery_seen"]=true;
          finalTrial_["lane_phase"]="HEADING_RECOVERY";
        }
        if(redTrial&&gg&&headingRestored){
          finalTrial_["heading_restored_once"]=true;
          if(!nav2Handoff)finalTrial_["lane_phase"]="READY_NAV2_HANDOFF";
        }else if(redTrial&&gg){
          if(!nav2Handoff)finalTrial_["lane_phase"]="HEADING_RECOVERY";
        }else if(redTrial){
          if(!nav2Handoff)finalTrial_["lane_phase"]="RECENTER";
        }

        // Once the lane is centered and the original heading is restored,
        // hand the remaining motion to Nav2/MPPI. The goal was frozen at trial
        // start: 5 m forward along the original map heading.
        if(redTrial&&nav2Handoff){
          const QString goalState=s.value("goal_state").toObject().value("state").toString().toUpper();
          finalTrial_["goal_state"]=goalState;
          finalTrial_["lane_phase"]="NAV2_TO_GOAL";
          const QJsonObject mppi=s.value("mppi_trajectories").toObject();
          const bool mppiFresh=finalFresh(s,"mppi_trajectories",1800)&&mppi.value("count").toDouble(0)>0;
          if(mppiFresh)finalTrial_["mppi_seen"]=true;
          if(goalState.contains("ABORT")||goalState.contains("REJECT")||goalState.contains("FAIL")){
            finalStop("failed","Nav2 lane goal gagal: "+goalState);return;
          }
          const bool reached=goalState.contains("SUCCEEDED")||forwardProgress>=std::max(0.0,laneTarget-0.25);
          if(reached){
            const bool success=finalTrial_.value("recovery_seen").toBool(false)&&
              finalTrial_.value("heading_restored_once").toBool(false)&&
              finalTrial_.value("steering_response_seen").toBool(false)&&
              finalTrial_.value("steering_direction_match").toBool(false)&&
              finalTrial_.value("web_trial_authority_seen").toBool(false)&&
              finalTrial_.value("lane_nav2_authority_seen").toBool(false)&&
              finalTrial_.value("mppi_seen").toBool(false);
            finalTrial_["integration_pass"]=success;
            finalStop(success?"complete":"failed",success?
              QString("Lane Safety: RED->GREEN/GREEN, heading awal pulih, Nav2/MPPI mencapai goal %1 m").arg(laneTarget,0,'f',1):
              QString("Goal %1 m tercapai tetapi bukti recenter/heading/authority/MPPI belum lengkap").arg(laneTarget,0,'f',1));
          }
          return;
        }

        if(finalCondition_=="center"){
          if(!gg){finalStop("invalid","GREEN/GREEN berubah sebelum target 2 m");return;}
          cmdV=0.20;cmdW=0.0;response="GREEN/GREEN -> straight 2 m";
        }else if(finalCondition_=="left_yellow"){
          if(yg&&!leftLatched){cmdV=0.20;cmdW=0.0;response="YELLOW LEFT warning-only -> straight";}
          else if(gg){cmdV=0.20;cmdW=0.0;response="YELLOW LEFT cleared -> continue straight";}
          else commandValid=false;
        }else if(finalCondition_=="left_red"){
          if(rg&&leftLatched){cmdV=0.10;cmdW=-0.040;response="RED LEFT -> recenter RIGHT";}
          else if(yg&&leftLatched){cmdV=0.10;cmdW=-0.020;response="RED LEFT latched recovery through YELLOW";}
          else if(gg){cmdV=headingRestored?0.20:0.12;cmdW=headingCmd;response=headingRestored?"RED LEFT recovered GREEN -> initial heading restored -> straight 5 m":"RED LEFT recovered GREEN -> restoring initial heading";}
          else commandValid=false;
        }else if(finalCondition_=="right_yellow"){
          if(gy&&!rightLatched){cmdV=0.20;cmdW=0.0;response="YELLOW RIGHT warning-only -> straight";}
          else if(gg){cmdV=0.20;cmdW=0.0;response="YELLOW RIGHT cleared -> continue straight";}
          else commandValid=false;
        }else if(finalCondition_=="right_red"){
          if(gr&&rightLatched){cmdV=0.10;cmdW=0.040;response="RED RIGHT -> recenter LEFT";}
          else if(gy&&rightLatched){cmdV=0.10;cmdW=0.020;response="RED RIGHT latched recovery through YELLOW";}
          else if(gg){cmdV=headingRestored?0.20:0.12;cmdW=headingCmd;response=headingRestored?"RED RIGHT recovered GREEN -> initial heading restored -> straight 5 m":"RED RIGHT recovered GREEN -> restoring initial heading";}
          else commandValid=false;
        }else commandValid=false;

        if(redTrial&&gg&&headingRestored&&!nav2Handoff){
          QString msg;
          // Release manual WEB_TRIAL authority first, then publish the fixed
          // map-frame goal. cmd_vel_router gives AUTONOMY precedence once the
          // navigation core opens its motion gate.
          bridge_->publishFinalTrialVelocity(0.0,0.0,false,&msg);
          const double gx=finalTrial_.value("goal_x_map").toDouble(std::numeric_limits<double>::quiet_NaN());
          const double gy=finalTrial_.value("goal_y_map").toDouble(std::numeric_limits<double>::quiet_NaN());
          const double gyaw=finalTrial_.value("goal_yaw_map").toDouble(std::numeric_limits<double>::quiet_NaN());
          if(!bridge_->publishGoal(gx,gy,gyaw,&msg)){finalStop("invalid",msg);return;}
          finalTrial_["lane_nav2_handoff"]=true;
          finalTrial_["lane_nav2_handoff_at_ms"]=t;
          // Manual Lane Safety commissioning is complete. From this point the
          // normal Nav2 safety chain owns motion; do not keep the special LANE
          // final-trial stream watchdog latched over the autonomous mission.
          bridge_->publishFinalGuard("IDLE","");
          finalTrial_["lane_phase"]="NAV2_GOAL_SENT";
          finalTrial_["goal_state"]="SENT";
          finalTrial_["response"]=QString("GREEN/GREEN + heading awal pulih -> Nav2 goal %1 m").arg(laneTarget,0,'f',1);
          return;
        }

        if(!commandValid){finalStop("invalid","Transisi lane tidak sesuai state/latch produksi");return;}
        if(straightTrial&&std::isfinite(steerActual)&&std::abs(steerActual)>5.0*kPi/180.0){
          finalTrial_["steering_neutral_maintained"]=false;
          finalStop("failed",QString("Trial warning/straight menghasilkan steering tak terduga %1 deg").arg(steerActual*180.0/kPi,0,'f',1));return;
        }

        if(std::abs(cmdW)>.005 && !(redTrial&&finalGreenAt_>0)){
          finalTurn_=true;
          const double expectedPhysicalSign=cmdW<0.0?1.0:-1.0; // ROS yaw sign -> physical steering sign
          if(std::isfinite(steerActual)){
            const double directional=expectedPhysicalSign*steerActual;
            if(directional>=2.0*kPi/180.0){
              if(!finalTrial_.value("steering_response_seen").toBool(false))
                finalTrial_["steering_response_ms"]=t-finalGo_;
              finalTrial_["steering_response_seen"]=true;
              finalTrial_["steering_direction_match"]=true;
            }else if(t-finalGo_>750&&steerActual*expectedPhysicalSign<=-2.0*kPi/180.0){
              finalTrial_["steering_direction_match"]=false;
              finalStop("failed",QString("Feedback steering bergerak ke arah salah (%1 deg)").arg(steerActual*180.0/kPi,0,'f',1));return;
            }
          }
          const qint64 authorityAt=static_cast<qint64>(finalTrial_.value("web_trial_authority_at_ms").toDouble(0));
          if(authorityAt>0&&t-authorityAt>1500&&!finalTrial_.value("steering_response_seen").toBool(false)){
            finalStop("failed","Command RED aktif tetapi feedback steering tidak merespons >=2 deg dalam 1.5 s");return;
          }
        }
        if(redTrial&&gg&&!finalTrial_.value("steering_response_seen").toBool(false)){
          finalStop("failed","Lane kembali GREEN/GREEN tanpa bukti respons steering RED");return;
        }

        QString msg;if(!bridge_->publishFinalTrialVelocity(cmdV,cmdW,true,&msg)){finalStop("invalid",msg);return;}
        if(finalTrial_.value("response_ms").isNull()&&std::isfinite(actualSpeed)&&std::abs(actualSpeed)>.03)
          finalTrial_["response_ms"]=t-finalGo_;
        finalTrial_["response"]=response;
        finalTrial_["lane_actual"]=left+QStringLiteral("|")+right;
        finalTrial_["lane_left_latched"]=leftLatched;finalTrial_["lane_right_latched"]=rightLatched;
        finalTrial_["command_v_mps"]=cmdV;finalTrial_["command_w_rad_s"]=cmdW;

        if(!redTrial&&forwardProgress>=laneTarget){
          bool success=false;QString successReason,failReason;
          if(finalCondition_=="center"){
            success=gg&&finalTrial_.value("steering_neutral_maintained").toBool(true)&&
              finalTrial_.value("web_trial_authority_seen").toBool(false);
            successReason="GREEN/GREEN: straight 2 m, steering tetap netral, lalu STOP";
            failReason="GREEN/GREEN mencapai 2 m tetapi steering/lane tidak tetap valid";
          }else if(warningTrial){
            const bool stateOk=finalCondition_=="left_yellow"?(yg||gg):(gy||gg);
            const bool latchOk=finalCondition_=="left_yellow"?!leftLatched:!rightLatched;
            success=stateOk&&latchOk&&finalTrial_.value("steering_neutral_maintained").toBool(true)&&
              finalTrial_.value("web_trial_authority_seen").toBool(false);
            successReason="YELLOW warning-only: 2 m tercapai tanpa steering aktif, lalu STOP";
            failReason="YELLOW mencapai 2 m tetapi warning berubah menjadi latched/steering aktif";
          }else if(redTrial){
            success=gg&&headingRestored&&finalTrial_.value("heading_restored_once").toBool(false)&&
              finalGreenAt_>0&&finalTurn_&&finalTrial_.value("steering_response_seen").toBool(false)&&
              finalTrial_.value("steering_direction_match").toBool(false)&&
              finalTrial_.value("web_trial_authority_seen").toBool(false);
            successReason=QString("RED: steering koreksi valid, recovery GREEN/GREEN, heading awal dipulihkan, target %1 m tercapai, lalu STOP").arg(laneTarget,0,'f',1);
            failReason=QString("Target %1 m tercapai tetapi heading awal / bukti steering / recovery GREEN belum lengkap").arg(laneTarget,0,'f',1);
          }
          finalTrial_["integration_pass"]=success;
          finalStop(success?"complete":"failed",success?successReason:failReason);
        }
      }else{
        // Obstacle response is Nav2-owned. Web trial never synthesizes steering here.
        QString msg;bridge_->publishFinalTrialVelocity(0.0,0.0,false,&msg);
        const bool noHuman=finalCondition_=="none";
        const QJsonObject h=finalHumanCandidate(s);
        const bool seen=!h.isEmpty();
        if(noHuman&&seen){finalStop("invalid","Human muncul pada trial NO HUMAN");return;}

        const QJsonObject ts=s.value("trajectory_safety_state").toObject();
        const QJsonObject sem=s.value("semantic_detections").toObject();
        const QJsonObject objCloud=s.value("object_points").toObject();
        const QJsonObject planningCloud=s.value("planning_relevant_points").toObject();
        const QJsonObject costmap=s.value("local_costmap_meta").toObject();
        const QJsonObject globalCostmap=s.value("global_costmap_meta").toObject();
        const QJsonObject navPath=s.value("nav_path").toObject();
        const QJsonObject mppi=s.value("mppi_trajectories").toObject();
        const QJsonObject navCmd=s.value("cmd_nav").toObject();
        const QJsonObject updated=s.value("__updated").toObject();
        const int candidateCount=static_cast<int>(std::max(ts.value("candidate_points").toDouble(0),objCloud.value("count").toDouble(0)));
        const int planningCount=static_cast<int>(std::max(ts.value("planning_relevant_points").toDouble(0),planningCloud.value("count").toDouble(0)));
        const bool costmapFresh=finalFresh(s,"local_costmap_meta",2200)&&!costmap.isEmpty();
        const bool globalCostmapFresh=finalFresh(s,"global_costmap_meta",2200)&&!globalCostmap.isEmpty();
        const bool mppiFresh=finalFresh(s,"mppi_trajectories",1800)&&mppi.value("count").toDouble(0)>0;
        const QString decision=ts.value("decision").toString().toUpper();
        const double navV=navCmd.value("linear_x").toDouble(0.0),navW=navCmd.value("angular_z").toDouble(0.0);
        const QString goalState=s.value("goal_state").toObject().value("state").toString().toUpper();
        const double target=finalTrial_.value("target_distance_m").toDouble(0.0);
        const bool reached=goalState.contains("SUCCEEDED")||(target>0&&forwardProgress>=std::max(0.0,target-0.25));

        const qint64 navPathUpdated=static_cast<qint64>(updated.value("nav_path").toDouble(0));
        const qint64 mppiUpdated=static_cast<qint64>(updated.value("mppi_trajectories").toDouble(0));
        const qint64 globalCostmapUpdated=static_cast<qint64>(updated.value("global_costmap_meta").toDouble(0));
        const qint64 navPathAge=navPathUpdated>0&&t>=navPathUpdated?t-navPathUpdated:std::numeric_limits<qint64>::max();
        const int navPathCount=static_cast<int>(navPath.value("count").toDouble(0));
        const double navPathLength=navPath.value("length_m").toDouble(std::numeric_limits<double>::quiet_NaN());
        const double navPathHeadingVar=navPath.value("heading_variation_rad").toDouble(std::numeric_limits<double>::quiet_NaN());
        const double maxReasonablePath=std::max(4.0,target+3.0);
        const bool navPathFresh=navPathUpdated>0&&navPathAge>=0&&navPathAge<4000;
        const bool navPathQuality=navPathCount>1&&std::isfinite(navPathLength)&&navPathLength>0.05&&
          navPathLength<=maxReasonablePath&&
          (!std::isfinite(navPathHeadingVar)||navPathHeadingVar<=3.50);
        const bool localCostmapMarked=costmap.value("nonzero_cells").toDouble(0)>0&&costmap.value("max_cost").toDouble(0)>0;
        const bool localCostmapAligned=planningCount>0&&costmapFresh&&localCostmapMarked&&
          costmap.value("planning_alignment_pass").toBool(false);
        const bool obstacleObject=finalTrial_.value("object_cloud_seen").toBool(false)||candidateCount>0;
        const bool planningSeen=finalTrial_.value("planning_cloud_seen").toBool(false)||planningCount>0;
        const bool costmapSeen=finalTrial_.value("costmap_marking_seen").toBool(false)||localCostmapAligned;
        const bool mppiSeen=finalTrial_.value("mppi_seen").toBool(false)||mppiFresh;

        finalTrial_["planner_path_age_ms"]=navPathUpdated>0?QJsonValue(static_cast<double>(navPathAge)):QJsonValue();
        finalTrial_["planner_path_count"]=navPathCount;
        finalTrial_["planner_path_length_m"]=std::isfinite(navPathLength)?QJsonValue(navPathLength):QJsonValue();
        finalTrial_["planner_path_heading_variation_rad"]=std::isfinite(navPathHeadingVar)?QJsonValue(navPathHeadingVar):QJsonValue();
        finalTrial_["planner_path_quality_ok"]=navPathQuality;
        finalTrial_["trial_person_only_planning_filter"]=sem.value("trial_person_only_planning_filter");
        finalTrial_["goal_state"]=goalState;

        const bool humanPlannerTrial=!noHuman&&finalCondition_!="1";
        const bool memoryLatched=sem.value("obstacle_memory_latched").toBool(false);
        const bool memoryReleased=sem.value("obstacle_memory_released").toBool(false);
        qint64 latchGateAt=static_cast<qint64>(finalTrial_.value("obstacle_memory_latched_gate_at_ms").toDouble(0));
        if(humanPlannerTrial&&memoryLatched&&latchGateAt<=0){
          latchGateAt=t;
          finalTrial_["obstacle_memory_latched_gate_at_ms"]=t;
        }

        // PRE-MOTION PLANNER GATE:
        // 1) HUMAN must be world-fixed, 2) only HUMAN may feed planning during
        // formal 4.5A, 3) local/global costmaps must update after the latch,
        // 4) a NEW non-looping global path must be published after that update,
        // 5) MPPI must consume that new path. Until all are true, actuator
        // output stays zero through OBSTACLE_GATE.
        if(humanPlannerTrial&&!finalTrial_.value("obstacle_motion_released").toBool(false)){
          const qint64 gateStart=static_cast<qint64>(finalTrial_.value("planner_gate_started_ms").toDouble(finalGo_));
          const bool personOnly=sem.value("trial_person_only_planning_filter").toBool(false);
          const bool globalAfterLatch=latchGateAt>0&&globalCostmapFresh&&globalCostmapUpdated>latchGateAt;
          const bool pathAfterGlobal=globalAfterLatch&&navPathFresh&&navPathUpdated>globalCostmapUpdated&&navPathQuality;
          const bool mppiAfterPath=pathAfterGlobal&&mppiFresh&&mppiUpdated>=navPathUpdated;
          const bool gateReady=memoryLatched&&personOnly&&localCostmapAligned&&globalAfterLatch&&pathAfterGlobal&&mppiAfterPath;

          QString gateReason;
          if(!memoryLatched)gateReason="WAIT_HUMAN_WORLD_MEMORY";
          else if(!personOnly)gateReason="WAIT_PERSON_ONLY_PLANNING_FILTER";
          else if(planningCount<=0)gateReason="WAIT_HUMAN_PLANNING_POINTS";
          else if(!localCostmapAligned)gateReason="WAIT_LOCAL_COSTMAP_ALIGNMENT";
          else if(!globalAfterLatch)gateReason="WAIT_GLOBAL_COSTMAP_POST_LATCH";
          else if(!navPathFresh)gateReason="WAIT_NEW_GLOBAL_PATH";
          else if(!navPathQuality)gateReason="REJECT_LOOP_OR_EXCESSIVE_GLOBAL_PATH";
          else if(!pathAfterGlobal)gateReason="WAIT_POST_COSTMAP_GLOBAL_PATH";
          else if(!mppiAfterPath)gateReason="WAIT_MPPI_POST_PATH";
          else gateReason="READY";

          finalTrial_["planner_gate_reason"]=gateReason;
          finalTrial_["planner_gate_elapsed_ms"]=std::max<qint64>(0,t-gateStart);
          finalTrial_["planner_gate_planning_points"]=planningCount;
          finalTrial_["planner_gate_local_costmap_aligned"]=localCostmapAligned;
          finalTrial_["planner_gate_global_costmap_post_latch"]=globalAfterLatch;
          finalTrial_["planner_gate_path_post_costmap"]=pathAfterGlobal;
          finalTrial_["planner_gate_mppi_post_path"]=mppiAfterPath;
          finalTrial_["command_v_mps"]=0.0;finalTrial_["command_w_rad_s"]=0.0;

          if(!gateReady){
            bridge_->publishFinalGuard("OBSTACLE_GATE",finalCondition_);
            finalTrial_["response"]="HOLD STOP -> "+gateReason;
            if(gateStart>0&&t-gateStart>15000){
              finalStop("failed","Planner gate timeout: "+gateReason+
                QString(" | path=%1 m headingVar=%2 rad").arg(navPathLength,0,'f',2).arg(navPathHeadingVar,0,'f',2));
            }
            return;
          }

          finalTrial_["planner_gate_pass"]=true;
          finalTrial_["planner_gate_passed_ms"]=t;
          finalTrial_["planner_gate_reason"]="PASS";
          finalTrial_["obstacle_motion_released"]=true;
          finalTrial_["obstacle_motion_released_at_ms"]=t;
          finalTrial_["planner_gate_path_length_m"]=navPathLength;
          finalTrial_["planner_gate_path_heading_variation_rad"]=navPathHeadingVar;
          // Rebase physical-travel evidence at the instant motion authority is
          // released. Planning time while STOP must not look like response time.
          finalGo_=t;finalTrial_["go_at_ms"]=t;finalTravel_=0.0;
          const auto gateOd=s.value("esc_odom").toObject();
          const double gateX=finalNumber(gateOd,"x"),gateY=finalNumber(gateOd,"y");
          if(std::isfinite(gateX)&&std::isfinite(gateY)){
            finalX_=finalPrevX_=gateX;finalY_=finalPrevY_=gateY;
          }
          bridge_->publishFinalGuard("OBSTACLE",finalCondition_);
          finalTrial_["response"]="PLANNER GATE PASS -> MOTION RELEASED";
          return;
        }

        // RUN-TIME PLANNER FAIL-SAFE:
        // If the planner stops publishing or produces an implausible looping
        // path, immediately HOLD motion but keep the Nav2 goal alive so a fresh
        // replan can recover. A new path must reach MPPI before motion resumes.
        if(humanPlannerTrial&&finalTrial_.value("obstacle_motion_released").toBool(false)&&!reached){
          const bool holdActive=finalTrial_.value("planner_hold_active").toBool(false);
          if(holdActive){
            const qint64 holdStart=static_cast<qint64>(finalTrial_.value("planner_hold_started_ms").toDouble(t));
            const bool obstacleStillProtected=memoryLatched&&!memoryReleased;
            const bool costmapReadyForResume=!obstacleStillProtected||localCostmapAligned;
            const bool newPath=navPathFresh&&navPathQuality&&navPathUpdated>holdStart;
            const bool newMppi=newPath&&mppiFresh&&mppiUpdated>=navPathUpdated;
            finalTrial_["planner_hold_age_ms"]=std::max<qint64>(0,t-holdStart);
            finalTrial_["response"]="PLANNER HOLD STOP -> WAIT FRESH PATH";
            bridge_->publishFinalGuard("OBSTACLE_GATE",finalCondition_);
            if(costmapReadyForResume&&newPath&&newMppi){
              finalTrial_["planner_hold_active"]=false;
              finalTrial_["planner_hold_recovery_count"]=finalTrial_.value("planner_hold_recovery_count").toInt(0)+1;
              finalTrial_["planner_hold_recovered_ms"]=t;
              finalTrial_["planner_hold_reason"]="RECOVERED";
              bridge_->publishFinalGuard("OBSTACLE",finalCondition_);
              finalTrial_["response"]="PLANNER RECOVERED -> MOTION RESUMED";
              return;
            }
            if(t-holdStart>12000){
              finalStop("failed","Planner tidak pulih dalam 12 s setelah HOLD");return;
            }
            return;
          }

          if(!navPathFresh||!navPathQuality){
            const QString holdReason=!navPathFresh?QStringLiteral("GLOBAL_PATH_STALE"):
              QStringLiteral("LOOP_OR_EXCESSIVE_GLOBAL_PATH");
            finalTrial_["planner_hold_active"]=true;
            finalTrial_["planner_hold_started_ms"]=t;
            finalTrial_["planner_hold_count"]=finalTrial_.value("planner_hold_count").toInt(0)+1;
            finalTrial_["planner_hold_reason"]=holdReason;
            finalTrial_["response"]="PLANNER FAIL-SAFE HOLD -> "+holdReason;
            bridge_->publishFinalGuard("OBSTACLE_GATE",finalCondition_);
            return;
          }
        }

        const bool avoidNow=planningCount>0&&(std::abs(navW)>0.02||decision.startsWith("AVOID_")||decision.startsWith("SEARCH_AVOIDANCE_"));
        const bool avoidSeen=finalTrial_.value("avoidance_seen").toBool(false)||avoidNow;
        if(avoidNow){
          finalTurn_=true;finalAvoid_=true;
          if(decision=="AVOID_LEFT"||navW>0.02)finalTrial_["avoid_direction"]="LEFT";
          else if(decision=="AVOID_RIGHT"||navW<-0.02)finalTrial_["avoid_direction"]="RIGHT";
        }
        const double reference=finalObstacleDistance_;
        const bool repeatTrial=QRegularExpression(QStringLiteral("^trial[1-3]$")).match(finalCondition_).hasMatch();
        const bool recoveryProgressOk=repeatTrial?forwardProgress>1.0:forwardProgress>reference+0.5;
        const bool recoveryNow=!noHuman&&avoidSeen&&planningCount==0&&recoveryProgressOk&&std::abs(navW)<0.03;
        const bool recoverySeen=finalTrial_.value("recovery_seen").toBool(false)||recoveryNow;
        if(recoveryNow&&finalTrial_.value("recovery_ms").isNull())finalTrial_["recovery_ms"]=t-finalGo_;

        finalTrial_["object_cloud_seen"]=obstacleObject;
        finalTrial_["obstacle_memory_latched_seen"]=
          finalTrial_.value("obstacle_memory_latched_seen").toBool(false)||
          sem.value("obstacle_memory_latched").toBool(false);
        finalTrial_["obstacle_memory_released_seen"]=
          finalTrial_.value("obstacle_memory_released_seen").toBool(false)||
          sem.value("obstacle_memory_released").toBool(false);
        finalTrial_["obstacle_memory_latched"]=sem.value("obstacle_memory_latched");
        finalTrial_["obstacle_memory_released"]=sem.value("obstacle_memory_released");
        finalTrial_["obstacle_memory_base_forward_m"]=sem.value("obstacle_memory_base_forward_m");
        finalTrial_["obstacle_memory_odom_x"]=sem.value("obstacle_memory_odom_x");
        finalTrial_["obstacle_memory_odom_y"]=sem.value("obstacle_memory_odom_y");
        finalTrial_["planning_cloud_seen"]=planningSeen;
        finalTrial_["costmap_marking_seen"]=costmapSeen;
        finalTrial_["mppi_seen"]=mppiSeen;
        finalTrial_["avoidance_seen"]=avoidSeen;
        finalTrial_["recovery_seen"]=recoverySeen;
        finalTrial_["safety_decision_last"]=decision;
        finalTrial_["command_v_mps"]=navV;finalTrial_["command_w_rad_s"]=navW;
        finalTrial_["human_visible"]=seen;
        if(seen)finalTrial_["human_last_confidence"]=h.value("score");
        if(finalTrial_.value("response_ms").isNull()&&std::isfinite(actualSpeed)&&std::abs(actualSpeed)>.03)
          finalTrial_["response_ms"]=t-finalGo_;

        if(goalState.contains("ABORT")||goalState.contains("REJECT")||goalState.contains("FAIL")){
          finalStop("failed","Nav2 goal gagal: "+goalState);return;
        }
        if(noHuman){
          finalTrial_["response"]="NO HUMAN -> NAV2 STRAIGHT 2M";
          if(reached){
            finalTrial_["integration_pass"]=mppiSeen;
            finalStop(mppiSeen?"complete":"failed",mppiSeen?"NO HUMAN: Nav2 mencapai target 2 m dan MPPI aktif":"NO HUMAN: target tercapai tetapi bukti MPPI tidak tersedia");
          }
        }else{
          finalTrial_["response"]=planningCount>0?QStringLiteral("NAV2/MPPI AVOIDANCE ACTIVE"):QStringLiteral("NAV2 FOLLOW/RECOVERY");
          const bool pass=finalTrial_.value("metric_position_valid").toBool(false)&&obstacleObject&&planningSeen&&costmapSeen&&mppiSeen&&avoidSeen&&recoverySeen;
          finalTrial_["integration_pass"]=pass;
          if(reached){
            finalStop(pass?"complete":"failed",pass?
              QStringLiteral("Human masuk planning costmap, MPPI menghindar, recovery tervalidasi, target tercapai"):
              QStringLiteral("Target tercapai tetapi rantai obstacle->costmap->MPPI->avoidance->recovery belum lengkap"));
          }
        }
        const double hardLimit=target+1.5;
        if(target>0&&forwardProgress>hardLimit)finalStop("failed",QString("Batas aman trial terlewati (%1 m)").arg(hardLimit,0,'f',1));
      }
      return;
    }
    if(phase=="stopping"){
      bridge_->publishFinalGuard("STOP",finalCondition_);
      QString stopMsg;bridge_->publishFinalTrialVelocity(0.0,0.0,false,&stopMsg);
      const double v=finalNumber(s,"esc_drive_actual");
      const bool stopped=finalFresh(s,"esc_drive_actual",500)&&std::isfinite(v)&&std::abs(v)<.03;
      if(stopped){if(!finalStillAt_)finalStillAt_=t;}else finalStillAt_=0;
      if((finalStillAt_&&t-finalStillAt_>=600)||t-finalStopAt_>4000){
        finalTrial_["stopped"]=stopped;finalTrial_["phase"]=stopped?finalTargetResult_:QString("invalid");
        if(!stopped)finalTrial_["reason"]="Tidak dapat memverifikasi kendaraan berhenti";
        finalTrial_["duration_s"]=finalGo_?(t-finalGo_)/1000.0:0;finalTrial_["travel_m"]=finalTravel_;finalPersist();
        Bab4PerceptionSessionStore store(agvRootPath(), loadConfigSnapshot());
        const QJsonObject activeReply=store.state();
        const QString activeSession=activeReply.value("state").toObject().value("session_id").toString();
        if(activeSession==finalSessionId_){
          const QJsonObject saved=store.recordArmed(finalMode_, finalCondition_, finalTrial_, s, bridge_->cameraJpeg());
          if(!saved.value("ok").toBool(false)){
            const QString m=saved.value("message").toString(QStringLiteral("Gagal menyimpan hasil ARMED"));
            finalTrial_["persistence_error"]=m;finalTrial_["phase"]="invalid";
            finalTrial_["reason"]=QStringLiteral("Research persistence failed: ")+m;finalPersist();
          }
        }else{
          finalTrial_["persistence_error"]="SESSION_CHANGED_BEFORE_RECORD";finalTrial_["phase"]="invalid";
          finalTrial_["reason"]="Research persistence failed: session berubah sebelum record";finalPersist();
        }
        // STOP hanya diperlukan selama kendaraan benar-benar berhenti. Setelah
        // terminal state tersimpan, lepaskan guard ke IDLE agar STOP transient-local
        // tidak meracuni trial berikutnya atau command Nav2 setelah abort/complete.
        bridge_->publishFinalGuard("IDLE","");
      }
    }
  }

  WebRosBridge *bridge_{nullptr};
  QTcpServer server_;
  QTimer eventTimer_;
  QTimer bindRetryTimer_;
  QTimer recordingTimer_;
  QTimer hostTimer_;
  QProcess rm3100CalibrationProcess_;
  QString rm3100CalibrationLastOutput_;
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
  QStringList lastGraphTitles_;
  QMap<QString, QJsonObject> validationSnapshots_;
  QMap<QString, QJsonObject> configProposals_;

  QString navigationSharePath() const {
    try { return QString::fromStdString(ament_index_cpp::get_package_share_directory("navigation")); }
    catch (...) { return agvPath(QStringLiteral("install/navigation/share/navigation")); }
  }

  QJsonObject rm3100CalibrationStatus() const {
    QJsonObject out{{"running", rm3100CalibrationProcess_.state()!=QProcess::NotRunning},
                    {"process_state", static_cast<int>(rm3100CalibrationProcess_.state())},
                    {"last_output", rm3100CalibrationLastOutput_.right(2000)}};
    const QString statePath=agvPath(QStringLiteral("calibration/rm3100_calibration_state.json"));
    QFile sf(statePath);
    if (sf.open(QIODevice::ReadOnly)) {
      QJsonParseError e{}; const auto d=QJsonDocument::fromJson(sf.readAll(),&e);
      if (e.error==QJsonParseError::NoError && d.isObject())
        for (auto it=d.object().constBegin(); it!=d.object().constEnd(); ++it) out[it.key()]=it.value();
    }
    const QString fitPath=agvPath(QStringLiteral("calibration/rm3100_ardupilot_latest.yaml"));
    if (QFileInfo(fitPath).isFile()) {
      try { out["fit"] = yamlToJson(YAML::LoadFile(fitPath.toStdString())); }
      catch (const std::exception &e) { out["fit_error"]=QString::fromUtf8(e.what()); }
    }
    QString stationaryReason;
    const bool stationaryOk = bridge_ && bridge_->vehicleStationary(&stationaryReason);
    const QJsonObject snap = bridge_ ? bridge_->snapshot() : QJsonObject{};
    const bool sensorOnline = snap.value("connected.rm3100").toBool(false);
    out["stationary_ok"] = stationaryOk;
    out["stationary_reason"] = stationaryOk ? QStringLiteral("Kendaraan diam dan command netral.") : stationaryReason;
    out["sensor_online"] = sensorOnline;
    out["wire_semantics"]="/neo3pro/mag = DroneCAN/AP_Periph field before ROS-host correction; native RM3100 register raw is not exposed by this transport";
    return out;
  }

  bool setRm3100CalibrationFace(const QString &requestedFace, QString *message) {
    if (rm3100CalibrationProcess_.state()==QProcess::NotRunning) { if(message)*message="Kalibrasi RM3100 belum berjalan"; return false; }
    const QString face=requestedFace.trimmed().toUpper();
    static const QSet<QString> allowed{QStringLiteral("LEVEL"),QStringLiteral("LEFT_SIDE"),QStringLiteral("RIGHT_SIDE"),QStringLiteral("NOSE_UP"),QStringLiteral("NOSE_DOWN"),QStringLiteral("INVERTED")};
    if (!allowed.contains(face)) { if(message)*message="Fase RM3100 tidak valid"; return false; }
    const QString path=agvPath(QStringLiteral("calibration/rm3100_calibration_control.json"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly|QIODevice::Text)) { if(message)*message="Gagal membuka control RM3100"; return false; }
    const QJsonObject payload{{"target_face",face},{"updated_at_ms",nowMs()}};
    file.write(QJsonDocument(payload).toJson(QJsonDocument::Compact));
    if (!file.commit()) { if(message)*message="Gagal commit control RM3100"; return false; }
    if(message)*message=QStringLiteral("RM3100 target face → ")+face;
    return true;
  }

  bool startRm3100Calibration(QString *message) {
    if (bridge_->readOnly()) { if(message)*message="Web GUI read-only"; return false; }
    if (rm3100CalibrationProcess_.state()!=QProcess::NotRunning) { if(message)*message="Kalibrasi RM3100 masih berjalan"; return false; }
    QString stationary; if (!bridge_->vehicleStationary(&stationary)) { if(message)*message=stationary; return false; }
    const QJsonObject snap=bridge_->snapshot();
    if (!snap.value("connected.rm3100").toBool(false)) {
      if (message) *message = "RM3100/NEO3 Pro belum online";
      return false;
    }
    const QString script=agvPath(QStringLiteral("src/navigation/tools/rm3100_ardupilot_calibration.py"));
    if (!QFileInfo(script).isFile()) { if(message)*message="Backend RM3100 calibration tidak ditemukan: "+script; return false; }
    rm3100CalibrationLastOutput_.clear();
    rm3100CalibrationProcess_.setProgram(agvPythonPath());
    rm3100CalibrationProcess_.setArguments({script,QStringLiteral("--workspace"),agvRootPath(),QStringLiteral("--topic"),QStringLiteral("/neo3pro/mag"),QStringLiteral("--imu-topic"),QStringLiteral("/imu/data")});
    rm3100CalibrationProcess_.setWorkingDirectory(agvRootPath());
    rm3100CalibrationProcess_.start();
    if (!rm3100CalibrationProcess_.waitForStarted(1500)) { if(message)*message="Gagal start recorder RM3100"; return false; }
    if(message)*message="RM3100 full-3D recorder dimulai. Ikuti enam orientasi dan putar perlahan 360 derajat pada setiap orientasi.";
    return true;
  }

  bool stopRm3100Calibration(QString *message) {
    if (rm3100CalibrationProcess_.state()==QProcess::NotRunning) { if(message)*message="Tidak ada kalibrasi RM3100 aktif"; return false; }
    rm3100CalibrationProcess_.terminate();
    if (!rm3100CalibrationProcess_.waitForFinished(8000)) { rm3100CalibrationProcess_.kill(); rm3100CalibrationProcess_.waitForFinished(1000); if(message)*message="RM3100 fit timeout; process dihentikan"; return false; }
    const QJsonObject st=rm3100CalibrationStatus();
    const bool pass=st.value("status").toString()==QStringLiteral("PASS");
    if(message)*message=pass?QStringLiteral("RM3100 full-3D fit PASS; review parameter dan Stage ke ROS melalui Diff Review"):st.value("instruction").toString(QStringLiteral("RM3100 fit belum PASS"));
    return pass;
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

  bool buildPerceptionZip(const QStringList &folders, const QString &zipName, QByteArray *bytes, QString *message) const {
    if (!bytes) return false;
    const QString rootPath = QFileInfo(agvPath(QStringLiteral("data/presepsi"))).canonicalFilePath();
    if (rootPath.isEmpty()) { if (message) *message = QStringLiteral("Folder data/presepsi belum tersedia"); return false; }
    QStringList names;
    for (const QString &folder : folders) {
      const QFileInfo info(folder); const QString canonical = info.canonicalFilePath();
      if (canonical.isEmpty() || !info.isDir() || !canonical.startsWith(rootPath + QDir::separator())) continue;
      const QString relative = QDir(rootPath).relativeFilePath(canonical);
      if (!relative.startsWith(QStringLiteral(".."))) names << relative;
    }
    names.removeDuplicates();
    if (names.isEmpty()) { if (message) *message = QStringLiteral("Belum ada folder hasil Persepsi untuk di-ZIP"); return false; }
    QString safeName = zipName; safeName.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_.-]+")), QStringLiteral("_"));
    if (!safeName.endsWith(QStringLiteral(".zip"), Qt::CaseInsensitive)) safeName += QStringLiteral(".zip");
    const QString zipPath = QDir(QStringLiteral("/tmp")).filePath(QStringLiteral("agv_") + QString::number(QCoreApplication::applicationPid()) + QStringLiteral("_") + safeName);
    QFile::remove(zipPath);
    QProcess proc; proc.setProgram(QStringLiteral("/usr/bin/zip")); proc.setWorkingDirectory(rootPath);
    QStringList args{QStringLiteral("-r"), QStringLiteral("-q"), zipPath}; args << names; proc.setArguments(args); proc.start();
    const bool started = proc.waitForStarted(3000), done = started && proc.waitForFinished(120000);
    if (!done || proc.exitStatus()!=QProcess::NormalExit || proc.exitCode()!=0) {
      if (message) *message = QStringLiteral("ZIP gagal: ") + QString::fromUtf8(proc.readAllStandardError()).left(500);
      QFile::remove(zipPath);
      return false;
    }
    QFile f(zipPath); if (!f.open(QIODevice::ReadOnly)) { if (message) *message = QStringLiteral("ZIP selesai dibuat tetapi gagal dibaca"); QFile::remove(zipPath); return false; }
    *bytes = f.readAll(); f.close(); QFile::remove(zipPath);
    if (bytes->isEmpty()) { if (message) *message = QStringLiteral("ZIP kosong"); return false; }
    if (message) *message = QStringLiteral("ZIP siap: ") + names.join(QStringLiteral(", "));
    return true;
  }

  QStringList latestPerceptionFinalFolders() const {
    const QDir root(agvPath(QStringLiteral("data/presepsi"))); QStringList out;
    for (int i=1;i<=5;++i) {
      const QString prefix = QStringLiteral("BAB4_F4.%1_").arg(i);
      const QFileInfoList entries = root.entryInfoList(QStringList{prefix + QStringLiteral("*")}, QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
      if (!entries.isEmpty()) out << entries.first().absoluteFilePath();
    }
    return out;
  }

  QString navigationDataPackageRoot() const {
    return agvPath(QStringLiteral("data/navigasi/packages"));
  }

  static QJsonObject readJsonObjectFile(const QString &path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return {};
    QJsonParseError pe{};
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    return pe.error == QJsonParseError::NoError && doc.isObject() ? doc.object() : QJsonObject{};
  }

  QJsonArray navigationDataPackages() const {
    const QDir root(navigationDataPackageRoot());
    QJsonArray out;
    if (!root.exists()) return out;
    const QFileInfoList dirs = root.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
    for (const QFileInfo &dirInfo : dirs) {
      QJsonObject one = readJsonObjectFile(QDir(dirInfo.absoluteFilePath()).filePath(QStringLiteral("package.json")));
      if (one.isEmpty()) continue;
      const QString zip = one.value(QStringLiteral("zip_path")).toString();
      const QFileInfo zi(zip);
      one[QStringLiteral("zip_exists")] = zi.isFile();
      one[QStringLiteral("bytes")] = zi.isFile() ? static_cast<double>(zi.size()) : 0.0;
      out.append(one);
    }
    return out;
  }

  bool buildNavigationBab4Package(const QJsonObject &trialSummary, const QJsonObject &clientArtifacts,
                                  QJsonObject *recordResult, QString *message) const {
    if (!recordResult || recordingSubsystem_ != QStringLiteral("navigation") ||
        recordingSourceExperimentId_ != QStringLiteral("R4.4.5")) return true;
    const QString sourceFolder = recordResult->value(QStringLiteral("report_folder")).toString();
    const QString primaryCsv = recordResult->value(QStringLiteral("primary_csv")).toString();
    const QString masterXlsx = recordResult->value(QStringLiteral("xlsx_path")).toString();
    const QString manifest = recordResult->value(QStringLiteral("manifest_path")).toString();
    const QJsonObject analysis = clientArtifacts.value(QStringLiteral("analysis_summary")).toObject();
    if (sourceFolder.isEmpty() || primaryCsv.isEmpty() || masterXlsx.isEmpty()) {
      if (message) *message = QStringLiteral("Package 4.2–4.4 gagal: artifact utama belum lengkap");
      return false;
    }
    if (!QDir().mkpath(navigationDataPackageRoot())) {
      if (message) *message = QStringLiteral("Package root tidak dapat dibuat");
      return false;
    }
    QJsonObject descriptor{
      {QStringLiteral("source_folder"), sourceFolder},
      {QStringLiteral("packages_root"), navigationDataPackageRoot()},
      {QStringLiteral("primary_csv"), primaryCsv},
      {QStringLiteral("master_xlsx"), masterXlsx},
      {QStringLiteral("session_manifest"), manifest},
      {QStringLiteral("table_csv_paths"), clientArtifacts.value(QStringLiteral("table_csv_paths"))},
      {QStringLiteral("table_entries"), clientArtifacts.value(QStringLiteral("table_entries"))},
      {QStringLiteral("graph_png_paths"), recordResult->value(QStringLiteral("graph_png_paths"))},
      {QStringLiteral("graph_png_titles"), recordResult->value(QStringLiteral("graph_png_titles"))},
      {QStringLiteral("browser_graph_png_count"), recordResult->value(QStringLiteral("browser_graph_png_count"))},
      {QStringLiteral("browser_graph_png_indices"), recordResult->value(QStringLiteral("browser_graph_png_indices"))},
      {QStringLiteral("expected_graph_count"), recordResult->value(QStringLiteral("expected_graph_count"))},
      {QStringLiteral("strict_real_field"), analysis.value(QStringLiteral("strict_real_field"))},
      {QStringLiteral("table_origin"), analysis.value(QStringLiteral("table_origin"))},
      {QStringLiteral("graph_origin"), analysis.value(QStringLiteral("graph_origin"))},
      {QStringLiteral("synthetic_fill"), analysis.value(QStringLiteral("synthetic_fill"))},
      {QStringLiteral("mission_id"), analysis.value(QStringLiteral("mission_id"))},
      {QStringLiteral("mission_result"), analysis.value(QStringLiteral("mission_result"))},
      {QStringLiteral("table_continuity"), analysis.value(QStringLiteral("table_continuity"))},
      {QStringLiteral("candidate"), trialSummary.value(QStringLiteral("candidate"))},
      {QStringLiteral("cycle"), analysis.value(QStringLiteral("cycle"))},
      {QStringLiteral("trial_id"), trialSummary.value(QStringLiteral("trial_id"))},
      {QStringLiteral("source_experiment_id"), recordingSourceExperimentId_}
    };
    const QString token = QString::number(QCoreApplication::applicationPid()) + QStringLiteral("_") +
                          QString::number(QDateTime::currentMSecsSinceEpoch());
    const QString tmp = QDir(QStringLiteral("/tmp")).filePath(QStringLiteral("agv_bab4_package_") + token + QStringLiteral(".json"));
    QSaveFile df(tmp);
    if (!df.open(QIODevice::WriteOnly) ||
        df.write(QJsonDocument(descriptor).toJson(QJsonDocument::Compact)) < 0 || !df.commit()) {
      if (message) *message = QStringLiteral("Descriptor package gagal ditulis");
      return false;
    }
    QProcess proc;
    proc.setProgram(agvPythonPath());
    proc.setArguments({agvPath(QStringLiteral("src/navigation/tools/package_bab4_navigation.py")),
                       QStringLiteral("--descriptor"), tmp});
    proc.start();
    const bool started = proc.waitForStarted(3000);
    const bool done = started && proc.waitForFinished(120000);
    const QByteArray stdoutBytes = proc.readAllStandardOutput();
    const QByteArray stderrBytes = proc.readAllStandardError();
    QFile::remove(tmp);
    if (!done || proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0) {
      if (message) *message = QStringLiteral("Package builder gagal: ") + QString::fromUtf8(stderrBytes).left(700);
      return false;
    }
    QJsonParseError pe{};
    const QJsonDocument doc = QJsonDocument::fromJson(stdoutBytes.trimmed(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
      if (message) *message = QStringLiteral("Output package builder tidak valid");
      return false;
    }
    const QJsonObject package = doc.object();
    (*recordResult)[QStringLiteral("data_package")] = package;
    (*recordResult)[QStringLiteral("package_ready")] = package.value(QStringLiteral("status")).toString() == QStringLiteral("READY");
    (*recordResult)[QStringLiteral("package_download_url")] = package.value(QStringLiteral("download_url"));
    if (message) *message = QStringLiteral("BAB4 package ") + package.value(QStringLiteral("status")).toString() +
                            QStringLiteral(" • ") + package.value(QStringLiteral("package_id")).toString();
    return true;
  }

  bool deleteNavigationDataPackage(const QJsonObject &json, QString *message, QJsonObject *result) {
    QString id = json.value(QStringLiteral("package_id")).toString().trimmed();
    if (!QRegularExpression(QStringLiteral("^BAB4_[A-Za-z0-9_.-]+$")).match(id).hasMatch()) {
      if (message) *message = QStringLiteral("Package ID tidak valid");
      return false;
    }
    const QString root = QFileInfo(navigationDataPackageRoot()).absoluteFilePath();
    const QString packageDir = QDir(root).filePath(id);
    const QFileInfo pi(packageDir);
    const QString canonical = pi.canonicalFilePath();
    if (canonical.isEmpty() || !pi.isDir() || !canonical.startsWith(root + QDir::separator())) {
      if (message) *message = QStringLiteral("Package tidak ditemukan");
      return false;
    }
    const QJsonObject meta = readJsonObjectFile(QDir(canonical).filePath(QStringLiteral("package.json")));
    bool sourceDeleted = false, trialDeleted = false;
    QString trialDeleteMessage;
    if (json.value(QStringLiteral("delete_source")).toBool(true)) {
      const QString source = meta.value(QStringLiteral("source_folder")).toString();
      const QString navRoot = QFileInfo(agvPath(QStringLiteral("data/navigasi"))).absoluteFilePath();
      const QFileInfo si(source);
      const QString sourceCanonical = si.canonicalFilePath();
      if (!sourceCanonical.isEmpty() && si.isDir() &&
          sourceCanonical.startsWith(navRoot + QDir::separator()) &&
          QFileInfo(sourceCanonical).fileName().startsWith(QStringLiteral("BAB4_MISSION_4_2_4_4_"))) {
        sourceDeleted = QDir(sourceCanonical).removeRecursively();
      }
      const QString trialId = meta.value(QStringLiteral("trial_id")).toString();
      const QString sourceId = meta.value(QStringLiteral("source_experiment_id")).toString(QStringLiteral("R4.4.5"));
      if (!trialId.isEmpty()) {
        QJsonObject ignored;
        trialDeleted = deleteTrial(QJsonObject{{QStringLiteral("subsystem"), QStringLiteral("navigation")},
                                               {QStringLiteral("id"), sourceId},
                                               {QStringLiteral("trial_id"), trialId}},
                                   &trialDeleteMessage, &ignored);
      }
    }
    const bool packageDeleted = QDir(canonical).removeRecursively();
    if (!packageDeleted) {
      if (message) *message = QStringLiteral("Folder package gagal dihapus");
      return false;
    }
    if (result) *result = QJsonObject{{QStringLiteral("package_id"), id},
                                      {QStringLiteral("package_deleted"), true},
                                      {QStringLiteral("source_deleted"), sourceDeleted},
                                      {QStringLiteral("trial_deleted"), trialDeleted}};
    if (message) *message = QStringLiteral("Package dihapus") +
      (sourceDeleted ? QStringLiteral(" + raw mission") : QString()) +
      (trialDeleted ? QStringLiteral(" + trial metadata") : QString());
    return true;
  }

  void handle(QTcpSocket *socket, const Request &request) {
    if (request.method == "GET" && request.path == "/api/events") return openSse(socket);
    if (request.method == "GET" && request.path == "/api/state") return sendJson(socket, 200, bridge_->snapshot());
    if (request.method == "GET" && request.path == "/api/health") {
      return sendJson(socket, 200, QJsonObject{{"ok", true}, {"ros", true}, {"read_only", bridge_->readOnly()},
                                               {"server_time_ms", nowMs()}});
    }
    if (request.method == "GET" && request.path == "/api/perception/bab4/state") {
      Bab4PerceptionSessionStore store(agvRootPath(), loadConfigSnapshot());
      return sendJson(socket, 200, store.state());
    }
    if (request.method == "GET" && request.path.startsWith("/api/perception/bab4/export/")) {
      Bab4PerceptionSessionStore store(agvRootPath(), loadConfigSnapshot());
      const QJsonObject stateReply = store.state();
      if (!stateReply.value("ok").toBool()) return sendJson(socket, 409, stateReply);
      const QString folder = store.activeSessionPath();
      if (folder.isEmpty() || !QDir(folder).exists()) return sendText(socket,404,"text/plain; charset=utf-8","Session Bab 4 tidak tersedia");
      const QString target = request.path.mid(QString("/api/perception/bab4/export/").size());
      const QJsonObject state=stateReply.value("state").toObject();
      const QJsonObject subs=state.value("subchapters").toObject();
      auto count=[&](const QString &sub,const QString &cond)->int{
        return subs.value(sub).toObject().value("conditions").toObject().value(cond).toObject().value("sample_count").toInt(0);
      };
      auto status=[&](const QString &sub,const QString &cond)->QString{
        return subs.value(sub).toObject().value("conditions").toObject().value(cond).toObject().value("status").toString();
      };
      auto ready=[&](const QString &sub)->bool{
        if(sub=="4.1"){
          const QJsonArray rows=subs.value("4.1").toObject().value("conditions").toObject().value("validation").toObject().value("rows").toArray();
          if(rows.isEmpty()) return false;
          const QJsonObject r=rows.last().toObject();
          return r.value("stream_status").toString()=="LIVE"&&r.value("semantic_stream").toString()=="LIVE"&&r.value("lane_stream").toString()=="LIVE"&&
                 r.value("yolop_active").toBool(false)&&r.value("orientation_left_ok").toBool(false)&&r.value("orientation_right_ok").toBool(false);
        }
        if(sub=="4.2"){
          const QJsonObject c42=subs.value("4.2").toObject().value("conditions").toObject();
          for(const QString &light:{QStringLiteral("night"),QStringLiteral("day")}){
            for(int d=1;d<=5;++d){
              const QString key=QString("human_%1_%2m").arg(light).arg(d);
              int n=count("4.2",key);
              // Backward compatibility: historical human_1m..5m captures were
              // recorded at night before lighting became an explicit factor.
              if(light=="night"&&!c42.contains(key))n=count("4.2",QString("human_%1m").arg(d));
              if(n!=10)return false;
            }
          }
          return true;
        }
        if(sub=="4.3"){for(int i=1;i<=6;++i)if(count("4.3",QString("condition_%1").arg(i,2,10,QChar('0')))!=10)return false;return true;}
        if(sub=="4.4"){for(const QString &c:{QString("condition_A"),QString("condition_B"),QString("condition_C")})if(status("4.4",c)!="SELESAI")return false;return true;}
        if(sub=="4.5"){
          auto complete45=[&](const QString &c)->bool{
            const QJsonObject co=subs.value("4.5").toObject().value("conditions").toObject().value(c).toObject();
            const QJsonArray rows=co.value("rows").toArray();
            bool dryPass=co.value("dry_run_pass").toBool(false),armedComplete=co.value("armed_complete").toBool(false);
            bool dryVisual=false,armedVisual=false;const bool needVisual=c.startsWith(QStringLiteral("obstacle_"));
            for(const QJsonValue &v:rows){const QJsonObject r=v.toObject();const QString mode=r.value("mode").toString();
              const bool visual=!r.value("map_filename").toString().isEmpty()&&!r.value("combined_filename").toString().isEmpty();
              if(mode=="DRY_RUN"&&(r.value("integration_pass").toBool(false)||r.value("result").toString()=="DRY_RUN_PASS"))dryPass=true;
              if(mode=="DRY_RUN"&&visual)dryVisual=true;
              if(mode=="ARMED_VEHICLE_TEST"&&r.value("result").toString().toLower()=="complete"&&r.value("final_vehicle_state").toString()=="STOPPED"&&r.value("integration_pass").toBool(false))armedComplete=true;
              if(mode=="ARMED_VEHICLE_TEST"&&visual)armedVisual=true;
            }
            return dryPass&&armedComplete&&(!needVisual||(dryVisual&&armedVisual));
          };
          for(int i=1;i<=3;++i)if(!complete45(QString("obstacle_trial_%1").arg(i)))return false;
          for(int i=1;i<=3;++i)if(!complete45(QString("lane_trial_%1").arg(i)))return false;
          return true;
        }
        return false;
      };
      QString requestedSub;
      if(target=="4.1.zip")requestedSub="4.1";else if(target=="4.2.zip")requestedSub="4.2";else if(target=="4.3.zip")requestedSub="4.3";else if(target=="4.4.zip")requestedSub="4.4";else if(target=="4.5.zip")requestedSub="4.5";
      else if(target!="full.zip")return sendText(socket,400,"text/plain; charset=utf-8","Target export tidak valid");
      if(!requestedSub.isEmpty()&&!ready(requestedSub))
        return sendText(socket,409,"text/plain; charset=utf-8",QString("EXPORT DITOLAK: %1 belum DATA_VALID/lengkap").arg(requestedSub).toUtf8());
      if(target=="full.zip"&&!(ready("4.1")&&ready("4.2")&&ready("4.3")&&ready("4.4")&&ready("4.5")))
        return sendText(socket,409,"text/plain; charset=utf-8","EXPORT DITOLAK: Full Bab 4 belum lengkap");
      QProcess prep; prep.setProgram(agvPythonPath());
      prep.setArguments(QStringList{agvPath(QStringLiteral("src/navigation/tools/export_perception_bab4.py")),
                                    QStringLiteral("--session"), folder});
      prep.start();
      const bool prepDone=prep.waitForStarted(3000)&&prep.waitForFinished(120000)&&
        prep.exitStatus()==QProcess::NormalExit&&prep.exitCode()==0;
      if(!prepDone) return sendText(socket,500,"text/plain; charset=utf-8",
        QByteArray("Gagal menyiapkan summary/grafik/manifest: ")+prep.readAllStandardError().left(500));
      QString exportFolder = folder;
      if (target != QStringLiteral("full.zip")) {
        const QMap<QString,QString> map{{"4.1.zip","4.1_VALIDASI_SISTEM"},{"4.2.zip","4.2_HUMAN_DETECTION"},
          {"4.3.zip","4.3_LANE_SAFETY"},{"4.4.zip","4.4_REALTIME_PERFORMANCE"},{"4.5.zip","4.5_VEHICLE_RESPONSE"}};
        if (!map.contains(target)) return sendText(socket,400,"text/plain; charset=utf-8","Target export tidak valid");
        exportFolder = QDir(folder).filePath(map.value(target));
      }
      QByteArray bytes; QString zipMessage;
      const QString name = QFileInfo(folder).fileName()+QStringLiteral("_")+
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))+QStringLiteral("_")+target;
      if (!buildPerceptionZip(QStringList{exportFolder}, name, &bytes, &zipMessage))
        return sendText(socket,409,"text/plain; charset=utf-8",zipMessage.toUtf8());
      const QByteArray disposition=QByteArray("attachment; filename=\"")+name.toUtf8()+"\"";
      return sendBytes(socket,200,"application/zip",bytes,{{"Cache-Control","no-store"},{"Content-Disposition",disposition}});
    }
    if (request.method == "GET" && request.path == "/api/perception/f43/evidence-bank") {
      const QString root = agvPath(QStringLiteral("data/presepsi/F4.3_EVIDENCE_BANK"));
      const QStringList keys{QStringLiteral("center"), QStringLiteral("left_yellow"), QStringLiteral("left_red"),
                             QStringLiteral("right_yellow"), QStringLiteral("right_red"), QStringLiteral("gray")};
      QJsonArray images;
      for (const QString &key : keys) {
        QFile f(QDir(root).filePath(key + QStringLiteral(".jpg")));
        if (!f.open(QIODevice::ReadOnly)) continue;
        const QByteArray bytes = f.readAll();
        images.append(QJsonObject{{"key", key},
                                  {"data_url", QStringLiteral("data:image/jpeg;base64,") + QString::fromLatin1(bytes.toBase64())},
                                  {"bytes", bytes.size()}});
      }
      return sendJson(socket, 200, QJsonObject{{"ok", true}, {"images", images}, {"count", images.size()},
                                               {"source", QStringLiteral("F4.3_EVIDENCE_BANK")}, {"at_ms", nowMs()}});
    }
    if (request.method == "GET" && request.path == "/api/experiments") return sendJson(socket, 200, experimentCatalogJson());
    if (request.method == "GET" && request.path == "/api/navigation/data-packages") {
      const QJsonArray packages = navigationDataPackages();
      return sendJson(socket, 200, QJsonObject{{"ok", true}, {"packages", packages}, {"count", packages.size()}, {"at_ms", nowMs()}});
    }
    if (request.method == "GET" && request.path.startsWith("/api/navigation/data-package/") && request.path.endsWith(".zip")) {
      QString name = request.path.mid(QString("/api/navigation/data-package/").size());
      name.chop(4);
      if (!QRegularExpression(QStringLiteral("^BAB4_[A-Za-z0-9_.-]+$")).match(name).hasMatch())
        return sendText(socket, 400, "text/plain; charset=utf-8", "Package ID tidak valid");
      const QString root = QFileInfo(navigationDataPackageRoot()).absoluteFilePath();
      const QString dirPath = QDir(root).filePath(name);
      const QFileInfo di(dirPath);
      const QString canonicalDir = di.canonicalFilePath();
      if (canonicalDir.isEmpty() || !di.isDir() || !canonicalDir.startsWith(root + QDir::separator()))
        return sendText(socket, 404, "text/plain; charset=utf-8", "Package tidak ditemukan");
      const QJsonObject meta = readJsonObjectFile(QDir(canonicalDir).filePath(QStringLiteral("package.json")));
      const QString zipPath = meta.value(QStringLiteral("zip_path")).toString();
      QFile zf(zipPath);
      if (!zf.open(QIODevice::ReadOnly))
        return sendText(socket, 404, "text/plain; charset=utf-8", "ZIP package tidak ditemukan");
      const QByteArray disposition = QByteArray("attachment; filename=\"") + QFileInfo(zipPath).fileName().toUtf8() + "\"";
      return sendBytes(socket, 200, "application/zip", zf.readAll(), {{"Cache-Control","no-store"},{"Content-Disposition",disposition}});
    }
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
    if (request.method == "GET" && request.path == "/api/rm3100/calibration/status") return sendJson(socket, 200, rm3100CalibrationStatus());
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
    if (request.method == "GET" && request.path.startsWith("/api/map-editor/download/")) {
      const QString name=request.path.mid(QString("/api/map-editor/download/").size());
      if(name.isEmpty()||QFileInfo(name).fileName()!=name||QFileInfo(name).suffix().toLower()!=QStringLiteral("pgm")||
         !QRegularExpression(QStringLiteral("^undip_nav2_edit_[A-Za-z0-9_.-]+\\.pgm$")).match(name).hasMatch())
        return sendText(socket,400,"text/plain; charset=utf-8","Nama file PGM tidak valid");
      const QString navCfg=packageConfigDir("navigation","AGV_CONFIG_DIR");
      const QString editRoot=QDir::cleanPath(QDir(navCfg).absoluteFilePath(QStringLiteral("../maps/undip/edits")));
      const QString path=QDir(editRoot).absoluteFilePath(name);
      QFile f(path); if(!f.open(QIODevice::ReadOnly)) return sendText(socket,404,"text/plain; charset=utf-8","PGM hasil edit tidak ditemukan");
      const QByteArray disposition=QByteArray("attachment; filename=\"")+name.toUtf8()+"\"";
      return sendBytes(socket,200,"image/x-portable-graymap",f.readAll(),{{"Cache-Control","no-store"},{"Content-Disposition",disposition}});
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
    if (request.method == "GET" && request.path == "/api/experiment/record/last.zip") {
      QString folder;
      if (!lastXlsxPath_.isEmpty()) folder = QFileInfo(lastXlsxPath_).absolutePath();
      if (folder.isEmpty() || !QDir(folder).exists()) {
        const QStringList candidates = latestPerceptionFinalFolders();
        QDateTime newest;
        for (const QString &candidate : candidates) {
          const QFileInfo fi(candidate);
          if (folder.isEmpty() || fi.lastModified() > newest) { folder = candidate; newest = fi.lastModified(); }
        }
      }
      if (folder.isEmpty() || !QDir(folder).exists()) return sendText(socket,404,"text/plain; charset=utf-8","Belum ada folder hasil Persepsi yang dapat di-ZIP");
      QByteArray bytes; QString message;
      const QString name = QFileInfo(folder).fileName() + QStringLiteral(".zip");
      if (!buildPerceptionZip(QStringList{folder}, name, &bytes, &message)) return sendText(socket,404,"text/plain; charset=utf-8",message.toUtf8());
      const QByteArray disposition=QByteArray("attachment; filename=\"")+name.toUtf8()+"\"";
      return sendBytes(socket,200,"application/zip",bytes,{{"Cache-Control","no-store"},{"Content-Disposition",disposition}});
    }
    if (request.method == "GET" && request.path == "/api/perception/f43/latest.zip") {
      const QDir root(agvPath(QStringLiteral("data/presepsi")));
      const QFileInfoList entries = root.entryInfoList(QStringList{QStringLiteral("BAB4_F4.3_*")}, QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
      if (entries.isEmpty()) return sendText(socket,404,"text/plain; charset=utf-8","Belum ada folder hasil F4.3");
      const QString folder = entries.first().absoluteFilePath(); QByteArray bytes; QString message;
      const QString name = QFileInfo(folder).fileName() + QStringLiteral(".zip");
      if (!buildPerceptionZip(QStringList{folder}, name, &bytes, &message)) return sendText(socket,404,"text/plain; charset=utf-8",message.toUtf8());
      const QByteArray disposition=QByteArray("attachment; filename=\"")+name.toUtf8()+"\"";
      return sendBytes(socket,200,"application/zip",bytes,{{"Cache-Control","no-store"},{"Content-Disposition",disposition}});
    }
    if (request.method == "GET" && request.path == "/api/perception/final-dataset/latest.zip") {
      const QStringList folders = latestPerceptionFinalFolders(); QByteArray bytes; QString message;
      const QString name = QStringLiteral("FINAL_DATASET_PERSEPSI_") + QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")) + QStringLiteral(".zip");
      if (!buildPerceptionZip(folders, name, &bytes, &message)) return sendText(socket,404,"text/plain; charset=utf-8",message.toUtf8());
      const QByteArray disposition=QByteArray("attachment; filename=\"")+name.toUtf8()+"\"";
      return sendBytes(socket,200,"application/zip",bytes,{{"Cache-Control","no-store"},{"Content-Disposition",disposition}});
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
    auto bab4SessionGuard = [&](Bab4PerceptionSessionStore &store)->QJsonObject {
      const QJsonObject currentReply=store.state();
      if(!currentReply.value("ok").toBool(false)) return currentReply;
      const QString active=currentReply.value("state").toObject().value("session_id").toString();
      const QString client=json.value("session_id").toString();
      if(client.isEmpty()||active.isEmpty()||client!=active)
        return QJsonObject{{"ok",false},{"code","SESSION_CHANGED"},
          {"message","Session Bab 4 berubah — refresh state sebelum melanjutkan"},
          {"active_session_id",active}};
      return QJsonObject();
    };
    if (request.path == "/api/perception/bab4/session") {
      Bab4PerceptionSessionStore store(agvRootPath(), loadConfigSnapshot());
      const QString action=json.value("action").toString(QStringLiteral("ensure"));
      QJsonObject result;
      if(action=="ensure") result=store.ensure();
      else if(action=="new"){
        if(finalActive())return sendJson(socket,409,QJsonObject{{"ok",false},{"message","Session baru ditolak: ARMED trial masih aktif"}});
        result=store.create();
      }
      else if(action=="reset"){
        if(finalActive())return sendJson(socket,409,QJsonObject{{"ok",false},{"message","Reset session ditolak: ARMED trial masih aktif"}});
        if(json.value("confirm").toString()!=QStringLiteral("RESET"))
          return sendJson(socket,400,QJsonObject{{"ok",false},{"message","Ketik RESET untuk menghapus seluruh session Bab 4"}});
        const QJsonObject guard=bab4SessionGuard(store);if(!guard.isEmpty())return sendJson(socket,409,guard);
        const QString folder=store.activeSessionPath();
        if(!folder.isEmpty()&&QDir(folder).exists()&&!QDir(folder).removeRecursively())
          return sendJson(socket,500,QJsonObject{{"ok",false},{"message","Gagal menghapus session aktif"}});
        result=store.create();
      } else return sendJson(socket,400,QJsonObject{{"ok",false},{"message","Action session tidak valid"}});
      return sendJson(socket,result.value("ok").toBool()?200:409,result);
    } else if (request.path == "/api/perception/bab4/start") {
      Bab4PerceptionSessionStore store(agvRootPath(), loadConfigSnapshot());
      const QJsonObject guard=bab4SessionGuard(store);if(!guard.isEmpty())return sendJson(socket,409,guard);
      const QJsonObject result=store.startCondition(json.value("subchapter").toString(),json.value("condition").toString());
      return sendJson(socket,result.value("ok").toBool()?200:409,result);
    } else if (request.path == "/api/perception/bab4/capture") {
      Bab4PerceptionSessionStore store(agvRootPath(), loadConfigSnapshot());
      const QJsonObject guard=bab4SessionGuard(store);if(!guard.isEmpty())return sendJson(socket,409,guard);
      const QString sub=json.value("subchapter").toString(),condition=json.value("condition").toString();
      const QJsonObject result=store.capture(sub,condition,json,bridge_->snapshot(),bridge_->cameraJpeg());
      return sendJson(socket,result.value("ok").toBool()?200:409,result);
    } else if (request.path == "/api/perception/bab4/visual-evidence") {
      Bab4PerceptionSessionStore store(agvRootPath(), loadConfigSnapshot());
      const QJsonObject guard=bab4SessionGuard(store);if(!guard.isEmpty())return sendJson(socket,409,guard);
      const QJsonObject stateReply=store.state();const QString activeSession=stateReply.value("state").toObject().value("session_id").toString();
      if(json.value("session_id").toString()!=activeSession)
        return sendJson(socket,409,QJsonObject{{"ok",false},{"code","SESSION_CHANGED"},{"message","Session visual evidence berubah"}});
      auto pngData=[&](const QString &key)->QByteArray{
        const QString u=json.value(key).toString();const QString prefix=QStringLiteral("data:image/png;base64,");
        return u.startsWith(prefix)?QByteArray::fromBase64(u.mid(prefix.size()).toLatin1()):QByteArray();
      };
      const QByteArray mapPng=pngData(QStringLiteral("map_data_url")),combinedPng=pngData(QStringLiteral("combined_data_url"));
      const QJsonObject result=store.save45VisualEvidence(json.value("condition").toString(),json.value("mode").toString(),
        json.value("capture_id").toInt(0),mapPng,combinedPng);
      return sendJson(socket,result.value("ok").toBool()?200:409,result);
    } else if (request.path == "/api/perception/bab4/reset") {
      if(finalActive())return sendJson(socket,409,QJsonObject{{"ok",false},{"message","Reset ditolak: ARMED trial masih aktif"}});
      Bab4PerceptionSessionStore store(agvRootPath(), loadConfigSnapshot());
      const QJsonObject guard=bab4SessionGuard(store);if(!guard.isEmpty())return sendJson(socket,409,guard);
      const QString scope=json.value("scope").toString(),sub=json.value("subchapter").toString(),condition=json.value("condition").toString();
      if(scope=="group"){
        if(sub!="4.5")return sendJson(socket,400,QJsonObject{{"ok",false},{"message","Group reset hanya tersedia untuk 4.5"}});
        const QJsonObject result=store.reset45Group(json.value("group").toString());
        return sendJson(socket,result.value("ok").toBool()?200:409,result);
      }
      if(scope!="condition"&&scope!="subchapter") return sendJson(socket,400,QJsonObject{{"ok",false},{"message","Scope reset tidak valid"}});
      const QJsonObject result=store.reset(sub,condition,scope=="subchapter");
      return sendJson(socket,result.value("ok").toBool()?200:409,result);
    } else if (request.path == "/api/perception/f43/evidence-bank") {
      const QString root = agvPath(QStringLiteral("data/presepsi/F4.3_EVIDENCE_BANK"));
      const QSet<QString> allowed{QStringLiteral("center"),QStringLiteral("left_yellow"),QStringLiteral("left_red"),
                                  QStringLiteral("right_yellow"),QStringLiteral("right_red"),QStringLiteral("gray")};
      if (json.value("action").toString().trimmed().toLower() == QStringLiteral("clear")) {
        int removed = 0; for (const QString &k : allowed) if (QFile::remove(QDir(root).filePath(k + QStringLiteral(".jpg")))) ++removed;
        return sendJson(socket,200,QJsonObject{{"ok",true},{"removed",removed},{"at_ms",nowMs()}});
      }
      const QString key = json.value("key").toString().trimmed();
      if (!allowed.contains(key)) return sendJson(socket,400,QJsonObject{{"ok",false},{"message","Kondisi F4.3 tidak valid"}});
      const QString dataUrl = json.value("data_url").toString();
      const int comma = dataUrl.indexOf(',');
      if (!dataUrl.startsWith(QStringLiteral("data:image/jpeg;base64,")) || comma < 0)
        return sendJson(socket,400,QJsonObject{{"ok",false},{"message","Format citra F4.3 harus JPEG data URL"}});
      const QByteArray bytes = QByteArray::fromBase64(dataUrl.mid(comma + 1).toLatin1());
      if (bytes.isEmpty() || bytes.size() > 2 * 1024 * 1024)
        return sendJson(socket,400,QJsonObject{{"ok",false},{"message","Ukuran citra F4.3 tidak valid"}});
      if (!QDir().mkpath(root)) return sendJson(socket,500,QJsonObject{{"ok",false},{"message","Gagal membuat folder bank citra F4.3"}});
      const QString path = QDir(root).filePath(key + QStringLiteral(".jpg"));
      QFile f(path);
      if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate) || f.write(bytes) != bytes.size())
        return sendJson(socket,500,QJsonObject{{"ok",false},{"message","Gagal menyimpan citra representatif F4.3"}});
      f.close();
      return sendJson(socket,200,QJsonObject{{"ok",true},{"key",key},{"bytes",bytes.size()},{"path",path},{"at_ms",nowMs()}});
    } else if (request.path == "/api/navigation/tuning/entry") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"code","READ_ONLY"},{"message","Read-only; tuning state persistence ditolak"}});
      const QString id=json.value("id").toString().trimmed(),action=json.value("action").toString(QStringLiteral("ensure")).trimmed().toLower();
      QJsonObject result;
      if(action==QStringLiteral("ensure")) {
        const bool reportId=QRegularExpression(QStringLiteral(R"(^R4\.[234]\.[1-5]$)")).match(id).hasMatch();
        ok=reportId?ensureNavigationReportBaseline(id,json.value("items").toArray(),&message,&result):ensureNavigationTuningEntry(id,&message,&result);
      }
      else if(action==QStringLiteral("restore")) ok=buildNavigationStepEntryProposal(id,&message,&result);
      else return sendJson(socket,400,QJsonObject{{"ok",false},{"message","action harus ensure atau restore"},{"at_ms",nowMs()}});
      result["ok"]=ok;result["message"]=message;result["commissioning"]=commissioningState();result["at_ms"]=nowMs();
      return sendJson(socket,ok?200:409,result);
    } else if (request.path == "/api/map-editor/save") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"code","READ_ONLY"},{"message","Read-only; save map copy ditolak"}});
      QJsonObject result; ok=saveMapEditorCopy(json,&result,&message); result["ok"]=ok; result["message"]=message; result["at_ms"]=nowMs();
      return sendJson(socket,ok?200:409,result);
    } else if (request.path == "/api/map-editor/open-gimp") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"code","READ_ONLY"},{"message","Read-only; open GIMP ditolak"}});
      const QString navCfg=packageConfigDir("navigation","AGV_CONFIG_DIR"),editRoot=QDir::cleanPath(QDir(navCfg).absoluteFilePath(QStringLiteral("../maps/undip/edits")));
      const QString requested=QDir::cleanPath(json.value("path").toString());
      if(requested.isEmpty()||!requested.startsWith(editRoot+QDir::separator())||QFileInfo(requested).suffix().toLower()!=QStringLiteral("pgm")||!QFileInfo(requested).isFile())
        return sendJson(socket,400,QJsonObject{{"ok",false},{"message","Path PGM edit tidak valid"}});
      const bool started=QProcess::startDetached(QStringLiteral("gimp"),QStringList{requested});
      return sendJson(socket,started?200:409,QJsonObject{{"ok",started},{"message",started?QStringLiteral("GIMP dibuka dengan map hasil edit"):QStringLiteral("Gagal membuka GIMP")},{"path",requested},{"at_ms",nowMs()}});
    } else if (request.path == "/api/config/proposal") {
      const QString sourceTask=json.value("source_task").toString();const QJsonArray proposalItems=json.value("items").toArray();
      const QMap<QString,QSet<QString>> allowedPaths{
        {QStringLiteral("perception:homography"),QSet<QString>{QStringLiteral("perception.ros__parameters.ground_src_points"),QStringLiteral("perception.ros__parameters.ground_dst_points")}},
        {QStringLiteral("perception:obstacle-distance"),QSet<QString>{QStringLiteral("perception.ros__parameters.obstacle_distance_calibration_coefficients")}},
        {QStringLiteral("perception:lane-roi"),QSet<QString>{QStringLiteral("perception.ros__parameters.lane_safety_enabled"),QStringLiteral("perception.ros__parameters.lane_corridor_correction_gain_m_per_px"),QStringLiteral("perception.ros__parameters.lane_corridor_top_y_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_bottom_y_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_left_top_x_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_left_top_y_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_left_bottom_x_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_left_bottom_y_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_right_top_x_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_right_top_y_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_right_bottom_x_ratio"),QStringLiteral("perception.ros__parameters.lane_corridor_right_bottom_y_ratio"),QStringLiteral("perception.ros__parameters.nav2_obstacle_roi_enabled"),QStringLiteral("perception.ros__parameters.nav2_obstacle_roi_points")}}
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
    } else if (request.path == "/api/config/runtime-check") {
      const QJsonArray items = json.value("items").toArray();
      if (items.isEmpty() || items.size() > 64)
        return sendJson(socket,400,QJsonObject{{"ok",false},{"message","items harus berisi 1..64 parameter"}});
      QSet<QString> seen; QJsonArray rows; bool checkedAny=false, allMatch=true; int checkedCount=0;
      for (const QJsonValue &v : items) {
        if (!v.isObject()) continue;
        const QJsonObject item=v.toObject();
        const QString fk=item.value("file_key").toString().trimmed(), path=item.value("path").toString().trimmed(), identity=fk+":"+path;
        if (fk.isEmpty() || path.isEmpty() || seen.contains(identity)) continue;
        seen.insert(identity);
        const QJsonObject runtime=bridge_->configRuntimeState(fk,path);
        const bool hasTarget=runtime.value("has_runtime_target").toBool(false);
        const bool readbackCapable=runtime.value("readback_capable").toBool(false);
        bool itemMatch=hasTarget && readbackCapable;
        if (hasTarget) {
          checkedAny=true; ++checkedCount;
          const QJsonArray readback=runtime.value("readback").toArray();
          if (readback.isEmpty()) itemMatch=false;
          for (const QJsonValue &rv : readback) itemMatch=itemMatch && rv.toObject().value("match").toBool(false);
          allMatch=allMatch && itemMatch;
        }
        rows.append(QJsonObject{{"identity",identity},{"file_key",fk},{"path",path},
          {"status",hasTarget?(itemMatch?QStringLiteral("ACTIVE_MATCH"):QStringLiteral("RUNTIME_MISMATCH")):QStringLiteral("NO_RUNTIME_TARGET")},
          {"match",itemMatch},{"runtime",runtime}});
      }
      return sendJson(socket,200,QJsonObject{{"ok",true},{"runtime_match",checkedAny&&allMatch},{"checked_count",checkedCount},
        {"items",rows},{"at_ms",nowMs()}});
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
    } else if (request.path == "/api/navigation/data-package/delete") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"message","Web GUI read-only; hapus package ditolak"}});
      QJsonObject result; ok=deleteNavigationDataPackage(json,&message,&result);result["ok"]=ok;result["message"]=message;result["at_ms"]=nowMs();
      return sendJson(socket,ok?200:409,result);
    } else if (request.path == "/api/experiment/trial/optimal-scale") {
      if (json.value("apply").toBool(false)) return sendJson(socket,409,QJsonObject{{"ok",false},{"code","USE_CONFIG_TRANSACTION"},{"message","Direct optimal-scale apply dinonaktifkan; gunakan proposal_items → Validate → Diff Review → Apply"}});
      QJsonObject result;ok=calculateOptimalErpmPerMpsProposal(&message,&result);
      if(ok){const double erpmPerMps=result.value("erpm_per_mps").toDouble();const QString iso=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);QJsonArray proposalItems;
        proposalItems.append(QJsonObject{{"file_key","esc"},{"path","esc_ackermann.ros__parameters.drive_erpm_per_mps"},{"value",erpmPerMps}});
        proposalItems.append(QJsonObject{{"file_key","vehicle"},{"path","vehicle.ros__parameters.drive_erpm_per_mps"},{"value",erpmPerMps},{"generated",true}});
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
    } else if (request.path == "/api/rm3100/calibration/start") {
      ok=startRm3100Calibration(&message);
    } else if (request.path == "/api/rm3100/calibration/stop") {
      ok=stopRm3100Calibration(&message);
    } else if (request.path == "/api/rm3100/calibration/step") {
      ok=setRm3100CalibrationFace(json.value("face").toString(),&message);
    } else if (request.path == "/api/imu/profile/optimal") {
      if (bridge_->readOnly()) return sendJson(socket,403,QJsonObject{{"ok",false},{"message","Web GUI read-only"}});
      QString stationary; if(!bridge_->vehicleStationary(&stationary)) return sendJson(socket,409,QJsonObject{{"ok",false},{"message",stationary}});
      ok=bridge_->triggerService("/imu/configure_optimal_profile","imu_optimal_profile",&message);
        } else if (request.path == "/api/perception/inference") {
      ok = bridge_->setPerceptionInference(json.value("enabled").toBool(false), &message);
    } else if (request.path == "/api/perception/lane-preview") {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      ok = bridge_->setPerceptionLanePreview(
        json.value("top_y").toDouble(nan), json.value("bottom_y").toDouble(nan),
        json.value("left_top_x").toDouble(nan), json.value("left_top_y").toDouble(nan),
        json.value("left_bottom_x").toDouble(nan), json.value("left_bottom_y").toDouble(nan),
        json.value("right_top_x").toDouble(nan), json.value("right_top_y").toDouble(nan),
        json.value("right_bottom_x").toDouble(nan), json.value("right_bottom_y").toDouble(nan),
        &message);
    } else if (request.path.startsWith("/api/perception/final-trial/")) {
      const auto result=finalRequest(request.path.mid(QString("/api/perception/final-trial/").size()),json);
      return sendJson(socket,result.value("ok").toBool()?200:409,result);
    } else if (request.path == "/api/navigation/goal") {
      if(finalActive())return sendJson(socket,409,QJsonObject{{"ok",false},{"message","Final Dataset trial masih aktif"}});
      const bool missionRecorderReady = recording_ &&
        recordingSubsystem_ == QStringLiteral("navigation") &&
        recordingId_ == QStringLiteral("BAB4_MISSION_4.2_4.4") &&
        recordingSourceExperimentId_ == QStringLiteral("R4.4.5");
      if (!missionRecorderReady) {
        return sendJson(socket, 409, QJsonObject{{"ok", false},
          {"message", "SEND GOAL diblok: master recorder BAB4 4.2-4.4 belum aktif. Start recorder harus sukses sebelum goal."},
          {"recorder_active", recording_},
          {"recorder_id", recordingId_},
          {"source_experiment_id", recordingSourceExperimentId_},
          {"at_ms", nowMs()}});
      }
      bridge_->publishFinalGuard("IDLE", "");
      ok = bridge_->publishGoal(json.value("x").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                json.value("y").toDouble(std::numeric_limits<double>::quiet_NaN()),
                                json.contains("yaw_rad") ? json.value("yaw_rad").toDouble() : json.value("yaw_deg").toDouble() * kPi / 180.0,
                                &message);
    } else if (request.path == "/api/navigation/cancel") {
      ok = bridge_->cancelNavigation(&message);
    } else if (request.path == "/api/navigation/clear-goal-visual") {
      ok = bridge_->clearGoalVisualization(&message);
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
      else if (action == "STEER_STOP") ok = bridge_->publishHmiRequest("STEER:STOP", &message);
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
    QString root = agvPath(QStringLiteral("data/presepsi/evidence"));
    if (recording_ && recordingSubsystem_ == QStringLiteral("perception") && recordingSourceExperimentId_.startsWith(QStringLiteral("F4.")) && recordingStartedMs_ > 0) {
      const QString dataRoot = agvPath(QStringLiteral("data/presepsi"));
      const QString runName = QStringLiteral("BAB4_") + recordingCsvStem(recordingSourceExperimentId_) + QStringLiteral("_") +
        QDateTime::fromMSecsSinceEpoch(recordingStartedMs_).toString(QStringLiteral("yyyyMMdd_HHmmss"));
      root = QDir(QDir(dataRoot).filePath(runName)).filePath(QStringLiteral("evidence_frames"));
    }
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
    const QString sourceExperimentId = json.value("source_experiment_id").toString().trimmed().isEmpty() ? id : json.value("source_experiment_id").toString().trimmed();
    const auto navigationRequiresMotionQualification = [](const QString &taskId) {
      return taskId == QStringLiteral("N2.1") || taskId == QStringLiteral("N3.1") || taskId == QStringLiteral("N3.2") ||
             taskId.startsWith(QStringLiteral("N12.")) || taskId.startsWith(QStringLiteral("N13.")) ||
             taskId.startsWith(QStringLiteral("N14.")) || taskId.startsWith(QStringLiteral("N15.")) ||
             taskId.startsWith(QStringLiteral("N16.")) || taskId.startsWith(QStringLiteral("N17."));
    };
    if (subsystem == QStringLiteral("navigation") && navigationRequiresMotionQualification(sourceExperimentId) &&
        !taskQualified(QStringLiteral("steering"), QStringLiteral("4.9"))) {
      if (message) *message = QStringLiteral("Commissioning gate: motion task memerlukan ESC 4.9 Final Gate sebelum recording Navigasi");
      return false;
    }
    if (subsystem == QStringLiteral("perception") && !sourceExperimentId.startsWith(QStringLiteral("F4."))) {
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
    recordingRateHz_ = std::clamp(json.value("sample_rate_hz").toDouble(5.0), 1.0, 100.0);
    recordingStartedIso_ = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    recordingStartedMs_ = nowMs();
    recordingRows_.clear();
    recording_ = true;
    recordingTimer_.start(std::max(10, static_cast<int>(std::lround(1000.0 / recordingRateHz_))));
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
    QMap<QString, QString> flatTrialInputs;
    for (auto it = recordingTrialInputs_.constBegin(); it != recordingTrialInputs_.constEnd(); ++it)
      flattenJson(QStringLiteral("trial_input.") + it.key(), it.value(), flatTrialInputs);
    for (auto it = flatTrialInputs.cbegin(); it != flatTrialInputs.cend(); ++it) row[it.key()] = it.value();
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
    QString outputRoot = dataRoot;
    if (subsystem == QStringLiteral("perception") && recording_ && recordingSubsystem_ == QStringLiteral("perception") &&
        recordingSourceExperimentId_.startsWith(QStringLiteral("F4.")) && recordingStartedMs_ > 0) {
      const QString runName = QStringLiteral("BAB4_") + recordingCsvStem(recordingSourceExperimentId_) + QStringLiteral("_") +
        QDateTime::fromMSecsSinceEpoch(recordingStartedMs_).toString(QStringLiteral("yyyyMMdd_HHmmss"));
      outputRoot = QDir(dataRoot).filePath(runName);
    } else if (subsystem == QStringLiteral("navigation") && recordingSubsystem_ == QStringLiteral("navigation") &&
               recordingSourceExperimentId_ == QStringLiteral("R4.4.5") && recordingStartedMs_ > 0) {
      const QString runName = QStringLiteral("BAB4_MISSION_4_2_4_4_") +
        QDateTime::fromMSecsSinceEpoch(recordingStartedMs_).toString(QStringLiteral("yyyyMMdd_HHmmss"));
      outputRoot = QDir(dataRoot).filePath(runName);
    }
    if (!QDir().mkpath(outputRoot)) {
      if (message) *message = QStringLiteral("Gagal membuat folder data: ") + outputRoot;
      return false;
    }
    const QString stemLabel = id.isEmpty() ? label : id;
    const QString minuteStamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString baseStem = recordingCsvStem(stemLabel) + QStringLiteral("_T%1_").arg(tableIndex + 1) + minuteStamp;
    QString fileName = baseStem + QStringLiteral(".csv");
    int suffix = 2;
    while (QFileInfo::exists(QDir(outputRoot).filePath(fileName)))
      fileName = baseStem + QStringLiteral("_%1.csv").arg(suffix++, 2, 10, QChar('0'));
    const QString path = QDir(outputRoot).filePath(fileName);
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
    if (result) *result = QJsonObject{{"path", path}, {"table_index", tableIndex}, {"section_id", id}, {"subsystem", subsystem}, {"report_folder", outputRoot}};
    if (message) *message = QStringLiteral("Template table CSV tersimpan: ") + path;
    return true;
  }

  QString trialStorePath() const {
    return agvPath(QStringLiteral("data/experiment_trials.yaml"));
  }

  bool catalogTaskExists(const QString &subsystem, const QString &id) const {
    for (const ExperimentSpec &spec : buildExperimentCatalog(subsystem)) if (spec.id == id) return true;
    return false;
  }

  QJsonObject taskConfigSnapshot(const QString &subsystem, const QString &id) const {
    QJsonObject values;
    QJsonArray items;
    for (const ExperimentSpec &spec : buildExperimentCatalog(subsystem)) {
      if (spec.id != id) continue;
      QSet<QString> seen;
      for (const ExperimentParameterField &field : spec.parameterFields) {
        if (field.yamlFileKey.isEmpty() || field.yamlPath.isEmpty()) continue;
        const QString identity = field.yamlFileKey + QStringLiteral(":") + field.yamlPath;
        if (seen.contains(identity)) continue;
        seen.insert(identity);
        const QJsonValue value = configStoredValue(field.yamlFileKey, field.yamlPath);
        if (value.isUndefined() || value.isNull()) continue;
        values[identity] = value;
        items.append(QJsonObject{{"identity",identity},{"file_key",field.yamlFileKey},{"path",field.yamlPath},
                                 {"label",field.label},{"kind",field.kind},{"locked",field.locked},{"value",value}});
      }
      break;
    }
    return QJsonObject{{"subsystem",subsystem},{"task_id",id},{"values",values},{"items",items},
                       {"captured_at",QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},
                       {"config_revision",configRevisionState().value("config_revision")}};
  }

  QString taskConfigFingerprint(const QString &subsystem, const QString &id) const {
    const bool finalGate = (subsystem == QStringLiteral("steering") && id == QStringLiteral("4.9")) ||
      (subsystem == QStringLiteral("navigation") && (id == QStringLiteral("N16.1") || id == QStringLiteral("N17.1"))) ||
      (subsystem == QStringLiteral("perception") && id == QStringLiteral("4.9"));
    const QJsonObject snapshot = taskConfigSnapshot(subsystem,id);
    const QJsonObject values = snapshot.value("values").toObject();
    if (!finalGate && !values.isEmpty()) {
      QStringList rows;
      for (const QString &identity : values.keys()) {
        QJsonArray wrapper; wrapper.append(values.value(identity));
        rows << identity + QStringLiteral("=") + QString::fromUtf8(QJsonDocument(wrapper).toJson(QJsonDocument::Compact));
      }
      std::sort(rows.begin(),rows.end());
      return QString::fromLatin1(QCryptographicHash::hash(rows.join(QStringLiteral("\n")).toUtf8(),QCryptographicHash::Sha256).toHex());
    }
    QSet<QString> keys;
    for (const ExperimentSpec &spec : buildExperimentCatalog(subsystem)) {
      if (spec.id != id) continue;
      for (const ExperimentParameterField &field : spec.parameterFields)
        if (!field.yamlFileKey.isEmpty()) keys.insert(field.yamlFileKey);
      break;
    }
    if (finalGate || keys.isEmpty()) {
      if (subsystem == QStringLiteral("steering")) keys.unite(QSet<QString>{"esc","vehicle","foc_thesis","vesc_tool"});
      else if (subsystem == QStringLiteral("navigation")) keys.unite(QSet<QString>{"vehicle","navigation_core","nav2","ekf","localization","gnss","imu","mag_heading","imu_speed","trajectory_safety","collision","mppi_closed_loop","esc","hmi"});
      else if (subsystem == QStringLiteral("perception")) keys.unite(QSet<QString>{"perception","bbox_calibration","trajectory_safety","navigation_core"});
    }
    const QJsonObject revs = configRevisionState().value("files").toObject();
    QStringList rows;
    for (const QString &key : keys) if (revs.contains(key)) rows << key + QStringLiteral("=") + revs.value(key).toString();
    std::sort(rows.begin(), rows.end());
    return QString::fromLatin1(QCryptographicHash::hash(rows.join(QStringLiteral("\n")).toUtf8(), QCryptographicHash::Sha256).toHex());
  }

  QJsonObject navigationTuningState() const {
    const QJsonObject store = loadTrialStore();
    const QJsonObject persisted = store.value("navigation_tuning").toObject();
    QJsonObject out;
    for (const ExperimentSpec &spec : buildExperimentCatalog(QStringLiteral("navigation"))) {
      if (!QRegularExpression(QStringLiteral(R"(^N\d+\.\d+$)")).match(spec.id).hasMatch()) continue;
      QJsonObject row = persisted.value(spec.id).toObject();
      if (row.isEmpty()) continue;
      const QJsonObject current = taskConfigSnapshot(QStringLiteral("navigation"),spec.id);
      const QJsonObject entryValues = row.value("entry_snapshot").toObject().value("values").toObject();
      const QJsonObject currentValues = current.value("values").toObject();
      QJsonArray changed;
      for (const QString &identity : entryValues.keys())
        if (currentValues.contains(identity) && !jsonRuntimeEquivalent(entryValues.value(identity),currentValues.value(identity))) changed.append(identity);
      row["current_snapshot"] = current;
      row["changed_identities"] = changed;
      row["candidate_changed"] = !changed.isEmpty();
      row["qualification"] = effectiveQualification(QStringLiteral("navigation"),spec.id);
      out[spec.id] = row;
    }
    for (const QString &id : persisted.keys()) {
      if (!QRegularExpression(QStringLiteral(R"(^R4\.[234]\.[1-5]$)")).match(id).hasMatch()) continue;
      QJsonObject row=persisted.value(id).toObject();
      if(row.isEmpty())continue;
      const QJsonObject entry=row.value("entry_snapshot").toObject();
      const QJsonObject entryValues=entry.value("values").toObject();
      QJsonObject currentValues;QJsonArray currentItems;
      for(const QJsonValue &v:entry.value("items").toArray()){
        QJsonObject item=v.toObject();const QString fk=item.value("file_key").toString(),path=item.value("path").toString(),identity=fk+QStringLiteral(":")+path;
        QJsonValue current;QString why;
        if(!currentYamlValue(fk,path,&current,&why))continue;
        currentValues[identity]=current;item["value"]=current;currentItems.append(item);
      }
      const QJsonObject current{{"subsystem","navigation"},{"task_id",id},{"values",currentValues},{"items",currentItems},
        {"captured_at",QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},{"config_revision",configRevisionState().value("config_revision")}};
      QJsonArray changed;
      for(const QString &identity:entryValues.keys())
        if(currentValues.contains(identity)&&!jsonRuntimeEquivalent(entryValues.value(identity),currentValues.value(identity)))changed.append(identity);
      row["current_snapshot"]=current;row["changed_identities"]=changed;row["candidate_changed"]=!changed.isEmpty();
      row["qualification"]=effectiveQualification(QStringLiteral("navigation"),id);out[id]=row;
    }
    return out;
  }

  bool ensureNavigationReportBaseline(const QString &id, const QJsonArray &requestedItems, QString *message, QJsonObject *result) {
    if (!QRegularExpression(QStringLiteral(R"(^R4\.[234]\.[1-5]$)")).match(id).hasMatch()) {
      if (message) *message = QStringLiteral("BAB4 navigation report id tidak valid");
      return false;
    }
    QJsonObject store=loadTrialStore(),tuning=store.value("navigation_tuning").toObject(),row=tuning.value(id).toObject();
    if (requestedItems.isEmpty() || requestedItems.size()>64) {
      if (!row.contains("entry_snapshot")) {
        if (message) *message = QStringLiteral("Baseline sesi BAB4 memerlukan daftar parameter 1..64");
        return false;
      }
    } else {
      const QJsonObject metadata=loadUiParameterMetadata().value("parameters").toObject();
      QJsonObject entry=row.value("entry_snapshot").toObject();
      QJsonObject values=entry.value("values").toObject();
      QJsonArray items=entry.value("items").toArray();
      QSet<QString> seen;
      for (const QJsonValue &v : items) {
        const QString identity=v.toObject().value("identity").toString();
        if(!identity.isEmpty())seen.insert(identity);
      }
      int added=0;
      for (const QJsonValue &v : requestedItems) {
        const QJsonObject req=v.toObject();
        const QString fk=req.value("file_key").toString().trimmed(),path=req.value("path").toString().trimmed(),identity=fk+QStringLiteral(":")+path;
        if (fk.isEmpty()||path.isEmpty()||seen.contains(identity)) continue;
        const QJsonObject meta=metadata.value(identity).toObject();
        if (!meta.value("metadata_complete").toBool(false) || meta.value("write_authority").toString()!=QStringLiteral("ros_yaml")) continue;
        QJsonValue current; QString why;
        if (!currentYamlValue(fk,path,&current,&why) || current.isUndefined() || current.isNull()) continue;
        seen.insert(identity); values[identity]=current;
        QString kind=QStringLiteral("string");
        if (current.isBool()) kind=QStringLiteral("bool");
        else if (current.isDouble()) kind=QStringLiteral("float");
        else if (current.isArray()) kind=QStringLiteral("list");
        items.append(QJsonObject{{"identity",identity},{"file_key",fk},{"path",path},{"label",meta.value("label").toString(path)},
                                 {"kind",kind},{"locked",false},{"value",current}});
        ++added;
      }
      if (items.isEmpty()) {
        if (message) *message = QStringLiteral("Tidak ada parameter ros_yaml valid untuk baseline sesi BAB4");
        return false;
      }
      entry["subsystem"]=QStringLiteral("navigation");
      entry["task_id"]=id;
      entry["values"]=values;
      entry["items"]=items;
      if(!entry.contains("captured_at"))entry["captured_at"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      if(!entry.contains("config_revision"))entry["config_revision"]=configRevisionState().value("config_revision");
      if(added>0)entry["augmented_at"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      row["entry_snapshot"]=entry;
      if(!row.contains("entry_created_at"))row["entry_created_at"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      row["source"]=QStringLiteral("server_persistent_session_baseline");
      tuning[id]=row;store["navigation_tuning"]=tuning;store["version"]=3;
      store["updated_at"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      if(added>0 && !saveTrialStore(store,message))return false;
    }
    const QJsonObject state=navigationTuningState().value(id).toObject();
    if(result)*result=state;
    if(message)*message=QStringLiteral("Baseline sesi persisten tersedia untuk ")+id;
    return true;
  }

  bool ensureNavigationTuningEntry(const QString &id, QString *message, QJsonObject *result) {
    const bool normalId=QRegularExpression(QStringLiteral(R"(^N\d+\.\d+$)")).match(id).hasMatch();
    const bool reportId=QRegularExpression(QStringLiteral(R"(^R4\.[234]\.[1-5]$)")).match(id).hasMatch();
    if ((!normalId&&!reportId) || (normalId&&!catalogTaskExists(QStringLiteral("navigation"),id))) {
      if (message) *message = QStringLiteral("Navigation tuning id tidak valid");
      return false;
    }
    QJsonObject store = loadTrialStore();
    QJsonObject tuning = store.value("navigation_tuning").toObject();
    QJsonObject row = tuning.value(id).toObject();
    if (!row.contains("entry_snapshot")) {
      if (reportId) {
        if (message) *message=QStringLiteral("Baseline sesi BAB4 belum dibuat; buka subbab terlebih dahulu sebelum Restore");
        return false;
      }
      row["entry_snapshot"] = taskConfigSnapshot(QStringLiteral("navigation"),id);
      row["entry_created_at"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      row["source"] = QStringLiteral("server_persistent_step_entry");
      tuning[id] = row; store["navigation_tuning"] = tuning; store["version"] = 3;
      store["updated_at"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      if (!saveTrialStore(store,message)) return false;
    }
    const QJsonObject state = navigationTuningState().value(id).toObject();
    if (result) *result = state;
    if (message) *message = (id.startsWith(QStringLiteral("R4.")) ? QStringLiteral("Baseline sesi persisten tersedia untuk ") : QStringLiteral("Step Entry persisten tersedia untuk ")) + id;
    return true;
  }

  bool buildNavigationStepEntryProposal(const QString &id, QString *message, QJsonObject *result) {
    QJsonObject state;
    if (!ensureNavigationTuningEntry(id,message,&state)) return false;
    const QJsonObject entry = state.value("entry_snapshot").toObject();
    const QJsonArray items = entry.value("items").toArray();
    const QJsonObject metadata = loadUiParameterMetadata().value("parameters").toObject();
    QJsonArray proposalItems;
    for (const QJsonValue &v : items) {
      const QJsonObject snap = v.toObject();
      const QString fk=snap.value("file_key").toString(), path=snap.value("path").toString(), identity=fk+QStringLiteral(":")+path;
      QJsonValue current; QString why;
      if (!currentYamlValue(fk,path,&current,&why) || jsonRuntimeEquivalent(current,snap.value("value"))) continue;
      const QJsonObject meta = metadata.value(identity).toObject();
      const QString authority = meta.value("write_authority").toString();
      const bool generatedAllowed = authority == QStringLiteral("calibration_generated");
      const bool manualAllowed = authority == QStringLiteral("ros_yaml") && snap.value("kind").toString() != QStringLiteral("yaml_readonly") && !snap.value("locked").toBool(false);
      const bool n2DrivePair = id == QStringLiteral("N2.1") && identity == QStringLiteral("esc:esc_ackermann.ros__parameters.drive_erpm_per_mps");
      if (!generatedAllowed && !manualAllowed && !n2DrivePair) continue;
      QJsonObject item{{"file_key",fk},{"path",path},{"value",snap.value("value")}};
      if (generatedAllowed) item["generated"] = true;
      proposalItems.append(item);
    }
    if (proposalItems.isEmpty()) {
      if (result) *result = QJsonObject{{"proposal_items",QJsonArray()},{"state",state}};
      if (message) *message = QStringLiteral("Current YAML sudah sama dengan Step Entry");
      return true;
    }
    const QJsonObject evidence{{"restore_step",id},{"entry_created_at",state.value("entry_created_at")}};
    const QJsonObject proposal = registerConfigProposal(QStringLiteral("navigation:step-entry:")+id,proposalItems,evidence);
    if (result) *result = QJsonObject{{"proposal",proposal},{"proposal_items",proposalItems},{"state",state}};
    if (message) *message = QStringLiteral("Proposal restore Step Entry dibuat; belum ada YAML/runtime write");
    return true;
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
    out["navigation_tuning"] = navigationTuningState();
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
    const QString qualifiedAt=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    const QJsonObject configSnapshot=taskConfigSnapshot(subsystem,id);
    QJsonObject q{{"subsystem",subsystem},{"task_id",id},{"status",status},{"effective_status",status},
      {"evidence_trial_ids",evidence},{"config_fingerprint",taskConfigFingerprint(subsystem,id)},
      {"config_snapshot",configSnapshot},{"config_revision",configRevisionState().value("config_revision")},{"reviewer_source","operator_review"},
      {"reason",json.value("reason").toString()},{"metric_summary",json.value("metric_summary")},
      {"qualified_at",qualifiedAt}};
    QJsonObject store=loadTrialStore(),qual=store.value("qualifications").toObject(),domain=qual.value(subsystem).toObject();
    domain[id]=q;qual[subsystem]=domain;store["qualifications"]=qual;
    if(subsystem==QStringLiteral("navigation")&&QRegularExpression(QStringLiteral(R"(^N\d+\.\d+$)")).match(id).hasMatch()){
      QJsonObject tuning=store.value("navigation_tuning").toObject(),row=tuning.value(id).toObject();
      if(!row.contains("entry_snapshot")){row["entry_snapshot"]=configSnapshot;row["entry_created_at"]=qualifiedAt;}
      if(status==QStringLiteral("PASS")){
        row["accepted_snapshot"]=configSnapshot;row["accepted_at"]=qualifiedAt;row["entry_snapshot"]=configSnapshot;
        row["entry_created_at"]=qualifiedAt;row["accepted_evidence_trial_ids"]=evidence;row["source"]=QStringLiteral("server_persistent_step_entry");
      }
      tuning[id]=row;store["navigation_tuning"]=tuning;
      if(id==QStringLiteral("N2.1")&&status==QStringLiteral("PASS")){
        QJsonObject results=store.value("calibration_results").toObject(),nav=results.value("navigation").toObject(),cal=nav.value("N2.1").toObject();
        cal["applied"]=true;cal["accepted_at"]=qualifiedAt;
        const QJsonValue accepted=configStoredValue(QStringLiteral("vehicle"),QStringLiteral("vehicle.ros__parameters.drive_erpm_per_mps"));
        if(!accepted.isUndefined())cal["accepted_erpm_per_mps"]=accepted;
        nav["N2.1"]=cal;results["navigation"]=nav;store["calibration_results"]=results;
      }
    }
    store["version"]=3;store["updated_at"]=qualifiedAt;
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

  QJsonObject buildTrialSummary(int trialNo,const QJsonObject &clientAnalysis=QJsonObject()) const {
    const QString analysisCandidate=clientAnalysis.value("candidate").toString().trimmed();
    const QString analysisVariation=clientAnalysis.value("variation").toString().trimmed();
    const QString analysisCondition=clientAnalysis.value("condition").toString().trimmed();
    QJsonObject o{{"trial_no",trialNo},{"trial_id",QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz")},
      {"subsystem",recordingSubsystem_},{"experiment_id",recordingSourceExperimentId_},{"candidate",analysisCandidate.isEmpty()?recordingCandidate_:analysisCandidate},{"variation",analysisVariation.isEmpty()?recordingVariation_:analysisVariation},
      {"condition",analysisCondition.isEmpty()?recordingCondition_:analysisCondition},{"started_at",recordingStartedIso_},{"stopped_at",QDateTime::currentDateTime().toString(Qt::ISODateWithMs)},
      {"samples",recordingRows_.size()},{"sample_rate_hz",recordingRateHz_},{"inputs",recordingTrialInputs_},
      {"config_revision",configRevisionState().value("config_revision")},
      {"task_config_fingerprint",taskConfigFingerprint(recordingSubsystem_,recordingSourceExperimentId_)}};
    auto input=[&](const char*k)->std::optional<double>{const QJsonValue v=recordingTrialInputs_.value(k); if(v.isDouble())return v.toDouble(); bool ok=false;double x=v.toString().toDouble(&ok);return ok?std::optional<double>(x):std::nullopt;};
    putNumber(o,"erpm_set",input("test_erpm")); putNumber(o,"steering_set_deg",input("test_steering_deg"));
    const auto erpm=meanRecording("vesc_right_values.erpm"),raw=meanRecording("esc_drive_raw"),esc=meanRecording("esc_odom.v"),
      gnss=meanRecording("gnss_vel.speed"),imu=meanRecording("imu_speed_kalman"),hacc=meanRecording("gnss_quality.hacc_m");
    putNumber(o,"mean_erpm_esc",erpm);putNumber(o,"mean_v_esc_raw_mps",raw);putNumber(o,"mean_v_esc_mps",esc);
    putNumber(o,"mean_v_gnss_mps",gnss);putNumber(o,"mean_v_imu_mps",imu);putNumber(o,"mean_hacc_m",hacc);
    const auto xp=firstLastRecording("gnss_map_odom.x"),yp=firstLastRecording("gnss_map_odom.y");
    if(xp&&yp){const double dx=xp->second-xp->first,dy=yp->second-yp->first;o["delta_x_m"]=dx;o["delta_y_m"]=dy;o["distance_gnss_m"]=std::hypot(dx,dy);}
    if(erpm&&gnss&&std::abs(*gnss)>0.02)o["erpm_per_mps_candidate"]=std::abs(*erpm)/std::abs(*gnss);
    if(recordingSourceExperimentId_=="N3.1"||recordingSourceExperimentId_=="N3.2"){
      putNumber(o,"mean_steering_actual_rad",meanRecording("esc_steer_actual"));putNumber(o,"mean_gyro_z_rps",meanRecording("imu.gz"));
      putNumber(o,"mean_yaw_rate_model_rps",meanRecording("esc_kinematic_yaw_rate"));o["max_lateral_deviation_m"]=maxLateralDeviation();
      for(const auto &pair:QVector<QPair<QString,QString>>{{"delta_yaw_imu_rad","imu.yaw_rad"},{"delta_yaw_rm3100_rad","rm3100_heading.yaw_rad"},{"delta_cog_gnss_rad","gnss_cog_fusion.yaw_rad"}})
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
    const QJsonObject seriesMeans=clientAnalysis.value("series_means").toObject(),seriesStats=clientAnalysis.value("series_stats").toObject();
    if(!seriesMeans.isEmpty()) o["series_means"] = seriesMeans;
    if(!seriesStats.isEmpty()) o["series_stats"] = seriesStats;
    if(clientAnalysis.value("duration_s").isDouble())o["analysis_duration_s"]=clientAnalysis.value("duration_s");
    o["chart_capture_hz"]=clientAnalysis.value("chart_capture_hz").toDouble(5.0);o["table_capture_hz"]=clientAnalysis.value("table_capture_hz").toDouble(1.0);
    return o;
  }

  bool appendCurrentTrial(const QJsonObject &clientAnalysis,QJsonObject *summary,QJsonArray *trials,QString *message) {
    QJsonObject store=loadTrialStore(); QJsonObject domain=store.value(recordingSubsystem_).toObject();
    QJsonArray list=domain.value(recordingSourceExperimentId_).toArray();
    QJsonObject one=buildTrialSummary(list.size()+1,clientAnalysis); list.append(one);
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

  bool calculateOptimalErpmPerMpsProposal(QString *message,QJsonObject *result) {
    const QJsonArray list = trialList("navigation", "N2.1");
    struct P { double erpm, gnss, erpm_per_mps; QString id; };
    QVector<P> valid;
    QVector<double> candidates;
    for (const auto &v : list) {
      const auto o = v.toObject();
      const double erpm = std::abs(o.value("mean_erpm_esc").toDouble());
      const double gnss = std::abs(o.value("mean_v_gnss_mps").toDouble());
      if (erpm <= 50.0 || gnss <= 0.03) continue;
      const double k = erpm / gnss;
      if (std::isfinite(k) && k >= 100.0 && k <= 50000.0) {
        valid.push_back({erpm, gnss, k, o.value("trial_id").toString()});
        candidates.push_back(k);
      }
    }
    if (valid.size() < 3) {
      if (message) *message = "Butuh minimal 3 trial valid N2.1 untuk kalibrasi eRPM/(m/s)";
      return false;
    }
    const double med = median(candidates);
    QVector<double> dev;
    for (double k : candidates) dev.push_back(std::abs(k - med));
    const double mad = median(dev);
    const double tol = std::max(0.05 * std::abs(med), 3.0 * 1.4826 * mad);
    double xy = 0.0, xx = 0.0;
    int accepted = 0;
    QJsonArray rejected;
    for (const auto &p : valid) {
      if (std::abs(p.erpm_per_mps - med) > tol) { rejected.append(p.id); continue; }
      // Through-origin fit: eRPM = K * V_ground. This is the single runtime conversion.
      xy += p.gnss * p.erpm;
      xx += p.gnss * p.gnss;
      ++accepted;
    }
    if (accepted < 2 || xx <= 1e-9) {
      if (message) *message = "Trial valid setelah outlier rejection tidak cukup";
      return false;
    }
    const double erpm_per_mps = std::clamp(xy / xx, 100.0, 50000.0);
    QJsonObject store = loadTrialStore();
    QJsonObject results = store.value("calibration_results").toObject();
    QJsonObject nav = results.value("navigation").toObject();
    nav["N2.1"] = QJsonObject{{"erpm_per_mps", erpm_per_mps}, {"accepted_trials", accepted},
      {"total_valid_trials", valid.size()}, {"median_candidate", med}, {"mad", mad},
      {"rejected_trial_ids", rejected}, {"applied", false}, {"proposal_only", true},
      {"updated_at", QDateTime::currentDateTime().toString(Qt::ISODateWithMs)}};
    results["navigation"] = nav;
    store["calibration_results"] = results;
    saveTrialStore(store, nullptr);
    if (result) *result = QJsonObject{{"erpm_per_mps", erpm_per_mps}, {"accepted_trials", accepted},
      {"valid_trials", valid.size()}, {"rejected_trial_ids", rejected},
      {"applied", false}, {"proposal_only", true}};
    if (message) *message = "Kalibrasi eRPM/(m/s) berhasil dihitung sebagai proposal; belum ada YAML/runtime write";
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
    const auto artifacts=doc.object();lastXlsxPath_=artifacts.value("xlsx").toString();lastXlsxName_=QFileInfo(lastXlsxPath_).fileName();lastGraphPaths_.clear();lastGraphTitles_.clear();
    for(const auto&v:artifacts.value("pngs").toArray())if(QFileInfo::exists(v.toString()))lastGraphPaths_<<v.toString();
    for(const auto&v:artifacts.value("png_titles").toArray())lastGraphTitles_<<v.toString();
    while(lastGraphTitles_.size()<lastGraphPaths_.size())lastGraphTitles_<<QStringLiteral("Grafik %1").arg(lastGraphTitles_.size()+1);
    int browserPngCount = 0;
    QJsonArray browserPngIndices;
    const int clientExpectedGraphs = clientArtifacts.value("analysis_summary").toObject().value("expected_browser_graphs").toInt(0);
    const int graphSlotCount = std::clamp(std::max(static_cast<int>(recordingGraphs_.size()), clientExpectedGraphs), 0, 32);
    const auto browserPngs = clientArtifacts.value("browser_graph_pngs").toArray();
    for (const auto &v : browserPngs) {
      const QJsonObject one = v.toObject(); const int idx = one.value("index").toInt();
      if (idx < 1 || idx > graphSlotCount || idx > 32) continue;
      QString dataUrl = one.value("data_url").toString();
      const QString prefix = QStringLiteral("data:image/png;base64,");
      if (!dataUrl.startsWith(prefix) || dataUrl.size() > 6 * 1024 * 1024) continue;
      const QByteArray bytes = QByteArray::fromBase64(dataUrl.mid(prefix.size()).toLatin1());
      if (bytes.size() < 8 || bytes.size() > 4 * 1024 * 1024 || !bytes.startsWith(QByteArray::fromHex("89504e470d0a1a0a"))) continue;
      const QString path = QStringLiteral("%1_G%2.png").arg(base).arg(idx, 2, 10, QChar('0'));
      QSaveFile f(path); if (!f.open(QIODevice::WriteOnly) || f.write(bytes) != bytes.size() || !f.commit()) continue;
      const int pos = idx - 1; if (pos < lastGraphPaths_.size()) lastGraphPaths_[pos] = path; else if (QFileInfo::exists(path)) lastGraphPaths_ << path;
      browserPngIndices.append(idx);
      ++browserPngCount;
    }
    if(result){(*result)["xlsx_path"]=lastXlsxPath_;(*result)["xlsx_download_url"]="/api/experiment/record/last.xlsx";(*result)["table_sheet_count"]=tableSheetCount;(*result)["browser_graph_png_count"]=browserPngCount;(*result)["browser_graph_png_indices"]=browserPngIndices;(*result)["expected_graph_count"]=graphSlotCount;QJsonArray urls,paths;for(int i=0;i<lastGraphPaths_.size();++i){urls.append(QString("/api/experiment/record/last-graph-%1.png").arg(i+1));paths.append(lastGraphPaths_[i]);}(*result)["graph_download_urls"]=urls;(*result)["graph_png_paths"]=paths;QJsonArray titles;for(const auto&t:lastGraphTitles_)titles.append(t);(*result)["graph_png_titles"]=titles;}
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
    QString outputRoot = dataRoot;
    if (recordingSubsystem_ == QStringLiteral("perception") && recordingSourceExperimentId_.startsWith(QStringLiteral("F4.")) && recordingStartedMs_ > 0) {
      const QString runName = QStringLiteral("BAB4_") + recordingCsvStem(recordingSourceExperimentId_) + QStringLiteral("_") +
        QDateTime::fromMSecsSinceEpoch(recordingStartedMs_).toString(QStringLiteral("yyyyMMdd_HHmmss"));
      outputRoot = QDir(dataRoot).filePath(runName);
    } else if (recordingSubsystem_ == QStringLiteral("navigation") &&
               recordingSourceExperimentId_ == QStringLiteral("R4.4.5") && recordingStartedMs_ > 0) {
      const QString runName = QStringLiteral("BAB4_MISSION_4_2_4_4_") +
        QDateTime::fromMSecsSinceEpoch(recordingStartedMs_).toString(QStringLiteral("yyyyMMdd_HHmmss"));
      outputRoot = QDir(dataRoot).filePath(runName);
    }
    if (!QDir().mkpath(outputRoot)) {
      if (message) *message = QStringLiteral("Gagal membuat folder data: ") + outputRoot;
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
    while (QFileInfo::exists(QDir(outputRoot).filePath(fileName))) {
      fileName = baseStem + QStringLiteral("_%1.csv").arg(suffix++, 2, 10, QChar('0'));
    }
    const QString primaryPath = QDir(outputRoot).filePath(fileName);
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
                           {"report_root", outputRoot}, {"report_folder", outputRoot},
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
    const bool trialSaved = appendCurrentTrial(clientArtifacts.value("analysis_summary").toObject(),&summary, &trials, &trialMessage);
    bool artifactsSaved = false; QString artifactMessage;
    bool manifestSaved = false; QString manifestMessage;
    bool packageSaved = true; QString packageMessage;
    if (csvSaved && result) {
      const QString csvPath = result->value("primary_csv").toString();
      artifactsSaved = exportTrialArtifacts(csvPath, summary, trials, clientArtifacts, result, &artifactMessage);
      manifestSaved = writeSessionManifest(csvPath, result, &manifestMessage);
      (*result)["trial_summary"] = summary;
      (*result)["trials"] = trials;
      (*result)["trial_yaml"] = trialStorePath();
      (*result)["trial_saved"] = trialSaved;
      (*result)["artifacts_saved"] = artifactsSaved;
      (*result)["zip_download_url"] = QStringLiteral("/api/experiment/record/last.zip");
      (*result)["final_dataset_zip_download_url"] = QStringLiteral("/api/perception/final-dataset/latest.zip");
      if (artifactsSaved && manifestSaved && recordingSubsystem_ == QStringLiteral("navigation") &&
          recordingSourceExperimentId_ == QStringLiteral("R4.4.5")) {
        packageSaved = buildNavigationBab4Package(summary, clientArtifacts, result, &packageMessage);
      }
      (*result)["package_saved"] = packageSaved;
    }
    if (message) *message = csvMessage + QStringLiteral(" | ") + trialMessage + QStringLiteral(" | ") +
      artifactMessage + QStringLiteral(" | ") + manifestMessage + (packageMessage.isEmpty() ? QString() : QStringLiteral(" | ") + packageMessage);
    recordingRows_.clear(); recordingTuningConfig_.clear(); recordingPaths_.clear();
    recordingCandidate_.clear(); recordingTrialInputs_=QJsonObject(); recordingLiveSeries_=QJsonObject(); recordingGraphs_=QJsonArray();
    recordingSubsystem_.clear(); recordingId_.clear(); recordingSourceExperimentId_.clear(); recordingLabel_.clear();
    recordingSectionLabel_.clear(); recordingVariation_.clear(); recordingCondition_.clear(); recordingStartedIso_.clear();
    recordingStartedMs_=0; recordingRateHz_=5.0;
    return csvSaved && trialSaved && artifactsSaved && manifestSaved && packageSaved;
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

  void ensureBab4RecorderForObservedGoal(const QJsonObject &delta) {
    if (!delta.contains(QStringLiteral("goal_request_event")) || recording_) return;
    const QJsonObject goal = delta.value(QStringLiteral("goal_request_event")).toObject();
    QJsonObject trialInputs{
      {QStringLiteral("capture_mode"), QStringLiteral("AUTO_OBSERVED_GOAL_MASTER_4.2_4.4")},
      {QStringLiteral("master_sections"), QJsonArray{
        QStringLiteral("R4.2.1"),QStringLiteral("R4.2.2"),QStringLiteral("R4.2.3"),QStringLiteral("R4.2.4"),QStringLiteral("R4.2.5"),
        QStringLiteral("R4.3.1"),QStringLiteral("R4.3.2"),QStringLiteral("R4.3.3"),QStringLiteral("R4.3.4"),QStringLiteral("R4.3.5"),
        QStringLiteral("R4.4.1"),QStringLiteral("R4.4.2"),QStringLiteral("R4.4.3"),QStringLiteral("R4.4.4"),QStringLiteral("R4.4.5")}},
      {QStringLiteral("goal_x_m"), goal.value(QStringLiteral("x"))},
      {QStringLiteral("goal_y_m"), goal.value(QStringLiteral("y"))},
      {QStringLiteral("goal_yaw_rad"), goal.value(QStringLiteral("yaw"))},
      {QStringLiteral("observed_goal_at_ms"), goal.value(QStringLiteral("at_ms"))}
    };
    QJsonObject payload{
      {QStringLiteral("subsystem"), QStringLiteral("navigation")},
      {QStringLiteral("id"), QStringLiteral("BAB4_MISSION_4.2_4.4")},
      {QStringLiteral("source_experiment_id"), QStringLiteral("R4.4.5")},
      {QStringLiteral("label"), QStringLiteral("BAB4_MISSION_4.2_4.4_AUTO")},
      {QStringLiteral("section_label"), QStringLiteral("BAB4 SEND GOAL MASTER 4.2-4.4")},
      {QStringLiteral("candidate"), QStringLiteral("external-auto")},
      {QStringLiteral("variation"), QStringLiteral("AUTO-OBSERVED-GOAL")},
      {QStringLiteral("condition"), QStringLiteral("FIELD-GOAL")},
      {QStringLiteral("sample_rate_hz"), 20.0},
      {QStringLiteral("trial_inputs"), trialInputs},
      // Empty allow-list intentionally records the complete backend snapshot.
      // This is the strongest raw evidence source for an externally-originated goal.
      {QStringLiteral("record_paths"), QJsonArray{}},
      {QStringLiteral("live_series"), QJsonObject{}},
      {QStringLiteral("graphs"), QJsonArray{}},
      {QStringLiteral("tuning_config"), QJsonObject{}}
    };
    QString why;
    if (startRecording(payload, &why)) {
      RCLCPP_INFO(rclcpp::get_logger("agv_web_gui"),
        "BAB4 AUTO RECORDER armed from observed /navigation/goal_request: x=%.3f y=%.3f",
        goal.value(QStringLiteral("x")).toDouble(), goal.value(QStringLiteral("y")).toDouble());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("agv_web_gui"),
        "BAB4 AUTO RECORDER failed on observed goal: %s", why.toUtf8().constData());
    }
  }

  void broadcastEvents() {
    finalTick();
    for (int i = sseClients_.size() - 1; i >= 0; --i) {
      if (sseClients_[i].isNull() || sseClients_[i]->state() != QAbstractSocket::ConnectedState) sseClients_.removeAt(i);
    }
    QJsonObject delta = bridge_->takeDelta();
    if(!finalActive())ensureBab4RecorderForObservedGoal(delta);
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
    headers += "Content-Security-Policy: default-src 'self'; img-src 'self' data: blob:; style-src 'self'; script-src 'self'; connect-src 'self'\r\n";
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

  QLockFile instanceLock(QDir::temp().filePath(QStringLiteral("agv_web_gui.lock")));
  instanceLock.setStaleLockTime(5000);
  if (!instanceLock.tryLock(0)) {
    qint64 ownerPid = 0;
    QString ownerHost;
    QString ownerApp;
    instanceLock.getLockInfo(&ownerPid, &ownerHost, &ownerApp);
    RCLCPP_FATAL(
      rclcpp::get_logger("agv_web_gui"),
      "Web GUI instance kedua ditolak; lock aktif pid=%lld host=%s app=%s",
      static_cast<long long>(ownerPid), ownerHost.toUtf8().constData(), ownerApp.toUtf8().constData());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 3;
  }

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
