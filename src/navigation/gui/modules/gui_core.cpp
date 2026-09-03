// Extracted from agv_gui.cpp for maintainability.
constexpr double kPi = 3.14159265358979323846;
const QString kAppTitle = QStringLiteral("Autonomous Vehicle Interface");
const QString kGold = QStringLiteral("#d8b033");
const QString kDark = QStringLiteral("#101215");
const QString kPanel = QStringLiteral("#171a1f");
const QString kText = QStringLiteral("#f5f5f5");
const QString kMuted = QStringLiteral("#aab0ba");
const QString kRed = QStringLiteral("#e7462f");
const QString kGreen = QStringLiteral("#46b36b");
const QString kBlue = QStringLiteral("#4b8fe8");
const QString kOrange = QStringLiteral("#e7953f");
QString nowStamp() {
  return QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
}
double normalizeAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}
double yawFromQuat(double x, double y, double z, double w) {
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}
QVariantMap parseSemicolonKv(const QString &text) {
  QVariantMap out;
  for (const QString &chunk : text.split(';', Qt::SkipEmptyParts)) {
    const int eq = chunk.indexOf('=');
    if (eq <= 0) continue;
    const QString key = chunk.left(eq).trimmed();
    const QString raw = chunk.mid(eq + 1).trimmed();
    const QString low = raw.toLower();
    if (low == QStringLiteral("true")) out[key] = true;
    else if (low == QStringLiteral("false")) out[key] = false;
    else {
      bool okInt = false;
      const qlonglong iv = raw.toLongLong(&okInt);
      bool okDouble = false;
      const double dv = raw.toDouble(&okDouble);
      if (okInt && !raw.contains('.') && !raw.contains('e', Qt::CaseInsensitive)) out[key] = iv;
      else if (okDouble) out[key] = dv;
      else out[key] = raw;
    }
  }
  return out;
}
QVariant scalarFromText(QString raw) {
  raw = raw.trimmed();
  const QString low = raw.toLower();
  if (low == QStringLiteral("true") || low == QStringLiteral("on") || low == QStringLiteral("yes")) return true;
  if (low == QStringLiteral("false") || low == QStringLiteral("off") || low == QStringLiteral("no")) return false;
  const QRegularExpression numeric(QStringLiteral("^([-+]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][-+]?[0-9]+)?)"));
  const auto match = numeric.match(raw);
  if (match.hasMatch()) {
    bool ok = false;
    const double value = match.captured(1).toDouble(&ok);
    if (ok) return value;
  }
  return raw;
}
QVariantMap parseFlexibleKv(const QString &text) {
  QVariantMap out = parseSemicolonKv(text);
  const QRegularExpression token(QStringLiteral("(?:^|\\s)([A-Za-z_][A-Za-z0-9_\\-]*)=([^\\s;]+)"));
  auto iterator = token.globalMatch(text);
  while (iterator.hasNext()) {
    const auto match = iterator.next();
    out[match.captured(1)] = scalarFromText(match.captured(2));
  }
  return out;
}
QVariantMap parseJsonOrKv(const QString &text) {
  const QByteArray raw = text.trimmed().toUtf8();
  if (raw.startsWith('{')) {
    QJsonParseError err{
    };
    const QJsonDocument doc = QJsonDocument::fromJson(raw, &err);
    if (err.error == QJsonParseError::NoError && doc.isObject()) return doc.object().toVariantMap();
  }
  QVariantMap m = parseFlexibleKv(text);
  if (m.isEmpty() && !text.trimmed().isEmpty()) m[QStringLiteral("raw")] = text.trimmed();
  return m;
}
double number(const QVariant &v, double fallback);
QVariantMap normalizePerceptionPerformance(QVariantMap map) {
  const auto alias = [&](const QString &target, const QStringList &sources) {
    if (map.contains(target)) return;
    for (const QString &source : sources) {
      if (map.contains(source)) {
        map[target] = map[source];
        return;
      }
    }
  };
  alias(QStringLiteral("fps"), {
    QStringLiteral("pipeline_fps"), QStringLiteral("pipeline_fps_ema")
  });
  alias(QStringLiteral("mean_ms"), {
    QStringLiteral("pipeline_ms_per_frame"), QStringLiteral("pipeline_ms")
  });
  alias(QStringLiteral("p95_ms"), {
    QStringLiteral("pipeline_p95_ms"), QStringLiteral("pipeline_ms_per_frame"), QStringLiteral("pipeline_ms")
  });
  alias(QStringLiteral("capture_dropped"), {
    QStringLiteral("capture_dropped_total")
  });
  alias(QStringLiteral("rviz_dropped"), {
    QStringLiteral("rviz_dropped_total")
  });
  alias(QStringLiteral("raw_count"), {
    QStringLiteral("raw_detection_count"), QStringLiteral("detections")
  });
  alias(QStringLiteral("metric_count"), {
    QStringLiteral("metric_candidate_count"), QStringLiteral("object_points")
  });
  return map;
}
QVariantMap enrichObstacleMetrics(QVariantMap map) {
  const QVariantList detections = map.value(QStringLiteral("detections")).toList();
  map[QStringLiteral("count")] = map.value(QStringLiteral("count"), detections.size());
  double nearest = std::numeric_limits<double>::infinity();
  double nearestLeft = std::numeric_limits<double>::quiet_NaN();
  double nearestHomography = std::numeric_limits<double>::quiet_NaN();
  double nearestScore = std::numeric_limits<double>::quiet_NaN();
  int nearestClass = -1;
  int nearestTrack = -1;
  double confidenceSum = 0.0;
  int confidenceCount = 0;
  for (const QVariant &entry : detections) {
    const QVariantMap detection = entry.toMap();
    const double forward = number(detection.value(QStringLiteral("forward_m")), std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(forward) && forward < nearest) {
      nearest = forward;
      nearestLeft = number(detection.value(QStringLiteral("left_m")), std::numeric_limits<double>::quiet_NaN());
      nearestHomography = number(detection.value(QStringLiteral("forward_homography_m")), forward);
      nearestScore = number(detection.value(QStringLiteral("score")), std::numeric_limits<double>::quiet_NaN());
      nearestClass = detection.value(QStringLiteral("class_id"), -1).toInt();
      nearestTrack = detection.value(QStringLiteral("track_id"), -1).toInt();
    }
    const double confidence = number(detection.value(QStringLiteral("score")), std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(confidence)) {
      confidenceSum += confidence;
      ++confidenceCount;
    }
  }
  if (std::isfinite(nearest)) {
    map[QStringLiteral("nearest_forward_m")] = nearest;
    map[QStringLiteral("nearest_left_m")] = nearestLeft;
    map[QStringLiteral("nearest_forward_homography_m")] = nearestHomography;
    map[QStringLiteral("nearest_score")] = nearestScore;
    map[QStringLiteral("nearest_class_id")] = nearestClass;
    map[QStringLiteral("nearest_track_id")] = nearestTrack;
  }
  if (confidenceCount > 0) map[QStringLiteral("mean_confidence")] = confidenceSum / confidenceCount;
  return map;
}
QVariantMap parseRawDetectionSummary(const QString &text) {
  QVariantMap map = parseJsonOrKv(text);
  QVector<double> confidences;
  QVariantList detections;
  int count = map.value(QStringLiteral("count"), -1).toInt();

  // GPU summary format:
  // person[cls=0](0.923 x=1.25 y=-0.10) car[cls=2](...)
  const QRegularExpression gpuPattern(
    QStringLiteral("[A-Za-z_]+\\[cls=(-?[0-9]+)\\]\\(([-+]?[0-9]*\\.?[0-9]+)\\s+x=([-+]?[0-9]*\\.?[0-9]+|--)\\s+y=([-+]?[0-9]*\\.?[0-9]+|--)\\)"));
  auto gpu = gpuPattern.globalMatch(text);
  int gpuCount = 0;
  while (gpu.hasNext()) {
    const auto match = gpu.next();
    const int classId=match.captured(1).toInt();
    const double score=match.captured(2).toDouble();
    QVariantMap detection{{QStringLiteral("class_id"),classId},{QStringLiteral("score"),score}};
    if(match.captured(3)!=QStringLiteral("--"))detection[QStringLiteral("forward_m")]=match.captured(3).toDouble();
    if(match.captured(4)!=QStringLiteral("--"))detection[QStringLiteral("left_m")]=match.captured(4).toDouble();
    detections<<detection;
    confidences << score;
    ++gpuCount;
  }
  if (gpuCount > 0) count = gpuCount;

  // CPU summary format: backend=cpu;count=2;d0=cls:0,score:0.91;...
  const QRegularExpression cpuPattern(QStringLiteral("d[0-9]+=cls:(-?[0-9]+),score:([-+]?[0-9]*\\.?[0-9]+)"));
  auto cpu = cpuPattern.globalMatch(text);
  int cpuCount=0;
  while (cpu.hasNext()) {
    const auto match=cpu.next();
    const int classId=match.captured(1).toInt();
    const double score=match.captured(2).toDouble();
    detections<<QVariantMap{{QStringLiteral("class_id"),classId},{QStringLiteral("score"),score}};
    confidences << score;
    ++cpuCount;
  }
  if(cpuCount>0 && gpuCount==0)count=cpuCount;
  if (count < 0) count = text.trimmed() == QStringLiteral("none") ? 0 : confidences.size();
  map[QStringLiteral("count")] = count;
  map[QStringLiteral("raw")] = text;
  if(!detections.isEmpty())map[QStringLiteral("detections")]=detections;
  if (!confidences.isEmpty()) {
    const double sum = std::accumulate(confidences.begin(), confidences.end(), 0.0);
    map[QStringLiteral("mean_confidence")] = sum / confidences.size();
    map[QStringLiteral("max_confidence")] = *std::max_element(confidences.begin(), confidences.end());
  }
  return map;
}
double number(const QVariant &v, double fallback = std::numeric_limits<double>::quiet_NaN()) {
  bool ok = false;
  const double d = v.toDouble(&ok);
  return ok && std::isfinite(d) ? d : fallback;
}
// Deep numeric/list equivalence used to avoid rewriting YAML files whose values
// are already correct (int -1 == double -1.0; list elements compared recursively).
bool yamlValueSame(const QVariant &a, const QVariant &b) {
  const double da = number(a), db = number(b);
  if (std::isfinite(da) && std::isfinite(db)) return std::abs(da - db) < 1e-9;
  if (a.userType() == QMetaType::QVariantList && b.userType() == QMetaType::QVariantList) {
    const QVariantList la = a.toList(), lb = b.toList();
    if (la.size() != lb.size()) return false;
    for (int i = 0;
    i < la.size();
    ++i) if (!yamlValueSame(la.at(i), lb.at(i))) return false;
    return true;
  }
  return a == b;
}
QVariant nestedValue(const QVariant &root, const QString &path, const QVariant &fallback = {
}) {
  QVariant cur = root;
  for (const QString &token : path.split('.', Qt::SkipEmptyParts)) {
    if (cur.userType() == QMetaType::QVariantMap) {
      const QVariantMap m = cur.toMap();
      if (!m.contains(token)) return fallback;
      cur = m.value(token);
    }
    else if (cur.userType() == QMetaType::QVariantList) {
      bool ok = false;
      const int idx = token.toInt(&ok);
      const QVariantList list = cur.toList();
      if (!ok || idx < 0 || idx >= list.size()) return fallback;
      cur = list.at(idx);
    }
    else return fallback;
  }
  return cur.isValid() ? cur : fallback;
}
QString variantText(const QVariant &v, int decimals = 3) {
  if (!v.isValid() || v.isNull()) return QStringLiteral("--");
  if (v.userType() == QMetaType::Bool) return v.toBool() ? QStringLiteral("TRUE") : QStringLiteral("FALSE");
  if (v.canConvert<double>()) {
    bool ok=false;
    const double d=v.toDouble(&ok);
    if (ok && std::isfinite(d)) return QString::number(d, 'f', decimals);
  }
  return v.toString();
}
QVariant parseEditorList(const QString &s) {
  const QString t = s.trimmed();
  if (t.startsWith('[')) {
    QJsonParseError err{
    };
    const QJsonDocument doc = QJsonDocument::fromJson(t.toUtf8(), &err);
    if (err.error == QJsonParseError::NoError && doc.isArray()) return doc.array().toVariantList();
  }
  QVariantList out;
  for (const QString &p : t.split(',', Qt::SkipEmptyParts)) {
    const QString q = p.trimmed();
    bool ok=false;
    const double d=q.toDouble(&ok);
    out.push_back(ok ? QVariant(d) : QVariant(q));
  }
  return out;
}
QString csvEscape(QString s) {
  s.replace('"', QStringLiteral("\"\""));
  if (s.contains(',') || s.contains('\n') || s.contains('"')) return QStringLiteral("\"") + s + QStringLiteral("\"");
  return s;
}
QString variantJsonText(const QVariant &v) {
  if (v.userType() == QMetaType::QVariantMap)
  return QString::fromUtf8(QJsonDocument(QJsonObject::fromVariantMap(v.toMap())).toJson(QJsonDocument::Compact));
  if (v.userType() == QMetaType::QVariantList)
  return QString::fromUtf8(QJsonDocument(QJsonArray::fromVariantList(v.toList())).toJson(QJsonDocument::Compact));
  return v.toString();
}
QString valueToInlineYaml(const QVariant &v) {
  if (!v.isValid() || v.isNull()) return QStringLiteral("null");
  const int type = v.userType();
  if (type == QMetaType::Bool) return v.toBool() ? QStringLiteral("true") : QStringLiteral("false");
  if (type == QMetaType::Int || type == QMetaType::LongLong || type == QMetaType::UInt || type == QMetaType::ULongLong)
  return QString::number(v.toLongLong());
  if (type == QMetaType::Double || type == QMetaType::Float) {
    const double d = v.toDouble();
    if (std::isnan(d)) return QStringLiteral(".nan");
    if (std::isinf(d)) return d > 0 ? QStringLiteral(".inf") : QStringLiteral("-.inf");
    QString s = QString::number(d, 'g', 15);
    // Preserve YAML float type: a bare integer (e.g. "0") would be reparsed as
    // an int and break float-only sequences such as velocity_smoother min_velocity.
    if (!s.contains('.') && !s.contains('e', Qt::CaseInsensitive)) s += QStringLiteral(".0");
    return s;
  }
  if (type == QMetaType::QVariantList) {
    QStringList items;
    for (const QVariant &x : v.toList()) items << valueToInlineYaml(x);
    return QStringLiteral("[") + items.join(QStringLiteral(", ")) + QStringLiteral("]");
  }
  if (type == QMetaType::QVariantMap) {
    QStringList items;
    const QVariantMap m=v.toMap();
    for (auto it=m.cbegin();
    it!=m.cend();
    ++it) items << it.key()+QStringLiteral(": ")+valueToInlineYaml(it.value());
    return QStringLiteral("{") + items.join(QStringLiteral(", ")) + QStringLiteral("}");
  }
  QString s=v.toString();
  s.replace('\\', QStringLiteral("\\\\"));
  s.replace('"', QStringLiteral("\\\""));
  return QStringLiteral("\"") + s + QStringLiteral("\"");
}
QString slug(QString s) {
  s=s.toLower();
  s.replace(QRegularExpression(QStringLiteral("[^a-z0-9]+")), QStringLiteral("_"));
  while (s.startsWith('_')) {
    s.remove(0, 1);
  }
  while (s.endsWith('_')) {
    s.chop(1);
  }
  return s;
}
}
// ----------------------------- Workspace paths -----------------------------
struct WorkspacePaths {
  fs::path navShare, navSource, perceptionSource, escSource;
  static fs::path sourceFromShare(const fs::path &share, const std::string &package) {
    const QString envName = QStringLiteral("AGV_%1_SOURCE").arg(QString::fromStdString(package).toUpper());
    const QByteArray env = qgetenv(envName.toUtf8().constData());
    if (!env.isEmpty()) {
      fs::path p = QFileInfo(QString::fromUtf8(env)).absoluteFilePath().toStdString();
      if (fs::exists(p / "package.xml")) return fs::canonical(p);
    }
    const std::string s = share.string();
    const auto pos = s.find("/install/");
    if (pos != std::string::npos) {
      const fs::path ws = s.substr(0,pos);
      const fs::path candidate = ws / "src" / package;
      if (fs::exists(candidate / "package.xml")) return fs::canonical(candidate);
    }
    return share;
  }
  static WorkspacePaths detect() {
    WorkspacePaths p;
    // Resolve mandatory and optional packages independently. Previously one
    // missing optional perception package threw the whole block into a
    // current-working-directory fallback, so GUI edits could target the wrong
    // YAML even though navigation and esc were installed correctly.
    try {
      p.navShare = ament_index_cpp::get_package_share_directory("navigation");
      p.navSource = sourceFromShare(p.navShare,"navigation");
    }
    catch (...) {
      p.navShare = fs::current_path()/"src/navigation";
      p.navSource = p.navShare;
    }
    try {
      const fs::path escShare = ament_index_cpp::get_package_share_directory("esc");
      p.escSource = sourceFromShare(escShare,"esc");
    }
    catch (...) {
      p.escSource = p.navSource.parent_path()/"esc";
    }
    try {
      const fs::path perceptionShare = ament_index_cpp::get_package_share_directory("perception");
      p.perceptionSource = sourceFromShare(perceptionShare,"perception");
    }
    catch (...) {
      // Optional: nonexistent paths load as empty YamlStore objects and all
      // camera pages remain inert while navigation/ESC calibration keeps working.
      p.perceptionSource = p.navSource.parent_path()/"perception";
    }
    return p;
  }
  QMap<QString,QString> fileMap() const {
    auto envDir=[](const char *name,const fs::path &fallback){
      const QByteArray v=qgetenv(name);
      return v.isEmpty()?fallback:fs::path(QString::fromUtf8(v).toStdString());
    };
    const fs::path navCfg=envDir("AGV_CONFIG_DIR",navSource/"config");
    const fs::path escCfg=envDir("AGV_ESC_CONFIG_DIR",escSource/"config");
    const fs::path perCfg=envDir("AGV_PERCEPTION_CONFIG_DIR",perceptionSource/"config");
    QMap<QString,QString> m;
    auto put=[&](const QString &k,const fs::path &v){
      m[k]=QString::fromStdString(v.string());
    };
    put("map",navSource/"maps/undip/undip_nav2.yaml");
    put("map_pgm",navSource/"maps/undip/undip_nav2.pgm");
    put("osm_generator",navSource/"tools/osm_pgm.py");
    put("gui",navCfg/"gui_calibration.yaml");
    put("gnss",navCfg/"gnss.yaml");
    put("imu",navCfg/"imu.yaml");
    put("imu_calibration",navCfg/"imu_calibration.yaml");
    put("ekf",navCfg/"ekf.yaml");
    put("localization",navCfg/"localization_cpp.yaml");
    put("navigation_core",navCfg/"navigation_core.yaml");
    put("nav2",navCfg/"nav2_ackermann.yaml");
    put("mppi",navCfg/"mppi_closed_loop.yaml");
    put("collision",navCfg/"collision_monitor_production.yaml");
    put("trajectory_safety",navCfg/"trajectory_safety.yaml");
    put("stage3",navCfg/"stage3_navigation.yaml");
    put("vehicle",navCfg/"vehicle.yaml");
    put("saved_targets",navCfg/"saved_targets.yaml");
    put("perception",perCfg/"astra_yolop_gpu.yaml");
    put("bbox_calib",perCfg/"bbox_obstacle_calibration.yaml");
    put("esc",escCfg/"ackermann.yaml");
    put("foc_thesis",escSource/"config/foc_thesis.yaml");
    put("teleop",escSource/"config/teleop.yaml");
    return m;
  }
};
// ----------------------------- YAML storage -----------------------------
class YamlStore {
  public:
  explicit YamlStore(QString path):path_(std::move(path)){
    reload();
  }
  QString path() const {
    return path_;
  }
  bool reload(){
    std::lock_guard<std::recursive_mutex> lk(mu_);
    try {
      root_ = QFileInfo::exists(path_)
      ? YAML::LoadFile(path_.toStdString())
      : YAML::Node(YAML::NodeType::Map);
      return true;
    }
    catch (...) {
      root_ = YAML::Node(YAML::NodeType::Map);
      return false;
    }
  }
  QVariant get(const QString &path,const QVariant &fallback={
  }) const {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    YAML::Node n=root_;
    try {
      for(const QString &tok:path.split('.',Qt::SkipEmptyParts)){
        bool ok=false;
        int idx=tok.toInt(&ok);
        n=ok?n[idx]:n[tok.toStdString()];
        if(!n) return fallback;
      }
      return yamlToVariant(n);
    }
    catch(...) {
      return fallback;
    }
  }
  bool set(const QString &path,const QVariant &value,QString *error=nullptr){
    std::lock_guard<std::recursive_mutex> lk(mu_);
    try {
      // Skip no-op writes: an equivalent value must not rewrite (and risk re-serializing) the file.
      if(yamlValueSame(get(path),value))return true;
      const QStringList toks=path.split('.',Qt::SkipEmptyParts);
      if(toks.isEmpty()) return false;
      YAML::Node cur=root_;
      for(int i=0;
      i<toks.size()-1;
      ++i){
        bool ok=false;
        int idx=toks[i].toInt(&ok);
        if(ok){
          while(cur.size()<=static_cast<size_t>(idx))cur.push_back(YAML::Node());
          cur=cur[idx];
        }
        else {
          if(!cur[toks[i].toStdString()]) cur[toks[i].toStdString()]=YAML::Node(YAML::NodeType::Map);
          cur=cur[toks[i].toStdString()];
        }
      }
      bool lastIdx=false;
      int idx=toks.last().toInt(&lastIdx);
      if(lastIdx){
        while(cur.size()<=static_cast<size_t>(idx))cur.push_back(YAML::Node());
        cur[idx]=variantToYaml(value);
      }
      else cur[toks.last().toStdString()]=variantToYaml(value);
      QString patchPath=path;
      QVariant patchValue=value;
      if(lastIdx){
        patchPath=toks.mid(0,toks.size()-1).join('.');
        patchValue=get(patchPath);
      }
      if(!patchExisting(patchPath,patchValue)) fullAtomicSave();
      reload();
      return true;
    }
    catch(const std::exception &e){
      if(error)*error=QString::fromUtf8(e.what());
      return false;
    }
  }
  QByteArray raw() const {
    QFile f(path_);
    if(!f.open(QIODevice::ReadOnly)) return {
    };
    return f.readAll();
  }
  bool restoreRaw(const QByteArray &raw){
    QSaveFile f(path_);
    if(!f.open(QIODevice::WriteOnly))return false;
    f.write(raw);
    if(!f.commit())return false;
    reload();
    return true;
  }
  private:
  QString path_;
  mutable std::recursive_mutex mu_;
  YAML::Node root_;
  static QVariant yamlToVariant(const YAML::Node &n){
    if(!n||n.IsNull())return {
    };
    if(n.IsSequence()){
      QVariantList l;
      for(const auto &x:n)l<<yamlToVariant(x);
      return l;
    }
    if(n.IsMap()){
      QVariantMap m;
      for(auto it=n.begin();
      it!=n.end();
      ++it)m[QString::fromStdString(it->first.as<std::string>())]=yamlToVariant(it->second);
      return m;
    }
    const QString s=QString::fromStdString(n.as<std::string>());
    const QString low=s.toLower();
    if(low=="true")return true;
    if(low=="false")return false;
    bool okI=false;
    qlonglong iv=s.toLongLong(&okI);
    if(okI&&!s.contains('.')&&!s.contains('e',Qt::CaseInsensitive))return iv;
    bool okD=false;
    double d=s.toDouble(&okD);
    if(okD)return d;
    return s;
  }
  static YAML::Node variantToYaml(const QVariant &v){
    if (!v.isValid() || v.isNull()) {
      return YAML::Node();
    }
    const int t = v.userType();
    if(t==QMetaType::Bool)return YAML::Node(v.toBool());
    if(t==QMetaType::Int||t==QMetaType::LongLong||t==QMetaType::UInt||t==QMetaType::ULongLong)return YAML::Node(static_cast<long long>(v.toLongLong()));
    if(t==QMetaType::Double||t==QMetaType::Float)return YAML::Node(v.toDouble());
    if(t==QMetaType::QVariantList){
      YAML::Node n(YAML::NodeType::Sequence);
      for(const QVariant &x:v.toList())n.push_back(variantToYaml(x));
      return n;
    }
    if(t==QMetaType::QVariantMap){
      YAML::Node n(YAML::NodeType::Map);
      for(auto it=v.toMap().cbegin();
      it!=v.toMap().cend();
      ++it)n[it.key().toStdString()]=variantToYaml(it.value());
      return n;
    }
    return YAML::Node(v.toString().toStdString());
  }
  bool patchExisting(const QString &path,const QVariant &value){
    QFile in(path_);
    if(!in.open(QIODevice::ReadOnly|QIODevice::Text))return false;
    QString text=QString::fromUtf8(in.readAll());
    in.close();
    QStringList lines=text.split('\n');
    const QStringList wanted=path.split('.',Qt::SkipEmptyParts);
    QVector<QPair<int,QString>> stack;
    int target=-1;
    for(int i=0;
    i<lines.size();
    ++i){
      QString raw=lines[i];
      QString stripped=raw.trimmed();
      if(stripped.isEmpty()||stripped.startsWith('#')||stripped.startsWith('-')||!stripped.contains(':'))continue;
      int indent=raw.size()-raw.trimmed().size();
      QString key=stripped.section(':',0,0).trimmed();
      key.remove('"');
      key.remove('\'');
      while(!stack.isEmpty()&&indent<=stack.last().first)stack.removeLast();
      QStringList full;
      for(const auto &p:stack)full<<p.second;
      full<<key;
      if(full==wanted){
        target=i;
        break;
      }
      stack.push_back({
        indent,key
      });
    }
    if(target<0)return false;
    QString raw=lines[target];
    const int colon=raw.indexOf(':');
    if(colon<0)return false;
    const QString prefix=raw.left(colon+1);
    QString rest=raw.mid(colon+1);
    QString comment;
    bool quote=false;
    int hash=-1;
    for(int j=0;
    j<rest.size();
    ++j){
      if(rest[j]=='"'&&(j==0||rest[j-1]!='\\'))quote=!quote;
      if(rest[j]=='#'&&!quote){
        hash=j;
        break;
      }
    }
    if(hash>=0)comment=QStringLiteral(" ")+rest.mid(hash).trimmed();
    lines[target]=prefix+QStringLiteral(" ")+valueToInlineYaml(value)+comment;
    QSaveFile out(path_);
    if(!out.open(QIODevice::WriteOnly|QIODevice::Text))return false;
    out.write(lines.join('\n').toUtf8());
    return out.commit();
  }
  void fullAtomicSave(){
    YAML::Emitter e;
    e<<root_;
    QSaveFile f(path_);
    if(!f.open(QIODevice::WriteOnly|QIODevice::Text))throw std::runtime_error("cannot write yaml");
    f.write(QByteArray(e.c_str(),static_cast<int>(e.size())));
    f.write("\n");
    if(!f.commit())throw std::runtime_error("yaml atomic commit failed");
  }
};
struct TelemetryEvent {
  quint64 sequence{0};
  double received_sec{0.0};
  QVariant value;
};
class TelemetryStore {
  public:
  void update(const QString &channel,const QVariant &value){
    QMutexLocker lk(&mu_);
    const double now=QDateTime::currentMSecsSinceEpoch()/1000.0;
    values_[channel]=value;
    times_[channel]=now;
    const quint64 seq=++sequences_[channel];
    // BAB-IV final perception acquisition must consume every perception event,
    // not merely poll the latest value at the GUI refresh rate.  Keep a bounded
    // event history only for channels used by the perception recorder so other
    // GUI subsystems keep the exact same lightweight behaviour.
    static const QSet<QString> historyChannels={
      QStringLiteral("raw_detections"), QStringLiteral("obstacle_metrics"),
      QStringLiteral("perception_performance"), QStringLiteral("lane_state"),
      QStringLiteral("drivable_space"), QStringLiteral("near_field_state"),
      QStringLiteral("trajectory_safety_state"), QStringLiteral("perception_emergency")
    };
    if(historyChannels.contains(channel)){
      auto &queue=history_[channel];
      queue.push_back(TelemetryEvent{seq,now,value});
      while(queue.size()>4096)queue.pop_front();
    }
  }
  QVariant get(const QString &path,const QVariant &fallback={
  }) const {
    QMutexLocker lk(&mu_);
    // Some ROS channels intentionally contain dots (for example
    // "connected.gnss" and "system.nav2_ready").  Resolve an exact channel
    // first; only interpret dots as nested QVariantMap access when there is no
    // exact channel.  Without this order every readiness pill stays OFF even
    // though the latched Bool topic has arrived.
    if(values_.contains(path))return values_.value(path);
    const QString channel=path.section('.',0,0);
    if(!values_.contains(channel))return fallback;
    if(path==channel)return values_.value(channel);
    return nestedValue(values_.value(channel),path.mid(channel.size()+1),fallback);
  }
  double timestamp(const QString &channel) const {
    QMutexLocker lk(&mu_);
    return times_.value(channel,0.0);
  }
  quint64 sequence(const QString &channel) const {
    QMutexLocker lk(&mu_);
    return sequences_.value(channel,0);
  }
  QVector<TelemetryEvent> eventsSince(const QString &channel,quint64 afterSequence,int maximum=8192) const {
    QMutexLocker lk(&mu_);
    QVector<TelemetryEvent> out;
    const auto it=history_.constFind(channel);
    if(it==history_.cend())return out;
    for(const TelemetryEvent &event:it.value()){
      if(event.sequence<=afterSequence)continue;
      out<<event;
      if(out.size()>=maximum)break;
    }
    return out;
  }
  double age(const QString &channel) const {
    QMutexLocker lk(&mu_);
    if(!times_.contains(channel))return std::numeric_limits<double>::infinity();
    return QDateTime::currentMSecsSinceEpoch()/1000.0-times_.value(channel);
  }
  private: mutable QMutex mu_;
  QHash<QString,QVariant> values_;
  QHash<QString,double> times_;
  QHash<QString,quint64> sequences_;
  QHash<QString,std::deque<TelemetryEvent>> history_;
};
class NoWheelSpinBox:public QSpinBox{
  protected:void wheelEvent(QWheelEvent *e)override{
    e->ignore();
  }
};
class NoWheelDoubleSpinBox:public QDoubleSpinBox{
  protected:void wheelEvent(QWheelEvent *e)override{
    e->ignore();
  }
};
class NoWheelComboBox:public QComboBox{
  protected:void wheelEvent(QWheelEvent *e)override{
    e->ignore();
  }
};
class StatusPill:public QLabel{
  public:explicit StatusPill(const QString&t=QStringLiteral("UNKNOWN")){
    setAlignment(Qt::AlignCenter);
    setMinimumWidth(86);
    setStatus("unknown",t);
  }
  void setStatus(const QString&s,const QString&t={
  }){
    if(!t.isEmpty())setText(t);
    QString c=s=="ok"?kGreen:s=="bad"||s=="error"?kRed:s=="warn"?kOrange:kMuted;
    setStyleSheet(QStringLiteral("QLabel{background:%1;color:#080808;border-radius:9px;padding:4px 8px;font-weight:800;}").arg(c));
  }
};
class PoseEntryDialog:public QDialog{
  public:
  PoseEntryDialog(const QString&title,const QString&name,double x,double y,double yawDeg,const QString&description,QWidget*parent=nullptr):QDialog(parent){
    setWindowTitle(title);
    setModal(true);
    setMinimumWidth(380);
    auto*l=new QVBoxLayout(this);
    auto*f=new QFormLayout();
    name_=new QLineEdit(name);
    x_=new NoWheelDoubleSpinBox();
    y_=new NoWheelDoubleSpinBox();
    yaw_=new NoWheelDoubleSpinBox();
    desc_=new QLineEdit(description);
    for(auto*w:{
      x_,y_
    }){
      w->setRange(-1e6,1e6);
      w->setDecimals(6);
    }
    x_->setValue(x);
    y_->setValue(y);
    yaw_->setRange(-180,180);
    yaw_->setDecimals(3);
    yaw_->setSuffix("°");
    yaw_->setValue(yawDeg);
    f->addRow("Nama",name_);
    f->addRow("Map X",x_);
    f->addRow("Map Y",y_);
    f->addRow("Yaw",yaw_);
    f->addRow("Keterangan",desc_);
    l->addLayout(f);
    auto*b=new QDialogButtonBox(QDialogButtonBox::Save|QDialogButtonBox::Cancel);
    connect(b,&QDialogButtonBox::accepted,this,&QDialog::accept);
    connect(b,&QDialogButtonBox::rejected,this,&QDialog::reject);
    l->addWidget(b);
  }
  QString name()const{
    return name_->text().trimmed();
  }
  double x()const{
    return x_->value();
  }
  double y()const{
    return y_->value();
  }
  double yawDeg()const{
    return yaw_->value();
  }
  QString description()const{
    return desc_->text().trimmed();
  }
  private:QLineEdit*name_;
  NoWheelDoubleSpinBox*x_,*y_,*yaw_;
  QLineEdit*desc_;
};
