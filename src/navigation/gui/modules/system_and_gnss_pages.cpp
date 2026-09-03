// Extracted from agv_gui.cpp for maintainability.
class DiagnosticPage:public QWidget{
  public:DiagnosticPage(const QString&title,const QString&desc,TelemetryStore*telemetry,ReportManager*reports,const QMap<QString,QString>&series,const QStringList&raw,QWidget*p=nullptr):QWidget(p),telemetry_(telemetry),reports_(reports),series_(series),raw_(raw),title_(title){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel(title);
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*d=new QLabel(desc);
    d->setWordWrap(true);
    d->setObjectName("pageDescription");
    l->addWidget(d);
    auto*cards=new QGridLayout();
    int c=0;
    for(auto it=series_.cbegin();
    it!=series_.cend();
    ++it){
      auto*lab=new QLabel(it.key()+": --");
      lab->setObjectName("metricCard");
      metrics_[it.key()]=lab;
      cards->addWidget(lab,c/3,c%3);
      ++c;
    }
    l->addLayout(cards);
    plot_=new LivePlotWidget(title);
    l->addWidget(plot_,1);
    rawText_=new QPlainTextEdit();
    rawText_->setReadOnly(true);
    rawText_->setMaximumBlockCount(250);
    rawText_->setMaximumHeight(170);
    l->addWidget(rawText_);
    auto*bar=new QHBoxLayout();
    record_=new QPushButton("● Mulai Rekam CSV");
    auto*clear=new QPushButton("Bersihkan Grafik");
    auto*png=new QPushButton("Ekspor PNG");
    bar->addWidget(record_);
    bar->addWidget(clear);
    bar->addWidget(png);
    bar->addStretch();
    l->addLayout(bar);
    connect(record_,&QPushButton::clicked,this,[this](){
      toggleRecording();
    });
    connect(clear,&QPushButton::clicked,plot_,&LivePlotWidget::clear);
    connect(png,&QPushButton::clicked,this,[this](){
      QString p=reports_->savePng(slug(title_),plot_);
      if(!p.isEmpty())QMessageBox::information(this,"Ekspor PNG",p);
    });
    timer_=new QTimer(this);
    timer_->setInterval(200);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
  }
  void refresh(){
    QMap<QString,double>vals;
    for(auto it=series_.cbegin();
    it!=series_.cend();
    ++it){
      QVariant v=telemetry_->get(it.value());
      double x=number(v);
      metrics_[it.key()]->setText(it.key()+": "+variantText(v));
      if(std::isfinite(x))vals[it.key()]=x;
    }
    if(!vals.isEmpty())plot_->append(vals);
    QStringList rawLines;
    for(const QString&ch:raw_){
      QVariant v=telemetry_->get(ch);
      if(v.isValid()){
        QString s;
        if(v.userType()==QMetaType::QVariantMap)s=QString::fromUtf8(QJsonDocument(QJsonObject::fromVariantMap(v.toMap())).toJson(QJsonDocument::Compact));
        else s=v.toString();
        rawLines<<ch+": "+s;
      }
    }
    rawText_->setPlainText(rawLines.join('\n'));
    if(recording_){
      QVariantMap row;
      row["time_iso"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      for(auto it=series_.cbegin();
      it!=series_.cend();
      ++it)row[it.key()]=telemetry_->get(it.value());
      for(const QString&ch:raw_)row[ch]=telemetry_->get(ch);
      rows_.push_back(row);
    }
  }
  private:TelemetryStore*telemetry_;
  ReportManager*reports_;
  QMap<QString,QString>series_;
  QStringList raw_;
  QString title_;
  QHash<QString,QLabel*>metrics_;
  LivePlotWidget*plot_;
  QPlainTextEdit*rawText_;
  QPushButton*record_;
  QTimer*timer_;
  bool recording_=false;
  QVector<QVariantMap>rows_;
  void toggleRecording(){
    if(!recording_){
      rows_.clear();
      recording_=true;
      record_->setText("■ Stop + Simpan CSV");
    }
    else{
      recording_=false;
      record_->setText("● Mulai Rekam CSV");
      QString p=reports_->saveCsv(slug(title_),rows_);
      QMessageBox::information(this,"CSV tersimpan",p);
    }
  }
};
class ConnectionPage:public QWidget{
  public:explicit ConnectionPage(TelemetryStore*t,QWidget*p=nullptr):QWidget(p),t_(t){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Koneksi & Readiness Sistem");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*g=new QGridLayout();
    const QStringList names={
      "GNSS","IMU","CAMERA","ESC READY","ESC ARMED","NAV2","AUTONOMY","MOTION","E-STOP"
    };
    const QStringList keys={
      "connected.gnss","connected.imu","connected.camera","connected.esc_ready","connected.esc_armed","system.nav2_ready","system.autonomy_ready","system.motion_ready","system.estop"
    };
    for(int i=0;
    i<names.size();
    ++i){
      auto*pill=new StatusPill(names[i]);
      pills_.push_back(pill);
      keys_.push_back(keys[i]);
      g->addWidget(new QLabel(names[i]),i/3*2,i%3);
      g->addWidget(pill,i/3*2+1,i%3);
    }
    l->addLayout(g);
    auto*note=new QLabel("Hijau berarti data/readiness aktif. E-STOP hijau hanya saat false (aman). Periksa source serial dan launch jika status tetap OFF.");
    note->setWordWrap(true);
    l->addWidget(note);
    l->addStretch();
    auto*tm=new QTimer(this);
    tm->setInterval(250);
    connect(tm,&QTimer::timeout,this,[this](){
      for(int i=0;
      i<pills_.size();
      ++i){
        bool v=t_->get(keys_[i],false).toBool();
        if(keys_[i]=="system.estop")pills_[i]->setStatus(v?"bad":"ok",v?"E-STOP":"AMAN");
        else pills_[i]->setStatus(v?"ok":"bad",v?"READY":"OFF");
      }
    });
    tm->start();
  }
  private:TelemetryStore*t_;
  QVector<StatusPill*>pills_;
  QStringList keys_;
};
class NavigationTuningPage:public QWidget{
  public:NavigationTuningPage(TelemetryStore*t,ReportManager*r,const QMap<QString,std::shared_ptr<YamlStore>>&s,QWidget*p=nullptr):QWidget(p),t_(t),r_(r),s_(s){
    auto*l=new QVBoxLayout(this);
    auto*h=new QLabel("Navigasi & MPPI — Tuning Terukur");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto*d=new QLabel("Bandingkan command Nav2 dengan feedback kendaraan. Profile A/B/C hanya mengubah YAML dan selalu membuat backup; lakukan restart lifecycle sebelum run pengujian.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto*box=new QGroupBox("Profil MPPI");
    auto*g=new QGridLayout(box);
    profile_=new NoWheelComboBox();
    profile_->addItems({
      "A - Konservatif","B - Balanced","C - Responsif"
    });
    auto*apply=new QPushButton("Terapkan Profil");
    auto*sweep=new QPushButton("Buat Paket Sweep");
    g->addWidget(new QLabel("Profil"),0,0);
    g->addWidget(profile_,0,1);
    g->addWidget(apply,0,2);
    g->addWidget(sweep,0,3);
    summary_=new QLabel();
    summary_->setWordWrap(true);
    g->addWidget(summary_,1,0,1,4);
    l->addWidget(box);
    auto*sm=new QGroupBox("Velocity Smoother");
    auto*sg=new QHBoxLayout(sm);
    smoother_=new NoWheelComboBox();
    smoother_->addItems({
      "OPEN_LOOP","CLOSED_LOOP"
    });
    auto*save=new QPushButton("Tulis Mode ke YAML");
    eligible_=new StatusPill("BELUM QUALIFIED");
    sg->addWidget(new QLabel("Feedback"));
    sg->addWidget(smoother_);
    sg->addWidget(save);
    sg->addWidget(eligible_);
    sg->addStretch();
    l->addWidget(sm);
    plot_=new LivePlotWidget("Command vs Feedback / Error");
    l->addWidget(plot_,1);
    record_=new QPushButton("● Mulai Run Tuning");
    l->addWidget(record_);
    connect(profile_,&QComboBox::currentTextChanged,this,[this](){
      updateSummary();
    });
    connect(apply,&QPushButton::clicked,this,[this](){
      applyProfile();
    });
    connect(sweep,&QPushButton::clicked,this,[this](){
      generateSweep();
    });
    connect(save,&QPushButton::clicked,this,[this](){
      if(smoother_->currentText()=="CLOSED_LOOP"&&!t_->get("smoother_closed_loop_eligible",false).toBool()){
        QMessageBox::warning(this,"CLOSED_LOOP","Qualification belum PASS.");
        return;
      }
      s_["nav2"]->set("velocity_smoother.ros__parameters.feedback",smoother_->currentText());
      QMessageBox::information(this,"Velocity smoother","YAML tersimpan. Restart velocity_smoother sebelum pengujian.");
    });
    connect(record_,&QPushButton::clicked,this,[this](){
      if(!rec_){
        rows_.clear();
        rec_=true;
        record_->setText("■ Stop + Analisis CSV");
      }
      else{
        rec_=false;
        record_->setText("● Mulai Run Tuning");
        QMessageBox::information(this,"Run selesai",r_->saveCsv("mppi_tuning",rows_));
      }
    });
    timer_=new QTimer(this);
    timer_->setInterval(150);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
    updateSummary();
  }
  private:TelemetryStore*t_;
  ReportManager*r_;
  QMap<QString,std::shared_ptr<YamlStore>>s_;
  QComboBox*profile_;
  QComboBox*smoother_;
  StatusPill*eligible_;
  QLabel*summary_;
  LivePlotWidget*plot_;
  QPushButton*record_;
  QTimer*timer_;
  bool rec_=false;
  QVector<QVariantMap>rows_;
  QVariantMap profileValues()const{
    QString p=profile_->currentText();
    if(p.startsWith('A'))return{
      {
        "vx_max",0.20
      },{
        "vx_std",0.10
      },{
        "wz_std",0.15
      },{
        "temperature",0.20
      },{
        "batch_size",1000
      },{
        "path_align",18.0
      },{
        "path_follow",8.0
      },{
        "path_angle",4.0
      }
    };
    if(p.startsWith('C'))return{
      {
        "vx_max",0.40
      },{
        "vx_std",0.25
      },{
        "wz_std",0.35
      },{
        "temperature",0.35
      },{
        "batch_size",1800
      },{
        "path_align",12.0
      },{
        "path_follow",12.0
      },{
        "path_angle",6.0
      }
    };
    return{
      {
        "vx_max",0.30
      },{
        "vx_std",0.20
      },{
        "wz_std",0.25
      },{
        "temperature",0.30
      },{
        "batch_size",1400
      },{
        "path_align",15.0
      },{
        "path_follow",10.0
      },{
        "path_angle",5.0
      }
    };
  }
  void updateSummary(){
    auto v=profileValues();
    summary_->setText(QString("vx_max=%1 m/s • vx_std=%2 • wz_std=%3 • temperature=%4 • batch=%5").arg(v["vx_max"].toDouble()).arg(v["vx_std"].toDouble()).arg(v["wz_std"].toDouble()).arg(v["temperature"].toDouble()).arg(v["batch_size"].toInt()));
  }
  void applyProfile(){
    auto st=s_.value("nav2");
    auto veh=s_.value("vehicle");
    if(!st||!veh)return;
    const QString backup=st->path()+".before_"+nowStamp()+".bak";
    QFile::copy(st->path(),backup);
    auto v=profileValues();
    double reqV=v["vx_max"].toDouble(),vehicleV=number(veh->get("vehicle.ros__parameters.max_forward_speed_mps",0.5),0.5),rmin=number(veh->get("vehicle.ros__parameters.minimum_turning_radius_m",1.712159378317),1.712159378317),vehicleYaw=number(veh->get("vehicle.ros__parameters.max_yaw_rate_rps",0.292028888392),0.292028888392);
    double vx=std::min(reqV,vehicleV),wz=std::min(vehicleYaw,vx/std::max(0.10,rmin));
    st->set("controller_server.ros__parameters.FollowPath.vx_max",vx);
    st->set("controller_server.ros__parameters.FollowPath.wz_max",wz);
    st->set("controller_server.ros__parameters.FollowPath.vx_std",v["vx_std"]);
    st->set("controller_server.ros__parameters.FollowPath.wz_std",v["wz_std"]);
    st->set("controller_server.ros__parameters.FollowPath.temperature",v["temperature"]);
    st->set("controller_server.ros__parameters.FollowPath.batch_size",v["batch_size"]);
    st->set("controller_server.ros__parameters.FollowPath.PathAlignCritic.cost_weight",v["path_align"]);
    st->set("controller_server.ros__parameters.FollowPath.PathFollowCritic.cost_weight",v["path_follow"]);
    st->set("controller_server.ros__parameters.FollowPath.PathAngleCritic.cost_weight",v["path_angle"]);
    st->set("velocity_smoother.ros__parameters.max_velocity",QVariantList{
      vx,0.0,wz
    });
    st->set("velocity_smoother.ros__parameters.min_velocity",QVariantList{
      0.0,0.0,-wz
    });
    auto ts=s_.value("trajectory_safety");
    if(ts){
      ts->set("trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps",wz);
      ts->set("trajectory_safety_supervisor.ros__parameters.minimum_turning_radius_m",rmin);
    }
    QMessageBox::information(this,"MPPI profile",QString("Profile tersimpan aman: vx_max=%1 m/s, wz_max=%2 rad/s, Rmin=%3 m.\nBackup: %4\nRestart controller_server + velocity_smoother sebelum run.").arg(vx).arg(wz).arg(rmin).arg(backup));
  }
  void generateSweep(){
    QString dir=QDir(r_->root()).filePath("mppi_sweep_"+nowStamp());
    QDir().mkpath(dir);
    QString old=profile_->currentText();
    for(const QString&p:{
      QString("A - Konservatif"),QString("B - Balanced"),QString("C - Responsif")
    }){
      profile_->setCurrentText(p);
      QJsonObject o=QJsonObject::fromVariantMap(profileValues());
      QSaveFile f(QDir(dir).filePath(slug(p)+".json"));
      if(f.open(QIODevice::WriteOnly)){
        f.write(QJsonDocument(o).toJson(QJsonDocument::Indented));
        f.commit();
      }
    }
    profile_->setCurrentText(old);
    QMessageBox::information(this,"Sweep Pack",dir);
  }
  void refresh(){
    bool ok=t_->get("smoother_closed_loop_eligible",false).toBool();
    eligible_->setStatus(ok?"ok":"warn",ok?"CLOSED_LOOP ELIGIBLE":"OPEN_LOOP DISARANKAN");
    QMap<QString,double>v;
    for(auto p:{
      std::pair<QString,QString>{
        "v err","mppi_velocity_error"
      },{
        "steer err","mppi_steering_error"
      },{
        "yaw err","mppi_yaw_error"
      },{
        "target v","esc_drive_target"
      },{
        "actual v","esc_drive_actual"
      }
    }){
      double x=number(t_->get(p.second));
      if(std::isfinite(x))v[p.first]=x;
    }
    if(!v.isEmpty())plot_->append(v);
    if(rec_){
      QVariantMap row{
        {
          "time",QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
        }
      };
      for(auto it=v.cbegin();
      it!=v.cend();
      ++it)row[it.key()]=it.value();
      row["smoother_eligible"]=ok;
      rows_.push_back(row);
    }
  }
};
class HostMetricsSampler {
  public:
  QVariant value(const QString &path) {
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if (nowMs - lastSampleMs_ > 900) {
      sample();
      lastSampleMs_ = nowMs;
    }
    return cache_.value(path);
  }
  private:
  qint64 lastSampleMs_{
    0
  };
  qulonglong previousTotal_{
    0
  }, previousIdle_{
    0
  };
  QVariantMap cache_;
  static double readNumber(const QString &path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return std::numeric_limits<double>::quiet_NaN();
    bool ok = false;
    const double value = QString::fromUtf8(file.readAll()).trimmed().toDouble(&ok);
    return ok ? value : std::numeric_limits<double>::quiet_NaN();
  }
  void sample() {
    QFile mem(QStringLiteral("/proc/meminfo"));
    if (mem.open(QIODevice::ReadOnly | QIODevice::Text)) {
      const QString text = QString::fromUtf8(mem.readAll());
      const auto totalMatch = QRegularExpression(QStringLiteral("MemTotal:\\s+([0-9]+)")).match(text);
      const auto availableMatch = QRegularExpression(QStringLiteral("MemAvailable:\\s+([0-9]+)")).match(text);
      if (totalMatch.hasMatch() && availableMatch.hasMatch()) {
        const double totalKb = totalMatch.captured(1).toDouble();
        const double availableKb = availableMatch.captured(1).toDouble();
        cache_[QStringLiteral("host.ram_used_gb")] = (totalKb - availableKb) / 1048576.0;
        cache_[QStringLiteral("host.ram_percent")] = totalKb > 0.0 ? 100.0 * (totalKb - availableKb) / totalKb : 0.0;
      }
    }
    QFile stat(QStringLiteral("/proc/stat"));
    if (stat.open(QIODevice::ReadOnly | QIODevice::Text)) {
      const QStringList fields = QString::fromUtf8(stat.readLine()).simplified().split(' ');
      if (fields.size() >= 8 && fields[0] == QStringLiteral("cpu")) {
        qulonglong total = 0;
        for (int index = 1;
        index < fields.size();
        ++index) total += fields[index].toULongLong();
        const qulonglong idle = fields[4].toULongLong() + (fields.size() > 5 ? fields[5].toULongLong() : 0ULL);
        if (previousTotal_ > 0 && total > previousTotal_) {
          const qulonglong deltaTotal = total - previousTotal_;
          const qulonglong deltaIdle = idle - previousIdle_;
          cache_[QStringLiteral("host.cpu_percent")] = 100.0 * double(deltaTotal - std::min(deltaIdle, deltaTotal)) / double(deltaTotal);
        }
        previousTotal_ = total;
        previousIdle_ = idle;
      }
    }
    double maximumTemperature = std::numeric_limits<double>::quiet_NaN();
    QDirIterator thermal(QStringLiteral("/sys/class/thermal"), QStringList{
      QStringLiteral("thermal_zone*")
    }, QDir::Dirs | QDir::NoDotAndDotDot);
    while (thermal.hasNext()) {
      const QString directory = thermal.next();
      double temperature = readNumber(QDir(directory).filePath(QStringLiteral("temp")));
      if (std::isfinite(temperature) && temperature > 1000.0) temperature /= 1000.0;
      if (std::isfinite(temperature)) maximumTemperature = std::isfinite(maximumTemperature) ? std::max(maximumTemperature, temperature) : temperature;
    }
    if (std::isfinite(maximumTemperature)) cache_[QStringLiteral("host.temperature_c")] = maximumTemperature;
    for (const QString &candidate : {
      QStringLiteral("/sys/class/drm/card0/device/gpu_busy_percent"),
      QStringLiteral("/sys/devices/gpu.0/load")
    }) {
      double load = readNumber(candidate);
      if (!std::isfinite(load)) continue;
      if (load > 100.0) load /= 10.0;
      cache_[QStringLiteral("host.gpu_percent")] = std::clamp(load, 0.0, 100.0);
      break;
    }
  }
};
struct ExperimentSessionData {
  QVector<QVariantMap> rawRows;
  QVector<QVariantMap> summaryRows;
  QMap<QString, QVector<QPointF>> liveSeries;
  QMap<int, QVector<QPointF>> liveScatter;
  QMap<int, QPointF> scatterRawOrigin;  // raw lon/lat origin for metric GNSS scatter
};
class ExperimentWorkspacePage : public QWidget {
  public:
  explicit ExperimentWorkspacePage(const QString &subsystem, TelemetryStore *telemetry, ReportManager *reports,
  const QMap<QString, std::shared_ptr<YamlStore>> &stores, RosBridge *ros = nullptr, QWidget *parent = nullptr)
  : QWidget(parent), subsystem_(subsystem), telemetry_(telemetry), reports_(reports), stores_(stores), ros_(ros),
  catalog_(buildExperimentCatalog(subsystem)) {
    for (int i = 0;
    i < catalog_.size();
    ++i) leafIndex_[catalog_[i].id] = i;
    auto *layout = new QVBoxLayout(this);
    auto *title = new QLabel(subsystem_==QStringLiteral("navigation")
      ? QStringLiteral("NAVIGASI — Tuning & Commissioning Autonomous N0 → N17")
      : subsystemTitle() + QStringLiteral(" — Akuisisi Data BAB IV"));
    title->setObjectName(QStringLiteral("pageTitle"));
    layout->addWidget(title);
    auto *description = new QLabel(subsystem_==QStringLiteral("navigation")
      ? QStringLiteral("Urutan commissioning: N0 timing → N1/N3 fisik & odometri → N4/N5 sensor → N7/N8 EKF P-Q-R-K_eff → N9 localization → N10/N11 planning → N12/N13 control → N14 safety → N16 end-to-end. Grafik memakai Time [s] relatif; data live ROS, bukan nilai estimasi DOCX.")
      : QStringLiteral("Tabel mengikuti kolom laporan. Hijau diisi otomatis dari topic/source yang tersedia; kuning harus diisi dari ground truth atau instrumen eksternal. Tidak ada nilai estimasi yang dipakai sebagai hasil aktual."));
    description->setWordWrap(true);
    description->setObjectName(QStringLiteral("pageDescription"));
    layout->addWidget(description);
    breadcrumb_ = new QLabel(subsystemTitle());
    breadcrumb_->setObjectName(QStringLiteral("breadcrumb"));
    breadcrumb_->setWordWrap(true);
    layout->addWidget(breadcrumb_);
    // Perception-only FINAL BAB IV acquisition guard.  The detailed tuning
    // catalog remains intact; this box appears only on F4.1--F4.4 and provides
    // configuration verification plus the extra ground-truth/trial tools needed
    // by the narrowed thesis Chapter IV.
    finalBox_ = new QGroupBox(QStringLiteral("FINAL BAB IV — Validasi & Akuisisi"));
    finalBox_->setObjectName(QStringLiteral("finalBab4Box"));
    auto *finalLayout = new QGridLayout(finalBox_);
    finalStatus_ = new QLabel(QStringLiteral("Verifikasi YAML = runtime sebelum merekam data final."));
    finalStatus_->setWordWrap(true);
    finalStatus_->setObjectName(QStringLiteral("finalBab4Status"));
    finalProgress_ = new QLabel(QStringLiteral("Progress: —"));
    finalProgress_->setObjectName(QStringLiteral("finalBab4Progress"));
    verifyFinal_ = new QPushButton(QStringLiteral("✓ Verifikasi Config Runtime"));
    prepareMasks_ = new QPushButton(QStringLiteral("Siapkan Topic Mask 4.3"));
    loadGtDrivable_ = new QPushButton(QStringLiteral("Load GT Drivable"));
    loadGtLane_ = new QPushButton(QStringLiteral("Load GT Lane"));
    captureIou_ = new QPushButton(QStringLiteral("Capture + Hitung IoU"));
    trialStart_ = new QPushButton(QStringLiteral("▶ Mulai Trial"));
    trialPass_ = new QPushButton(QStringLiteral("✓ PASS"));
    trialFail_ = new QPushButton(QStringLiteral("✕ FAIL"));
    maskStatus_ = new QLabel(QStringLiteral("Mask GT: belum dimuat"));
    maskStatus_->setWordWrap(true);
    finalLayout->addWidget(finalStatus_,0,0,1,4);
    finalLayout->addWidget(finalProgress_,1,0,1,2);
    finalLayout->addWidget(verifyFinal_,1,2,1,2);
    finalLayout->addWidget(prepareMasks_,2,0);
    finalLayout->addWidget(loadGtDrivable_,2,1);
    finalLayout->addWidget(loadGtLane_,2,2);
    finalLayout->addWidget(captureIou_,2,3);
    finalLayout->addWidget(maskStatus_,3,0,1,4);
    finalLayout->addWidget(trialStart_,4,0,1,2);
    finalLayout->addWidget(trialPass_,4,2);
    finalLayout->addWidget(trialFail_,4,3);
    finalBox_->setVisible(false);
    layout->addWidget(finalBox_);
    auto *selectorBox = new QGroupBox(QStringLiteral("Tabel & grafik aktif"));
    auto *selectorLayout = new QGridLayout(selectorBox);
    tableSelector_ = new NoWheelComboBox();
    graphSelector_ = new NoWheelComboBox();
    plotMode_ = new NoWheelComboBox();
    plotMode_->addItems({
      QStringLiteral("Grafik live per waktu"), QStringLiteral("Grafik ringkasan tabel")
    });
    selectorLayout->addWidget(new QLabel(QStringLiteral("Tabel")), 0, 0);
    selectorLayout->addWidget(tableSelector_, 0, 1);
    selectorLayout->addWidget(new QLabel(QStringLiteral("Grafik")), 0, 2);
    selectorLayout->addWidget(graphSelector_, 0, 3);
    selectorLayout->addWidget(new QLabel(QStringLiteral("Mode grafik")), 1, 0);
    selectorLayout->addWidget(plotMode_, 1, 1, 1, 3);
    tableCaption_ = new QLabel();
    tableCaption_->setWordWrap(true);
    tableCaption_->setObjectName(QStringLiteral("metricCard"));
    selectorLayout->addWidget(tableCaption_, 2, 0, 1, 4);
    layout->addWidget(selectorBox);
    // Toolbar kecil: rekam / ringkasan / simpan / bersihkan
    auto *buttons = new QHBoxLayout();
    record_ = new QPushButton(QStringLiteral("● Start CSV"));
    auto *snapshot = new QPushButton(QStringLiteral("Ambil Ringkasan"));
    auto *manual = new QPushButton(QStringLiteral("Tambah Baris"));
    auto *remove = new QPushButton(QStringLiteral("Hapus Baris"));
    auto *save = new QPushButton(QStringLiteral("Simpan CSV + PNG + Raw"));
    auto *clear = new QPushButton(QStringLiteral("Bersihkan"));
    buttons->addWidget(record_);
    buttons->addWidget(snapshot);
    buttons->addWidget(manual);
    buttons->addWidget(remove);
    buttons->addStretch();
    buttons->addWidget(save);
    buttons->addWidget(clear);
    layout->addLayout(buttons);
    // Multi-graph workspace (scrollable vertical stack of GraphCards)
    graphScroll_ = new QScrollArea();
    graphScroll_->setWidgetResizable(true);
    graphScroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphContainer_ = new QWidget();
    graphLayout_ = new QVBoxLayout(graphContainer_);
    graphLayout_->setContentsMargins(0, 0, 0, 0);
    graphLayout_->setSpacing(10);
    graphScroll_->setWidget(graphContainer_);
    layout->addWidget(graphScroll_, 1);
    // Table section (collapsible)
    tableBox_ = new QGroupBox(QStringLiteral("Tabel & Ringkasan"));
    auto *tableBoxLayout = new QVBoxLayout(tableBox_);
    table_ = new QTableWidget();
    table_->setAlternatingRowColors(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    table_->verticalHeader()->setVisible(false);
    tableBoxLayout->addWidget(table_);
    layout->addWidget(tableBox_);
    status_ = new QLabel(QStringLiteral("Siap. Isi varian/ground truth lalu mulai run."));
    status_->setWordWrap(true);
    layout->addWidget(status_);
    availability_ = new QLabel();
    availability_->setWordWrap(true);
    layout->addWidget(availability_);
    connect(tableSelector_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i){
      setTableIndex(i);
    });
    connect(graphSelector_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i){
      if (i >= 0 && i < graphCards_.size()) {
        graphScroll_->ensureWidgetVisible(graphCards_.at(i), 0, 24);
        graphCards_.at(i)->setFocus(Qt::OtherFocusReason);
      }
    });
    connect(plotMode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int){
      refreshGraphs();
    });
    connect(record_, &QPushButton::clicked, this, [this](){
      toggleRecording();
    });
    connect(snapshot, &QPushButton::clicked, this, [this](){
      appendSummaryRow();
    });
    connect(manual, &QPushButton::clicked, this, [this](){
      addManualRow();
    });
    connect(remove, &QPushButton::clicked, this, [this](){
      removeSelectedRows();
    });
    connect(save, &QPushButton::clicked, this, [this](){
      saveEvidence();
    });
    connect(clear, &QPushButton::clicked, this, [this](){
      clearCurrent();
    });
    connect(verifyFinal_, &QPushButton::clicked, this, [this](){ requestFinalRuntimeVerification(); });
    connect(prepareMasks_, &QPushButton::clicked, this, [this](){ prepareSegmentationMasks(); });
    connect(loadGtDrivable_, &QPushButton::clicked, this, [this](){ loadGroundTruthMask(true); });
    connect(loadGtLane_, &QPushButton::clicked, this, [this](){ loadGroundTruthMask(false); });
    connect(captureIou_, &QPushButton::clicked, this, [this](){ captureSegmentationIou(); });
    connect(trialStart_, &QPushButton::clicked, this, [this](){ startIntegrationTrial(); });
    connect(trialPass_, &QPushButton::clicked, this, [this](){ finishIntegrationTrial(true,false); });
    connect(trialFail_, &QPushButton::clicked, this, [this](){ finishIntegrationTrial(false,false); });
    if(ros_){
      connect(ros_, &RosBridge::parametersResult, this, [this](const QString&tag,bool ok,const QVariantMap&values){
        if(tag!=QStringLiteral("final_perception_verify:")+currentId_)return;
        handleFinalRuntimeVerification(ok,values);
      });
    }
    connect(table_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*){
      if(!loading_)saveTableState();
      refreshSummaryPlotIfNeeded();
    });
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this](){
      captureSample();
    });
    availabilityTimer_ = new QTimer(this);
    availabilityTimer_->setInterval(1000);
    connect(availabilityTimer_, &QTimer::timeout, this, [this](){
      updateAvailability();
    });
    availabilityTimer_->start();
    // Keep the acquisition view live even before a recording run starts.
    // captureSample() only appends to rawRows while recording, so this does not
    // contaminate the saved run buffer.
    liveElapsed_.start();
    timer_->start(200);
    if (!catalog_.isEmpty()) selectLeaf(catalog_.first().id);
  }
  // Called by the floating BAB IV menu: open a specific report leaf.
  void selectLeaf(const QString &id) {
    if (!leafIndex_.contains(id)) return;
    if (recording_) stopRecording(true);
    if (!currentId_.isEmpty()) saveTableState();
    currentId_ = id;
    currentTableIndex_ = 0;
    diagnosticPrevious_.clear();
    lastGraphSecond_ = -1;
    applyLeaf();
  }
  // Used by MainWindow to push the active leaf's parameter definition to the sidebar.
  const QVector<ExperimentParameterField> &parameterFields() const {
    static const QVector<ExperimentParameterField> empty;
    int idx = leafIndex_.value(currentId_, -1);
    return idx >= 0 ? catalog_[idx].parameterFields : empty;
  }
  QString currentLeafId() const {
    return currentId_;
  }
  // Read a run-identity / ground-truth field value from the active leaf panel.
  QVariant parameterValue(const QString &key) const {
    if (!paramPanel_) return {};
    return paramPanel_->value(key);
  }
  QString parameterText(const QString &key) const {
    if (!paramPanel_) return QString();
    return paramPanel_->text(key);
  }
  void setParameterPanel(ExperimentParameterPanel *panel) {
    if (paramPanel_ == panel) return;
    paramPanel_ = panel;
    if (panel) {
      connect(panel, &ExperimentParameterPanel::parameterEdited, this, [this](const QString &, const QVariant &) {
        // Run identity/ground truth is read live. For FINAL perception leaves,
        // immediately refresh the start interlock when the operator edits it.
        updateFinalStartAvailability();
      });
    }
  }
  void pushParameterPanel() {
    if (!paramPanel_) return;
    paramPanel_->buildFor(subsystem_, currentId_, parameterFields(), stores_);
  }
  private:
  QString subsystem_;
  TelemetryStore *telemetry_;
  ReportManager *reports_;
  QMap<QString, std::shared_ptr<YamlStore>> stores_;
  RosBridge *ros_{nullptr};
  QVector<ExperimentSpec> catalog_;
  QMap<QString, ExperimentSessionData> sessions_;
  QComboBox *plotMode_, *tableSelector_, *graphSelector_;
  QLabel *breadcrumb_, *tableCaption_, *availability_, *status_;
  QGroupBox *finalBox_{nullptr};
  QLabel *finalStatus_{nullptr}, *finalProgress_{nullptr}, *maskStatus_{nullptr};
  QPushButton *verifyFinal_{nullptr}, *prepareMasks_{nullptr}, *loadGtDrivable_{nullptr}, *loadGtLane_{nullptr}, *captureIou_{nullptr};
  QPushButton *trialStart_{nullptr}, *trialPass_{nullptr}, *trialFail_{nullptr};
  QPushButton *record_;
  QScrollArea *graphScroll_;
  QWidget *graphContainer_;
  QVBoxLayout *graphLayout_;
  QGroupBox *tableBox_;
  QVector<GraphCard*> graphCards_;
  QTableWidget *table_;
  QTimer *timer_, *availabilityTimer_;
  QElapsedTimer elapsed_;
  QElapsedTimer liveElapsed_;
  HostMetricsSampler hostMetrics_;
  // Previous covariance/time samples used only for GUI drift diagnostics.
  // This never feeds the EKF; it quantifies uncertainty growth for the operator.
  QMap<QString,QPair<double,double>> diagnosticPrevious_;
  qint64 lastGraphSecond_{-1};
  ExperimentParameterPanel *paramPanel_ = nullptr;
  GraphFullscreenDialog *fullscreenDialog_ = nullptr;
  int fullscreenGraphIndex_ = -1;
  bool recording_{
    false
  }, loading_{
    false
  };
  int activeRecordingSummaryRow_{-1};
  QElapsedTimer summaryUpdateClock_;
  QVariantMap verifiedRuntime_;
  QVariantMap finalExpectedRuntime_;
  QString verifiedLeafId_;
  bool finalConfigVerified_{false};
  quint64 lastRawEventSeq_{0}, lastObstacleEventSeq_{0}, lastPerformanceEventSeq_{0};
  QImage gtDrivableMask_, gtLaneMask_;
  QString gtDrivablePath_, gtLanePath_;
  bool trialActive_{false};
  double trialStartedSec_{0.0};
  int completedTrials_{0};
  QString currentId_;
  int currentTableIndex_{
    0
  };
  QMap<QString, int> leafIndex_;
  QString subsystemTitle() const {
    if (subsystem_ == QStringLiteral("navigation")) return QStringLiteral("NAVIGASI");
    if (subsystem_ == QStringLiteral("perception")) return QStringLiteral("PERSEPSI");
    return QStringLiteral("ESC / FOC");
  }
  const ExperimentSpec &spec() const {
    static ExperimentSpec empty;
    int idx = leafIndex_.value(currentId_, -1);
    return idx >= 0 ? catalog_[idx] : empty;
  }
  ExperimentSessionData &session() {
    return sessions_[currentId_];
  }
  QStringList currentColumns() const {
    return spec().tableColumns.value(currentTableIndex_);
  }
  QString currentTableCaption() const {
    return spec().tableNames.value(currentTableIndex_, QStringLiteral("Tabel 1"));
  }
  QString selectedGraphCaption() const {
    const ExperimentSpec &s = spec();
    const int index = graphSelector_ ? graphSelector_->currentIndex() : 0;
    return s.graphCaptions.value(index);
  }
  static bool numericText(QString text, double &value) {
    text = text.trimmed();
    text.replace(',', '.');
    const auto match = QRegularExpression(QStringLiteral("[-+]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][-+]?[0-9]+)?")).match(text);
    if (!match.hasMatch()) return false;
    bool ok = false;
    value = match.captured(0).toDouble(&ok);
    return ok && std::isfinite(value);
  }
  QVariant groundTruth(const QString &key) const {
    double value=0.0;
    return numericText(parameterText(key), value) ? QVariant(value) : QVariant();
  }
  bool isFinalPerception() const {
    return subsystem_ == QStringLiteral("perception") && currentId_.startsWith(QStringLiteral("F4."));
  }
  int integerParameter(const QString &key,int fallback) const {
    bool ok=false;
    const int value=parameterText(key).toInt(&ok);
    return ok?value:fallback;
  }
  double doubleParameter(const QString &key,double fallback) const {
    double value=0.0;
    return numericText(parameterText(key),value)?value:fallback;
  }
  void setFinalVerified(bool verified,const QString &message){
    finalConfigVerified_=verified;
    if(verified)verifiedLeafId_=currentId_;
    else if(verifiedLeafId_==currentId_)verifiedLeafId_.clear();
    if(finalStatus_){
      finalStatus_->setText(message);
      finalStatus_->setStyleSheet(verified?QStringLiteral("color:#46b36b;font-weight:800;"):
        QStringLiteral("color:#e7953f;font-weight:800;"));
    }
    updateFinalStartAvailability();
  }
  bool finalInputsReady(QString *reason=nullptr) const {
    if(!isFinalPerception())return true;
    auto fail=[&](const QString&m){if(reason)*reason=m;return false;};
    if(parameterText(QStringLiteral("variation")).trimmed().isEmpty())return fail(QStringLiteral("Isi Variasi."));
    if(currentId_==QStringLiteral("F4.1")||currentId_==QStringLiteral("F4.2")){
      if(!groundTruth(QStringLiteral("gt_x")).isValid()||!groundTruth(QStringLiteral("gt_y")).isValid())
        return fail(QStringLiteral("Isi GT Forward dan GT Lateral."));
      bool ok=false; parameterText(QStringLiteral("target_class_id")).toInt(&ok);
      if(!ok)return fail(QStringLiteral("Isi Target raw class ID agar detection rate target-specific."));
    }
    if(currentId_==QStringLiteral("F4.3")){
      if(parameterText(QStringLiteral("condition")).trimmed().isEmpty())return fail(QStringLiteral("Isi kondisi Terang/Berbayang/Redup."));
      if(gtDrivableMask_.isNull()||gtLaneMask_.isNull())return fail(QStringLiteral("Load GT Drivable dan GT Lane terlebih dahulu."));
    }
    if(currentId_==QStringLiteral("F4.4")&&parameterText(QStringLiteral("condition")).trimmed().isEmpty())
      return fail(QStringLiteral("Isi skenario jalur kosong / obstacle / near-field."));
    if(reason)reason->clear();
    return true;
  }
  void updateFinalStartAvailability(){
    if(!record_)return;
    if(!isFinalPerception()){
      record_->setEnabled(true);
      return;
    }
    QString reason;
    const bool inputs=finalInputsReady(&reason);
    record_->setEnabled(finalConfigVerified_&&inputs);
    if(finalProgress_){
      if(currentId_==QStringLiteral("F4.1"))
        finalProgress_->setText(QStringLiteral("Unique frame: %1 / %2").arg(session().rawRows.size()).arg(integerParameter(QStringLiteral("frame_target"),100)));
      else if(currentId_==QStringLiteral("F4.2"))
        finalProgress_->setText(QStringLiteral("Sampel posisi valid: %1 / %2").arg(session().rawRows.size()).arg(integerParameter(QStringLiteral("sample_target"),30)));
      else if(currentId_==QStringLiteral("F4.3")){
        int n=0; for(const QVariantMap&r:session().rawRows)if(r.value(QStringLiteral("row_type")).toString()==QStringLiteral("iou_frame"))++n;
        finalProgress_->setText(QStringLiteral("Frame IoU tersimpan: %1").arg(n));
      } else if(currentId_==QStringLiteral("F4.4"))
        finalProgress_->setText(QStringLiteral("Trial selesai: %1 / %2%3").arg(completedTrials_).arg(integerParameter(QStringLiteral("trial_target"),20)).arg(trialActive_?QStringLiteral(" • TRIAL AKTIF"):QString()));
    }
    if(finalConfigVerified_&&!inputs&&finalStatus_)finalStatus_->setText(QStringLiteral("Config runtime MATCH ✓ • ")+reason);
  }
  void updateFinalPerceptionUi(){
    const bool final=isFinalPerception();
    if(finalBox_)finalBox_->setVisible(final);
    if(!final){
      record_->setEnabled(true);
      return;
    }
    if(verifiedLeafId_!=currentId_){
      finalConfigVerified_=false;
      verifiedRuntime_.clear();
      finalExpectedRuntime_.clear();
      if(finalStatus_)finalStatus_->setText(QStringLiteral("FINAL BAB IV: verifikasi YAML = runtime sebelum merekam."));
    }
    const bool segmentation=currentId_==QStringLiteral("F4.3");
    const bool integration=currentId_==QStringLiteral("F4.4");
    for(QWidget*w:{static_cast<QWidget*>(prepareMasks_),static_cast<QWidget*>(loadGtDrivable_),static_cast<QWidget*>(loadGtLane_),static_cast<QWidget*>(captureIou_),static_cast<QWidget*>(maskStatus_)})if(w)w->setVisible(segmentation);
    for(QWidget*w:{static_cast<QWidget*>(trialStart_),static_cast<QWidget*>(trialPass_),static_cast<QWidget*>(trialFail_)})if(w)w->setVisible(integration);
    if(maskStatus_&&segmentation){
      const QString drv=gtDrivableMask_.isNull()?QStringLiteral("belum"):QFileInfo(gtDrivablePath_).fileName();
      const QString lane=gtLaneMask_.isNull()?QStringLiteral("belum"):QFileInfo(gtLanePath_).fileName();
      const bool haveDrv=!telemetry_->get(QStringLiteral("drivable_mask.image")).value<QImage>().isNull();
      const bool haveLane=!telemetry_->get(QStringLiteral("lane_mask.image")).value<QImage>().isNull();
      maskStatus_->setText(QStringLiteral("GT Drivable=%1 • GT Lane=%2 • topic prediksi: drivable=%3 lane=%4")
        .arg(drv,lane,haveDrv?QStringLiteral("OK"):QStringLiteral("MISSING"),haveLane?QStringLiteral("OK"):QStringLiteral("MISSING")));
    }
    updateFinalStartAvailability();
  }
  void requestFinalRuntimeVerification(){
    if(!isFinalPerception()||!ros_){
      setFinalVerified(false,QStringLiteral("ROS bridge belum tersedia untuk verifikasi runtime."));
      return;
    }
    auto store=stores_.value(QStringLiteral("perception"));
    if(!store){setFinalVerified(false,QStringLiteral("perception YAML tidak ditemukan."));return;}
    QStringList names;
    finalExpectedRuntime_.clear();
    for(const ExperimentParameterField&field:parameterFields()){
      if(field.yamlFileKey!=QStringLiteral("perception")||field.yamlPath.isEmpty())continue;
      const QString marker=QStringLiteral(".ros__parameters.");
      const int pos=field.yamlPath.indexOf(marker);
      if(pos<0)continue;
      const QString name=field.yamlPath.mid(pos+marker.size());
      if(name.isEmpty()||names.contains(name))continue;
      names<<name;
      finalExpectedRuntime_[name]=store->get(field.yamlPath);
    }
    if(names.isEmpty()){
      setFinalVerified(true,QStringLiteral("Tidak ada parameter runtime pada leaf ini; identitas run siap."));
      return;
    }
    finalStatus_->setText(QStringLiteral("VERIFY /perception: %1 parameter...").arg(names.size()));
    finalStatus_->setStyleSheet(QStringLiteral("color:#d8b033;font-weight:800;"));
    ros_->getParameters(QStringLiteral("/perception"),names,QStringLiteral("final_perception_verify:")+currentId_);
  }
  void handleFinalRuntimeVerification(bool ok,const QVariantMap&values){
    if(!isFinalPerception())return;
    verifiedRuntime_=values;
    if(!ok){setFinalVerified(false,QStringLiteral("Runtime /perception tidak dapat dibaca. Restart/launch perception lalu verifikasi lagi."));return;}
    QStringList mismatch;
    for(auto it=finalExpectedRuntime_.cbegin();it!=finalExpectedRuntime_.cend();++it){
      const QVariant got=values.value(it.key());
      if(!got.isValid()||!yamlValueSame(it.value(),got))
        mismatch<<QStringLiteral("%1 YAML=%2 runtime=%3").arg(it.key(),it.value().toString(),got.isValid()?got.toString():QStringLiteral("<unset>"));
    }
    if(mismatch.isEmpty())setFinalVerified(true,QStringLiteral("CONFIG FROZEN ✓ • YAML = /perception runtime (%1 parameter)").arg(finalExpectedRuntime_.size()));
    else setFinalVerified(false,QStringLiteral("RUNTIME MISMATCH — jangan rekam final:\n")+mismatch.join(QStringLiteral("\n")));
  }
  void prepareSegmentationMasks(){
    if(currentId_!=QStringLiteral("F4.3"))return;
    auto store=stores_.value(QStringLiteral("perception"));
    if(!store){QMessageBox::warning(this,QStringLiteral("Mask BAB IV"),QStringLiteral("perception YAML tidak ditemukan."));return;}
    QString error;
    const bool a=store->set(QStringLiteral("perception.ros__parameters.publish_drivable_mask"),true,&error);
    const bool b=store->set(QStringLiteral("perception.ros__parameters.publish_lane_mask"),true,&error);
    pushParameterPanel();
    setFinalVerified(false,QStringLiteral("Mask YAML disiapkan. WAJIB restart perception/autonomous.launch lalu klik Verifikasi Config Runtime."));
    QMessageBox::information(this,QStringLiteral("Mask BAB IV"),a&&b?
      QStringLiteral("publish_drivable_mask=true dan publish_lane_mask=true tersimpan. Restart node perception sebelum pengujian 4.3."):
      QStringLiteral("Gagal menyimpan parameter mask: ")+error);
  }
  void loadGroundTruthMask(bool drivable){
    const QString path=QFileDialog::getOpenFileName(this,drivable?QStringLiteral("Load Ground Truth Drivable"):QStringLiteral("Load Ground Truth Lane"),QString(),QStringLiteral("Image (*.png *.jpg *.jpeg *.bmp)"));
    if(path.isEmpty())return;
    QImage image(path);
    if(image.isNull()){QMessageBox::warning(this,QStringLiteral("Ground Truth"),QStringLiteral("File mask tidak dapat dibaca."));return;}
    image=image.convertToFormat(QImage::Format_Grayscale8);
    if(drivable){gtDrivableMask_=image;gtDrivablePath_=path;}
    else{gtLaneMask_=image;gtLanePath_=path;}
    updateFinalPerceptionUi();
  }
  static double binaryMaskIou(const QImage&prediction,const QImage&gt){
    if(prediction.isNull()||gt.isNull()||prediction.size()!=gt.size())return std::numeric_limits<double>::quiet_NaN();
    const QImage p=prediction.convertToFormat(QImage::Format_Grayscale8);
    const QImage g=gt.convertToFormat(QImage::Format_Grayscale8);
    quint64 intersection=0,uni=0;
    for(int y=0;y<p.height();++y){
      const uchar*pp=p.constScanLine(y); const uchar*gg=g.constScanLine(y);
      for(int x=0;x<p.width();++x){
        const bool a=pp[x]>127,b=gg[x]>127;
        if(a&&b)++intersection;
        if(a||b)++uni;
      }
    }
    return uni?double(intersection)/double(uni):std::numeric_limits<double>::quiet_NaN();
  }
  void captureSegmentationIou(){
    if(currentId_!=QStringLiteral("F4.3"))return;
    if(!finalConfigVerified_){QMessageBox::warning(this,QStringLiteral("IoU"),QStringLiteral("Verifikasi config runtime terlebih dahulu."));return;}
    QString reason; if(!finalInputsReady(&reason)){QMessageBox::warning(this,QStringLiteral("IoU"),reason);return;}
    const QImage predDrv=telemetry_->get(QStringLiteral("drivable_mask.image")).value<QImage>();
    const QImage predLane=telemetry_->get(QStringLiteral("lane_mask.image")).value<QImage>();
    if(predDrv.isNull()||predLane.isNull()){
      QMessageBox::warning(this,QStringLiteral("IoU"),QStringLiteral("Predicted mask belum tersedia. Pastikan topic /yolop/drivable_mask dan /yolop/lane_mask aktif setelah restart."));
      return;
    }
    if(predDrv.size()!=gtDrivableMask_.size()||predLane.size()!=gtLaneMask_.size()){
      QMessageBox::warning(this,QStringLiteral("IoU"),QStringLiteral("Ukuran GT harus sama persis dengan mask prediksi. Drivable pred=%1x%2 GT=%3x%4; Lane pred=%5x%6 GT=%7x%8")
        .arg(predDrv.width()).arg(predDrv.height()).arg(gtDrivableMask_.width()).arg(gtDrivableMask_.height())
        .arg(predLane.width()).arg(predLane.height()).arg(gtLaneMask_.width()).arg(gtLaneMask_.height()));
      return;
    }
    const double iouDrv=binaryMaskIou(predDrv,gtDrivableMask_);
    const double iouLane=binaryMaskIou(predLane,gtLaneMask_);
    QVariantMap row;
    row[QStringLiteral("row_type")]=QStringLiteral("iou_frame");
    row[QStringLiteral("time_iso")]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    row[QStringLiteral("variation")]=parameterText(QStringLiteral("variation"));
    row[QStringLiteral("condition")]=parameterText(QStringLiteral("condition"));
    row[QStringLiteral("derived.iou_drivable")]=iouDrv;
    row[QStringLiteral("derived.iou_lane")]=iouLane;
    row[QStringLiteral("lane_state.valid")]=telemetry_->get(QStringLiteral("lane_state.valid"));
    row[QStringLiteral("lane_state.center_error_m")]=telemetry_->get(QStringLiteral("lane_state.center_error_m"));
    row[QStringLiteral("lane_state.state")]=telemetry_->get(QStringLiteral("lane_state.state"));
    row[QStringLiteral("lane_state.confidence")]=telemetry_->get(QStringLiteral("lane_state.confidence"));
    row[QStringLiteral("lane_metrics.valid_rows")]=telemetry_->get(QStringLiteral("lane_metrics.valid_rows"));
    row[QStringLiteral("lane_metrics.confidence")]=telemetry_->get(QStringLiteral("lane_metrics.confidence"));
    session().rawRows<<row;
    status_->setText(QStringLiteral("IoU frame tersimpan: drivable=%1 lane=%2").arg(iouDrv,0,'f',4).arg(iouLane,0,'f',4));
    updateRecordingSummaryRow();
    updateFinalStartAvailability();
  }
  int targetMatches(const QVariantMap&raw,double*bestConfidence=nullptr) const {
    bool classOk=false; const int targetClass=parameterText(QStringLiteral("target_class_id")).toInt(&classOk);
    const QVariant gxVar=groundTruth(QStringLiteral("gt_x")),gyVar=groundTruth(QStringLiteral("gt_y"));
    const double gx=number(gxVar),gy=number(gyVar),tol=doubleParameter(QStringLiteral("target_match_radius_m"),0.5);
    int matches=0; double best=-1.0;
    const QVariantList detections=raw.value(QStringLiteral("detections")).toList();
    for(const QVariant&entry:detections){
      const QVariantMap d=entry.toMap();
      if(classOk&&d.value(QStringLiteral("class_id"),-9999).toInt()!=targetClass)continue;
      const double x=number(d.value(QStringLiteral("forward_m"))),y=number(d.value(QStringLiteral("left_m")));
      if(std::isfinite(gx)&&std::isfinite(gy)&&std::isfinite(x)&&std::isfinite(y)&&tol>0.0&&std::hypot(x-gx,y-gy)>tol)continue;
      ++matches; best=std::max(best,number(d.value(QStringLiteral("score")),-1.0));
    }
    if(detections.isEmpty()&&!classOk)matches=raw.value(QStringLiteral("count"),0).toInt();
    if(bestConfidence&&best>=0.0)*bestConfidence=best;
    return matches;
  }
  void appendLivePoint(const QVariantMap&row){
    const double time=number(row.value(QStringLiteral("elapsed_s")),recording_?elapsed_.elapsed()/1000.0:liveElapsed_.elapsed()/1000.0);
    for(auto it=spec().liveSeries.cbegin();it!=spec().liveSeries.cend();++it){
      const double value=number(row.value(it.value()));
      if(std::isfinite(value))session().liveSeries[it.key()]<<QPointF(time,value);
    }
  }
  void captureFinalDetectionEvents(){
    const QVector<TelemetryEvent>events=telemetry_->eventsSince(QStringLiteral("raw_detections"),lastRawEventSeq_);
    const int target=integerParameter(QStringLiteral("frame_target"),100);
    for(const TelemetryEvent&event:events){
      lastRawEventSeq_=event.sequence;
      if(session().rawRows.size()>=target)break;
      const QVariantMap raw=event.value.toMap();
      double targetConfidence=std::numeric_limits<double>::quiet_NaN();
      const int matches=targetMatches(raw,&targetConfidence);
      const int total=raw.value(QStringLiteral("count"),0).toInt();
      QVariantMap row;
      row[QStringLiteral("row_type")]=QStringLiteral("raw_detection_frame");
      row[QStringLiteral("frame_seq")]=QVariant::fromValue<qulonglong>(event.sequence);
      row[QStringLiteral("time_iso")]=QDateTime::fromMSecsSinceEpoch(qint64(event.received_sec*1000.0)).toString(Qt::ISODateWithMs);
      row[QStringLiteral("elapsed_s")]=elapsed_.elapsed()/1000.0;
      row[QStringLiteral("variation")]=parameterText(QStringLiteral("variation"));
      row[QStringLiteral("condition")]=parameterText(QStringLiteral("condition"));
      row[QStringLiteral("raw_detections.count")]=total;
      row[QStringLiteral("raw_detections.mean_confidence")]=raw.value(QStringLiteral("mean_confidence"));
      row[QStringLiteral("raw_detections.raw")]=raw.value(QStringLiteral("raw"));
      row[QStringLiteral("derived.target_matches")]=matches;
      row[QStringLiteral("derived.target_detected")]=matches>0;
      row[QStringLiteral("derived.target_confidence")]=std::isfinite(targetConfidence)?QVariant(targetConfidence):QVariant();
      row[QStringLiteral("derived.false_detection_count")]=std::max(0,total-matches);
      row[QStringLiteral("derived.missed_detection")]=matches==0;
      session().rawRows<<row;
      appendLivePoint(row);
    }
    updateFinalStartAvailability();
    if(recording_&&session().rawRows.size()>=target)stopRecording(false);
  }
  QVariantMap selectMetricTarget(const QVariantMap&metrics) const {
    bool classOk=false; const int targetClass=parameterText(QStringLiteral("target_class_id")).toInt(&classOk);
    const double gx=number(groundTruth(QStringLiteral("gt_x"))),gy=number(groundTruth(QStringLiteral("gt_y")));
    const double tol=doubleParameter(QStringLiteral("target_match_radius_m"),0.7);
    QVariantMap best; double bestDistance=std::numeric_limits<double>::infinity();
    for(const QVariant&entry:metrics.value(QStringLiteral("detections")).toList()){
      const QVariantMap d=entry.toMap();
      if(classOk&&d.value(QStringLiteral("class_id"),-9999).toInt()!=targetClass)continue;
      const double x=number(d.value(QStringLiteral("forward_m"))),y=number(d.value(QStringLiteral("left_m")));
      if(!std::isfinite(x)||!std::isfinite(y))continue;
      const double distance=(std::isfinite(gx)&&std::isfinite(gy))?std::hypot(x-gx,y-gy):x;
      if(distance<bestDistance){bestDistance=distance;best=d;}
    }
    if(!best.isEmpty()&&std::isfinite(gx)&&std::isfinite(gy)&&tol>0.0&&bestDistance>tol)return {};
    return best;
  }
  void captureFinalObstacleEvents(){
    const QVector<TelemetryEvent>events=telemetry_->eventsSince(QStringLiteral("obstacle_metrics"),lastObstacleEventSeq_);
    const int target=integerParameter(QStringLiteral("sample_target"),30);
    const double gx=number(groundTruth(QStringLiteral("gt_x"))),gy=number(groundTruth(QStringLiteral("gt_y")));
    for(const TelemetryEvent&event:events){
      lastObstacleEventSeq_=event.sequence;
      if(session().rawRows.size()>=target)break;
      const QVariantMap metrics=event.value.toMap();
      const QVariantMap d=selectMetricTarget(metrics);
      if(d.isEmpty())continue;
      const double tracked=number(d.value(QStringLiteral("forward_m")));
      const double homography=number(d.value(QStringLiteral("forward_homography_m")),tracked);
      const double left=number(d.value(QStringLiteral("left_m")));
      QVariantMap row;
      row[QStringLiteral("row_type")]=QStringLiteral("metric_position_sample");
      row[QStringLiteral("sample_seq")]=QVariant::fromValue<qulonglong>(event.sequence);
      row[QStringLiteral("time_iso")]=QDateTime::fromMSecsSinceEpoch(qint64(event.received_sec*1000.0)).toString(Qt::ISODateWithMs);
      row[QStringLiteral("elapsed_s")]=elapsed_.elapsed()/1000.0;
      row[QStringLiteral("variation")]=parameterText(QStringLiteral("variation"));
      row[QStringLiteral("condition")]=parameterText(QStringLiteral("condition"));
      row[QStringLiteral("gt_forward_m")]=gx; row[QStringLiteral("gt_left_m")]=gy;
      row[QStringLiteral("obstacle_metrics.nearest_forward_homography_m")]=homography;
      row[QStringLiteral("obstacle_metrics.nearest_forward_m")]=tracked;
      row[QStringLiteral("obstacle_metrics.nearest_left_m")]=left;
      row[QStringLiteral("obstacle_metrics.mean_confidence")]=d.value(QStringLiteral("score"));
      row[QStringLiteral("track_id")]=d.value(QStringLiteral("track_id"));
      row[QStringLiteral("class_id")]=d.value(QStringLiteral("class_id"));
      row[QStringLiteral("derived.obstacle_error_x_m")]=tracked-gx;
      row[QStringLiteral("derived.obstacle_abs_error_x_m")]=std::abs(tracked-gx);
      row[QStringLiteral("derived.homography_abs_error_x_m")]=std::abs(homography-gx);
      row[QStringLiteral("derived.obstacle_error_2d_m")]=std::hypot(tracked-gx,left-gy);
      session().rawRows<<row;
      appendLivePoint(row);
    }
    updateFinalStartAvailability();
    if(recording_&&session().rawRows.size()>=target)stopRecording(false);
  }
  void captureFinalPerformanceEvents(){
    const QVector<TelemetryEvent>events=telemetry_->eventsSince(QStringLiteral("perception_performance"),lastPerformanceEventSeq_);
    for(const TelemetryEvent&event:events){
      lastPerformanceEventSeq_=event.sequence;
      const QVariantMap perf=event.value.toMap();
      QVariantMap row;
      row[QStringLiteral("row_type")]=QStringLiteral("performance_window");
      row[QStringLiteral("event_seq")]=QVariant::fromValue<qulonglong>(event.sequence);
      row[QStringLiteral("time_iso")]=QDateTime::fromMSecsSinceEpoch(qint64(event.received_sec*1000.0)).toString(Qt::ISODateWithMs);
      row[QStringLiteral("elapsed_s")]=elapsed_.elapsed()/1000.0;
      row[QStringLiteral("variation")]=parameterText(QStringLiteral("variation"));
      row[QStringLiteral("condition")]=parameterText(QStringLiteral("condition"));
      for(const QString&key:{QStringLiteral("fps"),QStringLiteral("mean_ms"),QStringLiteral("p95_ms"),QStringLiteral("capture_dropped"),QStringLiteral("rviz_dropped"),QStringLiteral("pipeline_min_ms"),QStringLiteral("pipeline_max_ms"),QStringLiteral("pipeline_std_ms"),QStringLiteral("window_frames")})
        if(perf.contains(key))row[QStringLiteral("perception_performance.")+key]=perf.value(key);
      row[QStringLiteral("object_points.count")]=telemetry_->get(QStringLiteral("object_points.count"));
      row[QStringLiteral("path_relevant_points.count")]=telemetry_->get(QStringLiteral("path_relevant_points.count"));
      row[QStringLiteral("planning_relevant_points.count")]=telemetry_->get(QStringLiteral("planning_relevant_points.count"));
      row[QStringLiteral("trajectory_safety_state.decision")]=telemetry_->get(QStringLiteral("trajectory_safety_state.decision"));
      row[QStringLiteral("host.gpu_percent")]=hostMetrics_.value(QStringLiteral("host.gpu_percent"));
      row[QStringLiteral("host.ram_used_gb")]=hostMetrics_.value(QStringLiteral("host.ram_used_gb"));
      row[QStringLiteral("host.temperature_c")]=hostMetrics_.value(QStringLiteral("host.temperature_c"));
      session().rawRows<<row;
      appendLivePoint(row);
    }
  }
  bool expectedIntegrationDecisionReached() const {
    const QString scenario=parameterText(QStringLiteral("condition")).toLower();
    const QString decision=telemetry_->get(QStringLiteral("trajectory_safety_state.decision")).toString().toUpper();
    if(scenario.contains(QStringLiteral("sangat dekat"))||scenario.contains(QStringLiteral("near")))
      return telemetry_->get(QStringLiteral("perception_emergency"),false).toBool()||decision.contains(QStringLiteral("HARD"))||decision.contains(QStringLiteral("STOP"));
    if(scenario.contains(QStringLiteral("obstacle")))return decision.contains(QStringLiteral("SLOW"))||decision.contains(QStringLiteral("AVOID"));
    return false;
  }
  void startIntegrationTrial(){
    if(currentId_!=QStringLiteral("F4.4"))return;
    if(!recording_){QMessageBox::warning(this,QStringLiteral("Trial"),QStringLiteral("Klik Mulai Rekam Run terlebih dahulu."));return;}
    if(trialActive_){QMessageBox::information(this,QStringLiteral("Trial"),QStringLiteral("Trial masih aktif."));return;}
    if(completedTrials_>=integerParameter(QStringLiteral("trial_target"),20)){QMessageBox::information(this,QStringLiteral("Trial"),QStringLiteral("Target trial sudah terpenuhi."));return;}
    trialActive_=true;
    trialStartedSec_=QDateTime::currentMSecsSinceEpoch()/1000.0;
    status_->setText(QStringLiteral("TRIAL %1 aktif — ubah kondisi fisik sekarang. PASS otomatis saat decision yang diharapkan terdeteksi; jalur kosong gunakan tombol PASS setelah observasi.").arg(completedTrials_+1));
    updateFinalStartAvailability();
  }
  void finishIntegrationTrial(bool pass,bool automatic){
    if(currentId_!=QStringLiteral("F4.4")||!trialActive_){if(!automatic)QMessageBox::information(this,QStringLiteral("Trial"),QStringLiteral("Belum ada trial aktif."));return;}
    const double now=QDateTime::currentMSecsSinceEpoch()/1000.0;
    const QString scenario=parameterText(QStringLiteral("condition")).toLower();
    const bool responseApplicable=!scenario.contains(QStringLiteral("kosong"));
    QVariantMap row;
    row[QStringLiteral("row_type")]=QStringLiteral("trial");
    row[QStringLiteral("time_iso")]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    row[QStringLiteral("variation")]=parameterText(QStringLiteral("variation"));
    row[QStringLiteral("condition")]=parameterText(QStringLiteral("condition"));
    row[QStringLiteral("trial_index")]=completedTrials_+1;
    row[QStringLiteral("trial_success")]=pass;
    if(pass&&responseApplicable)row[QStringLiteral("derived.response_time_s")]=std::max(0.0,now-trialStartedSec_);
    row[QStringLiteral("object_points.count")]=telemetry_->get(QStringLiteral("object_points.count"));
    row[QStringLiteral("path_relevant_points.count")]=telemetry_->get(QStringLiteral("path_relevant_points.count"));
    row[QStringLiteral("planning_relevant_points.count")]=telemetry_->get(QStringLiteral("planning_relevant_points.count"));
    row[QStringLiteral("trajectory_safety_state.decision")]=telemetry_->get(QStringLiteral("trajectory_safety_state.decision"));
    row[QStringLiteral("perception_emergency")]=telemetry_->get(QStringLiteral("perception_emergency"));
    session().rawRows<<row;
    trialActive_=false; ++completedTrials_;
    status_->setText(QStringLiteral("Trial %1 = %2%3").arg(completedTrials_).arg(pass?QStringLiteral("PASS"):QStringLiteral("FAIL"),automatic?QStringLiteral(" (otomatis)"):QString()));
    updateRecordingSummaryRow();
    updateFinalStartAvailability();
  }
  void tryAutoCompleteIntegrationTrial(){
    if(trialActive_&&expectedIntegrationDecisionReached())finishIntegrationTrial(true,true);
  }
  #if defined(__GNUC__) || defined(__clang__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wmisleading-indentation"
  #endif
  void setTableIndex(int i) {
    if (i < 0 || i >= spec().tableColumns.size()) return;
    currentTableIndex_ = i;
    applyLeaf();
  }
  void rebuildGraphCards() {
    // Clear every prior renderer/placeholder. 4.10 intentionally has zero
    // graphs, so stale cards from the previous leaf must never remain visible.
    while (QLayoutItem *item = graphLayout_->takeAt(0)) {
      if (QWidget *widget = item->widget()) widget->deleteLater();
      delete item;
    }
    graphCards_.clear();
    const ExperimentSpec &s = spec();
    for (int i = 0; i < s.graphCaptions.size(); ++i) {
      GraphCard *card = new GraphCard(s.graphCaptions.at(i));
      card->setFocusPolicy(Qt::StrongFocus);
      connect(card, &GraphCard::maximizeRequested, this, [this, i]() {
        openFullscreenGraph(i);
      });
      graphLayout_->addWidget(card);
      graphCards_ << card;
    }
    if (graphCards_.isEmpty()) {
      auto *placeholder = new QLabel(QStringLiteral("Subbab ini memang tidak memiliki grafik (0 grafik)."));
      placeholder->setObjectName(QStringLiteral("metricCard"));
      placeholder->setWordWrap(true);
      graphLayout_->addWidget(placeholder);
    }
    graphLayout_->addStretch(1);
    graphScroll_->setVisible(!graphCards_.isEmpty());
  }
  void applyLeaf() {
    if (catalog_.isEmpty() || currentId_.isEmpty()) return;
    const ExperimentSpec &s = spec();
    breadcrumb_->setText(subsystemTitle() + QStringLiteral("  >  ") + s.groupId + QStringLiteral("  >  ") + s.section);
    tableCaption_->setText(QStringLiteral("TABEL: ") + currentTableCaption() +
      QStringLiteral("  •  %1 tabel  •  %2 grafik").arg(s.tableColumns.size()).arg(s.graphCaptions.size()));
    {
      const QSignalBlocker blocker(tableSelector_);
      tableSelector_->clear();
      tableSelector_->addItems(s.tableNames);
      tableSelector_->setEnabled(s.tableNames.size() > 1);
      tableSelector_->setCurrentIndex(currentTableIndex_);
    }
    {
      const QSignalBlocker blocker(graphSelector_);
      graphSelector_->clear();
      if (s.graphCaptions.isEmpty()) graphSelector_->addItem(QStringLiteral("0 grafik"));
      else {
        for (int i = 0; i < s.graphCaptions.size(); ++i)
          graphSelector_->addItem(QStringLiteral("Grafik %1 — %2").arg(i + 1).arg(s.graphCaptions.at(i)));
      }
      graphSelector_->setEnabled(!s.graphCaptions.isEmpty());
      graphSelector_->setCurrentIndex(0);
    }
    rebuildGraphCards();
    loading_ = true;
    const QStringList cols = currentColumns();
    table_->clear();
    table_->setColumnCount(cols.size());
    table_->setHorizontalHeaderLabels(cols);
    table_->setRowCount(0);
    for (const QVariantMap &row : session().summaryRows) insertRow(row);
    table_->resizeColumnsToContents();
    loading_ = false;
    refreshGraphs();
    pushParameterPanel();
    updateFinalPerceptionUi();
    updateAvailability();
  }
  QStringList commonPaths() const {
    if (subsystem_ == QStringLiteral("navigation")) return {
      "gnss_quality.sat","gnss_quality.dop","gnss_quality.hacc_m","gnss_fix.lat","gnss_fix.lon",
      "imu.gx","imu.gy","imu.gz","imu.yaw_rad","imu_status.yaw_residual",
      "esc_odom.x","esc_odom.y","esc_odom.v","esc_odom.w","ekf_local.x","ekf_local.y","ekf_local.v","ekf_local.w",
      "ekf_global.x","ekf_global.y","ekf_global.yaw","localization_state.map_x","localization_state.map_y","localization_state.yaw",
      "cmd_nav.linear_x","cmd_nav.angular_z","cmd_autonomy_integrated.linear_x","cmd_autonomy_integrated.angular_z",
      "cmd_final.linear_x","cmd_final.angular_z","esc_drive_target","esc_drive_actual","esc_steer_target","esc_steer_actual",
      "mppi_velocity_error","mppi_steering_error","mppi_yaw_error","nav_path.length_m","nav_path.planning_latency_ms",
      "goal_state.duration_s","derived.cte_m","derived.path_heading_error_rad","derived.endpoint_error_m","derived.goal_yaw_error_rad",
      "derived.velocity_error_mps","derived.steering_error_rad","derived.yaw_error_rps","host.cpu_percent"
    };
    if (subsystem_ == QStringLiteral("perception")) return {
      "perception_performance.fps","perception_performance.mean_ms","perception_performance.p95_ms","perception_performance.capture_dropped",
      "perception_performance.rviz_dropped","raw_detections.count","raw_detections.mean_confidence","obstacle_metrics.count",
      "obstacle_metrics.nearest_forward_m","obstacle_metrics.nearest_left_m","obstacle_metrics.mean_confidence",
      "object_points.count","path_relevant_points.count","planning_relevant_points.count","drivable_boundary_points.count",
      "drivable_space.valid_rows","drivable_space.valid","lane_metrics.valid_rows","lane_metrics.confidence",
      "lane_state.valid","lane_state.left_clearance_m","lane_state.right_clearance_m",
      "lane_state.center_error_m","lane_state.heading_error_rad","lane_state.confidence","lane_state.state","camera_healthy","camera_health_state.mean_luma",
      "camera_health_state.stddev_luma","camera_health_state.mean_gradient","perception_emergency","near_field_state.confidence",
      "near_field_state.near_field_drivable_fraction","trajectory_safety_state.speed_scale","trajectory_safety_state.decision",
      "derived.obstacle_error_x_m","derived.obstacle_error_y_m","derived.obstacle_error_2d_m","host.gpu_percent","host.ram_used_gb","host.temperature_c"
    };
    return {
      "esc_steer_target","esc_steer_actual","esc_drive_target","esc_drive_actual","esc_yaw_rate","esc_kinematic_yaw_rate",
      "derived.steering_error_rad","derived.steering_target_deg","derived.steering_actual_deg","derived.target_rpm","derived.actual_rpm",
      "foc_telemetry.ia_a","foc_telemetry.ib_a","foc_telemetry.id_a","foc_telemetry.iq_a","foc_telemetry.iq_ref_a",
      "foc_telemetry.vd_v","foc_telemetry.vq_v","foc_telemetry.vbus_v","foc_telemetry.encoder_count","foc_telemetry.electrical_sector"
    };
  }
  QVariant instantValue(const QString &path) {
    if (path.startsWith(QStringLiteral("host."))) return hostMetrics_.value(path);
    if (!path.startsWith(QStringLiteral("derived."))) return telemetry_->get(path);
    if(path==QStringLiteral("derived.rate_gnss_hz"))return telemetry_->rate(QStringLiteral("gnss_fix"));
    if(path==QStringLiteral("derived.rate_imu_hz"))return telemetry_->rate(QStringLiteral("imu"));
    if(path==QStringLiteral("derived.rate_esc_hz"))return telemetry_->rate(QStringLiteral("esc_odom"));
    if(path==QStringLiteral("derived.rate_ekf_local_hz"))return telemetry_->rate(QStringLiteral("ekf_local"));
    if(path==QStringLiteral("derived.rate_ekf_global_hz"))return telemetry_->rate(QStringLiteral("ekf_global"));
    if(path==QStringLiteral("derived.dt_gnss_s"))return telemetry_->interval(QStringLiteral("gnss_fix"));
    if(path==QStringLiteral("derived.dt_imu_s"))return telemetry_->interval(QStringLiteral("imu"));
    if(path==QStringLiteral("derived.dt_esc_s"))return telemetry_->interval(QStringLiteral("esc_odom"));
    if(path==QStringLiteral("derived.age_gnss_s"))return telemetry_->age(QStringLiteral("gnss_fix"));
    if(path==QStringLiteral("derived.age_imu_s"))return telemetry_->age(QStringLiteral("imu"));
    if(path==QStringLiteral("derived.age_esc_s"))return telemetry_->age(QStringLiteral("esc_odom"));
    if(path==QStringLiteral("derived.age_ekf_local_s"))return telemetry_->age(QStringLiteral("ekf_local"));
    if(path==QStringLiteral("derived.age_ekf_global_s"))return telemetry_->age(QStringLiteral("ekf_global"));
    if (path == QStringLiteral("derived.velocity_error_mps")) {
      const double target=number(telemetry_->get("esc_drive_target")), actual=number(telemetry_->get("esc_drive_actual"));
      return std::isfinite(target)&&std::isfinite(actual)?QVariant(target-actual):QVariant();
    }
    if (path == QStringLiteral("derived.steering_error_rad")) {
      const double target=number(telemetry_->get("esc_steer_target")), actual=number(telemetry_->get("esc_steer_actual"));
      return std::isfinite(target)&&std::isfinite(actual)?QVariant(normalizeAngle(target-actual)):QVariant();
    }
    if (path == QStringLiteral("derived.yaw_error_rps")) {
      const double direct=number(telemetry_->get("mppi_yaw_error"));
      if(std::isfinite(direct))return direct;
      const double target=number(telemetry_->get("cmd_final.angular_z")), actual=number(telemetry_->get("esc_yaw_rate"));
      return std::isfinite(target)&&std::isfinite(actual)?QVariant(target-actual):QVariant();
    }
    if (path == QStringLiteral("derived.steering_target_deg")) {
      const double v=number(telemetry_->get("esc_steer_target"));
      return std::isfinite(v)?QVariant(v*180.0/kPi):QVariant();
    }
    if (path == QStringLiteral("derived.steering_actual_deg")) {
      const double v=number(telemetry_->get("esc_steer_actual"));
      return std::isfinite(v)?QVariant(v*180.0/kPi):QVariant();
    }
    if (path == QStringLiteral("derived.target_rpm") || path == QStringLiteral("derived.actual_rpm")) {
      const QString statusKey = path.endsWith(QStringLiteral("target_rpm"))
      ? QStringLiteral("esc_status.right_target") : QStringLiteral("esc_status.right");
      const double measuredRpm = number(telemetry_->get(statusKey));
      if (std::isfinite(measuredRpm)) return measuredRpm;
      auto store=stores_.value("esc");
      if(!store)return {
      };
      const double maxRpm=number(store->get("esc_ackermann.ros__parameters.right_max_rpm",300.0),300.0);
      const double maxSpeed=number(store->get("esc_ackermann.ros__parameters.speed_max_mps",1.0),1.0);
      const double speed=number(telemetry_->get(path.endsWith("target_rpm")?"esc_drive_target":"esc_drive_actual"));
      return std::isfinite(speed)&&maxSpeed>0.0?QVariant(speed/maxSpeed*maxRpm):QVariant();
    }
    if (path.startsWith(QStringLiteral("derived.obstacle_error_"))) {
      const double gx=number(groundTruth(QStringLiteral("gt_x"))), gy=number(groundTruth(QStringLiteral("gt_y")));
      const double sx=number(telemetry_->get("obstacle_metrics.nearest_forward_m")), sy=number(telemetry_->get("obstacle_metrics.nearest_left_m"));
      if(!std::isfinite(gx)||!std::isfinite(gy)||!std::isfinite(sx)||!std::isfinite(sy))return {
      };
      if(path.endsWith("x_m"))return sx-gx;
      if(path.endsWith("y_m"))return sy-gy;
      return std::hypot(sx-gx,sy-gy);
    }
    if (path.startsWith(QStringLiteral("derived.ekf_"))) return ekfDiagnosticValue(path);
    return navigationDerived(path);
  }
  QVariant ekfDiagnosticValue(const QString &path) {
    auto v=[this](const QString&k){return number(telemetry_->get(k));};
    auto positive=[](double x){return std::isfinite(x)&&x>=0.0;};
    auto sigma=[&](double p)->QVariant{return positive(p)?QVariant(std::sqrt(p)):QVariant();};
    auto gain=[&](double p,double r)->QVariant{
      if(!positive(p)||!positive(r)||p+r<=1e-15)return {};
      // Diagnostic authority proxy only. robot_localization does not publish
      // its internal prior covariance/K matrix through the public ROS API.
      return QVariant(std::clamp(p/(p+r),0.0,1.0));
    };
    auto residual=[](double measurement,double estimate,bool angular=false)->QVariant{
      if(!std::isfinite(measurement)||!std::isfinite(estimate))return {};
      return angular?QVariant(normalizeAngle(measurement-estimate)):QVariant(measurement-estimate);
    };
    auto nis=[&](double res,double p,double r)->QVariant{
      if(!std::isfinite(res)||!positive(p)||!positive(r)||p+r<=1e-15)return {};
      return QVariant((res*res)/(p+r));
    };
    auto growth=[&](const QString&key,double p)->QVariant{
      if(!positive(p))return {};
      const double t=(recording_?elapsed_.elapsed():liveElapsed_.elapsed())/1000.0;
      const auto prev=diagnosticPrevious_.value(key,QPair<double,double>(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()));
      diagnosticPrevious_[key]=qMakePair(t,p);
      if(!std::isfinite(prev.first)||!std::isfinite(prev.second)||t-prev.first<=1e-4)return {};
      return QVariant((p-prev.second)/(t-prev.first));
    };

    const bool local=path.startsWith(QStringLiteral("derived.ekf_local_"));
    const QString prefix=local?QStringLiteral("ekf_local."):QStringLiteral("ekf_global.");
    const QString tail=path.mid(local?QStringLiteral("derived.ekf_local_").size():QStringLiteral("derived.ekf_global_").size());
    const double px=v(prefix+QStringLiteral("var_x")),py=v(prefix+QStringLiteral("var_y")),pyaw=v(prefix+QStringLiteral("var_yaw"));
    const double pv=v(prefix+QStringLiteral("var_v")),pw=v(prefix+QStringLiteral("var_w"));
    if(tail==QStringLiteral("sigma_x"))return sigma(px);
    if(tail==QStringLiteral("sigma_y"))return sigma(py);
    if(tail==QStringLiteral("sigma_yaw"))return sigma(pyaw);
    if(tail==QStringLiteral("sigma_vx"))return sigma(pv);
    if(tail==QStringLiteral("sigma_w"))return sigma(pw);
    if(tail==QStringLiteral("p_growth_x"))return growth(prefix+QStringLiteral("x"),px);
    if(tail==QStringLiteral("p_growth_y"))return growth(prefix+QStringLiteral("y"),py);
    if(tail==QStringLiteral("p_growth_yaw"))return growth(prefix+QStringLiteral("yaw"),pyaw);
    if(tail==QStringLiteral("p_growth_vx"))return growth(prefix+QStringLiteral("vx"),pv);
    if(tail==QStringLiteral("p_growth_w"))return growth(prefix+QStringLiteral("w"),pw);

    if(local){
      const double rEscV=v(QStringLiteral("esc_odom.var_v"));
      const double rEscW=v(QStringLiteral("esc_odom.var_w"));
      const double rImuYaw=v(QStringLiteral("imu.var_yaw"));
      const double rImuW=v(QStringLiteral("imu.var_gz"));
      const double rGnssV=v(QStringLiteral("gnss_base_vel_fusion.cov_x"));
      const double eV=v(QStringLiteral("ekf_local.v")),eW=v(QStringLiteral("ekf_local.w")),eYaw=v(QStringLiteral("ekf_local.yaw"));
      const double mEscV=v(QStringLiteral("esc_odom.v")),mEscW=v(QStringLiteral("esc_odom.w"));
      const double mGnssV=v(QStringLiteral("gnss_base_vel_fusion.vx")),mImuW=v(QStringLiteral("imu.gz")),mImuYaw=v(QStringLiteral("imu.yaw_rad"));
      if(tail==QStringLiteral("k_vx_esc"))return gain(pv,rEscV);
      if(tail==QStringLiteral("k_vx_gnss"))return gain(pv,rGnssV);
      if(tail==QStringLiteral("k_yaw_imu"))return gain(pyaw,rImuYaw);
      if(tail==QStringLiteral("k_w_esc"))return gain(pw,rEscW);
      if(tail==QStringLiteral("k_w_imu"))return gain(pw,rImuW);
      const QVariant rvEsc=residual(mEscV,eV),rvGnss=residual(mGnssV,eV),rwEsc=residual(mEscW,eW),rwImu=residual(mImuW,eW),ryaw=residual(mImuYaw,eYaw,true);
      if(tail==QStringLiteral("res_vx_esc"))return rvEsc;
      if(tail==QStringLiteral("res_vx_gnss"))return rvGnss;
      if(tail==QStringLiteral("res_yaw_imu"))return ryaw;
      if(tail==QStringLiteral("res_w_esc"))return rwEsc;
      if(tail==QStringLiteral("res_w_imu"))return rwImu;
      if(tail==QStringLiteral("nis_vx_esc"))return nis(number(rvEsc),pv,rEscV);
      if(tail==QStringLiteral("nis_vx_gnss"))return nis(number(rvGnss),pv,rGnssV);
      if(tail==QStringLiteral("nis_yaw_imu"))return nis(number(ryaw),pyaw,rImuYaw);
      if(tail==QStringLiteral("nis_w_esc"))return nis(number(rwEsc),pw,rEscW);
      if(tail==QStringLiteral("nis_w_imu"))return nis(number(rwImu),pw,rImuW);
    }else{
      const double rX=v(QStringLiteral("gnss_map_odom.var_x")),rY=v(QStringLiteral("gnss_map_odom.var_y"));
      const double rV=v(QStringLiteral("gnss_base_vel_fusion.cov_x"));
      const double rCog=v(QStringLiteral("gnss_cog_fusion.yaw_variance"));
      const double rImuW=v(QStringLiteral("imu.var_gz"));
      const double ex=v(QStringLiteral("ekf_global.x")),ey=v(QStringLiteral("ekf_global.y")),ev=v(QStringLiteral("ekf_global.v")),ew=v(QStringLiteral("ekf_global.w")),eyaw=v(QStringLiteral("ekf_global.yaw"));
      const double mx=v(QStringLiteral("gnss_map_odom.x")),my=v(QStringLiteral("gnss_map_odom.y")),mv=v(QStringLiteral("gnss_base_vel_fusion.vx")),mcog=v(QStringLiteral("gnss_cog_fusion.yaw_rad")),miw=v(QStringLiteral("imu.gz"));
      if(tail==QStringLiteral("k_x_gnss"))return gain(px,rX);
      if(tail==QStringLiteral("k_y_gnss"))return gain(py,rY);
      if(tail==QStringLiteral("k_vx_gnss"))return gain(pv,rV);
      if(tail==QStringLiteral("k_yaw_cog"))return gain(pyaw,rCog);
      if(tail==QStringLiteral("k_w_imu"))return gain(pw,rImuW);
      const QVariant rx=residual(mx,ex),ry=residual(my,ey),rv=residual(mv,ev),rcog=residual(mcog,eyaw,true),rw=residual(miw,ew);
      if(tail==QStringLiteral("res_x_gnss"))return rx;
      if(tail==QStringLiteral("res_y_gnss"))return ry;
      if(tail==QStringLiteral("res_vx_gnss"))return rv;
      if(tail==QStringLiteral("res_yaw_cog"))return rcog;
      if(tail==QStringLiteral("res_w_imu"))return rw;
      if(tail==QStringLiteral("nis_x_gnss"))return nis(number(rx),px,rX);
      if(tail==QStringLiteral("nis_y_gnss"))return nis(number(ry),py,rY);
      if(tail==QStringLiteral("nis_vx_gnss"))return nis(number(rv),pv,rV);
      if(tail==QStringLiteral("nis_yaw_cog"))return nis(number(rcog),pyaw,rCog);
      if(tail==QStringLiteral("nis_w_imu"))return nis(number(rw),pw,rImuW);
    }
    return {};
  }
  QVariant navigationDerived(const QString &path) const {
    const double x=number(telemetry_->get("localization_state.map_x"),number(telemetry_->get("ekf_global.x")));
    const double y=number(telemetry_->get("localization_state.map_y"),number(telemetry_->get("ekf_global.y")));
    const double yaw=number(telemetry_->get("localization_state.yaw"),number(telemetry_->get("ekf_global.yaw")));
    if(path==QStringLiteral("derived.endpoint_error_m")){
      const double gx=number(telemetry_->get("goal_pose.x")),gy=number(telemetry_->get("goal_pose.y"));
      return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(gx)&&std::isfinite(gy)?QVariant(std::hypot(x-gx,y-gy)):QVariant();
    }
    if(path==QStringLiteral("derived.goal_yaw_error_rad")){
      const double gyaw=number(telemetry_->get("goal_pose.yaw"));
      return std::isfinite(yaw)&&std::isfinite(gyaw)?QVariant(std::abs(normalizeAngle(yaw-gyaw))):QVariant();
    }
    const QVariantList points=telemetry_->get("nav_path.points").toList();
    if(!std::isfinite(x)||!std::isfinite(y)||points.size()<2)return {
    };
    double best=std::numeric_limits<double>::infinity(),bestHeading=0.0;
    for(int i=1;
    i<points.size();
    ++i){
      const QVariantList a=points[i-1].toList(),b=points[i].toList();
      if(a.size()<2||b.size()<2)continue;
      const double ax=a[0].toDouble(),ay=a[1].toDouble(),bx=b[0].toDouble(),by=b[1].toDouble();
      const double dx=bx-ax,dy=by-ay,l2=dx*dx+dy*dy;
      if(l2<1e-12)continue;
      const double u=std::clamp(((x-ax)*dx+(y-ay)*dy)/l2,0.0,1.0);
      const double distance=std::hypot(x-(ax+u*dx),y-(ay+u*dy));
      if(distance<best){
        best=distance;
        bestHeading=std::atan2(dy,dx);
      }
    }
    if(path==QStringLiteral("derived.cte_m"))return std::isfinite(best)?QVariant(best):QVariant();
    if(path==QStringLiteral("derived.path_heading_error_rad"))return std::isfinite(yaw)?QVariant(std::abs(normalizeAngle(yaw-bestHeading))):QVariant();
    return {
    };
  }
  void toggleRecording() {
    if(recording_)stopRecording(false);
    else startRecording();
  }
  void startRecording() {
    if(isFinalPerception()){
      QString reason;
      if(!finalConfigVerified_||!finalInputsReady(&reason)){
        QMessageBox::warning(this,QStringLiteral("FINAL BAB IV"),!finalConfigVerified_?
          QStringLiteral("Config runtime belum terverifikasi. Klik Verifikasi Config Runtime."):reason);
        updateFinalStartAvailability();
        return;
      }
    }
    saveTableState();
    session().rawRows.clear();
    session().liveSeries.clear();
    session().liveScatter.clear();
    session().scatterRawOrigin.clear();
    diagnosticPrevious_.clear();
    lastGraphSecond_ = -1;
    elapsed_.restart();
    activeRecordingSummaryRow_ = -1;
    trialActive_=false;
    completedTrials_=0;
    lastRawEventSeq_=telemetry_->sequence(QStringLiteral("raw_detections"));
    lastObstacleEventSeq_=telemetry_->sequence(QStringLiteral("obstacle_metrics"));
    lastPerformanceEventSeq_=telemetry_->sequence(QStringLiteral("perception_performance"));
    recording_=true;
    double rate=5.0;
    if(stores_.contains("gui")) rate=number(stores_["gui"]->get("reporting.sample_rate_hz",5.0),5.0);
    const double leafRate=number(parameterValue(QStringLiteral("sample_rate")));
    if(std::isfinite(leafRate)) rate=leafRate;
    rate=std::clamp(rate,0.5,50.0);
    // FINAL F4.1/F4.2 drains topic-event history, therefore the timer only needs
    // to wake often enough to consume the queue; acquisition rate is the ROS
    // message rate, not this GUI timer.
    const int interval=isFinalPerception()?50:std::max(20,int(std::lround(1000.0/rate)));
    timer_->start(interval);
    record_->setText(QStringLiteral("■ Stop CSV + Auto Save"));
    status_->setText(isFinalPerception()?
      QStringLiteral("MEREKAM FINAL ")+spec().id+QStringLiteral(" • event-based ROS acquisition"):
      QStringLiteral("MEREKAM run ")+spec().id+QStringLiteral(" @ ")+QString::number(rate,'f',1)+QStringLiteral(" Hz"));
    updateFinalStartAvailability();
  }
  void stopRecording(bool switching) {
    recording_=false;
    trialActive_=false;
    record_->setText(QStringLiteral("● Start CSV"));
    const int rawCount=session().rawRows.size();
    if(!switching && rawCount>0){
      updateRecordingSummaryRow();
      activeRecordingSummaryRow_ = -1;
      saveEvidence(false);
      status_->setText(QStringLiteral("CSV AUTO-SAVED ✓ • %1 sampel • %2")
        .arg(rawCount).arg(reports_->root()));
    } else {
      activeRecordingSummaryRow_ = -1;
      status_->setText(switching
        ? QStringLiteral("Run dihentikan karena pindah subbab; autosave hanya dilakukan saat Stop CSV.")
        : QStringLiteral("Run berhenti tanpa sampel; tidak ada CSV yang dibuat."));
    }
    updateFinalStartAvailability();
  }
  void captureSample() {
    if(recording_&&currentId_==QStringLiteral("F4.1")){
      captureFinalDetectionEvents();
      if(plotMode_->currentIndex()==0)refreshGraphs();
      return;
    }
    if(recording_&&currentId_==QStringLiteral("F4.2")){
      captureFinalObstacleEvents();
      if(plotMode_->currentIndex()==0)refreshGraphs();
      return;
    }
    if(recording_&&currentId_==QStringLiteral("F4.4")){
      captureFinalPerformanceEvents();
      tryAutoCompleteIntegrationTrial();
      if(!summaryUpdateClock_.isValid()||summaryUpdateClock_.elapsed()>=500){
        updateRecordingSummaryRow();
        summaryUpdateClock_.restart();
      }
      updateFinalStartAvailability();
      if(plotMode_->currentIndex()==0)refreshGraphs();
      return;
    }
    QVariantMap row;
    row["time_iso"]=QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    row["elapsed_s"]=(recording_ ? elapsed_.elapsed() : liveElapsed_.elapsed())/1000.0;
    row["variation"]=parameterText(QStringLiteral("variation"));
    row["condition"]=parameterText(QStringLiteral("condition"));
    QStringList paths=commonPaths();
    for(auto it=spec().liveSeries.cbegin();
    it!=spec().liveSeries.cend();
    ++it)if(!paths.contains(it.value()))paths<<it.value();
    for(const ExperimentGraphSpec&g:spec().graphs){
      if(g.type!=QStringLiteral("scatter"))continue;
      if(!g.xSeries.isEmpty()&&!paths.contains(g.xSeries))paths<<g.xSeries;
      if(!g.ySeries.isEmpty()&&!paths.contains(g.ySeries))paths<<g.ySeries;
    }
    for(const QString&path:paths){
      QVariant value=instantValue(path);
      if(value.isValid())row[path]=value;
    }
    if(recording_) session().rawRows<<row;
    const double time=row["elapsed_s"].toDouble();
    const qint64 graphSecond = std::max<qint64>(0, static_cast<qint64>(std::floor(time + 1.0e-9)));
    const bool graphTick = graphSecond != lastGraphSecond_;
    if (graphTick) {
      lastGraphSecond_ = graphSecond;
      const double graphTime = static_cast<double>(graphSecond);
      for(auto it=spec().liveSeries.cbegin(); it!=spec().liveSeries.cend(); ++it){
        const double value=number(row.value(it.value()));
        if(std::isfinite(value))session().liveSeries[it.key()]<<QPointF(graphTime,value);
      }
      // Plot evidence is intentionally 1 Hz. Raw CSV may retain the configured
      // acquisition rate, but every time-axis graph uses exact integer seconds.
      for(auto it=spec().liveSeries.cbegin(); it!=spec().liveSeries.cend(); ++it){
        auto &series=session().liveSeries[it.key()];
        while(series.size()>7200)series.removeFirst();
      }
      for (int gi = 0; gi < spec().graphs.size(); ++gi) {
        const ExperimentGraphSpec &g = spec().graphs.at(gi);
        if (g.type != QStringLiteral("scatter")) continue;
        double x = number(row.value(g.xSeries));
        double y = number(row.value(g.ySeries));
        if (std::isfinite(x) && std::isfinite(y)) {
          if (g.xSeries == QStringLiteral("gnss_fix.lon") && g.ySeries == QStringLiteral("gnss_fix.lat")) {
            if (!session().scatterRawOrigin.contains(gi)) session().scatterRawOrigin[gi] = QPointF(x, y);
            const QPointF origin = session().scatterRawOrigin.value(gi);
            constexpr double earthRadiusM = 6378137.0;
            const double lat0Rad = origin.y() * kPi / 180.0;
            x = (x - origin.x()) * kPi / 180.0 * earthRadiusM * std::cos(lat0Rad);
            y = (y - origin.y()) * kPi / 180.0 * earthRadiusM;
          }
          auto &pts = session().liveScatter[gi];
          pts << QPointF(x, y);
          while (pts.size() > 7200) pts.removeFirst();
        }
      }
    }
    if(recording_ && (!summaryUpdateClock_.isValid() || summaryUpdateClock_.elapsed() >= 1000)){
      updateRecordingSummaryRow();
      summaryUpdateClock_.restart();
    }
    if(plotMode_->currentIndex()==0)refreshGraphs();
  }
  QVector<double> values(const QString &key) const {
    QVector<double> out;
    for(const QVariantMap&r:sessions_.value(currentId_).rawRows){
      const double v=number(r.value(key));
      if(std::isfinite(v))out<<v;
    }
    return out;
  }
  QVariant aggregate(const QString &key,const QString &mode) const {
    const QVector<double> data=values(key);
    if(data.isEmpty())return {
    };
    if(mode=="last")return data.last();
    if(mode=="max")return *std::max_element(data.begin(),data.end());
    if(mode=="min")return *std::min_element(data.begin(),data.end());
    if(mode=="p95"){
      QVector<double> sorted=data;
      std::sort(sorted.begin(),sorted.end());
      const int idx=std::clamp(int(std::ceil(0.95*sorted.size()))-1,0,sorted.size()-1);
      return sorted[idx];
    }
    const double mean=std::accumulate(data.begin(),data.end(),0.0)/data.size();
    if(mode=="mean")return mean;
    double sum=0.0;
    if(mode=="std"){
      for(double v:data)sum+=(v-mean)*(v-mean);
      return std::sqrt(sum/data.size());
    }
    if(mode=="mae"){
      for(double v:data)sum+=std::abs(v);
      return sum/data.size();
    }
    for(double v:data)sum+=v*v;
    return std::sqrt(sum/data.size());
  }
  QVariant percentTrue(const QString &key) const {
    int valid=0,yes=0;
    for(const QVariantMap&r:sessions_.value(currentId_).rawRows){
      if(!r.contains(key))continue;
      ++valid;
      if(r.value(key).toBool())++yes;
    }
    return valid?QVariant(100.0*yes/valid):QVariant();
  }
  QVariant lastText(const QString &key) const {
    const auto&rows=sessions_.value(currentId_).rawRows;
    for(auto it=rows.crbegin();
    it!=rows.crend();
    ++it)if(it->contains(key))return it->value(key);
    return {
    };
  }
  QVariant responseMetric(const QString &metric) const {
    const auto &rows=sessions_.value(currentId_).rawRows;
    if(rows.size()<3)return {
    };
    QVector<double>time,target,actual;
    for(const auto&r:rows){
      double t=number(r.value("elapsed_s")),a=number(r.value("esc_steer_actual")),g=number(r.value("esc_steer_target"));
      if(std::isfinite(t)&&std::isfinite(a)&&std::isfinite(g)){
        time<<t;
        actual<<a;
        target<<g;
      }
    }
    if(time.size()<3)return {
    };
    const int tail=std::max(1,time.size()/10);
    double finalTarget=0.0,finalActual=0.0;
    for(int i=time.size()-tail;
    i<time.size();
    ++i){
      finalTarget+=target[i];
      finalActual+=actual[i];
    }
    finalTarget/=tail;
    finalActual/=tail;
    const double initial=actual.first(),delta=finalTarget-initial;
    if(std::abs(delta)<1e-6)return {
    };
    if(metric=="ess")return (finalTarget-finalActual)*180.0/kPi;
    if(metric=="overshoot"){
      const double peak=delta>0?*std::max_element(actual.begin(),actual.end()):*std::min_element(actual.begin(),actual.end());
      return std::max(0.0,(peak-finalTarget)/delta*100.0);
    }
    auto crossing=[&](double ratio){
      const double threshold=initial+ratio*delta;
      for(int i=0;
      i<actual.size();
      ++i)if((delta>0&&actual[i]>=threshold)||(delta<0&&actual[i]<=threshold))return time[i];
      return std::numeric_limits<double>::quiet_NaN();
    };
    if(metric=="rise"){
      const double t10=crossing(.1),t90=crossing(.9);
      return std::isfinite(t10)&&std::isfinite(t90)?QVariant(t90-t10):QVariant();
    }
    if(metric=="settling"){
      const double band=std::max(std::abs(delta)*.02,0.25*kPi/180.0);
      int lastOutside=-1;
      for(int i=0;
      i<actual.size();
      ++i)if(std::abs(actual[i]-finalTarget)>band)lastOutside=i;
      return lastOutside+1<time.size()?QVariant(time[std::max(0,lastOutside+1)]):QVariant();
    }
    if(metric=="gain"){
      const auto [tmin,tmax]=std::minmax_element(target.begin(),target.end());
      const auto [amin,amax]=std::minmax_element(actual.begin(),actual.end());
      const double input=*tmax-*tmin;
      return input>1e-9?QVariant((*amax-*amin)/input):QVariant();
    }
    return {
    };
  }
  QVariant summaryValue(const QString &column) const {
    QString key=column.toLower();
    key.replace('\\',' ');
    key=key.simplified();
    const QString variant=parameterText(QStringLiteral("variation")),condition=parameterText(QStringLiteral("condition"));
    if(isFinalPerception()){
      const auto countBool=[&](const QString&field,bool wanted){
        int count=0; for(const QVariantMap&r:sessions_.value(currentId_).rawRows)if(r.contains(field)&&r.value(field).toBool()==wanted)++count; return count;
      };
      const auto sumInt=[&](const QString&field){
        qlonglong total=0; for(const QVariantMap&r:sessions_.value(currentId_).rawRows)total+=r.value(field,0).toLongLong(); return total;
      };
      const auto countField=[&](const QString&field){
        int n=0; for(const QVariantMap&r:sessions_.value(currentId_).rawRows)if(r.contains(field)&&number(r.value(field))==number(r.value(field)))++n; return n;
      };
      const auto medianField=[&](const QString&field)->QVariant{
        QVector<double>v; for(const QVariantMap&r:sessions_.value(currentId_).rawRows){double x=number(r.value(field));if(std::isfinite(x))v<<x;}
        if(v.isEmpty())return QVariant(); std::sort(v.begin(),v.end()); const int n=v.size();
        return n%2?QVariant(v[n/2]):QVariant(0.5*(v[n/2-1]+v[n/2]));
      };
      if(key==QStringLiteral("variasi"))return variant;
      if(key==QStringLiteral("kondisi"))return condition.isEmpty()?variant:condition;
      if(currentId_==QStringLiteral("F4.1")){
        if(key==QStringLiteral("jarak"))return groundTruth(QStringLiteral("gt_x"));
        if(key.contains(QStringLiteral("total frame")))return sessions_.value(currentId_).rawRows.size();
        if(key.contains(QStringLiteral("frame terdeteksi")))return countBool(QStringLiteral("derived.target_detected"),true);
        if(key.contains(QStringLiteral("detection rate"))){const int n=sessions_.value(currentId_).rawRows.size();return n?QVariant(100.0*countBool(QStringLiteral("derived.target_detected"),true)/n):QVariant();}
        if(key.contains(QStringLiteral("mean confidence")))return aggregate(QStringLiteral("derived.target_confidence"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("false detection")))return QVariant::fromValue<qlonglong>(sumInt(QStringLiteral("derived.false_detection_count")));
        if(key.contains(QStringLiteral("missed detection")))return countBool(QStringLiteral("derived.missed_detection"),true);
      }
      if(currentId_==QStringLiteral("F4.2")){
        if(key.contains(QStringLiteral("ground truth")))return groundTruth(QStringLiteral("gt_x"));
        if(key.contains(QStringLiteral("mean homography")))return aggregate(QStringLiteral("obstacle_metrics.nearest_forward_homography_m"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("mean tracked")))return aggregate(QStringLiteral("obstacle_metrics.nearest_forward_m"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("error absolut"))){
          const double mean=number(aggregate(QStringLiteral("obstacle_metrics.nearest_forward_m"),QStringLiteral("mean")));
          const double gt=number(groundTruth(QStringLiteral("gt_x"))); return std::isfinite(mean)&&std::isfinite(gt)?QVariant(std::abs(mean-gt)):QVariant();
        }
        if(key==QStringLiteral("mae"))return aggregate(QStringLiteral("derived.obstacle_abs_error_x_m"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("std")))return aggregate(QStringLiteral("obstacle_metrics.nearest_forward_m"),QStringLiteral("std"));
        if(key.contains(QStringLiteral("sampel")))return countField(QStringLiteral("obstacle_metrics.nearest_forward_m"));
      }
      if(currentId_==QStringLiteral("F4.3")){
        if(key.contains(QStringLiteral("iou drivable")))return aggregate(QStringLiteral("derived.iou_drivable"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("iou lane")))return aggregate(QStringLiteral("derived.iou_lane"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("lane valid rate")))return percentTrue(QStringLiteral("lane_state.valid"));
        if(key.contains(QStringLiteral("valid rows")))return aggregate(QStringLiteral("lane_metrics.valid_rows"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("lane confidence")))return aggregate(QStringLiteral("lane_state.confidence"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("center error")))return aggregate(QStringLiteral("lane_state.center_error_m"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("lane state")))return lastText(QStringLiteral("lane_state.state"));
      }
      if(currentId_==QStringLiteral("F4.4")){
        if(key==QStringLiteral("candidate"))return aggregate(QStringLiteral("object_points.count"),QStringLiteral("max"));
        if(key.contains(QStringLiteral("path relevant")))return aggregate(QStringLiteral("path_relevant_points.count"),QStringLiteral("max"));
        if(key.contains(QStringLiteral("planning relevant")))return aggregate(QStringLiteral("planning_relevant_points.count"),QStringLiteral("max"));
        if(key==QStringLiteral("keputusan"))return lastText(QStringLiteral("trajectory_safety_state.decision"));
        if(key==QStringLiteral("success")){
          int pass=0,total=0; for(const QVariantMap&r:sessions_.value(currentId_).rawRows)if(r.value(QStringLiteral("row_type")).toString()==QStringLiteral("trial")){++total;if(r.value(QStringLiteral("trial_success")).toBool())++pass;}
          return total?QVariant(QStringLiteral("%1/%2").arg(pass).arg(total)):QVariant();
        }
        if(key.contains(QStringLiteral("pipeline fps")))return aggregate(QStringLiteral("perception_performance.fps"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("mean processframe")))return aggregate(QStringLiteral("perception_performance.mean_ms"),QStringLiteral("mean"));
        // Each /perception/performance event already represents the native backend
        // window; do not average P95 values into a fake percentile. Use the latest
        // completed window for the report row.
        if(key.contains(QStringLiteral("p95 processframe")))return aggregate(QStringLiteral("perception_performance.p95_ms"),QStringLiteral("last"));
        if(key.contains(QStringLiteral("capture drop"))){const QVector<double>v=values(QStringLiteral("perception_performance.capture_dropped"));return v.isEmpty()?QVariant():QVariant(v.last()-v.first());}
        if(key.contains(QStringLiteral("median response")))return medianField(QStringLiteral("derived.response_time_s"));
        if(key==QStringLiteral("gpu"))return aggregate(QStringLiteral("host.gpu_percent"),QStringLiteral("mean"));
        if(key==QStringLiteral("ram"))return aggregate(QStringLiteral("host.ram_used_gb"),QStringLiteral("mean"));
        if(key.contains(QStringLiteral("temperatur")))return aggregate(QStringLiteral("host.temperature_c"),QStringLiteral("mean"));
      }
    }
    if(key=="gt x")return groundTruth(QStringLiteral("gt_x"));
    if(key=="gt y")return groundTruth(QStringLiteral("gt_y"));
    if(key.contains("sistem x"))return aggregate("obstacle_metrics.nearest_forward_m","mean");
    if(key.contains("sistem y"))return aggregate("obstacle_metrics.nearest_left_m","mean");
    if(key=="error x")return aggregate("derived.obstacle_error_x_m","mean");
    if(key=="error y")return aggregate("derived.obstacle_error_y_m","mean");
    if(key.contains("error 2d")||key.contains("rmse 2d"))return aggregate("derived.obstacle_error_2d_m",key.contains("rmse")?"rmse":"mean");
    if(key.contains("pipeline fps"))return aggregate("perception_performance.fps","mean");
    if(key=="mean"||key.contains("mean processframe"))return aggregate("perception_performance.mean_ms","mean");
    if(key.contains("p95"))return aggregate("perception_performance.p95_ms","mean");
    if(key.contains("capture drop")){
      const auto v=values("perception_performance.capture_dropped");
      return v.isEmpty()?QVariant():QVariant(v.last()-v.first());
    }
    if(key.contains("rviz drop")){
      const auto v=values("perception_performance.rviz_dropped");
      return v.isEmpty()?QVariant():QVariant(v.last()-v.first());
    }
    if(key=="gpu")return aggregate("host.gpu_percent","mean");
    if(key=="ram")return aggregate("host.ram_used_gb","mean");
    if(key.contains("temperatur"))return aggregate("host.temperature_c","mean");
    if(key.contains("jumlah frame"))return sessions_.value(currentId_).rawRows.size();
    if(key.contains("frame terdeteksi")){
      int count=0;
      for(const auto&r:sessions_.value(currentId_).rawRows)if(number(r.value("raw_detections.count"),0)>0)++count;
      return count;
    }
    if(key.contains("detection rate")){
      const int total=sessions_.value(currentId_).rawRows.size();
      int detected=0;
      for(const auto&r:sessions_.value(currentId_).rawRows)if(number(r.value("raw_detections.count"),0)>0)++detected;
      return total?QVariant(100.0*detected/total):QVariant();
    }
    if(key.contains("miss rate")){
      QVariant detection=summaryValue("Detection Rate");
      return detection.isValid()?QVariant(100.0-detection.toDouble()):QVariant();
    }
    if(key.contains("mean confidence"))return aggregate("raw_detections.mean_confidence","mean");
    if(key.contains("std center error")||key.contains("standar deviasi x"))return aggregate(key.contains("center")?"lane_state.center_error_m":"obstacle_metrics.nearest_forward_m","std");
    if(key=="valid lane")return percentTrue("lane_state.valid");
    if(key.contains("system left"))return aggregate("lane_state.left_clearance_m","mean");
    if(key.contains("system right"))return aggregate("lane_state.right_clearance_m","mean");
    if(key=="center error")return aggregate("lane_state.center_error_m","mean");
    if(key=="state")return lastText("lane_state.state");
    if(key.contains("healthy detection"))return lastText("camera_healthy");
    if(key.contains("detection success"))return percentTrue("camera_healthy");
    if(key.contains("trigger emergency"))return lastText("perception_emergency");
    if(key=="candidate")return aggregate("object_points.count","max");
    if(key.contains("path relevant"))return aggregate("path_relevant_points.count","max");
    if(key.contains("planning relevant"))return aggregate("planning_relevant_points.count","max");
    if(key=="keputusan")return lastText("trajectory_safety_state.decision");
    if(key.contains("rmse v"))return aggregate("derived.velocity_error_mps","rmse");
    if(key.contains("rmse yaw"))return aggregate("derived.yaw_error_rps","rmse");
    if(key.contains("cte rmse")||key.contains("tracking rmse")||key.contains("rmse posisi")||key.contains("rmse dinamis"))return aggregate("derived.cte_m","rmse");
    if(key.contains("max cte"))return aggregate("derived.cte_m","max");
    if(key.contains("heading rmse"))return aggregate("derived.path_heading_error_rad","rmse");
    if(key.contains("endpoint rmse")||key.contains("error akhir")||key.contains("endpoint error")||key.contains("stop error"))return aggregate("derived.endpoint_error_m","last");
    if(key.contains("mean endpoint"))return aggregate("derived.endpoint_error_m","mean");
    if(key.contains("std endpoint"))return aggregate("derived.endpoint_error_m","std");
    if(key.contains("path length"))return aggregate("nav_path.length_m","last");
    if(key.contains("planning time"))return aggregate("nav_path.planning_latency_ms","last");
    if(key.contains("time-to-goal")||key=="waktu")return aggregate("goal_state.duration_s","last");
    if(key.contains("actual speed"))return aggregate("esc_drive_actual","mean");
    if(key.contains("speed oscillation")||key.contains("noise output v"))return aggregate("esc_drive_actual","std");
    if(key.contains("overshoot speed"))return aggregate("derived.velocity_error_mps","max");
    if(key.contains("rpm perintah"))return aggregate("derived.target_rpm","mean");
    if(key.contains("rpm feedback"))return aggregate("derived.actual_rpm","mean");
    if(key=="steering perintah")return aggregate("derived.steering_target_deg","mean");
    if(key=="feedback"||key.contains("feedback encoder"))return aggregate("derived.steering_actual_deg","mean");
    if(key=="error"||key.contains("error relatif"))return aggregate("derived.steering_error_rad",key.contains("relatif")?"mae":"mean");
    if(key.contains("rise time"))return responseMetric("rise");
    if(key.contains("settling"))return responseMetric("settling");
    if(key.contains("overshoot"))return responseMetric("overshoot");
    if(key.contains("e ss"))return responseMetric("ess");
    if(key.contains("gain amplitudo"))return responseMetric("gain");
    if(key.contains("iq peak"))return aggregate("foc_telemetry.iq_a","max");
    if(key.contains("iq rms")||key.contains("ripple iq"))return aggregate("foc_telemetry.iq_a",key.contains("ripple")?"std":"rmse");
    // Generic staged-tuning table support. Columns can use the same readable
    // label as a YAML field or live graph series and are filled automatically.
    auto normalizeLabel=[](QString x){
      x=x.toLower();
      x.replace(QRegularExpression(QStringLiteral("[^a-z0-9]+")),QStringLiteral(" "));
      return x.simplified();
    };
    const QString normalizedKey=normalizeLabel(column);
    for(const ExperimentParameterField&field:spec().parameterFields){
      const QString normalizedField=normalizeLabel(field.label);
      const QString normalizedFieldKey=normalizeLabel(field.key);
      if((!normalizedField.isEmpty()&&(normalizedKey==normalizedField||normalizedField.contains(normalizedKey)||normalizedKey.contains(normalizedField)))||
         (!normalizedFieldKey.isEmpty()&&normalizedKey==normalizedFieldKey)){
        const QVariant pv=parameterValue(field.key);
        if(pv.isValid())return pv;
        if(!field.yamlFileKey.isEmpty()&&!field.yamlPath.isEmpty()&&stores_.contains(field.yamlFileKey))
          return stores_.value(field.yamlFileKey)->get(field.yamlPath);
      }
    }
    for(auto it=spec().liveSeries.cbegin();it!=spec().liveSeries.cend();++it){
      const QString label=normalizeLabel(it.key());
      if(label.isEmpty()||(!normalizedKey.contains(label)&&normalizedKey!=label))continue;
      QString mode=QStringLiteral("mean");
      if(normalizedKey.contains(QStringLiteral("p95")))mode=QStringLiteral("p95");
      else if(normalizedKey.contains(QStringLiteral("rmse")))mode=QStringLiteral("rmse");
      else if(normalizedKey.contains(QStringLiteral("std")))mode=QStringLiteral("std");
      else if(normalizedKey.contains(QStringLiteral("max")))mode=QStringLiteral("max");
      else if(normalizedKey.contains(QStringLiteral("min")))mode=QStringLiteral("min");
      else if(normalizedKey.contains(QStringLiteral("akhir"))||normalizedKey.contains(QStringLiteral("last")))mode=QStringLiteral("last");
      return aggregate(it.value(),mode);
    }
    if(key.contains(QStringLiteral("jumlah sampel"))||key==QStringLiteral("samples"))return sessions_.value(currentId_).rawRows.size();
    if(key.contains(QStringLiteral("durasi run"))||key==QStringLiteral("duration s"))return aggregate(QStringLiteral("elapsed_s"),QStringLiteral("last"));
    if(key.contains("mean gyro")){
      const QString axis=variant.toLower();
      return aggregate(axis.startsWith('x')?"imu.gx":axis.startsWith('y')?"imu.gy":"imu.gz","mean");
    }
    if(key=="std (rad/s)"){
      const QString axis=variant.toLower();
      return aggregate(axis.startsWith('x')?"imu.gx":axis.startsWith('y')?"imu.gy":"imu.gz","std");
    }
    if(key.contains("heading imu"))return aggregate("imu.yaw_rad","mean");
    if(key=="success"||key.contains("nav2 success")){
      const QString state=lastText("goal_state.state").toString();
      return state.contains("SUCCEEDED",Qt::CaseInsensitive);
    }
    const QStringList identityTokens={
      "frequency","sensor timeout","threshold","set","mode","alpha","padding","radius","faktor","weight","penalty","max length","command","distance","tolerance","target","rmin","downsampling","bins","parameter","kanal","sumbu","heading referensi","metode","jarak","publish rate","mode capture","kondisi","skenario","id","lapisan","kelompok","tahap","item","bagian","posisi lateral"
    };
    for(const QString&token:identityTokens)if(key.contains(token))return key.contains("kondisi")||key.contains("skenario")?QVariant(condition.isEmpty()?variant:condition):QVariant(variant);
    return {
    };
  }
  void appendSummaryRow() {
    if(session().rawRows.isEmpty()&&!recording_){
      status_->setText("Tidak ada sampel. Mulai run atau gunakan baris manual.");
      return;
    }
    QVariantMap row;
    const QStringList cols=currentColumns();
    for(const QString&column:cols)row[column]=summaryValue(column);
    const QString condition = parameterText(QStringLiteral("condition"));
    if(!condition.isEmpty()&&!cols.isEmpty()&&!row.value(cols.first()).isValid())row[cols.first()]=condition;
    session().summaryRows<<row;
    loading_=true;
    insertRow(row);
    loading_=false;
    table_->resizeColumnsToContents();
    refreshSummaryPlotIfNeeded();
    status_->setText(QStringLiteral("Ringkasan run ditambahkan. Sel kuning wajib dilengkapi secara manual sebelum dipakai di laporan."));
  }
  void addManualRow(){
    QVariantMap row;
    const QStringList cols=currentColumns();
    const QString variation=parameterText(QStringLiteral("variation"));
    const QString condition=parameterText(QStringLiteral("condition"));
    if(!cols.isEmpty())row[cols.first()]=variation.isEmpty()?condition:variation;
    session().summaryRows<<row;
    loading_=true;
    insertRow(row);
    loading_=false;
  }
  void insertRow(const QVariantMap&row){
    const int r=table_->rowCount();
    table_->insertRow(r);
    const QStringList cols=currentColumns();
    for(int c=0;
    c<cols.size();
    ++c){
      const QVariant value=row.value(cols[c]);
      QString text;
      if(value.isValid()){
        if(value.userType()==QMetaType::Bool)text=value.toBool()?"true":"false";
        else{
          bool ok=false;
          double numberValue=value.toDouble(&ok);
          text=ok&&std::isfinite(numberValue)?QString::number(numberValue,'g',10):value.toString();
        }
      }
      auto*item=new QTableWidgetItem(text);
      item->setBackground(text.isEmpty()?QColor("#5a4317"):QColor("#173b25"));
      item->setToolTip(text.isEmpty()?"Tidak tersedia otomatis — isi dari ground truth/instrumen eksternal":"Diisi otomatis; tetap dapat dikoreksi jika metode pengujian mensyaratkan ground truth");
      table_->setItem(r,c,item);
    }
  }
  // While a run is recording, keep the active summary row live so the table
  // reflects the in-progress acquisition instead of waiting for STOP.
  void updateRecordingSummaryRow(){
    if(session().rawRows.isEmpty()||currentId_.isEmpty())return;
    QVariantMap row;
    const QStringList cols=currentColumns();
    for(const QString&column:cols)row[column]=summaryValue(column);
    const QString condition=parameterText(QStringLiteral("condition"));
    if(!condition.isEmpty()&&!cols.isEmpty()&&!row.value(cols.first()).isValid())row[cols.first()]=condition;
    if(activeRecordingSummaryRow_<0||activeRecordingSummaryRow_>=table_->rowCount()){
      loading_=true;
      insertRow(row);
      loading_=false;
      activeRecordingSummaryRow_=table_->rowCount()-1;
    }else{
      loading_=true;
      const int r=activeRecordingSummaryRow_;
      for(int c=0;c<cols.size();++c){
        const QVariant value=row.value(cols[c]);
        QString text;
        if(value.isValid()){
          if(value.userType()==QMetaType::Bool)text=value.toBool()?"true":"false";
          else{
            bool ok=false;
            double numberValue=value.toDouble(&ok);
            text=ok&&std::isfinite(numberValue)?QString::number(numberValue,'g',10):value.toString();
          }
        }
        auto*item=table_->item(r,c);
        if(!item){
          item=new QTableWidgetItem();
          table_->setItem(r,c,item);
        }
        item->setText(text);
        item->setBackground(text.isEmpty()?QColor("#5a4317"):QColor("#173b25"));
        item->setToolTip(text.isEmpty()?"Tidak tersedia otomatis — isi dari ground truth/instrumen eksternal":"Diisi otomatis; tetap dapat dikoreksi jika metode pengujian mensyaratkan ground truth");
      }
      loading_=false;
    }
    table_->resizeColumnsToContents();
  }
  void saveTableState(){
    if(currentId_.isEmpty())return;
    QVector<QVariantMap>rows;
    const QStringList cols=currentColumns();
    for(int r=0;
    r<table_->rowCount();
    ++r){
      QVariantMap row;
      for(int c=0;
      c<cols.size();
      ++c)row[cols[c]]=table_->item(r,c)?table_->item(r,c)->text():QString();
      rows<<row;
    }
    sessions_[currentId_].summaryRows=rows;
  }
  void removeSelectedRows(){
    QSet<int>rows;
    for(const auto&range:table_->selectedRanges())for(int r=range.topRow();
    r<=range.bottomRow();
    ++r)rows.insert(r);
    QList<int>ordered=rows.values();
    std::sort(ordered.begin(),ordered.end(),std::greater<int>());
    loading_=true;
    for(int r:ordered)table_->removeRow(r);
    loading_=false;
    saveTableState();
    refreshSummaryPlotIfNeeded();
  }
  void clearCurrent(){
    if(QMessageBox::question(this,"Bersihkan data","Hapus tabel, raw sample, dan grafik untuk subpengujian ini?")!=QMessageBox::Yes)return;
    sessions_[currentId_]=ExperimentSessionData{
    };
    loading_=true;
    table_->setRowCount(0);
    loading_=false;
    for (GraphCard *c : graphCards_) c->clear();
    status_->setText("Data subpengujian dibersihkan.");
  }
  struct GraphSnapshot {
    QMap<QString, QVector<QPointF>> series;
    QString xLabel = QStringLiteral("Time [s]");
    QString yLabel = QStringLiteral("Nilai");
    bool connectPoints = true;
    QString emptyMessage;
  };
  // Human-readable reason shown on a graph card when the expected source has
  // not delivered any data yet. Data-driven: derived from the graph spec and
  // the live telemetry store — no hardcoded per-leaf UI branches.
  static QString sourceNameForPath(const QString &path) {
    if (path.startsWith(QStringLiteral("ekf_global"))) return QStringLiteral("EKF global /odometry/filtered_map");
    if (path.startsWith(QStringLiteral("ekf_local"))) return QStringLiteral("EKF lokal /odometry/filtered");
    if (path.startsWith(QStringLiteral("esc_odom"))) return QStringLiteral("odometri ESC /esc/odom");
    if (path.startsWith(QStringLiteral("localization_state"))) return QStringLiteral("localization_state (anchor global)");
    if (path.startsWith(QStringLiteral("gnss_quality"))) return QStringLiteral("GNSS /gnss/quality");
    if (path.startsWith(QStringLiteral("gnss_fix"))) return QStringLiteral("GNSS /gnss/fix_raw");
    if (path.startsWith(QStringLiteral("gnss_vel"))) return QStringLiteral("kecepatan GNSS");
    if (path.startsWith(QStringLiteral("gnss_cog"))) return QStringLiteral("COG /gnss/cog_heading_fusion");
    if (path.startsWith(QStringLiteral("imu"))) return QStringLiteral("IMU /imu/data");
    if (path.startsWith(QStringLiteral("esc_"))) return QStringLiteral("ESC");
    if (path.startsWith(QStringLiteral("foc_telemetry"))) return QStringLiteral("ESC FOC /esc/foc/telemetry");
    if (path.startsWith(QStringLiteral("cmd_"))) return QStringLiteral("perintah /cmd_vel");
    if (path.startsWith(QStringLiteral("mppi_"))) return QStringLiteral("MPPI supervisor");
    if (path.startsWith(QStringLiteral("nav_path"))) return QStringLiteral("path Nav2 /plan");
    if (path.startsWith(QStringLiteral("goal_state"))) return QStringLiteral("status goal");
    if (path.startsWith(QStringLiteral("raw_detections"))) return QStringLiteral("deteksi YOLOP /perception/raw_detections");
    if (path.startsWith(QStringLiteral("perception_performance"))) return QStringLiteral("performa YOLOP /perception/performance");
    if (path.startsWith(QStringLiteral("obstacle_metrics"))) return QStringLiteral("metrik obstacle");
    if (path.startsWith(QStringLiteral("lane_state"))) return QStringLiteral("lane safety state");
    if (path.startsWith(QStringLiteral("drivable_"))) return QStringLiteral("drivable space YOLOP");
    if (path.startsWith(QStringLiteral("camera_"))) return QStringLiteral("kesehatan kamera");
    if (path.startsWith(QStringLiteral("trajectory_safety"))) return QStringLiteral("trajectory safety");
    if (path.startsWith(QStringLiteral("near_field"))) return QStringLiteral("near-field state");
    if (path.startsWith(QStringLiteral("derived."))) return QStringLiteral("nilai turunan");
    if (path.startsWith(QStringLiteral("host."))) return QStringLiteral("metrik host");
    return path;
  }
  static QString emptyMessageForSeries(const QStringList &missing, const QString &mode) {
    if (missing.isEmpty()) return QString();
    QStringList names;
    for (const QString &p : missing) {
      const QString n = sourceNameForPath(p);
      if (!names.contains(n)) names << n;
    }
    std::sort(names.begin(), names.end());
    const QString head = mode == QStringLiteral("scatter")
      ? QStringLiteral("BELUM ADA DATA\nMenunggu data valid dari:\n")
      : QStringLiteral("BELUM ADA DATA\nGrafik aktif otomatis begitu data masuk dari:\n");
    QString msg = head + names.join(QStringLiteral("\n")) +
      QStringLiteral("\n\nPeriksa status sensor dan jalannya stack; jangan isi manual.");
    msg += QStringLiteral("\n\nTips:\n") + troubleshootMissingTelemetry(missing);
    return msg;
  }
  static QString troubleshootMissingTelemetry(const QStringList &missing) {
    QStringList tips;
    bool wantEKFLocal = false, wantEKFGlobal = false, wantESC = false, wantIMU = false, wantGNSS = false, wantPerception = false;
    for (const QString &p : missing) {
      if (p.startsWith(QStringLiteral("ekf_local"))) wantEKFLocal = true;
      else if (p.startsWith(QStringLiteral("ekf_global"))) wantEKFGlobal = true;
      else if (p.startsWith(QStringLiteral("esc_")) || p.startsWith(QStringLiteral("foc_telemetry"))) wantESC = true;
      else if (p.startsWith(QStringLiteral("imu"))) wantIMU = true;
      else if (p.startsWith(QStringLiteral("gnss")) || p.startsWith(QStringLiteral("localization_state"))) wantGNSS = true;
      else if (p.startsWith(QStringLiteral("raw_detections")) || p.startsWith(QStringLiteral("perception_performance")) ||
               p.startsWith(QStringLiteral("obstacle_metrics")) || p.startsWith(QStringLiteral("lane_state")) ||
               p.startsWith(QStringLiteral("camera_"))) wantPerception = true;
    }
    if (wantEKFLocal) tips << QStringLiteral("- EKF lokal: cek /odometry/filtered dan /imu/data; pastikan IMU fresh dan sensor_timeout tidak terlewat.");
    if (wantEKFGlobal) tips << QStringLiteral("- EKF global: cek /odometry/filtered_map; butuh GNSS fiks + anchor map dari LocalizationCore.");
    if (wantESC) tips << QStringLiteral("- ESC/aktuator: cek ESC armed/ready, port serial, dan topik /esc/status.");
    if (wantIMU) tips << QStringLiteral("- IMU: cek koneksi serial/USB dan /imu/connected.");
    if (wantGNSS) tips << QStringLiteral("- GNSS: cek /gnss/connected, fix GPS, satelit, DOP/hAcc; outdoor dengan view langit.");
    if (wantPerception) tips << QStringLiteral("- Persepsi: cek /perception/camera_healthy dan pipeline YOLOP; frame gelap/berdebu akan membuat count=0.");
    return tips.join(QStringLiteral("\n"));
  }
  // Build one renderer snapshot from the SAME raw/session data. No acquisition,
  // subscriber, or timer lives here. Special graph semantics are data-driven via
  // ExperimentSpec::graphs; unspecified graphs safely fall back to all liveSeries.
  GraphSnapshot liveDataForGraph(int index) const {
    GraphSnapshot out;
    const ExperimentSpec &s = spec();
    if (index < 0 || index >= s.graphs.size()) {
      out.series = sessions_.value(currentId_).liveSeries;
      if (out.series.isEmpty()) out.emptyMessage = emptyMessageForSeries(
        s.liveSeries.values(), QStringLiteral("time_series"));
      return out;
    }
    const ExperimentGraphSpec &g = s.graphs.at(index);
    if (g.type == QStringLiteral("scatter")) {
      out.xLabel = g.xLabel.isEmpty()?g.xSeries:g.xLabel;
      out.yLabel = g.yLabel.isEmpty()?g.ySeries:g.yLabel;
      out.connectPoints = false;
      // The live scatter buffer starts as soon as the GUI receives valid
      // telemetry. startRecording() clears it together with rawRows, therefore
      // after a recorded run it contains exactly that run's display points.
      out.series[QStringLiteral("Posisi aktual")] = sessions_.value(currentId_).liveScatter.value(index);
      if (out.series.value(QStringLiteral("Posisi aktual")).isEmpty()) {
        out.emptyMessage = emptyMessageForSeries({g.xSeries, g.ySeries}, QStringLiteral("scatter"));
      }
      return out;
    }
    if(!g.xLabel.isEmpty())out.xLabel=g.xLabel;
    if(!g.yLabel.isEmpty())out.yLabel=g.yLabel;
    const QMap<QString, QVector<QPointF>> &all = sessions_.value(currentId_).liveSeries;
    QStringList missing;
    for (const QString &label : g.series) {
      if (all.contains(label) && !all.value(label).isEmpty()) out.series[label] = all.value(label);
      else missing << s.liveSeries.value(label, label);
    }
    if (!missing.isEmpty()) out.emptyMessage = emptyMessageForSeries(missing, QStringLiteral("time_series"));
    return out;
  }
  // Multi-graph refresh. Both normal cards and optional fullscreen receive the
  // same GraphSnapshot from this single refresh flow.
  void refreshGraphs(){
    const ExperimentSpec &s = spec();
    if (graphCards_.size() != s.graphCaptions.size()) rebuildGraphCards();
    for (int i = 0; i < graphCards_.size(); ++i) {
      const QString caption = s.graphCaptions.value(i);
      const QString title = QStringLiteral("Format mengacu %1 — Data Aktual GUI").arg(caption.isEmpty() ? s.section : caption);
      if (plotMode_->currentIndex() == 0) {
        const GraphSnapshot snap = liveDataForGraph(i);
        graphCards_[i]->setData(title, snap.xLabel, snap.yLabel, snap.series, snap.connectPoints, snap.emptyMessage);
      } else {
        refreshSummaryForCard(i, title);
      }
    }
    if (fullscreenDialog_ && fullscreenDialog_->isVisible()) {
      const int idx = fullscreenGraphIndex_;
      if (idx >= 0 && idx < graphCards_.size()) {
        const QString caption = s.graphCaptions.value(idx);
        const QString title = QStringLiteral("Format mengacu %1 — Data Aktual GUI").arg(caption.isEmpty() ? s.section : caption);
        if (plotMode_->currentIndex() == 0) {
          const GraphSnapshot snap = liveDataForGraph(idx);
          fullscreenDialog_->updateData(title, snap.xLabel, snap.yLabel, snap.series, snap.connectPoints, snap.emptyMessage);
        } else refreshSummaryForCard(idx, title, fullscreenDialog_);
      }
    }
  }
  void refreshSummaryForCard(int cardIndex, const QString &title, GraphFullscreenDialog *target = nullptr) {
    Q_UNUSED(cardIndex);
    QMap<QString, QVector<QPointF>> series;
    const QStringList cols = currentColumns();
    for (int c = 1; c < cols.size(); ++c) {
      for (int r = 0; r < table_->rowCount(); ++r) {
        double y = 0.0;
        if (!table_->item(r, c) || !numericText(table_->item(r, c)->text(), y)) continue;
        double x = r + 1.0;
        if (table_->item(r, 0)) {
          double parsed = 0.0;
          if (numericText(table_->item(r, 0)->text(), parsed)) x = parsed;
        }
        series[cols[c]] << QPointF(x, y);
      }
    }
    const QString axisX = cols.value(0, QStringLiteral("Varian"));
    if (target) target->updateData(title, axisX, QStringLiteral("Nilai tabel"), series, true);
    else if (cardIndex >= 0 && cardIndex < graphCards_.size())
    graphCards_[cardIndex]->setData(title, axisX, QStringLiteral("Nilai tabel"), series, true);
  }
  void refreshSummaryPlotIfNeeded(){
    if(plotMode_->currentIndex()==1)refreshGraphs();
  }
  // Fullscreen: single extra renderer, same data model, NO timer/subscriber.
  void openFullscreenGraph(int index){
    const ExperimentSpec &s = spec();
    if (index < 0 || index >= s.graphCaptions.size()) return;
    const QString caption = s.graphCaptions.at(index);
    const QString title = QStringLiteral("Format mengacu %1 — Data Aktual GUI").arg(caption.isEmpty() ? s.section : caption);
    if (!fullscreenDialog_) {
      fullscreenDialog_ = new GraphFullscreenDialog(title, this);
      fullscreenDialog_->setWindowModality(Qt::NonModal);
    } else {
      fullscreenDialog_->setWindowTitle(title);
    }
    fullscreenGraphIndex_ = index;
    if (plotMode_->currentIndex() == 0) {
      const GraphSnapshot snap=liveDataForGraph(index);
      fullscreenDialog_->updateData(title,snap.xLabel,snap.yLabel,snap.series,snap.connectPoints,snap.emptyMessage);
    } else refreshSummaryForCard(index, title, fullscreenDialog_);
    fullscreenDialog_->show();
    fullscreenDialog_->raise();
    fullscreenDialog_->activateWindow();
  }
  void updateAvailability(){
    if(catalog_.isEmpty())return;
    QStringList available,missing;
    for(auto it=spec().liveSeries.cbegin();
    it!=spec().liveSeries.cend();
    ++it){
      const QVariant value=instantValue(it.value());
      if(value.isValid())available<<it.key();
      else missing<<it.key();
    }
    QString text=QStringLiteral("AUTO %1/%2: %3").arg(available.size()).arg(spec().liveSeries.size()).arg(available.join(", "));
    if(!missing.isEmpty())text+=QStringLiteral(" | TIDAK TERSEDIA: ")+missing.join(", ");
    if(subsystem_=="steering"&&telemetry_->age("foc_telemetry")>2.0)text+=QStringLiteral(" | Source ESC tidak mempublish /esc/foc/telemetry: Id/Iq/Vd/Vq/Vbus wajib tetap kosong/manual.");
    availability_->setText(text);
    availability_->setStyleSheet(missing.isEmpty()?"color:#46b36b":"color:#e7953f");
  }
  QStringList missingTopics() {
    QStringList out;
    for(auto it=spec().liveSeries.cbegin();
    it!=spec().liveSeries.cend();
    ++it)if(!instantValue(it.value()).isValid())out<<it.value();
    return out;
  }
  void saveEvidence(bool notify=true){
    saveTableState();
    const QString variation = parameterText(QStringLiteral("variation"));
    const QString stem=slug(subsystem_+"_"+spec().id+"_"+variation);
    // Export every table defined by the leaf. For non-active tables derive a
    // run-summary row from the same raw acquisition so a 2-table section never
    // loses its second report artifact merely because another table was selected.
    QStringList tableCsvPaths;
    QString tablePath;
    const ExperimentSpec &activeSpec=spec();
    for(int tableIndex=0; tableIndex<activeSpec.tableColumns.size(); ++tableIndex){
      const QStringList tableCols=activeSpec.tableColumns.value(tableIndex);
      QVector<QVariantMap> rows;
      if(tableIndex==currentTableIndex_) rows=session().summaryRows;
      else if(!session().rawRows.isEmpty()){
        QVariantMap row;
        for(const QString &column:tableCols) row[column]=summaryValue(column);
        const QString condition=parameterText(QStringLiteral("condition"));
        if(!condition.isEmpty()&&!tableCols.isEmpty()&&!row.value(tableCols.first()).isValid())row[tableCols.first()]=condition;
        rows<<row;
      }
      const QString tableName=activeSpec.tableNames.value(tableIndex,QStringLiteral("Tabel %1").arg(tableIndex+1));
      const QString path=reports_->saveCsvOrdered(
        QStringLiteral("%1_table%2_%3").arg(stem).arg(tableIndex+1).arg(slug(tableName)),tableCols,rows);
      if(!path.isEmpty()){
        tableCsvPaths<<path;
        if(tableIndex==currentTableIndex_)tablePath=path;
      }
    }
    if(tablePath.isEmpty()&&!tableCsvPaths.isEmpty())tablePath=tableCsvPaths.first();
    const QString rawPath=reports_->saveCsv(stem+"_raw",session().rawRows);
    // Save ALL graphs defined for this leaf, not just one.
    QStringList pngPaths;
    for (int i = 0; i < graphCards_.size(); ++i) {
      const QString cap = spec().graphCaptions.value(i);
      const QString pngName = QStringLiteral("%1_graph%2%3").arg(stem).arg(i + 1).arg(cap.isEmpty() ? QString() : QStringLiteral("_") + slug(cap));
      const QString p = reports_->savePng(pngName, graphCards_[i]->plot());
      if (!p.isEmpty()) pngPaths << p;
    }
    const QString pngPath = pngPaths.isEmpty() ? QString() : pngPaths.join("; ");
    QString startTime=session().rawRows.isEmpty()?QString():session().rawRows.first().value("time_iso").toString();
    QString stopTime=session().rawRows.isEmpty()?QString():session().rawRows.last().value("time_iso").toString();

    QString perceptionYamlSnapshot;
    QString trajectorySafetyYamlSnapshot;
    QString runtimeSnapshot;
    QString gtDrivableSnapshot,gtLaneSnapshot,predDrivableSnapshot,predLaneSnapshot;
    if(isFinalPerception()){
      const QString root=reports_->root();
      const auto perceptionStore=stores_.value(QStringLiteral("perception"));
      if(perceptionStore&&!perceptionStore->path().isEmpty()&&QFileInfo::exists(perceptionStore->path())){
        perceptionYamlSnapshot=QDir(root).filePath(stem+QStringLiteral("_perception_yaml_snapshot.yaml"));
        QFile::remove(perceptionYamlSnapshot);
        if(!QFile::copy(perceptionStore->path(),perceptionYamlSnapshot))perceptionYamlSnapshot.clear();
      }
      const auto trajectoryStore=stores_.value(QStringLiteral("trajectory_safety"));
      if(trajectoryStore&&!trajectoryStore->path().isEmpty()&&QFileInfo::exists(trajectoryStore->path())){
        trajectorySafetyYamlSnapshot=QDir(root).filePath(stem+QStringLiteral("_trajectory_safety_yaml_snapshot.yaml"));
        QFile::remove(trajectorySafetyYamlSnapshot);
        if(!QFile::copy(trajectoryStore->path(),trajectorySafetyYamlSnapshot))trajectorySafetyYamlSnapshot.clear();
      }
      QJsonObject runtimeObj;
      runtimeObj.insert(QStringLiteral("section_id"),currentId_);
      runtimeObj.insert(QStringLiteral("verified"),finalConfigVerified_);
      runtimeObj.insert(QStringLiteral("verified_leaf_id"),verifiedLeafId_);
      runtimeObj.insert(QStringLiteral("captured_at"),QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
      runtimeObj.insert(QStringLiteral("yaml_expected"),QJsonObject::fromVariantMap(finalExpectedRuntime_));
      runtimeObj.insert(QStringLiteral("runtime_actual"),QJsonObject::fromVariantMap(verifiedRuntime_));
      runtimeSnapshot=QDir(root).filePath(stem+QStringLiteral("_runtime_parameters.json"));
      QSaveFile rf(runtimeSnapshot);
      if(rf.open(QIODevice::WriteOnly)){
        rf.write(QJsonDocument(runtimeObj).toJson(QJsonDocument::Indented));
        if(!rf.commit())runtimeSnapshot.clear();
      }else runtimeSnapshot.clear();

      if(currentId_==QStringLiteral("F4.3")){
        auto saveMask=[&](const QImage&image,const QString&suffix)->QString{
          if(image.isNull())return {};
          const QString path=QDir(root).filePath(stem+suffix+QStringLiteral(".png"));
          return image.save(path, "PNG")?path:QString();
        };
        gtDrivableSnapshot=saveMask(gtDrivableMask_,QStringLiteral("_gt_drivable"));
        gtLaneSnapshot=saveMask(gtLaneMask_,QStringLiteral("_gt_lane"));
        predDrivableSnapshot=saveMask(telemetry_->get(QStringLiteral("drivable_mask.image")).value<QImage>(),QStringLiteral("_pred_drivable_latest"));
        predLaneSnapshot=saveMask(telemetry_->get(QStringLiteral("lane_mask.image")).value<QImage>(),QStringLiteral("_pred_lane_latest"));
      }
    }

    QJsonObject identityJson;
    const QMap<QString,QString> identity=paramPanel_?paramPanel_->runIdentity():QMap<QString,QString>();
    for(auto it=identity.cbegin();it!=identity.cend();++it)identityJson.insert(it.key(),it.value());
    QJsonObject manifest{
      {"subsystem",subsystem_},
      {"section_id",spec().id},
      {"section_title",spec().section},
      {"table_name",currentTableCaption()},
      {"table_options",QJsonArray::fromStringList(spec().tableNames)},
      {"table_csv_files",QJsonArray::fromStringList(tableCsvPaths)},
      {"graph_options",QJsonArray::fromStringList(spec().graphCaptions)},
      {"graph_png_files",QJsonArray::fromStringList(pngPaths)},
      {"variation",variation},
      {"condition",parameterText(QStringLiteral("condition"))},
      {"run_identity",identityJson},
      {"final_bab4_mode",isFinalPerception()},
      {"config_runtime_verified",isFinalPerception()?finalConfigVerified_:false},
      {"sample_rate_hz",std::isfinite(number(parameterValue(QStringLiteral("sample_rate"))))
        ?number(parameterValue(QStringLiteral("sample_rate")))
        :(stores_.contains("gui")?number(stores_["gui"]->get("reporting.sample_rate_hz",5.0),5.0):5.0)},
      {"start_time",startTime},
      {"stop_time",stopTime},
      {"raw_sample_count",session().rawRows.size()},
      {"summary_row_count",session().summaryRows.size()},
      {"topic_availability",availability_->text()},
      {"missing_topics",QJsonArray::fromStringList(missingTopics())},
      {"table_csv",tablePath},
      {"raw_csv",rawPath},
      {"graph_png",pngPath},
      {"perception_yaml_snapshot",perceptionYamlSnapshot},
      {"trajectory_safety_yaml_snapshot",trajectorySafetyYamlSnapshot},
      {"runtime_parameter_snapshot",runtimeSnapshot},
      {"gt_drivable_mask",gtDrivableSnapshot},
      {"gt_lane_mask",gtLaneSnapshot},
      {"pred_drivable_mask_latest",predDrivableSnapshot},
      {"pred_lane_mask_latest",predLaneSnapshot},
      {"note",isFinalPerception()?QStringLiteral("FINAL BAB IV: raw rows berasal dari event ROS/measurement yang sesuai leaf; konfigurasi runtime diverifikasi sebelum record. Ground truth tetap berasal dari pengukuran/mask eksternal."):QStringLiteral("Blank/yellow cells are not published or require external ground truth; no estimated report values were injected.")}
    };
    const QString manifestPath=QDir(reports_->root()).filePath(stem+"_manifest.json");
    QSaveFile file(manifestPath);
    if(file.open(QIODevice::WriteOnly)){
      file.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented));
      file.commit();
    }
    QStringList extra;
    if(!perceptionYamlSnapshot.isEmpty())extra<<QStringLiteral("YAML snapshot: ")+perceptionYamlSnapshot;
    if(!trajectorySafetyYamlSnapshot.isEmpty())extra<<QStringLiteral("Trajectory safety snapshot: ")+trajectorySafetyYamlSnapshot;
    if(!runtimeSnapshot.isEmpty())extra<<QStringLiteral("Runtime snapshot: ")+runtimeSnapshot;
    if(!gtDrivableSnapshot.isEmpty())extra<<QStringLiteral("GT masks/prediksi 4.3 tersimpan bersama run.");
    const QString savedMessage=QString("Tabel: %1\nSemua tabel: %2\nRaw: %3\nGrafik: %4\nManifest: %5%6")
      .arg(tablePath,tableCsvPaths.join(QStringLiteral("; ")),rawPath,pngPath,manifestPath,
           extra.isEmpty()?QString():QStringLiteral("\n")+extra.join(QStringLiteral("\n")));
    if(notify) QMessageBox::information(this,"Bukti pengujian tersimpan",savedMessage);
  }
};
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
class ImuCalibrationPage:public QWidget{
  public:
  ImuCalibrationPage(
  TelemetryStore *t, ReportManager *r,
  const QMap<QString,std::shared_ptr<YamlStore>> &s, QWidget *p=nullptr)
  : QWidget(p), t_(t), r_(r), s_(s)
  {
    auto *l=new QVBoxLayout(this);
    auto *h=new QLabel("Wizard Kalibrasi Presisi IMU — Stage 2");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto *d=new QLabel(
    "Letakkan kendaraan benar-benar diam. Stage 2 tidak hanya menghitung bias/covariance, "
    "tetapi juga memverifikasi durasi capture, gyro-Z noise, dan error magnitudo gravitasi. "
    "Autonomous motion tetap diblokir sampai stationary_calibration_valid=PASS.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto *b=new QHBoxLayout();
    start_=new QPushButton("▶ Mulai Stationary Capture");
    stop_=new QPushButton("■ Stop + Analisis");
    apply_=new QPushButton("✓ Terapkan + Certify IMU");
    export_=new QPushButton("Ekspor CSV");
    apply_->setEnabled(false);
    export_->setEnabled(false);
    for(auto *w:{
      start_,stop_,apply_,export_
    }) b->addWidget(w);
    l->addLayout(b);
    status_=new QLabel("READY • minimum mengikuti imu.yaml; rekomendasi 30–60 s");
    l->addWidget(status_);
    summary_=new QPlainTextEdit();
    summary_->setReadOnly(true);
    summary_->setMinimumHeight(190);
    l->addWidget(summary_);
    plot_=new LivePlotWidget("IMU: gyro Z, Ackermann yaw-rate & residual");
    l->addWidget(plot_,1);
    connect(start_,&QPushButton::clicked,this,[this](){
      samples_.clear();
      result_.clear();
      rec_=true;
      lastStamp_=-1.0;
      apply_->setEnabled(false);
      export_->setEnabled(false);
      plot_->clear();
      status_->setText("CAPTURING • kendaraan WAJIB diam dan tidak disentuh");
    });
    connect(stop_,&QPushButton::clicked,this,[this](){
      analyze();
    });
    connect(apply_,&QPushButton::clicked,this,[this](){
      applyYaml();
    });
    connect(export_,&QPushButton::clicked,this,[this](){
      QMessageBox::information(this,"CSV",r_->saveCsv("imu_stationary_calibration_stage2",samples_));
    });
    timer_=new QTimer(this);
    timer_->setInterval(50);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
  }
  private:
  TelemetryStore *t_;
  ReportManager *r_;
  QMap<QString,std::shared_ptr<YamlStore>> s_;
  QPushButton *start_,*stop_,*apply_,*export_;
  QLabel *status_,*live_;
  QPlainTextEdit *summary_;
  LivePlotWidget *plot_,*headingPlot_;
  QTimer *timer_;
  bool rec_=false;
  QVector<QVariantMap> samples_;
  QVariantMap result_;
  double lastStamp_=-1.0;
  double imuParam(const QString &name,double fallback)const{
    auto st=s_.value("imu");
    return st ? number(st->get("data_imu_node.ros__parameters."+name,fallback),fallback) : fallback;
  }
  static double mean(const QVector<double>&v){
    return v.isEmpty()?0.0:std::accumulate(v.begin(),v.end(),0.0)/v.size();
  }
  static double variance(const QVector<double>&v){
    if(v.size()<2) return 0.0;
    const double m=mean(v);
    double sum=0.0;
    for(double x:v) sum+=(x-m)*(x-m);
    return sum/(v.size()-1);
  }
  QVector<double> col(const QString&k)const{
    QVector<double> v;
    for(const auto&r:samples_){
      const double x=number(r.value(k));
      if(std::isfinite(x)) v<<x;
    }
    return v;
  }
  void refresh(){
    const QVariantMap imu=t_->get("imu").toMap();
    if(imu.isEmpty()) return;
    const double gyroZ=number(imu.value("gz"));
    const double ackW=number(t_->get("esc_kinematic_yaw_rate"));
    const double residual=(std::isfinite(gyroZ)&&std::isfinite(ackW))?
    ackW-gyroZ:std::numeric_limits<double>::quiet_NaN();
    plot_->append({
      {
        "gyro Z",gyroZ
      },{
        "Ackermann w",ackW
      },{
        "model-IMU residual",residual
      }
    });
    if(!rec_||t_->age("imu")>0.5) return;
    const double stamp=t_->timestamp("imu");
    if(stamp<0.0||stamp==lastStamp_) return;
    lastStamp_=stamp;
    QVariantMap row{
      {
        "t_monotonic",stamp
      }
    };
    for(const char *k:{
      "roll_rad","pitch_rad","yaw_rad","gx","gy","gz","ax","ay","az"
    }){
      const QString key=QString::fromLatin1(k);
      row[key]=imu.value(key);
    }
    row["ackermann_w"]=ackW;
    row["model_imu_residual"]=residual;
    samples_.push_back(row);
    if(samples_.size()%10==0){
      status_->setText(QString("CAPTURING • %1 sampel").arg(samples_.size()));
    }
  }
  void analyze(){
    rec_=false;
    const int minSamples=static_cast<int>(std::lround(imuParam("stationary_calibration_min_samples",80.0)));
    const double minDuration=imuParam("stationary_calibration_min_duration_sec",8.0);
    const double maxGzStd=imuParam("stationary_calibration_max_gyro_z_std_rps",0.03);
    const double maxAccelErr=imuParam("stationary_calibration_max_accel_norm_error_mps2",0.75);
    if(samples_.size()<2){
      status_->setText("FAIL • tidak ada cukup sampel IMU");
      return;
    }
    const QVector<double> gx=col("gx"),gy=col("gy"),gz=col("gz");
    const QVector<double> ax=col("ax"),ay=col("ay"),az=col("az");
    const QVector<double> roll=col("roll_rad"),pitch=col("pitch_rad"),yaw=col("yaw_rad");
    const QVector<double> ts=col("t_monotonic");
    if(gx.isEmpty()||gy.isEmpty()||gz.isEmpty()||ax.isEmpty()||ay.isEmpty()||az.isEmpty()||
    roll.isEmpty()||pitch.isEmpty()||yaw.isEmpty()||ts.size()<2){
      status_->setText("FAIL • kolom IMU tidak lengkap");
      return;
    }
    const double duration=*std::max_element(ts.cbegin(),ts.cend())-
    *std::min_element(ts.cbegin(),ts.cend());
    const double gzStd=std::sqrt(std::max(0.0,variance(gz)));
    QVector<double> accNorm;
    const int n=std::min({
      ax.size(),ay.size(),az.size()
    });
    for(int i=0;
    i<n;
    ++i) accNorm<<std::sqrt(ax[i]*ax[i]+ay[i]*ay[i]+az[i]*az[i]);
    const double accelNormError=std::abs(mean(accNorm)-9.80665);
    const bool pass=samples_.size()>=minSamples && duration>=minDuration &&
    gzStd<=maxGzStd && accelNormError<=maxAccelErr;
    QVariantList oldG=s_.value("imu")->get(
    "data_imu_node.ros__parameters.gyro_bias",QVariantList{
      0.0,0.0,0.0
    }).toList();
    QVariantList oldA=s_.value("imu")->get(
    "data_imu_node.ros__parameters.accel_bias",QVariantList{
      0.0,0.0,0.0
    }).toList();
    while(oldG.size()<3) oldG<<0.0;
    while(oldA.size()<3) oldA<<0.0;
    const double mr=mean(roll),mp=mean(pitch),g=9.80665;
    const QVector<double> accMean={
      mean(ax),mean(ay),mean(az)
    };
    const QVector<double> expected={
      -g*std::sin(mp),g*std::sin(mr)*std::cos(mp),g*std::cos(mr)*std::cos(mp)
    };
    const QVector<double> gm={
      mean(gx),mean(gy),mean(gz)
    };
    const QVector<double> gv={
      variance(gx),variance(gy),variance(gz)
    };
    const QVector<double> av={
      variance(ax),variance(ay),variance(az)
    };
    QVariantList gb,ab,gcov,acov,ocov;
    for(int i=0;
    i<3;
    ++i){
      gb<<oldG[i].toDouble()+gm[i];
      ab<<oldA[i].toDouble()+(accMean[i]-expected[i]);
      gcov<<std::max(1e-7,2.0*gv[i]);
      acov<<std::max(1e-5,2.0*av[i]);
    }
    double sy=0.0,cy=0.0;
    for(double y:yaw){
      sy+=std::sin(y);
      cy+=std::cos(y);
    }
    const double ym=std::atan2(sy/yaw.size(),cy/yaw.size());
    QVector<double> yd;
    for(double y:yaw) yd<<normalizeAngle(y-ym);
    ocov<<std::max(1e-7,2.0*variance(roll))
    <<std::max(1e-7,2.0*variance(pitch))
    <<std::max(1e-7,2.0*variance(yd));
    result_={
      {
        "pass",pass
      },{
        "gyro_bias",gb
      },{
        "accel_bias",ab
      },
      {
        "angular_velocity_covariance",gcov
      },{
        "linear_acceleration_covariance",acov
      },
      {
        "orientation_covariance",ocov
      },{
        "sample_count",samples_.size()
      },
      {
        "duration_sec",duration
      },{
        "gyro_z_std_rps",gzStd
      },
      {
        "accel_norm_error_mps2",accelNormError
      },{
        "mean_yaw_rad",ym
      },
      {
        "limit_min_samples",minSamples
      },{
        "limit_min_duration_sec",minDuration
      },
      {
        "limit_max_gyro_z_std_rps",maxGzStd
      },{
        "limit_max_accel_norm_error_mps2",maxAccelErr
      }
    };
    summary_->setPlainText(QString::fromUtf8(
    QJsonDocument(QJsonObject::fromVariantMap(result_)).toJson(QJsonDocument::Indented)));
    status_->setText(QString("%1 • N=%2 duration=%3 s • gyroZ std=%4 rad/s • |a|-g=%5 m/s²")
    .arg(pass?"PASS":"FAIL").arg(samples_.size()).arg(duration,0,'f',1)
    .arg(gzStd,0,'f',5).arg(accelNormError,0,'f',3));
    apply_->setEnabled(pass);
    export_->setEnabled(true);
  }
  void applyYaml(){
    if(result_.isEmpty()||!result_.value("pass").toBool()) return;
    const QString saved=QDateTime::currentDateTime().toString(Qt::ISODate);
    for(const QString &key:{
      QString("imu"),QString("imu_calibration")
    }){
      if(!s_.contains(key)) continue;
      for(const QString &name:{
        QString("gyro_bias"),QString("accel_bias"),
        QString("angular_velocity_covariance"),QString("linear_acceleration_covariance"),
        QString("orientation_covariance")
      }){
        s_[key]->set("data_imu_node.ros__parameters."+name,result_.value(name));
      }
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_valid",true);
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_saved_at",saved);
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_sample_count",result_.value("sample_count"));
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_duration_sec",result_.value("duration_sec"));
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_gyro_z_std_rps",result_.value("gyro_z_std_rps"));
      s_[key]->set("data_imu_node.ros__parameters.stationary_calibration_accel_norm_error_mps2",result_.value("accel_norm_error_mps2"));
    }
    if(s_.contains("localization")){
      auto loc=s_.value("localization");
      // Estimator policy is invariant: GNSS contributes body vx+vyaw while
      // absolute yaw remains IMU-only. Calibration revocation only clears
      // evidence used by the autonomy quality gate; it must not starve EKF.
      loc->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
      loc->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",false);
      loc->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_valid",false);
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_valid",false);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_saved_at",QString());
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_saved_at",QString());
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_epoch_count",0);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_qualified_ratio",0.0);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_sync_p95_sec",0.0);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_wheel_residual_p95_mps",0.0);
      loc->set("localization_core.ros__parameters.gnss_velocity_calibration_lateral_p95_mps",0.0);
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_epoch_count",0);
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_qualified_ratio",0.0);
      loc->set("localization_core.ros__parameters.gnss_cog_calibration_residual_p95_rad",0.0);
    }
    status_->setText("IMU STAGE-2 CERTIFIED ✓ • GNSS evidence dicabut; EKF GNSS vx+vyaw tetap ON, ulangi straight-run Stage 2");
    apply_->setEnabled(false);
  }
};
class GnssCalibrationPage:public QWidget{
  public:
  GnssCalibrationPage(
  TelemetryStore *t, ReportManager *r,
  const QMap<QString,std::shared_ptr<YamlStore>> &s, QWidget *p=nullptr)
  : QWidget(p), t_(t), r_(r), s_(s)
  {
    auto *l=new QVBoxLayout(this);
    auto *h=new QLabel("GNSS Motion Qualification & Certified Fusion — Stage 2");
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto *d=new QLabel(
    "Live qualification hanya menunjukkan epoch saat ini. Stage 2 mewajibkan satu straight-run "
    "lengkap dengan statistik PASS untuk quality gate autonomy. Estimator selalu memakai GNSS vx+vyaw, "
    "sedangkan absolute yaw EKF hanya dari IMU. COG GNSS divalidasi sebagai diagnostik, bukan yaw source. "
    "Drive odometry dan IMU stationary calibration harus PASS untuk sertifikasi commissioning.");
    d->setWordWrap(true);
    l->addWidget(d);
    auto *bar=new QHBoxLayout();
    stationary_=new QPushButton("▶ Stationary Capture");
    cog_=new QPushButton("▶ Straight Motion Run");
    stop_=new QPushButton("■ Stop + Analisis");
    export_=new QPushButton("Ekspor CSV");
    for(auto *w:{
      stationary_,cog_,stop_,export_
    }) bar->addWidget(w);
    l->addLayout(bar);
    auto *fuse=new QHBoxLayout();
    velEnable_=new QPushButton("Certify GNSS vx + vyaw");
    cogEnable_=new QPushButton("Certify COG Diagnostic");
    disable_=new QPushButton("Restore EKF Sensor Policy");
    revoke_=new QPushButton("Revoke Certifications");
    velPill_=new StatusPill("VEL --");
    cogPill_=new StatusPill("COG --");
    for(auto *w:{
      velEnable_,cogEnable_,disable_,revoke_
    }) fuse->addWidget(w);
    fuse->addWidget(velPill_);
    fuse->addWidget(cogPill_);
    l->addLayout(fuse);
    status_=new QLabel("READY • lakukan IMU + drive odometry calibration lebih dulu");
    status_->setWordWrap(true);
    l->addWidget(status_);
    live_=new QLabel("GNSS live: --");
    live_->setWordWrap(true);
    live_->setObjectName("telemetryCard");
    l->addWidget(live_);
    summary_=new QPlainTextEdit();
    summary_->setReadOnly(true);
    summary_->setMinimumHeight(210);
    l->addWidget(summary_);
    plot_=new LivePlotWidget("GNSS speed: wheel / Doppler / point-fit");
    l->addWidget(plot_,1);
    headingPlot_=new LivePlotWidget("GNSS heading: COG yaw / derived vyaw");
    l->addWidget(headingPlot_,1);
    connect(stationary_,&QPushButton::clicked,this,[this](){
      start("stationary");
    });
    connect(cog_,&QPushButton::clicked,this,[this](){
      start("motion");
    });
    connect(stop_,&QPushButton::clicked,this,[this](){
      analyze();
    });
    connect(export_,&QPushButton::clicked,this,[this](){
      QMessageBox::information(this,"CSV",r_->saveCsv(
      mode_=="motion"?"gnss_stage2_motion_run":"gnss_stationary_accuracy",samples_));
    });
    connect(velEnable_,&QPushButton::clicked,this,[this](){
      certifyVelocity();
    });
    connect(cogEnable_,&QPushButton::clicked,this,[this](){
      certifyCog();
    });
    connect(disable_,&QPushButton::clicked,this,[this](){
      restoreFusionPolicy();
    });
    connect(revoke_,&QPushButton::clicked,this,[this](){
      revokeCertifications();
    });
    timer_=new QTimer(this);
    timer_->setInterval(100);
    connect(timer_,&QTimer::timeout,this,[this](){
      refresh();
    });
    timer_->start();
  }
  private:
  TelemetryStore *t_;
  ReportManager *r_;
  QMap<QString,std::shared_ptr<YamlStore>> s_;
  QPushButton *stationary_,*cog_,*stop_,*export_,*velEnable_,*cogEnable_,*disable_,*revoke_;
  StatusPill *velPill_,*cogPill_;
  QLabel *status_,*live_;
  QPlainTextEdit *summary_;
  LivePlotWidget *plot_,*headingPlot_;
  QTimer *timer_;
  QString mode_="stationary";
  bool rec_=false;
  bool velocityRunPass_=false;
  bool cogRunPass_=false;
  QVector<QVariantMap> samples_;
  QVariantMap result_;
  double lastStamp_=-1.0;
  QVariant localParam(const QString &name,const QVariant &fallback=QVariant())const{
    auto st=s_.value("localization");
    return st?st->get("localization_core.ros__parameters."+name,fallback):fallback;
  }
  bool driveCalibrated()const{
    auto st=s_.value("vehicle");
    return st&&st->get("vehicle.ros__parameters.drive_odometry_calibration_valid",false).toBool();
  }
  bool imuCalibrated()const{
    auto st=s_.value("imu");
    return st&&st->get("data_imu_node.ros__parameters.stationary_calibration_valid",false).toBool();
  }
  bool velocityCertified()const{
    return localParam("gnss_velocity_calibration_valid",false).toBool();
  }
  bool cogCertified()const{
    return localParam("gnss_cog_calibration_valid",false).toBool();
  }
  static double percentile(QVector<double>v,double p){
    QVector<double> finite;
    for(double x:v) if(std::isfinite(x)) finite<<x;
    if(finite.isEmpty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(finite.begin(),finite.end());
    const double x=(finite.size()-1)*std::clamp(p,0.0,1.0);
    const int lo=static_cast<int>(std::floor(x));
    const int hi=static_cast<int>(std::ceil(x));
    return lo==hi?finite[lo]:finite[lo]+(finite[hi]-finite[lo])*(x-lo);
  }
  void start(const QString &m){
    mode_=m;
    samples_.clear();
    result_.clear();
    velocityRunPass_=false;
    cogRunPass_=false;
    rec_=true;
    lastStamp_=-1.0;
    plot_->clear();
    status_->setText(m=="motion"?
    "CAPTURE MOTION • maju lurus stabil; jangan sertifikasi dari belokan/reverse":
    "CAPTURE STATIONARY • kendaraan diam");
  }
  void refresh(){
    const bool liveV=t_->get("gnss_velocity_qualified",false).toBool();
    const bool liveC=t_->get("gnss_cog_qualified",false).toBool();
    const bool certV=velocityCertified();
    const bool certC=cogCertified();
    velPill_->setStatus(certV?"ok":(liveV?"warn":"bad"),
    certV?"VEL CERTIFIED":(liveV?"VEL LIVE PASS":"VEL WAIT"));
    cogPill_->setStatus(certC?"ok":(liveC?"warn":"bad"),
    certC?"COG CERTIFIED":(liveC?"COG LIVE PASS":"COG WAIT"));
    const QVariantMap q=t_->get("gnss_quality").toMap();
    const QVariantMap fit=t_->get("gnss_vel_fit").toMap();
    const QVariantMap base=t_->get("gnss_base_vel").toMap();
    const QVariantMap val=t_->get("gnss_motion_validation").toMap();
    const QVariantMap fix=t_->get("gnss_fix").toMap();
    const QVariantMap gs=t_->get("gnss_status").toMap();
    const double cog=number(q.value("course_enu_rad"));
    const double vyaw=number(gs.value("gnss_vyaw"));
    const bool vyawValid=gs.value("gnss_vyaw_valid").toBool();
    live_->setText(QString("lat %1 | lon %2 | sat %3 | fixType %4 | gnssFixOK %5 | DOP %6 | hAcc %7 m | sAcc %8 m/s\nCOG/yaw %9 deg | base vx %10 m/s | vyaw %11 deg/s (%12) | PVT %13 Hz")
      .arg(variantText(fix.value("lat"))).arg(variantText(fix.value("lon")))
      .arg(variantText(q.value("sat"))).arg(variantText(q.value("fix_type")))
      .arg(q.value("gnss_fix_ok").toBool()?"true":"false")
      .arg(variantText(q.value("dop"))).arg(variantText(q.value("hacc_m")))
      .arg(variantText(q.value("sacc_mps")))
      .arg(std::isfinite(cog)?QString::number(cog*180.0/kPi,'f',2):"--")
      .arg(variantText(base.value("vx")))
      .arg(vyawValid&&std::isfinite(vyaw)?QString::number(vyaw*180.0/kPi,'f',2):"--")
      .arg(vyawValid?"VALID":"LOW-SPEED/INVALID")
      .arg(variantText(q.value("pvt_rate_hz"))));
    QMap<QString,double> pl{
      {
        "Doppler",number(q.value("ground_speed_mps"))
      },
      {
        "Position fit",number(fit.value("speed"))
      },{
        "Base vx",number(base.value("vx"))
      },
      {
        "Wheel vx",number(val.value("wheel_vx_at_measurement_mps"))
      }
    };
    plot_->append(pl);
    QMap<QString,double> hp;
    if(std::isfinite(cog)) hp["COG yaw deg"]=cog*180.0/kPi;
    if(vyawValid&&std::isfinite(vyaw)) hp["GNSS vyaw deg/s"]=vyaw*180.0/kPi;
    if(!hp.isEmpty()) headingPlot_->append(hp);
    if(!rec_) return;
    const double stamp=t_->timestamp("gnss_fix");
    if(stamp<0.0||stamp==lastStamp_) return;
    lastStamp_=stamp;
    QVariantMap row{
      {
        "t_monotonic",stamp
      },{
        "lat",fix.value("lat")
      },
      {
        "lon",fix.value("lon")
      },{
        "alt",fix.value("alt")
      }
    };
    for(const char *k:{
      "sat","dop","hacc_m","vacc_m","sacc_mps","ground_speed_mps",
      "course_enu_rad","course_accuracy_rad","itow_ms","vel_e_mps","vel_n_mps","vel_d_mps",
      "pvt_rate_hz","measurement_age_sec","hdop","vdop","nav_cov_vel_valid"
    }){
      const QString key=QString::fromLatin1(k);
      row[key]=q.value(key);
    }
    for(const char *k:{
      "vx","vy","speed","course_enu_rad"
    }){
      const QString key=QString::fromLatin1(k);
      row[QStringLiteral("fit_")+key]=fit.value(key);
    }
    for(const char *k:{
      "vx","vy","speed"
    }){
      const QString key=QString::fromLatin1(k);
      row[QStringLiteral("base_")+key]=base.value(key);
    }
    for(const char *k:{
      "sync_gap_sec","yaw_at_measurement_rad","yaw_rate_at_measurement_rps",
      "wheel_vx_at_measurement_mps","wheel_minus_gnss_mps","cog_minus_vel_course_rad",
      "fit_minus_gnss_speed_mps","fit_minus_cog_rad","velocity_qualified","cog_qualified",
      "wheel_slip","quality_fresh","velocity_covariance_valid","reject_reason",
      "timestamp_reject_count","covariance_reject_count","sync_reject_count"
    }){
      const QString key=QString::fromLatin1(k);
      row[QStringLiteral("validation_")+key]=val.value(key);
    }
    samples_.push_back(row);
  }
  QVector<double> numericColumn(const QString &key,bool absolute=false)const{
    QVector<double> out;
    for(const auto &r:samples_){
      double x=number(r.value(key));
      if(!std::isfinite(x)||std::abs(x)>100.0) continue;
      if(absolute) x=std::abs(x);
      out<<x;
    }
    return out;
  }
  double trueRatio(const QString &key)const{
    if(samples_.isEmpty()) return 0.0;
    int valid=0,total=0;
    for(const auto&r:samples_){
      if(!r.contains(key)) continue;
      ++total;
      if(r.value(key).toBool()) ++valid;
    }
    return total>0?static_cast<double>(valid)/total:0.0;
  }
  void analyze(){
    rec_=false;
    if(samples_.size()<10){
      status_->setText("FAIL • sampel terlalu sedikit");
      return;
    }
    if(mode_=="stationary"){
      QVector<double> lat,lon;
      for(const auto&r:samples_){
        const double a=number(r.value("lat")),b=number(r.value("lon"));
        if(std::isfinite(a)&&std::isfinite(b)){
          lat<<a;
          lon<<b;
        }
      }
      if(lat.size()<5){
        status_->setText("FAIL • GNSS fix stationary tidak cukup");
        return;
      }
      const double lat0=std::accumulate(lat.begin(),lat.end(),0.0)/lat.size();
      const double lon0=std::accumulate(lon.begin(),lon.end(),0.0)/lon.size();
      QVector<double> rad;
      for(int i=0;
      i<lat.size();
      ++i){
        const double x=6378137.0*(lon[i]-lon0)*kPi/180.0*std::cos(lat0*kPi/180.0);
        const double y=6378137.0*(lat[i]-lat0)*kPi/180.0;
        rad<<std::hypot(x,y);
      }
      double rms=0.0;
      for(double x:rad) rms+=x*x;
      rms=std::sqrt(rms/rad.size());
      result_={
        {
          "mode","stationary"
        },{
          "samples",lat.size()
        },{
          "mean_lat",lat0
        },{
          "mean_lon",lon0
        },
        {
          "rms_m",rms
        },{
          "cep50_m",percentile(rad,.5)
        },{
          "cep95_m",percentile(rad,.95)
        }
      };
      summary_->setPlainText(QString::fromUtf8(
      QJsonDocument(QJsonObject::fromVariantMap(result_)).toJson(QJsonDocument::Indented)));
      status_->setText(QString("STATIONARY • RMS=%1 m • CEP95=%2 m • evidence only, bukan fusion certificate")
      .arg(rms,0,'f',2).arg(percentile(rad,.95),0,'f',2));
      export_->setEnabled(true);
      return;
    }
    const int minVel=localParam("stage2_min_velocity_epochs",50).toInt();
    const int minCog=localParam("stage2_min_cog_epochs",30).toInt();
    const double minVelRatio=number(localParam("stage2_min_velocity_qualified_ratio",0.85),0.85);
    const double minCogRatio=number(localParam("stage2_min_cog_qualified_ratio",0.70),0.70);
    const double maxSync=number(localParam("stage2_max_sync_gap_p95_sec",0.20),0.20);
    const double maxWheel=number(localParam("stage2_max_wheel_gnss_residual_p95_mps",0.25),0.25);
    const double maxLat=number(localParam("stage2_max_lateral_velocity_p95_mps",0.15),0.15);
    const double maxCog=number(localParam("stage2_max_cog_doppler_residual_p95_rad",0.1745329252),0.1745329252);
    const double velRatio=trueRatio("validation_velocity_qualified");
    const double cogRatio=trueRatio("validation_cog_qualified");
    const double covRatio=trueRatio("validation_velocity_covariance_valid");
    const double qualityFreshRatio=trueRatio("validation_quality_fresh");
    const double syncP95=percentile(numericColumn("validation_sync_gap_sec",true),.95);
    const double wheelP95=percentile(numericColumn("validation_wheel_minus_gnss_mps",true),.95);
    const double latP95=percentile(numericColumn("base_vy",true),.95);
    const QVector<double> cogResidual=numericColumn("validation_cog_minus_vel_course_rad",true);
    const double cogP95=percentile(cogResidual,.95);
    const int cogEpochs=cogResidual.size();
    const bool prereq=driveCalibrated()&&imuCalibrated();
    velocityRunPass_=prereq && samples_.size()>=minVel && velRatio>=minVelRatio &&
    covRatio>=minVelRatio && qualityFreshRatio>=minVelRatio &&
    std::isfinite(syncP95)&&syncP95<=maxSync && std::isfinite(wheelP95)&&wheelP95<=maxWheel &&
    std::isfinite(latP95)&&latP95<=maxLat;
    cogRunPass_=velocityRunPass_ && cogEpochs>=minCog && cogRatio>=minCogRatio &&
    std::isfinite(cogP95)&&cogP95<=maxCog;
    result_={
      {
        "mode","motion"
      },{
        "total_epochs",samples_.size()
      },{
        "drive_odometry_calibrated",driveCalibrated()
      },
      {
        "imu_stationary_calibrated",imuCalibrated()
      },{
        "velocity_qualified_ratio",velRatio
      },
      {
        "cog_qualified_ratio",cogRatio
      },{
        "velocity_covariance_valid_ratio",covRatio
      },
      {
        "quality_fresh_ratio",qualityFreshRatio
      },{
        "sync_gap_p95_sec",syncP95
      },
      {
        "wheel_gnss_residual_p95_mps",wheelP95
      },{
        "base_lateral_velocity_p95_mps",latP95
      },
      {
        "cog_residual_epochs",cogEpochs
      },{
        "cog_vs_doppler_p95_deg",cogP95*180.0/kPi
      },
      {
        "velocity_run_pass",velocityRunPass_
      },{
        "cog_run_pass",cogRunPass_
      },
      {
        "limit_min_velocity_epochs",minVel
      },{
        "limit_min_cog_epochs",minCog
      },
      {
        "limit_velocity_ratio",minVelRatio
      },{
        "limit_cog_ratio",minCogRatio
      },
      {
        "limit_sync_p95_sec",maxSync
      },{
        "limit_wheel_residual_p95_mps",maxWheel
      },
      {
        "limit_lateral_p95_mps",maxLat
      },{
        "limit_cog_p95_deg",maxCog*180.0/kPi
      }
    };
    summary_->setPlainText(QString::fromUtf8(
    QJsonDocument(QJsonObject::fromVariantMap(result_)).toJson(QJsonDocument::Indented)));
    status_->setText(QString("MOTION %1 • COG %2 • vel ratio=%3% • wheel p95=%4 m/s • sync p95=%5 s • COG p95=%6°")
    .arg(velocityRunPass_?"PASS":"FAIL").arg(cogRunPass_?"PASS":"WAIT/FAIL")
    .arg(100.0*velRatio,0,'f',1).arg(wheelP95,0,'f',3).arg(syncP95,0,'f',3)
    .arg(cogP95*180.0/kPi,0,'f',2));
    export_->setEnabled(true);
  }
  void certifyVelocity(){
    if(!velocityRunPass_){
      QMessageBox::warning(this,"GNSS Velocity","Straight-run Stage-2 belum PASS. Stop + Analisis terlebih dahulu.");
      return;
    }
    auto st=s_.value("localization");
    if(!st) return;
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_valid",true);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_saved_at",QDateTime::currentDateTime().toString(Qt::ISODate));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_epoch_count",result_.value("total_epochs"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_qualified_ratio",result_.value("velocity_qualified_ratio"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_sync_p95_sec",result_.value("sync_gap_p95_sec"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_wheel_residual_p95_mps",result_.value("wheel_gnss_residual_p95_mps"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_lateral_p95_mps",result_.value("base_lateral_velocity_p95_mps"));
    st->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
    st->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",false);
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    status_->setText("GNSS vx+vyaw CERTIFIED ✓ • EKF policy: GNSS motion + IMU absolute yaw");
  }
  void certifyCog(){
    if(!cogRunPass_){
      QMessageBox::warning(this,"GNSS COG","COG straight-run Stage-2 belum PASS.");
      return;
    }
    if(!velocityCertified()&&!velocityRunPass_){
      QMessageBox::warning(this,"GNSS COG","Velocity certification harus PASS lebih dulu.");
      return;
    }
    auto st=s_.value("localization");
    if(!st) return;
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_valid",true);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_valid",true);
    const QString saved=QDateTime::currentDateTime().toString(Qt::ISODate);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_saved_at",saved);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_saved_at",saved);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_epoch_count",result_.value("total_epochs"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_qualified_ratio",result_.value("velocity_qualified_ratio"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_sync_p95_sec",result_.value("sync_gap_p95_sec"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_wheel_residual_p95_mps",result_.value("wheel_gnss_residual_p95_mps"));
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_lateral_p95_mps",result_.value("base_lateral_velocity_p95_mps"));
    st->set("localization_core.ros__parameters.gnss_cog_calibration_epoch_count",result_.value("cog_residual_epochs"));
    st->set("localization_core.ros__parameters.gnss_cog_calibration_qualified_ratio",result_.value("cog_qualified_ratio"));
    st->set("localization_core.ros__parameters.gnss_cog_calibration_residual_p95_rad",number(result_.value("cog_vs_doppler_p95_deg"),0.0)*kPi/180.0);
    st->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
    st->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",true);
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    status_->setText("GNSS COG CERTIFIED ✓ • EKF global: COG absolute yaw + IMU gyro continuity");
  }
  void restoreFusionPolicy(){
    auto st=s_.value("localization");
    if(!st) return;
    st->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
    st->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",true);
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    status_->setText("EKF SENSOR POLICY RESTORED ✓ • GNSS=vx, COG=yaw(abs), IMU=vyaw(continuity)");
  }
  void revokeCertifications(){
    if(QMessageBox::question(this,"Revoke Stage-2",
    "Hapus certification velocity + COG? Estimator GNSS vx+vyaw tetap aktif; autonomy quality gate kembali WAIT. Gunakan setelah perubahan mounting, odometry scale, IMU, GNSS lever arm, atau wiring.")!=QMessageBox::Yes) return;
    auto st=s_.value("localization");
    if(!st) return;
    st->set("localization_core.ros__parameters.enable_global_gnss_velocity_fusion",true);
    st->set("localization_core.ros__parameters.enable_global_gnss_cog_fusion",false);
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_valid",false);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_valid",false);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_saved_at",QString());
    st->set("localization_core.ros__parameters.gnss_cog_calibration_saved_at",QString());
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_epoch_count",0);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_qualified_ratio",0.0);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_sync_p95_sec",0.0);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_wheel_residual_p95_mps",0.0);
    st->set("localization_core.ros__parameters.gnss_velocity_calibration_lateral_p95_mps",0.0);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_epoch_count",0);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_qualified_ratio",0.0);
    st->set("localization_core.ros__parameters.gnss_cog_calibration_residual_p95_rad",0.0);
    velocityRunPass_=false;
    cogRunPass_=false;
    st->set("localization_core.ros__parameters.enable_gnss_course_yaw_correction",false);
    status_->setText("STAGE-2 CERTIFICATION REVOKED • estimator GNSS vx+vyaw tetap ON; autonomy gate WAIT");
  }
};
