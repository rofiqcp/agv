// Embedded RViz/Nav2 map for the native Qt operator GUI.
//
// Design goals:
// - the GUI launch still owns the same autonomous.launch.py runtime;
// - no second rviz2 process/window is started;
// - OGRE/RViz is created lazily only after the operator opens NAV2 MAP;
// - rendering stops when the page is hidden;
// - large costmaps are opt-in while map, URDF, A* path and sampled MPPI
//   trajectories stay visible by default.

class EmbeddedNav2Panel:public QWidget{
  public:EmbeddedNav2Panel(
    const WorkspacePaths&paths,
    const QMap<QString,std::shared_ptr<YamlStore>>&stores,
    TelemetryStore*telemetry,
    RosBridge*ros,
    QWidget*parent=nullptr)
  :QWidget(parent),paths_(paths),stores_(stores),telemetry_(telemetry),ros_(ros){
    auto*root=new QVBoxLayout(this);
    root->setContentsMargins(0,0,0,0);
    root->setSpacing(6);

    auto*title=new QLabel(QStringLiteral("Nav2 Live Map — Smac Hybrid-A* + MPPI Ackermann"));
    title->setObjectName(QStringLiteral("pageTitle"));
    root->addWidget(title);
    auto*description=new QLabel(QStringLiteral(
      "Fixed frame map. MAP, URDF, goal, global path Smac Hybrid-A*, local transformed plan, "
      "footprint, dan kandidat trajectory MPPI tampil di panel ini. Costmap besar default OFF agar GUI ringan."));
    description->setObjectName(QStringLiteral("pageDescription"));
    description->setWordWrap(true);
    root->addWidget(description);

    auto*tools=new QHBoxLayout();
    moveButton_=new QPushButton(QStringLiteral("Geser / Zoom"));
    goalButton_=new QPushButton(QStringLiteral("2D GOAL POSE"));
    initialPoseButton_=new QPushButton(QStringLiteral("Initial Pose"));
    auto*fitMapButton=new QPushButton(QStringLiteral("Map Penuh"));
    auto*focusRobotButton=new QPushButton(QStringLiteral("Fokus URDF"));
    auto*cancelButton=new QPushButton(QStringLiteral("Batalkan Goal"));
    for(auto*button:{moveButton_,goalButton_,initialPoseButton_})button->setCheckable(true);
    goalButton_->setObjectName(QStringLiteral("primaryButton"));
    for(auto*button:{moveButton_,goalButton_,initialPoseButton_,fitMapButton,focusRobotButton,cancelButton})tools->addWidget(button);
    tools->addStretch();
    root->addLayout(tools);

    auto*layers=new QHBoxLayout();
    globalCostmapCheck_=new QCheckBox(QStringLiteral("Global Costmap"));
    localCostmapCheck_=new QCheckBox(QStringLiteral("Local Costmap"));
    footprintCheck_=new QCheckBox(QStringLiteral("Footprint"));
    mppiCheck_=new QCheckBox(QStringLiteral("Kandidat MPPI"));
    globalCostmapCheck_->setChecked(false);
    // Local costmap hanya 8x8 m dan ringan; nyalakan default agar obstacle +
    // inflation langsung terlihat. Global costmap UNDIP tetap opt-in karena besar.
    localCostmapCheck_->setChecked(true);
    footprintCheck_->setChecked(true);
    mppiCheck_->setChecked(true);
    layers->addWidget(new QLabel(QStringLiteral("Layer:")));
    for(auto*box:{globalCostmapCheck_,localCostmapCheck_,footprintCheck_,mppiCheck_})layers->addWidget(box);
    layers->addStretch();
    status_=new QLabel(QStringLiteral("RViz standby — buka halaman ini untuk inisialisasi"));
    status_->setObjectName(QStringLiteral("metricCard"));
    layers->addWidget(status_,1);
    root->addLayout(layers);

    renderHost_=new QFrame();
    renderHost_->setObjectName(QStringLiteral("metricCard"));
    renderLayout_=new QVBoxLayout(renderHost_);
    renderLayout_->setContentsMargins(0,0,0,0);
    placeholder_=new QLabel(QStringLiteral(
      "NAV2 LIVE belum dimuat\n\nKlik NAV2 MAP untuk memuat renderer.\n"
      "Jika OpenGL/OGRE tidak tersedia, gunakan subtab Ground Truth / PGM sebagai fallback."));
    placeholder_->setAlignment(Qt::AlignCenter);
    placeholder_->setStyleSheet(QStringLiteral("color:#aab0ba;font-size:15px;"));
    renderLayout_->addWidget(placeholder_,1);
    root->addWidget(renderHost_,1);

    connect(moveButton_,&QPushButton::clicked,this,[this](){selectTool(moveTool_,moveButton_);});
    connect(goalButton_,&QPushButton::clicked,this,[this](){selectTool(goalTool_,goalButton_);});
    connect(initialPoseButton_,&QPushButton::clicked,this,[this](){selectTool(initialPoseTool_,initialPoseButton_);});
    connect(fitMapButton,&QPushButton::clicked,this,[this](){
      ensureInitialized();
      fitFullMap();
    });
    connect(focusRobotButton,&QPushButton::clicked,this,[this](){
      ensureInitialized();
      focusRobot();
    });
    connect(cancelButton,&QPushButton::clicked,this,[this](){
      if(ros_)ros_->cancelNavigation();
    });
    connect(globalCostmapCheck_,&QCheckBox::toggled,this,[this](bool on){
      if(globalCostmapDisplay_)globalCostmapDisplay_->setEnabled(on);
    });
    connect(localCostmapCheck_,&QCheckBox::toggled,this,[this](bool on){
      if(localCostmapDisplay_)localCostmapDisplay_->setEnabled(on);
    });
    connect(footprintCheck_,&QCheckBox::toggled,this,[this](bool on){
      if(footprintDisplay_)footprintDisplay_->setEnabled(on);
    });
    connect(mppiCheck_,&QCheckBox::toggled,this,[this](bool on){
      if(mppiDisplay_)mppiDisplay_->setEnabled(on);
    });

    statusTimer_=new QTimer(this);
    statusTimer_->setInterval(500);
    connect(statusTimer_,&QTimer::timeout,this,[this](){updateStatus();});
    statusTimer_->start();
  }

  ~EmbeddedNav2Panel()override{
    teardownRviz();
  }

  void setActive(bool active){
    active_=active;
    if(active_)ensureInitialized();
    setRendering(active_&&initialized_);
  }

  private:WorkspacePaths paths_;
  const QMap<QString,std::shared_ptr<YamlStore>>&stores_;
  TelemetryStore*telemetry_;
  RosBridge*ros_;
  QFrame*renderHost_=nullptr;
  QVBoxLayout*renderLayout_=nullptr;
  QLabel*placeholder_=nullptr;
  QLabel*status_=nullptr;
  QPushButton*moveButton_=nullptr;
  QPushButton*goalButton_=nullptr;
  QPushButton*initialPoseButton_=nullptr;
  QCheckBox*globalCostmapCheck_=nullptr;
  QCheckBox*localCostmapCheck_=nullptr;
  QCheckBox*footprintCheck_=nullptr;
  QCheckBox*mppiCheck_=nullptr;
  QTimer*statusTimer_=nullptr;

  rviz_common::RenderPanel*renderPanel_=nullptr;
  rviz_common::VisualizationManager*manager_=nullptr;
  rviz_common::Display*mapDisplay_=nullptr;
  rviz_common::Display*globalCostmapDisplay_=nullptr;
  rviz_common::Display*localCostmapDisplay_=nullptr;
  rviz_common::Display*robotDisplay_=nullptr;
  rviz_common::Display*globalPathDisplay_=nullptr;
  rviz_common::Display*localPathDisplay_=nullptr;
  rviz_common::Display*mppiDisplay_=nullptr;
  rviz_common::Display*footprintDisplay_=nullptr;
  rviz_common::Display*goalDisplay_=nullptr;
  rviz_common::Tool*moveTool_=nullptr;
  rviz_common::Tool*goalTool_=nullptr;
  rviz_common::Tool*initialPoseTool_=nullptr;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction>rvizNodeOwner_;
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rvizNodeWeak_;
  rclcpp::Node::SharedPtr rvizNode_;
  bool active_=false;
  bool initialized_=false;
  bool rendering_=false;
  bool failed_=false;
  QStringList displayWarnings_;

  void ensureInitialized(){
    if(initialized_||failed_||!active_)return;
    try{
      status_->setText(QStringLiteral("Menginisialisasi RViz/OGRE..."));
      QApplication::processEvents();

      rvizNodeOwner_=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
        "agv_gui_embedded_rviz");
      rvizNodeWeak_=rvizNodeOwner_;
      rvizNode_=rvizNodeOwner_->get_raw_node();
      if(!rvizNode_)throw std::runtime_error("RViz ROS node tidak tersedia");

      renderPanel_=new rviz_common::RenderPanel(renderHost_);
      renderPanel_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
      renderLayout_->addWidget(renderPanel_,1);
      placeholder_->hide();
      QApplication::processEvents();
      renderPanel_->getRenderWindow()->initialize();

      rviz_common::WindowManagerInterface*windowManager=nullptr;
      manager_=new rviz_common::VisualizationManager(
        renderPanel_,rvizNodeWeak_,windowManager,rvizNode_->get_clock());
      renderPanel_->initialize(manager_);
      manager_->initialize();
      manager_->setFixedFrame(QStringLiteral("map"));
      configureGlobalOptions();
      configureDisplays();
      configureTools();
      configureView();

      // PENTING: VisualizationManager Humble sudah memiliki executor internal
      // dan menambahkan rvizNode_ sendiri. Jangan add_node() ke executor kedua;
      // itu menghasilkan: "Node ... has already been added to an executor".
      // Callback RViz diproses oleh VisualizationManager::onUpdate()->spin_some().

      initialized_=true;
      setRendering(active_);
      selectTool(moveTool_,moveButton_);
      QTimer::singleShot(200,this,[this](){fitFullMap();});
      updateStatus();
    }catch(const std::exception&e){
      const QString error=QString::fromUtf8(e.what());
      teardownRviz();
      failed_=true;
      placeholder_->show();
      placeholder_->setText(QStringLiteral(
        "Embedded RViz gagal dimuat\n%1\n\nGunakan subtab Ground Truth / PGM.\n"
        "Periksa OpenGL dan paket ros-humble-rviz2.").arg(error));
      status_->setText(QStringLiteral("RViz ERROR: ")+error);
      status_->setStyleSheet(QStringLiteral("color:#ff6b6b;font-weight:700;"));
    }
  }

  void configureGlobalOptions(){
    if(!manager_)return;
    auto*root=manager_->getRootDisplayGroup();
    auto*global=root?root->subProp(QStringLiteral("Global Options")):nullptr;
    if(!global)return;
    if(auto*p=global->subProp(QStringLiteral("Background Color")))p->setValue(QColor(22,24,28));
    // Ten frames per second is adequate for an outdoor low-speed AGV map and
    // materially lowers OGRE load compared with RViz's desktop default.
    if(auto*p=global->subProp(QStringLiteral("Frame Rate")))p->setValue(10);
  }

  rviz_common::Display*addDisplay(
    const QString&classId,const QString&name,bool enabled,bool required=true){
    rviz_common::Display*display=manager_->createDisplay(classId,name,enabled);
    if(!display){
      const QString warning=QStringLiteral("%1 (%2)").arg(name,classId);
      displayWarnings_<<warning;
      if(required)throw std::runtime_error(
        (QStringLiteral("Plugin RViz gagal: ")+warning).toStdString());
    }
    return display;
  }

  static rviz_common::properties::Property*property(
    rviz_common::Display*display,const QString&name){
    return display?display->subProp(name):nullptr;
  }

  static void setProperty(
    rviz_common::Display*display,const QString&name,const QVariant&value){
    if(auto*p=property(display,name))p->setValue(value);
  }

  static void setTopic(
    rviz_common::Display*display,const QString&topic,
    const QString&reliability=QStringLiteral("Reliable"),
    const QString&durability=QStringLiteral("Volatile")){
    auto*topicProperty=property(display,QStringLiteral("Topic"));
    if(!topicProperty)return;
    topicProperty->setValue(topic);
    if(auto*p=topicProperty->subProp(QStringLiteral("Depth")))p->setValue(1);
    if(auto*p=topicProperty->subProp(QStringLiteral("Reliability Policy")))p->setValue(reliability);
    if(auto*p=topicProperty->subProp(QStringLiteral("Durability Policy")))p->setValue(durability);
  }

  static void setRosTopicProperty(
    rviz_common::Display*display,const QString&propertyName,const QString&topic,
    const QString&reliability=QStringLiteral("Reliable"),
    const QString&durability=QStringLiteral("Volatile")){
    auto*topicProperty=property(display,propertyName);
    if(!topicProperty)return;
    topicProperty->setValue(topic);
    if(auto*p=topicProperty->subProp(QStringLiteral("Depth")))p->setValue(1);
    if(auto*p=topicProperty->subProp(QStringLiteral("Reliability Policy")))p->setValue(reliability);
    if(auto*p=topicProperty->subProp(QStringLiteral("Durability Policy")))p->setValue(durability);
  }

  void configureDisplays(){
    auto*grid=addDisplay(QStringLiteral("rviz_default_plugins/Grid"),QStringLiteral("Grid"),true,false);
    setProperty(grid,QStringLiteral("Alpha"),0.28);
    setProperty(grid,QStringLiteral("Cell Size"),1.0);

    mapDisplay_=addDisplay(QStringLiteral("rviz_default_plugins/Map"),QStringLiteral("MAP /map"),true);
    setTopic(mapDisplay_,QStringLiteral("/map"),QStringLiteral("Reliable"),QStringLiteral("Transient Local"));
    setRosTopicProperty(mapDisplay_,QStringLiteral("Update Topic"),QStringLiteral("/map_updates"));
    setProperty(mapDisplay_,QStringLiteral("Alpha"),0.90);
    setProperty(mapDisplay_,QStringLiteral("Color Scheme"),QStringLiteral("map"));
    setProperty(mapDisplay_,QStringLiteral("Draw Behind"),true);

    // The 6200x4000 UNDIP costmaps are useful for tuning but expensive to draw.
    // They are created disabled and only subscribe after the operator checks them.
    globalCostmapDisplay_=addDisplay(
      QStringLiteral("rviz_default_plugins/Map"),QStringLiteral("Global Costmap"),false,false);
    setTopic(globalCostmapDisplay_,QStringLiteral("/global_costmap/costmap"),
      QStringLiteral("Reliable"),QStringLiteral("Transient Local"));
    setRosTopicProperty(globalCostmapDisplay_,QStringLiteral("Update Topic"),
      QStringLiteral("/global_costmap/costmap_updates"));
    setProperty(globalCostmapDisplay_,QStringLiteral("Alpha"),0.48);
    setProperty(globalCostmapDisplay_,QStringLiteral("Color Scheme"),QStringLiteral("costmap"));

    localCostmapDisplay_=addDisplay(
      QStringLiteral("rviz_default_plugins/Map"),QStringLiteral("Local Costmap"),false,false);
    setTopic(localCostmapDisplay_,QStringLiteral("/local_costmap/costmap"),
      QStringLiteral("Reliable"),QStringLiteral("Transient Local"));
    setRosTopicProperty(localCostmapDisplay_,QStringLiteral("Update Topic"),
      QStringLiteral("/local_costmap/costmap_updates"));
    setProperty(localCostmapDisplay_,QStringLiteral("Alpha"),0.62);
    setProperty(localCostmapDisplay_,QStringLiteral("Color Scheme"),QStringLiteral("costmap"));

    robotDisplay_=addDisplay(
      QStringLiteral("rviz_default_plugins/RobotModel"),QStringLiteral("URDF Kendaraan"),true);
    setProperty(robotDisplay_,QStringLiteral("Description Source"),QStringLiteral("Topic"));
    // robot_state_publisher menerbitkan /robot_description secara transient-local.
    // Embedded RViz dibuat lazy (setelah publisher sudah hidup), jadi QoS ini wajib
    // supaya URDF tetap diterima saat subscriber terlambat dibuat.
    setRosTopicProperty(robotDisplay_,QStringLiteral("Description Topic"),
      QStringLiteral("/robot_description"),QStringLiteral("Reliable"),QStringLiteral("Transient Local"));
    setProperty(robotDisplay_,QStringLiteral("Visual Enabled"),true);
    setProperty(robotDisplay_,QStringLiteral("Collision Enabled"),false);

    globalPathDisplay_=addDisplay(
      QStringLiteral("rviz_default_plugins/Path"),QStringLiteral("Smac Hybrid-A* Global Path"),true);
    setTopic(globalPathDisplay_,QStringLiteral("/plan"));
    setProperty(globalPathDisplay_,QStringLiteral("Color"),QColor(0,255,70));
    setProperty(globalPathDisplay_,QStringLiteral("Line Width"),0.10);
    setProperty(globalPathDisplay_,QStringLiteral("Buffer Length"),1);

    localPathDisplay_=addDisplay(
      QStringLiteral("rviz_default_plugins/Path"),QStringLiteral("MPPI Transformed Plan"),true);
    setTopic(localPathDisplay_,QStringLiteral("/controller_server/transformed_global_plan"));
    setProperty(localPathDisplay_,QStringLiteral("Color"),QColor(255,180,0));
    setProperty(localPathDisplay_,QStringLiteral("Line Width"),0.06);
    setProperty(localPathDisplay_,QStringLiteral("Buffer Length"),1);

    mppiDisplay_=addDisplay(
      QStringLiteral("rviz_default_plugins/MarkerArray"),QStringLiteral("Kandidat Trajectory MPPI"),true);
    setTopic(mppiDisplay_,QStringLiteral("/trajectories"));

    footprintDisplay_=addDisplay(
      QStringLiteral("rviz_default_plugins/Polygon"),QStringLiteral("Robot Footprint"),true,false);
    setTopic(footprintDisplay_,QStringLiteral("/local_costmap/published_footprint"));
    setProperty(footprintDisplay_,QStringLiteral("Color"),QColor(255,90,0));
    setProperty(footprintDisplay_,QStringLiteral("Alpha"),1.0);

    goalDisplay_=addDisplay(
      QStringLiteral("rviz_default_plugins/Pose"),QStringLiteral("Goal Pose"),true,false);
    setTopic(goalDisplay_,QStringLiteral("/navigation/goal_request"));
    setProperty(goalDisplay_,QStringLiteral("Color"),QColor(255,70,90));

    if(globalCostmapDisplay_)globalCostmapDisplay_->setEnabled(globalCostmapCheck_->isChecked());
    if(localCostmapDisplay_)localCostmapDisplay_->setEnabled(localCostmapCheck_->isChecked());
    if(footprintDisplay_)footprintDisplay_->setEnabled(footprintCheck_->isChecked());
    if(mppiDisplay_)mppiDisplay_->setEnabled(mppiCheck_->isChecked());
  }

  void configureTools(){
    auto*toolManager=manager_->getToolManager();
    if(!toolManager)throw std::runtime_error("RViz ToolManager tidak tersedia");
    moveTool_=toolManager->addTool(QStringLiteral("rviz_default_plugins/MoveCamera"));
    goalTool_=toolManager->addTool(QStringLiteral("rviz_default_plugins/SetGoal"));
    initialPoseTool_=toolManager->addTool(QStringLiteral("rviz_default_plugins/SetInitialPose"));
    if(!moveTool_||!goalTool_||!initialPoseTool_){
      throw std::runtime_error("Tool Move/Goal/InitialPose gagal dimuat");
    }
    if(auto*p=goalTool_->getPropertyContainer()->subProp(QStringLiteral("Topic"))){
      p->setValue(QStringLiteral("/navigation/goal_request"));
    }
    if(auto*p=initialPoseTool_->getPropertyContainer()->subProp(QStringLiteral("Topic"))){
      p->setValue(QStringLiteral("/initialpose"));
    }
  }

  void configureView(){
    auto*viewManager=manager_->getViewManager();
    if(!viewManager)throw std::runtime_error("RViz ViewManager tidak tersedia");
    viewManager->setCurrentViewControllerType(QStringLiteral("rviz_default_plugins/TopDownOrtho"));
    if(auto*view=viewManager->getCurrent()){
      if(auto*p=view->subProp(QStringLiteral("Target Frame")))p->setValue(QStringLiteral("map"));
      if(auto*p=view->subProp(QStringLiteral("Angle")))p->setValue(0.0);
    }
  }

  void selectTool(rviz_common::Tool*tool,QPushButton*selected){
    if(!initialized_){
      ensureInitialized();
      if(!initialized_)return;
    }
    if(!tool||!manager_||!manager_->getToolManager())return;
    manager_->getToolManager()->setCurrentTool(tool);
    for(auto*button:{moveButton_,goalButton_,initialPoseButton_})button->setChecked(button==selected);
  }

  void setRendering(bool on){
    if(!manager_||rendering_==on)return;
    if(on)manager_->startUpdate();
    else manager_->stopUpdate();
    rendering_=on;
  }

  void setTopDownView(double x,double y,double scale){
    if(!manager_||!manager_->getViewManager())return;
    auto*view=manager_->getViewManager()->getCurrent();
    if(!view)return;
    if(auto*p=view->subProp(QStringLiteral("X")))p->setValue(x);
    if(auto*p=view->subProp(QStringLiteral("Y")))p->setValue(y);
    if(auto*p=view->subProp(QStringLiteral("Scale")))p->setValue(scale);
  }

  void fitFullMap(){
    if(!initialized_||!renderPanel_)return;
    QImageReader reader(paths_.fileMap().value(QStringLiteral("map_pgm")));
    const QSize pixels=reader.size();
    double resolution=0.1;
    double originX=0.0,originY=0.0,originYaw=0.0;
    const auto mapStore=stores_.value(QStringLiteral("map"));
    if(mapStore){
      resolution=number(mapStore->get(QStringLiteral("resolution"),resolution),resolution);
      const QVariantList origin=mapStore->get(QStringLiteral("origin"),QVariantList{0.0,0.0,0.0}).toList();
      if(origin.size()>=3){
        originX=number(origin[0]);
        originY=number(origin[1]);
        originYaw=number(origin[2]);
      }
    }
    if(!pixels.isValid()||resolution<=0.0){
      setTopDownView(0.0,0.0,10.0);
      return;
    }
    const double widthM=pixels.width()*resolution;
    const double heightM=pixels.height()*resolution;
    const double c=std::cos(originYaw),s=std::sin(originYaw);
    const double centerX=originX+c*widthM*0.5-s*heightM*0.5;
    const double centerY=originY+s*widthM*0.5+c*heightM*0.5;
    const double scaleX=renderPanel_->width()/std::max(1.0,widthM);
    const double scaleY=renderPanel_->height()/std::max(1.0,heightM);
    setTopDownView(centerX,centerY,std::max(0.05,0.88*std::min(scaleX,scaleY)));
  }

  void focusRobot(){
    if(!telemetry_)return;
    const double x=number(telemetry_->get(QStringLiteral("localization_state.map_x")),NAN);
    const double y=number(telemetry_->get(QStringLiteral("localization_state.map_y")),NAN);
    if(!std::isfinite(x)||!std::isfinite(y)){
      status_->setText(QStringLiteral("Pose map kendaraan belum valid; tunggu localization ready."));
      return;
    }
    setTopDownView(x,y,12.0);
  }

  void updateStatus(){
    if(failed_||!status_)return;
    if(!initialized_){
      status_->setText(active_?QStringLiteral("Menunggu RViz..."):QStringLiteral("RViz standby (lazy)"));
      return;
    }
    const bool navReady=telemetry_&&telemetry_->get(QStringLiteral("system.nav2_ready"),false).toBool();
    const QVariantMap global=telemetry_?telemetry_->get(QStringLiteral("nav_path")).toMap():QVariantMap{};
    const QVariantMap local=telemetry_?telemetry_->get(QStringLiteral("local_path")).toMap():QVariantMap{};
    const QString globalText=global.isEmpty()?QStringLiteral("WAIT"):QStringLiteral("%1 pts").arg(global.value(QStringLiteral("count")).toLongLong());
    const QString localText=local.isEmpty()?QStringLiteral("WAIT"):QStringLiteral("%1 pts").arg(local.value(QStringLiteral("count")).toLongLong());
    const bool mppiVisual=stores_.value(QStringLiteral("nav2"))&&
      stores_.value(QStringLiteral("nav2"))->get(
        QStringLiteral("controller_server.ros__parameters.FollowPath.visualize"),false).toBool();
    QString text=QStringLiteral("NAV2 %1  |  A* %2  |  MPPI plan %3  |  candidates %4")
      .arg(navReady?QStringLiteral("READY"):QStringLiteral("WAIT"),globalText,localText,
        mppiVisual&&mppiCheck_->isChecked()?QStringLiteral("ON"):QStringLiteral("OFF"));
    if(!displayWarnings_.isEmpty())text+=QStringLiteral("  |  optional display unavailable: ")+displayWarnings_.join(QStringLiteral(", "));
    status_->setText(text);
    status_->setStyleSheet(QStringLiteral("color:%1;font-weight:700;").arg(navReady?kGreen:kGold));
  }

  void teardownRviz(){
    // VisualizationManager memiliki executor RViz internal. Hentikan update dan
    // hapus manager selagi RenderPanel masih hidup, baru lepaskan panel/ROS node.
    if(manager_){
      manager_->stopUpdate();
      manager_->removeAllDisplays();
      delete manager_;
      manager_=nullptr;
    }
    if(renderPanel_){
      renderLayout_->removeWidget(renderPanel_);
      delete renderPanel_;
      renderPanel_=nullptr;
    }
    rvizNode_.reset();
    rvizNodeWeak_.reset();
    rvizNodeOwner_.reset();
    mapDisplay_=globalCostmapDisplay_=localCostmapDisplay_=robotDisplay_=nullptr;
    globalPathDisplay_=localPathDisplay_=mppiDisplay_=footprintDisplay_=goalDisplay_=nullptr;
    moveTool_=goalTool_=initialPoseTool_=nullptr;
    initialized_=false;
    rendering_=false;
  }
};

class NavigationMapPage:public QWidget{
  public:NavigationMapPage(
    const WorkspacePaths&paths,
    const QMap<QString,std::shared_ptr<YamlStore>>&stores,
    TelemetryStore*telemetry,
    ReportManager*reports,
    RosBridge*ros,
    QWidget*parent=nullptr)
  :QWidget(parent),paths_(paths),stores_(stores),telemetry_(telemetry),reports_(reports),ros_(ros){
    auto*layout=new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    tabs_=new QTabWidget();
    tabs_->setDocumentMode(true);
    nav2_=new EmbeddedNav2Panel(paths_,stores_,telemetry_,ros_);
    tabs_->addTab(nav2_,QStringLiteral("NAV2 LIVE • MAP / URDF / A* / MPPI"));

    legacyHolder_=new QWidget();
    legacyLayout_=new QVBoxLayout(legacyHolder_);
    legacyPlaceholder_=new QLabel(QStringLiteral(
      "Peta PGM/OSM, tabel target, dan ground truth dimuat saat subtab ini dibuka."));
    legacyPlaceholder_->setAlignment(Qt::AlignCenter);
    legacyPlaceholder_->setStyleSheet(QStringLiteral("color:#aab0ba;font-size:14px;"));
    legacyLayout_->addWidget(legacyPlaceholder_,1);
    tabs_->addTab(legacyHolder_,QStringLiteral("GROUND TRUTH • PGM / OSM / TARGET"));
    layout->addWidget(tabs_);

    connect(tabs_,&QTabWidget::currentChanged,this,[this](int index){
      if(index==1)ensureLegacyMap();
      nav2_->setActive(active_&&index==0);
    });
  }

  void setActive(bool active){
    active_=active;
    if(active_&&tabs_->currentIndex()==1)ensureLegacyMap();
    nav2_->setActive(active_&&tabs_->currentIndex()==0);
  }

  void refreshTargets(){
    if(legacyMap_)legacyMap_->refreshTargets();
  }

  private:WorkspacePaths paths_;
  const QMap<QString,std::shared_ptr<YamlStore>>&stores_;
  TelemetryStore*telemetry_;
  ReportManager*reports_;
  RosBridge*ros_;
  QTabWidget*tabs_=nullptr;
  EmbeddedNav2Panel*nav2_=nullptr;
  QWidget*legacyHolder_=nullptr;
  QVBoxLayout*legacyLayout_=nullptr;
  QLabel*legacyPlaceholder_=nullptr;
  MapPage*legacyMap_=nullptr;
  bool active_=false;

  void ensureLegacyMap(){
    if(legacyMap_)return;
    legacyLayout_->removeWidget(legacyPlaceholder_);
    delete legacyPlaceholder_;
    legacyPlaceholder_=nullptr;
    legacyMap_=new MapPage(paths_,stores_,telemetry_,reports_,ros_,legacyHolder_);
    legacyLayout_->addWidget(legacyMap_,1);
  }
};
