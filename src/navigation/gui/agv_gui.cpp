#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <functional>
#include <memory>
#include <tuple>
#include <signal.h>
#include <sys/types.h>
#include <QAbstractItemView>
#include <QByteArray>
#include <QChar>
#include <QTreeWidget>
#include <QFont>
#include <QIODevice>
#include <QLineF>
#include <QListWidgetItem>
#include <QObject>
#include <QPaintEvent>
#include <QPair>
#include <QPoint>
#include <QRect>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QApplication>
#include <QGraphicsSceneMouseEvent>
#include <QInputDialog>
#include <QListWidget>
#include <QMetaType>
#include <QMutex>
#include <QMutexLocker>
#include <QNetworkRequest>
#include <QRegularExpression>
#include <QResizeEvent>
#include <QCheckBox>
#include <QCloseEvent>
#include <QColor>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDirIterator>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGraphicsEllipseItem>
#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QHash>
#include <QImage>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMap>
#include <QMessageBox>
#include <QMetaObject>
#include <QMouseEvent>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPlainTextEdit>
#include <QPointF>
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSet>
#include <QScreen>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QTreeWidgetItemIterator>
#include <QTextStream>
#include <QTimer>
#include <QToolButton>
#include <QTransform>
#include <QUrl>
#include <QUrlQuery>
#include <QVariant>
#include <QVariantList>
#include <QVariantMap>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>
#include <QXmlStreamReader>
#include <yaml-cpp/yaml.h>
#include <action_msgs/srv/cancel_goal.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_rendering/render_window.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "agv_gui_specs.hpp"
#include "agv_experiment_catalog.hpp"
namespace fs = std::filesystem;
using namespace std::chrono_literals;
namespace {
  #include "modules/gui_core.cpp"
  class SettingsForm:public QWidget{
    Q_OBJECT
    public:
    SettingsForm(const QString &title,const QVector<SettingSpec>&specs,const QMap<QString,std::shared_ptr<YamlStore>>&stores,QWidget*parent=nullptr):QWidget(parent),specs_(specs),stores_(stores){
      auto *lay=new QVBoxLayout(this);
      lay->setContentsMargins(10,10,10,10);
      auto *h=new QLabel(title);
      h->setObjectName("settingsTitle");
      lay->addWidget(h);
      QString current;
      QFormLayout *form=nullptr;
      for(const auto&s:specs_){
        if(s.group!=current){
          current=s.group;
          auto*g=new QGroupBox(current);
          form=new QFormLayout(g);
          form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
          lay->addWidget(g);
        }
        QWidget*w=createWidget(s);
        widgets_[s.fileKey+"|"+s.path]=w;
        form->addRow(s.label+s.suffix,w);
      }
      lay->addStretch();
      reloadValues();
    }
    QSet<QString> settingKeys()const{
      QSet<QString>s;
      for(const auto&x:specs_)s.insert(x.fileKey+"|"+x.path);
      return s;
    }
    void reloadValues(){
      for(const auto&s:specs_)loadOne(s);
    }
    void reloadKeys(const QSet<QString>&keys){
      for(const auto&s:specs_)if(keys.contains(s.fileKey+"|"+s.path))loadOne(s);
    }
    signals:void valueEdited(QString fileKey,QString path,QVariant value);
    private:
    QVector<SettingSpec> specs_;
    QMap<QString,std::shared_ptr<YamlStore>> stores_;
    QHash<QString,QWidget*>widgets_;
    QWidget*createWidget(const SettingSpec&s){
      QWidget*w=nullptr;
      if(s.kind=="bool"){
        auto*c=new QCheckBox();
        connect(c,&QCheckBox::toggled,this,[this,s](bool v){
          emit valueEdited(s.fileKey,s.path,v);
        });
        w=c;
      }
      else if(s.kind=="int"){
        auto*x=new NoWheelSpinBox();
        x->setRange(static_cast<int>(std::max(-2147483647.0,s.min)),static_cast<int>(std::min(2147483647.0,s.max)));
        x->setSingleStep(std::max(1,static_cast<int>(s.step)));
        connect(x,qOverload<int>(&QSpinBox::valueChanged),this,[this,s](int v){
          emit valueEdited(s.fileKey,s.path,v);
        });
        w=x;
      }
      else if(s.kind=="choice"){
        auto*x=new NoWheelComboBox();
        x->addItems(s.choices);
        connect(x,&QComboBox::currentTextChanged,this,[this,s](const QString&v){
          emit valueEdited(s.fileKey,s.path,v);
        });
        w=x;
      }
      else if(s.kind=="float"){
        auto*x=new NoWheelDoubleSpinBox();
        x->setDecimals(s.decimals);
        x->setRange(s.min,s.max);
        x->setSingleStep(s.step);
        connect(x,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this,s](double v){
          emit valueEdited(s.fileKey,s.path,v);
        });
        w=x;
      }
      else {
        auto*x=new QLineEdit();
        connect(x,&QLineEdit::editingFinished,this,[this,s,x](){
          emit valueEdited(s.fileKey,s.path,s.kind=="list"?parseEditorList(x->text()):QVariant(x->text()));
        });
        w=x;
      }
      w->setToolTip(s.tip);
      return w;
    }
    void loadOne(const SettingSpec&s){
      auto it=widgets_.find(s.fileKey+"|"+s.path);
      if(it==widgets_.end()||!stores_.contains(s.fileKey))return;
      const QVariant v=stores_[s.fileKey]->get(s.path);
      QSignalBlocker b(it.value());
      if(auto*x=qobject_cast<QCheckBox*>(it.value()))x->setChecked(v.toBool());
      else if(auto*x=qobject_cast<QSpinBox*>(it.value()))x->setValue(v.toInt());
      else if(auto*x=qobject_cast<QDoubleSpinBox*>(it.value()))x->setValue(v.toDouble());
      else if(auto*x=qobject_cast<QComboBox*>(it.value())){
        int i=x->findText(v.toString());
        if(i>=0)x->setCurrentIndex(i);
        else{
          x->addItem(v.toString());
          x->setCurrentText(v.toString());
        }
      }
      else if(auto*x=qobject_cast<QLineEdit*>(it.value())){
        if(v.userType()==QMetaType::QVariantList)x->setText(QString::fromUtf8(QJsonDocument(QJsonArray::fromVariantList(v.toList())).toJson(QJsonDocument::Compact)));
        else x->setText(v.toString());
      }
    }
  };
  #include "modules/reporting_widgets.cpp"
  class RosBridge:public QObject{
    Q_OBJECT
    public:explicit RosBridge(QObject*p=nullptr):QObject(p){
    }
    ~RosBridge()override{
      shutdown();
    }
    void start(){
      if(thread_.joinable())return;
      stop_=false;
      thread_=std::thread([this](){
        run();
      });
    }
    void shutdown(){
      stop_=true;
      if(executor_)executor_->cancel();
      if(thread_.joinable())thread_.join();
      std::lock_guard<std::mutex> lock(subscriptionsMutex_);
      subscriptions_.clear();
    }
    bool publishGoal(double x,double y,double yaw){
      std::lock_guard<std::mutex>lk(mu_);
      if(!node_||!goalPub_)return false;
      geometry_msgs::msg::PoseStamped m;
      m.header.stamp=node_->now();
      m.header.frame_id="map";
      m.pose.position.x=x;
      m.pose.position.y=y;
      m.pose.orientation.z=std::sin(yaw/2.0);
      m.pose.orientation.w=std::cos(yaw/2.0);
      goalPub_->publish(m);
      {
        std::lock_guard<std::mutex>goalLock(goalMutex_);
        goalStarted_=std::chrono::steady_clock::now();
        goalActive_=true;
        firstPlanSeen_=false;
        lastPlanningLatencyMs_=std::numeric_limits<double>::quiet_NaN();
      }
      emitMap("goal_pose",{
        {
          "x",x
        },{
          "y",y
        },{
          "yaw",yaw
        }
      });
      return true;
    }
    bool publishGroundTruth(double x,double y,double yaw){
      std::lock_guard<std::mutex>lk(mu_);
      if(!node_||!initialPub_)return false;
      geometry_msgs::msg::PoseWithCovarianceStamped m;
      m.header.stamp=node_->now();
      m.header.frame_id="map";
      m.pose.pose.position.x=x;
      m.pose.pose.position.y=y;
      m.pose.pose.orientation.z=std::sin(yaw/2.0);
      m.pose.pose.orientation.w=std::cos(yaw/2.0);
      m.pose.covariance[0]=0.01;
      m.pose.covariance[7]=0.01;
      m.pose.covariance[35]=std::pow(kPi/180.0,2);
      initialPub_->publish(m);
      return true;
    }
    void callTrigger(const QString&service,const QString&tag={
    }){
      auto n=nodeCopy();
      if(!n){
        emit serviceResult(tag.isEmpty()?service:tag,false,"ROS node belum aktif");
        return;
      }
      auto c=n->create_client<std_srvs::srv::Trigger>(service.toStdString());
      if(!c->wait_for_service(300ms)){
        emit serviceResult(tag.isEmpty()?service:tag,false,"Service belum tersedia");
        return;
      }
      auto req=std::make_shared<std_srvs::srv::Trigger::Request>();
      c->async_send_request(req,[this,c,service,tag](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
        try{
          auto r=f.get();
          emit serviceResult(tag.isEmpty()?service:tag,r->success,QString::fromStdString(r->message));
        }
        catch(const std::exception&e){
          emit serviceResult(tag.isEmpty()?service:tag,false,e.what());
        }
      });
    }
    void setParametersAtomically(const QString&nodeName,const QVariantMap&params,const QString&tag){
      auto n=nodeCopy();
      if(!n){
        emit serviceResult(tag,false,"ROS node belum aktif");
        return;
      }
      const QString service="/"+nodeName.trimmed().remove(QRegularExpression("^/"))+"/set_parameters_atomically";
      auto c=n->create_client<rcl_interfaces::srv::SetParametersAtomically>(service.toStdString());
      if(!c->wait_for_service(600ms)){
        emit serviceResult(tag,false,"Parameter service belum tersedia");
        return;
      }
      auto req=std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
      for(auto it=params.cbegin();
      it!=params.cend();
      ++it){
        rcl_interfaces::msg::Parameter p;
        p.name=it.key().toStdString();
        auto&v=p.value;
        const QVariant x=it.value();
        if(x.userType()==QMetaType::Bool){
          v.type=rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
          v.bool_value=x.toBool();
        }
        else if(x.userType()==QMetaType::Int||x.userType()==QMetaType::LongLong){
          v.type=rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
          v.integer_value=x.toLongLong();
        }
        else if(x.type()==QVariant::List){
          const QVariantList list=x.toList();
          bool numeric=true;
          for(const QVariant&item:list){
            if(!item.canConvert<double>()||item.userType()==QMetaType::QString){
              numeric=false;
              break;
            }
          }
          if(numeric){
            v.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
            for(const QVariant&item:list)v.double_array_value.push_back(item.toDouble());
          }
          else{
            v.type=rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
            for(const QVariant&item:list)v.string_array_value.push_back(item.toString().toStdString());
          }
        }
        else if(x.canConvert<double>()&&x.userType()!=QMetaType::QString){
          v.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          v.double_value=x.toDouble();
        }
        else{
          v.type=rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
          v.string_value=x.toString().toStdString();
        }
        req->parameters.push_back(p);
      }
      c->async_send_request(req,[this,c,tag](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture f){
        try{
          auto r=f.get();
          emit serviceResult(tag,r->result.successful,QString::fromStdString(r->result.reason));
        }
        catch(const std::exception&e){
          emit serviceResult(tag,false,e.what());
        }
      });
    }
    void getParameters(const QString&nodeName,const QStringList&names,const QString&tag){
      auto n=nodeCopy();
      if(!n){
        emit parametersResult(tag,false,QVariantMap{
          {
            "error","ROS node belum aktif"
          }
        });
        return;
      }
      const QString service="/"+QString(nodeName).remove(QRegularExpression("^/"))+"/get_parameters";
      auto c=n->create_client<rcl_interfaces::srv::GetParameters>(service.toStdString());
      if(!c->wait_for_service(600ms)){
        emit parametersResult(tag,false,QVariantMap{
          {
            "error","Parameter service belum tersedia"
          }
        });
        return;
      }
      auto req=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      for(const auto&s:names)req->names.push_back(s.toStdString());
      c->async_send_request(req,[this,c,tag,names](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture f){
        try{
          auto r=f.get();
          QVariantMap m;
          for(int i=0;
          i<std::min<int>(names.size(),r->values.size());
          ++i)m[names[i]]=parameterValue(r->values[i]);
          emit parametersResult(tag,true,m);
        }
        catch(const std::exception&e){
          emit parametersResult(tag,false,QVariantMap{
            {
              "error",e.what()
            }
          });
        }
      });
    }
    void cancelNavigation(){
      auto n=nodeCopy();
      if(!n){
        emit serviceResult("Cancel navigation",false,"ROS node belum aktif");
        return;
      }
      auto c=n->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
      if(!c->wait_for_service(400ms)){
        emit serviceResult("Cancel navigation",false,"Service cancel Nav2 belum tersedia");
        return;
      }
      auto req=std::make_shared<action_msgs::srv::CancelGoal::Request>();
      c->async_send_request(req,[this,c](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture f){
        try{
          auto r=f.get();
          emit serviceResult("Cancel navigation",r->return_code==0||!r->goals_canceling.empty(),QString("return_code=%1, goals=%2").arg(r->return_code).arg(r->goals_canceling.size()));
        }
        catch(const std::exception&e){
          emit serviceResult("Cancel navigation",false,e.what());
        }
      });
    }
    signals:void telemetry(QString channel,QVariant data);
    void image(QImage image);
    void perceptionMask(QString channel,QImage image);
    void ready(bool ok,QString message);
    void serviceResult(QString tag,bool ok,QString message);
    void parametersResult(QString tag,bool ok,QVariantMap values);
    private:std::thread thread_;
    std::atomic_bool stop_{
      false
    };
    std::mutex mu_;
    std::mutex subscriptionsMutex_;
    std::mutex goalMutex_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>executor_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPub_;
    std::chrono::steady_clock::time_point goalStarted_{
    };
    bool goalActive_{
      false
    };
    bool firstPlanSeen_{
      false
    };
    double lastPlanningLatencyMs_{
      std::numeric_limits<double>::quiet_NaN()
    };
    rclcpp::Node::SharedPtr nodeCopy(){
      std::lock_guard<std::mutex>lk(mu_);
      return node_;
    }
    static QVariant parameterValue(const rcl_interfaces::msg::ParameterValue&v){
      using PT=rcl_interfaces::msg::ParameterType;
      switch(v.type){
        case PT::PARAMETER_BOOL:return v.bool_value;
        case PT::PARAMETER_INTEGER:return QVariant::fromValue<qlonglong>(v.integer_value);
        case PT::PARAMETER_DOUBLE:return v.double_value;
        case PT::PARAMETER_STRING:return QString::fromStdString(v.string_value);
        case PT::PARAMETER_DOUBLE_ARRAY:{
          QVariantList l;
          for(double x:v.double_array_value)l<<x;
          return l;
        }
        case PT::PARAMETER_INTEGER_ARRAY:{
          QVariantList l;
          for(auto x:v.integer_array_value)l<<QVariant::fromValue<qlonglong>(x);
          return l;
        }
        case PT::PARAMETER_STRING_ARRAY:{
          QVariantList l;
          for(auto&x:v.string_array_value)l<<QString::fromStdString(x);
          return l;
        }
        default:return {
        };
      }
    }
    template<class Msg,class Callback>void sub(const rclcpp::Node::SharedPtr&n,const std::string&topic,const rclcpp::QoS&q,Callback cb){
      auto subscription=n->create_subscription<Msg>(topic,q,cb);
      std::lock_guard<std::mutex> lock(subscriptionsMutex_);
      subscriptions_.push_back(subscription);
    }
    void emitMap(const QString&ch,const QVariantMap&m){
      emit telemetry(ch,m);
    }
    void run(){
      try{
        auto n=std::make_shared<rclcpp::Node>("agv_gui");
        auto ex=std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),2);
        ex->add_node(n);
        {
          std::lock_guard<std::mutex>lk(mu_);
          node_=n;
          executor_=ex;
        }
        auto state=rclcpp::QoS(1).reliable();
        auto latched=rclcpp::QoS(1).reliable().transient_local();
        auto sensor=rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();
        auto boolSub=[&](const char*t,const char*c,const rclcpp::QoS&q){
          sub<std_msgs::msg::Bool>(n,t,q,[this,c](std_msgs::msg::Bool::ConstSharedPtr m){
            emit telemetry(c,m->data);
          });
        };
        const std::vector<std::pair<const char*,const char*>> bools={
          {
            "/gnss/connected","connected.gnss"
          },{
            "/imu/connected","connected.imu"
          },{
            "/perception/camera_connected","connected.camera"
          },{
            "/esc/ready","connected.esc_ready"
          },{
            "/esc/armed","connected.esc_armed"
          },{
            "/esc/feedback_valid","connected.esc_feedback"
          },{
            "/esc/steering_calibration/controller_ready","esc_calibration_controller_ready"
          },{
            "/system/autonomy_ready","system.autonomy_ready"
          },{
            "/system/motion_ready","system.motion_ready"
          },{
            "/system/nav2_ready","system.nav2_ready"
          },{
            "/navigation/mppi_closed_loop/ready","mppi_closed_loop_ready"
          },{
            "/navigation/velocity_smoother/closed_loop_eligible","smoother_closed_loop_eligible"
          },{
            "/gnss/velocity_qualified","gnss_velocity_qualified"
          },{
            "/gnss/cog_qualified","gnss_cog_qualified"
          },{
            "/gnss/velocity_fusion_active","gnss_velocity_fusion_active"
          },{
            "/gnss/cog_fusion_active","gnss_cog_fusion_active"
          },{
            "/perception/camera_healthy","camera_healthy"
          },{
            "/perception/emergency_stop","perception_emergency"
          }
        };
        for(auto&p:bools)boolSub(p.first,p.second,latched);
        boolSub("/safety/estop","system.estop",state);
        const std::vector<std::pair<const char*,const char*>> strings={
          {
            "/system/localization_state","localization_state"
          },{
            "/system/gnss_status","gnss_status"
          },{
            "/gnss/state","gnss_driver_state"
          },{
            "/gnss/motion_diagnostics","gnss_motion"
          },{
            "/gnss/motion_validation","gnss_motion_validation"
          },{
            "/gnss/fusion_status","gnss_fusion_status"
          },{
            "/system/imu_status","imu_status"
          },{
            "/system/ekf_local_status","ekf_local_status"
          },{
            "/system/ekf_global_status","ekf_global_status"
          },{
            "/system/pose_estimator","pose_estimator"
          },{
            "/system/sensor_status","sensor_status"
          },{
            "/navigation/goal_state","goal_state"
          },{
            "/navigation/mppi_closed_loop/status","mppi_status"
          },{
            "/navigation/velocity_smoother/qualification","smoother_qualification"
          },{
            "/perception/lane_safety_state","lane_state"
          },{
            "/perception/lane_control_state","lane_control"
          },{
            "/yolop/lane_metrics","lane_metrics"
          },{
            "/perception/drivable_space","drivable_space"
          },{
            "/perception/camera_health_state","camera_health_state"
          },{
            "/perception/near_field_state","near_field_state"
          },{
            "/perception/obstacle_metrics","obstacle_metrics"
          },{
            "/perception/raw_detections","raw_detections"
          },{
            "/perception/performance","perception_performance"
          },{
            "/navigation/trajectory_safety_state","trajectory_safety_state"
          },{
            "/collision_monitor/state","collision_monitor_state"
          },{
            "/esc/status","esc_status"
          },{
            "/esc/foc/telemetry","foc_telemetry"
          },{
            "/esc/mux/active_source","esc_mux"
          }
        };
        for(auto&p:strings)sub<std_msgs::msg::String>(n,p.first,state,[this,p](std_msgs::msg::String::ConstSharedPtr m){
          const QString raw=QString::fromStdString(m->data);
          const QString channel=QString::fromLatin1(p.second);
          QVariantMap x=parseJsonOrKv(raw);
          if(channel=="raw_detections")x=parseRawDetectionSummary(raw);
          else if(channel=="perception_performance")x=normalizePerceptionPerformance(x);
          else if(channel=="obstacle_metrics")x=enrichObstacleMetrics(x);
          else if(channel=="goal_state"){
            x=QVariantMap{
              {
                "state",raw.trimmed().toUpper()
              },{
                "raw",raw
              }
            };
            std::lock_guard<std::mutex>goalLock(goalMutex_);
            if(goalActive_){
              x["duration_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-goalStarted_).count();
              if(raw.contains("SUCCEEDED",Qt::CaseInsensitive)||raw.contains("CANCELED",Qt::CaseInsensitive)||raw.contains("FAILED",Qt::CaseInsensitive)||raw.contains("ABORT",Qt::CaseInsensitive))goalActive_=false;
            }
          }
          emitMap(channel,x);
        });
        sub<sensor_msgs::msg::NavSatFix>(n,"/gnss/fix_raw",sensor,[this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m){
          emitMap("gnss_fix",{
            {
              "lat",m->latitude
            },{
              "lon",m->longitude
            },{
              "alt",m->altitude
            },{
              "status",m->status.status
            },{
              "cov_x",m->position_covariance[0]
            },{
              "cov_y",m->position_covariance[4]
            },{
              "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
            }
          });
        });
        sub<std_msgs::msg::Float64MultiArray>(n,"/gnss/quality",sensor,[this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m){
          std::vector<double>d=m->data;
          d.resize(std::max<size_t>(45,d.size()),std::numeric_limits<double>::quiet_NaN());
          const char*keys[]={
            "sat","dop","hacc_m","fix_type","source_id","sacc_mps","ground_speed_mps","course_enu_rad","course_accuracy_rad","itow_ms","vacc_m","vel_e_mps","vel_n_mps","vel_d_mps","gdop","hdop","vdop","ndop","edop","tdop","nav_cov_pos_valid","nav_cov_vel_valid","pvt_rate_hz","measurement_age_sec","timestamp_source_code","flags2","flags3","nav_dop_itow_ms","nav_dop_exact_epoch","nav_cov_itow_ms","nav_cov_exact_epoch","utc_valid_flags","tacc_ns","height_ellipsoid_m","head_vehicle_enu_rad","mag_declination_rad","mag_accuracy_rad","diff_solution","carrier_solution","invalid_llh","last_correction_age_code","auth_time","head_vehicle_valid","mag_valid","gnss_fix_ok"
          };
          QVariantMap x;
          for(int i=0;
          i<45;
          ++i)x[keys[i]]=d[i];
          for(const char*k:{
            "nav_cov_pos_valid","nav_cov_vel_valid","nav_dop_exact_epoch","nav_cov_exact_epoch","diff_solution","invalid_llh","auth_time","head_vehicle_valid","mag_valid","gnss_fix_ok"
          })x[k]=number(x[k],0)>0.5;
          emitMap("gnss_quality",x);
        });
        auto velSub=[&](const char*topic,const char*ch){
          sub<geometry_msgs::msg::TwistWithCovarianceStamped>(n,topic,sensor,[this,ch](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m){
            double vx=m->twist.twist.linear.x,vy=m->twist.twist.linear.y;
            emitMap(ch,{
              {
                "vx",vx
              },{
                "vy",vy
              },{
                "vz",m->twist.twist.linear.z
              },{
                "speed",std::hypot(vx,vy)
              },{
                "course_enu_rad",std::hypot(vx,vy)>1e-9?std::atan2(vy,vx):std::numeric_limits<double>::quiet_NaN()
              },{
                "cov_x",m->twist.covariance[0]
              },{
                "cov_y",m->twist.covariance[7]
              },{
                "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
              }
            });
          });
        };
        velSub("/gnss/vel","gnss_vel");
        velSub("/gnss/velocity_position_fit","gnss_vel_fit");
        velSub("/gnss/vel_map","gnss_vel_map");
        velSub("/gnss/base_velocity","gnss_base_vel");
        velSub("/gnss/base_velocity_fusion","gnss_base_vel_fusion");
        sub<geometry_msgs::msg::PoseWithCovarianceStamped>(n,"/gnss/cog_heading_fusion",sensor,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
          auto&q=m->pose.pose.orientation;
          emitMap("gnss_cog_fusion",{
            {
              "yaw_rad",yawFromQuat(q.x,q.y,q.z,q.w)
            },{
              "yaw_variance",m->pose.covariance[35]
            },{
              "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
            }
          });
        });
        sub<sensor_msgs::msg::Imu>(n,"/imu/data",sensor,[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
          auto&q=m->orientation;
          double sinr=2*(q.w*q.x+q.y*q.z),cosr=1-2*(q.x*q.x+q.y*q.y),roll=std::atan2(sinr,cosr),sinp=2*(q.w*q.y-q.z*q.x),pitch=std::abs(sinp)>=1?std::copysign(kPi/2,sinp):std::asin(sinp);
          emitMap("imu",{
            {
              "roll_rad",roll
            },{
              "pitch_rad",pitch
            },{
              "yaw_rad",yawFromQuat(q.x,q.y,q.z,q.w)
            },{
              "gx",m->angular_velocity.x
            },{
              "gy",m->angular_velocity.y
            },{
              "gz",m->angular_velocity.z
            },{
              "ax",m->linear_acceleration.x
            },{
              "ay",m->linear_acceleration.y
            },{
              "az",m->linear_acceleration.z
            },{
              "var_gx",m->angular_velocity_covariance[0]
            },{
              "var_gy",m->angular_velocity_covariance[4]
            },{
              "var_gz",m->angular_velocity_covariance[8]
            },{
              "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
            }
          });
        });
        auto odomSub=[&](const char*topic,const char*ch){
          sub<nav_msgs::msg::Odometry>(n,topic,sensor,[this,ch](nav_msgs::msg::Odometry::ConstSharedPtr m){
            auto&p=m->pose.pose.position;
            auto&q=m->pose.pose.orientation;
            emitMap(ch,{
              {
                "x",p.x
              },{
                "y",p.y
              },{
                "yaw",yawFromQuat(q.x,q.y,q.z,q.w)
              },{
                "v",m->twist.twist.linear.x
              },{
                "w",m->twist.twist.angular.z
              },{
                "var_x",m->pose.covariance[0]
              },{
                "var_y",m->pose.covariance[7]
              },{
                "var_yaw",m->pose.covariance[35]
              },{
                "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
              }
            });
          });
        };
        odomSub("/esc/odom","esc_odom");
        odomSub("/odometry/filtered","ekf_local");
        odomSub("/odometry/filtered_map","ekf_global");
        auto pathSub=[&](const char*topic,const char*ch){
          sub<nav_msgs::msg::Path>(n,topic,sensor,[this,ch](nav_msgs::msg::Path::ConstSharedPtr m){
            QVariantList points;
            points.reserve(int(m->poses.size()));
            double length=0.0;
            double headingVariation=0.0;
            double previousHeading=0.0;
            bool havePrevious=false;
            for(size_t i=0;
            i<m->poses.size();
            ++i){
              const auto&p=m->poses[i].pose.position;
              const auto&q=m->poses[i].pose.orientation;
              double yaw=yawFromQuat(q.x,q.y,q.z,q.w);
              if(i>0){
                const auto&prev=m->poses[i-1].pose.position;
                length+=std::hypot(p.x-prev.x,p.y-prev.y);
              }
              if(havePrevious)headingVariation+=std::abs(normalizeAngle(yaw-previousHeading));
              previousHeading=yaw;
              havePrevious=true;
              points<<QVariantList{
                p.x,p.y,yaw
              };
            }
            double latency=std::numeric_limits<double>::quiet_NaN();
            {
              std::lock_guard<std::mutex>goalLock(goalMutex_);
              if(goalActive_&&!firstPlanSeen_){
                lastPlanningLatencyMs_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-goalStarted_).count();
                firstPlanSeen_=true;
              }
              latency=lastPlanningLatencyMs_;
            }
            emitMap(ch,{
              {
                "count",qlonglong(m->poses.size())
              },{
                "length_m",length
              },{
                "heading_variation_rad",headingVariation
              },{
                "planning_latency_ms",latency
              },{
                "points",points
              },{
                "measurement_stamp_sec",double(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9
              }
            });
          });
        };
        pathSub("/plan","nav_path");
        pathSub("/controller_server/transformed_global_plan","local_path");
        pathSub("/local_plan","local_path");
        auto goalSub=[&](const char*topic){
          sub<geometry_msgs::msg::PoseStamped>(n,topic,state,[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
            const auto&q=m->pose.orientation;
            {
              std::lock_guard<std::mutex>goalLock(goalMutex_);
              goalStarted_=std::chrono::steady_clock::now();
              goalActive_=true;
              firstPlanSeen_=false;
              lastPlanningLatencyMs_=std::numeric_limits<double>::quiet_NaN();
            }
            emitMap("goal_pose",{
              {
                "x",m->pose.position.x
              },{
                "y",m->pose.position.y
              },{
                "yaw",yawFromQuat(q.x,q.y,q.z,q.w)
              }
            });
          });
        };
        goalSub("/navigation/goal_request");
        goalSub("/goal_pose");
        auto twSub=[&](const char*topic,const char*ch){
          sub<geometry_msgs::msg::Twist>(n,topic,sensor,[this,ch](geometry_msgs::msg::Twist::ConstSharedPtr m){
            emitMap(ch,{
              {
                "linear_x",m->linear.x
              },{
                "angular_z",m->angular.z
              }
            });
          });
        };
        twSub("/cmd_vel_nav_raw","cmd_nav");
        twSub("/cmd_vel/perception_advisory","cmd_perception_advisory");
        twSub("/cmd_vel/autonomy_integrated","cmd_autonomy_integrated");
        twSub("/cmd_vel/nav2_pre_collision","cmd_pre_collision");
        twSub("/cmd_vel/collision_preview","cmd_collision_preview");
        twSub("/cmd_vel","cmd_final");
        twSub("/cmd_vel/actuator","cmd_actuator");
        const std::vector<std::pair<const char*,const char*>> floats={
          {
            "/esc/drive_target_mps","esc_drive_target"
          },{
            "/esc/drive_actual_mps","esc_drive_actual"
          },{
            "/esc/steering_target_rad","esc_steer_target"
          },{
            "/esc/steering_command_uncalibrated_rad","esc_steer_uncal_target"
          },{
            "/esc/steering_uncalibrated_rad","esc_steer_uncalibrated"
          },{
            "/esc/steering_protocol_command_rad","esc_steer_protocol_cmd"
          },{
            "/esc/steering_feedback_raw_rad","esc_steer_feedback_raw"
          },{
            "/esc/steering_actual_rad","esc_steer_actual"
          },{
            "/esc/yaw_rate_actual_rps","esc_yaw_rate"
          },{
            "/esc/kinematic_yaw_rate_rps","esc_kinematic_yaw_rate"
          },{
            "/navigation/mppi_closed_loop/velocity_error_mps","mppi_velocity_error"
          },{
            "/navigation/mppi_closed_loop/steering_error_rad","mppi_steering_error"
          },{
            "/navigation/mppi_closed_loop/yaw_rate_error_rps","mppi_yaw_error"
          }
        };
        for(auto&p:floats)sub<std_msgs::msg::Float64>(n,p.first,sensor,[this,p](std_msgs::msg::Float64::ConstSharedPtr m){
          emit telemetry(p.second,m->data);
        });
        sub<sensor_msgs::msg::Image>(n,"/camera/yolop/image_annotated",sensor,[this](sensor_msgs::msg::Image::ConstSharedPtr m){
          if(m->width==0||m->height==0)return;
          QImage img;
          if(m->encoding=="rgb8")img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888).copy();
          else if(m->encoding=="bgr8")img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888).rgbSwapped().copy();
          else if(m->encoding=="mono8"||m->encoding=="8UC1")img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_Grayscale8).copy();
          if(!img.isNull())emit image(img);
        });
        auto maskSub=[&](const char*topic,const char*channel){
          sub<sensor_msgs::msg::Image>(n,topic,sensor,[this,channel](sensor_msgs::msg::Image::ConstSharedPtr m){
            if(m->width==0||m->height==0)return;
            QImage img;
            if(m->encoding=="mono8"||m->encoding=="8UC1")
              img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_Grayscale8).copy();
            else if(m->encoding=="rgb8")
              img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888).convertToFormat(QImage::Format_Grayscale8);
            else if(m->encoding=="bgr8")
              img=QImage(m->data.data(),m->width,m->height,m->step,QImage::Format_RGB888).rgbSwapped().convertToFormat(QImage::Format_Grayscale8);
            if(!img.isNull())emit perceptionMask(QString::fromLatin1(channel),img);
          });
        };
        // Subscribers are GUI-only observers.  If publish_drivable_mask is false
        // in perception YAML the drivable channel simply remains unavailable and
        // Final BAB IV 4.3 reports that prerequisite instead of inventing IoU.
        maskSub("/yolop/drivable_mask","drivable_mask");
        maskSub("/yolop/lane_mask","lane_mask");
        auto cloud=[&](const char*t,const char*c){
          sub<sensor_msgs::msg::PointCloud2>(n,t,sensor,[this,c](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
            emitMap(c,{
              {
                "count",qlonglong(m->width)*m->height
              },{
                "frame_id",QString::fromStdString(m->header.frame_id)
              }
            });
          });
        };
        cloud("/perception/object_points","object_points");
        cloud("/perception/path_relevant_points","path_relevant_points");
        cloud("/perception/planning_relevant_points","planning_relevant_points");
        cloud("/perception/drivable_boundary_points","drivable_boundary_points");
        goalPub_=n->create_publisher<geometry_msgs::msg::PoseStamped>("/navigation/goal_request",10);
        initialPub_=n->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
        emit ready(true,"ROS 2 C++ bridge aktif");
        while(rclcpp::ok()&&!stop_)ex->spin_once(100ms);
      }
      catch(const std::exception&e){
        emit ready(false,QStringLiteral("ROS bridge gagal: ")+e.what());
      }
      std::lock_guard<std::mutex>lk(mu_);
      if(executor_&&node_)executor_->remove_node(node_);
      node_.reset();
      executor_.reset();
      goalPub_.reset();
      initialPub_.reset();
    }
  };
  #include "modules/experiment_components.hpp"
  #include "modules/experiment_components.cpp"
  #include "modules/system_and_gnss_pages.cpp"
  #include "modules/steering_calibration_page.cpp"
  #include "modules/esc_map_overview_pages.cpp"
  #include "modules/navigation_rviz_panel.cpp"
  class CameraCalibrationCanvas:public QWidget{
    Q_OBJECT
    public:CameraCalibrationCanvas(const QMap<QString,std::shared_ptr<YamlStore>>&s,QWidget*p=nullptr):QWidget(p),s_(s){
      per_=s_.value("perception");
      gui_=s_.value("gui");
      calW_=per_?per_->get("perception.ros__parameters.ground_calibration_width",1280).toInt():1280;
      calH_=per_?per_->get("perception.ros__parameters.ground_calibration_height",720).toInt():720;
      auto*l=new QVBoxLayout(this);
      l->setContentsMargins(0,0,0,0);
      auto*bar=new QHBoxLayout();
      auto*fit=new QPushButton("Fit Camera");
      auto*reloadBtn=new QPushButton("Reload YAML");
      coord_=new QLabel("Drag titik overlay; metric tampil di sini");
      bar->addWidget(fit);
      bar->addWidget(reloadBtn);
      bar->addWidget(coord_,1);
      l->addLayout(bar);
      scene_=new QGraphicsScene(this);
      scene_->setSceneRect(0,0,calW_,calH_);
      view_=new ZoomGraphicsView();
      view_->setScene(scene_);
      l->addWidget(view_,1);
      pix_=scene_->addPixmap(QPixmap());
      pix_->setZValue(-100);
      connect(fit,&QPushButton::clicked,this,[this](){
        fitView();
      });
      connect(reloadBtn,&QPushButton::clicked,this,[this](){
        this->reload();
      });
      placeholder();
      this->reload();
    }
    void setImage(const QImage&i){
      last_=i.copy();
      pix_->setPixmap(QPixmap::fromImage(last_).scaled(calW_,calH_,Qt::IgnoreAspectRatio,Qt::SmoothTransformation));
    }
    ZoomGraphicsView*view()const{
      return view_;
    }
    void setVisibleLayer(const QString&layer,bool on){
      visible_[layer]=on;
      for(auto*h:handles_[layer])h->setVisible(on);
      for(auto*l:lines_[layer])l->setVisible(on);
    }
    QVector<QPointF> metricPoints(const QString&layer)const{
      QVector<QPointF>out;
      for(int i=0;
      i<4;
      ++i){
        bool ok=false;
        QPointF p=projectPixel(number(points_.value(layer).value(2*i)),number(points_.value(layer).value(2*i+1)),&ok);
        if(!ok)return{
        };
        out<<p;
      }
      return out;
    }
    signals:void pointsChanged(QString layer,QVariantList points);
    protected:void resizeEvent(QResizeEvent*e)override{
      QWidget::resizeEvent(e);
      QTimer::singleShot(0,this,[this](){
        fitView();
      });
    }
    private:QMap<QString,std::shared_ptr<YamlStore>>s_;
    std::shared_ptr<YamlStore>per_,gui_;
    int calW_,calH_;
    QGraphicsScene*scene_;
    ZoomGraphicsView*view_;
    QGraphicsPixmapItem*pix_;
    QLabel*coord_;
    QImage last_;
    QMap<QString,QVariantList>points_;
    QMap<QString,QVector<DragHandle*>>handles_;
    QMap<QString,QVector<QGraphicsLineItem*>>lines_;
    QMap<QString,bool>visible_{
      {
        "ground",true
      },{
        "obstacle",true
      },{
        "lane",true
      }
    };
    void placeholder(){
      QImage i(calW_,calH_,QImage::Format_RGB32);
      i.fill(QColor("#11151a"));
      QPainter p(&i);
      p.setPen(QColor(kMuted));
      p.setFont(QFont("DejaVu Sans",22,QFont::Bold));
      p.drawText(i.rect(),Qt::AlignCenter,"Menunggu /camera/yolop/image_annotated");
      pix_->setPixmap(QPixmap::fromImage(i));
    }
    QVariantList valid(const QVariant&v,const QVariantList&fallback){
      QVariantList x=v.toList();
      return x.size()==8?x:fallback;
    }
    void reload(){
      if(per_)per_->reload();
      if(gui_)gui_->reload();
      points_["ground"]=valid(per_->get("perception.ros__parameters.ground_src_points"),{
        40.,680.,1240.,680.,760.,350.,520.,350.
      });
      points_["obstacle"]=valid(gui_->get("camera_overlay.obstacle_roi_points_px"),{
        180.,700.,1100.,700.,780.,380.,500.,380.
      });
      points_["lane"]=valid(gui_->get("camera_overlay.lane_safety_points_px"),{
        340.,700.,940.,700.,740.,420.,540.,420.
      });
      redraw();
      QTimer::singleShot(0,this,[this](){
        fitView();
      });
    }
    QColor color(const QString&l)const{
      return l=="ground"?QColor(kGold):l=="obstacle"?QColor(kRed):QColor(kGreen);
    }
    void redraw(){
      for(auto hs:handles_)for(auto*h:hs)scene_->removeItem(h);
      for(auto ls:lines_)for(auto*x:ls)scene_->removeItem(x);
      handles_.clear();
      lines_.clear();
      for(const QString&layer:{
        QString("ground"),QString("obstacle"),QString("lane")
      }){
        QVariantList pts=points_[layer];
        for(int i=0;
        i<4;
        ++i){
          auto*h=new DragHandle({
            pts[2*i].toDouble(),pts[2*i+1].toDouble()
          },color(layer));
          h->setVisible(visible_.value(layer,true));
          h->moved=[this,layer,i](QPointF p){
            auto pts=points_[layer];
            pts[2*i]=p.x();
            pts[2*i+1]=p.y();
            points_[layer]=pts;
            updateLines(layer);
            bool ok=false;
            QPointF m=projectPixel(p.x(),p.y(),&ok);
            coord_->setText(ok?QString("%1 P%2 px=(%3,%4) → forward=%5 m, left=%6 m").arg(layer.toUpper()).arg(i+1).arg(p.x(),0,'f',1).arg(p.y(),0,'f',1).arg(m.x(),0,'f',3).arg(m.y(),0,'f',3):QString("%1 P%2 px=(%3,%4)").arg(layer).arg(i+1).arg(p.x()).arg(p.y()));
          };
          h->released=[this,layer](){
            emit pointsChanged(layer,points_[layer]);
          };
          scene_->addItem(h);
          handles_[layer]<<h;
        }
        for(int i=0;
        i<4;
        ++i){
          auto*line=scene_->addLine(0,0,0,0,QPen(color(layer),2.5));
          line->setZValue(110);
          line->setVisible(visible_.value(layer,true));
          lines_[layer]<<line;
        }
        updateLines(layer);
      }
    }
    void updateLines(const QString&layer){
      auto hs=handles_[layer];
      auto ls=lines_[layer];
      if(hs.size()!=4||ls.size()!=4)return;
      for(int i=0;
      i<4;
      ++i){
        QPointF a=hs[i]->pos(),b=hs[(i+1)%4]->pos();
        ls[i]->setLine(QLineF(a,b));
      }
    }
    QPointF projectPixel(double px,double py,bool*ok)const{
      double H[9];
      QVariantList src=points_.value("ground"),dst=per_->get("perception.ros__parameters.ground_dst_points",QVariantList{
      }).toList();
      if(!solveHomography8(src,dst,H)){
        if(ok)*ok=false;
        return{
        };
      }
      bool hOk=false;
      QPointF b=projectH(H,px,py,&hOk);
      if(!hOk){
        if(ok)*ok=false;
        return{
        };
      }
      double ox=number(per_->get("perception.ros__parameters.ground_origin_x_px",0),0),oy=number(per_->get("perception.ros__parameters.ground_origin_y_px",0),0),sx=number(per_->get("perception.ros__parameters.ground_meters_per_pixel_x",1),1),sy=number(per_->get("perception.ros__parameters.ground_meters_per_pixel_y",1),1),fo=number(per_->get("perception.ros__parameters.metric_forward_offset_m",0),0),lo=number(per_->get("perception.ros__parameters.metric_lateral_offset_m",0),0);
      if(ok)*ok=true;
      return{
        (oy-b.y())*sy+fo,(ox-b.x())*sx+lo
      };
    }
    void fitView(){
      view_->fitInView(scene_->sceneRect(),Qt::KeepAspectRatio);
    }
  };
  #include "modules/camera_reports_pages.cpp"
  #include "modules/system_overview_page.cpp"
  #include "modules/main_window.cpp"
  int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling,true);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps,true);
    int qargc=1;
    char* qargv[]={
      argv[0],nullptr
    };
    QApplication app(qargc,qargv);
    app.setApplicationName(kAppTitle);
    qRegisterMetaType<QImage>("QImage");
    qRegisterMetaType<QVariantMap>("QVariantMap");
    MainWindow w;
    w.showMaximized();
    int rc=app.exec();
    if(rclcpp::ok())rclcpp::shutdown();
    return rc;
  }
  #include "agv_gui.moc"
