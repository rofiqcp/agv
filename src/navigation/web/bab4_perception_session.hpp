#pragma once
// Backend-owned Bab 4 Perception research session.
// Browser is a view/controller only; counters and samples live here.
class Bab4PerceptionSessionStore {
 public:
  Bab4PerceptionSessionStore(const QString &agvRoot, const QJsonObject &configSnapshot)
      : dataRoot_(QDir(agvRoot).filePath(QStringLiteral("data/presepsi"))),
        configSnapshot_(configSnapshot) {}

  QJsonObject ensure() {
    QDir().mkpath(dataRoot_);
    const QJsonObject pointer = readJson(QDir(dataRoot_).filePath(QStringLiteral("bab4_active_session.json")));
    const QString id = pointer.value(QStringLiteral("session_id")).toString();
    if (validSessionId(id) && QFileInfo(statePath(id)).isFile()) return response(load(id));
    return create();
  }

  QJsonObject create() {
    const QString id = QStringLiteral("BAB4_PERSEPSI_") +
      QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    const QString root = sessionPath(id);
    for (const QString &d : {QStringLiteral("4.1_VALIDASI_SISTEM"), QStringLiteral("4.2_HUMAN_DETECTION"),
                             QStringLiteral("4.3_LANE_SAFETY"), QStringLiteral("4.4_REALTIME_PERFORMANCE"),
                             QStringLiteral("4.5_VEHICLE_RESPONSE")}) QDir().mkpath(QDir(root).filePath(d));
    QJsonObject st = initialState(id);
    if (!saveJson(statePath(id), st)) return error(QStringLiteral("Gagal menyimpan state session"));
    saveJson(QDir(root).filePath(QStringLiteral("metadata.json")), metadataFor(st));
    saveJson(QDir(dataRoot_).filePath(QStringLiteral("bab4_active_session.json")),
             QJsonObject{{QStringLiteral("session_id"), id}});
    logEvent(id, QStringLiteral("SESSION_CREATE"), QStringLiteral(""), QStringLiteral(""), QStringLiteral("Session baru dibuat"));
    return response(st);
  }
  QJsonObject state() {
    const QJsonObject e = ensure();
    if (!e.value(QStringLiteral("ok")).toBool()) return e;
    return e;
  }

  QJsonObject startCondition(const QString &sub, const QString &condition) {
    QJsonObject st = current();
    if (st.isEmpty()) return error(QStringLiteral("Session aktif tidak tersedia"));
    QString why;
    if (!validCondition(sub, condition, &why)) return error(why);
    QJsonObject subs = st.value(QStringLiteral("subchapters")).toObject();
    QJsonObject s = subs.value(sub).toObject();
    QJsonObject cs = s.value(QStringLiteral("conditions")).toObject();
    QJsonObject c = cs.value(condition).toObject();
    const QJsonArray existingRows = c.value(QStringLiteral("rows")).toArray();
    if ((sub == QStringLiteral("4.2") || sub == QStringLiteral("4.3")) && existingRows.size() >= 10)
      return error(QStringLiteral("Kondisi sudah 10/10 — reset kondisi jika ingin mengulang"));
    if (sub == QStringLiteral("4.4") && c.value(QStringLiteral("status")).toString() == QStringLiteral("SELESAI"))
      return error(QStringLiteral("Kondisi 4.4 sudah selesai — reset kondisi jika ingin mengulang"));
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const int windowSec = sub == QStringLiteral("4.4") ? 60 : 30;
    c[QStringLiteral("started_at_ms")] = static_cast<double>(now);
    c[QStringLiteral("deadline_ms")] = static_cast<double>(now + windowSec * 1000LL);
    c[QStringLiteral("status")] = QStringLiteral("BERJALAN");
    if (!c.contains(QStringLiteral("rows"))) c[QStringLiteral("rows")] = QJsonArray();
    cs[condition] = c; s[QStringLiteral("conditions")] = cs; subs[sub] = s;
    st[QStringLiteral("subchapters")] = subs;
    st[QStringLiteral("updated_at")] = isoNow();
    if (!saveState(st)) return error(QStringLiteral("Gagal menyimpan start condition"));
    logEvent(st.value("session_id").toString(), QStringLiteral("TEST_START"), sub, condition,
             QStringLiteral("Window %1 detik dimulai").arg(windowSec));
    return response(st);
  }

  QJsonObject capture(const QString &sub, const QString &condition, const QJsonObject &request,
                      const QJsonObject &snapshot, const QByteArray &jpeg) {
    if (jpeg.isEmpty()) return error(QStringLiteral("CAMERA STREAM STALE / JPEG kosong"));
    QJsonObject st = current(); if (st.isEmpty()) return error(QStringLiteral("Session aktif tidak tersedia"));
    const QJsonObject originalState=st;
    QString why; if (!validCondition(sub, condition, &why)) return error(why);
    QJsonObject subs = st.value(QStringLiteral("subchapters")).toObject();
    QJsonObject s = subs.value(sub).toObject();
    QJsonObject cs = s.value(QStringLiteral("conditions")).toObject();
    const bool conditionExisted = cs.contains(condition);
    QJsonObject c = cs.value(condition).toObject();
    QJsonArray rows = c.value(QStringLiteral("rows")).toArray();
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if(sub==QStringLiteral("4.5")){
      bool armedComplete=c.value(QStringLiteral("armed_complete")).toBool(false);
      if(!armedComplete){
        for(const QJsonValue &v:rows){const QJsonObject rr=v.toObject();
          if(rr.value("mode").toString()=="ARMED_VEHICLE_TEST"&&rr.value("result").toString().toLower()=="complete"&&rr.value("final_vehicle_state").toString()=="STOPPED"&&rr.value("integration_pass").toBool(false)){armedComplete=true;break;}
        }
      }
      if(armedComplete)
        return error(QStringLiteral("Kondisi 4.5 sudah SELESAI — reset kondisi untuk mengulang"));
      if(c.value(QStringLiteral("dry_run_complete")).toBool(false)||std::any_of(rows.begin(),rows.end(),[](const QJsonValue &v){return v.toObject().value("mode").toString()=="DRY_RUN";}))
        return error(QStringLiteral("DRY RUN kondisi 4.5 sudah tersimpan — lanjut ARMED atau reset kondisi untuk mengulang DRY RUN"));
    }
    const qint64 started = static_cast<qint64>(c.value(QStringLiteral("started_at_ms")).toDouble(0));
    const qint64 deadline = static_cast<qint64>(c.value(QStringLiteral("deadline_ms")).toDouble(0));
    if ((sub == "4.2" || sub == "4.3" || sub == "4.4") && (!started || now > deadline))
      return error(QStringLiteral("Window pengujian belum dimulai atau sudah berakhir"));
    if ((sub == "4.2" || sub == "4.3") && rows.size() >= 10)
      return error(QStringLiteral("Kondisi sudah 10/10"));
    const QJsonObject updated=snapshot.value(QStringLiteral("__updated")).toObject();
    const QJsonObject prev=rows.isEmpty()?QJsonObject():rows.last().toObject();
    auto requireFreshSource=[&](const QString &channel,const QString &rowKey,const QString &label)->QJsonObject {
      const double at=updated.value(channel).toDouble(0);
      if(at<=0 || static_cast<double>(now)-at>1500.0)
        return error(label+QStringLiteral(" STALE — counter tidak bertambah"));
      if(!prev.isEmpty() && prev.value(rowKey).toDouble(0)>=at)
        return error(label+QStringLiteral(" BELUM UPDATE — sample duplikat ditolak"));
      return QJsonObject();
    };
    if(sub==QStringLiteral("4.2")){
      const QJsonObject e=requireFreshSource(QStringLiteral("semantic_detections"),QStringLiteral("semantic_updated_ms"),QStringLiteral("SEMANTIC DETECTOR"));
      if(!e.isEmpty())return e;
    } else if(sub==QStringLiteral("4.3")){
      const QJsonObject e=requireFreshSource(QStringLiteral("lane_state"),QStringLiteral("lane_updated_ms"),QStringLiteral("LANE STATE"));
      if(!e.isEmpty())return e;
    } else if(sub==QStringLiteral("4.5")){
      const bool obstacle=condition.startsWith(QStringLiteral("obstacle_"));
      const QJsonObject e=requireFreshSource(obstacle?QStringLiteral("semantic_detections"):QStringLiteral("lane_state"),
                                             QStringLiteral("source_updated_ms"),
                                             obstacle?QStringLiteral("SEMANTIC DETECTOR"):QStringLiteral("LANE STATE"));
      if(!e.isEmpty())return e;
    }
    QJsonObject row = buildRow(sub, condition, request, snapshot, rows.size() + 1);
    const double captureAge=row.value(QStringLiteral("frame_age_ms")).toDouble(std::numeric_limits<double>::infinity());
    if(sub==QStringLiteral("4.1")){
      if(!row.value("camera_stream").toBool(false)||!row.value("camera_healthy").toBool(false)||!row.value("yolop_active").toBool(false)||
         row.value("stream_status").toString()!=QStringLiteral("LIVE")||row.value("semantic_stream").toString()!=QStringLiteral("LIVE")||
         row.value("lane_stream").toString()!=QStringLiteral("LIVE")||!row.value("orientation_left_ok").toBool(false)||!row.value("orientation_right_ok").toBool(false))
        return error(QStringLiteral("VALIDASI 4.1 BELUM PASS — capture bukti ditolak"));
    }
    if ((sub == QStringLiteral("4.2") || sub == QStringLiteral("4.3") || sub == QStringLiteral("4.5")) &&
        (!std::isfinite(captureAge) || captureAge > 1500.0))
      return error(QStringLiteral("CAMERA STREAM STALE — counter tidak bertambah"));
    if (sub == QStringLiteral("4.4")) {
      const QJsonObject perf=snapshot.value(QStringLiteral("perception_performance")).toObject();
      const double curDrop=perf.value(QStringLiteral("capture_dropped_total")).toDouble(0);
      const double curOverwrite=perf.value(QStringLiteral("capture_overwrite_total")).toDouble(0);
      const QJsonObject prev=rows.isEmpty()?QJsonObject():rows.last().toObject();
      const double prevDrop=prev.value(QStringLiteral("capture_dropped_total")).toDouble(curDrop);
      const double prevOverwrite=prev.value(QStringLiteral("capture_overwrite_total")).toDouble(curOverwrite);
      row[QStringLiteral("capture_dropped_total")]=curDrop;
      row[QStringLiteral("capture_overwrite_total")]=curOverwrite;
      row[QStringLiteral("dropped_frame")]=std::max(0.0,curDrop-prevDrop)+std::max(0.0,curOverwrite-prevOverwrite);
      const double prevStamp=prev.value(QStringLiteral("frame_timestamp_ms")).toDouble(0);
      const double curStamp=row.value(QStringLiteral("frame_timestamp_ms")).toDouble(0);
      row[QStringLiteral("stream_gap")]=(prevStamp>0&&curStamp>0&&curStamp-prevStamp>1500.0)?1:0;
      const double perfAt=snapshot.value(QStringLiteral("__updated")).toObject().value(QStringLiteral("perception_performance")).toDouble(0);
      row[QStringLiteral("subscriber_latency_ms")]=perfAt>0?static_cast<double>(now)-perfAt:QJsonValue();
    }
    const QByteArray displayJpeg = sub == QStringLiteral("4.4") ? QByteArray() : renderJpegForDisplay(jpeg,row,sub==QStringLiteral("4.2")||sub==QStringLiteral("4.5"));
    QString imageAbs;
    if (sub != QStringLiteral("4.4")) {
      const QString imageRel = imageRelative(sub, condition, rows.size() + 1);
      imageAbs = QDir(sessionPath(st.value("session_id").toString())).filePath(imageRel);
      QDir().mkpath(QFileInfo(imageAbs).absolutePath());
      QSaveFile im(imageAbs);
      if (!im.open(QIODevice::WriteOnly) || im.write(displayJpeg) != displayJpeg.size() || !im.commit())
        return error(QStringLiteral("DATA STORAGE ERROR: gambar gagal disimpan"));
      row[QStringLiteral("frame_filename")] = imageRel;
    } else {
      row[QStringLiteral("frame_filename")] = QJsonValue();
    }
    rows.append(row); c[QStringLiteral("rows")] = rows;
    c[QStringLiteral("sample_count")] = rows.size();
    if ((sub == "4.2" || sub == "4.3") && rows.size() == 10) c[QStringLiteral("status")] = QStringLiteral("SELESAI");
    if (sub == "4.4" && now - started >= 59000) c[QStringLiteral("status")] = QStringLiteral("SELESAI");
    if (sub == "4.5") {
      const bool pass=row.value(QStringLiteral("integration_pass")).toBool(false);
      c[QStringLiteral("dry_run_complete")] = true;
      c[QStringLiteral("dry_run_pass")] = pass;
      c[QStringLiteral("status")] = pass ? QStringLiteral("DRY_RUN_PASS") : QStringLiteral("DRY_RUN_FAIL");
    }
    cs[condition] = c; s[QStringLiteral("conditions")] = cs; subs[sub] = s;
    st[QStringLiteral("subchapters")] = subs; st[QStringLiteral("updated_at")] = isoNow();
    if (!writeCsvArtifacts(st, sub, condition)) {
      writeCsvArtifacts(originalState, sub, condition);
      if(!conditionExisted)QDir(conditionPath(st.value("session_id").toString(),sub,condition)).removeRecursively();
      if(!imageAbs.isEmpty())QFile::remove(imageAbs);
      return error(QStringLiteral("DATA STORAGE ERROR: CSV gagal disimpan; counter tidak bertambah"));
    }
    if (!saveState(st)) {
      writeCsvArtifacts(originalState, sub, condition);
      if(!conditionExisted)QDir(conditionPath(st.value("session_id").toString(),sub,condition)).removeRecursively();
      if(!imageAbs.isEmpty())QFile::remove(imageAbs);
      return error(QStringLiteral("DATA STORAGE ERROR: state gagal disimpan; CSV di-rollback; counter tidak bertambah"));
    }
    logEvent(st.value("session_id").toString(), QStringLiteral("CAPTURE_SUCCESS"), sub, condition,
             QStringLiteral("Sample %1 tersimpan").arg(rows.size()));
    QJsonObject out = response(st); out[QStringLiteral("row")] = row;
    if (sub != QStringLiteral("4.4"))
      out[QStringLiteral("captured_image_data_url")] = QStringLiteral("data:image/jpeg;base64,") + QString::fromLatin1(displayJpeg.toBase64());
    return out;
  }
  QJsonObject recordArmed(const QString &mode,const QString &legacyCondition,const QJsonObject &trial,
                          const QJsonObject &snapshot,const QByteArray &jpeg) {
    QString condition;
    if(mode=="obstacle"){
      const QRegularExpression trialRe(QStringLiteral("^trial([1-3])$"));
      const auto tm=trialRe.match(legacyCondition);
      condition=tm.hasMatch()?QStringLiteral("obstacle_trial_")+tm.captured(1):
        (legacyCondition=="none" ? QStringLiteral("obstacle_no_human") :
         QStringLiteral("obstacle_")+legacyCondition+QStringLiteral("m"));
    } else {
      const int laneTrial=trial.value("trial_index").toInt(0);
      if(laneTrial>=1&&laneTrial<=3) condition=QStringLiteral("lane_trial_")+QString::number(laneTrial);
      else {
        const QMap<QString,QString> map{{"center","lane_trial_green"},{"left_yellow","lane_trial_yellow"},
          {"right_yellow","lane_trial_yellow"},{"left_red","lane_trial_red"},{"right_red","lane_trial_red"},
          {"gray","lane_gray_gray"}};
        condition=map.value(legacyCondition);
      }
    }
    QString why;if(condition.isEmpty()||!validCondition("4.5",condition,&why))return error(QStringLiteral("Kondisi ARMED tidak valid"));
    QJsonObject st=current();if(st.isEmpty())return error(QStringLiteral("Session aktif tidak tersedia"));
    const QJsonObject originalState=st;
    QJsonObject subs=st.value("subchapters").toObject(),s=subs.value("4.5").toObject(),cs=s.value("conditions").toObject();
    QJsonObject c=cs.value(condition).toObject();QJsonArray rows=c.value("rows").toArray();
    bool armedComplete=c.value("armed_complete").toBool(false);
    if(!armedComplete){
      for(const QJsonValue &v:rows){const QJsonObject rr=v.toObject();
        if(rr.value("mode").toString()=="ARMED_VEHICLE_TEST"&&rr.value("result").toString().toLower()=="complete"&&rr.value("final_vehicle_state").toString()=="STOPPED"&&rr.value("integration_pass").toBool(false)){armedComplete=true;break;}
      }
    }
    if(armedComplete)return error(QStringLiteral("Kondisi 4.5 sudah SELESAI — reset kondisi untuk mengulang"));
    QJsonObject row=buildRow("4.5",condition,QJsonObject(),snapshot,rows.size()+1);
    row["mode"]="ARMED_VEHICLE_TEST";row["command_sent"]=true;row["response_start"]=trial.value("go_at_ms");
    row["response_end"]=static_cast<double>(QDateTime::currentMSecsSinceEpoch());row["response_time_ms"]=trial.value("response_ms");
    row["final_vehicle_state"]=trial.value("stopped").toBool(false)?QStringLiteral("STOPPED"):QStringLiteral("UNVERIFIED");
    row["heartbeat_count"]=trial.value("heartbeat_count");
    row["heartbeat_last_ms"]=trial.value("heartbeat_last_ms");
    row["heartbeat_age_ms"]=trial.value("heartbeat_age_ms");
    row["heartbeat_transport"]=trial.value("heartbeat_transport");
    row["heartbeat_sequence"]=trial.value("heartbeat_sequence");
    row["trial_person_only_planning_filter"]=trial.value("trial_person_only_planning_filter");
    row["obstacle_motion_released"]=trial.value("obstacle_motion_released");
    row["obstacle_motion_released_at_ms"]=trial.value("obstacle_motion_released_at_ms");
    row["obstacle_memory_latched_gate_at_ms"]=trial.value("obstacle_memory_latched_gate_at_ms");
    row["planner_gate_pass"]=trial.value("planner_gate_pass");
    row["planner_gate_started_ms"]=trial.value("planner_gate_started_ms");
    row["planner_gate_passed_ms"]=trial.value("planner_gate_passed_ms");
    row["planner_gate_elapsed_ms"]=trial.value("planner_gate_elapsed_ms");
    row["planner_gate_reason"]=trial.value("planner_gate_reason");
    row["planner_gate_planning_points"]=trial.value("planner_gate_planning_points");
    row["planner_gate_local_costmap_aligned"]=trial.value("planner_gate_local_costmap_aligned");
    row["planner_gate_global_costmap_post_latch"]=trial.value("planner_gate_global_costmap_post_latch");
    row["planner_gate_path_post_costmap"]=trial.value("planner_gate_path_post_costmap");
    row["planner_gate_mppi_post_path"]=trial.value("planner_gate_mppi_post_path");
    row["planner_gate_path_length_m"]=trial.value("planner_gate_path_length_m");
    row["planner_gate_path_heading_variation_rad"]=trial.value("planner_gate_path_heading_variation_rad");
    row["planner_hold_active"]=trial.value("planner_hold_active");
    row["planner_hold_count"]=trial.value("planner_hold_count");
    row["planner_hold_recovery_count"]=trial.value("planner_hold_recovery_count");
    row["planner_hold_reason"]=trial.value("planner_hold_reason");
    row["planner_path_age_ms"]=trial.value("planner_path_age_ms");
    row["planner_path_count"]=trial.value("planner_path_count");
    row["planner_path_length_m"]=trial.value("planner_path_length_m");
    row["planner_path_heading_variation_rad"]=trial.value("planner_path_heading_variation_rad");
    row["planner_path_quality_ok"]=trial.value("planner_path_quality_ok");
    row["result"]=trial.value("phase");row["trial_reason"]=trial.value("reason");row["travel_m"]=trial.value("travel_m");
    row["forward_progress_m"]=trial.value("forward_progress_m");row["lateral_displacement_m"]=trial.value("lateral_displacement_m");
    row["recovery_ms"]=trial.value("recovery_ms");row["command_v_mps"]=trial.value("command_v_mps");row["command_w_rad_s"]=trial.value("command_w_rad_s");
    row["initial_lane_status"]=trial.value("initial_lane_status");row["initial_lane_state"]=trial.value("initial_lane_state");
    row["initial_lane_recommendation"]=trial.value("initial_lane_recommendation");
    row["initial_left_latched"]=trial.value("initial_left_latched");row["initial_right_latched"]=trial.value("initial_right_latched");
    row["lane_left_latched"]=trial.value("lane_left_latched");row["lane_right_latched"]=trial.value("lane_right_latched");
    row["expected_steering_direction"]=trial.value("expected_steering_direction");
    row["web_trial_authority_seen"]=trial.value("web_trial_authority_seen");row["web_trial_authority_at_ms"]=trial.value("web_trial_authority_at_ms");row["cmd_mux_source_last"]=trial.value("cmd_mux_source_last");
    row["steering_target_runtime_rad"]=trial.value("steering_target_rad");row["steering_actual_runtime_rad"]=trial.value("steering_actual_rad");
    const double steerRef=trial.value("steering_target_rad").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double steerActual=trial.value("steering_actual_rad").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double steerErr=(std::isfinite(steerRef)&&std::isfinite(steerActual))?steerActual-steerRef:std::numeric_limits<double>::quiet_NaN();
    row["steering_reference_rad"]=finiteOrNull(steerRef);row["steering_actual_feedback_rad"]=finiteOrNull(steerActual);
    row["steering_signed_error_rad"]=finiteOrNull(steerErr);
    row["steering_abs_error_rad"]=finiteOrNull(std::isfinite(steerErr)?std::abs(steerErr):steerErr);
    row["steering_signed_error_deg"]=finiteOrNull(std::isfinite(steerErr)?steerErr*180.0/3.14159265358979323846:steerErr);
    row["steering_abs_error_deg"]=finiteOrNull(std::isfinite(steerErr)?std::abs(steerErr)*180.0/3.14159265358979323846:steerErr);
    row["peak_steering_actual_rad"]=trial.value("peak_steering_actual_rad");
    row["steering_response_seen"]=trial.value("steering_response_seen");row["steering_direction_match"]=trial.value("steering_direction_match");
    row["steering_response_ms"]=trial.value("steering_response_ms");row["steering_neutral_maintained"]=trial.value("steering_neutral_maintained");
    row["target_distance_m"]=trial.value("target_distance_m");row["clear_target_m"]=trial.value("clear_target_m");
    const double goalRef=trial.value("target_distance_m").toDouble(
      trial.value("goal_distance_m").toDouble(mode=="obstacle"?5.0:std::numeric_limits<double>::quiet_NaN()));
    const double goalActual=trial.value("forward_progress_m").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double goalErr=(std::isfinite(goalRef)&&std::isfinite(goalActual))?goalActual-goalRef:std::numeric_limits<double>::quiet_NaN();
    row["goal_reference_m"]=finiteOrNull(goalRef);row["goal_actual_progress_m"]=finiteOrNull(goalActual);
    row["goal_signed_error_m"]=finiteOrNull(goalErr);row["goal_abs_error_m"]=finiteOrNull(std::isfinite(goalErr)?std::abs(goalErr):goalErr);
    row["reference_type"]=QStringLiteral("GOAL_FORWARD_DISTANCE_M");
    row["reference_source"]=QStringLiteral("trial_protocol");
    row["actual_source"]=QStringLiteral("esc_odom_forward_progress_m");
    row["reference_value"]=finiteOrNull(goalRef);row["actual_value"]=finiteOrNull(goalActual);
    row["signed_error"]=finiteOrNull(goalErr);row["abs_error"]=finiteOrNull(std::isfinite(goalErr)?std::abs(goalErr):goalErr);

    row["initial_heading_rad"]=trial.value("initial_heading_rad");row["current_heading_rad"]=trial.value("current_heading_rad");
    row["heading_error_rad"]=trial.value("heading_error_rad");row["heading_error_deg"]=trial.value("heading_error_deg");
    const double headingRef=trial.value("initial_heading_rad").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double headingActual=trial.value("current_heading_rad").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double headingSigned=(std::isfinite(headingRef)&&std::isfinite(headingActual))?
      std::atan2(std::sin(headingActual-headingRef),std::cos(headingActual-headingRef)):
      std::numeric_limits<double>::quiet_NaN();
    row["heading_reference_rad"]=finiteOrNull(headingRef);row["heading_actual_rad"]=finiteOrNull(headingActual);
    row["heading_signed_error_rad"]=finiteOrNull(headingSigned);
    row["heading_abs_error_rad"]=finiteOrNull(std::isfinite(headingSigned)?std::abs(headingSigned):headingSigned);
    row["heading_signed_error_deg"]=finiteOrNull(std::isfinite(headingSigned)?headingSigned*180.0/3.14159265358979323846:headingSigned);
    row["heading_abs_error_deg"]=finiteOrNull(std::isfinite(headingSigned)?std::abs(headingSigned)*180.0/3.14159265358979323846:headingSigned);
    row["heading_restored"]=trial.value("heading_restored");row["heading_restored_once"]=trial.value("heading_restored_once");
    row["lane_phase"]=trial.value("lane_phase");
    row["lane_nav2_handoff"]=trial.value("lane_nav2_handoff");
    row["lane_nav2_handoff_at_ms"]=trial.value("lane_nav2_handoff_at_ms");
    row["lane_nav2_authority_seen"]=trial.value("lane_nav2_authority_seen");
    row["initial_map_x"]=trial.value("initial_map_x");row["initial_map_y"]=trial.value("initial_map_y");
    row["initial_map_heading_rad"]=trial.value("initial_map_heading_rad");
    row["goal_x_map"]=trial.value("goal_x_map");row["goal_y_map"]=trial.value("goal_y_map");row["goal_state"]=trial.value("goal_state");
    const QJsonObject finalMap=snapshot.value("ekf_global").toObject();
    const double finalMapX=number(finalMap,"x"),finalMapY=number(finalMap,"y");
    const double goalX=trial.value("goal_x_map").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double goalY=trial.value("goal_y_map").toDouble(std::numeric_limits<double>::quiet_NaN());
    const double endpointErr=(std::isfinite(finalMapX)&&std::isfinite(finalMapY)&&std::isfinite(goalX)&&std::isfinite(goalY))?
      std::hypot(finalMapX-goalX,finalMapY-goalY):std::numeric_limits<double>::quiet_NaN();
    row["final_map_x"]=finiteOrNull(finalMapX);row["final_map_y"]=finiteOrNull(finalMapY);
    row["goal_endpoint_error_m"]=finiteOrNull(endpointErr);
    row["metric_position_valid"]=trial.value("metric_position_valid");
    row["object_cloud_seen"]=trial.value("object_cloud_seen");
    row["planning_cloud_seen"]=trial.value("planning_cloud_seen");
    row["costmap_marking_seen"]=trial.value("costmap_marking_seen");
    row["mppi_seen"]=trial.value("mppi_seen");
    row["avoidance_seen"]=trial.value("avoidance_seen");
    row["recovery_seen"]=trial.value("recovery_seen");
    row["integration_pass"]=trial.value("integration_pass");
    row["safety_decision_last"]=trial.value("safety_decision_last");
    row["avoid_direction"]=trial.value("avoid_direction");row["human_start_confidence"]=trial.value("human_start_confidence");
    row["actual_action"]=trial.value("response");
    row["human_start_display_x_fraction"]=trial.value("human_start_display_x_fraction");
    row["initial_condition_match"]=trial.value("initial_condition_match").toBool(true);
    if(mode=="obstacle"){
      const QRegularExpression trialRe(QStringLiteral("^trial([1-3])$"));
      const auto tm=trialRe.match(legacyCondition);
      if(tm.hasMatch()){
        row["trial_index"]=tm.captured(1).toInt();
        row["goal_distance_m"]=5.0;
        row["reference_distance_m"]=QJsonValue();
        row["expected_status"]=QStringLiteral("HUMAN_ON_5M_GOAL_ROUTE");
        row["expected_action"]=QStringLiteral("AVOID_PASS_HUMAN_RECOVER_REACH_5M_GOAL_STOP");
      }else if(legacyCondition=="none"){
        row["expected_status"]=QStringLiteral("NO_HUMAN");
        row["expected_action"]=QStringLiteral("STRAIGHT_2M_THEN_STOP");
      }else{
        row["reference_distance_m"]=trial.value("reference_distance_m");
        row["expected_status"]=QStringLiteral("HUMAN_AT_OPERATOR_REFERENCE_DISTANCE");
        row["expected_action"]=legacyCondition=="1"?QStringLiteral("STOP"):QStringLiteral("AVOID_PASS_HUMAN_THEN_STOP");
      }
      row["status_match"]=true;
    }else{
      row["trial_index"]=trial.value("trial_index");
      row["goal_distance_m"]=trial.value("target_distance_m");
      row["expected_status"]=expectedLane45Status(condition);
      row["expected_action"]=expectedLane45Action(condition);
      if(row.value("initial_lane_status").toString().isEmpty())row["initial_lane_status"]=row.value("expected_status");
      const QString finalLane=row.value("left_status").toString()+QStringLiteral("|")+row.value("right_status").toString();
      row["final_lane_status"]=finalLane;
      const bool finalStatusMatch=row.value("initial_condition_match").toBool(false)&&finalLane==QStringLiteral("GREEN|GREEN");
      row["status_match"]=finalStatusMatch;row["status_error"]=finalStatusMatch?0:1;
      const double finalLaneOffset=number(snapshot.value("lane_state").toObject(),"center_error_m");
      const double finalLaneErr=std::isfinite(finalLaneOffset)?finalLaneOffset:std::numeric_limits<double>::quiet_NaN();
      row["lane_offset_reference_m"]=0.0;row["lane_offset_actual_m"]=finiteOrNull(finalLaneOffset);
      row["lane_offset_error_m"]=finiteOrNull(finalLaneErr);
      row["lane_offset_abs_error_m"]=finiteOrNull(std::isfinite(finalLaneErr)?std::abs(finalLaneErr):finalLaneErr);
    }
    if(jpeg.isEmpty())return error(QStringLiteral("CAMERA STREAM STALE / JPEG kosong saat menyimpan hasil ARMED"));
    QString imageAbs;{const QByteArray displayJpeg=renderJpegForDisplay(jpeg,row,false);const QString rel=imageRelative("4.5",condition,rows.size()+1);
      imageAbs=QDir(sessionPath(st.value("session_id").toString())).filePath(rel);QDir().mkpath(QFileInfo(imageAbs).absolutePath());
      QSaveFile im(imageAbs);if(!im.open(QIODevice::WriteOnly)||im.write(displayJpeg)!=displayJpeg.size()||!im.commit())
        return error(QStringLiteral("Gagal menyimpan gambar bukti hasil ARMED"));
      row["frame_filename"]=rel;}
    rows.append(row);c["rows"]=rows;c["sample_count"]=rows.size();
    const bool completed=trial.value("phase").toString().toLower()==QStringLiteral("complete")&&
      trial.value("stopped").toBool(false)&&trial.value("integration_pass").toBool(false);
    c["armed_complete"]=completed;
    c["last_armed_result"]=trial.value("phase");
    c["status"]=completed?QStringLiteral("SELESAI"):(c.value("dry_run_complete").toBool(false)?QStringLiteral("DRY_RUN_SELESAI"):QStringLiteral("BELUM"));
    cs[condition]=c;s["conditions"]=cs;subs["4.5"]=s;st["subchapters"]=subs;st["updated_at"]=isoNow();
    if(!writeCsvArtifacts(st,"4.5",condition)){
      writeCsvArtifacts(originalState,"4.5",condition);if(!imageAbs.isEmpty())QFile::remove(imageAbs);
      return error(QStringLiteral("Gagal menyimpan CSV hasil ARMED; perubahan di-rollback"));
    }
    if(!saveState(st)){
      writeCsvArtifacts(originalState,"4.5",condition);if(!imageAbs.isEmpty())QFile::remove(imageAbs);
      return error(QStringLiteral("Gagal menyimpan state hasil ARMED; CSV di-rollback"));
    }
    logEvent(st.value("session_id").toString(),"CAPTURE_SUCCESS","4.5",condition,"Hasil ARMED tersimpan");
    return response(st);
  }

  QJsonObject reset(const QString &sub, const QString &condition, bool wholeSubchapter) {
    QJsonObject st = current(); if (st.isEmpty()) return error(QStringLiteral("Session aktif tidak tersedia"));
    QJsonObject subs = st.value(QStringLiteral("subchapters")).toObject();
    if (!subs.contains(sub)) return error(QStringLiteral("Subbab tidak valid"));
    QJsonObject s = subs.value(sub).toObject();
    if (wholeSubchapter) {
      s[QStringLiteral("conditions")] = QJsonObject();
      removeSubchapterData(st.value("session_id").toString(), sub);
      QDir().mkpath(subchapterPath(st.value("session_id").toString(), sub));
      logEvent(st.value("session_id").toString(), QStringLiteral("RESET_SUBCHAPTER"), sub, QStringLiteral(""), QStringLiteral("Semua data subbab dihapus"));
    } else {
      QString why; if (!validCondition(sub, condition, &why)) return error(why);
      QJsonObject cs = s.value(QStringLiteral("conditions")).toObject(); cs.remove(condition);
      s[QStringLiteral("conditions")] = cs;
      const QString p = conditionPath(st.value("session_id").toString(), sub, condition);
      if (QDir(p).exists()) QDir(p).removeRecursively();
      logEvent(st.value("session_id").toString(), QStringLiteral("RESET_CONDITION"), sub, condition, QStringLiteral("Kondisi aktif dihapus"));
    }
    subs[sub] = s; st[QStringLiteral("subchapters")] = subs; st[QStringLiteral("updated_at")] = isoNow();
    if (!saveState(st)) return error(QStringLiteral("Reset gagal disimpan"));
    writeCsvArtifacts(st, sub, condition);
    return response(st);
  }

  QJsonObject reset45Group(const QString &group) {
    QJsonObject st=current();if(st.isEmpty())return error(QStringLiteral("Session aktif tidak tersedia"));
    if(group!="obstacle"&&group!="lane")return error(QStringLiteral("Group 4.5 harus obstacle atau lane"));
    QJsonObject subs=st.value(QStringLiteral("subchapters")).toObject();
    QJsonObject s=subs.value(QStringLiteral("4.5")).toObject();
    QJsonObject cs=s.value(QStringLiteral("conditions")).toObject();
    const QSet<QString> trialKeys=group=="obstacle"?
      QSet<QString>{QStringLiteral("obstacle_trial_1"),QStringLiteral("obstacle_trial_2"),QStringLiteral("obstacle_trial_3")}:
      QSet<QString>{QStringLiteral("lane_trial_1"),QStringLiteral("lane_trial_2"),QStringLiteral("lane_trial_3")};
    QStringList removed;
    for(auto it=cs.begin();it!=cs.end();){
      if(trialKeys.contains(it.key())){
        removed<<it.key();
        const QString cp=conditionPath(st.value("session_id").toString(),QStringLiteral("4.5"),it.key());
        if(QDir(cp).exists())QDir(cp).removeRecursively();
        it=cs.erase(it);
      }else ++it;
    }
    s[QStringLiteral("conditions")]=cs;
    subs[QStringLiteral("4.5")]=s;
    st[QStringLiteral("subchapters")]=subs;
    st[QStringLiteral("updated_at")]=isoNow();
    if(!saveState(st))return error(QStringLiteral("Reset group 4.5 gagal disimpan"));
    if(!writeCsvArtifacts(st,QStringLiteral("4.5"),QString()))return error(QStringLiteral("Reset group 4.5 tersimpan tetapi CSV gagal dibangun ulang"));
    logEvent(st.value("session_id").toString(),QStringLiteral("RESET_GROUP"),QStringLiteral("4.5"),group,
             QStringLiteral("Reset group %1: %2 kondisi").arg(group).arg(removed.size()));
    QJsonObject out=response(st);out[QStringLiteral("removed_conditions")]=QJsonArray::fromStringList(removed);return out;
  }

  QJsonObject save45VisualEvidence(const QString &condition,const QString &mode,int captureId,
                                   const QByteArray &mapPng,const QByteArray &combinedPng) {
    QJsonObject st=current();if(st.isEmpty())return error(QStringLiteral("Session aktif tidak tersedia"));
    const QJsonObject originalState=st;
    QString why;if(!validCondition(QStringLiteral("4.5"),condition,&why)||!condition.startsWith(QStringLiteral("obstacle_")))
      return error(QStringLiteral("Visual evidence hanya untuk 4.5A obstacle"));
    if(mode!=QStringLiteral("DRY_RUN")&&mode!=QStringLiteral("ARMED_VEHICLE_TEST"))
      return error(QStringLiteral("Mode visual evidence tidak valid"));
    if(captureId<1||mapPng.isEmpty()||combinedPng.isEmpty()||mapPng.size()>4*1024*1024||combinedPng.size()>8*1024*1024)
      return error(QStringLiteral("Payload visual evidence tidak valid"));
    if(QImage::fromData(mapPng,"PNG").isNull()||QImage::fromData(combinedPng,"PNG").isNull())
      return error(QStringLiteral("Visual evidence bukan PNG valid"));

    QJsonObject subs=st.value(QStringLiteral("subchapters")).toObject();
    QJsonObject s=subs.value(QStringLiteral("4.5")).toObject();
    QJsonObject cs=s.value(QStringLiteral("conditions")).toObject();
    QJsonObject c=cs.value(condition).toObject();QJsonArray rows=c.value(QStringLiteral("rows")).toArray();
    int rowIndex=-1;
    for(int i=rows.size()-1;i>=0;--i){const QJsonObject rr=rows.at(i).toObject();
      if(rr.value(QStringLiteral("mode")).toString()==mode&&rr.value(QStringLiteral("capture_id")).toInt(-1)==captureId){rowIndex=i;break;}
    }
    if(rowIndex<0)return error(QStringLiteral("Row capture 4.5A belum tersedia untuk visual evidence"));

    const QString id=st.value(QStringLiteral("session_id")).toString();
    const QString relBase=subchapterFolder(QStringLiteral("4.5"))+QStringLiteral("/")+
      conditionFolder(QStringLiteral("4.5"),condition)+QStringLiteral("/visual/");
    const QString stem=QStringLiteral("capture_%1_%2").arg(captureId,3,10,QChar('0')).arg(mode==QStringLiteral("DRY_RUN")?QStringLiteral("dry_run"):QStringLiteral("armed"));
    const QString mapRel=relBase+stem+QStringLiteral("_map.png");
    const QString combinedRel=relBase+stem+QStringLiteral("_combined.png");
    auto writeBytes=[&](const QString &rel,const QByteArray &bytes)->bool{
      const QString abs=QDir(sessionPath(id)).filePath(rel);QDir().mkpath(QFileInfo(abs).absolutePath());
      QSaveFile f(abs);return f.open(QIODevice::WriteOnly)&&f.write(bytes)==bytes.size()&&f.commit();
    };
    if(!writeBytes(mapRel,mapPng)||!writeBytes(combinedRel,combinedPng)){
      QFile::remove(QDir(sessionPath(id)).filePath(mapRel));QFile::remove(QDir(sessionPath(id)).filePath(combinedRel));
      return error(QStringLiteral("Gagal menyimpan map/combined evidence 4.5A"));
    }

    QJsonObject row=rows.at(rowIndex).toObject();
    row[QStringLiteral("map_filename")]=mapRel;row[QStringLiteral("combined_filename")]=combinedRel;
    row[QStringLiteral("visual_capture_at_ms")]=static_cast<double>(QDateTime::currentMSecsSinceEpoch());
    rows[rowIndex]=row;c[QStringLiteral("rows")]=rows;cs[condition]=c;s[QStringLiteral("conditions")]=cs;
    subs[QStringLiteral("4.5")]=s;st[QStringLiteral("subchapters")]=subs;st[QStringLiteral("updated_at")]=isoNow();
    if(!writeCsvArtifacts(st,QStringLiteral("4.5"),condition)||!saveState(st)){
      writeCsvArtifacts(originalState,QStringLiteral("4.5"),condition);
      QFile::remove(QDir(sessionPath(id)).filePath(mapRel));QFile::remove(QDir(sessionPath(id)).filePath(combinedRel));
      return error(QStringLiteral("Visual evidence gagal commit; file dan CSV di-rollback"));
    }
    logEvent(id,QStringLiteral("VISUAL_EVIDENCE"),QStringLiteral("4.5"),condition,
             QStringLiteral("%1 capture %2 map+combined").arg(mode).arg(captureId));
    QJsonObject out=response(st);out[QStringLiteral("map_filename")]=mapRel;out[QStringLiteral("combined_filename")]=combinedRel;return out;
  }

  QString activeSessionPath() const {
    const QJsonObject p = readJson(QDir(dataRoot_).filePath(QStringLiteral("bab4_active_session.json")));
    const QString id = p.value(QStringLiteral("session_id")).toString();
    return validSessionId(id) ? sessionPath(id) : QString();
  }

 private:
  QString dataRoot_;
  QJsonObject configSnapshot_;
  static QString isoNow() { return QDateTime::currentDateTime().toString(Qt::ISODateWithMs); }
  static QJsonObject error(const QString &m) { return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("message"), m}}; }
  static bool validSessionId(const QString &id) {
    return QRegularExpression(QStringLiteral("^BAB4_PERSEPSI_[0-9]{8}_[0-9]{6}$")).match(id).hasMatch();
  }
  QString sessionPath(const QString &id) const { return QDir(dataRoot_).filePath(id); }
  QString statePath(const QString &id) const { return QDir(sessionPath(id)).filePath(QStringLiteral("state.json")); }
  QJsonObject load(const QString &id) const { return readJson(statePath(id)); }
  QJsonObject current() {
    const QJsonObject e = ensure();
    return e.value(QStringLiteral("ok")).toBool() ? e.value(QStringLiteral("state")).toObject() : QJsonObject();
  }
  static QJsonObject readJson(const QString &path) {
    QFile f(path); if (!f.open(QIODevice::ReadOnly)) return {};
    QJsonParseError pe{}; const QJsonDocument d = QJsonDocument::fromJson(f.readAll(), &pe);
    return pe.error == QJsonParseError::NoError && d.isObject() ? d.object() : QJsonObject();
  }
  static bool saveJson(const QString &path, const QJsonObject &o) {
    QDir().mkpath(QFileInfo(path).absolutePath()); QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) return false;
    if (f.write(QJsonDocument(o).toJson(QJsonDocument::Indented)) < 0) return false;
    return f.commit();
  }
  bool saveState(const QJsonObject &st) const {
    const QString id = st.value(QStringLiteral("session_id")).toString();
    return validSessionId(id) && saveJson(statePath(id), st);
  }
  static QJsonObject response(const QJsonObject &st) {
    return QJsonObject{{QStringLiteral("ok"), !st.isEmpty()}, {QStringLiteral("state"), st},
                       {QStringLiteral("message"), st.isEmpty() ? QStringLiteral("State kosong") : QStringLiteral("OK")}};
  }
  QJsonObject initialState(const QString &id) const {
    QJsonObject subs;
    for (const QString &k : {QStringLiteral("4.1"),QStringLiteral("4.2"),QStringLiteral("4.3"),QStringLiteral("4.4"),QStringLiteral("4.5")})
      subs[k] = QJsonObject{{QStringLiteral("status"), QStringLiteral("BELUM")},
                            {QStringLiteral("conditions"), QJsonObject()}};
    return QJsonObject{{QStringLiteral("schema_version"), 6},
      {QStringLiteral("session_id"), id}, {QStringLiteral("created_at"), isoNow()},
      {QStringLiteral("updated_at"), isoNow()}, {QStringLiteral("camera_flip"), true},
      {QStringLiteral("display_mirror_x"), true},
      {QStringLiteral("coordinate_reference"), QStringLiteral("DISPLAY_OVERVIEW_MIRRORED")},
      {QStringLiteral("left_reference"), QStringLiteral("DISPLAY_LEFT")},
      {QStringLiteral("right_reference"), QStringLiteral("DISPLAY_RIGHT")},
      {QStringLiteral("vehicle_test_mode"), QStringLiteral("DRY_RUN")},
      {QStringLiteral("software_version"), QStringLiteral("bab4-perception-v6-gray-fail-closed")},
      {QStringLiteral("subchapters"), subs}};
  }
  QJsonObject metadataFor(const QJsonObject &st) const {
    return QJsonObject{{QStringLiteral("session_id"), st.value("session_id")},
      {QStringLiteral("created_at"), st.value("created_at")}, {QStringLiteral("camera_flip"), true},
      {QStringLiteral("display_mirror_x"), true},
      {QStringLiteral("coordinate_reference"), QStringLiteral("DISPLAY_OVERVIEW_MIRRORED")},
      {QStringLiteral("left_reference"), QStringLiteral("DISPLAY_LEFT")},
      {QStringLiteral("right_reference"), QStringLiteral("DISPLAY_RIGHT")},
      {QStringLiteral("software_version"), QStringLiteral("bab4-perception-v6-gray-fail-closed")},
      {QStringLiteral("config_snapshot"), configSnapshot_}};
  }
  QString subchapterFolder(const QString &sub) const {
    if (sub=="4.1") return QStringLiteral("4.1_VALIDASI_SISTEM");
    if (sub=="4.2") return QStringLiteral("4.2_HUMAN_DETECTION");
    if (sub=="4.3") return QStringLiteral("4.3_LANE_SAFETY");
    if (sub=="4.4") return QStringLiteral("4.4_REALTIME_PERFORMANCE");
    return QStringLiteral("4.5_VEHICLE_RESPONSE");
  }
  QString conditionFolder(const QString &sub, const QString &c) const {
    if (sub=="4.2" && c.startsWith("human_")) return c.mid(6);
    if (sub=="4.3") return c;
    if (sub=="4.4") return c;
    if (sub=="4.5") return c.startsWith("lane_") ? QStringLiteral("lane_response/")+c.mid(5) :
      QStringLiteral("obstacle_response/")+c;
    return QStringLiteral("validation");
  }
  QString subchapterPath(const QString &id,const QString &sub) const {
    return QDir(sessionPath(id)).filePath(subchapterFolder(sub));
  }
  QString conditionPath(const QString &id,const QString &sub,const QString &c) const {
    return QDir(subchapterPath(id,sub)).filePath(conditionFolder(sub,c));
  }
  QString imageRelative(const QString &sub,const QString &c,int n) const {
    return subchapterFolder(sub)+QStringLiteral("/") + conditionFolder(sub,c) +
      QStringLiteral("/images/capture_%1.jpg").arg(n,3,10,QChar('0'));
  }
  bool validCondition(const QString &sub,const QString &c,QString *why) const {
    bool ok=false;
    if (sub=="4.1") ok=(c=="validation");
    else if (sub=="4.2") ok=QRegularExpression("^human_((day|night)_)?[1-5]m$").match(c).hasMatch();
    else if (sub=="4.3") ok=QSet<QString>{"condition_01","condition_02","condition_03","condition_04","condition_05","condition_06"}.contains(c);
    else if (sub=="4.4") ok=QSet<QString>{"condition_A","condition_B","condition_C"}.contains(c);
    else if (sub=="4.5") ok=QRegularExpression("^(obstacle_(no_human|[1-5]m|trial_[1-3])|lane_(green_green|yellow_green|red_green|green_yellow|green_red|gray_gray|trial_([1-3]|green|yellow|red)))$").match(c).hasMatch();
    if (!ok && why) *why=QStringLiteral("Kondisi/subbab tidak valid: ")+sub+QStringLiteral("/")+c;
    return ok;
  }
  static QJsonValue finiteOrNull(double v) { return std::isfinite(v) ? QJsonValue(v) : QJsonValue(); }
  static double number(const QJsonObject &o,const QString &k) {
    const QJsonValue v=o.value(k); return v.isDouble()?v.toDouble():std::numeric_limits<double>::quiet_NaN();
  }
  static QJsonObject bestHuman(const QJsonObject &snapshot, bool evaluation, double minScore=0.0, bool requireNav2Eligible=false) {
    const QJsonObject sem=snapshot.value(QStringLiteral("semantic_detections")).toObject();
    const QJsonArray primary=evaluation && sem.value(QStringLiteral("evaluation_candidates")).isArray()
      ? sem.value(QStringLiteral("evaluation_candidates")).toArray()
      : sem.value(QStringLiteral("detections")).toArray();
    double best=-1.0; int bestPriority=-1; QJsonObject out;
    for (const QJsonValue &v : primary) {
      const QJsonObject d=v.toObject(); const QString name=d.value("class_name").toString().toLower();
      if (d.value("class_id").toInt(-1)!=0 && name!="person" && name!="human") continue;
      if(requireNav2Eligible && d.contains("nav2_eligible") && !d.value("nav2_eligible").toBool(false)) continue;
      const double score=d.value("score").toDouble(-1.0);
      const bool safetyTracked=requireNav2Eligible&&d.value("safety_human_valid").toBool(false);
      if(score<minScore&&!safetyTracked) continue;
      const QString tracking=d.value("tracking_state").toString().toUpper();
      const int priority=safetyTracked?2:((tracking=="STRONG"||tracking=="TRACKED"||tracking=="HELD")?1:0);
      if(priority>bestPriority||(priority==bestPriority&&score>best)){bestPriority=priority;best=score;out=d;}
    }
    return out;
  }
  static QString laneExpectedAction(const QString &l,const QString &r,bool leftLatched=false,bool rightLatched=false) {
    if(l=="GREEN"&&r=="GREEN") return QStringLiteral("STRAIGHT_FORWARD_2M_THEN_STOP");
    // Production semantics: standalone YELLOW is warning-only. Steering authority
    // starts at RED/touch; YELLOW continues recenter only while the RED latch remains active.
    if(l=="YELLOW"&&r=="GREEN") return leftLatched ?
      QStringLiteral("RECENTER_RIGHT_LATCHED_RECOVERY_UNTIL_GREEN_THEN_FORWARD_2M_STOP") :
      QStringLiteral("WARNING_LEFT_NO_STEER_FORWARD_2M_THEN_STOP");
    if(l=="GREEN"&&r=="YELLOW") return rightLatched ?
      QStringLiteral("RECENTER_LEFT_LATCHED_RECOVERY_UNTIL_GREEN_THEN_FORWARD_2M_STOP") :
      QStringLiteral("WARNING_RIGHT_NO_STEER_FORWARD_2M_THEN_STOP");
    if(l=="RED"&&r=="GREEN") return QStringLiteral("RECENTER_RIGHT_UNTIL_GREEN_THEN_STRAIGHT_TO_FORWARD_2M_STOP");
    if(l=="GREEN"&&r=="RED") return QStringLiteral("RECENTER_LEFT_UNTIL_GREEN_THEN_STRAIGHT_TO_FORWARD_2M_STOP");
    if(l=="GRAY"&&r=="GRAY") return QStringLiteral("STOP");
    return QStringLiteral("INVALID_LANE_STATE");
  }
  static QString obstacleExpectedAction(const QString &decision,bool detected) {
    const QString d=decision.toUpper();
    if(d.contains("AVOID")) return QStringLiteral("AVOID_THEN_STOP");
    if(d.contains("STOP")||d.contains("HARD")) return QStringLiteral("STOP");
    return detected?QStringLiteral("STOP_OR_AVOID_REQUIRED"):QStringLiteral("NO_OBSTACLE_ACTION");
  }
  static QString expectedLaneCondition(const QString &condition) {
    if(condition=="condition_01") return "GREEN|GREEN";
    if(condition=="condition_02") return "YELLOW|GREEN";
    if(condition=="condition_03") return "RED|GREEN";
    if(condition=="condition_04") return "GREEN|YELLOW";
    if(condition=="condition_05") return "GREEN|RED";
    if(condition=="condition_06") return "GRAY|GRAY";
    return "";
  }
  static QString expectedLane45Status(const QString &condition) {
    if(QRegularExpression(QStringLiteral("^lane_trial_[1-3]$")).match(condition).hasMatch())
      return "RED|ANY_SIDE_START -> GREEN|GREEN_FINAL";
    if(condition=="lane_trial_green") return "GREEN|GREEN";
    if(condition=="lane_trial_yellow") return "YELLOW|ANY_SIDE";
    if(condition=="lane_trial_red") return "RED|ANY_SIDE";
    if(condition=="lane_green_green") return "GREEN|GREEN";
    if(condition=="lane_yellow_green") return "YELLOW|GREEN";
    if(condition=="lane_red_green") return "RED|GREEN";
    if(condition=="lane_green_yellow") return "GREEN|YELLOW";
    if(condition=="lane_green_red") return "GREEN|RED";
    if(condition=="lane_gray_gray") return "GRAY|GRAY";
    return "";
  }
  static QString expectedLane45Action(const QString &condition) {
    if(QRegularExpression(QStringLiteral("^lane_trial_[1-3]$")).match(condition).hasMatch())
      return QStringLiteral("AUTO_SIDE_RED_RECENTER_GREEN_RESTORE_INITIAL_HEADING_FORWARD_5M_STOP");
    if(condition==QStringLiteral("lane_trial_green"))return QStringLiteral("STRAIGHT_FORWARD_2M_THEN_STOP");
    if(condition==QStringLiteral("lane_trial_yellow"))return QStringLiteral("YELLOW_WARNING_ANY_SIDE_NO_STEER_THEN_STOP");
    if(condition==QStringLiteral("lane_trial_red"))return QStringLiteral("RED_ANY_SIDE_RECENTER_TO_GREEN_THEN_STOP");
    if(condition==QStringLiteral("lane_gray_gray"))return QStringLiteral("STOP_FAIL_CLOSED");
    const QString s=expectedLane45Status(condition);
    const QStringList p=s.split('|');
    return p.size()==2?laneExpectedAction(p[0],p[1],false,false):QStringLiteral("INVALID_LANE_STATE");
  }
  QJsonObject buildRow(const QString &sub,const QString &condition,const QJsonObject &req,
                       const QJsonObject &snap,int sample) const {
    const qint64 now=QDateTime::currentMSecsSinceEpoch();
    const QJsonObject updated=snap.value(QStringLiteral("__updated")).toObject();
    const QJsonObject frame=snap.value(QStringLiteral("camera_frame")).toObject();
    const QJsonObject perf=snap.value(QStringLiteral("perception_performance")).toObject();
    const bool is45=sub==QStringLiteral("4.5");
    const QJsonObject human=bestHuman(snap, sub==QStringLiteral("4.2"), is45?0.20:0.0, is45);
    const QJsonObject lane=snap.value(QStringLiteral("lane_state")).toObject();
    const QJsonObject corridor=lane.value(QStringLiteral("corridor")).toObject();
    const QJsonObject safety=snap.value(QStringLiteral("trajectory_safety_state")).toObject();
    const double frameStamp=frame.value("at_ms").toDouble(updated.value("camera_frame").toDouble(0));
    QJsonObject r{{"capture_id",sample},{"timestamp",isoNow()},{"server_time_ms",static_cast<double>(now)},
      {"frame_id",QString::number(static_cast<qint64>(frameStamp))},{"frame_timestamp_ms",frameStamp},
      {"frame_age_ms",finiteOrNull(number(perf,"frame_age_ms"))},{"fps",finiteOrNull(number(perf,"pipeline_fps"))},
      {"inference_ms",finiteOrNull(number(perf,"inference_ms"))},{"processing_ms",finiteOrNull(number(perf,"pipeline_ms"))},
      {"condition",condition},{"camera_flip",true},{"display_mirror_x",true},{"coordinate_reference","DISPLAY_OVERVIEW_MIRRORED"}};
    if(sub=="4.1"){
      const double semAt=updated.value("semantic_detections").toDouble(0),laneAt=updated.value("lane_state").toDouble(0);
      r["camera_stream"]=snap.value("connected.camera").toBool(false); r["camera_healthy"]=snap.value("camera_healthy").toBool(false);
      r["orientation_left_ok"]=req.value("orientation_left_ok").toBool(false); r["orientation_right_ok"]=req.value("orientation_right_ok").toBool(false);
      r["stream_status"]=(frameStamp>0&&now-frameStamp<1500)?QStringLiteral("LIVE"):QStringLiteral("STALE");
      r["yolop_active"]=perf.value("inference_enabled").toBool(snap.value("perception_inference_enabled").toBool(false));
      r["semantic_stream"]=(semAt>0&&now-semAt<1500)?QStringLiteral("LIVE"):QStringLiteral("STALE");
      r["semantic_status"]=snap.value("semantic_status").toObject().value("state").toString("UNKNOWN");
      r["lane_stream"]=(laneAt>0&&now-laneAt<1500)?QStringLiteral("LIVE"):QStringLiteral("STALE");
      const double fpsRef=number(perf,"target_fps");
      const double fpsActual=number(perf,"pipeline_fps");
      const double fpsErr=(std::isfinite(fpsRef)&&std::isfinite(fpsActual))?fpsActual-fpsRef:std::numeric_limits<double>::quiet_NaN();
      r["reference_type"]=QStringLiteral("TARGET_INFERENCE_FPS");
      r["reference_source"]=QStringLiteral("perception_performance.target_fps");
      r["actual_source"]=QStringLiteral("perception_performance.pipeline_fps");
      r["reference_value"]=finiteOrNull(fpsRef);r["actual_value"]=finiteOrNull(fpsActual);
      r["signed_error"]=finiteOrNull(fpsErr);r["abs_error"]=finiteOrNull(std::isfinite(fpsErr)?std::abs(fpsErr):fpsErr);
      r["fps_reference_hz"]=finiteOrNull(fpsRef);r["fps_actual_hz"]=finiteOrNull(fpsActual);
      r["fps_error_hz"]=finiteOrNull(fpsErr);r["fps_abs_error_hz"]=finiteOrNull(std::isfinite(fpsErr)?std::abs(fpsErr):fpsErr);
    } else if(sub=="4.2"){      const bool detected=!human.isEmpty(); const double ref=req.value("reference_distance_m").toDouble();
      QString lighting=req.value("lighting_condition").toString().toLower();
      if(lighting!="day"&&lighting!="night"){
        if(condition.startsWith("human_day_"))lighting="day";
        else lighting="night"; // legacy human_1m..5m = historical night dataset
      }
      const double est=human.value("calibrated_forward_m").isDouble()?human.value("calibrated_forward_m").toDouble():human.value("forward_m").toDouble(std::numeric_limits<double>::quiet_NaN());
      const QJsonObject sem=snap.value("semantic_detections").toObject();
      const QJsonObject cameraHealth=snap.value("camera_health_state").toObject();
      const int imageW=sem.value("image_width").toInt(0);
      const int imageH=sem.value("image_height").toInt(0);
      const double x1=human.value("x1").toDouble(),x2=human.value("x2").toDouble();
      const double y1=human.value("y1").toDouble(),y2=human.value("y2").toDouble();
      const double bboxW=detected?std::max(0.0,x2-x1):std::numeric_limits<double>::quiet_NaN();
      const double bboxH=detected?std::max(0.0,y2-y1):std::numeric_limits<double>::quiet_NaN();
      const double hfrac=human.value("bbox_height_fraction").isDouble()?human.value("bbox_height_fraction").toDouble():
        ((detected&&imageH>0)?bboxH/static_cast<double>(imageH):std::numeric_limits<double>::quiet_NaN());
      const double wfrac=(detected&&imageW>0)?bboxW/static_cast<double>(imageW):std::numeric_limits<double>::quiet_NaN();
      const bool bottomClipped=detected&&human.value("bottom_clipped").toBool(false);
      const bool bboxSaturated=detected&&(bottomClipped||(std::isfinite(hfrac)&&hfrac>=0.95));
      const double displayX1=imageW>0?static_cast<double>(imageW-1)-x2:x1;
      r["lighting_condition"]=lighting;
      r["illumination_label"]=lighting=="day"?QStringLiteral("SIANG_BANYAK_CAHAYA"):QStringLiteral("MALAM_MINIM_CAHAYA");
      r["illumination_mean_luma"]=cameraHealth.value("mean_luma").isDouble()?cameraHealth.value("mean_luma"):sem.value("camera_mean_luma");
      r["illumination_stddev_luma"]=cameraHealth.value("stddev_luma").isDouble()?cameraHealth.value("stddev_luma"):sem.value("camera_stddev");
      r["illumination_extreme_fraction"]=cameraHealth.value("extreme_fraction");
      r["illumination_mean_gradient"]=cameraHealth.value("mean_gradient");
      r["camera_health_reason"]=cameraHealth.value("reason");
      r["reference_distance_m"]=ref;r["detected_class"]=detected?QStringLiteral("human"):QStringLiteral("none");r["detected"]=detected?1:0;
      r["semantic_updated_ms"]=updated.value("semantic_detections");r["semantic_inference_ms"]=sem.value("inference_ms");
      r["evaluation_candidate_count"]=sem.value("evaluation_candidate_count");r["detector_source"]=sem.value("source");
      r["detection_source"]=QStringLiteral("evaluation_candidates");
      r["confidence"]=detected?human.value("score"):QJsonValue(); r["bbox_x"]=detected?QJsonValue(displayX1):QJsonValue();r["bbox_y"]=detected?QJsonValue(y1):QJsonValue();
      r["bbox_width"]=finiteOrNull(bboxW);r["bbox_height"]=finiteOrNull(bboxH);
      r["bbox_width_fraction"]=finiteOrNull(wfrac);r["bbox_height_fraction"]=finiteOrNull(hfrac);
      r["stable_bbox_height_fraction"]=detected?human.value("stable_bbox_height_fraction"):QJsonValue();
      r["bottom_clipped"]=detected?QJsonValue(bottomClipped):QJsonValue();
      r["bbox_saturated"]=detected?QJsonValue(bboxSaturated):QJsonValue();
      r["distance_quality"]=!detected?QStringLiteral("NO_DETECTION"):
        (bboxSaturated?QStringLiteral("SATURATED_NEAR"):
         (std::isfinite(est)?QStringLiteral("VALID_BBOX_GEOMETRY"):QStringLiteral("METRIC_INVALID")));
      r["image_width"]=imageW;r["image_height"]=imageH;
      r["raw_forward_m"]=detected?human.value("raw_forward_m"):QJsonValue();
      r["raw_left_m"]=detected?human.value("raw_left_m"):QJsonValue();
      r["bbox_calibrated_forward_m"]=detected?human.value("bbox_calibrated_forward_m"):QJsonValue();
      r["legacy_affine_forward_m"]=detected?human.value("legacy_affine_forward_m"):QJsonValue();
      r["calibrated_forward_m"]=detected?human.value("calibrated_forward_m"):QJsonValue();
      r["stable_forward_m"]=detected?human.value("stable_forward_m"):QJsonValue();
      r["nav2_forward_m"]=detected?human.value("nav2_forward_m"):QJsonValue();
      r["nav2_distance_source"]=detected?human.value("nav2_distance_source"):QJsonValue();
      r["metric_match_mode"]=detected?human.value("metric_match_mode"):QJsonValue();
      const double distanceErr=(std::isfinite(est)&&std::isfinite(ref))?est-ref:std::numeric_limits<double>::quiet_NaN();
      r["estimated_distance_m"]=finiteOrNull(est);r["distance_error_m"]=finiteOrNull(distanceErr);
      r["abs_distance_error_m"]=finiteOrNull(std::isfinite(distanceErr)?std::abs(distanceErr):distanceErr);
      r["reference_type"]=QStringLiteral("REFERENCE_DISTANCE_METERAN");
      r["reference_source"]=QStringLiteral("operator_meter_measurement");
      r["actual_source"]=QStringLiteral("semantic_human_calibrated_forward_m");
      r["reference_value"]=finiteOrNull(ref);r["actual_value"]=finiteOrNull(est);
      r["signed_error"]=finiteOrNull(distanceErr);
      r["abs_error"]=finiteOrNull(std::isfinite(distanceErr)?std::abs(distanceErr):distanceErr);
      r["prediction_stamp_ns"]=sem.value("stamp_ns");
    } else if(sub=="4.3"){
      const bool invalid=!corridor.value("drivable_detected").toBool(false);
      const QString l=invalid?QStringLiteral("GRAY"):corridor.value("left_status").toString("UNKNOWN").toUpper();
      const QString rr=invalid?QStringLiteral("GRAY"):corridor.value("right_status").toString("UNKNOWN").toUpper();
      const double rawOffset=number(lane,"center_error_m");
      r["lane_updated_ms"]=updated.value("lane_state");
      r["condition_id"]=condition;r["left_status"]=l;r["right_status"]=rr;
      r["left_gap_m"]=invalid?QJsonValue():corridor.value("left_gap_m_est");r["right_gap_m"]=invalid?QJsonValue():corridor.value("right_gap_m_est");
      r["left_gap_px"]=invalid?QJsonValue():corridor.value("left_gap_px");r["right_gap_px"]=invalid?QJsonValue():corridor.value("right_gap_px");
      r["vehicle_offset_m"]=invalid?QJsonValue():finiteOrNull(rawOffset);r["drivable_width_m"]=finiteOrNull(number(lane,"road_width_m"));
      r["left_lane_confidence"]=lane.value("confidence");r["right_lane_confidence"]=lane.value("confidence");
      r["recommended_action"]=invalid?QStringLiteral("STOP_NO_DRIVABLE"):corridor.value("recommendation").toString();
      const QString expectedStatus=expectedLaneCondition(condition);
      const QString actualStatus=l+QStringLiteral("|")+rr;
      const bool statusMatch=actualStatus==expectedStatus;
      const double laneOffsetActual=invalid?std::numeric_limits<double>::quiet_NaN():rawOffset;
      const double centeringErr=std::isfinite(laneOffsetActual)?laneOffsetActual:std::numeric_limits<double>::quiet_NaN();
      r["expected_status"]=expectedStatus;r["actual_status"]=actualStatus;r["status_match"]=statusMatch;
      r["status_error"]=statusMatch?0:1;
      r["lane_offset_reference_m"]=0.0;r["lane_offset_actual_m"]=finiteOrNull(laneOffsetActual);
      r["centering_error_m"]=finiteOrNull(centeringErr);
      r["abs_centering_error_m"]=finiteOrNull(std::isfinite(centeringErr)?std::abs(centeringErr):centeringErr);
      r["reference_type"]=QStringLiteral("LANE_CENTER_OFFSET_M");
      r["reference_source"]=QStringLiteral("lane_center_definition");
      r["actual_source"]=QStringLiteral("lane_state.center_error_m");
      r["reference_value"]=0.0;r["actual_value"]=finiteOrNull(laneOffsetActual);
      r["signed_error"]=finiteOrNull(centeringErr);
      r["abs_error"]=finiteOrNull(std::isfinite(centeringErr)?std::abs(centeringErr):centeringErr);
    } else if(sub=="4.4"){
      const double age=number(perf,"frame_age_ms"); const bool live=std::isfinite(age)&&age>=0&&age<1500;
      r["elapsed_s"]=req.value("elapsed_s");r["subscriber_latency_ms"]=req.value("subscriber_latency_ms");      r["stream_gap"]=req.value("stream_gap");r["dropped_frame"]=req.value("dropped_frame");
      int humans=0;for(const QJsonValue &v:snap.value("semantic_detections").toObject().value("detections").toArray()){
        const QJsonObject z=v.toObject();const QString n=z.value("class_name").toString().toLower();
        if(z.value("class_id").toInt(-1)==0||n=="person"||n=="human")++humans;
      }
      r["human_count"]=humans;r["lane_valid"]=lane.value("valid").toBool(lane.value("drivable_valid").toBool(false));
      r["drivable_valid"]=lane.value("drivable_valid").toBool(false);r["stream_status"]=live?QStringLiteral("LIVE"):QStringLiteral("STALE");
      const double fpsRef=number(perf,"target_fps");
      const double fpsActual=number(perf,"pipeline_fps");
      const double fpsErr=(std::isfinite(fpsRef)&&std::isfinite(fpsActual))?fpsActual-fpsRef:std::numeric_limits<double>::quiet_NaN();
      const double infRefMs=(std::isfinite(fpsRef)&&fpsRef>0.0)?1000.0/fpsRef:std::numeric_limits<double>::quiet_NaN();
      const double infActualMs=number(perf,"inference_ms");
      const double infErr=(std::isfinite(infRefMs)&&std::isfinite(infActualMs))?infActualMs-infRefMs:std::numeric_limits<double>::quiet_NaN();
      const double captureRef=number(perf,"capture_source_fps_target");
      const double captureActual=number(perf,"capture_fps");
      const double captureErr=(std::isfinite(captureRef)&&std::isfinite(captureActual))?captureActual-captureRef:std::numeric_limits<double>::quiet_NaN();
      r["reference_type"]=QStringLiteral("TARGET_INFERENCE_FPS");
      r["reference_source"]=QStringLiteral("perception_performance.target_fps");
      r["actual_source"]=QStringLiteral("perception_performance.pipeline_fps");
      r["reference_value"]=finiteOrNull(fpsRef);r["actual_value"]=finiteOrNull(fpsActual);
      r["signed_error"]=finiteOrNull(fpsErr);r["abs_error"]=finiteOrNull(std::isfinite(fpsErr)?std::abs(fpsErr):fpsErr);
      r["fps_reference_hz"]=finiteOrNull(fpsRef);r["fps_actual_hz"]=finiteOrNull(fpsActual);
      r["fps_error_hz"]=finiteOrNull(fpsErr);r["fps_abs_error_hz"]=finiteOrNull(std::isfinite(fpsErr)?std::abs(fpsErr):fpsErr);
      r["fps_performance_margin_hz"]=finiteOrNull(fpsErr);
      r["inference_time_reference_ms"]=finiteOrNull(infRefMs);r["inference_time_actual_ms"]=finiteOrNull(infActualMs);
      r["inference_time_error_ms"]=finiteOrNull(infErr);
      r["inference_time_abs_error_ms"]=finiteOrNull(std::isfinite(infErr)?std::abs(infErr):infErr);
      r["inference_time_margin_ms"]=finiteOrNull(std::isfinite(infErr)?-infErr:infErr);
      r["capture_fps_reference_hz"]=finiteOrNull(captureRef);r["capture_fps_actual_hz"]=finiteOrNull(captureActual);
      r["capture_fps_error_hz"]=finiteOrNull(captureErr);
    } else if(sub=="4.5"){
      const bool detected=!human.isEmpty();
      const bool hasVisual=corridor.value("drivable_detected").toBool(false)||corridor.value("lane_mask_detected").toBool(false)||corridor.value("raw_lane_mask_detected").toBool(false);
      const QString evidence=corridor.value("evidence_mode").toString().toUpper();
      const QString recommendation=corridor.value("recommendation").toString().toUpper();
      const QString laneState=lane.value("state").toString("UNKNOWN").toUpper();
      const bool laneValid=lane.value("valid").toBool(false)&&corridor.value("drivable_detected").toBool(false);
      const bool invalid=!laneValid||evidence.contains("NO_EVIDENCE")||recommendation=="NO_MASK_EVIDENCE"||(!hasVisual&&(laneState=="UNKNOWN"||laneState=="LANE_LOST"));
      const bool leftLatched=corridor.value("left_recenter_latched").toBool(false);
      const bool rightLatched=corridor.value("right_recenter_latched").toBool(false);
      const QString l=invalid?QStringLiteral("GRAY"):corridor.value("left_status").toString("UNKNOWN").toUpper();
      const QString rr=invalid?QStringLiteral("GRAY"):corridor.value("right_status").toString("UNKNOWN").toUpper();
      const QString decision=safety.value("decision").toString();
      const bool laneMode=condition.startsWith("lane_"); const bool noHuman=condition=="obstacle_no_human";
      const double laneOffsetActual=number(lane,"center_error_m");
      const double laneCenterErr=std::isfinite(laneOffsetActual)?laneOffsetActual:std::numeric_limits<double>::quiet_NaN();
      r["mode"]="DRY_RUN";r["left_status"]=l;r["right_status"]=rr;
      r["lane_offset_reference_m"]=laneMode?QJsonValue(0.0):QJsonValue();
      r["lane_offset_actual_m"]=laneMode?finiteOrNull(laneOffsetActual):QJsonValue();
      r["lane_offset_error_m"]=laneMode?finiteOrNull(laneCenterErr):QJsonValue();
      r["lane_offset_abs_error_m"]=laneMode?finiteOrNull(std::isfinite(laneCenterErr)?std::abs(laneCenterErr):laneCenterErr):QJsonValue();
      r["lane_state"]=laneState;r["lane_recommendation"]=recommendation;r["lane_valid"]=laneValid;
      r["lane_evidence_source"]=lane.value("evidence_source");
      r["lane_corridor_evidence_mode"]=corridor.value("evidence_mode");
      r["lane_left_boundary_source"]=corridor.value("left_boundary_source");
      r["lane_right_boundary_source"]=corridor.value("right_boundary_source");
      r["lane_left_gap_px"]=corridor.value("left_gap_px");
      r["lane_right_gap_px"]=corridor.value("right_gap_px");
      r["lane_left_active_samples"]=corridor.value("left_samples");
      r["lane_right_active_samples"]=corridor.value("right_samples");
      r["lane_left_drivable_edge_samples"]=corridor.value("left_drivable_edge_samples");
      r["lane_right_drivable_edge_samples"]=corridor.value("right_drivable_edge_samples");
      r["lane_classical_fallback_active"]=lane.value("classical_fallback_active");
      r["lane_classical_left_samples"]=lane.value("classical_left_samples");
      r["lane_classical_right_samples"]=lane.value("classical_right_samples");
      r["lane_left_latched"]=leftLatched;r["lane_right_latched"]=rightLatched;
      r["drivable_valid"]=laneValid&&!invalid;r["lane_invalid"]=invalid;
      r["source_updated_ms"]=updated.value(laneMode?"lane_state":"semantic_detections");
      const QJsonObject sem=snap.value("semantic_detections").toObject();const int imageW=sem.value("image_width").toInt(0);
      const double hx1=human.value("x1").toDouble(),hx2=human.value("x2").toDouble();
      const double displayX1=imageW>0?static_cast<double>(imageW-1)-hx2:hx1;
      const double displayX2=imageW>0?static_cast<double>(imageW-1)-hx1:hx2;
      const double rawHd=human.value("raw_forward_m").isDouble()?human.value("raw_forward_m").toDouble():human.value("forward_m").toDouble(std::numeric_limits<double>::quiet_NaN());
      const double bboxHd=human.value("bbox_calibrated_forward_m").isDouble()?human.value("bbox_calibrated_forward_m").toDouble():std::numeric_limits<double>::quiet_NaN();
      const double legacyHd=human.value("legacy_affine_forward_m").isDouble()?human.value("legacy_affine_forward_m").toDouble():std::numeric_limits<double>::quiet_NaN();
      const double calibratedHd=human.value("calibrated_forward_m").isDouble()?human.value("calibrated_forward_m").toDouble():std::numeric_limits<double>::quiet_NaN();
      const double nav2Hd=human.value("nav2_forward_m").isDouble()?human.value("nav2_forward_m").toDouble():
        (std::isfinite(calibratedHd)?calibratedHd:rawHd);
      const double stableHd=human.value("stable_forward_m").isDouble()?human.value("stable_forward_m").toDouble():nav2Hd;
      const double rawLeft=human.value("raw_left_m").isDouble()?human.value("raw_left_m").toDouble():std::numeric_limits<double>::quiet_NaN();
      const double hd=stableHd;
      r["human_detected"]=detected;r["detected"]=detected?1:0;r["human_distance"]=finiteOrNull(hd);r["human_confidence"]=detected?human.value("score"):QJsonValue();
      r["raw_distance_m"]=finiteOrNull(rawHd);
      r["bbox_calibrated_distance_m"]=finiteOrNull(bboxHd);
      r["legacy_affine_distance_m"]=finiteOrNull(legacyHd);
      r["calibrated_distance_m"]=finiteOrNull(calibratedHd);
      r["stable_distance_m"]=finiteOrNull(stableHd);
      r["nav2_distance_m"]=finiteOrNull(nav2Hd);
      r["nav2_distance_source"]=human.value("nav2_distance_source");
      r["distance_filter_state"]=human.value("distance_filter_state");
      r["distance_history_count"]=human.value("distance_history_count");
      r["stable_bbox_height_fraction"]=human.value("stable_bbox_height_fraction");
      r["obstacle_memory_trial_active"]=sem.value("obstacle_memory_trial_active");
      r["obstacle_memory_latched"]=sem.value("obstacle_memory_latched");
      r["obstacle_memory_released"]=sem.value("obstacle_memory_released");
      r["obstacle_memory_release_count"]=sem.value("obstacle_memory_release_count");
      r["obstacle_memory_odom_x"]=sem.value("obstacle_memory_odom_x");
      r["obstacle_memory_odom_y"]=sem.value("obstacle_memory_odom_y");
      r["obstacle_memory_base_forward_m"]=sem.value("obstacle_memory_base_forward_m");
      r["lateral_position_m"]=finiteOrNull(rawLeft);
      r["human_detection_source"]=detected?
        (human.value("tracking_state").toString().isEmpty()?QStringLiteral("semantic_safety_detection"):
         QStringLiteral("semantic_safety_")+human.value("tracking_state").toString().toLower()):
        QStringLiteral("none");
      r["human_tracking_state"]=human.value("tracking_state");
      r["human_tracking_promoted"]=human.value("tracking_promoted");
      r["human_temporal_hold"]=human.value("temporal_hold");
      r["human_temporal_hold_age_ms"]=human.value("temporal_hold_age_ms");
      r["human_strong_age_ms"]=human.value("strong_age_ms");
      r["bbox_x"]=detected?QJsonValue(displayX1):QJsonValue();r["bbox_y"]=detected?human.value("y1"):QJsonValue();
      r["bbox_width"]=detected?QJsonValue(displayX2-displayX1):QJsonValue();
      r["bbox_height"]=detected?QJsonValue(human.value("y2").toDouble()-human.value("y1").toDouble()):QJsonValue();
      r["image_width"]=imageW;r["image_height"]=sem.value("image_height");
      const QJsonObject objectCloud=snap.value("object_points").toObject();
      const QJsonObject pathCloud=snap.value("path_relevant_points").toObject();
      const QJsonObject planningCloud=snap.value("planning_relevant_points").toObject();
      const QJsonObject localCostmap=snap.value("local_costmap_meta").toObject();
      const QJsonObject mppi=snap.value("mppi_trajectories").toObject();
      const int objectCount=static_cast<int>(objectCloud.value("count").toDouble(0));
      const int pathCount=static_cast<int>(pathCloud.value("count").toDouble(0));
      const int planningCount=static_cast<int>(planningCloud.value("count").toDouble(0));
      const bool metricValid=detected&&std::isfinite(hd);
      const bool metricSafety=safety.value("metric_obstacle_safety").toBool(false);
      const double costmapAt=updated.value("local_costmap_meta").toDouble(0);
      const bool localCostmapFresh=costmapAt>0&&static_cast<double>(now)-costmapAt<2200.0&&!localCostmap.isEmpty();
      const double mppiAt=updated.value("mppi_trajectories").toDouble(0);
      const bool mppiFresh=mppiAt>0&&static_cast<double>(now)-mppiAt<1800.0&&mppi.value("count").toDouble(0)>0;
      const bool costmapAligned=localCostmap.value("planning_alignment_pass").toBool(false);
      const bool costmapMarked=planningCount>0&&localCostmapFresh&&costmapAligned&&
        localCostmap.value("nonzero_cells").toDouble(0)>0&&localCostmap.value("max_cost").toDouble(0)>0;
      const QString decisionUpper=decision.toUpper();
      const bool avoidanceRuntime=decisionUpper.startsWith("AVOID_")||decisionUpper.startsWith("SEARCH_AVOIDANCE_")||
        decisionUpper==QStringLiteral("PATH_OBSTACLE_SLOW_SEARCH");
      r["perception_decision"]=decision;
      r["metric_position_valid"]=metricValid;
      r["metric_obstacle_safety"]=metricSafety;
      r["object_points_count"]=objectCount;
      r["path_relevant_points_count"]=pathCount;
      r["planning_relevant_points_count"]=planningCount;
      r["local_costmap_fresh"]=localCostmapFresh;
      r["costmap_nonzero_cells"]=localCostmap.value("nonzero_cells");
      r["costmap_lethal_cells"]=localCostmap.value("lethal_cells");
      r["costmap_max_cost"]=localCostmap.value("max_cost");
      r["costmap_source"]=localCostmap.value("source");
      r["costmap_sync_valid"]=localCostmap.value("planning_alignment_valid");
      r["costmap_sync_pass"]=costmapAligned;
      r["costmap_sync_min_m"]=localCostmap.value("planning_alignment_min_m");
      r["costmap_sync_max_m"]=localCostmap.value("planning_alignment_max_m");
      r["costmap_sync_threshold_m"]=localCostmap.value("planning_alignment_threshold_m");
      r["costmap_marked"]=costmapMarked;
      r["mppi_trajectory_count"]=mppi.value("count");
      r["mppi_optimal_count"]=mppi.value("optimal_count");
      r["mppi_active"]=mppiFresh;
      r["avoidance_runtime"]=avoidanceRuntime;
      r["integration_pass"]=false;
      if(laneMode){
        const QString expectedStatus=expectedLane45Status(condition);
        const QRegularExpression laneTrialRe(QStringLiteral("^lane_trial_([1-3])$"));
        const auto laneTrialMatch=laneTrialRe.match(condition);
        const bool repeatLaneTrial=laneTrialMatch.hasMatch();
        const bool trialGreen=condition==QStringLiteral("lane_trial_green");
        const bool trialYellow=condition==QStringLiteral("lane_trial_yellow");
        const bool trialRed=condition==QStringLiteral("lane_trial_red");
        const bool actualGreen=l=="GREEN"&&rr=="GREEN";
        const bool actualYellow=(l=="YELLOW"&&rr=="GREEN")||(l=="GREEN"&&rr=="YELLOW");
        const bool actualRed=(l=="RED"&&rr=="GREEN")||(l=="GREEN"&&rr=="RED");
        const bool repeatLaneLatchOk=(l=="RED"&&leftLatched)||(rr=="RED"&&rightLatched);
        const bool statusMatch=repeatLaneTrial?(actualRed&&repeatLaneLatchOk):
          trialGreen?actualGreen:trialYellow?actualYellow:trialRed?actualRed:
          ((l+QStringLiteral("|")+rr)==expectedStatus);
        const bool grayCondition=condition==QStringLiteral("lane_gray_gray");
        const bool warningCondition=trialYellow||condition==QStringLiteral("lane_yellow_green")||condition==QStringLiteral("lane_green_yellow");
        const bool redCondition=repeatLaneTrial||trialRed||condition==QStringLiteral("lane_red_green")||condition==QStringLiteral("lane_green_red");
        const bool warningLatchClear=trialYellow?((l=="YELLOW"&&!leftLatched)||(rr=="YELLOW"&&!rightLatched)):
          condition==QStringLiteral("lane_yellow_green")?!leftLatched:
          condition==QStringLiteral("lane_green_yellow")?!rightLatched:true;
        const bool redLatchActive=repeatLaneTrial?repeatLaneLatchOk:
          trialRed?((l=="RED"&&leftLatched)||(rr=="RED"&&rightLatched)):
          condition==QStringLiteral("lane_red_green")?leftLatched:
          condition==QStringLiteral("lane_green_red")?rightLatched:true;
        const bool semanticMatch=grayCondition?invalid:
          (laneValid&&!invalid&&(!warningCondition||warningLatchClear)&&(!redCondition||redLatchActive));
        if(repeatLaneTrial){r["trial_index"]=laneTrialMatch.captured(1).toInt();r["target_distance_m"]=5.0;}
        r["expected_status"]=expectedStatus;
        r["actual_status"]=l+QStringLiteral("|")+rr;
        r["status_match"]=statusMatch;
        r["status_error"]=statusMatch?0:1;
        r["reference_type"]=QStringLiteral("LANE_CENTER_OFFSET_M");
        r["reference_source"]=QStringLiteral("lane_center_definition");
        r["actual_source"]=QStringLiteral("lane_state.center_error_m");
        r["reference_value"]=0.0;r["actual_value"]=finiteOrNull(laneOffsetActual);
        r["signed_error"]=finiteOrNull(laneCenterErr);
        r["abs_error"]=finiteOrNull(std::isfinite(laneCenterErr)?std::abs(laneCenterErr):laneCenterErr);
        r["actual_action"]=laneExpectedAction(l,rr,leftLatched,rightLatched);
        r["expected_action"]=expectedLane45Action(condition);
        r["integration_pass"]=statusMatch&&semanticMatch;
      }else if(noHuman){
        const bool clearPath=pathCount==0;
        const bool mppiOptimal=mppi.value("optimal_count").toDouble(0)>0;
        r["expected_status"]=QStringLiteral("NO_HUMAN_PATH_CLEAR");
        r["status_match"]=!detected&&clearPath;
        r["reference_distance_m"]=QJsonValue();
        r["distance_error_m"]=QJsonValue();
        r["path_clear"]=clearPath;
        r["off_path_planning_points_count"]=std::max(0,planningCount-pathCount);
        r["actual_action"]=detected?QStringLiteral("HUMAN_PRESENT"):
          (clearPath?QStringLiteral("CLEAR_PATH_FORWARD"):QStringLiteral("PATH_OBSTACLE_PRESENT"));
        r["expected_action"]=QStringLiteral("STRAIGHT_2M_THEN_STOP");
        // NO HUMAN qualifies the forward route, not the entire local costmap.
        // Off-path objects must remain visible to MPPI/costmap but do not fail
        // the baseline as long as no path-relevant obstacle intersects the route.
        r["integration_pass"]=!detected&&clearPath&&localCostmapFresh&&mppiFresh&&mppiOptimal;
      }else{
        const QRegularExpression trialRe(QStringLiteral("^obstacle_trial_([1-3])$"));const auto trialMatch=trialRe.match(condition);
        const bool repeatTrial=trialMatch.hasMatch();
        const QRegularExpression re(QStringLiteral("^obstacle_([1-5])m$"));const auto mm=re.match(condition);
        const double ref=mm.hasMatch()?mm.captured(1).toDouble():std::numeric_limits<double>::quiet_NaN();
        if(repeatTrial){
          r["trial_index"]=trialMatch.captured(1).toInt();
          r["goal_distance_m"]=5.0;
          r["goal_reference_m"]=5.0;
          r["goal_actual_progress_m"]=QJsonValue();
          r["goal_signed_error_m"]=QJsonValue();
          r["goal_abs_error_m"]=QJsonValue();
          r["reference_type"]=QStringLiteral("GOAL_FORWARD_DISTANCE_M");
          r["reference_source"]=QStringLiteral("trial_protocol");
          r["actual_source"]=QStringLiteral("forward_progress_m_after_armed");
          r["reference_value"]=5.0;r["actual_value"]=QJsonValue();r["signed_error"]=QJsonValue();r["abs_error"]=QJsonValue();
          r["reference_distance_m"]=QJsonValue();
          r["distance_error_m"]=QJsonValue();
          r["expected_status"]=QStringLiteral("HUMAN_ON_5M_GOAL_ROUTE");
          r["status_match"]=detected;
          r["actual_action"]=detected?QStringLiteral("HUMAN_DETECTED_READY_FOR_NAV2"):QStringLiteral("HUMAN_REQUIRED");
          r["expected_action"]=QStringLiteral("NAV2_MPPI_AVOID_PASS_RECOVER_GOAL_5M_STOP");
          const bool mppiOptimal=mppi.value("optimal_count").toDouble(0)>0;
          r["integration_pass"]=detected&&metricValid&&metricSafety&&objectCount>0&&
            pathCount>0&&planningCount>0&&costmapMarked&&mppiFresh&&mppiOptimal;
        }else{
          const double obstacleDistanceErr=(std::isfinite(hd)&&std::isfinite(ref))?hd-ref:std::numeric_limits<double>::quiet_NaN();
          r["reference_distance_m"]=finiteOrNull(ref);
          r["distance_error_m"]=finiteOrNull(obstacleDistanceErr);
          r["abs_distance_error_m"]=finiteOrNull(std::isfinite(obstacleDistanceErr)?std::abs(obstacleDistanceErr):obstacleDistanceErr);
          r["reference_type"]=QStringLiteral("REFERENCE_DISTANCE_METERAN");
          r["reference_source"]=QStringLiteral("operator_meter_measurement");
          r["actual_source"]=QStringLiteral("stable_human_distance_m");
          r["reference_value"]=finiteOrNull(ref);r["actual_value"]=finiteOrNull(hd);
          r["signed_error"]=finiteOrNull(obstacleDistanceErr);
          r["abs_error"]=finiteOrNull(std::isfinite(obstacleDistanceErr)?std::abs(obstacleDistanceErr):obstacleDistanceErr);
          r["expected_status"]=QStringLiteral("HUMAN_AT_OPERATOR_REFERENCE_DISTANCE");
          r["status_match"]=detected;
        if(std::isfinite(ref)&&std::abs(ref-1.0)<1.0e-9){
          const bool nearFieldStop=snap.value("perception_emergency").toBool(false)||decisionUpper.contains("EMERGENCY");
          r["near_field_stop_active"]=nearFieldStop;
          r["actual_action"]=detected&&nearFieldStop?QStringLiteral("STOP"):QStringLiteral("STOP_GATE_NOT_ACTIVE");
          r["expected_action"]=QStringLiteral("STOP");
          r["integration_pass"]=detected&&nearFieldStop;
        }else{
          r["actual_action"]=detected?QStringLiteral("HUMAN_DETECTED_READY_FOR_NAV2"):QStringLiteral("HUMAN_REQUIRED");
          r["expected_action"]=QStringLiteral("NAV2_MPPI_AVOID_PASS_RECOVER_STOP");
          const bool mppiOptimal=mppi.value("optimal_count").toDouble(0)>0;
          r["integration_pass"]=detected&&metricValid&&metricSafety&&objectCount>0&&
            pathCount>0&&planningCount>0&&costmapMarked&&mppiFresh&&mppiOptimal;
        }
        }
      }
      r["dry_run_ready_for_armed"]=r.value("integration_pass").toBool(false);
      r["drive_target"]=snap.value("esc_drive_target");r["drive_actual"]=snap.value("esc_drive_actual");
      r["steering_target"]=snap.value("esc_steer_target");r["steering_actual"]=snap.value("esc_steer_actual");
      r["command_sent"]=false;
      r["result"]=r.value("integration_pass").toBool(false)?QStringLiteral("DRY_RUN_PASS"):QStringLiteral("DRY_RUN_FAIL");
    }
    return r;
  }

  static QByteArray renderJpegForDisplay(const QByteArray &jpeg,const QJsonObject &row,bool annotateHuman) {
    QImage img = QImage::fromData(jpeg, "JPG");
    if (img.isNull()) img = QImage::fromData(jpeg);
    if (img.isNull()) return jpeg;
    QImage mirrored = img.mirrored(true, false).convertToFormat(QImage::Format_RGB888);
    if(annotateHuman && row.value("detected").toInt(0)==1) {
      const double srcW=std::max(1.0,row.value("image_width").toDouble(mirrored.width()));
      const double srcH=std::max(1.0,row.value("image_height").toDouble(mirrored.height()));
      const double sx=static_cast<double>(mirrored.width())/srcW, sy=static_cast<double>(mirrored.height())/srcH;
      int x1=static_cast<int>(std::round(row.value("bbox_x").toDouble()*sx));
      int y1=static_cast<int>(std::round(row.value("bbox_y").toDouble()*sy));
      int x2=static_cast<int>(std::round((row.value("bbox_x").toDouble()+row.value("bbox_width").toDouble())*sx));
      int y2=static_cast<int>(std::round((row.value("bbox_y").toDouble()+row.value("bbox_height").toDouble())*sy));
      x1=std::clamp(x1,0,mirrored.width()-1);x2=std::clamp(x2,0,mirrored.width()-1);
      y1=std::clamp(y1,0,mirrored.height()-1);y2=std::clamp(y2,0,mirrored.height()-1);
      if(x2<x1) std::swap(x1,x2);
      if(y2<y1) std::swap(y1,y2);
      const int thick=std::max(2,mirrored.width()/320);
      auto paintPixel=[&](int x,int y){
        if(x<0||x>=mirrored.width()||y<0||y>=mirrored.height())return;
        uchar *px=mirrored.scanLine(y)+x*3;px[0]=72;px[1]=224;px[2]=164;
      };
      for(int t=0;t<thick;++t){
        for(int x=x1;x<=x2;++x){paintPixel(x,y1+t);paintPixel(x,y2-t);}
        for(int y=y1;y<=y2;++y){paintPixel(x1+t,y);paintPixel(x2-t,y);}
      }
    }
    QByteArray out;
    QBuffer buffer(&out);
    if (!buffer.open(QIODevice::WriteOnly) || !mirrored.save(&buffer, "JPEG", 92) || out.isEmpty()) return jpeg;
    return out;
  }

  static QByteArray csvCell(const QJsonValue &v) {
    QString s;if(v.isNull()||v.isUndefined())s=QString();else if(v.isBool())s=v.toBool()?QStringLiteral("true"):QStringLiteral("false");
    else if(v.isDouble())s=QString::number(v.toDouble(),'g',15);else s=v.toVariant().toString();
    s.replace(QStringLiteral("\""), QStringLiteral("\"\"")); return QByteArray("\"") + s.toUtf8() + QByteArray("\"");
  }  static bool writeCsv(const QString &path,const QStringList &headers,const QJsonArray &rows) {
    QSaveFile f(path);QDir().mkpath(QFileInfo(path).absolutePath());if(!f.open(QIODevice::WriteOnly))return false;
    QByteArray out;for(int i=0;i<headers.size();++i){if(i)out+=',';out+=csvCell(headers[i]);}out+='\n';
    for(const QJsonValue &v:rows){const QJsonObject r=v.toObject();for(int i=0;i<headers.size();++i){if(i)out+=',';out+=csvCell(r.value(headers[i]));}out+='\n';}
    if (f.write(out) != out.size()) return false;
    return f.commit();
  }
  static QJsonArray allRows(const QJsonObject &sub) {
    QJsonArray out;const QJsonObject cs=sub.value("conditions").toObject();
    for(auto it=cs.begin();it!=cs.end();++it)for(const QJsonValue &v:it.value().toObject().value("rows").toArray())out.append(v);
    return out;
  }
  bool writeCsvArtifacts(const QJsonObject &st,const QString &sub,const QString &condition) const {
    const QString id=st.value("session_id").toString();const QJsonObject s=st.value("subchapters").toObject().value(sub).toObject();
    const QJsonArray rows=allRows(s);QStringList h;
    if(sub=="4.1")h=QStringList{"capture_id","timestamp","reference_type","reference_source","actual_source","reference_value","actual_value","signed_error","abs_error","fps_reference_hz","fps_actual_hz","fps_error_hz","fps_abs_error_hz","camera_stream","camera_healthy","orientation_left_ok","orientation_right_ok","yolop_active","semantic_stream","semantic_status","lane_stream","fps","inference_ms","frame_age_ms","stream_status","frame_filename"};
    else if(sub=="4.2")h=QStringList{"capture_id","timestamp","reference_type","reference_source","actual_source","reference_value","actual_value","signed_error","abs_error","lighting_condition","illumination_label","illumination_mean_luma","illumination_stddev_luma","illumination_extreme_fraction","illumination_mean_gradient","camera_health_reason","reference_distance_m","estimated_distance_m","distance_error_m","abs_distance_error_m","detected_class","detected","confidence","bbox_x","bbox_y","bbox_width","bbox_height","bbox_width_fraction","bbox_height_fraction","stable_bbox_height_fraction","bottom_clipped","bbox_saturated","distance_quality","image_width","image_height","raw_forward_m","raw_left_m","bbox_calibrated_forward_m","legacy_affine_forward_m","calibrated_forward_m","stable_forward_m","nav2_forward_m","nav2_distance_source","metric_match_mode","semantic_updated_ms","semantic_inference_ms","evaluation_candidate_count","detector_source","detection_source","inference_ms","fps","frame_filename"};
    else if(sub=="4.3")h=QStringList{"capture_id","timestamp","reference_type","reference_source","actual_source","reference_value","actual_value","signed_error","abs_error","condition_id","expected_status","actual_status","status_match","status_error","lane_offset_reference_m","lane_offset_actual_m","centering_error_m","abs_centering_error_m","left_status","right_status","left_gap_m","right_gap_m","vehicle_offset_m","drivable_width_m","left_lane_confidence","right_lane_confidence","recommended_action","lane_updated_ms","fps","inference_ms","frame_filename"};
    else if(sub=="4.4")h=QStringList{"timestamp","elapsed_s","reference_type","reference_source","actual_source","reference_value","actual_value","signed_error","abs_error","fps_reference_hz","fps_actual_hz","fps_error_hz","fps_abs_error_hz","fps_performance_margin_hz","inference_time_reference_ms","inference_time_actual_ms","inference_time_error_ms","inference_time_abs_error_ms","inference_time_margin_ms","capture_fps_reference_hz","capture_fps_actual_hz","capture_fps_error_hz","fps","inference_ms","processing_ms","frame_age_ms","subscriber_latency_ms","stream_gap","dropped_frame","human_count","lane_valid","drivable_valid","stream_status"};
    else h=QStringList{"capture_id","timestamp","mode","condition","trial_index","reference_type","reference_source","actual_source","reference_value","actual_value","signed_error","abs_error","goal_distance_m","goal_reference_m","goal_actual_progress_m","goal_signed_error_m","goal_abs_error_m","goal_endpoint_error_m","final_map_x","final_map_y","lane_offset_reference_m","lane_offset_actual_m","lane_offset_error_m","lane_offset_abs_error_m","heading_reference_rad","heading_actual_rad","heading_signed_error_rad","heading_abs_error_rad","heading_signed_error_deg","heading_abs_error_deg","steering_reference_rad","steering_actual_feedback_rad","steering_signed_error_rad","steering_abs_error_rad","steering_signed_error_deg","steering_abs_error_deg","left_status","right_status","final_lane_status","expected_status","actual_status","status_match","status_error","initial_condition_match","initial_lane_status","initial_lane_state","initial_lane_recommendation","initial_left_latched","initial_right_latched","lane_left_latched","lane_right_latched","lane_evidence_source","lane_corridor_evidence_mode","lane_left_boundary_source","lane_right_boundary_source","lane_left_gap_px","lane_right_gap_px","lane_left_active_samples","lane_right_active_samples","lane_left_drivable_edge_samples","lane_right_drivable_edge_samples","lane_classical_fallback_active","lane_classical_left_samples","lane_classical_right_samples","human_detected","human_distance","human_confidence","reference_distance_m","distance_error_m","raw_distance_m","bbox_calibrated_distance_m","legacy_affine_distance_m","calibrated_distance_m","stable_distance_m","nav2_distance_m","nav2_distance_source","distance_filter_state","distance_history_count","stable_bbox_height_fraction","obstacle_memory_trial_active","obstacle_memory_latched","obstacle_memory_released","obstacle_memory_release_count","obstacle_memory_odom_x","obstacle_memory_odom_y","obstacle_memory_base_forward_m","lateral_position_m","metric_position_valid","near_field_stop_active","metric_obstacle_safety","object_points_count","path_relevant_points_count","planning_relevant_points_count","local_costmap_fresh","costmap_nonzero_cells","costmap_lethal_cells","costmap_max_cost","costmap_source","costmap_marked","mppi_trajectory_count","mppi_optimal_count","mppi_active","avoidance_runtime","integration_pass","dry_run_ready_for_armed","perception_decision","expected_action","actual_action","source_updated_ms","drive_target","drive_actual","steering_target","steering_actual","expected_steering_direction","web_trial_authority_seen","web_trial_authority_at_ms","cmd_mux_source_last","steering_target_runtime_rad","steering_actual_runtime_rad","peak_steering_actual_rad","steering_response_seen","steering_direction_match","steering_response_ms","steering_neutral_maintained","command_sent","response_time_ms","recovery_ms","travel_m","forward_progress_m","lateral_displacement_m","target_distance_m","clear_target_m","initial_heading_rad","current_heading_rad","heading_error_rad","heading_error_deg","heading_restored","heading_restored_once","lane_phase","goal_x_map","goal_y_map","goal_state","object_cloud_seen","planning_cloud_seen","costmap_marking_seen","mppi_seen","avoidance_seen","recovery_seen","safety_decision_last","avoid_direction","human_start_confidence","human_start_display_x_fraction","command_v_mps","command_w_rad_s","heartbeat_count","heartbeat_last_ms","heartbeat_age_ms","heartbeat_transport","heartbeat_sequence","trial_person_only_planning_filter","obstacle_motion_released","obstacle_motion_released_at_ms","obstacle_memory_latched_gate_at_ms","planner_gate_pass","planner_gate_started_ms","planner_gate_passed_ms","planner_gate_elapsed_ms","planner_gate_reason","planner_gate_planning_points","planner_gate_local_costmap_aligned","planner_gate_global_costmap_post_latch","planner_gate_path_post_costmap","planner_gate_mppi_post_path","planner_gate_path_length_m","planner_gate_path_heading_variation_rad","planner_hold_active","planner_hold_count","planner_hold_recovery_count","planner_hold_reason","planner_path_age_ms","planner_path_count","planner_path_length_m","planner_path_heading_variation_rad","planner_path_quality_ok","final_vehicle_state","trial_reason","result","frame_filename","map_filename","combined_filename","visual_capture_at_ms"};
    const QString root=subchapterPath(id,sub);
    bool ok=writeCsv(QDir(root).filePath(sub=="4.1"?QStringLiteral("validation.csv"):QStringLiteral("raw_data.csv")),h,rows);    if(sub=="4.2"||sub=="4.3"||sub=="4.4"||sub=="4.5"){
      const QJsonObject c=s.value("conditions").toObject().value(condition).toObject();
      if(!c.isEmpty())ok=writeCsv(QDir(conditionPath(id,sub,condition)).filePath(sub=="4.4"?QStringLiteral("raw.csv"):QStringLiteral("data.csv")),h,c.value("rows").toArray())&&ok;
    }
    return ok;
  }
  void removeSubchapterData(const QString &id,const QString &sub) const {
    const QString p=subchapterPath(id,sub);if(QDir(p).exists())QDir(p).removeRecursively();
  }
  void logEvent(const QString &id,const QString &event,const QString &sub,const QString &condition,const QString &message) const {
    const QString p=QDir(sessionPath(id)).filePath(QStringLiteral("events.csv"));QFile f(p);
    const bool fresh=!QFileInfo(p).isFile();if(!f.open(QIODevice::WriteOnly|QIODevice::Append))return;
    if(fresh)f.write("timestamp,session_id,subchapter,condition,event,message\n");
    auto q=[](QString s){s.replace(QStringLiteral("\""), QStringLiteral("\"\"")); return QByteArray("\"") + s.toUtf8() + QByteArray("\"");};
    f.write(q(isoNow())+","+q(id)+","+q(sub)+","+q(condition)+","+q(event)+","+q(message)+"\n");
  }
};