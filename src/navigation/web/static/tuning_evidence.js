'use strict';
function renderExperimentTable(){
  const x=selectedExperiment,box=$('experimentTables');if(!box)return;const all=x?.tableColumns||[];
  if(!all.length){box.innerHTML='<div class="empty-state">Subbab ini tidak mendefinisikan tabel.</div>';return}
  const timed=tableHasTimeAxis(),report=window.reportDualEnabled?.()===true,lifecycle=typeof reportEvidenceLifecycle==='function'?reportEvidenceLifecycle():(recordStartedMs?'RECORDING':'PREVIEW');
  box.innerHTML=all.map((cols,ti)=>{
    const title=x.tableNames?.[ti]||`Tabel ${ti+1}`,reportSeconds=timed&&currentExp==='navigation'&&x?.reportMode===true,head=timed?[reportSeconds?'Detik [s]':'t [s]',...cols]:cols,rows=tableRunRows.get(ti)||[];
    let body='';
    if(rows.length)body=rows.slice(-300).map(r=>'<tr>'+((timed?[String(reportSeconds?(Number(r.sec)+1):r.sec),...r.values]:r.values).map(v=>`<td>${escapeHtml(v)}</td>`).join(''))+'</tr>').join('');
    else if(report)body=`<tr class="report-wait-row"><td colspan="${Math.max(1,head.length)}">${lifecycle==='RECORDING'?'RECORDING dimulai • menunggu row pertama':'PREVIEW LIVE • tabel evidence belum merekam • tekan START untuk reset t=0 dan mulai capture'}</td></tr>`;
    else{const vals=cols.map(c=>inferColumnValue(c));body='<tr class="live-preview-row">'+((timed?[reportSeconds?'1':'0',...vals]:vals).map(v=>`<td>${escapeHtml(v)}</td>`).join(''))+'</tr>'}
    const state=rows.length?(lifecycle==='RECORDING'?'RECORDING':lifecycle==='FROZEN'?'FROZEN':'RECORDED'):(report?lifecycle:'LIVE PREVIEW'),cross=window.reportTableCrossButtons?.(ti)||'';
    return `<section class="direct-table-card"><div class="direct-table-title"><span>Tabel ${ti+1}</span><b>${escapeHtml(title)}</b><small>${escapeHtml(state)}${rows.length?' • '+rows.length+' row':''}</small>${cross}</div><div class="table-scroll"><table class="data-table experiment-data-table"><thead><tr>${head.map(c=>`<th>${escapeHtml(c)}</th>`).join('')}</tr></thead><tbody>${body}</tbody></table></div>${window.reportLinkedGraphHtml?.(ti,title)||''}</section>`
  }).join('');
  box.onclick=e=>{if(window.reportHandleTableAction?.(e.target))return};
  window.reportRefreshLinkedTableGraphs?.();
  setText('tableLiveHint',report?`${all.length} tabel rekap • ${lifecycle==='PREVIEW'?'PREVIEW tidak disimpan':lifecycle==='RECORDING'?'RECORDING 1 Hz • grafik sumber memakai buffer yang sama':'FROZEN • data siap Save CSV / Excel / PNG'}`:`${all.length} tabel langsung ditampilkan • ${recordStartedMs?'RECORDING 1 Hz':'preview live'}`)
}
function templateTableCsv(tableIndex=selectedTableIndex){const cols=selectedExperiment?.tableColumns?.[tableIndex]||[],rows=tableRunRows.get(tableIndex)||[],timed=tableHasTimeAxis(),reportSeconds=timed&&currentExp==='navigation'&&selectedExperiment?.reportMode===true,yamlKeys=[...new Set(rows.flatMap(r=>(r.yaml||[]).map(x=>x.key)))],yamlLabels=Object.fromEntries(rows.flatMap(r=>(r.yaml||[]).map(x=>[x.key,x.label]))),head=[...(timed?[reportSeconds?'Detik [s]':'t [s]',...cols]:cols),...yamlKeys.map(k=>`YAML ${k} | ${yamlLabels[k]||''}`)];if(!head.length||!rows.length)return null;const esc=v=>'"'+String(v??'').replaceAll('"','""')+'"';return [head.map(esc).join(','),...rows.map(r=>{const ym=Object.fromEntries((r.yaml||[]).map(x=>[x.key,x.value]));return [...(timed?[reportSeconds?(Number(r.sec)+1):r.sec,...r.values]:r.values),...yamlKeys.map(k=>ym[k]??'--')].map(esc).join(',')})].join('\n')+'\n'}
function saveCurrentTemplateCsv(auto=false){const csv=templateTableCsv();if(!csv){if(!auto)toast('Belum ada row hasil recording untuk tabel ini',true);return false}const safe=reportRunId(),name=`${safe}_T${selectedTableIndex+1}_${stamp()}.csv`;downloadBlob(name,'text/csv;charset=utf-8',csv);if(!auto)toast('Template table CSV disimpan');return true}
async function saveTemplateTableServer(tableIndex){const csv=templateTableCsv(tableIndex);if(!csv||!selectedExperiment)return null;const runId=reportRunId();const r=await writeRequest('/api/experiment/table/save',{subsystem:currentExp,id:runId,label:runId,source_experiment_id:selectedExperiment.id,table_index:tableIndex,csv}),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);return j}
async function saveAllTemplateTablesServer(){if(!selectedExperiment)return[];const count=(selectedExperiment.tableColumns||[]).length,results=[];for(let ti=0;ti<count;ti++){if(!(tableRunRows.get(ti)||[]).length)continue;try{const j=await saveTemplateTableServer(ti);if(j)results.push(j)}catch(e){toast(`Auto-save Tabel ${ti+1} gagal: ${e.message}`,true)}}return results}
function ensureContextSummary(){let e=$('contextSummary');if(e)return e;e=document.createElement('div');e.id='contextSummary';e.className='context-summary';$('contextVisual').appendChild(e);return e}
function navigationContextKind(){const id=String(selectedExperiment?.id||'');if(/^N(?:2|3)\./.test(id))return'esc';if(/^N(?:1|4|13|17)\./.test(id))return'summary';return'map'}
function renderNavigationSummary(){const e=ensureContextSummary(),id=String(selectedExperiment?.id||'');let rows=[];
  if(/^N1\./.test(id))rows=[['Wheelbase',configValue('vehicle','vehicle.ros__parameters.wheelbase_m'),'m'],['Track width',configValue('vehicle','vehicle.ros__parameters.track_width_m'),'m'],['Vehicle width',configValue('vehicle','vehicle.ros__parameters.total_width_m'),'m'],['Vehicle length',configValue('vehicle','vehicle.ros__parameters.vehicle_length_m'),'m'],['Wheel radius',configValue('vehicle','vehicle.ros__parameters.wheel_radius_m'),'m'],['Max forward',configValue('vehicle','vehicle.ros__parameters.max_forward_speed_mps'),'m/s']];
  else if(/^N4\./.test(id)){const im=obj('imu');rows=[['Roll',deg(im.roll_rad),'deg'],['Pitch',deg(im.pitch_rad),'deg'],['Yaw',deg(im.yaw_rad),'deg'],['Gyro X',im.gx,'rad/s'],['Gyro Y',im.gy,'rad/s'],['Gyro Z',im.gz,'rad/s']]}
  else if(/^N13\./.test(id))rows=[['Nav2 vx',obj('cmd_nav').linear_x,'m/s'],['Integrated vx',obj('cmd_autonomy_integrated').linear_x,'m/s'],['Pre-collision vx',obj('cmd_pre_collision').linear_x,'m/s'],['Final vx',obj('cmd_final').linear_x,'m/s'],['Actuator vx',obj('cmd_actuator').linear_x,'m/s'],['Actual vx',raw('esc_drive_actual'),'m/s']];
  else rows=[['Motion ready',bool(raw('system.motion_ready'))?'READY':'LOCKED',''],['Nav2',bool(raw('system.nav2_ready'))?'READY':'WAIT',''],['GNSS',(typeof gnssUiConnected==='function'?gnssUiConnected():bool(raw('connected.gnss')))?'ONLINE':'OFFLINE',''],['IMU',bool(raw('connected.imu'))?'ONLINE':'OFFLINE',''],['ESC',bool(raw('connected.esc_feedback'))?'ACK':'WAIT',''],['Goal',getPath(obj('goal_state'),'state')||'IDLE','']];
  e.innerHTML=`<div class="context-summary-head"><span>${escapeHtml(displayExperimentGroup(selectedExperiment))}</span><b>${escapeHtml(displayExperimentId(selectedExperiment))} live evidence</b></div><div class="context-summary-grid">${rows.map(([k,v,u])=>`<div><span>${escapeHtml(k)}</span><b>${typeof v==='number'?fmt(v,3):escapeHtml(v??'--')}</b><small>${escapeHtml(u||'')}</small></div>`).join('')}</div>`
}
function renderContextVisual(){if(!selectedExperiment)return;const map=$('contextMap'),cam=$('contextCamera'),esc=$('contextEsc'),summary=ensureContextSummary();map.style.display='none';cam.style.display='none';esc.style.display='none';summary.style.display='none';if(currentExp==='navigation'){const kind=navigationContextKind();if(kind==='map'){map.style.display='block';setText('contextTitle','Map • Path • Robot');drawExperimentMap()}else if(kind==='esc'){esc.style.display='block';setText('contextTitle',/^N2\./.test(String(selectedExperiment.id))?'Drive eRPM • GNSS • ESC • IMU Speed':'Steering • Yaw • Multi-sensor');drawExperimentEsc()}else{summary.style.display='block';setText('contextTitle',/^N4\./.test(String(selectedExperiment.id))?'IMU Orientation & Motion':/^N13\./.test(String(selectedExperiment.id))?'Command Chain':'Engineering Evidence');renderNavigationSummary()}}else if(currentExp==='perception'){cam.style.display='block';setText('contextTitle','Camera • YOLOPv2');updateLabCamera()}else{esc.style.display='block';setText('contextTitle','FOC • Steering • Drive');drawExperimentEsc()}renderContextMetrics()}
function drawExperimentMap(){const c=$('expMapCanvas'),m=obj('map_meta');if(!c||!mapImage||!m.width){if($('expMapEmpty'))$('expMapEmpty').style.display='grid';return}const {ctx,w,h,dpr}=canvasFit(c,310),scale=Math.min(w/mapImage.width,h/mapImage.height),dw=mapImage.width*scale,dh=mapImage.height*scale,ox=(w-dw)/2,oy=(h-dh)/2;ctx.clearRect(0,0,w,h);ctx.fillStyle='#050b0d';ctx.fillRect(0,0,w,h);ctx.drawImage(mapImage,ox,oy,dw,dh);const tr=(x,y)=>[ox+((x-m.origin_x)/m.resolution)*scale,oy+(m.height-(y-m.origin_y)/m.resolution)*scale];function path(key,color,width){const data=obj(key),pts=data.points||[],frame=data.frame_id||'map';if(pts.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=width*dpr;ctx.beginPath();let started=false;for(const p of pts){const mp=framePointToMap(frame,+p[0],+p[1]);if(!mp)continue;const [x,y]=tr(mp.x,mp.y);started?ctx.lineTo(x,y):(ctx.moveTo(x,y),started=true)}if(started)ctx.stroke()}path('nav_path','#4aa8ff',2);path('local_path','#54d6e8',2);for(const t of (obj('mppi_trajectories').trajectories||[]).filter(x=>x.optimal)){ctx.strokeStyle='#ffc86b';ctx.lineWidth=3*dpr;ctx.beginPath();let started=false;for(const p of (t.points||[])){const mp=framePointToMap(t.frame_id||'odom',+p[0],+p[1]);if(!mp)continue;const [x,y]=tr(mp.x,mp.y);started?ctx.lineTo(x,y):(ctx.moveTo(x,y),started=true)}if(started)ctx.stroke()}const pose=currentPose();if(pose){const [x,y]=tr(pose.x,pose.y);ctx.fillStyle='#48e0a4';ctx.beginPath();ctx.arc(x,y,7*dpr,0,Math.PI*2);ctx.fill()}const goal=obj('goal_pose');if(Number.isFinite(+goal.x)){const [x,y]=tr(goal.x,goal.y),goalReport=String(selectedExperiment?.id||'')==='R4.4.5',tol=+configValue('nav2','controller_server.ros__parameters.goal_checker.xy_goal_tolerance');if(goalReport&&Number.isFinite(tol)&&tol>0){const ppm=scale/m.resolution,r=tol*ppm;ctx.save();ctx.strokeStyle='rgba(255,123,138,.92)';ctx.fillStyle='rgba(255,123,138,.10)';ctx.lineWidth=2*dpr;ctx.setLineDash([6*dpr,4*dpr]);ctx.beginPath();ctx.arc(x,y,r,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#ffb1ba';ctx.font=`${11*dpr}px ui-monospace`;ctx.fillText(`Goal tolerance R=${tol.toFixed(2)} m`,x+10*dpr,y-Math.max(12*dpr,r+6*dpr));ctx.restore()}ctx.strokeStyle='#ff7b8a';ctx.lineWidth=2*dpr;ctx.beginPath();ctx.arc(x,y,8*dpr,0,Math.PI*2);ctx.stroke()}$('expMapEmpty').style.display='none'}
function updateLabCamera(){/* CameraFrameStore updates the active experiment camera. */}
function drawExperimentEsc(){const c=$('expEscCanvas');if(!c)return;const {ctx,w,h,dpr}=canvasFit(c,310),cx=w/2;ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.strokeStyle='rgba(130,160,165,.25)';ctx.lineWidth=2*dpr;ctx.beginPath();ctx.moveTo(60*dpr,110*dpr);ctx.lineTo(w-60*dpr,110*dpr);ctx.stroke();const stm=currentExp==='steering',target=deg(raw(stm?'esc_steer_protocol_cmd':'esc_steer_target'))||0,actual=deg(raw(stm?'esc_steer_feedback_raw':'esc_steer_actual'))||0,limit=stm?90:45,mapAngle=a=>cx+(Math.max(-limit,Math.min(limit,a))/limit)*(w*.36);[[stm?'STM TARGET':'TARGET',target,'#ffc86b',85],[stm?'STM FEEDBACK':'ACTUAL',actual,'#48e0a4',135]].forEach(([label,v,color,y])=>{ctx.fillStyle=color;ctx.fillRect(mapAngle(v)-3*dpr,y*dpr,6*dpr,48*dpr);ctx.font=`${11*dpr}px ui-monospace`;ctx.fillText(`${label} ${v.toFixed(2)}°`,18*dpr,(y+24)*dpr)});const foc=obj('foc_telemetry'),maint=bool(raw('vesc_maintenance_active')),left=maint?obj('vesc_left_values'):(foc.left||obj('vesc_left_values')),right=maint?obj('vesc_right_values'):(foc.right||obj('vesc_right_values')),lines=[`Drive target ${fmt(raw('esc_drive_target'),3)} m/s`,`Drive actual ${fmt(raw('esc_drive_actual'),3)} m/s`,`LEFT Iq ${fmt(left.iq_a,3)} A / Id ${fmt(left.id_a,3)} A`,`RIGHT Iq ${fmt(right.iq_a,3)} A / Id ${fmt(right.id_a,3)} A`,`Vbus L/R ${fmt(left.vbus_v,2)} / ${fmt(right.vbus_v,2)} V`];ctx.fillStyle='rgba(210,230,230,.82)';ctx.font=`${12*dpr}px ui-monospace`;lines.forEach((t,i)=>ctx.fillText(t,18*dpr,(210+i*22)*dpr))}
function renderContextMetrics(){let metrics=[];if(currentExp==='navigation'&&String(selectedExperiment?.id||'')==='R4.4.5'){const g=goalCheckerMetrics(),nav=String(obj('goal_state').state||'--').toUpperCase();metrics=[['Distance',Number.isFinite(g.distance)?fmt(g.distance,3)+' m':'--'],['XY radius',g.inside===1?'PASS':g.inside===0?'WAIT':'--'],['Speed evidence',Number.isFinite(g.speed)?fmt(g.speed,3)+' m/s':'--'],['Yaw-rate evidence',Number.isFinite(g.yawRate)?fmt(g.yawRate,3)+' rad/s':'--'],['Position pass',g.pass===1?'PASS':g.pass===0?'WAIT':'--'],['Nav2',nav]]}else if(currentExp==='navigation')metrics=[['Pose X',fmt(currentPose()?.x,3)+' m'],['Pose Y',fmt(currentPose()?.y,3)+' m'],['CTE',fmt(derivedCte(),3)+' m'],['Endpoint',fmt(derivedEndpoint(),3)+' m'],['Path',fmt(obj('nav_path').length_m,2)+' m'],['Goal',obj('goal_state').state||'--']];else if(currentExp==='perception')metrics=[['Camera',bool(raw('connected.camera'))?'ONLINE':'OFF'],['Health',bool(raw('camera_healthy'))?'HEALTHY':'WAIT'],['FPS',fmt(resolveMetricPath('perception_performance.fps'),2)],['Objects',obj('object_points').count??'--'],['Lane',obj('lane_state').state||obj('lane_state').raw||'--'],['Near field',obj('near_field_state').state||obj('near_field_state').raw||'--']];else{const foc=obj('foc_telemetry'),maint=bool(raw('vesc_maintenance_active')),left=maint?obj('vesc_left_values'):(foc.left||obj('vesc_left_values')),right=maint?obj('vesc_right_values'):(foc.right||obj('vesc_right_values'));metrics=[['ESC',bool(raw('connected.esc_feedback'))?'ACK':'WAIT'],['Drive',fmt(raw('esc_drive_actual'),3)+' m/s'],['Steer',fmt(deg(raw('esc_steer_actual')),2)+'°'],['LEFT Iq',fmt(left.iq_a,2)+' A'],['RIGHT Iq',fmt(right.iq_a,2)+' A'],['Mux',obj('esc_mux').raw||'--']]}$('contextMetrics').innerHTML=metrics.map(([k,v])=>`<div><span>${escapeHtml(k)}</span><b>${escapeHtml(v)}</b></div>`).join('')}
function normalizedExperimentGraphs(x=selectedExperiment){return (x?.graphCaptions||[]).map((_,i)=>{const g={...(x?.graphs?.[i]||{})};if(!g.type)g.type='time_series';if(g.type==='time_series'&&!(g.series||[]).length)g.series=autoGraphLabels(x,i);return g})}
function graphSourcePaths(x=selectedExperiment){const src=[...Object.values(x?.liveSeries||{})];for(const g of normalizedExperimentGraphs(x)){if(g?.xSeries)src.push(g.xSeries);if(g?.ySeries)src.push(g.ySeries);if(g?.pathKey)src.push(g.pathKey);for(const pair of (g?.pairs||[])){src.push(x?.liveSeries?.[pair?.[0]]||pair?.[0]);src.push(x?.liveSeries?.[pair?.[1]]||pair?.[1])}}return [...new Set(src.filter(Boolean))]}
function renderSourceAudit(){if(!selectedExperiment)return;const paths=graphSourcePaths(),pills=paths.map(p=>{const v=resolveMetricPath(p),ok=v!==undefined&&v!==null;return `<span class="${ok?'ok':'wait'}"><i></i>${escapeHtml(p)}</span>`});$('sourceAvailability').innerHTML=pills.join('')||'<span class="wait"><i></i>No direct live-series binding; gunakan tabel/config evidence.</span>';const sid=String(selectedExperiment?.id||''),method=currentExp==='navigation'&&/^R4\.1\./.test(sid)?'RAW sensor / actuator telemetry (PRE-EKF) + parameter YAML':currentExp==='navigation'?'Nav2/localization telemetry + map/path + parameter YAML':currentExp==='perception'?'Camera/YOLOPv2 metrics + safety telemetry + parameter YAML':'ESC feedback + FOC telemetry + calibration/tuning YAML';setText('sourceMethod',method);setText('experimentSourceBadge',paths.some(p=>resolveMetricPath(p)!=null)?'LIVE SOURCE':'WAITING SOURCE')}
// Recorder contract remains template-only report compatible; report recap only changes the selected schema/source mapping.
function recordPathsForSelected(x=selectedExperiment){
  if(!x)return ['__meta_only__'];
  const out=new Set(),add=p=>{p=String(p||'').trim();if(p&&!p.startsWith('derived.'))out.add(p)};
  const live=Object.values(x.liveSeries||{});
  live.forEach(add);
  for(const g of normalizedExperimentGraphs(x)){
    if(g?.xSeries)add(g.xSeries);if(g?.ySeries)add(g.ySeries);if(g?.pathKey)add(g.pathKey);
    for(const pair of (g?.pairs||[])){add(x.liveSeries?.[pair?.[0]]||pair?.[0]);add(x.liveSeries?.[pair?.[1]]||pair?.[1])}
  }
  const derived=live.filter(p=>String(p).startsWith('derived.'));
  const need=(...paths)=>paths.forEach(add);
  for(const d of derived){
    if(d==='derived.firda_phys_actual_deg')need('vesc_left_values.position_deg');
    else if(d==='derived.firda_phys_target_deg')need('vesc_command_state.value');
    else if(d==='derived.firda_phys_error_deg')need('vesc_left_values.position_deg','vesc_command_state.value');
    else if(d==='derived.steering_error_rad')need('esc_steer_target','esc_steer_actual');
    else if(d==='derived.ackermann_yaw_error')need('esc_yaw_rate','esc_kinematic_yaw_rate');
    else if(d==='derived.velocity_error_mps')need('cmd_final','esc_drive_actual');
    else if(d==='derived.firda_runtime_target_deg')need('esc_steer_target');
    else if(d==='derived.firda_runtime_actual_deg')need('esc_steer_actual');
    else if(d==='derived.firda_runtime_error_deg'){need('esc_steer_target');need('esc_steer_actual');}
    else if(d.startsWith('derived.ekf_local_'))need('ekf_local','esc_odom','gnss_base_vel_fusion','imu');
    else if(d.startsWith('derived.ekf_global_'))need('ekf_global','gnss_map_odom','gnss_base_vel_fusion','gnss_cog_fusion','imu');
    else if(d.includes('gnss_'))need('gnss_quality','gnss_fix','gnss_base_vel_fusion','gnss_cog_fusion');
    else if(d.includes('age_imu')||d.includes('rate_imu')||d.includes('dt_imu'))need('imu');
    else if(d.includes('age_esc')||d.includes('rate_esc')||d.includes('dt_esc'))need('esc_odom');
    else if(d.includes('age_ekf_local')||d.includes('rate_ekf_local'))need('ekf_local');
    else if(d.includes('age_ekf_global')||d.includes('rate_ekf_global'))need('ekf_global');
    else if(d.startsWith('derived.goal_'))need('goal_pose','goal_state','ekf_global','ekf_local','esc_drive_actual','esc_yaw_rate');
    else if(d.includes('cte')||d.includes('endpoint')||d.includes('heading_error'))need('localization_pose','ekf_local','ekf_global','nav_path');
  }
  if(currentExp==='steering'){
    if(live.some(p=>String(p).startsWith('vesc_command_state.')))need('vesc_command_state.mode','vesc_command_state.motor','vesc_command_state.value');
    if(x.id==='4.2.1')need('connected.vesc_transport','vesc_maintenance_active','vesc_tool_status');
    if(x.id==='4.9')need('vesc_left_values','vesc_right_values','vesc_steering_state','esc_status','esc_mux');
  }
  // Never send an empty allow-list: the server interprets empty as "record all".
  return out.size?[...out]:['__meta_only__'];
}
function navigationGoalMasterSections(){
  return navigationReportList().filter(x=>/^R4\.[234]\.[1-5]$/.test(String(x?.id||'')));
}
function navigationGoalMasterOwnerSpec(){
  return navigationReportList().find(x=>String(x?.id||'')==='R4.4.5')||null;
}
function navigationGoalMasterRecordPaths(x=navigationGoalMasterOwnerSpec()){
  const out=new Set();
  if(x)for(const path of recordPathsForSelected(x))out.add(path);
  for(const spec of navigationGoalMasterSections())for(const path of recordPathsForSelected(spec))out.add(path);
  ['goal_pose','goal_state','localization_pose','localization_state','nav_path','ekf_local','ekf_global','gnss_fix','gnss_quality','gnss_map_odom','gnss_vel','gnss_base_vel_fusion','gnss_cog_fusion','imu','esc_odom','cmd_nav','cmd_final','global_costmap_meta'].forEach(p=>out.add(p));
  out.delete('__meta_only__');return out.size?[...out]:['__meta_only__'];
}
function navigationGoalMasterMetricPaths(){
  if(recordingDomain!=='navigation'||recordingLeafId!=='R4.4.5'||!recordStartedMs)return[];
  const out=new Set();for(const spec of navigationGoalMasterSections())for(const p of Object.values(spec.liveSeries||{}))out.add(p);
  return [...out];
}
function navigationCaptureProfile(x=selectedExperiment){
  const id=String(x?.id||'');
  // Sensor/static acquisition must remain usable before actuator commissioning.
  if(['N4.1','N5.1','R4.1.1','R4.1.2'].includes(id))return {required:['imu'].filter(k=>id==='N4.1'||id==='R4.1.2').concat(['gnss_fix','gnss_quality'].filter(k=>id==='N5.1'||id==='R4.1.1')),actuator:false,phase:false};
  // BAB IV 4.1.3 is evidence capture of existing ESC/encoder feedback, not a motion command.
  // Keep the ESC feedback freshness gate, but never require ESC 4.9 qualification just to record it.
  if(id==='R4.1.3')return {required:['esc_drive_raw','esc_steer_feedback_raw','gnss_quality'],actuator:true,phase:false};
  if(id==='R4.1.4')return {required:['rm3100_heading','imu','rm3100_mag'],actuator:false,phase:false};
  if(id==='N4.2')return {required:['imu'],actuator:false,phase:false};
  if(id==='N5.2')return {required:['gnss_fix','gnss_quality'],actuator:false,phase:false};
  if(/^N6\./.test(id))return {required:['gnss_fix','gnss_quality'],actuator:false,phase:false};
  // BAB IV 4.2/4.3 are localization evidence. They may consume ESC feedback, but START itself
  // does not command motion, so commissioning phase qualification must not block acquisition.
  if(/^R4\.[23]\./.test(id)){
    const roots=new Set(id.startsWith('R4.2.')?['ekf_local']:['ekf_global']);
    for(const path of recordPathsForSelected(x)){
      const text=String(path||'').trim();if(!text||text==='__meta_only__'||text.startsWith('derived.'))continue;
      roots.add(text.split('.')[0]);
    }
    const actuator=[...roots].some(r=>/^esc_|^cmd_|^foc_|^vesc_/.test(r));
    return {required:[...roots],actuator,phase:false};
  }
  const roots=new Set();
  for(const path of graphSourcePaths(x)){
    const text=String(path||'');if(!text||text.startsWith('derived.'))continue;
    roots.add(text.split('.')[0]);
  }
  const actuator=[...roots].some(r=>/^esc_|^cmd_|^foc_|^vesc_/.test(r))||!!automaticTrialId();
  return {required:[...roots],actuator,phase:actuator};
}
function navigationTaskRequiresMotion(x=selectedExperiment){
  if(currentExp!=='navigation'||!x)return false;const id=String(x.id||'');
  return ['N2.1','N3.1','N3.2'].includes(id)||/^N(?:12|13|14|15|16|17)\./.test(id);
}
function taskCommissioningGate(){
  if(currentExp==='navigation'&&!navigationTaskRequiresMotion())return {ok:true,reason:'Sensor/localization/planner evidence: ESC phase qualification tidak diperlukan'};
  return phaseGate();
}
function testRequiredRoots(x=selectedExperiment){
  if(!x)return [];
  if(currentExp==='navigation')return navigationCaptureProfile(x).required;
  const roots=new Set();
  for(const path of graphSourcePaths(x)){
    const text=String(path||'');if(!text||text.startsWith('derived.'))continue;
    roots.add(text.split('.')[0]);
  }
  if(currentExp==='steering'){const firdaStep=String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep),firdaDirect=/^4\.[2345]\./.test(firdaStep)||firdaStep==='4.1.5';roots.add(firdaDirect?'vesc_left_values':'foc_telemetry');}
  if(currentExp==='perception'){roots.add('camera_frame');roots.add('perception_performance')}
  return [...roots];
}
function testPreflightStatus(){
  if(!selectedExperiment)return {ok:false,items:[],reason:'Pilih subpengujian'};
  const items=[];const add=(label,ok,detail,critical=true)=>items.push({label,ok:ok===true,detail,critical});
  const replay=window.AdvReplay?.isActive?.()===true;add('Live control mode',!replay,replay?'REPLAY MODE aktif':'LIVE MODE',true);
  add('ROS bridge',obj('server').ros===true,obj('server').ros===true?'ROS context active':'ROS bridge belum ready',true);
  const phase=phaseGate(),navProfile=currentExp==='navigation'?navigationCaptureProfile():null,phaseRequired=currentExp!=='navigation'||navProfile.phase;
  add('Commissioning prerequisite',phaseRequired?phase.ok:true,phaseRequired?phase.reason:'Tidak diperlukan untuk akuisisi sensor/static',phaseRequired);
  if(currentExp==='steering'){
    const tool=obj('vesc_tool_status'),link=bool(raw('connected.esc_feedback'))||bool(raw('connected.vesc_transport'))||tool.transport_connected===true||tool.gateway_connected===true;
    add('ESC transport / feedback',link,link?'gateway/feedback tersedia':'ESC transport dan feedback belum tersedia',true);
  }else if(currentExp==='navigation'&&navProfile.actuator){
    add('ESC feedback',bool(raw('connected.esc_feedback')),bool(raw('connected.esc_feedback'))?'actuator feedback fresh':'ESC feedback belum fresh',true);
  }else if(currentExp==='perception'){
    add('Camera',bool(raw('connected.camera'))&&channelFresh('camera_frame',2.5),`frame age ${Number.isFinite(age('camera_frame'))?age('camera_frame').toFixed(2):'--'} s`,true);
  }
  for(const root of testRequiredRoots()){
    const reg=CHANNEL_REGISTRY[root],a=age(root),rate=channelRate(root),firdaStaticEsc=currentExp==='steering'&&/^4\.1\.[1-4]$/.test(String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep))&&root==='foc_telemetry',freshLimit=reg?Math.max(reg.staleMs/1000,firdaStaticEsc ? .5 : 0):5,fresh=Number.isFinite(a)&&a<freshLimit;
    const rateOk=!reg||!Number.isFinite(rate)||rate>=Math.max(1,reg.expectedHz*.45);
    const critical=!!reg;
    const label=reg?.label||root;
    const detail=`${Number.isFinite(rate)?rate.toFixed(1)+' Hz':'rate --'} • age ${Number.isFinite(a)?a.toFixed(2)+' s':'--'}${reg?' • target '+reg.expectedHz+' Hz':''}`;
    if(!items.some(i=>i.label===label))add(label,fresh&&rateOk,detail,critical);
  }
  const auto=automaticTrialPreflight();if(automaticTrialId())add('Automatic motion gate',auto.ok,auto.reason||'ready',true);
  const bad=items.filter(i=>i.critical&&!i.ok);return {ok:bad.length===0,items,reason:bad.map(x=>x.label).join(', ')};
}
function renderTestPreflight(){
  const box=$('testPreflightGrid'),chip=$('testPreflightState'),summary=$('testPreflightSummary');if(!box||!selectedExperiment)return;
  const p=testPreflightStatus();
  box.innerHTML=p.items.map(i=>`<div class="test-preflight-item ${i.ok?'pass':i.critical?'fail':'warn'}"><i>${i.ok?'✓':i.critical?'!':'○'}</i><div><b>${escapeHtml(i.label)}</b><span>${escapeHtml(i.detail)}</span></div><em>${i.ok?'PASS':i.critical?'BLOCK':'CHECK'}</em></div>`).join('');
  const autoGoal=currentExp==='navigation'&&String(selectedExperiment?.id||'')==='R4.4.5';
  if(chip){chip.textContent=autoGoal?(navigationGoalMasterActive()?'RECORDING':'AUTO'):p.ok?'READY':'BLOCKED';chip.className='status-chip '+((autoGoal||p.ok)?'':'bad')}
  if(summary)summary.textContent=autoGoal?'4.4.5 direkam otomatis oleh SEND GOAL bersama seluruh 4.2–4.4.':p.ok?'Semua source wajib dan safety gate siap untuk START.':`START dikunci: ${p.reason||'required source belum siap'}`;
  const b=$('recordToggleBtn');if(b){b.hidden=autoGoal;if(!autoGoal&&b.dataset.active!=='true'){b.disabled=!p.ok;b.title=p.ok?'Ready to record':summary?.textContent||'Preflight blocked'}}
}
const NAV_REPORT_AUTO_STOP_SEC=60;
let navReportAutoTimer=null,navReportAutoStopBusy=false;
const navGoalMasterRows=new Map();
let navGoalMasterState={armed:false,sawActive:false,stopping:false,lastState:'',result:''};
let navGoalMissionContext={missionId:'',primaryId:'',candidate:'baseline',variation:'',cycle:0,goal:null,start:null};
function navigationGoalMasterActive(){return recordingDomain==='navigation'&&recordingLeafId==='R4.4.5'&&!!recordStartedMs}
function resetNavigationGoalMaster(arm=false){
  navGoalMasterRows.clear();navGoalMasterState={armed:arm,sawActive:false,stopping:false,lastState:'',result:''};
}
function navigationGoalMissionPrimarySpec(){
  const id=String(selectedExperiment?.id||'');
  return currentExp==='navigation'&&/^R4\.[234]\.[1-5]$/.test(id)?selectedExperiment:null;
}
function navigationGoalMissionNextCycle(primaryId,candidate,variation){
  const key=`adv-nav-goal-cycle:${primaryId||'MISSION'}:${candidate||'baseline'}:${variation||'default'}`;
  const n=Math.max(0,parseInt(localStorage.getItem(key)||'0',10)||0)+1;localStorage.setItem(key,String(n));return n;
}
function navigationGoalMissionConfigSnapshot(sections){
  const out={},keep=selectedExperiment;
  try{
    for(const spec of sections){
      selectedExperiment=spec;
      for(const x of parameterSnapshotEntries()){
        const p=tuningFieldsFor(spec).find(f=>(f.label||f.key||f.yamlPath)===x.label);
        out[`${x.file}:${p?.yamlPath||x.key}`]=x.value;
      }
    }
  }finally{selectedExperiment=keep}
  return out;
}
function navigationGoalMissionGraphs(sections){
  const live={},graphs=[];
  for(const spec of sections){
    Object.assign(live,spec.liveSeries||{});
    normalizedExperimentGraphs(spec).forEach((g,i)=>graphs.push({...g,title:`${displayExperimentId(spec)} • ${spec.graphCaptions?.[i]||`Grafik ${i+1}`}`}));
  }
  return {live,graphs};
}
function navigationMasterCanvasDataUrl(canvas,maxWidth=1920){
  if(!canvas?.width||!canvas?.height)return'';
  if(canvas.width<=maxWidth)return canvas.toDataURL('image/png');
  const scale=maxWidth/canvas.width,out=document.createElement('canvas');
  out.width=maxWidth;out.height=Math.max(1,Math.round(canvas.height*scale));
  const ctx=out.getContext('2d');ctx.fillStyle='#ffffff';ctx.fillRect(0,0,out.width,out.height);
  ctx.imageSmoothingEnabled=true;ctx.imageSmoothingQuality='high';ctx.drawImage(canvas,0,0,out.width,out.height);
  return out.toDataURL('image/png');
}
function collectNavigationGoalMasterGraphPngPayload(){
  if(!navigationGoalMasterActive()||typeof buildExperimentGraphExportCanvas!=='function')return[];
  const out=[],keep=selectedExperiment;let globalIndex=1;
  try{
    for(const spec of navigationGoalMasterSections()){
      selectedExperiment=spec;
      for(let i=0;i<(spec.graphCaptions||[]).length;i++){
        const c=buildExperimentGraphExportCanvas(i),caption=spec.graphCaptions?.[i]||`Grafik ${i+1}`;
        if(c)out.push({index:globalIndex,title:`${displayExperimentId(spec)} • ${caption}`,section_id:spec.id,graph_index:i,graph_caption:caption,data_url:navigationMasterCanvasDataUrl(c,1280)});
        globalIndex++;
      }
    }
  }finally{selectedExperiment=keep}
  return out;
}
function captureNavigationGoalMasterCompanionRows(sec,force=false){
  if(!navigationGoalMasterActive())return;
  const keep=selectedExperiment;
  try{
    for(const spec of navigationGoalMasterSections()){
      selectedExperiment=spec;
      const yaml=parameterSnapshotEntries().map(x=>({key:`${x.file}:${tuningFieldsFor(spec).find(p=>(p.label||p.key||p.yamlPath)===x.label)?.yamlPath||x.key}`,label:x.label,value:x.value}));
      let byTable=navGoalMasterRows.get(spec.id);if(!byTable){byTable=new Map();navGoalMasterRows.set(spec.id,byTable)}
      (spec.tableColumns||[]).forEach((cols,ti)=>{
        if(!cols?.length)return;
        const a=byTable.get(ti)||[],lastSec=a.length?Number(a[a.length-1].sec):-1;
        if(a.length&&lastSec===sec){if(!force)return;a.pop()}
        // STRICT FIELD RECORDING: never synthesize/backfill missed seconds.
        // Each row represents a value actually sampled while this second was live.
        a.push({sec,values:cols.map(c=>inferColumnValue(c)),yaml,backfilled:false,captured_at_ms:Date.now()});
        while(a.length>7200)a.shift();
        byTable.set(ti,a)
      });
    }
  }finally{selectedExperiment=keep}
}
function captureNavigationGoalMasterFinalRows(){
  if(!navigationGoalMasterActive())return;const sec=Math.max(0,Math.floor((Date.now()-recordStartedMs)/1000));
  captureNavigationGoalMasterCompanionRows(sec,true);lastLabSecond=sec;
}
function navigationGoalMasterContinuityReport(){
  if(!navigationGoalMasterActive())return {ok:false,expected_seconds:0,issues:['master inactive']};
  const terminalSec=Math.max(0,Math.floor((Date.now()-recordStartedMs)/1000));
  const expectedSeconds=terminalSec+1,issues=[],sections=[];
  for(const spec of navigationGoalMasterSections()){
    for(let ti=0;ti<(spec.tableColumns||[]).length;ti++){
      const rows=navGoalMasterRows.get(spec.id)?.get(ti)||[];
      const secs=[...new Set(rows.map(r=>Number(r.sec)).filter(Number.isFinite))].sort((a,b)=>a-b);
      const missing=[],backfilled=[...new Set(rows.filter(r=>r.backfilled===true).map(r=>Number(r.sec)+1).filter(Number.isFinite))].sort((a,b)=>a-b);
      for(let s=0;s<=terminalSec;s++)if(!secs.includes(s))missing.push(s+1);
      const row={section_id:spec.id,table_index:ti,rows:secs.length,expected_seconds:expectedSeconds,first_second:secs.length?secs[0]+1:null,last_second:secs.length?secs[secs.length-1]+1:null,missing_seconds:missing,backfilled_seconds:backfilled};
      sections.push(row);
      if(missing.length)issues.push(`${displayExperimentId(spec)} T${ti+1}: missing ${missing.slice(0,12).join(',')}${missing.length>12?'…':''}`);
      if(backfilled.length)issues.push(`${displayExperimentId(spec)} T${ti+1}: synthetic/backfill forbidden at ${backfilled.slice(0,12).join(',')}${backfilled.length>12?'…':''}`);
    }
  }
  return {ok:issues.length===0,expected_seconds:expectedSeconds,terminal_second:terminalSec+1,issues,sections};
}
function navigationGoalMasterTableCsv(spec,ti){
  const rows=navGoalMasterRows.get(spec.id)?.get(ti)||[],cols=spec.tableColumns?.[ti]||[];if(!rows.length||!cols.length)return null;
  const head=['Detik [s]',...cols],esc=v=>'"'+String(v??'').replaceAll('"','""')+'"';
  return [head.map(esc).join(','),...rows.map(r=>[Number(r.sec)+1,...r.values].map(esc).join(','))].join('\n')+'\n';
}
async function saveNavigationGoalMasterCompanionTablesServer(){
  if(!navigationGoalMasterActive())return[];const results=[];
  for(const spec of navigationGoalMasterSections())for(let ti=0;ti<(spec.tableColumns||[]).length;ti++){
    const csv=navigationGoalMasterTableCsv(spec,ti);if(!csv)continue;const runId=reportRunId(spec,'navigation');
    try{
      const r=await writeRequest('/api/experiment/table/save',{subsystem:'navigation',id:runId,label:runId,source_experiment_id:spec.id,table_index:ti,csv}),j=await r.json();
      if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
      results.push({...j,section_id:spec.id,table_index:ti,table_name:spec.tableNames?.[ti]||`Tabel ${ti+1}`,columns:['Detik [s]',...(spec.tableColumns?.[ti]||[])]})
    }catch(e){toast(`Master ${displayExperimentId(spec)} Tabel ${ti+1} gagal: ${e.message}`,true)}
  }
  return results;
}
async function startNavigationGoalMissionRecording(goal){
  if(recordStartedMs){toast('SEND GOAL diblok: recorder lain masih aktif. STOP + SAVE dulu.',true);return false}
  const gx=+goal?.x,gy=+goal?.y,gyaw=+goal?.yaw_deg;if(![gx,gy,gyaw].every(Number.isFinite)){toast('Goal X/Y/Yaw tidak valid.',true);return false}
  if(typeof window.ensureNavigationTuningRuntimeReady==='function'){
    setText('navGoalBlockReason','VERIFYING TUNING • YAML ↔ ROS RUNTIME');
    const gate=await window.ensureNavigationTuningRuntimeReady();
    if(!gate?.ok){const why=gate?.reason||'tuning belum ACTIVE MATCH';setText('navGoalBlockReason',`SEND GOAL BLOCKED • ${why}`);toast(`SEND GOAL diblok: ${why}`,true);return false}
  }
  const owner=navigationGoalMasterOwnerSpec(),sections=navigationGoalMasterSections();if(!owner||!sections.length){toast('Template BAB IV 4.2–4.4 belum tersedia.',true);return false}
  const primary=navigationGoalMissionPrimarySpec(),profileCandidate=(typeof navigationGlobalProfileSelected==='function'?navigationGlobalProfileSelected():''),candidate=profileCandidate||$('runCandidate')?.value||'baseline',variation=$('runVariation')?.value||`PROFILE-${String(candidate).toUpperCase()}`,cycle=navigationGoalMissionNextCycle(primary?.id||'MISSION',candidate,variation);if($('runCandidate')&&$('runCandidate').value!==candidate)$('runCandidate').value=candidate;
  const ep=obj('ekf_global'),lp=obj('localization_pose'),sx=Number.isFinite(+ep.x)?+ep.x:+lp.x,sy=Number.isFinite(+ep.y)?+ep.y:+lp.y,syaw=Number.isFinite(+ep.yaw)?+ep.yaw:+lp.yaw;
  const missionId=`GOAL_${new Date().toISOString().replace(/[-:.TZ]/g,'').slice(0,17)}_C${String(cycle).padStart(2,'0')}`,cfg=navigationGoalMissionConfigSnapshot(sections),bundle=navigationGoalMissionGraphs(sections);
  Object.assign(cfg,{
    'mission:mission_id':missionId,
    'mission:primary_experiment_id':primary?.id||'MISSION',
    'mission:cycle':cycle,
    'mission:start_x_m':sx,'mission:start_y_m':sy,'mission:start_yaw_rad':syaw,
    'mission:goal_x_m':gx,'mission:goal_y_m':gy,'mission:goal_yaw_deg':gyaw,
    'mission:goal_requested_yaw_deg':Number.isFinite(+goal?.requested_yaw_deg)?+goal.requested_yaw_deg:gyaw,
    'mission:goal_position_priority':goal?.position_priority===true,
    'nav_profile:smac_motion_model':configValue('nav2','planner_server.ros__parameters.GridBased.motion_model_for_search'),
    'nav_profile:minimum_turning_radius_m':configValue('nav2','planner_server.ros__parameters.GridBased.minimum_turning_radius'),
    'nav_profile:cost_penalty':configValue('nav2','planner_server.ros__parameters.GridBased.cost_penalty'),
    'nav_profile:global_inflation_radius_m':configValue('nav2','global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius'),
    'nav_profile:global_cost_scaling_factor':configValue('nav2','global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor'),
    'nav_profile:local_inflation_radius_m':configValue('nav2','local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius'),
    'nav_profile:local_cost_scaling_factor':configValue('nav2','local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor'),
    'nav_profile:goal_angle_critic_enabled':configValue('nav2','controller_server.ros__parameters.FollowPath.GoalAngleCritic.enabled')
  });
  const trialInputs={capture_mode:'SEND_GOAL_MASTER_4.2_4.3_4.4',master_sections:sections.map(x=>x.id),mission_id:missionId,primary_experiment_id:primary?.id||'MISSION',primary_experiment_label:primary?displayExperimentId(primary):'MISSION',cycle,start_x_m:sx,start_y_m:sy,start_yaw_rad:syaw,goal_x_m:gx,goal_y_m:gy,goal_yaw_deg:gyaw,goal_requested_yaw_deg:Number.isFinite(+goal?.requested_yaw_deg)?+goal.requested_yaw_deg:gyaw,goal_position_priority:goal?.position_priority===true,goal_approach_distance_m:Number.isFinite(+goal?.approach_distance_m)?+goal.approach_distance_m:null};
  const uiRate=+$('runSampleRate')?.value,sampleRate=Number.isFinite(uiRate)?Math.max(20,uiRate):20,payload={subsystem:'navigation',id:'BAB4_MISSION_4.2_4.4',source_experiment_id:'R4.4.5',label:'BAB4_MISSION_4.2_4.4',section_label:'AUTO SEND GOAL • 4.2–4.4',candidate,variation,condition:'SEND_GOAL_AUTO',sample_rate_hz:sampleRate,tuning_config:cfg,trial_inputs:trialInputs,live_series:bundle.live,graphs:bundle.graphs,record_paths:navigationGoalMasterRecordPaths(owner)};
  try{
    const r=await writeRequest('/api/experiment/record/start',payload),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    const now=Date.now();recordStartedMs=now;recordingLeafId='R4.4.5';recordingDomain='navigation';recordingRunToken=new Date(now).toISOString().replace(/[:.]/g,'-');chartEpochMs=now;lastLabSecond=-1;lastLabTelemetryTick=-1;metricHistory.clear();scatterHistory.clear();scatterOrigins.clear();tableRunRows.clear();reportPathSnapshots.clear();reportEvidenceFrozen=false;reportEvidenceFrozenLeaf='';navGoalMissionContext={missionId,primaryId:primary?.id||'MISSION',candidate,variation,cycle,goal:{x:gx,y:gy,yaw_deg:gyaw,requested_yaw_deg:Number.isFinite(+goal?.requested_yaw_deg)?+goal.requested_yaw_deg:gyaw,position_priority:goal?.position_priority===true},start:{x:sx,y:sy,yaw_rad:syaw}};resetNavigationGoalMaster(true);setRecordingUi(true,j.recording);captureNavigationGoalMasterCompanionRows(0,true);captureLabTelemetry();
    setText('recordMessage',`${missionId} • AUTO RECORD 4.2–4.4 • Cycle ${cycle} • menunggu Nav2 SENDING/ACTIVE`);setText('navGoalBlockReason',`MISSION RECORDING • ${missionId} • 4.2–4.4`);return true;
  }catch(e){toast('Mission recorder gagal start: '+e.message,true);setText('navGoalBlockReason','MISSION RECORDER START FAILED');return false}
}
async function stopNavigationGoalMissionRecording(resultState=''){
  if(!navigationGoalMasterActive())return false;if(navGoalMasterState.stopping)return false;navGoalMasterState.stopping=true;navGoalMasterState.result=resultState||navGoalMasterState.lastState||'UNKNOWN';
  try{
    captureLabTelemetry();captureNavigationGoalMasterFinalRows();
    const continuity=navigationGoalMasterContinuityReport();
    const masterTables=await saveNavigationGoalMasterCompanionTablesServer(),paths=masterTables.map(x=>x.path).filter(Boolean),browserGraphPngs=collectNavigationGoalMasterGraphPngPayload(),duration=Math.max(0,(Date.now()-recordStartedMs)/1000);
    const analysis_summary={duration_s:duration,chart_capture_hz:5,table_capture_hz:1,mission_result:navGoalMasterState.result,mission_id:navGoalMissionContext.missionId,primary_experiment_id:navGoalMissionContext.primaryId,cycle:navGoalMissionContext.cycle,candidate:navGoalMissionContext.candidate||$('runCandidate')?.value||'baseline',variation:navGoalMissionContext.variation||$('runVariation')?.value||'',condition:$('runCondition')?.value||'',goal:navGoalMissionContext.goal,start:navGoalMissionContext.start,table_continuity:continuity,strict_real_field:true,table_origin:'GUI_LIVE_1HZ_NO_BACKFILL',graph_origin:'GUI_BROWSER_CANVAS',synthetic_fill:false,expected_browser_graphs:navigationGoalMissionGraphs(navigationGoalMasterSections()).graphs.length};
    if(!continuity.ok)setText('recordMessage',`TABLE CONTINUITY WARNING • ${continuity.issues.join(' | ')} • package akan ditandai INCOMPLETE`);
    const table_entries=masterTables.map(x=>({path:x.path,section_id:x.section_id,table_index:x.table_index,table_name:x.table_name,columns:x.columns}));
    const r=await writeRequest('/api/experiment/record/stop',{table_csv_paths:paths,table_entries,browser_graph_pngs:browserGraphPngs,analysis_summary},{timeoutMs:180000}),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    renderTrialArtifacts(j);setText('recordPath',`RAW: ${j.primary_csv||j.raw_csv||'saved'}${j.xlsx_path?' • XLSX: '+j.xlsx_path:''}${j.data_package?.zip_path?' • ZIP: '+j.data_package.zip_path:''}`);setText('recordMessage',`${navGoalMissionContext.missionId} • ${navGoalMasterState.result} • 4.2–4.4 tersimpan • ${paths.length} tabel • ${j.graph_png_paths?.length||0} grafik • package ${j.data_package?.status||'--'}`);setText('navGoalBlockReason',`MISSION ${navGoalMasterState.result} • DATA 4.2–4.4 + ZIP SAVED`);
    recordingLeafId='';recordingDomain='';recordingRunToken='';setRecordingUi(false,j);await loadTrials();await window.navigationDataPackageOnMissionSaved?.(j.data_package,navGoalMasterState.result);toast(`Mission ${navGoalMasterState.result}: 4.2–4.4 + ZIP tersimpan`,false);return true;
  }catch(e){toast('Mission auto-stop/save gagal: '+e.message,true);setText('recordMessage','AUTO STOP/SAVE ERROR • '+e.message);try{await pollRecorder()}catch(_){}return false}
  finally{navGoalMasterState.stopping=false}
}
function navigationGoalMasterCaptureTick(){
  if(!navigationGoalMasterActive())return;if(!navGoalMasterState.armed)navGoalMasterState.armed=true;
  // Dedicated master watchdog: keep 5 Hz telemetry + 1 Hz tables alive regardless of active GUI page.
  // captureLabTelemetry() is internally de-duplicated at 5 Hz, so the 100 ms watchdog is safe.
  captureLabTelemetry();
  const st=String(obj('goal_state').state||'').trim().toUpperCase(),terminal=['SUCCEEDED','SUCCESS','ARRIVED','CANCELED','CANCELLED','ABORTED','FAILED','FAILURE','REJECTED'],idle=['','IDLE','UNKNOWN'];
  if(st&&!terminal.includes(st)&&!idle.includes(st))navGoalMasterState.sawActive=true;
  navGoalMasterState.lastState=st;
  if(terminal.includes(st)&&navGoalMasterState.sawActive&&!navGoalMasterState.stopping){
    setText('recordMessage',`${navGoalMissionContext.missionId} • Nav2 ${st} • AUTO STOP + SAVE 4.2–4.4…`);
    Promise.resolve(stopNavigationGoalMissionRecording(st)).catch(()=>{});
  }
}
window.startNavigationGoalMissionRecording=startNavigationGoalMissionRecording;
window.stopNavigationGoalMissionRecording=stopNavigationGoalMissionRecording;
window.navigationGoalMasterMetricPaths=navigationGoalMasterMetricPaths;
window.navigationGoalMasterSections=navigationGoalMasterSections;
window.captureNavigationGoalMasterCompanionRows=captureNavigationGoalMasterCompanionRows;
window.navigationGoalMasterCaptureTick=navigationGoalMasterCaptureTick;
setInterval(navigationGoalMasterCaptureTick,100);
function navigationReportTimedId(id=recordingLeafId||selectedExperiment?.id||''){return /^R4\.[1-3]\.\d+$/.test(String(id||''))}
function navigationReportGenericTimedId(id=recordingLeafId||selectedExperiment?.id||''){return /^R4\.[23]\.\d+$/.test(String(id||''))}
function clearNavigationReportAutoTimer(){if(navReportAutoTimer){clearInterval(navReportAutoTimer);navReportAutoTimer=null}navReportAutoStopBusy=false}
function navigationReportCountdownTick(){
  const id=String(recordingLeafId||selectedExperiment?.id||'');
  if(recordingDomain!=='navigation'||!navigationReportTimedId(id)||!recordStartedMs)return false;
  // 4.1 owns its source-rate 60 s timer in report_41_timed.js.
  if(window.report41TimedActive?.())return false;
  const elapsed=Math.max(0,(Date.now()-recordStartedMs)/1000),remaining=Math.max(0,NAV_REPORT_AUTO_STOP_SEC-elapsed),m=Math.floor(remaining/60),sec=remaining-m*60;
  setText('recordElapsed',`COUNTDOWN ${String(m).padStart(2,'0')}:${sec.toFixed(1).padStart(4,'0')}`);
  if(remaining<=0&&navigationReportGenericTimedId(id)&&!navReportAutoStopBusy){
    // Selection is locked while recording, so STOP uses the same report schema that START used.
    if(currentExp==='navigation'&&String(selectedExperiment?.id||'')===id){
      navReportAutoStopBusy=true;setText('recordMessage',`${displayExperimentId(selectedExperiment)} • 60.0 s selesai • AUTO STOP + SAVE sedang diproses…`);
      Promise.resolve(stopWebRecording()).catch(()=>{}).finally(()=>{navReportAutoStopBusy=false});
    }else setText('recordMessage',`${id} mencapai 60 s • kembali ke subbab recording untuk AUTO STOP + SAVE`);
  }
  return true;
}
function armNavigationReportAutoTimer(id=recordingLeafId){
  clearNavigationReportAutoTimer();
  if(recordingDomain!=='navigation'||!navigationReportGenericTimedId(id)||!recordStartedMs)return;
  navigationReportCountdownTick();navReportAutoTimer=setInterval(navigationReportCountdownTick,100);
}

async function startWebRecording(){
  if(!selectedExperiment)return toast('Pilih subbab dulu',true);
  const sid=String(selectedExperiment.id||''),navAllowed=/^N(?:[1-9]\d*)\.\d+$/.test(sid)||/^R4\.[1-4]\.[1-5]$/.test(sid);
  if(currentExp==='navigation'&&sid==='R4.4.5')return toast('4.4.5 AUTO: gunakan SEND GOAL di halaman Navigasi. Recorder 4.2–4.4 akan mulai otomatis.',true);
  if(currentExp==='navigation'&&!navAllowed)return toast('Pilih tahap Tuning Navigasi atau subjudul Laporan BAB IV terlebih dahulu',true);
  const gate=phaseGate(),navProfile=currentExp==='navigation'?navigationCaptureProfile():null;
  if((currentExp!=='navigation'||navProfile.phase)&&!gate.ok)return toast(`Commissioning terkunci: ${gate.reason}`,true);
  const universal=testPreflightStatus();if(!universal.ok)return toast(`Preflight gagal: ${universal.reason}`,true);
  const pre=automaticTrialPreflight();if(!pre.ok)return toast(`Preflight trial gagal: ${pre.reason}`,true);
  try{
    if(automaticTrialId())await validateTrialMotion(trialMotionSpec());
    const runId=reportRunId(),tuning_config=Object.fromEntries(parameterSnapshotEntries().map(x=>[`${x.file}:${tuningFieldsFor(selectedExperiment).find(p=>(p.label||p.key||p.yamlPath)===x.label)?.yamlPath||x.key}`,x.value]));
    const requestedRate=window.report41RequestedRate?.(sid),sampleRate=Number.isFinite(requestedRate)?requestedRate:+$('runSampleRate').value;
    const masterGoal=currentExp==='navigation'&&sid==='R4.4.5',trialInputs=trialInputsForSelected();
    if(masterGoal){trialInputs.capture_mode='GOAL_MASTER_4.2_4.3_4.4.5';trialInputs.master_sections=[...navigationGoalMasterSections().map(x=>x.id),'R4.4.5']}
    const payload={subsystem:currentExp,id:runId,source_experiment_id:selectedExperiment.id,label:runId,section_label:selectedExperiment.section||selectedExperiment.id,candidate:$('runCandidate')?.value||'baseline',variation:$('runVariation').value,condition:$('runCondition').value,sample_rate_hz:sampleRate,tuning_config,trial_inputs:trialInputs,live_series:selectedExperiment.liveSeries||{},graphs:normalizedExperimentGraphs(selectedExperiment).map((g,i)=>({...g,title:selectedExperiment.graphCaptions?.[i]||`Grafik ${i+1}`})),record_paths:masterGoal?navigationGoalMasterRecordPaths(selectedExperiment):recordPathsForSelected(selectedExperiment)};
    const r=await writeRequest('/api/experiment/record/start',payload),j=await r.json();
    toast(j.message||'Recorder response',!r.ok);if(!r.ok)return;
    recordStartedMs=Date.now();recordingLeafId=selectedExperiment.id;recordingDomain=currentExp;recordingRunToken=new Date(recordStartedMs).toISOString().replace(/[:.]/g,'-');window.analysisSession?.setSource('CURRENT_RECORDING');window.analysisSession?.selectTask(currentExp,selectedExperiment.id);chartEpochMs=recordStartedMs;reportEvidenceFrozen=false;reportEvidenceFrozenLeaf='';reportPathSnapshots.clear();reportSourceTableRenderMs=0;metricHistory.clear();scatterHistory.clear();scatterOrigins.clear();tableRunRows.clear();ekfGrowthState.clear();reportDerivedCache.clear();reportCostmapMetricCache={key:'',minClearance:NaN,valid:NaN};lastLabSecond=-1;lastLabTelemetryTick=-1;lastTrialArtifacts=null;resetNavigationGoalMaster(masterGoal);setRecordingUi(true,j.recording);window.startReport41TimedEvidence?.(sid);armNavigationReportAutoTimer(sid);captureLabTelemetry();renderExperimentTable();
    if(automaticTrialId())await startAutomaticTrialMotion();
    const timingNote=masterGoal?'MASTER 4.2 + 4.3 + 4.4.5 • AUTO STOP 0.5 s setelah Nav2 SUCCEEDED':currentExp==='navigation'&&navigationReportTimedId(sid)?'AUTO STOP + SAVE pada 60.0 s':currentExp==='navigation'&&/^R4\.4\./.test(sid)?'MANUAL STOP • durasi bebas > 60 s':'STOP manual';
    setText('recordMessage',`${displayExperimentId(selectedExperiment)} • ${automaticTrialId()?'AUTO ROS MOTION + ':''}${selectedExperiment.reportMode?'rekap laporan':'trial evidence'} • ${timingNote} • grafik reset t=0 • CSV + YAML + XLSX + PNG`)
  }catch(e){await stopAutomaticTrialMotion();toast('Start trial gagal: '+e.message,true)}
}
async function stopWebRecording(){
  try{
    if(!recordStartedMs)return toast('Tidak ada recording aktif',true);
    if(selectedExperiment?.id!==recordingLeafId||currentExp!==recordingDomain)return toast(`Konteks recording berubah dari ${recordingLeafId}; data tidak disimpan agar template tidak tertukar`,true);
    await stopAutomaticTrialMotion();window.finishReport41TimedEvidence?.('STOP');captureLabTelemetry();captureNavigationGoalMasterFinalRows();if(currentExp==='navigation'&&selectedExperiment?.reportMode===true){reportEvidenceFrozen=true;reportEvidenceFrozenLeaf=selectedExperiment.id}drawAllExperimentCharts();window.reportRefreshGraphSourceTables?.(true);const savedTables=await saveAllTemplateTablesServer(),masterTables=await saveNavigationGoalMasterCompanionTablesServer();
    const paths=[...savedTables,...masterTables].map(x=>x.path).filter(Boolean),browserGraphPngs=collectExperimentGraphPngPayload();
    const stopPayload={table_csv_paths:paths,browser_graph_pngs:browserGraphPngs,analysis_summary:buildClientAnalysisSummary()},stopTimeout=navigationGoalMasterActive()?90000:30000;
    const r=await writeRequest('/api/experiment/record/stop',stopPayload,{timeoutMs:stopTimeout}),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    renderTrialArtifacts(j);await loadTrials();
    setText('recordPath',`RAW: ${j.primary_csv||j.raw_csv||'saved'}${j.xlsx_path?' • XLSX: '+j.xlsx_path:''}${j.manifest_path?' • MANIFEST: '+j.manifest_path:''}`);
    setText('recordMessage',`STOP selesai • ${recordingLeafId} • trial YAML appended • ${paths.length} tabel CSV • ${j.graph_png_paths?.length||0} PNG Matplotlib • Excel siap`);
    const finalCursor=Math.max(0,(Date.now()-recordStartedMs)/1000);window.analysisSession?.setCursor(finalCursor,'record-stop');toast(`${recordingLeafId}: CSV + YAML trial + XLSX + PNG tersimpan`,false);setRecordingUi(false,j);recordingLeafId='';recordingDomain='';recordingRunToken='';renderExperimentTable();renderTrialRecap();return true
  }catch(e){await stopAutomaticTrialMotion();if(recordStartedMs){reportEvidenceFrozen=false;reportEvidenceFrozenLeaf=''}toast('Stop/save gagal: '+e.message,true);return false}
}
function setRecordingUi(active,info={}){const b=$('recordToggleBtn');if(b){b.dataset.active=active?'true':'false';b.textContent=active?'■ STOP + SAVE':'● START';b.className='record-toggle '+(active?'stop':'start')}setText('labRecorderState',active?'RECORDING':'IDLE');setText('labRecorderSamples',`${info.samples||0} samples`);if(!active){clearNavigationReportAutoTimer();resetNavigationGoalMaster(false);recordStartedMs=0;setText('recordElapsed','00:00.0')}}
function adoptObservedGoalMasterRecorder(j){
  if(!j?.active||String(j.subsystem)!=='navigation'||String(j.source_experiment_id)!=='R4.4.5'||recordStartedMs)return false;
  const started=Number.isFinite(Date.parse(j.started_at))?Date.parse(j.started_at):Date.now();
  recordStartedMs=started;recordingLeafId='R4.4.5';recordingDomain='navigation';recordingRunToken=String(j.started_at||new Date(started).toISOString()).replace(/[:.]/g,'-');
  chartEpochMs=started;lastLabSecond=-1;lastLabTelemetryTick=-1;metricHistory.clear();scatterHistory.clear();scatterOrigins.clear();tableRunRows.clear();reportPathSnapshots.clear();reportEvidenceFrozen=false;reportEvidenceFrozenLeaf='';
  const candidate=$('runCandidate')?.value||'baseline',variation=$('runVariation')?.value||`PROFILE-${String(candidate).toUpperCase()}`,goal=obj('goal_pose')||{},pose=obj('ekf_global')||{},primaryId='MISSION';
  const cycle=navigationGoalMissionNextCycle(primaryId,candidate,variation),missionId=`GOAL_AUTO_${new Date(started).toISOString().replace(/[-:TZ.]/g,'').slice(0,17)}_C${String(cycle).padStart(2,'0')}`;
  navGoalMissionContext={missionId,primaryId,candidate,variation,cycle,goal:{x:+goal.x,y:+goal.y,yaw_deg:Number.isFinite(+goal.yaw)?+goal.yaw*180/Math.PI:0,requested_yaw_deg:Number.isFinite(+goal.yaw)?+goal.yaw*180/Math.PI:0,position_priority:true},start:{x:+pose.x,y:+pose.y,yaw_rad:+pose.yaw||0}};
  resetNavigationGoalMaster(true);
  captureLabTelemetry();const sec=Math.max(0,Math.floor((Date.now()-started)/1000));captureNavigationGoalMasterCompanionRows(sec,true);lastLabSecond=sec;
  setText('recordMessage',`${missionId} • AUTO-ADOPT goal eksternal/HMI • RAW backend 20 Hz + tabel/grafik 4.2–4.4`);
  setText('navGoalBlockReason',`MISSION RECORDING AUTO-ADOPT • ${missionId}`);
  return true
}
async function pollRecorder(){
  try{
    const r=await readRequest('/api/experiment/record/status',{timeoutMs:2200}),j=await r.json();if(!r.ok)throw new Error(j.message||`recorder HTTP ${r.status}`);
    if(j.active&&!recordStartedMs)adoptObservedGoalMasterRecorder(j);
    setRecordingUi(!!j.active,j);
    if(j.active){
      if(!recordStartedMs&&j.started_at)recordStartedMs=Date.parse(j.started_at);
      if(!recordingLeafId)recordingLeafId=String(j.source_experiment_id||j.id||'');
      if(!recordingDomain)recordingDomain=String(j.subsystem||'');
      if(!recordingRunToken&&j.started_at)recordingRunToken=String(j.started_at).replace(/[:.]/g,'-')
    }
    if(j.active&&recordStartedMs){
      if(window.report41TimedActive?.())window.report41WriteCountdown?.();
      else if(navigationReportCountdownTick()){if(navigationReportGenericTimedId(recordingLeafId)&&!navReportAutoTimer)armNavigationReportAutoTimer(recordingLeafId)}
      else{const sec=(Date.now()-recordStartedMs)/1000,min=Math.floor(sec/60);setText('recordElapsed',`${String(min).padStart(2,'0')}:${(sec-min*60).toFixed(1).padStart(4,'0')}`)}
    }
  }catch(e){setText('labRecorderState',e.message?.includes('timeout')?'RETRYING':'UNAVAILABLE');setText('recordMessage',`Recorder status: ${e.message||'request failed'}`)}
}
$('recordToggleBtn').onclick=()=>{$('recordToggleBtn').dataset.active==='true'?(navigationGoalMasterActive()?stopNavigationGoalMissionRecording('MANUAL_STOP'):stopWebRecording()):startWebRecording()};if($('graphLiveMode'))$('graphLiveMode').onclick=resumeAnalysisLive;if($('qualificationPass'))$('qualificationPass').onclick=()=>submitQualification('PASS');if($('qualificationFail'))$('qualificationFail').onclick=()=>submitQualification('FAIL');if($('qualificationInvalidate'))$('qualificationInvalidate').onclick=()=>submitQualification('INVALIDATED');if($('testPreflightRefresh'))$('testPreflightRefresh').onclick=renderTestPreflight;if($('saveTableCsvBtn'))$('saveTableCsvBtn').onclick=()=>saveCurrentTemplateCsv(false);if($('saveTableExcelBtn'))$('saveTableExcelBtn').onclick=()=>{if(!lastTrialArtifacts?.xlsx_download_url)return toast('Belum ada hasil STOP dengan XLSX pada sesi ini',true);window.location.href=lastTrialArtifacts.xlsx_download_url};if($('trialCompareBtn'))$('trialCompareBtn').onclick=renderTrialComparison;if($('trialCompareClear'))$('trialCompareClear').onclick=clearTrialComparison;if($('refreshTrialsBtn'))$('refreshTrialsBtn').onclick=loadTrials;if($('optimalScaleBtn'))$('optimalScaleBtn').onclick=calculateApplyOptimalScale;if($('resetExperimentYaml'))$('resetExperimentYaml').onclick=resetSelectedExperimentYaml;if($('tuningValidateDraft'))$('tuningValidateDraft').onclick=validateTuningDrafts;if($('tuningApplyDraft'))$('tuningApplyDraft').onclick=applyTuningDrafts;if($('tuningRevertDraft'))$('tuningRevertDraft').onclick=revertTuningDrafts;if($('tuningQuickApplyBtn'))$('tuningQuickApplyBtn').onclick=quickApplyTuning;if($('tuningQuickVerifyBtn'))$('tuningQuickVerifyBtn').onclick=()=>verifySelectedTuningRuntime();if($('tuningQuickBaselineBtn'))$('tuningQuickBaselineBtn').onclick=quickRestoreBaseline;if($('tuningQuickDiscardBtn'))$('tuningQuickDiscardBtn').onclick=revertTuningDrafts;if($('homographyResetBtn'))$('homographyResetBtn').onclick=()=>{perceptionCalPoints=[];renderHomographyPoints()};if($('applyHomographyBtn'))$('applyHomographyBtn').onclick=applyHomographyPoints;if($('contextCamera'))$('contextCamera').addEventListener('click',captureHomographyPoint);if($('obcalCaptureBtn'))$('obcalCaptureBtn').onclick=captureObstacleCalibrationSample;if($('obcalDistance'))$('obcalDistance').onchange=renderObstacleCalibrationWizard;if($('obcalType'))$('obcalType').onchange=renderObstacleCalibrationWizard;if($('obcalOrientation'))$('obcalOrientation').onchange=renderObstacleCalibrationWizard;if($('obcalExportCsv'))$('obcalExportCsv').onclick=exportObstacleCalibrationCsv;if($('obcalExportGraph'))$('obcalExportGraph').onclick=exportObstacleCalibrationGraph;if($('obcalReset'))$('obcalReset').onclick=resetObstacleCalibration;if($('obcalSaveApply'))$('obcalSaveApply').onclick=saveObstacleCalibrationDraft;if($('obcalApplyYaml'))$('obcalApplyYaml').onclick=applyObstacleCalibrationYaml;$('refreshExperimentConfig').onclick=async()=>{await loadConfig();renderTuningFields();toast('YAML dimuat ulang')};$('experimentSearch').oninput=renderExperimentList;$('graphClearBtn').onclick=()=>{metricHistory.clear();scatterHistory.clear();scatterOrigins.clear();tableRunRows.clear();reportPathSnapshots.clear();reportEvidenceFrozen=false;reportEvidenceFrozenLeaf='';reportSourceTableRenderMs=0;chartEpochMs=Date.now();lastLabSecond=-1;lastLabTelemetryTick=-1;renderExperimentGraphs();renderExperimentTable()};

