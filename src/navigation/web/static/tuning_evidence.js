'use strict';
function renderExperimentTable(){
  const x=selectedExperiment,box=$('experimentTables');if(!box)return;const all=x?.tableColumns||[];
  if(!all.length){box.innerHTML='<div class="empty-state">Subbab ini tidak mendefinisikan tabel.</div>';return}
  const timed=tableHasTimeAxis();
  box.innerHTML=all.map((cols,ti)=>{const title=x.tableNames?.[ti]||`Tabel ${ti+1}`,head=timed?['t [s]',...cols]:cols,rows=tableRunRows.get(ti)||[];let body;if(rows.length)body=rows.slice(-300).map(r=>'<tr>'+((timed?[String(r.sec),...r.values]:r.values).map(v=>`<td>${escapeHtml(v)}</td>`).join(''))+'</tr>').join('');else{const vals=cols.map(c=>inferColumnValue(c));body='<tr class="live-preview-row">'+((timed?['0',...vals]:vals).map(v=>`<td>${escapeHtml(v)}</td>`).join(''))+'</tr>'}return `<section class="direct-table-card"><div class="direct-table-title"><span>Tabel ${ti+1}</span><b>${escapeHtml(title)}</b><small>${rows.length?(recordStartedMs?'RECORDING':'RECORDED')+' • '+rows.length+' row':'LIVE PREVIEW'}</small></div><div class="table-scroll"><table class="data-table experiment-data-table"><thead><tr>${head.map(c=>`<th>${escapeHtml(c)}</th>`).join('')}</tr></thead><tbody>${body}</tbody></table></div></section>`}).join('');
  setText('tableLiveHint',`${all.length} tabel langsung ditampilkan • ${recordStartedMs?'RECORDING 1 Hz':'preview live'}`)
}
function templateTableCsv(tableIndex=selectedTableIndex){const cols=selectedExperiment?.tableColumns?.[tableIndex]||[],rows=tableRunRows.get(tableIndex)||[],timed=tableHasTimeAxis(),yamlKeys=[...new Set(rows.flatMap(r=>(r.yaml||[]).map(x=>x.key)))],yamlLabels=Object.fromEntries(rows.flatMap(r=>(r.yaml||[]).map(x=>[x.key,x.label]))),head=[...(timed?['t [s]',...cols]:cols),...yamlKeys.map(k=>`YAML ${k} | ${yamlLabels[k]||''}`)];if(!head.length||!rows.length)return null;const esc=v=>'"'+String(v??'').replaceAll('"','""')+'"';return [head.map(esc).join(','),...rows.map(r=>{const ym=Object.fromEntries((r.yaml||[]).map(x=>[x.key,x.value]));return [...(timed?[r.sec,...r.values]:r.values),...yamlKeys.map(k=>ym[k]??'--')].map(esc).join(',')})].join('\n')+'\n'}
function saveCurrentTemplateCsv(auto=false){const csv=templateTableCsv();if(!csv){if(!auto)toast('Belum ada row hasil recording untuk tabel ini',true);return false}const safe=reportRunId(),name=`${safe}_T${selectedTableIndex+1}_${stamp()}.csv`;downloadBlob(name,'text/csv;charset=utf-8',csv);if(!auto)toast('Template table CSV disimpan');return true}
async function saveTemplateTableServer(tableIndex){const csv=templateTableCsv(tableIndex);if(!csv||!selectedExperiment)return null;const runId=reportRunId();const r=await writeRequest('/api/experiment/table/save',{subsystem:currentExp,id:runId,label:runId,source_experiment_id:selectedExperiment.id,table_index:tableIndex,csv}),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);return j}
async function saveAllTemplateTablesServer(){if(!selectedExperiment)return[];const count=(selectedExperiment.tableColumns||[]).length,results=[];for(let ti=0;ti<count;ti++){if(!(tableRunRows.get(ti)||[]).length)continue;try{const j=await saveTemplateTableServer(ti);if(j)results.push(j)}catch(e){toast(`Auto-save Tabel ${ti+1} gagal: ${e.message}`,true)}}return results}
function ensureContextSummary(){let e=$('contextSummary');if(e)return e;e=document.createElement('div');e.id='contextSummary';e.className='context-summary';$('contextVisual').appendChild(e);return e}
function navigationContextKind(){const id=String(selectedExperiment?.id||'');if(/^N(?:2|3)\./.test(id))return'esc';if(/^N(?:1|4|13|17)\./.test(id))return'summary';return'map'}
function renderNavigationSummary(){const e=ensureContextSummary(),id=String(selectedExperiment?.id||'');let rows=[];
  if(/^N1\./.test(id))rows=[['Wheelbase',configValue('vehicle','vehicle.ros__parameters.wheelbase_m'),'m'],['Track width',configValue('vehicle','vehicle.ros__parameters.track_width_m'),'m'],['Vehicle width',configValue('vehicle','vehicle.ros__parameters.total_width_m'),'m'],['Vehicle length',configValue('vehicle','vehicle.ros__parameters.vehicle_length_m'),'m'],['Wheel radius',configValue('vehicle','vehicle.ros__parameters.wheel_radius_m'),'m'],['Max forward',configValue('vehicle','vehicle.ros__parameters.max_forward_speed_mps'),'m/s']];
  else if(/^N4\./.test(id)){const im=obj('imu');rows=[['Roll',deg(im.roll_rad),'deg'],['Pitch',deg(im.pitch_rad),'deg'],['Yaw',deg(im.yaw_rad),'deg'],['Gyro X',im.gx,'rad/s'],['Gyro Y',im.gy,'rad/s'],['Gyro Z',im.gz,'rad/s']]}
  else if(/^N13\./.test(id))rows=[['Nav2 vx',obj('cmd_nav').linear_x,'m/s'],['Integrated vx',obj('cmd_autonomy_integrated').linear_x,'m/s'],['Pre-collision vx',obj('cmd_pre_collision').linear_x,'m/s'],['Final vx',obj('cmd_final').linear_x,'m/s'],['Actuator vx',obj('cmd_actuator').linear_x,'m/s'],['Actual vx',raw('esc_drive_actual'),'m/s']];
  else rows=[['Motion ready',bool(raw('system.motion_ready'))?'READY':'LOCKED',''],['Nav2',bool(raw('system.nav2_ready'))?'READY':'WAIT',''],['GNSS',bool(raw('connected.gnss'))?'ONLINE':'OFFLINE',''],['IMU',bool(raw('connected.imu'))?'ONLINE':'OFFLINE',''],['ESC',bool(raw('connected.esc_feedback'))?'ACK':'WAIT',''],['Goal',getPath(obj('goal_state'),'state')||'IDLE','']];
  e.innerHTML=`<div class="context-summary-head"><span>${escapeHtml(displayExperimentGroup(selectedExperiment))}</span><b>${escapeHtml(displayExperimentId(selectedExperiment))} live evidence</b></div><div class="context-summary-grid">${rows.map(([k,v,u])=>`<div><span>${escapeHtml(k)}</span><b>${typeof v==='number'?fmt(v,3):escapeHtml(v??'--')}</b><small>${escapeHtml(u||'')}</small></div>`).join('')}</div>`
}
function renderContextVisual(){if(!selectedExperiment)return;const map=$('contextMap'),cam=$('contextCamera'),esc=$('contextEsc'),summary=ensureContextSummary();map.style.display='none';cam.style.display='none';esc.style.display='none';summary.style.display='none';if(currentExp==='navigation'){const kind=navigationContextKind();if(kind==='map'){map.style.display='block';setText('contextTitle','Map • Path • Robot');drawExperimentMap()}else if(kind==='esc'){esc.style.display='block';setText('contextTitle',/^N2\./.test(String(selectedExperiment.id))?'Drive RPM • GNSS • ESC • IMU Speed':'Steering • Yaw • Multi-sensor');drawExperimentEsc()}else{summary.style.display='block';setText('contextTitle',/^N4\./.test(String(selectedExperiment.id))?'IMU Orientation & Motion':/^N13\./.test(String(selectedExperiment.id))?'Command Chain':'Engineering Evidence');renderNavigationSummary()}}else if(currentExp==='perception'){cam.style.display='block';setText('contextTitle','Camera • YOLOPv2');updateLabCamera()}else{esc.style.display='block';setText('contextTitle','FOC • Steering • Drive');drawExperimentEsc()}renderContextMetrics()}
function drawExperimentMap(){const c=$('expMapCanvas'),m=obj('map_meta');if(!c||!mapImage||!m.width){if($('expMapEmpty'))$('expMapEmpty').style.display='grid';return}const {ctx,w,h,dpr}=canvasFit(c,310),scale=Math.min(w/mapImage.width,h/mapImage.height),dw=mapImage.width*scale,dh=mapImage.height*scale,ox=(w-dw)/2,oy=(h-dh)/2;ctx.clearRect(0,0,w,h);ctx.fillStyle='#050b0d';ctx.fillRect(0,0,w,h);ctx.drawImage(mapImage,ox,oy,dw,dh);const tr=(x,y)=>[ox+((x-m.origin_x)/m.resolution)*scale,oy+(m.height-(y-m.origin_y)/m.resolution)*scale];function path(key,color,width){const data=obj(key),pts=data.points||[],frame=data.frame_id||'map';if(pts.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=width*dpr;ctx.beginPath();let started=false;for(const p of pts){const mp=framePointToMap(frame,+p[0],+p[1]);if(!mp)continue;const [x,y]=tr(mp.x,mp.y);started?ctx.lineTo(x,y):(ctx.moveTo(x,y),started=true)}if(started)ctx.stroke()}path('nav_path','#4aa8ff',2);path('local_path','#54d6e8',2);for(const t of (obj('mppi_trajectories').trajectories||[]).filter(x=>x.optimal)){ctx.strokeStyle='#ffc86b';ctx.lineWidth=3*dpr;ctx.beginPath();let started=false;for(const p of (t.points||[])){const mp=framePointToMap(t.frame_id||'odom',+p[0],+p[1]);if(!mp)continue;const [x,y]=tr(mp.x,mp.y);started?ctx.lineTo(x,y):(ctx.moveTo(x,y),started=true)}if(started)ctx.stroke()}const pose=currentPose();if(pose){const [x,y]=tr(pose.x,pose.y);ctx.fillStyle='#48e0a4';ctx.beginPath();ctx.arc(x,y,7*dpr,0,Math.PI*2);ctx.fill()}const goal=obj('goal_pose');if(Number.isFinite(+goal.x)){const [x,y]=tr(goal.x,goal.y);ctx.strokeStyle='#ff7b8a';ctx.lineWidth=2*dpr;ctx.beginPath();ctx.arc(x,y,8*dpr,0,Math.PI*2);ctx.stroke()}$('expMapEmpty').style.display='none'}
function updateLabCamera(){/* CameraFrameStore updates the active experiment camera. */}
function drawExperimentEsc(){const c=$('expEscCanvas');if(!c)return;const {ctx,w,h,dpr}=canvasFit(c,310),cx=w/2;ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.strokeStyle='rgba(130,160,165,.25)';ctx.lineWidth=2*dpr;ctx.beginPath();ctx.moveTo(60*dpr,110*dpr);ctx.lineTo(w-60*dpr,110*dpr);ctx.stroke();const stm=currentExp==='steering',target=deg(raw(stm?'esc_steer_protocol_cmd':'esc_steer_target'))||0,actual=deg(raw(stm?'esc_steer_feedback_raw':'esc_steer_actual'))||0,limit=stm?90:45,mapAngle=a=>cx+(Math.max(-limit,Math.min(limit,a))/limit)*(w*.36);[[stm?'STM TARGET':'TARGET',target,'#ffc86b',85],[stm?'STM FEEDBACK':'ACTUAL',actual,'#48e0a4',135]].forEach(([label,v,color,y])=>{ctx.fillStyle=color;ctx.fillRect(mapAngle(v)-3*dpr,y*dpr,6*dpr,48*dpr);ctx.font=`${11*dpr}px ui-monospace`;ctx.fillText(`${label} ${v.toFixed(2)}°`,18*dpr,(y+24)*dpr)});const foc=obj('foc_telemetry'),left=foc.left||obj('vesc_left_values'),right=foc.right||obj('vesc_right_values'),lines=[`Drive target ${fmt(raw('esc_drive_target'),3)} m/s`,`Drive actual ${fmt(raw('esc_drive_actual'),3)} m/s`,`LEFT Iq ${fmt(left.iq_a,3)} A / Id ${fmt(left.id_a,3)} A`,`RIGHT Iq ${fmt(right.iq_a,3)} A / Id ${fmt(right.id_a,3)} A`,`Vbus L/R ${fmt(left.vbus_v,2)} / ${fmt(right.vbus_v,2)} V`];ctx.fillStyle='rgba(210,230,230,.82)';ctx.font=`${12*dpr}px ui-monospace`;lines.forEach((t,i)=>ctx.fillText(t,18*dpr,(210+i*22)*dpr))}
function renderContextMetrics(){let metrics=[];if(currentExp==='navigation')metrics=[['Pose X',fmt(currentPose()?.x,3)+' m'],['Pose Y',fmt(currentPose()?.y,3)+' m'],['CTE',fmt(derivedCte(),3)+' m'],['Endpoint',fmt(derivedEndpoint(),3)+' m'],['Path',fmt(obj('nav_path').length_m,2)+' m'],['Goal',obj('goal_state').state||'--']];else if(currentExp==='perception')metrics=[['Camera',bool(raw('connected.camera'))?'ONLINE':'OFF'],['Health',bool(raw('camera_healthy'))?'HEALTHY':'WAIT'],['FPS',fmt(resolveMetricPath('perception_performance.fps'),2)],['Objects',obj('object_points').count??'--'],['Lane',obj('lane_state').state||obj('lane_state').raw||'--'],['Near field',obj('near_field_state').state||obj('near_field_state').raw||'--']];else{const foc=obj('foc_telemetry'),left=foc.left||obj('vesc_left_values'),right=foc.right||obj('vesc_right_values');metrics=[['ESC',bool(raw('connected.esc_feedback'))?'ACK':'WAIT'],['Drive',fmt(raw('esc_drive_actual'),3)+' m/s'],['Steer',fmt(deg(raw('esc_steer_actual')),2)+'°'],['LEFT Iq',fmt(left.iq_a,2)+' A'],['RIGHT Iq',fmt(right.iq_a,2)+' A'],['Mux',obj('esc_mux').raw||'--']]}$('contextMetrics').innerHTML=metrics.map(([k,v])=>`<div><span>${escapeHtml(k)}</span><b>${escapeHtml(v)}</b></div>`).join('')}
function normalizedExperimentGraphs(x=selectedExperiment){return (x?.graphCaptions||[]).map((_,i)=>{const g={...(x?.graphs?.[i]||{})};if(!g.type)g.type='time_series';if(g.type==='time_series'&&!(g.series||[]).length)g.series=autoGraphLabels(x,i);return g})}
function graphSourcePaths(x=selectedExperiment){const src=[...Object.values(x?.liveSeries||{})];for(const g of normalizedExperimentGraphs(x)){if(g?.xSeries)src.push(g.xSeries);if(g?.ySeries)src.push(g.ySeries);if(g?.pathKey)src.push(g.pathKey);for(const pair of (g?.pairs||[])){src.push(x?.liveSeries?.[pair?.[0]]||pair?.[0]);src.push(x?.liveSeries?.[pair?.[1]]||pair?.[1])}}return [...new Set(src.filter(Boolean))]}
function renderSourceAudit(){if(!selectedExperiment)return;const paths=graphSourcePaths(),pills=paths.map(p=>{const v=resolveMetricPath(p),ok=v!==undefined&&v!==null;return `<span class="${ok?'ok':'wait'}"><i></i>${escapeHtml(p)}</span>`});$('sourceAvailability').innerHTML=pills.join('')||'<span class="wait"><i></i>No direct live-series binding; gunakan tabel/config evidence.</span>';const method=currentExp==='navigation'?'Nav2/localization telemetry + map/path + parameter YAML':currentExp==='perception'?'Camera/YOLOPv2 metrics + safety telemetry + parameter YAML':'ESC feedback + FOC telemetry + calibration/tuning YAML';setText('sourceMethod',method);setText('experimentSourceBadge',paths.some(p=>resolveMetricPath(p)!=null)?'LIVE SOURCE':'WAITING SOURCE')}
// Recorder contract remains template-only report compatible; report recap only changes the selected schema/source mapping.
function recordPathsForSelected(){
  if(!selectedExperiment)return ['__meta_only__'];
  const out=new Set(),add=p=>{p=String(p||'').trim();if(p&&!p.startsWith('derived.'))out.add(p)};
  const live=Object.values(selectedExperiment.liveSeries||{});
  live.forEach(add);
  for(const g of normalizedExperimentGraphs(selectedExperiment)){
    if(g?.xSeries)add(g.xSeries);if(g?.ySeries)add(g.ySeries);if(g?.pathKey)add(g.pathKey);
    for(const pair of (g?.pairs||[])){add(selectedExperiment.liveSeries?.[pair?.[0]]||pair?.[0]);add(selectedExperiment.liveSeries?.[pair?.[1]]||pair?.[1])}
  }
  const derived=live.filter(p=>String(p).startsWith('derived.'));
  const need=(...paths)=>paths.forEach(add);
  for(const d of derived){
    if(d==='derived.steering_error_rad')need('esc_steer_target','esc_steer_actual');
    else if(d==='derived.ackermann_yaw_error')need('esc_yaw_rate','esc_kinematic_yaw_rate');
    else if(d==='derived.velocity_error_mps')need('cmd_final','esc_drive_actual');
    else if(d.startsWith('derived.ekf_local_'))need('ekf_local','esc_odom','gnss_base_vel_fusion','imu');
    else if(d.startsWith('derived.ekf_global_'))need('ekf_global','gnss_map_odom','gnss_base_vel_fusion','gnss_cog_fusion','imu');
    else if(d.includes('gnss_'))need('gnss_quality','gnss_fix','gnss_base_vel_fusion','gnss_cog_fusion');
    else if(d.includes('age_imu')||d.includes('rate_imu')||d.includes('dt_imu'))need('imu');
    else if(d.includes('age_esc')||d.includes('rate_esc')||d.includes('dt_esc'))need('esc_odom');
    else if(d.includes('age_ekf_local')||d.includes('rate_ekf_local'))need('ekf_local');
    else if(d.includes('age_ekf_global')||d.includes('rate_ekf_global'))need('ekf_global');
    else if(d.includes('cte')||d.includes('endpoint')||d.includes('heading_error'))need('localization_pose','ekf_local','ekf_global','nav_path');
  }
  if(currentExp==='steering'){
    if(live.some(p=>String(p).startsWith('vesc_command_state.')))need('vesc_command_state.mode','vesc_command_state.motor','vesc_command_state.value');
    if(selectedExperiment.id==='4.2.1')need('connected.vesc_transport','vesc_maintenance_active','vesc_tool_status');
    if(selectedExperiment.id==='4.9')need('vesc_left_values','vesc_right_values','vesc_steering_state','esc_status','esc_mux');
  }
  // Never send an empty allow-list: the server interprets empty as "record all".
  return out.size?[...out]:['__meta_only__'];
}
function testRequiredRoots(x=selectedExperiment){
  if(!x)return [];
  const roots=new Set();
  for(const path of graphSourcePaths(x)){
    const text=String(path||'');if(!text||text.startsWith('derived.'))continue;
    roots.add(text.split('.')[0]);
  }
  if(currentExp==='steering')roots.add('foc_telemetry');
  if(currentExp==='navigation')roots.add('esc_odom');
  if(currentExp==='perception'){roots.add('camera_frame');roots.add('perception_performance')}
  return [...roots];
}
function testPreflightStatus(){
  if(!selectedExperiment)return {ok:false,items:[],reason:'Pilih subpengujian'};
  const items=[];const add=(label,ok,detail,critical=true)=>items.push({label,ok:ok===true,detail,critical});
  const replay=window.AdvReplay?.isActive?.()===true;add('Live control mode',!replay,replay?'REPLAY MODE aktif':'LIVE MODE',true);
  add('ROS bridge',obj('server').ros===true,obj('server').ros===true?'ROS context active':'ROS bridge belum ready',true);
  const phase=phaseGate();add('Commissioning prerequisite',phase.ok,phase.reason,true);
  if(currentExp==='steering'){
    const link=bool(raw('connected.esc_feedback'))||bool(raw('connected.vesc_transport'));
    add('ESC transport / feedback',link,link?'gateway/feedback tersedia':'ESC transport dan feedback belum tersedia',true);
  }else if(currentExp==='navigation'){
    add('ESC feedback',bool(raw('connected.esc_feedback')),bool(raw('connected.esc_feedback'))?'actuator feedback fresh':'ESC feedback belum fresh',true);
  }else if(currentExp==='perception'){
    add('Camera',bool(raw('connected.camera'))&&channelFresh('camera_frame',2.5),`frame age ${Number.isFinite(age('camera_frame'))?age('camera_frame').toFixed(2):'--'} s`,true);
  }
  for(const root of testRequiredRoots()){
    const reg=CHANNEL_REGISTRY[root],a=age(root),rate=channelRate(root),fresh=reg?a<(reg.staleMs/1000):Number.isFinite(a)&&a<5;
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
  if(chip){chip.textContent=p.ok?'READY':'BLOCKED';chip.className='status-chip '+(p.ok?'':'bad')}
  if(summary)summary.textContent=p.ok?'Semua source wajib dan safety gate siap untuk START.':`START dikunci: ${p.reason||'required source belum siap'}`;
  const b=$('recordToggleBtn');if(b&&b.dataset.active!=='true'){b.disabled=!p.ok;b.title=p.ok?'Ready to record':summary?.textContent||'Preflight blocked'}
}
async function startWebRecording(){
  if(!selectedExperiment)return toast('Pilih subbab dulu',true);
  const sid=String(selectedExperiment.id||''),navAllowed=/^N(?:[1-9]\d*)\.\d+$/.test(sid)||/^R4\.[1-4]\.[1-3]$/.test(sid);
  if(currentExp==='navigation'&&!navAllowed)return toast('Pilih tahap Tuning Navigasi atau subjudul Laporan BAB IV terlebih dahulu',true);
  const gate=phaseGate();if(!gate.ok)return toast(`Commissioning terkunci: ${gate.reason}`,true);
  const universal=testPreflightStatus();if(!universal.ok)return toast(`Preflight gagal: ${universal.reason}`,true);
  const pre=automaticTrialPreflight();if(!pre.ok)return toast(`Preflight trial gagal: ${pre.reason}`,true);
  try{
    if(automaticTrialId())await validateTrialMotion(trialMotionSpec());
    const runId=reportRunId(),tuning_config=Object.fromEntries(parameterSnapshotEntries().map(x=>[`${x.file}:${tuningFieldsFor(selectedExperiment).find(p=>(p.label||p.key||p.yamlPath)===x.label)?.yamlPath||x.key}`,x.value]));
    const payload={subsystem:currentExp,id:runId,source_experiment_id:selectedExperiment.id,label:runId,section_label:selectedExperiment.section||selectedExperiment.id,candidate:$('runCandidate')?.value||'baseline',variation:$('runVariation').value,condition:$('runCondition').value,sample_rate_hz:+$('runSampleRate').value,tuning_config,trial_inputs:trialInputsForSelected(),live_series:selectedExperiment.liveSeries||{},graphs:normalizedExperimentGraphs(selectedExperiment).map((g,i)=>({...g,title:selectedExperiment.graphCaptions?.[i]||`Grafik ${i+1}`})),record_paths:recordPathsForSelected()};
    const r=await writeRequest('/api/experiment/record/start',payload),j=await r.json();
    toast(j.message||'Recorder response',!r.ok);if(!r.ok)return;
    recordStartedMs=Date.now();recordingLeafId=selectedExperiment.id;recordingDomain=currentExp;recordingRunToken=new Date(recordStartedMs).toISOString().replace(/[:.]/g,'-');window.analysisSession?.setSource('CURRENT_RECORDING');window.analysisSession?.selectTask(currentExp,selectedExperiment.id);chartEpochMs=recordStartedMs;metricHistory.clear();scatterHistory.clear();scatterOrigins.clear();tableRunRows.clear();ekfGrowthState.clear();reportDerivedCache.clear();reportCostmapMetricCache={key:'',minClearance:NaN,valid:NaN};lastLabSecond=-1;lastTrialArtifacts=null;setRecordingUi(true,j.recording);captureLabTelemetry();renderExperimentTable();
    if(automaticTrialId())await startAutomaticTrialMotion();
    setText('recordMessage',`${displayExperimentId(selectedExperiment)} • ${automaticTrialId()?'AUTO ROS MOTION + ':''}${selectedExperiment.reportMode?'rekap laporan':'trial evidence'} • grafik reset t=0 • STOP → CSV + YAML + XLSX + PNG`)
  }catch(e){await stopAutomaticTrialMotion();toast('Start trial gagal: '+e.message,true)}
}
async function stopWebRecording(){
  try{
    if(!recordStartedMs)return toast('Tidak ada recording aktif',true);
    if(selectedExperiment?.id!==recordingLeafId||currentExp!==recordingDomain)return toast(`Konteks recording berubah dari ${recordingLeafId}; data tidak disimpan agar template tidak tertukar`,true);
    await stopAutomaticTrialMotion();captureLabTelemetry();drawAllExperimentCharts();const savedTables=await saveAllTemplateTablesServer();
    const paths=savedTables.map(x=>x.path).filter(Boolean),browserGraphPngs=collectExperimentGraphPngPayload();
    const stopPayload={table_csv_paths:paths,browser_graph_pngs:browserGraphPngs};
    const r=await writeRequest('/api/experiment/record/stop',stopPayload),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    renderTrialArtifacts(j);await loadTrials();
    setText('recordPath',`RAW: ${j.primary_csv||j.raw_csv||'saved'}${j.xlsx_path?' • XLSX: '+j.xlsx_path:''}${j.manifest_path?' • MANIFEST: '+j.manifest_path:''}`);
    setText('recordMessage',`STOP selesai • ${recordingLeafId} • trial YAML appended • ${paths.length} tabel CSV • ${j.graph_png_paths?.length||0} PNG Matplotlib • Excel siap`);
    const finalCursor=Math.max(0,(Date.now()-recordStartedMs)/1000);window.analysisSession?.setCursor(finalCursor,'record-stop');toast(`${recordingLeafId}: CSV + YAML trial + XLSX + PNG tersimpan`,false);setRecordingUi(false,j);recordingLeafId='';recordingDomain='';recordingRunToken='';renderExperimentTable();renderTrialRecap()
  }catch(e){await stopAutomaticTrialMotion();toast('Stop/save gagal: '+e.message,true)}
}
function setRecordingUi(active,info={}){const b=$('recordToggleBtn');if(b){b.dataset.active=active?'true':'false';b.textContent=active?'■ STOP + SAVE':'● START';b.className='record-toggle '+(active?'stop':'start')}setText('labRecorderState',active?'RECORDING':'IDLE');setText('labRecorderSamples',`${info.samples||0} samples`);if(!active){recordStartedMs=0;setText('recordElapsed','00:00.0')}}
async function pollRecorder(){try{const r=await readRequest('/api/experiment/record/status',{timeoutMs:2200}),j=await r.json();if(!r.ok)throw new Error(j.message||`recorder HTTP ${r.status}`);setRecordingUi(!!j.active,j);if(j.active){if(!recordStartedMs&&j.started_at)recordStartedMs=Date.parse(j.started_at);if(!recordingLeafId)recordingLeafId=String(j.source_experiment_id||j.id||'');if(!recordingDomain)recordingDomain=String(j.subsystem||'');if(!recordingRunToken&&j.started_at)recordingRunToken=String(j.started_at).replace(/[:.]/g,'-')}if(j.active&&recordStartedMs){const sec=(Date.now()-recordStartedMs)/1000,min=Math.floor(sec/60);setText('recordElapsed',`${String(min).padStart(2,'0')}:${(sec-min*60).toFixed(1).padStart(4,'0')}`)}}catch(e){setText('labRecorderState',e.message?.includes('timeout')?'RETRYING':'UNAVAILABLE');setText('recordMessage',`Recorder status: ${e.message||'request failed'}`)}}
$('recordToggleBtn').onclick=()=>{$('recordToggleBtn').dataset.active==='true'?stopWebRecording():startWebRecording()};if($('graphLiveMode'))$('graphLiveMode').onclick=resumeAnalysisLive;if($('qualificationPass'))$('qualificationPass').onclick=()=>submitQualification('PASS');if($('qualificationFail'))$('qualificationFail').onclick=()=>submitQualification('FAIL');if($('qualificationInvalidate'))$('qualificationInvalidate').onclick=()=>submitQualification('INVALIDATED');if($('testPreflightRefresh'))$('testPreflightRefresh').onclick=renderTestPreflight;if($('saveTableCsvBtn'))$('saveTableCsvBtn').onclick=()=>saveCurrentTemplateCsv(false);if($('saveTableExcelBtn'))$('saveTableExcelBtn').onclick=()=>{if(!lastTrialArtifacts?.xlsx_download_url)return toast('Belum ada hasil STOP dengan XLSX pada sesi ini',true);window.location.href=lastTrialArtifacts.xlsx_download_url};if($('trialCompareBtn'))$('trialCompareBtn').onclick=renderTrialComparison;if($('trialCompareClear'))$('trialCompareClear').onclick=clearTrialComparison;if($('refreshTrialsBtn'))$('refreshTrialsBtn').onclick=loadTrials;if($('optimalScaleBtn'))$('optimalScaleBtn').onclick=calculateApplyOptimalScale;if($('resetExperimentYaml'))$('resetExperimentYaml').onclick=resetSelectedExperimentYaml;if($('tuningValidateDraft'))$('tuningValidateDraft').onclick=validateTuningDrafts;if($('tuningApplyDraft'))$('tuningApplyDraft').onclick=applyTuningDrafts;if($('tuningRevertDraft'))$('tuningRevertDraft').onclick=revertTuningDrafts;if($('homographyResetBtn'))$('homographyResetBtn').onclick=()=>{perceptionCalPoints=[];renderHomographyPoints()};if($('applyHomographyBtn'))$('applyHomographyBtn').onclick=applyHomographyPoints;if($('contextCamera'))$('contextCamera').addEventListener('click',captureHomographyPoint);if($('obcalCaptureBtn'))$('obcalCaptureBtn').onclick=captureObstacleCalibrationSample;if($('obcalNextBtn'))$('obcalNextBtn').onclick=nextObstacleCalibrationDistance;if($('obcalDistance'))$('obcalDistance').onchange=renderObstacleCalibrationWizard;if($('obcalType'))$('obcalType').onchange=renderObstacleCalibrationWizard;if($('obcalOrientation'))$('obcalOrientation').onchange=renderObstacleCalibrationWizard;if($('obcalExportCsv'))$('obcalExportCsv').onclick=exportObstacleCalibrationCsv;if($('obcalExportGraph'))$('obcalExportGraph').onclick=exportObstacleCalibrationGraph;if($('obcalReset'))$('obcalReset').onclick=resetObstacleCalibration;if($('obcalSaveApply'))$('obcalSaveApply').onclick=saveApplyObstacleCalibration;$('refreshExperimentConfig').onclick=async()=>{await loadConfig();renderTuningFields();toast('YAML dimuat ulang')};$('experimentSearch').oninput=renderExperimentList;$('graphClearBtn').onclick=()=>{metricHistory.clear();scatterHistory.clear();scatterOrigins.clear();tableRunRows.clear();chartEpochMs=Date.now();lastLabSecond=-1;renderExperimentGraphs();renderExperimentTable()};

