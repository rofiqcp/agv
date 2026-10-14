/* BAB IV Perception v3 — backend-owned session, canonical post-flip frame. */
window.__BAB4_PERCEPTION_V3__=true;
const V3_IDS=['F4.1','F4.2','F4.3','F4.4','F4.5'];
const V3_42=[1,2,3,4,5];
const V3_42_LIGHT=[['night','Malam / minim cahaya'],['day','Siang / banyak cahaya']];
const V3_43=[
 ['condition_01','GREEN','GREEN','Kondisi Aman Simetris'],
 ['condition_02','YELLOW','GREEN','Peringatan Sisi Kiri'],
 ['condition_03','RED','GREEN','Kondisi Kritis Sisi Kiri'],
 ['condition_04','GREEN','YELLOW','Peringatan Sisi Kanan'],
 ['condition_05','GREEN','RED','Kondisi Kritis Sisi Kanan'],
 ['condition_06','GRAY','GRAY','Area Tidak Dapat Dilalui']
];
const V3_44=[['condition_A','Jalur Relatif Kosong'],['condition_B','Satu Manusia'],['condition_C','Kondisi Kompleks']];
const V3={state:null,sel42:1,light42:'night',sel43:'condition_01',sel44:'condition_A',tab45:'obstacle',obs45:'1',lane45:'1',
  validationStarted:false,orientationLeft:false,orientationRight:false,captured:null,capturedImage:'',cameraHome:null,cameraNext:null,
  auto42Timer:null,auto42Cond:null,auto42Busy:false,auto43Timer:null,auto43Cond:null,auto43Busy:false,
  run44:null,timer44:null,busy44:false,preview45Busy:false,armed:false,armedToken:'',armedSessionId:'',heartbeat:null,heartbeatBusy:false,heartbeatWorker:null,heartbeatFails:0,armedMode45:'',armedVisual45:null,armedCondition45:'',visualHuman45:null,visualHuman45At:0,obsMetricLatchCond45:'',obsMetricLatchUntil45:0,obsMetricLatchDistance45:NaN,exports:{}};
const v3esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const v3num=(v,n=2)=>Number.isFinite(+v)?(+v).toFixed(n):'--';
function v3Key42(light,d){return 'human_'+light+'_'+d+'m'}
function v3Rows42(light,d){
  const explicit=v3Rows('4.2',v3Key42(light,d));
  if(explicit.length||light!=='night')return explicit;
  return v3Rows('4.2','human_'+d+'m'); // legacy dataset = malam
}
function v3Count42(light,d){return v3Rows42(light,d).length}
function v3IsActive(){return activeDomain==='perception'&&activeWorkspaceTab?.perception==='bab4'&&V3_IDS.includes(String(selectedExperiment?.id||''))}
function v3PatchCatalog(){
  const list=experiments.perception||[];let f=list.find(x=>x.id==='F4.5');
  if(!f){const base=list.find(x=>x.id==='F4.4')||list[0];if(base){f=JSON.parse(JSON.stringify(base));f.id='F4.5';list.push(f)}}
  const names={'F4.1':'4.1 Validasi Sistem Persepsi','F4.2':'4.2 Pengujian Deteksi Manusia','F4.3':'4.3 Pengujian Lane Safety',
    'F4.4':'4.4 Pengujian Kinerja Waktu Nyata','F4.5':'4.5 Pengujian Respons Kendaraan'};
  V3_IDS.forEach(id=>{const x=list.find(y=>y.id===id);if(x){x.section=names[id];x.groupTitle=names[id];x.reportMode=true}})
}async function v3FetchState(){
  try{const r=await fetch('/api/perception/bab4/state',{cache:'no-store'}),j=await r.json();if(!j.ok)throw Error(j.message||'state gagal');
    V3.state=j.state;if(v3IsActive())v3Render(false);return j.state}catch(e){toast('Backend Bab 4 offline: '+e.message,true);return null}
}
async function v3Post(path,payload,timeoutMs=0){
  const body=Object.assign({},payload||{}),bab4Mutation=path.startsWith('/api/perception/bab4/')&&(path!=='/api/perception/bab4/session'||body.action==='reset');
  const finalTrialMutation=path.startsWith('/api/perception/final-trial/');
  if(bab4Mutation){
    const sid=V3.state?.session_id;if(!sid)throw Error('Session Bab 4 belum termuat');
    body.session_id=sid;
  }
  if(finalTrialMutation&&!body.session_id){
    const sid=V3.armedSessionId||V3.state?.session_id;if(!sid)throw Error('Session ARMED belum termuat');
    body.session_id=sid;
  }
  const ctl=timeoutMs>0?new AbortController():null;
  const timeout=ctl?setTimeout(()=>ctl.abort(),timeoutMs):null;
  let r;
  try{
    r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),...(ctl?{signal:ctl.signal}:{})});
  }catch(e){
    if(e?.name==='AbortError'){const x=Error('request timeout');x.code='TIMEOUT';throw x}
    throw e
  }finally{if(timeout)clearTimeout(timeout)}
  let j={};try{j=await r.json()}catch(_){}
  if(!r.ok||!j.ok){const e=Error(j.message||('HTTP '+r.status));e.code=j.code||'';e.activeSessionId=j.active_session_id||'';throw e}
  if(j.state)V3.state=j.state;if(j.captured_image_data_url)V3.capturedImage=j.captured_image_data_url;if(j.row)V3.captured=j.row;
  return j;
}
function v3Sub(k){return V3.state?.subchapters?.[k]||{conditions:{}}}
function v3Cond(k,c){return v3Sub(k).conditions?.[c]||{rows:[],sample_count:0,status:'BELUM'}}
function v3Rows(k,c){return v3Cond(k,c).rows||[]}
function v3Count(k,c){return v3Rows(k,c).length}
function v345ConditionComplete(c){
  const x=v3Cond('4.5',c),rows=Array.isArray(x.rows)?x.rows:[];
  const dryPass=x.dry_run_pass===true||rows.some(r=>r.mode==='DRY_RUN'&&(r.integration_pass===true||r.result==='DRY_RUN_PASS'));
  const armedComplete=x.armed_complete===true||rows.some(r=>r.mode==='ARMED_VEHICLE_TEST'&&String(r.result||'').toLowerCase()==='complete'&&r.final_vehicle_state==='STOPPED'&&r.integration_pass===true);
  const needVisual=String(c).startsWith('obstacle_');
  const dryVisual=rows.some(r=>r.mode==='DRY_RUN'&&r.map_filename&&r.combined_filename);
  const armedVisual=rows.some(r=>r.mode==='ARMED_VEHICLE_TEST'&&r.map_filename&&r.combined_filename);
  return dryPass&&armedComplete&&(!needVisual||(dryVisual&&armedVisual))
}
function v3ExportReady45(){
  const obs=[1,2,3].map(i=>'obstacle_trial_'+i);
  const lane=[1,2,3].map(i=>'lane_trial_'+i);
  return [...obs,...lane].every(v345ConditionComplete)
}
function v3Window(k,c){const x=v3Cond(k,c),left=Math.max(0,Math.ceil(((+x.deadline_ms||0)-Date.now())/1000));return {status:x.status||'BELUM',left}}
function v3Human(evaluation=false){
  const s=obj('semantic_detections');
  const a=evaluation&&Array.isArray(s.evaluation_candidates)?s.evaluation_candidates:(Array.isArray(s.detections)?s.detections:[]);
  const hum=a.filter(x=>+x.class_id===0||['person','human'].includes(String(x.class_name||'').toLowerCase()));
  hum.sort((a,b)=>(+b.score||0)-(+a.score||0));return hum[0]||null
}
function v3HumanCandidates(){
  const s=obj('semantic_detections'),a=Array.isArray(s.detections)&&s.detections.length?s.detections:(Array.isArray(s.evaluation_candidates)?s.evaluation_candidates:[]);
  return a.filter(x=>+x.class_id===0||['person','human'].includes(String(x.class_name||'').toLowerCase())).sort((a,b)=>(+b.score||0)-(+a.score||0))
}
function v3Human45Detected(){
  if(!v3SemanticFresh())return null;
  return v3Human(false)
}
function v3Human45(){if(!v3SemanticFresh())return null;const s=obj('semantic_detections'),a=Array.isArray(s.detections)?s.detections:[];return a.filter(x=>(+x.class_id===0||['person','human'].includes(String(x.class_name||'').toLowerCase()))&&x.nav2_eligible!==false&&((+x.score||0)>=0.20||x.safety_human_valid===true)).sort((a,b)=>{const ap=a.safety_human_valid===true?1:0,bp=b.safety_human_valid===true?1:0;if(bp!==ap)return bp-ap;const at=['STRONG','TRACKED','HELD'].includes(String(a.tracking_state||'').toUpperCase())?1:0,bt=['STRONG','TRACKED','HELD'].includes(String(b.tracking_state||'').toUpperCase())?1:0;if(bt!==at)return bt-at;return (+b.score||0)-(+a.score||0)})[0]||null}
function v3SemanticFresh(){return channelFresh('semantic_detections',1.2)}
function v3SemanticAge(){const a=typeof age==='function'?age('semantic_detections'):NaN;return Number.isFinite(+a)?+a:NaN}
function v3Lane(){
  const l=obj('lane_state'),c=l.corridor||{},off=+l.center_error_m;
  const hasVisual=Boolean(c.drivable_detected||c.lane_mask_detected||c.raw_lane_mask_detected);
  const laneValid=l.valid===true&&c.drivable_detected===true;
  const evidence=String(c.evidence_mode||'').toUpperCase(),recommendation=String(c.recommendation||'').toUpperCase(),laneState=String(l.state||'UNKNOWN').toUpperCase();
  const invalid=!laneValid||evidence.includes('NO_EVIDENCE')||recommendation==='NO_MASK_EVIDENCE'||recommendation==='DRIVABLE_INVALID'||(!hasVisual&&['UNKNOWN','LANE_LOST'].includes(laneState));
  const left=invalid?'GRAY':String(c.left_status||'UNKNOWN').toUpperCase();
  const right=invalid?'GRAY':String(c.right_status||'UNKNOWN').toUpperCase();
  return {left,right,
    leftGap:invalid?NaN:+c.left_gap_m_est,rightGap:invalid?NaN:+c.right_gap_m_est,
    leftGapPx:invalid?NaN:+c.left_gap_px,rightGapPx:invalid?NaN:+c.right_gap_px,
    offset:invalid?NaN:off,width:l.road_width_m,
    leftLatched:c.left_recenter_latched===true,rightLatched:c.right_recenter_latched===true,
    state:laneState,action:invalid?'STOP_NO_DRIVABLE':(c.recommendation||'--'),confidence:l.confidence,
    drivable:c.drivable_detected===true,laneValid,invalid}
}
function v3Perf(){const p=obj('perception_performance');return {fps:p.pipeline_fps,inference:p.inference_ms,processing:p.pipeline_ms_per_frame??p.pipeline_ms,
  age:p.frame_age_ms,drop:p.capture_dropped_total,overwrite:p.capture_overwrite_total,capture:p.capture_fps}}
function v3CameraReady(){return !!raw('connected.camera')&&!!raw('camera_healthy')&&channelFresh('camera_frame',2.5)}
function v3AttachCamera(){
  const img=$('v3CameraImage'),placeholder=$('v3CameraEmpty');if(!img||typeof cameraFrameStore==='undefined'||!cameraFrameStore)return;
  img.style.setProperty('transform','scaleX(-1)','important');
  cameraFrameStore.subscribe('bab4-v3',{img,placeholder,isActive:()=>v3IsActive()&&elementVisible($('v3CameraHost')),onFrame:()=>v3DrawOverlay()});
}
function v3RestoreCamera(){}
function v3Nav(){
  return '<div class="lf-nav">'+V3_IDS.map(id=>'<button data-v3-id="'+id+'" class="'+(selectedExperiment?.id===id?'active':'')+'"><em>'+id.replace('F','')+'</em><b>'+
    v3esc({'F4.1':'Validasi Sistem','F4.2':'Deteksi Manusia','F4.3':'Lane Safety','F4.4':'Kinerja Waktu Nyata','F4.5':'Respons Kendaraan'}[id])+'</b></button>').join('')+'</div>'
}
function v3SessionBar(){
  const id=V3.state?.session_id||'MEMUAT...';
  return '<div class="v3-session"><b>SESSION AKTIF: '+v3esc(id)+'</b><span>Canonical: SAMA DENGAN OVERVIEW • LEFT = kiri layar • RIGHT = kanan layar</span>'+
    '<div class="lf-actions"><button class="soft-btn" id="v3NewSession">Mulai Session Baru</button><button class="danger-btn" id="v3ResetSession">Reset Session Bab 4</button>'+
    '<a class="primary-btn" href="/api/perception/bab4/export/full.zip">Download Full Bab 4</a></div></div>'
}
function v3LivePreview(){
  const id=selectedExperiment?.id,h=(id==='F4.5'&&V3.tab45==='obstacle')?v3Human45Detected():v3Human(id==='F4.2'),l=v3Lane(),p=v3Perf();
  return '<div class="lf-preview"><div class="lf-preview-head"><b>LIVE PREVIEW — ORIENTASI SAMA DENGAN OVERVIEW</b><span id="v3Stream">'+(v3CameraReady()?'LIVE':'STALE')+'</span></div>'+
    '<div class="lf-preview-stage" id="v3CameraHost"><img id="v3CameraImage" alt="Live camera Bab 4"><div class="v3-camera-empty" id="v3CameraEmpty">Menunggu frame kamera...</div><canvas id="v3Overlay"></canvas></div>'+
    '<div class="lf-live-strip">Human: <b>'+v3esc(h?((h.class_name||'human')+' • '+v3num((+h.score||0)*100,1)+'%'):'tidak terdeteksi')+
    '</b> &nbsp; | &nbsp; LEFT <b>'+v3esc(l.left)+'</b> • RIGHT <b>'+v3esc(l.right)+'</b> &nbsp; | &nbsp; FPS <b>'+v3num(p.fps,2)+
    '</b> • Inference <b>'+v3num(p.inference,1)+' ms</b> • Frame Age <b>'+v3num(p.age,1)+' ms</b></div></div>'
}
function v3CapturedBlock(){
  if(!V3.captured)return '<div class="lf-message">Belum ada captured event pada sesi browser ini. Progress penelitian tetap dibaca dari backend session.</div>';
  const r=V3.captured;return '<div class="v3-captured"><b>CAPTURED EVENT TERAKHIR — row yang sama dengan CSV</b>'+
    (V3.capturedImage?'<img src="'+V3.capturedImage+'" alt="Captured event">':'')+
    '<pre>'+v3esc(JSON.stringify(r,null,2))+'</pre></div>'
}
function v3Shell(body){
  return '<div class="lf-head"><div><span class="eyebrow">BAB IV • PERSEPSI • BACKEND SESSION V6</span><h2>'+v3esc(selectedExperiment?.section||'Final Dataset Bab 4')+
    '</h2><p>Data flow tunggal: source frame → processing → GUI → capture → CSV → summary/grafik → ZIP.</p></div>'+
    '<span class="status-chip '+(v3CameraReady()?'':'bad')+'">'+(v3CameraReady()?'CAMERA LIVE':'CAMERA STALE')+'</span></div>'+
    v3SessionBar()+v3Nav()+'<div class="lf-layout">'+v3LivePreview()+'<div class="lf-control">'+body+'</div></div>'+
    '<div class="lf-lower"><div class="lf-section-title"><b>Hasil / Ringkasan</b><span>Seluruh counter berasal dari backend state.</span></div>'+
    '<canvas id="v3Chart" width="1200" height="300"></canvas>'+v3CapturedBlock()+'<div id="v3Table"></div></div>'
}
function v3Render41(){
  const p=v3Perf(),latest=v3Rows('4.1','validation').slice(-1)[0],ori=V3.orientationLeft&&V3.orientationRight,perf=obj('perception_performance');
  const yolo=!!perf.inference_enabled,semantic=v3SemanticFresh(),laneFresh=channelFresh('lane_state',2.5);
  const functional=v3CameraReady()&&yolo&&semantic&&laneFresh&&ori;
  const dataValid=!!latest&&latest.stream_status==='LIVE'&&latest.semantic_stream==='LIVE'&&latest.lane_stream==='LIVE'&&latest.orientation_left_ok&&latest.orientation_right_ok;
  return '<div class="lf-section-title"><b>4.1 Validasi Sistem Persepsi</b><span>Gate sebelum 4.2–4.5.</span></div>'+
    '<div class="lf-kpis"><div><span>Camera Stream</span><b>'+(v3CameraReady()?'PASS':'FAIL')+'</b></div><div><span>Camera Flip</span><b>PASS • OVERVIEW</b></div>'+
    '<div><span>LEFT/RIGHT</span><b>'+(ori?'PASS':'BELUM DIVERIFIKASI')+'</b></div><div><span>YOLOPv2</span><b>'+(yolo?'PASS':'FAIL')+'</b></div>'+
    '<div><span>Semantic Detector</span><b>'+(semantic?'PASS':'FAIL')+'</b></div><div><span>Lane State</span><b>'+(laneFresh?'PASS':'FAIL')+'</b></div>'+
    '<div><span>FPS Aktual</span><b>'+v3num(p.fps,2)+'</b></div><div><span>Inference YOLOPv2</span><b>'+v3num(p.inference,1)+' ms</b></div>'+
    '<div><span>Frame Age</span><b>'+v3num(p.age,1)+' ms</b></div><div><span>Stream</span><b>'+(v3CameraReady()?'LIVE':'STALE')+'</b></div></div>'+
    '<label class="lf-ready"><input type="checkbox" id="v3OriLeft" '+(V3.orientationLeft?'checked':'')+'><span>Saya sudah menempatkan marker/objek di kiri preview dan sistem membaca LEFT.</span></label>'+
    '<label class="lf-ready"><input type="checkbox" id="v3OriRight" '+(V3.orientationRight?'checked':'')+'><span>Saya sudah menempatkan marker/objek di kanan preview dan sistem membaca RIGHT.</span></label>'+
    '<div class="lf-actions"><button class="soft-btn" id="v3Start41">Start Validasi</button><button class="primary-btn" id="v3Capture41" '+(!V3.validationStarted||!functional?'disabled':'')+'>Capture Bukti</button>'+
    '<button class="danger-btn" id="v3Reset41">Reset 4.1</button><button class="soft-btn" data-v3-export="4.1">Download 4.1</button></div>'+
    '<div class="v3-gates"><span>UI_READY = true</span><span>FUNCTION_READY = '+functional+'</span><span>DATA_VALID = '+dataValid+'</span><span>EXPORT_VALID = '+!!V3.exports['4.1']+'</span></div>'
}
function v3Render42(){
  const key=v3Key42(V3.light42,V3.sel42),w=v3Window('4.2',key),count=v3Count42(V3.light42,V3.sel42),h=v3Human(true),cands=v3HumanCandidates(),sem=obj('semantic_detections'),cam=obj('camera_health_state'),fresh=v3SemanticFresh(),running=w.status==='BERJALAN'&&w.left>0,auto=V3.auto42Timer&&V3.auto42Cond===key;
  const lightLabel=(V3_42_LIGHT.find(x=>x[0]===V3.light42)||V3_42_LIGHT[0])[1];
  return '<div class="lf-section-title"><b>4.2 Pengujian Deteksi Manusia</b><span>Human-only • SIANG vs MALAM • 1–5 m • 10 capture/kondisi • 100 sampel formal.</span></div>'+
    '<div class="lf-kpis"><div><span>Deteksi Human</span><b>'+(h?'TERDETEKSI':'TIDAK TERDETEKSI')+'</b></div><div><span>Confidence Aktual</span><b>'+(h?v3num((+h.score||0)*100,2)+' %':'--')+'</b></div>'+
    '<div><span>Pencahayaan</span><b>'+v3esc(lightLabel)+'</b></div><div><span>Mean Luma</span><b>'+v3num(cam.mean_luma??sem.camera_mean_luma,1)+'</b></div>'+
    '<div><span>Stddev Luma</span><b>'+v3num(cam.stddev_luma??sem.camera_stddev,1)+'</b></div><div><span>Kandidat Human</span><b>'+cands.length+'</b></div>'+
    '<div><span>Semantic Stream</span><b>'+(fresh?'LIVE':'STALE')+'</b></div><div><span>Semantic Inference</span><b>'+v3num(sem.inference_ms,1)+' ms</b></div></div>'+
    '<div class="lf-inline-grid"><label>Kondisi Pencahayaan<select id="v3Light42" '+(auto?'disabled':'')+'>'+V3_42_LIGHT.map(x=>'<option value="'+x[0]+'" '+(x[0]===V3.light42?'selected':'')+'>'+v3esc(x[1])+'</option>').join('')+'</select></label>'+
    '<label>Jarak Referensi<select id="v3Sel42" '+(auto?'disabled':'')+'>'+V3_42.map(d=>'<option value="'+d+'" '+(d===V3.sel42?'selected':'')+'>'+d+' meter</option>').join('')+'</select></label></div>'+
    '<div class="lf-main-progress"><span>Kondisi aktif</span><b>'+v3esc(lightLabel)+' • '+V3.sel42+' m • '+count+'/10 • '+v3esc(w.status)+' • sisa '+w.left+' s</b></div>'+
    '<div class="lf-progress">'+V3_42_LIGHT.flatMap(l=>V3_42.map(d=>{const n=v3Count42(l[0],d);return '<span class="'+(n===10?'done':'')+'">'+(l[0]==='day'?'SIANG ':'MALAM ')+d+' m <em>'+n+'/10</em></span>'})).join('')+'</div>'+
    '<div class="tuning-note">Protokol: jarak diukur dari kamera ke posisi manusia. Gunakan orang/pose yang sama. Jangan mengejar confidence agar monoton; yang dibandingkan adalah detection rate, confidence mean ± std, luma, serta error jarak.</div>'+
    '<div class="lf-actions"><button class="soft-btn" id="v3Start42" '+(auto||count>=10?'disabled':'')+'>'+(auto?'AUTO CAPTURE BERJALAN...':count>=10?'SELESAI 10/10 — reset untuk mengulang':'Mulai Otomatis 10x / 30 Detik')+'</button><button class="primary-btn" id="v3Capture42" '+(count>=10||!running||auto?'disabled':'')+'>Ambil 1 Data (Manual)</button>'+
    '<button class="warning-btn" id="v3Reset42">Reset Kondisi Aktif</button><button class="danger-btn" id="v3ResetAll42">Reset Seluruh 4.2</button>'+
    '<button class="soft-btn" data-v3-export="4.2">Download Dataset 4.2</button></div>'
}
function v3Render43(){
  const meta=V3_43.find(x=>x[0]===V3.sel43)||V3_43[0],w=v3Window('4.3',V3.sel43),count=v3Count('4.3',V3.sel43),lane=v3Lane(),fresh=channelFresh('lane_state',2.5),running=w.status==='BERJALAN'&&w.left>0,auto=V3.auto43Timer&&V3.auto43Cond===V3.sel43;
  return '<div class="lf-section-title"><b>4.3 Pengujian Lane Safety</b><span>Enam kondisi formal • LEFT/RIGHT mengikuti preview Overview • hanya sample lane baru yang diterima.</span></div>'+
    '<div class="lf-kpis"><div><span>LEFT Aktual</span><b>'+v3esc(lane.left)+'</b></div><div><span>RIGHT Aktual</span><b>'+v3esc(lane.right)+'</b></div>'+
    '<div><span>LEFT Gap</span><b>'+v3num(lane.leftGap,3)+' m</b></div><div><span>RIGHT Gap</span><b>'+v3num(lane.rightGap,3)+' m</b></div>'+
    '<div><span>Offset Kendaraan</span><b>'+v3num(lane.offset,3)+' m</b></div><div><span>Lane Stream</span><b>'+(fresh?'LIVE':'STALE')+'</b></div></div>'+
    '<label>Kondisi Pengujian<select id="v3Sel43" '+(auto?'disabled':'')+'>'+V3_43.map(x=>'<option value="'+x[0]+'" '+(x[0]===V3.sel43?'selected':'')+'>'+v3esc(x[3])+' — '+x[1]+' | '+x[2]+'</option>').join('')+'</select></label>'+
    '<div class="lf-expected">Expected: LEFT <b>'+meta[1]+'</b> • RIGHT <b>'+meta[2]+'</b> • Aktual: <b>'+v3esc(lane.left)+' | '+v3esc(lane.right)+'</b> • '+v3esc(meta[3])+'</div>'+
    '<div class="lf-main-progress"><span>Kondisi aktif</span><b>'+count+'/10 • '+v3esc(w.status)+' • sisa '+w.left+' s</b></div>'+
    '<div class="lf-progress">'+V3_43.map(x=>{const n=v3Count('4.3',x[0]);return '<span class="'+(n===10?'done':'')+'">'+v3esc(x[3])+' <em>'+n+'/10</em></span>'}).join('')+'</div>'+
    '<div class="lf-actions"><button class="soft-btn" id="v3Start43" '+(auto||count>=10?'disabled':'')+'>'+(auto?'AUTO CAPTURE BERJALAN...':count>=10?'SELESAI 10/10 — reset untuk mengulang':'Mulai Otomatis 10x / 30 Detik')+'</button><button class="primary-btn" id="v3Capture43" '+(count>=10||!running||auto?'disabled':'')+'>Ambil 1 Data (Manual)</button>'+
    '<button class="warning-btn" id="v3Reset43">Reset Kondisi Aktif</button><button class="danger-btn" id="v3ResetAll43">Reset Seluruh 4.3</button>'+
    '<button class="soft-btn" data-v3-export="4.3">Download Dataset 4.3</button></div>'
}
function v3Render44(){
  const c=v3Cond('4.4',V3.sel44),w=v3Window('4.4',V3.sel44),n=v3Count('4.4',V3.sel44),running=V3.run44===V3.sel44,perfFresh=channelFresh('perception_performance',2.5),ready=v3CameraReady()&&perfFresh;
  return '<div class="lf-section-title"><b>4.4 Pengujian Kinerja Waktu Nyata</b><span>Continuous recorder 60 detik • metric runtime aktual.</span></div>'+
    '<label>Kondisi<select id="v3Sel44">'+V3_44.map(x=>'<option value="'+x[0]+'" '+(x[0]===V3.sel44?'selected':'')+'>'+v3esc(x[1])+'</option>').join('')+'</select></label>'+
    '<div class="lf-kpis"><div><span>Elapsed</span><b>'+((+c.started_at_ms)?Math.min(60,Math.max(0,(Date.now()-+c.started_at_ms)/1000)).toFixed(1):'0.0')+' / 60 s</b></div>'+
    '<div><span>Sampel</span><b>'+n+'</b></div><div><span>Status</span><b>'+v3esc(c.status||'BELUM')+'</b></div><div><span>Runtime Source</span><b>'+(ready?'LIVE':'STALE')+'</b></div></div>'+
    '<div class="lf-progress">'+V3_44.map(x=>'<span class="'+(v3Cond('4.4',x[0]).status==='SELESAI'?'done':'')+'">'+v3esc(x[1])+' <em>'+v3Count('4.4',x[0])+' sampel</em></span>').join('')+'</div>'+
    '<div class="lf-actions"><button class="primary-btn" id="v3Start44" '+(running||!ready||c.status==='SELESAI'?'disabled':'')+'>'+(c.status==='SELESAI'?'SELESAI — reset untuk mengulang':'Start Recorder 60 Detik')+'</button><button class="warning-btn" id="v3Stop44" '+(!running?'disabled':'')+'>STOP</button>'+
    '<button class="warning-btn" id="v3Reset44">Reset Kondisi Aktif</button><button class="danger-btn" id="v3ResetAll44">Reset Seluruh 4.4</button>'+
    '<button class="soft-btn" data-v3-export="4.4">Download Dataset 4.4</button></div>'
}
function v3Draw45Map(){
  const c=$('v3Map45');if(!c)return;
  const rect=c.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2),W=Math.max(1,Math.round(rect.width*dpr)),H=Math.max(1,Math.round(rect.height*dpr));
  if(c.width!==W||c.height!==H){c.width=W;c.height=H}
  const ctx=c.getContext('2d'),m=obj('map_meta'),lm=obj('local_costmap_meta'),mapPose=navigationRobotPose?.(),lp=obj('ekf_local'),tf=obj('map_odom_tf');
  ctx.setTransform(1,0,0,1,0,0);ctx.clearRect(0,0,W,H);ctx.fillStyle='#050b0d';ctx.fillRect(0,0,W,H);
  const haveTf=Number.isFinite(+tf.x)&&Number.isFinite(+tf.y)&&Number.isFinite(+tf.yaw);
  const haveMap=!!(m.width&&m.height&&m.resolution&&Number.isFinite(+mapPose?.x)&&Number.isFinite(+mapPose?.y));
  const localMode=!haveTf||!haveMap;
  const pose=localMode?{x:Number.isFinite(+lp.x)?+lp.x:0,y:Number.isFinite(+lp.y)?+lp.y:0,yaw:Number.isFinite(+lp.yaw)?+lp.yaw:0}:mapPose;
  if(localMode&&!lm.width){
    ctx.fillStyle='#9fb1b2';ctx.font=`${12*dpr}px sans-serif`;ctx.fillText('Menunggu local costmap...',14*dpr,26*dpr);return
  }
  const spanM=14,ppm=Math.min(W/spanM,H/spanM),cx=W*.5,cy=H*.62;
  const trans=(x,y)=>[cx+(x-(+pose.x))*ppm,cy-(y-(+pose.y))*ppm];
  const g={m:{resolution:localMode?(+lm.resolution||.1):(+m.resolution||.1)},scale:ppm*(localMode?(+lm.resolution||.1):(+m.resolution||.1)),trans},info={dpr,w:W,h:H};
  if(!localMode&&mapImage){
    const p0=trans(+m.origin_x,+m.origin_y+(+m.height)*(+m.resolution));
    ctx.save();ctx.globalAlpha=.82;ctx.imageSmoothingEnabled=false;
    ctx.drawImage(mapImage,p0[0],p0[1],(+m.width)*(+m.resolution)*ppm,(+m.height)*(+m.resolution)*ppm);ctx.restore()
  }
  if(localMode&&localCostmapImage&&lm.width&&lm.height&&lm.resolution){
    const x0=+lm.origin_x,y0=+lm.origin_y,wM=+lm.width*+lm.resolution,hM=+lm.height*+lm.resolution;
    const bl=trans(x0,y0),tl=trans(x0,y0+hM),tr=trans(x0+wM,y0+hM);
    ctx.save();ctx.globalAlpha=.62;
    ctx.setTransform((tr[0]-tl[0])/localCostmapImage.width,(tr[1]-tl[1])/localCostmapImage.width,(bl[0]-tl[0])/localCostmapImage.height,(bl[1]-tl[1])/localCostmapImage.height,tl[0],tl[1]);
    ctx.imageSmoothingEnabled=false;ctx.drawImage(localCostmapImage,0,0);ctx.restore()
  }else{
    drawCostmapLayer(ctx,globalCostmapImage,obj('global_costmap_meta'),g,.22);
    drawCostmapLayer(ctx,localCostmapImage,lm,g,.62);
  }
  const css=getComputedStyle(document.body);
  const baseToView=(x,y)=>{const c0=Math.cos(+pose.yaw||0),s0=Math.sin(+pose.yaw||0);return{x:+pose.x+c0*x-s0*y,y:+pose.y+s0*x+c0*y}};
  const toView=(frame,x,y)=>{const f=String(frame||'');if(localMode){if(!f||f==='odom')return{x,y};if(f==='base_footprint'||f==='base_link')return baseToView(x,y);return null}if(f==='base_footprint'||f==='base_link')return baseToView(x,y);return framePointToMap(f||'map',x,y)};
  const path=(key,col,w,dash=[])=>{const q=obj(key),pts=q.points||[],frame=q.frame_id||(key==='local_path'?'odom':'map');if(pts.length<2)return;ctx.save();ctx.strokeStyle=col;ctx.lineWidth=w*dpr;ctx.lineJoin='round';if(dash.length)ctx.setLineDash(dash.map(v=>v*dpr));ctx.beginPath();let begun=false;for(const pt of pts){const vp=toView(frame,+pt[0],+pt[1]);if(!vp)continue;const [x,y]=trans(vp.x,vp.y);begun?ctx.lineTo(x,y):(ctx.moveTo(x,y),begun=true)}if(begun)ctx.stroke();ctx.restore()};
  path('previous_nav_path','rgba(160,178,188,.72)',1.8,[7,5]);
  path('nav_path',css.getPropertyValue('--blue')||'#4aa8ff',2.4);
  path('local_path',css.getPropertyValue('--cyan')||'#54d6e8',2.5);
  const planningCloud=obj('planning_relevant_points'),planningPts=planningCloud.points||[],planningFrame=planningCloud.frame_id||'base_footprint';
  if(planningPts.length){ctx.save();ctx.fillStyle='#d86cff';ctx.strokeStyle='#f2b8ff';ctx.lineWidth=1.5*dpr;for(const pt of planningPts){const vp=toView(planningFrame,+pt[0],+pt[1]);if(!vp)continue;const [px,py]=trans(vp.x,vp.y),r=4*dpr;ctx.fillRect(px-r,py-r,2*r,2*r);ctx.strokeRect(px-r,py-r,2*r,2*r)}ctx.restore()}
  for(const tr of (obj('mppi_trajectories').trajectories||[])){const pts=tr.points||[],frame=tr.frame_id||'odom';if(pts.length<2)continue;ctx.save();ctx.strokeStyle=tr.optimal?'#ffc86b':'rgba(84,214,232,.22)';ctx.lineWidth=(tr.optimal?3:1)*dpr;ctx.beginPath();let begun=false;for(const pt of pts){const vp=toView(frame,+pt[0],+pt[1]);if(!vp)continue;const [x,y]=trans(vp.x,vp.y);begun?ctx.lineTo(x,y):(ctx.moveTo(x,y),begun=true)}if(begun)ctx.stroke();ctx.restore()}
  drawRobotFootprint(ctx,g,info,pose,css);
  const [rx,ry]=trans(+pose.x,+pose.y);drawPoseArrow(ctx,rx,ry,+pose.yaw||0,'#48e0a4',info,6);
  const pyaw=+pose.yaw||0,pc=Math.cos(pyaw),ps=Math.sin(pyaw);
  if(v3SemanticFresh()){
    for(const det of (obj('semantic_detections').detections||[])){
      if(det.nav2_eligible===false)continue;
      const f=+det.nav2_forward_m,l=+det.raw_left_m;
      if(!Number.isFinite(f)||!Number.isFinite(l)||f<=0)continue;
      const mx=(+pose.x)+pc*f-ps*l,my=(+pose.y)+ps*f+pc*l,[ox,oy]=trans(mx,my);
      const cname=String(det.class_name||'obstacle').toLowerCase();
      const diameterM=cname==='person'?.50:cname==='motorcycle'?1.0:1.6;
      const radiusPx=Math.max(5,diameterM*ppm*.5);
      ctx.save();ctx.beginPath();ctx.arc(ox,oy,radiusPx,0,Math.PI*2);
      ctx.fillStyle='rgba(255,62,78,.34)';ctx.strokeStyle='#ff3e4e';ctx.lineWidth=2*dpr;ctx.fill();ctx.stroke();
      ctx.beginPath();ctx.moveTo(ox-radiusPx*.55,oy);ctx.lineTo(ox+radiusPx*.55,oy);
      ctx.moveTo(ox,oy-radiusPx*.55);ctx.lineTo(ox,oy+radiusPx*.55);ctx.stroke();
      ctx.fillStyle='#ffd1d5';ctx.font=`700 ${10*dpr}px ui-monospace`;
      ctx.fillText((cname==='person'?'HUMAN':cname.toUpperCase())+' '+f.toFixed(2)+'m',ox-radiusPx,oy-radiusPx-5*dpr);ctx.restore()
    }
  }
  if(!localMode){const goal=obj('goal_pose');if(Number.isFinite(+goal.x)&&Number.isFinite(+goal.y)){const [gx,gy]=trans(+goal.x,+goal.y);drawPoseArrow(ctx,gx,gy,+goal.yaw||0,'#ff7b8a',info,5)}}
  ctx.fillStyle='rgba(4,10,12,.82)';ctx.fillRect(8*dpr,8*dpr,Math.min(W-16*dpr,390*dpr),46*dpr);
  ctx.fillStyle='#e7f1f0';ctx.font=`700 ${11*dpr}px ui-monospace`;ctx.fillText(localMode?'4.5 LOCAL ODOM • COSTMAP • MPPI':'4.5 MAP • LOCAL COSTMAP • MPPI',16*dpr,26*dpr);
  ctx.font=`${10*dpr}px ui-monospace`;ctx.fillText(`planning ${+obj('planning_relevant_points').count||0} • cells ${+lm.nonzero_cells||0} • MPPI ${+obj('mppi_trajectories').count||0} • replan ${+obj('nav_replan').count||0}`,16*dpr,43*dpr)
}
function v3DrawContained(ctx,src,x,y,w,h){
  const sw=src?.naturalWidth||src?.width||0,sh=src?.naturalHeight||src?.height||0;if(!sw||!sh)return false;
  const sc=Math.min(w/sw,h/sh),dw=sw*sc,dh=sh*sc;ctx.drawImage(src,x+(w-dw)/2,y+(h-dh)/2,dw,dh);return true
}
function v3Make45VisualBundle(meta={}){
  if(V3.tab45!=='obstacle')return null;
  const img=$('v3CameraImage'),overlay=$('v3Overlay'),stage=$('v3CameraHost'),map=$('v3Map45');
  if(!img?.naturalWidth||!img?.naturalHeight||!map)return null;
  v3DrawOverlay(false);v3Draw45Map();
  const sr=stage?.getBoundingClientRect(),cw=Math.max(1,overlay?.width||Math.round(sr?.width||640)),ch=Math.max(1,overlay?.height||Math.round(sr?.height||360));
  const cam=document.createElement('canvas');cam.width=cw;cam.height=ch;const cx=cam.getContext('2d');
  cx.fillStyle='#050b0d';cx.fillRect(0,0,cw,ch);
  const sc=Math.min(cw/img.naturalWidth,ch/img.naturalHeight),dw=img.naturalWidth*sc,dh=img.naturalHeight*sc;
  cx.drawImage(img,(cw-dw)/2,(ch-dh)/2,dw,dh);if(overlay?.width&&overlay?.height)cx.drawImage(overlay,0,0,cw,ch);

  const out=document.createElement('canvas');out.width=1440;out.height=810;const x=out.getContext('2d');
  x.fillStyle='#071013';x.fillRect(0,0,out.width,out.height);x.strokeStyle='#34515a';x.lineWidth=2;
  x.font='800 24px sans-serif';x.fillStyle='#e9f4f3';
  const mode=String(meta.mode||'EVIDENCE'),condition=String(meta.condition||'4.5A');
  x.fillText('BAB 4.5A • OBSTACLE PERCEPTION × NAV2 EVIDENCE',36,42);
  x.font='700 16px ui-monospace';x.fillStyle='#9fd6ca';x.fillText(mode+' • '+condition+' • '+new Date().toLocaleString('id-ID'),36,68);
  const lx=36,rx=738,py=100,pw=666,ph=500;x.strokeRect(lx,py,pw,ph);x.strokeRect(rx,py,pw,ph);
  x.font='800 17px sans-serif';x.fillStyle='#e9f4f3';x.fillText('CAMERA PREVIEW + DETECTION',lx+14,py+26);x.fillText('MAP / COSTMAP / MPPI EVIDENCE',rx+14,py+26);
  v3DrawContained(x,cam,lx+12,py+42,pw-24,ph-54);v3DrawContained(x,map,rx+12,py+42,pw-24,ph-54);
  const human=v3Human45(),safety=obj('trajectory_safety_state'),lm=obj('local_costmap_meta'),mppi=obj('mppi_trajectories');
  const planning=+obj('planning_relevant_points').count||0,cells=+lm.nonzero_cells||0,mppiCount=+mppi.count||0;
  const dist=human&&Number.isFinite(+(human.stable_forward_m??human.nav2_forward_m))?(+(human.stable_forward_m??human.nav2_forward_m)).toFixed(2):'--';
  const decision=String(safety.decision||meta.decision||'--'),avoid=String(meta.avoid_direction||safety.avoid_direction||'--');
  x.fillStyle='#0d1b1f';x.fillRect(36,624,1368,150);x.strokeStyle='#34515a';x.strokeRect(36,624,1368,150);
  x.fillStyle='#e9f4f3';x.font='700 16px ui-monospace';
  x.fillText('Human '+dist+' m  |  Planning '+planning+' pts  |  Costmap '+cells+' cells  |  MPPI '+mppiCount,54,654);
  x.fillText('Decision '+decision+'  |  Avoid '+avoid+'  |  Recovery '+(meta.recovery_seen===true?'YES':'NO')+'  |  Result '+String(meta.result||'LIVE'),54,684);
  x.font='14px ui-monospace';x.fillStyle='#a9bec0';
  x.fillText('Sumber kontrol: PointCloud2 → planning_relevant_points → local costmap → MPPI. Camera dan map dibekukan pada capture yang sama.',54,718);
  x.fillText('capture_id '+String(meta.capture_id??'--')+'  •  visual timestamp '+Date.now()+' ms',54,746);
  return {map_data_url:map.toDataURL('image/png'),combined_data_url:out.toDataURL('image/png')}
}
async function v3Save45VisualEvidence(condition,row,bundle){
  if(!bundle||!row||!String(condition).startsWith('obstacle_'))return null;
  const j=await v3Post('/api/perception/bab4/visual-evidence',{session_id:V3.state?.session_id||'',condition,
    mode:String(row.mode||''),capture_id:+row.capture_id||0,map_data_url:bundle.map_data_url,combined_data_url:bundle.combined_data_url});
  if(j?.state)V3.state=j.state;return j
}
async function v3SaveLatestArmed45Visual(condition,bundle){
  if(!bundle||!String(condition).startsWith('obstacle_'))return null;
  for(let i=0;i<8;i++){await v3FetchState();const rows=v3Cond('4.5',condition).rows||[];
    const row=[...rows].reverse().find(r=>r.mode==='ARMED_VEHICLE_TEST');
    if(row)return v3Save45VisualEvidence(condition,row,bundle);
    await new Promise(r=>setTimeout(r,250));
  }
  throw Error('Row ARMED sudah selesai tetapi belum tersedia untuk visual evidence')
}
function v3Render45(){
  const obstacleMode=V3.tab45==='obstacle';
  const obstacleTrial=Math.max(1,Math.min(3,+V3.obs45||1));
  const laneTrial=Math.max(1,Math.min(3,+V3.lane45||1));
  const cond=obstacleMode?('obstacle_trial_'+obstacleTrial):('lane_trial_'+laneTrial);
  const condState=v3Cond('4.5',cond),rows45=Array.isArray(condState.rows)?condState.rows:[],lastDry=[...rows45].reverse().find(r=>r.mode==='DRY_RUN');
  const dryDone=!!condState.dry_run_complete||!!lastDry;
  const dryPass=condState.dry_run_pass===true||lastDry?.integration_pass===true||lastDry?.result==='DRY_RUN_PASS';
  const armedDone=!!condState.armed_complete||rows45.some(r=>r.mode==='ARMED_VEHICLE_TEST'&&String(r.result||'').toLowerCase()==='complete'&&r.final_vehicle_state==='STOPPED'&&r.integration_pass===true);

  const lane=v3Lane(),humanDetected=obstacleMode?v3Human45Detected():null,human=obstacleMode?v3Human45():v3Human(false),safety=obj('trajectory_safety_state');
  const sourceFresh=obstacleMode?v3SemanticFresh():channelFresh('lane_state',1.5);
  const safetyFresh=channelFresh('trajectory_safety_state',1.5),perfFresh=channelFresh('perception_performance',2.5);
  const decision=String(safety.decision||'--');
  const actualState=lane.left+'|'+lane.right;
  const laneGreen=lane.left==='GREEN'&&lane.right==='GREEN';
  const laneRedLeft=lane.left==='RED'&&lane.right==='GREEN'&&lane.leftLatched;
  const laneRedRight=lane.left==='GREEN'&&lane.right==='RED'&&lane.rightLatched;
  const laneConditionMatch=laneRedLeft||laneRedRight;
  const conditionMatch=obstacleMode?!!human:laneConditionMatch;

  const dryExpected=obstacleMode?
    'Human terdeteksi → object/planning cloud → local costmap → MPPI merencanakan penghindaran menuju goal 5 m; kendaraan tetap STOP':
    'Validasi kondisi awal RED pada salah satu sisi + lane/drivable valid; kendaraan tetap STOP';
  const armedExpected=obstacleMode?
    'Goal 5 m → Nav2/MPPI menghindari human → recovery → goal tercapai → STOP':
    'Baca sisi RED aktual → recenter otomatis → GREEN/GREEN → kembalikan heading ke arah awal → pertahankan heading awal → lanjut sampai 5 m → STOP';

  const exportReady45=v3ExportReady45();
  const finalSupported=!!safety.final_trial_supported&&!safety.commissioning_bypass,metricSafety=!!safety.metric_obstacle_safety;
  const semanticList=obstacleMode&&sourceFresh?(obj('semantic_detections').detections||[]):[];
  const trackedCount=semanticList.filter(x=>(+x.class_id===0||['person','human'].includes(String(x.class_name||'').toLowerCase()))&&x.nav2_eligible!==false&&((+x.score||0)>=0.20||x.safety_human_valid===true)).length;
  const objectFresh=channelFresh('object_points',1.2),pathFresh=channelFresh('path_relevant_points',1.2),planningFresh=channelFresh('planning_relevant_points',1.2);
  const objCount=objectFresh?(+obj('object_points').count||0):0,pathCount=pathFresh?(+obj('path_relevant_points').count||0):0,planningCount=planningFresh?(+obj('planning_relevant_points').count||0):0;
  const sourceHz=channelRate(obstacleMode?'semantic_detections':'lane_state');
  const costmapHz=(typeof costmapDisplayRate==='function'?costmapDisplayRate('local'):channelRate('local_costmap_meta'));
  const costmapFresh=channelFresh('local_costmap_meta',0.75),mppiCount=+obj('mppi_trajectories').count||0,mppiFresh=channelFresh('mppi_trajectories',2.0)&&mppiCount>0;
  const displayHuman=obstacleMode?(human||humanDetected):human;
  const hd=human?(Number.isFinite(+human.stable_forward_m)?+human.stable_forward_m:(Number.isFinite(+human.nav2_forward_m)?+human.nav2_forward_m:(Number.isFinite(+human.calibrated_forward_m)?+human.calibrated_forward_m:+human.forward_m))):NaN;
  const hdDisplay=displayHuman?(Number.isFinite(+displayHuman.stable_forward_m)?+displayHuman.stable_forward_m:(Number.isFinite(+displayHuman.nav2_forward_m)?+displayHuman.nav2_forward_m:(Number.isFinite(+displayHuman.calibrated_forward_m)?+displayHuman.calibrated_forward_m:+displayHuman.forward_m))):NaN;
  const metricValid=!!human&&Number.isFinite(hd)&&hd>0;
  const nav2RejectReason=obstacleMode&&humanDetected&&!human?String(humanDetected.nav2_rejection_reason||'nav2_not_eligible').replace(/_/g,' ').toUpperCase():'';

  const navReady=!!raw('system.nav2_ready'),planningLocReady=!!raw('system.planning_localization_ready'),autonomyReady=!!raw('system.autonomy_ready');
  const autonomyMotionAllowed=!!raw('system.autonomy_motion_allowed'),escFeedback=!!raw('connected.esc_feedback'),escReady=!!raw('connected.esc_ready');
  const cameraReady=raw('connected.camera')===true&&raw('camera_healthy')===true;
  const estopKnown=raw('system.estop')!==undefined&&raw('system.estop')!==null,estopActive=raw('system.estop')===true,maintenanceActive=raw('vesc_maintenance_active')===true;
  const escOdom=obj('esc_odom'),escFresh=channelFresh('esc_drive_actual',1.0)&&channelFresh('esc_steer_actual',1.0)&&channelFresh('esc_odom',1.0)&&
    raw('esc_drive_actual')!=null&&Number.isFinite(+raw('esc_drive_actual'))&&raw('esc_steer_actual')!=null&&Number.isFinite(+raw('esc_steer_actual'))&&
    ['x','y','yaw'].every(k=>escOdom[k]!=null&&Number.isFinite(+escOdom[k]));
  const goalState=String(obj('goal_state').state||'').toUpperCase(),goalActive=['QUEUED','SENDING','ACTIVE','RUNNING','EXECUTING'].includes(goalState);
  const costmapMeta=obj('local_costmap_meta'),costmapCells=+costmapMeta.nonzero_cells||0;
  const alignmentValid=costmapMeta.planning_alignment_valid===true,alignmentPass=costmapMeta.planning_alignment_pass===true;
  const alignmentMax=Number.isFinite(+costmapMeta.planning_alignment_max_m)?+costmapMeta.planning_alignment_max_m:NaN;
  const alignmentThreshold=Number.isFinite(+costmapMeta.planning_alignment_threshold_m)?+costmapMeta.planning_alignment_threshold_m:.25;
  const obstacleToCostmap=!obstacleMode?true:(objCount>0&&planningCount>0&&costmapFresh&&costmapCells>0&&alignmentPass);
  const muxSource=String(obj('nav_cmd_mux').raw||'').toUpperCase(),teleopActive=muxSource.startsWith('TELEOP');
  const autonomyOwnsActuator=muxSource==='AUTONOMY'||muxSource==='AUTONOMY_STALE_STOP';
  const laneAuthorityOk=obstacleMode||(!autonomyMotionAllowed&&!autonomyOwnsActuator&&!goalActive);
  const vehicleReady=escFeedback&&escReady&&escFresh&&!teleopActive&&laneAuthorityOk&&!estopActive&&!maintenanceActive;
  const obstacleRuntimeOk=obstacleMode?(autonomyReady&&metricSafety&&navReady&&planningLocReady):(autonomyReady&&navReady&&planningLocReady);
  const laneEmergency=!!raw('perception_emergency')||safety.emergency===true;
  const laneObstacleStreamFresh=safety.obstacle_stream_fresh===true;
  const laneMotionSafetyOk=obstacleMode||(!laneEmergency&&lane.laneValid&&!lane.invalid);

  // DRY RUN 4.5A is planning-only: let the operator start the preview before
  // the human enters the path. The preview guard keeps the vehicle STOP while
  // evidence is collected. Human/metric/cloud/costmap/MPPI are still mandatory
  // for DRY RUN PASS and remain mandatory for the later ARMED admission.
  const dryPrereq=!cameraReady?'CAMERA BELUM READY':!perfFresh?'PERCEPTION PERFORMANCE STALE':!sourceFresh?'SOURCE STALE':!safetyFresh?'SAFETY STATE STALE':!finalSupported?'FINAL TRIAL WATCHDOG OFF':
    (!obstacleMode&&!laneConditionMatch?('KONDISI LANE BELUM SIAP RECENTER • aktual '+actualState+' • wajib RED di salah satu sisi'):
    (obstacleMode&&!metricSafety?'METRIC SAFETY OFF':
    (obstacleMode&&!planningLocReady?'SET INITIAL POSE / MAP ANCHOR DULU':
    (obstacleMode&&!navReady?'NAV2 LIFECYCLE BELUM READY':'READY'))));
  const dryReady=dryPrereq==='READY';
  const armedReady=dryDone&&dryPass&&cameraReady&&perfFresh&&sourceFresh&&safetyFresh&&finalSupported&&estopKnown&&!estopActive&&!maintenanceActive&&escFresh&&
    obstacleRuntimeOk&&laneMotionSafetyOk&&conditionMatch&&vehicleReady&&!armedDone&&!V3.preview45Busy;
  const gate=armedDone?'TRIAL SUDAH SELESAI':!dryDone?('DRY RUN: '+dryPrereq):!dryPass?'DRY RUN FAIL — RESET & ULANGI':!cameraReady?'CAMERA BELUM READY':
    !perfFresh?'PERCEPTION PERFORMANCE STALE':teleopActive?'TELEOP AKTIF — LEPASKAN JOYSTICK':!estopKnown?'STATUS E-STOP BELUM TERSEDIA':
    estopActive?'E-STOP AKTIF':maintenanceActive?'VESC MAINTENANCE AKTIF':!sourceFresh?'SOURCE STALE':!safetyFresh?'SAFETY STATE STALE':
    !finalSupported?'FINAL TRIAL WATCHDOG OFF':!obstacleMode&&!laneAuthorityOk?(goalActive?'GOAL NAV2 MASIH AKTIF — CANCEL DULU':'AUTONOMY MASIH MEMILIKI AKTUATOR'):
    !obstacleMode&&!laneMotionSafetyOk?'LANE SAFETY BELUM READY':!planningLocReady?'PLANNING LOCALIZATION BELUM READY':
    !navReady?'NAV2 LIFECYCLE BELUM READY':!autonomyReady?'AUTONOMY START GATE BELUM READY':
    !conditionMatch?'KONDISI AKTUAL BELUM SESUAI':!escFeedback?'ESC FEEDBACK OFFLINE':!escReady?'ESC BELUM READY':!escFresh?'ESC ACTUAL / ODOM STALE':'READY';

  const dryLabel=V3.preview45Busy?(obstacleMode?'DRY RUN NAV2 PREVIEW...':'DRY RUN LANE VALIDATION...'):armedDone?'TRIAL SELESAI':dryDone?(dryPass?'DRY RUN PASS':'DRY RUN FAIL — reset untuk ulang'):(dryReady?'Jalankan DRY RUN':dryPrereq);
  const evidenceCard=obstacleMode?
    '<div class="v3-nav45-card"><div class="v3-nav45-head"><b>RVIZ / NAV2 EVIDENCE</b><span>Human → PointCloud → local costmap → MPPI → goal 5 m</span></div><canvas id="v3Map45" width="720" height="360"></canvas></div>':
    '<div class="v3-nav45-card"><div class="v3-nav45-head"><b>LANE SAFETY CONTROL PATH</b><span>lane_state → WEB_TRIAL → cmd_vel_router → ESC/Ackermann</span></div><div class="lf-message"><b>GREEN</b> = lurus • <b>YELLOW</b> = warning-only tanpa steering • <b>RED</b> = recenter sesuai sisi aktual sampai GREEN.</div></div>';

  const trialSummary=obstacleMode?
    [1,2,3].map(i=>{const c=v3Cond('4.5','obstacle_trial_'+i);return '<div><span>Obstacle Trial '+i+'</span><b>'+(c.armed_complete?'PASS':c.dry_run_pass?'DRY PASS':c.status||'BELUM')+'</b></div>'}).join(''):
    [1,2,3].map(i=>{const c=v3Cond('4.5','lane_trial_'+i);return '<div><span>Lane Trial '+i+'</span><b>'+(c.armed_complete?'PASS':c.dry_run_pass?'DRY PASS':c.status||'BELUM')+'</b></div>'}).join('');

  return '<div class="lf-section-title"><b>4.5 Pengujian Respons Kendaraan — 3 Pengulangan</b><span>'+(obstacleMode?
    '4.5A: tiga trial identik, goal 5 m, human ditempatkan pada lintasan; Nav2/MPPI wajib menghindar dan mencapai goal.':
    '4.5B: tiga trial identik, recenter dari RED sampai GREEN/GREEN, pulihkan heading awal, lalu handoff ke Nav2/MPPI untuk mencapai goal 5 m dan STOP.')+'</span></div>'+
    evidenceCard+
    '<div class="lf-kpis">'+trialSummary+
    '<div><span>Source</span><b>'+(sourceFresh?'LIVE':'STALE')+'</b></div><div><span>Safety Runtime</span><b>'+(safetyFresh?'LIVE':'STALE')+'</b></div>'+
    '<div><span>Planning Localization</span><b>'+(planningLocReady?'READY':'WAIT')+'</b></div><div><span>Nav2 Lifecycle</span><b>'+(navReady?'READY':'WAIT')+'</b></div>'+
    '<div><span>'+(obstacleMode?'Autonomy Start Gate':'Vehicle Authority')+'</span><b>'+(obstacleMode?(autonomyReady?'READY':'BLOCKED'):(laneAuthorityOk?'WEB_TRIAL READY':'BLOCKED'))+'</b></div>'+
    '<div><span>Metric Safety</span><b>'+(obstacleMode?(metricSafety?'ACTIVE':'OFF'):'N/A')+'</b></div>'+
    '<div><span>Tracked Human</span><b>'+(obstacleMode?trackedCount:'N/A')+'</b></div><div><span>Object / Planning</span><b>'+(obstacleMode?(objCount+' / '+planningCount):'N/A')+'</b></div>'+
    '<div><span>Local Costmap</span><b>'+(obstacleMode?(costmapFresh?(costmapCells+' cells'):'STALE'):'N/A')+'</b></div><div><span>Costmap Sync</span><b>'+(obstacleMode?(alignmentPass?'PASS':alignmentValid?'MISALIGNED':'WAIT'):'N/A')+'</b></div>'+
    '<div><span>MPPI</span><b>'+(!obstacleMode?'N/A':mppiFresh?(mppiCount+' ACTIVE'):'WAIT')+'</b></div><div><span>Decision</span><b>'+v3esc(decision)+'</b></div>'+
    '<div><span>DRY RUN</span><b>'+(dryPass?'PASS':dryDone?'FAIL':'BELUM')+'</b></div><div><span>ARMED Gate</span><b>'+v3esc(gate)+'</b></div></div>'+
    (obstacleMode?'<div class="lf-result-grid"><div><span>Human</span><b>'+(humanDetected?'TERDETEKSI':'TIDAK')+'</b></div><div><span>Confidence</span><b>'+(humanDetected?v3num((+humanDetected.score||0)*100,1)+' %':'--')+'</b></div>'+
      '<div><span>Valid Nav2</span><b>'+(human?'VALID':humanDetected?'WAIT':'--')+'</b></div><div><span>Jarak Aktual</span><b>'+(Number.isFinite(hdDisplay)?v3num(hdDisplay,2)+' m':'--')+'</b></div>'+
      '<div><span>Goal Trial</span><b>5.00 m</b></div><div><span>Obstacle → Costmap</span><b>'+(obstacleToCostmap?'PASS':'WAIT')+'</b></div></div>':
      '<div class="lf-result-grid"><div><span>LEFT</span><b>'+v3esc(lane.left)+'</b></div><div><span>RIGHT</span><b>'+v3esc(lane.right)+'</b></div>'+
      '<div><span>Target Awal</span><b>RED salah satu sisi</b></div><div><span>Siap Recenter</span><b>'+(conditionMatch?'YES':'NO')+'</b></div>'+
      '<div><span>Recommendation</span><b>'+v3esc(lane.action||'--')+'</b></div><div><span>Target Akhir</span><b>GREEN | GREEN • heading awal • 5.00 m</b></div></div>')+
    '<div class="lf-subtabs"><button id="v3TabObs" '+(V3.preview45Busy?'disabled':'')+' class="'+(obstacleMode?'active':'')+'">4.5A Obstacle Human</button>'+
    '<button id="v3TabLane" '+(V3.preview45Busy?'disabled':'')+' class="'+(!obstacleMode?'active':'')+'">4.5B Lane Safety</button></div>'+
    (obstacleMode?'<label>Pengulangan<select id="v3Obs45" '+(V3.preview45Busy?'disabled':'')+'>'+
      [1,2,3].map(i=>'<option value="'+i+'" '+(String(i)===String(obstacleTrial)?'selected':'')+'>Trial '+i+' — Goal 5 m</option>').join('')+'</select></label>':
      '<label>Pengulangan<select id="v3Lane45" '+(V3.preview45Busy?'disabled':'')+'>'+
      [1,2,3].map(i=>'<option value="'+i+'" '+(String(i)===String(laneTrial)?'selected':'')+'>Trial '+i+' — Recenter 5 m</option>').join('')+'</select></label>')+
    '<div class="lf-expected"><b>Expected DRY RUN:</b> '+v3esc(dryExpected)+'<br><b>Expected ARMED:</b> '+v3esc(armedExpected)+
      (obstacleMode?'<br><span>Tidak ada syarat jarak referensi. Human cukup berada pada jalur menuju goal 5 m. Jarak aktual tetap direkam sebagai data.':
       '<br><span>Tidak memilih warna. Sistem membaca kondisi aktual saat START; trial hanya dibuka jika salah satu sisi RED dan sisi lain GREEN.</span>')+'</div>'+
    '<div class="lf-actions"><button class="primary-btn" id="v3Dry45" '+((dryDone||armedDone||V3.preview45Busy||!dryReady)?'disabled':'')+'>'+dryLabel+'</button>'+
    '<button class="warning-btn" id="v3ResetCurrent45" '+(V3.preview45Busy?'disabled':'')+'>Reset Trial Ini</button>'+
    '<button class="danger-btn" id="v3ResetGroup45" '+(V3.preview45Busy?'disabled':'')+'>Reset 3 Trial '+(obstacleMode?'Obstacle':'Lane')+'</button>'+
    '<button class="soft-btn" data-v3-export="4.5" '+(!exportReady45?'disabled title="Lengkapi 3 obstacle trial + 3 lane trial"':'')+'>Download ZIP 4.5</button></div>'+
    '<label class="lf-ready"><input type="checkbox" id="v3ArmToggle" '+(V3.armed?'checked':'')+' '+(!armedReady||armedDone?'disabled':'')+'><span>ARMED VEHICLE TEST — hanya setelah DRY RUN PASS. '+
      (obstacleMode?'Nav2/MPPI memiliki steering; GUI hanya menetapkan goal 5 m.':'WEB_TRIAL recenter dari RED, pulihkan heading awal, lalu kontrol diserahkan ke Nav2/MPPI untuk mencapai goal 5 m.')+'</span></label>'+
    '<div class="lf-actions"><button class="danger-btn" id="v3Armed45" '+(!V3.armed||!armedReady?'disabled':'')+'>Mulai Trial</button>'+
    '<button class="warning-btn" id="v3Abort45" '+(!V3.armedToken?'disabled':'')+'>ABORT / STOP</button></div>'+
    '<div class="lf-message">Condition data: <b>'+v3esc(cond)+'</b> • '+(obstacleMode?
      'PASS = human terdeteksi → cloud/costmap → MPPI avoidance → recovery → goal 5 m tercapai → STOP.':
      'PASS = RED → steering recenter → GREEN/GREEN → heading awal pulih → handoff AUTONOMY → Nav2/MPPI mencapai goal 5 m → STOP.')+'</div>'
}
function v3Render(liveOnly=false){
  if(!v3IsActive()){document.body.removeAttribute('data-per-locked');const p=$('perLockedBab4Panel');if(p)p.hidden=true;v3RestoreCamera();return}
  v3PatchCatalog();document.body.setAttribute('data-per-locked','1');
  const main=$('page-experiments')?.querySelector('.workbench-main');if(!main)return;
  let p=$('perLockedBab4Panel');if(!p){p=document.createElement('section');p.id='perLockedBab4Panel';p.className='per-locked-bab4';main.prepend(p)}
  const id=selectedExperiment?.id;let body=id==='F4.1'?v3Render41():id==='F4.2'?v3Render42():id==='F4.3'?v3Render43():id==='F4.4'?v3Render44():v3Render45();
  // Keep the live camera DOM node across Bab-4 re-renders. Replacing the <img>
  // every 500 ms caused in-flight camera fetches to finish against detached nodes,
  // producing intermittent black/vanishing preview even while the backend stayed LIVE.
  const keepCamera=p.querySelector('#v3CameraHost');
  p.hidden=false;p.innerHTML=v3Shell(body);
  if(keepCamera){const freshCamera=p.querySelector('#v3CameraHost');if(freshCamera)freshCamera.replaceWith(keepCamera)}
  v3AttachCamera();v3Bind();v3RenderLower();v3DrawOverlay();if(id==='F4.5'){if(!liveOnly){if(!mapImage)loadMapImage();loadCostmapImage('global');loadCostmapImage('local')}setTimeout(v3Draw45Map,40)}if(!V3.state)v3FetchState();
}
function v3Table(head,rows){
  return '<div class="lf-table-wrap"><table class="data-table"><thead><tr>'+head.map(x=>'<th>'+v3esc(x)+'</th>').join('')+'</tr></thead><tbody>'+
    (rows.length?rows.map(r=>'<tr>'+r.map(x=>'<td>'+v3esc(x)+'</td>').join('')+'</tr>').join(''):'<tr><td colspan="'+head.length+'">Belum ada data.</td></tr>')+'</tbody></table></div>'
}
function v3Mean(a,k,n=3){const z=a.map(r=>+r[k]).filter(Number.isFinite);return z.length?v3num(z.reduce((p,q)=>p+q,0)/z.length,n):'--'}
function v3RenderLower(){
  const box=$('v3Table');if(!box)return;const id=selectedExperiment?.id;
  if(id==='F4.1'){const a=v3Rows('4.1','validation');box.innerHTML=v3Table(['Timestamp','Stream','Target FPS','Actual FPS','Error FPS','Inference [ms]','Frame Age [ms]','LEFT','RIGHT'],
    a.map(r=>[r.timestamp,r.stream_status,v3num(r.fps_reference_hz,2),v3num(r.fps_actual_hz??r.fps,2),v3num(r.fps_error_hz,2),v3num(r.inference_ms,1),v3num(r.frame_age_ms,1),r.orientation_left_ok?'PASS':'FAIL',r.orientation_right_ok?'PASS':'FAIL']))}
  if(id==='F4.2'){box.innerHTML=v3Table(['Cahaya','Reference [m]','N','Detected','Detection Rate','Confidence mean ± std','Mean Luma','Actual/Estimate mean [m]','Signed Error/Bias [m]','MAE [m]','BBox H/W','Saturated'],
    V3_42_LIGHT.flatMap(l=>V3_42.map(d=>{const a=v3Rows42(l[0],d),ok=a.filter(r=>+r.detected===1),conf=ok.map(r=>+r.confidence).filter(Number.isFinite),
      err=a.map(r=>+r.distance_error_m).filter(Number.isFinite),est=a.map(r=>+r.estimated_distance_m).filter(Number.isFinite),lum=a.map(r=>+r.illumination_mean_luma).filter(Number.isFinite),
      hf=ok.map(r=>Number.isFinite(+r.bbox_height_fraction)?+r.bbox_height_fraction:(+r.image_height>0?+r.bbox_height/+r.image_height:NaN)).filter(Number.isFinite),
      wf=ok.map(r=>Number.isFinite(+r.bbox_width_fraction)?+r.bbox_width_fraction:(+r.image_width>0?+r.bbox_width/+r.image_width:NaN)).filter(Number.isFinite),
      sat=ok.filter(r=>r.bbox_saturated===true||r.distance_quality==='SATURATED_NEAR'||
        (Number.isFinite(+r.bbox_height_fraction)?+r.bbox_height_fraction:(+r.image_height>0?+r.bbox_height/+r.image_height:0))>=0.95).length,
      cm=conf.length?conf.reduce((x,y)=>x+y,0)/conf.length:NaN,
      cs=conf.length>1?Math.sqrt(conf.reduce((n,x)=>n+(x-cm)*(x-cm),0)/(conf.length-1)):conf.length?0:NaN;
      return [l[0]==='day'?'SIANG':'MALAM',d+' m',a.length,ok.length,(a.length?100*ok.length/a.length:0).toFixed(1)+'%',
        Number.isFinite(cm)?v3num(cm,3)+' ± '+v3num(cs,3):'--',
        lum.length?v3num(lum.reduce((x,y)=>x+y,0)/lum.length,1):'--',
        est.length?v3num(est.reduce((x,y)=>x+y,0)/est.length,3):'--',
        err.length?v3num(err.reduce((x,y)=>x+y,0)/err.length,3):'--',
        err.length?v3num(err.reduce((x,y)=>x+Math.abs(y),0)/err.length,3):'--',
        (hf.length?v3num(hf.reduce((x,y)=>x+y,0)/hf.length,3):'--')+' / '+(wf.length?v3num(wf.reduce((x,y)=>x+y,0)/wf.length,3):'--'),
        sat+'/'+ok.length]})))}
  if(id==='F4.3'){box.innerHTML=v3Table(['Kondisi','N','Expected Status','Success Rate','Ref Offset [m]','Actual Offset [m]','MAE Center [m]','LEFT Gap [m]','RIGHT Gap [m]'],
    V3_43.map(x=>{const a=v3Rows('4.3',x[0]),ok=a.filter(r=>r.status_match===true),ce=a.map(r=>Math.abs(+r.centering_error_m)).filter(Number.isFinite);
      return [x[3],a.length,a[0]?.expected_status||'--',(a.length?100*ok.length/a.length:0).toFixed(1)+'%',a.length?v3num(+a[0].lane_offset_reference_m,3):'--',
        v3Mean(a,'lane_offset_actual_m'),ce.length?v3num(ce.reduce((p,q)=>p+q,0)/ce.length,3):'--',v3Mean(a,'left_gap_m'),v3Mean(a,'right_gap_m')]})) }
  if(id==='F4.4'){box.innerHTML=v3Table(['Kondisi','Sampel','Target FPS','Actual FPS','FPS Error/Margin','Target Period [ms]','Actual Inference [ms]','Inference Error [ms]','Mean Frame Age','Dropped','STALE'],
    V3_44.map(x=>{const a=v3Rows('4.4',x[0]);return [x[1],a.length,v3Mean(a,'fps_reference_hz',2),v3Mean(a,'fps_actual_hz',2),v3Mean(a,'fps_error_hz',2),
      v3Mean(a,'inference_time_reference_ms',1),v3Mean(a,'inference_time_actual_ms',1),v3Mean(a,'inference_time_error_ms',1),v3Mean(a,'frame_age_ms',2),
      a.reduce((n,r)=>n+(+r.dropped_frame||0),0),a.filter(r=>r.stream_status!=='LIVE').length]})) }
  if(id==='F4.5'){const a=[];Object.values(v3Sub('4.5').conditions||{}).forEach(c=>(c.rows||[]).forEach(r=>a.push(r)));
    if(V3.tab45==='lane'){
      const z=a.filter(r=>String(r.condition||'').startsWith('lane_trial_')).slice(-60);
      box.innerHTML=v3Table(['Mode','Kondisi','Initial Lane','Final Lane','Goal Ref [m]','Actual [m]','Goal Error [m]','Endpoint Error [m]','Lane Offset Err [m]','Heading Err [deg]','Steer Err [deg]','Integration','Recovery [ms]','Final State','Result'],
        z.map(r=>[r.mode,r.condition,r.initial_lane_status||r.expected_status||'--',r.final_lane_status||((r.left_status||'--')+'|'+(r.right_status||'--')),
          v3num(r.goal_reference_m??r.target_distance_m,3),v3num(r.goal_actual_progress_m??r.forward_progress_m,3),v3num(r.goal_signed_error_m,3),v3num(r.goal_endpoint_error_m,3),
          v3num(r.lane_offset_error_m,3),v3num(r.heading_signed_error_deg,2),v3num(r.steering_signed_error_deg,2),
          r.integration_pass===true?'PASS':r.integration_pass===false?'FAIL':'--',v3num(r.recovery_ms,0),r.final_vehicle_state||'--',r.result]))
    }else{
      const z=a.filter(r=>String(r.condition||'').startsWith('obstacle_trial_')).slice(-60);
      box.innerHTML=v3Table(['Mode','Kondisi','Goal Ref [m]','Actual [m]','Goal Error [m]','Endpoint Error [m]','Metric','World Memory','Planning','Costmap','MPPI','Avoid','Side','Response [ms]','Final State','Result'],
        z.map(r=>[r.mode,r.condition,v3num(r.goal_reference_m??r.goal_distance_m,3),v3num(r.goal_actual_progress_m??r.forward_progress_m,3),v3num(r.goal_signed_error_m,3),v3num(r.goal_endpoint_error_m,3),
          r.metric_position_valid===true?'PASS':r.metric_position_valid===false?'FAIL':'--',
          r.obstacle_memory_released_seen===true?'LATCH → RELEASE':r.obstacle_memory_latched_seen===true||r.obstacle_memory_latched===true?'LATCHED':'--',
          r.planning_cloud_seen===true||+r.planning_relevant_points_count>0?'PASS':'--',r.costmap_marking_seen===true||r.costmap_marked===true?'PASS':'--',
          r.mppi_seen===true||r.mppi_active===true?'PASS':'--',r.avoidance_seen===true||r.avoidance_runtime===true?'YES':'--',r.avoid_direction||'--',
          v3num(r.response_time_ms,0),r.final_vehicle_state||'--',r.result]))
    }}
  v3DrawChart();
}
function v3DrawChart(){
  const cv=$('v3Chart');if(!cv)return;
  const x=cv.getContext('2d'),W=cv.width,H=cv.height;x.clearRect(0,0,W,H);x.fillStyle='#071113';x.fillRect(0,0,W,H);
  let vals=[],labels=[],title='';const id=selectedExperiment?.id;
  if(id==='F4.1'){const a=v3Rows('4.1','validation');vals=a.map(r=>+r.fps||0);labels=a.map((_,i)=>String(i+1));title='FPS Aktual'}
  if(id==='F4.2'){vals=V3_42_LIGHT.flatMap(l=>V3_42.map(d=>{const a=v3Rows42(l[0],d);return a.length?100*a.filter(r=>+r.detected===1).length/a.length:0}));labels=V3_42_LIGHT.flatMap(l=>V3_42.map(d=>(l[0]==='day'?'S':'M')+d));title='Detection Rate SIANG vs MALAM [%]'}
  if(id==='F4.3'){vals=V3_43.map(c=>{const a=v3Rows('4.3',c[0]);return a.length?100*a.filter(r=>r.status_match===true).length/a.length:0});labels=V3_43.map((_,i)=>'K'+(i+1));title='Kesesuaian Status [%]'}
  if(id==='F4.4'){vals=V3_44.map(c=>{const a=v3Rows('4.4',c[0]),z=a.map(r=>+r.fps).filter(Number.isFinite);return z.length?z.reduce((p,q)=>p+q,0)/z.length:0});labels=['A','B','C'];title='Mean Pipeline FPS'}
  if(id==='F4.5'){const c=v3Sub('4.5').conditions||{},keys=['obstacle_trial_1','obstacle_trial_2','obstacle_trial_3','lane_trial_1','lane_trial_2','lane_trial_3'];labels=['O1','O2','O3','L1','L2','L3'];vals=keys.map(k=>(c[k]?.rows||[]).length);title='Jumlah Run Konsep 3-Trial'}
  x.fillStyle='#dcebea';x.font='700 15px sans-serif';x.fillText(title,18,24);if(!vals.length)return;
  let max=1;vals.forEach(v=>{if(v>max)max=v});const pad=50,bw=(W-pad*2)/vals.length;
  x.strokeStyle='#4b6669';x.beginPath();x.moveTo(pad,35);x.lineTo(pad,H-pad);x.lineTo(W-pad,H-pad);x.stroke();
  vals.forEach((v,i)=>{const h=(H-pad-55)*(v/max),px=pad+i*bw+bw*.18,py=H-pad-h;x.fillStyle='#31a8ff';x.fillRect(px,py,bw*.64,h);x.fillStyle='#b9cfcb';x.font='12px sans-serif';x.fillText(labels[i],px,H-pad+18);x.fillText(v3num(v,1),px,Math.max(48,py-5))});
}
function v3DrawOverlay(allowVisualHold=true){
  const c=$('v3Overlay'),stage=$('v3CameraHost'),img=$('v3CameraImage');if(!c||!stage)return;
  const r=stage.getBoundingClientRect();c.width=Math.max(1,Math.round(r.width));c.height=Math.max(1,Math.round(r.height));
  const x=c.getContext('2d'),l=v3Lane();x.clearRect(0,0,c.width,c.height);x.font='800 14px sans-serif';x.lineWidth=4;
  [['LEFT • '+l.left,12],['RIGHT • '+l.right,Math.max(12,c.width-170)]].forEach(t=>{x.strokeStyle='rgba(0,0,0,.65)';x.strokeText(t[0],t[1],24);x.fillStyle='#fff';x.fillText(t[0],t[1],24)});
  const id=selectedExperiment?.id,drawHuman=id==='F4.2'||(id==='F4.5'&&V3.tab45==='obstacle');
  if(!drawHuman||!img?.naturalWidth||!img?.naturalHeight)return;
  let h=id==='F4.5'?v3Human45Detected():v3Human(id==='F4.2'),visualHold=false,holdAge=0;
  if(id==='F4.5'){
    if(h){V3.visualHuman45=Object.assign({},h);V3.visualHuman45At=Date.now()}
    else if(allowVisualHold&&V3.visualHuman45&&Date.now()-V3.visualHuman45At<=2000){
      h=V3.visualHuman45;visualHold=true;holdAge=Date.now()-V3.visualHuman45At
    }else if(!allowVisualHold||Date.now()-V3.visualHuman45At>2000){V3.visualHuman45=null;V3.visualHuman45At=0}
  }
  if(!h)return;
  const sem=obj('semantic_detections'),iw=Math.max(1,+sem.image_width||img.naturalWidth),ih=Math.max(1,+sem.image_height||img.naturalHeight);
  const scale=Math.min(r.width/img.naturalWidth,r.height/img.naturalHeight),renderW=img.naturalWidth*scale,renderH=img.naturalHeight*scale,ox=(r.width-renderW)/2,oy=(r.height-renderH)/2;
  const x1=iw-(+h.x2||0),x2=iw-(+h.x1||0),y1=+h.y1||0,y2=+h.y2||0;
  const sx=renderW/iw,sy=renderH/ih,px=ox+x1*sx,py=oy+y1*sy,pw=Math.max(1,(x2-x1)*sx),ph=Math.max(1,(y2-y1)*sy);
  x.strokeStyle=visualHold?'#f6bd60':'#48e0a4';x.lineWidth=3;if(visualHold)x.setLineDash([9,6]);x.strokeRect(px,py,pw,ph);x.setLineDash([]);
  const label=visualHold?('HUMAN LAST '+v3num(holdAge/1000,1)+'s'):('HUMAN '+v3num((+h.score||0)*100,1)+'% '+v3esc(h.tracking_state||'STRONG'));
  x.font='800 13px sans-serif';const tw=x.measureText(label).width+10,ty=Math.max(18,py-5);
  x.fillStyle='rgba(0,0,0,.72)';x.fillRect(px,ty-16,tw,19);x.fillStyle=visualHold?'#f6bd60':'#48e0a4';x.fillText(label,px+5,ty-2);
}
function v3StopAuto30(sub,msg=''){
  const t=sub==='4.2'?'auto42Timer':'auto43Timer',c=sub==='4.2'?'auto42Cond':'auto43Cond',b=sub==='4.2'?'auto42Busy':'auto43Busy';
  if(V3[t])clearInterval(V3[t]);V3[t]=null;V3[c]=null;V3[b]=false;if(msg)toast(msg);v3Render()
}
async function v3StartAuto30(sub,condition,extra={}){
  const t=sub==='4.2'?'auto42Timer':'auto43Timer',c=sub==='4.2'?'auto42Cond':'auto43Cond',b=sub==='4.2'?'auto42Busy':'auto43Busy';
  if(V3[t])return;
  try{
    const j=await v3Post('/api/perception/bab4/start',{subchapter:sub,condition});V3.state=j.state;V3[c]=condition;V3[b]=false;
    const started=+(j.state?.subchapters?.[sub]?.conditions?.[condition]?.started_at_ms||Date.now());
    const deadline=+(j.state?.subchapters?.[sub]?.conditions?.[condition]?.deadline_ms||started+30000);
    const tick=async()=>{
      if(V3[c]!==condition||V3[b])return;
      const cond=v3Cond(sub,condition),count=+(cond.sample_count||0),now=Date.now(),elapsed=Math.max(0,now-started);
      if(count>=10){v3StopAuto30(sub,sub+' '+condition+' selesai otomatis 10/10');return}
      if(now>deadline){v3StopAuto30(sub,sub+' '+condition+' berhenti: hanya '+count+'/10 sample fresh dalam 30 detik');return}
      const target=Math.min(10,Math.floor(elapsed/2800)+1);
      if(count>=target)return;
      V3[b]=true;
      try{
        const q=Object.assign({subchapter:sub,condition},extra),r=await v3Post('/api/perception/bab4/capture',q);
        V3.state=r.state;V3.captured=r.row||null;V3.capturedImage=r.captured_image_data_url||'';v3Render()
      }catch(e){
        if(e.code==='SESSION_CHANGED'||/Session Bab 4 berubah/.test(String(e.message||''))){
          v3StopAuto30(sub,'Session berubah — auto capture dihentikan');await v3FetchState();return
        }
        if(!/BELUM UPDATE|STALE/.test(String(e.message||'')))toast('AUTO '+sub+' • '+e.message,true)
      }finally{V3[b]=false}
    };
    V3[t]=setInterval(tick,250);toast(sub+' '+condition+' • auto capture 10x/30 detik dimulai');v3Render();tick()
  }catch(e){V3[c]=null;V3[b]=false;toast(e.message,true)}
}
async function v3Start(sub,condition){try{await v3Post('/api/perception/bab4/start',{subchapter:sub,condition});toast(sub+' '+condition+' dimulai');v3Render()}catch(e){toast(e.message,true)}}
async function v3Capture(sub,condition,extra={}){
  try{const j=await v3Post('/api/perception/bab4/capture',Object.assign({subchapter:sub,condition},extra));if(j.state)V3.state=j.state;toast('Capture tersimpan • '+condition);v3Render();return j}
  catch(e){toast('CAPTURE FAILED — '+e.message,true);return null}
}
async function v3Reset(sub,condition,all=false){
  if(!confirm(all?'Hapus seluruh data '+sub+'?':'Reset hanya kondisi aktif '+condition+'?'))return;
  if(sub==='4.2')v3StopAuto30('4.2');if(sub==='4.3')v3StopAuto30('4.3');
  try{await v3Post('/api/perception/bab4/reset',{scope:all?'subchapter':'condition',subchapter:sub,condition});V3.captured=null;V3.capturedImage='';V3.armed=false;await v3FetchState();toast('Reset berhasil');v3Render()}catch(e){toast(e.message,true)}
}
async function v3Reset45Group(group){
  const label=group==='obstacle'?'seluruh Object Detection 4.5':'seluruh Lane Safety 4.5';
  if(!confirm('Hapus '+label+'? Data kelompok lain tetap dipertahankan.'))return;
  try{await v3Post('/api/perception/bab4/reset',{scope:'group',subchapter:'4.5',group});V3.captured=null;V3.capturedImage='';V3.armed=false;await v3FetchState();toast('Reset '+label+' berhasil');v3Render()}catch(e){toast('RESET 4.5 GAGAL — '+e.message,true)}
}
async function v3Export(sub){
  try{const r=await fetch('/api/perception/bab4/export/'+sub+'.zip',{cache:'no-store'});if(!r.ok)throw Error(await r.text());const b=await r.blob();if(!b.size)throw Error('ZIP kosong');
    V3.exports[sub]=true;const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download=(V3.state?.session_id||'BAB4')+'_'+sub+'.zip';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),2000);toast('ZIP '+sub+' tervalidasi • '+b.size+' bytes');v3Render()}
  catch(e){toast('EXPORT FAILED — '+e.message,true)}
}
function v3Stop44(){if(V3.timer44)clearInterval(V3.timer44);V3.timer44=null;V3.run44=null;V3.busy44=false;toast('Trigger recorder dihentikan. Data yang sudah tersimpan tetap di backend.');v3Render()}
async function v3Start44(){
  const cond=V3.sel44;try{await v3Post('/api/perception/bab4/start',{subchapter:'4.4',condition:cond});V3.run44=cond;V3.sampleTick=0;
    if(V3.timer44)clearInterval(V3.timer44);V3.timer44=setInterval(async()=>{if(V3.busy44||V3.run44!==cond)return;const c=v3Cond('4.4',cond),deadline=+c.deadline_ms||0;
      if(deadline&&Date.now()>deadline){v3Stop44();return}V3.busy44=true;try{const j=await v3Post('/api/perception/bab4/capture',{subchapter:'4.4',condition:cond,elapsed_s:(Date.now()-(+c.started_at_ms||Date.now()))/1000});
        V3.sampleTick++;if(j.state?.subchapters?.['4.4']?.conditions?.[cond]?.status==='SELESAI'){v3Stop44();return}if(V3.sampleTick%4===0)v3Render()}
      catch(e){toast('4.4 recorder: '+e.message,true);v3Stop44()}finally{V3.busy44=false}},500);toast('Recorder 4.4 berjalan 60 detik');v3Render()}
  catch(e){toast(e.message,true)}
}
function v3Token(){const a=new Uint32Array(6);crypto.getRandomValues(a);return Array.from(a).map(x=>x.toString(16)).join('-')}
async function v3DryRun45(){
  if(V3.preview45Busy)return;
  const obstacle=V3.tab45==='obstacle';
  const trial=Math.max(1,Math.min(3,+V3.obs45||1));
  const laneTrial=Math.max(1,Math.min(3,+V3.lane45||1));
  const key=obstacle?('trial'+trial):('lane_trial_'+laneTrial);
  const cond=obstacle?('obstacle_trial_'+trial):('lane_trial_'+laneTrial);
  let preview=false;V3.preview45Busy=true;v3Render();
  try{
    if(obstacle){
      await v3Post('/api/perception/final-trial/preview-start',{mode:'obstacle',condition:key,session_id:V3.state?.session_id||''});
      preview=true;toast('DRY RUN Trial '+trial+' • goal preview 5 m • kendaraan dikunci STOP');
      const startedAt=Date.now(),until=startedAt+12000;let peakEvidence=false;
      while(Date.now()<until){
        await new Promise(r=>setTimeout(r,250));loadCostmapImage('local');loadCostmapImage('global');v3Draw45Map();
        const hNow=v3Human45(),cmNow=obj('local_costmap_meta'),mppiNow=obj('mppi_trajectories');
        const humanNowOk=!!hNow&&String(hNow.tracking_state||'STRONG').toUpperCase()!=='HELD';
        const cloudNowOk=channelFresh('object_points',1.2)&&(+obj('object_points').count||0)>0&&
          channelFresh('path_relevant_points',1.2)&&(+obj('path_relevant_points').count||0)>0&&
          channelFresh('planning_relevant_points',1.2)&&(+obj('planning_relevant_points').count||0)>0;
        const costmapNowOk=channelFresh('local_costmap_meta',1.2)&&cmNow.planning_alignment_pass===true&&(+cmNow.nonzero_cells||0)>0;
        const mppiNowOk=channelFresh('mppi_trajectories',2.0)&&(+mppiNow.count||0)>0&&(+mppiNow.optimal_count||0)>0;
        if(Date.now()-startedAt>=750&&humanNowOk&&cloudNowOk&&costmapNowOk&&mppiNowOk){peakEvidence=true;break}
      }
      if(peakEvidence)toast('Evidence obstacle lengkap • capture dilakukan pada momen valid');
    }
    const j=await v3Capture('4.5',cond);
    if(j?.row){
      if(obstacle){
        const bundle=v3Make45VisualBundle({mode:'DRY_RUN',condition:cond,capture_id:j.row.capture_id,result:j.row.integration_pass===true?'PASS':'FAIL',
          avoid_direction:j.row.avoid_direction,recovery_seen:j.row.recovery_seen===true});
        if(bundle){try{await v3Save45VisualEvidence(cond,j.row,bundle);toast('Visual DRY RUN tersimpan • camera + map + combined')}catch(e){toast('VISUAL DRY RUN GAGAL — '+e.message,true)}}
        else toast('VISUAL DRY RUN GAGAL — camera/map belum siap',true)
      }
      const pass=j.row.integration_pass===true;
      toast(pass?'DRY RUN '+cond+' PASS — START Trial dapat dibuka':'DRY RUN '+cond+' FAIL — reset trial lalu ulangi',!pass)
    }
  }catch(e){toast('DRY RUN 4.5 GAGAL — '+e.message,true)}
  finally{
    if(preview){try{await v3Post('/api/perception/final-trial/preview-stop',{})}catch(e){toast('STOP preview gagal: '+e.message,true)}}
    V3.preview45Busy=false;await v3FetchState();v3Render()
  }
}
function v3StopHeartbeatWorker(){
  if(V3.heartbeat){clearInterval(V3.heartbeat);V3.heartbeat=null}
  if(V3.heartbeatWorker){
    try{V3.heartbeatWorker.postMessage({type:'stop'})}catch(_){}
    try{V3.heartbeatWorker.terminate()}catch(_){}
    V3.heartbeatWorker=null
  }
  V3.heartbeatBusy=false
}
async function v3HandleHeartbeatWorkerMessage(event){
  const d=event?.data||{};
  if(d.type==='heartbeat_error'){
    if(d.code==='SESSION_CHANGED'||/Session ARMED tidak cocok|Session Bab 4 berubah/.test(String(d.message||''))){
      v3StopHeartbeatWorker();V3.armedToken='';V3.armedSessionId='';V3.armedMode45='';V3.armed=false;
      await v3FetchState();toast('Trial dihentikan: session berubah',true);v3Render();return
    }
    V3.heartbeatFails=(V3.heartbeatFails||0)+1;
    if(V3.heartbeatFails===1)toast('Heartbeat worker tersendat — retry otomatis, safety watchdog tetap aktif',true);
    return
  }
  if(d.type!=='heartbeat_ok')return;
  V3.heartbeatFails=0;
  const h={trial:d.trial||{}},ph=String(h.trial?.phase||''),mode=V3.armedMode45,armedCondition=V3.armedCondition45;
  if(mode==='obstacle'&&ph==='running'&&h.trial?.avoidance_seen===true&&!V3.armedVisual45){
    V3.armedVisual45=v3Make45VisualBundle({mode:'ARMED • PEAK AVOIDANCE',condition:armedCondition,result:'AVOIDANCE ACTIVE',
      avoid_direction:h.trial?.avoid_direction,recovery_seen:h.trial?.recovery_seen===true,decision:h.trial?.safety_decision_last});
    if(V3.armedVisual45)toast('Snapshot avoidance dibekukan')
  }
  if(!['countdown','running','stopping'].includes(ph)){
    v3StopHeartbeatWorker();
    const visual=mode==='obstacle'?(V3.armedVisual45||v3Make45VisualBundle({mode:'ARMED • FINAL',condition:armedCondition,result:ph,
      avoid_direction:h.trial?.avoid_direction,recovery_seen:h.trial?.recovery_seen===true,decision:h.trial?.safety_decision_last})):null;
    V3.armedToken='';V3.armedSessionId='';V3.armedMode45='';V3.armed=false;await v3FetchState();
    if(mode==='obstacle'&&visual){try{await v3SaveLatestArmed45Visual(armedCondition,visual);toast('Visual ARMED tersimpan • camera + map + combined')}catch(e){toast('VISUAL ARMED GAGAL — '+e.message,true)}}
    V3.armedVisual45=null;V3.armedCondition45='';toast('Trial selesai: '+ph);v3Render()
  }
}
function v3StartHeartbeatWorker(mode,armedCondition){
  v3StopHeartbeatWorker();
  V3.heartbeatFails=0;V3.armedMode45=mode;V3.armedCondition45=armedCondition;
  try{
    const w=new Worker('/perception_bab4_heartbeat_worker.js?v=20260926-worker-v1');
    V3.heartbeatWorker=w;
    w.onmessage=v3HandleHeartbeatWorkerMessage;
    w.onerror=()=>{V3.heartbeatFails=(V3.heartbeatFails||0)+1;if(V3.heartbeatFails===1)toast('Heartbeat worker error — backend watchdog tetap aktif',true)};
    w.postMessage({type:'start',token:V3.armedToken,session_id:V3.armedSessionId,mode,condition:armedCondition})
  }catch(e){
    V3.heartbeatWorker=null;
    throw Error('Heartbeat worker gagal dibuat: '+e.message)
  }
}
async function v3ArmedStart(){
  if(!V3.armed)return toast('ARMED belum diaktifkan.',true);
  if(prompt('Ketik ARMED untuk mengizinkan vehicle motion test.')!=='ARMED')return toast('ARMED dibatalkan.',true);
  const mode=V3.tab45;
  const obstacleTrial=Math.max(1,Math.min(3,+V3.obs45||1));
  const laneTrial=Math.max(1,Math.min(3,+V3.lane45||1));
  const lane=v3Lane();
  let key='';
  if(mode==='obstacle')key='trial'+obstacleTrial;
  else if(lane.left==='RED'&&lane.right==='GREEN'&&lane.leftLatched)key='left_red';
  else if(lane.left==='GREEN'&&lane.right==='RED'&&lane.rightLatched)key='right_red';
  if(!key)return toast('Kondisi aktual belum siap recenter: wajib RED di salah satu sisi dan sisi lain GREEN.',true);
  const armedCondition=mode==='obstacle'?('obstacle_trial_'+obstacleTrial):('lane_trial_'+laneTrial);
  V3.armedVisual45=null;V3.armedCondition45=armedCondition;
  V3.armedToken=v3Token();V3.armedSessionId=V3.state?.session_id||'';
  try{
    await v3Post('/api/perception/final-trial/start',{mode,condition:key,trial_index:mode==='lane'?laneTrial:obstacleTrial,token:V3.armedToken,session_id:V3.armedSessionId});
    toast('Trial dimulai • tetap siap menekan ABORT');
    v3StartHeartbeatWorker(mode,armedCondition);
    v3Render()
  }catch(e){V3.armedToken='';V3.armedSessionId='';V3.armed=false;toast('Trial ditolak backend: '+e.message,true);v3Render()}
}
async function v3ArmedAbort(reason='Operator abort'){
  v3StopHeartbeatWorker();
  if(!V3.armedToken){V3.armed=false;V3.armedSessionId='';V3.armedMode45='';v3Render();return}
  const token=V3.armedToken,sessionId=V3.armedSessionId;
  try{await v3Post('/api/perception/final-trial/stop',{token,session_id:sessionId,reason})}
  catch(e){toast('ABORT ARMED gagal: '+e.message,true)}
  finally{V3.armedToken='';V3.armedSessionId='';V3.armedMode45='';V3.armed=false;V3.armedVisual45=null;V3.armedCondition45='';await v3FetchState();v3Render()}
}
function v3Bind(){
  document.querySelectorAll('#perLockedBab4Panel [data-v3-id]').forEach(b=>b.onclick=()=>selectExperimentById(b.dataset.v3Id));
  if($('v3NewSession'))$('v3NewSession').onclick=async()=>{if(!confirm('Mulai session Bab 4 baru? Session lama tetap tersimpan.'))return;v3StopHeartbeatWorker();v3StopAuto30('4.2');v3StopAuto30('4.3');try{const j=await v3Post('/api/perception/bab4/session',{action:'new'});V3.state=j.state;V3.captured=null;V3.capturedImage='';toast('Session baru dibuat');v3Render()}catch(e){toast(e.message,true)}};
  if($('v3ResetSession'))$('v3ResetSession').onclick=async()=>{if(prompt('Ketik RESET untuk menghapus seluruh session Bab 4 aktif.')!=='RESET')return;v3StopHeartbeatWorker();v3StopAuto30('4.2');v3StopAuto30('4.3');try{const j=await v3Post('/api/perception/bab4/session',{action:'reset',confirm:'RESET'});V3.state=j.state;V3.captured=null;V3.capturedImage='';toast('Session direset');v3Render()}catch(e){toast(e.message,true)}};
  document.querySelectorAll('#perLockedBab4Panel [data-v3-export]').forEach(b=>b.onclick=()=>v3Export(b.dataset.v3Export));
  if($('v3OriLeft'))$('v3OriLeft').onchange=e=>{V3.orientationLeft=e.target.checked;v3Render()};if($('v3OriRight'))$('v3OriRight').onchange=e=>{V3.orientationRight=e.target.checked;v3Render()};
  if($('v3Start41'))$('v3Start41').onclick=()=>{V3.validationStarted=true;toast('Validasi 4.1 dimulai');v3Render()};
  if($('v3Capture41'))$('v3Capture41').onclick=()=>v3Capture('4.1','validation',{orientation_left_ok:V3.orientationLeft,orientation_right_ok:V3.orientationRight});
  if($('v3Reset41'))$('v3Reset41').onclick=()=>v3Reset('4.1','validation',true);
  if($('v3Light42'))$('v3Light42').onchange=e=>{V3.light42=e.target.value;v3Render()};
  if($('v3Sel42'))$('v3Sel42').onchange=e=>{V3.sel42=+e.target.value;v3Render()};if($('v3Start42'))$('v3Start42').onclick=()=>v3StartAuto30('4.2',v3Key42(V3.light42,V3.sel42),{reference_distance_m:V3.sel42,lighting_condition:V3.light42});
  if($('v3Capture42'))$('v3Capture42').onclick=()=>v3Capture('4.2',v3Key42(V3.light42,V3.sel42),{reference_distance_m:V3.sel42,lighting_condition:V3.light42});
  if($('v3Reset42'))$('v3Reset42').onclick=()=>v3Reset('4.2',v3Key42(V3.light42,V3.sel42),false);if($('v3ResetAll42'))$('v3ResetAll42').onclick=()=>v3Reset('4.2','',true);
  if($('v3Sel43'))$('v3Sel43').onchange=e=>{V3.sel43=e.target.value;v3Render()};if($('v3Start43'))$('v3Start43').onclick=()=>v3StartAuto30('4.3',V3.sel43);
  if($('v3Capture43'))$('v3Capture43').onclick=()=>v3Capture('4.3',V3.sel43);if($('v3Reset43'))$('v3Reset43').onclick=()=>v3Reset('4.3',V3.sel43,false);
  if($('v3ResetAll43'))$('v3ResetAll43').onclick=()=>v3Reset('4.3','',true);
  if($('v3Sel44'))$('v3Sel44').onchange=e=>{V3.sel44=e.target.value;v3Render()};if($('v3Start44'))$('v3Start44').onclick=v3Start44;if($('v3Stop44'))$('v3Stop44').onclick=v3Stop44;
  if($('v3Reset44'))$('v3Reset44').onclick=()=>v3Reset('4.4',V3.sel44,false);if($('v3ResetAll44'))$('v3ResetAll44').onclick=()=>v3Reset('4.4','',true);
  if($('v3TabObs'))$('v3TabObs').onclick=()=>{V3.tab45='obstacle';v3Render()};if($('v3TabLane'))$('v3TabLane').onclick=()=>{V3.tab45='lane';v3Render()};
  if($('v3Obs45'))$('v3Obs45').onchange=e=>{V3.obs45=e.target.value;V3.obsMetricLatchCond45='';V3.obsMetricLatchUntil45=0;V3.obsMetricLatchDistance45=NaN;e.target.blur();v3Render()};if($('v3Lane45'))$('v3Lane45').onchange=e=>{V3.lane45=e.target.value;e.target.blur();v3Render()};
  if($('v3Dry45'))$('v3Dry45').onclick=v3DryRun45;
  if($('v3ResetCurrent45'))$('v3ResetCurrent45').onclick=()=>{const c=V3.tab45==='obstacle'?('obstacle_trial_'+Math.max(1,Math.min(3,+V3.obs45||1))):('lane_trial_'+V3.lane45);v3Reset('4.5',c,false)};
  if($('v3ResetGroup45'))$('v3ResetGroup45').onclick=()=>v3Reset45Group(V3.tab45==='obstacle'?'obstacle':'lane');
  if($('v3ArmToggle'))$('v3ArmToggle').onchange=e=>{V3.armed=e.target.checked;e.target.blur();v3Render()};if($('v3Armed45'))$('v3Armed45').onclick=v3ArmedStart;
  if($('v3Abort45'))$('v3Abort45').onclick=()=>v3ArmedAbort();
}
v3PatchCatalog();
const v3BaseList=effectiveExperimentList;
effectiveExperimentList=function(){
  if(currentExp==='perception'&&activeWorkspaceTab?.perception==='bab4'){v3PatchCatalog();return (experiments.perception||[]).filter(x=>V3_IDS.includes(x.id))}
  return v3BaseList();
};
const v3BaseRender=renderSelectedExperiment;
renderSelectedExperiment=function(){
  if(v3IsActive()){v3Render();return}
  const r=v3BaseRender();setTimeout(v3Render,0);return r
};
const v3BaseWorkspace=activateWorkspaceTab;
activateWorkspaceTab=function(id){
  const r=v3BaseWorkspace(id);if(activeDomain==='perception'&&id==='bab4'){v3PatchCatalog();if(!V3_IDS.includes(selectedExperiment?.id))selectExperimentById('F4.1');setTimeout(v3Render,0)}
  else setTimeout(v3Render,0);return r
};
let v3Last45LiveRender=0;
function v3Refresh45Live(){
  if(!v3IsActive()||selectedExperiment?.id!=='F4.5')return;
  const panel=$('perLockedBab4Panel'),ae=document.activeElement;
  // Buttons must never freeze live lane KPIs after a click. Only preserve an actively edited form control.
  if(panel&&ae&&panel.contains(ae)&&/^(SELECT|INPUT)$/.test(ae.tagName))return;
  const now=performance.now();if(now-v3Last45LiveRender<500)return;v3Last45LiveRender=now;
  const sx=window.scrollX,sy=window.scrollY;v3Render(true);
  requestAnimationFrame(()=>{if(Math.abs(window.scrollY-sy)>2||Math.abs(window.scrollX-sx)>2)window.scrollTo(sx,sy)});
}
window.v3Refresh45Live=v3Refresh45Live;
// 250 ms fallback: overlays/maps stay fast; 4.5 live KPI/gates are rebuilt at <=2 Hz.
setInterval(()=>{if(v3IsActive()){v3DrawOverlay();if(selectedExperiment?.id==='F4.5'){v3Refresh45Live();v3Draw45Map()}}},250);
setTimeout(()=>{v3PatchCatalog();v3FetchState();v3Render()},700);

window.addEventListener('beforeunload',()=>v3StopHeartbeatWorker());
