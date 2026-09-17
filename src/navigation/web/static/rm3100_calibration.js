'use strict';
let rm3100CalibrationUi={running:false,status:'IDLE',face_counts:{},coverage_pct:0,sample_count:0};
let rm3100PollBusy=false,rm3100LastWireAt=0;
const rm3100Plot={raw:[],cal:[],norm:[]};
const RM3100_FACE_MIN=30;
const RM3100_FACES=[
  ['LEVEL','Level / datar','Sensor datar. Putar perlahan 360° terhadap sumbu vertikal.'],
  ['LEFT_SIDE','Sisi kiri','Miringkan modul/kendaraan ±90° ke kiri, lalu putar perlahan 360°.'],
  ['RIGHT_SIDE','Sisi kanan','Miringkan ±90° ke kanan, lalu putar perlahan 360°.'],
  ['NOSE_UP','Depan atas','Arah depan sensor menghadap ke atas, lalu putar perlahan 360°.'],
  ['NOSE_DOWN','Depan bawah','Arah depan sensor menghadap ke bawah, lalu putar perlahan 360°.'],
  ['INVERTED','Terbalik','Balik sensor 180°, lalu putar perlahan 360°.']
];
function rm3100FitRoot(){return rm3100CalibrationUi?.fit?.rm3100_ardupilot_calibration||null}
function rm3100CurrentFace(){return rm3100CalibrationUi?.target_face||rm3100CalibrationUi?.raw_wire?.face||'LEVEL'}
function rm3100NextFace(){const c=rm3100CalibrationUi.face_counts||{};return RM3100_FACES.find(([k])=>(+c[k]||0)<RM3100_FACE_MIN)?.[0]||'REVIEW'}
function rm3100FaceInstruction(face){const row=RM3100_FACES.find(x=>x[0]===face);return row?row[2]:'Coverage minimum tercapai. Hentikan dan jalankan fitting.'}
function rm3100Fmt(v,n=2){return Number.isFinite(+v)?(+v).toFixed(n):'--'}
function rm3100ApState(){
  const p=obj('neo3_compass_params'),keys=['COMPASS_OFS_X','COMPASS_OFS_Y','COMPASS_OFS_Z'],ofs=keys.map(k=>+p[k]);
  const diaKeys=['COMPASS_DIA_X','COMPASS_DIA_Y','COMPASS_DIA_Z'],odiKeys=['COMPASS_ODI_X','COMPASS_ODI_Y','COMPASS_ODI_Z'];
  const dia=diaKeys.map(k=>+p[k]),odi=odiKeys.map(k=>+p[k]),softText=[...diaKeys,...odiKeys].map(k=>String(p[k]??''));
  const baseComplete=ofs.every(Number.isFinite)&&Number.isFinite(+p.COMPASS_ORIENT)&&Number.isFinite(+p.COMPASS_DEV_ID);
  const softSupported=[...dia,...odi].every(Number.isFinite),softUnsupported=softText.length===6&&softText.every(v=>v==='NOT_FOUND');
  const hardIronIdentity=ofs.every(v=>Math.abs(v)<=0.05),softIronIdentity=softSupported?dia.every(v=>Math.abs(v-1)<=0.001)&&odi.every(v=>Math.abs(v)<=0.001):softUnsupported;
  const complete=baseComplete&&(softSupported||softUnsupported),identity=complete&&hardIronIdentity&&softIronIdentity;
  let state='WAIT';if(complete&&identity)state=softUnsupported?'AP_PERIPH RAW • HOST FULL-3D SAFE':'IDENTITY / HOST SAFE';else if(complete)state='AP_PERIPH CALIBRATED / HOST BLOCKED';
  return{params:p,complete,identity,softSupported,softUnsupported,state};
}
async function pollRm3100Calibration(force=false){
  if(rm3100PollBusy||(!force&&activePage!=='rm3100-calibration'&&!rm3100CalibrationUi.running))return;
  rm3100PollBusy=true;
  try{const r=await readRequest('/api/rm3100/calibration/status',{timeoutMs:2500}),j=await r.json();if(r.ok){rm3100CalibrationUi=j;queueRender(true)}}catch(_){ }finally{rm3100PollBusy=false}
}
async function startRm3100Calibration(){
  const b=$('rm3100StartBtn');if(b)b.disabled=true;
  try{const r=await writeRequest('/api/rm3100/calibration/start',{}),j=await r.json();toast(j.message||'RM3100 start',!r.ok);await pollRm3100Calibration(true)}finally{if(b)b.disabled=false}
}
async function stopRm3100Calibration(){
  const b=$('rm3100StopBtn');if(b)b.disabled=true;
  try{const r=await writeRequest('/api/rm3100/calibration/stop',{}, {timeoutMs:12000}),j=await r.json();toast(j.message||'RM3100 stop',!r.ok);await pollRm3100Calibration(true)}finally{if(b)b.disabled=false}
}
async function setRm3100Face(face){
  if(!rm3100CalibrationUi.running)return toast('Start kalibrasi dulu.',true);
  const f=String(face||'').toUpperCase();if(!RM3100_FACES.some(x=>x[0]===f))return;
  const r=await writeRequest('/api/rm3100/calibration/step',{face:f}),j=await r.json();toast(j.message||('Target '+f),!r.ok);await pollRm3100Calibration(true);
}
async function nextRm3100Face(){
  const cur=rm3100CurrentFace(),counts=rm3100CalibrationUi.face_counts||{},i=RM3100_FACES.findIndex(x=>x[0]===cur);
  if(i<0)return setRm3100Face('LEVEL');
  if((+counts[cur]||0)<RM3100_FACE_MIN)return toast(`Fase ${cur} belum cukup ${RM3100_FACE_MIN} sample. Selesaikan satu putaran 360° dulu.`,true);
  if(i>=RM3100_FACES.length-1)return toast('Fase INVERTED selesai. Tekan STOP & FIT.',false);
  return setRm3100Face(RM3100_FACES[i+1][0]);
}async function stageRm3100Full3d(){
  const fit=rm3100FitRoot(),apState=rm3100ApState();if(!fit?.valid)return toast('Fit 3D belum PASS.',true);
  if(!apState.complete)return toast('AP_Periph compass readback belum lengkap. Refresh/reconnect F411 terlebih dahulu.',true);
  if(!apState.identity)return toast('AP_Periph sudah memiliki koreksi compass. Host Full-3D diblok untuk mencegah double calibration.',true);
  const host=fit.host_ros||{};if(!Array.isArray(host.bias_xyz_ut)||host.bias_xyz_ut.length!==3||!Array.isArray(host.matrix_3x3)||host.matrix_3x3.length!==9)return toast('Fit 3D tidak lengkap.',true);
  const rows=[
    ['rm3100_calibration_owner','ros_host'],
    ['rm3100_full_calibration_enabled',true],['rm3100_planar_calibration_enabled',false],
    ['rm3100_mag_bias_xyz_ut',host.bias_xyz_ut],['rm3100_mag_matrix_3x3',host.matrix_3x3],
    ['rm3100_heading_lut_enabled',false]
  ].map(([name,value])=>({fileKey:'mag_heading',path:`mag_heading_fusion.ros__parameters.${name}`,value,name:`RM3100 ${name}`}));
  const staged=[];
  for(const row of rows){if(stageConfigChange(row.fileKey,row.path,row.value,{name:row.name,source:'rm3100-full3d'})){const x=configPending.get(`${row.fileKey}:${row.path}`);if(x)staged.push(x)}}
  if(!staged.length)return toast('Nilai fit sama dengan YAML aktif; tidak ada diff baru.');
  try{await validateConfigItems(staged,{openReview:true});toast('RM3100 Full 3D staged + validated. Review Diff lalu Apply.')}catch(e){toast('Validate RM3100 gagal: '+e.message,true)}
}
function rm3100CapturePlot(){
  const rawMag=obj('rm3100_mag'),calMag=obj('rm3100_mag_calibrated'),at=+updated.rm3100_mag||0;if(!at||at===rm3100LastWireAt)return;rm3100LastWireAt=at;
  if(Number.isFinite(+rawMag.x_ut)&&Number.isFinite(+rawMag.y_ut)){rm3100Plot.raw.push([+rawMag.x_ut,+rawMag.y_ut]);if(rm3100Plot.raw.length>500)rm3100Plot.raw.shift()}
  if(Number.isFinite(+calMag.x_ut)&&Number.isFinite(+calMag.y_ut)){rm3100Plot.cal.push([+calMag.x_ut,+calMag.y_ut]);if(rm3100Plot.cal.length>500)rm3100Plot.cal.shift()}
  if(Number.isFinite(+rawMag.norm_ut)){rm3100Plot.norm.push([+rawMag.norm_ut,Number.isFinite(+calMag.norm_ut)?+calMag.norm_ut:NaN]);if(rm3100Plot.norm.length>500)rm3100Plot.norm.shift()}
}
function rm3100Canvas(id){const c=$(id);if(!c)return null;const dpr=Math.min(devicePixelRatio||1,2),r=c.getBoundingClientRect(),w=Math.max(300,Math.round(r.width*dpr)),h=Math.max(180,Math.round(r.height*dpr));if(c.width!==w||c.height!==h){c.width=w;c.height=h}return{c,ctx:c.getContext('2d'),w,h,dpr}}
function drawRm3100XY(){
  const o=rm3100Canvas('rm3100XYCanvas');if(!o)return;const{ctx,w,h,dpr}=o,all=[...rm3100Plot.raw,...rm3100Plot.cal];ctx.clearRect(0,0,w,h);ctx.fillStyle=getComputedStyle(document.body).getPropertyValue('--bg2');ctx.fillRect(0,0,w,h);
  if(!all.length)return;const lim=Math.max(20,...all.flatMap(p=>p.map(Math.abs)))*1.12,sx=x=>w/2+x/lim*w*.44,sy=y=>h/2-y/lim*h*.44;ctx.strokeStyle='rgba(140,160,165,.25)';ctx.beginPath();ctx.moveTo(w/2,0);ctx.lineTo(w/2,h);ctx.moveTo(0,h/2);ctx.lineTo(w,h/2);ctx.stroke();
  const css=getComputedStyle(document.body);for(const [arr,col] of [[rm3100Plot.raw,'--amber'],[rm3100Plot.cal,'--cyan']]){ctx.fillStyle=css.getPropertyValue(col);for(const p of arr.slice(-350)){ctx.globalAlpha=.55;ctx.beginPath();ctx.arc(sx(p[0]),sy(p[1]),1.7*dpr,0,Math.PI*2);ctx.fill()}}ctx.globalAlpha=1;
}
function drawRm3100Norm(){
  const o=rm3100Canvas('rm3100NormCanvas');if(!o)return;const{ctx,w,h}=o,arr=rm3100Plot.norm;ctx.clearRect(0,0,w,h);ctx.fillStyle=getComputedStyle(document.body).getPropertyValue('--bg2');ctx.fillRect(0,0,w,h);if(arr.length<2)return;
  const vals=arr.flat().filter(Number.isFinite),lo=Math.min(...vals)-2,hi=Math.max(...vals)+2,span=Math.max(5,hi-lo),css=getComputedStyle(document.body);for(const [idx,col] of [[0,'--amber'],[1,'--cyan']]){ctx.strokeStyle=css.getPropertyValue(col);ctx.lineWidth=2;ctx.beginPath();arr.forEach((p,i)=>{const v=p[idx];if(!Number.isFinite(v))return;const x=i/(arr.length-1)*w,y=h-(v-lo)/span*h;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke()}
}function renderRm3100Calibration(){
  if(!$('page-rm3100-calibration'))return;rm3100CapturePlot();const s=rm3100CalibrationUi||{},fit=rm3100FitRoot(),wire=obj('rm3100_mag'),cal=obj('rm3100_mag_calibrated'),heading=obj('rm3100_heading'),next=rm3100NextFace(),current=rm3100CurrentFace(),observed=s.observed_face||s.raw_wire?.observed_face||'--',counts=s.face_counts||{},preflight=!!s.stationary_ok&&!!s.sensor_online;
  setChip('rm3100CalChip',s.running,'RECORDING',s.status==='PASS'?'PASS':s.status||'IDLE');const chip=$('rm3100CalChip');if(chip){chip.classList.toggle('waiting',!s.running&&s.status!=='PASS');chip.classList.toggle('bad',s.status==='FAILED')}
  setText('rm3100Samples',String(s.sample_count??0));setText('rm3100Coverage',`${rm3100Fmt(s.coverage_pct,1)}%`);setText('rm3100Elapsed',`${rm3100Fmt(s.elapsed_sec,1)} s`);setText('rm3100CurrentFace',s.running?`${current} • IMU ${observed}`:current);setText('rm3100FitState',fit?.valid?'PASS':s.status||'WAIT');
  const progress=$('rm3100Progress');if(progress)progress.style.width=`${Math.max(0,Math.min(100,+s.coverage_pct||0))}%`;
  const faceBox=$('rm3100FaceGrid');if(faceBox)faceBox.innerHTML=RM3100_FACES.map(([k,label])=>`<div class="rm3100-face-card ${(counts[k]||0)>=RM3100_FACE_MIN?'done':''} ${current===k&&s.running?'active':''}"><b>${escapeHtml(label)}</b><span>${counts[k]||0}</span><small>samples</small></div>`).join('');
  qa('[data-rm-step]').forEach(el=>{const k=el.dataset.rmStep,done=k==='REVIEW'?s.status==='PASS':(+counts[k]||0)>=RM3100_FACE_MIN,active=s.running?k===current:((k===next)||(k==='REVIEW'&&next==='REVIEW'));el.classList.toggle('done',done);el.classList.toggle('active',active)});
  const activeFace=s.running?current:(next==='REVIEW'?'REVIEW':next),faceLabel=RM3100_FACES.find(x=>x[0]===activeFace)?.[1]||activeFace;setText('rm3100InstructionTitle',activeFace==='REVIEW'?'Coverage minimum selesai — STOP & FIT':`${s.running?'Sedang merekam':'Berikutnya'}: ${faceLabel}`);setText('rm3100InstructionText',s.running?`${rm3100FaceInstruction(activeFace)} Setelah satu putaran penuh dan ≥${RM3100_FACE_MIN} sample, tekan NEXT ORIENTATION.`:(s.instruction||'Tekan START FULL 3D untuk memulai recorder.'));
  const cube=$('rm3100Cube');if(cube){const roll=Number.isFinite(+s.roll_deg)?+s.roll_deg:0,pitch=Number.isFinite(+s.pitch_deg)?+s.pitch_deg:0;cube.style.setProperty('--rm-roll',`${roll}deg`);cube.style.setProperty('--rm-pitch',`${pitch}deg`)}
  for(const [p,o] of [['Wire',wire],['Cal',cal]]){const ready=Number.isFinite(+o.norm_ut);setText(`rm3100${p}X`,ready?`${rm3100Fmt(o.x_ut,2)} µT`:(p==='Cal'?'WAIT FULL-3D':'--'));setText(`rm3100${p}Y`,ready?`${rm3100Fmt(o.y_ut,2)} µT`:(p==='Cal'?'WAIT FULL-3D':'--'));setText(`rm3100${p}Z`,ready?`${rm3100Fmt(o.z_ut,2)} µT`:(p==='Cal'?'WAIT FULL-3D':'--'));setText(`rm3100${p}Norm`,ready?`${rm3100Fmt(o.norm_ut,2)} µT`:(p==='Cal'?'WAIT FULL-3D':'--'))}
  setText('rm3100WireRate',streamHealthText('rm3100_mag'));setText('rm3100CalRate',streamHealthText('rm3100_mag_calibrated'));setText('rm3100HeadingOut',Number.isFinite(+heading.yaw_rad)?`${rm3100Fmt(deg(heading.yaw_rad),2)}°`:'--');setText('rm3100Owner',getPath(obj('magnetic_heading_status'),'cal_owner')||String(configValue?.('mag_heading','mag_heading_fusion.ros__parameters.rm3100_calibration_owner')??'--'));
  const host=fit?.host_ros||{},ap=fit?.ardupilot_equivalent||{},val=fit?.validation||{},apLive=rm3100ApState();const bias=host.bias_xyz_ut||[];setText('rm3100Bias',bias.length===3?bias.map(v=>rm3100Fmt(v,4)).join(' / ')+' µT':'--');setText('rm3100Validation',fit?`CV ${rm3100Fmt(val.corrected_norm_cv,4)} • cond ${rm3100Fmt(val.matrix_condition,2)} • coverage ${rm3100Fmt(fit.coverage_pct,1)}%`:'Belum ada fit 3D');setText('rm3100ApReadback',apLive.state);
  const m=host.matrix_3x3||[];for(let i=0;i<9;i++)setText(`rm3100M${i}`,m.length===9?rm3100Fmt(m[i],6):'--');const apBody=$('rm3100ApParams');if(apBody){const keys=['COMPASS_OFS_X_mG','COMPASS_OFS_Y_mG','COMPASS_OFS_Z_mG','COMPASS_DIA_X','COMPASS_DIA_Y','COMPASS_DIA_Z','COMPASS_ODI_X','COMPASS_ODI_Y','COMPASS_ODI_Z','COMPASS_ORIENT','COMPASS_DEV_ID'];apBody.innerHTML=keys.map(k=>{const liveKey=k.replace('_mG',''),lv=apLive.params[liveKey],live=(lv==='NOT_FOUND'?'UNSUPPORTED':rm3100Fmt(lv,6)),hostVal=(liveKey==='COMPASS_ORIENT'||liveKey==='COMPASS_DEV_ID')?'--':rm3100Fmt(ap[k],6);return`<tr><td>${escapeHtml(liveKey)}</td><td>${escapeHtml(hostVal)}</td><td>${escapeHtml(live)}</td></tr>`}).join('')}
  setText('rm3100RawSemantics',s.raw_semantics||s.wire_semantics||'DroneCAN/AP_Periph pre-ROS correction field');setText('rm3100CsvPath',s.csv_path||'--');setText('rm3100FitPath',s.fit_path||'--');
  const pre=$('rm3100PreflightStep');if(pre){pre.classList.toggle('done',preflight);pre.classList.toggle('active',!preflight&&!s.running)}setText('rm3100PreflightReason',preflight?'PRE-FLIGHT SAFE • RM3100 online • kendaraan/command netral':(`PRE-FLIGHT LOCKED • ${s.sensor_online?'RM3100 online':'RM3100 offline'} • ${s.stationary_reason||'status kendaraan belum tersedia'}`));
  const start=$('rm3100StartBtn'),nextBtn=$('rm3100NextBtn'),stop=$('rm3100StopBtn'),stage=$('rm3100StageBtn');if(start)start.disabled=!!s.running||!preflight;if(nextBtn){const last=current===RM3100_FACES[RM3100_FACES.length-1][0];nextBtn.disabled=!s.running||(+counts[current]||0)<RM3100_FACE_MIN||last;nextBtn.textContent=last?'READY → STOP & FIT':'NEXT ORIENTATION'}if(stop)stop.disabled=!s.running;if(stage)stage.disabled=!!s.running||!fit?.valid||!apLive.complete||!apLive.identity;drawRm3100XY();drawRm3100Norm();
}
if($('rm3100StartBtn'))$('rm3100StartBtn').onclick=()=>confirmModal('Start RM3100 Full 3D','Mulai perekaman magnetometer. Pastikan kendaraan/sensor aman untuk mengikuti seluruh orientasi.',startRm3100Calibration);
if($('rm3100StopBtn'))$('rm3100StopBtn').onclick=()=>confirmModal('Stop & fit RM3100','Hentikan recording dan hitung full 3D ellipsoid fit sekarang?',stopRm3100Calibration);
if($('rm3100NextBtn'))$('rm3100NextBtn').onclick=nextRm3100Face;
qa('[data-rm-step]').forEach(el=>{const face=el.dataset.rmStep;if(face&&face!=='REVIEW')el.onclick=()=>setRm3100Face(face)});
if($('rm3100StageBtn'))$('rm3100StageBtn').onclick=stageRm3100Full3d;
if($('rm3100ClearPlotBtn'))$('rm3100ClearPlotBtn').onclick=()=>{rm3100Plot.raw.length=0;rm3100Plot.cal.length=0;rm3100Plot.norm.length=0;drawRm3100XY();drawRm3100Norm()};
setInterval(()=>pollRm3100Calibration(false),750);pollRm3100Calibration(true);