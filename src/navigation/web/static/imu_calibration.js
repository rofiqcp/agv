'use strict';
let yahboomCalStatus={};
const HEADING360_STOPS=9;
const HEADING360_LABELS={CW:['N','NE','E','SE','S','SW','W','NW','N'],CCW:['N','NW','W','SW','S','SE','E','NE','N']};
function heading360Root(j=yahboomCalStatus){return j?.fit?.heading_360_calibration||{}}
function imuCalFitRoot(j=yahboomCalStatus){return heading360Root(j)?.yahboom||j?.fit?.yahboom_mag_planar_calibration||{}}
function neoCalFitRoot(j=yahboomCalStatus){return heading360Root(j)?.neo3||j?.fit?.neo3_planar_candidate||{}}
function imuWizardSetChip(text,kind='waiting'){const e=$('imuWizardChip');if(!e)return;e.textContent=text||'WAIT';e.className='status-chip '+(kind==='pass'?'':kind==='fail'?'danger':'waiting')}
function stopCompassDeg(i,direction){const step=45*i;return ((direction==='CCW'?-step:step)%360+360)%360}
function ensureImuWaypoints(direction='CW'){
  const ring=$('imuCalRing');if(!ring)return;let pts=qa('.imu-waypoint',ring);
  if(pts.length!==HEADING360_STOPS){pts.forEach(x=>x.remove());for(let i=0;i<HEADING360_STOPS;i++){const d=document.createElement('div');d.className='imu-waypoint';d.dataset.index=String(i);d.textContent=String(i+1);ring.append(d)}pts=qa('.imu-waypoint',ring)}
  const labels=HEADING360_LABELS[direction]||HEADING360_LABELS.CW;
  pts.forEach((n,i)=>{const deg=stopCompassDeg(i,direction),a=deg*Math.PI/180,r=i===8?34:42;n.style.left=(50+r*Math.sin(a))+'%';n.style.top=(50-r*Math.cos(a))+'%';n.title=`${i+1}/9 • ${labels[i]} • ${deg.toFixed(0)}°${i===8?' • CLOSURE':''}`;n.classList.toggle('closure',i===8)})
}
function calibrationPass(j){const h=heading360Root(j);return h.ready_for_stage2===true&&h.valid===true&&h.yahboom?.valid===true&&h.neo3?.valid===true&&h.gyro?.pass===true&&h.cross_sensor?.pass===true}
function num(v,d=2){return Number.isFinite(+v)?fmt(+v,d):'--'}
function renderYahboomCalibration(j=yahboomCalStatus){
  if(!$('yahboomCalibrationWizard'))return;
  const live=obj('imu_raw_sensor'),profile=obj('imu_profile_status'),h=heading360Root(j),yf=imuCalFitRoot(j),nf=neoCalFitRoot(j),dir=String(j.direction||h.direction||yf.direction||'CW').toUpperCase();
  const seg=+j.segment||0,next=Math.max(1,Math.min(HEADING360_STOPS,+j.next_segment||Math.min(HEADING360_STOPS,seg+1))),status=String(j.status||'IDLE').toUpperCase();ensureImuWaypoints(dir);
  const rawMount=String(j.mounting_state||live.mounting_state||'WAIT'),mountOk=(j.mounting_ok!==undefined?j.mounting_ok:live.mounting_ok)===true;
  setText('imuMountingState',rawMount);setText('imuMountingGravity',Number.isFinite(+live.acc_norm)?fmt(live.acc_norm,3)+' m/s²':Number.isFinite(+j.acc_norm_mps2)?fmt(j.acc_norm_mps2,3)+' m/s²':'--');setText('imuWizardGyro',Number.isFinite(+j.gyro_z_rps)?fmt(j.gyro_z_rps,4)+' rad/s':Number.isFinite(+live.gz)?fmt(live.gz,4)+' rad/s':'--');setText('imuWizardStatic',j.static===true?'STATIC':j.running?'MOVING / WAIT':'--');
  const yah=obj('imu_mag_heading'),neo=obj('neo3_mag_heading');setText('imuWizardYahHeading',Number.isFinite(+yah.yaw_rad)?fmt(deg(yah.yaw_rad),2)+'°':'--');setText('imuWizardNeoHeading',Number.isFinite(+j.neo_heading_deg)?fmt(j.neo_heading_deg,2)+'°':Number.isFinite(+neo.yaw_rad)?fmt(deg(neo.yaw_rad),2)+'°':'--');
  setText('imuWizardPosition',`${seg} / ${HEADING360_STOPS}`);setText('imuWizardDirection',`${dir} • NORTH REF`);setText('imuWizardInstruction',j.instruction||'Hadapkan depan AGV tepat ke UTARA 0°, lalu pilih arah putaran.');
  const actual=Number.isFinite(+j.actual_relative_deg)?+j.actual_relative_deg:0,targetComp=Number.isFinite(+j.target_compass_deg)?+j.target_compass_deg:stopCompassDeg(next-1,dir),targetRel=45*(next-1),label=j.target_label||(HEADING360_LABELS[dir]||HEADING360_LABELS.CW)[next-1];
  setText('imuWizardHeading',`Target ${label} ${targetComp.toFixed(0)}° • progress ${num(actual,1)}° / ${targetRel}°`);if($('imuWizardProgress'))$('imuWizardProgress').style.width=`${Math.max(0,Math.min(100,(+j.progress||seg/HEADING360_STOPS)*100))}%`;
  if($('imuVehicleArrow'))$('imuVehicleArrow').style.transform=`rotate(${actual}deg)`;if($('imuTargetArrow'))$('imuTargetArrow').style.transform=`rotate(${targetComp}deg)`;
  qa('.imu-waypoint',$('imuCalRing')).forEach((n,i)=>{n.classList.toggle('done',i<seg);n.classList.toggle('active',i===next-1&&!!j.running);n.classList.toggle('error',status.includes('MISMATCH')||status==='FAIL')});
  const yv=yf.validation||{},nv=nf.validation||{};setText('imuWizardRms',`Y ${num(yv.fit_rms_error_deg??yv.rms_error_deg,2)}° • N ${num(nv.fit_rms_error_deg??nv.rms_error_deg,2)}°`);setText('imuWizardMax',`Y ${num(yv.fit_max_abs_error_deg??yv.max_abs_error_deg,2)}° • N ${num(nv.fit_max_abs_error_deg??nv.max_abs_error_deg,2)}°`);
  const ps=String(profile.state||'WAIT').toUpperCase();setText('imuProfileState',ps);setText('imuProfileDetail',profile.detail||`${profile.baud||921600} baud • RRATE ${profile.rrate??8} • AXIS6 ${profile.axis6??1}`);
  const valid=calibrationPass(j);if(status==='PASS_STAGE1'||valid)imuWizardSetChip('STAGE-1 PASS','pass');else if(status==='FAIL'||status.includes('MISMATCH'))imuWizardSetChip(status,'fail');else if(j.running)imuWizardSetChip(`${dir} ${seg}/${HEADING360_STOPS}`,'waiting');else imuWizardSetChip('IDLE','waiting');
  const gate=$('imuWizardGateDetail');if(gate){gate.className='imu-gate-box '+(valid&&mountOk?'pass':status==='FAIL'||status.includes('MISMATCH')?'fail':'');gate.textContent=valid&&mountOk?`STAGE-1 PASS • 9/9 • Yah closure ${num(yv.closure_mean_error_deg,2)}° • NEO closure ${num(nv.closure_mean_error_deg,2)}° • cross RMS ${num(h.cross_sensor?.rms_delta_deg,2)}° • gyro closure ${num(h.gyro?.closure_error_deg,2)}° • YAML/runtime LOCKED.`:(j.mounting_detail||live.mounting_detail||'Fail-closed: N→8 arah 45°→N closure, dual MAG fit, gyro ±360°, RMS<3°, max<5°. Tahap 1 tidak menulis YAML.')}
  if($('imuCalApply')){$('imuCalApply').disabled=!(valid&&mountOk&&!j.running);$('imuCalApply').textContent='Generate Stage-1 Preview'}if($('imuCalStartCw'))$('imuCalStartCw').disabled=!!j.running;if($('imuCalStartCcw'))$('imuCalStartCcw').disabled=!!j.running;if($('imuCalStop'))$('imuCalStop').disabled=!j.running;
}
async function pollYahboomCalibration(){try{const r=await readRequest('/api/imu/calibration/status',{timeoutMs:2200}),j=await r.json();if(!r.ok)throw new Error(j.message||`imu calibration HTTP ${r.status}`);yahboomCalStatus=j;renderYahboomCalibration(j)}catch(e){imuWizardSetChip(e.message?.includes('timeout')?'RETRYING':'UNAVAILABLE','waiting');const gate=$('imuWizardGateDetail');if(gate){gate.className='imu-gate-box';gate.textContent=`Heading calibration status unavailable • ${e.message||'request failed'}`}}}
async function setYahboomOptimalProfile(){const b=$('imuOptimalProfileBtn');if(b){b.disabled=true;b.textContent='SETTING 921600...'}try{const j=await post('/api/imu/profile/optimal',{},false);if(j?.ok)toast('Profil optimal dikirim. Verifikasi live /imu/profile_status...');setTimeout(fullState,500)}finally{if(b)setTimeout(()=>{b.disabled=false;b.textContent='Set Optimal 921600'},900)}}
async function startYahboomCalibration(direction){const j=await post('/api/imu/calibration/start',{direction},false);if(j?.ok){yahboomCalStatus={running:true,direction,status:'RUNNING',segment:0,stop_count:9,next_segment:1,target_label:'N',target_compass_deg:0,instruction:'Hadapkan depan AGV tepat ke UTARA 0° dan tahan diam'};renderYahboomCalibration(yahboomCalStatus);setTimeout(pollYahboomCalibration,300)}}
async function stopYahboomCalibration(){await post('/api/imu/calibration/stop',{},false);setTimeout(pollYahboomCalibration,250)}
async function previewHeading360Calibration(){try{const r=await writeRequest('/api/imu/calibration/proposal',{}),j=await r.json();if(!r.ok||!j.ok)throw new Error(j.message||`HTTP ${r.status}`);if(j.stage1_only!==true||j.apply_locked!==true||j.yaml_write!==false||j.runtime_write!==false)throw new Error('Stage-1 safety lock response invalid');const n=(j.proposal_items||[]).length;toast(`Stage-1 preview PASS • ${n} kandidat parameter • APPLY/YAML LOCKED sampai Tahap 2.`)}catch(e){toast('Stage-1 preview gagal: '+e.message,true)}finally{setTimeout(pollYahboomCalibration,400)}}
if($('imuOptimalProfileBtn'))$('imuOptimalProfileBtn').onclick=setYahboomOptimalProfile;if($('imuCalStartCw'))$('imuCalStartCw').onclick=()=>startYahboomCalibration('CW');if($('imuCalStartCcw'))$('imuCalStartCcw').onclick=()=>startYahboomCalibration('CCW');if($('imuCalStop'))$('imuCalStop').onclick=stopYahboomCalibration;if($('imuCalApply'))$('imuCalApply').onclick=previewHeading360Calibration;

let labRenderQueued=false,lastLabRenderAt=0,lastLabSlowAt=0;
function queueLabWorkbench(force=false){if(activePage!=='experiments'||document.hidden||!selectedExperiment)return;if(labRenderQueued)return;labRenderQueued=true;const delay=force?0:Math.max(0,125-(performance.now()-lastLabRenderAt)),run=()=>requestAnimationFrame(()=>{labRenderQueued=false;if(activePage!=='experiments'||document.hidden||!selectedExperiment)return;lastLabRenderAt=performance.now();renderLabWorkbench(force)});delay>1?setTimeout(run,delay):run()}
function renderLabWorkbench(force=false){if(activePage!=='experiments'||!selectedExperiment)return;captureLabTelemetry();uiPerf.labFast++;drawAllExperimentCharts();renderContextVisual();const now=performance.now();if(force||now-lastLabSlowAt>=1000){lastLabSlowAt=now;uiPerf.labSlow++;renderExperimentTable();renderSourceAudit();renderPerceptionEngineering();renderTestPreflight()}}
// Stage 5: acquisition remains 1 Hz deterministic; expensive DOM/table qualification work is 1 Hz and charts/context max 8 Hz while visible.
let yahboomIdlePoll=0,recorderIdlePoll=0;
setInterval(()=>{if(document.hidden)return;if(activePage==='calibration'||yahboomCalStatus.running||(++yahboomIdlePoll%7===0))pollYahboomCalibration()},700);
setInterval(()=>{if(document.hidden)return;if(activePage==='experiments'||recordStartedMs||(++recorderIdlePoll%5===0))pollRecorder()},1000);

