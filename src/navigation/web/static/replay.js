'use strict';

// Offline recorder replay. This module never sends ROS commands.
const replayStore={rows:[],headers:[],index:0,playing:false,raf:0,playWall:0,playData:0,fileName:'',duration:0};
const replayFlatPrefixes=['connected.','system.'];

function replayCsvRows(text){
  const rows=[];let row=[],field='',quoted=false;
  for(let i=0;i<text.length;i++){
    const c=text[i];
    if(quoted){if(c==='"'&&text[i+1]==='"'){field+='"';i++}else if(c==='"')quoted=false;else field+=c;continue}
    if(c==='"'){quoted=true;continue}
    if(c===','){row.push(field);field='';continue}
    if(c==='\n'){row.push(field.replace(/\r$/,''));field='';if(row.some(v=>v!==''))rows.push(row);row=[];continue}
    field+=c;
  }
  if(field||row.length){row.push(field.replace(/\r$/,''));rows.push(row)}
  return rows;
}

function replayScalar(v){
  if(v===''||v==null)return null;
  if(v==='true')return true;if(v==='false')return false;
  const n=Number(v);return Number.isFinite(n)?n:v;
}

function replayTimeOf(row,i){
  for(const k of ['elapsed_s','t','phase_sec']){const n=Number(row[k]);if(Number.isFinite(n))return Math.max(0,n)}
  const u=Number(row.unix_time);if(Number.isFinite(u)){const u0=Number(replayStore.rows[0]?.raw?.unix_time);return Number.isFinite(u0)?Math.max(0,u-u0):i*.2}
  return i*.2;
}
function replaySetDeep(root,path,value){
  let o=root;
  for(let i=0;i<path.length-1;i++){const k=path[i];if(!o[k]||typeof o[k]!=='object')o[k]={};o=o[k]}
  o[path.at(-1)]=value;
}

function replayApplyRow(index){
  if(!replayStore.rows.length)return;
  const i=Math.max(0,Math.min(replayStore.rows.length-1,index|0)),entry=replayStore.rows[i],row=entry.raw,now=Date.now();
  replayStore.index=i;
  for(const [key,rawValue] of Object.entries(row)){
    if(['time_iso','elapsed_s','subsystem','section_id','source_experiment_id','section_label','variation','condition'].includes(key))continue;
    const value=replayScalar(rawValue);
    if(replayFlatPrefixes.some(p=>key.startsWith(p))||!key.includes('.')){state[key]=value;updated[key]=now;continue}
    const parts=key.split('.'),root=parts.shift();
    if(!state[root]||typeof state[root]!=='object'||Array.isArray(state[root]))state[root]={};
    replaySetDeep(state[root],parts,value);updated[root]=now;
  }
  if(entry.timeIso)state.replay_time_iso=entry.timeIso;
  renderReplayUi();queueRender();
}

function replayFormatTime(sec){const s=Math.max(0,+sec||0),m=Math.floor(s/60),r=s-m*60;return `${String(m).padStart(2,'0')}:${r.toFixed(3).padStart(6,'0')}`}

function renderReplaySignals(row){
  const box=$('replaySignals');if(!box)return;
  const wanted=['subsystem','section_id','variation','condition','ekf_global.x','ekf_global.y','ekf_global.yaw','ekf_global.v','ekf_global.w','esc_odom.v','esc_odom.w','gnss_quality.sat','gnss_quality.dop','gnss_quality.hacc_m','imu.gz','perception_performance.fps'];
  const pairs=wanted.filter(k=>row[k]!=null&&row[k]!=='').map(k=>[k,row[k]]);
  box.innerHTML=pairs.length?pairs.map(([k,v])=>`<div><span>${escapeHtml(k)}</span><b>${escapeHtml(String(v))}</b></div>`).join(''):'<div class="empty-state">No standard replay signals in this row.</div>';
}
function renderReplayUi(){
  const e=replayStore.rows[replayStore.index],sec=e?.time||0,slider=$('replayTimeline');
  if(slider){slider.max=String(Math.max(0,replayStore.rows.length-1));slider.value=String(replayStore.index)}
  setText('replayTime',`${replayFormatTime(sec)} / ${replayFormatTime(replayStore.duration)}`);
  if(e)renderReplaySignals(e.raw);
  setText('replayStatusDetail',e?JSON.stringify({file:replayStore.fileName,row:replayStore.index+1,rows:replayStore.rows.length,time_s:sec,time_iso:e.timeIso||null,mode:replayMode?'REPLAY_LOCKED':'LOADED_NOT_ACTIVE'},null,2):'No replay loaded.');
}

function replayStop(){replayStore.playing=false;if(replayStore.raf)cancelAnimationFrame(replayStore.raf);replayStore.raf=0;const b=$('replayPlayBtn');if(b)b.textContent='▶ Play'}

function replayTick(now){
  if(!replayStore.playing)return;
  const speed=+$('replaySpeed')?.value||1,target=replayStore.playData+(now-replayStore.playWall)/1000*speed;
  let i=replayStore.index;
  while(i+1<replayStore.rows.length&&replayStore.rows[i+1].time<=target)i++;
  if(i!==replayStore.index)replayApplyRow(i);
  if(i>=replayStore.rows.length-1){replayStop();return}
  replayStore.raf=requestAnimationFrame(replayTick);
}

function replayTogglePlay(){
  if(!replayMode)return toast('Enter Replay dulu.',true);
  if(replayStore.playing){replayStop();return}
  replayStore.playing=true;replayStore.playWall=performance.now();replayStore.playData=replayStore.rows[replayStore.index]?.time||0;
  $('replayPlayBtn').textContent='Ⅱ Pause';replayStore.raf=requestAnimationFrame(replayTick);
}

function replaySetEnabled(enabled){
  for(const id of ['replayPlayBtn','replayStepBackBtn','replayStepForwardBtn','replaySpeed','replayTimeline','replayOpenMapBtn'])if($(id))$(id).disabled=!enabled;
}
async function replayLoadFile(file){
  replayStop();const text=await file.text(),matrix=replayCsvRows(text);if(matrix.length<2)throw new Error('CSV tidak memiliki data');
  const headers=matrix[0],rawRows=matrix.slice(1).map(cols=>Object.fromEntries(headers.map((h,i)=>[h,cols[i]??''])));
  replayStore.headers=headers;replayStore.rows=rawRows.map((raw,i)=>({raw,time:0,timeIso:raw.time_iso||''}));
  replayStore.rows.forEach((e,i)=>e.time=replayTimeOf(e.raw,i));replayStore.rows.sort((a,b)=>a.time-b.time);replayStore.fileName=file.name;replayStore.index=0;replayStore.duration=replayStore.rows.at(-1)?.time||0;
  $('replayEnterBtn').disabled=false;replaySetEnabled(false);$('replayStateChip').textContent='FILE READY';$('replayStateChip').className='status-chip';
  setText('replayMeta',`${file.name} • ${replayStore.rows.length} rows • ${replayStore.headers.length} columns • ${replayStore.duration.toFixed(2)} s`);renderReplayUi();
}

function replayEnter(){
  if(!replayStore.rows.length)return toast('Load recorder CSV terlebih dahulu.',true);
  replayStop();replayMode=true;if(sse){sse.close();sse=null}document.body.classList.add('replay-mode');setRealtimeUi('offline');
  $('replayEnterBtn').disabled=true;$('replayExitBtn').disabled=false;replaySetEnabled(true);$('replayStateChip').textContent='REPLAY LOCKED';$('replayStateChip').className='status-chip waiting';
  replayApplyRow(replayStore.index);toast('REPLAY MODE aktif • command/write locked');
}

async function replayExit(){
  replayStop();replayMode=false;document.body.classList.remove('replay-mode');$('replayEnterBtn').disabled=!replayStore.rows.length;$('replayExitBtn').disabled=true;replaySetEnabled(false);$('replayStateChip').textContent='LIVE RESYNC';$('replayStateChip').className='status-chip waiting';
  connectSse();await fullState(true);$('replayStateChip').textContent='FILE READY';$('replayStateChip').className='status-chip';queueRender();toast('Replay selesai • live state restored');
}
$('replayFile').onchange=async e=>{const file=e.target.files?.[0];if(!file)return;try{await replayLoadFile(file)}catch(err){toast('Replay load gagal: '+err.message,true)}};
$('replayEnterBtn').onclick=replayEnter;$('replayExitBtn').onclick=replayExit;$('replayPlayBtn').onclick=replayTogglePlay;
$('replayStepBackBtn').onclick=()=>{replayStop();replayApplyRow(replayStore.index-1)};$('replayStepForwardBtn').onclick=()=>{replayStop();replayApplyRow(replayStore.index+1)};
$('replayTimeline').oninput=e=>{replayStop();replayApplyRow(+e.target.value)};
$('replayOpenMapBtn').onclick=()=>{if(!replayMode)return toast('Enter Replay dulu.',true);activatePage('navigation',false,'navigation');drawMap()};
$('replaySpeed').onchange=()=>{if(replayStore.playing){replayStore.playWall=performance.now();replayStore.playData=replayStore.rows[replayStore.index]?.time||0}};

replaySetEnabled(false);renderReplayUi();
