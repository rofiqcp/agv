/* GUI-only BAB IV perception acquisition aligned with the latest thesis draft. */
const PER_BAB4_IDS=new Set(['F4.1','F4.2','F4.3','F4.4']);
const perBab4={perf:{active:false,count:0,lastUpdate:0,gapStart:0,started:0,stopBusy:false,timer:null},selectedCandidate:0};
const perRunField=(key,label,kind='text',placeholder='',options=[])=>({key,label,kind,placeholder,options,group:'Protokol BAB IV',yamlFileKey:'',yamlPath:'',isGroundTruth:false,locked:false});
const perRo=(label,path,kind='float')=>({label,yamlFileKey:'perception',yamlPath:`perception.ros__parameters.${path}`,kind:'yaml_readonly',group:'Konfigurasi Aktif',locked:true});
function perBab4Is(id=selectedExperiment?.id){return currentExp==='perception'&&PER_BAB4_IDS.has(String(id||''))}
function perBab4DetectionFields(){return [perRo('Raw confidence threshold','confidence_threshold'),perRo('NMS IoU threshold','iou_threshold'),perRo('Minimum obstacle confidence','minimum_obstacle_confidence'),perRo('CPU inference target FPS','cpu_inference_fps'),perRo('CPU threads','cpu_threads','int')]}
function perBab4LaneFields(){return [perRo('Warning gap px','lane_corridor_warning_gap_px'),perRo('Touch margin px','lane_corridor_touch_margin_px'),perRo('Release gap px','lane_corridor_release_gap_px'),perRo('Left top X ratio','lane_corridor_left_top_x_ratio'),perRo('Left bottom X ratio','lane_corridor_left_bottom_x_ratio'),perRo('Right top X ratio','lane_corridor_right_top_x_ratio'),perRo('Right bottom X ratio','lane_corridor_right_bottom_x_ratio')]}
function perBab4PerfFields(){return [perRo('RGB width','rgb_width','int'),perRo('RGB height','rgb_height','int'),perRo('Camera FPS request','fps','int'),perRo('CPU inference target FPS','cpu_inference_fps'),perRo('CPU threads','cpu_threads','int')]}
function patchPerceptionBab4Catalog(){
  const list=experiments.perception||[],byId=id=>list.find(x=>x.id===id);let x;
  if((x=byId('F4.1')))Object.assign(x,{groupId:'FINAL-4.1',groupTitle:'FINAL BAB IV — 4.1 Implementasi Sistem Persepsi Visual',section:'FINAL 4.1 Implementasi Sistem Persepsi Visual — audit read-only',reportMode:true,bab4Protocol:'implementation',parameterFields:[perRo('RGB width','rgb_width','int'),perRo('RGB height','rgb_height','int'),perRo('Camera FPS request','fps','int'),perRo('CPU inference target FPS','cpu_inference_fps'),perRo('CPU threads','cpu_threads','int'),perRo('Web preview FPS','web_preview_fps')],tableColumns:[],tableNames:[],graphCaptions:[],graphs:[],liveSeries:{'Camera capture FPS':'perception_performance.capture_fps','Pipeline FPS':'perception_performance.pipeline_fps'}});
  if((x=byId('F4.2')))Object.assign(x,{groupId:'FINAL-4.2',groupTitle:'FINAL BAB IV — 4.2 Pengujian Object Detection YOLOPv2',section:'FINAL 4.2 Object Detection — Human + Motorcycle • 1–5 m • 20 percobaan/jarak',reportMode:true,bab4Protocol:'detection',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4DetectionFields(),tableColumns:[['Trial','Target Fisik','Jarak GT [m]','Ulangan','Frame timestamp','Backend','Raw Candidate','Confidence Kandidat','Hasil Spasial','Raw Class ID(s)','Catatan Visual','Evaluasi Otomatis','Evidence Frame'],['Target Fisik','Jarak [m]','Percobaan','Berhasil','Gagal','Detection Rate [%]','Error [%]','Confidence rata-rata']],tableNames:['Raw Percobaan Object Detection','Rekap Detection Rate dan Confidence'],graphCaptions:[],graphs:[],liveSeries:{'Raw detections':'raw_detections.count','Mean confidence':'raw_detections.mean_confidence','Pipeline FPS':'perception_performance.pipeline_fps'}});
  if((x=byId('F4.3')))Object.assign(x,{groupId:'FINAL-4.3',groupTitle:'FINAL BAB IV — 4.3 Pengujian Lane Safety',section:'FINAL 4.3 Lane Safety — Abu-abu / Hijau / Kuning / Merah • 20 percobaan/kondisi',reportMode:true,bab4Protocol:'lane',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4LaneFields(),tableColumns:[['Trial','Kondisi Acuan','Sisi Uji','Ulangan','Timestamp','Drivable/Mask Valid','Left Gap [px]','Right Gap [px]','Left Color','Right Color','Latch L','Latch R','Recommendation Aktual','Expected','Hasil'],['Kondisi Acuan','Percobaan','Benar','Salah','Success Rate [%]']],tableNames:['Raw Percobaan Lane Safety','Rekap Success Rate Lane Safety'],graphCaptions:[],graphs:[],liveSeries:{'Left gap':'lane_state.corridor.left_gap_px','Right gap':'lane_state.corridor.right_gap_px','Lane valid':'lane_state.valid','Drivable valid':'lane_state.drivable_valid'}});
  if((x=byId('F4.4')))Object.assign(x,{groupId:'FINAL-4.4',groupTitle:'FINAL BAB IV — 4.4 Pengujian Performa Real-Time',section:'FINAL 4.4 Performa CPU — 3 kondisi visual • 60 detik/kondisi',reportMode:true,bab4Protocol:'performance',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4PerfFields(),tableColumns:[['Kondisi','Durasi [s]','Frame selesai diterima','FPS hitung','Mean pipeline FPS EMA','Mean inference [ms]','Mean process total [ms]','Mean frame age [ms]','Mean subscriber latency [ms]','SSE gap','Backend','Status data']],tableNames:['Hasil Pengujian Performa Mini PC CPU'],graphCaptions:['Pipeline FPS selama 60 detik','Inference dan process total','Frame age dan subscriber latency'],graphs:[{type:'time_series',series:['Pipeline FPS'],xLabel:'Time [s]',yLabel:'FPS'},{type:'time_series',series:['Inference ms','Process total ms'],xLabel:'Time [s]',yLabel:'Time [ms]'},{type:'time_series',series:['Frame age ms','Subscriber latency ms'],xLabel:'Time [s]',yLabel:'Latency [ms]'}],liveSeries:{'Pipeline FPS':'perception_performance.pipeline_fps','Inference ms':'perception_performance.inference_ms','Process total ms':'perception_performance.pipeline_ms_per_frame','Frame age ms':'perception_performance.frame_age_ms','Subscriber latency ms':'derived.per_bab4_latency_ms','Processed event':'raw_detections.count'}});
}
function perBab4PatchWorkspace(){/* FINAL BAB IV now lives directly in the left sidebar with other perception tasks. */}
const perBab4BaseEffectiveExperimentList=effectiveExperimentList;
effectiveExperimentList=function(){return perBab4BaseEffectiveExperimentList()};
const perBab4BaseActivateWorkspaceTab=activateWorkspaceTab;
activateWorkspaceTab=function(id){
  const r=perBab4BaseActivateWorkspaceTab(id);
  document.body.classList.remove('per-bab4-final-mode');delete document.body.dataset.bab4;
  if(activeDomain==='perception'&&id==='tune'){
    currentExp='perception';renderExperimentList();const list=effectiveExperimentList();
    if(list.length&&!list.some(x=>x.id===selectedExperiment?.id))selectExperimentById(list[0].id);
    else if(selectedExperiment)renderSelectedExperiment();
  }
  return r;
};
function perBab4ServerNow(){return Date.now()-(Number.isFinite(serverDelta)?serverDelta:0)}
function perBab4LatencyMs(){const ns=Number(obj('lane_state')?.stamp_ns);if(!Number.isFinite(ns)||ns<1e15)return undefined;const v=perBab4ServerNow()-ns/1e6;return Number.isFinite(v)&&v>=0&&v<5000?v:undefined}
const perBab4BaseResolve=resolveMetricPath;
resolveMetricPath=function(path){if(path==='derived.per_bab4_latency_ms')return perBab4LatencyMs();return perBab4BaseResolve(path)};
const perBab4BasePhaseGate=phaseGate;
phaseGate=function(){if(perBab4Is()&&selectedExperiment?.reportMode)return {ok:true,reason:'BAB IV persepsi standalone: tidak memerlukan qualification aktuator/Nav2'};return perBab4BasePhaseGate()};
const perBab4BaseTableAxis=tableHasTimeAxis;
tableHasTimeAxis=function(){if(perBab4Is()&&selectedExperiment?.bab4NoTimeAxis)return false;return perBab4BaseTableAxis()};
const perBab4BaseCaptureTemplateRows=captureTemplateRows;
captureTemplateRows=function(sec){if(perBab4Is()&&selectedExperiment?.bab4ManualCapture)return;return perBab4BaseCaptureTemplateRows(sec)};
function perBab4InferenceOn(){const p=obj('perception_performance');return p.inference_enabled===true||String(p.backend||'').toLowerCase()==='cpu'}
const perBab4BasePreflight=testPreflightStatus;
testPreflightStatus=function(){
  const p=perBab4BasePreflight();if(!perBab4Is()||!selectedExperiment?.reportMode)return p;
  const items=p.items.map(i=>i.label==='Commissioning prerequisite'?{...i,ok:true,critical:false,detail:'Tidak diperlukan untuk pengujian visual standalone BAB IV'}:i);
  const add=(label,ok,detail)=>{if(!items.some(i=>i.label===label))items.push({label,ok:!!ok,detail,critical:true})};
  add('YOLOPv2 CPU inference',perBab4InferenceOn(),perBab4InferenceOn()?'inference aktif':'aktifkan YOLOPv2 pada tab Live');
  if(selectedExperiment.id==='F4.2')add('Raw detection stream',channelFresh('raw_detections',2.5),`age ${Number.isFinite(age('raw_detections'))?age('raw_detections').toFixed(2):'--'} s`);
  if(selectedExperiment.id==='F4.3')add('Lane Safety stream',channelFresh('lane_state',2.5),`age ${Number.isFinite(age('lane_state'))?age('lane_state').toFixed(2):'--'} s`);
  if(selectedExperiment.id==='F4.4')add('Processed-frame event stream',channelFresh('raw_detections',2.5),`raw_detections age ${Number.isFinite(age('raw_detections'))?age('raw_detections').toFixed(2):'--'} s`);
  const bad=items.filter(i=>i.critical&&!i.ok);return {ok:bad.length===0,items,reason:bad.map(x=>x.label).join(', ')};
};
const perBab4BaseLoadExperiments=loadExperiments;
loadExperiments=async function(){await perBab4BaseLoadExperiments();patchPerceptionBab4Catalog();if(currentExp==='perception'){const keep=selectedExperiment?.id;renderExperimentList();if(keep&&findExperimentById(keep))selectExperimentById(keep);else{const list=effectiveExperimentList();if(list.length)selectExperimentById(list[0].id)}}};
function perBab4RawDetections(){const d=obj('raw_detections')?.detections;return Array.isArray(d)?d:[]}
function perBab4FrameTimestamp(){const u=updated.raw_detections||updated.lane_state||Date.now()-serverDelta;return Number.isFinite(+u)?new Date(+u+serverDelta).toISOString():new Date().toISOString()}
function perBab4YamlSnapshot(){return parameterSnapshotEntries().map(x=>({key:`${x.file}:${x.key}`,label:x.label,value:x.value}))}
function perBab4Push(tableIndex,values){const a=tableRunRows.get(tableIndex)||[];a.push({sec:Math.max(0,Math.floor((Date.now()-(recordStartedMs||Date.now()))/1000)),values,yaml:perBab4YamlSnapshot()});tableRunRows.set(tableIndex,a);renderExperimentTable();perBab4RenderProgress();perBab4DrawSummary()}
function perBab4Rows(){return tableRunRows.get(0)||[]}
function perBab4Get(id){return document.getElementById(id)}
function perBab4SelectedDetection(){const d=perBab4RawDetections(),i=Math.max(0,Math.min(d.length-1,+perBab4Get('perBab4Candidate')?.value||0));return d[i]}
function perBab4DetectionCapture(success){
  if(!recordStartedMs)return toast('Klik START dahulu agar satu sesi menyimpan seluruh trial.',true);const target=perBab4Get('perBab4Target')?.value||'human',dist=+(perBab4Get('perBab4Distance')?.value||1),rep=+(perBab4Get('perBab4Repeat')?.value||1),dets=perBab4RawDetections(),sel=perBab4SelectedDetection();
  if(success&&!sel)return toast('SUCCESS membutuhkan kandidat bbox yang dipilih.',true);const conf=success&&Number.isFinite(+sel?.score)?(+sel.score).toFixed(5):'--',classes=dets.map(d=>d.class_id).filter(v=>v!==undefined).join('|')||'--',backend=obj('raw_detections').backend||obj('perception_performance').backend||'--',note=perBab4Get('perBab4Note')?.value?.trim()||'';
  perBab4Push(0,[perBab4Rows().length+1,target,dist,rep,perBab4FrameTimestamp(),backend,dets.length,conf,success?'BERHASIL':'GAGAL',success?String(sel.class_id??'--'):classes,note]);perBab4AdvanceDetection(target,dist,rep);toast(`Trial ${target} ${dist} m #${rep}: ${success?'BERHASIL':'GAGAL'}`);
}
function perBab4AdvanceDetection(target,dist,rep){let n=rep+1,d=dist,t=target;if(n>20){n=1;d++;if(d>5){d=1;t=target==='human'?'motorcycle':'human'}}if(perBab4Get('perBab4Target'))perBab4Get('perBab4Target').value=t;if(perBab4Get('perBab4Distance'))perBab4Get('perBab4Distance').value=String(d);if(perBab4Get('perBab4Repeat'))perBab4Get('perBab4Repeat').value=String(n);if(perBab4Get('perBab4Note'))perBab4Get('perBab4Note').value='';perBab4RefreshLive()}
function perBab4LaneExpected(cond,side){if(cond==='gray')return {rec:'NO_MASK_EVIDENCE',left:'UNKNOWN',right:'UNKNOWN'};if(cond==='green')return {rec:'CLEAR',left:'GREEN',right:'GREEN'};if(cond==='yellow')return {rec:'WARNING',left:side==='left'?'YELLOW':'GREEN',right:side==='right'?'YELLOW':'GREEN'};return {rec:side==='left'?'RECENTER_RIGHT':'RECENTER_LEFT',left:side==='left'?'RED':'GREEN',right:side==='right'?'RED':'GREEN'}}
function perBab4LaneCapture(){
  if(!recordStartedMs)return toast('Klik START dahulu agar satu sesi menyimpan seluruh trial.',true);const cond=perBab4Get('perBab4LaneCondition')?.value||'gray',side=perBab4Get('perBab4LaneSide')?.value||'left',rep=+(perBab4Get('perBab4LaneRepeat')?.value||1),ls=obj('lane_state'),c=ls.corridor||{},exp=perBab4LaneExpected(cond,side),actual=String(c.recommendation||'--'),left=String(c.left_status||'--'),right=String(c.right_status||'--'),match=actual===exp.rec&&left===exp.left&&right===exp.right,label={gray:'Abu-abu',green:'Hijau',yellow:'Kuning',red:'Merah'}[cond];
  perBab4Push(0,[perBab4Rows().length+1,label,cond==='gray'||cond==='green'?'N/A':side,rep,perBab4FrameTimestamp(),String(ls.drivable_valid??ls.valid??'--'),Number.isFinite(+c.left_gap_px)?(+c.left_gap_px).toFixed(2):'--',Number.isFinite(+c.right_gap_px)?(+c.right_gap_px).toFixed(2):'--',left,right,String(c.left_recenter_latched??'--'),String(c.right_recenter_latched??'--'),actual,`${exp.rec} • ${exp.left}/${exp.right}`,match?'BENAR':'SALAH']);
  let nr=rep+1,nc=cond;if(nr>20){nr=1;nc={gray:'green',green:'yellow',yellow:'red',red:'gray'}[cond]}perBab4Get('perBab4LaneRepeat').value=String(nr);perBab4Get('perBab4LaneCondition').value=nc;perBab4SyncLaneSide();toast(`${label} #${rep}: ${match?'BENAR':'SALAH'}`,!match);
}
function perBab4SyncLaneSide(){const cond=perBab4Get('perBab4LaneCondition')?.value||'gray',s=perBab4Get('perBab4LaneSide');if(s)s.disabled=cond==='gray'||cond==='green';perBab4RefreshLive()}
function perBab4Undo(){const a=perBab4Rows();if(!a.length)return toast('Belum ada trial untuk di-undo',true);a.pop();tableRunRows.set(0,a);renderExperimentTable();perBab4RenderProgress();perBab4DrawSummary();toast('Trial terakhir dihapus')}
function perBab4BuildDetectionSummary(){const groups=new Map();for(const r of perBab4Rows()){const v=r.values,key=`${v[1]}|${v[2]}`,g=groups.get(key)||{target:v[1],dist:v[2],n:0,ok:0,conf:[]};g.n++;if(v[8]==='BERHASIL'){g.ok++;const c=+v[7];if(Number.isFinite(c))g.conf.push(c)}groups.set(key,g)}return [...groups.values()].sort((a,b)=>String(a.target).localeCompare(String(b.target))||(+a.dist)-(+b.dist)).map(g=>[g.target,g.dist,g.n,g.ok,g.n-g.ok,(100*g.ok/g.n).toFixed(2),(100*(g.n-g.ok)/g.n).toFixed(2),g.conf.length?(g.conf.reduce((a,b)=>a+b,0)/g.conf.length).toFixed(5):'N/A'])}
function perBab4BuildLaneSummary(){const groups=new Map();for(const r of perBab4Rows()){const v=r.values,g=groups.get(v[1])||{cond:v[1],n:0,ok:0};g.n++;if(v[14]==='BENAR')g.ok++;groups.set(v[1],g)}const order=['Abu-abu','Hijau','Kuning','Merah'];return [...groups.values()].sort((a,b)=>order.indexOf(a.cond)-order.indexOf(b.cond)).map(g=>[g.cond,g.n,g.ok,g.n-g.ok,(100*g.ok/g.n).toFixed(2)])}
function perBab4SetSummaryRows(rows){tableRunRows.set(1,rows.map(values=>({sec:0,values,yaml:perBab4YamlSnapshot()})))}
function perBab4PerformanceSummary(){const dur=Math.max(.001,(Date.now()-perBab4.perf.started)/1000),gaps=Math.max(0,sseGapCount-perBab4.perf.gapStart),valid=gaps===0&&perBab4.perf.count>0,st=p=>metricStats(p),mean=p=>{const v=st(p)?.mean;return Number.isFinite(v)?v.toFixed(3):'--'},fps=valid?(perBab4.perf.count/dur).toFixed(3):'--',cond=perBab4Get('perBab4PerfCondition')?.value||$('runCondition')?.value||'--',backend=obj('perception_performance').backend||'--';return [cond,dur.toFixed(2),perBab4.perf.count,fps,mean('perception_performance.pipeline_fps'),mean('perception_performance.inference_ms'),mean('perception_performance.pipeline_ms_per_frame'),mean('perception_performance.frame_age_ms'),mean('derived.per_bab4_latency_ms'),gaps,backend,valid?'VALID':'CHECK / SSE GAP OR NO FRAME']}
function perBab4PrepareSummary(){if(!perBab4Is())return;if(selectedExperiment.id==='F4.2')perBab4SetSummaryRows(perBab4BuildDetectionSummary());else if(selectedExperiment.id==='F4.3')perBab4SetSummaryRows(perBab4BuildLaneSummary());else if(selectedExperiment.id==='F4.4')tableRunRows.set(0,[{sec:0,values:perBab4PerformanceSummary(),yaml:perBab4YamlSnapshot()}]);renderExperimentTable();perBab4DrawSummary()}
function perBab4PerfStart(){clearInterval(perBab4.perf.timer);perBab4.perf={active:true,count:0,lastUpdate:+updated.raw_detections||0,gapStart:sseGapCount,started:Date.now(),stopBusy:false,timer:null};perBab4.perf.timer=setInterval(()=>{perBab4RefreshLive();if(perBab4.perf.active&&!perBab4.perf.stopBusy&&Date.now()-perBab4.perf.started>=60000){perBab4.perf.stopBusy=true;stopWebRecording()}},250)}
function perBab4PerfStop(){perBab4.perf.active=false;clearInterval(perBab4.perf.timer);perBab4.perf.timer=null}
const perBab4BaseMerge=merge;
merge=function(delta){if(perBab4.perf.active&&delta&&Object.prototype.hasOwnProperty.call(delta,'raw_detections')){const t=+(delta.__updated?.raw_detections||0);if(t>perBab4.perf.lastUpdate){perBab4.perf.lastUpdate=t;perBab4.perf.count++}}perBab4BaseMerge(delta);if(perBab4Is())perBab4RefreshLive()};
const perBab4BaseStart=startWebRecording;
startWebRecording=async function(){if(perBab4Is('F4.4')){const c=perBab4Get('perBab4PerfCondition')?.value;if(c&&$('runCondition'))$('runCondition').value=c}await perBab4BaseStart();if(recordStartedMs&&perBab4Is('F4.4'))perBab4PerfStart();if(recordStartedMs&&perBab4Is())perBab4RenderProgress()};
const perBab4BaseStop=stopWebRecording;
stopWebRecording=async function(){if(recordStartedMs&&perBab4Is())perBab4PrepareSummary();perBab4PerfStop();return perBab4BaseStop()};
function perBab4ProgressCounts(){if(selectedExperiment?.id==='F4.2'){const out={};for(const r of perBab4Rows()){const v=r.values,k=`${v[1]}-${v[2]}`;out[k]=(out[k]||0)+1}return out}if(selectedExperiment?.id==='F4.3'){const out={};for(const r of perBab4Rows()){const k=r.values[1];out[k]=(out[k]||0)+1}return out}return {}}
function perBab4RenderProgress(){const box=perBab4Get('perBab4Progress');if(!box||!perBab4Is())return;const c=perBab4ProgressCounts();if(selectedExperiment.id==='F4.2'){box.innerHTML=['human','motorcycle'].map(t=>`<div class="per-bab4-progress-row"><b>${t==='human'?'Manusia':'Sepeda motor'}</b>${[1,2,3,4,5].map(d=>`<span class="${(c[`${t}-${d}`]||0)>=20?'done':''}">${d}m <em>${c[`${t}-${d}`]||0}/20</em></span>`).join('')}</div>`).join('')}else if(selectedExperiment.id==='F4.3'){box.innerHTML=['Abu-abu','Hijau','Kuning','Merah'].map(k=>`<span class="${(c[k]||0)>=20?'done':''}">${k} <em>${c[k]||0}/20</em></span>`).join('')}else if(selectedExperiment.id==='F4.4'){const s=perBab4.perf.started>0?Math.min(60,Math.max(0,(Date.now()-perBab4.perf.started)/1000)):0;box.innerHTML=`<span class="${s>=60?'done':''}">60 s <em>${s.toFixed(1)} / 60.0</em></span>`}}
function perBab4CandidateOptions(){const s=perBab4Get('perBab4Candidate'),ds=perBab4RawDetections();if(!s)return;const old=+s.value||0;s.innerHTML=ds.length?ds.map((d,i)=>`<option value="${i}">#${i+1} • class ${escapeHtml(d.class_id??'--')} • conf ${Number.isFinite(+d.score)?(+d.score).toFixed(3):'--'}</option>`).join(''):'<option value="0">No candidate</option>';s.value=String(Math.min(old,Math.max(0,ds.length-1)));s.disabled=!ds.length}
function perBab4RefreshLive(){if(!perBab4Is())return;perBab4RenderProgress();const p=obj('perception_performance');setText('perBab4LiveBackend',p.backend||'--');setText('perBab4LiveFps',Number.isFinite(+p.pipeline_fps)?(+p.pipeline_fps).toFixed(2):'--');if(selectedExperiment.id==='F4.2'){perBab4CandidateOptions();setText('perBab4LivePrimary',`${perBab4RawDetections().length} raw candidate`)}else if(selectedExperiment.id==='F4.3'){const c=obj('lane_state').corridor||{};setText('perBab4LivePrimary',`${c.recommendation||'--'} • ${c.left_status||'--'} / ${c.right_status||'--'}`)}else if(selectedExperiment.id==='F4.4'){setText('perBab4LivePrimary',`${perBab4.perf.count} frame event • SSE gap ${Math.max(0,sseGapCount-perBab4.perf.gapStart)}`)}perBab4DrawOverlay()}
function perBab4DetectionControls(){return `<div class="per-bab4-controls"><label>Target fisik<select id="perBab4Target"><option value="human">Manusia</option><option value="motorcycle">Sepeda motor</option></select></label><label>Jarak GT<select id="perBab4Distance">${[1,2,3,4,5].map(x=>`<option value="${x}">${x} meter</option>`).join('')}</select></label><label>Ulangan<input id="perBab4Repeat" type="number" min="1" max="20" step="1" value="1"></label><label>Kandidat untuk confidence<select id="perBab4Candidate"></select></label><label class="wide">Catatan visual<input id="perBab4Note" type="text" placeholder="opsional: occlusion / bayangan / bbox objek lain"></label><div class="per-bab4-actions"><button class="primary-btn" id="perBab4Success">Capture BERHASIL</button><button class="warning-btn" id="perBab4Fail">Capture GAGAL</button><button class="soft-btn" id="perBab4Undo">Undo</button></div></div>`}
function perBab4LaneControls(){return `<div class="per-bab4-controls"><label>Kondisi acuan<select id="perBab4LaneCondition"><option value="gray">Abu-abu • no mask evidence</option><option value="green">Hijau • clear</option><option value="yellow">Kuning • warning</option><option value="red">Merah • intrusion</option></select></label><label>Sisi uji<select id="perBab4LaneSide"><option value="left">Kiri</option><option value="right">Kanan</option></select></label><label>Ulangan<input id="perBab4LaneRepeat" type="number" min="1" max="20" step="1" value="1"></label><div class="per-bab4-actions"><button class="primary-btn" id="perBab4LaneCapture">Capture + Auto Compare</button><button class="soft-btn" id="perBab4Undo">Undo</button></div></div>`}
function perBab4PerfControls(){return `<div class="per-bab4-controls"><label>Kondisi visual<select id="perBab4PerfCondition"><option>Jalur relatif kosong</option><option>Satu motor pada jalur</option><option>Obstacle dan lane padat</option></select></label><div class="per-bab4-note"><b>Protocol</b><span>START → hitung event frame selesai di subscriber → AUTO STOP 60 s. Jika SSE gap > 0, FPS hitung ditandai tidak valid dan tidak diisi dari estimasi.</span></div></div>`}
function perBab4InfoText(){if(selectedExperiment?.id==='F4.1')return '4.1 adalah audit implementasi. Tidak ada angka hasil estimasi yang ditanam di GUI; gunakan Live/Configuration/Evidence sebagai bukti aktual.';if(selectedExperiment?.id==='F4.2')return '200 percobaan: manusia + sepeda motor, 1–5 m, 20 percobaan per jarak. SUCCESS dipilih observer hanya jika bbox melingkupi sasaran fisik; confidence hanya disimpan dari kandidat SUCCESS yang dipilih.';if(selectedExperiment?.id==='F4.3')return '80 percobaan: abu-abu, hijau, kuning, merah × 20. GUI membandingkan recommendation + warna kiri/kanan terhadap kondisi acuan dan mencatat latch/gap aktual.';return 'Tiga kondisi visual × 60 s. Frame count berasal dari event raw_detections yang benar-benar diterima subscriber web; SSE gap membuat hasil FPS count invalid, bukan diisi estimasi.'}
function perBab4StepText(){
  if(selectedExperiment?.id==='F4.1')return ['Tujuan','Buktikan implementasi kamera → YOLOPv2 CPU → ROS 2. Tidak perlu 200/80 trial.','Yang dilakukan','Pastikan kamera dan YOLOPv2 aktif, cek backend/parameter efektif, lalu simpan bukti runtime/screenshot untuk audit implementasi.','Output laporan','Bukti implementasi + konfigurasi aktif; bukan angka hasil deteksi.'];
  if(selectedExperiment?.id==='F4.2')return ['Tujuan','Ambil 200 trial kandidat obstacle: 2 target × 5 jarak × 20 ulangan.','Yang dilakukan','START sekali untuk satu sesi → ukur jarak fisik → pilih target/jarak → Capture BERHASIL atau GAGAL → GUI maju otomatis ke ulangan berikutnya.','Output laporan','Tabel raw + rekap Detection Rate/Error/Confidence; STOP + SAVE otomatis menyimpan grafik DR, grafik confidence, serta montage 5 jarak untuk tiap objek.'];
  if(selectedExperiment?.id==='F4.3')return ['Tujuan','Ambil 80 trial Lane Safety: abu-abu, hijau, kuning, merah × 20.','Yang dilakukan','START → pilih kondisi acuan dan sisi bila perlu → stabilkan mask/lane → Capture + Auto Compare → ulang sampai 80/80.','Output laporan','Tabel raw gap/latch/status + rekap Success Rate; STOP + SAVE otomatis menyimpan grafik success rate dan montage visual 6 kondisi lane.'];
  return ['Tujuan','Uji performa CPU pada 3 kondisi visual, masing-masing 60 detik.','Yang dilakukan','Pilih kondisi → START → jangan ubah kondisi selama run → GUI AUTO STOP 60 s → ulangi untuk dua kondisi lainnya.','Output laporan','FPS, inference, process total, frame age, subscriber latency + PNG time-series.'];
}
function perBab4RenderFinalNavigator(){
  const grid=$('page-experiments')?.querySelector('.workbench-grid');if(!grid)return;let box=$('perBab4FinalNavigator');
  const finalMode=activeDomain==='perception'&&(activeWorkspaceTab?.perception==='tune'||activePage==='experiments');
  if(!finalMode){if(box)box.hidden=true;return}
  if(!box){box=document.createElement('section');box.id='perBab4FinalNavigator';box.className='per-bab4-final-nav';grid.before(box)}
  box.hidden=false;const cards=[['F4.1','4.1','Implementasi','Audit runtime • tanpa trial massal'],['F4.2','4.2','Object Detection','200 trial • manusia + motor • 1–5 m'],['F4.3','4.3','Lane Safety','80 trial • 4 kondisi'],['F4.4','4.4','Performa Real-Time','3 kondisi • 60 s/kondisi']];
  box.innerHTML=`<div class="per-final-title"><div><span class="eyebrow">PENGAMBILAN DATA LAPORAN</span><h2>FINAL BAB IV • 4.1–4.4</h2><p>Gunakan hanya empat tahap ini saat pengambilan data sidang. Commissioning lengkap tetap tersedia di tab <b>Commissioning</b>.</p></div><div class="per-final-flow"><span>1 PILIH TAHAP</span><i>→</i><span>2 CEK SOURCE</span><i>→</i><span>3 START / CAPTURE</span><i>→</i><span>4 STOP + SAVE</span><i>→</i><span>5 XLSX / PNG</span></div></div><div class="per-final-cards">${cards.map(c=>`<button data-final-id="${c[0]}" class="${selectedExperiment?.id===c[0]?'active':''}"><em>${c[1]}</em><b>${c[2]}</b><small>${c[3]}</small></button>`).join('')}</div>`;
  box.querySelectorAll('[data-final-id]').forEach(b=>b.onclick=()=>selectExperimentById(b.dataset.finalId));
}
function perBab4RenderPanel(){
  const main=$('page-experiments')?.querySelector('.workbench-main');if(!main)return;let p=perBab4Get('perceptionBab4Acquisition');if(!perBab4Is()){if(p)p.hidden=true;perBab4RenderFinalNavigator();return}if(!p){p=document.createElement('section');p.id='perceptionBab4Acquisition';p.className='per-bab4-panel';const run=$('page-experiments')?.querySelector('.run-console');run?.after(p)}p.hidden=false;const controls=selectedExperiment.id==='F4.2'?perBab4DetectionControls():selectedExperiment.id==='F4.3'?perBab4LaneControls():selectedExperiment.id==='F4.4'?perBab4PerfControls():'';const st=perBab4StepText();
  const summary=selectedExperiment.id==='F4.2'?`<div class="per-bab4-summary-head"><b>Grafik rekap otomatis</b><div><button class="soft-btn" id="perBab4SaveDrPng">PNG Detection Rate</button><button class="soft-btn" id="perBab4SaveConfPng">PNG Confidence</button></div></div><canvas id="perBab4SummaryCanvas" width="900" height="260"></canvas><canvas id="perBab4ConfidenceCanvas" width="900" height="260"></canvas>`:selectedExperiment.id==='F4.3'?`<div class="per-bab4-summary-head"><b>Grafik rekap otomatis</b><button class="soft-btn" id="perBab4SaveDrPng">PNG Success Rate</button></div><canvas id="perBab4SummaryCanvas" width="900" height="260"></canvas>`:'';
  p.innerHTML=`<div class="per-bab4-head"><div><span class="eyebrow">FINAL BAB IV • MODE PENGAMBILAN DATA</span><b>${escapeHtml(displayExperimentId(selectedExperiment))} ${escapeHtml(displayExperimentTitle(selectedExperiment))}</b><p>${escapeHtml(perBab4InfoText())}</p></div><div class="per-bab4-live"><span>Backend <b id="perBab4LiveBackend">--</b></span><span>Pipeline <b id="perBab4LiveFps">--</b> FPS</span><span id="perBab4LivePrimary">--</span></div></div><div class="per-bab4-now"><div><span>${st[0]}</span><b>${st[1]}</b></div><div><span>${st[2]}</span><b>${st[3]}</b></div><div><span>${st[4]}</span><b>${st[5]}</b></div></div>${controls}<div class="per-bab4-progress" id="perBab4Progress"></div>${summary}`;perBab4RenderFinalNavigator();
  perBab4Get('perBab4Success')?.addEventListener('click',()=>perBab4DetectionCapture(true));perBab4Get('perBab4Fail')?.addEventListener('click',()=>perBab4DetectionCapture(false));perBab4Get('perBab4LaneCapture')?.addEventListener('click',perBab4LaneCapture);perBab4Get('perBab4Undo')?.addEventListener('click',perBab4Undo);perBab4Get('perBab4LaneCondition')?.addEventListener('change',perBab4SyncLaneSide);perBab4Get('perBab4Candidate')?.addEventListener('change',()=>{perBab4.selectedCandidate=+perBab4Get('perBab4Candidate').value||0;perBab4DrawOverlay()});perBab4Get('perBab4SaveDrPng')?.addEventListener('click',()=>perBab4DownloadCanvas('perBab4SummaryCanvas',selectedExperiment.id==='F4.3'?'lane_success_rate':'detection_rate'));perBab4Get('perBab4SaveConfPng')?.addEventListener('click',()=>perBab4DownloadCanvas('perBab4ConfidenceCanvas','confidence_vs_distance'));perBab4SyncLaneSide();perBab4RefreshLive();perBab4DrawSummary();
}
function perBab4DrawSummary(){const c=perBab4Get('perBab4SummaryCanvas');if(!c||!perBab4Is())return;const ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='700 16px system-ui';let rows=[],valueKey='';if(selectedExperiment.id==='F4.2'){rows=perBab4BuildDetectionSummary();valueKey='Detection Rate';rows=rows.map(r=>({label:`${r[0]} ${r[1]}m`,value:+r[5]}))}else if(selectedExperiment.id==='F4.3'){rows=perBab4BuildLaneSummary().map(r=>({label:r[0],value:+r[4]}));valueKey='Success Rate'}else{ctx.fillStyle='#8fa8a9';ctx.font='13px system-ui';ctx.fillText(selectedExperiment.id==='F4.4'?'Grafik time-series tersedia pada panel GRAPH setelah START.':'Bagian 4.1 tidak membutuhkan grafik eksperimen.',20,40);return}if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.font='13px system-ui';ctx.fillText('Belum ada trial dicapture.',20,40);return}const left=60,bottom=42,top=35,right=20,bw=(w-left-right)/rows.length*.66,gap=(w-left-right)/rows.length;ctx.fillStyle='#8fa8a9';ctx.font='12px system-ui';ctx.fillText(`${valueKey} [%]`,18,20);for(let y=0;y<=100;y+=25){const py=h-bottom-(y/100)*(h-top-bottom);ctx.strokeStyle='rgba(150,180,180,.16)';ctx.beginPath();ctx.moveTo(left,py);ctx.lineTo(w-right,py);ctx.stroke();ctx.fillStyle='#8fa8a9';ctx.fillText(String(y),20,py+4)}rows.forEach((r,i)=>{const x=left+i*gap+(gap-bw)/2,bh=(Math.max(0,Math.min(100,r.value))/100)*(h-top-bottom),y=h-bottom-bh;ctx.fillStyle='#48e0a4';ctx.fillRect(x,y,bw,bh);ctx.fillStyle='#d8eeee';ctx.font='11px system-ui';ctx.fillText(Number.isFinite(r.value)?r.value.toFixed(1):'--',x,y-5);ctx.save();ctx.translate(x+bw/2,h-bottom+8);ctx.rotate(-.35);ctx.fillText(r.label,0,0);ctx.restore()})}

function perBab4DownloadCanvas(id,name){const c=$(id);if(!c)return toast('Grafik belum tersedia',true);c.toBlob(blob=>{if(!blob)return;const a=document.createElement('a'),u=URL.createObjectURL(blob);a.href=u;a.download=`FINAL_${selectedExperiment?.id||'BAB4'}_${name}_${stamp()}.png`;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(u),1000)},'image/png')}
function perBab4DrawConfidence(){
  const c=$('perBab4ConfidenceCanvas');if(!c||selectedExperiment?.id!=='F4.2')return;const ctx=c.getContext('2d'),w=c.width,h=c.height,p={l:62,r:24,t:34,b:42};ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='700 16px system-ui';ctx.fillText('Confidence rata-rata terhadap jarak',18,22);ctx.font='12px system-ui';ctx.fillStyle='#8fa8a9';for(let i=0;i<=4;i++){const v=i/4,y=h-p.b-v*(h-p.t-p.b);ctx.strokeStyle='rgba(150,180,180,.16)';ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.fillText(v.toFixed(2),18,y+4)}for(let d=1;d<=5;d++){const x=p.l+(d-1)/4*(w-p.l-p.r);ctx.fillText(`${d} m`,x-10,h-15)}const rows=perBab4BuildDetectionSummary(),series=[['human','Manusia','#48e0a4'],['motorcycle','Sepeda motor','#ffc86b']];for(const [key,label,col] of series){const pts=rows.filter(r=>r[0]===key&&Number.isFinite(+r[7])).map(r=>({d:+r[1],v:+r[7]}));ctx.strokeStyle=col;ctx.fillStyle=col;ctx.lineWidth=3;ctx.beginPath();pts.forEach((q,i)=>{const x=p.l+(q.d-1)/4*(w-p.l-p.r),y=h-p.b-q.v*(h-p.t-p.b);i?ctx.lineTo(x,y):ctx.moveTo(x,y);ctx.beginPath;ctx.arc(x,y,4,0,Math.PI*2);ctx.fill();if(i===0){ctx.beginPath();ctx.moveTo(x,y)}else{const prev=pts[i-1],px=p.l+(prev.d-1)/4*(w-p.l-p.r),py=h-p.b-prev.v*(h-p.t-p.b);ctx.beginPath();ctx.moveTo(px,py);ctx.lineTo(x,y);ctx.stroke()}});ctx.fillStyle=col;ctx.fillText(label,w-p.r-150,20+series.findIndex(s=>s[0]===key)*15)}if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.fillText('Belum ada trial. Grafik terisi otomatis setelah Capture BERHASIL/GAGAL.',p.l,60)}
}
const perBab4DrawSummaryBase=perBab4DrawSummary;
perBab4DrawSummary=function(){perBab4DrawSummaryBase();perBab4DrawConfidence()};
function perBab4EnsureOverlay(){const box=$('contextCamera');if(!box)return null;let c=perBab4Get('perBab4DetectionOverlay');if(!c){c=document.createElement('canvas');c.id='perBab4DetectionOverlay';c.className='per-bab4-detection-overlay';box.appendChild(c)}c.hidden=!(perBab4Is('F4.2'));return c}
function perBab4DrawOverlay(){const c=perBab4EnsureOverlay();if(!c||c.hidden)return;const img=$('expCameraImage'),g=cameraGeometryFor(img);if(!g)return;const fit=g.resizeCanvas(c),ctx=fit.ctx,dpr=fit.dpr,cr=c.getBoundingClientRect();ctx.clearRect(0,0,fit.width,fit.height);const ds=perBab4RawDetections(),candidateSelect=perBab4Get('perBab4Candidate'),autoIndex=!candidateSelect&&typeof perBab4AutoDetectionDecision==='function'?perBab4AutoDetectionDecision().index:-1,selectedIndex=candidateSelect?(+candidateSelect.value||0):autoIndex;ds.forEach((d,i)=>{const a=g.imageToScreen?.(+d.x1,+d.y1),b=g.imageToScreen?.(+d.x2,+d.y2);if(!a||!b)return;const x=(a.x-cr.left)*dpr,y=(a.y-cr.top)*dpr,w=(b.x-a.x)*dpr,h=(b.y-a.y)*dpr;ctx.strokeStyle=i===selectedIndex?'#ffc86b':'#48e0a4';ctx.lineWidth=(i===selectedIndex?3:2)*dpr;ctx.strokeRect(x,y,w,h);ctx.fillStyle=ctx.strokeStyle;ctx.font=`${12*dpr}px ui-monospace`;ctx.fillText(`#${i+1} c${d.class_id??'--'} ${(Number(d.score)||0).toFixed(2)}`,x+3*dpr,Math.max(14*dpr,y-3*dpr))})}
function perBab4LanePointCards(){const panel=$('perceptionCalibrationSection');if(!panel)return;let box=perBab4Get('perLanePointCards');if(!box){box=document.createElement('div');box.id='perLanePointCards';box.className='per-lane-point-cards';const read=$('perceptionCalReadout');read?.before(box)}const c=perceptionDirectCal?.candidate;if(!c){box.innerHTML='<span>Menunggu YAML lane...</span>';return}const pts=[['LT','Kiri Atas',c.leftTopX,c.topY,0],['LB','Kiri Bawah',c.leftBottomX,c.bottomY,1],['RT','Kanan Atas',c.rightTopX,c.topY,2],['RB','Kanan Bawah',c.rightBottomX,c.bottomY,3]];box.innerHTML=`<div class="per-lane-point-note"><b>4 TITIK LANE SAFETY</b><span>Drag LT/LB/RT/RB pada kamera. Runtime memakai Y bersama untuk pasangan atas dan pasangan bawah; X tiap titik independen. Keyboard 1–4 + panah = fine adjust.</span></div>${pts.map(([id,name,x,y,i])=>`<button type="button" data-lane-handle="${i}"><b>${id}</b><span>${name}</span><code>x ${(+x).toFixed(4)} • y ${(+y).toFixed(4)}</code></button>`).join('')}`;box.querySelectorAll('[data-lane-handle]').forEach(b=>b.onclick=()=>{perceptionDirectCal.mode='lane';perceptionDirectCal.keyboardIndex=+b.dataset.laneHandle;$('perceptionCalOverlay')?.focus();drawPerceptionDirectOverlay()})}
const perBab4BaseRenderDirect=renderPerceptionDirectCalibration;
renderPerceptionDirectCalibration=function(){const r=perBab4BaseRenderDirect();perBab4LanePointCards();return r};
const perBab4BaseDrawDirect=drawPerceptionDirectOverlay;
drawPerceptionDirectOverlay=function(){perBab4BaseDrawDirect();const canvas=$('perceptionCalOverlay'),img=$('cameraImage'),g=cameraGeometryFor(img),c=perceptionDirectCal?.candidate;if(!canvas||!g||!c||perceptionDirectCal.mode!=='lane')return;const fit=g.resizeCanvas(canvas),ctx=fit.ctx,dpr=fit.dpr,cr=canvas.getBoundingClientRect(),pts=directCalLanePoints(c),labels=['LT','LB','RT','RB'];ctx.font=`700 ${11*dpr}px ui-monospace`;pts.forEach((p,i)=>{const s=g.normalizedToScreen(p.x,p.y);if(!s)return;ctx.fillStyle=i===perceptionDirectCal.keyboardIndex?'#ffc86b':'#d8eeee';ctx.fillText(labels[i],(s.x-cr.left+10)*dpr,(s.y-cr.top-9)*dpr)});perBab4LanePointCards()};
const perBab4BaseRenderSelected=renderSelectedExperiment;
renderSelectedExperiment=function(){const r=perBab4BaseRenderSelected();perBab4RenderPanel();if(perBab4Is()&&(activeWorkspaceTab?.perception==='tune'||activePage==='experiments')){document.body.dataset.bab4=selectedExperiment.id;setText('pageEyebrow','PERSEPSI • LAPORAN BAB IV');const badge=$('experimentGraphBadge');if(badge){if(selectedExperiment.id==='F4.2')badge.textContent='2 GRAPH REKAP';else if(selectedExperiment.id==='F4.3')badge.textContent='1 GRAPH REKAP';else if(selectedExperiment.id==='F4.1')badge.textContent='BUKTI LIVE';}const sub=$('experimentSubtitle');if(sub)sub.textContent='Mode FINAL laporan: ikuti panel langkah di bawah. Tidak perlu masuk workflow tuning/qualification.'}return r};
function perBab4FixDisplay(){const base=displayExperimentId;displayExperimentId=function(x){const id=String(x?.id||'');if(currentExp==='perception'&&/^F4\.\d+$/.test(id))return id.slice(1);return base(x)}}
perBab4PatchWorkspace();perBab4FixDisplay();


/* ===== BAB IV REPORT ACQUISITION PARITY v2 (2026-09-16) =====
 * Scope: perception FINAL only. Navigation and direct obstacle/lane calibration stay untouched.
 * Adds the report-only negative-detection test, lane hysteresis evidence, and 1 s performance evidence.
 */
perBab4.report={negativeScenario:'indoor',hysStep:1,hysCycle:1,hysSide:'left'};
const perBab4ReportBasePatchCatalog=patchPerceptionBab4Catalog;
patchPerceptionBab4Catalog=function(){
  perBab4ReportBasePatchCatalog();
  const list=experiments.perception||[],byId=id=>list.find(x=>x.id===id);let x;
  if((x=byId('F4.2'))){
    x.graphCaptions=['Detection Rate terhadap jarak','Confidence rata-rata terhadap jarak','Montage Manusia 1–5 m','Montage Sepeda Motor 1–5 m'];
    x.graphs=x.graphCaptions.map(()=>({type:'time_series',series:['Raw detections'],xLabel:'BAB IV',yLabel:'Evidence'}));
    x.tableColumns=[
      ['Trial','Target Fisik','Jarak GT [m]','Ulangan','Frame timestamp','Backend','Raw Candidate','Confidence Kandidat','Hasil Spasial','Raw Class ID(s)','Catatan Visual','Evaluasi Otomatis','Evidence Frame'],
      ['Target Fisik','Jarak [m]','Percobaan','Berhasil','Gagal','Detection Rate [%]','Error [%]','Confidence rata-rata'],
      ['Frame','Skenario Negatif','Ulangan','Timestamp','Raw Candidate','Max Confidence','Raw Class ID(s)','Tanpa Kandidat','Kandidat Palsu','Catatan'],
      ['Skenario Negatif','Frame','Tanpa Kandidat','Kandidat Palsu','Rasio Kandidat Palsu [%]']
    ];
    x.tableNames=['Raw 200 Percobaan Object Detection','Rekap Detection Rate dan Confidence','Raw 30 Frame Uji Negatif','Rekap Uji Negatif Kandidat'];
  }
  if((x=byId('F4.3'))){
    x.graphCaptions=['Success Rate Lane Safety','Montage Lane Safety 6 kondisi'];
    x.graphs=x.graphCaptions.map(()=>({type:'time_series',series:['Lane valid'],xLabel:'BAB IV',yLabel:'Evidence'}));
    x.tableColumns=[[...x.tableColumns[0],'Evidence Frame'],x.tableColumns[1],['Siklus','Step','Target Gap [px]','Sisi','Timestamp','Gap Aktual [px]','Status Sisi','Status Lawan','Latch Aktual','Recommendation Aktual','Expected','Hasil']];
    x.tableNames=['Raw 80 Percobaan Lane Safety','Rekap Success Rate Lane Safety','Verifikasi Urutan Histeresis Lane Safety'];
  }
  if((x=byId('F4.4'))){
    x.tableColumns=[x.tableColumns[0],['Detik','N Performance','N Frame Event','Mean Pipeline FPS','Mean Inference [ms]','Mean Process Total [ms]','Mean Frame Age [ms]','Mean Subscriber Latency [ms]'],['Parameter','Mean','Std','Min','Max','N','Unit']];
    x.tableNames=['Hasil Pengujian Performa Mini PC CPU','Detail Mean per Detik','Ringkasan Statistik 60 Detik'];
  }
};
function perBab4RowsAt(index){return tableRunRows.get(index)||[]}
function perBab4PushAt(index,values){const a=perBab4RowsAt(index);a.push({sec:Math.max(0,Math.floor((Date.now()-(recordStartedMs||Date.now()))/1000)),values,yaml:perBab4YamlSnapshot()});tableRunRows.set(index,a);renderExperimentTable();perBab4RefreshReportExtensions()}
function perBab4SetRowsAt(index,rows){tableRunRows.set(index,rows.map(values=>({sec:0,values,yaml:perBab4YamlSnapshot()})));renderExperimentTable()}
function perBab4Mean(values){const a=values.map(Number).filter(Number.isFinite);return a.length?a.reduce((s,v)=>s+v,0)/a.length:NaN}
function perBab4Std(values){const a=values.map(Number).filter(Number.isFinite);if(!a.length)return NaN;const m=perBab4Mean(a);return Math.sqrt(a.reduce((s,v)=>s+(v-m)*(v-m),0)/a.length)}
function perBab4FmtNum(v,d=3){return Number.isFinite(+v)?(+v).toFixed(d):'--'}

const PER_BAB4_NEGATIVE_SCENARIOS=[
  ['indoor','Indoor tanpa manusia/motor'],
  ['empty','Jalur relatif kosong'],
  ['nontarget','Latar statis dengan objek non-target']
];
function perBab4NegativeLabel(key){return PER_BAB4_NEGATIVE_SCENARIOS.find(x=>x[0]===key)?.[1]||key}
function perBab4NegativeCapture(){
  if(!recordStartedMs)return toast('Klik START dahulu agar uji negatif tersimpan pada evidence run.',true);
  const scenario=$('perBab4NegativeScenario')?.value||'indoor',rep=+$('perBab4NegativeRepeat')?.value||1,dets=perBab4RawDetections();
  const scores=dets.map(d=>+d.score).filter(Number.isFinite),maxConf=scores.length?Math.max(...scores):NaN,classes=dets.map(d=>d.class_id).filter(v=>v!==undefined).join('|')||'--';
  const note=$('perBab4NegativeNote')?.value?.trim()||'',falseCandidate=dets.length>0;
  perBab4PushAt(2,[perBab4RowsAt(2).length+1,perBab4NegativeLabel(scenario),rep,perBab4FrameTimestamp(),dets.length,perBab4FmtNum(maxConf,5),classes,falseCandidate?'TIDAK':'YA',falseCandidate?'YA':'TIDAK',note]);
  perBab4BuildNegativeSummary();
  let nr=rep+1,ns=scenario;if(nr>10){nr=1;const i=PER_BAB4_NEGATIVE_SCENARIOS.findIndex(x=>x[0]===scenario);ns=PER_BAB4_NEGATIVE_SCENARIOS[(i+1)%PER_BAB4_NEGATIVE_SCENARIOS.length][0]}
  if($('perBab4NegativeRepeat'))$('perBab4NegativeRepeat').value=String(nr);if($('perBab4NegativeScenario'))$('perBab4NegativeScenario').value=ns;if($('perBab4NegativeNote'))$('perBab4NegativeNote').value='';
  perBab4RefreshReportExtensions();toast(`Uji negatif ${perBab4NegativeLabel(scenario)} #${rep}: ${falseCandidate?'KANDIDAT PALSU':'TANPA KANDIDAT'}`,falseCandidate);
}
function perBab4BuildNegativeSummary(){
  const groups=new Map();for(const r of perBab4RowsAt(2)){const v=r.values,g=groups.get(v[1])||{name:v[1],n:0,empty:0,false:0};g.n++;if(v[7]==='YA')g.empty++;if(v[8]==='YA')g.false++;groups.set(v[1],g)}
  const order=PER_BAB4_NEGATIVE_SCENARIOS.map(x=>x[1]),rows=[...groups.values()].sort((a,b)=>order.indexOf(a.name)-order.indexOf(b.name)).map(g=>[g.name,g.n,g.empty,g.false,(100*g.false/g.n).toFixed(2)]);
  if(rows.length){const n=rows.reduce((s,r)=>s+(+r[1]||0),0),e=rows.reduce((s,r)=>s+(+r[2]||0),0),f=rows.reduce((s,r)=>s+(+r[3]||0),0);rows.push(['TOTAL',n,e,f,n?(100*f/n).toFixed(2):'--'])}
  perBab4SetRowsAt(3,rows);
}
function perBab4NegativeUndo(){const a=perBab4RowsAt(2);if(!a.length)return toast('Belum ada frame uji negatif.',true);a.pop();tableRunRows.set(2,a);perBab4BuildNegativeSummary();perBab4RefreshReportExtensions();toast('Frame uji negatif terakhir dihapus')}

const PER_BAB4_HYS_SEQUENCE=[
  {gap:60,status:'GREEN',latched:false,phase:'CLEAR'},
  {gap:25,status:'YELLOW',latched:false,phase:'WARNING'},
  {gap:0,status:'RED',latched:true,phase:'TOUCH'},
  {gap:20,status:'YELLOW',latched:true,phase:'RECOVERY'},
  {gap:40,status:'YELLOW',latched:true,phase:'HOLD LATCH'},
  {gap:50,status:'GREEN',latched:false,phase:'RELEASE'}
];
function perBab4HysExpected(step,side){const q=PER_BAB4_HYS_SEQUENCE[Math.max(0,Math.min(5,step-1))],rec=q.status==='GREEN'?'CLEAR':q.status==='YELLOW'&&!q.latched?'WARNING':(side==='left'?'RECENTER_RIGHT':'RECENTER_LEFT');return {...q,rec}}
function perBab4HysCapture(){
  if(!recordStartedMs)return toast('Klik START dahulu agar urutan histeresis tersimpan.',true);
  const step=+$('perBab4HysStep')?.value||1,cycle=+$('perBab4HysCycle')?.value||1,side=$('perBab4HysSide')?.value||'left',c=obj('lane_state').corridor||{},exp=perBab4HysExpected(step,side);
  const actualStatus=String(side==='left'?c.left_status:c.right_status),otherStatus=String(side==='left'?c.right_status:c.left_status),latched=Boolean(side==='left'?c.left_recenter_latched:c.right_recenter_latched),gap=+(side==='left'?c.left_gap_px:c.right_gap_px),rec=String(c.recommendation||'--');
  const ok=actualStatus===exp.status&&latched===exp.latched&&rec===exp.rec;
  perBab4PushAt(2,[cycle,step,exp.gap,side==='left'?'Kiri':'Kanan',perBab4FrameTimestamp(),perBab4FmtNum(gap,2),actualStatus,otherStatus,String(latched),rec,`${exp.phase} • ${exp.status} • latch ${exp.latched} • ${exp.rec}`,ok?'BENAR':'CHECK']);
  let ns=step+1,nc=cycle;if(ns>6){ns=1;nc++}if($('perBab4HysStep'))$('perBab4HysStep').value=String(ns);if($('perBab4HysCycle'))$('perBab4HysCycle').value=String(nc);
  perBab4RefreshReportExtensions();toast(`Histeresis step ${step}: ${ok?'BENAR':'CHECK'}`,!ok);
}
function perBab4HysUndo(){const a=perBab4RowsAt(2);if(!a.length)return toast('Belum ada step histeresis.',true);a.pop();tableRunRows.set(2,a);renderExperimentTable();perBab4RefreshReportExtensions();toast('Step histeresis terakhir dihapus')}
function perBab4LaneProtocolSide(){const cond=$('perBab4LaneCondition')?.value||'gray',rep=+$('perBab4LaneRepeat')?.value||1;if(cond==='yellow'||cond==='red')return rep<=10?'left':'right';return 'left'}
const perBab4ReportBaseLaneCapture=perBab4LaneCapture;
perBab4LaneCapture=function(){const s=$('perBab4LaneSide'),cond=$('perBab4LaneCondition')?.value||'gray';if(s&&(cond==='yellow'||cond==='red'))s.value=perBab4LaneProtocolSide();return perBab4ReportBaseLaneCapture()};
const perBab4ReportBaseSyncLaneSide=perBab4SyncLaneSide;
perBab4SyncLaneSide=function(){perBab4ReportBaseSyncLaneSide();const cond=$('perBab4LaneCondition')?.value||'gray',s=$('perBab4LaneSide'),rep=+$('perBab4LaneRepeat')?.value||1;if(s&&(cond==='yellow'||cond==='red')){s.disabled=true;s.value=rep<=10?'left':'right';s.title='BAB IV: ulangan 1–10 sisi kiri, 11–20 sisi kanan'}perBab4RefreshReportExtensions()};

const perBab4ReportBasePerfStart=perBab4PerfStart;
perBab4PerfStart=function(){perBab4ReportBasePerfStart();perBab4.perf.perfSamples=[];perBab4.perf.secondBuckets=new Map();perBab4.perf.detailLastPerf=0;perBab4.perf.detailLastRaw=0;perBab4RefreshReportExtensions()};
function perBab4PerfBucket(sec){if(!perBab4.perf.secondBuckets)perBab4.perf.secondBuckets=new Map();if(!perBab4.perf.secondBuckets.has(sec))perBab4.perf.secondBuckets.set(sec,{sec,perf:[],frames:0});return perBab4.perf.secondBuckets.get(sec)}
function perBab4PerfSample(){
  if(!perBab4.perf.active)return;const t=+updated.perception_performance||0;if(!t||t<=perBab4.perf.detailLastPerf)return;perBab4.perf.detailLastPerf=t;
  const elapsed=Math.max(0,(Date.now()-perBab4.perf.started)/1000),p=obj('perception_performance'),row={elapsed,pipeline:+p.pipeline_fps,inference:+p.inference_ms,process:+p.pipeline_ms_per_frame,age:+p.frame_age_ms,latency:+perBab4LatencyMs()};
  perBab4.perf.perfSamples.push(row);perBab4PerfBucket(Math.min(59,Math.floor(elapsed))).perf.push(row);
}
function perBab4PerfFrameEvent(){if(!perBab4.perf.active)return;const t=+updated.raw_detections||0;if(!t||t<=perBab4.perf.detailLastRaw)return;perBab4.perf.detailLastRaw=t;const sec=Math.min(59,Math.max(0,Math.floor((Date.now()-perBab4.perf.started)/1000)));perBab4PerfBucket(sec).frames++}
const perBab4ReportMergeBase=merge;
merge=function(delta){perBab4ReportMergeBase(delta);if(perBab4.perf.active){if(delta&&Object.prototype.hasOwnProperty.call(delta,'perception_performance'))perBab4PerfSample();if(delta&&Object.prototype.hasOwnProperty.call(delta,'raw_detections'))perBab4PerfFrameEvent()}};
function perBab4PerfSecondRows(){const out=[];for(let sec=0;sec<60;sec++){const b=perBab4.perf.secondBuckets?.get(sec);if(!b)continue;const m=k=>perBab4Mean(b.perf.map(x=>x[k]));out.push([sec+1,b.perf.length,b.frames,perBab4FmtNum(m('pipeline')),perBab4FmtNum(m('inference')),perBab4FmtNum(m('process')),perBab4FmtNum(m('age')),perBab4FmtNum(m('latency'))])}return out}
function perBab4PerfStatsRows(){
  const samples=perBab4.perf.perfSamples||[],spec=[['Pipeline FPS','pipeline','FPS'],['Inference','inference','ms'],['Process total','process','ms'],['Frame age','age','ms'],['Subscriber latency','latency','ms']];
  return spec.map(([label,key,unit])=>{const a=samples.map(x=>x[key]).filter(Number.isFinite);if(!a.length)return[label,'--','--','--','--',0,unit];return[label,perBab4FmtNum(perBab4Mean(a)),perBab4FmtNum(perBab4Std(a)),perBab4FmtNum(Math.min(...a)),perBab4FmtNum(Math.max(...a)),a.length,unit]});
}
const perBab4ReportBasePrepareSummary=perBab4PrepareSummary;
perBab4PrepareSummary=function(){perBab4ReportBasePrepareSummary();if(perBab4Is('F4.4')){perBab4SetRowsAt(1,perBab4PerfSecondRows());perBab4SetRowsAt(2,perBab4PerfStatsRows())}};
function perBab4PerfResearchGate(){const s=perBab4PerformanceSummary(),fps=+s[3],lat=+s[8];if(!Number.isFinite(fps))return{ok:false,text:'WAIT DATA'};if(!Number.isFinite(lat))return{ok:false,text:`FPS ${fps.toFixed(1)} • LATENCY CHECK`};return{ok:fps>=15&&lat<=100,text:`${fps.toFixed(1)} FPS • ${lat.toFixed(1)} ms`}}

function perBab4ReportExtensionHost(){const main=$('perceptionBab4Acquisition');if(!main)return null;let e=$('perBab4ReportExtensions');if(!e){e=document.createElement('div');e.id='perBab4ReportExtensions';e.className='per-bab4-report-ext';main.appendChild(e)}return e}
function perBab4RenderReportExtensions(){
  const box=perBab4ReportExtensionHost();if(!box||!perBab4Is())return;if(selectedExperiment.id==='F4.2'){
    box.innerHTML=`<section class="per-bab4-evidence-card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">4.2.5 • UJI NEGATIF KANDIDAT</span><b>30 frame tanpa target fisik utama</b><p>3 skenario × 10 frame. Semua kandidat pada frame negatif dicatat sebagai kandidat palsu terhadap protokol manusia/motor.</p></div><span class="status-chip" id="perBab4NegativeState">0 / 30</span></div><div class="per-bab4-controls compact"><label>Skenario<select id="perBab4NegativeScenario">${PER_BAB4_NEGATIVE_SCENARIOS.map(x=>`<option value="${x[0]}">${x[1]}</option>`).join('')}</select></label><label>Ulangan<input id="perBab4NegativeRepeat" type="number" min="1" max="10" value="1"></label><label class="wide">Catatan<input id="perBab4NegativeNote" type="text" placeholder="pantulan / objek non-target / pencahayaan"></label><div class="per-bab4-actions"><button class="primary-btn" id="perBab4NegativeCapture">Capture Frame Negatif</button><button class="soft-btn" id="perBab4NegativeUndo">Undo</button><button class="soft-btn" id="perBab4NegativeCsv">CSV Uji Negatif</button></div></div><div class="per-bab4-kpis"><div><span>Raw candidate live</span><b id="perBab4NegLiveCount">--</b></div><div><span>Max confidence</span><b id="perBab4NegLiveConf">--</b></div><div><span>Kandidat palsu</span><b id="perBab4NegFalse">--</b></div><div><span>Rasio palsu</span><b id="perBab4NegRatio">--</b></div></div></section>`;
  }else if(selectedExperiment.id==='F4.3'){
    box.innerHTML=`<section class="per-bab4-evidence-card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">4.3.5 • VERIFIKASI HISTERESIS</span><b>Clear → Warning → Touch → Recovery → Release</b><p>Urutan acuan gap 60, 25, 0, 20, 40, 50 px. Rekam status + latch aktual per frame; jangan mengubah threshold saat satu siklus berjalan.</p></div><span class="status-chip" id="perBab4HysState">0 step</span></div><div class="per-bab4-controls compact"><label>Sisi<select id="perBab4HysSide"><option value="left">Kiri</option><option value="right">Kanan</option></select></label><label>Siklus<input id="perBab4HysCycle" type="number" min="1" value="1"></label><label>Step<select id="perBab4HysStep">${PER_BAB4_HYS_SEQUENCE.map((q,i)=>`<option value="${i+1}">${i+1} • ${q.phase} • ${q.gap}px</option>`).join('')}</select></label><div class="per-bab4-actions"><button class="primary-btn" id="perBab4HysCapture">Capture Step</button><button class="soft-btn" id="perBab4HysUndo">Undo</button><button class="soft-btn" id="perBab4HysCsv">CSV Histeresis</button></div></div><div class="per-bab4-kpis"><div><span>Expected</span><b id="perBab4HysExpected">--</b></div><div><span>Gap aktual</span><b id="perBab4HysGap">--</b></div><div><span>Status / latch</span><b id="perBab4HysActual">--</b></div><div><span>Recommendation</span><b id="perBab4HysRec">--</b></div></div></section>`;
  }else if(selectedExperiment.id==='F4.4'){
    box.innerHTML=`<section class="per-bab4-evidence-card per-bab4-timed"><div class="per-bab4-evidence-head"><div><span class="eyebrow">SOURCE-RATE EVIDENCE • 60 DETIK</span><b>Pengambilan data performa seperti panel Navigasi</b><p>Detik laporan selalu 1–60 bulat. Mean per detik berasal dari update /perception/performance aktual; frame event berasal dari /perception/raw_detections.</p></div><span class="status-chip" id="perBab4PerfState">READY</span></div><div class="per-bab4-kpis six"><div><span>Countdown</span><b id="perBab4PerfCountdown">60 s</b></div><div><span>Frame event</span><b id="perBab4PerfFrames">0</b></div><div><span>Perf samples</span><b id="perBab4PerfSamples">0</b></div><div><span>Raw source Hz</span><b id="perBab4PerfRawHz">--</b></div><div><span>Perf source Hz</span><b id="perBab4PerfSourceHz">--</b></div><div><span>Research gate</span><b id="perBab4PerfGate">WAIT</b></div></div><div class="per-bab4-second-table"><table><thead><tr><th>Detik</th><th>N perf</th><th>N frame</th><th>FPS</th><th>Inference ms</th><th>Process ms</th><th>Latency ms</th></tr></thead><tbody id="perBab4PerfSecondBody"><tr><td colspan="7">Belum recording.</td></tr></tbody></table></div></section>`;
  }else{box.innerHTML=`<section class="per-bab4-evidence-card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">4.1 • AUDIT IMPLEMENTASI</span><b>Bukti runtime tanpa angka estimasi</b><p>Pastikan backend CPU, kamera, model, raw detections, Lane Safety, dan performance topic terhubung. Bagian kalibrasi obstacle serta Lane Safety tetap berada pada panel kalibrasi asli dan tidak diubah.</p></div><span class="status-chip">READ ONLY</span></div></section>`}
  perBab4BindReportExtensions();perBab4RefreshReportExtensions();
}

function perBab4DownloadTableCsv(index,name){const cols=selectedExperiment?.tableColumns?.[index]||[],rows=perBab4RowsAt(index).map(r=>r.values||[]);if(!rows.length)return toast('Belum ada data untuk diekspor.',true);const q=v=>`"${String(v??'').replace(/"/g,'""')}"`,csv=[cols,...rows].map(r=>r.map(q).join(',')).join('\n');downloadBlob(`${name}_${stamp()}.csv`,'text/csv;charset=utf-8',csv)}
function perBab4BindReportExtensions(){
  $('perBab4NegativeCapture')?.addEventListener('click',perBab4NegativeCapture);$('perBab4NegativeUndo')?.addEventListener('click',perBab4NegativeUndo);$('perBab4NegativeCsv')?.addEventListener('click',()=>perBab4DownloadTableCsv(2,'BAB4_4.2_uji_negatif'));
  $('perBab4HysCapture')?.addEventListener('click',perBab4HysCapture);$('perBab4HysUndo')?.addEventListener('click',perBab4HysUndo);$('perBab4HysCsv')?.addEventListener('click',()=>perBab4DownloadTableCsv(2,'BAB4_4.3_histeresis'));
  $('perBab4HysStep')?.addEventListener('change',perBab4RefreshReportExtensions);$('perBab4HysSide')?.addEventListener('change',perBab4RefreshReportExtensions);$('perBab4LaneRepeat')?.addEventListener('input',perBab4SyncLaneSide);
}
function perBab4RefreshNegative(){const rows=perBab4RowsAt(2),summary=perBab4RowsAt(3),d=perBab4RawDetections(),scores=d.map(x=>+x.score).filter(Number.isFinite),falseCount=rows.filter(r=>r.values?.[8]==='YA').length;setText('perBab4NegativeState',`${rows.length} / 30`);const chip=$('perBab4NegativeState');if(chip)chip.className='status-chip '+(rows.length>=30?'':'waiting');setText('perBab4NegLiveCount',String(d.length));setText('perBab4NegLiveConf',scores.length?Math.max(...scores).toFixed(3):'--');setText('perBab4NegFalse',String(falseCount));setText('perBab4NegRatio',rows.length?`${(100*falseCount/rows.length).toFixed(1)}%`:'--');void summary}
function perBab4RefreshHys(){const step=+$('perBab4HysStep')?.value||1,side=$('perBab4HysSide')?.value||'left',exp=perBab4HysExpected(step,side),c=obj('lane_state').corridor||{},gap=+(side==='left'?c.left_gap_px:c.right_gap_px),st=String(side==='left'?c.left_status:c.right_status),latched=Boolean(side==='left'?c.left_recenter_latched:c.right_recenter_latched);setText('perBab4HysState',`${perBab4RowsAt(2).length} step`);setText('perBab4HysExpected',`${exp.gap}px • ${exp.status} • latch ${exp.latched}`);setText('perBab4HysGap',Number.isFinite(gap)?`${gap.toFixed(1)} px`:'--');setText('perBab4HysActual',`${st||'--'} • ${latched}`);setText('perBab4HysRec',c.recommendation||'--')}
function perBab4RefreshPerfExt(){const active=!!perBab4.perf.active,elapsed=active?Math.max(0,(Date.now()-perBab4.perf.started)/1000):0,left=Math.max(0,60-elapsed),gate=perBab4PerfResearchGate();setText('perBab4PerfState',active?'RECORDING':(perBab4.perf.started?'DONE':'READY'));const chip=$('perBab4PerfState');if(chip)chip.className='status-chip '+(active?'waiting':perBab4.perf.started?'':'');setText('perBab4PerfCountdown',`${Math.ceil(left)} s`);setText('perBab4PerfFrames',String(perBab4.perf.count||0));setText('perBab4PerfSamples',String(perBab4.perf.perfSamples?.length||0));setText('perBab4PerfRawHz',perBab4FmtNum(channelRate('raw_detections'),2));setText('perBab4PerfSourceHz',perBab4FmtNum(channelRate('perception_performance'),2));setText('perBab4PerfGate',gate.text);const g=$('perBab4PerfGate');if(g)g.className=gate.ok?'ok':'check';const body=$('perBab4PerfSecondBody');if(body){const rows=perBab4PerfSecondRows().slice(-12);body.innerHTML=rows.length?rows.map(r=>`<tr>${[r[0],r[1],r[2],r[3],r[4],r[5],r[7]].map(v=>`<td>${escapeHtml(v)}</td>`).join('')}</tr>`).join(''):'<tr><td colspan="7">Belum ada sampel performance.</td></tr>'}}
function perBab4RefreshReportExtensions(){if(!perBab4Is())return;if(selectedExperiment.id==='F4.2')perBab4RefreshNegative();else if(selectedExperiment.id==='F4.3')perBab4RefreshHys();else if(selectedExperiment.id==='F4.4')perBab4RefreshPerfExt()}
const perBab4ReportRefreshLiveBase=perBab4RefreshLive;
perBab4RefreshLive=function(){perBab4ReportRefreshLiveBase();perBab4RefreshReportExtensions()};
const perBab4ReportRenderPanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){perBab4ReportRenderPanelBase();if(perBab4Is())perBab4RenderReportExtensions()};

const perBab4ReportSurfaceRenderBase=renderSelectedExperiment;
renderSelectedExperiment=function(){
  const r=perBab4ReportSurfaceRenderBase(),finalSurface=perBab4Is()&&activeDomain==='perception'&&(activePage==='experiments'||activeWorkspaceTab?.perception==='tune');
  document.body.classList.toggle('per-bab4-final-mode',finalSurface);
  if(finalSurface)document.body.dataset.bab4=String(selectedExperiment?.id||'');else delete document.body.dataset.bab4;
  return r;
};


/* ===== BAB IV AUTO VISUAL EVIDENCE PACKAGE v1 =====
 * Perception-only extension. Every BAB IV Capture stores the camera frame in memory.
 * STOP + SAVE exports report tables (existing path), table-derived graphs, and compact
 * montage PNG evidence. No navigation / ESC / other GUI tab is touched here.
 */
perBab4.evidence=perBab4.evidence||{f42:[],f43:[],exports:[],captureWarnings:0};
function perBab4EvidenceReset(){perBab4.evidence={f42:[],f43:[],exports:[],captureWarnings:0};perBab4RefreshEvidencePackage()}
function perBab4EvidenceLaneColor(status){status=String(status||'').toUpperCase();if(status==='GREEN')return '#48e0a4';if(status==='YELLOW')return '#ffc86b';if(status==='RED')return '#ff6677';return '#8fa8a9'}
function perBab4EvidenceText(ctx,text,x,y,size=18,color='#e5f4f3',weight=700){ctx.fillStyle=color;ctx.font=`${weight} ${size}px system-ui,-apple-system,sans-serif`;ctx.fillText(String(text),x,y)}
function perBab4EvidenceFrameCanvas(meta={}){
  const img=$('expCameraImage');if(!img||!img.complete||!img.naturalWidth||!img.naturalHeight)return null;
  const maxW=720,scale=Math.min(1,maxW/img.naturalWidth),w=Math.max(320,Math.round(img.naturalWidth*scale)),fh=Math.max(180,Math.round(img.naturalHeight*scale)),footer=96,c=document.createElement('canvas');c.width=w;c.height=fh+footer;const ctx=c.getContext('2d');
  ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);ctx.drawImage(img,0,0,w,fh);
  if(meta.kind==='detection'){
    const sx=w/img.naturalWidth,sy=fh/img.naturalHeight,dets=meta.detections||[];
    dets.forEach((d,i)=>{const norm=Math.max(Math.abs(+d.x1||0),Math.abs(+d.x2||0),Math.abs(+d.y1||0),Math.abs(+d.y2||0))<=1.2,x1=(norm?(+d.x1||0)*img.naturalWidth:(+d.x1||0))*sx,y1=(norm?(+d.y1||0)*img.naturalHeight:(+d.y1||0))*sy,x2=(norm?(+d.x2||0)*img.naturalWidth:(+d.x2||0))*sx,y2=(norm?(+d.y2||0)*img.naturalHeight:(+d.y2||0))*sy,selected=i===meta.selectedIndex;ctx.strokeStyle=selected?'#ffc86b':'#48e0a4';ctx.lineWidth=selected?5:3;ctx.strokeRect(x1,y1,Math.max(1,x2-x1),Math.max(1,y2-y1));ctx.fillStyle=ctx.strokeStyle;ctx.font='700 16px ui-monospace,SFMono-Regular,Menlo,monospace';ctx.fillText(`#${i+1} c${d.class_id??'--'} ${Number.isFinite(+d.score)?(+d.score).toFixed(2):'--'}`,Math.max(4,x1+5),Math.max(20,y1-5))});
  }
  ctx.fillStyle='rgba(4,12,14,.96)';ctx.fillRect(0,fh,w,footer);perBab4EvidenceText(ctx,meta.title||'BAB IV evidence',18,fh+28,20,'#e5f4f3',800);perBab4EvidenceText(ctx,meta.subtitle||'',18,fh+55,15,'#9dc0bd',600);perBab4EvidenceText(ctx,new Date().toISOString(),18,fh+80,13,'#708f8d',500);
  if(meta.kind==='lane'){
    const chip=(label,status,x)=>{const col=perBab4EvidenceLaneColor(status);ctx.fillStyle='rgba(4,12,14,.82)';ctx.fillRect(x,14,210,58);ctx.strokeStyle=col;ctx.lineWidth=3;ctx.strokeRect(x,14,210,58);perBab4EvidenceText(ctx,label,x+12,36,13,'#9dc0bd',700);perBab4EvidenceText(ctx,status||'--',x+12,61,20,col,900)};chip('LEFT',meta.left,14);chip('RIGHT',meta.right,238);ctx.fillStyle='rgba(4,12,14,.82)';ctx.fillRect(Math.max(14,w-370),14,356,58);perBab4EvidenceText(ctx,'RECOMMENDATION',Math.max(26,w-358),36,12,'#9dc0bd',700);perBab4EvidenceText(ctx,meta.recommendation||'--',Math.max(26,w-358),61,18,'#e5f4f3',900)
  }
  return c;
}
function perBab4EvidenceJpeg(meta){try{const c=perBab4EvidenceFrameCanvas(meta);return c?c.toDataURL('image/jpeg',.82):null}catch(e){console.warn('BAB4 evidence frame failed',e);return null}}
function perBab4LoadEvidenceImage(url){return new Promise((resolve,reject)=>{const im=new Image();im.onload=()=>resolve(im);im.onerror=reject;im.src=url})}
function perBab4DrawImageContain(ctx,img,x,y,w,h){const s=Math.min(w/img.naturalWidth,h/img.naturalHeight),dw=img.naturalWidth*s,dh=img.naturalHeight*s;ctx.fillStyle='#03090a';ctx.fillRect(x,y,w,h);ctx.drawImage(img,x+(w-dw)/2,y+(h-dh)/2,dw,dh)}
function perBab4RepresentativeDetection(target,dist){const all=perBab4.evidence.f42.filter(x=>x.target===target&&+x.distance===+dist&&x.dataUrl);if(!all.length)return null;const good=all.filter(x=>x.success&&Number.isFinite(+x.confidence)).sort((a,b)=>(+a.confidence)-(+b.confidence));if(good.length)return good[Math.floor((good.length-1)/2)];const success=all.filter(x=>x.success);return (success.length?success:all)[Math.floor(((success.length?success:all).length-1)/2)]}
function perBab4RepresentativeLane(key){const all=perBab4.evidence.f43.filter(x=>x.key===key&&x.dataUrl);if(!all.length)return null;const good=all.filter(x=>x.match);return (good.length?good:all)[Math.floor(((good.length?good:all).length-1)/2)]}
async function perBab4BuildDetectionMontage(target){
  const reps=[1,2,3,4,5].map(d=>perBab4RepresentativeDetection(target,d));if(!reps.some(Boolean))return null;
  const c=document.createElement('canvas');c.width=1280;c.height=800;const ctx=c.getContext('2d');ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);perBab4EvidenceText(ctx,`Bukti Visual ${target==='human'?'MANUSIA':'SEPEDA MOTOR'} • Variasi Jarak 1–5 m`,54,58,30,'#e5f4f3',900);perBab4EvidenceText(ctx,'Satu frame representatif dipilih dari 20 ulangan pada setiap jarak (median confidence dari trial berhasil).',54,91,17,'#8fa8a9',600);
  const pos=[[35,105],[450,105],[865,105],[242,438],[657,438]],pw=380,ph=300,imgH=226;
  for(let i=0;i<5;i++){const [x,y]=pos[i],rep=reps[i];ctx.fillStyle='#0b1b1d';ctx.fillRect(x,y,pw,ph);ctx.strokeStyle=rep?'rgba(72,224,164,.45)':'rgba(255,200,107,.4)';ctx.lineWidth=2;ctx.strokeRect(x,y,pw,ph);perBab4EvidenceText(ctx,`${i+1} m`,x+16,y+30,22,'#e5f4f3',900);if(rep){try{const im=await perBab4LoadEvidenceImage(rep.dataUrl);perBab4DrawImageContain(ctx,im,x+10,y+44,pw-20,imgH)}catch(_){perBab4EvidenceText(ctx,'FRAME ERROR',x+185,y+200,18,'#ff6677',800)}const status=rep.success?'BERHASIL':'GAGAL',conf=rep.success&&Number.isFinite(+rep.confidence)?` • conf ${(+rep.confidence).toFixed(3)}`:'';perBab4EvidenceText(ctx,`Trial ${rep.repeat} • ${status}${conf}`,x+14,y+286,13,rep.success?'#48e0a4':'#ff6677',800)}else{perBab4EvidenceText(ctx,'Belum ada frame evidence',x+86,y+175,15,'#8fa8a9',700)}}
  perBab4EvidenceText(ctx,'Generated automatically by GUI Persepsi • BAB IV 4.2',35,778,13,'#708f8d',600);return c;
}
async function perBab4BuildLaneMontage(){
  const spec=[['gray','Abu-abu'],['green','Hijau'],['yellow-left','Kuning kiri'],['yellow-right','Kuning kanan'],['red-left','Merah kiri'],['red-right','Merah kanan']],reps=spec.map(x=>perBab4RepresentativeLane(x[0]));if(!reps.some(Boolean))return null;
  const c=document.createElement('canvas');c.width=1280;c.height=830;const ctx=c.getContext('2d');ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);perBab4EvidenceText(ctx,'Bukti Visual Lane Safety • Enam Kondisi Representatif',54,58,30,'#e5f4f3',900);perBab4EvidenceText(ctx,'Abu-abu, hijau, serta sisi kiri/kanan untuk kuning dan merah dipilih dari trial aktual.',54,91,17,'#8fa8a9',600);const pw=390,ph=320,imgH=238,gap=20;
  for(let i=0;i<6;i++){const col=i%3,row=Math.floor(i/3),x=35+col*(pw+gap),y=105+row*(ph+30),rep=reps[i];ctx.fillStyle='#0b1b1d';ctx.fillRect(x,y,pw,ph);ctx.strokeStyle=rep?'rgba(91,181,173,.42)':'rgba(255,200,107,.4)';ctx.lineWidth=2;ctx.strokeRect(x,y,pw,ph);perBab4EvidenceText(ctx,spec[i][1],x+16,y+30,22,'#e5f4f3',900);if(rep){try{const im=await perBab4LoadEvidenceImage(rep.dataUrl);perBab4DrawImageContain(ctx,im,x+10,y+44,pw-20,imgH)}catch(_){perBab4EvidenceText(ctx,'FRAME ERROR',x+185,y+210,18,'#ff6677',800)}const colr=rep.match?'#48e0a4':'#ff6677';perBab4EvidenceText(ctx,`Trial ${rep.repeat} • ${rep.left}/${rep.right} • ${rep.recommendation}`,x+14,y+306,12,colr,800)}else{perBab4EvidenceText(ctx,'Belum ada frame evidence',x+88,y+180,15,'#8fa8a9',700)}}
  perBab4EvidenceText(ctx,'Generated automatically by GUI Persepsi • BAB IV 4.3',35,808,13,'#708f8d',600);return c;
}
function perBab4DownloadGeneratedCanvas(canvas,name){if(!canvas)return toast('Montage belum mempunyai frame evidence.',true);canvas.toBlob(blob=>{if(!blob)return toast('Gagal membuat PNG montage.',true);const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=`${name}_${stamp()}.png`;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000)},'image/png')}
async function perBab4PrepareEvidenceExports(){
  perBab4.evidence.exports=[];
  if(selectedExperiment?.id==='F4.2')for(const [target,title] of [['human','Montage Manusia 1-5 m'],['motorcycle','Montage Sepeda Motor 1-5 m']]){const c=await perBab4BuildDetectionMontage(target);if(c)perBab4.evidence.exports.push({index:target==='human'?3:4,title,data_url:c.toDataURL('image/png')})}
  if(selectedExperiment?.id==='F4.3'){const c=await perBab4BuildLaneMontage();if(c)perBab4.evidence.exports.push({index:2,title:'Montage Lane Safety 6 kondisi',data_url:c.toDataURL('image/png')})}
}
function perBab4EvidenceDistanceReady(target){let n=0;for(let d=1;d<=5;d++)if(perBab4.evidence.f42.some(x=>x.target===target&&+x.distance===d&&x.dataUrl))n++;return n}
function perBab4EvidencePackageHtml(){
  if(selectedExperiment?.id==='F4.2')return `<section class="per-bab4-evidence-card per-bab4-save-package" id="perBab4SavePackage"><div class="per-bab4-evidence-head"><div><span class="eyebrow">OUTPUT OTOMATIS SAAT STOP + SAVE</span><b>Tabel + grafik + bukti visual 5 jarak</b><p>Setiap AMBIL DATA menyimpan JPEG + metadata ke folder evidence_frames. STOP + SAVE menambahkan tabel CSV/XLSX, PNG Detection Rate, PNG Confidence, montage Manusia 1–5 m, montage Sepeda Motor 1–5 m, dan manifest dalam satu folder run.</p></div><span class="status-chip" id="perBab4VisualState">0 frame</span></div><div class="per-bab4-package-grid"><div><span>Human 1–5 m</span><b id="perBab4HumanReady">0 / 5 jarak</b></div><div><span>Motorcycle 1–5 m</span><b id="perBab4MotorReady">0 / 5 jarak</b></div><div><span>Visual capture</span><b id="perBab4VisualFrames">0 / 200</b></div><div><span>File STOP + SAVE</span><b>CSV/XLSX + graph + montage siap</b></div></div><div class="per-bab4-package-actions"><button class="soft-btn" id="perBab4PreviewHuman">PNG Montage Human</button><button class="soft-btn" id="perBab4PreviewMotor">PNG Montage Motorcycle</button></div></section>`;
  if(selectedExperiment?.id==='F4.3')return `<section class="per-bab4-evidence-card per-bab4-save-package" id="perBab4SavePackage"><div class="per-bab4-evidence-head"><div><span class="eyebrow">OUTPUT OTOMATIS SAAT STOP + SAVE</span><b>Tabel + grafik + bukti visual Lane Safety</b><p>Setiap AMBIL DATA LANE menyimpan JPEG + metadata ke folder evidence_frames. STOP + SAVE menyatukan tabel CSV/XLSX, PNG Success Rate, montage 6 kondisi, dan manifest dalam satu folder run.</p></div><span class="status-chip" id="perBab4VisualState">0 frame</span></div><div class="per-bab4-package-grid"><div><span>Visual capture</span><b id="perBab4VisualFrames">0 / 80</b></div><div><span>Kondisi montage</span><b id="perBab4LaneReady">0 / 6</b></div><div><span>Output visual</span><b>1 graph + 1 montage</b></div><div><span>Data utama</span><b>Raw + rekap</b></div></div><div class="per-bab4-package-actions"><button class="soft-btn" id="perBab4PreviewLane">PNG Montage Lane Safety</button></div></section>`;
  return '';
}
function perBab4BindEvidencePackage(){
  $('perBab4PreviewHuman')?.addEventListener('click',async()=>perBab4DownloadGeneratedCanvas(await perBab4BuildDetectionMontage('human'),'BAB4_4.2_Human_1-5m_montage'));
  $('perBab4PreviewMotor')?.addEventListener('click',async()=>perBab4DownloadGeneratedCanvas(await perBab4BuildDetectionMontage('motorcycle'),'BAB4_4.2_Motorcycle_1-5m_montage'));
  $('perBab4PreviewLane')?.addEventListener('click',async()=>perBab4DownloadGeneratedCanvas(await perBab4BuildLaneMontage(),'BAB4_4.3_LaneSafety_montage'));
}
function perBab4RefreshEvidencePackage(){
  if(!$('perBab4SavePackage'))return;if(selectedExperiment?.id==='F4.2'){const n=perBab4.evidence.f42.length,h=perBab4EvidenceDistanceReady('human'),m=perBab4EvidenceDistanceReady('motorcycle');setText('perBab4VisualState',`${n} frame`);setText('perBab4HumanReady',`${h} / 5 jarak`);setText('perBab4MotorReady',`${m} / 5 jarak`);setText('perBab4VisualFrames',`${n} / 200`);const chip=$('perBab4VisualState');if(chip)chip.className='status-chip '+(n>=200?'':'waiting')}else if(selectedExperiment?.id==='F4.3'){const n=perBab4.evidence.f43.length,keys=new Set(perBab4.evidence.f43.map(x=>x.key));setText('perBab4VisualState',`${n} frame`);setText('perBab4VisualFrames',`${n} / 80`);setText('perBab4LaneReady',`${keys.size} / 6`);const chip=$('perBab4VisualState');if(chip)chip.className='status-chip '+(n>=80?'':'waiting')}}
const perBab4EvidenceBaseRenderReportExtensions=perBab4RenderReportExtensions;
perBab4RenderReportExtensions=function(){perBab4EvidenceBaseRenderReportExtensions();const host=perBab4ReportExtensionHost(),html=perBab4EvidencePackageHtml();if(host&&html){host.insertAdjacentHTML('afterbegin',html);perBab4BindEvidencePackage();perBab4RefreshEvidencePackage()}};
const perBab4EvidenceBaseRefreshReportExtensions=perBab4RefreshReportExtensions;
perBab4RefreshReportExtensions=function(){perBab4EvidenceBaseRefreshReportExtensions();perBab4RefreshEvidencePackage()};
const perBab4EvidenceBaseDetectionCapture=perBab4DetectionCapture;
perBab4DetectionCapture=function(success){
  const before=perBab4Rows().length,target=$('perBab4Target')?.value||'human',distance=+$('perBab4Distance')?.value||1,repeat=+$('perBab4Repeat')?.value||1,dets=perBab4RawDetections().map(d=>({...d})),selectedIndex=Math.max(0,+$('perBab4Candidate')?.value||0),sel=dets[selectedIndex],confidence=success&&Number.isFinite(+sel?.score)?+sel.score:NaN,dataUrl=perBab4EvidenceJpeg({kind:'detection',detections:dets,selectedIndex,title:`${target==='human'?'MANUSIA':'SEPEDA MOTOR'} • ${distance} m • ulangan ${repeat}`,subtitle:`${success?'BERHASIL':'GAGAL'}${success&&Number.isFinite(confidence)?` • confidence ${confidence.toFixed(3)}`:''}`});
  const result=perBab4EvidenceBaseDetectionCapture(success);if(perBab4Rows().length===before+1){if(dataUrl)perBab4.evidence.f42.push({target,distance,repeat,success,confidence,dataUrl});else perBab4.evidence.captureWarnings++;perBab4RefreshEvidencePackage()}return result;
};
const perBab4EvidenceBaseLaneCapture=perBab4LaneCapture;
perBab4LaneCapture=function(){
  const before=perBab4Rows().length,cond=$('perBab4LaneCondition')?.value||'gray',repeat=+$('perBab4LaneRepeat')?.value||1,side=perBab4LaneProtocolSide(),ls=obj('lane_state'),corr=ls.corridor||{},label={gray:'Abu-abu',green:'Hijau',yellow:'Kuning',red:'Merah'}[cond],left=String(corr.left_status||'--'),right=String(corr.right_status||'--'),recommendation=String(corr.recommendation||'--'),expected=perBab4LaneExpected(cond,side),match=recommendation===expected.rec&&left===expected.left&&right===expected.right,key=(cond==='yellow'||cond==='red')?`${cond}-${side}`:cond,dataUrl=perBab4EvidenceJpeg({kind:'lane',left,right,recommendation,title:`LANE SAFETY • ${label}${(cond==='yellow'||cond==='red')?` ${side==='left'?'KIRI':'KANAN'}`:''} • ulangan ${repeat}`,subtitle:`${left}/${right} • ${recommendation} • ${match?'BENAR':'SALAH'}`});
  const result=perBab4EvidenceBaseLaneCapture();if(perBab4Rows().length===before+1){if(dataUrl)perBab4.evidence.f43.push({key,cond,side,repeat,left,right,recommendation,match,dataUrl});else perBab4.evidence.captureWarnings++;perBab4RefreshEvidencePackage()}return result;
};
const perBab4EvidenceBaseUndo=perBab4Undo;
perBab4Undo=function(){const before=perBab4Rows().length,id=selectedExperiment?.id;const r=perBab4EvidenceBaseUndo();if(perBab4Rows().length===before-1){if(id==='F4.2')perBab4.evidence.f42.pop();if(id==='F4.3')perBab4.evidence.f43.pop();perBab4RefreshEvidencePackage()}return r};
const perBab4EvidenceBaseStart=startWebRecording;
startWebRecording=async function(){const was=!!recordStartedMs;await perBab4EvidenceBaseStart();if(!was&&recordStartedMs&&perBab4Is()&&(selectedExperiment.id==='F4.2'||selectedExperiment.id==='F4.3'))perBab4EvidenceReset()};
const perBab4EvidenceBaseStop=stopWebRecording;
stopWebRecording=async function(){if(recordStartedMs&&perBab4Is()&&(selectedExperiment.id==='F4.2'||selectedExperiment.id==='F4.3')){try{await perBab4PrepareEvidenceExports()}catch(e){console.warn('BAB4 montage prepare failed',e);toast('Data tabel tetap disimpan, tetapi montage gagal dibuat: '+(e?.message||e),true)}}return perBab4EvidenceBaseStop()};
const perBab4EvidenceBaseCollectPng=collectExperimentGraphPngPayload;
collectExperimentGraphPngPayload=function(){if(!perBab4Is()||!['F4.2','F4.3'].includes(selectedExperiment?.id))return perBab4EvidenceBaseCollectPng();const out=[],addCanvas=(id,title,index)=>{const c=$(id);if(c&&c.width&&c.height)out.push({index,title,data_url:c.toDataURL('image/png')})};if(selectedExperiment.id==='F4.2'){perBab4DrawSummary();addCanvas('perBab4SummaryCanvas','Detection Rate terhadap jarak',1);addCanvas('perBab4ConfidenceCanvas','Confidence rata-rata terhadap jarak',2)}else{perBab4DrawSummary();addCanvas('perBab4SummaryCanvas','Success Rate Lane Safety',1)}for(const e of (perBab4.evidence.exports||[]))out.push({index:e.index,title:e.title,data_url:e.data_url});return out};

/* ===== BAB IV 4.2 AUTO TRIAL FLOW v3 =====
 * Scope: Persepsi FINAL 4.2/4.3 only.
 * START opens a session. One AMBIL DATA click freezes one trial, persists one
 * evidence JPEG+metadata on the server, evaluates the raw bbox automatically,
 * then advances repeat/distance/target. Raw YOLOPv2 class IDs are NOT treated
 * as semantic Human/Motorcycle labels; the controlled target is identified by
 * the visible center TARGET zone.
 */
perBab4.autoTrial=perBab4.autoTrial||{busy:false,lastResult:'WAIT START'};
function perBab4AutoFrameGeometry(){
  const rd=obj('raw_detections')||{},img=$('expCameraImage'),w=+rd.image_width||+img?.naturalWidth||0,h=+rd.image_height||+img?.naturalHeight||0;
  return {w,h,cx:w*.5,cy:h*.5,zx:w*.10,zy:h*.12};
}
function perBab4AutoNormalizeBox(d,w,h){
  if(!d||!(w>0&&h>0))return null;let x1=+d.x1,y1=+d.y1,x2=+d.x2,y2=+d.y2;if(![x1,y1,x2,y2].every(Number.isFinite))return null;
  const normalized=Math.max(Math.abs(x1),Math.abs(x2),Math.abs(y1),Math.abs(y2))<=1.2;if(normalized){x1*=w;x2*=w;y1*=h;y2*=h}if(x2<x1)[x1,x2]=[x2,x1];if(y2<y1)[y1,y2]=[y2,y1];return{x1,y1,x2,y2};
}
function perBab4AutoDetectionDecision(){
  const dets=perBab4RawDetections(),g=perBab4AutoFrameGeometry();if(!(g.w>0&&g.h>0))return{success:false,index:-1,candidate:null,reason:'NO FRAME GEOMETRY',dets};
  let best=null;
  dets.forEach((d,index)=>{const b=perBab4AutoNormalizeBox(d,g.w,g.h);if(!b)return;const contains=b.x1<=g.cx&&b.x2>=g.cx&&b.y1<=g.cy&&b.y2>=g.cy,bcx=.5*(b.x1+b.x2),bcy=.5*(b.y1+b.y2),centerNear=Math.abs(bcx-g.cx)<=g.zx&&Math.abs(bcy-g.cy)<=g.zy;if(!contains&&!centerNear)return;
    const ix=Math.max(0,Math.min(b.x2,g.cx+g.zx)-Math.max(b.x1,g.cx-g.zx)),iy=Math.max(0,Math.min(b.y2,g.cy+g.zy)-Math.max(b.y1,g.cy-g.zy)),overlap=(ix*iy)/Math.max(1,4*g.zx*g.zy),score=Number.isFinite(+d.score)?+d.score:0,rank=(contains?10:0)+overlap+score*.05;
    if(!best||rank>best.rank)best={index,candidate:d,rank,contains,overlap};
  });
  return best?{success:true,index:best.index,candidate:best.candidate,reason:best.contains?'BBOX COVERS TARGET':'BBOX CENTER IN TARGET ZONE',dets}:{success:false,index:-1,candidate:null,reason:dets.length?'NO BBOX ON TARGET ZONE':'NO RAW CANDIDATE',dets};
}
async function perBab4PersistTrialEvidence(label){
  try{const r=await writeRequest('/api/perception/evidence',{label}),j=await r.json();if(!r.ok||!j.ok)throw new Error(j.message||`HTTP ${r.status}`);return j.evidence||{};}catch(e){throw new Error(`Evidence image gagal disimpan: ${e.message||e}`)}
}
function perBab4AutoTargetText(){const target=$('perBab4Target')?.value||'human',distance=+$('perBab4Distance')?.value||1,repeat=+$('perBab4Repeat')?.value||1;return`${target==='human'?'MANUSIA':'SEPEDA MOTOR'} • ${distance} m • ${repeat}/20`}
async function perBab4AutoCaptureTrial(){
  if(perBab4.autoTrial.busy)return;if(!recordStartedMs)return toast('Klik START SESI dahulu.',true);if(!channelFresh('raw_detections',2.5))return toast('Raw detection stream belum fresh. Tunggu source READY.',true);
  const target=$('perBab4Target')?.value||'human',distance=+$('perBab4Distance')?.value||1,repeat=+$('perBab4Repeat')?.value||1,decision=perBab4AutoDetectionDecision(),dets=decision.dets.map(d=>({...d})),sel=decision.candidate,success=decision.success,confidence=success&&Number.isFinite(+sel?.score)?+sel.score:NaN,backend=obj('raw_detections').backend||obj('perception_performance').backend||'--',classes=dets.map(d=>d.class_id).filter(v=>v!==undefined).join('|')||'--',note=$('perBab4Note')?.value?.trim()||'',label=`F4.2_${target}_${distance}m_trial_${String(repeat).padStart(2,'0')}_${success?'BERHASIL':'GAGAL'}`;
  perBab4.autoTrial.busy=true;const btn=$('perBab4AutoCapture');if(btn){btn.disabled=true;btn.textContent='MENYIMPAN FRAME...'}
  try{
    const ev=await perBab4PersistTrialEvidence(label),evidencePath=ev.image_path||'--',dataUrl=perBab4EvidenceJpeg({kind:'detection',detections:dets,selectedIndex:Math.max(0,decision.index),title:`${target==='human'?'MANUSIA':'SEPEDA MOTOR'} • ${distance} m • ulangan ${repeat}`,subtitle:`AUTO ${success?'BERHASIL':'GAGAL'} • ${decision.reason}${success&&Number.isFinite(confidence)?` • confidence ${confidence.toFixed(3)}`:''}`});
    perBab4Push(0,[perBab4Rows().length+1,target,distance,repeat,perBab4FrameTimestamp(),backend,dets.length,success?confidence.toFixed(5):'--',success?'BERHASIL':'GAGAL',success?String(sel?.class_id??'--'):classes,note,decision.reason,evidencePath]);
    if(dataUrl)perBab4.evidence.f42.push({target,distance,repeat,success,confidence,dataUrl,evidencePath});else perBab4.evidence.captureWarnings++;
    perBab4.autoTrial.lastResult=`${success?'BERHASIL':'GAGAL'} • ${decision.reason}${success&&Number.isFinite(confidence)?` • conf ${confidence.toFixed(3)}`:''}`;perBab4AdvanceDetection(target,distance,repeat);perBab4RefreshEvidencePackage();perBab4RefreshAutoTrialUi();toast(`${target==='human'?'Manusia':'Sepeda motor'} ${distance} m #${repeat}: ${success?'BERHASIL':'GAGAL'} • frame tersimpan`,!success);
  }catch(e){perBab4.autoTrial.lastResult='SAVE ERROR';toast(e.message||String(e),true)}finally{perBab4.autoTrial.busy=false;if(btn){btn.disabled=false;btn.textContent='📷 AMBIL DATA'}perBab4RefreshAutoTrialUi()}
}
function perBab4RefreshAutoTrialUi(){
  if(selectedExperiment?.id!=='F4.2')return;setText('perBab4AutoCurrent',perBab4AutoTargetText());setText('perBab4AutoResult',perBab4.autoTrial.lastResult||'WAIT START');const d=perBab4AutoDetectionDecision();setText('perBab4AutoPreview',d.success?`READY • candidate #${d.index+1} • ${d.reason}`:`${d.dets.length} raw candidate • ${d.reason}`);const b=$('perBab4AutoCapture');if(b)b.disabled=!recordStartedMs||perBab4.autoTrial.busy||!channelFresh('raw_detections',2.5)
}
perBab4DetectionControls=function(){return `<div class="per-bab4-auto-guide"><b>MEKANISME 1 TRIAL</b><span>Posisikan target fisik pada crosshair <strong>TARGET</strong> di kamera → klik <strong>AMBIL DATA</strong>. GUI otomatis menilai raw bbox pada target-zone, menyimpan tabel + JPEG evidence, lalu maju ke ulangan berikutnya. Raw class ID tidak dipakai sebagai label Human/Motorcycle.</span></div><div class="per-bab4-controls"><label>Target fisik<select id="perBab4Target"><option value="human">Manusia</option><option value="motorcycle">Sepeda motor</option></select></label><label>Jarak GT<select id="perBab4Distance">${[1,2,3,4,5].map(x=>`<option value="${x}">${x} meter</option>`).join('')}</select></label><label>Ulangan<input id="perBab4Repeat" type="number" min="1" max="20" step="1" value="1" readonly></label><label class="wide">Catatan visual<input id="perBab4Note" type="text" placeholder="opsional: occlusion / bayangan / kondisi khusus"></label><div class="per-bab4-auto-status"><div><span>Trial aktif</span><b id="perBab4AutoCurrent">--</b></div><div><span>Evaluasi live</span><b id="perBab4AutoPreview">--</b></div><div><span>Hasil terakhir</span><b id="perBab4AutoResult">WAIT START</b></div></div><div class="per-bab4-actions"><button class="primary-btn per-bab4-capture-main" id="perBab4AutoCapture">📷 AMBIL DATA</button><button class="soft-btn" id="perBab4Undo">Undo trial terakhir</button></div></div>`}
const perBab4AutoBaseInfoText=perBab4InfoText;
perBab4InfoText=function(){if(selectedExperiment?.id==='F4.2')return '200 trial: START membuka sesi; AMBIL DATA merekam tepat satu trial. Hasil BERHASIL/GAGAL ditentukan otomatis dari raw bbox terhadap target-zone kamera, bukan dipilih operator.';return perBab4AutoBaseInfoText()};
const perBab4AutoBaseStepText=perBab4StepText;
perBab4StepText=function(){if(selectedExperiment?.id==='F4.2')return ['Tujuan','Ambil 200 trial objektif: 2 target × 5 jarak × 20 ulangan.','Yang dilakukan','START SESI → posisikan target pada crosshair → klik AMBIL DATA satu kali per ulangan → GUI simpan frame + nilai otomatis → lanjut sampai 200/200.','Output laporan','STOP + SAVE membuat satu folder run berisi CSV/XLSX, grafik Detection Rate/Confidence, montage 5 jarak, manifest, dan evidence frame.'];return perBab4AutoBaseStepText()};
const perBab4AutoBaseRenderPanel=perBab4RenderPanel;
perBab4RenderPanel=function(){const r=perBab4AutoBaseRenderPanel();if(selectedExperiment?.id==='F4.2'){$('perBab4AutoCapture')?.addEventListener('click',perBab4AutoCaptureTrial);$('perBab4Target')?.addEventListener('change',perBab4RefreshAutoTrialUi);$('perBab4Distance')?.addEventListener('change',perBab4RefreshAutoTrialUi);perBab4RefreshAutoTrialUi()}return r};
const perBab4AutoBaseRefreshLive=perBab4RefreshLive;
perBab4RefreshLive=function(){perBab4AutoBaseRefreshLive();perBab4RefreshAutoTrialUi()};
const perBab4AutoBaseDrawOverlay=perBab4DrawOverlay;
perBab4DrawOverlay=function(){perBab4AutoBaseDrawOverlay();if(selectedExperiment?.id!=='F4.2')return;const c=$('perBab4DetectionOverlay'),img=$('expCameraImage'),g=cameraGeometryFor(img);if(!c||c.hidden||!g)return;const fit=g.resizeCanvas(c),ctx=fit.ctx,dpr=fit.dpr,cr=c.getBoundingClientRect(),p=g.normalizedToScreen(.5,.5);if(!p)return;const x=(p.x-cr.left)*dpr,y=(p.y-cr.top)*dpr,rx=Math.max(18,cr.width*.10)*dpr,ry=Math.max(18,cr.height*.12)*dpr;ctx.save();ctx.setLineDash([8*dpr,5*dpr]);ctx.strokeStyle='rgba(255,200,107,.95)';ctx.lineWidth=2*dpr;ctx.strokeRect(x-rx,y-ry,2*rx,2*ry);ctx.setLineDash([]);ctx.beginPath();ctx.moveTo(x-14*dpr,y);ctx.lineTo(x+14*dpr,y);ctx.moveTo(x,y-14*dpr);ctx.lineTo(x,y+14*dpr);ctx.stroke();ctx.fillStyle='#ffc86b';ctx.font=`800 ${12*dpr}px ui-monospace`;ctx.fillText('TARGET',x+18*dpr,y-8*dpr);ctx.restore()};
const perBab4AutoBaseSetRecordingUi=setRecordingUi;
setRecordingUi=function(active,info={}){perBab4AutoBaseSetRecordingUi(active,info);if(perBab4Is('F4.2')){const b=$('recordToggleBtn');if(b)b.textContent=active?'■ STOP + SAVE FOLDER':'● START SESI DATA';perBab4RefreshAutoTrialUi()}else if(perBab4Is('F4.3')){const b=$('recordToggleBtn');if(b)b.textContent=active?'■ STOP + SAVE FOLDER':'● START SESI DATA'}};

/* Persist 4.3 lane trial evidence into the same BAB IV run folder. */
perBab4.autoTrial.laneBusy=false;
perBab4LaneControls=function(){return `<div class="per-bab4-auto-guide"><b>MEKANISME 1 TRIAL</b><span>Stabilkan kondisi lane sesuai acuan → klik <strong>AMBIL DATA LANE</strong>. GUI otomatis membandingkan status kiri/kanan + recommendation, menyimpan JPEG evidence, lalu maju ke ulangan berikutnya.</span></div><div class="per-bab4-controls"><label>Kondisi acuan<select id="perBab4LaneCondition"><option value="gray">Abu-abu • no mask evidence</option><option value="green">Hijau • clear</option><option value="yellow">Kuning • warning</option><option value="red">Merah • intrusion</option></select></label><label>Sisi uji<select id="perBab4LaneSide"><option value="left">Kiri</option><option value="right">Kanan</option></select></label><label>Ulangan<input id="perBab4LaneRepeat" type="number" min="1" max="20" step="1" value="1" readonly></label><div class="per-bab4-actions"><button class="primary-btn per-bab4-capture-main" id="perBab4LaneCapture">📷 AMBIL DATA LANE</button><button class="soft-btn" id="perBab4Undo">Undo trial terakhir</button></div></div>`}
perBab4LaneCapture=async function(){
  if(perBab4.autoTrial.laneBusy)return;if(!recordStartedMs)return toast('Klik START SESI dahulu.',true);if(!channelFresh('lane_state',2.5))return toast('Lane Safety stream belum fresh.',true);
  const cond=$('perBab4LaneCondition')?.value||'gray',rep=+$('perBab4LaneRepeat')?.value||1,side=(cond==='yellow'||cond==='red')?perBab4LaneProtocolSide():($('perBab4LaneSide')?.value||'left'),ls=obj('lane_state'),c=ls.corridor||{},exp=perBab4LaneExpected(cond,side),actual=String(c.recommendation||'--'),left=String(c.left_status||'--'),right=String(c.right_status||'--'),match=actual===exp.rec&&left===exp.left&&right===exp.right,label={gray:'Abu-abu',green:'Hijau',yellow:'Kuning',red:'Merah'}[cond],key=(cond==='yellow'||cond==='red')?`${cond}-${side}`:cond,evidenceLabel=`F4.3_${key}_trial_${String(rep).padStart(2,'0')}_${match?'BENAR':'SALAH'}`;
  perBab4.autoTrial.laneBusy=true;const btn=$('perBab4LaneCapture');if(btn){btn.disabled=true;btn.textContent='MENYIMPAN FRAME...'}
  try{
    const ev=await perBab4PersistTrialEvidence(evidenceLabel),evidencePath=ev.image_path||'--',dataUrl=perBab4EvidenceJpeg({kind:'lane',left,right,recommendation:actual,title:`LANE SAFETY • ${label}${(cond==='yellow'||cond==='red')?` ${side==='left'?'KIRI':'KANAN'}`:''} • ulangan ${rep}`,subtitle:`${left}/${right} • ${actual} • ${match?'BENAR':'SALAH'}`});
    perBab4Push(0,[perBab4Rows().length+1,label,cond==='gray'||cond==='green'?'N/A':side,rep,perBab4FrameTimestamp(),String(ls.drivable_valid??ls.valid??'--'),Number.isFinite(+c.left_gap_px)?(+c.left_gap_px).toFixed(2):'--',Number.isFinite(+c.right_gap_px)?(+c.right_gap_px).toFixed(2):'--',left,right,String(c.left_recenter_latched??'--'),String(c.right_recenter_latched??'--'),actual,`${exp.rec} • ${exp.left}/${exp.right}`,match?'BENAR':'SALAH',evidencePath]);
    if(dataUrl)perBab4.evidence.f43.push({key,cond,side,repeat:rep,left,right,recommendation:actual,match,dataUrl,evidencePath});else perBab4.evidence.captureWarnings++;
    let nr=rep+1,nc=cond;if(nr>20){nr=1;nc={gray:'green',green:'yellow',yellow:'red',red:'gray'}[cond]}if($('perBab4LaneRepeat'))$('perBab4LaneRepeat').value=String(nr);if($('perBab4LaneCondition'))$('perBab4LaneCondition').value=nc;perBab4SyncLaneSide();perBab4RefreshEvidencePackage();toast(`${label} #${rep}: ${match?'BENAR':'SALAH'} • frame tersimpan`,!match);
  }catch(e){toast(e.message||String(e),true)}finally{perBab4.autoTrial.laneBusy=false;if(btn){btn.disabled=false;btn.textContent='📷 AMBIL DATA LANE'}}
};


/* ===== BAB IV 4.1 ONE-CLICK IMPLEMENTATION SNAPSHOT v1 =====
 * Scope: GUI Persepsi FINAL 4.1 only. One click opens a short recorder session,
 * captures one annotated camera evidence frame, freezes runtime/config values,
 * then STOP+SAVE produces one BAB4_F4.1 folder with CSV/XLSX/PNG/manifest/evidence.
 */
perBab4.f41=perBab4.f41||{busy:false,lastState:'READY',lastFolder:'',evidencePath:'',canvas:null};
function perBab4F41ApplySchema(){
  const x=(experiments.perception||[]).find(v=>v.id==='F4.1');if(!x)return;
  Object.assign(x,{section:'FINAL 4.1 Implementasi Sistem Persepsi Visual — 1 klik snapshot runtime',reportMode:true,bab4Protocol:'implementation',bab4ManualCapture:true,bab4NoTimeAxis:true,
    tableColumns:[['Parameter Implementasi','Nilai Aktual','Sumber','Status']],tableNames:['Snapshot Implementasi Sistem Persepsi Visual'],
    graphCaptions:['Bukti Implementasi Runtime'],graphs:[{type:'time_series',series:['Camera capture FPS'],xLabel:'Snapshot',yLabel:'Evidence'}],
    liveSeries:{'Camera capture FPS':'perception_performance.capture_fps','Pipeline FPS':'perception_performance.pipeline_fps'}});
  if(selectedExperiment?.id==='F4.1')selectedExperiment=x;
}
const perBab4F41BasePatchCatalog=patchPerceptionBab4Catalog;
patchPerceptionBab4Catalog=function(){perBab4F41BasePatchCatalog();perBab4F41ApplySchema()};
perBab4F41ApplySchema();
function perBab4F41Num(...keys){const p=obj('perception_performance')||{};for(const k of keys){const v=+p[k];if(Number.isFinite(v))return v}return NaN}
function perBab4F41Fmt(v,d=2,unit=''){return Number.isFinite(+v)?`${(+v).toFixed(d)}${unit}`:'--'}
function perBab4F41Config(){return configData('perception')?.perception?.ros__parameters||{}}
function perBab4F41RuntimeRows(evidencePath='--'){
  const cfg=perBab4F41Config(),p=obj('perception_performance')||{},rd=obj('raw_detections')||{},lane=obj('lane_state')||{},corr=lane.corridor||{};
  const cam=bool(raw('connected.camera')),healthy=bool(raw('camera_healthy')),inf=perBab4InferenceOn(),rawFresh=channelFresh('raw_detections',2.5),laneFresh=channelFresh('lane_state',2.5);
  const cap=perBab4F41Num('capture_fps','camera_fps'),pipe=perBab4F41Num('pipeline_fps','fps'),infer=perBab4F41Num('inference_ms'),process=perBab4F41Num('pipeline_ms_per_frame','mean_ms');
  const cfgVal=(v,suffix='')=>v===undefined||v===null?'--':`${v}${suffix}`;
  return [
    ['Timestamp snapshot',new Date().toISOString(),'GUI Persepsi','INFO'],
    ['Camera connected',cam?'ONLINE':'OFFLINE','connected.camera',cam?'OK':'CHECK'],
    ['Camera health',healthy?'HEALTHY':'WAIT','camera_healthy',healthy?'OK':'CHECK'],
    ['Resolusi RGB',`${cfgVal(cfg.rgb_width)} × ${cfgVal(cfg.rgb_height)}`,'YAML perception','CONFIG'],
    ['Camera FPS request',cfgVal(cfg.fps,' FPS'),'YAML perception','CONFIG'],
    ['Capture FPS aktual',perBab4F41Fmt(cap,2,' FPS'),'/perception/performance',Number.isFinite(cap)&&cap>0?'MEASURED':'CHECK'],
    ['Backend inferensi',String(p.backend||'--').toUpperCase(),'/perception/performance',String(p.backend||'').toLowerCase()==='cpu'?'OK':'CHECK'],
    ['YOLOPv2 inference',inf?'ON':'OFF','runtime inference state',inf?'OK':'CHECK'],
    ['Target inference FPS',cfgVal(cfg.cpu_inference_fps,' FPS'),'YAML perception','CONFIG'],
    ['Pipeline FPS aktual',perBab4F41Fmt(pipe,2,' FPS'),'/perception/performance',Number.isFinite(pipe)&&pipe>0?'MEASURED':'CHECK'],
    ['Inference time',perBab4F41Fmt(infer,2,' ms'),'/perception/performance',Number.isFinite(infer)?'MEASURED':'CHECK'],
    ['Process total',perBab4F41Fmt(process,2,' ms'),'/perception/performance',Number.isFinite(process)?'MEASURED':'CHECK'],
    ['CPU threads',cfgVal(cfg.cpu_threads),'YAML perception','CONFIG'],
    ['Web preview FPS',cfgVal(cfg.web_preview_fps,' FPS'),'YAML perception','CONFIG'],
    ['Raw detection stream',rawFresh?`FRESH • ${rd.count??0} candidate`:`STALE • age ${perBab4F41Fmt(age('raw_detections'),2,' s')}`,'/perception/raw_detections',rawFresh?'OK':'CHECK'],
    ['Lane Safety stream',laneFresh?`FRESH • ${corr.recommendation||lane.state||'ACTIVE'}`:`STALE • age ${perBab4F41Fmt(age('lane_state'),2,' s')}`,'/perception/lane_safety_state',laneFresh?'OK':'CHECK'],
    ['Evidence frame',evidencePath,'camera annotated preview',evidencePath&&evidencePath!=='--'?'SAVED':'CHECK']
  ];
}
function perBab4F41BuildCanvas(rows=perBab4F41RuntimeRows()){
  const c=document.createElement('canvas');c.width=1280;c.height=720;const ctx=c.getContext('2d'),img=$('expCameraImage');
  ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);perBab4EvidenceText(ctx,'BAB IV 4.1 • Implementasi Sistem Persepsi Visual',38,48,28,'#e5f4f3',900);perBab4EvidenceText(ctx,'Snapshot runtime satu kali klik • data aktual + konfigurasi efektif',38,78,15,'#8fa8a9',600);
  const ix=38,iy=105,iw=805,ih=540;ctx.fillStyle='#031012';ctx.fillRect(ix,iy,iw,ih);ctx.strokeStyle='rgba(72,224,164,.28)';ctx.lineWidth=2;ctx.strokeRect(ix,iy,iw,ih);
  if(img&&img.complete&&img.naturalWidth&&img.naturalHeight){const s=Math.min(iw/img.naturalWidth,ih/img.naturalHeight),dw=img.naturalWidth*s,dh=img.naturalHeight*s;ctx.drawImage(img,ix+(iw-dw)/2,iy+(ih-dh)/2,dw,dh)}else{perBab4EvidenceText(ctx,'CAMERA FRAME BELUM TERSEDIA',ix+210,iy+275,20,'#ffc86b',800)}
  const wanted=['Camera connected','Camera health','Resolusi RGB','Capture FPS aktual','Backend inferensi','YOLOPv2 inference','Pipeline FPS aktual','Inference time','CPU threads','Raw detection stream','Lane Safety stream'];let y=118;
  for(const name of wanted){const r=rows.find(v=>v[0]===name);if(!r)continue;const ok=['OK','MEASURED','CONFIG','SAVED'].includes(r[3]),col=ok?'#48e0a4':'#ffc86b';ctx.fillStyle='rgba(255,255,255,.025)';ctx.fillRect(875,y-18,365,44);perBab4EvidenceText(ctx,r[0],890,y,11,'#779696',700);perBab4EvidenceText(ctx,r[1],890,y+20,15,'#d8eeee',800);ctx.fillStyle=col;ctx.fillRect(1218,y-8,8,8);y+=49}
  perBab4EvidenceText(ctx,new Date().toISOString(),38,690,13,'#708f8d',500);perBab4EvidenceText(ctx,'Generated automatically by GUI Persepsi • F4.1',875,690,13,'#708f8d',600);return c;
}
function perBab4F41TableRows(evidencePath){return perBab4F41RuntimeRows(evidencePath).map(values=>({sec:0,values,yaml:perBab4YamlSnapshot()}))}
function perBab4F41CardHtml(){return `<section class="per-bab4-evidence-card per-bab4-f41-card" id="perBab4F41Card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">ONE-CLICK IMPLEMENTATION SNAPSHOT</span><b>Ambil seluruh data BAB 4.1 dalam satu klik</b><p>GUI membuka sesi singkat, mengambil frame kamera aktual, membekukan parameter runtime + YAML, membuat tabel, lalu otomatis STOP + SAVE ke satu folder.</p></div><span class="status-chip" id="perBab4F41State">READY</span></div><div class="per-bab4-f41-grid"><div><span>Camera</span><b id="perBab4F41Camera">--</b></div><div><span>YOLOPv2</span><b id="perBab4F41Inference">--</b></div><div><span>Pipeline FPS</span><b id="perBab4F41Fps">--</b></div><div><span>Output</span><b>CSV + XLSX + PNG + JPEG + Manifest</b></div></div><button class="primary-btn per-bab4-f41-capture" id="perBab4F41Capture">📸 CAPTURE DATA 4.1 + SAVE FOLDER</button><div class="per-bab4-f41-result"><span>Folder hasil</span><code id="perBab4F41Folder">${escapeHtml(perBab4.f41.lastFolder||'Belum ada capture')}</code></div></section>`}
function perBab4F41RefreshCard(){if(selectedExperiment?.id!=='F4.1'||!$('perBab4F41Card'))return;const p=obj('perception_performance')||{},cap=perBab4F41Num('pipeline_fps','fps'),gate=testPreflightStatus(),ready=gate.ok,state=perBab4.f41.busy?'CAPTURING':perBab4.f41.lastState==='SAVED'?'SAVED':perBab4.f41.lastState==='ERROR'?'ERROR':ready?'READY':'WAIT SOURCE';setText('perBab4F41Camera',`${bool(raw('connected.camera'))?'ONLINE':'OFFLINE'} • ${bool(raw('camera_healthy'))?'HEALTHY':'WAIT'}`);setText('perBab4F41Inference',`${perBab4InferenceOn()?'ON':'OFF'} • ${String(p.backend||'--').toUpperCase()}`);setText('perBab4F41Fps',perBab4F41Fmt(cap,2,' FPS'));setText('perBab4F41State',state);const s=$('perBab4F41State');if(s)s.className='status-chip '+(state==='SAVED'||state==='READY'?'':'waiting');const b=$('perBab4F41Capture');if(b){b.disabled=perBab4.f41.busy||!ready;b.title=ready?'Sekali klik: capture runtime + frame + save folder':`Belum READY: ${gate.reason||'source belum siap'}`;b.textContent=perBab4.f41.busy?'MENYIMPAN DATA 4.1...':'📸 CAPTURE DATA 4.1 + SAVE FOLDER'}setText('perBab4F41Folder',perBab4.f41.lastFolder||(ready?'Belum ada capture':`Menunggu: ${gate.reason||'source'}`))}
async function perBab4F41CaptureOnce(){
  if(perBab4.f41.busy)return;if(selectedExperiment?.id!=='F4.1')return;if(recordStartedMs)return toast('Recorder lain masih aktif. STOP dahulu.',true);
  const gate=testPreflightStatus();if(!gate.ok)return toast(`4.1 belum READY: ${gate.reason}`,true);
  perBab4.f41.busy=true;perBab4.f41.lastState='CAPTURING';perBab4F41RefreshCard();
  try{
    perBab4F41ApplySchema();await startWebRecording();if(!recordStartedMs)throw new Error('Recorder 4.1 tidak berhasil START');
    await new Promise(r=>setTimeout(r,300));
    const ev=await perBab4PersistTrialEvidence(`F4.1_IMPLEMENTASI_${new Date().toISOString().replace(/[:.]/g,'-')}`),evidencePath=ev.image_path||'--';
    tableRunRows.set(0,perBab4F41TableRows(evidencePath));perBab4.f41.evidencePath=evidencePath;perBab4.f41.canvas=perBab4F41BuildCanvas(perBab4F41RuntimeRows(evidencePath));renderExperimentTable();
    await stopWebRecording();if(recordStartedMs)throw new Error('STOP + SAVE 4.1 belum selesai');
    perBab4.f41.lastFolder=lastTrialArtifacts?.report_folder||lastTrialArtifacts?.report_root||lastTrialArtifacts?.primary_csv||'Folder tersimpan';perBab4.f41.lastState='SAVED';toast('BAB 4.1 selesai: tabel + XLSX + PNG + evidence tersimpan dalam satu folder');
  }catch(e){perBab4.f41.lastState='ERROR';if(recordStartedMs&&recordingLeafId==='F4.1'){try{await stopWebRecording()}catch(_){}}toast(`Capture 4.1 gagal: ${e.message||e}`,true)
  }finally{perBab4.f41.busy=false;perBab4F41RefreshCard()}
}
const perBab4F41BaseRenderPanel=perBab4RenderPanel;
perBab4RenderPanel=function(){const r=perBab4F41BaseRenderPanel();if(selectedExperiment?.id==='F4.1'){perBab4F41ApplySchema();const p=$('perceptionBab4Acquisition');if(p&&!$('perBab4F41Card')){p.insertAdjacentHTML('beforeend',perBab4F41CardHtml());$('perBab4F41Capture')?.addEventListener('click',perBab4F41CaptureOnce)}perBab4F41RefreshCard()}return r};
const perBab4F41BaseRefreshLive=perBab4RefreshLive;
perBab4RefreshLive=function(){perBab4F41BaseRefreshLive();perBab4F41RefreshCard()};
const perBab4F41BaseCollectPng=collectExperimentGraphPngPayload;
collectExperimentGraphPngPayload=function(){if(selectedExperiment?.id==='F4.1'&&perBab4.f41.canvas)return[{index:1,title:'Bukti Implementasi Runtime',data_url:perBab4.f41.canvas.toDataURL('image/png')}];return perBab4F41BaseCollectPng()};
const perBab4F41BaseRenderSelected=renderSelectedExperiment;
renderSelectedExperiment=function(){if(currentExp==='perception'&&selectedExperiment?.id==='F4.1')perBab4F41ApplySchema();const r=perBab4F41BaseRenderSelected();const rec=$('recordToggleBtn');if(rec)rec.style.display=perBab4Is('F4.1')?'none':'';return r};
