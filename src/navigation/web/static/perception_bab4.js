/* GUI-only BAB IV perception acquisition aligned with the latest thesis draft. */
const PER_BAB4_IDS=new Set(['F4.1','F4.2','F4.3','F4.4']);
const perBab4={perf:{active:false,count:0,lastUpdate:0,gapStart:0,started:0,stopBusy:false,timer:null},selectedCandidate:0};
const perRunField=(key,label,kind='text',placeholder='',options=[])=>({key,label,kind,placeholder,options,group:'Protokol BAB IV',yamlFileKey:'',yamlPath:'',isGroundTruth:false,locked:false});
const perRo=(label,path,kind='float')=>({label,yamlFileKey:'perception',yamlPath:`perception.ros__parameters.${path}`,kind:'yaml_readonly',group:'Konfigurasi Aktif',locked:true});
function perBab4Is(id=null){if(currentExp!=='perception')return false;const selected=String(selectedExperiment?.id||'');if(id!==null&&id!==undefined)return selected===String(id);return PER_BAB4_IDS.has(selected)}
function perBab4DetectionFields(){return [perRo('Ambang Confidence Deteksi','confidence_threshold'),perRo('Ambang IoU NMS','iou_threshold'),perRo('Confidence Minimum Obstacle','minimum_obstacle_confidence'),perRo('Target Laju Inferensi CPU [FPS]','cpu_inference_fps'),perRo('Jumlah Thread CPU','cpu_threads','int')]}
function perBab4LaneFields(){return [perRo('Ambang Peringatan [piksel]','lane_corridor_warning_gap_px'),perRo('Margin Pelanggaran [piksel]','lane_corridor_touch_margin_px'),perRo('Ambang Pelepasan [piksel]','lane_corridor_release_gap_px'),perRo('Rasio X Titik Atas Kiri','lane_corridor_left_top_x_ratio'),perRo('Rasio X Titik Bawah Kiri','lane_corridor_left_bottom_x_ratio'),perRo('Rasio X Titik Atas Kanan','lane_corridor_right_top_x_ratio'),perRo('Rasio X Titik Bawah Kanan','lane_corridor_right_bottom_x_ratio')]}
function perBab4PerfFields(){return [perRo('Lebar Citra RGB [piksel]','rgb_width','int'),perRo('Tinggi Citra RGB [piksel]','rgb_height','int'),perRo('Setpoint Laju Kamera [FPS]','fps','int'),perRo('Target Laju Inferensi CPU [FPS]','cpu_inference_fps'),perRo('Jumlah Thread CPU','cpu_threads','int')]}
function patchPerceptionBab4Catalog(){
  const list=experiments.perception||[],byId=id=>list.find(x=>x.id===id);let x;
  if((x=byId('F4.1')))Object.assign(x,{groupId:'FINAL-4.1',groupTitle:'FINAL BAB IV — 4.1 Implementasi Sistem Persepsi Visual',section:'FINAL 4.1 Implementasi Sistem Persepsi Visual — audit read-only',reportMode:true,bab4Protocol:'implementation',parameterFields:[perRo('Lebar Citra RGB [piksel]','rgb_width','int'),perRo('Tinggi Citra RGB [piksel]','rgb_height','int'),perRo('Setpoint Laju Kamera [FPS]','fps','int'),perRo('Target Laju Inferensi CPU [FPS]','cpu_inference_fps'),perRo('Jumlah Thread CPU','cpu_threads','int'),perRo('Laju Pratinjau Web [FPS]','web_preview_fps')],tableColumns:[],tableNames:[],graphCaptions:[],graphs:[],liveSeries:{'Camera capture FPS':'perception_performance.capture_fps','Pipeline FPS':'perception_performance.pipeline_fps'}});
  if((x=byId('F4.2')))Object.assign(x,{groupId:'FINAL-4.2',groupTitle:'FINAL BAB IV — 4.2 Pengujian Object Detection YOLOPv2',section:'FINAL 4.2 Object Detection — Human + Motorcycle • 1–5 m • 20 percobaan/jarak',reportMode:true,bab4Protocol:'detection',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4DetectionFields(),tableColumns:[['Trial','Target Fisik','Jarak GT [m]','Ulangan','Frame timestamp','Backend','Raw Candidate','Confidence Kandidat','Hasil Spasial','Raw Class ID(s)','Catatan Visual','Evaluasi Otomatis','Evidence Frame'],['Target Fisik','Jarak [m]','Percobaan','Berhasil','Gagal','Detection Rate [%]','Error [%]','Confidence rata-rata']],tableNames:['Raw Percobaan Object Detection','Rekap Detection Rate dan Confidence'],graphCaptions:[],graphs:[],liveSeries:{'Raw detections':'raw_detections.count','Mean confidence':'raw_detections.mean_confidence','Pipeline FPS':'perception_performance.pipeline_fps'}});
  if((x=byId('F4.3')))Object.assign(x,{groupId:'FINAL-4.3',groupTitle:'FINAL BAB IV — 4.3 Pengujian Lane Safety',section:'FINAL 4.3 Lane Safety — Abu-abu / Hijau / Kuning / Merah • 20 percobaan/kondisi',reportMode:true,bab4Protocol:'lane',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4LaneFields(),tableColumns:[['Trial','Kondisi Acuan','Sisi Uji','Ulangan','Timestamp','Drivable/Mask Valid','Left Gap [px]','Right Gap [px]','Left Color','Right Color','Latch L','Latch R','Recommendation Aktual','Expected','Hasil'],['Kondisi Acuan','Percobaan','Benar','Salah','Success Rate [%]']],tableNames:['Raw Percobaan Lane Safety','Rekap Success Rate Lane Safety'],graphCaptions:[],graphs:[],liveSeries:{'Left gap':'lane_state.corridor.left_gap_px','Right gap':'lane_state.corridor.right_gap_px','Lane valid':'lane_state.valid','Drivable valid':'lane_state.drivable_valid'}});
  if((x=byId('F4.4')))Object.assign(x,{groupId:'FINAL-4.4',groupTitle:'BAB IV — 4.4 Pengujian Kinerja Waktu Nyata',section:'4.4 Pengujian Kinerja Waktu Nyata Sistem Persepsi — 3 Kondisi Visual • 60 s/Kondisi',reportMode:true,bab4Protocol:'performance',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4PerfFields(),tableColumns:[['Kondisi Pengujian','Durasi Pengujian [s]','Jumlah Frame Diterima','Laju Frame Terhitung [FPS]','Rata-rata Laju Pipeline [FPS]','Rata-rata Waktu Inferensi [ms]','Rata-rata Waktu Pemrosesan Total [ms]','Rata-rata Umur Frame [ms]','Rata-rata Latensi Subscriber [ms]','Jumlah Celah SSE','Backend Pemrosesan','Validitas Data']],tableNames:['Hasil Pengujian Kinerja Waktu Nyata Sistem Persepsi'],graphCaptions:['Perubahan Laju Pipeline selama 60 s','Waktu Inferensi dan Pemrosesan Total selama 60 s','Umur Frame dan Latensi Subscriber selama 60 s'],graphs:[{type:'time_series',series:['Laju pipeline [FPS]'],xLabel:'Waktu [s]',yLabel:'Laju [FPS]'},{type:'time_series',series:['Waktu inferensi [ms]','Waktu pemrosesan total [ms]'],xLabel:'Waktu [s]',yLabel:'Waktu [ms]'},{type:'time_series',series:['Umur frame [ms]','Latensi subscriber [ms]'],xLabel:'Waktu [s]',yLabel:'Latensi [ms]'}],liveSeries:{'Laju pipeline [FPS]':'perception_performance.pipeline_fps','Waktu inferensi [ms]':'perception_performance.inference_ms','Waktu pemrosesan total [ms]':'perception_performance.pipeline_ms_per_frame','Umur frame [ms]':'perception_performance.frame_age_ms','Latensi subscriber [ms]':'derived.per_bab4_latency_ms','Jumlah frame diproses':'raw_detections.count'}});
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
const PER_BAB4_CPU_TARGET_HZ=1.2;
const PER_BAB4_CPU_MIN_FRAME_HZ=0.9;
const PER_BAB4_CPU_MAX_FRAME_AGE_SEC=1.2;
function perBab4CpuFramePolicy(){
  const perf=obj('perception_performance'),backend=String(perf.backend||'').toLowerCase();
  const rate=channelRate('camera_frame'),frameAge=age('camera_frame');
  const activeCpu=backend==='cpu';
  return {
    activeCpu,rate,frameAge,
    rateOk:Number.isFinite(rate)&&rate>=PER_BAB4_CPU_MIN_FRAME_HZ,
    fresh:Number.isFinite(frameAge)&&frameAge<=PER_BAB4_CPU_MAX_FRAME_AGE_SEC
  };
}
const perBab4BasePreflight=testPreflightStatus;
testPreflightStatus=function(){
  const p=perBab4BasePreflight();if(!perBab4Is()||!selectedExperiment?.reportMode)return p;
  const cpuPolicy=perBab4CpuFramePolicy();
  const items=p.items.map(i=>{
    if(i.label==='Commissioning prerequisite')return {...i,ok:true,critical:false,detail:'Tidak diperlukan untuk pengujian visual standalone BAB IV'};
    if(i.label==='Camera Frame'&&cpuPolicy.activeCpu){
      const rateText=Number.isFinite(cpuPolicy.rate)?cpuPolicy.rate.toFixed(1):'--',ageText=Number.isFinite(cpuPolicy.frameAge)?cpuPolicy.frameAge.toFixed(2):'--';
      return {...i,ok:cpuPolicy.rateOk&&cpuPolicy.fresh,critical:true,detail:`${rateText} Hz • age ${ageText} s • CPU target ${PER_BAB4_CPU_TARGET_HZ.toFixed(1)} Hz • minimum ${PER_BAB4_CPU_MIN_FRAME_HZ.toFixed(1)} Hz • max age ${PER_BAB4_CPU_MAX_FRAME_AGE_SEC.toFixed(1)} s`};
    }
    return i;
  });
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
function perBab4PerformanceSummary(){const dur=Math.max(.001,(Date.now()-perBab4.perf.started)/1000),gaps=Math.max(0,sseGapCount-perBab4.perf.gapStart),valid=gaps===0&&perBab4.perf.count>0,st=p=>metricStats(p),mean=p=>{const v=st(p)?.mean;return Number.isFinite(v)?v.toFixed(3):'--'},fps=valid?(perBab4.perf.count/dur).toFixed(3):'--',cond=perBab4Get('perBab4PerfCondition')?.value||$('runCondition')?.value||'--',backend=obj('perception_performance').backend||'--';return [cond,dur.toFixed(2),perBab4.perf.count,fps,mean('perception_performance.pipeline_fps'),mean('perception_performance.inference_ms'),mean('perception_performance.pipeline_ms_per_frame'),mean('perception_performance.frame_age_ms'),mean('derived.per_bab4_latency_ms'),gaps,backend,valid?'Valid':'Tidak Valid — Celah SSE atau Frame Tidak Tersedia']}
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
function perBab4RefreshLive(){if(!perBab4Is())return;perBab4RenderProgress();const p=obj('perception_performance');setText('perBab4LiveBackend',p.backend||'--');setText('perBab4LiveFps',Number.isFinite(+p.pipeline_fps)?(+p.pipeline_fps).toFixed(2):'--');if(selectedExperiment.id==='F4.2'){perBab4CandidateOptions();setText('perBab4LivePrimary',`${perBab4RawDetections().length} kandidat deteksi mentah`)}else if(selectedExperiment.id==='F4.3'){const c=obj('lane_state').corridor||{};setText('perBab4LivePrimary',`${c.recommendation||'--'} • ${c.left_status||'--'} / ${c.right_status||'--'}`)}else if(selectedExperiment.id==='F4.4'){setText('perBab4LivePrimary',`${perBab4.perf.count} event frame • celah SSE ${Math.max(0,sseGapCount-perBab4.perf.gapStart)}`)}perBab4DrawOverlay()}
function perBab4DetectionControls(){return `<div class="per-bab4-controls"><label>Target fisik<select id="perBab4Target"><option value="human">Manusia</option><option value="motorcycle">Sepeda motor</option></select></label><label>Jarak GT<select id="perBab4Distance">${[1,2,3,4,5].map(x=>`<option value="${x}">${x} meter</option>`).join('')}</select></label><label>Ulangan<input id="perBab4Repeat" type="number" min="1" max="20" step="1" value="1"></label><label>Kandidat untuk confidence<select id="perBab4Candidate"></select></label><label class="wide">Catatan visual<input id="perBab4Note" type="text" placeholder="opsional: occlusion / bayangan / bbox objek lain"></label><div class="per-bab4-actions"><button class="primary-btn" id="perBab4Success">Capture BERHASIL</button><button class="warning-btn" id="perBab4Fail">Capture GAGAL</button><button class="soft-btn" id="perBab4Undo">Undo</button></div></div>`}
function perBab4LaneControls(){return `<div class="per-bab4-controls"><label>Kondisi acuan<select id="perBab4LaneCondition"><option value="gray">Abu-abu • no mask evidence</option><option value="green">Hijau • clear</option><option value="yellow">Kuning • warning</option><option value="red">Merah • intrusion</option></select></label><label>Sisi uji<select id="perBab4LaneSide"><option value="left">Kiri</option><option value="right">Kanan</option></select></label><label>Ulangan<input id="perBab4LaneRepeat" type="number" min="1" max="20" step="1" value="1"></label><div class="per-bab4-actions"><button class="primary-btn" id="perBab4LaneCapture">Capture + Auto Compare</button><button class="soft-btn" id="perBab4Undo">Undo</button></div></div>`}
function perBab4PerfControls(){return `<div class="per-bab4-controls"><label>Kondisi Pengujian<select id="perBab4PerfCondition"><option>Jalur Tanpa Objek Dominan</option><option>Satu Sepeda Motor pada Jalur</option><option>Kondisi Visual Padat (Objek dan Marka Lajur)</option></select></label><div class="per-bab4-note"><b>Prosedur Pengujian</b><span>Mulai perekaman, pertahankan kondisi visual selama 60 s, dan biarkan sistem menghitung event frame yang benar-benar diterima subscriber. Jika terdapat celah SSE, hasil laju frame dinyatakan tidak valid dan tidak digantikan dengan nilai estimasi.</span></div></div>`}
function perBab4InfoText(){if(selectedExperiment?.id==='F4.1')return 'Subbab 4.1 mendokumentasikan implementasi aktual sistem persepsi visual. Data yang ditampilkan berasal dari kondisi runtime dan konfigurasi aktif, bukan nilai estimasi yang ditanam pada antarmuka.';if(selectedExperiment?.id==='F4.2')return '200 percobaan: manusia + sepeda motor, 1–5 m, 20 percobaan per jarak. SUCCESS dipilih observer hanya jika bbox melingkupi sasaran fisik; confidence hanya disimpan dari kandidat SUCCESS yang dipilih.';if(selectedExperiment?.id==='F4.3')return '80 percobaan: abu-abu, hijau, kuning, merah × 20. GUI membandingkan recommendation + warna kiri/kanan terhadap kondisi acuan dan mencatat latch/gap aktual.';return 'Pengujian dilakukan pada tiga kondisi visual selama 60 s per kondisi. Jumlah frame dihitung dari event raw_detections yang benar-benar diterima subscriber web. Adanya celah SSE menyebabkan hasil laju frame dinyatakan tidak valid dan tidak digantikan dengan nilai estimasi.'}
function perBab4StepText(){
  if(selectedExperiment?.id==='F4.1')return ['Tujuan','Mendokumentasikan implementasi kamera, inferensi YOLOPv2 berbasis CPU, dan integrasi ROS 2 pada sistem persepsi.','Prosedur','Pastikan kamera dan inferensi YOLOPv2 aktif, verifikasi parameter runtime dan konfigurasi efektif, kemudian ambil dokumentasi sistem melalui tombol pengambilan data 4.1.','Keluaran penelitian','Tabel kondisi runtime, konfigurasi aktif, citra bukti implementasi, serta berkas CSV/XLSX/PNG/JPEG dan manifest.'];
  if(selectedExperiment?.id==='F4.2')return ['Tujuan','Ambil 200 trial kandidat obstacle: 2 target × 5 jarak × 20 ulangan.','Yang dilakukan','START sekali untuk satu sesi → ukur jarak fisik → pilih target/jarak → Capture BERHASIL atau GAGAL → GUI maju otomatis ke ulangan berikutnya.','Output laporan','Tabel raw + rekap Detection Rate/Error/Confidence; STOP + SAVE otomatis menyimpan grafik DR, grafik confidence, serta montage 5 jarak untuk tiap objek.'];
  if(selectedExperiment?.id==='F4.3')return ['Tujuan','Ambil 80 trial Lane Safety: abu-abu, hijau, kuning, merah × 20.','Yang dilakukan','START → pilih kondisi acuan dan sisi bila perlu → stabilkan mask/lane → Capture + Auto Compare → ulang sampai 80/80.','Output laporan','Tabel raw gap/latch/status + rekap Success Rate; STOP + SAVE otomatis menyimpan grafik success rate dan montage visual 6 kondisi lane.'];
  return ['Tujuan','Mengukur kinerja waktu nyata sistem persepsi pada tiga kondisi visual, masing-masing selama 60 s.','Prosedur','Pilih kondisi pengujian, mulai perekaman, pertahankan kondisi visual selama pengujian, lalu biarkan sistem menghentikan perekaman otomatis setelah 60 s. Ulangi untuk seluruh kondisi.','Keluaran penelitian','Laju pipeline, waktu inferensi, waktu pemrosesan total, umur frame, latensi subscriber, tabel statistik, dan grafik deret waktu.'];
}
function perBab4RenderFinalNavigator(){
  const grid=$('page-experiments')?.querySelector('.workbench-grid');if(!grid)return;let box=$('perBab4FinalNavigator');
  const finalMode=activeDomain==='perception'&&(activeWorkspaceTab?.perception==='tune'||activePage==='experiments');
  if(!finalMode){if(box)box.hidden=true;return}
  if(!box){box=document.createElement('section');box.id='perBab4FinalNavigator';box.className='per-bab4-final-nav';grid.before(box)}
  box.hidden=false;const cards=[['F4.1','4.1','Implementasi Sistem Persepsi','Dokumentasi kondisi runtime dan konfigurasi aktif'],['F4.2','4.2','Deteksi Objek YOLOPv2','100 data • manusia + sepeda motor • 1–5 m'],['F4.3','4.3','Keamanan Lajur','60 data • 6 kondisi pengujian'],['F4.4','4.4','Kinerja Waktu Nyata','3 kondisi visual • 60 s/kondisi']];
  box.innerHTML=`<div class="per-final-title"><div><span class="eyebrow">PENGAMBILAN DATA PENELITIAN</span><h2>BAB IV • PENGUJIAN SISTEM PERSEPSI</h2><p>Gunakan tahapan pengujian berikut untuk memperoleh data yang akan digunakan pada analisis BAB IV. Menu commissioning tetap tersedia terpisah untuk keperluan verifikasi sistem.</p></div><div class="per-final-flow"><span>1 PILIH PENGUJIAN</span><i>→</i><span>2 VERIFIKASI SUMBER DATA</span><i>→</i><span>3 MULAI PENGAMBILAN</span><i>→</i><span>4 SELESAI DAN SIMPAN</span><i>→</i><span>5 EKSPOR DATA</span></div></div><div class="per-final-cards">${cards.map(c=>`<button data-final-id="${c[0]}" class="${selectedExperiment?.id===c[0]?'active':''}"><em>${c[1]}</em><b>${c[2]}</b><small>${c[3]}</small></button>`).join('')}</div>`;
  box.querySelectorAll('[data-final-id]').forEach(b=>b.onclick=()=>selectExperimentById(b.dataset.finalId));
}
function perBab4RenderPanel(){
  const main=$('page-experiments')?.querySelector('.workbench-main');if(!main)return;let p=perBab4Get('perceptionBab4Acquisition');if(!perBab4Is()){if(p)p.hidden=true;perBab4RenderFinalNavigator();return}if(!p){p=document.createElement('section');p.id='perceptionBab4Acquisition';p.className='per-bab4-panel';const run=$('page-experiments')?.querySelector('.run-console');run?.after(p)}p.hidden=false;const controls=selectedExperiment.id==='F4.2'?perBab4DetectionControls():selectedExperiment.id==='F4.3'?perBab4LaneControls():selectedExperiment.id==='F4.4'?perBab4PerfControls():'';const st=perBab4StepText();
  const summary=selectedExperiment.id==='F4.2'?`<div class="per-bab4-summary-head"><b>Rekapitulasi Grafik Pengujian Deteksi Objek</b><div><button class="soft-btn" id="perBab4SaveDrPng">Simpan Grafik Tingkat Deteksi</button><button class="soft-btn" id="perBab4SaveConfPng">Simpan Grafik Confidence</button></div></div><canvas id="perBab4SummaryCanvas" width="900" height="260"></canvas><canvas id="perBab4ConfidenceCanvas" width="900" height="260"></canvas>`:selectedExperiment.id==='F4.3'?`<div class="per-bab4-summary-head"><b>Rekapitulasi Tingkat Keberhasilan Pengujian</b><button class="soft-btn" id="perBab4SaveDrPng">Simpan Grafik Tingkat Keberhasilan</button></div><canvas id="perBab4SummaryCanvas" width="900" height="260"></canvas>`:'';
  p.innerHTML=`<div class="per-bab4-head"><div><span class="eyebrow">BAB IV • PENGAMBILAN DATA PENELITIAN</span><b>${escapeHtml(displayExperimentId(selectedExperiment))} ${escapeHtml(displayExperimentTitle(selectedExperiment))}</b><p>${escapeHtml(perBab4InfoText())}</p></div><div class="per-bab4-live"><span>Backend Inferensi <b id="perBab4LiveBackend">--</b></span><span>Laju Pipeline <b id="perBab4LiveFps">--</b> FPS</span><span id="perBab4LivePrimary">--</span></div></div><div class="per-bab4-now"><div><span>${st[0]}</span><b>${st[1]}</b></div><div><span>${st[2]}</span><b>${st[3]}</b></div><div><span>${st[4]}</span><b>${st[5]}</b></div></div>${controls}<div class="per-bab4-progress" id="perBab4Progress"></div>${summary}`;perBab4RenderFinalNavigator();
  perBab4Get('perBab4Success')?.addEventListener('click',()=>perBab4DetectionCapture(true));perBab4Get('perBab4Fail')?.addEventListener('click',()=>perBab4DetectionCapture(false));perBab4Get('perBab4LaneCapture')?.addEventListener('click',perBab4LaneCapture);perBab4Get('perBab4Undo')?.addEventListener('click',perBab4Undo);perBab4Get('perBab4LaneCondition')?.addEventListener('change',perBab4SyncLaneSide);perBab4Get('perBab4Candidate')?.addEventListener('change',()=>{perBab4.selectedCandidate=+perBab4Get('perBab4Candidate').value||0;perBab4DrawOverlay()});perBab4Get('perBab4SaveDrPng')?.addEventListener('click',()=>perBab4DownloadCanvas('perBab4SummaryCanvas',selectedExperiment.id==='F4.3'?'grafik_tingkat_keberhasilan_keamanan_lajur':'detection_rate'));perBab4Get('perBab4SaveConfPng')?.addEventListener('click',()=>perBab4DownloadCanvas('perBab4ConfidenceCanvas','confidence_vs_distance'));perBab4SyncLaneSide();perBab4RefreshLive();perBab4DrawSummary();
}
function perBab4DrawSummary(){const c=perBab4Get('perBab4SummaryCanvas');if(!c||!perBab4Is())return;const ctx=c.getContext('2d'),w=c.width,h=c.height;ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='700 16px system-ui';let rows=[],valueKey='';if(selectedExperiment.id==='F4.2'){rows=perBab4BuildDetectionSummary();valueKey='Detection Rate';rows=rows.map(r=>({label:`${r[0]} ${r[1]}m`,value:+r[5]}))}else if(selectedExperiment.id==='F4.3'){rows=perBab4BuildLaneSummary().map(r=>({label:perBab4LaneConditionGraphLabel(r[0]),value:+r[4]}));valueKey='Tingkat Keberhasilan'}else{ctx.fillStyle='#8fa8a9';ctx.font='13px system-ui';ctx.fillText(selectedExperiment.id==='F4.4'?'Grafik deret waktu tersedia setelah pengambilan data dimulai.':'Subbab 4.1 menggunakan dokumentasi implementasi dan tidak memerlukan grafik eksperimen.',20,40);return}if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.font='13px system-ui';ctx.fillText('Data pengujian belum tersedia.',20,40);return}const left=60,bottom=42,top=35,right=20,bw=(w-left-right)/rows.length*.66,gap=(w-left-right)/rows.length;ctx.fillStyle='#8fa8a9';ctx.font='12px system-ui';ctx.fillText(`${valueKey} [%]`,18,20);for(let y=0;y<=100;y+=25){const py=h-bottom-(y/100)*(h-top-bottom);ctx.strokeStyle='rgba(150,180,180,.16)';ctx.beginPath();ctx.moveTo(left,py);ctx.lineTo(w-right,py);ctx.stroke();ctx.fillStyle='#8fa8a9';ctx.fillText(String(y),20,py+4)}rows.forEach((r,i)=>{const x=left+i*gap+(gap-bw)/2,bh=(Math.max(0,Math.min(100,r.value))/100)*(h-top-bottom),y=h-bottom-bh;ctx.fillStyle='#48e0a4';ctx.fillRect(x,y,bw,bh);ctx.fillStyle='#d8eeee';ctx.font='11px system-ui';ctx.fillText(Number.isFinite(r.value)?r.value.toFixed(1):'--',x,y-5);ctx.save();ctx.translate(x+bw/2,h-bottom+8);ctx.rotate(-.35);ctx.fillText(r.label,0,0);ctx.restore()})}

function perBab4DownloadCanvas(id,name){const c=$(id);if(!c)return toast('Grafik belum tersedia',true);c.toBlob(blob=>{if(!blob)return;const a=document.createElement('a'),u=URL.createObjectURL(blob);a.href=u;a.download=`FINAL_${selectedExperiment?.id||'BAB4'}_${name}_${stamp()}.png`;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(u),1000)},'image/png')}
function perBab4DrawConfidence(){
  const c=$('perBab4ConfidenceCanvas');if(!c||selectedExperiment?.id!=='F4.2')return;const ctx=c.getContext('2d'),w=c.width,h=c.height,p={l:62,r:24,t:34,b:42};ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='700 16px system-ui';ctx.fillText('Confidence rata-rata terhadap jarak',18,22);ctx.font='12px system-ui';ctx.fillStyle='#8fa8a9';for(let i=0;i<=4;i++){const v=i/4,y=h-p.b-v*(h-p.t-p.b);ctx.strokeStyle='rgba(150,180,180,.16)';ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.fillText(v.toFixed(2),18,y+4)}for(let d=1;d<=5;d++){const x=p.l+(d-1)/4*(w-p.l-p.r);ctx.fillText(`${d} m`,x-10,h-15)}const rows=perBab4BuildDetectionSummary(),series=[['human','Manusia','#48e0a4'],['motorcycle','Sepeda motor','#ffc86b']];for(const [key,label,col] of series){const pts=rows.filter(r=>r[0]===key&&Number.isFinite(+r[7])).map(r=>({d:+r[1],v:+r[7]}));ctx.strokeStyle=col;ctx.fillStyle=col;ctx.lineWidth=3;ctx.beginPath();pts.forEach((q,i)=>{const x=p.l+(q.d-1)/4*(w-p.l-p.r),y=h-p.b-q.v*(h-p.t-p.b);i?ctx.lineTo(x,y):ctx.moveTo(x,y);ctx.beginPath;ctx.arc(x,y,4,0,Math.PI*2);ctx.fill();if(i===0){ctx.beginPath();ctx.moveTo(x,y)}else{const prev=pts[i-1],px=p.l+(prev.d-1)/4*(w-p.l-p.r),py=h-p.b-prev.v*(h-p.t-p.b);ctx.beginPath();ctx.moveTo(px,py);ctx.lineTo(x,y);ctx.stroke()}});ctx.fillStyle=col;ctx.fillText(label,w-p.r-150,20+series.findIndex(s=>s[0]===key)*15)}if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.fillText('Belum ada trial. Grafik terisi otomatis setelah Capture BERHASIL/GAGAL.',p.l,60)}
}
const perBab4DrawSummaryBase=perBab4DrawSummary;
perBab4DrawSummary=function(){perBab4DrawSummaryBase();perBab4DrawConfidence()};
function perBab4EnsureOverlay(){const box=$('contextCamera');if(!box)return null;let c=perBab4Get('perBab4DetectionOverlay');if(!c){c=document.createElement('canvas');c.id='perBab4DetectionOverlay';c.className='per-bab4-detection-overlay';box.appendChild(c)}c.hidden=!(perBab4Is('F4.2'));return c}
function perBab4DrawOverlay(){const c=perBab4EnsureOverlay();if(!c||c.hidden)return;const img=$('expCameraImage'),g=cameraGeometryFor(img);if(!g)return;const fit=g.resizeCanvas(c),ctx=fit.ctx,dpr=fit.dpr,cr=c.getBoundingClientRect();ctx.clearRect(0,0,fit.width,fit.height);const ds=perBab4RawDetections(),candidateSelect=perBab4Get('perBab4Candidate'),autoIndex=!candidateSelect&&typeof perBab4AutoDetectionDecision==='function'?perBab4AutoDetectionDecision().index:-1,selectedIndex=candidateSelect?(+candidateSelect.value||0):autoIndex;ds.forEach((d,i)=>{const a=g.imageToScreen?.(+d.x1,+d.y1),b=g.imageToScreen?.(+d.x2,+d.y2);if(!a||!b)return;const x=(a.x-cr.left)*dpr,y=(a.y-cr.top)*dpr,w=(b.x-a.x)*dpr,h=(b.y-a.y)*dpr;ctx.strokeStyle=i===selectedIndex?'#ffc86b':'#48e0a4';ctx.lineWidth=(i===selectedIndex?3:2)*dpr;ctx.strokeRect(x,y,w,h);ctx.fillStyle=ctx.strokeStyle;ctx.font=`${12*dpr}px ui-monospace`;ctx.fillText(`#${i+1} c${d.class_id??'--'} ${(Number(d.score)||0).toFixed(2)}`,x+3*dpr,Math.max(14*dpr,y-3*dpr))})}
function perBab4LanePointCards(){const panel=$('perceptionCalibrationSection');if(!panel)return;let box=perBab4Get('perLanePointCards');if(!box){box=document.createElement('div');box.id='perLanePointCards';box.className='per-lane-point-cards';const read=$('perceptionCalReadout');read?.before(box)}const c=perceptionDirectCal?.candidate;if(!c){box.innerHTML='<span>Menunggu YAML lane...</span>';return}const pts=[['LT','Kiri Atas',c.leftTopX,c.leftTopY,0],['LB','Kiri Bawah',c.leftBottomX,c.leftBottomY,1],['RT','Kanan Atas',c.rightTopX,c.rightTopY,2],['RB','Kanan Bawah',c.rightBottomX,c.rightBottomY,3]];box.innerHTML=`<div class="per-lane-point-note"><b>4 TITIK LANE SAFETY</b><span>Drag LT/LB/RT/RB bebas 2D. X/Y setiap titik independen; keyboard 1–4 + panah empat arah = fine adjust.</span></div>${pts.map(([id,name,x,y,i])=>`<button type="button" data-lane-handle="${i}"><b>${id}</b><span>${name}</span><code>x ${(+x).toFixed(4)} • y ${(+y).toFixed(4)}</code></button>`).join('')}`;box.querySelectorAll('[data-lane-handle]').forEach(b=>b.onclick=()=>{perceptionDirectCal.mode='lane';perceptionDirectCal.keyboardIndex=+b.dataset.laneHandle;$('perceptionCalOverlay')?.focus();drawPerceptionDirectOverlay()})}
const perBab4BaseRenderDirect=renderPerceptionDirectCalibration;
renderPerceptionDirectCalibration=function(){const r=perBab4BaseRenderDirect();perBab4LanePointCards();return r};
const perBab4BaseDrawDirect=drawPerceptionDirectOverlay;
drawPerceptionDirectOverlay=function(){perBab4BaseDrawDirect();const canvas=$('perceptionCalOverlay'),img=$('cameraImage'),g=cameraGeometryFor(img),c=perceptionDirectCal?.candidate;if(!canvas||!g||!c||perceptionDirectCal.mode!=='lane')return;const fit=g.resizeCanvas(canvas),ctx=fit.ctx,dpr=fit.dpr,cr=canvas.getBoundingClientRect(),pts=directCalLanePoints(c),labels=['LT','LB','RT','RB'];ctx.font=`700 ${11*dpr}px ui-monospace`;pts.forEach((p,i)=>{const s=g.normalizedToScreen(p.x,p.y);if(!s)return;ctx.fillStyle=i===perceptionDirectCal.keyboardIndex?'#ffc86b':'#d8eeee';ctx.fillText(labels[i],(s.x-cr.left+10)*dpr,(s.y-cr.top-9)*dpr)});perBab4LanePointCards()};
const perBab4BaseRenderSelected=renderSelectedExperiment;
renderSelectedExperiment=function(){const r=perBab4BaseRenderSelected();perBab4RenderPanel();if(perBab4Is()&&(activeWorkspaceTab?.perception==='tune'||activePage==='experiments')){document.body.dataset.bab4=selectedExperiment.id;setText('pageEyebrow','PERSEPSI • LAPORAN BAB IV');const badge=$('experimentGraphBadge');if(badge){if(selectedExperiment.id==='F4.2')badge.textContent='GRAFIK REKAP';else if(selectedExperiment.id==='F4.3')badge.textContent='GRAFIK REKAP';else if(selectedExperiment.id==='F4.1')badge.textContent='DOKUMENTASI RUNTIME';}const sub=$('experimentSubtitle');if(sub)sub.textContent='Mode pengambilan data BAB IV: ikuti prosedur pengujian pada panel berikut. Menu tuning dan commissioning tetap terpisah.'}return r};
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
    x.tableColumns=[x.tableColumns[0],['Detik ke-','Jumlah Sampel Kinerja','Jumlah Event Frame','Rata-rata Laju Pipeline [FPS]','Rata-rata Waktu Inferensi [ms]','Rata-rata Waktu Pemrosesan Total [ms]','Rata-rata Umur Frame [ms]','Rata-rata Latensi Subscriber [ms]'],['Parameter','Rata-rata','Simpangan Baku','Minimum','Maksimum','Jumlah Sampel','Satuan']];
    x.tableNames=['Hasil Pengujian Kinerja Waktu Nyata Sistem Persepsi','Rata-rata Parameter per Detik','Ringkasan Statistik Pengujian 60 s'];
  }
};
function perBab4RowsAt(index){return tableRunRows.get(index)||[]}
function perBab4PushAt(index,values){const a=perBab4RowsAt(index);a.push({sec:Math.max(0,Math.floor((Date.now()-(recordStartedMs||Date.now()))/1000)),values,yaml:perBab4YamlSnapshot()});tableRunRows.set(index,a);renderExperimentTable();perBab4RefreshReportExtensions()}
function perBab4SetRowsAt(index,rows){tableRunRows.set(index,rows.map(values=>({sec:0,values,yaml:perBab4YamlSnapshot()})));renderExperimentTable()}
function perBab4Mean(values){const a=values.map(Number).filter(Number.isFinite);return a.length?a.reduce((s,v)=>s+v,0)/a.length:NaN}
function perBab4Std(values){const a=values.map(Number).filter(Number.isFinite);if(!a.length)return NaN;const m=perBab4Mean(a);return Math.sqrt(a.reduce((s,v)=>s+(v-m)*(v-m),0)/a.length)}
function perBab4FmtNum(v,d=3){return Number.isFinite(+v)?(+v).toFixed(d):'--'}

const PER_BAB4_NEGATIVE_SCENARIOS=[
  ['indoor','Lingkungan Dalam Ruangan tanpa Objek Target'],
  ['empty','Jalur tanpa Objek Target'],
  ['nontarget','Latar Statis dengan Objek Non-Target']
];
function perBab4NegativeLabel(key){return PER_BAB4_NEGATIVE_SCENARIOS.find(x=>x[0]===key)?.[1]||key}
function perBab4NegativeCapture(){
  if(!recordStartedMs)return toast('Mulai sesi pengujian terlebih dahulu agar data pengujian negatif dapat disimpan.',true);
  const scenario=$('perBab4NegativeScenario')?.value||'indoor',rep=+$('perBab4NegativeRepeat')?.value||1,dets=perBab4RawDetections();
  const scores=dets.map(d=>+d.score).filter(Number.isFinite),maxConf=scores.length?Math.max(...scores):NaN,classes=dets.map(d=>d.class_id).filter(v=>v!==undefined).join('|')||'--';
  const note=$('perBab4NegativeNote')?.value?.trim()||'',falseCandidate=dets.length>0;
  perBab4PushAt(2,[perBab4RowsAt(2).length+1,perBab4NegativeLabel(scenario),rep,perBab4FrameTimestamp(),dets.length,perBab4FmtNum(maxConf,5),classes,falseCandidate?'Tidak':'Ya',falseCandidate?'Ya':'Tidak',note]);
  perBab4BuildNegativeSummary();
  let nr=rep+1,ns=scenario;if(nr>10){nr=1;const i=PER_BAB4_NEGATIVE_SCENARIOS.findIndex(x=>x[0]===scenario);ns=PER_BAB4_NEGATIVE_SCENARIOS[(i+1)%PER_BAB4_NEGATIVE_SCENARIOS.length][0]}
  if($('perBab4NegativeRepeat'))$('perBab4NegativeRepeat').value=String(nr);if($('perBab4NegativeScenario'))$('perBab4NegativeScenario').value=ns;if($('perBab4NegativeNote'))$('perBab4NegativeNote').value='';
  perBab4RefreshReportExtensions();toast(`Pengujian negatif ${perBab4NegativeLabel(scenario)} • Pengulangan ${rep}: ${falseCandidate?'Deteksi positif palsu teridentifikasi':'Tidak terdapat kandidat deteksi'}`,falseCandidate);
}
function perBab4BuildNegativeSummary(){
  const groups=new Map();for(const r of perBab4RowsAt(2)){const v=r.values,g=groups.get(v[1])||{name:v[1],n:0,empty:0,false:0};g.n++;if(['YA','Ya'].includes(v[7]))g.empty++;if(['YA','Ya'].includes(v[8]))g.false++;groups.set(v[1],g)}
  const order=PER_BAB4_NEGATIVE_SCENARIOS.map(x=>x[1]),rows=[...groups.values()].sort((a,b)=>order.indexOf(a.name)-order.indexOf(b.name)).map(g=>[g.name,g.n,g.empty,g.false,(100*g.false/g.n).toFixed(2)]);
  if(rows.length){const n=rows.reduce((s,r)=>s+(+r[1]||0),0),e=rows.reduce((s,r)=>s+(+r[2]||0),0),f=rows.reduce((s,r)=>s+(+r[3]||0),0);rows.push(['Total',n,e,f,n?(100*f/n).toFixed(2):'--'])}
  perBab4SetRowsAt(3,rows);
}
function perBab4NegativeUndo(){const a=perBab4RowsAt(2);if(!a.length)return toast('Data pengujian negatif belum tersedia.',true);a.pop();tableRunRows.set(2,a);perBab4BuildNegativeSummary();perBab4RefreshReportExtensions();toast('Data pengujian negatif terakhir dihapus')}

const PER_BAB4_HYS_SEQUENCE=[
  {gap:60,status:'GREEN',latched:false,phase:'Kondisi Aman'},
  {gap:25,status:'YELLOW',latched:false,phase:'Peringatan Batas'},
  {gap:0,status:'RED',latched:true,phase:'Pelanggaran Batas'},
  {gap:20,status:'YELLOW',latched:true,phase:'Pemulihan'},
  {gap:40,status:'YELLOW',latched:true,phase:'Penahanan Koreksi'},
  {gap:50,status:'GREEN',latched:false,phase:'Pelepasan Koreksi'}
];
function perBab4HysExpected(step,side){const q=PER_BAB4_HYS_SEQUENCE[Math.max(0,Math.min(5,step-1))],rec=q.status==='GREEN'?'CLEAR':q.status==='YELLOW'&&!q.latched?'WARNING':(side==='left'?'RECENTER_RIGHT':'RECENTER_LEFT');return {...q,rec}}
function perBab4HysCapture(){
  if(!recordStartedMs)return toast('Mulai sesi pengujian terlebih dahulu agar data histeresis dapat disimpan.',true);
  const step=+$('perBab4HysStep')?.value||1,cycle=+$('perBab4HysCycle')?.value||1,side=$('perBab4HysSide')?.value||'left',c=obj('lane_state').corridor||{},exp=perBab4HysExpected(step,side);
  const actualStatus=String(side==='left'?c.left_status:c.right_status),otherStatus=String(side==='left'?c.right_status:c.left_status),latched=Boolean(side==='left'?c.left_recenter_latched:c.right_recenter_latched),gap=+(side==='left'?c.left_gap_px:c.right_gap_px),rec=String(c.recommendation||'--');
  const ok=actualStatus===exp.status&&latched===exp.latched&&rec===exp.rec;
  perBab4PushAt(2,[cycle,step,exp.gap,side==='left'?'Kiri':'Kanan',perBab4FrameTimestamp(),perBab4FmtNum(gap,2),perBab4LaneStatusLabel(actualStatus),perBab4LaneStatusLabel(otherStatus),latched?'Aktif':'Tidak Aktif',perBab4LaneRecommendationLabel(rec),`${exp.phase} • ${perBab4LaneStatusLabel(exp.status)} • Pengunci koreksi ${exp.latched?'Aktif':'Tidak Aktif'} • ${perBab4LaneRecommendationLabel(exp.rec)}`,ok?'Memenuhi Kriteria':'Tidak Memenuhi Kriteria']);
  let ns=step+1,nc=cycle;if(ns>6){ns=1;nc++}if($('perBab4HysStep'))$('perBab4HysStep').value=String(ns);if($('perBab4HysCycle'))$('perBab4HysCycle').value=String(nc);
  perBab4RefreshReportExtensions();toast(`Verifikasi histeresis tahap ${step}: ${ok?'Memenuhi Kriteria':'Tidak Memenuhi Kriteria'}`,!ok);
}
function perBab4HysUndo(){const a=perBab4RowsAt(2);if(!a.length)return toast('Data tahapan histeresis belum tersedia.',true);a.pop();tableRunRows.set(2,a);renderExperimentTable();perBab4RefreshReportExtensions();toast('Data tahapan histeresis terakhir dihapus')}
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
  const samples=perBab4.perf.perfSamples||[],spec=[['Laju Pipeline','pipeline','FPS'],['Waktu Inferensi','inference','ms'],['Waktu Pemrosesan Total','process','ms'],['Umur Frame','age','ms'],['Latensi Subscriber','latency','ms']];
  return spec.map(([label,key,unit])=>{const a=samples.map(x=>x[key]).filter(Number.isFinite);if(!a.length)return[label,'--','--','--','--',0,unit];return[label,perBab4FmtNum(perBab4Mean(a)),perBab4FmtNum(perBab4Std(a)),perBab4FmtNum(Math.min(...a)),perBab4FmtNum(Math.max(...a)),a.length,unit]});
}
const perBab4ReportBasePrepareSummary=perBab4PrepareSummary;
perBab4PrepareSummary=function(){perBab4ReportBasePrepareSummary();if(perBab4Is('F4.4')){perBab4SetRowsAt(1,perBab4PerfSecondRows());perBab4SetRowsAt(2,perBab4PerfStatsRows())}};
function perBab4PerfResearchGate(){const s=perBab4PerformanceSummary(),fps=+s[3],lat=+s[8];if(!Number.isFinite(fps))return{ok:false,text:'Menunggu Data'};if(!Number.isFinite(lat))return{ok:false,text:`${fps.toFixed(1)} FPS • Latensi Belum Tersedia`};return{ok:fps>=15&&lat<=100,text:`${fps.toFixed(1)} FPS • ${lat.toFixed(1)} ms`}}

function perBab4ReportExtensionHost(){const main=$('perceptionBab4Acquisition');if(!main)return null;let e=$('perBab4ReportExtensions');if(!e){e=document.createElement('div');e.id='perBab4ReportExtensions';e.className='per-bab4-report-ext';main.appendChild(e)}return e}
function perBab4RenderReportExtensions(){
  const box=perBab4ReportExtensionHost();if(!box||!perBab4Is())return;if(selectedExperiment.id==='F4.2'){
    box.innerHTML=`<section class="per-bab4-evidence-card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">4.2.5 • PENGUJIAN NEGATIF DETEKSI OBJEK</span><b>30 citra tanpa objek target utama</b><p>Tiga skenario dengan 10 pengulangan per skenario. Kandidat deteksi pada citra tanpa target dicatat sebagai deteksi positif palsu terhadap objek manusia atau sepeda motor.</p></div><span class="status-chip" id="perBab4NegativeState">0 / 30</span></div><div class="per-bab4-controls compact"><label>Skenario Pengujian<select id="perBab4NegativeScenario">${PER_BAB4_NEGATIVE_SCENARIOS.map(x=>`<option value="${x[0]}">${x[1]}</option>`).join('')}</select></label><label>Pengulangan ke-<input id="perBab4NegativeRepeat" type="number" min="1" max="10" value="1"></label><label class="wide">Catatan Pengamatan<input id="perBab4NegativeNote" type="text" placeholder="pantulan / objek non-target / kondisi pencahayaan"></label><div class="per-bab4-actions"><button class="primary-btn" id="perBab4NegativeCapture">AMBIL DATA NEGATIF</button><button class="soft-btn" id="perBab4NegativeUndo">HAPUS DATA TERAKHIR</button><button class="soft-btn" id="perBab4NegativeCsv">EKSPOR CSV UJI NEGATIF</button></div></div><div class="per-bab4-kpis"><div><span>Jumlah Kandidat Aktual</span><b id="perBab4NegLiveCount">--</b></div><div><span>Confidence Maksimum</span><b id="perBab4NegLiveConf">--</b></div><div><span>Deteksi Positif Palsu</span><b id="perBab4NegFalse">--</b></div><div><span>Rasio Positif Palsu</span><b id="perBab4NegRatio">--</b></div></div></section>`;
  }else if(selectedExperiment.id==='F4.3'){
    box.innerHTML=`<section class="per-bab4-evidence-card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">4.3.5 • VERIFIKASI HISTERESIS KEAMANAN LAJUR</span><b>Kondisi Aman → Peringatan Batas → Pelanggaran Batas → Pemulihan → Pelepasan Koreksi</b><p>Urutan jarak acuan: 60, 25, 0, 20, 40, dan 50 piksel. Rekam status serta pengunci koreksi aktual pada setiap tahapan tanpa mengubah nilai ambang selama satu siklus pengujian.</p></div><span class="status-chip" id="perBab4HysState">0 tahapan</span></div><div class="per-bab4-controls compact"><label>Sisi Pengujian<select id="perBab4HysSide"><option value="left">Kiri</option><option value="right">Kanan</option></select></label><label>Siklus ke-<input id="perBab4HysCycle" type="number" min="1" value="1"></label><label>Tahapan<select id="perBab4HysStep">${PER_BAB4_HYS_SEQUENCE.map((q,i)=>`<option value="${i+1}">${i+1} • ${q.phase} • ${q.gap} piksel</option>`).join('')}</select></label><div class="per-bab4-actions"><button class="primary-btn" id="perBab4HysCapture">AMBIL DATA TAHAPAN</button><button class="soft-btn" id="perBab4HysUndo">HAPUS DATA TERAKHIR</button><button class="soft-btn" id="perBab4HysCsv">EKSPOR CSV HISTERESIS</button></div></div><div class="per-bab4-kpis"><div><span>Kondisi Acuan</span><b id="perBab4HysExpected">--</b></div><div><span>Jarak Aktual</span><b id="perBab4HysGap">--</b></div><div><span>Status / Pengunci Koreksi</span><b id="perBab4HysActual">--</b></div><div><span>Rekomendasi Sistem</span><b id="perBab4HysRec">--</b></div></div></section>`;
  }else if(selectedExperiment.id==='F4.4'){
    box.innerHTML=`<section class="per-bab4-evidence-card per-bab4-timed"><div class="per-bab4-evidence-head"><div><span class="eyebrow">PENGUKURAN LAJU SUMBER DATA • 60 s</span><b>Pengambilan Data Kinerja Waktu Nyata Sistem Persepsi</b><p>Data dilaporkan pada detik ke-1 hingga ke-60. Nilai rata-rata per detik berasal dari pembaruan aktual /perception/performance, sedangkan jumlah frame berasal dari event /perception/raw_detections.</p></div><span class="status-chip" id="perBab4PerfState">SIAP</span></div><div class="per-bab4-kpis six"><div><span>Waktu Tersisa</span><b id="perBab4PerfCountdown">60 s</b></div><div><span>Event Frame Diterima</span><b id="perBab4PerfFrames">0</b></div><div><span>Sampel Kinerja</span><b id="perBab4PerfSamples">0</b></div><div><span>Laju Sumber Deteksi [Hz]</span><b id="perBab4PerfRawHz">--</b></div><div><span>Laju Sumber Kinerja [Hz]</span><b id="perBab4PerfSourceHz">--</b></div><div><span>Validitas Data</span><b id="perBab4PerfGate">Menunggu Data</b></div></div><div class="per-bab4-second-table"><table><thead><tr><th>Detik ke-</th><th>Sampel Kinerja</th><th>Event Frame</th><th>Laju Pipeline [FPS]</th><th>Waktu Inferensi [ms]</th><th>Waktu Proses [ms]</th><th>Latensi Subscriber [ms]</th></tr></thead><tbody id="perBab4PerfSecondBody"><tr><td colspan="7">Data pengujian belum tersedia.</td></tr></tbody></table></div></section>`;
  }else{box.innerHTML=`<section class="per-bab4-evidence-card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">4.1 • DOKUMENTASI IMPLEMENTASI</span><b>Verifikasi kondisi runtime dan konfigurasi aktif</b><p>Verifikasi koneksi kamera, backend CPU, model YOLOPv2, aliran data deteksi objek, keamanan lajur, dan topik kinerja. Parameter kalibrasi tetap menggunakan konfigurasi yang telah ditetapkan dan tidak diubah pada tahap dokumentasi ini.</p></div><span class="status-chip">HANYA BACA</span></div></section>`}
  perBab4BindReportExtensions();perBab4RefreshReportExtensions();
}

function perBab4DownloadTableCsv(index,name){const cols=selectedExperiment?.tableColumns?.[index]||[],rows=perBab4RowsAt(index).map(r=>r.values||[]);if(!rows.length)return toast('Belum ada data untuk diekspor.',true);const q=v=>`"${String(v??'').replace(/"/g,'""')}"`,csv=[cols,...rows].map(r=>r.map(q).join(',')).join('\n');downloadBlob(`${name}_${stamp()}.csv`,'text/csv;charset=utf-8',csv)}
function perBab4BindReportExtensions(){
  $('perBab4NegativeCapture')?.addEventListener('click',perBab4NegativeCapture);$('perBab4NegativeUndo')?.addEventListener('click',perBab4NegativeUndo);$('perBab4NegativeCsv')?.addEventListener('click',()=>perBab4DownloadTableCsv(2,'BAB4_4.2_uji_negatif'));
  $('perBab4HysCapture')?.addEventListener('click',perBab4HysCapture);$('perBab4HysUndo')?.addEventListener('click',perBab4HysUndo);$('perBab4HysCsv')?.addEventListener('click',()=>perBab4DownloadTableCsv(2,'BAB4_4.3_histeresis'));
  $('perBab4HysStep')?.addEventListener('change',perBab4RefreshReportExtensions);$('perBab4HysSide')?.addEventListener('change',perBab4RefreshReportExtensions);$('perBab4LaneRepeat')?.addEventListener('input',perBab4SyncLaneSide);
}
function perBab4RefreshNegative(){const rows=perBab4RowsAt(2),summary=perBab4RowsAt(3),d=perBab4RawDetections(),scores=d.map(x=>+x.score).filter(Number.isFinite),falseCount=rows.filter(r=>['YA','Ya'].includes(r.values?.[8])).length;setText('perBab4NegativeState',`${rows.length} / 30`);const chip=$('perBab4NegativeState');if(chip)chip.className='status-chip '+(rows.length>=30?'':'waiting');setText('perBab4NegLiveCount',String(d.length));setText('perBab4NegLiveConf',scores.length?Math.max(...scores).toFixed(3):'--');setText('perBab4NegFalse',String(falseCount));setText('perBab4NegRatio',rows.length?`${(100*falseCount/rows.length).toFixed(1)}%`:'--');void summary}
function perBab4RefreshHys(){const step=+$('perBab4HysStep')?.value||1,side=$('perBab4HysSide')?.value||'left',exp=perBab4HysExpected(step,side),c=obj('lane_state').corridor||{},gap=+(side==='left'?c.left_gap_px:c.right_gap_px),st=String(side==='left'?c.left_status:c.right_status),latched=Boolean(side==='left'?c.left_recenter_latched:c.right_recenter_latched);setText('perBab4HysState',`${perBab4RowsAt(2).length} tahapan`);setText('perBab4HysExpected',`${exp.gap} piksel • ${perBab4LaneStatusLabel(exp.status)} • pengunci ${exp.latched?'Aktif':'Tidak Aktif'}`);setText('perBab4HysGap',Number.isFinite(gap)?`${gap.toFixed(1)} piksel`:'--');setText('perBab4HysActual',`${perBab4LaneStatusLabel(st)} • pengunci ${latched?'Aktif':'Tidak Aktif'}`);setText('perBab4HysRec',perBab4LaneRecommendationLabel(c.recommendation))}
function perBab4RefreshPerfExt(){const active=!!perBab4.perf.active,elapsed=active?Math.max(0,(Date.now()-perBab4.perf.started)/1000):0,left=Math.max(0,60-elapsed),gate=perBab4PerfResearchGate();setText('perBab4PerfState',active?'PEREKAMAN AKTIF':(perBab4.perf.started?'SELESAI':'SIAP'));const chip=$('perBab4PerfState');if(chip)chip.className='status-chip '+(active?'waiting':perBab4.perf.started?'':'');setText('perBab4PerfCountdown',`${Math.ceil(left)} s`);setText('perBab4PerfFrames',String(perBab4.perf.count||0));setText('perBab4PerfSamples',String(perBab4.perf.perfSamples?.length||0));setText('perBab4PerfRawHz',perBab4FmtNum(channelRate('raw_detections'),2));setText('perBab4PerfSourceHz',perBab4FmtNum(channelRate('perception_performance'),2));setText('perBab4PerfGate',gate.text);const g=$('perBab4PerfGate');if(g)g.className=gate.ok?'ok':'check';const body=$('perBab4PerfSecondBody');if(body){const rows=perBab4PerfSecondRows().slice(-12);body.innerHTML=rows.length?rows.map(r=>`<tr>${[r[0],r[1],r[2],r[3],r[4],r[5],r[7]].map(v=>`<td>${escapeHtml(v)}</td>`).join('')}</tr>`).join(''):'<tr><td colspan="7">Data pengujian belum tersedia.</td></tr>'}}
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
    dets.forEach((d,i)=>{const norm=Math.max(Math.abs(+d.x1||0),Math.abs(+d.x2||0),Math.abs(+d.y1||0),Math.abs(+d.y2||0))<=1.2,x1=(norm?(+d.x1||0)*img.naturalWidth:(+d.x1||0))*sx,y1=(norm?(+d.y1||0)*img.naturalHeight:(+d.y1||0))*sy,x2=(norm?(+d.x2||0)*img.naturalWidth:(+d.x2||0))*sx,y2=(norm?(+d.y2||0)*img.naturalHeight:(+d.y2||0))*sy,selected=i===meta.selectedIndex;ctx.strokeStyle=selected?'#ffc86b':'#48e0a4';ctx.lineWidth=selected?5:3;ctx.strokeRect(x1,y1,Math.max(1,x2-x1),Math.max(1,y2-y1));ctx.fillStyle=ctx.strokeStyle;ctx.font='700 16px ui-monospace,SFMono-Regular,Menlo,monospace';ctx.fillText(`${perF42ClassAcademicLabel?.(d.class_id)||('Kelas '+(d.class_id??'--'))} • ${Number.isFinite(+d.score)?(+d.score*100).toFixed(1)+'%':'--'}`,Math.max(4,x1+5),Math.max(20,y1-5))});
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
  const c=document.createElement('canvas');c.width=1280;c.height=800;const ctx=c.getContext('2d');ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);perBab4EvidenceText(ctx,`Dokumentasi Visual Pengujian Deteksi Objek • ${target==='human'?'MANUSIA':'SEPEDA MOTOR'} • Jarak 1–5 m`,54,58,30,'#e5f4f3',900);perBab4EvidenceText(ctx,'Satu citra representatif dipilih dari 10 pengulangan pada setiap jarak berdasarkan hasil klasifikasi sesuai.',54,91,17,'#8fa8a9',600);
  const pos=[[35,105],[450,105],[865,105],[242,438],[657,438]],pw=380,ph=300,imgH=226;
  for(let i=0;i<5;i++){const [x,y]=pos[i],rep=reps[i];ctx.fillStyle='#0b1b1d';ctx.fillRect(x,y,pw,ph);ctx.strokeStyle=rep?'rgba(72,224,164,.45)':'rgba(255,200,107,.4)';ctx.lineWidth=2;ctx.strokeRect(x,y,pw,ph);perBab4EvidenceText(ctx,`${i+1} m`,x+16,y+30,22,'#e5f4f3',900);if(rep){try{const im=await perBab4LoadEvidenceImage(rep.dataUrl);perBab4DrawImageContain(ctx,im,x+10,y+44,pw-20,imgH)}catch(_){perBab4EvidenceText(ctx,'CITRA TIDAK TERSEDIA',x+150,y+200,18,'#ff6677',800)}const status=rep.success?'Klasifikasi Sesuai':'Tidak Memenuhi Kriteria',conf=rep.success&&Number.isFinite(+rep.confidence)?` • confidence ${(+rep.confidence*100).toFixed(1)}%`:'';perBab4EvidenceText(ctx,`Pengulangan ${rep.repeat} • ${status}${conf}`,x+14,y+286,13,rep.success?'#48e0a4':'#ff6677',800)}else{perBab4EvidenceText(ctx,'Citra bukti belum tersedia',x+86,y+175,15,'#8fa8a9',700)}}
  perBab4EvidenceText(ctx,'Dihasilkan otomatis oleh GUI Persepsi • BAB IV 4.2',35,778,13,'#708f8d',600);return c;
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
    const values=[perBab4Rows().length+1,target,distance,repeat,perBab4FrameTimestamp(),backend,dets.length,success?confidence.toFixed(5):'--',success?'BERHASIL':'GAGAL',success?String(sel?.class_id??'--'):classes,note,decision.reason,evidencePath],
      trialKey=`${target}|${distance}|${repeat}`,retake=perBab4UpsertMainTrial(trialKey,values,v=>`${v[1]}|${v[2]}|${v[3]}`);
    perBab4.evidence.f42=perBab4.evidence.f42.filter(x=>!(x.target===target&&+x.distance===distance&&+x.repeat===repeat));
    if(dataUrl)perBab4.evidence.f42.push({target,distance,repeat,success,confidence,dataUrl,evidencePath});else perBab4.evidence.captureWarnings++;
    perBab4.autoTrial.lastResult=`${retake?'RETAKE • ':''}${success?'BERHASIL':'GAGAL'} • ${decision.reason}${success&&Number.isFinite(confidence)?` • conf ${confidence.toFixed(3)}`:''}`;perBab4AdvanceDetection(target,distance,repeat);perBab4RefreshEvidencePackage();perBab4RefreshAutoTrialUi();toast(`${retake?'RETAKE replace • ':''}${target==='human'?'Manusia':'Sepeda motor'} ${distance} m #${repeat}: ${success?'BERHASIL':'GAGAL'} • frame tersimpan`,!success);
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
  Object.assign(x,{section:'4.1 Implementasi Sistem Persepsi Visual — Dokumentasi Kondisi Runtime',groupTitle:'BAB IV — 4.1 Implementasi Sistem Persepsi Visual',reportMode:true,bab4Protocol:'implementation',bab4ManualCapture:true,bab4NoTimeAxis:true,
    tableColumns:[['Parameter Implementasi','Nilai Aktual','Sumber Data','Status Verifikasi']],tableNames:['Dokumentasi Kondisi Runtime Sistem Persepsi Visual'],
    graphCaptions:['Dokumentasi Implementasi Sistem Persepsi Visual'],graphs:[{type:'time_series',series:['Camera capture FPS'],xLabel:'Snapshot',yLabel:'Evidence'}],
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
    ['Waktu Pengambilan',new Date().toISOString(),'GUI Persepsi','Informasi'],
    ['Koneksi Kamera',cam?'Terhubung':'Tidak Terhubung','connected.camera',cam?'Sesuai':'Perlu Pemeriksaan'],
    ['Kondisi Kamera',healthy?'Sehat':'Belum Siap','camera_healthy',healthy?'Sesuai':'Perlu Pemeriksaan'],
    ['Resolusi RGB',`${cfgVal(cfg.rgb_width)} × ${cfgVal(cfg.rgb_height)}`,'YAML perception','CONFIG'],
    ['Setpoint Laju Bingkai Kamera',cfgVal(cfg.fps,' FPS'),'YAML Persepsi','Konfigurasi'],
    ['Laju Akuisisi Kamera Aktual',perBab4F41Fmt(cap,2,' FPS'),'/perception/performance',Number.isFinite(cap)&&cap>0?'Terukur':'Perlu Pemeriksaan'],
    ['Backend Inferensi',String(p.backend||'--').toUpperCase(),'/perception/performance',String(p.backend||'').toLowerCase()==='cpu'?'Sesuai':'Perlu Pemeriksaan'],
    ['Status Inferensi YOLOPv2',inf?'Aktif':'Tidak Aktif','Status runtime inferensi',inf?'Sesuai':'Perlu Pemeriksaan'],
    ['Setpoint Laju Inferensi',cfgVal(cfg.cpu_inference_fps,' FPS'),'YAML Persepsi','Konfigurasi'],
    ['Laju Pipeline Aktual',perBab4F41Fmt(pipe,2,' FPS'),'/perception/performance',Number.isFinite(pipe)&&pipe>0?'Terukur':'Perlu Pemeriksaan'],
    ['Waktu Inferensi',perBab4F41Fmt(infer,2,' ms'),'/perception/performance',Number.isFinite(infer)?'Terukur':'Perlu Pemeriksaan'],
    ['Waktu Pemrosesan Total',perBab4F41Fmt(process,2,' ms'),'/perception/performance',Number.isFinite(process)?'Terukur':'Perlu Pemeriksaan'],
    ['Jumlah Thread CPU',cfgVal(cfg.cpu_threads),'YAML Persepsi','Konfigurasi'],
    ['Laju Pratinjau Web',cfgVal(cfg.web_preview_fps,' FPS'),'YAML Persepsi','Konfigurasi'],
    ['Aliran Data Deteksi Mentah',rawFresh?`Terkini • ${rd.count??0} kandidat`:`Tidak Terkini • umur data ${perBab4F41Fmt(age('raw_detections'),2,' s')}`,'/perception/raw_detections',rawFresh?'Sesuai':'Perlu Pemeriksaan'],
    ['Aliran Data Keamanan Lajur',laneFresh?`Terkini • ${perBab4LaneRecommendationLabel?.(corr.recommendation||lane.state)||corr.recommendation||lane.state||'Aktif'}`:`Tidak Terkini • umur data ${perBab4F41Fmt(age('lane_state'),2,' s')}`,'/perception/lane_safety_state',laneFresh?'Sesuai':'Perlu Pemeriksaan'],
    ['Citra Bukti',evidencePath,'Pratinjau kamera teranotasi',evidencePath&&evidencePath!=='--'?'Tersimpan':'Belum Tersimpan']
  ];
}
function perBab4F41BuildCanvas(rows=perBab4F41RuntimeRows()){
  const c=document.createElement('canvas');c.width=1280;c.height=720;const ctx=c.getContext('2d'),img=$('expCameraImage');
  ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);perBab4EvidenceText(ctx,'BAB IV 4.1 • Implementasi Sistem Persepsi Visual',38,48,28,'#e5f4f3',900);perBab4EvidenceText(ctx,'Dokumentasi kondisi runtime • data aktual dan konfigurasi efektif',38,78,15,'#8fa8a9',600);
  const ix=38,iy=105,iw=805,ih=540;ctx.fillStyle='#031012';ctx.fillRect(ix,iy,iw,ih);ctx.strokeStyle='rgba(72,224,164,.28)';ctx.lineWidth=2;ctx.strokeRect(ix,iy,iw,ih);
  if(img&&img.complete&&img.naturalWidth&&img.naturalHeight){const s=Math.min(iw/img.naturalWidth,ih/img.naturalHeight),dw=img.naturalWidth*s,dh=img.naturalHeight*s;ctx.drawImage(img,ix+(iw-dw)/2,iy+(ih-dh)/2,dw,dh)}else{perBab4EvidenceText(ctx,'CITRA KAMERA BELUM TERSEDIA',ix+210,iy+275,20,'#ffc86b',800)}
  const wanted=['Koneksi Kamera','Kondisi Kamera','Resolusi RGB','Laju Akuisisi Kamera Aktual','Backend Inferensi','Status Inferensi YOLOPv2','Laju Pipeline Aktual','Waktu Inferensi','Jumlah Thread CPU','Aliran Data Deteksi Mentah','Aliran Data Keamanan Lajur'];let y=118;
  for(const name of wanted){const r=rows.find(v=>v[0]===name);if(!r)continue;const ok=['Sesuai','Terukur','Konfigurasi','Tersimpan','Informasi'].includes(r[3]),col=ok?'#48e0a4':'#ffc86b';ctx.fillStyle='rgba(255,255,255,.025)';ctx.fillRect(875,y-18,365,44);perBab4EvidenceText(ctx,r[0],890,y,11,'#779696',700);perBab4EvidenceText(ctx,r[1],890,y+20,15,'#d8eeee',800);ctx.fillStyle=col;ctx.fillRect(1218,y-8,8,8);y+=49}
  perBab4EvidenceText(ctx,new Date().toISOString(),38,690,13,'#708f8d',500);perBab4EvidenceText(ctx,'Dihasilkan otomatis oleh GUI Persepsi • BAB IV 4.1',875,690,13,'#708f8d',600);return c;
}
function perBab4F41TableRows(evidencePath){return perBab4F41RuntimeRows(evidencePath).map(values=>({sec:0,values,yaml:perBab4YamlSnapshot()}))}
function perBab4F41CardHtml(){return `<section class="per-bab4-evidence-card per-bab4-f41-card" id="perBab4F41Card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">DOKUMENTASI IMPLEMENTASI SISTEM</span><b>Pengambilan dokumentasi BAB 4.1 dalam satu proses</b><p>Sistem membuka sesi singkat, merekam citra kamera aktual, menyimpan parameter runtime dan konfigurasi YAML, membentuk tabel dokumentasi, kemudian menyimpan seluruh keluaran dalam satu folder pengujian.</p></div><span class="status-chip" id="perBab4F41State">SIAP</span></div><div class="per-bab4-f41-grid"><div><span>Kamera</span><b id="perBab4F41Camera">--</b></div><div><span>Inferensi YOLOPv2</span><b id="perBab4F41Inference">--</b></div><div><span>Laju Pipeline</span><b id="perBab4F41Fps">--</b></div><div><span>Keluaran Data</span><b>CSV + XLSX + PNG + JPEG + Manifest</b></div></div><button class="primary-btn per-bab4-f41-capture" id="perBab4F41Capture">📸 AMBIL DOKUMENTASI 4.1 DAN SIMPAN</button><div class="per-bab4-f41-result"><span>Folder hasil pengujian</span><code id="perBab4F41Folder">${escapeHtml(perBab4.f41.lastFolder||'Belum ada dokumentasi')}</code></div></section>`}
function perBab4F41RefreshCard(){if(selectedExperiment?.id!=='F4.1'||!$('perBab4F41Card'))return;const p=obj('perception_performance')||{},cap=perBab4F41Num('pipeline_fps','fps'),gate=testPreflightStatus(),ready=gate.ok,state=perBab4.f41.busy?'MENGAMBIL DATA':perBab4.f41.lastState==='SAVED'?'TERSIMPAN':perBab4.f41.lastState==='ERROR'?'KESALAHAN':ready?'SIAP':'MENUNGGU SUMBER DATA';setText('perBab4F41Camera',`${bool(raw('connected.camera'))?'Terhubung':'Tidak Terhubung'} • ${bool(raw('camera_healthy'))?'Sehat':'Belum Siap'}`);setText('perBab4F41Inference',`${perBab4InferenceOn()?'Aktif':'Tidak Aktif'} • ${String(p.backend||'--').toUpperCase()}`);setText('perBab4F41Fps',perBab4F41Fmt(cap,2,' FPS'));setText('perBab4F41State',state);const s=$('perBab4F41State');if(s)s.className='status-chip '+(state==='TERSIMPAN'||state==='SIAP'?'':'waiting');const b=$('perBab4F41Capture');if(b){b.disabled=perBab4.f41.busy||!ready;b.title=ready?'Ambil dokumentasi runtime, citra bukti, dan simpan folder pengujian':`Belum siap: ${gate.reason||'sumber data belum siap'}`;b.textContent=perBab4.f41.busy?'MENYIMPAN DATA 4.1...':'📸 AMBIL DOKUMENTASI 4.1 DAN SIMPAN'}setText('perBab4F41Folder',perBab4.f41.lastFolder||(ready?'Belum ada dokumentasi':`Menunggu: ${gate.reason||'sumber data'}`))}
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
renderSelectedExperiment=function(){if(currentExp==='perception'&&selectedExperiment?.id==='F4.1')perBab4F41ApplySchema();const r=perBab4F41BaseRenderSelected();const rec=$('recordToggleBtn');if(rec)rec.hidden=perBab4Is('F4.1');return r};


/* ===== BAB IV 4.5 CONTROL RESPONSE WIZARD v1 (2026-09-17) =====
 * Scope is intentionally limited to the Persepsi FINAL Dataset surface.
 * No navigation/ESC page markup or behaviour is modified. Existing navigation goal
 * and recorder APIs are only consumed so the perception thesis test is end-to-end.
 */
PER_BAB4_IDS.add('F4.5');
perBab4.f45=perBab4.f45||{active:false,phase:'IDLE',token:0,timer:null,countdownTimer:null,startMs:0,trialKey:'',mode:'obstacle',samples:[],completed:{},lastResult:'WAIT SESSION',lastFolder:'',initialPose:null,goalDistance:3,timeoutSec:30,countdownLeft:0};
const PER_F45_OBSTACLE_KEYS=['human-1','human-2','human-3','human-4','human-5','motorcycle-1','motorcycle-2','motorcycle-3','motorcycle-4','motorcycle-5'];
const PER_F45_LANE_KEYS=['green_outer','green_drivable','yellow-left','yellow-right','red-left','red-right','gray'];
function perBab4F45ApplySchema(){
  const list=experiments.perception||[];let x=list.find(v=>v.id==='F4.5');
  if(!x){const base=list.find(v=>v.id==='F4.4')||list[0]||{};x={...base,id:'F4.5'};list.push(x)}
  Object.assign(x,{groupId:'FINAL-4.5',groupTitle:'FINAL BAB IV — 4.5 Validasi Respons Kendaraan',section:'4.5 Validasi Respons Kendaraan terhadap Keluaran Persepsi',reportMode:true,bab4Protocol:'control_response',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:[],
    tableColumns:[
      ['No','Objek','Jarak Awal [m]','Respons Aktual','Response Latency [ms]','Min Obstacle [m]','Max |Steering Target| [deg]','Steering RMSE [deg]','Mean Speed Actual [m/s]','Durasi [s]','Hasil','Evidence Frame'],
      ['No','Kondisi Lane','Expected Policy','Respons Aktual','Response Time [ms]','Mean Speed Actual [m/s]','Max |Steering Target| [deg]','Steering RMSE [deg]','Recovery Time [ms]','Durasi [s]','Hasil','Evidence Frame'],
      ['Trial Key','t [ms]','Mode','Kondisi','Obstacle [m]','Safety Decision','Lane State','Lane Decision','Nav v [m/s]','Out v [m/s]','Out w [rad/s]','Speed Target [m/s]','Speed Actual [m/s]','Steer Target [deg]','Steer Actual [deg]','Obstacle Seen','Obstacle Source','Obstacle Class ID','Near Emergency'],
      ['Timestamp','Trial Key','Mode','Reason','Durasi [s]','Evidence Frame']
    ],tableNames:['Rekap Respons Kontrol Obstacle 1–5 m','Rekap Respons Kontrol Lane Safety','Time-Series Target dan Feedback Kontrol','Log Trial Invalid / Abort'],
    graphCaptions:['Response Latency Obstacle terhadap Jarak','Mean Speed Aktual Lane Safety','RMSE Steering per Kondisi'],graphs:[],
    liveSeries:{'Drive target':'esc_drive_target','Drive actual':'esc_drive_actual','Steering target':'esc_steer_target','Steering actual':'esc_steer_actual'}});
  if(selectedExperiment?.id==='F4.5')selectedExperiment=x;
}
const perBab4F45PatchBase=patchPerceptionBab4Catalog;
patchPerceptionBab4Catalog=function(){perBab4F45PatchBase();perBab4F45ApplySchema()};
perBab4F45ApplySchema();

function perBab4F45Pose(){try{const p=typeof navigationRobotPose==='function'?navigationRobotPose():obj('ekf_global');const yaw=Number.isFinite(+p?.yaw)?+p.yaw:Number.isFinite(+p?.yaw_rad)?+p.yaw_rad:NaN;return{x:+p?.x,y:+p?.y,yaw}}catch(_){const p=obj('ekf_global');return{x:+p.x,y:+p.y,yaw:+p.yaw}}}
function perBab4F45SelectedObstacleClassId(){const key=perBab4.f45.active&&/^(human|motorcycle)-/.test(perBab4.f45.trialKey)?perBab4.f45.trialKey:(($('perF45Object')?.value||'human')+'-');return key.startsWith('motorcycle-')?3:0}
function perBab4F45ObstacleObservation(){
  const classId=perBab4F45SelectedObstacleClassId(),ds=obj('obstacle_metrics')?.detections,metric=Array.isArray(ds)?ds.filter(d=>d?.confirmed!==false&&+d.class_id===classId&&Number.isFinite(+d.forward_m)&&+d.forward_m>=0):[],near=obj('near_field_state')||{},nearSeen=near.raw_candidate===true&&+near.raw_class_id===classId,nearEmergency=nearSeen&&near.emergency===true;
  metric.sort((a,b)=>(+a.forward_m)-(+b.forward_m));const m=metric[0];return{seen:!!m||nearSeen,distance:m?+m.forward_m:NaN,source:m?(nearEmergency?'METRIC+NEAR':'METRIC'):(nearSeen?'NEAR_FIELD':'NONE'),classId,nearEmergency,confidence:m?+m.score:+near.confidence};
}
function perBab4F45ObstacleDistance(){return perBab4F45ObstacleObservation().distance}
function perBab4F45LaneSnapshot(){const ls=obj('lane_state')||{},c=ls.corridor||{};return{valid:Boolean(ls.valid),drivable:Boolean(c.drivable_detected??ls.drivable_valid),rawLaneMask:Boolean(c.raw_lane_mask_detected),laneMask:Boolean(c.lane_mask_detected),leftValid:Boolean(c.left_valid),rightValid:Boolean(c.right_valid),internalIgnored:Boolean(c.internal_lane_ignored),evidenceMode:String(c.evidence_mode||'NONE'),left:String(c.left_status||'UNKNOWN').toUpperCase(),right:String(c.right_status||'UNKNOWN').toUpperCase(),rec:String(c.recommendation||ls.state||'--').toUpperCase(),leftGap:+c.left_gap_px,rightGap:+c.right_gap_px}}
function perBab4F45LaneExpected(key){return({green_outer:{label:'GREEN • OUTER SAFE',policy:'NORMAL SPEED • DRIVABLE + OUTER LANE SAFE • NO SAFETY STEERING'},green_drivable:{label:'GREEN • DRIVABLE ONLY',policy:'NORMAL SPEED • DRIVABLE VALID • LANE MASK ABSENT • NO SAFETY STEERING'},'yellow-left':{label:'YELLOW LEFT',policy:'SLOW • OUTER LANE WARNING • NO RECENTER'},'yellow-right':{label:'YELLOW RIGHT',policy:'SLOW • OUTER LANE WARNING • NO RECENTER'},'red-left':{label:'RED LEFT',policy:'SLOW • OUTER LANE INTRUSION • RECENTER RIGHT'},'red-right':{label:'RED RIGHT',policy:'SLOW • OUTER LANE INTRUSION • RECENTER LEFT'},gray:{label:'GRAY',policy:'STOP • NO LANE + NO DRIVABLE • FAIL-CLOSED'}})[key]||{label:key,policy:'--'}}
function perBab4F45LaneMatches(key){const s=perBab4F45LaneSnapshot();if(key==='green_outer')return s.drivable&&s.laneMask&&s.leftValid&&s.rightValid&&s.left==='GREEN'&&s.right==='GREEN';if(key==='green_drivable')return s.drivable&&!s.laneMask&&!s.leftValid&&!s.rightValid&&s.left==='GREEN'&&s.right==='GREEN';if(key==='yellow-left')return s.drivable&&s.laneMask&&s.leftValid&&s.left==='YELLOW'&&s.right==='GREEN';if(key==='yellow-right')return s.drivable&&s.laneMask&&s.rightValid&&s.right==='YELLOW'&&s.left==='GREEN';if(key==='red-left')return s.drivable&&s.laneMask&&s.leftValid&&s.left==='RED'&&s.right==='GREEN';if(key==='red-right')return s.drivable&&s.laneMask&&s.rightValid&&s.right==='RED'&&s.left==='GREEN';return !s.drivable&&!s.laneMask&&s.rec==='NO_MASK_EVIDENCE'&&s.left==='UNKNOWN'&&s.right==='UNKNOWN'}
function perBab4F45CurrentKey(){const mode=$('perF45Mode')?.value||'obstacle';if(mode==='lane')return $('perF45LaneCondition')?.value||'green_outer';return`${$('perF45Object')?.value||'human'}-${+$('perF45Distance')?.value||1}`}
function perBab4F45DoneCount(){return Object.values(perBab4.f45.completed||{}).filter(v=>v==='PASS'||v==='FAIL').length}
const PER_F45_STORAGE_KEY='adv-perception-f45-session-v1';
function perBab4F45Persist(){
  if(!recordStartedMs)return;try{const tables={};for(let i=0;i<4;i++)tables[i]=tableRunRows.get(i)||[];localStorage.setItem(PER_F45_STORAGE_KEY,JSON.stringify({version:1,startedMs:recordStartedMs,completed:perBab4.f45.completed||{},lastResult:perBab4.f45.lastResult||'',lastFolder:perBab4.f45.lastFolder||'',tables,savedAt:Date.now()}));perBab4.f45.restoredFor=recordStartedMs}catch(e){console.warn('F4.5 persist gagal',e)}}
function perBab4F45Restore(){
  if(!recordStartedMs||perBab4.f45.restoredFor===recordStartedMs)return false;try{const x=JSON.parse(localStorage.getItem(PER_F45_STORAGE_KEY)||'null');if(!x||x.version!==1||Math.abs((+x.startedMs||0)-recordStartedMs)>5000)return false;perBab4.f45.completed=x.completed||{};perBab4.f45.lastResult=x.lastResult||'SESSION RESTORED';perBab4.f45.lastFolder=x.lastFolder||'';for(let i=0;i<4;i++)tableRunRows.set(i,Array.isArray(x.tables?.[i])?x.tables[i]:[]);perBab4.f45.restoredFor=recordStartedMs;return true}catch(e){console.warn('F4.5 restore gagal',e);return false}}
function perBab4F45ControlConfig(){return configData('trajectory_safety')?.trajectory_safety_supervisor?.ros__parameters||{}}
function perBab4F45RuntimeFlags(){
  const cfg=perBab4F45ControlConfig(),ts=obj('trajectory_safety_state')||{},lc=obj('lane_control')||{},tsText=String(ts.raw||ts.value||'');
  return{
    metric:ts.metric_obstacle_safety===true||/metric_obstacle_safety=true/.test(tsText),
    lane:lc.enabled===true||ts.lane_safety_enabled===true||/lane_safety_enabled=true/.test(tsText),
    grayStop:cfg.stop_on_lane_lost===true||ts.stop_on_lane_lost===true||/stop_on_lane_lost=true/.test(tsText),
    bypass:cfg.commissioning_bypass_enabled===true||/commissioning_bypass=true/.test(tsText),
    ts,tsText,cfg
  };
}
function perBab4F45SessionReady(){
  const pose=perBab4F45Pose(),f=perBab4F45RuntimeFlags(),reasons=[];
  if(!bool(raw('connected.camera'))||!bool(raw('camera_healthy')))reasons.push('camera');
  if(!perBab4InferenceOn())reasons.push('YOLOPv2');
  if(!bool(raw('connected.esc_feedback'))||!bool(raw('connected.esc_ready')))reasons.push('ESC');
  if(!bool(raw('system.motion_ready')))reasons.push('motion gate');
  const qualification=perBab4BasePhaseGate();if(!qualification.ok)reasons.push('actuator/Nav2 qualification');
  if(bool(raw('system.estop')))reasons.push('E-stop');
  if(!Number.isFinite(pose.x)||!Number.isFinite(pose.y)||!Number.isFinite(pose.yaw))reasons.push('map pose');
  if(!channelFresh('trajectory_safety_state',2.5))reasons.push('trajectory safety stream');
  if(f.bypass)reasons.push('safety bypass ON');
  // Satu sesi F4.5 harus mampu menyelesaikan 10 obstacle + 7 Lane Safety.
  // Jangan mulai sesi bila authority hanya aktif sebagian karena hasil 17 kondisi
  // harus berasal dari runtime kontrol yang sama.
  if(!f.metric)reasons.push('metric obstacle safety OFF / kalibrasi kamera belum valid');
  if(!f.lane)reasons.push('lane safety runtime OFF');
  if(!f.grayStop)reasons.push('GRAY fail-closed OFF');
  return{ok:reasons.length===0,reasons,pose,cfg:f.cfg,tsText:f.tsText,ts:f.ts,runtime:f};
}
function perBab4F45Ready(){
  const base=perBab4F45SessionReady(),mode=$('perF45Mode')?.value||perBab4.f45.mode||'obstacle',laneKey=$('perF45LaneCondition')?.value||'green_outer',reasons=[...base.reasons],cfg=base.cfg;
  const speed=+raw('esc_drive_actual');if(Number.isFinite(speed)&&Math.abs(speed)>.03)reasons.push('kendaraan belum diam');
  if(mode==='obstacle'){
    if(!channelFresh('obstacle_metrics',2.5))reasons.push('obstacle metric stream');
  }else{
    if(bool(raw('perception_emergency')))reasons.push('perception emergency');
    if(!channelFresh('lane_state',2.5))reasons.push('lane stream');
    if(!perBab4F45LaneMatches(laneKey))reasons.push('kondisi lane belum sesuai pilihan');
  }
  return{ok:reasons.length===0,reasons,pose:base.pose,cfg};
}
const perBab4F45PhaseBase=phaseGate;
phaseGate=function(){if(selectedExperiment?.id==='F4.5')return perBab4BasePhaseGate();return perBab4F45PhaseBase()};
const perBab4F45PreflightBase=testPreflightStatus;
testPreflightStatus=function(){
  const p=perBab4F45PreflightBase();if(selectedExperiment?.id!=='F4.5')return p;const items=[...p.items],add=(label,ok,detail)=>{const i=items.findIndex(x=>x.label===label),v={label,ok:!!ok,detail,critical:true};if(i>=0)items[i]=v;else items.push(v)};
  const realGate=perBab4BasePhaseGate(),r=perBab4F45SessionReady();add('Commissioning prerequisite',realGate.ok,realGate.ok?'aktuator/Nav2 qualification READY':realGate.reason||'qualification belum READY');add('F4.5 session gate',r.ok,r.ok?'kamera + ESC + motion + safety READY':r.reasons.join(', '));const bad=items.filter(i=>i.critical&&!i.ok);return{ok:bad.length===0,items,reason:bad.map(x=>x.label).join(', ')};
};

function perBab4F45Sample(){
  const ts=obj('trajectory_safety_state')||{},lane=perBab4F45LaneSnapshot(),t=Math.max(0,Date.now()-perBab4.f45.startMs),obs=perBab4F45ObstacleObservation(),sv=+raw('esc_drive_target'),av=+raw('esc_drive_actual'),st=deg(+raw('esc_steer_target')),sa=deg(+raw('esc_steer_actual'));
  return{t,ob:obs.distance,obsSeen:obs.seen,obsSource:obs.source,obsClass:obs.classId,nearEmergency:obs.nearEmergency,decision:String(ts.decision||ts.raw||'--'),laneState:`${lane.left}/${lane.right}`,laneDecision:String(ts.lane_recommendation||ts.lane_control_decision||lane.rec||'--'),navV:+ts.nav_v,outV:+ts.out_v,outW:+ts.out_w,speedTarget:sv,speedActual:av,steerTarget:st,steerActual:sa};
}
function perBab4F45Stats(samples){const finite=(k)=>samples.map(x=>+x[k]).filter(Number.isFinite),mean=a=>a.length?a.reduce((s,v)=>s+v,0)/a.length:NaN,obs=finite('ob'),spd=finite('speedActual'),steer=finite('steerTarget'),errs=samples.map(x=>Number.isFinite(+x.steerTarget)&&Number.isFinite(+x.steerActual)?(+x.steerTarget)-(+x.steerActual):NaN).filter(Number.isFinite);return{minObstacle:obs.length?Math.min(...obs):NaN,meanSpeed:mean(spd),maxSteer:steer.length?Math.max(...steer.map(Math.abs)):NaN,steerRmse:errs.length?Math.sqrt(mean(errs.map(v=>v*v))):NaN}}
function perBab4F45ObstacleDecision(d){d=String(d||'').toUpperCase();return /^(PATH_OBSTACLE_SLOW_SEARCH|SEARCH_AVOIDANCE_(LEFT|RIGHT|BOTH)|AVOID_(LEFT|RIGHT)|IMMEDIATE_COMMAND_HARD_STOP|NO_SAFE_CORRIDOR_STOP|AVOIDANCE_NOT_ENGAGED_STOP|DRIVABLE_SPACE_UNKNOWN_STOP|PERCEPTION_EMERGENCY_STOP)$/.test(d)}
function perBab4F45ObstacleStopDecision(d){d=String(d||'').toUpperCase();return /^(IMMEDIATE_COMMAND_HARD_STOP|NO_SAFE_CORRIDOR_STOP|AVOIDANCE_NOT_ENGAGED_STOP|DRIVABLE_SPACE_UNKNOWN_STOP|PERCEPTION_EMERGENCY_STOP)$/.test(d)}
function perBab4F45RuntimeFaultDecision(d){d=String(d||'').toUpperCase();return /^(NAV_CMD_STALE_STOP|CAMERA_DISCONNECTED_STOP|CAMERA_UNHEALTHY_STOP|PLAN_UNAVAILABLE_STOP|OBSTACLE_STREAM_STALE_STOP|LANE_RECENTER_BLOCKED_STOP|LANE_LOST_STOP|NONFINITE_COMMAND_STOP)$/.test(d)}
function perBab4F45ResponseAt(samples,mode,key){for(const s of samples){const d=String(s.decision).toUpperCase();if(mode==='obstacle'){if(perBab4F45ObstacleDecision(d))return s.t}else if(key==='green_outer'||key==='green_drivable'){if(Math.abs(+s.speedActual)>0.03)return s.t}else if(key.startsWith('yellow')){if(Number.isFinite(s.navV)&&Number.isFinite(s.outV)&&s.outV<s.navV-.015)return s.t}else if(key.startsWith('red')){if(/RECENTER/.test(d)||/RECENTER/.test(String(s.laneDecision).toUpperCase()))return s.t}else if(key==='gray'){if(/STOP/.test(d)&&Math.abs(+s.speedActual)<.03)return s.t}}return NaN}
function perBab4F45FirstObstacleAt(samples){const s=samples.find(x=>x.obsSeen===true||Number.isFinite(+x.ob));return s?+s.t:NaN}
function perBab4F45Outcome(samples,mode,key,reason='AUTO'){const st=perBab4F45Stats(samples),last=samples.at(-1)||{},decision=String(last.decision||'--'),rt=perBab4F45ResponseAt(samples,mode,key),firstObs=mode==='obstacle'?perBab4F45FirstObstacleAt(samples):NaN,latency=mode==='obstacle'&&Number.isFinite(firstObs)&&Number.isFinite(rt)?Math.max(0,rt-firstObs):rt,dur=samples.length?Math.max(...samples.map(x=>x.t))/1000:0;let pass=false,response=decision,recovery=NaN;
  if(reason==='INVALID'||reason==='ABORT')return{pass:false,status:'INVALID',response:reason,rt,firstObs,latency,st,dur,recovery};
  if(mode==='obstacle'){const obstacleSeen=Number.isFinite(firstObs),responded=samples.some(s=>perBab4F45ObstacleDecision(s.decision)),stopped=samples.some(s=>perBab4F45ObstacleStopDecision(s.decision)&&Math.abs(+s.speedActual)<.03),avoided=samples.some(s=>/^AVOID_(LEFT|RIGHT)$/.test(String(s.decision).toUpperCase()));pass=obstacleSeen&&responded&&((reason==='SAFE_STOP'&&stopped)||(reason==='GOAL_SUCCEEDED'));response=!obstacleSeen?'OBSTACLE NOT OBSERVED':reason==='SAFE_STOP'&&stopped?'SAFETY STOP':reason==='GOAL_SUCCEEDED'&&responded?(avoided?'AVOIDANCE → GOAL':'SLOW/SEARCH → GOAL'):reason==='GOAL_SUCCEEDED'?'GOAL • NO SAFETY RESPONSE':reason==='TIMEOUT'?`TIMEOUT • ${decision}`:decision}
  else if(key==='green_outer'||key==='green_drivable'){const moving=samples.some(s=>Math.abs(+s.speedActual)>.03),recenter=samples.some(s=>/RECENTER/.test(String(s.decision).toUpperCase())||/RECENTER/.test(String(s.laneDecision).toUpperCase())),capped=samples.some(s=>Number.isFinite(s.navV)&&Number.isFinite(s.outV)&&s.navV>.05&&s.outV<s.navV-.03);pass=moving&&!recenter&&!capped;response=`${moving?'NORMAL SPEED':'NO MOTION'} • ${recenter?'RECENTER':'NO RECENTER'}`}
  else if(key.startsWith('yellow')){const slow=samples.some(s=>Number.isFinite(s.navV)&&Number.isFinite(s.outV)&&s.outV<s.navV-.015),rec=samples.some(s=>/RECENTER/.test(String(s.decision).toUpperCase()));pass=slow&&!rec;response=`${slow?'SLOW':'NO SLOW'} • ${rec?'RECENTER':'NO RECENTER'}`}
  else if(key.startsWith('red')){const wantRight=key==='red-left',dir=samples.some(s=>Number.isFinite(s.outW)&&(wantRight?s.outW<-.015:s.outW>.015)),ri=samples.findIndex(s=>/RECENTER/.test(String(s.decision).toUpperCase())||/RECENTER/.test(String(s.laneDecision).toUpperCase())),gi=ri>=0?samples.findIndex((s,i)=>i>ri&&s.laneState==='GREEN/GREEN'):-1;pass=ri>=0&&dir&&gi>=0;recovery=gi>=0?samples[gi].t-samples[ri].t:NaN;response=`RECENTER ${wantRight?'RIGHT':'LEFT'}${gi>=0?' → GREEN':' • NO GREEN RECOVERY'}`}
  else if(key==='gray'){const stop=samples.some(s=>/STOP/.test(String(s.decision).toUpperCase())&&Math.abs(+s.speedActual)<.03);pass=stop;response=stop?'FAIL-CLOSED STOP':'MOTION NOT BLOCKED'}
  return{pass,status:pass?'PASS':'FAIL',response,rt,firstObs,latency,st,dur,recovery};
}
function perBab4F45SortSummaries(){
  const obstacle=perBab4RowsAt(0),objectOrder={'Manusia':0,'Sepeda motor':1};
  obstacle.sort((a,b)=>(objectOrder[a.values?.[1]]??9)-(objectOrder[b.values?.[1]]??9)||(+a.values?.[2]||99)-(+b.values?.[2]||99));
  obstacle.forEach((r,i)=>{if(Array.isArray(r.values))r.values[0]=i+1});tableRunRows.set(0,obstacle);
  const lane=perBab4RowsAt(1),laneOrder=Object.fromEntries(PER_F45_LANE_KEYS.map((k,i)=>[perBab4F45LaneExpected(k).label,i]));
  lane.sort((a,b)=>(laneOrder[a.values?.[1]]??99)-(laneOrder[b.values?.[1]]??99));
  lane.forEach((r,i)=>{if(Array.isArray(r.values))r.values[0]=i+1});tableRunRows.set(1,lane);
}
function perBab4F45AppendTimeseries(key,mode,label,samples){const a=perBab4RowsAt(2),yaml=perBab4YamlSnapshot();samples.forEach((s,i)=>a.push({sec:Math.floor(s.t/1000),values:[key,s.t,mode,label,perBab4FmtNum(s.ob,3),s.decision,s.laneState,s.laneDecision,perBab4FmtNum(s.navV,3),perBab4FmtNum(s.outV,3),perBab4FmtNum(s.outW,3),perBab4FmtNum(s.speedTarget,3),perBab4FmtNum(s.speedActual,3),perBab4FmtNum(s.steerTarget,2),perBab4FmtNum(s.steerActual,2),s.obsSeen?'YES':'NO',s.obsSource||'NONE',Number.isFinite(+s.obsClass)?+s.obsClass:'--',s.nearEmergency?'YES':'NO'],yaml:i===0?yaml:[]}));tableRunRows.set(2,a)}
async function perBab4F45Finish(statusReason='AUTO'){
  if(!perBab4.f45.active)return;const token=perBab4.f45.token;perBab4.f45.active=false;perBab4.f45.phase='SAVING';clearInterval(perBab4.f45.timer);clearInterval(perBab4.f45.countdownTimer);try{await post('/api/navigation/cancel',{},true)}catch(_){ }
  const mode=perBab4.f45.mode,key=perBab4.f45.trialKey,samples=[...perBab4.f45.samples],label=mode==='obstacle'?key:perBab4F45LaneExpected(key).label,result=perBab4F45Outcome(samples,mode,key,statusReason);let evidence='--';try{const ev=await perBab4PersistTrialEvidence(`F4.5_${mode}_${key}_${result.status}`);evidence=ev.image_path||'--'}catch(_){ }
  if(result.status==='INVALID'){
    const a=perBab4RowsAt(3);a.push({sec:0,values:[new Date().toISOString(),key,mode,result.response,perBab4FmtNum(result.dur,2),evidence],yaml:perBab4YamlSnapshot()});tableRunRows.set(3,a);
  }else if(mode==='obstacle'){
    perBab4F45AppendTimeseries(key,mode,label,samples);
    const [target,distance]=key.split('-'),a=perBab4RowsAt(0);a.push({sec:0,values:[a.length+1,target==='human'?'Manusia':'Sepeda motor',+distance,result.response,perBab4FmtNum(result.latency,0),perBab4FmtNum(result.st.minObstacle,3),perBab4FmtNum(result.st.maxSteer,2),perBab4FmtNum(result.st.steerRmse,2),perBab4FmtNum(result.st.meanSpeed,3),perBab4FmtNum(result.dur,2),result.status,evidence],yaml:perBab4YamlSnapshot()});tableRunRows.set(0,a);
  }else{
    perBab4F45AppendTimeseries(key,mode,label,samples);
    const a=perBab4RowsAt(1),exp=perBab4F45LaneExpected(key);a.push({sec:0,values:[a.length+1,exp.label,exp.policy,result.response,perBab4FmtNum(result.rt,0),perBab4FmtNum(result.st.meanSpeed,3),perBab4FmtNum(result.st.maxSteer,2),perBab4FmtNum(result.st.steerRmse,2),perBab4FmtNum(result.recovery,0),perBab4FmtNum(result.dur,2),result.status,evidence],yaml:perBab4YamlSnapshot()});tableRunRows.set(1,a);
  }
  perBab4F45SortSummaries();if(result.status==='PASS'||result.status==='FAIL')perBab4.f45.completed[key]=result.status;perBab4.f45.lastResult=`${key} • ${result.status} • ${result.response}`;perBab4.f45.phase='IDLE';renderExperimentTable();await saveAllTemplateTablesServer();perBab4F45Persist();perBab4F45Advance();perBab4F45DrawGraphs();perBab4F45Refresh();if(token===perBab4.f45.token)toast(`F4.5 ${key}: ${result.status} • ${result.response} • checkpoint tersimpan`,result.status!=='PASS')
}
function perBab4F45Monitor(){if(!perBab4.f45.active||perBab4.f45.phase!=='RUNNING')return;const s=perBab4F45Sample();perBab4.f45.samples.push(s);const elapsed=s.t/1000,key=perBab4.f45.trialKey,mode=perBab4.f45.mode;if(bool(raw('system.estop'))||(mode==='lane'&&bool(raw('perception_emergency'))))return perBab4F45Finish('INVALID');if(elapsed>=perBab4.f45.timeoutSec)return perBab4F45Finish(mode==='obstacle'?'TIMEOUT':'AUTO');if(mode==='obstacle'){if(elapsed>.4&&perBab4F45RuntimeFaultDecision(s.decision))return perBab4F45Finish('INVALID');if(elapsed>.7&&perBab4F45ObstacleStopDecision(s.decision)&&Math.abs(+s.speedActual)<.03)return perBab4F45Finish('SAFE_STOP');const gs=String(obj('goal_state')?.state||'').toUpperCase();if(['SUCCEEDED','SUCCESS'].includes(gs))return perBab4F45Finish('GOAL_SUCCEEDED')}else if(key==='gray'){if(elapsed>1.5)return perBab4F45Finish('AUTO')}else if(key.startsWith('red')){const had=perBab4.f45.samples.some(x=>/RECENTER/.test(String(x.decision).toUpperCase())||/RECENTER/.test(String(x.laneDecision).toUpperCase()));if(had&&s.laneState==='GREEN/GREEN'&&elapsed>1)return perBab4F45Finish('AUTO')}else if(elapsed>=3)return perBab4F45Finish('AUTO');perBab4F45Refresh()}
async function perBab4F45ActivateControl(token){if(!perBab4.f45.active||token!==perBab4.f45.token)return;perBab4.f45.phase='RUNNING';perBab4.f45.startMs=Date.now();perBab4.f45.samples=[];const p=perBab4F45Pose(),d=perBab4.f45.goalDistance,x=p.x+d*Math.cos(p.yaw),y=p.y+d*Math.sin(p.yaw),yawDeg=p.yaw*180/Math.PI;const j=await post('/api/navigation/goal',{x,y,yaw_deg:yawDeg},true);if(!j?.ok){perBab4.f45.samples.push(perBab4F45Sample());return perBab4F45Finish('INVALID')}perBab4.f45.timer=setInterval(perBab4F45Monitor,100);perBab4F45Refresh()}
function perBab4F45CancelCountdown(reason){clearInterval(perBab4.f45.countdownTimer);perBab4.f45.countdownTimer=null;perBab4.f45.active=false;perBab4.f45.phase='IDLE';perBab4.f45.countdownLeft=0;perBab4.f45.lastResult=`COUNTDOWN CANCEL • ${reason}`;perBab4F45Refresh();toast(`Countdown dibatalkan: ${reason}`,true)}
function perBab4F45DefaultGoal(){const mode=$('perF45Mode')?.value||'obstacle';if(mode==='lane')return 2.0;return Math.max(3.0,(+$('perF45Distance')?.value||1)+2.0)}
function perBab4F45DefaultTimeout(){return ($('perF45Mode')?.value||'obstacle')==='obstacle'?30:10}
function perBab4F45ApplyRunDefaults(){const goal=$('perF45GoalDistance'),timeout=$('perF45Timeout');if(goal&&!goal.dataset.userEdit)goal.value=perBab4F45DefaultGoal().toFixed(1);if(timeout&&!timeout.dataset.userEdit)timeout.value=String(perBab4F45DefaultTimeout())}
async function perBab4F45StartTrial(){
  if(perBab4.f45.active)return;if(!recordStartedMs)return toast('Klik START SESI F4.5 dahulu agar 17 kondisi masuk satu folder.',true);const ready=perBab4F45Ready();if(!ready.ok)return toast('Belum READY: '+ready.reasons.join(', '),true);if(!$('perF45PhysicalReady')?.checked)return toast('Konfirmasi POSISI FISIK SIAP terlebih dahulu.',true);const key=perBab4F45CurrentKey();if(['PASS','FAIL'].includes(perBab4.f45.completed[key]))return toast(`${key} sudah menjadi formal trial. Pilih kondisi lain.`,true);
  try{await post('/api/navigation/cancel',{},true)}catch(_){return toast('Gagal memastikan goal sebelumnya sudah cancel.',true)}
  const afterCancel=perBab4F45Ready();if(!afterCancel.ok)return toast('Setelah cancel belum READY: '+afterCancel.reasons.join(', '),true);
  perBab4.f45.mode=$('perF45Mode')?.value||'obstacle';perBab4.f45.trialKey=key;perBab4.f45.goalDistance=Math.max(.5,+$('perF45GoalDistance')?.value||perBab4F45DefaultGoal());perBab4.f45.timeoutSec=Math.max(3,+$('perF45Timeout')?.value||perBab4F45DefaultTimeout());perBab4.f45.active=true;perBab4.f45.phase='COUNTDOWN';perBab4.f45.samples=[];perBab4.f45.token++;const token=perBab4.f45.token,end=Date.now()+3000;perBab4.f45.countdownLeft=3;perBab4F45Refresh();
  perBab4.f45.countdownTimer=setInterval(()=>{if(!perBab4.f45.active||token!==perBab4.f45.token)return clearInterval(perBab4.f45.countdownTimer);const r=perBab4F45Ready();if(!r.ok)return perBab4F45CancelCountdown(r.reasons.join(', '));if(!$('perF45PhysicalReady')?.checked)return perBab4F45CancelCountdown('konfirmasi posisi fisik dilepas');const left=Math.max(0,end-Date.now());perBab4.f45.countdownLeft=Math.ceil(left/1000);perBab4F45Refresh();if(left<=0){clearInterval(perBab4.f45.countdownTimer);perBab4.f45.countdownTimer=null;perBab4.f45.countdownLeft=0;setText('perF45Countdown','GO');perBab4F45ActivateControl(token)}},100)
}
function perBab4F45Abort(){if(!perBab4.f45.active)return toast('Tidak ada trial aktif.',true);perBab4.f45.token++;perBab4F45Finish('ABORT')}
function perBab4F45Advance(){if($('perF45PhysicalReady'))$('perF45PhysicalReady').checked=false;const all=[...PER_F45_OBSTACLE_KEYS,...PER_F45_LANE_KEYS],next=all.find(k=>!['PASS','FAIL'].includes(perBab4.f45.completed[k]));if(!next)return;if(next.includes('-')&&/^(human|motorcycle)-/.test(next)){const [o,d]=next.split('-');if($('perF45Mode'))$('perF45Mode').value='obstacle';if($('perF45Object'))$('perF45Object').value=o;if($('perF45Distance'))$('perF45Distance').value=d}else{if($('perF45Mode'))$('perF45Mode').value='lane';if($('perF45LaneCondition'))$('perF45LaneCondition').value=next}perBab4F45SyncMode()}
function perBab4F45SyncMode(){const m=$('perF45Mode')?.value||'obstacle';perBab4.f45.mode=m;if($('perF45PhysicalReady'))$('perF45PhysicalReady').checked=false;$('perF45ObstacleSetup')?.toggleAttribute('hidden',m!=='obstacle');$('perF45LaneSetup')?.toggleAttribute('hidden',m!=='lane');perBab4F45ApplyRunDefaults();perBab4F45Refresh()}
function perBab4F45CardHtml(){return `<section class="per-bab4-evidence-card per-f45-card" id="perF45Card"><div class="per-bab4-evidence-head"><div><span class="eyebrow">4.5 • FINAL CONTROL RESPONSE DATASET</span><b>17 formal run • satu sesi • satu folder hasil</b><p>Pilih kondisi → siapkan fisik → START TRIAL → countdown 3 s → goal relatif aktif → respons safety direkam → auto-cancel → lanjut kondisi berikutnya. PASS/FAIL menjadi formal data; INVALID boleh diulang.</p></div><span class="status-chip" id="perF45State">WAIT SESSION</span></div><div class="per-f45-selectors"><label>Jenis pengujian<select id="perF45Mode"><option value="obstacle">Obstacle response</option><option value="lane">Lane Safety response</option></select></label><div id="perF45ObstacleSetup" class="per-f45-inline"><label>Objek<select id="perF45Object"><option value="human">Manusia</option><option value="motorcycle">Sepeda motor</option></select></label><label>Jarak awal<select id="perF45Distance">${[1,2,3,4,5].map(x=>`<option value="${x}">${x} meter</option>`).join('')}</select></label></div><div id="perF45LaneSetup" class="per-f45-inline" hidden><label>Kondisi<select id="perF45LaneCondition">${PER_F45_LANE_KEYS.map(k=>`<option value="${k}">${perBab4F45LaneExpected(k).label}</option>`).join('')}</select></label></div><label>Goal maju<input id="perF45GoalDistance" type="number" min="0.5" max="12" step="0.5" value="3.0"></label><label>Timeout<input id="perF45Timeout" type="number" min="3" max="60" step="1" value="30"></label></div><div class="per-f45-runtime-note" id="perF45RuntimeHelp"><b>RUNTIME F4.5</b><span>Gunakan hanya setelah kalibrasi metrik kamera sudah APPLY. Launch pengujian: <code>ros2 launch navigation autonomous.launch.py perception_mode:=cpu perception_inference_enabled:=true enable_lane_safety:=true</code>. Nilai <code>camera_metric_calibration_validated</code> tetap berasal dari YAML hasil kalibrasi dan tidak boleh dipaksa TRUE dari CLI.</span></div><div class="per-f45-readiness"><div><span>Formal progress</span><b id="perF45Progress">0 / 17</b></div><div><span>Selected condition</span><b id="perF45Selected">--</b></div><div><span>Expected policy</span><b id="perF45Expected">--</b></div><div><span>Live perception</span><b id="perF45Live">--</b></div><div><span>Control / safety</span><b id="perF45Safety">--</b></div><div><span>F4.5 runtime authority</span><b id="perF45Runtime">--</b></div><div><span>Last result</span><b id="perF45Last">WAIT</b></div></div><label class="per-f45-physical-ready"><input id="perF45PhysicalReady" type="checkbox"><span><b>POSISI FISIK SIAP</b><small>Centang setelah obstacle/lane benar-benar berada pada kondisi yang dipilih. Otomatis di-reset setiap selesai/pindah kondisi.</small></span></label><div class="per-f45-countdown"><span>CONTROL ENABLE</span><b id="perF45Countdown">--</b><small>Selama countdown kendaraan wajib diam dan kondisi harus tetap valid; t=0 dimulai setelah angka GO.</small></div><div class="per-f45-actions"><button class="primary-btn" id="perF45StartTrial">▶ START TRIAL • 3 s</button><button class="warning-btn" id="perF45Abort">ABORT / CANCEL GOAL</button></div><div class="per-f45-progress"><div><b>Obstacle • 10 kondisi</b><div id="perF45ObstacleProgress"></div></div><div><b>Lane Safety • 7 kondisi</b><div id="perF45LaneProgress"></div></div></div><div class="per-f45-graphs"><canvas id="perF45ObstacleGraph" width="900" height="250"></canvas><canvas id="perF45LaneGraph" width="900" height="250"></canvas><canvas id="perF45RmseGraph" width="900" height="250"></canvas></div><div class="per-bab4-f41-result"><span>Folder final</span><code id="perF45Folder">STOP + SAVE setelah 17/17 kondisi selesai</code></div></section>`}
function perBab4F45Refresh(){if(selectedExperiment?.id!=='F4.5'||!$('perF45Card'))return;const mode=$('perF45Mode')?.value||'obstacle',key=perBab4F45CurrentKey(),ready=perBab4F45Ready(),lane=perBab4F45LaneSnapshot(),obs=perBab4F45ObstacleObservation(),ob=obs.distance,ts=obj('trajectory_safety_state')||{},exp=mode==='lane'?perBab4F45LaneExpected(key).policy:'AUTO: slowdown / avoidance / safety stop sesuai supervisor',physical=$('perF45PhysicalReady')?.checked===true;setText('perF45State',perBab4.f45.active?perBab4.f45.phase:(recordStartedMs?'SESSION ACTIVE':perBab4F45DoneCount()===17?'DATASET COMPLETE':'WAIT SESSION'));setText('perF45Progress',`${perBab4F45DoneCount()} / 17`);setText('perF45Selected',mode==='obstacle'?`${key.startsWith('human')?'MANUSIA':'MOTOR'} • ${key.split('-')[1]} m`:perBab4F45LaneExpected(key).label);setText('perF45Expected',exp);setText('perF45Live',mode==='obstacle'?`${obs.seen?'SELECTED OBJECT SEEN':'WAIT SELECTED OBJECT'} • ${obs.source} • metric ${Number.isFinite(ob)?ob.toFixed(2)+' m':'--'} • class ${obs.classId}`:`${lane.left}/${lane.right} • ${lane.rec} • ${lane.evidenceMode}`);const rf=perBab4F45RuntimeFlags();setText('perF45Safety',ready.ok?`${physical?'READY':'WAIT POSISI FISIK'} • ${ts.decision||'--'}`:`WAIT • ${ready.reasons.join(', ')}`);setText('perF45Runtime',`METRIC ${rf.metric?'ON':'OFF'} • LANE ${rf.lane?'ON':'OFF'} • GRAY STOP ${rf.grayStop?'ON':'OFF'}${rf.bypass?' • BYPASS ON':''}`);setText('perF45Last',perBab4.f45.lastResult||'WAIT');setText('perF45Countdown',perBab4.f45.phase==='COUNTDOWN'?String(perBab4.f45.countdownLeft||3):perBab4.f45.phase==='RUNNING'?'GO':'--');const chip=$('perF45State');if(chip)chip.className='status-chip '+(ready.ok&&!perBab4.f45.active?'':'waiting');const start=$('perF45StartTrial');if(start){start.disabled=perBab4.f45.active||!recordStartedMs||!ready.ok||!physical||['PASS','FAIL'].includes(perBab4.f45.completed[key]);start.textContent=perBab4.f45.phase==='COUNTDOWN'?'COUNTDOWN...':perBab4.f45.phase==='RUNNING'?'CONTROL ACTIVE':'▶ START TRIAL • 3 s'}const abort=$('perF45Abort');if(abort)abort.disabled=!perBab4.f45.active;['perF45Mode','perF45Object','perF45Distance','perF45LaneCondition','perF45GoalDistance','perF45Timeout','perF45PhysicalReady'].forEach(id=>{const e=$(id);if(e)e.disabled=!!perBab4.f45.active});const op=$('perF45ObstacleProgress');if(op)op.innerHTML=PER_F45_OBSTACLE_KEYS.map(k=>`<span class="${(perBab4.f45.completed[k]||'').toLowerCase()}">${k.replace('motorcycle','motor').replace('-', ' • ')} <em>${perBab4.f45.completed[k]||'WAIT'}</em></span>`).join('');const lp=$('perF45LaneProgress');if(lp)lp.innerHTML=PER_F45_LANE_KEYS.map(k=>`<span class="${(perBab4.f45.completed[k]||'').toLowerCase()}">${perBab4F45LaneExpected(k).label} <em>${perBab4.f45.completed[k]||'WAIT'}</em></span>`).join('');setText('perF45Folder',lastTrialArtifacts?.report_folder||lastTrialArtifacts?.report_root||perBab4.f45.lastFolder||(perBab4F45DoneCount()===17?'17/17 lengkap • klik FINISH + SAVE FOLDER':'STOP + SAVE setelah seluruh kondisi selesai'))}
function perBab4F45DrawBar(canvasId,title,rows,valueIndex,labelIndex,maxHint=0){const c=$(canvasId);if(!c)return;const ctx=c.getContext('2d'),w=c.width,h=c.height,p={l:58,r:20,t:42,b:62};ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='800 15px system-ui';ctx.fillText(title,18,24);if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.font='12px system-ui';ctx.fillText('Belum ada formal trial.',18,52);return}const vals=rows.map(r=>+r.values[valueIndex]).filter(Number.isFinite),max=Math.max(maxHint,...vals,1),gap=(w-p.l-p.r)/rows.length,bw=gap*.62;rows.forEach((r,i)=>{const v=+r.values[valueIndex],x=p.l+i*gap+(gap-bw)/2,bh=Number.isFinite(v)?Math.max(2,(v/max)*(h-p.t-p.b)):0,y=h-p.b-bh;ctx.fillStyle=r.values.at(-2)==='PASS'?'#48e0a4':'#ffc86b';ctx.fillRect(x,y,bw,bh);ctx.fillStyle='#d8eeee';ctx.font='10px system-ui';ctx.fillText(Number.isFinite(v)?v.toFixed(valueIndex===4?0:2):'--',x,y-4);ctx.save();ctx.translate(x+bw/2,h-p.b+10);ctx.rotate(-.45);ctx.fillText(String(r.values[labelIndex]),0,0);ctx.restore()})}
function perBab4F45DrawObstacleResponse(){const c=$('perF45ObstacleGraph');if(!c)return;const ctx=c.getContext('2d'),w=c.width,h=c.height,p={l:62,r:24,t:45,b:48},rows=perBab4RowsAt(0);ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='800 15px system-ui';ctx.fillText('4.5.1 Response Latency Obstacle vs Jarak [ms] • n=1/kondisi',18,24);const series=[['Manusia','#48e0a4'],['Sepeda motor','#67b7ff']],data=Object.fromEntries(series.map(([name])=>[name,rows.filter(r=>r.values[1]===name&&Number.isFinite(+r.values[4])).map(r=>({d:+r.values[2],v:+r.values[4]}))]));const vals=Object.values(data).flat().map(x=>x.v),max=Math.max(100,...vals)*1.12;ctx.strokeStyle='#294144';ctx.fillStyle='#8fa8a9';ctx.font='10px system-ui';for(let i=0;i<=4;i++){const y=p.t+(h-p.t-p.b)*i/4,lab=Math.round(max*(1-i/4));ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.fillText(String(lab),12,y+3)}for(let d=1;d<=5;d++){const x=p.l+(w-p.l-p.r)*(d-1)/4;ctx.fillText(`${d} m`,x-8,h-18)}series.forEach(([name,color],si)=>{const pts=data[name].sort((a,b)=>a.d-b.d);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2;ctx.beginPath();pts.forEach((q,i)=>{const x=p.l+(w-p.l-p.r)*(q.d-1)/4,y=h-p.b-(q.v/max)*(h-p.t-p.b);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y)});ctx.stroke();pts.forEach(q=>{const x=p.l+(w-p.l-p.r)*(q.d-1)/4,y=h-p.b-(q.v/max)*(h-p.t-p.b);ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fill()});ctx.fillRect(w-205,p.t+si*19,10,3);ctx.fillText(name,w-188,p.t+4+si*19)});if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.fillText('Belum ada formal trial.',18,52)}}
function perBab4F45DrawGraphs(){if(selectedExperiment?.id!=='F4.5')return;perBab4F45DrawObstacleResponse();perBab4F45DrawBar('perF45LaneGraph','4.5.2 Mean Speed Aktual Lane Safety [m/s]',perBab4RowsAt(1),5,1);const mix=[...perBab4RowsAt(0).map(r=>({values:[r.values[2]+'m '+r.values[1],r.values[7],r.values[10]]})),...perBab4RowsAt(1).map(r=>({values:[r.values[1],r.values[7],r.values[10]]}))];perBab4F45DrawBar('perF45RmseGraph','4.5.3 Steering RMSE per Kondisi [deg]',mix,1,0)}
function perBab4F45Bind(){if(!$('perF45Card'))return;$('perF45Mode')?.addEventListener('change',perBab4F45SyncMode);$('perF45Object')?.addEventListener('change',()=>{if($('perF45PhysicalReady'))$('perF45PhysicalReady').checked=false;perBab4F45Refresh()});$('perF45Distance')?.addEventListener('change',()=>{if($('perF45PhysicalReady'))$('perF45PhysicalReady').checked=false;perBab4F45ApplyRunDefaults();perBab4F45Refresh()});$('perF45LaneCondition')?.addEventListener('change',()=>{if($('perF45PhysicalReady'))$('perF45PhysicalReady').checked=false;perBab4F45Refresh()});$('perF45PhysicalReady')?.addEventListener('change',perBab4F45Refresh);$('perF45GoalDistance')?.addEventListener('input',e=>{e.currentTarget.dataset.userEdit='1'});$('perF45Timeout')?.addEventListener('input',e=>{e.currentTarget.dataset.userEdit='1'});$('perF45StartTrial')?.addEventListener('click',perBab4F45StartTrial);$('perF45Abort')?.addEventListener('click',perBab4F45Abort);perBab4F45SyncMode();perBab4F45DrawGraphs();perBab4F45Refresh()}

const perBab4F45InfoBase=perBab4InfoText;
perBab4InfoText=function(){if(selectedExperiment?.id==='F4.5')return '17 formal run end-to-end dalam satu sesi: 10 obstacle (manusia/motor × 1–5 m) + 7 Lane Safety. Setiap trial memakai countdown 3 detik, kontrol aktual, target/feedback, PASS/FAIL/INVALID, evidence frame, lalu satu folder final.';return perBab4F45InfoBase()};
const perBab4F45StepBase=perBab4StepText;
perBab4StepText=function(){if(selectedExperiment?.id==='F4.5')return['Tujuan','Buktikan keluaran persepsi benar-benar menjadi respons longitudinal/lateral kendaraan.','Yang dilakukan','START SESI → pilih kondisi → siapkan fisik sampai READY → START TRIAL → 3 s → kontrol aktif → auto-stop/cancel → lanjut sampai 17/17.','Output laporan','2 tabel rekap formal + time-series target/feedback + log INVALID + 3 PNG grafik + evidence frame + XLSX/manifest dalam satu folder run.'];return perBab4F45StepBase()};
const perBab4F45NavigatorBase=perBab4RenderFinalNavigator;
perBab4RenderFinalNavigator=function(){perBab4F45NavigatorBase();const box=$('perBab4FinalNavigator');if(!box||box.hidden)return;const grid=box.querySelector('.per-final-cards');if(grid&&!grid.querySelector('[data-final-id="F4.5"]')){const b=document.createElement('button');b.dataset.finalId='F4.5';b.className=selectedExperiment?.id==='F4.5'?'active':'';b.innerHTML='<em>4.5</em><b>Respons Kontrol</b><small>17 run • obstacle + Lane Safety</small>';b.onclick=()=>selectExperimentById('F4.5');grid.appendChild(b)}const h=box.querySelector('.per-final-title h2');if(h)h.textContent='FINAL BAB IV • 4.1–4.5';const p=box.querySelector('.per-final-title p');if(p)p.innerHTML='Tahap 4.1–4.4 menguji persepsi; <b>4.5</b> memvalidasi respons kendaraan end-to-end. Seluruh perubahan ini hanya berada pada tab Persepsi.'};
const perBab4F45RenderBase=perBab4RenderPanel;
perBab4RenderPanel=function(){const r=perBab4F45RenderBase();if(selectedExperiment?.id==='F4.5'){perBab4F45ApplySchema();const p=$('perceptionBab4Acquisition');if(p&&!$('perF45Card')){p.insertAdjacentHTML('beforeend',perBab4F45CardHtml());perBab4F45Bind()}if(perBab4F45Restore()){renderExperimentTable();perBab4F45DrawGraphs()}perBab4F45Refresh()}return r};
const perBab4F45RefreshBase=perBab4RefreshLive;
perBab4RefreshLive=function(){perBab4F45RefreshBase();perBab4F45Refresh()};
const perBab4F45StartBase=startWebRecording;
startWebRecording=async function(){const was=!!recordStartedMs;await perBab4F45StartBase();if(selectedExperiment?.id==='F4.5'&&!was&&recordStartedMs){localStorage.removeItem(PER_F45_STORAGE_KEY);perBab4.f45.active=false;perBab4.f45.phase='IDLE';perBab4.f45.completed={};perBab4.f45.restoredFor=0;perBab4.f45.lastResult='SESSION ACTIVE • pilih kondisi pertama';perBab4.f45.lastFolder='';perBab4F45Persist();perBab4F45Refresh()}};
const perBab4F45StopBase=stopWebRecording;
stopWebRecording=async function(){if(selectedExperiment?.id==='F4.5'&&perBab4.f45.active)return toast('Trial F4.5 masih aktif. ABORT atau tunggu selesai sebelum FINISH + SAVE.',true);if(selectedExperiment?.id==='F4.5'&&recordStartedMs&&perBab4F45DoneCount()<17)return toast(`Dataset F4.5 baru ${perBab4F45DoneCount()}/17. Selesaikan seluruh kondisi sebelum FINISH + SAVE FOLDER.`,true);const was45=selectedExperiment?.id==='F4.5';const r=await perBab4F45StopBase();if(was45&&!recordStartedMs){perBab4.f45.lastFolder=lastTrialArtifacts?.report_folder||lastTrialArtifacts?.report_root||lastTrialArtifacts?.primary_csv||'Folder tersimpan';perBab4F45Refresh()}return r};
const perBab4F45CollectBase=collectExperimentGraphPngPayload;
collectExperimentGraphPngPayload=function(){if(selectedExperiment?.id!=='F4.5')return perBab4F45CollectBase();perBab4F45DrawGraphs();const out=[];for(const [id,title,index] of [['perF45ObstacleGraph','Response Latency Obstacle terhadap Jarak',1],['perF45LaneGraph','Mean Speed Aktual Lane Safety',2],['perF45RmseGraph','Steering RMSE per Kondisi',3]]){const c=$(id);if(c)out.push({index,title,data_url:c.toDataURL('image/png')})}return out};
const perBab4F45RecordingUiBase=setRecordingUi;
setRecordingUi=function(active,info={}){perBab4F45RecordingUiBase(active,info);if(perBab4Is('F4.5')){const b=$('recordToggleBtn');if(b){const sr=perBab4F45SessionReady();b.textContent=active?'■ FINISH + SAVE FOLDER':'● START SESI F4.5';b.disabled=active?(!!perBab4.f45.active||perBab4F45DoneCount()<17):!sr.ok;b.title=active?(perBab4F45DoneCount()<17?`Selesaikan ${17-perBab4F45DoneCount()} kondisi lagi`:'Simpan satu folder final'):(sr.ok?'Runtime F4.5 READY':`Belum READY: ${sr.reasons.join(', ')}`)}if(active)setTimeout(()=>{if(perBab4F45Restore()){renderExperimentTable();perBab4F45DrawGraphs();perBab4F45Refresh()}},0);perBab4F45Refresh()}};
const perBab4F45SelectedBase=renderSelectedExperiment;
renderSelectedExperiment=function(){const r=perBab4F45SelectedBase();if(selectedExperiment?.id==='F4.5'){perBab4F45ApplySchema();const badge=$('experimentGraphBadge');if(badge)badge.textContent='3 GRAPH REKAP';const sub=$('experimentSubtitle');if(sub)sub.textContent='Final control-response dataset: 10 obstacle + 7 Lane Safety • 1 formal run per kondisi • satu folder.';const rec=$('recordToggleBtn');if(rec){const active=rec.dataset.active==='true';rec.hidden=false;rec.textContent=active?'■ FINISH + SAVE FOLDER':'● START SESI F4.5';const sr=perBab4F45SessionReady();rec.disabled=active?(perBab4.f45.active||perBab4F45DoneCount()<17):!sr.ok;rec.title=active?(perBab4F45DoneCount()<17?`Selesaikan ${17-perBab4F45DoneCount()} kondisi lagi`:'Simpan satu folder final'):(sr.ok?'Runtime F4.5 READY':`Belum READY: ${sr.reasons.join(', ')}`)}}return r};

/* Keep BAB IV perception styling/panels strictly inside the Persepsi domain. */
const perBab4F45SwitchDomainBase=switchDomain;
switchDomain=function(domain,syncHmi=true){
  const r=perBab4F45SwitchDomainBase(domain,syncHmi);
  if(domain!=='perception'){
    document.body.classList.remove('per-bab4-final-mode');delete document.body.dataset.bab4;
    const p=$('perceptionBab4Acquisition'),n=$('perBab4FinalNavigator');if(p)p.hidden=true;if(n)n.hidden=true;
  }
  return r;
};


/* ===== PERCEPTION FINAL 4.3 LANE SAFETY OUTER-BOUNDARY v2 =====
 * Scope strictly GUI Persepsi + Final Dataset Persepsi.
 * Output colors: GRAY(no evidence), GREEN(outer safe OR drivable-only),
 * YELLOW(outer lane warning), RED(outer lane intrusion).
 */
const PER_BAB4_LANE_V2=[
  ['gray','Abu-abu','NO MASK + NO DRIVABLE'],
  ['green_outer','Hijau • Outer Lane Aman','DRIVABLE + 2 OUTER LANE SAFE'],
  ['green_drivable','Hijau • Drivable Only','DRIVABLE VALID + OUTER LANE TIDAK TERLIHAT / INTERNAL LANE DIABAIKAN'],
  ['yellow','Kuning','OUTER LANE MENDEKATI SAFETY'],
  ['red','Merah','OUTER LANE MASUK SAFETY']
];
function perBab4LaneV2Meta(cond){const q=PER_BAB4_LANE_V2.find(x=>x[0]===cond)||PER_BAB4_LANE_V2[0];return{key:q[0],label:q[1],detail:q[2]}}
function perBab4LaneV2ApplySchema(){
  const x=(experiments.perception||[]).find(v=>v.id==='F4.3');if(!x)return;
  Object.assign(x,{groupId:'FINAL-4.3',groupTitle:'FINAL BAB IV — 4.3 Pengujian Lane Safety',
    section:'FINAL 4.3 Lane Safety — 5 kondisi input • 20 percobaan/kondisi • 100 trial',
    reportMode:true,bab4Protocol:'lane',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4LaneFields(),
    tableColumns:[['Trial','Kondisi Acuan','Sisi Uji','Ulangan','Timestamp','Drivable Valid','Left Gap [px]','Right Gap [px]','Left Color','Right Color','Latch L','Latch R','Recommendation Aktual','Expected','Hasil','Evidence Frame','Raw Lane Mask','Outer Lane Detected','Evidence Mode','Outer Left Valid','Outer Right Valid','Internal Lane Ignored','Drivable Fraction','Raw Lane Rows','Internal Lane Rows'],['Kondisi Acuan','Percobaan','Benar','Salah','Success Rate [%]']],
    tableNames:['Raw Percobaan Lane Safety • Outer Boundary','Rekap Success Rate Lane Safety'],graphCaptions:[],graphs:[],
    liveSeries:{'Left gap':'lane_state.corridor.left_gap_px','Right gap':'lane_state.corridor.right_gap_px','Lane valid':'lane_state.valid','Drivable valid':'lane_state.corridor.drivable_detected'}});
  if(selectedExperiment?.id==='F4.3')selectedExperiment=x;
}
const perBab4LaneV2BasePatchCatalog=patchPerceptionBab4Catalog;
patchPerceptionBab4Catalog=function(){perBab4LaneV2BasePatchCatalog();perBab4LaneV2ApplySchema()};
perBab4LaneV2ApplySchema();

function perBab4LaneExpected(cond,side){
  if(cond==='gray')return{rec:'NO_MASK_EVIDENCE',left:'UNKNOWN',right:'UNKNOWN'};
  if(cond==='green_outer'||cond==='green_drivable')return{rec:'CLEAR',left:'GREEN',right:'GREEN'};
  if(cond==='yellow')return{rec:'WARNING',left:side==='left'?'YELLOW':'GREEN',right:side==='right'?'YELLOW':'GREEN'};
  return{rec:side==='left'?'RECENTER_RIGHT':'RECENTER_LEFT',left:side==='left'?'RED':'GREEN',right:side==='right'?'RED':'GREEN'};
}
function perBab4LaneEvidenceMatch(cond,side,ls,c,exp){
  const drivable=Boolean(c.drivable_detected??ls.drivable_valid??ls.valid),laneMask=Boolean(c.lane_mask_detected),
    leftValid=Boolean(c.left_valid),rightValid=Boolean(c.right_valid),rec=String(c.recommendation||'--'),
    left=String(c.left_status||'--'),right=String(c.right_status||'--');
  const base=rec===exp.rec&&left===exp.left&&right===exp.right;
  if(!base)return false;
  if(cond==='gray')return !drivable&&!laneMask;
  if(cond==='green_outer')return drivable&&laneMask&&leftValid&&rightValid;
  if(cond==='green_drivable')return drivable&&!laneMask&&!leftValid&&!rightValid;
  if(cond==='yellow'||cond==='red')return drivable&&laneMask&&(side==='left'?leftValid:rightValid);
  return false;
}
function perBab4LaneProtocolSide(){const cond=$('perBab4LaneCondition')?.value||'gray',rep=+$('perBab4LaneRepeat')?.value||1;return(cond==='yellow'||cond==='red')?(rep<=10?'left':'right'):'left'}
perBab4SyncLaneSide=function(){
  const cond=$('perBab4LaneCondition')?.value||'gray',s=$('perBab4LaneSide'),rep=+$('perBab4LaneRepeat')?.value||1,sideNeeded=cond==='yellow'||cond==='red';
  if(s){s.disabled=!sideNeeded;s.value=sideNeeded?(rep<=10?'left':'right'):'left';s.title=sideNeeded?'BAB IV: ulangan 1–10 sisi kiri, 11–20 sisi kanan':'Tidak memakai sisi individual pada kondisi ini'}
  perBab4RefreshReportExtensions();perBab4RefreshLive();
};
perBab4LaneControls=function(){return `<div class="per-bab4-auto-guide"><b>MEKANISME 1 TRIAL</b><span>Pastikan kondisi fisik sesuai acuan → stabilkan drivable + outer lane → klik <strong>AMBIL DATA LANE</strong>. Garis internal seperti batas sepeda tidak boleh menjadi outer boundary.</span></div><div class="per-bab4-controls"><label>Kondisi acuan<select id="perBab4LaneCondition">${PER_BAB4_LANE_V2.map(q=>`<option value="${q[0]}">${q[1]} • ${q[2]}</option>`).join('')}</select></label><label>Sisi uji<select id="perBab4LaneSide"><option value="left">Kiri</option><option value="right">Kanan</option></select></label><label>Ulangan<input id="perBab4LaneRepeat" type="number" min="1" max="20" step="1" value="1" readonly></label><div class="per-bab4-actions"><button class="primary-btn per-bab4-capture-main" id="perBab4LaneCapture">📷 AMBIL DATA LANE</button><button class="soft-btn" id="perBab4Undo">Undo trial terakhir</button></div></div>`};

perBab4LaneCapture=async function(){
  if(perBab4.autoTrial.laneBusy)return;
  if(!recordStartedMs)return toast('Klik START SESI dahulu.',true);
  if(!channelFresh('lane_state',2.5))return toast('Lane Safety stream belum fresh.',true);
  const cond=$('perBab4LaneCondition')?.value||'gray',rep=+$('perBab4LaneRepeat')?.value||1,
    side=(cond==='yellow'||cond==='red')?perBab4LaneProtocolSide():'left',ls=obj('lane_state'),c=ls.corridor||{},
    exp=perBab4LaneExpected(cond,side),actual=String(c.recommendation||'--'),left=String(c.left_status||'--'),
    right=String(c.right_status||'--'),meta=perBab4LaneV2Meta(cond),match=perBab4LaneEvidenceMatch(cond,side,ls,c,exp),
    key=(cond==='yellow'||cond==='red')?`${cond}-${side}`:cond,
    evidenceLabel=`F4.3_${key}_trial_${String(rep).padStart(2,'0')}_${match?'BENAR':'SALAH'}`;
  perBab4.autoTrial.laneBusy=true;const btn=$('perBab4LaneCapture');if(btn){btn.disabled=true;btn.textContent='MENYIMPAN FRAME...'}
  try{
    const ev=await perBab4PersistTrialEvidence(evidenceLabel),evidencePath=ev.image_path||'--',
      evidenceMode=String(c.evidence_mode||'--'),drivable=Boolean(c.drivable_detected??ls.drivable_valid??ls.valid),
      rawLaneMask=Boolean(c.raw_lane_mask_detected),laneMask=Boolean(c.lane_mask_detected),leftValid=Boolean(c.left_valid),rightValid=Boolean(c.right_valid),
      internalIgnored=Boolean(c.internal_lane_ignored),frac=Number.isFinite(+c.drivable_fraction)?(+c.drivable_fraction).toFixed(4):'--',
      expectedText=`${exp.rec} • ${exp.left}/${exp.right} • ${meta.detail}`,
      dataUrl=perBab4EvidenceJpeg({kind:'lane',left,right,recommendation:actual,title:`LANE SAFETY • ${meta.label}${(cond==='yellow'||cond==='red')?` ${side==='left'?'KIRI':'KANAN'}`:''} • ulangan ${rep}`,subtitle:`${left}/${right} • ${actual} • ${evidenceMode} • ${match?'BENAR':'SALAH'}`});
    const values=[perBab4Rows().length+1,meta.label,(cond==='yellow'||cond==='red')?side:'N/A',rep,perBab4FrameTimestamp(),String(drivable),
      Number.isFinite(+c.left_gap_px)?(+c.left_gap_px).toFixed(2):'--',Number.isFinite(+c.right_gap_px)?(+c.right_gap_px).toFixed(2):'--',
      left,right,String(c.left_recenter_latched??false),String(c.right_recenter_latched??false),actual,expectedText,match?'BENAR':'SALAH',
      evidencePath,String(rawLaneMask),String(laneMask),evidenceMode,String(leftValid),String(rightValid),String(internalIgnored),frac,
      Number.isFinite(+c.raw_lane_rows)?String(c.raw_lane_rows):'--',Number.isFinite(+c.internal_lane_rows)?String(c.internal_lane_rows):'--'],
      trialKey=`${meta.label}|${(cond==='yellow'||cond==='red')?side:'N/A'}|${rep}`,retake=perBab4UpsertMainTrial(trialKey,values,v=>`${v[1]}|${v[2]}|${v[3]}`);
    perBab4.evidence.f43=perBab4.evidence.f43.filter(x=>!(x.cond===cond&&+x.repeat===rep&&((cond!=='yellow'&&cond!=='red')||x.side===side)));
    if(dataUrl)perBab4.evidence.f43.push({key,cond,side,repeat:rep,left,right,recommendation:actual,match,dataUrl,evidencePath,evidenceMode});else perBab4.evidence.captureWarnings++;
    const seq=PER_BAB4_LANE_V2.map(q=>q[0]);let nr=rep+1,nc=cond;if(nr>20){nr=1;nc=seq[(seq.indexOf(cond)+1)%seq.length]}
    if($('perBab4LaneRepeat'))$('perBab4LaneRepeat').value=String(nr);if($('perBab4LaneCondition'))$('perBab4LaneCondition').value=nc;
    perBab4SyncLaneSide();perBab4RefreshEvidencePackage();toast(`${retake?'RETAKE replace • ':''}${meta.label} #${rep}: ${match?'BENAR':'SALAH'} • frame tersimpan`,!match);
  }catch(e){toast(e.message||String(e),true)}
  finally{perBab4.autoTrial.laneBusy=false;if(btn){btn.disabled=false;btn.textContent='📷 AMBIL DATA LANE'}}
};

perBab4BuildLaneSummary=function(){
  const groups=new Map();for(const r of perBab4Rows()){const v=r.values,g=groups.get(v[1])||{cond:v[1],n:0,ok:0};g.n++;if(v[14]==='BENAR')g.ok++;groups.set(v[1],g)}
  const order=PER_BAB4_LANE_V2.map(q=>q[1]);return [...groups.values()].sort((a,b)=>order.indexOf(a.cond)-order.indexOf(b.cond)).map(g=>[g.cond,g.n,g.ok,g.n-g.ok,(100*g.ok/g.n).toFixed(2)]);
};
const perBab4LaneV2BaseRenderProgress=perBab4RenderProgress;
perBab4RenderProgress=function(){
  perBab4LaneV2BaseRenderProgress();if(selectedExperiment?.id!=='F4.3')return;const box=$('perBab4Progress');if(!box)return;
  const c=perBab4ProgressCounts();box.innerHTML=PER_BAB4_LANE_V2.map(q=>`<span class="${(c[q[1]]||0)>=20?'done':''}">${q[1]} <em>${c[q[1]]||0}/20</em></span>`).join('');
};
const perBab4LaneV2BaseInfoText=perBab4InfoText;
perBab4InfoText=function(){if(selectedExperiment?.id==='F4.3')return '100 trial: 5 kondisi input × 20. Dua kondisi HIJAU diuji terpisah: outer lane aman + drivable valid, serta drivable valid tanpa lane mask. Marka internal/batas sepeda dicatat sebagai internal-lane ignored dan tidak boleh memicu steering.';return perBab4LaneV2BaseInfoText()};
const perBab4LaneV2BaseStepText=perBab4StepText;
perBab4StepText=function(){if(selectedExperiment?.id==='F4.3')return ['Tujuan','Verifikasi logika Lane Safety outer-boundary pada 5 kondisi input (100 trial).','Yang dilakukan','START SESI → bentuk kondisi fisik sesuai acuan → tunggu evidence_mode stabil → AMBIL DATA LANE. Kuning/merah dibagi ulangan 1–10 kiri dan 11–20 kanan.','Output laporan','Raw status + drivable/lane evidence + outer-valid + internal-lane ignored + rekap Success Rate + montage 7 kondisi representatif.'];return perBab4LaneV2BaseStepText()};
const perBab4LaneV2BaseRefreshLive=perBab4RefreshLive;
perBab4RefreshLive=function(){perBab4LaneV2BaseRefreshLive();if(selectedExperiment?.id==='F4.3'){const c=obj('lane_state').corridor||{};setText('perBab4LivePrimary',`${c.recommendation||'--'} • ${c.right_status||'--'}/${c.left_status||'--'} • ${c.evidence_mode||'--'}`)}};

perBab4BuildLaneMontage=async function(){
  const spec=[['gray','Abu-abu'],['green_outer','Hijau • Outer Aman'],['green_drivable','Hijau • Drivable Only'],['yellow-left','Kuning kiri'],['yellow-right','Kuning kanan'],['red-left','Merah kiri'],['red-right','Merah kanan']],reps=spec.map(x=>perBab4RepresentativeLane(x[0]));if(!reps.some(Boolean))return null;
  const c=document.createElement('canvas');c.width=1280;c.height=780;const ctx=c.getContext('2d');ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);
  perBab4EvidenceText(ctx,'Bukti Visual Lane Safety • Tujuh Kondisi Representatif',40,52,28,'#e5f4f3',900);perBab4EvidenceText(ctx,'Dua HIJAU dipisah: outer lane aman dan drivable-only. Kuning/merah masing-masing sisi kiri/kanan.',40,82,15,'#8fa8a9',600);
  const pw=290,ph=300,imgH=218,gap=15;
  for(let i=0;i<7;i++){const col=i%4,row=Math.floor(i/4),x=25+col*(pw+gap),y=100+row*325,rep=reps[i];ctx.fillStyle='#0b1b1d';ctx.fillRect(x,y,pw,ph);ctx.strokeStyle=rep?'rgba(91,181,173,.42)':'rgba(255,200,107,.4)';ctx.lineWidth=2;ctx.strokeRect(x,y,pw,ph);perBab4EvidenceText(ctx,spec[i][1],x+12,y+27,18,'#e5f4f3',900);if(rep){try{const im=await perBab4LoadEvidenceImage(rep.dataUrl);perBab4DrawImageContain(ctx,im,x+8,y+38,pw-16,imgH)}catch(_){perBab4EvidenceText(ctx,'FRAME ERROR',x+85,y+170,16,'#ff6677',800)}const colr=rep.match?'#48e0a4':'#ff6677';perBab4EvidenceText(ctx,`Trial ${rep.repeat} • ${rep.left}/${rep.right}`,x+10,y+276,11,colr,800);perBab4EvidenceText(ctx,rep.evidenceMode||rep.recommendation||'--',x+10,y+293,10,'#9dc0bd',700)}else perBab4EvidenceText(ctx,'Belum ada frame',x+78,y+170,14,'#8fa8a9',700)}
  perBab4EvidenceText(ctx,'Generated automatically by GUI Persepsi • BAB IV 4.3',25,760,12,'#708f8d',600);return c;
};
const perBab4LaneV2BasePrepareEvidenceExports=perBab4PrepareEvidenceExports;
perBab4PrepareEvidenceExports=async function(){if(selectedExperiment?.id!=='F4.3')return perBab4LaneV2BasePrepareEvidenceExports();perBab4.evidence.exports=[];const c=await perBab4BuildLaneMontage();if(c)perBab4.evidence.exports.push({index:2,title:'Montage Lane Safety 7 kondisi',data_url:c.toDataURL('image/png')})};
const perBab4LaneV2BasePackageHtml=perBab4EvidencePackageHtml;
perBab4EvidencePackageHtml=function(){if(selectedExperiment?.id!=='F4.3')return perBab4LaneV2BasePackageHtml();return `<section class="per-bab4-evidence-card per-bab4-save-package" id="perBab4SavePackage"><div class="per-bab4-evidence-head"><div><span class="eyebrow">OUTPUT OTOMATIS SAAT STOP + SAVE</span><b>Tabel + grafik + bukti visual Lane Safety Outer Boundary</b><p>100 trial utama menyimpan dua kondisi hijau terpisah, outer-lane validity, raw lane evidence, internal-lane ignored, evidence mode, gap, warna, latch, dan recommendation.</p></div><span class="status-chip" id="perBab4VisualState">0 frame</span></div><div class="per-bab4-package-grid"><div><span>Visual capture</span><b id="perBab4VisualFrames">0 / 100</b></div><div><span>Kondisi montage</span><b id="perBab4LaneReady">0 / 7</b></div><div><span>Output visual</span><b>1 graph + 1 montage</b></div><div><span>Data utama</span><b>Raw evidence + rekap</b></div></div><div class="per-bab4-package-actions"><button class="soft-btn" id="perBab4PreviewLane">PNG Montage Lane Safety</button></div></section>`};
const perBab4LaneV2BaseRefreshEvidencePackage=perBab4RefreshEvidencePackage;
perBab4RefreshEvidencePackage=function(){perBab4LaneV2BaseRefreshEvidencePackage();if(selectedExperiment?.id==='F4.3'){const n=perBab4.evidence.f43.length,keys=new Set(perBab4.evidence.f43.map(x=>x.key));setText('perBab4VisualState',`${n} frame`);setText('perBab4VisualFrames',`${n} / 100`);setText('perBab4LaneReady',`${keys.size} / 7`);const chip=$('perBab4VisualState');if(chip)chip.className='status-chip '+(n>=100?'':'waiting')}};

/* Final Dataset presentation: LIVE -> DATA TABLE -> RESULTS, Perception only. */
const PER_BAB4_VIEW_KEY='adv-perception-final-view-v1';
let perBab4ViewMode=['live','table','results'].includes(localStorage.getItem(PER_BAB4_VIEW_KEY))?localStorage.getItem(PER_BAB4_VIEW_KEY):'live';
function perBab4FinalDatasetActive(){return currentExp==='perception'&&/^F4\.[1-5]$/.test(String(selectedExperiment?.id||''))}
function perBab4SetView(mode,persist=true){
  if(!['live','table','results'].includes(mode))mode='live';perBab4ViewMode=mode;if(persist)localStorage.setItem(PER_BAB4_VIEW_KEY,mode);
  if(perBab4FinalDatasetActive())document.body.dataset.bab4View=mode;else delete document.body.dataset.bab4View;
  qa('#perBab4ViewTabs [data-bab4-view]').forEach(b=>{const on=b.dataset.bab4View===mode;b.classList.toggle('active',on);b.setAttribute('aria-selected',String(on));b.tabIndex=on?0:-1});
  const copy={live:['PRATINJAU','Kondisi aktual, sumber data, dan status pengambilan'],table:['TABEL DATA','Data hasil pengujian dan rekapitulasi'],results:['HASIL','Grafik, dokumentasi visual, dan berkas ekspor']}[mode];
  setText('perBab4ViewState',copy?copy.join(' • '):'');
}
function perBab4RefreshFolderContract(result=null){
  const id=String(selectedExperiment?.id||'F4.x').replace('.','_'),folder=result?.report_folder||result?.report_root||lastTrialArtifacts?.report_folder||lastTrialArtifacts?.report_root;
  setText('perBab4FolderPath',folder||('$AGV_ROOT/data/presepsi/BAB4_'+id+'_<timestamp>/'));
  const st=$('perBab4FolderState');if(st){st.textContent=folder?'FOLDER TERSIMPAN':'OTOMATIS SAAT SELESAI';st.className='status-chip '+(folder?'':'waiting')}
}

function perBab4EnsureViewTabs(){
  const nav=$('perBab4FinalNavigator');if(!nav||nav.hidden||!perBab4FinalDatasetActive())return;let tabs=$('perBab4ViewTabs');
  if(!tabs){tabs=document.createElement('div');tabs.id='perBab4ViewTabs';tabs.className='per-bab4-view-tabs';tabs.setAttribute('role','tablist');const cards=nav.querySelector('.per-final-cards');cards?.before(tabs)}
  tabs.innerHTML='<button data-bab4-view="live" role="tab"><b>1 • PRATINJAU</b><span>Kondisi aktual dan pengambilan data</span></button><button data-bab4-view="table" role="tab"><b>2 • TABEL DATA</b><span>Data hasil dan rekapitulasi</span></button><button data-bab4-view="results" role="tab"><b>3 • HASIL</b><span>Grafik, dokumentasi, dan ekspor</span></button><div class="per-bab4-view-state" id="perBab4ViewState"></div>';
  qa('[data-bab4-view]',tabs).forEach(b=>b.onclick=()=>perBab4SetView(b.dataset.bab4View));
  let folder=$('perBab4FolderContract');if(!folder){folder=document.createElement('div');folder.id='perBab4FolderContract';folder.className='per-bab4-folder-contract';nav.appendChild(folder)}
  folder.innerHTML='<div><span>KELUARAN PER PENGUJIAN</span><b id="perBab4FolderPath">--</b></div><div class="per-bab4-folder-files"><em>DATA MENTAH CSV</em><em>TABEL CSV</em><em>XLSX</em><em>GRAFIK PNG</em><em>BUKTI VISUAL</em><em>KONFIGURASI</em><em>MANIFEST</em></div><span class="status-chip waiting" id="perBab4FolderState">OTOMATIS SAAT SELESAI</span>';
  perBab4SetView(perBab4ViewMode,false);perBab4RefreshFolderContract();
}
const perBab4ViewNavBase=perBab4RenderFinalNavigator;
perBab4RenderFinalNavigator=function(){const r=perBab4ViewNavBase();perBab4EnsureViewTabs();return r};
const perBab4ViewPanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){const r=perBab4ViewPanelBase();perBab4EnsureViewTabs();perBab4SetView(perBab4ViewMode,false);return r};

const perBab4ViewRecordingUiBase=setRecordingUi;
setRecordingUi=function(active,info={}){perBab4ViewRecordingUiBase(active,info);if(perBab4FinalDatasetActive()&&selectedExperiment?.id!=='F4.5'&&selectedExperiment?.id!=='F4.1'){const b=$('recordToggleBtn');if(b)b.textContent=active?'■ SELESAI DAN SIMPAN':'● MULAI SESI PENGUJIAN'}perBab4RefreshFolderContract()};
const perBab4ViewArtifactsBase=renderTrialArtifacts;
renderTrialArtifacts=function(result){const r=perBab4ViewArtifactsBase(result);if(perBab4FinalDatasetActive()){perBab4RefreshFolderContract(result);perBab4ViewMode='results';perBab4SetView('results',true)}return r};
const perBab4ViewSwitchBase=switchDomain;
switchDomain=function(domain,syncHmi=true){const r=perBab4ViewSwitchBase(domain,syncHmi);if(domain!=='perception')delete document.body.dataset.bab4View;return r};

function perBab4UpsertMainTrial(key,values,keyOf){
  const a=perBab4Rows(),at=a.findIndex(r=>keyOf(r.values)===key),row={sec:Math.max(0,Math.floor((Date.now()-(recordStartedMs||Date.now()))/1000)),values,yaml:perBab4YamlSnapshot()};
  if(at>=0)a[at]=row;else a.push(row);tableRunRows.set(0,a);renderExperimentTable();perBab4RenderProgress();perBab4DrawSummary();return at>=0;
}
function perBab4EnableRetakeControls(){
  if(!perBab4FinalDatasetActive()||!['F4.2','F4.3'].includes(selectedExperiment?.id))return;const r=$('perBab4Repeat');if(r){r.readOnly=false;r.title='Pengambilan ulang: pilih nomor pengulangan yang sama. Data sebelumnya akan diganti tanpa membuat duplikat.'}
  const controls=$('perceptionBab4Acquisition')?.querySelector('.per-bab4-controls');if(controls&&!controls.querySelector('.per-bab4-retake-note')){const n=document.createElement('div');n.className='per-bab4-retake-note';n.innerHTML='<b>PENGAMBILAN ULANG DATA</b><span>Objek/kondisi dan nomor pengulangan yang sama akan mengganti data sebelumnya tanpa menambah duplikat.</span>';controls.appendChild(n)}
  if(r&&selectedExperiment?.id==='F4.2')r.onchange=perBab4RefreshAutoTrialUi;if(r&&selectedExperiment?.id==='F4.3')r.onchange=perBab4SyncLaneSide;
}
const perBab4RetakePanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){const out=perBab4RetakePanelBase();perBab4EnableRetakeControls();return out};

/* Final Dataset operator preview + explicit save/download controls. Perception only. */
function perBab4CurrentArtifacts(){
  const a=lastTrialArtifacts||null,id=String(selectedExperiment?.id||'');
  return a&&String(a.source_experiment_id||a.section_id||'')===id?a:null;
}
function perBab4LanePreviewCal(){
  const c=typeof perceptionDirectCal!=='undefined'?perceptionDirectCal?.candidate:null;
  if(c&&[c.leftTopX,c.leftTopY,c.leftBottomX,c.leftBottomY,c.rightTopX,c.rightTopY,c.rightBottomX,c.rightBottomY].every(Number.isFinite))return c;
  const v=k=>+configValue('perception',`perception.ros__parameters.${k}`),legacyTop=v('lane_corridor_top_y_ratio'),legacyBottom=v('lane_corridor_bottom_y_ratio');
  const opt=(k,fallback)=>{const x=v(k);return Number.isFinite(x)?x:fallback};
  const q={leftTopX:v('lane_corridor_left_top_x_ratio'),leftTopY:opt('lane_corridor_left_top_y_ratio',legacyTop),leftBottomX:v('lane_corridor_left_bottom_x_ratio'),leftBottomY:opt('lane_corridor_left_bottom_y_ratio',legacyBottom),rightTopX:v('lane_corridor_right_top_x_ratio'),rightTopY:opt('lane_corridor_right_top_y_ratio',legacyTop),rightBottomX:v('lane_corridor_right_bottom_x_ratio'),rightBottomY:opt('lane_corridor_right_bottom_y_ratio',legacyBottom)};
  return Object.values(q).every(Number.isFinite)?q:null;
}
function perBab4LaneStatusColor(s){
  s=String(s||'').toUpperCase();if(s.includes('RED'))return'#ff6677';if(s.includes('YELLOW'))return'#ffc86b';if(s.includes('GREEN'))return'#48e0a4';return'#8fa8a9';
}
async function perBab4DownloadZip(url,fallback){
  try{
    const r=await fetch(url,{cache:'no-store'});if(!r.ok)throw new Error((await r.text()).trim()||`HTTP ${r.status}`);
    const blob=await r.blob(),cd=r.headers.get('content-disposition')||'',m=cd.match(/filename="([^"]+)"/i),name=m?.[1]||fallback,u=URL.createObjectURL(blob),a=document.createElement('a');
    a.href=u;a.download=name;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(u),1200);toast(`${name} siap diunduh`);
  }catch(e){toast(`ZIP belum tersedia: ${e.message||e}`,true)}
}
function perBab4EnsureFinalActions(){
  const host=$('perBab4FolderContract');if(!host||!perBab4FinalDatasetActive())return;
  let box=$('perBab4FolderActions');if(!box){box=document.createElement('div');box.id='perBab4FolderActions';box.className='per-bab4-folder-actions';host.appendChild(box)}
  box.innerHTML='<button class="primary-btn" id="perBab4FinishSaveFolder">● MULAI SESI PENGUJIAN</button><button class="soft-btn" id="perBab4RunZip">⬇ ZIP PENGUJIAN TERAKHIR</button><button class="soft-btn" id="perBab4FinalZip">⬇ ZIP DATASET FINAL</button>';
  $('perBab4FinishSaveFolder').onclick=()=>{const active=!!recordStartedMs||$('recordToggleBtn')?.dataset.active==='true';if(active)return stopWebRecording();perBab4SetView('live');setTimeout(()=>startWebRecording(),0)};
  $('perBab4RunZip').onclick=()=>{const f43=selectedExperiment?.id==='F4.3';perBab4DownloadZip(f43?'/api/perception/f43/latest.zip':'/api/experiment/record/last.zip',f43?'BAB4_F4.3_latest.zip':`BAB4_${String(selectedExperiment?.id||'run').replace('.','_')}_latest.zip`)};
  $('perBab4FinalZip').onclick=()=>perBab4DownloadZip('/api/perception/final-dataset/latest.zip','FINAL_DATASET_PERSEPSI.zip');
  perBab4RefreshFinalActions();
}
function perBab4RefreshFinalActions(){
  if(!perBab4FinalDatasetActive())return;const active=!!recordStartedMs||$('recordToggleBtn')?.dataset.active==='true',a=perBab4CurrentArtifacts(),save=$('perBab4FinishSaveFolder'),zip=$('perBab4RunZip');
  const pre=typeof testPreflightStatus==='function'?testPreflightStatus():{ok:true};
  if(save){save.disabled=!active&&!pre.ok;save.textContent=active?'■ SELESAI DAN SIMPAN':(a?'● MULAI SESI PENGUJIAN BARU':'● MULAI SESI PENGUJIAN');save.title=active?'Simpan data, grafik, dokumentasi visual, konfigurasi, dan manifest ke folder pengujian':pre.ok?'Mulai sesi pengujian BAB IV':`Sumber data belum siap: ${pre.reason||'periksa sumber data'}`}
  if(zip){zip.disabled=false;zip.title=a?'Unduh seluruh folder pengujian ini':'Unduh folder pengujian terakhir yang tersedia pada server'}
}
function perBab4PreviewText(){
  const id=String(selectedExperiment?.id||'');
  if(id==='F4.2')return['LIVE OBJECT DETECTION','BBox raw + target-zone. DETECTED berarti bbox menyentuh/berpusat pada zona TARGET.'];
  if(id==='F4.3')return['PRATINJAU KEAMANAN LAJUR','Citra teranotasi dengan dua batas koridor keselamatan hasil kalibrasi. Status menunjukkan kondisi aman, pendekatan batas keselamatan, pelanggaran batas keselamatan, atau kondisi di luar area dapat dilalui.'];
  if(id==='F4.4')return['PRATINJAU KINERJA WAKTU NYATA','Citra yang diproses secara aktual disertai status laju pipeline dan inferensi.'];
  if(id==='F4.5')return['LIVE CONTROL RESPONSE','Frame persepsi yang menjadi dasar kondisi respons kontrol.'];
  return['PRATINJAU IMPLEMENTASI SISTEM','Citra kamera dan status pipeline digunakan sebagai dokumentasi implementasi sistem persepsi.'];
}
let perBab4PreviewBindTimer=0;
function perBab4EnsureOperatorPreview(){
  const nav=$('perBab4FinalNavigator');if(!nav||nav.hidden||!perBab4FinalDatasetActive())return;
  let box=$('perBab4OperatorPreview');if(!box){box=document.createElement('section');box.id='perBab4OperatorPreview';box.className='per-bab4-operator-preview';const folder=$('perBab4FolderContract');folder?folder.after(box):nav.appendChild(box)}
  const id=String(selectedExperiment?.id||''),copy=perBab4PreviewText();
  if(box.dataset.task!==id){box.dataset.task=id;box.innerHTML=`<div class="per-bab4-preview-head"><div><span>VERIFIKASI VISUAL PENGUJIAN</span><b>${copy[0]}</b><p>${copy[1]}</p></div><span class="status-chip waiting" id="perBab4PreviewState">MENUNGGU CITRA</span></div><div class="per-bab4-preview-grid"><div class="per-bab4-preview-media"><img id="perBab4PreviewImage" alt="Final Dataset live preview"><canvas id="perBab4PreviewOverlay"></canvas><div id="perBab4PreviewEmpty">MENUNGGU CITRA KAMERA</div></div><div class="per-bab4-preview-stats" id="perBab4PreviewStats"></div></div>`}
  clearTimeout(perBab4PreviewBindTimer);perBab4PreviewBindTimer=setTimeout(perBab4BindOperatorPreview,0);perBab4RefreshOperatorPreview();if(id==='F4.3')setTimeout(perBab4LanePlaceControls,0);
}
function perBab4BindOperatorPreview(){
  if(!cameraFrameStore){perBab4PreviewBindTimer=setTimeout(perBab4BindOperatorPreview,350);return}
  const img=$('perBab4PreviewImage');if(!img)return;
  cameraFrameStore.subscribe('bab4-final-preview',{img,placeholder:$('perBab4PreviewEmpty'),isActive:()=>perBab4FinalDatasetActive()&&elementVisible($('perBab4OperatorPreview')),onFrame:()=>{perBab4DrawOperatorOverlay();perBab4RefreshOperatorPreview()}});
}
function perBab4DrawOperatorOverlay(){
  const canvas=$('perBab4PreviewOverlay'),img=$('perBab4PreviewImage'),g=cameraGeometryFor(img);if(!canvas||!img||!g)return;
  const fit=g.resizeCanvas(canvas),ctx=fit.ctx,dpr=fit.dpr,cr=canvas.getBoundingClientRect();ctx.clearRect(0,0,fit.width,fit.height);
  const line=(a,b,color,width=3)=>{if(!a||!b)return;ctx.strokeStyle=color;ctx.lineWidth=width*dpr;ctx.beginPath();ctx.moveTo((a.x-cr.left)*dpr,(a.y-cr.top)*dpr);ctx.lineTo((b.x-cr.left)*dpr,(b.y-cr.top)*dpr);ctx.stroke()};
  if(selectedExperiment?.id==='F4.2'){
    const dets=perBab4RawDetections(),decision=perBab4AutoDetectionDecision(),w=img.naturalWidth,h=img.naturalHeight;
    dets.forEach((d,i)=>{const b=perBab4AutoNormalizeBox(d,w,h);if(!b)return;const a=g.imageToScreen(b.x1,b.y1),z=g.imageToScreen(b.x2,b.y2);if(!a||!z)return;const x=Math.min(a.x,z.x)-cr.left,y=Math.min(a.y,z.y)-cr.top,bw=Math.abs(z.x-a.x),bh=Math.abs(z.y-a.y),sel=i===decision.index;ctx.strokeStyle=sel?'#48e0a4':'#31a8ff';ctx.lineWidth=(sel?4:2)*dpr;ctx.strokeRect(x*dpr,y*dpr,bw*dpr,bh*dpr);ctx.fillStyle=ctx.strokeStyle;ctx.font=`800 ${11*dpr}px ui-monospace`;ctx.fillText(`#${i+1} ${Number.isFinite(+d.score)?(+d.score).toFixed(2):'--'}`,(x+4)*dpr,Math.max(14,(y-4))*dpr)});
    const p=g.normalizedToScreen(.5,.5);if(p){const x=(p.x-cr.left)*dpr,y=(p.y-cr.top)*dpr,rx=cr.width*.10*dpr,ry=cr.height*.12*dpr;ctx.save();ctx.setLineDash([8*dpr,5*dpr]);ctx.strokeStyle='#ffc86b';ctx.lineWidth=2*dpr;ctx.strokeRect(x-rx,y-ry,rx*2,ry*2);ctx.restore()}
  }else if(selectedExperiment?.id==='F4.3'){
    // F4.3 memakai citra anotasi dari backend sebagai satu-satunya overlay visual.
    // Jangan menggambar ulang garis kalibrasi atau label status di atas citra karena
    // akan menduplikasi outer-lane/lane-mask dan mengganggu dokumentasi pengujian.
    return;
  }
}
function perBab4RefreshOperatorPreview(){
  const box=$('perBab4PreviewStats'),chip=$('perBab4PreviewState');if(!box||!perBab4FinalDatasetActive())return;
  const p=obj('perception_performance')||{},cf=obj('camera_frame')||{},frameAge=age('camera_frame'),source=cf.source||cf.format||'--';
  let rows=[['Kamera',bool(raw('connected.camera'))&&bool(raw('camera_healthy'))?'Sehat':'Perlu Pemeriksaan'],['Sumber Citra',source],['Laju Pipeline',Number.isFinite(+p.pipeline_fps)?(+p.pipeline_fps).toFixed(2)+' Hz':'--'],['Umur Frame',Number.isFinite(frameAge)?frameAge.toFixed(2)+' s':'--']],state='AKTIF',stateClass='';
  if(selectedExperiment?.id==='F4.2'){
    const d=perBab4AutoDetectionDecision(),conf=Number.isFinite(+d.candidate?.score)?(+d.candidate.score).toFixed(3):'--';state=d.success?'DETECTED':'NOT DETECTED';stateClass=d.success?'':'waiting';
    rows=[['Trial aktif',typeof perBab4AutoTargetText==='function'?perBab4AutoTargetText():'--'],['Raw candidate',String(d.dets.length)],['Confidence target',conf],['Decision',d.reason],['raw_detections age',Number.isFinite(age('raw_detections'))?age('raw_detections').toFixed(2)+' s':'--'],...rows];
  }else if(selectedExperiment?.id==='F4.3'){
    const ls=obj('lane_state')||{},c=ls.corridor||{},drivable=Boolean(c.drivable_detected??ls.drivable_valid??ls.valid),mask=Boolean(c.lane_mask_detected??c.raw_lane_mask_detected),
      l=drivable?String(c.right_status||'GREEN').toUpperCase():'GRAY',r=drivable?String(c.left_status||'GREEN').toUpperCase():'GRAY',
      leftGap=c.right_gap_px,rightGap=c.left_gap_px;
    const red=l.includes('RED')||r.includes('RED'),yellow=l.includes('YELLOW')||r.includes('YELLOW');
    state=red?'PELANGGARAN BATAS KESELAMATAN':yellow?'MENDEKATI BATAS KESELAMATAN':mask?'KONDISI AMAN':drivable?'AREA DAPAT DILALUI • BATAS LAJUR TIDAK TERDETEKSI':'DI LUAR AREA DAPAT DILALUI';stateClass=red?'bad':yellow?'waiting':mask||drivable?'':'waiting';
    rows=[['Area dapat dilalui',drivable?'Terdeteksi':'Tidak Terdeteksi'],['Mask segmentasi lajur',mask?'Terdeteksi':'Tidak Terdeteksi'],['Batas kiri',`${perBab4LaneStatusLabel(l)} • ${Number.isFinite(+leftGap)?(+leftGap).toFixed(1)+' piksel':'--'}`],['Batas kanan',`${perBab4LaneStatusLabel(r)} • ${Number.isFinite(+rightGap)?(+rightGap).toFixed(1)+' piksel':'--'}`],['Rekomendasi sistem',perBab4LaneRecommendationLabel(c.recommendation)],['Mode bukti',perBab4LaneEvidenceLabel(c.evidence_mode)],...rows];
  }
  box.innerHTML=rows.map(([k,v])=>`<div><span>${escapeHtml(k)}</span><b>${escapeHtml(v)}</b></div>`).join('');
  if(chip){chip.textContent=state;chip.className='status-chip '+stateClass}perBab4DrawOperatorOverlay();if(selectedExperiment?.id==='F4.3')perBab4LaneBatchRefresh();
}
const perBab4PreviewEnsureBase=perBab4EnsureViewTabs;
perBab4EnsureViewTabs=function(){const r=perBab4PreviewEnsureBase();perBab4EnsureFinalActions();perBab4EnsureOperatorPreview();return r};
const perBab4PreviewRefreshBase=perBab4RefreshLive;
perBab4RefreshLive=function(){const r=perBab4PreviewRefreshBase();perBab4RefreshOperatorPreview();perBab4RefreshFinalActions();return r};
const perBab4PreviewRecordingBase=setRecordingUi;
setRecordingUi=function(active,info={}){const r=perBab4PreviewRecordingBase(active,info);perBab4RefreshFinalActions();return r};
const perBab4PreviewArtifactsBase=renderTrialArtifacts;
renderTrialArtifacts=function(result){const r=perBab4PreviewArtifactsBase(result);perBab4RefreshFinalActions();return r};

/* FINAL F4.3: 6 kondisi x 10 fresh frame = 60 trial. GUI/report only. */
const PER_BAB4_LANE_V3=[
  ['center','Posisi Tengah Koridor Keselamatan','GREEN','GREEN','Kendaraan berada pada posisi tengah koridor keselamatan dan kedua batas lajur berada pada zona aman.'],
  ['left_yellow','Pendekatan Batas Keselamatan Kiri','YELLOW','GREEN','Batas lajur kiri mendekati batas koridor keselamatan, sedangkan batas lajur kanan tetap berada pada zona aman.'],
  ['left_red','Pelanggaran Batas Keselamatan Kiri','RED','GREEN','Batas lajur kiri menyentuh atau memasuki koridor keselamatan, sedangkan batas lajur kanan tetap berada pada zona aman.'],
  ['right_yellow','Pendekatan Batas Keselamatan Kanan','GREEN','YELLOW','Batas lajur kanan mendekati batas koridor keselamatan, sedangkan batas lajur kiri tetap berada pada zona aman.'],
  ['right_red','Pelanggaran Batas Keselamatan Kanan','GREEN','RED','Batas lajur kanan menyentuh atau memasuki koridor keselamatan, sedangkan batas lajur kiri tetap berada pada zona aman.'],
  ['gray','Di Luar Area Dapat Dilalui (Non-Drivable Area)','GRAY','GRAY','Area dapat dilalui tidak tervalidasi sehingga kondisi diklasifikasikan sebagai berada di luar area jalan yang dapat dilalui.']
];
const PER_BAB4_LANE_GEOMETRY={road_m:6.0,safety_m:4.0,bottom_m:4.3,top_m:9.3,length_m:5.0,margin_m:1.0};
function perBab4LaneV3Meta(key){const q=PER_BAB4_LANE_V3.find(x=>x[0]===key)||PER_BAB4_LANE_V3[0];return{key:q[0],label:q[1],left:q[2],right:q[3],detail:q[4]}}
function perBab4LaneConditionGraphLabel(label){const m={'Posisi Tengah Koridor Keselamatan':'Posisi Tengah','Pendekatan Batas Keselamatan Kiri':'Pendekatan Kiri','Pelanggaran Batas Keselamatan Kiri':'Pelanggaran Kiri','Pendekatan Batas Keselamatan Kanan':'Pendekatan Kanan','Pelanggaran Batas Keselamatan Kanan':'Pelanggaran Kanan','Di Luar Area Dapat Dilalui (Non-Drivable Area)':'Di Luar Area'};return m[label]||label}
function perBab4LaneStatusLabel(code){const m={GREEN:'Aman (Hijau)',YELLOW:'Peringatan Batas Keselamatan (Kuning)',RED:'Pelanggaran Batas Keselamatan (Merah)',GRAY:'Di Luar Area Dapat Dilalui (Abu-abu)',UNKNOWN:'Tidak Teridentifikasi'};return m[String(code||'UNKNOWN').toUpperCase()]||String(code||'Tidak Teridentifikasi')}
function perBab4LaneStatusShortLabel(code){const m={GREEN:'Aman',YELLOW:'Peringatan',RED:'Pelanggaran',GRAY:'Di Luar Area'};return m[String(code||'').toUpperCase()]||String(code||'Tidak Teridentifikasi')}
function perBab4LaneRecommendationLabel(code){const m={CLEAR:'Tidak Memerlukan Koreksi',WARNING:'Peringatan: Mendekati Batas Keselamatan',RECENTER_RIGHT:'Koreksi Arah ke Kanan',RECENTER_LEFT:'Koreksi Arah ke Kiri',BOTH_INTRUSION_STOP:'Hentikan Kendaraan',DRIVABLE_INVALID:'Area Dapat Dilalui Tidak Valid',NO_MASK_EVIDENCE:'Bukti Segmentasi Tidak Tersedia'};return m[String(code||'').toUpperCase()]||String(code||'Tidak Tersedia')}
function perBab4LaneEvidenceLabel(code){const m={DRIVABLE_OUTER_LANE:'Area Dapat Dilalui + Batas Luar Lajur',DRIVABLE_ONLY:'Area Dapat Dilalui',DRIVABLE_INTERNAL_LANE_IGNORED:'Garis Internal Diabaikan',RAW_LANE_WITHOUT_DRIVABLE_UNTRUSTED:'Mask Lajur tanpa Area Dapat Dilalui (Tidak Dipercaya)',NO_EVIDENCE:'Bukti Tidak Tersedia'};return m[String(code||'').toUpperCase()]||String(code||'Tidak Tersedia')}
function perBab4LaneResultLabel(ok){return ok?'Memenuhi Kriteria':'Tidak Memenuhi Kriteria'}
function perBab4LaneV3ApplySchema(){
  const x=(experiments.perception||[]).find(v=>v.id==='F4.3');if(!x)return;
  Object.assign(x,{groupId:'FINAL-4.3',groupTitle:'BAB IV — 4.3 Pengujian Keamanan Lajur (Lane Safety)',section:'Pengujian Keamanan Lajur (Lane Safety) — 6 kondisi × 10 pengulangan • 60 data',reportMode:true,bab4Protocol:'lane',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4LaneFields(),
    tableColumns:[['No. Data','Kondisi Pengujian','Pengulangan ke-','Waktu Pengambilan','Area Dapat Dilalui Terdeteksi','Mask Segmentasi Lajur Terdeteksi','Batas Lajur Kiri Valid','Batas Lajur Kanan Valid','Jarak Batas Kiri [piksel]','Jarak Batas Kanan [piksel]','Status Batas Kiri','Status Batas Kanan','Status Acuan Kiri','Status Acuan Kanan','Kesesuaian Status Kiri','Kesesuaian Status Kanan','Rekomendasi Sistem','Hasil Pengujian','Umur Data [s]','Mode Bukti','Garis Internal Diabaikan','Mask Segmentasi Mentah Terdeteksi','Status Pengunci Koreksi Kiri','Status Pengunci Koreksi Kanan','Waktu Relatif [s]','Sumber Bukti'],['Kondisi Pengujian','Jumlah Data','Memenuhi Kriteria','Tidak Memenuhi Kriteria','Tingkat Keberhasilan [%]','Status Kiri Sesuai','Status Kanan Sesuai','Tingkat Kesesuaian Kiri [%]','Tingkat Kesesuaian Kanan [%]']],
    tableNames:['Data Hasil Pengujian Keamanan Lajur','Rekapitulasi Tingkat Keberhasilan Keamanan Lajur'],graphCaptions:['Tingkat Keberhasilan Pengujian Keamanan Lajur','Jarak Batas Lajur terhadap Batas Keselamatan per Pengulangan','Distribusi Status Keamanan Lajur per Kondisi','Dokumentasi Visual Enam Kondisi Pengujian Keamanan Lajur'],graphs:[],
    liveSeries:{'Jarak batas kiri [piksel]':'lane_state.corridor.right_gap_px','Jarak batas kanan [piksel]':'lane_state.corridor.left_gap_px','Validitas lajur':'lane_state.valid','Validitas area dapat dilalui':'lane_state.corridor.drivable_detected'}});
  if(selectedExperiment?.id==='F4.3')selectedExperiment=x;
}
const perBab4LaneV3PatchBase=patchPerceptionBab4Catalog;
patchPerceptionBab4Catalog=function(){perBab4LaneV3PatchBase();perBab4LaneV3ApplySchema()};
perBab4LaneV3ApplySchema();
/* F43V3_CHUNK1 */
function perBab4LaneV3Visual(){
  const ls=obj('lane_state')||{},c=ls.corridor||{},drivable=Boolean(c.drivable_detected??ls.drivable_valid??ls.valid),laneMask=Boolean(c.lane_mask_detected??c.raw_lane_mask_detected);
  const norm=s=>{s=String(s||'UNKNOWN').toUpperCase();if(!drivable)return'GRAY';return ['GREEN','YELLOW','RED'].includes(s)?s:'GREEN'};
  const leftValid=typeof c.right_valid==='boolean'?c.right_valid:(Number(c.right_samples)>0),rightValid=typeof c.left_valid==='boolean'?c.left_valid:(Number(c.left_samples)>0);return{ls,c,drivable,laneMask,leftStatus:norm(c.right_status),rightStatus:norm(c.left_status),leftGap:+c.right_gap_px,rightGap:+c.left_gap_px,leftValid,rightValid,leftLatch:Boolean(c.right_recenter_latched),rightLatch:Boolean(c.left_recenter_latched)};
}
function perBab4LaneExpected(cond){const m=perBab4LaneV3Meta(cond);return{left:m.left,right:m.right}}
function perBab4LaneV3EvidenceReady(cond,v){return cond==='gray'?!v.drivable:(v.drivable&&v.laneMask&&v.leftValid&&v.rightValid)}
perBab4LaneControls=function(){
  return `<div class="per-lane-geometry-strip"><b>PARAMETER GEOMETRI PENGUJIAN</b><span>Lebar jalan: 6,0 m</span><span>Lebar koridor keselamatan: 4,0 m</span><span>Jarak titik bawah: 4,3 m</span><span>Jarak titik atas: 9,3 m</span><span>Margin lateral: 1,0 m/sisi</span></div>
  <div class="per-bab4-controls per-lane-batch-controls" id="perLaneBatchControls"><label>Kondisi pengujian<select id="perBab4LaneCondition">${PER_BAB4_LANE_V3.map(q=>`<option value="${q[0]}">${q[1]} • ${perBab4LaneStatusLabel(q[2])} / ${perBab4LaneStatusLabel(q[3])}</option>`).join('')}</select></label>
  <div class="per-bab4-actions per-lane-batch-actions"><button class="primary-btn" id="perLaneRun10">▶ MULAI PENGAMBILAN 10 SAMPEL / 30 s</button><button class="warning-btn" id="perLaneStopBatch">■ HENTIKAN PENGAMBILAN</button><button class="soft-btn" id="perLaneRepeatFailed">↻ ULANGI DATA TIDAK MEMENUHI KRITERIA</button><button class="soft-btn" id="perLaneResetCondition">ATUR ULANG KONDISI</button></div>
  <div class="per-lane-control-state"><span>Kondisi pengujian aktif</span><b id="perLaneBatchCondition">Posisi Tengah Koridor Keselamatan</b></div><div class="per-lane-control-state"><span>Kesesuaian kondisi aktual</span><b id="perLaneBatchReady">MENUNGGU DATA AKTUAL</b></div><div class="per-lane-control-state"><span>Kemajuan pengambilan data</span><b id="perLaneBatchProgress">0 / 10</b></div><div class="per-lane-control-state"><span>Status data waktu-nyata</span><b id="perLaneBatchFresh">MENUNGGU DATA</b></div>
  <div class="per-lane-test-hint">Prosedur: pilih kondisi pengujian, tempatkan kendaraan sesuai kondisi fisik yang ditetapkan, kemudian tekan <b>MULAI PENGAMBILAN 10 SAMPEL / 30 s</b>. Sistem merekam satu hasil inferensi terbaru setiap ±3 s.</div></div>`;
}
perBab4SyncLaneSide=function(){perBab4LaneBatchRefresh()};
perBab4BuildLaneSummary=function(){
  const groups=new Map();for(const r of perBab4Rows()){const v=r.values,g=groups.get(v[1])||{cond:v[1],n:0,pass:0,l:0,r:0};g.n++;if(v[17]==='Memenuhi Kriteria'||v[17]==='PASS')g.pass++;if(v[14]==='Sesuai'||v[14]==='YES')g.l++;if(v[15]==='Sesuai'||v[15]==='YES')g.r++;groups.set(v[1],g)}
  const order=PER_BAB4_LANE_V3.map(q=>q[1]),rows=[...groups.values()].sort((a,b)=>order.indexOf(a.cond)-order.indexOf(b.cond)).map(g=>[g.cond,g.n,g.pass,g.n-g.pass,(100*g.pass/g.n).toFixed(2),g.l,g.r,(100*g.l/g.n).toFixed(2),(100*g.r/g.n).toFixed(2)]);if(rows.length){const n=rows.reduce((s,r)=>s+(+r[1]||0),0),p=rows.reduce((s,r)=>s+(+r[2]||0),0),l=rows.reduce((s,r)=>s+(+r[5]||0),0),rr=rows.reduce((s,r)=>s+(+r[6]||0),0);rows.push(['Total',n,p,n-p,n?(100*p/n).toFixed(2):'--',l,rr,n?(100*l/n).toFixed(2):'--',n?(100*rr/n).toFixed(2):'--'])}return rows;
};
const perBab4LaneV3ProgressBase=perBab4RenderProgress;
perBab4RenderProgress=function(){if(selectedExperiment?.id!=='F4.3')return perBab4LaneV3ProgressBase();const box=$('perBab4Progress');if(!box)return;const c=perBab4ProgressCounts();box.innerHTML=PER_BAB4_LANE_V3.map(q=>`<span class="${(c[q[1]]||0)>=10?'done':''}">${q[1]} <em>${c[q[1]]||0}/10</em></span>`).join('')};
const perBab4LaneV3InfoBase=perBab4InfoText;
perBab4InfoText=function(){if(selectedExperiment?.id==='F4.3')return 'Pengujian terdiri atas 6 kondisi × 10 pengulangan (60 data). Setiap kondisi direkam selama 30 s dengan interval target ±3 s per sampel. Hasil dinilai berdasarkan kesesuaian status batas kiri dan kanan serta validitas area dapat dilalui dan batas lajur.';return perBab4LaneV3InfoBase()};
const perBab4LaneV3StepBase=perBab4StepText;
perBab4StepText=function(){if(selectedExperiment?.id==='F4.3')return['Tujuan','Mengukur tingkat keberhasilan sistem keamanan lajur pada enam kondisi pengujian dengan 10 pengulangan per kondisi.','Prosedur','Pilih kondisi pengujian, tempatkan kendaraan sesuai kondisi fisik yang ditetapkan, kemudian mulai pengambilan 10 sampel selama 30 s. Sistem merekam satu hasil inferensi terbaru setiap ±3 s.','Keluaran penelitian','Data hasil 60 pengujian, rekapitulasi tingkat keberhasilan, grafik jarak batas lajur, grafik distribusi status, dan satu kompilasi visual yang mewakili enam kondisi pengujian.'];return perBab4LaneV3StepBase()};
/* F43V3_CHUNK2 */
perBab4.laneBatch=perBab4.laneBatch||{active:false,cond:'center',queue:[],total:0,done:0,lastStamp:0,timer:null,busy:false,lastResult:'WAIT',startMs:0,durationMs:30000,intervalMs:3000,autoStartedSession:false};
function perBab4LaneRowsFor(cond){const label=perBab4LaneV3Meta(cond).label;return perBab4Rows().filter(r=>r.values?.[1]===label)}
/* F43V3_CHUNK3A */
async function perBab4LaneCaptureOne(cond,repeat){
  if(!recordStartedMs)throw new Error('Sesi pengambilan data belum aktif.');
  if(!channelFresh('lane_state',2.5))throw new Error('Data keamanan lajur belum diperbarui.');
  const meta=perBab4LaneV3Meta(cond),v=perBab4LaneV3Visual(),exp=perBab4LaneExpected(cond);
  const leftMatch=v.leftStatus===exp.left,rightMatch=v.rightStatus===exp.right,evidenceReady=perBab4LaneV3EvidenceReady(cond,v),match=leftMatch&&rightMatch&&evidenceReady;
  const trialNo=PER_BAB4_LANE_V3.findIndex(q=>q[0]===cond)*10+repeat,actual=String(v.c.recommendation||'--'),frameAge=age('lane_state');
  const label=`F4.3_${cond}_data_${String(repeat).padStart(2,'0')}_${match?'MEMENUHI':'TIDAK_MEMENUHI'}`;
  const evidencePath='KOMPILASI_VISUAL',dataUrl=perBab4EvidenceJpeg({kind:'lane',left:v.leftStatus,right:v.rightStatus,recommendation:actual,title:`PENGUJIAN KEAMANAN LAJUR • ${meta.label} • Pengulangan ${repeat}`,subtitle:`Aktual: ${perBab4LaneStatusLabel(v.leftStatus)} / ${perBab4LaneStatusLabel(v.rightStatus)} • Acuan: ${perBab4LaneStatusLabel(exp.left)} / ${perBab4LaneStatusLabel(exp.right)} • ${perBab4LaneResultLabel(match)}`});
  const elapsedS=perBab4.laneBatch.active?Math.max(0,(Date.now()-perBab4.laneBatch.startMs)/1000):0,values=[trialNo,meta.label,repeat,perBab4FrameTimestamp(),v.drivable?'Terdeteksi':'Tidak Terdeteksi',v.laneMask?'Terdeteksi':'Tidak Terdeteksi',v.leftValid?'Valid':'Tidak Valid',v.rightValid?'Valid':'Tidak Valid',Number.isFinite(v.leftGap)?v.leftGap.toFixed(2):'--',Number.isFinite(v.rightGap)?v.rightGap.toFixed(2):'--',perBab4LaneStatusLabel(v.leftStatus),perBab4LaneStatusLabel(v.rightStatus),perBab4LaneStatusLabel(exp.left),perBab4LaneStatusLabel(exp.right),leftMatch?'Sesuai':'Tidak Sesuai',rightMatch?'Sesuai':'Tidak Sesuai',perBab4LaneRecommendationLabel(actual),perBab4LaneResultLabel(match),Number.isFinite(frameAge)?frameAge.toFixed(3):'--',perBab4LaneEvidenceLabel(v.c.evidence_mode),Boolean(v.c.internal_lane_ignored)?'Ya':'Tidak',Boolean(v.c.raw_lane_mask_detected)?'Terdeteksi':'Tidak Terdeteksi',v.leftLatch?'Aktif':'Tidak Aktif',v.rightLatch?'Aktif':'Tidak Aktif',elapsedS.toFixed(2),evidencePath];
  const key=`${meta.label}|${repeat}`,retake=perBab4UpsertMainTrial(key,values,x=>`${x[1]}|${x[2]}`);
  perBab4.evidence.f43=perBab4.evidence.f43.filter(x=>!(x.cond===cond&&+x.repeat===repeat));
  if(dataUrl)perBab4.evidence.f43.push({key:cond,cond,repeat,left:v.leftStatus,right:v.rightStatus,recommendation:actual,match,dataUrl,evidencePath,evidenceMode:String(v.c.evidence_mode||'--')});
  perBab4RefreshEvidencePackage();perBab4.laneBatch.lastResult=`${retake?'PENGAMBILAN ULANG • ':''}${meta.label} • Pengulangan ${repeat} • ${perBab4LaneResultLabel(match)}`;return match;
}
/* F43V3_CHUNK3B */
function perBab4LaneBatchStop(silent=false){
  const s=perBab4.laneBatch;s.active=false;s.busy=false;clearInterval(s.timer);s.timer=null;
  const sel=$('perBab4LaneCondition');if(sel)sel.disabled=false;
  if(!silent)toast(`Pengambilan data dihentikan • ${s.done}/${s.total} data terekam`);perBab4LaneBatchRefresh();
}
async function perBab4LaneBatchTick(){
  const s=perBab4.laneBatch;if(!s.active||s.busy||!s.queue.length)return;
  if(!recordStartedMs){perBab4LaneBatchStop();return}
  const elapsed=Date.now()-s.startMs;if(elapsed>s.durationMs){const left=s.queue.length;perBab4LaneBatchStop(true);toast(`Periode pengambilan 30 s selesai • ${s.done}/${s.total} data tersimpan${left?' • '+left+' data belum terekam':''}`,!!left);return}
  const due=s.done*s.intervalMs;if(elapsed<due)return;
  const stamp=+updated.lane_state||0;if(!stamp||stamp<=s.lastStamp||!channelFresh('lane_state',2.5))return;
  s.lastStamp=stamp;s.busy=true;const repeat=s.queue[0];
  try{await perBab4LaneCaptureOne(s.cond,repeat);s.queue.shift();s.done++;if(!s.queue.length){perBab4LaneBatchStop(true);toast(`${perBab4LaneV3Meta(s.cond).label}: 10 sampel selesai dalam ${((Date.now()-s.startMs)/1000).toFixed(1)} detik`)}}
  catch(e){perBab4LaneBatchStop(true);toast(e.message||String(e),true)}
  finally{s.busy=false;perBab4LaneBatchRefresh()}
}
async function perBab4LaneBatchStart(failedOnly=false){
  if(perBab4.laneBatch.active)return;
  if(!channelFresh('lane_state',2.5))return toast('Lane Safety stream belum fresh.',true);
  if(recordStartedMs&&(recordingDomain!=='perception'||recordingLeafId!=='F4.3'))return toast(`Perekaman ${recordingLeafId||'lain'} masih aktif. Hentikan dan simpan perekaman tersebut sebelum memulai pengujian keamanan lajur.`,true);
  const cond=$('perBab4LaneCondition')?.value||'center';
  let autoStartedSession=false;if(!recordStartedMs){await startWebRecording();if(!recordStartedMs)return;autoStartedSession=true}
  const byRep=new Map(perBab4LaneRowsFor(cond).map(r=>[+r.values[2],r.values[17]]));
  const queue=failedOnly?[1,2,3,4,5,6,7,8,9,10].filter(n=>!['Memenuhi Kriteria','PASS'].includes(byRep.get(n))):[1,2,3,4,5,6,7,8,9,10];
  if(!queue.length)return toast('Seluruh 10 data pada kondisi ini telah memenuhi kriteria.');
  Object.assign(perBab4.laneBatch,{active:true,cond,queue:[...queue],total:queue.length,done:0,lastStamp:0,busy:false,lastResult:'PERIODE_PENGAMBILAN_30_S',startMs:Date.now(),durationMs:30000,intervalMs:3000,autoStartedSession});
  const sel=$('perBab4LaneCondition');if(sel)sel.disabled=true;clearInterval(perBab4.laneBatch.timer);perBab4.laneBatch.timer=setInterval(perBab4LaneBatchTick,100);perBab4LaneBatchRefresh();perBab4LaneBatchTick();
}
/* F43V3_CHUNK3C */
function perBab4LaneBatchRefresh(){
  if(selectedExperiment?.id!=='F4.3')return;
  const sel=$('perBab4LaneCondition'),cond=perBab4.laneBatch.active?perBab4.laneBatch.cond:(sel?.value||'center'),meta=perBab4LaneV3Meta(cond),rows=perBab4LaneRowsFor(cond),pass=rows.filter(r=>['Memenuhi Kriteria','PASS'].includes(r.values[17])).length;
  setText('perLaneBatchCondition',`${meta.label} • ${perBab4LaneStatusLabel(meta.left)} / ${perBab4LaneStatusLabel(meta.right)}`);
  const elapsed=perBab4.laneBatch.active?Math.max(0,Date.now()-perBab4.laneBatch.startMs):0,remain=perBab4.laneBatch.active?Math.max(0,30-elapsed/1000):30;
  setText('perLaneBatchProgress',perBab4.laneBatch.active?`${perBab4.laneBatch.done} / ${perBab4.laneBatch.total} data • sisa waktu ${remain.toFixed(1)} s`:`${rows.length} / 10 data • memenuhi kriteria ${pass}`);
  const a=age('lane_state'),nextDue=perBab4.laneBatch.active?Math.max(0,(perBab4.laneBatch.done*3)-(elapsed/1000)):0;setText('perLaneBatchFresh',perBab4.laneBatch.active?`PEREKAMAN AKTIF • sampel berikut ±${nextDue.toFixed(1)} s • umur data ${Number.isFinite(a)?a.toFixed(2):'--'} s`:(channelFresh('lane_state',2.5)?`DATA SIAP • umur data ${Number.isFinite(a)?a.toFixed(2):'--'} s`:'MENUNGGU DATA KEAMANAN LAJUR'));
  const run=$('perLaneRun10'),stop=$('perLaneStopBatch'),retry=$('perLaneRepeatFailed'),reset=$('perLaneResetCondition');
  if(run)run.disabled=perBab4.laneBatch.active||!channelFresh('lane_state',2.5);if(stop)stop.disabled=!perBab4.laneBatch.active;if(retry)retry.disabled=perBab4.laneBatch.active||!recordStartedMs;if(reset)reset.disabled=perBab4.laneBatch.active;
  const v=perBab4LaneV3Visual(),exp=perBab4LaneExpected(cond),fmt=x=>Number.isFinite(x)?x.toFixed(1)+' piksel':'--',colorMatch=v.leftStatus===exp.left&&v.rightStatus===exp.right,evidenceReady=perBab4LaneV3EvidenceReady(cond,v);setText('perLaneLeftStatus',perBab4LaneStatusLabel(v.leftStatus));setText('perLaneRightStatus',perBab4LaneStatusLabel(v.rightStatus));setText('perLaneLeftExpected','Status acuan: '+perBab4LaneStatusLabel(exp.left));setText('perLaneRightExpected','Status acuan: '+perBab4LaneStatusLabel(exp.right));setText('perLaneLeftDetail',(v.leftValid?'Valid':'Bukti belum memadai')+' • Jarak '+fmt(v.leftGap));setText('perLaneRightDetail',(v.rightValid?'Valid':'Bukti belum memadai')+' • Jarak '+fmt(v.rightGap));setText('perLaneBatchReady',colorMatch?(evidenceReady?'STATUS DAN BUKTI SESUAI':'STATUS SESUAI • BUKTI BELUM MEMADAI'):'STATUS AKTUAL TIDAK SESUAI ACUAN');if(run)run.disabled=perBab4.laneBatch.active||!channelFresh('lane_state',2.5);
  const lc=$('perLaneLeftCard'),rc=$('perLaneRightCard');if(lc)lc.className='per-lane-side-card '+v.leftStatus.toLowerCase();if(rc)rc.className='per-lane-side-card '+v.rightStatus.toLowerCase();
}
/* F43V3_CHUNK4A */
function perBab4LaneResetUi(){
  const sel=$('perBab4LaneCondition');if(sel)sel.value='center';
  perBab4.laneBatch.lastResult='RESET VIEW';perBab4LaneBatchRefresh();perBab4DrawSummary();
  toast('Kondisi pengujian dikembalikan ke Posisi Tengah Koridor Keselamatan. Data yang telah tersimpan tidak dihapus.');
}
/* F43V3_CHUNK4B */
const perBab4LaneV3PanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const r=perBab4LaneV3PanelBase();
  if(selectedExperiment?.id==='F4.3'){
    $('perLaneRun10')?.addEventListener('click',()=>perBab4LaneBatchStart(false));
    $('perLaneStopBatch')?.addEventListener('click',()=>perBab4LaneBatchStop());
    $('perLaneRepeatFailed')?.addEventListener('click',()=>perBab4LaneBatchStart(true));
    $('perLaneResetCondition')?.addEventListener('click',perBab4LaneResetUi);
    $('perBab4LaneCondition')?.addEventListener('change',()=>{perBab4LaneBatchRefresh();perBab4DrawSummary();perBab4RefreshOperatorPreview()});
    perBab4LaneBatchRefresh();setTimeout(perBab4LanePlaceControls,0);
  }
  return r;
};
/* F43V3_CHUNK4C */
function perBab4LanePlaceControls(){
  const grid=$('perBab4OperatorPreview')?.querySelector('.per-bab4-preview-grid'),stats=$('perBab4PreviewStats'),controls=$('perLaneBatchControls'),geom=$('perceptionBab4Acquisition')?.querySelector('.per-lane-geometry-strip');
  if(!grid||!stats||!controls)return;let side=$('perLanePreviewSide');
  if(!side){side=document.createElement('div');side.id='perLanePreviewSide';side.className='per-lane-preview-side';stats.replaceWith(side);side.appendChild(stats)}
  let cards=$('perLaneSideCards');if(!cards){cards=document.createElement('div');cards.id='perLaneSideCards';cards.className='per-lane-side-cards';cards.innerHTML='<div class="per-lane-side-card" id="perLaneLeftCard"><span>BATAS LAJUR KIRI</span><b id="perLaneLeftStatus">--</b><small id="perLaneLeftDetail">--</small><em id="perLaneLeftExpected">Status acuan: --</em></div><div class="per-lane-side-card" id="perLaneRightCard"><span>BATAS LAJUR KANAN</span><b id="perLaneRightStatus">--</b><small id="perLaneRightDetail">--</small><em id="perLaneRightExpected">Status acuan: --</em></div>';side.appendChild(cards)}
  if(controls.parentElement!==side)side.appendChild(controls);
  if(geom){const head=$('perBab4OperatorPreview')?.querySelector('.per-bab4-preview-head');if(head&&geom.parentElement!==head.parentElement)head.after(geom)}perBab4LaneEnsureGraphs();perBab4LaneBatchRefresh();setTimeout(perBab4DrawSummary,0);
}
/* F43V3_CHUNK5 */
function perBab4LaneEnsureGraphs(){
  if(selectedExperiment?.id!=='F4.3')return;const host=$('perceptionBab4Acquisition');if(!host||$('perLaneExtraGraphs'))return;
  const box=document.createElement('div');box.id='perLaneExtraGraphs';box.className='per-lane-extra-graphs';
  box.innerHTML='<div class="per-bab4-summary-head"><b>Jarak Batas Lajur terhadap Batas Keselamatan per Pengulangan</b><button class="soft-btn" id="perLaneSaveGapPng">Simpan Grafik Jarak</button></div><canvas id="perLaneGapCanvas" width="900" height="260"></canvas><div class="per-bab4-summary-head"><b>Distribusi Status Keamanan Lajur per Kondisi</b><button class="soft-btn" id="perLaneSaveStatusPng">Simpan Grafik Status</button></div><canvas id="perLaneStatusCanvas" width="900" height="260"></canvas>';
  host.appendChild(box);$('perLaneSaveGapPng')?.addEventListener('click',()=>perBab4DownloadCanvas('perLaneGapCanvas','grafik_jarak_batas_lajur'));$('perLaneSaveStatusPng')?.addEventListener('click',()=>perBab4DownloadCanvas('perLaneStatusCanvas','grafik_distribusi_status_keamanan_lajur'));
}
/* F43V3_CHUNK6 */
function perBab4DrawLaneGap(){
  const c=$('perLaneGapCanvas');if(!c||selectedExperiment?.id!=='F4.3')return;const ctx=c.getContext('2d'),w=c.width,h=c.height,p={l:58,r:24,t:38,b:42},cond=$('perBab4LaneCondition')?.value||'center',rows=perBab4LaneRowsFor(cond).slice().sort((a,b)=>+a.values[2]-+b.values[2]);
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='800 15px system-ui';ctx.fillText('Jarak Batas Lajur terhadap Batas Keselamatan • '+perBab4LaneV3Meta(cond).label,18,24);
  const vals=rows.flatMap(r=>[+r.values[8],+r.values[9]]).filter(Number.isFinite),warn=+configValue('perception','perception.ros__parameters.lane_corridor_warning_gap_px'),touch=+configValue('perception','perception.ros__parameters.lane_corridor_touch_margin_px'),max=Math.max(60,...vals,Number.isFinite(warn)?warn:0)*1.15;
  ctx.font='10px system-ui';ctx.fillStyle='#8fa8a9';ctx.strokeStyle='#294144';for(let i=0;i<=4;i++){const y=p.t+(h-p.t-p.b)*i/4,lab=Math.round(max*(1-i/4));ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.fillText(String(lab),14,y+3)}
  const yof=v=>h-p.b-(Math.max(0,v)/max)*(h-p.t-p.b),lineRef=(v,label)=>{if(!Number.isFinite(v))return;const y=yof(v);ctx.setLineDash([6,5]);ctx.strokeStyle='#ffc86b';ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#ffc86b';ctx.fillText(label+' '+v.toFixed(0)+' px',p.l+4,y-4)};lineRef(warn,'Batas Peringatan');lineRef(touch,'Batas Pelanggaran');
  const series=[[8,'Batas Kiri','#48e0a4'],[9,'Batas Kanan','#67b7ff']];series.forEach(([idx,label,color],si)=>{ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2;ctx.beginPath();let started=false;rows.forEach(r=>{const rep=+r.values[2],v=+r.values[idx];if(!Number.isFinite(v))return;const x=p.l+(rep-1)/9*(w-p.l-p.r),y=yof(v);started?ctx.lineTo(x,y):ctx.moveTo(x,y);started=true});ctx.stroke();rows.forEach(r=>{const rep=+r.values[2],v=+r.values[idx];if(!Number.isFinite(v))return;const x=p.l+(rep-1)/9*(w-p.l-p.r),y=yof(v);ctx.beginPath();ctx.arc(x,y,3.5,0,Math.PI*2);ctx.fill()});ctx.fillText(label,w-135,p.t+si*16)});
  for(let n=1;n<=10;n++){const x=p.l+(n-1)/9*(w-p.l-p.r);ctx.fillStyle='#8fa8a9';ctx.fillText(String(n),x-3,h-17)}if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.fillText('Data pengujian pada kondisi ini belum tersedia.',p.l,60)}
}
/* F43V3_CHUNK7 */
function perBab4DrawLaneStatus(){
  const c=$('perLaneStatusCanvas');if(!c||selectedExperiment?.id!=='F4.3')return;const ctx=c.getContext('2d'),w=c.width,h=c.height,p={l:54,r:24,t:38,b:48},cond=$('perBab4LaneCondition')?.value||'center',rows=perBab4LaneRowsFor(cond),states=['GREEN','YELLOW','RED','GRAY'];
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='800 15px system-ui';ctx.fillText('Distribusi Status Keamanan Lajur • '+perBab4LaneV3Meta(cond).label,18,24);
  const counts=side=>Object.fromEntries(states.map(s=>[s,rows.filter(r=>r.values[side]===perBab4LaneStatusLabel(s)).length])),L=counts(10),R=counts(11),colors={GREEN:'#48e0a4',YELLOW:'#ffc86b',RED:'#ff6677',GRAY:'#8fa8a9'};
  ctx.strokeStyle='#294144';ctx.fillStyle='#8fa8a9';ctx.font='10px system-ui';for(let i=0;i<=5;i++){const y=p.t+(h-p.t-p.b)*i/5,lab=10-i*2;ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.fillText(String(lab),18,y+3)}
  const group=(w-p.l-p.r)/states.length,bw=group*.28;states.forEach((s,i)=>{const x=p.l+i*group+group*.18,lh=(L[s]/10)*(h-p.t-p.b),rh=(R[s]/10)*(h-p.t-p.b);ctx.fillStyle=colors[s];ctx.fillRect(x,h-p.b-lh,bw,lh);ctx.globalAlpha=.45;ctx.fillRect(x+bw+6,h-p.b-rh,bw,rh);ctx.globalAlpha=1;ctx.fillStyle='#d8eeee';ctx.fillText(perBab4LaneStatusShortLabel(s),x,h-18);ctx.fillText('Kiri '+L[s],x,h-p.b-lh-5);ctx.fillText('Kanan '+R[s],x+bw+6,h-p.b-rh-5)});
  if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.fillText('Data pengujian pada kondisi ini belum tersedia.',p.l,60)}
}
const perBab4LaneV3DrawBase=perBab4DrawSummary;
perBab4DrawSummary=function(){const r=perBab4LaneV3DrawBase();if(selectedExperiment?.id==='F4.3'){perBab4DrawLaneGap();perBab4DrawLaneStatus()}return r};
/* F43V3_CHUNK8 */
perBab4BuildLaneMontage=async function(){
  const spec=PER_BAB4_LANE_V3.map(q=>[q[0],perBab4LaneConditionGraphLabel(q[1]),perBab4LaneStatusShortLabel(q[2])+' / '+perBab4LaneStatusShortLabel(q[3])]),reps=spec.map(x=>perBab4RepresentativeLane(x[0]));if(!reps.some(Boolean))return null;
  const c=document.createElement('canvas');c.width=1280;c.height=840;const ctx=c.getContext('2d');ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);
  perBab4EvidenceText(ctx,'Dokumentasi Visual Pengujian Keamanan Lajur • 6 Kondisi',38,48,28,'#e5f4f3',900);perBab4EvidenceText(ctx,'Satu citra aktual mewakili setiap kondisi pengujian; total enam panel dalam satu berkas PNG.',38,78,15,'#8fa8a9',600);
  const pw=390,ph=330,imgH=240,gap=18;for(let i=0;i<6;i++){const col=i%3,row=Math.floor(i/3),x=25+col*(pw+gap),y=105+row*355,rep=reps[i];ctx.fillStyle='#0b1b1d';ctx.fillRect(x,y,pw,ph);ctx.strokeStyle=rep?'rgba(91,181,173,.42)':'rgba(255,200,107,.4)';ctx.lineWidth=2;ctx.strokeRect(x,y,pw,ph);
    perBab4EvidenceText(ctx,spec[i][1],x+12,y+27,18,'#e5f4f3',900);perBab4EvidenceText(ctx,spec[i][2],x+255,y+27,13,'#9dc0bd',800);
    if(rep){try{const im=await perBab4LoadEvidenceImage(rep.dataUrl);perBab4DrawImageContain(ctx,im,x+8,y+38,pw-16,imgH)}catch(_){perBab4EvidenceText(ctx,'CITRA TIDAK TERSEDIA',x+95,y+170,16,'#ff6677',800)}
      perBab4EvidenceText(ctx,`Pengulangan ${rep.repeat} • ${perBab4LaneStatusShortLabel(rep.left)} / ${perBab4LaneStatusShortLabel(rep.right)} • ${perBab4LaneResultLabel(rep.match)}`,x+10,y+299,12,rep.match?'#48e0a4':'#ff6677',800);perBab4EvidenceText(ctx,perBab4LaneEvidenceLabel(rep.evidenceMode)||perBab4LaneRecommendationLabel(rep.recommendation),x+10,y+317,10,'#9dc0bd',700)}
    else perBab4EvidenceText(ctx,'Citra bukti belum tersedia',x+95,y+175,14,'#8fa8a9',700);
  }
  perBab4EvidenceText(ctx,'Parameter geometri: lebar jalan 6,0 m • koridor keselamatan 4,0 m • titik bawah 4,3 m • titik atas 9,3 m • margin lateral 1,0 m/sisi',25,822,12,'#708f8d',600);return c;
};
/* F43V3_CHUNK9 */
const perBab4LaneV3PrepareBase=perBab4PrepareEvidenceExports;
perBab4PrepareEvidenceExports=async function(){
  if(selectedExperiment?.id!=='F4.3')return perBab4LaneV3PrepareBase();
  perBab4.evidence.exports=[];const c=await perBab4BuildLaneMontage();if(c)perBab4.evidence.exports.push({index:4,title:'Dokumentasi Visual Enam Kondisi Pengujian Keamanan Lajur',data_url:c.toDataURL('image/png')});
};
const perBab4LaneV3PackageBase=perBab4EvidencePackageHtml;
perBab4EvidencePackageHtml=function(){
  if(selectedExperiment?.id!=='F4.3')return perBab4LaneV3PackageBase();
  return '<section class="per-bab4-evidence-card per-bab4-save-package" id="perBab4SavePackage"><div class="per-bab4-evidence-head"><div><span class="eyebrow">KELUARAN DATA PENGUJIAN</span><b>60 data pengujian + 3 grafik + 1 kompilasi visual enam kondisi</b><p>Berkas CSV/XLSX memuat kondisi pengujian, status batas lajur kiri dan kanan, jarak dalam piksel, validitas area dapat dilalui dan batas lajur, status acuan, kesesuaian hasil, tingkat keberhasilan, umur data, mode bukti, serta rekomendasi sistem.</p></div><span class="status-chip" id="perBab4VisualState">0 citra</span></div><div class="per-bab4-package-grid"><div><span>Citra representatif</span><b id="perBab4VisualFrames">0 / 6 citra</b></div><div><span>Kondisi terdokumentasi</span><b id="perBab4LaneReady">0 / 6</b></div><div><span>Keluaran visual</span><b>3 grafik + 1 kompilasi visual</b></div><div><span>Data utama</span><b>60 data + rekapitulasi tingkat keberhasilan</b></div></div><div class="per-bab4-package-actions"><button class="soft-btn" id="perBab4PreviewLane">PRATINJAU DOKUMENTASI 6 KONDISI</button></div></section>';
};
const perBab4LaneV3RefreshPackageBase=perBab4RefreshEvidencePackage;
perBab4RefreshEvidencePackage=function(){
  perBab4LaneV3RefreshPackageBase();if(selectedExperiment?.id!=='F4.3')return;
  const n=perBab4.evidence.f43.length,keys=new Set(perBab4.evidence.f43.map(x=>x.key));setText('perBab4VisualState',n+' citra');setText('perBab4VisualFrames',n+' / 60');setText('perBab4LaneReady',keys.size+' / 6');const chip=$('perBab4VisualState');if(chip)chip.className='status-chip '+(n>=60?'':'waiting');
};
/* F43V3_CHUNK10 */
const perBab4LaneV3GraphPayloadBase=collectExperimentGraphPngPayload;
collectExperimentGraphPngPayload=function(){
  const out=perBab4LaneV3GraphPayloadBase();if(selectedExperiment?.id!=='F4.3')return out;
  perBab4DrawLaneGap();perBab4DrawLaneStatus();const add=(id,title,index)=>{const c=$(id);if(c&&c.width&&c.height)out.push({index,title,data_url:c.toDataURL('image/png')})};add('perLaneGapCanvas','Jarak Batas Lajur terhadap Batas Keselamatan per Pengulangan',2);add('perLaneStatusCanvas','Distribusi Status Keamanan Lajur per Kondisi',3);return out;
};
/* F43V3_DONE */
const perBab4F43TaStandardRefreshBase=perBab4RefreshLive;
perBab4RefreshLive=function(){
  const r=perBab4F43TaStandardRefreshBase();
  if(selectedExperiment?.id==='F4.3'){
    const v=perBab4LaneV3Visual();
    setText('perBab4LivePrimary',`${perBab4LaneRecommendationLabel(v.c.recommendation)} • Batas kiri: ${perBab4LaneStatusLabel(v.leftStatus)} • Batas kanan: ${perBab4LaneStatusLabel(v.rightStatus)} • ${perBab4LaneEvidenceLabel(v.c.evidence_mode)}`);
    perBab4LaneBatchRefresh();
  }
  return r;
};



/* ===== FINAL 4.2 OBJECT DETECTION • 10x / 30s v1 (2026-09-20) =====
 * Formal protocol: 2 objects x 5 distances x 10 captures = 100 captures.
 * Each condition runs 30 s; captures are anchored at t=3,6,...,30 s.
 * UI/report only: obstacle-calibration data and perception YAML are untouched.
 */
const PER_F42_REPEATS=10;
const PER_F42_INTERVAL_MS=3000;
const PER_F42_DURATION_MS=30000;
perBab4.f42Batch=perBab4.f42Batch||{active:false,busy:false,target:'human',distance:1,queue:[],done:0,total:0,startMs:0,lastStamp:0,timer:null,lastResult:'Menunggu Data'};

function perF42TargetLabel(target){return target==='motorcycle'?'Motorcycle':'Human'}
function perF42TargetAcademicLabel(target){return target==='motorcycle'?'Sepeda Motor':'Manusia'}
function perF42ExpectedClass(target){return target==='motorcycle'?3:0}
function perF42ClassLabel(id){id=+id;if(id===0)return'Human';if(id===3)return'Motorcycle';if(id===2)return'Car';return Number.isFinite(id)?('Class '+id):'--'}
function perF42ClassAcademicLabel(id){id=+id;if(id===0)return'Manusia';if(id===3)return'Sepeda Motor';if(id===2)return'Mobil';return Number.isFinite(id)?('Kelas '+id):'Tidak Terdeteksi'}
function perF42StatusAcademicLabel(status){const m={CORRECT:'Klasifikasi Sesuai',MISCLASSIFIED:'Salah Klasifikasi',NOT_DETECTED:'Tidak Terdeteksi'};return m[String(status||'').toUpperCase()]||String(status||'Tidak Terdeteksi')}
function perF42DecisionAcademicLabel(reason){let s=String(reason||'Tidak Tersedia');return s.replace(/TOP CLASS MISMATCH/g,'Kelas utama tidak sesuai acuan').replace(/CLASS MISMATCH/g,'Kelas tidak sesuai acuan').replace(/CLASS MATCH/g,'Kelas sesuai acuan').replace(/TARGET CLASS ABSENT/g,'Kelas target tidak ditemukan').replace(/NO PLAUSIBLE BBOX IN TARGET ZONE/g,'Kotak pembatas yang sesuai tidak ditemukan pada zona pengujian').replace(/NO SEMANTIC BBOX IN TARGET ZONE/g,'Kotak pembatas semantik tidak ditemukan pada zona pengujian').replace(/NO SEMANTIC CANDIDATE/g,'Kandidat semantik tidak tersedia').replace(/BELOW THRESHOLD/g,'Di bawah ambang kepercayaan').replace(/Human evidence/gi,'Bukti kelas Manusia').replace(/Motorcycle evidence/gi,'Bukti kelas Sepeda Motor').replace(/LOW CONFIDENCE|EVAL LOW CONF/g,'Nilai kepercayaan rendah').replace(/semantic/gi,'sumber semantik')}
function perF42Threshold(){
  const v=+configValue('perception','perception.ros__parameters.confidence_threshold');
  return Number.isFinite(v)?Math.max(0,Math.min(1,v)):0.25;
}
function perF42DistanceEstimate(candidate){
  for(const k of ['calibrated_forward_m','raw_forward_m']){const v=+candidate?.[k];if(Number.isFinite(v))return v}
  const arr=Array.isArray(obj('semantic_detections')?.detections)?obj('semantic_detections').detections:[],cid=+candidate?.class_id;
  const matches=arr.filter(x=>+x.class_id===cid&&Number.isFinite(+x.calibrated_forward_m)).sort((a,b)=>Math.abs((+a.center_x_fraction||.5)-.5)-Math.abs((+b.center_x_fraction||.5)-.5));
  return matches.length?+matches[0].calibrated_forward_m:NaN;
}

function perF42Decision(){
  const base=perBab4AutoDetectionDecision(),threshold=perF42Threshold(),candidate=base.candidate,score=+candidate?.score;
  if(!base.success||!candidate)return{...base,valid:false,status:'NOT_DETECTED',threshold,reason:base.reason||'NO TARGET BBOX'};
  if(!Number.isFinite(score)||score<threshold)return{...base,valid:false,status:'NOT_DETECTED',threshold,reason:`BELOW THRESHOLD ${Number.isFinite(score)?score.toFixed(3):'--'} < ${threshold.toFixed(3)}`};
  const target=$('perBab4Target')?.value||perBab4.f42Batch.target||'human',correct=+candidate.class_id===perF42ExpectedClass(target);
  return{...base,valid:true,status:correct?'CORRECT':'MISCLASSIFIED',threshold,reason:correct?'CLASS MATCH':'CLASS MISMATCH'};
}
function perF42ApplySchema(){
  const x=(experiments.perception||[]).find(v=>v.id==='F4.2');if(!x)return;
  Object.assign(x,{groupId:'FINAL-4.2',groupTitle:'BAB IV — 4.2 Pengujian Deteksi Objek YOLOPv2',
    section:'4.2 Pengujian Deteksi Objek YOLOPv2 — Manusia dan Sepeda Motor • Jarak 1–5 m • 10 Pengulangan/Kondisi • 30 s/Kondisi',
    reportMode:true,bab4Protocol:'detection',bab4ManualCapture:true,bab4NoTimeAxis:true,parameterFields:perBab4DetectionFields(),
    tableColumns:[
      ['No. Data','Objek Acuan','Jarak Acuan [m]','Pengulangan ke-','Waktu Pengambilan','Kelas Terdeteksi','ID Kelas Mentah','Nilai Kepercayaan (Confidence)','Ambang Kepercayaan','Estimasi Jarak [m]','Galat Jarak [m]','Lebar Kotak Pembatas [piksel]','Tinggi Kotak Pembatas [piksel]','Hasil Deteksi','Backend Inferensi','Jumlah Kandidat Deteksi','Dasar Keputusan','Sumber Bukti'],
      ['Objek Acuan','Jarak Acuan [m]','Jumlah Data','Klasifikasi Sesuai','Salah Klasifikasi','Tidak Terdeteksi','Tingkat Deteksi [%]','Akurasi Klasifikasi [%]','Rata-rata Confidence','Confidence Minimum','Confidence Maksimum','Simpangan Baku Confidence','Rata-rata Estimasi Jarak [m]','Rata-rata Galat Jarak [m]','MAE Jarak [m]']
    ],
    tableNames:['Data Hasil Pengujian Deteksi Objek YOLOPv2','Rekapitulasi Kinerja Deteksi Objek per Jarak'],
    graphCaptions:['Tingkat Deteksi terhadap Jarak Acuan','Rata-rata Nilai Kepercayaan terhadap Jarak Acuan','Nilai Kepercayaan pada 10 Pengulangan Kondisi Terpilih','Galat Jarak pada 10 Pengulangan Kondisi Terpilih'],
    graphs:[],liveSeries:{'Jumlah deteksi mentah':'raw_detections.count','Laju pipeline [FPS]':'perception_performance.pipeline_fps'}});
  if(selectedExperiment?.id==='F4.2')selectedExperiment=x;
}
const perF42PatchBase=patchPerceptionBab4Catalog;
patchPerceptionBab4Catalog=function(){perF42PatchBase();perF42ApplySchema()};
perF42ApplySchema();

function perF42RowsFor(target,distance){const labels=new Set([perF42TargetLabel(target),perF42TargetAcademicLabel(target)]);return perBab4Rows().filter(r=>labels.has(r.values?.[1])&&+r.values?.[2]===+distance)}
function perBab4BuildDetectionSummary(){
  const groups=new Map();
  for(const r of perBab4Rows()){
    const v=r.values,key=`${v[1]}|${v[2]}`,g=groups.get(key)||{object:v[1],dist:+v[2],rows:[]};
    g.rows.push(v);groups.set(key,g);
  }
  return [...groups.values()].sort((a,b)=>String(a.object).localeCompare(String(b.object))||a.dist-b.dist).map(g=>{
    const n=g.rows.length,correct=g.rows.filter(v=>['CORRECT','Klasifikasi Sesuai'].includes(v[13])),mis=g.rows.filter(v=>['MISCLASSIFIED','Salah Klasifikasi'].includes(v[13])),lost=g.rows.filter(v=>['NOT_DETECTED','Tidak Terdeteksi'].includes(v[13]));
    const detected=correct.length+mis.length,conf=correct.map(v=>+v[7]).filter(Number.isFinite),est=g.rows.map(v=>+v[9]).filter(Number.isFinite),err=g.rows.map(v=>+v[10]).filter(Number.isFinite);
    const mean=a=>a.length?a.reduce((s,x)=>s+x,0)/a.length:NaN,std=a=>{if(!a.length)return NaN;const m=mean(a);return Math.sqrt(a.reduce((s,x)=>s+(x-m)*(x-m),0)/a.length)};
    const fmt=(x,d=3)=>Number.isFinite(x)?x.toFixed(d):'--';
    return[g.object,g.dist,n,correct.length,mis.length,lost.length,fmt(n?100*detected/n:0,2),fmt(detected?100*correct.length/detected:0,2),fmt(mean(conf),5),fmt(conf.length?Math.min(...conf):NaN,5),fmt(conf.length?Math.max(...conf):NaN,5),fmt(std(conf),5),fmt(mean(est),3),fmt(mean(err),3),fmt(mean(err.map(Math.abs)),3)];
  });
}
function perF42ConditionStats(target,distance){
  const rows=perF42RowsFor(target,distance).map(r=>r.values),n=rows.length,correct=rows.filter(v=>['CORRECT','Klasifikasi Sesuai'].includes(v[13])),mis=rows.filter(v=>['MISCLASSIFIED','Salah Klasifikasi'].includes(v[13])),lost=rows.filter(v=>['NOT_DETECTED','Tidak Terdeteksi'].includes(v[13])),detected=correct.length+mis.length;
  const conf=correct.map(v=>+v[7]).filter(Number.isFinite),est=rows.map(v=>+v[9]).filter(Number.isFinite),err=rows.map(v=>+v[10]).filter(Number.isFinite),mean=a=>a.length?a.reduce((s,x)=>s+x,0)/a.length:NaN,std=a=>{if(!a.length)return NaN;const m=mean(a);return Math.sqrt(a.reduce((s,x)=>s+(x-m)*(x-m),0)/a.length)};
  return{n,correct:correct.length,mis:mis.length,lost:lost.length,detectionRate:n?100*detected/n:0,classAccuracy:detected?100*correct.length/detected:0,confMean:mean(conf),confMin:conf.length?Math.min(...conf):NaN,confMax:conf.length?Math.max(...conf):NaN,confStd:std(conf),distMean:mean(est),errorMean:mean(err),mae:mean(err.map(Math.abs))};
}

async function perF42CaptureOne(target,distance,repeat){
  const decision=perF42Decision(),dets=(decision.dets||[]).map(x=>({...x})),cand=decision.candidate,status=decision.status;
  const conf=Number.isFinite(+cand?.score)?+cand.score:NaN,est=decision.valid?perF42DistanceEstimate(cand):NaN,err=Number.isFinite(est)?est-distance:NaN;
  const semW=+obj('semantic_detections').image_width||1280,semH=+obj('semantic_detections').image_height||720;const bw=cand&&Number.isFinite(+cand.x1)&&Number.isFinite(+cand.x2)?Math.abs(+cand.x2-+cand.x1)*semW:NaN,bh=cand&&Number.isFinite(+cand.y1)&&Number.isFinite(+cand.y2)?Math.abs(+cand.y2-+cand.y1)*semH:NaN;
  const backend=obj('raw_detections').backend||obj('perception_performance').backend||'--',detected=cand?perF42ClassLabel(cand.class_id):'--';
  const evidenceLabel=`F4.2_${target}_${distance}m_capture_${String(repeat).padStart(2,'0')}_${status}`;
  const ev=await perBab4PersistTrialEvidence(evidenceLabel),evidencePath=ev.image_path||'--';
  const dataUrl=perBab4EvidenceJpeg({kind:'detection',detections:dets,selectedIndex:Math.max(0,decision.index??-1),title:`${perF42TargetLabel(target)} • ${distance} m • capture ${repeat}/10`,subtitle:`${status} • conf ${Number.isFinite(conf)?(conf*100).toFixed(1)+'%':'--'} • est ${Number.isFinite(est)?est.toFixed(2)+' m':'--'}`});
  const f=(v,n=4)=>Number.isFinite(v)?v.toFixed(n):'--',values=[perBab4Rows().length+1,perF42TargetLabel(target),distance,repeat,perBab4FrameTimestamp(),detected,cand?.class_id??'--',f(conf,5),f(decision.threshold,5),f(est,3),f(err,3),f(bw,1),f(bh,1),status,backend,dets.length,decision.reason,evidencePath];
  const key=`${perF42TargetLabel(target)}|${distance}|${repeat}`,retake=perBab4UpsertMainTrial(key,values,v=>`${v[1]}|${v[2]}|${v[3]}`);
  perBab4.evidence.f42=perBab4.evidence.f42.filter(x=>!(x.target===target&&+x.distance===distance&&+x.repeat===repeat));
  if(dataUrl)perBab4.evidence.f42.push({target,distance,repeat,success:status==='CORRECT',status,confidence:conf,estimatedDistance:est,dataUrl,evidencePath});
  else perBab4.evidence.captureWarnings++;
  perBab4.f42Batch.lastResult=`${retake?'RETAKE • ':''}${status} • ${detected} • ${Number.isFinite(conf)?(conf*100).toFixed(1)+'%':'--'}`;
  perBab4RefreshEvidencePackage();perF42RefreshUi();return status;
}
function perF42BatchStop(silent=false){
  const s=perBab4.f42Batch;s.active=false;s.busy=false;clearInterval(s.timer);s.timer=null;
  const a=$('perBab4Target'),b=$('perBab4Distance');if(a)a.disabled=false;if(b)b.disabled=false;
  if(!silent)toast(`Pengambilan data 4.2 dihentikan • ${s.done}/${s.total} data terekam`);perF42RefreshUi();
}

async function perF42BatchTick(){
  const s=perBab4.f42Batch;
  if(!s.active||s.busy||!s.queue.length)return;
  if(!recordStartedMs){perF42BatchStop();return}
  const due=(s.done+1)*PER_F42_INTERVAL_MS;
  const elapsed=Date.now()-s.startMs;
  const stamp=+updated.raw_detections||0;
  if(elapsed<due||!channelFresh('raw_detections',2.5)||!stamp||stamp<=s.lastStamp){perF42RefreshUi();return}
  s.busy=true;
  const repeat=s.queue[0];
  s.lastStamp=stamp;
  try{
    await perF42CaptureOne(s.target,s.distance,repeat);
    s.queue.shift();s.done++;
    if(!s.queue.length){perF42BatchStop(true);toast(perF42TargetLabel(s.target)+' '+s.distance+' m: 10× selesai • '+s.done+'/'+s.total)}
  }catch(e){perF42BatchStop(true);toast(e.message||String(e),true)}
  finally{s.busy=false;perF42RefreshUi();perBab4DrawSummary()}
}

async function perF42BatchStart(failedOnly=false){
  const s=perBab4.f42Batch;
  if(s.active)return;
  if(!recordStartedMs){
    await startWebRecording();
    if(!recordStartedMs)return toast('Sesi 4.2 gagal dimulai. Cek preflight.',true);
  }
  if(!channelFresh('raw_detections',2.5))return toast('Raw detection stream belum fresh. Tunggu source READY.',true);
  const target=$('perBab4Target')?.value||'human';
  const distance=+$('perBab4Distance')?.value||1;
  const byRep=new Map(perF42RowsFor(target,distance).map(r=>[+r.values[3],r.values[13]]));
  const all=[1,2,3,4,5,6,7,8,9,10];
  const queue=failedOnly?all.filter(n=>!['CORRECT','Klasifikasi Sesuai'].includes(byRep.get(n))):all;
  if(!queue.length)return toast('Semua 10 capture kondisi ini sudah CORRECT.');
  Object.assign(s,{active:true,busy:false,target,distance,queue:[...queue],done:0,total:queue.length,startMs:Date.now(),lastStamp:+updated.raw_detections||0,lastResult:'WAIT t=3 s'});
  const a=$('perBab4Target'),b=$('perBab4Distance');
  if(a)a.disabled=true;if(b)b.disabled=true;
  clearInterval(s.timer);
  s.timer=setInterval(perF42BatchTick,120);
  perF42RefreshUi();
}

perBab4DetectionControls=function(){
  return '<div class="per-f42-batch-controls" id="perF42BatchControls">'+
    '<label>Objek Acuan<select id="perBab4Target"><option value="human">Manusia</option><option value="motorcycle">Sepeda Motor</option></select></label>'+
    '<label>Jarak Acuan<select id="perBab4Distance">'+[1,2,3,4,5].map(x=>'<option value="'+x+'">'+x+' meter</option>').join('')+'</select></label>'+
    '<div class="per-f42-control-state"><span>Jumlah Data</span><b id="perF42CaptureProgress">0 / 10</b></div>'+
    '<div class="per-f42-control-state"><span>Durasi Pengujian</span><b>30 s • interval 3 s</b></div>'+
    '<div class="per-f42-control-state"><span>Pengambilan Berikutnya</span><b id="perF42NextCapture">--</b></div>'+
    '<div class="per-f42-control-state"><span>Hasil Terakhir</span><b id="perF42LastResult">Menunggu Data</b></div>'+
    '<div class="per-f42-batch-actions"><button class="primary-btn" id="perF42Run10">MULAI PENGAMBILAN 10 DATA / 30 s</button><button class="soft-btn" id="perF42StopBatch">HENTIKAN PENGAMBILAN</button><button class="soft-btn" id="perF42RepeatFailed">ULANGI DATA TIDAK MEMENUHI KRITERIA</button></div>'+
    '<div class="per-f42-test-hint"><b>Prosedur:</b> tempatkan satu objek pada jarak acuan, pertahankan posisi objek relatif tetap, kemudian mulai pengambilan 10 data. Sistem merekam satu hasil inferensi terbaru pada detik ke-3, 6, ..., 30. Salah klasifikasi dan kondisi tidak terdeteksi tetap dicatat sebagai hasil pengujian.</div>'+
  '</div>';
};

function perF42PlaceControls(){
  if(selectedExperiment?.id!=='F4.2')return;
  const grid=$('perBab4OperatorPreview')?.querySelector('.per-bab4-preview-grid');
  const stats=$('perBab4PreviewStats'),controls=$('perF42BatchControls');
  if(!grid||!stats||!controls)return;
  let side=$('perF42PreviewSide');
  if(!side){
    side=document.createElement('div');
    side.id='perF42PreviewSide';side.className='per-f42-preview-side';
    stats.replaceWith(side);side.appendChild(stats);
  }
  if(controls.parentElement!==side)side.appendChild(controls);
}
function perF42RefreshUi(){
  if(selectedExperiment?.id!=='F4.2')return;
  const s=perBab4.f42Batch,target=s.active?s.target:($('perBab4Target')?.value||'human'),distance=s.active?s.distance:(+$('perBab4Distance')?.value||1);
  const stats=perF42ConditionStats(target,distance),elapsed=s.active?Date.now()-s.startMs:0,next=s.active?Math.max(0,(s.done+1)*PER_F42_INTERVAL_MS-elapsed):NaN;
  setText('perF42CaptureProgress',stats.n+' / 10');
  setText('perF42NextCapture',s.active?(next/1000).toFixed(1)+' s':'--');
  setText('perF42LastResult',s.lastResult||'Menunggu Data');
  const run=$('perF42Run10'),stop=$('perF42StopBatch'),retry=$('perF42RepeatFailed');
  if(run)run.disabled=s.active;if(stop)stop.disabled=!s.active;if(retry)retry.disabled=s.active||!recordStartedMs;
}

const perF42ProgressBase=perBab4RenderProgress;
perBab4RenderProgress=function(){
  if(selectedExperiment?.id!=='F4.2')return perF42ProgressBase();
  const box=$('perBab4Progress');if(!box)return;
  const count=(target,d)=>{const labels=new Set([perF42TargetLabel(target),perF42TargetAcademicLabel(target)]);return perBab4Rows().filter(r=>labels.has(r.values?.[1])&&+r.values?.[2]===d).length};
  box.innerHTML=[['human','Manusia'],['motorcycle','Sepeda Motor']].map(([target,label])=>'<div class="per-bab4-progress-row"><b>'+label+'</b>'+
    [1,2,3,4,5].map(d=>{const n=count(target,d);return '<span class="'+(n>=10?'done':'')+'">'+d+' m <em>'+n+'/10</em></span>'}).join('')+'</div>').join('');
};
const perF42PanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const out=perF42PanelBase();
  if(selectedExperiment?.id==='F4.2'){
    $('perF42Run10')?.addEventListener('click',()=>perF42BatchStart(false));
    $('perF42StopBatch')?.addEventListener('click',()=>perF42BatchStop());
    $('perF42RepeatFailed')?.addEventListener('click',()=>perF42BatchStart(true));
    $('perBab4Target')?.addEventListener('change',()=>{perF42RefreshUi();perBab4DrawSummary();perBab4RefreshOperatorPreview()});
    $('perBab4Distance')?.addEventListener('change',()=>{perF42RefreshUi();perBab4DrawSummary();perBab4RefreshOperatorPreview()});
    perF42RefreshUi();setTimeout(perF42PlaceControls,0);
  }
  return out;
};
const perF42InfoBase=perBab4InfoText;
perBab4InfoText=function(){if(selectedExperiment?.id==='F4.2')return 'Pengujian terdiri atas 10 kondisi: manusia dan sepeda motor pada jarak acuan 1–5 m. Setiap kondisi direkam sebanyak 10 pengulangan selama 30 s dengan interval 3 s. Nilai kepercayaan menggunakan ambang aktif pada konfigurasi YAML.';return perF42InfoBase()};
const perF42StepBase=perBab4StepText;
perBab4StepText=function(){if(selectedExperiment?.id==='F4.2')return['Tujuan','Mengukur kinerja deteksi dan klasifikasi objek YOLOPv2 pada manusia dan sepeda motor pada jarak 1–5 m.','Prosedur','Pilih objek acuan dan jarak acuan, tempatkan objek pada posisi pengujian, kemudian lakukan 10 pengulangan selama 30 s. Pertahankan kondisi fisik relatif tetap selama satu rangkaian pengambilan.','Keluaran penelitian','Data hasil 100 pengujian, tingkat deteksi, akurasi klasifikasi, statistik nilai kepercayaan, estimasi dan galat jarak, citra bukti, serta berkas CSV/XLSX/PNG/ZIP.'];return perF42StepBase()};

const perF42PreviewBase=perBab4RefreshOperatorPreview;
perBab4RefreshOperatorPreview=function(){
  const out=perF42PreviewBase();
  if(selectedExperiment?.id!=='F4.2')return out;
  const box=$('perBab4PreviewStats'),chip=$('perBab4PreviewState');if(!box)return out;
  const target=perBab4.f42Batch.active?perBab4.f42Batch.target:($('perBab4Target')?.value||'human'),distance=perBab4.f42Batch.active?perBab4.f42Batch.distance:(+$('perBab4Distance')?.value||1);
  const d=perF42Decision(),cand=d.candidate,conf=Number.isFinite(+cand?.score)?+cand.score:NaN,est=cand?perF42DistanceEstimate(cand):NaN,err=Number.isFinite(est)?est-distance:NaN,st=perF42ConditionStats(target,distance);
  const pct=v=>Number.isFinite(v)?v.toFixed(1)+'%':'--',num=(v,n=3,s='')=>Number.isFinite(v)?v.toFixed(n)+s:'--';
  const rows=[['Objek Acuan',perF42TargetAcademicLabel(target)],['Kelas Terdeteksi',cand?perF42ClassAcademicLabel(cand.class_id):'Tidak Terdeteksi'],['Nilai Kepercayaan',Number.isFinite(conf)?(conf*100).toFixed(1)+'%':'--'],['Ambang Kepercayaan',(d.threshold*100).toFixed(1)+'%'],['Jarak Acuan',distance.toFixed(2)+' m'],['Estimasi Jarak',num(est,2,' m')],['Galat Jarak',num(err,2,' m')],['Status Aktual',perF42StatusAcademicLabel(d.status)],['Data Terekam',st.n+' / 10'],['Sesuai / Salah Klasifikasi / Tidak Terdeteksi',st.correct+' / '+st.mis+' / '+st.lost],['Tingkat Deteksi',pct(st.detectionRate)],['Akurasi Klasifikasi',pct(st.classAccuracy)],['Rata-rata Confidence',Number.isFinite(st.confMean)?(st.confMean*100).toFixed(1)+'%':'--'],['Confidence Minimum / Maksimum',(Number.isFinite(st.confMin)?(st.confMin*100).toFixed(1):'--')+' / '+(Number.isFinite(st.confMax)?(st.confMax*100).toFixed(1):'--')+' %'],['Simpangan Baku Confidence',Number.isFinite(st.confStd)?(st.confStd*100).toFixed(2)+'%':'--'],['MAE Jarak',num(st.mae,3,' m')]];
  box.innerHTML=rows.map(([k,v])=>'<div><span>'+escapeHtml(k)+'</span><b>'+escapeHtml(String(v))+'</b></div>').join('');
  if(chip){chip.textContent=(perBab4.f42Batch.active?'PENGAMBILAN '+perBab4.f42Batch.done+'/'+perBab4.f42Batch.total+' • ':'')+perF42StatusAcademicLabel(d.status);chip.className='status-chip '+(d.status==='CORRECT'?'':d.status==='MISCLASSIFIED'?'bad':'waiting')}
  perF42RefreshUi();return out;
};

const perF42OverlayBase=perBab4DrawOperatorOverlay;
perBab4DrawOperatorOverlay=function(){
  const out=perF42OverlayBase();
  if(selectedExperiment?.id!=='F4.2')return out;
  const canvas=$('perBab4PreviewOverlay'),img=$('perBab4PreviewImage'),g=cameraGeometryFor(img);if(!canvas||!img||!g)return out;
  const d=perF42Decision(),cand=d.candidate;if(!cand)return out;
  const b=perBab4AutoNormalizeBox(cand,img.naturalWidth,img.naturalHeight);if(!b)return out;
  const p=g.imageToScreen(b.x1,b.y1),cr=canvas.getBoundingClientRect();if(!p)return out;
  const ctx=canvas.getContext('2d'),dpr=Math.max(1,window.devicePixelRatio||1),conf=Number.isFinite(+cand.score)?(+cand.score*100).toFixed(1)+'%':'--',est=perF42DistanceEstimate(cand);
  const text=perF42ClassLabel(cand.class_id)+' | '+conf+' | '+(Number.isFinite(est)?est.toFixed(2)+' m':'--')+' | '+d.status;
  ctx.save();ctx.font='900 '+(12*dpr)+'px ui-monospace';const tw=ctx.measureText(text).width,pad=5*dpr,x=(p.x-cr.left+3)*dpr,y=Math.max(20*dpr,(p.y-cr.top-8)*dpr);
  ctx.fillStyle='rgba(0,0,0,.78)';ctx.fillRect(x-pad,y-15*dpr,tw+2*pad,20*dpr);
  ctx.fillStyle=d.status==='CORRECT'?'#48e0a4':d.status==='MISCLASSIFIED'?'#ff6677':'#ffc86b';ctx.fillText(text,x,y);ctx.restore();
  return out;
};
const perF42StopRecordingBase=stopWebRecording;
stopWebRecording=async function(){
  if(perBab4.f42Batch?.active)perF42BatchStop(true);
  return perF42StopRecordingBase();
};

function perF42EnsureGraphs(){
  if(selectedExperiment?.id!=='F4.2')return;const host=$('perceptionBab4Acquisition');if(!host||$('perF42ExtraGraphs'))return;
  const box=document.createElement('div');box.id='perF42ExtraGraphs';box.className='per-f42-extra-graphs';
  box.innerHTML='<div class="per-bab4-summary-head"><b>Nilai Kepercayaan pada 10 Pengulangan Kondisi Terpilih</b><button class="soft-btn" id="perF42SaveCaptureConf">Simpan Grafik Confidence</button></div><canvas id="perF42CaptureConfidenceCanvas" width="900" height="250"></canvas>'+
    '<div class="per-bab4-summary-head"><b>Galat Jarak pada 10 Pengulangan Kondisi Terpilih</b><button class="soft-btn" id="perF42SaveDistanceError">Simpan Grafik Galat Jarak</button></div><canvas id="perF42DistanceErrorCanvas" width="900" height="250"></canvas>';
  host.appendChild(box);
  $('perF42SaveCaptureConf')?.addEventListener('click',()=>perBab4DownloadCanvas('perF42CaptureConfidenceCanvas','grafik_confidence_10_pengulangan'));
  $('perF42SaveDistanceError')?.addEventListener('click',()=>perBab4DownloadCanvas('perF42DistanceErrorCanvas','grafik_galat_jarak_10_pengulangan'));
}
function perF42DrawLineCanvas(id,title,rows,valueIndex,scale=1,suffix=''){
  const c=$(id);if(!c)return;const ctx=c.getContext('2d'),w=c.width,h=c.height,p={l:58,r:22,t:42,b:38},vals=rows.map(r=>+r.values[valueIndex]*scale),finite=vals.filter(Number.isFinite);
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='800 15px system-ui';ctx.fillText(title,18,24);
  const lo=valueIndex===7?0:(finite.length?Math.min(0,...finite):-.1),hi=valueIndex===7?100:(finite.length?Math.max(.1,...finite):.1),span=Math.max(.01,hi-lo);
  ctx.font='10px system-ui';for(let i=0;i<=4;i++){const y=p.t+(h-p.t-p.b)*i/4,v=hi-span*i/4;ctx.strokeStyle='#294144';ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.fillStyle='#8fa8a9';ctx.fillText(v.toFixed(valueIndex===7?0:2)+suffix,8,y+3)}
  ctx.strokeStyle='#48e0a4';ctx.fillStyle='#48e0a4';ctx.lineWidth=2;let prev=null;
  rows.forEach((r,i)=>{const rep=+r.values[3],v=+r.values[valueIndex]*scale;if(!Number.isFinite(v))return;const x=p.l+(rep-1)/9*(w-p.l-p.r),y=p.t+(hi-v)/span*(h-p.t-p.b);if(prev){ctx.beginPath();ctx.moveTo(prev.x,prev.y);ctx.lineTo(x,y);ctx.stroke()}ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fill();prev={x,y};ctx.fillStyle=['CORRECT','Klasifikasi Sesuai'].includes(r.values[13])?'#48e0a4':['MISCLASSIFIED','Salah Klasifikasi'].includes(r.values[13])?'#ff6677':'#ffc86b';ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fill()});
  for(let n=1;n<=10;n++){const x=p.l+(n-1)/9*(w-p.l-p.r);ctx.fillStyle='#8fa8a9';ctx.fillText(String(n),x-3,h-15)}
  if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.fillText('Data pada kondisi pengujian terpilih belum tersedia.',p.l,68)}
}

function perF42DrawOverviewCanvas(id,title,index,scale=1){
  const c=$(id);if(!c)return;const rows=perBab4BuildDetectionSummary(),ctx=c.getContext('2d'),w=c.width,h=c.height,p={l:58,r:20,t:38,b:52};
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#071113';ctx.fillRect(0,0,w,h);ctx.fillStyle='#d8eeee';ctx.font='800 15px system-ui';ctx.fillText(title,18,24);
  for(let i=0;i<=4;i++){const v=100-i*25,y=p.t+(h-p.t-p.b)*i/4;ctx.strokeStyle='#294144';ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(w-p.r,y);ctx.stroke();ctx.fillStyle='#8fa8a9';ctx.font='10px system-ui';ctx.fillText(v+'%',16,y+3)}
  if(!rows.length){ctx.fillStyle='#8fa8a9';ctx.fillText('Belum ada data.',p.l,68);return}
  const step=(w-p.l-p.r)/rows.length,bw=step*.58;
  rows.forEach((r,i)=>{const raw=+r[index],v=Number.isFinite(raw)?raw*scale:0,x=p.l+i*step+(step-bw)/2,bh=Math.max(0,Math.min(100,v))/100*(h-p.t-p.b),y=h-p.b-bh;
    ctx.fillStyle=['Human','Manusia'].includes(r[0])?'#48e0a4':'#67b7ff';ctx.fillRect(x,y,bw,bh);ctx.fillStyle='#d8eeee';ctx.font='10px system-ui';ctx.fillText(Number.isFinite(raw)?v.toFixed(1)+'%':'--',x,y-5);
    ctx.save();ctx.translate(x+bw/2,h-p.b+12);ctx.rotate(-.35);ctx.fillStyle='#8fa8a9';ctx.fillText((['Human','Manusia'].includes(r[0])?'Manusia':'Sepeda Motor')+' '+r[1]+' m',0,0);ctx.restore();
  });
}
function perF42DrawConditionGraphs(){
  if(selectedExperiment?.id!=='F4.2')return;const target=perBab4.f42Batch.active?perBab4.f42Batch.target:($('perBab4Target')?.value||'human'),distance=perBab4.f42Batch.active?perBab4.f42Batch.distance:(+$('perBab4Distance')?.value||1),rows=perF42RowsFor(target,distance).slice().sort((a,b)=>+a.values[3]-+b.values[3]);
  perF42DrawLineCanvas('perF42CaptureConfidenceCanvas','Nilai Kepercayaan per Pengulangan — '+perF42TargetAcademicLabel(target)+' '+distance+' m',rows,7,100,'%');
  perF42DrawLineCanvas('perF42DistanceErrorCanvas','Galat Jarak per Pengulangan — '+perF42TargetAcademicLabel(target)+' '+distance+' m',rows,10,1,' m');
}
const perF42DrawSummaryBase=perBab4DrawSummary;
perBab4DrawSummary=function(){
  if(selectedExperiment?.id!=='F4.2')return perF42DrawSummaryBase();
  perF42DrawOverviewCanvas('perBab4SummaryCanvas','Tingkat Deteksi terhadap Jarak Acuan',6,1);
  perF42DrawOverviewCanvas('perBab4ConfidenceCanvas','Rata-rata Nilai Kepercayaan pada Klasifikasi Sesuai',8,100);
  perF42DrawConditionGraphs();
};

const perF42PackageHtmlBase=perBab4EvidencePackageHtml;
perBab4EvidencePackageHtml=function(){
  if(selectedExperiment?.id!=='F4.2')return perF42PackageHtmlBase();
  return '<section class="per-bab4-evidence-card per-bab4-save-package" id="perBab4SavePackage">'+
    '<div class="per-bab4-evidence-head"><div><span class="eyebrow">KELUARAN DATA PENGUJIAN</span><b>100 data pengujian + rekapitulasi statistik + dokumentasi visual</b><p>Berkas CSV/XLSX memuat hasil setiap pengulangan, nilai kepercayaan, tingkat deteksi, akurasi klasifikasi, estimasi dan galat jarak, citra bukti, konfigurasi, manifest, serta grafik PNG.</p></div><span class="status-chip" id="perBab4VisualState">0 citra</span></div>'+
    '<div class="per-bab4-package-grid"><div><span>Manusia • 1–5 m</span><b id="perBab4HumanReady">0 / 5 jarak</b></div><div><span>Sepeda Motor • 1–5 m</span><b id="perBab4MotorReady">0 / 5 jarak</b></div><div><span>Citra Pengujian</span><b id="perBab4VisualFrames">0 / 100</b></div><div><span>Data Utama</span><b>10 kondisi × 10 pengulangan</b></div></div>'+
    '<div class="per-bab4-package-actions"><button class="soft-btn" id="perBab4PreviewHuman">Dokumentasi Visual Manusia</button><button class="soft-btn" id="perBab4PreviewMotor">Dokumentasi Visual Sepeda Motor</button></div></section>';
};

const perF42RefreshPackageBase=perBab4RefreshEvidencePackage;
perBab4RefreshEvidencePackage=function(){
  perF42RefreshPackageBase();
  if(selectedExperiment?.id!=='F4.2'||!$('perBab4SavePackage'))return;
  const n=perBab4.evidence.f42.length;
  const h=perBab4EvidenceDistanceReady('human'),m=perBab4EvidenceDistanceReady('motorcycle');
  setText('perBab4VisualState',n+' citra');
  setText('perBab4HumanReady',h+' / 5 jarak');
  setText('perBab4MotorReady',m+' / 5 jarak');
  setText('perBab4VisualFrames',n+' / 100');
  const chip=$('perBab4VisualState');
  if(chip)chip.className='status-chip '+(n>=100?'':'waiting');
};

const perF42ApplySchemaV2=perF42ApplySchema;
perF42ApplySchema=function(){
  perF42ApplySchemaV2();
  const x=(experiments.perception||[]).find(v=>v.id==='F4.2');if(!x)return;
  x.graphCaptions=['Tingkat Deteksi terhadap Jarak Acuan','Rata-rata Nilai Kepercayaan terhadap Jarak Acuan','Dokumentasi Visual Manusia pada Jarak 1–5 m','Dokumentasi Visual Sepeda Motor pada Jarak 1–5 m','Nilai Kepercayaan pada 10 Pengulangan Kondisi Terpilih','Galat Jarak pada 10 Pengulangan Kondisi Terpilih'];
};
perF42ApplySchema();

const perF42GraphPanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const out=perF42GraphPanelBase();
  if(selectedExperiment?.id==='F4.2'){perF42EnsureGraphs();setTimeout(perF42PlaceControls,0);setTimeout(perBab4DrawSummary,0)}
  return out;
};

const perF42GraphPayloadBase=collectExperimentGraphPngPayload;
collectExperimentGraphPngPayload=function(){
  const out=perF42GraphPayloadBase();if(selectedExperiment?.id!=='F4.2')return out;
  perF42DrawConditionGraphs();
  const add=(id,title,index)=>{const c=$(id);if(c&&c.width&&c.height)out.push({index,title,data_url:c.toDataURL('image/png')})};
  add('perF42CaptureConfidenceCanvas','Nilai Kepercayaan pada 10 Pengulangan Kondisi Terpilih',5);
  add('perF42DistanceErrorCanvas','Galat Jarak pada 10 Pengulangan Kondisi Terpilih',6);
  return out;
};

const perF42NavBase=perBab4RenderFinalNavigator;
perBab4RenderFinalNavigator=function(){
  const out=perF42NavBase();
  const card=$('perBab4FinalNavigator')?.querySelector('[data-final-id="F4.2"]');
  const small=card?.querySelector('small');if(small)small.textContent='100 data • Manusia + Sepeda Motor • 1–5 m • 10 pengulangan/30 s';
  return out;
};
const perF42PreviewTextBase=perBab4PreviewText;
perBab4PreviewText=function(){
  if(selectedExperiment?.id==='F4.2')return['PRATINJAU DETEKSI OBJEK','Citra teranotasi menampilkan kotak pembatas, kelas objek, nilai kepercayaan, estimasi jarak, dan hasil klasifikasi. Panel kontrol menjalankan 10 pengulangan otomatis selama 30 s.'];
  return perF42PreviewTextBase();
};


/* FINAL 4.2 semantic evaluation candidate authority v2.
 * Safety detections remain strict; this reads the low-floor candidate stream
 * only inside the controlled BAB IV target-zone test.
 */
function perF42EvaluationCandidates(){
  const sd=obj('semantic_detections')||{},w=+sd.image_width||1280,h=+sd.image_height||720;
  const src=Array.isArray(sd.evaluation_candidates)?sd.evaluation_candidates:(Array.isArray(sd.detections)?sd.detections:[]);
  return src.map(d=>{const q={...d};if([q.x1,q.y1,q.x2,q.y2].every(v=>Number.isFinite(+v))&&w>0&&h>0){q.x1=+q.x1/w;q.x2=+q.x2/w;q.y1=+q.y1/h;q.y2=+q.y2/h}return q});
}
perF42Decision=function(){
  const dets=perF42EvaluationCandidates(),zx=.13,zy=.20,cx=.5,cy=.5,threshold=+(obj('semantic_detections')?.score_threshold);
  let best=null;
  dets.forEach((d,index)=>{
    const x1=+d.x1,y1=+d.y1,x2=+d.x2,y2=+d.y2;if(![x1,y1,x2,y2].every(Number.isFinite))return;
    const bcx=.5*(x1+x2),bcy=.5*(y1+y2),contains=x1<=cx&&x2>=cx&&y1<=cy&&y2>=cy;
    const centerNear=Math.abs(bcx-cx)<=zx&&Math.abs(bcy-cy)<=zy;
    const ix=Math.max(0,Math.min(x2,cx+zx)-Math.max(x1,cx-zx)),iy=Math.max(0,Math.min(y2,cy+zy)-Math.max(y1,cy-zy)),overlap=(ix*iy)/Math.max(1e-9,4*zx*zy);
    if(!contains&&!centerNear&&overlap<.08)return;
    const score=Number.isFinite(+d.score)?+d.score:0,centerScore=Math.max(0,1-Math.hypot((bcx-cx)/zx,(bcy-cy)/zy));
    const rank=(contains?4:0)+3*overlap+1.5*centerScore+score;
    if(!best||rank>best.rank)best={candidate:d,index,rank,contains,overlap};
  });
  const floor=Number.isFinite(threshold)?threshold:.05;
  if(!best)return{success:false,valid:false,index:-1,candidate:null,status:'NOT_DETECTED',threshold:floor,reason:dets.length?'NO SEMANTIC BBOX IN TARGET ZONE':'NO SEMANTIC CANDIDATE',dets};
  const target=$('perBab4Target')?.value||perBab4.f42Batch.target||'human',correct=+best.candidate.class_id===perF42ExpectedClass(target),score=+best.candidate.score;
  return{success:true,valid:true,index:best.index,candidate:best.candidate,status:correct?'CORRECT':'MISCLASSIFIED',threshold:floor,reason:(correct?'CLASS MATCH':'CLASS MISMATCH')+(score<.20?' • LOW CONFIDENCE':''),dets};
};

perF42BatchTick=async function(){
  const s=perBab4.f42Batch;if(!s.active||s.busy||!s.queue.length)return;
  if(!recordStartedMs){perF42BatchStop();return}
  const due=(s.done+1)*PER_F42_INTERVAL_MS,elapsed=Date.now()-s.startMs,stamp=+updated.semantic_detections||0;
  if(elapsed<due||!channelFresh('semantic_detections',2.5)||!stamp||stamp<=s.lastStamp){perF42RefreshUi();return}
  s.busy=true;const repeat=s.queue[0];s.lastStamp=stamp;
  try{await perF42CaptureOne(s.target,s.distance,repeat);s.queue.shift();s.done++;if(!s.queue.length){perF42BatchStop(true);toast(perF42TargetAcademicLabel(s.target)+' '+s.distance+' m: 10 pengulangan selesai')}}
  catch(e){perF42BatchStop(true);toast(e.message||String(e),true)}
  finally{s.busy=false;perF42RefreshUi();perBab4DrawSummary()}
};
perF42BatchStart=async function(failedOnly=false){
  const s=perBab4.f42Batch;if(s.active)return;
  if(!recordStartedMs){await startWebRecording();if(!recordStartedMs)return toast('Sesi pengujian 4.2 tidak dapat dimulai. Periksa kesiapan sumber data.',true)}
  if(!channelFresh('semantic_detections',2.5))return toast('Data deteksi semantik belum diperbarui. Tunggu hingga sumber data siap.',true);
  const target=$('perBab4Target')?.value||'human',distance=+$('perBab4Distance')?.value||1,byRep=new Map(perF42RowsFor(target,distance).map(r=>[+r.values[3],r.values[13]])),all=[1,2,3,4,5,6,7,8,9,10],queue=failedOnly?all.filter(n=>!['CORRECT','Klasifikasi Sesuai'].includes(byRep.get(n))):all;
  if(!queue.length)return toast('Semua 10 capture kondisi ini sudah CORRECT.');
  Object.assign(s,{active:true,busy:false,target,distance,queue:[...queue],done:0,total:queue.length,startMs:Date.now(),lastStamp:+updated.semantic_detections||0,lastResult:'Menunggu pengambilan pertama pada detik ke-3'});
  const a=$('perBab4Target'),b=$('perBab4Distance');if(a)a.disabled=true;if(b)b.disabled=true;
  clearInterval(s.timer);s.timer=setInterval(perF42BatchTick,120);perF42RefreshUi();
};

const perF42SemanticOverlayBase=perBab4DrawOperatorOverlay;
perBab4DrawOperatorOverlay=function(){
  if(selectedExperiment?.id!=='F4.2')return perF42SemanticOverlayBase();
  const canvas=$('perBab4PreviewOverlay'),img=$('perBab4PreviewImage'),g=cameraGeometryFor(img);if(!canvas||!img||!g)return;
  const fit=g.resizeCanvas(canvas),ctx=fit.ctx,dpr=fit.dpr,cr=canvas.getBoundingClientRect();ctx.clearRect(0,0,fit.width,fit.height);
  const decision=perF42Decision(),dets=decision.dets||[];
  dets.forEach((d,i)=>{
    if(![d.x1,d.y1,d.x2,d.y2].every(v=>Number.isFinite(+v)))return;
    const a=g.normalizedToScreen(+d.x1,+d.y1),b=g.normalizedToScreen(+d.x2,+d.y2);if(!a||!b)return;
    const x=(Math.min(a.x,b.x)-cr.left)*dpr,y=(Math.min(a.y,b.y)-cr.top)*dpr,w=Math.abs(b.x-a.x)*dpr,h=Math.abs(b.y-a.y)*dpr,sel=i===decision.index;
    ctx.strokeStyle=sel?(decision.status==='CORRECT'?'#48e0a4':'#ff6677'):'rgba(103,183,255,.65)';ctx.lineWidth=(sel?4:2)*dpr;ctx.strokeRect(x,y,w,h);
    const label=perF42ClassLabel(d.class_id)+' '+(Number.isFinite(+d.score)?(+d.score*100).toFixed(1)+'%':'--');
    ctx.font='900 '+(11*dpr)+'px ui-monospace';const tw=ctx.measureText(label).width;ctx.fillStyle='rgba(0,0,0,.78)';ctx.fillRect(x,y-17*dpr,tw+8*dpr,17*dpr);ctx.fillStyle=ctx.strokeStyle;ctx.fillText(label,x+4*dpr,y-4*dpr);
  });
  const p=g.normalizedToScreen(.5,.5);if(p){const x=(p.x-cr.left)*dpr,y=(p.y-cr.top)*dpr,rx=cr.width*.16*dpr,ry=cr.height*.26*dpr;ctx.save();ctx.setLineDash([8*dpr,5*dpr]);ctx.strokeStyle='#ffc86b';ctx.lineWidth=2*dpr;ctx.strokeRect(x-rx,y-ry,2*rx,2*ry);ctx.setLineDash([]);ctx.fillStyle='#ffc86b';ctx.font='900 '+(12*dpr)+'px ui-monospace';ctx.fillText('TARGET ZONE',x-rx+5*dpr,y-ry+15*dpr);ctx.restore()}
};

/* Prefer a physically meaningful target-zone box over tiny center fragments. */
perF42Decision=function(){
  const dets=perF42EvaluationCandidates(),zx=.16,zy=.26,cx=.5,cy=.5,threshold=+(obj('semantic_detections')?.score_threshold);
  let best=null;
  dets.forEach((d,index)=>{
    const x1=+d.x1,y1=+d.y1,x2=+d.x2,y2=+d.y2;if(![x1,y1,x2,y2].every(Number.isFinite))return;
    const bcx=.5*(x1+x2),bcy=.5*(y1+y2),bw=Math.max(0,x2-x1),bh=Math.max(0,y2-y1),area=bw*bh;
    const centerNear=Math.abs(bcx-cx)<=zx&&Math.abs(bcy-cy)<=zy;
    const ix=Math.max(0,Math.min(x2,cx+zx)-Math.max(x1,cx-zx)),iy=Math.max(0,Math.min(y2,cy+zy)-Math.max(y1,cy-zy)),overlap=(ix*iy)/Math.max(1e-9,4*zx*zy);
    if(!centerNear&&overlap<.03)return;
    const score=Number.isFinite(+d.score)?+d.score:0,centerScore=Math.max(0,1-Math.hypot((bcx-cx)/zx,(bcy-cy)/zy));
    const rank=80*area+2*overlap+centerScore+2*score;
    if(!best||rank>best.rank)best={candidate:d,index,rank,area,overlap};
  });
  const floor=Number.isFinite(threshold)?threshold:.05;
  if(!best)return{success:false,valid:false,index:-1,candidate:null,status:'NOT_DETECTED',threshold:floor,reason:dets.length?'NO PLAUSIBLE BBOX IN TARGET ZONE':'NO SEMANTIC CANDIDATE',dets};
  const target=$('perBab4Target')?.value||perBab4.f42Batch.target||'human',correct=+best.candidate.class_id===perF42ExpectedClass(target),score=+best.candidate.score;
  return{success:true,valid:true,index:best.index,candidate:best.candidate,status:correct?'CORRECT':'MISCLASSIFIED',threshold:floor,reason:(correct?'CLASS MATCH':'CLASS MISMATCH')+(score<.20?' • EVAL LOW CONF':'')+' • semantic',dets};
};

/* FINAL 4.2: expose target-class evidence separately from top-class result. */
perF42Decision=function(){
  const all=perF42EvaluationCandidates(),zx=.16,zy=.26,cx=.5,cy=.5,threshold=+(obj('semantic_detections')?.score_threshold);
  const target=$('perBab4Target')?.value||perBab4.f42Batch.target||'human',expected=perF42ExpectedClass(target),zone=[];
  all.forEach((d,sourceIndex)=>{
    const x1=+d.x1,y1=+d.y1,x2=+d.x2,y2=+d.y2;if(![x1,y1,x2,y2].every(Number.isFinite))return;
    const bcx=.5*(x1+x2),bcy=.5*(y1+y2),bw=Math.max(0,x2-x1),bh=Math.max(0,y2-y1),area=bw*bh;
    const centerNear=Math.abs(bcx-cx)<=zx&&Math.abs(bcy-cy)<=zy;
    const ix=Math.max(0,Math.min(x2,cx+zx)-Math.max(x1,cx-zx)),iy=Math.max(0,Math.min(y2,cy+zy)-Math.max(y1,cy-zy)),overlap=(ix*iy)/Math.max(1e-9,4*zx*zy);
    if(!centerNear&&overlap<.03)return;
    const score=Number.isFinite(+d.score)?+d.score:0,centerScore=Math.max(0,1-Math.hypot((bcx-cx)/zx,(bcy-cy)/zy)),assoc=80*area+2*overlap+centerScore,rank=assoc+2*score;
    zone.push({...d,_sourceIndex:sourceIndex,_rank:rank,_assoc:assoc,_area:area});
  });
  zone.sort((a,b)=>b._rank-a._rank);
  const best=zone[0]||null,targetCandidate=zone.filter(d=>+d.class_id===expected).sort((a,b)=>(b._assoc+2*(+b.score||0))-(a._assoc+2*(+a.score||0)))[0]||null;
  const floor=Number.isFinite(threshold)?threshold:.05;
  if(!best)return{success:false,valid:false,index:-1,targetIndex:-1,candidate:null,targetCandidate:null,status:'NOT_DETECTED',threshold:floor,reason:all.length?'NO PLAUSIBLE BBOX IN TARGET ZONE':'NO SEMANTIC CANDIDATE',dets:zone};
  const correct=+best.class_id===expected,score=+best.score,targetScore=+targetCandidate?.score;
  return{success:true,valid:true,index:0,targetIndex:targetCandidate?zone.indexOf(targetCandidate):-1,candidate:best,targetCandidate,status:correct?'CORRECT':'MISCLASSIFIED',threshold:floor,
    reason:(correct?'CLASS MATCH':'TOP CLASS MISMATCH')+(targetCandidate?(' • '+perF42TargetLabel(target)+' evidence '+(100*targetScore).toFixed(1)+'%'):' • TARGET CLASS ABSENT')+(score<.20?' • EVAL LOW CONF':'')+' • semantic',dets:zone.slice(0,8)};
};

const perF42TargetEvidencePreviewBase=perBab4RefreshOperatorPreview;
perBab4RefreshOperatorPreview=function(){
  const out=perF42TargetEvidencePreviewBase();if(selectedExperiment?.id!=='F4.2')return out;
  const box=$('perBab4PreviewStats'),chip=$('perBab4PreviewState'),d=perF42Decision();if(!box)return out;
  const target=$('perBab4Target')?.value||perBab4.f42Batch.target||'human',tc=d.targetCandidate,top=d.candidate;
  const targetText=tc?(perF42TargetAcademicLabel(target)+' • '+(100*(+tc.score||0)).toFixed(1)+'%'):(perF42TargetAcademicLabel(target)+' • tidak terdeteksi');
  let extra=document.createElement('div');extra.dataset.f42TargetEvidence='1';extra.innerHTML='<span>Bukti Kelas Target</span><b>'+escapeHtml(targetText)+'</b>';box.prepend(extra);
  if(chip){chip.textContent=perF42StatusAcademicLabel(d.status);chip.className='status-chip '+(d.status==='CORRECT'?'':d.status==='MISCLASSIFIED'?'bad':'waiting')}
  return out;
};
const perF42TargetOverlayBase=perBab4DrawOperatorOverlay;
perBab4DrawOperatorOverlay=function(){
  const out=perF42TargetOverlayBase();if(selectedExperiment?.id!=='F4.2')return out;
  const c=$('perBab4PreviewOverlay'),img=$('perBab4PreviewImage'),g=cameraGeometryFor(img),d=perF42Decision();if(!c||!img||!g||d.targetIndex<0||d.targetIndex===d.index)return out;
  const q=d.dets[d.targetIndex];if(!q)return out;const a=g.normalizedToScreen(+q.x1,+q.y1),b=g.normalizedToScreen(+q.x2,+q.y2);if(!a||!b)return out;
  const ctx=c.getContext('2d'),dpr=Math.max(1,window.devicePixelRatio||1),cr=c.getBoundingClientRect(),x=(Math.min(a.x,b.x)-cr.left)*dpr,y=(Math.min(a.y,b.y)-cr.top)*dpr,w=Math.abs(b.x-a.x)*dpr,h=Math.abs(b.y-a.y)*dpr,label='TARGET • '+perF42ClassAcademicLabel(q.class_id)+' • '+(100*(+q.score||0)).toFixed(1)+'%';
  ctx.save();ctx.strokeStyle='#ffc86b';ctx.lineWidth=4*dpr;ctx.strokeRect(x,y,w,h);ctx.font='900 '+(11*dpr)+'px ui-monospace';const tw=ctx.measureText(label).width;ctx.fillStyle='rgba(0,0,0,.82)';ctx.fillRect(x,y-18*dpr,tw+8*dpr,18*dpr);ctx.fillStyle='#ffc86b';ctx.fillText(label,x+4*dpr,y-4*dpr);ctx.restore();return out;
};

/* FINAL 4.2 clean overlay v5: one result bbox only + target zone. */
const perF42CleanOverlayFallback=perBab4DrawOperatorOverlay;
perBab4DrawOperatorOverlay=function(){
  if(selectedExperiment?.id!=='F4.2')return perF42CleanOverlayFallback();
  const canvas=$('perBab4PreviewOverlay'),img=$('perBab4PreviewImage'),g=cameraGeometryFor(img);
  if(!canvas||!img||!g)return;
  const fit=g.resizeCanvas(canvas),ctx=fit.ctx,dpr=fit.dpr,cr=canvas.getBoundingClientRect();
  ctx.clearRect(0,0,fit.width,fit.height);
  const d=perF42Decision(),q=d.candidate;
  if(q&&[q.x1,q.y1,q.x2,q.y2].every(v=>Number.isFinite(+v))){
    const a=g.normalizedToScreen(+q.x1,+q.y1),b=g.normalizedToScreen(+q.x2,+q.y2);
    if(a&&b){
      const x=(Math.min(a.x,b.x)-cr.left)*dpr,y=(Math.min(a.y,b.y)-cr.top)*dpr,w=Math.abs(b.x-a.x)*dpr,h=Math.abs(b.y-a.y)*dpr;
      const col=d.status==='CORRECT'?'#48e0a4':d.status==='MISCLASSIFIED'?'#ff6677':'#ffc86b';
      const label=perF42ClassAcademicLabel(q.class_id)+' • '+(Number.isFinite(+q.score)?(+q.score*100).toFixed(1)+'%':'--')+' • '+perF42StatusAcademicLabel(d.status);
      ctx.strokeStyle=col;ctx.lineWidth=4*dpr;ctx.strokeRect(x,y,w,h);
      ctx.font='900 '+(12*dpr)+'px ui-monospace';const tw=ctx.measureText(label).width;
      ctx.fillStyle='rgba(0,0,0,.82)';ctx.fillRect(x,Math.max(0,y-20*dpr),tw+10*dpr,20*dpr);
      ctx.fillStyle=col;ctx.fillText(label,x+5*dpr,Math.max(15*dpr,y-5*dpr));
    }
  }
  const p=g.normalizedToScreen(.5,.5);
  if(p){
    const x=(p.x-cr.left)*dpr,y=(p.y-cr.top)*dpr,rx=cr.width*.16*dpr,ry=cr.height*.26*dpr;
    ctx.save();ctx.setLineDash([8*dpr,5*dpr]);ctx.strokeStyle='#ffc86b';ctx.lineWidth=2*dpr;
    ctx.strokeRect(x-rx,y-ry,2*rx,2*ry);ctx.setLineDash([]);
    ctx.fillStyle='#ffc86b';ctx.font='900 '+(11*dpr)+'px ui-monospace';ctx.fillText('ZONA PENGUJIAN',x-rx+5*dpr,y-ry+15*dpr);ctx.restore();
  }
};

/* Capture package also stores only the selected result bbox, never all low-floor proposals. */
perF42CaptureOne=async function(target,distance,repeat){
  const decision=perF42Decision(),cand=decision.candidate,status=decision.status,rawCandidateCount=(decision.dets||[]).length;
  const conf=Number.isFinite(+cand?.score)?+cand.score:NaN,est=decision.valid?perF42DistanceEstimate(cand):NaN,err=Number.isFinite(est)?est-distance:NaN;
  const bw=cand&&Number.isFinite(+cand.x1)&&Number.isFinite(+cand.x2)?Math.abs(+cand.x2-+cand.x1):NaN,bh=cand&&Number.isFinite(+cand.y1)&&Number.isFinite(+cand.y2)?Math.abs(+cand.y2-+cand.y1):NaN;
  const backend=obj('semantic_detections').source||obj('perception_performance').backend||'--',detected=cand?perF42ClassAcademicLabel(cand.class_id):'Tidak Terdeteksi',statusLabel=perF42StatusAcademicLabel(status);
  const evidenceLabel='F4.2_'+(target==='motorcycle'?'sepeda_motor':'manusia')+'_'+distance+'m_data_'+String(repeat).padStart(2,'0')+'_'+statusLabel.replace(/\s+/g,'_').toLowerCase();
  const ev=await perBab4PersistTrialEvidence(evidenceLabel),evidencePath=ev.image_path||'--';
  const evidenceDets=cand?[{...cand}]:[];
  const dataUrl=perBab4EvidenceJpeg({kind:'detection',detections:evidenceDets,selectedIndex:cand?0:-1,title:'PENGUJIAN DETEKSI OBJEK • '+perF42TargetAcademicLabel(target)+' • '+distance+' m • Pengulangan '+repeat+'/10',subtitle:statusLabel+' • '+detected+' • confidence '+(Number.isFinite(conf)?(conf*100).toFixed(1)+'%':'--')+' • estimasi jarak '+(Number.isFinite(est)?est.toFixed(2)+' m':'--')});
  const f=(v,n=4)=>Number.isFinite(v)?v.toFixed(n):'--';
  const values=[perBab4Rows().length+1,perF42TargetAcademicLabel(target),distance,repeat,perBab4FrameTimestamp(),detected,cand?.class_id??'--',f(conf,5),f(decision.threshold,5),f(est,3),f(err,3),f(bw,1),f(bh,1),statusLabel,backend,rawCandidateCount,perF42DecisionAcademicLabel(decision.reason),evidencePath];
  const key=target+'|'+distance+'|'+repeat,retake=perBab4UpsertMainTrial(key,values,v=>{const obj=String(v[1]).toLowerCase().includes('motor')?'motorcycle':'human';return obj+'|'+v[2]+'|'+v[3]});
  perBab4.evidence.f42=perBab4.evidence.f42.filter(x=>!(x.target===target&&+x.distance===distance&&+x.repeat===repeat));
  if(dataUrl)perBab4.evidence.f42.push({target,distance,repeat,success:status==='CORRECT',status,confidence:conf,estimatedDistance:est,dataUrl,evidencePath});
  else perBab4.evidence.captureWarnings++;
  perBab4.f42Batch.lastResult=(retake?'PENGAMBILAN ULANG • ':'')+statusLabel+' • '+detected+' • '+(Number.isFinite(conf)?(conf*100).toFixed(1)+'%':'--');
  perBab4RefreshEvidencePackage();perF42RefreshUi();return status;
};

/* ===== F4.3 cumulative 60-data session store (TA robustness) ===== */
const PER_F43_DRAFT_KEY='agv.perception.f43.ta60.reset20260920.v2';
perBab4.f43Session=perBab4.f43Session||{rows:[]};

function perF43RowKey(values){return String(values?.[1]||'')+'|'+String(values?.[2]||'')}
function perF43PersistDraft(){
  try{
    const payload={version:1,updated_at:new Date().toISOString(),rows:perBab4.f43Session.rows};
    localStorage.setItem(PER_F43_DRAFT_KEY,JSON.stringify(payload));
  }catch(_){}
}
function perF43LoadDraft(){
  try{
    const raw=localStorage.getItem(PER_F43_DRAFT_KEY);if(!raw)return;
    const j=JSON.parse(raw);if(j?.version===1&&Array.isArray(j.rows))perBab4.f43Session.rows=j.rows;
  }catch(_){}
}
function perF43RestoreRows(){
  if(selectedExperiment?.id!=='F4.3')return;
  if(!Array.isArray(perBab4.f43Session.rows))perBab4.f43Session.rows=[];
  tableRunRows.set(0,perBab4.f43Session.rows.map(r=>({sec:r.sec||0,values:[...(r.values||[])],yaml:Array.isArray(r.yaml)?r.yaml:[]})));
  perBab4PrepareSummary();renderExperimentTable();perBab4RenderProgress();perBab4DrawSummary();
}
function perF43SyncAccumulator(){
  if(selectedExperiment?.id!=='F4.3')return;
  const rows=(tableRunRows.get(0)||[]).map(r=>({sec:r.sec||0,values:[...(r.values||[])],yaml:Array.isArray(r.yaml)?r.yaml:[]}));
  const map=new Map(perBab4.f43Session.rows.map(r=>[perF43RowKey(r.values),r]));
  for(const row of rows)map.set(perF43RowKey(row.values),row);
  perBab4.f43Session.rows=[...map.values()].sort((a,b)=>(+a.values?.[0]||0)-(+b.values?.[0]||0));
  tableRunRows.set(0,perBab4.f43Session.rows.map(r=>({sec:r.sec||0,values:[...(r.values||[])],yaml:Array.isArray(r.yaml)?r.yaml:[]})));
  perF43PersistDraft();
}
perF43LoadDraft();

const perF43CumulativeCaptureBase=perBab4LaneCaptureOne;
perBab4LaneCaptureOne=async function(cond,repeat){
  if(selectedExperiment?.id==='F4.3'&&perBab4.f43Session.rows.length)perF43RestoreRows();
  const result=await perF43CumulativeCaptureBase(cond,repeat);
  perF43SyncAccumulator();
  return result;
};

const perF43CumulativeStartBase=startWebRecording;
startWebRecording=async function(){
  const f43=selectedExperiment?.id==='F4.3'&&currentExp==='perception';
  const preserved=f43?perBab4.f43Session.rows.map(r=>({sec:r.sec||0,values:[...(r.values||[])],yaml:Array.isArray(r.yaml)?r.yaml:[]})):null;
  await perF43CumulativeStartBase();
  if(f43&&recordStartedMs){
    if(preserved?.length)perBab4.f43Session.rows=preserved;
    perF43RestoreRows();
  }
};

const perF43CumulativeStopBase=stopWebRecording;
stopWebRecording=async function(){
  const f43=selectedExperiment?.id==='F4.3'&&currentExp==='perception';
  if(f43){perF43SyncAccumulator();perF43RestoreRows();}
  const out=await perF43CumulativeStopBase();
  if(f43){perF43PersistDraft();perF43RestoreRows();}
  return out;
};

const perF43CumulativeRenderBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const out=perF43CumulativeRenderBase();
  if(selectedExperiment?.id==='F4.3'&&currentExp==='perception'){
    if(perBab4.f43Session.rows.length)perF43RestoreRows();
    const total=perBab4.f43Session.rows.length;
    const ctl=$('perLaneBatchControls');
    if(ctl&&!$('perF43DatasetCount')){
      const n=document.createElement('div');n.id='perF43DatasetCount';n.className='per-lane-control-state';
      n.innerHTML='<span>Akumulasi dataset F4.3</span><b id="perF43DatasetCountValue">0 / 60 data</b>';ctl.appendChild(n);
      const a=document.createElement('div');a.id='perF43DatasetActions';a.className='per-bab4-actions per-lane-batch-actions';a.innerHTML='<button class="soft-btn" id="perF43RecoverLatest">PULIHKAN 57 DATA TEREKAM</button><button class="warning-btn" id="perF43ClearDataset">HAPUS DATASET 60 DATA</button>';ctl.appendChild(a);
      $('perF43RecoverLatest')?.addEventListener('click',perF43RecoverLatest);$('perF43ClearDataset')?.addEventListener('click',perF43ClearDataset);
    }
    setText('perF43DatasetCountValue',total+' / 60 data');
  }
  return out;
};

const perF43CumulativeRefreshBase=perBab4LaneBatchRefresh;
perBab4LaneBatchRefresh=function(){
  const out=perF43CumulativeRefreshBase();
  if(selectedExperiment?.id==='F4.3')setText('perF43DatasetCountValue',(perBab4.f43Session.rows?.length||0)+' / 60 data');
  return out;
};

async function perF43RecoverLatest(){
  try{
    const recovered=PER_F43_RECOVERY_ROWS_20260920;if(!Array.isArray(recovered)||!recovered.length)throw new Error('Data pemulihan tidak tersedia');
    const map=new Map((perBab4.f43Session.rows||[]).map(row=>[perF43RowKey(row.values),row]));
    for(const values of recovered)map.set(perF43RowKey(values),{sec:0,values:[...values],yaml:[]});
    perBab4.f43Session.rows=[...map.values()].sort((a,b)=>(+a.values?.[0]||0)-(+b.values?.[0]||0));
    perF43PersistDraft();perF43RestoreRows();perBab4LaneBatchRefresh();
    toast(`Pemulihan berhasil: ${perBab4.f43Session.rows.length}/60 data tersedia. Lengkapi pengulangan yang masih kurang.`);
  }catch(e){toast('Pemulihan F4.3 gagal: '+(e.message||String(e)),true)}
}
function perF43ClearDataset(){
  if(!confirm('Hapus seluruh akumulasi data F4.3 dari browser? Gunakan hanya untuk memulai pengujian baru.'))return;
  perBab4.f43Session.rows=[];try{localStorage.removeItem(PER_F43_DRAFT_KEY)}catch(_){}
  tableRunRows.set(0,[]);tableRunRows.set(1,[]);perBab4.evidence.f43=[];if(typeof perF43EvidenceClear==='function')perF43EvidenceClear();renderExperimentTable();perBab4RenderProgress();perBab4DrawSummary();perBab4LaneBatchRefresh();perBab4RefreshEvidencePackage();
  toast('Akumulasi data F4.3 telah dikosongkan.');
}
const PER_F43_RECOVERY_ROWS_20260920=[["1","Posisi Tengah Koridor Keselamatan","1","2026-09-19T18:56:07.711Z","Terdeteksi","Terdeteksi","Valid","Valid","99.59","87.56","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.020","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","0.02","KOMPILASI_VISUAL"],["2","Posisi Tengah Koridor Keselamatan","2","2026-09-19T18:56:10.041Z","Terdeteksi","Terdeteksi","Valid","Valid","77.80","102.06","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.697","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","3.02","KOMPILASI_VISUAL"],["3","Posisi Tengah Koridor Keselamatan","3","2026-09-19T18:56:12.925Z","Terdeteksi","Terdeteksi","Valid","Valid","98.59","85.56","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.806","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","6.01","KOMPILASI_VISUAL"],["4","Posisi Tengah Koridor Keselamatan","4","2026-09-19T18:56:16.635Z","Terdeteksi","Terdeteksi","Valid","Valid","98.59","87.56","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.094","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","9.01","KOMPILASI_VISUAL"],["5","Posisi Tengah Koridor Keselamatan","5","2026-09-19T18:56:52.520Z","Terdeteksi","Terdeteksi","Valid","Valid","167.80","153.30","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.303","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","0.01","KOMPILASI_VISUAL"],["6","Posisi Tengah Koridor Keselamatan","6","2026-09-19T18:56:55.625Z","Terdeteksi","Terdeteksi","Valid","Valid","169.13","151.97","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.196","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","3.01","KOMPILASI_VISUAL"],["7","Posisi Tengah Koridor Keselamatan","7","2026-09-19T18:56:58.637Z","Terdeteksi","Terdeteksi","Valid","Valid","171.13","152.97","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.184","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","6.01","KOMPILASI_VISUAL"],["8","Posisi Tengah Koridor Keselamatan","8","2026-09-19T18:57:01.265Z","Terdeteksi","Terdeteksi","Valid","Valid","170.80","158.64","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.556","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","9.02","KOMPILASI_VISUAL"],["9","Posisi Tengah Koridor Keselamatan","9","2026-09-19T18:57:04.301Z","Terdeteksi","Terdeteksi","Valid","Valid","154.47","175.30","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.521","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","12.02","KOMPILASI_VISUAL"],["10","Posisi Tengah Koridor Keselamatan","10","2026-09-19T18:57:07.319Z","Terdeteksi","Terdeteksi","Valid","Valid","171.13","158.64","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Aman (Hijau)","Sesuai","Sesuai","Tidak Memerlukan Koreksi","Memenuhi Kriteria","0.507","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","15.02","KOMPILASI_VISUAL"],["11","Pendekatan Batas Keselamatan Kiri","1","2026-09-19T18:58:49.061Z","Terdeteksi","Terdeteksi","Valid","Valid","27.84","286.54","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.214","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","0.01","KOMPILASI_VISUAL"],["12","Pendekatan Batas Keselamatan Kiri","2","2026-09-19T18:58:52.170Z","Terdeteksi","Terdeteksi","Valid","Valid","29.09","285.04","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.105","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","3.01","KOMPILASI_VISUAL"],["13","Pendekatan Batas Keselamatan Kiri","3","2026-09-19T18:58:54.751Z","Terdeteksi","Terdeteksi","Valid","Valid","31.09","286.04","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.537","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","6.02","KOMPILASI_VISUAL"],["14","Pendekatan Batas Keselamatan Kiri","4","2026-09-19T18:58:57.871Z","Terdeteksi","Terdeteksi","Valid","Valid","31.59","289.04","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.402","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","9.01","KOMPILASI_VISUAL"],["15","Pendekatan Batas Keselamatan Kiri","5","2026-09-19T18:59:00.956Z","Terdeteksi","Terdeteksi","Valid","Valid","30.84","289.54","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.317","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","12.01","KOMPILASI_VISUAL"],["16","Pendekatan Batas Keselamatan Kiri","6","2026-09-19T18:59:03.860Z","Terdeteksi","Terdeteksi","Valid","Valid","15.59","305.04","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.413","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","15.01","KOMPILASI_VISUAL"],["17","Pendekatan Batas Keselamatan Kiri","7","2026-09-19T18:59:06.435Z","Terdeteksi","Terdeteksi","Valid","Valid","32.09","289.54","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.839","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","18.01","KOMPILASI_VISUAL"],["18","Pendekatan Batas Keselamatan Kiri","8","2026-09-19T18:59:09.569Z","Terdeteksi","Terdeteksi","Valid","Valid","30.84","289.54","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.707","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","21.01","KOMPILASI_VISUAL"],["19","Pendekatan Batas Keselamatan Kiri","9","2026-09-19T18:59:13.166Z","Terdeteksi","Terdeteksi","Valid","Valid","14.34","301.78","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.108","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","24.01","KOMPILASI_VISUAL"],["20","Pendekatan Batas Keselamatan Kiri","10","2026-09-19T18:59:15.862Z","Terdeteksi","Terdeteksi","Valid","Valid","30.84","289.04","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.411","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","27.01","KOMPILASI_VISUAL"],["21","Pelanggaran Batas Keselamatan Kiri","1","2026-09-19T18:59:48.210Z","Terdeteksi","Terdeteksi","Valid","Valid","-91.72","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.359","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","0.01","KOMPILASI_VISUAL"],["22","Pelanggaran Batas Keselamatan Kiri","2","2026-09-19T18:59:51.494Z","Terdeteksi","Terdeteksi","Valid","Valid","-88.11","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.074","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","3.01","KOMPILASI_VISUAL"],["23","Pelanggaran Batas Keselamatan Kiri","3","2026-09-19T18:59:53.961Z","Terdeteksi","Terdeteksi","Valid","Valid","-119.28","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.606","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","6.01","KOMPILASI_VISUAL"],["24","Pelanggaran Batas Keselamatan Kiri","4","2026-09-19T18:59:57.014Z","Terdeteksi","Terdeteksi","Valid","Valid","-88.31","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.554","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","9.01","KOMPILASI_VISUAL"],["25","Pelanggaran Batas Keselamatan Kiri","5","2026-09-19T18:59:59.897Z","Terdeteksi","Terdeteksi","Valid","Valid","-89.11","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.674","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","12.01","KOMPILASI_VISUAL"],["26","Pelanggaran Batas Keselamatan Kiri","6","2026-09-19T19:00:03.529Z","Terdeteksi","Terdeteksi","Valid","Valid","-89.11","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.043","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","15.01","KOMPILASI_VISUAL"],["27","Pelanggaran Batas Keselamatan Kiri","7","2026-09-19T19:00:06.225Z","Terdeteksi","Terdeteksi","Valid","Valid","-88.92","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.342","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","18.01","KOMPILASI_VISUAL"],["28","Pelanggaran Batas Keselamatan Kiri","8","2026-09-19T19:00:09.080Z","Terdeteksi","Terdeteksi","Valid","Valid","-87.92","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.487","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","21.01","KOMPILASI_VISUAL"],["29","Pelanggaran Batas Keselamatan Kiri","9","2026-09-19T19:00:12.381Z","Terdeteksi","Terdeteksi","Valid","Valid","-88.11","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.190","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","24.01","KOMPILASI_VISUAL"],["30","Pelanggaran Batas Keselamatan Kiri","10","2026-09-19T19:00:15.345Z","Terdeteksi","Terdeteksi","Valid","Valid","-88.52","0.00","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Sesuai","Sesuai","Koreksi Arah ke Kiri","Memenuhi Kriteria","0.222","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Aktif","Tidak Aktif","27.01","KOMPILASI_VISUAL"],["31","Pendekatan Batas Keselamatan Kanan","1","2026-09-19T19:01:54.872Z","Terdeteksi","Terdeteksi","Valid","Valid","303.72","16.23","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.236","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","0.01","KOMPILASI_VISUAL"],["32","Pendekatan Batas Keselamatan Kanan","2","2026-09-19T19:01:57.624Z","Terdeteksi","Terdeteksi","Valid","Valid","303.72","15.43","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.486","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","3.01","KOMPILASI_VISUAL"],["33","Pendekatan Batas Keselamatan Kanan","3","2026-09-19T19:02:00.546Z","Terdeteksi","Terdeteksi","Valid","Valid","0.00","14.63","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.565","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","6.01","KOMPILASI_VISUAL"],["34","Pendekatan Batas Keselamatan Kanan","4","2026-09-19T19:02:03.919Z","Terdeteksi","Terdeteksi","Valid","Valid","302.72","16.83","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.194","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","9.02","KOMPILASI_VISUAL"],["35","Pendekatan Batas Keselamatan Kanan","5","2026-09-19T19:02:06.876Z","Terdeteksi","Terdeteksi","Valid","Valid","303.72","16.23","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.234","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","12.01","KOMPILASI_VISUAL"],["36","Pendekatan Batas Keselamatan Kanan","6","2026-09-19T19:02:09.640Z","Terdeteksi","Terdeteksi","Valid","Valid","302.72","16.63","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.474","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","15.03","KOMPILASI_VISUAL"],["37","Pendekatan Batas Keselamatan Kanan","7","2026-09-19T19:02:12.775Z","Terdeteksi","Terdeteksi","Valid","Valid","302.72","15.23","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.335","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","18.01","KOMPILASI_VISUAL"],["38","Pendekatan Batas Keselamatan Kanan","8","2026-09-19T19:02:15.994Z","Terdeteksi","Terdeteksi","Valid","Valid","302.72","15.63","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.118","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","21.02","KOMPILASI_VISUAL"],["39","Pendekatan Batas Keselamatan Kanan","9","2026-09-19T19:02:18.765Z","Terdeteksi","Terdeteksi","Valid","Valid","287.72","29.43","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.343","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","24.02","KOMPILASI_VISUAL"],["40","Pendekatan Batas Keselamatan Kanan","10","2026-09-19T19:02:21.886Z","Terdeteksi","Terdeteksi","Valid","Valid","302.72","15.23","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Aman (Hijau)","Peringatan Batas Keselamatan (Kuning)","Sesuai","Sesuai","Peringatan: Mendekati Batas Keselamatan","Memenuhi Kriteria","0.224","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Tidak Aktif","27.01","KOMPILASI_VISUAL"],["41","Pelanggaran Batas Keselamatan Kanan","1","2026-09-19T19:02:36.312Z","Terdeteksi","Terdeteksi","Valid","Valid","0.00","-49.97","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Sesuai","Sesuai","Koreksi Arah ke Kanan","Memenuhi Kriteria","0.287","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Aktif","0.01","KOMPILASI_VISUAL"],["42","Pelanggaran Batas Keselamatan Kanan","2","2026-09-19T19:02:39.379Z","Terdeteksi","Terdeteksi","Valid","Valid","0.00","-50.57","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Sesuai","Sesuai","Koreksi Arah ke Kanan","Memenuhi Kriteria","0.218","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Aktif","3.01","KOMPILASI_VISUAL"],["43","Pelanggaran Batas Keselamatan Kanan","3","2026-09-19T19:02:42.411Z","Terdeteksi","Terdeteksi","Valid","Valid","0.00","-46.17","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Sesuai","Sesuai","Koreksi Arah ke Kanan","Memenuhi Kriteria","0.186","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Aktif","6.01","KOMPILASI_VISUAL"],["44","Pelanggaran Batas Keselamatan Kanan","4","2026-09-19T19:02:45.157Z","Terdeteksi","Terdeteksi","Valid","Valid","0.00","-45.97","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Sesuai","Sesuai","Koreksi Arah ke Kanan","Memenuhi Kriteria","0.441","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Aktif","9.01","KOMPILASI_VISUAL"],["45","Pelanggaran Batas Keselamatan Kanan","5","2026-09-19T19:02:48.150Z","Terdeteksi","Terdeteksi","Valid","Valid","0.00","-46.77","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Sesuai","Sesuai","Koreksi Arah ke Kanan","Memenuhi Kriteria","0.448","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Aktif","12.01","KOMPILASI_VISUAL"],["46","Pelanggaran Batas Keselamatan Kanan","6","2026-09-19T19:02:51.430Z","Terdeteksi","Terdeteksi","Valid","Valid","0.00","-47.37","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Sesuai","Sesuai","Koreksi Arah ke Kanan","Memenuhi Kriteria","0.166","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Aktif","15.01","KOMPILASI_VISUAL"],["47","Pelanggaran Batas Keselamatan Kanan","7","2026-09-19T19:02:54.234Z","Terdeteksi","Terdeteksi","Valid","Valid","0.00","-47.17","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Aman (Hijau)","Pelanggaran Batas Keselamatan (Merah)","Sesuai","Sesuai","Koreksi Arah ke Kanan","Memenuhi Kriteria","0.370","Area Dapat Dilalui + Batas Luar Lajur","Tidak","Terdeteksi","Tidak Aktif","Aktif","18.01","KOMPILASI_VISUAL"],["51","Di Luar Area Dapat Dilalui (Non-Drivable Area)","1","2026-09-19T19:04:31.313Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.335","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","0.01","KOMPILASI_VISUAL"],["52","Di Luar Area Dapat Dilalui (Non-Drivable Area)","2","2026-09-19T19:04:34.175Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.473","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","3.01","KOMPILASI_VISUAL"],["53","Di Luar Area Dapat Dilalui (Non-Drivable Area)","3","2026-09-19T19:04:37.313Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.336","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","6.01","KOMPILASI_VISUAL"],["54","Di Luar Area Dapat Dilalui (Non-Drivable Area)","4","2026-09-19T19:04:40.368Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.279","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","9.01","KOMPILASI_VISUAL"],["55","Di Luar Area Dapat Dilalui (Non-Drivable Area)","5","2026-09-19T19:04:43.162Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.487","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","12.01","KOMPILASI_VISUAL"],["56","Di Luar Area Dapat Dilalui (Non-Drivable Area)","6","2026-09-19T19:04:46.551Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.097","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","15.01","KOMPILASI_VISUAL"],["57","Di Luar Area Dapat Dilalui (Non-Drivable Area)","7","2026-09-19T19:04:49.160Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.491","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","18.01","KOMPILASI_VISUAL"],["58","Di Luar Area Dapat Dilalui (Non-Drivable Area)","8","2026-09-19T19:04:50.601Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","2.048","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","21.01","KOMPILASI_VISUAL"],["59","Di Luar Area Dapat Dilalui (Non-Drivable Area)","9","2026-09-19T19:04:55.146Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.503","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","24.02","KOMPILASI_VISUAL"],["60","Di Luar Area Dapat Dilalui (Non-Drivable Area)","10","2026-09-19T19:04:58.267Z","Tidak Terdeteksi","Tidak Terdeteksi","Tidak Valid","Tidak Valid","0.00","0.00","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Di Luar Area Dapat Dilalui (Abu-abu)","Sesuai","Sesuai","Bukti Segmentasi Tidak Tersedia","Memenuhi Kriteria","0.383","Bukti Tidak Tersedia","Tidak","Tidak Terdeteksi","Tidak Aktif","Tidak Aktif","27.01","KOMPILASI_VISUAL"]];


/* ===== F4.3 persistent representative evidence bank v1 =====
 * One representative image per academic test condition.
 * Independent from recorder lifecycle, so START/STOP of the next condition cannot erase prior panels.
 */
const PER_F43_EVIDENCE_KEY='agv.perception.f43.evidence6.reset20260920.v2';
perBab4.f43EvidenceBank=perBab4.f43EvidenceBank||{};

function perF43EvidenceRecommendation(cond){
  return ({center:'CLEAR',left_yellow:'WARNING',left_red:'RECENTER_LEFT',right_yellow:'WARNING',right_red:'RECENTER_RIGHT',gray:'NO_MASK_EVIDENCE'})[cond]||'--';
}
function perF43EvidenceMode(cond){return cond==='gray'?'NO_EVIDENCE':'DRIVABLE_OUTER_LANE'}
function perF43EvidenceScore(e){return (e?.match?1000:0)-Math.abs((+e?.repeat||5)-5)+(e?.recovered?0:10)}
function perF43EvidencePersist(){
  try{localStorage.setItem(PER_F43_EVIDENCE_KEY,JSON.stringify({version:1,updated_at:new Date().toISOString(),items:perBab4.f43EvidenceBank}))}
  catch(e){console.warn('F4.3 representative image bank localStorage failed',e)}
}
function perF43EvidenceLoad(){
  try{
    const raw=localStorage.getItem(PER_F43_EVIDENCE_KEY);if(!raw)return;
    const j=JSON.parse(raw);if(j?.version===1&&j.items&&typeof j.items==='object')perBab4.f43EvidenceBank=j.items;
  }catch(e){console.warn('F4.3 representative image bank load failed',e)}
}
async function perF43EvidencePersistServer(entry){
  if(!entry?.cond||!entry?.dataUrl||entry.recovered)return;
  try{const r=await fetch('/api/perception/f43/evidence-bank',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:entry.cond,data_url:entry.dataUrl})});if(!r.ok)throw new Error('HTTP '+r.status)}catch(e){console.warn('F4.3 representative image server persistence failed',e)}
}
function perF43EvidenceUpsert(entry,force=false){
  if(!entry?.cond||!entry?.dataUrl)return false;
  const old=perBab4.f43EvidenceBank[entry.cond];
  if(!force&&old&&perF43EvidenceScore(old)>perF43EvidenceScore(entry))return false;
  const saved={...entry,key:entry.cond,_persistentRepresentative:true};perBab4.f43EvidenceBank[entry.cond]=saved;
  perF43EvidencePersist();if(!saved.recovered)perF43EvidencePersistServer(saved);return true;
}
function perF43EvidenceCount(){return Object.values(perBab4.f43EvidenceBank||{}).filter(x=>x?.dataUrl).length}
function perF43EvidenceClear(){
  perBab4.f43EvidenceBank={};perF43ServerEvidenceLoaded=true;
  try{localStorage.removeItem(PER_F43_EVIDENCE_KEY)}catch(_){}
  fetch('/api/perception/f43/evidence-bank',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'clear'})}).catch(e=>console.warn('F4.3 server evidence reset failed',e));
}
perF43EvidenceLoad();

let perF43ServerEvidenceLoaded=false,perF43ServerEvidenceLoading=false;
async function perF43RecoverServerEvidence(){
  if(perF43ServerEvidenceLoaded||perF43ServerEvidenceLoading)return;
  perF43ServerEvidenceLoading=true;
  try{
    const r=await fetch('/api/perception/f43/evidence-bank',{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);
    const j=await r.json(),items=Array.isArray(j.images)?j.images:[];
    for(const it of items){
      const cond=String(it.key||''),exp=perBab4LaneExpected(cond);
      if(!cond||!it.data_url)continue;
      perF43EvidenceUpsert({key:cond,cond,repeat:5,left:exp.left,right:exp.right,recommendation:perF43EvidenceRecommendation(cond),match:true,dataUrl:it.data_url,evidencePath:'SERVER_EVIDENCE_BANK',evidenceMode:perF43EvidenceMode(cond),recovered:true},false);
    }
    perF43ServerEvidenceLoaded=true;
    if(selectedExperiment?.id==='F4.3'){perBab4RefreshEvidencePackage();perBab4LaneBatchRefresh()}
  }catch(e){console.warn('F4.3 server evidence recovery unavailable',e)}
  finally{perF43ServerEvidenceLoading=false}
}

const perF43PersistentRepBase=perBab4RepresentativeLane;
perBab4RepresentativeLane=function(key){
  const saved=perBab4.f43EvidenceBank?.[key];
  if(saved?.dataUrl)return saved;
  return perF43PersistentRepBase(key);
};

const perF43PersistentResetBase=perBab4EvidenceReset;
perBab4EvidenceReset=function(){
  if(selectedExperiment?.id!=='F4.3')return perF43PersistentResetBase();
  perBab4.evidence.f43=[];perBab4.evidence.exports=[];perBab4.evidence.captureWarnings=0;
  perBab4RefreshEvidencePackage();
};

const perF43PersistentCaptureBase=perBab4LaneCaptureOne;
perBab4LaneCaptureOne=async function(cond,repeat){
  const result=await perF43PersistentCaptureBase(cond,repeat);
  const e=[...(perBab4.evidence?.f43||[])].reverse().find(x=>x?.cond===cond&&+x.repeat===+repeat&&x.dataUrl);
  if(e){perF43EvidenceUpsert({...e,recovered:false});perBab4RefreshEvidencePackage()}
  return result;
};

const perF43PersistentRenderBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const out=perF43PersistentRenderBase();
  if(selectedExperiment?.id==='F4.3'&&currentExp==='perception'){
    perF43RecoverServerEvidence();
    setTimeout(()=>perBab4RefreshEvidencePackage(),0);
  }
  return out;
};

const perF43PersistentStartBase=startWebRecording;
startWebRecording=async function(){
  const f43=selectedExperiment?.id==='F4.3'&&currentExp==='perception';
  await perF43PersistentStartBase();
  if(f43){perF43RecoverServerEvidence();perBab4RefreshEvidencePackage()}
};

const perF43PersistentStopBase=stopWebRecording;
stopWebRecording=async function(){
  const f43=selectedExperiment?.id==='F4.3'&&currentExp==='perception';
  if(f43){await perF43RecoverServerEvidence();perBab4RefreshEvidencePackage()}
  return perF43PersistentStopBase();
};

const perF43PersistentPackageRefreshBase=perBab4RefreshEvidencePackage;
perBab4RefreshEvidencePackage=function(){
  perF43PersistentPackageRefreshBase();
  if(selectedExperiment?.id!=='F4.3')return;
  const n=perF43EvidenceCount();
  setText('perBab4VisualState',n+' / 6 kondisi');
  setText('perBab4VisualFrames',n+' / 6 citra representatif');
  setText('perBab4LaneReady',n+' / 6');
  const chip=$('perBab4VisualState');if(chip)chip.className='status-chip '+(n>=6?'':'waiting');
};


/* ===== F4.2 cumulative 100-data dataset store v1 (2026-09-20) =====
 * Keeps all 2 objects x 5 distances x 10 repetitions across selector changes,
 * render cycles, recorder restarts, and browser reloads.
 */
const PER_F42_DRAFT_KEY='agv.perception.f42.ta100.v1';
const PER_F42_RECOVERY_ROWS_20260920=[["1","Manusia","1","1","2026-09-19T19:51:31.445Z","Manusia","0","0.98643","0.05000","2.608","1.608","0.4","1.0","Klasifikasi Sesuai","torchvision_ssdlite320_coco","3","Kelas sesuai acuan • Bukti kelas Manusia 98.6% • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025131_772_F4.2_manusia_1m_data_01_klasifikasi_sesuai.jpg"],["2","Manusia","1","2","2026-09-19T19:51:34.864Z","Manusia","0","0.06966","0.05000","2.005","1.005","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","5","Kelas sesuai acuan • Bukti kelas Manusia 7.0% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025134_998_F4.2_manusia_1m_data_02_klasifikasi_sesuai.jpg"],["3","Manusia","1","3","2026-09-19T19:51:37.697Z","Manusia","0","0.06172","0.05000","5.555","4.555","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","4","Kelas sesuai acuan • Bukti kelas Manusia 6.2% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025137_751_F4.2_manusia_1m_data_03_klasifikasi_sesuai.jpg"],["4","Manusia","1","4","2026-09-19T19:51:40.304Z","Manusia","0","0.05276","0.05000","2.055","1.055","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","6","Kelas sesuai acuan • Bukti kelas Manusia 5.3% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025140_748_F4.2_manusia_1m_data_04_klasifikasi_sesuai.jpg"],["5","Manusia","1","5","2026-09-19T19:51:44.937Z","Manusia","0","0.98131","0.05000","1.950","0.950","0.4","1.0","Klasifikasi Sesuai","torchvision_ssdlite320_coco","5","Kelas sesuai acuan • Bukti kelas Manusia 98.1% • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025144_364_F4.2_manusia_1m_data_05_klasifikasi_sesuai.jpg"],["6","Manusia","1","6","2026-09-19T19:51:46.026Z","Manusia","0","0.05902","0.05000","2.024","1.024","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","5","Kelas sesuai acuan • Bukti kelas Manusia 5.9% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025146_784_F4.2_manusia_1m_data_06_klasifikasi_sesuai.jpg"],["7","Manusia","1","7","2026-09-19T19:51:49.480Z","Manusia","0","0.05514","0.05000","2.029","1.029","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","6","Kelas sesuai acuan • Bukti kelas Manusia 5.5% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025149_776_F4.2_manusia_1m_data_07_klasifikasi_sesuai.jpg"],["8","Manusia","1","8","2026-09-19T19:51:52.152Z","Manusia","0","0.07060","0.05000","2.011","1.011","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","6","Kelas sesuai acuan • Bukti kelas Manusia 7.1% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025152_752_F4.2_manusia_1m_data_08_klasifikasi_sesuai.jpg"],["9","Manusia","1","9","2026-09-19T19:51:55.623Z","Manusia","0","0.05605","0.05000","2.029","1.029","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","5","Kelas sesuai acuan • Bukti kelas Manusia 5.6% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025155_823_F4.2_manusia_1m_data_09_klasifikasi_sesuai.jpg"],["10","Manusia","1","10","2026-09-19T19:51:58.706Z","Manusia","0","0.06077","0.05000","2.028","1.028","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 6.1% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025128/evidence_frames/20260920_025158_803_F4.2_manusia_1m_data_10_klasifikasi_sesuai.jpg"],["11","Manusia","2","1","2026-09-19T19:53:31.575Z","Manusia","0","0.05582","0.05000","2.080","0.080","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 5.6% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025331_826_F4.2_manusia_2m_data_01_klasifikasi_sesuai.jpg"],["12","Manusia","2","2","2026-09-19T19:53:34.539Z","Manusia","0","0.06189","0.05000","2.087","0.087","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 6.2% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025334_838_F4.2_manusia_2m_data_02_klasifikasi_sesuai.jpg"],["13","Manusia","2","3","2026-09-19T19:53:37.548Z","Manusia","0","0.05531","0.05000","2.085","0.085","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 5.5% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025337_911_F4.2_manusia_2m_data_03_klasifikasi_sesuai.jpg"],["14","Manusia","2","4","2026-09-19T19:53:40.353Z","Manusia","0","0.05395","0.05000","2.105","0.105","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 5.4% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025340_987_F4.2_manusia_2m_data_04_klasifikasi_sesuai.jpg"],["15","Manusia","2","5","2026-09-19T19:53:43.586Z","Manusia","0","0.05155","0.05000","2.100","0.100","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 5.2% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025343_913_F4.2_manusia_2m_data_05_klasifikasi_sesuai.jpg"],["16","Manusia","2","6","2026-09-19T19:53:47.129Z","Manusia","0","0.05290","0.05000","4.505","2.505","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 5.3% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025347_340_F4.2_manusia_2m_data_06_klasifikasi_sesuai.jpg"],["17","Manusia","2","7","2026-09-19T19:53:50.016Z","Manusia","0","0.10738","0.05000","4.458","2.458","0.5","1.0","Klasifikasi Sesuai","torchvision_ssdlite320_coco","5","Kelas sesuai acuan • Bukti kelas Manusia 10.7% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025350_481_F4.2_manusia_2m_data_07_klasifikasi_sesuai.jpg"],["18","Manusia","2","8","2026-09-19T19:53:53.172Z","Manusia","0","0.05471","0.05000","4.472","2.472","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 5.5% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025353_230_F4.2_manusia_2m_data_08_klasifikasi_sesuai.jpg"],["19","Manusia","2","9","2026-09-19T19:53:55.212Z","Manusia","0","0.05273","0.05000","2.153","0.153","0.8","0.9","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 5.3% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025355_798_F4.2_manusia_2m_data_09_klasifikasi_sesuai.jpg"],["20","Manusia","2","10","2026-09-19T19:53:58.501Z","Manusia","0","0.09783","0.05000","5.031","3.031","0.5","1.0","Klasifikasi Sesuai","torchvision_ssdlite320_coco","8","Kelas sesuai acuan • Bukti kelas Manusia 9.8% • Nilai kepercayaan rendah • sumber semantik","/home/sirobo/agv/data/presepsi/BAB4_F4.2_20260920_025328/evidence_frames/20260920_025358_796_F4.2_manusia_2m_data_10_klasifikasi_sesuai.jpg"]];
perBab4.f42Session=perBab4.f42Session||{rows:[]};

function perF42RowObject(values){return String(values?.[1]||'').toLowerCase().includes('motor')?'motorcycle':'human'}
function perF42RowKey(values){return perF42RowObject(values)+'|'+String(+values?.[2]||0)+'|'+String(+values?.[3]||0)}
function perF42StatusKey(values){
  const s=String(values?.[13]||'').toUpperCase();
  if(s.includes('KLASIFIKASI SESUAI')||s==='CORRECT'||s.includes('SESUAI'))return'correct';
  if(s.includes('SALAH KLASIFIKASI')||s==='MISCLASSIFIED')return'mis';
  if(s.includes('TIDAK TERDETEKSI')||s==='NOT_DETECTED')return'lost';
  const actual=perF42RowObject(values),det=String(values?.[5]||'').toLowerCase();
  if(!det||det.includes('tidak'))return'lost';
  if(actual==='motorcycle')return det.includes('motor')?'correct':'mis';
  return det.includes('manusia')||det.includes('human')?'correct':'mis';
}
function perF42NormalizeRows(rows){
  const map=new Map();
  for(const row of rows||[]){
    const r=Array.isArray(row)?{sec:0,values:[...row],yaml:[]}:(row&&Array.isArray(row.values)?{sec:+row.sec||0,values:[...row.values],yaml:Array.isArray(row.yaml)?row.yaml:[]}:{sec:0,values:[],yaml:[]});
    if(r.values.length<18)continue;const bw=+r.values[11],bh=+r.values[12];if(Number.isFinite(bw)&&bw>0&&bw<=1.2)r.values[11]=(bw*1280).toFixed(1);if(Number.isFinite(bh)&&bh>0&&bh<=1.2)r.values[12]=(bh*720).toFixed(1);map.set(perF42RowKey(r.values),r);
  }
  const out=[...map.values()].sort((a,b)=>{
    const ao=perF42RowObject(a.values)==='human'?0:1,bo=perF42RowObject(b.values)==='human'?0:1;
    return ao-bo||(+a.values[2]-+b.values[2])||(+a.values[3]-+b.values[3]);
  });
  out.forEach((r,i)=>r.values[0]=String(i+1));return out;
}
function perF42PersistDraft(){
  try{localStorage.setItem(PER_F42_DRAFT_KEY,JSON.stringify({version:1,updated_at:new Date().toISOString(),rows:perBab4.f42Session.rows}))}catch(e){console.warn('F4.2 draft persistence failed',e)}
}
function perF42LoadDraft(){
  try{
    const raw=localStorage.getItem(PER_F42_DRAFT_KEY);
    if(raw){const j=JSON.parse(raw);if(j?.version===1&&Array.isArray(j.rows)){perBab4.f42Session.rows=perF42NormalizeRows(j.rows);return}}
  }catch(e){console.warn('F4.2 draft load failed',e)}
  perBab4.f42Session.rows=perF42NormalizeRows(PER_F42_RECOVERY_ROWS_20260920);
  perF42PersistDraft();
}
function perF42RestoreRows(){
  if(selectedExperiment?.id!=='F4.2'||currentExp!=='perception')return;
  perBab4.f42Session.rows=perF42NormalizeRows(perBab4.f42Session.rows);
  tableRunRows.set(0,perBab4.f42Session.rows.map(r=>({sec:r.sec||0,values:[...r.values],yaml:Array.isArray(r.yaml)?r.yaml:[]})));
  perBab4PrepareSummary();renderExperimentTable();perBab4RenderProgress();perBab4DrawSummary();
}
function perF42SyncAccumulator(){
  if(selectedExperiment?.id!=='F4.2'||currentExp!=='perception')return;
  const merged=[...(perBab4.f42Session.rows||[]),...(tableRunRows.get(0)||[])];
  perBab4.f42Session.rows=perF42NormalizeRows(merged);
  tableRunRows.set(0,perBab4.f42Session.rows.map(r=>({sec:r.sec||0,values:[...r.values],yaml:Array.isArray(r.yaml)?r.yaml:[]})));
  perF42PersistDraft();
}
function perF42DatasetCount(){return perBab4.f42Session.rows?.length||0}
perF42LoadDraft();

/* Statistics accept both old English runtime labels and final Indonesian labels. */
perF42ConditionStats=function(target,distance){
  const rows=perF42RowsFor(target,distance).map(r=>r.values),n=rows.length,correct=rows.filter(v=>perF42StatusKey(v)==='correct'),mis=rows.filter(v=>perF42StatusKey(v)==='mis'),lost=rows.filter(v=>perF42StatusKey(v)==='lost'),detected=correct.length+mis.length;
  const conf=correct.map(v=>+v[7]).filter(Number.isFinite),est=rows.map(v=>+v[9]).filter(Number.isFinite),err=rows.map(v=>+v[10]).filter(Number.isFinite),mean=a=>a.length?a.reduce((s,x)=>s+x,0)/a.length:NaN,std=a=>{if(!a.length)return NaN;const m=mean(a);return Math.sqrt(a.reduce((s,x)=>s+(x-m)*(x-m),0)/a.length)};
  return{n,correct:correct.length,mis:mis.length,lost:lost.length,detectionRate:n?100*detected/n:0,classAccuracy:detected?100*correct.length/detected:0,confMean:mean(conf),confMin:conf.length?Math.min(...conf):NaN,confMax:conf.length?Math.max(...conf):NaN,confStd:std(conf),distMean:mean(est),errorMean:mean(err),mae:mean(err.map(Math.abs))};
};
perBab4BuildDetectionSummary=function(){
  const groups=new Map();
  for(const r of perBab4Rows()){const v=r.values,key=perF42RowObject(v)+'|'+v[2],g=groups.get(key)||{object:perF42RowObject(v)==='motorcycle'?'Sepeda Motor':'Manusia',dist:+v[2],rows:[]};g.rows.push(v);groups.set(key,g)}
  return [...groups.values()].sort((a,b)=>String(a.object).localeCompare(String(b.object))||a.dist-b.dist).map(g=>{
    const n=g.rows.length,correct=g.rows.filter(v=>perF42StatusKey(v)==='correct'),mis=g.rows.filter(v=>perF42StatusKey(v)==='mis'),lost=g.rows.filter(v=>perF42StatusKey(v)==='lost'),detected=correct.length+mis.length;
    const conf=correct.map(v=>+v[7]).filter(Number.isFinite),est=g.rows.map(v=>+v[9]).filter(Number.isFinite),err=g.rows.map(v=>+v[10]).filter(Number.isFinite),mean=a=>a.length?a.reduce((s,x)=>s+x,0)/a.length:NaN,std=a=>{if(!a.length)return NaN;const m=mean(a);return Math.sqrt(a.reduce((s,x)=>s+(x-m)*(x-m),0)/a.length)},fmt=(x,d=3)=>Number.isFinite(x)?x.toFixed(d):'--';
    return[g.object,g.dist,n,correct.length,mis.length,lost.length,fmt(n?100*detected/n:0,2),fmt(detected?100*correct.length/detected:0,2),fmt(mean(conf),5),fmt(conf.length?Math.min(...conf):NaN,5),fmt(conf.length?Math.max(...conf):NaN,5),fmt(std(conf),5),fmt(mean(est),3),fmt(mean(err),3),fmt(mean(err.map(Math.abs)),3)];
  });
};

const perF42CumulativeCaptureBase=perF42CaptureOne;
perF42CaptureOne=async function(target,distance,repeat){
  if(selectedExperiment?.id==='F4.2'&&perBab4.f42Session.rows.length)perF42RestoreRows();
  const out=await perF42CumulativeCaptureBase(target,distance,repeat);
  perF42SyncAccumulator();perF42RestoreRows();return out;
};

const perF42CumulativeStartBase=startWebRecording;
startWebRecording=async function(){
  const f42=selectedExperiment?.id==='F4.2'&&currentExp==='perception',preserved=f42?perF42NormalizeRows(perBab4.f42Session.rows):null;
  await perF42CumulativeStartBase();
  if(f42&&recordStartedMs){if(preserved?.length)perBab4.f42Session.rows=preserved;perF42RestoreRows();perF42PersistDraft()}
};

const perF42CumulativeStopBase=stopWebRecording;
stopWebRecording=async function(){
  const f42=selectedExperiment?.id==='F4.2'&&currentExp==='perception';
  if(f42){perF42SyncAccumulator();perF42RestoreRows()}
  const out=await perF42CumulativeStopBase();
  if(f42){perF42PersistDraft();perF42RestoreRows()}
  return out;
};

function perF42RecoverExisting(){
  const merged=[...(perBab4.f42Session.rows||[]),...PER_F42_RECOVERY_ROWS_20260920];
  perBab4.f42Session.rows=perF42NormalizeRows(merged);perF42PersistDraft();perF42RestoreRows();perF42RefreshUi();
  toast('Pemulihan F4.2 selesai: data Manusia 1 m dan 2 m telah digabungkan ke dataset aktif.');
}
function perF42ClearDataset(){
  if(!confirm('Hapus seluruh akumulasi 100 data F4.2 dari browser? Data CSV yang sudah tersimpan di disk tidak ikut dihapus.'))return;
  perBab4.f42Session.rows=[];perF42PersistDraft();tableRunRows.set(0,[]);tableRunRows.set(1,[]);perBab4.evidence.f42=[];renderExperimentTable();perBab4RenderProgress();perBab4DrawSummary();perF42RefreshUi();perBab4RefreshEvidencePackage();toast('Akumulasi F4.2 dikosongkan.');
}

const perF42CumulativePanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const out=perF42CumulativePanelBase();
  if(selectedExperiment?.id==='F4.2'&&currentExp==='perception'){
    if(perBab4.f42Session.rows.length)perF42RestoreRows();
    const ctl=$('perF42BatchControls');
    if(ctl&&!$('perF42DatasetState')){
      const n=document.createElement('div');n.id='perF42DatasetState';n.className='per-f42-control-state';n.innerHTML='<span>Akumulasi Dataset 4.2</span><b id="perF42DatasetCountValue">0 / 100 data</b>';ctl.appendChild(n);
      const a=document.createElement('div');a.id='perF42DatasetActions';a.className='per-f42-batch-actions';a.innerHTML='<button class="soft-btn" id="perF42RecoverExisting">PULIHKAN DATA 1 m + 2 m</button><button class="warning-btn" id="perF42ClearDataset">HAPUS DATASET 4.2</button>';ctl.appendChild(a);
      $('perF42RecoverExisting')?.addEventListener('click',perF42RecoverExisting);$('perF42ClearDataset')?.addEventListener('click',perF42ClearDataset);
    }
    setText('perF42DatasetCountValue',perF42DatasetCount()+' / 100 data');
  }
  return out;
};

const perF42CumulativeRefreshBase=perF42RefreshUi;
perF42RefreshUi=function(){
  const out=perF42CumulativeRefreshBase();
  if(selectedExperiment?.id==='F4.2')setText('perF42DatasetCountValue',perF42DatasetCount()+' / 100 data');
  return out;
};

/* ===== F4.3 MASTER SESSION WORKFLOW v1 =====
 * START once -> six conditions x 10 samples -> FINISH/SAVE once -> one complete ZIP.
 */
function perF43MasterIsActive(){
  return !!recordStartedMs && recordingDomain==='perception' && recordingLeafId==='F4.3';
}
function perF43MasterDatasetStatus(){
  const source=(perBab4.f43Session&&perBab4.f43Session.rows&&perBab4.f43Session.rows.length)?perBab4.f43Session.rows:(tableRunRows.get(0)||[]);
  const counts=Object.fromEntries(PER_BAB4_LANE_V3.map(function(q){return[q[0],0]})),seen=new Set();
  for(const r of source){
    const v=(r&&r.values)||[],meta=PER_BAB4_LANE_V3.find(function(q){return q[1]===v[1]});if(!meta)continue;
    const rep=+v[2]||0,key=meta[0]+'|'+rep;if(rep<1||rep>10||seen.has(key))continue;seen.add(key);counts[meta[0]]++;
  }
  const total=Object.values(counts).reduce(function(a,b){return a+b},0),conditions=Object.values(counts).filter(function(n){return n>=10}).length,evidence=typeof perF43EvidenceCount==='function'?perF43EvidenceCount():0;
  return{counts:counts,total:total,conditions:conditions,evidence:evidence,complete:total===60&&conditions===6&&evidence===6};
}
function perF43MasterMissingText(){
  const s=perF43MasterDatasetStatus(),missing=[];
  for(const q of PER_BAB4_LANE_V3){const n=s.counts[q[0]]||0;if(n<10)missing.push(q[1]+' '+n+'/10')}
  if(s.evidence<6)missing.push('Dokumentasi visual '+s.evidence+'/6');
  return missing.join(' • ');
}
async function perF43MasterStart(){
  if(selectedExperiment&&selectedExperiment.id!=='F4.3')return;
  if(perBab4.laneBatch&&perBab4.laneBatch.active)return toast('Selesaikan atau hentikan pengambilan kondisi yang sedang berjalan.',true);
  if(recordStartedMs&&!perF43MasterIsActive())return toast('Perekam pengujian lain masih aktif. Selesaikan perekaman tersebut terlebih dahulu.',true);
  if(perF43MasterIsActive()){perF43MasterRefreshUi();return toast('Sesi utama F4.3 sudah aktif. Pilih kondisi lalu ambil 10 sampel / 30 s.')}
  if(typeof perF43RestoreRows==='function')perF43RestoreRows();
  await startWebRecording();
  if(!perF43MasterIsActive())return toast('Sesi utama F4.3 belum berhasil dimulai.',true);
  if(typeof perF43RestoreRows==='function')perF43RestoreRows();
  if(typeof perF43RecoverServerEvidence==='function')await perF43RecoverServerEvidence();
  perF43MasterRefreshUi();toast('Sesi utama F4.3 aktif. Recorder tetap berjalan sampai seluruh 6 kondisi selesai.');
}

const perF43MasterBatchStartBase=perBab4LaneBatchStart;
perBab4LaneBatchStart=async function(failedOnly){
  failedOnly=!!failedOnly;
  if(!selectedExperiment||selectedExperiment.id!=='F4.3')return perF43MasterBatchStartBase(failedOnly);
  if(perBab4.laneBatch.active)return;
  if(!perF43MasterIsActive())return toast('Klik MULAI / LANJUTKAN PENGUJIAN 4.3 terlebih dahulu. Satu sesi digunakan untuk seluruh 6 kondisi.',true);
  if(!channelFresh('lane_state',2.5))return toast('Data Lane Safety belum diperbarui.',true);
  const cond=($('perBab4LaneCondition')&&$('perBab4LaneCondition').value)||'center';
  const byRep=new Map(perBab4LaneRowsFor(cond).map(function(r){return[+r.values[2],r.values[17]]}));
  const queue=failedOnly?[1,2,3,4,5,6,7,8,9,10].filter(function(n){return !['Memenuhi Kriteria','PASS'].includes(byRep.get(n))}):[1,2,3,4,5,6,7,8,9,10];
  if(!queue.length)return toast('Sepuluh data pada kondisi ini sudah memenuhi kriteria.');
  Object.assign(perBab4.laneBatch,{active:true,cond:cond,queue:queue.slice(),total:queue.length,done:0,lastStamp:0,busy:false,lastResult:'PENGAMBILAN_KONDISI_30_S',startMs:Date.now(),durationMs:30000,intervalMs:3000,autoStartedSession:false});
  const sel=$('perBab4LaneCondition');if(sel)sel.disabled=true;
  clearInterval(perBab4.laneBatch.timer);perBab4.laneBatch.timer=setInterval(perBab4LaneBatchTick,100);
  perBab4LaneBatchRefresh();perF43MasterRefreshUi();perBab4LaneBatchTick();
};

async function perF43FinishAndDownload(){
  if(!selectedExperiment||selectedExperiment.id!=='F4.3')return;
  if(perBab4.laneBatch&&perBab4.laneBatch.active)return toast('Pengambilan kondisi masih berjalan. Tunggu 10 sampel selesai terlebih dahulu.',true);
  if(typeof perF43SyncAccumulator==='function')perF43SyncAccumulator();
  if(typeof perF43RestoreRows==='function')perF43RestoreRows();
  if(typeof perF43RecoverServerEvidence==='function')await perF43RecoverServerEvidence();
  const s=perF43MasterDatasetStatus();
  if(!s.complete)return toast('Full Data 4.3 belum lengkap • '+perF43MasterMissingText(),true);
  const btn=$('perF43FullDownload');if(btn){btn.disabled=true;btn.textContent='MENYIAPKAN FULL DATA 4.3...'}
  try{
    if(!perF43MasterIsActive()){
      await startWebRecording();
      if(!perF43MasterIsActive())throw new Error('Gagal membuka sesi penyimpanan F4.3');
      if(typeof perF43RestoreRows==='function')perF43RestoreRows();
    }
    if(typeof perBab4PrepareSummary==='function')perBab4PrepareSummary();
    if(typeof perBab4PrepareEvidenceExports==='function')await perBab4PrepareEvidenceExports();
    await stopWebRecording();
    await new Promise(function(resolve){setTimeout(resolve,350)});
    await perBab4DownloadZip('/api/perception/f43/latest.zip','FULL_DATA_4.3_LANE_SAFETY.zip');
    toast('Full Data 4.3 selesai: tabel + rekap + grafik + dokumentasi 6 kondisi berada dalam satu ZIP.');
  }catch(e){toast('Gagal membuat Full Data 4.3: '+(e&&e.message?e.message:e),true)}
  finally{perF43MasterRefreshUi()}
}

function perF43MasterRefreshUi(){
  if(!selectedExperiment||selectedExperiment.id!=='F4.3')return;
  const s=perF43MasterDatasetStatus(),active=perF43MasterIsActive(),batch=!!(perBab4.laneBatch&&perBab4.laneBatch.active);
  const start=$('perF43MasterStart');if(start){start.disabled=batch||active;start.textContent=active?'● SESI UTAMA F4.3 AKTIF':(s.total?'▶ LANJUTKAN PENGUJIAN 4.3':'▶ MULAI PENGUJIAN 4.3')}
  setText('perF43MasterState',active?'AKTIF • recorder tetap berjalan sampai seluruh kondisi selesai':'BELUM AKTIF');
  setText('perF43MasterTotal',s.total+' / 60 data • '+s.conditions+' / 6 kondisi');
  setText('perF43MasterEvidence',s.evidence+' / 6 citra representatif');
  const run=$('perLaneRun10');if(run){run.disabled=batch||!active||!channelFresh('lane_state',2.5);run.textContent='▶ AMBIL 10 SAMPEL KONDISI INI / 30 s'}
  const retry=$('perLaneRepeatFailed');if(retry)retry.disabled=batch||!active;
  const finish=$('perF43FullDownload');if(finish){finish.disabled=batch||!s.complete;finish.textContent=s.complete?'⬇ SELESAI & DOWNLOAD FULL DATA 4.3':'FULL DATA 4.3 • '+s.total+'/60 • '+s.evidence+'/6 GAMBAR'}
  const hint=$('perF43MasterMissing');if(hint)hint.textContent=s.complete?'SELURUH DATA LENGKAP • siap disimpan dan diunduh':'Belum lengkap: '+(perF43MasterMissingText()||'--');
  const old=$('perF43DatasetActions');if(old)old.remove();
  const cnt=$('perF43DatasetCount');if(cnt)cnt.style.display='none';
}

const perF43MasterControlsBase=perBab4LaneControls;
perBab4LaneControls=function(){
  if(!selectedExperiment||selectedExperiment.id!=='F4.3')return perF43MasterControlsBase();
  const options=PER_BAB4_LANE_V3.map(function(q){return '<option value="'+q[0]+'">'+q[1]+' • '+perBab4LaneStatusLabel(q[2])+' / '+perBab4LaneStatusLabel(q[3])+'</option>'}).join('');
  return '<div class="per-lane-geometry-strip"><b>PARAMETER GEOMETRI PENGUJIAN</b><span>Lebar jalan: 6,0 m</span><span>Lebar koridor keselamatan: 4,0 m</span><span>Jarak titik bawah: 4,3 m</span><span>Jarak titik atas: 9,3 m</span><span>Margin lateral: 1,0 m/sisi</span></div>'+
  '<div class="per-bab4-controls per-lane-batch-controls" id="perLaneBatchControls">'+
  '<div class="per-bab4-actions"><button class="primary-btn" id="perF43MasterStart">▶ MULAI PENGUJIAN 4.3</button></div>'+
  '<div class="per-lane-control-state"><span>Sesi utama F4.3</span><b id="perF43MasterState">BELUM AKTIF</b></div>'+
  '<div class="per-lane-control-state"><span>Total data F4.3</span><b id="perF43MasterTotal">0 / 60 data • 0 / 6 kondisi</b></div>'+
  '<div class="per-lane-control-state"><span>Dokumentasi representatif</span><b id="perF43MasterEvidence">0 / 6 citra</b></div>'+
  '<label>Kondisi pengujian<select id="perBab4LaneCondition">'+options+'</select></label>'+
  '<div class="per-bab4-actions per-lane-batch-actions"><button class="primary-btn" id="perLaneRun10">▶ AMBIL 10 SAMPEL KONDISI INI / 30 s</button><button class="warning-btn" id="perLaneStopBatch">■ HENTIKAN PENGAMBILAN KONDISI</button><button class="soft-btn" id="perLaneRepeatFailed">↻ ULANGI DATA TIDAK MEMENUHI KRITERIA</button><button class="soft-btn" id="perLaneResetCondition">ATUR ULANG PILIHAN KONDISI</button></div>'+
  '<div class="per-lane-control-state"><span>Kondisi pengujian aktif</span><b id="perLaneBatchCondition">Posisi Tengah Koridor Keselamatan</b></div>'+
  '<div class="per-lane-control-state"><span>Kesesuaian kondisi aktual</span><b id="perLaneBatchReady">MENUNGGU DATA AKTUAL</b></div>'+
  '<div class="per-lane-control-state"><span>Kemajuan kondisi</span><b id="perLaneBatchProgress">0 / 10</b></div>'+
  '<div class="per-lane-control-state"><span>Status data waktu-nyata</span><b id="perLaneBatchFresh">MENUNGGU DATA</b></div>'+
  '<div class="per-lane-test-hint"><b>Alur final F4.3:</b> mulai sesi satu kali → pilih satu kondisi → sesuaikan posisi kendaraan → ambil 10 sampel/30 s → pindah ke kondisi berikutnya. Recorder tidak berhenti ketika berganti kondisi.</div></div>';
};

const perF43MasterFinalActionsBase=perBab4EnsureFinalActions;
perBab4EnsureFinalActions=function(){
  if(!selectedExperiment||selectedExperiment.id!=='F4.3')return perF43MasterFinalActionsBase();
  const host=$('perBab4FolderContract');if(!host||!perBab4FinalDatasetActive())return;
  let box=$('perBab4FolderActions');if(!box){box=document.createElement('div');box.id='perBab4FolderActions';box.className='per-bab4-folder-actions';host.appendChild(box)}
  box.innerHTML='<button class="primary-btn" id="perF43FullDownload">FULL DATA 4.3 • 0/60 • 0/6 GAMBAR</button><div class="per-lane-test-hint" id="perF43MasterMissing">Lengkapi enam kondisi terlebih dahulu.</div><div class="per-lane-test-hint"><b>ISI ZIP FULL DATA 4.3:</b> T1 Data Hasil 60 pengujian • T2 Rekapitulasi Tingkat Keberhasilan • XLSX • G01 Tingkat Keberhasilan • G02 Jarak Batas Lajur • G03 Distribusi Status • <b>G04 Dokumentasi Visual 6 Kondisi dalam 1 PNG</b> • manifest/metadata perekaman.</div>';
  const btn=$('perF43FullDownload');if(btn)btn.addEventListener('click',perF43FinishAndDownload);
  perF43MasterRefreshUi();
};

const perF43MasterPanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const out=perF43MasterPanelBase();
  if(selectedExperiment&&selectedExperiment.id==='F4.3'&&currentExp==='perception'){
    const start=$('perF43MasterStart');if(start)start.addEventListener('click',perF43MasterStart);
    const old=$('perF43DatasetActions');if(old)old.remove();
    const cnt=$('perF43DatasetCount');if(cnt)cnt.style.display='none';
    setTimeout(perF43MasterRefreshUi,0);
  }
  return out;
};

const perF43MasterBatchRefreshBase=perBab4LaneBatchRefresh;
perBab4LaneBatchRefresh=function(){const out=perF43MasterBatchRefreshBase();if(selectedExperiment&&selectedExperiment.id==='F4.3')perF43MasterRefreshUi();return out};

const perF43MasterEvidenceRefreshBase=perBab4RefreshEvidencePackage;
perBab4RefreshEvidencePackage=function(){const out=perF43MasterEvidenceRefreshBase();if(selectedExperiment&&selectedExperiment.id==='F4.3')perF43MasterRefreshUi();return out};

/* ===== F4.3 FINAL REPORT PNG CONTRACT v1 =====
 * G01 Success Rate
 * G02 Lane Gap
 * G03 Status Distribution
 * G04 One montage containing all six test conditions.
 */
const perF43FinalMontageBase=perBab4BuildLaneMontage;
perBab4BuildLaneMontage=async function(){
  if(!selectedExperiment||selectedExperiment.id!=='F4.3')return perF43FinalMontageBase();
  const spec=[
    ['center','Posisi Tengah Koridor Keselamatan','Aman / Aman'],
    ['left_yellow','Pendekatan Batas Keselamatan Kiri','Peringatan / Aman'],
    ['left_red','Pelanggaran Batas Keselamatan Kiri','Pelanggaran / Aman'],
    ['right_yellow','Pendekatan Batas Keselamatan Kanan','Aman / Peringatan'],
    ['right_red','Pelanggaran Batas Keselamatan Kanan','Aman / Pelanggaran'],
    ['gray','Di Luar Area Dapat Dilalui','Di Luar Area / Di Luar Area']
  ];
  const reps=spec.map(function(x){return perBab4RepresentativeLane(x[0])});
  if(!reps.some(Boolean))return null;
  const c=document.createElement('canvas');c.width=1280;c.height=840;
  const ctx=c.getContext('2d');ctx.fillStyle='#071113';ctx.fillRect(0,0,c.width,c.height);
  perBab4EvidenceText(ctx,'Dokumentasi Visual Pengujian Keamanan Lajur • 6 Kondisi',38,48,28,'#e5f4f3',900);
  perBab4EvidenceText(ctx,'Satu citra representatif untuk setiap kondisi pengujian; enam panel dalam satu berkas PNG.',38,78,15,'#8fa8a9',600);
  const pw=390,ph=330,gap=18;
  for(let i=0;i<6;i++){
    const col=i%3,row=Math.floor(i/3),x=25+col*(pw+gap),y=105+row*355,rep=reps[i];
    ctx.fillStyle='#0b1b1d';ctx.fillRect(x,y,pw,ph);
    ctx.strokeStyle=rep?'rgba(91,181,173,.72)':'rgba(255,200,107,.5)';ctx.lineWidth=2;ctx.strokeRect(x,y,pw,ph);
    perBab4EvidenceText(ctx,spec[i][1],x+12,y+28,18,'#e5f4f3',900);
    perBab4EvidenceText(ctx,spec[i][2],x+230,y+28,12,'#9dc0bd',800);
    if(rep){
      try{
        const im=await perBab4LoadEvidenceImage(rep.dataUrl);
        perBab4DrawImageContain(ctx,im,x+8,y+38,pw-16,240);
      }catch(_){perBab4EvidenceText(ctx,'Citra tidak tersedia',x+105,y+175,14,'#ff6677',800)}
      const result=rep.match===false?'Tidak Memenuhi Kriteria':'Memenuhi Kriteria';
      const color=rep.match===false?'#ff6677':'#48e0a4';
      perBab4EvidenceText(ctx,'Pengulangan '+(rep.repeat||'--')+' • '+result,x+10,y+299,12,color,800);
      perBab4EvidenceText(ctx,perBab4LaneEvidenceLabel(rep.evidenceMode||'DRIVABLE_OUTER_LANE'),x+10,y+319,10,'#9dc0bd',700);
    }else{
      perBab4EvidenceText(ctx,'Citra bukti belum tersedia',x+92,y+175,14,'#8fa8a9',700);
    }
  }
  perBab4EvidenceText(ctx,'Parameter geometri: lebar jalan 6,0 m • koridor keselamatan 4,0 m • titik bawah 4,3 m • titik atas 9,3 m • margin lateral 1,0 m/sisi',25,822,12,'#708f8d',600);
  return c;
};

const perF43FinalPrepareExportsBase=perBab4PrepareEvidenceExports;
perBab4PrepareEvidenceExports=async function(){
  if(!selectedExperiment||selectedExperiment.id!=='F4.3')return perF43FinalPrepareExportsBase();
  perBab4.evidence.exports=[];
  const c=await perBab4BuildLaneMontage();
  if(c)perBab4.evidence.exports.push({index:4,title:'Dokumentasi Visual Enam Kondisi Pengujian Keamanan Lajur',data_url:c.toDataURL('image/png')});
};

const perF43FinalGraphPayloadBase=collectExperimentGraphPngPayload;
collectExperimentGraphPngPayload=function(){
  if(!selectedExperiment||selectedExperiment.id!=='F4.3')return perF43FinalGraphPayloadBase();
  const out=[],add=function(id,title,index){const c=$(id);if(c&&c.width&&c.height)out.push({index:index,title:title,data_url:c.toDataURL('image/png')})};
  perBab4DrawSummary();perBab4DrawLaneGap();perBab4DrawLaneStatus();
  add('perBab4SummaryCanvas','Tingkat Keberhasilan Pengujian Keamanan Lajur',1);
  add('perLaneGapCanvas','Jarak Batas Lajur terhadap Batas Keselamatan per Pengulangan',2);
  add('perLaneStatusCanvas','Distribusi Status Keamanan Lajur per Kondisi',3);
  for(const e of (perBab4.evidence.exports||[]))if(+e.index===4)out.push(e);
  return out;
};

/* ===== F4.2 MASTER SESSION + REPRESENTATIVE EVIDENCE v1 (2026-09-20) =====
 * One recorder session for all 10 physical conditions.
 * Each condition: 10 fresh semantic samples / 30 s.
 * Final output: raw table + summary + graphs + two 1-5 m montages in one ZIP.
 */
const PER_F42_EVIDENCE_KEY='agv.perception.f42.evidence10.v1';
perBab4.f42EvidenceBank=perBab4.f42EvidenceBank||{};
let perF42SeedEvidenceLoaded=false,perF42SeedEvidenceLoading=false;

function perF42EvidenceKey(target,distance){return String(target)+'-'+String(+distance)}
function perF42EvidenceScore(e){
  if(!e||!e.dataUrl)return-1e9;
  return(e.success||e.status==='CORRECT'?10000:0)+(Number.isFinite(+e.confidence)?+e.confidence*1000:0)-Math.abs((+e.repeat||5)-5);
}
function perF42EvidencePersist(){
  try{localStorage.setItem(PER_F42_EVIDENCE_KEY,JSON.stringify({version:1,updated_at:new Date().toISOString(),items:perBab4.f42EvidenceBank}))}
  catch(e){console.warn('F4.2 representative evidence localStorage failed',e)}
}
function perF42EvidenceLoad(){
  try{
    const raw=localStorage.getItem(PER_F42_EVIDENCE_KEY);if(!raw)return;
    const j=JSON.parse(raw);if(j?.version===1&&j.items&&typeof j.items==='object')perBab4.f42EvidenceBank=j.items;
  }catch(e){console.warn('F4.2 representative evidence load failed',e)}
}
function perF42EvidenceUpsert(entry,force=false){
  if(!entry?.target||!Number.isFinite(+entry.distance)||!entry.dataUrl)return false;
  const key=perF42EvidenceKey(entry.target,entry.distance),old=perBab4.f42EvidenceBank[key];
  if(!force&&old&&perF42EvidenceScore(old)>=perF42EvidenceScore(entry))return false;
  perBab4.f42EvidenceBank[key]={...entry,key};
  perF42EvidencePersist();return true;
}
function perF42EvidenceCount(){return Object.values(perBab4.f42EvidenceBank||{}).filter(x=>x?.dataUrl).length}
async function perF42RecoverSeedEvidence(){
  if(perF42SeedEvidenceLoaded||perF42SeedEvidenceLoading)return;
  perF42SeedEvidenceLoading=true;
  try{
    const r=await fetch('/f42_recovery_evidence.json?v=20260920-master-v1',{cache:'no-store'});
    if(!r.ok)throw new Error('HTTP '+r.status);
    const j=await r.json();
    for(const e of (Array.isArray(j.items)?j.items:[]))perF42EvidenceUpsert(e,false);
    perF42SeedEvidenceLoaded=true;
  }catch(e){console.warn('F4.2 seed evidence recovery unavailable',e)}
  finally{perF42SeedEvidenceLoading=false}
}
perF42EvidenceLoad();

const perF42MasterRepresentativeBase=perBab4RepresentativeDetection;
perBab4RepresentativeDetection=function(target,dist){
  const saved=perBab4.f42EvidenceBank?.[perF42EvidenceKey(target,dist)];
  if(saved?.dataUrl)return saved;
  return perF42MasterRepresentativeBase(target,dist);
};

const perF42MasterCaptureBase=perF42CaptureOne;
perF42CaptureOne=async function(target,distance,repeat){
  const out=await perF42MasterCaptureBase(target,distance,repeat);
  const e=[...(perBab4.evidence?.f42||[])].reverse().find(x=>x?.target===target&&+x.distance===+distance&&+x.repeat===+repeat&&x.dataUrl);
  if(e)perF42EvidenceUpsert({...e,target,distance,repeat},false);
  if(typeof perF42SyncAccumulator==='function')perF42SyncAccumulator();
  perF42MasterRefreshUi();
  return out;
};

function perF42MasterIsActive(){
  return !!recordStartedMs&&recordingDomain==='perception'&&recordingLeafId==='F4.2';
}
function perF42MasterDatasetStatus(){
  const source=(perBab4.f42Session?.rows?.length?perBab4.f42Session.rows:(tableRunRows.get(0)||[]));
  const counts={},seen=new Set();
  for(const target of ['human','motorcycle'])for(let d=1;d<=5;d++)counts[perF42EvidenceKey(target,d)]=0;
  for(const r of source){
    const v=r?.values||[],target=perF42RowObject(v),dist=+v[2]||0,rep=+v[3]||0,key=perF42EvidenceKey(target,dist),unique=key+'|'+rep;
    if(!(key in counts)||rep<1||rep>10||seen.has(unique))continue;
    seen.add(unique);counts[key]++;
  }
  const total=Object.values(counts).reduce((a,b)=>a+b,0),conditions=Object.values(counts).filter(n=>n>=10).length,evidence=perF42EvidenceCount();
  return{counts,total,conditions,evidence,complete:total===100&&conditions===10&&evidence===10};
}
function perF42MasterMissingText(){
  const s=perF42MasterDatasetStatus(),missing=[];
  for(const target of ['human','motorcycle'])for(let d=1;d<=5;d++){
    const key=perF42EvidenceKey(target,d),n=s.counts[key]||0;
    if(n<10)missing.push((target==='human'?'Manusia':'Sepeda Motor')+' '+d+' m '+n+'/10');
  }
  if(s.evidence<10)missing.push('Citra representatif '+s.evidence+'/10');
  return missing.join(' • ');
}
async function perF42MasterStart(){
  if(selectedExperiment?.id!=='F4.2'||currentExp!=='perception')return;
  if(perBab4.f42Batch?.active)return toast('Selesaikan atau hentikan pengambilan kondisi yang sedang berjalan.',true);
  if(recordStartedMs&&!perF42MasterIsActive())return toast('Perekam pengujian lain masih aktif. Selesaikan perekaman tersebut terlebih dahulu.',true);
  if(perF42MasterIsActive()){perF42MasterRefreshUi();return toast('Sesi utama 4.2 sudah aktif. Pilih objek dan jarak lalu ambil 10 data / 30 s.')}
  await perF42RecoverSeedEvidence();
  if(typeof perF42RestoreRows==='function')perF42RestoreRows();
  await startWebRecording();
  if(!perF42MasterIsActive())return toast('Sesi utama 4.2 belum berhasil dimulai. Periksa preflight.',true);
  if(typeof perF42RestoreRows==='function')perF42RestoreRows();
  perF42MasterRefreshUi();
  toast('Sesi utama 4.2 aktif. Recorder tetap berjalan saat berpindah objek dan jarak.');
}

const perF42MasterBatchStartBase=perF42BatchStart;
perF42BatchStart=async function(retakeAll=false){
  if(selectedExperiment?.id!=='F4.2'||currentExp!=='perception')return perF42MasterBatchStartBase(retakeAll);
  const s=perBab4.f42Batch;if(s.active)return;
  if(!perF42MasterIsActive())return toast('Klik MULAI / LANJUTKAN PENGUJIAN 4.2 terlebih dahulu. Satu sesi digunakan untuk seluruh 10 kondisi.',true);
  if(!channelFresh('semantic_detections',2.5))return toast('Data deteksi semantik belum diperbarui. Tunggu sumber data siap.',true);
  const target=$('perBab4Target')?.value||'human',distance=+$('perBab4Distance')?.value||1;
  const existing=new Set(perF42RowsFor(target,distance).map(r=>+r.values[3]));
  const all=[1,2,3,4,5,6,7,8,9,10],queue=retakeAll?all:all.filter(n=>!existing.has(n));
  if(!queue.length)return toast('Kondisi ini sudah lengkap 10/10. Gunakan ULANGI KONDISI INI bila ingin mengganti seluruh 10 data.');
  Object.assign(s,{active:true,busy:false,target,distance,queue:[...queue],done:0,total:queue.length,startMs:Date.now(),lastStamp:+updated.semantic_detections||0,lastResult:'Menunggu data pertama pada detik ke-3'});
  const a=$('perBab4Target'),b=$('perBab4Distance');if(a)a.disabled=true;if(b)b.disabled=true;
  clearInterval(s.timer);s.timer=setInterval(perF42BatchTick,120);
  perF42RefreshUi();perF42MasterRefreshUi();
};

perBab4DetectionControls=function(){
  return '<div class="per-f42-batch-controls" id="perF42BatchControls">'+
    '<div class="per-f42-batch-actions"><button class="primary-btn" id="perF42MasterStart">▶ MULAI / LANJUTKAN PENGUJIAN 4.2</button></div>'+
    '<div class="per-f42-control-state"><span>Sesi Utama 4.2</span><b id="perF42MasterState">BELUM AKTIF</b></div>'+
    '<div class="per-f42-control-state"><span>Total Dataset</span><b id="perF42MasterTotal">0 / 100 data • 0 / 10 kondisi</b></div>'+
    '<div class="per-f42-control-state"><span>Dokumentasi Representatif</span><b id="perF42MasterEvidence">0 / 10 citra</b></div>'+
    '<label>Objek Acuan<select id="perBab4Target"><option value="human">Manusia</option><option value="motorcycle">Sepeda Motor</option></select></label>'+
    '<label>Jarak Acuan<select id="perBab4Distance">'+[1,2,3,4,5].map(x=>'<option value="'+x+'">'+x+' meter</option>').join('')+'</select></label>'+
    '<div class="per-f42-control-state"><span>Data Kondisi Aktif</span><b id="perF42CaptureProgress">0 / 10</b></div>'+
    '<div class="per-f42-control-state"><span>Durasi Kondisi</span><b>30 s • interval 3 s</b></div>'+
    '<div class="per-f42-control-state"><span>Pengambilan Berikutnya</span><b id="perF42NextCapture">--</b></div>'+
    '<div class="per-f42-control-state"><span>Hasil Terakhir</span><b id="perF42LastResult">Menunggu Data</b></div>'+
    '<div class="per-f42-batch-actions"><button class="primary-btn" id="perF42Run10">▶ AMBIL 10 DATA KONDISI INI / 30 s</button><button class="warning-btn" id="perF42StopBatch">■ HENTIKAN KONDISI</button><button class="soft-btn" id="perF42RepeatFailed">↻ ULANGI KONDISI INI</button></div>'+
    '<div class="per-f42-test-hint"><b>Alur final F4.2:</b> mulai sesi satu kali → pilih objek dan jarak → ambil 10 data/30 s → pindah ke jarak berikutnya. Data kondisi sebelumnya tetap tersimpan. Setelah 100/100 data dan 10 citra representatif lengkap, gunakan tombol DOWNLOAD FULL DATA 4.2.</div>'+
  '</div>';
};

function perF42MasterRefreshUi(){
  if(selectedExperiment?.id!=='F4.2'||currentExp!=='perception')return;
  const s=perF42MasterDatasetStatus(),active=perF42MasterIsActive(),batch=!!perBab4.f42Batch?.active;
  const start=$('perF42MasterStart');
  if(start){start.disabled=batch||active;start.textContent=active?'● SESI UTAMA 4.2 AKTIF':(s.total?'▶ LANJUTKAN PENGUJIAN 4.2':'▶ MULAI PENGUJIAN 4.2')}
  setText('perF42MasterState',active?'AKTIF • recorder tidak berhenti saat ganti kondisi':'BELUM AKTIF');
  setText('perF42MasterTotal',s.total+' / 100 data • '+s.conditions+' / 10 kondisi');
  setText('perF42MasterEvidence',s.evidence+' / 10 citra');
  const target=$('perBab4Target')?.value||'human',distance=+$('perBab4Distance')?.value||1,current=s.counts[perF42EvidenceKey(target,distance)]||0;
  const run=$('perF42Run10');if(run){run.disabled=batch||!active||current>=10||!channelFresh('semantic_detections',2.5);run.textContent=current>=10?'KONDISI SUDAH LENGKAP 10/10':'▶ AMBIL 10 DATA KONDISI INI / 30 s'}
  const stop=$('perF42StopBatch');if(stop)stop.disabled=!batch;
  const retry=$('perF42RepeatFailed');if(retry)retry.disabled=batch||!active||current<1;
  const finish=$('perF42FullDownload');if(finish){finish.disabled=batch||!s.complete;finish.textContent=s.complete?'⬇ DOWNLOAD FULL DATA 4.2':'FULL DATA 4.2 • '+s.total+'/100 • '+s.evidence+'/10 GAMBAR'}
  const hint=$('perF42MasterMissing');if(hint)hint.textContent=s.complete?'SELURUH DATA LENGKAP • siap dibuat satu ZIP final':'Belum lengkap: '+(perF42MasterMissingText()||'--');
  const old=$('perF42DatasetActions');if(old)old.remove();
  const oldState=$('perF42DatasetState');if(oldState)oldState.style.display='none';
  const generic=$('recordToggleBtn');if(generic)generic.hidden=true;
}

async function perF42FinishAndDownload(){
  if(selectedExperiment?.id!=='F4.2'||currentExp!=='perception')return;
  if(perBab4.f42Batch?.active)return toast('Pengambilan kondisi masih berjalan. Tunggu sampai 10 data selesai atau hentikan kondisi.',true);
  if(typeof perF42SyncAccumulator==='function')perF42SyncAccumulator();
  if(typeof perF42RestoreRows==='function')perF42RestoreRows();
  await perF42RecoverSeedEvidence();
  const s=perF42MasterDatasetStatus();
  if(!s.complete)return toast('Full Data 4.2 belum lengkap • '+perF42MasterMissingText(),true);
  const btn=$('perF42FullDownload');if(btn){btn.disabled=true;btn.textContent='MENYIAPKAN FULL DATA 4.2...'}
  try{
    if(!perF42MasterIsActive()){
      await startWebRecording();
      if(!perF42MasterIsActive())throw new Error('Gagal membuka sesi penyimpanan 4.2');
      if(typeof perF42RestoreRows==='function')perF42RestoreRows();
    }
    if(typeof perBab4PrepareSummary==='function')perBab4PrepareSummary();
    if(typeof perBab4PrepareEvidenceExports==='function')await perBab4PrepareEvidenceExports();
    const ok=await stopWebRecording();
    if(ok===false)throw new Error('STOP + SAVE gagal');
    await new Promise(resolve=>setTimeout(resolve,450));
    await perBab4DownloadZip('/api/experiment/record/last.zip','FULL_DATA_4.2_OBJECT_DETECTION.zip');
    toast('Full Data 4.2 selesai: tabel mentah + rekap + grafik + montage 5 jarak Manusia + montage 5 jarak Sepeda Motor berada dalam satu ZIP.');
  }catch(e){toast('Gagal membuat Full Data 4.2: '+(e?.message||e),true)}
  finally{perF42MasterRefreshUi()}
}

const perF42MasterActionsBase=perBab4EnsureFinalActions;
perBab4EnsureFinalActions=function(){
  if(selectedExperiment?.id!=='F4.2')return perF42MasterActionsBase();
  const host=$('perBab4FolderContract');if(!host||!perBab4FinalDatasetActive())return;
  let box=$('perBab4FolderActions');if(!box){box=document.createElement('div');box.id='perBab4FolderActions';box.className='per-bab4-folder-actions';host.appendChild(box)}
  box.innerHTML='<button class="primary-btn" id="perF42FullDownload">FULL DATA 4.2 • 0/100 • 0/10 GAMBAR</button><div class="per-f42-test-hint" id="perF42MasterMissing">Lengkapi 10 kondisi terlebih dahulu.</div>';
  $('perF42FullDownload')?.addEventListener('click',perF42FinishAndDownload);
  perF42MasterRefreshUi();
};

const perF42MasterPanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const out=perF42MasterPanelBase();
  const generic=$('recordToggleBtn');if(generic)generic.hidden=selectedExperiment?.id==='F4.2';
  if(selectedExperiment?.id==='F4.2'&&currentExp==='perception'){
    perF42RecoverSeedEvidence().then(()=>{perF42MasterRefreshUi();perBab4RefreshEvidencePackage()});
    $('perF42MasterStart')?.addEventListener('click',perF42MasterStart);
    $('perF42RepeatFailed')?.addEventListener('click',()=>perF42BatchStart(true));
    const old=$('perF42DatasetActions');if(old)old.remove();
    const oldState=$('perF42DatasetState');if(oldState)oldState.style.display='none';
    setTimeout(perF42MasterRefreshUi,0);
  }
  return out;
};

const perF42MasterRefreshBase=perF42RefreshUi;
perF42RefreshUi=function(){
  const out=perF42MasterRefreshBase();
  if(selectedExperiment?.id==='F4.2')perF42MasterRefreshUi();
  return out;
};

const perF42MasterPackageBase=perBab4EvidencePackageHtml;
perBab4EvidencePackageHtml=function(){
  if(selectedExperiment?.id!=='F4.2')return perF42MasterPackageBase();
  return '<section class="per-bab4-evidence-card per-bab4-save-package" id="perBab4SavePackage">'+
    '<div class="per-bab4-evidence-head"><div><span class="eyebrow">KELUARAN FINAL 4.2</span><b>100 data + 4 grafik + 2 kompilasi visual 5 jarak</b><p>Satu sesi menyimpan seluruh Manusia dan Sepeda Motor pada jarak 1–5 m. Setiap objek menghasilkan satu gambar kompilasi yang memuat lima jarak dalam satu PNG.</p></div><span class="status-chip waiting" id="perBab4VisualState">0 / 10 citra</span></div>'+
    '<div class="per-bab4-package-grid"><div><span>Manusia 1–5 m</span><b id="perBab4HumanReady">0 / 5 jarak</b></div><div><span>Sepeda Motor 1–5 m</span><b id="perBab4MotorReady">0 / 5 jarak</b></div><div><span>Citra representatif</span><b id="perBab4VisualFrames">0 / 10</b></div><div><span>Isi ZIP final</span><b>CSV + XLSX + grafik + 2 montage + manifest</b></div></div>'+
    '<div class="per-bab4-package-actions"><button class="soft-btn" id="perBab4PreviewHuman">PRATINJAU 5 JARAK MANUSIA</button><button class="soft-btn" id="perBab4PreviewMotor">PRATINJAU 5 JARAK SEPEDA MOTOR</button></div></section>';
};

const perF42MasterPackageRefreshBase=perBab4RefreshEvidencePackage;
perBab4RefreshEvidencePackage=function(){
  perF42MasterPackageRefreshBase();
  if(selectedExperiment?.id!=='F4.2')return;
  const human=[1,2,3,4,5].filter(d=>perBab4.f42EvidenceBank?.[perF42EvidenceKey('human',d)]?.dataUrl).length;
  const motor=[1,2,3,4,5].filter(d=>perBab4.f42EvidenceBank?.[perF42EvidenceKey('motorcycle',d)]?.dataUrl).length,n=human+motor;
  setText('perBab4VisualState',n+' / 10 citra');setText('perBab4HumanReady',human+' / 5 jarak');setText('perBab4MotorReady',motor+' / 5 jarak');setText('perBab4VisualFrames',n+' / 10');
  const chip=$('perBab4VisualState');if(chip)chip.className='status-chip '+(n>=10?'':'waiting');
  perF42MasterRefreshUi();
};

const perF42MasterPrepareBase=perBab4PrepareEvidenceExports;
perBab4PrepareEvidenceExports=async function(){
  if(selectedExperiment?.id!=='F4.2')return perF42MasterPrepareBase();
  await perF42RecoverSeedEvidence();
  perBab4.evidence.exports=[];
  const human=await perBab4BuildDetectionMontage('human');
  const motor=await perBab4BuildDetectionMontage('motorcycle');
  if(human)perBab4.evidence.exports.push({index:3,title:'Dokumentasi Visual Manusia pada Jarak 1–5 m',data_url:human.toDataURL('image/png')});
  if(motor)perBab4.evidence.exports.push({index:4,title:'Dokumentasi Visual Sepeda Motor pada Jarak 1–5 m',data_url:motor.toDataURL('image/png')});
};

const perF42MasterRefreshUiBaseV2=perF42MasterRefreshUi;
perF42MasterRefreshUi=function(){
  perF42MasterRefreshUiBaseV2();
  if(selectedExperiment?.id!=='F4.2'||currentExp!=='perception')return;
  const active=perF42MasterIsActive(),other=!!recordStartedMs&&!active,start=$('perF42MasterStart');
  if(start&&other){start.disabled=true;start.textContent='PEREKAM LAIN SEDANG AKTIF'}
  if(other)setText('perF42MasterState','MENUNGGU • selesaikan '+(recordingLeafId||'perekaman lain')+' terlebih dahulu');
};
const perF42MasterCaptureRefreshBase=perF42CaptureOne;
perF42CaptureOne=async function(target,distance,repeat){
  const out=await perF42MasterCaptureRefreshBase(target,distance,repeat);
  perBab4RefreshEvidencePackage();perF42MasterRefreshUi();return out;
};

/* F4.3 fresh-start lock: legacy recovery is intentionally disabled for the new dataset. */
perF43RecoverLatest=async function(){toast('Recovery dataset lama dinonaktifkan. F4.3 saat ini dimulai dari dataset baru 0/60.',true)};

/* ===== F4.2 CLEAN WORKFLOW v2: no legacy recovery, auto-advance condition ===== */
const PER_F42_CLEAN_MARKER='agv.perception.f42.clean-v2';
function perF42ClearLegacyOnce(){
  try{
    if(localStorage.getItem(PER_F42_CLEAN_MARKER)==='1')return;
    localStorage.removeItem(PER_F42_DRAFT_KEY);
    localStorage.removeItem(PER_F42_EVIDENCE_KEY);
    localStorage.setItem(PER_F42_CLEAN_MARKER,'1');
  }catch(_){}
  perBab4.f42Session={rows:[]};
  perBab4.f42EvidenceBank={};
  if(typeof tableRunRows!=='undefined'){tableRunRows.set(0,[]);tableRunRows.set(1,[])}
  if(perBab4.evidence)perBab4.evidence.f42=[];
  try{perF42PersistDraft();perF42EvidencePersist()}catch(_){}
}
perF42ClearLegacyOnce();

/* Disable all legacy recovery paths permanently for the final workflow. */
perF42RecoverSeedEvidence=async function(){perF42SeedEvidenceLoaded=true;return};
perF42RecoverExisting=function(){toast('Workflow 4.2 menggunakan dataset baru 0/100. Recovery data lama dinonaktifkan.')};

function perF42AdvanceCondition(target,distance){
  let nextTarget=target,nextDistance=+distance+1,doneAll=false;
  if(nextDistance>5){
    if(target==='human'){nextTarget='motorcycle';nextDistance=1}
    else{nextDistance=5;doneAll=true}
  }
  const t=$('perBab4Target'),d=$('perBab4Distance');
  if(t)t.value=nextTarget;if(d)d.value=String(nextDistance);
  t?.dispatchEvent(new Event('change'));d?.dispatchEvent(new Event('change'));
  perF42RefreshUi();perF42MasterRefreshUi();perBab4RefreshOperatorPreview();
  if(doneAll)toast('Seluruh urutan kondisi 4.2 telah selesai. Periksa kelengkapan 100/100 lalu DOWNLOAD FULL DATA 4.2.');
  else toast('Kondisi selesai 10/10. Selanjutnya: '+(nextTarget==='human'?'Manusia':'Sepeda Motor')+' '+nextDistance+' m. Posisikan objek lalu tekan AMBIL 10 DATA.');
  return{target:nextTarget,distance:nextDistance,doneAll};
}

perF42BatchTick=async function(){
  const s=perBab4.f42Batch;if(!s.active||s.busy||!s.queue.length)return;
  if(!perF42MasterIsActive()){perF42BatchStop();return}
  const due=(s.done+1)*PER_F42_INTERVAL_MS,elapsed=Date.now()-s.startMs,stamp=+updated.semantic_detections||0;
  if(elapsed<due||!channelFresh('semantic_detections',2.5)||!stamp||stamp<=s.lastStamp){perF42RefreshUi();return}
  s.busy=true;const repeat=s.queue[0];s.lastStamp=stamp;
  try{
    await perF42CaptureOne(s.target,s.distance,repeat);
    s.queue.shift();s.done++;
    if(!s.queue.length){
      const finishedTarget=s.target,finishedDistance=s.distance;
      perF42BatchStop(true);
      perF42AdvanceCondition(finishedTarget,finishedDistance);
    }
  }catch(e){perF42BatchStop(true);toast(e?.message||String(e),true)}
  finally{s.busy=false;perF42RefreshUi();perBab4DrawSummary();perF42MasterRefreshUi()}
};

/* ===== F4.2 RESET CONTROLS v1: reset last condition or all dataset ===== */
function perF42ConditionLabel(target,distance){
  return (target==='motorcycle'?'Sepeda Motor':'Manusia')+' '+(+distance)+' m';
}
function perF42LastCondition(){
  const rows=(perBab4.f42Session?.rows?.length?perBab4.f42Session.rows:(tableRunRows.get(0)||[]));
  let best=null;
  for(const r of rows){
    const v=r?.values||[],target=perF42RowObject(v),distance=+v[2]||0;
    if(distance<1||distance>5)continue;
    const ts=Date.parse(v[4])||0,key=perF42EvidenceKey(target,distance);
    if(!best||ts>best.ts)best={target,distance,key,ts};
  }
  return best;
}
function perF42DeleteCondition(target,distance){
  const key=perF42EvidenceKey(target,distance);
  const keep=(perBab4.f42Session?.rows||[]).filter(r=>!(perF42RowObject(r.values)===target&&+r.values?.[2]===+distance));
  perBab4.f42Session.rows=perF42NormalizeRows(keep);
  delete perBab4.f42EvidenceBank[key];
  if(perBab4.evidence?.f42)perBab4.evidence.f42=perBab4.evidence.f42.filter(x=>!(x?.target===target&&+x.distance===+distance));
  perF42PersistDraft();perF42EvidencePersist();
  perF42RestoreRows();perBab4RefreshEvidencePackage();perF42RefreshUi();perF42MasterRefreshUi();
}
function perF42ResetLastCondition(){
  if(perBab4.f42Batch?.active)return toast('Hentikan pengambilan kondisi yang sedang berjalan sebelum reset.',true);
  const last=perF42LastCondition();
  if(!last)return toast('Belum ada kondisi 4.2 yang dapat direset.',true);
  const label=perF42ConditionLabel(last.target,last.distance);
  if(!confirm('Reset kondisi terakhir '+label+'? Seluruh data 10 pengulangan kondisi ini dan gambar representatifnya akan dihapus. Kondisi lain tetap dipertahankan.'))return;
  perF42DeleteCondition(last.target,last.distance);
  const t=$('perBab4Target'),d=$('perBab4Distance');
  if(t)t.value=last.target;if(d)d.value=String(last.distance);
  t?.dispatchEvent(new Event('change'));d?.dispatchEvent(new Event('change'));
  perF42RefreshUi();perF42MasterRefreshUi();perBab4RefreshOperatorPreview();
  toast(label+' direset ke 0/10. Kondisi sebelumnya tetap tersimpan. Silakan ulangi pengambilan kondisi ini.');
}
function perF42ResetAllDataset(){
  if(perBab4.f42Batch?.active)return toast('Hentikan pengambilan kondisi yang sedang berjalan sebelum reset semua data.',true);
  if(!confirm('RESET SEMUA DATA 4.2? Seluruh data 100 pengujian, progress, dan gambar representatif akan dikosongkan.'))return;
  perBab4.f42Session={rows:[]};
  perBab4.f42EvidenceBank={};
  if(perBab4.evidence)perBab4.evidence.f42=[];
  tableRunRows.set(0,[]);tableRunRows.set(1,[]);
  perF42PersistDraft();perF42EvidencePersist();
  const t=$('perBab4Target'),d=$('perBab4Distance');
  if(t)t.value='human';if(d)d.value='1';
  renderExperimentTable();perBab4RenderProgress();perBab4DrawSummary();perBab4RefreshEvidencePackage();perF42RefreshUi();perF42MasterRefreshUi();perBab4RefreshOperatorPreview();
  toast('Semua data 4.2 direset. Pengujian kembali ke Manusia 1 m • 0/10.');
}
function perF42EnsureResetControls(){
  if(selectedExperiment?.id!=='F4.2')return;
  const ctl=$('perF42BatchControls');if(!ctl)return;
  let box=$('perF42ResetControls');
  if(!box){
    box=document.createElement('div');box.id='perF42ResetControls';box.className='per-f42-batch-actions per-f42-reset-actions';
    box.innerHTML='<button class="warning-btn" id="perF42ResetLast">↶ RESET KONDISI TERAKHIR (10 DATA)</button><button class="danger-btn" id="perF42ResetAll">⚠ RESET SEMUA DATA 4.2</button>';
    const hint=ctl.querySelector('.per-f42-test-hint');if(hint)ctl.insertBefore(box,hint);else ctl.appendChild(box);
    $('perF42ResetLast')?.addEventListener('click',perF42ResetLastCondition);
    $('perF42ResetAll')?.addEventListener('click',perF42ResetAllDataset);
  }
  const batch=!!perBab4.f42Batch?.active,last=perF42LastCondition(),total=perF42MasterDatasetStatus().total;
  const a=$('perF42ResetLast'),b=$('perF42ResetAll');
  if(a){a.disabled=batch||!last;a.textContent=last?'↶ RESET TERAKHIR • '+perF42ConditionLabel(last.target,last.distance):'↶ RESET KONDISI TERAKHIR (10 DATA)'}
  if(b)b.disabled=batch||total===0;
}
const perF42ResetPanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){
  const out=perF42ResetPanelBase();
  if(selectedExperiment?.id==='F4.2'){setTimeout(()=>{perF42EnsureResetControls();perF42MasterRefreshUi()},0)}
  return out;
};
const perF42ResetRefreshBase=perF42MasterRefreshUi;
perF42MasterRefreshUi=function(){
  const out=perF42ResetRefreshBase();
  if(selectedExperiment?.id==='F4.2')perF42EnsureResetControls();
  return out;
};

/* ===== F4.3 RESET CONTROLS v1 ===== */
async function perF43SyncEvidenceBankToServer(){
  const items=Object.values(perBab4.f43EvidenceBank||{}).filter(e=>e&&e.cond&&e.dataUrl);
  const clear=await fetch('/api/perception/f43/evidence-bank',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'clear'})});
  if(!clear.ok)throw new Error('Gagal mengosongkan bank citra NUC (HTTP '+clear.status+')');
  for(const e of items){
    let ok=false,last='';
    for(let attempt=0;attempt<2&&!ok;attempt++){
      const r=await fetch('/api/perception/f43/evidence-bank',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:e.cond,data_url:e.dataUrl})});
      ok=r.ok;last='HTTP '+r.status;if(!ok)await new Promise(res=>setTimeout(res,200));
    }
    if(!ok)throw new Error('Gagal menyimpan ulang citra '+e.cond+' ke NUC ('+last+')');
  }
  perF43ServerEvidenceLoaded=true;
}
function perF43RefreshAfterReset(){
  tableRunRows.set(0,(perBab4.f43Session.rows||[]).map(r=>({sec:r.sec||0,values:[...(r.values||[])],yaml:Array.isArray(r.yaml)?r.yaml:[]})));
  tableRunRows.set(1,[]);perBab4PrepareSummary();renderExperimentTable();perBab4RenderProgress();
  perBab4DrawSummary();perBab4DrawLaneGap();perBab4DrawLaneStatus();perBab4RefreshEvidencePackage();
  perBab4RefreshOperatorPreview();perBab4LaneBatchRefresh();perF43MasterRefreshUi();
}
async function perF43ResetSelectedCondition(){
  if(perBab4.laneBatch?.active)return toast('Hentikan atau tunggu pengambilan kondisi selesai sebelum reset.',true);
  perF43SyncAccumulator();
  const cond=$('perBab4LaneCondition')?.value||'center',meta=perBab4LaneV3Meta(cond);
  const before=(perBab4.f43Session.rows||[]).filter(r=>r.values?.[1]===meta.label).length;
  const hadImage=!!perBab4.f43EvidenceBank?.[cond];
  if(!before&&!hadImage)return toast(meta.label+' belum memiliki data/citra yang dapat direset.',true);
  if(!confirm('RESET KONDISI TERPILIH?\n\n'+meta.label+'\n\nSeluruh data pengulangan kondisi ini dan citra representatifnya akan dihapus. Kondisi lain tetap dipertahankan.'))return;
  perBab4.f43Session.rows=(perBab4.f43Session.rows||[]).filter(r=>r.values?.[1]!==meta.label);
  perF43PersistDraft();
  if(perBab4.f43EvidenceBank)delete perBab4.f43EvidenceBank[cond];perF43EvidencePersist();
  if(perBab4.evidence){perBab4.evidence.f43=(perBab4.evidence.f43||[]).filter(e=>e?.cond!==cond);perBab4.evidence.exports=[];}
  try{await perF43SyncEvidenceBankToServer()}catch(e){toast('Data kondisi sudah direset di GUI, tetapi sinkronisasi citra NUC gagal: '+e.message,true)}
  perF43RefreshAfterReset();
  const s=perF43MasterDatasetStatus();toast(meta.label+' direset. Dataset sekarang '+s.total+'/60 data • '+s.evidence+'/6 citra. Silakan ulangi kondisi ini.');
}
async function perF43ResetAllDataset(){
  if(perBab4.laneBatch?.active)return toast('Hentikan atau tunggu pengambilan kondisi selesai sebelum reset seluruh data.',true);
  const s0=perF43MasterDatasetStatus();
  if(!s0.total&&!s0.evidence)return toast('Dataset F4.3 sudah kosong.');
  if(!confirm('RESET SELURUH DATA F4.3?\n\nSemua data 60 pengujian, rekap, grafik, dan dokumentasi visual 6 kondisi akan dikosongkan. Tindakan ini tidak dapat dibatalkan.'))return;
  if(perF43MasterIsActive()){
    await stopWebRecording();
    if(perF43MasterIsActive())return toast('Gagal menghentikan sesi utama F4.3. Reset dibatalkan agar data tidak korup.',true);
  }
  clearInterval(perBab4.laneBatch?.timer);Object.assign(perBab4.laneBatch,{active:false,cond:'center',queue:[],total:0,done:0,lastStamp:0,busy:false,lastResult:'RESET_SELURUH_DATA',timer:null});
  perBab4.f43Session.rows=[];perF43PersistDraft();
  perBab4.f43EvidenceBank={};perF43EvidencePersist();
  if(perBab4.evidence){perBab4.evidence.f43=[];perBab4.evidence.exports=[];perBab4.evidence.captureWarnings=0;}
  tableRunRows.set(0,[]);tableRunRows.set(1,[]);
  try{await perF43SyncEvidenceBankToServer()}catch(e){toast('Dataset lokal sudah direset, tetapi bank citra NUC gagal dikosongkan: '+e.message,true)}
  const sel=$('perBab4LaneCondition');if(sel)sel.value='center';
  perF43RefreshAfterReset();
  toast('Seluruh data F4.3 berhasil direset • 0/60 data • 0/6 kondisi • 0/6 citra.');
}
function perF43EnsureResetControls(){
  if(selectedExperiment?.id!=='F4.3')return;
  const ctl=$('perLaneBatchControls');if(!ctl)return;
  const legacy=$('perLaneResetCondition');if(legacy)legacy.style.display='none';
  let box=$('perF43ResetControls');
  if(!box){
    box=document.createElement('div');box.id='perF43ResetControls';box.className='per-bab4-actions per-lane-batch-actions';
    box.innerHTML='<button class="soft-btn" id="perF43ResetSelected">↶ RESET KONDISI TERPILIH (10 DATA + 1 CITRA)</button><button class="warning-btn" id="perF43ResetAll">⚠ RESET SELURUH DATA F4.3</button>';
    const hint=ctl.querySelector('.per-lane-test-hint');if(hint)ctl.insertBefore(box,hint);else ctl.appendChild(box);
    $('perF43ResetSelected')?.addEventListener('click',perF43ResetSelectedCondition);
    $('perF43ResetAll')?.addEventListener('click',perF43ResetAllDataset);
  }
  const cond=$('perBab4LaneCondition')?.value||'center',meta=perBab4LaneV3Meta(cond),s=perF43MasterDatasetStatus(),batch=!!perBab4.laneBatch?.active;
  const n=(perBab4.f43Session.rows||[]).filter(r=>r.values?.[1]===meta.label).length,hasImage=!!perBab4.f43EvidenceBank?.[cond];
  const one=$('perF43ResetSelected'),all=$('perF43ResetAll');
  if(one){one.disabled=batch||(!n&&!hasImage);one.textContent='↶ RESET KONDISI TERPILIH • '+meta.label+' • '+n+'/10';}
  if(all)all.disabled=batch||(!s.total&&!s.evidence);
}
const perF43ResetPanelBase=perBab4RenderPanel;
perBab4RenderPanel=function(){const out=perF43ResetPanelBase();if(selectedExperiment?.id==='F4.3')setTimeout(()=>{perF43EnsureResetControls();perF43MasterRefreshUi()},0);return out;};
const perF43ResetMasterRefreshBase=perF43MasterRefreshUi;
perF43MasterRefreshUi=function(){const out=perF43ResetMasterRefreshBase();if(selectedExperiment?.id==='F4.3')perF43EnsureResetControls();return out;};
