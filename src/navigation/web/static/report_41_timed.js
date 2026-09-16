'use strict';
const R41_DURATION_SEC=60;
const R41_IDS=new Set(['R4.1.1','R4.1.2','R4.1.3','R4.1.4']);
const R41_PROFILES={
  'R4.1.1':{title:'GNSS • Akuisisi 60 Detik',targetHz:10,primary:'gnss_fix',channels:['gnss_fix','gnss_quality'],countLabels:{gnss_fix:'N Fix',gnss_quality:'N Quality'},metrics:[
    {key:'lat',label:'Latitude',unit:'°',channel:'gnss_fix',pick:v=>r41Num(v?.lat)},
    {key:'lon',label:'Longitude',unit:'°',channel:'gnss_fix',pick:v=>r41Num(v?.lon)},
    {key:'sat',label:'Satelit',unit:'',channel:'gnss_quality',pick:v=>r41Num(v?.sat)},
    {key:'dop',label:'pDOP',unit:'',channel:'gnss_quality',pick:v=>r41Num(v?.dop)},
    {key:'hdop',label:'HDOP',unit:'',channel:'gnss_quality',pick:v=>r41Num(v?.hdop)},
    {key:'hacc',label:'hAcc',unit:'m',channel:'gnss_quality',pick:v=>r41Num(v?.hacc_m)},
    {key:'speed',label:'Ground speed',unit:'m/s',channel:'gnss_quality',pick:v=>r41Num(v?.ground_speed_mps)}]},
  'R4.1.2':{title:'Stabilitas Keluaran Gyro IMU Saat Diam • 60 Detik',targetHz:50,primary:'imu',channels:['imu'],countLabels:{imu:'N IMU'},metrics:[
    {key:'gx',label:'Gyro X',unit:'rad/s',channel:'imu',pick:v=>r41Num(v?.gx)},
    {key:'gy',label:'Gyro Y',unit:'rad/s',channel:'imu',pick:v=>r41Num(v?.gy)},
    {key:'gz',label:'Gyro Z',unit:'rad/s',channel:'imu',pick:v=>r41Num(v?.gz)}]},
  'R4.1.3':{title:'Encoder Steering & Feedback ESC • Akuisisi 60 Detik',targetHz:50,primary:'esc_steer_feedback_raw',channels:['esc_drive_target','esc_drive_raw','esc_steer_target','esc_steer_feedback_raw','gnss_quality'],countLabels:{esc_drive_raw:'N Drive',esc_steer_feedback_raw:'N Encoder',gnss_quality:'N GNSS'},metrics:[
    {key:'drive_cmd',label:'V perintah',unit:'m/s',channel:'esc_drive_target',pick:v=>r41Num(v)},
    {key:'drive_raw',label:'V ESC raw',unit:'m/s',channel:'esc_drive_raw',pick:v=>r41Num(v)},
    {key:'gnss_speed',label:'V GNSS raw',unit:'m/s',channel:'gnss_quality',pick:v=>r41Num(v?.ground_speed_mps)},
    {key:'steer_cmd',label:'Steering perintah',unit:'rad',channel:'esc_steer_target',pick:v=>r41Num(v)},
    {key:'steer_raw',label:'Encoder steering raw',unit:'rad',channel:'esc_steer_feedback_raw',pick:v=>r41Num(v)}]},
  'R4.1.4':{title:'Karakteristik Heading NEO3 & Inersia • Akuisisi 60 Detik',targetHz:50,primary:'imu',channels:['neo3_mag_heading','r41_inertia_heading','imu','neo3_mag'],countLabels:{neo3_mag_heading:'N NEO3 heading',r41_inertia_heading:'N Inersia',imu:'N IMU',neo3_mag:'N NEO3 mag'},detailHint:'Heading NEO3 dan inersia direkam pada rate sumber aktual; gyro IMU ditargetkan 50 Hz. Mean heading dihitung secara circular, bukan mean aritmatika biasa.',metrics:[
    {key:'neo_compass',label:'NEO3 compass',unit:'°',channel:'neo3_mag_heading',angular:true,pick:v=>r41RadToCompassDeg(v?.yaw_rad)},
    {key:'inertia_compass',label:'Inersia compass',unit:'°',channel:'r41_inertia_heading',angular:true,pick:v=>r41RadToCompassDeg(v?.yaw_rad)},
    {key:'delta_heading',label:'Δ Inersia−NEO3',unit:'°',channel:'r41_inertia_heading',pick:()=>report41HeadingMetric('derived.r41_heading_delta_deg')},
    {key:'inertia_drift',label:'Drift inersia',unit:'°',channel:'r41_inertia_heading',pick:()=>report41HeadingMetric('derived.r41_heading_drift_deg')},
    {key:'gyro_z',label:'Gyro Z',unit:'rad/s',channel:'imu',pick:v=>r41Num(v?.gz)},
    {key:'field_norm',label:'NEO3 field norm',unit:'µT',channel:'neo3_mag',pick:v=>r41Num(v?.norm_ut)}]}
};
let r41Session=null,r41Timer=null,r41LastRender=0;function r41Num(v){const n=Number(v);return Number.isFinite(n)?n:NaN}
function r41Profile(id=String(selectedExperiment?.id||'')){return R41_PROFILES[id]||null}
function report41RequestedRate(id=String(selectedExperiment?.id||'')){return r41Profile(id)?.targetHz||null}
function report41IsSelected(){return currentExp==='navigation'&&!!r41Profile()}
function report41TimedActive(){return !!r41Session&&r41Session.state==='RECORDING'}
function r41Mean(a){const v=a.filter(Number.isFinite);return v.length?v.reduce((s,x)=>s+x,0)/v.length:NaN}
function r41Std(a){const v=a.filter(Number.isFinite);if(!v.length)return NaN;const m=r41Mean(v);return Math.sqrt(v.reduce((s,x)=>s+(x-m)*(x-m),0)/v.length)}
function r41WrapDeg180(v){const n=r41Num(v);return Number.isFinite(n)?((n+180)%360+360)%360-180:NaN}
function r41WrapDeg360(v){const n=r41Num(v);return Number.isFinite(n)?((n%360)+360)%360:NaN}
function r41RadToCompassDeg(v){const n=r41Num(v);return Number.isFinite(n)?r41WrapDeg360(90-n*180/Math.PI):NaN}
function r41CircularMean(a){const v=a.filter(Number.isFinite);if(!v.length)return NaN;const sx=v.reduce((s,x)=>s+Math.cos(x*Math.PI/180),0),sy=v.reduce((s,x)=>s+Math.sin(x*Math.PI/180),0);return r41WrapDeg360(Math.atan2(sy,sx)*180/Math.PI)}
function r41CircularStd(a){const v=a.filter(Number.isFinite);if(!v.length)return NaN;const sx=v.reduce((s,x)=>s+Math.cos(x*Math.PI/180),0)/v.length,sy=v.reduce((s,x)=>s+Math.sin(x*Math.PI/180),0)/v.length,R=Math.min(1,Math.hypot(sx,sy));return Math.sqrt(Math.max(0,-2*Math.log(Math.max(1e-12,R))))*180/Math.PI}
const r41InertiaModel={fallbackYawRad:NaN,lastImuStampMs:0,directYawRad:NaN,directStampMs:0,source:'WAIT'};
function r41WrapRad(v){const n=r41Num(v);return Number.isFinite(n)?((n+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI:NaN}
function r41ResetInertiaModel(){const neo=r41Num(state?.neo3_mag_heading?.yaw_rad),imu=r41Num(state?.imu?.yaw_rad);r41InertiaModel.fallbackYawRad=Number.isFinite(neo)?r41WrapRad(neo):(Number.isFinite(imu)?r41WrapRad(imu):NaN);r41InertiaModel.lastImuStampMs=0;r41InertiaModel.directYawRad=NaN;r41InertiaModel.directStampMs=0;r41InertiaModel.source=Number.isFinite(neo)?'NEO3 anchor + integral gyro-Z':(Number.isFinite(imu)?'IMU orientation + integral gyro-Z':'WAIT')}
function r41UpdateInertiaModel(delta={},ups={}){const now=Date.now();if(Object.prototype.hasOwnProperty.call(delta,'imu_inertial_heading')){const y=r41Num(delta?.imu_inertial_heading?.yaw_rad);if(Number.isFinite(y)){r41InertiaModel.directYawRad=r41WrapRad(y);r41InertiaModel.directStampMs=Number(ups?.imu_inertial_heading)||now}}if(Object.prototype.hasOwnProperty.call(delta,'neo3_mag_heading')&&!Number.isFinite(r41InertiaModel.fallbackYawRad)){const y=r41Num(delta?.neo3_mag_heading?.yaw_rad);if(Number.isFinite(y)){r41InertiaModel.fallbackYawRad=r41WrapRad(y);r41InertiaModel.source='NEO3 anchor + integral gyro-Z'}}if(Object.prototype.hasOwnProperty.call(delta,'imu')){const stamp=Number(ups?.imu)||now,gz=r41Num(delta?.imu?.gz);if(!Number.isFinite(r41InertiaModel.fallbackYawRad)){const neo=r41Num(state?.neo3_mag_heading?.yaw_rad),iy=r41Num(delta?.imu?.yaw_rad??state?.imu?.yaw_rad);if(Number.isFinite(neo)){r41InertiaModel.fallbackYawRad=r41WrapRad(neo);r41InertiaModel.source='NEO3 anchor + integral gyro-Z'}else if(Number.isFinite(iy)){r41InertiaModel.fallbackYawRad=r41WrapRad(iy);r41InertiaModel.source='IMU orientation + integral gyro-Z'}}const dt=r41InertiaModel.lastImuStampMs>0?(stamp-r41InertiaModel.lastImuStampMs)/1000:0;r41InertiaModel.lastImuStampMs=stamp;if(Number.isFinite(r41InertiaModel.fallbackYawRad)&&Number.isFinite(gz)&&dt>0&&dt<=.2)r41InertiaModel.fallbackYawRad=r41WrapRad(r41InertiaModel.fallbackYawRad+gz*dt)}}
function r41InertiaHeading(){const now=Date.now();if(Number.isFinite(r41InertiaModel.directYawRad)&&now-r41InertiaModel.directStampMs<=750)return{yawRad:r41InertiaModel.directYawRad,source:'/imu/inertial_heading',direct:true};return{yawRad:r41InertiaModel.fallbackYawRad,source:r41InertiaModel.source,direct:false}}
function r41MetricMean(metric,a){return metric?.angular?r41CircularMean(a):r41Mean(a)}
function r41MetricStd(metric,a){return metric?.angular?r41CircularStd(a):r41Std(a)}
function r41HeadingPair(){const n=r41Num(state?.neo3_mag_heading?.yaw_rad),ih=r41InertiaHeading(),neo=r41RadToCompassDeg(n),inertia=r41RadToCompassDeg(ih.yawRad);return{neo,inertia,inertiaSource:ih.source,inertiaDirect:ih.direct,delta:Number.isFinite(neo)&&Number.isFinite(inertia)?r41WrapDeg180(inertia-neo):NaN}}
function report41HeadingMetric(path){const h=r41HeadingPair();if(path==='derived.r41_heading_neo3_compass_deg')return h.neo;if(path==='derived.r41_heading_inertia_compass_deg')return h.inertia;if(path==='derived.r41_heading_delta_deg')return h.delta;if(path==='derived.r41_heading_drift_deg'){if(!r41Session||r41Session.id!=='R4.1.4'||!Number.isFinite(h.delta))return NaN;if(!Number.isFinite(r41Session.headingBaselineDeltaDeg))r41Session.headingBaselineDeltaDeg=h.delta;return r41WrapDeg180(h.delta-r41Session.headingBaselineDeltaDeg)}return NaN}
function r41Fmt(v,d=4){return Number.isFinite(v)?Number(v).toFixed(d):'--'}
function r41Bucket(sec){if(!r41Session.buckets.has(sec)){const p=r41Session.profile,b={sec,counts:{},metrics:{}};p.channels.forEach(c=>b.counts[c]=0);p.metrics.forEach(m=>b.metrics[m.key]=[]);r41Session.buckets.set(sec,b)}return r41Session.buckets.get(sec)}
function r41OverallMetric(metric){const out=[];for(const b of r41Session?.buckets?.values?.()||[]){const a=b.metrics[metric.key]||[];out.push(...a)}return out}
function r41ElapsedSec(now=Date.now()){if(!r41Session)return 0;const end=r41Session.finishedMs||now;return Math.max(0,(end-r41Session.startedMs)/1000)}
function r41RemainingSec(){return Math.max(0,R41_DURATION_SEC-r41ElapsedSec())}
function report41WriteCountdown(){if(!report41TimedActive())return;const rem=r41RemainingSec(),m=Math.floor(rem/60),s=rem-m*60;setText('recordElapsed',`COUNTDOWN ${String(m).padStart(2,'0')}:${s.toFixed(1).padStart(4,'0')}`)}
function r41Rate(channel){if(!r41Session)return NaN;const n=r41Session.channelCounts[channel]||0,d=Math.min(R41_DURATION_SEC,Math.max(.001,r41ElapsedSec()));return n/d}
function r41RateOk(){if(!r41Session)return false;const r=r41Rate(r41Session.profile.primary),t=r41Session.profile.targetHz;return Number.isFinite(r)&&r>=t*.90&&r<=t*1.10}
function r41OverallSummary(){if(!r41Session)return[];return r41Session.profile.metrics.map(m=>{const a=r41OverallMetric(m),mean=r41MetricMean(m,a);let min=a.length?Math.min(...a):NaN,max=a.length?Math.max(...a):NaN;if(m.angular&&Number.isFinite(mean)&&a.length){const u=a.map(x=>mean+r41WrapDeg180(x-mean));min=r41WrapDeg360(Math.min(...u));max=r41WrapDeg360(Math.max(...u))}return{...m,mean,std:r41MetricStd(m,a),n:a.length,min,max}})}function captureReport41TimedDelta(delta,ups={}){
  r41UpdateInertiaModel(delta,ups);
  if(!report41TimedActive()||String(selectedExperiment?.id||'')!==r41Session.id)return;
  let workDelta=delta,workUps=ups;
  if(r41Session.id==='R4.1.4'&&(Object.prototype.hasOwnProperty.call(delta,'imu')||Object.prototype.hasOwnProperty.call(delta,'imu_inertial_heading'))){const ih=r41InertiaHeading();if(Number.isFinite(ih.yawRad)){workDelta={...delta,r41_inertia_heading:{yaw_rad:ih.yawRad,source:ih.source,direct:ih.direct}};workUps={...ups,r41_inertia_heading:Number(ups?.imu_inertial_heading??ups?.imu) || Date.now()}}}
  const p=r41Session.profile,now=Date.now(),elapsed=(now-r41Session.startedMs)/1000;
  if(elapsed<0||elapsed>=R41_DURATION_SEC)return;
  const sec=Math.max(0,Math.min(59,Math.floor(elapsed))),bucket=r41Bucket(sec);
  for(const channel of p.channels){
    if(!Object.prototype.hasOwnProperty.call(workDelta,channel))continue;
    const stamp=Number(workUps?.[channel]??now),last=r41Session.lastStamps[channel];
    if(Number.isFinite(last)&&Number.isFinite(stamp)&&stamp<=last)continue;
    r41Session.lastStamps[channel]=stamp;r41Session.channelCounts[channel]=(r41Session.channelCounts[channel]||0)+1;bucket.counts[channel]=(bucket.counts[channel]||0)+1;
    const value=workDelta[channel],raw={elapsed_s:elapsed,second:sec+1,channel,stamp_ms:stamp};
    if(value&&typeof value==='object'&&!Array.isArray(value))Object.assign(raw,value);else raw.value=value;
    r41Session.raw.push(raw);
    for(const metric of p.metrics){if(metric.channel!==channel)continue;const v=metric.pick(value,state,r41Session);if(Number.isFinite(v))bucket.metrics[metric.key].push(v)}
  }
  if(now-r41LastRender>180){r41LastRender=now;renderReport41TimedPanel()}
}
function startReport41TimedEvidence(id){
  const p=r41Profile(id);if(!p)return;
  if(r41Timer)clearInterval(r41Timer);
  if(id==='R4.1.4')r41ResetInertiaModel();
  r41Session={id,profile:p,state:'RECORDING',startedMs:Date.now(),finishedMs:0,buckets:new Map(),raw:[],lastStamps:{},channelCounts:{},autoStopRequested:false,headingBaselineDeltaDeg:NaN};
  p.channels.forEach(c=>r41Session.channelCounts[c]=0);renderReport41TimedPanel();report41WriteCountdown();
  r41Timer=setInterval(()=>{if(!r41Session||r41Session.state!=='RECORDING')return;report41WriteCountdown();renderReport41TimedPanel();if(r41ElapsedSec()>=R41_DURATION_SEC&&!r41Session.autoStopRequested){r41Session.autoStopRequested=true;r41Session.state='FINALIZING';renderReport41TimedPanel();Promise.resolve(window.stopWebRecording?.()).catch(()=>{})}},100);
}
function finishReport41TimedEvidence(reason='STOP'){
  if(!r41Session||!['RECORDING','FINALIZING'].includes(r41Session.state))return;
  if(r41Timer){clearInterval(r41Timer);r41Timer=null}r41Session.finishedMs=Date.now();
  const elapsed=r41ElapsedSec(r41Session.finishedMs),complete=elapsed>=59.5;r41Session.state=complete?'DONE':'STOPPED EARLY';r41Session.reason=reason;renderReport41TimedPanel();
}
function resetReport41TimedEvidence(){if(report41TimedActive())return toast('STOP recording sebelum reset.',true);r41Session=null;if(r41Timer){clearInterval(r41Timer);r41Timer=null}renderReport41TimedPanel()}
function r41SecondRows(){if(!r41Session)return[];const rows=[];for(let sec=0;sec<60;sec++){const b=r41Session.buckets.get(sec);if(!b&&sec>=Math.ceil(Math.min(60,r41ElapsedSec())))break;rows.push({sec:sec+1,b:b||r41Bucket(sec)})}return rows}
function r41CountsText(b){const p=r41Session.profile,keys=Object.keys(p.countLabels||{});return keys.map(k=>`${p.countLabels[k]} ${b.counts[k]||0}`).join(' • ')}
function r41RateDetails(){if(!r41Session)return'--';const p=r41Session.profile;return Object.keys(p.countLabels||{}).map(k=>`${p.countLabels[k].replace(/^N /,'')} ${r41Rate(k).toFixed(2)} Hz`).join(' • ')}
function r41MetricHeaders(profile=r41Session?.profile||r41Profile()){return (profile?.metrics||[]).map(m=>`<th>${escapeHtml(m.label)}${m.unit?`<small>${escapeHtml(m.unit)}</small>`:''}</th>`).join('')}
function r41SummaryGraphRows(){return r41Session?r41OverallSummary().filter(m=>Number.isFinite(m.mean)&&m.n>0):[]}
function r41SummaryGraphFmt(metric,v){if(!Number.isFinite(v))return'--';const x=metric?.angular?r41WrapDeg360(v):v,a=Math.abs(x);const d=a>=100?2:a>=10?3:a>=1?4:5;return `${Number(x).toFixed(d)}${metric?.unit?` ${metric.unit}`:''}`}
function r41SummaryGraphRange(metric){const mean=r41Num(metric?.mean),std=Math.abs(r41Num(metric?.std)),min=r41Num(metric?.min),max=r41Num(metric?.max);if(!Number.isFinite(mean))return[-1,1];let lo=min,hi=max;if(metric?.angular){const spread=[min,max].filter(Number.isFinite).map(v=>Math.abs(r41WrapDeg180(v-mean)));const half=Math.max(.05,Number.isFinite(std)?std:0,...spread);lo=mean-half;hi=mean+half}else{if(Number.isFinite(std)){lo=Number.isFinite(lo)?Math.min(lo,mean-std):mean-std;hi=Number.isFinite(hi)?Math.max(hi,mean+std):mean+std}if(!Number.isFinite(lo))lo=mean;if(!Number.isFinite(hi))hi=mean}if(!(hi>lo)){const e=Math.max(Math.abs(mean)*1e-6,Math.abs(mean)<1?.01:.001);lo=mean-e;hi=mean+e}const pad=(hi-lo)*.12;return[lo-pad,hi+pad]}
function r41DrawSummaryGraph(canvas,{exportMode=false}={}){if(!canvas)return;const rows=r41SummaryGraphRows(),rect=canvas.getBoundingClientRect(),cssW=exportMode?3840:Math.max(320,Math.round(rect.width||1200)),cssH=exportMode?2160:Math.max(320,120+rows.length*82),dpr=exportMode?1:Math.min(window.devicePixelRatio||1,2);canvas.width=Math.round(cssW*dpr);canvas.height=Math.round(cssH*dpr);if(!exportMode)canvas.style.height=`${cssH}px`;const ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height,S=dpr,dark=!exportMode&&!document.body.classList.contains('light'),bg=dark?'#111719':'#ffffff',fg=dark?'#eef5f5':'#111827',muted=dark?'#9fb0b4':'#64748b',grid=dark?'rgba(160,185,190,.20)':'rgba(100,116,139,.20)',accent='#2f9e73',stdFill=dark?'rgba(72,224,164,.20)':'rgba(47,158,115,.16)',rowBg=dark?'rgba(255,255,255,.025)':'rgba(15,23,42,.025)';ctx.fillStyle=bg;ctx.fillRect(0,0,w,h);const pad=exportMode?150:18*S,titleH=exportMode?250:64*S;ctx.fillStyle=fg;ctx.textBaseline='middle';ctx.font=`700 ${exportMode?64:16*S}px system-ui`;ctx.fillText(`${r41Session?.id||'4.1'} • Grafik Ringkasan 60 Detik`,pad,exportMode?92:titleH*.42);ctx.fillStyle=muted;ctx.font=`${exportMode?34:10*S}px system-ui`;ctx.fillText('Sumber sama dengan tabel ringkasan • titik = mean • pita = ±1 SD • garis = min–max • setiap parameter memakai skala dan unit sendiri',pad,exportMode?160:titleH*.78);if(!rows.length){ctx.fillStyle=muted;ctx.font=`600 ${exportMode?42:13*S}px system-ui`;ctx.textAlign='center';ctx.fillText('Belum ada data ringkasan. Tekan START dan selesaikan akuisisi.',w/2,h/2);ctx.textAlign='left';return}const usableH=h-titleH-(exportMode?170:30*S),rowH=Math.max(exportMode?190:66*S,usableH/rows.length),labelW=Math.min(exportMode?720:220*S,w*.27),statsW=Math.min(exportMode?760:220*S,w*.28),x0=pad+labelW,x1=w-pad-statsW,trackW=Math.max(80*S,x1-x0);rows.forEach((m,i)=>{const y=titleH+i*rowH+rowH*.48,top=y-rowH*.36;ctx.fillStyle=rowBg;ctx.fillRect(pad,top,w-2*pad,rowH*.72);ctx.fillStyle=fg;ctx.font=`700 ${exportMode?38:11*S}px system-ui`;ctx.fillText(m.label,pad+(exportMode?24:8*S),y-(exportMode?18:5*S));ctx.fillStyle=muted;ctx.font=`${exportMode?29:9*S}px system-ui`;ctx.fillText(`${m.unit||'tanpa unit'} • N=${m.n}`,pad+(exportMode?24:8*S),y+(exportMode?28:11*S));let[lo,hi]=r41SummaryGraphRange(m);const px=v=>x0+(v-lo)/Math.max(1e-12,hi-lo)*trackW;ctx.strokeStyle=grid;ctx.lineWidth=exportMode?4:1*S;ctx.beginPath();ctx.moveTo(x0,y);ctx.lineTo(x1,y);ctx.stroke();const minv=m.angular?m.mean+r41WrapDeg180(m.min-m.mean):m.min,maxv=m.angular?m.mean+r41WrapDeg180(m.max-m.mean):m.max,std=Number.isFinite(m.std)?Math.abs(m.std):0,mean=m.mean;const a=Math.min(px(minv),px(maxv)),b=Math.max(px(minv),px(maxv));ctx.strokeStyle=dark?'#76c9ff':'#2563eb';ctx.lineWidth=exportMode?10:3*S;ctx.beginPath();ctx.moveTo(a,y);ctx.lineTo(b,y);ctx.stroke();const sa=Math.max(x0,px(mean-std)),sb=Math.min(x1,px(mean+std));ctx.fillStyle=stdFill;ctx.fillRect(Math.min(sa,sb),y-(exportMode?34:9*S),Math.max(exportMode?4:1*S,Math.abs(sb-sa)),exportMode?68:18*S);ctx.fillStyle=accent;ctx.beginPath();ctx.arc(px(mean),y,exportMode?16:5*S,0,Math.PI*2);ctx.fill();ctx.strokeStyle=accent;ctx.lineWidth=exportMode?5:1.5*S;ctx.beginPath();ctx.moveTo(px(mean),y-(exportMode?40:11*S));ctx.lineTo(px(mean),y+(exportMode?40:11*S));ctx.stroke();ctx.fillStyle=muted;ctx.font=`${exportMode?25:8*S}px ui-monospace`;ctx.textAlign='left';ctx.fillText(r41SummaryGraphFmt(m,lo),x0,y+(exportMode?62:18*S));ctx.textAlign='right';ctx.fillText(r41SummaryGraphFmt(m,hi),x1,y+(exportMode?62:18*S));ctx.fillStyle=fg;ctx.font=`700 ${exportMode?34:10*S}px ui-monospace`;ctx.fillText(`μ ${r41SummaryGraphFmt(m,mean)}`,w-pad-(exportMode?24:8*S),y-(exportMode?18:5*S));ctx.fillStyle=muted;ctx.font=`${exportMode?27:8.5*S}px ui-monospace`;ctx.fillText(`σ ${r41SummaryGraphFmt(m,m.std)}`,w-pad-(exportMode?24:8*S),y+(exportMode?28:11*S));ctx.textAlign='left'});if(exportMode){ctx.fillStyle=muted;ctx.font='28px system-ui';ctx.fillText(`Durasi ${r41ElapsedSec().toFixed(2)} s • source ${r41Session?.profile?.primary||'--'} • capture ${r41Rate(r41Session?.profile?.primary).toFixed(2)} Hz • ${new Date().toLocaleString('id-ID')}`,pad,h-74)}}
function r41DownloadSummaryGraphPng(){if(!r41Session||!r41SummaryGraphRows().length)return toast('Belum ada data ringkasan untuk diekspor.',true);const out=document.createElement('canvas');r41DrawSummaryGraph(out,{exportMode:true});out.toBlob(blob=>{if(!blob)return toast('Gagal membuat PNG grafik ringkasan.',true);const u=URL.createObjectURL(blob),a=document.createElement('a');a.href=u;a.download=`${r41Session.id}_grafik_ringkasan_60s_${stamp()}.png`;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(u),1000);toast('PNG grafik ringkasan 4.1 disimpan.')},'image/png')}
function r41SecondCells(b){return r41Session.profile.metrics.map(m=>`<td>${r41Fmt(r41MetricMean(m,b.metrics[m.key]||[]),m.key.includes('lat')||m.key.includes('lon')?7:4)}</td>`).join('')}
function r41HeadingLiveHtml(){if(String(selectedExperiment?.id||'')!=='R4.1.4')return'';const h=r41HeadingPair(),neo=obj('neo3_mag_heading'),ine=obj('imu_inertial_heading'),mag=obj('neo3_mag'),imu=obj('imu'),drift=report41HeadingMetric('derived.r41_heading_drift_deg'),pairLive=Number.isFinite(h.neo)&&Number.isFinite(h.inertia),source=h.inertiaSource||'WAIT';return `<div class="r41-heading-live"><div class="r41-heading-source"><span>NEO3 MAG • ABSOLUTE</span><b>${r41Fmt(h.neo,2)}°</b><small>ROS yaw ${r41Fmt(r41Num(neo.yaw_rad)*180/Math.PI,2)}° • field ${r41Fmt(r41Num(mag.norm_ut),2)} µT</small></div><div class="r41-heading-source"><span>INERSIA • CONTINUOUS</span><b>${r41Fmt(h.inertia,2)}°</b><small>gyro-Z ${r41Fmt(r41Num(imu.gz),4)} rad/s • ${escapeHtml(source)}${h.inertiaDirect&&Number.isFinite(r41Num(ine.yaw_variance))?` • var ${r41Fmt(r41Num(ine.yaw_variance),5)}`:''}</small></div><div class="r41-heading-source comparison"><span>COMPARISON</span><b>Δ ${r41Fmt(h.delta,2)}°</b><small>drift dari awal ${r41Fmt(drift,2)}° • ${pairLive?'PAIR LIVE':'WAITING SOURCE'}</small></div></div>`}
function renderReport41TimedPanel(){
  const box=$('report41TimedPanel');if(!box)return;const p=r41Profile();
  if(currentExp!=='navigation'||!p){box.hidden=true;box.innerHTML='';const rate=$('runSampleRate');if(rate){rate.readOnly=false;rate.max='100'}return}
  box.hidden=false;const rateInput=$('runSampleRate');if(rateInput&&!recordStartedMs){rateInput.value=String(p.targetHz);rateInput.readOnly=true;rateInput.max='100';rateInput.title=`R4.1 source-rate recorder dikunci ${p.targetHz} Hz untuk subbagian ini.`}
  const s=r41Session&&r41Session.id===String(selectedExperiment?.id||'')?r41Session:null,source=channelRate(p.primary),elapsed=s?Math.min(60,r41ElapsedSec()):0,remaining=Math.max(0,60-elapsed),effective=s?r41Rate(p.primary):NaN;
  const status=!s?'READY':s.state,rateState=s&&['DONE','STOPPED EARLY'].includes(s.state)?(r41RateOk()?'RATE PASS':'RATE CHECK'):'LIVE';
  const summary=s?r41OverallSummary():[],detail=s?r41SecondRows():[];
  const summaryRows=summary.length?summary.map(m=>`<tr><td>${escapeHtml(m.label)}</td><td>${r41Fmt(m.mean,m.key.includes('lat')||m.key.includes('lon')?7:5)}</td><td>${r41Fmt(m.std,5)}</td><td>${r41Fmt(m.min,5)}</td><td>${r41Fmt(m.max,5)}</td><td>${m.n}</td><td>${escapeHtml(m.unit||'')}</td></tr>`).join(''):'<tr><td colspan="7">Tekan START untuk memulai akuisisi 60 detik.</td></tr>';
  const detailRows=detail.length?detail.map(({sec,b})=>`<tr><td>${sec}</td><td>${escapeHtml(r41CountsText(b))}</td>${r41SecondCells(b)}</tr>`).join(''):`<tr><td colspan="${2+p.metrics.length}">Belum ada detail per detik.</td></tr>`;
  box.innerHTML=`<article class="panel r41-timed-card"><div class="panel-head"><div><span class="eyebrow">4.1 SOURCE-RATE EVIDENCE</span><h2>${escapeHtml(p.title)}</h2><p>START → countdown 60 detik → semua update unik pada rate sumber direkam → setiap detik dirata-ratakan → mean keseluruhan dihitung dari seluruh sampel aktual.</p></div><span class="status-chip ${status==='DONE'&&r41RateOk()?'':'waiting'}">${escapeHtml(status)}</span></div>
  <div class="r41-timed-kpis"><div><span>Countdown</span><b>${remaining.toFixed(1)} s</b></div><div><span>Target source</span><b>${p.targetHz} Hz</b></div><div><span>Source live</span><b>${Number.isFinite(source)?source.toFixed(2):'--'} Hz</b></div><div><span>Capture efektif</span><b>${Number.isFinite(effective)?effective.toFixed(2):'--'} Hz</b></div><div><span>Durasi</span><b>${elapsed.toFixed(1)} s</b></div><div><span>Rate gate</span><b>${escapeHtml(rateState)}</b></div></div>
  <div class="r41-rate-line">${s?escapeHtml(r41RateDetails()):`Primary source ${escapeHtml(p.primary)} • target ${p.targetHz} Hz`} ${s?` • raw event ${s.raw.length}`:''}</div>
  ${r41HeadingLiveHtml()}
  <div class="r41-timed-actions"><button class="soft-btn" data-r41-export="second" ${s?'':'disabled'}>CSV Detail per Detik</button><button class="soft-btn" data-r41-export="raw" ${s?'':'disabled'}>CSV Raw Source-rate</button><button class="soft-btn" data-r41-reset ${report41TimedActive()?'disabled':''}>Reset Evidence 4.1</button></div>
  <section class="r41-table-section"><div class="direct-table-title"><span>MEAN</span><b>Ringkasan Seluruh Periode</b><small>Mean/Std dihitung dari semua sampel aktual selama recording, bukan dari 1 nilai per detik.</small></div><div class="table-scroll"><table class="data-table"><thead><tr><th>Parameter</th><th>Mean</th><th>Std</th><th>Min</th><th>Max</th><th>N</th><th>Unit</th></tr></thead><tbody>${summaryRows}</tbody></table></div></section>
  <section class="r41-summary-graph-section"><div class="direct-table-title"><span>GRAPH</span><b>Grafik Ringkasan Seluruh Periode</b><small>Visualisasi langsung dari tabel ringkasan di atas. Tiap parameter memakai skala sendiri agar perbedaan unit tidak menyesatkan.</small></div><div class="r41-summary-graph-wrap"><canvas id="r41SummaryGraph" aria-label="Grafik ringkasan 60 detik"></canvas></div><div class="r41-summary-legend"><span><i class="mean"></i>Mean</span><span><i class="std"></i>±1 SD</span><span><i class="range"></i>Min–Max</span><button class="soft-btn" data-r41-summary-png ${summary.length?'':'disabled'}>PNG Grafik Ringkasan</button></div></section>
  <section class="r41-table-section"><div class="direct-table-title"><span>DETAIL 1 s</span><b>Mean Sampel per Detik</b><small>${escapeHtml(p.detailHint||'Mean per detik dihitung dari seluruh update unik pada source-rate aktual; N aktual setiap source selalu ditampilkan.')}</small></div><div class="table-scroll r41-detail-scroll"><table class="data-table"><thead><tr><th>Detik</th><th>N source</th>${r41MetricHeaders()}</tr></thead><tbody>${detailRows}</tbody></table></div></section></article>`;
  box.onclick=e=>{const ex=e.target.closest('[data-r41-export]');if(ex)return exportReport41Csv(ex.dataset.r41Export);if(e.target.closest('[data-r41-summary-png]'))return r41DownloadSummaryGraphPng();if(e.target.closest('[data-r41-reset]'))resetReport41TimedEvidence()};
  requestAnimationFrame(()=>r41DrawSummaryGraph($('r41SummaryGraph')));
}
function r41CsvEscape(v){return '"'+String(v??'').replaceAll('"','""')+'"'}
function exportReport41Csv(kind='second'){
  if(!r41Session)return toast('Belum ada evidence 4.1.',true);const p=r41Session.profile,lines=[];
  if(kind==='raw'){
    const keys=[...new Set(r41Session.raw.flatMap(r=>Object.keys(r)))],head=['section_id',...keys];lines.push(head.map(r41CsvEscape).join(','));
    for(const r of r41Session.raw)lines.push([r41Session.id,...keys.map(k=>r[k]??'')].map(r41CsvEscape).join(','));
  }else{
    const countKeys=Object.keys(p.countLabels||{}),head=['second',...countKeys.map(k=>p.countLabels[k]),...p.metrics.map(m=>`${m.label}${m.unit?' ['+m.unit+']':''}`)];lines.push(head.map(r41CsvEscape).join(','));
    for(const {sec,b} of r41SecondRows())lines.push([sec,...countKeys.map(k=>b.counts[k]||0),...p.metrics.map(m=>r41Mean(b.metrics[m.key]||[]))].map(r41CsvEscape).join(','));
    lines.push('');lines.push(['parameter','mean','std','min','max','samples','unit'].map(r41CsvEscape).join(','));
    for(const m of r41OverallSummary())lines.push([m.label,m.mean,m.std,m.min,m.max,m.n,m.unit||''].map(r41CsvEscape).join(','));
  }
  downloadBlob(`${r41Session.id}_${kind==='raw'?'raw_source_rate':'mean_per_second'}_${stamp()}.csv`,'text/csv;charset=utf-8',lines.join('\n')+'\n');toast(kind==='raw'?'CSV raw source-rate disimpan.':'CSV mean per detik disimpan.')
}
window.report41HeadingMetric=report41HeadingMetric;
window.report41RequestedRate=report41RequestedRate;
window.report41TimedActive=report41TimedActive;
window.report41WriteCountdown=report41WriteCountdown;
window.captureReport41TimedDelta=captureReport41TimedDelta;
window.startReport41TimedEvidence=startReport41TimedEvidence;
window.finishReport41TimedEvidence=finishReport41TimedEvidence;
window.renderReport41TimedPanel=renderReport41TimedPanel;
