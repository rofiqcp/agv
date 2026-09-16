'use strict';
const R41_DURATION_SEC=60;
const R41_IDS=new Set(['R4.1.1','R4.1.2','R4.1.3']);
const R41_PROFILES={
  'R4.1.1':{title:'GNSS • Akuisisi 60 Detik',targetHz:5,primary:'gnss_fix',channels:['gnss_fix','gnss_quality'],countLabels:{gnss_fix:'N Fix',gnss_quality:'N Quality'},metrics:[
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
    {key:'steer_raw',label:'Encoder steering raw',unit:'rad',channel:'esc_steer_feedback_raw',pick:v=>r41Num(v)}]}
};
let r41Session=null,r41Timer=null,r41LastRender=0;function r41Num(v){const n=Number(v);return Number.isFinite(n)?n:NaN}
function r41Profile(id=String(selectedExperiment?.id||'')){return R41_PROFILES[id]||null}
function report41RequestedRate(id=String(selectedExperiment?.id||'')){return r41Profile(id)?.targetHz||null}
function report41IsSelected(){return currentExp==='navigation'&&!!r41Profile()}
function report41TimedActive(){return !!r41Session&&r41Session.state==='RECORDING'}
function r41Mean(a){const v=a.filter(Number.isFinite);return v.length?v.reduce((s,x)=>s+x,0)/v.length:NaN}
function r41Std(a){const v=a.filter(Number.isFinite);if(!v.length)return NaN;const m=r41Mean(v);return Math.sqrt(v.reduce((s,x)=>s+(x-m)*(x-m),0)/v.length)}
function r41Fmt(v,d=4){return Number.isFinite(v)?Number(v).toFixed(d):'--'}
function r41Bucket(sec){if(!r41Session.buckets.has(sec)){const p=r41Session.profile,b={sec,counts:{},metrics:{}};p.channels.forEach(c=>b.counts[c]=0);p.metrics.forEach(m=>b.metrics[m.key]=[]);r41Session.buckets.set(sec,b)}return r41Session.buckets.get(sec)}
function r41OverallMetric(metric){const out=[];for(const b of r41Session?.buckets?.values?.()||[]){const a=b.metrics[metric.key]||[];out.push(...a)}return out}
function r41ElapsedSec(now=Date.now()){if(!r41Session)return 0;const end=r41Session.finishedMs||now;return Math.max(0,(end-r41Session.startedMs)/1000)}
function r41RemainingSec(){return Math.max(0,R41_DURATION_SEC-r41ElapsedSec())}
function report41WriteCountdown(){if(!report41TimedActive())return;const rem=r41RemainingSec(),m=Math.floor(rem/60),s=rem-m*60;setText('recordElapsed',`COUNTDOWN ${String(m).padStart(2,'0')}:${s.toFixed(1).padStart(4,'0')}`)}
function r41Rate(channel){if(!r41Session)return NaN;const n=r41Session.channelCounts[channel]||0,d=Math.min(R41_DURATION_SEC,Math.max(.001,r41ElapsedSec()));return n/d}
function r41RateOk(){if(!r41Session)return false;const r=r41Rate(r41Session.profile.primary),t=r41Session.profile.targetHz;return Number.isFinite(r)&&r>=t*.90&&r<=t*1.10}
function r41OverallSummary(){if(!r41Session)return[];return r41Session.profile.metrics.map(m=>{const a=r41OverallMetric(m);return{...m,mean:r41Mean(a),std:r41Std(a),n:a.length,min:a.length?Math.min(...a):NaN,max:a.length?Math.max(...a):NaN}})}function captureReport41TimedDelta(delta,ups={}){
  if(!report41TimedActive()||String(selectedExperiment?.id||'')!==r41Session.id)return;
  const p=r41Session.profile,now=Date.now(),elapsed=(now-r41Session.startedMs)/1000;
  if(elapsed<0||elapsed>=R41_DURATION_SEC)return;
  const sec=Math.max(0,Math.min(59,Math.floor(elapsed))),bucket=r41Bucket(sec);
  for(const channel of p.channels){
    if(!Object.prototype.hasOwnProperty.call(delta,channel))continue;
    const stamp=Number(ups?.[channel]??now),last=r41Session.lastStamps[channel];
    if(Number.isFinite(last)&&Number.isFinite(stamp)&&stamp<=last)continue;
    r41Session.lastStamps[channel]=stamp;r41Session.channelCounts[channel]=(r41Session.channelCounts[channel]||0)+1;bucket.counts[channel]=(bucket.counts[channel]||0)+1;
    const value=delta[channel],raw={elapsed_s:elapsed,second:sec+1,channel,stamp_ms:stamp};
    if(value&&typeof value==='object'&&!Array.isArray(value))Object.assign(raw,value);else raw.value=value;
    r41Session.raw.push(raw);
    for(const metric of p.metrics){if(metric.channel!==channel)continue;const v=metric.pick(value);if(Number.isFinite(v))bucket.metrics[metric.key].push(v)}
  }
  if(now-r41LastRender>180){r41LastRender=now;renderReport41TimedPanel()}
}
function startReport41TimedEvidence(id){
  const p=r41Profile(id);if(!p)return;
  if(r41Timer)clearInterval(r41Timer);
  r41Session={id,profile:p,state:'RECORDING',startedMs:Date.now(),finishedMs:0,buckets:new Map(),raw:[],lastStamps:{},channelCounts:{},autoStopRequested:false};
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
function r41SecondCells(b){return r41Session.profile.metrics.map(m=>`<td>${r41Fmt(r41Mean(b.metrics[m.key]||[]),m.key.includes('lat')||m.key.includes('lon')?7:4)}</td>`).join('')}
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
  <div class="r41-timed-actions"><button class="soft-btn" data-r41-export="second" ${s?'':'disabled'}>CSV Detail per Detik</button><button class="soft-btn" data-r41-export="raw" ${s?'':'disabled'}>CSV Raw Source-rate</button><button class="soft-btn" data-r41-reset ${report41TimedActive()?'disabled':''}>Reset Evidence 4.1</button></div>
  <section class="r41-table-section"><div class="direct-table-title"><span>MEAN</span><b>Ringkasan Seluruh Periode</b><small>Mean/Std dihitung dari semua sampel aktual selama recording, bukan dari 1 nilai per detik.</small></div><div class="table-scroll"><table class="data-table"><thead><tr><th>Parameter</th><th>Mean</th><th>Std</th><th>Min</th><th>Max</th><th>N</th><th>Unit</th></tr></thead><tbody>${summaryRows}</tbody></table></div></section>
  <section class="r41-table-section"><div class="direct-table-title"><span>DETAIL 1 s</span><b>Mean Sampel per Detik</b><small>GNSS ≈5 sampel/detik • IMU ≈50 sampel/detik • encoder/ESC ≈50 sampel/detik; N aktual selalu ditampilkan.</small></div><div class="table-scroll r41-detail-scroll"><table class="data-table"><thead><tr><th>Detik</th><th>N source</th>${r41MetricHeaders()}</tr></thead><tbody>${detailRows}</tbody></table></div></section></article>`;
  box.onclick=e=>{const ex=e.target.closest('[data-r41-export]');if(ex)return exportReport41Csv(ex.dataset.r41Export);if(e.target.closest('[data-r41-reset]'))resetReport41TimedEvidence()};
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
window.report41RequestedRate=report41RequestedRate;
window.report41TimedActive=report41TimedActive;
window.report41WriteCountdown=report41WriteCountdown;
window.captureReport41TimedDelta=captureReport41TimedDelta;
window.startReport41TimedEvidence=startReport41TimedEvidence;
window.finishReport41TimedEvidence=finishReport41TimedEvidence;
window.renderReport41TimedPanel=renderReport41TimedPanel;
