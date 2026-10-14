'use strict';
let navigationDataPackagesCache=[];
let navigationDataPackageHighlight='';

function navigationDataPackagePageActive(){
  return currentExp==='navigation'&&String(selectedExperiment?.id||'')==='R4.5.1';
}
function navigationPackageBytes(v){
  const n=Number(v)||0;
  if(n<1024)return n+' B';
  if(n<1024*1024)return (n/1024).toFixed(1)+' KB';
  if(n<1024*1024*1024)return (n/(1024*1024)).toFixed(1)+' MB';
  return (n/(1024*1024*1024)).toFixed(2)+' GB';
}
function navigationPackageTime(v){
  if(!v)return'--';
  const d=new Date(v);
  return Number.isFinite(d.getTime())?d.toLocaleString('id-ID'):String(v);
}
function navigationPackageResultClass(v){
  const s=String(v||'').toUpperCase();
  if(['SUCCEEDED','SUCCESS','ARRIVED'].includes(s))return'ok';
  if(['FAILED','FAILURE','ABORTED','REJECTED'].includes(s))return'error';
  if(['CANCELED','CANCELLED'].includes(s))return'warning';
  return'waiting';
}
function navigationPackageChapterState(p,prefix){
  const x=p?.integrity||{},keys=Object.keys(x).filter(k=>k.startsWith(prefix+'.'));
  return `${keys.filter(k=>x[k]?.ready===true).length}/${keys.length||5}`;
}
function renderNavigationDataPackageCards(){
  const box=$('navigationPackageList');if(!box)return;
  const rows=navigationDataPackagesCache||[];
  setText('navigationPackageCount',String(rows.length));
  setText('navigationPackageReady',String(rows.filter(x=>x.status==='READY'&&x.zip_exists!==false).length));
  setText('navigationPackageSuccess',String(rows.filter(x=>['SUCCEEDED','SUCCESS','ARRIVED'].includes(String(x.mission_result||'').toUpperCase())).length));
  setText('navigationPackageBytes',navigationPackageBytes(rows.reduce((a,x)=>a+(+x.bytes||0),0)));
  const chip=$('navigationPackageState');
  if(chip){chip.textContent=rows.length?'PACKAGE READY':'EMPTY';chip.className='status-chip '+(rows.length?'ok':'waiting')}
  if(!rows.length){box.innerHTML='<div class="empty-state">Belum ada package. Jalankan SEND GOAL; setelah mission terminal, package 4.2–4.4 dibuat otomatis.</div>';return}
  box.innerHTML=rows.map((p,i)=>{
    const id=String(p.package_id||''),result=String(p.mission_result||'UNKNOWN').toUpperCase();
    const ready=p.status==='READY'&&p.zip_exists!==false,downloadable=p.zip_exists!==false,hi=id===navigationDataPackageHighlight;
    const dl=p.download_url||`/api/navigation/data-package/${encodeURIComponent(id)}.zip`;
    return `<article class="navigation-package-card ${hi?'highlight':''}" data-package-id="${escapeHtml(id)}">
      <header><div><span class="navigation-package-index">PACKAGE ${String(i+1).padStart(2,'0')}</span><b>${escapeHtml(p.mission_id||id)}</b><small>${escapeHtml(navigationPackageTime(p.created_at))} • Trial ${escapeHtml(p.trial_id||'--')}</small></div>
      <div class="navigation-package-badges"><span class="status-chip ${ready?'ok':'warning'}">${ready?'READY':'INCOMPLETE'}</span><span class="status-chip ${navigationPackageResultClass(result)}">${escapeHtml(result)}</span></div></header>
      <div class="navigation-package-metrics"><div><span>Candidate</span><b>${escapeHtml(p.candidate||'--')}</b></div><div><span>Cycle</span><b>${escapeHtml(p.cycle??'--')}</b></div>
      <div><span>4.2</span><b>${escapeHtml(navigationPackageChapterState(p,'4.2'))}</b></div><div><span>4.3</span><b>${escapeHtml(navigationPackageChapterState(p,'4.3'))}</b></div>
      <div><span>4.4</span><b>${escapeHtml(navigationPackageChapterState(p,'4.4'))}</b></div><div><span>ZIP</span><b>${escapeHtml(navigationPackageBytes(p.bytes))}</b></div></div>
      <div class="navigation-package-path"><span>REPORT ZIP</span><code>4.2 / 4.3 / 4.4 → Excel tabel + PNG grafik aktual. Overlay 3-profile hanya untuk 4.2/4.3 setelah Baseline + A + B lengkap.</code></div>
      <div class="button-row navigation-package-actions"><a class="primary-btn ${downloadable?'':'disabled'}" ${downloadable?`href="${escapeHtml(dl)}"`:'aria-disabled="true"'}>${ready?'DOWNLOAD ZIP':'DOWNLOAD ZIP • AUDIT'}</a>
      <button class="danger-btn" data-delete-package="${escapeHtml(id)}">DELETE</button></div></article>`;
  }).join('');
  qa('[data-delete-package]',box).forEach(b=>b.onclick=()=>deleteNavigationDataPackage(b.dataset.deletePackage));
}
async function loadNavigationDataPackages({highlight=''}={}){
  if(highlight)navigationDataPackageHighlight=highlight;
  try{
    const r=await readRequest('/api/navigation/data-packages',{timeoutMs:10000}),j=await r.json();
    if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    navigationDataPackagesCache=Array.isArray(j.packages)?j.packages:[];
    renderNavigationDataPackageCards();
    if(navigationDataPackageHighlight){
      requestAnimationFrame(()=>{const e=[...qa('[data-package-id]',$('navigationPackageList'))].find(x=>x.dataset.packageId===navigationDataPackageHighlight);e?.scrollIntoView({behavior:'smooth',block:'center'})});
    }
    return navigationDataPackagesCache;
  }catch(e){
    if($('navigationPackageState')){$('navigationPackageState').textContent='ERROR';$('navigationPackageState').className='status-chip error'}
    const box=$('navigationPackageList');if(box)box.innerHTML=`<div class="empty-state">Package API belum siap: ${escapeHtml(e.message)}</div>`;
    return[];
  }
}
function renderNavigationDataPackagePanel(){
  const active=navigationDataPackagePageActive(),panel=$('navigationDataPackagePanel');
  document.body.classList.toggle('navigation-package-mode',active);
  if(panel)panel.hidden=!active;
  if(active)loadNavigationDataPackages();
}
function deleteNavigationDataPackage(id){
  const p=navigationDataPackagesCache.find(x=>x.package_id===id);if(!p)return;
  const label=p.mission_id||id;
  confirmModal('Hapus data mission',`Hapus ${label}?\n\nYang dihapus:\n• ZIP + package terstruktur\n• folder raw mission 4.2–4.4\n• metadata trial mission ini\n\nYAML/config tuning dan package mission lain TIDAK disentuh. Tindakan ini tidak dapat dibatalkan.`,async()=>{
    try{
      const r=await writeRequest('/api/navigation/data-package/delete',{package_id:id,delete_source:true}),j=await r.json();
      if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
      navigationDataPackageHighlight='';await loadNavigationDataPackages();
      if(typeof loadTrials==='function')await loadTrials();
      toast(j.message||'Package dihapus.');
    }catch(e){toast('Delete package gagal: '+e.message,true)}
  });
}
window.renderNavigationDataPackagePanel=renderNavigationDataPackagePanel;
window.loadNavigationDataPackages=loadNavigationDataPackages;
window.navigationDataPackageOnMissionSaved=async function(pkg,resultState=''){
  if(!pkg?.package_id)return;
  navigationDataPackageHighlight=pkg.package_id;
  await loadNavigationDataPackages({highlight:pkg.package_id});
  const succeeded=['SUCCEEDED','SUCCESS','ARRIVED'].includes(String(resultState||pkg.mission_result||'').toUpperCase());
  toast(`BAB 4 package ${pkg.status||'READY'} • ${pkg.package_id}${succeeded?' • download siap':''}`,pkg.status!=='READY');
  if(succeeded&&currentExp==='navigation'&&typeof selectExperimentById==='function')setTimeout(()=>selectExperimentById('R4.5.1'),120);
};
if($('navigationPackageRefresh'))$('navigationPackageRefresh').onclick=()=>loadNavigationDataPackages();
