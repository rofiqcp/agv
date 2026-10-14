'use strict';
function renderExperimentList(){const all=effectiveExperimentList();setText('labLeafCount',all.length);const box=$('experimentList');if(!box)return;const term=($('experimentSearch')?.value||'').trim().toLowerCase();const list=all.filter(x=>!term||`${displayExperimentId(x)} ${displayExperimentTitle(x)} ${displayExperimentGroup(x)} ${phaseLabel(experimentPhase(x))} ${x.id}`.toLowerCase().includes(term));const visited=visitedSet();let lastGroup='',html='';for(const x of list){const group=`${phaseLabel(experimentPhase(x))} · ${displayExperimentGroup(x)}`,displayId=displayExperimentId(x),title=displayExperimentTitle(x);if(group!==lastGroup){html+=`<div class="chapter-group"><span>${escapeHtml(group)}</span></div>`;lastGroup=group}html+=`<button class="experiment-item ${selectedExperiment?.id===x.id?'active':''}" data-id="${escapeHtml(x.id)}"><i>${visited.has(x.id)?'✓':'○'}</i><div><span>${escapeHtml(displayId)}</span><b>${escapeHtml(title)}</b><small>${x.id==='R4.5.1'?'ZIP package • download / delete':`${(x.tableNames||[]).length} tabel • ${(x.graphCaptions||[]).length} grafik • ${tuningFieldsFor(x).filter(p=>p.yamlFileKey).length} parameter`}</small></div></button>`}box.innerHTML=html||'<div class="empty-state">Tidak ada tahap tuning yang cocok.</div>';qa('.experiment-item',box).forEach(e=>e.onclick=()=>selectExperimentById(e.dataset.id));window.navigationTuningPolicy?.decorateList?.(box,list);setText('chapterDomainTitle',currentExp==='navigation'?'Navigasi':currentExp==='perception'?'Persepsi':'ESC / FOC');renderChapterProgress();if(activeDomain!=='overview')renderContextNav()}
function selectExperimentById(id){if(window.navigationTuningPolicy?.beforeSelect?.(selectedExperiment,id)===false)return;if(recordStartedMs&&selectedExperiment&&id!==selectedExperiment.id){toast(`Recording ${selectedExperiment.id} masih aktif. STOP dulu sebelum pindah tahap.`,true);return}const x=findExperimentById(id);if(!x)return;selectedExperiment=x;window.analysisSession?.selectTask(currentExp,id);selectedTableIndex=0;selectedGraphIndex=0;metricHistory.clear();scatterHistory.clear();scatterOrigins.clear();tableRunRows.clear();reportPathSnapshots.clear();reportEvidenceFrozen=false;reportEvidenceFrozenLeaf='';reportSourceTableRenderMs=0;ekfGrowthState.clear();reportDerivedCache.clear();reportCostmapMetricCache={key:'',minClearance:NaN,valid:NaN};chartEpochMs=Date.now();lastLabSecond=-1;lastLabTelemetryTick=-1;experimentCursorTime=null;if(!x.reportMode)markVisited(id);if(activeDomain==='navigation')activeWorkspaceTab.navigation=x.reportMode===true?'bab4':'tune';else if(activeDomain==='perception')activeWorkspaceTab.perception=/^F4\./.test(String(x.id||''))?'bab4':'tune';else if(activeDomain==='esc'&&currentExp==='steering'&&activeWorkspaceTab.esc!=='tests')activeWorkspaceTab.esc='bab4';window.navigationTuningPolicy?.onSelect?.(x);renderExperimentList();renderSelectedExperiment();renderWorkspaceTabs();if(currentExp==='navigation')setTimeout(async()=>{if(selectedExperiment?.id!==id)return;if(isNavigationBab4Tuning(x))await ensureNavigationSessionBaseline(x,{rerender:true});if(!tuningDraftRows().length)verifySelectedTuningRuntime({silent:true})},180)}
function syncReportDocumentLayout(){
  const table=document.querySelector('.table-workspace'),graph=document.querySelector('.graph-workspace'),two=document.querySelector('.lab-two-col'),per=$('perceptionEngineeringPanel'),recap=$('trialRecapPanel');if(!table||!graph||!two||!recap)return;const main=table.parentNode,report=currentExp==='navigation'&&selectedExperiment?.reportMode===true;
  let banner=$('reportDocumentFlow');if(!banner){banner=document.createElement('article');banner.id='reportDocumentFlow';banner.className='panel report-document-flow';banner.innerHTML='<div><span>FORMAT BAB IV • DATA AKTUAL</span><b>1. PREVIEW → START</b><small>Preview live tidak masuk evidence • START reset sesi ke t=0</small></div><i>↓</i><div><b>2. Tabel Sumber + Grafik Selaras</b><small>Satu buffer recording • setiap grafik punya tabel sumber aktual</small></div><i>↓</i><div><b>3. STOP → FREEZE → SAVE</b><small>Tabel + grafik berhenti bersama • CSV / Excel / PNG tidak berubah lagi</small></div>';}
  document.body.classList.toggle('navigation-report-mode',report);banner.hidden=!report;
  if(report){main.insertBefore(table,graph);main.insertBefore(banner,table);main.insertBefore(graph,two);if(per)main.insertBefore(two,per)}
  else{if(banner.parentNode===main)main.insertBefore(banner,recap);main.insertBefore(graph,two);if(per){main.insertBefore(two,per);main.insertBefore(per,table)}main.insertBefore(table,recap)}
}
function renderSelectedExperiment(){const x=selectedExperiment;if(!x)return;const domain=currentExp==='navigation'?'Navigasi':currentExp==='perception'?'Persepsi':'ESC / FOC',id=displayExperimentId(x),title=displayExperimentTitle(x),report=x.reportMode===true;setText('experimentGroup',displayExperimentGroup(x));setText('experimentTitle',`${id} ${title}`);const n0=currentExp==='navigation'&&x.id==='N0.1',ekfReport=currentExp==='navigation'&&/^R4\.[23]\.[1-5]$/.test(String(x.id||''));setText('experimentSubtitle',report?(ekfReport?`${domain} • LAPORAN BAB IV ${id} • YAML source-of-truth → safe runtime apply/read-back → data aktual berjalan • setiap kondisi diulang 3 cycle dan dibandingkan overall.`:`${domain} • LAPORAN BAB IV • rekap data dari ${x.sourceIds?.join(', ')||'source tuning'} • START/STOP, Excel, grafik, dan parameter YAML memakai mesin akuisisi Navigasi yang sama.`):n0?'Navigasi • N0 Timing Readiness • target konfigurasi dan rate aktual dipisahkan. ESC command-loop target bukan rate /esc/odom feedback.':`${domain} • Tuning ${id} • parameter, tabel, grafik, dan live evidence mengikuti source pengujian ini.`);if(activePage==='experiments'){setText('pageEyebrow',x.id==='R4.5.1'?'NAVIGASI • DATA PACKAGE':report?'NAVIGASI • LAPORAN BAB IV':`${domain.toUpperCase()} • TUNING`);setText('pageTitle',`${id} ${title}`)}if(x.id==='R4.5.1')setText('experimentSubtitle','Navigasi • arsip hasil SEND GOAL 4.2–4.4 • satu mission = satu ZIP • CSV/XLSX/PNG per subbab • 4.1 tetap terpisah.');syncReportDocumentLayout();const imuLinearityExtra=report&&x.id==='R4.1.2';setText('experimentTableBadge',`${(x.tableNames||[]).length+(imuLinearityExtra?1:0)} TABLE`);setText('experimentGraphBadge',`${(x.graphCaptions||[]).length+(imuLinearityExtra?2:0)} GRAPH`);setText('experimentRawMetadata',JSON.stringify(x,null,2));renderTuningFields();renderLabTabs();renderExperimentTable();renderExperimentGraphs();window.renderImuHeadingLinearity?.();window.renderReport41TimedPanel?.();renderContextVisual();renderSourceAudit();renderN0TimingGuide();renderPerceptionEngineering();renderTrialGuideDetailed();renderTrialRecap();renderCommissioningRoadmap();renderQualificationPanel();window.renderEkfReportRuntimePanel?.();window.navigationTuningPolicy?.render?.();renderFirdaKpQuickPanel();renderFirdaKiQuickPanel();renderFirdaPiQuickPanel();renderFirdaZnQuickPanel();renderFirdaZnBaselinePanel();renderFirdaPosKpQuickPanel();renderFirdaPosAdvancedPanels();renderFirdaResponsePanel();renderFirdaLoadPanel();renderFirdaFinalRecapPanel();renderFirdaPerformanceRecapPanel();renderFirdaIntegrationPanel();window.renderNavigationDataPackagePanel?.()}
let tuningMode='unified',tuningLevel='all';
const HELP_REGISTRY=new Map([
  ['action.tuning.validate',{label:'Validate Draft',summary:'Memeriksa draft tanpa menulis YAML atau mengubah runtime.',apply_mode:'read-only validation',risk:'Tidak menggerakkan actuator dan aman digunakan sebelum Apply.'}],
  ['action.tuning.apply',{label:'Apply + Save',summary:'Menulis seluruh draft terpilih sebagai satu transaksi, lalu menerapkan runtime dan melakukan read-back.',apply_mode:'atomic batch + runtime verify',risk:'Untuk parameter motion/safety, stationary gate dan runtime capability tetap berlaku.'}],
  ['action.tuning.revert',{label:'Revert Draft',summary:'Membuang draft browser dan kembali menampilkan nilai YAML aktual.',apply_mode:'browser only',risk:'Tidak menulis YAML.'}],
  ['action.tuning.baseline',{label:'Restore Baseline',summary:'Untuk BAB 4.2–4.4, baseline adalah snapshot YAML aktif sebelum tuning sesi dimulai. Restore hanya menyiapkannya sebagai Draft; runtime berubah setelah Apply + ACTIVE MATCH.',apply_mode:'session baseline → draft → apply + verify',risk:'Tidak memakai baseline historis lama sebagai reset BAB 4.2–4.4.'}],
  ['panel.perception.safety',{label:'Perception Safety Gate',summary:'Fail-closed gate untuk candidate obstacle dan trajectory safety. Bypass hanya untuk commissioning terkontrol.',apply_mode:'safety configuration',risk:'HIGH — bypass mengurangi proteksi persepsi; E-stop dan watchdog lain tetap wajib.'}],
  ['panel.tuning.editor',{label:'Parameter Tuning',summary:'BAB 4.2–4.4 menampilkan nilai aktif sistem sebagai acuan utama. Nilai historis lama disembunyikan; Draft hanya muncul saat kamu mengedit.',apply_mode:'Edit → Apply + Verify → ACTIVE MATCH',risk:'RESTORE BASELINE kembali ke nilai aktif sebelum sesi tuning, bukan file baseline historis.'}],
  ['panel.run.control',{label:'Run Control',summary:'Menjalankan akuisisi/evidence untuk task tuning aktif.',apply_mode:'runtime recorder',risk:'Pastikan preflight PASS sebelum motion test.'}],
  ['panel.config',{label:'Configuration Inspector',summary:'Browser YAML penuh. Hanya parameter yang terdaftar pada tuning catalog yang editable.',apply_mode:'staged batch transaction',risk:'Parameter di luar catalog tetap LOCKED.'}]
]);
function setTuningMode(){tuningMode='unified';const page=$('page-experiments');if(page)page.dataset.tuningMode='unified';setText('tuningModeGuideTitle','SATU HALAMAN');setText('tuningModeGuideText','Prosedur, preflight, run, parameter, grafik, tabel, evidence, dan qualification ditampilkan bersama.')}
function setTuningLevel(){tuningLevel='all';const page=$('page-experiments');if(page)page.dataset.tuningLevel='all';if(selectedExperiment)renderTuningFields()}
function helpValue(v){if(v===undefined||v===null||v==='')return'—';return Array.isArray(v)||typeof v==='object'?JSON.stringify(v):String(v)}
function closeHelp(){const d=$('helpDrawer'),b=$('helpBackdrop');if(d){d.hidden=true;d.setAttribute('aria-hidden','true')}if(b)b.hidden=true;restoreDialogFocus(d)}
function openHelp(key){const m=HELP_REGISTRY.get(key)||{};setText('helpTitle',m.label||'Context Help');setText('helpSummary',m.summary||'Metadata bantuan belum tersedia untuk item ini.');setText('helpUnit',m.unit||'—');setText('helpRange',m.range||'—');setText('helpBaseline',helpValue(m.baseline));setText('helpYaml',helpValue(m.yaml));setText('helpDraft',helpValue(m.draft));setText('helpRuntime',helpValue(m.runtime));setText('helpEffectUp',m.effect_up||'—');setText('helpEffectDown',m.effect_down||'—');setText('helpApplyMode',m.apply_mode||'—');setText('helpRisk',m.risk||'—');setText('helpRelated',Array.isArray(m.related)?m.related.join(', '):(m.related||'—'));setText('helpSource',m.source||key);const d=$('helpDrawer'),b=$('helpBackdrop');if(d){d.hidden=false;d.setAttribute('aria-hidden','false')}if(b)b.hidden=false;openDialogFocus(d,$('helpCloseBtn'))}
function registerParamHelp(key,p,values={}){const m=schemaMetaForParam(p)||{},guide=window.navigationTuningPolicy?.isYellow?.(selectedExperiment)?window.navigationTuningPolicy?.guidance?.(p):null,spec=(p.kind==='int'||p.kind==='float')?paramNumericSpec(p,values.yaml,values.baseline):null,hard=spec?.hasRange?`${fmt(spec.min,6)} … ${fmt(spec.max,6)}`:'—',recommended=(Number.isFinite(+m.recommended_min)||Number.isFinite(+m.recommended_max))?`${m.recommended_min??'—'} … ${m.recommended_max??'—'}`:'';HELP_REGISTRY.set(key,{label:m.label||p.label||p.key||p.yamlPath,summary:guide?.summary||m.summary||p.help||p.description||'Metadata bantuan belum tersedia.',unit:m.unit||p.unit||'—',range:recommended||hard,baseline:values.baseline,yaml:values.yaml,draft:values.draft,runtime:values.runtime,effect_up:guide?.up||m.effect_up||'—',effect_down:guide?.down||m.effect_down||'—',apply_mode:m.apply_mode||p.applyMode||'browser/session only',risk:m.risk||'—',related:[...(m.related||p.related||[]),...(m.dependencies||[]).map(d=>d.identity).filter(Boolean)],source:p.yamlFileKey?`${p.yamlFileKey}:${p.yamlPath}`:(p.key||'run input')})}
function decoratePanelHelp(){const targets={'perceptionSafetySection':'panel.perception.safety','tuningFields':'panel.tuning.editor','runConsole':'panel.run.control'};for(const [id,key] of Object.entries(targets)){const root=$(id),head=root?.closest('.panel')?.querySelector('.panel-head')||root?.querySelector?.('.panel-head');if(head&&!head.querySelector('.help-trigger'))head.insertAdjacentHTML('beforeend',`<button class="help-trigger" type="button" data-help-key="${key}" aria-label="Bantuan" title="Bantuan">?</button>`)}}
document.addEventListener('click',e=>{const h=e.target.closest?.('.help-trigger[data-help-key]');if(h){e.preventDefault();e.stopPropagation();openHelp(h.dataset.helpKey)}});if($('helpCloseBtn'))$('helpCloseBtn').onclick=closeHelp;if($('helpBackdrop'))$('helpBackdrop').onclick=closeHelp;document.addEventListener('keydown',e=>{const d=activeManagedDialog();if(!d)return;if(e.key==='Escape'){e.preventDefault();if(d.classList.contains('modal'))closeConfirmModal();else if(d.id==='configDiffDrawer')closeConfigDiff();else closeHelp();return}if(e.key!=='Tab')return;const items=dialogFocusables(d);if(!items.length){e.preventDefault();d.focus();return}const first=items[0],last=items.at(-1),active=document.activeElement;if(e.shiftKey&&(active===first||!d.contains(active))){e.preventDefault();last.focus()}else if(!e.shiftKey&&(active===last||!d.contains(active))){e.preventDefault();first.focus()}});qa('button[data-tuning-mode]').forEach(b=>b.onclick=()=>setTuningMode(b.dataset.tuningMode));qa('button[data-tuning-level]').forEach(b=>b.onclick=()=>setTuningLevel(b.dataset.tuningLevel));setTuningMode('unified');setTuningLevel('all');
const PARAM_OPTION_HINTS={feedback:['OPEN_LOOP','CLOSED_LOOP'],motion_model:['Ackermann','DiffDrive','Omni'],command_mode:['ERPM','CURRENT','DUTY','BRAKE'],gain_variant:['baseline','soft','medium','aggressive'],load_level:['no-load','light','medium','heavy'],final_scenario:['combined-safe','straight','turning','obstacle'],stateful:['true','false']};
function paramOptions(p,val,base){const m=schemaMetaForParam(p);let out=[];if(m){out=Array.isArray(m.options)?m.options.map(String):[]}else{let src=p?.options||p?.enum||p?.choices||[];if(src&&typeof src==='object'&&!Array.isArray(src))src=Object.keys(src);out=Array.isArray(src)?src.map(String):[];const key=String(p?.key||p?.name||'').toLowerCase(),leaf=key.split('.').pop();for(const [hint,vals] of Object.entries(PARAM_OPTION_HINTS))if(leaf===hint||key.includes(hint))out.push(...vals)}for(const v of [val,base,p?.placeholder])if(v!==undefined&&v!==null&&String(v)!=='')out.push(String(v));return [...new Set(out)]}
function paramNumericSpec(p,val,base){const m=schemaMetaForParam(p)||{},stepRaw=m.step??p?.step??p?.increment,step=Number.isFinite(+stepRaw)&&+stepRaw>0?+stepRaw:(p?.kind==='int'?1:.001),minRaw=m.hard_min??p?.min??p?.minimum,maxRaw=m.hard_max??p?.max??p?.maximum,min=Number(minRaw),max=Number(maxRaw),hasRange=Number.isFinite(min)&&Number.isFinite(max)&&max>min;return{min:hasRange?min:undefined,max:hasRange?max:undefined,step,hasRange}}
function paramNumericHtml(id,p,val,base,ro,rangeId=''){const m=paramNumericSpec(p,val,base),rid=rangeId||id+'Range',v=Number.isFinite(Number(val))?Number(val):(Number.isFinite(Number(base))?Number(base):0),num=`<input id="${id}" type="number" step="${m.step}" value="${escapeHtml(String(v))}" ${ro?'readonly':''}>`;if(!m.hasRange)return `<div class="param-number-control no-slider">${num}</div>`;return `<div class="param-number-control">${num}<input id="${rid}" class="param-slider" type="range" min="${m.min}" max="${m.max}" step="${m.step}" value="${Math.min(m.max,Math.max(m.min,v))}" ${ro?'disabled':''}></div>`}
function bindParamNumber(input,range,onCommit){if(!input)return;input.addEventListener('wheel',e=>{e.preventDefault();input.blur()},{passive:false});input.addEventListener('input',()=>{const v=Number(input.value);if(!range||!Number.isFinite(v))return;if(v<+range.min)range.min=String(v);if(v>+range.max)range.max=String(v);range.value=String(v)});input.addEventListener('change',()=>onCommit?.());input.addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();input.blur()}});if(range){range.addEventListener('wheel',e=>e.preventDefault(),{passive:false});range.addEventListener('input',()=>{input.value=range.value});range.addEventListener('change',()=>onCommit?.())}}
function paramSelectHtml(id,p,val,base,ro){const opts=paramOptions(p,val,base);return `<select id="${id}" ${ro?'disabled':''}>${opts.map(v=>`<option value="${escapeHtml(v)}" ${String(v)===String(val)?'selected':''}>${escapeHtml(v)}</option>`).join('')}</select>`}
function findConfigFieldMeta(r){let p=null;for(const list of Object.values(experiments||{}))if(Array.isArray(list))for(const x of list)for(const q of tuningFieldsFor(x))if(q.yamlFileKey===r.fileKey&&q.yamlPath===r.path){p=q;break}const m=schemaMeta(r.fileKey,r.path)||{};return{...(p||{key:r.name,yamlPath:r.path,kind:r.type==='number'?'float':r.type}),level:m.level,step:m.step,min:m.hard_min,max:m.hard_max,options:m.options,unit:m.unit,risk:m.risk,summary:m.summary,effect_up:m.effect_up,effect_down:m.effect_down,applyMode:m.apply_mode,yamlFileKey:r.fileKey,yamlPath:r.path}}
const navigationSessionBaselineEnsured=new Set();
const NAV_BAB4_GLOBAL_PROFILE_BASELINE_KEY='adv-nav-bab4-global-profile-baseline-v1';
const NAV_BAB4_GLOBAL_PROFILE_ACTIVE_KEY='adv-nav-bab4-global-profile-active-v1';
const NAV_BAB4_GLOBAL_PROFILE_FIELDS=[
  {fileKey:'ekf',path:'ekf_filter_node_odom.ros__parameters.frequency',a:25,b:40},
  {fileKey:'ekf',path:'ekf_filter_node_odom.ros__parameters.sensor_timeout',a:0.30,b:0.20},
  {fileKey:'ekf',path:'ekf_filter_node_odom.ros__parameters.predict_to_current_time',a:false,b:true},
  {fileKey:'ekf',path:'ekf_filter_node_odom.ros__parameters.process_noise_covariance.0',a:0.04,b:0.06},
  {fileKey:'ekf',path:'ekf_filter_node_odom.ros__parameters.process_noise_covariance.16',a:0.04,b:0.06},
  {fileKey:'ekf',path:'ekf_filter_node_odom.ros__parameters.process_noise_covariance.80',a:0.05,b:0.075},
  {fileKey:'ekf',path:'ekf_filter_node_odom.ros__parameters.process_noise_covariance.96',a:0.02,b:0.035},
  {fileKey:'ekf',path:'ekf_filter_node_odom.ros__parameters.process_noise_covariance.176',a:0.015,b:0.025},
  {fileKey:'esc',path:'esc_ackermann.ros__parameters.odom_v_variance_base',a:0.035,b:0.025},
  {fileKey:'esc',path:'esc_ackermann.ros__parameters.odom_yaw_rate_variance_base',a:0.06,b:0.04},
  {fileKey:'imu',path:'data_imu_node.ros__parameters.orientation_covariance.2',a:0.009,b:0.006},
  {fileKey:'imu',path:'data_imu_node.ros__parameters.angular_velocity_covariance.2',a:0.0005,b:0.0003},
  {fileKey:'localization',path:'localization_core.ros__parameters.gnss_velocity_fusion_min_variance',a:0.003,b:0.002},
  {fileKey:'localization',path:'localization_core.ros__parameters.gnss_velocity_fusion_max_variance',a:0.30,b:0.20},
  {fileKey:'ekf',path:'ekf_filter_node_map.ros__parameters.frequency',a:25,b:40},
  {fileKey:'ekf',path:'ekf_filter_node_map.ros__parameters.sensor_timeout',a:0.60,b:0.40},
  {fileKey:'ekf',path:'ekf_filter_node_map.ros__parameters.predict_to_current_time',a:false,b:true},
  {fileKey:'ekf',path:'ekf_filter_node_map.ros__parameters.process_noise_covariance.0',a:0.10,b:0.15},
  {fileKey:'ekf',path:'ekf_filter_node_map.ros__parameters.process_noise_covariance.16',a:0.10,b:0.15},
  {fileKey:'localization',path:'localization_core.ros__parameters.gnss_cog_fusion_min_variance_rad2',a:0.0015,b:0.0010},
  {fileKey:'localization',path:'localization_core.ros__parameters.gnss_cog_fusion_max_variance_rad2',a:0.32,b:0.22},
  {fileKey:'nav2',path:'planner_server.ros__parameters.GridBased.downsample_costmap',a:false,b:true},
  {fileKey:'nav2',path:'planner_server.ros__parameters.GridBased.downsampling_factor',a:1,b:2},
  {fileKey:'nav2',path:'planner_server.ros__parameters.GridBased.tolerance',a:0.04,b:0.07},
  {fileKey:'nav2',path:'planner_server.ros__parameters.GridBased.cost_penalty',a:3.2,b:2.8},
  {fileKey:'nav2',path:'controller_server.ros__parameters.goal_checker.xy_goal_tolerance',a:1.0,b:1.0},
  {fileKey:'nav2',path:'controller_server.ros__parameters.goal_checker.yaw_goal_tolerance',a:3.14159265359,b:3.14159265359}
];
function isNavigationBab4Tuning(x=selectedExperiment){return currentExp==='navigation'&&/^R4\.[234]\.[1-5]$/.test(String(x?.id||''))}
function navigationGlobalProfileOwner(x=selectedExperiment){return currentExp==='navigation'&&String(x?.id||'')==='R4.2.1'}
function navigationGlobalProfileBaselineSnapshot(){
  const server=commissioningState?.navigation_tuning?.['R4.2.1']?.entry_snapshot?.values||{};
  const serverComplete=NAV_BAB4_GLOBAL_PROFILE_FIELDS.every(f=>Object.prototype.hasOwnProperty.call(server,`${f.fileKey}:${f.path}`));
  if(serverComplete){try{localStorage.setItem(NAV_BAB4_GLOBAL_PROFILE_BASELINE_KEY,JSON.stringify(server))}catch(_){};return {...server}}
  let saved={};try{saved=JSON.parse(localStorage.getItem(NAV_BAB4_GLOBAL_PROFILE_BASELINE_KEY)||'{}')||{}}catch(_){saved={}}
  const complete=NAV_BAB4_GLOBAL_PROFILE_FIELDS.every(f=>Object.prototype.hasOwnProperty.call(saved,`${f.fileKey}:${f.path}`));
  if(!complete){saved={};for(const f of NAV_BAB4_GLOBAL_PROFILE_FIELDS){const v=configValue(f.fileKey,f.path);if(v!==undefined)saved[`${f.fileKey}:${f.path}`]=v}try{localStorage.setItem(NAV_BAB4_GLOBAL_PROFILE_BASELINE_KEY,JSON.stringify(saved))}catch(_){}}
  return saved
}
function navigationGlobalProfileBaselineValueByIdentity(identity){return navigationGlobalProfileBaselineSnapshot()?.[identity]}
function navigationGlobalProfileRows(profile='baseline'){
  const base=navigationGlobalProfileBaselineSnapshot(),key=String(profile||'baseline').toLowerCase();
  return NAV_BAB4_GLOBAL_PROFILE_FIELDS.map(f=>({fileKey:f.fileKey,path:f.path,value:key==='candidate-a'?f.a:key==='candidate-b'?f.b:base[`${f.fileKey}:${f.path}`]})).filter(r=>r.value!==undefined)
}
function navigationGlobalProfileEffectiveValue(f){const id=`${f.fileKey}:${f.path}`;return configPending.get(id)?.value??configValue(f.fileKey,f.path)}
function navigationGlobalProfileSelected(){
  const base=navigationGlobalProfileBaselineSnapshot();
  for(const key of ['baseline','candidate-a','candidate-b']){
    const ok=NAV_BAB4_GLOBAL_PROFILE_FIELDS.every(f=>{const target=key==='candidate-a'?f.a:key==='candidate-b'?f.b:base[`${f.fileKey}:${f.path}`];return JSON.stringify(navigationGlobalProfileEffectiveValue(f))===JSON.stringify(target)});
    if(ok)return key
  }
  return localStorage.getItem(NAV_BAB4_GLOBAL_PROFILE_ACTIVE_KEY)||'baseline'
}
function navigationGlobalProfileHtml(){
  const active=navigationGlobalProfileSelected();
  const base=navigationGlobalProfileBaselineSnapshot();
  return `<section class="tuning-subgroup bab4-candidate-box nav-global-profile-box"><div class="bab4-candidate-head"><div><b>PROFILE GLOBAL • 3 PERCOBAAN</b><small>4.2 + 4.3 tuning utama • 4.4 mengikuti sebagai validasi sistem</small></div><span>${NAV_BAB4_GLOBAL_PROFILE_FIELDS.length} parameter bundle</span></div><div class="button-row"><button type="button" class="soft-btn ${active==='baseline'?'active':''}" data-nav-global-profile="baseline">BASELINE</button><button type="button" class="soft-btn ${active==='candidate-a'?'active':''}" data-nav-global-profile="candidate-a">CANDIDATE A • STABIL</button><button type="button" class="soft-btn ${active==='candidate-b'?'active':''}" data-nav-global-profile="candidate-b">CANDIDATE B • RESPONSIF</button></div><div class="bab4-candidate-foot">Pilih profile → seluruh bundle masuk DRAFT → APPLY TUNING → tunggu ACTIVE MATCH → SEND GOAL. Baseline tersimpan dari YAML aktif sebelum eksperimen profile dimulai.</div><div class="param-state-line"><span>BASE LOCAL EKF <b>${escapeHtml(configValueText(base['ekf:ekf_filter_node_odom.ros__parameters.frequency']))} Hz</b></span><span>A <b>25 Hz / timeout 0.30</b></span><span>B <b>40 Hz / timeout 0.20</b></span><span>Planner cost <b>BASE ${escapeHtml(configValueText(base['nav2:planner_server.ros__parameters.GridBased.cost_penalty']))} • A 3.2 • B 2.8</b></span></div></section>`
}
async function stageNavigationGlobalProfile(profile){
  if(recordStartedMs)return toast('STOP recording dulu sebelum ganti profile.',true);
  if(obj('server').read_only===true)return toast('Server read-only; profile tidak bisa di-stage.',true);
  const owner=findExperimentById('R4.2.1')||selectedExperiment;
  const baselineReady=await ensureNavigationSessionBaseline(owner,{rerender:false});
  if(!baselineReady)return toast('Profile diblok: Baseline global 4.2–4.4 belum tersimpan persisten.',true);
  const rows=navigationGlobalProfileRows(profile);let n=0;
  for(const row of rows){
    const meta=schemaMeta(row.fileKey,row.path),id=`${row.fileKey}:${row.path}`,current=configValue(row.fileKey,row.path);
    if(!meta?.metadata_complete||meta.write_authority!=='ros_yaml'||current===undefined)continue;
    if(JSON.stringify(current)===JSON.stringify(row.value))configPending.delete(id);
    else configPending.set(id,{fileKey:row.fileKey,path:row.path,name:`PROFILE ${String(profile).toUpperCase()} • ${meta.label||row.path}`,value:row.value,oldValue:current,action:'set',source:'navigation-global-profile',experimentId:'R4.2.1'});
    n++
  }
  try{localStorage.setItem(NAV_BAB4_GLOBAL_PROFILE_ACTIVE_KEY,profile)}catch(_){}
  if($('runCandidate'))$('runCandidate').value=profile;
  if($('runVariation'))$('runVariation').value=`PROFILE-${String(profile).toUpperCase()}`;
  if($('runCondition'))$('runCondition').value='GLOBAL-PROFILE-3RUN';
  invalidateConfigValidation('global navigation profile changed');renderTuningFields();renderConfigBrowser();setTuningQuickState('draft',`${String(profile).toUpperCase()} • GLOBAL PROFILE`,`${n} parameter 4.2–4.4 disiapkan. Klik APPLY TUNING lalu tunggu ACTIVE MATCH.`);
  toast(`${String(profile).toUpperCase()}: bundle 4.2–4.4 masuk DRAFT; runtime belum berubah.`)
}
function syncNavigationGlobalProfileRunIdentity(){
  if(!navigationGlobalProfileOwner())return;
  const profile=navigationGlobalProfileSelected();
  if($('runCandidate'))$('runCandidate').value=profile;
  if($('runVariation'))$('runVariation').value=`PROFILE-${String(profile).toUpperCase()}`;
  if($('runCondition'))$('runCondition').value='GLOBAL-PROFILE-3RUN'
}
const navigationGlobalProfileCandidateSelect=$('runCandidate');
if(navigationGlobalProfileCandidateSelect&&!navigationGlobalProfileCandidateSelect.dataset.navGlobalProfileBound){
  navigationGlobalProfileCandidateSelect.dataset.navGlobalProfileBound='1';
  navigationGlobalProfileCandidateSelect.addEventListener('change',e=>{if(navigationGlobalProfileOwner())stageNavigationGlobalProfile(e.target.value)})
}
function navigationSessionBaselineRow(x=selectedExperiment){return commissioningState?.navigation_tuning?.[x?.id]||null}
function navigationSessionBaselineValueByIdentity(identity,x=selectedExperiment){return navigationSessionBaselineRow(x)?.entry_snapshot?.values?.[identity]}
function navigationSessionBaselineValue(p,x=selectedExperiment){return p?.yamlFileKey&&p?.yamlPath?navigationSessionBaselineValueByIdentity(`${p.yamlFileKey}:${p.yamlPath}`,x):undefined}
function navigationSessionBaselineItems(x=selectedExperiment){if(navigationGlobalProfileOwner(x))return NAV_BAB4_GLOBAL_PROFILE_FIELDS.map(f=>({file_key:f.fileKey,path:f.path}));const seen=new Set(),out=[];for(const p of tuningFieldsFor(x)){if(!p?.yamlFileKey||!p?.yamlPath)continue;const m=schemaMetaForParam(p);if(!m?.metadata_complete||m.write_authority!=='ros_yaml')continue;const id=`${p.yamlFileKey}:${p.yamlPath}`;if(seen.has(id))continue;seen.add(id);out.push({file_key:p.yamlFileKey,path:p.yamlPath})}return out}
function navigationCandidateClamp(v,p){const m=schemaMetaForParam(p)||{},path=String(p?.yamlPath||'').toLowerCase(),lo=Number(m.hard_min),hi=Number(m.hard_max);if(Number.isFinite(lo))v=Math.max(lo,v);if(Number.isFinite(hi))v=Math.min(hi,v);if(path.endsWith('.frequency'))v=Math.max(1,Math.min(100,v));if(path.includes('sensor_timeout'))v=Math.max(.01,v);if(path.includes('variance')||path.includes('covariance'))v=Math.max(1e-8,v);if(path.includes('downsampling_factor'))v=Math.max(1,v);if(path.includes('minimum_turning_radius')||path.includes('min_turning_r'))v=Math.max(.1,v);if(path.includes('xy_goal_tolerance'))v=Math.max(.01,v);if(path.includes('yaw_goal_tolerance'))v=Math.max(.001,Math.min(Math.PI,v));if(path.includes('trans_stopped_velocity')||path.includes('rot_stopped_velocity'))v=Math.max(.001,v);return v}
function navigationCandidateRound(v,p){const path=String(p?.yamlPath||'').toLowerCase();if(p?.kind==='int'||path.includes('downsampling_factor')||path.endsWith('.frequency'))return Math.round(v);let d=6;if(path.includes('sensor_timeout')||path.includes('minimum_turning_radius')||path.includes('min_turning_r')||path.endsWith('.tolerance')||path.includes('cost_penalty')||path.includes('goal_tolerance')||path.includes('stopped_velocity'))d=3;else if(path.includes('variance')||path.includes('covariance'))d=7;return +(Number(v).toFixed(d))}
function navigationBab4CandidateValues(p,active){
  if(!isNavigationBab4Tuning()||!p?.yamlFileKey||active===undefined||active===null)return[];
  const path=String(p.yamlPath||'').toLowerCase(),guide=window.navigationTuningPolicy?.guidance?.(p)||{};
  if(p.kind==='bool'||typeof active==='boolean'){
    const v=!Boolean(active),reason=path.includes('predict_to_current_time')?(v?'Menguji prediksi state ke waktu sekarang; cek apakah EKF age turun tanpa residual/jump membesar.':'Menguji tanpa extrapolation ke waktu sekarang; cek apakah output lebih stabil tanpa menambah state age.'):path.includes('downsample_costmap')?(v?'Mengaktifkan downsample costmap dapat menurunkan beban planning; cek planning time, clearance, dan detail path.':'Memakai resolusi costmap penuh untuk mempertahankan detail obstacle; cek kenaikan planning time.'):(v?'Menguji fitur dalam keadaan aktif dan membandingkan dampaknya terhadap metrik subbab ini.':'Menguji fitur dalam keadaan nonaktif dan membandingkan dampaknya terhadap metrik subbab ini.');
    return[{value:v,label:'ALTERNATIF',reason}];
  }
  const a=Number(active);if(!Number.isFinite(a))return[];
  let low,high,lowReason=guide.down||'Nilai lebih rendah diuji untuk melihat trade-off terhadap respons, noise, dan stabilitas.',highReason=guide.up||'Nilai lebih tinggi diuji untuk melihat trade-off terhadap respons, noise, dan stabilitas.';
  if(path.endsWith('.frequency')){const d=a>=20?10:Math.max(1,Math.round(a*.25));low=a-d;high=a+d;lowReason='Loop lebih ringan dan jitter CPU berpotensi turun; bandingkan EKF age, continuity, dan residual.';highReason='Update estimator lebih rapat; gunakan hanya bila CPU/jitter tetap sehat dan residual tidak memburuk.'}
  else if(path.includes('sensor_timeout')){low=a*.75;high=a*1.25;lowReason='Lebih fail-fast terhadap data stale; cek apakah dropout/continuity justru memburuk saat jitter sensor.';highReason='Lebih toleran terhadap gap sensor; cek agar measurement lama tidak dipertahankan terlalu lama.'}
  else if(path.includes('process_noise_covariance')){low=a*.75;high=a*1.25;lowReason='Q lebih kecil membuat EKF lebih percaya model dan lebih halus; cek lag serta residual.';highReason='Q lebih besar membuat EKF lebih responsif terhadap perubahan; cek sigma/noise dan position jump.'}
  else if(path.includes('variance')||path.includes('covariance')){low=a*.75;high=a*1.25;lowReason='Variance lebih kecil memberi bobot sensor lebih besar; cek apakah noise/outlier ikut menarik estimate.';highReason='Variance lebih besar mengurangi bobot sensor; cek apakah estimate menjadi terlalu lambat atau drift.'}
  else if(path.includes('minimum_turning_radius')||path.includes('min_turning_r')){low=a*.90;high=a*1.10;lowReason='Radius lebih kecil mengizinkan belokan lebih tajam, tetapi wajib tetap di atas kemampuan fisik Ackermann.';highReason='Radius lebih besar lebih konservatif dan mengurangi perintah belok tajam, dengan konsekuensi path bisa lebih panjang.'}
  else if(path.includes('downsampling_factor')){low=a-1;high=a+1;lowReason='Downsampling lebih kecil mempertahankan detail costmap lebih banyak tetapi planning lebih berat.';highReason='Downsampling lebih besar dapat mempercepat planning tetapi detail obstacle/path berkurang.'}
  else if(path.endsWith('.tolerance')){low=a*.75;high=a*1.25;lowReason='Tolerance lebih ketat menekan endpoint error tetapi planning dapat lebih sulit/lambat.';highReason='Tolerance lebih longgar dapat meningkatkan kemudahan planning tetapi endpoint error yang diterima lebih besar.'}
  else if(path.includes('cost_penalty')){low=a*.80;high=a*1.20;lowReason='Penalty lebih rendah mengurangi hukuman area cost tinggi; path bisa lebih pendek tetapi clearance perlu diawasi.';highReason='Penalty lebih tinggi mendorong path menjauhi area cost tinggi; cek clearance, path length, dan planning time.'}
  else if(path.includes('xy_goal_tolerance')||path.includes('yaw_goal_tolerance')){low=a*.75;high=a*1.25;lowReason='Goal dibuat lebih ketat; ukur apakah error akhir turun tanpa meningkatkan timeout/oscillation.';highReason='Goal dibuat lebih longgar; completion lebih mudah tetapi akurasi akhir yang diterima lebih besar.'}
  else if(path.includes('trans_stopped_velocity')||path.includes('rot_stopped_velocity')){low=a*.75;high=a*1.25;lowReason='Syarat berhenti lebih ketat; cek final error dan tambahan time-to-goal.';highReason='Syarat berhenti lebih longgar; goal lebih cepat selesai tetapi jangan sampai dinyatakan selesai saat masih bergerak.'}
  else if(p.kind==='int'){low=a-1;high=a+1}
  else{const d=Math.max(Math.abs(a)*.20,Number(schemaMetaForParam(p)?.step)||.001);low=a-d;high=a+d}
  const vals=[];
  for(const [v,label,reason] of [[low,'RENDAH',lowReason],[high,'TINGGI',highReason]]){
    const c=navigationCandidateClamp(v,p);if(Math.abs(c-a)<=Math.max(1e-12,Math.abs(a)*1e-9))continue;const n=navigationCandidateRound(c,p);if(!Number.isFinite(n)||Object.is(n,a)||vals.some(x=>Object.is(x.value,n)))continue;vals.push({value:n,label,reason})
  }
  return vals;
}
function navigationBab4CandidateHtml(p,active,i,disabled=false){const cs=navigationBab4CandidateValues(p,active);if(!cs.length)return'';return `<div class="bab4-candidate-box"><div class="bab4-candidate-head"><b>SARAN UJI — BELUM AKTIF</b><span>Active tetap ${escapeHtml(configValueText(active))}</span></div>${cs.map(c=>`<div class="bab4-candidate-row"><button type="button" class="bab4-use-candidate" data-field-index="${i}" data-candidate="${escapeHtml(JSON.stringify(c.value))}" ${disabled?'disabled':''}>UJI ${escapeHtml(configValueText(c.value))}</button><div><b>${escapeHtml(c.label)}</b><small>${escapeHtml(c.reason)}</small></div></div>`).join('')}<small class="bab4-candidate-foot">Candidate hanya mengisi DRAFT. Tidak mengubah YAML/runtime sampai APPLY TUNING + ACTIVE MATCH.</small></div>`}
async function ensureNavigationSessionBaseline(x=selectedExperiment,{rerender=true}={}){
  if(!isNavigationBab4Tuning(x))return true;
  const existing=navigationSessionBaselineRow(x)?.entry_snapshot?.values||{},needed=navigationSessionBaselineItems(x),complete=needed.length&&needed.every(i=>Object.prototype.hasOwnProperty.call(existing,`${i.file_key}:${i.path}`));
  if(complete){navigationSessionBaselineEnsured.add(x.id);return true}
  if(navigationSessionBaselineEnsured.has(x.id)&&!navigationGlobalProfileOwner(x))return true;
  if(obj('server').read_only===true)return false;
  try{
    const r=await writeRequest('/api/navigation/tuning/entry',{id:x.id,action:'ensure',items:navigationSessionBaselineItems(x)}),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    if(j.commissioning)commissioningState=j.commissioning;navigationSessionBaselineEnsured.add(x.id);
    if(rerender&&selectedExperiment?.id===x.id){renderTuningFields();renderTuningQuickState()}
    return true;
  }catch(e){setTuningQuickState('error','BASELINE ERROR',e.message,x.id);toast('Gagal menyimpan baseline sesi: '+e.message,true);return false}
}
function renderTuningFields(){
  const x=selectedExperiment,fields=tuningFieldsFor(x),readOnly=obj('server').read_only===true;setText('labYamlMode',readOnly?'LOCKED':'EDIT → APPLY → MATCH');
  const resetBtn=$('resetExperimentYaml'),resettable=fields.filter(p=>p.yamlFileKey&&p.kind!=='yaml_readonly'&&!p.locked&&schemaMetaForParam(p)?.write_authority==='ros_yaml'&&baselineConfigValue(p.yamlFileKey,p.yamlPath)!==undefined);if(resetBtn){resetBtn.hidden=true;resetBtn.disabled=true;resetBtn.textContent=`↶ Stage ${resettable.length} Baseline Awal`}
  if(!fields.length){$('tuningFields').innerHTML='<div class="empty-state">Tahap validasi/ringkasan; tidak ada parameter editable.</div>';updateTuningDraftControls();decoratePanelHelp();return}
  const groups=new Map();fields.forEach((p,i)=>{let g=p.group||'';if(!g){if(['sample_rate','duration'].includes(p.key))g='Akuisisi';else if(['variation','condition'].includes(p.key))g='Identitas Run';else if(String(p.key||'').startsWith('gt_'))g='Ground Truth';else g='Parameter / YAML'}if(!groups.has(g))groups.set(g,[]);groups.get(g).push({p,i})});let html=navigationGlobalProfileOwner(x)?navigationGlobalProfileHtml():'';
  for(const [group,items] of groups){html+=`<section class="tuning-subgroup"><div class="tuning-subgroup-title"><b>${escapeHtml(group)}</b><span>${items.length} field</span></div>`;for(const {p,i} of items){const isYaml=!!p.yamlFileKey,meta=isYaml?schemaMetaForParam(p):null,schemaLocked=isYaml&&(!meta?.metadata_complete||meta.write_authority!=='ros_yaml'),reportEditable=isNavigationBab4Tuning(x)&&isYaml&&meta?.metadata_complete&&meta.write_authority==='ros_yaml'&&p.kind!=='yaml_readonly',ro=p.kind==='yaml_readonly'||(p.locked&&!reportEditable)||readOnly||schemaLocked,identity=isYaml?`${p.yamlFileKey}:${p.yamlPath}`:'',yamlVal=isYaml?configValue(p.yamlFileKey,p.yamlPath):undefined,pending=isYaml?configPending.get(identity):null,rawVal=isYaml?(pending?.value??yamlVal):(localRunValues[`${currentExp}:${x.id}:${p.key||p.label}`]??p.placeholder??''),val=p.kind==='list'&&Array.isArray(rawVal)?JSON.stringify(rawVal):rawVal,baseVal=isYaml?(isNavigationBab4Tuning(x)?navigationSessionBaselineValue(p,x):baselineConfigValue(p.yamlFileKey,p.yamlPath)):undefined,baseFull=baseVal===undefined?'':(Array.isArray(baseVal)||baseVal&&typeof baseVal==='object'?JSON.stringify(baseVal):String(baseVal)),baseShort=baselineText(baseVal),candidateHtml='';let control;if(p.kind==='bool')control=`<label class="switch"><input id="tuneField-${i}" data-auto-index="${i}" type="checkbox" ${rawVal?'checked':''} ${ro?'disabled':''}><i></i><b>${rawVal?'TRUE':'FALSE'}</b></label>`;else if(p.kind==='int'||p.kind==='float')control=paramNumericHtml(`tuneField-${i}`,p,val,baseVal,ro,`tuneRange-${i}`);else if(p.kind==='string')control=paramSelectHtml(`tuneField-${i}`,p,val,baseVal,ro);else control=`<input id="tuneField-${i}" data-auto-index="${i}" type="text" value="${escapeHtml(val??'')}" ${ro?'readonly':''}>`;html+=`<div class="tuning-field ${ro?'readonly':''} ${pending?'draft':''}" data-field-index="${i}"><div class="tuning-field-head"><label>${escapeHtml(p.label||p.key||'Parameter')}</label><div class="tuning-badges"><span>${isYaml?escapeHtml(p.yamlFileKey)+' • YAML':'RUN / GT'} ${ro?'• LOCKED':''}</span>${isYaml?`<em id="tuneRuntime-${i}" class="runtime-badge">${ro?'LOCKED':'READY'}</em>`:''}${isYaml&&baseVal!==undefined&&!isNavigationBab4Tuning(x)?`<em class="runtime-badge baseline" title="Baseline awal: ${escapeHtml(baseFull)}">BASE ${escapeHtml(baseShort)}</em>`:''}</div></div><div class="tuning-input-row">${control}${ro||isYaml?'':`<button class="param-save" data-field-index="${i}">SET</button>`}</div>${candidateHtml}<code>${escapeHtml(p.yamlPath||p.key||'run identity')}</code></div>`}html+='</section>'}
  $('tuningFields').innerHTML=html;qa('[data-nav-global-profile]',$('tuningFields')).forEach(b=>b.onclick=()=>stageNavigationGlobalProfile(b.dataset.navGlobalProfile));qa('.param-save',$('tuningFields')).forEach(b=>b.onclick=()=>saveTuningField(+b.dataset.fieldIndex));fields.forEach((p,i)=>{const e=$(`tuneField-${i}`);if(!e)return;if(p.kind==='int'||p.kind==='float')bindParamNumber(e,$(`tuneRange-${i}`),()=>saveTuningField(i,true));else e.onchange=()=>{if(e.type==='checkbox'){const b=q('b',e.parentElement);if(b)b.textContent=e.checked?'TRUE':'FALSE'}saveTuningField(i,true)}});decorateTuningFieldStates(fields);updateTuningDraftControls();decoratePanelHelp();window.navigationTuningPolicy?.render?.();if(navigationGlobalProfileOwner(x))syncNavigationGlobalProfileRunIdentity()
}
async function saveTuningField(i,automatic=false){
  const p=tuningFieldsFor(selectedExperiment)[i],el=$(`tuneField-${i}`);if(!p||!el||el.readOnly||el.disabled)return;
  let value=p.kind==='bool'?el.checked:el.value;
  if(p.kind==='int')value=parseInt(value,10);if(p.kind==='float')value=parseFloat(value);
  if(p.kind==='list'){try{const text=String(value).trim();value=text.startsWith('[')?JSON.parse(text):text.split(',').map(v=>Number(v.trim())).filter(Number.isFinite);if(!Array.isArray(value)||!value.length)throw new Error('empty')}catch(_){return toast('List tidak valid. Pakai [0,2,3] atau 0,2,3',true)}}
  if((p.kind==='int'||p.kind==='float')&&!Number.isFinite(value))return toast('Nilai numerik tidak valid',true);
  const btn=q(`.param-save[data-field-index="${i}"]`),badge=$(`tuneRuntime-${i}`);
  if(!p.yamlFileKey){localRunValues[`${currentExp}:${selectedExperiment.id}:${p.key||p.label}`]=value;if(btn){btn.textContent='SET';btn.classList.add('saved');setTimeout(()=>btn.classList.remove('saved'),900)}if(!automatic)toast('Nilai run/ground truth disimpan untuk sesi browser');return}
  const identity=`${p.yamlFileKey}:${p.yamlPath}`,current=configValue(p.yamlFileKey,p.yamlPath);
  if(JSON.stringify(value)===JSON.stringify(current)){configPending.delete(identity);if(btn){btn.textContent='STAGE';btn.classList.remove('staged')}if(badge){badge.textContent='READY';badge.className='runtime-badge'}if(!automatic)toast(`${p.label||p.yamlPath}: draft dibuang karena sama dengan YAML.`)}
  else{configPending.set(identity,{fileKey:p.yamlFileKey,path:p.yamlPath,name:p.label||p.key||p.yamlPath,value,oldValue:current,action:'set',source:'tuning',experimentId:selectedExperiment.id});if(btn){btn.textContent='STAGED';btn.classList.add('staged')}if(badge){badge.textContent='DRAFT';badge.className='runtime-badge pending'}if(!automatic)toast(`${p.label||p.yamlPath}: staged; YAML belum berubah.`)}
  invalidateConfigValidation('tuning draft changed');const row=q(`.tuning-field[data-field-index="${i}"]`);if(row)row.classList.toggle('draft',configPending.has(identity));decorateTuningFieldStates(tuningFieldsFor(selectedExperiment));updateTuningDraftControls();renderConfigBrowser()
}
function tuningDraftRows(){if(!selectedExperiment)return[];if(navigationGlobalProfileOwner())return [...configPending.values()].filter(x=>x.source==='navigation-global-profile'||NAV_BAB4_GLOBAL_PROFILE_FIELDS.some(f=>f.fileKey===x.fileKey&&f.path===x.path));const ids=new Set(tuningFieldsFor(selectedExperiment).filter(p=>p.yamlFileKey).map(p=>`${p.yamlFileKey}:${p.yamlPath}`));return [...configPending.values()].filter(x=>ids.has(`${x.fileKey}:${x.path}`))}
function tuningLevelFor(){return'all'}
function decorateTuningFieldStates(fields){fields.forEach((p,i)=>{const row=q(`.tuning-field[data-field-index="${i}"]`);if(!row)return;row.dataset.paramLevel=tuningLevelFor(p,i);const identity=p.yamlFileKey?`${p.yamlFileKey}:${p.yamlPath}`:`run:${selectedExperiment?.id||'none'}:${p.key||i}`,yaml=p.yamlFileKey?configValue(p.yamlFileKey,p.yamlPath):undefined,sessionMode=isNavigationBab4Tuning(),base=p.yamlFileKey?(sessionMode?navigationSessionBaselineValue(p):baselineConfigValue(p.yamlFileKey,p.yamlPath)):undefined,draft=p.yamlFileKey?configPending.get(identity)?.value:undefined,runtime=p.yamlFileKey?(configRuntimeState.get(identity)?.label||'—'):'RUN / GT';if(p.yamlFileKey){let line=q('.param-state-line',row);if(!line){line=document.createElement('div');line.className='param-state-line';row.appendChild(line)}if(sessionMode){if(draft===undefined){line.innerHTML='';line.hidden=true}else{line.hidden=false;line.innerHTML=`<span>AKTIF SISTEM <b>${escapeHtml(configValueText(yaml))}</b></span><span>DRAFT <b>${escapeHtml(configValueText(draft))}</b></span>`}}else{line.hidden=false;line.innerHTML=`<span>BASE <b>${escapeHtml(configValueText(base))}</b></span><span>YAML <b>${escapeHtml(configValueText(yaml))}</b></span><span>DRAFT <b>${escapeHtml(configValueText(draft))}</b></span><span>RUNTIME <b>${escapeHtml(runtime)}</b></span>`}}const key=`param:${identity}`;registerParamHelp(key,p,{baseline:base,yaml,draft,runtime});if(!q('.help-trigger',row))row.insertAdjacentHTML('beforeend',`<button class="help-trigger" type="button" data-help-key="${escapeHtml(key)}" aria-label="Bantuan ${escapeHtml(p.label||p.yamlPath||p.key||'parameter')}">?</button>`)})}
const tuningQuickStatusByExperiment=new Map();
function tuningRuntimeCheckItems(x=selectedExperiment){
  const seen=new Set(),out=[];
  if(navigationGlobalProfileOwner(x)){
    for(const f of NAV_BAB4_GLOBAL_PROFILE_FIELDS){const m=schemaMeta(f.fileKey,f.path),id=`${f.fileKey}:${f.path}`;if(!m?.metadata_complete||m.write_authority!=='ros_yaml'||m.apply_mode==='read_only'||m.apply_mode==='startup_only'||seen.has(id))continue;seen.add(id);out.push({file_key:f.fileKey,path:f.path})}
    return out
  }
  for(const p of tuningFieldsFor(x)){
    if(!p?.yamlFileKey||!p?.yamlPath)continue;
    const m=schemaMetaForParam(p);
    if(!m?.metadata_complete||m.write_authority!=='ros_yaml'||m.apply_mode==='read_only'||m.apply_mode==='startup_only')continue;
    const id=`${p.yamlFileKey}:${p.yamlPath}`;if(seen.has(id))continue;seen.add(id);out.push({file_key:p.yamlFileKey,path:p.yamlPath});
  }
  return out;
}
function setTuningQuickState(state,status,detail,experimentId=selectedExperiment?.id){
  if(experimentId)tuningQuickStatusByExperiment.set(experimentId,{state,status,detail,at:Date.now()});
  const card=$('tuningMatchCard');if(card)card.dataset.state=state||'unknown';setText('tuningMatchStatus',status||'VERIFY');setText('tuningMatchDetail',detail||'Belum diverifikasi terhadap parameter ROS runtime.');
}
function renderTuningQuickState(){
  const items=tuningDraftRows(),n=items.length,ro=obj('server').read_only===true,busy=!!recordStartedMs,stored=tuningQuickStatusByExperiment.get(selectedExperiment?.id);
  const apply=$('tuningQuickApplyBtn'),verify=$('tuningQuickVerifyBtn'),discard=$('tuningQuickDiscardBtn'),baseline=$('tuningQuickBaselineBtn');
  if(apply){apply.disabled=ro||busy;apply.textContent=n?`APPLY TUNING (${n})`:'APPLY TUNING'}
  if(verify)verify.disabled=busy||n>0;if(discard)discard.disabled=!n||busy;
  if(baseline){const resettable=tuningFieldsFor(selectedExperiment).filter(p=>p.yamlFileKey&&p.kind!=='yaml_readonly'&&(!p.locked||isNavigationBab4Tuning())&&schemaMetaForParam(p)?.write_authority==='ros_yaml'&&(isNavigationBab4Tuning()?navigationSessionBaselineValue(p)!==undefined:baselineConfigValue(p.yamlFileKey,p.yamlPath)!==undefined));baseline.disabled=ro||busy||!resettable.length;baseline.title=isNavigationBab4Tuning()?'Kembali ke YAML aktif yang disimpan sebelum tuning sesi ini.':'Kembali ke baseline konfigurasi.'}
  if(n){
    const baselineDraft=items.every(x=>JSON.stringify(x.value)===JSON.stringify(navigationGlobalProfileOwner()?navigationGlobalProfileBaselineValueByIdentity(`${x.fileKey}:${x.path}`):isNavigationBab4Tuning()?navigationSessionBaselineValueByIdentity(`${x.fileKey}:${x.path}`):baselineConfigValue(x.fileKey,x.path)));
    const card=$('tuningMatchCard');if(card)card.dataset.state='draft';
    setText('tuningMatchStatus',baselineDraft?`BASELINE SESI • ${n} DRAFT`:`DRAFT • ${n} CHANGE`);
    setText('tuningMatchDetail',baselineDraft?'Nilai aktif awal sebelum tuning sudah disiapkan. Klik APPLY TUNING untuk kembali dengan runtime verify.':'Nilai GUI belum aktif di robot. Klik APPLY TUNING sebelum SEND GOAL.');
    return
  }
  if(stored){const card=$('tuningMatchCard');if(card)card.dataset.state=stored.state;setText('tuningMatchStatus',stored.status);setText('tuningMatchDetail',stored.detail);return}
  const check=tuningRuntimeCheckItems();if(!check.length){const card=$('tuningMatchCard');if(card)card.dataset.state='match';setText('tuningMatchStatus','NO YAML CHANGE');setText('tuningMatchDetail','Subbagian ini tidak memiliki parameter runtime yang perlu diverifikasi.');return}
  const card=$('tuningMatchCard');if(card)card.dataset.state='verify';setText('tuningMatchStatus','VERIFY');setText('tuningMatchDetail','Klik VERIFY MATCH atau APPLY TUNING untuk memastikan YAML = ROS runtime.');
}
async function verifySelectedTuningRuntime({silent=false}={}){
  const expId=selectedExperiment?.id,items=tuningRuntimeCheckItems();
  if(!expId||!items.length){setTuningQuickState('match','ACTIVE MATCH','Tidak ada parameter runtime editable pada subbagian ini.',expId);return {ok:true,runtime_match:true,checked_count:0}}
  if(tuningDraftRows().length){setTuningQuickState('draft','DRAFT','Masih ada nilai tuning yang belum di-apply.',expId);return {ok:false,runtime_match:false,reason:'DRAFT_PENDING'}}
  setTuningQuickState('applying','VERIFYING…',`Membaca kembali ${items.length} parameter ROS runtime.`,expId);
  try{
    const r=await writeRequest('/api/config/runtime-check',{items}),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    for(const row of j.items||[]){const st=row.status||'RUNTIME_MISMATCH';configRuntimeState.set(row.identity,{label:st,kind:st==='ACTIVE_MATCH'?'ready':st==='RUNTIME_MISMATCH'?'error':'pending',runtime:row.runtime,verifiedAt:Date.now()})}
    if(j.runtime_match===true){setTuningQuickState('match','ACTIVE MATCH',`YAML = ROS runtime • ${j.checked_count||items.length} parameter terverifikasi.`,expId);if(!silent)toast('Runtime tuning ACTIVE MATCH');return {...j,ok:true}}
    setTuningQuickState('error','NOT MATCH',`Runtime belum sama dengan YAML • ${j.checked_count||0} parameter dicek. Jangan SEND GOAL.`,expId);if(!silent)toast('Runtime tuning NOT MATCH',true);return {...j,ok:false};
  }catch(e){setTuningQuickState('error','VERIFY ERROR',e.message,expId);if(!silent)toast('Verify runtime gagal: '+e.message,true);return {ok:false,runtime_match:false,reason:e.message}}
  finally{decorateTuningFieldStates(tuningFieldsFor(selectedExperiment));}
}
async function stageVisibleTuningInputs(){const fields=tuningFieldsFor(selectedExperiment);for(let i=0;i<fields.length;i++){const e=$(`tuneField-${i}`);if(e&&!e.disabled&&!e.readOnly)await saveTuningField(i,true)}return tuningDraftRows()}
async function quickRestoreBaseline(){
  if(recordStartedMs)return toast('STOP recording dulu sebelum RESTORE BASELINE.',true);
  if(obj('server').read_only===true)return toast('Server read-only; Restore Baseline ditolak.',true);
  if(navigationGlobalProfileOwner()){stageNavigationGlobalProfile('baseline');return}
  if(isNavigationBab4Tuning()){
    const ready=await ensureNavigationSessionBaseline(selectedExperiment,{rerender:false});if(!ready)return;
    const row=navigationSessionBaselineRow(),values=row?.entry_snapshot?.values||{};
    const fields=tuningFieldsFor(selectedExperiment).filter(p=>p.yamlFileKey&&p.kind!=='yaml_readonly'&&(!p.locked||isNavigationBab4Tuning())&&schemaMetaForParam(p)?.write_authority==='ros_yaml'&&Object.prototype.hasOwnProperty.call(values,`${p.yamlFileKey}:${p.yamlPath}`));
    if(!fields.length)return toast('Baseline sesi belum memiliki parameter yang bisa direstore.',true);
    confirmModal('Restore baseline sesi',`${fields.length} parameter akan dikembalikan ke nilai YAML aktif yang tersimpan sebelum tuning sesi ini. Nilai hanya menjadi DRAFT sampai APPLY TUNING berhasil dan ACTIVE MATCH.`,async()=>{
      try{
        const r=await writeRequest('/api/navigation/tuning/entry',{id:selectedExperiment.id,action:'restore'}),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
        if(j.commissioning)commissioningState=j.commissioning;
        const items=j.proposal_items||[];
        if(!items.length){renderTuningFields();await verifySelectedTuningRuntime({silent:true});return toast('Sistem sudah sama dengan baseline sesi.')}
        for(const item of items)stageConfigChange(item.file_key,item.path,item.value,{name:`${selectedExperiment.id} Baseline Sesi • ${item.path.split('.').pop()}`,source:'navigation-session-baseline',generated:item.generated===true,proposalId:j.proposal?.proposal_id||''});
        renderTuningFields();renderConfigBrowser();setTuningQuickState('draft',`BASELINE SESI • ${items.length} DRAFT`,'Nilai aktif awal sebelum tuning sudah disiapkan. Tekan APPLY TUNING untuk kembali dengan runtime verify.');
        toast(`${items.length} nilai baseline sesi disiapkan sebagai DRAFT; runtime belum berubah.`);
      }catch(e){setTuningQuickState('error','RESTORE ERROR',e.message);toast('Restore baseline sesi gagal: '+e.message,true)}
    });
    return;
  }
  const fields=tuningFieldsFor(selectedExperiment).filter(p=>p.yamlFileKey&&p.kind!=='yaml_readonly'&&!p.locked&&schemaMetaForParam(p)?.write_authority==='ros_yaml'&&baselineConfigValue(p.yamlFileKey,p.yamlPath)!==undefined);
  if(!fields.length)return toast('Subbagian ini tidak memiliki baseline YAML yang bisa direstore.',true);
  confirmModal('Restore baseline awal',`${fields.length} parameter akan dikembalikan ke baseline konfigurasi sebagai DRAFT saja. Runtime belum berubah sampai APPLY TUNING.`,()=>{resetSelectedExperimentYaml()})
}
async function quickApplyTuning(){
  if(recordStartedMs)return toast('STOP recording dulu sebelum APPLY TUNING.',true);
  if(obj('server').read_only===true)return toast('Server read-only; Apply ditolak.',true);
  if(isNavigationBab4Tuning()){const baselineReady=await ensureNavigationSessionBaseline(selectedExperiment,{rerender:false});if(!baselineReady)return toast('APPLY diblok: baseline sesi belum tersimpan.',true)}
  const items=await stageVisibleTuningInputs();
  if(!items.length){toast('Tidak ada perubahan YAML; saya verifikasi runtime sekarang.');return verifySelectedTuningRuntime()}
  const expId=selectedExperiment?.id;
  confirmModal('Apply tuning ke sistem',`${items.length} perubahan akan otomatis divalidasi, disimpan ke YAML, node terkait direstart saat stationary, lalu runtime dibaca ulang. SEND GOAL tetap dikunci sampai ACTIVE MATCH.`,async()=>{
    setTuningQuickState('applying','VALIDATING…',`${items.length} parameter sedang divalidasi.`,expId);
    try{
      const validation=await validateConfigItems(items,{openReview:false});
      setTuningQuickState('applying','APPLYING…','YAML ditulis atomik + backup, lalu node runtime diaktifkan dan diverifikasi.',expId);
      const r=await writeRequest('/api/config/apply',{validation_id:validation.validation_id,items:configPayload(items)}),j=await r.json();
      if(!r.ok)throw new Error(`${j.code||''} ${j.message||`HTTP ${r.status}`}`.trim());
      for(const row of j.items||[]){const st=row.apply_status||'YAML_SAVED';configRuntimeState.set(row.identity,{label:st,kind:st==='ACTIVE_MATCH'?'ready':st==='RUNTIME_MISMATCH'?'error':'pending',runtime:row.runtime,savedValue:row.saved_value,transactionId:j.transaction_id,verifiedAt:Date.now()})}
      for(const x of items)configPending.delete(`${x.fileKey}:${x.path}`);
      configValidation=null;closeConfigDiff();await loadConfig();if(typeof loadTrials==='function')await loadTrials();window.navigationTuningPolicy?.onApplied?.();
      if(j.runtime_match!==true){const statuses=(j.items||[]).map(x=>x.apply_status).filter(Boolean).join(', ');setTuningQuickState('next-start','NOT ACTIVE',statuses||'Runtime belum MATCH.',expId);renderTuningFields();return toast('YAML tersimpan tetapi runtime belum ACTIVE MATCH; SEND GOAL tetap dikunci.',true)}
      renderTuningFields();const verified=await verifySelectedTuningRuntime({silent:true});if(verified.ok)toast(`APPLY TUNING selesai • ACTIVE MATCH • ${items.length} parameter aktif`);else toast('Apply selesai tetapi final runtime verify gagal.',true);
    }catch(e){configValidation=null;setTuningQuickState('error','APPLY FAILED',e.message,expId);renderTuningQuickState();toast('Apply tuning gagal: '+e.message,true)}
  })
}
function navigationMissionSpecs(){return navigationReportList().filter(x=>/^R4\.[234]\.[1-5]$/.test(String(x?.id||'')))}
function navigationMissionPendingDrafts(){
  if(typeof configPending==='undefined')return[];
  const ids=new Set();
  for(const spec of navigationMissionSpecs())for(const p of tuningFieldsFor(spec))if(p?.yamlFileKey&&p?.yamlPath)ids.add(`${p.yamlFileKey}:${p.yamlPath}`);
  return [...configPending.values()].filter(x=>ids.has(`${x.fileKey}:${x.path}`));
}
function navigationMissionRuntimeCheckItems(){
  const seen=new Set(),out=[];
  for(const spec of navigationMissionSpecs())for(const p of tuningFieldsFor(spec)){
    if(!p?.yamlFileKey||!p?.yamlPath)continue;
    const m=schemaMetaForParam(p);
    if(!m?.metadata_complete||m.write_authority!=='ros_yaml'||m.apply_mode==='read_only'||m.apply_mode==='startup_only')continue;
    const id=`${p.yamlFileKey}:${p.yamlPath}`;if(seen.has(id))continue;seen.add(id);out.push({file_key:p.yamlFileKey,path:p.yamlPath});
  }
  return out;
}
async function ensureNavigationTuningRuntimeReady(){
  const pending=navigationMissionPendingDrafts();if(pending.length)return {ok:false,reason:`${pending.length} DRAFT/STAGED belum di-apply untuk tuning 4.2–4.4`};
  const items=navigationMissionRuntimeCheckItems();if(!items.length)return {ok:true};
  setTuningQuickState('applying','VERIFYING 4.2–4.4…',`Memeriksa ${items.length} parameter YAML terhadap ROS runtime sebelum SEND GOAL.`);
  try{
    const r=await writeRequest('/api/config/runtime-check',{items}),j=await r.json();if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    for(const row of j.items||[]){const st=row.status||'RUNTIME_MISMATCH';configRuntimeState.set(row.identity,{label:st,kind:st==='ACTIVE_MATCH'?'ready':st==='RUNTIME_MISMATCH'?'error':'pending',runtime:row.runtime,verifiedAt:Date.now()})}
    if(j.runtime_match===true){setTuningQuickState('match','4.2–4.4 ACTIVE MATCH',`${j.checked_count||items.length} parameter runtime terverifikasi. SEND GOAL diizinkan.`);return {ok:true,runtime_match:true,checked_count:j.checked_count||items.length}}
    const bad=(j.items||[]).filter(x=>x.status!=='ACTIVE_MATCH').map(x=>x.identity).slice(0,3);
    setTuningQuickState('error','4.2–4.4 NOT MATCH',bad.length?`Mismatch: ${bad.join(', ')}`:'Ada parameter runtime yang tidak sama dengan YAML.');
    return {ok:false,reason:bad.length?`runtime mismatch: ${bad.join(', ')}`:'YAML dan ROS runtime 4.2–4.4 belum MATCH'};
  }catch(e){setTuningQuickState('error','VERIFY ERROR',e.message);return {ok:false,reason:e.message}}
}
window.ensureNavigationTuningRuntimeReady=ensureNavigationTuningRuntimeReady;
function updateTuningDraftControls(){const items=tuningDraftRows(),n=items.length,ro=obj('server').read_only===true,busy=!!recordStartedMs,validated=validationMatches(items);setText('tuningDraftCount',`${n} DRAFT${validated?' • VALIDATED':''}`);const v=$('tuningValidateDraft'),a=$('tuningApplyDraft'),r=$('tuningRevertDraft');if(v)v.disabled=!n;if(a)a.disabled=!n||ro||busy||!validated;if(r)r.disabled=!n;renderTuningQuickState()}
async function validateTuningDrafts(){const items=tuningDraftRows();if(!items.length)return toast('Tidak ada draft tuning.',true);try{const j=await validateConfigItems(items);toast(`Validate OK: ${items.length} draft • review diff sebelum Apply`)}catch(e){toast('Validate gagal: '+e.message,true)}}
function revertTuningDrafts(){const items=tuningDraftRows();for(const x of items)configPending.delete(`${x.fileKey}:${x.path}`);invalidateConfigValidation('draft reverted');renderTuningFields();renderConfigBrowser();window.navigationTuningPolicy?.render?.();toast(`${items.length} draft tuning dibuang; kembali ke CURRENT YAML aktif.`);if(currentExp==='navigation')verifySelectedTuningRuntime({silent:true})}
async function applyTuningDrafts(){const items=tuningDraftRows();if(!items.length)return toast('Tidak ada draft tuning.',true);if(obj('server').read_only===true)return toast('Server read-only; Apply ditolak.',true);if(!validationMatches(items))return toast('Validate + Diff Review wajib diulang sebelum Apply.',true);confirmModal('Konfirmasi ubah YAML + runtime',`${items.length} parameter tervalidasi akan mengubah YAML source-of-truth. Backend membuat backup, menulis atomik, me-restart node terkait saat kendaraan stationary, lalu membaca ulang parameter runtime. Jika read-back tidak MATCH, transaksi di-rollback otomatis.`,async()=>{try{const r=await writeRequest('/api/config/apply',{validation_id:configValidation.id,items:configPayload(items)}),j=await r.json();if(!r.ok)throw new Error(`${j.code||''} ${j.message||`HTTP ${r.status}`}`.trim());for(const row of j.items||[]){const id=row.identity,st=row.apply_status||'YAML_SAVED';configRuntimeState.set(id,{label:st,kind:st==='ACTIVE_MATCH'?'ready':st==='RUNTIME_MISMATCH'?'error':'pending',runtime:row.runtime,savedValue:row.saved_value,transactionId:j.transaction_id,verifiedAt:Date.now()})}for(const x of items)configPending.delete(`${x.fileKey}:${x.path}`);configValidation=null;closeConfigDiff();await loadConfig();if(typeof loadTrials==='function')await loadTrials();window.navigationTuningPolicy?.onApplied?.();renderTuningFields();window.renderEkfReportRuntimePanel?.();toast(j.runtime_match===true?`YAML TERSIMPAN + RUNTIME MATCH • transaction ${j.transaction_id||'--'} • ${items.length} parameter aktif`:`Transaction ${j.transaction_id||'--'} selesai • cek status runtime tiap parameter`)}catch(e){invalidateConfigValidation('apply rejected');toast('Apply tuning gagal: '+e.message,true)}})}
function resetSelectedExperimentYaml(){if(!selectedExperiment)return toast('Pilih tahap tuning terlebih dahulu.',true);if(recordStartedMs)return toast('STOP recording dulu sebelum stage baseline.',true);const fields=tuningFieldsFor(selectedExperiment).filter(p=>p.yamlFileKey&&p.kind!=='yaml_readonly'&&!p.locked&&schemaMetaForParam(p)?.write_authority==='ros_yaml'&&baselineConfigValue(p.yamlFileKey,p.yamlPath)!==undefined);let n=0;for(const p of fields){const id=`${p.yamlFileKey}:${p.yamlPath}`,cur=configValue(p.yamlFileKey,p.yamlPath),base=baselineConfigValue(p.yamlFileKey,p.yamlPath);if(JSON.stringify(cur)===JSON.stringify(base)){configPending.delete(id);continue}configPending.set(id,{fileKey:p.yamlFileKey,path:p.yamlPath,name:p.label||p.key||p.yamlPath,value:base,oldValue:cur,action:'reset',source:'tuning',experimentId:selectedExperiment.id});n++}invalidateConfigValidation('baseline staged');renderTuningFields();renderConfigBrowser();window.navigationTuningPolicy?.render?.();toast(n?`${n} baseline staged; YAML belum berubah.`:'Semua parameter sudah sama dengan baseline.')}


// N0 timing contract: keep configuration targets separate from measured runtime streams.
function renderN0TimingGuide(){
  const panel=$('n0TimingGuide');
  if(!panel)return;
  const active=currentExp==='navigation'&&selectedExperiment?.id==='N0.1';
  panel.hidden=!active;
  if(!active)return;
  const number=(v)=>Number.isFinite(+v)?+v:NaN;
  const cfg=(file,path)=>number(configValue(file,path));
  const mean=(path)=>{const st=metricStats(path);return number(st?.mean??resolveMetricPath(path))};
  const target={
    gnss:cfg('hmi','stmf4_hmi_bridge.ros__parameters.neo3pro_gnss_rate_hz'),
    gnssGate:cfg('hmi','stmf4_hmi_bridge.ros__parameters.neo3pro_min_usable_rate_hz'),
    imu:cfg('imu','data_imu_node.ros__parameters.publish_rate_hz'),
    esc:cfg('esc','esc_ackermann.ros__parameters.command_rate_hz'),
    local:cfg('ekf','ekf_filter_node_odom.ros__parameters.frequency'),
    global:cfg('ekf','ekf_filter_node_map.ros__parameters.frequency')
  };
  const actual={
    gnss:mean('derived.rate_gnss_hz'),imu:mean('derived.rate_imu_hz'),
    esc:mean('derived.rate_esc_hz'),local:mean('derived.rate_ekf_local_hz'),
    global:mean('derived.rate_ekf_global_hz')
  };
  const age={gnss:number(resolveMetricPath('derived.age_gnss_s')),imu:number(resolveMetricPath('derived.age_imu_s')),esc:number(resolveMetricPath('derived.age_esc_s'))};
  const match=(a,t)=>Number.isFinite(a)&&Number.isFinite(t)&&t>0&&Math.abs(a-t)<=Math.max(.5,.10*t);
  const label=(v,d=1)=>Number.isFinite(v)?v.toFixed(d):'--';
  setText('n0GnssTarget',label(target.gnss));setText('n0GnssActual',label(actual.gnss));setText('n0GnssGate',label(target.gnssGate));
  setText('n0ImuTarget',label(target.imu));setText('n0ImuActual',label(actual.imu));
  setText('n0EscCmdTarget',label(target.esc));setText('n0EscFeedbackActual',label(actual.esc));
  setText('n0LocalTarget',label(target.local));setText('n0LocalActual',label(actual.local));
  setText('n0GlobalTarget',label(target.global));setText('n0GlobalActual',label(actual.global));
  const ackTimeout=cfg('esc','esc_ackermann.ros__parameters.serial_ack_timeout_sec');
  const escFresh=Number.isFinite(actual.esc)&&actual.esc>0&&Number.isFinite(age.esc)&&age.esc<=Math.max(.1,Number.isFinite(ackTimeout)?ackTimeout:.6);
  const states={
    gnss:!Number.isFinite(actual.gnss)?['WAIT','wait']:(Number.isFinite(target.gnssGate)&&actual.gnss>target.gnssGate)?(match(actual.gnss,target.gnss)?['MATCH','ok']:['USABLE','ok']):['RATE < GATE','warn'],
    imu:!Number.isFinite(actual.imu)?['WAIT','wait']:match(actual.imu,target.imu)?['MATCH','ok']:['CHECK','warn'],
    esc:!Number.isFinite(actual.esc)?['WAIT','wait']:escFresh?['FB LIVE','ok']:['STALE','warn'],
    local:!Number.isFinite(actual.local)?['WAIT','wait']:match(actual.local,target.local)?['MATCH','ok']:['CHECK','warn'],
    global:!Number.isFinite(actual.global)?['WAIT','wait']:match(actual.global,target.global)?['MATCH','ok']:['CHECK','warn']
  };
  for(const key of ['gnss','imu','esc','local','global']){
    const card=panel.querySelector(`[data-n0-card="${key}"]`);if(card)card.dataset.state=states[key][1];
    const id={gnss:'n0GnssStatus',imu:'n0ImuStatus',esc:'n0EscStatus',local:'n0LocalStatus',global:'n0GlobalStatus'}[key];setText(id,states[key][0]);
  }
  const anyData=Object.values(actual).some(Number.isFinite);
  const rateReady=['gnss','imu','local','global'].every(k=>states[k][1]==='ok')&&states.esc[1]==='ok';
  const chip=$('n0TimingState');if(chip){chip.className='status-chip '+(!anyData?'waiting':rateReady?'':'waiting');chip.textContent=!anyData?'WAIT DATA':rateReady?'RATE READY':'REVIEW';}
  const directSummary=[['GNSS',actual.gnss,target.gnss],['IMU',actual.imu,target.imu],['EKF-L',actual.local,target.local],['EKF-G',actual.global,target.global]].map(([n,a,t])=>`${n} ${label(a)}/${label(t)} Hz`).join(' • ');
  setText('n0RateReadiness',anyData?`${directSummary} • GNSS usable gate >${label(target.gnssGate)} Hz • ESC feedback ${label(actual.esc)} Hz (tidak dibandingkan 1:1 ke command ${label(target.esc)} Hz)`:'Belum ada cukup data aktual.');
  setText('n0FreshnessSummary',`GNSS ${label(age.gnss,3)} s • IMU ${label(age.imu,3)} s • ESC feedback ${label(age.esc,3)} s`);
  setText('n0NextStepSummary',rateReady?'Rate path siap: GNSS > gate 7 Hz dan stream lain sehat. Target produksi GNSS tetap 10 Hz; selesaikan RECORD N0 dan evidence sebelum N1.':'Tahan N0 sampai GNSS > gate 7 Hz dan stream aktual lain stabil. Target produksi GNSS tetap 10 Hz.');
}
window.renderN0TimingGuide=renderN0TimingGuide;
setInterval(renderN0TimingGuide,1000);

// Experiments workbench scheduler. Keep fast telemetry/chart refresh separate from
// the slower table/preflight DOM work so the research UI stays responsive.
let labRenderQueued=false,lastLabRenderAt=0,lastLabSlowAt=0,recorderIdlePoll=0;
function queueLabWorkbench(force=false){
  if(activePage!=='experiments'||document.hidden||!selectedExperiment||labRenderQueued)return;
  labRenderQueued=true;
  const delay=force?0:Math.max(0,125-(performance.now()-lastLabRenderAt));
  const run=()=>requestAnimationFrame(()=>{
    labRenderQueued=false;
    if(activePage!=='experiments'||document.hidden||!selectedExperiment)return;
    lastLabRenderAt=performance.now();
    renderLabWorkbench(force);
  });
  delay>1?setTimeout(run,delay):run();
}
function renderLabWorkbench(force=false){
  if(activePage!=='experiments'||!selectedExperiment)return;
  captureLabTelemetry();
  if(window.uiPerf)uiPerf.labFast=(uiPerf.labFast||0)+1;
  drawAllExperimentCharts();
  renderContextVisual();
  const now=performance.now();
  if(force||now-lastLabSlowAt>=1000){
    lastLabSlowAt=now;
    if(window.uiPerf)uiPerf.labSlow=(uiPerf.labSlow||0)+1;
    renderExperimentTable();
    renderSourceAudit();
    renderPerceptionEngineering();
    renderTestPreflight();
    window.renderEkfReportRuntimePanel?.();
    window.renderReport41TimedPanel?.();
    window.renderImuHeadingLinearity?.();
  }
}
setInterval(()=>{pollRecorder()},250);

// FIRDA 4.2.1 one-page helper. Reuses the existing Workbench controls and recorder.
let firdaKpPulseCount=0,firdaKpPulseBusy=false,firdaKpAutoBusy=false,firdaKpAbort=false;
function firdaKpActive(){return currentExp==='steering'&&String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)==='4.2.1'}
function firdaKpTune(){const t=obj('vesc_tuning_state');return (+t.motor===1&&+t.status===0)?t:null}
function firdaKpSetText(id,v){const e=$(id);if(!e)return;if(e.matches?.('input,textarea,select'))e.value=v;else e.textContent=v}
function firdaKpMeta(v){if(Math.abs(v-.4)<.08)return['KP-05X','candidate-a'];if(Math.abs(v-.8)<.12)return['KP-10X','candidate-b'];if(Math.abs(v-1.2)<.15)return['KP-15X','candidate-c'];return['KP-'+Number(v).toFixed(3),'candidate-a']}
function firdaKpApplyIdentity(v){const [variation,candidate]=firdaKpMeta(v);if($('runVariation'))$('runVariation').value=variation;if($('runCondition'))$('runCondition').value='IQ_STEP_1A';if($('runCandidate'))$('runCandidate').value=candidate}
function renderFirdaKpQuickPanel(){
  const p=$('firdaKpQuickPanel');if(!p)return;const active=firdaKpActive();p.hidden=!active;if(!active)return;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=maint?obj('vesc_left_values'):((obj('foc_telemetry')||{}).left||{}),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$('firdaKpModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaKpActual'))$('firdaKpActual').value=t?fmt(t.foc_q_kp,6):'--';
  if($('firdaKiActual'))$('firdaKiActual').value=t?fmt(t.foc_q_ki,6):'--';
  firdaKpSetText('firdaKpLive','Iq '+fmt(lv.iq_a,3)+' A • Id '+fmt(lv.id_a,3)+' A • Imotor '+fmt(lv.current_motor_a,3)+' A • Duty '+fmt(lv.duty,4)+' • Vbus '+fmt(lv.vbus_v,2)+' V');
  firdaKpSetText('firdaKpPulseCount','Pulse: '+firdaKpPulseCount+' / 3');
  if($('firdaKpEnterMaint')){$('firdaKpEnterMaint').disabled=maint||firdaKpPulseBusy;$('firdaKpEnterMaint').textContent=maint?'1. MAINTENANCE ACTIVE':'1. WEB MAINTENANCE'}
  if($('firdaKpRead'))$('firdaKpRead').disabled=!maint||firdaKpPulseBusy;
  if($('firdaKpApply'))$('firdaKpApply').disabled=!maint||!t||firdaKpPulseBusy;
  if($('firdaKpAuto')){$('firdaKpAuto').disabled=rec||firdaKpPulseBusy||firdaKpAutoBusy;$('firdaKpAuto').textContent=firdaKpAutoBusy?'TEST OTOMATIS BERJALAN…':'START TEST OTOMATIS • 3 PULSE'}
  if($('firdaKpRecordStart'))$('firdaKpRecordStart').disabled=!maint||rec||firdaKpPulseBusy||firdaKpAutoBusy;
  if($('firdaKpPulse'))$('firdaKpPulse').disabled=!maint||!rec||firdaKpPulseBusy||firdaKpAutoBusy||firdaKpPulseCount>=3;
  if($('firdaKpRecordStop'))$('firdaKpRecordStop').disabled=!rec||firdaKpPulseBusy||firdaKpAutoBusy;
}

async function firdaKpWait(predicate,timeoutMs=5000,stepMs=80){
  const t0=Date.now();
  while(Date.now()-t0<timeoutMs){
    try{if(predicate())return true}catch(_){}
    await sleep(stepMs);
  }
  return false;
}
async function firdaKpCommand(command,timeoutMs=12000){
  const r=await writeRequest('/api/esc/vesc/command',{command},{timeoutMs}),j=await r.json();
  if(!r.ok)throw new Error(j.message||('VESC command gagal: '+command));
  return j;
}
async function firdaKpPollLeft(durationMs,intervalMs=90){
  const until=Date.now()+Math.max(0,durationMs);
  while(Date.now()<until){
    if(firdaKpAbort)throw new Error('Test dibatalkan');
    await firdaKpCommand('VALUES:1',5000);
    await sleep(intervalMs);
  }
}
async function firdaKpAutoExecute(){
  if(firdaKpAutoBusy)return;
  const kp=+$('firdaKpValue').value;
  if(!Number.isFinite(kp)||kp<=0||kp>5)return toast('Kp candidate tidak valid.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaKpAutoBusy=true;firdaKpAbort=false;firdaKpPulseCount=0;firdaKpApplyIdentity(kp);if($('runSampleRate'))$('runSampleRate').value='50';renderFirdaKpQuickPanel();
  try{
    firdaKpSetText('firdaKpApplyState','Menyiapkan Web Maintenance…');
    if(!bool(raw('vesc_maintenance_active'))){
      await firdaKpCommand('MODE:MAINTENANCE');
      if(!await firdaKpWait(()=>bool(raw('vesc_maintenance_active')),5000))throw new Error('Maintenance tidak aktif');
    }
    if(firdaKpAbort)throw new Error('Test dibatalkan');
    const initialReadbackBefore=+updated.vesc_tuning_state||0;await firdaKpCommand('TUNING:GET:1');
    if(!await firdaKpWait(()=>(+updated.vesc_tuning_state||0)>initialReadbackBefore&&!!firdaKpTune(),3500))throw new Error('Read LEFT tuning baru gagal');
    const t=firdaKpTune(),keys=['foc_q_ki','foc_d_kp','foc_d_ki','speed_kp','speed_ki','speed_kd','pos_kp','pos_ki','pos_kd','current_filter'];
    if(!keys.every(k=>Number.isFinite(+t[k])))throw new Error('Readback tuning tidak lengkap');
    const vals=[kp,...keys.map(k=>+t[k])];
    firdaKpSetText('firdaKpApplyState','Apply Q Kp '+kp.toFixed(3)+' • runtime only…');
    await firdaKpCommand('TUNING:SET:1:'+vals.join(':')+':0');
    await sleep(250);const readbackBefore=+updated.vesc_tuning_state||0;await firdaKpCommand('TUNING:GET:1');
    if(!await firdaKpWait(()=>(+updated.vesc_tuning_state||0)>readbackBefore&&!!firdaKpTune(),3500))throw new Error('Readback Kp baru tidak diterima');
    const rb=firdaKpTune(),actual=rb?+rb.foc_q_kp:NaN;
    if(!Number.isFinite(actual)||Math.abs(actual-kp)>.01)throw new Error('Kp readback tidak sesuai');
    await firdaKpCommand('SAFE_STOP:BOTH');
    await firdaKpCommand('VALUES:1',5000);
    if(!await firdaKpWait(()=>age('vesc_left_values')<1.0,2000))throw new Error('Telemetry LEFT belum fresh');
    firdaKpSetText('firdaKpApplyState','Mulai recording • baseline 0 A…');
    await startWebRecording();
    if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    await firdaKpPollLeft(700,70);
    for(let i=0;i<3;i++){
      if(firdaKpAbort)throw new Error('Test dibatalkan');
      firdaKpSetText('firdaKpApplyState','Pulse '+(i+1)+'/3 • Iq 1.0 A selama 1 s');
      await firdaKpCommand('SET:CURRENT:1:1.0');
      await firdaKpPollLeft(1000,60);
      await firdaKpCommand('SAFE_STOP:BOTH');
      await firdaKpPollLeft(250,60);
      firdaKpPulseCount=i+1;renderFirdaKpQuickPanel();
      firdaKpSetText('firdaKpApplyState','Pulse '+firdaKpPulseCount+'/3 selesai • rest 2 s');
      await firdaKpPollLeft(1750,80);
    }
    firdaKpSetText('firdaKpApplyState','Menyimpan CSV / XLSX / PNG…');
    const saved=await stopWebRecording();
    if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');
    await sleep(300);
    await firdaKpCommand('SAFE_STOP:BOTH');
    await firdaKpCommand('MODE:RUNTIME',15000);
    await firdaKpWait(()=>!bool(raw('vesc_maintenance_active')),5000);
    firdaKpSetText('firdaKpApplyState','SELESAI • '+firdaKpMeta(kp)[0]+' • 3 pulse tersimpan • kembali RUNTIME');
    toast('4.2.1 '+firdaKpMeta(kp)[0]+' selesai dan tersimpan',false);
  }catch(e){
    try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}
    try{if($('recordToggleBtn')?.dataset.active==='true')await stopWebRecording()}catch(_){}
    try{if(bool(raw('vesc_maintenance_active')))await firdaKpCommand('MODE:RUNTIME',15000)}catch(_){}
    firdaKpSetText('firdaKpApplyState','TEST STOP: '+(e?.message||e));
    toast('Tuning Kp berhenti: '+(e?.message||e),true);
  }finally{
    firdaKpAutoBusy=false;renderFirdaKpQuickPanel();
  }
}
function firdaKpBind(){
  const p=$('firdaKpQuickPanel');if(!p||p.dataset.bound==='1')return;p.dataset.bound='1';
  const preset=(id,v)=>{$(id).onclick=()=>{if($('firdaKpValue'))$('firdaKpValue').value=String(v);firdaKpApplyIdentity(v);firdaKpPulseCount=0;renderFirdaKpQuickPanel()}};
  preset('firdaKpPresetLow',0.4);preset('firdaKpPresetMid',0.8);preset('firdaKpPresetHigh',1.2);
  $('firdaKpValue').onchange=()=>{const v=+$('firdaKpValue').value;if(Number.isFinite(v)){firdaKpApplyIdentity(v);firdaKpPulseCount=0}renderFirdaKpQuickPanel()};
  $('firdaKpEnterMaint').onclick=()=>{$('vescEnterMaintenance')?.click()};
  $('firdaKpRead').onclick=()=>{firdaKpSetText('firdaKpApplyState','Reading LEFT tuning…');$('vescLReadTuning')?.click()};
  $('firdaKpApply').onclick=()=>{
    const kp=+$('firdaKpValue').value;if(!Number.isFinite(kp)||kp<=0||kp>5)return toast('Kp candidate tidak valid.',true);
    const q=$('vescLTuneFocQKp'),store=$('vescLTuneStore'),write=$('vescLWriteTuning');
    if(!q||!store||!write)return toast('Workbench LEFT tuning belum tersedia.',true);
    q.value=String(kp);store.checked=false;firdaKpApplyIdentity(kp);firdaKpPulseCount=0;
    firdaKpSetText('firdaKpApplyState','Q Kp '+kp.toFixed(3)+' siap • confirm Runtime only.');
    write.click();
  };
  $('firdaKpAuto').onclick=()=>confirmModal('START TEST Kp OTOMATIS','Roda steering wajib terangkat dan area gerak aman. Sistem akan memberi 3 pulse LEFT Iq 1.0 A selama 1 s, otomatis safe-stop di tiap pulse, menyimpan data, lalu kembali ke RUNTIME.',()=>firdaKpAutoExecute());
  $('firdaKpRecordStart').onclick=()=>{firdaKpPulseCount=0;$('recordToggleBtn')?.click()};
  $('firdaKpRecordStop').onclick=()=>{$('recordToggleBtn')?.click()};
  $('firdaKpStop').onclick=async()=>{firdaKpAbort=true;try{await vescCmd('SAFE_STOP:BOTH',true)}catch(_){}firdaKpSetText('firdaKpApplyState','STOP CURRENT • membatalkan test otomatis…')};
  $('firdaKpPulse').onclick=async()=>{
    if(firdaKpPulseBusy)return;if($('recordToggleBtn')?.dataset.active!=='true')return toast('START recorder dulu.',true);
    const input=$('vescLeftCurrentInput'),set=$('vescLeftSetCurrent'),stop=$('vescSendZero');if(!input||!set||!stop)return toast('Direct LEFT control belum tersedia.',true);
    firdaKpPulseBusy=true;renderFirdaKpQuickPanel();
    try{firdaKpSetText('firdaKpApplyState','Pulse '+(firdaKpPulseCount+1)+'/3 • 1.0 A');input.value='1.0';set.click();await sleep(1000);stop.click();firdaKpPulseCount++;firdaKpSetText('firdaKpApplyState','Pulse '+firdaKpPulseCount+'/3 selesai • jeda 2 s.');await sleep(2000)}
    finally{firdaKpPulseBusy=false;renderFirdaKpQuickPanel()}
  };
}
firdaKpBind();
setInterval(()=>{if(firdaKpActive())renderFirdaKpQuickPanel()},250);

// FIRDA 4.2.2 one-page Ki current-loop test. Q Kp is fixed from 4.2.1.
const FIRDA_Q_KP_FINAL=0.800130208;
let firdaKiPulseCount=0,firdaKiAutoBusy=false,firdaKiAbort=false;
function firdaKiActive(){return currentExp==='steering'&&String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)==='4.2.2'}
function firdaKiMeta(v){
  if(Math.abs(v-75)<20)return['KI-05X','candidate-a'];
  if(Math.abs(v-150)<25)return['KI-10X','candidate-b'];
  if(Math.abs(v-225)<30)return['KI-15X','candidate-c'];
  return['KI-'+Number(v).toFixed(1),'candidate-a'];
}
function firdaKiApplyIdentity(v){
  const [variation,candidate]=firdaKiMeta(v);
  if($('runVariation'))$('runVariation').value=variation;
  if($('runCondition'))$('runCondition').value='IQ_STEP_1A';
  if($('runCandidate'))$('runCandidate').value=candidate;
}
function renderFirdaKiQuickPanel(){
  const p=$('firdaKiQuickPanel');if(!p)return;
  const active=firdaKiActive();p.hidden=!active;if(!active)return;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=maint?obj('vesc_left_values'):((obj('foc_telemetry')||{}).left||{}),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$('firdaKiModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaKiFixedKp'))$('firdaKiFixedKp').value=FIRDA_Q_KP_FINAL.toFixed(6);
  if($('firdaKiActual2'))$('firdaKiActual2').value=t?fmt(t.foc_q_ki,6):'--';
  firdaKpSetText('firdaKiLive','Iq '+fmt(lv.iq_a,3)+' A • Id '+fmt(lv.id_a,3)+' A • Imotor '+fmt(lv.current_motor_a,3)+' A • Duty '+fmt(lv.duty,4)+' • Vbus '+fmt(lv.vbus_v,2)+' V');
  firdaKpSetText('firdaKiPulseCount','Pulse: '+firdaKiPulseCount+' / 3');
  if($('firdaKiAuto')){$('firdaKiAuto').disabled=rec||firdaKiAutoBusy;$('firdaKiAuto').textContent=firdaKiAutoBusy?'TEST OTOMATIS BERJALAN…':'START TEST OTOMATIS • 3 PULSE'}
}
async function firdaKiPollLeft(durationMs,intervalMs=90){
  const until=Date.now()+Math.max(0,durationMs);
  while(Date.now()<until){
    if(firdaKiAbort)throw new Error('Test dibatalkan');
    await firdaKpCommand('VALUES:1',5000);
    await sleep(intervalMs);
  }
}
async function firdaKiAutoExecute(){
  if(firdaKiAutoBusy)return;
  const ki=+$('firdaKiValue').value;
  if(!Number.isFinite(ki)||ki<0||ki>1000)return toast('Ki candidate tidak valid.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaKiAutoBusy=true;firdaKiAbort=false;firdaKiPulseCount=0;firdaKiApplyIdentity(ki);if($('runSampleRate'))$('runSampleRate').value='50';renderFirdaKiQuickPanel();
  try{
    firdaKpSetText('firdaKiState','Menyiapkan Web Maintenance…');
    if(!bool(raw('vesc_maintenance_active'))){
      await firdaKpCommand('MODE:MAINTENANCE');
      if(!await firdaKpWait(()=>bool(raw('vesc_maintenance_active')),5000))throw new Error('Maintenance tidak aktif');
    }
    const kiInitialReadbackBefore=+updated.vesc_tuning_state||0;await firdaKpCommand('TUNING:GET:1');
    if(!await firdaKpWait(()=>(+updated.vesc_tuning_state||0)>kiInitialReadbackBefore&&!!firdaKpTune(),3500))throw new Error('Read LEFT tuning baru gagal');
    const t=firdaKpTune(),keys=['foc_d_kp','foc_d_ki','speed_kp','speed_ki','speed_kd','pos_kp','pos_ki','pos_kd','current_filter'];
    if(!keys.every(k=>Number.isFinite(+t[k])))throw new Error('Readback tuning tidak lengkap');
    const vals=[FIRDA_Q_KP_FINAL,ki,...keys.map(k=>+t[k])];
    firdaKpSetText('firdaKiState','Apply Q Kp 0.80013 + Q Ki '+ki.toFixed(1)+' • runtime only…');
    await firdaKpCommand('TUNING:SET:1:'+vals.join(':')+':0');
    await sleep(250);const kiReadbackBefore=+updated.vesc_tuning_state||0;await firdaKpCommand('TUNING:GET:1');
    if(!await firdaKpWait(()=>(+updated.vesc_tuning_state||0)>kiReadbackBefore&&!!firdaKpTune(),3500))throw new Error('Readback Ki baru tidak diterima');
    const rb=firdaKpTune(),actualKp=rb?+rb.foc_q_kp:NaN,actualKi=rb?+rb.foc_q_ki:NaN;
    if(!Number.isFinite(actualKp)||Math.abs(actualKp-FIRDA_Q_KP_FINAL)>.01)throw new Error('Fixed Kp readback tidak sesuai');
    if(!Number.isFinite(actualKi)||Math.abs(actualKi-ki)>.5)throw new Error('Ki readback tidak sesuai');
    await firdaKpCommand('SAFE_STOP:BOTH');
    await firdaKpCommand('VALUES:1',5000);
    if(!await firdaKpWait(()=>age('vesc_left_values')<1.0,2000))throw new Error('Telemetry LEFT belum fresh');
    firdaKpSetText('firdaKiState','Mulai recording • baseline 0 A…');
    await startWebRecording();
    if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    await firdaKiPollLeft(700,70);
    for(let i=0;i<3;i++){
      if(firdaKiAbort)throw new Error('Test dibatalkan');
      firdaKpSetText('firdaKiState','Pulse '+(i+1)+'/3 • Iq 1.0 A selama 1 s');
      await firdaKpCommand('SET:CURRENT:1:1.0');
      await firdaKiPollLeft(1000,60);
      await firdaKpCommand('SAFE_STOP:BOTH');
      await firdaKiPollLeft(250,60);
      firdaKiPulseCount=i+1;renderFirdaKiQuickPanel();
      firdaKpSetText('firdaKiState','Pulse '+firdaKiPulseCount+'/3 selesai • rest 2 s');
      await firdaKiPollLeft(1750,80);
    }
    firdaKpSetText('firdaKiState','Menyimpan CSV / XLSX / PNG…');
    const saved=await stopWebRecording();
    if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');
    await sleep(300);await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpCommand('MODE:RUNTIME',15000);
    await firdaKpWait(()=>!bool(raw('vesc_maintenance_active')),5000);
    firdaKpSetText('firdaKiState','SELESAI • '+firdaKiMeta(ki)[0]+' • 3 pulse tersimpan • Kp tetap 0.80013');
    toast('4.2.2 '+firdaKiMeta(ki)[0]+' selesai dan tersimpan',false);
  }catch(e){
    try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}
    try{if($('recordToggleBtn')?.dataset.active==='true')await stopWebRecording()}catch(_){}
    try{if(bool(raw('vesc_maintenance_active')))await firdaKpCommand('MODE:RUNTIME',15000)}catch(_){}
    firdaKpSetText('firdaKiState','TEST STOP: '+(e?.message||e));
    toast('Tuning Ki berhenti: '+(e?.message||e),true);
  }finally{firdaKiAutoBusy=false;renderFirdaKiQuickPanel()}
}
function firdaKiBind(){
  const p=$('firdaKiQuickPanel');if(!p||p.dataset.bound==='1')return;p.dataset.bound='1';
  const preset=(id,v)=>{$(id).onclick=()=>{if($('firdaKiValue'))$('firdaKiValue').value=String(v);firdaKiApplyIdentity(v);firdaKiPulseCount=0;renderFirdaKiQuickPanel()}};
  preset('firdaKiPresetLow',75);preset('firdaKiPresetMid',150);preset('firdaKiPresetHigh',225);
  $('firdaKiValue').onchange=()=>{const v=+$('firdaKiValue').value;if(Number.isFinite(v)){firdaKiApplyIdentity(v);firdaKiPulseCount=0}renderFirdaKiQuickPanel()};
  $('firdaKiAuto').onclick=()=>confirmModal('START TEST Ki OTOMATIS','Roda steering wajib terangkat dan area gerak aman. Kp akan dikunci 0.80013. Sistem menguji Ki terpilih dengan 3 pulse LEFT Iq 1.0 A × 1 s, runtime-only, lalu menyimpan data dan kembali RUNTIME.',()=>firdaKiAutoExecute());
  $('firdaKiStop').onclick=async()=>{firdaKiAbort=true;try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}firdaKpSetText('firdaKiState','STOP CURRENT • membatalkan test otomatis…')};
}
firdaKiBind();
setInterval(()=>{if(firdaKiActive())renderFirdaKiQuickPanel()},250);

// FIRDA 4.2.3 one-page validation of final PI current gains.
const FIRDA_Q_KI_FINAL=266.0;
let firdaPiPulseCount=0,firdaPiAutoBusy=false,firdaPiAbort=false;
function firdaPiActive(){return currentExp==='steering'&&String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)==='4.2.3'}
function firdaPiMeta(iq){
  const s=Number(iq).toFixed(1);
  return ['PI-IQ'+s+'A','KP0.80013_KI266'];
}
function firdaPiApplyIdentity(iq){
  const [variation,condition]=firdaPiMeta(iq);
  if($('runVariation'))$('runVariation').value=variation;
  if($('runCondition'))$('runCondition').value=condition;
  if($('runCandidate'))$('runCandidate').value='final';
}
function renderFirdaPiQuickPanel(){
  const p=$('firdaPiQuickPanel');if(!p)return;
  const active=firdaPiActive();p.hidden=!active;if(!active)return;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=maint?obj('vesc_left_values'):((obj('foc_telemetry')||{}).left||{}),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$('firdaPiModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaPiKp'))$('firdaPiKp').value=FIRDA_Q_KP_FINAL.toFixed(6);
  if($('firdaPiKi'))$('firdaPiKi').value=FIRDA_Q_KI_FINAL.toFixed(3);
  if($('firdaPiActual'))$('firdaPiActual').value=t?(fmt(t.foc_q_kp,6)+' / '+fmt(t.foc_q_ki,6)):'-- / --';
  firdaKpSetText('firdaPiLive','Iq '+fmt(lv.iq_a,3)+' A • Id '+fmt(lv.id_a,3)+' A • Imotor '+fmt(lv.current_motor_a,3)+' A • Duty '+fmt(lv.duty,4)+' • Vbus '+fmt(lv.vbus_v,2)+' V');
  firdaKpSetText('firdaPiPulseCount','Pulse: '+firdaPiPulseCount+' / 3');
  if($('firdaPiAuto')){$('firdaPiAuto').disabled=rec||firdaPiAutoBusy;$('firdaPiAuto').textContent=firdaPiAutoBusy?'VALIDASI BERJALAN…':'START VALIDASI OTOMATIS • 3 PULSE'}
}
async function firdaPiPollLeft(durationMs,intervalMs=90){
  const until=Date.now()+Math.max(0,durationMs);
  while(Date.now()<until){
    if(firdaPiAbort)throw new Error('Validasi dibatalkan');
    await firdaKpCommand('VALUES:1',5000);
    await sleep(intervalMs);
  }
}
async function firdaPiAutoExecute(){
  if(firdaPiAutoBusy)return;
  const iq=+$('firdaPiIqValue').value;
  if(!Number.isFinite(iq)||iq<0.1||iq>3)return toast('Iq reference tidak valid.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaPiAutoBusy=true;firdaPiAbort=false;firdaPiPulseCount=0;firdaPiApplyIdentity(iq);localRunValues[`${currentExp}:${selectedExperiment.id}:iq_ref_a`]=iq;localRunValues[`${currentExp}:${selectedExperiment.id}:gain_variant`]='final';if($('runSampleRate'))$('runSampleRate').value='50';renderTuningFields();renderFirdaPiQuickPanel();
  try{
    firdaKpSetText('firdaPiState','Menyiapkan gain final PI…');
    if(!bool(raw('vesc_maintenance_active'))){
      await firdaKpCommand('MODE:MAINTENANCE');
      if(!await firdaKpWait(()=>bool(raw('vesc_maintenance_active')),5000))throw new Error('Maintenance tidak aktif');
    }
    await firdaKpCommand('TUNING:GET:1');
    if(!await firdaKpWait(()=>!!firdaKpTune(),2500))throw new Error('Read LEFT tuning gagal');
    const t=firdaKpTune(),keys=['foc_d_kp','foc_d_ki','speed_kp','speed_ki','speed_kd','pos_kp','pos_ki','pos_kd','current_filter'];
    if(!keys.every(k=>Number.isFinite(+t[k])))throw new Error('Readback tuning tidak lengkap');
    const vals=[FIRDA_Q_KP_FINAL,FIRDA_Q_KI_FINAL,...keys.map(k=>+t[k])];
    await firdaKpCommand('TUNING:SET:1:'+vals.join(':')+':0');
    await sleep(250);await firdaKpCommand('TUNING:GET:1');await sleep(250);
    const rb=firdaKpTune(),akp=rb?+rb.foc_q_kp:NaN,aki=rb?+rb.foc_q_ki:NaN;
    if(!Number.isFinite(akp)||Math.abs(akp-FIRDA_Q_KP_FINAL)>.01)throw new Error('Q Kp final readback tidak sesuai');
    if(!Number.isFinite(aki)||Math.abs(aki-FIRDA_Q_KI_FINAL)>.5)throw new Error('Q Ki final readback tidak sesuai');
    firdaKpSetText('firdaPiState','Gain final verified • Kp '+akp.toFixed(6)+' • Ki '+aki.toFixed(3));
    await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpCommand('VALUES:1',5000);
    if(!await firdaKpWait(()=>age('vesc_left_values')<1.0,2000))throw new Error('Telemetry LEFT belum fresh');
    await startWebRecording();
    if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    await firdaPiPollLeft(700,70);
    for(let i=0;i<3;i++){
      if(firdaPiAbort)throw new Error('Validasi dibatalkan');
      firdaKpSetText('firdaPiState','Pulse '+(i+1)+'/3 • Iq '+iq.toFixed(1)+' A selama 1 s');
      await firdaKpCommand('SET:CURRENT:1:'+iq.toFixed(3));
      await firdaPiPollLeft(1000,60);
      await firdaKpCommand('SAFE_STOP:BOTH');
      await firdaPiPollLeft(250,60);
      firdaPiPulseCount=i+1;renderFirdaPiQuickPanel();
      firdaKpSetText('firdaPiState','Pulse '+firdaPiPulseCount+'/3 selesai • rest 2 s');
      await firdaPiPollLeft(1750,80);
    }
    firdaKpSetText('firdaPiState','Menyimpan CSV / XLSX / PNG…');
    const saved=await stopWebRecording();
    if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');
    await sleep(300);await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpCommand('MODE:RUNTIME',15000);
    await firdaKpWait(()=>!bool(raw('vesc_maintenance_active')),5000);
    firdaKpSetText('firdaPiState','SELESAI • '+firdaPiMeta(iq)[0]+' • 3 pulse tersimpan • gain final Kp 0.80013 Ki 266');
    toast('4.2.3 '+firdaPiMeta(iq)[0]+' selesai dan tersimpan',false);
  }catch(e){
    try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}
    try{if($('recordToggleBtn')?.dataset.active==='true')await stopWebRecording()}catch(_){}
    try{if(bool(raw('vesc_maintenance_active')))await firdaKpCommand('MODE:RUNTIME',15000)}catch(_){}
    firdaKpSetText('firdaPiState','VALIDASI STOP: '+(e?.message||e));
    toast('Validasi PI berhenti: '+(e?.message||e),true);
  }finally{firdaPiAutoBusy=false;renderFirdaPiQuickPanel()}
}
function firdaPiBind(){
  const p=$('firdaPiQuickPanel');if(!p||p.dataset.bound==='1')return;p.dataset.bound='1';
  const preset=(id,v)=>{$(id).onclick=()=>{if($('firdaPiIqValue'))$('firdaPiIqValue').value=String(v);firdaPiApplyIdentity(v);firdaPiPulseCount=0;renderFirdaPiQuickPanel()}};
  preset('firdaPiPreset05',0.5);preset('firdaPiPreset10',1.0);preset('firdaPiPreset15',1.5);
  $('firdaPiIqValue').onchange=()=>{const v=+$('firdaPiIqValue').value;if(Number.isFinite(v)){firdaPiApplyIdentity(v);firdaPiPulseCount=0}renderFirdaPiQuickPanel()};
  $('firdaPiAuto').onclick=()=>confirmModal('START VALIDASI PI OTOMATIS','Roda steering wajib terangkat dan area gerak aman. Gain final Q Kp 0.80013 + Q Ki 75 akan diterapkan runtime-only. Sistem memberi 3 pulse pada Iq terpilih, menyimpan evidence, lalu kembali RUNTIME.',()=>firdaPiAutoExecute());
  $('firdaPiStop').onclick=async()=>{firdaPiAbort=true;try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}firdaKpSetText('firdaPiState','STOP CURRENT • membatalkan validasi…')};
}
firdaPiBind();
setInterval(()=>{if(firdaPiActive())renderFirdaPiQuickPanel()},250);

// FIRDA 4.3.1 one-page position Kp tuning. Current PI is fixed from 4.2.
let firdaPosKpCycleCount=0,firdaPosKpAutoBusy=false,firdaPosKpAbort=false;
function firdaPosKpActive(){return currentExp==='steering'&&String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)==='4.3.3'}
function firdaPosKpMeta(v){
  if(Math.abs(v-.0125)<.003)return['POSKP-0125','candidate-a'];
  if(Math.abs(v-.0250)<.004)return['POSKP-0250','candidate-b'];
  if(Math.abs(v-.0375)<.005)return['POSKP-0375','candidate-c'];
  return['POSKP-'+Number(v).toFixed(4),'candidate-a'];
}
function firdaPosKpApplyIdentity(v){
  const [variation,candidate]=firdaPosKpMeta(v);
  if($('runVariation'))$('runVariation').value=variation;
  if($('runCondition'))$('runCondition').value='STEP_PM5DEG_KI0_KD0';
  if($('runCandidate'))$('runCandidate').value=candidate;
  if(selectedExperiment){localRunValues[`${currentExp}:${selectedExperiment.id}:test_steering_deg`]=5;localRunValues[`${currentExp}:${selectedExperiment.id}:gain_variant`]=variation;}
}
function firdaPosPhysicalFromVesc(v){return Number.isFinite(+v)?((+v-180)/6):NaN}
function renderFirdaPosKpQuickPanel(){
  const p=$('firdaPosKpQuickPanel');if(!p)return;
  const active=firdaPosKpActive();p.hidden=!active;if(!active)return;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$('firdaPosKpModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaPosActual'))$('firdaPosActual').value=t?(fmt(t.pos_kp,6)+' / '+fmt(t.pos_ki,6)+' / '+fmt(t.pos_kd,6)):'-- / -- / --';
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN;
  firdaKpSetText('firdaPosKpLive','Target '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • Iq '+fmt(lv.iq_a,2)+' A');
  firdaKpSetText('firdaPosKpCycleCount','Cycle: '+firdaPosKpCycleCount+' / 3');
  if($('firdaPosKpAuto')){$('firdaPosKpAuto').disabled=rec||firdaPosKpAutoBusy;$('firdaPosKpAuto').textContent=firdaPosKpAutoBusy?'TEST POSISI BERJALAN…':'START TEST OTOMATIS • ±5° • 3 CYCLE'}
}
async function firdaPosPoll(durationMs,intervalMs=70){
  const until=Date.now()+Math.max(0,durationMs);
  while(Date.now()<until){
    if(firdaPosKpAbort)throw new Error('Test posisi dibatalkan');
    await firdaKpCommand('VALUES:1',5000);
    const lv=obj('vesc_left_values');
    if(Number.isFinite(+lv.fault)&&+lv.fault!==0)throw new Error('Fault LEFT '+lv.fault);
    await sleep(intervalMs);
  }
}
async function firdaPosWaitTarget(vescTarget,tolVescDeg=12,timeoutMs=4500){
  const t0=Date.now();let stable=0;
  while(Date.now()-t0<timeoutMs){
    if(firdaPosKpAbort)throw new Error('Test posisi dibatalkan');
    await firdaKpCommand('VALUES:1',5000);await sleep(70);
    const lv=obj('vesc_left_values'),p=+lv.position_deg;
    if(Number.isFinite(+lv.fault)&&+lv.fault!==0)throw new Error('Fault LEFT '+lv.fault);
    if(Number.isFinite(p)&&Math.abs(p-vescTarget)<=tolVescDeg){stable++;if(stable>=3)return true}else stable=0;
  }
  return false;
}
async function firdaPosApplyTuning(posKp){
  await firdaKpCommand('TUNING:GET:1');
  if(!await firdaKpWait(()=>!!firdaKpTune(),2500))throw new Error('Read LEFT tuning gagal');
  const t=firdaKpTune(),keys=['foc_d_kp','foc_d_ki','speed_kp','speed_ki','speed_kd','current_filter'];
  if(!keys.every(k=>Number.isFinite(+t[k])))throw new Error('Readback tuning tidak lengkap');
  const vals=[FIRDA_Q_KP_FINAL,FIRDA_Q_KI_FINAL,+t.foc_d_kp,+t.foc_d_ki,+t.speed_kp,+t.speed_ki,+t.speed_kd,posKp,0,0,+t.current_filter];
  await firdaKpCommand('TUNING:SET:1:'+vals.join(':')+':0');
  await sleep(250);await firdaKpCommand('TUNING:GET:1');await sleep(250);
  const rb=firdaKpTune();
  if(!rb||Math.abs(+rb.foc_q_kp-FIRDA_Q_KP_FINAL)>.01||Math.abs(+rb.foc_q_ki-FIRDA_Q_KI_FINAL)>.5)throw new Error('Gain FOC final readback tidak sesuai');
  if(Math.abs(+rb.pos_kp-posKp)>.001||Math.abs(+rb.pos_ki)>.001||Math.abs(+rb.pos_kd)>.001)throw new Error('Position PID readback tidak sesuai');
  return rb;
}
async function firdaPosKpAutoExecute(){
  if(firdaPosKpAutoBusy)return;
  const kp=+$('firdaPosKpValue').value;
  if(!Number.isFinite(kp)||kp<=0||kp>1)return toast('Position Kp candidate tidak valid.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaPosKpAutoBusy=true;firdaPosKpAbort=false;firdaPosKpCycleCount=0;firdaPosKpApplyIdentity(kp);if($('runSampleRate'))$('runSampleRate').value='50';renderFirdaPosKpQuickPanel();
  try{
    firdaKpSetText('firdaPosKpState','Menyiapkan Web Maintenance dan center…');
    if(!bool(raw('vesc_maintenance_active'))){
      await firdaKpCommand('MODE:MAINTENANCE');
      if(!await firdaKpWait(()=>bool(raw('vesc_maintenance_active')),5000))throw new Error('Maintenance tidak aktif');
    }
    // Setup-only centering uses the previously proven runtime P gain and is not recorded.
    await firdaPosApplyPID(0.05,0.005,0);
    await firdaKpCommand('SET:POS:1:180');
    if(!await firdaPosWaitTarget(180,9,7000))throw new Error('Steering belum mencapai CENTER ±1.5° sebelum test');
    firdaKpSetText('firdaPosKpState','CENTER siap • apply Position Kp '+kp.toFixed(4)+' runtime-only…');
    const rb=await firdaPosApplyTuning(kp);
    firdaKpSetText('firdaPosKpState','Verified Pos Kp '+(+rb.pos_kp).toFixed(4)+' • Ki 0 • Kd 0');
    await firdaKpCommand('SET:POS:1:180');await firdaPosPoll(700,70);
    await startWebRecording();
    if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    await firdaPosPoll(700,70);
    const moves=[[210,'+5°'],[180,'0°'],[150,'−5°'],[180,'0°']];
    for(let c=0;c<3;c++){
      for(const [target,label] of moves){
        firdaKpSetText('firdaPosKpState','Cycle '+(c+1)+'/3 • target '+label);
        await firdaKpCommand('SET:POS:1:'+target);
        await firdaPosPoll(target===180?1200:1800,70);
      }
      firdaPosKpCycleCount=c+1;renderFirdaPosKpQuickPanel();
    }
    await firdaKpCommand('SET:POS:1:180');await firdaPosPoll(1200,70);
    firdaKpSetText('firdaPosKpState','Menyimpan CSV / XLSX / PNG…');
    const saved=await stopWebRecording();if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');
    await sleep(300);await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpCommand('MODE:RUNTIME',15000);
    await firdaKpWait(()=>!bool(raw('vesc_maintenance_active')),5000);
    firdaKpSetText('firdaPosKpState','SELESAI • '+firdaPosKpMeta(kp)[0]+' • 3 cycle ±5° tersimpan • kembali RUNTIME');
    toast('4.3.1 '+firdaPosKpMeta(kp)[0]+' selesai dan tersimpan',false);
  }catch(e){
    try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}
    try{if($('recordToggleBtn')?.dataset.active==='true')await stopWebRecording()}catch(_){}
    try{if(bool(raw('vesc_maintenance_active')))await firdaKpCommand('MODE:RUNTIME',15000)}catch(_){}
    firdaKpSetText('firdaPosKpState','TEST STOP: '+(e?.message||e));
    toast('Tuning Kp posisi berhenti: '+(e?.message||e),true);
  }finally{firdaPosKpAutoBusy=false;renderFirdaPosKpQuickPanel()}
}
function firdaPosKpBind(){
  const p=$('firdaPosKpQuickPanel');if(!p||p.dataset.bound==='1')return;p.dataset.bound='1';
  const preset=(id,v)=>{$(id).onclick=()=>{if($('firdaPosKpValue'))$('firdaPosKpValue').value=String(v);firdaPosKpApplyIdentity(v);firdaPosKpCycleCount=0;renderFirdaPosKpQuickPanel()}};
  preset('firdaPosKpPresetLow',.0125);preset('firdaPosKpPresetMid',.0250);preset('firdaPosKpPresetHigh',.0375);
  $('firdaPosKpValue').onchange=()=>{const v=+$('firdaPosKpValue').value;if(Number.isFinite(v)){firdaPosKpApplyIdentity(v);firdaPosKpCycleCount=0}renderFirdaPosKpQuickPanel()};
  $('firdaPosKpAuto').onclick=()=>confirmModal('START TEST Kp POSISI','Roda steering wajib terangkat dan area gerak aman. Sistem akan center terlebih dahulu, lalu menguji step fisik +5° / 0° / −5° / 0° sebanyak 3 cycle dengan current PI final. STOP POSITION bila gerakan kasar atau tidak normal.',()=>firdaPosKpAutoExecute());
  $('firdaPosKpStop').onclick=async()=>{firdaPosKpAbort=true;try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}firdaKpSetText('firdaPosKpState','STOP POSITION • test dibatalkan…')};
}
firdaPosKpBind();
setInterval(()=>{if(firdaPosKpActive())renderFirdaPosKpQuickPanel()},250);

// FIRDA 4.3.2–4.3.4 shared position-loop runner.
const firdaPosAdv={busy:false,abort:false,cycles:0};
function firdaPosStep(){return String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)}
function firdaPosPanelCfg(){
  const s=firdaPosStep();
  if(s==='4.3.2')return {panel:'firdaZnBaselinePanel',chip:'firdaZnBaselineModeState',state:'firdaZnBaselineState',live:'firdaZnBaselineLive',count:'firdaZnBaselineCount',actual:'firdaZnBaseActual',auto:'firdaZnBaselineAuto',cycles:3};
  if(s==='4.3.4')return {panel:'firdaPosKiQuickPanel',chip:'firdaPosKiModeState',state:'firdaPosKiState',live:'firdaPosKiLive',count:'firdaPosKiCycleCount',actual:'firdaPosKiActual',auto:'firdaPosKiAuto',cycles:3};
  if(s==='4.3.5')return {panel:'firdaPosKdQuickPanel',chip:'firdaPosKdModeState',state:'firdaPosKdState',live:'firdaPosKdLive',count:'firdaPosKdCycleCount',actual:'firdaPosKdActual',auto:'firdaPosKdAuto',cycles:3};
  if(s==='4.3.6')return {panel:'firdaPosFinalQuickPanel',chip:'firdaPosFinalModeState',state:'firdaPosFinalState',live:'firdaPosFinalLive',count:'firdaPosFinalCycleCount',actual:'firdaPosFinalActual',auto:'firdaPosFinalAuto',cycles:5};
  return null;
}
function renderFirdaPosAdvancedPanels(){
  for(const id of ['firdaPosKiQuickPanel','firdaPosKdQuickPanel','firdaPosFinalQuickPanel'])if($(id))$(id).hidden=true;
  const c=firdaPosPanelCfg();if(!c)return;
  const p=$(c.panel);if(!p)return;p.hidden=false;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$(c.chip);if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($(c.actual))$(c.actual).value=t?(fmt(t.pos_kp,6)+' / '+fmt(t.pos_ki,6)+' / '+fmt(t.pos_kd,6)):'-- / -- / --';
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN;
  firdaKpSetText(c.live,'Target '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • Iq '+fmt(lv.iq_a,2)+' A');
  firdaKpSetText(c.count,'Cycle: '+firdaPosAdv.cycles+' / '+c.cycles);
  if($(c.auto)){$(c.auto).disabled=rec||firdaPosAdv.busy;$(c.auto).textContent=firdaPosAdv.busy?'TEST POSISI BERJALAN…':(firdaPosStep()==='4.3.4'?'START VALIDASI FINAL • ±5° • 5 CYCLE':'START TEST OTOMATIS • ±5° • 3 CYCLE')}
}
async function firdaPosApplyPID(kp,ki,kd){
  await firdaKpCommand('TUNING:GET:1');
  if(!await firdaKpWait(()=>!!firdaKpTune(),2500))throw new Error('Read LEFT tuning gagal');
  const t=firdaKpTune(),keys=['foc_d_kp','foc_d_ki','speed_kp','speed_ki','speed_kd','current_filter'];
  if(!keys.every(k=>Number.isFinite(+t[k])))throw new Error('Readback tuning tidak lengkap');
  const vals=[FIRDA_Q_KP_FINAL,FIRDA_Q_KI_FINAL,+t.foc_d_kp,+t.foc_d_ki,+t.speed_kp,+t.speed_ki,+t.speed_kd,kp,ki,kd,+t.current_filter];
  await firdaKpCommand('TUNING:SET:1:'+vals.join(':')+':0');
  await sleep(250);await firdaKpCommand('TUNING:GET:1');await sleep(250);
  const rb=firdaKpTune();
  if(!rb||Math.abs(+rb.foc_q_kp-FIRDA_Q_KP_FINAL)>.01||Math.abs(+rb.foc_q_ki-FIRDA_Q_KI_FINAL)>.5)throw new Error('Gain FOC final readback tidak sesuai');
  if(Math.abs(+rb.pos_kp-kp)>.001||Math.abs(+rb.pos_ki-ki)>.001||Math.abs(+rb.pos_kd-kd)>.001)throw new Error('Position PID readback tidak sesuai');
  return rb;
}
function firdaPosAdvParams(){
  const s=firdaPosStep();
  if(s==='4.3.2'){const kp=+$('firdaZnBaseKp').value,ki=+$('firdaZnBaseKi').value,kd=+$('firdaZnBaseKd').value;return {kp,ki,kd,variation:'ZN-BASELINE',candidate:'baseline',cycles:3};}
  if(s==='4.3.4'){const kpRaw=+$('firdaPosKiFixedKp').value,kiRaw=+$('firdaPosKiValue').value;const kp=Number.isFinite(kpRaw)&&kpRaw>0?kpRaw:.0375,ki=Number.isFinite(kiRaw)&&kiRaw>=0?kiRaw:0;const stage=$('firdaPosKiStage')?.value||'coarse',f=+$('firdaPosKiFactor').value;return {kp,ki,kd:0,variation:'POSKI-'+stage.toUpperCase()+'-X'+(Number.isFinite(f)?f.toFixed(3):'MANUAL')+'-KI'+ki.toFixed(6),candidate:stage==='fine'?'candidate-b':'candidate-a',cycles:3};}
  if(s==='4.3.5'){const kpRaw=+$('firdaPosKdFixedKp').value,kiRaw=+$('firdaPosKdFixedKi').value,kdRaw=+$('firdaPosKdValue').value;const kp=Number.isFinite(kpRaw)&&kpRaw>0?kpRaw:.0375,ki=Number.isFinite(kiRaw)&&kiRaw>=0&&kiRaw>0?kiRaw:.0025,kd=Number.isFinite(kdRaw)&&kdRaw>=0?kdRaw:0;const stage=$('firdaPosKdStage')?.value||'coarse',f=+$('firdaPosKdFactor').value;return {kp,ki,kd,variation:'POSKD-'+stage.toUpperCase()+'-X'+(Number.isFinite(f)?f.toFixed(3):'MANUAL')+'-KD'+kd.toFixed(6),candidate:stage==='fine'?'candidate-b':'candidate-a',cycles:3};}
  if(s==='4.3.6'){const kp=+$('firdaPosFinalKp').value,ki=+$('firdaPosFinalKi').value,kd=+$('firdaPosFinalKd').value;return {kp,ki,kd,variation:'POS-FINAL-KP'+kp.toFixed(6)+'-KI'+ki.toFixed(6)+'-KD'+kd.toFixed(6),candidate:'final',cycles:5};}
  return null;
}
async function firdaPosAdvRun(){
  if(firdaPosAdv.busy)return;
  const c=firdaPosPanelCfg(),p=firdaPosAdvParams();if(!c||!p)return;
  if(firdaPosStep()==='4.3.4'){p.kp=(Number.isFinite(+p.kp)&&+p.kp>0)?+p.kp:0.0375;p.ki=(Number.isFinite(+p.ki)&&+p.ki>=0)?+p.ki:0;p.kd=0;}
  if(firdaPosStep()==='4.3.5'){p.kp=(Number.isFinite(+p.kp)&&+p.kp>0)?+p.kp:0.0375;p.ki=(Number.isFinite(+p.ki)&&+p.ki>0)?+p.ki:0.0025;p.kd=(Number.isFinite(+p.kd)&&+p.kd>=0)?+p.kd:0;}
  if(![p.kp,p.ki,p.kd].every(Number.isFinite)||p.kp<=0||p.ki<0||p.kd<0)return toast('Gain posisi tidak valid: Kp='+p.kp+' Ki='+p.ki+' Kd='+p.kd,true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaPosAdv.busy=true;firdaPosAdv.abort=false;firdaPosAdv.cycles=0;
  if($('runVariation'))$('runVariation').value=p.variation;
  if($('runCondition'))$('runCondition').value='STEP_PM5DEG';
  if($('runCandidate'))$('runCandidate').value=p.candidate;
  if($('runSampleRate'))$('runSampleRate').value='50';
  if(selectedExperiment){localRunValues[`${currentExp}:${selectedExperiment.id}:test_steering_deg`]=5;localRunValues[`${currentExp}:${selectedExperiment.id}:gain_variant`]=`KP${p.kp.toFixed(4)}_KI${p.ki.toFixed(4)}_KD${p.kd.toFixed(4)}`;}
  renderTuningFields();renderFirdaPosAdvancedPanels();
  try{
    firdaKpSetText(c.state,'Menyiapkan center dan gain posisi…');
    if(!bool(raw('vesc_maintenance_active'))){await firdaKpCommand('MODE:MAINTENANCE');if(!await firdaKpWait(()=>bool(raw('vesc_maintenance_active')),5000))throw new Error('Maintenance tidak aktif')}
    await firdaPosApplyPID(.05,.005,0);await firdaKpCommand('SET:POS:1:180');
    if(!await firdaPosWaitTarget(180,9,7000))throw new Error('Steering belum mencapai CENTER ±1.5° sebelum test');
    const rb=await firdaPosApplyPID(p.kp,p.ki,p.kd);
    firdaKpSetText(c.state,'Verified Kp '+(+rb.pos_kp).toFixed(4)+' • Ki '+(+rb.pos_ki).toFixed(4)+' • Kd '+(+rb.pos_kd).toFixed(4));
    await firdaKpCommand('SET:POS:1:180');await firdaPosPoll(700,70);
    await startWebRecording();if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    await firdaPosPoll(700,70);
    const moves=[[210,'+5°'],[180,'0°'],[150,'−5°'],[180,'0°']];
    for(let n=0;n<p.cycles;n++){
      for(const [target,label] of moves){if(firdaPosAdv.abort)throw new Error('Test posisi dibatalkan');firdaKpSetText(c.state,'Cycle '+(n+1)+'/'+p.cycles+' • target '+label);await firdaKpCommand('SET:POS:1:'+target);await firdaPosPoll(target===180?1200:1800,70)}
      firdaPosAdv.cycles=n+1;renderFirdaPosAdvancedPanels();
    }
    await firdaKpCommand('SET:POS:1:180');await firdaPosPoll(1200,70);
    firdaKpSetText(c.state,'Menyimpan CSV / XLSX / PNG…');const saved=await stopWebRecording();if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');
    await sleep(300);await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpCommand('MODE:RUNTIME',15000);await firdaKpWait(()=>!bool(raw('vesc_maintenance_active')),5000);
    if(firdaPosStep()==='4.3.6')firdaPersistFinalGain(p.kp,p.ki,p.kd);firdaKpSetText(c.state,'SELESAI • '+p.variation+' • '+p.cycles+' cycle tersimpan • kembali RUNTIME');toast(firdaPosStep()+' '+p.variation+' selesai',false);
  }catch(e){
    try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}
    try{if($('recordToggleBtn')?.dataset.active==='true')await stopWebRecording()}catch(_){}
    try{if(bool(raw('vesc_maintenance_active')))await firdaKpCommand('MODE:RUNTIME',15000)}catch(_){}
    firdaKpSetText(c.state,'TEST STOP: '+(e?.message||e));toast('Tuning posisi berhenti: '+(e?.message||e),true);
  }finally{firdaPosAdv.busy=false;renderFirdaPosAdvancedPanels()}
}
function firdaPosAdvancedBind(){
  const bindPreset=(id,input,v)=>{if($(id))$(id).onclick=()=>{$(input).value=String(v);firdaPosAdv.cycles=0;renderFirdaPosAdvancedPanels()}};
  bindPreset('firdaPosKiPresetLow','firdaPosKiValue',.0025);bindPreset('firdaPosKiPresetMid','firdaPosKiValue',.005);bindPreset('firdaPosKiPresetHigh','firdaPosKiValue',.01);
  bindPreset('firdaPosKdPreset0','firdaPosKdValue',0);bindPreset('firdaPosKdPreset2','firdaPosKdValue',.002);bindPreset('firdaPosKdPreset4','firdaPosKdValue',.004);
  for(const id of ['firdaPosKiAuto','firdaPosKdAuto','firdaPosFinalAuto'])if($(id))$(id).onclick=()=>confirmModal('START TEST POSISI','Roda steering wajib terangkat dan area gerak aman. Sistem akan center, apply gain runtime-only, menjalankan step ±5°, menyimpan data, lalu kembali RUNTIME.',()=>firdaPosAdvRun());
  for(const id of ['firdaPosKiStop','firdaPosKdStop','firdaPosFinalStop'])if($(id))$(id).onclick=async()=>{firdaPosAdv.abort=true;try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}const c=firdaPosPanelCfg();if(c)firdaKpSetText(c.state,'STOP POSITION • test dibatalkan…')};
}
firdaPosAdvancedBind();
setInterval(()=>{if(/^4\.3\.[234]$/.test(firdaPosStep()))renderFirdaPosAdvancedPanels()},250);

// FIRDA 4.4 response-validation runner with locked final gains.
const FIRDA_POS_FINAL={kp:.0375,ki:.0025,kd:0};
try{const fg=JSON.parse(localStorage.getItem('firda-steering-final-gain-r3')||'null');if(fg&&[fg.kp,fg.ki,fg.kd].every(Number.isFinite)){FIRDA_POS_FINAL.kp=+fg.kp;FIRDA_POS_FINAL.ki=+fg.ki;FIRDA_POS_FINAL.kd=+fg.kd}}catch(_){}
const firdaResp={busy:false,abort:false,cycles:0};
function firdaResponseCfg(){
  const s=firdaPosStep();
  if(s==='4.4.1')return {title:'4.4.1 Tracking Setpoint Absolut',variation:'TRACK-ABS-25-15-0-15-25',condition:'FINAL_PID',cycles:3,moves:[[30,'−25°'],[90,'−15°'],[180,'0°'],[270,'+15°'],[330,'+25°'],[180,'0°']],hint:'Tracking posisi absolut −25° / −15° / 0° / +15° / +25° sebanyak 3 cycle.'};
  if(s==='4.4.2')return {title:'4.4.2 Step ±10°',variation:'STEP-PM10',condition:'FINAL_PID',cycles:3,moves:[[240,'+10°'],[180,'0°'],[120,'−10°'],[180,'0°']],hint:'Step +10° / 0° / −10° / 0° sebanyak 3 cycle.'};
  if(s==='4.4.3')return {title:'4.4.3 Step ±20°',variation:'STEP-PM20',condition:'FINAL_PID',cycles:3,moves:[[300,'+20°'],[180,'0°'],[60,'−20°'],[180,'0°']],hint:'Step +20° / 0° / −20° / 0° sebanyak 3 cycle.'};
  if(s==='4.4.4')return {title:'4.4.4 Repeatability',variation:'REPEAT-PM20',condition:'FINAL_PID',cycles:5,moves:[[300,'+20°'],[180,'0°'],[60,'−20°'],[180,'0°']],hint:'Repeatability LEFT → CENTER → RIGHT → CENTER, ±20°, sebanyak 5 cycle.'};
  if(s==='4.4.5')return {title:'4.4.5 Simetri Kiri / Kanan',variation:'SYMMETRY-10-20-25',condition:'FINAL_PID',cycles:3,moves:[[240,'+10°'],[180,'0°'],[120,'−10°'],[180,'0°'],[300,'+20°'],[180,'0°'],[60,'−20°'],[180,'0°'],[330,'+25°'],[180,'0°'],[30,'−25°'],[180,'0°']],hint:'Bandingkan pasangan ±10°, ±20°, dan ±25° sebanyak 3 paired cycle.'};
  return null;
}
function renderFirdaResponsePanel(){
  const p=$('firdaResponseQuickPanel');if(!p)return;
  const c=firdaResponseCfg();p.hidden=!c;if(!c)return;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),rec=$('recordToggleBtn')?.dataset.active==='true';
  firdaKpSetText('firdaResponseEyebrow','FIRDA • '+firdaPosStep()+' VALIDASI RESPONS');
  firdaKpSetText('firdaResponseTitle',c.title+' • LEFT');
  firdaKpSetText('firdaResponseHint','Gain final dikunci: Position Kp 0.0375, Ki 0.0025, Kd 0; FOC Q Kp 0.80013, Q Ki 75. '+c.hint);
  const chip=$('firdaResponseModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaResponseActual'))$('firdaResponseActual').value=t?(fmt(t.pos_kp,6)+' / '+fmt(t.pos_ki,6)+' / '+fmt(t.pos_kd,6)):'-- / -- / --';
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN;
  firdaKpSetText('firdaResponseLive','Target '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • Iq '+fmt(lv.iq_a,2)+' A');
  firdaKpSetText('firdaResponseCount','Cycle: '+firdaResp.cycles+' / '+c.cycles);
  if($('firdaResponseAuto')){$('firdaResponseAuto').disabled=rec||firdaResp.busy;$('firdaResponseAuto').textContent=firdaResp.busy?'PENGUJIAN BERJALAN…':'START PENGUJIAN OTOMATIS • '+c.cycles+' CYCLE'}
}
async function firdaResponseWaitCommandTarget(target,timeoutMs=10000){
  const t0=Date.now();let stable=0;
  while(Date.now()-t0<timeoutMs){
    if(firdaResp.abort)throw new Error('Pengujian dibatalkan');
    await firdaKpCommand('VALUES:1',5000);await sleep(70);
    const lv=obj('vesc_left_values'),cmd=obj('vesc_command_state');
    if(Number.isFinite(+lv.fault)&&+lv.fault!==0)throw new Error('Fault LEFT '+lv.fault);
    const v=+cmd.value;
    if(String(cmd.mode||'')==='POS'&&Number.isFinite(v)&&Math.abs(v-target)<=2){stable++;if(stable>=3)return true}else stable=0;
  }
  return false;
}
async function firdaResponseRun(){
  if(firdaResp.busy)return;
  const c=firdaResponseCfg();if(!c)return;if(!firdaFinalGainValid())return toast('Gain final belum ditetapkan melalui 4.3.6.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaResp.busy=true;firdaResp.abort=false;firdaResp.cycles=0;
  if($('runVariation'))$('runVariation').value=c.variation;
  if($('runCondition'))$('runCondition').value=c.condition;
  if($('runCandidate'))$('runCandidate').value='final';
  if($('runSampleRate'))$('runSampleRate').value='50';
  if(selectedExperiment){
    const maxPhysical=Math.max(...c.moves.map(x=>Math.abs(firdaPosPhysicalFromVesc(x[0]))));
    localRunValues[`${currentExp}:${selectedExperiment.id}:repeat_count`]=c.cycles;
    localRunValues[`${currentExp}:${selectedExperiment.id}:test_steering_deg`]=maxPhysical;
    localRunValues[`${currentExp}:${selectedExperiment.id}:gain_variant`]='FINAL_PID';
  }
  renderTuningFields();renderFirdaResponsePanel();
  try{
    firdaKpSetText('firdaResponseState','Menyiapkan center dan gain final…');
    if(!bool(raw('vesc_maintenance_active'))){await firdaKpCommand('MODE:MAINTENANCE');if(!await firdaKpWait(()=>bool(raw('vesc_maintenance_active')),5000))throw new Error('Maintenance tidak aktif')}
    // Proven centering assist only before recording.
    await firdaPosApplyPID(.05,.005,0);await firdaKpCommand('SET:POS:1:180');
    if(!await firdaPosWaitTarget(180,9,7000))throw new Error('Steering belum mencapai CENTER ±1.5° sebelum test');
    const rb=await firdaPosApplyPID(FIRDA_POS_FINAL.kp,FIRDA_POS_FINAL.ki,FIRDA_POS_FINAL.kd);
    firdaKpSetText('firdaResponseState','Gain final verified • Kp '+(+rb.pos_kp).toFixed(4)+' • Ki '+(+rb.pos_ki).toFixed(4)+' • Kd '+(+rb.pos_kd).toFixed(4));
    await firdaKpCommand('SET:POS:1:180');await firdaPosPoll(800,70);
    await startWebRecording();if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    await firdaPosPoll(800,70);
    for(let n=0;n<c.cycles;n++){
      for(const [target,label] of c.moves){
        if(firdaResp.abort)throw new Error('Pengujian dibatalkan');
        firdaKpSetText('firdaResponseState','Cycle '+(n+1)+'/'+c.cycles+' • target '+label);
        await firdaKpCommand('SET:POS:1:'+target);
        const reached=await firdaResponseWaitCommandTarget(target,target===180?8000:12000);
        if(!reached)throw new Error('Command setpoint belum mencapai '+label+' sebelum timeout');
        firdaKpSetText('firdaResponseState','Cycle '+(n+1)+'/'+c.cycles+' • HOLD '+label+' • capture steady-state');
        await firdaPosPoll(target===180?1400:2000,70);
      }
      firdaResp.cycles=n+1;renderFirdaResponsePanel();
    }
    await firdaKpCommand('SET:POS:1:180');await firdaPosPoll(1400,70);
    firdaKpSetText('firdaResponseState','Menyimpan CSV / XLSX / PNG…');
    const saved=await stopWebRecording();if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');
    await sleep(300);await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpCommand('MODE:RUNTIME',15000);await firdaKpWait(()=>!bool(raw('vesc_maintenance_active')),5000);
    firdaKpSetText('firdaResponseState','SELESAI • '+c.variation+' • '+c.cycles+' cycle tersimpan • kembali RUNTIME');toast(firdaPosStep()+' selesai dan tersimpan',false);
  }catch(e){
    try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}
    try{if($('recordToggleBtn')?.dataset.active==='true')await stopWebRecording()}catch(_){}
    try{if(bool(raw('vesc_maintenance_active')))await firdaKpCommand('MODE:RUNTIME',15000)}catch(_){}
    firdaKpSetText('firdaResponseState','TEST STOP: '+(e?.message||e));toast('Validasi respons berhenti: '+(e?.message||e),true);
  }finally{firdaResp.busy=false;renderFirdaResponsePanel()}
}
function firdaResponseBind(){
  if($('firdaResponseAuto'))$('firdaResponseAuto').onclick=()=>confirmModal('START VALIDASI RESPONS','Roda steering wajib terangkat dan area gerak aman. Gain final dikunci. Sistem akan center, menjalankan skenario respons, menyimpan data, lalu kembali Runtime.',()=>firdaResponseRun());
  if($('firdaResponseStop'))$('firdaResponseStop').onclick=async()=>{firdaResp.abort=true;try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}firdaKpSetText('firdaResponseState','STOP POSITION • pengujian dibatalkan…')};
}
firdaResponseBind();
setInterval(()=>{if(/^4\.4\.[1-5]$/.test(firdaPosStep()))renderFirdaResponsePanel()},250);

// FIRDA 4.5 load test: wheels on ground, vehicle stationary, no joystick commands during recording.
const firdaLoad={busy:false,abort:false,cycles:0};
function firdaLoadCfg(){
  const s=firdaPosStep();if(s!=='4.5.1'&&s!=='4.5.2')return null;
  const n=Math.max(0,Math.min(3,parseInt($('firdaLoadLevel')?.value||'0',10)||0));
  return {galon:n,variation:(s==='4.5.2'?'TORQUE-':'LOAD-')+n+'GALON',condition:n+'_GALON_WHEELS_GROUND',cycles:3,moves:[[60,'−20°'],[180,'0°'],[300,'+20°'],[180,'0°']],torque:s==='4.5.2'};
}
function renderFirdaLoadPanel(){
  const p=$('firdaLoadQuickPanel');if(!p)return;
  const c=firdaLoadCfg();p.hidden=!c;if(!c)return;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),rec=$('recordToggleBtn')?.dataset.active==='true',ready=$('firdaLoadReady')?.checked===true;
  firdaKpSetText('firdaLoadEyebrow','FIRDA • '+firdaPosStep()+' UJI BEBAN STEERING');
  firdaKpSetText('firdaLoadTitle','Beban '+c.galon+' Galon • LEFT');
  firdaKpSetText('firdaLoadLevel',c.galon+' galon');
  firdaKpSetText('firdaLoadHint','Kondisi wajib: '+c.galon+' galon pada posisi chassis yang sama, roda depan menyentuh tanah, kendaraan diam, joystick netral/tidak digunakan. Gerakan otomatis: −20° → 0° → +20° → 0°, 3 cycle.');
  const chip=$('firdaLoadModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaLoadActual'))$('firdaLoadActual').value=t?(fmt(t.pos_kp,6)+' / '+fmt(t.pos_ki,6)+' / '+fmt(t.pos_kd,6)):'-- / -- / --';
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN;
  firdaKpSetText('firdaLoadLive','Target '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • Iq '+fmt(lv.iq_a,2)+' A • Imotor '+fmt(lv.current_motor_a,2)+' A');
  firdaKpSetText('firdaLoadCount','Cycle: '+firdaLoad.cycles+' / '+c.cycles);
  if($('firdaLoadAuto')){$('firdaLoadAuto').disabled=rec||firdaLoad.busy;$('firdaLoadAuto').textContent=firdaLoad.busy?'UJI BEBAN BERJALAN…':'START UJI BEBAN • −20° / 0° / +20° • 3 CYCLE'}
}
async function firdaLoadPoll(durationMs,intervalMs=70){
  const until=Date.now()+Math.max(0,durationMs);
  while(Date.now()<until){
    if(firdaLoad.abort)throw new Error('Uji beban dibatalkan');
    await firdaKpCommand('VALUES:1',5000);
    const lv=obj('vesc_left_values');
    if(Number.isFinite(+lv.fault)&&+lv.fault!==0)throw new Error('Fault LEFT '+lv.fault);
    await sleep(intervalMs);
  }
}
async function firdaLoadWaitCenter(timeoutMs=10000){
  const t0=Date.now();let stable=0;
  while(Date.now()-t0<timeoutMs){
    if(firdaLoad.abort)throw new Error('Uji beban dibatalkan');
    await firdaKpCommand('VALUES:1',5000);await sleep(80);
    const lv=obj('vesc_left_values'),p=+lv.position_deg;
    if(Number.isFinite(+lv.fault)&&+lv.fault!==0)throw new Error('Fault LEFT '+lv.fault);
    if(Number.isFinite(p)&&Math.abs(p-180)<=12){stable++;if(stable>=4)return true}else stable=0;
  }
  return false;
}
async function firdaLoadRun(){
  if(firdaLoad.busy)return;
  const c=firdaLoadCfg();if(!c)return;if(!firdaFinalGainValid())return toast('Gain final belum ditetapkan melalui 4.3.6.',true);
  if($('firdaLoadReady')?.checked!==true)return toast('Centang konfirmasi kondisi beban terlebih dahulu.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaLoad.busy=true;firdaLoad.abort=false;firdaLoad.cycles=0;
  if($('runVariation'))$('runVariation').value=c.variation;
  if($('runCondition'))$('runCondition').value=c.condition;
  if($('runCandidate'))$('runCandidate').value='final';
  if($('runSampleRate'))$('runSampleRate').value='50';
  renderFirdaLoadPanel();
  try{
    firdaKpSetText('firdaLoadState','Menyiapkan center dengan roda di tanah…');
    if(!bool(raw('vesc_maintenance_active'))){await firdaKpCommand('MODE:MAINTENANCE');if(!await firdaKpWait(()=>bool(raw('vesc_maintenance_active')),5000))throw new Error('Maintenance tidak aktif')}
    await firdaPosApplyPID(.05,.005,0);
    await firdaKpCommand('SET:POS:1:180');
    let centered=await firdaLoadWaitCenter(10000);
    let lv0=obj('vesc_left_values'),centerErr0=Number.isFinite(+lv0.position_deg)?Math.abs((+lv0.position_deg-180)/6):NaN;
    if(!centered && Number.isFinite(centerErr0) && centerErr0<=6.0){
      firdaKpSetText('firdaLoadState','CENTER setup masih '+centerErr0.toFixed(2)+'° dari nol • lanjut uji beban dan residual akan terekam pada target 0°');
    }else if(!centered){
      firdaKpSetText('firdaLoadState','CENTER setup berat • mencoba assist setup kedua…');
      await firdaPosApplyPID(.08,.01,0);await firdaKpCommand('SET:POS:1:180');
      centered=await firdaLoadWaitCenter(8000);
      lv0=obj('vesc_left_values');centerErr0=Number.isFinite(+lv0.position_deg)?Math.abs((+lv0.position_deg-180)/6):NaN;
      if(!centered && (!Number.isFinite(centerErr0)||centerErr0>6.0))throw new Error('Steering masih >±6° dari CENTER; recording belum dimulai agar data 4.5 tidak invalid');
    }
    const rb=await firdaPosApplyPID(FIRDA_POS_FINAL.kp,FIRDA_POS_FINAL.ki,FIRDA_POS_FINAL.kd);
    firdaKpSetText('firdaLoadState','Gain final verified • mulai recording beban '+c.galon+' galon');
    await firdaKpCommand('SET:POS:1:180');await firdaLoadPoll(1000,70);
    await startWebRecording();if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    await firdaLoadPoll(1000,70);
    for(let n=0;n<c.cycles;n++){
      for(const [target,label] of c.moves){
        if(firdaLoad.abort)throw new Error('Uji beban dibatalkan');
        firdaKpSetText('firdaLoadState','Beban '+c.galon+' galon • cycle '+(n+1)+'/'+c.cycles+' • target '+label);
        await firdaKpCommand('SET:POS:1:'+target);
        await firdaLoadPoll(target===180?2000:3000,70);
      }
      firdaLoad.cycles=n+1;renderFirdaLoadPanel();
    }
    await firdaKpCommand('SET:POS:1:180');await firdaLoadPoll(2000,70);
    firdaKpSetText('firdaLoadState','Menyimpan CSV / XLSX / PNG…');
    const saved=await stopWebRecording();if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');
    await sleep(300);await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpCommand('MODE:RUNTIME',15000);await firdaKpWait(()=>!bool(raw('vesc_maintenance_active')),5000);
    firdaKpSetText('firdaLoadState','SELESAI • '+c.galon+' galon • 3 cycle tersimpan • kembali RUNTIME');
    if($('firdaLoadReady'))$('firdaLoadReady').checked=false;
    toast(firdaPosStep()+' beban '+c.galon+' galon selesai dan tersimpan',false);
  }catch(e){
    try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}
    try{if($('recordToggleBtn')?.dataset.active==='true')await stopWebRecording()}catch(_){}
    try{if(bool(raw('vesc_maintenance_active')))await firdaKpCommand('MODE:RUNTIME',15000)}catch(_){}
    firdaKpSetText('firdaLoadState','TEST STOP: '+(e?.message||e));toast('Uji beban berhenti: '+(e?.message||e),true);
  }finally{firdaLoad.busy=false;renderFirdaLoadPanel()}
}
function firdaLoadBind(){
  if($('firdaLoadReady'))$('firdaLoadReady').onchange=()=>renderFirdaLoadPanel();
  if($('firdaLoadAuto'))$('firdaLoadAuto').onclick=()=>{const c=firdaLoadCfg();if(!c)return;if($('firdaLoadReady')?.checked!==true){toast('Centang konfirmasi kondisi beban dulu, lalu tekan START lagi.',true);$('firdaLoadReady')?.focus();return}confirmModal('START UJI BEBAN '+c.galon+' GALON','Pastikan tepat '+c.galon+' galon terpasang aman di posisi chassis yang sama, roda depan menyentuh tanah, kendaraan tidak bergerak maju/mundur, dan joystick tidak digunakan selama test.',()=>firdaLoadRun())};
  if($('firdaLoadStop'))$('firdaLoadStop').onclick=async()=>{firdaLoad.abort=true;try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}firdaKpSetText('firdaLoadState','STOP POSITION • uji beban dibatalkan…')};
}
firdaLoadBind();
setInterval(()=>{if(/^4\.5\.[1-4]$/.test(firdaPosStep()))renderFirdaLoadPanel()},250);

// FIRDA 4.6.1: read-only final parameter verification. No actuator motion, no YAML/EEPROM write.
const firdaFinalRecap={reads:(Array.isArray(window.FIRDA_461_SAVED_READS)&&window.FIRDA_461_SAVED_READS.length===3?window.FIRDA_461_SAVED_READS.map(x=>({...x})):[null,null,null]),busy:false,lastSavedPath:''};
function firdaFinalRecapActive(){return currentExp==='steering'&&firdaPosStep()==='4.6.1'}
function firdaFinalRecapRows(){
  return [
    {key:'foc_q_kp',label:'FOC Q Kp',ref:0.80013,tol:0.001,unit:'',note:'gain final 4.2'},
    {key:'foc_q_ki',label:'FOC Q Ki',ref:75,tol:0.20,unit:'',note:'gain final 4.2; fixed-point controller'},
    {key:'foc_d_kp',label:'FOC D Kp',ref:null,tol:0.001,unit:'',note:'dokumentasi + konsistensi'},
    {key:'foc_d_ki',label:'FOC D Ki',ref:null,tol:0.25,unit:'',note:'dokumentasi + konsistensi'},
    {key:'current_filter',label:'Current filter',ref:null,tol:0.0001,unit:'',note:'dokumentasi + konsistensi'},
    {key:'pos_kp',label:'Position Kp',ref:0.0375,tol:0.00051,unit:'',note:'readback controller resolusi 0.001; 0.0375 → 0.038'},
    {key:'pos_ki',label:'Position Ki',ref:0.0025,tol:0.00051,unit:'',note:'readback controller resolusi 0.001; 0.0025 → 0.003'},
    {key:'pos_kd',label:'Position Kd',ref:0,tol:0.00051,unit:'',note:'gain final 4.3'},
    {key:'current_limit_a',label:'Current limit',ref:12,tol:0.05,unit:'A',note:'limit aktif controller'},
    {key:'raw_center',label:'Encoder center TIM4',ref:2101,tol:0,unit:'count',note:'hasil center steering 4.1.3'},
    {key:'physical_left_deg',label:'Physical left limit',ref:-30,tol:0.01,unit:'deg',note:'konfigurasi aktif steering'},
    {key:'physical_right_deg',label:'Physical right limit',ref:30,tol:0.01,unit:'deg',note:'konfigurasi aktif steering'},
    {key:'operational_limit_deg',label:'Operational steering limit',ref:30,tol:0.01,unit:'deg',note:'konfigurasi aktif steering'},
    {key:'encoder_synced',label:'Encoder synced',ref:true,tol:0,unit:'bool',note:'status encoder'},
    {key:'fault',label:'LEFT fault',ref:0,tol:0,unit:'code',note:'harus 0'},
    {key:'store_requested',label:'EEPROM store requested',ref:false,tol:0,unit:'bool',note:'harus false; verifikasi read-only/runtime'}
  ];
}
function firdaFinalFmt(v,key){
  if(v===undefined||v===null||Number.isNaN(v))return'--';
  if(typeof v==='boolean')return v?'TRUE':'FALSE';
  if(key==='raw_center'||key==='fault')return String(Math.round(+v));
  if(['foc_q_kp','foc_d_kp','current_filter','pos_kp','pos_ki','pos_kd'].includes(key))return Number(v).toFixed(6);
  if(['foc_q_ki','foc_d_ki'].includes(key))return Number(v).toFixed(6);
  if(['physical_left_deg','physical_right_deg','operational_limit_deg','current_limit_a'].includes(key))return Number(v).toFixed(3);
  return String(v);
}
function firdaFinalEq(a,b,tol=0){
  if(typeof b==='boolean')return a===b;
  if(!Number.isFinite(+a)||!Number.isFinite(+b))return false;
  return Math.abs(+a-(+b))<=tol+1e-12;
}
function firdaFinalRowStatus(row){
  const vals=firdaFinalRecap.reads.map(r=>r?.[row.key]);
  if(vals.some(v=>v===undefined||v===null||Number.isNaN(v)))return'WAIT';
  if(row.ref!==null){return vals.every(v=>firdaFinalEq(v,row.ref,row.tol))?'PASS':'CHECK'}
  if(typeof vals[0]==='boolean')return vals.every(v=>v===vals[0])?'KONSISTEN':'CHECK';
  const nums=vals.map(Number);return nums.every(Number.isFinite)&&(Math.max(...nums)-Math.min(...nums)<=row.tol+1e-12)?'KONSISTEN':'CHECK';
}
function firdaFinalToggleLegacy(active){
  const nodes=[
    document.getElementById('trialGuidePanel'),document.getElementById('qualificationPanel'),document.getElementById('testPreflightPanel'),
    document.querySelector('.run-console'),document.querySelector('.graph-workspace'),document.querySelector('.lab-two-col'),
    document.querySelector('.table-workspace'),document.getElementById('trialRecapPanel'),document.querySelector('.audit-ribbon')
  ];
  nodes.forEach(n=>{if(n)n.hidden=active});
}
function renderFirdaFinalRecapPanel(){
  const p=$('firdaFinalRecapPanel');if(!p)return;
  const active=firdaFinalRecapActive();p.hidden=!active;firdaFinalToggleLegacy(active);if(!active)return;
  const body=$('firdaFinalRecapBody'),rows=firdaFinalRecapRows();
  const n=firdaFinalRecap.reads.filter(Boolean).length;if(n===3&&firdaFinalRecap.lastSavedPath===''&&$('firdaFinalRecapMessage')?.textContent?.includes('Kendaraan diam'))setText('firdaFinalRecapMessage','Data readback 3× yang sudah tersimpan dimuat kembali dari hasil 4.6.1 • siap Save Excel tanpa mengulang pengujian.');
  if(body)body.innerHTML=rows.map(row=>{
    const st=firdaFinalRowStatus(row),cls=st==='PASS'||st==='KONSISTEN'?'':'muted';
    const ref=row.ref===null?'konsistensi 3×':firdaFinalFmt(row.ref,row.key);
    return `<tr><td><b>${escapeHtml(row.label)}</b><br><small class="muted">${escapeHtml(row.note||'')}</small></td><td>${escapeHtml(ref)}</td>${firdaFinalRecap.reads.map(x=>`<td>${escapeHtml(firdaFinalFmt(x?.[row.key],row.key))}</td>`).join('')}<td>${escapeHtml(row.unit||'')}</td><td class="${cls}">${escapeHtml(st)}</td></tr>`;
  }).join('');
  const complete=n===3,allOk=complete&&rows.every(row=>['PASS','KONSISTEN'].includes(firdaFinalRowStatus(row)));
  const chip=$('firdaFinalRecapState');if(chip){chip.textContent=allOk?'PASS 3/3':complete?'CHECK 3/3':`WAIT ${n}/3`;chip.className='status-chip '+(allOk?'':complete?'bad':'waiting')}
  const latest=[...firdaFinalRecap.reads].reverse().find(Boolean),lv=obj('vesc_left_values');
  firdaKpSetText('firdaFinalRecapLive','LEFT fault '+firdaFinalFmt(latest?.fault??lv.fault,'fault')+' • encoder '+(latest?.encoder_synced===true?'SYNC':latest?.encoder_synced===false?'NOT SYNC':'--')+' • Vbus '+fmt(lv.vbus_v,2)+' V');
  if($('firdaFinalSave'))$('firdaFinalSave').disabled=!complete||firdaFinalRecap.busy;if($('firdaFinalSaveExcel'))$('firdaFinalSaveExcel').disabled=!complete||firdaFinalRecap.busy;
  for(let i=1;i<=3;i++){const b=$('firdaFinalRead'+i);if(b){const prevOk=i===1||!!firdaFinalRecap.reads[i-2];b.disabled=firdaFinalRecap.busy||!prevOk;b.textContent=firdaFinalRecap.reads[i-1]?`READBACK ${i} ✓`:`READBACK ${i}`}}
}
async function firdaFinalTakeReadback(index){
  if(!firdaFinalRecapActive()||firdaFinalRecap.busy)return;
  firdaFinalRecap.busy=true;renderFirdaFinalRecapPanel();
  firdaKpSetText('firdaFinalRecapMessage',`Membaca controller untuk READBACK ${index+1}… tidak ada command gerak.`);
  try{
    await loadConfig();
    await firdaKpCommand('TUNING:GET:1',5000);await sleep(280);
    await firdaKpCommand('STEERING:CAL',5000);await sleep(280);
    await firdaKpCommand('VALUES:1',5000);await sleep(250);
    const t=firdaKpTune(),s=obj('vesc_steering_state'),f=obj('foc_telemetry'),lv=obj('vesc_left_values'),cal=f?.steering_cal||{};
    if(!t)throw new Error('Tuning LEFT belum terbaca valid');
    if(Number.isFinite(+lv.fault)&&+lv.fault!==0)throw new Error('LEFT fault '+lv.fault+' aktif');
    const snap={
      captured_at:new Date().toISOString(),foc_q_kp:+t.foc_q_kp,foc_q_ki:+t.foc_q_ki,foc_d_kp:+t.foc_d_kp,foc_d_ki:+t.foc_d_ki,
      current_filter:+t.current_filter,pos_kp:+t.pos_kp,pos_ki:+t.pos_ki,pos_kd:+t.pos_kd,current_limit_a:+t.current_limit_a,
      raw_center:Number.isFinite(+cal.raw_center)?+cal.raw_center:undefined,
      physical_left_deg:+configValue('esc','esc_ackermann.ros__parameters.steering_physical_left_limit_deg'),
      physical_right_deg:+configValue('esc','esc_ackermann.ros__parameters.steering_physical_right_limit_deg'),
      operational_limit_deg:+configValue('esc','esc_ackermann.ros__parameters.steering_physical_operational_limit_deg'),
      encoder_synced:s.encoder_synced===true||cal.encoder_synced===true,fault:Number.isFinite(+lv.fault)?+lv.fault:+s.fault,
      store_requested:t.store_requested===true
    };
    const required=['foc_q_kp','foc_q_ki','pos_kp','pos_ki','pos_kd','current_limit_a','raw_center','physical_left_deg','physical_right_deg','operational_limit_deg','fault'];
    if(required.some(k=>!Number.isFinite(+snap[k])))throw new Error('Readback belum lengkap: '+required.filter(k=>!Number.isFinite(+snap[k])).join(', '));
    firdaFinalRecap.reads[index]=snap;
    firdaKpSetText('firdaFinalRecapMessage',`READBACK ${index+1} tersimpan • ${new Date().toLocaleTimeString('id-ID')} • tidak ada parameter yang diubah.`);
    toast(`4.6.1 READBACK ${index+1} berhasil`,false);
  }catch(e){firdaKpSetText('firdaFinalRecapMessage','READBACK gagal: '+(e?.message||e));toast('4.6.1 readback gagal: '+(e?.message||e),true)}
  finally{firdaFinalRecap.busy=false;renderFirdaFinalRecapPanel()}
}
function firdaFinalCsv(){
  if(firdaFinalRecap.reads.filter(Boolean).length!==3)return'';
  const rows=[['Section','Parameter','Acuan final','Readback 1','Readback 2','Readback 3','Unit','Status','Catatan']];
  for(const row of firdaFinalRecapRows())rows.push(['4.6.1',row.label,row.ref===null?'konsistensi 3x':firdaFinalFmt(row.ref,row.key),...firdaFinalRecap.reads.map(x=>firdaFinalFmt(x?.[row.key],row.key)),row.unit||'',firdaFinalRowStatus(row),row.note||'']);
  rows.push([]);rows.push(['Captured at',...firdaFinalRecap.reads.map(x=>x?.captured_at||'')]);
  rows.push(['Method','READ-ONLY controller readback; no actuator motion; no YAML/EEPROM write']);
  return rows.map(r=>r.map(csvCell).join(',')).join('\n');
}
async function firdaFinalSaveRecap(){
  const csv=firdaFinalCsv();if(!csv)return toast('Lengkapi READBACK 1, 2, dan 3 terlebih dahulu.',true);
  if(firdaFinalRecap.busy)return;firdaFinalRecap.busy=true;renderFirdaFinalRecapPanel();
  try{
    const id='E4.6.1_FINAL_PARAMETER_READBACK';
    const r=await writeRequest('/api/experiment/table/save',{subsystem:'steering',id,label:id,source_experiment_id:'4.9',table_index:0,csv}),j=await r.json();
    if(!r.ok)throw new Error(j.message||`HTTP ${r.status}`);
    firdaFinalRecap.lastSavedPath=j.path||'';
    downloadBlob(`E4.6.1_FINAL_PARAMETER_READBACK_${stamp()}.csv`,'text/csv;charset=utf-8',csv);
    firdaKpSetText('firdaFinalRecapMessage','REKAP TERSIMPAN • '+(j.path||'data/esc')+' • CSV juga didownload.');
    toast('4.6.1 rekap final tersimpan',false);
  }catch(e){firdaKpSetText('firdaFinalRecapMessage','SAVE gagal: '+(e?.message||e));toast('Save rekap 4.6.1 gagal: '+(e?.message||e),true)}
  finally{firdaFinalRecap.busy=false;renderFirdaFinalRecapPanel()}
}
function firdaFinalExcelMatrix(){
  if(firdaFinalRecap.reads.filter(Boolean).length!==3)return null;
  const head=['Parameter','Acuan final','Readback 1','Readback 2','Readback 3','Unit','Status','Catatan'];
  const rows=firdaFinalRecapRows().map(row=>[row.label,row.ref===null?'konsistensi 3x':firdaFinalFmt(row.ref,row.key),...firdaFinalRecap.reads.map(x=>firdaFinalFmt(x?.[row.key],row.key)),row.unit||'',firdaFinalRowStatus(row),row.note||'']);
  rows.push([]);rows.push(['Captured at','',...firdaFinalRecap.reads.map(x=>x?.captured_at||''),'','','']);rows.push(['Method','READ-ONLY controller readback; no actuator motion; no YAML/EEPROM write','','','','','','']);return{head,rows};
}
function firdaFinalSaveExcel(){const m=firdaFinalExcelMatrix();if(!m)return toast('Lengkapi READBACK 1, 2, dan 3 terlebih dahulu.',true);if(typeof window.downloadSimpleXlsx!=='function')return toast('Exporter Excel belum termuat; Ctrl+Shift+R lalu coba lagi.',true);window.downloadSimpleXlsx(`E4.6.1_FINAL_PARAMETER_READBACK_${stamp()}.xlsx`,'4.6.1 Readback','4.6.1 Verifikasi Parameter Akhir FOC dan Position Controller',m.head,m.rows);toast('Excel 4.6.1 didownload',false)}
function firdaFinalRecapBind(){
  for(let i=1;i<=3;i++)if($('firdaFinalRead'+i))$('firdaFinalRead'+i).onclick=()=>firdaFinalTakeReadback(i-1);
  if($('firdaFinalSave'))$('firdaFinalSave').onclick=()=>firdaFinalSaveRecap();if($('firdaFinalSaveExcel'))$('firdaFinalSaveExcel').onclick=()=>firdaFinalSaveExcel();
  if($('firdaFinalReset'))$('firdaFinalReset').onclick=()=>{firdaFinalRecap.reads=[null,null,null];firdaFinalRecap.lastSavedPath='';firdaKpSetText('firdaFinalRecapMessage','Readback direset. Mulai lagi dari READBACK 1.');renderFirdaFinalRecapPanel()};
}
firdaFinalRecapBind();
setInterval(()=>{if(firdaFinalRecapActive())renderFirdaFinalRecapPanel()},500);

// FIRDA 4.6.2: read-only recap of the locked final 4.4 datasets.
const firdaPerformanceRecap={data:null,loading:false,error:''};
function firdaPerformanceActive(){return currentExp==='steering'&&firdaPosStep()==='4.6.2'}
function firdaPerformanceNum(v,d=3){return Number.isFinite(+v)?Number(v).toFixed(d):'--'}
async function firdaPerformanceLoad(force=false){
  if(firdaPerformanceRecap.loading)return;
  if(firdaPerformanceRecap.data&&!force){renderFirdaPerformanceRecapPanel();return}
  firdaPerformanceRecap.loading=true;firdaPerformanceRecap.error='';renderFirdaPerformanceRecapPanel();
  try{
    const j=window.FIRDA_462_RECAP;if(!j||j.section!=='4.6.2')throw new Error('Dataset rekap 4.6.2 belum termuat');
    firdaPerformanceRecap.data=JSON.parse(JSON.stringify(j));
  }catch(e){firdaPerformanceRecap.error=e?.message||String(e);toast('Rekap 4.6.2 gagal dimuat: '+firdaPerformanceRecap.error,true)}
  finally{firdaPerformanceRecap.loading=false;renderFirdaPerformanceRecapPanel()}
}
function renderFirdaPerformanceRecapPanel(){
  const p=$('firdaPerformanceRecapPanel');if(!p)return;
  const active=firdaPerformanceActive();p.hidden=!active;if(active)firdaFinalToggleLegacy(true);if(!active)return;
  const d=firdaPerformanceRecap.data,chip=$('firdaPerformanceRecapState');
  if(!d){
    if(chip){chip.textContent=firdaPerformanceRecap.error?'ERROR':'LOADING';chip.className='status-chip '+(firdaPerformanceRecap.error?'bad':'waiting')}
    if(!firdaPerformanceRecap.loading&&!firdaPerformanceRecap.error)setTimeout(()=>firdaPerformanceLoad(false),0);
    return;
  }
  if(chip){chip.textContent='DATA 4.4 COMPLETE';chip.className='status-chip'}
  const tr=Object.values(d.tracking||{}).flat(),rpt=d.repeatability_4_4_4||[],sym=d.symmetry_4_4_5||[],peaks=Object.values(d.peaks||{});
  const nonzero441=(d.tracking?.['4.4.1']||[]).filter(x=>Math.abs(+x.target_deg)>0);
  const maxMae=nonzero441.length?Math.max(...nonzero441.map(x=>+x.mae_endpoint_deg)):NaN;
  const worstSd=rpt.length?Math.max(...rpt.map(x=>+x.sd_repeat_deg)):NaN;
  const symMax=sym.length?Math.max(...sym.map(x=>+x.magnitude_difference_deg)):NaN;
  const peakIq=peaks.length?Math.max(...peaks.map(x=>+x.peak_abs_iq_a)):NaN;
  setText('firdaPerfMaxMae',firdaPerformanceNum(maxMae,3));setText('firdaPerfWorstSd',firdaPerformanceNum(worstSd,3));setText('firdaPerfSymDiff',firdaPerformanceNum(symMax,3));setText('firdaPerfPeakIq',firdaPerformanceNum(peakIq,2));
  const tb=$('firdaPerformanceTrackingBody');if(tb)tb.innerHTML=['4.4.1','4.4.2','4.4.3'].flatMap(sec=>(d.tracking?.[sec]||[]).map(x=>`<tr><td>${sec}</td><td>${firdaPerformanceNum(x.target_deg,0)}°</td><td>${firdaPerformanceNum(x.actual_mean_deg)}°</td><td>${firdaPerformanceNum(x.signed_error_deg)}°</td><td>${firdaPerformanceNum(x.mae_endpoint_deg)}°</td><td>${firdaPerformanceNum(x.sd_repeat_deg)}°</td><td>${x.repeats}</td></tr>`)).join('');
  const rb=$('firdaPerformanceRepeatBody');if(rb)rb.innerHTML=rpt.map(x=>`<tr><td>${firdaPerformanceNum(x.target_deg,0)}°</td><td>${firdaPerformanceNum(x.actual_mean_deg)}°</td><td>${firdaPerformanceNum(x.mae_endpoint_deg)}°</td><td>${firdaPerformanceNum(x.sd_repeat_deg)}°</td><td>${firdaPerformanceNum(x.range_repeat_deg)}°</td><td>${x.repeats}</td></tr>`).join('');
  const sb=$('firdaPerformanceSymBody');if(sb)sb.innerHTML=sym.map(x=>`<tr><td>±${firdaPerformanceNum(x.pair_deg,0)}°</td><td>${firdaPerformanceNum(x.left_actual_deg)}°</td><td>${firdaPerformanceNum(x.right_actual_deg)}°</td><td>${firdaPerformanceNum(x.magnitude_difference_deg)}°</td><td>${firdaPerformanceNum(x.left_sd_deg)}°</td><td>${firdaPerformanceNum(x.right_sd_deg)}°</td></tr>`).join('');
  const src=Object.entries(d.sources||{}).map(([k,v])=>`${k}: ${v}`).join(' • ');setText('firdaPerformanceSources','Source final terkunci • '+src+' • 4.5 belum termasuk (PENDING INDOOR).');
  setText('firdaPerformanceMessage','Rekap dihitung dari raw 50 Hz final 4.4 • steady-state = 0.7 s terakhir tiap plateau • tidak ada data dummy.');
  setText('firdaPerformanceLoadStatus','4.5: PENDING INDOOR • tidak dimasukkan ke 4.6.2 saat ini');
  if($('firdaPerformanceSave'))$('firdaPerformanceSave').disabled=false;if($('firdaPerformanceSaveExcel'))$('firdaPerformanceSaveExcel').disabled=false;
}
function firdaPerformanceCsv(){
  const d=firdaPerformanceRecap.data;if(!d)return'';const rows=[['Section','Test','Target/Pair [deg]','Actual Mean [deg]','Signed Error [deg]','Endpoint MAE [deg]','Repeat SD [deg]','Repeat Range [deg]','N Repeat','Source']];
  for(const sec of ['4.4.1','4.4.2','4.4.3'])for(const x of d.tracking?.[sec]||[])rows.push(['4.6.2',sec,x.target_deg,x.actual_mean_deg,x.signed_error_deg,x.mae_endpoint_deg,x.sd_repeat_deg,x.range_repeat_deg,x.repeats,d.sources?.[sec]||'']);
  for(const x of d.repeatability_4_4_4||[])rows.push(['4.6.2','4.4.4 Repeatability',x.target_deg,x.actual_mean_deg,x.signed_error_deg,x.mae_endpoint_deg,x.sd_repeat_deg,x.range_repeat_deg,x.repeats,d.sources?.['4.4.4']||'']);
  rows.push([]);rows.push(['Section','Test','Pair [deg]','Left Mean [deg]','Right Mean [deg]','Magnitude Difference [deg]','Left SD [deg]','Right SD [deg]','N each','Source']);
  for(const x of d.symmetry_4_4_5||[])rows.push(['4.6.2','4.4.5 Symmetry',x.pair_deg,x.left_actual_deg,x.right_actual_deg,x.magnitude_difference_deg,x.left_sd_deg,x.right_sd_deg,x.repeats_each,d.sources?.['4.4.5']||'']);
  rows.push([]);rows.push(['4.5 load test status','PENDING INDOOR - not included']);rows.push(['Method',d.method||'']);return rows.map(r=>r.map(csvCell).join(',')).join('\n');
}
function firdaPerformanceExcelMatrix(){
  const d=firdaPerformanceRecap.data;if(!d)return null;const head=['Test','Target/Pair [deg]','Actual Mean [deg]','Signed Error [deg]','Endpoint MAE [deg]','Repeat SD [deg]','Repeat Range [deg]','N Repeat','Source'];const rows=[];
  for(const sec of ['4.4.1','4.4.2','4.4.3'])for(const x of d.tracking?.[sec]||[])rows.push([sec,x.target_deg,x.actual_mean_deg,x.signed_error_deg,x.mae_endpoint_deg,x.sd_repeat_deg,x.range_repeat_deg,x.repeats,d.sources?.[sec]||'']);
  for(const x of d.repeatability_4_4_4||[])rows.push(['4.4.4 Repeatability',x.target_deg,x.actual_mean_deg,x.signed_error_deg,x.mae_endpoint_deg,x.sd_repeat_deg,x.range_repeat_deg,x.repeats,d.sources?.['4.4.4']||'']);
  for(const x of d.symmetry_4_4_5||[])rows.push(['4.4.5 Symmetry ±'+x.pair_deg,x.pair_deg,'L '+x.left_actual_deg+' / R '+x.right_actual_deg,'','','L '+x.left_sd_deg+' / R '+x.right_sd_deg,x.magnitude_difference_deg,x.repeats_each,d.sources?.['4.4.5']||'']);
  rows.push([]);rows.push(['4.5 Load Test','PENDING INDOOR','','','','','','','']);rows.push(['Method',d.method||'','','','','','','','']);return{head,rows};
}
function firdaPerformanceSaveExcel(){const m=firdaPerformanceExcelMatrix();if(!m)return toast('Rekap 4.6.2 belum dimuat.',true);if(typeof window.downloadSimpleXlsx!=='function')return toast('Exporter Excel belum termuat; Ctrl+Shift+R lalu coba lagi.',true);window.downloadSimpleXlsx(`E4.6.2_REKAP_KINERJA_STEERING_${stamp()}.xlsx`,'4.6.2 Rekap','4.6.2 Rekap Kinerja Steering',m.head,m.rows);toast('Excel 4.6.2 didownload',false)}
function firdaPerformanceBind(){
  if($('firdaPerformanceReload'))$('firdaPerformanceReload').onclick=()=>firdaPerformanceLoad(true);
  if($('firdaPerformanceSave'))$('firdaPerformanceSave').onclick=()=>{const csv=firdaPerformanceCsv();if(!csv)return toast('Rekap belum dimuat.',true);downloadBlob(`E4.6.2_REKAP_KINERJA_STEERING_${stamp()}.csv`,'text/csv;charset=utf-8',csv);toast('Rekap 4.6.2 CSV didownload',false)};if($('firdaPerformanceSaveExcel'))$('firdaPerformanceSaveExcel').onclick=()=>firdaPerformanceSaveExcel();
}
firdaPerformanceBind();
setInterval(()=>{if(firdaPerformanceActive())renderFirdaPerformanceRecapPanel()},750);

// FIRDA 4.6.3 integration validation: recorder-only. Vehicle motion remains under operator teleop authority.
const firdaIntegration={busy:false};
function firdaIntegrationActive(){return currentExp==='steering'&&firdaPosStep()==='4.6.3'}
function renderFirdaIntegrationPanel(){
  const p=$('firdaIntegrationPanel');if(!p)return;const active=firdaIntegrationActive();p.hidden=!active;if(!active)return;
  const rec=$('recordToggleBtn')?.dataset.active==='true',target=resolveMetricPath('derived.firda_runtime_target_deg'),actual=resolveMetricPath('derived.firda_runtime_actual_deg'),err=resolveMetricPath('derived.firda_runtime_error_deg'),speed=+raw('esc_drive_actual'),f=obj('foc_telemetry'),left=f?.left||{},mux=obj('nav_cmd_mux').raw||'--';
  setText('firdaIntegrationTarget',firdaPerformanceNum(target,2));setText('firdaIntegrationActual',firdaPerformanceNum(actual,2));setText('firdaIntegrationError',firdaPerformanceNum(err,2));setText('firdaIntegrationSpeed',firdaPerformanceNum(speed,3));
  setText('firdaIntegrationLive','Iq '+fmt(left.iq_a,2)+' A • Imotor '+fmt(left.current_motor_a,2)+' A • Duty '+fmt(left.duty,3)+' • Fault '+(left.fault??'--')+' • Mux '+mux);
  const chip=$('firdaIntegrationState');if(chip){chip.textContent=rec?'RECORDING':'READY';chip.className='status-chip '+(rec?'':'waiting')}
  if($('firdaIntegrationStart'))$('firdaIntegrationStart').disabled=rec||firdaIntegration.busy;
  if($('firdaIntegrationStop'))$('firdaIntegrationStop').disabled=!rec||firdaIntegration.busy;
}
async function firdaIntegrationStart(){
  if(!firdaIntegrationActive()||firdaIntegration.busy)return;if($('firdaIntegrationReady')?.checked!==true)return toast('Centang konfirmasi area aman dan joystick siap terlebih dahulu.',true);
  if(bool(raw('vesc_maintenance_active')))return toast('4.6.3 wajib mode RUNTIME. Keluar dari maintenance terlebih dahulu.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('Recorder sudah aktif.',true);
  firdaIntegration.busy=true;renderFirdaIntegrationPanel();
  try{
    if($('runVariation'))$('runVariation').value='INTEGRATION-TELEOP-3CYCLE';
    if($('runCondition'))$('runCondition').value='VEHICLE_MOVING_RUNTIME';
    if($('runCandidate'))$('runCandidate').value='final';
    if($('runSampleRate'))$('runSampleRate').value='50';
    if(selectedExperiment){localRunValues[`${currentExp}:${selectedExperiment.id}:test_steering_deg`]=20;localRunValues[`${currentExp}:${selectedExperiment.id}:test_erpm`]=0;}
    renderTuningFields();await startWebRecording();if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    setText('firdaIntegrationMessage','RECORDING • jalankan lurus 3–5 s → +10° → 0° → −10° → 0° → +20° → 0° → −20° → 0°. Ulang 3 cycle. Hentikan kendaraan sebelum STOP + SAVE.');
    toast('4.6.3 recording aktif • kendaraan tetap dikontrol joystick',false);
  }catch(e){setText('firdaIntegrationMessage','START gagal: '+(e?.message||e));toast('4.6.3 gagal START: '+(e?.message||e),true)}finally{firdaIntegration.busy=false;renderFirdaIntegrationPanel()}
}
async function firdaIntegrationStop(){
  if(!firdaIntegrationActive()||firdaIntegration.busy)return;firdaIntegration.busy=true;renderFirdaIntegrationPanel();
  try{const speed=Math.abs(+raw('esc_drive_actual')||0);if(speed>0.08)return toast('Hentikan kendaraan dulu (speed masih '+speed.toFixed(2)+' m/s), baru STOP + SAVE.',true);const saved=await stopWebRecording();if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');setText('firdaIntegrationMessage','SELESAI • data integrasi steering tersimpan. Jangan tuning ulang parameter final.');if($('firdaIntegrationReady'))$('firdaIntegrationReady').checked=false;toast('4.6.3 STOP + SAVE selesai',false)}catch(e){setText('firdaIntegrationMessage','STOP/SAVE gagal: '+(e?.message||e));toast('4.6.3 save gagal: '+(e?.message||e),true)}finally{firdaIntegration.busy=false;renderFirdaIntegrationPanel()}
}
function firdaIntegrationBind(){if($('firdaIntegrationReady'))$('firdaIntegrationReady').onchange=renderFirdaIntegrationPanel;if($('firdaIntegrationStart'))$('firdaIntegrationStart').onclick=()=>confirmModal('START 4.6.3 INTEGRASI','GUI hanya merekam. Pastikan area lapang, kendaraan mode Runtime, joystick siap, dan operator siap menghentikan kendaraan. Mulai dengan kecepatan rendah.',()=>firdaIntegrationStart());if($('firdaIntegrationStop'))$('firdaIntegrationStop').onclick=()=>firdaIntegrationStop()}
firdaIntegrationBind();setInterval(()=>{if(firdaIntegrationActive())renderFirdaIntegrationPanel()},250);


// FIRDA ESC ZN R-L REVISION R3
const FIRDA_ZN_KEY='firda-steering-zn-r3';
function firdaLoadStored(key){try{return JSON.parse(localStorage.getItem(key)||'null')}catch(_){return null}}
let firdaZnState=Object.assign({r:NaN,l:NaN,kp:NaN,ki:NaN,kd:NaN,iqStep:.5,stepDuration:.6},firdaLoadStored(FIRDA_ZN_KEY)||{});
function firdaRunSet(key,value){if(!selectedExperiment)return;localRunValues[currentExp+':'+selectedExperiment.id+':'+key]=value}
function firdaRmse(path){const a=metricHistory.get(path||'derived.firda_phys_error_deg')||[],v=a.map(x=>+x.v).filter(Number.isFinite);return v.length?Math.sqrt(v.reduce((q,x)=>q+x*x,0)/v.length):NaN}
function firdaZnCompute(r,l){r=+r;l=+l;if(!(r>0)||!(l>0))return null;const kp=1.2/(r*l),ti=2*l,td=.5*l,ki=kp/ti,kd=kp*td;return [kp,ki,kd].every(Number.isFinite)?{r,l,kp,ki,kd,ti,td}:null}
function firdaZnStore(z){if(!z)return false;firdaZnState=Object.assign({},firdaZnState,z,{updatedAt:new Date().toISOString()});localStorage.setItem(FIRDA_ZN_KEY,JSON.stringify(firdaZnState));return true}
function firdaZnActive(){return currentExp==='steering'&&firdaPosStep()==='4.3.1'}
const firdaZnRun={busy:false,abort:false};

function renderFirdaZnQuickPanel(){
  const p=$('firdaZnQuickPanel');if(!p)return;p.hidden=!firdaZnActive();if(p.hidden)return;
  const maint=bool(raw('vesc_maintenance_active')),lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$('firdaZnModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaZnKp'))$('firdaZnKp').value=Number.isFinite(+firdaZnState.kp)?fmt(firdaZnState.kp,8):'--';
  if($('firdaZnKi'))$('firdaZnKi').value=Number.isFinite(+firdaZnState.ki)?fmt(firdaZnState.ki,8):'--';
  if($('firdaZnKd'))$('firdaZnKd').value=Number.isFinite(+firdaZnState.kd)?fmt(firdaZnState.kd,8):'--';
  const iqRef=String(cmd.mode||'').toUpperCase().includes('CURRENT')?+cmd.value:NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg);
  firdaKpSetText('firdaZnLive','Iq* '+fmt(iqRef,3)+' A • Iq '+fmt(lv.iq_a,3)+' A • Steering '+fmt(actual,2)+'°');
  firdaKpSetText('firdaZnResult',Number.isFinite(+firdaZnState.kp)?'ZN: Kp '+fmt(firdaZnState.kp,6)+' • Ki '+fmt(firdaZnState.ki,6)+' • Kd '+fmt(firdaZnState.kd,6):'ZN: --');
  if($('firdaZnAuto')){$('firdaZnAuto').disabled=rec||firdaZnRun.busy;$('firdaZnAuto').textContent=firdaZnRun.busy?'IDENTIFIKASI BERJALAN…':'START IDENTIFIKASI R-L • RECORD + SAVE'}
}
async function firdaZnSaveCalculation(z){
  if(!selectedExperiment||!z)return false;
  const csv='R [deg/s/A],L [s],Kp ZN,Ti [s],Ki ZN,Td [s],Kd ZN,Metode\n'+[z.r,z.l,z.kp,z.ti,z.ki,z.td,z.kd,'Ziegler-Nichols reaction-curve R-L'].join(',')+'\n';
  try{const resp=await writeRequest('/api/experiment/table/save',{subsystem:'steering',id:'E4.3.1',label:'E4.3.1_ZN_RL_CALCULATION',source_experiment_id:selectedExperiment.id,table_index:2,csv}),j=await resp.json();if(!resp.ok)throw new Error(j.message||('HTTP '+resp.status));return true}catch(e){toast('Save perhitungan ZN gagal: '+e.message,true);return false}
}
async function firdaZnCalculateManual(){
  const z=firdaZnCompute(+$('firdaZnR').value,+$('firdaZnL').value);
  if(!z)return toast('R dan L harus > 0 dan finite.',true);
  firdaZnStore(z);
  for(const pair of [['zn_r',z.r],['zn_l_s',z.l],['zn_kp',z.kp],['zn_ki',z.ki],['zn_kd',z.kd]])firdaRunSet(pair[0],pair[1]);
  renderTuningFields();renderFirdaZnQuickPanel();
  const saved=await firdaZnSaveCalculation(z);
  firdaKpSetText('firdaZnState','ZN dihitung dari R/L manual'+(saved?' • CSV perhitungan tersimpan':' • cek save calculation')+'. Lanjutkan 4.3.2 untuk baseline.');
  toast('ZN R-L: Kp '+fmt(z.kp,6)+' • Ki '+fmt(z.ki,6)+' • Kd '+fmt(z.kd,6),!saved);
}
async function firdaZnAutoRun(){
  if(firdaZnRun.busy)return;
  if($('firdaZnReady')?.checked!==true)return toast('Centang konfirmasi safety 4.3.1 terlebih dahulu.',true);
  const iq=+$('firdaZnIqStep').value,dur=+$('firdaZnStepDuration').value;
  if(!Number.isFinite(iq)||iq<.1||iq>1.0||!Number.isFinite(dur)||dur<.2||dur>1.0)return toast('Iq harus 0.1–1.0 A dan durasi 0.2–1.0 s.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaZnRun.busy=true;firdaZnRun.abort=false;firdaZnState.iqStep=iq;firdaZnState.stepDuration=dur;localStorage.setItem(FIRDA_ZN_KEY,JSON.stringify(firdaZnState));
  if($('runVariation'))$('runVariation').value='ZN-RL-IQ'+iq.toFixed(2)+'A';
  if($('runCondition'))$('runCondition').value='OPEN_LOOP_IQ_STEP';
  if($('runCandidate'))$('runCandidate').value='baseline';
  if($('runSampleRate'))$('runSampleRate').value='50';
  firdaRunSet('iq_step_a',iq);firdaRunSet('step_duration_s',dur);firdaRunSet('identification_note','raw reaction curve; hitung R/L dari grafik');
  renderTuningFields();renderFirdaZnQuickPanel();
  try{
    firdaKpSetText('firdaZnState','Menyiapkan center sebelum step Iq*…');
    if(!bool(raw('vesc_maintenance_active'))){await firdaKpCommand('MODE:MAINTENANCE');if(!await firdaKpWait(()=>bool(raw('vesc_maintenance_active')),5000))throw new Error('Maintenance tidak aktif')}
    await firdaPosApplyPID(.05,.005,0);await firdaKpCommand('SET:POS:1:180');
    if(!await firdaPosWaitTarget(180,9,7000))throw new Error('Steering belum mencapai CENTER ±1.5°');
    await firdaKpCommand('SAFE_STOP:BOTH');metricHistory.clear();scatterHistory.clear();
    await startWebRecording();if(!await firdaKpWait(()=>$('recordToggleBtn')?.dataset.active==='true',2500))throw new Error('Recorder gagal START');
    await firdaKpPollLeft(800,60);if(firdaZnRun.abort)throw new Error('Identifikasi dibatalkan');
    firdaKpSetText('firdaZnState','Step Iq* '+iq.toFixed(2)+' A selama '+dur.toFixed(2)+' s…');
    await firdaKpCommand('SET:CURRENT:1:'+iq.toFixed(3));await firdaKpPollLeft(dur*1000,40);
    await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpPollLeft(1500,60);
    const saved=await stopWebRecording();if(saved!==true)throw new Error('Save recorder belum terkonfirmasi');
    await firdaKpCommand('SAFE_STOP:BOTH');await firdaKpCommand('MODE:RUNTIME',15000);await firdaKpWait(()=>!bool(raw('vesc_maintenance_active')),5000);
    firdaKpSetText('firdaZnState','SELESAI • raw reaction curve tersimpan. Tentukan R/L dari grafik lalu klik HITUNG ZN.');
    if($('firdaZnReady'))$('firdaZnReady').checked=false;toast('4.3.1 reaction curve tersimpan',false)
  }catch(e){
    try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}
    try{if($('recordToggleBtn')?.dataset.active==='true')await stopWebRecording()}catch(_){}
    try{if(bool(raw('vesc_maintenance_active')))await firdaKpCommand('MODE:RUNTIME',15000)}catch(_){}
    firdaKpSetText('firdaZnState','STOP: '+(e?.message||e));toast('Identifikasi R-L berhenti: '+(e?.message||e),true)
  }finally{firdaZnRun.busy=false;renderFirdaZnQuickPanel()}
}

function renderFirdaZnBaselinePanel(){
  const p=$('firdaZnBaselinePanel');if(!p)return;const active=currentExp==='steering'&&firdaPosStep()==='4.3.2';p.hidden=!active;if(!active)return;
  if($('firdaZnBaseKp')&&!$('firdaZnBaseKp').matches(':focus')&&!$('firdaZnBaseKp').value&&Number.isFinite(+firdaZnState.kp))$('firdaZnBaseKp').value=String(firdaZnState.kp);
  if($('firdaZnBaseKi')&&!$('firdaZnBaseKi').matches(':focus')&&!$('firdaZnBaseKi').value&&Number.isFinite(+firdaZnState.ki))$('firdaZnBaseKi').value=String(firdaZnState.ki);
  if($('firdaZnBaseKd')&&!$('firdaZnBaseKd').matches(':focus')&&!$('firdaZnBaseKd').value&&Number.isFinite(+firdaZnState.kd))$('firdaZnBaseKd').value=String(firdaZnState.kd);
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),rm=firdaRmse(),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$('firdaZnBaselineModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaZnBaseActual'))$('firdaZnBaseActual').value=t?(fmt(t.pos_kp,6)+' / '+fmt(t.pos_ki,6)+' / '+fmt(t.pos_kd,6)):'-- / -- / --';
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN;
  firdaKpSetText('firdaZnBaselineLive','Setpoint '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • RMSE '+fmt(rm,3)+'° • Iq '+fmt(lv.iq_a,2)+' A');
  firdaKpSetText('firdaZnBaselineCount','Cycle: '+firdaPosAdv.cycles+' / 3');if($('firdaZnBaselineAuto'))$('firdaZnBaselineAuto').disabled=rec||firdaPosAdv.busy;
}

function firdaFinalGainValid(){return [FIRDA_POS_FINAL.kp,FIRDA_POS_FINAL.ki,FIRDA_POS_FINAL.kd].every(Number.isFinite)}
function firdaPersistFinalGain(kp,ki,kd){
  if(![kp,ki,kd].every(Number.isFinite))return false;
  FIRDA_POS_FINAL.kp=+kp;FIRDA_POS_FINAL.ki=+ki;FIRDA_POS_FINAL.kd=+kd;
  localStorage.setItem('firda-steering-final-gain-r3',JSON.stringify({kp:+kp,ki:+ki,kd:+kd,selectedAt:new Date().toISOString()}));
  return true;
}
function firdaFillZnCandidate(base,idFactor,idValue){
  const measured=+firdaZnState[base],fallback=base==='kp'?.025:(base==='ki'?.0025:(base==='kd'?.004:NaN)),b=measured>0?measured:fallback,f=+$(idFactor)?.value;
  if(Number.isFinite(b)&&Number.isFinite(f)&&$(idValue))$(idValue).value=String(b*f)
}
function firdaKpMetaR3(v){const st=$('firdaPosKpStage')?.value||'coarse',f=+$('firdaPosKpFactor')?.value;return ['POSKP-'+st.toUpperCase()+'-X'+(Number.isFinite(f)?f.toFixed(3):'MANUAL')+'-KP'+Number(v).toFixed(6),st==='fine'?'candidate-b':'candidate-a']}
firdaPosKpMeta=firdaKpMetaR3;
firdaPosKpApplyIdentity=function(v){
  const m=firdaPosKpMeta(v),variation=m[0],candidate=m[1];
  if($('runVariation'))$('runVariation').value=variation;if($('runCondition'))$('runCondition').value='STEP_PM5DEG_KI0_KD0';if($('runCandidate'))$('runCandidate').value=candidate;
  firdaRunSet('tuning_stage',$('firdaPosKpStage')?.value||'coarse');firdaRunSet('zn_kp',firdaZnState.kp);firdaRunSet('zn_factor',+$('firdaPosKpFactor')?.value);firdaRunSet('kp_test',v);firdaRunSet('ki_locked',0);firdaRunSet('kd_locked',0);firdaRunSet('setpoint_deg',5);
}
const _firdaPosKpRenderR3=renderFirdaPosKpQuickPanel;
renderFirdaPosKpQuickPanel=function(){
  _firdaPosKpRenderR3();
  if(!firdaPosKpActive())return;
  if($('firdaPosKpZn'))$('firdaPosKpZn').value=Number.isFinite(+firdaZnState.kp)?fmt(firdaZnState.kp,8):'--';
  const rm=firdaRmse();const lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN;
  firdaKpSetText('firdaPosKpLive','Setpoint '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • RMSE '+fmt(rm,3)+'° • Iq '+fmt(lv.iq_a,2)+' A');
}
const _renderAdvR3=renderFirdaPosAdvancedPanels;
renderFirdaPosAdvancedPanels=function(){
  _renderAdvR3();
  const st=firdaPosStep();if(st==='4.3.4'){if($('firdaPosKiZn'))$('firdaPosKiZn').value=(Number.isFinite(+firdaZnState.ki)&&+firdaZnState.ki>0)?fmt(firdaZnState.ki,8):fmt(.0025,8);if($('firdaPosKiFixedKp')&&!$('firdaPosKiFixedKp').matches(':focus')&&(!(+$('firdaPosKiFixedKp').value)||+$('firdaPosKiFixedKp').value<=0))$('firdaPosKiFixedKp').value='0.0375';if($('firdaPosKiValue')&&!$('firdaPosKiValue').matches(':focus')&&!Number.isFinite(+$('firdaPosKiValue').value))$('firdaPosKiValue').value='0';}
  if(st==='4.3.6'){if($('firdaPosFinalKp')&&!$('firdaPosFinalKp').matches(':focus'))$('firdaPosFinalKp').value='.0375';if($('firdaPosFinalKi')&&!$('firdaPosFinalKi').matches(':focus'))$('firdaPosFinalKi').value='.0025';if($('firdaPosFinalKd')&&!$('firdaPosFinalKd').matches(':focus'))$('firdaPosFinalKd').value='0';}if(st==='4.3.5'&&$('firdaPosKdZn'))$('firdaPosKdZn').value=(Number.isFinite(+firdaZnState.kd)&&+firdaZnState.kd>0)?fmt(firdaZnState.kd,8):fmt(.004,8);
  const c=firdaPosPanelCfg();if(!c)return;const lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN,rm=firdaRmse();
  firdaKpSetText(c.live,'Setpoint '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • RMSE '+fmt(rm,3)+'° • Iq '+fmt(lv.iq_a,2)+' A');
}
const _firdaPosAdvParamsR3=firdaPosAdvParams;
firdaPosAdvParams=function(){
  const p=_firdaPosAdvParamsR3();if(!p)return p;const st=firdaPosStep();
  if(st==='4.3.2'){firdaRunSet('zn_kp',p.kp);firdaRunSet('zn_ki',p.ki);firdaRunSet('zn_kd',p.kd);firdaRunSet('setpoint_deg',5);firdaRunSet('repeat_count',3)}
  if(st==='4.3.4'){firdaRunSet('tuning_stage',$('firdaPosKiStage')?.value||'coarse');firdaRunSet('kp_locked',p.kp);firdaRunSet('zn_ki',firdaZnState.ki);firdaRunSet('zn_factor',+$('firdaPosKiFactor')?.value);firdaRunSet('ki_test',p.ki);firdaRunSet('kd_locked',0);firdaRunSet('setpoint_deg',5)}
  if(st==='4.3.5'){firdaRunSet('tuning_stage',$('firdaPosKdStage')?.value||'coarse');firdaRunSet('kp_locked',p.kp);firdaRunSet('ki_locked',p.ki);firdaRunSet('zn_kd',firdaZnState.kd);firdaRunSet('zn_factor',+$('firdaPosKdFactor')?.value);firdaRunSet('kd_test',p.kd);firdaRunSet('setpoint_deg',5)}
  if(st==='4.3.6'){firdaRunSet('final_kp',p.kp);firdaRunSet('final_ki',p.ki);firdaRunSet('final_kd',p.kd);firdaRunSet('setpoint_deg',5);firdaRunSet('selection_note','RMSE + overshoot + settling + Iq');firdaRunSet('firmware_handoff','VALIDASI_GUI_SELESAI')}
  return p;
}

const _renderResponseR3=renderFirdaResponsePanel;
renderFirdaResponsePanel=function(){
  _renderResponseR3();const c=firdaResponseCfg();if(!c)return;
  if($('firdaResponseFinalKp'))$('firdaResponseFinalKp').value=firdaFinalGainValid()?String(FIRDA_POS_FINAL.kp):'--';
  if($('firdaResponseFinalKi'))$('firdaResponseFinalKi').value=firdaFinalGainValid()?String(FIRDA_POS_FINAL.ki):'--';
  if($('firdaResponseFinalKd'))$('firdaResponseFinalKd').value=firdaFinalGainValid()?String(FIRDA_POS_FINAL.kd):'--';
  firdaKpSetText('firdaResponseHint',(firdaFinalGainValid()?'Gain final 4.3.6: Kp '+FIRDA_POS_FINAL.kp+', Ki '+FIRDA_POS_FINAL.ki+', Kd '+FIRDA_POS_FINAL.kd+'. ':'GAIN FINAL BELUM DITETAPKAN DI 4.3.6. ')+c.hint);
  const rm=firdaRmse();const lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN;
  firdaKpSetText('firdaResponseLive','Setpoint '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • RMSE '+fmt(rm,3)+'° • Iq '+fmt(lv.iq_a,2)+' A');
  if($('firdaResponseAuto'))$('firdaResponseAuto').disabled=$('firdaResponseAuto').disabled||!firdaFinalGainValid();
}
const _responseRunR3=firdaResponseRun;
firdaResponseRun=async function(){if(!firdaFinalGainValid())return toast('Gain final belum ditetapkan melalui 4.3.6.',true);firdaRunSet('final_kp',FIRDA_POS_FINAL.kp);firdaRunSet('final_ki',FIRDA_POS_FINAL.ki);firdaRunSet('final_kd',FIRDA_POS_FINAL.kd);return await _responseRunR3()}

const _renderLoadR3=renderFirdaLoadPanel;
renderFirdaLoadPanel=function(){
  _renderLoadR3();const c=firdaLoadCfg();if(!c)return;
  firdaKpSetText('firdaLoadEyebrow','FIRDA • '+firdaPosStep()+(c.torque?' TORSI MEKANISME':' TRACKING BEBAN'));firdaKpSetText('firdaLoadTitle',(c.torque?'Torsi Mekanisme • ':'Tracking ')+c.galon+' Galon • LEFT');
  if($('firdaLoadFinalPid'))$('firdaLoadFinalPid').value=firdaFinalGainValid()?'Kp '+FIRDA_POS_FINAL.kp+' • Ki '+FIRDA_POS_FINAL.ki+' • Kd '+FIRDA_POS_FINAL.kd:'BELUM DITETAPKAN DI 4.3.6';
  if($('firdaTorqueFields'))$('firdaTorqueFields').hidden=!c.torque;if($('firdaTorqueSave'))$('firdaTorqueSave').hidden=!c.torque;
  const F=+$('firdaTorqueForce')?.value,r=+$('firdaTorqueArm')?.value,tau=Number.isFinite(F)&&Number.isFinite(r)&&F>=0&&r>0?F*r:NaN;if($('firdaTorqueNm'))$('firdaTorqueNm').value=Number.isFinite(tau)?tau.toFixed(4)+' N·m':'-- N·m';
  firdaKpSetText('firdaLoadHint',c.torque?'Pilih beban 0–3 galon. Rekam tracking/current lalu masukkan Force Peak dan lever arm. τ=F×r adalah torsi keluaran mekanisme, bukan shaft torque BLDC.':'Pilih 0–3 galon. Roda depan di tanah, kendaraan diam, joystick netral. Gerakan −20° → 0° → +20° → 0°, 3 cycle.');
  const rm=firdaRmse();const lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?target-actual:NaN;
  firdaKpSetText('firdaLoadLive','Setpoint '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • RMSE '+fmt(rm,3)+'° • Iq '+fmt(lv.iq_a,2)+' A • Imotor '+fmt(lv.current_motor_a,2)+' A');
  if($('firdaLoadAuto'))$('firdaLoadAuto').disabled=$('firdaLoadAuto').disabled||!firdaFinalGainValid();
}
const _loadRunR3=firdaLoadRun;
firdaLoadRun=async function(){const c=firdaLoadCfg();if(!c)return;if(!firdaFinalGainValid())return toast('Gain final belum ditetapkan melalui 4.3.6.',true);firdaRunSet('load_galon',c.galon);firdaRunSet('final_kp',FIRDA_POS_FINAL.kp);firdaRunSet('final_ki',FIRDA_POS_FINAL.ki);firdaRunSet('final_kd',FIRDA_POS_FINAL.kd);firdaRunSet('setpoint_deg',20);return await _loadRunR3()}
async function firdaSaveTorqueMetadata(){
  const c=firdaLoadCfg(),F=+$('firdaTorqueForce')?.value,r=+$('firdaTorqueArm')?.value;if(!c||!c.torque)return;if(!firdaFinalGainValid())return toast('Gain final 4.3.6 belum ada.',true);if(!Number.isFinite(F)||F<0||!Number.isFinite(r)||r<=0)return toast('Force Peak dan lever arm harus valid.',true);
  const tau=F*r;firdaRunSet('force_n',F);firdaRunSet('lever_arm_m',r);firdaRunSet('torque_nm',tau);
  const csv='Beban [galon],Force Peak [N],Lever Arm [m],Torque Mekanisme [N.m],Kp Final,Ki Final,Kd Final,Catatan\n'+[c.galon,F,r,tau,FIRDA_POS_FINAL.kp,FIRDA_POS_FINAL.ki,FIRDA_POS_FINAL.kd,'torsi keluaran mekanisme; bukan shaft BLDC'].join(',')+'\n';
  try{const resp=await writeRequest('/api/experiment/table/save',{subsystem:'steering',id:'E4.5.2',label:'E4.5.2_TORQUE',source_experiment_id:selectedExperiment?.id||'4.8.1',table_index:1,csv}),j=await resp.json();if(!resp.ok)throw new Error(j.message||('HTTP '+resp.status));toast('Metadata torsi tersimpan: '+(j.path||'CSV'),false)}catch(e){toast('Save torsi gagal: '+e.message,true)}
}
const _finalRowsR3=firdaFinalRecapRows;
firdaFinalRecapRows=function(){const rows=_finalRowsR3();for(const row of rows){if(row.key==='pos_kp')row.ref=firdaFinalGainValid()?FIRDA_POS_FINAL.kp:null;if(row.key==='pos_ki')row.ref=firdaFinalGainValid()?FIRDA_POS_FINAL.ki:null;if(row.key==='pos_kd')row.ref=firdaFinalGainValid()?FIRDA_POS_FINAL.kd:null}return rows}

function firdaEscRevisionBindR3(){
  if($('firdaZnCalc'))$('firdaZnCalc').onclick=firdaZnCalculateManual;
  if($('firdaZnAuto'))$('firdaZnAuto').onclick=()=>confirmModal('START IDENTIFIKASI R-L','Steering akan dicenter lalu diberi step Iq* singkat. Roda wajib aman/terangkat dan area gerak bebas. Data reaction curve akan direkam dan disimpan.',()=>firdaZnAutoRun());
  if($('firdaZnStop'))$('firdaZnStop').onclick=async()=>{firdaZnRun.abort=true;try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}firdaKpSetText('firdaZnState','STOP CURRENT • identifikasi dibatalkan')};
  if($('firdaZnBaselineAuto'))$('firdaZnBaselineAuto').onclick=()=>confirmModal('START BASELINE ZN','Baseline ZN akan diterapkan runtime-only dan diuji ±5° selama 3 cycle.',()=>firdaPosAdvRun());
  if($('firdaZnBaselineStop'))$('firdaZnBaselineStop').onclick=async()=>{firdaPosAdv.abort=true;try{await firdaKpCommand('SAFE_STOP:BOTH',8000)}catch(_){}};
  const setFactor=(id,factorId,valueId,base,f)=>{if($(id))$(id).onclick=()=>{$(factorId).value=String(f);firdaFillZnCandidate(base,factorId,valueId);if(valueId==='firdaPosKpValue')firdaPosKpApplyIdentity(+$('firdaPosKpValue').value);renderTuningFields();renderFirdaPosKpQuickPanel();renderFirdaPosAdvancedPanels()}};
  setFactor('firdaPosKpPresetLow','firdaPosKpFactor','firdaPosKpValue','kp',.5);setFactor('firdaPosKpPresetMid','firdaPosKpFactor','firdaPosKpValue','kp',.75);setFactor('firdaPosKpPresetHigh','firdaPosKpFactor','firdaPosKpValue','kp',1);setFactor('firdaPosKpPreset125','firdaPosKpFactor','firdaPosKpValue','kp',1.25);setFactor('firdaPosKpPreset150','firdaPosKpFactor','firdaPosKpValue','kp',1.5);
  if($('firdaPosKpFactor'))$('firdaPosKpFactor').onchange=()=>{firdaFillZnCandidate('kp','firdaPosKpFactor','firdaPosKpValue');firdaPosKpApplyIdentity(+$('firdaPosKpValue').value);renderTuningFields()};
  setFactor('firdaPosKiPreset0','firdaPosKiFactor','firdaPosKiValue','ki',0);setFactor('firdaPosKiPresetLow','firdaPosKiFactor','firdaPosKiValue','ki',.5);setFactor('firdaPosKiPresetMid','firdaPosKiFactor','firdaPosKiValue','ki',1);setFactor('firdaPosKiPresetHigh','firdaPosKiFactor','firdaPosKiValue','ki',1.5);setFactor('firdaPosKiPreset200','firdaPosKiFactor','firdaPosKiValue','ki',2);
  if($('firdaPosKiFactor'))$('firdaPosKiFactor').onchange=()=>{firdaFillZnCandidate('ki','firdaPosKiFactor','firdaPosKiValue');renderTuningFields()};
  setFactor('firdaPosKdPreset0','firdaPosKdFactor','firdaPosKdValue','kd',0);setFactor('firdaPosKdPreset050','firdaPosKdFactor','firdaPosKdValue','kd',.5);setFactor('firdaPosKdPreset2','firdaPosKdFactor','firdaPosKdValue','kd',1);setFactor('firdaPosKdPreset150','firdaPosKdFactor','firdaPosKdValue','kd',1.5);setFactor('firdaPosKdPreset4','firdaPosKdFactor','firdaPosKdValue','kd',2);
  if($('firdaPosKdFactor'))$('firdaPosKdFactor').onchange=()=>{firdaFillZnCandidate('kd','firdaPosKdFactor','firdaPosKdValue');renderTuningFields()};
  for(const id of ['firdaZnBaselineAuto','firdaPosKiAuto','firdaPosKdAuto','firdaPosFinalAuto'])if($(id)&&id!=='firdaZnBaselineAuto')$(id).onclick=()=>confirmModal('START TEST POSISI','Steering wajib aman. Sistem akan center, apply gain runtime-only, menjalankan ±5°, menyimpan data, lalu kembali Runtime.',()=>firdaPosAdvRun());
  if($('firdaLoadLevel'))$('firdaLoadLevel').onchange=()=>{if($('firdaLoadReady'))$('firdaLoadReady').checked=false;renderFirdaLoadPanel()};
  for(const id of ['firdaTorqueForce','firdaTorqueArm'])if($(id))$(id).oninput=renderFirdaLoadPanel;
  if($('firdaTorqueSave'))$('firdaTorqueSave').onclick=firdaSaveTorqueMetadata;
}
firdaEscRevisionBindR3();
setInterval(()=>{renderFirdaZnQuickPanel();renderFirdaZnBaselinePanel();if(firdaPosStep()==='4.3.3')renderFirdaPosKpQuickPanel();if(/^4\.3\.[456]$/.test(firdaPosStep()))renderFirdaPosAdvancedPanels()},250);
