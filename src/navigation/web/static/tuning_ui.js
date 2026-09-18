'use strict';
function renderExperimentList(){const all=effectiveExperimentList();setText('labLeafCount',all.length);const box=$('experimentList');if(!box)return;const term=($('experimentSearch')?.value||'').trim().toLowerCase();const list=all.filter(x=>!term||`${displayExperimentId(x)} ${displayExperimentTitle(x)} ${displayExperimentGroup(x)} ${phaseLabel(experimentPhase(x))} ${x.id}`.toLowerCase().includes(term));const visited=visitedSet();let lastGroup='',html='';for(const x of list){const group=`${phaseLabel(experimentPhase(x))} · ${displayExperimentGroup(x)}`,displayId=displayExperimentId(x),title=displayExperimentTitle(x);if(group!==lastGroup){html+=`<div class="chapter-group"><span>${escapeHtml(group)}</span></div>`;lastGroup=group}html+=`<button class="experiment-item ${selectedExperiment?.id===x.id?'active':''}" data-id="${escapeHtml(x.id)}"><i>${visited.has(x.id)?'✓':'○'}</i><div><span>${escapeHtml(displayId)}</span><b>${escapeHtml(title)}</b><small>${(x.tableNames||[]).length} tabel • ${(x.graphCaptions||[]).length} grafik • ${tuningFieldsFor(x).filter(p=>p.yamlFileKey).length} parameter</small></div></button>`}box.innerHTML=html||'<div class="empty-state">Tidak ada tahap tuning yang cocok.</div>';qa('.experiment-item',box).forEach(e=>e.onclick=()=>selectExperimentById(e.dataset.id));window.navigationTuningPolicy?.decorateList?.(box,list);setText('chapterDomainTitle',currentExp==='navigation'?'Navigasi':currentExp==='perception'?'Persepsi':'ESC / FOC');renderChapterProgress();if(activeDomain!=='overview')renderContextNav()}
function selectExperimentById(id){if(window.navigationTuningPolicy?.beforeSelect?.(selectedExperiment,id)===false)return;if(recordStartedMs&&selectedExperiment&&id!==selectedExperiment.id){toast(`Recording ${selectedExperiment.id} masih aktif. STOP dulu sebelum pindah tahap.`,true);return}const x=findExperimentById(id);if(!x)return;selectedExperiment=x;window.analysisSession?.selectTask(currentExp,id);selectedTableIndex=0;selectedGraphIndex=0;metricHistory.clear();scatterHistory.clear();scatterOrigins.clear();tableRunRows.clear();reportPathSnapshots.clear();reportEvidenceFrozen=false;reportEvidenceFrozenLeaf='';reportSourceTableRenderMs=0;ekfGrowthState.clear();reportDerivedCache.clear();reportCostmapMetricCache={key:'',minClearance:NaN,valid:NaN};chartEpochMs=Date.now();lastLabSecond=-1;lastLabTelemetryTick=-1;experimentCursorTime=null;if(!x.reportMode)markVisited(id);if(activeDomain==='navigation')activeWorkspaceTab.navigation=x.reportMode===true?'bab4':'tune';else if(activeDomain==='perception')activeWorkspaceTab.perception=/^F4\./.test(String(x.id||''))?'bab4':'tune';else if(activeDomain==='esc'&&currentExp==='steering'&&activeWorkspaceTab.esc!=='tests')activeWorkspaceTab.esc='bab4';window.navigationTuningPolicy?.onSelect?.(x);renderExperimentList();renderSelectedExperiment();renderWorkspaceTabs()}
function syncReportDocumentLayout(){
  const table=document.querySelector('.table-workspace'),graph=document.querySelector('.graph-workspace'),two=document.querySelector('.lab-two-col'),per=$('perceptionEngineeringPanel'),recap=$('trialRecapPanel');if(!table||!graph||!two||!recap)return;const main=table.parentNode,report=currentExp==='navigation'&&selectedExperiment?.reportMode===true;
  let banner=$('reportDocumentFlow');if(!banner){banner=document.createElement('article');banner.id='reportDocumentFlow';banner.className='panel report-document-flow';banner.innerHTML='<div><span>FORMAT BAB IV • DATA AKTUAL</span><b>1. PREVIEW → START</b><small>Preview live tidak masuk evidence • START reset sesi ke t=0</small></div><i>↓</i><div><b>2. Tabel Sumber + Grafik Selaras</b><small>Satu buffer recording • setiap grafik punya tabel sumber aktual</small></div><i>↓</i><div><b>3. STOP → FREEZE → SAVE</b><small>Tabel + grafik berhenti bersama • CSV / Excel / PNG tidak berubah lagi</small></div>';}
  document.body.classList.toggle('navigation-report-mode',report);banner.hidden=!report;
  if(report){main.insertBefore(table,graph);main.insertBefore(banner,table);main.insertBefore(graph,two);if(per)main.insertBefore(two,per)}
  else{if(banner.parentNode===main)main.insertBefore(banner,recap);main.insertBefore(graph,two);if(per){main.insertBefore(two,per);main.insertBefore(per,table)}main.insertBefore(table,recap)}
}
function renderSelectedExperiment(){const x=selectedExperiment;if(!x)return;const domain=currentExp==='navigation'?'Navigasi':currentExp==='perception'?'Persepsi':'ESC / FOC',id=displayExperimentId(x),title=displayExperimentTitle(x),report=x.reportMode===true;setText('experimentGroup',displayExperimentGroup(x));setText('experimentTitle',`${id} ${title}`);const n0=currentExp==='navigation'&&x.id==='N0.1',ekfReport=currentExp==='navigation'&&/^R4\.[23]\.[1-5]$/.test(String(x.id||''));setText('experimentSubtitle',report?(ekfReport?`${domain} • LAPORAN BAB IV ${id} • YAML source-of-truth → safe runtime apply/read-back → data aktual berjalan • setiap kondisi diulang 3 cycle dan dibandingkan overall.`:`${domain} • LAPORAN BAB IV • rekap data dari ${x.sourceIds?.join(', ')||'source tuning'} • START/STOP, Excel, grafik, dan parameter YAML memakai mesin akuisisi Navigasi yang sama.`):n0?'Navigasi • N0 Timing Readiness • target konfigurasi dan rate aktual dipisahkan. ESC command-loop target bukan rate /esc/odom feedback.':`${domain} • Tuning ${id} • parameter, tabel, grafik, dan live evidence mengikuti source pengujian ini.`);if(activePage==='experiments'){setText('pageEyebrow',report?'NAVIGASI • LAPORAN BAB IV':`${domain.toUpperCase()} • TUNING`);setText('pageTitle',`${id} ${title}`)}syncReportDocumentLayout();const imuLinearityExtra=report&&x.id==='R4.1.2';setText('experimentTableBadge',`${(x.tableNames||[]).length+(imuLinearityExtra?1:0)} TABLE`);setText('experimentGraphBadge',`${(x.graphCaptions||[]).length+(imuLinearityExtra?2:0)} GRAPH`);setText('experimentRawMetadata',JSON.stringify(x,null,2));renderTuningFields();renderLabTabs();renderExperimentTable();renderExperimentGraphs();window.renderImuHeadingLinearity?.();window.renderReport41TimedPanel?.();renderContextVisual();renderSourceAudit();renderN0TimingGuide();renderPerceptionEngineering();renderTrialGuideDetailed();renderTrialRecap();renderCommissioningRoadmap();renderQualificationPanel();window.renderEkfReportRuntimePanel?.();window.navigationTuningPolicy?.render?.();renderFirdaKpQuickPanel();renderFirdaKiQuickPanel();renderFirdaPiQuickPanel();renderFirdaPosKpQuickPanel();renderFirdaPosAdvancedPanels();renderFirdaResponsePanel();renderFirdaLoadPanel()}
let tuningMode='unified',tuningLevel='all';
const HELP_REGISTRY=new Map([
  ['action.tuning.validate',{label:'Validate Draft',summary:'Memeriksa draft tanpa menulis YAML atau mengubah runtime.',apply_mode:'read-only validation',risk:'Tidak menggerakkan actuator dan aman digunakan sebelum Apply.'}],
  ['action.tuning.apply',{label:'Apply + Save',summary:'Menulis seluruh draft terpilih sebagai satu transaksi, lalu menerapkan runtime dan melakukan read-back.',apply_mode:'atomic batch + runtime verify',risk:'Untuk parameter motion/safety, stationary gate dan runtime capability tetap berlaku.'}],
  ['action.tuning.revert',{label:'Revert Draft',summary:'Membuang draft browser dan kembali menampilkan nilai YAML aktual.',apply_mode:'browser only',risk:'Tidak menulis YAML.'}],
  ['action.tuning.baseline',{label:'Stage Baseline',summary:'Menyalin baseline awal ke Draft. Nilai belum ditulis sampai Apply + Save.',apply_mode:'stage then batch apply',risk:'Periksa diff sebelum Apply.'}],
  ['panel.perception.safety',{label:'Perception Safety Gate',summary:'Fail-closed gate untuk candidate obstacle dan trajectory safety. Bypass hanya untuk commissioning terkontrol.',apply_mode:'safety configuration',risk:'HIGH — bypass mengurangi proteksi persepsi; E-stop dan watchdog lain tetap wajib.'}],
  ['panel.tuning.editor',{label:'Parameter Tuning',summary:'Editor empat-state: Baseline, YAML, Draft, dan Runtime. Input tidak auto-write.',apply_mode:'Draft → Validate → Apply + Verify',risk:'Semua parameter relevan tampil dalam satu halaman; review source/path teknis sebelum Apply.'}],
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
function renderTuningFields(){
  const x=selectedExperiment,fields=tuningFieldsFor(x),readOnly=obj('server').read_only===true;setText('labYamlMode',readOnly?'LOCKED':'DRAFT → VALIDATE → APPLY');
  const resetBtn=$('resetExperimentYaml'),resettable=fields.filter(p=>p.yamlFileKey&&p.kind!=='yaml_readonly'&&!p.locked&&schemaMetaForParam(p)?.write_authority==='ros_yaml'&&baselineConfigValue(p.yamlFileKey,p.yamlPath)!==undefined);if(resetBtn){resetBtn.hidden=!resettable.length;resetBtn.disabled=readOnly||!!recordStartedMs;resetBtn.textContent=`↶ Stage ${resettable.length} Baseline Awal`}
  if(!fields.length){$('tuningFields').innerHTML='<div class="empty-state">Tahap validasi/ringkasan; tidak ada parameter editable.</div>';updateTuningDraftControls();decoratePanelHelp();return}
  const groups=new Map();fields.forEach((p,i)=>{let g=p.group||'';if(!g){if(['sample_rate','duration'].includes(p.key))g='Akuisisi';else if(['variation','condition'].includes(p.key))g='Identitas Run';else if(String(p.key||'').startsWith('gt_'))g='Ground Truth';else g='Parameter / YAML'}if(!groups.has(g))groups.set(g,[]);groups.get(g).push({p,i})});let html='';
  for(const [group,items] of groups){html+=`<section class="tuning-subgroup"><div class="tuning-subgroup-title"><b>${escapeHtml(group)}</b><span>${items.length} field</span></div>`;for(const {p,i} of items){const isYaml=!!p.yamlFileKey,meta=isYaml?schemaMetaForParam(p):null,schemaLocked=isYaml&&(!meta?.metadata_complete||meta.write_authority!=='ros_yaml'),ro=p.kind==='yaml_readonly'||p.locked||readOnly||schemaLocked,identity=isYaml?`${p.yamlFileKey}:${p.yamlPath}`:'',yamlVal=isYaml?configValue(p.yamlFileKey,p.yamlPath):undefined,pending=isYaml?configPending.get(identity):null,rawVal=isYaml?(pending?.value??yamlVal):(localRunValues[`${currentExp}:${x.id}:${p.key||p.label}`]??p.placeholder??''),val=p.kind==='list'&&Array.isArray(rawVal)?JSON.stringify(rawVal):rawVal,baseVal=isYaml?baselineConfigValue(p.yamlFileKey,p.yamlPath):undefined,baseFull=baseVal===undefined?'':(Array.isArray(baseVal)||baseVal&&typeof baseVal==='object'?JSON.stringify(baseVal):String(baseVal)),baseShort=baselineText(baseVal);let control;if(p.kind==='bool')control=`<label class="switch"><input id="tuneField-${i}" data-auto-index="${i}" type="checkbox" ${rawVal?'checked':''} ${ro?'disabled':''}><i></i><b>${rawVal?'TRUE':'FALSE'}</b></label>`;else if(p.kind==='int'||p.kind==='float')control=paramNumericHtml(`tuneField-${i}`,p,val,baseVal,ro,`tuneRange-${i}`);else if(p.kind==='string')control=paramSelectHtml(`tuneField-${i}`,p,val,baseVal,ro);else control=`<input id="tuneField-${i}" data-auto-index="${i}" type="text" value="${escapeHtml(val??'')}" ${ro?'readonly':''}>`;html+=`<div class="tuning-field ${ro?'readonly':''} ${pending?'draft':''}" data-field-index="${i}"><div class="tuning-field-head"><label>${escapeHtml(p.label||p.key||'Parameter')}</label><div class="tuning-badges"><span>${isYaml?escapeHtml(p.yamlFileKey)+' • YAML':'RUN / GT'} ${ro?'• LOCKED':''}</span>${isYaml?`<em id="tuneRuntime-${i}" class="runtime-badge">${ro?'LOCKED':'READY'}</em>`:''}${isYaml&&baseVal!==undefined?`<em class="runtime-badge baseline" title="Baseline awal: ${escapeHtml(baseFull)}">BASE ${escapeHtml(baseShort)}</em>`:''}</div></div><div class="tuning-input-row">${control}${ro?'':`<button class="param-save" data-field-index="${i}">${isYaml?(pending?'STAGED':'STAGE'):'SET'}</button>`}</div><code>${escapeHtml(p.yamlPath||p.key||'run identity')}</code></div>`}html+='</section>'}
  $('tuningFields').innerHTML=html;qa('.param-save',$('tuningFields')).forEach(b=>b.onclick=()=>saveTuningField(+b.dataset.fieldIndex));fields.forEach((p,i)=>{const e=$(`tuneField-${i}`);if(!e)return;if(p.kind==='int'||p.kind==='float')bindParamNumber(e,$(`tuneRange-${i}`),()=>saveTuningField(i,true));else e.onchange=()=>{if(e.type==='checkbox'){const b=q('b',e.parentElement);if(b)b.textContent=e.checked?'TRUE':'FALSE'}saveTuningField(i,true)}});decorateTuningFieldStates(fields);updateTuningDraftControls();decoratePanelHelp();window.navigationTuningPolicy?.render?.()
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
function tuningDraftRows(){if(!selectedExperiment)return[];const ids=new Set(tuningFieldsFor(selectedExperiment).filter(p=>p.yamlFileKey).map(p=>`${p.yamlFileKey}:${p.yamlPath}`));return [...configPending.values()].filter(x=>ids.has(`${x.fileKey}:${x.path}`))}
function tuningLevelFor(){return'all'}
function decorateTuningFieldStates(fields){fields.forEach((p,i)=>{const row=q(`.tuning-field[data-field-index="${i}"]`);if(!row)return;row.dataset.paramLevel=tuningLevelFor(p,i);const identity=p.yamlFileKey?`${p.yamlFileKey}:${p.yamlPath}`:`run:${selectedExperiment?.id||'none'}:${p.key||i}`,yaml=p.yamlFileKey?configValue(p.yamlFileKey,p.yamlPath):undefined,base=p.yamlFileKey?baselineConfigValue(p.yamlFileKey,p.yamlPath):undefined,draft=p.yamlFileKey?configPending.get(identity)?.value:undefined,runtime=p.yamlFileKey?(configRuntimeState.get(identity)?.label||'—'):'RUN / GT';if(p.yamlFileKey){let line=q('.param-state-line',row);if(!line){line=document.createElement('div');line.className='param-state-line';row.appendChild(line)}line.innerHTML=`<span>BASE <b>${escapeHtml(configValueText(base))}</b></span><span>YAML <b>${escapeHtml(configValueText(yaml))}</b></span><span>DRAFT <b>${escapeHtml(configValueText(draft))}</b></span><span>RUNTIME <b>${escapeHtml(runtime)}</b></span>`}const key=`param:${identity}`;registerParamHelp(key,p,{baseline:base,yaml,draft,runtime});if(!q('.help-trigger',row))row.insertAdjacentHTML('beforeend',`<button class="help-trigger" type="button" data-help-key="${escapeHtml(key)}" aria-label="Bantuan ${escapeHtml(p.label||p.yamlPath||p.key||'parameter')}">?</button>`)})}
function updateTuningDraftControls(){const items=tuningDraftRows(),n=items.length,ro=obj('server').read_only===true,busy=!!recordStartedMs,validated=validationMatches(items);setText('tuningDraftCount',`${n} DRAFT${validated?' • VALIDATED':''}`);const v=$('tuningValidateDraft'),a=$('tuningApplyDraft'),r=$('tuningRevertDraft');if(v)v.disabled=!n;if(a)a.disabled=!n||ro||busy||!validated;if(r)r.disabled=!n}
async function validateTuningDrafts(){const items=tuningDraftRows();if(!items.length)return toast('Tidak ada draft tuning.',true);try{const j=await validateConfigItems(items);toast(`Validate OK: ${items.length} draft • review diff sebelum Apply`)}catch(e){toast('Validate gagal: '+e.message,true)}}
function revertTuningDrafts(){const items=tuningDraftRows();for(const x of items)configPending.delete(`${x.fileKey}:${x.path}`);invalidateConfigValidation('draft reverted');renderTuningFields();renderConfigBrowser();window.navigationTuningPolicy?.render?.();toast(`${items.length} draft tuning dibuang; YAML tidak berubah.`)}
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
setInterval(()=>{
  if(document.hidden)return;
  if(activePage==='experiments'||recordStartedMs||(++recorderIdlePoll%5===0))pollRecorder();
},1000);

// FIRDA 4.2.1 one-page helper. Reuses the existing Workbench controls and recorder.
let firdaKpPulseCount=0,firdaKpPulseBusy=false,firdaKpAutoBusy=false,firdaKpAbort=false;
function firdaKpActive(){return currentExp==='steering'&&String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)==='4.2.1'}
function firdaKpTune(){const t=obj('vesc_tuning_state');return (+t.motor===1&&+t.status===0)?t:null}
function firdaKpSetText(id,v){const e=$(id);if(e)e.textContent=v}
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
    await firdaKpCommand('TUNING:GET:1');
    if(!await firdaKpWait(()=>!!firdaKpTune(),2500))throw new Error('Read LEFT tuning gagal');
    const t=firdaKpTune(),keys=['foc_q_ki','foc_d_kp','foc_d_ki','speed_kp','speed_ki','speed_kd','pos_kp','pos_ki','pos_kd','current_filter'];
    if(!keys.every(k=>Number.isFinite(+t[k])))throw new Error('Readback tuning tidak lengkap');
    const vals=[kp,...keys.map(k=>+t[k])];
    firdaKpSetText('firdaKpApplyState','Apply Q Kp '+kp.toFixed(3)+' • runtime only…');
    await firdaKpCommand('TUNING:SET:1:'+vals.join(':')+':0');
    await sleep(250);await firdaKpCommand('TUNING:GET:1');await sleep(250);
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
    await firdaKpCommand('TUNING:GET:1');
    if(!await firdaKpWait(()=>!!firdaKpTune(),2500))throw new Error('Read LEFT tuning gagal');
    const t=firdaKpTune(),keys=['foc_d_kp','foc_d_ki','speed_kp','speed_ki','speed_kd','pos_kp','pos_ki','pos_kd','current_filter'];
    if(!keys.every(k=>Number.isFinite(+t[k])))throw new Error('Readback tuning tidak lengkap');
    const vals=[FIRDA_Q_KP_FINAL,ki,...keys.map(k=>+t[k])];
    firdaKpSetText('firdaKiState','Apply Q Kp 0.80013 + Q Ki '+ki.toFixed(1)+' • runtime only…');
    await firdaKpCommand('TUNING:SET:1:'+vals.join(':')+':0');
    await sleep(250);await firdaKpCommand('TUNING:GET:1');await sleep(250);
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
const FIRDA_Q_KI_FINAL=75.0;
let firdaPiPulseCount=0,firdaPiAutoBusy=false,firdaPiAbort=false;
function firdaPiActive(){return currentExp==='steering'&&String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)==='4.2.3'}
function firdaPiMeta(iq){
  const s=Number(iq).toFixed(1);
  return ['PI-IQ'+s+'A','KP0.80013_KI75'];
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
  firdaPiAutoBusy=true;firdaPiAbort=false;firdaPiPulseCount=0;firdaPiApplyIdentity(iq);if($('runSampleRate'))$('runSampleRate').value='50';renderFirdaPiQuickPanel();
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
    firdaKpSetText('firdaPiState','SELESAI • '+firdaPiMeta(iq)[0]+' • 3 pulse tersimpan • gain final Kp 0.80013 Ki 75');
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
function firdaPosKpActive(){return currentExp==='steering'&&String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)==='4.3.1'}
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
}
function firdaPosPhysicalFromVesc(v){return Number.isFinite(+v)?((+v-180)/6):NaN}
function renderFirdaPosKpQuickPanel(){
  const p=$('firdaPosKpQuickPanel');if(!p)return;
  const active=firdaPosKpActive();p.hidden=!active;if(!active)return;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$('firdaPosKpModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaPosActual'))$('firdaPosActual').value=t?(fmt(t.pos_kp,6)+' / '+fmt(t.pos_ki,6)+' / '+fmt(t.pos_kd,6)):'-- / -- / --';
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?actual-target:NaN;
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
  if(s==='4.3.2')return {panel:'firdaPosKiQuickPanel',chip:'firdaPosKiModeState',state:'firdaPosKiState',live:'firdaPosKiLive',count:'firdaPosKiCycleCount',actual:'firdaPosKiActual',auto:'firdaPosKiAuto',cycles:3};
  if(s==='4.3.3')return {panel:'firdaPosKdQuickPanel',chip:'firdaPosKdModeState',state:'firdaPosKdState',live:'firdaPosKdLive',count:'firdaPosKdCycleCount',actual:'firdaPosKdActual',auto:'firdaPosKdAuto',cycles:3};
  if(s==='4.3.4')return {panel:'firdaPosFinalQuickPanel',chip:'firdaPosFinalModeState',state:'firdaPosFinalState',live:'firdaPosFinalLive',count:'firdaPosFinalCycleCount',actual:'firdaPosFinalActual',auto:'firdaPosFinalAuto',cycles:5};
  return null;
}
function renderFirdaPosAdvancedPanels(){
  for(const id of ['firdaPosKiQuickPanel','firdaPosKdQuickPanel','firdaPosFinalQuickPanel'])if($(id))$(id).hidden=true;
  const c=firdaPosPanelCfg();if(!c)return;
  const p=$(c.panel);if(!p)return;p.hidden=false;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=obj('vesc_left_values'),cmd=obj('vesc_command_state'),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$(c.chip);if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($(c.actual))$(c.actual).value=t?(fmt(t.pos_kp,6)+' / '+fmt(t.pos_ki,6)+' / '+fmt(t.pos_kd,6)):'-- / -- / --';
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?actual-target:NaN;
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
  if(s==='4.3.2')return {kp:+$('firdaPosKiFixedKp').value,ki:+$('firdaPosKiValue').value,kd:0,variation:'POSKI-'+(+$('firdaPosKiValue').value).toFixed(4),candidate:'candidate-a',cycles:3};
  if(s==='4.3.3')return {kp:+$('firdaPosKdFixedKp').value,ki:+$('firdaPosKdFixedKi').value,kd:+$('firdaPosKdValue').value,variation:'POSKD-'+(+$('firdaPosKdValue').value).toFixed(4),candidate:'candidate-a',cycles:3};
  if(s==='4.3.4'){const kp=+$('firdaPosFinalKp').value,ki=+$('firdaPosFinalKi').value,kd=+$('firdaPosFinalKd').value;return {kp,ki,kd,variation:'POS-FINAL-KP'+kp.toFixed(4)+'-KI'+ki.toFixed(4)+'-KD'+kd.toFixed(4),candidate:'final',cycles:5};}
  return null;
}
async function firdaPosAdvRun(){
  if(firdaPosAdv.busy)return;
  const c=firdaPosPanelCfg(),p=firdaPosAdvParams();if(!c||!p)return;
  if(![p.kp,p.ki,p.kd].every(Number.isFinite)||p.kp<=0||p.ki<0||p.kd<0)return toast('Gain posisi tidak valid.',true);
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaPosAdv.busy=true;firdaPosAdv.abort=false;firdaPosAdv.cycles=0;
  if($('runVariation'))$('runVariation').value=p.variation;
  if($('runCondition'))$('runCondition').value='STEP_PM5DEG';
  if($('runCandidate'))$('runCandidate').value=p.candidate;
  if($('runSampleRate'))$('runSampleRate').value='50';
  renderFirdaPosAdvancedPanels();
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
    firdaKpSetText(c.state,'SELESAI • '+p.variation+' • '+p.cycles+' cycle tersimpan • kembali RUNTIME');toast(firdaPosStep()+' '+p.variation+' selesai',false);
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
const FIRDA_POS_FINAL={kp:0.0375,ki:0.0025,kd:0};
const firdaResp={busy:false,abort:false,cycles:0};
function firdaResponseCfg(){
  const s=firdaPosStep();
  if(s==='4.4.1')return {title:'4.4.1 Step ±10°',variation:'STEP-PM10',condition:'FINAL_PID',cycles:3,moves:[[240,'+10°'],[180,'0°'],[120,'−10°'],[180,'0°']],hint:'Step +10° / 0° / −10° / 0° sebanyak 3 cycle.'};
  if(s==='4.4.2')return {title:'4.4.2 Step ±20°',variation:'STEP-PM20',condition:'FINAL_PID',cycles:3,moves:[[300,'+20°'],[180,'0°'],[60,'−20°'],[180,'0°']],hint:'Step +20° / 0° / −20° / 0° sebanyak 3 cycle.'};
  if(s==='4.4.3')return {title:'4.4.3 Repeatability',variation:'REPEAT-PM20',condition:'FINAL_PID',cycles:5,moves:[[300,'+20°'],[180,'0°'],[60,'−20°'],[180,'0°']],hint:'Repeatability LEFT → CENTER → RIGHT → CENTER, ±20°, sebanyak 5 cycle.'};
  if(s==='4.4.4')return {title:'4.4.4 Simetri Kiri / Kanan',variation:'SYMMETRY-10-20',condition:'FINAL_PID',cycles:3,moves:[[240,'+10°'],[180,'0°'],[120,'−10°'],[180,'0°'],[300,'+20°'],[180,'0°'],[60,'−20°'],[180,'0°']],hint:'Bandingkan +10°/−10° dan +20°/−20° sebanyak 3 paired cycle.'};
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
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?actual-target:NaN;
  firdaKpSetText('firdaResponseLive','Target '+fmt(target,2)+'° • Actual '+fmt(actual,2)+'° • Error '+fmt(err,2)+'° • Iq '+fmt(lv.iq_a,2)+' A');
  firdaKpSetText('firdaResponseCount','Cycle: '+firdaResp.cycles+' / '+c.cycles);
  if($('firdaResponseAuto')){$('firdaResponseAuto').disabled=rec||firdaResp.busy;$('firdaResponseAuto').textContent=firdaResp.busy?'PENGUJIAN BERJALAN…':'START PENGUJIAN OTOMATIS • '+c.cycles+' CYCLE'}
}
async function firdaResponseRun(){
  if(firdaResp.busy)return;
  const c=firdaResponseCfg();if(!c)return;
  if($('recordToggleBtn')?.dataset.active==='true')return toast('STOP recording lama dulu.',true);
  firdaResp.busy=true;firdaResp.abort=false;firdaResp.cycles=0;
  if($('runVariation'))$('runVariation').value=c.variation;
  if($('runCondition'))$('runCondition').value=c.condition;
  if($('runCandidate'))$('runCandidate').value='final';
  if($('runSampleRate'))$('runSampleRate').value='50';
  renderFirdaResponsePanel();
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
        await firdaPosPoll(target===180?1400:2200,70);
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
setInterval(()=>{if(/^4\.4\.[1-4]$/.test(firdaPosStep()))renderFirdaResponsePanel()},250);

// FIRDA 4.5 load test: wheels on ground, vehicle stationary, no joystick commands during recording.
const firdaLoad={busy:false,abort:false,cycles:0};
function firdaLoadCfg(){
  const s=firdaPosStep(),m={'4.5.1':0,'4.5.2':1,'4.5.3':2,'4.5.4':3};
  if(!(s in m))return null;
  const n=m[s];
  return {galon:n,variation:'LOAD-'+n+'GALON',condition:n+'_GALON_WHEELS_GROUND',cycles:3,moves:[[60,'−20°'],[180,'0°'],[300,'+20°'],[180,'0°']]};
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
  const target=String(cmd.mode||'')==='POS'?firdaPosPhysicalFromVesc(cmd.value):NaN,actual=firdaPosPhysicalFromVesc(lv.position_deg),err=Number.isFinite(target)&&Number.isFinite(actual)?actual-target:NaN;
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
  const c=firdaLoadCfg();if(!c)return;
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
