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
function renderSelectedExperiment(){const x=selectedExperiment;if(!x)return;const domain=currentExp==='navigation'?'Navigasi':currentExp==='perception'?'Persepsi':'ESC / FOC',id=displayExperimentId(x),title=displayExperimentTitle(x),report=x.reportMode===true;setText('experimentGroup',displayExperimentGroup(x));setText('experimentTitle',`${id} ${title}`);const n0=currentExp==='navigation'&&x.id==='N0.1',ekfReport=currentExp==='navigation'&&/^R4\.[23]\.[1-5]$/.test(String(x.id||''));setText('experimentSubtitle',report?(ekfReport?`${domain} • LAPORAN BAB IV ${id} • YAML source-of-truth → safe runtime apply/read-back → data aktual berjalan • setiap kondisi diulang 3 cycle dan dibandingkan overall.`:`${domain} • LAPORAN BAB IV • rekap data dari ${x.sourceIds?.join(', ')||'source tuning'} • START/STOP, Excel, grafik, dan parameter YAML memakai mesin akuisisi Navigasi yang sama.`):n0?'Navigasi • N0 Timing Readiness • target konfigurasi dan rate aktual dipisahkan. ESC command-loop target bukan rate /esc/odom feedback.':`${domain} • Tuning ${id} • parameter, tabel, grafik, dan live evidence mengikuti source pengujian ini.`);if(activePage==='experiments'){setText('pageEyebrow',report?'NAVIGASI • LAPORAN BAB IV':`${domain.toUpperCase()} • TUNING`);setText('pageTitle',`${id} ${title}`)}syncReportDocumentLayout();const imuLinearityExtra=report&&x.id==='R4.1.2';setText('experimentTableBadge',`${(x.tableNames||[]).length+(imuLinearityExtra?1:0)} TABLE`);setText('experimentGraphBadge',`${(x.graphCaptions||[]).length+(imuLinearityExtra?2:0)} GRAPH`);setText('experimentRawMetadata',JSON.stringify(x,null,2));renderTuningFields();renderLabTabs();renderExperimentTable();renderExperimentGraphs();window.renderImuHeadingLinearity?.();window.renderReport41TimedPanel?.();renderContextVisual();renderSourceAudit();renderN0TimingGuide();renderPerceptionEngineering();renderTrialGuideDetailed();renderTrialRecap();renderCommissioningRoadmap();renderQualificationPanel();window.renderEkfReportRuntimePanel?.();window.navigationTuningPolicy?.render?.();renderFirdaKpQuickPanel()}
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
let firdaKpPulseCount=0,firdaKpPulseBusy=false;
function firdaKpActive(){return currentExp==='steering'&&String(selectedExperiment?.id||'')==='4.2.1'&&String(typeof activeFirdaEscStep==='undefined'?'':activeFirdaEscStep)==='4.2.1'}
function firdaKpTune(){const t=obj('vesc_tuning_state');return (+t.motor===1&&+t.status===0)?t:null}
function firdaKpSetText(id,v){const e=$(id);if(e)e.textContent=v}
function firdaKpMeta(v){if(Math.abs(v-.4)<.08)return['KP-05X','candidate-a'];if(Math.abs(v-.8)<.12)return['KP-10X','candidate-b'];if(Math.abs(v-1.2)<.15)return['KP-15X','candidate-c'];return['KP-'+Number(v).toFixed(3),'candidate-a']}
function firdaKpApplyIdentity(v){const [variation,candidate]=firdaKpMeta(v);if($('runVariation'))$('runVariation').value=variation;if($('runCondition'))$('runCondition').value='IQ_STEP_1A';if($('runCandidate'))$('runCandidate').value=candidate}
function renderFirdaKpQuickPanel(){
  const p=$('firdaKpQuickPanel');if(!p)return;const active=firdaKpActive();p.hidden=!active;if(!active)return;
  const maint=bool(raw('vesc_maintenance_active')),t=firdaKpTune(),lv=obj('vesc_left_values'),rec=$('recordToggleBtn')?.dataset.active==='true';
  const chip=$('firdaKpModeState');if(chip){chip.textContent=maint?'WEB MAINTENANCE':'RUNTIME';chip.className='status-chip '+(maint?'':'waiting')}
  if($('firdaKpActual'))$('firdaKpActual').value=t?fmt(t.foc_q_kp,6):'--';
  if($('firdaKiActual'))$('firdaKiActual').value=t?fmt(t.foc_q_ki,6):'--';
  firdaKpSetText('firdaKpLive','Iq '+fmt(lv.iq_a,3)+' A • Id '+fmt(lv.id_a,3)+' A • Imotor '+fmt(lv.current_motor_a,3)+' A • Duty '+fmt(lv.duty,4)+' • Vbus '+fmt(lv.vbus_v,2)+' V');
  firdaKpSetText('firdaKpPulseCount','Pulse: '+firdaKpPulseCount+' / 3');
  if($('firdaKpEnterMaint')){$('firdaKpEnterMaint').disabled=maint||firdaKpPulseBusy;$('firdaKpEnterMaint').textContent=maint?'1. MAINTENANCE ACTIVE':'1. WEB MAINTENANCE'}
  if($('firdaKpRead'))$('firdaKpRead').disabled=!maint||firdaKpPulseBusy;
  if($('firdaKpApply'))$('firdaKpApply').disabled=!maint||!t||firdaKpPulseBusy;
  if($('firdaKpRecordStart'))$('firdaKpRecordStart').disabled=!maint||rec||firdaKpPulseBusy;
  if($('firdaKpPulse'))$('firdaKpPulse').disabled=!maint||!rec||firdaKpPulseBusy||firdaKpPulseCount>=3;
  if($('firdaKpRecordStop'))$('firdaKpRecordStop').disabled=!rec||firdaKpPulseBusy;
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
  $('firdaKpRecordStart').onclick=()=>{firdaKpPulseCount=0;$('recordToggleBtn')?.click()};
  $('firdaKpRecordStop').onclick=()=>{$('recordToggleBtn')?.click()};
  $('firdaKpStop').onclick=()=>{$('vescSendZero')?.click()};
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
