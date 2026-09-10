'use strict';

(function(global){
  const session={
    source:'LIVE',
    cursor_time:null,
    range_start:null,
    range_end:null,
    follow_live:true,
    selected_task:null,
    selected_trial:null,
    selected_event:null,
    metric_preset:null,
    nearest_evidence:null,
    replayAdapter:null,
    setSource(source){
      if(!['LIVE','CURRENT_RECORDING','REPLAY'].includes(source))return;
      this.source=source;this.follow_live=source!=='REPLAY';
      global.dispatchEvent(new CustomEvent('agv-analysis-source',{detail:{source}}));
    },
    registerReplayAdapter(adapter){this.replayAdapter=adapter||null},
    setCursor(time,origin='ui'){
      const t=Number(time);if(!Number.isFinite(t))return;
      this.cursor_time=Math.max(0,t);this.follow_live=false;
      global.dispatchEvent(new CustomEvent('agv-analysis-cursor',{detail:{time:this.cursor_time,origin,source:this.source}}));
      if(this.source==='REPLAY'&&origin!=='replay'&&this.replayAdapter?.seekTime)this.replayAdapter.seekTime(this.cursor_time);
    },
    syncReplay(time,index){
      const t=Number(time);if(!Number.isFinite(t))return;
      this.source='REPLAY';this.cursor_time=Math.max(0,t);this.follow_live=false;
      global.dispatchEvent(new CustomEvent('agv-analysis-cursor',{detail:{time:this.cursor_time,index,origin:'replay',source:'REPLAY'}}));
    },
    syncLive(time){
      if(this.source==='REPLAY'||!this.follow_live)return;
      const t=Number(time);if(!Number.isFinite(t))return;
      this.cursor_time=Math.max(0,t);
    },
    selectTask(subsystem,id){this.selected_task={subsystem,id}},
    selectTrial(id){this.selected_trial=id||null},
    selectEvent(event){this.selected_event=event||null;if(event?.analysis_time_s!=null)this.setCursor(event.analysis_time_s,'event')},
    setEvidenceIndex(items=[]){this.evidenceIndex=(items||[]).filter(x=>Number.isFinite(+x.timestamp_ms)).sort((a,b)=>a.timestamp_ms-b.timestamp_ms)},
    nearestEvidenceAt(timestampMs,toleranceMs=750){
      if(!this.evidenceIndex?.length||!Number.isFinite(+timestampMs))return null;
      let best=null,delta=Infinity;for(const e of this.evidenceIndex){const d=Math.abs(+e.timestamp_ms-+timestampMs);if(d<delta){best=e;delta=d}}
      this.nearest_evidence=best&&delta<=toleranceMs?{...best,delta_ms:delta}:null;return this.nearest_evidence;
    },
    snapshot(){return {source:this.source,cursor_time:this.cursor_time,range_start:this.range_start,range_end:this.range_end,follow_live:this.follow_live,selected_task:this.selected_task,selected_trial:this.selected_trial,selected_event:this.selected_event,metric_preset:this.metric_preset,nearest_evidence:this.nearest_evidence}}
  };
  global.analysisSession=session;
})(window);
