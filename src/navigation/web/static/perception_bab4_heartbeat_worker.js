/* Dedicated BAB IV 4.5 ARMED heartbeat worker.
 * Runs independently from camera/costmap/UI rendering on the page main thread.
 */
let active=false, token='', sessionId='', mode='', condition='', timer=null, busy=false, seq=0;

function stopWorker(){
  active=false;
  if(timer){clearInterval(timer);timer=null;}
  busy=false;
}

async function beat(){
  if(!active||busy||!token||!sessionId)return;
  busy=true;
  const controller=new AbortController();
  const timeout=setTimeout(()=>controller.abort(),1600);
  const sequence=++seq;
  try{
    const response=await fetch('/api/perception/final-trial/heartbeat',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({
        token,
        session_id:sessionId,
        transport:'web_worker',
        sequence
      }),
      signal:controller.signal,
      cache:'no-store'
    });
    let data={};
    try{data=await response.json()}catch(_){}
    if(!response.ok||!data.ok){
      postMessage({
        type:'heartbeat_error',
        sequence,
        code:data.code||'',
        message:data.message||('HTTP '+response.status)
      });
      return;
    }
    postMessage({type:'heartbeat_ok',sequence,trial:data.trial||{}});
  }catch(err){
    postMessage({
      type:'heartbeat_error',
      sequence,
      code:err&&err.name==='AbortError'?'TIMEOUT':'NETWORK',
      message:err&&err.name==='AbortError'?'worker heartbeat timeout':String((err&&err.message)||err||'network error')
    });
  }finally{
    clearTimeout(timeout);
    busy=false;
  }
}

self.onmessage=(event)=>{
  const m=event.data||{};
  if(m.type==='start'){
    stopWorker();
    token=String(m.token||'');
    sessionId=String(m.session_id||'');
    mode=String(m.mode||'');
    condition=String(m.condition||'');
    seq=0;
    active=!!token&&!!sessionId;
    if(active){
      beat();
      timer=setInterval(beat,500);
    }
    return;
  }
  if(m.type==='stop'){
    stopWorker();
  }
};

self.onclose=stopWorker;
