const sleep=ms=>new Promise(r=>setTimeout(r,ms));
const targets=await fetch('http://127.0.0.1:9222/json').then(r=>r.json());
const page=targets.find(x=>x.type==='page');
if(!page) throw new Error('Chrome page target tidak ditemukan');
const ws=new WebSocket(page.webSocketDebuggerUrl);
await new Promise((res,rej)=>{ws.onopen=res;ws.onerror=rej});
let seq=0;const pending=new Map();const exceptions=[];
ws.onmessage=e=>{
  const m=JSON.parse(e.data);
  if(m.id&&pending.has(m.id)){const p=pending.get(m.id);pending.delete(m.id);m.error?p.reject(m.error):p.resolve(m.result);return}
  if(m.method==='Runtime.exceptionThrown')exceptions.push(m.params?.exceptionDetails?.text||'exception');
};
const send=(method,params={})=>new Promise((resolve,reject)=>{
  const id=++seq;pending.set(id,{resolve,reject});ws.send(JSON.stringify({id,method,params}));
});
const ev=async expression=>{
  const r=await send('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
  return r.result?.value;
};
await send('Runtime.enable');await sleep(1800);
const result={};
result.initial=await ev('({readyState:document.readyState,title:document.title})');
result.runtime=await ev(`({
  v3:window.__BAB4_PERCEPTION_V3__===true,
  ids:typeof V3_IDS==='undefined'?[]:V3_IDS,
  state:typeof V3==='undefined'?null:{armed:V3.armed,tab45:V3.tab45},
  funcs:{render:typeof v3Render,post:typeof v3Post,capture:typeof v3Capture,exporter:typeof v3Export}
})`);
await sleep(700);
result.exceptions=exceptions;
console.log(JSON.stringify(result,null,2));
ws.close();
