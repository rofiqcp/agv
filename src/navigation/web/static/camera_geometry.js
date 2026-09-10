'use strict';

(function(global){
  class CameraGeometry {
    constructor(opts={}){this.update(opts)}
    update(opts={}){
      this.naturalWidth=Math.max(0,+opts.naturalWidth||this.naturalWidth||0);
      this.naturalHeight=Math.max(0,+opts.naturalHeight||this.naturalHeight||0);
      this.rect=opts.rect||this.rect||{left:0,top:0,width:0,height:0};
      this.mirrorX=opts.mirrorX!==undefined?!!opts.mirrorX:!!this.mirrorX;
      this.dpr=Math.max(1,+opts.dpr||this.dpr||1);return this;
    }
    contentRect(){
      const r=this.rect,nw=this.naturalWidth,nh=this.naturalHeight;
      if(!(r&&r.width>0&&r.height>0&&nw>0&&nh>0))return null;
      const scale=Math.min(r.width/nw,r.height/nh),width=nw*scale,height=nh*scale;
      return {left:r.left+(r.width-width)/2,top:r.top+(r.height-height)/2,width,height,scale};
    }
    screenToImage(clientX,clientY){
      const c=this.contentRect();if(!c)return null;
      let nx=(clientX-c.left)/c.width,ny=(clientY-c.top)/c.height;
      if(nx<0||nx>1||ny<0||ny>1)return null;
      if(this.mirrorX)nx=1-nx;
      return {x:nx*Math.max(0,this.naturalWidth-1),y:ny*Math.max(0,this.naturalHeight-1),nx,ny};
    }
    imageToScreen(x,y){
      const c=this.contentRect();if(!c||this.naturalWidth<=1||this.naturalHeight<=1)return null;
      let nx=Math.max(0,Math.min(1,+x/(this.naturalWidth-1))),ny=Math.max(0,Math.min(1,+y/(this.naturalHeight-1)));
      if(this.mirrorX)nx=1-nx;
      return {x:c.left+nx*c.width,y:c.top+ny*c.height,nx,ny};
    }
    normalizedToScreen(nx,ny){
      return this.imageToScreen((+nx||0)*Math.max(0,this.naturalWidth-1),(+ny||0)*Math.max(0,this.naturalHeight-1));
    }
    screenToNormalized(x,y){const p=this.screenToImage(x,y);return p?{x:p.nx,y:p.ny}:null}
    resizeCanvas(canvas){
      const r=canvas.getBoundingClientRect(),dpr=Math.max(1,Math.min(3,global.devicePixelRatio||this.dpr||1));
      const w=Math.max(1,Math.round(r.width*dpr)),h=Math.max(1,Math.round(r.height*dpr));
      if(canvas.width!==w)canvas.width=w;if(canvas.height!==h)canvas.height=h;
      return {ctx:canvas.getContext('2d'),width:w,height:h,dpr,cssWidth:r.width,cssHeight:r.height};
    }
    static distancePointSegment(px,py,ax,ay,bx,by){
      const dx=bx-ax,dy=by-ay,l2=dx*dx+dy*dy;if(l2<=1e-12)return Math.hypot(px-ax,py-ay);
      const t=Math.max(0,Math.min(1,((px-ax)*dx+(py-ay)*dy)/l2)),x=ax+t*dx,y=ay+t*dy;return Math.hypot(px-x,py-y);
    }
    hitTestScreen(clientX,clientY,points,{handlePx=12,edgePx=8,closed=false}={}){
      const screen=points.map((p,i)=>({i,p,s:this.normalizedToScreen(p.x,p.y)})).filter(x=>x.s);
      let best=null;for(const x of screen){const d=Math.hypot(clientX-x.s.x,clientY-x.s.y);if(d<=handlePx&&(!best||d<best.distance))best={kind:'handle',index:x.i,distance:d}}
      if(best)return best;
      const n=screen.length,limit=closed?n:Math.max(0,n-1);for(let i=0;i<limit;i++){const a=screen[i],b=screen[(i+1)%n];if(!a||!b)continue;const d=CameraGeometry.distancePointSegment(clientX,clientY,a.s.x,a.s.y,b.s.x,b.s.y);if(d<=edgePx&&(!best||d<best.distance))best={kind:'edge',index:i,distance:d}}
      return best;
    }
  }

  class CameraFrameStore {
    constructor({url='/api/camera.jpg',intervalMs=250,canFetch=()=>true}={}){
      this.url=url;this.intervalMs=Math.max(100,intervalMs);this.canFetch=canFetch;this.targets=new Map();this.timer=0;this.busy=false;this.lastUrl='';this.lastFrame=null;
    }
    subscribe(key,{img,placeholder=null,isActive=()=>true,onFrame=null}={}){
      this.targets.set(key,{img,placeholder,isActive,onFrame});this.start();return()=>{this.targets.delete(key);if(!this.targets.size)this.stop()};
    }
    start(){if(!this.timer)this.timer=global.setInterval(()=>this.tick(),this.intervalMs);this.tick()}
    stop(){if(this.timer)global.clearInterval(this.timer);this.timer=0}
    activeTargets(){return [...this.targets.values()].filter(t=>t.img&&t.isActive())}
    async tick(){
      if(this.busy||!this.canFetch())return;const targets=this.activeTargets();if(!targets.length)return;this.busy=true;
      try{
        const r=await fetch(`${this.url}?frame=${Date.now()}`,{cache:'no-store'});if(!r.ok)throw new Error(`camera HTTP ${r.status}`);const blob=await r.blob(),url=URL.createObjectURL(blob),probe=new Image();
        await new Promise((resolve,reject)=>{probe.onload=resolve;probe.onerror=reject;probe.src=url});
        const frame={url,width:probe.naturalWidth,height:probe.naturalHeight,atMs:Date.now(),bytes:blob.size};
        for(const t of targets){t.img.src=url;t.img.style.display='block';if(t.placeholder)t.placeholder.style.display='none';if(t.onFrame)t.onFrame(frame,t.img)}
        const old=this.lastUrl;this.lastUrl=url;this.lastFrame=frame;if(old&&old!==url)global.setTimeout(()=>URL.revokeObjectURL(old),1000);
      }catch(_){for(const t of targets){if(t.placeholder)t.placeholder.style.display='grid'}}finally{this.busy=false}
    }
  }
  global.AGVCameraGeometry=CameraGeometry;global.AGVCameraFrameStore=CameraFrameStore;
})(window);
