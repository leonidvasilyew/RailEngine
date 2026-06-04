"""Визуализация графа и движения поездов с отрезком поезда."""

import json, webbrowser, os
from topology import Topology, INF
from pathfinder import PathResult, Train


def _ser_topo(topo):
    nodes = {}
    for nid, n in topo._nodes.items():
        nodes[str(nid)] = {'id': nid, 'name': n.name,
                           'edges': [e if e is not None else -1 for e in n.edges],
                           'port_count': n.port_count,
                           'type': type(n).__name__}
    edges = {}
    for eid, e in topo._edges.items():
        edges[str(eid)] = {'id': eid, 'name': e.name,
                           'node_a': e.node_a, 'node_b': e.node_b,
                           'length': e.length}
    return {'nodes': nodes, 'edges': edges}


def _ser_train(result, train, tid, color):
    entries = []
    for s in result.schedule:
        entries.append({
            'edge_id': s.edge_id, 's_enter': s.s_enter, 's_exit': s.s_exit,
            't_enter': s.t_enter, 't_exit': s.t_exit,
            'v_enter': s.v_enter, 'v_peak': s.v_peak, 'v_exit': s.v_exit,
            'direction': s.direction,
            'dist': s.dist,
        })
    return {'id': tid, 'color': color, 'total_time': result.total_time,
            'accel': min(train.accel, 1e9), 'decel': min(train.decel, 1e9),
            'length': train.length,
            'schedule': entries}


_HTML = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Railway</title>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<style>
*{margin:0;padding:0;box-sizing:border-box}
html,body{touch-action:none;overscroll-behavior:none}
body{background:#0a0e17;color:#c8d6e5;font-family:system-ui;overflow:hidden;height:100vh}
canvas{display:block;width:100%;height:100%;touch-action:none}
#hud{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);
  display:flex;gap:10px;align-items:center;background:rgba(15,20,35,.92);
  border:1px solid rgba(100,120,160,.3);border-radius:12px;padding:8px 18px;z-index:10}
#hud button{background:rgba(100,120,160,.2);border:1px solid rgba(100,120,160,.4);
  color:#c8d6e5;font:13px system-ui;padding:5px 10px;border-radius:6px;cursor:pointer;
  display:flex;align-items:center;justify-content:center;min-width:36px;height:32px}
#hud button:hover{background:rgba(100,120,160,.4)}
#hud button svg{width:14px;height:14px;fill:currentColor}
#td{font:bold 15px system-ui;color:#ecf0f1;min-width:80px;text-align:center;font-variant-numeric:tabular-nums}
#sd{font:11px system-ui;color:#7f8c8d;min-width:28px}
input[type=range]{width:180px;accent-color:#3498db}
#info{position:fixed;top:14px;left:14px;font:11px system-ui;color:#576574;line-height:1.6;z-index:10}

/* Мобильная адаптация: всё в одну линию, но крупнее */
@media (max-width: 768px) and (orientation: portrait) {
  #hud{
    left:0; right:0; bottom:0; transform:none;
    width:100%;
    flex-wrap:wrap;
    justify-content:center;
    border-radius:16px 16px 0 0;
    padding:12px 12px env(safe-area-inset-bottom,12px) 12px;
    gap:10px;
  }
  #hud button{
    font-size:14px; padding:6px 10px; min-width:40px; height:36px;
  }
  #hud button svg{width:16px;height:16px}
  input[type=range]{
    flex-basis:100%; order:99; width:100%; height:32px;
  }
  #td{font-size:15px; min-width:70px}
  #sd{font-size:13px; min-width:32px}
  #info{font-size:12px; line-height:1.6; max-width:calc(100% - 28px)}
}
</style></head><body>
<div id="info"></div><canvas id="c"></canvas>
<div id="hud">
  <button id="bpp" title="play/pause">
    <svg id="bpp-icon" viewBox="0 0 24 24"><path d="M8 5v14l11-7z"/></svg>
  </button>
  <button id="br" title="restart">
    <svg viewBox="0 0 24 24"><path d="M6 6h2v12H6zm3.5 6l8.5 6V6z"/></svg>
  </button>
  <input type="range" id="sl" min="0" max="1000" value="0">
  <span id="td">0.00s</span><span id="sd">×1</span>
  <button id="bm" title="slower">−</button>
  <button id="bf" title="faster">+</button>
</div>
<script>
const D=__DATA_JSON__;
const T=D.topology, trains=D.trains;
const maxT=Math.max(...trains.map(t=>t.total_time),.001);
const cv=document.getElementById('c'), cx=cv.getContext('2d');
let W,H;
const DPR = window.devicePixelRatio || 1;
function resize(){
  W=innerWidth; H=innerHeight;
  cv.width=W*DPR; cv.height=H*DPR;
  cv.style.width=W+'px'; cv.style.height=H+'px';
  cx.setTransform(DPR,0,0,DPR,0,0);
}
resize();

// Мобильный режим: текст НЕМНОГО меньше, точки и рёбра ТОЛЩЕ
const isMobile = (window.matchMedia && window.matchMedia('(max-width: 768px)').matches) ||
                 (navigator.maxTouchPoints && navigator.maxTouchPoints > 0);
const txtScale = isMobile ? 1.1 : 1.0;
const nodeScale = isMobile ? 1.5 : 1.0;
const edgeScale = isMobile ? 1.5 : 1.0;

const nids=Object.keys(T.nodes).map(Number);
const eids=Object.keys(T.edges).map(Number);

// ── Layout ──
const pos={};
function hash(n){return(n*2654435761)>>>0}
nids.forEach(id=>{
  let h=hash(id);
  pos[id]={x:((h&0xffff)/0xffff-0.5)*300,y:(((h>>>16)&0xffff)/0xffff-0.5)*300,vx:0,vy:0};
});
const adjacent=new Set();
eids.forEach(eid=>{
  const e=T.edges[eid];
  adjacent.add(Math.min(e.node_a,e.node_b)+','+Math.max(e.node_a,e.node_b));
});
function isAdj(a,b){return adjacent.has(Math.min(a,b)+','+Math.max(a,b))}

// Тупиковые узлы (одно ребро) и их сосед — для выталкивания наружу
const degree={};
const deadEndNeighbor={};
nids.forEach(id=>{
  const n=T.nodes[id];
  const connected=n.edges.filter(e=>e>=0);
  degree[id]=connected.length;
  if(connected.length===1){
    const e=T.edges[connected[0]];
    deadEndNeighbor[id]=e.node_a===id?e.node_b:e.node_a;
  }
});

function segIntersect(x1,y1,x2,y2,x3,y3,x4,y4){
  const d=(x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
  if(Math.abs(d)<1e-10)return false;
  const t=((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/d;
  const u=-((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/d;
  return t>0.05&&t<0.95&&u>0.05&&u<0.95;
}

function layout(iter){
  for(let it=0;it<iter;it++){
    // (1) Отталкивание несоединённых узлов
    for(let i=0;i<nids.length;i++) for(let j=i+1;j<nids.length;j++){
      if(isAdj(nids[i],nids[j]))continue;
      const a=pos[nids[i]],b=pos[nids[j]];
      let dx=b.x-a.x,dy=b.y-a.y,d=Math.sqrt(dx*dx+dy*dy)||1;
      let f=10000/(d*d);
      a.vx-=dx/d*f;a.vy-=dy/d*f;b.vx+=dx/d*f;b.vy+=dy/d*f;
    }

    // (2) Пружины пропорциональной длины
    eids.forEach(eid=>{
      const e=T.edges[eid],a=pos[e.node_a],b=pos[e.node_b];
      let dx=b.x-a.x,dy=b.y-a.y,d=Math.sqrt(dx*dx+dy*dy)||1;
      let f=(d-e.length)*0.08;
      a.vx+=dx/d*f;a.vy+=dy/d*f;b.vx-=dx/d*f;b.vy-=dy/d*f;
    });

    // (3) Антипересечение рёбер: если два ребра без общего узла
    //     пересекаются, расталкиваем их узлы.
    for(let i=0;i<eids.length;i++) for(let j=i+1;j<eids.length;j++){
      const e1=T.edges[eids[i]], e2=T.edges[eids[j]];
      if(e1.node_a===e2.node_a||e1.node_a===e2.node_b||
         e1.node_b===e2.node_a||e1.node_b===e2.node_b) continue;
      const a1=pos[e1.node_a],b1=pos[e1.node_b];
      const a2=pos[e2.node_a],b2=pos[e2.node_b];
      if(segIntersect(a1.x,a1.y,b1.x,b1.y,a2.x,a2.y,b2.x,b2.y)){
        let mx1=(a1.x+b1.x)/2,my1=(a1.y+b1.y)/2;
        let mx2=(a2.x+b2.x)/2,my2=(a2.y+b2.y)/2;
        let dx=mx2-mx1,dy=my2-my1,d=Math.sqrt(dx*dx+dy*dy)||1;
        let f=50;  // сильное расталкивание
        a1.vx-=dx/d*f;a1.vy-=dy/d*f;b1.vx-=dx/d*f;b1.vy-=dy/d*f;
        a2.vx+=dx/d*f;a2.vy+=dy/d*f;b2.vx+=dx/d*f;b2.vy+=dy/d*f;
      }
    }

    // (4) Тупики выталкиваются наружу от их соседа.
    //     Сила обратно пропорциональна расстоянию — чем дальше тупик, тем
    //     слабее тянет. Это не даёт соседу тащиться слишком далеко.
    nids.forEach(id=>{
      if(degree[id]!==1)return;
      const nb=deadEndNeighbor[id];
      if(nb===undefined)return;
      const p=pos[id],q=pos[nb];
      let dx=p.x-q.x,dy=p.y-q.y;
      let d=Math.sqrt(dx*dx+dy*dy)||1;
      // Затухающая сила: на коротких расстояниях сильная, на длинных слабая
      let f=Math.min(2.0, 60/d);
      p.vx+=dx/d*f;p.vy+=dy/d*f;
    });

    // (5) Слабая центральная гравитация + затухание
    let gcx=0,gcy=0;
    nids.forEach(id=>{gcx+=pos[id].x;gcy+=pos[id].y});
    gcx/=nids.length||1;gcy/=nids.length||1;
    nids.forEach(id=>{
      pos[id].vx+=(gcx-pos[id].x)*0.0008;
      pos[id].vy+=(gcy-pos[id].y)*0.0008;
      pos[id].vx*=0.82;pos[id].vy*=0.82;
      pos[id].x+=pos[id].vx;pos[id].y+=pos[id].vy;
    });
  }
}
layout(600);

// ── Camera ──
let cam={x:0,y:0,zoom:1};
function getHudHeight(){
  const hud=document.getElementById('hud');
  if(!hud)return 0;
  return hud.offsetHeight + 20;
}
function fitToScreen(){
  let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  nids.forEach(id=>{x0=Math.min(x0,pos[id].x);x1=Math.max(x1,pos[id].x);
    y0=Math.min(y0,pos[id].y);y1=Math.max(y1,pos[id].y)});
  const p=60;
  const hudH=getHudHeight();
  const availH=H-hudH-p*2;
  const availW=W-p*2;
  cam.zoom=Math.min(availW/(x1-x0||1),availH/(y1-y0||1));
  cam.x=W/2-(x0+x1)/2*cam.zoom;
  cam.y=(H-hudH)/2-(y0+y1)/2*cam.zoom;
}
fitToScreen();
addEventListener('resize',()=>{resize();fitToScreen()});
function w2s(wx,wy){return{x:wx*cam.zoom+cam.x,y:wy*cam.zoom+cam.y}}

let dragging=false,dragX=0,dragY=0;
cv.addEventListener('mousedown',e=>{if(e.button===0){dragging=true;dragX=e.clientX;dragY=e.clientY;cv.style.cursor='grabbing'}});
addEventListener('mousemove',e=>{if(!dragging)return;cam.x+=e.clientX-dragX;cam.y+=e.clientY-dragY;dragX=e.clientX;dragY=e.clientY});
addEventListener('mouseup',()=>{dragging=false;cv.style.cursor='default'});
cv.addEventListener('wheel',e=>{e.preventDefault();const f=e.deltaY<0?1.15:1/1.15;
  cam.x=e.clientX-(e.clientX-cam.x)*f;cam.y=e.clientY-(e.clientY-cam.y)*f;cam.zoom*=f},{passive:false});

// ── Touch: 1 палец = pan, 2 пальца = pinch zoom (только над canvas) ──
let touchState=null; // {mode:'pan'|'pinch', x, y, dist, cx, cy}
function touchCenter(t1,t2){return{x:(t1.clientX+t2.clientX)/2,y:(t1.clientY+t2.clientY)/2}}
function touchDist(t1,t2){const dx=t2.clientX-t1.clientX,dy=t2.clientY-t1.clientY;return Math.sqrt(dx*dx+dy*dy)}

cv.addEventListener('touchstart',e=>{
  e.preventDefault();
  if(e.touches.length===1){
    touchState={mode:'pan',x:e.touches[0].clientX,y:e.touches[0].clientY};
  }else if(e.touches.length>=2){
    const c=touchCenter(e.touches[0],e.touches[1]);
    touchState={mode:'pinch',cx:c.x,cy:c.y,dist:touchDist(e.touches[0],e.touches[1])};
  }
},{passive:false});

cv.addEventListener('touchmove',e=>{
  e.preventDefault();
  if(!touchState)return;
  if(touchState.mode==='pan'&&e.touches.length===1){
    const t=e.touches[0];
    cam.x+=t.clientX-touchState.x;
    cam.y+=t.clientY-touchState.y;
    touchState.x=t.clientX;touchState.y=t.clientY;
  }else if(touchState.mode==='pinch'&&e.touches.length>=2){
    const c=touchCenter(e.touches[0],e.touches[1]);
    const d=touchDist(e.touches[0],e.touches[1]);
    const f=d/touchState.dist;
    // Зум вокруг центра между пальцами
    cam.x=c.x-(touchState.cx-cam.x)*f;
    cam.y=c.y-(touchState.cy-cam.y)*f;
    cam.zoom*=f;
    touchState.cx=c.x;touchState.cy=c.y;touchState.dist=d;
  }
},{passive:false});

cv.addEventListener('touchend',e=>{
  if(e.touches.length===0){touchState=null}
  else if(e.touches.length===1){
    // Переход с pinch на pan
    touchState={mode:'pan',x:e.touches[0].clientX,y:e.touches[0].clientY};
  }
},{passive:false});

// ── Physics ──
function phases(e,tr){
  const dist=e.dist;
  const ve=e.v_enter,vp=e.v_peak,vx=e.v_exit,ac=tr.accel,dc=tr.decel;
  let t1=0,d1=0;if(vp>ve&&ac<1e8){t1=(vp-ve)/ac;d1=(ve+vp)/2*t1}
  let t3=0,d3=0;if(vp>vx&&dc<1e8){t3=(vp-vx)/dc;d3=(vp+vx)/2*t3}
  return{t1,d1,t2:vp>0?Math.max(0,dist-d1-d3)/vp:0,d2:Math.max(0,dist-d1-d3),t3,d3,dist,ve,vp,vx,ac,dc};
}

function headState(tr,t){
  const sc=tr.schedule;if(!sc.length)return null;
  let entry=null,ei=-1;
  for(let i=0;i<sc.length;i++)if(t>=sc[i].t_enter&&t<sc[i].t_exit){entry=sc[i];ei=i;break}
  // Особый случай: t точно равен t_exit последнего сегмента или попадает на стык —
  // берём ПОСЛЕДНИЙ сегмент с t_enter == t (входим в новый, а не висим на старом)
  if(!entry){
    for(let i=sc.length-1;i>=0;i--)if(t===sc[i].t_enter){entry=sc[i];ei=i;break}
  }
  if(!entry){
    for(let i=0;i<sc.length-1;i++)if(t>sc[i].t_exit&&t<sc[i+1].t_enter)
      return{s:sc[i].s_exit,eid:sc[i].edge_id,spd:0,dir:sc[i].direction,wait:true,ei:i};
    if(sc.length&&t>=sc[sc.length-1].t_exit)
      return{s:sc[sc.length-1].s_exit,eid:sc[sc.length-1].edge_id,spd:0,
             dir:sc[sc.length-1].direction,wait:true,ei:sc.length-1};
    if(sc.length&&t<sc[0].t_enter)
      return{s:sc[0].s_enter,eid:sc[0].edge_id,spd:0,dir:sc[0].direction,wait:true,ei:0};
    return null;
  }
  const ph=phases(entry,tr),lt=t-entry.t_enter;
  let traveled,spd;
  if(lt<=ph.t1){spd=ph.ve+ph.ac*lt;traveled=ph.ve*lt+.5*ph.ac*lt*lt}
  else if(lt<=ph.t1+ph.t2){spd=ph.vp;traveled=ph.d1+ph.vp*(lt-ph.t1)}
  else{const bt=lt-ph.t1-ph.t2;spd=Math.max(0,ph.vp-ph.dc*bt);traveled=ph.d1+ph.d2+ph.vp*bt-.5*ph.dc*bt*bt}
  const prog=ph.dist>0?Math.max(0,Math.min(1,traveled/ph.dist)):0;
  return{s:entry.s_enter+(entry.s_exit-entry.s_enter)*prog,
         eid:entry.edge_id,spd:Math.max(0,spd),dir:entry.direction,wait:false,ei};
}

// ── Train body: polyline from tail to head ──
function sToWorld(eid,s){
  const e=T.edges[eid],a=pos[e.node_a],b=pos[e.node_b];
  return{x:a.x+(b.x-a.x)*s,y:a.y+(b.y-a.y)*s};
}

function trainBody(tr,t){
  const hs=headState(tr,t);
  if(!hs)return{pts:[],spd:0,dir:1,wait:true};
  const tl=tr.length||0;
  const headPt=sToWorld(hs.eid,hs.s);
  if(tl<=0)return{pts:[headPt],spd:hs.spd,dir:hs.dir,wait:hs.wait};

  const sc=tr.schedule;
  let pts=[headPt];
  let rem=tl;

  // Текущий сегмент: от головы до s_enter
  const ce=sc[hs.ei];
  const ceLen=T.edges[ce.edge_id].length;
  const headDist=Math.abs(hs.s-ce.s_enter)*ceLen;

  if(headDist>=rem){
    const ratio=rem/headDist;
    const tailS=hs.s+(ce.s_enter-hs.s)*ratio;
    pts.push(sToWorld(ce.edge_id,tailS));
    return{pts:pts.reverse(),spd:hs.spd,dir:hs.dir,wait:hs.wait};
  }
  pts.push(sToWorld(ce.edge_id,ce.s_enter));
  rem-=headDist;

  // Идём назад по предыдущим сегментам.
  // curS, curEid — точка "стыка" с уже отрисованным куском хвоста.
  let curEid=ce.edge_id;
  let curS=ce.s_enter;

  for(let i=hs.ei-1;i>=0&&rem>0;i--){
    const e=sc[i];
    const eLen=T.edges[e.edge_id].length;

    // (1) Стык: от curS (s_enter следующего сегмента) к e.s_exit (этого).
    //     На одном ребре — реальная физическая длина (для pocket-разворота
    //     это L_train). На разных — 0 (стык в узле).
    if(e.edge_id===curEid&&Math.abs(curS-e.s_exit)>1e-9){
      const stitchDist=Math.abs(curS-e.s_exit)*eLen;
      if(stitchDist>=rem){
        const ratio=rem/stitchDist;
        const tailS=curS+(e.s_exit-curS)*ratio;
        pts.push(sToWorld(e.edge_id,tailS));
        return{pts:pts.reverse(),spd:hs.spd,dir:hs.dir,wait:hs.wait};
      }
      rem-=stitchDist;
    }
    pts.push(sToWorld(e.edge_id,e.s_exit));

    // (2) Сам сегмент: от e.s_exit к e.s_enter
    const eDist=Math.abs(e.s_exit-e.s_enter)*eLen;
    if(eDist>=rem){
      const ratio=rem/eDist;
      const tailS=e.s_exit+(e.s_enter-e.s_exit)*ratio;
      pts.push(sToWorld(e.edge_id,tailS));
      rem=0;
    }else{
      pts.push(sToWorld(e.edge_id,e.s_enter));
      rem-=eDist;
      curEid=e.edge_id;
      curS=e.s_enter;
    }
  }

  // Хвост торчит за начало расписания
  if(rem>0){
    const first=sc[0];
    const e=T.edges[first.edge_id];
    const dirSign=first.s_exit>first.s_enter?-1:1;
    const remNorm=rem/e.length*dirSign;
    const tailS=first.s_enter+remNorm;
    pts.push(sToWorld(first.edge_id,tailS));
  }

  return{pts:pts.reverse(),spd:hs.spd,dir:hs.dir,wait:hs.wait};
}

// ── Drawing ──
function draw(t){
  cx.clearRect(0,0,W,H);
  const z=cam.zoom;

  // Edges
  eids.forEach(eid=>{
    const e=T.edges[eid],a=w2s(pos[e.node_a].x,pos[e.node_a].y),b=w2s(pos[e.node_b].x,pos[e.node_b].y);
    cx.beginPath();cx.moveTo(a.x,a.y);cx.lineTo(b.x,b.y);
    cx.strokeStyle='rgba(100,120,160,.35)';cx.lineWidth=Math.max(1,3*z/2*edgeScale);cx.stroke();
    if(z>0.3){cx.font=`${Math.max(8,10*z/2)*txtScale}px system-ui`;cx.fillStyle='rgba(100,120,160,.45)';
      cx.textAlign='center';cx.fillText(e.name,(a.x+b.x)/2,(a.y+b.y)/2-8*z/2)}
  });

  // Nodes
  nids.forEach(nid=>{
    const n=T.nodes[nid],p=w2s(pos[nid].x,pos[nid].y),sz=Math.max(3,8*z/2*nodeScale);
    cx.beginPath();
    if(n.type==='Switch'){cx.moveTo(p.x,p.y-sz);cx.lineTo(p.x+sz,p.y);cx.lineTo(p.x,p.y+sz);cx.lineTo(p.x-sz,p.y);cx.closePath();cx.fillStyle='rgba(46,204,113,.5)'}
    else if(n.type==='Turntable'){cx.arc(p.x,p.y,sz,0,Math.PI*2);cx.fillStyle='rgba(155,89,182,.5)'}
    else if(n.port_count<=1){cx.rect(p.x-sz*.75,p.y-sz*.75,sz*1.5,sz*1.5);cx.fillStyle='rgba(231,76,60,.5)'}
    else{cx.arc(p.x,p.y,sz*.7,0,Math.PI*2);cx.fillStyle='rgba(52,152,219,.5)'}
    cx.fill();cx.strokeStyle='rgba(200,214,229,.4)';cx.lineWidth=1.5;cx.stroke();
    if(z>0.3){cx.font=`${Math.max(8,11*z/2)*txtScale}px system-ui`;cx.fillStyle='#8395a7';cx.textAlign='center';
      cx.fillText(n.name,p.x,p.y-sz-4)}
  });

  // Goal markers — флажки в конце маршрута каждого поезда
  trains.forEach(tr=>{
    if(!tr.schedule.length)return;
    const last=tr.schedule[tr.schedule.length-1];
    const gp=sToWorld(last.edge_id,last.s_exit);
    const gs=w2s(gp.x,gp.y);
    const flagH=Math.max(10,18*z/2*nodeScale);
    const flagW=flagH*0.7;
    cx.save();
    cx.translate(gs.x,gs.y);
    // Шест
    cx.strokeStyle=tr.color;cx.lineWidth=Math.max(1.5,2.5*z/2);
    cx.beginPath();cx.moveTo(0,0);cx.lineTo(0,-flagH);cx.stroke();
    // Полотнище
    cx.fillStyle=tr.color+'cc';
    cx.beginPath();
    cx.moveTo(0,-flagH);
    cx.lineTo(flagW,-flagH+flagH*0.2);
    cx.lineTo(0,-flagH+flagH*0.4);
    cx.closePath();cx.fill();
    // Точка-цель на земле
    cx.beginPath();cx.arc(0,0,Math.max(2,3*z/2),0,Math.PI*2);
    cx.fillStyle=tr.color;cx.fill();
    cx.restore();
  });

  // Trains
  trains.forEach(tr=>{
    const body=trainBody(tr,t);
    if(!body.pts.length)return;

    // Draw body as polyline
    if(body.pts.length>=2){
      cx.beginPath();
      const p0=w2s(body.pts[0].x,body.pts[0].y);
      cx.moveTo(p0.x,p0.y);
      for(let i=1;i<body.pts.length;i++){
        const p=w2s(body.pts[i].x,body.pts[i].y);
        cx.lineTo(p.x,p.y);
      }
      cx.strokeStyle=tr.color;
      cx.lineWidth=Math.max(2,6*z/2);
      cx.lineCap='round';cx.lineJoin='round';
      cx.stroke();
    }

    // Head dot + glow
    const hp=body.pts[body.pts.length-1];
    const hps=w2s(hp.x,hp.y);
    const glowR=Math.max(6,14*z/2);
    const g=cx.createRadialGradient(hps.x,hps.y,0,hps.x,hps.y,glowR);
    g.addColorStop(0,tr.color+'50');g.addColorStop(1,tr.color+'00');
    cx.beginPath();cx.arc(hps.x,hps.y,glowR,0,Math.PI*2);cx.fillStyle=g;cx.fill();

    const r=Math.max(2,(body.wait?3:4)*z/2);
    cx.beginPath();cx.arc(hps.x,hps.y,r,0,Math.PI*2);
    cx.fillStyle=tr.color;cx.fill();cx.strokeStyle='#fff';cx.lineWidth=1;cx.stroke();

    // Tail dot
    if(body.pts.length>=2){
      const tp=body.pts[0];
      const tps=w2s(tp.x,tp.y);
      cx.beginPath();cx.arc(tps.x,tps.y,Math.max(1,2*z/2),0,Math.PI*2);
      cx.fillStyle=tr.color+'80';cx.fill();
    }

    // Label
    if(z>0.3){
      cx.font=`bold ${Math.max(8,11*z/2)*txtScale}px system-ui`;cx.fillStyle=tr.color;cx.textAlign='center';
      const sv=body.spd*body.dir;
      const labelY=hps.y-r-6;
      if(body.wait){
        // "T{id}" + нарисованная иконка паузы рядом
        const label=`T${tr.id+1}`;
        const lw=cx.measureText(label).width;
        const iconSize=Math.max(6,8*z/2)*txtScale;
        cx.fillText(label, hps.x-iconSize*0.7, labelY);
        // Иконка паузы: две полоски
        const bx=hps.x+lw/2-iconSize*0.3;
        const by=labelY-iconSize*0.7;
        cx.fillRect(bx, by, iconSize*0.28, iconSize);
        cx.fillRect(bx+iconSize*0.45, by, iconSize*0.28, iconSize);
      } else {
        cx.fillText(`T${tr.id+1} ${sv>=0?'+':''}${sv.toFixed(1)}`, hps.x, labelY);
      }
    }
  });
}

// ── Controls ──
let curT=0,playing=true,spd=1;
const sl=document.getElementById('sl'),td=document.getElementById('td'),sd=document.getElementById('sd');
const bpp=document.getElementById('bpp'),bppIcon=document.getElementById('bpp-icon');
const ICON_PLAY='<path d="M8 5v14l11-7z"/>';
const ICON_PAUSE='<path d="M6 4h4v16H6zm8 0h4v16h-4z"/>';
function setPlaying(v){
  playing=v;
  bppIcon.innerHTML = v ? ICON_PAUSE : ICON_PLAY;
}
bpp.onclick=()=>setPlaying(!playing);
setPlaying(true);
document.getElementById('br').onclick=()=>{curT=0;sl.value=0};
document.getElementById('bm').onclick=()=>{spd=Math.max(.1,spd/2);sd.textContent='×'+spd};
document.getElementById('bf').onclick=()=>{spd=Math.min(32,spd*2);sd.textContent='×'+spd};
sl.oninput=()=>{curT=(sl.value/1000)*maxT;setPlaying(false)};

let last=performance.now();
function loop(now){
  const dt=(now-last)/1000;last=now;
  if(playing){curT+=dt*spd;if(curT>maxT+3)curT=0;sl.value=(curT/maxT)*1000}
  td.textContent=curT.toFixed(2)+'s';draw(curT);requestAnimationFrame(loop);
}

document.getElementById('info').innerHTML=
  `Nodes: ${nids.length} | Edges: ${eids.length} | Trains: ${trains.length}<br>`+
  trains.map(t=>`<span style="color:${t.color}">T${t.id+1}</span>: ${t.total_time.toFixed(2)}s`).join(' | ')+
  `<br><span style="color:#576574">Scroll: zoom | Drag: pan</span>`;
requestAnimationFrame(loop);
</script></body></html>"""


def visualize(topo, results, train=None, filename='railway_viz.html'):
    if isinstance(results, PathResult):
        train = train or Train()
        results = [(results, train, '#e74c3c')]
    data = {'topology': _ser_topo(topo),
            'trains': [_ser_train(r,t,i,c) for i,(r,t,c) in enumerate(results) if r.found]}
    html = _HTML.replace('__DATA_JSON__', json.dumps(data, indent=2))
    path = os.path.abspath(filename)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(html)
    webbrowser.open(f'file://{path}')
    return path
