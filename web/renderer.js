import {AU,add,sub,mul,norm,unit,cross,orbitalElements} from './physics.js';
const TAU=Math.PI*2;
export class OrbitalRenderer {
  constructor(canvas) {
    this.canvas=canvas;this.ctx=canvas.getContext('2d');this.focus='earth';this.span=23500;this.yaw=-.35;this.tilt=.4;
    this.pointers=new Map();this.resizeObserver=new ResizeObserver(()=>this.draw());this.resizeObserver.observe(canvas);
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom(Math.exp(e.deltaY*.001));},{passive:false});
    canvas.addEventListener('pointerdown',e=>{canvas.setPointerCapture(e.pointerId);this.pointers.set(e.pointerId,[e.clientX,e.clientY]);});
    canvas.addEventListener('pointermove',e=>{
      const old=this.pointers.get(e.pointerId);if(!old)return;
      if(this.pointers.size===2){const other=[...this.pointers.entries()].find(([id])=>id!==e.pointerId)[1];const d0=Math.hypot(old[0]-other[0],old[1]-other[1]);const d1=Math.hypot(e.clientX-other[0],e.clientY-other[1]);if(d1>0)this.zoom(d0/d1);}
      else{this.yaw+=(e.clientX-old[0])*.006;this.tilt=Math.max(-1.3,Math.min(1.3,this.tilt+(e.clientY-old[1])*.006));}
      this.pointers.set(e.pointerId,[e.clientX,e.clientY]);this.draw();
    });
    for(const event of ['pointerup','pointercancel','lostpointercapture'])canvas.addEventListener(event,e=>this.pointers.delete(e.pointerId));
  }
  setFocus(focus) {this.focus=focus;this.reset();}
  reset() {this.span={earth:23500,moon:18000,system:1050000,ship:20000,sun:75*AU}[this.focus];this.yaw=-.35;this.tilt=.4;this.draw();}
  zoom(factor) {this.span=Math.max(100,Math.min(150*AU,this.span*factor));this.draw();}
  update(snapshot,trail) {this.s=snapshot;this.trail=trail;this.draw();}
  setup() {
    const w=this.canvas.clientWidth,h=this.canvas.clientHeight,dpr=Math.min(devicePixelRatio||1,2);
    if(!w||!h)return false;
    if(this.canvas.width!==Math.round(w*dpr)||this.canvas.height!==Math.round(h*dpr)){this.canvas.width=Math.round(w*dpr);this.canvas.height=Math.round(h*dpr);}
    this.ctx.setTransform(dpr,0,0,dpr,0,0);this.w=w;this.h=h;this.scale=Math.min(w,h)/this.span;return true;
  }
  project(p) {
    const x=p[0]*Math.cos(this.yaw)-p[1]*Math.sin(this.yaw),y=p[0]*Math.sin(this.yaw)+p[1]*Math.cos(this.yaw);
    return [this.w/2+x*this.scale,this.h/2-(y*Math.cos(this.tilt)-p[2]*Math.sin(this.tilt))*this.scale];
  }
  path(points,color,dashed=false,width=1) {
    if(points.length<2)return;const c=this.ctx;c.beginPath();
    points.forEach((p,i)=>{const [x,y]=this.project(p);if(i===0)c.moveTo(x,y);else c.lineTo(x,y);});
    c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dashed?[4,6]:[]);c.stroke();c.setLineDash([]);
  }
  label(text,p,color='#8b9fb8',dx=12,dy=-10) {
    const [x,y]=this.project(p);if(x<0||x>this.w||y<0||y>this.h)return;
    const c=this.ctx;c.font='11px ui-monospace, monospace';const width=c.measureText(text).width;
    const tx=Math.max(8,Math.min(this.w-width-8,x+dx)),ty=Math.max(15,Math.min(this.h-10,y+dy));
    c.fillStyle='#0a111de0';c.fillRect(tx-4,ty-12,width+8,17);c.fillStyle=color;c.fillText(text,tx,ty);
  }
  draw() {
    if(!this.setup())return;const c=this.ctx,w=this.w,h=this.h;c.clearRect(0,0,w,h);
    if(!this.s){c.fillStyle='#9aaabf';c.font='14px system-ui';c.fillText('Preparando vuelo…',20,h/2);return;}
    const s=this.s,earth=s.bodies.find(b=>b.name==='Tierra'),moon=s.bodies.find(b=>b.name==='Luna'),sun=s.bodies.find(b=>b.name==='Sol');
    let target=this.focus==='moon'?moon:this.focus==='sun'?sun:earth;
    if(!target)target=s.bodies[0];
    const center=this.focus==='ship'&&s.ship?s.ship.p:this.focus==='system'&&moon?mul(add(earth.p,moon.p),.5):target.p;
    const relative=p=>sub(p,center);
    // Cartesian grid in the inertial orbital plane. Distances remain linear.
    const step=10**Math.floor(Math.log10(this.span/5));
    for(let i=-8;i<=8;i++){
      this.path([[i*step,-8*step,0],[i*step,8*step,0]],i===0?'#24384c':'#142234');
      this.path([[-8*step,i*step,0],[8*step,i*step,0]],i===0?'#24384c':'#142234');
    }
    if(s.preset==='solar') {
      for(const b of s.bodies.filter(b=>b!==sun)) {
        const r=norm(sub(b.p,sun.p));const points=Array.from({length:161},(_,i)=>relative(add(sun.p,[r*Math.cos(i/160*TAU),r*Math.sin(i/160*TAU),0])));
        this.path(points,'#344b65',true);
      }
    } else if(moon&&(this.focus==='system'||this.span>200000)) {
      const r=norm(sub(moon.p,earth.p));this.path(Array.from({length:181},(_,i)=>relative(add(earth.p,[r*Math.cos(i/180*TAU),r*Math.sin(i/180*TAU),0]))),'#344b65',true);
    }
    if(s.ship) {
      const ref=s.bodies.find(b=>b.name===s.reference)||earth;
      const rp=sub(s.ship.p,ref.p),rv=sub(s.ship.v,ref.v),o=orbitalElements(rp,rv,ref.mu,ref.radius);
      if(norm(o.h)>1e-6) {
        const x=o.eccentricity>1e-5?unit(o.evec):unit(rp),y=unit(cross(unit(o.h),x));
        const maxAngle=o.eccentricity>=1?Math.acos(-1/o.eccentricity)-.03:Math.PI;
        const points=[];
        for(let i=0;i<=300;i++) {
          const a=-maxAngle+2*maxAngle*i/300,r=o.semiLatus/(1+o.eccentricity*Math.cos(a));
          if(r<this.span*30)points.push(relative(add(ref.p,add(mul(x,r*Math.cos(a)),mul(y,r*Math.sin(a))))));
        }
        this.path(points,'#577da6',true);
        if(o.eccentricity>.005){
          this.label('Pe',relative(add(ref.p,mul(x,o.periapsis+ref.radius))),'#94adc9',8,15);
          if(o.apoapsis!==null)this.label('Ap',relative(add(ref.p,mul(x,-o.apoapsis-ref.radius))),'#94adc9',8,15);
        }
      }
      if(this.trail?.length) {
        const fixedFocus=['earth','moon','system','ship'].includes(this.focus)?(this.focus==='moon'?'Luna':'Tierra'):null;
        const b=s.bodies.find(b=>b.name===fixedFocus);
        this.path(this.trail.map(t=>relative(b&&t.relative[b.name]?add(b.p,t.relative[b.name]):t.p)),'#b6f27f',false,1.6);
      }
    }
    for(const b of s.bodies) {
      const p=relative(b.p),[x,y]=this.project(p),r=Math.max(b.radius*this.scale,3.5);
      if(x+r<0||x-r>w||y+r<0||y-r>h)continue;
      c.beginPath();c.arc(x,y,Math.min(r,10000),0,TAU);
      const gradient=c.createRadialGradient(x-r*.3,y-r*.35,r*.05,x,y,r);
      gradient.addColorStop(0,b.color);gradient.addColorStop(.65,b.color+'bb');gradient.addColorStop(1,'#101a2a');
      c.fillStyle=gradient;c.fill();c.strokeStyle=b.color+'88';c.lineWidth=1;c.stroke();
      if(b.name==='Tierra'&&r>25){c.beginPath();c.ellipse(x,y,r,r*.2,-this.yaw,0,TAU);c.strokeStyle='#89b6ec40';c.stroke();c.beginPath();c.ellipse(x,y,r*.35,r,0,0,TAU);c.stroke();}
      this.label(b.name.toUpperCase(),p,b.color,r+9,0);
    }
    if(s.ship) {
      const p=relative(s.ship.p),[x,y]=this.project(p);
      if(x>-10&&x<w+10&&y>-10&&y<h+10) {
        c.beginPath();c.arc(x,y,12,0,TAU);c.strokeStyle='#bcf78555';c.stroke();
        c.beginPath();c.moveTo(x,y-6);c.lineTo(x+5,y+5);c.lineTo(x,y+2);c.lineTo(x-5,y+5);c.closePath();c.fillStyle=s.finished?'#ff947d':'#bcf785';c.fill();
        if(s.force>0){c.beginPath();c.moveTo(x-2,y+7);c.lineTo(x,y+14);c.lineTo(x+2,y+7);c.fillStyle='#ffb476';c.fill();}
        this.label('ASTRA',p,'#bcf785',17,-14);
      } else {
        c.fillStyle='#9faec2';c.font='12px system-ui';c.fillText('Nave fuera de vista · selecciona « Nave »',16,h-80);
      }
    }
    const distance=80/this.scale;
    const scaleElement=document.getElementById('scale');
    if(scaleElement)scaleElement.textContent=distance>AU/10?(distance/AU).toFixed(2)+' UA':Math.round(distance).toLocaleString('es-ES')+' km';
  }
}
export function drawChart(canvas,history,metric,solar=false) {
  const w=canvas.clientWidth,h=canvas.clientHeight,dpr=Math.min(devicePixelRatio||1,2);if(!w||!h)return;
  canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
  const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);
  const pad={l:54,r:12,t:22,b:29},pw=w-pad.l-pad.r,ph=h-pad.t-pad.b;
  c.font='10px ui-monospace, monospace';
  if(history.length<2||solar){c.fillStyle='#74869c';c.fillText(solar?'Observa las órbitas de los planetas en el visor.':'Inicia el vuelo para registrar telemetría.',12,h/2);return;}
  const values=history.map(x=>x[metric]),lo=Math.min(...values),hi=Math.max(...values);
  const min=Math.min(0,lo),max=Math.max(hi,1),first=history[0].t,last=history.at(-1).t;
  for(let i=0;i<4;i++) {
    const y=pad.t+i*ph/3;c.beginPath();c.moveTo(pad.l,y);c.lineTo(w-pad.r,y);c.strokeStyle='#203044';c.stroke();
    c.fillStyle='#8696ab';c.textAlign='right';c.fillText(((1-i/3)*(max-min)+min).toLocaleString('es-ES',{maximumFractionDigits:max<10?2:0,notation:max>=1e6?'compact':'standard'}),pad.l-7,y+3);
  }
  const point=(v,i)=>[pad.l+(history[i].t-first)/(last-first||1)*pw,pad.t+(max-v)/(max-min)*ph];
  c.beginPath();values.forEach((v,i)=>{const [x,y]=point(v,i);i?c.lineTo(x,y):c.moveTo(x,y);});c.strokeStyle='#bcf785';c.lineWidth=1.5;c.stroke();
  c.lineTo(w-pad.r,h-pad.b);c.lineTo(pad.l,h-pad.b);c.closePath();
  const g=c.createLinearGradient(0,pad.t,0,h-pad.b);g.addColorStop(0,'#bcf78520');g.addColorStop(1,'#bcf78500');c.fillStyle=g;c.fill();
  c.fillStyle='#8c9db2';c.textAlign='left';c.fillText((first/60).toFixed(0)+' min',pad.l,h-9);c.textAlign='right';c.fillText((last/60).toLocaleString('es-ES',{maximumFractionDigits:0})+' min',w-pad.r,h-9);
}
