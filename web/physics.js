/** Newtonian flight mechanics. km, s, kg, N. No rendering or wall-clock state. */
export const G = 6.67430e-20;
export const G0 = 9.80665;
export const AU = 149597870.7;
export const DAY = 86400;
export const EARTH = { name: 'Tierra', mu: 398600.435507, radius: 6371.0084, color: '#579eff' };
export const MOON = { name: 'Luna', mu: 4902.800118, radius: 1737.4, color: '#c4cddd' };
export const SUN = { name: 'Sol', mu: 132712440041.9394, radius: 695700, color: '#ffd589' };
export const add = (a,b) => a.map((v,i)=>v+b[i]);
export const sub = (a,b) => a.map((v,i)=>v-b[i]);
export const mul = (a,k) => a.map(v=>v*k);
export const dot = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
export const norm = a => Math.hypot(...a);
export const unit = a => mul(a,1/(norm(a)||1));
export const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const clamp = (x,a,b) => Math.max(a,Math.min(b,x));

export function orbitalElements(p,v,mu,radius=0) {
  const r=norm(p), speed=norm(v), h=cross(p,v), energy=speed*speed/2-mu/r;
  const evec=sub(mul(cross(v,h),1/mu),unit(p)), e=norm(evec);
  const a=Math.abs(energy)>1e-14 ? -mu/(2*energy) : Infinity;
  const semiLatus=dot(h,h)/mu;
  return { altitude:r-radius, speed, radialSpeed:dot(p,v)/r, energy, eccentricity:e,
    periapsis:semiLatus/(1+e)-radius, apoapsis:energy<0 ? a*(1+e)-radius : null,
    period:energy<0 ? 2*Math.PI*Math.sqrt(a*a*a/mu):null, a, evec, h, semiLatus,
    circularSpeed:Math.sqrt(mu/r), escapeSpeed:Math.sqrt(2*mu/r) };
}
export function rocketDeltaV(dry,fuel,isp) { return isp*G0*Math.log((dry+fuel)/dry)/1000; }
export function hohmann(r1,r2,mu) {
  const a=(r1+r2)/2;
  return { departure:Math.sqrt(mu/r1)*(Math.sqrt(2*r2/(r1+r2))-1),
    arrival:Math.sqrt(mu/r2)*(1-Math.sqrt(2*r1/(r1+r2))), duration:Math.PI*Math.sqrt(a*a*a/mu) };
}
export function atmosphere(altitude) { return altitude>=180 ? 0 : 1.225*Math.exp(-Math.max(0,altitude)*1000/8500); }

export const PRESETS = {
  orbit: { name:'Laboratorio orbital', description:'Empieza a 400 km. Enciende el motor y descubre cómo cambia tu órbita.', focus:'earth', warp:100 },
  lunar: { name:'Rumbo a la Luna', description:'Inyección translunar desde 200 km y sobrevuelo bajo gravedad Tierra–Luna–Sol.', focus:'earth', warp:100 },
  launch: { name:'De la Tierra a órbita', description:'Despegue, giro gravitatorio y separación automática de un cohete de dos etapas.', focus:'earth', warp:10 },
  solar: { name:'Explorar el sistema solar', description:'Sol y ocho planetas con atracción mutua. Estados iniciales circulares idealizados.', focus:'sun', warp:1000000 },
};
function makeBody(seed,p,v) { return {...seed,p,v}; }
function earthMoonSystem(phase) {
  const d=384400, total=EARTH.mu+MOON.mu, w=Math.sqrt(total/d**3), f=MOON.mu/total;
  const rp=[d*Math.cos(phase),d*Math.sin(phase),0], rv=[-w*rp[1],w*rp[0],0];
  const earth=makeBody(EARTH,mul(rp,-f),mul(rv,-f));
  const moon=makeBody(MOON,mul(rp,1-f),mul(rv,1-f));
  // Earth–Moon barycentre at the origin initially; inertial frame, not a fixed Earth.
  const sun=makeBody(SUN,[-AU,0,0],[0,-Math.sqrt((SUN.mu+total)/AU),0]);
  return [earth,moon,sun];
}
function solarSystem() {
  const seeds=[['Mercurio',.387,22031.86855,2439.7,'#b8b2aa',.3],['Venus',.723,324858.592,6051.8,'#d2b780',1.6],['Tierra',1,EARTH.mu,EARTH.radius,EARTH.color,0],['Marte',1.524,42828.375816,3389.5,'#f18a66',2.1],['Júpiter',5.203,126712764.1,69911,'#dcbb9d',3],['Saturno',9.537,37940584.84,58232,'#e4d297',4.1],['Urano',19.191,5794556.4,25362,'#8cd7df',4.8],['Neptuno',30.069,6836527.1,24622,'#688fff',5.6]];
  const b=[makeBody(SUN,[0,0,0],[0,0,0]),...seeds.map(([name,a,mu,radius,color,phase])=>{
    const r=a*AU,v=Math.sqrt(SUN.mu/r);
    return makeBody({name,mu,radius,color},[r*Math.cos(phase),r*Math.sin(phase),0],[-v*Math.sin(phase),v*Math.cos(phase),0]);
  })];
  const total=b.reduce((s,x)=>s+x.mu,0);
  const com=b.reduce((s,x)=>add(s,mul(x.p,x.mu/total)),[0,0,0]);
  const cv=b.reduce((s,x)=>add(s,mul(x.v,x.mu/total)),[0,0,0]);
  for(const x of b) {x.p=sub(x.p,com);x.v=sub(x.v,cv);} return b;
}
export function systemEnergy(bodies) {
  let result=0;
  for(let i=0;i<bodies.length;i++) {
    result+=.5*bodies[i].mu*dot(bodies[i].v,bodies[i].v);
    for(let j=i+1;j<bodies.length;j++) result-=bodies[i].mu*bodies[j].mu/norm(sub(bodies[i].p,bodies[j].p));
  } return result;
}

export class Simulation {
  constructor(preset='lunar',options={}) {
    if(!PRESETS[preset]) throw new Error('Escenario desconocido');
    this.preset=preset; this.time=0; this.events=[]; this.finished=false;
    this.reference='Tierra'; this.guidance='prograde'; this.throttle=0;
    this.autopilot=['lunar','launch'].includes(preset); this.stageIndex=0;
    this.phase='En espera'; this.minMoonDistance=Infinity; this.closestTime=0;
    this.bodies=preset==='solar' ? solarSystem() : earthMoonSystem(options.moonPhase ?? 2.32);
    this.initialEnergy=systemEnergy(this.bodies); this.burnDV=0; this.maxQ=0;
    if(preset==='solar') {this.ship=null;this.phase='Gravitación N-body';return;}
    this.payload=6000;
    this.stages=preset==='launch' ? [
      {name:'Astra I · primera etapa',dry:30000,fuel:400000,capacity:400000,thrust:7600000,isp:300},
      {name:'Astra II · etapa orbital',dry:8000,fuel:100000,capacity:100000,thrust:1100000,isp:440},
    ] : [{name:'Etapa de transferencia',dry:12000,fuel:25000,capacity:25000,thrust:450000,isp:450}];
    const earth=this.bodies[0],r=EARTH.radius+(preset==='launch' ? .01 : preset==='orbit'?400:200);
    this.ship={p:add(earth.p,[r,0,0]),v:add(earth.v,[0,preset==='launch' ? 7.2921159e-5*r : Math.sqrt(EARTH.mu/r),0])};
    this.phase=preset==='launch'?'Lista para despegue':preset==='lunar'?'Lista para inyección':'Órbita de aparcamiento';
    this.log(this.phase);
  }
  get activeStage() {return this.stages?.[this.stageIndex];}
  get fuel() {return this.stages?.slice(this.stageIndex).reduce((s,x)=>s+x.fuel,0)||0;}
  get mass() {return this.payload+this.stages.slice(this.stageIndex).reduce((s,x)=>s+x.dry+x.fuel,0);}
  get deltaV() {
    if(!this.ship)return 0; let result=0,m=this.mass;
    for(const s of this.stages.slice(this.stageIndex)) {result+=rocketDeltaV(m-s.fuel,s.fuel,s.isp);m-=s.fuel+s.dry;}
    return result;
  }
  log(text) {this.events.push({time:this.time,text});if(this.events.length>100)this.events.shift();}
  setControl({guidance,throttle,autopilot,reference}) {
    if(guidance && !['prograde','retrograde','radial_out','radial_in','normal','antinormal'].includes(guidance))throw new Error('Orientación no válida');
    if(throttle!==undefined && (!Number.isFinite(throttle)||throttle<0||throttle>1))throw new Error('Potencia fuera de rango');
    if(reference && !['Tierra','Luna'].includes(reference))throw new Error('Referencia no válida');
    if(!this.ship || this.finished)return;
    if(guidance)this.guidance=guidance;
    if(reference)this.reference=reference;
    if(throttle!==undefined)this.throttle=throttle;
    if(autopilot!==undefined)this.autopilot=Boolean(autopilot);
  }
  separate() {
    if(!this.ship || this.finished || this.stageIndex>=this.stages.length-1)return false;
    this.log('Separación: '+this.activeStage.name);this.stageIndex++;return true;
  }
  relative(bodyName=this.reference) {
    const b=this.bodies.find(x=>x.name===bodyName)||this.bodies[0];
    return {p:sub(this.ship.p,b.p),v:sub(this.ship.v,b.v),body:b};
  }
  updateAutopilot() {
    if(!this.ship||this.finished)return;
    if(this.activeStage.fuel<=1e-8) {
      if(this.autopilot&&this.stageIndex<this.stages.length-1)this.separate();
      else if(this.throttle>0) {this.throttle=0;this.log('Motor apagado: propelente agotado');}
    }
    if(!this.autopilot)return;
    const rel=this.relative('Tierra'), o=orbitalElements(rel.p,rel.v,EARTH.mu,EARTH.radius);
    if(this.preset==='lunar' && !this.injectionDone) {
      if(this.time===0){this.phase='Inyección translunar';this.log('Encendido para inyección translunar');}
      this.throttle=1;this.guidance='prograde';this.reference='Tierra';
      if(o.apoapsis!==null&&o.apoapsis>=395000) {
        this.injectionDone=true;this.throttle=0;this.phase='Crucero translunar';this.log('Corte de inyección · vuelo libre hacia la Luna');
      }
    } else if(this.preset==='launch'&&!this.orbitReached) {
      if(this.time===0){this.phase='Ascenso propulsado';this.log('Despegue · programa de ascenso automático');}
      this.throttle=1;this.guidance='launch';this.reference='Tierra';
      if(o.periapsis>=160&&o.altitude>160) {
        this.orbitReached=true;this.throttle=0;this.phase='Órbita alcanzada';this.log('Inserción orbital · periapsis superior a 160 km');
      }
    }
    if(this.activeStage.fuel<=1e-8)this.throttle=0;
  }
  direction(p,v,bodies,elapsed=0,mass=this.mass) {
    const ref=bodies.find(x=>x.name===this.reference)||bodies[0];
    const r=sub(p,ref.p),rv=sub(v,ref.v),radial=unit(r),normal=unit(cross(r,rv));
    if(this.guidance==='launch') {
      const vr=dot(rv,radial), tangent=unit(sub(rv,mul(radial,vr))),vt=norm(sub(rv,mul(radial,vr))),alt=norm(r)-EARTH.radius;
      if(this.time+elapsed<12)return radial;
      const targetVr=clamp((200-alt)/120,-.08,.65);
      const needed=(targetVr-vr)/18+EARTH.mu/norm(r)**2-vt*vt/norm(r);
      const thrustAccel=this.activeStage.thrust/1000/mass;
      const sine=clamp(needed/thrustAccel,-.18,.98);
      return add(mul(radial,sine),mul(tangent,Math.sqrt(1-sine*sine)));
    }
    return {prograde:unit(rv),retrograde:mul(unit(rv),-1),radial_out:radial,radial_in:mul(radial,-1),normal,antinormal:mul(normal,-1)}[this.guidance]||unit(rv);
  }
  derivative(y,elapsed,flow,force) {
    const n=this.bodies.length, bodies=this.bodies.map((b,i)=>({...b,p:y.slice(i*6,i*6+3),v:y.slice(i*6+3,i*6+6)}));
    const out=new Array(y.length).fill(0);
    for(let i=0;i<n;i++) {
      const acc=[0,0,0];
      for(let j=0;j<n;j++)if(i!==j) {
        const d=sub(bodies[j].p,bodies[i].p),r=norm(d),k=bodies[j].mu/(r*r*r);
        for(let q=0;q<3;q++)acc[q]+=d[q]*k;
      }
      out.splice(i*6,6,...bodies[i].v,...acc);
    }
    if(this.ship) {
      const p=y.slice(n*6,n*6+3),v=y.slice(n*6+3,n*6+6),mass=this.mass-flow*elapsed;
      let acc=[0,0,0];
      for(const b of bodies){const d=sub(b.p,p),r=norm(d);acc=add(acc,mul(d,b.mu/r**3));}
      if(force>0) acc=add(acc,mul(this.direction(p,v,bodies,elapsed,mass),force/1000/mass));
      const earth=bodies.find(x=>x.name==='Tierra');
      const rp=sub(p,earth.p),alt=norm(rp)-earth.radius;
      if(alt<180) {
        const airV=add(earth.v,cross([0,0,7.2921159e-5],rp));
        const rv=sub(v,airV),speed=norm(rv)*1000;
        const drag=.5*atmosphere(alt)*speed*speed*.4*20;
        acc=sub(acc,mul(unit(rv),drag/mass/1000));
      }
      out.splice(n*6,6,...v,...acc);
    }
    return out;
  }
  step(dt) {
    if(!Number.isFinite(dt)||dt<=0)throw new Error('Paso temporal no válido');
    if(this.finished)return 0;
    this.updateAutopilot();
    let force=this.ship && this.activeStage.fuel>1e-8 ? this.activeStage.thrust*this.throttle : 0;
    const flow=force>0?force/(this.activeStage.isp*G0):0;
    if(flow>0)dt=Math.min(dt,this.activeStage.fuel/flow);
    if(dt<1e-10){this.activeStage.fuel=0;return 0;}
    const oldShip=this.ship?structuredClone(this.ship):null;
    const oldBodies=this.bodies.map(b=>b.p.slice());
    const y=this.bodies.flatMap(b=>[...b.p,...b.v]);if(this.ship)y.push(...this.ship.p,...this.ship.v);
    const k1=this.derivative(y,0,flow,force);
    const k2=this.derivative(y.map((v,i)=>v+k1[i]*dt/2),dt/2,flow,force);
    const k3=this.derivative(y.map((v,i)=>v+k2[i]*dt/2),dt/2,flow,force);
    const k4=this.derivative(y.map((v,i)=>v+k3[i]*dt),dt,flow,force);
    const next=y.map((v,i)=>v+dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6);
    if(next.some(v=>!Number.isFinite(v)))throw new Error('La integración produjo un estado no válido');
    this.bodies.forEach((b,i)=>{b.p=next.slice(i*6,i*6+3);b.v=next.slice(i*6+3,i*6+6);});
    if(this.ship) {
      const n=this.bodies.length*6, m=this.mass;
      this.ship.p=next.slice(n,n+3);this.ship.v=next.slice(n+3,n+6);
      if(flow>0) {this.activeStage.fuel=Math.max(0,this.activeStage.fuel-flow*dt);this.burnDV+=this.activeStage.isp*G0*Math.log(m/this.mass)/1000;}
      const er=this.relative('Tierra'),rv=sub(er.v,cross([0,0,7.2921159e-5],er.p));
      this.maxQ=Math.max(this.maxQ,.5*atmosphere(norm(er.p)-EARTH.radius)*(norm(rv)*1000)**2);
      // Swept segment collision test prevents crossing through a body between samples.
      for(let i=0;i<this.bodies.length;i++) {
        const a=sub(oldShip.p,oldBodies[i]),b=sub(this.ship.p,this.bodies[i].p),d=sub(b,a);
        const u=clamp(-dot(a,d)/(dot(d,d)||1),0,1);
        if(norm(add(a,mul(d,u)))<=this.bodies[i].radius) {
          const A=dot(d,d),B=2*dot(a,d),C=dot(a,a)-this.bodies[i].radius**2;
          const fraction=A>0?clamp((-B-Math.sqrt(Math.max(0,B*B-4*A*C)))/(2*A),0,1):0;
          this.ship.p=add(oldShip.p,mul(sub(this.ship.p,oldShip.p),fraction));
          this.finished=true;this.throttle=0;this.phase='Impacto en '+this.bodies[i].name;this.log(this.phase+' · simulación detenida');
          break;
        }
      }
      const distance=norm(this.relative('Luna').p)-MOON.radius;
      if(distance<this.minMoonDistance){this.minMoonDistance=distance;this.closestTime=this.time+dt;}
      if(this.preset==='lunar'&&this.injectionDone&&!this.finished) {
        if(distance<66000&&!this.enteredLunar){this.enteredLunar=true;this.phase='Encuentro lunar';this.log('Entrada en la región de influencia lunar');}
        if(this.enteredLunar&&!this.flybyDone&&this.time-this.closestTime>120&&distance>this.minMoonDistance+10){
          this.flybyDone=true;this.phase='Sobrevuelo completado';this.log('Máxima aproximación lunar: '+Math.round(this.minMoonDistance)+' km sobre la superficie');
        }
      }
    }
    this.time+=dt;
    if(this.time>=365.25*DAY*20){this.finished=true;this.log('Límite de 20 años de simulación alcanzado');}
    return dt;
  }
  safeStep() {
    if(!this.ship)return 1800;
    if(this.throttle>0||(this.autopilot&&this.time===0))return .5;
    let dt=120;
    for(const b of this.bodies) {
      const d=norm(sub(this.ship.p,b.p));
      dt=Math.min(dt,.012*Math.sqrt(d**3/b.mu));
      const alt=d-b.radius, speed=norm(sub(this.ship.v,b.v));
      if(alt<180)dt=Math.min(dt,.5,Math.max(.02,alt/Math.max(speed,1)/4));
    }
    return dt;
  }
  advance(seconds,maxSteps=1000000) {
    if(!Number.isFinite(seconds)||seconds<0)throw new Error('Duración no válida');
    const end=this.time+seconds;let steps=0;
    while(this.time<end-1e-9&&!this.finished&&steps++<maxSteps) this.step(Math.min(this.safeStep(),end-this.time));
    return this.time;
  }
  snapshot() {
    const base={preset:this.preset,time:this.time,phase:this.phase,finished:this.finished,bodies:this.bodies,ship:this.ship,
      events:this.events,autopilot:this.autopilot,throttle:this.throttle,guidance:this.guidance,reference:this.reference,
      energyDrift:(systemEnergy(this.bodies)-this.initialEnergy)/Math.abs(this.initialEnergy)};
    if(!this.ship)return base;
    const r=this.relative(), earth=this.relative('Tierra'), moon=this.relative('Luna');
    return {...base,orbit:orbitalElements(r.p,r.v,r.body.mu,r.body.radius),earthAltitude:norm(earth.p)-EARTH.radius,
      moonAltitude:norm(moon.p)-MOON.radius,minMoonDistance:this.minMoonDistance,mass:this.mass,fuel:this.fuel,deltaV:this.deltaV,
      burnDV:this.burnDV,maxQ:this.maxQ,stageIndex:this.stageIndex,stages:this.stages,force:this.activeStage.fuel>1e-8?this.activeStage.thrust*this.throttle:0};
  }
}
