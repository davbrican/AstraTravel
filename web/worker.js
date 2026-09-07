import { Simulation } from './physics.js';
let sim=new Simulation('lunar'),running=false,warp=100,goal=0,last=performance.now(),lastSend=0,generation=0;
let history=[],trail=[],lastSample=-Infinity;
function sample(force=false) {
  const interval=sim.preset==='solar'?21600:sim.throttle>0?2:Math.max(5,sim.safeStep());
  if(!force&&sim.time-lastSample<interval)return;
  lastSample=sim.time;
  const x=sim.snapshot();
  if(x.ship) {
    // Store positions relative to each moving focus at the same epoch.
    const relative={};
    for(const b of x.bodies)relative[b.name]=x.ship.p.map((v,i)=>v-b.p[i]);
    trail.push({p:x.ship.p.slice(),relative});
    history.push({t:x.time,alt:x.earthAltitude,speed:x.orbit.speed,fuel:x.fuel,reference:x.reference});
    if(trail.length>4000)trail=trail.filter((_,i)=>i%2===0);
    if(history.length>2000)history=history.filter((_,i)=>i%2===0);
  }
}
function send(error) {
  postMessage({type:'state',generation,snapshot:sim.snapshot(),running,warp,history,trail,error});lastSend=performance.now();
}
onmessage=({data})=>{
  try {
    if(data.type==='reset') {
      sim=new Simulation(data.preset);generation++;running=false;goal=0;history=[];trail=[];lastSample=-Infinity;sample(true);
    } else if(data.type==='running') {running=Boolean(data.value)&&!sim.finished;goal=sim.time;}
    else if(data.type==='warp') {if(!Number.isFinite(data.value)||data.value<1||data.value>10000000)throw new Error('Escala temporal no válida');warp=data.value;goal=sim.time;}
    else if(data.type==='control')sim.setControl(data.value);
    else if(data.type==='separate'){sim.separate();}
    else if(data.type==='step'){goal=sim.time+Math.min(Math.max(Number(data.seconds)||0,0),86400);}
    else if(data.type==='export') {postMessage({type:'export',history});return;}
    last=performance.now();send();
  } catch(e){running=false;goal=sim.time;send(e.message);}
};
function tick() {
  const now=performance.now(),elapsed=Math.min((now-last)/1000,.1);last=now;
  if(running)goal=Math.min(goal+elapsed*warp,sim.time+warp*.3);
  try {
    const deadline=performance.now()+18;
    while(sim.time<goal-1e-9&&!sim.finished&&performance.now()<deadline) {
      const dt=sim.safeStep();
      // Accumulate fractional wall-clock time; rendering cadence must not change integration steps.
      if(running && goal-sim.time<dt)break;
      sim.step(Math.min(dt,goal-sim.time));sample();
    }
    if(sim.finished){running=false;goal=sim.time;}
    if(performance.now()-lastSend>90)send();
  } catch(e){running=false;goal=sim.time;send(e.message);}
  setTimeout(tick,12);
}
sample(true);send();tick();
