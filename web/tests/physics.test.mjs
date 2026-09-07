import test from 'node:test';
import assert from 'node:assert/strict';
import {Simulation,EARTH,DAY,G0,orbitalElements,hohmann,rocketDeltaV,sub,norm} from '../physics.js';

test('400 km circular orbit matches the analytical two-body solution',()=>{
  const r=EARTH.radius+400,v=Math.sqrt(EARTH.mu/r),o=orbitalElements([r,0,0],[0,v,0],EARTH.mu,EARTH.radius);
  assert.ok(Math.abs(o.periapsis-400)<1e-8);assert.ok(Math.abs(o.apoapsis-400)<1e-8);
  assert.ok(Math.abs(o.period-5544.88)<1);assert.ok(o.eccentricity<1e-12);
});
test('orbital coast retains altitude and celestial energy over ten revolutions',()=>{
  const s=new Simulation('orbit');s.advance(55449);
  assert.ok(Math.abs(s.snapshot().orbit.altitude-400)<.1);
  assert.ok(Math.abs(s.snapshot().energyDrift)<1e-10);
  assert.equal(s.fuel,25000);
});
test('powered vacuum flight consumes the prescribed mass flow and rocket delta-v',()=>{
  const s=new Simulation('orbit'),initial=s.mass,stage=s.activeStage;
  s.setControl({autopilot:false,throttle:.4,guidance:'prograde'});s.advance(20);
  const used=.4*stage.thrust/(stage.isp*G0)*20;
  assert.ok(Math.abs(s.mass-(initial-used))<1e-7);
  assert.ok(Math.abs(s.burnDV-stage.isp*G0*Math.log(initial/s.mass)/1000)<1e-10);
  assert.ok(s.snapshot().orbit.apoapsis>500);
});
test('burnout is split exactly: no negative fuel or unearned impulse',()=>{
  const s=new Simulation('orbit');s.activeStage.fuel=1;
  const initial=s.mass;s.setControl({throttle:1});s.advance(10);
  assert.equal(s.activeStage.fuel,0);assert.equal(s.throttle,0);
  assert.ok(Math.abs(s.burnDV-rocketDeltaV(initial-1,1,450))<1e-10);
});
test('staging drops dry mass and remaining propellant exactly once',()=>{
  const s=new Simulation('launch'),mass=s.mass,stage=s.activeStage;
  assert.equal(s.separate(),true);assert.equal(s.mass,mass-stage.dry-stage.fuel);
  assert.equal(s.separate(),false);assert.equal(s.stageIndex,1);
});
test('automated launch reaches a viable orbit without invented velocity changes',()=>{
  const s=new Simulation('launch');s.advance(1800);const x=s.snapshot();
  assert.equal(x.finished,false);assert.equal(s.orbitReached,true);
  assert.ok(x.orbit.periapsis>150);assert.ok(x.orbit.apoapsis<1000);
  assert.equal(x.stageIndex,1);assert.ok(x.fuel>0);assert.equal(x.throttle,0);
  assert.equal(x.events.filter(e=>e.text.includes('Separación:')).length,1);
});
test('lunar scenario produces a safe flyby within 5000 km above the Moon',()=>{
  const s=new Simulation('lunar');s.advance(5*DAY);
  assert.equal(s.finished,false);assert.equal(s.flybyDone,true);
  assert.ok(s.minMoonDistance>2000&&s.minMoonDistance<5000);
  assert.ok(s.closestTime>3*DAY&&s.closestTime<4*DAY);
  assert.ok(s.fuel>2000);assert.equal(s.throttle,0);
});
test('lunar result converges when numerical steps are halved',()=>{
  const a=new Simulation('lunar'),b=new Simulation('lunar');
  const original=b.safeStep.bind(b);b.safeStep=()=>original()/2;
  a.advance(4*DAY);b.advance(4*DAY);
  assert.ok(Math.abs(a.minMoonDistance-b.minMoonDistance)<.05);
});
test('radial fall collides and stops instead of passing through the Earth',()=>{
  const s=new Simulation('orbit');s.ship.v=s.bodies[0].v.slice();s.advance(2000);
  assert.equal(s.finished,true);assert.equal(s.phase,'Impacto en Tierra');
  const time=s.time;s.advance(100);assert.equal(s.time,time);
});
test('prograde direction is relative to the selected moving body',()=>{
  const s=new Simulation('orbit');const d=s.direction(s.ship.p,s.ship.v,s.bodies);
  assert.ok(Math.abs(d[0])<1e-12);assert.ok(Math.abs(d[1]-1)<1e-12);
  s.setControl({reference:'Luna'});const moon=s.bodies[1],delta=sub(s.ship.v,moon.v);
  assert.ok(norm(sub(s.direction(s.ship.p,s.ship.v,s.bodies),delta.map(x=>x/norm(delta))))<1e-12);
});
test('normal thrust changes inclination and retrograde thrust lowers perigee',()=>{
  const n=new Simulation('orbit');n.setControl({guidance:'normal',throttle:1});n.advance(30);
  assert.ok(Math.abs(n.relative().v[2])>.1);
  const r=new Simulation('orbit');r.setControl({guidance:'retrograde',throttle:.2});r.advance(30);
  assert.ok(r.snapshot().orbit.periapsis<350);
});
test('Earth-Mars Hohmann transfer has the expected energy and travel time',()=>{
  const h=hohmann(149597870.7,149597870.7*1.524,132712440041.9394);
  assert.ok(h.duration/DAY>258&&h.duration/DAY<260);assert.ok(h.departure>2.9&&h.departure<3);
});
test('solar N-body model stays finite and conserves energy over one year',()=>{
  const s=new Simulation('solar');s.advance(365.25*DAY);
  assert.equal(s.bodies.length,9);assert.ok(Math.abs(s.snapshot().energyDrift)<1e-8);
  assert.ok(s.bodies.every(b=>b.p.concat(b.v).every(Number.isFinite)));
});
test('rejects invalid times, throttle, guidance and unknown missions',()=>{
  const s=new Simulation('orbit');for(const dt of [-1,0,NaN,Infinity])assert.throws(()=>s.step(dt));
  assert.throws(()=>s.advance(-1));assert.throws(()=>s.setControl({throttle:2}));
  assert.throws(()=>s.setControl({guidance:'teleport'}));assert.throws(()=>new Simulation('unknown'));
});
