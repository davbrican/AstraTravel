import {PRESETS,AU} from './physics.js';
import {OrbitalRenderer,drawChart} from './renderer.js';
const $=id=>document.getElementById(id);
const renderer=new OrbitalRenderer($('space'));
let state=null,history=[],trail=[],toastTimer,worker;
const fmt=(v,d=1)=>Number.isFinite(v)?v.toLocaleString('es-ES',{maximumFractionDigits:d,minimumFractionDigits:d}):'—';
const quantity=(v,u,d=1)=>`${fmt(v,d)} <small>${u}</small>`;
function duration(t){const days=Math.floor(t/86400),h=Math.floor(t/3600)%24,m=Math.floor(t/60)%60,s=Math.floor(t)%60;return (days?days+'d ':'')+[h,m,s].map(v=>String(v).padStart(2,'0')).join(':');}
function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,5000);}
function post(type,params={}){worker?.postMessage({type,...params});}
function controls(){
  if(!state)return;const s=state.snapshot,auto=s.autopilot,disabled=s.finished;
  $('autopilot').checked=auto;$('autopilot').disabled=!['lunar','launch'].includes(s.preset)||disabled;
  $('control-mode').textContent=auto?'AUTO':'MANUAL';
  for(const id of ['guidance','throttle','reference'])$(id).disabled=auto||disabled;
  if(document.activeElement!==$('guidance'))$('guidance').value=s.guidance;
  $('reference').value=s.reference;$('throttle').value=Math.round(s.throttle*100);$('throttle-value').textContent=Math.round(s.throttle*100)+' %';
  $('cutoff').disabled=disabled;$('separate').disabled=disabled||auto;
  $('play').disabled=disabled;$('play').innerHTML=state.running?'Pausar vuelo <span aria-hidden="true">Ⅱ</span>':s.time>0?'Continuar <span aria-hidden="true">▶</span>':'Iniciar vuelo <span aria-hidden="true">▶</span>';
  $('step').disabled=state.running||disabled;
  $('run-status').textContent=disabled?'FINALIZADA':state.running?'EN SIMULACIÓN':'EN PAUSA';$('run-status').classList.toggle('active',state.running);
}
function update(message) {
  const s=message.snapshot;state=message;history=message.history;trail=message.trail;
  $('phase').textContent=s.phase;$('mission-time').textContent='T+ '+duration(s.time);
  $('energy-error').textContent=(Math.abs(s.energyDrift)*100).toExponential(2)+' %';
  $('solar-days').textContent=fmt(s.time/86400,1)+' días';
  if(s.ship){
    $('alt-reference').textContent=s.reference==='Luna'?'sobre la Luna':'sobre la Tierra';
    $('altitude').innerHTML=quantity(s.orbit.altitude,'km');$('speed').innerHTML=quantity(s.orbit.speed,'km/s',3);
    $('periapsis').innerHTML=quantity(s.orbit.periapsis,'km',0);$('apoapsis').innerHTML=s.orbit.apoapsis===null?'Escape':quantity(s.orbit.apoapsis,'km',0);
    $('delta-v').innerHTML=quantity(s.deltaV,'km/s',2);$('mass').innerHTML=quantity(s.mass/1000,'t',1);$('eccentricity').textContent=fmt(s.orbit.eccentricity,4);$('thrust').innerHTML=quantity(s.force/1000,'kN',0);
    $('moon-distance').textContent=fmt(s.moonAltitude,0)+' km';$('closest').textContent='Aproximación mínima: '+fmt(s.minMoonDistance,0)+' km';
    const stage=s.stages[s.stageIndex],percent=stage.fuel/stage.capacity*100;
    $('stage-name').textContent=stage.name;$('fuel-percent').textContent=fmt(percent,0)+' %';$('fuel-bar').value=percent;$('fuel-mass').textContent=fmt(stage.fuel,0)+' kg';
    $('separate').hidden=s.stageIndex>=s.stages.length-1;
  }
  const signature=s.events.map(e=>e.text+e.time).join('|');
  if($('events').dataset.signature!==signature){
    $('events').replaceChildren(...s.events.slice().reverse().map(event=>{const li=document.createElement('li'),t=document.createElement('time');t.textContent='T+ '+duration(event.time);li.append(t,document.createTextNode(event.text));return li;}));
    $('events').dataset.signature=signature;
  }
  controls();renderer.update(s,trail);drawChart($('chart'),history,$('chart-metric').value,s.preset==='solar');
  if(message.error)toast(message.error);
}
function configurePreset(preset) {
  const solar=preset==='solar';$('description').textContent=PRESETS[preset].description;
  $('ship-controls').hidden=solar;$('telemetry-ship').hidden=solar;$('solar-stats').hidden=!solar;
  $('chart-metric').disabled=solar;$('export').disabled=solar;
  $('warp').value=String(PRESETS[preset].warp);post('warp',{value:PRESETS[preset].warp});
  $('focus').value=PRESETS[preset].focus;renderer.setFocus(PRESETS[preset].focus);
  for(const option of $('focus').options)option.disabled=solar&&!['sun','earth'].includes(option.value);
  $('view-title').textContent=solar?'Sistema solar':preset==='launch'?'Ascenso desde la Tierra':preset==='orbit'?'Órbita terrestre':'Sistema Tierra–Luna';
  if(solar)renderer.span=75*AU;
}
try {
  worker=new Worker(new URL('./worker.js',import.meta.url),{type:'module'});
  worker.onmessage=({data})=>{
    if(data.type==='export') {
      const csv='tiempo_s,altitud_tierra_km,velocidad_relativa_km_s,propelente_kg,referencia_velocidad\n'+data.history.map(r=>[r.t,r.alt,r.speed,r.fuel,r.reference||'Tierra'].join(',')).join('\n');
      const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='astratravel-'+state.snapshot.preset+'-telemetria.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);toast('Telemetría exportada.');
    } else update(data);
  };
  worker.onerror=()=>{toast('No se pudo iniciar el motor. Abre la aplicación mediante npm start o Docker, no como archivo local.');$('phase').textContent='Motor de simulación no disponible';$('play').disabled=true;};
} catch {toast('Este navegador no permite iniciar el motor de simulación. Abre la web con un servidor HTTP.');}
configurePreset('lunar');
$('preset').addEventListener('change',()=>{post('reset',{preset:$('preset').value});configurePreset($('preset').value);});
$('play').addEventListener('click',()=>post('running',{value:!state?.running}));
$('step').addEventListener('click',()=>post('step',{seconds:60}));
$('reset').addEventListener('click',()=>{post('reset',{preset:$('preset').value});configurePreset($('preset').value);toast('Misión reiniciada.');});
$('warp').addEventListener('change',()=>post('warp',{value:Number($('warp').value)}));
$('autopilot').addEventListener('change',()=>{post('control',{value:{autopilot:$('autopilot').checked,throttle:0,guidance:$('guidance').value==='launch'?'prograde':$('guidance').value}});toast($('autopilot').checked?'Guiado automático activado.':'Control manual. El motor está apagado.');});
$('guidance').addEventListener('change',()=>post('control',{value:{guidance:$('guidance').value}}));
$('reference').addEventListener('change',()=>post('control',{value:{reference:$('reference').value}}));
$('throttle').addEventListener('input',()=>{$('throttle-value').textContent=$('throttle').value+' %';post('control',{value:{throttle:Number($('throttle').value)/100}});});
$('cutoff').addEventListener('click',()=>{post('control',{value:{autopilot:false,throttle:0,guidance:'prograde'}});toast('Motor apagado. Control manual activado.');});
$('separate').addEventListener('click',()=>{post('separate');toast('Etapa separada.');});
$('focus').addEventListener('change',()=>renderer.setFocus($('focus').value));
$('zoom-in').addEventListener('click',()=>renderer.zoom(.7));$('zoom-out').addEventListener('click',()=>renderer.zoom(1/.7));
$('reset-view').addEventListener('click',()=>renderer.reset());$('top-view').addEventListener('click',()=>{renderer.tilt=0;renderer.yaw=0;renderer.draw();});
$('chart-metric').addEventListener('change',()=>drawChart($('chart'),history,$('chart-metric').value,state?.snapshot.preset==='solar'));
$('export').addEventListener('click',()=>post('export'));
$('help').addEventListener('click',()=>{$('model-dialog').showModal();});$('close-model').addEventListener('click',()=>$('model-dialog').close());
$('model-dialog').addEventListener('click',e=>{if(e.target===$('model-dialog')){const r=$('model-dialog').getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)$('model-dialog').close();}});
window.addEventListener('keydown',e=>{
  if(['INPUT','SELECT','BUTTON','TEXTAREA'].includes(document.activeElement?.tagName)||$('model-dialog').open)return;
  if(e.code==='Space'){e.preventDefault();if(state&&!state.snapshot.finished)post('running',{value:!state.running});}
  if(e.key==='+')renderer.zoom(.7);if(e.key==='-')renderer.zoom(1/.7);
});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&state?.running)post('running',{value:false});});
new ResizeObserver(()=>drawChart($('chart'),history,$('chart-metric').value,state?.snapshot.preset==='solar')).observe($('chart'));
