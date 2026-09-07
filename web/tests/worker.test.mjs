import test from 'node:test';
import assert from 'node:assert/strict';
import {Worker} from 'node:worker_threads';

// Execute the actual browser worker module with the worker messaging globals adapted.
test('worker supports pause, stepping, mission reset and invalid-command recovery',async()=>{
  const moduleURL=new URL('../worker.js',import.meta.url).href;
  const source=`const {parentPort}=require('node:worker_threads');globalThis.postMessage=m=>parentPort.postMessage(m);globalThis.onmessage=null;parentPort.on('message',data=>globalThis.onmessage?.({data}));import(${JSON.stringify(moduleURL)});`;
  const w=new Worker(source,{eval:true});
  const messages=[];w.on('message',m=>messages.push(m));
  const next=(predicate,timeout=5000)=>new Promise((resolve,reject)=>{
    const ready=messages.find(predicate);if(ready){resolve(ready);return;}
    const timer=setTimeout(()=>{w.off('message',handler);reject(new Error('Timed out waiting for worker state'));},timeout);
    function handler(m){if(predicate(m)){clearTimeout(timer);w.off('message',handler);resolve(m);}}
    w.on('message',handler);
  });
  try {
    const initial=await next(m=>m.type==='state');assert.equal(initial.running,false);assert.equal(initial.snapshot.time,0);
    w.postMessage({type:'step',seconds:60});const stepped=await next(m=>m.snapshot?.time>=60);assert.equal(stepped.snapshot.time,60);assert.equal(stepped.running,false);
    w.postMessage({type:'reset',preset:'orbit'});const reset=await next(m=>m.snapshot?.preset==='orbit');assert.equal(reset.snapshot.time,0);assert.equal(reset.history.length,1);
    w.postMessage({type:'warp',value:NaN});const invalid=await next(m=>m.error);assert.match(invalid.error,/temporal/);assert.equal(invalid.running,false);
    w.postMessage({type:'running',value:true});await next(m=>m.snapshot?.preset==='orbit'&&m.snapshot.time>0);
    w.postMessage({type:'running',value:false});await next(m=>m.snapshot?.preset==='orbit'&&m.snapshot.time>0&&!m.running&&!m.error);
    w.postMessage({type:'export'});const exported=await next(m=>m.type==='export');assert.ok(Array.isArray(exported.history));
  } finally {await w.terminate();}
});
