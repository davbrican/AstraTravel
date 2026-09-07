import test from 'node:test';
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import {readFile} from 'node:fs/promises';

test('serves the complete application with correct module MIME types and no private files',async()=>{
  const child=spawn(process.execPath,[new URL('../server.mjs',import.meta.url).pathname],{env:{...process.env,PORT:'18895'},stdio:['ignore','pipe','pipe']});
  try {
    await Promise.race([once(child.stdout,'data'),once(child,'exit').then(()=>{throw new Error('HTTP server exited before becoming ready');})]);
    const origin='http://127.0.0.1:18895';
    for(const file of ['/','/app.js','/worker.js','/physics.js','/renderer.js','/styles.css','/favicon.svg']) {
      const r=await fetch(origin+file);assert.equal(r.status,200,file);
      if(file.endsWith('.js'))assert.match(r.headers.get('content-type'),/javascript/);
      assert.ok((await r.text()).length>40,file);
    }
    for(const file of ['/package.json','/tests/physics.test.mjs','/server.mjs','/.git/config','/missing.js'])assert.equal((await fetch(origin+file)).status,404,file);
    const html=await readFile(new URL('../index.html',import.meta.url),'utf8');
    const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);assert.equal(ids.length,new Set(ids).size,'Duplicate HTML IDs');
    const app=await readFile(new URL('../app.js',import.meta.url),'utf8');
    for(const [,id] of app.matchAll(/\$\('([^']+)'\)/g))assert.ok(ids.includes(id),'Missing element: '+id);
  } finally {child.kill();await once(child,'exit');}
});
