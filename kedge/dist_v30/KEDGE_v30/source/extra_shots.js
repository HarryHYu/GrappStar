const { chromium } = require('playwright'); const http=require('http'),fs=require('fs'),path=require('path');
const OUT=path.resolve(process.argv[2]); const ROOT=__dirname;
const srv=http.createServer((q,s)=>{let u=decodeURIComponent(q.url.split('?')[0]);let f=u==='/'||u==='/index.html'?path.join(ROOT,'viewer.html'):path.join(OUT,u);fs.readFile(f,(e,d)=>{if(e){s.writeHead(404);s.end();return}s.writeHead(200);s.end(d)})}).listen(8772);
(async()=>{const b=await chromium.launch({args:['--use-gl=swiftshader','--enable-webgl','--ignore-gpu-blocklist']});
const p=await b.newPage({viewport:{width:1400,height:900}});
await p.route('**/three.min.js',r=>r.fulfill({path:path.join(ROOT,'vendor/three.min.js')}));
await p.goto('http://localhost:8772/index.html');await p.waitForFunction(()=>window.AUD&&AUD.ready(),null,{timeout:120000});
await p.evaluate(()=>document.body.classList.add('shot'));
const S=[['exploded',`AUD.flags(true,true,false);AUD.cut('y',999);AUD.pose(0,0);document.querySelector('#exp').value=100;pose();AUD.cam([75,0,0],420,-0.9,0.45);`],
['tray_populated',`document.querySelector('#exp').value=0;AUD.flags(false,true,false);AUD.pose(0,0);AUD.cam([85,0,0],230,-1.1,0.55);`],
['pod_section',`AUD.flags(true,true,false);AUD.pose(0,0);AUD.cut('z',7.0);AUD.cam([80,30,7],150,-1.5707,1.45);`],
['kick_hub',`AUD.flags(true,true,false);AUD.cut('y',999);AUD.pose(0,0);AUD.cam([31,15,27],90,0.35,0.35);`],
['deployed_side',`AUD.flags(true,true,true);AUD.cut('y',999);AUD.pose(J.P.KICK_DEG,J.P.DEPLOY_DEG);AUD.cam([40,0,-5],380,-1.5707,0.05);`]];
for(const [n,c] of S){await p.evaluate(c);await p.waitForTimeout(500);await p.screenshot({path:path.join(OUT,'img_'+n+'.png')});}
await b.close();srv.close();})().catch(e=>{console.error(e);process.exit(1)});
