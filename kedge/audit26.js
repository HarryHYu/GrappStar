const { chromium } = require('playwright');
(async()=>{const b=await chromium.launch({executablePath:'/opt/pw-browsers/chromium'});const p=await b.newPage({viewport:{width:1000,height:650}});
await p.goto('http://localhost:8770/index.html');await p.waitForTimeout(9000);
await p.evaluate(()=>{document.querySelector('#ui').style.display='none';});
const S=(id,v)=>`{const e=document.querySelector('#${id}');e.value=${v};e.dispatchEvent(new Event('input'));}`;
const shots=[];
for(const k of [0,45,90,135,180,200]) shots.push(['kick'+k,`${S('kick',k)}${S('dep',0)}fly.pos.set(60,-260,30);fly.yaw=1.57;fly.pitch=-0.1;`]);
shots.push(['top',`${S('kick',0)}fly.pos.set(90,0,230);fly.yaw=0;fly.pitch=-1.5;`]);
shots.push(['top_dep',`${S("kick",200)}${S('dep',38)}fly.pos.set(40,0,230);fly.yaw=0;fly.pitch=-1.5;`]);
shots.push(['under_dep',`document.querySelector('#ground').checked=false;document.querySelector('#ground').dispatchEvent(new Event('input'));fly.pos.set(40,0,-200);fly.yaw=0;fly.pitch=1.5;`]);
shots.push(['front_dep',`fly.pos.set(-200,-20,-10);fly.yaw=0.1;fly.pitch=0.05;`]);
shots.push(['rear_fold',`${S('kick',0)}${S('dep',0)}fly.pos.set(330,-30,30);fly.yaw=3.05;fly.pitch=-0.15;`]);
shots.push(['iso_fold',`document.querySelector('#ground').checked=true;document.querySelector('#ground').dispatchEvent(new Event('input'));fly.pos.set(-80,-160,100);fly.yaw=0.8;fly.pitch=-0.5;`]);
for(const [n,code] of shots){await p.evaluate(code);await p.waitForTimeout(300);await p.screenshot({path:'d_'+n+'.png'});}
await b.close();})();
