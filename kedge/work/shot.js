const { chromium } = require('playwright');
(async()=>{
  const [,,url,out,w,h,wait]=process.argv;
  const b=await chromium.launch({args:['--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const p=await b.newPage({viewport:{width:+w||1600,height:+h||1000}});
  const errs=[];p.on('pageerror',e=>errs.push(e.message));p.on('console',m=>{if(m.type()==='error')errs.push(m.text())});
  await p.goto(url);await p.waitForTimeout(+wait||4000);
  await p.screenshot({path:out});console.log('errors:',errs.slice(0,5));
  await b.close();
})();
