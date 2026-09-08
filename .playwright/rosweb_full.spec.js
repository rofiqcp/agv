const {test,expect}=require("@playwright/test");
const fs=require("fs");
const path=require("path"); const ROOT=process.env.AGV_ROOT||path.resolve(__dirname,".."); const OUT=path.join(ROOT,"data/playwright_rosweb"); fs.mkdirSync(OUT,{recursive:true});

test("ROS Web all tabs and subtabs",async({page})=>{
  const errs=[]; const badResp=[];
  page.on("pageerror",e=>errs.push("pageerror: "+e.message));
  page.on("console",m=>{if(m.type()==="error") errs.push("console: "+m.text())});
  page.on("response",r=>{if(r.status()>=400) badResp.push(`${r.status()} ${r.url()}`)});
  await page.goto("http://127.0.0.1:5000",{waitUntil:"domcontentloaded"});
  await page.waitForTimeout(1200);
  await page.evaluate(()=>activatePage("overview",false));
  await expect(page.locator("#page-overview")).toHaveClass(/active/);
  const state=await page.evaluate(()=>fetch("/api/state").then(r=>r.json()));
  expect(state["system.motion_ready"]).toBe(true);
  expect(state["connected.esc_feedback"]).toBe(true);
  expect(state["connected.imu"]).toBe(true);
  expect(state["connected.camera"]).toBe(true);
  const cfg=await page.evaluate(()=>fetch("/api/config").then(r=>r.json()));
  expect(cfg.files.imu.data.data_imu_node.ros__parameters.stationary_calibration_valid).toBe(true);

  const topDomains=["overview","navigation","perception","esc"];
  for(const d of topDomains){
    await page.locator(`.top-domain[data-domain="${d}"]`).click();
    await page.waitForTimeout(250);
    await page.screenshot({path:`${OUT}/domain_${d}.png`,fullPage:true});
  }

  // Exercise every concrete page section without invoking control actions.
  for(const p of ["overview","reports","diagnostics","configuration","navigation","perception","esc","esc-status","calibration","tuning","sensors"]){
    const exists=await page.locator(`#page-${p}`).count(); if(!exists) continue;
    await page.evaluate(x=>activatePage(x,false),p); await page.waitForTimeout(80);
    await expect(page.locator(`#page-${p}`)).toHaveClass(/active/);
  }

  // Exercise all staged tuning/report leaves for all domains.
  let leafCount=0;
  for(const d of ["navigation","perception","esc"]){
    await page.locator(`.top-domain[data-domain="${d}"]`).click(); await page.waitForTimeout(150);
    const ids=await page.locator("#contextNav [data-exp-id]").evaluateAll(es=>es.map(e=>e.dataset.expId));
    for(const id of ids){
      await page.evaluate(x=>openDomainTuning(x),id); await page.waitForTimeout(35);
      await expect(page.locator("#page-experiments")).toHaveClass(/active/); leafCount++;
    }
    if(d==="navigation"){
      const rids=await page.locator("#contextNav [data-report-id]").evaluateAll(es=>es.map(e=>e.dataset.reportId));
      for(const id of rids){await page.evaluate(x=>openNavigationReport(x),id); await page.waitForTimeout(35); leafCount++;}
    }
  }
  expect(leafCount).toBeGreaterThan(50);

  await page.evaluate(()=>activatePage("calibration",false)); await page.waitForTimeout(300);
  await expect(page.locator("#calImuStationary")).toHaveText("PASS");
  await expect(page.locator("#calImuEvidence")).toContainText("93 sampel");
  await page.evaluate(()=>activatePage("tuning",false));
  await expect(page.locator("#tuneMppiReady")).toHaveText("READY");
  await page.evaluate(()=>activatePage("perception",false));
  await expect(page.locator("#yoloToggleBtn")).toBeVisible();
  await expect(page.locator("#perceptionSafetyBypassBtn")).toBeVisible();

  // Mobile responsive smoke.
  await page.setViewportSize({width:390,height:844}); await page.reload({waitUntil:"domcontentloaded"}); await page.waitForTimeout(800);
  await expect(page.locator("#menuButton")).toBeVisible(); await page.locator("#menuButton").click(); await expect(page.locator("#sidebar")).toHaveClass(/open/);

  fs.writeFileSync(`${OUT}/result.json`,JSON.stringify({leafCount,errs,badResp,state:{motion_ready:state["system.motion_ready"],nav2_ready:state["system.nav2_ready"],esc:state["connected.esc_feedback"],imu:state["connected.imu"],camera:state["connected.camera"]}},null,2));
  expect(errs,errs.join("\n")).toEqual([]);
  expect(badResp,badResp.join("\n")).toEqual([]);
});
