import {expect,test} from "@playwright/test";
import {mkdirSync,writeFileSync,readFileSync} from "node:fs";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {ROBOT_URDF} from "../../src/test/robotUrdf";
const shots="screenshots.local/layered-mower";
for(const mobile of [false,true])test(`full app map ${mobile ? "mobile" : "desktop"} with moving URDF assembly`,async({page})=>{
    test.setTimeout(60000);
    mkdirSync(shots,{recursive:true});
    await page.setViewportSize(mobile ? {width:390,height:844} : {width:1440,height:1000});
    await page.addInitScript(()=>{
        localStorage.setItem("mowglinext.lang","en");
        localStorage.setItem("mowgli.display-mode","efficient");
        localStorage.setItem("mowgli.robot-visual.v1",JSON.stringify({style:"yardforce",transparent:false}));
    });
    await page.route("https://api.mapbox.com/**",route=>route.request().url().includes("/styles/")
        ? route.fulfill({json:{version:8,sources:{},glyphs:"https://api.mapbox.com/fonts/v1/mapbox/{fontstack}/{range}.pbf",layers:[{id:"background",type:"background",paint:{"background-color":"#112820"}}]}})
        : route.fulfill({body:Buffer.from([10,0])}));
    const pose=(x:number)=>({pose:{pose:{position:{x,y:0,z:0}}},motion_heading:Math.PI/2});
    const errors:string[]=[];page.on("pageerror",e=>errors.push(e.message));
    await installMockBackend(page,{...SCENARIOS[0],rest:{
        "/api/settings/yaml":{datum_lat:48.1,datum_lon:11.5},
        "/api/config/keys/get":{"gui.map.mower.appearance":"urdf","gui.map.dock.appearance":"styled"},
    },topics:{...SCENARIOS[0].topics,robotDescription:{data:ROBOT_URDF},pose:pose(0),map:{
        dock_x:0,dock_y:-.8,dock_heading:Math.PI/2,
        working_area:[{id:1,name:"Preview lawn",area:{points:[{x:-1.8,y:-1.5},{x:1.8,y:-1.5},{x:1.8,y:1.5},{x:-1.8,y:1.5}]}}],
    }},topicSequences:{pose:[pose(0),pose(.03),pose(.06),pose(.09),pose(.06),pose(.03)]}},{liveStatusIntervalMs:50});
    await page.goto("/#/map");
    await page.addStyleTag({content:readFileSync("node_modules/mapbox-gl/dist/mapbox-gl.css","utf8")});
    const marker=page.getByTestId("assembled-mower-marker");
    await expect(marker).toBeVisible();
    await expect(page.getByTestId("styled-dock-marker")).toBeVisible();
    await page.evaluate(async()=>{
        await Promise.all([...document.querySelectorAll('image')].map(e=>{const img=new Image();img.src=e.getAttribute('href')!;return img.decode();}));
    });
    const cdp=await page.context().newCDPSession(page);
    if(mobile)await cdp.send("Emulation.setCPUThrottlingRate",{rate:4});
    const metrics=await page.evaluate(async()=>{
        const marker=document.querySelector('[data-testid="assembled-mower-marker"]')!;
        const artwork=marker.querySelector('[data-mower-style]')!;
        let mutations=0;const transforms=new Set<string>();const frames:number[]=[];let previous=performance.now();
        const observer=new MutationObserver(records=>mutations+=records.length);
        observer.observe(artwork,{attributes:true,childList:true,subtree:true});
        const deadline=previous+3000;
        await new Promise<void>(resolve=>{
            const step=(now:number)=>{
                frames.push(now-previous);previous=now;transforms.add(marker.closest(".mapboxgl-marker")!.getAttribute("style")!);
                if(now<deadline)requestAnimationFrame(step);else resolve();
            };requestAnimationFrame(step);
        });observer.disconnect();
        frames.sort((a,b)=>a-b);
        const assets=performance.getEntriesByType("resource").filter(e=>e.name.includes("/assets/robots/layered/")) as PerformanceResourceTiming[];
        return {durationMs:3000,artworkMutations:mutations,distinctPlacements:transforms.size,frameMedianMs:frames[Math.floor(frames.length*.5)],frameP95Ms:frames[Math.floor(frames.length*.95)],artworkElements:artwork.querySelectorAll("*").length,
            assetEncodedBytes:assets.reduce((sum,e)=>sum+e.encodedBodySize,0),assetFiles:[...new Set(assets.map(e=>e.name.split("/").pop()))]};
    });
    writeFileSync(`${shots}/map-${mobile ? "mobile" : "desktop"}-metrics.json`,JSON.stringify({baseline:`${process.env.E2E_PRODUCTION ? "production" : "development"} app, local empty Mapbox basemap, mocked 20 Hz pose/status, efficient display mode`,cpuThrottle:mobile?4:1,...metrics},null,2));
    expect(metrics.distinctPlacements).toBeGreaterThan(2);
    expect(metrics.artworkMutations).toBe(0);
    expect(metrics.assetFiles).not.toContain("blade-top.webp");
    expect(metrics.assetFiles).not.toContain("imu-top.webp");
    expect(metrics.assetEncodedBytes).toBeLessThan(150000);
    for(const file of metrics.assetFiles)expect(file).toMatch(/\.webp$/);
    const dimensions=await page.evaluate(async()=>Promise.all([...document.querySelectorAll('image')].map(async e=>{
        const image=new Image();image.src=e.getAttribute('href')!;await image.decode();
        return {width:image.naturalWidth,height:image.naturalHeight};
    })));
    for(const d of dimensions)expect(Math.max(d.width,d.height)).toBeLessThanOrEqual(384);
    expect(errors).toEqual([]);
    await page.screenshot({path:`${shots}/app-map-${mobile ? "mobile" : "desktop"}.png`,fullPage:true});
    await cdp.send("Emulation.setCPUThrottlingRate",{rate:1});
    await page.evaluate(()=>{
        localStorage.setItem("mowgli.robot-visual.v1",JSON.stringify({style:"yardforce",transparent:true}));
        window.dispatchEvent(new Event("mowgli-robot-visual"));
    });
    await expect(marker.locator('[data-layer="shell"]')).toHaveAttribute("opacity","0.24");
    await page.evaluate(async()=>{
        await Promise.all([...document.querySelectorAll('image')].map(e=>{const img=new Image();img.src=e.getAttribute('href')!;return img.decode();}));
    });
    await page.screenshot({path:`${shots}/app-map-${mobile ? "mobile" : "desktop"}-transparent.png`,fullPage:true});
});
