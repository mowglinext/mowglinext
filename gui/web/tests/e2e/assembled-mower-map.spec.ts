import {expect,test} from "@playwright/test";
import {mkdirSync} from "node:fs";
test("assembled URDF marker keeps metre dimensions and rear-axle anchor across map rotations",async({page})=>{
    await page.route("https://api.mapbox.com/**",route=>route.fulfill({json:{}}));
    await page.addInitScript(()=>localStorage.setItem("mowgli.robot-visual.v1",JSON.stringify({style:"yardforce",transparent:true})));
    await page.goto("/tests/e2e/fixtures/assembled-mower.html");
    await expect(page.getByTestId("assembled-mower-marker")).toBeVisible();
    await page.waitForFunction(()=>!!window.assembledMowerTest);
    const errors=await page.evaluate(async()=>{
        const {map,setHeading}=window.assembledMowerTest!;
        const svg=document.querySelector<SVGSVGElement>('[data-testid="assembled-mower-marker"]')!;
        const shell=svg.querySelector<SVGRectElement>("[data-shell-bounds]")!;
        const center=map.getCenter(), R=6378137;
        const errors:number[]=[];
        for(const state of [{zoom:23,bearing:0,pitch:0,heading:0},{zoom:24,bearing:90,pitch:0,heading:Math.PI/2},{zoom:24,bearing:45,pitch:50,heading:Math.PI},{zoom:23,bearing:90,pitch:50,heading:-Math.PI/2}]){
            map.jumpTo(state);setHeading(state.heading);
            await new Promise<void>(resolve=>requestAnimationFrame(()=>requestAnimationFrame(()=>resolve())));
            const actual=shell.getBoundingClientRect();
            const pixels=[];
            for(const x of [-.12,.48]) for(const y of [-.225,.225]){
                const east=x*Math.cos(state.heading)-y*Math.sin(state.heading);
                const north=x*Math.sin(state.heading)+y*Math.cos(state.heading);
                pixels.push(map.project([center.lng+east/(R*Math.cos(center.lat*Math.PI/180))*180/Math.PI,center.lat+north/R*180/Math.PI]));
            }
            const canvas=map.getCanvas().getBoundingClientRect();
            errors.push(Math.abs(actual.left-canvas.left-Math.min(...pixels.map(p=>p.x))),
                Math.abs(actual.right-canvas.left-Math.max(...pixels.map(p=>p.x))),
                Math.abs(actual.top-canvas.top-Math.min(...pixels.map(p=>p.y))),
                Math.abs(actual.bottom-canvas.top-Math.max(...pixels.map(p=>p.y))));
        }
        map.jumpTo({zoom:24,bearing:0,pitch:0});setHeading(Math.PI/2);
        return errors;
    });
    for(const error of errors) expect(error).toBeLessThan(3);
    mkdirSync("screenshots.local/layered-mower",{recursive:true});
    await expect(page.getByTestId("styled-dock-marker")).toBeVisible();
    await page.screenshot({path:"screenshots.local/layered-mower/map-urdf.png"});
    await page.evaluate(()=>{
        localStorage.setItem("mowgli.robot-visual.v1",JSON.stringify({style:"yardforce",transparent:false}));
        window.dispatchEvent(new Event("mowgli-robot-visual"));
    });
    await expect(page.getByTestId("assembled-mower-marker").locator('[data-layer="shell"]')).toHaveAttribute("opacity","1");
    await page.screenshot({path:"screenshots.local/layered-mower/map-docked-solid.png"});
});
