import {expect,test, type Page} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {ROBOT_URDF} from "../../src/test/robotUrdf";

test("shell is a draft until Save, then shared across fresh browsers, Sensors and Map",async({page,browser})=>{
    let settings:Record<string,unknown>={mower_model:"YardForce500",mower_visual_style:"yardforce",lidar_enabled:false,datum_lat:48.1,datum_lon:11.5};
    const writes:unknown[]=[];
    const forbidden:string[]=[];
    const setup=async(p:Page)=>{
        await p.addInitScript(()=>{
            localStorage.setItem("mowglinext.lang","en");
            // A stale browser must never win over the saved mower configuration.
            localStorage.setItem("mowgli.robot-visual.v1",JSON.stringify({style:"utility",transparent:true}));
        });
        await installMockBackend(p,{...SCENARIOS[0],topics:{...SCENARIOS[0].topics,robotDescription:{data:ROBOT_URDF},pose:{pose:{pose:{position:{x:0,y:0,z:0}}},motion_heading:Math.PI/2},map:{dock_x:0,dock_y:0,dock_heading:Math.PI/2,working_area:[]}},rest:{
            "/api/config/keys/get":{"gui.map.mower.appearance":"urdf"},
        }});
        await p.route("**/api/settings/yaml",async route=>{
            if(route.request().method()==="POST"){
                const patch=route.request().postDataJSON(); writes.push(patch);settings={...settings,...patch};
            }
            await route.fulfill({json:settings});
        });
        p.on("request",req=>{if(req.method()==="POST" && /\/(params|containers|mowglinext\/call)/.test(req.url()))forbidden.push(req.url());});
        await p.route("https://api.mapbox.com/**",route=>route.request().url().includes("/styles/")
            ? route.fulfill({json:{version:8,sources:{},glyphs:"https://api.mapbox.com/fonts/v1/mapbox/{fontstack}/{range}.pbf",layers:[{id:"background",type:"background",paint:{"background-color":"#112820"}}]}}) : route.fulfill({body:Buffer.from([10,0])}));
    };
    await setup(page);
    await page.goto("/#/settings?section=hardware");
    const top=page.getByTestId("mower-top");
    await expect(top.locator("[data-mower-style]")).toHaveAttribute("data-mower-style","yardforce");
    await page.getByRole("combobox",{name:"Body style"}).press("ArrowDown");
    await page.getByText("Worx-inspired",{exact:true}).last().click();
    await page.getByRole("combobox",{name:"Body style"}).press("Escape");
    await expect(top.locator("[data-mower-style]")).toHaveAttribute("data-mower-style","worx");
    expect(writes).toEqual([]);
    await page.reload();
    await expect(top.locator("[data-mower-style]")).toHaveAttribute("data-mower-style","yardforce");
    await page.getByRole("combobox",{name:"Body style"}).press("ArrowDown");
    await page.getByText("Worx-inspired",{exact:true}).last().click();
    await page.getByRole("combobox",{name:"Body style"}).press("Escape");
    await page.getByRole("button",{name:"Suggested",exact:true}).click();
    await page.getByRole("button",{name:/Save \(/}).click();
    await expect(page.getByRole("button",{name:/Saved$/})).toBeDisabled();
    expect(writes).toEqual([{mower_visual_style:"worx",mower_visual_colour:"#e97822"}]);
    expect(forbidden).toEqual([]);
    expect(await page.evaluate(()=>sessionStorage.getItem("mowgli.pendingRosRestart"))).not.toBe("true");
    const context=await browser.newContext();
    try {
        const other=await context.newPage();await setup(other);
        await other.goto(new URL("/#/settings?section=hardware",page.url()).href);
        await expect(other.getByTestId("mower-top").locator("[data-mower-style]")).toHaveAttribute("data-mower-style","worx");
        await other.goto(new URL("/#/settings?section=sensors",page.url()).href);
        await expect(other.getByTestId("mower-side").locator("[data-mower-style]")).toHaveAttribute("data-mower-style","worx");
        await other.goto(new URL("/#/map",page.url()).href);
        const marker=other.getByTestId("assembled-mower-marker");
        await expect(marker.locator("[data-mower-style]")).toHaveAttribute("data-mower-style","worx");
        await expect(marker.locator('[data-layer="shell"]')).toHaveAttribute("opacity","1");
        const art=marker.locator('[data-layer="shell-art"]');
        await expect(art).toHaveAttribute("data-paint-colour","#e97822");
        await expect(art).toHaveAttribute("data-paint-ready","true");
        const size=await art.locator("image").evaluate(async el=>{const img=new Image();img.src=el.getAttribute("href")!;await img.decode();return Math.max(img.width,img.height);});
        expect(size).toBeLessThanOrEqual(384);
    } finally {await context.close();}
});
