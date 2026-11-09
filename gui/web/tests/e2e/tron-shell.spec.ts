import {test,expect} from "@playwright/test";
import {mkdirSync} from "node:fs";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {ROBOT_URDF} from "../../src/test/robotUrdf";
const shots="screenshots.local/tron";
test("Tron silver cover recolours in both projections",async({page})=>{
    mkdirSync(shots,{recursive:true});
    await page.setViewportSize({width:1440,height:1100});
    await page.goto("/tests/e2e/fixtures/tron-gallery.html");
    await expect(page.locator('[data-layer="shell-art"][data-paint-ready="true"]')).toHaveCount(6);
    await page.evaluate(async()=>{await Promise.all([...document.querySelectorAll('image')].map(el=>{const im=new Image();im.src=el.getAttribute('href')!;return im.decode();}));});
    // Read real painted pixels: a loaded but wrongly sized SVG can silently
    // produce a completely clear mask despite data-paint-ready being true.
    const samples=await page.evaluate(async()=>{
        const {paintedShell}=await import("/src/components/robot/usePaintedShell.ts");
        const {SHELLS}=await import("/src/components/robot/shellAssets.ts");
        const out=[];
        for(const view of ["top","side"] as const) for(const artwork of ["full","map"] as const) {
            const img=new Image();img.src=await paintedShell("tron",view,artwork,"#167bc1");await img.decode();
            const c=document.createElement("canvas");c.width=img.width;c.height=img.height;
            const ctx=c.getContext("2d")!;ctx.drawImage(img,0,0);
            const [x,y,w,h]=SHELLS.tron[view];
            const point=view==="top" ? [410,420] : [1100,420];
            const px=ctx.getImageData(Math.floor((point[0]-x)/w*c.width),Math.floor((point[1]-y)/h*c.height),1,1).data;
            out.push([...px]);
        }
        return out;
    });
    for(const rgba of samples){expect(rgba[3]).toBeGreaterThan(240);expect(rgba[2]-rgba[0]).toBeGreaterThan(25);}
    await page.locator("main").screenshot({path:shots+"/colours.png"});
});
// Photo-proportioned demo only; no manufacturer dimensions/preset are introduced.
const settings={mower_model:"CUSTOM",mower_visual_style:"tron",mower_visual_colour:"#3f8e79",lidar_enabled:false,
    chassis_length:.75,chassis_width:.59,chassis_height:.297,chassis_center_x:.28,chassis_z_offset:-.070,
    wheel_radius:.13,wheel_width:.065,wheel_track:.53,wheel_x_offset:0,caster_radius:.09,caster_track:.43,caster_x_offset:.52,
    gps_x:.30,gps_y:0,gps_z:.165,
    blade_radius:.11,tool_width:.22,dock_visual_length:.90,dock_visual_width:.68,dock_visual_x_offset:0,dock_visual_y_offset:0};
test("Tron hardware and transparent sensors share artwork and standard dock",async({page})=>{
    mkdirSync(shots,{recursive:true});
    await page.setViewportSize({width:1440,height:1050});
    await page.addInitScript(()=>{localStorage.setItem("mowglinext.lang","en");localStorage.setItem("mowgli.display-mode","efficient");});
    await installMockBackend(page,{...SCENARIOS[0],rest:{"/api/settings/yaml":settings},topics:{...SCENARIOS[0].topics,robotDescription:{data:ROBOT_URDF}}});
    await page.goto("/#/settings?section=hardware");
    const shell=page.getByTestId("mower-preview").locator('[data-layer="shell-art"]');
    await expect(shell).toHaveCount(2);
    await expect(page.getByTestId("mower-preview").locator('[data-dock-style="standard"]')).toHaveCount(2);
    for(const item of await shell.all()) await expect(item).toHaveAttribute("data-paint-ready","true");
    await expect(page.getByTestId("mower-preview").locator('[data-sensor="lidar"]')).toHaveCount(0);
    await page.getByTestId("mower-preview").screenshot({path:shots+"/hardware.png"});
    const solid=await shell.evaluateAll(nodes=>nodes.map(n=>n.outerHTML));
    await page.getByRole("switch",{name:"Transparent shell"}).click();
    expect(await shell.evaluateAll(nodes=>nodes.map(n=>n.outerHTML))).toEqual(solid);
    await page.setViewportSize({width:1440,height:1550});
    await page.goto("/#/settings?section=sensors");
    await expect(page.locator('[data-layer="shell"]').first()).toHaveAttribute("opacity","0.24");
    await expect(page.locator('[data-layer="shell-art"][data-paint-ready="true"]')).toHaveCount(2);
    await page.screenshot({path:shots+"/sensors.png"});
});
