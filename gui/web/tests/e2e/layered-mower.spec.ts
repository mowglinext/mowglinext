import {expect,test} from "@playwright/test";
import {mkdirSync} from "node:fs";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {SHELLS} from "../../src/components/robot/shellAssets";
import {ROBOT_URDF} from "../../src/test/robotUrdf";
const base=SCENARIOS[0];
const shots="screenshots.local/layered-mower";
const settings={mower_model:"YardForce500",chassis_length:.6,chassis_width:.45,chassis_height:.19,
    chassis_center_x:.18,wheel_radius:.1,wheel_width:.04,wheel_track:.325,wheel_x_offset:0,
    caster_radius:.03,caster_track:.36,blade_radius:.09,gps_x:.3,gps_y:0,gps_z:.2,
    lidar_x:0,lidar_y:.024,lidar_z:.3,lidar_yaw:3.1408,imu_x:.18,imu_y:-.195,imu_z:.095};
test.beforeEach(async({page})=>{
    mkdirSync(shots,{recursive:true});
    await page.setViewportSize({width:1440,height:1800});
    await page.addInitScript(()=>localStorage.setItem("mowglinext.lang","en"));
    await installMockBackend(page,{...base,topics:{...base.topics,robotDescription:{data:ROBOT_URDF}},
        rest:{...base.rest,"/api/settings/yaml":settings}});
});
test("four styles keep the same shell and sensor positions in transparent views",async({page})=>{
    await page.goto("/#/settings?section=hardware");
    const preview=page.getByTestId("mower-preview");
    await expect(preview.locator('[data-layer="shell-art"]')).toHaveCount(2);
    for(const [id,label] of [["rounded","Rounded"],["sculpted","Sculpted"],["utility","Utility"],["yardforce","Yardforce-inspired"]]){
        await page.getByRole("combobox",{name:"Body style"}).press("ArrowDown");
        await page.getByText(label,{exact:true}).last().click();
        await page.getByRole("combobox",{name:"Body style"}).press("Escape");
        await expect(page.locator(".ant-select-dropdown:visible")).toHaveCount(0);
        await expect(preview.locator('[data-mower-style="'+id+'"]')).toHaveCount(2);
        await page.evaluate(async()=>{
            const sources=Array.from(document.querySelectorAll('image')).map(e=>e.getAttribute("href")!).filter(Boolean);
            await Promise.all(sources.map(src=>{const img=new Image();img.src=src;return img.decode();}));
        });
        const art=await preview.locator('[data-layer="shell-art"]').evaluateAll(nodes=>nodes.map(n=>n.outerHTML));
        const mounts=await preview.locator("[data-sensor]").evaluateAll(nodes=>nodes.map(n=>n.outerHTML));
        await preview.screenshot({path:shots+"/"+id+"-solid.png"});
        await preview.getByRole("switch").click();
        await expect(preview.locator('[data-layer="shell"]').first()).toHaveAttribute("opacity","0.24");
        expect(await preview.locator('[data-layer="shell-art"]').evaluateAll(nodes=>nodes.map(n=>n.outerHTML))).toEqual(art);
        expect(await preview.locator("[data-sensor]").evaluateAll(nodes=>nodes.map(n=>n.outerHTML))).toEqual(mounts);
        await preview.screenshot({path:shots+"/"+id+"-transparent.png"});
        await preview.getByRole("switch").click();
    }
    await page.screenshot({path:shots+"/hardware-desktop.png",fullPage:true});
});
test("sensor edits update both projections and appearance survives navigation on mobile",async({page})=>{
    await page.setViewportSize({width:390,height:844});
    await page.goto("/#/settings?section=hardware");
    await page.getByRole("combobox",{name:"Body style"}).press("ArrowDown");
    await page.getByText("Yardforce-inspired",{exact:true}).last().click();
    await page.getByRole("combobox",{name:"Body style"}).press("Escape");
        await expect(page.locator(".ant-select-dropdown:visible")).toHaveCount(0);
    await page.goto("/#/settings?section=sensors");
    await expect(page.getByTestId("mower-side")).toBeVisible();
    await expect(page.getByRole("combobox",{name:"Body style"})).toHaveCount(0);
    await page.getByRole("switch",{name:"Transparent shell"}).click();
    const gps=page.getByRole("spinbutton",{name:/GPS.*X/});
    await gps.fill("0.36");
    await gps.press("Tab");
    await expect(page.locator('[data-sensor="gps"]').first()).toHaveAttribute("data-x","0.36");
    await expect(page.locator('[data-sensor="gps"]').last()).toHaveAttribute("data-x","0.36");
    await page.screenshot({path:shots+"/sensors-mobile.png",fullPage:true});
    expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
    await page.goto("/#/settings?section=hardware");
    await expect(page.getByTestId("mower-top").locator("[data-mower-style]")).toHaveAttribute("data-mower-style","yardforce");
    await expect(page.getByRole("switch",{name:"Transparent shell"})).toBeChecked();
    await page.getByTestId("mower-top").scrollIntoViewIfNeeded();
    await page.screenshot({path:shots+"/hardware-mobile.png"});
    await page.getByTestId("mower-side").scrollIntoViewIfNeeded();
    await page.screenshot({path:shots+"/hardware-mobile-side.png"});
});
test("sensors desktop renders both views without console errors",async({page})=>{
    const errors:string[]=[];page.on("pageerror",e=>errors.push(e.message));
    await page.goto("/#/settings?section=sensors");
    await expect(page.getByTestId("mower-side")).toBeVisible();
    await page.getByRole("switch",{name:"Transparent shell"}).click();
    await page.locator("[data-testid=sensor-placement]").screenshot({path:shots+"/sensors-desktop.png"});
    expect(errors).toEqual([]);
});

test("sensor drag keeps metre coordinates when the preview is resized",async({page})=>{
    await page.setViewportSize({width:900,height:1000});
    await page.goto("/#/settings?section=sensors");
    const hit=page.locator('[data-sensor-control="gps"] circle[fill="transparent"]').first();
    await hit.scrollIntoViewIfNeeded();
    const b=await hit.boundingBox();
    expect(b).not.toBeNull();
    await page.mouse.move(b!.x+b!.width/2,b!.y+b!.height/2);
    await page.mouse.down();
    await page.mouse.move(b!.x+b!.width/2,b!.y+b!.height/2-30,{steps:5});
    await page.mouse.up();
    const x=Number(await page.locator('[data-sensor="gps"]').first().getAttribute("data-x"));
    expect(x).toBeGreaterThan(.32);
    expect(x).toBeLessThan(.5);
    await expect(page.locator('[data-sensor="gps"]').last()).toHaveAttribute("data-x",String(x));
});
test("chassis edits resize the assembly without changing the wheel diameter",async({page})=>{
    await page.goto("/#/settings?section=hardware");
    await page.getByText("Chassis & Geometry",{exact:true}).click();
    await page.locator("#setting-chassis_length").fill("0.8");
    await page.locator("#setting-chassis_length").press("Tab");
    await expect(page.getByTestId("mower-top").locator('[data-layer="shell-art"]')).toHaveAttribute("height","0.8");
    await expect(page.getByTestId("mower-side").locator('[data-layer="shell-art"]')).toHaveAttribute("width","0.8");
    expect(Number(await page.getByRole("spinbutton",{name:/Wheel Radius/}).inputValue())).toBe(.1);
});

test("caster fore/aft setting changes preview and automatic restores chassis placement",async({page})=>{
    await page.goto("/#/settings?section=hardware");
    await page.getByText("Chassis & Geometry",{exact:true}).click();
    await page.getByRole("switch",{name:"Automatic from chassis"}).click();
    const input=page.getByRole("spinbutton",{name:"Caster front/back position, m"});
    await input.fill("0.32");await input.press("Tab");
    await expect(page.getByTestId("mower-top").locator('[data-layer="casters"] > g').first()).toHaveAttribute("transform",/ -0.32\)/);
    await page.getByRole("switch",{name:"Automatic from chassis"}).click();
    await expect(input).toHaveCount(0);
    await expect(page.getByTestId("mower-top").locator('[data-layer="casters"] > g').first()).toHaveAttribute("transform",/ -0.44999999999999996\)| -0.45\)/);
});

// Inspect source alpha outside the configured crop as well. Testing only the
// SVG rectangle would pass even if it represented padding or clipped the body.
test("all shell dimensions describe opaque chassis edges, excluding atlas margins",async({page})=>{
    await page.goto("/#/settings?section=hardware");
    await expect(page.getByTestId("mower-top")).toBeVisible();
    const measured=await page.evaluate(async()=>{
        const result:Record<string,Record<string,number[]>>={};
        for(const style of ["rounded","sculpted","utility","yardforce"]){
            const image=new Image();image.src=`/assets/robots/layered/${style}.png`;await image.decode();
            const canvas=document.createElement("canvas");canvas.width=image.width;canvas.height=image.height;
            const ctx=canvas.getContext("2d",{willReadFrequently:true})!;ctx.drawImage(image,0,0);
            const pixels=ctx.getImageData(0,0,image.width,image.height).data;
            result[style]={};
            // The two isolated projections are separated at x=768 in these
            // source atlases. Scan their whole cells, not just the crop metadata.
            for(const [view,start,end] of [["top",0,768],["side",768,image.width]] as const){
                let minX=image.width,minY=image.height,maxX=-1,maxY=-1;
                for(let y=0;y<image.height;y++)for(let x=start;x<end;x++){
                    // Ignore the antialias fringe; alpha >220 defines solid shell.
                    if(pixels[(y*image.width+x)*4+3]<=220)continue;
                    minX=Math.min(minX,x);minY=Math.min(minY,y);maxX=Math.max(maxX,x);maxY=Math.max(maxY,y);
                }
                result[style][view]=[minX,minY,maxX-minX+1,maxY-minY+1];
            }
        }
        return result;
    });
    for(const [style,asset] of Object.entries(SHELLS)){
        for(const view of ["top","side"] as const){
            expect(measured[style][view],`${style} ${view}: physical crop must touch all four solid edges`).toEqual(asset[view]);
        }
    }
    // The same measured edge crop occupies the configured metric extent.
    const top=page.getByTestId("mower-top").locator('[data-layer="shell-art"]');
    const side=page.getByTestId("mower-side").locator('[data-layer="shell-art"]');
    await expect(top).toHaveAttribute("width","0.45");
    await expect(top).toHaveAttribute("height","0.6");
    await expect(side).toHaveAttribute("width","0.6");
    await expect(side).toHaveAttribute("height","0.19");
});
