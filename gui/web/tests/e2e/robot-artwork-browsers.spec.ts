import {mkdirSync} from "node:fs";
import {test,expect,type Page} from "@playwright/test";

import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {ROBOT_URDF} from "../../src/test/robotUrdf";
// Test raster output: Firefox can report a correct SVG transform while drawing
// the nested image in the wrong place. CSS zoom exercises layout scaling; it is
// not a substitute for a real browser-toolbar zoom or a physical Safari device.
async function pixelCentre(page:Page,png:Buffer) {
    return page.evaluate(async(src)=>{
        const image=new Image();image.src=src;await image.decode();
        const canvas=document.createElement("canvas");canvas.width=image.width;canvas.height=image.height;
        const ctx=canvas.getContext("2d")!;ctx.drawImage(image,0,0);
        const pixels=ctx.getImageData(0,0,canvas.width,canvas.height).data;
        let left=canvas.width,top=canvas.height,right=-1,bottom=-1;
        for(let y=0;y<canvas.height;y++)for(let x=0;x<canvas.width;x++) {
            const i=(y*canvas.width+x)*4;
            if((pixels[i]+pixels[i+1]+pixels[i+2])/3>35){left=Math.min(left,x);right=Math.max(right,x);top=Math.min(top,y);bottom=Math.max(bottom,y);}
        }
        return right<0 ? null : {x:(left+right+1)/2,y:(top+bottom+1)/2};
    },`data:image/png;base64,${png.toString("base64")}`);
}
for(const width of [390,768,1440,2046])for(const zoom of [.8,1,1.25,1.5])test(`width ${width} zoom ${zoom}`,async({page,browser},info)=>{
    const out=info.outputPath("captures");
    mkdirSync(out,{recursive:true});
    await page.setViewportSize({width,height:1200});
    await page.addInitScript(()=>localStorage.setItem("mowglinext.lang","en"));
    const base=SCENARIOS[0];
    await installMockBackend(page,{...base,topics:{...base.topics,robotDescription:{data:ROBOT_URDF}},rest:{...base.rest,"/api/settings/yaml":{lidar_enabled:true,mower_visual_style:"yardforce",mower_visual_colour:"#e97822",chassis_length:.6,chassis_width:.45,chassis_height:.19,chassis_z_offset:-.05,chassis_center_x:.18,lidar_x:.176,lidar_y:0,lidar_z:.3,lidar_yaw:-Math.PI/2,gps_x:.4,gps_y:.001,gps_z:.2,imu_x:.381,imu_y:-.045,imu_z:0,imu_yaw:3.126}}});
    await page.goto("/#/settings?section=sensors");
    await expect(page.locator('[data-sensor="lidar"]').first()).toBeVisible();
    await page.evaluate(async()=>{await Promise.all(Array.from(document.querySelectorAll("image")).map(e=>{const im=new Image();im.src=e.getAttribute("href")!;return im.decode();}));});

    await page.evaluate(z=>document.documentElement.style.zoom=String(z),zoom);
    const editor=page.getByTestId("sensor-top-editor");
    await editor.scrollIntoViewIfNeeded();

    for(const yaw of [-90,45]) {
        const input=page.getByRole("spinbutton",{name:/LiDAR.*Yaw/});
        await input.fill(String(yaw));await input.press("Tab");
        for(const id of ["lidar","gps","imu"]) {
            const control=page.locator(`[data-sensor-control="${id}"]`);
            await control.locator('circle[fill="transparent"]').first().dispatchEvent('mouseenter');

            if(yaw===-90 && id==='lidar') await editor.screenshot({path:`${out}/hover.png`});
            // Visibility-only isolation preserves the actual nested SVG, transforms and filter.
            const style=await page.addStyleTag({content:`[data-testid="sensor-top-editor"]{background:#000!important} [data-testid="sensor-top-editor"] *{visibility:hidden!important} [data-sensor="${id}"], [data-sensor="${id}"] *{visibility:visible!important} [data-sensor-highlight], [data-sensor-highlight] *{visibility:hidden!important}`});
            await editor.scrollIntoViewIfNeeded();
            await control.evaluate(e=>e.querySelector('circle[fill="transparent"]')!.setAttribute('data-position-marker','true'));
            const file=`${out}/${id}-${yaw}.png`;const raster=await editor.screenshot({path:file});
            await style.evaluate((e,id)=>e.textContent=`[data-testid="sensor-top-editor"]{background:#000!important} [data-testid="sensor-top-editor"] *{visibility:hidden!important} [data-sensor-control="${id}"] > circle[data-position-marker]{visibility:visible!important;fill:white!important}`,id);
            const markerFile=`${out}/${id}-${yaw}-marker.png`;const marker=await editor.screenshot({path:markerFile});
            const drawn=await pixelCentre(page,raster);const expected=await pixelCentre(page,marker);
            expect(drawn,`${id} raster must be visible (${browser.version()})`).not.toBeNull();
            expect(expected,`${id} position marker must be visible`).not.toBeNull();
            expect(Math.abs(drawn!.x-expected!.x),`${id} horizontal shift at yaw ${yaw}`).toBeLessThan(1.75);
            expect(Math.abs(drawn!.y-expected!.y),`${id} vertical shift at yaw ${yaw}`).toBeLessThan(1.75);
            await style.evaluate(e=>e.remove());
        }
    }
});
