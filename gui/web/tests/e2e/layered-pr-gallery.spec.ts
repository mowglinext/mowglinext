import {test,expect} from "@playwright/test";
import {mkdirSync} from "node:fs";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {ROBOT_URDF} from "../../src/test/robotUrdf";
import {MOWER_MODELS} from "../../src/constants/mowerModels";
const output="screenshots.local/layered-mower/pr";
// Illustrative custom geometry, not a measured mower or a shipped preset.
const custom={mower_model:"CUSTOM",robot_name:"Angular custom concept",chassis_length:.66,chassis_width:.46,
    chassis_height:.18,chassis_z_offset:-.06,chassis_center_x:.18,wheel_radius:.10,wheel_width:.05,
    wheel_track:.30,wheel_x_offset:0,caster_radius:.04,caster_track:.28,caster_x_offset:.39,blade_radius:.115,tool_width:.23,ticks_per_meter:300,
    gps_x:.16,gps_y:0,gps_z:0.13,lidar_x:.32,lidar_y:0,lidar_z:.131,lidar_yaw:0,
    imu_x:.035,imu_y:-.09,imu_z:.005,imu_yaw:0};
for(const example of ["yardforce500","custom-angular"] as const){
    test(`PR screenshots ${example}`,async({page})=>{
        test.setTimeout(60000);
        mkdirSync(output,{recursive:true});
        await page.setViewportSize({width:1440,height:1250});
        const values=example === "yardforce500"
            ? {mower_model:"YardForce500",robot_name:"Yardforce 500",...MOWER_MODELS[0].defaults,
                // Example installation: preset chassis/drive geometry, with
                // inset casters and a plausible roof-mounted sensor layout.
                caster_x_offset:.40,caster_track:.30,
                gps_x:.15,gps_y:0,gps_z:0.148,
                lidar_x:.31,lidar_y:0,lidar_z:.139,lidar_yaw:0,
                imu_x:.04,imu_y:-.09,imu_z:.015,imu_yaw:0} : custom;
        await page.addInitScript(({style})=>{
            localStorage.setItem("mowglinext.lang","en");
            localStorage.setItem("mowgli.robot-visual.v1",JSON.stringify({style,transparent:false}));
        },{style:example === "yardforce500" ? "yardforce" : "utility"});
        await installMockBackend(page,{...SCENARIOS[0],topics:{...SCENARIOS[0].topics,robotDescription:{data:ROBOT_URDF}},
            rest:{"/api/settings/yaml":values}},{liveStatusIntervalMs:1000});
        await page.goto("/#/settings?section=hardware");
        const preview=page.getByTestId("mower-preview");
        await expect(preview).toBeVisible();
        await page.getByRole("radiogroup").evaluate(node=>node.scrollIntoView({block:"start"}));
        await page.evaluate(async()=>{
            await Promise.all([...document.querySelectorAll('image')].map(e=>{const img=new Image();img.src=e.getAttribute('href')!;return img.decode();}));
        });
        await expect(preview.locator('[data-layer="shell-art"]').first()).toHaveAttribute("width",String(values.chassis_width));
        await page.waitForTimeout(500); // Let scroll and toggle transitions settle for the capture.
        await page.screenshot({path:`${output}/${example}-hardware.png`,fullPage:true});
        if(example === "custom-angular"){
            await preview.getByRole("switch").click();
            await expect(preview.locator('[data-layer="shell"]').first()).toHaveAttribute("opacity","0.24");
            await page.waitForTimeout(500); // Let scroll and toggle transitions settle for the capture.
            await page.screenshot({path:`${output}/${example}-transparent.png`,fullPage:true});
        }
        await page.goto("/#/settings?section=sensors");
        await expect(page.getByTestId("mower-side")).toBeVisible();
        if(example === "yardforce500")await page.getByRole("switch",{name:"Transparent shell"}).click();
        await page.getByTestId("sensor-placement").evaluate(node=>node.scrollIntoView({block:"start"}));
        await page.waitForTimeout(500); // Let scroll and toggle transitions settle for the capture.
        await page.screenshot({path:`${output}/${example}-sensors.png`,fullPage:true});
        if(example === "custom-angular"){
            await page.setViewportSize({width:390,height:844});
            await page.getByTestId("mower-side").evaluate(node=>node.scrollIntoView({block:"center",behavior:"instant"}));
            await page.waitForTimeout(500); // Let scroll and toggle transitions settle for the capture.
            await page.screenshot({path:`${output}/${example}-mobile.png`,fullPage:true});
        }
    });
}
