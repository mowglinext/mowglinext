import {test,expect} from "@playwright/test";
import {mkdirSync} from "node:fs";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {ROBOT_URDF} from "../../src/test/robotUrdf";
const shots="screenshots.local/layered-mower/measurements";
const settings={mower_visual_style:"worx",lidar_enabled:false,mower_model:"CUSTOM",chassis_length:.54,chassis_width:.36,chassis_height:.14,chassis_z_offset:-.07,
    chassis_center_x:.16,wheel_radius:.11,wheel_width:.04,wheel_track:.32,wheel_x_offset:0,caster_radius:.025,
    caster_track:.22,caster_x_offset:.34,blade_radius:.09,tool_width:.21,dock_visual_length:.67,dock_visual_width:.46,
    dock_visual_x_offset:0,dock_visual_y_offset:0,gps_x:.20,gps_y:0,gps_z:.08,imu_x:.10,imu_y:0,imu_z:.01};
test.beforeEach(async({page})=>{
    mkdirSync(shots,{recursive:true});
    await page.addInitScript(()=>{
        localStorage.setItem("mowglinext.lang","en");
        localStorage.setItem("mowgli.robot-visual.v1",JSON.stringify({style:"worx",dockAppearance:"worx",transparent:false}));
    });
    await installMockBackend(page,{...SCENARIOS[0],topics:{...SCENARIOS[0].topics,robotDescription:{data:ROBOT_URDF}},
        rest:{"/api/settings/yaml":settings}});
});
test("desktop focuses dimensions on the existing previews and keeps zero dock offsets visible",async({page})=>{
    await page.setViewportSize({width:1440,height:1100});
    await page.goto("/#/settings?section=hardware");
    await page.getByText("Chassis & Geometry",{exact:true}).click();
    const preview=page.getByTestId("mower-preview");
    await page.locator("#setting-chassis_z_offset").fill("-0.060");
    await expect(preview.getByTestId("measurement-overlay")).toHaveAttribute("data-measurement","chassis_z_offset");
    await expect(preview.getByTestId("mower-side").getByTestId("measurement-overlay")).toBeVisible();
    await expect(preview.getByTestId("measurement-overlay")).toContainText("-0.060 m");
    await expect(page.getByRole("dialog")).toHaveCount(0);
    await page.screenshot({path:shots+"/desktop-height.png"});
    await page.locator("#setting-wheel_track").focus();
    await expect(preview.getByTestId("mower-top").getByTestId("measurement-overlay")).toBeVisible();
    await page.screenshot({path:shots+"/desktop-track.png"});
    await page.locator("#setting-dock_visual_x_offset").focus();
    await expect(preview.getByTestId("measurement-overlay")).toContainText("0.000 m");
    await expect(preview.getByTestId("measurement-legend")).toContainText("Built-in dock alignment");
    await page.screenshot({path:shots+"/desktop-dock.png"});
    await page.locator("#setting-wheel_radius").fill("0");
    await expect(preview.getByTestId("measurement-overlay")).toHaveCount(0);
    await page.locator("#setting-wheel_radius").fill("");
    await expect(preview.getByTestId("measurement-overlay")).toHaveCount(0);
});
test("mobile edits are isolated until Apply, Cancel and Escape discard, no automatic writes",async({page})=>{
    await page.setViewportSize({width:390,height:844});
    const writes:string[]=[];
    page.on("request",r=>{if(r.method()!=="GET"&&r.url().includes("/api/")&&!r.url().endsWith("/api/config/keys/get")) writes.push(r.url());});
    await page.goto("/#/settings?section=hardware");
    await page.getByText("Chassis & Geometry",{exact:true}).click();
    const field=page.locator("#setting-chassis_z_offset");
    await field.click();
    const dialog=page.getByRole("dialog"), input=dialog.locator("#measurement-value");
    await expect(dialog).toBeVisible();
    await expect(dialog.getByTestId("mower-side")).toBeVisible();
    await input.fill("-0.090");
    await expect(dialog.getByTestId("measurement-overlay")).toContainText("-0.090 m");
    await expect(field).toHaveValue("-0.070");
    await page.waitForTimeout(350); // Finish the modal entry transition before the PR capture.
    await page.screenshot({path:shots+"/mobile-height.png"});
    await dialog.getByRole("button",{name:"Cancel",exact:true}).click();
    await expect(dialog).toHaveCount(0); await expect(field).toHaveValue("-0.070");
    await expect(field).toBeFocused();
    await field.press("Enter");
    await input.fill("-0.080");
    await input.press("Escape");
    await expect(dialog).toHaveCount(0); await expect(field).toHaveValue("-0.070");
    await field.click(); await input.fill("-0.060");
    await dialog.getByRole("button",{name:"Apply",exact:true}).click();
    await expect(field).toHaveValue("-0.060");
    const track=page.locator("#setting-wheel_track");
    await track.click();
    await expect(dialog.getByTestId("mower-top")).toBeVisible();
    await input.fill("0");
    await expect(dialog.getByRole("button",{name:"Apply",exact:true})).toBeDisabled();
    await input.fill("0.330");
    await page.waitForTimeout(350); // Finish the modal entry transition before the PR capture.
    await page.screenshot({path:shots+"/mobile-track.png"});
    await dialog.getByRole("button",{name:"Apply",exact:true}).click();
    await expect(track).toHaveValue("0.330");
    expect(writes).toEqual([]);
});
test("shrinking the mobile viewport keeps editor actions accessible",async({page})=>{
    await page.setViewportSize({width:390,height:844});
    await page.goto("/#/settings?section=hardware");
    await page.locator("#setting-wheel_radius").click();
    const dialog=page.getByRole("dialog");
    await expect(dialog.locator("#measurement-value")).toBeVisible();
    await page.setViewportSize({width:390,height:500});
    await expect.poll(async () => {
        const box=await dialog.getByRole("button",{name:"Apply",exact:true}).boundingBox();
        return box ? box.y+box.height : Infinity;
    }).toBeLessThanOrEqual(500);
    await dialog.locator("#measurement-value").fill(".12");
    await dialog.getByRole("button",{name:"Apply",exact:true}).click();
    await expect(page.locator("#setting-wheel_radius")).toHaveValue("0.12000");
});


test("small desktop keeps the active drawing and numeric field in view",async({page})=>{
    await page.setViewportSize({width:1280,height:720});
    await page.goto("/#/settings?section=hardware");
    await page.getByText("Chassis & Geometry",{exact:true}).click();
    await page.locator("#setting-dock_visual_y_offset").click();
    const preview=page.getByTestId("mower-preview");
    await expect(preview).toHaveCSS("position","sticky");
    await expect.poll(async()=>{
        const svg=await preview.getByTestId("mower-top").boundingBox();
        const input=await page.locator("#setting-dock_visual_y_offset").boundingBox();
        return !!svg && !!input && svg.y>=80 && svg.y+svg.height<=input.y && input.y+input.height<680;
    }).toBe(true);
    await page.screenshot({path:shots+"/desktop-compact.png"});
});


for (const width of [1440,390]) test(`blade side offset uses signed metres and a transparent top preview at ${width}px`,async({page})=>{
    await page.setViewportSize({width,height:1000});
    await page.goto("/#/settings?section=hardware");
    await page.getByText("Chassis & Geometry",{exact:true}).click();
    const field=page.locator("#setting-blade_y_offset");
    await field.click();
    const mobile=width<768;
    const scope=mobile?page.getByRole("dialog"):page.getByTestId("mower-preview");
    const input=mobile?scope.locator("#measurement-value"):field;
    await input.fill("-0.0575");
    const overlay=scope.getByTestId("measurement-overlay");
    await expect(overlay).toHaveAttribute("data-measurement","blade_y_offset");
    await expect(overlay).toContainText("-0.0575 m");
    await expect(scope.getByTestId("mower-top").locator('[data-mower-style]')).toHaveAttribute("data-transparent","true");
    await page.waitForTimeout(350); // Let the mobile modal finish its entry transition for the PR capture.
    await page.screenshot({path:shots+`/blade-offset-${width}.png`});
    if(mobile) await scope.getByRole("button",{name:"Apply",exact:true}).click();
    await expect(field).toHaveValue("-0.0575");
});
