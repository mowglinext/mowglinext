import {expect, test} from "@playwright/test";
import {ROBOT_URDF} from "../../src/test/robotUrdf";
import {mkdirSync} from "node:fs";
import {installMockBackend} from "./mock/mockBackend";

for (const model of ["YardForce SA650", "YardForce 900 ECO", "YardForce LUV1000RI", "Sabo MOWiT 500F"]) {
    test(`${model} applies wheel radii in metres only after confirming the preset`, async ({page}) => {
        const writes: string[]=[];
        page.on("request", request => {
            if(request.method() !== "GET" && !request.url().includes("/config/keys/get")) writes.push(request.url());
        });
        await page.addInitScript(() => localStorage.setItem("mowglinext.lang","en"));
        await installMockBackend(page, {name:"wheel-presets",rest:{
            "/api/settings/yaml":{mower_model:"CUSTOM",wheel_radius:0.04475,caster_radius:0.03},
        }});
        await page.goto('/#/settings?section=hardware');
        const wheel=page.getByRole('spinbutton',{name:'Wheel Radius, m',exact:true});
        await expect(wheel).toHaveValue('0.04475');
        await page.getByRole('radio',{name:model,exact:true}).click();
        const confirm=page.getByRole('dialog');
        await expect(confirm).toBeVisible();
        await expect(wheel).toHaveValue('0.04475');
        await confirm.getByRole('button',{name:'Apply preset',exact:true}).click();
        await expect(wheel).toHaveValue(/^0\.0950*$/);
        await page.getByText('Chassis & Geometry',{exact:true}).click();
        await expect(page.getByRole('spinbutton',{name:'Caster Radius, m',exact:true})).toHaveValue('0.045');
        // Applying a preset edits the draft; an update must not silently rewrite
        // an existing robot's calibration or send a restart/actuation command.
        expect(writes).toEqual([]);
    });
}


test("WORX 500M Plus applies supplied dimensions and blade origin, with its matching appearance",async({page})=>{
    await page.setViewportSize({width:1440,height:1100});
    await page.addInitScript(()=>localStorage.setItem("mowglinext.lang","en"));
    const writes:string[]=[];
    page.on("request",r=>{if(r.method()!=="GET" && !r.url().includes("/config/keys/get")) writes.push(r.url());});
    await installMockBackend(page,{name:"worx-preset",topics:{robotDescription:{data:ROBOT_URDF}},rest:{
        "/api/settings/yaml":{mower_model:"CUSTOM",blade_x_offset:.04,blade_y_offset:0,lidar_enabled:false,gps_z:.055,imu_z:0,chassis_z_offset:.02,caster_x_offset:.4},
    }});
    await page.goto("/#/settings?section=hardware");
    await page.getByRole("radio",{name:"WORX 500M Plus",exact:true}).click();
    await page.getByRole("dialog").getByRole("button",{name:"Apply preset",exact:true}).click();
    await expect(page.locator("#setting-wheel_radius")).toHaveValue("0.10000");
    await expect(page.locator("#setting-wheel_track")).toHaveValue("0.360");
    await expect(page.locator("#setting-ticks_per_meter")).toHaveValue("343.000");
    await page.getByText("Chassis & Geometry",{exact:true}).click();
    for(const [key,value] of Object.entries({chassis_length:"0.580",chassis_width:"0.395",chassis_height:"0.140",
        chassis_center_x:"0.195",wheel_x_offset:"-0.005",caster_radius:"0.030",caster_track:"0.000",
        blade_x_offset:"0.0000",blade_y_offset:"-0.0575"}))
        await expect(page.locator("#setting-"+key)).toHaveValue(value);
    await expect(page.locator("#setting-caster_track")).toHaveAttribute("aria-valuemin","0");
    await expect(page.locator("#setting-caster_x_offset")).toHaveValue("0.405");
    await expect(page.locator("#setting-chassis_z_offset")).toHaveValue("-0.070");
    await page.locator("#setting-blade_y_offset").click();
    const preview=page.getByTestId("mower-preview");
    await expect(preview.getByTestId("mower-top").locator('[data-mower-style]')).toHaveAttribute("data-mower-style","worx");
    const sideShell=preview.getByTestId("mower-side").locator('[data-layer="shell-art"]');
    await expect(sideShell).toHaveAttribute("height","0.14");
    expect(Number(await sideShell.getAttribute("y"))).toBeCloseTo(-.07);
    await expect(preview.getByTestId("measurement-overlay")).toContainText("-0.0575 m");
    mkdirSync("screenshots.local/wheel-presets",{recursive:true});
    await page.screenshot({path:"screenshots.local/wheel-presets/worx-preset.png",fullPage:true});
    expect(writes).toEqual([]);
});
