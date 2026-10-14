import {expect,test} from "@playwright/test";
import {mkdirSync} from "node:fs";
import {APPEARANCE_REGISTRY} from "../../src/components/robot/appearanceRegistry";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {MOWER_MODELS} from "../../src/constants/mowerModels";
import {ROBOT_URDF} from "../../src/test/robotUrdf";

test("all shell paint masks render both views with the production canvas pipeline",async({page})=>{
    await page.setViewportSize({width:1600,height:1150});
    // Simulate old immutable asset URLs being unusable: every rendered source,
    // mask and compact derivative must request this build's versioned URL.
    await page.route("**/assets/robots/layered/**",route=>new URL(route.request().url()).searchParams.get("v") ? route.continue() : route.abort());
    await page.goto("/tests/e2e/fixtures/chassis-gallery.html?paint=1");
    const shells=page.locator('[data-layer="shell-art"]');
    await expect(shells).toHaveCount(Object.keys(APPEARANCE_REGISTRY.shells).length*2);
    for(const shell of await shells.all()) {
        await expect(shell).toHaveAttribute("data-paint-ready","true");
        expect(await shell.locator("image").getAttribute("href")).toMatch(/^data:image\/png/);
    }
    mkdirSync("screenshots.local/layered-mower/pr",{recursive:true});
    await page.locator("main").screenshot({path:"screenshots.local/layered-mower/pr/chassis-colours.png"});
});

test("missing paint masks preserve the original shell",async({page})=>{
    let failed=0;
    await page.route("**/*-paint.svg*",route=>{failed++;return route.abort();});
    await page.goto("/tests/e2e/fixtures/chassis-gallery.html?paint=1");
    await expect(page.locator('[data-layer="shell-art"]')).toHaveCount(Object.keys(APPEARANCE_REGISTRY.shells).length*2);
    await expect.poll(()=>failed).toBe(Object.keys(APPEARANCE_REGISTRY.shells).length*2);
    expect(await page.locator('[data-paint-ready="true"]').count()).toBe(0);
    for(const shell of await page.locator('[data-layer="shell-art"]').all()) {
        await expect(shell.locator("image")).toHaveAttribute("href",/assets\/robots\/layered\/.*\.png\?v=/);
    }
});

test("colour picker edits a draft, saves only colour and can restore Original",async({page})=>{
    let settings:Record<string,unknown>={mower_model:"YardForce500",mower_visual_style:"yardforce",mower_visual_colour:"",lidar_enabled:false,...MOWER_MODELS[0].defaults,caster_x_offset:.40,caster_track:.30,gps_x:.15,gps_y:0,gps_z:.148};
    const writes:unknown[]=[];const forbidden:string[]=[];
    await page.addInitScript(()=>localStorage.setItem("mowglinext.lang","en"));
    await installMockBackend(page,{...SCENARIOS[0],topics:{...SCENARIOS[0].topics,robotDescription:{data:ROBOT_URDF}}});
    await page.route("**/api/settings/yaml",async route=>{
        if(route.request().method()==="POST"){
            const patch=route.request().postDataJSON();writes.push(patch);settings={...settings,...patch};
        }
        await route.fulfill({json:settings});
    });
    page.on("request",r=>{if(r.method()==="POST" && /\/(params|containers|mowglinext\/call)/.test(r.url()))forbidden.push(r.url());});
    await page.goto("/#/settings?section=hardware");
    const preview=page.getByTestId("mower-preview");
    await expect(preview.locator('[data-layer="shell-art"]').first()).toHaveAttribute("data-paint-colour","original");
    await preview.getByRole("button",{name:"Suggested",exact:true}).click();
    await expect(preview.locator('[data-layer="shell-art"]').first()).toHaveAttribute("data-paint-ready","true");
    expect(writes).toEqual([]);
    await page.reload();
    await expect(preview.locator('[data-layer="shell-art"]').first()).toHaveAttribute("data-paint-colour","original");
    await preview.getByRole("button",{name:"Choose body colour"}).click();
    await expect(page.locator(".ant-color-picker-panel")).toBeVisible();
    await page.keyboard.press("Escape");
    await preview.getByRole("button",{name:"Suggested",exact:true}).click();
    await page.getByRole("button",{name:/Save \(/}).click();
    await expect(page.getByRole("button",{name:/Saved$/})).toBeDisabled();
    expect(writes).toEqual([{mower_visual_colour:"#e97822"}]);
    expect(forbidden).toEqual([]);
    expect(await page.evaluate(()=>sessionStorage.getItem("mowgli.pendingRosRestart"))).not.toBe("true");
    await page.reload();
    await expect(preview.locator('[data-layer="shell-art"]').first()).toHaveAttribute("data-paint-colour","#e97822");
    await page.setViewportSize({width:1440,height:1100});
    await preview.evaluate(el=>el.scrollIntoView({block:"center"}));
    await expect(preview.locator('[data-paint-ready="true"]')).toHaveCount(2);
    await page.screenshot({path:"screenshots.local/layered-mower/pr/colour-hardware.png"});
    await page.setViewportSize({width:390,height:844});
    await expect(preview.locator('[data-paint-ready="true"]')).toHaveCount(2);
    await preview.evaluate(el=>{el.scrollIntoView({block:"start"});window.dispatchEvent(new Event("resize"));});
    await page.screenshot({path:"screenshots.local/layered-mower/pr/colour-picker-mobile.png"});
    await page.setViewportSize({width:1440,height:1100});
    await page.goto("/#/settings?section=sensors");
    await expect(page.getByTestId("mower-side").locator('[data-layer="shell-art"]')).toHaveAttribute("data-paint-colour","#e97822");
    await expect(page.getByTestId("mower-side").locator('[data-layer="shell-art"]')).toHaveAttribute("data-paint-ready","true");
    await page.getByTestId("sensor-placement").evaluate(el=>el.scrollIntoView({block:"start"}));
    await page.screenshot({path:"screenshots.local/layered-mower/pr/colour-sensors.png"});
    await page.goto("/#/settings?section=hardware");
    await preview.getByRole("button",{name:"Original",exact:true}).click();
    await page.getByRole("button",{name:/Save \(/}).click();
    await expect(page.getByRole("button",{name:/Saved$/})).toBeDisabled();
    expect(writes).toEqual([{mower_visual_colour:"#e97822"},{mower_visual_colour:""}]);
});


test("side panel paint registration review",async({page})=>{
    await page.setViewportSize({width:1500,height:1000});
    await page.goto("/tests/e2e/fixtures/side-paint-review.html");
    await expect(page.locator("main")).toHaveAttribute("data-ready","true");
    await page.locator("main").screenshot({path:"screenshots.local/layered-mower/pr/side-paint-registration.png"});
});
