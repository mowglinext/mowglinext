import {expect, test} from "@playwright/test";
import {mkdirSync} from "node:fs";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {ROBOT_URDF} from "../../src/test/robotUrdf";

const urdf = ROBOT_URDF.replace('0.3 0 0.2','0.15 0 0.148');
for (const width of [1440, 390, 320]) for (const docked of [true, false]) {
    test(`health card ${docked ? "docked" : "mowing"} at ${width}px`, async ({page}) => {
        const style = docked ? "yardforce" : "rm1000";
        await page.setViewportSize({width, height: 1000});
        await page.addInitScript(() => {
            localStorage.setItem("mowglinext.lang", "en");
            localStorage.setItem("mowgli.display-mode", "balanced");
            localStorage.setItem("mowgli.robot-visual.v1", JSON.stringify({transparent:true}));
        });
        const scenario = SCENARIOS[docked ? 0 : 1];
        const writes: string[] = [];
        page.on("request", request => {
            if (request.method() !== "GET" && !request.url().includes("/config/keys/get")) writes.push(request.url());
        });
        await installMockBackend(page, {...scenario, rest:{...scenario.rest,
            "/api/settings/yaml":{mower_visual_style:style, mower_visual_colour:docked ? "#e97822" : "#167bc1", lidar_enabled:false},
        }, topics:{...scenario.topics, robotDescription:{data:urdf},
            // Full battery / zero current must retain the dock; no current heuristic.
            status:{...(scenario.topics!.status as object), mower_motor_temperature:docked ? 20 : 41, firmware_version:"1.0.0"},
            power:{v_battery:28, v_charge:docked ? 29 : 0, charge_current:0, charger_enabled:docked},
        }}, {liveStatusIntervalMs:100});
        await page.goto("/#/mowglinext");
        const art=page.getByTestId("status-mower-top");
        await expect(art).toBeAttached();
        await expect(art.locator('[data-layer="shell-art"]')).toHaveAttribute("data-paint-ready", "true");
        await expect(art.locator('[data-mower-style]')).toHaveAttribute("data-mower-style",style);
        await expect(art.locator('[data-layer="styled-dock"]')).toHaveCount(docked ? 1 : 0);
        await expect(art.locator('[data-layer="shell"]')).toHaveAttribute("opacity","1");
        await expect(art.locator('[data-sensor="lidar"], [data-sensor="imu"], [data-layer="blade"], [data-layer="casters"]')).toHaveCount(0);
        await expect(art.locator('[data-sensor="gps"]')).toHaveCount(1);
        await page.locator('.health-card').scrollIntoViewIfNeeded();
        await page.evaluate(async () => {
            await Promise.all([...document.querySelectorAll('.health-card image')].map(async e => {
                const image=new Image(); image.src=e.getAttribute('href')!; await image.decode();
            }));
        });
        const card=page.locator('.health-card');
        const bounds=await card.boundingBox();
        for(const element of await card.locator('svg[role="img"], button').all()) {
            const box=await element.boundingBox();
            expect(box!.x).toBeGreaterThanOrEqual(bounds!.x);
            expect(box!.x+box!.width).toBeLessThanOrEqual(bounds!.x+bounds!.width+1);
        }
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
        const requests=await page.evaluate(() => performance.getEntriesByType("resource").map(e=>e.name).filter(s=>s.includes("/assets/robots/layered/")));
        expect(requests.length).toBeGreaterThan(0);
        expect(requests.every(s=>s.includes('/map/') && new URL(s).searchParams.has('v'))).toBe(true);
        expect(writes).toEqual([]);
        mkdirSync('screenshots.local/status-mower',{recursive:true});
        await card.screenshot({path:`screenshots.local/status-mower/card-${docked ? "docked" : "mowing"}-${width}.png`});
        if(width !== 320) await page.screenshot({path:`screenshots.local/status-mower/page-${docked ? "docked" : "mowing"}-${width}.png`,fullPage:true});
    });
}

for (const sample of [
    {name:"failed return with IDLE_DOCKED", power:{charger_enabled:false}, hls:{state_name:"IDLE_DOCKED",is_charging:true}},
    {name:"no telemetry", power:{}, hls:{}},
]) test(`health card does not invent dock contact: ${sample.name}`, async({page}) => {
    await installMockBackend(page, {name:sample.name, topics:{robotDescription:{data:urdf},power:sample.power,highLevelStatus:sample.hls}});
    await page.goto('/#/mowglinext');
    await expect(page.getByTestId('status-mower-top')).toBeAttached();
    await expect(page.getByTestId('status-mower-top').locator('[data-layer="styled-dock"]')).toHaveCount(0);
});
