import {test, expect} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend";
import {SCENARIOS} from "./mock/scenarios";
import {ROBOT_URDF} from "../../src/test/robotUrdf";
import {MOWER_MODELS} from "../../src/constants/mowerModels";

for (const savedOffset of [undefined, 0]) test(`preset upgrade keeps saved offset ${savedOffset}`, async ({page}) => {
    const preset = MOWER_MODELS.find(model => model.value === "YardForce500B")!;
    const defaultOffset = preset.defaults.chassis_z_offset;
    await page.addInitScript(() => localStorage.setItem("mowglinext.lang", "en"));
    await installMockBackend(page, {...SCENARIOS[0],
        topics: {...SCENARIOS[0].topics, robotDescription: {data: ROBOT_URDF}},
        rest: {"/api/settings/yaml": {...preset.defaults, mower_model: preset.value,
            chassis_z_offset: savedOffset ?? defaultOffset}},
    });
    const requests: string[] = [];
    await page.route("**/api/settings/yaml/defaults*", async route => {
        const model = new URL(route.request().url()).searchParams.get("mower_model")!;
        requests.push(model);
        const selected = MOWER_MODELS.find(p => p.value === model);
        await route.fulfill({json: {chassis_z_offset: selected?.defaults.chassis_z_offset ?? 0}});
    });
    const writes: string[] = [];
    page.on("request", r => {if (r.method() === "POST" && r.url().endsWith("/settings/yaml")) writes.push(r.url());});
    await page.goto("/#/settings?section=hardware");
    await page.getByText("Chassis & Geometry", {exact: true}).click();
    const input = page.locator("#setting-chassis_z_offset");
    await expect.poll(async () => Number(await input.inputValue())).toBe(savedOffset ?? defaultOffset);
    await expect.poll(() => requests).toContain(preset.value);
    await page.getByText("Custom Robot", {exact: true}).click();
    await page.getByRole("button", {name: "Apply preset", exact: true}).click();
    await expect.poll(() => requests).toContain("CUSTOM");
    await input.fill("-0.073");
    await input.press("Tab");
    await page.getByRole("button", {name: /Reset Chassis vertical offset/}).click();
    await expect.poll(async () => Number(await input.inputValue())).toBe(0);
    expect(writes).toEqual([]);
});
