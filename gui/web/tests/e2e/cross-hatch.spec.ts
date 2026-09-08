import {test, expect} from "@playwright/test";
import {mkdir} from "node:fs/promises";
import {resolve} from "node:path";
import {installMockBackend} from "./mock/mockBackend.ts";

for (const angle of [25, -1]) {
    const baseLabel = angle < 0 ? "Auto" : "25°";
    const crossLabel = angle < 0 ? "Auto + 90°" : "115°";
    const imageSuffix = angle < 0 ? "-auto" : "";
    test(`${baseLabel}: per-area next direction survives reload without changing the active run`, async ({page}) => {
        await page.addInitScript(() => localStorage.setItem("mowglinext.lang", "en"));
        await installMockBackend(page, {
            name: "cross-hatch-two-areas",
            rest: {"/api/settings/yaml": {mow_cross_hatch: true, mow_angle_deg: angle,
                headland_width: 0.4, min_turning_radius: 0.3, tool_width: 0.18,
                chassis_safety_inset: 0.1, swath_overlap: 0.02}},
            topics: {map: {
                working_area: [{name: "Front lawn"}, {name: "Back lawn"}],
                // A navigation area occupies ROS index 1; Back lawn must use 2.
                working_area_indices: [0, 2], navigation_areas: [{name: "Passage"}],
            }},
        });
        const next = new Map([[0, false], [2, true]]);
        const writes: number[] = [];
        let fail = false;
        await page.route("**/api/mowglinext/call/coverage_orientation", async route => {
            if (fail) return route.fulfill({status: 500, json: {error: "ROS unavailable"}});
            const body = route.request().postDataJSON();
            expect([0, 2]).toContain(body.area_index);
            if (body.set_next) { next.set(body.area_index, body.perpendicular); writes.push(body.area_index); }
            return route.fulfill({json: {
                success: true, enabled: true, base_angle_deg: angle,
                current_active: body.area_index === 2, current_perpendicular: false,
                next_perpendicular: next.get(body.area_index),
            }});
        });
        const openSettings = async () => {
            await page.goto("/#/settings");
            await page.getByRole("menuitem", {name: /Mowing$/}).click();
            await expect(page.getByTestId("cross-hatch-area-2").getByText(crossLabel, {exact: true})).toBeVisible();
        };
        await openSettings();
        const front = page.getByTestId("cross-hatch-area-0");
        const back = page.getByTestId("cross-hatch-area-2");
        await expect(front.getByText(baseLabel, {exact: true})).toBeVisible();
        await expect(back.getByText(`Current: ${baseLabel}`, {exact: true})).toBeVisible();
        await expect(page.getByTestId("cross-hatch-area-1")).toHaveCount(0);
        await expect(front.getByRole("combobox")).toBeEnabled();
        await expect(back.getByRole("combobox")).toBeEnabled();
        const images = resolve(process.cwd(), "../../docs/images");
        await mkdir(images, {recursive: true});
        if (process.env.CROSS_HATCH_SCREENSHOTS) {
            await page.getByTestId("cross-hatch-settings").locator("..").screenshot({animations: "disabled", path: resolve(images, `cross-hatch-settings${imageSuffix}.png`)});
        }
        await back.getByText(crossLabel, {exact: true}).click();
        await page.getByTitle(baseLabel, {exact: true}).last().click();
        await expect(back.getByTitle(baseLabel, {exact: true})).toBeVisible();
        await expect(back.getByText(`Current: ${baseLabel}`, {exact: true})).toBeVisible();
        expect(writes).toEqual([2]);
        expect(next.get(0)).toBe(false);
        await back.getByText(`Current: ${baseLabel}`, {exact: true}).click();
        await expect(page.locator(".ant-select-dropdown:visible")).toHaveCount(0);
        if (process.env.CROSS_HATCH_SCREENSHOTS) {
            await page.getByTestId("cross-hatch-settings").screenshot({path: resolve(images, `cross-hatch-next-override${imageSuffix}.png`)});
        }
        await page.reload();
        await page.getByRole("menuitem", {name: /Mowing$/}).click();
        await expect(back.getByTitle(baseLabel, {exact: true})).toBeVisible();
        fail = true;
        await back.getByRole("button", {name: /Refresh/}).click();
        await expect(back.getByRole("alert")).toBeVisible();
        await expect(back.getByRole("combobox")).toBeDisabled();
    });
}
