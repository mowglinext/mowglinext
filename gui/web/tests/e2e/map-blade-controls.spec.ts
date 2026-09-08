import {expect, test} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend.ts";
import {SCENARIOS} from "./mock/scenarios.ts";

for (const mobile of [false, true]) {
    test.describe(mobile ? "mobile blade menu" : "desktop blade menu", () => {
        test.use({viewport: mobile ? {width: 390, height: 844} : {width: 1440, height: 900}});

        test.beforeEach(async ({page}) => {
            await page.addInitScript(() => localStorage.setItem("mowglinext.lang", "en"));
            // Manual mode opens this socket even without joystick interaction.
            await page.routeWebSocket(/\/api\/mowglinext\/publish\/joy/, () => {});
        });

        for (const manual of [false, true]) {
            test(`forward, reverse and off use the session policy during ${manual ? "manual" : "autonomous"} mowing`, async ({page}) => {
                const scenario = SCENARIOS.find(s => s.name === "mowing-area2-rtk-fixed")!;
                await installMockBackend(page, {
                    ...scenario,
                    topics: {
                        ...scenario.topics,
                        highLevelStatus: {
                            ...(scenario.topics?.highLevelStatus as object),
                            state: manual ? 4 : 2,
                            state_name: manual ? "MANUAL_MOWING" : "MOWING",
                        },
                    },
                    rest: {...scenario.rest, "/api/settings/yaml": {datum_lat: 48.1, datum_lon: 11.5}},
                });
                const commands: {path: string; body: unknown}[] = [];
                await page.route(/\/api\/mowglinext\/call\//, async route => {
                    commands.push({path: new URL(route.request().url()).pathname, body: route.request().postDataJSON()});
                    await route.fulfill({json: {}});
                });
                await page.goto("/#/map");
                for (const [index, label] of ["Blade Forward", "Blade Backward", "Blade Off"].entries()) {
                    await page.getByRole("main").getByRole("button", {name: /More$/}).click();
                    await page.getByRole("menuitem", {name: new RegExp(label, "i")}).click();
                    await expect.poll(() => commands.length).toBe(index + 1);
                }
                expect(commands).toEqual([
                    {path: "/api/mowglinext/call/blade_control", body: {mow_enabled: 1, mow_direction: 0}},
                    {path: "/api/mowglinext/call/blade_control", body: {mow_enabled: 1, mow_direction: 1}},
                    {path: "/api/mowglinext/call/blade_control", body: {mow_enabled: 0, mow_direction: 0}},
                ]);
            });
        }

        test("a rejected direction reports failure without a direct ON fallback", async ({page}) => {
            const scenario = SCENARIOS.find(s => s.name === "mowing-area2-rtk-fixed")!;
            await installMockBackend(page, {
                ...scenario,
                rest: {...scenario.rest, "/api/settings/yaml": {datum_lat: 48.1, datum_lon: 11.5}},
            });
            const commands: string[] = [];
            await page.route(/\/api\/mowglinext\/call\//, async route => {
                commands.push(new URL(route.request().url()).pathname);
                await route.fulfill({status: 500, json: {error: "blade control request was not accepted"}});
            });
            await page.goto("/#/map");
            await page.getByRole("main").getByRole("button", {name: /More$/}).click();
            await page.getByRole("menuitem", {name: /Blade Backward/i}).click();
            await expect(page.getByText("blade control request was not accepted", {exact: true})).toBeVisible();
            expect(commands).toEqual(["/api/mowglinext/call/blade_control"]);
        });
    });
}
