import {expect, test} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend.ts";

for (const mapped of [true, false]) {
    test(`obstacle promotion ${mapped ? "uses the original ROS area ID" : "requires ID metadata"}`, async ({page}) => {
        await page.addInitScript(() => localStorage.setItem("mowglinext.lang", "en"));
        const rectangle = (x: number) => ({points: [
            {x, y: 0}, {x: x + 5, y: 0}, {x: x + 5, y: 5}, {x, y: 5},
        ]});
        await installMockBackend(page, {
            name: "interleaved-obstacle-area",
            rest: {"/api/settings/yaml": {datum_lat: 48.1, datum_lon: 11.5}},
            topics: {
                map: {
                    working_area: [
                        {name: "Front lawn", area: rectangle(10)},
                        {name: "Back lawn", area: rectangle(0)},
                    ],
                    working_area_indices: mapped ? [0, 2] : undefined,
                    navigation_areas: [{name: "Passage", area: rectangle(20)}],
                },
                obstacles: {obstacles: [{id: 7, status: 1, polygon: {
                    points: [{x: 1, y: 1}, {x: 2, y: 1}, {x: 2, y: 2}],
                }}]},
            },
        });
        const requests: unknown[] = [];
        await page.route("**/api/mowglinext/call/promote_obstacle", async route => {
            requests.push(route.request().postDataJSON());
            await route.fulfill({json: {success: true}});
        });
        await page.goto("/#/map");
        const promote = page.getByRole("button", {name: /Promote$/});
        if (!mapped) {
            await expect(promote).toBeDisabled();
            expect(requests).toEqual([]);
            return;
        }
        await expect(promote).toBeEnabled();
        await page.getByRole("button", {name: /Edit Map$/}).click();
        await page.getByRole("button", {name: /Move up/i}).last().click();
        await promote.click();
        const dialog = page.getByRole("dialog");
        await expect(dialog.getByText("Back lawn (1)", {exact: true})).toBeVisible();
        await dialog.getByRole("button", {name: /Promote$/}).click();
        await expect.poll(() => requests.length).toBe(1);
        expect(requests[0]).toMatchObject({area_index: 2, obstacle_id: 7});
    });
}
