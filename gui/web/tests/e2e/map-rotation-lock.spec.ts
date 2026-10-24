import {expect, test, type Page} from "@playwright/test";
import {mkdir} from "node:fs/promises";
import {resolve} from "node:path";
import {installMockBackend} from "./mock/mockBackend.ts";
import {SCENARIOS} from "./mock/scenarios.ts";

const base = SCENARIOS.find(scenario => scenario.name === "idle-docked-full")!;
const mapReady = {
    ...base,
    rest: {
        ...base.rest,
        "/api/settings/yaml": {datum_lat: 48.1, datum_lon: 11.5},
    },
};

async function openMap(page: Page, viewport: {width: number; height: number}) {
    await page.setViewportSize(viewport);
    await page.addInitScript(() => localStorage.setItem("mowglinext.lang", "en"));
    await installMockBackend(page, mapReady);

    const writes: Record<string, string>[] = [];
    await page.route("**/api/config/keys/get", route => route.fulfill({
        json: {
            "gui.map.display.bearing": "33",
            "gui.map.display.rotation_locked": "true",
        },
    }));
    await page.route("**/api/config/keys/set", route => {
        const body = route.request().postDataJSON() as Record<string, string>;
        writes.push(body);
        return route.fulfill({json: body});
    });

    await page.goto("/#/map");
    await expect(page.locator(".mapboxgl-canvas")).toBeVisible({timeout: 15_000});
    return writes;
}

test("the map rotation panel controls the mower-wide gesture lock", async ({page}) => {
    const writes = await openMap(page, {width: 1440, height: 900});
    const rotationHeader = page.getByRole("button", {name: /Map Rotation/});
    await rotationHeader.click();

    const panel = page.getByTestId("map-rotation-panel");
    await expect(panel.getByRole("spinbutton")).toHaveValue("33");
    const rotationLock = panel.getByRole("button", {name: "Unlock map rotation"});
    await expect(rotationLock).toHaveAttribute("aria-pressed", "true");
    await expect(panel.getByRole("slider")).toBeEnabled();
    await expect(panel.getByRole("spinbutton")).toBeEnabled();

    await rotationLock.click();
    await expect.poll(() => writes).toContainEqual({"gui.map.display.rotation_locked": "false"});
    const unlockedButton = panel.getByRole("button", {name: "Lock map rotation"});
    await expect(unlockedButton).toHaveAttribute("aria-pressed", "false");

    await unlockedButton.click();
    await expect.poll(() => writes).toContainEqual({"gui.map.display.rotation_locked": "true"});
    await panel.getByRole("spinbutton").fill("25");
    await panel.getByRole("spinbutton").press("Enter");
    await expect.poll(() => writes).toContainEqual({"gui.map.display.bearing": "25"});

    if (process.env.MAP_ROTATION_SCREENSHOT) {
        const images = resolve(process.cwd(), "../../docs/images");
        await mkdir(images, {recursive: true});
        await rotationHeader.locator("..").locator("..").screenshot({
            animations: "disabled",
            path: resolve(images, "map-rotation-lock.png"),
        });
    }
});

test("the mobile map menu controls the same mower-wide lock", async ({page}) => {
    const writes = await openMap(page, {width: 390, height: 844});

    await page.getByRole("main").getByRole("button", {name: "More"}).click();
    if (process.env.MAP_ROTATION_SCREENSHOT) {
        const images = resolve(process.cwd(), "../../docs/images");
        await mkdir(images, {recursive: true});
        await page.screenshot({
            animations: "disabled",
            path: resolve(images, "map-rotation-lock-mobile.png"),
        });
    }
    await page.getByText("Unlock map rotation", {exact: true}).click();

    await expect.poll(() => writes).toContainEqual({"gui.map.display.rotation_locked": "false"});
});
