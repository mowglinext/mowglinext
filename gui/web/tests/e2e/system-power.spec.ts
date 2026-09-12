import {expect, test, type Page} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend.ts";
import {SCENARIOS} from "./mock/scenarios.ts";
import {GnssStatusConstants} from "../../src/types/ros.ts";

async function openSystem(page: Page, mobile = false) {
    await installMockBackend(page, {
        ...SCENARIOS[0],
        topics: {
            ...SCENARIOS[0].topics,
            gnssStatus: {fix_valid: true, rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
                capability_flags: GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
                value_flags: GnssStatusConstants.CAP_HORIZONTAL_ACCURACY, horizontal_accuracy_m: 0.014},
        },
        rest: {
            "/api/diagnostics/snapshot": {
                timestamp: new Date().toISOString(),
                system: {cpu_temperature: 43.2, cpu_usage: 14.8},
                containers: ["mowgli-ros2", "mowgli-gui", "mowgli-gps"].map(name => ({
                    name, state: "running", status: "Up 2 hours", started_at: "2026-09-08T12:00:00Z",
                })),
                coverage: [],
                cross_checks: {dock_pose: {}, warnings: [], overall_status: "ok"},
            },
        },
    });
    await page.goto("/#/diagnostics");
    if (mobile) await page.getByRole("button", {name: /System$/}).click();
    await expect(page.getByRole("button", {name: "Host power…", exact: true})).toBeVisible();
    await expect(page.getByText("GPS: RTK Fixed", {exact: true})).toBeVisible();
    await expect(page.getByRole("button", {name: "Battery and power menu"})).toContainText("100%");
    await page.getByRole("button", {name: "Host power…", exact: true}).scrollIntoViewIfNeeded();
}

async function screenshot(page: Page, name: string) {
    await page.evaluate(() => document.fonts.ready);
    // Let the dialog/collapse entrance transition finish before recording it.
    await page.waitForTimeout(350);
    await page.screenshot({path: `tests/e2e/.artifacts/${name}.png`, animations: "disabled"});
}

test("desktop shortcut uses the existing confirmation and reboot reconnect flow", async ({page}) => {
    await page.setViewportSize({width: 1440, height: 1000});
    await openSystem(page);
    let requests = 0;
    await page.route("**/api/system/reboot", route => { requests++; return route.fulfill({json: {}}); });
    let polls = 0;
    await page.route("**/api/system/info", route => { polls++; return route.fulfill({status: 503}); });
    const trigger = page.getByRole("button", {name: "Host power…", exact: true});
    await trigger.click();
    const restart = page.getByRole("menuitem", {name: /Restart Host/});
    await expect(restart).toBeVisible();
    await screenshot(page, "system-power-desktop");
    expect(requests).toBe(0);
    await restart.click();
    let dialog = page.getByRole("dialog");
    await expect(page.getByRole("dialog", {name: "Restart Host", exact: true})).toBeVisible();
    await dialog.getByRole("button", {name: "Cancel", exact: true}).click();
    await expect(dialog).not.toBeVisible();
    expect(requests).toBe(0);
    await trigger.click();
    await restart.click();
    dialog = page.getByRole("dialog");
    await screenshot(page, "system-power-reboot-confirmation");
    await page.clock.install();
    await dialog.getByRole("button", {name: "Confirm", exact: true}).click();
    await expect(page.getByText("Rebooting the Host…", {exact: true})).toBeVisible();
    expect(requests).toBe(1);
    await page.clock.fastForward(15_000);
    await expect.poll(() => polls).toBe(1);
    expect(requests).toBe(1);
});

test("mobile shortcut uses the existing shutdown confirmation", async ({page}) => {
    await page.setViewportSize({width: 390, height: 844});
    await openSystem(page, true);
    let requests = 0;
    await page.route("**/api/system/shutdown", route => { requests++; return route.fulfill({json: {}}); });
    await page.getByRole("button", {name: "Host power…", exact: true}).click();
    await expect(page.getByRole("menuitem", {name: /Shut down Host/})).toBeVisible();
    await screenshot(page, "system-power-mobile");
    await page.getByRole("menuitem", {name: /Shut down Host/}).click();
    const dialog = page.getByRole("dialog");
    await expect(dialog.getByText(/Physical access will be required/)).toBeVisible();
    expect(requests).toBe(0);
    await screenshot(page, "system-power-shutdown-mobile");
    await dialog.getByRole("button", {name: "Confirm", exact: true}).click();
    await expect.poll(() => requests).toBe(1);
    await expect(dialog).not.toBeVisible();
});

test("battery Advanced menu retains the same host actions", async ({page}) => {
    await openSystem(page);
    let requests = 0;
    await page.route("**/api/system/shutdown", route => { requests++; return route.fulfill({json: {}}); });
    await page.getByRole("button", {name: "Battery and power menu"}).click();
    await page.getByRole("menuitem", {name: /Advanced/}).hover();
    await page.getByRole("menuitem", {name: /Shut down Host/}).click();
    const dialog = page.getByRole("dialog");
    await expect(dialog.getByText(/Physical access will be required/)).toBeVisible();
    expect(requests).toBe(0);
    await dialog.getByRole("button", {name: "Confirm", exact: true}).click();
    await expect.poll(() => requests).toBe(1);
});

test("shortcut retains existing failure notification and clears reconnect overlay", async ({page}) => {
    await openSystem(page);
    let requests = 0;
    await page.route("**/api/system/reboot", route => { requests++; return route.abort("connectionreset"); });
    await page.getByRole("button", {name: "Host power…", exact: true}).click();
    await page.getByRole("menuitem", {name: /Restart Host/}).click();
    await page.getByRole("dialog").getByRole("button", {name: "Confirm", exact: true}).click();
    await expect(page.getByText("Restart failed", {exact: true})).toBeVisible();
    await expect(page.getByText("Rebooting the Host…", {exact: true})).not.toBeVisible();
    expect(requests).toBe(1);
});
