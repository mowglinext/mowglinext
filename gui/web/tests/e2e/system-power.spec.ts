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
    await expect(page.getByRole("button", {name: "Reboot Pi", exact: true})).toBeVisible();
    await expect(page.getByText("GPS: RTK Fixed", {exact: true})).toBeVisible();
    await expect(page.getByRole("button", {name: "Battery and power menu"})).toContainText("100%");
}

async function screenshot(page: Page, name: string) {
    await page.evaluate(() => document.fonts.ready);
    // Let the dialog/collapse entrance transition finish before recording it.
    await page.waitForTimeout(350);
    await page.screenshot({path: `tests/e2e/.artifacts/${name}.png`, animations: "disabled"});
}

test("reboot confirms, cancels without a request, and submits only once", async ({page}) => {
    await page.setViewportSize({width: 1440, height: 1000});
    await openSystem(page);
    const requests: string[] = [];
    let release: (() => void) | undefined;
    await page.route("**/api/system/reboot", async route => {
        requests.push(route.request().method() + " " + new URL(route.request().url()).pathname);
        await new Promise<void>(resolve => { release = resolve; });
        await route.fulfill({json: {}});
    });
    await screenshot(page, "system-power-desktop");
    await page.getByRole("button", {name: "Reboot Pi", exact: true}).click();
    let dialog = page.getByRole("dialog");
    await expect(dialog.getByText("Reboot the Raspberry Pi?")).toBeVisible();
    expect(requests).toEqual([]);
    await dialog.getByRole("button", {name: "Cancel", exact: true}).click();
    await expect(dialog).not.toBeVisible();
    expect(requests).toEqual([]);
    await page.getByRole("button", {name: "Reboot Pi", exact: true}).click();
    dialog = page.getByRole("dialog");
    await screenshot(page, "system-power-reboot-confirmation");
    await dialog.getByRole("button", {name: "Reboot Pi", exact: true}).click();
    await expect.poll(() => requests.length).toBe(1);
    await expect(dialog.getByRole("button", {name: "Cancel", exact: true})).toBeDisabled();
    await expect(dialog.getByRole("button", {name: "Reboot Pi", exact: true})).toHaveClass(/ant-btn-loading/);
    release!();
    await expect(page.getByText("Reboot requested", {exact: true})).toBeVisible();
    expect(requests).toEqual(["POST /api/system/reboot"]);
    await expect(page.getByRole("button", {name: "Reload page"})).toBeVisible();
    await expect(page.getByRole("button", {name: "Shut down Pi", exact: true})).not.toBeVisible();
});

test("mobile shutdown confirms the need for physical access and calls the shutdown endpoint", async ({page}) => {
    await page.setViewportSize({width: 390, height: 844});
    await openSystem(page, true);
    const requests: string[] = [];
    await page.route("**/api/system/shutdown", async route => {
        requests.push(route.request().method() + " " + new URL(route.request().url()).pathname);
        await route.fulfill({json: {}});
    });
    await page.getByRole("button", {name: "Shut down Pi", exact: true}).scrollIntoViewIfNeeded();
    await screenshot(page, "system-power-mobile");
    await page.getByRole("button", {name: "Shut down Pi", exact: true}).click();
    const dialog = page.getByRole("dialog");
    await expect(dialog.getByText(/Physical access is required to power it back on/)).toBeVisible();
    expect(requests).toEqual([]);
    await screenshot(page, "system-power-shutdown-mobile");
    await dialog.getByRole("button", {name: "Shut down Pi", exact: true}).click();
    await expect(page.getByText("Shutdown requested", {exact: true})).toBeVisible();
    expect(requests).toEqual(["POST /api/system/shutdown"]);
    await expect(page.getByRole("button", {name: "Reload page"})).not.toBeVisible();
});

test("a lost response is reported as unconfirmed and never retried automatically", async ({page}) => {
    await openSystem(page);
    let requests = 0;
    await page.route("**/api/system/reboot", async route => {
        requests++;
        await route.abort("connectionreset");
    });
    await page.getByRole("button", {name: "Reboot Pi", exact: true}).click();
    const dialog = page.getByRole("dialog");
    await dialog.getByRole("button", {name: "Reboot Pi", exact: true}).click();
    await expect(dialog.getByText("Request could not be confirmed", {exact: true})).toBeVisible();
    await expect(dialog.getByRole("button", {name: "Reboot Pi", exact: true})).toBeDisabled();
    expect(requests).toBe(1);
    await expect(page.getByText("Reboot requested", {exact: true})).not.toBeVisible();
    await dialog.getByRole("button", {name: "Cancel", exact: true}).click();
    expect(requests).toBe(1);
});
