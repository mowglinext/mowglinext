import {expect, test} from "@playwright/test";
import {readFile} from "node:fs/promises";
import {BLACKBOX_STATUS, installMockBackend} from "./mock/mockBackend.ts";
import {SCENARIOS} from "./mock/scenarios.ts";

test("malformed blackbox responses stay isolated from Diagnostics and recover", async ({page}) => {
    const errors: string[] = [];
    page.on("pageerror", error => errors.push(error.message));
    await installMockBackend(page, {...SCENARIOS[0], rest: {"/api/tools/blackbox/status": {}}});
    await page.goto("/#/diagnostics");
    await expect(page.getByText("Invalid blackbox status response", {exact: true})).toBeVisible();
    await expect(page.getByRole("button", {name: "Host power…", exact: true})).toBeVisible();
    await page.getByRole("tab", {name: /Localisation/}).click();
    await expect(page.getByTestId("gnss-fix-status-value")).toBeVisible();
    await page.getByRole("tab", {name: /System/}).click();
    await page.route("**/api/tools/blackbox/status", route => route.fulfill({json: BLACKBOX_STATUS}));
    await expect(page.getByText("Invalid blackbox status response", {exact: true})).not.toBeVisible({timeout: 10_000});
    await expect(page.getByRole("button", {name: /Save blackbox now/})).toBeEnabled();
    expect(errors).toEqual([]);
});

test("Diagnostics saves, downloads and deletes a snapshot through the blackbox API", async ({page}) => {
    await installMockBackend(page, SCENARIOS[0]);
    const name = "blackbox-20261010T120000.000000000Z-0123456789abcdef.jsonl";
    let saved = false;
    let deleted = false;
    await page.route("**/api/tools/blackbox/save", route => {
        expect(route.request().method()).toBe("POST"); saved = true;
        return route.fulfill({status: 202, json: {capture_id: name.slice(0, -6), accepted: true}});
    });
    await page.route("**/api/tools/blackbox/status", route => route.fulfill({json: {
        ...BLACKBOX_STATUS,
        recordings: saved && !deleted ? [{name, size: 1024, triggered_at: "2026-10-10T12:00:00Z",
            reasons: ["manual"], actual_pre_seconds: 12, actual_post_seconds: 15,
            capture_dropped_messages: 0, interrupted: false}] : [],
    }}));
    await page.route(`**/api/tools/blackbox/download/${name}`, route => route.fulfill({
        headers: {"Content-Type": "application/x-ndjson", "Content-Disposition": `attachment; filename="${name}"`},
        body: '{"kind":"blackbox_metadata","reasons":["manual"]}\n{"kind":"blackbox_summary","complete":true}\n',
    }));
    await page.route(`**/api/tools/blackbox/${name}`, route => {
        expect(route.request().method()).toBe("DELETE"); deleted = true;
        return route.fulfill({json: {message: "snapshot deleted"}});
    });
    await page.goto("/#/diagnostics");
    await page.getByRole("button", {name: /Save blackbox now/}).click();
    await expect(page.getByText("12.0 s before · 15.0 s after")).toBeVisible();
    expect(saved).toBe(true);
    const pendingDownload = page.waitForEvent("download");
    await page.getByRole("link", {name: /Download/}).click();
    const download = await pendingDownload;
    expect(download.suggestedFilename()).toBe(name);
    const path = await download.path();
    expect(path).not.toBeNull();
    expect(await readFile(path!, "utf8")).toContain('"complete":true');
    await page.getByRole("button", {name: `Delete snapshot ${name}`, exact: true}).click();
    await page.getByRole("dialog").getByRole("button", {name: "Delete", exact: true}).click();
    await expect(page.getByRole("link", {name: /Download/})).toHaveCount(0);
    expect(deleted).toBe(true);
});
