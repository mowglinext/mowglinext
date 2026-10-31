import {expect, test} from "@playwright/test";
import {installMockBackend} from "./mock/mockBackend.ts";
import {SCENARIOS} from "./mock/scenarios.ts";

test.use({hasTouch: true});

const settings = {
    robot_name: "Mowgli",
    mower_model: "YardForce500",
    wheel_radius: 0.1,
    wheel_track: 0.325,
    blade_radius: 0.09,
    tool_width: 0.18,
    ticks_per_meter: 280.441,
};

async function openMocked(page: import("@playwright/test").Page, language = "en") {
    await page.addInitScript((lang) => localStorage.setItem("mowglinext.lang", lang), language);
    const scenario = {
        ...SCENARIOS[0],
        topics: {
            ...SCENARIOS[0].topics,
            status: {...SCENARIOS[0].topics?.status as object, firmware_compatible: false, firmware_version: "unknown"},
        },
        rest: {
            ...SCENARIOS[0].rest,
            "/api/settings/yaml": settings,
            "/api/settings/yaml/defaults": settings,
            "/api/settings/schema": {properties: {}},
        },
    };
    // Register the network guard first so the more-specific local mock routes
    // below take precedence for REST and WebSocket API requests.
    await page.route("**/*", async (route) => {
        const url = new URL(route.request().url());
        if (url.hostname === "localhost" || url.hostname === "127.0.0.1") {
            await route.fallback();
            return;
        }
        await route.fulfill({status: 204, body: ""});
    });
    await page.routeWebSocket(/.*/, () => { /* unmatched sockets stay local and silent */ });
    await installMockBackend(page, scenario);
}

async function robotNameWidth(page: import("@playwright/test").Page) {
    const box = await page.locator("#setting-robot_name").boundingBox();
    expect(box, "Robot Name input should be rendered").not.toBeNull();
    return box!.width;
}

test.describe("responsive Settings and onboarding forms", () => {
    test("Hardware gives Robot Name a usable width on tablet and desktop", async ({page}, testInfo) => {
        await page.setViewportSize({width: 768, height: 1024});
        await openMocked(page);
        await page.goto("/#/settings?section=hardware", {waitUntil: "domcontentloaded"});
        const input = page.locator("#setting-robot_name");
        await expect(input).toBeVisible();
        await expect(page.locator(".settings-preview-rail")).toBeHidden();

        const tabletWidth = await robotNameWidth(page);
        expect(tabletWidth).toBeGreaterThanOrEqual(240);
        await input.click({trial: true});
        await page.screenshot({path: testInfo.outputPath("settings-hardware-tablet-768.png")});

        // The columns must reflow again when the tablet rotates or the page is
        // opened on a standard desktop; the field must remain easy to edit.
        await page.setViewportSize({width: 921, height: 768});
        await expect(input).toBeVisible();
        await expect(page.locator(".settings-preview-rail")).toBeHidden();
        expect(await robotNameWidth(page)).toBeGreaterThanOrEqual(280);
        await input.click({trial: true});

        await page.setViewportSize({width: 1024, height: 768});
        await expect(page.locator(".settings-preview-rail")).toBeVisible();
        expect(await robotNameWidth(page)).toBeGreaterThanOrEqual(280);
        await page.setViewportSize({width: 1440, height: 900});
        await expect(page.locator(".settings-preview-rail")).toBeVisible();
        expect(await robotNameWidth(page)).toBeGreaterThanOrEqual(320);
        await input.click({trial: true});
    });

    for (const [language, skipName] of [
        ["en", "Skip without verifying"],
        ["fr", "Ignorer sans vérifier"],
    ] as const) {
        test(`firmware introduction actions wrap and stay reachable at 320px (${language})`, async ({page}, testInfo) => {
            await page.setViewportSize({width: 320, height: 568});
            await openMocked(page, language);
            await page.goto("/#/onboarding", {waitUntil: "domcontentloaded"});
            await page.getByRole("button", {name: language === "fr" ? "Commencer" : "Get Started"}).click();
            await page.getByRole("button", {name: language === "fr" ? "Suivant" : "Next"}).click();

            const flash = page.getByRole("button", {
                name: language === "fr" ? "Flasher le firmware" : "Flash Firmware",
                exact: true,
            });
            const skip = page.getByRole("button", {name: skipName, exact: true});
            await expect(flash).toBeVisible();
            await expect(skip).toBeVisible();
            const buttons = [flash, skip];
            const geometry = await Promise.all(buttons.map((button) => button.evaluate((node) => {
                const rect = node.getBoundingClientRect();
                return {x: rect.x, right: rect.right};
            })));
            for (const button of geometry) {
                expect(button.x).toBeGreaterThanOrEqual(0);
                expect(button.right).toBeLessThanOrEqual(320);
            }
            expect(await page.evaluate(() => document.documentElement.scrollWidth)).toBeLessThanOrEqual(320);

            // The mobile navigation stays fixed; explicitly scroll each action
            // into the main viewport before proving its centre is an active hit target.
            await skip.scrollIntoViewIfNeeded();
            await skip.click({trial: true});
            await flash.scrollIntoViewIfNeeded();
            await flash.click({trial: true});
            for (const button of buttons) {
                const targetIsButton = await button.evaluate((node) => {
                    const rect = node.getBoundingClientRect();
                    const target = document.elementFromPoint(rect.x + rect.width / 2, rect.y + rect.height / 2);
                    return target === node || node.contains(target);
                });
                expect(targetIsButton).toBe(true);
            }
            await page.screenshot({path: testInfo.outputPath(`firmware-intro-${language}-320.png`)});

            // Exercise both real actions: the primary CTA opens the separate
            // flash flow; skipping advances the wizard without API side effects.
            await flash.click();
            await expect(page.getByText(/Flash Firmware|Flasher le firmware/).last()).toBeVisible();
            const skipPage = await page.context().newPage();
            await skipPage.setViewportSize({width: 320, height: 568});
            await openMocked(skipPage, language);
            await skipPage.goto("/#/onboarding", {waitUntil: "domcontentloaded"});
            await skipPage.getByRole("button", {name: language === "fr" ? "Commencer" : "Get Started"}).click();
            await skipPage.getByRole("button", {name: language === "fr" ? "Suivant" : "Next"}).click();
            await skipPage.getByRole("button", {name: skipName, exact: true}).click();
            await expect(skipPage.getByRole("region", {name: /NTRIP/i})).toBeVisible();
            await skipPage.close();
        });
    }
});
