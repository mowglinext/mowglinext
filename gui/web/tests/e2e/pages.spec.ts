import {test, expect} from "@playwright/test";
import {SCENARIOS} from "./mock/scenarios.ts";
import {installMockBackend} from "./mock/mockBackend.ts";

/**
 * Walk every page under every mocked robot-state permutation. Each test:
 *   1. installs the scenario mocks (REST + multiplex WS),
 *   2. loads the page,
 *   3. asserts the shell + page heading render (no crash / error boundary),
 *   4. captures a screenshot named "<page>__<scenario>.png" so every
 *      situation is reviewable and reproducible.
 */

const PAGES = [
    {route: "/mowglinext", heading: /Accueil|Home/i},
    {route: "/map", heading: /Carte|Map/i},
    {route: "/schedule", heading: /Planning|Schedule/i},
    {route: "/diagnostics", heading: /Diagnostic/i},
    {route: "/statistics", heading: /Stat/i},
    {route: "/settings", heading: /Réglages|Settings/i},
    {route: "/parameters", heading: /Paramètres|Parameters/i},
    {route: "/logs", heading: /Logs/i},
];

for (const scenario of SCENARIOS) {
    test.describe(`scenario: ${scenario.name}`, () => {
        for (const pg of PAGES) {
            test(`${pg.route}`, async ({page}) => {
                const errors: string[] = [];
                page.on("pageerror", (e) => errors.push(e.message));

                await installMockBackend(page, scenario);
                await page.goto(`/#${pg.route}`);

                // The left rail is always present once the shell mounts.
                await expect(page.getByText("MOWGLI").first()).toBeVisible({timeout: 15_000});
                // The page title (a styled div, not a semantic heading) confirms
                // the route actually rendered.
                await expect(page.getByText(pg.heading).first())
                    .toBeVisible({timeout: 15_000});

                // Let live data settle, then snapshot the state.
                await page.waitForTimeout(1200);
                await page.screenshot({
                    path: `tests/e2e/.artifacts/${pg.route.replace(/\//g, "")}__${scenario.name}.png`,
                    fullPage: true,
                });

                // No uncaught render errors in any state.
                expect(errors, `page errors on ${pg.route} / ${scenario.name}:\n${errors.join("\n")}`)
                    .toEqual([]);
            });
        }
    });
}
