import {defineConfig, devices} from "@playwright/test";

/**
 * E2E harness for the MowgliNext GUI.
 *
 * Every page is driven entirely by MOCKED data — no Go backend and no robot
 * are required. REST is intercepted with page.route and the multiplex
 * WebSocket with page.routeWebSocket (see tests/e2e/mock/mockBackend.ts).
 * Scenarios in tests/e2e/mock/scenarios.ts enumerate the robot-state
 * permutations so every situation is reproducible in CI.
 */
export default defineConfig({
    testDir: "./tests/e2e",
    fullyParallel: true,
    forbidOnly: !!process.env.CI,
    retries: process.env.CI ? 1 : 0,
    reporter: process.env.CI ? "line" : [["list"], ["html", {open: "never"}]],
    outputDir: "./tests/e2e/.artifacts",
    use: {
        baseURL: "http://localhost:5173",
        trace: "on-first-retry",
        screenshot: "only-on-failure",
        // The app is dark-themed; give screenshots a stable viewport.
        viewport: {width: 1440, height: 900},
    },
    projects: [
        {name: "chromium", use: {...devices["Desktop Chrome"]}},
    ],
    webServer: {
        command: "npx vite --port 5173 --strictPort",
        url: "http://localhost:5173",
        reuseExistingServer: true,
        timeout: 120_000,
    },
});
