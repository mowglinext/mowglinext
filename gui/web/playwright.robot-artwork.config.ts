import {defineConfig} from "@playwright/test";
import base from "./playwright.config";

/** Opt-in cross-engine raster regressions for the shared robot artwork. */
export default defineConfig({
    ...base,
    testMatch: "**/robot-artwork-browsers.spec.ts",
    timeout: 90_000,
    projects: [
        {name: "chromium", use: {browserName: "chromium"}},
        {name: "firefox", use: {browserName: "firefox"}},
        {name: "webkit", use: {browserName: "webkit"}},
    ],
});
