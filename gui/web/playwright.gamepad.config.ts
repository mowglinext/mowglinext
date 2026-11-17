import {defineConfig, devices} from '@playwright/test';
import base from './playwright.config';

const port = Number(process.env.E2E_PORT) || 5173;

// WebKit covers focus/Drawer sequencing as well as the Chrome controller path.
// These are mocked browser workflows, not a physical Safari/Bluetooth test.
export default defineConfig({
    ...base,
    testMatch: ['**/gamepad.spec.ts', '**/gamepad-failures.spec.ts'],
    workers: 1,
    projects: [
        {name: 'chromium', use: {...devices['Desktop Chrome']}},
        {name: 'webkit', use: {...devices['Desktop Safari']}},
    ],
    // Exercise the bundle deployed to clients. Development StrictMode's effect
    // replay hits a Mapbox globe/source teardown error in the WebKit fixture.
    webServer: {
        command: `npm run build && npx vite preview --port ${port} --strictPort`,
        url: `http://localhost:${port}`,
        reuseExistingServer: false,
        timeout: 180_000,
    },
});
