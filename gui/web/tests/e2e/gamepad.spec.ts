import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import type {GamepadDiagnosticReport} from '../../src/gamepad/diagnostics';

test.afterEach(async ({page}, info) => {
    if (info.status !== info.expectedStatus) {
        const trace = await page.evaluate(() => (window as any).controllerPollTrace ?? []).catch(() => []);
        await info.attach('controller-poll-trace', {body: JSON.stringify(trace), contentType: 'application/json'});
    }
});

for (const width of [390, 1440]) {
    test(`browser controller mapping, driving and loss at ${width}px`, async ({page}) => {
        await page.setViewportSize({width, height: 900});
        await page.addInitScript(() => {
            localStorage.setItem('i18nextLng', 'en');
            // WebKit exposes an independent clock and updates only on value changes.
            const input = {axes: [0, 0], buttons: Array(16).fill(false), connected: true, signature: '', timestamp: 864000};
            const trace: Array<{at: number; stamp: number; axes: number[]; deadman: boolean}> = [];
            Object.assign(window, {controllerTestInput: input, controllerPollTrace: trace});
            Object.defineProperty(navigator, 'getGamepads', {value: () => {
                const signature = JSON.stringify([input.axes, input.buttons]);
                if (signature !== input.signature) {
                    input.signature = signature;
                    input.timestamp += 0.001;
                }
                trace.push({at: performance.now(), stamp: input.timestamp, axes: [...input.axes], deadman: input.buttons[4]});
                if (trace.length > 1000) trace.shift();
                return input.connected ? [{
                    id: 'Fixture standard controller', index: 0, mapping: 'standard', connected: true,
                    timestamp: input.timestamp, axes: input.axes,
                    buttons: input.buttons.map(pressed => ({pressed, touched: pressed, value: pressed ? 1 : 0})),
                }] : [null];
            }});
        });
        await installMockBackend(page, {
            name: 'browser-controller',
            rest: {'/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}},
            topics: {emergency: {active_emergency: false, latched_emergency: false}},
            topicSequences: {highLevelStatus: [{state: 3, state_name: 'RECORDING', emergency: false}, {state: 3, state_name: 'RECORDING', emergency: false}]},
        }, {liveStatusIntervalMs: 200});
        const frames: Array<{twist: {twist: {linear: {x: number}; angular: {z: number}}}; deadman: boolean}> = [];
        const commands: number[] = [];
        const reports: GamepadDiagnosticReport[] = [];
        const reportOrigins: (string | undefined)[] = [];
        const errors: string[] = [];
        let opened = 0;
        page.on('pageerror', error => errors.push(error.message));
        page.on('console', msg => { if (msg.type() === 'error' && msg.text().includes('same key')) errors.push(msg.text()); });
        await page.route('**/api/mowglinext/call/high_level_control', async route => {
            commands.push(route.request().postDataJSON().Command);
            await route.fulfill({json: {}});
        });
        await page.route('**/api/mowglinext/gamepad/diagnostics', async route => {
            reports.push(route.request().postDataJSON() as GamepadDiagnosticReport);
            reportOrigins.push(route.request().headers().origin);
            await route.fulfill({status: 204});
        });
        await page.routeWebSocket(/\/api\/mowglinext\/publish\/gamepad/, socket => {
            opened++;
            socket.onMessage(raw => frames.push(JSON.parse(String(raw))));
        });
        await page.routeWebSocket(/\/api\/mowglinext\/publish\/joy/, () => {});
        await page.route('https://api.mapbox.com/styles/v1/**', route => route.fulfill({json: {version: 8, sources: {}, layers: [], glyphs: 'https://gamepad-fixture.invalid/fonts/{fontstack}/{range}.pbf'}}));
        await page.goto('/#/map');
        await page.getByRole('button', {name: 'Controller', exact: true}).click();
        const dialog = page.getByRole('dialog');
        await dialog.getByRole('combobox').first().click();
        await page.getByText('1: Fixture standard controller', {exact: true}).click();
        await expect(dialog.getByText('Built-in standard mapping', {exact: true})).toBeVisible();
        await expect(dialog.getByLabel('Live inputs (test before enabling)')).toBeVisible();
        await dialog.getByRole('button', {name: 'Bind input', exact: true}).nth(2).click();
        await expect(dialog.getByRole('status')).toContainText('Hold to drive');
        await page.waitForTimeout(150); // neutral sample before the bind gesture
        await page.evaluate(() => { (window as any).controllerTestInput.buttons[4] = true; });
        await expect(dialog.getByRole('status')).toHaveCount(0);
        await page.evaluate(() => { (window as any).controllerTestInput.buttons[4] = false; });
        await dialog.getByRole('button', {name: 'Save mapping', exact: true}).click();
        await expect(dialog.getByText('Saved mapping', {exact: true})).toBeVisible();
        await dialog.locator('.ant-drawer-body').evaluate(el => { el.scrollTop = 0; });
        await page.screenshot({path: `tests/e2e/.artifacts/gamepad-settings-${width}.png`, fullPage: true});
        expect(await dialog.evaluate(el => el.scrollWidth <= el.clientWidth)).toBe(true);
        await page.reload();
        await expect(dialog).not.toBeVisible();
        const enable = page.getByRole('button', {name: 'Enable for this robot', exact: true});
        await expect(enable).toBeEnabled();
        await expect(page.getByText('1: Fixture standard controller', {exact: true})).toBeVisible();
        await page.waitForTimeout(650); // Restoring centered input must not activate a session.
        expect(opened).toBe(0);
        expect(frames).toHaveLength(0);
        await enable.click();
        await expect(page.getByRole('button', {name: 'Disable controller'})).toBeVisible();
        await expect.poll(() => frames.length).toBeGreaterThan(1);
        await expect(page.getByRole('status')).toContainText('LB / L1 (Button 4)');
        await expect(page.getByText('Drive: hold LB / L1 (Button 4) · Stop: A / × (Button 0) · Max 0.25 m/s')).toBeVisible();
        await page.evaluate(() => { (window as any).controllerTestInput.buttons[4] = true; });
        await expect.poll(() => frames.some(frame => frame.deadman)).toBe(true);
        await page.evaluate(() => { (window as any).controllerTestInput.axes[1] = -1; });
        await expect.poll(() => frames.at(-1)?.twist.twist.linear.x).toBe(0.25);
        await page.waitForTimeout(650); // A constant stick/deadman is valid input.
        await expect(page.getByRole('button', {name: 'Disable controller'})).toBeVisible();
        expect(frames.at(-1)?.twist.twist.linear.x).toBe(0.25);
        await page.evaluate(() => { (window as any).controllerTestInput.buttons[4] = false; });
        await expect.poll(() => frames.at(-1)?.twist.twist.linear.x).toBe(0);
        expect(frames.at(-1)?.deadman).toBe(false);
        await page.evaluate(() => { (window as any).controllerTestInput.axes[1] = 0; });
        await page.waitForTimeout(150);
        await page.evaluate(() => { (window as any).controllerTestInput.buttons[2] = true; });
        await expect.poll(() => commands).toEqual([5]); // 800 ms hold, one input event.
        await page.evaluate(() => { (window as any).controllerTestInput.buttons[2] = false; (window as any).controllerTestInput.buttons[0] = true; });
        await expect.poll(() => commands).toEqual([5, 8]);
        await page.evaluate(() => { (window as any).controllerTestInput.connected = false; });
        await expect(page.getByRole('button', {name: 'Disable controller'})).toHaveCount(0);
        await expect(page.getByRole('alert')).toContainText('The browser no longer reports the selected controller as connected.');
        await expect.poll(() => reports.some(report => report.events.some(event =>
            event.event === 'sessionEnded' && event.detail.reason === 'disconnected'))).toBe(true);
        expect(reportOrigins.every(origin => origin === new URL(page.url()).origin)).toBe(true);
        await page.evaluate(() => {
            const input = (window as any).controllerTestInput;
            input.connected = true;
            input.buttons.fill(false);
            input.axes[1] = 1;
        });
        await page.getByRole('button', {name: 'Controller', exact: true}).click();
        const live = dialog.getByLabel('Live inputs (test before enabling)');
        await expect(live).toContainText('Axis 1: 1.00');
        await page.evaluate(() => { (window as any).controllerTestInput.axes[1] = 0; });
        await expect(live).toContainText('Axis 1: 0.00');
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
        const stoppedCount = frames.length;
        await page.waitForTimeout(350);
        expect(frames.length).toBe(stoppedCount);
        expect(frames.at(-1)?.twist.twist.linear.x).toBe(0);
        expect(errors).toEqual([]);
    });
}
