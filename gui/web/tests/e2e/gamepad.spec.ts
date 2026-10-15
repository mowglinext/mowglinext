import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';

for (const width of [390, 1440]) {
    test(`browser controller mapping, driving and loss at ${width}px`, async ({page}) => {
        await page.setViewportSize({width, height: 900});
        await page.addInitScript(() => {
            localStorage.setItem('i18nextLng', 'en');
            const input = {axes: [0, 0], buttons: Array(16).fill(false), connected: true, frozen: false, timestamp: 0};
            Object.assign(window, {controllerTestInput: input});
            Object.defineProperty(navigator, 'getGamepads', {value: () => input.connected ? [{
                id: 'Fixture standard controller', index: 0, mapping: 'standard', connected: true,
                timestamp: input.frozen ? input.timestamp : performance.now(), axes: input.axes,
                buttons: input.buttons.map(pressed => ({pressed, touched: pressed, value: pressed ? 1 : 0})),
            }] : [null]});
        });
        await installMockBackend(page, {
            name: 'browser-controller',
            rest: {'/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}},
            topics: {emergency: {active_emergency: false, latched_emergency: false}},
            topicSequences: {highLevelStatus: [{state: 3, state_name: 'RECORDING', emergency: false}, {state: 3, state_name: 'RECORDING', emergency: false}]},
        }, {liveStatusIntervalMs: 200});
        const frames: Array<{twist: {twist: {linear: {x: number}; angular: {z: number}}}; deadman: boolean}> = [];
        const commands: number[] = [];
        const errors: string[] = [];
        page.on('pageerror', error => errors.push(error.message));
        page.on('console', msg => { if (msg.type() === 'error' && msg.text().includes('same key')) errors.push(msg.text()); });
        await page.route('**/api/mowglinext/call/high_level_control', async route => {
            commands.push(route.request().postDataJSON().Command);
            await route.fulfill({json: {}});
        });
        await page.routeWebSocket(/\/api\/mowglinext\/publish\/gamepad/, socket => {
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
        await page.getByRole('button', {name: 'Controller', exact: true}).click();
        await dialog.getByRole('combobox').first().click();
        await page.getByText('1: Fixture standard controller', {exact: true}).click();
        await expect(dialog.getByText('Saved mapping', {exact: true})).toBeVisible();
        await dialog.getByRole('button', {name: 'Enable for this robot', exact: true}).click();
        await expect(page.getByRole('button', {name: 'Disable controller'})).toBeVisible();
        await expect.poll(() => frames.length).toBeGreaterThan(1);
        await page.evaluate(() => { (window as any).controllerTestInput.buttons[4] = true; });
        await expect.poll(() => frames.some(frame => frame.deadman)).toBe(true);
        await page.evaluate(() => { (window as any).controllerTestInput.axes[1] = -1; });
        await expect.poll(() => frames.at(-1)?.twist.twist.linear.x).toBe(0.25);
        await page.evaluate(() => { (window as any).controllerTestInput.buttons[4] = false; });
        await expect.poll(() => frames.at(-1)?.twist.twist.linear.x).toBe(0);
        expect(frames.at(-1)?.deadman).toBe(false);
        await page.evaluate(() => { (window as any).controllerTestInput.axes[1] = 0; (window as any).controllerTestInput.buttons[0] = true; });
        await expect.poll(() => commands).toEqual([8]);
        await page.evaluate(() => { const input = (window as any).controllerTestInput; input.frozen = true; input.timestamp = performance.now(); });
        await expect(page.getByRole('button', {name: 'Disable controller'})).toHaveCount(0);
        await expect(page.getByRole('alert')).toContainText('Input lost or stale');
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
        const stoppedCount = frames.length;
        await page.waitForTimeout(350);
        expect(frames.length).toBe(stoppedCount);
        expect(frames.at(-1)?.twist.twist.linear.x).toBe(0);
        expect(errors).toEqual([]);
    });
}
