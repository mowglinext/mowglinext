import {expect, test, type Page} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';

interface InputOptions {
    mapping?: string;
    getGamepadsThrows?: boolean;
    denyProfileStorage?: boolean;
}

interface GamepadFrame {
    deadman?: boolean;
    twist?: {twist?: {linear?: {x?: number}; angular?: {z?: number}}};
}

interface FixtureState {
    opened: number;
    closed: number;
    frames: GamepadFrame[];
    commands: number[];
    pageErrors: string[];
}

async function setup(page: Page, options: InputOptions = {}): Promise<FixtureState> {
    await page.addInitScript(({mapping, getGamepadsThrows, denyProfileStorage}) => {
        localStorage.setItem('i18nextLng', 'en');
        const input = {
            axes: [0, 0], buttons: Array(16).fill(false), connected: true,
            mapping, getGamepadsThrows,
        };
        Object.assign(window, {controllerFailureInput: input});
        if (denyProfileStorage) {
            const originalSetItem = Storage.prototype.setItem;
            Storage.prototype.setItem = function(key: string, value: string) {
                if (key.startsWith('mowglinext.gamepad.v1:')) {
                    throw new DOMException('Storage is blocked', 'QuotaExceededError');
                }
                return originalSetItem.call(this, key, value);
            };
        }
        Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: () => {
            if (input.getGamepadsThrows) throw new DOMException('Permission denied', 'SecurityError');
            if (!input.connected) return [null];
            return [{
                id: 'Fixture standard controller', index: 0, mapping: input.mapping, connected: true,
                timestamp: Math.max(1, performance.now()), axes: input.axes,
                buttons: input.buttons.map(pressed => ({pressed, touched: pressed, value: pressed ? 1 : 0})),
            }];
        }});
    }, {
        mapping: options.mapping ?? 'standard',
        getGamepadsThrows: options.getGamepadsThrows ?? false,
        denyProfileStorage: options.denyProfileStorage ?? false,
    });

    const state: FixtureState = {opened: 0, closed: 0, frames: [], commands: [], pageErrors: []};
    page.on('pageerror', error => state.pageErrors.push(error.message));
    await installMockBackend(page, {
        name: 'browser-controller-failures',
        rest: {'/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}},
        topics: {emergency: {active_emergency: false, latched_emergency: false}},
        topicSequences: {
            highLevelStatus: [
                {state: 3, state_name: 'RECORDING', emergency: false},
                {state: 3, state_name: 'RECORDING', emergency: false},
            ],
        },
    }, {liveStatusIntervalMs: 200});
    await page.route('**/api/mowglinext/call/high_level_control', async route => {
        state.commands.push(route.request().postDataJSON().Command);
        await route.fulfill({json: {}});
    });
    await page.routeWebSocket(/\/api\/mowglinext\/publish\/gamepad/, socket => {
        state.opened++;
        socket.onMessage(raw => state.frames.push(JSON.parse(String(raw)) as GamepadFrame));
        socket.onClose(() => { state.closed++; });
    });
    await page.routeWebSocket(/\/api\/mowglinext\/publish\/joy/, () => {});
    await page.route('https://api.mapbox.com/styles/v1/**', route => route.fulfill({json: {
        version: 8,
        sources: {},
        layers: [],
        glyphs: 'https://gamepad-fixture.invalid/fonts/{fontstack}/{range}.pbf',
    }}));
    await page.route('https://gamepad-fixture.invalid/fonts/**', route => route.fulfill({
        status: 200, contentType: 'application/x-protobuf', body: Buffer.alloc(0),
    }));
    await page.goto('/#/map');
    return state;
}

async function openSettings(page: Page) {
    await page.getByRole('button', {name: 'Controller', exact: true}).click();
    return page.getByRole('dialog');
}

async function selectController(page: Page, dialog: Awaited<ReturnType<typeof openSettings>>) {
    await dialog.getByRole('combobox').first().click();
    await page.getByText('1: Fixture standard controller', {exact: true}).click();
}

function expectClean(state: FixtureState) {
    expect(state.pageErrors).toEqual([]);
    expect(state.commands).toEqual([]);
}

test('a blocked or missing Gamepad API reports the problem without opening a control socket', async ({page}) => {
    const state = await setup(page, {getGamepadsThrows: true});
    const dialog = await openSettings(page);

    await expect(dialog.getByText('Controller input is unavailable or blocked by this browser. Use a browser with Gamepad API support.')).toBeVisible();
    await expect(dialog.getByRole('button', {name: 'Enable for this robot', exact: true})).toBeDisabled();
    await dialog.getByRole('button', {name: 'Close', exact: true}).click();
    await page.evaluate(() => Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: undefined}));
    const missingApiDialog = await openSettings(page);
    await expect(missingApiDialog.getByText('Controller input is unavailable or blocked by this browser. Use a browser with Gamepad API support.')).toBeVisible();
    await expect(missingApiDialog.getByRole('button', {name: 'Enable for this robot', exact: true})).toBeDisabled();

    expect(state.opened).toBe(0);
    expectClean(state);
});

test('unknown layouts stay disabled when saving the required mapping is denied', async ({page}) => {
    const state = await setup(page, {mapping: '', denyProfileStorage: true});
    const dialog = await openSettings(page);
    await selectController(page, dialog);

    await expect(dialog.getByText(/Unsaved mapping/)).toBeVisible();
    const enable = dialog.getByRole('button', {name: 'Enable for this robot', exact: true});
    await expect(enable).toBeDisabled();
    await dialog.getByRole('button', {name: 'Save mapping', exact: true}).click();
    await expect(page.getByText('Could not save the mapping. Allow browser storage and try again.')).toBeVisible();
    await expect(dialog.getByText(/Unsaved mapping/)).toBeVisible();
    await expect(enable).toBeDisabled();

    expect(state.opened).toBe(0);
    expectClean(state);
});

test('conflicting axes prevent saving or enabling a controller profile', async ({page}) => {
    const state = await setup(page);
    const dialog = await openSettings(page);
    await selectController(page, dialog);

    await dialog.getByRole('button', {name: 'Bind input', exact: true}).first().click();
    await expect(dialog.getByRole('status')).toContainText('Forward / reverse axis');
    await page.waitForTimeout(150); // let the mapper observe a neutral sample first
    await page.evaluate(() => { (window as any).controllerFailureInput.axes[0] = 1; });
    await expect(dialog.getByText('Use a different button for each function and a different axis for driving and steering. All assigned inputs must exist on this controller.')).toBeVisible();
    await expect(dialog.getByRole('button', {name: 'Save mapping', exact: true})).toBeDisabled();
    await expect(dialog.getByRole('button', {name: 'Enable for this robot', exact: true})).toBeDisabled();

    expect(state.opened).toBe(0);
    expectClean(state);
});

test('disconnect and reconnect do not re-arm the browser controller session', async ({page}) => {
    const state = await setup(page);
    const dialog = await openSettings(page);
    await selectController(page, dialog);
    await dialog.getByRole('button', {name: 'Enable for this robot', exact: true}).click();
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toBeVisible();
    await expect.poll(() => state.opened).toBe(1);
    await expect.poll(() => state.frames.length).toBeGreaterThan(0);

    await page.evaluate(() => {
        const input = (window as any).controllerFailureInput;
        input.connected = false;
        window.dispatchEvent(Object.assign(new Event('gamepaddisconnected'), {gamepad: {index: 0}}));
    });
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toHaveCount(0);
    await expect(page.locator('.ant-tag[role="alert"]')).toContainText('Input lost or stale');
    await expect.poll(() => state.closed).toBe(1);
    const neutralFrame = state.frames.at(-1);
    expect(neutralFrame?.deadman).toBe(false);
    expect(neutralFrame?.twist?.twist?.linear?.x).toBe(0);
    expect(neutralFrame?.twist?.twist?.angular?.z).toBe(0);

    await page.evaluate(() => {
        const input = (window as any).controllerFailureInput;
        input.connected = true;
        window.dispatchEvent(Object.assign(new Event('gamepadconnected'), {gamepad: {index: 0}}));
    });
    await page.waitForTimeout(250);
    expect(state.opened).toBe(1);
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toHaveCount(0);
    expectClean(state);
});

test('blur closes the controller stream and restores the recording touch controls', async ({page}) => {
    const state = await setup(page);
    const dialog = await openSettings(page);
    await selectController(page, dialog);
    await expect(page.getByRole('button', {name: 'check Finish', exact: true})).toBeVisible();
    await dialog.getByRole('button', {name: 'Enable for this robot', exact: true}).click();
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toBeVisible();
    await expect(page.getByRole('button', {name: 'check Finish', exact: true})).toHaveCount(0);
    await expect.poll(() => state.opened).toBe(1);

    await page.evaluate(() => window.dispatchEvent(new Event('blur')));
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toHaveCount(0);
    await expect.poll(() => state.closed).toBe(1);
    await expect(page.locator('.ant-tag[role="alert"]')).toContainText('Input lost or stale');
    await expect(page.getByRole('button', {name: 'check Finish', exact: true})).toBeVisible();
    const neutralFrame = state.frames.at(-1);
    expect(neutralFrame?.deadman).toBe(false);
    expect(neutralFrame?.twist?.twist?.linear?.x).toBe(0);
    expect(neutralFrame?.twist?.twist?.angular?.z).toBe(0);
    expectClean(state);
});
