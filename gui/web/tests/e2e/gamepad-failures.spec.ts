import {expect, test, type Page} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';

interface InputOptions {
    mapping?: string;
    insecureContext?: boolean;
    getGamepadsThrows?: boolean;
    denyProfileStorage?: boolean;
    deferFirstGamepadHello?: boolean;
}

interface GamepadFrame {
    deadman?: boolean;
    physicalDeadman?: boolean;
    neutral?: boolean;
    epoch?: number;
    twist?: {twist?: {linear?: {x?: number}; angular?: {z?: number}}};
}

interface FixtureState {
    opened: number;
    closed: number;
    frames: GamepadFrame[];
    challenges: string[];
    claims: string[];
    echoedHellos: string[];
    preReadyFrames: GamepadFrame[];
    commands: number[];
    pageErrors: string[];
    releaseLateHello: (() => void) | null;
    control: ((epoch: number, phase: 'inactive' | 'active' | 'pausing' | 'paused' | 'resuming', resumed?: true) => void) | null;
    status: {state: number; state_name: string; emergency: boolean};
}

async function setup(page: Page, options: InputOptions = {}): Promise<FixtureState> {
    await page.addInitScript(({mapping, getGamepadsThrows, denyProfileStorage, insecureContext}) => {
        if (insecureContext) Object.defineProperty(window, 'isSecureContext', {configurable: true, value: false});
        localStorage.setItem('i18nextLng', 'en');
        const input = {
            axes: [0, 0], buttons: Array(16).fill(false), connected: true,
            mapping, getGamepadsThrows, reads: 0,
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
            input.reads++;
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
        insecureContext: options.insecureContext ?? false,
        getGamepadsThrows: options.getGamepadsThrows ?? false,
        denyProfileStorage: options.denyProfileStorage ?? false,
    });

    const state: FixtureState = {opened: 0, closed: 0, frames: [], challenges: [], claims: [], echoedHellos: [], preReadyFrames: [], commands: [], pageErrors: [],
        releaseLateHello: null, control: null,
        status: {state: 3, state_name: 'RECORDING', emergency: false}};
    page.on('pageerror', error => state.pageErrors.push(error.message));
    await installMockBackend(page, {
        name: 'browser-controller-failures',
        rest: {'/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}},
        topics: {emergency: {active_emergency: false, latched_emergency: false}},
        topicSequences: {
            highLevelStatus: [state.status, state.status],
        },
    }, {liveStatusIntervalMs: 200});
    await page.route('**/api/mowglinext/gamepad/diagnostics', route => route.fulfill({status: 204}));
    await page.route('**/api/mowglinext/call/high_level_control', async route => {
        state.commands.push(route.request().postDataJSON().Command);
        await route.fulfill({json: {}});
    });
    await page.routeWebSocket(/\/api\/mowglinext\/publish\/gamepad/, socket => {
        state.opened++;
        const challenge = state.opened.toString(16).padStart(32, '0');
        state.challenges.push(challenge);
        let ready = false;
        state.control = (epoch, phase, resumed) => socket.send(JSON.stringify({type: 'gamepadControl', epoch, phase, ...(resumed ? {resumed} : {})}));
        socket.onMessage(raw => {
            const message = JSON.parse(String(raw)) as GamepadFrame & {type?: string; challenge?: string; version?: number};
            if (message.type === 'gamepadClaim') {
                state.claims.push(message.challenge ?? '');
                if (message.challenge === challenge && message.version === 3) {
                    ready = true;
                    socket.send(JSON.stringify({type: 'gamepadReady'}));
                }
                return;
            }
            if (message.type === 'gamepadHello') {
                state.echoedHellos.push(message.challenge ?? '');
                return;
            }
            if (!ready) state.preReadyFrames.push(message);
            state.frames.push(message);
        });
        const hello = JSON.stringify({type: 'gamepadHello', challenge, version: 3});
        if (options.deferFirstGamepadHello && state.opened === 1) state.releaseLateHello = () => socket.send(hello);
        else socket.send(hello);
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

async function finishSettings(page: Page, dialog: Awaited<ReturnType<typeof openSettings>>) {
    await expect(dialog.getByRole('button', {name: 'Done', exact: true})).toBeEnabled();
    await dialog.getByRole('button', {name: 'Done', exact: true}).click();
    await expect(dialog).not.toBeVisible();
    return page.getByRole('button', {name: 'Enable for this robot', exact: true});
}

function expectClean(state: FixtureState) {
    expect(state.pageErrors).toEqual([]);
    expect(state.commands).toEqual([]);
    expect(state.echoedHellos).toEqual([]);
}

function expectReadySessions(state: FixtureState) {
    expect(state.claims).toEqual(state.challenges);
    expect(state.preReadyFrames).toEqual([]);
}

test('an exposed Gamepad API can enable outside a secure context and stops on deadman release', async ({page}) => {
    const state = await setup(page, {insecureContext: true});
    expect(await page.evaluate(() => window.isSecureContext)).toBe(false);
    const dialog = await openSettings(page);
    await selectController(page, dialog);
    const enable = await finishSettings(page, dialog);
    await expect(enable).toBeEnabled();
    expect(state.opened).toBe(0);
    await enable.click();
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toBeVisible();
    await expect.poll(() => state.frames.length).toBeGreaterThan(2);
    expectReadySessions(state);
    expect(state.frames.every(frame => frame.deadman === false && frame.twist?.twist?.linear?.x === 0)).toBe(true);

    await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = true; });
    await expect.poll(() => state.frames.at(-1)?.deadman).toBe(true);
    await page.evaluate(() => { (window as any).controllerFailureInput.axes[1] = -1; });
    await expect.poll(() => state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0.25);
    await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = false; });
    await expect.poll(() => state.frames.at(-1)?.deadman).toBe(false);
    expect(state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0);
    expect(state.frames.at(-1)?.twist?.twist?.angular?.z).toBe(0);
    expect(state.opened).toBe(1);
    expectReadySessions(state);
    expectClean(state);
});

test('a blocked or missing Gamepad API reports the problem without opening a control socket', async ({page}) => {
    const state = await setup(page, {getGamepadsThrows: true, insecureContext: true});
    const dialog = await openSettings(page);

    await expect(dialog.getByText('Controller input is unavailable or blocked by this browser. Try a browser with Gamepad API support, or HTTPS if input is blocked over HTTP.')).toBeVisible();
    await finishSettings(page, dialog);
    await expect(page.getByRole('button', {name: 'Enable for this robot', exact: true})).toHaveCount(0);
    await page.evaluate(() => Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: undefined}));
    const missingApiDialog = await openSettings(page);
    await expect(missingApiDialog.getByText('Controller input is unavailable or blocked by this browser. Try a browser with Gamepad API support, or HTTPS if input is blocked over HTTP.')).toBeVisible();
    await finishSettings(page, missingApiDialog);
    await expect(page.getByRole('button', {name: 'Enable for this robot', exact: true})).toHaveCount(0);

    expect(state.opened).toBe(0);
    expectClean(state);
});

test('unknown layouts stay disabled when saving the required mapping is denied', async ({page}) => {
    const state = await setup(page, {mapping: '', denyProfileStorage: true});
    const dialog = await openSettings(page);
    await selectController(page, dialog);

    await expect(dialog.getByText(/Unsaved mapping/)).toBeVisible();
    await expect(page.getByRole('button', {name: 'Enable for this robot', exact: true})).toBeDisabled();
    await dialog.getByRole('button', {name: 'Save mapping', exact: true}).click();
    await expect(page.getByText('Could not save the mapping. Allow browser storage and try again.')).toBeVisible();
    await expect(dialog.getByText(/Unsaved mapping/)).toBeVisible();
    const enable = await finishSettings(page, dialog);
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
    const reads = await page.evaluate(() => (window as any).controllerFailureInput.reads as number);
    // Axis learning needs two stable preview samples. Wait for actual polls,
    // rather than assuming they occurred during a fixed wall-clock delay.
    await page.waitForFunction(start => (window as any).controllerFailureInput.reads >= start + 6, reads);
    await page.evaluate(() => { (window as any).controllerFailureInput.axes[0] = 1; });
    await expect(dialog.getByText('Use a different button for each function and a different axis for driving and steering. All assigned inputs must exist on this controller.')).toBeVisible();
    await expect(dialog.getByRole('button', {name: 'Save mapping', exact: true})).toBeDisabled();
    const enable = await finishSettings(page, dialog);
    await expect(enable).toBeDisabled();

    expect(state.opened).toBe(0);
    expectClean(state);
});

test('disconnect and reconnect do not re-arm the browser controller session', async ({page}) => {
    const state = await setup(page);
    const dialog = await openSettings(page);
    await selectController(page, dialog);
    const enable = await finishSettings(page, dialog);
    await enable.click();
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toBeVisible();
    await expect.poll(() => state.opened).toBe(1);
    await expect.poll(() => state.frames.length).toBeGreaterThan(0);

    await page.evaluate(() => {
        const input = (window as any).controllerFailureInput;
        input.connected = false;
        window.dispatchEvent(Object.assign(new Event('gamepaddisconnected'), {gamepad: {index: 0}}));
    });
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toHaveCount(0);
    await expect(page.locator('.ant-tag[role="alert"]')).toContainText('The browser no longer reports the selected controller as connected.');
    await expect.poll(() => state.closed).toBe(1);
    const neutralFrame = state.frames.at(-1);
    expect(neutralFrame?.deadman).toBe(false);
    expect(neutralFrame?.twist?.twist?.linear?.x).toBe(0);
    expect(neutralFrame?.twist?.twist?.angular?.z).toBe(0);
    expectReadySessions(state);

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
    const enable = await finishSettings(page, dialog);
    await enable.click();
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toBeVisible();
    await expect(page.getByRole('button', {name: 'check Finish', exact: true})).toHaveCount(0);
    await expect(page.getByRole('status')).toContainText('Ready — hold RB / R1 (Button 5)');
    await expect.poll(() => state.opened).toBe(1);
    await expect.poll(() => state.frames.length).toBeGreaterThan(0);
    await expect.poll(() => state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0);

    await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = true; });
    await expect.poll(() => state.frames.at(-1)?.deadman).toBe(true);
    await page.evaluate(() => { (window as any).controllerFailureInput.axes[1] = -1; });
    await expect.poll(() => state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0.25);

    await page.evaluate(() => window.dispatchEvent(new Event('blur')));
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toHaveCount(0);
    await expect.poll(() => state.closed).toBe(1);
    await expect(page.locator('.ant-tag[role="alert"]')).toContainText('The browser page lost focus.');
    await expect(page.getByRole('button', {name: 'check Finish', exact: true})).toBeVisible();
    await expect.poll(() => state.frames.at(-1)?.deadman).toBe(false);
    const neutralFrame = state.frames.at(-1);
    expect(neutralFrame?.deadman).toBe(false);
    expect(neutralFrame?.twist?.twist?.linear?.x).toBe(0);
    expect(neutralFrame?.twist?.twist?.angular?.z).toBe(0);
    expectReadySessions(state);
    expectClean(state);
});

test('canceling a pending hello sends no claim, then explicit enable claims a fresh socket', async ({page}) => {
    const state = await setup(page, {deferFirstGamepadHello: true});
    const dialog = await openSettings(page);
    await selectController(page, dialog);
    const enable = await finishSettings(page, dialog);

    await enable.click();
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toBeVisible();
    await expect.poll(() => state.opened).toBe(1);
    expect(state.claims).toEqual([]);
    expect(state.frames).toEqual([]);

    await page.evaluate(() => window.dispatchEvent(new Event('blur')));
    await expect(enable).toBeEnabled();
    await expect.poll(() => state.closed).toBe(1);
    try { state.releaseLateHello?.(); } catch { /* A closed route cannot deliver a message to its retired socket. */ }
    await page.waitForTimeout(100);
    expect(state.claims).toEqual([]);
    expect(state.frames).toEqual([]);

    await page.evaluate(() => {
        const input = (window as any).controllerFailureInput;
        input.axes.fill(0);
        input.buttons.fill(false);
    });
    await enable.click();
    await expect.poll(() => state.opened).toBe(2);
    await expect.poll(() => state.claims.length).toBe(1);
    await expect.poll(() => state.frames.length).toBeGreaterThan(0);
    expect(state.claims).toEqual([state.challenges[1]]);
    expect(state.echoedHellos).toEqual([]);
    expect(state.preReadyFrames).toEqual([]);
    expectClean(state);
});

test('re-enables directly after a focus fault and rearms after switching to manual mowing', async ({page}) => {
    const state = await setup(page);
    const dialog = await openSettings(page);
    await selectController(page, dialog);
    const enable = await finishSettings(page, dialog);
    const stableButton = await enable.elementHandle();
    if (!stableButton) throw new Error('Controller activation button is missing');
    expect(state.opened).toBe(0);
    await enable.click();
    await expect(page.getByRole('status')).toContainText('Ready — hold RB / R1 (Button 5)');

    await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = true; });
    await expect.poll(() => state.frames.at(-1)?.deadman).toBe(true);
    await page.evaluate(() => { (window as any).controllerFailureInput.axes[1] = -1; });
    await expect.poll(() => state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0.25);
    expectReadySessions(state);
    await page.evaluate(() => window.dispatchEvent(new Event('blur')));
    await expect(enable).toBeEnabled();
    await expect(page.locator('.ant-tag[role="alert"]')).toContainText('The browser page lost focus.');
    await expect.poll(() => state.closed).toBe(1);
    expect(await stableButton.evaluate(button => button.isConnected && button.textContent === 'Enable for this robot')).toBe(true);
    await expect(dialog).not.toBeVisible();

    await page.evaluate(() => {
        const input = (window as any).controllerFailureInput;
        input.axes.fill(0);
        input.buttons.fill(false);
    });
    await enable.click();
    await expect.poll(() => state.opened).toBe(2);
    await expect(page.getByRole('status')).toContainText('Ready — hold RB / R1 (Button 5)');
    expect(await stableButton.evaluate(button => document.activeElement === button && button.textContent === 'Disable controller')).toBe(true);
    await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = true; });
    await expect.poll(() => state.frames.at(-1)?.deadman).toBe(true);
    await page.evaluate(() => { (window as any).controllerFailureInput.axes[1] = -1; });
    await expect.poll(() => state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0.25);
    expectReadySessions(state);

    // The live mock robot changes mode while controls remain held. Old input
    // must stay neutral until the operator releases and presses deadman again.
    state.status.state = 4;
    state.status.state_name = 'MANUAL_MOWING';
    await expect(page.getByRole('status')).toContainText('Center sticks and release RB / R1 (Button 5)');
    await expect.poll(() => state.frames.at(-1)?.deadman).toBe(false);
    expect(state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0);
    await expect(page.getByRole('button', {name: 'Disable controller', exact: true})).toBeVisible();
    await page.evaluate(() => {
        const input = (window as any).controllerFailureInput;
        input.axes.fill(0);
        input.buttons.fill(false);
    });
    await expect(page.getByRole('status')).toContainText('Ready — hold RB / R1 (Button 5)');
    await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = true; });
    await expect.poll(() => state.frames.at(-1)?.deadman).toBe(true);
    await page.evaluate(() => { (window as any).controllerFailureInput.axes[1] = -1; });
    await expect.poll(() => state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0.25);
    expectClean(state);
});


for (const width of [390, 1440]) {
    test(`manual pause notices keep activation and require a new neutral deadman gesture at ${width}px`, async ({page}, info) => {
        await page.setViewportSize({width, height: 900});
        const state = await setup(page);
        state.status.state = 4;
        state.status.state_name = 'MANUAL_MOWING';
        const dialog = await openSettings(page);
        await selectController(page, dialog);
        await (await finishSettings(page, dialog)).click();
        await expect.poll(() => state.frames.length).toBeGreaterThan(2);
        const disable = page.getByRole('button', {name: 'Disable controller', exact: true});
        await expect(disable).toBeVisible();
        await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = true; });
        await expect.poll(() => state.frames.at(-1)?.deadman).toBe(true);
        state.control!(1, 'active');
        await expect.poll(() => state.frames.at(-1)?.epoch).toBe(1);
        let epoch = 1;
        for (let cycle = 0; cycle < 2; cycle++) {
            await page.evaluate(() => { (window as any).controllerFailureInput.axes[1] = -1; });
            await expect.poll(() => state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0.25);
            await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = false; });
            await expect.poll(() => state.frames.at(-1)?.physicalDeadman).toBe(false);
            expect(state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0);
            state.control!(++epoch, 'pausing');
            await expect(page.getByRole('status')).toContainText('Stopping Manual Mowing');
            await expect(disable).toBeVisible();
            state.status.state = 1;
            state.status.state_name = 'IDLE';
            state.control!(++epoch, 'paused');
            await expect(page.getByRole('status')).toContainText('Center sticks and release');
            await expect(page.getByRole('button', {name: 'Stop Manual Mowing', exact: true})).toBeVisible();
            await page.evaluate(() => { (window as any).controllerFailureInput.axes[1] = 0; });
            await expect(page.getByRole('status')).toContainText('Manual Mowing paused');
            // WebKit's screenshot capture can suspend native polling beyond
            // its real 200ms fail-safe; inspect both layouts in Chromium.
            if (cycle === 0 && info.project.name === 'chromium') {
                await page.screenshot({path: info.outputPath(`manual-paused-${width}px.png`)});
            }
            await expect.poll(() => state.frames.at(-1)?.neutral).toBe(true);
            await expect.poll(() => state.frames.at(-1)?.epoch).toBe(epoch);
            await page.evaluate(() => { (window as any).controllerFailureInput.buttons[5] = true; });
            await expect.poll(() => state.frames.at(-1)?.physicalDeadman).toBe(true);
            // The mock server grants resume only after the physical gesture; no HTTP
            // command 7 may be issued by the browser itself.
            expect(state.commands).toEqual([]);
            if (cycle === 0) {
                state.control!(++epoch, 'resuming');
                await expect(page.getByRole('status')).toContainText('Starting Manual Mowing');
            } else {
                // A fast server may coalesce resuming into the confirmed active
                // snapshot. Its explicit resume provenance must preserve the flow.
                epoch++;
            }
            expect(state.frames.at(-1)?.twist?.twist?.linear?.x).toBe(0);
            state.status.state = 4;
            state.status.state_name = 'MANUAL_MOWING';
            state.control!(++epoch, 'active', true);
            await expect.poll(() => state.frames.at(-1)?.epoch).toBe(epoch);
            await expect.poll(() => state.frames.at(-1)?.deadman).toBe(true);
        }
        await page.getByRole('button', {name: 'Stop Manual Mowing', exact: true}).click();
        await expect.poll(() => state.commands).toEqual([8]);
        state.control!(++epoch, 'inactive');
        state.status.state = 1;
        state.status.state_name = 'IDLE';
        await expect(page.getByRole('button', {name: 'Stop Manual Mowing', exact: true})).toHaveCount(0);
        await expect(disable).toBeVisible();
        expect(state.opened).toBe(1);
        expect(state.closed).toBe(0);
        expect(state.pageErrors).toEqual([]);
        expectReadySessions(state);
    });
}
