import {expect, test, type Locator, type Page} from '@playwright/test';
import {createRequire} from 'node:module';
import {installMockBackend} from './mock/mockBackend';
import type {Scenario} from './mock/scenarios';

const require = createRequire(import.meta.url);
const IOS_UA = 'Mozilla/5.0 (iPhone; CPU iPhone OS 17_6 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/17.6 Mobile/15E148 Safari/604.1';

async function setup(page: Page, state = 'MOWING', areas = 1) {
    await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
    // No WebSocket may reach a backend, including the dedicated teleop publisher.
    await page.routeWebSocket(/.*/, () => {});
    const topics = {
        highLevelStatus: {state: state === 'RECORDING' ? 3 : state === 'MANUAL_MOWING' ? 4 : 2, state_name: state, emergency: false, battery_percent: 76},
        emergency: {active_emergency: false, latched_emergency: false},
        status: {firmware_compatible: true, firmware_version: '1.0.0', rain_detected: false},
        power: {v_battery: 27, charge_current: 0},
        map: {working_area: Array.from({length: areas}, (_, i) => ({id: i + 1, name: `Lawn ${i + 1}`, area: {points: [{x: 0, y: 0}, {x: 10, y: 0}, {x: 10, y: 10}, {x: 0, y: 10}]}}))},
    };
    const scenario: Scenario = {
        name: 'responsive-map', topics,
        rest: {'/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}},
        topicSequences: Object.fromEntries(Object.entries(topics).map(([topic, value]) => [topic, [value, value]])),
    };
    await installMockBackend(page, scenario, {liveStatusIntervalMs: 500});
    await page.route(/^https:\/\//, route => route.abort());
    await page.route('**/api.mapbox.com/styles/v1/**', route => route.fulfill({json: {version: 8, glyphs: 'http://localhost/mock-glyphs/{fontstack}/{range}.pbf', sources: {}, layers: []}}));
    await page.route('**/mock-glyphs/**', route => route.fulfill({body: ''}));
    await page.route('**/mapbox-gl.css', route => route.fulfill({path: require.resolve('mapbox-gl/dist/mapbox-gl.css'), contentType: 'text/css'}));
    await page.goto('/#/map');
    await expect(page.locator('.mapboxgl-canvas')).toBeVisible({timeout: 15000});
}

async function reachable(control: Locator) {
    await control.click({trial: true, timeout: 5000});
    // Rounded scroll-container borders can clip a fractional pixel after scrolling.
    await expect(control).toBeInViewport({ratio: 0.99});
    await expect.poll(() => control.evaluate(node => {
        const box = node.getBoundingClientRect();
        const hit = document.elementFromPoint(box.x + box.width / 2, box.y + box.height / 2);
        return hit !== null && node.contains(hit);
    })).toBe(true);
}

test.describe('responsive map controls', () => {
    test.use({hasTouch: true});

    for (const width of [320, 390]) {
        test(`iOS prompt leaves STOP and map tools reachable at ${width}px`, async ({browser}, testInfo) => {
            const context = await browser.newContext({viewport: {width, height: 844}, hasTouch: true, userAgent: IOS_UA});
            const page = await context.newPage();
            await setup(page);
            await expect(page.getByText('Add to Home Screen', {exact: true})).toBeVisible();
            await reachable(page.getByRole('button', {name: 'Emergency stop', exact: true}));
            await reachable(page.getByRole('button', {name: 'Mow area', exact: true}));
            await reachable(page.getByRole('button', {name: 'Edit Map', exact: true}));
            await page.screenshot({path: testInfo.outputPath(`ios-${width}.png`)});
            await page.setViewportSize({width: 740, height: 360});
            await expect(page.getByText('Add to Home Screen', {exact: true})).toBeHidden();
            await reachable(page.getByRole('button', {name: 'Emergency stop', exact: true}));
            await page.setViewportSize({width, height: 844});
            await expect(page.getByText('Add to Home Screen', {exact: true})).toBeVisible();
            await page.getByRole('button', {name: 'Dismiss', exact: true}).tap();
            await expect(page.getByText('Add to Home Screen', {exact: true})).toBeHidden();
            await context.close();
        });
    }

    for (const state of ['MOWING', 'MANUAL_MOWING', 'RECORDING']) {
        for (const viewport of [{width: 740, height: 360}, {width: 844, height: 390}, {width: 1440, height: 900}]) {
            test(`${state} controls survive ${viewport.width}x${viewport.height} and rotation`, async ({page}, testInfo) => {
                await page.setViewportSize(viewport);
                await setup(page, state);
                const mow = page.getByRole('button', {name: 'Mow area', exact: true});
                await reachable(mow);
                if (viewport.width >= 768) {
                    await reachable(page.getByRole('button', {name: /Map rotation —/i}));
                    await reachable(page.getByRole('button', {name: /Mowing areas.*—/i}));
                }
                if (state === 'RECORDING') {
                    await reachable(page.getByRole('button', {name: /Finish$/}));
                    await reachable(page.getByRole('button', {name: /Cancel$/}).last());
                }
                if (state !== 'MOWING') {
                    const joystick = page.getByTestId('joystick-base');
                    await reachable(joystick);
                }
                await page.screenshot({path: testInfo.outputPath(`${state}-${viewport.width}.png`)});
                await page.setViewportSize({width: 390, height: 844});
                await reachable(page.getByRole('button', {name: 'Emergency stop', exact: true}));
                if (state === 'RECORDING') await reachable(page.getByRole('button', {name: /Finish$/}));
                await page.setViewportSize(viewport);
                await reachable(mow);
                if (state === 'RECORDING') await reachable(page.getByRole('button', {name: /Finish$/}));
            });
        }
    }

    test('thirty-area menu can reach first, middle and last by touch and keyboard', async ({page}, testInfo) => {
        await page.setViewportSize({width: 390, height: 844});
        await setup(page, 'MOWING', 30);
        await page.getByRole('button', {name: 'Mow area', exact: true}).click();
        const menu = page.getByRole('menu');
        await expect(menu).toBeVisible();
        const items = menu.getByRole('menuitem');
        await expect(items).toHaveCount(30);
        for (const index of [29, 0, 14]) {
            await reachable(items.nth(index));
            await items.nth(index).tap({trial: true});
        }
        await items.first().focus();
        await page.keyboard.press('End');
        await expect(items.last()).toBeFocused();
        await reachable(items.last());
        await page.keyboard.press('Home');
        await expect(items.first()).toBeFocused();
        await reachable(items.first());
        await page.screenshot({path: testInfo.outputPath('many-areas.png')});
    });
});
