import {expect, test, type Locator, type Page} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import gnss from '../../src/test/fixtures/gnss-mavros-sanitized.json' with {type: 'json'};

async function mockPage(page: Page, emergency = false, language = 'en') {
    await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
    const scenario = SCENARIOS.find(s => s.name === (emergency ? 'emergency-latched' : 'idle-docked-full'))!;
    const topics = {...scenario.topics, gnssStatus: gnss};
    await page.routeWebSocket(/.*/, () => {});
    await installMockBackend(page, {
        ...scenario,
        rest: {...scenario.rest, '/api/settings/yaml': {robot_name: 'Mowgli'},
            '/api/containers': {containers: [{id: 'layout-logs', names: ['/layout-logs'], state: 'running', labels: {app: 'mock'}}]}},
        topics,
        containerLogs: Array.from({length: 80}, (_, i) => `INFO layout line ${i}`),
        topicSequences: Object.fromEntries(Object.entries(topics).map(([key, value]) => [key, [value, value]])),
    }, {liveStatusIntervalMs: 500});
    await page.routeWebSocket(/\/api\/mowglinext\/publish\//, () => {});
    await page.route(/^https?:\/\/(?!localhost:|127\.0\.0\.1:)/, route => route.abort());
}

async function reachable(target: Locator) {
    await target.click({trial: true});
    expect(await target.evaluate(el => {
        const r = el.getBoundingClientRect();
        return r.left >= 0 && r.right <= innerWidth && r.top >= 0 && r.bottom <= innerHeight &&
            el.contains(document.elementFromPoint(r.x + r.width / 2, r.y + r.height / 2));
    })).toBe(true);
}

test.use({hasTouch: true});
for (const width of [320, 390]) {
    for (const language of ['en', 'fr']) {
        test(`emergency header controls fit and operate at ${width}px ${language}`, async ({page}, info) => {
            await page.setViewportSize({width, height: 844});
            await mockPage(page, true, language);
            await page.goto('/#/mowglinext', {waitUntil: 'domcontentloaded'});
            const header = page.locator('header').first();
            const rearm = header.getByRole('button', {name: language === 'fr' ? /Réarmer$/ : /Re-arm$/});
            const power = header.getByRole('button', {name: language === 'fr' ? 'Menu batterie et alimentation' : 'Battery and power menu', exact: true});
            await expect(rearm).toBeVisible({timeout: 15000});
            await reachable(rearm);
            await reachable(power);
            expect(await header.evaluate(el => el.scrollWidth <= el.clientWidth)).toBe(true);
            // Exercise the existing power menu without requesting a host action.
            await power.tap();
            await expect(page.getByRole('menu')).toBeVisible();
            await power.tap();
            await expect(page.getByRole('menu')).not.toBeVisible();
            await page.screenshot({path: info.outputPath('header.png'), animations: 'disabled'});
            await page.setViewportSize({width: 844, height: 390});
            await reachable(page.getByRole('button', {name: /Battery and power menu|Menu batterie et alimentation/}));
            await page.setViewportSize({width, height: 844});
            await reachable(page.locator('header').first().getByRole('button', {name: /Battery and power menu|Menu batterie et alimentation/}));
        });
    }
}

for (const viewport of [{width: 740, height: 360}, {width: 320, height: 568}, {width: 1440, height: 900}]) {
    test(`logs toolbar and useful log viewport recover at ${viewport.width}x${viewport.height}`, async ({page}, info) => {
        await page.setViewportSize(viewport);
        await mockPage(page);
        await page.goto('/#/logs', {waitUntil: 'domcontentloaded'});
        await expect(page.getByTestId('log-line')).toHaveCount(80);
        const clear = page.getByRole('button', {name: 'Clear', exact: true});
        const live = page.getByRole('button', {name: /↓ Live/});
        const zone = page.getByRole('button', {name: /Switch.*UTC|Toggle.*time|time zone/i});
        await reachable(clear);
        await reachable(live);
        await live.tap();
        await expect(page.getByRole('button', {name: /↓ Paused/})).toBeVisible();
        await reachable(zone);
        await zone.tap();
        await expect(zone).toContainText('UTC');
        const lines = page.getByTestId('log-line').first().locator('..');
        await lines.scrollIntoViewIfNeeded();
        const logHeight = (await lines.boundingBox())!.height;
        expect(logHeight).toBeGreaterThanOrEqual(150);
        expect(logHeight).toBeLessThanOrEqual(viewport.height);
        expect(await lines.evaluate(el => el.scrollHeight > el.clientHeight)).toBe(true);
        const last = page.getByTestId('log-line').last();
        await reachable(last);
        await page.screenshot({path: info.outputPath('logs.png'), animations: 'disabled'});
        await reachable(clear);
        await clear.tap();
        await expect(page.getByTestId('log-line')).toHaveCount(0);
        await expect(page.getByText('Waiting for log output...')).toBeVisible();
        if (viewport.width === 740) {
            await page.setViewportSize({width: 390, height: 844});
            await reachable(clear);
            await page.setViewportSize(viewport);
            await reachable(clear);
        }
    });
}
