import {expect, test, type Page, type TestInfo} from '@playwright/test';
import {readFileSync} from 'node:fs';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};

// AntD formats property names with backticks in the actual console message.
const deprecatedTooltip = /\[antd: Tooltip\].*`?overlayStyle`? is deprecated/;

async function observe(page: Page, info: TestInfo, key: string, texts: typeof en.mapLidarCorridors, editable: boolean) {
    const consoleMessages: {type: string; text: string}[] = [];
    const errors: string[] = [];
    page.on('console', message => consoleMessages.push({type: message.type(), text: message.text()}));
    page.on('pageerror', error => errors.push(error.message));
    const capture = async (trigger: ReturnType<Page['locator']>) => {
        await expect(trigger).toHaveCount(1);
        await expect(trigger).toBeVisible();
        await page.evaluate(() => document.fonts.ready);
        await page.screenshot({path: info.outputPath('closed.png')});
        await trigger.hover();
        const tip = page.getByRole('tooltip');
        await expect(tip).toBeVisible();
        const popup = page.locator('.ant-tooltip');
        await expect(popup).not.toHaveClass(/ant-zoom-big-fast-(enter|appear)/);
        await expect(popup).toHaveCSS('max-width', '320px');
        await expect(tip.getByText(texts.hint, {exact: true})).toBeVisible();
        await expect(tip.getByText(texts.safetyWarning, {exact: true})).toBeVisible();
        await expect(tip.getByText(texts.lockedHint, {exact: true})).toHaveCount(editable ? 0 : 1);
        await expect(popup.locator('.ant-tooltip-arrow')).toBeVisible();
        const measurement = await popup.evaluate(node => {
            const rect = node.getBoundingClientRect();
            const css = getComputedStyle(node);
            const inner = node.querySelector('.ant-tooltip-inner')!;
            const safety = inner.firstElementChild!.children[1];
            return {
                rect: {x: rect.x, y: rect.y, width: rect.width, height: rect.height},
                maxWidth: css.maxWidth, text: inner.textContent,
                safetyText: safety.textContent, safetyColor: getComputedStyle(safety).color,
                font: getComputedStyle(inner).font,
                placement: [...node.classList].find(name => name.startsWith('ant-tooltip-placement-')),
                arrow: getComputedStyle(node.querySelector('.ant-tooltip-arrow')!).width,
            };
        });
        await page.screenshot({path: info.outputPath('open.png')});
        await popup.screenshot({path: info.outputPath('tooltip.png')});
        await info.attach('measurement', {body: JSON.stringify(measurement, null, 2), contentType: 'application/json'});
        // Optional external baseline produced by the RED run; no snapshots in the repository.
        if (process.env.TOOLTIP_BASELINE_DIR) {
            const baseline = JSON.parse(readFileSync(`${process.env.TOOLTIP_BASELINE_DIR}/${key}.json`, 'utf8')) as typeof measurement;
            expect(measurement).toEqual(baseline);
        }
        await page.mouse.move(0, page.viewportSize()!.height - 1);
        await expect(tip).toBeHidden();
        await trigger.hover();
        await expect(tip).toBeVisible();
        await page.mouse.move(0, page.viewportSize()!.height - 1);
        await expect(tip).toBeHidden();
        await info.attach('console', {body: JSON.stringify(consoleMessages, null, 2), contentType: 'application/json'});
        expect(errors).toEqual([]);
        // Keep last: RED must preserve all rendering evidence before this assertion fails.
        expect(consoleMessages.filter(message => deprecatedTooltip.test(message.text))).toEqual([]);
        return measurement;
    };
    return {capture};
}

for (const lang of ['en', 'fr'] as const) {
    for (const editable of [false, true]) {
        test(`real LidarCorridorsInfo ${lang} editable=${editable}`, async ({page}, info) => {
            const {capture} = await observe(page, info, `${lang}-${editable}`, lang === 'en' ? en.mapLidarCorridors : fr.mapLidarCorridors, editable);
            await page.goto(`/tests/e2e/fixtures/lidar-corridors-tooltip.html?lang=${lang}&editable=${editable}`);
            await capture(page.locator('.anticon-info-circle'));
        });
    }
}

test('real LidarCorridorsInfo shifts and flips near the viewport edge', async ({page}, info) => {
    const {capture} = await observe(page, info, 'edge', en.mapLidarCorridors, false);
    await page.goto('/tests/e2e/fixtures/lidar-corridors-tooltip.html?lang=en&edge=1');
    const measurement = await capture(page.locator('.anticon-info-circle'));
    expect(measurement.placement).toBe('ant-tooltip-placement-bottom');
    const trigger = await page.locator('.anticon-info-circle').boundingBox();
    // Preserve AntD's native shift/arrow constraints, rather than require a new clipping policy.
    expect(measurement.rect.x).toBeGreaterThan(trigger!.x + trigger!.width / 2 - measurement.rect.width / 2);
    expect(measurement.rect.y).toBeGreaterThan(trigger!.y + trigger!.height);
    expect(measurement.rect.y).toBeGreaterThanOrEqual(0);
    expect(measurement.rect.x + measurement.rect.width).toBeLessThanOrEqual(page.viewportSize()!.width);
    expect(measurement.rect.y + measurement.rect.height).toBeLessThanOrEqual(page.viewportSize()!.height);
});

test('actual MapPage Tooltip uses the application theme without commands', async ({page}, info) => {
    const {capture} = await observe(page, info, 'map', en.mapLidarCorridors, false);
    await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
    const base = SCENARIOS.find(scenario => scenario.name === 'idle-docked-full')!;
    await installMockBackend(page, {...base, rest: {...base.rest, '/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}}});
    const commands: string[] = [];
    await page.route(/\/api\/mowglinext\/call\//, async route => {
        const name = new URL(route.request().url()).pathname.split('/').pop()!;
        commands.push(name);
        await route.fulfill({json: {}});
    });
    await page.goto('/#/map');
    const header = page.locator('#map-sidebar-header-lidar');
    const expanded = await header.getAttribute('aria-expanded');
    await capture(header.locator('..').locator('.anticon-info-circle'));
    await expect(header).toHaveAttribute('aria-expanded', expanded!);
    expect(commands.filter(name => name !== 'get_lidar_ignore_corridors')).toEqual([]);
});
