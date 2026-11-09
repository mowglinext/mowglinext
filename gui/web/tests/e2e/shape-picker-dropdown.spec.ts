import {expect, test, type Locator, type Page, type TestInfo} from '@playwright/test';
import {readFileSync} from 'node:fs';
import type {Choice} from './fixtures/shape-picker-dropdown';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};

const configurations = [
    {key: 'en-desktop', lang: 'en', mobile: false, edge: false},
    {key: 'fr-desktop', lang: 'fr', mobile: false, edge: false},
    {key: 'en-mobile', lang: 'en', mobile: true, edge: false},
    {key: 'fr-mobile', lang: 'fr', mobile: true, edge: false},
    {key: 'en-edge', lang: 'en', mobile: false, edge: true},
] as const;
const deprecatedDropdown = /\[antd: Dropdown\].*`?dropdownRender`? is deprecated/;

function observe(page: Page) {
    const consoles: {type: string; text: string}[] = [];
    const errors: string[] = [];
    const writes: string[] = [];
    const reads: string[] = [];
    page.on('console', message => consoles.push({type: message.type(), text: message.text()}));
    page.on('pageerror', error => errors.push(error.message));
    page.on('request', request => {
        const path = new URL(request.url()).pathname;
        // These existing POST endpoints read data; retain them in the evidence.
        if (path === '/api/config/keys/get' || path === '/api/mowglinext/call/get_lidar_ignore_corridors') {
            reads.push(path);
            return;
        }
        if (path.startsWith('/api/') && request.method() !== 'GET') writes.push(path);
    });
    return {consoles, errors, writes, reads};
}

async function stable(popup: Locator) {
    await expect(popup).toBeVisible();
    await expect(popup).not.toHaveClass(/ant-slide-(up|down)-(enter|appear|leave)/);
}

async function animationsFinished(page: Page) {
    await expect(page.locator('.ant-wave')).toHaveCount(0);
    await page.evaluate(async () => {
        await document.fonts.ready;
        await Promise.all(document.getAnimations()
            .filter(animation => animation.effect?.getTiming().iterations !== Infinity)
            .map(animation => animation.finished.catch(() => {})));
    });
}

async function measure(popup: Locator) {
    return popup.evaluate(node => {
        const content = node.firstElementChild!;
        const rect = node.getBoundingClientRect();
        const signature = (element: Element) => {
            const css = getComputedStyle(element);
            return Object.fromEntries(['padding', 'margin', 'gap', 'border', 'border-radius', 'color',
                'background-color', 'box-shadow', 'font', 'min-width', 'max-height', 'overflow-y']
                .map(key => [key, css.getPropertyValue(key)]));
        };
        return {rect: {x: rect.x, y: rect.y, width: rect.width, height: rect.height},
            placement: [...node.classList].find(name => name.startsWith('ant-dropdown-placement-')),
            content: {styles: signature(content), scrollHeight: content.scrollHeight,
                clientHeight: content.clientHeight, scrollTop: content.scrollTop},
            buttons: [...content.querySelectorAll('button')].map(button => ({text: button.textContent,
                title: button.title, styles: signature(button)}))};
    });
}

async function capture(page: Page, popup: Locator, info: TestInfo, key: string) {
    await stable(popup);
    await animationsFinished(page);
    const measurement = await measure(popup);
    await page.screenshot({path: info.outputPath('open.png')});
    await popup.screenshot({path: info.outputPath('popup.png')});
    await info.attach('measurement', {body: JSON.stringify(measurement, null, 2), contentType: 'application/json'});
    if (process.env.DROPDOWN_BASELINE_DIR) {
        const before = JSON.parse(readFileSync(`${process.env.DROPDOWN_BASELINE_DIR}/${key}.json`, 'utf8')) as typeof measurement;
        expect(measurement).toEqual(before);
    }
    return measurement;
}

async function finish(info: TestInfo, observed: ReturnType<typeof observe>, details: unknown, expectedWrites: string[] = []) {
    await info.attach('observations', {body: JSON.stringify({...observed, details}, null, 2), contentType: 'application/json'});
    expect(observed.errors).toEqual([]);
    expect(observed.writes).toEqual(expectedWrites);
    // Last so RED retains all nominal interaction evidence before the warning assertion fails.
    expect(observed.consoles.filter(message => deprecatedDropdown.test(message.text))).toEqual([]);
}

for (const configuration of configurations) {
    test.describe(configuration.key, () => {
        test.use({viewport: configuration.mobile ? {width: 390, height: 320} : {width: 1280, height: 900},
            isMobile: configuration.mobile, hasTouch: configuration.mobile});
        test('real shape picker keeps its rendering and interactions', async ({page}, info) => {
            const observed = observe(page);
            const locale = configuration.lang === 'fr' ? fr : en;
            await page.goto(`/tests/e2e/fixtures/shape-picker-dropdown.html?lang=${configuration.lang}${configuration.mobile ? '&mobile=1' : ''}${configuration.edge ? '&edge=1' : ''}`);
            const trigger = page.getByRole('button', {name: configuration.mobile ? 'Mobile add' : new RegExp(`${locale.mapShapePicker.add}$`)});
            const popup = page.locator('.ant-dropdown').filter({visible: true});
            const activate = async (target: Locator) => configuration.mobile ? target.tap() : target.click();
            const open = async () => {await activate(trigger); await stable(popup);};
            await expect(trigger).toHaveCount(1);
            await expect(trigger).toBeVisible();
            await page.screenshot({path: info.outputPath('closed.png')});
            await open();
            const initial = await capture(page, popup, info, configuration.key);
            if (configuration.edge) expect(initial.placement).toBe('ant-dropdown-placement-bottomLeft');
            await activate(page.locator('#outside'));
            await expect(popup).toBeHidden();

            const focus = () => page.evaluate(() => ({tag: document.activeElement?.tagName,
                text: document.activeElement?.textContent, label: document.activeElement?.getAttribute('aria-label')}));
            const keyboard = [];
            for (const key of ['Enter', 'Space']) {
                await trigger.focus();
                const before = await focus();
                await trigger.press(key);
                await stable(popup);
                await page.keyboard.press('Escape');
                await expect(popup).toBeHidden();
                await expect(trigger).toBeFocused();
                const after = await focus();
                expect(after).toEqual(before);
                keyboard.push({key, before, after});
            }
            const expected: Choice[] = [];
            const choices = () => page.evaluate(() => window.shapeChoices);
            expect(await choices()).toEqual(expected);
            await open();
            const input = popup.getByRole('spinbutton');
            await input.fill('7.5');
            await input.press('Enter');
            await stable(popup);
            await activate(page.locator('#outside'));
            await expect(popup).toBeHidden();
            for (const shape of ['square', 'circle', 'hexagon'] as const) {
                await open();
                await expect(popup.getByRole('spinbutton')).toHaveValue('7.5');
                const option = popup.getByRole('button', {name: new RegExp(`${locale.mapShapePicker[shape]}$`)});
                if (shape === 'square') {await option.focus(); await option.press('Enter');}
                else await activate(option);
                await expect(popup).toBeHidden();
                expected.push({kind: 'shape', shape, size: 7.5});
                expect(await choices()).toEqual(expected);
            }
            await open();
            const content = popup.locator('div[style*="overflow-y: auto"]');
            let scroll: {top: number; height: number; client: number} | undefined;
            if (configuration.mobile) {
                await content.hover();
                await page.mouse.wheel(0, -400);
                await expect.poll(() => content.evaluate(node => node.scrollTop)).toBe(0);
                await page.mouse.wheel(0, 400);
                await expect.poll(() => content.evaluate(node => node.scrollTop)).toBeGreaterThan(0);
                scroll = await content.evaluate(node => ({top: node.scrollTop, height: node.scrollHeight, client: node.clientHeight}));
                expect(scroll.height).toBeGreaterThan(scroll.client);
                await animationsFinished(page);
                await page.screenshot({path: info.outputPath('scrolled.png')});
            }
            await activate(popup.getByRole('button', {name: '🎯', exact: true}));
            await expect(popup).toBeHidden();
            expected.push({kind: 'emoji', emoji: '🎯', size: 7.5});
            expect(await choices()).toEqual(expected);
            await open();
            await activate(popup.getByRole('button', {name: new RegExp(`${locale.mapLidarCorridors.draw}$`)}));
            await expect(popup).toBeHidden();
            expected.push({kind: 'corridor'});
            expect(await choices()).toEqual(expected);
            await finish(info, observed, {keyboard, scroll, choices: await choices()});
        });
    });
}

for (const mobile of [false, true]) {
    test.describe(`actual MapPage ${mobile ? 'mobile' : 'desktop'}`, () => {
        test.use({viewport: mobile ? {width: 390, height: 844} : {width: 1280, height: 900}, isMobile: mobile, hasTouch: mobile});
        test('shape picker in edit mode preserves theme and local actions', async ({page}, info) => {
            const observed = observe(page);
            await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
            await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}}});
            await page.goto('/#/map');
            const activate = async (target: Locator) => mobile ? target.tap() : target.click();
            const main = page.getByRole('main');
            await activate(main.getByRole('button', {name: new RegExp(`${en.mapToolbar.editMap}$`)}));
            const trigger = main.getByRole('button', {name: mobile ? en.mapToolbarMobile.addShape : en.mapEditorToolbar.addShape, exact: true});
            const popup = page.locator('.ant-dropdown').filter({visible: true});
            await expect(trigger).toBeVisible();
            // MapPage requires this existing backup before opening its editor.
            expect(observed.writes).toEqual(['/api/map-backups']);
            await page.screenshot({path: info.outputPath('closed.png')});
            await activate(trigger);
            await capture(page, popup, info, mobile ? 'map-mobile' : 'map-desktop');
            await activate(popup.getByRole('button', {name: new RegExp(`${en.mapShapePicker.square}$`)}));
            await expect(popup).toBeHidden();
            const dialog = page.getByRole('dialog');
            await expect(dialog).toBeVisible();
            await activate(dialog.getByRole('button', {name: 'Cancel', exact: true}));
            await expect(dialog).toBeHidden();
            await activate(trigger);
            await stable(popup);
            await activate(popup.getByRole('button', {name: '🎯', exact: true}));
            await expect(popup).toBeHidden();
            await expect(dialog).toBeVisible();
            await activate(dialog.getByRole('button', {name: 'Cancel', exact: true}));
            await expect(dialog).toBeHidden();
            await activate(trigger);
            await stable(popup);
            await activate(popup.getByRole('button', {name: new RegExp(`${en.mapLidarCorridors.draw}$`)}));
            await expect(popup).toBeHidden();
            // Entering a draw mode must not save or issue a remote command by itself.
            await finish(info, observed, {mobile, selected: ['square', '🎯', 'lidar corridor']}, ['/api/map-backups']);
        });
    });
}
