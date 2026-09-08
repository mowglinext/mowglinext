import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';

for (const viewport of [{width: 390, height: 844}, {width: 375, height: 667}]) {
    test.describe(`mobile page clearance ${viewport.width}x${viewport.height}`, () => {
        test.use({viewport, hasTouch: true});

        for (const route of ['mowglinext', 'settings']) {
            test(`${route} can scroll its final content above navigation`, async ({page}, testInfo) => {
                await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
                await installMockBackend(page, {name: 'mobile-page-clearance', silentSocket: true});
                await page.goto(`/#/${route}`);
                const main = page.getByRole('main');
                if (route === 'settings') {
                    await page.getByPlaceholder('Search a setting…').fill('Appearance');
                    await page.getByRole('tab', {name: /Appearance/}).click();
                    await expect(page.getByRole('radio', {name: /^UTC/})).toBeVisible();
                } else {
                    await expect(main.getByText('nominal', {exact: true})).toBeVisible();
                    // Dashboard cards enter with staggered motion. Wait for
                    // the final card to finish fading before measuring it.
                    await expect.poll(() => main.getByText('nominal', {exact: true}).evaluate(element => {
                        let opacity = 1;
                        for (let node: HTMLElement | null = element; node; node = node.parentElement) {
                            opacity *= Number(getComputedStyle(node).opacity);
                        }
                        return opacity;
                    })).toBe(1);
                }
                await expect.poll(async () => {
                    await main.evaluate(element => { element.scrollTop = element.scrollHeight; });
                    // Check the actual page, not the possibly clipped outlet.
                    const content = await main.locator(':scope > div > div').boundingBox();
                    const nav = await page.getByRole('navigation').boundingBox();
                    return content && nav ? content.y + content.height <= nav.y : false;
                }).toBe(true);
                if (route === 'settings') {
                    const utc = page.getByRole('radio', {name: /^UTC/});
                    await utc.tap();
                    await expect(utc).toBeChecked();
                }
                await page.screenshot({path: testInfo.outputPath(`${route}-mobile.png`), animations: 'disabled'});
            });
        }
    });
}

for (const viewport of [{width: 390, height: 844}, {width: 1440, height: 900}]) {
    for (const route of ['map', 'logs']) {
        test(`${route} retains its viewport at ${viewport.width}x${viewport.height}`, async ({page}) => {
            await page.setViewportSize(viewport);
            await installMockBackend(page, {
                name: 'viewport-layout', silentSocket: true,
                rest: {'/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}},
            });
            await page.goto(`/#/${route}`);
            const main = page.getByRole('main');
            const content = route === 'map' ? page.locator('.mapboxgl-canvas') : main.locator(':scope > div > div');
            await expect(content).toBeVisible();
            await expect.poll(async () => (await content.boundingBox())?.height ?? 0).toBeGreaterThan(300);
            // Neither full-height page should become an unbounded document.
            await expect.poll(async () => (await content.boundingBox())?.height ?? Infinity).toBeLessThanOrEqual(viewport.height);
        });
    }
}
