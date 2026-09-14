import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';

for (const viewport of [{width: 390, height: 844}, {width: 375, height: 667}, {width: 740, height: 360}]) {
    test.describe(`mobile diagnostics ${viewport.width}x${viewport.height}`, () => {
        test.use({viewport, hasTouch: true});

        test('last panels can be expanded and collapsed above navigation', async ({page}, testInfo) => {
            await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
            await installMockBackend(page, {name: 'mobile-diagnostics', silentSocket: true});
            await page.goto('/#/diagnostics');
            const main = page.getByRole('main');
            const nav = page.getByRole('navigation');
            const panels = ['Rosbag Recording', 'ROS Diagnostics'];
            await expect(page.getByRole('button', {name: /ROS Diagnostics$/})).toBeVisible();

            for (const name of panels) {
                const header = page.getByRole('button', {name: new RegExp(`${name}$`)});
                await main.evaluate(element => { element.scrollTop = element.scrollHeight; });
                // Visibility alone is insufficient: the fixed navigation can
                // intercept a visible accordion header's touch target.
                await expect.poll(async () => {
                    const row = await header.boundingBox();
                    const bar = await nav.boundingBox();
                    return row && bar ? row.y + row.height <= bar.y : false;
                }).toBe(true);
                await header.tap();
                await expect(header).toHaveAttribute('aria-expanded', 'true');
                await header.scrollIntoViewIfNeeded();
                await header.tap();
                await expect(header).toHaveAttribute('aria-expanded', 'false');
            }

            await expect(main.getByText('No diagnostic messages received.', {exact: true})).toBeHidden();
            await main.evaluate(element => {
                element.scrollTop = element.scrollHeight;
                element.scrollLeft = 0;
            });
            await page.evaluate(() => window.scrollTo(0, 0));
            await page.screenshot({path: testInfo.outputPath('diagnostics-mobile.png'), animations: 'disabled'});
        });
    });
}
