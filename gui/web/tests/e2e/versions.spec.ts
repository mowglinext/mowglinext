import {test, expect} from '@playwright/test';
import {SCENARIOS} from './mock/scenarios';
import {installMockBackend} from './mock/mockBackend';

const inventory = {
    observed_at: '2026-09-05T16:00:00Z', docker_available: true,
    server: {revision: '15519db3', version: 'dev', modified: false},
    components: [
        {component: 'robot', name: 'mowgli-ros2', state: 'running', image: 'ghcr.io/mowglinext/mowglinext/mowgli-ros2:dev', image_id: 'sha256:running-robot', digests: ['ghcr.io/mowglinext/mowglinext/mowgli-ros2@sha256:manifest'], revision: '5cb07fab', version: 'dev', metadata_available: true},
        {component: 'gui', name: 'mowgli-gui', state: 'running', image: 'ghcr.io/wjcloudy/mowglinext/mowglinext-gui:feat-settings-updates', image_id: 'sha256:running-gui', digests: [], revision: '15519db3', version: 'feat-settings-updates', metadata_available: true},
    ],
};

for (const mobile of [false, true]) {
    test(`version shortcuts and read-only inventory on ${mobile ? 'mobile' : 'desktop'}`, async ({page}, testInfo) => {
        await page.setViewportSize(mobile ? {width: 390, height: 844} : {width: 1440, height: 1000});
        const errors: string[] = [];
        page.on('pageerror', error => errors.push(error.message));
        await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/versions': inventory, '/api/settings/yaml': {mower_model: 'YardForce500'}}, topics: {...SCENARIOS[0].topics, status: {...SCENARIOS[0].topics?.status as object, stamp: {sec: 1, nanosec: 0}, firmware_version: '1.8.213', firmware_protocol_version: 6, firmware_compatible: true}}}, {liveStatusIntervalMs: 200});
        const forbidden: string[] = [];
        page.on('request', req => { if (/\/api\/(containers|mowglinext\/call|system\/(reboot|shutdown))/.test(req.url()) && req.method() !== 'GET') forbidden.push(req.url()); });
        await page.goto('/#/mowglinext');
        await expect(page.getByText('v1.8.213', {exact: true})).toBeVisible();
        if (mobile) {
            await page.getByRole('button', {name: 'More', exact: true}).click();
            const more = page.getByRole('button', {name: 'Close', exact: true}).locator('../..');
            const shortcut = more.getByTestId('running-version-summary');
            await expect(shortcut).toHaveCount(1);
            await expect(shortcut).toBeVisible();
            await shortcut.click();
            await expect(more).toBeHidden();
        } else {
            const shortcut = page.locator('aside').getByTestId('running-version-summary');
            await expect(shortcut).toHaveCount(1);
            await expect(shortcut).toBeVisible();
            await shortcut.click();
        }
        await expect(page).toHaveURL(/section=updates/);
        const panel = page.getByTestId('installed-versions');
        await panel.locator('.ant-segmented').getByText('Advanced', {exact: true}).click();
        const details = panel.locator('details.update-diagnostics');
        await expect(details).toHaveCount(1);
        await details.locator(':scope > summary').click();
        await expect(details).toHaveAttribute('open', '');
        await expect(panel.getByText('dev · 5cb07fab', {exact: true})).toBeVisible();
        await expect(panel.getByText('Protocol compatible', {exact: true})).toBeVisible();
        await expect(panel.getByText('1.8.213', {exact: true})).toBeVisible();
        await panel.getByTestId('version-robot').locator('summary').click();
        await expect(panel.getByText('sha256:running-robot', {exact: true})).toBeVisible();
        await expect(panel.getByText('ghcr.io/mowglinext/mowglinext/mowgli-ros2@sha256:manifest', {exact: true})).toBeVisible();
        const refreshed = page.waitForResponse(response => response.url().endsWith('/api/system/versions') && response.request().method() === 'GET');
        await panel.getByRole('button', {name: 'Refresh versions'}).click();
        expect((await refreshed).ok()).toBe(true);
        await expect(panel.getByRole('button', {name: 'Refresh versions'})).toBeEnabled();
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        await page.screenshot({path: testInfo.outputPath(`versions-${mobile ? 'mobile' : 'desktop'}.png`), fullPage: true});
        expect(errors).toEqual([]);
        expect(forbidden).toEqual([]);
    });
}

test('Docker unavailable and a cached firmware sample never claim live compatibility', async ({page}, testInfo) => {
    const errors: string[] = [];
    const forbidden: string[] = [];
    page.on('pageerror', error => errors.push(error.message));
    page.on('request', req => { if (/\/api\/(containers|mowglinext\/call|system\/(reboot|shutdown))/.test(req.url()) && req.method() !== 'GET') forbidden.push(req.url()); });
    await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/versions': {...inventory, docker_available: false, components: []}}, topics: {status: {stamp: {sec: 1, nanosec: 0}, firmware_version: '1.8.213', firmware_protocol_version: 6, firmware_compatible: true}}});
    await page.goto('/#/settings?section=updates');
    const panel = page.getByTestId('installed-versions');
    await panel.locator('.ant-segmented').getByText('Advanced', {exact: true}).click();
    const details = panel.locator('details.update-diagnostics');
    await expect(details).toHaveCount(1);
    await details.locator(':scope > summary').click();
    await expect(details).toHaveAttribute('open', '');
    await expect(panel.getByText(/Docker is unavailable/)).toBeVisible();
    await expect(panel.getByText('Waiting for live status', {exact: true})).toBeVisible();
    await expect(panel.getByText('Protocol compatible', {exact: true})).toHaveCount(0);
    await page.route('**/api/system/versions', route => route.fulfill({status: 503, body: '{}'}));
    const refreshed = page.waitForResponse(response => response.url().endsWith('/api/system/versions') && response.request().method() === 'GET');
    await panel.getByRole('button', {name: 'Refresh versions'}).click();
    expect((await refreshed).status()).toBe(503);
    await expect(panel.getByRole('button', {name: 'Refresh versions'})).toBeEnabled();
    await expect(panel.getByText('Version information could not be refreshed')).toBeVisible();
    await expect(panel.getByText('Waiting for live status', {exact: true})).toBeVisible();
    await expect(panel.getByText('Protocol compatible', {exact: true})).toHaveCount(0);
    expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
    await page.screenshot({path: testInfo.outputPath('versions-docker-unavailable.png'), fullPage: true});
    expect(errors).toEqual([]);
    expect(forbidden).toEqual([]);
});
