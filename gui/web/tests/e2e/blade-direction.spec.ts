import {test, expect} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {pack} from 'msgpackr';

for (const mobile of [false, true]) {
    test.describe(mobile ? 'mobile requested direction' : 'desktop requested direction', () => {
        test.use({viewport: mobile ? {width: 390, height: 844} : {width: 1440, height: 900}});
        for (const route of ['diagnostics', 'mowglinext']) {
            test(`requested blade direction on ${route}`, async ({page}) => {
                await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
                const status = {
                    mower_motor_rpm: 3100, mower_esc_current: 0.6,
                    mower_motor_temperature: 25, mower_esc_temperature: 24,
                    mower_esc_status: 1, mow_enabled: true,
                    blade_requested_direction: 'reverse',
                    firmware_compatible: true, firmware_version: '1.9.205',
                };
                await installMockBackend(page, {name: 'blade-reverse'});
                let stopUpdates = () => {};
                await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, (ws) => {
                    const timer = setInterval(() => ws.send(Buffer.from(pack({topic: 'status', data: status}))), 250);
                    stopUpdates = () => clearInterval(timer);
                    ws.onClose(() => { clearInterval(timer); ws.close(); });
                });
                await page.goto(`/#/${route}`);
                if (route === 'diagnostics') {
                    if (mobile) await page.getByRole('button', {name: /Sensors/}).click();
                    else await page.getByRole('tab', {name: /Robot/}).click();
                }
                const display = page.getByLabel('Requested direction: Reverse', {exact: true});
                await expect(display).toBeVisible();
                await display.scrollIntoViewIfNeeded();
                if (route === 'mowglinext') await display.click();
                await expect(page.getByText(/Older firmware may still run forward/)).toBeVisible();
                if (route === 'mowglinext') {
                    await expect(page.getByRole('tooltip')).toBeInViewport();
                    await expect(page.locator('.ant-popover')).not.toHaveClass(/ant-zoom-big-(appear|enter)/);
                }
                await page.screenshot({path: `tests/e2e/.artifacts/blade-direction-${route}-${mobile ? 'mobile' : 'desktop'}.png`});
                stopUpdates();
                await expect(page.getByLabel('Requested direction: Unknown', {exact: true})).toBeVisible({timeout: 7000});
            });
        }
    });
}
