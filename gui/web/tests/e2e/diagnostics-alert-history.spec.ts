import {expect, test, type WebSocketRoute} from '@playwright/test';
import {pack} from 'msgpackr';
import {healthy} from '../../src/test/diagnosticHealthFixture';
import {isJsonObject} from '../../src/utils/json';
import {installMockBackend} from './mock/mockBackend';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};

for (const language of ['en', 'fr']) {
    for (const width of [390, 1440]) {
        test(`alert consultation survives WARN OK WARN in ${language} at ${width}px`, async ({page}) => {
            await page.setViewportSize({width, height: 1000});
            await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
            const input = healthy(), snapshot = input.snapshot;
            if (!snapshot) throw new Error('Missing fixture');
            snapshot.timestamp = new Date().toISOString();
            await installMockBackend(page, {name: 'alert-history', rest: {'/api/settings/yaml': {lidar_enabled: false}}});
            let socket: WebSocketRoute | undefined;
            let sequence = 0;
            let connections = 0;
            let diagnosticSubscriptions = 0;
            let allowDelivery = true;
            const diagnostic = (level: number, message: string) => ({header: {stamp: {sec: ++sequence, nanosec: 0}},
                status: [{name: 'Stable fixture/detail', hardware_id: 'fictional-source', level, message,
                    values: [{key: 'evidence', value: 'fictional-value'}]}]});
            await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, ws => {
                socket = ws; connections++;
                const topics: Record<string, unknown> = {highLevelStatus: {state: 1, battery_percent: 80},
                    emergency: {active_emergency: false, latched_emergency: false}, power: {v_battery: 28},
                    gnssStatus: input.gnss, diagnostics: diagnostic(1, 'Fictional initial warning')};
                ws.onMessage(message => {
                    const request: unknown = JSON.parse(typeof message === 'string' ? message : message.toString());
                    if (!isJsonObject(request) || request.op !== 'subscribe' || typeof request.topic !== 'string') return;
                    if (request.topic === 'diagnostics') diagnosticSubscriptions++;
                    const data = topics[request.topic];
                    if (allowDelivery && data !== undefined) ws.send(pack({topic: request.topic, data}));
                });
            });
            await page.route('**/api/diagnostics/snapshot', route => route.fulfill({json: snapshot}));
            const writes: string[] = [];
            page.on('request', request => {
                if (['POST', 'PUT', 'DELETE'].includes(request.method()) && new URL(request.url()).pathname !== '/api/config/keys/get') writes.push(request.url());
            });
            await page.goto('/#/diagnostics');
            const locale = language === 'fr' ? fr : en;
            const panel = page.locator('.ant-card').filter({has: page.getByText(locale.diagnosticsPage.alerts, {exact: true})});
            const group = panel.getByRole('button', {name: /Stable fixture/});
            await expect(group).toBeVisible();
            await group.click();
            await expect(panel.getByText('Fictional initial warning', {exact: true})).toBeVisible();
            await page.evaluate(() => document.fonts.ready);
            await panel.locator('.ant-collapse-content-active').evaluate(async element => {
                await new Promise<void>(resolve => requestAnimationFrame(() => resolve()));
                await new Promise<void>(resolve => requestAnimationFrame(() => resolve()));
                await Promise.all(element.getAnimations({subtree: true}).map(animation => animation.finished));
            });
            // Measure final content height, not the opening Collapse animation.
            await expect(panel.locator('[class*="ant-motion-collapse"]')).toHaveCount(0);
            await group.focus();
            const before = await panel.boundingBox();
            const beforeCause = await page.getByTestId('diagnostic-health-cause').boundingBox();
            if (!before) throw new Error('Missing panel bounds');
            if (!socket) throw new Error('Missing socket');
            socket.send(pack({topic: 'diagnostics', data: diagnostic(0, 'OK')}));
            const resolved = language === 'fr' ? 'Résolue' : 'Resolved';
            await expect(panel.getByText(resolved, {exact: true}).first()).toBeVisible();
            await expect(panel.getByText('Fictional initial warning', {exact: true})).toBeVisible();
            await expect(group).toHaveAttribute('aria-expanded', 'true');
            await expect(group).toBeFocused();
            await expect(page.getByTestId('diagnostic-health-verdict')).toHaveText(locale.diagnosticsPage.verdictAllGood);
            const after = await panel.boundingBox();
            if (!after) throw new Error('Missing retained panel');
            const afterCause = await page.getByTestId('diagnostic-health-cause').boundingBox();
            expect(Math.abs(before.height - after.height), JSON.stringify({before, after, beforeCause, afterCause})).toBeLessThan(2);
            expect(Math.abs(before.y - after.y), JSON.stringify({before, after, beforeCause, afterCause})).toBeLessThan(2);
            socket.send(pack({topic: 'diagnostics', data: diagnostic(1, 'Fictional repeated warning')}));
            await expect(panel.getByText('Fictional repeated warning', {exact: true})).toBeVisible();
            await expect(group).toHaveAttribute('aria-expanded', 'true');
            await expect(page.getByTestId('diagnostic-health-verdict')).toHaveText(locale.diagnosticsPage.verdictAttention);
            socket.send(pack({topic: 'diagnostics', data: diagnostic(2, 'Fictional confirmed error')}));
            await expect(panel.getByTestId('diagnostic-alert-episode')).toHaveClass(/ant-alert-error/);
            await expect(group).toHaveAttribute('aria-expanded', 'true');
            expect(await page.evaluate(() => document.documentElement.scrollWidth)).toBeLessThanOrEqual(width);
            // Hold reconnect delivery so an open connection alone cannot resolve.
            allowDelivery = false;
            socket.close({code: 1001, reason: 'Fictional lost transport'});
            await expect(panel.getByTestId('diagnostic-alert-episode')).toHaveAttribute('data-state', 'unverified');
            await expect.poll(() => connections).toBeGreaterThan(1);
            await expect.poll(() => diagnosticSubscriptions).toBeGreaterThan(1);
            await expect(group).toHaveAttribute('aria-expanded', 'true');
            if (!socket) throw new Error('Missing reconnected socket');
            socket.send(pack({topic: 'diagnostics', data: diagnostic(0, 'OK')}));
            await expect(panel.getByTestId('diagnostic-alert-episode')).toHaveAttribute('data-state', 'resolved');
            await expect(panel.getByTestId('diagnostic-alert-message')).toHaveText('Fictional confirmed error');
            expect(writes).toEqual([]);
        });
    }
    test(`bounded recent history and open-group protection in ${language}`, async ({page}) => {
        await page.setViewportSize({width: 390, height: 900});
        await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
        await page.goto('/tests/e2e/fixtures/diagnostic-alert-history.html');
        const panel = page.getByTestId('diagnostic-alert-panel');
        await expect(page.getByTestId('diagnostic-alert-empty')).toBeVisible();
        await page.getByRole('button', {name: 'Begin fictional burst'}).click();
        await page.getByRole('button', {name: 'Resolve burst'}).click();
        await expect(panel.getByTestId('diagnostic-alert-group')).toHaveAttribute('data-state', 'resolved');
        await page.getByRole('button', {name: 'Advance clock by 31 seconds'}).click();
        await expect(page.getByTestId('diagnostic-alert-empty')).toBeVisible();
        const locale = language === 'fr' ? fr : en;
        await panel.getByRole('button', {name: locale.diagnosticAlertHistory.showRecent.replace('{{count}}', '100')}).click();
        const group = panel.getByRole('button', {name: /Burst/});
        await group.press('Enter');
        await expect(panel.getByTestId('diagnostic-alert-episode')).toHaveCount(100);
        await page.getByRole('button', {name: 'Advance clock by 6 minutes'}).click();
        await expect(group).toHaveAttribute('aria-expanded', 'true');
        await expect(panel.getByTestId('diagnostic-alert-episode')).toHaveCount(100);
        // Responsive sizing preserves the panel instance in this controlled fixture.
        await page.setViewportSize({width: 1440, height: 1000});
        await expect(group).toHaveAttribute('aria-expanded', 'true');
        await expect(panel.getByTestId('diagnostic-alert-episode')).toHaveCount(100);
        await panel.getByRole('button', {name: locale.diagnosticAlertHistory.collapseAll}).click();
        await expect(page.getByTestId('diagnostic-alert-empty')).toBeVisible();
        await expect(panel.getByRole('button', {name: locale.diagnosticAlertHistory.hideRecent})).toHaveText(locale.diagnosticAlertHistory.recent.replace('{{count}}', '0'));
    });

}
