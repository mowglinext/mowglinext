import {expect, test, type WebSocketRoute} from '@playwright/test';
import {pack} from 'msgpackr';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import {bridgeFixedWireFixture} from '../../src/test/gnssBridgeFixedFixture';
import {isJsonObject} from '../../src/utils/json';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};

for (const language of ['en', 'fr']) for (const width of [390, 1440]) {
    test(`bridge 3D RTK, quality and MAVROS Float in ${language} at ${width}px`, async ({page}) => {
        const locale = language === 'fr' ? fr : en;
        await page.setViewportSize({width, height: 1100});
        await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
        await installMockBackend(page, SCENARIOS[0]);
        let socket: WebSocketRoute | undefined;
        await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, ws => {
            socket = ws;
            ws.onMessage(message => {
                const request: unknown = JSON.parse(typeof message === 'string' ? message : message.toString());
                if (!isJsonObject(request) || request.op !== 'subscribe' || typeof request.topic !== 'string' || request.topic === 'gnssStatus') return;
                const data = SCENARIOS[0].topics?.[request.topic];
                if (data !== undefined) ws.send(pack({topic: request.topic, data}));
            });
        });
        const writes: string[] = [];
        page.on('request', request => {
            if (request.method() !== 'GET' && /^\/api\/(?:params|settings\/yaml|mowglinext\/(?:call|map|publish))/.test(new URL(request.url()).pathname)) writes.push(request.url());
        });
        await page.goto('/#/diagnostics');
        const banner = (label: string, percent: number) => page.getByText(`${percent}% · ${label}`, {exact: true}).first();
        if (width < 768) {
            await page.getByRole('button', {name: new RegExp(locale.diagnosticsPage.tabLocalization)}).click();
        } else await page.getByRole('tab', {name: new RegExp(locale.diagnosticsPage.tabLocalization)}).click();
        const fix = page.getByTestId('gnss-fix-status-value');
        const rtk = page.getByTestId('gnss-rtk-status-group');
        await expect(fix).toBeVisible();
        const send = (data: unknown) => {
            if (!socket) throw new Error('Missing mock socket');
            socket.send(pack({topic: 'gnssStatus', data}));
        };
        // Latest Peuchmower2 backend, numeric fields and sequence are exact.
        // Additional scaffolding fields are not hardware evidence.
        const fixed = bridgeFixedWireFixture;
        const sequence = fixed.position_observation_sequence;
        send(fixed);
        await expect(fix).toHaveText(locale.gpsStatus.compactRtkFixed);
        await expect(rtk).toContainText(locale.gpsStatus.rtkFixed);
        await expect(banner(locale.gpsStatus.compactRtkFixed, 100)).toBeVisible();
        await expect(page.getByTestId('diagnostic-health-gnss')).toContainText(locale.gpsStatus.compactRtkFixed);
        send({...fixed, position_observation_sequence: sequence + 1, rtk_mode: 2});
        await expect(fix).toHaveText(locale.gpsStatus.compactRtkFloat);
        await expect(rtk).toContainText(locale.gpsStatus.rtkFloat);
        await expect(banner(locale.gpsStatus.compactRtkFloat, 80)).toBeVisible();
        send({...fixed, position_observation_sequence: sequence + 2, value_flags: fixed.value_flags & ~1});
        await expect(fix).toHaveText(locale.gpsStatus.compact3d);
        await expect(banner(locale.gpsStatus.compact3d, 40)).toBeVisible();
        await expect(rtk).not.toContainText(locale.gpsStatus.rtkFixed);
        send({...fixed, position_observation_sequence: sequence + 3, fix_valid: false});
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        send(fixed);
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        send({...fixed, position_observation_sequence: sequence + 4});
        await expect(fix).toHaveText(locale.gpsStatus.compactRtkFixed);
        // A new source alone is not acquisition evidence: establish a negative
        // baseline before the next progressing MAVROS Float observation.
        send({backend: 'mavros_gps1', fix_valid: false, fix_type: 0, position_observation_sequence: 1});
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        send({backend: 'mavros_gps1', fix_valid: true, fix_type: 2, rtk_mode: 2, position_observation_sequence: 2});
        await expect(fix).toHaveText(locale.gpsStatus.compactRtkFloat);
        await expect(banner(locale.gpsStatus.compactRtkFloat, 80)).toBeVisible();
        expect(writes).toEqual([]);
    });
}
