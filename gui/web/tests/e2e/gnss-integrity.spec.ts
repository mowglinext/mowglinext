import {expect, test, type WebSocketRoute} from '@playwright/test';
import {pack} from 'msgpackr';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};
import sanitized from '../../src/test/fixtures/gnss-mavros-sanitized.json' with {type: 'json'};
import {isJsonObject} from '../../src/utils/json';

for (const language of ['en', 'fr']) {
    test(`GNSS source/availability and malformed-frame safety in ${language}`, async ({page}) => {
        const locale = language === 'fr' ? fr : en;
        await page.setViewportSize({width: 1440, height: 1100});
        await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
        await installMockBackend(page, SCENARIOS[0]);
        let socket: WebSocketRoute | undefined;
        const topics: Record<string, unknown> = {...SCENARIOS[0].topics,
            gps: {pose: {pose: {position: {x: 0, y: 0, z: 0}}}}};
        await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, ws => {
            socket = ws;
            ws.onMessage(message => {
                const request: unknown = JSON.parse(typeof message === 'string' ? message : message.toString());
                if (!isJsonObject(request) || request.op !== 'subscribe' || typeof request.topic !== 'string'
                    || request.topic === 'gnssStatus') return;
                const data = topics[request.topic];
                if (data !== undefined) ws.send(pack({topic: request.topic, data}));
            });
        });
        const writes: string[] = [];
        page.on('request', request => {
            if (request.method() !== 'GET' && /^\/api\/(?:params|settings\/yaml|mowglinext\/(?:call|map|publish))/.test(new URL(request.url()).pathname)) writes.push(request.url());
        });
        await page.goto('/#/diagnostics');
        await page.getByRole('tab', {name: new RegExp(locale.diagnosticsPage.tabLocalization)}).click();
        const card = page.locator('.ant-card').filter({has: page.getByTestId('gnss-fix-status-value')});
        const fix = page.getByTestId('gnss-fix-status-value');
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        await expect(card).toContainText('0.000000000');
        await expect(card).not.toContainText(locale.gpsStatus.rtkFixed);
        const send = (data: unknown) => {
            if (!socket) throw new Error('Missing mocked socket');
            socket.send(pack({topic: 'gnssStatus', data}));
        };
        send(sanitized);
        await expect(fix).toHaveText(locale.gpsStatus.compactRtkFixed);
        await expect(card).toContainText('mavros_gps1');
        const accuracy = card.locator('.ant-statistic').filter({hasText: locale.diagnosticsPage.horizontalAccuracyM});
        await expect(accuracy.locator('.ant-statistic-content')).toHaveText('0.000');
        await expect(page.getByTestId('gnss-correction-status-group')).toContainText(locale.diagnosticsPage.unknown);
        send({...sanitized, fix_valid: false});
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        await expect(page.getByTestId('gnss-rtk-status-group')).not.toContainText(locale.gpsStatus.rtkFixed);
        // Cached positive data cannot undo the explicit negative observation.
        send(sanitized);
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        send({...sanitized, position_observation_sequence: 43, receiver_vendor: 'Fictional', receiver_model: 'Receiver A',
            capability_flags: 8388633, value_flags: 8388617, correction_stream_status: 3});
        await expect(fix).toHaveText(locale.gpsStatus.compactRtkFixed);
        await expect(card).toContainText('Fictional Receiver A');
        await expect(page.getByTestId('gnss-correction-status-group')).toContainText(locale.gpsStatus.correctionStreamActive);
        // Changing the declared source must not erase the original source barrier.
        send({backend: 'universal', fix_valid: false, fix_type: 0, position_observation_sequence: 1});
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        send({...sanitized, position_observation_sequence: 43});
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        const {backend: omittedBackend, ...anonymous} = sanitized;
        expect(omittedBackend).toBe('mavros_gps1');
        send({...anonymous, position_observation_sequence: 44});
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        send({...sanitized, position_observation_sequence: 44});
        await expect(fix).toHaveText(locale.gpsStatus.compactRtkFixed);
        await expect(page.getByTestId('gnss-correction-status-group')).toContainText(locale.diagnosticsPage.unknown);
        send({...sanitized, position_observation_sequence: 45, horizontal_accuracy_m: 'bad'});
        await expect(fix).toHaveText(locale.gpsStatus.compactUnknown);
        await expect(accuracy.locator('.ant-statistic-content')).toHaveText('—');
        expect(writes).toEqual([]);
    });
}
