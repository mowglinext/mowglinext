import {expect, test, type WebSocketRoute} from '@playwright/test';
import {pack} from 'msgpackr';
import {isJsonObject} from '../../src/utils/json';
import {healthy} from '../../src/test/diagnosticHealthFixture';
import {installMockBackend} from './mock/mockBackend';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};

for (const language of ['en', 'fr']) {
    const locale = language === 'en' ? en : fr;
    test(`missing health evidence remains unknown in ${language}`, async ({page}) => {
        await page.setViewportSize({width:390,height:900});
        await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
        await installMockBackend(page, {name: 'unknown-health', topics: {
            highLevelStatus: {}, status: {}, power: {}, emergency: {}, gnssStatus: {}, diagnostics: {status: []},
        }});
        await page.route('**/api/diagnostics/snapshot', route => route.fulfill({status: 503, json: {error: 'fictional unavailable'}}));
        await page.goto('/#/diagnostics');
        await expect(page.locator('.diagnostics-page .mn-num').first()).toHaveText(locale.telemetry.statusUnknown);
        const bounds=await page.locator('.diagnostics-page').evaluate(element=>({width:element.scrollWidth,viewport:innerWidth}));
        expect(bounds.width).toBeLessThanOrEqual(bounds.viewport);
        await expect(page.getByText(locale.diagnosticsPage.containersOk, {exact: true})).not.toBeVisible();
    });
    test(`missing CPU and container data cannot be green in ${language}`, async ({page}) => {
        await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
        await installMockBackend(page, {name: 'missing-system', topics: {
            highLevelStatus: {state: 1, battery_percent: 90}, emergency: {active_emergency: false, latched_emergency: false},
            power: {v_battery: 28}, gnssStatus: {fix_type: 3, fix_valid: true, horizontal_accuracy_m: 0},
        }});
        await page.route('**/api/diagnostics/snapshot', route => route.fulfill({json: {timestamp: new Date().toISOString(), containers: [], system: {}, coverage: [], cross_checks: {}}}));
        await page.goto('/#/diagnostics');
        await expect(page.locator('.diagnostics-page .mn-num').first()).toBeVisible();
        const cpu = page.getByTestId('diagnostic-health-cpu');
        await expect(cpu).toBeVisible();
        await expect(cpu).not.toHaveClass(/ant-tag-success/);
        await expect(page.getByText(locale.diagnosticsPage.containersOk, {exact: true})).not.toBeVisible();
    });
    test(`confirmed CPU critical follows a healthy display in ${language}`, async ({page}) => {
        await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
        const input=healthy();
        const snapshot=input.snapshot;
        if(!snapshot)throw new Error('fixture');
        snapshot.timestamp=new Date().toISOString();
        const writes:string[]=[];
        page.on('request',request=>{const path=new URL(request.url()).pathname;
            // /config/keys/get is the existing POST-based read, not a settings write.
            if((request.method()==='POST'||request.method()==='PUT')&&path!=='/api/config/keys/get')writes.push(path);});
        await installMockBackend(page,{name:'health-transition',rest:{'/api/settings/yaml':{lidar_enabled:false}},topics:{
            highLevelStatus:{state:1,battery_percent:80},
            emergency:{active_emergency:false,latched_emergency:false,stamp:{sec:1,nanosec:0}},
            power:{v_battery:28,stamp:{sec:1,nanosec:0}},gnssStatus:input.gnss,
            diagnostics:{header:{stamp:{sec:1,nanosec:0}},status:input.diagnostics.map(({receivedAt,...entry})=>{
                void receivedAt;return entry;
            })},
        }});
        await page.route('**/api/diagnostics/snapshot',route=>route.fulfill({json:snapshot}));
        await page.goto('/#/diagnostics');
        const verdict=page.getByTestId('diagnostic-health-verdict');
        await expect(verdict).toHaveText(locale.diagnosticsPage.verdictAllGood);
        await expect(page.getByTestId('diagnostic-health-cpu')).toHaveAttribute('data-level','ok');
        snapshot.system.cpu_temperature=71;
        snapshot.timestamp=new Date(Date.parse(snapshot.timestamp)+1000).toISOString();
        await page.getByRole('button',{name:new RegExp(locale.diagnosticsPage.refresh)}).first().click();
        await expect(verdict).toHaveText(locale.diagnosticsPage.verdictCritical);
        await expect(page.getByTestId('diagnostic-health-cpu')).toHaveAttribute('data-level','critical');
        await expect(page.getByTestId('diagnostic-health-cause')).toContainText(locale.diagnosticHealth.reason.cpu);
        snapshot.system.cpu_temperature=40;
        snapshot.containers[0].state='';
        snapshot.timestamp=new Date(Date.parse(snapshot.timestamp)+1000).toISOString();
        await page.getByRole('button',{name:new RegExp(locale.diagnosticsPage.refresh)}).first().click();
        await expect(verdict).toHaveText(locale.telemetry.statusUnknown);
        await expect(page.getByTestId('diagnostic-health-containers')).toHaveAttribute('data-level','unknown');
        const coreRow=page.getByRole('row').filter({hasText:'mowgli-ros2'});
        await expect(coreRow.locator('.ant-tag')).toHaveClass(/ant-tag-default/);
        await expect(coreRow).toContainText(locale.telemetry.statusUnknown);
        expect(writes).toEqual([]);
    });

    test(`unstamped autonomous context recovers after invalid data and reconnect in ${language}`, async ({page}) => {
        await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
        const input = healthy();
        const snapshot = input.snapshot;
        if (!snapshot) throw new Error('Missing snapshot fixture');
        snapshot.timestamp = new Date().toISOString();
        await installMockBackend(page, {name: 'unstamped-context', rest: {'/api/settings/yaml': {lidar_enabled: false}}});
        let socket: WebSocketRoute | undefined;
        let connections = 0;
        const highLevel = {state: 2, state_name: 'AUTONOMOUS', sub_state_name: '', current_area: 0, current_path: 0,
            current_path_index: 0, total_swaths: 0, completed_swaths: 0, skipped_swaths: 0, coverage_percent: 0,
            gps_quality_percent: 0, battery_percent: 80, is_charging: false, emergency: false};
        await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, ws => {
            socket = ws; connections++;
            const stamp = {sec: connections, nanosec: 0};
            const topics: Record<string, unknown> = {
                highLevelStatus: highLevel, gnssStatus: input.gnss,
                emergency: {active_emergency: false, latched_emergency: false, stamp}, power: {v_battery: 28, stamp},
                diagnostics: {header: {stamp}, status: [{name: 'Path Tracking', hardware_id: 'mowgli/controller_server',
                    level: 2, message: 'Fictional tracking error', values: [{key: 'recent_p95_lateral_error_m', value: '1.2'}]}]},
            };
            ws.onMessage(message => {
                const request: unknown = JSON.parse(typeof message === 'string' ? message : message.toString());
                if (!isJsonObject(request) || request.op !== 'subscribe' || typeof request.topic !== 'string') return;
                const data = topics[request.topic];
                if (data !== undefined) ws.send(pack({topic: request.topic, data}));
            });
        });
        await page.route('**/api/diagnostics/snapshot', route => route.fulfill({json: snapshot}));
        const writes: string[] = [];
        page.on('request', request => {
            if (['POST', 'PUT', 'DELETE'].includes(request.method()) && new URL(request.url()).pathname !== '/api/config/keys/get') writes.push(request.url());
        });
        await page.goto('/#/diagnostics');
        const operation = page.getByTestId('diagnostic-health-operation');
        const verdict = page.getByTestId('diagnostic-health-verdict');
        await expect(operation).toHaveAttribute('data-level', 'ok');
        await expect(verdict).toHaveText(locale.diagnosticsPage.verdictCritical);
        if (!socket) throw new Error('Missing mock socket');
        socket.send(pack({topic: 'highLevelStatus', data: {...highLevel, state: 'invalid'}}));
        await expect(operation).toHaveAttribute('data-level', 'unknown');
        socket.send(pack({topic: 'highLevelStatus', data: highLevel}));
        await expect(operation).toHaveAttribute('data-level', 'ok');
        await expect(verdict).toHaveText(locale.diagnosticsPage.verdictCritical);
        const previous = connections;
        socket.close({code: 1001, reason: 'Mock reconnect'});
        await expect.poll(() => connections).toBeGreaterThan(previous);
        await expect(operation).toHaveAttribute('data-level', 'ok');
        await expect(verdict).toHaveText(locale.diagnosticsPage.verdictCritical);
        await expect(page.getByTestId('diagnostic-health-ros')).toHaveAttribute('data-level', 'critical');
        expect(writes).toEqual([]);
    });

}
