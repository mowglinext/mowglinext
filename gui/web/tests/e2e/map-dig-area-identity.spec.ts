import {expect, test} from '@playwright/test';
import {pack} from 'msgpackr';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};

const cases = [
    {name: 'unchanged map', action: 'area', change: 'none', phase: 'none'},
    {name: 'reordered during confirmation', action: 'area', change: 'reorder', phase: 'confirmation'},
    {name: 'reordered during clear', action: 'area', change: 'reorder', phase: 'clear'},
    {name: 'removed during confirmation', action: 'area', change: 'remove', phase: 'confirmation'},
    {name: 'removed during clear', action: 'area', change: 'remove', phase: 'clear'},
    {name: 'missing index during confirmation', action: 'area', change: 'missing-index', phase: 'confirmation'},
    {name: 'missing index during clear', action: 'area', change: 'missing-index', phase: 'clear'},
    {name: 'cancelled confirmation', action: 'area', change: 'none', phase: 'cancel'},
    {name: 'refused clear', action: 'area', change: 'none', phase: 'refuse'},
    {name: 'without DIG', action: 'area', change: 'none', phase: 'no-dig'},
    {name: 'global Start', action: 'start', change: 'none', phase: 'none'},
    {name: 'global Continue', action: 'continue', change: 'none', phase: 'none'},
] as const;

for (const scenario of cases) {
    test(`MapPage stable area identity: ${scenario.name}`, async ({page}, info) => {
        const base = SCENARIOS.find(item => item.name === 'mowing-area2-rtk-fixed')!;
        const points = [{x: -5, y: -5}, {x: 5, y: -5}, {x: 5, y: 5}, {x: -5, y: 5}];
        const area = (id: number, name: string) => ({id, name, area: {points}, obstacles: []});
        const topics: Record<string, unknown> = {
            ...base.topics,
            highLevelStatus: {
                ...(base.topics!.highLevelStatus as object), state: 1,
                state_name: scenario.phase === 'no-dig' ? 'IDLE' : 'DIG_OBSTRUCTION',
            },
            status: {
                ...(base.topics!.status as object), dig_escalated: scenario.phase !== 'no-dig',
                dig_escalated_distance_m: 0, dig_escalated_required_distance_m: 0.5,
            },
            map: {working_area_indices: [7], working_area: [area(42, 'Selected original')]},
        };
        const pageErrors: string[] = [];
        page.on('pageerror', error => pageErrors.push(error.message));
        await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
        await installMockBackend(page, {
            ...base, topics,
            rest: {...base.rest, '/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}},
        });
        let publishMap: (map: unknown) => void = () => { throw new Error('Map subscription not ready'); };
        await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, ws => {
            ws.onMessage(message => {
                const op = JSON.parse(message.toString()) as {op?: string; topic?: string};
                if (op.op !== 'subscribe' || !op.topic) return;
                const data = topics[op.topic];
                if (data !== undefined) ws.send(pack({topic: op.topic, data}));
                if (op.topic === 'map') publishMap = map => ws.send(pack({topic: 'map', data: map}));
            });
        });
        const commands: {name: string; body: unknown}[] = [];
        let releaseClear: () => void = () => {};
        const clearGate = new Promise<void>(resolve => { releaseClear = resolve; });
        await page.route(/\/api\/mowglinext\/call\//, async route => {
            const name = new URL(route.request().url()).pathname.split('/').pop()!;
            if (name !== 'get_lidar_ignore_corridors') {
                commands.push({name, body: route.request().postDataJSON()});
            }
            if (name === 'clear_dig_escalation' && scenario.phase === 'clear') await clearGate;
            await route.fulfill(name === 'clear_dig_escalation' && scenario.phase === 'refuse'
                ? {status: 500, json: {error: 'Release refused'}} : {json: {}});
        });
        await page.goto('/#/map');
        const main = page.getByRole('main');
        const areaButton = main.getByRole('button', {name: new RegExp(en.mapToolbar.mowArea)});
        const startButton = main.getByRole('button', {name: new RegExp(`${en.mapToolbar.start}$`)});
        const moreButton = main.getByRole('button', {name: /More$/});
        const trigger = scenario.action === 'area' ? areaButton : scenario.action === 'start' ? startButton : moreButton;
        await expect(trigger).toHaveCount(1);
        await trigger.click();
        if (scenario.action === 'area') {
            const choice = page.getByRole('menuitem');
            await expect(choice).toHaveCount(1);
            await choice.click();
        } else if (scenario.action === 'continue') {
            await page.getByRole('menuitem', {name: new RegExp(en.mapToolbar.continue)}).click();
        }
        const updateMap = async () => {
            if (scenario.change === 'reorder') {
                publishMap({working_area_indices: [7, 9], working_area: [area(99, 'Inserted'), area(42, 'Selected updated')]});
            } else if (scenario.change === 'remove') {
                publishMap({working_area_indices: [7], working_area: [area(99, 'Replacement updated')]});
            } else {
                publishMap({working_area_indices: [], working_area: [area(42, 'Selected updated')]});
            }
            // Observe the new map committed by React before releasing the next phase.
            await expect(main.getByText(scenario.change === 'remove' ? 'Replacement updated' : 'Selected updated', {exact: false})).toBeVisible();
        };
        const dialog = page.getByRole('dialog');
        if (scenario.phase === 'no-dig') {
            await expect(dialog).toHaveCount(0);
        } else {
            await expect(dialog).toBeVisible();
            expect(commands).toEqual([]);
            if (scenario.phase === 'confirmation') await updateMap();
            await dialog.getByRole('button', {
                name: scenario.phase === 'cancel' ? en.digObstructionAck.cancel : en.digObstructionAck.ok,
                exact: true,
            }).click();
            if (scenario.phase === 'clear') {
                await expect.poll(() => commands.length).toBe(1);
                expect(commands[0].name).toBe('clear_dig_escalation');
                await updateMap();
                releaseClear();
            }
        }
        const unavailable = scenario.change === 'remove' || scenario.change === 'missing-index';
        if (unavailable || scenario.phase === 'refuse') {
            await expect(page.getByText(unavailable ? en.crossHatch.areaUnavailable : 'Release refused', {exact: true})).toBeVisible();
            await expect(trigger).not.toHaveClass(/ant-btn-loading/);
            expect(commands).toEqual([{name: 'clear_dig_escalation', body: {}}]);
        } else if (scenario.phase === 'cancel') {
            await expect(dialog).toBeHidden();
            await expect(trigger).not.toHaveClass(/ant-btn-loading/);
            expect(commands).toEqual([]);
        } else {
            const clear = scenario.phase === 'no-dig' ? [] : [{name: 'clear_dig_escalation', body: {}}];
            await expect.poll(() => commands.length).toBe(clear.length + 1);
            expect(commands).toEqual([...clear, scenario.action === 'area'
                ? {name: 'start_in_area', body: {area: scenario.change === 'reorder' ? 9 : 7}}
                : {name: 'high_level_control', body: {Command: 1}}]);
            await expect(trigger).not.toHaveClass(/ant-btn-loading/);
        }
        expect(pageErrors).toEqual([]);
        await info.attach('commands', {body: JSON.stringify(commands), contentType: 'application/json'});
        await page.screenshot({path: info.outputPath('map.png')});
    });
}
