import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};
import {SCENARIOS} from './mock/scenarios';

const base = SCENARIOS.find(s => s.name === 'idle-docked-full')!;
const shots = process.env.UX_SCREENSHOT_DIR || 'tests/e2e/.artifacts';
const settings = {robot_name: 'Test mower', mower_model: 'YardForce500', wheel_radius: 0.1,
    wheel_track: 0.325, blade_radius: 0.09, tool_width: 0.15, ticks_per_meter: 280.441,
    datum_lat: 48.1, datum_lon: 11.5, battery_low_percent: 15, rain_mode: 2, wheel_pid_kp: 1};

test.beforeEach(async ({page}) => {
    await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'fr'));
});

for (const viewport of [{width: 1440, height: 900}, {width: 390, height: 844}]) {
    test.describe(`${viewport.width}px`, () => {
        test.use({viewport});
        test('localized settings search targets a labelled field and appearance has no restart action', async ({page}) => {
            await installMockBackend(page, {...base, rest: {...base.rest, '/api/settings/yaml': settings}});
            await page.goto('/#/settings');
            await expect(page.getByRole('spinbutton', {name: /Largeur de coupe/})).toBeVisible();
            const unnamed = await page.getByRole('spinbutton').evaluateAll(inputs => inputs.filter(input => {
                const id = input.id;
                return !input.getAttribute('aria-label') && !input.getAttribute('aria-labelledby')
                    && !(id && document.querySelector(`label[for="${id}"]`));
            }).length);
            expect(unnamed).toBe(0);
            await page.getByRole('textbox', {name: 'Rechercher un réglage…'}).fill('LARGEUR DE COUPE');
            await page.getByRole('button', {name: /Matériel · Largeur de coupe/}).click();
            await expect(page.getByRole('spinbutton', {name: /Largeur de coupe/})).toBeFocused();
            await page.screenshot({path: `${shots}/settings-search-${viewport.width}.png`});
            await page.goto('/#/settings?section=appearance');
            await expect(page.getByRole('button', {name: /Redémarrer ROS2/})).toHaveCount(0);
            await expect(page.getByText(/Ajustez les réglages/)).toHaveCount(0);
        });

        for (const state of ['idle-docked-full', 'emergency-latched', 'offline-silent-socket']) {
            test(`STOP remains STOP with ${state}`, async ({page}) => {
                const scenario = SCENARIOS.find(s => s.name === state) ?? {name: state, silentSocket: true};
                await installMockBackend(page, {...scenario, rest: {...scenario.rest, '/api/settings/yaml': settings}});
                const commands: unknown[] = [];
                await page.route('**/api/mowglinext/call/emergency', route => {
                    commands.push(route.request().postDataJSON());
                    return route.fulfill({json: {}});
                });
                await page.goto('/#/map');
                const stop = page.getByRole('button', {name: 'Arrêt d’urgence', exact: true});
                await expect(stop).toBeVisible();
                await stop.click();
                await expect.poll(() => commands.length).toBe(1);
                expect(commands[0]).toEqual({Emergency: 1});
                if (viewport.width === 390) {
                    const style = await stop.evaluate(el => ({bg: getComputedStyle(el).backgroundColor, fg: getComputedStyle(el).color}));
                    expect(style).toEqual({bg: 'rgb(255, 107, 122)', fg: 'rgb(2, 17, 13)'});
                }
                await page.screenshot({path: `${shots}/map-${state}-${viewport.width}.png`});
            });
        }
    });
}

test('statistics distinguish completed sessions, missing odometry and active days', async ({page}) => {
    const sessions = Array.from({length: 55}, (_, i) => ({id: String(i), start_time: new Date().toISOString(),
        duration_sec: 3600, area_index: 0, coverage_percent: 50, status: i < 18 ? 'completed' : 'aborted'}));
    await installMockBackend(page, {...base, rest: {
        '/api/diagnostics/sessions': {sessions, total: 55},
        '/api/diagnostics/sessions/stats': {total_sessions: 55, completed: 18, total_duration_sec: 198000, avg_coverage_pct: 50},
    }});
    await page.goto('/#/statistics');
    await expect(page.getByText('Tontes terminées').locator('..').getByText('18', {exact: true})).toBeVisible();
    await expect(page.getByText('Jours actifs').locator('..').getByText('1', {exact: true})).toBeVisible();
    await expect(page.getByText('Aucune distance enregistrée pour ces sessions')).toBeVisible();
    await expect(page.getByText('Terminée', {exact: true}).first()).toBeVisible();
    await page.screenshot({path: `${shots}/statistics.png`});
});

test('planning uses saved thresholds and labels inactive schedules', async ({page}) => {
    await installMockBackend(page, {...base, rest: {'/api/settings/yaml': settings,
        '/api/schedules': {schedules: [{id: 'mock', time: '09:00', area: 0, daysOfWeek: [1], enabled: false}]}}});
    await page.goto('/#/schedule');
    await expect(page.getByText('retour sous 15 % de batterie')).toBeVisible();
    await expect(page.getByRole('switch', {name: 'Activer le planning 1'})).not.toBeChecked();
    await expect(page.getByText('Inactif').first()).toBeVisible();
    await page.screenshot({path: `${shots}/planning.png`});
});

test('mobile onboarding restores scroll position and keeps model names readable', async ({page}) => {
    await page.setViewportSize({width: 390, height: 844});
    await installMockBackend(page, {...base, rest: {'/api/settings/yaml': settings}});
    await page.goto('/#/onboarding');
    await page.getByRole('button', {name: /Commencer/}).click();
    await expect(page.getByRole('textbox', {name: /Nom du robot/})).toBeVisible();
    await expect.poll(() => page.getByRole('main').evaluate(el => el.scrollTop)).toBe(0);
    await expect(page.getByRole('radio', {name: /YardForce.*500/}).first()).toBeVisible();
    await page.screenshot({path: `${shots}/onboarding-mobile.png`});
});

test('all settings categories render in French without errors', async ({page}) => {
    test.setTimeout(60000);
    const errors: string[] = [];
    page.on('pageerror', error => errors.push(error.message));
    await installMockBackend(page, {...base, rest: {'/api/settings/yaml': settings}});
    for (const section of ['updates', 'appearance', 'hardware', 'drive_motor', 'ntrip', 'positioning', 'sensors',
        'localization', 'mowing', 'docking', 'battery', 'safety', 'obstacles', 'navigation', 'weather', 'leds',
        'mqtt', 'remote_access', 'notifications', 'advanced']) {
        await page.goto(`/#/settings?section=${section}`);
        await expect(page.getByRole('main').getByRole('heading', {level: 1, name: fr.settingsSections[section as keyof typeof fr.settingsSections].label, exact: true})).toBeVisible();
        await expect.soft(page.getByRole('spinbutton', {name: '', exact: true}), `unlabelled numbers in ${section}`).toHaveCount(0);
        await expect.soft(page.getByRole('switch', {name: '', exact: true}), `unlabelled switches in ${section}`).toHaveCount(0);
    }
    expect(errors).toEqual([]);
});

for (const failLiveApply of [false, true]) {
    test(`drive settings save only dirty keys; live apply failure=${failLiveApply}`, async ({page}) => {
        await installMockBackend(page, {...base, rest: {'/api/settings/yaml': settings}});
        const writes: unknown[] = [];
        let liveCalls = 0;
        await page.route('**/api/settings/yaml', async route => {
            if (route.request().method() === 'POST') writes.push(route.request().postDataJSON());
            await route.fulfill({json: settings});
        });
        await page.route('**/api/params', async route => {
            liveCalls++;
            await route.fulfill({status: failLiveApply ? 503 : 200, json: failLiveApply ? {error: 'Mock apply failure'} : {parameters: []}});
        });
        await page.goto('/#/settings?section=drive_motor');
        await page.locator('#setting-wheel_pid_kp').fill('2');
        await page.getByRole('button', {name: /Enregistrer \(/}).click();
        await expect.poll(() => liveCalls).toBe(1);
        expect(writes).toEqual([{wheel_pid_kp: 2}]);
        const pending = page.getByText(fr.settingsPage.restartRequired, {exact: true});
        if (failLiveApply) {
            await expect(pending).toBeVisible();
            await page.goto('/#/settings?section=hardware');
            await expect(pending).toBeVisible();
        } else {
            await expect(pending).toHaveCount(0);
            await expect(page.getByRole('button', {name: /Redémarrer ROS2/})).toHaveCount(0);
        }
    });
}

test('diagnostics do not claim an accepted estimate before any LiDAR candidate', async ({page}) => {
    await installMockBackend(page, {...base, topics: {...base.topics, fusionDiag: {
        header: {stamp: {sec: 2000000000, nanosec: 0}},
        status: [{name: 'fusion_graph', level: 0, message: 'OK', values: [
            {key: 'lidar_anchor_state', value: '2'}, {key: 'lidar_anchor_verdict', value: 'accepted'},
            {key: 'lidar_anchor_updates', value: '0'}, {key: 'lidar_anchor_factors', value: '0'},
        ]}],
    }}});
    await page.goto('/#/diagnostics?tab=localization');
    await expect(page.getByText('Pas encore d’estimation')).toBeVisible();
    await expect(page.getByText(/Publication du producteur \(horloge ROS\)/)).toContainText('2000000000');
    await expect(page.getByText(/message reçu il y a -/)).toHaveCount(0);
    await page.screenshot({path: `${shots}/diagnostics.png`});
});
