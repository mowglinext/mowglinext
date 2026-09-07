import {test, expect} from '@playwright/test';
import {SCENARIOS} from './mock/scenarios';
import {installMockBackend} from './mock/mockBackend';

const source = {repository: 'mowglinext/mowglinext', track: 'dev', branch: 'dev'};
const target = {gui_compatibility: 'ros-gui-1', layout: 1, data_schema: 1, updater_api: 1, maintenance_api: 1, firmware_protocol: 6, id: 'deployment-a9132f4e-274-1', source, revision: 'a9132f4e'.padEnd(40, 'a'), published_at: '2026-09-07T09:00:00Z', updater: {'linux/arm64': {version: 'deployment-a9132f4e-274-1'}}};
const status = {
    api: 1, capabilities: ['component-overrides', 'declared-services'], runtime: {identity: 'matched', health: 'healthy', checked_at: '2026-09-07T09:30:00Z'}, agent: {version: 'updater-1944f97d', revision: '1944f97d', platform: 'linux/arm64'}, trusted_repositories: [source.repository, 'wjcloudy/mowglinext'],
    state: {policy: {source, interval_hours: 4, pinned: false}, installed_policy: {source, interval_hours: 4, pinned: true},
        active: {...target, id: 'deployment-1944f97d-231-1', revision: '1944f97d'.padEnd(40, 'a')}, last_check: '2026-09-07T09:30:00Z', next_check: '2026-09-07T13:35:00Z', last_success: '2026-09-07T09:30:00Z',
        releases: [target], notices: [{id: 'sha256:notice', deployment: target.id, kind: 'available', read: false, dismissed: false, created_at: '2026-09-07T09:30:00Z'}], history: [],
    },
};
// Screenshot releases are illustrative fixtures, not claims about published upstream builds.
const productionSource = {...source, track: 'stable', branch: 'main'};
const productionTarget = {...target, source: productionSource, release_tag: 'v1.2.0'};
const productionStatus = {...status, trusted_repositories: [source.repository], state: {...status.state,
    policy: {...status.state.policy, source: productionSource},
    installed_policy: {...status.state.installed_policy, source: productionSource},
    active: {...status.state.active, source: productionSource, release_tag: 'v1.1.0'},
    releases: [productionTarget],
}};
for (const mobile of [false, true]) {
    test(`unpublished versions remain discoverable ${mobile ? 'mobile' : 'desktop'}`, async ({page}) => {
        await page.setViewportSize(mobile ? {width: 390, height: 844} : {width: 1440, height: 1400});
        const fixture = {...status, trusted_repositories: [source.repository], runtime: {...status.runtime, identity: 'custom'},
            state: {...status.state, active: undefined, installed_policy: undefined, releases: [], notices: []}};
        const posts: string[] = [];
        await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': fixture, '/api/system/versions': {docker_available: true, components: [], server: {version: 'dev'}}}});
        page.on('request', r => {if (r.method() === 'POST' && r.url().includes('/system/updater/')) posts.push(r.url());});
        await page.goto('/#/settings?section=updates');
        await page.locator('.ant-segmented').getByText('Advanced', {exact: true}).click();
        const panel = page.getByTestId('host-updater');
        await expect(panel.getByRole('combobox', {name: 'Deployment version', exact: true})).toBeDisabled();
        await expect(panel.getByRole('combobox', {name: 'GUI version', exact: true})).toBeDisabled();
        await expect(panel.getByText(/Version choices require a compatible complete build/)).toBeVisible();
        await expect(panel.getByRole('button', {name: 'Review installation', exact: true})).toBeDisabled();
        await expect(panel.getByRole('button', {name: 'Check now', exact: true})).toBeEnabled();
        await expect(panel.getByRole('checkbox')).toBeDisabled();
        if (mobile) await panel.getByText('No installable build published for this source.', {exact: true}).scrollIntoViewIfNeeded();
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        await page.screenshot({path: `tests/e2e/.artifacts/host-updater-no-versions-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true, animations: 'disabled'});
        expect(posts).toEqual([]);
    });
    test(`release stack membership review ${mobile ? 'mobile' : 'desktop'}`, async ({page}) => {
        await page.setViewportSize(mobile ? {width: 390, height: 844} : {width: 1440, height: 1100});
        const next = {...productionTarget, id: 'illustration-production-130', release_tag: 'v1.3.0'};
        const fixture = {...productionStatus, capabilities: [...status.capabilities, 'release-compose'], state: {...productionStatus.state, releases: [next], active: {...productionTarget, release_tag: 'v1.2.0'}}};
        const posts: string[] = [];
        await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': fixture, '/api/system/versions': {docker_available: true, components: [], server: {version: 'dev'}}}});
        page.on('request', r => {if (r.method() === 'POST' && r.url().includes('/system/updater/')) posts.push(new URL(r.url()).pathname);});
        await page.route('**/api/system/updater/plan', route => route.fulfill({json: {
            id: 'illustration-stack-plan', target: next, expires_at: '2026-09-07T09:45:00Z', previous: {}, images: {gui: 'fixture-gui', mowgli: 'fixture-ros', gps: 'fixture-gps', 'navigation-helper': 'fixture-helper'},
            stack: {selection: {options: {gnss: 'universal', lidar: 'none'}}, changes: [
                {service: 'mowgli', action: 'update'}, {service: 'gui', action: 'update'}, {service: 'gps', action: 'keep'},
                {service: 'navigation-helper', action: 'add'}, {service: 'legacy-helper', action: 'remove'}, {service: 'mqtt', action: 'unmanaged'},
            ]},
        }}));
        await page.goto('/#/settings?section=updates');
        const panel = page.getByTestId('host-updater');
        await expect(panel.getByText('v1.3.0')).toBeVisible();
        expect(posts).toEqual([]);
        await panel.getByRole('button', {name: 'Review installation', exact: true}).click();
        const review = page.getByRole('dialog');
        await expect(review.getByText('GPS: On', {exact: true})).toBeVisible();
        await expect(review.getByText('LiDAR: Off', {exact: true})).toBeVisible();
        await expect(review.getByText('Add', {exact: true})).toBeVisible();
        await expect(review.getByText('Remove', {exact: true})).toBeVisible();
        await expect(review.getByText('Keep · local', {exact: true})).toBeVisible();
        await expect(review.getByText(/Removed containers retain their data/)).toBeVisible();
        await expect(review).toHaveCSS('opacity', '1');
        await expect(review.getByRole('button', {name: 'Install reviewed deployment', exact: true})).toBeInViewport();
        await expect(panel.locator('button').filter({hasText: 'Check now'})).not.toHaveClass(/ant-btn-loading/);
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        await page.screenshot({path: `tests/e2e/.artifacts/host-updater-stack-review-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true, animations: 'disabled'});
        await review.getByRole('button', {name: 'Cancel', exact: true}).click();
        expect(posts).toEqual(['/api/system/updater/plan']);
    });
}

test('changed installer selection can review the installed release without a new version', async ({page}) => {
    const fixture = {...productionStatus, runtime: {...status.runtime, selection_pending: true}, state: {...productionStatus.state, releases: []}};
    await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': fixture}});
    await page.goto('/#/settings?section=updates');
    const panel = page.getByTestId('host-updater');
    await expect(panel.getByText(/Installer hardware choices have changed/)).toBeVisible();
    await expect(panel.getByRole('button', {name: 'Review installation', exact: true})).toBeEnabled();
});
for (const fixture of [
    {name: 'development', prefix: 'host-updater', status: {...status, trusted_repositories: [source.repository]}, target, installed: 'Development · 1944f97d', available: 'Development · a9132f4e'},
    {name: 'production', prefix: 'host-updater-production', status: productionStatus, target: productionTarget, installed: 'v1.1.0', available: 'v1.2.0'},
]) for (const mobile of [false, true]) {
    test(`coordinated ${fixture.name} update review ${mobile ? 'mobile' : 'desktop'}`, async ({page}) => {
        await page.setViewportSize(mobile ? {width: 390, height: 844} : {width: 1440, height: 1100});
        const errors: string[] = []; page.on('pageerror', error => errors.push(error.message));
        await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': fixture.status, '/api/system/versions': {docker_available: true, components: [], server: {version: 'dev'}}}});
        const requests: string[] = [];
        page.on('request', r => {if (r.method() === 'POST' && r.url().includes('/system/updater/')) requests.push(new URL(r.url()).pathname);});
        const digest = 'sha256:' + '1'.repeat(64);
        await page.route('**/api/system/updater/plan', route => {expect(route.request().postDataJSON()).toEqual({deployment: fixture.target.id, pinned: true}); return route.fulfill({json: {id: 'plan-123', target: fixture.target, previous: {gui: 'sha256:previous-gui', mowgli: 'sha256:previous-ros'}, images: {gui: `ghcr.io/mowglinext/mowglinext/mowglinext-gui@${digest}`, mowgli: `ghcr.io/mowglinext/mowglinext/mowgli-ros2@${digest}`}, expires_at: '2026-09-07T09:45:00Z'}});});
        await page.goto('/#/settings?section=updates');
        const panel = page.getByTestId('host-updater');
        await expect(panel.getByText(fixture.installed, {exact: true})).toBeVisible();
        await expect(panel.getByText(fixture.available)).toBeVisible();
        await expect(panel.getByRole('combobox')).toHaveCount(0);
        await expect(panel.getByText(/wjcloudy/)).toHaveCount(0);
        await expect(page.getByTestId('update-checks')).toHaveCount(0);
        await expect(panel.getByText('Pinned', {exact: true})).toBeVisible();
        expect(requests).toEqual([]);
        await expect(page.locator('.ant-select-dropdown:visible')).toHaveCount(0);
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        await page.screenshot({path: `tests/e2e/.artifacts/${fixture.prefix}-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true, animations: 'disabled'});
        await panel.getByRole('button', {name: 'Review installation', exact: true}).click();
        const review = page.getByRole('dialog');
        await expect(review.getByText(fixture.available, {exact: true})).toBeVisible();
        await expect(review.getByText(`${source.repository} · ${fixture.target.source.branch}`, {exact: true})).toBeVisible();
        await expect(review.getByText('sha256:previous-gui', {exact: true})).not.toBeVisible();
        await expect(review.getByText('Settings and maps are backed up before installation. Mainboard firmware is unchanged.')).toBeVisible();
        expect(requests).toEqual(['/api/system/updater/plan']);
        await expect(page.locator('.ant-modal')).not.toHaveClass(/ant-zoom/);
        await expect(review).toHaveCSS('opacity', '1');
        await expect(review).toHaveCSS('opacity', '1');
        await expect(panel.locator('button').filter({hasText: 'Check now'})).not.toHaveClass(/ant-btn-loading/);
        await page.screenshot({path: `tests/e2e/.artifacts/${fixture.prefix}-review-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true, animations: 'disabled'});
        await review.getByRole('button', {name: 'Cancel', exact: true}).click();
        expect(requests).not.toContain('/api/system/updater/apply');
        await page.locator('.ant-segmented').getByText('Advanced', {exact: true}).click();
        await expect(panel.getByRole('combobox', {name: 'Repository', exact: true})).toBeVisible();
        await expect(panel.getByRole('checkbox')).toBeChecked();
        if (fixture.name === 'development') {
            await panel.getByRole('combobox', {name: 'Update source', exact: true}).locator('..').locator('..').click();
            await page.locator('.ant-select-dropdown:visible').getByText('Custom branch', {exact: true}).click();
            await panel.getByRole('textbox', {name: 'Custom branch', exact: true}).fill('feat/example-update');
            await expect(panel.getByRole('button', {name: 'Review installation', exact: true})).toBeDisabled();
        }
        await expect(panel.getByText(/wjcloudy/)).toHaveCount(0);
        await expect(page.locator('.ant-select-dropdown:visible')).toHaveCount(0);
        await expect(page.locator('.ant-segmented-thumb')).toHaveCount(0);
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        expect(await panel.evaluate(e => e.getBoundingClientRect().left)).toBeGreaterThanOrEqual(0);
        await page.screenshot({path: `tests/e2e/.artifacts/${fixture.prefix}-advanced-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true, animations: 'disabled'});
        expect(errors).toEqual([]);
    });
}
test('reconnect preserves job status and never repeats an installation', async ({page}) => {
    await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': {...status, state: {...status.state, job: {id: 'job-123', kind: 'containers', phase: 'applying', plan: {target}}}}}});
    await page.goto('/#/settings?section=updates');
    const panel = page.getByTestId('host-updater');
    await expect(panel.getByText('Replacing containers')).toBeVisible();
    await page.route('**/api/system/updater/state', route => route.fulfill({status: 503, json: {error: 'GUI restarting'}}));
    await expect(panel.getByText('Update in progress — reconnecting to the GUI')).toBeVisible({timeout: 12000});
    await expect(panel.getByRole('button', {name: 'Review installation', exact: true})).toBeDisabled();
});

test('custom fork selection is saved explicitly and never installs on selection', async ({page}) => {
    let current = structuredClone(status);
    const posts: {path: string; body: unknown}[] = [];
    await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': current}});
    await page.route('**/api/system/updater/state', route => route.fulfill({json: current}));
    await page.route('**/api/system/updater/policy', async route => {
        const body = route.request().postDataJSON();
        posts.push({path: 'policy', body});
        current = {...current, state: {...current.state, policy: body, releases: []}};
        await route.fulfill({json: {ok: true}});
    });
    await page.route('**/api/system/updater/check', route => {posts.push({path: 'check', body: route.request().postDataJSON()}); return route.fulfill({status: 202});});
    await page.goto('/#/settings?section=updates');
    await page.locator('.ant-segmented').getByText('Advanced', {exact: true}).click();
    const panel = page.getByTestId('host-updater');
    await panel.getByRole('combobox', {name: 'Update source', exact: true}).locator('..').locator('..').click();
    await page.locator('.ant-select-dropdown:visible').getByText('Custom branch', {exact: true}).click();
    await panel.getByRole('textbox', {name: 'Custom branch', exact: true}).fill('feat/settings-updates');
    await panel.getByRole('combobox', {name: 'Repository', exact: true}).locator('..').locator('..').click();
    await page.locator('.ant-select-dropdown:visible .ant-select-item-option-content').filter({hasText: 'wjcloudy/mowglinext'}).click();
    expect(posts).toEqual([]);
    await panel.getByRole('button', {name: 'Save and check', exact: true}).click();
    await expect(panel.getByText('No installable build published for this source.')).toBeVisible();
    expect(posts).toEqual([{path: 'policy', body: {source: {repository: 'wjcloudy/mowglinext', track: 'custom', branch: 'feat/settings-updates'}, interval_hours: 4, pinned: false}}, {path: 'check', body: {}}]);
    await page.locator('.ant-segmented').getByText('Simple', {exact: true}).click();
    await expect(panel.getByText(/wjcloudy\/mowglinext \/ feat\/settings-updates/)).toBeVisible();
    await expect(panel.getByRole('button', {name: 'Review installation', exact: true})).toBeDisabled();
});

for (const mobile of [false, true]) {
    test(`advanced GUI override and matched return ${mobile ? 'mobile' : 'desktop'}`, async ({page}) => {
        await page.setViewportSize(mobile ? {width: 390, height: 844} : {width: 1440, height: 1100});
        if (!mobile) await page.setViewportSize({width: 1440, height: 1400});
        const base = {...productionTarget, id: 'deployment-prod-120', release_tag: 'v1.2.0'};
        const gui = {...productionTarget, id: 'deployment-prod-121', release_tag: 'v1.2.1', revision: 'b'.repeat(40)};
        const incompatible = {...productionTarget, id: 'deployment-prod-130', release_tag: 'v1.3.0', gui_compatibility: 'ros-gui-2'};
        const mixed = {...productionStatus, runtime: {...status.runtime, identity: 'mixed'}, state: {...productionStatus.state,
            active: base, overrides: {gui}, releases: [gui, base, incompatible]}};
        const errors: string[] = []; page.on('pageerror', error => errors.push(error.message));
        await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': mixed,
            '/api/system/versions': {docker_available: true, components: [], server: {version: 'dev'}}}});
        const requests: Record<string, unknown>[] = [];
        await page.route('**/api/system/updater/plan', route => {
            const body = route.request().postDataJSON(); requests.push(body);
            const selectedBase = body.deployment === base.id ? base : gui;
            return route.fulfill({json: {id: 'plan-mixed', target: selectedBase, overrides: body.gui_deployment ? {gui} : {},
                previous: {gui: 'sha256:old-gui', mowgli: 'sha256:base-ros'},
                images: {gui: 'ghcr.io/mowglinext/mowglinext/mowglinext-gui@sha256:' + 'a'.repeat(64), mowgli: 'ghcr.io/mowglinext/mowglinext/mowgli-ros2@sha256:' + 'b'.repeat(64)},
                expires_at: '2026-09-07T09:45:00Z'}});
        });
        await page.goto('/#/settings?section=updates');
        const panel = page.getByTestId('host-updater');
        await expect(panel.getByText('Custom combination', {exact: true})).toBeVisible();
        await expect(panel.getByText(/Container health: Running/)).toBeVisible();
        await page.screenshot({path: `tests/e2e/.artifacts/host-updater-mixed-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true, animations: 'disabled'});
        await page.locator('.ant-segmented').getByText('Advanced', {exact: true}).click();
        await panel.getByRole('combobox', {name: 'Deployment version', exact: true}).locator('..').locator('..').click();
        await page.locator('.ant-select-dropdown:visible .ant-select-item-option-content').filter({hasText: 'v1.2.0'}).click();
        await panel.getByRole('combobox', {name: 'GUI version', exact: true}).locator('..').locator('..').click();
        await expect(page.locator('.ant-select-dropdown:visible').getByText('v1.3.0', {exact: true})).toHaveCount(0);
        await page.locator('.ant-select-dropdown:visible').getByText('v1.2.1', {exact: true}).click();
        await expect(page.locator('.ant-select-dropdown:visible')).toHaveCount(0);
        if (mobile) await panel.getByRole('combobox', {name: 'GUI version', exact: true}).scrollIntoViewIfNeeded();
        expect(requests).toEqual([]);
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        await page.screenshot({path: `tests/e2e/.artifacts/host-updater-gui-override-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true, animations: 'disabled'});
        await panel.getByRole('button', {name: 'Review installation', exact: true}).click();
        await expect(page.getByRole('dialog').getByText('Custom combination: GUI v1.2.1; other components use the base release.')).toBeVisible();
        expect(requests[0]).toEqual({deployment: base.id, pinned: true, gui_deployment: gui.id});
        await expect(page.getByRole('dialog')).toHaveCSS('opacity', '1');
        await expect(panel.locator('button').filter({hasText: 'Check now'})).not.toHaveClass(/ant-btn-loading/);
        await page.screenshot({path: `tests/e2e/.artifacts/host-updater-gui-review-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true, animations: 'disabled'});
        await page.getByRole('dialog').getByRole('button', {name: 'Cancel', exact: true}).click();
        await page.locator('.ant-segmented').getByText('Simple', {exact: true}).click();
        await panel.getByRole('button', {name: 'Review matched release', exact: true}).click();
        expect(requests[1]).toEqual({deployment: gui.id, pinned: true});
        expect(errors).toEqual([]);
    });
}

test('manual Docker drift is visible independently of container health', async ({page}) => {
    const drifted = {...status, runtime: {...status.runtime, identity: 'drifted', health: 'healthy'}};
    await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': drifted}});
    await page.goto('/#/settings?section=updates');
    const panel = page.getByTestId('host-updater');
    await expect(panel.getByText('Installation changed', {exact: true})).toBeVisible();
    await expect(panel.getByText(/Container health: Running/)).toBeVisible();
    await expect(panel.getByRole('button', {name: 'Review matched release'})).toBeEnabled();
});
