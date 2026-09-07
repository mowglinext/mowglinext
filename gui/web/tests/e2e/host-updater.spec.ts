import {test, expect} from '@playwright/test';
import {SCENARIOS} from './mock/scenarios';
import {installMockBackend} from './mock/mockBackend';

const source = {repository: 'mowglinext/mowglinext', track: 'dev', branch: 'dev'};
const target = {id: 'deployment-a9132f4e-274-1', source, revision: 'a9132f4e'.padEnd(40, 'a'), published_at: '2026-09-07T09:00:00Z', updater: {'linux/arm64': {version: 'deployment-a9132f4e-274-1'}}};
const status = {
    api: 1, agent: {version: 'updater-1944f97d', revision: '1944f97d', platform: 'linux/arm64'}, trusted_repositories: [source.repository, 'wjcloudy/mowglinext'],
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
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        await page.screenshot({path: `tests/e2e/.artifacts/${fixture.prefix}-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true});
        await panel.getByRole('button', {name: 'Review installation', exact: true}).click();
        const review = page.getByRole('dialog');
        await expect(review.getByText(fixture.available, {exact: true})).toBeVisible();
        await expect(review.getByText(`${source.repository} · ${fixture.target.source.branch}`, {exact: true})).toBeVisible();
        await expect(review.getByText('sha256:previous-gui', {exact: true})).not.toBeVisible();
        await expect(review.getByText('Settings and maps are backed up before installation. Mainboard firmware is unchanged.')).toBeVisible();
        expect(requests).toEqual(['/api/system/updater/plan']);
        await expect(page.locator('.ant-modal')).not.toHaveClass(/ant-zoom/);
        await expect(review).toHaveCSS('opacity', '1');
        await page.screenshot({path: `tests/e2e/.artifacts/${fixture.prefix}-review-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true});
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
        await page.screenshot({path: `tests/e2e/.artifacts/${fixture.prefix}-advanced-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true});
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
