import {test, expect} from '@playwright/test';
import {SCENARIOS} from './mock/scenarios';
import {installMockBackend} from './mock/mockBackend';

const source = {repository: 'mowglinext/mowglinext', track: 'dev', branch: 'dev'};
const target = {id: 'deployment-a9132f4e-274-1', source, revision: 'a9132f4e'.padEnd(40, 'a'), published_at: '2026-09-07T09:00:00Z', updater: {'linux/arm64': {version: 'deployment-a9132f4e-274-1'}}};
const status = {
    api: 1, agent: {version: 'updater-1944f97d', revision: '1944f97d', platform: 'linux/arm64'}, trusted_repositories: [source.repository],
    state: {policy: {source, interval_hours: 4, pinned: false}, installed_policy: {source, interval_hours: 4, pinned: true},
        active: {...target, id: 'deployment-1944f97d-231-1'}, last_check: '2026-09-07T09:30:00Z', next_check: '2026-09-07T13:35:00Z', last_success: '2026-09-07T09:30:00Z',
        releases: [target], notices: [{id: 'sha256:notice', deployment: target.id, kind: 'available', read: false, dismissed: false, created_at: '2026-09-07T09:30:00Z'}], history: [],
    },
};
for (const mobile of [false, true]) {
    test(`coordinated update review ${mobile ? 'mobile' : 'desktop'}`, async ({page}) => {
        await page.setViewportSize(mobile ? {width: 390, height: 844} : {width: 1440, height: 1100});
        const errors: string[] = []; page.on('pageerror', error => errors.push(error.message));
        await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/system/updater/state': status, '/api/system/versions': {docker_available: true, components: [], server: {version: 'dev'}}}});
        const requests: string[] = [];
        page.on('request', r => {if (r.method() === 'POST' && r.url().includes('/system/updater/')) requests.push(new URL(r.url()).pathname);});
        const digest = 'sha256:' + '1'.repeat(64);
        await page.route('**/api/system/updater/plan', route => route.fulfill({json: {id: 'plan-123', target, previous: {gui: 'sha256:previous-gui', mowgli: 'sha256:previous-ros'}, images: {gui: `ghcr.io/mowglinext/mowglinext/mowglinext-gui@${digest}`, mowgli: `ghcr.io/mowglinext/mowglinext/mowgli-ros2@${digest}`}, expires_at: '2026-09-07T09:45:00Z'}}));
        await page.goto('/#/settings?section=updates');
        const panel = page.getByTestId('host-updater');
        await expect(panel.getByText('updater-1944f97d', {exact: true})).toBeVisible();
        await expect(panel.getByText('Pinned', {exact: true})).toBeVisible();
        expect(requests).toEqual([]);
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        await page.screenshot({path: `tests/e2e/.artifacts/host-updater-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true});
        await panel.getByRole('button', {name: 'Review installation', exact: true}).click();
        const review = page.getByRole('dialog');
        await expect(review.getByText('sha256:previous-gui', {exact: true})).toBeVisible();
        expect(requests).toEqual(['/api/system/updater/plan']);
        await expect(page.locator('.ant-modal')).not.toHaveClass(/ant-zoom/);
        await expect(review).toHaveCSS('opacity', '1');
        await page.screenshot({path: `tests/e2e/.artifacts/host-updater-review-${mobile ? 'mobile' : 'desktop'}.png`, fullPage: true});
        await review.getByRole('button', {name: 'Cancel', exact: true}).click();
        expect(requests).not.toContain('/api/system/updater/apply');
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
