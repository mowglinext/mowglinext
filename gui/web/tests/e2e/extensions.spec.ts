import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';

const manifest = {
    schema: 1, id: 'garden-link', version: '1.0.0',
    source: {repository: 'my-garden/garden-link', revision: 'a'.repeat(40)},
    runtime: {kind: 'compose'}, profile: 'isolated', grants: ['network:egress'], health: 'process-only',
};
const installed = {
    id: manifest.id, version: manifest.version, manifest, digest: 'sha256:' + 'b'.repeat(64),
    managed: true, enabled: false, compatible: true, blockers: [], recovery_required: false,
    observation: {process: 'stopped', application: 'unknown', data: 'unknown'},
};
for (const viewport of [{width: 1440, height: 1000}, {width: 390, height: 844}]) {
    test(`custom import is reviewed and installed disabled at ${viewport.width}px`, async ({page}) => {
        await page.setViewportSize(viewport);
        await installMockBackend(page, SCENARIOS[0]);
        let applied = false;
        let reviews = 0;
        const actions: unknown[] = [];
        await page.route('**/api/system/extensions/*', async route => {
            const op = new URL(route.request().url()).pathname.split('/').at(-1);
            if (op === 'state') return route.fulfill({json: {api: 1, configured: true, entries: applied ? [installed] : []}});
            const request = route.request().postDataJSON();
            expect(route.request().headers()['x-mowgli-update']).toBe('1');
            if (op === 'review') {
                reviews++;
                expect(typeof request.bundle).toBe('string');
                expect(JSON.parse(request.bundle).manifest.id).toBe(manifest.id);
                return route.fulfill({json: {action: 'install', manifest, digest: 'reviewed-token', blockers: [],
                    binding: {project: 'garden-ext-link', config_dir: '/etc/garden-link', data_dir: '/var/lib/garden-link'}}});
            }
            if (op === 'apply') {
                expect(request.digest).toBe('reviewed-token');
                expect(request.acknowledged).toBe(true);
                applied = true;
                return route.fulfill({status: 202, json: {action: 'install', id: manifest.id, phase: 'succeeded'}});
            }
            actions.push(request);
            return route.fulfill({status: 500, json: {error: 'unexpected action'}});
        });
        await page.goto('/#/extensions');
        await page.getByRole('button', {name: 'Custom import', exact: true}).click();
        const dialog = page.getByRole('dialog');
        await dialog.getByLabel('Bundle file').setInputFiles({name: 'garden-link.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify({manifest}))});
        await dialog.getByRole('button', {name: 'Review bundle', exact: true}).click();
        await expect(dialog.getByText(manifest.source.revision, {exact: true})).toBeVisible();
        await expect(dialog.getByText('network:egress', {exact: true})).toBeVisible();
        expect(applied).toBe(false);
        await dialog.getByRole('button', {name: 'Apply reviewed import', exact: true}).scrollIntoViewIfNeeded();
        await page.screenshot({path: `tests/e2e/.artifacts/extensions-review-${viewport.width}.png`, animations: 'disabled'});
        await dialog.getByRole('button', {name: 'Apply reviewed import', exact: true}).click();
        await expect(dialog).not.toBeVisible();
        await page.getByRole('button', {name: 'Refresh', exact: true}).click();
        await expect(page.locator('.extension-row')).toContainText(manifest.id);
        await expect(page.locator('.extension-row').getByRole('button', {name: 'Enable', exact: true})).toBeEnabled();
        await expect(page.locator('.extension-row')).toContainText('Unknown');
        expect(reviews).toBe(1);
        expect(actions).toEqual([]);
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
        await page.screenshot({path: `tests/e2e/.artifacts/extensions-installed-${viewport.width}.png`, fullPage: true, animations: 'disabled'});
    });
}
