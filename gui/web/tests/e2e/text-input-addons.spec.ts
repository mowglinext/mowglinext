import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
for (const width of [390, 1440]) {
    test(`text/password addons preserve layout and form submission at ${width}px`, async ({page}) => {
        await page.setViewportSize({width, height: 900});
        const warnings: string[] = [];
        page.on('console', message => {
            if (message.text().includes('[antd: Input]') && /addonBefore|addonAfter/.test(message.text())) warnings.push(message.text());
        });
        await page.goto('/tests/e2e/fixtures/text-addons.html');
        // Measure the final layout rather than the opening zoom transform.
        await expect(page.getByRole('dialog')).not.toHaveClass(/ant-zoom-(appear|enter)/);
        for (const label of ['Fixed caster', 'Fluid password', 'Bounded email']) {
            const layout = await page.getByLabel(label).evaluate(element => {
                const compact = element.closest('.ant-space-compact');
                if (!compact) throw new Error('Missing compact');
                const rect = compact.getBoundingClientRect(), input = element.getBoundingClientRect();
                const addons = [...compact.querySelectorAll('.ant-space-addon')].map(addon => addon.getBoundingClientRect());
                return {width: rect.width, right: rect.right, left: rect.left, viewport: innerWidth, inputWidth: input.width,
                    heightDelta: Math.max(...addons.map(addon => Math.abs(addon.height - rect.height))),
                    addonRight: Math.max(...addons.map(addon => addon.right))};
            });
            expect(layout.left).toBeGreaterThanOrEqual(0); expect(layout.right).toBeLessThanOrEqual(layout.viewport);
            expect(layout.inputWidth).toBeGreaterThan(30); expect(layout.heightDelta).toBeLessThan(2);
            expect(layout.addonRight).toBeLessThanOrEqual(layout.right + 1);
            if (label === 'Fixed caster') expect(layout.width).toBeCloseTo(240, 0);
            if (label === 'Bounded email') expect(layout.width).toBeLessThanOrEqual(380);
        }
        const password = page.getByLabel('Fluid password');
        await password.fill('fake-updated'); await expect(password).toHaveAttribute('type', 'password');
        await password.locator('..').locator('.ant-input-password-icon').click();
        await expect(password).toHaveAttribute('type', 'text'); await expect(password).toBeFocused();
        await password.locator('..').locator('.ant-input-password-icon').click();
        await expect(password).toHaveAttribute('type', 'password');
        await expect(page.getByLabel('Disabled password')).toBeDisabled();
        await page.getByLabel('Fixed caster').fill('edited.example.test');
        await page.getByRole('button', {name: 'Save fixture'}).click(); await expect(page.locator('output')).toHaveText('Saved');
        expect(warnings).toEqual([]);
    });
    test(`NTRIP composed fields fit and keep credentials synchronized at ${width}px`, async ({page}) => {
        await page.setViewportSize({width, height: 900});
        await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
        const warnings: string[] = [];
        page.on('console', message => {
            if (message.text().includes('[antd: Input]') && /addonBefore|addonAfter/.test(message.text())) warnings.push(message.text());
        });
        await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/settings/yaml': {
            ntrip_enabled: true, ntrip_host: '', ntrip_user: 'fictional-user', ntrip_password: 'fake-secret',
        }}});
        const writes: (string | null)[] = [];
        await page.route('**/api/settings/yaml', async route => {
            if (route.request().method() !== 'POST') {await route.fallback(); return;}
            writes.push(route.request().postData());
            await route.fulfill({json: {}});
        });
        await page.goto('/#/settings?section=ntrip');
        await page.getByRole('button', {name: /SAPOS/}).click();
        const password = page.getByLabel('Pass', {exact: true}).first();
        await password.fill('fake-updated'); await expect(password).toHaveAttribute('type', 'password');
        await password.locator('..').locator('.ant-input-password-icon').click();
        await expect(password).toHaveAttribute('type', 'text');
        await page.getByText('Advanced: raw NTRIP fields').click();
        await expect(page.getByLabel('Pass', {exact: true}).last()).toHaveValue('fake-updated');
        for (const input of await page.locator('.ant-space-compact input.ant-input').all()) {
            const rect = await input.evaluate(element => {
                const compact = element.closest('.ant-space-compact');
                if (!compact) throw new Error('Missing compact');
                const r = compact.getBoundingClientRect(); return {left: r.left, right: r.right, viewport: innerWidth};
            });
            expect(rect.left).toBeGreaterThanOrEqual(0); expect(rect.right).toBeLessThanOrEqual(rect.viewport);
        }
        expect(writes).toHaveLength(0);
        await page.getByRole('button', {name: /Save \(/}).click();
        await expect.poll(() => writes.length).toBe(1);
        const saved: unknown = JSON.parse(writes[0] ?? 'null');
        expect(saved).toEqual(expect.objectContaining({ntrip_password: 'fake-updated'}));
        expect(warnings).toEqual([]);
    });
}
