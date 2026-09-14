import {test, expect} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';

for (const mobile of [false,true]) {
    test(`read-only tag checks report matching and changed images on ${mobile ? 'mobile' : 'desktop'}`, async ({page}) => {
        await page.setViewportSize(mobile ? {width:390,height:844} : {width:1440,height:1000});
        await installMockBackend(page, SCENARIOS[0]);
        const requests: string[] = [];
        const errors: string[] = [];
        let relation = 'newer';
        page.on('pageerror', error => errors.push(error.message));
        page.on('request', request => { if (request.method() !== 'GET' && /\/api\/(containers|system|mowglinext\/call)/.test(request.url())) requests.push(request.url()); });
        await page.route('**/api/system/updates?*', route => {
            const url = new URL(route.request().url());
            const channel = url.searchParams.get('channel');
            const checking = url.searchParams.get('check') === 'true';
            return route.fulfill({json: {
                channel, state: checking ? channel === 'dev' ? 'available' : 'current' : 'not_checked',
                version: checking ? channel === 'dev' ? 'dev' : '1.2.0' : undefined,
                checked_at: checking ? '2026-09-05T16:00:00Z' : undefined,
                last_successful_at: checking ? '2026-09-05T16:00:00Z' : undefined,
                notes_url: 'https://github.com/mowglinext/mowglinext/releases/tag/v1.2.0',
                components: checking ? [{name:'mowgli-gui',state:channel === 'dev' ? 'changed' : 'current',source_relation:channel === 'dev' ? relation : undefined,installed_revision:'aaaaaaa',available_revision:channel === 'dev' ? 'bbbbbbb' : 'aaaaaaa',custom_image:true,digest_reference:true,available_image:'ghcr.io/mowglinext/mowglinext/mowglinext-gui@sha256:'+'b'.repeat(64)}] : [],
            }});
        });
        await page.goto('/#/settings?section=updates');
        await page.locator('.ant-segmented').getByText('Advanced', {exact: true}).click();
        const card = page.getByTestId('update-checks');
        await expect(card.getByText('Not checked yet', {exact:true})).toBeVisible();
        await card.getByRole('button', {name:'Check now'}).click();
        await expect(card.getByText('Different images are available', {exact:true})).toBeVisible();
        await expect(card.getByText('Different image available', {exact:true})).toBeVisible();
        await expect(card.getByText('Available image uses newer source', {exact:true})).toBeVisible();
        await card.screenshot({path:`tests/e2e/.artifacts/update-source-newer-${mobile ? 'mobile' : 'desktop'}.png`});
        for (const [value, label] of [
            ['older', 'Installed source is newer'],
            ['diverged', 'Source histories have diverged; neither is strictly newer'],
            ['same', 'Same source commit, different build'],
            ['unknown', 'Source order could not be determined'],
        ]) {
            relation = value;
            await card.getByRole('button', {name:'Check now'}).click();
            await expect(card.getByText(label, {exact:true})).toBeVisible();
            await expect(card.getByText('Available image uses newer source', {exact:true})).toHaveCount(0);
            expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
        }
        await card.getByText('Development', {exact:true}).click();
        await page.getByText('Stable releases', {exact:true}).click();
        await expect(card.getByText('Not checked yet', {exact:true})).toBeVisible();
        await card.getByRole('button', {name:'Check now'}).click();
        await expect(card.getByText('Installed images match the selected channel', {exact:true})).toBeVisible();
        await expect(card.getByText('Matches', {exact:true})).toBeVisible();
        await expect(card.getByText(/unverified|catalog|preparing/i)).toHaveCount(0);
        await card.getByText('Available image details').click();
        expect(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
        await page.screenshot({path:`tests/e2e/.artifacts/update-checks-${mobile ? 'mobile' : 'desktop'}.png`});
        await page.route('**/api/system/updates?*', route => route.fulfill({status:503,body:'{}'}));
        await card.getByRole('button', {name:'Check now'}).click();
        await expect(card.getByText('Update check failed', {exact:true})).toBeVisible();
        await expect(card.getByText('Installed images match the selected channel', {exact:true})).toHaveCount(0);
        expect(requests).toEqual([]);
        expect(errors).toEqual([]);
    });
}
