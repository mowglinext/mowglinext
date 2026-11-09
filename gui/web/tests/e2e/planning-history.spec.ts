import {test, expect} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';

const stats = {total_sessions: 0, total_duration_sec: 0, total_distance_m: 0, completed: 0, aborted: 0, errors: 0, avg_coverage_pct: 0};

test.beforeEach(async ({page}) => {
    await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
    await installMockBackend(page, {name: 'planning-history', topics: {}, rest: {
        '/api/diagnostics/sessions/stats': stats,
    }});
});

test('More contains keyboard focus, excludes background, closes and restores its opener', async ({page}) => {
    await page.setViewportSize({width: 390, height: 844});
    await page.goto('/#/schedule');
    const opener = page.getByRole('button', {name: 'More', exact: true});
    await opener.focus();
    await page.keyboard.press('Enter');
    const dialog = page.getByRole('dialog', {name: 'More', exact: true});
    const close = dialog.getByRole('button', {name: 'Close', exact: true});
    await expect(close).toBeFocused();
    const buttons = dialog.getByRole('button');
    const count = await buttons.count();
    for (let i = 0; i < count + 2; i++) {
        await page.keyboard.press('Tab');
        await expect.poll(() => dialog.evaluate(el => el.contains(document.activeElement))).toBe(true);
    }
    await close.focus();
    await page.keyboard.press('Shift+Tab');
    await expect(buttons.last()).toBeFocused();
    await page.keyboard.press('Tab');
    await expect(close).toBeFocused();
    await expect.poll(() => dialog.evaluate(el => el.contains(document.activeElement))).toBe(true);
    // Even programmatic focus on the background is refused by the native modal.
    await opener.evaluate(el => el.focus());
    await expect(opener).not.toBeFocused();
    await page.keyboard.press('Escape');
    await expect(dialog).not.toBeVisible();
    await expect(opener).toBeFocused();
    await page.keyboard.press('Enter');
    await expect(close).toBeFocused();
    await close.click();
    await expect(opener).toBeFocused();
    await page.keyboard.press('Enter');
    await page.mouse.click(10, 10);
    await expect(dialog).not.toBeVisible();
    await expect(opener).toBeFocused();
    await page.keyboard.press('Enter');
    await dialog.getByRole('button', {name: 'Stats', exact: true}).click();
    await expect(page).toHaveURL(/#\/statistics$/);
    await expect(dialog).not.toBeVisible();
    await expect(opener).toBeFocused();
});

test('weekly runtime values are available by tabbing, including empty weeks', async ({page}) => {
    await page.goto('/#/statistics');
    const bars = page.getByRole('button', {name: /^Week \d+: 0 h$/});
    await expect(bars).toHaveCount(12);
    await bars.first().focus();
    for (let i = 0; i < 12; i++) {
        await expect(bars.nth(i)).toBeFocused();
        await expect(bars.nth(i)).toHaveAccessibleName(`Week ${i + 1}: 0 h`);
        await page.keyboard.press('Tab');
    }
});

test('weekly overview shows all saved minutes without presenting a mowing duration', async ({page}) => {
    const times = ['00:00', '05:30', '09:45', '21:00', '23:59', '09:15', '09:45'];
    await page.route('**/api/schedules', route => route.fulfill({json: {schedules: times.map((time, i) => ({
        id: `s${i}`, time, enabled: false, daysOfWeek: [1], createdAt: '2026-10-22T00:00:00Z',
    }))}}));
    await page.goto('/#/schedule');
    const overview = page.getByRole('region', {name: 'Weekly start times'});
    for (const time of times.filter(time => time !== '09:45')) await expect(overview.locator(`time[datetime="${time}"]`)).toBeVisible();
    await expect(overview.getByText('09:45', {exact: true})).toHaveCount(2);
    await expect(page.getByText(/Blocks mark scheduled starts.*60 minutes.*no fixed duration/)).toBeVisible();
});
