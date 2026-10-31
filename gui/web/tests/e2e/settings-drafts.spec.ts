import {expect, test, type Page} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};

async function fixture(page: Page) {
    await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
    await installMockBackend(page, SCENARIOS.find(s => s.name === 'idle-docked-full')!);
    let yaml: Record<string, unknown> = {robot_name: 'Stored mower', mower_model: 'YardForce500', datum_lat: 48.1, datum_lon: 11.5};
    let notifications = {enabled: false, channel: 'ntfy', language: 'en', events: {}, eventKinds: [], channels: ['ntfy']};
    const writes: unknown[] = [], puts: unknown[] = [];
    await page.route('**/api/settings/yaml', async route => {
        if (route.request().method() === 'POST') {
            const body = route.request().postDataJSON();
            writes.push(body);
            yaml = {...yaml, ...body};
        }
        await route.fulfill({json: yaml});
    });
    await page.route('**/api/notifications/settings', async route => {
        if (route.request().method() === 'PUT') {
            const body = route.request().postDataJSON();
            puts.push(body);
            notifications = {...notifications, ...body};
        }
        await route.fulfill({json: notifications});
    });
    return {writes, puts};
}
const save = (page: Page, count: number) => page.getByRole('button', {name: `Save (${count} changes)`});
const push = (page: Page) => page.getByRole('switch', {name: en.settingsNotifications.title_section});
const section = (page: Page, name: string) => page.getByRole('menuitem', {name}).click();
const home = (page: Page) => page.getByRole('navigation').getByRole('button', {name: 'Home', exact: true}).click();

test('YAML page leave requires discard; cancellation and section navigation preserve the draft', async ({page}) => {
    const h = await fixture(page);
    await page.goto('/#/settings');
    await page.locator('#setting-robot_name').fill('Draft mower');
    await section(page, 'Notifications');
    await section(page, 'Hardware');
    await expect(page.locator('#setting-robot_name')).toHaveValue('Draft mower');
    await home(page);
    await expect(page.getByRole('dialog', {name: en.settingsPage.leaveTitle})).toBeVisible();
    await page.getByRole('button', {name: en.settingsPage.keepEditing}).click();
    await expect(page.locator('#setting-robot_name')).toHaveValue('Draft mower');
    await expect(save(page, 1)).toBeVisible();
    await home(page);
    await page.getByRole('button', {name: en.settingsPage.discardAndLeave}).click();
    await expect(page.locator('.settings-page')).toHaveCount(0);
    await page.goto('/#/settings');
    await expect(page.locator('#setting-robot_name')).toHaveValue('Stored mower');
    expect(h.writes).toEqual([]);
});

test('integration draft survives section navigation and search with a combined dirty count', async ({page}) => {
    const h = await fixture(page);
    await page.goto('/#/settings?section=notifications');
    await push(page).click();
    await section(page, 'Hardware');
    await expect(save(page, 1)).toBeVisible();
    await page.locator('#setting-robot_name').fill('Draft mower');
    await expect(save(page, 2)).toBeVisible();
    await expect(page.locator('.settings-page .ant-badge-count')).toHaveAttribute('title', '2');
    const search = page.getByRole('textbox', {name: en.settingsPage.searchSettingPlaceholder});
    await search.fill('no such setting xyz');
    await expect(save(page, 2)).toBeVisible();
    await search.fill('');
    await section(page, 'Notifications');
    await expect(push(page)).toBeChecked();
    await home(page);
    await page.getByRole('button', {name: en.settingsPage.keepEditing}).click();
    await expect(push(page)).toBeChecked();
    await page.getByRole('button', {name: en.settingsPage.revert}).click();
    await expect(push(page)).not.toBeChecked();
    await expect(page.getByRole('button', {name: 'Saved'})).toBeVisible();
    expect(h.puts).toEqual([]);
    expect(h.writes).toEqual([]);
});

test('explicit Save persists YAML and hidden Notifications across page navigation and reload', async ({page}) => {
    const h = await fixture(page);
    await page.goto('/#/settings?section=notifications');
    await push(page).click();
    await section(page, 'Hardware');
    await page.locator('#setting-robot_name').fill('Saved mower');
    await save(page, 2).click();
    await expect(page.getByRole('button', {name: 'Saved'})).toBeVisible();
    expect(h.writes).toEqual([{robot_name: 'Saved mower'}]);
    expect(h.puts).toEqual([{enabled: true}]);
    await home(page);
    await expect(page.locator('.settings-page')).toHaveCount(0);
    await page.goto('/#/settings');
    await expect(page.locator('#setting-robot_name')).toHaveValue('Saved mower');
    await section(page, 'Notifications');
    await expect(push(page)).toBeChecked();
    await page.reload();
    await expect(push(page)).toBeChecked();
});

for (const kind of ['yaml', 'notifications']) {
    test(`${kind}: refresh and close warn before discarding an unsaved draft`, async ({page}) => {
        await fixture(page);
        await page.goto(kind === 'yaml' ? '/#/settings' : '/#/settings?section=notifications');
        if (kind === 'yaml') await page.locator('#setting-robot_name').fill('Draft mower');
        else await push(page).click();
        await expect(save(page, 1)).toBeVisible();
        const refreshDialog = page.waitForEvent('dialog');
        const reload = page.evaluate(() => window.location.reload());
        const dialog = await refreshDialog;
        expect(dialog.type()).toBe('beforeunload');
        await dialog.dismiss();
        await reload;
        await expect(save(page, 1)).toBeVisible();
        const closeDialog = page.waitForEvent('dialog');
        await page.close({runBeforeUnload: true});
        const close = await closeDialog;
        expect(close.type()).toBe('beforeunload');
        await close.dismiss();
        await expect(save(page, 1)).toBeVisible();
        const discardDialog = page.waitForEvent('dialog');
        const discardReload = page.reload();
        await (await discardDialog).accept();
        await discardReload;
        await expect(page.getByRole('button', {name: 'Saved'})).toBeVisible();
        if (kind === 'yaml') await expect(page.locator('#setting-robot_name')).toHaveValue('Stored mower');
        else await expect(push(page)).not.toBeChecked();
    });
}

test('onboarding keeps calibrated draft on save failure and retries before readiness', async ({page}) => {
    await fixture(page);
    const writes: Record<string, unknown>[] = [];
    let fail = true;
    await page.route('**/api/settings/yaml', async (route) => {
        if (route.request().method() !== 'POST') return route.fallback();
        const body = route.request().postDataJSON();
        writes.push(body);
        if (body.imu_yaw === 0.25 && fail) return route.fulfill({status: 500, json: {error: 'Fixture save refused'}});
        return route.fallback();
    });
    await page.route('**/api/calibration/imu-yaw', route => route.fulfill({json: {
        success: true, message: 'Complete', imu_yaw_rad: .25, imu_yaw_deg: 14.32, samples_used: 250, std_dev_deg: .2,
    }}));
    const followups: string[] = [];
    page.on('request', request => {
        if (request.method() === 'POST' && /restart|\/apply|\/settings\/status/.test(request.url())) followups.push(request.url());
    });
    await page.goto('/#/onboarding?step=firmware');
    await page.getByRole('button', {name: en.onboardingPage.skipAlreadyFlashed}).click();
    for (const step of ['stepNtrip', 'stepGps', 'stepDatum', 'stepSensors'] as const) {
        await expect(page.getByRole('region', {name: en.onboardingPage[step]})).toBeVisible();
        await page.getByRole('button', {name: step === 'stepDatum' ? en.onboardingPage.saveAndFinish : en.onboardingPage.next}).click();
    }
    const calibration = page.getByRole('region', {name: en.onboardingPage.stepCalibration});
    await expect(calibration).toBeVisible();
    await page.getByRole('button', {name: en.onboardingPage.startImuYawCalibration}).click();
    await page.getByRole('button', {name: en.onboardingPage.applyToSettings}).click();
    await page.getByRole('button', {name: en.onboardingPage.next}).click();
    await expect(page.getByText('Fixture save refused')).toBeVisible();
    await expect(calibration).toBeVisible();
    expect(writes.at(-1)?.imu_yaw).toBe(.25);
    expect(followups).toEqual([]);
    fail = false;
    await page.getByRole('button', {name: en.onboardingPage.next}).click();
    await expect(page.getByRole('region', {name: en.onboardingPage.stepComplete})).toBeVisible();
    expect(writes.at(-1)?.imu_yaw).toBe(.25);
    expect(followups).toEqual([]);
});

test('edits during an integration save survive hiding and returning to the section', async ({page}) => {
    await fixture(page);
    let release!: () => void;
    const pending = new Promise<void>(resolve => {release = resolve;});
    const puts: unknown[] = [];
    await page.route('**/api/notifications/settings', async route => {
        if (route.request().method() !== 'PUT') return route.fallback();
        puts.push(route.request().postDataJSON());
        if (puts.length === 1) await pending;
        return route.fallback();
    });
    await page.goto('/#/settings?section=notifications');
    await push(page).click();
    await save(page, 1).click();
    await expect.poll(() => puts.length).toBe(1);
    await push(page).click();
    await section(page, 'Hardware');
    release();
    await expect(save(page, 1)).not.toHaveClass(/ant-btn-loading/);
    await section(page, 'Notifications');
    await expect(push(page)).not.toBeChecked();
    await save(page, 1).click();
    await expect(page.getByRole('button', {name: 'Saved'})).toBeVisible();
    expect(puts).toEqual([{enabled: true}, {enabled: false}]);
});

test('failed GPS step save does not restart the container and retries the same draft', async ({page}) => {
    await fixture(page);
    let fail = false;
    const writes: Record<string, unknown>[] = [];
    const restarts: string[] = [];
    await page.route('**/api/settings/yaml', route => {
        if (route.request().method() !== 'POST') return route.fallback();
        writes.push(route.request().postDataJSON());
        return fail ? route.fulfill({status: 500, json: {error: 'GPS save refused'}}) : route.fallback();
    });
    await page.route('**/api/settings/gnss/restart', route => {
        restarts.push(route.request().url());
        return route.fulfill({json: {}});
    });
    await page.goto('/#/onboarding?step=firmware');
    await page.getByRole('button', {name: en.onboardingPage.skipAlreadyFlashed}).click();
    await page.getByRole('button', {name: en.onboardingPage.next}).click();
    const gps = page.getByRole('region', {name: en.onboardingPage.stepGps});
    await expect(gps).toBeVisible();
    await page.locator('[data-setting-key="gnss_serial_baud"] .ant-select-selector').click();
    await page.getByTitle('115200', {exact: true}).click();
    fail = true;
    await page.getByRole('button', {name: en.onboardingPage.next}).click();
    await expect(page.getByText('GPS save refused')).toBeVisible();
    await expect(gps).toBeVisible();
    expect(writes.at(-1)?.gnss_serial_baud).toBe(115200);
    expect(restarts).toEqual([]);
    fail = false;
    await page.getByRole('button', {name: en.onboardingPage.next}).click();
    await expect(page.getByRole('region', {name: en.onboardingPage.stepDatum})).toBeVisible();
    expect(restarts).toHaveLength(1);
    expect(writes.at(-1)?.gnss_serial_baud).toBe(115200);
});

test('Revert while a hidden integration is saving keeps the response owner alive', async ({page}) => {
    await fixture(page);
    let release!: () => void;
    const pending = new Promise<void>(resolve => {release = resolve;});
    let started = false;
    await page.route('**/api/notifications/settings', async route => {
        if (route.request().method() !== 'PUT') return route.fallback();
        started = true;
        await pending;
        return route.fallback();
    });
    await page.goto('/#/settings?section=notifications');
    await push(page).click();
    await save(page, 1).click();
    await expect.poll(() => started).toBe(true);
    await section(page, 'Hardware');
    await page.getByRole('button', {name: en.settingsPage.revert}).click();
    await section(page, 'Notifications');
    await expect(push(page)).not.toBeChecked();
    release();
    await expect(push(page)).toBeChecked();
    await expect(page.getByRole('button', {name: 'Saved'})).not.toHaveClass(/ant-btn-loading/);
});

test('mobile preserves unapplied secret text and does not write it until Apply and Save', async ({page}) => {
    await page.setViewportSize({width: 390, height: 844});
    const h = await fixture(page);
    await page.goto('/#/settings?section=notifications');
    const token = page.getByLabel(en.settingsNotifications.ntfyToken, {exact: true});
    await token.fill('fixture-secret');
    await expect(save(page, 1)).toBeVisible();
    const search = page.getByRole('textbox', {name: en.settingsPage.searchSettingPlaceholder});
    await search.fill('Robot Name');
    await expect(page.locator('#setting-robot_name')).toBeVisible();
    await expect(save(page, 1)).toBeVisible();
    await save(page, 1).click();
    await expect(save(page, 1)).not.toHaveClass(/ant-btn-loading/);
    expect(h.puts).toEqual([]);
    await search.fill('');
    await expect(token).toHaveValue('fixture-secret');
    await home(page);
    await expect(page.getByRole('dialog', {name: en.settingsPage.leaveTitle})).toBeVisible();
    await page.getByRole('button', {name: en.settingsPage.keepEditing}).click();
    await token.press('Enter');
    await expect(token).toHaveValue('');
    await page.getByRole('button', {name: /Save \(/}).click();
    await expect(page.getByRole('button', {name: 'Saved'})).toBeVisible();
    expect(h.puts).toEqual([{ntfyToken: 'fixture-secret'}]);
});
