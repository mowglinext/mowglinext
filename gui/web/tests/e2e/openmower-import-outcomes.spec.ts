import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import {isJsonObject} from '../../src/utils/json';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};

const source = {areas: [{id: 'B', properties: {name: 'Fictional B', type: 'mow'},
    outline: [{x: 0, y: 0}, {x: 2, y: 0}, {x: 2, y: 2}]}],
    docking_stations: [{id: 'dock-B', position: {x: 0, y: 0}, heading: 0}]};
const preview = {mowing_areas: 1, navigation_areas: 0, obstacles: 0, orphan_obstacles: 0,
    datum_shift_east_m: 0, datum_shift_north_m: 0, warnings: [], areas: [],
    dock_pose: {x: 0, y: 0, yaw_rad: 0}, applied: false, mn_datum_configured: true,
    map_status: 'unchanged', dock_status: 'not_attempted'};

for (const language of ['en', 'fr']) {
    for (const outcome of ['rejected', 'unconfirmed', 'not_attempted', 'confirmed']) {
        test(`OpenMower ${outcome} is explicit and never retries in ${language}`, async ({page}) => {
            const locale = language === 'fr' ? fr : en;
            await page.addInitScript(lang => localStorage.setItem('mowglinext.lang', lang), language);
            await installMockBackend(page, {...SCENARIOS[0], rest: {'/api/settings/yaml': {datum_lat: 48.1, datum_lon: 11.5}}});
            const writes: string[] = [];
            page.on('request', request => {
                const path = new URL(request.url()).pathname;
                // This existing ROS service is a read despite its HTTP POST.
                if (path === '/api/mowglinext/call/get_lidar_ignore_corridors') return;
                if (request.method() !== 'GET' && /^\/api\/(settings\/yaml$|mowglinext\/(?:map|call|publish))/.test(path)) writes.push(path);
            });
            let applies = 0;
            let appliedBody: unknown;
            await page.route('**/api/import/openmower', async route => {
                const body: unknown = route.request().postDataJSON();
                if (!isJsonObject(body) || body.apply !== true) {await route.fulfill({json: preview}); return;}
                applies++;
                appliedBody = body;
                await route.fulfill(outcome === 'confirmed'
                    ? {json: {...preview, applied: true, map_status: 'persisted', dock_status: 'confirmed'}}
                    : {status: 500, json: {error: 'fixture service result', map_status: 'persisted', dock_status: outcome}});
            });
            await page.goto('/#/map');
            await page.getByRole('button').filter({hasText: locale.mapToolbar.more}).click();
            const chooser = page.waitForEvent('filechooser');
            await page.getByRole('menuitem').filter({hasText: locale.mapToolbar.importFromOpenMower}).click();
            await (await chooser).setFiles({name: 'fictional-map.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(source))});
            const dialog = page.getByRole('dialog', {name: locale.mapImportOpenMower.title, exact: true});
            await expect(dialog).toBeVisible();
            expect(applies).toBe(0); expect(writes).toEqual([]);
            if (outcome !== 'confirmed') {
                const latitude = dialog.getByPlaceholder('OM_DATUM_LAT'), longitude = dialog.getByPlaceholder('OM_DATUM_LONG');
                await latitude.fill('0'); await latitude.blur();
                await expect(latitude).toBeEnabled(); // The existing blur starts a read-only reprojection.
                await longitude.fill('0'); await longitude.blur();
                await expect(longitude).toBeEnabled();
                await expect(latitude).toHaveValue('0.000000000'); await expect(longitude).toHaveValue('0.000000000');
                await expect(dialog.getByRole('checkbox')).toHaveCount(0); // Existing robot datum cannot be overwritten.
            }
            const apply = dialog.getByRole('button', {name: locale.mapImportOpenMower.applyReplaces, exact: true});
            await expect(apply).toBeEnabled(); await apply.click();
            if (outcome === 'confirmed') {
                await expect(dialog).not.toBeVisible();
                await expect(page.locator('.ant-notification-notice-success')).toContainText(locale.mapFiles.openMowerMapImported);
            } else {
                await expect(dialog.locator('.ant-alert-error')).toContainText(outcome === 'rejected'
                    ? locale.mapImportOpenMower.dockRejected : outcome === 'not_attempted'
                        ? locale.mapImportOpenMower.dockNotAttempted : locale.mapImportOpenMower.dockUnconfirmed);
                if (outcome === 'not_attempted') {
                    await expect(dialog.locator('.ant-alert-error')).not.toContainText(locale.mapImportOpenMower.dockUnconfirmed);
                }
                await expect(dialog.getByPlaceholder('OM_DATUM_LAT')).toHaveValue('0.000000000');
                await expect(dialog.getByPlaceholder('OM_DATUM_LONG')).toHaveValue('0.000000000');
                await expect(page.locator('.ant-notification-notice-success')).toHaveCount(0);
                await dialog.getByRole('button', {name: locale.mapImportOpenMower.cancel, exact: true}).click();
                await expect(dialog).not.toBeVisible();
            }
            expect(applies).toBe(1); expect(writes).toEqual([]);
            if (outcome !== 'confirmed') expect(appliedBody).toEqual(expect.objectContaining({om_datum_lat: 0, om_datum_lon: 0}));
        });
    }
}
