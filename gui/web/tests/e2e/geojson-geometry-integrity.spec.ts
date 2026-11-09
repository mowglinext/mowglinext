import {expect, test, type Page} from '@playwright/test';
import {readFile} from 'node:fs/promises';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};
import {isJsonObject} from '../../src/utils/json';

const ring = (x: number, y: number, size: number) => [[x,y],[x+size,y],[x+size,y+size],[x,y+size],[x,y]];
const outer = ring(11.5,48.1,.0002);
function source(holes: number) {
    return {type:'FeatureCollection',features:[{type:'Feature',id:holes ? 'incoming' : 0,
        properties:{feature_type:'workarea',name:holes ? 'Rejected holes' : 'Local draft',mowing_order:0},
        geometry:{type:'Polygon',coordinates:[outer,...Array.from({length:holes},(_,i)=>ring(11.50002+i*.00008,48.10002,.00002))]}}]};
}

async function upload(page: Page, locale: typeof en | typeof fr, value: unknown) {
    await page.getByRole('main').getByRole('button',{name:locale.mapToolbarMobile.more,exact:true}).click();
    const chooser = page.waitForEvent('filechooser');
    await page.getByRole('menuitem').filter({hasText:locale.mapToolbarMobile.uploadGeojson}).click();
    await (await chooser).setFiles({name:'fictional-map.geojson',mimeType:'application/geo+json',buffer:Buffer.from(JSON.stringify(value))});
}

for (const language of ['en','fr']) {
    for (const holes of [0,1,2]) {
        test(`GeoJSON ${holes} holes: ${holes ? 'refuse without losing the draft' : 'save a simple polygon once'} in ${language}`, async ({page})=>{
            const locale = language === 'fr' ? fr : en;
            await page.setViewportSize({width:390,height:900});
            await page.addInitScript(lang=>localStorage.setItem('mowglinext.lang',lang),language);
            await installMockBackend(page,{...SCENARIOS[0],rest:{'/api/settings/yaml':{datum_lat:48.1,datum_lon:11.5}}});
            // The existing explicit Edit action makes a server-side backup first.
            await page.route('**/api/map-backups',route=>route.fulfill({json:{}}));
            const writes: string[]=[];
            let saved:unknown;
            page.on('request',request=>{
                const path=new URL(request.url()).pathname;
                if (request.method()==='GET' || path==='/api/mowglinext/call/get_lidar_ignore_corridors') return;
                if (/^\/api\/(?:mowglinext\/(?:map|call|publish)|settings\/yaml$|import\/)/.test(path)) writes.push(path);
            });
            await page.route('**/api/mowglinext/map',async route=>{
                if (route.request().method()!=='PUT') {await route.fallback();return;}
                saved=route.request().postDataJSON(); await route.fulfill({json:{}});
            });
            await page.goto('/#/map');
            await page.getByRole('button',{name:locale.mapToolbarMobile.editMap,exact:true}).click();
            const save=page.getByRole('button',{name:locale.mapToolbarMobile.save,exact:true});
            await expect(save).toBeVisible();
            await upload(page,locale,source(0));
            await expect(page.locator('.ant-notification-notice-success')).toContainText(locale.mapFiles.uploadSuccess);
            await expect(save).toHaveClass(/ant-btn-dangerous/);
            expect(writes).toEqual([]);
            if (holes) {
                await upload(page,locale,source(holes));
                await expect(page.locator('.ant-notification-notice-error')).toContainText(locale.mapFiles.unsupportedGeometry);
                await expect(save).toBeVisible(); await expect(save).toHaveClass(/ant-btn-dangerous/);
                await page.getByRole('main').getByRole('button',{name:locale.mapToolbarMobile.more,exact:true}).click();
                const downloaded=page.waitForEvent('download');
                await page.getByRole('menuitem').filter({hasText:locale.mapToolbarMobile.downloadGeojson}).click();
                const path=await (await downloaded).path();
                if (!path) throw new Error('Missing export');
                const exported:unknown=JSON.parse(await readFile(path,'utf8'));
                expect(exported).toMatchObject({type:'FeatureCollection',features:[{id:'0',properties:{name:'Local draft'},geometry:{type:'Polygon',coordinates:[outer]}}]});
                expect(JSON.stringify(exported)).not.toContain('Rejected holes');
                expect(writes).toEqual([]);
            } else {
                await save.click();
                await expect.poll(()=>writes.length).toBe(1);
                expect(writes).toEqual(['/api/mowglinext/map']);
                if (!isJsonObject(saved) || !Array.isArray(saved.areas)) throw new Error('Invalid save');
                expect(saved.areas).toHaveLength(1);
                expect(saved).toMatchObject({areas:[{is_navigation_area:false,area:{name:'Local draft',area:{points:expect.any(Array)}}}]});
                await expect(save).not.toBeVisible();
                await expect(page.locator('.ant-notification-notice-success').filter({hasText:locale.mapFiles.areaSaved})).toBeVisible();
            }
        });
    }
}
