import {test,expect,type Page,type TestInfo} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
declare global { interface Window { __waves: string[]; } }
const base=SCENARIOS.find(s=>s.name==='mowing-area2-rtk-fixed')!;
interface Observation {
    consoles: {type: string; text: string}[];
    errors: string[];
    waves?: string[];
    emptyWaveHolders?: number;
    reducedMotion?: boolean;
    commands?: string[];
    posts?: string[];
    result?: unknown;
}
const observations: Record<string, Observation> = {};
async function setup(page:Page, info:TestInfo, scenario=base) {
 const consoles: Observation["consoles"] = [];const errors:string[]=[];
 page.on('console',m=>consoles.push({type:m.type(),text:m.text()}));
 page.on('pageerror',e=>errors.push(e.message));
 observations[info.testId]={consoles,errors};
 await page.addInitScript(()=>{
  localStorage.setItem('mowglinext.lang','en');
  window.__waves=[];
  new MutationObserver(records=>{for(const record of records)for(const node of record.addedNodes)if(node instanceof Element){
   const matches=[...(node.matches('.ant-wave')?[node]:[]),...node.querySelectorAll('.ant-wave')];
   for(const wave of matches)window.__waves.push(wave.outerHTML);
  }}).observe(document,{childList:true,subtree:true});
 });
 await installMockBackend(page,scenario);
 return {consoles,errors};
}
async function frames(page:Page){await page.evaluate(()=>new Promise<void>(r=>requestAnimationFrame(()=>requestAnimationFrame(()=>r()))));}
test.afterEach(async({page},info)=>{
 const item=observations[info.testId];
 if(item){item.waves=await page.evaluate(()=>window.__waves??[]).catch(()=>[]); item.emptyWaveHolders=await page.locator('button > div[style*="position: absolute"]').evaluateAll(nodes=>nodes.filter(n=>n.childNodes.length===0).length); item.reducedMotion=await page.evaluate(()=>matchMedia('(prefers-reduced-motion: reduce)').matches);await info.attach('observations',{body:JSON.stringify(item,null,2),contentType:'application/json'});expect(item.errors).toEqual([]);expect(JSON.stringify(item.consoles)).not.toContain('[antd: compatible]');}
});

test('actual DIG confirmation cancel refusal notification and reuse',async({page},info)=>{
 const scenario={...base,rest:{...base.rest,'/api/settings/yaml':{datum_lat:48.1,datum_lon:11.5}},topics:{...base.topics,highLevelStatus:{...(base.topics!.highLevelStatus as object),state:1,state_name:'DIG_OBSTRUCTION'},status:{...(base.topics!.status as object),dig_escalated:true,dig_escalated_distance_m:0,dig_escalated_required_distance_m:.5},map:{working_area_indices:[7],working_area:[{id:42,name:'Chosen',area:{points:[{x:-5,y:-5},{x:5,y:-5},{x:5,y:5},{x:-5,y:5}]},obstacles:[]}]}}};
 const {errors}=await setup(page,info,scenario);let rejected=true;const commands:string[]=[];
 await page.route(/\/api\/mowglinext\/call\//,async route=>{const name=new URL(route.request().url()).pathname.split('/').pop()!;if(name!=='get_lidar_ignore_corridors')commands.push(name);await route.fulfill(name==='clear_dig_escalation'&&rejected?{status:503,json:{error:'Release refused'}}:{json:{}});});
 await page.goto('/#/map');const trigger=page.getByRole('main').getByRole('button',{name:new RegExp(en.mapToolbar.mowArea)});
 const open=async()=>{await trigger.click();await page.getByRole('menuitem').click();await expect(page.getByRole('dialog')).toBeVisible();};
 await open();await page.getByRole('dialog').getByRole('button',{name:en.digObstructionAck.cancel,exact:true}).click();await expect(page.getByRole('dialog')).toBeHidden();expect(commands).toEqual([]);
 await open();await page.getByRole('dialog').getByRole('button',{name:en.digObstructionAck.ok,exact:true}).click();
 await expect(page.getByText('Release refused',{exact:true})).toBeVisible();await expect(trigger).not.toHaveClass(/ant-btn-loading/);
 await page.locator('.ant-notification-notice-close').click();await expect(page.getByText('Release refused',{exact:true})).toBeHidden();
 rejected=false;await open();await page.getByRole('dialog').getByRole('button',{name:en.digObstructionAck.ok,exact:true}).click();await expect(page.getByRole('dialog')).toBeHidden();await expect(trigger).not.toHaveClass(/ant-btn-loading/);
 expect(commands).toEqual(['clear_dig_escalation','clear_dig_escalation','start_in_area']);expect(errors).toEqual([]);await frames(page);
 expect(await page.evaluate(()=>window.__waves.length)).toBeGreaterThan(0);
 observations[info.testId].commands=commands;
 await page.screenshot({path:info.outputPath('dig.png')});
});

for(const acknowledgement of [{name:'success',body:{},text:en.fleetPage.peerAdded},{name:'native warning',body:{warning:'Native reciprocal warning'},text:'Native reciprocal warning'},{name:'neutral warning',body:{warning:false},text:en.fleetPage.peerAddWarningUnusable}]) test(`actual Fleet ${acknowledgement.name} appears expires and reopens`,async({page},info)=>{
 const {errors}=await setup(page,info,{...base,rest:{...base.rest,'/api/fleet/robots':[],'/api/fleet/coordination':{}}});
 const posts:string[]=[];await page.route(/\/api\/fleet\/peers$/,async route=>{posts.push(route.request().postData()!);await route.fulfill({json:acknowledgement.body});});
 await page.goto('/#/fleet');const input=page.getByRole('textbox',{name:en.fleetPage.addPeerTitle});const button=page.getByRole('button',{name:new RegExp(en.fleetPage.add+'$')});
 for(let i=0;i<2;i++){await input.fill('peer.local:4006');await button.click();await expect(page.getByText(acknowledgement.text,{exact:true})).toBeVisible();await expect(button).not.toHaveClass(/ant-btn-loading/);await expect(page.getByText(acknowledgement.text,{exact:true})).toBeHidden({timeout:10000});}
 expect(posts).toEqual(['{"address":"peer.local:4006"}','{"address":"peer.local:4006"}']);expect(errors).toEqual([]);
 observations[info.testId].posts=posts;
});

test('actual Statistics static feedback appears on refusal and success',async({page},info)=>{
 const {errors}=await setup(page,info);let failed=true;let removed=false;let deletes=0;
 const session={id:'fixture',start_time:'2026-10-18T12:00:00Z',end_time:'2026-10-18T12:10:00Z',duration_sec:600,area_index:1,coverage_percent:100,strips_completed:10,strips_skipped:0,status:'completed',errors:[]};
 await page.route(/\/api\/diagnostics\/sessions(?:\?.*)?$/,async route=>{
  if(route.request().method()==='DELETE'){deletes++;if(!failed)removed=true;await route.fulfill(failed?{status:503,json:{error:'Refused'}}:{json:{}});}
  else await route.fulfill({json:{sessions:removed?[]:[session],total:removed?0:1}});
 });
 await page.goto('/#/statistics');const trigger=page.getByRole('button',{name:new RegExp(en.statisticsPage.clear+'$')});await expect(trigger).toBeEnabled();
 const clear=async()=>{await trigger.click();const pop=page.locator('.ant-popconfirm');await expect(pop).toBeVisible();await pop.getByRole('button',{name:en.statisticsPage.clear,exact:true}).click();await expect(pop).toBeHidden();await frames(page);};
 await clear();expect(deletes).toBe(1);await expect(trigger).toBeEnabled();await expect(page.getByText(en.statisticsPage.failedToClearStatistics,{exact:true})).toBeVisible();await expect(page.getByText(en.statisticsPage.failedToClearStatistics,{exact:true})).toBeHidden();
 failed=false;await clear();await expect(trigger).toBeDisabled();expect(deletes).toBe(2);await expect(page.getByText(en.statisticsPage.statisticsCleared,{exact:true})).toBeVisible();await expect(page.getByText(en.statisticsPage.statisticsCleared,{exact:true})).toBeHidden();expect(errors).toEqual([]);
 observations[info.testId].result={deletes,removed,errorFeedbackPresent:true,successFeedbackPresent:true};
 await page.screenshot({path:info.outputPath('statistics.png')});
});


test('static APIs clean up and reuse roots under StrictMode', async ({page}, info) => {
    await setup(page, info);
    await page.addInitScript(() => {
        const roots = new Map<object, {id: number; active: boolean}>();
        const renderers = new Map<number, unknown>();
        Object.assign(window, {
            compatRootStats: () => ({active: [...roots.values()].filter(r => r.active).length, total: roots.size}),
            __REACT_DEVTOOLS_GLOBAL_HOOK__: {
                supportsFiber: true,
                renderers,
                inject: (renderer: unknown) => {
                    const id = renderers.size + 1;
                    renderers.set(id, renderer);
                    return id;
                },
                onCommitFiberRoot: (_id: number, root: {current: {memoizedState?: {element?: unknown} | null}}) => {
                    const record = roots.get(root) ?? {id: roots.size + 1, active: false};
                    record.active = root.current.memoizedState?.element != null;
                    roots.set(root, record);
                },
                onCommitFiberUnmount: () => {},
            },
        });
    });
    const origin = info.project.name === 'production' ? String(info.config.metadata.compatFixtureOrigin ?? '') : '';
    await page.goto(`${origin}/tests/e2e/fixtures/antd-react19-compat.html`);
    await expect(page.getByRole('button', {name: 'Static confirm', exact: true})).toBeVisible();
    const activeRoots = () => page.evaluate(() => (window as unknown as {compatRootStats: () => {active: number}}).compatRootStats().active);
    const life = () => page.evaluate(() => (window as unknown as {compatLifecycle: {setups: number; cleanups: number; confirmed: number; cancelled: number}}).compatLifecycle);
    await expect.poll(async () => (await life()).setups).toBe(info.project.name === 'production' ? 1 : 2);
    for (let cycle = 0; cycle < 3; cycle++) {
        await page.getByRole('button', {name: 'Static message', exact: true}).click();
        await expect(page.locator('.ant-message-notice-content')).toHaveText('Static message');
        await page.getByRole('button', {name: 'Close message', exact: true}).click();
        await expect(page.locator('.ant-message-notice')).toHaveCount(0);
        await page.getByRole('button', {name: 'Static notification', exact: true}).click();
        await expect(page.locator('.ant-notification-notice-message')).toHaveText('Static notification');
        await page.locator('.ant-notification-notice-close').click();
        await expect(page.locator('.ant-notification-notice')).toHaveCount(0);
        // Two global static holders intentionally persist; transient wave roots must finish.
        await expect.poll(activeRoots).toBe(3);
        await page.getByRole('button', {name: 'Static confirm', exact: true}).click();
        const dialog = page.getByRole('dialog');
        await expect(dialog).toBeVisible();
        if (cycle === 0) {
            await dialog.getByRole('button', {name: 'Update inside modal'}).click();
            await expect(dialog).toContainText('Updated content');
            await dialog.getByRole('button', {name: 'OK', exact: true}).click();
        } else if (cycle === 1) {
            await dialog.getByRole('button', {name: 'Cancel', exact: true}).click();
        } else {
            await dialog.getByRole('button', {name: 'Destroy inside modal'}).click();
        }
        await expect(dialog).toBeHidden();
        await expect(page.locator('.ant-modal-root')).toHaveCount(0);
        await expect.poll(activeRoots).toBe(3);
    }
    expect(await life()).toMatchObject({confirmed: 1, cancelled: 1});
    await page.getByRole('button', {name: 'Unmount fixture'}).click();
    await expect(page.locator('#root')).toBeEmpty();
    await expect.poll(activeRoots).toBe(2);
    const lifecycle = await life();
    expect(lifecycle.cleanups).toBe(lifecycle.setups);
    observations[info.testId].result = {lifecycle, activeRootsAfterUnmount: await activeRoots()};
});

test('wave renders then cleans up without permanent button changes', async ({page}, info) => {
    await setup(page, info);
    const origin = info.project.name === 'production' ? String(info.config.metadata.compatFixtureOrigin ?? '') : '';
    await page.goto(`${origin}/tests/e2e/fixtures/antd-react19-compat.html`);
    const button = page.getByRole('button', {name: 'Wave baseline', exact: true});
    await expect(button).toBeVisible();
    await page.evaluate(() => document.fonts.ready);
    const signature = () => button.evaluate(node => {
        const rect = node.getBoundingClientRect();
        const css = getComputedStyle(node);
        return {x: rect.x, y: rect.y, width: rect.width, height: rect.height,
            styles: Object.fromEntries(['background-color', 'color', 'border-radius', 'border-width', 'padding', 'font-size', 'font-family', 'box-shadow'].map(key => [key, css.getPropertyValue(key)]))};
    });
    await page.mouse.move(0, 0);
    const before = await signature();
    await button.screenshot({path: info.outputPath('button-before.png')});
    await button.click();
    await expect(page.locator('.ant-wave')).toHaveCount(1);
    await page.screenshot({path: info.outputPath('wave-active.png')});
    await expect(page.locator('.ant-wave')).toHaveCount(0);
    await expect(button.locator('div[style*="position: absolute"]')).toHaveCount(0);
    await page.mouse.move(0, 0);
    await expect.poll(signature).toEqual(before);
    await button.screenshot({path: info.outputPath('button-after.png')});
    observations[info.testId].result = {before, after: await signature(), identicalButtonGeometryAndStyles: true};
});
