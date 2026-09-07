import {expect, test, type Page} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';

const source = {repository:'mowglinext/mowglinext',track:'dev',branch:'dev'};
const familyMap = {mowgli:'mowgli-ros2',gui:'mowglinext-gui',gps:'gps',lidar:'lidar-ldlidar',camera:'camera'};
const contracts = Object.fromEntries(Object.values(familyMap).map(f=>[f,`${f}-contract-1`]));
function release(id:string, track='dev', tag='') {
    return {id,source:{...source,track,branch:track==='stable'?'main':'dev'},release_tag:tag,revision:(id.includes('new')?'a':'b').repeat(40),published_at:'2026-09-07T09:00:00Z',
        layout:1,data_schema:1,updater_api:1,maintenance_api:1,firmware_protocol:6,component_compatibility:contracts,
        service_choices:[{service:'mowgli',image:'mowgli-ros2'},{service:'gui',image:'mowglinext-gui'},{service:'gps',image:'gps',when:{gnss:'universal'}},{service:'lidar',image:'lidar-ldlidar',when:{lidar:'ldlidar'}}],
        images:Object.fromEntries(Object.values(familyMap).map(f=>[f,{repository:`ghcr.io/${source.repository}/${f}`,platforms:{'linux/arm64':{manifest:'sha256:'+'1'.repeat(64)}}}])),
        updater:{'linux/arm64':{version:'updater-current'}}};
}
function fixture(track='dev', mixed=false, expanded=false) {
    const base=release('release-base',track,'v1.2.0');const next=release('release-new',track,'v1.3.0');const alternative=release('release-alternative',track,'v1.2.1');
    if(expanded) {base.service_choices.push({service:'camera',image:'camera'});next.service_choices.push({service:'camera',image:'camera'});}
    const names=expanded?['mowgli','gui','gps','lidar','camera']:['mowgli','gui','gps'];
    const components=Object.fromEntries(names.map(name=>[name,{name:`mowgli-${name}`,family:familyMap[name as keyof typeof familyMap],reference:`ghcr.io/${source.repository}/${familyMap[name as keyof typeof familyMap]}:dev`,version:track==='stable'?(mixed&&['mowgli','gui'].includes(name)?'v1.2.1':'v1.2.0'):'dev',revision:'1944f97d'.padEnd(40,'a'),image:`sha256:installed-${name}`,healthy:true,healthcheck:false}]));
    return {api:1,capabilities:['component-overrides','declared-services','release-compose','service-version-overrides'],
        runtime:{identity:mixed?'mixed':'matched',health:'healthy',checked_at:'2026-09-07T09:30:00Z',selection:{gnss:'universal',lidar:expanded?'ldlidar':'none'},components},
        agent:{version:'updater-current',revision:'1944f97d',platform:'linux/arm64'},trusted_repositories:[source.repository],
        state:{policy:{source:base.source,interval_hours:4,pinned:false},installed_policy:{source:base.source,interval_hours:4,pinned:true},active:base,
            overrides:mixed?{gui:alternative,mowgli:alternative}:{},last_check:'2026-09-07T09:30:00Z',next_check:'2026-09-07T13:35:00Z',last_success:'2026-09-07T09:30:00Z',releases:[next,base,alternative],notices:[],history:[]}};
}
const inventory={docker_available:true,server:{version:'dev'},components:[
    ...['mowgli','gui','gps','lidar','camera'].map(name=>({name:`mowgli-${name}`,component:name==='mowgli'?'robot':name,version:'dev',revision:'1944f97d',state:'running',image:`ghcr.io/${source.repository}/${familyMap[name as keyof typeof familyMap]}:dev`,image_id:`sha256:installed-${name}`})),
    {name:'mowgli-mqtt',component:'mqtt',version:'2.0.22',state:'running',image:'eclipse-mosquitto:2.0.22'},
]};
async function open(page:Page, data:ReturnType<typeof fixture>, mobile=false) {
    await page.setViewportSize(mobile?{width:390,height:844}:{width:1440,height:1800});
    const names=new Set(Object.values(data.runtime.components).map(c=>c.name));
    await installMockBackend(page,{...SCENARIOS[0],rest:{'/api/system/updater/state':data,'/api/system/versions':{...inventory,components:inventory.components.filter(c=>c.component==='mqtt'||names.has(c.name)).map(c=>({...c,version:Object.values(data.runtime.components).find(r=>r.name===c.name)?.version??c.version}))}}});
    const posts:{path:string;body:Record<string,unknown>}[]=[];const errors:string[]=[];
    page.on('pageerror',e=>errors.push(e.message));
    page.on('request',r=>{if(r.method()==='POST'&&r.url().includes('/system/updater/'))posts.push({path:new URL(r.url()).pathname,body:r.postDataJSON()});});
    await page.goto('/#/settings?section=updates');await expect(page.getByTestId('host-updater')).toBeVisible();
    return {panel:page.getByTestId('host-updater'),posts,errors};
}
async function advanced(page:Page) {await page.locator('.ant-segmented').getByText('Advanced',{exact:true}).click();}
async function choose(page:Page,name:string,text:string) {
    await page.getByRole('combobox',{name,exact:true}).locator('..').locator('..').click();
    await page.locator('.ant-select-dropdown:visible').last().locator('.ant-select-item-option-content').getByText(text,{exact:true}).click();
    await expect(page.locator('.ant-select-dropdown:visible')).toHaveCount(0);
}
async function shot(page:Page,name:string,mobile:boolean,focus?:string) {
    if(mobile&&focus)await page.getByTestId(focus).evaluate(el=>el.scrollIntoView({block:'start'}));
    await page.waitForTimeout(250);
    expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
    await page.screenshot({path:`tests/e2e/.artifacts/${name}-${mobile?'mobile':'desktop'}.png`,fullPage:true,animations:'disabled'});
}
function plan(base:ReturnType<typeof release>, overrides:Record<string,ReturnType<typeof release>>={}) {
    const names=[...new Set(['mowgli','gui','gps',...Object.keys(overrides)])];
    return {id:'review-plan',target:base,overrides,previous:Object.fromEntries(names.map(n=>[n,'sha256:old-'+n])),images:Object.fromEntries(names.map(n=>[n,'sha256:new-'+n])),expires_at:'2026-09-07T09:45:00Z'};
}

for(const track of ['dev','stable'])for(const mobile of [false,true])test(`${track} release flow ${mobile?'mobile':'desktop'}`,async({page})=>{
    const data=fixture(track);const {panel,posts,errors}=await open(page,data,mobile);const prefix=track==='dev'?'host-updater':'host-updater-production';
    await expect(panel.getByText('Installed stack',{exact:true})).toBeVisible();await expect(panel.getByText('MQTT',{exact:true})).toBeVisible();
    await expect(panel.getByRole('combobox')).toHaveCount(0);await expect(page.getByTestId('update-checks')).toHaveCount(0);
    await expect(panel.getByText('Installed',{exact:true})).toBeVisible();
    await expect(panel.getByText('After update',{exact:false})).toHaveCount(0);
    if(track==='stable')await expect(panel.locator('.available-release')).toHaveText('v1.3.0');
    await expect(panel.getByRole('button',{name:'Review changes',exact:true})).toBeInViewport();
    if(!mobile)await expect(page.getByTestId('running-version-summary')).toContainText(track==='stable'?'v1.2.0':'bbbbbbbb');
    await shot(page,prefix,mobile);
    await page.route('**/api/system/updater/plan',r=>r.fulfill({json:plan(data.state.releases[0])}));
    await panel.getByRole('button',{name:'Review changes',exact:true}).click();await expect(page.getByRole('dialog')).toBeVisible();
    await expect(page.getByRole('dialog').getByRole('button',{name:'Install reviewed deployment'})).toBeInViewport();
    await shot(page,prefix+'-review',mobile);await page.getByRole('dialog').getByRole('button',{name:'Cancel',exact:true}).click();
    expect(posts).toEqual([{path:'/api/system/updater/plan',body:{deployment:'release-new',pinned:true}}]);
    await advanced(page);await expect(panel.getByRole('combobox',{name:'Robot software version',exact:true})).toBeEnabled();
    await expect(panel.getByRole('combobox',{name:'GPS version',exact:true})).toBeEnabled();
    await expect(panel.getByRole('combobox',{name:'Repository',exact:true})).not.toBeVisible();
    await shot(page,prefix+'-advanced',mobile,'stack-mowgli');expect(errors).toEqual([]);
});

for(const mobile of [false,true])test(`unpublished source ${mobile?'mobile':'desktop'}`,async({page})=>{
    const data=fixture();data.runtime.identity='custom';data.state.releases=[];data.state.active=undefined as never;data.state.installed_policy=undefined as never;
    const {panel,posts}=await open(page,data,mobile);await advanced(page);
    await expect(panel.getByRole('combobox',{name:'Release version',exact:true})).toBeDisabled();
    for(const name of ['Robot software','Web interface','GPS'])await expect(panel.getByRole('combobox',{name:name+' version',exact:true})).toBeDisabled();
    await expect(panel.getByRole('button',{name:'Review changes',exact:true})).toBeDisabled();
    await expect(panel.getByRole('button',{name:'Check for updates',exact:true})).toBeEnabled();
    await expect(panel.getByText('No installable build published for this source.')).toBeVisible();
    await shot(page,'host-updater-no-versions',mobile,'stack-mowgli');expect(posts).toEqual([]);
});

for(const mobile of [false,true])test(`stack membership review ${mobile?'mobile':'desktop'}`,async({page})=>{
    const data=fixture('stable');const {panel,posts}=await open(page,data,mobile);
    await page.route('**/api/system/updater/plan',r=>r.fulfill({json:{...plan(data.state.releases[0]),stack:{selection:{options:{gnss:'universal',lidar:'none'}},changes:[{service:'mowgli',action:'update'},{service:'gui',action:'update'},{service:'gps',action:'keep'},{service:'navigation-helper',action:'add'},{service:'legacy-helper',action:'remove'},{service:'mqtt',action:'unmanaged'}]}}}));
    await panel.getByRole('button',{name:'Review changes',exact:true}).click();const dialog=page.getByRole('dialog');
    await expect(dialog.getByText('GPS: On',{exact:true})).toBeVisible();await expect(dialog.getByText('LiDAR: Off',{exact:true})).toBeVisible();
    for(const text of ['Add','Remove','Keep · local'])await expect(dialog.getByText(text,{exact:true})).toBeVisible();
    await expect(dialog.getByRole('button',{name:'Install reviewed deployment'})).toBeInViewport();await shot(page,'host-updater-stack-review',mobile);
    expect(posts.map(p=>p.path)).toEqual(['/api/system/updater/plan']);
});

for(const mobile of [false,true])test(`independent components and matched reset ${mobile?'mobile':'desktop'}`,async({page})=>{
    const data=fixture('stable',true,true);const alternative=data.state.releases[2];const incompatible={...release('incompatible','stable','v2.0.0'),component_compatibility:{}};data.state.releases.push(incompatible);
    const {panel,posts,errors}=await open(page,data,mobile);await shot(page,'host-updater-mixed',mobile);await advanced(page);
    for(const name of ['Robot software','Web interface','GPS','LiDAR','camera'])await choose(page,name+' version','v1.2.1');
    await expect(panel.getByRole('button',{name:'Reset all to release versions'})).toBeVisible();
    await panel.getByRole('combobox',{name:'GPS version',exact:true}).locator('..').locator('..').click();
    await expect(page.locator('.ant-select-dropdown:visible .ant-select-item-option-disabled').filter({hasText:'v2.0.0'})).toBeVisible();await page.keyboard.press('Escape');
    await shot(page,'host-updater-gui-override',mobile,'stack-gui');await shot(page,'host-updater-component-overrides',mobile,'stack-gps');
    const overrides=Object.fromEntries(['mowgli','gui','gps','lidar','camera'].map(name=>[name,alternative]));
    await page.route('**/api/system/updater/plan',r=>r.fulfill({json:plan(data.state.releases[0],overrides)}));
    expect(posts).toEqual([]);await panel.getByRole('button',{name:'Review changes',exact:true}).click();
    expect(posts[0].body).toEqual({deployment:'release-new',pinned:true,component_deployments:{mowgli:alternative.id,gui:alternative.id,gps:alternative.id,lidar:alternative.id,camera:alternative.id}});
    await expect(page.getByRole('dialog').getByText('Robot software: custom version v1.2.1')).toBeVisible();
    await expect(page.getByRole('dialog').getByText('GPS: custom version v1.2.1')).toBeVisible();
    await shot(page,'host-updater-gui-review',mobile);await shot(page,'host-updater-component-review',mobile);
    await page.getByRole('dialog').getByRole('button',{name:'Cancel',exact:true}).click();
    await panel.getByRole('button',{name:'Reset all to release versions'}).click();
    await expect(panel.getByRole('button',{name:'Reset all to release versions'})).toHaveCount(0);
    await choose(page,'GPS version','v1.2.1');await page.locator('.ant-segmented').getByText('Simple',{exact:true}).click();
    await panel.getByRole('button',{name:'Review matched release',exact:true}).click();expect(posts[1].body).toEqual({deployment:'release-new',pinned:true});expect(errors).toEqual([]);
});

test('preferences stay separate and source changes never install',async({page})=>{
    const data=fixture();data.trusted_repositories.push('wjcloudy/mowglinext');const {panel,posts}=await open(page,data);await advanced(page);
    await panel.getByText('Update settings',{exact:true}).click();await choose(page,'Update source','Custom branch');
    await panel.getByRole('textbox',{name:'Custom branch',exact:true}).fill('feature/test');
    await choose(page,'Repository','wjcloudy/mowglinext');
    expect(posts).toEqual([]);await expect(panel.getByRole('button',{name:'Review changes',exact:true})).toBeDisabled();
    await page.route('**/api/system/updater/policy',r=>r.fulfill({json:{ok:true}}));await page.route('**/api/system/updater/check',r=>r.fulfill({status:202,body:''}));
    await panel.getByRole('button',{name:'Save settings',exact:true}).click();await expect.poll(()=>posts.length).toBe(2);
    expect(posts.map(p=>p.path)).toEqual(['/api/system/updater/policy','/api/system/updater/check']);
    expect(posts[0].body).toMatchObject({source:{repository:'wjcloudy/mowglinext',track:'custom',branch:'feature/test'}});
});

test('host updater selection has its own reviewed action',async({page})=>{
    const data=fixture('stable');data.state.releases[2].updater['linux/arm64'].version='updater-alternative';const {panel,posts}=await open(page,data);await advanced(page);
    await choose(page,'Update service version','v1.2.1');expect(posts).toEqual([]);
    await panel.getByRole('button',{name:'Update the update service'}).click();await expect(page.getByRole('dialog')).toBeVisible();expect(posts).toEqual([]);
});

test('unknown health and runtime drift remain distinct',async({page})=>{
    const data=fixture();data.runtime.identity='drifted';data.runtime.health='healthy';const {panel}=await open(page,data);
    await expect(panel.getByText('Installation changed',{exact:true})).toBeVisible();await expect(panel.getByText(/Container health: Running/)).toBeVisible();
    await expect(panel.getByRole('button',{name:'Review matched release',exact:true})).toBeEnabled();
});


test('cached progress survives GUI reconnect without a new check',async({page})=>{
    const data=fixture(); const {panel,posts}=await open(page,data);
    await page.route('**/api/system/updater/state',r=>r.fulfill({json:{...data,state:{...data.state,job:{id:'job-1',phase:'verifying',started_at:'2026-09-07T09:30:00Z',plan:plan(data.state.releases[0])}}}}));
    await expect(panel.getByRole('button',{name:'Review changes',exact:true})).toBeDisabled({timeout:10000});
    await page.route('**/api/system/updater/state',r=>r.fulfill({status:503,json:{error:'restarting'}}));
    await expect(panel.getByText(/reconnecting to the GUI/)).toBeVisible({timeout:10000});expect(posts).toEqual([]);
});

test('pending installer choices can be reviewed on the current release',async({page})=>{
    const data=fixture();data.state.releases=[data.state.active];Object.assign(data.runtime,{selection_pending:true});
    const {panel,posts}=await open(page,data);
    await expect(panel.getByRole('button',{name:'Review changes',exact:true})).toBeEnabled();
    await page.route('**/api/system/updater/plan',r=>r.fulfill({json:plan(data.state.active)}));
    await panel.getByRole('button',{name:'Review changes',exact:true}).click();
    expect(posts[0].body).toEqual({deployment:data.state.active.id,pinned:true});
});

test('missing platform and disabled sensors cannot be selected',async({page})=>{
    const data=fixture('stable');delete data.state.releases[2].images.gps.platforms['linux/arm64'];
    const {panel}=await open(page,data);await advanced(page);
    await expect(panel.getByRole('combobox',{name:'LiDAR version',exact:true})).toHaveCount(0);
    await panel.getByRole('combobox',{name:'GPS version',exact:true}).locator('..').locator('..').click();
    await expect(page.locator('.ant-select-dropdown:visible .ant-select-item-option-disabled').filter({hasText:'v1.2.1'})).toBeVisible();
});

for(const mobile of [false,true])test(`upstream custom branch settings ${mobile?'mobile':'desktop'}`,async({page})=>{
    const data=fixture(); const {panel,posts}=await open(page,data,mobile);await advanced(page);
    const preferences=panel.locator('.update-preferences');await preferences.locator(':scope > summary').click();
    await choose(page,'Update source','Custom branch');await panel.getByRole('textbox',{name:'Custom branch',exact:true}).fill('feat/example-update');
    await preferences.evaluate(el=>el.scrollIntoView({block:'center'}));
    await shot(page,'host-updater-preferences',mobile);expect(posts).toEqual([]);
});


for(const mobile of [false,true])test(`update notification bell ${mobile?'mobile':'desktop'}`,async({page})=>{
    const data=fixture('stable');
    Object.assign(data.state,{notices:[
        {id:'production-update',kind:'available',deployment:'v1.3.0',created_at:new Date().toISOString(),read:false,dismissed:false},
        {id:'updater-update',kind:'updater',deployment:'v1.3.0',created_at:new Date().toISOString(),read:false,dismissed:false},
    ]});
    const {posts,errors}=await open(page,data,mobile);
    if(!mobile)await page.setViewportSize({width:1440,height:1000});
    await page.goto('/#/mowglinext');
    const bell=page.getByRole('button',{name:'Notifications (2 unread)',exact:true});await expect(bell).toBeVisible();
    await expect(page.getByTestId('host-updater')).toHaveCount(0);
    // Lazy route loading can temporarily replace the entire shell with Suspense.
    // Wait for actual dashboard content and its entrance animation, not just the bell.
    await expect(page.getByText('idle',{exact:true})).toBeVisible();
    await expect(page.getByText('Firmware OK',{exact:true})).toBeVisible();
    await expect(page.locator('main').getByText('100',{exact:true})).toBeVisible();
    await expect(page.getByText('No area recorded yet',{exact:true})).toHaveCount(0);
    await expect(page.locator('main > div')).toHaveCSS('opacity','1');
    await page.evaluate(()=>document.fonts.ready);
    await expect(page.locator('circle[stroke="url(#concept-batt)"]').last()).toHaveCSS('stroke-dashoffset','0px');
    await expect(page.locator('path[stroke="url(#lawnEdge)"]').first()).toHaveCSS('opacity','1');
    await expect(bell).toBeInViewport();
    const header=page.locator('header').filter({has:bell});
    expect(await header.evaluate(el=>Array.from(el.querySelectorAll('button')).every(b=>b.getBoundingClientRect().right<=innerWidth))).toBe(true);
    await expect(page.getByText('An update is available',{exact:true})).toHaveCount(0);
    await shot(page,'host-updater-notification-badge',mobile);
    await bell.click();
    await expect(page.getByText('An update is available',{exact:true})).toBeVisible();
    await expect(page.getByText('An update to the host updater is available',{exact:true})).toBeVisible();
    await shot(page,'host-updater-notification-panel',mobile);
    const bounds=await page.getByText('Notifications',{exact:true}).locator('..').locator('..').boundingBox();
    expect(bounds).not.toBeNull();expect(bounds!.x).toBeGreaterThanOrEqual(0);
    expect(bounds!.x+bounds!.width).toBeLessThanOrEqual(page.viewportSize()!.width);
    expect(posts).toEqual([]);
    await page.route('**/api/system/updater/notice',r=>r.fulfill({json:{ok:true}}));
    await page.getByRole('link',{name:'Open Updates',exact:true}).first().click();
    await expect(page.getByTestId('host-updater')).toBeVisible();
    expect(posts.every(p=>p.path==='/api/system/updater/notice')).toBe(true);expect(errors).toEqual([]);
});

for(const identity of ['matched','mixed','drifted','custom','unverified'])test(`running summary reports ${identity}, never the available version`,async({page})=>{
    const data=fixture('stable');data.runtime.identity=identity;
    const {posts}=await open(page,data);
    const summary=page.getByTestId('running-version-summary');
    await expect(summary).toContainText(identity==='matched'?'v1.2.0':identity==='unverified'?'Unknown':'Custom');
    await expect(summary).not.toContainText('v1.3.0');
    await summary.click();await expect(page).toHaveURL(/settings\?section=updates/);
    expect(posts).toEqual([]);
});
test('mobile More shows the running version',async({page})=>{
    const {posts}=await open(page,fixture('stable'),true);
    await page.getByRole('button',{name:'More',exact:true}).click();
    const summary=page.getByTestId('running-version-summary');await expect(summary).toContainText('v1.2.0');
    await shot(page,'host-updater-more',true);
    await summary.click();await expect(summary).toHaveCount(0);expect(posts).toEqual([]);
});
