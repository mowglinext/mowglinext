import {expect,test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';

for (const mobile of [false,true]) test(`served web identity and reload ${mobile?'mobile':'desktop'}`,async({page,request})=>{
    await page.setViewportSize(mobile?{width:390,height:844}:{width:1440,height:1000});
    const served=await (await request.get('/web-build.json')).json();
    expect(served.id).toBeTruthy();
    await installMockBackend(page,{...SCENARIOS[0],rest:{'/api/system/versions':{
        server:{revision:'backend-c40e03c3',built_at:'earlier-backend-build'},docker_available:true,components:[],
    }}});
    const warning=page.getByText('This browser is using a different web build from the server',{exact:true});
    await page.goto('/#/settings?section=updates');
    await expect(page.getByTestId('installed-versions')).toBeVisible();
    await expect(warning).toHaveCount(0);
    // Same source revision may still produce different assets. Compare build ID.
    await page.route('**/web-build.json',route=>route.fulfill({json:{...served,id:'another-web-build'}}));
    await page.reload();await expect(warning).toBeVisible();
    await page.route('**/web-build.json',route=>route.fulfill({json:served}));
    await page.getByRole('button',{name:'Refresh browser',exact:true}).click();
    await expect(page.getByTestId('installed-versions')).toBeVisible();await expect(warning).toHaveCount(0);
    // Older servers may send the SPA fallback instead of a manifest: unknown,
    // never substitute the unrelated backend revision or retain a false alert.
    await page.route('**/web-build.json',route=>route.fulfill({contentType:'text/html',body:'<html>legacy</html>'}));
    await page.reload();await expect(page.getByTestId('installed-versions')).toBeVisible();await expect(warning).toHaveCount(0);
});
