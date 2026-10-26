import {expect, test, type WebSocketRoute} from '@playwright/test';
import {pack} from 'msgpackr';
import {healthy} from '../../src/test/diagnosticHealthFixture';
import {isJsonObject} from '../../src/utils/json';
import {installMockBackend} from './mock/mockBackend';
import en from '../../src/i18n/locales/en.json' with {type: 'json'};
import fr from '../../src/i18n/locales/fr.json' with {type: 'json'};

for (const language of ['en','fr']) for (const width of [390,1440]) {
    test(`quiet reception deadline, recovery and immediate fault in ${language} at ${width}px`,async({page})=>{
        const locale=language==='fr'?fr:en;
        await page.setViewportSize({width,height:1000});
        await page.addInitScript(lang=>localStorage.setItem('mowglinext.lang',lang),language);
        await page.clock.install({time:new Date('2026-10-16T08:00:00Z')});
        await installMockBackend(page,{name:'quiet-diagnostics',rest:{'/api/settings/yaml':{lidar_enabled:false,gnss_stack:'disabled'}}});
        const snapshot=healthy().snapshot;
        if(!snapshot)throw new Error('No fixture');
        let snapshotSequence=0;
        await page.route('**/api/diagnostics/snapshot',route=>route.fulfill({json:{...snapshot,timestamp:new Date(1000*(++snapshotSequence)).toISOString()}}));
        let socket:WebSocketRoute|undefined;
        let sequence=0;
        const highLevel={state:1,battery_percent:80};
        const diagnostic=(level=0)=>({header:{stamp:{sec:++sequence,nanosec:0}},status:[
            ...[['Hardware Bridge','mowgli/hardware_bridge'],['Odometry','mowgli/odometry']].map(([name,hardware_id])=>({name,hardware_id,level:0,message:'OK',values:[]})),
            {name:'Motor fixture',hardware_id:'fictional-motor',level,message:level?'Confirmed motor fault':'OK',values:[]},
        ]});
        await page.routeWebSocket(/\/api\/mowglinext\/multiplex/,ws=>{
            socket=ws;
            const topics:Record<string,unknown>={highLevelStatus:highLevel,diagnostics:diagnostic(),
                emergency:{active_emergency:false,latched_emergency:false,stamp:{sec:++sequence,nanosec:0}},
                power:{v_battery:28,stamp:{sec:++sequence,nanosec:0}},gnssStatus:healthy().gnss};
            ws.onMessage(message=>{
                const request:unknown=JSON.parse(typeof message==='string'?message:message.toString());
                if(!isJsonObject(request)||request.op!=='subscribe'||typeof request.topic!=='string')return;
                if(topics[request.topic]!==undefined)ws.send(pack({topic:request.topic,data:topics[request.topic]}));
            });
        });
        const writes:string[]=[];
        page.on('request',request=>{if(['POST','PUT','DELETE'].includes(request.method())&&!new URL(request.url()).pathname.endsWith('/config/keys/get'))writes.push(request.url());});
        await page.goto('/#/diagnostics');
        const panel=page;
        await expect(page.getByTestId('diagnostic-health-verdict')).toBeVisible();
        await page.clock.pauseAt(new Date(await page.evaluate(()=>Date.now())+1000));
        const send=(topic:string,data:unknown)=>{if(!socket)throw new Error('No socket');socket.send(pack({topic,data}));};
        const heartbeat=()=>{
            send('diagnostics',diagnostic());
            send('emergency',{active_emergency:false,latched_emergency:false,stamp:{sec:++sequence,nanosec:0}});
            send('power',{v_battery:28,stamp:{sec:++sequence,nanosec:0}});
        };
        // Establish the target at the controlled clock origin; other sources
        // keep progressing and cannot reset its independent reception watch.
        send('highLevelStatus',highLevel);heartbeat();
        await page.clock.runFor(1000); // Flush the existing diagnostics throttle.
        send('highLevelStatus',{...highLevel,battery_percent:81});
        await expect(page.getByText(locale.diagnosticsPage.batteryBadge.replace('{{value}}','81'),{exact:true})).toBeVisible();
        await expect(panel.getByTestId('diagnostic-reception-alert')).toHaveCount(0);
        const sections=width<768?page.locator('.diagnostics-page > .ant-collapse'):page.getByRole('tablist');
        const before=await sections.boundingBox();
        let second=0;
        for(const target of [10,30,59,60,61,70,90]){
            heartbeat();
            await page.clock.fastForward((target-second)*1000);second=target;
            const alerts=panel.getByTestId('diagnostic-reception-alert');
            await expect(alerts).toHaveCount(target<60?0:1);
            if(target>=60){
                await expect(alerts).toHaveAttribute('data-source','operation');
                await expect(alerts).toHaveText(locale.diagnosticReception.missing.replace('{{source}}',locale.diagnosticReception.source.operation));
            }else{
                const current=await sections.boundingBox();
                expect(current?.y).toBeCloseTo(before?.y??0,0);
                expect(current?.height).toBeCloseTo(before?.height??0,0);
            }
        }
        send('highLevelStatus',{...highLevel,battery_percent:81});
        await expect(page.getByText(locale.diagnosticsPage.batteryBadge.replace('{{value}}','81'),{exact:true})).toBeVisible();
        await expect(panel.getByTestId('diagnostic-reception-alert')).toHaveCount(0);
        // Several minutes of repeated short interruptions never create a visual alert.
        for(let i=0;i<12;i++){
            heartbeat();await page.clock.fastForward((5+i%4*5)*1000);
            send('highLevelStatus',highLevel);
            await expect(panel.getByTestId('diagnostic-reception-alert')).toHaveCount(0);
        }
        send('diagnostics',diagnostic(2));await page.clock.runFor(1000);
        const fault=page.getByTestId('diagnostic-alert-panel').getByRole('button',{name:/Motor fixture/});
        await expect(fault).toBeVisible();await fault.click();
        await expect(page.getByTestId('diagnostic-alert-panel').getByText('Confirmed motor fault',{exact:true})).toBeVisible();
        send('diagnostics',diagnostic());await page.clock.runFor(1000);
        await expect(fault).toHaveCount(0);
        send('emergency',{active_emergency:true,latched_emergency:true,stamp:{sec:++sequence,nanosec:0}});
        await expect(page.getByTestId('diagnostic-health-verdict')).toHaveAttribute('data-level','critical');
        await expect(page.getByTestId('diagnostic-alert-panel')).toHaveCount(0);
        await expect(page.locator('.diagnostics-page')).not.toContainText(locale.telemetry.freshnessUnverified);
        expect(await page.evaluate(()=>document.documentElement.scrollWidth)).toBeLessThanOrEqual(width);
        expect(writes).toEqual([]);
    });
}
