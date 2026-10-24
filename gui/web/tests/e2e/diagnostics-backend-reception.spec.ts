import {expect,test,type Page,type WebSocketRoute} from '@playwright/test';
import {pack} from 'msgpackr';
import {healthy} from '../../src/test/diagnosticHealthFixture';
import {isJsonObject} from '../../src/utils/json';
import {installMockBackend} from './mock/mockBackend';
import en from '../../src/i18n/locales/en.json' with {type:'json'};
import fr from '../../src/i18n/locales/fr.json' with {type:'json'};

async function setup(page:Page,backend:'mowgli'|'mavros'|'openmower',language:string,width:number){
    const locale=language==='fr'?fr:en;
    await page.setViewportSize({width,height:1000});
    await page.addInitScript(lang=>localStorage.setItem('mowglinext.lang',lang),language);
    await page.clock.install();
    await installMockBackend(page,{name:'backend-reception',rest:{
        '/api/settings/yaml':{lidar_enabled:false,gnss_stack:'universal'},
        '/api/settings/hardware-backend':{backend,supported:['mowgli','mavros','openmower'],runtime_routing:'available',parameter_routes:{}},
    }});
    const snapshot=healthy().snapshot;
    let snapshotSequence=0,sequence=0,battery=80,connections=0;
    let socket:WebSocketRoute|undefined,allowDelivery=true,subscribed=false;
    const stamp=()=>({sec:++sequence,nanosec:0});
    const topics=(wheelRequired=false,emergency=false)=>({
        highLevelStatus:{state:1,battery_percent:battery},
        emergency:{active_emergency:emergency,latched_emergency:emergency,stamp:stamp()},
        power:{v_battery:28,stamp:stamp()},
        gnssStatus:{...healthy().gnss,backend:backend==='mavros'?'mavros_gps1':'universal',position_observation_sequence:++sequence},
        diagnostics:{header:{stamp:stamp()},status:[
            // Common diagnostics_node reports, NOT publications by the native STM32 node.
            {name:'Hardware Bridge',hardware_id:'mowgli/hardware_bridge',level:0,message:'Last status age: 0s',values:[]},
            {name:'Odometry',hardware_id:'mowgli/odometry',level:2,message:'No odometry received',values:[]},
            ...(backend==='mavros'?[{name:'mowgli_mavros_bridge/wheel_odometry_source',hardware_id:'',level:1,message:'missing_or_stale',values:[{key:'required',value:String(wheelRequired)}]}]:[]),
        ]},
    });
    await page.route('**/api/diagnostics/snapshot',route=>route.fulfill({json:{...snapshot,timestamp:new Date(++snapshotSequence*1000).toISOString()}}));
    await page.routeWebSocket(/\/api\/mowglinext\/multiplex/,ws=>{
        socket=ws;connections++;subscribed=false;
        ws.onClose(()=>{subscribed=false;});
        ws.onMessage(message=>{
            const request:unknown=JSON.parse(typeof message==='string'?message:message.toString());
            if(!isJsonObject(request)||request.op!=='subscribe'||typeof request.topic!=='string')return;
            if(request.topic==='highLevelStatus')subscribed=true;
            if(!allowDelivery)return;
            const values:Record<string,unknown>=topics();
            if(values[request.topic]!==undefined)ws.send(pack({topic:request.topic,data:values[request.topic]}));
        });
    });
    const writes:string[]=[];
    page.on('request',request=>{if(['POST','PUT','DELETE'].includes(request.method())&&!new URL(request.url()).pathname.endsWith('/config/keys/get'))writes.push(request.url());});
    await page.goto('/#/diagnostics');
    await expect(page.getByTestId('diagnostic-health-verdict')).toBeVisible();
    await page.clock.pauseAt(new Date(await page.evaluate(()=>Date.now())+1000));
    const pulse=async(wheelRequired=false,emergency=false)=>{
        // A silence watchdog may have retired the last socket during a clock jump.
        // Finish the existing reconnect timer and wait for actual subscriptions.
        await page.clock.runFor(1000);
        await expect.poll(()=>subscribed).toBe(true);
        battery=battery===80?81:80;
        if(!socket)throw new Error('No socket');
        for(const [topic,data] of Object.entries(topics(wheelRequired,emergency)))socket.send(pack({topic,data}));
        await expect(page.getByText(locale.diagnosticsPage.batteryBadge.replace('{{value}}',String(battery)),{exact:true})).toBeVisible();
        await page.clock.runFor(1000); // Existing diagnostics throttle only.
    };
    await pulse();
    return{locale,writes,pulse,connections:()=>connections,diagnostic:(required:boolean)=>topics(required).diagnostics,send:(topic:string,data:unknown)=>{if(!socket)throw new Error('No socket');socket.send(pack({topic,data}));},disconnect:async()=>{
        allowDelivery=false;
        if(!socket)throw new Error('No socket');
        socket.close({code:1001,reason:'Mock global transport loss'});
        await page.clock.runFor(1000);
    }};
}

for(const language of ['en','fr'])for(const backend of ['mowgli','mavros','openmower'] as const){
    test(`${backend} required sources without a native-node assumption in ${language}`,async({page})=>{
        const app=await setup(page,backend,language,language==='fr'?390:1440);
        for(let i=0;i<5;i++){await page.clock.fastForward(20000);await app.pulse();}
        const reception=page.getByTestId('diagnostic-reception-alert');
        if(backend==='mavros'){
            await expect(reception).toHaveCount(0);
            await expect(page.getByTestId('diagnostic-health-verdict')).toHaveText('—');
            // An explicit producer requirement must not be hidden, even though
            // the optional wheel stream was absent throughout the preceding run.
            await app.pulse(true);
            for(let i=0;i<4;i++){await page.clock.fastForward(20000);await app.pulse(true);}
        }
        await expect(reception).toHaveCount(1);
        await expect(reception).toHaveAttribute('data-source','odometry');
        expect(app.writes).toEqual([]);
    });
}
for(const language of ['en','fr'])for(const width of [390,1440]){
    test(`global ROS disconnect stays one indication and retains emergency in ${language} at ${width}px`,async({page})=>{
        const app=await setup(page,'mavros',language,width);
        await app.pulse(true,true);
        await expect(page.getByTestId('diagnostic-health-verdict')).toHaveAttribute('data-level','critical');
        await app.disconnect();
        let elapsed=0;
        for(const second of [10,30,58,61,70,90]){
            await page.clock.fastForward((second-elapsed)*1000);elapsed=second;
            const reception=page.getByTestId('diagnostic-reception-alert');
            await expect(reception).toHaveCount(second<60?0:1);
            if(second>=60){
                await expect(reception).toHaveAttribute('data-source','communication');
                await expect(reception).toHaveText(app.locale.diagnosticReception.communication);
            }
            await expect(page.getByTestId('diagnostic-health-verdict')).toHaveAttribute('data-level','critical');
        }
        // Automatic socket reconnect and successful HTTP polling have occurred;
        // neither proves admissible ROS recovery or resets the source watches.
        expect(app.connections()).toBeGreaterThan(1);
        await app.pulse(true,true);
        await expect(page.getByTestId('diagnostic-reception-alert')).toHaveCount(1);
        await expect(page.getByTestId('diagnostic-reception-alert')).toHaveAttribute('data-source','odometry');
        // Odometry is genuinely still required/missing. Global coalescing did
        // not clear or restart its own watch when the other streams recovered.
        await expect(page.getByTestId('diagnostic-health-verdict')).toHaveAttribute('data-level','critical');
        expect(app.writes).toEqual([]);
    });
}


test('backend changes scope MAVROS requirements without remounting the page or resetting operation',async({page})=>{
    // The production backend hook currently fetches once. Inject only its next
    // result to exercise a future hot update while keeping the real page mounted.
    await page.route('**/src/hooks/useHardwareBackend.ts',async route=>{
        const response=await route.fetch();
        let body=await response.text();
        body=body.replace('const guiApi = useApi();',`const [testBackend,setTestBackend] = useState(null);
            useEffect(()=>{const change=e=>setTestBackend(e.detail);window.addEventListener('test-backend',change);
                return ()=>window.removeEventListener('test-backend',change);},[]);
            const guiApi = useApi();`);
        body=body.replace(/return \{\s*\.\.\.info,/, 'return {...info,...(testBackend?{backend:testBackend}:{}),');
        await route.fulfill({response,body});
    });
    const app=await setup(page,'mavros','en',1440);
    await page.locator('.diagnostics-page').evaluate(element=>element.setAttribute('data-mount-witness','same-page'));
    await app.pulse(true);
    // Leave operation silent while keeping the diagnostic reports progressing.
    const diagnostic=app.diagnostic;
    const change=async(backend:string)=>{
        await page.evaluate(value=>window.dispatchEvent(new CustomEvent('test-backend',{detail:value})),backend);
        await page.clock.runFor(1000);
    };
    for(let i=0;i<3;i++){await page.clock.fastForward(10000);app.send('diagnostics',diagnostic(true));await page.clock.runFor(1000);}
    await change('openmower');
    await expect(page.getByTestId('diagnostic-health-verdict')).toBeVisible();
    // A MAVROS false declaration delivered under OpenMower must not affect it.
    app.send('diagnostics',diagnostic(false));await page.clock.runFor(1000);
    for(let i=0;i<3;i++){await page.clock.fastForward(10000);app.send('diagnostics',diagnostic(false));await page.clock.runFor(1000);}
    await expect(page.locator('[data-source="operation"]')).toHaveCount(1);
    await expect(page.locator('[data-source="odometry"]')).toHaveCount(0);
    await change('mavros'); // cached false belongs to the preceding context
    await page.clock.fastForward(1000);
    await app.pulse(true); // a new declaration in this context
    for(let i=0;i<4;i++){await page.clock.fastForward(20000);await app.pulse(true);}
    await expect(page.locator('[data-source="odometry"]')).toHaveCount(1);
    await change('mowgli');
    await change('mavros'); // former true and cached true must not survive re-entry
    for(let i=0;i<4;i++){
        await page.clock.fastForward(20000);
        // Omit the declaration entirely; ordinary source data cannot restore it.
        app.send('highLevelStatus',{state:1,battery_percent:80});
        app.send('diagnostics',{header:{stamp:{sec:2000000000+i,nanosec:0}},status:[]});
        await page.clock.runFor(1000);
    }
    await expect(page.locator('[data-source="odometry"]')).toHaveCount(0);
    await expect(page.locator('.diagnostics-page')).toHaveAttribute('data-mount-witness','same-page');
    expect(app.writes).toEqual([]);
});
