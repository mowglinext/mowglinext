import {describe, expect, it} from 'vitest';
import {deriveDiagnosticHealth, diagnosticTelemetryFresh, type DiagnosticHealthInput} from './diagnosticHealth';
import type {ValidatedTelemetry} from '../hooks/useValidatedTelemetry';

import {healthy} from '../test/diagnosticHealthFixture';
const one = (input: DiagnosticHealthInput, id: string) => deriveDiagnosticHealth(input).checks.find(c => c.id === id);

describe('diagnostic display truth and priority', () => {
    it('reports all evidence absent as unknown rather than critical or OK', () => {
        const input = {...healthy(), emergency: {}, highLevel: {}, emergencyFresh: false, operationalFresh: false,
            batteryFresh: false, snapshot: null, snapshotFresh: false, gnss: {}, diagnostics: [], settings: {}};
        const result = deriveDiagnosticHealth(input);
        expect(result.level).toBe('unknown'); expect(result.critical).toEqual([]);
        expect(result.unknown.length).toBeGreaterThan(5);
    });
    it('reports OK only with all required positive evidence', () => {
        expect(deriveDiagnosticHealth(healthy()).level).toBe('ok');
        expect(deriveDiagnosticHealth(healthy()).notApplicable.map(c => c.id)).toEqual(['lidar']);
    });
    it.each(['missing','loading','failed','stale','empty','legacy','partial core'])('has no Containers OK for %s', scenario => {
        const input = healthy();
        if (scenario === 'missing' || scenario === 'loading') input.snapshot = null;
        else if (scenario === 'failed' || scenario === 'stale') input.snapshotFresh = false;
        else if (input.snapshot) {
            if (scenario === 'empty') input.snapshot.containers = [];
            if (scenario === 'legacy') delete input.snapshot.containers_available;
            if (scenario === 'partial core') input.snapshot.containers.pop();
        }
        expect(one(input, 'containers')?.level).toBe('unknown');
        expect(deriveDiagnosticHealth(input).level).toBe('unknown');
    });
    it('distinguishes a required stopped service from an unrelated stopped container', () => {
        const input = healthy(); if (!input.snapshot) throw new Error('fixture');
        input.snapshot.containers[0].state = 'exited';
        expect(one(input, 'containers')?.level).toBe('critical');
        input.snapshot.containers[0].state = 'running';
        input.snapshot.containers.push({name: 'optional-tool', component: 'gps', state: 'exited', status: '', started_at: ''});
        expect(one(input, 'containers')?.level).toBe('warning');
    });
    it.each([undefined, NaN, Infinity])('does not fabricate CPU temperature for %j', temperature => {
        const input = healthy(); if (!input.snapshot) throw new Error('fixture');
        input.snapshot.system.cpu_temperature = temperature;
        expect(one(input, 'cpu')?.level).toBe('unknown');
    });
    it.each([{value: 0, level: 'ok'}, {value: 55, level: 'ok'}, {value: 55.01, level: 'warning'},
        {value: 70, level: 'warning'}, {value: 70.01, level: 'critical'}])('keeps exact CPU boundary $value as $level', ({value,level}) => {
        const input = healthy(); if (!input.snapshot) throw new Error('fixture');
        input.snapshot.system.cpu_temperature = value; expect(one(input, 'cpu')?.level).toBe(level);
        input.snapshot.system.cpu_temperature_available = false; expect(one(input, 'cpu')?.level).toBe('unknown');
    });
    it('withholds legacy zero but accepts an explicit modern zero', () => {
        const input = healthy(); if (!input.snapshot) throw new Error('fixture');
        input.snapshot.system = {cpu_temperature: 0}; expect(one(input, 'cpu')?.level).toBe('unknown');
        input.snapshot.system.cpu_temperature_available = true; expect(one(input, 'cpu')?.level).toBe('ok');
    });
    it.each([0, 1, 2, 3, 4])('unknown GNSS stays unknown in ROS operational mode %s', state => {
        const input = {...healthy(), highLevel: {state}, gnss: {}};
        expect(one(input, 'gnss')?.level).toBe('unknown');
        expect(deriveDiagnosticHealth(input).level).toBe('unknown');
    });
    it.each([{fix_type: 0, fix_valid: false}, {fix_type: 5, fix_valid: true}, {fix_type: 6, fix_valid: true},
        {fix_type: 2, fix_valid: true, horizontal_accuracy_m: .5}])('shows confirmed GNSS degradation without inventing a whole-robot critical fault: %j', gnss => {
        expect(one({...healthy(), gnss}, 'gnss')?.level).toBe('warning');
    });
    it('keeps Fixed with unavailable accuracy unknown; explicit zero is available', () => {
        expect(one({...healthy(),gnss:{fix_type:3,fix_valid:true}},'gnss')?.level).toBe('unknown');
        expect(one(healthy(),'gnss')?.level).toBe('ok');
    });
    it('does not infer global GNSS inapplicability from the direct bridge disable flag', () => {
        const input = {...healthy(), settings: {gnss_stack: 'disabled'}, gnss: {}};
        expect(one(input,'gnss')?.level).toBe('unknown');
        expect(one({...input,gnss:{fix_type:0,fix_valid:false}},'gnss')?.level).toBe('warning');
    });
    it.each([{value: NaN,level:'unknown'},{value:0,level:'critical'},{value:20,level:'critical'},
        {value:21,level:'warning'},{value:50,level:'warning'},{value:51,level:'ok'}])('keeps battery boundary $value as $level',({value,level})=>{
        expect(one({...healthy(),batteryPercent:value},'battery')?.level).toBe(level);
        expect(one({...healthy(),batteryPercent:value,batteryFresh:false},'battery')?.level).toBe('unknown');
    });
    it.each([{level:0,expected:'ok'},{level:1,expected:'warning'},{level:2,expected:'warning'},{level:3,expected:'unknown'}])('distinguishes ROS $level as $expected without a text filter',({level,expected})=>{
        const input=healthy();input.diagnostics[0].name='Optional Tool';input.diagnostics[0].hardware_id='fixture/tool';
        input.diagnostics[0].level=level;input.diagnostics[0].message='no data since last update';
        expect(one(input,'ros')?.level).toBe(expected);
        input.nowMs=31000;expect(one(input,'ros')?.level).toBe('unknown');
        const alerts=deriveDiagnosticHealth(input).alerts;expect(alerts[0].level).toBe(3);expect(alerts[0].reportedLevel).toBe(level);
    });
    it('does not infer optional LiDAR availability or hide its error when disabled',()=>{
        const input=healthy();input.diagnostics.push({...input.diagnostics[0], name:'LiDAR',level:2});
        expect(one(input,'lidar')?.level).toBe('not_applicable');expect(one(input,'ros')?.level).toBe('warning');
    });
    it.each([{}, {active_emergency:true}, {latched_emergency:true}, {clearanceUnverified:true}])('retains emergency priority for %j', emergency=>{
        const input={...healthy(),emergency};
        if(Object.keys(emergency).length===0) expect(deriveDiagnosticHealth(input).level).toBe('unknown');
        else {const result=deriveDiagnosticHealth({...input,snapshot:null,gnss:{}});expect(result.level).toBe('critical');expect(result.primary.id).toBe('emergency');}
    });
    it('uses a confirmed current localizer error in autonomous mode, never a stale one',()=>{
        const input={...healthy(),highLevel:{state:2},gnss:{},fusion:{receivedAt:1000,distinctMonotonic:1000,level:2,message:'fixture fault',values:{}}};
        expect(deriveDiagnosticHealth(input).primary.id).toBe('localizer');
        expect(deriveDiagnosticHealth(input).level).toBe('critical');
        expect(deriveDiagnosticHealth({...input,nowMonotonic:31000}).level).toBe('unknown');
        expect(deriveDiagnosticHealth({...input,highLevel:{state:1}}).level).toBe('unknown');
    });
    it('cannot revive stale ROS alerts through reconnect or same cached publication',()=>{
        const input={...healthy(),diagnosticsDisconnectedAt:1000};
        expect(one(input,'ros')?.level).toBe('unknown');
        input.diagnostics[0].publicationStamp='2:0';input.diagnostics[0].receivedAt=1001;input.nowMs=1001;
        expect(one(input,'ros')?.level).toBe('ok');
    });
    it('recovers from missing evidence when a genuinely valid sample becomes available',()=>{
        const input=healthy();input.gnss={};expect(deriveDiagnosticHealth(input).level).toBe('unknown');
        input.gnss=healthy().gnss;expect(deriveDiagnosticHealth(input).level).toBe('ok');
    });
});

it('distinguishes receipt freshness, disconnect barriers and producer progression',()=>{
    const telemetry:ValidatedTelemetry<object>={data:{},assertions:{},condition:'valid',hasReceivedFrame:true,lastValidReceiptAt:1000,stampChangedAt:1000,stampIdentity:'1:0'};
    expect(diagnosticTelemetryFresh(telemetry,1001)).toBe(true);
    expect(diagnosticTelemetryFresh({...telemetry,lastDisconnectAt:1000},1001)).toBe(false);
    expect(diagnosticTelemetryFresh({...telemetry,lastDisconnectAt:999},1001)).toBe(true);
    expect(diagnosticTelemetryFresh({...telemetry,lastDisconnectAt:999,stampChangedAt:null},1001)).toBe(false);
    expect(diagnosticTelemetryFresh(telemetry,31000)).toBe(false);
    expect(diagnosticTelemetryFresh({...telemetry,condition:'disconnected'},1001)).toBe(false);
});

it('promotes a current, structured required motor temperature ERROR without promoting an unrelated ERROR',()=>{
    const input=healthy();
    input.diagnostics=[{name:'Motors',hardware_id:'mowgli/motors',message:'critical temperature',level:2,
        receivedAt:1000,values:[{key:'motor_temperature_c',value:'90'}]}];
    expect(deriveDiagnosticHealth(input).level).toBe('critical');
    expect(deriveDiagnosticHealth(input).primary.id).toBe('ros');
    input.diagnostics[0].hardware_id='optional-device';expect(deriveDiagnosticHealth(input).level).toBe('warning');
    input.diagnostics[0].hardware_id='mowgli/motors';input.nowMs=31000;
    expect(deriveDiagnosticHealth(input).level).toBe('unknown');
});
it('does not report an initializing localizer as healthy merely because its publication level is OK',()=>{
    const input={...healthy(),highLevel:{state:2},fusion:{receivedAt:1000,distinctMonotonic:1000,level:0,message:'waiting init',values:{total_nodes:'0'}}};
    expect(deriveDiagnosticHealth(input).level).toBe('unknown');
    input.fusion.values.total_nodes='1';expect(deriveDiagnosticHealth(input).level).toBe('ok');
});
it('does not treat renamed old core containers as required live services',()=>{
    const input=healthy();if(!input.snapshot)throw new Error('fixture');
    input.snapshot.containers.push({name:'old-core-backup',component:'mowgli-ros2',state:'exited',status:'',started_at:''});
    expect(one(input,'containers')?.level).toBe('ok');
});
it('keeps legacy configuration uncertainty visible without claiming a malfunction',()=>{
    const input=healthy();if(!input.snapshot)throw new Error('fixture');
    input.snapshot.cross_checks.overall_status='warn';input.snapshot.cross_checks.warnings=['fixture configuration warning'];
    expect(deriveDiagnosticHealth(input).level).toBe('unknown');
    expect(deriveDiagnosticHealth(input).primary.id).toBe('configuration');
});
it('does not require an explicitly disabled LiDAR container or an unrelated host tool to run',()=>{
    const input=healthy();if(!input.snapshot)throw new Error('fixture');
    input.snapshot.containers.push({name:'optional-lidar',component:'lidar-ldlidar',state:'exited',status:'',started_at:''},
        {name:'unrelated-host-tool',state:'exited',status:'',started_at:''});
    expect(one(input,'containers')?.level).toBe('ok');
    input.settings.lidar_enabled=true;expect(one(input,'containers')?.level).toBe('warning');
});
it.each([{name:'Hardware Bridge',hardware_id:'mowgli/hardware_bridge'}, {name:'IMU',hardware_id:'mowgli/imu'},
    {name:'LiDAR',hardware_id:'mowgli/lidar'}, {name:'Odometry',hardware_id:'mowgli/odometry'}])('keeps producer-confirmed availability errors unknown for $name',source=>{
    const input=healthy();input.diagnostics=[{...source,level:2,message:'fixture unavailable',receivedAt:1000,values:[]}];
    expect(one(input,'ros')?.level).toBe('unknown');
    expect(deriveDiagnosticHealth(input).alerts[0]).toMatchObject({level:3,reportedLevel:2});
});
it('keeps a structured live path-tracking ERROR critical during autonomous operation',()=>{
    const input=healthy();input.highLevel={state:2};input.diagnostics=[{name:'Path Tracking',hardware_id:'mowgli/controller_server',
        level:2,message:'fixture active tracking fault',receivedAt:1000,values:[{key:'recent_p95_lateral_error_m',value:'0.5'}]}];
    expect(deriveDiagnosticHealth(input).level).toBe('critical');
    input.highLevel={state:1};expect(one(input,'ros')?.level).toBe('warning');
});
it('keeps a structured required battery ERROR critical even when another estimate is positive',()=>{
    const input=healthy();input.diagnostics=[{name:'Battery',hardware_id:'mowgli/battery',message:'fixture battery critical',
        level:2,receivedAt:1000,values:[{key:'voltage_v',value:'23'},{key:'percentage',value:'0'}]}];
    expect(deriveDiagnosticHealth(input).level).toBe('critical');
    expect(one(input,'battery')?.level).toBe('critical');
});
it.each(['0','NaN','0x17',''])('does not invent a critical measured battery from invalid/absent voltage %j',voltage=>{
    const input=healthy();input.diagnostics=[{name:'Battery',hardware_id:'mowgli/battery',message:'fixture unavailable estimate',
        level:2,receivedAt:1000,values:[{key:'voltage_v',value:voltage},{key:'percentage',value:'0'}]}];
    expect(one(input,'battery')?.level).toBe('ok');expect(deriveDiagnosticHealth(input).critical).toEqual([]);
});
it.each(['','unrecognized-state'])('does not turn an unknown Docker state %j into a confirmed core failure',state=>{
    const input=healthy();if(!input.snapshot)throw new Error('fixture');input.snapshot.containers[0].state=state;
    expect(one(input,'containers')?.level).toBe('unknown');expect(deriveDiagnosticHealth(input).level).toBe('unknown');
});
