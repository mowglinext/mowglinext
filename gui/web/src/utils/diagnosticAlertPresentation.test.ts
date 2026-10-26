import {describe, expect, it} from 'vitest';
import {autonomousReceptionRequired, advanceReceptionWatches as advance, missingReceptionSources as missing, type RequiredDiagnosticSource} from './diagnosticAlertPresentation';
const source = (publication: number | null = 0, ageMs = 0, key = 'sensor'): RequiredDiagnosticSource => ({key, label: key, publication, ageMs});
describe('60 continuous seconds without admissible delivery, presentation only', () => {
    it('keeps one unchanged alert at 60, 61, 70 and 90; recovers without history', () => {
        let watches = advance([], [source()], 0);
        for (const seconds of [10,30,59,60,61,70,90]) {
            watches = advance(watches, [source(null)], seconds*1000);
            expect(missing(watches, seconds*1000)).toHaveLength(seconds < 60 ? 0 : 1);
            expect(watches[0].lastAdmissibleAt).toBe(0);
        }
        watches = advance(watches, [source(90001)], 90001);
        expect(missing(watches,90001)).toHaveLength(0);
    });
    it('does not renew for replay, regression, invalid data or reconnect without progress', () => {
        let watches = advance([], [source(42)], 0);
        for (const publication of [42,null,41,null,42]) watches = advance(watches,[source(publication)],59000);
        expect(missing(watches,60000)).toHaveLength(1);
        watches = advance(watches,[source(43)],61000);
        expect(missing(watches,61000)).toHaveLength(0);
    });
    it('ignores repeated short gaps for several minutes and does not reset other sources', () => {
        let watches = advance([], [source()], 0);
        for (let at=5000;at<300000;at+=20000) {
            watches=advance(watches,[source(null)],at);
            expect(missing(watches,at)).toHaveLength(0);
            watches=advance(watches,[source(at+10000)],at+10000);
        }
        watches=advance([], [source(null,0,'A'),source(0,0,'B')],0);
        watches=advance(watches,[source(null,0,'A'),source(59000,0,'B')],59000);
        expect(missing(watches,60000).map(s=>s.key)).toEqual(['A']);
    });
    it('counts from actual delivery, not internal stale detection, and bounds state to required sources', () => {
        let watches=advance([], [source(0,30000)],30000);
        expect(missing(watches,59999)).toHaveLength(0);
        expect(missing(watches,60000)).toHaveLength(1);
        watches=advance(watches,[],120000);
        expect(watches).toEqual([]);
    });
});

it('retains an autonomous source requirement through invalid/disconnected context until an explicit operational change', () => {
    expect(autonomousReceptionRequired(false,2)).toBe(true);
    expect(autonomousReceptionRequired(true,undefined)).toBe(true);
    expect(autonomousReceptionRequired(true,0)).toBe(true);
    expect(autonomousReceptionRequired(true,1)).toBe(false);
    expect(autonomousReceptionRequired(false,undefined)).toBe(false);
});

import {backendReceptionReports} from './diagnosticAlertPresentation';
it.each(['mowgli','mavros','openmower','unknown'] as const)('uses the shared monitoring contract and explicit odometry requirement for %s', backend => {
    expect(backendReceptionReports(backend,false).map(report=>report[0])).toEqual(
        backend==='unknown'?[]:backend==='mavros'?['hardware']:['hardware','odometry']);
    if(backend==='mavros')expect(backendReceptionReports(backend,true).map(report=>report[0])).toEqual(['hardware','odometry']);
});

it('resets only watches whose backend context changed',()=>{
    const sources=[{...source(1,0,'hardware'),context:'mavros'},source(1,0,'operation')];
    const first=advance([],sources,0);
    const changed=advance(first,[{...sources[0],context:'openmower',publication:null},sources[1]],60000);
    expect(missing(changed,60000).map(s=>s.key)).toEqual(['operation']);
    expect(missing(changed,120000).map(s=>s.key)).toEqual(['hardware','operation']);
});
