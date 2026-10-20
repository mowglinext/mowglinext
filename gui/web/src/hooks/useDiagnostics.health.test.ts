import {act, cleanup, renderHook} from '@testing-library/react';
import {afterEach, expect, it, vi} from 'vitest';
import {useDiagnostics} from './useDiagnostics';
import {deriveDiagnosticHealth} from '../utils/diagnosticHealth';
import {healthy} from '../test/diagnosticHealthFixture';
const stream=vi.hoisted(()=>({receive:(value:unknown)=>{void value;},close:()=>{},start:vi.fn(),stop:vi.fn()}));
vi.mock('./useWS.ts',()=>({useWS:(close:()=>void,_open:unknown,receive:(value:unknown)=>void)=>{
    stream.receive=receive;stream.close=close;return {start:stream.start,stop:stream.stop};
}}));
afterEach(()=>{cleanup();vi.useRealTimers();});
it('cannot turn older different cached diagnostic publications into current healthy evidence after reconnect',()=>{
    vi.useFakeTimers();vi.setSystemTime(1000);
    const hook=renderHook(useDiagnostics);
    const frame=(sec:number)=>({header:{stamp:{sec,nanosec:0}},status:healthy().diagnostics.map(({receivedAt,...entry})=>{
        void receivedAt;return entry;
    })});
    act(()=>stream.receive(frame(10)));act(()=>{vi.advanceTimersByTime(1000);});
    act(()=>stream.close());act(()=>{vi.advanceTimersByTime(1000);});
    act(()=>stream.receive(frame(9)));act(()=>{vi.advanceTimersByTime(1000);});
    const verdict=()=>deriveDiagnosticHealth({...healthy(),nowMs:Date.now(),nowMonotonic:performance.now(),
        diagnostics:hook.result.current.diagnostics.status??[],diagnosticsDisconnectedAt:hook.result.current.disconnectedAt});
    expect(verdict().checks.find(c=>c.id==='ros')?.level).toBe('unknown');
    act(()=>stream.receive(frame(10)));act(()=>{vi.advanceTimersByTime(1000);});
    expect(verdict().checks.find(c=>c.id==='ros')?.level).toBe('unknown');
    act(()=>stream.receive(frame(11)));act(()=>{vi.advanceTimersByTime(1000);});
    expect(verdict().checks.find(c=>c.id==='ros')?.level).toBe('ok');
});
it('keeps per-source progress across a diagnostic source round trip without rewriting legacy receipts',()=>{
    vi.useFakeTimers();vi.setSystemTime(1000);
    const hook=renderHook(useDiagnostics);
    const send=(hardware_id:string,sec:number)=>{
        act(()=>stream.receive({header:{stamp:{sec,nanosec:0}},status:[{...healthy().diagnostics[0],hardware_id}]}));
        act(()=>{vi.advanceTimersByTime(1000);});
    };
    send('source-A',42);const originalAt=hook.result.current.diagnostics.status?.[0].receivedAt;
    send('source-B',1);send('source-A',42);
    expect(hook.result.current.diagnostics.status?.[0].healthBlocked).toBe(true);
    expect(hook.result.current.diagnostics.status?.[0].receivedAt).toBeGreaterThan(originalAt??0);
    send('source-A',43);expect(hook.result.current.diagnostics.status?.[0].healthBlocked).toBe(false);
});
