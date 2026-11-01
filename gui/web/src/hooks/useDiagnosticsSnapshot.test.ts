import {act, cleanup, renderHook, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, expect, it, vi} from 'vitest';
import {decodeDiagnosticsSnapshot, useDiagnosticsSnapshot} from './useDiagnosticsSnapshot';
import {healthy} from '../test/diagnosticHealthFixture';
const mock = vi.hoisted(()=>({request:vi.fn()}));
vi.mock('./useApi.ts',()=>({useApi:()=>mock}));
beforeEach(()=>mock.request.mockReset());
afterEach(cleanup);
it.each([null,{}, {containers:'wrong'}, {...healthy().snapshot,system:{cpu_temperature:NaN}}])('rejects malformed snapshots %j',raw=>{
    expect(()=>decodeDiagnosticsSnapshot(raw)).toThrow();
});
it('accepts legitimate zero and additive availability metadata without changing collection content',()=>{
    const fixture=healthy().snapshot;if(!fixture)throw new Error('fixture');fixture.system.cpu_temperature=0;
    expect(decodeDiagnosticsSnapshot(fixture)).toEqual(fixture);
});
it.each([
    {ready:false,transport:null},
    {ready:false,transport:{connected:false,heartbeat_ok:false}},
    {ready:false,transport:{connected:true,heartbeat_ok:false}},
    {ready:true,transport:{connected:true,heartbeat_ok:true,last_pong_at:'2026-10-28T12:00:00Z'}},
])('preserves optional ROS transport evidence %j',ros=>{
    expect(decodeDiagnosticsSnapshot({...healthy().snapshot,ros}).ros).toEqual(ros);
});
it.each([null, {}, {ready:true,transport:null}, {ready:true,transport:{connected:true,heartbeat_ok:false}},
    {ready:false,transport:{connected:'true',heartbeat_ok:false}},
    {ready:true,transport:{connected:true,heartbeat_ok:true,last_pong_at:'bad'}}])('rejects malformed ROS transport evidence %j',ros=>{
    expect(()=>decodeDiagnosticsSnapshot({...healthy().snapshot,ros})).toThrow('Invalid ROS transport health');
});
it('checks an API error that does not throw and preserves earlier data without confirming its freshness',async()=>{
    mock.request.mockResolvedValueOnce({data:healthy().snapshot});const hook=renderHook(useDiagnosticsSnapshot);
    await waitFor(()=>expect(hook.result.current.snapshot).not.toBeNull());
    const before=hook.result.current.snapshot;
    mock.request.mockResolvedValue({data:{},error:{error:'fixture refusal'},ok:false});
    await act(()=>hook.result.current.refresh());
    expect(hook.result.current.error).toContain('fixture refusal');expect(hook.result.current.snapshot).toBe(before);
});
it('reports transport rejection and malformed responses without inventing a snapshot',async()=>{
    mock.request.mockRejectedValueOnce(new Error('fixture offline'));const hook=renderHook(useDiagnosticsSnapshot);
    await waitFor(()=>expect(hook.result.current.error).toBe('fixture offline'));
    expect(hook.result.current.snapshot).toBeNull();mock.request.mockResolvedValue({data:{}});
    await act(()=>hook.result.current.refresh());expect(hook.result.current.error).toBe('Invalid diagnostics snapshot');
});
it('does not renew a cached snapshot timestamp or accept a regressed one',async()=>{
    mock.request.mockResolvedValue({data:healthy().snapshot});const hook=renderHook(useDiagnosticsSnapshot);
    await waitFor(()=>expect(hook.result.current.receivedAt).not.toBeNull());
    const at=hook.result.current.receivedAt;
    await act(()=>hook.result.current.refresh());expect(hook.result.current.receivedAt).toBe(at);
    mock.request.mockResolvedValue({data:{...healthy().snapshot,timestamp:'2026-10-14T00:00:00Z'}});
    await act(()=>hook.result.current.refresh());expect(hook.result.current.error).toBe('Regressed diagnostics snapshot');
    expect(hook.result.current.receivedAt).toBe(at);
});
it('ignores an old response when a newer refresh completes first',async()=>{
    let resolve: ((response:{data:unknown})=>void)|undefined;
    mock.request.mockReturnValueOnce(new Promise(r=>{resolve=r;}));const hook=renderHook(useDiagnosticsSnapshot);
    mock.request.mockResolvedValue({data:{...healthy().snapshot,timestamp:'2026-10-16T00:00:00Z'}});
    await act(()=>hook.result.current.refresh());
    await act(async()=>{resolve?.({data:healthy().snapshot});await Promise.resolve();});
    expect(hook.result.current.snapshot?.timestamp).toBe('2026-10-16T00:00:00Z');
});
it('keeps a failure unconfirmed during a pending retry and ignores completion after unmount',async()=>{
    mock.request.mockResolvedValueOnce({data:healthy().snapshot});const hook=renderHook(useDiagnosticsSnapshot);
    await waitFor(()=>expect(hook.result.current.snapshot).not.toBeNull());
    mock.request.mockRejectedValueOnce(new Error('fixture failed'));
    await act(()=>hook.result.current.refresh());
    let resolve:((response:{data:unknown})=>void)|undefined;
    mock.request.mockReturnValueOnce(new Promise(r=>{resolve=r;}));
    let retry:Promise<void>|undefined;
    act(()=>{retry=hook.result.current.refresh();});
    expect(hook.result.current.error).toBe('fixture failed');
    hook.unmount();await act(async()=>{resolve?.({data:healthy().snapshot});await retry;});
});
it('does not start overlapping background polls while the bounded backend read is still pending',async()=>{
    vi.useFakeTimers();
    try {
        let resolve:((response:{data:unknown})=>void)|undefined;
        mock.request.mockReturnValueOnce(new Promise(r=>{resolve=r;}));
        const hook=renderHook(useDiagnosticsSnapshot);
        act(()=>{vi.advanceTimersByTime(10000);});expect(mock.request).toHaveBeenCalledOnce();
        await act(async()=>{resolve?.({data:healthy().snapshot});await Promise.resolve();});
        expect(hook.result.current.snapshot).not.toBeNull();hook.unmount();
    } finally {vi.useRealTimers();}
});
it('does not represent an empty exception message as a successful read',async()=>{
    mock.request.mockRejectedValueOnce(new Error(''));
    const hook=renderHook(useDiagnosticsSnapshot);
    await waitFor(()=>expect(hook.result.current.loading).toBe(false));
    expect(hook.result.current.error).not.toBeNull();expect(hook.result.current.error).not.toBe('');
});
it('keeps temporary delivery failures quiet but exposes decoding and explicit server read errors',async()=>{
    mock.request.mockRejectedValueOnce({status:503,error:{error:'temporarily unavailable'}});
    const hook=renderHook(useDiagnosticsSnapshot);
    await waitFor(()=>expect(hook.result.current.error).not.toBeNull());
    expect(hook.result.current.readError).toBeNull();
    mock.request.mockResolvedValueOnce({data:{}});
    await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBe('Invalid diagnostics snapshot');
    mock.request.mockRejectedValueOnce({status:500,error:{error:'snapshot processing failed'}});
    await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBe('snapshot processing failed');
    mock.request.mockResolvedValueOnce({data:healthy().snapshot});
    await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBeNull();
});
it('exposes a JSON parsing failure returned by the API client as a read error',async()=>{
    mock.request.mockResolvedValueOnce({ok:true,status:200,error:new SyntaxError('Invalid JSON'),data:null});
    const hook=renderHook(useDiagnosticsSnapshot);
    await waitFor(()=>expect(hook.result.current.readError).toBe('Invalid JSON'));
});
