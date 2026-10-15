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
