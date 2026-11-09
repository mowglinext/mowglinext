import {act,cleanup,renderHook,waitFor} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
import {useCalibrationStatus} from './useCalibrationStatus';
const mock=vi.hoisted(()=>({request:vi.fn()}));
vi.mock('./useApi.ts',()=>({useApi:()=>mock}));
afterEach(()=>{cleanup();mock.request.mockReset();});
it('distinguishes absent optional resources and transport loss from malformed reads and server errors',async()=>{
    mock.request.mockRejectedValueOnce({status:404});
    const hook=renderHook(useCalibrationStatus);
    await waitFor(()=>expect(hook.result.current.error).not.toBeNull());
    expect(hook.result.current.readError).toBeNull();
    for(const failure of [{status:503},new TypeError('Failed to fetch')]){
        mock.request.mockRejectedValueOnce(failure);await act(()=>hook.result.current.refresh());
        expect(hook.result.current.readError).toBeNull();
    }
    mock.request.mockResolvedValueOnce({data:{dock:null}});await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBe('Invalid calibration status');
    mock.request.mockRejectedValueOnce({status:500,error:{error:'cannot read calibration'}});await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBe('cannot read calibration');
    const data={dock:{present:false},imu:{present:false,error:'invalid artifact'},mag:{present:false}};
    mock.request.mockResolvedValueOnce({data});await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBeNull();expect(hook.result.current.status).toEqual(data);
});

function deferred<T>() {
    let resolve!: (value: T) => void;
    let reject!: (error: unknown) => void;
    const promise = new Promise<T>((yes, no) => {resolve = yes; reject = no;});
    return {promise, resolve, reject};
}
const data = (present: boolean) => ({
    dock: {present: false}, imu: {present}, mag: {present: false},
});
function pendingReads() {
    const reads: ReturnType<typeof deferred<unknown>>[] = [];
    mock.request.mockImplementation(() => {
        const read = deferred<unknown>();
        reads.push(read);
        return read.promise;
    });
    return reads;
}
async function finish(read: ReturnType<typeof deferred<unknown>>, present: boolean) {
    await act(async () => {read.resolve({data: data(present)}); await read.promise;});
}
afterEach(() => {vi.useRealTimers(); vi.restoreAllMocks();});

it.each([false, true])('ignores old imu.present=%s after a newer poll publishes', async oldValue => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    expect(reads).toHaveLength(1);
    await act(() => vi.advanceTimersByTimeAsync(5000));
    expect(reads).toHaveLength(2);
    await finish(reads[1], !oldValue);
    await finish(reads[0], oldValue);
    expect(hook.result.current.status).toEqual(data(!oldValue));
    expect(hook.result.current.error).toBeNull();
    expect(hook.result.current.readError).toBeNull();
});
it('ignores an old error after a newer success', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    await act(() => vi.advanceTimersByTimeAsync(5000));
    await finish(reads[1], true);
    await act(async () => {reads[0].reject({status: 500, error: {error: 'Old failure'}}); await Promise.resolve();});
    expect(hook.result.current.status).toEqual(data(true));
    expect(hook.result.current.error).toBeNull();
    expect(hook.result.current.readError).toBeNull();
});
it('does not clear a newer error with an older success and keeps the last valid status', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    await finish(reads[0], false);
    let old!: Promise<void>, current!: Promise<void>;
    act(() => {old = hook.result.current.refresh(); current = hook.result.current.refresh();});
    await act(async () => {reads[2].reject({status: 500, error: {error: 'Current failure'}}); await current;});
    await act(async () => {reads[1].resolve({data: data(true)}); await old;});
    expect(hook.result.current.status).toEqual(data(false));
    expect(hook.result.current.error).toBe('Current failure');
    expect(hook.result.current.readError).toBe('Current failure');
});
it.each(['manual-newer', 'poll-newer'])('orders manual and automatic reads: %s', async order => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    await finish(reads[0], false);
    let manual!: Promise<void>;
    if (order === 'manual-newer') {
        await act(() => vi.advanceTimersByTimeAsync(5000));
        act(() => {manual = hook.result.current.refresh();});
    } else {
        act(() => {manual = hook.result.current.refresh();});
        await act(() => vi.advanceTimersByTimeAsync(5000));
    }
    expect(reads).toHaveLength(3);
    await finish(reads[2], true);
    await finish(reads[1], false);
    await act(() => manual);
    expect(hook.result.current.status).toEqual(data(true));
    expect(mock.request).toHaveBeenCalledTimes(3);
    for (const [options] of mock.request.mock.calls) {
        expect(options).toEqual({path: '/calibration/status', method: 'GET', format: 'json'});
    }
});
it('continues publishing 12-second reads while newer polls are pending', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    await act(() => vi.advanceTimersByTimeAsync(10000));
    expect(reads).toHaveLength(3);
    for (let id = 0; id < 4; id++) {
        await act(() => vi.advanceTimersByTimeAsync(id === 0 ? 2000 : 5000));
        expect(reads.length).toBeGreaterThan(id + 1);
        await finish(reads[id], id % 2 === 0);
        expect(hook.result.current.status).toEqual(data(id % 2 === 0));
        expect(hook.result.current.error).toBeNull();
    }
    expect(reads).toHaveLength(6);
    for (let id = 4; id < reads.length; id++) await finish(reads[id], true);
});
it('does not regress when slow responses finish out of order', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    await act(() => vi.advanceTimersByTimeAsync(15000));
    expect(reads).toHaveLength(4);
    await finish(reads[2], true);
    await finish(reads[0], false);
    await finish(reads[1], false);
    expect(hook.result.current.status).toEqual(data(true));
    await finish(reads[3], false);
    expect(hook.result.current.status).toEqual(data(false));
});
it('accepts a response with newer manual reads pending, then rejects it once a newer result publishes', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    let first!: Promise<void>, second!: Promise<void>, third!: Promise<void>;
    act(() => {
        first = hook.result.current.refresh();
        second = hook.result.current.refresh();
        third = hook.result.current.refresh();
    });
    await finish(reads[0], true);
    expect(hook.result.current.status).toEqual(data(true));
    await finish(reads[2], false);
    await finish(reads[1], true);
    expect(hook.result.current.status).toEqual(data(false));
    await finish(reads[3], true);
    await act(async () => {await Promise.all([first, second, third]);});
    expect(hook.result.current.status).toEqual(data(true));
    expect(mock.request).toHaveBeenCalledTimes(4);
});
it('treats accepted malformed data as a published error and recovers normally', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    let current!: Promise<void>;
    act(() => {current = hook.result.current.refresh();});
    await act(async () => {reads[1].resolve({data: null}); await current;});
    expect(hook.result.current.readError).toBe('Invalid calibration status');
    await finish(reads[0], true);
    expect(hook.result.current.status).toBeNull();
    expect(hook.result.current.readError).toBe('Invalid calibration status');
    act(() => {current = hook.result.current.refresh();});
    await act(async () => {reads[2].resolve({data: data(false)}); await current;});
    expect(hook.result.current.status).toEqual(data(false));
    expect(hook.result.current.error).toBeNull();
    expect(hook.result.current.readError).toBeNull();
});
it('stops processing a late response after unmount and clears the interval', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus);
    expect(vi.getTimerCount()).toBe(1);
    hook.unmount();
    expect(vi.getTimerCount()).toBe(0);
    await act(() => vi.advanceTimersByTimeAsync(15000));
    expect(mock.request).toHaveBeenCalledOnce();
    const readData = vi.fn(() => data(true));
    await act(async () => {reads[0].resolve({get data() {return readData();}}); await reads[0].promise;});
    expect(readData).not.toHaveBeenCalled();
});
it('does not revive the first lifetime during StrictMode replay', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus, {reactStrictMode: true});
    expect(reads).toHaveLength(2);
    expect(vi.getTimerCount()).toBe(1);
    await finish(reads[0], true);
    expect(hook.result.current.status).toBeNull();
    expect(hook.result.current.error).toBeNull();
    await finish(reads[1], false);
    expect(hook.result.current.status).toEqual(data(false));
    await act(() => vi.advanceTimersByTimeAsync(5000));
    expect(reads).toHaveLength(3);
    await finish(reads[2], true);
    expect(hook.result.current.status).toEqual(data(true));
    hook.unmount();
    expect(vi.getTimerCount()).toBe(0);
});
it('ignores an error from the cleaned-up StrictMode lifetime', async () => {
    vi.useFakeTimers();
    const reads = pendingReads();
    const hook = renderHook(useCalibrationStatus, {reactStrictMode: true});
    await act(async () => {reads[0].reject({status: 500, error: {error: 'Old lifetime'}}); await Promise.resolve();});
    expect(hook.result.current.error).toBeNull();
    expect(hook.result.current.readError).toBeNull();
    await finish(reads[1], true);
    expect(hook.result.current.status).toEqual(data(true));
});
it.each([
    {name: 'absent data', response: {data: undefined}, error: 'Invalid calibration status', readError: 'Invalid calibration status'},
    {name: 'unreadable JSON', response: {ok: false, status: 200, error: new SyntaxError('Broken JSON')}, error: 'Broken JSON', readError: 'Broken JSON'},
    {name: 'HTTP 503', response: {ok: false, status: 503}, error: 'Read request failed', readError: null},
])('preserves the current classification for $name', async ({response, error, readError}) => {
    const hookData = data(true);
    mock.request.mockResolvedValueOnce({data: hookData});
    const hook = renderHook(useCalibrationStatus);
    await waitFor(() => expect(hook.result.current.status).toEqual(hookData));
    mock.request.mockResolvedValueOnce(response);
    await act(() => hook.result.current.refresh());
    expect(hook.result.current.error).toBe(error);
    expect(hook.result.current.readError).toBe(readError);
    expect(hook.result.current.status).toEqual(hookData);
    mock.request.mockResolvedValueOnce({data: data(false)});
    await act(() => hook.result.current.refresh());
    expect(hook.result.current.status).toEqual(data(false));
    expect(hook.result.current.error).toBeNull();
    expect(hook.result.current.readError).toBeNull();
});
