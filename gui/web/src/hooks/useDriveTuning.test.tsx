import {act, renderHook, waitFor} from '@testing-library/react';
import {beforeEach, expect, it, vi} from 'vitest';
import {useDriveTuning} from './useDriveTuning';
import {validReport, validStart, validStatus} from '../test/driveTuningFixtures';

const mocks = vi.hoisted(() => ({request: vi.fn<(args: {path: string; body?: unknown}) => Promise<{data: unknown; error?: unknown}>>()}));
// useApi is a singleton in production; keep its mock identity stable too.
const api = {request: mocks.request};
vi.mock('./useApi', () => ({useApi: () => api}));
beforeEach(() => {vi.clearAllMocks(); mocks.request.mockResolvedValue({data: validStatus()});});

it('rejects malformed poll responses instead of publishing a successful job', async () => {
    mocks.request.mockResolvedValue({data: {...validStatus(), job: {state: 'succeeded'}}});
    const hook = renderHook(useDriveTuning);
    await waitFor(() => expect(hook.result.current.error).toBe('Invalid drive tuning status'));
    expect(hook.result.current.status).toBeNull(); hook.unmount();
});
it('clears previously trusted status when the newest poll fails', async () => {
    const hook = renderHook(useDriveTuning); await waitFor(() => expect(hook.result.current.status).not.toBeNull());
    mocks.request.mockRejectedValue({error: {error: 'offline'}});
    await act(() => hook.result.current.refresh());
    expect(hook.result.current.status).toBeNull(); expect(hook.result.current.error).toBe('offline'); hook.unmount();
});
it('does not re-poll on rerender, and ignores a delayed response after unmount', async () => {
    let finish: (value: {data: unknown}) => void = () => {};
    mocks.request.mockImplementation(() => new Promise(resolve => {finish = resolve;}));
    const hook = renderHook(useDriveTuning); hook.rerender(); expect(mocks.request).toHaveBeenCalledTimes(1);
    hook.unmount(); await act(async () => {finish({data: validStatus()}); await Promise.resolve();});
    expect(hook.result.current.status).toBeNull();
});
it('keeps the newest status when an older refresh arrives later', async () => {
    const finishes: Array<(value: {data: unknown}) => void> = [];
    mocks.request.mockImplementation(() => new Promise(resolve => {finishes.push(resolve);}));
    const hook = renderHook(useDriveTuning);
    let refresh = Promise.resolve(); act(() => {refresh = hook.result.current.refresh();});
    await act(async () => {finishes[1]({data: validStatus()}); await refresh;});
    await act(async () => {finishes[0]({data: {...validStatus(), pid: {status: 'warning'}}}); await Promise.resolve();});
    expect(hook.result.current.status?.pid.status).toBe('not_validated'); hook.unmount();
});
it.each(['malformed', 'rejected'])('clears a previously loaded report after a %s response', async failure => {
    const hook = renderHook(useDriveTuning); await waitFor(() => expect(hook.result.current.status).not.toBeNull());
    mocks.request.mockResolvedValue({data: validReport()}); await act(() => hook.result.current.loadLatestReport());
    if (failure === 'malformed') mocks.request.mockResolvedValue({data: {parsed: {proposed_params: {ticks_per_meter: 'invalid'}}}});
    else mocks.request.mockRejectedValue(new Error('offline'));
    await act(async () => {await expect(hook.result.current.loadLatestReport()).rejects.toThrow();});
    expect(hook.result.current.latestReport).toBeNull(); expect(hook.result.current.loadingLatestReport).toBe(false); hook.unmount();
});
it('does not announce rollback success for a rejected body', async () => {
    const hook = renderHook(useDriveTuning); await waitFor(() => expect(hook.result.current.status).not.toBeNull());
    mocks.request.mockResolvedValue({data: {success: false, message: 'Rejected'}});
    await act(async () => {await expect(hook.result.current.rollback()).rejects.toThrow('Rejected');}); hook.unmount();
});
it('uses the exact PID request and validates the start acknowledgement', async () => {
    const hook = renderHook(useDriveTuning); await waitFor(() => expect(hook.result.current.status).not.toBeNull());
    mocks.request.mockImplementation(args => Promise.resolve({data: args.path.endsWith('/start') ? {...validStart(), mode: 'pid'} : validStatus()}));
    const body = {apply: false, allow_undock: false, undock_distance_m: 2, passes: 1, max_speed_mps: .3, segment_duration_s: 5};
    await act(() => hook.result.current.startPID(body));
    expect(mocks.request).toHaveBeenCalledWith(expect.objectContaining({path: '/tools/drive/pid-tuning/start', body}));
    const calls = mocks.request.mock.calls.length;
    await act(async () => {await expect(hook.result.current.startPID({...body, max_speed_mps: NaN})).rejects.toThrow();});
    expect(mocks.request).toHaveBeenCalledTimes(calls); hook.unmount();
});

it.each(['failed', 'wrong_mode', 'malformed', 'error_body'])('a %s start response cannot be accepted as successful', async failure => {
    const hook = renderHook(useDriveTuning); await waitFor(() => expect(hook.result.current.status).not.toBeNull());
    const response = failure === 'error_body' ? {data: validStart(), error: {error: 'Rejected'}}
        : {data: failure === 'malformed' ? {} : {...validStart(), state: failure === 'failed' ? 'failed' : 'running', mode: failure === 'wrong_mode' ? 'pid' : 'ff'}};
    mocks.request.mockResolvedValue(response);
    const body = {apply: false, allow_undock: false, undock_distance_m: 2, passes: 1, distance_m: 2, test_speed_mps: .2, odom_timeout_s: 3, auto_turn: false, turn_direction: 'right' as const};
    await act(async () => {await expect(hook.result.current.startFeedForward(body)).rejects.toThrow();});
    hook.unmount();
});
