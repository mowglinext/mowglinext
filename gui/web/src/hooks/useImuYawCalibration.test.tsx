import {act, cleanup, renderHook} from '@testing-library/react';
import {useEffect} from 'react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useImuYawCalibration} from './useImuYawCalibration';

const mock = vi.hoisted(() => ({
    fetch: vi.fn<typeof fetch>(),
    error: vi.fn<(config: {message: string; description: string}) => void>(),
    success: vi.fn<(config: {message: string; description: string; duration: number}) => void>(),
    t: (key: string) => key,
}));
vi.mock('antd', () => ({App: {useApp: () => ({notification: mock})}}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: mock.t})}));

const complete = {
    success: true, message: 'Complete', imu_yaw_rad: .123456, imu_yaw_deg: 7.07,
    samples_used: 250, std_dev_deg: .2,
    imu_pitch_rad: .012345, imu_pitch_deg: .7073, imu_roll_rad: -.023456, imu_roll_deg: -1.344,
    stationary_samples_used: 150, gravity_mag_mps2: 9.81,
    dock_valid: true, dock_pose_x: 1, dock_pose_y: 2, dock_pose_yaw_rad: .5,
    dock_pose_yaw_deg: 28.65, dock_yaw_sigma_deg: .3, dock_undock_displacement_m: 2,
};
const minimal = {success: true, message: 'Complete', imu_yaw_rad: 0, imu_yaw_deg: 0, samples_used: 1, std_dev_deg: 0};
function reply(data: unknown) {
    // Preserve non-finite boundary witnesses: JSON.stringify would turn them into null.
    const response = new Response();
    vi.spyOn(response, 'json').mockResolvedValue(data);
    mock.fetch.mockResolvedValue(response);
}
function setup() {
    const onApplyValue = vi.fn<(key: string, value: number) => void>();
    const hook = renderHook(() => useImuYawCalibration({onApplyValue, currentImuYawRad: .1}));
    return {...hook, onApplyValue};
}
beforeEach(() => {
    vi.clearAllMocks(); mock.fetch.mockReset();
    vi.stubGlobal('fetch', mock.fetch);
});
afterEach(() => {cleanup(); vi.unstubAllGlobals(); vi.restoreAllMocks(); vi.useRealTimers();});

it('rejects the reproduced incomplete response before exposing it or applying any value', async () => {
    reply({success: true, message: 'fixture', imu_yaw_rad: .25});
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(hook.result.current.calibResult).toBeNull();
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue).not.toHaveBeenCalled();
    expect(mock.error).toHaveBeenCalledOnce();
});
it('rejects missing degrees in an applicable pitch/roll pair before applying even yaw', async () => {
    reply({...complete, imu_roll_deg: undefined});
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(hook.result.current.calibResult).toBeNull();
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue).not.toHaveBeenCalled();
    expect(mock.error).toHaveBeenCalledOnce();
});

it('preserves a complete result, request parameters, rounding and successful application', async () => {
    reply(complete);
    const hook = setup();
    act(() => hook.result.current.openCalibration());
    await act(() => hook.result.current.startCalibration());
    expect(mock.fetch).toHaveBeenCalledOnce();
    const [url, options] = mock.fetch.mock.calls[0];
    expect(url).toBe('/api/calibration/imu-yaw');
    expect(options).toMatchObject({method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({duration_sec: 30})});
    expect(options?.signal).toBeInstanceOf(AbortSignal);
    expect(hook.result.current.calibResult).toBe(complete);
    expect(hook.result.current.calibRunning).toBe(false);
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue.mock.calls).toEqual([['imu_yaw', .1235], ['imu_pitch', .0123], ['imu_roll', -.0235]]);
    expect(mock.error).not.toHaveBeenCalled();
    expect(mock.success).toHaveBeenCalledOnce();
    const notice = mock.success.mock.calls[0][0];
    expect(notice).toMatchObject({message: 'imuYawCalibration.applied', duration: 6});
    expect(notice.description).toContain('dock_pose_yaw = 28.65° (σ0.30°, displacement 2.00 m, persisted to mowgli_robot.yaml)');
    expect(hook.result.current.calibResult).toBeNull();
    expect(hook.result.current.calibOpen).toBe(false);
});
it.each(['imu_yaw_rad', 'imu_pitch_rad', 'imu_roll_rad'].flatMap(field =>
    [Number.MAX_VALUE, -Number.MAX_VALUE].map(value => ({field, value})),
))('rejects non-finite derived candidates atomically: $field = $value', async ({field, value}) => {
    const data = {...complete, [field]: value};
    mock.fetch.mockResolvedValue(new Response(JSON.stringify(data)));
    const hook = setup();
    act(() => hook.result.current.openCalibration());
    await act(() => hook.result.current.startCalibration());
    // The HTTP response contains finite source numbers and passes the existing validator.
    expect(hook.result.current.calibResult).toEqual(data);
    expect(mock.error).not.toHaveBeenCalled();
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue).not.toHaveBeenCalled();
    expect(mock.success).not.toHaveBeenCalled();
    expect(mock.error).toHaveBeenCalledExactlyOnceWith({
        message: 'imuYawCalibration.failed', description: 'asyncButton.errorOccurred',
    });
    expect(hook.result.current.calibOpen).toBe(true);
    expect(hook.result.current.calibResult).toEqual(data);
    expect(mock.fetch).toHaveBeenCalledOnce();
    act(() => hook.result.current.closeCalibration());
    expect(hook.result.current.calibOpen).toBe(false);
    expect(hook.result.current.calibResult).toBeNull();
});
it.each([
    [0, 0], [.000049, 0], [.000051, .0001], [-.000051, -.0001],
    [1e-12, 0], [-1e-12, -0], [.123456, .1235], [-.123456, -.1235],
])('preserves four-decimal rounding for finite candidate %s', async (value, expected) => {
    reply({...complete, imu_yaw_rad: value, imu_pitch_rad: value, imu_roll_rad: value});
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue.mock.calls).toEqual([
        ['imu_yaw', expected], ['imu_pitch', expected], ['imu_roll', expected],
    ]);
    expect(mock.error).not.toHaveBeenCalled();
    expect(mock.success).toHaveBeenCalledOnce();
});
it('does not apply or reject unused pitch/roll candidates below the stationary gate', async () => {
    reply({...complete, stationary_samples_used: 149, imu_pitch_rad: Number.MAX_VALUE, imu_roll_rad: -Number.MAX_VALUE});
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue.mock.calls).toEqual([['imu_yaw', .1235]]);
    expect(mock.error).not.toHaveBeenCalled();
    expect(mock.success).toHaveBeenCalledOnce();
});
it.each([
    {success: false, message: 'Insufficient motion'},
    {...complete, success: false, message: 'Rejected by producer'},
])('preserves a valid business failure without promoting it to an exception: $message', async data => {
    reply(data);
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(hook.result.current.calibResult).toBe(data);
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue).not.toHaveBeenCalled();
    expect(mock.error).not.toHaveBeenCalled();
    expect(mock.success).not.toHaveBeenCalled();
});
it('keeps zero measurements, absent optional fields and additive data without inventing measurements', async () => {
    const data = {...minimal, extension: {producer: 'fixture'}};
    reply(data);
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(hook.result.current.calibResult).toBe(data);
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue.mock.calls).toEqual([['imu_yaw', 0]]);
});
it.each([149, 150])('retains the existing stationary threshold at %s samples', async stationary_samples_used => {
    reply({...complete, stationary_samples_used});
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue.mock.calls).toEqual(stationary_samples_used < 150
        ? [['imu_yaw', .1235]] : [['imu_yaw', .1235], ['imu_pitch', .0123], ['imu_roll', -.0235]]);
});
it('does not require degrees from optional branches that the existing conditions do not use', async () => {
    const data = {...minimal, stationary_samples_used: 149, imu_pitch_rad: .1, imu_roll_rad: .2,
        dock_valid: false, dock_pose_yaw_rad: .5};
    reply(data);
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(hook.result.current.calibResult).toBe(data);
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue.mock.calls).toEqual([['imu_yaw', 0]]);
});

const numericFields = ['imu_yaw_rad', 'imu_yaw_deg', 'samples_used', 'std_dev_deg',
    'imu_pitch_rad', 'imu_pitch_deg', 'imu_roll_rad', 'imu_roll_deg', 'stationary_samples_used',
    'gravity_mag_mps2', 'dock_pose_x', 'dock_pose_y', 'dock_pose_yaw_rad', 'dock_pose_yaw_deg',
    'dock_yaw_sigma_deg', 'dock_undock_displacement_m'];
describe('malformed responses never become applicable', () => {
    const malformed: unknown[] = [null, [], {}, 'Complete',
        {...minimal, success: 'true'}, {...minimal, message: 42},
        {success: false}, {success: false, message: {}},
        {...minimal, imu_yaw_rad: '0'}, {...minimal, samples_used: null},
        {...complete, dock_valid: 'true'}, {...complete, dock_pose_x: '1'},
        ...['message', 'imu_yaw_rad', 'imu_yaw_deg', 'samples_used', 'std_dev_deg'].map(key => ({...minimal, [key]: undefined})),
        ...['imu_pitch_deg', 'imu_roll_deg', 'dock_pose_yaw_deg', 'dock_yaw_sigma_deg'].map(key => ({...complete, [key]: undefined})),
    ];
    it.each(malformed.map((data, index) => ({data, index})))('rejects incomplete or incorrect types, case $index', async ({data}) => {
        reply(data);
        const hook = setup();
        await act(() => hook.result.current.startCalibration());
        expect(hook.result.current.calibResult).toBeNull();
        expect(hook.result.current.calibRunning).toBe(false);
        act(() => hook.result.current.applyCalibration());
        expect(hook.onApplyValue).not.toHaveBeenCalled();
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.error).toHaveBeenCalledWith({message: 'imuYawCalibration.failed', description: 'Invalid IMU yaw calibration response'});
    });
    it.each(numericFields.flatMap(field => [NaN, Infinity, -Infinity].map(value => ({field, value}))))(
        'rejects non-finite $field=$value before any application', async ({field, value}) => {
            reply({...complete, [field]: value});
            const hook = setup();
            await act(() => hook.result.current.startCalibration());
            expect(hook.result.current.calibResult).toBeNull();
            act(() => hook.result.current.applyCalibration());
            expect(hook.onApplyValue).not.toHaveBeenCalled();
            expect(mock.error).toHaveBeenCalledOnce();
        });
});

it.each([
    {error: new TypeError('Failed to fetch'), description: 'Failed to fetch'},
    {error: new DOMException('Aborted', 'AbortError'), description: 'imuYawCalibration.timedOut'},
    {error: {name: 'AbortError'}, description: 'imuYawCalibration.timedOut'},
    {error: {message: 'Network unavailable'}, description: 'Network unavailable'},
    {error: 'Offline', description: 'Offline'},
    {error: new Error(''), description: 'Error'},
])('retains the existing error notification: $description', async ({error, description}) => {
    mock.fetch.mockRejectedValue(error);
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(mock.error).toHaveBeenCalledWith({message: 'imuYawCalibration.failed', description});
    expect(hook.result.current.calibRunning).toBe(false);
    expect(hook.result.current.calibResult).toBeNull();
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue).not.toHaveBeenCalled();
});
it('keeps HTTP failures on the existing error path', async () => {
    mock.fetch.mockResolvedValue(new Response('Service unavailable', {status: 503}));
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(mock.error).toHaveBeenCalledWith({message: 'imuYawCalibration.failed', description: 'HTTP 503: Service unavailable'});
    expect(hook.result.current.calibResult).toBeNull();
});
it('keeps JSON parsing errors on the existing error path', async () => {
    mock.fetch.mockResolvedValue(new Response('not json'));
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(mock.error).toHaveBeenCalledOnce();
    expect(hook.result.current.calibResult).toBeNull();
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue).not.toHaveBeenCalled();
});
it('keeps the 155-second abort deadline and clears its timer after completion', async () => {
    vi.useFakeTimers();
    let signal: AbortSignal | undefined;
    mock.fetch.mockImplementation((_url, init) => new Promise((_resolve, reject) => {
        if (!init?.signal) throw new Error('Missing abort signal');
        signal = init.signal;
        signal.addEventListener('abort', () => reject(new DOMException('Aborted', 'AbortError')), {once: true});
    }));
    const hook = setup();
    let pending: Promise<void> | undefined;
    act(() => {pending = hook.result.current.startCalibration();});
    expect(hook.result.current.calibRunning).toBe(true);
    await act(() => vi.advanceTimersByTimeAsync(154999));
    expect(signal?.aborted).toBe(false);
    await act(async () => {await vi.advanceTimersByTimeAsync(1); await pending;});
    expect(signal?.aborted).toBe(true);
    expect(hook.result.current.calibRunning).toBe(false);
    expect(mock.error).toHaveBeenCalledWith({message: 'imuYawCalibration.failed', description: 'imuYawCalibration.timedOut'});
    expect(vi.getTimerCount()).toBe(0);
});
it('clears the deadline after a valid response as well', async () => {
    vi.useFakeTimers(); reply(minimal);
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(vi.getTimerCount()).toBe(0);
    await act(() => vi.advanceTimersByTimeAsync(155000));
    expect(mock.error).not.toHaveBeenCalled();
});

it('does not retain a previous applicable result after a rejected subsequent response', async () => {
    reply(complete);
    const hook = setup();
    await act(() => hook.result.current.startCalibration());
    expect(hook.result.current.calibResult).toBe(complete);
    reply({...minimal, imu_yaw_deg: undefined});
    await act(() => hook.result.current.startCalibration());
    expect(hook.result.current.calibResult).toBeNull();
    act(() => hook.result.current.applyCalibration());
    expect(hook.onApplyValue).not.toHaveBeenCalled();
    expect(mock.error).toHaveBeenCalledOnce();
});

describe('ownership of asynchronous calibration results', () => {
    function deferred<T>() {
        let resolve!: (value: T) => void;
        let reject!: (error: unknown) => void;
        const promise = new Promise<T>((yes, no) => {resolve = yes; reject = no;});
        return {promise, resolve, reject};
    }
    it('invalidates a pending fetch on Reset without aborting the remote request', async () => {
        const pending = deferred<Response>();
        mock.fetch.mockReturnValue(pending.promise);
        const hook = setup();
        let task!: Promise<void>;
        act(() => {task = hook.result.current.startCalibration();});
        act(() => hook.result.current.resetCalibration());
        const response = new Response(JSON.stringify(complete));
        const json = vi.spyOn(response, 'json');
        await act(async () => {pending.resolve(response); await task;});
        expect(json).not.toHaveBeenCalled();
        expect(hook.result.current.calibResult).toBeNull();
        expect(hook.result.current.calibRunning).toBe(false);
        act(() => hook.result.current.applyCalibration());
        expect(hook.onApplyValue).not.toHaveBeenCalled();
        expect(mock.error).not.toHaveBeenCalled();
        expect(mock.success).not.toHaveBeenCalled();
        expect(mock.fetch).toHaveBeenCalledOnce();
        expect(mock.fetch.mock.calls[0][1]?.signal?.aborted).toBe(false);
    });
    it('invalidates a body decoded after Reset even when fetch has already completed', async () => {
        const body = deferred<unknown>();
        const response = new Response();
        const json = vi.spyOn(response, 'json').mockReturnValue(body.promise);
        mock.fetch.mockResolvedValue(response);
        const hook = setup();
        let task!: Promise<void>;
        await act(async () => {task = hook.result.current.startCalibration(); await Promise.resolve();});
        expect(json).toHaveBeenCalledOnce();
        act(() => hook.result.current.resetCalibration());
        await act(async () => {body.resolve(complete); await task;});
        expect(hook.result.current.calibResult).toBeNull();
        expect(hook.result.current.calibRunning).toBe(false);
        expect(mock.error).not.toHaveBeenCalled();
        expect(mock.success).not.toHaveBeenCalled();
        expect(hook.onApplyValue).not.toHaveBeenCalled();
    });
    it('ignores an error after Reset', async () => {
        const pending = deferred<Response>();
        mock.fetch.mockReturnValue(pending.promise);
        const hook = setup();
        let task!: Promise<void>;
        act(() => {task = hook.result.current.startCalibration();});
        act(() => hook.result.current.resetCalibration());
        await act(async () => {pending.reject(new TypeError('Old failure')); await task;});
        expect(mock.error).not.toHaveBeenCalled();
        expect(mock.success).not.toHaveBeenCalled();
        expect(hook.result.current.calibResult).toBeNull();
        expect(hook.result.current.calibRunning).toBe(false);
    });
    it.each(['old-first', 'new-first'])('only publishes the latest invocation: %s', async order => {
        const old = deferred<Response>(), current = deferred<Response>();
        mock.fetch.mockReturnValueOnce(old.promise).mockReturnValueOnce(current.promise);
        const hook = setup();
        let first!: Promise<void>, second!: Promise<void>;
        act(() => {first = hook.result.current.startCalibration(); second = hook.result.current.startCalibration();});
        const oldResponse = new Response(JSON.stringify({...minimal, imu_yaw_rad: .1}));
        const newResponse = new Response(JSON.stringify({...minimal, imu_yaw_rad: .2}));
        if (order === 'old-first') {
            await act(async () => {old.resolve(oldResponse); await first;});
            expect(hook.result.current.calibRunning).toBe(true);
            expect(hook.result.current.calibResult).toBeNull();
            await act(async () => {current.resolve(newResponse); await second;});
        } else {
            await act(async () => {current.resolve(newResponse); await second;});
            expect(hook.result.current.calibRunning).toBe(false);
            await act(async () => {old.resolve(oldResponse); await first;});
        }
        expect(hook.result.current.calibResult).toMatchObject({imu_yaw_rad: .2});
        expect(hook.result.current.calibRunning).toBe(false);
        expect(hook.onApplyValue).not.toHaveBeenCalled();
        expect(mock.success).not.toHaveBeenCalled();
        act(() => hook.result.current.applyCalibration());
        expect(hook.onApplyValue).toHaveBeenCalledExactlyOnceWith('imu_yaw', .2);
        expect(mock.success).toHaveBeenCalledOnce();
        expect(mock.error).not.toHaveBeenCalled();
        expect(mock.fetch).toHaveBeenCalledTimes(2);
    });
    it('ignores an old error after the current invocation succeeds', async () => {
        const old = deferred<Response>();
        mock.fetch.mockReturnValueOnce(old.promise).mockResolvedValueOnce(new Response(JSON.stringify(minimal)));
        const hook = setup();
        let first!: Promise<void>;
        act(() => {first = hook.result.current.startCalibration();});
        await act(() => hook.result.current.startCalibration());
        await act(async () => {old.reject(new TypeError('Old failure')); await first;});
        expect(mock.error).not.toHaveBeenCalled();
        expect(hook.result.current.calibResult).toEqual(minimal);
        expect(hook.result.current.calibRunning).toBe(false);
        expect(hook.onApplyValue).not.toHaveBeenCalled();
    });
    it('ignores an error after unmount and clears its timeout', async () => {
        vi.useFakeTimers();
        const pending = deferred<Response>();
        mock.fetch.mockReturnValue(pending.promise);
        const hook = setup();
        let task!: Promise<void>;
        act(() => {task = hook.result.current.startCalibration();});
        hook.unmount();
        expect(mock.fetch.mock.calls[0][1]?.signal?.aborted).toBe(false);
        await act(async () => {pending.reject(new TypeError('Unmounted failure')); await task;});
        expect(mock.error).not.toHaveBeenCalled();
        expect(mock.success).not.toHaveBeenCalled();
        expect(hook.onApplyValue).not.toHaveBeenCalled();
        expect(vi.getTimerCount()).toBe(0);
    });
    it('clears each obsolete timeout without clearing the current invocation timeout', async () => {
        vi.useFakeTimers();
        const old = deferred<Response>(), current = deferred<Response>();
        mock.fetch.mockReturnValueOnce(old.promise).mockReturnValueOnce(current.promise);
        const hook = setup();
        let first!: Promise<void>, second!: Promise<void>;
        act(() => {first = hook.result.current.startCalibration(); second = hook.result.current.startCalibration();});
        expect(vi.getTimerCount()).toBe(2);
        await act(async () => {old.resolve(new Response(JSON.stringify(minimal))); await first;});
        expect(vi.getTimerCount()).toBe(1);
        expect(hook.result.current.calibRunning).toBe(true);
        await act(async () => {current.resolve(new Response(JSON.stringify(minimal))); await second;});
        expect(vi.getTimerCount()).toBe(0);
        expect(hook.result.current.calibResult).toEqual(minimal);
    });
    it('does not revive an identity during StrictMode effect replay', async () => {
        const old = deferred<Response>(), current = deferred<Response>();
        mock.fetch.mockReturnValueOnce(old.promise).mockReturnValueOnce(current.promise);
        const tasks: Promise<void>[] = [];
        const onApplyValue = vi.fn<(key: string, value: number) => void>();
        const hook = renderHook(() => {
            const calibration = useImuYawCalibration({onApplyValue});
            const start = calibration.startCalibration;
            useEffect(() => {tasks.push(start());}, [start]);
            return calibration;
        }, {reactStrictMode: true});
        expect(tasks).toHaveLength(2);
        await act(async () => {old.reject(new TypeError('Replayed mount')); await tasks[0];});
        expect(mock.error).not.toHaveBeenCalled();
        expect(hook.result.current.calibRunning).toBe(true);
        await act(async () => {current.resolve(new Response(JSON.stringify(minimal))); await tasks[1];});
        expect(hook.result.current.calibResult).toEqual(minimal);
        expect(hook.result.current.calibRunning).toBe(false);
        act(() => hook.result.current.applyCalibration());
        expect(onApplyValue).toHaveBeenCalledExactlyOnceWith('imu_yaw', 0);
    });
});
