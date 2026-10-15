import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi, type Mock} from 'vitest';
import {DEFAULT_PROFILE, type ControllerAction} from './controller';
import {useBrowserGamepad} from './useBrowserGamepad';

const topics = vi.hoisted(() => ({
    status: {data: {state_name: 'IDLE', emergency: false}, lastMessageAt: null as number | null},
    emergency: {data: {active_emergency: false, latched_emergency: false}, lastMessageAt: null as number | null},
}));

vi.mock('../hooks/useTopic', () => ({
    useTopic: (key: string) => key === 'highLevelStatus' ? topics.status : topics.emergency,
}));

class FakeWebSocket {
    static CONNECTING = 0;
    static OPEN = 1;
    static CLOSING = 2;
    static CLOSED = 3;
    static instances: FakeWebSocket[] = [];

    readyState = FakeWebSocket.CONNECTING;
    bufferedAmount = 0;
    readonly sent: string[] = [];
    closeCount = 0;
    onclose: ((event: CloseEvent) => unknown) | null = null;
    onerror: ((event: Event) => unknown) | null = null;

    constructor(readonly url: string) {
        FakeWebSocket.instances.push(this);
    }

    open() { this.readyState = FakeWebSocket.OPEN; }
    send(value: string) { this.sent.push(value); }
    close() {
        this.closeCount++;
        this.readyState = FakeWebSocket.CLOSED;
        this.onclose?.({} as CloseEvent);
    }
    serverClose() {
        this.readyState = FakeWebSocket.CLOSED;
        this.onclose?.({} as CloseEvent);
    }
    fail() { this.onerror?.({} as Event); }
}

let pad: Gamepad;
let originalGetGamepads: PropertyDescriptor | undefined;
let originalSecureContext: PropertyDescriptor | undefined;
let originalVisibility: PropertyDescriptor | undefined;

type ControlMessage = {
    deadman: boolean;
    twist: {twist: {linear: {x: number}; angular: {z: number}}};
};
type RunAction = (action: ControllerAction) => Promise<void>;
type OwnershipChange = (owns: boolean) => void;

function parseControl(raw: string): ControlMessage {
    const parsed: unknown = JSON.parse(raw);
    if (!parsed || typeof parsed !== 'object') throw new Error('Invalid control frame');
    return parsed as ControlMessage;
}

function makePad(): Gamepad {
    const gamepad = {
        id: 'test pad', index: 0, mapping: 'standard', connected: true,
        axes: [0, 0],
        buttons: Array.from({length: 8}, () => ({pressed: false, touched: false, value: 0})),
    } as unknown as Gamepad;
    Object.defineProperty(gamepad, 'timestamp', {get: () => Math.max(1, performance.now())});
    return gamepad;
}

function setButton(index: number, down: boolean) {
    const button = pad.buttons[index] as {pressed: boolean; value: number};
    button.pressed = down;
    button.value = down ? 1 : 0;
}

function setAxis(index: number, value: number) {
    (pad.axes as number[])[index] = value;
}

function poll(ms = 50) {
    act(() => { vi.advanceTimersByTime(ms); });
}

function mount(
    runAction: Mock<RunAction> = vi.fn<RunAction>(async () => {}),
    onOwnershipChange: Mock<OwnershipChange> = vi.fn<OwnershipChange>(),
) {
    const hook = renderHook(() => useBrowserGamepad({
        profile: structuredClone(DEFAULT_PROFILE), selected: {id: 'test pad', index: 0}, runAction, onOwnershipChange,
    }));
    return {hook, runAction, onOwnershipChange};
}

function enable(hook: ReturnType<typeof mount>['hook']) {
    act(() => { hook.result.current.enable(); });
    const socket = FakeWebSocket.instances[FakeWebSocket.instances.length - 1];
    socket.open();
    return socket;
}

beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date('2026-01-01T00:00:00Z'));
    FakeWebSocket.instances = [];
    pad = makePad();
    topics.status = {data: {state_name: 'IDLE', emergency: false}, lastMessageAt: Date.now()};
    topics.emergency = {data: {active_emergency: false, latched_emergency: false}, lastMessageAt: Date.now()};

    originalGetGamepads = Object.getOwnPropertyDescriptor(navigator, 'getGamepads');
    Object.defineProperty(navigator, 'getGamepads', {
        configurable: true, value: () => [pad] as ReturnType<typeof navigator.getGamepads>,
    });
    originalSecureContext = Object.getOwnPropertyDescriptor(window, 'isSecureContext');
    Object.defineProperty(window, 'isSecureContext', {configurable: true, value: true});
    originalVisibility = Object.getOwnPropertyDescriptor(document, 'visibilityState');
    Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'});
    vi.spyOn(document, 'hasFocus').mockReturnValue(true);
    vi.stubGlobal('WebSocket', FakeWebSocket);
});

afterEach(() => {
    vi.useRealTimers();
    vi.restoreAllMocks();
    vi.unstubAllGlobals();
    if (originalGetGamepads) Object.defineProperty(navigator, 'getGamepads', originalGetGamepads);
    else Reflect.deleteProperty(navigator, 'getGamepads');
    if (originalSecureContext) Object.defineProperty(window, 'isSecureContext', originalSecureContext);
    else Reflect.deleteProperty(window, 'isSecureContext');
    if (originalVisibility) Object.defineProperty(document, 'visibilityState', originalVisibility);
    else Reflect.deleteProperty(document, 'visibilityState');
});

describe('useBrowserGamepad session ownership', () => {
    it('opens only an explicit session, sends neutral on disable, and releases ownership once', () => {
        const {hook, onOwnershipChange} = mount();
        const socket = enable(hook);
        expect(hook.result.current.enabled).toBe(true);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true]);

        poll();
        const control = parseControl(socket.sent[socket.sent.length - 1]);
        expect(Object.keys(control).sort()).toEqual(['deadman', 'twist']);
        expect(control).toMatchObject({deadman: false, twist: {twist: {linear: {x: 0}, angular: {z: 0}}}});

        act(() => { hook.result.current.disable(); });
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: false, twist: {twist: {linear: {x: 0}, angular: {z: 0}}}});
        expect(socket.closeCount).toBe(1);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false]);
        expect(hook.result.current.enabled).toBe(false);

        poll(1000);
        expect(FakeWebSocket.instances).toHaveLength(1);
    });

    it('treats hotplug and focus loss as terminal faults, without inheriting or reconnecting', () => {
        const {hook, onOwnershipChange} = mount();
        const socket = enable(hook);
        poll();
        const event = Object.assign(new Event('gamepadconnected'), {gamepad: pad});
        act(() => { window.dispatchEvent(event); });
        expect(hook.result.current.fault).toBe('inputLost');
        expect(hook.result.current.enabled).toBe(false);
        expect(socket.closeCount).toBe(1);
        poll(1000);
        expect(FakeWebSocket.instances).toHaveLength(1);

        const secondSocket = enable(hook);
        poll();
        vi.spyOn(document, 'hasFocus').mockReturnValue(false);
        poll();
        expect(hook.result.current.fault).toBe('inputLost');
        expect(secondSocket.closeCount).toBe(1);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false, true, false]);
        expect(FakeWebSocket.instances).toHaveLength(2);
    });

    it('does not queue behind a buffered socket and requires an explicit new enable after connection loss', () => {
        const {hook} = mount();
        const socket = enable(hook);
        poll();
        const sentBeforeBackpressure = socket.sent.length;
        socket.bufferedAmount = 1;
        poll();
        expect(hook.result.current.fault).toBe('connectionLost');
        expect(socket.closeCount).toBe(1);
        expect(socket.sent).toHaveLength(sentBeforeBackpressure);
        poll(1000);
        expect(FakeWebSocket.instances).toHaveLength(1);

        act(() => { hook.result.current.enable(); });
        expect(FakeWebSocket.instances).toHaveLength(2);
    });

    it('treats a delayed poll as terminal input loss', () => {
        let monotonic = 1000;
        vi.spyOn(performance, 'now').mockImplementation(() => monotonic);
        const {hook} = mount();
        const socket = enable(hook);
        poll();
        monotonic += 201;
        poll();
        expect(hook.result.current.fault).toBe('inputLost');
        expect(socket.closeCount).toBe(1);
        expect(FakeWebSocket.instances).toHaveLength(1);
    });

    it('stops and releases ownership on unmount', () => {
        const {hook, onOwnershipChange} = mount();
        const socket = enable(hook);
        poll();
        const sentBeforeUnmount = socket.sent.length;
        hook.unmount();
        expect(socket.sent).toHaveLength(sentBeforeUnmount + 1);
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: false, twist: {twist: {linear: {x: 0}, angular: {z: 0}}}});
        expect(socket.closeCount).toBe(1);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false]);
    });

    it('terminates the session when a held action promise rejects', async () => {
        let rejectAction!: (reason: Error) => void;
        const runAction = vi.fn<RunAction>(() => new Promise<void>((_, reject) => { rejectAction = reject; }));
        const {hook, onOwnershipChange} = mount(runAction);
        const socket = enable(hook);
        poll(); // Mode transition.
        poll(); // Initial neutral interlock.
        setButton(DEFAULT_PROFILE.buttons.recordStart, true);
        poll(); // Start the hold.
        poll(800); // Hold reaches 800 ms.
        expect(runAction).toHaveBeenCalledTimes(1);
        expect(runAction).toHaveBeenCalledWith('recordStart');

        await act(async () => {
            rejectAction(new Error('action failed'));
            await Promise.resolve();
            await Promise.resolve();
        });
        expect(hook.result.current.fault).toBe('actionFailed');
        expect(hook.result.current.enabled).toBe(false);
        expect(socket.closeCount).toBe(1);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false]);
    });

    it('lets STOP bypass a pending recording action and reasserts STOP after it settles', async () => {
        let resolveStart!: () => void;
        let resolveReassert!: () => void;
        const startRequest = new Promise<void>(resolve => { resolveStart = resolve; });
        let stopCalls = 0;
        const runAction = vi.fn<RunAction>(action => {
            if (action === 'recordStart') return startRequest;
            stopCalls++;
            return stopCalls === 1 ? Promise.resolve() : new Promise<void>(resolve => { resolveReassert = resolve; });
        });
        const {hook} = mount(runAction);
        const socket = enable(hook);
        poll(); // Mode transition.
        poll(); // Initial neutral interlock.
        setButton(DEFAULT_PROFILE.buttons.recordStart, true);
        poll(); // Begin recordStart hold.
        poll(800); // Start request remains unresolved.
        expect(runAction.mock.calls.map(([action]) => action)).toEqual(['recordStart']);
        const actionStartFrame = socket.sent.length;

        setButton(DEFAULT_PROFILE.buttons.recordStart, false);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        setAxis(1, -1);
        poll(); // Pending request must suppress movement.
        setButton(DEFAULT_PROFILE.buttons.stop, true);
        poll(); // STOP bypasses the pending start.
        expect(runAction.mock.calls.map(([action]) => action)).toEqual(['recordStart', 'stop']);

        setButton(DEFAULT_PROFILE.buttons.stop, false);
        poll();
        await act(async () => {
            resolveStart();
            await Promise.resolve();
            await Promise.resolve();
            await Promise.resolve();
        });
        expect(runAction.mock.calls.map(([action]) => action)).toEqual(['recordStart', 'stop', 'stop']);
        poll(); // The reasserted STOP is still pending, so input remains neutral.

        const pendingFrames = socket.sent.slice(actionStartFrame).map(parseControl);
        expect(pendingFrames.length).toBeGreaterThan(0);
        expect(pendingFrames.every(({deadman, twist}) =>
            !deadman && twist.twist.linear.x === 0 && twist.twist.angular.z === 0,
        )).toBe(true);
        await act(async () => {
            resolveReassert();
            await Promise.resolve();
            await Promise.resolve();
        });
    });

    it('ignores late action completion after the server closes the manual session', async () => {
        let rejectAction!: (reason: Error) => void;
        const runAction = vi.fn<RunAction>(() => new Promise<void>((_, reject) => { rejectAction = reject; }));
        const {hook, onOwnershipChange} = mount(runAction);
        const socket = enable(hook);
        poll();
        poll();
        setButton(DEFAULT_PROFILE.buttons.recordStart, true);
        poll();
        poll(800);
        expect(runAction).toHaveBeenCalledTimes(1);

        act(() => { socket.serverClose(); });
        expect(hook.result.current.fault).toBe('connectionLost');
        const sentWhenClosed = socket.sent.length;
        poll(1000);
        await act(async () => {
            rejectAction(new Error('late action failure'));
            await Promise.resolve();
            await Promise.resolve();
        });
        expect(runAction).toHaveBeenCalledTimes(1);
        expect(socket.sent).toHaveLength(sentWhenClosed);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false]);
        expect(hook.result.current.fault).toBe('connectionLost');
    });
});
