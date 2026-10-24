import {act, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi, type Mock} from 'vitest';
import {DEFAULT_PROFILE, type ControllerAction} from './controller';
import {useBrowserGamepad} from './useBrowserGamepad';
import type {GamepadDiagnosticReport} from './diagnostics';

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
    readonly claims: string[] = [];
    autoReady = true;
    closeCount = 0;
    closeReason = '';
    onclose: ((event: CloseEvent) => unknown) | null = null;
    onerror: ((event: Event) => unknown) | null = null;
    onmessage: ((event: MessageEvent) => unknown) | null = null;

    constructor(readonly url: string) {
        FakeWebSocket.instances.push(this);
    }

    message(value: unknown) { this.onmessage?.({data: JSON.stringify(value)} as MessageEvent); }
    open(hello = true) {
        this.readyState = FakeWebSocket.OPEN;
        if (hello) this.message({type: 'gamepadHello', version: 3, challenge: '0123456789abcdef0123456789abcdef'});
    }
    send(value: string) {
        const parsed: unknown = JSON.parse(value);
        if (parsed && typeof parsed === 'object' && 'type' in parsed && parsed.type === 'gamepadClaim') {
            this.claims.push(value);
            if (this.autoReady) this.message({type: 'gamepadReady'});
        } else this.sent.push(value);
    }
    close(_code?: number, reason = '') {
        this.closeCount++;
        this.closeReason = reason;
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
const SESSION_TOKEN = '0123456789abcdef0123456789abcdef';
type RunAction = (action: ControllerAction, sessionToken: string) => Promise<void>;
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

function enable(hook: {result: {current: ReturnType<typeof useBrowserGamepad>}}) {
    act(() => { hook.result.current.enable(); });
    const socket = FakeWebSocket.instances[FakeWebSocket.instances.length - 1];
    socket.open();
    poll(); // fresh native sample claims the server challenge
    return socket;
}

beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date('2026-01-01T00:00:00Z'));
    FakeWebSocket.instances = [];
    pad = makePad();
    topics.status = {data: {state_name: 'IDLE', emergency: false}, lastMessageAt: performance.now()};
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
    vi.stubGlobal('fetch', vi.fn<typeof fetch>().mockResolvedValue({ok: true} as Response));
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
    it('claims only after a fresh input poll and waits for server readiness before controls or actions', async () => {
        const {hook, runAction} = mount();
        act(() => { hook.result.current.enable(); });
        const socket = FakeWebSocket.instances[0];
        socket.autoReady = false;
        socket.open(false);
        poll(100);
        expect(socket.claims).toHaveLength(0);
        expect(socket.sent).toHaveLength(0);
        act(() => { socket.message({type: 'gamepadHello', version: 3, challenge: '0123456789abcdef0123456789abcdef'}); });
        expect(socket.claims).toHaveLength(0);
        poll();
        expect(socket.claims).toHaveLength(1);
        expect(hook.result.current.state).toBe('connecting');
        setButton(DEFAULT_PROFILE.buttons.stop, true);
        poll(100);
        expect(socket.sent).toHaveLength(0);
        expect(runAction).not.toHaveBeenCalled();
        act(() => { socket.message({type: 'gamepadReady'}); });
        poll();
        expect(runAction).toHaveBeenCalledExactlyOnceWith('stop', SESSION_TOKEN);
        expect(parseControl(socket.sent[0]).deadman).toBe(false);
        await act(async () => { await Promise.resolve(); });
        hook.unmount();
    });

    it('never claims a late server hello after cancellation or lets it affect the replacement', () => {
        const {hook} = mount();
        act(() => { hook.result.current.enable(); });
        const retired = FakeWebSocket.instances[0];
        retired.open(false);
        poll(2100);
        expect(hook.result.current.fault).toBe('connectionLost');
        expect(retired.claims).toHaveLength(0);
        expect(retired.sent).toHaveLength(0);
        const replacement = enable(hook);
        poll();
        act(() => {
            retired.message({type: 'gamepadHello', version: 3, challenge: 'fedcba9876543210fedcba9876543210'});
            retired.message({type: 'gamepadReady'});
        });
        expect(retired.claims).toHaveLength(0);
        expect(retired.sent).toHaveLength(0);
        expect(replacement.claims).toHaveLength(1);
        expect(hook.result.current.enabled).toBe(true);
        hook.unmount();
    });

    it('does not let a late Ready bypass the application handshake deadline between polls', () => {
        const {hook} = mount();
        act(() => { hook.result.current.enable(); });
        const socket = FakeWebSocket.instances[0];
        socket.autoReady = false;
        socket.open();
        poll(2000);
        expect(hook.result.current.enabled).toBe(true);
        act(() => { vi.advanceTimersByTime(1); socket.message({type: 'gamepadReady'}); });
        expect(hook.result.current.fault).toBe('connectionLost');
        expect(socket.sent).toHaveLength(0);
        hook.unmount();
    });

    it.each([
        {type: 'gamepadReady'},
        {type: 'gamepadHello', challenge: '0123456789abcdef0123456789abcdef'},
        {type: 'gamepadHello', version: 1, challenge: '0123456789abcdef0123456789abcdef'},
        {type: 'gamepadHello', version: 3, challenge: 'too-short'},
        {type: 'gamepadHello', version: 3, challenge: '0123456789abcdef0123456789abcdef', unexpected: true},
        {type: 'unknown'},
    ])('fails closed on an invalid or out-of-order server handshake: %j', message => {
        const {hook, runAction} = mount();
        act(() => { hook.result.current.enable(); });
        const socket = FakeWebSocket.instances[0];
        socket.open(false);
        act(() => { socket.message(message); });
        expect(hook.result.current.fault).toBe('connectionLost');
        expect(socket.claims).toHaveLength(0);
        expect(socket.sent).toHaveLength(0);
        expect(runAction).not.toHaveBeenCalled();
        hook.unmount();
    });

    it('validates current input before claiming instead of using the enable-time preview', () => {
        const {hook} = mount();
        act(() => { hook.result.current.enable(); });
        const socket = FakeWebSocket.instances[0];
        socket.open();
        pad = {...pad, connected: false};
        poll();
        expect(socket.claims).toHaveLength(0);
        expect(socket.sent).toHaveLength(0);
        expect(hook.result.current.inputLoss?.reason).toBe('disconnected');
        hook.unmount();
    });

    it('reports a persistent Gamepad permission failure once instead of uploading on every poll', async () => {
        Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: () => {
            throw new DOMException('Blocked by permissions policy', 'SecurityError');
        }});
        const {hook} = mount();
        await act(async () => { await vi.advanceTimersByTimeAsync(10_000); });
        expect(hook.result.current.fault).toBe('unsupported');
        expect(fetch).toHaveBeenCalledTimes(1);
        expect(FakeWebSocket.instances).toHaveLength(0);
        hook.unmount();
    });

    it('records a PWA disconnect and subsequent retry without losing the first fault', async () => {
        vi.spyOn(window, 'matchMedia').mockReturnValue({...window.matchMedia(''), matches: true});
        topics.status.data.state_name = 'RECORDING';
        const {hook} = mount();
        const socket = enable(hook);
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        poll();
        setAxis(1, -1);
        poll();
        act(() => { socket.serverClose(); });
        setAxis(1, 0);
        setButton(DEFAULT_PROFILE.buttons.deadman, false);
        enable(hook);
        poll();
        await act(async () => { await Promise.resolve(); await Promise.resolve(); });
        const send = vi.mocked(fetch);
        const body = send.mock.calls[send.mock.calls.length - 1][1]?.body;
        if (typeof body !== 'string') throw new Error('Missing diagnostic body');
        const report = JSON.parse(body) as GamepadDiagnosticReport;
        expect(report.snapshot.standalone).toBe(true);
        expect(report.snapshot.mode).toBe('RECORDING');
        expect(report.events.find(event => event.event === 'sessionEnded')?.detail.fault).toBe('connectionLost');
        expect(report.events.filter((event: {event: string}) => event.event === 'enableRequested')).toHaveLength(2);
        expect(new URL(socket.url).searchParams.get('client')).toMatch(/^browser-/);
        const retryFrames = FakeWebSocket.instances[1].sent;
        expect(parseControl(retryFrames[retryFrames.length - 1]).deadman).toBe(false);
        hook.unmount();
    });

    it('keeps a healthy session open beyond two minutes without a maximum-duration timer', () => {
        topics.status.data.state_name = 'RECORDING';
        const {hook} = mount();
        const socket = enable(hook);
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        poll();
        setAxis(1, -1);
        for (let i = 0; i < 2500; i++) {
            topics.status.lastMessageAt = performance.now();
            poll();
        }
        expect(hook.result.current.enabled).toBe(true);
        expect(hook.result.current.state).toBe('driving');
        expect(socket.closeCount).toBe(0);
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(true);
        hook.unmount();
    });

    it('updates device availability with settings closed without streaming stick previews or reconnecting', () => {
        const onSample = vi.fn();
        const hook = renderHook(() => useBrowserGamepad({
            profile: structuredClone(DEFAULT_PROFILE), selected: {id: 'test pad', index: 0},
            runAction: async () => {}, onOwnershipChange: vi.fn(), onSample, preview: false,
        }));
        expect(hook.result.current.pads).toHaveLength(1);
        const preview = hook.result.current.pads;
        setAxis(1, -1);
        poll(150);
        expect(hook.result.current.pads).toBe(preview);
        expect(preview[0].axes).toEqual([0, 0]);
        pad = {...pad, connected: false};
        poll();
        expect(hook.result.current.pads).toEqual([]);
        pad = {...pad, connected: true};
        poll();
        expect(hook.result.current.pads).toHaveLength(1);
        expect(onSample).not.toHaveBeenCalled();
        expect(FakeWebSocket.instances).toHaveLength(0);
        expect(hook.result.current.enabled).toBe(false);
    });

    it.each([
        {change: {connected: false}, reason: 'disconnected'},
        {change: {id: 'replacement'}, reason: 'identityChanged'},
        {change: {index: 1}, reason: 'identityChanged'},
        {change: {axes: [0]}, reason: 'mappingChanged'},
    ])('revalidates changed native input before opening a session ($reason)', ({change, reason}) => {
        const {hook, onOwnershipChange} = mount();
        pad = {...pad, ...change}; // The configuration preview is no longer current.
        act(() => { hook.result.current.enable(); });
        expect(hook.result.current.fault).toBe('inputLost');
        expect(hook.result.current.inputLoss).toEqual({reason});
        expect(FakeWebSocket.instances).toHaveLength(0);
        expect(onOwnershipChange).not.toHaveBeenCalled();
    });

    it('keeps safety polling active while the configuration preview is hidden', () => {
        topics.status.data.state_name = 'RECORDING';
        const onSample = vi.fn();
        const hook = renderHook(({preview}) => useBrowserGamepad({
            profile: structuredClone(DEFAULT_PROFILE), selected: {id: 'test pad', index: 0},
            runAction: async () => {}, onOwnershipChange: vi.fn(), onSample, preview,
        }), {initialProps: {preview: true}});
        const socket = enable(hook);
        hook.rerender({preview: false});
        const samplesBefore = onSample.mock.calls.length;
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        poll();
        setAxis(1, -1);
        poll(600);
        expect(parseControl(socket.sent[socket.sent.length - 1]).twist.twist.linear.x).toBe(0.25);
        expect(onSample).toHaveBeenCalledTimes(samplesBefore);
        setButton(DEFAULT_PROFILE.buttons.deadman, false);
        poll();
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(false);
        act(() => { hook.result.current.disable(); });
        setAxis(1, 0);
        hook.rerender({preview: true});
        poll(100);
        expect(hook.result.current.pads[0].axes).toEqual([0, 0]);
        expect(onSample.mock.calls.length).toBeGreaterThan(samplesBefore);
    });

    it('keeps reading change-only Safari input across enable, cancellation and configuration', () => {
        topics.status.data.state_name = 'RECORDING';
        pad = {...pad, timestamp: 864000.001};
        const onSample = vi.fn();
        const onOwnershipChange = vi.fn<OwnershipChange>();
        const hook = renderHook(() => useBrowserGamepad({
            profile: structuredClone(DEFAULT_PROFILE), selected: {id: 'test pad', index: 0},
            runAction: async () => {}, onOwnershipChange, onSample,
        }));
        poll(1000); // A user pauses in configuration before enabling.
        expect(onSample).toHaveBeenCalled();
        expect(FakeWebSocket.instances).toHaveLength(0);
        const socket = enable(hook);
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        pad = {...pad, timestamp: 864000.002};
        poll();
        setAxis(1, -1);
        pad = {...pad, timestamp: 864000.003};
        poll(100);
        expect(parseControl(socket.sent[socket.sent.length - 1]).twist.twist.linear.x).toBe(0.25);
        poll(600); // Constant stick and deadman do not produce input changes.
        expect(hook.result.current.enabled).toBe(true);
        setButton(DEFAULT_PROFILE.buttons.deadman, false);
        pad = {...pad, timestamp: 864000.004};
        poll();
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(false);
        act(() => { hook.result.current.disable(); });
        setAxis(1, 0);
        pad = {...pad, timestamp: 864000.005};
        poll(100);
        expect(hook.result.current.pads[0].axes[1]).toBe(0);
        setAxis(0, 1);
        pad = {...pad, timestamp: 864000.006};
        poll(100);
        expect(hook.result.current.pads[0].axes).toEqual([1, 0]);
        setAxis(0, 0);
        pad = {...pad, timestamp: 864000.007};
        poll(100);
        expect(hook.result.current.pads[0].axes).toEqual([0, 0]);
        expect(FakeWebSocket.instances).toHaveLength(1);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false]);
    });

    it('allows HTTP when the browser exposes Gamepad input and still stops on release or API loss', () => {
        Object.defineProperty(window, 'isSecureContext', {configurable: true, value: false});
        topics.status.data.state_name = 'RECORDING';
        const {hook, onOwnershipChange} = mount();
        act(() => { hook.result.current.enable(); });
        expect(hook.result.current.enabled).toBe(true);
        const socket = FakeWebSocket.instances[0];
        socket.open();
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        poll();
        setAxis(1, -1);
        poll();
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: true, twist: {twist: {linear: {x: 0.25}}}});

        setButton(DEFAULT_PROFILE.buttons.deadman, false);
        poll();
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: false, twist: {twist: {linear: {x: 0}, angular: {z: 0}}}});

        Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: () => {
            throw new DOMException('Browser blocked input', 'SecurityError');
        }});
        poll();
        expect(hook.result.current.enabled).toBe(false);
        expect(hook.result.current.fault).toBe('unsupported');
        expect(socket.closeCount).toBe(1);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false]);
        poll(500);
        expect(FakeWebSocket.instances).toHaveLength(1);
    });

    it.each(['missing', 'blocked'])('fails closed on HTTP when the Gamepad API is %s', availability => {
        Object.defineProperty(window, 'isSecureContext', {configurable: true, value: false});
        Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: availability === 'missing' ? undefined : () => {
            throw new DOMException('Browser blocked input', 'SecurityError');
        }});
        const {hook, onOwnershipChange} = mount();
        act(() => { hook.result.current.enable(); });
        expect(hook.result.current.enabled).toBe(false);
        expect(hook.result.current.fault).toBe('unsupported');
        expect(FakeWebSocket.instances).toHaveLength(0);
        expect(onOwnershipChange).not.toHaveBeenCalled();
    });

    it('opens only an explicit session, sends neutral on disable, and releases ownership once', () => {
        const {hook, onOwnershipChange} = mount();
        const socket = enable(hook);
        expect(hook.result.current.enabled).toBe(true);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true]);

        poll();
        const control = parseControl(socket.sent[socket.sent.length - 1]);
        expect(Object.keys(control).sort()).toEqual(['deadman', 'epoch', 'neutral', 'physicalDeadman', 'twist']);
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

    it.each([-1, 1])('expires status delivery on monotonic time after a wall-clock jump (%s)', direction => {
        topics.status.data.state_name = 'RECORDING';
        const {hook} = mount();
        const socket = enable(hook);
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        poll();
        setAxis(1, -1);
        poll();
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(true);

        vi.setSystemTime(Date.now() + direction * 60 * 60 * 1000);
        poll();
        // Changing the wall clock must neither expire a live status early nor
        // preserve a stale status until the wall clock catches up again.
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(true);
        poll(2300);
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(false);
        expect(parseControl(socket.sent[socket.sent.length - 1]).twist.twist.linear.x).toBe(0);
        expect(hook.result.current.state).toBe('statusLost');
        topics.status.lastMessageAt = performance.now();
        poll();
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(false);
        setButton(DEFAULT_PROFILE.buttons.deadman, false);
        setAxis(1, 0);
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        poll();
        setAxis(1, -1);
        poll();
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(false);
        expect(hook.result.current.enabled).toBe(false);
        expect(socket.closeCount).toBe(1);
    });

    it('reports a browser permission-policy failure before enabling', () => {
        Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: () => {
            throw new DOMException('Blocked by permissions policy', 'SecurityError');
        }});
        const {hook, onOwnershipChange} = mount();
        expect(hook.result.current.fault).toBe('unsupported');
        expect(FakeWebSocket.instances).toHaveLength(0);
        expect(onOwnershipChange).not.toHaveBeenCalled();
    });

    it.each(['blur', 'pagehide', 'visibilitychange'])('revokes immediately on %s and requires explicit rearming', event => {
        const {hook, onOwnershipChange} = mount();
        const socket = enable(hook);
        poll();
        act(() => { (event === 'visibilitychange' ? document : window).dispatchEvent(new Event(event)); });
        expect(socket.closeCount).toBe(1);
        expect(hook.result.current.enabled).toBe(false);
        const reason = event === 'blur' ? 'focusLost' : event === 'pagehide' ? 'pageExit' : 'pageHidden';
        expect(hook.result.current.inputLoss).toEqual(event === 'blur' ?
            {reason, source: 'windowBlur', documentFocused: true, visibility: 'visible', activeElement: 'BODY'} : {reason});
        expect(socket.closeReason).toBe(event === 'blur' ? 'input:focusLost:windowBlur:BODY:visible:true' : `input:${reason}`);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false]);
        poll(500);
        expect(FakeWebSocket.instances).toHaveLength(1);
    });

    it('ends a connection that never opens without sending or reconnecting', () => {
        const {hook} = mount();
        act(() => { hook.result.current.enable(); });
        const socket = FakeWebSocket.instances[0];
        poll(2100);
        expect(hook.result.current.fault).toBe('connectionLost');
        expect(socket.sent).toHaveLength(0);
        expect(socket.closeCount).toBe(1);
        poll(1000);
        expect(FakeWebSocket.instances).toHaveLength(1);
    });

    it('revokes ownership when a live WebSocket send throws', () => {
        const {hook, onOwnershipChange} = mount();
        const socket = enable(hook);
        poll();
        vi.spyOn(socket, 'send').mockImplementation(() => { throw new Error('Transport failed'); });
        poll();
        expect(hook.result.current.fault).toBe('connectionLost');
        expect(socket.closeCount).toBe(1);
        expect(onOwnershipChange.mock.calls.map(([owns]) => owns)).toEqual([true, false]);
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
        expect(hook.result.current.inputLoss).toEqual({reason: 'pollGap', pollGapMs: 201});
        expect(socket.closeReason).toBe('input:pollGap:201ms');
        expect(socket.closeCount).toBe(1);
        expect(FakeWebSocket.instances).toHaveLength(1);
        act(() => { hook.result.current.disable(); }); // Opening configuration must retain the first cause.
        poll();
        expect(hook.result.current.inputLoss).toEqual({reason: 'pollGap', pollGapMs: 201});
        enable(hook);
        expect(hook.result.current.inputLoss).toBeNull();
    });

    it('identifies focus loss on a poll even when no blur event was dispatched', () => {
        const {hook} = mount();
        const socket = enable(hook);
        poll();
        vi.spyOn(document, 'hasFocus').mockReturnValue(false);
        poll();
        expect(hook.result.current.inputLoss).toEqual({reason: 'focusLost', source: 'poll', documentFocused: false, visibility: 'visible', activeElement: 'BODY'});
        expect(socket.closeReason).toBe('input:focusLost:poll:BODY:visible:false');
        vi.spyOn(document, 'hasFocus').mockReturnValue(true);
        poll(500);
        expect(hook.result.current.enabled).toBe(false);
        expect(FakeWebSocket.instances).toHaveLength(1);
    });

    it('does not open a session if explicit activation still cannot obtain page focus', () => {
        const {hook, onOwnershipChange} = mount();
        vi.spyOn(document, 'hasFocus').mockReturnValue(false);
        act(() => { hook.result.current.enable(); });
        expect(FakeWebSocket.instances).toHaveLength(0);
        expect(onOwnershipChange).not.toHaveBeenCalled();
        expect(hook.result.current.inputLoss).toEqual({reason: 'focusLost', source: 'enable', documentFocused: false, visibility: 'visible', activeElement: 'BODY'});
    });

    it('bounds focus metadata so reporting it cannot break the WebSocket close handshake', () => {
        const {hook} = mount();
        const socket = enable(hook);
        const element = document.createElement('x-' + 'a'.repeat(180));
        element.tabIndex = -1;
        document.body.appendChild(element);
        try {
            element.focus();
            vi.spyOn(document, 'hasFocus').mockReturnValue(false);
            poll();
            expect(socket.closeCount).toBe(1);
            expect(new TextEncoder().encode(socket.closeReason).length).toBeLessThanOrEqual(123);
            expect(hook.result.current.inputLoss?.activeElement).toHaveLength(32);
        } finally { element.remove(); }
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
        expect(runAction).toHaveBeenCalledWith('recordStart', SESSION_TOKEN);

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

    it('sends one session-tagged STOP and never compensates from a retired hook after remount', async () => {
        let resolveStart!: () => void;
        let resolveStop!: () => void;
        const runAction = vi.fn<RunAction>(action => action === 'recordStart' ?
            new Promise<void>(resolve => { resolveStart = resolve; }) :
            new Promise<void>(resolve => { resolveStop = resolve; }));
        const {hook} = mount(runAction);
        const socket = enable(hook);
        poll();
        poll();
        setButton(DEFAULT_PROFILE.buttons.recordStart, true);
        poll();
        poll(800);
        expect(runAction).toHaveBeenCalledExactlyOnceWith('recordStart', SESSION_TOKEN);
        const actionStartFrame = socket.sent.length;
        setButton(DEFAULT_PROFILE.buttons.recordStart, false);
        setButton(DEFAULT_PROFILE.buttons.deadman, true);
        setAxis(1, -1);
        poll();
        setButton(DEFAULT_PROFILE.buttons.stop, true);
        poll();
        expect(runAction.mock.calls).toEqual([['recordStart', SESSION_TOKEN], ['stop', SESSION_TOKEN]]);
        expect(socket.sent.slice(actionStartFrame).map(parseControl).every(({deadman, twist}) =>
            !deadman && twist.twist.linear.x === 0 && twist.twist.angular.z === 0)).toBe(true);
        hook.unmount();
        setButton(DEFAULT_PROFILE.buttons.stop, false);
        setButton(DEFAULT_PROFILE.buttons.deadman, false);
        setAxis(1, 0);
        const replacement = mount();
        enable(replacement.hook);
        poll(100);
        await act(async () => {
            resolveStart();
            resolveStop();
            await Promise.resolve();
            await Promise.resolve();
        });
        // The server retains the action fence until compensation; the retired
        // component must issue no later STOP against the replacement's mode.
        expect(runAction.mock.calls).toEqual([['recordStart', SESSION_TOKEN], ['stop', SESSION_TOKEN]]);
        expect(replacement.hook.result.current.enabled).toBe(true);
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

describe('manual control notices', () => {
    it('keeps one session through pause/resume, echoes epochs, and confirms centered held input', () => {
        topics.status.data.state_name = 'MANUAL_MOWING';
        const {hook, runAction, onOwnershipChange} = mount();
        const socket = enable(hook);
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true); poll();
        act(() => socket.message({type: 'gamepadControl', epoch: 1, phase: 'active'}));
        setAxis(1, -1); poll();
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: true, physicalDeadman: true, neutral: false, epoch: 1});
        setButton(DEFAULT_PROFILE.buttons.deadman, false); poll();
        act(() => socket.message({type: 'gamepadControl', epoch: 2, phase: 'pausing'}));
        poll();
        expect(hook.result.current.state).toBe('manualPausing');
        expect(hook.result.current.enabled).toBe(true);
        topics.status.data.state_name = 'IDLE';
        topics.status.lastMessageAt = performance.now();
        act(() => socket.message({type: 'gamepadControl', epoch: 3, phase: 'paused'}));
        setAxis(1, 0); poll(100);
        expect(hook.result.current.state).toBe('manualPaused');
        setButton(DEFAULT_PROFILE.buttons.deadman, true); poll();
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: false, physicalDeadman: true, neutral: true, epoch: 3});
        act(() => socket.message({type: 'gamepadControl', epoch: 4, phase: 'resuming'}));
        poll();
        expect(hook.result.current.state).toBe('manualResuming');
        act(() => socket.message({type: 'gamepadControl', epoch: 5, phase: 'active', resumed: true}));
        poll(250); // server confirmation precedes the browser multiplex status
        expect(hook.result.current.state).toBe('manualResuming');
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: false, physicalDeadman: true, neutral: true, epoch: 5});
        topics.status.data.state_name = 'MANUAL_MOWING';
        topics.status.lastMessageAt = performance.now();
        poll(); // new centered native sample consumes confirmation, still zero
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: false, physicalDeadman: true, neutral: true, epoch: 5});
        setAxis(1, -1); poll();
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: true, physicalDeadman: true, neutral: false, epoch: 5});
        expect(runAction).not.toHaveBeenCalled(); // resume7 belongs to server
        expect(onOwnershipChange.mock.calls).toEqual([[true]]);
        expect(socket.closeCount).toBe(0);
    });

    it('handles notices after the two-second startup window and ignores stale snapshots', () => {
        const {hook} = mount();
        const socket = enable(hook);
        for (let i = 0; i < 60; i++) { topics.status.lastMessageAt = performance.now(); poll(); }
        act(() => socket.message({type: 'gamepadControl', epoch: 2, phase: 'inactive'}));
        act(() => socket.message({type: 'gamepadControl', epoch: 1, phase: 'paused'}));
        poll();
        expect(hook.result.current.enabled).toBe(true);
        expect(hook.result.current.manualPhase).toBe('inactive');
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({epoch: 2});
        expect(socket.closeCount).toBe(0);
    });

    it('keeps raw held intent separate while a mapped command is pending', () => {
        const runAction = vi.fn<RunAction>(() => new Promise(() => {}));
        const {hook} = mount(runAction);
        const socket = enable(hook); poll(100);
        setButton(DEFAULT_PROFILE.buttons.recordStart, true); poll(850);
        expect(runAction).toHaveBeenCalledTimes(1);
        setButton(DEFAULT_PROFILE.buttons.recordStart, false);
        setButton(DEFAULT_PROFILE.buttons.deadman, true); poll();
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: false, physicalDeadman: true, neutral: true});
    });

    it('does not let a retired socket or status recovery reinstall a paused capability', () => {
        const {hook} = mount();
        const old = enable(hook); poll(100);
        act(() => old.message({type: 'gamepadControl', epoch: 3, phase: 'paused'}));
        act(() => hook.result.current.disable());
        const current = enable(hook); poll(100);
        act(() => old.message({type: 'gamepadControl', epoch: 20, phase: 'paused'}));
        expect(hook.result.current.manualPhase).toBe('inactive');
        expect(parseControl(current.sent[current.sent.length - 1])).toMatchObject({epoch: 0});
        topics.status.data.emergency = true;
        topics.status.lastMessageAt = performance.now(); poll();
        expect(hook.result.current.enabled).toBe(false);
        topics.status.data.emergency = false;
        topics.status.lastMessageAt = performance.now(); poll();
        act(() => current.message({type: 'gamepadControl', epoch: 30, phase: 'active'}));
        expect(hook.result.current.enabled).toBe(false);
        expect(hook.result.current.manualPhase).toBe('inactive');
    });

    it('rejects a protocol v2 server before acquiring a claim', () => {
        const {hook} = mount();
        act(() => hook.result.current.enable());
        const socket = FakeWebSocket.instances[0]; socket.open(false);
        act(() => socket.message({type: 'gamepadHello', version: 2, challenge: SESSION_TOKEN}));
        expect(hook.result.current.enabled).toBe(false);
        expect(socket.claims).toHaveLength(0);
    });
});

describe('coalesced manual phase provenance', () => {
    it.each([true, false])('distinguishes owned resume from ordinary GUI7 when intermediate phases coalesce (resumed=%s)', resumed => {
        const {hook} = mount();
        const socket = enable(hook); poll(100);
        act(() => socket.message({type: 'gamepadControl', epoch: 3, phase: 'paused'}));
        poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true); poll();
        // The server may omit resuming, or omit inactive for an ordinary GUI7.
        act(() => socket.message({type: 'gamepadControl', epoch: 5, phase: 'active', ...(resumed ? {resumed: true} : {})}));
        poll(200); // browser ROS status is still IDLE
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(false);
        topics.status.data.state_name = 'MANUAL_MOWING';
        topics.status.lastMessageAt = performance.now(); poll();
        setAxis(1, -1); poll();
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(resumed);
        expect(hook.result.current.enabled).toBe(true);
    });

    it.each([
        {type: 'gamepadControl', epoch: 5, phase: 'active', resumed: false},
        {type: 'gamepadControl', epoch: 5, phase: 'paused', resumed: true},
        {type: 'gamepadControl', epoch: 5, phase: 'active', resumed: 'true'},
        {type: 'gamepadControl', epoch: 5, phase: 'active', resumed: true, extra: true},
    ])('rejects malformed resume provenance %o', notice => {
        const {hook} = mount(); const socket = enable(hook); poll(100);
        act(() => socket.message(notice));
        expect(hook.result.current.enabled).toBe(false);
        expect(hook.result.current.fault).toBe('connectionLost');
    });
});


describe('new active epochs require new operator input', () => {
    it.each([3, 4])('rearms after coalesced ordinary GUI7 at epoch %s while the old stick remains held', epoch => {
        topics.status.data.state_name = 'MANUAL_MOWING';
        const {hook} = mount();
        const socket = enable(hook); poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true); poll();
        act(() => socket.message({type: 'gamepadControl', epoch: 1, phase: 'active'}));
        setAxis(1, -1); poll();
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(true);
        // Generic GUI7, with or without a preceding STOP, revokes intermediate
        // epochs. Those snapshots may be coalesced before the next active notice.
        act(() => socket.message({type: 'gamepadControl', epoch, phase: 'active'}));
        poll();
        expect(parseControl(socket.sent[socket.sent.length - 1])).toMatchObject({deadman: false, twist: {twist: {linear: {x: 0}}}});
        expect(hook.result.current.state).toBe('releaseControls');
        setAxis(1, 0); setButton(DEFAULT_PROFILE.buttons.deadman, false); poll(100);
        setButton(DEFAULT_PROFILE.buttons.deadman, true); poll();
        setAxis(1, -1); poll();
        expect(parseControl(socket.sent[socket.sent.length - 1]).deadman).toBe(true);
    });
});
