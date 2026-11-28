import {useCallback, useEffect, useLayoutEffect, useRef, useState} from 'react';
import {wsBase} from '../utils/apiHost';
import {useTopic} from '../hooks/useTopic';
import {GamepadDiagnostics} from './diagnostics';
import type {Emergency, HighLevelStatus} from '../types/ros';
import {
    ControllerDriver, profileFits, teleopTwist,
    type ManualPhase, type ControllerAction, type ControllerIdentity, type ControllerProfile, type ControllerSample, type ControllerState, type InputLoss,
} from './controller';

interface Options {
    profile: ControllerProfile;
    selected: ControllerIdentity | null;
    runAction: (action: ControllerAction, sessionToken: string) => Promise<void>;
    onOwnershipChange: (owns: boolean) => void;
    onSample?: (devices: ControllerSample[]) => void;
    onInventoryChange?: (devices: ControllerSample[]) => void;
    /** Configuration preview only; input reads and safety polling never depend on it. */
    preview?: boolean;
}
export type ControllerFault = 'unsupported' | 'inputLost' | 'connectionLost' | 'actionFailed' | null;
export const focusInputLoss = (source: NonNullable<InputLoss['source']>): InputLoss => ({
    reason: 'focusLost', source, documentFocused: document.hasFocus(), visibility: document.visibilityState,
    activeElement: document.activeElement?.tagName.replace(/[^a-z0-9:-]/gi, '?').slice(0, 32) ?? 'none',
});
const copyPad = (pad: Gamepad): ControllerSample => ({
    id: pad.id, index: pad.index, mapping: pad.mapping, connected: pad.connected,
    timestamp: pad.timestamp, axes: [...pad.axes], buttons: pad.buttons.map(b => ({pressed: b.pressed, touched: b.touched, value: b.value})),
});
interface ControllerSession {
    socket: WebSocket;
    driver: ControllerDriver;
    identity: ControllerIdentity;
    opened: number;
    attempt: string;
    phase: 'hello' | 'claim' | 'ready';
    challenge: string | null;
    epoch: number;
    manualPhase: ManualPhase;
}

/** Browser-only input adapter. No queued sends, replay interval, or automatic reconnect.
 * The server owns the 300 ms session lease; the existing relay/firmware own robot safety.
 */
export function useBrowserGamepad(options: Options) {
    const opts = useRef(options);
    useLayoutEffect(() => { opts.current = options; });
    const status = useTopic<HighLevelStatus>('highLevelStatus', {}, {withTimestamp: true, timestampClock: 'monotonic'});
    const emergency = useTopic<Emergency>('emergency', {}, {withTimestamp: true});
    const context = useRef({status, emergency});
    useLayoutEffect(() => { context.current = {status, emergency}; });
    const [pads, setPads] = useState<ControllerSample[]>([]);
    const [enabled, setEnabled] = useState(false);
    const [state, setState] = useState<ControllerState>('releaseControls');
    const [manualPhase, setManualPhase] = useState<ManualPhase>('inactive');
    const [fault, setFault] = useState<ControllerFault>(null);
    // Retain the first cause when configuration is reopened, until Enable is retried.
    const [inputLoss, setInputLoss] = useState<InputLoss | null>(null);
    const [diagnostics] = useState(() => new GamepadDiagnostics());
    const attemptNumber = useRef(0);
    const session = useRef<ControllerSession | null>(null);
    const pending = useRef(false);
    const actionTask = useRef<Promise<void> | null>(null);
    const pendingSince = useRef<number | null>(null);
    const reportedFault = useRef<ControllerFault>(null);
    const mounted = useRef(true);
    const cancel = useCallback((reason: ControllerFault = null, detail: InputLoss | null = null) => {
        const active = session.current;
        session.current = null; // Revoke before onclose/late async callbacks.
        if (active) {
            try {
                if (active.phase === 'ready' && active.socket.readyState === WebSocket.OPEN && active.socket.bufferedAmount === 0) {
                    active.socket.send(JSON.stringify({twist: teleopTwist(), deadman: false, physicalDeadman: false, neutral: false, epoch: active.epoch}));
                }
            } catch { /* The server lease also handles an unusable socket. */ }
            const report = detail ? `input:${detail.reason}${detail.pollGapMs === undefined ? '' : `:${Math.round(detail.pollGapMs)}ms`}` +
                (detail.source ? `:${detail.source}:${detail.activeElement}:${detail.visibility}:${detail.documentFocused}` : '') : '';
            active.socket.close(1000, report);
            opts.current.onOwnershipChange(false);
        }
        if (active || (reason && reason !== reportedFault.current)) {
            diagnostics.update({enabled: false, fault: reason, documentFocused: document.hasFocus(), visibility: document.visibilityState});
            diagnostics.record('sessionEnded', {attempt: active?.attempt, fault: reason ?? 'disabled', ...detail});
            diagnostics.flushSoon();
        }
        reportedFault.current = reason;
        if (mounted.current) {
            setEnabled(false);
            setState('releaseControls');
            setManualPhase('inactive');
            setFault(reason);
            if (detail) setInputLoss(detail);
        }
    }, [diagnostics]);
    const disable = () => cancel();
    const enable = () => {
        cancel();
        setInputLoss(null);
        const attempt = `${diagnostics.clientId}-${++attemptNumber.current}`;
        diagnostics.update({attempt, enabled: false, fault: null, userAgent: navigator.userAgent.slice(0, 256),
            origin: window.location.origin,
            standalone: !!(navigator as Navigator & {standalone?: boolean}).standalone || window.matchMedia('(display-mode: standalone)').matches,
            webBuild: import.meta.env.VITE_WEB_BUILD_ID as string | undefined,
            revision: import.meta.env.VITE_BUILD_REVISION as string | undefined});
        diagnostics.record('enableRequested', {attempt, pending: pending.current});
        const reject = (reason: ControllerFault, detail: InputLoss | null = null) => {
            cancel(reason, detail);
        };
        if (typeof navigator.getGamepads !== 'function') { reject('unsupported'); return; }
        const {selected, profile} = opts.current;
        let pad: Gamepad | null | undefined;
        try { pad = selected ? navigator.getGamepads()[selected.index] : null; }
        catch { reject('unsupported'); return; }
        if (!pad || !pad.connected || !selected || pad.id !== selected.id || pad.index !== selected.index || !profileFits(profile, pad)) {
            reject('inputLost', {reason: !pad || !pad.connected ? 'disconnected' :
                !selected || pad.id !== selected.id || pad.index !== selected.index ? 'identityChanged' : 'mappingChanged'}); return;
        }
        if (document.visibilityState !== 'visible' || !document.hasFocus()) {
            reject('inputLost', document.visibilityState !== 'visible' ? {reason: 'pageHidden'} : focusInputLoss('enable')); return;
        }
        let socket: WebSocket;
        try { socket = new WebSocket(`${wsBase()}/api/mowglinext/publish/gamepad?client=${encodeURIComponent(attempt)}`); }
        catch { reject('connectionLost'); return; }
        const active: ControllerSession = {socket, driver: new ControllerDriver(structuredClone(profile), selected),
            identity: selected, opened: performance.now(), attempt, phase: 'hello', challenge: null, epoch: 0, manualPhase: 'inactive'};
        session.current = active;
        diagnostics.update({enabled: true, controller: pad.id.slice(0, 128), index: pad.index, mapping: pad.mapping,
            deadmanButton: profile.buttons.deadman, linearAxis: profile.linearAxis, angularAxis: profile.angularAxis});
        socket.onclose = event => {
            diagnostics.record('socketClosed', {attempt, code: event.code, reason: event.reason?.slice(0, 123), clean: event.wasClean});
            if (session.current === active) cancel('connectionLost');
            diagnostics.flushSoon();
        };
        socket.onerror = () => {
            diagnostics.record('socketError', {attempt, socketState: socket.readyState});
            if (session.current === active) cancel('connectionLost');
        };
        socket.onmessage = event => {
            // A timed-out/disabled attempt must never claim a late upgrade or
            // gain the backend's zero/STOP responsibility for a later mode.
            if (session.current !== active) return;
            if (active.phase !== 'ready' && performance.now() - active.opened > 2000) {
                diagnostics.record('connectTimeout', {phase: active.phase}); cancel('connectionLost'); return;
            }
            try {
                const data: unknown = event.data;
                if (typeof data !== 'string' || data.length > 256) throw new Error('invalid server handshake');
                const parsed: unknown = JSON.parse(data);
                if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) throw new Error('invalid server handshake');
                const message = parsed as {type?: unknown; challenge?: unknown; version?: unknown; epoch?: unknown; phase?: unknown; resumed?: unknown};
                if (message.type === 'gamepadHello' && active.phase === 'hello' && active.challenge === null &&
                    Object.keys(message).length === 3 && message.version === 3 && typeof message.challenge === 'string' && /^[0-9a-f]{32}$/.test(message.challenge)) {
                    active.challenge = message.challenge;
                    diagnostics.record('helloReceived', {attempt, handshakeMs: Math.round(performance.now() - active.opened)});
                } else if (message.type === 'gamepadReady' && active.phase === 'claim' && Object.keys(message).length === 1) {
                    active.driver.requireNeutral();
                    active.phase = 'ready';
                    diagnostics.record('serverReady', {attempt, handshakeMs: Math.round(performance.now() - active.opened)});
                } else if (message.type === 'gamepadControl' && active.phase === 'ready' && (Object.keys(message).length === 3 ||
                    (Object.keys(message).length === 4 && message.phase === 'active' && message.resumed === true)) &&
                    Number.isSafeInteger(message.epoch) && Number(message.epoch) > 0 &&
                    ['inactive', 'active', 'pausing', 'paused', 'resuming'].includes(String(message.phase))) {
                    // Ignore an older snapshot: it can never grant a new capability.
                    if (Number(message.epoch) <= active.epoch) return;
                    const phase = message.phase as ManualPhase;
                    // The very first valid manual deadman poll establishes
                    // active epoch1; this notice acknowledges that same input.
                    // Any explicit mode intent first revokes/increments epoch.
                    const bootstrap = active.epoch === 0 && message.epoch === 1 && active.manualPhase === 'inactive' && phase === 'active' && message.resumed !== true;
                    if (phase === 'active' && message.resumed === true && ['paused', 'resuming'].includes(active.manualPhase)) active.driver.confirmManualResume();
                    else if (!bootstrap) active.driver.requireNeutral();
                    active.epoch = Number(message.epoch);
                    active.manualPhase = phase;
                    setManualPhase(phase);
                } else throw new Error('unexpected server handshake');
            } catch {
                diagnostics.record('handshakeInvalid', {attempt, phase: active.phase});
                cancel('connectionLost');
            }
        };
        opts.current.onOwnershipChange(true);
        setEnabled(true);
        setFault(null);
        setState('connecting');
    };

    useEffect(() => {
        mounted.current = true;
        let lastDisplay = -Infinity;
        let lastInventory = '';
        let lastPoll: number | null = null;
        let maxPollGap = 0;
        let lastControlState = '';
        const poll = () => {
            let devices: Gamepad[] = [];
            try {
                if (typeof navigator.getGamepads === 'function') devices = [...navigator.getGamepads()].filter((p): p is Gamepad => !!p && p.connected);
            } catch {
                cancel('unsupported');
            }
            const now = performance.now();
            // Keep high-rate control out of React's render loop (including MapPage).
            // Inventory changes still reach the map so reconnect can offer an
            // explicit Enable without reopening settings. Never render each stick
            // sample or use this preview to validate an actual control session.
            const inventory = JSON.stringify(devices.map(p => [p.index, p.id, p.mapping, p.axes.length, p.buttons.length]));
            const inventoryChanged = inventory !== lastInventory;
            lastInventory = inventory;
            if (inventoryChanged || (opts.current.preview !== false && now - lastDisplay >= 100)) {
                const samples = devices.map(copyPad);
                setPads(samples);
                if (inventoryChanged) opts.current.onInventoryChange?.(samples);
                if (opts.current.preview !== false) {
                    opts.current.onSample?.(samples);
                    lastDisplay = now;
                }
            }
            const active = session.current;
            if (!active) { lastPoll = null; return; }
            const pollGap = lastPoll === null ? 0 : now - lastPoll;
            lastPoll = now;
            maxPollGap = Math.max(maxPollGap, pollGap);
            const pad = devices.find(p => p.index === active.identity.index && p.id === active.identity.id) ?? null;
            const current = context.current;
            const statusAge = current.status.lastMessageAt === null ? null : Math.round(now - current.status.lastMessageAt);
            const profile = opts.current.profile;
            diagnostics.update({at: new Date().toISOString(), attempt: active.attempt, sessionMs: Math.round(now - active.opened),
                socketState: active.socket.readyState, bufferedBytes: active.socket.bufferedAmount, handshake: active.phase,
                pollGapMs: Math.round(pollGap), maxPollGapMs: Math.round(maxPollGap),
                statusAgeMs: statusAge, mode: current.status.data.state_name ?? '',
                emergency: !!(current.status.data.emergency || current.emergency.data.active_emergency || current.emergency.data.latched_emergency),
                pending: pending.current, pendingMs: pendingSince.current === null ? null : Math.round(now - pendingSince.current),
                documentFocused: document.hasFocus(), visibility: document.visibilityState, online: navigator.onLine,
                nativeConnected: !!pad?.connected, nativeTimestamp: pad?.timestamp,
                linearInput: pad?.axes[profile.linearAxis], angularInput: pad?.axes[profile.angularAxis],
                deadmanInput: pad?.buttons[profile.buttons.deadman]?.value});
            if (document.visibilityState !== 'visible' || !document.hasFocus()) {
                cancel('inputLost', document.visibilityState !== 'visible' ? {reason: 'pageHidden'} : focusInputLoss('poll')); return;
            }
            if (active.phase !== 'ready' && now - active.opened > 2000) {
                diagnostics.record('connectTimeout', {phase: active.phase}); cancel('connectionLost'); return;
            }
            if (active.socket.readyState === WebSocket.CONNECTING) return;
            if (active.socket.readyState !== WebSocket.OPEN || active.socket.bufferedAmount !== 0) {
                diagnostics.record('socketUnavailable', {socketState: active.socket.readyState, bufferedBytes: active.socket.bufferedAmount});
                cancel('connectionLost'); return;
            }
            const frame = active.driver.step(pad, now, {
                stateName: current.status.data.state_name ?? '',
                // Delivery liveness only; robot-side interlocks remain authoritative.
                statusFresh: current.status.lastMessageAt !== null && now >= current.status.lastMessageAt && now - current.status.lastMessageAt < 2500,
                emergency: !!(current.status.data.emergency || current.emergency.data.active_emergency || current.emergency.data.latched_emergency),
                pending: pending.current,
                manualPhase: active.manualPhase,
            });
            if (active.phase === 'ready' && (frame.state === 'statusLost' || frame.state === 'emergency')) {
                cancel('inputLost'); setState(frame.state); return;
            }
            if (frame.state === 'inputLost') { cancel('inputLost', frame.inputLoss); return; }
            if (active.phase !== 'ready') {
                // Claim only from this poll's valid native input. TCP opening
                // alone is not proof that this browser attempt is still live.
                // No velocity or robot action is sent until the server is ready.
                if (active.phase === 'hello' && active.challenge !== null) {
                    const challenge = active.challenge;
                    active.phase = 'claim';
                    try { active.socket.send(JSON.stringify({type: 'gamepadClaim', challenge, version: 3})); }
                    catch { diagnostics.record('sendFailed'); cancel('connectionLost'); }
                }
                return;
            }
            diagnostics.update({state: frame.state});
            if (`${active.attempt}:${frame.state}` !== lastControlState) {
                lastControlState = `${active.attempt}:${frame.state}`;
                diagnostics.record('controlState', {attempt: active.attempt, state: frame.state, mode: current.status.data.state_name,
                    statusAgeMs: statusAge, pending: pending.current});
                if (['statusLost', 'emergency', 'actionPending'].includes(frame.state)) diagnostics.flushSoon();
            }
            setState(frame.state);
            try { active.socket.send(JSON.stringify({twist: teleopTwist(frame.linear, frame.angular), deadman: frame.deadman,
                physicalDeadman: frame.physicalDeadman, neutral: frame.neutral, epoch: active.epoch})); }
            catch { diagnostics.record('sendFailed'); cancel('connectionLost'); return; }
            if (frame.action && (frame.action === 'stop' || !pending.current)) {
                const action = frame.action;
                const runAction = opts.current.runAction;
                pending.current = true;
                pendingSince.current = now;
                diagnostics.record('actionStarted', {attempt: active.attempt, action});
                // The coordinator owns settlement/STOP compensation across
                // unmount, navigation and HTTP cancellation. Never send a late
                // browser compensation that could stop a replacement mode.
                const task = runAction(action, active.challenge!);
                actionTask.current = task;
                void task.then(() => { diagnostics.record('actionFinished', {attempt: active.attempt, action}); }).catch(() => {
                    diagnostics.record('actionFailed', {attempt: active.attempt, action});
                    if (session.current === active) cancel('actionFailed');
                }).finally(() => {
                    if (actionTask.current === task) {
                        actionTask.current = null;
                        pending.current = false;
                        pendingSince.current = null;
                    }
                    diagnostics.flushSoon();
                });
            }
        };
        const interval = window.setInterval(poll, 50);
        const diagnosticInterval = window.setInterval(() => { void diagnostics.flush(); }, 5000);
        poll();
        const suspend = (event: Event) => {
            if (session.current) cancel('inputLost', event.type === 'blur' ? focusInputLoss('windowBlur') :
                {reason: event.type === 'pagehide' ? 'pageExit' : 'pageHidden'});
        };
        const disconnect = (event: GamepadEvent) => {
            if (event.gamepad.index === session.current?.identity.index) {
                cancel('inputLost', {reason: event.type === 'gamepaddisconnected' ? 'disconnected' : 'controllerChanged'});
            }
        };
        window.addEventListener('blur', suspend);
        window.addEventListener('pagehide', suspend);
        document.addEventListener('visibilitychange', suspend);
        window.addEventListener('gamepaddisconnected', disconnect);
        window.addEventListener('gamepadconnected', disconnect); // Never inherit an index after hotplug.
        return () => {
            mounted.current = false;
            window.clearInterval(interval);
            window.clearInterval(diagnosticInterval);
            window.removeEventListener('blur', suspend);
            window.removeEventListener('pagehide', suspend);
            document.removeEventListener('visibilitychange', suspend);
            window.removeEventListener('gamepaddisconnected', disconnect);
            window.removeEventListener('gamepadconnected', disconnect);
            cancel();
        };
    }, [cancel, diagnostics]);
    return {pads, enabled, state, manualPhase, fault, inputLoss, enable, disable};
}
