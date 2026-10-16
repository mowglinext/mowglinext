import {useCallback, useEffect, useLayoutEffect, useRef, useState} from 'react';
import {wsBase} from '../utils/apiHost';
import {useTopic} from '../hooks/useTopic';
import {GamepadDiagnostics} from './diagnostics';
import type {Emergency, HighLevelStatus} from '../types/ros';
import {
    ControllerDriver, profileFits, teleopTwist,
    type ControllerAction, type ControllerIdentity, type ControllerProfile, type ControllerSample, type ControllerState, type InputLoss,
} from './controller';

interface Options {
    profile: ControllerProfile;
    selected: ControllerIdentity | null;
    runAction: (action: ControllerAction) => Promise<void>;
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
    const [fault, setFault] = useState<ControllerFault>(null);
    // Retain the first cause when configuration is reopened, until Enable is retried.
    const [inputLoss, setInputLoss] = useState<InputLoss | null>(null);
    const [diagnostics] = useState(() => new GamepadDiagnostics());
    const attemptNumber = useRef(0);
    const session = useRef<{socket: WebSocket; driver: ControllerDriver; identity: ControllerIdentity; opened: number; attempt: string} | null>(null);
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
                if (active.socket.readyState === WebSocket.OPEN && active.socket.bufferedAmount === 0) {
                    active.socket.send(JSON.stringify({twist: teleopTwist(), deadman: false}));
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
        const active = {socket, driver: new ControllerDriver(structuredClone(profile), selected), identity: selected, opened: performance.now(), attempt};
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
                socketState: active.socket.readyState, bufferedBytes: active.socket.bufferedAmount,
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
            if (active.socket.readyState === WebSocket.CONNECTING) {
                if (now - active.opened > 2000) {
                    diagnostics.record('connectTimeout'); cancel('connectionLost');
                }
                return;
            }
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
            });
            diagnostics.update({state: frame.state});
            if (`${active.attempt}:${frame.state}` !== lastControlState) {
                lastControlState = `${active.attempt}:${frame.state}`;
                diagnostics.record('controlState', {attempt: active.attempt, state: frame.state, mode: current.status.data.state_name,
                    statusAgeMs: statusAge, pending: pending.current});
                if (['statusLost', 'emergency', 'actionPending'].includes(frame.state)) diagnostics.flushSoon();
            }
            if (frame.state === 'inputLost') { cancel('inputLost', frame.inputLoss); return; }
            setState(frame.state);
            try { active.socket.send(JSON.stringify({twist: teleopTwist(frame.linear, frame.angular), deadman: frame.deadman})); }
            catch { diagnostics.record('sendFailed'); cancel('connectionLost'); return; }
            if (frame.action && (frame.action === 'stop' || !pending.current)) {
                const action = frame.action;
                const previous = actionTask.current;
                const runAction = opts.current.runAction;
                pending.current = true;
                pendingSince.current = now;
                diagnostics.record('actionStarted', {attempt: active.attempt, action});
                const task = (async () => {
                    if (action === 'stop' && previous) {
                        // STOP must bypass a hung recording request. Reassert it
                        // after that request completes so a late start cannot win.
                        try { await runAction('stop'); }
                        finally { await previous.catch(() => {}); await runAction('stop'); }
                    } else await runAction(action);
                })();
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
    return {pads, enabled, state, fault, inputLoss, enable, disable};
}
