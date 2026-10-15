import {useCallback, useEffect, useLayoutEffect, useRef, useState} from 'react';
import {wsBase} from '../utils/apiHost';
import {useTopic} from '../hooks/useTopic';
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
    const session = useRef<{socket: WebSocket; driver: ControllerDriver; identity: ControllerIdentity; opened: number} | null>(null);
    const pending = useRef(false);
    const actionTask = useRef<Promise<void> | null>(null);
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
        if (mounted.current) {
            setEnabled(false);
            setState('releaseControls');
            setFault(reason);
            if (detail) setInputLoss(detail);
        }
    }, []);
    const disable = () => cancel();
    const enable = () => {
        cancel();
        setInputLoss(null);
        if (typeof navigator.getGamepads !== 'function') { setFault('unsupported'); return; }
        const {selected, profile} = opts.current;
        let pad: Gamepad | null | undefined;
        try { pad = selected ? navigator.getGamepads()[selected.index] : null; }
        catch { setFault('unsupported'); return; }
        if (!pad || !pad.connected || !selected || pad.id !== selected.id || pad.index !== selected.index || !profileFits(profile, pad)) {
            setFault('inputLost'); setInputLoss({reason: !pad || !pad.connected ? 'disconnected' :
                !selected || pad.id !== selected.id || pad.index !== selected.index ? 'identityChanged' : 'mappingChanged'}); return;
        }
        if (document.visibilityState !== 'visible' || !document.hasFocus()) {
            setFault('inputLost'); setInputLoss(document.visibilityState !== 'visible' ? {reason: 'pageHidden'} : focusInputLoss('enable')); return;
        }
        let socket: WebSocket;
        try { socket = new WebSocket(`${wsBase()}/api/mowglinext/publish/gamepad`); }
        catch { setFault('connectionLost'); return; }
        const active = {socket, driver: new ControllerDriver(structuredClone(profile), selected), identity: selected, opened: performance.now()};
        session.current = active;
        socket.onclose = () => { if (session.current === active) cancel('connectionLost'); };
        socket.onerror = () => { if (session.current === active) cancel('connectionLost'); };
        opts.current.onOwnershipChange(true);
        setEnabled(true);
        setFault(null);
        setState('connecting');
    };

    useEffect(() => {
        mounted.current = true;
        let lastDisplay = -Infinity;
        let lastInventory = '';
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
            if (!active) return;
            if (document.visibilityState !== 'visible' || !document.hasFocus()) {
                cancel('inputLost', document.visibilityState !== 'visible' ? {reason: 'pageHidden'} : focusInputLoss('poll')); return;
            }
            if (active.socket.readyState === WebSocket.CONNECTING) {
                if (now - active.opened > 2000) cancel('connectionLost');
                return;
            }
            if (active.socket.readyState !== WebSocket.OPEN || active.socket.bufferedAmount !== 0) {
                cancel('connectionLost'); return;
            }
            const pad = devices.find(p => p.index === active.identity.index && p.id === active.identity.id) ?? null;
            const current = context.current;
            const frame = active.driver.step(pad, now, {
                stateName: current.status.data.state_name ?? '',
                // Delivery liveness only; robot-side interlocks remain authoritative.
                statusFresh: current.status.lastMessageAt !== null && now >= current.status.lastMessageAt && now - current.status.lastMessageAt < 2500,
                emergency: !!(current.status.data.emergency || current.emergency.data.active_emergency || current.emergency.data.latched_emergency),
                pending: pending.current,
            });
            if (frame.state === 'inputLost') { cancel('inputLost', frame.inputLoss); return; }
            setState(frame.state);
            try { active.socket.send(JSON.stringify({twist: teleopTwist(frame.linear, frame.angular), deadman: frame.deadman})); }
            catch { cancel('connectionLost'); return; }
            if (frame.action && (frame.action === 'stop' || !pending.current)) {
                const action = frame.action;
                const previous = actionTask.current;
                const runAction = opts.current.runAction;
                pending.current = true;
                const task = (async () => {
                    if (action === 'stop' && previous) {
                        // STOP must bypass a hung recording request. Reassert it
                        // after that request completes so a late start cannot win.
                        try { await runAction('stop'); }
                        finally { await previous.catch(() => {}); await runAction('stop'); }
                    } else await runAction(action);
                })();
                actionTask.current = task;
                void task.catch(() => {
                    if (session.current === active) cancel('actionFailed');
                }).finally(() => {
                    if (actionTask.current === task) {
                        actionTask.current = null;
                        pending.current = false;
                    }
                });
            }
        };
        const interval = window.setInterval(poll, 50);
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
            window.removeEventListener('blur', suspend);
            window.removeEventListener('pagehide', suspend);
            document.removeEventListener('visibilitychange', suspend);
            window.removeEventListener('gamepaddisconnected', disconnect);
            window.removeEventListener('gamepadconnected', disconnect);
            cancel();
        };
    }, [cancel]);
    return {pads, enabled, state, fault, inputLoss, enable, disable};
}
