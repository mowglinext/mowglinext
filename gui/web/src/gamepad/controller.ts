import type {TwistStamped} from '../types/ros';

// Match the existing touch joystick caps. These affect teleop only.
export const MAX_LINEAR_MPS = 0.25;
export const MAX_ANGULAR_RAD_S = 0.6;
export const POLL_MAX_GAP_MS = 200;
export const ACTION_HOLD_MS = 800;
export const ACTION_COMMANDS = {stop: 8, recordStart: 3, recordFinish: 5, recordCancel: 6} as const;
export type ControllerAction = keyof typeof ACTION_COMMANDS;
export const BUTTON_BINDINGS = ['deadman', 'stop', 'recordStart', 'recordFinish', 'recordCancel'] as const;
export type ButtonBinding = 'deadman' | ControllerAction;
export interface ControllerProfile {
    version: 1;
    linearAxis: number;
    angularAxis: number;
    invertLinear: boolean;
    invertAngular: boolean;
    deadzone: number;
    speedScale: number;
    buttons: Record<ButtonBinding, number>;
}
export const DEFAULT_PROFILE: ControllerProfile = {
    version: 1, linearAxis: 1, angularAxis: 0,
    invertLinear: true, invertAngular: true, deadzone: 0.12, speedScale: 1,
    buttons: {deadman: 5, stop: 0, recordStart: 3, recordFinish: 2, recordCancel: 1},
};
export type ControllerSample = Pick<Gamepad, 'id' | 'index' | 'connected' | 'timestamp' | 'axes' | 'buttons' | 'mapping'>;
export type ControllerIdentity = Pick<Gamepad, 'id' | 'index'>;
export function controllerKey(pad: ControllerSample): string {
    return JSON.stringify([pad.id, pad.mapping, pad.axes.length, pad.buttons.length]);
}
export function validProfile(value: unknown): value is ControllerProfile {
    if (!value || typeof value !== 'object') return false;
    const p = value as ControllerProfile;
    const index = (n: unknown) => Number.isInteger(n) && Number(n) >= 0 && Number(n) < 64;
    return p.version === 1 && index(p.linearAxis) && index(p.angularAxis) && p.linearAxis !== p.angularAxis &&
        typeof p.invertLinear === 'boolean' && typeof p.invertAngular === 'boolean' &&
        Number.isFinite(p.deadzone) && p.deadzone >= 0.05 && p.deadzone <= 0.4 &&
        Number.isFinite(p.speedScale) && p.speedScale >= 0.1 && p.speedScale <= 1 &&
        !!p.buttons && BUTTON_BINDINGS.every(key => index(p.buttons[key])) &&
        new Set(BUTTON_BINDINGS.map(key => p.buttons[key])).size === BUTTON_BINDINGS.length;
}
export function profileFits(p: ControllerProfile, pad: ControllerSample): boolean {
    return validProfile(p) && p.linearAxis < pad.axes.length && p.angularAxis < pad.axes.length &&
        BUTTON_BINDINGS.every(key => p.buttons[key] < pad.buttons.length);
}
export function teleopTwist(linear = 0, angular = 0): TwistStamped {
    return {
        header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
        twist: {linear: {x: linear, y: 0, z: 0}, angular: {x: 0, y: 0, z: angular}},
    };
}
function axis(value: number, deadzone: number): number {
    return Math.abs(value) <= deadzone ? 0 : Math.sign(value) * (Math.abs(value) - deadzone) / (1 - deadzone);
}
export interface ControllerContext {
    stateName: string;
    statusFresh: boolean;
    emergency: boolean;
    pending: boolean;
}
export type ControllerState = 'releaseControls' | 'ready' | 'driving' | 'inputLost' | 'waitingForMode' | 'holding';
export type InputLossReason = 'pollGap' | 'clockInvalid' | 'disconnected' | 'identityChanged' | 'mappingChanged' |
    'timestampInvalid' | 'timestampRegressed' | 'axisInvalid' | 'buttonInvalid' |
    'focusLost' | 'pageHidden' | 'pageExit' | 'controllerChanged';
export interface InputLoss {
    reason: InputLossReason;
    pollGapMs?: number;
    timestamp?: number;
    previousTimestamp?: number;
    inputIndex?: number;
    inputValue?: number;
}
export interface ControllerFrame {
    linear: number;
    angular: number;
    deadman: boolean;
    state: ControllerState;
    action?: ControllerAction;
    inputLoss?: InputLoss;
}

/** One explicit browser session. Faults are terminal: create a new driver only on Enable.
 * Gamepad.timestamp is an opaque input-change marker, not a heartbeat. Safari can
 * use a different origin/unit from performance.now() and update only on changes.
 * Check its ordering only; polling liveness belongs to the monotonic `now` clock.
 */
export class ControllerDriver {
    private neutralSeen = false;
    private lastPoll: number | null = null;
    private lastStamp = -1;
    private inputLoss: InputLoss | null = null;
    private held: {action: ControllerAction; since: number; fired: boolean} | null = null;
    private previousMode: string | null = null;
    private deadmanHeld = false;

    constructor(private profile: ControllerProfile, private identity: ControllerIdentity) {}

    step(pad: ControllerSample | null, now: number, ctx: ControllerContext): ControllerFrame {
        const zero = (state: ControllerState, action?: ControllerAction): ControllerFrame =>
            ({linear: 0, angular: 0, deadman: false, state, ...(action ? {action} : {})});
        const lost = (inputLoss: InputLoss): ControllerFrame => {
            this.inputLoss ??= inputLoss;
            return {...zero('inputLost'), inputLoss: this.inputLoss};
        };
        if (this.inputLoss) return lost(this.inputLoss);
        const pollGapMs = this.lastPoll === null ? 0 : now - this.lastPoll;
        this.lastPoll = now;
        if (!Number.isFinite(now) || pollGapMs < 0) return lost({reason: 'clockInvalid'});
        if (pollGapMs > POLL_MAX_GAP_MS) return lost({reason: 'pollGap', pollGapMs});
        if (!pad || !pad.connected) return lost({reason: 'disconnected'});
        if (pad.id !== this.identity.id || pad.index !== this.identity.index) return lost({reason: 'identityChanged'});
        if (!profileFits(this.profile, pad)) return lost({reason: 'mappingChanged'});
        const stamp = {timestamp: pad.timestamp, previousTimestamp: this.lastStamp};
        if (!Number.isFinite(pad.timestamp) || pad.timestamp <= 0) return lost({reason: 'timestampInvalid', ...stamp});
        if (pad.timestamp < this.lastStamp) return lost({reason: 'timestampRegressed', ...stamp});
        const badAxis = pad.axes.findIndex(v => !Number.isFinite(v) || Math.abs(v) > 1);
        if (badAxis !== -1) return lost({reason: 'axisInvalid', inputIndex: badAxis, inputValue: pad.axes[badAxis]});
        const badButton = pad.buttons.findIndex(b => typeof b.pressed !== 'boolean' || !Number.isFinite(b.value) || b.value < 0 || b.value > 1);
        if (badButton !== -1) return lost({reason: 'buttonInvalid', inputIndex: badButton, inputValue: pad.buttons[badButton].value});
        this.lastStamp = pad.timestamp;
        const p = this.profile;
        const linear = axis(pad.axes[p.linearAxis], p.deadzone) * (p.invertLinear ? -1 : 1);
        const angular = axis(pad.axes[p.angularAxis], p.deadzone) * (p.invertAngular ? -1 : 1);
        const pressed = (key: ButtonBinding) => pad.buttons[p.buttons[key]].pressed || pad.buttons[p.buttons[key]].value >= 0.5;
        // Track physical edges even while STOP or another interlock inhibits motion.
        const wasHeld = this.deadmanHeld;
        this.deadmanHeld = pressed('deadman');
        // STOP is immediate, once per press, and takes precedence over every other input.
        if (pressed('stop')) {
            const fire = this.held?.action !== 'stop';
            this.held = {action: 'stop', since: now, fired: true};
            this.neutralSeen = false;
            return zero('releaseControls', fire ? 'stop' : undefined);
        }
        if (!ctx.statusFresh || ctx.emergency || ctx.pending) {
            this.neutralSeen = false;
            this.held = null;
            return zero('waitingForMode');
        }
        if (this.previousMode !== ctx.stateName) {
            this.neutralSeen = false;
            this.held = null;
            this.previousMode = ctx.stateName;
        }
        const allReleased = BUTTON_BINDINGS.every(key => !pressed(key));
        if (!this.neutralSeen) {
            if (allReleased && linear === 0 && angular === 0) this.neutralSeen = true;
            this.held = null;
            return zero('releaseControls');
        }
        const recording = ctx.stateName === 'RECORDING';
        if (pressed('deadman')) {
            if ((!wasHeld && (linear !== 0 || angular !== 0)) ||
                Object.keys(ACTION_COMMANDS).some(key => pressed(key as ControllerAction))) {
                this.neutralSeen = false;
                this.held = null;
                return zero('releaseControls');
            }
            this.held = null;
            if (!recording && ctx.stateName !== 'MANUAL_MOWING') return zero('waitingForMode');
            // Axis must be neutral on each new deadman press. After release, rearm below.
            return {linear: linear * MAX_LINEAR_MPS * p.speedScale,
                angular: angular * MAX_ANGULAR_RAD_S * p.speedScale, deadman: true, state: 'driving'};
        }
        if (linear !== 0 || angular !== 0) {
            this.neutralSeen = false;
            this.held = null;
            return zero('releaseControls');
        }
        const allowed: ControllerAction[] = recording ? ['recordFinish', 'recordCancel'] :
            ['IDLE', 'IDLE_DOCKED'].includes(ctx.stateName) ? ['recordStart'] : [];
        const down = (Object.keys(ACTION_COMMANDS) as ControllerAction[]).filter(key => pressed(key));
        if (down.length !== 1 || !allowed.includes(down[0])) {
            this.held = null;
            return zero('ready');
        }
        const action = down[0];
        if (this.held?.action !== action) this.held = {action, since: now, fired: false};
        if (!this.held.fired && now - this.held.since >= ACTION_HOLD_MS) {
            this.held.fired = true;
            this.neutralSeen = false;
            return zero('releaseControls', action);
        }
        return zero('holding');
    }
}

export function loadProfile(key: string): ControllerProfile | null {
    try {
        const parsed: unknown = JSON.parse(localStorage.getItem(`mowglinext.gamepad.v1:${key}`) ?? 'null');
        return validProfile(parsed) ? parsed : null;
    } catch { return null; }
}
export function saveProfile(key: string, profile: ControllerProfile): void {
    if (!validProfile(profile)) throw new Error('Invalid controller profile');
    localStorage.setItem(`mowglinext.gamepad.v1:${key}`, JSON.stringify(profile));
}
