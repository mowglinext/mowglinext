import {beforeEach, describe, expect, it, vi} from 'vitest';
import {
    ACTION_COMMANDS, ACTION_HOLD_MS, ControllerDriver, DEFAULT_PROFILE,
    MAX_ANGULAR_RAD_S, MAX_LINEAR_MPS, POLL_MAX_GAP_MS, loadProfile, loadSelectedControllerKey, profileFits,
    saveProfile, saveSelectedControllerKey, validProfile, type ControllerContext, type ControllerProfile, type ControllerSample,
    type InputLoss,
} from './controller';

const idle: ControllerContext = {stateName: 'IDLE', statusFresh: true, emergency: false, pending: false};

function sample(overrides: Partial<ControllerSample> = {}): ControllerSample {
    return {
        id: 'test pad', index: 0, mapping: 'standard', connected: true, timestamp: 1000,
        axes: [0, 0],
        buttons: Array.from({length: 8}, () => ({pressed: false, touched: false, value: 0})),
        ...overrides,
    } as ControllerSample;
}

function pressed(key: keyof ControllerProfile['buttons'], value = true): ControllerSample {
    const buttons = sample().buttons.slice();
    buttons[DEFAULT_PROFILE.buttons[key]] = {pressed: value, touched: value, value: value ? 1 : 0};
    return sample({buttons});
}

function buttonsDown(...keys: Array<keyof ControllerProfile['buttons']>) {
    const buttons = sample().buttons.slice();
    for (const key of keys) {
        buttons[DEFAULT_PROFILE.buttons[key]] = {pressed: true, touched: true, value: 1};
    }
    return buttons;
}

function prime(driver: ControllerDriver, mode = idle.stateName): number {
    const ctx = {...idle, stateName: mode};
    driver.step(sample({timestamp: 1000}), 1000, ctx);
    driver.step(sample({timestamp: 1050}), 1050, ctx);
    return 1050;
}

describe('controller profile validation and storage', () => {
    beforeEach(() => localStorage.clear());

    it('rejects invalid, conflicting, and out-of-range profile values', () => {
        expect(validProfile(DEFAULT_PROFILE)).toBe(true);
        expect(validProfile({...DEFAULT_PROFILE, version: 2})).toBe(false);
        expect(validProfile({...DEFAULT_PROFILE, linearAxis: DEFAULT_PROFILE.angularAxis})).toBe(false);
        expect(validProfile({...DEFAULT_PROFILE, deadzone: 0.41})).toBe(false);
        expect(validProfile({...DEFAULT_PROFILE, speedScale: 0})).toBe(false);
        expect(validProfile({...DEFAULT_PROFILE, buttons: {...DEFAULT_PROFILE.buttons, stop: 5}})).toBe(false);
        expect(validProfile({...DEFAULT_PROFILE, buttons: {...DEFAULT_PROFILE.buttons, stop: -1}})).toBe(false);
        expect(profileFits(DEFAULT_PROFILE, sample({axes: [0], buttons: Array.from({length: 5}, () => ({pressed: false, touched: false, value: 0}))}))).toBe(false);
    });

    it('loads corrupt storage as absent and refuses to save invalid profiles', () => {
        const key = 'pad-layout';
        localStorage.setItem(`mowglinext.gamepad.v1:${key}`, '{broken');
        expect(loadProfile(key)).toBeNull();
        localStorage.setItem(`mowglinext.gamepad.v1:${key}`, JSON.stringify({...DEFAULT_PROFILE, buttons: {...DEFAULT_PROFILE.buttons, stop: 5}}));
        expect(loadProfile(key)).toBeNull();
        expect(() => saveProfile(key, {...DEFAULT_PROFILE, speedScale: 2})).toThrow('Invalid controller profile');
        saveProfile(key, DEFAULT_PROFILE);
        expect(loadProfile(key)).toEqual(DEFAULT_PROFILE);
    });

    it('remembers only a controller fingerprint and tolerates denied selection reads', () => {
        expect(loadSelectedControllerKey()).toBeNull();
        const key = JSON.stringify(['test pad', 'standard', 2, 8]);
        saveSelectedControllerKey(key);
        expect(loadSelectedControllerKey()).toBe(key);
        const denied = vi.spyOn(Storage.prototype, 'getItem').mockImplementation(() => { throw new Error('Storage denied'); });
        expect(loadSelectedControllerKey()).toBeNull();
        denied.mockRestore();
    });
});

describe('ControllerDriver', () => {
    it('waits for a drive mode in IDLE and requires release after entering Manual Mowing', () => {
        const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        let now = prime(driver);
        const step = (context = idle, down = false, linear = 0) => driver.step(sample({
            timestamp: now += 50, buttons: down ? buttonsDown('deadman') : sample().buttons, axes: [0, linear],
        }), now, context);
        expect(step()).toMatchObject({state: 'waitingForMode', deadman: false, linear: 0});
        expect(step(idle, true)).toMatchObject({state: 'waitingForMode', deadman: false});
        const manual = {...idle, stateName: 'MANUAL_MOWING'};
        expect(step(manual, true)).toMatchObject({state: 'releaseControls', deadman: false});
        expect(step(manual, true, -1)).toMatchObject({state: 'releaseControls', deadman: false, linear: 0});
        step(manual);
        expect(step(manual)).toMatchObject({state: 'ready', deadman: false});
        const armed = step(manual, true);
        expect(armed).toMatchObject({state: 'driving', deadman: true});
        expect(armed.linear).toBeCloseTo(0);
        expect(step(manual, true, -1).linear).toBe(MAX_LINEAR_MPS);
    });

    it.each([
        [{statusFresh: false}, 'statusLost'],
        [{emergency: true}, 'emergency'],
        [{pending: true}, 'actionPending'],
    ] as const)('explains the active interlock without changing neutral rearming (%s)', (guard, state) => {
        const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        const recording = {...idle, stateName: 'RECORDING'};
        let now = prime(driver, recording.stateName);
        const held = sample({timestamp: now, buttons: buttonsDown('deadman')});
        expect(driver.step(held, now += 50, recording).deadman).toBe(true);
        expect(driver.step(held, now += 50, {...recording, ...guard})).toMatchObject({state, deadman: false, linear: 0});
        expect(driver.step(held, now += 50, recording)).toMatchObject({state: 'releaseControls', deadman: false});
        driver.step(sample({timestamp: held.timestamp}), now += 50, recording);
        expect(driver.step(held, now += 50, recording).deadman).toBe(true);
    });

    it.each<[string, ControllerSample | null, Partial<InputLoss>]>([
        ['missing device', null, {reason: 'disconnected'}],
        ['disconnected device', sample({connected: false}), {reason: 'disconnected'}],
        ['different device', sample({id: 'replacement'}), {reason: 'identityChanged'}],
        ['changed layout', sample({axes: [0]}), {reason: 'mappingChanged'}],
        ['zero timestamp', sample({timestamp: 0}), {reason: 'timestampInvalid', timestamp: 0, previousTimestamp: 1000}],
        ['regressed timestamp', sample({timestamp: 999}), {reason: 'timestampRegressed', timestamp: 999, previousTimestamp: 1000}],
        ['out-of-range axis', sample({axes: [0, 1.01]}), {reason: 'axisInvalid', inputIndex: 1, inputValue: 1.01}],
        ['out-of-range button', sample({buttons: sample().buttons.map((b, i) => i === 5 ? {...b, value: -0.01} : b)}),
            {reason: 'buttonInvalid', inputIndex: 5, inputValue: -0.01}],
    ])('retains the exact first failure for %s without emitting motion', (_name, badPad, detail) => {
        const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        driver.step(sample(), 1000, idle);
        const failed = driver.step(badPad, 1050, idle);
        expect(failed).toMatchObject({linear: 0, angular: 0, deadman: false, state: 'inputLost', inputLoss: detail});
        expect(driver.step(sample(), 2000, idle)).toEqual(failed);
    });

    it('reports the observed polling gap and rejects a regressed browser clock', () => {
        const gap = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        gap.step(sample(), 1000, idle);
        expect(gap.step(sample(), 1234.5, idle)).toMatchObject({state: 'inputLost', inputLoss: {reason: 'pollGap', pollGapMs: 234.5}});
        const clock = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        clock.step(sample(), 1000, idle);
        expect(clock.step(sample(), 999, idle)).toMatchObject({state: 'inputLost', inputLoss: {reason: 'clockInvalid'}});
    });

    it.each([12.345, 864000.25])('accepts a change-only timestamp in an independent clock domain (%s)', timestamp => {
        const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        const recording = {...idle, stateName: 'RECORDING'};
        let now = 1000;
        const step = (overrides: Partial<ControllerSample> = {}) => {
            now += 50;
            return driver.step(sample({timestamp, ...overrides}), now, recording);
        };
        expect(step().state).toBe('releaseControls');
        // The operator can leave centered controls unchanged before enabling/driving.
        for (let i = 0; i < 20; i++) expect(step().state).toBe('ready');
        expect(step({timestamp: timestamp + 0.001, buttons: buttonsDown('deadman')}).deadman).toBe(true);
        const held = {timestamp: timestamp + 0.002, axes: [0, -1], buttons: buttonsDown('deadman')};
        for (let i = 0; i < 25; i++) {
            expect(step(held)).toMatchObject({state: 'driving', deadman: true, linear: MAX_LINEAR_MPS});
        }
        expect(step({timestamp: timestamp + 0.003, axes: [0, -1]})).toMatchObject({deadman: false, linear: 0, angular: 0});
        expect(step({timestamp: timestamp + 0.004, connected: false}).state).toBe('inputLost');
        expect(step({timestamp: timestamp + 0.005}).state).toBe('inputLost');
    });

    it.each(['recordStart', 'recordFinish', 'recordCancel'] as const)('recognizes a held %s when the timestamp changes only on input events', action => {
        const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        const context = {...idle, stateName: action === 'recordStart' ? 'IDLE' : 'RECORDING'};
        let now = 1000;
        driver.step(sample({timestamp: 10}), now, context);
        driver.step(sample({timestamp: 10}), now += 50, context);
        const held = sample({timestamp: 10.001, buttons: buttonsDown(action)});
        expect(driver.step(held, now += 50, context).state).toBe('holding');
        for (let i = 0; i < 15; i++) expect(driver.step(held, now += 50, context).action).toBeUndefined();
        expect(driver.step(held, now += 50, context).action).toBe(action);
        expect(driver.step(held, now += 50, context).action).toBeUndefined();
    });

    it('requires neutral input, scales and caps axes, applies the deadzone and inversion', () => {
        const profile: ControllerProfile = {
            ...DEFAULT_PROFILE, invertLinear: false, invertAngular: true, deadzone: 0.2, speedScale: 0.5,
        };
        const driver = new ControllerDriver(profile, {id: 'test pad', index: 0});
        const manual = {...idle, stateName: 'MANUAL_MOWING'};
        driver.step(sample({timestamp: 1000}), 1000, manual);
        expect(driver.step(sample({timestamp: 1050}), 1050, manual).state).toBe('ready');

        const atDeadzone = driver.step(sample({timestamp: 1100, axes: [0.2, -0.2], buttons: pressed('deadman').buttons}), 1100, manual);
        expect(atDeadzone.linear).toBe(0);
        expect(atDeadzone.angular).toBeCloseTo(0);
        expect(atDeadzone).toMatchObject({deadman: true, state: 'driving'});
        const full = driver.step(sample({timestamp: 1150, axes: [-1, 1], buttons: pressed('deadman').buttons}), 1150, manual);
        expect(full.linear).toBe(MAX_LINEAR_MPS * 0.5);
        expect(full.angular).toBe(MAX_ANGULAR_RAD_S * 0.5);
    });

    it('stops when deadman is released and requires a neutral re-arm before a new press', () => {
        const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        const manual = {...idle, stateName: 'MANUAL_MOWING'};
        let now = prime(driver, manual.stateName);
        const neutralDeadman = sample({timestamp: now + 50, buttons: pressed('deadman').buttons});
        expect(driver.step(neutralDeadman, now += 50, manual)).toMatchObject({deadman: true, state: 'driving'});
        const moving = sample({timestamp: now + 50, axes: [0, -0.5], buttons: pressed('deadman').buttons});
        expect(driver.step(moving, now += 50, manual).deadman).toBe(true);

        const releasedWithMotion = sample({timestamp: now + 50, axes: [0, -0.5]});
        expect(driver.step(releasedWithMotion, now += 50, manual)).toMatchObject({linear: 0, angular: 0, deadman: false, state: 'releaseControls'});
        const neutral = sample({timestamp: now + 50});
        expect(driver.step(neutral, now += 50, manual).state).toBe('releaseControls');

        const pressedWhileMoving = sample({timestamp: now + 50, axes: [0, -0.5], buttons: pressed('deadman').buttons});
        expect(driver.step(pressedWhileMoving, now += 50, manual).state).toBe('releaseControls');
        const stillHeldAndMoving = sample({timestamp: now + 50, axes: [0, -0.5], buttons: pressed('deadman').buttons});
        expect(driver.step(stillHeldAndMoving, now += 50, manual)).toMatchObject({linear: 0, angular: 0, deadman: false, state: 'releaseControls'});
    });

    it.each([
        {reason: 'STOP', buttons: ['deadman', 'stop'] as const, action: 'stop'},
        {reason: 'an action pressed with deadman', buttons: ['deadman', 'recordFinish'] as const, action: undefined},
    ])('re-arms deadman after $reason in RECORDING', ({buttons: rejectedButtons, action}) => {
        const recording = {...idle, stateName: 'RECORDING'};
        const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        let now = prime(driver, recording.stateName);
        const step = (overrides: Partial<ControllerSample> = {}) => {
            now += 50;
            return driver.step(sample({timestamp: now, ...overrides}), now, recording);
        };

        expect(step({buttons: buttonsDown('deadman')})).toMatchObject({state: 'driving', deadman: true});
        expect(step({axes: [0, -0.5], buttons: buttonsDown('deadman')})).toMatchObject({state: 'driving', deadman: true});
        const rejected = step({axes: [0, -0.5], buttons: buttonsDown(...rejectedButtons)});
        expect(rejected.action).toBe(action);
        expect(rejected.state).toBe('releaseControls');
        expect(rejected.deadman).toBe(false);
        expect(rejected.linear).toBeCloseTo(0);
        expect(rejected.angular).toBeCloseTo(0);

        expect(step().state).toBe('releaseControls'); // All buttons and axes neutral.
        const newPressWithDeflection = step({axes: [0, -0.5], buttons: buttonsDown('deadman')});
        expect(newPressWithDeflection).toMatchObject({linear: 0, angular: 0, deadman: false, state: 'releaseControls'});
        expect(step({axes: [0, -0.5], buttons: buttonsDown('deadman')})).toMatchObject({
            linear: 0, angular: 0, deadman: false, state: 'releaseControls',
        });
        expect(step({axes: [0, -0.5]})).toMatchObject({linear: 0, angular: 0, deadman: false, state: 'releaseControls'});
        expect(step().state).toBe('releaseControls'); // Re-arm after release plus centered stick.
        const rearmed = step({buttons: buttonsDown('deadman')});
        expect(rearmed.state).toBe('driving');
        expect(rearmed.deadman).toBe(true);
        expect(rearmed.linear).toBeCloseTo(0);
        expect(rearmed.angular).toBeCloseTo(0);
        expect(step({axes: [0, -0.5], buttons: buttonsDown('deadman')}).deadman).toBe(true);
    });

    it('holds actions for 800 ms, fires once, gives STOP priority, and enforces mode guards', () => {
        expect(ACTION_COMMANDS).toEqual({stop: 8, recordStart: 3, recordFinish: 5, recordCancel: 6});
        expect(Object.values(ACTION_COMMANDS)).not.toContain(7);
        expect(ACTION_HOLD_MS).toBe(800);
        const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        let now = prime(driver);
        const finish = (t: number) => sample({timestamp: t, buttons: pressed('recordStart').buttons});
        expect(driver.step(finish(now += 50), now, idle).state).toBe('holding');
        let frame;
        for (let i = 0; i < ACTION_HOLD_MS / 100; i++) {
            now += 100;
            frame = driver.step(finish(now), now, idle);
        }
        expect(frame).toMatchObject({action: 'recordStart', state: 'releaseControls'});
        now += 50;
        expect(driver.step(finish(now), now, idle).action).toBeUndefined();
        now += 50;
        expect(driver.step(finish(now), now, idle).action).toBeUndefined();

        const stopDriver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        now = prime(stopDriver);
        const bothButtons = pressed('stop').buttons.slice();
        bothButtons[DEFAULT_PROFILE.buttons.recordStart] = {pressed: true, touched: true, value: 1};
        const both = sample({timestamp: now + 50, buttons: bothButtons});
        expect(stopDriver.step(both, now + 50, {...idle, emergency: true})).toMatchObject({action: 'stop', state: 'releaseControls'});
        expect(stopDriver.step(sample({timestamp: now + 100, buttons: bothButtons}), now + 100, idle).action).toBeUndefined();

        const guarded = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        now = prime(guarded, 'MANUAL_MOWING');
        const startInManual = sample({timestamp: now + 50, buttons: pressed('recordStart').buttons});
        expect(guarded.step(startInManual, now + 50, {...idle, stateName: 'MANUAL_MOWING'})).toMatchObject({state: 'ready'});
    });

    it('fails terminally on invalid, zero or regressed device timestamps', () => {
        const invalid: Array<[string, ControllerSample, number]> = [
            ['NaN', sample({timestamp: NaN}), 1000],
            ['infinite', sample({timestamp: Infinity}), 1000],
            ['negative', sample({timestamp: -1}), 1000],
            ['zero', sample({timestamp: 0}), 1000],
        ];
        for (const [name, pad, now] of invalid) {
            const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
            expect(driver.step(pad, now, idle).state, name).toBe('inputLost');
            expect(driver.step(sample({timestamp: now + 50}), now + 50, idle).state, name).toBe('inputLost');
        }

        const regressed = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        regressed.step(sample({timestamp: 1000}), 1000, idle);
        expect(regressed.step(sample({timestamp: 999}), 1050, idle).state).toBe('inputLost');
    });

    it('neutral rearming preserves timestamp ordering, polling deadlines and terminal faults', () => {
        const regressed = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        prime(regressed);
        regressed.requireNeutral();
        expect(regressed.step(sample({timestamp: 1049}), 1100, idle).inputLoss?.reason).toBe('timestampRegressed');
        regressed.requireNeutral();
        expect(regressed.step(sample({timestamp: 1150}), 1150, idle).inputLoss?.reason).toBe('timestampRegressed');

        const delayed = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        prime(delayed);
        delayed.requireNeutral();
        expect(delayed.step(sample({timestamp: 1300}), 1300, idle).inputLoss?.reason).toBe('pollGap');
    });

    it('fails terminally for missing, disconnected, replaced, or late input', () => {
        const cases: Array<[string, ControllerSample | null, number]> = [
            ['missing', null, 1000],
            ['disconnected', sample({connected: false}), 1000],
            ['replaced', sample({id: 'another pad'}), 1000],
        ];
        for (const [name, pad, now] of cases) {
            const driver = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
            expect(driver.step(pad, now, idle).state, name).toBe('inputLost');
            expect(driver.step(sample({timestamp: now + 50}), now + 50, idle).state, name).toBe('inputLost');
        }

        const gap = new ControllerDriver(DEFAULT_PROFILE, {id: 'test pad', index: 0});
        gap.step(sample({timestamp: 1000}), 1000, idle);
        expect(gap.step(sample({timestamp: 1000 + POLL_MAX_GAP_MS}), 1000 + POLL_MAX_GAP_MS + 1, idle).state).toBe('inputLost');
        expect(gap.step(sample({timestamp: 1300}), 1300, idle).state).toBe('inputLost');
    });
});
