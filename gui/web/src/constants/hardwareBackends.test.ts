import { describe, expect, it } from "vitest";
import {
    appliesToBackend,
    parseHardwareBackendInfo,
    hardwareParameterValue,
    planHardwareParameters,
    liveHardwareParameters,
    normalizeHardwareBackend,
    presetValuesForBackend,
    settingsSectionsForBackend,
} from "./hardwareBackends.ts";
import { MOWER_MODELS } from "./mowerModels.ts";
import {
    FIRMWARE_SAFETY_GROUP,
    OPENMOWER_WIRING_GROUP,
    YAW_LOOP_GROUP,
    groupForBackend,
    groupKeys,
} from "../components/settings/settingsFieldGroups.ts";

describe("normalizeHardwareBackend", () => {
    it("keeps known backends and does not invent identity", () => {
        expect(normalizeHardwareBackend("openmower")).toBe("openmower");
        expect(normalizeHardwareBackend("mavros")).toBe("mavros");
        expect(normalizeHardwareBackend("bogus")).toBe("unknown");
        expect(normalizeHardwareBackend(undefined)).toBe("unknown");
    });
});

describe("appliesToBackend", () => {
    it("treats an absent limit as every backend", () => {
        expect(appliesToBackend(undefined, "openmower")).toBe(true);
        expect(appliesToBackend(["mowgli"], "openmower")).toBe(false);
        expect(appliesToBackend(["mowgli", "openmower"], "openmower")).toBe(true);
    });
});

describe("presetValuesForBackend", () => {
    it("never writes a model's STM32 ticks_per_meter on an OpenMower robot", () => {
        const yardforce = MOWER_MODELS.find((m) => m.value === "YardForce500")!;
        const overrides = { ticks_per_meter: 1600, both_wheels_lift_emergency_ms: 100 };

        const values = presetValuesForBackend(yardforce.defaults, overrides);

        expect(values.ticks_per_meter).toBe(1600);
        expect(values.wheel_track).toBe(yardforce.defaults.wheel_track);
        expect(values).not.toHaveProperty("both_wheels_lift_emergency_ms");
    });

    it("leaves the preset untouched without backend overrides", () => {
        const preset = { ticks_per_meter: 300, wheel_track: 0.325 };
        expect(presetValuesForBackend(preset, {})).toEqual(preset);
    });
});

describe("groupForBackend", () => {
    it("hides STM32-only fields behind the OpenMower board", () => {
        const om = groupForBackend(FIRMWARE_SAFETY_GROUP, "openmower");
        expect(om && groupKeys(om)).toEqual([
            "max_mps",
            "one_wheel_lift_emergency_ms",
            "both_wheels_lift_emergency_ms",
        ]);
        expect(groupForBackend(FIRMWARE_SAFETY_GROUP, "mowgli")?.fields).toHaveLength(
            FIRMWARE_SAFETY_GROUP.fields.length,
        );
    });

    it("drops a group that does not exist on the backend", () => {
        expect(groupForBackend(YAW_LOOP_GROUP, "openmower")).toBeNull();
        expect(groupForBackend(OPENMOWER_WIRING_GROUP, "mowgli")).toBeNull();
        expect(groupForBackend(OPENMOWER_WIRING_GROUP, "openmower")).not.toBeNull();
    });
});

describe("hardware backend settings", () => {
    it("keeps the Drive section for all three backends", () => {
        const sections = [
            { id: "hardware" },
            { id: "drive_motor" },
            { id: "navigation" },
        ];

        expect(settingsSectionsForBackend(sections, "mowgli")).toEqual(sections);
        expect(settingsSectionsForBackend(sections, "mavros")).toEqual(sections);
        expect(settingsSectionsForBackend(sections, "openmower")).toEqual(sections);
    });

    it("does not send runtime-pending MAVROS routes or Mowgli PID parameters", () => {
        const routes = {
            ticks_per_meter: {
                parameter: "mavros/esc_wheel_odometry.ticks_per_meter",
                runtime: "available" as const,
            },
            wheel_track: {
                parameter: "mavros/esc_wheel_odometry.track_width_m",
                runtime: "available" as const,
            },
        };
        expect(routes.ticks_per_meter.parameter).toBe("mavros/esc_wheel_odometry.ticks_per_meter");
        expect(routes.wheel_track.parameter).toBe("mavros/esc_wheel_odometry.track_width_m");

        const parameters = liveHardwareParameters(
            new Set(["ticks_per_meter", "wheel_track", "wheel_pid_kp"]),
            { ticks_per_meter: 401.5, wheel_track: 0.325, wheel_pid_kp: 10 },
            parseHardwareBackendInfo({backend: 'mavros', parameter_routes: routes, runtime_routing: 'available'}),
        );
        expect(parameters).toEqual([
            { name: "mavros/esc_wheel_odometry.ticks_per_meter", value: 401.5, type: 'float64' },
            { name: "mavros/esc_wheel_odometry.track_width_m", value: 0.325, type: 'float64' },
        ]);
    });

    it("preserves the existing Mowgli live parameter requests", () => {
        const routes = {
            ticks_per_meter: {
                parameter: "hardware_bridge.ticks_per_meter",
                runtime: "available" as const,
            },
            wheel_pid_kp: {
                parameter: "hardware_bridge.wheel_pid_kp",
                runtime: "available" as const,
            },
            wheel_pid_pwm_per_mps: {
                parameter: "hardware_bridge.wheel_pid_pwm_per_mps",
                runtime: "available" as const,
            },
        };
        expect(liveHardwareParameters(
            new Set(Object.keys(routes)),
            { ticks_per_meter: 399, wheel_pid_kp: 10, wheel_pid_pwm_per_mps: 282.135 },
            parseHardwareBackendInfo({backend: 'mowgli', parameter_routes: routes, runtime_routing: 'available'}),
        )).toEqual([
            { name: "hardware_bridge.ticks_per_meter", value: 399, type: 'float64' },
            { name: "hardware_bridge.wheel_pid_kp", value: 10, type: 'float64' },
            { name: "hardware_bridge.wheel_pid_pwm_per_mps", value: 282.135, type: 'float64' },
        ]);
    });
    it("routes MAVROS feed-forward through the shared canonical value", () => {
        const routes = {
            wheel_pid_pwm_per_mps: { parameter: "hardware_bridge.manual_control_linear_scale", runtime: "available" as const },
        };
        expect(liveHardwareParameters(new Set(Object.keys(routes)), {
            wheel_pid_pwm_per_mps: 282.135,
        }, parseHardwareBackendInfo({backend: 'mavros', parameter_routes: routes, runtime_routing: 'available'}))).toEqual([
            { name: "hardware_bridge.manual_control_linear_scale", value: 282.135, type: 'float64' },
        ]);
    });
});

describe('backend routing integrity', () => {
    const info = (backend: string, runtime = 'available', parameter = 'hardware_bridge.ticks_per_meter') =>
        parseHardwareBackendInfo({backend, runtime_routing: 'available', parameter_routes: {ticks_per_meter: {parameter, runtime}}});
    const badMetadata: unknown[] = [null, [], {}, {backend: 1}, {backend: 'bogus'}, {backend: 'mowgli', runtime_routing: 'bogus'},
        {backend: 'mowgli', parameter_routes: {ticks_per_meter: {parameter: '', runtime: 'available'}}},
        {backend: 'mavros', runtime_routing: 'available', parameter_routes: {wheel_pid_kp: {parameter: 'hardware_bridge.wheel_pid_kp', runtime: 'available'}}},
        {backend: 'openmower', parameter_routes: {ticks_per_meter: {parameter: 'hardware_bridge.ticks_per_meter', runtime: 'available'}}},
        {backend: 'mowgli', parameter_routes: {toString: {runtime: 'available'}}}];
    it.each(badMetadata.map(raw => ({raw})))('never authorizes bad metadata $raw', ({raw}) => {
        const parsed = parseHardwareBackendInfo(raw);
        expect(parsed.authority).not.toBe('confirmed'); expect(parsed.parameterRoutes).toEqual({});
        expect(liveHardwareParameters(new Set(['ticks_per_meter']), {ticks_per_meter: 1}, parsed)).toEqual([]);
    });
    it.each(['available', 'pending_image', 'restart_required'])('retains route state %s, authorizes only available', runtime => {
        const parsed = info('mowgli', runtime);
        expect(parsed.parameterRoutes.ticks_per_meter.runtime).toBe(runtime);
        expect(planHardwareParameters(new Set(['ticks_per_meter']), {ticks_per_meter: 0}, parsed).liveKeys.size).toBe(runtime === 'available' ? 1 : 0);
    });
    it.each([null, undefined, '', ' ', 'NaN', '1m', '0x10', NaN, Infinity, -Infinity, true, [], {}])('does not coerce bad double %j', value => {
        const plan = planHardwareParameters(new Set(['ticks_per_meter']), {ticks_per_meter: value}, info('mowgli'));
        expect(plan.parameters).toEqual([]); expect(plan.liveKeys.size).toBe(0);
    });
    it.each([0, 1.5, '0', ' 1.5 ', '1e2'])('retains valid double %j', value => {
        expect(hardwareParameterValue(value, 'float64')).toBe(Number(value));
    });
    it.each([true, false])('preserves a real boolean %s without numeric coercion', value => {
        expect(hardwareParameterValue(value, 'bool')).toBe(value);
        expect(hardwareParameterValue(value, 'float64')).toBeUndefined();
    });
    it('does not manufacture working routes for a legacy identity-only response', () => {
        const parsed = parseHardwareBackendInfo({backend: 'mowgli'});
        expect(parsed.backend).toBe('mowgli'); expect(parsed.runtimeRouting).toBe('unavailable');
        expect(parsed.parameterRoutes).toEqual({});
    });
    it('a backend-wide unavailable state blocks individual available routes', () => {
        const parsed = {...info('mowgli'), runtimeRouting: 'pending_image' as const};
        expect(planHardwareParameters(new Set(['ticks_per_meter']), {ticks_per_meter: 0}, parsed).parameters).toEqual([]);
    });
    it('does not accept contradictory MAVROS blade authorization as live', () => {
        const parsed = parseHardwareBackendInfo({backend: 'mavros', runtime_routing: 'available', parameter_routes: {
            mowing_enabled: {parameter: 'hardware_bridge.mowing_enabled', runtime: 'available'},
        }});
        expect(parsed.authority).toBe('malformed');
    });
});

it('does not authorize OpenMower hardware presets without encoder defaults', () => {
    expect(parseHardwareBackendInfo({backend: 'openmower', default_overrides: {}}).presetsAvailable).toBe(false);
    expect(parseHardwareBackendInfo({backend: 'openmower', default_overrides: {ticks_per_meter: 1600}}).presetsAvailable).toBe(true);
    expect(parseHardwareBackendInfo({backend: 'mowgli'}).presetsAvailable).toBe(false);
});
it('rejects contradictory support claims and malformed target defaults', () => {
    expect(parseHardwareBackendInfo({backend: 'mowgli', supported: ['mavros']}).authority).toBe('malformed');
    expect(parseHardwareBackendInfo({backend: 'mowgli', default_overrides: {ticks_per_meter: ''}}).authority).toBe('malformed');
});

it('does not authorize presets with malformed numeric geometry overrides', () => {
    expect(parseHardwareBackendInfo({backend: 'mowgli', default_overrides: {wheel_radius: 'invalid'}}).authority).toBe('malformed');
});
