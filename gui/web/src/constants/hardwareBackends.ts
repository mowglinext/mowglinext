import {isJsonObject} from '../utils/json';
import {MOWER_MODELS} from './mowerModels';

export type HardwareBackend = 'mowgli' | 'mavros' | 'openmower' | 'unknown';
export type RuntimeRouting = 'available' | 'pending_image' | 'restart_required' | 'unavailable';
export type BackendAuthority = 'loading' | 'confirmed' | 'legacy_absent' | 'unavailable' | 'malformed' | 'unsupported';
export type HardwareParameterRoute = {parameter: string; runtime: Exclude<RuntimeRouting, 'unavailable'>};
export interface HardwareBackendInfo {
    backend: HardwareBackend;
    authority: BackendAuthority;
    presetsAvailable: boolean;
    defaultOverrides: Record<string, unknown>;
    parameterRoutes: Record<string, HardwareParameterRoute>;
    runtimeRouting: RuntimeRouting;
    robotName: string;
}
export const DEFAULT_HARDWARE_BACKEND = 'mowgli';
export const normalizeHardwareBackend = (value: unknown): HardwareBackend =>
    value === 'mowgli' || value === 'mavros' || value === 'openmower' ? value : 'unknown';
export const unconfirmedHardwareBackend = (authority: BackendAuthority): HardwareBackendInfo => ({
    backend: 'unknown', authority, presetsAvailable: false, defaultOverrides: {}, parameterRoutes: {}, runtimeRouting: 'unavailable', robotName: '',
});

// Canonical GUI API routes. These are hardware writes, not a general ROS parameter allowlist.
const targets: Record<HardwareBackend, Record<string, {parameter: string; type: 'float64' | 'bool'; restartOnly?: boolean}>> = {
    mowgli: Object.fromEntries(['ticks_per_meter', 'wheel_pid_kp', 'wheel_pid_ki', 'wheel_pid_kd',
        'wheel_pid_integral_limit', 'wheel_pid_pwm_per_mps'].map(key => [key, {parameter: `hardware_bridge.${key}`, type: 'float64'}])),
    mavros: {
        ticks_per_meter: {parameter: 'mavros/esc_wheel_odometry.ticks_per_meter', type: 'float64'},
        wheel_track: {parameter: 'mavros/esc_wheel_odometry.track_width_m', type: 'float64'},
        wheel_pid_pwm_per_mps: {parameter: 'hardware_bridge.manual_control_linear_scale', type: 'float64'},
        mowing_enabled: {parameter: 'hardware_bridge.mowing_enabled', type: 'bool', restartOnly: true},
    },
    openmower: {}, unknown: {},
};
const runtimeState = (value: unknown): value is Exclude<RuntimeRouting, 'unavailable'> =>
    value === 'available' || value === 'pending_image' || value === 'restart_required';
const presetNumericKeys = new Set(MOWER_MODELS.flatMap(model => Object.keys(model.defaults)));
const jsonValue = (value: unknown): boolean => value === null || typeof value === 'string' || typeof value === 'boolean'
    || (typeof value === 'number' && Number.isFinite(value))
    || (Array.isArray(value) && value.every(jsonValue))
    || (isJsonObject(value) && Object.values(value).every(jsonValue));

export function parseHardwareBackendInfo(raw: unknown): HardwareBackendInfo {
    if (!isJsonObject(raw)) return unconfirmedHardwareBackend('malformed');
    const backend = normalizeHardwareBackend(raw.backend);
    if (backend === 'unknown') return unconfirmedHardwareBackend(typeof raw.backend === 'string' ? 'unsupported' : 'malformed');
    if ((raw.default_overrides !== undefined && (!isJsonObject(raw.default_overrides) || !jsonValue(raw.default_overrides)))
        || (raw.robot_name !== undefined && typeof raw.robot_name !== 'string')
        || (raw.runtime_routing !== undefined && !runtimeState(raw.runtime_routing))) return unconfirmedHardwareBackend('malformed');
    if (raw.supported !== undefined && (!Array.isArray(raw.supported)
        || !raw.supported.every(value => normalizeHardwareBackend(value) !== 'unknown')
        || !raw.supported.includes(backend))) return unconfirmedHardwareBackend('malformed');
    if (isJsonObject(raw.default_overrides)) {
        for (const [key, value] of Object.entries(raw.default_overrides)) {
            if (presetNumericKeys.has(key) && (typeof value !== 'number' || !Number.isFinite(value))) return unconfirmedHardwareBackend('malformed');
            const target = Object.prototype.hasOwnProperty.call(targets[backend], key) ? targets[backend][key] : undefined;
            if (target && hardwareParameterValue(value, target.type) === undefined) return unconfirmedHardwareBackend('malformed');
        }
    }
    const routes: Record<string, HardwareParameterRoute> = {};
    if (raw.parameter_routes !== undefined) {
        if (!isJsonObject(raw.parameter_routes)) return unconfirmedHardwareBackend('malformed');
        for (const [key, route] of Object.entries(raw.parameter_routes)) {
            const target = Object.prototype.hasOwnProperty.call(targets[backend], key) ? targets[backend][key] : undefined;
            if (!target || !isJsonObject(route) || typeof route.parameter !== 'string' || !route.parameter.trim() || route.parameter !== target.parameter || !runtimeState(route.runtime)
                || (target.restartOnly && route.runtime === 'available')) return unconfirmedHardwareBackend('malformed');
            routes[key] = {parameter: target.parameter, runtime: route.runtime};
        }
    }
    return {backend, authority: 'confirmed', parameterRoutes: routes,
        presetsAvailable: isJsonObject(raw.default_overrides) && (backend !== 'openmower'
            || (typeof raw.default_overrides.ticks_per_meter === 'number' && Number.isFinite(raw.default_overrides.ticks_per_meter) && raw.default_overrides.ticks_per_meter > 0)),
        defaultOverrides: isJsonObject(raw.default_overrides) ? raw.default_overrides : {},
        runtimeRouting: runtimeState(raw.runtime_routing) && raw.parameter_routes !== undefined ? raw.runtime_routing : 'unavailable',
        robotName: typeof raw.robot_name === 'string' ? raw.robot_name.trim() : ''};
}

export const appliesToBackend = (backends: readonly HardwareBackend[] | undefined, active: HardwareBackend): boolean =>
    !backends || (active !== 'unknown' && backends.includes(active));
export const presetValuesForBackend = (preset: Record<string, number>, overrides: Record<string, unknown>): Record<string, unknown> => {
    const out: Record<string, unknown> = {...preset};
    for (const key of Object.keys(preset)) if (Object.prototype.hasOwnProperty.call(overrides, key)) out[key] = overrides[key];
    return out;
};
export const settingsSectionsForBackend = <T extends {id: string}>(sections: readonly T[], backend: HardwareBackend): T[] =>
    backend === 'unknown' ? sections.filter(section => section.id !== 'drive_motor') : [...sections];

export function hardwareParameterValue(value: unknown, type: 'float64' | 'bool'): number | boolean | undefined {
    if (type === 'bool') return typeof value === 'boolean' ? value : undefined;
    if (typeof value === 'number') return Number.isFinite(value) ? value : undefined;
    if (typeof value !== 'string' || !/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(value.trim())) return undefined;
    const number = Number(value.trim());
    return Number.isFinite(number) ? number : undefined;
}
export function planHardwareParameters(dirtyKeys: ReadonlySet<string>, values: Record<string, unknown>, info: HardwareBackendInfo) {
    const parameters: {name: string; value: number | boolean; type: 'float64' | 'bool'}[] = [];
    const liveKeys = new Set<string>();
    if (info.authority === 'confirmed' && info.backend !== 'unknown' && info.runtimeRouting === 'available') {
        for (const key of dirtyKeys) {
            const route = info.parameterRoutes[key];
            const target = Object.prototype.hasOwnProperty.call(targets[info.backend], key) ? targets[info.backend][key] : undefined;
            if (!route || !target || target.restartOnly || route.runtime !== 'available' || route.parameter !== target.parameter) continue;
            const value = hardwareParameterValue(values[key], target.type);
            if (value === undefined) continue;
            parameters.push({name: target.parameter, value, type: target.type}); liveKeys.add(key);
        }
    }
    const hardwareKeys = new Set(Object.values(targets).flatMap(routes => Object.keys(routes)));
    const deferredKeys = [...dirtyKeys].filter(key => hardwareKeys.has(key) && !liveKeys.has(key));
    const deferredState = deferredKeys.length === 0 ? undefined : info.authority !== 'confirmed'
        ? 'unconfirmed' : info.runtimeRouting !== 'available' ? info.runtimeRouting
        : deferredKeys.some(key => info.parameterRoutes[key]?.runtime === 'pending_image') ? 'pending_image'
        : deferredKeys.some(key => info.parameterRoutes[key]?.runtime === 'restart_required') ? 'restart_required' : 'unavailable';
    return {parameters, liveKeys, deferredState};
}
export const liveHardwareParameters = (keys: ReadonlySet<string>, values: Record<string, unknown>, info: HardwareBackendInfo) =>
    planHardwareParameters(keys, values, info).parameters;
