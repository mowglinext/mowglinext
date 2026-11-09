import {isJsonObject} from './json';

export type GnssActionName = "plan" | "apply" | "factory-reset-apply" | "restart";
export type GnssProducerAction = 'plan' | 'apply' | 'factory_reset_apply' | 'restart';

// URL segments and Go response identifiers are separate contracts.
const producerActionByRoute: Record<GnssActionName, GnssProducerAction> = {
    plan: 'plan', apply: 'apply', 'factory-reset-apply': 'factory_reset_apply', restart: 'restart',
};

export type GnssCommandExecution = {
    tool: string;
    command: string[];
    exit_code: number;
    stdout?: string;
    stderr?: string;
    success: boolean;
};

export type GnssActionResponse = {
    success: boolean;
    partial_failure?: boolean;
    action?: GnssProducerAction;
    message?: string;
    warnings?: string[];
    receiver_family?: string;
    profile?: string;
    signal_profile?: string;
    profile_rate_hz?: string;
    serial_device?: string;
    execution_baud?: string;
    detected_baud?: string;
    target_baud?: string;
    active_verified_baud?: string;
    manual_baud_required?: boolean;
    runtime_baud?: string;
    config_baud?: string;
    runtime_baud_differs_from_config?: boolean;
    runtime_baud_updated?: boolean;
    gps_container?: string;
    gps_image?: string;
    gps_container_was_running?: boolean;
    stop_attempted?: boolean;
    restart_attempted?: boolean;
    restart_succeeded?: boolean;
    restart_error?: string;
    executions?: GnssCommandExecution[];
};

export type GnssRuntimeValue = {
    configured_value?: string;
    fallback_env_value?: string;
    active_value?: string;
    // Known values are "config" | "env" | "default"; the backend may add
    // more, so this stays an open string rather than a closed union that
    // TypeScript would collapse to `string` anyway.
    source?: string;
};

export type GnssSerialDeviceOption = {
    path: string;
    resolved_path?: string;
    display_label: string;
    exists: boolean;
};

export type GnssRuntimeConfig = {
    receiver_family?: GnssRuntimeValue;
    serial_device?: GnssRuntimeValue;
    serial_baud?: GnssRuntimeValue;
    serial_devices?: GnssSerialDeviceOption[];
    configured_serial_device_exists?: boolean;
    active_serial_device_exists?: boolean;
    configured_serial_resolved_path?: string;
    active_serial_resolved_path?: string;
};


const text = (value: unknown): value is string => typeof value === 'string';
const boolean = (value: unknown): value is boolean => typeof value === 'boolean';
const integer = (value: unknown): value is number => typeof value === 'number' && Number.isSafeInteger(value);
const texts = (value: unknown): value is string[] => Array.isArray(value) && value.every(text);
const optional = (raw: Record<string, unknown>, keys: string[], check: (value: unknown) => boolean): boolean =>
    keys.every(key => !Object.prototype.hasOwnProperty.call(raw, key) || check(raw[key]));
// Reported baud is a decimal string; don't limit detected baud to the GUI's
// four recovery options, or manufacture one from numeric/null values.
const baud = (value: unknown): boolean => text(value) && (value === '' || (/^\d+$/.test(value) && Number.isSafeInteger(Number(value)) && Number(value) > 0));
function execution(raw: unknown): raw is GnssCommandExecution {
    return isJsonObject(raw) && text(raw.tool) && texts(raw.command) && integer(raw.exit_code) && boolean(raw.success)
        && raw.success === (raw.exit_code === 0) && optional(raw, ['stdout', 'stderr'], text);
}
function actionResponse(raw: unknown): raw is GnssActionResponse {
    return isJsonObject(raw) && Object.prototype.hasOwnProperty.call(raw, 'success') && boolean(raw.success)
        && optional(raw, ['action'], value => Object.values(producerActionByRoute).some(action => action === value))
        && optional(raw, ['message', 'receiver_family', 'profile', 'signal_profile', 'serial_device', 'gps_container', 'gps_image', 'restart_error'], text)
        && optional(raw, ['profile_rate_hz'], value => text(value) && (value === '' || (/^\d+(?:\.\d+)?$/.test(value) && Number.isFinite(Number(value)) && Number(value) > 0)))
        && optional(raw, ['detected_baud', 'target_baud', 'active_verified_baud', 'runtime_baud', 'config_baud'], baud)
        && optional(raw, ['execution_baud'], value => value === 'auto' || baud(value))
        && optional(raw, ['partial_failure', 'manual_baud_required', 'runtime_baud_differs_from_config', 'runtime_baud_updated',
            'gps_container_was_running', 'stop_attempted', 'restart_attempted', 'restart_succeeded'], boolean)
        && optional(raw, ['warnings'], texts)
        && optional(raw, ['executions'], value => Array.isArray(value) && value.every(execution));
}
export function decodeGnssActionResponse(raw: unknown, requestedAction: GnssActionName): GnssActionResponse {
    if (!actionResponse(raw) || (raw.action !== undefined && raw.action !== producerActionByRoute[requestedAction])
        || (raw.restart_succeeded === true && (raw.restart_attempted === false || Boolean(raw.restart_error)))) throw new Error('Invalid GNSS action response');
    return raw;
}
export function gnssActionOutcome(response: GnssActionResponse): 'success' | 'partial' | 'failed' | 'unconfirmed' {
    if (response.partial_failure === true) return 'partial';
    if (response.success === false || response.manual_baud_required === true || Boolean(response.restart_error)) return 'failed';
    if (response.restart_attempted === true && response.restart_succeeded !== true) {
        return response.restart_succeeded === false || Boolean(response.restart_error) ? 'failed' : 'unconfirmed';
    }
    return 'success';
}
function runtimeValue(raw: unknown): raw is GnssRuntimeValue {
    return isJsonObject(raw) && optional(raw, ['configured_value', 'fallback_env_value', 'active_value', 'source'], text);
}
function device(raw: unknown): raw is GnssSerialDeviceOption {
    return isJsonObject(raw) && text(raw.path) && text(raw.display_label) && boolean(raw.exists) && optional(raw, ['resolved_path'], text);
}
function runtimeConfig(raw: unknown): raw is GnssRuntimeConfig {
    return isJsonObject(raw) && ['receiver_family', 'serial_device', 'serial_baud'].some(key => Object.prototype.hasOwnProperty.call(raw, key))
        && optional(raw, ['receiver_family', 'serial_device', 'serial_baud'], runtimeValue)
        && optional(raw, ['configured_serial_device_exists', 'active_serial_device_exists'], boolean)
        && optional(raw, ['configured_serial_resolved_path', 'active_serial_resolved_path'], text)
        && optional(raw, ['serial_devices'], value => Array.isArray(value) && value.every(device));
}
export function decodeGnssRuntimeConfig(raw: unknown): GnssRuntimeConfig {
    if (!runtimeConfig(raw)) throw new Error('Invalid GNSS runtime config');
    return raw;
}
