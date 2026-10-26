import {isJsonObject} from './json';
export type SettingsValue = string | number | boolean | null | SettingsValue[] | {[key: string]: SettingsValue};
export type SettingsConfig = Record<string, SettingsValue>;
function jsonValue(raw: unknown, ancestors = new Set<object>()): raw is SettingsValue {
    if (raw === null || typeof raw === 'string' || typeof raw === 'boolean') return true;
    if (typeof raw === 'number') return Number.isFinite(raw);
    if (!Array.isArray(raw) && !isJsonObject(raw)) return false;
    if (ancestors.has(raw)) return false;
    const next = new Set(ancestors).add(raw);
    const descriptors = Object.getOwnPropertyDescriptors(raw);
    if (Reflect.ownKeys(raw).some(key => typeof key !== 'string' || (key !== 'length' || !Array.isArray(raw))
        && (!descriptors[key].enumerable || !('value' in descriptors[key])))) return false;
    if (Array.isArray(raw) && (Object.keys(raw).length !== raw.length || !Object.keys(raw).every((key, index) => key === String(index)))) return false;
    return Object.values(raw).every(value => jsonValue(value, next));
}
export function decodeSettings(raw: unknown): SettingsConfig {
    if (!isJsonObject(raw) || !jsonValue(raw)) throw new Error('Invalid settings payload');
    return raw;
}
/** Flatten plain nesting only. Arrays/null/empty objects remain leaves. Reject
 * collisions rather than allowing traversal order to change the setting. */
export function flattenConfig(raw: unknown): SettingsConfig {
    const source = decodeSettings(raw);
    const entries = new Map<string, SettingsValue>();
    const walk = (object: SettingsConfig, prefix: string) => {
        for (const [key, value] of Object.entries(object)) {
            if (!key) throw new Error('Empty settings key');
            const path = prefix ? `${prefix}.${key}` : key;
            if (isJsonObject(value) && Object.keys(value).length > 0) walk(decodeSettings(value), path);
            else {
                if (entries.has(path)) throw new Error(`Conflicting settings key: ${path}`);
                entries.set(path, value);
            }
        }
    };
    walk(source, '');
    return Object.fromEntries(entries);
}
export const legacyEndpointMissing = (error: unknown): boolean => typeof error === 'object' && error !== null
    && 'status' in error && (error.status === 404 || error.status === 405);

/** Legacy datum parsing accepts a finite number or decimal string, never a
 * boolean/array/object converted through implicit string coercion. */
export function settingsCoordinate(raw: unknown, fallback = 0): number {
    if (raw === null || raw === undefined) return fallback;
    if (typeof raw !== 'number' && typeof raw !== 'string') return NaN;
    const value = typeof raw === 'number' ? raw : parseFloat(raw);
    return Number.isFinite(value) ? value : NaN;
}
export const isIntegerSetting = (raw: unknown): raw is number => typeof raw === 'number' && Number.isSafeInteger(raw);
