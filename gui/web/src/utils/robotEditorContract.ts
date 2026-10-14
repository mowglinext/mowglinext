import {isJsonObject} from './json';
export type PersistedDockPose = {dock_pose_x: number; dock_pose_y: number; dock_pose_yaw: number};
export const finiteNumber = (raw: unknown): raw is number => typeof raw === 'number' && Number.isFinite(raw);
export function geometryNumber(raw: unknown): number | undefined {
    if (finiteNumber(raw)) return raw;
    if (typeof raw === 'string' && /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(raw.trim())) {
        const value = Number(raw.trim());
        if (Number.isFinite(value)) return value;
    }
    return undefined;
}
export const roundTo = (v: number, decimals: number): number => {const f = Math.pow(10, decimals); return Math.round(v * f) / f;};
export const radToDeg = (r: number): number => (r * 180) / Math.PI;
export const degToRad = (d: number): number => (d * Math.PI) / 180;
export const yawRadToCompassBearing = (yawRad: number): number => {const yawDeg = radToDeg(yawRad); return ((90 - yawDeg) % 360 + 360) % 360;};
export const compassBearingToYawRad = (bearing: number): number => degToRad(90 - bearing);
export function decodeStoredDockPose(raw: unknown): {x: number; y: number; yaw: number} {
    if (!isJsonObject(raw) || !isJsonObject(raw.dock)) throw new Error('Dock readback was not confirmed');
    const dock = raw.dock;
    // Missing validity metadata cannot establish explicit YAML value presence.
    const valid = dock.pose_values_valid === true;
    if (!valid || (dock.error !== undefined && dock.error !== '') || !finiteNumber(dock.dock_pose_x)
        || !finiteNumber(dock.dock_pose_y) || !finiteNumber(dock.dock_pose_yaw_rad)) throw new Error('Dock readback was not confirmed');
    return {x: dock.dock_pose_x, y: dock.dock_pose_y, yaw: dock.dock_pose_yaw_rad};
}
/** Current Go returns {} (OkResponse with empty omitempty field). Legacy explicit
 * ok acknowledgements are accepted; explicit failures and unrelated shapes aren't. */
export function decodeDockWriteAcknowledgement(raw: unknown): void {
    if (!isJsonObject(raw) || Object.keys(raw).some(key => key !== 'ok') || ('ok' in raw && raw.ok !== 'ok' && raw.ok !== '')) throw new Error('Dock write was not confirmed');
    if ('error' in raw || raw.success === false) throw new Error('Dock write was not confirmed');
}
