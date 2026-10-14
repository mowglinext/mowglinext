import type {MavrosInfo} from '../hooks/useMavrosInfo';
import {formatMavrosBoardIdentity} from '../hooks/useMavrosInfo';
export type FirmwareFamily = 'unknown' | 'ardupilot' | 'px4' | 'other';
export interface ParamIdentity {family: FirmwareFamily; vehicle: string; version: string; board: string; source: 'unknown' | 'mavros' | 'file' | 'operator'}
export const unknownParamIdentity = (): ParamIdentity => ({family: 'unknown', vehicle: '', version: '', board: '', source: 'unknown'});
export function identityFromMavros(info?: MavrosInfo): ParamIdentity {
    const v = info?.vehicle;
    if (!v || info?.state.connected === false || ((v.available_info ?? 0) & 1) === 0) return unknownParamIdentity();
    const family = v.autopilot === 3 ? 'ardupilot' : v.autopilot === 12 ? 'px4' : 'unknown';
    const packed = ((v.available_info ?? 0) & 2) !== 0 ? (v.flight_sw_version ?? 0) >>> 0 : 0;
    return {family, vehicle: v.type === 10 ? 'rover' : '',
        version: packed ? `${packed >>> 24}.${(packed >>> 16) & 255}.${(packed >>> 8) & 255}` : '',
        board: formatMavrosBoardIdentity(v) ?? '', source: 'mavros'};
}
export function identityFromParamComments(text: string): ParamIdentity {
    const comments = text.split(/\r\n|\n|\r/).filter(line => /^\s*[#;]/.test(line)).join('\n');
    const versions = (label: string) => [...comments.matchAll(new RegExp(`^\\s*[#;]\\s*(?:Firmware:\\s*)?${label}\\s+v?(\\d+\\.\\d+\\.\\d+)\\b.*$`, 'gim'))].map(match => match[1]);
    const rover = versions('ArduRover'); const px4 = versions('PX4');
    const boards = [...comments.matchAll(/^\s*[#;]\s*Board\s*:\s*([^\r\n]+)$/gim)].map(match => match[1].trim());
    const board = new Set(boards).size === 1 ? boards[0] : '';
    const base = {...unknownParamIdentity(), board, source: board ? 'file' as const : 'unknown' as const};
    if ((rover.length && px4.length) || new Set([...rover, ...px4]).size > 1) return base;
    return rover.length ? {family: 'ardupilot', vehicle: 'rover', version: rover[0], board, source: 'file'}
        : px4.length ? {family: 'px4', vehicle: '', version: px4[0], board, source: 'file'} : base;
}
