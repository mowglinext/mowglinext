import {describe, expect, it} from 'vitest';
import {geometryNumber, decodeStoredDockPose, decodeDockWriteAcknowledgement, yawRadToCompassBearing, compassBearingToYawRad} from './robotEditorContract';

describe('robot geometry contract', () => {
    it.each([0, '0', '-0.25', ' 1e-2 '])('accepts finite geometric value %s', raw => expect(geometryNumber(raw)).toBe(Number(raw)));
    it.each([null, undefined, '', ' ', false, [], {}, NaN, Infinity, '1m', '0x12'])('keeps invalid geometric value %j unavailable', raw => expect(geometryNumber(raw)).toBeUndefined());
    it.each([[0, 90], [Math.PI / 2, 0], [-Math.PI / 2, 180], [Math.PI, 270]])('preserves ROS yaw %s to compass %s', (yaw, bearing) => {
        expect(yawRadToCompassBearing(yaw)).toBeCloseTo(bearing);
        expect(Math.cos(compassBearingToYawRad(bearing))).toBeCloseTo(Math.cos(yaw));
        expect(Math.sin(compassBearingToYawRad(bearing))).toBeCloseTo(Math.sin(yaw));
    });
});
describe('canonical dock readback', () => {
    const dock = {present: false, pose_values_valid: true, dock_pose_x: 0, dock_pose_y: 0, dock_pose_yaw_rad: 0};
    it('accepts explicit zeros independently of present', () => expect(decodeStoredDockPose({dock})).toEqual({x: 0, y: 0, yaw: 0}));
    it.each([{pose_values_valid: false}, {pose_values_valid: undefined}, {dock_pose_x: undefined}, {dock_pose_y: null}, {dock_pose_yaw_rad: NaN}, {dock_pose_x: '0'}, {error: 'Read failed'}])('rejects malformed or unconfirmed stored values %j', patch => {
        expect(() => decodeStoredDockPose({dock: {...dock, ...patch}})).toThrow();
    });
    it.each([{}, {ok: ''}, {ok: 'ok'}])('accepts Go write acknowledgement %j', raw => expect(() => decodeDockWriteAcknowledgement(raw)).not.toThrow());
    it.each([null, [], {error: 'Rejected'}, {success: false}, {ok: false}, {unrelated: true}])('rejects unexpected write response %j', raw => expect(() => decodeDockWriteAcknowledgement(raw)).toThrow());
});
