const RAD_TO_DEG = 180 / Math.PI;

/** Yaw (Z-axis rotation) of a quaternion, in degrees. */
export function yawFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * RAD_TO_DEG;
}

/** Roll (X-axis rotation) of a quaternion, in degrees. */
export function rollFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    return Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * RAD_TO_DEG;
}

/** Pitch (Y-axis rotation) of a quaternion, in degrees. Clamped at ±90°. */
export function pitchFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    const sinp = 2 * (w * y - z * x);
    return Math.abs(sinp) >= 1 ? Math.sign(sinp) * 90 : Math.asin(sinp) * RAD_TO_DEG;
}

/** Wrap an angle in degrees into (-180, 180]. */
export function wrapDeg180(deg: number): number {
    return ((deg + 180) % 360 + 360) % 360 - 180;
}
