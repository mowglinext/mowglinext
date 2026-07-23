/**
 * Formatting helpers for raw telemetry floats.
 */

/** Values below this magnitude are sensor noise around zero. */
export const TELEMETRY_ZERO_EPSILON = 1e-3;

/**
 * Clamp near-zero float noise (e.g. 6.9e-20 V from an unplugged charger) to
 * an exact 0 so antd's Statistic renders "0.00" instead of raw scientific
 * notation, which its precision formatter cannot handle.
 */
export function clampTinyToZero(
    value: number | null | undefined,
    epsilon: number = TELEMETRY_ZERO_EPSILON,
): number | null | undefined {
    if (value === null || value === undefined) return value;
    return Math.abs(value) < epsilon ? 0 : value;
}
