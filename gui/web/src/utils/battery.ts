/** Default battery voltage thresholds — must match mower_config.schema.json */
export const BATTERY_DEFAULTS = {
    // YardForce500 SLA packs top out around 28.0 V at the dock; the old
    // 28.5 V default capped the displayed percent at 88.9 % even when full.
    fullVoltage: 28.0,
    emptyVoltage: 24.0,
    criticalVoltage: 23.0,
} as const;

/**
 * Compute battery percentage from voltage and settings.
 * Prefers highLevelStatus.battery_percent when available (> 0).
 * Falls back to linear interpolation between empty and full voltage.
 * Returns a rounded integer 0–100.
 */
export type BatteryLevel = "ok" | "warn" | "danger";

/** Thresholds shared by every battery badge/statistic on the GUI. */
export const BATTERY_WARN_PERCENT = 50;
export const BATTERY_DANGER_PERCENT = 20;

/**
 * Map a battery percentage to a severity level.
 * > 50 % → ok, 21–50 % → warn, ≤ 20 % → danger.
 */
export const getBatteryLevel = (percent: number): BatteryLevel => {
    if (percent <= BATTERY_DANGER_PERCENT) {
        return "danger";
    }
    if (percent <= BATTERY_WARN_PERCENT) {
        return "warn";
    }
    return "ok";
};

export const computeBatteryPercent = (
    batteryPercent: number | null | undefined,
    voltage: number | undefined,
    settings: Record<string, any>,
): number => {
    if (batteryPercent != null && batteryPercent > 0) {
        return Math.round(batteryPercent);
    }
    if (voltage) {
        const full = parseFloat(settings["battery_full_voltage"] ?? BATTERY_DEFAULTS.fullVoltage);
        const empty = parseFloat(settings["battery_empty_voltage"] ?? BATTERY_DEFAULTS.emptyVoltage);
        const pct = ((voltage - empty) / (full - empty)) * 100;
        return Math.round(Math.max(0, Math.min(100, pct)));
    }
    return 0;
};
