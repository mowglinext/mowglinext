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
 * Returns a rounded integer 0–100, or NaN when no valid estimate exists.
 */
export type BatteryLevel = "ok" | "warn" | "danger" | "unknown";

/** Thresholds shared by every battery badge/statistic on the GUI. */
export const BATTERY_WARN_PERCENT = 50;
export const BATTERY_DANGER_PERCENT = 20;

/**
 * Map a battery percentage to a severity level.
 * > 50 % → ok, 21–50 % → warn, ≤ 20 % → danger.
 */
export const getBatteryLevel = (percent: number): BatteryLevel => {
    if (!Number.isFinite(percent) || percent < 0 || percent > 100) return "unknown";
    if (percent <= BATTERY_DANGER_PERCENT) {
        return "danger";
    }
    if (percent <= BATTERY_WARN_PERCENT) {
        return "warn";
    }
    return "ok";
};

/** A displayed percentage requires finite positive pack voltage; freshness is separate. */
export const hasBatteryReading = (voltage: unknown): voltage is number =>
    typeof voltage === "number" && Number.isFinite(voltage) && voltage > 0;

export const computeBatteryPercent = (
    batteryPercent: unknown,
    voltage: unknown,
    settings: Record<string, unknown>,
): number => {
    if (typeof batteryPercent === "number" && Number.isFinite(batteryPercent) && batteryPercent > 0 && batteryPercent <= 100) {
        return Math.round(batteryPercent);
    }
    if (hasBatteryReading(voltage)) {
        const full = batteryThreshold(settings["battery_full_voltage"], BATTERY_DEFAULTS.fullVoltage);
        const empty = batteryThreshold(settings["battery_empty_voltage"], BATTERY_DEFAULTS.emptyVoltage);
        if (!Number.isFinite(full) || !Number.isFinite(empty) || empty < 0 || full <= empty) return NaN;
        const pct = ((voltage - empty) / (full - empty)) * 100;
        return Math.round(Math.max(0, Math.min(100, pct)));
    }
    return NaN;
};

// Settings may be numeric YAML values or legacy DB strings. No partial parsing.
function batteryThreshold(raw: unknown, fallback: number): number {
    if (raw === undefined || raw === null) return fallback;
    if (typeof raw === 'number') return Number.isFinite(raw) ? raw : NaN;
    if (typeof raw !== 'string' || !/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?$/i.test(raw.trim())) return NaN;
    const parsed = Number(raw);
    return Number.isFinite(parsed) ? parsed : NaN;
}
