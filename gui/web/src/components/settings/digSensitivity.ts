export const DIG_SENSITIVITY_LEVELS = ["off", "low", "medium", "high"] as const;
export type DigSensitivity = (typeof DIG_SENSITIVITY_LEVELS)[number];

/**
 * The yaml may hold a bare `off`, which YAML 1.1 parses as boolean false — the
 * launch side reads that as "off" too (robot_config_util.resolve_dig_sensitivity).
 * Anything unknown shows as the shipped default.
 */
export const normalizeDigSensitivity = (raw: unknown): DigSensitivity => {
    if (raw === false) return "off";
    if (typeof raw !== "string") return "medium";
    const level = raw.trim().toLowerCase();
    return (DIG_SENSITIVITY_LEVELS as readonly string[]).includes(level)
        ? (level as DigSensitivity)
        : "medium";
};
