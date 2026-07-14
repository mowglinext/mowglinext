// Shared pure helpers for settings value handling.

/**
 * Parse the many boolean spellings that reach the GUI from the DB config,
 * the shell settings endpoint, and the YAML settings endpoint:
 * true / false (real booleans), "true" / "false" / "True" / "False"
 * (case-insensitive), and "1" / "0" (string or number).
 * Returns undefined when the value is not a recognizable boolean, so the
 * caller can keep the raw value instead of dropping the key.
 */
export const parseBoolish = (value: unknown): boolean | undefined => {
    if (typeof value === "boolean") return value;
    if (typeof value === "number") {
        if (value === 1) return true;
        if (value === 0) return false;
        return undefined;
    }
    if (typeof value === "string") {
        const normalized = value.trim().toLowerCase();
        if (normalized === "true" || normalized === "1") return true;
        if (normalized === "false" || normalized === "0") return false;
    }
    return undefined;
};

/**
 * Loose value equality used by the "reset to default" affordance: numbers
 * compare numerically (tolerating int/float JSON churn and numeric strings,
 * so 5 == 5.0 == "5"), everything else compares structurally. null/undefined
 * never match a real value (Number(null) === 0 would otherwise make
 * valuesMatch(null, 0) true).
 */
export const valuesMatch = (a: unknown, b: unknown): boolean => {
    if (a == null || b == null) return a === b;
    if (typeof a === "number" && typeof b === "number") return a === b;
    // Number vs numeric-string / int-vs-float from JSON round-trips.
    const an = Number(a);
    const bn = Number(b);
    if (!Number.isNaN(an) && !Number.isNaN(bn) &&
        (typeof a !== "string" || a.trim() !== "") &&
        (typeof b !== "string" || b.trim() !== "")) {
        return an === bn;
    }
    return JSON.stringify(a) === JSON.stringify(b);
};
