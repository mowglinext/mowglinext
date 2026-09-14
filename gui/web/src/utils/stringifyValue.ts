/**
 * Render an untyped value as a display string.
 *
 * Settings and ROS parameter payloads arrive from JSON as `unknown`, and plain
 * `String(value)` on an object yields the literal "[object Object]" — which is
 * exactly what reached the GNSS device field and the parameters table. Objects
 * and arrays are JSON-encoded instead so the operator sees the real payload,
 * and nullish values collapse to the empty string rather than "null".
 */
export function stringifyValue(value: unknown): string {
    if (value === null || value === undefined) {
        return ""
    }
    if (typeof value === "string") {
        return value
    }
    if (typeof value === "number" || typeof value === "boolean" || typeof value === "bigint") {
        return String(value)
    }
    if (typeof value === "symbol") {
        return value.toString()
    }
    if (typeof value === "function") {
        // Never expected from JSON; render something identifiable rather than
        // dumping the source text into the UI.
        return `[function ${value.name || "anonymous"}]`
    }
    return JSON.stringify(value)
}
