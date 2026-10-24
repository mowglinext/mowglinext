/** Shape guard only: callers must validate each field before treating it as domain data. */
export function isJsonObject(value: unknown): value is Record<string, unknown> {
    if (typeof value !== 'object' || value === null || Array.isArray(value)) return false;
    const prototype: unknown = Object.getPrototypeOf(value);
    return prototype === Object.prototype || prototype === null;
}

/** /config/keys endpoints store strings. Reject the whole record rather than coerce values. */
export function decodeStringRecord(value: unknown): Record<string, string> {
    if (!isJsonObject(value)) throw new Error('Expected a configuration object');
    const entries: Array<[string, string]> = [];
    for (const [key, item] of Object.entries(value)) {
        if (typeof item !== 'string') throw new Error(`Configuration value for ${key} must be a string`);
        entries.push([key, item]);
    }
    // Preserve own JSON keys such as __proto__ without invoking prototype setters.
    return Object.fromEntries(entries);
}
