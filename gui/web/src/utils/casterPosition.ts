/** Compatibility for configurations saved before caster position became explicit.
 * Freeze the old position before applying another edit, so changing chassis
 * length/radius no longer silently moves the caster axle. No physical defaults
 * are duplicated here: input values come from the settings API/schema. */
export function resolveCasterPosition(values: Record<string, unknown>): number | undefined {
    const read = (key: string) => {
        const value = values[key];
        return (typeof value === "number" || (typeof value === "string" && value.trim() !== "")) && Number.isFinite(Number(value)) ? Number(value) : undefined;
    };
    const x = read("caster_x_offset");
    if (x !== -1) return x;
    const centre = read("chassis_center_x"), length = read("chassis_length"), radius = read("caster_radius");
    if (centre === undefined || length === undefined || radius === undefined) return undefined;
    return Number((centre + length / 2 - radius).toFixed(6));
}

export function materializeLegacyCasterPosition<T extends Record<string, unknown>>(values: T): T {
    if (Number(values.caster_x_offset) !== -1) return values;
    const x = resolveCasterPosition(values);
    return x === undefined ? values : {...values, caster_x_offset: x};
}
