/// Memory of obstacle outlines before the recorded-obstacle correction shrank
/// them (see handleSaveAreaModal in MapPage). The shrunk outline is what
/// map_server stores; the original is only needed to undo the shrink when the
/// operator turns the obstacle back into an area. It is kept in map-frame
/// metres (x east, y north) so it survives a reload, a GUI offset change and a
/// map backup/restore — the editor's GeoJSON coordinates would not.
export interface XY {
    x: number;
    y: number;
}

export interface ObstacleOriginal {
    shrunk: XY[];
    original: XY[];
}

/// Two outlines are "the same obstacle" when they have the same vertices to
/// within 1 cm (the server stores float32 and de-duplicates the closing point).
export const MATCH_TOLERANCE_M = 0.01;

/// Enough for a garden; a runaway list must not grow the config store forever.
export const MAX_ORIGINALS = 200;

const withoutClosingPoint = (ring: XY[]): XY[] => {
    if (ring.length > 1) {
        const first = ring[0];
        const last = ring[ring.length - 1];
        if (Math.hypot(first.x - last.x, first.y - last.y) < 1e-6) {
            return ring.slice(0, -1);
        }
    }
    return ring;
};

export const sameOutline = (a: XY[], b: XY[], tolerance = MATCH_TOLERANCE_M): boolean => {
    const ra = withoutClosingPoint(a);
    const rb = withoutClosingPoint(b);
    return ra.length === rb.length && ra.every((p, i) => Math.hypot(p.x - rb[i].x, p.y - rb[i].y) <= tolerance);
};

export const findOriginal = (records: ObstacleOriginal[], shrunk: XY[]): XY[] | undefined =>
    records.find((r) => sameOutline(r.shrunk, shrunk))?.original;

/// Add (or replace) the record for this shrunk outline.
export const rememberOriginal = (records: ObstacleOriginal[], record: ObstacleOriginal): ObstacleOriginal[] =>
    [...records.filter((r) => !sameOutline(r.shrunk, record.shrunk)), record].slice(-MAX_ORIGINALS);

export const forgetOriginal = (records: ObstacleOriginal[], shrunk: XY[]): ObstacleOriginal[] =>
    records.filter((r) => !sameOutline(r.shrunk, shrunk));

/// Only the records that still describe an obstacle on the map (the rest are
/// stale: the obstacle was deleted or edited since).
export const originalsForObstacles = (records: ObstacleOriginal[], obstacles: XY[][]): ObstacleOriginal[] =>
    records.filter((r) => obstacles.some((o) => sameOutline(r.shrunk, o)));

const isXY = (p: unknown): p is XY =>
    typeof p === "object" && p !== null
    && Number.isFinite((p as XY).x) && Number.isFinite((p as XY).y);

const ringFrom = (value: unknown): XY[] | null =>
    Array.isArray(value) && value.length >= 3 && value.every(isXY)
        ? value.map((p) => ({x: p.x, y: p.y}))
        : null;

/// Parse a list of records from untrusted JSON (the config store or a backup
/// file). Returns the valid records, or null when `value` is not a list at all.
export const parseOriginals = (value: unknown): ObstacleOriginal[] | null => {
    if (!Array.isArray(value)) {
        return null;
    }
    const out: ObstacleOriginal[] = [];
    for (const entry of value) {
        const shrunk = ringFrom((entry as ObstacleOriginal | null)?.shrunk);
        const original = ringFrom((entry as ObstacleOriginal | null)?.original);
        if (shrunk && original) {
            out.push({shrunk, original});
        }
    }
    return out.slice(-MAX_ORIGINALS);
};

export const parseStoredOriginals = (text: string | undefined): ObstacleOriginal[] => {
    if (!text) {
        return [];
    }
    try {
        return parseOriginals(JSON.parse(text)) ?? [];
    } catch {
        return [];
    }
};
