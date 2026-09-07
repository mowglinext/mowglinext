/**
 * Pure derivation of the LiDAR map-anchor tile group from the flattened
 * `/fusion_graph/diagnostics` key/value map (`useFusionGraphDiagnostics`).
 *
 * The anchor is fusion_graph_node's optional scan-to-map particle filter
 * (`use_lidar_map_anchor`): under RTK Fixed it ray-casts scans into a
 * georeferenced occupancy grid, and once Fixed goes stale it localizes new
 * scans against that map and adds XY anchor factors. Its counters ride the
 * same DiagnosticStatus as the graph stats, all stringly-typed.
 */

export type LidarAnchorStateKey = "off" | "waitingForMap" | "mapping" | "anchoring";

export type LidarAnchorVerdictKey =
    | "accepted"
    | "rejectedScore"
    | "rejectedSpread"
    | "rejectedDeadReckoning";

export interface LidarAnchorSummary {
    /** Raw `lidar_anchor_state` (0 disabled, 1 waiting for map, 2 mapping, 3 anchoring). */
    state: number;
    stateKey: LidarAnchorStateKey;
    /** `lidar_anchor_shadow` == "1": the filter runs and scores under RTK Fixed but applies nothing. */
    isShadow: boolean;
    mapCells: number | null;
    updates: number | null;
    seeds: number | null;
    skipped: number | null;
    factors: number | null;
    /** `lidar_anchor_hit_ratio` (0..1) scaled to a whole percent. */
    hitRatioPct: number | null;
    sigmaM: number | null;
    verdictKey: LidarAnchorVerdictKey | null;
    rejScore: number;
    rejSpread: number;
    rejDr: number;
    /** updates − rejScore − rejSpread − rejDr, floored at 0; null until `lidar_anchor_updates` arrives. */
    accepted: number | null;
    rejected: number;
    reseeds: number | null;
    odomRebases: number | null;
}

const STATE_KEYS: Record<number, LidarAnchorStateKey> = {
    0: "off",
    1: "waitingForMap",
    2: "mapping",
    3: "anchoring",
};

const VERDICT_KEYS: Record<string, LidarAnchorVerdictKey> = {
    accepted: "accepted",
    rejected_score: "rejectedScore",
    rejected_spread: "rejectedSpread",
    rejected_dead_reckoning: "rejectedDeadReckoning",
};

function numberOrNull(values: Record<string, string>, key: string): number | null {
    const raw = values[key];
    if (raw === undefined) return null;
    const n = Number(raw);
    return Number.isFinite(n) ? n : null;
}

/**
 * Returns null when `lidar_anchor_state` is absent — an older
 * fusion_graph_node without the anchor — so the caller renders nothing.
 */
export function deriveLidarAnchor(values: Record<string, string>): LidarAnchorSummary | null {
    const state = numberOrNull(values, "lidar_anchor_state");
    if (state === null) return null;

    const updates = numberOrNull(values, "lidar_anchor_updates");
    const rejScore = numberOrNull(values, "lidar_anchor_rej_score") ?? 0;
    const rejSpread = numberOrNull(values, "lidar_anchor_rej_spread") ?? 0;
    const rejDr = numberOrNull(values, "lidar_anchor_rej_dr") ?? 0;
    const rejected = rejScore + rejSpread + rejDr;
    const hitRatio = numberOrNull(values, "lidar_anchor_hit_ratio");
    const verdictRaw = values["lidar_anchor_verdict"];

    return {
        state,
        stateKey: STATE_KEYS[state] ?? "off",
        isShadow: values["lidar_anchor_shadow"] === "1",
        mapCells: numberOrNull(values, "lidar_map_occupied_cells"),
        updates,
        seeds: numberOrNull(values, "lidar_anchor_seeds"),
        skipped: numberOrNull(values, "lidar_anchor_skipped"),
        factors: numberOrNull(values, "lidar_anchor_factors"),
        hitRatioPct: hitRatio === null ? null : Math.round(Math.min(1, Math.max(0, hitRatio)) * 100),
        sigmaM: numberOrNull(values, "lidar_anchor_sigma_m"),
        verdictKey: verdictRaw === undefined ? null : (VERDICT_KEYS[verdictRaw] ?? null),
        rejScore,
        rejSpread,
        rejDr,
        accepted: updates === null ? null : Math.max(0, updates - rejected),
        rejected,
        reseeds: numberOrNull(values, "lidar_anchor_reseeds"),
        odomRebases: numberOrNull(values, "lidar_anchor_odom_rebases"),
    };
}
