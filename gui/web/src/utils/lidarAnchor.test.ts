import {describe, expect, it} from "vitest";
import {deriveLidarAnchor} from "./lidarAnchor.ts";

describe("deriveLidarAnchor", () => {
    it("returns null when the node does not publish lidar_anchor_state", () => {
        expect(deriveLidarAnchor({total_nodes: "42"})).toBeNull();
    });

    it("maps the state code, shadow flag and verdict to their label keys", () => {
        const summary = deriveLidarAnchor({
            lidar_anchor_state: "3",
            lidar_anchor_shadow: "1",
            lidar_anchor_verdict: "rejected_dead_reckoning",
        });
        expect(summary).not.toBeNull();
        expect(summary?.stateKey).toBe("anchoring");
        expect(summary?.isShadow).toBe(true);
        expect(summary?.verdictKey).toBe("rejectedDeadReckoning");
    });

    it("derives accepted = updates − rejects and scales the hit ratio to percent", () => {
        const summary = deriveLidarAnchor({
            lidar_anchor_state: "2",
            lidar_anchor_shadow: "0",
            lidar_anchor_updates: "10",
            lidar_anchor_rej_score: "2",
            lidar_anchor_rej_spread: "1",
            lidar_anchor_rej_dr: "3",
            lidar_anchor_hit_ratio: "0.734",
            lidar_map_occupied_cells: "1234",
            lidar_anchor_factors: "4",
            lidar_anchor_reseeds: "1",
            lidar_anchor_odom_rebases: "2",
            lidar_anchor_sigma_m: "0.08",
        });
        expect(summary?.stateKey).toBe("mapping");
        expect(summary?.isShadow).toBe(false);
        expect(summary?.accepted).toBe(4);
        expect(summary?.rejected).toBe(6);
        expect(summary?.hitRatioPct).toBe(73);
        expect(summary?.mapCells).toBe(1234);
        expect(summary?.factors).toBe(4);
        expect(summary?.reseeds).toBe(1);
        expect(summary?.odomRebases).toBe(2);
        expect(summary?.sigmaM).toBeCloseTo(0.08);
    });

    it("tolerates missing counters and unknown codes without throwing", () => {
        const summary = deriveLidarAnchor({lidar_anchor_state: "7", lidar_anchor_verdict: "???"});
        expect(summary?.stateKey).toBe("off");
        expect(summary?.verdictKey).toBeNull();
        expect(summary?.accepted).toBeNull();
        expect(summary?.rejected).toBe(0);
        expect(summary?.hitRatioPct).toBeNull();
    });

    it("never reports a negative accepted count", () => {
        const summary = deriveLidarAnchor({
            lidar_anchor_state: "1",
            lidar_anchor_updates: "1",
            lidar_anchor_rej_score: "5",
        });
        expect(summary?.accepted).toBe(0);
    });
});
