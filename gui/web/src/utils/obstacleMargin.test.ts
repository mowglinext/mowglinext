import { describe, expect, it } from "vitest";
import { besideBodyClearanceM, chassisHalfWidthM, planningObstacleMarginFloorM } from "./obstacleMargin.ts";

describe("planningObstacleMarginFloorM", () => {
    it("mirrors robot_config_util.planning_obstacle_margin_floor for the shipped chassis", () => {
        // half_width = 0.45/2 + 0.05 = 0.275; transit_demand = 0.275 + 0.08*sqrt(2)
        // ~= 0.38814 -> ceil to mm = 0.389, and it dominates the default 0.05 m
        // clearance margin's controller_demand (0.375).
        expect(planningObstacleMarginFloorM(0.45, 0.05)).toBeCloseTo(0.389, 3);
    });

    it("falls back to the shipped defaults when values are omitted", () => {
        expect(planningObstacleMarginFloorM()).toBeCloseTo(0.389, 3);
    });

    it("lets a large clearance margin push the controller demand above the transit demand", () => {
        // half_width 0.275 + 0.5 (clamped max) + 0.05 tracking slack = 0.825.
        expect(planningObstacleMarginFloorM(0.45, 0.5)).toBeCloseTo(0.825, 3);
    });

    it("clamps a negative clearance margin request to zero, not below", () => {
        expect(planningObstacleMarginFloorM(0.45, -1)).toBeCloseTo(
            planningObstacleMarginFloorM(0.45, 0),
            3,
        );
    });

    it("tracks a wider chassis (transit demand scales with half-width)", () => {
        // half_width = 0.60/2 + 0.05 = 0.35; transit_demand = 0.35 + 0.11314 -> 0.464.
        expect(planningObstacleMarginFloorM(0.6, 0.05)).toBeCloseTo(0.464, 3);
    });
});

describe("chassisHalfWidthM / besideBodyClearanceM", () => {
    it("computes half-width as width/2 + the fixed footprint margin", () => {
        expect(chassisHalfWidthM(0.45)).toBeCloseTo(0.275, 3);
    });

    it("subtracts half-width from a centreline margin to get the beside-body gap", () => {
        expect(besideBodyClearanceM(0.389, 0.45)).toBeCloseTo(0.114, 3);
    });
});
