import {describe, expect, test} from "vitest";
import {nodeFor} from "./BTStateGraph";

/**
 * Every state_name published by the mowgli_behavior BT XML files
 * (grep 'state_name=' ros2/src/mowgli_behavior — keep in sync when adding
 * states). The graph must highlight SOME node for each of them; a null
 * mapping renders as "nothing active", which reads as a broken view.
 */
const PUBLISHED_STATE_NAMES = [
    "IDLE",
    "IDLE_DOCKED",
    "PREFLIGHT_CHECK",
    "UNDOCKING",
    "RESUMING_UNDOCKING",
    "UNDOCK_FAILED",
    "CALIBRATING_HEADING",
    "TRANSIT",
    "PLANNING",
    "MOWING",
    "MOWING_COMPLETE",
    "OBSTACLE_BACKOFF",
    "DYNAMIC_OBSTACLE_CLEARED",
    "AREA_UNREACHABLE",
    "BOUNDARY_RECOVERY",
    "BOUNDARY_EMERGENCY_STOP",
    "EMERGENCY",
    "RETURNING_HOME",
    "NAV_TO_DOCK_FAILED",
    "COVERAGE_FAILED_DOCKING",
    "CHARGING",
    "CHARGER_FAILED",
    "RAIN_DETECTED_DOCKING",
    "RAIN_WAITING",
    "RAIN_TIMEOUT",
    "RESUMING_AFTER_RAIN",
    "LOW_BATTERY_DOCKING",
    "CRITICAL_BATTERY_DOCKING",
    "CRITICAL_BATTERY_CHARGING",
    "CRITICAL_BATTERY_NAV_FAILED",
    "MANUAL_MOWING",
    "RECORDING",
    "RECORDING_COMPLETE",
];

describe("BTStateGraph nodeFor", () => {
    test.each(PUBLISHED_STATE_NAMES)("maps published state %s to a node", (state) => {
        expect(nodeFor(state)).not.toBeNull();
    });

    test("failure states collapse onto the generic FAILURE node", () => {
        expect(nodeFor("NAV_TO_DOCK_FAILED")).toBe("FAILURE");
        expect(nodeFor("CHARGER_FAILED")).toBe("FAILURE");
        expect(nodeFor("AREA_UNREACHABLE")).toBe("FAILURE");
    });

    test("obstacle states map to the obstacle node", () => {
        expect(nodeFor("OBSTACLE_BACKOFF")).toBe("OBSTACLE_BACKOFF");
        expect(nodeFor("DYNAMIC_OBSTACLE_CLEARED")).toBe("OBSTACLE_BACKOFF");
    });

    test("undefined and unknown states map to null", () => {
        expect(nodeFor(undefined)).toBeNull();
        expect(nodeFor("SOME_FUTURE_STATE")).toBeNull();
    });
});
