import {describe, expect, test} from "vitest";
import {detectNav2Recovery} from "./nav2Recovery";
import {BTNodeState, BT_NODE_STALE_MS} from "../hooks/useBTLog";

const NOW = 1_000_000;

function states(entries: Array<[string, string, number?]>): Map<string, BTNodeState> {
    return new Map(entries.map(([name, status, updatedAt]) => [
        name,
        {status, updatedAt: updatedAt ?? NOW},
    ]));
}

describe("detectNav2Recovery", () => {
    test("reports the running recovery behavior by name", () => {
        const result = detectNav2Recovery(
            states([["RecoveryFallback", "RUNNING"], ["Spin", "RUNNING"]]),
            NOW,
        );
        expect(result).toEqual({active: true, nodeName: "Spin"});
    });

    test("falls back to the wrapper when no concrete behavior is running", () => {
        const result = detectNav2Recovery(states([["RecoveryFallback", "RUNNING"]]), NOW);
        expect(result).toEqual({active: true, nodeName: null});
    });

    test("inactive when recovery nodes are not RUNNING", () => {
        const result = detectNav2Recovery(
            states([["Spin", "SUCCESS"], ["FollowPath", "RUNNING"]]),
            NOW,
        );
        expect(result).toEqual({active: false, nodeName: null});
    });

    test("ignores stale RUNNING states", () => {
        const result = detectNav2Recovery(
            states([["Spin", "RUNNING", NOW - BT_NODE_STALE_MS - 1]]),
            NOW,
        );
        expect(result).toEqual({active: false, nodeName: null});
    });

    test("empty map is inactive", () => {
        expect(detectNav2Recovery(new Map(), NOW)).toEqual({active: false, nodeName: null});
    });
});
