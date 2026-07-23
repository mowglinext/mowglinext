import {BTNodeState, isBTNodeStale} from "../hooks/useBTLog.ts";

/**
 * Detect an active Nav2 recovery from the /behavior_tree_log node states.
 *
 * The log is published by Nav2's bt_navigator (NOT the Mowgli high-level BT).
 * A RUNNING recovery behavior means Nav2 gave up on normal path following and
 * is spinning/backing up — when this persists, the robot is effectively stuck.
 */

/** Concrete recovery behaviors — the informative ones to surface by name. */
const RECOVERY_BEHAVIOR_NODES = ["Spin", "BackUp", "DriveOnHeading", "Wait"] as const;

/** Structural wrappers that only reveal the recovery subtree is ticking. */
const RECOVERY_WRAPPER_NODES = ["RecoveryFallback", "RecoveryActions"] as const;

export interface Nav2RecoveryInfo {
    active: boolean;
    /** Behavior name (e.g. "Spin"), or null when only a wrapper is running. */
    nodeName: string | null;
}

export function detectNav2Recovery(
    nodeStates: Map<string, BTNodeState>,
    nowMs: number,
): Nav2RecoveryInfo {
    const isRunning = (name: string): boolean => {
        const state = nodeStates.get(name);
        return state !== undefined && state.status === "RUNNING" && !isBTNodeStale(state, nowMs);
    };

    for (const name of RECOVERY_BEHAVIOR_NODES) {
        if (isRunning(name)) return {active: true, nodeName: name};
    }
    for (const name of RECOVERY_WRAPPER_NODES) {
        if (isRunning(name)) return {active: true, nodeName: null};
    }
    return {active: false, nodeName: null};
}
