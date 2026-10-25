/**
 * Machine codes of HighLevelStatus.stop_reason, published by behavior_tree_node
 * (ros2/src/mowgli_behavior/include/mowgli_behavior/stop_reason.hpp). Add a code
 * there and in the `stopReason` block of every locale file.
 */
export const KNOWN_STOP_REASONS = [
    "AREA_START_BLOCKED",
    "AREA_NO_PROGRESS",
    "DOCK_NAV_FAILED",
    "UNDOCK_FAILED",
    "CHARGER_FAILED",
] as const;

export type KnownStopReason = (typeof KNOWN_STOP_REASONS)[number];

export function isKnownStopReason(code: string): code is KnownStopReason {
    return (KNOWN_STOP_REASONS as readonly string[]).includes(code);
}
