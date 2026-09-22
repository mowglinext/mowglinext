import type {HighLevelStatus} from "../../types/ros.ts";

/**
 * SCAN_PAUSED is a live BT status overlay, not a distinct autonomous state.
 * Keep its narrow identity check in one place so Dashboard never presents a
 * stale or unrelated sub-state as a stopped mower.
 */
export function isCoverageScanPaused(status: Pick<HighLevelStatus,
  "state" | "state_name" | "sub_state_name">): boolean {
  return status.state === 2 && status.state_name === "MOWING" &&
    status.sub_state_name === "SCAN_PAUSED";
}
