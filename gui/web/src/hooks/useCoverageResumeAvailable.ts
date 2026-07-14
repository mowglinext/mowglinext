import {useTopic} from "./useTopic.ts";

// std_msgs/Bool arrives as {data: boolean}.
const selectBool = (raw: unknown): boolean =>
    !!(raw as { data?: boolean } | null | undefined)?.data;

// Latched std_msgs/Bool from behavior_tree_node: true when a prior mowing
// session was interrupted and can be resumed. The GUI uses it to offer a
// "Start fresh" choice next to Start, instead of silently resuming mid-path
// (the "starts at 2nd/3rd line" report).
export const useCoverageResumeAvailable = (): boolean =>
    useTopic<boolean>("coverageResumeAvailable", false, {select: selectBool}).data;
