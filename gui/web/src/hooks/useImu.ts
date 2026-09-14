import {Imu} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

/**
 * Live IMU sample. Pass `enabled=false` while nothing on screen shows it:
 * /imu/data runs at ~90 Hz and foxglove_bridge serialises every message for
 * as long as the upstream subscription exists.
 */
export const useImu = (enabled = true): Imu => useTopic<Imu>("imu", {}, {enabled}).data;
