import {Imu} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

export const useImu = (): Imu => useTopic<Imu>("imu", {}).data;
