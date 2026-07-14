import {WheelTick} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

export const useWheelTicks = (): WheelTick => useTopic<WheelTick>("ticks", {}).data;
