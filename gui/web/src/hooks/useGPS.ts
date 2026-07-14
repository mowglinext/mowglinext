import {AbsolutePose} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

export const useGPS = (): AbsolutePose => useTopic<AbsolutePose>("gps", {}).data;
