import {AbsolutePose} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

export const usePose = (): AbsolutePose => useTopic<AbsolutePose>("pose", {}).data;
