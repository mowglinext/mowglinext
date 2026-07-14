import {Power} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

export const usePower = (): Power => useTopic<Power>("power", {}).data;
