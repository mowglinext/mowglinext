import {Status} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

export const useStatus = (): Status => useTopic<Status>("status", {}).data;
