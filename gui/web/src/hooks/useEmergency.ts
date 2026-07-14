import {Emergency} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

export const useEmergency = (): Emergency => useTopic<Emergency>("emergency", {}).data;
