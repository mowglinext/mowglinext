import {DockingSensor} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

export const useDockingSensor = (): DockingSensor => useTopic<DockingSensor>("dockingSensor", {}).data;
