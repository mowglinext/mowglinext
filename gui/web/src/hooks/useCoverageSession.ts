import type {CoverageSession} from "../types/ros.ts";
import {useTopic} from "./useTopic.ts";

/** Latched BT session provenance; unlike a charging bit, this says a mow is still resumable. */
export const useCoverageSession = () =>
  useTopic<CoverageSession>("coverageSession", {session_active: false}).data;
