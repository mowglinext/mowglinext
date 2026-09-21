import {describe, expect, it} from "vitest";
import {detectLogSeverity} from "./logSeverity.ts";

describe("detectLogSeverity", () => {
    it.each([
        "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
        "[ERROR] [node-X]: process has died [pid 42, exit code -2, cmd '/opt/ros/node']",
    ])("classifies a normal ROS 2 SIGINT shutdown as INFO: %s", (line) => {
        expect(detectLogSeverity(line)).toBe('INFO');
    });

    it.each([
        "[ERROR] [node-X]: process has died [pid 42, exit code 1, cmd '/opt/ros/node']",
        "[ERROR] [node-X]: process has died [pid 42, exit code -6, cmd '/opt/ros/node']",
        "[ERROR] [node-X]: process has died [pid 42, exit code -20, cmd '/opt/ros/node']",
    ])("keeps a non-SIGINT process death as ERROR: %s", (line) => {
        expect(detectLogSeverity(line)).toBe('ERROR');
    });

    it("keeps unrelated launch warnings as WARN", () => {
        expect(detectLogSeverity("[WARNING] [launch]: required process exited")).toBe('WARN');
    });
});
