import {describe, expect, it} from "vitest";
import {createLogSeverityClassifier, SHUTDOWN_CONTEXT_WINDOW_MS} from "./logSeverity.ts";

describe("LogSeverityClassifier", () => {
    it("classifies a launch SIGINT message as INFO", () => {
        const classifier = createLogSeverityClassifier();

        expect(classifier.detect("[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)"))
            .toBe('INFO');
    });

    it("classifies a subsequent SIGINT process death as INFO", () => {
        const classifier = createLogSeverityClassifier();
        classifier.detect("[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)");

        expect(classifier.detect(
            "[ERROR] [node-X]: process has died [pid 42, exit code -2, cmd '/opt/ros/node']",
        )).toBe('INFO');
    });

    it("keeps an isolated SIGINT process death as ERROR", () => {
        const classifier = createLogSeverityClassifier();

        expect(classifier.detect(
            "[ERROR] [node-X]: process has died [pid 42, exit code -2, cmd '/opt/ros/node']",
        )).toBe('ERROR');
    });

    it("does not retain launch SIGINT context after reset", () => {
        const classifier = createLogSeverityClassifier();
        classifier.detect("[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)");
        classifier.reset();

        expect(classifier.detect(
            "[ERROR] [node-X]: process has died [pid 42, exit code -2, cmd '/opt/ros/node']",
        )).toBe('ERROR');
    });

    it("expires launch SIGINT context after the bounded shutdown window", () => {
        let nowMs = 1_000;
        const classifier = createLogSeverityClassifier(() => nowMs);
        classifier.detect("[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)");
        nowMs += SHUTDOWN_CONTEXT_WINDOW_MS + 1;

        expect(classifier.detect(
            "[ERROR] [node-X]: process has died [pid 42, exit code -2, cmd '/opt/ros/node']",
        )).toBe('ERROR');
    });

    it.each([
        "[ERROR] [node-X]: process has died [pid 42, exit code 1, cmd '/opt/ros/node']",
        "[ERROR] [node-X]: process has died [pid 42, exit code -6, cmd '/opt/ros/node']",
        "[ERROR] [node-X]: process has died [pid 42, exit code -20, cmd '/opt/ros/node']",
    ])("keeps a non-SIGINT process death as ERROR: %s", (line) => {
        expect(createLogSeverityClassifier().detect(line)).toBe('ERROR');
    });

    it("keeps unrelated launch warnings as WARN", () => {
        expect(createLogSeverityClassifier().detect("[WARNING] [launch]: required process exited"))
            .toBe('WARN');
    });
});
