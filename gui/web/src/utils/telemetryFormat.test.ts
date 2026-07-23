import {describe, expect, test} from "vitest";
import {clampTinyToZero} from "./telemetryFormat";

describe("clampTinyToZero", () => {
    test("clamps scientific-notation noise to exact zero", () => {
        expect(clampTinyToZero(6.922290682953276e-20)).toBe(0);
        expect(clampTinyToZero(-1.69e-11)).toBe(0);
    });

    test("keeps real readings untouched", () => {
        expect(clampTinyToZero(27.53)).toBe(27.53);
        expect(clampTinyToZero(-0.45)).toBe(-0.45);
    });

    test("passes through null and undefined", () => {
        expect(clampTinyToZero(null)).toBeNull();
        expect(clampTinyToZero(undefined)).toBeUndefined();
    });

    test("respects a custom epsilon", () => {
        expect(clampTinyToZero(0.05, 0.1)).toBe(0);
        expect(clampTinyToZero(0.15, 0.1)).toBe(0.15);
    });
});
