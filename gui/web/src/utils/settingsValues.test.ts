import { describe, expect, test } from "vitest";
import { parseBoolish, valuesMatch } from "./settingsValues.ts";

describe("parseBoolish", () => {
    test("parses real booleans", () => {
        expect(parseBoolish(true)).toBe(true);
        expect(parseBoolish(false)).toBe(false);
    });

    test("parses case-insensitive true/false strings", () => {
        expect(parseBoolish("true")).toBe(true);
        expect(parseBoolish("True")).toBe(true);
        expect(parseBoolish("FALSE")).toBe(false);
        expect(parseBoolish("false")).toBe(false);
    });

    test("parses 1/0 as string and number", () => {
        expect(parseBoolish("1")).toBe(true);
        expect(parseBoolish("0")).toBe(false);
        expect(parseBoolish(1)).toBe(true);
        expect(parseBoolish(0)).toBe(false);
    });

    test("returns undefined for unknown representations so callers keep the raw value", () => {
        expect(parseBoolish("yes")).toBeUndefined();
        expect(parseBoolish("")).toBeUndefined();
        expect(parseBoolish(2)).toBeUndefined();
        expect(parseBoolish(null)).toBeUndefined();
        expect(parseBoolish(undefined)).toBeUndefined();
        expect(parseBoolish({})).toBeUndefined();
    });
});

describe("valuesMatch", () => {
    test("null never matches a real value (Number(null) === 0 regression)", () => {
        expect(valuesMatch(null, 0)).toBe(false);
        expect(valuesMatch(0, null)).toBe(false);
        expect(valuesMatch(undefined, 0)).toBe(false);
        expect(valuesMatch(null, "")).toBe(false);
    });

    test("null matches null but not undefined", () => {
        expect(valuesMatch(null, null)).toBe(true);
        expect(valuesMatch(undefined, undefined)).toBe(true);
        expect(valuesMatch(null, undefined)).toBe(false);
    });

    test("tolerates int/float and numeric-string churn", () => {
        expect(valuesMatch(5, 5.0)).toBe(true);
        expect(valuesMatch("5", 5)).toBe(true);
        expect(valuesMatch("5.5", 5.5)).toBe(true);
        expect(valuesMatch(5, 6)).toBe(false);
    });

    test("compares non-numeric values structurally", () => {
        expect(valuesMatch("abc", "abc")).toBe(true);
        expect(valuesMatch("abc", "abd")).toBe(false);
        expect(valuesMatch(true, true)).toBe(true);
        expect(valuesMatch(true, false)).toBe(false);
        expect(valuesMatch("", "")).toBe(true);
        expect(valuesMatch("", 0)).toBe(false);
    });
});
