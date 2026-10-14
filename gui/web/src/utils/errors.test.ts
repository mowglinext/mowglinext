import {describe, expect, it} from "vitest";
import {apiErrorMessage, errorMessage, isAbortError} from "./errors.ts";

describe("errorMessage", () => {
    it("reads an Error", () => {
        expect(errorMessage(new Error("boom"))).toBe("boom");
    });

    it("reads an object that carries a message", () => {
        expect(errorMessage({message: "from the api"})).toBe("from the api");
    });

    it("falls back to a string for anything else", () => {
        expect(errorMessage("plain")).toBe("plain");
        expect(errorMessage(undefined)).toBe("undefined");
    });
});

describe("apiErrorMessage", () => {
    it("reads the error text of an API error body", () => {
        expect(apiErrorMessage({error: "not found"})).toBe("not found");
    });

    it("is undefined without a body or an error field", () => {
        expect(apiErrorMessage(undefined)).toBeUndefined();
        expect(apiErrorMessage(null)).toBeUndefined();
        expect(apiErrorMessage({})).toBeUndefined();
    });
});

describe("isAbortError", () => {
    it("recognises an aborted request", () => {
        expect(isAbortError(new DOMException("aborted", "AbortError"))).toBe(true);
        expect(isAbortError({name: "AbortError"})).toBe(true);
    });

    it("is false for anything else", () => {
        expect(isAbortError(new Error("boom"))).toBe(false);
        expect(isAbortError(undefined)).toBe(false);
        expect(isAbortError("AbortError")).toBe(false);
    });
});
