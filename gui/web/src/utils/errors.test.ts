import {describe, expect, it} from "vitest";
import {apiErrorMessage, errorMessage, isAbortError, requestErrorMessage} from "./errors.ts";

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


describe('request error boundaries', () => {
    it.each([null, undefined, 42, 'failure', [], {error: 42}, {error: {error: false}}, {message: {text: 'failure'}}])('does not treat malformed text as a backend reason: %j', payload => {
        expect(requestErrorMessage(payload)).toBeUndefined();
        expect(apiErrorMessage(payload)).toBeUndefined();
    });

    it.each([
        [new Error('ordinary'), 'ordinary'],
        [{error: 'direct'}, 'direct'],
        [{error: {error: 'server'}, message: 'generic'}, 'server'],
        [{message: 'message'}, 'message'],
        [{error: {unexpected: true}, message: 'fallback'}, 'fallback'],
        [{error: ''}, ''],
    ])('reads only the supported error-envelope text', (payload, expected) => {
        expect(requestErrorMessage(payload)).toBe(expected);
    });
});
