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

    // What api/Api.ts throws on a non-2xx answer: the Response, its parsed body on `error`.
    const rejected = (status: number, statusText: string, error: unknown): Response =>
        Object.assign(new Response(null, {status, statusText}), {error});

    it("reads the backend reason of a rejected API call instead of [object Response]", () => {
        const response = rejected(503, "Service Unavailable", {error: "unsupported HARDWARE_BACKEND configuration"});
        expect(errorMessage(response)).toBe("unsupported HARDWARE_BACKEND configuration");
    });

    it("falls back to the HTTP status when the rejected call has no usable body", () => {
        expect(errorMessage(rejected(502, "Bad Gateway", null))).toBe("HTTP 502 Bad Gateway");
        expect(errorMessage(rejected(500, "", new SyntaxError("Unexpected token <")))).toBe("HTTP 500");
        expect(errorMessage(rejected(500, "Internal Server Error", {error: ""}))).toBe("HTTP 500 Internal Server Error");
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
