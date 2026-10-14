/**
 * Typed helpers for the two places an error arrives as `unknown`: a `catch` clause and the `error`
 * field of an API response. They replace `catch (e: any)` / `(res.error as any).error`.
 */

/** The message of whatever was thrown: an Error, or an object carrying a `message`. */
export const errorMessage = (thrown: unknown): string => {
    if (thrown instanceof Error) {
        return thrown.message;
    }
    if (typeof thrown === "object" && thrown !== null && "message" in thrown) {
        return String(thrown.message);
    }
    return String(thrown);
};

/** The `error` text the GUI backend puts in an API error body (`{ error: "..." }`). */
export const apiErrorMessage = (apiError: unknown): string | undefined =>
    typeof apiError === "object" && apiError !== null && "error" in apiError && typeof apiError.error === "string"
        ? apiError.error : undefined;

/** True for the rejection of an aborted fetch / AbortController (`name === "AbortError"`). */
export const isAbortError = (thrown: unknown): boolean =>
    typeof thrown === "object" && thrown !== null && "name" in thrown && thrown.name === "AbortError";


/** Generated HttpResponse rejection, direct API error body, or an ordinary Error.
 * Read only textual error fields; malformed payloads cannot masquerade as a reason.
 */
export function requestErrorMessage(thrown: unknown): string | undefined {
    if (thrown instanceof Error) return thrown.message;
    if (typeof thrown !== 'object' || thrown === null) return undefined;
    const nested = 'error' in thrown ? apiErrorMessage(thrown.error) : undefined;
    const direct = apiErrorMessage(thrown);
    if (nested !== undefined) return nested;
    if (direct !== undefined) return direct;
    if ('message' in thrown && typeof thrown.message === 'string') return thrown.message;
    return undefined;
}
