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
        return String((thrown as { message: unknown }).message);
    }
    return String(thrown);
};

/** The `error` text the GUI backend puts in an API error body (`{ error: "..." }`). */
export const apiErrorMessage = (apiError: unknown): string | undefined =>
    (apiError as { error?: string } | null | undefined)?.error;

/** True for the rejection of an aborted fetch / AbortController (`name === "AbortError"`). */
export const isAbortError = (thrown: unknown): boolean =>
    typeof thrown === "object" && thrown !== null && (thrown as { name?: unknown }).name === "AbortError";
