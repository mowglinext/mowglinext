/** Last-resort diagnostics for background work whose normal errors are handled locally. */
export function reportAsyncError(error: unknown): void {
    console.error('Asynchronous GUI operation failed', error);
}

/** Observe detached work without deferring its start or changing its concurrency. */
export function observeAsync(result: unknown, onError: (error: unknown) => void = reportAsyncError): void {
    Promise.resolve(result).catch(onError).catch(reportAsyncError);
}

/** Adapt a callback for a synchronous event API. Do not use for promise-aware confirmations. */
export function asyncEvent<Args extends unknown[], Result>(
    callback: (...args: Args) => Result,
    onError: (error: unknown) => void,
): (...args: Args) => void {
    return (...args) => {
        try {
            observeAsync(callback(...args), onError);
        } catch (error) {
            try {
                onError(error);
            } catch (reportingError) {
                reportAsyncError(reportingError);
            }
        }
    };
}
