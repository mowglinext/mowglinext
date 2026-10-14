/**
 * Adapts an async handler for a callback typed `=> void` (an `onClick`, an `onChange`, an `onNext`
 * prop). Passing the async function directly hands the caller a Promise it ignores, which is what
 * `@typescript-eslint/no-misused-promises` flags. The adapter drops the Promise explicitly; the
 * arguments are passed through unchanged, and a rejection is as unhandled as it always was.
 */
export const voidify =
    <A extends unknown[]>(handler: (...args: A) => Promise<unknown>) =>
    (...args: A): void => {
        void handler(...args);
    };
