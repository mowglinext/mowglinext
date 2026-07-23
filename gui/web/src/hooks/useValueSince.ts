import {useRef} from "react";

/**
 * Timestamp (ms) of when `value` last changed, or null while it is nullish.
 *
 * Used for "in state X for N min" displays: the caller re-renders on its own
 * clock tick, so this only needs to remember when the observed value flipped.
 */
export function useValueSince(value: unknown): number | null {
    const ref = useRef<{value: unknown; since: number} | null>(null);
    if (value === null || value === undefined || value === false) {
        ref.current = null;
    } else if (ref.current === null || ref.current.value !== value) {
        ref.current = {value, since: Date.now()};
    }
    return ref.current?.since ?? null;
}
