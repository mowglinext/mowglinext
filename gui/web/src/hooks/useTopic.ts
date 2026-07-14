import {useEffect, useRef, useState} from "react";
import {getMultiplexedSocket} from "./multiplexedSocket.ts";

export interface UseTopicOptions<T> {
    /**
     * Minimum interval between state updates. Messages arriving faster are
     * coalesced: the latest payload is delivered on a trailing timer so the
     * final value is never dropped. 0/undefined = every message re-renders.
     */
    throttleMs?: number;
    /** Track Date.now() of the last delivered message in `lastMessageAt`. */
    withTimestamp?: boolean;
    /**
     * Map the raw multiplex payload to the hook's value. Return `undefined`
     * to ignore the message (keeps the previous value). Read fresh on every
     * message, so closures over refs are safe.
     */
    select?: (raw: unknown) => T | undefined;
}

export interface TopicState<T> {
    data: T;
    /** Only populated when `withTimestamp` is set; otherwise stays null. */
    lastMessageAt: number | null;
}

/**
 * useTopic — subscribe to one multiplexed backend topic
 * (`/api/mowglinext/multiplex`, see {@link getMultiplexedSocket}) and expose
 * the latest decoded message as React state. Unsubscribes on unmount.
 *
 * This is the shared engine behind the per-topic hooks (useGPS, usePose,
 * useStatus, ...), which are thin typed wrappers around it.
 */
export const useTopic = <T>(
    topic: string,
    initial: T,
    opts?: UseTopicOptions<T>,
): TopicState<T> => {
    const [state, setState] = useState<TopicState<T>>({data: initial, lastMessageAt: null});
    // Always read the latest options without re-subscribing when the caller
    // passes a fresh object/closure each render.
    const optsRef = useRef(opts);
    optsRef.current = opts;

    useEffect(() => {
        let lastEmitAt = 0;
        let pendingTimer: number | null = null;
        let latest: { data: T; at: number } | null = null;

        const emit = () => {
            if (latest === null) return;
            const {data, at} = latest;
            latest = null;
            lastEmitAt = Date.now();
            setState({
                data,
                lastMessageAt: optsRef.current?.withTimestamp ? at : null,
            });
        };

        const unsubscribe = getMultiplexedSocket().subscribe(topic, (raw) => {
            const select = optsRef.current?.select;
            const next = select ? select(raw) : (raw as T);
            if (next === undefined) return;
            latest = {data: next, at: Date.now()};

            const throttleMs = optsRef.current?.throttleMs ?? 0;
            if (throttleMs <= 0) {
                emit();
                return;
            }
            const elapsed = Date.now() - lastEmitAt;
            if (elapsed >= throttleMs) {
                emit();
                return;
            }
            if (pendingTimer === null) {
                pendingTimer = window.setTimeout(() => {
                    pendingTimer = null;
                    emit();
                }, throttleMs - elapsed);
            }
        });

        return () => {
            if (pendingTimer !== null) window.clearTimeout(pendingTimer);
            unsubscribe();
        };
    }, [topic]);

    return state;
};
