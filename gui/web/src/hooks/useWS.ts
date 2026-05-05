import reactUseWebSocketModule from "react-use-websocket";
import {useRef, useState} from "react";
import {
    getMultiplexedSocket,
    isMultiplexableSubscribeUri,
    topicFromSubscribeUri,
} from "./multiplexedSocket.ts";

// Vite 8 CJS interop may wrap the default export differently at runtime
const useWebSocket = (reactUseWebSocketModule as unknown as { default: typeof reactUseWebSocketModule }).default ?? reactUseWebSocketModule;

/**
 * useWS — start/stop a stream of payloads from the GUI backend.
 *
 * Subscribe URIs (`/api/mowglinext/subscribe/<topic>`) are multiplexed
 * over a single shared WebSocket via {@link getMultiplexedSocket}.
 * Other URIs (e.g. `/api/mowglinext/publish/joy`) keep their dedicated
 * connection because they need bidirectional traffic that the
 * multiplex protocol does not handle yet.
 *
 * The signature is unchanged from before #177 so every existing hook
 * keeps working without modification.
 */
export const useWS = <T>(
    onError: (e: Error) => void,
    onInfo: (msg: string) => void,
    onData: (data: T, first?: boolean) => void,
) => {
    // Refs to always invoke the latest callbacks, avoiding stale closures.
    const onDataRef = useRef(onData);
    onDataRef.current = onData;
    const onErrorRef = useRef(onError);
    onErrorRef.current = onError;
    const onInfoRef = useRef(onInfo);
    onInfoRef.current = onInfo;

    // Active multiplex unsubscribe (set when start() targets a subscribe URI).
    const muxUnsubscribeRef = useRef<(() => void) | null>(null);

    // Publish-side socket (only used for non-subscribe URIs).
    const [pubUri, setPubUri] = useState<string | null>(null);
    const pubFirstRef = useRef(true);
    const ws = useWebSocket(pubUri, {
        share: true,
        shouldReconnect: () => true,
        reconnectAttempts: Infinity,
        reconnectInterval: (attempt: number) => Math.min(1000 * Math.pow(2, attempt), 30000),
        onOpen: () => {
            onInfoRef.current("Stream connected");
        },
        onError: () => {
            onErrorRef.current(new Error("Stream error"));
        },
        onClose: () => {
            onErrorRef.current(new Error("Stream closed"));
        },
        onMessage: (e: MessageEvent) => {
            const isFirst = pubFirstRef.current;
            if (isFirst) pubFirstRef.current = false;
            onDataRef.current(atob(e.data) as T, isFirst);
        },
    });

    const teardown = () => {
        if (muxUnsubscribeRef.current) {
            muxUnsubscribeRef.current();
            muxUnsubscribeRef.current = null;
        }
        if (pubUri !== null) {
            setPubUri(null);
            pubFirstRef.current = false;
        }
    };

    const start = (uri: string) => {
        teardown();

        if (isMultiplexableSubscribeUri(uri)) {
            const topic = topicFromSubscribeUri(uri);
            let firstReported = false;
            muxUnsubscribeRef.current = getMultiplexedSocket().subscribe(
                topic,
                (data, isFirst) => {
                    if (isFirst && !firstReported) {
                        firstReported = true;
                        onInfoRef.current("Stream connected");
                    }
                    onDataRef.current(data as T, isFirst);
                },
            );
            return;
        }

        // Publish path: open a dedicated socket as before.
        pubFirstRef.current = true;
        const protocol = window.location.protocol === "https:" ? "wss" : "ws";
        if (import.meta.env.DEV) {
            setPubUri(`${protocol}://localhost:4006${uri}`);
        } else {
            setPubUri(`${protocol}://${window.location.host}${uri}`);
        }
    };

    const stop = () => {
        teardown();
    };

    return {start, stop, sendJsonMessage: ws.sendJsonMessage};
};
