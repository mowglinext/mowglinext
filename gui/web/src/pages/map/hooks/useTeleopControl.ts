import reactUseWebSocketModule, {ReadyState} from "react-use-websocket";
import {useCallback, useEffect, useRef, useState} from "react";
import {wsBase} from "../../../utils/apiHost.ts";
import type {TwistStamped} from "../../../types/ros.ts";

const useWebSocket = (reactUseWebSocketModule as unknown as { default: typeof reactUseWebSocketModule }).default ?? reactUseWebSocketModule;
const HEARTBEAT_INTERVAL_MS = 1000;

export type TeleopControlState = "disconnected" | "available" | "owner" | "busy" | "stopped";

interface TeleopStateMessage {
    type: "teleop_state";
    state: Exclude<TeleopControlState, "disconnected">;
    revision: number;
}

function isTeleopStateMessage(value: unknown): value is TeleopStateMessage {
    if (typeof value !== "object" || value === null) return false;
    const candidate = value as Record<string, unknown>;
    return candidate.type === "teleop_state"
        && typeof candidate.revision === "number"
        && (candidate.state === "available"
            || candidate.state === "owner"
            || candidate.state === "busy"
            || candidate.state === "stopped");
}

export function useTeleopControl() {
    const [uri, setUri] = useState<string | null>(null);
    const [controlState, setControlState] = useState<TeleopControlState>("disconnected");
    const pendingAcquireRef = useRef(false);
    // Keep the active operator's intent across transient WebSocket drops.
    // The backend zeros motion and releases the lease on disconnect, so a
    // reconnect may reacquire only if the session is still available.
    const wantsControlRef = useRef(false);
    const revisionRef = useRef(-1);

    const socket = useWebSocket(uri, {
        share: false,
        shouldReconnect: () => true,
        reconnectAttempts: Infinity,
        reconnectInterval: (attempt: number) => Math.min(1000 * Math.pow(2, attempt), 30000),
        onClose: () => {
            revisionRef.current = -1;
            pendingAcquireRef.current = wantsControlRef.current;
            setControlState("disconnected");
        },
        onError: () => {
            pendingAcquireRef.current = wantsControlRef.current;
            setControlState("disconnected");
        },
        onMessage: (event: MessageEvent) => {
            let message: unknown;
            try {
                if (typeof event.data !== "string") return;
                message = JSON.parse(event.data) as unknown;
            } catch {
                return;
            }
            if (!isTeleopStateMessage(message) || message.revision < revisionRef.current) return;
            revisionRef.current = message.revision;
            if (message.state === "stopped" || message.state === "busy") {
                // STOP is authoritative; a conflicting owner is also not
                // silently displaced when this socket reconnects.
                pendingAcquireRef.current = false;
                wantsControlRef.current = false;
            } else if (message.state === "owner") {
                pendingAcquireRef.current = false;
                wantsControlRef.current = true;
            } else if (message.state === "available" && wantsControlRef.current) {
                // Reacquire after lease expiry or a transient disconnect.
                // handleJoyMove clears the held stick when ownership drops,
                // so this never resumes stale non-zero motion.
                pendingAcquireRef.current = true;
            }
            setControlState(message.state);
        },
    });

    // Acquisition waits until the backend confirms the session is available.
    // A transient disconnect preserves intent; STOP and another active owner
    // clear it, preventing stale or conflicting automatic takeovers.
    useEffect(() => {
        if (socket.readyState !== ReadyState.OPEN || controlState !== "available" || !pendingAcquireRef.current) return;
        pendingAcquireRef.current = false;
        socket.sendJsonMessage({type: "acquire"});
    }, [controlState, socket]);

    useEffect(() => {
        if (controlState !== "owner") return;
        const timer = setInterval(() => {
            socket.sendJsonMessage({type: "heartbeat"});
        }, HEARTBEAT_INTERVAL_MS);
        return () => clearInterval(timer);
    }, [controlState, socket]);

    const start = useCallback((path: string) => {
        setUri(`${wsBase()}${path}`);
    }, []);

    const stop = useCallback(() => {
        pendingAcquireRef.current = false;
        wantsControlRef.current = false;
        if (controlState === "owner") socket.sendJsonMessage({type: "release"});
        setUri(null);
        setControlState("disconnected");
        revisionRef.current = -1;
    }, [controlState, socket]);

    const requestControl = useCallback(() => {
        if (controlState === "busy") return;
        wantsControlRef.current = true;
        pendingAcquireRef.current = true;
        // Setting the same state does not re-render, so send immediately when
        // the socket is already known to be available.
        if (socket.readyState === ReadyState.OPEN && controlState === "available") {
            pendingAcquireRef.current = false;
            socket.sendJsonMessage({type: "acquire"});
        }
    }, [controlState, socket]);

    const releaseControl = useCallback(() => {
        pendingAcquireRef.current = false;
        wantsControlRef.current = false;
        socket.sendJsonMessage({type: "release"});
    }, [socket]);

    const globalStop = useCallback(() => {
        pendingAcquireRef.current = false;
        wantsControlRef.current = false;
        socket.sendJsonMessage({type: "stop"});
    }, [socket]);

    const sendCommand = useCallback((command: TwistStamped) => {
        if (controlState !== "owner") return;
        socket.sendJsonMessage({type: "command", command});
    }, [controlState, socket]);

    return {
        start,
        stop,
        requestControl,
        releaseControl,
        globalStop,
        sendCommand,
        controlState,
        isOwner: controlState === "owner",
    };
}
