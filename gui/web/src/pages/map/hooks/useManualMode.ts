import {useCallback, useEffect, useLayoutEffect, useRef, useState} from "react";
import type {TwistStamped} from "../../../types/ros.ts";
import type {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";

const JOY_SEND_INTERVAL_MS = 100;
// Keep the existing UI latch through short telemetry blips. Motion itself is
// invalidated immediately on a mode change and requires a new pointer gesture.
const MANUAL_EXIT_DEBOUNCE_MS = 1200;
const MAX_LINEAR_MPS = 0.25;
const MAX_ANGULAR_RAD_S = 0.6;

interface UseManualModeOptions {
    mowerAction: (action: string, params: Record<string, unknown>) => () => Promise<void>;
    joyStream: {
        sendJsonMessage: (msg: unknown, keep?: boolean) => void;
        start: (uri: string) => void;
        motionSession?: () => {connected: boolean; generation: number};
    };
    stateName?: string;
}

const neutralTwist = (): TwistStamped => ({
    header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
    twist: {linear: {x: 0, y: 0, z: 0}, angular: {z: 0, x: 0, y: 0}},
});

export function useManualMode({mowerAction, joyStream, stateName}: UseManualModeOptions) {
    const [manualMode, setManualMode] = useState(() => stateName === "MANUAL_MOWING");
    const exitTimerRef = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);
    const lastTwistRef = useRef<TwistStamped | null>(null);
    const joyIntervalRef = useRef<ReturnType<typeof setInterval> | undefined>(undefined);
    const gestureRef = useRef(false);
    const gestureGenerationRef = useRef<number | undefined>(undefined);
    const stopBlockedRef = useRef(false);
    const stopPendingRef = useRef(false);
    const intentRef = useRef(0);
    const mountedRef = useRef(true);
    const previousStateRef = useRef(stateName);
    const eligibleRef = useRef(false);
    const {sendJsonMessage, motionSession} = joyStream;

    const neutralizeMotion = useCallback(() => {
        clearInterval(joyIntervalRef.current);
        joyIntervalRef.current = undefined;
        lastTwistRef.current = null;
        gestureRef.current = false;
        gestureGenerationRef.current = undefined;
        // Best effort only: an unavailable socket cannot attest physical STOP.
        // Never queue a Twist for a later WebSocket reconnection.
        try { sendJsonMessage(neutralTwist(), false); } catch { /* stream unavailable */ }
    }, [sendJsonMessage]);

    useLayoutEffect(() => {
        eligibleRef.current = stateName === "MANUAL_MOWING" || stateName === "RECORDING" ||
            (stateName === undefined && manualMode);
        if (previousStateRef.current !== stateName) {
            neutralizeMotion();
            if (eligibleRef.current && !stopPendingRef.current) stopBlockedRef.current = false;
            previousStateRef.current = stateName;
        }
    }, [stateName, manualMode, neutralizeMotion]);

    useEffect(() => {
        if (stateName === "MANUAL_MOWING") {
            clearTimeout(exitTimerRef.current);
            exitTimerRef.current = undefined;
            if (!stopBlockedRef.current) setManualMode(true);
            return;
        }
        if (manualMode && exitTimerRef.current === undefined) {
            exitTimerRef.current = setTimeout(() => {
                exitTimerRef.current = undefined;
                setManualMode(false);
            }, MANUAL_EXIT_DEBOUNCE_MS);
        }
    }, [stateName, manualMode]);

    // Run before passive stream teardown so neutral can still use an open socket.
    useLayoutEffect(() => {
        mountedRef.current = true;
        return () => {
            mountedRef.current = false;
            clearTimeout(exitTimerRef.current);
            neutralizeMotion();
        };
    }, [neutralizeMotion]);

    const handleManualMode = async () => {
        if (stopPendingRef.current) throw new Error("STOP request is still pending");
        const intent = ++intentRef.current;
        neutralizeMotion();
        await mowerAction("high_level_control", {Command: 7})();
        if (!mountedRef.current || intent !== intentRef.current) return;
        // The BT owns blade authorization. Starting this UI never enables it
        // directly, and still requires a fresh joystick pointer-down.
        stopBlockedRef.current = false;
        setManualMode(true);
    };

    const handleStopIntent = useCallback(() => {
        ++intentRef.current;
        stopBlockedRef.current = true;
        clearTimeout(exitTimerRef.current);
        exitTimerRef.current = undefined;
        neutralizeMotion();
        setManualMode(false);
    }, [neutralizeMotion]);

    const handleStopManualMode = async () => {
        handleStopIntent();
        stopPendingRef.current = true;
        try {
            await mowerAction("high_level_control", {Command: 8})();
            await mowerAction("mow_enabled", {mow_enabled: 0, mow_direction: 0})();
        } finally {
            stopPendingRef.current = false;
        }
    };

    const gestureAllowed = useCallback(() => {
        const session = motionSession?.();
        return mountedRef.current && eligibleRef.current && !stopBlockedRef.current &&
            (!session || (session.connected && session.generation === gestureGenerationRef.current));
    }, [motionSession]);

    const handleJoyStart = useCallback(() => {
        neutralizeMotion();
        const session = motionSession?.();
        if (!mountedRef.current || !eligibleRef.current || stopBlockedRef.current || (session && !session.connected)) return;
        gestureGenerationRef.current = session?.generation;
        gestureRef.current = true;
    }, [motionSession, neutralizeMotion]);

    const handleJoyMove = useCallback((event: IJoystickUpdateEvent) => {
        if (!gestureRef.current) return;
        if (!gestureAllowed()) { neutralizeMotion(); return; }
        const msg: TwistStamped = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {
                linear: {x: (event.y ?? 0) * MAX_LINEAR_MPS, y: 0, z: 0},
                angular: {x: 0, y: 0, z: (event.x ?? 0) * -MAX_ANGULAR_RAD_S},
            },
        };
        lastTwistRef.current = msg;
        try { sendJsonMessage(msg, false); } catch { neutralizeMotion(); return; }
        if (joyIntervalRef.current === undefined) {
            joyIntervalRef.current = setInterval(() => {
                if (!gestureAllowed()) { neutralizeMotion(); return; }
                if (lastTwistRef.current) {
                    try { sendJsonMessage(lastTwistRef.current, false); } catch { neutralizeMotion(); }
                }
            }, JOY_SEND_INTERVAL_MS);
        }
    }, [gestureAllowed, neutralizeMotion, sendJsonMessage]);

    const handleJoyStop = useCallback(() => neutralizeMotion(), [neutralizeMotion]);

    return {manualMode, handleManualMode, handleStopManualMode, handleStopIntent, handleJoyStart, handleJoyMove, handleJoyStop};
}
