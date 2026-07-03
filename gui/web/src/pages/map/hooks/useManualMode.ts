import {useCallback, useEffect, useRef, useState} from "react";
import type {TwistStamped} from "../../../types/ros.ts";
import type {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";

const JOY_SEND_INTERVAL_MS = 100;
// How long a sustained non-MANUAL_MOWING state must persist before we tear
// down the manual UI. A single stray guard frame (EMERGENCY/battery/boundary
// blip emits one non-MANUAL tick) must NOT collapse manual mode and kill the
// joystick socket mid-drive — only a genuine, sustained exit should.
const MANUAL_EXIT_DEBOUNCE_MS = 1200;
// Teleop velocity caps — raw joystick values are in [-1, 1] (normalized by
// react-joystick-component). Multiplied at this layer (before twist_mux) so
// Nav2 autonomous speeds are unaffected. Tuned for precise manual control:
// at 1.0 m/s the robot was too twitchy on grass.
const MAX_LINEAR_MPS = 0.25;
const MAX_ANGULAR_RAD_S = 0.6;

interface UseManualModeOptions {
    mowerAction: (action: string, params: Record<string, unknown>) => () => Promise<void>;
    joyStream: { sendJsonMessage: (msg: unknown) => void; start: (uri: string) => void };
    stateName?: string;
}

export function useManualMode({mowerAction, joyStream, stateName}: UseManualModeOptions) {
    const [manualMode, setManualMode] = useState(() => stateName === "MANUAL_MOWING");
    const exitTimerRef = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

    // LATCH + DEBOUNCE manual mode. Entering MANUAL_MOWING latches it ON
    // immediately; leaving it only tears the UI down after a sustained
    // (debounced) window of non-MANUAL frames. This keeps a single stray guard
    // frame (EMERGENCY/battery/boundary blip ahead of MainLogic) from collapsing
    // the manual UI and killing the joystick socket mid-drive. The explicit Stop
    // button flips manualMode directly, so it doesn't depend on this debounce.
    useEffect(() => {
        if (stateName === "MANUAL_MOWING") {
            clearTimeout(exitTimerRef.current);
            exitTimerRef.current = undefined;
            setManualMode(true);
            return;
        }
        // Non-MANUAL frame while latched: arm (or keep) the debounce timer.
        if (manualMode && exitTimerRef.current === undefined) {
            exitTimerRef.current = setTimeout(() => {
                exitTimerRef.current = undefined;
                setManualMode(false);
            }, MANUAL_EXIT_DEBOUNCE_MS);
        }
    }, [stateName, manualMode]);

    const lastTwistRef = useRef<TwistStamped | null>(null);
    const joyIntervalRef = useRef<ReturnType<typeof setInterval> | undefined>(undefined);

    const startJoyInterval = useCallback(() => {
        clearInterval(joyIntervalRef.current);
        joyIntervalRef.current = setInterval(() => {
            if (lastTwistRef.current) {
                joyStream.sendJsonMessage(lastTwistRef.current);
            }
        }, JOY_SEND_INTERVAL_MS);
    }, [joyStream]);

    const stopJoyInterval = useCallback(() => {
        clearInterval(joyIntervalRef.current);
        joyIntervalRef.current = undefined;
    }, []);

    // Cleanup on unmount — stop joy interval and any pending exit debounce
    useEffect(() => {
        return () => {
            clearInterval(joyIntervalRef.current);
            clearTimeout(exitTimerRef.current);
        };
    }, []);

    const handleManualMode = async () => {
        // Joy stream is auto-started by useMapStreams when state becomes MANUAL_MOWING.
        // Send the command first — the BT will transition to MANUAL_MOWING state.
        await mowerAction("high_level_control", {Command: 7})();
        // The BT owns the blade: once state=4 (MANUAL_MOWING) it re-ticks
        // SetMowerEnabled(true) ~10 Hz. We deliberately do NOT send mow_enabled=1
        // from the client here — that call races the firmware, which zeroes the
        // blade while the HL mode is still IDLE (this immediate send + a 10 s
        // keepalive was documented as REMOVED in main_tree.xml). Latch the UI.
        setManualMode(true);
    };

    const handleStopManualMode = async () => {
        await mowerAction("high_level_control", {Command: 2})();
        // Explicit Stop: drop the manual UI immediately and cancel any pending
        // debounce so a lingering timer can't re-toggle it.
        clearTimeout(exitTimerRef.current);
        exitTimerRef.current = undefined;
        stopJoyInterval();
        lastTwistRef.current = null;
        setManualMode(false);
        await mowerAction("mow_enabled", {mow_enabled: 0, mow_direction: 0})();
    };

    const handleJoyMove = useCallback((event: IJoystickUpdateEvent) => {
        const linear = (event.y ?? 0) * MAX_LINEAR_MPS;
        const angular = (event.x ?? 0) * -1 * MAX_ANGULAR_RAD_S;
        const msg: TwistStamped = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: linear, y: 0, z: 0}, angular: {z: angular, x: 0, y: 0}},
        };
        lastTwistRef.current = msg;
        joyStream.sendJsonMessage(msg);
        if (!joyIntervalRef.current) {
            startJoyInterval();
        }
    }, [joyStream, startJoyInterval]);

    const handleJoyStop = useCallback(() => {
        const msg: TwistStamped = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0, y: 0, z: 0}, angular: {z: 0, x: 0, y: 0}},
        };
        lastTwistRef.current = null;
        stopJoyInterval();
        joyStream.sendJsonMessage(msg);
    }, [joyStream, stopJoyInterval]);

    return {manualMode, handleManualMode, handleStopManualMode, handleJoyMove, handleJoyStop};
}
