import {useCallback, useEffect, useRef, useState} from "react";
import type {TwistStamped} from "../../../types/ros.ts";
import type {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";

const JOY_SEND_INTERVAL_MS = 100;
const BLADE_KEEPALIVE_MS = 10000;
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

    useEffect(() => {
        setManualMode(stateName === "MANUAL_MOWING");
    }, [stateName]);
    const lastTwistRef = useRef<TwistStamped | null>(null);
    const joyIntervalRef = useRef<ReturnType<typeof setInterval> | undefined>(undefined);
    const bladeIntervalRef = useRef<ReturnType<typeof setInterval> | undefined>(undefined);

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

    const stopBladeKeepalive = useCallback(() => {
        clearInterval(bladeIntervalRef.current);
        bladeIntervalRef.current = undefined;
    }, []);

    // Cleanup on unmount — stop blade keepalive and joy interval
    useEffect(() => {
        return () => {
            clearInterval(bladeIntervalRef.current);
            clearInterval(joyIntervalRef.current);
        };
    }, []);

    const handleManualMode = async () => {
        // Joy stream is auto-started by useMapStreams when state becomes MANUAL_MOWING.
        // Send the command first — the BT will transition to MANUAL_MOWING state.
        await mowerAction("high_level_control", {Command: 7})();
        // Enable mowing blade and keep it alive
        await mowerAction("mow_enabled", {MowEnabled: 1, MowDirection: 0})();
        stopBladeKeepalive();
        bladeIntervalRef.current = setInterval(() => {
            mowerAction("mow_enabled", {MowEnabled: 1, MowDirection: 0})();
        }, BLADE_KEEPALIVE_MS);
        setManualMode(true);
    };

    const handleStopManualMode = async () => {
        await mowerAction("high_level_control", {Command: 2})();
        stopBladeKeepalive();
        stopJoyInterval();
        lastTwistRef.current = null;
        setManualMode(false);
        await mowerAction("mow_enabled", {MowEnabled: 0, MowDirection: 0})();
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
