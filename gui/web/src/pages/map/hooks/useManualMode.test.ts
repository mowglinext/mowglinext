import {describe, it, expect, vi, beforeEach, afterEach} from 'vitest';
import {renderHook, act} from '@testing-library/react';
import type {IJoystickUpdateEvent} from 'react-joystick-component/build/lib/Joystick';
import {useManualMode} from './useManualMode.ts';

describe('useManualMode', () => {
    let mowerAction: (action: string, params: Record<string, unknown>) => () => Promise<void>;
    let sendJsonMessage: (msg: unknown) => void;
    let startStream: (uri: string) => void;

    beforeEach(() => {
        mowerAction = vi.fn(() => vi.fn().mockResolvedValue(undefined));
        sendJsonMessage = vi.fn();
        startStream = vi.fn();
        vi.useFakeTimers();
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    function renderManualMode(controllerOwnsInput = false) {
        return renderHook(({ownsInput}: {ownsInput: boolean}) =>
            useManualMode({
                mowerAction,
                joyStream: {sendJsonMessage, start: startStream},
                controllerOwnsInput: ownsInput,
            }),
            {initialProps: {ownsInput: controllerOwnsInput}},
        );
    }

    function advanceBy(milliseconds: number) {
        act(() => { void vi.advanceTimersByTime(milliseconds); });
    }

    function joystickMove(x: number, y: number): IJoystickUpdateEvent {
        return {type: 'move', x, y, direction: null, distance: 0};
    }

    it('starts with manual mode off', () => {
        const {result} = renderManualMode();
        expect(result.current.manualMode).toBe(false);
    });

    it('activates manual mode without sending a client-side blade command', async () => {
        const {result} = renderManualMode();
        await act(async () => {
            await result.current.handleManualMode();
        });
        expect(mowerAction).toHaveBeenCalledTimes(1);
        expect(mowerAction).toHaveBeenCalledWith('high_level_control', {Command: 7});
        expect(result.current.manualMode).toBe(true);
    });

    it('stops in place, disables the blade, and deactivates manual mode', async () => {
        const {result} = renderManualMode();
        await act(async () => {
            await result.current.handleManualMode();
        });
        expect(result.current.manualMode).toBe(true);
        vi.mocked(mowerAction).mockClear();

        await act(async () => {
            await result.current.handleStopManualMode();
        });
        expect(mowerAction).toHaveBeenCalledTimes(2);
        expect(mowerAction).toHaveBeenNthCalledWith(1, 'high_level_control', {Command: 8});
        expect(mowerAction).toHaveBeenNthCalledWith(2, 'mow_enabled', {mow_enabled: 0, mow_direction: 0});
        expect(result.current.manualMode).toBe(false);
    });

    it('keeps manual mode latched through a short non-manual state blip', () => {
        const {result, rerender} = renderHook(
            ({stateName}: {stateName: string | undefined}) => useManualMode({
                mowerAction,
                joyStream: {sendJsonMessage, start: startStream},
                stateName,
            }),
            {initialProps: {stateName: 'MANUAL_MOWING'}},
        );
        expect(result.current.manualMode).toBe(true);

        rerender({stateName: 'IDLE'});
        advanceBy(1199);
        expect(result.current.manualMode).toBe(true);

        advanceBy(1);
        expect(result.current.manualMode).toBe(false);
    });

    it('handleJoyMove scales joystick input by MAX_LINEAR_MPS and MAX_ANGULAR_RAD_S', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyMove(joystickMove(0.5, 0.8));
        });
        // Raw joystick: x=0.5, y=0.8 → scaled to linear=0.8*0.25=0.2, angular=-0.5*0.6=-0.3
        expect(sendJsonMessage).toHaveBeenCalledWith({
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0.2, y: 0, z: 0}, angular: {z: -0.3, x: 0, y: 0}},
        });
    });

    it('handleJoyStop sends zero velocity', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyStop();
        });
        expect(sendJsonMessage).toHaveBeenCalledWith({
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0, y: 0, z: 0}, angular: {z: 0, x: 0, y: 0}},
        });
    });

    it('revokes touch joystick ownership cleanly and requires fresh movement after gamepad release', () => {
        const {result, rerender} = renderManualMode();
        act(() => { result.current.handleJoyMove(joystickMove(0, 0.8)); });
        advanceBy(200);
        expect(sendJsonMessage).toHaveBeenCalledTimes(3);
        const oldTouchFrame = {
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0.2, y: 0, z: 0}, angular: {z: -0, x: 0, y: 0}},
        };
        expect(sendJsonMessage).toHaveBeenLastCalledWith(oldTouchFrame);

        rerender({ownsInput: true});
        advanceBy(200);
        act(() => { result.current.handleJoyMove(joystickMove(-0.5, 0.2)); });
        expect(sendJsonMessage).toHaveBeenCalledTimes(3);

        rerender({ownsInput: false});
        advanceBy(200);
        expect(sendJsonMessage).toHaveBeenCalledTimes(3);

        act(() => { result.current.handleJoyMove(joystickMove(-0.5, -0.4)); });
        expect(sendJsonMessage).toHaveBeenCalledTimes(4);
        expect(sendJsonMessage).toHaveBeenLastCalledWith({
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: -0.1, y: 0, z: 0}, angular: {z: 0.3, x: 0, y: 0}},
        });
        advanceBy(100);
        expect(sendJsonMessage).toHaveBeenCalledTimes(5);
        expect(sendJsonMessage).toHaveBeenLastCalledWith({
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: -0.1, y: 0, z: 0}, angular: {z: 0.3, x: 0, y: 0}},
        });
    });

    it('cleans up timers on unmount', async () => {
        const {result, unmount} = renderManualMode();
        await act(async () => {
            await result.current.handleManualMode();
        });
        expect(result.current.manualMode).toBe(true);
        unmount();
        // No assertion needed — just verifying no error/leak on unmount
    });
});
