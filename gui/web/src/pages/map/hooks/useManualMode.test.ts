import {describe, it, expect, vi, beforeEach, afterEach} from 'vitest';
import {renderHook, act} from '@testing-library/react';
import {useManualMode} from './useManualMode.ts';

describe('useManualMode', () => {
    let mowerAction: (action: string, params: Record<string, unknown>) => () => Promise<void>;
    let sendCommand: (msg: any) => void;
    let requestControl: () => void;
    let releaseControl: () => void;

    beforeEach(() => {
        mowerAction = vi.fn(() => vi.fn().mockResolvedValue(undefined));
        sendCommand = vi.fn();
        requestControl = vi.fn();
        releaseControl = vi.fn();
        vi.useFakeTimers();
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    function renderManualMode() {
        return renderHook(() =>
            useManualMode({
                mowerAction,
                joyStream: {sendCommand, requestControl, releaseControl, isOwner: true},
            })
        );
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
        expect(requestControl).toHaveBeenCalledTimes(1);
        expect(result.current.manualMode).toBe(true);
    });

    it('requests teleop control only after the manual-mode request succeeds', async () => {
        let completeAction!: () => void;
        mowerAction = vi.fn(() => () => new Promise<void>((resolve) => {
            completeAction = resolve;
        }));
        const {result} = renderManualMode();

        let pending!: Promise<void>;
        act(() => {
            pending = result.current.handleManualMode();
        });
        expect(requestControl).not.toHaveBeenCalled();

        await act(async () => {
            completeAction();
            await pending;
        });
        expect(requestControl).toHaveBeenCalledTimes(1);
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
                joyStream: {sendCommand, requestControl, releaseControl, isOwner: true},
                stateName,
            }),
            {initialProps: {stateName: 'MANUAL_MOWING'}},
        );
        expect(result.current.manualMode).toBe(true);

        rerender({stateName: 'IDLE'});
        act(() => vi.advanceTimersByTime(1199));
        expect(result.current.manualMode).toBe(true);

        act(() => vi.advanceTimersByTime(1));
        expect(result.current.manualMode).toBe(false);
    });

    it('handleJoyMove scales joystick input by MAX_LINEAR_MPS and MAX_ANGULAR_RAD_S', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyMove({x: 0.5, y: 0.8} as any);
        });
        // Raw joystick: x=0.5, y=0.8 → scaled to linear=0.8*0.25=0.2, angular=-0.5*0.6=-0.3
        expect(sendCommand).toHaveBeenCalledWith({
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0.2, y: 0, z: 0}, angular: {z: -0.3, x: 0, y: 0}},
        });
    });

    it('handleJoyStop sends zero velocity', () => {
        const {result} = renderManualMode();
        act(() => {
            result.current.handleJoyStop();
        });
        expect(sendCommand).toHaveBeenCalledWith({
            header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
            twist: {linear: {x: 0, y: 0, z: 0}, angular: {z: 0, x: 0, y: 0}},
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
