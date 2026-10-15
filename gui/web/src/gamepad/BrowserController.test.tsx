import {useEffect, useRef, useState, type ReactNode} from 'react';
import {act, cleanup, fireEvent, render, screen, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {App, type DrawerProps} from 'antd';
import {BrowserController} from './BrowserController';
import type {ControllerFault} from './useBrowserGamepad';
import type {InputLoss} from './controller';

const mocks = vi.hoisted(() => ({
    trace: [] as string[],
    focusRestored: false,
    completeClose: () => {},
    controller: {
        pads: [{
            id: 'Fixture standard controller', index: 0, mapping: 'standard', connected: true,
            timestamp: 1, axes: [0, 0],
            buttons: Array.from({length: 16}, () => ({pressed: false, touched: false, value: 0})),
        }],
        enabled: false,
        state: 'ready',
        fault: null as ControllerFault,
        inputLoss: null as InputLoss | null,
        enable: vi.fn<() => void>(),
        disable: vi.fn(),
        revokeInput: vi.fn<(detail: InputLoss) => void>(),
    },
}));

vi.mock('./useBrowserGamepad', async () => ({
    ...await vi.importActual<typeof import('./useBrowserGamepad')>('./useBrowserGamepad'),
    useBrowserGamepad: () => ({...mocks.controller, enable: () => mocks.controller.enable()}),
}));
vi.mock('antd', async () => {
    const actual = await vi.importActual<typeof import('antd')>('antd');
    function DrawerContents({children}: {children: ReactNode}) {
        useEffect(() => () => { mocks.trace.push('teardown'); }, []);
        return children;
    }
    function Drawer({open = false, children, title, afterOpenChange, onClose}: DrawerProps) {
        const [mounted, setMounted] = useState(open);
        const [previousOpen, setPreviousOpen] = useState(open);
        if (open !== previousOpen) {
            setPreviousOpen(open);
            if (open) setMounted(true);
        }
        const currentOpen = useRef(open);
        useEffect(() => {
            const wasOpen = currentOpen.current;
            currentOpen.current = open;
            if (open && !wasOpen) {
                mocks.focusRestored = false;
                afterOpenChange?.(true);
            }
        }, [open, afterOpenChange]);
        // Match rc-drawer's order: notify first, then restore focus. Destroying
        // the retained panel commits afterward, before activation's effect.
        useEffect(() => {
            mocks.completeClose = () => {
                mocks.trace.push('close callback');
                afterOpenChange?.(false);
                mocks.focusRestored = true;
                mocks.trace.push('focus restored');
                if (!currentOpen.current) setMounted(false);
            };
        }, [afterOpenChange]);
        return mounted ? <section role="dialog" aria-label={typeof title === 'string' ? title : undefined}
            aria-hidden={!open} data-testid="drawer-content">
            <button onClick={onClose}>Close configuration</button>
            <DrawerContents>{children}</DrawerContents>
        </section> : null;
    }
    return {...actual, Drawer};
});

const originalGetGamepads = Object.getOwnPropertyDescriptor(navigator, 'getGamepads');

beforeEach(() => {
    vi.clearAllMocks();
    localStorage.clear();
    mocks.trace = [];
    mocks.focusRestored = false;
    mocks.controller.enabled = false;
    mocks.controller.fault = null;
    mocks.controller.inputLoss = null;
    mocks.controller.disable.mockImplementation(() => {
        mocks.controller.enabled = false;
        mocks.controller.fault = null;
    });
    mocks.controller.revokeInput.mockImplementation(detail => {
        mocks.controller.fault = 'inputLost';
        mocks.controller.inputLoss = detail;
    });
    mocks.controller.enable.mockImplementation(() => {
        expect(mocks.focusRestored).toBe(true);
        expect(screen.queryByTestId('drawer-content')).not.toBeInTheDocument();
        expect(document.activeElement).toBe(screen.getByRole('button', {name: 'Controller'}));
        mocks.trace.push('enable');
    });
    Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: () => []});
});

afterEach(() => {
    cleanup();
    vi.restoreAllMocks();
    if (originalGetGamepads) Object.defineProperty(navigator, 'getGamepads', originalGetGamepads);
    else Reflect.deleteProperty(navigator, 'getGamepads');
});

function mount() {
    return render(<App><BrowserController mowerAction={vi.fn(() => vi.fn().mockResolvedValue(undefined))}
        onOwnershipChange={vi.fn()}/></App>);
}

function openSettings() {
    fireEvent.click(screen.getByRole('button', {name: 'Controller'}));
    return screen.getByRole('dialog');
}

function selectController() {
    fireEvent.mouseDown(within(screen.getByRole('dialog')).getAllByRole('combobox')[0]);
    fireEvent.click(screen.getByText('1: Fixture standard controller', {exact: true}));
}

function requestEnable() {
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', {name: 'Enable for this robot'}));
}

describe('BrowserController staged activation', () => {
    it('enables once after the close callback, focus restoration and panel teardown', () => {
        const view = mount();
        const focus = vi.spyOn(screen.getByRole('button', {name: 'Controller'}), 'focus');
        openSettings();
        selectController();
        requestEnable();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
        expect(focus).not.toHaveBeenCalled();
        expect(screen.getByTestId('drawer-content')).toBeInTheDocument();

        act(() => { mocks.completeClose(); });
        expect(mocks.trace).toEqual(['close callback', 'focus restored', 'teardown', 'enable']);
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
        expect(focus).toHaveBeenCalledExactlyOnceWith({preventScroll: true});
        view.rerender(<App><BrowserController mowerAction={vi.fn(() => vi.fn().mockResolvedValue(undefined))}
            onOwnershipChange={vi.fn()}/></App>);
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
        expect(focus).toHaveBeenCalledTimes(1);
    });

    it('does not activate when configuration is closed normally', () => {
        mount();
        const focus = vi.spyOn(screen.getByRole('button', {name: 'Controller'}), 'focus');
        const dialog = openSettings();
        selectController();
        fireEvent.click(within(dialog).getByRole('button', {name: 'Close configuration'}));
        act(() => { mocks.completeClose(); });
        expect(mocks.controller.enable).not.toHaveBeenCalled();
        expect(focus).not.toHaveBeenCalled();
    });

    it('cancels a pending activation when configuration is reopened during closing', () => {
        mount();
        openSettings();
        selectController();
        requestEnable();
        const finishPreviousClose = mocks.completeClose;
        openSettings();
        act(() => { finishPreviousClose(); });
        expect(mocks.controller.enable).not.toHaveBeenCalled();
        expect(screen.getByRole('dialog')).toBeInTheDocument();

        requestEnable();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
        act(() => { mocks.completeClose(); });
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
    });

    it.each([
        ['blur', 'focusLost'],
        ['pagehide', 'pageExit'],
        ['visibilitychange', 'pageHidden'],
        ['gamepaddisconnected', 'disconnected'],
        ['gamepadconnected', 'controllerChanged'],
    ] as const)('cancels pending activation on %s and preserves its diagnostic', (eventType, reason) => {
        mount();
        const focus = vi.spyOn(screen.getByRole('button', {name: 'Controller'}), 'focus');
        openSettings();
        selectController();
        requestEnable();

        if (eventType === 'blur') vi.spyOn(document, 'hasFocus').mockReturnValue(false);
        const expectedDetail = eventType === 'blur' ? {
            reason, source: 'pendingBlur', documentFocused: false,
            visibility: document.visibilityState, activeElement: document.activeElement?.tagName ?? 'none',
        } : {reason};
        act(() => {
            const event = new Event(eventType);
            if (eventType.startsWith('gamepad')) Object.assign(event, {gamepad: {index: 0}});
            (eventType === 'visibilitychange' ? document : window).dispatchEvent(event);
        });
        act(() => { mocks.completeClose(); });

        expect(mocks.controller.enable).not.toHaveBeenCalled();
        expect(focus).not.toHaveBeenCalled();
        expect(mocks.controller.revokeInput).toHaveBeenCalledExactlyOnceWith(expect.objectContaining(expectedDetail));
        expect(mocks.controller.inputLoss).toEqual(expect.objectContaining(expectedDetail));
    });

    it.each(['gamepaddisconnected', 'gamepadconnected'])('ignores another controller slot during pending activation (%s)', eventType => {
        mount();
        openSettings();
        selectController();
        requestEnable();

        act(() => {
            window.dispatchEvent(Object.assign(new Event(eventType), {gamepad: {index: 1}}));
        });
        expect(mocks.controller.enable).not.toHaveBeenCalled();
        expect(mocks.controller.revokeInput).not.toHaveBeenCalled();
        act(() => { mocks.completeClose(); });
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
    });

    it('allows another explicit activation after reopening configuration', () => {
        mount();
        const focus = vi.spyOn(screen.getByRole('button', {name: 'Controller'}), 'focus');
        openSettings();
        selectController();
        requestEnable();
        act(() => { mocks.completeClose(); });
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
        expect(focus).toHaveBeenCalledTimes(1);

        openSettings();
        requestEnable();
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
        expect(focus).toHaveBeenCalledTimes(1);
        act(() => { mocks.completeClose(); });
        expect(mocks.controller.enable).toHaveBeenCalledTimes(2);
        expect(focus).toHaveBeenCalledTimes(2);
        expect(mocks.trace.filter(event => event === 'enable')).toHaveLength(2);
    });

    it('does not refocus or reactivate after a later genuine blur', () => {
        const view = mount();
        const focus = vi.spyOn(screen.getByRole('button', {name: 'Controller'}), 'focus');
        openSettings();
        selectController();
        requestEnable();
        act(() => { mocks.completeClose(); });
        expect(focus).toHaveBeenCalledTimes(1);

        render(<input aria-label="Unrelated control"/>);
        const other = screen.getByRole('textbox', {name: 'Unrelated control'});
        vi.spyOn(document, 'hasFocus').mockReturnValue(false);
        act(() => {
            other.focus();
            window.dispatchEvent(new Event('blur'));
        });
        view.rerender(<App><BrowserController mowerAction={vi.fn(() => vi.fn().mockResolvedValue(undefined))}
            onOwnershipChange={vi.fn()}/></App>);

        expect(document.activeElement).toBe(other);
        expect(focus).toHaveBeenCalledTimes(1);
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
    });

    it('keeps the prior input-loss diagnostic visible when configuration reopens', () => {
        mocks.controller.fault = 'inputLost';
        mocks.controller.inputLoss = {reason: 'pollGap', pollGapMs: 287.4};
        mount();
        expect(screen.getByRole('alert')).toHaveTextContent('287 ms');

        const dialog = openSettings();
        expect(within(dialog).getByText('The browser paused controller reads for 287 ms (limit 200 ms).')).toBeVisible();
        expect(within(dialog).getByText(JSON.stringify(mocks.controller.inputLoss))).toBeVisible();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
    });
});
