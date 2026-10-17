import {useEffect, useRef, useState, type ReactNode} from 'react';
import {act, cleanup, fireEvent, render, screen, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {App, type DrawerProps} from 'antd';
import {BrowserController} from './BrowserController';
import type {ControllerFault, useBrowserGamepad} from './useBrowserGamepad';
import {controllerKey, DEFAULT_PROFILE, saveProfile, saveSelectedControllerKey, type InputLoss} from './controller';

type HookOptions = Parameters<typeof useBrowserGamepad>[0];

const mocks = vi.hoisted(() => ({
    trace: [] as string[],
    focusRestored: false,
    completeClose: () => {},
    hookOptions: null as HookOptions | null,
    controller: {
        pads: [{
            id: 'Fixture standard controller', index: 0, mapping: 'standard' as GamepadMappingType, connected: true,
            timestamp: 1, axes: [0, 0],
            buttons: Array.from({length: 16}, () => ({pressed: false, touched: false, value: 0})),
        }],
        enabled: false,
        state: 'ready',
        fault: null as ControllerFault,
        inputLoss: null as InputLoss | null,
        enable: vi.fn<() => void>(),
        disable: vi.fn(),
    },
}));

vi.mock('./useBrowserGamepad', async () => ({
    ...await vi.importActual<typeof import('./useBrowserGamepad')>('./useBrowserGamepad'),
    useBrowserGamepad: (options: HookOptions) => {
        useEffect(() => { mocks.hookOptions = options; }, [options]);
        return {...mocks.controller, enable: () => mocks.controller.enable()};
    },
}));
vi.mock('antd', async () => {
    const actual = await vi.importActual<typeof import('antd')>('antd');
    function DrawerContents({children}: {children: ReactNode}) {
        useEffect(() => () => { mocks.trace.push('teardown'); }, []);
        return children;
    }
    function Drawer({open = false, children, title, afterOpenChange, onClose, footer}: DrawerProps) {
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
            <DrawerContents>{children}{footer}</DrawerContents>
        </section> : null;
    }
    return {...actual, Drawer};
});

const originalGetGamepads = Object.getOwnPropertyDescriptor(navigator, 'getGamepads');
const initialPad = structuredClone(mocks.controller.pads[0]);

beforeEach(() => {
    vi.clearAllMocks();
    localStorage.clear();
    mocks.trace = [];
    mocks.focusRestored = false;
    mocks.hookOptions = null;
    mocks.controller.enabled = false;
    mocks.controller.fault = null;
    mocks.controller.inputLoss = null;
    mocks.controller.pads = [structuredClone(initialPad)];
    mocks.controller.disable.mockImplementation(() => {
        mocks.controller.enabled = false;
        mocks.controller.fault = null;
    });
    mocks.controller.enable.mockImplementation(() => {
        expect(screen.queryByTestId('drawer-content')).not.toBeInTheDocument();
        expect(document.activeElement).toBe(screen.getByRole('button', {name: 'Enable for this robot'}));
        mocks.trace.push('enable');
        mocks.controller.enabled = true;
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
    const scene = () => <App><BrowserController mowerAction={vi.fn(() => vi.fn().mockResolvedValue(undefined))}
        onOwnershipChange={vi.fn()}/></App>;
    const view = render(scene());
    return {...view, refresh: () => view.rerender(scene())};
}

function openSettings() {
    fireEvent.click(screen.getByRole('button', {name: 'Controller'}));
    return screen.getByRole('dialog');
}

function selectController() {
    fireEvent.mouseDown(within(screen.getByRole('dialog')).getAllByRole('combobox')[0]);
    fireEvent.click(screen.getByText('1: Fixture standard controller', {exact: true}));
}

function done() {
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', {name: 'Done'}));
}

function configure() {
    openSettings();
    selectController();
    done();
    act(() => { mocks.completeClose(); });
    return screen.getByRole('button', {name: 'Enable for this robot'});
}

function notifyInventory() {
    const observer = mocks.hookOptions?.onInventoryChange;
    if (!observer) throw new Error('Inventory observer is missing');
    act(() => { observer(mocks.controller.pads); });
}

describe('BrowserController map activation', () => {
    it('restores one remembered fingerprint at its current slot without opening settings or activating', () => {
        const key = controllerKey(mocks.controller.pads[0]);
        saveSelectedControllerKey(key);
        saveProfile(key, {...DEFAULT_PROFILE, buttons: {...DEFAULT_PROFILE.buttons, deadman: 7}});
        mocks.controller.pads[0].index = 3;
        const view = mount();
        notifyInventory();

        const enable = screen.getByRole('button', {name: 'Enable for this robot'});
        expect(enable).toBeEnabled();
        expect(screen.getByText('4: Fixture standard controller')).toBeVisible();
        expect(screen.queryByRole('dialog')).not.toBeInTheDocument();
        expect(mocks.hookOptions?.selected).toEqual({index: 3, id: 'Fixture standard controller'});
        expect(mocks.hookOptions?.profile.buttons.deadman).toBe(7);
        expect(mocks.controller.enable).not.toHaveBeenCalled();

        fireEvent.click(enable);
        view.refresh();
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
        expect(screen.getByRole('status')).toHaveTextContent('RT / R2 (Button 7)');
    });

    it('does not choose between two devices with the remembered fingerprint', () => {
        saveSelectedControllerKey(controllerKey(mocks.controller.pads[0]));
        mocks.controller.pads.push({...structuredClone(initialPad), index: 1});
        mount();
        notifyInventory();
        expect(mocks.hookOptions?.selected).toBeNull();
        expect(screen.queryByRole('button', {name: 'Enable for this robot'})).not.toBeInTheDocument();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
    });

    it('withdraws an automatic choice when a duplicate appears and requires an explicit selection', () => {
        saveSelectedControllerKey(controllerKey(mocks.controller.pads[0]));
        const view = mount();
        notifyInventory();
        expect(screen.getByRole('button', {name: 'Enable for this robot'})).toBeEnabled();

        mocks.controller.pads.push({...structuredClone(initialPad), index: 1});
        view.refresh();
        notifyInventory();
        expect(mocks.hookOptions?.selected).toBeNull();
        expect(screen.queryByRole('button', {name: 'Enable for this robot'})).not.toBeInTheDocument();
        expect(mocks.controller.enable).not.toHaveBeenCalled();

        const enable = configure();
        notifyInventory();
        expect(mocks.hookOptions?.selected).toEqual({index: 0, id: initialPad.id});
        expect(enable).toBeEnabled();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
    });

    it('retains an explicit selection when an identical device appears in another slot', () => {
        const view = mount();
        const enable = configure();
        mocks.controller.disable.mockClear();
        mocks.controller.pads.push({...structuredClone(initialPad), index: 1});
        view.refresh();
        notifyInventory();

        expect(mocks.hookOptions?.selected).toEqual({index: 0, id: initialPad.id});
        expect(screen.getByRole('button', {name: 'Enable for this robot'})).toBe(enable);
        expect(enable).toBeEnabled();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
        expect(mocks.controller.disable).not.toHaveBeenCalled();
    });

    it('retains an enabled restored choice when an identical device appears in another slot', () => {
        saveSelectedControllerKey(controllerKey(mocks.controller.pads[0]));
        const view = mount();
        notifyInventory();
        const enable = screen.getByRole('button', {name: 'Enable for this robot'});
        fireEvent.click(enable);
        view.refresh();
        mocks.controller.pads.push({...structuredClone(initialPad), index: 1});
        view.refresh();
        notifyInventory();

        expect(mocks.hookOptions?.selected).toEqual({index: 0, id: initialPad.id});
        expect(screen.getByRole('button', {name: 'Disable controller'})).toBe(enable);
        expect(mocks.controller.enabled).toBe(true);
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
        expect(mocks.controller.disable).not.toHaveBeenCalled();
    });

    it('does not restore a controller whose fingerprint does not match the remembered choice', () => {
        saveSelectedControllerKey(controllerKey(mocks.controller.pads[0]));
        mocks.controller.pads[0].id = 'Different controller';
        mount();
        notifyInventory();
        expect(mocks.hookOptions?.selected).toBeNull();
        expect(screen.queryByRole('button', {name: 'Enable for this robot'})).not.toBeInTheDocument();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
    });

    it.each(['missing', 'invalid', 'incompatible'] as const)('keeps a restored unknown layout disabled with a %s mapping', mapping => {
        mocks.controller.pads[0].mapping = '';
        const key = controllerKey(mocks.controller.pads[0]);
        saveSelectedControllerKey(key);
        if (mapping !== 'missing') localStorage.setItem(`mowglinext.gamepad.v1:${key}`, JSON.stringify({
            ...DEFAULT_PROFILE, ...(mapping === 'invalid' ? {angularAxis: DEFAULT_PROFILE.linearAxis} : {linearAxis: 2}),
        }));
        mount();
        notifyInventory();
        const enable = screen.getByRole('button', {name: 'Enable for this robot'});
        expect(enable).toBeDisabled();
        fireEvent.click(enable);
        expect(mocks.controller.enable).not.toHaveBeenCalled();
    });

    it('keeps a manual standard-layout choice usable when browser storage is denied', () => {
        vi.spyOn(Storage.prototype, 'getItem').mockImplementation(() => { throw new DOMException('Denied', 'SecurityError'); });
        vi.spyOn(Storage.prototype, 'setItem').mockImplementation(() => { throw new DOMException('Denied', 'SecurityError'); });
        mount();
        const enable = configure();
        expect(enable).toBeEnabled();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
        fireEvent.click(enable);
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
    });

    it('Done only closes settings and keeps activation disabled until teardown completes', () => {
        mount();
        openSettings();
        selectController();
        const enable = screen.getByRole('button', {name: 'Enable for this robot'});
        const focus = vi.spyOn(enable, 'focus');
        expect(enable).toBeDisabled();
        expect(within(screen.getByRole('dialog')).queryByRole('button', {name: 'Enable for this robot'})).not.toBeInTheDocument();

        done();
        expect(enable).toBeDisabled();
        fireEvent.click(enable);
        expect(mocks.controller.enable).not.toHaveBeenCalled();
        act(() => { mocks.completeClose(); });
        expect(enable).toBeEnabled();
        expect(focus).not.toHaveBeenCalled();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
    });

    it('uses a direct gesture and keeps the focused button across enable, fault and quick recovery', () => {
        const view = mount();
        const enable = configure();
        const focus = vi.spyOn(enable, 'focus');
        fireEvent.click(enable);
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
        expect(focus).toHaveBeenCalledExactlyOnceWith({preventScroll: true});
        view.refresh();
        expect(screen.getByRole('button', {name: 'Disable controller'})).toBe(enable);
        expect(document.activeElement).toBe(enable);
        view.refresh();
        expect(focus).toHaveBeenCalledTimes(1);
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);

        mocks.controller.enabled = false;
        mocks.controller.fault = 'inputLost';
        mocks.controller.inputLoss = {reason: 'focusLost', source: 'windowBlur'};
        view.refresh();
        expect(screen.getByRole('button', {name: 'Enable for this robot'})).toBe(enable);
        expect(enable).toBeEnabled();
        expect(screen.getByRole('alert')).toHaveTextContent('The browser page lost focus.');
        expect(screen.getByRole('alert')).toHaveTextContent('Center sticks, release all buttons, then tap Enable again.');
        expect(focus).toHaveBeenCalledTimes(1);
        expect(screen.queryByRole('dialog')).not.toBeInTheDocument();

        fireEvent.click(enable);
        view.refresh();
        expect(mocks.controller.enable).toHaveBeenCalledTimes(2);
        expect(focus).toHaveBeenCalledTimes(2);
        expect(screen.getByRole('button', {name: 'Disable controller'})).toBe(enable);
        mocks.controller.disable.mockClear();
        fireEvent.click(enable);
        view.refresh();
        expect(mocks.controller.disable).toHaveBeenCalledTimes(1);
        expect(mocks.controller.enable).toHaveBeenCalledTimes(2);
        expect(screen.getByRole('button', {name: 'Enable for this robot'})).toBe(enable);
    });

    it('opening settings revokes active input and does not reactivate when closed', () => {
        const view = mount();
        const enable = configure();
        fireEvent.click(enable);
        view.refresh();
        mocks.controller.disable.mockClear();
        openSettings();
        expect(mocks.controller.disable).toHaveBeenCalledTimes(1);
        expect(screen.getByRole('button', {name: 'Enable for this robot'})).toBe(enable);
        expect(enable).toBeDisabled();
        done();
        act(() => { mocks.completeClose(); });
        expect(enable).toBeEnabled();
        expect(mocks.controller.enable).toHaveBeenCalledTimes(1);
    });

    it('unknown unsaved layouts can finish settings but cannot enable on the map', () => {
        mocks.controller.pads[0].mapping = '';
        mount();
        openSettings();
        selectController();
        expect(within(screen.getByRole('dialog')).getByRole('button', {name: 'Done'})).toBeEnabled();
        done();
        act(() => { mocks.completeClose(); });
        const enable = screen.getByRole('button', {name: 'Enable for this robot'});
        expect(enable).toBeDisabled();
        fireEvent.click(enable);
        expect(mocks.controller.enable).not.toHaveBeenCalled();

        openSettings();
        fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', {name: 'Save mapping'}));
        done();
        act(() => { mocks.completeClose(); });
        expect(enable).toBeEnabled();
    });

    it('invalid axis assignments can finish settings but cannot enable on the map', () => {
        mount();
        openSettings();
        selectController();
        fireEvent.mouseDown(screen.getByRole('combobox', {name: 'Steering axis'}));
        const option = document.querySelector('.ant-select-dropdown .ant-select-item-option[title="Axis 1"]');
        if (!option) throw new Error('Axis 1 option is missing');
        fireEvent.click(option);
        expect(within(screen.getByRole('dialog')).getByRole('button', {name: 'Save mapping'})).toBeDisabled();
        expect(within(screen.getByRole('dialog')).getByRole('button', {name: 'Done'})).toBeEnabled();
        done();
        act(() => { mocks.completeClose(); });
        expect(screen.getByRole('button', {name: 'Enable for this robot'})).toBeDisabled();
        expect(mocks.controller.enable).not.toHaveBeenCalled();
    });

    it('shows the saved deadman and speed cap using standard controller names', () => {
        localStorage.setItem(`mowglinext.gamepad.v1:${controllerKey(mocks.controller.pads[0])}`, JSON.stringify({
            ...DEFAULT_PROFILE, speedScale: 0.5, buttons: {...DEFAULT_PROFILE.buttons, deadman: 7},
        }));
        const view = mount();
        const enable = configure();
        fireEvent.click(enable);
        view.refresh();
        expect(screen.getByRole('status')).toHaveTextContent('Ready — hold RT / R2 (Button 7), then move a stick');
        expect(screen.getByText('Drive: hold RT / R2 (Button 7) · Stop: A / × (Button 0) · Max 0.13 m/s')).toBeVisible();
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
