import {StrictMode, useEffect, useState} from 'react';
import {act, render} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import DrawControl from './DrawControl';

type Listener = (event: unknown) => void;
const mock = vi.hoisted(() => ({
    listeners: new Map<string, Set<Listener>>(),
    controls: [] as Array<{changeMode: ReturnType<typeof vi.fn>; add: ReturnType<typeof vi.fn>}>,
    on: vi.fn<(event: string, listener: Listener) => void>(),
    off: vi.fn<(event: string, listener: Listener) => void>(),
}));
const map = {
    on: (event: string, listener: Listener) => {
        mock.on(event, listener);
        const listeners = mock.listeners.get(event) ?? new Set<Listener>();
        listeners.add(listener);
        mock.listeners.set(event, listeners);
    },
    off: (event: string, listener: Listener) => {
        mock.off(event, listener);
        mock.listeners.get(event)?.delete(listener);
    },
};
const context = {map: {...map, getMap: () => map}};
type DrawInstance = {changeMode: ReturnType<typeof vi.fn>; add: ReturnType<typeof vi.fn>};
vi.mock('@mapbox/mapbox-gl-draw', () => ({default: class {
    static modes = {};
    changeMode = vi.fn();
    add = vi.fn();
    deleteAll = vi.fn();
    getAll = () => ({features: []});
    constructor() { mock.controls.push(this); }
}}));
vi.mock('../modes/DirectSelectWithBoxMode', () => ({default: {}}));
vi.mock('../modes/SplitLineMode', () => ({default: {}}));
vi.mock('react-map-gl/mapbox', () => ({
    // Match useControl's single resource and effect replay, without WebGL.
    useControl: (create: () => DrawInstance, add: (ctx: typeof context) => void, remove: (ctx: typeof context) => void) => {
        const [resource] = useState(() => ({control: create(), add, remove}));
        useEffect(() => {
            resource.add(context);
            return () => resource.remove(context);
        }, [resource]);
        return resource.control;
    },
}));

const callbacks = () => ({onCreate: vi.fn(), onUpdate: vi.fn(), onCombine: vi.fn(), onDelete: vi.fn(), onSelectionChange: vi.fn(), onOpenDetails: vi.fn()});
const emit = (event: string) => mock.listeners.get(event)?.forEach(listener => listener({features: []}));

describe('DrawControl listener lifecycle', () => {
    beforeEach(() => { mock.listeners.clear(); mock.controls.length = 0; mock.on.mockClear(); mock.off.mockClear(); vi.useFakeTimers(); });
    afterEach(() => vi.useRealTimers());

    it('uses committed callbacks without rebinding listeners or recreating the control', () => {
        const first = callbacks();
        const next = callbacks();
        const view = render(<DrawControl {...first} editMode/>);
        const registrations = mock.on.mock.calls.length;
        const controls = mock.controls.length;
        view.rerender(<DrawControl {...next} editMode/>);
        act(() => emit('draw.create'));
        expect(first.onCreate).not.toHaveBeenCalled();
        expect(next.onCreate).toHaveBeenCalledOnce();
        expect(mock.on).toHaveBeenCalledTimes(registrations);
        expect(mock.controls).toHaveLength(controls);
        view.unmount();
        expect([...mock.listeners.values()].every(listeners => listeners.size === 0)).toBe(true);
        for (const [event, listener] of mock.on.mock.calls) expect(mock.off).toHaveBeenCalledWith(event, listener);
    });

    it('has one effective draw listener after StrictMode replay and no work after unmount', () => {
        const props = callbacks();
        const view = render(<StrictMode><DrawControl {...props} editMode={false}/></StrictMode>);
        act(() => emit('draw.create'));
        expect(props.onCreate).toHaveBeenCalledOnce();
        expect(mock.listeners.get('draw.create')?.size).toBe(1);
        act(() => emit('draw.selectionchange'));
        expect(props.onSelectionChange).toHaveBeenCalledOnce();
        const mountedControl = mock.controls[mock.controls.length - 1];
        const calls = mountedControl?.changeMode.mock.calls.length;
        view.unmount();
        act(() => { vi.runAllTimers(); });
        expect(mountedControl?.changeMode).toHaveBeenCalledTimes(calls ?? 0);
        expect([...mock.listeners.values()].every(listeners => listeners.size === 0)).toBe(true);
        const remount = render(<DrawControl {...props} editMode/>);
        act(() => emit('draw.create'));
        expect(props.onCreate).toHaveBeenCalledTimes(2);
        remount.unmount();
    });
});


const origin: GeoJSON.Feature = {type: 'Feature', id: 0, properties: null, geometry: {type: 'Point', coordinates: [0, 0]}};
const eventCases = [
    {event: 'draw.create', callback: 'onCreate', payload: {features: [origin]}},
    {event: 'draw.update', callback: 'onUpdate', payload: {features: [origin], action: 'change_coordinates'}},
    {event: 'draw.combine', callback: 'onCombine', payload: {createdFeatures: [origin], deletedFeatures: []}},
    {event: 'draw.delete', callback: 'onDelete', payload: {features: [origin]}},
    {event: 'draw.selectionchange', callback: 'onSelectionChange', payload: {features: []}},
    {event: 'feature.open', callback: 'onOpenDetails', payload: {feature: origin}},
    {event: 'feature.open', callback: 'onOpenDetails', payload: {}},
] satisfies Array<{event: string; callback: keyof ReturnType<typeof callbacks>; payload: object}>;

describe('DrawControl event payload forwarding', () => {
    beforeEach(() => {mock.listeners.clear(); mock.on.mockClear(); mock.off.mockClear(); vi.useFakeTimers();});
    afterEach(() => vi.useRealTimers());
    it.each(eventCases)('forwards $event to the current $callback without coercing its payload', ({event, callback, payload}) => {
        const first = callbacks(), current = callbacks();
        const view = render(<DrawControl {...first} editMode/>);
        const registrations = mock.on.mock.calls.length;
        view.rerender(<DrawControl {...current} editMode/>);
        act(() => mock.listeners.get(event)?.forEach(listener => listener(payload)));
        expect(first[callback]).not.toHaveBeenCalled();
        expect(current[callback]).toHaveBeenCalledExactlyOnceWith(payload);
        expect(current[callback].mock.calls[0][0]).toBe(payload);
        for (const [key, handler] of Object.entries(current)) {
            if (key !== callback) expect(handler).not.toHaveBeenCalled();
        }
        expect(mock.on).toHaveBeenCalledTimes(registrations);
        view.unmount();
        for (const [name, listener] of mock.on.mock.calls) expect(mock.off).toHaveBeenCalledWith(name, listener);
        expect([...mock.listeners.values()].every(listeners => listeners.size === 0)).toBe(true);
    });
});
