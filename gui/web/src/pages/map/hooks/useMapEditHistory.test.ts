import {describe, it, expect, vi, beforeEach} from 'vitest';
import {renderHook, act} from '@testing-library/react';
import {useMapEditHistory} from './useMapEditHistory.ts';
import type {MowingFeature} from '../../../types/map.ts';

// Mock antd App.useApp to provide modal.confirm
const mockConfirm = vi.fn(({onOk}: {onOk: () => void}) => onOk());
vi.mock('antd', async () => {
    const actual = await vi.importActual('antd');
    return {
        ...actual,
        App: {
            ...((actual as Record<string, unknown>).App as Record<string, unknown>),
            useApp: () => ({
                notification: {},
                message: {},
                modal: {confirm: mockConfirm},
            }),
        },
    };
});

describe('useMapEditHistory', () => {
    let features: Record<string, MowingFeature>;
    let setFeatures: (features: Record<string, MowingFeature> | ((prev: Record<string, MowingFeature>) => Record<string, MowingFeature>)) => void;
    let editMap: boolean;
    let setEditMap: (v: boolean) => void;

    beforeEach(() => {
        features = {};
        setFeatures = vi.fn((updater) => {
            if (typeof updater === 'function') {
                features = updater(features);
            } else {
                features = updater;
            }
        });
        editMap = false;
        setEditMap = vi.fn((val) => {
            editMap = val;
        });
    });

    function renderHistory() {
        return renderHook(() =>
            useMapEditHistory({features, setFeatures, editMap, setEditMap})
        );
    }

    it('initializes with no unsaved changes', () => {
        const {result} = renderHistory();
        expect(result.current.hasUnsavedChanges).toBe(false);
        expect(result.current.historyIndex).toBe(-1);
    });

    it('handleEditMap toggles edit mode on', () => {
        const {result} = renderHistory();
        act(() => {
            result.current.handleEditMap();
        });
        expect(setEditMap).toHaveBeenCalledWith(true);
    });

    describe('beforeEdit (the map backup taken before the editor opens)', () => {
        function renderWith(beforeEdit: () => Promise<boolean>) {
            return renderHook(() =>
                useMapEditHistory({features, setFeatures, editMap, setEditMap, beforeEdit})
            );
        }

        it('opens the editor only after the backup resolved true', async () => {
            let finish!: (ok: boolean) => void;
            const beforeEdit = vi.fn(() => new Promise<boolean>((resolve) => { finish = resolve; }));
            const {result} = renderWith(beforeEdit);

            act(() => { result.current.handleEditMap(); });
            expect(beforeEdit).toHaveBeenCalledOnce();
            expect(setEditMap).not.toHaveBeenCalled(); // not before the copy exists

            await act(() => Promise.resolve(finish(true)));
            expect(setEditMap).toHaveBeenCalledWith(true);
        });

        it('keeps the editor closed when the backup failed', async () => {
            const {result} = renderWith(vi.fn(() => Promise.resolve(false)));

            await act(() => Promise.resolve(result.current.handleEditMap()));
            expect(setEditMap).not.toHaveBeenCalled();
        });

        it('ignores a second click while the backup is still being made', async () => {
            let finish!: (ok: boolean) => void;
            const beforeEdit = vi.fn(() => new Promise<boolean>((resolve) => { finish = resolve; }));
            const {result} = renderWith(beforeEdit);

            act(() => { result.current.handleEditMap(); });
            act(() => { result.current.handleEditMap(); });
            expect(beforeEdit).toHaveBeenCalledOnce();

            await act(() => Promise.resolve(finish(true)));
            expect(setEditMap).toHaveBeenCalledTimes(1);
        });
    });

    it('setHasUnsavedChanges updates state', () => {
        const {result} = renderHistory();
        act(() => {
            result.current.setHasUnsavedChanges(true);
        });
        expect(result.current.hasUnsavedChanges).toBe(true);
    });

    it('calls onDiscard when cancelling with nothing to confirm', () => {
        const onDiscard = vi.fn();
        editMap = true;
        const {result} = renderHook(() =>
            useMapEditHistory({features, setFeatures, editMap, setEditMap, onDiscard})
        );
        act(() => {
            result.current.handleEditMap();
        });
        expect(setEditMap).toHaveBeenCalledWith(false);
        expect(onDiscard).toHaveBeenCalledTimes(1);
    });

    it('extraUnsavedChanges (a corridor-only edit) still prompts to confirm, and onDiscard fires on confirm', () => {
        const onDiscard = vi.fn();
        editMap = true;
        const {result} = renderHook(() =>
            useMapEditHistory({features, setFeatures, editMap, setEditMap, onDiscard, extraUnsavedChanges: true})
        );
        act(() => {
            result.current.handleEditMap();
        });
        expect(mockConfirm).toHaveBeenCalled();
        expect(setEditMap).toHaveBeenCalledWith(false);
        expect(onDiscard).toHaveBeenCalledTimes(1);
    });
});
