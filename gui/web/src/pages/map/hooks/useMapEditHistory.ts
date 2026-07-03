import {useCallback, useEffect, useRef, useState} from "react";
import {App} from "antd";
import {useTranslation} from "react-i18next";
import type {MowingFeature} from "../../../types/map.ts";

interface UseMapEditHistoryOptions {
    features: Record<string, MowingFeature>;
    setFeatures: (features: Record<string, MowingFeature>) => void;
    editMap: boolean;
    setEditMap: (v: boolean) => void;
}

export function useMapEditHistory({features, setFeatures, editMap, setEditMap}: UseMapEditHistoryOptions) {
    const {modal} = App.useApp();
    const {t} = useTranslation();
    const [editHistory, setEditHistory] = useState<Record<string, MowingFeature>[]>([]);
    const [historyIndex, setHistoryIndex] = useState(-1);
    const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
    const skipHistoryRef = useRef(false);

    function exitEditMode() {
        setEditHistory([]);
        setHistoryIndex(-1);
        setHasUnsavedChanges(false);
        setEditMap(false);
    }

    function handleEditMap() {
        if (!editMap) {
            // Deep-clone: a shallow {...features} shares the inner MowingFeature
            // objects, so later in-place geometry/name edits mutate the stored
            // snapshot and undo can't revert.
            setEditHistory([structuredClone(features)]);
            setHistoryIndex(0);
            setEditMap(true);
        } else if (hasUnsavedChanges) {
            modal.confirm({
                title: t('mapEditHistory.discardTitle'),
                content: t('mapEditHistory.discardContent'),
                okText: t('mapEditHistory.discardOk'),
                okType: 'danger',
                cancelText: t('mapEditHistory.keepEditing'),
                onOk: exitEditMode,
            });
        } else {
            exitEditMode();
        }
    }

    const pushHistory = useCallback((newFeatures: Record<string, MowingFeature>) => {
        // Snapshot a deep clone so subsequent in-place edits to the live
        // features don't retroactively mutate this history entry.
        const snapshot = structuredClone(newFeatures);
        setEditHistory(prev => {
            const truncated = prev.slice(0, historyIndex + 1);
            const next = [...truncated, snapshot].slice(-10);
            setHistoryIndex(next.length - 1);
            return next;
        });
    }, [historyIndex]);

    // Track feature changes during edit mode
    useEffect(() => {
        if (!editMap) return;
        if (skipHistoryRef.current) {
            skipHistoryRef.current = false;
            return;
        }
        if (editHistory.length > 0 && editHistory[historyIndex] === features) return;
        pushHistory(features);
        setHasUnsavedChanges(true);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [features, editMap]);

    const handleUndo = useCallback(() => {
        if (historyIndex <= 0) return;
        const newIndex = historyIndex - 1;
        setHistoryIndex(newIndex);
        skipHistoryRef.current = true;
        // Clone on the way out so editing the restored state doesn't mutate
        // the stored snapshot (which redo / a later undo would reuse).
        setFeatures(structuredClone(editHistory[newIndex]));
    }, [historyIndex, editHistory, setFeatures]);

    const handleRedo = useCallback(() => {
        if (historyIndex >= editHistory.length - 1) return;
        const newIndex = historyIndex + 1;
        setHistoryIndex(newIndex);
        skipHistoryRef.current = true;
        setFeatures(structuredClone(editHistory[newIndex]));
    }, [historyIndex, editHistory, setFeatures]);

    return {
        hasUnsavedChanges,
        setHasUnsavedChanges,
        handleEditMap,
        exitEditMode,
        handleUndo,
        handleRedo,
        historyIndex,
        editHistory,
    };
}
