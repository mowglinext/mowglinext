import {useCallback, useEffect, useRef, useState} from "react";
import {App} from "antd";
import {useTranslation} from "react-i18next";
import {
    featuresFromJSON,
    serializeFeatures,
    type MowingFeature,
    type SerializedMapFeature,
} from "../../../types/map.ts";

const MAX_HISTORY_ENTRIES = 10;

interface UseMapEditHistoryOptions {
    features: Record<string, MowingFeature>;
    setFeatures: (features: Record<string, MowingFeature>) => void;
    editMap: boolean;
    setEditMap: (v: boolean) => void;
}

export function useMapEditHistory({features, setFeatures, editMap, setEditMap}: UseMapEditHistoryOptions) {
    const {modal} = App.useApp();
    const {t} = useTranslation();
    // History entries are PLAIN serialized snapshots, never the class
    // instances themselves: structuredClone strips prototypes, so cloning
    // MowingAreaFeature/ObstacleFeature/... instances would make every
    // instanceof fail after undo (polygons vanish + Save persists an EMPTY
    // map). serializeFeatures/featuresFromJSON round-trip class identity,
    // ids, properties and obstacle→area parent links.
    const [editHistory, setEditHistory] = useState<Record<string, SerializedMapFeature>[]>([]);
    const [historyIndex, setHistoryIndex] = useState(-1);
    const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
    // Identity of the features object whose state is already recorded in
    // history (initial snapshot, or the object we just restored via
    // undo/redo). The watcher effect skips it so entering edit mode leaves
    // history at length 1 with no dirty flag, and undo/redo don't re-push.
    const lastRecordedRef = useRef<Record<string, MowingFeature> | null>(null);

    function exitEditMode() {
        setEditHistory([]);
        setHistoryIndex(-1);
        setHasUnsavedChanges(false);
        lastRecordedRef.current = null;
        setEditMap(false);
    }

    function handleEditMap() {
        if (!editMap) {
            setEditHistory([serializeFeatures(features)]);
            setHistoryIndex(0);
            lastRecordedRef.current = features;
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
        // Snapshot serialized data so subsequent in-place edits to the live
        // features can't retroactively mutate this history entry.
        const snapshot = serializeFeatures(newFeatures);
        setEditHistory(prev => {
            const truncated = prev.slice(0, historyIndex + 1);
            const next = [...truncated, snapshot].slice(-MAX_HISTORY_ENTRIES);
            setHistoryIndex(next.length - 1);
            return next;
        });
    }, [historyIndex]);

    // Track feature changes during edit mode
    useEffect(() => {
        if (!editMap) return;
        if (lastRecordedRef.current === features) return;
        lastRecordedRef.current = features;
        pushHistory(features);
        setHasUnsavedChanges(true);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [features, editMap]);

    const handleUndo = useCallback(() => {
        if (historyIndex <= 0) return;
        const newIndex = historyIndex - 1;
        setHistoryIndex(newIndex);
        // Rehydrate real class instances from the plain snapshot; the stored
        // entry itself stays untouched so redo / a later undo can reuse it.
        const restored = featuresFromJSON(editHistory[newIndex]);
        lastRecordedRef.current = restored;
        setFeatures(restored);
    }, [historyIndex, editHistory, setFeatures]);

    const handleRedo = useCallback(() => {
        if (historyIndex >= editHistory.length - 1) return;
        const newIndex = historyIndex + 1;
        setHistoryIndex(newIndex);
        const restored = featuresFromJSON(editHistory[newIndex]);
        lastRecordedRef.current = restored;
        setFeatures(restored);
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
