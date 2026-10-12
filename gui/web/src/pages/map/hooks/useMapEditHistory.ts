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
    // Called whenever an edit session is genuinely ABANDONED (either branch
    // of exitEditMode — no unsaved area/obstacle changes to begin with, or
    // the operator confirmed "discard"). NOT called on a successful Save
    // Map, which sets editMap false directly without going through here.
    // MapPage uses this to revert LiDAR-ignore corridors, which (unlike
    // areas/obstacles) are written to map_server immediately on every edit
    // rather than staying buffered in `features` until Save — see
    // useLidarCorridors's own doc comment.
    onDiscard?: () => void;
    // Additional "there are unsaved changes" signal from outside this hook
    // (area/obstacle edits are tracked internally via the `features` watcher
    // below; this covers a corridor-only edit, which never touches
    // `features` and so would otherwise silently skip the confirm dialog).
    extraUnsavedChanges?: boolean;
    // Runs before edit mode opens; the editor opens only when it resolves true
    // (MapPage uses it to take a server-side copy of the map first). Absent =
    // open at once, as before.
    beforeEdit?: () => Promise<boolean>;
}

export function useMapEditHistory({features, setFeatures, editMap, setEditMap, onDiscard, extraUnsavedChanges, beforeEdit}: UseMapEditHistoryOptions) {
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
    // The map may update while beforeEdit runs: open the editor on the LATEST features.
    const featuresRef = useRef(features);
    featuresRef.current = features;
    const openingRef = useRef(false);

    function exitEditMode() {
        setEditHistory([]);
        setHistoryIndex(-1);
        setHasUnsavedChanges(false);
        lastRecordedRef.current = null;
        setEditMap(false);
        onDiscard?.();
    }

    function enterEditMode() {
        const current = featuresRef.current;
        setEditHistory([serializeFeatures(current)]);
        setHistoryIndex(0);
        lastRecordedRef.current = current;
        setEditMap(true);
    }

    function handleEditMap() {
        if (!editMap) {
            if (!beforeEdit) {
                enterEditMode();
                return;
            }
            if (openingRef.current) return; // a second click while the copy is being made
            openingRef.current = true;
            void beforeEdit()
                .then((ok) => { if (ok) enterEditMode(); })
                .finally(() => { openingRef.current = false; });
        } else if (hasUnsavedChanges || extraUnsavedChanges) {
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
