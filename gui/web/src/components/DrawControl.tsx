import MapboxDraw from '@mapbox/mapbox-gl-draw';
import type {ControlPosition} from 'mapbox-gl';
import {useControl} from 'react-map-gl/mapbox';
import type {MapRef} from 'react-map-gl/mapbox';
import {useEffect, useRef} from "react";
import type {RefObject} from "react";
import DirectSelectWithBoxMode from '../modes/DirectSelectWithBoxMode';
import SplitLineMode from '../modes/SplitLineMode';

type DrawControlProps = ConstructorParameters<typeof MapboxDraw>[0] & {
    position?: ControlPosition;
    features?: GeoJSON.Feature[];
    editMode?: boolean;
    drawRef?: RefObject<MapboxDraw | null>;

    onCreate: (evt: { features: GeoJSON.Feature[] }) => void;
    onUpdate: (evt: { features: GeoJSON.Feature[]; action: string }) => void;
    onCombine: (evt: { createdFeatures: GeoJSON.Feature[]; deletedFeatures: GeoJSON.Feature[] }) => void;
    onDelete: (evt: { features: GeoJSON.Feature[] }) => void;
    onSelectionChange: (evt: { features: GeoJSON.Feature[] }) => void;
    onOpenDetails: (evt: { feature?: GeoJSON.Feature }) => void;
};

export default function DrawControl(props: DrawControlProps) {
    const {
        drawRef, features, editMode, position,
        onCreate = () => {},
        onUpdate = () => {},
        onCombine = () => {},
        onDelete = () => {},
        onSelectionChange = () => {},
        onOpenDetails = () => {},
        ...drawOptions
    } = props;
    const rawMapRef = useRef<ReturnType<MapRef['getMap']> | null>(null);
    const editModeRef = useRef(editMode);
    editModeRef.current = editMode;

    // Use refs for all callbacks so event listeners always call the latest version.
    // useControl only binds listeners once during setup — without refs, stale closures
    // would be called when the callbacks change (e.g. when splitTargetId updates).
    const onCreateRef = useRef(onCreate);
    onCreateRef.current = onCreate;
    const onUpdateRef = useRef(onUpdate);
    onUpdateRef.current = onUpdate;
    const onCombineRef = useRef(onCombine);
    onCombineRef.current = onCombine;
    const onDeleteRef = useRef(onDelete);
    onDeleteRef.current = onDelete;
    const onSelectionChangeRef = useRef(onSelectionChange);
    onSelectionChangeRef.current = onSelectionChange;
    const onOpenDetailsRef = useRef(onOpenDetails);
    onOpenDetailsRef.current = onOpenDetails;

    // Keep stable references to the bound listeners so cleanup can map.off()
    // exactly what was map.on()'d. Without this, StrictMode's mount/unmount/
    // remount cycle stacks duplicate listeners → each draw.* event fires twice.
    const drawHandlersRef = useRef<Record<string, (e: any) => void>>({});
    const mp = useControl<MapboxDraw>(
        () => new MapboxDraw({
            ...drawOptions,
            modes: {
                ...MapboxDraw.modes,
                direct_select: DirectSelectWithBoxMode,
                split_line: SplitLineMode,
            }
        }),
        ({map}: {map: MapRef}) => {
            rawMapRef.current = map.getMap();
            const handlers: Record<string, (e: any) => void> = {
                'draw.create': (e: any) => onCreateRef.current(e),
                'draw.update': (e: any) => onUpdateRef.current(e),
                'draw.combine': (e: any) => onCombineRef.current(e),
                'draw.delete': (e: any) => onDeleteRef.current(e),
                'draw.selectionchange': (e: any) => onSelectionChangeRef.current(e),
                'feature.open': (e: any) => onOpenDetailsRef.current(e),
            };
            drawHandlersRef.current = handlers;
            for (const [event, handler] of Object.entries(handlers)) {
                map.on(event as any, handler);
            }
        },
        ({map}: {map: MapRef}) => {
            for (const [event, handler] of Object.entries(drawHandlersRef.current)) {
                map.off(event as any, handler);
            }
            drawHandlersRef.current = {};
            rawMapRef.current = null;
        }
        ,
        {
            position,
        }
    );
    useEffect(() => {
        if (drawRef) {
            drawRef.current = mp ?? null;
        }
    }, [mp, drawRef]);
    // Sync features into MapboxDraw whenever they change.
    // Uses a delayed sync to handle React StrictMode's mount/unmount/remount cycle,
    // which causes useControl to remove and re-add the control (wiping its internal store).
    // By deferring, we ensure we write to the final, mounted instance.
    const syncTimerRef = useRef<ReturnType<typeof setTimeout>>(undefined);
    // The SET key (ids + types only, NOT geometry) — a change here means
    // features were added/removed and the store must be rebuilt (deleteAll +
    // re-add). Geometry-only edits (a vertex drag) leave the set key unchanged
    // and are upserted per-feature via mp.add() WITHOUT clearing the store, so
    // the active direct_select session (and the user's selection) survives.
    const prevSetKeyRef = useRef<string>('');
    useEffect(() => {
        if (!mp || !features) return;
        clearTimeout(syncTimerRef.current);
        // Use 300ms instead of 0ms: on mobile the Mapbox GL map may still be
        // initialising tiles/layers when React StrictMode or a mapKey remount
        // fires this effect, and a 0ms flush races against the GL context setup.
        // 300ms outlasts the GL bootstrap without noticeably delaying first paint.
        syncTimerRef.current = setTimeout(() => {
            const setKey = JSON.stringify(features.map(f => [f.id, f.geometry?.type]));
            const storeEmpty = mp.getAll().features.length === 0;
            if (setKey !== prevSetKeyRef.current || storeEmpty) {
                // Feature set changed (or the store was wiped by a remount):
                // rebuild it wholesale. This DOES clear selection, but a
                // set-level change is a create/delete, not an in-place edit.
                prevSetKeyRef.current = setKey;
                mp.deleteAll();
                features.forEach((f) => mp.add(f));
            } else {
                // Geometry-only change: upsert each feature in place. mp.add()
                // replaces a feature with the same id without touching the
                // selection, so a mid-drag direct_select session is preserved.
                features.forEach((f) => mp.add(f));
            }
        }, 300);
        return () => clearTimeout(syncTimerRef.current);
    }, [mp, features]);
    useEffect(() => {
        if (!mp) return;
        mp.changeMode('simple_select');
        if (!editMode) {
            // Deselect everything so features can't be dragged
            mp.changeMode('simple_select', { featureIds: [] });
        }
    }, [mp, editMode]);

    // When not in edit mode, intercept selection and immediately deselect
    // to prevent users from dragging features.
    useEffect(() => {
        if (!mp) return;
        const rawMap = rawMapRef.current;
        if (!rawMap) return;

        const blockSelection = () => {
            if (!editModeRef.current) {
                // Use setTimeout to avoid re-entrant mode changes during event dispatch
                setTimeout(() => {
                    mp.changeMode('simple_select', { featureIds: [] });
                }, 0);
            }
        };
        rawMap.on('draw.selectionchange', blockSelection);
        return () => {
            rawMap.off('draw.selectionchange', blockSelection);
        };
    }, [mp]);
    return null;
}
