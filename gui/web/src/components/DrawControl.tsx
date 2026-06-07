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
        onCreate, onUpdate, onCombine, onDelete, onSelectionChange, onOpenDetails,
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
            map.on('draw.create', (e: any) => onCreateRef.current(e));
            map.on('draw.update', (e: any) => onUpdateRef.current(e));
            map.on('draw.combine', (e: any) => onCombineRef.current(e));
            map.on('draw.delete', (e: any) => onDeleteRef.current(e));
            map.on('draw.selectionchange', (e: any) => onSelectionChangeRef.current(e));
            map.on('feature.open', (e: any) => onOpenDetailsRef.current(e));
        },
        ({map: _map}: {map: MapRef}) => {
            rawMapRef.current = null;
            void _map; // cleanup only runs on unmount
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
    const prevFeaturesKeyRef = useRef<string>('');
    useEffect(() => {
        if (!mp || !features) return;
        clearTimeout(syncTimerRef.current);
        // Use 300ms instead of 0ms: on mobile the Mapbox GL map may still be
        // initialising tiles/layers when React StrictMode or a mapKey remount
        // fires this effect, and a 0ms flush races against the GL context setup.
        // 300ms outlasts the GL bootstrap without noticeably delaying first paint.
        syncTimerRef.current = setTimeout(() => {
            const key = JSON.stringify(features.map(f => [f.id, f.geometry]));
            if (key === prevFeaturesKeyRef.current && mp.getAll().features.length > 0) return;
            prevFeaturesKeyRef.current = key;
            mp.deleteAll();
            features.forEach((f) => {
                mp.add(f);
            });
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

DrawControl.defaultProps = {
    onCreate: () => {},
    onUpdate: () => {},
    onDelete: () => {},
    onCombine: () => {},
    onSelectionChange: () => {},
    onOpenDetails: () => {},
};
