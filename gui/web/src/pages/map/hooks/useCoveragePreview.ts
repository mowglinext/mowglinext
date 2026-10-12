import {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import {useApi} from "../../../hooks/useApi.ts";
import type {MapArea} from "../../../types/ros.ts";
import type {MowingAreaFeature, ObstacleFeature} from "../../../types/map.ts";
import {itranspose} from "../../../utils/map.tsx";
import {
    buildPreviewLayers,
    choicesFromArea,
    effectiveAngleDeg,
    emptyLayers,
    overridesFromChoices,
    requestedValues,
    ringToRosPolygon,
    sameChoices,
    type AngleMode,
    type AreaChoices,
    type AreaOverrideFields,
    type CoveragePreviewResult,
    type DirectionChoice,
    type MapPoint,
} from "../coveragePreview.ts";

/** Wait this long after the last change before asking the planner again. */
const DEBOUNCE_MS = 300;

interface Args {
    areas: MowingAreaFeature[];
    obstacles: ObstacleFeature[];
    datum: [number, number, number];
    offsetX: number;
    offsetY: number;
    /** The robot-wide settings: mow_angle_deg (< 0 = auto) and mow_direction (0/1/2). */
    globalAngleDeg: number;
    globalDirection: number;
    /** Area to start on (e.g. the one selected in the editor). */
    preferredAreaId?: string;
    /**
     * The lines can only be changed while the map is being edited. A change is then part of that
     * edit: it goes into the edit session (see onCommit), "Save map" keeps it and "Cancel" drops it.
     */
    editMode: boolean;
    /** Put an area's new lines into the edit session (the page's `features`, so undo/redo and cancel see it). */
    onCommit: (areaFeatureId: string, overrides: Required<AreaOverrideFields>) => void;
    /** The areas as the server has them, to tell whether the session changed any lines. */
    savedAreas?: MapArea[];
}

/// The Map page's "mowing lines" overlay. The lines come from coverage_server's
/// preview_coverage service — the real planner run with the live geometry
/// parameters — so what the operator sees is what the robot will drive. This hook
/// only decides WHAT to ask: one chosen area, with the angle, perimeter direction
/// and start point the operator is trying.
///
/// Viewing is always possible; changing is only possible in edit mode, where it is part
/// of the map edit like moving a polygon corner: nothing is written until "Save map"
/// (which already round-trips these fields) and "Cancel" throws it away. A slider drag
/// or a half-typed number only moves the preview; it is committed to the edit session when
/// the operator lets go, so undo gets one step per change instead of one per pixel.
export const useCoveragePreview = ({
    areas, obstacles, datum, offsetX, offsetY, globalAngleDeg, globalDirection, preferredAreaId,
    editMode, onCommit, savedAreas,
}: Args) => {
    const api = useApi();
    const {t} = useTranslation();

    const [enabled, setEnabled] = useState(false);
    const [areaId, setAreaId] = useState<string | undefined>(preferredAreaId);
    const [draft, setDraft] = useState<Partial<AreaChoices>>({});
    const [result, setResult] = useState<CoveragePreviewResult | undefined>();
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState<string | undefined>();
    // Counts planner answers (or failures) received for the CURRENT request, so the start marker
    // knows when to let go of the spot it was dropped on and show the planner's real start.
    const [settledCount, setSettledCount] = useState(0);
    const requestSeq = useRef(0);

    // Follow the editor's selection until the operator picks an area by hand.
    const pickedByHand = useRef(false);
    useEffect(() => {
        if (!pickedByHand.current && preferredAreaId && areas.some((a) => a.id === preferredAreaId)) {
            setAreaId(preferredAreaId);
        }
    }, [preferredAreaId, areas]);

    const area = useMemo(() => areas.find((a) => a.id === areaId) ?? areas[0], [areas, areaId]);

    // A half-made change belongs to the area and the edit session it was made in: leaving edit
    // mode (Save or Cancel) or switching area drops it. Adjusting state while rendering on a change
    // is React's documented alternative to an effect.
    const draftScope = `${editMode ? "edit" : "view"}:${area?.id ?? ""}`;
    const [prevDraftScope, setPrevDraftScope] = useState(draftScope);
    if (draftScope !== prevDraftScope) {
        setPrevDraftScope(draftScope);
        setDraft({});
    }

    // What the edit session holds for this area right now (the feature carries it once committed).
    const saved = useMemo(() => choicesFromArea(area?.area), [area]);
    const choices: AreaChoices = useMemo(() => ({
        angleMode: draft.angleMode ?? saved.angleMode,
        angleDeg: draft.angleDeg ?? saved.angleDeg,
        direction: draft.direction ?? saved.direction,
        // null is a choice (the planner's own start), so only undefined means "no draft".
        start: draft.start === undefined ? saved.start : draft.start,
    }), [draft, saved]);
    const requested = useMemo(
        () => requestedValues(choices, globalAngleDeg, globalDirection),
        [choices, globalAngleDeg, globalDirection],
    );

    // Do this area's lines differ from what the server has? Then a paused mow resumes on a re-planned
    // area, which the panel warns about.
    const changedFromServer = useMemo(() => {
        const server = savedAreas?.find((a) => a.id !== undefined && a.id === area?.area?.id);
        return !!area?.area && !sameChoices(choicesFromArea(area.area), choicesFromArea(server));
    }, [area, savedAreas]);

    // A content signature, not array identities: the caller rebuilds both lists
    // on every render, so keying the fetch on them would refetch constantly.
    const requestBody = useMemo(() => {
        const ring = area?.geometry.coordinates[0];
        if (!enabled || !ring || ring.length < 3 || datum[0] === 0) return undefined;
        const holes = obstacles
            .filter((o) => o.getMowingArea()?.id === area.id)
            .map((o) => ringToRosPolygon(o.geometry.coordinates[0] ?? [], datum, offsetX, offsetY))
            .filter((p) => p.points.length >= 3);
        return {
            outer_boundary: ringToRosPolygon(ring, datum, offsetX, offsetY),
            obstacles: holes,
            mow_angle_deg: requested.mow_angle_deg,
            ring_direction: requested.ring_direction,
            has_start_point: requested.start !== null,
            start_x: requested.start?.x ?? 0,
            start_y: requested.start?.y ?? 0,
        };
    }, [enabled, area, obstacles, datum, offsetX, offsetY, requested]);
    const signature = useMemo(() => (requestBody ? JSON.stringify(requestBody) : ""), [requestBody]);

    useEffect(() => {
        if (!requestBody) {
            setResult(undefined);
            setError(undefined);
            setLoading(false);
            return;
        }
        const seq = ++requestSeq.current;
        setLoading(true);
        const timer = setTimeout(async () => {
            try {
                const res = await api.mowglinext.callCreate("preview_coverage", requestBody);
                if (seq !== requestSeq.current) return; // a newer request superseded this one
                if (res.error) {
                    setResult(undefined);
                    setError((res.error as {error?: string}).error ?? t("coveragePreview.failed"));
                    return;
                }
                const data = res.data as unknown as CoveragePreviewResult;
                setResult(data);
                setError(data.success ? undefined : (data.message || t("coveragePreview.failed")));
            } catch (e) {
                if (seq !== requestSeq.current) return;
                // coverage_server may not be advertised yet; show the reason, keep the map clean.
                setResult(undefined);
                setError(e instanceof Error ? e.message : t("coveragePreview.failed"));
            } finally {
                if (seq === requestSeq.current) {
                    setLoading(false);
                    setSettledCount((n) => n + 1);
                }
            }
        }, DEBOUNCE_MS);
        return () => clearTimeout(timer);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [signature, api]);

    const layers = useMemo(
        () => (enabled ? buildPreviewLayers(result, datum, offsetX, offsetY) : emptyLayers()),
        [enabled, result, datum, offsetX, offsetY],
    );

    /** The angle to show: the one asked for, or what the planner resolved "auto" to. */
    const shownAngle = effectiveAngleDeg(requested.mow_angle_deg, result?.mow_angle_deg);

    const selectArea = useCallback((id: string) => {
        pickedByHand.current = true;
        setAreaId(id);
    }, []);

    // Put `next` into the edit session and drop the draft: from here the feature carries it.
    const commit = useCallback((next: AreaChoices) => {
        if (!editMode || !area) return;
        setDraft({});
        onCommit(area.id, overridesFromChoices(next));
    }, [editMode, area, onCommit]);

    // Discrete choices commit at once; continuous ones (slider, typing) commit when released.
    const setAngleMode = useCallback((mode: AngleMode) => {
        if (!editMode) return;
        // Switching to "fixed" starts from the angle the plan is using right now.
        const angleDeg = mode === "fixed" ? Math.round(shownAngle) % 180 : choices.angleDeg;
        commit({...choices, angleMode: mode, angleDeg});
    }, [editMode, shownAngle, choices, commit]);
    /** Moves the preview only; `commitAngle` makes it part of the edit. */
    const setAngleDeg = useCallback((deg: number) => {
        if (!editMode) return;
        setDraft((d) => ({...d, angleMode: "fixed", angleDeg: deg}));
    }, [editMode]);
    const commitAngle = useCallback(() => commit(choices), [commit, choices]);
    const setDirection = useCallback((direction: DirectionChoice) => commit({...choices, direction}), [commit, choices]);
    /** Where the route starts, or null for the planner's own start. The planner snaps it onto the outer ring. */
    const setStart = useCallback((start: MapPoint | null) => commit({...choices, start}), [commit, choices]);
    /** The operator dropped the start marker at this spot on the map. */
    const moveStartTo = useCallback((lon: number, lat: number) => {
        const [x, y] = itranspose(offsetX, offsetY, datum, lat, lon);
        setStart({x, y});
    }, [offsetX, offsetY, datum, setStart]);

    return {
        enabled, setEnabled,
        areas, area, selectArea, editMode,
        choices, setAngleMode, setAngleDeg, commitAngle, setDirection, setStart, moveStartTo,
        // Where the route really starts (the planner snaps the chosen point onto the outer ring),
        // and whether a start can be placed at all (not with the headland rings off).
        startLonLat: layers.startLonLat,
        outerRingLonLat: layers.outerRingLonLat,
        settledCount,
        startAdjustable: result?.start_adjustable !== false,
        robotWideAngle: globalAngleDeg, robotWideDirection: globalDirection, shownAngle,
        changedFromServer,
        loading, error, result, layers,
    };
};
