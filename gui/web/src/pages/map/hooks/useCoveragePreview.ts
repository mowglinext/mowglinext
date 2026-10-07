import {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {App} from "antd";
import {useTranslation} from "react-i18next";
import {useApi} from "../../../hooks/useApi.ts";
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
    /** false while the mower is mowing or its state is unknown: an area's lines are then read-only. */
    canEdit: boolean;
}

/// The Map page's "mowing lines" overlay. The lines come from coverage_server's
/// preview_coverage service — the real planner run with the live geometry
/// parameters — so what the operator sees is what the robot will drive. This hook
/// only decides WHAT to ask: one chosen area, with the angle and perimeter
/// direction the operator is trying. Each area either follows the robot-wide
/// settings or carries its own; saving writes that area through
/// set_area_coverage_lines, and "use for all areas" writes the robot-wide ones.
export const useCoveragePreview = ({
    areas, obstacles, datum, offsetX, offsetY, globalAngleDeg, globalDirection, preferredAreaId, canEdit,
}: Args) => {
    const api = useApi();
    const {notification} = App.useApp();
    const {t} = useTranslation();

    const [enabled, setEnabled] = useState(false);
    const [areaId, setAreaId] = useState<string | undefined>(preferredAreaId);
    const [draft, setDraft] = useState<Partial<AreaChoices>>({});
    // The robot-wide values after a save from here; null = whatever the page loaded.
    const [globalOverride, setGlobalOverride] = useState<{angle: number; direction: number} | null>(null);
    // The area's overrides are mutated in place after a save (so a later map save
    // writes them back); this makes React notice.
    const [areaVersion, setAreaVersion] = useState(0);
    const [result, setResult] = useState<CoveragePreviewResult | undefined>();
    const [loading, setLoading] = useState(false);
    const [saving, setSaving] = useState(false);
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
    const mapAreaId = area?.area?.id ?? 0;

    // A draft belongs to the area it was made on.
    useEffect(() => { setDraft({}); }, [area?.id]);

    const robotWideAngle = globalOverride?.angle ?? globalAngleDeg;
    const robotWideDirection = globalOverride?.direction ?? globalDirection;

    // eslint-disable-next-line react-hooks/exhaustive-deps
    const saved = useMemo(() => choicesFromArea(area?.area), [area, areaVersion]);
    const choices: AreaChoices = useMemo(() => ({
        angleMode: draft.angleMode ?? saved.angleMode,
        angleDeg: draft.angleDeg ?? saved.angleDeg,
        direction: draft.direction ?? saved.direction,
        // null is a choice (the planner's own start), so only undefined means "no draft".
        start: draft.start === undefined ? saved.start : draft.start,
    }), [draft, saved]);
    const dirty = !sameChoices(choices, saved);
    const requested = useMemo(
        () => requestedValues(choices, robotWideAngle, robotWideDirection),
        [choices, robotWideAngle, robotWideDirection],
    );
    const differsFromRobotWide = requested.mow_angle_deg !== robotWideAngle
        || requested.ring_direction !== (robotWideDirection === 1 || robotWideDirection === 2 ? robotWideDirection : 0);

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

    const setAngleMode = useCallback((mode: AngleMode) => {
        // Switching to "fixed" starts from the angle the plan is using right now.
        setDraft((d) => ({
            ...d,
            angleMode: mode,
            ...(mode === "fixed" ? {angleDeg: Math.round(shownAngle) % 180} : {}),
        }));
    }, [shownAngle]);
    const setAngleDeg = useCallback((deg: number) => setDraft((d) => ({...d, angleMode: "fixed", angleDeg: deg})), []);
    const setDirection = useCallback((direction: DirectionChoice) => setDraft((d) => ({...d, direction})), []);
    /** Where the route starts, or null for the planner's own start. The planner snaps it onto the outer ring. */
    const setStart = useCallback((start: MapPoint | null) => setDraft((d) => ({...d, start})), []);
    /** The operator dropped the start marker at this spot on the map. */
    const moveStartTo = useCallback((lon: number, lat: number) => {
        const [x, y] = itranspose(offsetX, offsetY, datum, lat, lon);
        setStart({x, y});
    }, [offsetX, offsetY, datum, setStart]);
    const reset = useCallback(() => setDraft({}), []);

    // callCreate hands back {error: "reason"} for an application error and a
    // rejected Response ({error: {error: "reason"}}) for a transport one.
    const failure = (e: unknown) => {
        const body = (e as {error?: unknown} | null)?.error;
        notification.error({
            message: t("coveragePreview.saveFailed"),
            description: typeof body === "string"
                ? body
                : (body as {error?: string} | undefined)?.error ?? (e instanceof Error ? e.message : undefined),
        });
    };

    /** Persist this area's own angle / direction (or clear them): applies from its next plan. */
    const saveArea = useCallback(async () => {
        if (!area?.area || !mapAreaId) return;
        setSaving(true);
        try {
            const overrides = overridesFromChoices(choices);
            const res = await api.mowglinext.callCreate("set_area_coverage_lines", {id: mapAreaId, ...overrides});
            if (res.error) {
                failure(res.error);
                return;
            }
            // Keep the loaded map entry in step: a later "save map" writes these
            // fields back, and the 5 s map poll may not have caught up yet.
            Object.assign(area.area, overrides);
            setAreaVersion((v) => v + 1);
            setDraft({});
            notification.success({
                message: t("coveragePreview.savedArea"),
                description: t("coveragePreview.savedAreaDescription"),
            });
        } catch (e) {
            failure(e);
        } finally {
            setSaving(false);
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [api, area, mapAreaId, choices, notification, t]);

    /** Make what is shown the robot-wide default for every area without its own. Needs a ROS2 restart. */
    const saveRobotWide = useCallback(async () => {
        setSaving(true);
        try {
            const angle = requested.mow_angle_deg;
            const direction = requested.ring_direction;
            const res = await api.settings.yamlCreate({mow_angle_deg: angle, mow_direction: direction});
            if (res.error) {
                failure(res.error);
                return;
            }
            setGlobalOverride({angle, direction});
            setDraft({angleMode: "global", direction: "global"});
            notification.success({
                message: t("coveragePreview.savedRobotWide"),
                description: t("coveragePreview.savedRobotWideRestart"),
            });
        } catch (e) {
            failure(e);
        } finally {
            setSaving(false);
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [api, requested, notification, t]);

    return {
        enabled, setEnabled,
        areas, area, selectArea, hasAreaId: mapAreaId !== 0, canEdit,
        choices, setAngleMode, setAngleDeg, setDirection, setStart, moveStartTo,
        // Where the route really starts (the planner snaps the chosen point onto the outer ring),
        // and whether a start can be placed at all (not with the headland rings off).
        startLonLat: layers.startLonLat,
        outerRingLonLat: layers.outerRingLonLat,
        settledCount,
        startAdjustable: result?.start_adjustable !== false,
        robotWideAngle, robotWideDirection, shownAngle,
        dirty, differsFromRobotWide, reset, saveArea, saveRobotWide, saving,
        loading, error, result, layers,
    };
};
