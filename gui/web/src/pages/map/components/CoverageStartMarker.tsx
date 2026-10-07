import {memo, useState} from "react";
import {Marker} from "react-map-gl/mapbox";
import {nearestOnPolyline} from "../coveragePreview.ts";

interface CoverageStartMarkerProps {
    /** Where the planner says the route starts. */
    longitude: number;
    latitude: number;
    /** The outermost headland ring as a closed [lon, lat] line; the marker slides along it. */
    ring: [number, number][] | null;
    /** Counts planner answers, so the marker knows when its dropped spot is superseded. */
    settledCount: number;
    /** The operator dropped the marker here (already on the ring). */
    onMove: (longitude: number, latitude: number) => void;
    /** Tooltip, which also tells the operator it can be dragged. */
    title: string;
}

type LngLat = [number, number];

/// Where the route starts, as a marker the operator can drag.
///
///  * While dragging it SLIDES ALONG the outermost ring (the route always starts there), so
///    the operator sees at once where it will end up instead of a dot floating over the lawn.
///  * On drop it STAYS where it was dropped until the planner answers. Without that it
///    jumped back to the old start for the length of the round trip, then to the new one.
///  * The planner has the last word: it keeps the start on a straight side, clear of the
///    corners, and answers with the real start, which the marker then moves to.
/// Move is reported on DROP only, so the planner is asked once per placement, not per pixel.
const CoverageStartMarkerImpl = ({
    longitude, latitude, ring, settledCount, onMove, title,
}: CoverageStartMarkerProps) => {
    // Where the marker was dropped, until the planner's answer (or a new start) replaces it.
    const [dropped, setDropped] = useState<{at: LngLat; settledAtDrop: number; reported: LngLat} | null>(null);
    // Where the marker is while it is being dragged. react-map-gl's Marker puts the marker back
    // at its `longitude`/`latitude` props on EVERY render, and this page re-renders constantly
    // (robot pose, map streams): without this the dot snapped back to the old start in the middle
    // of a drag and then to the pointer again, which looked like stuttering. Rendering from this
    // keeps the marker where the drag put it, whatever else re-renders it.
    const [live, setLive] = useState<LngLat | null>(null);

    // The dropped spot only counts until the planner has answered or the start has moved on. Decided
    // while rendering (it only ever becomes invalid: the answer counter just goes up), so there is no
    // effect that has to clear it.
    const held = dropped
        && settledCount <= dropped.settledAtDrop
        && dropped.reported[0] === longitude
        && dropped.reported[1] === latitude
        ? dropped
        : null;

    const snap = (lngLat: LngLat): LngLat => (ring && ring.length > 1 ? nearestOnPolyline(ring, lngLat) : lngLat);

    const shown = live ?? held?.at ?? [longitude, latitude];

    return (
        <Marker
            longitude={shown[0]}
            latitude={shown[1]}
            anchor="center"
            draggable
            onDragStart={() => setLive([shown[0], shown[1]])}
            onDrag={(event) => {
                // Keep the marker on the ring while it is being dragged. The marker is set
                // directly as well, so it follows at once without waiting for a render.
                const on = snap([event.lngLat.lng, event.lngLat.lat]);
                setLive(on);
                event.target.setLngLat(on);
            }}
            onDragEnd={(event) => {
                const on = snap([event.lngLat.lng, event.lngLat.lat]);
                setLive(null);
                setDropped({at: on, settledAtDrop: settledCount, reported: [longitude, latitude]});
                onMove(on[0], on[1]);
            }}
            style={{zIndex: 5, cursor: "grab"}}
        >
            {/* 28 px: a green dot with a generous, touch-sized hit area. */}
            <div
                title={title}
                aria-label={title}
                role="img"
                style={{
                    width: 28,
                    height: 28,
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                }}
            >
                <div
                    style={{
                        width: 16,
                        height: 16,
                        borderRadius: "50%",
                        background: "#34c759",
                        border: "3px solid #ffffff",
                        boxShadow: "0 0 0 1px rgba(0,0,0,0.45), 0 1px 4px rgba(0,0,0,0.5)",
                    }}
                />
            </div>
        </Marker>
    );
};

// The page re-renders many times a second. The marker only has to render when its inputs change
// (a render during a drag is harmless thanks to `live`, but pointless work). onMove is compared
// too: it converts the dropped point with the current map offset, so a stale one would place the
// start wrongly.
export const CoverageStartMarker = memo(
    CoverageStartMarkerImpl,
    (a, b) => a.longitude === b.longitude
        && a.latitude === b.latitude
        && a.ring === b.ring
        && a.settledCount === b.settledCount
        && a.onMove === b.onMove
        && a.title === b.title,
);
