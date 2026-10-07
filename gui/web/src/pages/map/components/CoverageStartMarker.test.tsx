import React from "react";
import {act, render, screen} from "@testing-library/react";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {CoverageStartMarker} from "./CoverageStartMarker.tsx";

type DragEvent = {lngLat: {lng: number; lat: number}; target: {setLngLat: (p: [number, number]) => void}};
type MarkerProps = {
    children: React.ReactNode;
    longitude: number;
    latitude: number;
    draggable?: boolean;
    anchor?: string;
    onDragStart?: () => void;
    onDrag?: (event: DragEvent) => void;
    onDragEnd?: (event: DragEvent) => void;
};

const seen = vi.hoisted(() => ({props: null as null | MarkerProps, renders: 0}));

vi.mock("react-map-gl/mapbox", () => ({
    Marker: (props: MarkerProps) => {
        seen.props = props;
        seen.renders += 1;
        return <div data-testid="marker">{props.children}</div>;
    },
}));

// A 100 m-ish east-west line at lat 52: lon 5.000 -> 5.0015.
const ring: [number, number][] = [[5.0, 52.0], [5.0015, 52.0]];

const drag = (lng: number, lat: number) => {
    const setLngLat = vi.fn();
    act(() => seen.props?.onDrag?.({lngLat: {lng, lat}, target: {setLngLat}}));
    return setLngLat;
};
const drop = (lng: number, lat: number) => act(() => seen.props?.onDragEnd?.({lngLat: {lng, lat}, target: {setLngLat: vi.fn()}}));

const renderMarker = (over: Partial<React.ComponentProps<typeof CoverageStartMarker>> = {}) => {
    const onMove = vi.fn();
    const props = {
        longitude: 5.0005, latitude: 52.0, ring, settledCount: 0, onMove, title: "Drag to move the start", ...over,
    };
    const utils = render(<CoverageStartMarker {...props}/>);
    return {onMove, props, ...utils, again: (next: Partial<typeof props>) =>
        utils.rerender(<CoverageStartMarker {...props} {...next}/>)};
};

describe("coverage start marker", () => {
    beforeEach(() => {
        seen.props = null;
        seen.renders = 0;
    });

    it("is a draggable marker at the reported start", () => {
        renderMarker();
        expect(seen.props?.draggable).toBe(true);
        expect(seen.props?.longitude).toBe(5.0005);
        expect(seen.props?.latitude).toBe(52.0);
        expect(seen.props?.anchor).toBe("center");
    });

    it("tells the operator it can be dragged", () => {
        renderMarker();
        expect(screen.getByRole("img", {name: "Drag to move the start"})).toBeInTheDocument();
    });

    it("slides along the ring while it is dragged", () => {
        renderMarker();
        // The pointer is 30 m north of the line, over its first third.
        const setLngLat = drag(5.0005, 52.0003);
        expect(setLngLat).toHaveBeenCalledOnce();
        const [lng, lat] = setLngLat.mock.calls[0][0] as [number, number];
        expect(lng).toBeCloseTo(5.0005, 6);
        expect(lat).toBeCloseTo(52.0, 9);
    });

    it("stops at the end of the ring rather than leaving it", () => {
        renderMarker();
        const setLngLat = drag(5.005, 52.0);
        const [lng] = setLngLat.mock.calls[0][0] as [number, number];
        expect(lng).toBeCloseTo(5.0015, 9);
    });

    it("follows the pointer freely when there is no ring to slide along", () => {
        renderMarker({ring: null});
        const setLngLat = drag(5.0005, 52.0003);
        expect(setLngLat).toHaveBeenCalledWith([5.0005, 52.0003]);
    });

    it("reports where it was dropped, on the ring, once, and not while dragging", () => {
        const {onMove} = renderMarker();
        drag(5.0007, 52.0004);
        expect(onMove).not.toHaveBeenCalled();
        drop(5.0007, 52.0004);
        expect(onMove).toHaveBeenCalledOnce();
        const [lng, lat] = onMove.mock.calls[0] as [number, number];
        expect(lng).toBeCloseTo(5.0007, 6);
        expect(lat).toBeCloseTo(52.0, 9);
    });

    it("stays where it was dropped until the planner answers, then shows the real start", () => {
        const {again} = renderMarker();
        drop(5.0007, 52.0004);
        // Waiting for the planner: it must NOT jump back to the old start.
        expect(seen.props?.longitude).toBeCloseTo(5.0007, 6);
        expect(seen.props?.latitude).toBeCloseTo(52.0, 9);

        // The planner answers with the start it really chose (clear of a corner).
        again({settledCount: 1, longitude: 5.0009, latitude: 52.0});
        expect(seen.props?.longitude).toBe(5.0009);
        expect(seen.props?.latitude).toBe(52.0);
    });

    it("lets go of the dropped spot when the answer leaves the start where it was", () => {
        const {again} = renderMarker();
        drop(5.0007, 52.0004);
        again({settledCount: 1});
        expect(seen.props?.longitude).toBe(5.0005);
    });

    it("does not let go early on an answer that was already counted", () => {
        const {again} = renderMarker({settledCount: 3});
        drop(5.0007, 52.0004);
        again({settledCount: 3});
        expect(seen.props?.longitude).toBeCloseTo(5.0007, 6);
    });

    it("lets go of the dropped spot when the start itself changes, for example after a reset", () => {
        const {again} = renderMarker();
        drop(5.0007, 52.0004);
        again({longitude: 5.0012, latitude: 52.0});
        expect(seen.props?.longitude).toBe(5.0012);
    });

    it("does not render again when the page re-renders with the same marker inputs", () => {
        const {again, props} = renderMarker();
        const before = seen.renders;
        // The page re-renders constantly (robot pose, map streams) with the same marker inputs.
        again({onMove: props.onMove});
        again({onMove: props.onMove});
        expect(seen.renders).toBe(before);
    });

    it("keeps the marker under the drag when it is rendered again mid-drag", () => {
        const {again} = renderMarker();
        act(() => seen.props?.onDragStart?.());
        drag(5.0009, 52.0004);
        // Something changes while the operator is still dragging (an answer for an earlier
        // request arrives). The marker must not be put back on the old start.
        again({settledCount: 1});
        expect(seen.props?.longitude).toBeCloseTo(5.0009, 6);
        expect(seen.props?.latitude).toBeCloseTo(52.0, 9);
    });

    it("uses the latest onMove at drop time", () => {
        const first = vi.fn();
        const second = vi.fn();
        const {again} = renderMarker({onMove: first});
        again({onMove: second});
        drop(5.0007, 52.0004);
        expect(first).not.toHaveBeenCalled();
        expect(second).toHaveBeenCalledOnce();
    });
});
