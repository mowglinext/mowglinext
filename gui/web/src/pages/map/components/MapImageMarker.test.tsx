import React from "react";
import {fireEvent, render, screen, waitFor} from "@testing-library/react";
import {beforeEach, describe, expect, it, vi} from "vitest";
import type {MapImageAppearance} from "../../../constants/mowerAppearances.ts";
import {MapImageMarker} from "./MapImageMarker.tsx";

const {testMap, markerEvents, markerPositions} = vi.hoisted(() => ({
    testMap: {
        scale: 10,
        project: ([longitude, latitude]: [number, number]) => ({x: longitude * testMap.scale, y: latitude * testMap.scale}),
        handlers: new Map<string, Set<() => void>>(),
        on: vi.fn((event: string, handler: () => void) => {
            const handlers = testMap.handlers.get(event) ?? new Set();
            handlers.add(handler);
            testMap.handlers.set(event, handlers);
        }),
        off: vi.fn((event: string, handler: () => void) => testMap.handlers.get(event)?.delete(handler)),
    },
    markerEvents: [] as {rotation: number; rotationAlignment: string; pitchAlignment: string; anchor: string}[],
    markerPositions: [] as Array<{longitude: number; latitude: number; style?: React.CSSProperties}>,
}));

vi.mock("react-map-gl/mapbox", () => ({
    Marker: ({children, style, rotation, rotationAlignment, pitchAlignment, anchor, longitude, latitude}: {
        children: React.ReactNode;
        style?: React.CSSProperties;
        rotation: number;
        rotationAlignment: string;
        pitchAlignment: string;
        anchor: string;
        longitude: number;
        latitude: number;
    }) => {
        markerEvents.push({rotation, rotationAlignment, pitchAlignment, anchor});
        markerPositions.push({longitude, latitude, style});
        return <div data-testid="map-marker" data-longitude={longitude} data-latitude={latitude} style={style}>{children}</div>;
    },
    useMap: () => ({current: testMap}),
}));

const image: MapImageAppearance = {
    src: "/assets/robots/biltema-rm1000/mower.webp",
    altKey: "mapToolbar.mowerAppearanceBiltemaRm1000Alt",
    visibleLengthM: 0.57,
    visibleLengthFraction: 0.9,
    poseAnchor: {x: 0.5, y: 0.77},
};

const dockImage: MapImageAppearance = {
    src: "/assets/robots/generic/dock.webp",
    altKey: "mapToolbar.dockAppearanceGenericAlt",
    visibleLengthM: 0.63,
    visibleLengthFraction: 0.951,
    visibleWidthM: 0.46,
    visibleWidthFraction: 0.678,
    poseAnchor: {x: 0.5, y: 0.16},
    headingOffsetRad: Math.PI,
};

function renderMarker(src = image.src, headingRad = 0, callbacks = {onLoad: vi.fn(), onError: vi.fn()}) {
    const selectedImage = {...image, src};
    const result = render(
        <MapImageMarker
            image={selectedImage}
            alt="Biltema RM1000 mower"
            longitude={18.06}
            latitude={59.33}
            headingRad={headingRad}
            {...callbacks}
        />,
    );
    return {...result, callbacks};
}

describe("MapImageMarker", () => {
    beforeEach(() => {
        vi.clearAllMocks();
        markerEvents.length = 0;
        markerPositions.length = 0;
        testMap.handlers.clear();
        testMap.scale = 10;
        Object.defineProperty(HTMLImageElement.prototype, "decode", {
            configurable: true,
            value: vi.fn().mockResolvedValue(undefined),
        });
    });

    it("passes the live ROS heading and map-aligned ground-plane options into the actual Marker", () => {
        for (const [heading, rotation] of [[0, 90], [Math.PI / 2, 0], [Math.PI, -90], [-Math.PI / 2, 180]] as const) {
            const view = renderMarker(image.src, heading);
            expect(markerEvents[markerEvents.length - 1]).toEqual({rotation, rotationAlignment: "map", pitchAlignment: "map", anchor: "center"});
            view.unmount();
        }
    });

    it("recalculates physical size after map move and resize and cleans up both listeners", () => {
        const {unmount} = renderMarker();
        const marker = screen.getByTestId("map-marker");
        const firstSize = Number.parseFloat(marker.style.width);
        testMap.scale = 20;
        for (const event of ["move", "resize"]) {
            for (const handler of testMap.handlers.get(event) ?? []) handler();
        }
        const updatedSize = Number.parseFloat(screen.getByTestId("map-marker").style.width);
        expect(updatedSize).toBeCloseTo(firstSize * 2);
        unmount();
        expect(testMap.off).toHaveBeenCalledTimes(2);
        expect(testMap.handlers.get("move")?.size).toBe(0);
        expect(testMap.handlers.get("resize")?.size).toBe(0);
    });

    it("uses separate calibrated dock width and length without moving its pose anchor", () => {
        render(
            <MapImageMarker image={dockImage} alt="Generic visual dock approximation"
                longitude={18.06} latitude={59.33} headingRad={0} onLoad={vi.fn()} onError={vi.fn()} />,
        );
        const marker = screen.getByTestId("map-marker");
        const width = Number.parseFloat(marker.style.width);
        const height = Number.parseFloat(marker.style.height);
        const imageElement = screen.getByAltText("Generic visual dock approximation");
        const imageOffsetTop = Number.parseFloat(imageElement.style.top);
        expect(width / height).toBeCloseTo((0.46 / 0.678) / (0.63 / 0.951), 3);
        expect(imageOffsetTop + 0.16 * height).toBeCloseTo(height / 2);
        expect(markerEvents[markerEvents.length - 1]?.rotation).toBe(-90);
    });

    it("applies an optional forward display offset and foreground clip without changing the source pose", () => {
        const clipped = "polygon(43% 78%, 57% 78%, 68% 83%, 68% 92%, 59% 96%, 41% 96%, 32% 92%, 32% 83%)";
        render(
            <MapImageMarker image={{...dockImage, forwardOffsetM: 0.02}} alt="Dock foreground"
                longitude={18.06} latitude={59.33} headingRad={Math.PI / 2} clipPath={clipped} zIndex={1000}
                onLoad={vi.fn()} onError={vi.fn()} />,
        );
        const marker = screen.getByTestId("map-marker");
        const imageElement = screen.getByAltText("Dock foreground");
        expect(Number(marker.getAttribute("data-longitude"))).toBeCloseTo(18.06, 6);
        expect(Number(marker.getAttribute("data-latitude"))).toBeGreaterThan(59.33);
        expect(marker.style.zIndex).toBe("1000");
        expect(imageElement.style.clipPath).toBe(clipped);
        expect(markerPositions[0]?.longitude).toBe(Number(marker.getAttribute("data-longitude")));
    });

    it("keeps the image hidden until decode and reports the decoded source", async () => {
        const {callbacks} = renderMarker();
        const element = screen.getByAltText("Biltema RM1000 mower");
        expect(element).toHaveStyle({visibility: "hidden"});
        fireEvent.load(element);
        await waitFor(() => expect(callbacks.onLoad).toHaveBeenCalledOnce());
        expect(element).toHaveStyle({visibility: "visible"});
    });

    it("falls back when the browser cannot decode the loaded source", async () => {
        Object.defineProperty(HTMLImageElement.prototype, "decode", {
            configurable: true,
            value: vi.fn().mockRejectedValue(new Error("decode failed")),
        });
        const {callbacks} = renderMarker();
        const imageElement = screen.getByAltText("Biltema RM1000 mower");
        fireEvent.load(imageElement);
        await waitFor(() => expect(callbacks.onError).toHaveBeenCalledOnce());
        expect(callbacks.onLoad).not.toHaveBeenCalled();
        expect(screen.queryByTestId("map-marker")).not.toBeInTheDocument();
    });

    it("ignores a late decode from the previous src after switching sources", async () => {
        let resolveOld: (() => void) | undefined;
        Object.defineProperty(HTMLImageElement.prototype, "decode", {
            configurable: true,
            value: vi.fn().mockImplementationOnce(() => new Promise<void>((resolve) => { resolveOld = resolve; }))
                .mockResolvedValue(undefined),
        });
        const oldCallbacks = {onLoad: vi.fn(), onError: vi.fn()};
        const view = renderMarker(image.src, 0, oldCallbacks);
        fireEvent.load(screen.getByAltText("Biltema RM1000 mower"));
        const newCallbacks = {onLoad: vi.fn(), onError: vi.fn()};
        view.rerender(
            <MapImageMarker image={{...image, src: "/another.webp"}} alt="Other mower"
                longitude={18.06} latitude={59.33} headingRad={0} {...newCallbacks} />,
        );
        fireEvent.load(screen.getByAltText("Other mower"));
        await waitFor(() => expect(newCallbacks.onLoad).toHaveBeenCalledOnce());
        resolveOld?.();
        await Promise.resolve();
        expect(oldCallbacks.onLoad).not.toHaveBeenCalled();
        expect(oldCallbacks.onError).not.toHaveBeenCalled();
    });

    it("removes a failed marker and reports failure so the caller can keep its fallback", async () => {
        const {callbacks} = renderMarker();
        fireEvent.error(screen.getByAltText("Biltema RM1000 mower"));
        await waitFor(() => expect(callbacks.onError).toHaveBeenCalledOnce());
        expect(callbacks.onLoad).not.toHaveBeenCalled();
        expect(screen.queryByTestId("map-marker")).not.toBeInTheDocument();
    });
});
