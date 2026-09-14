import {useCallback, useRef, useState} from "react";
import {useWS} from "../../../hooks/useWS.ts";
import {OccupancyGrid} from "../../../types/ros.ts";
import {RasterizedGrid} from "../../../utils/occupancyGrid.ts";
import {transpose} from "../../../utils/map.tsx";

export type GridImage = {
    url: string;
    /** Mapbox image-source corners: [top-left, top-right, bottom-right, bottom-left]. */
    coordinates: [[number, number], [number, number], [number, number], [number, number]];
};

type Datum = [number, number, number];

/** Map a rasterized grid's map-frame geometry to Mapbox lon/lat corners. */
export function gridImageFromRaster(raster: RasterizedGrid, offsetX: number, offsetY: number, datum: Datum): GridImage {
    const {originX, originY, resolution} = raster;
    const gridWidth = raster.width * resolution;
    const gridHeight = raster.height * resolution;
    return {
        url: raster.dataUrl,
        coordinates: [
            transpose(offsetX, offsetY, datum, originY + gridHeight, originX),
            transpose(offsetX, offsetY, datum, originY + gridHeight, originX + gridWidth),
            transpose(offsetX, offsetY, datum, originY, originX + gridWidth),
            transpose(offsetX, offsetY, datum, originY, originX),
        ],
    };
}

/**
 * One OccupancyGrid WebSocket stream rendered as a Mapbox image source. The
 * latest grid waits in a ref and is rasterized at most once per animation
 * frame: the raster + toDataURL is too heavy for the WebSocket message handler
 * (it would stall pose/lidar frames), and a burst of grids collapses to one
 * paint. Used for the mow-progress overlay and the LiDAR anchor map.
 */
export function useGridImageStream(
    rasterize: (grid: OccupancyGrid) => RasterizedGrid | null,
    offsetX: number,
    offsetY: number,
    datum: Datum,
) {
    const [image, setImage] = useState<GridImage | null>(null);
    const pendingRef = useRef<{ grid: OccupancyGrid; offsetX: number; offsetY: number; datum: Datum } | null>(null);
    const rafRef = useRef<number | null>(null);

    const stream = useWS<string>(
        () => {},
        () => {},
        (e) => {
            const grid = (e as unknown) as OccupancyGrid;
            if (!grid.info || !grid.data) return;
            if ((grid.info.width ?? 0) === 0 || (grid.info.height ?? 0) === 0) return;
            pendingRef.current = {grid, offsetX, offsetY, datum};
            if (rafRef.current != null) return;
            rafRef.current = requestAnimationFrame(() => {
                rafRef.current = null;
                const pending = pendingRef.current;
                pendingRef.current = null;
                if (!pending) return;
                const raster = rasterize(pending.grid);
                if (raster) setImage(gridImageFromRaster(raster, pending.offsetX, pending.offsetY, pending.datum));
            });
        },
    );

    const cancel = useCallback(() => {
        if (rafRef.current != null) {
            cancelAnimationFrame(rafRef.current);
            rafRef.current = null;
        }
    }, []);

    return {image, stream, cancel};
}

export type UseGridImageStream = ReturnType<typeof useGridImageStream>;
