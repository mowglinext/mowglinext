import {OccupancyGrid} from "../types/ros.ts";

export type Rgba = readonly [number, number, number, number];

/** Per-cell colour: return null for a transparent pixel. */
export type CellPaint = (value: number) => Rgba | null;

export interface PaintedGrid {
    /** RGBA pixels, canvas row order (row 0 = top = max map Y). */
    pixels: Uint8ClampedArray;
    width: number;
    height: number;
}

export interface RasterizedGrid {
    /** PNG data URL of the overlay (transparent where paint returned null). */
    dataUrl: string;
    /** Cells along the map X axis. */
    width: number;
    /** Cells along the map Y axis. */
    height: number;
    /** Metres per cell. */
    resolution: number;
    /** Map-frame X of grid column 0 (min X). */
    originX: number;
    /** Map-frame Y of grid row 0 (min Y). */
    originY: number;
}

/**
 * Pure pixel pass over a nav_msgs/OccupancyGrid. OccupancyGrid row 0 is the
 * BOTTOM of the map frame (min Y) while an image's row 0 is the top, so rows
 * are flipped vertically (CLAUDE.md Invariant 14: width = X cells, height = Y
 * cells, row-major `data[y * width + x]`). Returns null for an empty grid.
 */
export function paintOccupancyGrid(grid: OccupancyGrid, paint: CellPaint): PaintedGrid | null {
    if (!grid.info || !grid.data) return null;
    const width = grid.info.width ?? 0;
    const height = grid.info.height ?? 0;
    if (width === 0 || height === 0) return null;

    const pixels = new Uint8ClampedArray(width * height * 4);
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            const rgba = paint(grid.data[row * width + col]);
            if (!rgba) continue; // transparent
            const idx = ((height - 1 - row) * width + col) * 4;
            pixels[idx] = rgba[0];
            pixels[idx + 1] = rgba[1];
            pixels[idx + 2] = rgba[2];
            pixels[idx + 3] = rgba[3];
        }
    }
    return {pixels, width, height};
}

/**
 * Rasterize an OccupancyGrid to a PNG data URL through a 2D canvas. Callers own
 * placement: MapPage maps the returned geometry to Mapbox lon/lat corners, the
 * dashboard mini-map maps it into its normalised 0..1 space. Returns null for
 * a degenerate grid or when a 2D canvas context is unavailable.
 */
export function rasterizeOccupancyGrid(grid: OccupancyGrid, paint: CellPaint): RasterizedGrid | null {
    const painted = paintOccupancyGrid(grid, paint);
    if (!painted || !grid.info) return null;
    const {width, height} = painted;
    const info = grid.info;

    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    if (!ctx) return null;
    const imageData = ctx.createImageData(width, height);
    imageData.data.set(painted.pixels);
    ctx.putImageData(imageData, 0, 0);

    return {
        dataUrl: canvas.toDataURL(),
        width,
        height,
        resolution: info.resolution ?? 0.1,
        originX: info.origin?.position?.x ?? 0,
        originY: info.origin?.position?.y ?? 0,
    };
}
