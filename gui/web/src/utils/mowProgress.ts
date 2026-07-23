import {OccupancyGrid} from "../types/ros.ts";

export interface RasterizedMowProgress {
    /** PNG data URL of the mowed-cell overlay (transparent where unmowed). */
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

/** Translucent lime overlay used for mowed cells across the GUI. */
const MOWED_RGBA: readonly [number, number, number, number] = [124, 255, 178, 150];

/**
 * Rasterize the mow-progress OccupancyGrid (cell value >= 100 == mowed) into a
 * canvas whose pixels are the translucent lime overlay. OccupancyGrid row 0 is
 * the bottom of the map frame while canvas row 0 is the top, so rows are
 * flipped vertically. Returns null for an empty/degenerate grid or when a 2D
 * canvas context is unavailable.
 *
 * This is the single source of truth for the mowed-cell pixel pass. Callers own
 * placement: MapPage maps the returned geometry to Mapbox lon/lat corners, the
 * dashboard mini-map maps it into its normalised 0..1 space.
 */
export function rasterizeMowProgress(grid: OccupancyGrid): RasterizedMowProgress | null {
    if (!grid.info || !grid.data) return null;
    const width = grid.info.width ?? 0;
    const height = grid.info.height ?? 0;
    if (width === 0 || height === 0) return null;
    const resolution = grid.info.resolution ?? 0.1;
    const originX = grid.info.origin?.position?.x ?? 0;
    const originY = grid.info.origin?.position?.y ?? 0;

    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    if (!ctx) return null;

    const imageData = ctx.createImageData(width, height);
    const [r, g, b, a] = MOWED_RGBA;
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            const gridIdx = row * width + col;
            if (grid.data[gridIdx] < 100) continue; // unmowed / unknown -> transparent
            // Grid row 0 = bottom, canvas row 0 = top -> flip vertically.
            const canvasIdx = ((height - 1 - row) * width + col) * 4;
            imageData.data[canvasIdx] = r;
            imageData.data[canvasIdx + 1] = g;
            imageData.data[canvasIdx + 2] = b;
            imageData.data[canvasIdx + 3] = a;
        }
    }
    ctx.putImageData(imageData, 0, 0);
    return {dataUrl: canvas.toDataURL(), width, height, resolution, originX, originY};
}
