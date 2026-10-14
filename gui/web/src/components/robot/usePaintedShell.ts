import {artworkUrl} from "./artworkUrl";
import {useEffect, useState} from "react";
import {SHELLS, shellProjectionFrame, type MowerStyle} from "./shellAssets";
import {normalizeShellColour, paintCoverage, recolourShellPixels} from "./shellPaint";
import type {RobotView} from "../../utils/robotModel";

type Artwork = "full" | "map";
type Prepared = {pixels: ImageData; coverage: Float32Array};
// Bounded caches are shared by Hardware, Sensors and Map. Pose updates never
// enter the pixel pipeline; only style, view, resolution and colour identify it.
const prepared = new Map<string, Promise<Prepared>>();
const painted = new Map<string, Promise<string>>();
function cached<T>(cache: Map<string, Promise<T>>, key: string, limit: number, create: () => Promise<T>): Promise<T> {
    const existing = cache.get(key);
    if (existing) { cache.delete(key); cache.set(key, existing); return existing; }
    const value = create();
    cache.set(key, value);
    if (cache.size > limit) cache.delete(cache.keys().next().value!);
    void value.catch(() => { if (cache.get(key) === value) cache.delete(key); });
    return value;
}
function image(src: string): Promise<HTMLImageElement> {
    return new Promise((resolve, reject) => {
        const img = new Image();
        img.onload = () => resolve(img);
        img.onerror = () => reject(new Error("Shell artwork unavailable"));
        img.src = src;
    });
}
function prepare(style: MowerStyle, view: RobotView, artwork: Artwork): Promise<Prepared> {
    return cached(prepared, `${style}:${view}:${artwork}`, 8, async () => {
        const asset = SHELLS[style];
        if (!asset.paint) throw new Error("Shell has no paint mask");
        const compact = artwork === "map";
        const [source, mask] = await Promise.all([
            image(artworkUrl(compact ? `map/${style}-${view}.webp` : asset.files?.[view] ?? `${style}.png`)),
            image(artworkUrl(compact ? `map/${style}-${view}-paint.png` : asset.paint.masks[view])),
        ]);
        const frame = shellProjectionFrame(style, view);
        const [x, y, w, h] = compact ? [0, 0, source.width, source.height] : frame;
        const scale = Math.min(1, (compact ? asset.mapMaxSize : 768) / Math.max(w, h));
        const canvas = document.createElement("canvas");
        canvas.width = Math.max(1, Math.round(w * scale));
        canvas.height = Math.max(1, Math.round(h * scale));
        const ctx = canvas.getContext("2d", {willReadFrequently: true});
        if (!ctx) throw new Error("Canvas unavailable");
        ctx.imageSmoothingQuality = "high";
        ctx.drawImage(source, x, y, w, h, 0, 0, canvas.width, canvas.height);
        const pixels = ctx.getImageData(0, 0, canvas.width, canvas.height);
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.drawImage(mask, x, y, w, h, 0, 0, canvas.width, canvas.height);
        return {pixels, coverage: paintCoverage(pixels.data, ctx.getImageData(0, 0, canvas.width, canvas.height).data)};
    });
}
export function paintedShell(style: MowerStyle, view: RobotView, artwork: Artwork, colour: string): Promise<string> {
    return cached(painted, `${style}:${view}:${artwork}:${colour}`, 16, async () => {
        const {pixels, coverage} = await prepare(style, view, artwork);
        const canvas = document.createElement("canvas");
        canvas.width = pixels.width; canvas.height = pixels.height;
        const ctx = canvas.getContext("2d");
        if (!ctx) throw new Error("Canvas unavailable");
        const output = ctx.createImageData(pixels.width, pixels.height);
        output.data.set(recolourShellPixels(pixels.data, coverage, colour));
        ctx.putImageData(output, 0, 0);
        return canvas.toDataURL("image/png");
    });
}
export function usePaintedShell(style: MowerStyle, view: RobotView, artwork: Artwork, colour?: string): string | undefined {
    const normalized = normalizeShellColour(colour);
    const key = `${style}:${view}:${artwork}:${normalized}`;
    const [result, setResult] = useState<{key: string; src: string}>();
    useEffect(() => {
        if (!normalized || !SHELLS[style].paint) return;
        let active = true;
        // Coalesce picker drags. This never delays configuration writes or pose updates.
        const timer = window.setTimeout(() => {
            void paintedShell(style, view, artwork, normalized).then(src => {
                if (active) setResult({key, src});
            }).catch(() => { /* Preserve original artwork if a mask/canvas fails. */ });
        }, 60);
        return () => { active = false; window.clearTimeout(timer); };
    }, [style, view, artwork, normalized, key]);
    return result?.key === key && normalized ? result.src : undefined;
}
