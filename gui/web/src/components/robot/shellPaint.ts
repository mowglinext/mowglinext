/** Display-only pigment rendering. Input and output share exactly the same alpha. */
export function normalizeShellColour(value: unknown): string {
    return typeof value === "string" && /^#[0-9a-f]{6}$/i.test(value) ? value.toLowerCase() : "";
}
const clamp = (n: number) => Math.max(0, Math.min(1, n));
function smooth(a: number, b: number, value: number) {
    const t = clamp((value - a) / (b - a));
    return t * t * (3 - 2 * t);
}
/** Antialiased mask coverage, with original mint accents excluded. */
export function paintCoverage(source: Uint8ClampedArray, mask: Uint8ClampedArray): Float32Array {
    if (source.length !== mask.length || source.length % 4) throw new Error("Paint mask dimensions differ");
    const coverage = new Float32Array(source.length / 4);
    for (let p = 0; p < coverage.length; p++) {
        const i = p * 4;
        const mint = smooth(3, 16, source[i + 1] - source[i]) * smooth(0, 12, source[i + 1] - source[i + 2]);
        coverage[p] = mask[i + 3] / 255 * (1 - mint);
    }
    return coverage;
}
/** Match the approved preview: source luminance supplies diffuse shadows and
 * neutral highlights, rather than rotating the hue of almost-grey artwork. */
export function recolourShellPixels(source: Uint8ClampedArray, coverage: Float32Array, colour: string): Uint8ClampedArray {
    const output = new Uint8ClampedArray(source);
    const normalized = normalizeShellColour(colour);
    if (!normalized) return output;
    if (coverage.length * 4 !== source.length) throw new Error("Paint coverage dimensions differ");
    const pigment = [1, 3, 5].map(i => parseInt(normalized.slice(i, i + 2), 16));
    for (let p = 0; p < coverage.length; p++) {
        const i = p * 4, a = coverage[p];
        if (!a || !source[i + 3]) continue;
        const luminance = (.2126 * source[i] + .7152 * source[i + 1] + .0722 * source[i + 2]) / 255;
        const shadow = Math.min(1, luminance / .44), highlight = clamp((luminance - .44) / .56);
        for (let c = 0; c < 3; c++) {
            const painted = pigment[c] * shadow * (1 - highlight) + 255 * highlight;
            output[i + c] = source[i + c] * (1 - a) + painted * a;
        }
    }
    return output;
}
