import {describe, expect, it} from "vitest";
import {DOCK_APPEARANCES, DOCK_FOREGROUND_CLIP_PATH, getAvailableDockAppearances, getDockAppearanceResetForMowerChange, resolveDockAppearance, resolveMowerAppearance, shouldDisplayMapImage, shouldDisplayMowerImage} from "./mowerAppearances.ts";

describe("mower appearance registry", () => {
    it("uses the bundled RM1000 image only after an explicit GUI appearance selection", () => {
        expect(resolveMowerAppearance("biltema-rm1000").mowerImage?.src)
            .toBe("/assets/robots/biltema-rm1000/mower.webp");
        expect(resolveMowerAppearance("biltema-rm1000").mowerImage?.forwardOffsetM).toBe(0.02);
        expect(resolveMowerAppearance("YardForce500").mowerImage).toBeUndefined();
        expect(resolveMowerAppearance("YardForce500B").mowerImage).toBeUndefined();
    });

    it("falls back to the URDF appearance for unknown values", () => {
        expect(resolveMowerAppearance("custom-shell").id).toBe("urdf");
        expect(resolveMowerAppearance(undefined).id).toBe("urdf");
    });

    it("keeps the URDF silhouette until the selected image is loaded and a pose exists", () => {
        const appearance = resolveMowerAppearance("biltema-rm1000");
        const src = appearance.mowerImage!.src;
        expect(shouldDisplayMowerImage(appearance, undefined, true, true)).toBe(false);
        expect(shouldDisplayMowerImage(appearance, src, false, true)).toBe(false);
        expect(shouldDisplayMowerImage(appearance, src, true, false)).toBe(false);
        expect(shouldDisplayMowerImage(appearance, "/missing.webp", true, true)).toBe(false);
        expect(shouldDisplayMowerImage(appearance, src, true, true)).toBe(true);
        expect(shouldDisplayMowerImage(resolveMowerAppearance("urdf"), src, true, true)).toBe(false);
    });

    it("offers the generic dock to every mower and the branded dock only for the RM1000", () => {
        expect(getAvailableDockAppearances("urdf").map(({id}) => id)).toEqual(["marker", "generic"]);
        expect(getAvailableDockAppearances("biltema-rm1000").map(({id}) => id)).toEqual([
            "marker", "generic", "biltema-rm1000",
        ]);
        expect(resolveDockAppearance(undefined, "urdf").id).toBe("marker");
        expect(resolveDockAppearance("stale", "biltema-rm1000").id).toBe("marker");
        expect(resolveDockAppearance("biltema-rm1000", "urdf").id).toBe("marker");
        expect(resolveDockAppearance("generic", "urdf").image?.src).toBe("/assets/robots/generic/dock.webp");
        expect(resolveDockAppearance("generic", "biltema-rm1000").image?.src).toBe("/assets/robots/generic/dock.webp");
        expect(resolveDockAppearance("biltema-rm1000", "biltema-rm1000").image?.src)
            .toBe("/assets/robots/biltema-rm1000/dock.webp");
    });

    it("uses the reviewed dock overlap anchor and center-tongue foreground mask", () => {
        for (const id of ["generic", "biltema-rm1000"] as const) {
            expect(DOCK_APPEARANCES[id].image?.poseAnchor).toEqual({x: 0.5, y: 0.16});
            expect(DOCK_APPEARANCES[id].image?.headingOffsetRad).toBe(Math.PI);
            expect(DOCK_APPEARANCES[id].foregroundClipPath).toBe(DOCK_FOREGROUND_CLIP_PATH);
        }
    });

    it("resets only an incompatible RM1000 dock selection when the mower changes", () => {
        expect(getDockAppearanceResetForMowerChange(DOCK_APPEARANCES["biltema-rm1000"], "urdf")).toBe("marker");
        expect(getDockAppearanceResetForMowerChange(DOCK_APPEARANCES.generic, "urdf")).toBeUndefined();
        expect(getDockAppearanceResetForMowerChange(DOCK_APPEARANCES.generic, "biltema-rm1000")).toBeUndefined();
        expect(getDockAppearanceResetForMowerChange(DOCK_APPEARANCES.marker, "urdf")).toBeUndefined();
    });

    it("keeps the existing dock marker until the selected image decodes with a valid pose and heading", () => {
        const image = resolveDockAppearance("generic", "urdf").image;
        expect(shouldDisplayMapImage(image, undefined, true, true)).toBe(false);
        expect(shouldDisplayMapImage(image, image?.src, false, true)).toBe(false);
        expect(shouldDisplayMapImage(image, image?.src, true, false)).toBe(false);
        expect(shouldDisplayMapImage(image, image?.src, true, true)).toBe(true);
        expect(shouldDisplayMapImage(undefined, image?.src, true, true)).toBe(false);
    });
});
