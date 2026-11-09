import {describe, expect, it} from "vitest";
import {DOCK_APPEARANCES, resolveDockAppearance, shouldDisplayMapImage} from "./mowerAppearances";

describe("paired hardware docks", () => {
    it.each([undefined, "", "retired", "constructor"])("defaults %s to the styled dock", value => {
        expect(resolveDockAppearance(value)).toBe(DOCK_APPEARANCES.styled);
    });
    it("allows future hardware presets to use any registered dock", () => {
        for (const dock of Object.values(DOCK_APPEARANCES)) expect(resolveDockAppearance(dock.id)).toBe(dock);
    });
    it("keeps the drawn fallback until a paired photo dock is ready", () => {
        const image = DOCK_APPEARANCES.generic.image!;
        expect(shouldDisplayMapImage(image, undefined, true, true)).toBe(false);
        expect(shouldDisplayMapImage(image, image.src, false, true)).toBe(false);
        expect(shouldDisplayMapImage(image, image.src, true, false)).toBe(false);
        expect(shouldDisplayMapImage(image, image.src, true, true)).toBe(true);
        expect(shouldDisplayMapImage(undefined, image.src, true, true)).toBe(false);
    });
});
