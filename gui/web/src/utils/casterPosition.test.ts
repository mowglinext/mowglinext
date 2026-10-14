import {describe, expect, it} from "vitest";
import {materializeLegacyCasterPosition, resolveCasterPosition} from "./casterPosition";
import {MOWER_MODELS} from "../constants/mowerModels";

describe("explicit caster position", () => {
    it("preserves the old axle position before a chassis edit without mutating saved values", () => {
        const saved = {caster_x_offset: -1, chassis_center_x: .18, chassis_length: .54, caster_radius: .03};
        expect(resolveCasterPosition(saved)).toBe(.42);
        const edit = {...materializeLegacyCasterPosition(saved), chassis_length: .8};
        expect(resolveCasterPosition(edit)).toBe(.42);
        expect(saved.caster_x_offset).toBe(-1);
    });
    it.each([0, -.1, .4])("retains explicit position %s", x => {
        const saved = {caster_x_offset: x};
        expect(materializeLegacyCasterPosition(saved)).toBe(saved);
        expect(resolveCasterPosition(saved)).toBe(x);
    });
    it("does not invent dimensions for incomplete legacy data", () => {
        expect(resolveCasterPosition({caster_x_offset: -1})).toBeUndefined();
        expect(resolveCasterPosition({caster_x_offset: ""})).toBeUndefined();
        expect(resolveCasterPosition({caster_x_offset: Infinity})).toBeUndefined();
    });
    it("gives every built-in model an explicit provisional position inside its body", () => {
        for (const model of MOWER_MODELS.filter(m => m.value !== "CUSTOM")) {
            const d = model.defaults;
            expect(d.caster_x_offset).toBeGreaterThan(0);
            if (d.chassis_length) {
                expect(d.caster_x_offset + d.caster_radius).toBeLessThan(d.chassis_center_x + d.chassis_length / 2);
                expect(d.caster_x_offset - d.caster_radius).toBeGreaterThan(d.chassis_center_x - d.chassis_length / 2);
            }
        }
    });
});
