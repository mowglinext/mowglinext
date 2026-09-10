import { describe, expect, it } from "vitest";
import en from "./locales/en.json";
import fr from "./locales/fr.json";

/**
 * The operator running this robot is French, so both locales are maintained in
 * lockstep — a key added to only one of them renders as its raw dotted path in
 * the other UI (i18next falls back to the key, not to the English string, for
 * a missing nested leaf). The two files were in exact parity when this guard
 * was added, so any failure here is drift introduced by the change under test.
 */
type Json = Record<string, unknown>;

function leafKeys(node: Json, prefix = ""): string[] {
    return Object.entries(node).flatMap(([key, value]) => {
        const path = prefix ? `${prefix}.${key}` : key;
        return value !== null && typeof value === "object" && !Array.isArray(value)
            ? leafKeys(value as Json, path)
            : [path];
    });
}

describe("i18n locales", () => {
    const enKeys = leafKeys(en as Json);
    const frKeys = leafKeys(fr as Json);

    it("has no English string missing from French", () => {
        const missing = enKeys.filter((key) => !frKeys.includes(key));
        expect(missing).toEqual([]);
    });

    it("has no French string missing from English", () => {
        const orphan = frKeys.filter((key) => !enKeys.includes(key));
        expect(orphan).toEqual([]);
    });

    it("has no empty translation in either locale", () => {
        const empty = (node: Json) =>
            leafKeys(node).filter((path) => {
                const value = path
                    .split(".")
                    .reduce<unknown>((acc, part) => (acc as Json)?.[part], node);
                return typeof value === "string" && value.trim() === "";
            });
        expect(empty(en as Json)).toEqual([]);
        expect(empty(fr as Json)).toEqual([]);
    });

    // Sanity anchor: if the settings sections and their string namespace ever
    // drift apart, the nav renders a section whose fields have no labels.
    it("translates every settings section that has a strings namespace", () => {
        for (const section of ["sensors", "leds", "rain", "advanced"]) {
            expect(enKeys).toContain(`settingsSections.${section}.label`);
            expect(frKeys).toContain(`settingsSections.${section}.label`);
        }
    });
});
