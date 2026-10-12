import {describe, expect, it} from "vitest";
import {MOWER_APPEARANCES} from "./mowerAppearances.ts";
import {parseMowerAppearanceMenuKey} from "./mowerAppearanceMenuKey.ts";

describe("parseMowerAppearanceMenuKey", () => {
    it.each(Object.values(MOWER_APPEARANCES).map(({id}) => id))(
        "parses the registered %s appearance",
        (id) => {
            expect(parseMowerAppearanceMenuKey(`mowerAppearance:${id}`)).toBe(id);
        },
    );

    it.each([
        "",
        "mowerAppearance",
        "mowerAppearance:",
        "mowerAppearance:unknown",
        "mowerAppearance:urdf:extra",
        "satellite",
        "mowerAppearance:toString",
        "mowerAppearance:constructor",
    ])("rejects invalid menu key %s", (key) => {
        expect(parseMowerAppearanceMenuKey(key)).toBeUndefined();
    });
});
