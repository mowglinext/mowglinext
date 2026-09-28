import {describe, expect, it} from "vitest";
import {DOCK_APPEARANCES} from "./mowerAppearances.ts";
import {parseDockAppearanceMenuKey} from "./dockAppearanceMenuKey.ts";

describe("parseDockAppearanceMenuKey", () => {
    it("accepts every registered dock appearance", () => {
        for (const {id} of Object.values(DOCK_APPEARANCES)) {
            expect(parseDockAppearanceMenuKey(`dockAppearance:${id}`)).toBe(id);
        }
    });

    it.each(["dockAppearance:", "dockAppearance:custom", "dockAppearance:marker:extra", "mowerAppearance:generic", "dockAppearance:constructor"])(
        "rejects invalid menu key %s",
        (key) => expect(parseDockAppearanceMenuKey(key)).toBeUndefined(),
    );
});
