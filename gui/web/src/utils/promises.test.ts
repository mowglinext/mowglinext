import {describe, expect, it, vi} from "vitest";
import {voidify} from "./promises.ts";

describe("voidify", () => {
    it("passes the arguments through and returns nothing", () => {
        const handler = vi.fn((checked: boolean, label: string) => Promise.resolve(`${String(checked)}${label}`));
        const returned: void = voidify(handler)(true, "x");
        expect(returned).toBeUndefined();
        expect(handler).toHaveBeenCalledExactlyOnceWith(true, "x");
    });

    it("still runs an async handler to completion", async () => {
        let ran = false;
        voidify(() =>
            Promise.resolve().then(() => {
                ran = true;
            }),
        )();
        await Promise.resolve();
        await Promise.resolve();
        expect(ran).toBe(true);
    });
});
