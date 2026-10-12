import {describe, expect, it} from "vitest";
import {flashPercent, parseFlashStage} from "./flashStage.ts";

describe("parseFlashStage", () => {
    it("accepts the backend's FlashStageEvent shape", () => {
        expect(parseFlashStage('{"stages":["manifest","flash"],"current":1}')).toEqual({
            stages: ["manifest", "flash"],
            current: 1,
        });
    });

    it.each([
        ["not json", "garbage"],
        ["index past the plan", '{"stages":["flash"],"current":1}'],
        ["negative index", '{"stages":["flash"],"current":-1}'],
        ["non-string stage", '{"stages":[1],"current":0}'],
        ["missing stages", '{"current":0}'],
        ["null", "null"],
    ])("rejects %s", (_name, data) => {
        expect(parseFlashStage(data)).toBeNull();
    });
});

describe("flashPercent", () => {
    it("advances by whole stages and completes on done", () => {
        const stage = {stages: ["a", "b", "c", "d"], current: 0};
        expect(flashPercent(null, false)).toBe(0);
        expect(flashPercent(stage, false)).toBe(0);
        expect(flashPercent({...stage, current: 1}, false)).toBe(25);
        expect(flashPercent({...stage, current: 3}, false)).toBe(75);
        expect(flashPercent({...stage, current: 3}, true)).toBe(100);
        expect(flashPercent(null, true)).toBe(100);
    });
});
