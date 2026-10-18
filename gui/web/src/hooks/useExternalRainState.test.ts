import {describe, expect, it} from "vitest";
import {parseExternalRainState} from "./useExternalRainState.ts";

const wrap = (obj: unknown) => ({data: JSON.stringify(obj)});

describe("parseExternalRainState", () => {
    it("reads the JSON out of a std_msgs/String", () => {
        const state = parseExternalRainState(wrap({
            state: "wet", enabled: true, received: true, wet: true, active: true, onboard_wet: false,
            age_s: 312, expires_in_s: 6888, max_age_s: 7200,
        }));
        expect(state?.state).toBe("wet");
        expect(state?.active).toBe(true);
        expect(state?.expires_in_s).toBe(6888);
    });

    it("keeps nulls for ages that do not apply", () => {
        const state = parseExternalRainState(wrap({
            state: "no_data", enabled: true, received: false, wet: false, active: false, onboard_wet: false,
            age_s: null, expires_in_s: null, max_age_s: 0,
        }));
        expect(state?.age_s).toBeNull();
        expect(state?.expires_in_s).toBeNull();
    });

    it("ignores anything that is not a usable state, so the last good one stays", () => {
        expect(parseExternalRainState(undefined)).toBeUndefined();
        expect(parseExternalRainState(null)).toBeUndefined();
        expect(parseExternalRainState({data: ""})).toBeUndefined();
        expect(parseExternalRainState({data: "not json"})).toBeUndefined();
        expect(parseExternalRainState({data: "{}"})).toBeUndefined();
        expect(parseExternalRainState({data: 5})).toBeUndefined();
    });
});
