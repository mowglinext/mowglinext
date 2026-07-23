import {describe, expect, test} from "vitest";
import {groupAlertsByComponent} from "./diagnosticsAlerts";

describe("groupAlertsByComponent", () => {
    test("groups alerts by the component before the first slash", () => {
        const groups = groupAlertsByComponent([
            {level: 1, name: "universal_gnss_ntrip/summary"},
            {level: 1, name: "universal_gnss_ntrip/rtcm.1230_not_valid"},
            {level: 1, name: "universal_gnss/summary"},
        ]);
        expect(groups.map(g => g.component)).toEqual(["universal_gnss_ntrip", "universal_gnss"]);
        expect(groups[0].items).toHaveLength(2);
    });

    test("keeps the worst level per group and sorts error-first", () => {
        const groups = groupAlertsByComponent([
            {level: 1, name: "a/warn"},
            {level: 3, name: "b/stale"},
            {level: 2, name: "b/error"},
        ]);
        expect(groups[0].component).toBe("b");
        expect(groups[0].worstLevel).toBe(2);
        expect(groups[1].worstLevel).toBe(1);
    });

    test("stale (3) ranks below warn (1)", () => {
        const groups = groupAlertsByComponent([
            {level: 3, name: "stale_comp/x"},
            {level: 1, name: "warn_comp/y"},
        ]);
        expect(groups.map(g => g.component)).toEqual(["warn_comp", "stale_comp"]);
    });

    test("handles missing names", () => {
        const groups = groupAlertsByComponent([{level: 2}]);
        expect(groups).toHaveLength(1);
        expect(groups[0].component).toBe("unknown");
    });

    test("empty input yields no groups", () => {
        expect(groupAlertsByComponent([])).toEqual([]);
    });
});
