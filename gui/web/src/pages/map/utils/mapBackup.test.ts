import {describe, expect, it} from "vitest";
import {parseMapBackup} from "./mapBackup.ts";

const square = {points: [{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}]};

const validMap = (extra: Record<string, unknown> = {}) => ({
    working_area: [{name: "Ängen", area: square, obstacles: []}],
    navigation_areas: [],
    ...extra,
});

describe("parseMapBackup", () => {
    it("accepts a backup with at least one area and reports its dock", () => {
        const result = parseMapBackup(JSON.stringify(validMap({dock_x: 1.5, dock_y: -2, dock_heading: 0.3})));

        expect(result.ok).toBe(true);
        if (!result.ok) return;
        expect(result.hasDock).toBe(true);
        expect(result.map.working_area?.[0].name).toBe("Ängen");
    });

    it("keeps a legitimate dock at the origin distinct from a missing dock", () => {
        const atOrigin = parseMapBackup(JSON.stringify(validMap({dock_x: 0, dock_y: 0, dock_heading: 0})));
        const missing = parseMapBackup(JSON.stringify(validMap()));

        expect(atOrigin.ok && atOrigin.hasDock).toBe(true);
        expect(missing.ok && !missing.hasDock).toBe(true);
    });

    it("treats a partial or non-finite dock as missing", () => {
        const partial = parseMapBackup(JSON.stringify(validMap({dock_x: 1})));
        const notANumber = parseMapBackup(JSON.stringify(validMap({dock_x: "1", dock_y: 2, dock_heading: 0})));

        expect(partial.ok && !partial.hasDock).toBe(true);
        expect(notANumber.ok && !notANumber.hasDock).toBe(true);
    });

    it.each([
        ["not JSON", "{"],
        ["null", "null"],
        ["an array", "[]"],
        ["an empty object", "{}"],
        ["empty area lists", JSON.stringify({working_area: [], navigation_areas: []})],
        ["a non-array area list", JSON.stringify({working_area: {}})],
        ["an area without a polygon", JSON.stringify({working_area: [{name: "a"}]})],
        ["a polygon with under 3 points", JSON.stringify({working_area: [{area: {points: [{x: 0, y: 0}]}}]})],
        ["a non-finite coordinate", JSON.stringify({working_area: [{area: {points: [{x: 0, y: 0}, {x: "1", y: 0}, {x: 1, y: 1}]}}]})],
        ["a malformed obstacle", JSON.stringify({working_area: [{area: square, obstacles: [{points: "x"}]}]})],
    ])("rejects %s", (_label, text) => {
        expect(parseMapBackup(text).ok).toBe(false);
    });
});

describe("parseMapBackup obstacle originals", () => {
    const ring = [{x: 4.3, y: 4.3}, {x: 5.7, y: 4.3}, {x: 5.7, y: 5.7}, {x: 4.3, y: 5.7}];
    const record = {shrunk: ring, original: [{x: 4, y: 4}, {x: 6, y: 4}, {x: 6, y: 6}, {x: 4, y: 6}]};

    it("reads them and keeps them out of the map", () => {
        const result = parseMapBackup(JSON.stringify(validMap({obstacle_originals: [record]})));

        expect(result.ok).toBe(true);
        if (!result.ok) return;
        expect(result.obstacleOriginals).toEqual([record]);
        expect(result.map).not.toHaveProperty("obstacle_originals");
    });

    it("tells 'no field' (leave the stored ones) from an empty list (clear them)", () => {
        const absent = parseMapBackup(JSON.stringify(validMap()));
        const empty = parseMapBackup(JSON.stringify(validMap({obstacle_originals: []})));

        expect(absent.ok && absent.obstacleOriginals).toBeNull();
        expect(empty.ok && empty.obstacleOriginals).toEqual([]);
    });

    it("refuses a field that is not a list", () => {
        expect(parseMapBackup(JSON.stringify(validMap({obstacle_originals: {}}))).ok).toBe(false);
    });
});

describe("parseMapBackup ignore lines", () => {
    const line = {name: "Hedge", polyline: {points: [{x: 0, y: 0, z: 0}, {x: 2, y: 0, z: 0}]}, width_m: 0.4, id: 7};

    it("reads the ignore lines, with fresh ids, and keeps them out of the map", () => {
        const result = parseMapBackup(JSON.stringify(validMap({lidar_ignore_corridors: [line]})));

        expect(result.ok).toBe(true);
        if (!result.ok) return;
        expect(result.corridors).toEqual([{name: "Hedge", polyline: line.polyline, width_m: 0.4, id: 0}]);
        expect(result.map).not.toHaveProperty("lidar_ignore_corridors");
    });

    it("distinguishes 'no field' (leave current lines) from an empty list (clear them)", () => {
        const absent = parseMapBackup(JSON.stringify(validMap()));
        const empty = parseMapBackup(JSON.stringify(validMap({lidar_ignore_corridors: []})));

        expect(absent.ok && absent.corridors).toBeNull();
        expect(empty.ok && empty.corridors).toEqual([]);
    });

    it.each([
        ["not a list", {lidar_ignore_corridors: {}}],
        ["no polyline", {lidar_ignore_corridors: [{name: "x", width_m: 0.4}]}],
        ["a single point", {lidar_ignore_corridors: [{...line, polyline: {points: [{x: 0, y: 0}]}}]}],
        ["a non-numeric width", {lidar_ignore_corridors: [{...line, width_m: "0.4"}]}],
    ])("refuses a backup whose ignore lines are broken (%s)", (_label, extra) => {
        expect(parseMapBackup(JSON.stringify(validMap(extra))).ok).toBe(false);
    });

    it("keeps an area's own mowing lines (angle, winding, start point) untouched", () => {
        const lines = {
            has_mow_angle: true, mow_angle_deg: 35,
            has_ring_direction: true, ring_direction: 2,
            has_start_point: true, start_x: 9.5, start_y: 4,
        };
        const result = parseMapBackup(JSON.stringify({
            working_area: [{name: "Voor", id: 7, area: square, obstacles: [], ...lines}],
            navigation_areas: [],
        }));

        expect(result.ok).toBe(true);
        if (!result.ok) return;
        expect(result.map.working_area?.[0]).toMatchObject({id: 7, ...lines});
    });

    it("keeps a legitimate 0 degrees, planner-default winding and (0, 0) start as real overrides", () => {
        const lines = {
            has_mow_angle: true, mow_angle_deg: 0,
            has_ring_direction: true, ring_direction: 0,
            has_start_point: true, start_x: 0, start_y: 0,
        };
        const result = parseMapBackup(JSON.stringify({
            working_area: [{name: "Voor", area: square, obstacles: [], ...lines}],
            navigation_areas: [],
        }));
        expect(result.ok && result.map.working_area?.[0]).toMatchObject(lines);
    });

    it("restores an older backup that has no mowing lines as an area that follows the robot-wide settings", () => {
        const result = parseMapBackup(JSON.stringify(validMap()));
        expect(result.ok).toBe(true);
        if (!result.ok) return;
        const area = result.map.working_area?.[0];
        expect(area?.has_mow_angle).toBeFalsy();
        expect(area?.has_ring_direction).toBeFalsy();
        expect(area?.has_start_point).toBeFalsy();
    });
});
