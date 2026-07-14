/**
 * Robot-state permutations used to drive the mocked GUI.
 *
 * Each scenario is a partial snapshot: `rest` overrides specific REST
 * endpoints and `topics` supplies the payload delivered when the page
 * subscribes to a multiplex topic. Anything omitted falls back to the neutral
 * defaults in mockBackend.ts, so a scenario only states what makes it
 * distinct. Add a scenario here and every parametrized spec picks it up.
 */

export interface Scenario {
    name: string;
    /** Per-endpoint REST overrides, keyed by pathname (no query string). */
    rest?: Record<string, unknown>;
    /** Per-topic multiplex payloads (the decoded ROS message object). */
    topics?: Record<string, unknown>;
    /** When true, the multiplex socket accepts the connection but never sends
     *  a frame — exercises the "offline / stale data" UI. */
    silentSocket?: boolean;
}

// A small square work area (map-frame metres) reused across scenarios.
const WORK_AREA = {
    working_area: [
        {area: {points: [{x: -5, y: -5}, {x: 5, y: -5}, {x: 5, y: 5}, {x: -5, y: 5}]}},
    ],
};

const coverage = (areaIndex: number, mowed: number, total: number) => ({
    coverage: [{area_index: areaIndex, mowed_cells: mowed, total_cells: total}],
});

const hls = (over: Record<string, unknown>) => ({
    state: 0, state_name: "IDLE_DOCKED", is_charging: false, emergency: false,
    latched_emergency: false, battery_percent: 100, current_area: 0, ...over,
});

const gnssFixed = {rtk_mode: 2, fix_type: 4, satellites: 24};
const gnssFloat = {rtk_mode: 1, fix_type: 5, satellites: 18};
const gnssNone = {rtk_mode: 0, fix_type: 0, satellites: 0};

export const SCENARIOS: Scenario[] = [
    {
        name: "idle-docked-full",
        rest: {"/api/diagnostics/snapshot": coverage(0, 0, 4000)},
        topics: {
            highLevelStatus: hls({state: 0, state_name: "IDLE_DOCKED", is_charging: false, battery_percent: 100}),
            status: {mower_motor_rpm: 0, mower_esc_current: 0, rain_detected: false, is_charging: false, firmware_compatible: true, firmware_version: "1.0.0"},
            power: {v_battery: 28.4, charge_current: 0, battery_percent: 100},
            emergency: {active_emergency: false, latched_emergency: false},
            gnssStatus: gnssFixed,
            gps: {position: {x: 48.1, y: 11.5, z: 0}},
            map: WORK_AREA,
        },
    },
    {
        name: "mowing-area2-rtk-fixed",
        rest: {"/api/diagnostics/snapshot": coverage(1, 1800, 4000)},
        topics: {
            highLevelStatus: hls({state: 2, state_name: "MOWING", is_charging: false, battery_percent: 76, current_area: 1}),
            status: {mower_motor_rpm: 3200, mower_esc_current: 4.2, mower_esc_temperature: 38, mower_motor_temperature: 41, rain_detected: false, firmware_compatible: true},
            power: {v_battery: 26.8, charge_current: 0, battery_percent: 76},
            emergency: {active_emergency: false, latched_emergency: false},
            gnssStatus: gnssFixed,
            pose: {pose: {pose: {position: {x: 1.2, y: -0.6, z: 0}, orientation: {x: 0, y: 0, z: 0.38, w: 0.92}}}, twist: {twist: {linear: {x: 0.34}}}},
            map: WORK_AREA,
        },
    },
    {
        name: "low-battery-returning",
        rest: {"/api/diagnostics/snapshot": coverage(0, 3200, 4000)},
        topics: {
            highLevelStatus: hls({state: 2, state_name: "LOW_BATTERY_DOCKING", battery_percent: 18}),
            status: {mower_motor_rpm: 0, mower_esc_current: 0, rain_detected: false, firmware_compatible: true},
            power: {v_battery: 24.3, charge_current: 0, battery_percent: 18},
            emergency: {active_emergency: false, latched_emergency: false},
            gnssStatus: gnssFixed,
            map: WORK_AREA,
        },
    },
    {
        name: "emergency-latched",
        rest: {"/api/diagnostics/snapshot": coverage(0, 500, 4000)},
        topics: {
            highLevelStatus: hls({state: 0, state_name: "EMERGENCY", emergency: true, latched_emergency: true, battery_percent: 64}),
            status: {mower_motor_rpm: 0, mower_esc_current: 0, rain_detected: false, firmware_compatible: true},
            power: {v_battery: 26.0, charge_current: 0, battery_percent: 64},
            emergency: {active_emergency: true, latched_emergency: true},
            gnssStatus: gnssFixed,
            map: WORK_AREA,
        },
    },
    {
        name: "charging-on-dock",
        rest: {"/api/diagnostics/snapshot": coverage(0, 4000, 4000)},
        topics: {
            highLevelStatus: hls({state: 0, state_name: "CHARGING", is_charging: true, battery_percent: 47}),
            status: {mower_motor_rpm: 0, mower_esc_current: 0, rain_detected: false, is_charging: true, firmware_compatible: true},
            power: {v_battery: 25.5, charge_current: 1.8, battery_percent: 47},
            emergency: {active_emergency: false, latched_emergency: false},
            gnssStatus: gnssFixed,
            map: WORK_AREA,
        },
    },
    {
        name: "rain-detected-docking",
        rest: {"/api/diagnostics/snapshot": coverage(0, 900, 4000)},
        topics: {
            highLevelStatus: hls({state: 2, state_name: "RAIN_DETECTED_DOCKING", battery_percent: 71}),
            status: {mower_motor_rpm: 0, mower_esc_current: 0, rain_detected: true, firmware_compatible: true},
            power: {v_battery: 26.6, charge_current: 0, battery_percent: 71},
            emergency: {active_emergency: false, latched_emergency: false},
            gnssStatus: gnssFloat,
            map: WORK_AREA,
        },
    },
    {
        name: "no-gps-float",
        rest: {"/api/diagnostics/snapshot": coverage(0, 0, 4000)},
        topics: {
            highLevelStatus: hls({state: 0, state_name: "IDLE_DOCKED", battery_percent: 88}),
            status: {mower_motor_rpm: 0, mower_esc_current: 0, rain_detected: false, firmware_compatible: true},
            power: {v_battery: 27.5, charge_current: 0, battery_percent: 88},
            emergency: {active_emergency: false, latched_emergency: false},
            gnssStatus: gnssNone,
        },
    },
    {
        name: "offline-stale",
        silentSocket: true,
    },
];
